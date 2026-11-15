"""Send the local BGR HUD to MediaMTX over RTSP."""
from time import monotonic
from urllib.parse import urlsplit

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class HudStreamer(Node):
    def __init__(self):
        super().__init__('hud_streamer')
        defaults = {
            'url': '', 'input_topic': '/pl/monitor/image', 'width': 640, 'height': 360,
            'fps': 30, 'bitrate_kbps': 700, 'rtsp_transport': 'udp',
        }
        cfg = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.url = cfg['url']
        endpoint = urlsplit(self.url)
        self.width, self.height, self.fps = cfg['width'], cfg['height'], cfg['fps']
        self.bitrate, self.transport = cfg['bitrate_kbps'], cfg['rtsp_transport']
        if endpoint.scheme not in ('rtsp', 'rtsps') or not endpoint.hostname or not endpoint.path.strip('/'):
            raise ValueError('Set url to a MediaMTX RTSP URL including the stream path')
        if (min(self.width, self.height) < 2 or self.width % 2 or self.height % 2
                or not 1 <= self.fps <= 60 or self.bitrate <= 0
                or self.transport not in ('udp', 'tcp')):
            raise ValueError('Use even dimensions, 1–60 fps, positive bitrate and UDP or TCP')
        Gst.init(None)
        if Gst.version()[:2] < (1, 20):
            raise RuntimeError('The HUD streamer requires GStreamer 1.20 or newer')
        for name in ('appsrc', 'videoconvert', 'videoscale', 'x264enc', 'h264parse', 'rtph264pay', 'rtspclientsink'):
            if Gst.ElementFactory.find(name) is None:
                raise RuntimeError(f'Missing GStreamer element: {name}')
        self.pipeline = None
        self.source = None
        self.input_size = None
        self.frame = None
        self.retry_at = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, cfg['input_topic'], self.receive, qos)
        self.create_timer(1.0 / self.fps, self.tick)

    def start_pipeline(self):
        # Keep encoded video compatible with WebRTC decoders and avoid frame reordering.
        self.pipeline = Gst.parse_launch(
            'appsrc name=source is-live=true format=time do-timestamp=true '
            'block=false max-bytes=0 max-buffers=1 leaky-type=downstream ! '
            'videoconvert ! videoscale add-borders=true ! '
            f'video/x-raw,format=I420,width={self.width},height={self.height} ! '
            f'x264enc tune=zerolatency speed-preset=ultrafast bitrate={self.bitrate} '
            f'key-int-max={max(1, self.fps // 2)} bframes=0 threads=2 vbv-buf-capacity=100 ! '
            'video/x-h264,profile=constrained-baseline ! h264parse config-interval=-1 ! '
            f'rtspclientsink name=output protocols={self.transport} latency=0 rtx-time=0 tcp-timeout=2000000'
        )
        self.source = self.pipeline.get_by_name('source')
        self.pipeline.get_by_name('output').set_property('location', self.url)
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.retry('could not start GStreamer')

    def close_pipeline(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = self.source = self.input_size = None

    def retry(self, reason):
        self.get_logger().warning(f'Stream interrupted ({reason}); retrying in 1 s')
        self.close_pipeline()
        self.retry_at = monotonic() + 1.0

    def receive(self, msg):
        if (msg.encoding != 'bgr8' or min(msg.width, msg.height) <= 0
                or msg.step < msg.width * 3 or len(msg.data) != msg.step * msg.height):
            self.get_logger().warning('Skipping invalid BGR HUD frame', throttle_duration_sec=5)
            return
        self.frame = msg

    def tick(self):
        if self.pipeline is not None:
            event = self.pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if event is not None:
                reason = event.src.get_name() if event.src else 'GStreamer'
                self.retry(reason)
        msg, self.frame = self.frame, None
        if msg is None or monotonic() < self.retry_at:
            return
        if self.pipeline is None:
            self.start_pipeline()
            if self.pipeline is None:
                return
        size = (msg.width, msg.height)
        if size != self.input_size:
            self.source.set_property('caps', Gst.Caps.from_string(
                f'video/x-raw,format=BGR,width={msg.width},height={msg.height},framerate={self.fps}/1'))
            self.input_size = size
        row_bytes = msg.width * 3
        stride = (row_bytes + 3) & ~3
        if msg.step == stride:
            payload = bytes(msg.data)
        else:
            payload = bytearray(stride * msg.height)
            for row in range(msg.height):
                payload[row*stride:row*stride+row_bytes] = msg.data[row*msg.step:row*msg.step+row_bytes].tobytes()
        buf = Gst.Buffer.new_allocate(None, len(payload), None)
        buf.fill(0, payload)
        buf.duration = Gst.SECOND // self.fps
        if self.source.emit('push-buffer', buf) != Gst.FlowReturn.OK:
            self.retry('could not queue a HUD frame')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HudStreamer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close_pipeline()
            node.destroy_node()
        rclpy.try_shutdown()
