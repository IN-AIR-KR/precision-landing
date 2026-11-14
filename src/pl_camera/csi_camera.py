#!/usr/bin/env python3
"""IMX219 CSI camera publisher for Jetson (Argus ISP via GStreamer).

Publishes `sensor_msgs/Image` (bgr8 or mono8) and `sensor_msgs/CameraInfo`
so `pl_detection/aruco_tracker` can consume the hardware camera the same way
it consumes the Gazebo bridge.

Capture path: `nvarguscamerasrc -> nvvidconv -> videoconvert -> appsink`.
Argus runs the Tegra ISP, so exposure and white balance are automatic and the
frames arrive as ready-to-use BGR. See docs/csi_camera_ov5647.md for the
sensor bring-up that this node sits on top of.

GStreamer is driven through PyGObject (`gi`) rather than `cv2.VideoCapture`,
because the project's pinned OpenCV wheel in `.deps/python` is built without
GStreamer support (`GStreamer: NO`). `gi` lives in the system dist-packages
and stays importable under `scripts/env.sh`.

Requires the Argus GStreamer plugin:

    gst-inspect-1.0 nvarguscamerasrc     # nvidia-l4t-gstreamer

Only one Argus capture session may exist at a time, so stop any preview
(`scripts/camera_preview/imx219_preview.py`) before starting this node.
"""
from array import array
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GstVideo  # noqa: E402  (gi needs require_version first)

# IMX219 sensor modes exposed by the nv_imx219 driver, as (width, height, fps).
# Picking a listed pair keeps Argus from renegotiating to a different mode.
SENSOR_MODES = ((3280, 2464, 21), (3280, 1848, 28), (1920, 1080, 30),
                (1640, 1232, 30), (1280, 720, 60))


def video_info(caps):
    """GstVideo.VideoInfo from caps, across the two PyGObject spellings."""
    factory = getattr(GstVideo.VideoInfo, 'new_from_caps', None)
    if factory is not None:
        return factory(caps)
    info = GstVideo.VideoInfo()
    info.from_caps(caps)
    return info


def load_camera_info(path, width, height, frame_id, logger, *, calibration_json=''):
    """Read calibration from a file or the real flight profile into a CameraInfo message.

    Returns an uncalibrated message (k and d zeroed) when no path is given or
    the file cannot be read. aruco_tracker keeps its camera_info subscription
    open until k[0][0] is non-zero, so an uncalibrated message is harmless:
    detection simply waits for real intrinsics instead of using wrong ones.
    """
    info = CameraInfo(width=width, height=height, distortion_model='plumb_bob')
    info.header.frame_id = frame_id
    if not path and not calibration_json:
        logger.warn('No camera_info_url set: publishing uncalibrated CameraInfo. '
                    'Run camera_calibration and point camera_info_url at the result '
                    'before trusting target poses.')
        return info
    try:
        if calibration_json:
            import json
            data = json.loads(calibration_json)
        else:
            import yaml
            with open(path) as handle:
                data = yaml.safe_load(handle) or {}
        k = list(data['camera_matrix']['data'])
        d = list(data['distortion_coefficients']['data'])
        if len(k) != 9:
            raise ValueError('camera_matrix must hold 9 values')
        file_w = int(data.get('image_width', width))
        file_h = int(data.get('image_height', height))
        if calibration_json:
            from math import isfinite
            if (file_w, file_h) != (width, height):
                raise ValueError(f'Expected {width}x{height} calibration, got {file_w}x{file_h}')
            if not all(isfinite(float(v)) for v in k + d) or float(k[0]) <= 0 or float(k[4]) <= 0:
                raise ValueError('Calibration must have finite coefficients and positive focal lengths')
        if (file_w, file_h) != (width, height):
            # Intrinsics scale linearly with resolution for the same field of view.
            sx, sy = width / file_w, height / file_h
            k = [k[0]*sx, k[1]*sx, k[2]*sx,
                 k[3]*sy, k[4]*sy, k[5]*sy,
                 k[6], k[7], k[8]]
            logger.warn(f'Calibration is for {file_w}x{file_h} but capturing '
                        f'{width}x{height}; intrinsics scaled by '
                        f'({sx:.4f}, {sy:.4f}). Re-calibrate at the capture '
                        'resolution if the fields of view differ.')
        info.k = [float(v) for v in k]
        info.d = [float(v) for v in d]
        info.distortion_model = str(data.get('distortion_model', 'plumb_bob'))
        info.r = [float(v) for v in data.get('rectification_matrix', {}).get(
            'data', [1., 0., 0., 0., 1., 0., 0., 0., 1.])]
        projection = data.get('projection_matrix', {}).get('data')
        info.p = ([float(v) for v in projection] if projection else
                  [info.k[0], info.k[1], info.k[2], 0.,
                   info.k[3], info.k[4], info.k[5], 0.,
                   info.k[6], info.k[7], info.k[8], 0.])
        logger.info(f'Loaded camera intrinsics from {path or "real flight profile"}')
    except Exception as exc:                      # noqa: BLE001 - report and degrade
        if calibration_json:
            raise ValueError(f'Invalid real flight camera calibration: {exc}') from exc
        logger.error(f'Could not read camera_info_url {path}: {exc}. '
                     'Publishing uncalibrated CameraInfo.')
        info.k = [0.] * 9
        info.d = []
    return info


class CsiCamera(Node):
    def __init__(self, name='csi_camera', **kwargs):
        super().__init__(name, **kwargs)
        defaults = {
            'sensor_id': 0, 'width': 1920, 'height': 1080, 'fps': 30.0,
            'flip_method': 0,            # nvvidconv: 0 none, 2 180deg, 4 horiz, 6 vert
            'encoding': 'bgr8',          # bgr8 or mono8 (mono8 is cheaper for ArUco)
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'frame_id': 'camera_optical_frame',
            'camera_info_url': '',
            'calibration_json': '',
            'wbmode': 1,                 # Argus auto white balance
            'exposure_time_range': '',   # e.g. '5000000 20000000' (ns), '' = Argus auto
            'gain_range': '',            # e.g. '1 10'
        }
        cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}

        self.width, self.height = int(cfg['width']), int(cfg['height'])
        self.fps = float(cfg['fps'])
        self.encoding = str(cfg['encoding'])
        if self.encoding not in ('bgr8', 'mono8'):
            raise ValueError("encoding must be 'bgr8' or 'mono8'")
        if self.width <= 0 or self.height <= 0 or not 0 < self.fps <= 120:
            raise ValueError('Invalid width, height or fps')
        if (self.width, self.height, round(self.fps)) not in SENSOR_MODES:
            self.get_logger().warn(
                f'{self.width}x{self.height}@{self.fps:g} is not an IMX219 sensor mode '
                f'{SENSOR_MODES}. Argus will scale or renegotiate, which costs '
                'frame rate and may change the field of view.')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(Image, str(cfg['image_topic']), qos)
        self.info_pub = self.create_publisher(CameraInfo, str(cfg['camera_info_topic']), qos)
        self.frame_id = str(cfg['frame_id'])
        self.info = load_camera_info(str(cfg['camera_info_url']), self.width,
                                     self.height, self.frame_id, self.get_logger(),
                                     calibration_json=cfg['calibration_json'])

        Gst.init(None)
        self.pipeline = Gst.parse_launch(self.build_pipeline(cfg))
        self.sink = self.pipeline.get_by_name('sink')
        self.running = True
        self.frames = 0
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(
                'Could not start the Argus pipeline. Another process may hold the '
                'camera (only one Argus session is allowed), or nvarguscamerasrc is '
                'missing (install nvidia-l4t-gstreamer).')
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'Capturing {self.width}x{self.height}@{self.fps:g} as {self.encoding} '
            f'-> {cfg["image_topic"]}')

    def build_pipeline(self, cfg):
        src = f'nvarguscamerasrc sensor-id={int(cfg["sensor_id"])} wbmode={int(cfg["wbmode"])}'
        if cfg['exposure_time_range']:
            src += f' exposuretimerange="{cfg["exposure_time_range"]}"'
        if cfg['gain_range']:
            src += f' gainrange="{cfg["gain_range"]}"'
        out = 'GRAY8' if self.encoding == 'mono8' else 'BGR'
        return (
            f'{src} ! '
            f'video/x-raw(memory:NVMM),width={self.width},height={self.height},'
            f'framerate={round(self.fps)}/1,format=NV12 ! '
            f'nvvidconv flip-method={int(cfg["flip_method"])} ! '
            'video/x-raw,format=BGRx ! '
            'videoconvert ! '
            f'video/x-raw,format={out} ! '
            'appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false')

    def capture_loop(self):
        channels = 1 if self.encoding == 'mono8' else 3
        while self.running:
            sample = self.sink.emit('pull-sample')
            if sample is None:                     # pipeline stopped or EOS
                if self.running:
                    self.get_logger().error('Camera stream ended unexpectedly.')
                break
            try:
                self.publish(sample, channels)
            except Exception as exc:               # noqa: BLE001 - keep streaming
                self.get_logger().error(f'Dropping frame: {exc}',
                                        throttle_duration_sec=5.0)

    def publish(self, sample, channels):
        info = video_info(sample.get_caps())
        width, height, stride = info.width, info.height, info.stride[0]
        row = width * channels

        buffer = sample.get_buffer()
        ok, mapped = buffer.map(Gst.MapFlags.READ)
        if not ok:
            raise RuntimeError('Could not map the GStreamer buffer')
        try:
            raw = bytes(mapped.data)
        finally:
            buffer.unmap(mapped)
        if len(raw) < stride * height:
            raise RuntimeError(f'Short frame: {len(raw)} bytes for {height}x{stride}')
        # GStreamer pads each row up to a 4-byte boundary; ROS Image wants the
        # stride it is told about, so hand over tightly packed rows.
        payload = (raw[:row * height] if stride == row else
                   b''.join(raw[y * stride:y * stride + row] for y in range(height)))

        stamp = self.get_clock().now().to_msg()
        message = Image(height=height, width=width, encoding=self.encoding,
                        is_bigendian=0, step=row, data=array('B', payload))
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        self.image_pub.publish(message)
        self.info.header.stamp = stamp
        self.info_pub.publish(self.info)
        self.frames += 1

    def report(self):
        self.get_logger().info(f'{self.frames / 5.0:.1f} fps published',
                               throttle_duration_sec=30.0)
        self.frames = 0

    def stop(self):
        self.running = False
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)


def main():
    rclpy.init()
    node = CsiCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
