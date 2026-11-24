"""Publish IMX708 RAW capture as the project's calibrated ROS camera input."""
from array import array
from queue import Empty, SimpleQueue
from threading import Event, Thread

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

from pl_camera.csi_camera import load_camera_info
from pl_camera.imx708_capture import capture_imx708


class Imx708Camera(Node):
    def __init__(self):
        super().__init__('csi_camera')
        defaults = {
            'device': '/dev/video0', 'sensor_mode': 1,
            'width': 1536, 'height': 864, 'fps': 60.0,
            'output_width': 640, 'output_height': 360,
            'calibration_json': '', 'encoding': 'bgr8',
            'frame_id': 'camera_optical_frame',
        }
        cfg = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        if (cfg['sensor_mode'], cfg['width'], cfg['height'], cfg['fps']) != (1, 1536, 864, 60.0):
            raise ValueError('IMX708 ROS capture requires mode 1 at 1536x864 and 60 fps')
        width, height = cfg['output_width'], cfg['output_height']
        if not (0 < width <= 1536 and 0 < height <= 864 and width * 864 == height * 1536):
            raise ValueError('Output dimensions must fit the sensor and preserve its aspect ratio')
        if cfg['encoding'] not in ('bgr8', 'mono8'):
            raise ValueError('Use bgr8 or mono8 encoding')
        self.encoding = cfg['encoding']
        self.frame_id = cfg['frame_id']
        self.info = load_camera_info(
            '', width, height, self.frame_id, self.get_logger(),
            calibration_json=cfg['calibration_json'], capture_size=(1536, 864))
        self.image_pub = self.create_publisher(Image, '/pl/camera/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/pl/camera/camera_info', qos_profile_sensor_data)
        self.frames, self.errors, self.stopped = SimpleQueue(), SimpleQueue(), Event()
        self.thread = Thread(
            target=capture_imx708,
            args=(cfg['device'], self.frames, self.errors, self.stopped, 1, (width, height)),
            daemon=True)
        self.thread.start()
        self.create_timer(1 / 120, self.publish_frame)
        self.get_logger().info(f'IMX708 1536x864 @ 60 fps -> {width}x{height}, automatic exposure enabled')

    def publish_frame(self):
        if not self.errors.empty():
            raise RuntimeError(f'IMX708 capture failed: {self.errors.get()}')
        if not self.thread.is_alive() and not self.stopped.is_set():
            raise RuntimeError('IMX708 capture thread exited')
        try:
            frame = self.frames.get_nowait()
        except Empty:
            return
        if self.encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg = Image(
            height=frame.shape[0], width=frame.shape[1], encoding=self.encoding,
            step=frame.shape[1] * (1 if self.encoding == 'mono8' else 3),
            data=array('B', frame.tobytes()))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.info.header = msg.header
        self.image_pub.publish(msg)
        self.info_pub.publish(self.info)

    def stop(self):
        self.stopped.set()
        self.thread.join(timeout=6)


def main():
    rclpy.init()
    node = None
    try:
        node = Imx708Camera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.try_shutdown()
