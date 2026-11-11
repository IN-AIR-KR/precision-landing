"""ROS camera conversion, ArUco detection and pose publication."""
from array import array
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import math
import json
import time
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


def to_bgr(msg):
    channels = {'mono8': 1, 'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}
    if msg.encoding not in channels:
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')
    c = channels[msg.encoding]
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * c:
        raise ValueError('Invalid image dimensions or row stride')
    if len(msg.data) != msg.step * msg.height:
        raise ValueError('Image data length does not match height * step')
    # All supported channels are one byte, so endianness has no effect.
    view = np.ndarray((msg.height, msg.width, c), dtype=np.uint8,
                      buffer=msg.data, strides=(msg.step, c, 1))
    codes = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
             'bgra8': cv2.COLOR_BGRA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
    return view.copy() if msg.encoding == 'bgr8' else cv2.cvtColor(view, codes[msg.encoding])


def quaternion_xyzw(r):
    # Preserve cv::Quatd::createFromRotMat's branch/sign convention, including
    # the negative representative used for trace-positive and X-major cases.
    trace = np.trace(r)
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        q = [(r[1, 2]-r[2, 1])/s, (r[2, 0]-r[0, 2])/s,
             (r[0, 1]-r[1, 0])/s, -.25*s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1+r[0, 0]-r[1, 1]-r[2, 2])*2
        q = [-.25*s, -(r[1, 0]+r[0, 1])/s, -(r[0, 2]+r[2, 0])/s,
             (r[1, 2]-r[2, 1])/s]
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1-r[0, 0]+r[1, 1]-r[2, 2])*2
        q = [(r[0, 1]+r[1, 0])/s, .25*s, (r[1, 2]+r[2, 1])/s,
             (r[0, 2]-r[2, 0])/s]
    else:
        s = math.sqrt(1-r[0, 0]-r[1, 1]+r[2, 2])*2
        q = [(r[0, 2]+r[2, 0])/s, (r[1, 2]+r[2, 1])/s, .25*s,
             -(r[0, 1]-r[1, 0])/s]
    return np.asarray(q)/np.linalg.norm(q)


class Detector:
    def __init__(self, dictionary, marker_id, marker_size):
        if not math.isfinite(marker_size) or marker_size <= 0:
            raise ValueError('marker_size must be finite and positive')
        self.marker_id, self.marker_size = marker_id, marker_size
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(dictionary), cv2.aruco.DetectorParameters())
        h = np.float32(marker_size / 2)
        self.objects = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)

    def process(self, image, camera, distortion, draw=True):
        corners, ids, _ = self.detector.detectMarkers(image)
        self.detected = ids is not None and self.marker_id in ids.flatten()
        if draw:
            if ids is not None:
                stroke = max(3, round(image.shape[1] / 256))
                for corner, marker_id in zip(corners, ids.flatten()):
                    points = np.rint(corner.reshape(4, 2)).astype(np.int32)
                    cv2.polylines(image, [points], True, (0, 255, 0), stroke, cv2.LINE_AA)
                    label = f'ID {marker_id}'
                    origin = tuple(points[0] + [0, -12])
                    cv2.putText(image, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
                                .8, (0, 0, 0), 4, cv2.LINE_AA)
                    cv2.putText(image, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
                                .8, (0, 255, 0), 2, cv2.LINE_AA)
                    if marker_id == self.marker_id:
                        # Use corners from this exact frame, never a later pose/status sample.
                        center = tuple(np.rint(corner.reshape(4, 2).mean(axis=0)).astype(int))
                        origin = (image.shape[1] // 2, image.shape[0] // 2)
                        if center != origin:
                            cv2.arrowedLine(image, origin, center, (0, 255, 255),
                                            stroke + 1, cv2.LINE_AA, tipLength=.15)
        if camera is None or distortion is None or not distortion.size or ids is None:
            return None
        for corner, marker_id in zip(corners, ids.flatten()):
            if marker_id != self.marker_id:
                continue
            undistorted = cv2.undistortPoints(corner, camera, distortion, P=camera)
            ok, rvec, tvec = cv2.solvePnP(self.objects, undistorted, camera, None,
                                        flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return None
            position = tvec.flatten()
            return position, quaternion_xyzw(cv2.Rodrigues(rvec)[0])
        return None


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')
        defaults = {'aruco_id': 0, 'dictionary': 2, 'marker_size': 0.5,
                    'optical_frame': 'camera_frame', 'input_transport': 'raw', 'debug_fps': 15.0, 'jpeg_quality': 80}
        values = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.frame = values['optical_frame']
        self.debug_fps = float(values['debug_fps'])
        self.jpeg_quality = int(values['jpeg_quality'])
        if self.debug_fps <= 0 or not 1 <= self.jpeg_quality <= 100:
            raise ValueError('debug_fps must be positive and jpeg_quality in [1,100]')
        self.last_debug = -math.inf
        self.detector = Detector(values['dictionary'], values['aruco_id'], values['marker_size'])
        self.camera = self.distortion = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        transport = values['input_transport']
        if transport not in ('raw', 'compressed'):
            raise ValueError('input_transport must be raw or compressed')
        self.create_subscription(CompressedImage if transport == 'compressed' else Image,
                                 '/camera/image_raw/compressed' if transport == 'compressed' else '/camera/image_raw',
                                 self.image_callback, qos)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, qos)
        self.image_pub = self.create_publisher(Image, '/image_proc', qos)
        self.compressed_pub = self.create_publisher(CompressedImage, '/image_proc/compressed', qos)
        self.status_pub = self.create_publisher(String, '/pl/detection/status', qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', qos)

    def info_callback(self, msg):
        self.camera = np.asarray(msg.k, dtype=np.float64).reshape(3, 3).copy()
        self.distortion = np.asarray(msg.d, dtype=np.float64).copy()
        if self.camera[0, 0] != 0:
            self.get_logger().info('Updated camera intrinsics from camera_info topic.')
            self.destroy_subscription(self.info_sub)
            self.info_sub = None

    def image_callback(self, msg):
        try:
            if isinstance(msg, CompressedImage):
                image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError('Invalid compressed camera image')
            else:
                image = to_bgr(msg)
            now = time.monotonic()
            raw = self.image_pub.get_subscription_count() > 0
            compressed = self.compressed_pub.get_subscription_count() > 0
            draw = (raw or compressed) and now - self.last_debug >= 1 / self.debug_fps
            result = self.detector.process(image, self.camera, self.distortion, draw=draw)
            if self.status_pub.get_subscription_count():
                self.status_pub.publish(String(data=json.dumps({
                    'stamp_ns': msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec,
                    'detected': bool(self.detector.detected), 'marker_id': self.detector.marker_id,
                    'pose_valid': result is not None,
                    'position_m': result[0].tolist() if result is not None else None,
                    'distance_m': float(np.linalg.norm(result[0])) if result is not None else None})))
            if result is not None:
                p, q = result
                pose = PoseStamped()
                pose.header.stamp = msg.header.stamp
                pose.header.frame_id = self.frame or msg.header.frame_id
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, p)
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
                self.pose_pub.publish(pose)
            if draw:
                self.last_debug = now
                if raw:
                    output = Image()
                    output.header = msg.header
                    output.height, output.width = image.shape[:2]
                    output.encoding, output.is_bigendian = 'bgr8', 0
                    output.step = output.width * 3
                    output.data = array('B', image.tobytes())
                    self.image_pub.publish(output)
                if compressed:
                    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                    if ok:
                        output = CompressedImage(header=msg.header, format='bgr8; jpeg compressed bgr8')
                        output.data = array('B', encoded.tobytes())
                        self.compressed_pub.publish(output)
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
