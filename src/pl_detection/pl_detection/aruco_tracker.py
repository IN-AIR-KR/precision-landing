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

    def process(self, image, camera, distortion):
        corners, ids, _ = self.detector.detectMarkers(image)
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
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
            cv2.drawFrameAxes(image, camera, None, rvec, tvec, self.marker_size)
            position = tvec.flatten()
            text = 'X: %.2f Y: %.2f Z: %.2f' % tuple(position)
            (width, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
            cv2.putText(image, text, (image.shape[1]-width-10, image.shape[0]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_8)
            return position, quaternion_xyzw(cv2.Rodrigues(rvec)[0])
        return None


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')
        defaults = {'aruco_id': 0, 'dictionary': 2, 'marker_size': 0.5,
                    'optical_frame': 'camera_frame'}
        values = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.frame = values['optical_frame']
        self.detector = Detector(values['dictionary'], values['aruco_id'], values['marker_size'])
        self.camera = self.distortion = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, qos)
        self.image_pub = self.create_publisher(Image, '/image_proc', qos)
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
            image = to_bgr(msg)
            result = self.detector.process(image, self.camera, self.distortion)
            if result is not None:
                p, q = result
                pose = PoseStamped()
                pose.header.stamp = msg.header.stamp
                pose.header.frame_id = self.frame or msg.header.frame_id
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, p)
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
                self.pose_pub.publish(pose)
            output = Image()
            output.header = msg.header
            output.height, output.width = image.shape[:2]
            output.encoding, output.is_bigendian = 'bgr8', 0
            output.step = output.width * 3
            output.data = array('B', image.tobytes())
            self.image_pub.publish(output)
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
