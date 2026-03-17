import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped


class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')

        self.declare_parameter('aruco_id', 0)
        self.declare_parameter('dictionary', 2)  # DICT_4X4_250
        self.declare_parameter('marker_size', 0.5)

        self._param_aruco_id = self.get_parameter('aruco_id').value
        self._param_dictionary = self.get_parameter('dictionary').value
        self._param_marker_size = self.get_parameter('marker_size').value

        detector_params = aruco.DetectorParameters()
        dictionary = aruco.getPredefinedDictionary(self._param_dictionary)
        self._detector = aruco.ArucoDetector(dictionary, detector_params)

        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._image_sub = self.create_subscription(
            Image,
            '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image',
            self._image_callback,
            qos,
        )

        self._cam_info_sub = self.create_subscription(
            CameraInfo,
            '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info',
            self._camera_info_callback,
            qos,
        )

        self._image_pub = self.create_publisher(Image, '/image_proc', qos)
        self._target_pose_pub = self.create_publisher(PoseStamped, '/target_pose', qos)

    def _image_callback(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        corners, ids, _ = self._detector.detectMarkers(cv_image)

        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)

        if self._camera_matrix is None or self._dist_coeffs is None:
            self.get_logger().error('Missing camera calibration')
        elif ids is not None:
            undistorted_corners = [
                cv2.undistortPoints(c, self._camera_matrix, self._dist_coeffs, None, self._camera_matrix)
                for c in corners
            ]

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id != self._param_aruco_id:
                    continue

                half = self._param_marker_size / 2.0
                obj_pts = np.array([
                    [-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0],
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_pts, undistorted_corners[i], self._camera_matrix, None
                )
                if not success:
                    continue

                cv2.drawFrameAxes(
                    cv_image, self._camera_matrix, None, rvec, tvec, self._param_marker_size
                )

                rot_mat, _ = cv2.Rodrigues(rvec)
                quat_xyzw = Rotation.from_matrix(rot_mat).as_quat()  # [x, y, z, w]

                tx, ty, tz = tvec.flatten()

                pose_msg = PoseStamped()
                pose_msg.header.stamp = msg.header.stamp
                pose_msg.header.frame_id = 'camera_frame'
                pose_msg.pose.position.x = float(tx)
                pose_msg.pose.position.y = float(ty)
                pose_msg.pose.position.z = float(tz)
                pose_msg.pose.orientation.x = float(quat_xyzw[0])
                pose_msg.pose.orientation.y = float(quat_xyzw[1])
                pose_msg.pose.orientation.z = float(quat_xyzw[2])
                pose_msg.pose.orientation.w = float(quat_xyzw[3])
                self._target_pose_pub.publish(pose_msg)

                self._annotate_image(cv_image, tx, ty, tz)
                break  # Only publish the first matching marker

        out_msg = self._bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        out_msg.header = msg.header
        self._image_pub.publish(out_msg)

    def _camera_info_callback(self, msg):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)

        self.get_logger().info(
            f'Camera matrix updated:\n'
            f'[{K[0,0]:.6f}, {K[0,1]:.6f}, {K[0,2]:.6f}]\n'
            f'[{K[1,0]:.6f}, {K[1,1]:.6f}, {K[1,2]:.6f}]\n'
            f'[{K[2,0]:.6f}, {K[2,1]:.6f}, {K[2,2]:.6f}]'
        )
        self.get_logger().info(
            f'Camera Matrix: fx={K[0,0]:.6f}, fy={K[1,1]:.6f}, '
            f'cx={K[0,2]:.6f}, cy={K[1,2]:.6f}'
        )

        if K[0, 0] == 0:
            self.get_logger().error('Focal length is zero after update!')
            return

        self._camera_matrix = K
        self._dist_coeffs = D
        self.get_logger().info('Updated camera intrinsics from camera_info topic.')
        self.get_logger().info('Unsubscribing from camera info topic')
        self.destroy_subscription(self._cam_info_sub)

    def _annotate_image(self, image, tx, ty, tz):
        text = f'X: {tx:.2f} Y: {ty:.2f} Z: {tz:.2f}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 1.0
        thickness = 2
        (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
        h, w = image.shape[:2]
        org = (w - tw - 10, h - 10)
        cv2.putText(image, text, org, font, scale, (0, 255, 255), thickness, cv2.LINE_8)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
