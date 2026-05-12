#!/usr/bin/env python3
"""
ArUco 마커 검출 노드

하방 카메라 영상에서 ArUco 마커(DICT_4X4_50, ID 0)를 검출하고
solvePnP로 카메라 프레임 기준 3D 포즈를 추정한다.
"""

import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

from pl_msgs.msg import MarkerDetection


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('aruco_id', 0)
        self.declare_parameter('dictionary', 0)   # DICT_4X4_50
        self.declare_parameter('marker_size', 0.5)
        self.declare_parameter('publish_debug_image', True)

        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sub_img = self.create_subscription(
            Image, '/pl/image_raw', self._image_cb, sensor_qos)
        self._sub_info = self.create_subscription(
            CameraInfo, '/pl/camera_info', self._camera_info_cb, sensor_qos)

        self._pub_detection = self.create_publisher(
            MarkerDetection, '/pl/aruco_detection', 10)
        self._pub_debug = self.create_publisher(
            Image, '/pl/aruco_debug_image', 1)

        dict_id = self.get_parameter('dictionary').value
        self._aruco_dict = aruco.getPredefinedDictionary(dict_id)
        self._aruco_params = aruco.DetectorParameters()
        self._detector = aruco.ArucoDetector(self._aruco_dict, self._aruco_params)

        self.get_logger().info('ArUco 검출 노드 시작')

    def _camera_info_cb(self, msg: CameraInfo):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _image_cb(self, msg: Image):
        if self._camera_matrix is None:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 실패: {e}')
            return

        detection, debug_frame = self._detect(frame, msg.header)
        self._pub_detection.publish(detection)

        if self.get_parameter('publish_debug_image').value:
            self._pub_debug.publish(
                self._bridge.cv2_to_imgmsg(debug_frame, 'bgr8'))

    def _detect(self, frame: np.ndarray, header):
        msg = MarkerDetection()
        msg.header = header
        msg.detector_type = 'aruco'
        msg.detected = False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        target_id = self.get_parameter('aruco_id').value
        debug_frame = frame.copy()

        if ids is not None:
            aruco.drawDetectedMarkers(debug_frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id != target_id:
                    continue

                marker_size = self.get_parameter('marker_size').value
                half = marker_size / 2.0
                obj_pts = np.array([
                    [-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0],
                ], dtype=np.float64)

                img_pts = corners[i][0].astype(np.float64)
                success, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, self._camera_matrix, self._dist_coeffs)

                if not success:
                    continue

                # 중심 픽셀 좌표
                cx_px = float(np.mean(img_pts[:, 0]))
                cy_px = float(np.mean(img_pts[:, 1]))

                # 회전 행렬 → 쿼터니언
                rot_mat, _ = cv2.Rodrigues(rvec)
                quat = Rotation.from_matrix(rot_mat).as_quat()  # [x,y,z,w]

                pose_stamped = PoseStamped()
                pose_stamped.header = header
                pose_stamped.pose.position = Point(
                    x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
                pose_stamped.pose.orientation = Quaternion(
                    x=quat[0], y=quat[1], z=quat[2], w=quat[3])

                msg.detected = True
                msg.image_x = cx_px
                msg.image_y = cy_px
                msg.image_width = float(np.linalg.norm(img_pts[0] - img_pts[1]))
                msg.image_height = float(np.linalg.norm(img_pts[1] - img_pts[2]))
                msg.confidence = 1.0
                msg.estimated_distance = float(tvec[2])
                msg.pose = pose_stamped

                # 디버그: 축 그리기
                cv2.drawFrameAxes(
                    debug_frame, self._camera_matrix, self._dist_coeffs,
                    rvec, tvec, marker_size * 0.5)
                cv2.putText(
                    debug_frame,
                    f'ArUco d={tvec[2]:.2f}m',
                    (int(cx_px) - 60, int(cy_px) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                )
                break  # 첫 번째 타겟 마커만 사용

        return msg, debug_frame


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
