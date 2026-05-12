#!/usr/bin/env python3
"""
V-마커 검출 노드

하방 카메라 영상에서 V-마커의 원형 테두리를 Hough 원 변환으로 검출한다.
검출 결과는 MarkerDetection 메시지로 발행하며, 디버그 이미지도 선택적으로 발행한다.
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from pl_msgs.msg import MarkerDetection


class VMarkerDetectorNode(Node):
    def __init__(self):
        super().__init__('v_marker_detector_node')

        self.declare_parameter('real_circle_diameter', 3.0)
        self.declare_parameter('hough_dp', 1.2)
        self.declare_parameter('hough_param1', 50)
        self.declare_parameter('hough_param2', 30)
        self.declare_parameter('min_confidence', 0.25)
        self.declare_parameter('publish_debug_image', True)

        self._bridge = CvBridge()
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None

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
            MarkerDetection, '/pl/v_marker_detection', 10)
        self._pub_debug = self.create_publisher(
            Image, '/pl/v_marker_debug_image', 1)

        self.get_logger().info('V-마커 검출 노드 시작')

    def _camera_info_cb(self, msg: CameraInfo):
        if self._fx is None:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]

    def _image_cb(self, msg: Image):
        if self._fx is None:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 실패: {e}')
            return

        detection = self._detect(frame, msg.header)

        self._pub_detection.publish(detection)

        if self.get_parameter('publish_debug_image').value:
            self._pub_debug.publish(
                self._bridge.cv2_to_imgmsg(self._draw_debug(frame, detection), 'bgr8'))

    def _detect(self, frame: np.ndarray, header) -> MarkerDetection:
        msg = MarkerDetection()
        msg.header = header
        msg.detector_type = 'v_marker'
        msg.detected = False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        h, w = gray.shape

        dp = self.get_parameter('hough_dp').value
        p1 = float(self.get_parameter('hough_param1').value)
        p2 = float(self.get_parameter('hough_param2').value)
        min_r = int(h * 0.02)
        max_r = int(h * 0.60)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=dp,
            minDist=h / 4,
            param1=p1,
            param2=p2,
            minRadius=min_r,
            maxRadius=max_r,
        )

        best = self._best_hough_circle(circles, w, h)

        if best is None:
            best = self._contour_fallback(blurred, w, h)

        if best is not None:
            cx_px, cy_px, r_px, conf = best
            min_conf = self.get_parameter('min_confidence').value
            if conf >= min_conf:
                real_d = self.get_parameter('real_circle_diameter').value
                dist = (self._fx * real_d) / (2.0 * r_px) if r_px > 0 else 0.0

                msg.detected = True
                msg.image_x = float(cx_px)
                msg.image_y = float(cy_px)
                msg.image_width = float(2 * r_px)
                msg.image_height = float(2 * r_px)
                msg.confidence = float(conf)
                msg.estimated_distance = float(dist)

        return msg

    def _best_hough_circle(self, circles, w, h):
        """이미지 중심에 가장 가까운 원을 반환."""
        if circles is None:
            return None

        circles = np.round(circles[0, :]).astype(int)
        cx_img, cy_img = w / 2.0, h / 2.0
        best = None
        best_dist = float('inf')

        for (x, y, r) in circles:
            d = math.hypot(x - cx_img, y - cy_img)
            if d < best_dist:
                best_dist = d
                best = (x, y, r)

        if best is None:
            return None

        # 이미지 중심으로부터의 거리를 신뢰도로 환산 (가까울수록 높음)
        max_dist = math.hypot(w, h) / 2.0
        conf = max(0.0, 1.0 - best_dist / max_dist)
        return (best[0], best[1], best[2], conf)

    def _contour_fallback(self, blurred, w, h):
        """Hough 실패 시 이진화 + 윤곽선으로 원 추정."""
        _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 100:
            return None

        (cx, cy), r = cv2.minEnclosingCircle(largest)
        cx_img, cy_img = w / 2.0, h / 2.0
        dist = math.hypot(cx - cx_img, cy - cy_img)
        max_dist = math.hypot(w, h) / 2.0
        conf = max(0.0, 0.6 * (1.0 - dist / max_dist))  # 폴백은 최대 0.6
        return (int(cx), int(cy), int(r), conf)

    def _draw_debug(self, frame: np.ndarray, det: MarkerDetection) -> np.ndarray:
        out = frame.copy()
        if det.detected:
            cx = int(det.image_x)
            cy = int(det.image_y)
            r = int(det.image_width / 2)
            cv2.circle(out, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(out, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(
                out,
                f'V conf={det.confidence:.2f} d={det.estimated_distance:.1f}m',
                (cx - 80, cy - r - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )
        return out


def main(args=None):
    rclpy.init(args=args)
    node = VMarkerDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
