#!/usr/bin/env python3
"""Demand-driven JPEG transport preserving the original camera header."""
from array import array
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from pl_detection.aruco_tracker import to_bgr


class ImageRelay(Node):
    def __init__(self, name='image_relay', **kwargs):
        super().__init__(name, **kwargs)
        defaults = {'direction': 'encode', 'input_topic': '/hitl/pc/camera/image_raw',
                    'output_topic': '/camera/image_raw/compressed', 'fps': 30.0, 'jpeg_quality': 90}
        cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.encode = cfg['direction'] == 'encode'
        self.fps, self.quality = float(cfg['fps']), int(cfg['jpeg_quality'])
        if cfg['direction'] not in ('encode', 'decode') or not math.isfinite(self.fps) or self.fps <= 0 or not 1 <= self.quality <= 100:
            raise ValueError('Invalid image relay direction, fps or JPEG quality')
        self.input_topic = cfg['input_topic']
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(CompressedImage if self.encode else Image, cfg['output_topic'], self.qos)
        self.subscription = None
        self.last_frame = -math.inf
        self.create_timer(.1, self.update_subscription)

    def update_subscription(self):
        if self.publisher.get_subscription_count() and self.subscription is None:
            self.subscription = self.create_subscription(
                Image if self.encode else CompressedImage, self.input_topic, self.convert, self.qos)
        elif not self.publisher.get_subscription_count() and self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

    def convert(self, msg):
        now = time.monotonic()
        if not self.publisher.get_subscription_count() or now-self.last_frame < 1/self.fps:
            return
        try:
            if self.encode:
                image = to_bgr(msg)
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if not ok:
                    return
                output = CompressedImage(header=msg.header, format='bgr8; jpeg compressed bgr8')
                output.data = array('B', encoded.tobytes())
            else:
                image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError('Invalid compressed image')
                output = Image(header=msg.header, height=image.shape[0], width=image.shape[1],
                               encoding='bgr8', step=image.shape[1]*3, data=array('B', image.tobytes()))
            self.publisher.publish(output)
            self.last_frame = now
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5)


def main():
    rclpy.init()
    node = ImageRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
