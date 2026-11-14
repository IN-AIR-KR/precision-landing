#!/usr/bin/env python3
"""Demand-driven camera/control HUD. No GUI or Gazebo dependency at runtime."""
from array import array
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from px4_msgs.msg import BatteryStatus, SensorGps, VehicleLocalPosition
from pl_detection.aruco_tracker import to_bgr

SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def px4_topic(name, cls):
    version = getattr(cls, 'MESSAGE_VERSION', 0)
    return '/fmu/out/' + name + (f'_v{version}' if version else '')


def finite(value):
    return float(value) if math.isfinite(value) else None


def body_relative_position(detection, extrinsic):
    if not detection.get('pose_valid') or detection.get('position_m') is None:
        return None
    position = np.asarray(detection['position_m'], dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        return None
    translation, rotation = extrinsic
    w, xyz = rotation[0], rotation[1:]
    cross = 2 * np.cross(xyz, position)
    return (translation + position + w * cross + np.cross(xyz, cross)).tolist()


def render(image, state):
    """Overlay telemetry directly on the camera frame at its original resolution."""
    canvas = image.copy() if image is not None else np.full((960, 1280, 3), 28, np.uint8)
    h, w = canvas.shape[:2]
    unit = min(w / 1280, h / 960)
    margin = max(12, round(28 * unit))
    thickness = max(1, round(2 * unit))
    white, yellow = (255, 255, 255), (0, 235, 255)

    def text(value, x, y, size=.8, color=white, align='left', outline=True):
        scale = size * unit
        width = cv2.getTextSize(value, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0][0]
        if align == 'right':
            x -= width
        elif align == 'center':
            x -= width // 2
        # Black outline remains readable on both sky and the white landing pad.
        strokes = [((0, 0, 0), thickness + max(2, round(3 * unit)))] if outline else []
        for ink, stroke in strokes + [(color, thickness)]:
            cv2.putText(canvas, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, ink, stroke, cv2.LINE_AA)

    def current(key):
        entry = state[key]
        return entry['value'] if entry['health'] == 'ok' else None

    c = current('control')
    phase = (c['state'].upper() if c['active'] else 'INACTIVE') if c else 'CONTROL ' + state['control']['health'].upper()
    # Status colour is shared by the label and the frame border (OpenCV BGR).
    phase_color = (160, 160, 160)
    if c and c['active']:
        if c['state'] == 'FINISHED':
            phase_color = (0, 220, 0)
        elif c['state'] in ('SEARCH', 'ALIGN', 'APPROACH', 'FINAL_APPROACH'):
            phase_color = yellow
    if c is None or (c['active'] and c['target_lost']):
        phase_color = (0, 0, 255)
    border = max(3, round(7 * unit))
    # Draw entirely inside the image so all four edges have the same width.
    canvas[:border, :] = phase_color
    canvas[-border:, :] = phase_color
    canvas[:, :border] = phase_color
    canvas[:, -border:] = phase_color
    text(phase, margin, margin + round(38 * unit), 1.35, phase_color, outline=False)
    if c and c['active'] and c['target_lost']:
        text('TARGET LOST', margin, margin + round(77 * unit), .7, phase_color)
    if state['image']['health'] != 'ok':
        text('CAMERA: ' + state['image']['health'].upper(), margin,
             margin + round(113 * unit), .7, yellow)

    g, v = current('gps'), current('velocity')
    fix = {0: 'NO FIX', 1: 'NO FIX', 2: '2D', 3: '3D', 4: 'DGPS',
           5: 'RTK FLOAT', 6: 'RTK FIXED', 8: 'EXTRAPOLATED'}
    gps = fix.get(g['fix_type'], 'UNKNOWN') if g else state['gps']['health'].upper()
    satellites = str(g['satellites']) if g else '--'
    def speed(key):
        if v is None:
            return state['velocity']['health'].upper()
        return f'{v[key]:+.2f} m/s' if v[key] is not None else 'INVALID'
    lines = [f'GPS  {gps}', f'SAT  {satellites}',
             'H SPEED  ' + speed('horizontal_m_s'), 'V SPEED  ' + speed('vertical_m_s')]
    for i, line in enumerate(lines):
        color = white
        if i == 0:
            color = (0, 220, 0) if g and g['fix_type'] in (3, 4, 5, 6) else (0, 0, 255)
        text(line, w-margin, margin+round((26+40*i)*unit), color=color, align='right')

    b = current('battery')
    voltage = f'{b["voltage_v"]:.2f} V' if b and b['connected'] and b['voltage_v'] is not None else '-- V'
    percent = f'{b["remaining_pct"]:.0f} %' if b and b['connected'] and b['remaining_pct'] is not None else '-- %'
    text(voltage + '     ' + percent, w//2, h-margin, .95, align='center')

    detection = current('detection')
    relative = detection.get('body_position_m') if detection else None
    if relative is None:
        text('TARGET FRD: --', w-margin, h-margin, .8, align='right')
    else:
        text('TARGET RELATIVE (m)', w-margin, h-margin-round(114*unit), .8, align='right')
        for index, (axis, value) in enumerate(zip(('FWD', 'RIGHT', 'DOWN'), relative)):
            text(f'{axis}  {value:+.2f}', w-margin, h-margin-round((76-38*index)*unit),
                 .8, align='right')

    center, length = (w//2, h//2), max(8, round(18*unit))
    for color, stroke in [((0, 0, 0), thickness+3), (white, thickness)]:
        cv2.line(canvas, (center[0]-length, center[1]), (center[0]+length, center[1]), color, stroke, cv2.LINE_AA)
        cv2.line(canvas, (center[0], center[1]-length), (center[0], center[1]+length), color, stroke, cv2.LINE_AA)
    return canvas

class Monitor(Node):
    def __init__(self):
        super().__init__('landing_monitor')
        defaults = {'transport': 'raw', 'fps': 10.0, 'jpeg_quality': 80, 'stale_after': 2.0}
        config = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        translation = np.asarray(self.declare_parameter('camera_position_body', [0., 0., -.1]).value)
        rotation = np.asarray(self.declare_parameter(
            'camera_orientation_body', [math.sqrt(.5), 0., 0., math.sqrt(.5)]).value)
        norm = np.linalg.norm(rotation)
        if (translation.shape != (3,) or rotation.shape != (4,) or
                not np.isfinite(translation).all() or not math.isfinite(norm) or norm < 1e-9):
            raise ValueError('Invalid camera mounting transform')
        self.camera_extrinsic = (translation, rotation / norm)
        self.transport = config['transport']
        self.fps, self.quality, self.stale = float(config['fps']), int(config['jpeg_quality']), float(config['stale_after'])
        if self.transport not in ('raw', 'compressed') or self.fps <= 0 or self.stale <= 0 or not 1 <= self.quality <= 100:
            raise ValueError('Invalid monitoring transport/rate/quality/staleness configuration')
        qos = SENSOR_QOS
        self.raw_pub = self.create_publisher(Image, '/pl/monitor/image', qos)
        self.jpeg_pub = self.create_publisher(CompressedImage, '/pl/monitor/image/compressed', qos)
        self.status_pub = self.create_publisher(String, '/pl/monitor/status', qos)
        self.inputs, self.samples = {}, {}
        self.image = None
        self.header = None
        self.last_image_decode = -math.inf
        self.create_timer(1/self.fps, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, key, value):
        self.samples[key] = (time.monotonic(), value)

    def json_input(self, key, msg):
        try:
            value = json.loads(msg.data)
            if key == 'detection':
                value['body_position_m'] = body_relative_position(value, self.camera_extrinsic)
            self.receive(key, value)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5)

    def image_input(self, msg):
        now = time.monotonic()
        if now - self.last_image_decode < 1/self.fps:
            return
        try:
            if self.transport == 'raw':
                image = to_bgr(msg)
            else:
                image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError('Invalid JPEG input')
            self.image, self.header = image, msg.header
            self.last_image_decode = now
            self.receive('image', {'stamp_ns': msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec})
        except (ValueError, cv2.error) as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5)

    def sync_inputs(self, need_status, need_image):
        desired = {}
        if need_status:
            desired = {
                'control': (String, '/pl/control/status', lambda m: self.json_input('control', m)),
                'detection': (String, '/pl/detection/status', lambda m: self.json_input('detection', m)),
                'velocity': (VehicleLocalPosition, px4_topic('vehicle_local_position', VehicleLocalPosition),
                             lambda m: self.receive('velocity', {
                                 'horizontal_m_s': finite(math.hypot(m.vx, m.vy)) if m.v_xy_valid else None,
                                 'vertical_m_s': finite(m.vz) if m.v_z_valid else None})),
                'gps': (SensorGps, px4_topic('vehicle_gps_position', SensorGps),
                        lambda m: self.receive('gps', {'fix_type': m.fix_type, 'satellites': m.satellites_used})),
                'battery': (BatteryStatus, px4_topic('battery_status', BatteryStatus),
                            lambda m: self.receive('battery', {'connected': m.connected,
                                'voltage_v': finite(m.voltage_v) if m.voltage_v > 0 else None,
                                'remaining_pct': float(m.remaining*100) if 0 <= m.remaining <= 1 else None})),
            }
        if need_image:
            cls, topic = (Image, '/image_proc') if self.transport == 'raw' else (CompressedImage, '/image_proc/compressed')
            desired['image'] = (cls, topic, self.image_input)
        for key in set(self.inputs) - set(desired):
            self.destroy_subscription(self.inputs.pop(key))
            self.samples.pop(key, None)
            if key == 'image':
                self.image = self.header = None
        for key, (cls, topic, callback) in desired.items():
            if key not in self.inputs:
                self.inputs[key] = self.create_subscription(cls, topic, callback, SENSOR_QOS)

    def snapshot(self):
        now = time.monotonic()
        result = {}
        for key in ('image', 'control', 'detection', 'velocity', 'gps', 'battery'):
            sample = self.samples.get(key)
            age = now-sample[0] if sample else None
            result[key] = {'health': ('ok' if age < self.stale else 'stale') if sample else 'no data',
                           'age_s': age, 'value': sample[1] if sample else None}
        if 'image' not in self.inputs:
            result['image']['health'] = 'not subscribed'
        return result

    def tick(self):
        raw = self.raw_pub.get_subscription_count() > 0
        jpeg = self.jpeg_pub.get_subscription_count() > 0
        status = self.status_pub.get_subscription_count() > 0
        self.sync_inputs(raw or jpeg or status, raw or jpeg)
        if not (raw or jpeg or status):
            return
        state = self.snapshot()
        if status:
            self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))
        if not (raw or jpeg):
            return
        frame = render(self.image, state)
        if raw:
            msg = Image(height=frame.shape[0], width=frame.shape[1], encoding='bgr8', step=frame.shape[1]*3)
            if self.header is not None:
                msg.header = self.header
            msg.data = array('B', frame.tobytes())
            self.raw_pub.publish(msg)
        if jpeg:
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                msg = CompressedImage(format='bgr8; jpeg compressed bgr8')
                if self.header is not None:
                    msg.header = self.header
                msg.data = array('B', encoded.tobytes())
                self.jpeg_pub.publish(msg)


def main():
    rclpy.init()
    node = Monitor()
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
