#!/usr/bin/env python3
"""
정밀착륙 제어 노드 (FSM + PID)

FSM 상태:
  IDLE → SEARCH_V_MARKER → ALIGN_V_MARKER → DESCEND_V_MARKER
       → ALIGN_ARUCO → DESCEND_ARUCO → FINAL_DESCENT → LANDED
  (각 단계에서 ABORT 폴백 존재)
"""

import math
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation

from std_srvs.srv import Trigger
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleLandDetected,
)
from pl_msgs.msg import MarkerDetection, LandingState


class State(Enum):
    IDLE = auto()
    SEARCH_V_MARKER = auto()
    ALIGN_V_MARKER = auto()
    DESCEND_V_MARKER = auto()
    ALIGN_ARUCO = auto()
    DESCEND_ARUCO = auto()
    FINAL_DESCENT = auto()
    LANDED = auto()
    ABORT = auto()


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, max_output: float, dt: float = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.dt = dt
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, error: float) -> float:
        self._integral += error * self.dt
        self._integral = max(-self.max_output, min(self.max_output, self._integral))
        derivative = (error - self._prev_error) / self.dt
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.max_output, min(self.max_output, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


class LandingControllerNode(Node):
    # optical-to-NED 좌표계 변환 행렬 (tracktor-beam 동일)
    R_OPT_NED = Rotation.from_matrix([[0, -1, 0],
                                       [1,  0, 0],
                                       [0,  0, 1]])

    def __init__(self):
        super().__init__('landing_controller_node')

        self._declare_all_parameters()

        # 상태 변수
        self._state = State.IDLE
        self._prev_state = State.IDLE
        self._state_entry_time: float = 0.0
        self._align_ok_start: float = 0.0
        self._fallback_reason: str = ''

        # 센서 캐시
        self._local_pos: VehicleLocalPosition | None = None
        self._vehicle_status: VehicleStatus | None = None
        self._land_detected = False
        self._v_marker: MarkerDetection | None = None
        self._aruco: MarkerDetection | None = None
        self._last_v_marker_time: float = 0.0
        self._last_aruco_time: float = 0.0

        # 홀드 포지션 (m, NED)
        self._hold_x = 0.0
        self._hold_y = 0.0
        self._hold_z = -35.0  # 초기 고도 (음수 = 위쪽)

        # OFFBOARD 카운터
        self._offboard_counter = 0
        self._offboard_ready = False

        # PID 컨트롤러 (x, y 각각)
        self._pid_x_coarse = self._make_pid('v_marker')
        self._pid_y_coarse = self._make_pid('v_marker')
        self._pid_x_fine = self._make_pid('aruco')
        self._pid_y_fine = self._make_pid('aruco')

        # QoS
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 구독
        self.create_subscription(
            MarkerDetection, '/pl/v_marker_detection', self._v_marker_cb, 10)
        self.create_subscription(
            MarkerDetection, '/pl/aruco_detection', self._aruco_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_cb, px4_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._vehicle_status_cb, px4_qos)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._land_detected_cb, px4_qos)

        # 발행
        self._pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self._pub_setpoint = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self._pub_state = self.create_publisher(
            LandingState, '/pl/landing_state', 10)

        # 서비스
        self.create_service(Trigger, '~/start_landing', self._start_landing_srv)
        self.create_service(Trigger, '~/abort', self._abort_srv)

        # 제어 루프 10 Hz
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info('정밀착륙 컨트롤러 시작 (IDLE)')

    # ────────────────────────────────────────────
    # 파라미터 선언
    # ────────────────────────────────────────────
    def _declare_all_parameters(self):
        params = {
            'alt_switch_to_aruco': 8.0,
            'alt_final_descent': 2.0,
            'v_marker_kp': 0.8,
            'v_marker_ki': 0.05,
            'v_marker_kd': 0.10,
            'v_marker_max_vel': 3.0,
            'aruco_kp': 1.5,
            'aruco_ki': 0.0,
            'aruco_kd': 0.15,
            'aruco_max_vel': 1.5,
            'descend_v_marker_vel': 0.5,
            'descend_aruco_vel': 0.4,
            'final_descent_vel': 0.3,
            'align_v_marker_xy_threshold': 1.5,
            'align_v_marker_vel_threshold': 0.3,
            'align_v_marker_hold_time': 1.0,
            'align_aruco_xy_threshold': 0.5,
            'align_aruco_vel_threshold': 0.2,
            'align_aruco_hold_time': 0.5,
            'search_timeout': 60.0,
            'align_v_marker_timeout': 120.0,
            'descend_v_marker_timeout': 120.0,
            'align_aruco_timeout': 60.0,
            'descend_aruco_timeout': 90.0,
            'final_descent_timeout': 60.0,
            'target_lost_timeout': 3.0,
            'camera_offset_z': -0.1,
        }
        for k, v in params.items():
            self.declare_parameter(k, v)

    def _p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _make_pid(self, prefix: str) -> PIDController:
        return PIDController(
            kp=self._p(f'{prefix}_kp'),
            ki=self._p(f'{prefix}_ki'),
            kd=self._p(f'{prefix}_kd'),
            max_output=self._p(f'{prefix}_max_vel'),
        )

    # ────────────────────────────────────────────
    # 콜백
    # ────────────────────────────────────────────
    def _v_marker_cb(self, msg: MarkerDetection):
        self._v_marker = msg
        if msg.detected:
            self._last_v_marker_time = self.get_clock().now().nanoseconds * 1e-9

    def _aruco_cb(self, msg: MarkerDetection):
        self._aruco = msg
        if msg.detected:
            self._last_aruco_time = self.get_clock().now().nanoseconds * 1e-9

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg
        if self._state == State.IDLE:
            self._hold_x = msg.x
            self._hold_y = msg.y
            self._hold_z = msg.z

    def _vehicle_status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _land_detected_cb(self, msg: VehicleLandDetected):
        self._land_detected = msg.landed

    # ────────────────────────────────────────────
    # 서비스 핸들러
    # ────────────────────────────────────────────
    def _start_landing_srv(self, request, response):
        if self._state == State.IDLE:
            self._transition(State.SEARCH_V_MARKER)
            response.success = True
            response.message = 'Landing sequence started'
        else:
            response.success = False
            response.message = f'Cannot start: current state is {self._state.name}'
        return response

    def _abort_srv(self, request, response):
        self._abort('서비스 호출에 의한 중단')
        response.success = True
        response.message = 'Abort issued'
        return response

    # ────────────────────────────────────────────
    # 메인 제어 루프
    # ────────────────────────────────────────────
    def _control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # OFFBOARD 하트비트 (IDLE 포함 항상 발행)
        self._publish_offboard_mode()

        if self._state == State.IDLE:
            self._hold_position()
            self._publish_state()
            return

        # OFFBOARD 준비 (5회 하트비트 이후 Arm + Mode 전환)
        if not self._offboard_ready:
            self._offboard_counter += 1
            if self._offboard_counter >= 5:
                self._arm()
                self._set_offboard_mode()
                self._offboard_ready = True
            self._hold_position()
            self._publish_state()
            return

        alt = self._altitude_agl()
        time_in_state = now - self._state_entry_time

        # ── SEARCH_V_MARKER ──
        if self._state == State.SEARCH_V_MARKER:
            self._hold_position()
            if self._v_marker and self._v_marker.detected and self._v_marker.confidence > 0.4:
                self._transition(State.ALIGN_V_MARKER)
            elif time_in_state > self._p('search_timeout'):
                self._abort(f'V-마커 탐색 타임아웃 ({self._p("search_timeout")}s)')

        # ── ALIGN_V_MARKER ──
        elif self._state == State.ALIGN_V_MARKER:
            err_x, err_y = self._compute_error_v_marker()
            vx = self._pid_x_coarse.update(err_x)
            vy = self._pid_y_coarse.update(err_y)
            self._publish_velocity(vx, vy, 0.0, self._hold_z)

            xy_err = math.hypot(err_x, err_y)
            vel_ok = self._current_xy_speed() < self._p('align_v_marker_vel_threshold')
            if xy_err < self._p('align_v_marker_xy_threshold') and vel_ok:
                if self._align_ok_start == 0.0:
                    self._align_ok_start = now
                elif now - self._align_ok_start >= self._p('align_v_marker_hold_time'):
                    self._transition(State.DESCEND_V_MARKER)
            else:
                self._align_ok_start = 0.0

            if self._v_marker_lost():
                self._transition(State.SEARCH_V_MARKER)
            elif time_in_state > self._p('align_v_marker_timeout'):
                self._abort(f'V-마커 정렬 타임아웃')

        # ── DESCEND_V_MARKER ──
        elif self._state == State.DESCEND_V_MARKER:
            err_x, err_y = self._compute_error_v_marker()
            vx = self._pid_x_coarse.update(err_x)
            vy = self._pid_y_coarse.update(err_y)
            vz = self._p('descend_v_marker_vel')
            self._publish_velocity(vx, vy, vz, None)

            if alt < self._p('alt_switch_to_aruco') and self._aruco and self._aruco.detected:
                self._transition(State.ALIGN_ARUCO)
            elif self._v_marker_lost():
                self._transition(State.ALIGN_V_MARKER)
            elif time_in_state > self._p('descend_v_marker_timeout'):
                self._abort('V-마커 하강 타임아웃')

        # ── ALIGN_ARUCO ──
        elif self._state == State.ALIGN_ARUCO:
            err_x, err_y = self._compute_error_aruco()
            vx = self._pid_x_fine.update(err_x)
            vy = self._pid_y_fine.update(err_y)
            self._publish_velocity(vx, vy, 0.0, self._hold_z)

            xy_err = math.hypot(err_x, err_y)
            vel_ok = self._current_xy_speed() < self._p('align_aruco_vel_threshold')
            if xy_err < self._p('align_aruco_xy_threshold') and vel_ok:
                if self._align_ok_start == 0.0:
                    self._align_ok_start = now
                elif now - self._align_ok_start >= self._p('align_aruco_hold_time'):
                    self._transition(State.DESCEND_ARUCO)
            else:
                self._align_ok_start = 0.0

            if alt < self._p('alt_final_descent'):
                self._transition(State.FINAL_DESCENT)
            elif self._aruco_lost() and alt > self._p('alt_switch_to_aruco'):
                self._transition(State.DESCEND_V_MARKER)
            elif time_in_state > self._p('align_aruco_timeout'):
                self._abort('ArUco 정렬 타임아웃')

        # ── DESCEND_ARUCO ──
        elif self._state == State.DESCEND_ARUCO:
            err_x, err_y = self._compute_error_aruco()
            vx = self._pid_x_fine.update(err_x)
            vy = self._pid_y_fine.update(err_y)
            vz = self._p('descend_aruco_vel')
            self._publish_velocity(vx, vy, vz, None)

            if alt < self._p('alt_final_descent'):
                self._transition(State.FINAL_DESCENT)
            elif self._aruco_lost() and alt > self._p('alt_final_descent'):
                self._transition(State.ALIGN_ARUCO)
            elif time_in_state > self._p('descend_aruco_timeout'):
                self._abort('ArUco 하강 타임아웃')

        # ── FINAL_DESCENT (맹목 하강) ──
        elif self._state == State.FINAL_DESCENT:
            vz = self._p('final_descent_vel')
            self._publish_velocity(0.0, 0.0, vz, None)

            if self._land_detected:
                self._transition(State.LANDED)
            elif time_in_state > self._p('final_descent_timeout'):
                self._abort('최종 하강 타임아웃')

        # ── LANDED ──
        elif self._state == State.LANDED:
            self._disarm()
            self._offboard_ready = False
            self._offboard_counter = 0
            self._transition(State.IDLE)

        # ── ABORT ──
        elif self._state == State.ABORT:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0)  # AUTO LOITER
            self._offboard_ready = False
            self._offboard_counter = 0
            self._transition(State.IDLE)

        self._publish_state()

    # ────────────────────────────────────────────
    # 오차 계산
    # ────────────────────────────────────────────
    def _compute_error_v_marker(self):
        """V-마커 픽셀 오프셋 → NED 오차(m)"""
        if self._v_marker is None or not self._v_marker.detected:
            return 0.0, 0.0
        if self._local_pos is None:
            return 0.0, 0.0

        dist = max(self._v_marker.estimated_distance, 0.1)
        # CameraInfo 없이 추정한 fx는 v_marker_detector에서 씀
        # 여기서는 estimated_distance와 각도 기반 근사 사용
        # image offset은 normalized [-1,1] 로 변환이 필요하나,
        # 실제 focal length를 모르므로 간단히 비율로 처리
        # 더 정밀하려면 camera_info를 직접 구독해야 함
        # 여기서는 estimated_distance * tan(angle) 근사 사용
        img_w = 1280.0  # 기본값, 실제 해상도에 맞게 조정
        img_h = 960.0
        fx_approx = img_w / (2.0 * math.tan(1.74 / 2.0))  # mono_cam FOV 1.74 rad

        dx_cam = (self._v_marker.image_x - img_w / 2.0) / fx_approx * dist
        dy_cam = (self._v_marker.image_y - img_h / 2.0) / fx_approx * dist

        err_ned = self.R_OPT_NED.apply([dx_cam, dy_cam, 0.0])
        return err_ned[0], err_ned[1]

    def _compute_error_aruco(self):
        """ArUco tvec → NED 오차(m)"""
        if self._aruco is None or not self._aruco.detected:
            return 0.0, 0.0
        p = self._aruco.pose.pose.position
        tag_cam = np.array([p.x, p.y, p.z])
        offset = np.array([0.0, 0.0, self._p('camera_offset_z')])
        tag_ned = self.R_OPT_NED.apply(tag_cam) + offset
        return tag_ned[0], tag_ned[1]

    # ────────────────────────────────────────────
    # 검출 유실 판단
    # ────────────────────────────────────────────
    def _v_marker_lost(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self._last_v_marker_time) > self._p('target_lost_timeout')

    def _aruco_lost(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self._last_aruco_time) > self._p('target_lost_timeout')

    # ────────────────────────────────────────────
    # 고도 / 속도 헬퍼
    # ────────────────────────────────────────────
    def _altitude_agl(self) -> float:
        if self._local_pos is None:
            return 999.0
        return -self._local_pos.z  # NED: z 음수 = 위

    def _current_xy_speed(self) -> float:
        if self._local_pos is None:
            return 0.0
        return math.hypot(self._local_pos.vx, self._local_pos.vy)

    # ────────────────────────────────────────────
    # PX4 명령 발행
    # ────────────────────────────────────────────
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        self._pub_offboard.publish(msg)

    def _hold_position(self):
        if self._local_pos is None:
            return
        self._publish_velocity(0.0, 0.0, 0.0, self._hold_z)

    def _publish_velocity(self, vx: float, vy: float, vz: float, hold_z):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.velocity = [vx, vy, vz]
        if hold_z is not None:
            msg.position = [float('nan'), float('nan'), hold_z]
        else:
            msg.position = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        self._pub_setpoint.publish(msg)

        # 현재 hold_z 갱신 (하강 중 z 추적)
        if self._local_pos is not None:
            self._hold_z = self._local_pos.z

    def _arm(self):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('ARM 명령 전송')

    def _disarm(self):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('DISARM 명령 전송')

    def _set_offboard_mode(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self.get_logger().info('OFFBOARD 모드 설정')

    def _send_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._pub_cmd.publish(msg)

    # ────────────────────────────────────────────
    # FSM 전환
    # ────────────────────────────────────────────
    def _transition(self, new_state: State):
        self.get_logger().info(
            f'FSM: {self._state.name} → {new_state.name}')
        self._prev_state = self._state
        self._state = new_state
        self._state_entry_time = self.get_clock().now().nanoseconds * 1e-9
        self._align_ok_start = 0.0

        # 홀드 위치 갱신
        if self._local_pos is not None:
            self._hold_x = self._local_pos.x
            self._hold_y = self._local_pos.y
            self._hold_z = self._local_pos.z

        # PID 리셋
        for pid in (self._pid_x_coarse, self._pid_y_coarse,
                    self._pid_x_fine, self._pid_y_fine):
            pid.reset()

    def _abort(self, reason: str):
        self._fallback_reason = reason
        self.get_logger().warn(f'ABORT: {reason}')
        self._transition(State.ABORT)

    # ────────────────────────────────────────────
    # 상태 발행
    # ────────────────────────────────────────────
    def _publish_state(self):
        msg = LandingState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self._state.name
        msg.previous_state = self._prev_state.name
        msg.altitude_agl = self._altitude_agl()
        msg.v_marker_visible = bool(self._v_marker and self._v_marker.detected)
        msg.aruco_visible = bool(self._aruco and self._aruco.detected)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._state in (State.ALIGN_V_MARKER, State.DESCEND_V_MARKER):
            ex, ey = self._compute_error_v_marker()
        elif self._state in (State.ALIGN_ARUCO, State.DESCEND_ARUCO):
            ex, ey = self._compute_error_aruco()
        else:
            ex, ey = 0.0, 0.0

        msg.error_x = ex
        msg.error_y = ey
        msg.time_in_state = now - self._state_entry_time
        msg.fallback_reason = self._fallback_reason if self._state == State.ABORT else ''
        self._pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LandingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
