import math
from enum import Enum, auto

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    VehicleLandDetected,
)

NAN = float('nan')

# PX4 vehicle command constants
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_DO_SET_MODE = 176

# PX4 custom main modes
PX4_CUSTOM_MAIN_MODE_AUTO = 4.0
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0

# PX4 custom sub-modes (for AUTO)
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3.0

# Optical-to-NED rotation: X right, Y down, Z fwd -> X fwd, Y right, Z down
R_OPT_NED = Rotation.from_matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

# Camera-to-body offset in body frame (meters, NED)
CAMERA_OFFSET = np.array([0.0, 0.0, -0.1])


class State(Enum):
    IDLE = auto()
    SEARCH = auto()
    APPROACH = auto()
    DESCEND = auto()
    FINISHED = auto()


class ArucoTag:
    def __init__(self):
        self.position = np.array([NAN, NAN, NAN])
        self.orientation = Rotation.identity()
        self.timestamp = None

    def valid(self):
        return not math.isnan(self.position[0])


class PrecisionLandNode(Node):
    def __init__(self):
        super().__init__('precision_land')

        self._declare_parameters()

        self._state = State.IDLE
        self._tag = ArucoTag()
        self._target_lost_prev = True
        self._search_waypoints = []
        self._search_waypoint_index = 0
        self._approach_altitude = NAN
        self._land_detected = False
        self._search_started = False

        # PI controller integrators
        self._vel_x_integral = 0.0
        self._vel_y_integral = 0.0

        # OFFBOARD startup
        self._offboard_active = False
        self._heartbeat_count = 0
        self._startup_done = False

        # Vehicle state
        self._vehicle_pos = np.zeros(3)     # NED
        self._vehicle_vel = np.zeros(3)     # NED
        self._vehicle_rot = Rotation.identity()

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriptions
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._vehicle_local_position_callback,
            qos_best_effort,
        )
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._vehicle_attitude_callback,
            qos_best_effort,
        )
        self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self._vehicle_land_detected_callback,
            qos_best_effort,
        )
        self.create_subscription(
            PoseStamped,
            '/target_pose',
            self._target_pose_callback,
            qos_best_effort,
        )

        # Publishers
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_best_effort
        )
        self._trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_best_effort
        )
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_best_effort
        )
        self._target_pose_world_pub = self.create_publisher(
            PoseStamped, '/target_pose_world', qos_best_effort
        )

        # Services
        self.create_service(Trigger, '~/start_landing', self._start_landing_callback)
        self.create_service(Trigger, '~/abort', self._abort_callback)

        # 10 Hz update timer
        self.create_timer(0.1, self._update)

        self.get_logger().info('PrecisionLand node started. Call ~/start_landing to begin.')

    def _declare_parameters(self):
        self.declare_parameter('descent_vel', 1.0)
        self.declare_parameter('vel_p_gain', 1.5)
        self.declare_parameter('vel_i_gain', 0.0)
        self.declare_parameter('max_velocity', 3.0)
        self.declare_parameter('target_timeout', 3.0)
        self.declare_parameter('delta_position', 0.25)
        self.declare_parameter('delta_velocity', 0.25)

        self._param_descent_vel = self.get_parameter('descent_vel').value
        self._param_vel_p_gain = self.get_parameter('vel_p_gain').value
        self._param_vel_i_gain = self.get_parameter('vel_i_gain').value
        self._param_max_velocity = self.get_parameter('max_velocity').value
        self._param_target_timeout = self.get_parameter('target_timeout').value
        self._param_delta_position = self.get_parameter('delta_position').value
        self._param_delta_velocity = self.get_parameter('delta_velocity').value

        self.get_logger().info(f'descent_vel: {self._param_descent_vel}')
        self.get_logger().info(f'vel_i_gain: {self._param_vel_i_gain}')

    # ------------------------------------------------------------------ #
    # Subscriptions                                                        #
    # ------------------------------------------------------------------ #

    def _vehicle_local_position_callback(self, msg):
        self._vehicle_pos = np.array([msg.x, msg.y, msg.z])
        self._vehicle_vel = np.array([msg.vx, msg.vy, msg.vz])

    def _vehicle_attitude_callback(self, msg):
        # px4_msgs VehicleAttitude: q = [w, x, y, z]
        self._vehicle_rot = Rotation.from_quat(
            [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]  # scipy expects [x, y, z, w]
        )

    def _vehicle_land_detected_callback(self, msg):
        self._land_detected = msg.landed

    def _target_pose_callback(self, msg):
        if not self._search_started:
            return

        tag_pos_cam = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        tag_rot_cam = Rotation.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])

        # Transform: world = vehicle * camera_body * tag_camera
        #   tag_pos_body = camera_offset + R_opt_ned * tag_pos_cam
        #   tag_pos_world = vehicle_pos + vehicle_rot * tag_pos_body
        tag_pos_ned = R_OPT_NED.apply(tag_pos_cam)
        tag_pos_body = CAMERA_OFFSET + tag_pos_ned
        tag_pos_world = self._vehicle_pos + self._vehicle_rot.apply(tag_pos_body)

        # World orientation: vehicle_rot * R_opt_ned * tag_rot_cam
        tag_rot_world = self._vehicle_rot * R_OPT_NED * tag_rot_cam

        self._tag.position = tag_pos_world
        self._tag.orientation = tag_rot_world
        self._tag.timestamp = self.get_clock().now()

        # Publish world-frame pose for visualization
        world_pose = PoseStamped()
        world_pose.header.stamp = self.get_clock().now().to_msg()
        world_pose.header.frame_id = 'map'
        world_pose.pose.position.x = float(tag_pos_world[0])
        world_pose.pose.position.y = float(tag_pos_world[1])
        world_pose.pose.position.z = float(tag_pos_world[2])
        q = tag_rot_world.as_quat()  # [x, y, z, w]
        world_pose.pose.orientation.x = float(q[0])
        world_pose.pose.orientation.y = float(q[1])
        world_pose.pose.orientation.z = float(q[2])
        world_pose.pose.orientation.w = float(q[3])
        self._target_pose_world_pub.publish(world_pose)

    # ------------------------------------------------------------------ #
    # Services                                                             #
    # ------------------------------------------------------------------ #

    def _start_landing_callback(self, request, response):
        if self._offboard_active:
            response.success = False
            response.message = 'Landing already in progress'
            return response

        self.get_logger().info('start_landing called — starting OFFBOARD sequence')
        self._offboard_active = True
        self._heartbeat_count = 0
        self._startup_done = False
        response.success = True
        response.message = 'Landing sequence initiated'
        return response

    def _abort_callback(self, request, response):
        self.get_logger().info('Abort called — stopping landing sequence')
        self._offboard_active = False
        self._startup_done = False
        self._search_started = False
        self._state = State.IDLE
        # Switch to LOITER so the vehicle holds position
        self._send_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=PX4_CUSTOM_MAIN_MODE_AUTO,
            param3=PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
        )
        response.success = True
        response.message = 'Abort complete'
        return response

    # ------------------------------------------------------------------ #
    # 10 Hz timer                                                          #
    # ------------------------------------------------------------------ #

    def _update(self):
        if not self._offboard_active:
            return

        self._publish_offboard_control_mode()

        # Wait for a few heartbeats before arming/switching modes
        if self._heartbeat_count < 5:
            self._heartbeat_count += 1
            # Send a hold-position setpoint so PX4 has something to track
            self._publish_position_setpoint(self._vehicle_pos)
            return

        if not self._startup_done:
            self.get_logger().info('Sending ARM + OFFBOARD mode commands')
            self._arm()
            self._set_offboard_mode()
            self._startup_done = True
            self._generate_search_waypoints()
            self._search_started = True
            self._switch_to_state(State.SEARCH)
            return

        self._run_state_machine()

    # ------------------------------------------------------------------ #
    # State machine                                                        #
    # ------------------------------------------------------------------ #

    def _run_state_machine(self):
        target_lost = self._check_target_timeout()

        if target_lost and not self._target_lost_prev:
            self.get_logger().info(f'Target lost: State {self._state.name}')
        elif not target_lost and self._target_lost_prev:
            self.get_logger().info('Target acquired')
        self._target_lost_prev = target_lost

        if self._state == State.IDLE:
            pass

        elif self._state == State.SEARCH:
            if self._tag.valid():
                self._approach_altitude = self._vehicle_pos[2]
                self._switch_to_state(State.APPROACH)
                return

            wp = self._search_waypoints[self._search_waypoint_index]
            self._publish_position_setpoint(wp)

            if self._position_reached(wp):
                self._search_waypoint_index += 1
                if self._search_waypoint_index >= len(self._search_waypoints):
                    self._search_waypoint_index = 0

        elif self._state == State.APPROACH:
            if target_lost:
                self.get_logger().info(f'Failed! Target lost during {self._state.name}')
                self._switch_to_state(State.IDLE)
                return

            target_pos = np.array([
                self._tag.position[0],
                self._tag.position[1],
                self._approach_altitude,
            ])
            self._publish_position_setpoint(target_pos)

            if self._position_reached(target_pos):
                self._switch_to_state(State.DESCEND)

        elif self._state == State.DESCEND:
            if target_lost:
                self.get_logger().info(f'Failed! Target lost during {self._state.name}')
                self._switch_to_state(State.IDLE)
                return

            vx, vy = self._calculate_velocity_setpoint_xy()
            tag_yaw = self._tag.orientation.as_euler('zyx')[0]
            self._publish_velocity_setpoint(vx, vy, self._param_descent_vel, tag_yaw)

            if self._land_detected:
                self._switch_to_state(State.FINISHED)

        elif self._state == State.FINISHED:
            self.get_logger().info('Landing complete!')
            self._offboard_active = False
            self._search_started = False

    # ------------------------------------------------------------------ #
    # Control helpers                                                      #
    # ------------------------------------------------------------------ #

    def _calculate_velocity_setpoint_xy(self):
        p_gain = self._param_vel_p_gain
        i_gain = self._param_vel_i_gain
        max_vel = self._param_max_velocity

        delta_x = self._vehicle_pos[0] - self._tag.position[0]
        delta_y = self._vehicle_pos[1] - self._tag.position[1]

        self._vel_x_integral = max(-max_vel, min(max_vel, self._vel_x_integral + delta_x))
        self._vel_y_integral = max(-max_vel, min(max_vel, self._vel_y_integral + delta_y))

        vx = -(delta_x * p_gain + self._vel_x_integral * i_gain)
        vy = -(delta_y * p_gain + self._vel_y_integral * i_gain)

        vx = max(-max_vel, min(max_vel, vx))
        vy = max(-max_vel, min(max_vel, vy))

        return vx, vy

    def _check_target_timeout(self):
        if not self._tag.valid():
            return True
        if self._tag.timestamp is None:
            return True
        elapsed = (self.get_clock().now() - self._tag.timestamp).nanoseconds * 1e-9
        return elapsed > self._param_target_timeout

    def _position_reached(self, target):
        delta = target - self._vehicle_pos
        return (
            np.linalg.norm(delta) < self._param_delta_position
            and np.linalg.norm(self._vehicle_vel) < self._param_delta_velocity
        )

    # ------------------------------------------------------------------ #
    # Search waypoints                                                     #
    # ------------------------------------------------------------------ #

    def _generate_search_waypoints(self):
        start_x = 0.0
        start_y = 0.0
        current_z = self._vehicle_pos[2]
        min_z = -1.0  # Stop spiraling above 1 m AGL (NED z = -1)
        max_radius = 2.0
        layer_spacing = 0.5
        points_per_layer = 16

        raw = int((min_z - current_z) / layer_spacing)
        num_layers = max(1, raw // 2)

        waypoints = []
        for _ in range(num_layers):
            layer_wps = []
            radius = 0.0
            for point in range(points_per_layer + 1):
                angle = 2.0 * math.pi * point / points_per_layer
                x = start_x + radius * math.cos(angle)
                y = start_y + radius * math.sin(angle)
                layer_wps.append(np.array([x, y, current_z]))
                radius += max_radius / points_per_layer

            waypoints.extend(layer_wps)
            current_z += layer_spacing

            reversed_wps = [np.array([wp[0], wp[1], current_z]) for wp in reversed(layer_wps)]
            waypoints.extend(reversed_wps)
            current_z += layer_spacing

        self._search_waypoints = waypoints
        self._search_waypoint_index = 0
        self.get_logger().info(f'Generated {len(waypoints)} search waypoints')

    # ------------------------------------------------------------------ #
    # PX4 publishers                                                       #
    # ------------------------------------------------------------------ #

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        # Set the active control mode; only one should be true at a time
        msg.position = (self._state in (State.SEARCH, State.APPROACH) or not self._startup_done)
        msg.velocity = (self._state == State.DESCEND)
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self._offboard_mode_pub.publish(msg)

    def _publish_position_setpoint(self, pos):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = [float(pos[0]), float(pos[1]), float(pos[2])]
        msg.velocity = [NAN, NAN, NAN]
        msg.yaw = NAN
        self._trajectory_setpoint_pub.publish(msg)

    def _publish_velocity_setpoint(self, vx, vy, vz, yaw=NAN):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = [NAN, NAN, NAN]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yaw = float(yaw)
        self._trajectory_setpoint_pub.publish(msg)

    def _send_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._vehicle_command_pub.publish(msg)

    def _arm(self):
        self.get_logger().info('Sending ARM command')
        self._send_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _set_offboard_mode(self):
        self.get_logger().info('Switching to OFFBOARD mode')
        self._send_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        )

    def _switch_to_state(self, state):
        self.get_logger().info(f'Switching to {state.name}')
        self._state = state


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
