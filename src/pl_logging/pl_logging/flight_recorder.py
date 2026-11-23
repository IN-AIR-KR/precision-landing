"""Record precision-landing telemetry and the compressed HUD to SQLite."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess

import rclpy
from px4_msgs.msg import VehicleStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String


# Match both the unversioned (Humble/PX4 v1.16) and versioned PX4 DDS topics.
TOPIC_FILTER = (
    r'^(/pl/(control/status|detection/status|monitor/(status|image/compressed))'
    r'|/pl/sim/(marker_pose|vehicle_pose)'
    r'|/pl/camera/camera_info'
    r'|/pl/(target_observation|target_pose_world)'
    r'|/fmu/out/(vehicle_status|vehicle_land_detected|vehicle_local_position|'
    r'vehicle_attitude|vehicle_gps_position|battery_status|vehicle_command_ack)(_v[0-9]+)?'
    r'|/fmu/in/(trajectory_setpoint|vehicle_rates_setpoint|vehicle_command)(_v[0-9]+)?)$'
)


def default_log_root():
    explicit = os.environ.get('PRECISION_ROOT')
    if explicit:
        return Path(explicit).expanduser().resolve() / 'flight_logs'

    for start in (Path.cwd(), Path(__file__).resolve().parent):
        for path in (start, *start.parents):
            if (path / 'scripts/env.sh').is_file() and (path / 'src/pl_bringup/package.xml').is_file():
                return path / 'flight_logs'
    raise ValueError('Run from the precision-landing checkout or pass --log-dir.')


class FlightRecorder(Node):
    def __init__(self, root, profile, config_yaml=None, sim_world=None, controller=None):
        super().__init__('flight_recorder')
        self.root = root
        self.profile = profile
        self.config_yaml = config_yaml
        self.sim_world = sim_world
        self.controller = controller
        self.test_metadata = {}
        self.armed = False
        self.process = None
        self.output = None

        version = getattr(VehicleStatus, 'MESSAGE_VERSION', 0)
        topic = '/fmu/out/vehicle_status' + (f'_v{version}' if version else '')
        self.create_subscription(VehicleStatus, topic, self.on_status, qos_profile_sensor_data)
        if profile in ('sim', 'hitl'):
            self.create_subscription(
                String, '/pl/flight_test/metadata', self.on_test_metadata,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
        self.create_timer(1.0, self.check_process)
        self.get_logger().info(f'Waiting for PX4 ARM on {topic}')

    def on_status(self, msg):
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if armed == self.armed:
            return
        self.armed = armed
        if armed:
            self.start_recording()
        else:
            self.stop_recording()
            self.output = None
            self.test_metadata = {}

    def on_test_metadata(self, message):
        try:
            data = json.loads(message.data)
            world = data['world']
            motion = data['target_motion']
            speed = data['target_speed_mps']
            if not isinstance(world, str) or not world or (self.sim_world and world != self.sim_world):
                return
            if (isinstance(speed, bool) or not isinstance(speed, (int, float))
                    or not math.isfinite(speed) or motion != ('moving' if speed else 'stationary')):
                return
        except (KeyError, TypeError, ValueError):
            return
        self.test_metadata = dict(world=world, target_motion=motion, target_speed_mps=speed)
        if self.output is not None:
            self.write_metadata()

    def write_metadata(self):
        metadata = dict(world=self.sim_world) if self.sim_world else {}
        if self.controller:
            metadata['controller'] = self.controller
        if self.sim_world == 'aruco':
            metadata.update(target_motion='stationary', target_speed_mps=0.0)
        metadata.update(self.test_metadata)
        if not metadata:
            return
        path = self.output.parent / 'flight_metadata.json'
        temporary = path.with_suffix('.tmp')
        try:
            temporary.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
            temporary.replace(path)
        except OSError as exc:
            self.get_logger().warning(f'Could not save flight metadata: {exc}')

    def start_recording(self):
        try:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            record_dir = self.root / f'{stamp}_{self.profile}'
            record_dir.mkdir(mode=0o700)
            self.output = record_dir / 'bag'
            self.write_metadata()
            if self.config_yaml is not None:
                (self.output.parent / 'real.yaml').write_text(self.config_yaml, encoding='utf-8')
            self.process = subprocess.Popen(
                ['ros2', 'bag', 'record', '--storage', 'sqlite3',
                 '--regex', TOPIC_FILTER, '-o', str(self.output)],
                start_new_session=True,
            )
        except OSError as exc:
            self.get_logger().error(f'Could not start bag recorder: {exc}')
            return
        self.get_logger().info(f'ARM: recording to {self.output}')

    def check_process(self):
        if self.process is not None and self.process.poll() is not None:
            self.get_logger().error(f'Bag recorder exited during flight: {self.process.returncode}')
            self.process = None

    def stop_recording(self):
        process = self.process
        if process is None:
            return
        self.process = None
        # SIGINT lets rosbag flush its buffer and write metadata.yaml.
        process.send_signal(signal.SIGINT)
        try:
            code = process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            self.get_logger().error(f'Bag recorder timed out while closing {self.output}')
            return
        if code == 0:
            self.get_logger().info(f'Closed bag: {self.output}')
        else:
            self.get_logger().error(f'Bag recorder exited with status {code}: {self.output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, help='Log root (default: checkout/flight_logs)')
    parser.add_argument('--profile', default='manual', choices=['manual', 'real', 'sim', 'hitl'])
    parser.add_argument('--sim-world', help='Gazebo world for this recording')
    parser.add_argument('--controller', choices=['pid', 'mpc', 'pampc', 'chao_mpc'], help='Controller selected by bringup')
    args = parser.parse_args()
    try:
        root = args.log_dir.expanduser().resolve() if args.log_dir else default_log_root()
        root.mkdir(parents=True, exist_ok=True)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'flight_recorder: {exc}\n')

    rclpy.init()
    try:
        node = FlightRecorder(root, args.profile, os.environ.pop('PL_FLIGHT_REAL_YAML', None),
                              args.sim_world, args.controller)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.stop_recording()
            node.destroy_node()
    finally:
        rclpy.try_shutdown()
