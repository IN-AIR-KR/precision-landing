"""Record precision-landing telemetry and the compressed HUD to SQLite."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import tempfile

import rclpy
from px4_msgs.msg import VehicleStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


# Match both the unversioned (Humble/PX4 v1.16) and versioned PX4 DDS topics.
TOPIC_FILTER = (
    r'^(/pl/(control/status|detection/status|monitor/(status|image/compressed))'
    r'|/pl/sim/(marker_pose|vehicle_pose)'
    r'|/target_pose(_world)?'
    r'|/fmu/out/(vehicle_status|vehicle_land_detected|vehicle_local_position|'
    r'vehicle_attitude|vehicle_gps_position|battery_status|vehicle_command_ack)(_v[0-9]+)?'
    r'|/fmu/in/(trajectory_setpoint|vehicle_command)(_v[0-9]+)?)$'
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
    def __init__(self, root, profile):
        super().__init__('flight_recorder')
        self.root = root
        self.profile = profile
        self.armed = False
        self.process = None
        self.output = None

        version = getattr(VehicleStatus, 'MESSAGE_VERSION', 0)
        topic = '/fmu/out/vehicle_status' + (f'_v{version}' if version else '')
        self.create_subscription(VehicleStatus, topic, self.on_status, qos_profile_sensor_data)
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

    def start_recording(self):
        try:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.output = Path(tempfile.mkdtemp(prefix=f'{stamp}_{self.profile}_', dir=self.root)) / 'bag'
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
    args = parser.parse_args()
    try:
        root = args.log_dir.expanduser().resolve() if args.log_dir else default_log_root()
        root.mkdir(parents=True, exist_ok=True)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'flight_recorder: {exc}\n')

    rclpy.init()
    try:
        node = FlightRecorder(root, args.profile)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.stop_recording()
            node.destroy_node()
    finally:
        rclpy.try_shutdown()
