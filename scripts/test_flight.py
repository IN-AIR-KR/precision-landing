#!/usr/bin/env python3
"""SITL-only flight check. Run after scripts/run.sh, with scripts/env.sh sourced.
Uses the local PX4 commander and a GCS heartbeat on the onboard UDP endpoint.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleLandDetected
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from pymavlink import mavutil

parser = argparse.ArgumentParser()
parser.add_argument('--speed', type=float, default=0.3)
parser.add_argument('--output', type=Path, default=Path('.run/moving-flight.jsonl'))
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
px4 = Path(os.environ.get('PX4_DIR', str(Path.home() / 'PX4-precision-landing')))
binary = px4 / 'build/px4_sitl_default/bin/px4'
pids = subprocess.check_output(['pgrep', '-x', 'px4'], text=True).split()
assert len(pids) == 1 and Path(f'/proc/{pids[0]}/exe').resolve() == binary.resolve(), 'Expected dedicated local SITL process'
os.environ.setdefault('ROS_DOMAIN_ID', '42')
os.environ.setdefault('GZ_PARTITION', 'precision_landing')
stop = threading.Event()
def heartbeat():
    m = mavutil.mavlink_connection('udpout:127.0.0.1:14580', source_system=255, source_component=190)
    while not stop.is_set():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            m.recv_match(blocking=True, timeout=.1)
    m.close()
thread = threading.Thread(target=heartbeat, daemon=True)
thread.start()
rclpy.init()
node = rclpy.create_node('precision_flight_test')
state = {'tags': 0, 'images': 0}
def position(m): state.update(x=m.x, y=m.y, z=m.z)
def status(m): state.update(nav=m.nav_state, armed=m.arming_state == 2)
def landed(m): state.update(landed=m.landed)
def tag(m): state['tags'] += 1
def image(m): state['images'] += 1
subs = [node.create_subscription(t, topic, cb, qos_profile_sensor_data) for t, topic, cb in [
    (VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', position),
    (VehicleStatus, '/fmu/out/vehicle_status_v1', status),
    (VehicleLandDetected, '/fmu/out/vehicle_land_detected', landed),
    (PoseStamped, '/target_pose', tag), (Image, '/image_proc', image)]]
pub = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
def velocity(speed):
    if os.environ.get('SIM_WORLD') == 'aruco':
        return
    command = ['gz', 'topic', '-t', '/model/aruco_moving_target/cmd_vel',
               '-m', 'gz.msgs.Twist', '-p', f'linear: {{x: {speed}, y: 0}}']
    # A one-shot publisher can exit before Gazebo transport discovers the subscriber.
    for _ in range(3):
        subprocess.run(command, check=True, timeout=5)
phase = 'ready'
start = last = time.monotonic()
success = False
try:
    with args.output.open('w') as log:
        while time.monotonic() - start < 120:
            rclpy.spin_once(node, timeout_sec=.1)
            elapsed = time.monotonic() - start
            if phase == 'ready' and elapsed > 5 and 'z' in state:
                subprocess.run([str(binary.parent / 'px4-commander'), 'takeoff'], check=True)
                phase = 'takeoff'
            elif phase == 'takeoff' and state.get('z', 0) < -2.2:
                msg = VehicleCommand()
                msg.command = 100001
                msg.param1 = 23.
                msg.target_system = msg.target_component = 1
                msg.source_system = 255
                msg.source_component = 190
                msg.from_external = True
                pub.publish(msg)
                phase = 'precision'
            elif phase == 'precision' and state.get('nav') == 23 and state['tags'] > 5:
                velocity(args.speed)
                phase = 'tracking'
            elif phase == 'tracking' and state.get('landed') and not state.get('armed', True):
                success = True
                phase = 'passed'
            if time.monotonic() - last > 1 or success:
                row = dict(elapsed=round(elapsed, 2), phase=phase, **state)
                print(json.dumps(row), flush=True)
                log.write(json.dumps(row) + '\n'); log.flush()
                last = time.monotonic()
            if success:
                break
finally:
    velocity(0)
    if not success:
        subprocess.run([str(binary.parent / 'px4-commander'), 'land'], check=False)
    stop.set(); thread.join(timeout=2)
    node.destroy_node(); rclpy.shutdown()
if not success:
    raise SystemExit('FAIL: flight did not finish within 120 seconds; see .run logs')
print(f'PASS: landing and automatic disarm (target speed {args.speed} m/s)', flush=True)
