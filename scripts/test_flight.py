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
import re
import queue
import signal
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleLandDetected
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import Trigger
from pymavlink import mavutil

parser = argparse.ArgumentParser()
parser.add_argument('--exercise-handover', action='store_true', help='Use via test_mode_safety.py with its slow-descent profile: target loss then Hold/Land')
parser.add_argument('--start-via', choices=['service', 'mavlink'], default='service')
parser.add_argument('--image-transport', choices=['raw', 'compressed'], default='raw', help='Use compressed when testing the PC/Jetson split over Wi-Fi')
parser.add_argument('--speed', type=float, default=0.3)
parser.add_argument('--output', type=Path, default=Path('.run/moving-flight.jsonl'))
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
px4 = Path(os.environ.get('PX4_DIR', str(Path.home() / 'PX4-Autopilot')))
binary = px4 / 'build/px4_sitl_default/bin/px4'
pids = subprocess.check_output(['pgrep', '-x', 'px4'], text=True).split()
assert len(pids) == 1 and Path(f'/proc/{pids[0]}/exe').resolve() == binary.resolve(), 'Expected configured local SITL process'
runtime = Path(f'/proc/{pids[0]}/cwd').resolve()
assert runtime.name == 'px4-rootfs' and runtime.is_relative_to(root), 'Expected project-owned SITL working directory'
os.environ.setdefault('ROS_DOMAIN_ID', '42')
os.environ.setdefault('GZ_PARTITION', 'precision_landing')
stop = threading.Event()
mav_commands = queue.SimpleQueue()
def heartbeat():
    m = mavutil.mavlink_connection('udpout:127.0.0.1:14580', source_system=255, source_component=190)
    while not stop.is_set():
        while not mav_commands.empty():
            m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, *mav_commands.get())
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
    (VehicleStatus, '/fmu/out/vehicle_status_v4', status),
    (VehicleLandDetected, '/fmu/out/vehicle_land_detected', landed),
    (PoseStamped, '/target_pose', tag),
    (CompressedImage if args.image_transport == 'compressed' else Image,
     '/image_proc/compressed' if args.image_transport == 'compressed' else '/image_proc', image)]]
client = node.create_client(Trigger, '/pl/start_landing')
assert client.wait_for_service(timeout_sec=10), 'Start landing service unavailable'
mode_id = None
start_future = repeat_future = None
def velocity(speed):
    subprocess.run(['gz', 'topic', '-t', '/model/aruco_moving_target/cmd_vel', '-m', 'gz.msgs.Twist', '-p', f'linear: {{x: {speed}, y: 0}}'], check=True)
paused_detector = None
phase = 'ready'
start = last = time.monotonic()
success = False
try:
    with args.output.open('w') as log:
        while time.monotonic() - start < 120:
            rclpy.spin_once(node, timeout_sec=.1)
            elapsed = time.monotonic() - start
            if phase == 'ready' and elapsed > 5 and 'z' in state:
                rejected = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, rejected, timeout_sec=7)
                assert rejected.done() and not rejected.result().success, 'Ground start must be rejected'
                print('Ground request rejected: ' + rejected.result().message, flush=True)
                mode_id = int(re.search(r'mode=(\d+)', rejected.result().message)[1])
                subprocess.run([str(binary.parent / 'px4-commander'), 'takeoff'], check=True)
                phase = 'takeoff'
            elif phase == 'takeoff' and state.get('z', 0) < -2.2:
                if args.start_via == 'service':
                    start_future = client.call_async(Trigger.Request())
                else:
                    # PX4/QGC custom AUTO external submode mapping (px4_custom_mode.h).
                    mav_commands.put((1, 4, 11 + mode_id - VehicleStatus.NAVIGATION_STATE_EXTERNAL1, 0, 0, 0, 0))
                phase = 'precision'
            elif phase == 'precision':
                if start_future is not None and start_future.done():
                    response = start_future.result()
                    assert response.success, response.message
                    mode_id = int(re.search(r'mode=(\d+)', response.message)[1])
                    start_future = None
                if mode_id is not None and state.get('nav') == mode_id and state['tags'] > 5:
                    repeat_future = client.call_async(Trigger.Request())
                    velocity(args.speed)
                    phase = 'tracking'
                    if args.exercise_handover:
                        candidates = []
                        for proc in Path('/proc').iterdir():
                            if not proc.name.isdigit(): continue
                            try: command = (proc/'cmdline').read_bytes().split(b'\0')
                            except OSError: continue
                            if str(root/'install/pl_detection/lib/pl_detection/aruco_tracker').encode() in command:
                                candidates.append(int(proc.name))
                        assert len(candidates)==1, 'Expected one local detector'
                        paused_detector = candidates[0]
                        os.kill(paused_detector, signal.SIGSTOP)
                        loss_started = time.monotonic()
                        phase = 'target_loss'
            elif phase == 'target_loss' and time.monotonic()-loss_started > 5.5:
                roslog = Path(os.environ.get('RUN_LOG_DIR', str(root/'.run')))/'ros.log'
                assert 'Failed! Target lost during' in roslog.read_text(), 'Target loss was not detected'
                assert state.get('armed') and not state.get('landed'), 'Expected airborne target-loss test'
                os.kill(paused_detector, signal.SIGCONT)
                paused_detector = None
                mav_commands.put((1, 4, 3, 0, 0, 0, 0))  # QGC/MAVLink AUTO LOITER (Hold)
                phase = 'handover'
                hold_requested = time.monotonic()
            elif phase == 'handover':
                assert time.monotonic()-hold_requested < 8, 'Hold takeover timed out'
                if state.get('nav') == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    assert state['nav'] != mode_id, 'External mode retained control'
                    mav_commands.put((1, 4, 6, 0, 0, 0, 0))  # AUTO LAND
                    phase = 'tracking'
            elif phase == 'tracking' and state.get('landed') and not state.get('armed', True):
                assert repeat_future is not None and repeat_future.done() and repeat_future.result().success, 'Repeated active request failed'
                print('Repeated request: ' + repeat_future.result().message, flush=True)
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
    if paused_detector is not None:
        os.kill(paused_detector, signal.SIGCONT)
    velocity(0)
    if not success:
        subprocess.run([str(binary.parent / 'px4-commander'), 'land'], check=False)
    stop.set(); thread.join(timeout=2)
    node.destroy_node(); rclpy.shutdown()
if not success:
    raise SystemExit('FAIL: flight did not finish within 120 seconds; see .run logs')
print(f'PASS: landing and automatic disarm (target speed {args.speed} m/s)', flush=True)
