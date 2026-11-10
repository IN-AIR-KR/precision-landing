#!/usr/bin/env python3
"""Record an isolated static or moving SITL flight, preserving exact sources and versions."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument('scenario', choices=['static', 'moving'])
p.add_argument('--output', type=Path)
a = p.parse_args()
world = 'aruco' if a.scenario == 'static' else 'aruco_moving'
out = (a.output or ROOT / 'baselines' / (time.strftime('%Y%m%d-%H%M%S') + '-' + a.scenario)).resolve()
out.mkdir(parents=True, exist_ok=False)
env = dict(os.environ, ROS_DOMAIN_ID='42', GZ_PARTITION='precision_landing', HEADLESS='1', SIM_WORLD=world, RUN_LOG_DIR=str(out / 'logs'))
px4 = Path(env.get('PX4_DIR', str(Path.home() / 'PX4-precision-landing')))
def output(cmd, cwd=ROOT):
    return subprocess.check_output(cmd, cwd=cwd, text=True).strip()
def revision(path):
    return dict(path=str(path), commit=output(['git', 'rev-parse', 'HEAD'], path), status=output(['git', 'status', '--short'], path))
versions = {'scenario': a.scenario, 'world': world, 'speed_m_s': 0 if a.scenario == 'static' else .3,
            'ros_domain_id': 42, 'gazebo_partition': 'precision_landing',
            'repositories': {name: revision(path) for name, path in {
                'project': ROOT, 'PX4': px4, 'px4_msgs': ROOT/'src/px4_msgs',
                'px4_ros2_cpp': ROOT/'src/px4-ros2-interface-lib', 'ros_gz': ROOT/'src/ros_gz',
                'vision_opencv': ROOT/'src/vision_opencv', 'PX4_gz': px4/'Tools/simulation/gz'}.items()},
            'gazebo': output(['gz', 'sim', '--versions']), 'ros_distro': env.get('ROS_DISTRO'),
            'opencv_library_links': output(['ldd', str(ROOT/'install/aruco_tracker/lib/aruco_tracker/aruco_tracker')]),
            'time_basis': 'bag receive time uses ROS system time; original sensor/PX4 stamps retained without conversion'}
# Include uncommitted and untracked project files, excluding generated artifacts.
files = output(['git', 'ls-files', '--cached', '--others', '--exclude-standard']).splitlines()
hashes = {}
for rel in files:
    source = ROOT / rel
    if source.is_file():
        dest = out/'source'/rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
        hashes[rel] = hashlib.sha256(source.read_bytes()).hexdigest()
versions['source_sha256'] = hashes
(out/'manifest.json').write_text(json.dumps(versions, indent=2)+'\n')
(out/'project.patch').write_text(output(['git', 'diff', 'HEAD']))
prefix = f'/world/{world}/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/'
topics = [prefix+'image', prefix+'camera_info', '/target_pose', '/target_pose_world',
          '/fmu/out/vehicle_attitude', '/fmu/out/vehicle_local_position_v1',
          '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_land_detected',
          '/fmu/in/trajectory_setpoint', '/fmu/in/vehicle_command', '/rosout', '/clock']
(out/'topics.json').write_text(json.dumps(topics, indent=2)+'\n')
qos = {topic: dict(history='keep_last', depth=50 if topic.endswith('/image') else 100,
                   reliability='best_effort', durability='volatile') for topic in topics}
import yaml
(out/'record_qos.yaml').write_text(yaml.safe_dump(qos))
processes = []
handles = []
def start(cmd, log):
    f = (out/log).open('w'); handles.append(f)
    proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(proc)
    return proc
def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try: proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM); proc.wait(timeout=10)
try:
    sim = start(['bash', 'scripts/run.sh'], 'runner.log')
    for _ in range(120):
        if sim.poll() is not None: raise RuntimeError('SITL failed; see runner.log and logs/')
        log = out/'logs/ros.log'
        if log.exists() and 'Got RegisterExtComponentReply' in log.read_text(): break
        time.sleep(1)
    else: raise RuntimeError('Mode registration timeout')
    bag = start(['ros2', 'bag', 'record', '-o', str(out/'bag'), '--max-cache-size', '268435456',
                 '--qos-profile-overrides-path', str(out/'record_qos.yaml'), *topics], 'recorder.log')
    time.sleep(4)
    if bag.poll() is not None: raise RuntimeError('Recorder failed')
    subprocess.run(['ros2', 'param', 'dump', '/aruco_tracker', '--output-dir', str(out)], env=env, check=True)
    subprocess.run(['ros2', 'param', 'dump', '/precision_land', '--output-dir', str(out)], env=env, check=True)
    flight = start(['python3', 'scripts/test_flight.py', '--speed', str(versions['speed_m_s']), '--output', str(out/'flight.jsonl')], 'flight.log')
    if flight.wait(timeout=150): raise RuntimeError('Flight test failed')
    pose = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{world}/pose/info', '-n', '1'], env=env, capture_output=True, text=True, timeout=10)
    (out/'final-gazebo-poses.txt').write_text(pose.stdout)
    time.sleep(2)
    stop(bag)
    # Preserve the flight controller log generated during this run.
    ulogs = list((px4/'build/px4_sitl_default/rootfs/log').rglob('*.ulg'))
    if ulogs: shutil.copy2(max(ulogs, key=lambda x: x.stat().st_mtime), out/'flight.ulg')
finally:
    for proc in reversed(processes): stop(proc)
    for f in handles: f.close()
subprocess.run(['python3', 'scripts/verify_baseline.py', str(out)], cwd=ROOT, env=env, check=True)
print(f'PASS: {out}', flush=True)
