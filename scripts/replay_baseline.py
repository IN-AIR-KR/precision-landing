#!/usr/bin/env python3
"""Paced replay of original bag camera inputs in isolated domain 84.
Wait for each detector debug frame before sending the next, avoiding its depth-1
input queue dropping frames. No PX4 commands or vehicle state are published.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo

p = argparse.ArgumentParser()
p.add_argument('baseline', type=Path)
p.add_argument('--image-topic', default='/camera/image_raw')
p.add_argument('--camera-info-topic', default='/camera/camera_info')
p.add_argument('--output', type=Path)
p.add_argument('--detector-executable', type=Path, help='Optional archived C++ executable for full-input comparison')
p.add_argument('--compare-report', type=Path, help='Compare all detections against an earlier paced replay')
a = p.parse_args()
folder = a.baseline.resolve()
manifest = json.loads((folder/'manifest.json').read_text())
world = manifest['world']
topics=json.loads((folder/'topics.json').read_text())
image_topic=next(t for t in topics if t.endswith('/image') or t.endswith('/image_raw'))
info_topic=next(t for t in topics if t.endswith('/camera_info'))
report=(a.output or Path(__file__).resolve().parents[1]/'.run/replays'/(time.strftime('%Y%m%d-%H%M%S')+'-'+folder.name)).resolve()
report.mkdir(parents=True, exist_ok=False)
def key(msg): return (msg.header.stamp.sec, msg.header.stamp.nanosec)
def values(msg):
    p,q=msg.pose.position,msg.pose.orientation
    return [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
reference = {}
for db in sorted((folder/'bag').glob('*.db3')):
    with sqlite3.connect(db) as con:
        for (raw,) in con.execute("SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name='/target_pose' ORDER BY m.timestamp"):
            msg=deserialize_message(raw,PoseStamped); reference[key(msg)] = values(msg)
assert reference, 'No reference detections'
os.environ['ROS_DOMAIN_ID']='84'
os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(Path(__file__).resolve().parent/'config/replay_fastdds.xml')
rclpy.init(); node=rclpy.create_node('baseline_replay_compare')
received = {}; processed=set(); pose_frames=set()
def receive_pose(msg):
    received[key(msg)]=values(msg)
    pose_frames.add(msg.header.frame_id)
subs=[node.create_subscription(PoseStamped,'/target_pose',receive_pose,qos_profile_sensor_data),
      node.create_subscription(Image,'/image_proc',lambda m: processed.add(key(m)),qos_profile_sensor_data)]
image_pub=node.create_publisher(Image,a.image_topic,qos_profile_sensor_data)
info_pub=node.create_publisher(CameraInfo,a.camera_info_topic,qos_profile_sensor_data)
def spin_until(predicate,timeout):
    end=time.monotonic()+timeout
    while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    return predicate()
frames=0
input_stamps=set()
with (report/'replay-detector.log').open('w') as log:
    command = [str(a.detector_executable.resolve())] if a.detector_executable else ['ros2','run','pl_detection','aruco_tracker']
    detector=subprocess.Popen(command+['--ros-args','-r','__node:=aruco_tracker','-r',f'/camera/image_raw:={a.image_topic}','-r',f'/camera/camera_info:={a.camera_info_topic}','--params-file',str(folder/'aruco_tracker.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    assert spin_until(lambda:image_pub.get_subscription_count()>0 and info_pub.get_subscription_count()>0,10),'Detector input discovery failed'
    # Camera calibration must be delivered before the first image, regardless of
    # discovery/record receive ordering. It is copied unchanged from this bag.
    info=None
    for db in sorted((folder/'bag').glob('*.db3')):
        with sqlite3.connect(db) as con:
            row=con.execute('SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=? ORDER BY m.timestamp LIMIT 1',(info_topic,)).fetchone()
            if row:info=deserialize_message(row[0],CameraInfo);break
    assert info is not None
    for _ in range(10):
        info_pub.publish(info)
        end=time.monotonic()+.05
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    assert spin_until(lambda:info_pub.get_subscription_count()==0,5),'Detector did not accept CameraInfo'
    for db in sorted((folder/'bag').glob('*.db3')):
        with sqlite3.connect(db) as con:
            for (raw,) in con.execute('SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=? ORDER BY m.timestamp',(image_topic,)):
                assert detector.poll() is None,'Detector exited'
                msg=deserialize_message(raw,Image);stamp=key(msg);input_stamps.add(stamp)
                for attempt in range(3):
                    image_pub.publish(msg)
                    if spin_until(lambda:stamp in processed,2):break
                else:raise RuntimeError(f'Detector did not process frame {stamp}')
                # Pose is published before the processed image, but DDS topics may
                # arrive in either order. Allow its callback to catch up.
                if stamp in reference:spin_until(lambda:stamp in received,.2)
                frames+=1
                if frames%200==0: print(f'Replayed {frames} images',flush=True)
    spin_until(lambda:False,.5)
finally:
    if detector.poll() is None:
        os.killpg(detector.pid,signal.SIGINT)
        try:detector.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(detector.pid,signal.SIGTERM);detector.wait(timeout=5)
    node.destroy_node();rclpy.shutdown()
matched=reference.keys() & received.keys()
comparable=reference.keys() & input_stamps
max_error=max((max(abs(x-y) for x,y in zip(reference[k],received[k])) for k in matched),default=float('inf'))
result=dict(reference_count=len(reference),replayed_count=len(received),matched=len(matched),images_replayed=frames,
            coverage=len(matched)/len(reference),comparable_reference_count=len(comparable),
            reference_without_image=len(reference)-len(comparable),
            comparable_coverage=len(matched)/len(comparable) if comparable else 0.,
            max_component_error=max_error,domain=84,pose_frames=sorted(pose_frames),
            input_topics=[a.image_topic,a.camera_info_topic],
            detector_sha256=hashlib.sha256((a.detector_executable or Path(__file__).resolve().parents[1]/'install/pl_detection/lib/pl_detection/aruco_tracker').read_bytes()).hexdigest(),
            note='Original images replayed one at a time with original headers; no PX4 commands were published.')
if not a.detector_executable:
    import cv2, numpy, pl_detection
    result['python_runtime'] = {'opencv': cv2.__version__, 'numpy': numpy.__version__, 'cv2_path': cv2.__file__}
    result['detector_source_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(pl_detection.__file__).parent.glob('*.py')}
(report/'poses.json').write_text(json.dumps({f'{k[0]}:{k[1]}': v for k,v in received.items()}, indent=2)+'\n')
result['passed']=bool(comparable) and matched==comparable and max_error<1e-6
if a.compare_report:
    other=json.loads((a.compare_report/'poses.json').read_text())
    current={f'{k[0]}:{k[1]}':v for k,v in received.items()}
    common=other.keys() & current.keys()
    error=max((max(abs(x-y) for x,y in zip(other[k],current[k])) for k in common),default=float('inf'))
    result['full_input_comparison']={'reference':str(a.compare_report),'reference_detections':len(other),'current_detections':len(current),'same_detection_stamps':other.keys()==current.keys(),'max_component_error':error}
    result['passed'] = result['passed'] and other.keys()==current.keys() and error<1e-6
(report/'replay-verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
print(f'Report: {report}')
assert result['passed'],'Replay differs from baseline'
