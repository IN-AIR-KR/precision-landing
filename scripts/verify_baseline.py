#!/usr/bin/env python3
"""Validate bag contents, flight-state evidence and physical landing location."""
import json
import math
from pathlib import Path
import re
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

folder = Path(sys.argv[1]).resolve()
manifest = json.loads((folder/'manifest.json').read_text())
required = json.loads((folder/'topics.json').read_text())
counts = {}; landed = disarmed = airborne = False; images = {}; camera_info = {}
for db in sorted((folder/'bag').glob('*.db3')):
    con = sqlite3.connect(db)
    for ident, name, typ in con.execute('SELECT id,name,type FROM topics'):
        count = con.execute('SELECT count(*) FROM messages WHERE topic_id=?', (ident,)).fetchone()[0]
        counts[name] = counts.get(name, 0)+count
        if typ in ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CameraInfo']:
            raw = con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1', (ident,)).fetchone()
            if raw:
                msg = deserialize_message(raw[0], get_message(typ))
                if typ == 'sensor_msgs/msg/Image': images = dict(width=msg.width, height=msg.height, encoding=msg.encoding)
                else: camera_info = dict(width=msg.width, height=msg.height, k=list(msg.k), d=list(msg.d))
        if name in ['/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_land_detected']:
            for (raw,) in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (ident,)):
                msg = deserialize_message(raw, get_message(typ))
                if hasattr(msg, 'arming_state'):
                    airborne |= msg.arming_state == 2
                    disarmed |= airborne and msg.arming_state == 1
                else: landed |= msg.landed
    con.close()
missing = [t for t in required if not counts.get(t)]
assert not missing, f'Missing/empty topics: {missing}'
assert images['width'] == camera_info['width'] and images['height'] == camera_info['height'] and camera_info['k'][0]>0
roslog = (folder/'logs/ros.log').read_text()
assert all(f'Switching to {s}' in roslog for s in ['Search', 'Approach', 'Descend', 'Finished'])
assert 'Disarmed by landing' in (folder/'logs/px4.log').read_text()
assert airborne and landed and disarmed
poses = (folder/'final-gazebo-poses.txt').read_text()
def xy(name):
    match = re.search(r'name: "'+re.escape(name)+r'"\s+id: \d+\s+position\s*\{([^}]*)}', poses)
    assert match, f'No Gazebo pose for {name}'
    return [float(re.search(r'\b'+axis+r': ([^\s]+)', match[1])[1]) if re.search(r'\b'+axis+r': ([^\s]+)', match[1]) else 0. for axis in ['x','y']]
drone = xy('x500_mono_cam_down_0')
target = xy('aruco_moving_target') if manifest['scenario']=='moving' else xy('arucotag')
error = math.dist(drone, target)
assert error < .5, f'Landing error {error:.3f} m exceeds baseline limit'
if manifest['scenario']=='moving': assert target[0]>2., 'Target did not move'
result = dict(passed=True, scenario=manifest['scenario'], message_counts=counts, image=images,
              camera_info=camera_info, drone_xy=drone, target_xy=target, landing_error_m=error)
(folder/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
