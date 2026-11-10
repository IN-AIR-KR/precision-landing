"""Exercise real ROS discovery, independent outputs and stalled-camera HUD updates."""
import importlib.util
from pathlib import Path
import time
import json
from array import array

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data as qos
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from px4_msgs.msg import BatteryStatus, SensorGps, VehicleLocalPosition
from geometry_msgs.msg import PoseStamped
from pl_detection.aruco_tracker import ArucoTracker

spec = importlib.util.spec_from_file_location('landing_monitor', Path(__file__).parents[1] / 'monitor.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


@pytest.mark.parametrize('transport', ['raw', 'compressed'])
def test_demand_and_camera_loss(transport, monkeypatch, tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'transport:={transport}'])
    monitor, detector = mod.Monitor(), ArucoTracker()
    probe = rclpy.create_node('monitor_test_probe')
    executor = SingleThreadedExecutor()
    for node in (monitor, detector, probe):
        executor.add_node(node)
    encoded, drawings, poses = [], [], []
    encode = cv2.imencode
    monkeypatch.setattr(cv2, 'imencode', lambda *a, **kw: (encoded.append(a[1].shape), encode(*a, **kw))[1])
    process = detector.detector.process
    monkeypatch.setattr(detector.detector, 'process', lambda *a, **kw: (drawings.append(kw.get('draw', True)), process(*a, **kw))[1])
    image_pub = probe.create_publisher(Image, '/camera/image_raw', qos)
    control_pub = probe.create_publisher(String, '/pl/control/status', qos)
    gps_pub = probe.create_publisher(SensorGps, mod.px4_topic('vehicle_gps_position', SensorGps), qos)
    battery_pub = probe.create_publisher(BatteryStatus, mod.px4_topic('battery_status', BatteryStatus), qos)
    velocity_pub = probe.create_publisher(VehicleLocalPosition, mod.px4_topic('vehicle_local_position', VehicleLocalPosition), qos)
    gps = SensorGps(fix_type=3, satellites_used=14)
    battery = BatteryStatus(connected=True, voltage_v=15.2, remaining=.75)
    velocity = VehicleLocalPosition(v_xy_valid=True, v_z_valid=True, vx=3., vy=4., vz=-.5)
    info = CameraInfo(k=[500.,0.,400.,0.,500.,300.,0.,0.,1.], d=[0.]*5)
    detector.info_callback(info)
    frame = np.full((600,800,3),255,np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(2),0,200)
    frame[200:400,300:500] = cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
    msg = Image(height=600,width=800,encoding='bgr8',step=2400,data=array('B',frame.tobytes()))
    pose_sub = probe.create_subscription(PoseStamped, '/target_pose', poses.append, qos)
    subs = [pose_sub]
    camera_on = True
    phase = 'Approach'

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            if camera_on:
                msg.header.stamp = probe.get_clock().now().to_msg()
                image_pub.publish(msg)
            gps_pub.publish(gps)
            battery_pub.publish(battery)
            velocity_pub.publish(velocity)
            control_pub.publish(String(data=json.dumps({'state':phase,'active':True,'target_lost':False,'mode_id':23})))
            executor.spin_once(timeout_sec=.01)
            time.sleep(.015)

    def subscribe(cls, topic, values):
        sub = probe.create_subscription(cls, topic, values.append, qos)
        subs.append(sub)
        return sub

    def remove(sub):
        probe.destroy_subscription(sub)
        subs.remove(sub)

    try:
        spin(1)
        assert poses and not monitor.inputs and not any(drawings) and not encoded
        states=[]
        status_sub=subscribe(String,'/pl/monitor/status',states)
        spin(1)
        state=json.loads(states[-1].data)
        assert state['gps']['value']=={'fix_type':3, 'satellites':14}
        assert state['battery']['value']['remaining_pct']==75
        assert state['velocity']['value']=={'horizontal_m_s':5., 'vertical_m_s':-.5}
        assert states and 'control' in monitor.inputs and 'image' not in monitor.inputs
        assert not any(drawings) and not encoded
        pose_count=len(poses)
        images=[]
        raw_sub=subscribe(Image,'/pl/monitor/image',images)
        spin(2)
        assert images and any(drawings) and len(poses)>pose_count
        if transport=='raw':
            assert not encoded
        else:
            assert encoded and all(shape[:2]==(600,800) for shape in encoded)
        cv2.imwrite(str(tmp_path/'monitor.png'),mod.to_bgr(images[-1]))
        remove(raw_sub)
        spin(.8)
        assert 'image' not in monitor.inputs
        encoded.clear()
        drawings.clear()
        spin(.4)
        assert not encoded and not any(drawings)
        jpegs=[]
        jpeg_sub=subscribe(CompressedImage,'/pl/monitor/image/compressed',jpegs)
        spin(1.5)
        assert jpegs and cv2.imdecode(np.frombuffer(jpegs[-1].data,np.uint8),1).shape==(600,800,3)
        assert any(shape[:2]==(600,800) for shape in encoded)
        both_images=[]
        both_sub=subscribe(Image,'/pl/monitor/image',both_images)
        spin(.8)
        assert both_images
        remove(both_sub)
        camera_on=False
        phase='Descend'
        count=len(jpegs)
        spin(2.5)
        state=json.loads(states[-1].data)
        assert state['image']['health']=='stale'
        assert state['detection']['health']=='stale'
        assert state['control']['health']=='ok' and state['control']['value']['state']=='Descend'
        assert len(jpegs)>count
        velocity.v_xy_valid=False
        velocity.v_z_valid=False
        battery.remaining=-1.
        battery.voltage_v=float('nan')
        spin(.5)
        state=json.loads(states[-1].data)
        assert state['velocity']['value']=={'horizontal_m_s':None, 'vertical_m_s':None}
        assert state['battery']['value']['remaining_pct'] is None
        assert state['battery']['value']['voltage_v'] is None
        remove(jpeg_sub)
        remove(status_sub)
        spin(1)
        assert not monitor.inputs
        assert detector.image_pub.get_subscription_count()==0
        assert detector.compressed_pub.get_subscription_count()==0
        assert detector.status_pub.get_subscription_count()==0
        camera_on=True
        encoded.clear()
        drawings.clear()
        count=len(poses)
        spin(.5)
        assert len(poses)>count and not any(drawings) and not encoded
    finally:
        executor.shutdown()
        for node in (probe, detector, monitor):
            node.destroy_node()
        rclpy.shutdown()
