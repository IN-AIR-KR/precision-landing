"""ROS JPEG camera transport -> detector -> compressed debug viewer round trip."""
from array import array
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data as qos
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PoseStamped
from pl_camera.image_relay import ImageRelay
from pl_detection.aruco_tracker import ArucoTracker



def test_jpeg_header_detection_and_lazy_viewer():
    rclpy.init(args=['--ros-args', '-p', 'input_transport:=compressed', '-p', 'marker_size:=1.0'])
    encoder = ImageRelay('test_encoder')
    decoder = ImageRelay('test_decoder', parameter_overrides=[
        Parameter('direction', value='decode'), Parameter('input_topic', value='/image_proc/compressed'),
        Parameter('output_topic', value='/test/decoded')])
    detector = ArucoTracker()
    probe = rclpy.create_node('jpeg_test_probe')
    executor = SingleThreadedExecutor()
    for node in (encoder, decoder, detector, probe):
        executor.add_node(node)
    raw = probe.create_publisher(Image, '/hitl/pc/camera/image_raw', qos)
    info = probe.create_publisher(CameraInfo, '/camera/camera_info', qos)
    frame = np.full((600,800,3),255,np.uint8)
    frame[200:400,300:500] = cv2.cvtColor(cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(2),0,200),cv2.COLOR_GRAY2BGR)
    msg = Image(height=600,width=800,encoding='bgr8',step=2400,data=array('B',frame.tobytes()))
    msg.header.stamp.sec=123
    msg.header.stamp.nanosec=456
    msg.header.frame_id='camera_frame'
    calibration = CameraInfo(k=[500.,0.,399.5,0.,500.,299.5,0.,0.,1.],d=[0.]*5)
    poses, decoded, compressed = [], [], []
    probe.create_subscription(PoseStamped,'/target_pose',poses.append,qos)
    probe.create_subscription(CompressedImage,'/camera/image_raw/compressed',compressed.append,qos)
    image_sub = None

    def spin(seconds, publish=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if publish:
                raw.publish(msg)
                info.publish(calibration)
            executor.spin_once(timeout_sec=.01)
            time.sleep(.01)
    try:
        spin(1.5)
        assert poses and compressed
        assert decoder.subscription is None
        assert raw.get_subscription_count()==1  # detector never subscribes to network raw images
        assert poses[-1].header.stamp==msg.header.stamp
        assert compressed[-1].header==msg.header
        assert len(compressed[-1].data)<len(msg.data)/5
        assert abs(poses[-1].pose.position.z-500/199)<.03
        image_sub=probe.create_subscription(Image,'/test/decoded',decoded.append,qos)
        spin(1.5)
        assert decoded and decoded[-1].header==msg.header
        assert (decoded[-1].width,decoded[-1].height)==(800,600)
        probe.destroy_subscription(image_sub)
        image_sub=None
        spin(.6)
        assert decoder.subscription is None
        assert detector.compressed_pub.get_subscription_count()==0
        spin(.4,publish=False)
        count=len(poses)
        detector.image_callback(CompressedImage(data=array('B',b'invalid jpeg')))
        spin(.2,publish=False)
        assert len(poses)==count
        spin(.4)
        assert len(poses)>count  # malformed frame did not stop the detector
    finally:
        executor.shutdown()
        for node in (probe,detector,decoder,encoder):
            node.destroy_node()
        rclpy.shutdown()
