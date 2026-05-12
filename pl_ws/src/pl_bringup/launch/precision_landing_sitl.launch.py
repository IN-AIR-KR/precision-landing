"""
SITL 정밀착륙 실행 파일

Gazebo 하방 카메라를 ros_gz_bridge로 브리지하고,
V-마커 검출, ArUco 검출, 착륙 컨트롤러를 시작한다.

사용법:
    ros2 launch pl_bringup precision_landing_sitl.launch.py

Gazebo 카메라 토픽 경로:
    /world/precision_landing/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
    ※ PX4 gz_topic -l 명령으로 실제 센서 이름 확인 권장 (camera vs imager)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# Gazebo 카메라 토픽 (PX4 world/model 이름에 따라 변경)
GZ_WORLD = 'precision_landing'
GZ_MODEL = 'x500_mono_cam_down_0'
GZ_LINK = 'camera_link'
GZ_SENSOR = 'camera'   # mono_cam 모델의 센서 이름; 'imager'일 경우 수정

GZ_IMAGE_TOPIC = f'/world/{GZ_WORLD}/model/{GZ_MODEL}/link/{GZ_LINK}/sensor/{GZ_SENSOR}/image'
GZ_INFO_TOPIC = f'/world/{GZ_WORLD}/model/{GZ_MODEL}/link/{GZ_LINK}/sensor/{GZ_SENSOR}/camera_info'


def generate_launch_description():
    pl_cfg = FindPackageShare('pl_nodes')

    return LaunchDescription([
        # ── 카메라 이미지 브리지 (Gazebo → ROS2) ──
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_image_bridge',
            arguments=[
                GZ_IMAGE_TOPIC + '@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            remappings=[(GZ_IMAGE_TOPIC, '/pl/image_raw')],
            output='screen',
        ),

        # ── 카메라 캘리브레이션 브리지 ──
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_cam_info_bridge',
            arguments=[
                GZ_INFO_TOPIC + '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            remappings=[(GZ_INFO_TOPIC, '/pl/camera_info')],
            output='screen',
        ),

        # ── V-마커 검출 노드 ──
        Node(
            package='pl_nodes',
            executable='v_marker_detector',
            name='v_marker_detector_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pl_cfg, 'config', 'v_marker_detector_params.yaml'])
            ],
        ),

        # ── ArUco 검출 노드 ──
        Node(
            package='pl_nodes',
            executable='aruco_detector',
            name='aruco_detector_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pl_cfg, 'config', 'aruco_detector_params.yaml'])
            ],
        ),

        # ── 착륙 컨트롤러 노드 ──
        Node(
            package='pl_nodes',
            executable='landing_controller',
            name='landing_controller_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pl_cfg, 'config', 'landing_controller_params.yaml'])
            ],
        ),
    ])
