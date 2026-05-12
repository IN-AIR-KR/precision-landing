"""
실제 하드웨어 정밀착륙 실행 파일

usb_cam 패키지로 하방 USB 카메라를 구동하고,
V-마커 검출, ArUco 검출, 착륙 컨트롤러를 시작한다.

사용법:
    ros2 launch pl_bringup precision_landing_real.launch.py

파라미터:
    video_device  - 카메라 장치 경로 (기본: /dev/video0)
    image_width   - 해상도 가로 (기본: 1280)
    image_height  - 해상도 세로 (기본: 960)

참고: https://github.com/ros-drivers/usb_cam
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pl_cfg = FindPackageShare('pl_nodes')

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width',  default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='960'),

        # ── USB 카메라 노드 ──
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_node',
            output='screen',
            parameters=[{
                'video_device':   LaunchConfiguration('video_device'),
                'image_width':    LaunchConfiguration('image_width'),
                'image_height':   LaunchConfiguration('image_height'),
                'framerate':      30.0,
                'pixel_format':   'yuyv',
                'camera_name':    'downward_cam',
                'camera_info_url': '',
            }],
            remappings=[
                ('image_raw',   '/pl/image_raw'),
                ('camera_info', '/pl/camera_info'),
            ],
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
