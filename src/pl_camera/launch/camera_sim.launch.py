"""Gazebo camera adapter: expose common ROS camera inputs."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    prefix = f"/world/{arg('world')}/model/{arg('model')}/link/{arg('link')}/sensor/{arg('sensor')}/"
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge', output='screen',
        arguments=[prefix+'image@sensor_msgs/msg/Image[gz.msgs.Image',
                   prefix+'camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[(prefix+'image', arg('image_topic')),
                    (prefix+'camera_info', arg('camera_info_topic'))])
    # This ros_gz revision ignores the global CLI lazy flag. A separate YAML
    # bridge is necessary: config_file is read only when there are no CLI bridges.
    debug_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='debug_image_bridge', output='screen', parameters=[{
            'config_file': str(Path(get_package_share_directory('pl_camera')) / 'config/debug_bridge.yaml'),
            'subscription_heartbeat': 200,
            'qos_overrides./image_proc.subscription.reliability': 'best_effort'}])
    return [debug_bridge, RegisterEventHandler(OnProcessExit(target_action=bridge,
                on_exit=[EmitEvent(event=Shutdown(reason='Camera bridge exited'))])), bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='aruco_moving'),
        DeclareLaunchArgument('model', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('link', default_value='camera_link'),
        DeclareLaunchArgument('sensor', default_value='camera'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        OpaqueFunction(function=setup)])
