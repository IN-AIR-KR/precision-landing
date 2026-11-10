"""Gazebo camera adapter: expose common ROS camera inputs."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    prefix = f"/world/{arg('world')}/model/{arg('model')}/link/camera_link/sensor/imager/"
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge', output='screen',
        arguments=[prefix+'image@sensor_msgs/msg/Image[gz.msgs.Image',
                   prefix+'camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/image_proc@sensor_msgs/msg/Image]gz.msgs.Image'],
        remappings=[(prefix+'image', arg('image_topic')),
                    (prefix+'camera_info', arg('camera_info_topic'))])
    return [RegisterEventHandler(OnProcessExit(target_action=bridge,
                on_exit=[EmitEvent(event=Shutdown(reason='Camera bridge exited'))])), bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='aruco_moving'),
        DeclareLaunchArgument('model', default_value='x500_mono_cam_down_0'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        OpaqueFunction(function=setup)])
