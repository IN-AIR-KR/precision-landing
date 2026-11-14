"""Jetson half of distributed SITL: JPEG detection, control and local HUD."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pl_bringup.parameters import control_parameters


def setup(context):
    config = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    controller = control_parameters(config['control'])
    world = LaunchConfiguration('world').perform(context)
    if world not in ('aruco', 'aruco_moving'):
        raise ValueError('world must be aruco or aruco_moving')
    detector = Node(package='pl_detection', executable='aruco_tracker', name='aruco_tracker', output='screen',
        parameters=[dict(config['detection'], optical_frame='camera_frame',
                         input_transport='compressed', marker_size=.5 if world == 'aruco' else 1.0)])
    control = Node(package='pl_control', executable='precision_land', name='precision_land', output='screen',
                   parameters=[controller])
    # Raw stays on Jetson. Only the final overlay is encoded for the PC viewer.
    monitor = Node(package='pl_viz', executable='monitor', output='screen',
                   parameters=[dict({k: v for k, v in config['viz'].items() if k != 'gazebo'}, transport='raw')])
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Required Jetson process exited'))])) for node in (detector, control)]
    return [*handlers, detector, control, monitor]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '42')),
        DeclareLaunchArgument('config', default_value=str(Path(get_package_share_directory('pl_bringup'))/'config/sim.yaml')),
        DeclareLaunchArgument('world', default_value='aruco_moving'),
        OpaqueFunction(function=setup),
    ])
