"""Run the Jetson camera, detector, and HUD using the real flight settings."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from pl_bringup.parameters import control_parameters
from pl_bringup.real_vision import vision_nodes


def setup(context):
    path = Path(LaunchConfiguration('config').perform(context)).expanduser()
    config = yaml.safe_load(path.read_text())
    nodes = vision_nodes(config, control_parameters(config['control']))
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='A camera test node exited'))],
        ))
        for node in nodes
    ]
    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        *handlers, *nodes,
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/pl/monitor/image'],
            condition=IfCondition(LaunchConfiguration('view')),
            output='screen'),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('pl_bringup')) / 'config/real.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('domain_id', default_value='0',
                              description='ROS domain for the camera test'),
        DeclareLaunchArgument('view', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
