"""Jetson CSI camera adapter: expose the same ROS camera inputs as the Gazebo bridge.

Mirrors camera_sim.launch.py so pl_bringup can swap the frame source without
touching pl_detection. Defaults come from config/csi_camera.yaml; any of them
can be overridden on the command line.

    ros2 launch pl_camera camera_csi.launch.py
    ros2 launch pl_camera camera_csi.launch.py width:=1280 height:=720 fps:=60
    ros2 launch pl_camera camera_csi.launch.py camera_info_url:=/path/to/imx219.yaml
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULTS = Path(get_package_share_directory('pl_camera')) / 'config/csi_camera.yaml'
# Launch arguments map one-to-one onto the node's parameters; the YAML supplies
# the defaults so the file stays the single place to edit them.
INT_PARAMS = ('sensor_id', 'width', 'height', 'flip_method', 'wbmode')
FLOAT_PARAMS = ('fps',)


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config = yaml.safe_load(DEFAULTS.read_text())['csi_camera']['ros__parameters']
    parameters = {}
    for key in config:
        value = arg(key)
        if key in INT_PARAMS:
            parameters[key] = int(value)
        elif key in FLOAT_PARAMS:
            parameters[key] = float(value)
        else:
            parameters[key] = value
    camera = Node(package='pl_camera', executable='csi_camera', name='csi_camera',
                  output='screen', parameters=[parameters])
    # A camera that dies is not recoverable by itself: Argus holds the sensor
    # until the process goes away, so bring the launch down with it.
    return [RegisterEventHandler(OnProcessExit(
        target_action=camera,
        on_exit=[EmitEvent(event=Shutdown(reason='CSI camera exited'))])), camera]


def generate_launch_description():
    config = yaml.safe_load(DEFAULTS.read_text())['csi_camera']['ros__parameters']
    return LaunchDescription(
        [DeclareLaunchArgument(key, default_value=str(value))
         for key, value in config.items()] + [OpaqueFunction(function=setup)])
