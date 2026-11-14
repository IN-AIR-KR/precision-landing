"""Jetson CSI camera adapter: expose the same ROS camera inputs as the Gazebo bridge.

The camera node owns the defaults. Launch arguments override them when supplied.

    ros2 launch pl_camera camera_csi.launch.py
    ros2 launch pl_camera camera_csi.launch.py width:=1280 height:=720 fps:=60
    ros2 launch pl_camera camera_csi.launch.py camera_info_url:=/path/to/imx219.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INT_PARAMS = ('sensor_id', 'width', 'height', 'flip_method', 'wbmode')
FLOAT_PARAMS = ('fps',)
STRING_PARAMS = ('encoding', 'image_topic', 'camera_info_topic', 'frame_id',
                 'camera_info_url', 'exposure_time_range', 'gain_range')
PARAMETERS = INT_PARAMS + FLOAT_PARAMS + STRING_PARAMS


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    parameters = {}
    for key in PARAMETERS:
        value = arg(key)
        if not value:
            continue
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
    return LaunchDescription(
        [DeclareLaunchArgument(key, default_value='') for key in PARAMETERS] + [OpaqueFunction(function=setup)])
