"""Simulation process ownership: this launch owns the sole UDP DDS Agent."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    cfg = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    world = LaunchConfiguration('world').perform(context) or cfg['bridge']['world']
    size = LaunchConfiguration('marker_size').perform(context)
    detection = dict(cfg['detection'], world_name=world)
    if size:
        detection['marker_size'] = float(size)
    prefix = f"/world/{world}/model/{cfg['bridge']['model']}/link/camera_link/sensor/imager/"
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', str(cfg['agent']['port'])], output='screen')
    nodes = [agent,
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge', output='screen', arguments=[
            prefix+'image@sensor_msgs/msg/Image[gz.msgs.Image',
            prefix+'camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/image_proc@sensor_msgs/msg/Image]gz.msgs.Image']),
        Node(package='aruco_tracker', executable='aruco_tracker', name='aruco_tracker', output='screen', parameters=[detection]),
        Node(package='precision_land', executable='precision_land', name='precision_land', output='screen', parameters=[cfg['control']])]
    if cfg['viz']['gazebo']:
        nodes.append(Node(package='precision_land_viz', executable='tag_pose_visualizer', name='tag_pose_visualizer', output='screen'))
    # A dead Agent, bridge or flight node must be visible as a failed bringup.
    return nodes + [RegisterEventHandler(OnProcessExit(target_action=n, on_exit=[EmitEvent(event=Shutdown(reason='A required process exited'))])) for n in nodes]


def generate_launch_description():
    default = str(Path(get_package_share_directory('pl_bringup'))/'config/sim.yaml')
    return LaunchDescription([DeclareLaunchArgument('config', default_value=default),
                              DeclareLaunchArgument('world', default_value=''),
                              DeclareLaunchArgument('marker_size', default_value=''), OpaqueFunction(function=setup)])
