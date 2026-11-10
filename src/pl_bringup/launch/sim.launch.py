"""Simulation process ownership: this launch owns the sole UDP DDS Agent."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    cfg = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    world = LaunchConfiguration('world').perform(context) or cfg['bridge']['world']
    size = LaunchConfiguration('marker_size').perform(context)
    detection = dict(cfg['detection'], optical_frame=cfg['camera']['optical_frame'])
    if size:
        detection['marker_size'] = float(size)
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('pl_camera'))/'launch/camera_sim.launch.py')),
        launch_arguments={'world': world, 'model': cfg['bridge']['model'],
                          'link': cfg['bridge']['link'], 'sensor': cfg['bridge']['sensor'],
                          'image_topic': cfg['camera']['image_topic'],
                          'camera_info_topic': cfg['camera']['camera_info_topic']}.items())
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', str(cfg['agent']['port'])], output='screen')
    nodes = [agent,
        Node(package='pl_detection', executable='aruco_tracker', name='aruco_tracker', output='screen', parameters=[detection],
             remappings=[('/camera/image_raw', cfg['camera']['image_topic']),
                         ('/camera/camera_info', cfg['camera']['camera_info_topic'])]),
        Node(package='pl_control', executable='precision_land', name='precision_land', output='screen', parameters=[cfg['control']])]
    if cfg['viz']['gazebo']:
        nodes.append(Node(package='pl_viz', executable='tag_pose_visualizer', name='tag_pose_visualizer', output='screen'))
    # A dead Agent, bridge or flight node must be visible as a failed bringup.
    monitor = Node(package='pl_viz', executable='monitor', output='screen',
                   parameters=[{k: v for k, v in cfg['viz'].items() if k != 'gazebo'}])
    return [camera, monitor] + nodes + [RegisterEventHandler(OnProcessExit(target_action=n, on_exit=[EmitEvent(event=Shutdown(reason='A required process exited'))])) for n in nodes]


def generate_launch_description():
    default = str(Path(get_package_share_directory('pl_bringup'))/'config/sim.yaml')
    return LaunchDescription([DeclareLaunchArgument('config', default_value=default),
                              DeclareLaunchArgument('world', default_value=''),
                              DeclareLaunchArgument('marker_size', default_value=''), OpaqueFunction(function=setup)])
