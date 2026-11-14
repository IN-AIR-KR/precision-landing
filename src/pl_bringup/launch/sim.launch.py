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
from launch.conditions import IfCondition
from launch_ros.actions import Node
from pl_bringup.parameters import control_parameters


def setup(context):
    config = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    controller = control_parameters(config['control'])
    world = LaunchConfiguration('world').perform(context) or 'aruco_moving'
    size = LaunchConfiguration('marker_size').perform(context)
    detection = dict(config['detection'], optical_frame='camera_frame')
    if size:
        detection['marker_size'] = float(size)
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('pl_camera'))/'launch/camera_sim.launch.py')),
        launch_arguments={'world': world}.items())
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', str(config['px4']['udp']['port'])],
                           output='screen', condition=IfCondition(LaunchConfiguration('start_agent')))
    nodes = [agent,
        Node(package='pl_detection', executable='aruco_tracker', name='aruco_tracker', output='screen', parameters=[detection]),
        Node(package='pl_control', executable='precision_land', name='precision_land', output='screen',
             parameters=[controller])]
    viz = LaunchConfiguration('enable_gazebo_viz').perform(context)
    enable_gazebo_viz = viz == 'true' if viz else config['viz']['gazebo']
    if enable_gazebo_viz:
        nodes.append(Node(package='pl_viz', executable='tag_pose_visualizer', name='tag_pose_visualizer', output='screen'))
    # A dead Agent, bridge or flight node must be visible as a failed bringup.
    monitor = Node(package='pl_viz', executable='monitor', output='screen',
                   parameters=[{k: v for k, v in config['viz'].items() if k != 'gazebo'}])
    return [camera, monitor] + nodes + [RegisterEventHandler(OnProcessExit(target_action=n, on_exit=[EmitEvent(event=Shutdown(reason='A required process exited'))])) for n in nodes]


def generate_launch_description():
    default = str(Path(get_package_share_directory('pl_bringup'))/'config/sim.yaml')
    return LaunchDescription([DeclareLaunchArgument('config', default_value=default),
                              DeclareLaunchArgument('world', default_value=''),
                              DeclareLaunchArgument('marker_size', default_value=''),
                              DeclareLaunchArgument('start_agent', default_value='true', choices=['true', 'false']),
                              DeclareLaunchArgument('enable_gazebo_viz', default_value='', choices=['', 'true', 'false']),
                              OpaqueFunction(function=setup)])
