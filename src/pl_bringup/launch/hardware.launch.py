"""Hardware profile with external camera input or optional Jetson CSI capture."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    config = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    if not (config['camera']['calibration_verified'] and config['camera']['extrinsics_verified']):
        raise RuntimeError('Hardware profile is not commissioned: verify camera calibration and mounting first.')
    device = config['agent']['device']
    if not Path(device).exists():
        raise RuntimeError(f'UART device does not exist: {device}')
    nodes = [ExecuteProcess(cmd=['MicroXRCEAgent', 'serial', '-D', device, '-b', str(config['agent']['baud'])], output='screen'),
        Node(package='pl_detection', executable='aruco_tracker', name='aruco_tracker', output='screen',
             parameters=[dict(config['detection'], optical_frame=config['camera']['optical_frame'])],
             remappings=[('/camera/image_raw',config['camera']['image_topic']), ('/camera/camera_info',config['camera']['camera_info_topic'])]),
        Node(package='pl_control', executable='precision_land', name='precision_land', output='screen', parameters=[config['control']])]
    monitor = Node(package='pl_viz', executable='monitor', output='screen',
                   parameters=[{k: v for k, v in config['viz'].items() if k != 'gazebo'}])
    cameras = []
    if LaunchConfiguration('use_csi').perform(context) == 'true':
        calibration = LaunchConfiguration('camera_info_url').perform(context)
        if not calibration.strip():
            raise RuntimeError('CSI capture requires camera_info_url pointing to the verified calibration.')
        camera_launch = Path(get_package_share_directory('pl_camera')) / 'launch/camera_csi.launch.py'
        cameras.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(camera_launch)),
            launch_arguments={
                'image_topic': config['camera']['image_topic'],
                'camera_info_topic': config['camera']['camera_info_topic'],
                'frame_id': config['camera']['optical_frame'],
                'camera_info_url': calibration,
            }.items(),
        ))
    return cameras + [monitor] + nodes + [RegisterEventHandler(OnProcessExit(target_action=n, on_exit=[EmitEvent(event=Shutdown(reason='A required process exited'))])) for n in nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(
            Path(get_package_share_directory('pl_bringup')) / 'config/hardware.yaml')),
        DeclareLaunchArgument('use_csi', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        OpaqueFunction(function=setup),
    ])
