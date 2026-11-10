"""Stage-0 hardware boundary. CSI capture is intentionally deferred to stage 1."""
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
    if not cfg['camera']['calibration_verified']:
        raise RuntimeError('Hardware profile is not commissioned: verify external Image/CameraInfo calibration and camera mounting first. CSI capture is stage 1.')
    device = cfg['agent']['device']
    if not Path(device).exists():
        raise RuntimeError(f'UART device does not exist: {device}')
    prefix = f"/world/{cfg['bridge']['world']}/model/{cfg['bridge']['model']}/link/camera_link/sensor/imager/"
    nodes = [ExecuteProcess(cmd=['MicroXRCEAgent', 'serial', '-D', device, '-b', str(cfg['agent']['baud'])], output='screen'),
        Node(package='aruco_tracker', executable='aruco_tracker', name='aruco_tracker', output='screen',
             parameters=[dict(cfg['detection'], world_name=cfg['bridge']['world'])],
             remappings=[(prefix+'image',cfg['camera']['image_topic']), (prefix+'camera_info',cfg['camera']['camera_info_topic'])]),
        Node(package='precision_land', executable='precision_land', name='precision_land', output='screen', parameters=[cfg['control']])]
    return nodes + [RegisterEventHandler(OnProcessExit(target_action=n, on_exit=[EmitEvent(event=Shutdown(reason='A required process exited'))])) for n in nodes]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config', default_value=str(Path(get_package_share_directory('pl_bringup'))/'config/hardware.yaml')),OpaqueFunction(function=setup)])
