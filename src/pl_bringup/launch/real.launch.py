"""Real flight profile with Jetson CSI capture."""
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pl_bringup.parameters import control_parameters


def setup(context):
    config_path = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    config = yaml.safe_load(config_path.read_text())
    controller = control_parameters(config['control'])
    calibration = config['camera']['calibration']
    if not calibration:
        raise RuntimeError('Set camera.calibration in real.yaml to the measured camera calibration.')

    uart = config['px4']['uart']
    device = uart['device']
    if not Path(device).exists():
        raise RuntimeError(f'UART device does not exist: {device}')
    nodes = [
        ExecuteProcess(cmd=['MicroXRCEAgent', 'serial', '-D', device, '-b', str(uart['baud'])],
                       output='screen'),
        Node(package='pl_detection', executable='aruco_tracker', name='aruco_tracker', output='screen',
             parameters=[dict(config['detection'], optical_frame='camera_optical_frame')]),
        Node(package='pl_control', executable='precision_land', name='precision_land', output='screen',
             parameters=[controller]),
    ]
    monitor = Node(package='pl_viz', executable='monitor', output='screen',
                   parameters=[{k: v for k, v in config['viz'].items() if k != 'gazebo'}])
    camera = Node(package='pl_camera', executable='csi_camera', name='csi_camera',
                  output='screen', parameters=[{'calibration_json': json.dumps(calibration)}])
    required = [camera, *nodes]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='A required process exited'))],
    )) for node in required]
    return [*handlers, monitor, *required]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(
            Path(get_package_share_directory('pl_bringup')) / 'config/real.yaml')),
        OpaqueFunction(function=setup),
    ])
