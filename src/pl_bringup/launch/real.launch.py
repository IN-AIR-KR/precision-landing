"""Real flight profile with Jetson CSI capture."""
import os
from pathlib import Path
from shutil import which
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler,
                            EmitEvent, SetEnvironmentVariable, ExecuteProcess, LogInfo)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pl_bringup.parameters import control_parameters
from pl_bringup.recording import recorder, recording_arguments
from pl_bringup.real_vision import vision_nodes


def streaming_actions(stream_url):
    if not stream_url:
        return []
    actions = []
    if stream_url == 'local':
        try:
            config = Path(get_package_share_directory('pl_streaming')) / 'mediamtx.yml'
        except PackageNotFoundError:
            return [LogInfo(msg='HUD streaming disabled: build the pl_streaming package.')]
        binary = which('mediamtx')
        if binary is None and os.environ.get('PRECISION_ROOT'):
            binary = which(str(Path(os.environ['PRECISION_ROOT']) / '.deps/hud/mediamtx'))
        if binary is None or not config.is_file():
            return [LogInfo(msg='HUD streaming disabled: MediaMTX executable or configuration is missing.')]
        actions.append(ExecuteProcess(
            cmd=[binary, str(config)], cwd=launch_config.log_dir, name='mediamtx', output='screen'))
        stream_url = 'rtsp://127.0.0.1:8554/hud'
    actions.append(Node(
        package='pl_streaming', executable='hud_streamer', output='screen',
        parameters=[{'url': stream_url}]))
    return actions


def setup(context):
    config_path = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    config_yaml = config_path.read_text(encoding='utf-8')
    config = yaml.safe_load(config_yaml)
    controller = control_parameters(config['control'])
    camera, detection, monitor = vision_nodes(config, controller)
    control_node = Node(package='pl_control', executable='precision_land', name='precision_land',
                        output='screen', parameters=[controller])
    stream_url = LaunchConfiguration('stream_url').perform(context).strip()
    required = [camera, detection, control_node]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='A required process exited'))],
    )) for node in required]
    return [SetEnvironmentVariable('ROS_DOMAIN_ID', str(config['px4'].get('domain_id', 0))),
            *handlers, monitor, *required, recorder('real', config_yaml=config_yaml), *streaming_actions(stream_url)]


def generate_launch_description():
    return LaunchDescription([
        *recording_arguments(),
        DeclareLaunchArgument('config', default_value=str(
            Path(get_package_share_directory('pl_bringup')) / 'config/real.yaml')),
        DeclareLaunchArgument('stream_url', default_value='local',
                              description='local starts MediaMTX, an RTSP URL uses an existing server, '
                                          'and an empty value disables streaming'),
        OpaqueFunction(function=setup),
    ])
