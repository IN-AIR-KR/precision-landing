"""PC half of Jetson-in-the-loop SITL: simulation, DDS, JPEG and viewer."""
import os
from pathlib import Path
import socket
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    root, px4 = Path(arg('workspace')).expanduser().resolve(), Path(arg('px4_dir')).expanduser().resolve()
    config = yaml.safe_load(Path(arg('config')).read_text())
    world = arg('world')
    if world not in ('aruco', 'aruco_moving'):
        raise ValueError('world must be aruco or aruco_moving')
    binary = px4/'build/px4_sitl_default/bin/px4'
    gz_env = px4/'build/px4_sitl_default/rootfs/gz_env.sh'
    world_file = root/'simulation/gz/worlds'/f'{world}.sdf'
    if not world_file.is_file():
        world_file = px4/'Tools/simulation/gz/worlds'/f'{world}.sdf'
    for path in (binary, gz_env, world_file, root/'simulation/gz/models'):
        if not path.exists():
            raise RuntimeError(f'Missing {path}; source scripts/env.sh and run scripts/setup.sh hitl_pc first')
    # Do not connect this controller to a different user's running simulator.
    if subprocess.run(['pgrep', '-x', 'px4'], stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('Another PX4 is running. Stop it before starting hitl_pc.')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind(('0.0.0.0', int(config['px4']['udp']['port'])))
        except OSError as exc:
            raise RuntimeError('DDS Agent UDP port is already in use; run only one Agent on the PC') from exc
    runtime = Path(arg('run_dir')).expanduser().resolve()/'px4-rootfs'
    runtime.mkdir(parents=True, exist_ok=True)
    env = {
        'GZ_PARTITION': os.environ.get('GZ_PARTITION', 'precision_landing'),
        'GZ_SIM_RESOURCE_PATH': str(root/'simulation/gz/models') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        'PX4_GZ_MODELS': str(px4/'Tools/simulation/gz/models'),
        'PX4_GZ_WORLDS': str(world_file.parent),
    }
    # Source PX4's generated plugin/model environment without evaluating user paths as shell code.
    def gz_command(*command):
        return ['bash', '-c', 'source "$1"; shift; exec "$@"', '--', str(gz_env), *command]
    gazebo = ExecuteProcess(cmd=gz_command('gz', 'sim', '-r', '-s', str(world_file)), additional_env=env, output='screen')
    gui = ExecuteProcess(cmd=gz_command('gz', 'sim', '-g'), additional_env=env, output='screen',
                         condition=IfCondition(LaunchConfiguration('gui')))
    flight = ExecuteProcess(cmd=gz_command(str(binary), '-d', str(px4/'build/px4_sitl_default/etc'), '-w', str(runtime)),
        additional_env=dict(env, PX4_GZ_STANDALONE='1', PX4_SYS_AUTOSTART='4014',
                            PX4_SIM_MODEL='gz_x500_mono_cam_down', PX4_GZ_WORLD=world, GZ_IP='127.0.0.1'), output='screen')
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', str(config['px4']['udp']['port'])], output='screen')
    camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('pl_camera'))/'launch/camera_sim.launch.py')),
        launch_arguments={'world': world, 'image_topic': '/hitl/pc/camera/image_raw',
                          'debug_bridge': 'false'}.items())
    encoder = Node(package='pl_camera', executable='image_relay', name='hitl_camera_encoder', output='screen',
        parameters=[{'direction': 'encode', 'input_topic': '/hitl/pc/camera/image_raw',
            'output_topic': '/camera/image_raw/compressed', 'fps': float(arg('camera_fps')),
            'jpeg_quality': int(arg('jpeg_quality'))}])
    decoder = Node(package='pl_camera', executable='image_relay', name='hitl_monitor_decoder', output='screen',
        parameters=[{'direction': 'decode', 'input_topic': '/pl/monitor/image/compressed',
            'output_topic': '/hitl/pc/monitor/image', 'fps': float(config['viz']['fps'])}])
    viewer = Node(package='rqt_image_view', executable='rqt_image_view', name='hitl_image_view',
                  arguments=['/hitl/pc/monitor/image'], condition=IfCondition(LaunchConfiguration('viewer')))
    required = [gazebo, flight, agent, encoder]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Required PC process exited'))])) for node in required]
    # Optional viewers never own the lifetime of the flight stack.
    return [SetEnvironmentVariable('GZ_PARTITION', env['GZ_PARTITION']), *handlers,
            gazebo, gui, flight, agent, camera, encoder, decoder, viewer]


def generate_launch_description():
    root = os.environ.get('PRECISION_ROOT', os.getcwd())
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '42')),
        DeclareLaunchArgument('workspace', default_value=root),
        DeclareLaunchArgument('px4_dir', default_value=os.environ.get('PX4_DIR', str(Path.home()/'PX4-Autopilot'))),
        DeclareLaunchArgument('run_dir', default_value=str(Path(root)/'.run/hitl_pc')),
        DeclareLaunchArgument('config', default_value=str(Path(get_package_share_directory('pl_bringup'))/'config/sim.yaml')),
        DeclareLaunchArgument('world', default_value='aruco_moving'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('jpeg_quality', default_value='90'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true'),
        OpaqueFunction(function=setup),
    ])
