"""Optional recording action shared by the flight launch profiles."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def recording_arguments():
    return [
        DeclareLaunchArgument('record', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('log_dir', default_value=''),
    ]


def recorder(profile, *, config_yaml=None):
    def setup(context):
        executable = Path(get_package_prefix('pl_logging')) / 'lib/pl_logging/flight_recorder'
        command = [str(executable), '--profile', profile]
        log_dir = LaunchConfiguration('log_dir').perform(context)
        if log_dir:
            command += ['--log-dir', log_dir]
        # Keep recorder failures independent of the flight stack.
        env = {'PL_FLIGHT_REAL_YAML': config_yaml} if config_yaml is not None else None
        return [ExecuteProcess(cmd=command, name='flight_recorder', output='screen',
                               sigterm_timeout='10', additional_env=env)]

    return OpaqueFunction(function=setup, condition=IfCondition(LaunchConfiguration('record')))
