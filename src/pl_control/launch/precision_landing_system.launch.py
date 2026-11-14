"""Compatibility entry point for a running SITL and DDS Agent."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='aruco'),
        DeclareLaunchArgument('marker_size', default_value='0.5'),
        DeclareLaunchArgument('enable_gazebo_viz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pl_bringup'), 'launch', 'sim.launch.py',
            ])),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'marker_size': LaunchConfiguration('marker_size'),
                'enable_gazebo_viz': LaunchConfiguration('enable_gazebo_viz'),
                'start_agent': 'false',
            }.items(),
        ),
    ])
