from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    enable_viz_arg = DeclareLaunchArgument(
        'enable_viz',
        default_value='true',
        description='Enable RViz marker visualization',
    )

    return LaunchDescription([
        enable_viz_arg,

        # Bridge camera image from Gazebo to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_bridge',
            arguments=[
                '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
                '@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen',
        ),

        # Bridge camera info from Gazebo to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=[
                '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info'
                '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen',
        ),

        # Bridge processed image from ROS 2 back to Gazebo for display
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_proc_bridge',
            arguments=[
                '/image_proc@sensor_msgs/msg/Image[gz.msgs.Image'
            ],
            parameters=[{
                'qos_overrides./image_proc.subscription.reliability': 'best_effort',
                'qos_overrides./image_proc.publisher.reliability': 'best_effort',
            }],
            output='screen',
        ),

        # ArUco tracker
        Node(
            package='precision_land_py',
            executable='aruco_tracker',
            name='aruco_tracker_node',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('precision_land_py'), 'cfg', 'aruco_tracker_params.yaml']
                ),
            ],
        ),

        # Precision landing controller
        Node(
            package='precision_land_py',
            executable='precision_land',
            name='precision_land',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('precision_land_py'), 'cfg', 'precision_land_params.yaml']
                ),
            ],
        ),

        # RViz marker visualizer
        Node(
            package='precision_land_py',
            executable='precision_land_viz',
            name='precision_land_viz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_viz')),
        ),
    ])
