from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """
    Real-vehicle launch: aruco_tracker + precision_land only.

    The camera driver (usb_cam) and the Micro XRCE-DDS agent run separately -- as systemd
    services on the drone (see services/), or by hand during bring-up. Nothing here depends on
    Gazebo, so none of the ros_gz_bridge nodes or the Gazebo visualizer are started.
    """
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Image topic published by the camera driver'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera_info',
        description='Calibration topic published by the camera driver. Must carry a real '
                    'calibration for this camera -- PnP range is only as good as the intrinsics.'
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml']),
                {'image_topic': LaunchConfiguration('image_topic'),
                 'camera_info_topic': LaunchConfiguration('camera_info_topic')},
            ]
        ),
        Node(
            package='precision_land',
            executable='precision_land',
            name='precision_land',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('precision_land'), 'cfg', 'params.yaml'])
            ]
        ),
    ])
