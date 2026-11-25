"""Camera, detection, and HUD nodes shared by the Jetson launch profiles."""
import json

from launch_ros.actions import Node


def vision_nodes(config, controller):
    camera_config = config['camera']
    calibration = camera_config['calibration']
    if not calibration:
        raise RuntimeError('Set camera.calibration in real.yaml to the measured camera calibration.')

    detection = Node(package='pl_detection', executable='aruco_tracker', name='aruco_tracker',
                     output='screen',
                     parameters=[dict(config['detection'], optical_frame='camera_optical_frame')])
    monitor_parameters = {
        **{k: v for k, v in config['viz'].items() if k != 'gazebo'},
        'camera_position_body': controller['camera_position_body'],
        'camera_orientation_body': controller['camera_orientation_body'],
    }
    monitor = Node(package='pl_viz', executable='monitor', output='screen',
                   parameters=[monitor_parameters])
    camera_parameters = {'calibration_json': json.dumps(calibration)}
    for name in ('sensor_mode', 'width', 'height', 'fps', 'output_width', 'output_height'):
        if name in camera_config:
            camera_parameters[name] = camera_config[name]
    camera = Node(package='pl_camera', executable='csi_camera', name='csi_camera',
                  output='screen', parameters=[camera_parameters])
    return camera, detection, monitor
