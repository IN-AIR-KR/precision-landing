def control_parameters(control):
    """Translate grouped profile settings into controller parameters."""
    align = control['align']
    approach = control['approach']
    tracking = control['horizontal_tracking']
    target = control['target_estimation']
    extrinsic = control['camera_extrinsic']
    if extrinsic['position'] is None or extrinsic['rotation'] is None:
        raise RuntimeError('Set control.camera_extrinsic.position and rotation in the bringup YAML.')

    return {
        'delta_position': align['position_tolerance_m'],
        'delta_velocity': align['relative_speed_tolerance_mps'],
        'descent_vel': approach['descent_speed_mps'],
        'vel_p_gain': tracking['velocity_p_gain'],
        'vel_i_gain': tracking['velocity_i_gain'],
        'vel_d_gain': float(tracking.get('velocity_d_gain', 0.0)),
        'max_velocity': tracking['max_speed_mps'],
        'target_timeout': target['timeout_s'],
        'target_accel_noise': target['acceleration_noise_mps2'],
        'target_pos_noise': target['position_noise_m'],
        'camera_position_body': [float(value) for value in extrinsic['position']],
        'camera_orientation_body': [float(value) for value in extrinsic['rotation']],
    }


def real_mpc_parameters(config):
    """Require airframe-specific calibration before starting the CTBR controller."""
    from math import isfinite

    mpc = config.get('mpc') or {}
    values = {}
    for key in ('hover_thrust', 'px4_thrust_model_factor', 'measured_motor_thrust_model_factor',
                'land_detector_z_velocity_max', 'approach_descent_speed'):
        value = mpc.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
            raise RuntimeError(f'Set mpc.{key} to a measured or configured value in the real profile.')
        values[key] = float(value)
    if not 0. < values['hover_thrust'] < 1.:
        raise RuntimeError('mpc.hover_thrust must be between 0 and 1.')
    for key in ('px4_thrust_model_factor', 'measured_motor_thrust_model_factor'):
        if not 0. <= values[key] <= 1.:
            raise RuntimeError(f'mpc.{key} must be between 0 and 1.')
    for key in ('land_detector_z_velocity_max', 'approach_descent_speed'):
        if values[key] <= 0.:
            raise RuntimeError(f'mpc.{key} must be positive.')
    marker_size = float(config['detection']['marker_size'])
    if not isfinite(marker_size) or marker_size <= 0.:
        raise RuntimeError('Set detection.marker_size to the measured marker side length in meters.')
    return dict(values, marker_size=marker_size)
