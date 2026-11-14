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
        'max_velocity': tracking['max_speed_mps'],
        'target_timeout': target['timeout_s'],
        'target_accel_noise': target['acceleration_noise_mps2'],
        'target_pos_noise': target['position_noise_m'],
        'camera_position_body': [float(value) for value in extrinsic['position']],
        'camera_orientation_body': [float(value) for value in extrinsic['rotation']],
    }
