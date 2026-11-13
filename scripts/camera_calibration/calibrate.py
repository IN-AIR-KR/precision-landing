#!/usr/bin/env python3
"""Interactively calibrate the camera directly through Jetson Argus."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


WINDOW = 'Camera calibration'
MIN_SAMPLES = 10


def matrix_data(matrix):
    return {'rows': matrix.shape[0], 'cols': matrix.shape[1],
            'data': matrix.ravel().tolist()}


def save_calibration(path, camera_name, size, board, samples):
    rms, intrinsic, distortion, _, _ = cv2.calibrateCamera(
        [board] * len(samples), samples, size, None, None)
    if not all(np.isfinite(value).all() for value in (rms, intrinsic, distortion)):
        raise ValueError('Calibration returned non-finite values. Capture more varied poses.')
    projection = np.column_stack((intrinsic, np.zeros(3)))
    data = {
        'image_width': size[0],
        'image_height': size[1],
        'camera_name': camera_name,
        'camera_matrix': matrix_data(intrinsic),
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': matrix_data(distortion.reshape(1, -1)),
        'rectification_matrix': matrix_data(np.eye(3)),
        'projection_matrix': matrix_data(projection),
    }
    with path.open('x', encoding='utf-8') as output:
        yaml.safe_dump(data, output, sort_keys=False)
    print(f'Saved {path.resolve()} | {size[0]}x{size[1]} | RMS {rms:.3f} px')


def preview(camera, args):
    pattern = (args.cols, args.rows)
    board = np.zeros((args.cols * args.rows, 3), dtype=np.float32)
    board[:, :2] = np.mgrid[:args.cols, :args.rows].T.reshape(-1, 2) * (args.square_mm / 1000)
    samples = []
    size = None
    message = 'Show the entire checkerboard.'
    while True:
        ok, display = camera.read()
        if not ok:
            raise RuntimeError('CSI camera stopped delivering frames.')
        gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
        frame_size = (display.shape[1], display.shape[0])
        if size is not None and frame_size != size:
            samples.clear()
            message = 'Image size changed. Samples cleared.'
        size = frame_size
        found, corners = cv2.findChessboardCorners(
            gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, pattern, corners, True)
        else:
            corners = None

        canvas = display
        lines = [
            f'Samples: {len(samples)} (min {MIN_SAMPLES}) | Board: {"found" if corners is not None else "searching"}',
            'SPACE capture   U undo   C calibrate and save   Q quit',
            message,
        ]
        for row, line in enumerate(lines, start=1):
            cv2.putText(canvas, line, (12, 27 * row), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (30, 220, 30), 2, cv2.LINE_AA)
        cv2.imshow(WINDOW, canvas)
        key = cv2.waitKey(1) & 0xff
        if key in (ord('q'), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            return
        if key == ord(' '):
            if corners is None:
                message = 'Show the entire board before capturing.'
            elif any(np.sqrt(np.mean((sample - corners) ** 2)) < 5 for sample in samples):
                message = 'Move or tilt the board before capturing again.'
            else:
                samples.append(corners.copy())
                message = 'Captured. Move or tilt the board.'
        if key == ord('u') and samples:
            samples.pop()
            message = 'Last sample removed.'
        if key == ord('c'):
            if len(samples) < MIN_SAMPLES:
                message = f'Capture at least {MIN_SAMPLES} views.'
                continue
            try:
                save_calibration(args.output, args.camera_name, size, board, samples)
            except (OSError, ValueError, cv2.error) as error:
                message = 'Could not save calibration. See terminal.'
                print(error)
            else:
                return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cols', type=int, required=True, help='Inner corners per row')
    parser.add_argument('--rows', type=int, required=True, help='Inner corners per column')
    parser.add_argument('--square-mm', type=float, required=True, help='Measured square edge in mm')
    parser.add_argument('--sensor-id', type=int, default=0)
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--flip-method', type=int, choices=range(8), default=0)
    parser.add_argument('--camera-name', default='csi_camera')
    parser.add_argument('--output', type=Path, required=True, help='Path to a new YAML file')
    args = parser.parse_args()
    if min(args.cols, args.rows) < 3 or not np.isfinite(args.square_mm) or args.square_mm <= 0:
        parser.error('Use at least 3x3 inner corners and a positive square size.')
    if args.output.exists() or not args.output.parent.is_dir():
        parser.error('Output must be a new file in an existing directory.')

    if min(args.width, args.height, args.fps) <= 0 or args.sensor_id < 0:
        parser.error('Resolution and FPS must be positive, and sensor ID must be nonnegative.')

    pipeline = (
        f'nvarguscamerasrc sensor-id={args.sensor_id} wbmode=1 ! '
        f'video/x-raw(memory:NVMM),width={args.width},height={args.height},'
        f'framerate={args.fps}/1,format=NV12 ! '
        f'nvvidconv flip-method={args.flip_method} ! video/x-raw,format=BGRx ! '
        'videoconvert ! video/x-raw,format=BGR ! '
        'appsink max-buffers=1 drop=true sync=false'
    )
    camera = cv2.VideoCapture()
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 960, 540)
        splash = np.zeros((540, 960, 3), dtype=np.uint8)
        cv2.putText(splash, 'Opening CSI camera...', (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(WINDOW, splash)
        cv2.waitKey(1)
        print(f'Opening CSI sensor {args.sensor_id} at '
              f'{args.width}x{args.height} @ {args.fps} FPS', flush=True)
        camera.open(pipeline, cv2.CAP_GSTREAMER)
        if not camera.isOpened():
            parser.exit(1, 'Could not open CSI camera. Use Jetson system OpenCV with GStreamer '
                        'and close any other process holding the sensor.\n')
        preview(camera, args)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, cv2.error) as error:
        parser.exit(1, f'{error}\n')
    finally:
        camera.release()


if __name__ == '__main__':
    main()
