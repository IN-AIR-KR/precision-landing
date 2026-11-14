#!/usr/bin/env python3
"""Interactively calibrate the camera directly through Jetson Argus."""

import argparse
import os
from pathlib import Path
from queue import SimpleQueue
from tempfile import NamedTemporaryFile
from threading import Event, Thread

import cv2
import numpy as np
import yaml


WINDOW = 'Camera calibration'
MIN_SAMPLES = 10


def save_calibration(path, size, board, samples):
    rms, intrinsic, distortion, _, _ = cv2.calibrateCamera(
        [board] * len(samples), samples, size, None, None)
    if not all(np.isfinite(value).all() for value in (rms, intrinsic, distortion)):
        raise ValueError('Calibration returned non-finite values. Capture more varied poses.')
    temp_path = None
    try:
        with NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                delete=False) as output:
            temp_path = Path(output.name)
            output.write(
                'camera:\n'
                '  calibration:\n'
                f'    image_width: {size[0]}\n'
                f'    image_height: {size[1]}\n'
                '    camera_matrix:\n'
                f'      data: {intrinsic.ravel().tolist()}\n'
                '    distortion_coefficients:\n'
                f'      data: {distortion.ravel().tolist()}\n'
            )
        temp_path.replace(path)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)
    print(f'Saved {path.resolve()} | {size[0]}x{size[1]} | RMS {rms:.3f} px')


def capture(pipeline, frames, errors, stop):
    camera = cv2.VideoCapture()
    try:
        if not camera.open(pipeline, cv2.CAP_GSTREAMER):
            raise RuntimeError('Could not open CSI camera. Check GStreamer and whether the sensor is busy.')
        while not stop.is_set():
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError('CSI camera stopped delivering frames.')
            if not frames.empty():
                continue
            frames.put(frame)
    except (RuntimeError, cv2.error) as error:
        errors.put(error)
    finally:
        camera.release()


def detect_corners(frame, pattern):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    scale = min(1.0, 640 / frame.shape[1])
    search = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    found, corners = cv2.findChessboardCorners(
        search, pattern, cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_ADAPTIVE_THRESH)
    if not found:
        return None
    corners /= scale
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def calibrate_saved(args, sample_dir, captured=None, excluded=()):
    pattern = (args.cols, args.rows)
    size = (args.width, args.height)
    board = np.zeros((args.cols * args.rows, 3), dtype=np.float32)
    board[:, :2] = np.mgrid[:args.cols, :args.rows].T.reshape(-1, 2) * (args.square_mm / 1000)
    captured = captured or {}
    paths = sorted(set(sample_dir.rglob('*.png')) - set(excluded))
    print(f'Checking {len(paths)} saved PNGs in {sample_dir}', flush=True)
    samples = []
    for path in paths:
        if path in captured:
            corners = captured[path]
        else:
            frame = cv2.imread(str(path))
            if frame is None or frame.shape[:2] != size[::-1]:
                print(f'Skipped {path}: unreadable or wrong resolution', flush=True)
                continue
            corners = detect_corners(frame, pattern)
        if corners is None:
            print(f'Skipped {path}: checkerboard not found', flush=True)
            continue
        samples.append(corners)
    if len(samples) < MIN_SAMPLES:
        raise ValueError(f'Need {MIN_SAMPLES} accepted samples. Found {len(samples)}.')
    print(f'Calibrating {len(samples)} samples. Output: {args.output.resolve()}', flush=True)
    save_calibration(args.output, size, board, samples)


def load_preview_maps(path, size):
    with path.open(encoding='utf-8') as source:
        calibration = yaml.safe_load(source)['camera']['calibration']
    saved_size = (calibration['image_width'], calibration['image_height'])
    if saved_size != size:
        raise ValueError(f'Calibration is for {saved_size[0]}x{saved_size[1]}.')
    intrinsic = np.asarray(calibration['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(calibration['distortion_coefficients']['data'], dtype=np.float64)
    if not np.isfinite(intrinsic).all() or not np.isfinite(distortion).all():
        raise ValueError('Calibration contains non-finite coefficients.')
    return cv2.initUndistortRectifyMap(intrinsic, distortion, None, intrinsic, size, cv2.CV_16SC2)


def preview(frames, errors, args, sample_dir):
    samples = {}
    excluded = set()
    next_image = max((int(path.stem) for path in sample_dir.glob("*.png")
                      if path.stem.isascii() and path.stem.isdecimal()), default=0) + 1
    size = None
    pattern = (args.cols, args.rows)
    display = np.zeros((540, 960, 3), dtype=np.uint8)
    message = 'Opening CSI camera...'
    toggle = 'Undistort (0=off, 1=on)'
    cv2.createTrackbar(toggle, WINDOW, 0, 1, lambda _: None)
    maps = None
    while True:
        if not errors.empty():
            raise errors.get()
        if not frames.empty():
            display = frames.get()
            frame_size = (display.shape[1], display.shape[0])
            if size is None:
                print(f'First frame: {frame_size[0]}x{frame_size[1]}', flush=True)
                message = 'Show the entire checkerboard.'
            elif frame_size != size:
                excluded.update(samples)
                samples.clear()
                message = 'Image size changed. Samples cleared.'
            if frame_size != size:
                maps = None
            size = frame_size

        canvas = display.copy()
        if size is not None and cv2.getTrackbarPos(toggle, WINDOW):
            try:
                if maps is None:
                    maps = load_preview_maps(args.output, size)
                canvas = cv2.remap(display, *maps, cv2.INTER_LINEAR)
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError, cv2.error) as error:
                message = f'Cannot apply calibration: {error}'
                cv2.setTrackbarPos(toggle, WINDOW, 0)
        lines = [
            f'Samples: {len(samples)} this run (C also loads saved PNGs)',
            'SPACE capture   U undo   C calibrate and save   Q quit',
            message,
        ]
        for row, line in enumerate(lines, start=1):
            cv2.putText(canvas, line, (12, 27 * row), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (30, 220, 30), 2, cv2.LINE_AA)
        cv2.imshow(WINDOW, canvas)
        key = ord(chr(cv2.waitKey(1) & 0xff).lower())
        # GTK can report the window as invisible before its first live repaint.
        if key in (ord('q'), 27):
            return
        if key == ord(' '):
            corners = detect_corners(display, pattern) if size == (args.width, args.height) else None
            if corners is None:
                message = 'Checkerboard not found. Sample skipped.'
            elif any(np.sqrt(np.mean((sample - corners) ** 2)) < 5 for sample in samples.values()):
                message = 'Move or tilt the board before capturing again.'
            else:
                image_path = sample_dir / f'{next_image:04d}.png'
                try:
                    encoded, png = cv2.imencode('.png', display)
                    if not encoded:
                        raise OSError(f'Could not encode {image_path}')
                    while True:
                        try:
                            with image_path.open('xb') as output:
                                output.write(png.tobytes())
                            break
                        except FileExistsError:
                            next_image += 1
                            image_path = sample_dir / f'{next_image:04d}.png'
                except (OSError, cv2.error) as error:
                    message = 'Could not save sample. See terminal.'
                    print(error, flush=True)
                    continue
                samples[image_path] = corners
                next_image += 1
                print(f'Saved image: {image_path}', flush=True)
                message = f'Captured sample {len(samples)}. Move or tilt the board.'
            print(message, flush=True)
        if key == ord('u') and samples:
            path, _ = samples.popitem()
            excluded.add(path)
            message = 'Last sample removed.'
        if key == ord('c'):
            cv2.setWindowTitle(WINDOW, 'Camera calibration - calculating...')
            cv2.waitKey(1)
            try:
                calibrate_saved(args, sample_dir, samples, excluded)
            except (OSError, ValueError, cv2.error) as error:
                message = 'Could not save calibration. See terminal.'
                cv2.setWindowTitle(WINDOW, WINDOW)
                print(error, flush=True)
            else:
                maps = None
                message = 'Saved calibration.yaml. Preview corrected.'
                cv2.setWindowTitle(WINDOW, WINDOW)
                cv2.setTrackbarPos(toggle, WINDOW, 1)


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
    parser.add_argument('--from-saved', action='store_true', help='Calibrate from saved PNGs without opening the camera')
    args = parser.parse_args()
    if min(args.cols, args.rows) < 3 or not np.isfinite(args.square_mm) or args.square_mm <= 0:
        parser.error('Use at least 3x3 inner corners and a positive square size.')
    script_dir = Path(__file__).resolve().parent
    args.output = script_dir / 'calibration.yaml'
    sample_dir = script_dir / 'samples'
    if args.from_saved:
        try:
            calibrate_saved(args, sample_dir)
        except (OSError, ValueError, cv2.error) as error:
            parser.exit(1, f'{error}\n')
        return
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('No graphical display. Run from a terminal in the Jetson desktop session.')

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
    frames = SimpleQueue()
    errors = SimpleQueue()
    stop = Event()
    worker = Thread(target=capture, args=(pipeline, frames, errors, stop),
                    daemon=True)
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 960, 540)
        print(f'Opening CSI sensor {args.sensor_id} at '
              f'{args.width}x{args.height} @ {args.fps} FPS', flush=True)
        sample_dir.mkdir(exist_ok=True)
        print(f'Sample images: {sample_dir}', flush=True)
        worker.start()
        preview(frames, errors, args, sample_dir)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, cv2.error) as error:
        parser.exit(1, f'{error}\n')
    finally:
        stop.set()
        if worker.ident is not None:
            worker.join()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
