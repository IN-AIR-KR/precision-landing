"""IMX708 V4L2 RAW capture with automatic exposure and color conversion."""
import os
import select
import subprocess
from time import monotonic

import cv2
import numpy as np


def capture_imx708(device, frames, errors, stop, mode=0, output_size=None):
    width, height, fps, exposure = ((4608, 2592, 14, 20000) if mode == 0
                                    else (1536, 864, 60, 4000))
    process = subprocess.Popen(
        ['v4l2-ctl', '-d', device,
         f'--set-fmt-video=width={width},height={height},pixelformat=RG10',
         f'--set-ctrl=sensor_mode={mode},override_enable=1,frame_rate={fps * 1000000},exposure={exposure},gain=64',
         '--stream-mmap', '--stream-count=0', '--stream-to=-'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    try:
        data = bytearray()
        frame_bytes = width * height * 2
        rate_applied = False
        gain = 64
        digital_gain = 1.0
        max_exposure = 65000 if mode == 0 else 8000
        last_adjustment = 0.0
        lut = np.round(np.where(
            np.linspace(0, 1, 65536) <= 0.0031308,
            np.linspace(0, 1, 65536) * 12.92,
            1.055 * np.linspace(0, 1, 65536) ** (1 / 2.4) - 0.055
        ) * 255).astype(np.uint8)
        while not stop.is_set():
            if not select.select([process.stdout], [], [], 1)[0]:
                if process.poll() is not None:
                    raise RuntimeError('IMX708 capture exited.')
                continue
            chunk = os.read(process.stdout.fileno(), min(1048576, frame_bytes - len(data)))
            if not chunk:
                raise RuntimeError('IMX708 stream ended.')
            data.extend(chunk)
            if len(data) == frame_bytes:
                if not rate_applied:
                    # Tegra updates the control limits after selecting the stream mode.
                    subprocess.run(
                        ['v4l2-ctl', '-d', device,
                         f'--set-ctrl=frame_rate={fps * 1000000},exposure={exposure},gain=64'],
                        check=True, capture_output=True)
                    rate_applied = True
                raw = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
                # Meter both green sites. RG10 samples are left-aligned in Tegra's 16-bit container.
                green = np.concatenate((raw[0::16, 1::16].ravel(),
                                        raw[1::16, 0::16].ravel())).astype(np.float32)
                green = np.maximum(green - 4096.0, 0.0) / (65535.0 - 4096.0)
                level = float(np.percentile(green, 65))
                saturated = float(np.mean(green > 0.98))
                now = monotonic()
                if now - last_adjustment >= 0.35:
                    ratio = np.clip(0.22 / max(level, 0.005), 0.5, 2.0)
                    if saturated > 0.03:
                        ratio = min(ratio, 0.8)
                    if abs(float(ratio) - 1.0) > 0.12:
                        product = exposure * gain * float(ratio)
                        new_exposure = int(np.clip(product / 16, 100 if mode else 500, max_exposure))
                        new_gain = int(np.clip(product / new_exposure, 16, 256))
                        subprocess.run(
                            ['v4l2-ctl', '-d', device,
                             f'--set-ctrl=exposure={new_exposure},gain={new_gain}'],
                            check=True, capture_output=True)
                        exposure, gain = new_exposure, new_gain
                    last_adjustment = now
                if frames.empty():
                    color = cv2.cvtColor(raw, cv2.COLOR_BAYER_BG2BGR)
                    if output_size is not None:
                        color = cv2.resize(color, output_size, interpolation=cv2.INTER_AREA)
                    color = color.astype(np.float32)
                    color = np.maximum(color - 4096.0, 0.0)
                    means = color[::9, ::9].mean(axis=(0, 1))
                    color *= np.clip(means[1] / np.maximum(means, 1.0), 0.25, 4.0)
                    # Extend indoor exposure compensation after sensor controls reach their limits.
                    target_digital = (float(np.clip(0.22 / max(level, 0.005), 1.0, 16.0))
                                      if exposure == max_exposure and gain == 256 else 1.0)
                    digital_gain += 0.2 * (target_digital - digital_gain)
                    color *= digital_gain
                    linear = np.clip(color * (65535.0 / (65535.0 - 4096.0)), 0, 65535).astype(np.uint16)
                    frames.put(lut[linear])
                data = bytearray()
    except (OSError, RuntimeError, subprocess.CalledProcessError, cv2.error) as error:
        errors.put(error)
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        process.stdout.close()

