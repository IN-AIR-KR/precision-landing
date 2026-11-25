"""Contrast autofocus for Camera Module 3's DW9817 lens motor.

Register protocol: Raspberry Pi Linux drivers/media/i2c/dw9807-vcm.c
and arch/arm/boot/dts/overlays/imx708.dtsi (DW9817, address 0x0c).
The sensor must be streaming to power the lens. No sensor registers are touched.
"""
import fcntl
import os
import time

import cv2
import numpy as np


class Lens:
    def __init__(self, bus=9):
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR | os.O_CLOEXEC)
        try:
            # I2C_SLAVE, deliberately not I2C_SLAVE_FORCE: never override a kernel driver.
            fcntl.ioctl(self.fd, 0x0703, 0x0c)
            self.write(2, 0)  # active mode
            self.position = ((self.read(3) & 3) << 8) | self.read(4)
        except Exception:
            self.close()
            raise

    def write(self, register, *values):
        payload = bytes((register, *values))
        if os.write(self.fd, payload) != len(payload):
            raise OSError('Short DW9817 write')

    def read(self, register):
        self.write(register)
        value = os.read(self.fd, 1)
        if len(value) != 1:
            raise OSError('Short DW9817 read')
        return value[0]

    def move(self, position):
        if not 0 <= position <= 1023:
            raise ValueError('Lens position must be 0..1023')
        while self.position != position:
            target = self.position + max(-16, min(16, position - self.position))
            deadline = time.monotonic() + .02
            while self.read(5):
                if time.monotonic() >= deadline:
                    raise TimeoutError('DW9817 motor busy')
                time.sleep(.001)
            self.write(3, target >> 8, target & 255)
            self.position = target
            time.sleep(.001)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def sharpness(frame):
    """Brightness-normalized gradient energy over the central half of the image."""
    h, w = frame.shape[:2]
    roi = frame[h//4:3*h//4, w//4:3*w//4]
    if roi.ndim == 3:
        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    if roi.shape[1] > 640:
        roi = cv2.resize(roi, (640, round(roi.shape[0]*640/roi.shape[1])))
    image = roi.astype(np.float32)
    mean = float(image.mean())
    if mean < 5 or mean > 250:
        return 0.0
    dx = cv2.Sobel(image, cv2.CV_32F, 1, 0, ksize=3)
    dy = cv2.Sobel(image, cv2.CV_32F, 0, 1, ksize=3)
    return float(np.mean(dx*dx + dy*dy)) / (mean*mean)


class Autofocus:
    """Frame-driven coarse/fine search; holds focus between completed scans."""
    def __init__(self, bus=9, mode='continuous', logger=None):
        if mode not in ('once', 'continuous'):
            raise ValueError('autofocus mode must be once or continuous')
        self.bus, self.mode, self.logger = bus, mode, logger
        self.lens = None
        self.started = time.monotonic()
        self.ready_at = self.started + 2.0  # allow Argus exposure to settle
        self.positions = []
        self.scores = []
        self.samples = []
        self.stage = None
        self.completed = False
        self.locked = False
        self.failed = False
        self.history = []

    def update(self, frame):
        now = time.monotonic()
        if self.failed or now < self.ready_at or (self.completed and self.mode == 'once'):
            return
        try:
            if self.lens is None:
                self.lens = Lens(self.bus)
            if self.stage is None:
                self.history = []
                self.original = self.lens.position
                self.positions = list(range(0, 1024, 64)) + [1023]
                self.stage = 'coarse'
                self.scores = []
                self._next()
                return
            self.samples.append(sharpness(frame))
            if len(self.samples) < 3:
                return
            self.scores.append((float(np.median(self.samples)), self.lens.position))
            self.history.append(self.scores[-1])
            if self.positions:
                self._next()
                return
            best_score, best = max(self.scores)
            if self.stage == 'coarse':
                self.contrast_peak = best_score / max(.000001, float(np.median([x[0] for x in self.scores])))
                self.positions = list(range(max(0, best-64), min(1023, best+64)+1, 8))
                self.stage = 'fine'
                self.scores = []
                self._next()
                return
            self.locked = best_score > .01 and self.contrast_peak >= 1.5
            if not self.locked:
                best = self.original
                self._log('warning', 'Autofocus found no usable texture; restoring previous position')
            self.lens.move(best)
            self.completed = True
            self.stage = None
            self.ready_at = time.monotonic() + 10.0
            if self.locked:
                self._log('info', f'Autofocus locked: lens_position={best}, sharpness={best_score:.5f}')
        except (OSError, TimeoutError) as exc:
            self.failed = True
            if self.lens and hasattr(self, 'original'):
                try:
                    self.lens.move(self.original)
                except (OSError, TimeoutError):
                    pass
            self._log('error', f'Autofocus failed: {exc}; camera capture continues without autofocus')
            self.close()

    def _next(self):
        self.lens.move(self.positions.pop(0))
        self.samples = []
        self.ready_at = time.monotonic() + .15

    def _log(self, level, message):
        if self.logger:
            getattr(self.logger, level)(message)

    def close(self):
        if self.lens:
            self.lens.close()
            self.lens = None
