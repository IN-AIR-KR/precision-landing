# IMX708 on this Jetson

Validated on Orin Nano, L4T 36.5, kernel 5.15.185-tegra, Camera Module 3 on CAM1.
This board uses the existing UART PIO DTB and header overlay referenced by build.sh.

1. Build with bash scripts/imx708/build.sh.
2. Install with sudo bash scripts/imx708/install.sh.
3. Reboot with sudo reboot.

The installer backs up extlinux.conf and the previous module/DTB under /boot/imx708-backup-*.
To roll back, restore those files and run sudo depmod -a before rebooting.

Mode 0 is 4608×2592 at up to 14 fps. Mode 1 is 1536×864, tested at 60.06 fps.
Measured on 2026-11-26 at a requested 120 fps:
raw V4L2 capture sustained 120.12 fps, but Argus NV12 output sustained
62.65 fps. Hardware scaling to 640×360 BGRx measured 61.25 fps,
and BGR conversion measured 61.85 fps. ROS BGR reception measured
58.64 fps in a separate run. These are sequential short measurements,
so small differences include run-to-run variation.
A 1 ms exposure limit, disabling AE/AWB, disabling temporal noise
reduction and edge enhancement, and disabling AE antibanding did not
remove the approximately 62 fps Argus limit. The 120 fps sensor mode
works through raw V4L2, but 120 fps through Argus/ROS is not validated.
Keep the real profile at 60 fps pending an Argus/driver timing fix.
Mode 1 uses the Tegra timing representation of 283200000 Hz and 2608 pixels per line,
keeping the inferred CSI rate below the deskew threshold.
Argus handles automatic exposure, gain, white balance and ISP conversion.
The ROS csi_camera node and calibration tool both use nvarguscamerasrc.
The DW9817 lens is controlled directly through `/dev/i2c-9` at address `0x0c`.
The current Jetson Argus plugin does not expose focus controls; `pl_camera.autofocus`
performs a coarse/fine contrast search while Argus continues automatic exposure.
No additional kernel module or reboot is needed. The user must belong to `i2c`
(the current `inair` user already does). A bound kernel lens driver is never overridden.

`camera.autofocus_mode` in real.yaml defaults to `once`: focus automatically after
startup, then hold the position during flight. Put a textured target at the intended
working distance in view and wait for `Autofocus locked` before use. `continuous`
repeats a full search after a 10-second hold; it temporarily defocuses the image
and is intended for bench checks. `off` leaves the motor untouched. Each scan takes
roughly 8–12 seconds. Dark/featureless scenes cannot establish reliable focus.
Failures are logged explicitly; camera capture continues without autofocus.
`camera.focus_i2c_bus` defaults to 9 for this CAM1 wiring.

The IMX708 calibration tool also focuses once; wait for `Autofocus locked` before
capturing samples. Changing focus can change camera intrinsics, so calibration
accuracy must be rechecked at the working focus/distance.

Motor protocol references: [Raspberry Pi DW9817 driver](https://github.com/raspberrypi/linux/blob/rpi-6.6.y/drivers/media/i2c/dw9807-vcm.c)
and [IMX708 lens wiring](https://github.com/raspberrypi/linux/blob/rpi-6.6.y/arch/arm/boot/dts/overlays/imx708.dtsi).

For calibration, run the following from the repository root.

    env -u PYTHONPATH /usr/bin/python3 scripts/camera_calibration/calibrate.py --imx708 --imx708-mode 1 --cols 9 --rows 6 --square-mm 25

Use sensor_mode 1, width 1536, height 864 and fps 60 in real.yaml. Calibrate at 1536×864.
ROS output remains 640×360 with scaled intrinsics. The measured ROS frame rate was
approximately 29 Hz with the previous software RAW conversion. Argus capture
was measured at 55.44 fps including startup and exposure compensation changes.

The pinned driver source is [nvidia-jetson-toolkit](https://github.com/rm0nroe/nvidia-jetson-toolkit/tree/ed0b66fddcce9a941175892d70ffa3c8e784db3a).
Its sensor driver and the mode patch are GPL-2.0 derived from RidgeRun and Raspberry Pi Linux.
The CAM1 overlay derives from that repository's MIT-licensed docs/overlays/imx708-nvidia-csi.dts.
The mode register sequence comes from Raspberry Pi's imx708.c, rpi-6.6.y.
