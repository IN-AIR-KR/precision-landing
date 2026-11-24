# IMX708 on this Jetson

Validated on Orin Nano, L4T 36.5, kernel 5.15.185-tegra, Camera Module 3 on CAM1.
This board uses the existing UART PIO DTB and header overlay referenced by build.sh.

1. Build with bash scripts/imx708/build.sh.
2. Install with sudo bash scripts/imx708/install.sh.
3. Reboot with sudo reboot.

The installer backs up extlinux.conf and the previous module/DTB under /boot/imx708-backup-*.
To roll back, restore those files and run sudo depmod -a before rebooting.

Mode 0 is 4608×2592 at up to 14 fps. Mode 1 is 1536×864, tested at 60.06 fps.
The 120 fps mode-table limit has not been validated on this board.
Mode 1 uses the Tegra timing representation of 283200000 Hz and 2608 pixels per line,
keeping the inferred CSI rate below the deskew threshold.
RAW output needs software exposure control, black-level correction, white balance and gamma.
The ROS imx708_camera node and calibration tool share that processing.
Autofocus is not implemented.

For calibration, run the following from the repository root.

    env -u PYTHONPATH /usr/bin/python3 scripts/camera_calibration/calibrate.py --imx708 --imx708-mode 1 --cols 9 --rows 6 --square-mm 25

Set camera.driver to imx708_camera in real.yaml. Calibrate at 1536×864.
ROS output remains 640×360 with scaled intrinsics. The measured ROS frame rate was
approximately 29 Hz with detection and HUD running, despite 60 fps sensor capture.

The pinned driver source is [nvidia-jetson-toolkit](https://github.com/rm0nroe/nvidia-jetson-toolkit/tree/ed0b66fddcce9a941175892d70ffa3c8e784db3a).
Its sensor driver and the mode patch are GPL-2.0 derived from RidgeRun and Raspberry Pi Linux.
The CAM1 overlay derives from that repository's MIT-licensed docs/overlays/imx708-nvidia-csi.dts.
The mode register sequence comes from Raspberry Pi's imx708.c, rpi-6.6.y.
