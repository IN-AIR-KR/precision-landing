# pl_detection

Python ArUco detector. Install binary-only requirements with `scripts/setup.sh`;
source `scripts/env.sh` to select the workspace-local wheel runtime.

Inputs: `/camera/image_raw` (`sensor_msgs/Image`) and `/camera/camera_info`
(`sensor_msgs/CameraInfo`), best-effort, volatile, depth 1. Launch may remap them.
Outputs: `/target_pose` (`geometry_msgs/PoseStamped`) and `/image_proc` (BGR8),
with the same QoS and original image timestamp. Optical axes are x right, y down,
z forward; `optical_frame` defaults to `camera_frame`, or input frame if empty.

Supported encodings: `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`. Row padding is
respected using `Image.step`; data length must equal height * step. All supported
channels are 8-bit, so byte order is irrelevant. Unsupported encodings and invalid
buffers are rejected without publishing a pose. Output owns its BGR buffer.

Parameters: `aruco_id`, `dictionary`, `marker_size` (meters), `optical_frame`.
The first CameraInfo with nonzero fx is latched, matching the C++ baseline.
Restart after changing calibration or image resolution. Empty distortion arrays
retain the baseline's no-pose behavior; publish explicit zero distortion values
for a calibrated distortion-free camera.

Detection uses default ArucoDetector parameters, float32 marker corners,
undistortPoints followed by iterative solvePnP, and the original OpenCV quaternion
sign convention. Only the first matching target is published. Annotated images
are still published for every accepted image; demand-driven output is stage 4.

```bash
source scripts/env.sh
ros2 run pl_detection aruco_tracker --ros-args -r __node:=aruco_tracker --params-file src/pl_detection/cfg/params.yaml
python3 -m pytest -q src/pl_detection/test
```
