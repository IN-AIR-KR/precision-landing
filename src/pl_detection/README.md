# ArUco camera input

The detector is independent of Gazebo. It subscribes to:

- `/camera/image_raw` (`sensor_msgs/msg/Image`, best effort, depth 1)
- `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`, same QoS)

`/target_pose` is a `geometry_msgs/msg/PoseStamped` in optical coordinates
(x right, y down, z forward). Its timestamp is copied unchanged from the source image.
`optical_frame` sets the output frame name (default `camera_frame` for baseline compatibility).
An empty `optical_frame` uses the image's frame name. This names a frame; it does not rotate coordinates.
`/image_proc` retains the image header and contains the annotated BGR image.

The first calibration with a nonzero focal length is latched; restart the detector if calibration changes.
Supply CameraInfo for the actual image resolution. Physical CSI calibration and capture timestamps are
pending hardware commissioning.

For the full simulator, run `./scripts/run.sh` from the workspace root. `pl_bringup` maps the Gazebo
sensor topics to these common camera topics and supplies the physical marker size (1.0m moving pad,
0.5m static PX4 marker).

For another camera publisher, keep the same binary and remap inputs:

```bash
ros2 run pl_detection aruco_tracker --ros-args \
  -r /camera/image_raw:=/my_camera/image_raw \
  -r /camera/camera_info:=/my_camera/camera_info \
  -p marker_size:=1.0 -p optical_frame:=camera_optical_frame
```

`pl_bringup/config/hardware.yaml` holds the corresponding external-camera configuration. It is not
commissioned for flight until the actual camera calibration and mounting transform are verified.
