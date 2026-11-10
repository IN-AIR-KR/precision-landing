# pl_camera

`launch/camera_sim.launch.py` owns the Gazebo camera bridge. `pl_bringup`
includes it with world, model, image_topic and camera_info_topic arguments.
It forwards Image, CameraInfo and clock to ROS, and /image_proc to Gazebo.
CSI capture is deferred; no hardware camera driver is implemented yet.

```bash
ros2 launch pl_camera camera_sim.launch.py world:=aruco_moving
```

Do not start a second bridge while pl_bringup is running.
The simulation build explicitly selects pl_camera and pl_viz; hardware excludes
these Gazebo-only implementations until their hardware support is implemented.

`image_relay` provides lazy JPEG encoding/decoding with original headers.
`hitl_pc.launch.py` encodes `/hitl/pc/camera/image_raw` to
`/camera/image_raw/compressed` (quality 90, maximum 30 fps), and decodes
`/pl/monitor/image/compressed` to `/hitl/pc/monitor/image` for the local viewer.
The camera launch's `debug_bridge:=false` disables the Gazebo debug-image bridge
for the distributed PC/Jetson profile.
