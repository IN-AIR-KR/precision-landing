# TESTING.md — Precision Landing System (Python Port)

Step-by-step guide for Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic / PX4 latest firmware.

---

## 1. Prerequisites

### System packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-ros-gz-bridge \
  ros-humble-visualization-msgs \
  ros-humble-std-srvs \
  python3-numpy \
  python3-scipy

pip3 install opencv-contrib-python   # provides ArucoDetector API (4.7+)
```

### PX4 SITL

Follow the official PX4 dev guide to install PX4-Autopilot with Gazebo Harmonic:
<https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html>

### Micro XRCE-DDS Agent

```bash
pip3 install --user micro-xrce-dds-agent
# or build from source:
# https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html
```

---

## 2. Build

```bash
cd ~/path/to/tracktor-beam

# Initialise only the px4_msgs submodule (fast build, no large deps)
git submodule update --init src/px4_msgs

# Build
colcon build
source install/setup.bash
```

Expected: no `--allow-overriding` flags needed. Build should complete in ~1 minute.

---

## 3. Run the full simulation stack

Open **four** terminals and source the workspace in each:

```bash
source /opt/ros/humble/setup.bash
source ~/path/to/tracktor-beam/install/setup.bash
```

### Terminal 1 — PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down_aruco
```

Wait until you see `[commander] Ready for takeoff!`.

### Terminal 2 — Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 3 — ROS 2 launch

```bash
ros2 launch precision_land_py precision_landing_system.launch.py
```

You should see:
- `aruco_tracker_node` — waiting for camera_info
- `precision_land` — `PrecisionLand node started. Call ~/start_landing to begin.`
- `precision_land_viz` — `PrecisionLandViz started`
- Three `ros_gz_bridge` nodes bridging Gazebo ↔ ROS 2

### Terminal 4 — Arm, take off, and start precision landing

```bash
# Arm and take off to 5 m (using PX4 CLI in the SITL shell, or via QGC)
# In the PX4 shell:
commander takeoff

# Once the drone is hovering, trigger precision landing:
ros2 service call /precision_land/start_landing std_srvs/srv/Trigger
```

---

## 4. Verification

### ArUco tracking

```bash
ros2 topic echo /target_pose
```

Expect `geometry_msgs/PoseStamped` messages when the camera sees the ArUco tag.

### World-frame tag pose

```bash
ros2 topic echo /target_pose_world
```

### State machine logs

```bash
ros2 topic echo /rosout | grep precision_land
```

Look for state transitions:
```
[INFO] [precision_land]: Switching to SEARCH
[INFO] [precision_land]: Target acquired
[INFO] [precision_land]: Switching to APPROACH
[INFO] [precision_land]: Switching to DESCEND
[INFO] [precision_land]: Landing complete!
```

### RViz visualization

```bash
rviz2
```

- Set **Fixed Frame** to `map`
- Add a **Marker** display, topic `/target_pose_marker`
- A green flat box (0.5 × 0.5 × 0.02 m) should appear at the detected tag position

### Abort

```bash
ros2 service call /precision_land/abort std_srvs/srv/Trigger
```

The vehicle switches to LOITER hold and the landing sequence stops.

---

## 5. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `Missing camera calibration` in aruco_tracker | camera_info not bridged | Check `camera_info_bridge` node started |
| `Focal length is zero` | Wrong Gazebo world/model | Verify `gz_x500_mono_cam_down_aruco` target |
| OFFBOARD rejected by PX4 | Heartbeat not established | Wait 1 s after calling start_landing |
| Tag never detected | OpenCV version | Ensure `opencv-contrib-python >= 4.7` |
| Build fails with missing `px4_msgs` | Submodule not initialised | `git submodule update --init src/px4_msgs` |
