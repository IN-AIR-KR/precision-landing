#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/oem/ARK/tracktor-beam
source install/setup.bash
ros2 run pl_detection aruco_tracker
