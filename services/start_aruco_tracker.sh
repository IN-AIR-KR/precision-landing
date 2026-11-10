#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/oem/ARK/tracktor-beam
source scripts/env.sh
ros2 run pl_detection aruco_tracker
