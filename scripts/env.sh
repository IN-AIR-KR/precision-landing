#!/usr/bin/env bash
# Source this file before using ROS CLI tools for this workspace.
PRECISION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
export PX4_DIR="${PX4_DIR:-$HOME/PX4-precision-landing}"
export OpenCV_DIR="$PRECISION_ROOT/.deps/opencv/lib/cmake/opencv4"
export CMAKE_PREFIX_PATH="$PRECISION_ROOT/.deps/apt/root/opt/ros/humble:${CMAKE_PREFIX_PATH:-}"
export AMENT_PREFIX_PATH="$PRECISION_ROOT/.deps/apt/root/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$PRECISION_ROOT/.deps/opencv/lib:$PRECISION_ROOT/.deps/apt/root/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$PRECISION_ROOT/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
if [[ -f "$PRECISION_ROOT/install/setup.bash" ]]; then
  source "$PRECISION_ROOT/install/setup.bash"
fi
