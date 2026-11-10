#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
cd "$PRECISION_ROOT"
profile="${1:-sim}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
case "$profile" in
  sim|hitl_pc) packages=(pl_camera pl_bringup pl_control pl_viz pl_detection ros_gz_bridge); gazebo=ON ;;
  hardware|hitl_jetson) packages=(pl_bringup pl_control pl_detection pl_viz); gazebo=OFF ;;
  *) echo 'Usage: ./scripts/build.sh [sim|hardware|hitl_pc|hitl_jetson]' >&2; exit 1 ;;
esac
colcon build --base-paths src --packages-up-to "${packages[@]}" \
  --parallel-workers 2 --allow-overriding ros_gz_bridge \
  --cmake-args -DPL_VIZ_GAZEBO="$gazebo" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
if [[ "$profile" == sim || "$profile" == hitl_pc ]]; then
  make -C "$PX4_DIR" px4_sitl -j4
fi
