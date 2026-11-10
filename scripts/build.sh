#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
cd "$PRECISION_ROOT"
profile="${1:-sim}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
case "$profile" in
  sim) packages=(pl_camera pl_bringup pl_control pl_viz pl_detection ros_gz_bridge) ;;
  hardware) packages=(pl_bringup pl_control pl_detection) ;;
  *) echo 'Usage: ./scripts/build.sh [sim|hardware]' >&2; exit 1 ;;
esac
colcon build --base-paths src --packages-up-to "${packages[@]}" \
  --parallel-workers 2 --allow-overriding cv_bridge ros_gz_bridge \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR="$OpenCV_DIR"
if [[ "$profile" == sim ]]; then
  make -C "$PX4_DIR" px4_sitl -j4
fi
