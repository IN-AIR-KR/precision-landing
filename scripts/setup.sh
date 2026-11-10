#!/usr/bin/env bash
# Ubuntu 22.04 with ROS Humble, Gazebo Harmonic and PX4 build tools installed.
set -eo pipefail
cd "$(dirname "$0")/.."
root="$PWD"
px4_commit=86f2fdfd7de7f5e5d29ab61f60d81e892a557376
export PX4_DIR="${PX4_DIR:-$HOME/PX4-precision-landing}"
profile="${1:-sim}"
case "$profile" in sim|hardware) ;; *) echo "Usage: $0 [sim|hardware]" >&2; exit 1 ;; esac
git submodule update --init --recursive
if [[ "$profile" == sim ]]; then
if [[ ! -d "$PX4_DIR" ]]; then
  git -C "$HOME/PX4-Autopilot" worktree add --detach "$PX4_DIR" "$px4_commit"
fi
if [[ "$(git -C "$PX4_DIR" rev-parse HEAD)" != "$px4_commit" ]]; then
  echo "PX4_DIR must be at $px4_commit" >&2; exit 1
fi
git -C "$PX4_DIR" submodule update --init --recursive
fi
mkdir -p .deps/apt
touch .deps/COLCON_IGNORE
if [[ "$profile" == sim && ! -d .deps/apt/root/opt/ros/humble/share/gps_msgs ]]; then
  (cd .deps/apt; apt-get download ros-humble-gps-msgs; dpkg-deb -x ros-humble-gps-msgs*.deb root)
fi
if [[ ! -f .deps/opencv/lib/cmake/opencv4/OpenCVConfig.cmake ]]; then
  for project in opencv opencv_contrib; do
    if [[ ! -d ".deps/$project-4.10.0" ]]; then
      curl -fL "https://github.com/opencv/$project/archive/refs/tags/4.10.0.tar.gz" -o ".deps/$project.tar.gz"
      tar -xzf ".deps/$project.tar.gz" -C .deps
    fi
  done
  cmake -S .deps/opencv-4.10.0 -B .deps/opencv-build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/.deps/opencv" \
    -DOPENCV_EXTRA_MODULES_PATH="$root/.deps/opencv_contrib-4.10.0/modules" \
    -DBUILD_LIST=core,imgproc,imgcodecs,calib3d,aruco,videoio,highgui \
    -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_python3=OFF -DWITH_CUDA=OFF
  cmake --build .deps/opencv-build -j4
  cmake --install .deps/opencv-build
fi
exec ./scripts/build.sh "$profile"
