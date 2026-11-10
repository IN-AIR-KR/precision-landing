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
python3 -m pip install --only-binary=:all: --upgrade --target "$root/.deps/python" -r requirements-detection.txt
exec ./scripts/build.sh "$profile"
