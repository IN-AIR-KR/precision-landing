#!/usr/bin/env bash
# Ubuntu 22.04 with ROS Humble, Gazebo Harmonic and PX4 build tools installed.
set -eo pipefail
cd "$(dirname "$0")/.."
root="$PWD"
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
profile="${1:-sim}"
case "$profile" in sim|hardware) ;; *) echo "Usage: $0 [sim|hardware]" >&2; exit 1 ;; esac
git submodule update --init --recursive
if [[ "$profile" == sim && ! -f "$PX4_DIR/Makefile" ]]; then
  echo "PX4 checkout not found: $PX4_DIR. Set PX4_DIR to your existing checkout." >&2
  exit 1
fi
# Preserve the user's PX4 checkout, submodules and custom models.
mkdir -p .deps/apt
touch .deps/COLCON_IGNORE
if [[ "$profile" == sim && ! -d .deps/apt/root/opt/ros/humble/share/gps_msgs ]]; then
  (cd .deps/apt; apt-get download ros-humble-gps-msgs; dpkg-deb -x ros-humble-gps-msgs*.deb root)
fi
python3 -m pip install --only-binary=:all: --upgrade --target "$root/.deps/python" -r requirements-detection.txt
exec ./scripts/build.sh "$profile"
