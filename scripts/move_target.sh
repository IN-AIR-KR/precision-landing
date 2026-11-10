#!/usr/bin/env bash
set -eo pipefail
export GZ_PARTITION="${GZ_PARTITION:-precision_landing}"
# Speeds are in m/s; default is 0.3 m/s along world X.
x="${1:-0.3}"
y="${2:-0}"
if [[ ! "$x" =~ ^-?[0-9]+([.][0-9]+)?$ || ! "$y" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
  echo 'Usage: ./scripts/move_target.sh [vx] [vy] (m/s)' >&2
  exit 1
fi
exec gz topic -t /model/aruco_moving_target/cmd_vel -m gz.msgs.Twist -p "linear: {x: $x, y: $y}"
