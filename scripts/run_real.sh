#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
exec ros2 launch pl_bringup real.launch.py "$@"
