#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
# Hardware profile is a guarded stage-0 target, not a commissioned CSI setup.
exec ros2 launch pl_bringup hardware.launch.py "$@"
