#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
if [[ -f "$PRECISION_ROOT/install/px4-1.16/local_setup.bash" ]]; then
  source "$PRECISION_ROOT/install/px4-1.16/local_setup.bash"
fi
exec ros2 launch pl_bringup real.launch.py "$@"
