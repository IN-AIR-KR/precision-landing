#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
if [[ -f "$PRECISION_ROOT/install/px4-1.16/local_setup.bash" ]]; then
  # Re-source the overlay after removing its earlier entries so colcon puts it first.
  for variable in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do
    IFS=: read -r -a paths <<< "${!variable}"
    filtered=()
    for path in "${paths[@]}"; do
      if [[ "$path" != "$PRECISION_ROOT/install/px4-1.16"* ]]; then filtered+=("$path"); fi
    done
    printf -v "$variable" '%s' "$(IFS=:; echo "${filtered[*]}")"
    export "$variable"
  done
  unset COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX
  source "$PRECISION_ROOT/install/px4-1.16/local_setup.bash"
fi
exec ros2 launch pl_bringup "${BRINGUP_LAUNCH:-real.launch.py}" "$@"
