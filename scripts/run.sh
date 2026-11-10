#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
cd "$PRECISION_ROOT"
export RUN_LOG_DIR="${RUN_LOG_DIR:-$PRECISION_ROOT/.run}"
mkdir -p "$RUN_LOG_DIR"
export SIM_WORLD="${SIM_WORLD:-aruco_moving}"
case "$SIM_WORLD" in
  aruco) marker_size=0.5 ;;
  aruco_moving) marker_size=1.0 ;;
  *) echo "Unsupported SIM_WORLD: $SIM_WORLD" >&2; exit 1 ;;
esac
# Isolate Gazebo discovery from other simulations on this computer.
export GZ_PARTITION="${GZ_PARTITION:-precision_landing}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [[ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" || ! -f install/precision_land/share/precision_land/package.xml ]]; then
  echo 'Build the example first: ./scripts/setup.sh' >&2
  exit 1
fi
if pgrep -x px4 >/dev/null || ss -lun | grep -q ':8888 '; then
  echo 'Another PX4 or DDS Agent is running. Stop it before starting this example.' >&2
  exit 1
fi
cp "$PRECISION_ROOT/simulation/gz/worlds/aruco_moving.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/"
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT INT TERM
setsid bash -c 'cd "$PX4_DIR/build/px4_sitl_default/rootfs"; PX4_SIM_MODEL=gz_x500_mono_cam_down GZ_IP=127.0.0.1 PX4_GZ_WORLD="$SIM_WORLD" exec ../bin/px4 -d' > "$RUN_LOG_DIR"/px4.log 2>&1 &
pids+=("$!")
# Start the sole Agent through pl_bringup after PX4 has booted. The controller's
# registration timeout is only 15s, so do not start it while Gazebo is loading.
for ((i=0; i<120; i++)); do
  if grep -q 'Startup script returned successfully' "$RUN_LOG_DIR/px4.log"; then break; fi
  if ! kill -0 "${pids[0]}" 2>/dev/null; then cat "$RUN_LOG_DIR/px4.log"; exit 1; fi
  sleep 1
done
if ! grep -q 'Startup script returned successfully' "$RUN_LOG_DIR/px4.log"; then
  echo "PX4 startup timed out. Check $RUN_LOG_DIR/px4.log." >&2
  exit 1
fi
setsid ros2 launch pl_bringup sim.launch.py world:="$SIM_WORLD" marker_size:="$marker_size" > "$RUN_LOG_DIR"/ros.log 2>&1 &
pids+=("$!")
if [[ "${HEADLESS:-}" != 1 ]]; then
  setsid ros2 run rqt_image_view rqt_image_view /image_proc > "$RUN_LOG_DIR"/image_view.log 2>&1 &
  pids+=("$!")
fi
echo "Simulation started. Logs: $RUN_LOG_DIR/{px4,ros}.log (DDS Agent is in ros.log)"
echo 'Select PrecisionLandCustom in QGroundControl after takeoff. Ctrl+C stops this example.'
wait -n "${pids[@]}"
