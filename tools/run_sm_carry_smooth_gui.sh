#!/usr/bin/env bash
# Visible native MuJoCo session; keeps the viewer open after reference completion.
set -eo pipefail
helper_dir=$(cd -- "$(dirname -- "$0")" && pwd)
source "$helper_dir/sm_carry_env.sh"
clip="${1:-203}"
case "$clip" in 203|204|206|207) ;; *) echo 'Choose clip 203, 204, 206 or 207' >&2; exit 2;; esac
workspace="${SM_CARRY_WORKSPACE:-$HOME/sm_carry_box_ws}"
exec 9>"$workspace/.sm_carry_gui.lock"
if ! flock -n 9 || pgrep -x mujoco_sim >/dev/null; then
  echo 'A MuJoCo session is already running. Close its window before starting another.' >&2
  exit 1
fi
assets="$workspace/assets_sm_carry/smooth_ft_$clip"
test -f "$assets/actor.onnx"
result="$workspace/results_sm_carry/gui_smooth_ft_${clip}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$result"
export ROS_LOG_DIR="$result/ros_logs"
python3 "$helper_dir/sm_carry_observe.py" --output "$result" > "$result/observer.log" 2>&1 &
observer_pid=$!
trap 'kill -INT "$observer_pid" 2>/dev/null || true; wait "$observer_pid" 2>/dev/null || true' EXIT
printf 'Smooth FT clip %s. Close the MuJoCo window or Ctrl+C to stop.\nLogs: %s\n' "$clip" "$result"
ros2 launch motion_tracking_controller sm_carry_mujoco.launch.py \
  assets:="$assets" controller_start_delay:=6.0 2>&1 | tee "$result/simulation.log"
