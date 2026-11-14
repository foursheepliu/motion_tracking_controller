#!/usr/bin/env bash
# Starts hardware/state estimation with BOTH motor controllers inactive.
set -euo pipefail
carry_workspace="${SM_CARRY_WORKSPACE:-$HOME/sm_carry_box_ws}"
carry_image="${SM_CARRY_IMAGE:-qiayuanl/unitree:g1-carry-arm64-20261114}"
carry_name="${SM_CARRY_CONTAINER:-g1-carry-real}"
carry_tracking="${SM_CARRY_ENABLE_TRACKING:-false}"
case "$carry_tracking" in true|false) ;; *) echo 'SM_CARRY_ENABLE_TRACKING must be true or false' >&2; exit 2;; esac
if [ "$(docker inspect -f '{{.State.Running}}' g1-real-stage1 2>/dev/null || true)" = true ]; then
  echo 'The previous real controller is running. Stop it safely before starting carry.' >&2; exit 1
fi
if docker inspect "$carry_name" >/dev/null 2>&1; then
  echo "Container $carry_name already exists; inspect its state and logs before restarting." >&2; exit 1
fi
test -f "$carry_workspace/assets_sm_carry/full203/actor.onnx"
test -f "$carry_workspace/install/setup.bash"
carry_run="real_carry/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$carry_workspace/$carry_run"
printf '%s\n' "$carry_run" > "$carry_workspace/latest_real_carry_run.txt"
docker run -d --name "$carry_name" --network host --stop-timeout 8 \
  --cgroup-parent=/ --cpu-rt-runtime=950000 --cap-add SYS_NICE --cap-add IPC_LOCK \
  --ulimit rtprio=99 --ulimit memlock=-1 \
  -e ROS_DOMAIN_ID=97 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e ROS_LOG_DIR="/sm_carry_ws/$carry_run/ros_logs" \
  -v "$carry_workspace:/sm_carry_ws" -w /sm_carry_ws "$carry_image" \
  bash -lc 'source /sm_carry_ws/install/setup.bash; exec ros2 launch motion_tracking_controller sm_carry_real.launch.py \
    assets:=/sm_carry_ws/assets_sm_carry/full203 output:="$1" enable_tracking:="$2" network_interface:=eth0' \
  bash "/sm_carry_ws/$carry_run" "$carry_tracking"
