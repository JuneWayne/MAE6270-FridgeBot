#!/usr/bin/env bash
set -e

# Give each program a process group so Ctrl+C can also stop its child processes
set -m
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"

# Use the first argument as the joystick device ID, with zero as the default
HMI_JOY_DEVICE_ID="${1:-0}"
HMI_JOY_PID=''
HMI_TELEOP_PID=''

# Stop both programs when this script stops or either program exits
cleanup() {
    if [[ -n "$HMI_TELEOP_PID" ]]; then
        kill -INT -- "-$HMI_TELEOP_PID" 2>/dev/null || true
    fi
    if [[ -n "$HMI_JOY_PID" ]]; then
        kill -INT -- "-$HMI_JOY_PID" 2>/dev/null || true
    fi
    if [[ -n "$HMI_TELEOP_PID" ]]; then
        wait "$HMI_TELEOP_PID" 2>/dev/null || true
    fi
    if [[ -n "$HMI_JOY_PID" ]]; then
        wait "$HMI_JOY_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Keep joystick input active while the stick position stays constant
ros2 run joy joy_node --ros-args \
    -p "device_id:=$HMI_JOY_DEVICE_ID" \
    -p autorepeat_rate:=20.0 \
    -p deadzone:=0.10 &
HMI_JOY_PID=$!

# Read the saved axis settings and publish velocity commands
python3 week8_joystick_teleop.py --ros-args \
    --params-file config/week8_joystick.yaml &
HMI_TELEOP_PID=$!

printf 'Base programs started on ROS domain %s; press Ctrl+C to stop\n' "$ROS_DOMAIN_ID"

# Stop the other program if either one exits
wait -n "$HMI_JOY_PID" "$HMI_TELEOP_PID"
