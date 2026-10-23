#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"

# Use the first argument as the Arduino port, with ttyACM0 as the default
HMI_ARDUINO_PORT="${1:-/dev/ttyACM0}"

# Replace this shell with the motor interface so Ctrl+C reaches Python directly
exec python3 robot_motor.py --ros-args -p "port:=$HMI_ARDUINO_PORT"
