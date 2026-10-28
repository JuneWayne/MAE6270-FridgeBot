#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"

# Use the first argument as the USB camera path, with video0 as the default
HMI_CAMERA_DEVICE="${1:-/dev/video0}"

# Start the image publisher and send Ctrl+C directly to Python
exec python3 direct_camera.py --device "$HMI_CAMERA_DEVICE"
