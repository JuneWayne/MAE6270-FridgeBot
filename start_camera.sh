#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"

# Use the RealSense color device unless another camera path is supplied
HMI_CAMERA_DEVICE="${1:-/dev/video6}"

# Forward camera settings after the device path
if (( $# > 0 )); then
    shift
fi

# Start the image publisher and send Ctrl+C directly to Python
exec python3 direct_camera.py --device "$HMI_CAMERA_DEVICE" "$@"
