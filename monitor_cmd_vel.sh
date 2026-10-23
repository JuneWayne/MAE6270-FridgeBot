#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"

# Display velocity commands continuously on the team ROS domain
exec ros2 topic echo /cmd_vel
