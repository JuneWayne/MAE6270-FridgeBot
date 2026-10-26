#!/usr/bin/env bash

# Find the project folder from this file instead of the current terminal folder
HMI_PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return 1

# Load ROS before setting the team network values
source /opt/ros/jazzy/setup.bash || return 1
export ROS_DOMAIN_ID=3
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Use the project folder so Python can find the modules and configuration
cd -- "$HMI_PROJECT_DIR" || return 1
