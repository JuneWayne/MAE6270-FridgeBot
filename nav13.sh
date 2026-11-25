#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"

case "${1:-}" in
  view)
    exec rviz2 -d "$HMI_PROJECT_DIR/config/week13_nav.rviz"
    ;;
  lidar)
    shift
    exec bash "$HMI_PROJECT_DIR/vlp16.sh" lidar "$@"
    ;;
  mount)
    if [[ $# != 7 ]]; then
      echo 'Use: bash nav13.sh mount X Y Z YAW PITCH ROLL (metres and radians)' >&2
      exit 2
    fi
    exec ros2 run tf2_ros static_transform_publisher \
      --frame-id base_link --child-frame-id velodyne \
      --x "$2" --y "$3" --z "$4" --yaw "$5" --pitch "$6" --roll "$7"
    ;;
  odom)
    shift
    exec bash "$HMI_PROJECT_DIR/vlp16.sh" odom "$@"
    ;;
  nav)
    if [[ $# != 3 ]]; then
      echo 'Use: bash nav13.sh nav MAP.yaml ROBOT_RADIUS_METRES' >&2
      exit 2
    fi
    map_file=$(realpath -e -- "$2")
    bringup=$(ros2 pkg prefix --share nav2_bringup)
    params=$(mktemp --suffix=.yaml)
    trap 'rm -f -- "$params"' EXIT
    python3 - "$bringup/params/nav2_params.yaml" "$params" "$3" "$map_file" <<'PY'
import math
import sys
from pathlib import Path

import yaml

defaults, target, radius_text, map_file = sys.argv[1:]
radius = float(radius_text)
if not math.isfinite(radius) or radius <= 0:
    sys.exit("Enter the measured robot radius in metres")
metadata = yaml.safe_load(Path(map_file).read_text())
if not isinstance(metadata, dict) or not metadata.get("image"):
    sys.exit("The map YAML must contain an image entry")
image = Path(map_file).parent / metadata["image"]
if not image.is_file():
    sys.exit(f"Map image is missing: {image}")

config = yaml.safe_load(Path(defaults).read_text())
config["amcl"]["ros__parameters"].update(base_frame_id="base_link", scan_topic="/scan")
for name in ("local_costmap", "global_costmap"):
    costmap = config[name][name]["ros__parameters"]
    costmap.pop("footprint", None)
    costmap["robot_radius"] = radius
    inflation = costmap["inflation_layer"]
    inflation["inflation_radius"] = max(inflation["inflation_radius"], radius + 0.05)

# The Velodyne laser scan is a single plane at the sensor mount height
local = config["local_costmap"]["local_costmap"]["ros__parameters"]
if "voxel_layer" in local["plugins"]:
    layer = local.pop("voxel_layer")
    local["plugins"][local["plugins"].index("voxel_layer")] = "obstacle_layer"
    local["obstacle_layer"] = {
        "plugin": "nav2_costmap_2d::ObstacleLayer",
        "observation_sources": "scan",
        "scan": layer["scan"],
    }

# Use the velocity message type that the motor interface accepts
for name in ("controller_server", "behavior_server", "bt_navigator", "velocity_smoother",
             "collision_monitor", "docking_server"):
    if name in config:
        config[name]["ros__parameters"]["enable_stamped_cmd_vel"] = False
if "collision_monitor" in config:
    monitor = config["collision_monitor"]["ros__parameters"]
    monitor["base_frame_id"] = "base_link"

# Use the full-stick speed limits from the same file as start_base.sh
joystick_config = yaml.safe_load(Path("config/week8_joystick.yaml").read_text())
joystick = joystick_config["/joystick_teleop"]["ros__parameters"]
linear_limit = abs(joystick["linear_scale"])
turn_limit = abs(joystick["angular_scale"])
if not math.isfinite(linear_limit) or not math.isfinite(turn_limit):
    sys.exit("Joystick speed limits must be finite")

smoother = config["velocity_smoother"]["ros__parameters"]
smoother["max_velocity"] = [linear_limit, 0.0, turn_limit]
smoother["min_velocity"] = [-linear_limit, 0.0, -turn_limit]
controller = config["controller_server"]["ros__parameters"]["FollowPath"]
if "MPPIController" in controller["plugin"]:
    controller.update(vx_max=linear_limit, vx_min=-linear_limit, wz_max=turn_limit)
Path(target).write_text(yaml.safe_dump(config, sort_keys=False))
PY
    ros2 launch nav2_bringup bringup_launch.py map:="$map_file" params_file:="$params" \
      slam:=False use_sim_time:=False autostart:=True
    ;;
  *)
    echo 'Use: bash nav13.sh view | lidar | mount X Y Z YAW PITCH ROLL | odom | nav MAP.yaml RADIUS' >&2
    exit 2
    ;;
esac
