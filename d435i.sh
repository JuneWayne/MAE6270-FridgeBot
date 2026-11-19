#!/usr/bin/env bash
set -e

# Use the same ROS domain as the motor and gamepad programs
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"
camera=/camera/camera
results="$HOME/slam_hw"

case "${1:-}" in
  camera)
    # Start with the configuration that produced depth frames on the ThinkPad
    exec ros2 launch realsense2_camera rs_launch.py enable_sync:=true align_depth.enable:=false
    ;;
  map)
    # Check the camera streams before starting a new map
    for topic in "$camera/depth/image_rect_raw" "$camera/color/image_raw"; do
      echo "Waiting for $topic"
      if ! timeout 20s ros2 topic echo "$topic" sensor_msgs/msg/Image --once --field header --qos-reliability best_effort; then
        echo "No frame received on $topic. Check the camera terminal." >&2
        exit 1
      fi
    done
    ros2 param set "$camera" align_depth.enable true
    echo 'Waiting for aligned depth'
    if ! timeout 20s ros2 topic echo "$camera/aligned_depth_to_color/image_raw" sensor_msgs/msg/Image --once --field header --qos-reliability best_effort; then
      echo 'No aligned frame received. Check the camera terminal.' >&2
      exit 1
    fi
    mkdir -p "$results"
    run_dir=$(mktemp -d "$results/d435i_XXXXXX")
    echo "Camera map database: $run_dir/d435i.db"
    exec ros2 launch rtabmap_launch rtabmap.launch.py frame_id:=camera_link \
      rgb_topic:="$camera/color/image_raw" depth_topic:="$camera/aligned_depth_to_color/image_raw" \
      camera_info_topic:="$camera/color/camera_info" rtabmap_viz:=false qos:=2 database_path:="$run_dir/d435i.db"
    ;;
  save)
    # Save the image beside the database of the active camera mapper
    database_reply=$(ros2 param get /rtabmap/rtabmap database_path)
    database=${database_reply#String value: }
    if [[ "$database" != "$results"/d435i_*/d435i.db ]]; then
      echo 'Start the camera mapper with bash d435i.sh map before saving.' >&2
      exit 1
    fi
    exec ros2 run nav2_map_server map_saver_cli -t /rtabmap/map -f "${database%.db}" --fmt png --ros-args -p save_map_timeout:=15.0
    ;;
  *)
    echo 'Use bash d435i.sh camera, bash d435i.sh map, or bash d435i.sh save.' >&2
    exit 2
    ;;
esac
