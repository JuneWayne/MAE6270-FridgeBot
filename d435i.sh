#!/usr/bin/env bash
set -e

usage() {
    cat <<'EOF'
bash d435i.sh camera [--profile WIDTHxHEIGHTxFPS]
bash d435i.sh map [RUN] [--frame FRAME] [--planar] [--min-inliers COUNT]
bash d435i.sh odom [--frame FRAME] [--planar] [--min-inliers COUNT]
bash d435i.sh view
bash d435i.sh save
bash d435i.sh export RUN

map starts RGB-D odometry and SLAM. odom runs only RGB-D odometry on /odom.
Both use camera_link by default. Use --frame base_link only with a measured camera transform.
map run3 saves ~/slam_hw/d435i_run3/d435i.db. Omit RUN for an automatic name.
save exports a PNG from the active mapper. export writes a PGM and YAML after mapping.
EOF
}

die() {
    echo "$*" >&2
    exit 2
}

mode=${1:-help}
case "$mode" in
    help|--help|-h) usage; exit 0 ;;
    camera|map|odom|view|save|export) shift ;;
    *) usage >&2; exit 2 ;;
esac
if [[ "${1:-}" == --help ]]; then usage; exit 0; fi
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"
camera=/camera/camera
results="$HOME/slam_hw"

case "$mode" in
    camera)
        profile_args=()
        if (( $# )); then
            [[ $# == 2 && "$1" == --profile && "$2" =~ ^[1-9][0-9]*x[1-9][0-9]*x[1-9][0-9]*$ ]] || die 'Use: bash d435i.sh camera [--profile WIDTHxHEIGHTxFPS]'
            profile_args+=(depth_module.depth_profile:="$2" rgb_camera.color_profile:="$2")
        fi
        # Start depth first and turn on alignment once raw frames arrive
        exec ros2 launch realsense2_camera rs_launch.py enable_sync:=true align_depth.enable:=false "${profile_args[@]}" ;;
    view)
        [[ $# == 0 ]] || die 'Use: bash d435i.sh view'
        # Both mappers publish the same RViz map and cloud topics
        exec rviz2 -d "$HMI_PROJECT_DIR/config/vlp16_mapping.rviz" ;;
    save)
        [[ $# == 0 ]] || die 'Use: bash d435i.sh save'
        database_reply=$(ros2 param get /rtabmap/rtabmap database_path)
        database=${database_reply#String value: }
        [[ "$database" == "$results"/d435i_*/d435i.db ]] || die 'Start the D435i mapper before saving its live map'
        exec ros2 run nav2_map_server map_saver_cli -t /rtabmap/map -f "${database%.db}" --fmt png --ros-args -p save_map_timeout:=15.0 ;;
esac

run=''
if [[ "$mode" != odom && -n "${1:-}" && "$1" != -* ]]; then
    run=$1
    [[ "$run" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || die 'Supply a run name such as run3'
    shift
fi
if [[ "$mode" == export ]]; then
    [[ -n "$run" && $# == 0 ]] || die 'Use: bash d435i.sh export RUN'
    exec bash "$HMI_PROJECT_DIR/export_grid.sh" "$results/d435i_$run/d435i.db"
fi

frame=camera_link
odom_args=()
while (( $# )); do
    case "$1" in
        --frame)
            [[ -n "${2:-}" && "$2" != -* ]] || die 'Supply a frame name'
            frame=$2; shift 2 ;;
        --planar) odom_args+=(--Reg/Force3DoF true); shift ;;
        --min-inliers)
            [[ "${2:-}" =~ ^[1-9][0-9]*$ ]] || die 'Supply a positive integer for --min-inliers'
            odom_args+=(--Vis/MinInliers "$2"); shift 2 ;;
        *) die "Unknown option: $1 (see bash d435i.sh --help)" ;;
    esac
done

if [[ "$mode" == map ]]; then
    mkdir -p -- "$results"
    if [[ -z "$run" ]]; then
        directory=$(mktemp -d "$results/d435i_XXXXXX")
        run=${directory##*/d435i_}
    else
        directory="$results/d435i_$run"
        mkdir -p -- "$directory"
    fi
    exec 9<"$directory"
    flock -n 9 || die "Camera run $run is already in use"
    db="$directory/d435i.db"
    [[ ! -e "$db" ]] || die "Camera run $run already has a database. Choose a new run name."
fi

wait_frame() {
    echo "Waiting for $1"
    timeout 20s ros2 topic echo "$1" "$2" --once --field header --qos-reliability best_effort ||
        die "No frame received on $1. Check the D435i camera terminal."
}

rgb="$camera/color/image_raw"
depth="$camera/aligned_depth_to_color/image_raw"
info="$camera/color/camera_info"
wait_frame "$camera/depth/image_rect_raw" sensor_msgs/msg/Image
wait_frame "$rgb" sensor_msgs/msg/Image
ros2 param set "$camera" align_depth.enable true
wait_frame "$depth" sensor_msgs/msg/Image
wait_frame "$info" sensor_msgs/msg/CameraInfo

if [[ "$mode" == odom ]]; then
    exec ros2 run rtabmap_odom rgbd_odometry "${odom_args[@]}" --ros-args \
        -p frame_id:="$frame" -p odom_frame_id:=odom -p publish_tf:=true \
        -p approx_sync:=true -p qos:=2 -p qos_camera_info:=2 \
        -r rgb/image:="$rgb" -r depth/image:="$depth" -r rgb/camera_info:="$info" -r odom:=/odom
fi

echo "Camera map database: $db"
echo "After the mapper stops, use: bash d435i.sh export $run"
launch_options=()
if (( ${#odom_args[@]} )); then launch_options+=("odom_args:=${odom_args[*]}"); fi
# The shell holds the run lock until RTAB-Map has saved the database
ros2 launch rtabmap_launch rtabmap.launch.py frame_id:="$frame" \
    depth:=true visual_odometry:=true icp_odometry:=false approx_sync:=true \
    rgb_topic:="$rgb" depth_topic:="$depth" camera_info_topic:="$info" \
    rtabmap_viz:=false qos:=2 database_path:="$db" "${launch_options[@]}"
