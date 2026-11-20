#!/usr/bin/env bash
set -e

usage() {
    cat <<'EOF'
bash vlp16.sh lidar
bash vlp16.sh map RUN [options]
bash vlp16.sh odom [options]
bash vlp16.sh view
bash vlp16.sh export RUN

map starts ICP odometry and SLAM together. odom starts only ICP odometry.
Each map uses a fresh ~/slam_hw/RUN/RUN.db. export writes RUN.yaml and RUN.pgm beside it.

Options for map and odom:
  --frame FRAME              Robot frame (map: velodyne, odom: base_link)
  --cloud-topic TOPIC        PointCloud2 input (default: /velodyne_points)
  --max-translation METRES   ICP translation limit (Icp/MaxTranslation)
  --match-distance METRES    ICP correspondence distance (Icp/MaxCorrespondenceDistance)
  --voxel-size METRES        ICP voxel size (Icp/VoxelSize)
  --point-to-plane           Use point-to-plane ICP
  --planar                   Constrain odometry to motion on a level floor (default for odom)
EOF
}

die() {
    echo "$*" >&2
    exit 2
}

mode=${1:-help}
case "$mode" in
    help|-h|--help) usage; exit 0 ;;
    lidar|map|odom|view|export) shift ;;
    *) usage >&2; exit 2 ;;
esac
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.sh"
if [[ "$mode" == map || "$mode" == export ]]; then
    run=${1:-}
    [[ "$run" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || die 'Supply a run name such as run3'
    shift
    directory="$HOME/slam_hw/$run"
    db="$directory/$run.db"
fi

case "$mode" in
    lidar|view|export)
        [[ $# == 0 ]] || die "Unexpected argument: $1"
        case "$mode" in
            lidar) exec ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py ;;
            view) exec rviz2 -d "$HMI_PROJECT_DIR/config/vlp16_mapping.rviz" ;;
            export)
                [[ -s "$db" ]] || die "Database missing or empty: $db"
                exec 9<"$directory"
                flock -n 9 || die 'Stop the mapper and wait for the terminal prompt before exporting'
                command -v rtabmap-export >/dev/null || die 'Install the exporter with: sudo apt install ros-jazzy-rtabmap'
                # Export into a fresh directory so an old image cannot pass the output check
                staging=$(mktemp -d "$directory/.export-XXXXXX")
                trap 'rm -rf -- "$staging"' EXIT
                rtabmap-export --map --output_dir "$staging" "$db"
                [[ -s "$staging/$run.pgm" && -s "$staging/$run.yaml" ]] || die 'RTAB-Map did not export an occupancy grid'
                mv -- "$staging/$run.pgm" "$staging/$run.yaml" "$directory/"
                echo "Nav2 map: $directory/$run.yaml"
                exit 0 ;;
        esac ;;
esac

frame=velodyne
planar=false
if [[ "$mode" == odom ]]; then
    frame=base_link
    planar=true
fi
cloud=/velodyne_points
icp=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --frame|--cloud-topic)
            [[ -n "${2:-}" && "$2" != -* ]] || die "Missing value for $1"
            if [[ "$1" == --frame ]]; then frame=$2; else cloud=$2; fi
            shift 2 ;;
        --max-translation|--match-distance|--voxel-size)
            [[ "${2:-}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || die "Supply a distance in metres for $1"
            case "$1" in
                --max-translation) icp+=(--Icp/MaxTranslation "$2") ;;
                --match-distance) icp+=(--Icp/MaxCorrespondenceDistance "$2") ;;
                --voxel-size) icp+=(--Icp/VoxelSize "$2") ;;
            esac
            shift 2 ;;
        --point-to-plane) icp+=(--Icp/PointToPlane true); shift ;;
        --planar) planar=true; shift ;;
        *) die "Unknown option: $1 (see bash vlp16.sh --help)" ;;
    esac
done
if [[ "$planar" == true ]]; then icp+=(--Reg/Force3DoF true); fi

# Give the raw 2D scan to other consumers and use only the 3D cloud for ICP
if [[ "$mode" == odom ]]; then
    exec ros2 run rtabmap_odom icp_odometry "${icp[@]}" --ros-args \
        -p frame_id:="$frame" -p odom_frame_id:=odom -p publish_tf:=true -p qos:=2 \
        -r scan_cloud:="$cloud" -r scan:=/icp_unused_scan -r odom:=/odom
fi

mkdir -p -- "$directory"
exec 9<"$directory"
flock -n 9 || die "Run $run is already in use"
[[ ! -e "$db" ]] || die "Run $run already has a database. Choose a new run name."
echo "Recording map: $db"
# Keep the lock in this shell until the launch process has finished saving
launch_options=()
if (( ${#icp[@]} )); then launch_options+=("odom_args:=${icp[*]}"); fi
ros2 launch rtabmap_launch rtabmap.launch.py \
    frame_id:="$frame" scan_cloud_topic:="$cloud" scan_topic:=/icp_unused_scan \
    subscribe_scan_cloud:=true depth:=false icp_odometry:=true qos:=2 rtabmap_viz:=false \
    'args:=--Reg/Strategy 1 --Grid/Sensor 0 --Grid/RayTracing true' \
    database_path:="$db" "${launch_options[@]}"
