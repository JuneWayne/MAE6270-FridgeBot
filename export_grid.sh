#!/usr/bin/env bash
set -e

# Both sensor launchers call this after loading the ROS environment
db=${1:?Supply the path to the RTAB-Map database}
[[ -s "$db" ]] || { echo "Database missing or empty: $db" >&2; exit 2; }
directory=$(dirname -- "$db")
name=$(basename -- "$db" .db)
exec 9<"$directory"
flock -n 9 || { echo 'Stop the mapper and wait for its terminal prompt before exporting' >&2; exit 2; }
command -v rtabmap-export >/dev/null || { echo 'Install the exporter with: sudo apt install ros-jazzy-rtabmap' >&2; exit 2; }

# A fresh directory prevents a previous export from passing the output check
staging=$(mktemp -d "$directory/.export-XXXXXX")
trap 'rm -rf -- "$staging"' EXIT
rtabmap-export --map --output_dir "$staging" "$db"
if [[ ! -s "$staging/$name.pgm" || ! -s "$staging/$name.yaml" ]]; then
    echo 'RTAB-Map did not export an occupancy grid' >&2
    exit 2
fi
mv -- "$staging/$name.pgm" "$staging/$name.yaml" "$directory/"
echo "Nav2 map: $directory/$name.yaml"
