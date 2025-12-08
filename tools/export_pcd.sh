#!/usr/bin/env bash
set -euo pipefail

# Record a short bag and convert sensor_msgs/PointCloud2 topics to PCD files.
# Usage: ./tools/export_pcd.sh <pointcloud_topic> <duration_seconds>

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="$WS_ROOT/bags"
PCD_DIR="$WS_ROOT/pcd"

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <pointcloud_topic> <duration_seconds>" >&2
  exit 1
fi

topic=$1
record_duration=$2
bag_name="pcd_capture_$(date +%Y%m%d_%H%M%S)"
bag_path="$BAG_DIR/$bag_name"

mkdir -p "$BAG_DIR" "$PCD_DIR"

# Record a short bag with the requested topic
ros2 bag record -o "$bag_path" "$topic" --max-bag-duration "$record_duration"

# Convert to PCD files
ros2 bag play "$bag_path" --loop --remap "$topic:=$topic" \
  | ros2 run pcl_ros pointcloud_to_pcd --prefix "$PCD_DIR/${bag_name}_"

echo "PCD files written to: $PCD_DIR"
