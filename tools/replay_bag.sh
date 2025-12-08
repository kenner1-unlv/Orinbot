#!/usr/bin/env bash
set -euo pipefail

# Replay a rosbag with optional loop and rate settings.
# Usage: ./tools/replay_bag.sh <bag_path> [--rate 1.0] [--loop]

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <bag_path> [--rate 1.0] [--loop]" >&2
  exit 1
fi

bag_path=$1
shift || true

ros2 bag play "$bag_path" "$@"
