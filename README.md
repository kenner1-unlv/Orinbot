# Orinbot ROS 2 Humble Workspace

This repository hosts the Orinbot ROS 2 Humble workspace. ROS 2 packages live under `src/` (left untouched), while this scaffold adds config, tooling, and data directories for day-to-day development on Jetson Orin and desktop machines.

## Workspace layout

- `src/` — existing ROS 2 packages (not modified by this scaffold).
- `configs/` — YAML parameters and launch files for bringing up sensors, navigation, and tools.
- `tools/` — helper scripts for bag management, PCD export, and timestamp fixes.
- `bags/` — storage location for rosbag recordings (ignored by git).
- `pcd/` — exported point clouds generated from bags (ignored by git).
- `analysis/` — offline analysis utilities and notebooks.

## Prerequisites

- ROS 2 Humble desktop or base install
- `colcon` for building and testing
- Python dependencies for analysis scripts: `rosbag2_py`, `rclpy`, `rosidl-runtime-py`

## Build

```bash
# From the workspace root
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

> This scaffold intentionally omits `build/`, `install/`, and `log/`. They will be created during your first build.

## Run

1. Source your overlay after building:
   ```bash
   source install/setup.bash
   ```
2. Launch your system using a launch file in `configs/launch/` or within a package in `src/`:
   ```bash
   ros2 launch <package> <file.launch.py>
   ```
3. Configure parameters via YAML in `configs/params/` or package-specific config folders.

## Record and replay bags

Record selected topics into `bags/`:
```bash
ros2 bag record -o bags/demo_run /imu/data /points_raw
```

Replay a bag with optional rate and looping:
```bash
./tools/replay_bag.sh bags/demo_run --rate 0.5 --loop
```

## Export point clouds

Capture a short bag and convert to PCD files for inspection:
```bash
./tools/export_pcd.sh /points_raw 10
ls pcd/
```

## Fix timestamps (offline)

Use `tools/fix_timestamps.py` as a starting point for repairing inconsistent header stamps in bags:
```bash
./tools/fix_timestamps.py bags/demo_run --topics /points_raw /imu/data
```
Modify the script to add deserialization and writing logic before applying changes to production data.

## Analysis and notebooks

- Place ad-hoc Python utilities (e.g., synchronization, validation) in `analysis/`.
- Start new notebooks inside `analysis/notebooks/`. Large outputs and checkpoints are ignored by git.
- Example utility: `analysis/imu_lidar_sync.py` prints overlap windows between IMU and LiDAR data.

## Data hygiene

- Avoid committing large rosbags or PCDs. The `.gitignore` keeps these directories clean by default.
- Use descriptive names for recorded bags and exported data to aid future triage.

## Contributing

1. Keep changes to `src/` in their respective packages; place supporting configs in `configs/`.
2. When adding scripts, include a short usage description and make them executable.
3. Document new workflows in the relevant folder `README.md` files.
