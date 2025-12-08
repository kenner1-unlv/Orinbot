# Configurations and Launch Files

Use this directory for ROS 2 Humble configuration YAML files and launch scripts. Suggested layout:

- `params/` for controller, perception, and hardware parameter files.
- `launch/` for `*.launch.py` entrypoints that compose nodes and pass configs.

Add new configurations alongside the relevant package in `src/` and keep file names descriptive (e.g., `lidar_driver.yaml`, `navigation.launch.py`).
