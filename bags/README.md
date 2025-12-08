# Bag Storage

Place rosbag recordings here. Keep file names descriptive, for example `2024-04-20_garage_mapping`. Large bag files should be kept out of version control.

Tips:
- Record with `ros2 bag record -o bags/<name> <topics...>`.
- Use `../tools/replay_bag.sh` to play bags during development.
