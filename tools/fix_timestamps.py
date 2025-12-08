#!/usr/bin/env python3
"""Normalize ROS 2 message timestamps inside a bag.

This script is intended for quick offline fixes when loggers or sensors
produce inconsistent header stamps. It iterates over messages and rewrites
`header.stamp` using a monotonic sequence. Customize the topic list and
conversion logic for your sensors before running.
"""

from __future__ import annotations

import argparse
import pathlib
import sys
from typing import Iterable

import rosbag2_py  # type: ignore


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", type=pathlib.Path, help="Path to the rosbag (directory)")
    parser.add_argument(
        "--topics",
        nargs="*",
        default=None,
        help="Optional whitelist of topics to rewrite; defaults to all topics with header stamps.",
    )
    return parser.parse_args()


def load_reader(bag_path: pathlib.Path) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def iter_messages(reader: rosbag2_py.SequentialReader) -> Iterable[tuple[str, int, bytes]]:
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield topic, t, data


def main() -> int:
    args = parse_args()
    reader = load_reader(args.bag_path)

    selected_topics = set(args.topics) if args.topics else None

    for topic, timestamp, _data in iter_messages(reader):
        if selected_topics and topic not in selected_topics:
            continue
        # Placeholder for rewrite logic. Extend this block to deserialize messages
        # (using the topic type information) and assign new header stamps.
        # Example: msg.header.stamp = rclpy.time.Time(seconds=...).to_msg()
        print(f"[INFO] Would rewrite {topic} at {timestamp}")

    print("Inspection complete. Implement serialization + writer as needed before running on real data.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
