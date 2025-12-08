#!/usr/bin/env python3
"""Skeleton script for aligning IMU and LiDAR timestamps.

Customize the topic names and synchronization logic as needed for your sensors.
"""

from __future__ import annotations

import argparse
import pathlib
from typing import Iterable

import rclpy  # type: ignore
from rclpy.serialization import deserialize_message  # type: ignore
from rosidl_runtime_py.utilities import get_message  # type: ignore
import rosbag2_py  # type: ignore


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", type=pathlib.Path, help="Path to rosbag directory")
    parser.add_argument("--imu-topic", default="/imu/data", help="IMU topic name")
    parser.add_argument("--lidar-topic", default="/points_raw", help="LiDAR point cloud topic")
    return parser.parse_args()


def open_bag(bag_path: pathlib.Path) -> tuple[rosbag2_py.SequentialReader, dict[str, str]]:
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Extract topic type information for deserialization
    topics = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topics}
    return reader, type_map


def iterate_messages(reader: rosbag2_py.SequentialReader) -> Iterable[tuple[str, int, bytes]]:
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield topic, t, data


def main() -> int:
    args = parse_args()
    rclpy.init(args=None)

    reader, type_map = open_bag(args.bag_path)

    imu_stamps = []
    lidar_stamps = []

    for topic, timestamp, raw in iterate_messages(reader):
        if topic not in (args.imu_topic, args.lidar_topic):
            continue

        msg_cls = get_message(type_map[topic])
        msg = deserialize_message(raw, msg_cls)

        if hasattr(msg, "header"):
            secs = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            secs = timestamp * 1e-9

        if topic == args.imu_topic:
            imu_stamps.append(secs)
        else:
            lidar_stamps.append(secs)

    if imu_stamps and lidar_stamps:
        imu_range = (min(imu_stamps), max(imu_stamps))
        lidar_range = (min(lidar_stamps), max(lidar_stamps))
        print(f"IMU window:   {imu_range[0]:.3f}s to {imu_range[1]:.3f}s")
        print(f"LiDAR window: {lidar_range[0]:.3f}s to {lidar_range[1]:.3f}s")
        overlap_start = max(imu_range[0], lidar_range[0])
        overlap_end = min(imu_range[1], lidar_range[1])
        if overlap_end > overlap_start:
            print(f"Overlap: {overlap_end - overlap_start:.3f}s")
        else:
            print("No overlapping time window detected.")
    else:
        print("IMU or LiDAR stamps not found; check topic names.")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
