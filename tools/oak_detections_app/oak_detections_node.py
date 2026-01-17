#!/usr/bin/env python3
"""ROS 2 wrapper for Oak-D spatial detections exporter."""

import argparse
import json
import select
import subprocess
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OakDetectionsNode(Node):
    """Publish Oak-D detections JSON on a ROS 2 topic with ROS timestamps."""

    def __init__(self, oak_script: str, publisher_topic: str) -> None:
        super().__init__("oak_detections_node")
        self._publisher = self.create_publisher(String, publisher_topic, 10)
        self._oak_script = oak_script
        self._process = self._start_process()
        self._timer = self.create_timer(0.01, self._poll_stdout)

    def _start_process(self) -> subprocess.Popen:
        return subprocess.Popen(
            [sys.executable, self._oak_script],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

    def _poll_stdout(self) -> None:
        if self._process.poll() is not None:
            self.get_logger().error("Oak-D exporter exited; shutting down node.")
            rclpy.shutdown()
            return

        if self._process.stdout is None:
            return

        readable, _, _ = select.select([self._process.stdout], [], [], 0.0)
        if not readable:
            return

        line = self._process.stdout.readline()
        if not line:
            return

        payload = line.strip()
        if not payload:
            return

        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            self.get_logger().debug("Skipping non-JSON line from exporter: %s", payload)
            return

        now = self.get_clock().now()
        data["ros_time_ns"] = int(now.nanoseconds)
        data["ros_time_sec"] = float(now.nanoseconds) / 1e9

        msg = String()
        msg.data = json.dumps(data)
        self._publisher.publish(msg)

    def destroy_node(self) -> bool:
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--oak-script",
        default="tools/oak_detections_app/main.py",
        help="Path to the Oak-D exporter script.",
    )
    parser.add_argument(
        "--publisher-topic",
        default="/oak/detections",
        help="ROS 2 topic to publish JSON detections.",
    )
    return parser.parse_known_args()[0]


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = OakDetectionsNode(
        oak_script=args.oak_script,
        publisher_topic=args.publisher_topic,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
