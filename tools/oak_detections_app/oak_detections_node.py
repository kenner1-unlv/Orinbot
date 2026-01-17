#!/usr/bin/env python3
"""ROS 2 wrapper node for Oak-D spatial detections JSON output."""

import json
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OakDetectionsNode(Node):
    def __init__(self) -> None:
        super().__init__("oak_detections_node")

        self.declare_parameter("publisher_topic", "/oak/detections")
        self.declare_parameter("oak_script", "")

        topic = self.get_parameter("publisher_topic").get_parameter_value().string_value
        oak_script = self.get_parameter("oak_script").get_parameter_value().string_value

        if oak_script:
            script_path = Path(oak_script)
        else:
            script_path = Path(__file__).resolve().parent / "main.py"

        if not script_path.exists():
            raise FileNotFoundError(f"Oak-D script not found: {script_path}")

        self.publisher = self.create_publisher(String, topic, 10)
        self.process = subprocess.Popen(
            [sys.executable, str(script_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.get_logger().info("Started Oak-D process: %s", script_path)

        self.timer = self.create_timer(0.01, self._poll_stdout)

    def _poll_stdout(self) -> None:
        if self.process.stdout is None:
            return

        line = self.process.stdout.readline()
        if not line:
            return

        payload = line.strip()
        if not payload:
            return

        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            self.get_logger().debug("Skipping non-JSON output: %s", payload)
            return

        now = self.get_clock().now()
        data["ros_time_ns"] = int(now.nanoseconds)
        data["ros_time_sec"] = float(now.nanoseconds) * 1e-9

        msg = String()
        msg.data = json.dumps(data)
        self.publisher.publish(msg)

    def destroy_node(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = OakDetectionsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
