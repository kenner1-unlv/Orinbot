# Oak-D spatial detections exporter

This helper streams YOLOv6-nano spatial detections and IMU metadata from an Oak-D
camera as JSON on stdout. It is intended for Jetson Orin hosts with DepthAI
installed.

## Usage

Run the standalone exporter:

```bash
python3 tools/oak_detections_app/main.py
```

Run the ROS 2 wrapper node (publishes JSON on `/oak/detections` by default):

```bash
python3 tools/oak_detections_app/oak_detections_node.py
```

Override the topic or script path:

```bash
python3 tools/oak_detections_app/oak_detections_node.py \
  --oak-script /abs/path/main.py --publisher-topic /oak/detections
```

## Notes

- Requires the DepthAI Python API (`depthai`) and an Oak-D device.
- The ROS 2 wrapper expects `rclpy` and `std_msgs`.
- Update the configuration constants near the top of `main.py` to adjust
  confidence thresholds, IMU rates, or emission policy.
