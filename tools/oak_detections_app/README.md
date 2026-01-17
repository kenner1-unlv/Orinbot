# Oak-D spatial detections exporter

This helper streams YOLOv6-nano spatial detections and IMU metadata from an Oak-D
camera as JSON on stdout. It is intended for Jetson Orin hosts with DepthAI
installed.

## Usage

Run the standalone exporter:

```bash
python3 tools/oak_detections_app/main.py
```

## Notes

- Requires the DepthAI Python API (`depthai`) and an Oak-D device.
- Update the configuration constants near the top of `main.py` to adjust
  confidence thresholds, IMU rates, or emission policy.
