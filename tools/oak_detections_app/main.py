#!/usr/bin/env python3
"""Oak-D depth + IMU tracklet exporter.

Runs a DepthAI spatial detection network (YOLOv6-nano), tracks objects, and
emits spatial bounding boxes with optional IMU metadata as JSON on stdout.
"""

import depthai as dai
import json
import math
import time
from collections import deque

# ----------------------------
# Config
# ----------------------------
IMU_HZ_GYRO = 400
IMU_HZ_ACCEL = 400
IMU_HZ_RV = 100

IMU_BUFFER_SECONDS = 2.0
IMU_MATCH_WINDOW_MS = 30.0

# Stereo / Spatial
LR_CHECK = True
EXTENDED_DISPARITY = False
SUBPIXEL = True

CONF_THRESH = 0.50
DEPTH_LOWER_MM = 200
DEPTH_UPPER_MM = 20000

# NN input size used for bbox_px conversion (matches yolov6-nano default)
NN_W, NN_H = 512, 384

# Emission policy
CONF_GATE = 0.50  # only emit records with conf >= this (when conf is available)
DROP_UNKNOWN_CONF = False  # True = if we can't get conf, drop it; False = allow it through

EMIT_NEW = True
EMIT_TRACKED = True
EMIT_LOST = True  # emit LOST (but only if was previously active)
EMIT_REMOVED = True  # emit REMOVED once per id


# ----------------------------
# Timestamp helper
# ----------------------------
def ts_to_ms(ts) -> float:
    if ts is None:
        return None

    # datetime.timedelta
    if hasattr(ts, "total_seconds"):
        return ts.total_seconds() * 1000.0

    # depthai.Timestamp usually has .get() returning timedelta
    if hasattr(ts, "get"):
        v = ts.get()
        if hasattr(v, "total_seconds"):
            return v.total_seconds() * 1000.0
        return float(v) * 1000.0

    # fallback struct with sec/nsec
    if hasattr(ts, "sec") and hasattr(ts, "nsec"):
        return (float(ts.sec) + float(ts.nsec) * 1e-9) * 1000.0

    raise TypeError(f"Unknown timestamp type: {type(ts)}")


# ----------------------------
# IMU helpers (scaled ints)
# ----------------------------
def gyro_to_mdps(g):
    return [int(math.degrees(x) * 1000.0) for x in g]


def accel_to_mg(a):
    return [int((x / 9.80665) * 1000.0) for x in a]


def imu_packet_to_dict(pkt) -> dict:
    d = {}
    ts = None

    if hasattr(pkt, "gyroscope") and pkt.gyroscope is not None:
        g = pkt.gyroscope
        d["gyro_mdps"] = gyro_to_mdps([float(g.x), float(g.y), float(g.z)])
        ts = g.timestamp

    if hasattr(pkt, "acceleroMeter") and pkt.acceleroMeter is not None:
        a = pkt.acceleroMeter
        d["accel_mg"] = accel_to_mg([float(a.x), float(a.y), float(a.z)])
        ts = ts or a.timestamp

    if hasattr(pkt, "rotationVector") and pkt.rotationVector is not None:
        rv = pkt.rotationVector
        q = None
        if hasattr(rv, "real"):  # i,j,k,real
            q = [float(rv.i), float(rv.j), float(rv.k), float(rv.real)]
            ts = ts or rv.timestamp
        elif hasattr(rv, "w"):  # x,y,z,w
            q = [float(rv.x), float(rv.y), float(rv.z), float(rv.w)]
            ts = ts or rv.timestamp
        if q is not None:
            d["rotvec_q16"] = [int(x * 32767) for x in q]

    if ts is not None:
        d["ts_device_ms"] = ts_to_ms(ts)

    return d


def find_imu_for_ts(imu_buf: deque, target_ms: float, window_ms: float):
    if not imu_buf or target_ms is None:
        return None

    lo, hi = target_ms - window_ms, target_ms + window_ms
    in_win = [s for s in imu_buf if lo <= s["ts_device_ms"] <= hi]

    if not in_win:
        nearest = min(imu_buf, key=lambda s: abs(s["ts_device_ms"] - target_ms))
        if abs(nearest["ts_device_ms"] - target_ms) <= 100.0:
            return nearest
        return None

    if len(in_win) == 1:
        return in_win[0]

    out = {"ts_device_ms": int(target_ms), "n": len(in_win)}

    def avg_int_vec(key):
        vecs = [s[key] for s in in_win if key in s]
        if not vecs:
            return None
        return [
            int(round(sum(v[0] for v in vecs) / len(vecs))),
            int(round(sum(v[1] for v in vecs) / len(vecs))),
            int(round(sum(v[2] for v in vecs) / len(vecs))),
        ]

    g = avg_int_vec("gyro_mdps")
    a = avg_int_vec("accel_mg")
    if g is not None:
        out["gyro_mdps"] = g
    if a is not None:
        out["accel_mg"] = a

    q = [s for s in in_win if "rotvec_q16" in s]
    if q:
        nearest_q = min(q, key=lambda s: abs(s["ts_device_ms"] - target_ms))
        out["rotvec_q16"] = nearest_q["rotvec_q16"]

    return out


# ----------------------------
# Confidence extraction helper
# ----------------------------
def tracklet_confidence(t):
    # Varies by DepthAI build; try the common places
    if hasattr(t, "srcImgDetection") and t.srcImgDetection is not None:
        return float(getattr(t.srcImgDetection, "confidence", 0.0))
    if hasattr(t, "confidence"):
        return float(getattr(t, "confidence", 0.0))
    return None


# ----------------------------
# Main
# ----------------------------
with dai.Pipeline() as pipeline:
    # Color camera
    cam = pipeline.create(dai.node.Camera).build()

    # Stereo inputs
    monoL = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoR = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo.setRectification(True)
    stereo.setLeftRightCheck(LR_CHECK)
    stereo.setExtendedDisparity(EXTENDED_DISPARITY)
    stereo.setSubpixel(SUBPIXEL)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_DENSITY)

    monoL.requestOutput((1280, 720)).link(stereo.left)
    monoR.requestOutput((1280, 720)).link(stereo.right)

    # Spatial detection
    sdn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
        cam, stereo, dai.NNModelDescription("yolov6-nano")
    )
    label_map = sdn.getClasses()
    sdn.setConfidenceThreshold(CONF_THRESH)
    sdn.setDepthLowerThreshold(DEPTH_LOWER_MM)
    sdn.setDepthUpperThreshold(DEPTH_UPPER_MM)

    # Object Tracker (RVC4 supports SHORT_TERM_IMAGELESS)
    tracker = pipeline.create(dai.node.ObjectTracker)
    tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
    tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

    sdn.passthrough.link(tracker.inputTrackerFrame)
    sdn.out.link(tracker.inputDetections)

    q_track = tracker.out.createOutputQueue(maxSize=8, blocking=False)

    # IMU
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, IMU_HZ_GYRO)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, IMU_HZ_ACCEL)
    if IMU_HZ_RV > 0:
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, IMU_HZ_RV)

    imu.setBatchReportThreshold(10)
    imu.setMaxBatchReports(50)
    q_imu = imu.out.createOutputQueue(maxSize=50, blocking=False)

    pipeline.start()

    # ----------------------------
    # Host state
    # ----------------------------
    imu_buf = deque()
    seq = 0

    last_status = {}  # tid -> last status emitted
    was_active = {}  # tid -> True if we've emitted NEW/TRACKED at least once
    removed_once = set()  # tids we've already emitted REMOVED for (ever)

    while pipeline.isRunning():
        # Drain IMU
        imu_data = q_imu.tryGet()
        while imu_data is not None:
            for pkt in imu_data.packets:
                s = imu_packet_to_dict(pkt)
                if "ts_device_ms" in s:
                    imu_buf.append(s)
            imu_data = q_imu.tryGet()

        # Prune IMU buffer
        if imu_buf:
            newest = imu_buf[-1]["ts_device_ms"]
            cutoff = newest - (IMU_BUFFER_SECONDS * 1000.0)
            while imu_buf and imu_buf[0]["ts_device_ms"] < cutoff:
                imu_buf.popleft()

        # Tracklets
        track_msg = q_track.tryGet()
        if track_msg is None:
            time.sleep(0.001)
            continue

        ts_ms = ts_to_ms(track_msg.getTimestamp())
        imu_match = find_imu_for_ts(imu_buf, ts_ms, IMU_MATCH_WINDOW_MS)

        out = []
        for t in track_msg.tracklets:
            tid = int(t.id)
            status = str(t.status).split(".")[-1]  # TRACKED/LOST/NEW/REMOVED

            # Per-status allow list (early)
            if status == "NEW" and not EMIT_NEW:
                continue
            if status == "TRACKED" and not EMIT_TRACKED:
                continue
            if status == "LOST" and not EMIT_LOST:
                continue
            if status == "REMOVED" and not EMIT_REMOVED:
                continue

            # only emit each REMOVED once per id
            if status == "REMOVED":
                if tid in removed_once:
                    continue
                removed_once.add(tid)

            # status-change gate (primary anti-spam)
            prev = last_status.get(tid)
            if prev == status:
                continue
            last_status[tid] = status

            # confidence gate
            conf = tracklet_confidence(t)
            if conf is None:
                if DROP_UNKNOWN_CONF:
                    continue
            else:
                if conf < CONF_GATE:
                    continue

            # suppress LOST unless it was previously "active"
            if status == "LOST" and not was_active.get(tid, False):
                continue

            # mark active on NEW/TRACKED
            if status in ("NEW", "TRACKED"):
                was_active[tid] = True

            lbl = (
                label_map[int(t.label)]
                if int(t.label) < len(label_map)
                else str(int(t.label))
            )

            roi = t.roi
            bbox_px = [
                int(roi.topLeft().x * NN_W),
                int(roi.topLeft().y * NN_H),
                int(roi.bottomRight().x * NN_W),
                int(roi.bottomRight().y * NN_H),
            ]

            xyz = t.spatialCoordinates
            rec = {
                "id": tid,
                "status": status,
                "label": lbl,
                "bbox_px": bbox_px,
                "xyz_mm": [int(xyz.x), int(xyz.y), int(xyz.z)],
            }
            if conf is not None:
                rec["conf"] = round(float(conf), 2)

            out.append(rec)

            # cleanup on REMOVED (so a future re-use of tid can emit again)
            if status == "REMOVED":
                last_status.pop(tid, None)
                was_active.pop(tid, None)

        if not out:
            continue

        print(
            "\n"
            + json.dumps(
                {
                    "seq": seq,
                    "ts_device_ms": int(ts_ms) if ts_ms is not None else None,
                    "n": len(out),
                    "tracklets": out,
                    "imu": imu_match,
                }
            )
        )
        seq += 1
