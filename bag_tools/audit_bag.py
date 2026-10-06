#!/usr/bin/env python3
"""Read-only SQLite/CDR audit of a MID360 bag; writes only a separate report."""
import argparse
import json
import math
from pathlib import Path
import sqlite3
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def audit(bag):
    metadata = rosbag2_py.Info().read_metadata(str(bag), "sqlite3")
    duration = metadata.duration.nanoseconds / 1e9
    report = {"bag": str(bag), "duration_seconds": duration,
              "message_count": metadata.message_count, "topics": {}}
    imu_norms, gyro_norms, positions, imu_stamps, odom_stamps = [], [], [], [], []
    odom_by_stamp, transforms = {}, []
    for relative in metadata.relative_file_paths:
        path = bag / relative
        connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        try:
            for topic_id, name, msg_type in connection.execute("SELECT id,name,type FROM topics"):
                count, first, last = connection.execute(
                    "SELECT count(*),min(timestamp),max(timestamp) FROM messages WHERE topic_id=?",
                    (topic_id,)).fetchone()
                row = report["topics"].setdefault(name, {"type": msg_type, "messages": 0,
                                                       "frames": [], "samples": []})
                row["messages"] += count
                if not count:
                    continue
                row["first_record_ns"] = min(first, row.get("first_record_ns", first))
                row["last_record_ns"] = max(last, row.get("last_record_ns", last))
                detailed = name in ("/livox/imu", "/Odometry", "/tf")
                query = "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp"
                if not detailed:
                    query += " LIMIT 3"
                message_class = get_message(msg_type)
                for record_ns, data in connection.execute(query, (topic_id,)):
                    msg = deserialize_message(data, message_class)
                    if hasattr(msg, "header"):
                        frame = msg.header.frame_id
                        if frame not in row["frames"]:
                            row["frames"].append(frame)
                        header_ns = stamp_ns(msg.header.stamp)
                        if len(row["samples"]) < 3:
                            sample = {"header_ns": header_ns, "record_ns": record_ns, "frame": frame}
                            if hasattr(msg, "point_num"):
                                sample.update(point_count=msg.point_num, timebase=msg.timebase)
                            if hasattr(msg, "width"):
                                sample.update(point_count=msg.width * msg.height,
                                              fields=[field.name for field in msg.fields])
                            row["samples"].append(sample)
                    if name == "/livox/imu":
                        a, g = msg.linear_acceleration, msg.angular_velocity
                        imu_norms.append(math.hypot(a.x, a.y, a.z))
                        gyro_norms.append(math.hypot(g.x, g.y, g.z))
                        imu_stamps.append(header_ns)
                    elif name == "/Odometry":
                        p = msg.pose.pose.position
                        q = msg.pose.pose.orientation
                        positions.append((p.x, p.y, p.z))
                        odom_stamps.append(header_ns)
                        odom_by_stamp[header_ns] = (positions[-1], (q.x, q.y, q.z, q.w))
                        row["child_frame_id"] = msg.child_frame_id
                        row["twist_all_zero"] = row.get("twist_all_zero", True) and all(
                            getattr(vector, axis) == 0 for vector in
                            (msg.twist.twist.linear, msg.twist.twist.angular) for axis in "xyz")
                    elif name == "/tf":
                        for transform in msg.transforms:
                            edge = f"{transform.header.frame_id} -> {transform.child_frame_id}"
                            if edge not in row["frames"]:
                                row["frames"].append(edge)
                            transforms.append(transform)
        finally:
            connection.close()
    for row in report["topics"].values():
        span = (row.get("last_record_ns", 0) - row.get("first_record_ns", 0)) / 1e9
        row["record_rate_hz"] = (row["messages"] - 1) / span if span > 0 else 0
    for name, stamps in (("/livox/imu", imu_stamps), ("/Odometry", odom_stamps)):
        intervals = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
        report["topics"][name]["header_timing"] = {
            "nonpositive_intervals": sum(dt <= 0 for dt in intervals),
            "mean_dt_seconds": statistics.mean(intervals) if intervals else 0,
            "max_dt_seconds": max(intervals, default=0),
            "sensor_rate_hz": (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)
                              if len(stamps) > 1 and stamps[-1] > stamps[0] else 0}
    report["imu"] = {"driver_acceleration_unit": "g (verified in driver source)",
                     "accel_norm_mean_g": statistics.mean(imu_norms),
                     "accel_norm_std_g": statistics.pstdev(imu_norms),
                     "gyro_norm_rms_radps": math.sqrt(statistics.mean(x*x for x in gyro_norms)),
                     "note": "Acceleration includes gravity; identity orientation is not a measured attitude."}
    steps = [math.dist(a, b) for a, b in zip(positions, positions[1:])]
    report["odometry"] = {"unfiltered_path_length_m": sum(steps),
                          "displacement_m": math.dist(positions[0], positions[-1]),
                          "max_step_m": max(steps, default=0),
                          "note": "Position estimate, not ground truth; derived distance includes localization noise."}
    matched, max_position_error, max_quaternion_error = 0, 0.0, 0.0
    for transform in transforms:
        if transform.header.frame_id != "camera_init" or transform.child_frame_id != "body":
            continue
        pair = odom_by_stamp.get(stamp_ns(transform.header.stamp))
        if pair is None:
            continue
        p, q = transform.transform.translation, transform.transform.rotation
        position, rotation = pair
        matched += 1
        max_position_error = max(max_position_error, math.dist(position, (p.x, p.y, p.z)))
        vector = (q.x, q.y, q.z, q.w)
        max_quaternion_error = max(max_quaternion_error,
                                  min(math.dist(rotation, vector), math.dist(rotation, tuple(-x for x in vector))))
    report["tf_odometry_consistency"] = {"matched_timestamps": matched,
        "max_position_error_m": max_position_error, "max_quaternion_error": max_quaternion_error}
    report["static_tf_present"] = report["topics"].get("/tf_static", {}).get("messages", 0) > 0
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.bag.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "topics"},
                     ensure_ascii=False, indent=2))
    print("Report:", args.output)


if __name__ == "__main__":
    main()
