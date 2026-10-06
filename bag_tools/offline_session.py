#!/usr/bin/env python3
"""Run a complete offline session and save message counts and final analysis results.

recorded: replay the saved odometry/TF; raw: recompute with the provided FAST-LIO.
Only processes created by this script are stopped on exit.
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from bag_tool import load_config, validate_bag

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--mode", choices=("recorded", "raw"), default="recorded")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--rviz", action="store_true", help="Start an additional RViz window")
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be positive and finite")
    config = load_config(HERE / "topics.json")
    metadata = validate_bag(args.bag.resolve(), config)
    if not metadata["passed"]:
        parser.error("bag is missing required topics")
    output = args.output or PROJECT / "artifacts" / (args.mode + "_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=False)
    selected = list(metadata["topics"]) if args.mode == "recorded" else [
        topic for topic in config["raw_replay_topics"] if topic in metadata["topics"]]
    rclpy.init()
    node = rclpy.create_node("homework_offline_observer")
    rows, subscriptions, children, files = {}, [], [], []
    result = {"mode": args.mode, "bag": str(args.bag.resolve()), "rate": args.rate,
              "started_at": datetime.datetime.now().astimezone().isoformat(),
              "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
              "commands": [], "topics": rows, "passed": False}

    def start(name, command):
        result["commands"].append(command)
        log = (output / (name + ".log")).open("w")
        files.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(process)
        return process

    def received(msg, name):
        row = rows[name]
        row["received"] += 1
        if hasattr(msg, "header"):
            row["frame"] = msg.header.frame_id
        if name.startswith("/analysis/") and name != "/analysis/imu_si":
            row["last"] = message_to_ordereddict(msg)
        if name == "/tf":
            row["edges"] = sorted(set(row.get("edges", [])) | {
                f"{t.header.frame_id} -> {t.child_frame_id}" for t in msg.transforms})

    try:
        end = time.monotonic() + 2.0
        conflicts = set()
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            for name in selected + ["/clock", "/analysis/imu_stats", "/analysis/odom_stats"]:
                if node.get_publishers_info_by_topic(name):
                    conflicts.add(name)
            if args.mode == "raw" and node.get_publishers_info_by_topic("/Odometry"):
                conflicts.add("/Odometry")
        if conflicts:
            raise RuntimeError(f"Conflicting publishers: {sorted(conflicts)}; stop the current Driver/FAST-LIO/player first")
        observed = {entry["name"]: entry["type"] for entry in config["topics"]
                    if entry["name"] in ("/livox/imu", "/livox/lidar", "/Odometry", "/tf", "/cloud_registered")}
        observed.update({"/analysis/imu_si": "sensor_msgs/msg/Imu",
                         "/analysis/imu_stats": "sensor_analysis/msg/ImuStats",
                         "/analysis/odom_stats": "sensor_analysis/msg/OdomStats",
                         "/analysis/odom_velocity": "geometry_msgs/msg/TwistStamped"})
        for name, msg_type in observed.items():
            rows[name] = {"type": msg_type, "received": 0}
            subscriptions.append(node.create_subscription(get_message(msg_type), name,
                lambda msg, topic=name: received(msg, topic),
                QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT)))
        analysis = start("analysis", ["ros2", "launch", "sensor_analysis", "analysis.launch.py", "use_sim_time:=true"])
        fastlio = None
        if args.mode == "raw":
            fastlio = start("fastlio", ["ros2", "run", "fast_lio", "fastlio_mapping", "--ros-args",
                "--params-file", str(PROJECT / "FAST_LIO/config/mid360.yaml"),
                "-p", "use_sim_time:=true", "-p", "pcd_save.pcd_save_en:=false"])
        if args.rviz:
            start("rviz", ["rviz2", "-d", str(PROJECT / "rviz/homework.rviz"),
                           "--ros-args", "-p", "use_sim_time:=true"])
        ready = time.monotonic() + 15
        while time.monotonic() < ready:
            rclpy.spin_once(node, timeout_sec=0.1)
            if analysis.poll() is not None or (fastlio is not None and fastlio.poll() is not None):
                raise RuntimeError("Analysis/FAST-LIO exited during startup; inspect session logs")
            if all(node.get_publishers_info_by_topic(name) for name in
                   ["/analysis/imu_stats", "/analysis/odom_stats"] + (["/Odometry"] if fastlio else [])):
                break
        else:
            raise RuntimeError("Nodes did not become ready within 15 seconds")
        player = start("player", ["ros2", "bag", "play", str(args.bag.resolve()),
            "--clock", "100", "--rate", str(args.rate), "--delay", "3", "--topics", *selected])
        deadline = time.monotonic() + metadata["duration_seconds"] / args.rate + 60
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
            if analysis.poll() is not None or (fastlio is not None and fastlio.poll() is not None):
                raise RuntimeError("Analysis/FAST-LIO exited during playback")
            if time.monotonic() > deadline:
                raise RuntimeError("Playback exceeded the expected duration")
        drain = time.monotonic() + 3
        while time.monotonic() < drain:
            rclpy.spin_once(node, timeout_sec=0.02)
        result["player_exit_code"] = player.returncode
        result["missing_topics"] = [name for name, row in rows.items() if not row["received"]]
        last_imu = rows["/analysis/imu_stats"].get("last", {})
        last_odom = rows["/analysis/odom_stats"].get("last", {})
        result["input_imu_expected"] = metadata["topics"]["/livox/imu"]["messages"]
        result["imu_processed"] = rows["/analysis/imu_si"]["received"]
        result["odom_processed"] = last_odom.get("received_samples", 0)
        result["input_complete"] = (result["imu_processed"] == result["input_imu_expected"] and
            (args.mode == "raw" or result["odom_processed"] == metadata["topics"]["/Odometry"]["messages"]))
        result["passed"] = player.returncode == 0 and not result["missing_topics"] and result["input_complete"]
        if last_imu.get("rejected_samples", 0) or last_odom.get("rejected_samples", 0):
            result["data_warning"] = "Non-finite/invalid sensor samples were rejected; see counters"
    except (RuntimeError, KeyboardInterrupt) as error:
        result["error"] = str(error) or "Interrupted by user"
    finally:
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        for log in files:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("Session evidence:", output)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
