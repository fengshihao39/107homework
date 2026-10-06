#!/usr/bin/env python3
"""MID360 / FAST-LIO 的 rosbag 录制、检查及回放工具（ROS 2 Humble）。"""

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须是大于 0 的有限数值")
    return number


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("必须是大于等于 0 的有限数值")
    return number


def load_config(path):
    config = json.loads(path.read_text(encoding="utf-8"))
    topics = config["topics"]
    names = [entry["name"] for entry in topics]
    if not topics or len(set(names)) != len(names):
        raise ValueError("Topic 列表为空或包含重复项")
    for entry in topics:
        if not entry["name"].startswith("/") or "/msg/" not in entry["type"]:
            raise ValueError("配置需要绝对 Topic 名称和完整消息类型")
        if not isinstance(entry["required"], bool):
            raise ValueError("required 必须是布尔值")
    if not set(config["raw_replay_topics"]).issubset(names):
        raise ValueError("raw_replay_topics 必须属于 topics 列表")
    return config


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def inspect_live(config, seconds, require_data=True):
    """直接发现 DDS 并接收样本，避免仅凭 Topic 名称判断设备正常。"""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node("homework_bag_preflight", enable_rosout=False)
    results = {entry["name"]: {**entry, "samples": 0, "frames": [], "publishers": []}
               for entry in config["topics"]}
    subscriptions = {}
    start = time.monotonic()

    def sample(message, name):
        row = results[name]
        row["samples"] += 1
        frames = []
        if hasattr(message, "header"):
            frames.append(message.header.frame_id)
            stamp = message.header.stamp
            row["last_stamp_ns"] = stamp.sec * 1_000_000_000 + stamp.nanosec
        if hasattr(message, "child_frame_id"):
            frames.append(message.child_frame_id)
        if hasattr(message, "transforms"):
            frames.extend(f"{t.header.frame_id} -> {t.child_frame_id}" for t in message.transforms)
        row["frames"] = sorted(set(row["frames"] + frames))

    try:
        while time.monotonic() - start < seconds:
            for name, row in results.items():
                publishers = node.get_publishers_info_by_topic(name)
                row["publishers"] = [{"node": p.node_namespace.rstrip("/") + "/" + p.node_name,
                                      "type": p.topic_type,
                                      "reliability": p.qos_profile.reliability.name,
                                      "durability": p.qos_profile.durability.name}
                                     for p in publishers]
                if publishers and name not in subscriptions and "error" not in row:
                    types = {p.topic_type for p in publishers}
                    if types != {row["type"]}:
                        row["error"] = f"类型不匹配，实际为 {sorted(types)}"
                        continue
                    try:
                        msg_type = get_message(row["type"])
                        # best_effort 可连接 reliable 和 best_effort 发布者。
                        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
                        if name in static_topics(config):
                            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
                        subscriptions[name] = node.create_subscription(
                            msg_type, name, lambda msg, topic=name: sample(msg, topic), qos)
                    except (ImportError, AttributeError, ValueError) as error:
                        row["error"] = f"消息接口不可用，请先编译并 source 工作区：{error}"
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    okay = True
    for name, row in results.items():
        active = row["samples"] > 0 and "error" not in row
        status = "收到数据" if active else row.get("error", "未收到数据")
        requirement = "必需" if row["required"] else "可选"
        print(f"[{requirement}] {name}: {status}；样本 {row['samples']}；Frame {row['frames']}", flush=True)
        if row["required"] and not active:
            okay = False
    if require_data and not okay:
        print("预检查未通过：先启动 Driver / FAST-LIO，确认设备网络、ROS_DOMAIN_ID、Topic 名称和消息接口。")
    return okay, results


def static_topics(config):
    # 也支持验收时将 /tf_static 重命名，只需同步更改 raw_replay_topics。
    return [entry["name"] for entry in config["topics"]
            if entry["type"] == "tf2_msgs/msg/TFMessage"
            and entry["name"] in config["raw_replay_topics"]]


def write_qos(config, path):
    lines = []
    for name in static_topics(config):
        lines.extend([f"{json.dumps(name)}:", "  history: keep_last", "  depth: 100",
                      "  reliability: reliable", "  durability: transient_local"])
    path.write_text("\n".join(lines) + "\n" if lines else "{}\n", encoding="utf-8")


def managed_command(command, log_path, duration=0):
    """持续显示并保存输出；Ctrl+C 或定时结束时向 rosbag 发送 SIGINT。"""
    print("执行：", " ".join(command), flush=True)
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        started = time.monotonic()
        with log_path.open(encoding="utf-8", errors="replace") as reader:
            try:
                while process.poll() is None:
                    print(reader.read(), end="", flush=True)
                    if duration and time.monotonic() - started >= duration:
                        print("\n已达到录制时长，正在关闭 bag 并写入 metadata…", flush=True)
                        break
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n正在关闭 rosbag，请等待 metadata 写入完成…", flush=True)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    # 不强杀录制进程，确保数据库和 metadata 正常收尾。
                    while process.poll() is None:
                        try:
                            process.wait(timeout=0.5)
                        except subprocess.TimeoutExpired:
                            print(reader.read(), end="", flush=True)
                        except KeyboardInterrupt:
                            print("请等待 bag 正常关闭。", flush=True)
                print(reader.read(), end="", flush=True)
    return process.returncode


def validate_bag(path, config):
    import rosbag2_py
    if not (path / "metadata.yaml").is_file():
        raise ValueError(f"没有 metadata.yaml：{path}。录制可能尚未结束或未正常关闭。")
    metadata = rosbag2_py.Info().read_metadata(str(path), "sqlite3")
    counts = {item.topic_metadata.name: {"type": item.topic_metadata.type,
                                        "messages": item.message_count}
              for item in metadata.topics_with_message_count}
    failures = []
    for entry in config["topics"]:
        recorded = counts.get(entry["name"], {})
        if entry["required"] and not recorded.get("messages", 0):
            failures.append(f"{entry['name']} 没有消息")
        if recorded and recorded["type"] != entry["type"]:
            failures.append(f"{entry['name']} 类型不匹配：{recorded['type']}")
    # Humble returns rclpy.duration.Duration; some newer bindings use timedelta.
    duration = metadata.duration
    duration_seconds = (duration.nanoseconds / 1e9 if hasattr(duration, "nanoseconds")
                        else duration.total_seconds())
    report = {"bag": str(path), "duration_seconds": duration_seconds,
              "message_count": metadata.message_count, "topics": counts,
              "passed": not failures, "failures": failures}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def record(args, config):
    name = args.name or dt.datetime.now().strftime("mid360_%Y%m%d_%H%M%S_%f")
    if not name or name in (".", "..") or Path(name).name != name:
        raise ValueError("--name 只能是目录名称，不能包含路径")
    bag = args.bag_root.resolve() / name
    evidence = args.evidence_root.resolve() / name
    if bag.exists() or evidence.exists():
        raise ValueError("同名 bag 或证据目录已经存在，请换一个 --name")
    print(f"检查 {args.check_seconds:g} 秒真实消息接收情况…", flush=True)
    okay, observations = inspect_live(config, args.check_seconds)
    if not okay:
        return 2
    args.bag_root.mkdir(parents=True, exist_ok=True)
    free_gib = shutil.disk_usage(args.bag_root).free / (1024 ** 3)
    if free_gib < args.min_free_gib:
        raise ValueError(f"可用空间 {free_gib:.2f} GiB，小于要求的 {args.min_free_gib:g} GiB")
    evidence.mkdir(parents=True)
    save_json(evidence / "topics_config.json", config)
    save_json(evidence / "preflight.json", observations)
    qos = evidence / "qos.yaml"
    write_qos(config, qos)
    command = ["ros2", "bag", "record", "-s", "sqlite3", "-o", str(bag),
               "--qos-profile-overrides-path", str(qos),
               "--max-bag-size", str(args.split_mib * 1024 * 1024)]
    command += [entry["name"] for entry in config["topics"]]
    save_json(evidence / "session.json", {
        "started_at": dt.datetime.now().astimezone().isoformat(), "command": command,
        "duration_limit_seconds": args.duration, "free_gib_at_start": free_gib,
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
        "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY", "0")})
    print(f"bag：{bag}\n运行日志和检查结果：{evidence}\n按 Ctrl+C 提前结束。", flush=True)
    code = managed_command(command, evidence / "record.log", args.duration)
    info = subprocess.run(["ros2", "bag", "info", str(bag)], text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (evidence / "bag_info.txt").write_text(info.stdout, encoding="utf-8")
    print(info.stdout)
    report = validate_bag(bag, config)
    report["recorder_exit_code"] = code
    save_json(evidence / "validation.json", report)
    print("录制检查通过。" if report["passed"] else "录制不完整，请查看 validation.json。")
    return 0 if report["passed"] and code in (0, -signal.SIGINT) else 2


def play(args, config):
    from rosidl_runtime_py.utilities import get_message
    import rclpy
    bag = args.bag.resolve()
    report = validate_bag(bag, config)
    if not report["passed"]:
        raise ValueError("bag 的关键数据不完整，不能作为本次作业的完整回放")
    selected = list(report["topics"])
    if args.mode == "raw":
        selected = [name for name in config["raw_replay_topics"]
                    if report["topics"].get(name, {}).get("messages", 0)]
        if not selected:
            raise ValueError("bag 不包含配置中的原始传感器 Topic")
    for name in selected:
        try:
            get_message(report["topics"][name]["type"])
        except (ImportError, AttributeError, ValueError) as error:
            raise ValueError(f"{name} 的消息接口未安装，请编译并 source 工作区：{error}") from error
    rclpy.init()
    node = rclpy.create_node("homework_bag_replay_check", enable_rosout=False)
    try:
        end = time.monotonic() + args.check_seconds
        conflicts = set()
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            conflicts.update(name for name in selected + ["/clock"]
                             if node.get_publishers_info_by_topic(name))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if conflicts:
        raise ValueError(f"回放 Topic 已有发布者：{sorted(conflicts)}。先关闭实时 Driver；完整回放还需关闭 FAST-LIO；不要同时运行多个回放。")
    evidence = args.evidence_root.resolve() / ("play_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    evidence.mkdir(parents=True)
    qos = evidence / "qos.yaml"
    write_qos(config, qos)
    command = ["ros2", "bag", "play", str(bag), "--clock", "100", "--rate", str(args.rate),
               "--delay", "3", "--qos-profile-overrides-path", str(qos)]
    if args.loop:
        command.append("--loop")
    command += ["--topics", *selected]
    save_json(evidence / "session.json", {"command": command, "mode": args.mode,
                                         "started_at": dt.datetime.now().astimezone().isoformat()})
    print("请将 RViz 和分析节点设置 use_sim_time:=true。", flush=True)
    if args.mode == "raw":
        print("仅回放传感器与静态 TF：另开终端运行 FAST-LIO，设置 use_sim_time:=true。", flush=True)
    code = managed_command(command, evidence / "play.log")
    save_json(evidence / "result.json", {"player_exit_code": code, "mode": args.mode})
    return 0 if code in (0, -signal.SIGINT) else 2


def parser():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--config", type=Path, default=HERE / "topics.json", help="Topic 配置 JSON")
    cli.add_argument("--bag-root", type=Path, default=WORKSPACE / "bags" / "107")
    cli.add_argument("--evidence-root", type=Path, default=WORKSPACE / "bag_evidence" / "107")
    cli.add_argument("--check-seconds", type=positive, default=5, help="预检查采样秒数，默认 5")
    sub = cli.add_subparsers(dest="action", required=True)
    sub.add_parser("check", help="检查 Topic 类型、发布者、实际数据和 Frame")
    rec = sub.add_parser("record", help="录制并自动保存检查证据")
    rec.add_argument("--duration", type=nonnegative, default=60, help="录制秒数，默认 60；0 表示直到 Ctrl+C")
    rec.add_argument("--name", help="bag 名称，默认自动生成时间戳")
    rec.add_argument("--split-mib", type=int, default=1024, help="单个 db3 分卷大小 MiB，默认 1024；0 不分卷")
    rec.add_argument("--min-free-gib", type=nonnegative, default=2, help="录制前最低可用空间 GiB，默认 2")
    info = sub.add_parser("info", help="检查录制时长、消息数量和关键 Topic")
    info.add_argument("bag", type=Path)
    replay = sub.add_parser("play", help="离线回放并发布 /clock")
    replay.add_argument("bag", type=Path)
    replay.add_argument("--mode", choices=["recorded", "raw"], default="recorded")
    replay.add_argument("--rate", type=positive, default=1)
    replay.add_argument("--loop", action="store_true")
    return cli


def main():
    cli = parser()
    args = cli.parse_args()
    if not shutil.which("ros2"):
        cli.error("请在 ROS 2 容器中运行，并先 source /opt/ros/humble/setup.bash 和工作区 install/setup.bash")
    if args.action == "record" and args.split_mib < 0:
        cli.error("--split-mib 不能小于 0")
    try:
        config = load_config(args.config)
        if args.action == "check":
            return 0 if inspect_live(config, args.check_seconds)[0] else 2
        if args.action == "record":
            return record(args, config)
        if args.action == "info":
            return 0 if validate_bag(args.bag.resolve(), config)["passed"] else 2
        return play(args, config)
    except (ValueError, KeyError, OSError, ImportError, RuntimeError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("已取消。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
