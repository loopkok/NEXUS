#!/usr/bin/env python3
"""hz_best_effort.py — 对 BEST_EFFORT 话题测实际到达率/间隔（py3.10 + rclpy）。

ros2 topic hz 在 Humble 没有 QoS 选项，默认 RELIABLE 订阅，匹配不上 BEST_EFFORT
发布者（如 /left_arm/joint_commands），永远 0 条。本脚本用 BEST_EFFORT depth=1
订阅，滚动打印：到达率、相邻消息间隔的 min/mean/max/std。

用法（py3.10 + ROS 环境）：
  PYTHONPATH=astral_ws/src/astral_policy_inference:astral_ws/src/astral_data_collect \
    /usr/bin/python3 astral_ws/scripts/hz_best_effort.py \
      /left_arm/joint_commands --msg js
  # --msg: js(JointState) | float(Float64) | str(String)，按话题选
  # --window N：滚动窗口消息数（默认 50）；Ctrl-C 退出
"""
from __future__ import annotations

import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("--msg", choices=["js", "float", "str"], default="js")
    ap.add_argument("--window", type=int, default=50)
    args = ap.parse_args()

    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64, String

    msg_type = {"js": JointState, "float": Float64, "str": String}[args.msg]
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    rclpy.init()
    node = Node("hz_best_effort")
    stamps: list[float] = []
    intervals: list[float] = []

    def cb(_msg) -> None:
        t = time.monotonic()
        if stamps:
            intervals.append(t - stamps[-1])
            if len(intervals) > args.window:
                intervals.pop(0)
        stamps.append(t)
        if len(stamps) > args.window:
            stamps.pop(0)
        if len(stamps) >= 2:
            span = stamps[-1] - stamps[0]
            rate = (len(stamps) - 1) / span if span > 0 else 0.0
            iv = intervals[-min(len(intervals), 50):]
            s = ""
            if iv:
                s = (f" interval[min={min(iv)*1e3:.1f} mean={statistics.mean(iv)*1e3:.1f}"
                     f" max={max(iv)*1e3:.1f} std={statistics.pstdev(iv)*1e3:.1f} ms]")
            print(f"\rrate={rate:.1f} Hz{s}   ", end="", flush=True)

    node.create_subscription(msg_type, args.topic, cb, qos)
    print(f"subscribing {args.topic} (BEST_EFFORT depth=1), --window {args.window} ...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
