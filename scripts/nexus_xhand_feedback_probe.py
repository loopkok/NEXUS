#!/usr/bin/env python3
"""Observe XHand feedback only. Never enable, home or publish motor commands.

--start-readers opens the two profile serial ports in an isolated ROS domain;
it refuses ports already in use. Candidate stress only reaches disabled bridges.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from xhand_control_interfaces.msg import XHandCommand, XHandStateArray
from nexus_core.profile import Profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--output", required=True)
    parser.add_argument("--start-readers", action="store_true")
    parser.add_argument("--stress-candidates", action="store_true")
    args = parser.parse_args()
    profile = Profile.load(args.profile)
    hands = [c for c in profile.components if c.driver == "xhand_serial"]
    if not hands:
        raise SystemExit("profile has no xhand_serial components")
    if args.start_readers and int(os.environ.get("ROS_DOMAIN_ID", "0")) < 170:
        raise SystemExit("readers require an isolated test ROS_DOMAIN_ID >= 170")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    processes, handles = [], []
    rclpy.init()
    node = Node("xhand_feedback_probe")
    stats = {}
    command_count = {c.name: 0 for c in hands}
    subscriptions, publishers = [], []

    def receive(key, msg):
        now = time.monotonic()
        stat = stats.setdefault(key, {"count": 0, "max_gap_s": 0.0, "over_500ms": 0,
                                      "last": None, "first": None, "min": None, "max": None})
        if stat["last"] is not None:
            gap = now - stat["last"]
            stat["max_gap_s"] = max(stat["max_gap_s"], gap)
            stat["over_500ms"] += int(gap >= 0.5)
        stat["first"] = stat["first"] or now
        stat["last"] = now
        stat["count"] += 1
        values = list(msg.hand_states[0].position) if hasattr(msg, "hand_states") and msg.hand_states else list(getattr(msg, "position", []))
        if values:
            stat["min"] = values if stat["min"] is None else [min(a, b) for a, b in zip(stat["min"], values)]
            stat["max"] = values if stat["max"] is None else [max(a, b) for a, b in zip(stat["max"], values)]

    try:
        if args.start_readers:
            for c in hands:
                port = profile.adapter_config("xhand_serial")["serials"][c.side]
                busy = subprocess.run(["fuser", port], capture_output=True)
                if busy.returncode != 1:
                    raise RuntimeError(f"serial port busy or cannot check ownership: {port}")
            for c in hands:
                port = profile.adapter_config("xhand_serial")["serials"][c.side]
                commands = {
                    "native": ["ros2", "run", "xhand_control_ros2", "xhand_control_ros2_node", "--ros-args",
                               "-r", f"__ns:=/{c.side}_hand", "-p", f"port_name:={port}", "-p", "update_rate:=100.0"],
                    "bridge": ["ros2", "run", "nexus_core", "nexus_joint_bridge", "--ros-args",
                               "-r", f"__node:=probe_bridge_{c.name}", "-p", f"profile_file:={args.profile}",
                               "-p", f"component:={c.name}", "-p", f"side:={c.side}", "-p", "adapter_mode:=xhand"],
                }
                for kind, command in commands.items():
                    handle = (output / f"{c.name}_{kind}.log").open("w")
                    handles.append(handle)
                    processes.append(subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True))
        for c in hands:
            for key, topic, msg_type in (
                (f"native/{c.name}", f"/{c.side}_hand/xhand_state", XHandStateArray),
                (f"canonical/{c.name}", profile.topic(c.name, "joint_states"), JointState),
            ):
                subscriptions.append(node.create_subscription(msg_type, topic, lambda msg, k=key: receive(k, msg), qos_profile_sensor_data))
            subscriptions.append(node.create_subscription(XHandCommand, f"/{c.side}_hand/xhand_command",
                lambda msg, name=c.name: command_count.__setitem__(name, command_count[name] + 1),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
            if args.stress_candidates:
                if not args.start_readers:
                    raise RuntimeError("candidate stress requires private disabled reader bridges")
                pub = node.create_publisher(XHandCommand, f"{profile.namespace}/legacy/{c.side}_xhand_candidate", qos_profile_sensor_data)
                candidate = XHandCommand()
                candidate.name = [n.removeprefix(f"{c.side}_hand_") for n in c.joints]
                candidate.position = [0.0] * c.dim
                publishers.append((pub, candidate))
        start, next_candidate = time.monotonic(), time.monotonic()
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.003)
            now = time.monotonic()
            if now >= next_candidate:
                for pub, candidate in publishers:
                    pub.publish(candidate)
                next_candidate = now + 1.0 / 72.0
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("reader exited early; inspect logs")
            if any(command_count.values()):
                raise RuntimeError("unexpected motor command observed; aborting readers")
        end = time.monotonic()
        for stat in stats.values():
            stat["hz"] = (stat["count"] - 1) / max(1e-6, stat["last"] - stat["first"])
            stat["last_age_s"] = end - stat["last"]
            del stat["first"], stat["last"]
        report = {"domain": os.environ.get("ROS_DOMAIN_ID"), "duration_s": end - start,
                  "motor_commands_observed": command_count, "streams": stats}
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
