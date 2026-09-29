#!/usr/bin/env python3
"""Drive and measure a headless NEXUS simulation without hardware input.

Run after launching ``nero_dual_xhand_mujoco`` with ``with_inputs:=false``.
The node publishes synthetic Quest 3 messages and camera JPEGs to the profile's
raw source topics, then exercises recording, command arbitration, and (optionally)
the stub policy/HITL path. It never opens a physical device.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import defaultdict

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CompressedImage, Joy, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from xhand_control_interfaces.msg import XHandCommand

from nexus_core.profile import Profile
from nero_quest_teleop.ik_solver import IKSolver
from quest3_hand_mocap.pose_mapping import rotate_pose

_CANDIDATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def open_hand(curl: float) -> list[tuple[float, float, float]]:
    """21 stable MANO-style landmarks, smoothly curling the four fingers."""
    pts = np.array([
        [0, 0, 0], [.008, -.002, -.004], [.016, -.004, -.006], [.024, -.005, -.005], [.032, -.006, -.003],
        [.004, .002, -.018], [.003, .002, -.036], [.002, .002, -.048], [.001, .001, -.056],
        [0, .001, -.020], [-.001, 0, -.040], [-.002, 0, -.052], [-.003, 0, -.060],
        [-.004, -.001, -.017], [-.006, -.001, -.035], [-.007, -.001, -.046], [-.008, -.001, -.054],
        [-.007, -.002, -.014], [-.010, -.003, -.028], [-.012, -.003, -.036], [-.014, -.003, -.042],
    ], dtype=float)
    for start in (5, 9, 13, 17):
        root = pts[start].copy()
        for idx in range(start + 1, start + 4):
            orig = pts[idx].copy()
            k = curl * (idx - start) / 3.0 * 0.65
            pts[idx] = [orig[0], orig[1] + k * .018,
                        root[2] + (orig[2] - root[2]) * (1 - k)]
    return [tuple(map(float, p)) for p in pts]


class AcceptanceNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("nexus_sim_acceptance")
        self.args = args
        self.profile = Profile.load(args.profile)
        self.ns = self.profile.namespace
        self._callback_group = ReentrantCallbackGroup()
        self.started = time.monotonic()
        self.phase = "warmup"
        self.phase_started = self.started
        self.counts = defaultdict(int)
        self.last_input = {"left": None, "right": None}
        self.input_samples = {"left": [], "right": []}
        self.latencies = {"left": [], "right": []}
        self.states = {}
        self.state_history = defaultdict(list)
        self.commands = defaultdict(list)
        self.candidates = defaultdict(int)
        self.candidate_values = defaultdict(list)
        self.candidate_times = defaultdict(list)
        self.control_history = []
        self.link7_samples = {"left": [], "right": []}
        self.orientation_samples = {"left": [], "right": []}
        self.solvers = {side: IKSolver() for side in ("left", "right")}
        self.wrist_mapping = self.profile.raw.get("input_settings", {}).get(
            "quest3_wrist_pose_mapping", {}
        )
        self.wrist_rotations = {
            side: np.asarray(
                self.wrist_mapping.get(f"{side}_rotation", np.eye(3).reshape(-1)),
                dtype=float,
            ).reshape(3, 3)
            for side in ("left", "right")
        }
        self.latest_policy = {}
        self.latest_control = {}
        self.latest_collect = {}
        self.latest_dropout = {}

        self.wrist_pubs = {
            side: self.create_publisher(PoseStamped, f"/quest3/{side}_wrist_pose", qos_profile_sensor_data)
            for side in ("left", "right")
        }
        self.hand_pubs = {
            side: self.create_publisher(PoseArray, f"/hand_landmarks/{side}", qos_profile_sensor_data)
            for side in ("left", "right")
        }
        self.joy_pubs = {
            side: self.create_publisher(Joy, f"/quest3/{side}_controller_joy", qos_profile_sensor_data)
            for side in ("left", "right")
        }
        self.body_pub = self.create_publisher(PoseArray, "/quest3/body_joints", qos_profile_sensor_data)
        self.body_names_pub = self.create_publisher(String, "/quest3/body_joint_names", 10)
        self.camera_pubs = {
            role: self.create_publisher(CompressedImage, camera["capture_topic"], qos_profile_sensor_data)
            for role, camera in ((c["role"], c) for c in self.profile.raw["cameras"])
            if camera.get("capture_topic")
        }
        self.start_pub = self.create_publisher(Bool, f"{self.ns}/control/teleop_start", 10)
        self.collect_pub = self.create_publisher(String, f"{self.ns}/data/collect/control", 10)
        task_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.collect_task_pub = self.create_publisher(String, f"{self.ns}/data/collect/task", task_qos)
        self.policy_cmd_pub = self.create_publisher(String, f"{self.ns}/policy/cmd", 10)
        self.control_cmd_pub = self.create_publisher(String, f"{self.ns}/control/cmd", 10)

        for side in ("left", "right"):
            channel = side
            self.create_subscription(PoseStamped, f"{self.ns}/input/{channel}/wrist_pose",
                lambda msg, s=side: self._input(s, msg), qos_profile_sensor_data,
                callback_group=self._callback_group)
            self.create_subscription(PoseArray, f"{self.ns}/input/{channel}/hand_landmarks",
                lambda msg, s=side: self._count(f"bridge_hand:{s}"), qos_profile_sensor_data,
                callback_group=self._callback_group)
            self.create_subscription(Joy, f"{self.ns}/input/{channel}/controller_joy",
                lambda msg, s=side: self._count(f"bridge_joy:{s}"), qos_profile_sensor_data,
                callback_group=self._callback_group)
        for role in self.camera_pubs:
            self.create_subscription(CompressedImage, f"{self.ns}/camera/{role}/image/compressed",
                lambda msg, r=role: self._count(f"camera:{r}"), qos_profile_sensor_data,
                callback_group=self._callback_group)
        for spec in self.profile.components:
            self.create_subscription(JointState, self.profile.topic(spec.name, "joint_states"),
                lambda msg, n=spec.name: self._state(n, msg), qos_profile_sensor_data,
                callback_group=self._callback_group)
            self.create_subscription(JointState, self.profile.topic(spec.name, "joint_commands"),
                lambda msg, n=spec.name: self._command(n, msg), qos_profile_sensor_data,
                callback_group=self._callback_group)
            for src in ("teleop", "policy", "playback"):
                self.create_subscription(JointState, self.profile.candidate_topic(src, spec.name),
                    lambda msg, s=src, n=spec.name: self._candidate(s, n, msg), _CANDIDATE_QOS,
                    callback_group=self._callback_group)
        for side in ("left", "right"):
            self.create_subscription(XHandCommand,
                f"{self.ns}/legacy/{side}_xhand_candidate",
                lambda _msg, s=side: self._count(f"native_xhand_candidate:{s}"),
                qos_profile_sensor_data, callback_group=self._callback_group)
        self.create_subscription(String, f"{self.ns}/control/state", self._control_state, 10,
                                 callback_group=self._callback_group)
        self.create_subscription(String, f"{self.ns}/policy/state", self._policy_state, 10,
                                 callback_group=self._callback_group)
        self.create_subscription(String, f"{self.ns}/data/collect/state", self._collect_state, 10,
                                 callback_group=self._callback_group)
        self.driver_services = {
            op: self.create_client(Trigger, f"{self.ns}/drivers/{op}",
                                   callback_group=self._callback_group)
            for op in ("ready", "enable", "home", "estop")
        }
        self.timer = self.create_timer(.02, self._publish_inputs,
                                       callback_group=self._callback_group)
        self.image_timer = self.create_timer(1.0 / args.image_rate, self._publish_images,
                                             callback_group=self._callback_group)
        self._last_image = 0
        self._drop_wrists = False
        self._motion_epoch = None
        self._baseline_tcp = {}
        self._baseline_rot = {}
        self.get_logger().info(f"simulation acceptance profile={self.profile.profile_id} sha256={self.profile.digest}")

    def _count(self, key: str) -> None:
        self.counts[key] += 1

    def _input(self, side, msg):
        t = time.monotonic()
        self.last_input[side] = t
        self.input_samples[side].append((t, float(msg.pose.position.x)))
        self._count(f"bridge_wrist:{side}")

    def _state(self, name, msg):
        spec = self.profile.component(name)
        if list(msg.name) != list(spec.joints) or len(msg.position) != spec.dim:
            self._count(f"invalid_state:{name}")
            return
        q = np.asarray(msg.position, dtype=float)
        if not np.isfinite(q).all():
            self._count(f"invalid_state:{name}")
            return
        self.states[name] = q
        self.state_history[name].append((time.monotonic(), q.copy()))
        self._count(f"state:{name}")
        if spec.kind == "arm":
            side = spec.side
            T = self.solvers[side].fk(q)
            t = time.monotonic()
            if self.phase == "stationary":
                self.orientation_samples[side].append((t, T[:3, :3].copy()))
            elif self.phase == "motion":
                self.link7_samples[side].append((t, T[:3, 3].copy()))
                self.orientation_samples[side].append((t, T[:3, :3].copy()))
                self._count(f"motion_state:{side}")

    def _command(self, name, msg):
        spec = self.profile.component(name)
        if list(msg.name) != list(spec.joints) or len(msg.position) != spec.dim:
            self._count(f"invalid_command:{name}")
            return
        self.commands[name].append((time.monotonic(), np.asarray(msg.position, dtype=float)))
        self._count(f"command:{name}")

    def _candidate(self, source, name, msg):
        now = time.monotonic()
        self.candidates[f"{source}:{name}"] += 1
        self.candidate_values[f"{source}:{name}"].append(np.asarray(msg.position, dtype=float))
        self.candidate_times[f"{source}:{name}"].append(now)
        if source == "teleop" and self.profile.component(name).kind == "arm":
            side = self.profile.component(name).side
            if self.last_input[side] is not None:
                self.latencies[side].append((now - self.last_input[side]) * 1000.0)

    def _control_state(self, msg):
        try:
            self.latest_control = json.loads(msg.data)
            current = (self.latest_control.get("mode"), self.latest_control.get("fault", ""))
            if not self.control_history or self.control_history[-1][1:] != current:
                self.control_history.append((time.monotonic(), *current))
        except Exception:
            pass

    def _policy_state(self, msg):
        try:
            self.latest_policy = json.loads(msg.data)
        except Exception:
            pass

    def _collect_state(self, msg):
        try:
            self.latest_collect = json.loads(msg.data)
        except Exception:
            pass

    def _publish_inputs(self):
        now = time.monotonic()
        motion_t = 0.0 if self._motion_epoch is None else max(0.0, now - self._motion_epoch)
        curl = self.args.hand_curl
        stamp = self.get_clock().now().to_msg()
        for side in ("left", "right"):
            if not self._drop_wrists:
                wrist = PoseStamped()
                wrist.header.stamp = stamp
                wrist.header.frame_id = self.wrist_mapping.get(
                    f"{side}_frame_id", f"quest3_{side}_wrist")
                # Simulate the Quest-side, side-specific conversion. NEXUS
                # consumes this mapped link7 flange pose with an identity map.
                raw_position = np.array([
                    0.0,
                    (self.args.amplitude * math.sin(2 * math.pi * self.args.frequency * motion_t)
                     if self.phase == "motion" else 0.0),
                    0.0,
                ])
                mapped_position, mapped_quat = rotate_pose(
                    raw_position, np.array([0.0, 0.0, 0.0, 1.0]),
                    self.wrist_rotations[side])
                wrist.pose.position.x = float(mapped_position[0])
                wrist.pose.position.y = float(mapped_position[1])
                wrist.pose.position.z = float(mapped_position[2])
                wrist.pose.orientation.x = float(mapped_quat[0])
                wrist.pose.orientation.y = float(mapped_quat[1])
                wrist.pose.orientation.z = float(mapped_quat[2])
                wrist.pose.orientation.w = float(mapped_quat[3])
                self.wrist_pubs[side].publish(wrist)
            hand = PoseArray()
            hand.header.stamp = stamp
            hand.header.frame_id = f"hand_{side}"
            hand.poses = [Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(w=1.0))
                          for x, y, z in open_hand(curl)]
            self.hand_pubs[side].publish(hand)
            joy = Joy()
            joy.header.stamp = stamp
            joy.axes = [0.0, 0.0, 0.0, 0.0]
            joy.buttons = [0, 0, 0, 0]
            self.joy_pubs[side].publish(joy)
        body = PoseArray()
        body.header.stamp = stamp
        body.header.frame_id = "quest3_body"
        body.poses = [Pose(orientation=Quaternion(w=1.0))]
        self.body_pub.publish(body)
        self.body_names_pub.publish(String(data="[\"root\"]"))

    def _publish_images(self):
        if not self.camera_pubs:
            return
        self._last_image += 1
        for index, (role, pub) in enumerate(self.camera_pubs.items()):
            frame = np.zeros((120, 160, 3), dtype=np.uint8)
            frame[:, :, :] = (25 + index * 45, 40, 70 + (self._last_image % 120))
            x = (self._last_image * 3 + index * 29) % 140
            cv2.rectangle(frame, (x, 45), (x + 19, 74), (240, 230, 40), -1)
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"sim_camera_{role}"
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            pub.publish(msg)

    def _spin_for(self, seconds: float):
        time.sleep(max(0.0, seconds))

    def _service(self, operation: str, timeout: float | None = None):
        if timeout is None:
            timeout = 40.0 if operation == "home" else 8.0
        client = self.driver_services[operation]
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"driver manager {operation} service unavailable")
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f"driver manager {operation} timed out")
        result = future.result()
        if not result.success:
            raise RuntimeError(f"driver manager {operation} failed: {result.message}")
        return result.message

    def _wait_state(self, key: str, value: str, timeout: float = 8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(0.01)
            observed = self.latest_policy.get("state") if key == "policy" else self.latest_control.get("mode")
            if str(observed).upper() == value.upper():
                return
        raise RuntimeError(f"timeout waiting for {key} state={value}; last={self.latest_policy if key == 'policy' else self.latest_control}")

    def _teleop_start(self):
        self.start_pub.publish(Bool(data=True))
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            time.sleep(0.01)
            if self.latest_control.get("mode") == "TELEOP":
                return
        raise RuntimeError(f"teleop did not arm: {self.latest_control}")

    def _validate_publishers(self):
        result = {}
        for spec in self.profile.components:
            topic = self.profile.topic(spec.name, "joint_commands")
            count = self.count_publishers(topic)
            result[spec.name] = count
            if count != 1:
                raise RuntimeError(f"{topic} has {count} final-command publishers, expected exactly one")
        return result

    def _motion_metrics(self):
        result = {}
        for side in ("left", "right"):
            samples = self.link7_samples[side]
            rotations = self.orientation_samples[side]
            if len(samples) < 20:
                raise RuntimeError(f"insufficient {side} motion state samples: {len(samples)}")
            ts = np.asarray([row[0] for row in samples])
            x = np.asarray([row[1][0] for row in samples])
            raw = self.args.amplitude * np.sin(2 * np.pi * self.args.frequency * (ts - self._motion_epoch))
            expected = self.profile.teleop_config(f"{side}_arm")["motion_scale"] * raw
            corr = float(np.corrcoef(expected, x - x[0])[0, 1]) if np.std(expected) and np.std(x) else 0.0
            displacement = float(np.ptp(x))
            orient = [R for _, R in rotations]
            if orient:
                anchor = orient[0]
                angles = [math.degrees(math.acos(float(np.clip((np.trace(anchor.T @ R) - 1) / 2, -1, 1))))
                          for R in orient]
                max_rotation_deg = max(angles)
            else:
                max_rotation_deg = 0.0
            spec = self.profile.component(f"{side}_arm")
            state_rows = [(t, q) for t, q in self.state_history[spec.name]
                          if self._motion_epoch <= t <= time.monotonic()]
            command_rows = self.commands[spec.name]
            tracking_error = None
            if state_rows and command_rows:
                command_times = np.asarray([t for t, _ in command_rows])
                command_values = np.asarray([q for _, q in command_rows])
                error_rows = []
                for state_time, state in state_rows:
                    index = min(len(command_times) - 1,
                                max(0, int(np.searchsorted(command_times, state_time))))
                    if index and abs(command_times[index - 1] - state_time) <= abs(command_times[index] - state_time):
                        index -= 1
                    error_rows.append(np.abs(state - command_values[index]))
                errors = np.asarray(error_rows)
                tracking_error = {
                    "p95_abs_rad": float(np.percentile(errors, 95)),
                    "max_abs_rad": float(np.max(errors)),
                    "max_by_joint_rad": {
                        joint: float(value) for joint, value in
                        zip(spec.joints, np.max(errors, axis=0))
                    },
                    "matched_state_samples": len(state_rows),
                }
            candidate_key = f"teleop:{spec.name}"
            candidate_rows = [(t, q) for t, q in zip(
                self.candidate_times[candidate_key], self.candidate_values[candidate_key])
                if self._motion_epoch <= t <= time.monotonic()]
            candidate_motion = None
            if candidate_rows:
                candidate_times = np.asarray([t for t, _ in candidate_rows])
                candidate_q = np.asarray([q for _, q in candidate_rows])
                candidate_fk = [self.solvers[side].fk(q) for q in candidate_q]
                candidate_anchor = candidate_fk[0][:3, :3]
                candidate_angles = [math.degrees(math.acos(float(np.clip(
                    (np.trace(candidate_anchor.T @ T[:3, :3]) - 1) / 2, -1, 1))))
                    for T in candidate_fk]
                candidate_motion = {
                    "samples": len(candidate_rows),
                    "link7_orientation_max_drift_deg": max(candidate_angles),
                    "link7_position_peak_to_peak_xyz_m": np.ptp(
                        np.asarray([T[:3, 3] for T in candidate_fk]), axis=0).tolist(),
                    "joint_peak_to_peak_rad": {
                        joint: float(value) for joint, value in
                        zip(spec.joints, np.ptp(candidate_q, axis=0))
                    },
                    "joint_max_step_rad": {
                        joint: float(value) for joint, value in zip(
                            spec.joints, np.max(np.abs(np.diff(candidate_q, axis=0)), axis=0))
                    } if len(candidate_q) > 1 else {},
                    "max_gap_ms": float(np.max(np.diff(candidate_times)) * 1000)
                    if len(candidate_times) > 1 else None,
                }
            result[side] = {
                "link7_x_peak_to_peak_m": displacement,
                "link7_expected_peak_to_peak_m": 2 * self.args.amplitude * self.profile.teleop_config(f"{side}_arm")["motion_scale"],
                "link7_x_expected_correlation": corr,
                "link7_orientation_max_drift_deg": max_rotation_deg,
                "arm_joint_target_tracking_error": tracking_error,
                "ik_candidate_motion": candidate_motion,
                "motion_state_samples": len(samples),
                "quest_mapped_link7_input_x_peak_to_peak_m": (float(np.ptp([v for _, v in self.input_samples[side]]))
                                                   if self.input_samples[side] else 0.0),
                "teleop_candidate_joint_peak_to_peak_rad": (
                    float(np.max(np.ptp(np.asarray(self.candidate_values[f"teleop:{side}_arm"]), axis=0)))
                    if self.candidate_values[f"teleop:{side}_arm"] else 0.0),
                "teleop_candidate_max_gap_ms": (
                    float(np.max(np.diff(self.candidate_times[f"teleop:{side}_arm"])) * 1000)
                    if len(self.candidate_times[f"teleop:{side}_arm"]) > 1 else None),
                "end_effector_candidate_max_gap_ms": (
                    float(np.max(np.diff(self.candidate_times[f"teleop:{side}_ee"])) * 1000)
                    if len(self.candidate_times[f"teleop:{side}_ee"]) > 1 else None),
                "link7_position_peak_to_peak_xyz_m": np.ptp(
                    np.asarray([row[1] for row in samples]), axis=0).tolist(),
                "input_to_candidate_latency_ms": _percentiles(self.latencies[side]),
            }
            if self.latest_control.get("mode") != "TELEOP":
                result[side]["arbiter_mode_at_end"] = self.latest_control.get("mode")
                result[side]["control_history"] = self.control_history
                raise RuntimeError(f"{side} arbiter left TELEOP during tracking: {result[side]}")
            if corr < self.args.min_correlation or displacement < self.args.min_tcp_motion:
                raise RuntimeError(f"{side} arm did not track the synthetic hand path: {result[side]}")
            if max_rotation_deg > self.args.max_rotation_deg:
                raise RuntimeError(f"{side} arm rotated during translation-only input: {result[side]}")
            if (tracking_error is None
                    or tracking_error["p95_abs_rad"] > self.args.max_tracking_error_rad
                    or tracking_error["max_abs_rad"] > self.args.max_tracking_error_peak_rad):
                raise RuntimeError(f"{side} arm did not track IK targets within tolerance: {result[side]}")
        return result

    def run_record(self):
        self._spin_for(self.args.warmup)
        # A previous intentional dropout leaves the assembly PAUSED. Reset
        # that simulation test state before exercising enable and re-arming.
        self.control_cmd_pub.publish(String(data="IDLE"))
        self._wait_state("control", "IDLE")
        ready = self._service("ready")
        enabled = self._service("enable")
        homed = self._service("home")
        self._spin_for(self.args.settle)
        self._validate_publishers()
        self._teleop_start()
        self.phase = "stationary"
        self._spin_for(self.args.hold)
        self.orientation_samples = {side: [] for side in ("left", "right")}

        self.collect_task_pub.publish(String(data="NEXUS synthetic Quest3 teleoperation acceptance"))
        self.collect_pub.publish(String(data="start"))
        self._spin_for(1.2)
        self._motion_epoch = time.monotonic()
        self.phase = "motion"
        self._spin_for(self.args.motion)
        tracking = self._motion_metrics()
        # Loss of wrist tracking should pause the mux and preserve measured pose.
        dropout_started = time.monotonic()
        wrist_counts_before = {
            side: self.counts[f"bridge_wrist:{side}"] for side in ("left", "right")
        }
        self._drop_wrists = True
        self._spin_for(self.args.dropout)
        dropout_mode = self.latest_control.get("mode", "UNKNOWN")
        dropout_fault = self.latest_control.get("fault", "")
        pause_events = [t for t, mode, _fault in self.control_history
                        if mode == "PAUSED" and t >= dropout_started]
        pause_latency_ms = ((pause_events[0] - dropout_started) * 1000.0
                            if pause_events else None)
        self.latest_dropout = {
            "mode_after_timeout": dropout_mode,
            "fault": dropout_fault,
            "pause_latency_ms": pause_latency_ms,
            "post_stop_wrist_messages": {
                side: self.counts[f"bridge_wrist:{side}"] - wrist_counts_before[side]
                for side in ("left", "right")
            },
        }
        if dropout_mode != "PAUSED":
            raise RuntimeError(f"wrist-input timeout did not pause arbitration within "
                               f"{self.args.dropout:.2f}s: {self.latest_control}")
        if pause_latency_ms is None or pause_latency_ms > 1500.0:
            raise RuntimeError(f"wrist-input pause latency exceeded 1500 ms: {self.latest_dropout}")
        self._drop_wrists = False
        self.control_cmd_pub.publish(String(data="IDLE"))
        self._spin_for(.15)
        self._teleop_start()
        self._spin_for(1.0)
        self.collect_pub.publish(String(data="stop"))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            time.sleep(0.05)
            if self.latest_collect.get("state") == "IDLE":
                break
        if self.latest_collect.get("state") not in ("IDLE", "SAVING"):
            raise RuntimeError(f"recorder did not stop cleanly: {self.latest_collect}")
        self._spin_for(1.2)
        rates = {}
        max_steps = {}
        tracking_errors = {}
        for spec in self.profile.components:
            samples = self.commands[spec.name]
            times = np.asarray([s[0] for s in samples]) if samples else np.array([])
            values = np.asarray([s[1] for s in samples]) if samples else np.empty((0, spec.dim))
            rates[spec.name] = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
            max_steps[spec.name] = float(np.max(np.abs(np.diff(values, axis=0)))) if len(values) > 1 else 0.0
            tracking_errors[spec.name] = _percentiles([
                float(np.max(np.abs(state - cmd)))
                for state, cmd in [(self.states[spec.name], values[-1])] if len(values)
            ])
        output = {
            "profile_id": self.profile.profile_id,
            "profile_sha256": self.profile.digest,
            "lifecycle": {"ready": ready, "enable": enabled, "home": homed},
            "final_command_publishers": self._validate_publishers(),
            "counts": dict(self.counts),
            "teleop_candidates": dict(self.candidates),
            "command_rate_hz": rates,
            "max_joint_command_step_rad": max_steps,
            "last_command_to_measured_error_rad": tracking_errors,
            "tracking": tracking,
            "wrist_dropout": self.latest_dropout,
            "recorder": self.latest_collect,
            "policy_initial": self.latest_policy,
            "control_history": self.control_history,
        }
        if self.latest_collect.get("state") != "IDLE":
            raise RuntimeError(f"recorder is not idle after stop: {self.latest_collect}")
        for spec in self.profile.components:
            if self.counts[f"state:{spec.name}"] < 10 or self.counts[f"command:{spec.name}"] < 10:
                raise RuntimeError(f"insufficient state/command flow on {spec.name}")
            if self.counts[f"invalid_state:{spec.name}"] or self.counts[f"invalid_command:{spec.name}"]:
                raise RuntimeError(f"invalid joint interface messages found: {dict(self.counts)}")
        for role in self.camera_pubs:
            if self.counts[f"camera:{role}"] < self.args.motion * self.args.image_rate * .5:
                raise RuntimeError(f"camera relay too slow for {role}: {self.counts[f'camera:{role}']}")
        return output

    def run_policy(self, replay_path: str | None):
        self._spin_for(self.args.warmup)
        if replay_path:
            # The process launch must have received replay_path for this node.
            self.policy_cmd_pub.publish(String(data="playback"))
            self._wait_state("policy", "PLAYBACK")
            self.policy_cmd_pub.publish(String(data="pause"))
            self._wait_state("policy", "PLAYBACK_PAUSED")
            self.policy_cmd_pub.publish(String(data="resume"))
            self._wait_state("policy", "PLAYBACK")
            playback_mode = "PLAYBACK"
        else:
            self.policy_cmd_pub.publish(String(data="policy"))
            self._wait_state("policy", "POLICY")
            playback_mode = None
        self._spin_for(1.0)
        self.policy_cmd_pub.publish(String(data="takeover"))
        self._wait_state("policy", "HUMAN")
        self._wait_state("control", "TELEOP")
        self._spin_for(.3)
        self.policy_cmd_pub.publish(String(data="release"))
        self._wait_state("policy", "POLICY" if not replay_path else "PLAYBACK")
        self._spin_for(.5)
        self.policy_cmd_pub.publish(String(data="stop"))
        self._wait_state("policy", "IDLE")
        return {"initial_mode": playback_mode or "POLICY", "takeover_mode": "HUMAN",
                "returned_mode": "POLICY" if not replay_path else "PLAYBACK",
                "final_mode": self.latest_policy.get("state"),
                "counts": dict(self.candidates), "policy_state": self.latest_policy}

    def run_probe(self):
        self._spin_for(self.args.probe_duration)
        rates = {}
        candidate_joint_ranges = {}
        for key, stamps in self.candidate_times.items():
            rates[key] = {
                "count": len(stamps),
                "rate_hz": ((len(stamps) - 1) / (stamps[-1] - stamps[0])
                            if len(stamps) > 1 else 0.0),
                "max_gap_ms": (float(np.max(np.diff(stamps)) * 1000)
                               if len(stamps) > 1 else None),
            }
            values = self.candidate_values[key]
            if values:
                component = key.split(":", 1)[1]
                spec = self.profile.component(component)
                tail = np.asarray(values[-min(200, len(values)):])
                candidate_joint_ranges[key] = {
                    "sample_count": len(tail),
                    "mean_by_joint": {joint: float(value) for joint, value in
                                      zip(spec.joints, np.mean(tail, axis=0))},
                    "min_by_joint": {joint: float(value) for joint, value in
                                     zip(spec.joints, np.min(tail, axis=0))},
                    "max_by_joint": {joint: float(value) for joint, value in
                                     zip(spec.joints, np.max(tail, axis=0))},
                }
        return {"duration_s": self.args.probe_duration,
                "canonical_input_counts": dict(self.counts),
                "candidate_output_rates": rates,
                "candidate_joint_ranges": candidate_joint_ranges,
                "control": self.latest_control}

    def finish(self):
        self._service("estop")


def _percentiles(values):
    if not values:
        return {"count": 0}
    arr = np.asarray(values, dtype=float)
    return {"count": int(len(arr)), "p50": float(np.percentile(arr, 50)),
            "p95": float(np.percentile(arr, 95)), "max": float(np.max(arr))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--phase", choices=("record", "policy", "probe"), default="record")
    parser.add_argument("--replay-path", default="")
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--motion", type=float, default=10.0)
    # The input adapter has a 0.5 s stale-pose timeout and the mux then waits
    # for its 0.35 s candidate watchdog; leave scheduling margin for both.
    parser.add_argument("--dropout", type=float, default=1.5)
    parser.add_argument("--image-rate", type=float, default=30.0)
    parser.add_argument("--amplitude", type=float, default=.008)
    parser.add_argument("--frequency", type=float, default=.15)
    parser.add_argument("--min-correlation", type=float, default=.35)
    parser.add_argument("--min-tcp-motion", type=float, default=.001)
    parser.add_argument("--max-rotation-deg", type=float, default=8.0)
    parser.add_argument("--max-tracking-error-rad", type=float, default=0.15)
    parser.add_argument("--max-tracking-error-peak-rad", type=float, default=0.35)
    parser.add_argument("--hand-curl", type=float, default=0.0,
                        help="synthetic Quest finger curl during arm-tracking tests (0=open, 1=closed)")
    parser.add_argument("--probe-duration", type=float, default=10.0)
    parser.add_argument("--final-estop", action="store_true",
                        help="Latch the simulation-only estop after this test phase.")
    args = parser.parse_args()
    if (args.amplitude < 0 or args.frequency <= 0 or args.image_rate <= 0
            or not 0.0 <= args.hand_curl <= 1.0
            or args.max_tracking_error_rad <= 0 or args.max_tracking_error_peak_rad <= 0):
        parser.error("amplitude/rates/tracking tolerances must be positive; hand-curl must be in [0, 1]")
    rclpy.init()
    node = AcceptanceNode(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    try:
        if args.phase == "record":
            result = node.run_record()
        elif args.phase == "policy":
            result = node.run_policy(args.replay_path or None)
        else:
            result = node.run_probe()
        node._spin_for(1.0)
        if args.final_estop:
            node.finish()
        result["result"] = "PASS"
        print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
    except Exception as exc:
        if node.latest_collect.get("state") == "RECORDING":
            node.collect_pub.publish(String(data="stop"))
            node._spin_for(1.5)
        collect_summary = {k: node.latest_collect.get(k) for k in
                           ("state", "session", "episode_index", "elapsed_s", "samples_per_s",
                            "camera_counts", "folder") if k in node.latest_collect}
        print(json.dumps({"result": "FAIL", "error": str(exc),
                          "control": node.latest_control, "policy": node.latest_policy,
                          "collect": collect_summary, "counts": dict(node.counts),
                          "candidates": dict(node.candidates),
                          "control_history": node.control_history,
                          "wrist_dropout": node.latest_dropout,
                          "candidate_range_rad": {k: (float(np.max(np.ptp(np.asarray(v), axis=0))) if v else 0.0)
                                                   for k, v in node.candidate_values.items()},
                          "candidate_max_gap_ms": {k: (float(np.max(np.diff(v)) * 1000) if len(v) > 1 else None)
                                                   for k, v in node.candidate_times.items()}},
                         ensure_ascii=False, indent=2, default=str))
        raise
    finally:
        executor.shutdown(timeout_sec=3.0)
        executor_thread.join(timeout=3.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
