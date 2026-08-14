#!/usr/bin/env python3
"""MuJoCo sim — subscribe Astral arm joint_commands, drive dual-arm MJCF.

Drop-in stand-in for ``astral_robot_control`` (do not run both on same topics).

Pipeline:
  quest3 → astral_teleop_{left,right} → /{side}_arm/joint_commands
                                      → this node (viewer + optional joint_states)
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from astral_mujoco_sim.fps_counter import FPSCounter
from astral_mujoco_sim.joint_names import (
    LEFT_ARM_JOINT_NAMES,
    LEFT_MJCF_JOINTS,
    RIGHT_ARM_JOINT_NAMES,
    RIGHT_MJCF_JOINTS,
    normalize_joint_name,
)


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def _pack_arm(
    msg: JointState, expected: List[str]
) -> Optional[np.ndarray]:
    if not msg.position:
        return None
    q = np.asarray(msg.position, dtype=np.float64).reshape(-1)
    if msg.name and len(msg.name) == len(q):
        lut = {normalize_joint_name(n): float(v) for n, v in zip(msg.name, q)}
        matched = sum(1 for n in expected if n in lut)
        if matched >= max(1, len(expected) // 2):
            return np.array([lut.get(n, 0.0) for n in expected], dtype=np.float64)
    if q.size < 7:
        return None
    return q[:7].copy()


class AstralMujocoSimNode(Node):
    def __init__(self) -> None:
        super().__init__("astral_mujoco_sim_node")
        self.declare_parameter("mjcf_path", "")
        self.declare_parameter("enable_viewer", True)
        self.declare_parameter("realtime", True)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("state_rate", 50.0)
        self.declare_parameter("fps_print_interval", 5.0)
        self.declare_parameter(
            "init_pose_left",
            [0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0],
        )
        self.declare_parameter(
            "init_pose_right",
            [0.32, -0.11, 0.53, -0.80, 0.28, 0.0, 0.0],
        )

        mjcf_path = str(self.get_parameter("mjcf_path").value).strip()
        if not mjcf_path:
            share = Path(get_package_share_directory("astral_mujoco_sim"))
            mjcf_path = str(share / "assets" / "mjcf" / "astral_dual.xml")
        if not Path(mjcf_path).is_file():
            raise FileNotFoundError(f"MJCF not found: {mjcf_path}")

        try:
            import mujoco
            import mujoco.viewer
        except ImportError as exc:
            raise RuntimeError("pip install mujoco") from exc

        self._mujoco = mujoco
        self._viewer_mod = mujoco.viewer
        self.get_logger().info(f"Loading MJCF: {mjcf_path}")
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)

        self._qpos_addrs: Dict[str, np.ndarray] = {
            "left": self._addrs(LEFT_MJCF_JOINTS),
            "right": self._addrs(RIGHT_MJCF_JOINTS),
        }
        self._cmd_names = {
            "left": LEFT_ARM_JOINT_NAMES,
            "right": RIGHT_ARM_JOINT_NAMES,
        }

        q_l = np.asarray(
            self.get_parameter("init_pose_left").value, dtype=float
        ).reshape(7)
        q_r = np.asarray(
            self.get_parameter("init_pose_right").value, dtype=float
        ).reshape(7)
        self._latest = {"left": q_l.copy(), "right": q_r.copy()}
        self._apply_arms(q_l, q_r)

        self._lock = threading.Lock()
        self._cmd_count = {"left": 0, "right": 0}
        interval = float(self.get_parameter("fps_print_interval").value)
        self._cmd_fps = {
            "left": FPSCounter(print_interval=interval),
            "right": FPSCounter(print_interval=interval),
        }
        self._sim_fps = FPSCounter(window=200, print_interval=interval)

        qos = _sensor_qos()
        self.create_subscription(
            JointState, "/left_arm/joint_commands", self._on_left, qos
        )
        self.create_subscription(
            JointState, "/right_arm/joint_commands", self._on_right, qos
        )

        self._pub_state = bool(self.get_parameter("publish_joint_states").value)
        self._state_pubs = {}
        if self._pub_state:
            self._state_pubs["left"] = self.create_publisher(
                JointState, "/left_arm/joint_states", qos
            )
            self._state_pubs["right"] = self.create_publisher(
                JointState, "/right_arm/joint_states", qos
            )
            rate = float(self.get_parameter("state_rate").value)
            self.create_timer(1.0 / max(1.0, rate), self._publish_states)

        self.enable_viewer = bool(self.get_parameter("enable_viewer").value)
        self.realtime = bool(self.get_parameter("realtime").value)
        self.get_logger().info(
            f"Astral MuJoCo sim ready: nq={self.model.nq} viewer={self.enable_viewer} "
            f"topics=/left_arm|/right_arm/joint_commands"
        )

    def _addrs(self, joint_names: List[str]) -> np.ndarray:
        addrs = []
        for name in joint_names:
            jid = self._mujoco.mj_name2id(
                self.model, self._mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if jid < 0:
                raise RuntimeError(f"MJCF missing joint '{name}'")
            addrs.append(int(self.model.jnt_qposadr[jid]))
        return np.asarray(addrs, dtype=int)

    def _on_left(self, msg: JointState) -> None:
        self._on_cmd("left", msg)

    def _on_right(self, msg: JointState) -> None:
        self._on_cmd("right", msg)

    def _on_cmd(self, side: str, msg: JointState) -> None:
        q = _pack_arm(msg, self._cmd_names[side])
        if q is None:
            return
        with self._lock:
            self._latest[side] = q
            self._cmd_count[side] += 1
        fps = self._cmd_fps[side].tick()
        if self._cmd_fps[side].should_print():
            self.get_logger().info(
                f"[Sim Cmd FPS][{side}] {fps:.0f} Hz total={self._cmd_count[side]}"
            )

    def _apply_arms(self, q_l: np.ndarray, q_r: np.ndarray) -> None:
        for i, a in enumerate(self._qpos_addrs["left"]):
            self.data.qpos[a] = float(q_l[i])
        for i, a in enumerate(self._qpos_addrs["right"]):
            self.data.qpos[a] = float(q_r[i])
        # Keep actuators in sync for passive viewer
        for i, name in enumerate(LEFT_MJCF_JOINTS):
            aid = self._mujoco.mj_name2id(
                self.model, self._mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{name}"
            )
            if aid >= 0:
                self.data.ctrl[aid] = float(q_l[i])
        for i, name in enumerate(RIGHT_MJCF_JOINTS):
            aid = self._mujoco.mj_name2id(
                self.model, self._mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{name}"
            )
            if aid >= 0:
                self.data.ctrl[aid] = float(q_r[i])
        self._mujoco.mj_forward(self.model, self.data)

    def _publish_states(self) -> None:
        with self._lock:
            ql = self._latest["left"].copy()
            qr = self._latest["right"].copy()
        now = self.get_clock().now().to_msg()
        for side, q, names in (
            ("left", ql, LEFT_ARM_JOINT_NAMES),
            ("right", qr, RIGHT_ARM_JOINT_NAMES),
        ):
            msg = JointState()
            msg.header.stamp = now
            msg.name = list(names)
            msg.position = [float(x) for x in q.tolist()]
            self._state_pubs[side].publish(msg)

    def run(self) -> None:
        viewer = None
        if self.enable_viewer:
            viewer = self._viewer_mod.launch_passive(self.model, self.data)
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -15
            viewer.cam.distance = 1.6
            viewer.cam.lookat[:] = [0.0, 0.0, 0.35]

        try:
            while rclpy.ok() and (viewer is None or viewer.is_running()):
                rclpy.spin_once(self, timeout_sec=0.0)
                with self._lock:
                    ql = self._latest["left"].copy()
                    qr = self._latest["right"].copy()
                self._apply_arms(ql, qr)
                if viewer is not None:
                    viewer.sync()
                sim_fps = self._sim_fps.tick()
                if self._sim_fps.should_print():
                    self.get_logger().info(
                        f"[Sim Loop FPS] {sim_fps:.0f} Hz "
                        f"cmds L={self._cmd_count['left']} R={self._cmd_count['right']}"
                    )
                if self.realtime:
                    time.sleep(self.model.opt.timestep)
        finally:
            if viewer is not None:
                viewer.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AstralMujocoSimNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
