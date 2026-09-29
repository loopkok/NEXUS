#!/usr/bin/env python3
"""hand_landmarks → Wuji Hand joint_commands (20-DoF).

Backends:
  official         — wuji_sdk RetargetSession (rob_station aligned)
  wuji_retargeting — open-source wuji_retargeting.Retargeter (teleop yaml)
  dexpilot         — dex-retargeting DexPilot on Wuji URDF (from xhand)

Publishes sensor_msgs/JointState.position[20] to /{hand_name}/joint_commands
(index order, no name field required by wujihandros2).
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


def _command_qos() -> QoSProfile:
    """Reliable, latest-only delivery for retargeted command candidates."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

from wujihand_retargeting.constants import (
    DEFAULT_HAND_MODEL,
    NUM_JOINTS,
    WUJI_JOINT_NAMES,
)
from wujihand_retargeting.fps_counter import FPSCounter


def _env_or_default(name: str, default: str) -> str:
    val = os.environ.get(name)
    if val is None or str(val).strip() == "":
        return default
    return str(val).strip()


class WujiHandRetargetNode(Node):
    def __init__(self):
        super().__init__("wujihand_retarget_node")

        default_side = _env_or_default("GLOVE_HAND_SIDE", "right")
        default_model = _env_or_default("GLOVE_HAND_MODEL", DEFAULT_HAND_MODEL)

        self.declare_parameter("hand_side", default_side)  # left|right|both
        # official | wuji_retargeting | dexpilot
        self.declare_parameter("retarget_backend", "official")
        self.declare_parameter("hand_model", default_model)  # rob_station GLOVE_HAND_MODEL
        self.declare_parameter("input_topic", "hand_landmarks")
        self.declare_parameter("left_hand_name", "left_hand")
        self.declare_parameter("right_hand_name", "right_hand")
        self.declare_parameter("smoothing_alpha", 1.0)  # 1=off; official 无额外 EMA
        self.declare_parameter("dexpilot_config_left", "")
        self.declare_parameter("dexpilot_config_right", "")
        self.declare_parameter("dexpilot_urdf_left", "")
        self.declare_parameter("dexpilot_urdf_right", "")
        # wuji_retargeting (open-source) yaml overrides + speed knobs
        self.declare_parameter("wuji_lib_config_left", "")
        self.declare_parameter("wuji_lib_config_right", "")
        self.declare_parameter("nlopt_max_eval", 25)  # teleop-style; 0=library default
        self.declare_parameter("wuji_lib_disable_timing", True)
        self.declare_parameter("home_position", [0.0] * NUM_JOINTS)
        self.declare_parameter("viz", False)
        self.declare_parameter("fps_print_interval", 5.0)

        self.hand_side = str(self.get_parameter("hand_side").value).lower()
        self.backend_name = str(self.get_parameter("retarget_backend").value).lower()
        self.hand_model = str(self.get_parameter("hand_model").value).strip()
        self.input_topic = str(self.get_parameter("input_topic").value).rstrip("/")
        self.left_hand_name = str(self.get_parameter("left_hand_name").value)
        self.right_hand_name = str(self.get_parameter("right_hand_name").value)
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.viz = bool(self.get_parameter("viz").value)
        self.nlopt_max_eval = int(self.get_parameter("nlopt_max_eval").value)
        self.wuji_lib_disable_timing = bool(
            self.get_parameter("wuji_lib_disable_timing").value
        )
        home = list(self.get_parameter("home_position").value)
        if len(home) < NUM_JOINTS:
            home = home + [0.0] * (NUM_JOINTS - len(home))
        self.home_position = [float(v) for v in home[:NUM_JOINTS]]

        if self.hand_side not in ("left", "right", "both"):
            raise ValueError(f"hand_side 须为 left|right|both，收到 {self.hand_side}")
        if self.backend_name not in ("official", "wuji_retargeting", "dexpilot"):
            raise ValueError(
                "retarget_backend 须为 official|wuji_retargeting|dexpilot，"
                f"收到 {self.backend_name}"
            )

        sides = ("left", "right") if self.hand_side == "both" else (self.hand_side,)
        self._backends: Dict[str, object] = {}
        self._last_qpos: Dict[str, Optional[List[float]]] = {
            "left": None, "right": None
        }
        self._cmd_pub: Dict[str, object] = {}
        self._viz_pub: Dict[str, object] = {}
        fps_interval = float(self.get_parameter("fps_print_interval").value)
        self._fps: Dict[str, FPSCounter] = {
            s: FPSCounter(window=100, print_interval=fps_interval) for s in ("left", "right")
        }

        for side in sides:
            self._backends[side] = self._make_backend(side)
            hand_name = (
                self.left_hand_name if side == "left" else self.right_hand_name
            )
            self._cmd_pub[side] = self.create_publisher(
                JointState,
                f"/{hand_name}/joint_commands",
                _command_qos(),
            )
            if self.viz:
                self._viz_pub[side] = self.create_publisher(
                    JointState, f"/{hand_name}/retarget_joint_states", 10
                )
            self.create_subscription(
                PoseArray,
                f"{self.input_topic}/{side}",
                lambda msg, s=side: self._on_landmarks(msg, s),
                10,
            )
            self.create_service(
                Trigger,
                f"~/home_{side}",
                lambda req, res, s=side: self._home_cb(req, res, s),
            )

        self.get_logger().info(
            f"WujiHand retarget ready: side={self.hand_side}, "
            f"backend={self.backend_name}, hand_model={self.hand_model}, "
            f"input={self.input_topic}/{{side}}, "
            f"smoothing_alpha={self.smoothing_alpha}"
        )

    def _make_backend(self, side: str):
        if self.backend_name == "official":
            from wujihand_retargeting.backends.official import OfficialRetargetBackend

            return OfficialRetargetBackend(
                hand_side=side, hand_model=self.hand_model
            )

        if self.backend_name == "wuji_retargeting":
            from wujihand_retargeting.backends.wuji_lib import WujiLibRetargetBackend

            cfg = self.get_parameter(f"wuji_lib_config_{side}").value
            backend = WujiLibRetargetBackend(
                hand_side=side,
                config_path=str(cfg).strip() or None,
                nlopt_max_eval=self.nlopt_max_eval,
                disable_timing=self.wuji_lib_disable_timing,
            )
            self.get_logger().info(
                f"wuji_retargeting[{side}] yaml={backend.config_path}"
            )
            return backend

        from wujihand_retargeting.backends.dexpilot import DexPilotRetargetBackend

        cfg = self.get_parameter(f"dexpilot_config_{side}").value
        urdf = self.get_parameter(f"dexpilot_urdf_{side}").value
        return DexPilotRetargetBackend(
            hand_side=side,
            config_path=str(cfg).strip() or None,
            urdf_path=str(urdf).strip() or None,
        )

    def _on_landmarks(self, msg: PoseArray, side: str) -> None:
        if len(msg.poses) < 21:
            self.get_logger().warning(
                f"[{side}] PoseArray len={len(msg.poses)} < 21，跳过"
            )
            return

        kp = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in msg.poses[:21]],
            dtype=np.float32,
        )
        try:
            qpos = self._backends[side].retarget(kp)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[{side}] retarget failed: {exc}")
            return

        qpos = np.asarray(qpos, dtype=np.float64).reshape(-1)
        if qpos.size != NUM_JOINTS:
            self.get_logger().error(
                f"[{side}] qpos dim={qpos.size} != {NUM_JOINTS}"
            )
            return

        values = [float(v) for v in qpos]
        if self.smoothing_alpha < 1.0 and self._last_qpos[side] is not None:
            a = self.smoothing_alpha
            prev = self._last_qpos[side]
            values = [a * c + (1.0 - a) * p for c, p in zip(values, prev)]
        self._last_qpos[side] = values

        self._publish_cmd(side, values)

        fps = self._fps[side].tick()
        if self._fps[side].should_print():
            self.get_logger().info(
                f"[Retarget FPS][{side}][{self.backend_name}] {fps:.0f} Hz"
            )

    def _publish_cmd(self, side: str, values: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # wujihandros2 parses by index; names optional but useful for debug
        msg.name = list(WUJI_JOINT_NAMES)
        msg.position = [float(v) for v in values]
        self._cmd_pub[side].publish(msg)
        if side in self._viz_pub:
            self._viz_pub[side].publish(msg)

    def _home_cb(self, request, response, side: str):
        self._publish_cmd(side, self.home_position)
        self._last_qpos[side] = list(self.home_position)
        response.success = True
        response.message = f"{side} hand homed"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WujiHandRetargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
