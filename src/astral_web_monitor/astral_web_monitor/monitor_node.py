"""ROS 2 node: read-only subscriptions + thread-safe snapshot + pause/resume.

This node is the only ROS surface of the monitor. It:
  * subscribes to joint state/command topics (read-only, never republishes them)
  * caches the latest values behind a lock (latest-wins, like a SHM ring)
  * exposes pause()/resume() which publish ONE latched Bool to the existing
    /teleop/disarm and /teleop/armed topics — no new control topics are created
  * runs rclpy.spin in a background thread so the FastAPI/uvicorn event loop
    can run on the main thread

It deliberately does NOT subscribe to any hardware command topics and does
NOT call any driver services. Start/stop of the teleop stack is handled by
LaunchManager via subprocess, not by this node.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .config import (
    STALE_THRESHOLD_S,
    TOPICS,
    TOPIC_ARMED,
    TOPIC_DISARM,
)
from .rate_counter import RateRegistry


def _qos_best_effort() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


@dataclass
class JointSlot:
    values: list[float] = field(default_factory=list)
    ts: float = 0.0

    def is_stale(self, threshold: float = STALE_THRESHOLD_S, now: float | None = None) -> bool:
        if not self.ts:
            return True
        now = now if now is not None else time.time()
        return (now - self.ts) > threshold


class MonitorNode(Node):
    """Read-only monitor + pause/resume publisher."""

    def __init__(self) -> None:
        super().__init__("astral_web_monitor")
        self._lock = threading.Lock()
        self._state: dict[str, JointSlot] = {k: JointSlot() for k in TOPICS}
        self._rates = RateRegistry()

        # Pause/resume publishers (latched/transient so late arm nodes pick up).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_armed = self.create_publisher(Bool, TOPIC_ARMED, qos)
        self._pub_disarm = self.create_publisher(Bool, TOPIC_DISARM, qos)

        # Read-only subscriptions.
        for entity, pair in TOPICS.items():
            self.create_subscription(
                JointState, pair.state,
                lambda msg, e=entity: self._on_state(e, msg),
                _qos_best_effort(),
            )
            if pair.command:
                self.create_subscription(
                    JointState, pair.command,
                    lambda msg, e=entity: self._rates.tick(f"{entity}_cmd"),
                    _qos_best_effort(),
                )

        self.get_logger().info(
            "astral_web_monitor ready (read-only subs + pause/resume pubs)"
        )

    # --- callbacks (write snapshot) ---------------------------------------
    def _on_state(self, entity: str, msg: JointState) -> None:
        now = time.time()
        with self._lock:
            slot = self._state[entity]
            slot.values = list(msg.position)
            slot.ts = now

    # --- snapshot read (web thread) ---------------------------------------
    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            joints = {
                e: {
                    "values": list(s.values),
                    "ts": s.ts,
                    "stale": s.is_stale(now=now),
                }
                for e, s in self._state.items()
            }
        return {
            "joints": joints,
            "rates_hz": self._rates.snapshot(),
        }

    # --- pause/resume -----------------------------------------------------
    def publish_disarm(self) -> None:
        self._pub_disarm.publish(Bool(data=True))

    def publish_arm(self) -> None:
        self._pub_armed.publish(Bool(data=True))


# --- module-level singleton helpers -----------------------------------------
_node: MonitorNode | None = None
_spin_thread: threading.Thread | None = None


def init_node() -> MonitorNode:
    """Create the node and start a background rclpy spin thread."""
    global _node, _spin_thread
    if _node is not None:
        return _node
    rclpy.init(args=None)
    _node = MonitorNode()
    _spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    _spin_thread.start()
    return _node


def get_node() -> MonitorNode | None:
    return _node


def shutdown_node() -> None:
    global _node, _spin_thread
    if _node is not None:
        _node.publish_disarm()  # safety: disarm on monitor exit
    rclpy.shutdown()
    _node = None
    _spin_thread = None
