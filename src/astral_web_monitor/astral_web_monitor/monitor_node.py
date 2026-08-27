"""ROS 2 node: read-only subscriptions + thread-safe snapshot + pause/resume.

This node is the only ROS surface of the monitor. It:
  * subscribes to joint state/command topics (read-only, never republishes them)
  * caches the latest values behind a lock (latest-wins, like a SHM ring)
  * exposes pause()/resume() which publish ONE latched Bool to the existing
    /teleop/disarm and /teleop/armed topics — no new control topics are created
  * runs rclpy.spin in a background thread so the FastAPI/uvicorn event loop
    can run on the main thread
  * optionally drives the quest3_video_streamer runtime gate (SetBool master
    switch + latched active-camera subset) and mirrors its latched gate_state

It deliberately does NOT subscribe to any hardware command topics. Hardware
mode changes go through the driver's already-exposed services. Start/stop of
the teleop stack is handled by LaunchManager via subprocess, not by this node.
"""
from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from .config import (
    STALE_THRESHOLD_S,
    TOPICS,
    TOPIC_ARMED,
    TOPIC_DISARM,
    TOPIC_START,
    EXPECTED_RATES_HZ,
    DRIVER_SRV_READY,
    DRIVER_SRV_HOME,
    DRIVER_SRV_ESTOP,
    DRIVER_SRV_DAMPING,
    DRIVER_SRV_POSITION,
    VIDEO_SRV_PUSH,
    VIDEO_TOPIC_CAMERAS,
    VIDEO_TOPIC_GATE_STATE,
    VIDEO_TOPIC_PREVIEW,
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
        # State-topic rate counters (one per entity) for health inspection.
        self._state_rates = RateRegistry.for_state_topics()
        # Cached driver service clients (created lazily on first call).
        self._driver_clients: dict[str, object] = {}

        # Pause/resume publishers (latched/transient so late arm nodes pick up).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_armed = self.create_publisher(Bool, TOPIC_ARMED, qos)
        self._pub_disarm = self.create_publisher(Bool, TOPIC_DISARM, qos)

        # One-shot start trigger (volatile, not latched): publishing here is the
        # only write path for /teleop/start. Used by require_start_signal to
        # capture vr_init from the current pose and arm both arms at once.
        start_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self._pub_start = self.create_publisher(Bool, TOPIC_START, start_qos)

        # Video gate: latched active-cameras publisher + gate_state mirror.
        self._pub_video_cameras = self.create_publisher(String, VIDEO_TOPIC_CAMERAS, qos)
        self._video_gate_state: dict[str, Any] | None = None
        self.create_subscription(
            String, VIDEO_TOPIC_GATE_STATE, self._on_video_gate_state, qos,
        )
        self._video_push_client: Any = None  # SetBool client, created lazily
        # Web preview: latest JPEG bytes per camera label + dynamic subscriptions
        # (created when gate_state reports the camera list).
        self._preview_jpeg: dict[str, bytes] = {}
        self._preview_seq: dict[str, int] = {}
        self._preview_subs: dict[str, Any] = {}

        # Read-only subscriptions.
        for entity, pair in TOPICS.items():
            self.create_subscription(
                JointState, pair.state,
                lambda msg, e=entity: self._on_state(e, msg),
                _qos_best_effort(),
            )
            # State-topic rate tick (health: is the source itself alive?).
            self.create_subscription(
                JointState, pair.state,
                lambda msg, e=entity: self._state_rates.tick(f"{e}_state"),
                _qos_best_effort(),
            )
            if pair.command:
                cmd_type = Float64 if pair.cmd_kind == "float64" else JointState
                self.create_subscription(
                    cmd_type, pair.command,
                    lambda msg, e=entity: self._rates.tick(f"{e}_cmd"),
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

    def _on_video_gate_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            with self._lock:
                self._video_gate_state = data
            self._ensure_preview_subs(data)

    def _ensure_preview_subs(self, gate: dict[str, Any]) -> None:
        """Subscribe ~/preview/{label} for every camera the streamer reports."""
        from sensor_msgs.msg import CompressedImage

        labels = [
            str(c.get("label", ""))
            for c in (gate.get("cameras") or [])
            if c.get("label")
        ]
        if not labels:
            labels = [str(x) for x in (gate.get("configured") or [])]
        preview_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for label in labels:
            if not label or label in self._preview_subs:
                continue
            self._preview_subs[label] = self.create_subscription(
                CompressedImage,
                f"{VIDEO_TOPIC_PREVIEW}/{label}",
                lambda m, lb=label: self._on_preview(lb, m),
                preview_qos,
            )

    def _on_preview(self, label: str, msg: Any) -> None:
        data = bytes(msg.data)
        with self._lock:
            self._preview_jpeg[label] = data
            self._preview_seq[label] = self._preview_seq.get(label, 0) + 1

    def preview_frame(self, label: str) -> tuple[bytes | None, int]:
        """(latest JPEG bytes, sequence) for the web MJPEG relay."""
        with self._lock:
            return self._preview_jpeg.get(label), self._preview_seq.get(label, 0)

    def preview_labels(self) -> list[str]:
        """Labels that have a preview subscription (i.e. known cameras)."""
        return list(self._preview_subs.keys())

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
        cmd_rates = self._rates.snapshot()
        state_rates = self._state_rates.snapshot()
        with self._lock:
            video_gate = dict(self._video_gate_state) if self._video_gate_state else None
        return {
            "joints": joints,
            "rates_hz": cmd_rates,
            "state_rates_hz": state_rates,
            "health": self._health_summary(joints, cmd_rates, state_rates, now),
            "video_gate": video_gate,
        }

    @staticmethod
    def _health_summary(
        joints: dict[str, Any],
        cmd_rates: dict[str, float],
        state_rates: dict[str, float],
        now: float,
    ) -> dict[str, Any]:
        """Per-entity health: alive (state stream fresh) + command rate vs floor."""
        entities: dict[str, Any] = {}
        any_stale = False
        any_slow = False
        for e, slot in joints.items():
            stale = bool(slot["stale"])
            state_hz = float(state_rates.get(f"{e}_state", 0.0))
            cmd_key = f"{e}_cmd"
            cmd_hz = float(cmd_rates.get(cmd_key, 0.0))
            expected = float(EXPECTED_RATES_HZ.get(cmd_key, 0.0))
            slow = expected > 0.0 and cmd_hz < expected and not stale
            if stale:
                any_stale = True
            if slow:
                any_slow = True
            entities[e] = {
                "stale": stale,
                "state_hz": state_hz,
                "cmd_hz": cmd_hz,
                "expected_hz": expected,
                "slow": slow,
                "status": "stale" if stale else ("slow" if slow else "ok"),
            }
        overall = "ok"
        if any_stale:
            overall = "stale"
        elif any_slow:
            overall = "slow"
        return {"overall": overall, "entities": entities}

    # --- pause/resume -----------------------------------------------------
    def publish_disarm(self) -> None:
        self._pub_disarm.publish(Bool(data=True))

    def publish_arm(self) -> None:
        self._pub_armed.publish(Bool(data=True))

    def publish_start(self) -> None:
        """One-shot /teleop/start: arm teleop captures vr_init and arms."""
        self._pub_start.publish(Bool(data=True))

    # --- driver service calls (hardware mode) -----------------------------
    # The driver node (astral_robot_control) already exposes Trigger services
    # for one_click_ready / home / e_stop / damping / position. The monitor
    # calls them as a client — non-intrusive (the driver owns the hardware).
    # Clients are cached; the background spin thread completes the futures.
    _DRIVER_SERVICES = {
        "ready": DRIVER_SRV_READY,
        "home": DRIVER_SRV_HOME,
        "estop": DRIVER_SRV_ESTOP,
        "damping": DRIVER_SRV_DAMPING,
        "position": DRIVER_SRV_POSITION,
    }

    def call_driver_service(self, name: str, timeout_s: float = 4.0) -> tuple[bool, str]:
        """Invoke a driver Trigger service by short name (ready/home/estop/...).

        Returns (success, message). The driver is the hardware authority; this
        only calls its already-exposed service. Safe to call from the web
        thread — the background rclpy spin thread completes the future.
        """
        from std_srvs.srv import Trigger  # local import keeps module import light
        srv_name = self._DRIVER_SERVICES.get(name)
        if srv_name is None:
            return False, f"unknown driver service: {name}"
        cli = self._driver_clients.get(name)
        if cli is None:
            cli = self.create_client(Trigger, srv_name)
            self._driver_clients[name] = cli
        if not cli.service_is_ready():
            if not cli.wait_for_service(timeout_sec=2.0):
                return False, f"driver service 未就绪: {srv_name}（driver 未启动？）"
        future = cli.call_async(Trigger.Request())
        deadline = time.time() + timeout_s
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            return False, f"driver service 超时: {srv_name}"
        res = future.result()
        return bool(res.success), str(res.message)

    # --- video return gate (quest3_video_streamer) --------------------------
    # Non-intrusive: the streamer exposes ~/set_push_enabled (SetBool) and
    # ~/active_cameras (latched String); the monitor only calls/publishes them.
    def set_video_push(self, enabled: bool, timeout_s: float = 4.0) -> tuple[bool, str]:
        """Master push switch on the video streamer. Returns (success, message)."""
        from std_srvs.srv import SetBool
        if self._video_push_client is None:
            self._video_push_client = self.create_client(SetBool, VIDEO_SRV_PUSH)
        cli = self._video_push_client
        if not cli.service_is_ready():
            if not cli.wait_for_service(timeout_sec=2.0):
                return False, f"video service 未就绪: {VIDEO_SRV_PUSH}（streamer 未启动？）"
        req = SetBool.Request()
        req.data = bool(enabled)
        future = cli.call_async(req)
        deadline = time.time() + timeout_s
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            return False, f"video service 超时: {VIDEO_SRV_PUSH}"
        res = future.result()
        return bool(res.success), str(res.message)

    def publish_video_cameras(self, cameras: list[str]) -> None:
        """Latched active-camera subset (empty list = all configured cameras)."""
        msg = String()
        msg.data = ",".join(c.strip() for c in cameras if c.strip())
        self._pub_video_cameras.publish(msg)


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
