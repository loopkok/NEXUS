#!/usr/bin/env python3
"""ROS 2 policy node: deploy / replay / human-in-the-loop.

Modes (see :mod:`controller`): IDLE, POLICY(_PAUSED), PLAYBACK(_PAUSED), HUMAN.

Control flow (one timer at ``dataset_fps * control_interp``):

* POLICY   — latest obs (state + cameras) → engine ``tick()`` at the policy
  rate; each absolute action row is split per schema, safety-passed by the
  :class:`~astral_policy_inference.executor.SafeExecutor` and published on the
  existing arm/gripper/head command topics (last-writer-wins arbitration: we
  publish only in POLICY/PLAYBACK, and disarm teleop on entry).
* PLAYBACK — :class:`PlaybackSession` steps a recorded episode; human takeover
  re-anchors an offset to the robot's actual pose so the remainder continues
  without a jump.
* HUMAN    — VR teleop takes over via ``astral_arm_teleop`` ``~/reanchor``
  (robot origin ← FK(current joints), vr_init ← current VR pose). Returning to
  POLICY re-plans from the live state; returning to PLAYBACK re-anchors.

Commands arrive on ``~/cmd`` (String) — verbs: policy | playback | pause |
resume | takeover | release | stop, optionally ``playback:<source>:<episode>``.
The same verbs are sent by :mod:`keyboard`.

Robot schema parameters mirror ``astral_data_collect/data_collect.yaml`` and
build the shared :class:`~astral_data_collect.schema.CollectSchema`, so any
upstream robot-config change propagates here with no code edits.
"""

from __future__ import annotations

import collections
import json
import queue
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from astral_data_collect.schema import CollectSchema
from astral_policy_inference.backend import ObsBatch, make_backend
from astral_policy_inference.controller import Controller, InvalidTransition
from astral_policy_inference.engine import ActionEngine, EngineStateError
from astral_policy_inference.executor import SafeExecutor
from astral_policy_inference.replay import PlaybackSession, load_replay
from astral_policy_inference.robot_io import (
    ObsLayout,
    decode_jpeg_rgb,
    letterbox,
)

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class PolicyNode(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__("policy_node", parameter_overrides=parameter_overrides)
        self._declare_params()
        self._load_robot_params()

        self.controller = Controller()
        # Serializes the arbitration/control layer: every *cmd_* handler and the
        # control timer _tick run under this lock (single-writer guarantee for
        # command topics + engine/session swaps). Sensor subscription callbacks
        # (_on_joints/_on_ratio/_on_image/_on_task) stay lock-free on purpose.
        self._lock = threading.RLock()
        # /~/cmd strings are queued here by the ROS callback and drained by the
        # control timer, so slow transitions (engine build, reanchor waits) never
        # run inside an rclpy executor thread where the wait-set would starve.
        self._cmd_queue: "queue.Queue[str]" = queue.Queue()
        self._engine: Optional[ActionEngine] = None
        self._executor = SafeExecutor(
            max_joint_vel=float(self.get_parameter("max_joint_vel").value),
        )
        self._exec_events: list[str] = []
        self._playback: Optional[PlaybackSession] = None
        self._last_cmds: list = []
        self._ratio_last: dict[str, tuple[float, float]] = {}  # topic -> (val, t)
        # driver /{side}_gripper/joint_states（last-commanded rad 回显，state
        # 定时器每拍都发）缓存：夹爪命令从未出现时的 ratio 种子来源
        self._grip_rad: dict[str, np.ndarray] = {}
        self._grip_open_rad = float(self.get_parameter("gripper_open_rad").value)
        self._grip_closed_rad = float(self.get_parameter("gripper_closed_rad").value)
        self._prompt = str(self.get_parameter("default_prompt").value or "")

        # observation buffers ---------------------------------------------------
        self._values: dict[str, np.ndarray] = {}
        self._stamps: dict[str, float] = {}
        self._images: dict[str, np.ndarray] = {}
        self._image_stamps: dict[str, float] = {}
        self._cam_frames = 0
        # 端到端控制延迟统计（新观测 → 指令发布）：loop_ms=节点处理耗时, obs_age=观测龄期
        self._loop_ms: "collections.deque[float]" = collections.deque(maxlen=50)
        self._obs_age_ms: "collections.deque[float]" = collections.deque(maxlen=50)

        self._ctrl_rate = float(
            self.get_parameter("dataset_fps").value
        ) * int(self.get_parameter("control_interp").value)
        self._policy_dt = 1.0 / float(self.get_parameter("dataset_fps").value)
        self._acc = 0.0
        self._sub = 0
        self._hold_end: Optional[float] = None
        self._seg_prev: Optional[np.ndarray] = None
        self._seg_cur: Optional[np.ndarray] = None
        self._interp = int(self.get_parameter("control_interp").value)
        self._obs_stale_stop_s = float(self.get_parameter("obs_stale_stop_s").value)
        self._obs_lost_since: Optional[float] = None
        # Event-driven HUMAN-takeover handshake: re-anchor requests are sent
        # async and polled by the control timer, so the effect never blocks an
        # rclpy executor thread (which would starve the wait-set that completes
        # the very same service responses).
        self._takeover_pending: Optional[dict[str, object]] = None
        self._takeover_deadline = 0.0
        self._takeover_engine_was_enabled = False

        self._setup_io()
        self._pubs: dict[tuple[str, str], object] = {}
        self._teleop_disarm = self.create_publisher(
            Bool, self.get_parameter("teleop_disarm_topic").value, _LATCHED_QOS
        )
        self._state_pub = self.create_publisher(
            String, self.get_parameter("state_topic").value, _LATCHED_QOS
        )
        self._cmd_sub = self.create_subscription(
            String, self.get_parameter("cmd_topic").value, self._on_cmd, 10
        )
        self._task_sub = self.create_subscription(
            String, self.get_parameter("task_topic").value, self._on_task, 10
        )
        self._reanchor_clients = {
            name: self.create_client(Trigger, name)
            for name in self.get_parameter("teleop_reanchor_services").value
        }

        self._timer = self.create_timer(1.0 / max(1.0, self._ctrl_rate), self._tick)
        self._status_timer = self.create_timer(1.0, self._publish_state)
        self._last_publish = 0.0
        self.get_logger().info(
            f"policy_node up: schema={self._layout.state_dim}D "
            f"arms={self._schema.arms} ee=({self._schema.end_effector_left},"
            f"{self._schema.end_effector_right}) fps={self._schema.dataset_fps} "
            f"ctrl={self._ctrl_rate}Hz mode={self.get_parameter('engine_mode').value}"
        )

    # ------------------------------------------------------------- parameters

    def _declare_params(self) -> None:
        defaults = {
            # robot / schema
            "arms": ["left"],
            "end_effector_left": "gripper",
            "end_effector_right": "none",
            "include_waist": False,
            "include_head": False,
            "cameras": ["video8", "video0", "video2"],
            # JSON-encoded {model_camera_name: quest3_collect_label} map
            "camera_map": (
                '{"base_0_rgb": "video8", "left_wrist_0_rgb": "video0"}'
            ),
            "dataset_fps": 30,
            # backend / engine
            "backend_type": "remote",  # remote | inproc | stub（传输方式）
            "model": "act",            # act | pi05 | ...（模型族）
            "checkpoint_dir": "",
            "host": "127.0.0.1",
            "port": 8000,
            "engine_mode": "queue_async",
            "action_chunk": 50,
            "control_interp": 1,
            "camera_image_size": 224,
            "abs_action_min_scale": 0.5,
            "default_prompt": "",
            # replay
            "replay_source": "",
            "replay_episode": 0,
            "replay_hold_s": 1.0,
            # safety
            "max_joint_vel": 6.0,
            "obs_timeout_s": 0.5,
            "obs_stale_stop_s": 1.0,   # consecutive obs loss before POLICY auto-pauses
            "image_timeout_s": 1.0,
            "image_required": False,
            # 夹爪 rad↔ratio 线性映射（须与 driver 的 *_gripper_open/closed_rad
            # 一致）：首次进 POLICY 且 /{side}_gripper/command 从未出现时，用
            # driver /{side}_gripper/joint_states（last-commanded 回显）换算
            # ratio 作夹爪状态种子，避免"没命令过→obs 永远缺夹爪→进不了策略"
            "gripper_open_rad": 0.8,
            "gripper_closed_rad": 0.0,
            # hitl / topics
            "teleop_reanchor_services": ["/astral_arm_teleop_left/reanchor"],
            "teleop_disarm_topic": "/teleop/disarm",
            "teleop_start_topic": "/teleop/start",
            "cmd_topic": "/policy_inference/cmd",
            "state_topic": "/policy_inference/state",
            "task_topic": "/policy_inference/task",
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)

    def _load_robot_params(self) -> None:
        arms = [str(s) for s in self.get_parameter("arms").value]
        schema = CollectSchema(
            arms=arms,
            end_effector_left=str(self.get_parameter("end_effector_left").value),
            end_effector_right=str(self.get_parameter("end_effector_right").value),
            include_waist=bool(self.get_parameter("include_waist").value),
            include_head=bool(self.get_parameter("include_head").value),
            cameras=[str(c) for c in self.get_parameter("cameras").value],
            dataset_fps=int(self.get_parameter("dataset_fps").value),
        )
        camera_map = self._parse_camera_map(self.get_parameter("camera_map").value)
        labels = {str(v) for v in camera_map.values()}
        unknown = labels - set(schema.cameras)
        if unknown:
            self.get_logger().warn(
                f"camera_map labels {sorted(unknown)} not in cameras list; "
                "still subscribing (streamer label may differ from recorded set)"
            )
        self._schema = schema
        self._layout = ObsLayout(schema)
        self._topic_to_key = {src.topic: k for k, src in self._layout.sources.items()}
        self._camera_map = {str(k): str(v) for k, v in camera_map.items()}
        self._image_size = int(self.get_parameter("camera_image_size").value)
        self._obs_timeout = float(self.get_parameter("obs_timeout_s").value)
        self._image_timeout = float(self.get_parameter("image_timeout_s").value)
        self._image_required = bool(self.get_parameter("image_required").value)
        self._backend_cfg = {
            "backend_type": str(self.get_parameter("backend_type").value),
            "model": str(self.get_parameter("model").value),
            "checkpoint_dir": str(self.get_parameter("checkpoint_dir").value),
            "host": str(self.get_parameter("host").value),
            "port": int(self.get_parameter("port").value),
            "default_prompt": str(self.get_parameter("default_prompt").value),
            "action_dim": schema.state_dim,
            "camera_map": self._camera_map,
        }

    @staticmethod
    def _parse_camera_map(raw) -> dict:
        """camera_map arrives as JSON text (rclpy has no struct params)."""
        if isinstance(raw, dict):
            return {str(k): str(v) for k, v in raw.items()}
        text = str(raw or "").strip()
        if not text:
            return {}
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(
                "camera_map must be JSON text, e.g. "
                '{"base_0_rgb": "video8"}'
            ) from exc
        if not isinstance(data, dict):
            raise ValueError("camera_map JSON must be an object of string pairs")
        return {str(k): str(v) for k, v in data.items()}

    def _setup_io(self) -> None:
        for key, src in self._layout.sources.items():
            if src.kind == "ratio":
                cb = lambda msg, k=key: self._on_ratio(msg, k)  # noqa: E731
                self.create_subscription(Float64, src.topic, cb, _SENSOR_QOS)
                side = src.key.split("_", 1)[0]
                self.create_subscription(
                    JointState,
                    f"/{side}_gripper/joint_states",
                    lambda msg, s=side: self._on_gripper_rad(msg, s),
                    _SENSOR_QOS,
                )
            else:
                cb = lambda msg, k=key, d=src.dim: self._on_joints(msg, k, d)  # noqa: E731
                self.create_subscription(JointState, src.topic, cb, _SENSOR_QOS)
        for label in sorted({v for v in self._camera_map.values()}):
            topic = f"/quest3_video_streamer/collect/{label}"
            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, lab=label: self._on_image(msg, lab),
                _SENSOR_QOS,
            )

    # ------------------------------------------------------------ subscribers

    def _on_joints(self, msg: JointState, key: str, dim: int) -> None:
        arr = np.asarray(msg.position[:dim], dtype=np.float64)
        if arr.shape[0] < dim:
            return
        self._values[key] = arr.copy()
        self._stamps[key] = time.monotonic()

    def _on_ratio(self, msg: Float64, key: str) -> None:
        self._values[key] = np.asarray([msg.data], dtype=np.float64)
        self._stamps[key] = time.monotonic()

    def _on_gripper_rad(self, msg: JointState, side: str) -> None:
        if not msg.position:
            return
        self._grip_rad[side] = np.asarray([msg.position[0]], dtype=np.float64)

    def _on_image(self, msg: CompressedImage, label: str) -> None:
        img = decode_jpeg_rgb(msg.data)
        if img is None:
            self.get_logger().warn("collect image decode failed", throttle_duration_sec=5.0)
            return
        self._cam_frames += 1
        if self._image_size and (img.shape[0] != self._image_size or img.shape[1] != self._image_size):
            img = letterbox(img, self._image_size)
        self._images[label] = img
        self._image_stamps[label] = time.monotonic()

    def _on_task(self, msg: String) -> None:
        self._prompt = msg.data
        self.get_logger().info(f"task set -> {msg.data!r}")

    # -------------------------------------------------------------- commands

    def _on_cmd(self, msg: String) -> None:
        """ROS callback: enqueue only. The control timer drains the queue while
        holding ``self._lock``, so slow transitions never run on an rclpy
        executor thread (where a blocking wait would starve the wait-set)."""
        text = msg.data.strip()
        if text:
            self._cmd_queue.put_nowait(text)

    def _handle_cmd(self, text: str) -> None:
        """Synchronous command entry (ROS-timer drain + tests). Caller must not
        hold ``self._lock`` when it is invoked from outside."""
        text = text.strip()
        if not text:
            return
        with self._lock:
            self._exec_cmd(text)

    def _exec_cmd(self, text: str) -> None:
        verb = text.split(":", 1)[0].split()[0]
        self.get_logger().info(f"cmd {text!r}")
        try:
            if verb == "policy":
                self._cmd_policy()
            elif verb == "playback":
                self._cmd_playback(text)
            elif verb == "pause":
                self._cmd_pause()
            elif verb == "resume":
                self._cmd_resume()
            elif verb == "takeover":
                self._cmd_takeover()
            elif verb == "release":
                self._cmd_release()
            elif verb == "stop":
                self._cmd_stop()
            else:
                self.get_logger().warn(f"unknown cmd verb {verb!r}")
        except InvalidTransition as exc:
            self.get_logger().warn(f"transition rejected: {exc}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cmd {verb!r} failed: {exc}")

    def _drain_cmds(self) -> None:
        """Run queued commands now (caller holds ``self._lock``)."""
        while True:
            try:
                text = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            # While a takeover effect is in flight only 'stop' is honoured; the
            # pending re-anchor owns the transition and other verbs would race it.
            if self._takeover_pending is not None:
                verb = text.split(":", 1)[0].split()[0]
                if verb != "stop":
                    self.get_logger().warn(
                        f"cmd {verb!r} ignored during takeover effect"
                    )
                    continue
            try:
                self._exec_cmd(text)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"queued cmd {text!r} failed: {exc}")

    def _state_ok(self) -> tuple[Optional[np.ndarray], list[str]]:
        now = time.monotonic()
        vectors: dict[str, Optional[np.ndarray]] = {}
        for key, src in self._layout.sources.items():
            if key not in self._values:
                continue
            # Gripper has no true feedback: state == most recent commanded
            # ratio (collection convention). Once seen it stays valid; we also
            # seed from ratios this node itself commanded.
            if src.kind == "ratio":
                vectors[key] = self._values[key]
            elif now - self._stamps[key] <= self._obs_timeout:
                vectors[key] = self._values[key]
        for key, src in self._layout.sources.items():
            if key in vectors:
                continue
            if src.kind == "ratio":
                if src.topic in self._ratio_last:
                    val, _ts = self._ratio_last[src.topic]
                    vectors[key] = np.asarray([val], dtype=np.float64)
                elif src.key.split("_", 1)[0] in self._grip_rad:
                    # 命令从未出现且自身未发过：driver joint_states 的
                    # last-commanded rad 回显线性换算回 ratio（open↔closed）
                    side = src.key.split("_", 1)[0]
                    rad = float(self._grip_rad[side][0])
                    span = self._grip_open_rad - self._grip_closed_rad
                    if span <= 0.0:
                        continue
                    ratio = (self._grip_open_rad - rad) / span
                    vectors[key] = np.asarray(
                        [min(1.0, max(0.0, ratio))], dtype=np.float64
                    )
        state, missing = self._layout.assemble_state(vectors)
        images_missing = [
            lab for lab in self._camera_map.values()
            if lab not in self._images
            or now - self._image_stamps[lab] > self._image_timeout
        ]
        if self._image_required and images_missing:
            return None, [f"image:{m}" for m in images_missing]
        return state, missing + [f"image_stale:{m}" for m in images_missing]

    def _obs_batch(self, state: np.ndarray) -> ObsBatch:
        now = time.monotonic()
        images = {
            lab: arr
            for lab, arr in self._images.items()
            if now - self._image_stamps.get(lab, 0.0) <= self._image_timeout * 4
        }
        return ObsBatch(state=state, images=images, prompt=self._prompt)

    def _disarm_teleop(self) -> None:
        msg = Bool()
        msg.data = True
        self._teleop_disarm.publish(msg)

    def _release_teleop_gate(self) -> None:
        """Open the arbitration gate (disarm=False): teleop drives the robot.

        Used on entering HUMAN so the pinch/trigger gripper teleop resumes
        publishing its ratio stream (arm teleop is armed by the reanchor call).
        """
        msg = Bool()
        msg.data = False
        self._teleop_disarm.publish(msg)

    def _stop_engine(self) -> None:
        """Disable planning/ticking but keep the engine open (cheap hold)."""
        if self._engine is not None:
            try:
                self._engine.set_enabled(False)
                self._engine.reset()
            except Exception:  # noqa: BLE001
                pass
        self._seg_prev = None
        self._seg_cur = None
        self._acc = 0.0
        self._sub = 0

    def _teardown_engine(self) -> None:
        """Full engine teardown: join planner thread + close the backend."""
        self._stop_engine()
        if self._engine is not None:
            try:
                self._engine.stop()
            except Exception:  # noqa: BLE001
                pass
            self._engine = None

    def _new_engine(self) -> ActionEngine:
        return ActionEngine(
            make_backend(**self._backend_cfg),
            mode=str(self.get_parameter("engine_mode").value),
            action_dim=self._schema.state_dim,
            chunk=int(self.get_parameter("action_chunk").value),
            policy_fps=int(self.get_parameter("dataset_fps").value),
            autostart=True,
            abs_action_min_scale=float(
                self.get_parameter("abs_action_min_scale").value
            ),
        )

    def _cmd_policy(self) -> None:
        self.controller.request("policy")
        self._start_policy()

    def _start_policy(self) -> bool:
        """Enter/restart POLICY. Preconditions: ``controller`` already moved to
        the POLICY target via ``request()``; on any failure this reverts the FSM
        and leaves the previous engine/session resources intact, so the robot
        stays in the pre-transition activity instead of silently freezing.

        Order matters (adversarial-review F5): validate fresh obs → build+start
        the new engine while the old one is merely frozen → only then publish
        disarm (policy owns the command streams) and retire the old engine.
        """
        state, missing = self._state_ok()
        if state is None:
            self.controller.revert()
            self.get_logger().error(
                f"policy blocked: incomplete obs, missing={missing[:5]}"
            )
            self._publish_state()
            return False
        old = self._engine
        if old is not None:
            old.set_enabled(False)  # freeze: nothing consumes/re-plans meanwhile
        try:
            engine = self._new_engine()
            engine.set_enabled(True)
            engine.feed_obs(self._obs_batch(state))
            engine.start()  # blocks until first chunk planned
        except Exception as exc:  # noqa: BLE001
            if old is not None:
                old.set_enabled(False)
            self.controller.revert()
            self.get_logger().error(f"policy start failed: {exc}")
            self._publish_state()
            return False
        # success → acquire ownership, then swap in the new engine
        self._disarm_teleop()
        if old is not None and old is not engine:
            try:
                old.stop()
            except Exception:  # noqa: BLE001
                pass
        self._engine = engine
        self._playback = None
        self._hold_end = None
        self._last_cmds = []
        self._obs_lost_since = None
        self._executor.reset()
        self._seg_prev = None
        self._seg_cur = None
        self._acc = 0.0
        self._sub = 0
        self._publish_state()
        self.get_logger().info("POLICY running")
        return True

    def _cmd_playback(self, text: str) -> None:
        # Load + validate the source *before* touching the FSM or any resource
        # (adversarial-review F4): a load/dim/schema failure leaves the current
        # activity (e.g. a running POLICY engine) fully intact.
        parts = text.split(":")[1:]
        source = str(parts[0]) if parts and parts[0] else self.get_parameter("replay_source").value
        episode = int(parts[1]) if len(parts) > 1 and parts[1] else int(
            self.get_parameter("replay_episode").value
        )
        if not source:
            self.get_logger().error("playback needs replay_source param or playback:<path>")
            return
        try:
            ep = load_replay(source, episode)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"replay load failed: {exc}")
            return
        if ep.action_dim != self._schema.state_dim:
            self.get_logger().error(
                f"replay action_dim {ep.action_dim} != robot state_dim "
                f"{self._schema.state_dim}; schema mismatch"
            )
            return
        if ep.schema is not None:
            recorded = ep.schema.state_names()
            robot = self._schema.state_names()
            if recorded != robot:
                self.get_logger().error(
                    "replay state layout mismatch (same total dim, different "
                    "blocks/order): recorded "
                    f"{recorded} != robot {robot}; refusing to replay"
                )
                return
        ep_fps = int(ep.fps)
        if ep_fps != int(self.get_parameter("dataset_fps").value):
            self.get_logger().warn(
                f"replay fps {ep_fps} != dataset_fps "
                f"{int(self.get_parameter('dataset_fps').value)}; pacing "
                "uses dataset_fps (normally identical at collection time)"
            )
        # validated → commit
        self.controller.request("playback")
        self._disarm_teleop()
        self._teardown_engine()
        self._executor.reset()
        self._playback = PlaybackSession(ep)
        self._hold_end = None
        self._obs_lost_since = None
        self._publish_state()
        self.get_logger().info(
            f"PLAYBACK {source} ep={episode} frames={ep.num_frames} "
            f"fps={ep.fps} dim={ep.action_dim}"
        )

    def _cmd_pause(self) -> None:
        self.controller.request("pause")
        # Freeze planning: the paused branch only re-sends the last target, so a
        # background planner must not keep re-inferring (or drifting the chunk).
        if self._engine is not None:
            try:
                self._engine.set_enabled(False)
            except Exception:  # noqa: BLE001
                pass
        if not self._last_cmds:
            # never commanded yet (e.g. paused right at playback start):
            # freeze the robot at its measured pose so resume has a base
            self._hold_current()
        self._publish_state()
        self.get_logger().info("paused (holding last target)")

    def _cmd_resume(self) -> None:
        prev = self.controller.snapshot()
        self.controller.request("resume")
        activity = self.controller.activity
        if prev["paused"] and activity == "policy":
            # After a pause the robot held its pose while the policy kept going
            # (real-time); the old chunk's remaining rows are stale by the hold
            # time. Re-plan fresh from the live state (reverts to *_PAUSED on
            # failure) to avoid a jump.
            self._start_policy()
            return
        self._disarm_teleop()
        if prev["paused"] and activity == "playback":
            if self._playback is None or self._playback.done:
                self._cmd_stop()
                return
            state, _ = self._state_ok()
            if state is not None:
                self._playback.reanchor(state)
            self._publish_state()
            self.get_logger().info("PLAYBACK resumed")
            return
        if self.controller.state == "POLICY" and self._engine is None:
            # defensive: an interrupted run had torn the engine down
            self._start_policy()
            return
        self._publish_state()
        self.get_logger().info("resumed")

    def _cmd_takeover(self) -> None:
        if self.controller.state == "HUMAN":
            self.get_logger().warn("already in HUMAN takeover")
            return
        if self._takeover_pending is not None:
            self.get_logger().warn("takeover already in progress")
            return
        state, missing = self._state_ok()
        if state is None:
            self.get_logger().error(
                f"takeover blocked: no fresh joint state for re-anchor "
                f"({missing[:5]}); refusing (stale pose would jump)"
            )
            self._publish_state()
            return
        if not self._reanchor_clients:
            self.get_logger().error("no teleop_reanchor_services configured")
            return
        # Freeze policy/playback emission while the effect is in flight so the
        # first teleop node that a re-anchor arms cannot race our writes.
        engine = self._engine
        self._takeover_engine_was_enabled = bool(
            engine is not None and engine.enabled
        )
        if engine is not None:
            engine.set_enabled(False)
        pends: dict[str, object] = {}
        for name, client in self._reanchor_clients.items():
            if not client.service_is_ready():
                self.get_logger().warn(
                    f"reanchor {name} not ready yet; probing with 3s timeout"
                )
            pends[name] = client.call_async(Trigger.Request())
        self._takeover_pending = pends
        self._takeover_deadline = time.monotonic() + 3.0
        self.get_logger().info("takeover: re-anchoring VR teleop…")

    def _poll_takeover(self, now: float) -> bool:
        """Advance the takeover handshake; returns True while still pending
        (caller must skip normal control dispatch)."""
        pends = self._takeover_pending
        if pends is None:
            return False
        pending = [f for f in pends.values() if not f.done()]
        if pending:
            if now > self._takeover_deadline:
                self._abort_takeover(
                    "re-anchor timeout ("
                    f"{', '.join(sorted(pends))})"
                )
                return False
            return True
        for name, fut in pends.items():
            try:
                result = fut.result()
            except Exception as exc:  # noqa: BLE001
                self._abort_takeover(f"re-anchor {name} error: {exc}")
                return False
            if result is None or not result.success:
                msg = (
                    "no response"
                    if result is None
                    else (getattr(result, "message", "") or "rejected")
                )
                self._abort_takeover(f"re-anchor {name} {msg}")
                return False
        self._commit_takeover()
        return False

    def _commit_takeover(self) -> None:
        # Teleop owns arm + gripper streams while the operator is in the loop:
        # open the arbitration gate (False). Arm/head teleop are data-sensitive
        # and only disarm on True, so this does NOT disarm what we just armed.
        self._release_teleop_gate()
        try:
            self.controller.request("takeover")
        except InvalidTransition as exc:
            self._abort_takeover(f"takeover rejected: {exc}")
            return
        self._teardown_engine()
        self._last_cmds = []
        self._obs_lost_since = None
        self._takeover_pending = None
        self._publish_state()
        self.get_logger().warn("HUMAN takeover: VR teleop armed (incremental)")

    def _abort_takeover(self, reason: str) -> None:
        # Transactional rollback (adversarial-review F8): any arm teleop a
        # partial success already armed is re-disarmed (True) so it cannot keep
        # writing joint_commands while the policy still owns them; the frozen
        # engine is re-enabled so the pre-transition activity resumes seamlessly.
        self._takeover_pending = None
        self._disarm_teleop()
        engine = self._engine
        if engine is not None:
            engine.set_enabled(self._takeover_engine_was_enabled)
        self._obs_lost_since = None
        self.get_logger().error(f"takeover failed ({reason}); staying put")
        self._publish_state()

    def _cmd_release(self) -> None:
        self.controller.request("release")
        if self.controller.state == "IDLE":
            # nothing was interrupted: leave manual teleop in charge
            self._teardown_engine()
            self._last_cmds = []
            self._release_teleop_gate()
            self._publish_state()
            return
        activity = self.controller.activity
        paused = bool(self.controller.snapshot()["paused"])
        if activity == "policy":
            if paused:
                # robot keeps whatever pose the human left it at until 'resume'
                self._disarm_teleop()
                self._hold_current()
                self._publish_state()
            else:
                # disarms + installs a fresh engine on success; on failure the
                # FSM reverts to HUMAN and the gate stays open (VR teleop alive)
                self._start_policy()
        elif activity == "playback":
            if self._playback is None or self._playback.done:
                self._teardown_engine()
                self._last_cmds = []
                self._release_teleop_gate()
                self.get_logger().warn("release: playback finished")
                self._publish_state()
                return
            # policy owns the streams again after HUMAN
            self._disarm_teleop()
            state, _ = self._state_ok()
            if state is not None:
                self._playback.reanchor(state)
            if paused:
                self._hold_current()
            self._publish_state()
            self.get_logger().info("PLAYBACK resumed (offset re-anchored)")
        else:
            self._teardown_engine()
            self._release_teleop_gate()
            self._publish_state()

    def _hold_current(self) -> None:
        """Hold the robot at its measured pose (used entering a paused state)."""
        state, missing = self._state_ok()
        if state is None:
            self.get_logger().warn(f"hold_current: no state ({missing[:3]})")
            return
        self._executor.reset()
        self._send_targets_from(state)

    def _cmd_stop(self) -> None:
        if self._takeover_pending is not None:
            self._takeover_pending = None
            self.get_logger().warn("stop during takeover effect; effect aborted")
        self.controller.request("stop")
        self._teardown_engine()
        self._playback = None
        self._last_cmds = []
        self._hold_end = None
        self._obs_lost_since = None
        # Open the arbitration gate: teleop/pinch may drive again (latched True
        # would otherwise leave the gripper gate shut for the node's whole life).
        self._release_teleop_gate()
        self._publish_state()
        self.get_logger().info("stopped -> IDLE")

    # ---------------------------------------------------------- control loop

    def _tick(self) -> None:
        with self._lock:
            now = time.monotonic()
            # Event-driven takeover handshake first: while the re-anchor effect is
            # in flight we neither drain commands (except stop, handled in
            # _drain_cmds) nor emit control targets — single-writer guarantee.
            if self._takeover_pending is not None:
                if self._poll_takeover(now):
                    return
            self._drain_cmds()
            now = time.monotonic()
            dt = 1.0 / max(1.0, self._ctrl_rate)
            st = self.controller.state
            if st == "POLICY":
                if self._engine is None:
                    self._maybe_publish_state(now)
                    return
                if not self.controller.snapshot()["paused"]:
                    self._policy_tick(dt)
            elif st == "POLICY_PAUSED":
                self._resend_last()
            elif st == "PLAYBACK":
                self._playback_tick(dt)
            elif st == "PLAYBACK_PAUSED":
                self._resend_last()
            elif st == "HUMAN":
                pass  # teleop owns the command topics now
            # IDLE: silent
            self._maybe_publish_state(now)

    def _policy_tick(self, dt: float) -> None:
        self._acc += dt
        t_loop0: Optional[float] = None
        if self._acc + 1e-9 >= self._policy_dt:
            self._acc = max(0.0, self._acc - self._policy_dt)
            t_loop0 = time.monotonic()
            state, _ = self._state_ok()
            if state is None:
                # Mid-run observation loss gate: never keep re-inferring (or
                # emitting) from a stale joint-state snapshot. After a grace
                # period, auto-pause so the robot holds instead of running a
                # blind plan (adversarial-review F6).
                now = time.monotonic()
                if self._obs_lost_since is None:
                    self._obs_lost_since = now
                    return
                if now - self._obs_lost_since > self._obs_stale_stop_s:
                    self.get_logger().error(
                        f"joint state lost > {self._obs_stale_stop_s:.1f}s — "
                        "auto-pausing POLICY (holding last target)"
                    )
                    self._cmd_pause()
                return
            self._obs_lost_since = None
            try:
                self._engine.feed_obs(self._obs_batch(state))
            except Exception:  # noqa: BLE001
                pass
            try:
                row = self._engine.tick()
            except EngineStateError as exc:
                self.get_logger().error(f"engine failed: {exc}")
                self._cmd_stop()
                return
            if row is None:
                self._resend_last()
                return
            if self._seg_cur is not None:
                self._seg_prev = self._seg_cur
            self._seg_cur = np.asarray(row, dtype=np.float64)
            self._sub = 0
        else:
            self._sub += 1
        self._emit_target()
        # 端到端控制延迟（新观测 → 指令发布）与观测龄期；只测「消费了新观测」的 tick
        if t_loop0 is not None:
            now = time.monotonic()
            self._loop_ms.append((now - t_loop0) * 1000.0)
            # 关节反馈才是真观测（ratio 是自回显，发布时刷新 stamp，会污染龄期）
            joint_keys = [
                k for k, src in self._layout.sources.items()
                if src.kind != "ratio" and k in self._stamps
            ]
            if joint_keys:
                self._obs_age_ms.append(
                    (now - max(self._stamps[k] for k in joint_keys)) * 1000.0
                )

    def _emit_target(self) -> None:
        """Publish one control-rate target, linearly sub-dividing frame steps."""
        if self._seg_cur is None:
            return
        if self._interp <= 1 or self._seg_prev is None:
            self._send_targets_from(self._seg_cur)
            return
        f = min(1.0, (self._sub + 1) / float(self._interp))
        tgt = self._seg_prev * (1.0 - f) + self._seg_cur * f
        self._send_targets_from(tgt)

    def _playback_tick(self, dt: float) -> None:
        self._acc += dt
        if self._acc + 1e-9 < self._policy_dt:
            return
        self._acc = max(0.0, self._acc - self._policy_dt)
        if self._playback is None:
            self._cmd_stop()
            return
        row = self._playback.target()
        if row is None:
            if not self._last_cmds:
                self._cmd_stop()
                return
            # hold end pose then auto stop
            if self._hold_end is None:
                self._hold_end = time.monotonic()
            if time.monotonic() - self._hold_end > float(
                self.get_parameter("replay_hold_s").value
            ):
                self._cmd_stop()
                return
            self._resend_last()
            return
        self._playback.advance()
        self._send_targets_from(row)

    # -------------------------------------------------------- command sending

    def _send_targets_from(self, row: np.ndarray) -> None:
        try:
            targets = self._layout.split_action(row)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"split_action failed: {exc}")
            return
        safe = self._executor.step(targets, 1.0 / max(1.0, self._ctrl_rate))
        self._exec_events += [f"{e.kind}:{e.stream}" for e in self._executor.events]
        self._publish_targets(safe)
        self._last_cmds = safe

    def _resend_last(self) -> None:
        if self._last_cmds:
            self._publish_targets(self._last_cmds)

    def _publish_targets(self, targets: list) -> None:
        for t in targets:
            if t.kind == "ratio":
                pub = self._pubs.get(("ratio", t.topic))
                if pub is None:
                    pub = self.create_publisher(Float64, t.topic, _SENSOR_QOS)
                    self._pubs[("ratio", t.topic)] = pub
                msg = Float64()
                msg.data = float(t.values[0])
                pub.publish(msg)
                self._ratio_last[t.topic] = (msg.data, time.monotonic())
                key = self._topic_to_key.get(t.topic)
                if key is not None:
                    self._values[key] = np.asarray([msg.data], dtype=np.float64)
                    self._stamps[key] = time.monotonic()
            else:
                pub = self._pubs.get(("js", t.topic))
                if pub is None:
                    pub = self.create_publisher(JointState, t.topic, _SENSOR_QOS)
                    self._pubs[("js", t.topic)] = pub
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.position = [float(v) for v in t.values]
                pub.publish(msg)

    # ---------------------------------------------------------------- state

    @staticmethod
    def _deque_stats(dq: "collections.deque[float]") -> dict | None:
        if not dq:
            return None
        xs = sorted(dq)
        n = len(xs)
        return {
            "avg": round(sum(xs) / n, 2),
            "p50": round(xs[n // 2], 2),
            "p95": round(xs[min(n - 1, int(n * 0.95))], 2),
            "max": round(xs[-1], 2),
        }

    def _state_payload(self) -> dict:
        with self._lock:
            return self._state_payload_locked()

    def _state_payload_locked(self) -> dict:
        engine_stats = self._engine.stats if self._engine is not None else {}
        return {
            "state": self.controller.state,
            "activity": self.controller.activity,
            "paused": self.controller.snapshot()["paused"],
            "prompt": self._prompt,
            "state_dim": self._schema.state_dim,
            "engine": engine_stats,
            "playback": (
                {
                    "idx": self._playback.idx,
                    "frames": self._playback.episode.num_frames,
                    "remaining": self._playback.remaining,
                }
                if self._playback is not None
                else None
            ),
            "cam_frames": self._cam_frames,
            "latency_ms": {
                "loop": self._deque_stats(self._loop_ms),
                "obs_age": self._deque_stats(self._obs_age_ms),
            },
            "exec_events": self._exec_events[-20:],
            "error": getattr(self, "_last_error", None),
        }

    def _publish_state(self) -> None:
        msg = String()
        msg.data = json.dumps(self._state_payload(), ensure_ascii=False)
        self._state_pub.publish(msg)

    def _maybe_publish_state(self, now: float) -> None:
        if now - self._last_publish > 0.5:
            self._last_publish = now
            self._publish_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor(num_threads=4))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
