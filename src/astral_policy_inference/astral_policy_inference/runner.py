"""Non-ROS policy runner — full astral_policy_inference functionality, no rclpy.

Reuses the pure-Python core (backend / engine / executor / controller / replay /
robot_io) and replaces the ROS orchestration of ``node.py`` (timers / topics /
callbacks / disarm gate) with a plain-thread control loop and pluggable I/O:

  * **obs in**:  ``feed_observation(state, images, prompt)`` — 你随时喂最新观测
    （state 为绝对关节向量；images 为 {label: HxWx3 uint8}）；
  * **action out**: ``on_action(row)`` 回调——控制循环把安全层通过后的**绝对动作行**
    (state_dim,) 交给你下发（真机/仿真/记录）；
  * **命令**: ``request("policy"|"playback"|"pause"|"resume"|"takeover"|"release"|"stop")``；
  * **控制权仲裁**: ``on_acquire_control()``/``on_release_control()`` 回调，镜像 ROS 版
    ``/teleop/disarm`` 的电平门（默认 no-op，外部遥操栈可接）。

覆盖推理包全部功能：三种引擎节奏（queue_sync/async/rtc）、绝对动作、绝对语义守卫、
SafeExecutor 安全层、FSM（IDLE/POLICY(_PAUSED)/PLAYBACK(_PAUSED)/HUMAN）、真机回放
（h5/LeRobot v2.1）、暂停恢复重规划、HITL 接管交还、端到端延迟/引擎统计指标。
"""

from __future__ import annotations

import collections
import json
import queue
import threading
import time
from typing import Callable, Optional

import numpy as np

from astral_data_collect.schema import CollectSchema

from astral_policy_inference.backend import ObsBatch, make_backend
from astral_policy_inference.controller import Controller, InvalidTransition
from astral_policy_inference.engine import ActionEngine, EngineStateError
from astral_policy_inference.executor import SafeExecutor
from astral_policy_inference.replay import PlaybackSession, load_replay
from astral_policy_inference.robot_io import ObsLayout


class PolicyRunner:
    """Non-ROS policy execution / replay / pause / HITL, driven by one thread.

    Parameters mirror ``config/policy_inference.yaml`` (backend / engine / replay /
    safety). ``feed_observation`` supplies fresh obs at your own rate; the control
    loop runs at ``dataset_fps * control_interp``; ``on_action`` receives each
    safety-passed absolute action row.
    """

    def __init__(
        self,
        *,
        backend_cfg: dict,
        schema: CollectSchema,
        engine_mode: str = "queue_async",
        action_chunk: int = 50,
        control_interp: int = 1,
        abs_action_min_scale: float = 0.5,
        # replay
        replay_source: str = "",
        replay_episode: int = 0,
        replay_hold_s: float = 1.0,
        # safety / freshness
        max_joint_vel: float = 6.0,
        obs_timeout_s: float = 0.5,
        obs_stale_stop_s: float = 1.0,
        # callbacks（默认 no-op）
        on_action: Optional[Callable[[np.ndarray], None]] = None,
        on_state: Optional[Callable[[dict], None]] = None,
        on_acquire_control: Optional[Callable[[], None]] = None,
        on_release_control: Optional[Callable[[], None]] = None,
    ) -> None:
        self.schema = schema
        self.layout = ObsLayout(schema)
        self.state_dim = schema.state_dim
        self._backend_cfg = dict(backend_cfg)
        self._backend_cfg["action_dim"] = schema.state_dim
        self._engine_mode = engine_mode
        self._action_chunk = int(action_chunk)
        self._ctrl_rate = float(schema.dataset_fps) * int(control_interp)
        self._policy_dt = 1.0 / float(schema.dataset_fps)
        self._control_interp = int(control_interp)
        self._replay_source = replay_source
        self._replay_episode = int(replay_episode)
        self._replay_hold_s = float(replay_hold_s)
        self._obs_timeout = float(obs_timeout_s)
        self._obs_stale_stop_s = float(obs_stale_stop_s)
        self._abs_action_min_scale = float(abs_action_min_scale)

        self.controller = Controller()
        self._executor = SafeExecutor(max_joint_vel=float(max_joint_vel))
        self._lock = threading.RLock()
        self._cmd_queue: "queue.Queue[str]" = queue.Queue()
        self._engine: Optional[ActionEngine] = None
        self._playback: Optional[PlaybackSession] = None
        self._last_cmds: list = []
        self._last_row: Optional[np.ndarray] = None  # 最近一次完整绝对动作行（on_action 契约）
        self._seg_prev: Optional[np.ndarray] = None
        self._seg_cur: Optional[np.ndarray] = None
        self._sub = 0
        self._acc = 0.0
        self._hold_end: Optional[float] = None
        self._obs_lost_since: Optional[float] = None

        # 观测缓冲（feed_observation 写入，控制线程消费）
        self._obs_state: Optional[np.ndarray] = None
        self._obs_stamp = 0.0
        self._images: dict[str, np.ndarray] = {}
        self._image_stamps: dict[str, float] = {}
        self._prompt = ""

        # 指标
        self._loop_ms: "collections.deque[float]" = collections.deque(maxlen=50)
        self._obs_age_ms: "collections.deque[float]" = collections.deque(maxlen=50)
        self._exec_events: list[str] = []

        # 回调
        self.on_action = on_action or (lambda _row: None)
        self.on_state = on_state or (lambda _d: None)
        self._on_acquire = on_acquire_control or (lambda: None)
        self._on_release = on_release_control or (lambda: None)

        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

    # ------------------------------------------------------------- lifecycle

    def start(self) -> None:
        """启动控制线程（无观测时 idle，进 POLICY 需先 feed_observation）。"""
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, name="policy-runner",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        with self._lock:
            self._teardown_engine()
            self._playback = None

    # --------------------------------------------------------- obs ingress

    def feed_observation(
        self,
        *,
        state: np.ndarray,
        images: dict[str, np.ndarray] | None = None,
        prompt: str = "",
    ) -> None:
        """喂最新观测（非阻塞，任意频率调用）。

        state: (state_dim,) 绝对关节向量（schema 布局序）；
        images: {label: HxWx3 uint8 RGB}（label 与 schema.cameras 对应）；
        prompt: 语言指令（缺省用 backend default_prompt）。
        """
        s = np.asarray(state, dtype=np.float64).reshape(-1)
        if s.shape[0] != self.state_dim:
            raise ValueError(f"state dim {s.shape[0]} != schema {self.state_dim}")
        with self._lock:
            self._obs_state = s
            self._obs_stamp = time.monotonic()
            if images:
                self._images = {
                    k: np.asarray(v, dtype=np.uint8) for k, v in images.items()
                }
                self._image_stamps = {
                    k: time.monotonic() for k in self._images
                }
            if prompt:
                self._prompt = prompt

    # -------------------------------------------------------------- command

    def request(self, verb: str) -> None:
        """命令入口：policy / playback[:src[:ep]] / pause / resume / takeover /
        release / stop。与节点一样经队列由控制线程串行消费。"""
        verb = verb.strip()
        if verb:
            self._cmd_queue.put_nowait(verb)

    # ------------------------------------------------------------- metrics

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

    def stats(self) -> dict:
        with self._lock:
            engine_stats = self._engine.stats if self._engine is not None else {}
            return {
                "state": self.controller.state,
                "activity": self.controller.activity,
                "paused": self.controller.snapshot()["paused"],
                "prompt": self._prompt,
                "state_dim": self.state_dim,
                "engine": engine_stats,
                "playback": (
                    {"idx": self._playback.idx,
                     "frames": self._playback.episode.num_frames,
                     "remaining": self._playback.remaining}
                    if self._playback is not None else None
                ),
                "latency_ms": {
                    "loop": self._deque_stats(self._loop_ms),
                    "obs_age": self._deque_stats(self._obs_age_ms),
                },
                "exec_events": self._exec_events[-20:],
            }

    # -------------------------------------------------------- control loop

    def _loop(self) -> None:
        dt = 1.0 / max(1.0, self._ctrl_rate)
        next_tick = time.monotonic()
        while not self._stop_evt.is_set():
            self._tick(dt)
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def _tick(self, dt: float) -> None:
        with self._lock:
            self._drain_cmds()
            st = self.controller.state
            if st == "POLICY":
                if self._engine is not None and not self.controller.snapshot()["paused"]:
                    self._policy_tick(dt)
                elif self._engine is None:
                    self._cmd_stop()
            elif st == "POLICY_PAUSED":
                self._resend_last()
            elif st == "PLAYBACK":
                self._playback_tick(dt)
            elif st == "PLAYBACK_PAUSED":
                self._resend_last()
            elif st == "HUMAN":
                pass  # 外部遥操接管，本 runner 静默
            self._maybe_publish_state()

    def _drain_cmds(self) -> None:
        while True:
            try:
                text = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._exec_cmd(text)
            except Exception as exc:  # noqa: BLE001
                self._exec_events.append(f"cmd_failed:{exc}")

    def _exec_cmd(self, text: str) -> None:
        verb = text.split(":", 1)[0].split()[0]
        try:
            if verb == "policy":
                self.controller.request("policy")
                self._start_policy()
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
                self._exec_events.append(f"unknown_cmd:{verb}")
        except InvalidTransition as exc:
            self._exec_events.append(f"rejected:{exc}")

    # ------------------------------------------------------- observation

    def _state_ok(self) -> tuple[Optional[np.ndarray], list[str]]:
        now = time.monotonic()
        missing: list[str] = []
        if self._obs_state is None or now - self._obs_stamp > self._obs_timeout:
            missing.append("state")
            return None, missing
        stale_images = [
            lab for lab in self._images
            if now - self._image_stamps.get(lab, 0.0) > self._obs_timeout * 4
        ]
        return self._obs_state, missing + [f"image_stale:{m}" for m in stale_images]

    def _obs_batch(self, state: np.ndarray) -> ObsBatch:
        now = time.monotonic()
        images = {
            lab: arr
            for lab, arr in self._images.items()
            if now - self._image_stamps.get(lab, 0.0) <= self._obs_timeout * 4
        }
        return ObsBatch(state=state, images=images, prompt=self._prompt)

    # ----------------------------------------------------------- engines

    def _new_engine(self) -> ActionEngine:
        return ActionEngine(
            make_backend(**self._backend_cfg),
            mode=self._engine_mode,
            action_dim=self.state_dim,
            chunk=self._action_chunk,
            policy_fps=int(self.schema.dataset_fps),
            autostart=True,
            abs_action_min_scale=self._abs_action_min_scale,
        )

    def _stop_engine(self) -> None:
        if self._engine is not None:
            try:
                self._engine.set_enabled(False)
                self._engine.reset()
            except Exception:  # noqa: BLE001
                pass
        self._seg_prev = self._seg_cur = None
        self._acc = 0.0
        self._sub = 0

    def _teardown_engine(self) -> None:
        self._stop_engine()
        if self._engine is not None:
            try:
                self._engine.stop()
            except Exception:  # noqa: BLE001
                pass
            self._engine = None

    def _start_policy(self) -> bool:
        """进/重启 POLICY：先验新鲜 obs → 建新引擎（旧引擎仅冻结）→ 成功才
        acquire 控制权并退役旧引擎；失败 revert FSM，保持原状态。"""
        state, missing = self._state_ok()
        if state is None:
            self.controller.revert()
            self._exec_events.append(f"policy_blocked:{missing[:3]}")
            return False
        old = self._engine
        if old is not None:
            old.set_enabled(False)
        try:
            engine = self._new_engine()
            engine.set_enabled(True)
            engine.feed_obs(self._obs_batch(state))
            engine.start()
        except Exception as exc:  # noqa: BLE001
            if old is not None:
                old.set_enabled(False)
            self.controller.revert()
            self._exec_events.append(f"policy_start_failed:{exc}")
            return False
        self._on_acquire()  # 镜像 disarm=true：策略持有指令流
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
        self._seg_prev = self._seg_cur = None
        self._acc = 0.0
        self._sub = 0
        return True

    def _cmd_playback(self, text: str) -> None:
        parts = text.split(":")[1:]
        source = str(parts[0]) if parts and parts[0] else self._replay_source
        episode = int(parts[1]) if len(parts) > 1 and parts[1] else self._replay_episode
        if not source:
            self._exec_events.append("playback_no_source")
            return
        try:
            ep = load_replay(source, episode)
        except Exception as exc:  # noqa: BLE001
            self._exec_events.append(f"replay_load_failed:{exc}")
            return
        if ep.action_dim != self.state_dim:
            self._exec_events.append(f"replay_dim_mismatch:{ep.action_dim}!={self.state_dim}")
            return
        if ep.schema is not None and ep.schema.state_names() != self.schema.state_names():
            self._exec_events.append("replay_layout_mismatch")
            return
        self.controller.request("playback")
        self._on_acquire()
        self._teardown_engine()
        self._executor.reset()
        self._playback = PlaybackSession(ep)
        self._hold_end = None
        self._obs_lost_since = None

    def _cmd_pause(self) -> None:
        self.controller.request("pause")
        if self._engine is not None:
            try:
                self._engine.set_enabled(False)
            except Exception:  # noqa: BLE001
                pass
        if not self._last_cmds:
            self._hold_current()

    def _cmd_resume(self) -> None:
        prev = self.controller.snapshot()
        self.controller.request("resume")
        activity = self.controller.activity
        if prev["paused"] and activity == "policy":
            self._start_policy()  # 暂停期 chunk 过期 → 按实况重规划
            return
        self._on_acquire()
        if prev["paused"] and activity == "playback":
            if self._playback is None or self._playback.done:
                self._cmd_stop()
                return
            state, _ = self._state_ok()
            if state is not None:
                self._playback.reanchor(state)

    def _cmd_takeover(self) -> None:
        """HITL 接管：FSM→HUMAN + 释放控制权（外部遥操栈驱动）。"""
        if self.controller.state == "HUMAN":
            return
        self.controller.request("takeover")
        self._teardown_engine()
        self._last_cmds = []
        self._obs_lost_since = None
        self._on_release()  # 镜像 disarm=false：放行外部遥操

    def _cmd_release(self) -> None:
        """交还：回策略（按实况重规划）或回放（按实况重锚）。"""
        self.controller.request("release")
        if self.controller.state == "IDLE":
            self._teardown_engine()
            self._last_cmds = []
            self._on_release()
            return
        activity = self.controller.activity
        if activity == "policy":
            if self.controller.snapshot()["paused"]:
                self._on_acquire()
                self._hold_current()
            else:
                self._start_policy()
        elif activity == "playback":
            if self._playback is None or self._playback.done:
                self._cmd_stop()
                return
            self._on_acquire()
            state, _ = self._state_ok()
            if state is not None:
                self._playback.reanchor(state)

    def _cmd_stop(self) -> None:
        try:
            self.controller.request("stop")
        except InvalidTransition:
            pass
        self._teardown_engine()
        self._playback = None
        self._last_cmds = []
        self._hold_end = None
        self._obs_lost_since = None
        self._on_release()

    def _hold_current(self) -> None:
        state, missing = self._state_ok()
        if state is None:
            self._exec_events.append(f"hold_no_state:{missing[:2]}")
            return
        self._executor.reset()
        self._send_row(state)

    # ----------------------------------------------------------- tick 逻辑

    def _policy_tick(self, dt: float) -> None:
        self._acc += dt
        t_loop0: Optional[float] = None
        if self._acc + 1e-9 >= self._policy_dt:
            self._acc = max(0.0, self._acc - self._policy_dt)
            t_loop0 = time.monotonic()
            state, _ = self._state_ok()
            if state is None:
                now = time.monotonic()
                if self._obs_lost_since is None:
                    self._obs_lost_since = now
                    return
                if now - self._obs_lost_since > self._obs_stale_stop_s:
                    self._exec_events.append("obs_lost_autopause")
                    self._cmd_pause()
                return
            self._obs_lost_since = None
            try:
                self._engine.feed_obs(self._obs_batch(state))
                row = self._engine.tick()
            except EngineStateError as exc:
                self._exec_events.append(f"engine_error:{exc}")
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
        if t_loop0 is not None:
            now = time.monotonic()
            self._loop_ms.append((now - t_loop0) * 1000.0)
            if self._obs_state is not None:
                self._obs_age_ms.append((now - self._obs_stamp) * 1000.0)

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
            if self._hold_end is None:
                self._hold_end = time.monotonic()
            if time.monotonic() - self._hold_end > self._replay_hold_s:
                self._cmd_stop()
                return
            self._resend_last()
            return
        self._playback.advance()
        self._send_row(row)

    def _emit_target(self) -> None:
        if self._seg_cur is None:
            return
        if self._control_interp <= 1 or self._seg_prev is None:
            self._send_row(self._seg_cur)
            return
        f = min(1.0, (self._sub + 1) / float(self._control_interp))
        self._send_row(self._seg_prev * (1.0 - f) + self._seg_cur * f)

    # --------------------------------------------------------- action 下发

    def _send_row(self, row: np.ndarray) -> None:
        """安全层 + 拆分 + on_action 回调（完整**安全处理后的**绝对动作行）。

        按 schema 块序把安全层输出的各流目标拼回 state_dim 行 —— 消费者（真机驱动）
        拿到的是 clip/slew 后的实际下发值，而非原始行。
        """
        row_f = np.asarray(row, dtype=np.float64).reshape(-1)
        try:
            targets = self.layout.split_action(row_f)
        except Exception as exc:  # noqa: BLE001
            self._exec_events.append(f"split_failed:{exc}")
            return
        safe = self._executor.step(targets, 1.0 / max(1.0, self._ctrl_rate))
        self._exec_events += [f"{e.kind}:{e.stream}" for e in self._executor.events]
        self._last_cmds = safe
        # 安全流按 schema 块序拼回（split_action 与 step 都保持块序）
        safe_row = np.concatenate(
            [np.asarray(t.values, dtype=np.float64).reshape(-1) for t in safe]
        )
        self._last_row = safe_row
        self.on_action(safe_row)

    def _resend_last(self) -> None:
        # 暂停/空窗时保持：重发最近一次安全处理后的完整动作行
        if self._last_row is not None:
            self.on_action(self._last_row)

    # --------------------------------------------------------------- state

    def _maybe_publish_state(self) -> None:
        self.on_state(self.stats())


__all__ = ["PolicyRunner"]
