"""Action-chunk execution engines: queue_sync / queue_async / rtc.

The engine turns a backend's *chunks* into a stream of single absolute action
rows at the policy rate. pi0.5/ACT outputs are absolute joint targets, so a
re-plan can replace the unexecuted tail of the current chunk and simply resume
at the latency-compensated index — no blending needed, no jumps.

Modes:
* ``queue_sync``  — refill blocks the calling (control) thread at chunk end.
* ``queue_async`` — background planner pre-fetches the next chunk once the
  current one is within ``async_prefetch_ahead`` of empty; the control thread
  never blocks after the initial plan.
* ``rtc``         — rolling re-plan: planner re-infers every
  ``rtc_replan_interval_s`` and swaps the tail once ≥ ``rtc_min_tail`` rows of
  the current chunk have executed.

No ROS state here; backend is injected, so the engine is unit-testable with a
fake backend. Threading is optional (``autostart=False``) for deterministic
tests.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

from astral_policy_inference.backend import ObsBatch, PolicyBackend, PolicyError


class EngineStateError(RuntimeError):
    """Engine has no usable chunk (planner failed / no observation yet)."""


class ActionEngine:
    def __init__(
        self,
        backend: PolicyBackend,
        *,
        mode: str = "queue_async",
        action_dim: int,
        chunk: int = 50,
        policy_fps: int = 30,
        async_prefetch_ahead: Optional[int] = None,
        rtc_replan_interval_s: Optional[float] = None,
        rtc_min_tail: Optional[int] = None,
        queue_min_replan_interval_s: float = 0.05,
        autostart: bool = True,
    ):
        if mode not in ("queue_sync", "queue_async", "rtc"):
            raise ValueError(f"mode={mode!r} not in queue_sync|queue_async|rtc")
        self.backend = backend
        self.mode = mode
        self.action_dim = int(action_dim)
        self.chunk = int(chunk)
        self.policy_fps = int(policy_fps)
        self.async_prefetch_ahead = (
            int(async_prefetch_ahead)
            if async_prefetch_ahead is not None
            else max(1, self.chunk // 2)
        )
        self.rtc_replan_interval_s = (
            float(rtc_replan_interval_s)
            if rtc_replan_interval_s is not None
            else min(1.0, self.chunk / (2.0 * self.policy_fps))
        )
        self.rtc_min_tail = (
            int(rtc_min_tail) if rtc_min_tail is not None else max(1, self.chunk // 5)
        )
        self.queue_min_replan_interval_s = float(queue_min_replan_interval_s)
        self._autostart = autostart

        self._lock = threading.RLock()
        self._chunk: Optional[np.ndarray] = None
        self._i = 0                       # next row to emit from current chunk
        self._pops_since_install = 0
        self._obs: Optional[ObsBatch] = None
        self._planning = False
        self._last_plan_ms = 0.0
        self._last_accept_t = 0.0
        self._plans = 0
        self._pops = 0
        self._last_error: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.enabled = False

    # ------------------------------------------------------------------ obs

    def feed_obs(self, obs: ObsBatch) -> None:
        with self._lock:
            self._obs = ObsBatch(
                state=np.array(obs.state, copy=True),
                images={
                    k: np.array(v, copy=True) for k, v in (obs.images or {}).items()
                },
                prompt=str(obs.prompt or ""),
            )

    @property
    def has_obs(self) -> bool:
        with self._lock:
            return self._obs is not None

    # ------------------------------------------------------------- lifecycle

    def start(self) -> None:
        """Open backend; block until an initial chunk exists."""
        self.backend.open()
        self.enabled = True
        if self._chunk is None:
            self._plan_blocking()  # synchronous first plan → POLICY can move at once
        if self._chunk is None:
            err = self.last_error or "policy returned no chunk"
            raise EngineStateError(err)
        if self._autostart and self.mode != "queue_sync" and self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._planner_loop, name="policy-planner", daemon=True
            )
            self._thread.start()

    def set_enabled(self, enabled: bool) -> None:
        """Gate planning/ticking while the node is not in POLICY."""
        self.enabled = bool(enabled)

    def stop(self) -> None:
        self.enabled = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        # close under the lock so a queue_sync refill that is mid-infer (control
        # thread, holding the lock during _plan_blocking) can never race it
        try:
            with self._lock:
                self.backend.close()
        except Exception:  # noqa: BLE001
            pass

    def reset(self) -> None:
        """Drop queued chunk + policy-side context (used on human takeover)."""
        with self._lock:
            self._chunk = None
            self._i = 0
            self._pops_since_install = 0
            self._last_error = None
        try:
            self.backend.reset()
        except Exception:  # noqa: BLE001
            pass

    # --------------------------------------------------------------- ticks

    def tick(self) -> Optional[np.ndarray]:
        """One absolute action row for the current policy tick, or None."""
        if not self.enabled:
            return None
        while True:
            with self._lock:
                if self._chunk is not None and self._i < len(self._chunk):
                    row = self._chunk[self._i].copy()
                    self._i += 1
                    self._pops += 1
                    self._pops_since_install += 1
                    return row
                if self._chunk is not None and self._last_error is not None:
                    err = self._last_error
                    self._last_error = None
                    raise EngineStateError(err)
            if self.mode == "queue_sync":
                if not self.has_obs:
                    return None
                if self._chunk is None and self._last_error is not None:
                    err = self._last_error
                    self._last_error = None
                    raise EngineStateError(err)
                self._plan_blocking()
                continue
            # async/rtc: the planner thread is responsible for refills. If it
            # already failed hard, surface that instead of holding forever.
            err = self.last_error
            if err:
                raise EngineStateError(err)
            return None

    @property
    def need_refill(self) -> bool:
        with self._lock:
            return self._chunk is None or self._i >= len(self._chunk)

    @property
    def remaining(self) -> int:
        with self._lock:
            if self._chunk is None:
                return 0
            return max(0, len(self._chunk) - self._i)

    @property
    def last_error(self) -> Optional[str]:
        with self._lock:
            return self._last_error

    def clear_error(self) -> None:
        with self._lock:
            self._last_error = None

    @property
    def stats(self) -> dict:
        with self._lock:
            return {
                "mode": self.mode,
                "chunk": self.chunk,
                "plans": self._plans,
                "pops": self._pops,
                "last_plan_ms": self._last_plan_ms,
                "remaining": self.remaining,
                "error": self._last_error,
            }

    # -------------------------------------------------------------- planning

    def _snapshot_obs(self) -> ObsBatch:
        with self._lock:
            return ObsBatch(
                state=np.array(self._obs.state, copy=True),
                images={
                    k: np.array(v, copy=True)
                    for k, v in (self._obs.images or {}).items()
                },
                prompt=self._obs.prompt,
            )

    def _plan_blocking(self) -> None:
        """Blocking plan (first chunk / queue_sync refill).

        Holds the engine lock across inference so ``stop()``/teardown that
        close the backend on another thread cannot race a running infer.
        """
        with self._lock:
            if self._obs is None:
                return
            self._planning = True
            try:
                self._run_plan()
            finally:
                self._planning = False

    def _run_plan(self) -> bool:
        """Infer one chunk and (re)install it. Returns success.

        Holds the engine lock across inference: queue_sync plans run on the
        control thread and planner-thread plans on the planner thread, so this
        serializes them and guarantees ``stop()``/``backend.close()`` (also
        lock-guarded) can never race an in-flight ``infer()``.
        """
        with self._lock:
            obs = self._snapshot_obs()
            t0 = time.perf_counter()
            try:
                full = np.asarray(self.backend.infer(obs), dtype=np.float64)
            except PolicyError as exc:
                self._last_error = str(exc)
                return False
            except Exception as exc:  # noqa: BLE001
                self._last_error = f"infer failed: {exc}"
                return False
            ms = (time.perf_counter() - t0) * 1000.0
            self._install(full, latency_ms=ms)
            return True

    def _install(self, full: np.ndarray, latency_ms: float) -> None:
        """Swap the chunk and resume at the latency-compensated index."""
        full = np.asarray(full, dtype=np.float64)
        if full.ndim == 1:
            full = full.reshape(1, -1)
        if full.shape[1] != self.action_dim:
            self._last_error = (
                f"chunk action_dim {full.shape[1]} != {self.action_dim}"
            )
            return
        rows = min(self.chunk, len(full))
        tail = full[:rows]
        # Latency compensation only matters when we replace a chunk the robot is
        # *still executing* (it advanced since the fresh plan's observation). If
        # the robot is holding (first plan / refill at boundary) start at row 0.
        was_moving = self._chunk is not None and self._i < len(self._chunk)
        if self.mode != "queue_sync" and was_moving:
            latency_rows = int(round(latency_ms / 1000.0 * self.policy_fps))
            i0 = int(np.clip(latency_rows, 0, max(0, rows - 1)))
        else:
            i0 = 0
        self._chunk = tail
        self._i = i0
        self._pops_since_install = 0
        self._plans += 1
        self._last_plan_ms = latency_ms
        self._last_error = None
        self._last_accept_t = time.time()

    # ------------------------------------------------------- planner thread

    def _planner_loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                if not self.enabled or self._obs is None or self._planning:
                    idle = True
                elif self._chunk is None or self._i >= len(self._chunk):
                    idle = False          # boundary: urgent refill
                elif self.mode == "queue_async":
                    since = time.time() - self._last_accept_t
                    idle = (
                        self.remaining > self.async_prefetch_ahead
                        or since < self.queue_min_replan_interval_s
                    )
                elif self.mode == "rtc":
                    due = (time.time() - self._last_accept_t) >= self.rtc_replan_interval_s
                    idle = not (due and self._pops_since_install >= self.rtc_min_tail)
                else:
                    idle = True
                if idle:
                    self._stop.wait(0.005)
                    continue
                self._planning = True
            try:
                self._run_plan()
            finally:
                with self._lock:
                    self._planning = False
