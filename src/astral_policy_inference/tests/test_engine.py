#!/usr/bin/env python3
"""Tests for the three action engines (threading included)."""

import threading
import time
import unittest

import numpy as np

from astral_policy_inference.backend import ObsBatch, PolicyError, StubBackend
from astral_policy_inference.engine import ActionEngine, EngineStateError

DIM = 8


def make_engine(mode="queue_sync", **kw):
    backend = StubBackend(action_dim=DIM, camera_map={})
    kwargs = dict(mode=mode, action_dim=DIM, chunk=kw.pop("chunk", 10),
                  autostart=False)
    kwargs.update(kw)
    return ActionEngine(backend, **kwargs), backend


class FailingBackend(StubBackend):
    def infer(self, obs):
        raise PolicyError("server down")


class TestQueueSync(unittest.TestCase):
    def test_ticks_consume_and_refill(self):
        eng, _ = make_engine("queue_sync", chunk=3)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        self.assertEqual(eng.stats["plans"], 1)
        rows = [eng.tick() for _ in range(9)]
        self.assertTrue(all(r is not None for r in rows))
        self.assertGreaterEqual(eng.stats["plans"], 3)

    def test_failed_infer_raises_engine_state_error(self):
        eng = ActionEngine(FailingBackend(action_dim=DIM, camera_map={}),
                           mode="queue_sync", action_dim=DIM, chunk=5,
                           autostart=False)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        with self.assertRaises(EngineStateError):
            eng.start()

    def test_disabled_returns_none(self):
        eng, _ = make_engine("queue_sync")
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(False)
        self.assertIsNone(eng.tick())


class TestQueueAsync(unittest.TestCase):
    def test_queue_async_refill_logic(self):
        # Deterministic (no planner thread): exercise exactly what the planner
        # thread runs on refill — swap the chunk at the boundary and resume.
        eng, _ = make_engine("queue_async", chunk=10, autostart=False,
                             async_prefetch_ahead=1)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()  # sync initial plan (no thread with autostart=False)
        self.assertEqual(eng.stats["plans"], 1)
        pops = 0
        while eng.tick() is not None:
            pops += 1
        self.assertEqual(eng.remaining, 0)
        eng._run_plan()  # == what _planner_loop does when not idle
        self.assertGreater(eng.remaining, 0)
        self.assertGreaterEqual(eng.stats["plans"], 2)
        self.assertIsNotNone(eng.tick())
        eng.stop()

    def test_reset_then_refill(self):
        # deterministic: no planner thread in queue_sync
        eng, _ = make_engine("queue_sync", chunk=10)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        self.assertEqual(eng.remaining, 4)  # stub chunk is 4 rows
        eng.reset()
        self.assertEqual(eng.remaining, 0)
        self.assertIsNone(eng.last_error)
        row = eng.tick()  # queue_sync refills synchronously
        self.assertIsNotNone(row)
        eng.stop()


class TestRtc(unittest.TestCase):
    def test_rolling_replan_swaps_unexecuted_tail(self):
        # Deterministic: mimic the planner's rolling decision — after the min
        # tail has executed, a fresh plan swaps the tail of the running chunk.
        eng, _ = make_engine("rtc", chunk=10, autostart=False,
                             rtc_replan_interval_s=0.0, rtc_min_tail=2)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        self.assertEqual(eng.stats["plans"], 1)
        for _ in range(2):
            self.assertIsNotNone(eng.tick())  # execute the min tail
        self.assertEqual(eng.stats["plans"], 1)
        self.assertGreater(eng.remaining, 0)
        eng._run_plan()  # rolling replan while rows remain
        self.assertGreaterEqual(eng.stats["plans"], 2)
        self.assertGreater(eng.remaining, 0)
        eng.stop()


class TestThreadingSafety(unittest.TestCase):
    def test_engine_stop_joins_thread(self):
        eng, _ = make_engine("queue_async", autostart=True)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.start()
        self.assertIsNotNone(eng._thread)
        alive = True
        eng.stop()
        # thread references are cleared; verify no dangling writer
        self.assertIsNone(eng._thread)

    def test_start_raises_on_no_obs(self):
        eng, _ = make_engine("queue_sync")
        eng.set_enabled(True)
        with self.assertRaises(EngineStateError):
            eng.start()  # _plan_blocking silently no-ops without obs

    def test_queue_async_control_thread_not_starved_by_planner(self):
        """回归：queue_async 空闲时 planner 不得持锁饿死控制线程。

        根因：`_planner_loop` 把 `self._stop.wait(0.005)` 写在 `with self._lock`
        内，空闲时 planner 几乎 100% 持有引擎锁（5ms 等待+立即重获），`tick()`
        在锁上饿死 → 即使瞬时推理也只有 ~15Hz。修复：wait 移出锁外。
        被 plan A（后端返回完整 chunk → planner 进入空闲路径）暴露。
        """
        class FullChunkBackend(StubBackend):
            def infer(self, obs):
                return np.zeros((50, DIM), dtype=np.float64)

        eng = ActionEngine(
            FullChunkBackend(action_dim=DIM, camera_map={}),
            mode="queue_async", action_dim=DIM, chunk=50, policy_fps=30,
            autostart=True,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.start()
        t0 = time.monotonic()
        n = 0
        while time.monotonic() - t0 < 0.5:
            if eng.tick() is not None:
                n += 1
        eng.stop()
        # 修复前 ~7 pop/0.5s；修复后 >1000。200 为 10× 余量的宽松阈值。
        self.assertGreater(n, 200, f"control thread starved: {n} pops in 0.5s")


if __name__ == "__main__":
    unittest.main(verbosity=2)
