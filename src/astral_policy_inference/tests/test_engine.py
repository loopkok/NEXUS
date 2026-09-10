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


class TestAbsoluteSemanticsGuard(unittest.TestCase):
    """回归：后端返回 delta 语义（远离当前 state 的小值）时引擎拒绝 chunk。

    后端契约是绝对动作；openpi server 若漏 AbsoluteActions、或错 checkpoint，
    会把 delta 当绝对返回 → 首行≈小 delta，在 |state|>1 的维上 |action-state|≈|state|
    超阈值 → 引擎报错（节点据此安全 stop，不静默跳到错误目标）。
    """

    def _away_state(self):
        st = np.zeros(DIM)
        st[0], st[3] = 1.5, -1.8  # 臂维明显离开零位
        return st

    def test_rejects_delta_like_actions(self):
        class DeltaBackend(StubBackend):
            def infer(self, obs):
                return np.full((4, DIM), 0.03, dtype=np.float64)  # 小 delta

        eng = ActionEngine(DeltaBackend(action_dim=DIM, camera_map={}),
                           mode="queue_sync", action_dim=DIM, chunk=50,
                           policy_fps=30, autostart=False, abs_action_min_scale=0.5)
        eng.feed_obs(ObsBatch(state=self._away_state(), images={}, prompt=""))
        with self.assertRaises(EngineStateError):
            eng.start()  # 首次 plan 的绝对语义检查失败 → engine error

    def test_accepts_absolute_actions(self):
        class AbsBackend(StubBackend):
            def infer(self, obs):
                out = np.zeros((4, DIM))
                out[0] = obs.state  # 绝对 next-state ≈ 当前 state
                return out

        eng = ActionEngine(AbsBackend(action_dim=DIM, camera_map={}),
                           mode="queue_sync", action_dim=DIM, chunk=50,
                           policy_fps=30, autostart=False, abs_action_min_scale=0.5)
        eng.feed_obs(ObsBatch(state=self._away_state(), images={}, prompt=""))
        eng.start()  # 不应 raise
        self.assertIsNotNone(eng._chunk)

    def test_disabled_when_threshold_nonpositive(self):
        class DeltaBackend(StubBackend):
            def infer(self, obs):
                return np.full((4, DIM), 0.03, dtype=np.float64)

        eng = ActionEngine(DeltaBackend(action_dim=DIM, camera_map={}),
                           mode="queue_sync", action_dim=DIM, chunk=50,
                           policy_fps=30, autostart=False, abs_action_min_scale=0.0)
        eng.feed_obs(ObsBatch(state=self._away_state(), images={}, prompt=""))
        eng.start()  # 关闭时不拦截
        self.assertIsNotNone(eng._chunk)


class JumpBackend(StubBackend):
    """第 2 次起每次推理都返回一个全 1 的 (10, dim) chunk（第 1 次全 0）。
    故意制造"换 chunk 时 0→1 的硬跳变"，用于验证时序融合把边界步压下去。"""

    def __init__(self, dim=DIM):
        super().__init__(action_dim=dim, camera_map={})
        self.plans = 0

    def infer(self, obs):
        self.plans += 1
        val = 1.0 if self.plans >= 2 else 0.0
        return np.full((10, self.action_dim), val)


class TestTemporalEnsembling(unittest.TestCase):
    """引擎级 ACT 时序融合：换 chunk 时旧尾段与新头部按权重平均，边界不再硬跳。"""

    def _drive(self, coeff, chunk=10, prefetch=5, ticks=40):
        eng = ActionEngine(
            JumpBackend(), mode="queue_async", action_dim=DIM, chunk=chunk,
            autostart=False, async_prefetch_ahead=prefetch,
            temporal_ensemble_coeff=coeff,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()  # 首块（plans=1, 全 0）
        rows = []
        for _ in range(ticks):
            # 模拟 _planner_loop 判定：还有 >0 行可消费且剩余 <= prefetch → 重规划
            if 0 < eng.remaining <= prefetch:
                eng._run_plan()
            r = eng.tick()
            if r is None:  # 意外耗尽（async 无线程时兜底）
                eng._run_plan()
                r = eng.tick()
            if r is not None:
                rows.append(r)
        eng.stop()
        return np.asarray(rows)

    def test_ensembling_bounds_boundary_step(self):
        raw = self._drive(coeff=0.0)   # 无融合：0→1 硬跳
        ens = self._drive(coeff=0.01)  # 融合：边界步被压
        raw_steps = np.abs(np.diff(raw, axis=0)).max(axis=1)
        ens_steps = np.abs(np.diff(ens, axis=0)).max(axis=1)
        self.assertGreater(raw_steps.max(), 0.9, "无融合时应出现 ~1.0 的硬跳变")
        self.assertLess(ens_steps.max(), 0.9, "融合后边界步必须显著小于硬跳变")
        self.assertLess(ens_steps.max(), raw_steps.max(), "融合必须实际降低最大步长")
        self.assertEqual(ens.shape[1], DIM)
        self.assertTrue(np.isfinite(ens).all())

    def test_ensembling_first_plan_adopts_and_reset_clears(self):
        eng = ActionEngine(
            JumpBackend(), mode="queue_async", action_dim=DIM, chunk=10,
            autostart=False, temporal_ensemble_coeff=0.01,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        self.assertIsNotNone(eng._ensembler)
        self.assertIsNotNone(eng._ensembler.counts, "首块后计数应初始化")
        eng.reset()
        self.assertIsNone(eng._ensembler.counts, "reset 必须清空融合计数")
        self.assertEqual(eng.remaining, 0)
        eng.stop()

    def test_ensembling_short_chunk_does_not_crash(self):
        """后端返回行数 < chunk_size（StubBackend 4 行、截断响应）时不得越界。

        对抗性回归：update() 里 r 若只按 chunk_size 与 n 取小，会用 old[i+k]
        (k 到 r-1) 越出旧缓冲长度 → IndexError。r 必须同时受 len(old)-i 约束。
        """
        class ShortBackend(StubBackend):
            def __init__(self, dim=DIM):
                super().__init__(action_dim=dim, camera_map={})

            def infer(self, obs):
                return np.full((4, DIM), 0.5, dtype=np.float64)

        eng = ActionEngine(
            ShortBackend(), mode="queue_async", action_dim=DIM, chunk=10,
            autostart=False, async_prefetch_ahead=2, temporal_ensemble_coeff=0.01,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        for _ in range(30):
            if 0 < eng.remaining <= 2:
                eng._run_plan()  # 剩余 2 行时重规划 → 旧缓冲仅 4 行、已消费 2 行
            r = eng.tick()
            if r is None:
                eng._run_plan()
                r = eng.tick()
            self.assertIsNotNone(r, "短 chunk + 融合不应断流")
        eng.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)
