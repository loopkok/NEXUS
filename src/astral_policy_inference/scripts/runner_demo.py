#!/usr/bin/env python3
"""Non-ROS PolicyRunner 全功能演示（无 ROS2，真实 ACT 进程内推理）。

完整走一遍推理包的编排功能：
  policy → 30Hz 绝对动作 → pause 夹持 → resume 重规划 → playback 回放 →
  takeover(HUMAN, 释放控制) → release(回策略) → stop。

用法（lerobot 环境，真实 ACT 进程内）：
  PYTHONPATH=astral_ws/src/astral_policy_inference:astral_ws/src/astral_data_collect \
    <lerobot-env>/bin/python astral_ws/src/astral_policy_inference/scripts/runner_demo.py \
      --checkpoint-dir astral_ckpt/pickup_act_480/checkpoints/080000/pretrained_model
  或 --backend stub（任意 python，无需模型）。
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

import numpy as np

sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_policy_inference")
sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_data_collect")

from astral_data_collect.schema import CollectSchema
from astral_policy_inference.runner import PolicyRunner

fails: list[str] = []


def check(name, cond, detail=""):
    print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}")
    if not cond:
        fails.append(name)


def build_schema() -> CollectSchema:
    return CollectSchema(
        arms=["left"],
        end_effector_left="gripper",
        end_effector_right="none",
        include_waist=False,
        include_head=False,
        cameras=["video8", "video0"],
        dataset_fps=30,
    )


def build_runner(args) -> PolicyRunner:
    schema = build_schema()
    backend_cfg = {
        "backend_type": args.backend,
        "action_dim": schema.state_dim,
        "camera_map": {"base_0_rgb": "video8", "left_wrist_0_rgb": "video0"},
        "checkpoint_dir": args.checkpoint_dir,
        "host": "127.0.0.1",
        "port": 8001,
        "default_prompt": "Pick up the red-capped liquid container and place it into the box",
    }
    return PolicyRunner(
        backend_cfg=backend_cfg,
        schema=schema,
        engine_mode=args.engine_mode,
        replay_source=args.replay,
        # stub 是开发冒烟后端（微小漂移动作），不适用绝对语义守卫（守卫针对真实策略）
        abs_action_min_scale=0.0 if args.backend == "stub" else 0.5,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--backend", default="act", choices=["act", "stub"])
    ap.add_argument("--checkpoint-dir", default=None, help="act 后端需要的 checkpoint")
    ap.add_argument("--engine-mode", default="queue_async")
    ap.add_argument("--replay", default="", help="回放源（aligned h5 或 lerobot 目录）")
    ap.add_argument("--run-s", type=float, default=6.0, help="policy 阶段时长")
    args = ap.parse_args()

    if args.backend == "act" and not args.checkpoint_dir:
        ap.error("--backend act 需要 --checkpoint-dir")

    runner = build_runner(args)
    actions: list[np.ndarray] = []
    states: list[str] = []
    runner.on_action = lambda row: actions.append(np.asarray(row, dtype=np.float64))
    runner.on_state = lambda d: (states.append(d["state"]), states.__setitem__(-1, d["state"]))

    # 观测泵：30ms 喂一次真实量级关节 + 图像
    rng = np.random.default_rng(0)
    stop_pump = threading.Event()

    def pump():
        while not stop_pump.is_set():
            st = np.array([-0.35, 0.19, -0.004, -1.89, -0.20, -0.001, 0.014, 0.5])
            img = rng.integers(0, 256, (480, 480, 3), dtype=np.uint8)
            runner.feed_observation(
                state=st,
                images={"video8": img, "video0": img},
                prompt="Pick up the red-capped liquid container and place it into the box",
            )
            time.sleep(0.03)

    t_pump = threading.Thread(target=pump, daemon=True)
    t_pump.start()
    runner.start()
    time.sleep(0.3)  # 等观测到位

    def drain(s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            time.sleep(0.02)

    def wait_state(want, timeout=8.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if runner.stats()["state"] == want:
                return True
            time.sleep(0.05)
        return runner.stats()["state"] == want

    # ---- policy ----
    n_ticks = [0]
    runner.on_state = lambda d: (states.append(d["state"]),
                                 n_ticks.__setitem__(0, n_ticks[0] + 1))
    runner.request("policy")
    check("policy → POLICY", wait_state("POLICY"),
          f"state={runner.stats()['state']}")
    drain(0.5)  # warmup，等进入稳态
    n_ticks[0] = 0
    n_act0 = len(actions)
    t0 = time.monotonic()
    drain(args.run_s)
    el = time.monotonic() - t0
    rate = n_ticks[0] / el if el > 0 else 0.0
    new = actions[n_act0:]
    check("policy 控制率 ~30Hz",
          25 <= rate <= 35 and len(new) > 10,
          f"ctrl_rate={rate:.1f}Hz actions={len(new)}")
    check("动作有限且夹爪在 [0,1]",
          bool(np.isfinite(np.asarray(new)).all()) and all(0 <= a[-1] <= 1 for a in new),
          f"finite={bool(np.isfinite(np.asarray(new)).all())}")

    # ---- pause ----
    runner.request("pause")
    check("pause → POLICY_PAUSED", wait_state("POLICY_PAUSED"))
    a1 = actions[-1] if actions else None
    drain(0.5)
    a2 = actions[-1] if actions else None
    stable = a1 is not None and a2 is not None and np.allclose(a1, a2, atol=1e-9)
    check("pause 夹持最后目标", stable)

    # ---- resume ----
    runner.request("resume")
    check("resume → POLICY（重规划）", wait_state("POLICY"))

    # ---- takeover / release（中断 POLICY，release 应回到 POLICY）----
    runner.request("takeover")
    check("takeover → HUMAN（释放控制）", wait_state("HUMAN"))
    n2 = len(actions)
    drain(0.5)
    check("HUMAN 期间无输出", len(actions) == n2)
    runner.request("release")
    check("release → POLICY（重规划）", wait_state("POLICY"),
          f"state={runner.stats()['state']}")

    # ---- stop + playback ----
    runner.request("stop")
    check("stop → IDLE", wait_state("IDLE"))
    if args.replay:
        n1 = len(actions)
        runner.request(f"playback:{args.replay}")
        check("playback → PLAYBACK", wait_state("PLAYBACK"),
              f"state={runner.stats()['state']}")
        drain(1.0)
        check("回放出动作", len(actions) > n1, f"Δactions={len(actions)-n1}")

    # ---- 最终 stop + 指标 ----
    runner.request("stop")
    check("stop → IDLE", wait_state("IDLE"))
    st = runner.stats()
    print(f"  engine stats: {st['engine']}")
    print(f"  latency_ms: {st['latency_ms']}")
    print(f"  exec_events: {st['exec_events'][-5:]}")

    stop_pump.set()
    runner.stop()
    print("\nRESULT", "ALL PASS" if not fails else "FAILURES: " + ",".join(fails))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
