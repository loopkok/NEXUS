#!/usr/bin/env python3
"""推理时间曲线诊断：从 joint_stream_log 画 state/action 曲线，算错位时间与动作平滑度。

用法（先跑推理，节点 joint_stream_log_file:=/tmp/pi_cmds.jsonl 已记录）：
  /usr/bin/python3 astral_ws/scripts/plot_inference_curves.py \
      --log /tmp/pi_cmds.jsonl --out /tmp/pi_curves.png

输出：
  * PNG：7 个臂关节 state(蓝实) vs command(红虚) 时间曲线 + 动作步长(平滑度) + 夹爪；
  * 终端：state↔command 错位时间（逐关节互相关，+ = command 领先）、动作步长分布/尖峰。

日志每行 JSON：{"t": epoch, "/left_arm/joint_commands": [7], "/left_gripper/command": [1],
"state": [8]}（state 为节点新加的观测向量，旧日志可能没有 → 跳过错位分析）。
退出码 0=成功。
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np


def load(path: str) -> list[dict]:
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except Exception:  # noqa: BLE001
                continue
            if "/left_arm/joint_commands" in d:
                rows.append(d)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--log", required=True, help="joint_stream_log_file 产物 (jsonl)")
    ap.add_argument("--out", default="/tmp/pi_curves.png", help="PNG 输出路径")
    ap.add_argument("--lag-max", type=int, default=50, help="互相关搜索最大滞后（行）")
    ap.add_argument("--smooth-thresh", type=float, default=0.1, help="动作尖峰阈值 rad/步")
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover
        print(f"需要 matplotlib：{exc}（可用 /home/robot/miniconda3/envs/lerobot/bin/python）",
              file=sys.stderr)
        return 1

    rows = load(args.log)
    if not rows:
        print("日志无有效行", file=sys.stderr)
        return 1

    t = np.array([r["t"] for r in rows])
    cmd = np.array([r["/left_arm/joint_commands"] for r in rows], dtype=float)  # (N,7)
    grip = np.array([
        (r.get("/left_gripper/command") or [np.nan])[0] for r in rows
    ], dtype=float)
    has_state = all(r.get("state") is not None for r in rows)
    st = np.array([r["state"] for r in rows], dtype=float) if has_state else None  # (N,8)

    t = t - t[0]
    hz = len(t) / max(t[-1], 1e-6)
    NJ = cmd.shape[1]
    print(f"{len(rows)} 行, {t[-1]:.1f}s, ~{hz:.1f}Hz, state={'有' if has_state else '无'}")

    # ---------------- 动作平滑度：每行最大|Δ| ----------------
    dcmd = np.abs(np.diff(cmd, axis=0)).max(axis=1) if len(cmd) > 1 else np.zeros(0)
    dt = np.diff(t)
    if len(dcmd):
        print("动作步长(rad/步): "
              + " ".join(f"p{p}={np.percentile(dcmd, p):.4f}" for p in (50, 90, 99))
              + f"  max={dcmd.max():.4f}")
        spikes = dcmd > args.smooth_thresh
        print(f"步长>{args.smooth_thresh} 尖峰: {spikes.sum()} 个 ({(spikes.mean() * 100):.1f}%)")
        for i in np.where(spikes)[0][:10]:
            vel = dcmd[i] / max(dt[i], 1e-6)
            print(f"  t={t[i + 1]:6.2f}s  步长 {dcmd[i]:.3f} rad  (~{vel:.1f} rad/s)")

    # ---------------- state ↔ command 错位时间（逐关节互相关） ----------------
    lags: dict[int, tuple[int, float]] = {}
    if st is not None and len(st) > 4 * args.lag_max:
        n = len(st)
        L = min(args.lag_max, n // 4)
        for j in range(NJ):
            x = st[:, j] - st[:, j].mean()
            y = cmd[:, j] - cmd[:, j].mean()
            best_s, best_c = 0, -2.0
            for s in range(-L, L + 1):
                if s >= 0:
                    a, b = x[: n - s], y[s:]      # state[i] vs command[i+s]
                else:
                    a, b = x[-s:], y[: n + s]     # state[i-s] vs command[i]
                if len(a) < 4:
                    continue
                c = float(np.corrcoef(a, b)[0, 1])
                if c > best_c:
                    best_c, best_s = c, s
            lags[j] = (best_s, best_c)
        mean_s = np.mean([v[0] for v in lags.values()])
        # 符号约定：best_s<0 时 state[i]≈command[i-|s|] → command 领先 state。
        direction = "state 领先 command" if mean_s > 0 else "command 领先 state"
        print(f"state↔command 错位：")
        for j, (s, c) in lags.items():
            print(f"  joint{j}: {s:+d} 行 ({s / hz * 1000:+.0f}ms)  corr={c:.3f}")
        print(f"  平均 {abs(mean_s) / hz * 1000:.0f}ms ({direction}, {abs(mean_s):.1f} 行 @{hz:.0f}Hz)")
    else:
        print("state 数据不足或旧日志无 state，跳过错位分析")

    # ---------------- 绘图 ----------------
    fig, axes = plt.subplots(3, 3, figsize=(16, 12))
    order = [0, 1, 2, 3, 4, 5, 6]
    for k, j in enumerate(order):
        ax = axes[k // 3][k % 3]
        if st is not None:
            ax.plot(t, st[:, j], "b-", lw=0.9, label="state")
        ax.plot(t, cmd[:, j], "r--", lw=0.8, label="command")
        ax.set_title(f"joint{j}")
        ax.set_xlabel("t (s)")
        ax.legend(fontsize=7)
    # 平滑度面板
    ax = axes[2][1]
    ax.plot(t[1:], dcmd, "k-", lw=0.6)
    ax.axhline(args.smooth_thresh, color="r", ls=":", lw=0.8, label="thresh")
    ax.set_title("action step size (max|Δ|/step)")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("rad/step")
    ax.legend(fontsize=7)
    # 夹爪 + 统计面板
    ax = axes[2][2]
    ax.plot(t, grip, "g-", lw=0.9, label="grip cmd")
    if st is not None:
        ax.plot(t, st[:, -1], "b--", lw=0.8, label="grip state")
    ax.set_title("gripper")
    ax.set_xlabel("t (s)")
    ax.legend(fontsize=7)
    txt = [f"{len(rows)} rows, ~{hz:.0f}Hz"]
    if lags:
        ms = mean_s / hz * 1000
        txt.append(f"state->cmd offset: {ms:+.0f}ms ({direction})")
    if len(dcmd):
        txt.append(f"step p50/p90/max: "
                   f"{np.percentile(dcmd, 50):.3f}/{np.percentile(dcmd, 90):.3f}/{dcmd.max():.3f} rad")
    ax.text(0.02, 0.98, "\n".join(txt), transform=ax.transAxes, va="top",
            family="monospace", fontsize=9)
    fig.suptitle("Inference curves: state vs action, offset & smoothness", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(args.out, dpi=120)
    print(f"图已存: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
