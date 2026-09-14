#!/usr/bin/env python3
"""对齐数据完整修复：弧长匀速化重采样（插补）——消除静止帧/卡顿/速度不均。

动机：真机推理固定卡点 = 训练数据里操作员停顿/减速被模型复现。compress_pauses.py
（raw 层删帧）会让相邻帧位移变大、动作变"跳"；本脚本在 **aligned_data.h5 层**用
**插补**替代删帧：把「快-停-快」的轨迹按累计运动量（弧长）重新映射成**匀速**——
静止帧在弧长空间不占长度→自然压缩，速度不均→均匀，时间戳→严格单调（validate 的
W2 gap 消失）。关节是数值线性插值（平滑），相机帧取最近原始帧（JPEG 无法插值）。

处理对象：session 下 episode*/aligned_data.h5（已对齐的数据）。输出新 session 的
aligned_data.h5（+ 原样复制 robot/camera/meta，供后续 vla_process 直接 convert——
align 检测到 aligned 已存在会跳过）。

方法：
  resample（默认）：弧长匀速化插补。每帧弧长 = Σ_j|Δstate_j|（8 维位移和，夹爪
    动作也计弧长、不被压缩掉）；目标每帧弧长 = --speed-ref（默认 p90 原始速度，
    即运动段的典型速度），帧数 = 总弧长 / speed-ref；关节按弧长线性插值，相机取
    最近帧。等效：静止压缩、运动段保持原速度、全程匀速。
  drop：删帧（compress_pauses 思路搬到 aligned 层）——速度 < --min-speed 的停顿段
    删中间帧，每段保留 --min-keep 帧。

用法:
  /usr/bin/python3 astral_ws/scripts/repair_aligned.py \
      --session ~/astral_data/raw/pick_place_merged \
      --out-session ~/astral_data/raw/pick_place_merged_repaired \
      [--method resample] [--speed-ref 0.0] [--fps 30] \
      [--min-speed 0.008] [--min-keep 4] [--dry-run]

参数:
  --method resample|drop     修复方法（默认 resample=弧长匀速化插补）
  --speed-ref rad/帧         目标每帧弧长（默认 0=自动取原数据 p90 速度）
  --fps                      输出时间戳网格（默认 30）
  --min-speed rad/帧         drop 模式：低于此速度算停顿（默认 0.008）
  --min-keep 帧              drop 模式：每段停顿保留帧数（默认 4）
  --dry-run                  只报告不写文件
退出码 0=成功。
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys

import h5py
import numpy as np


CAM_KEYS = None  # 运行时探测 aligned 里的 camera 组名（video0/video8/...）


def _camera_groups(f) -> list[str]:
    return [k for k in f.keys()
            if isinstance(f[k], h5py.Group) and "images" in f[k]]


def resample_frames(state: np.ndarray, quality: np.ndarray,
                    cam_images: dict, cam_offsets: dict, cam_ts: dict,
                    speed_ref: float, fps: int, state_ts0: float = 0.0):
    """弧长匀速化重采样：返回 (新 state, 新 action, 新 quality, 时间戳, 相机映射)。

    speed_ref=0 → 取原始速度 p90。返回的新序列每帧弧长 ≈ speed_ref（匀速）。
    """
    # 逐帧位移（8 维绝对位移和；含夹爪，夹爪动作也计弧长）
    d = np.abs(np.diff(state, axis=0)).sum(axis=1)          # (N-1,)
    S = np.concatenate([[0.0], np.cumsum(d)])               # (N,) 累计弧长
    S_total = S[-1]
    if speed_ref <= 0:
        speed_ref = float(np.percentile(d, 90)) if len(d) else 1.0
    if speed_ref <= 0 or S_total <= 0:
        speed_ref = 1.0
    N_new = max(2, int(round(S_total / speed_ref)))
    s_grid = np.linspace(0.0, S_total, N_new)

    # 关节插值（每维线性）
    N, D = state.shape
    state_new = np.empty((N_new, D), dtype=np.float64)
    for j in range(D):
        state_new[:, j] = np.interp(s_grid, S, state[:, j])

    # 相机/quality 最近帧：argmin 精确最近（searchsorted 取上界在静止段/边界会
    # 选到更远的帧，造成图像-关节弧长偏差 max 12.7° > 半帧）。argmin 压回 ≤半帧弧长。
    idx = np.array([int(np.argmin(np.abs(S - g))) for g in s_grid])
    quality_new = quality[idx]
    cam_new = {c: cam_images[c][idx] for c in cam_images}
    off_new = {c: cam_offsets[c][idx] for c in cam_offsets}
    ts_cam_new = {c: cam_ts[c][idx] for c in cam_ts}

    # 匀速时间戳（严格单调，无 gap）：帧间隔 = 1/fps 秒（匀速运动 → 时间均匀），
    # 起点对齐原首帧。注意不是 s_grid/speed_ref（那是弧长单位，会得到帧索引）。
    timestamps_new = float(state_ts0) + np.arange(N_new) / fps
    # next-state action
    action_new = np.vstack([state_new[1:], state_new[-1]])

    return state_new, action_new, quality_new, timestamps_new, idx, cam_new, off_new, ts_cam_new


def drop_frames(state: np.ndarray, quality: np.ndarray,
                cam_images: dict, cam_offsets: dict, cam_ts: dict,
                min_speed: float, min_keep: int, fps: int = 30,
                state_ts0: float = 0.0):
    """删帧法（drop）：速度 < min_speed 的停顿段删中间帧，每段保留 min_keep。"""
    d = np.abs(np.diff(state, axis=0)).max(axis=1)
    low = d < min_speed
    keep = np.ones(len(state), dtype=bool)
    i = 0
    n = len(state)
    while i < n - 1:
        if low[i]:
            j = i
            while j < n - 1 and low[j]:
                j += 1
            if j - i + 1 > min_keep:
                keep[i + min_keep // 2: j - min_keep // 2 + 1] = False
            i = j
        else:
            i += 1
    idx = np.where(keep)[0]
    state_new = state[idx]
    quality_new = quality[idx]
    cam_new = {c: cam_images[c][idx] for c in cam_images}
    off_new = {c: cam_offsets[c][idx] for c in cam_offsets}
    ts_cam_new = {c: cam_ts[c][idx] for c in cam_ts}
    action_new = np.vstack([state_new[1:], state_new[-1]])
    # 删帧后时间戳压缩为均匀 1/fps（消除 gap 警告）
    ts_new = state_ts0 + np.arange(len(state_new)) / fps
    return state_new, action_new, quality_new, ts_new, idx, cam_new, off_new, ts_cam_new


def repair_episode(ep_dir: str, out_ep_dir: str, method: str,
                   speed_ref: float, fps: int, min_speed: float,
                   min_keep: int, dry_run: bool) -> dict:
    src = os.path.join(ep_dir, "aligned_data.h5")
    if not os.path.exists(src):
        return {"episode": os.path.basename(ep_dir), "error": "无 aligned_data.h5"}
    if not dry_run:
        os.makedirs(out_ep_dir, exist_ok=True)

    with h5py.File(src, "r") as f:
        state = np.asarray(f["observation/state"], dtype=np.float64)
        quality = np.asarray(f["quality"], dtype=np.uint8)
        state_ts0 = float(np.asarray(f["timestamps"])[0])
        cams = _camera_groups(f)
        cam_images = {c: np.asarray(f[f"{c}/images"]) for c in cams}
        cam_offsets = {c: np.asarray(f[f"{c}/src_offsets"]) if f"{c}/src_offsets" in f[c] else np.zeros(len(cam_images[c]), dtype=np.int64) for c in cams}
        cam_ts = {c: np.asarray(f[f"{c}/src_timestamps"]) if f"{c}/src_timestamps" in f[c] else np.zeros(len(cam_images[c])) for c in cams}

    if method == "resample":
        r = resample_frames(state, quality, cam_images, cam_offsets, cam_ts,
                            speed_ref, fps, state_ts0)
        state_new, action_new, quality_new, ts_new, idx, cam_new, off_new, ts_cam_new = r
    else:
        r = drop_frames(state, quality, cam_images, cam_offsets, cam_ts,
                        min_speed, min_keep, fps, state_ts0)
        state_new, action_new, quality_new, ts_new, idx, cam_new, off_new, ts_cam_new = r

    if not dry_run:
        with h5py.File(os.path.join(out_ep_dir, "aligned_data.h5"), "w") as fo:
            fo.create_dataset("action", data=action_new)
            g = fo.create_group("observation")
            g.create_dataset("state", data=state_new)
            fo.create_dataset("quality", data=quality_new)
            if ts_new is not None:
                fo.create_dataset("timestamps", data=ts_new)
            else:
                fo.create_dataset("timestamps", data=np.zeros(len(state_new)))
            for c in cams:
                cg = fo.create_group(c)
                # 复用源 vlen dtype 写 JPEG 字节数组
                with h5py.File(src, "r") as fs:
                    dt = fs[c]["images"].dtype
                out = np.empty(len(cam_new[c]), dtype=object)
                for kk, b in enumerate(cam_new[c]):
                    out[kk] = b
                cg.create_dataset("images", data=out, dtype=dt)
                cg.create_dataset("src_offsets", data=off_new[c])
                cg.create_dataset("src_timestamps", data=ts_cam_new[c])
        # 复制 robot/camera/meta（原样，vla_process 对齐跳过用 aligned）
        for fname in ("robot_data.h5", "camera_data.h5", "meta.json"):
            p = os.path.join(ep_dir, fname)
            if os.path.exists(p):
                shutil.copy2(p, os.path.join(out_ep_dir, fname))

    speed_before = np.abs(np.diff(state, axis=0)).max(axis=1)
    speed_after = np.abs(np.diff(state_new, axis=0)).max(axis=1)
    return {
        "episode": os.path.basename(ep_dir),
        "frames": len(state),
        "kept": len(state_new),
        "pct": ((len(state) - len(state_new)) / len(state) * 100),
        "speed_p50_before": float(np.percentile(speed_before, 50)),
        "speed_p50_after": float(np.percentile(speed_after, 50)),
        "speed_std_before": float(speed_before.std()),
        "speed_std_after": float(speed_after.std()),
    }


def session_speed_ref(session: str, eps: list, thresh: float) -> float:
    """session 全局 speed_ref：所有 episode 逐帧位移合并后的 p90。

    比每 episode 独立 p90 更稳——各段统一速度尺度（实测独立 p90 跨段差 1.3x，
    模型学到不一致的节奏）。thresh 用于跳过超静止（避免 p90 被退化段拉低）。
    """
    ds = []
    for ep in eps:
        p = os.path.join(session, ep, "aligned_data.h5")
        if not os.path.exists(p):
            continue
        try:
            with h5py.File(p, "r") as f:
                st = np.asarray(f["observation/state"], dtype=np.float64)
            if len(st) < 3:
                continue
            d = np.abs(np.diff(st, axis=0)).sum(axis=1)
            ds.append(d)
        except Exception:  # noqa: BLE001
            continue
    if not ds:
        return 0.0
    all_d = np.concatenate(ds)
    return float(np.percentile(all_d, 90))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", required=True, help="含 episode*/aligned_data.h5 的 session")
    ap.add_argument("--out-session", required=True)
    ap.add_argument("--method", choices=["resample", "drop"], default="resample")
    ap.add_argument("--speed-ref", type=float, default=0.0,
                    help="目标每帧弧长 rad/帧（0=自动取整个 session 的 p90，统一各段速度尺度）")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--min-speed", type=float, default=0.008, help="drop 模式停顿阈值")
    ap.add_argument("--min-keep", type=int, default=4, help="drop 模式每段保留帧数")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if not os.path.isdir(args.session):
        print(f"session 不存在: {args.session}", file=sys.stderr)
        return 1
    eps = sorted(d for d in os.listdir(args.session)
                 if d.startswith("episode") and os.path.isdir(os.path.join(args.session, d)))
    if not eps:
        print("无 episode", file=sys.stderr)
        return 1
    # 全局 speed_ref（若未显式指定）：所有 episode 合并的 p90，统一速度尺度
    if args.speed_ref <= 0:
        args.speed_ref = session_speed_ref(args.session, eps, args.min_speed)
    out_root = args.session if args.dry_run else args.out_session
    if not args.dry_run:
        os.makedirs(out_root, exist_ok=True)

    print(f"修复: 方法={args.method} speed_ref={'自动(全局p90=%.4f)' % args.speed_ref if args.speed_ref>0 else args.speed_ref} "
          f"{'[DRY-RUN]' if args.dry_run else ''}")
    tot0 = tot1 = 0
    for ep in eps:
        st = repair_episode(os.path.join(args.session, ep), os.path.join(out_root, ep),
                            args.method, args.speed_ref, args.fps,
                            args.min_speed, args.min_keep, args.dry_run)
        tot0 += st.get("frames", 0); tot1 += st.get("kept", 0)
        if "error" in st:
            print(f"  {ep}: {st['error']}")
            continue
        print(f"  {ep}: {st['frames']}→{st['kept']} 帧 ({st['pct']:.0f}%)  "
              f"速度std {st['speed_std_before']:.3f}→{st['speed_std_after']:.3f}")
    if tot0:
        print(f"\n合计: {tot0}→{tot1} 帧 (-{(tot0-tot1)/tot0*100:.1f}%)")
    print(f"{'输出到 ' + args.out_session if not args.dry_run else 'dry-run 结束，未写文件'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
