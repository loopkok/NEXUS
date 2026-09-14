#!/usr/bin/env python3
"""停顿压缩：删掉训练数据中的长时间停顿/慢速段，让 ACT 学到更流畅的轨迹。

动机：真机推理的固定卡点（到试管前/夹取后/放置前/释放后）＝ 训练数据里操作员
在任务阶段转换处的停顿/减速被模型忠实复现（实测该数据集 20.6% 帧速度 <0.008
rad/帧）。对 raw 数据做时间压缩（删停顿段中间帧），重新走 vla_process 转换，
模型学到的速度 profile 更均匀 → 真机更流畅。

**在 raw 层删帧**（robot_data.h5 / camera_data.h5），输出新 session 目录；后续
`vla_process_openpi.sh` / `vla_process_act.sh` 基于删帧后的 raw 重新对齐/转换
（next-state 语义在 align 阶段重建，无需手动处理；相机 JPEG 字节原样保留，零重编码）。

用法:
  /usr/bin/python3 astral_ws/scripts/compress_pauses.py \
      --session ~/astral_data/raw/pick_place_merged \
      --out-session ~/astral_data/raw/pick_place_merged_smooth \
      [--speed-thresh 0.008] [--min-pause 3] [--min-keep 4] [--dry-run]

参数:
  --speed-thresh  判定"低速"的阈值 rad/帧（对 left_arm_state 相邻帧 max|Δ|；
                  0.008=完全停顿，0.02=连减速移动也算，默认 0.008）
  --min-pause     至少连续多少帧低速才算停顿段（默认 3，0.1s @30fps）
  --min-keep      每段停顿保留帧数（首尾各 min_keep//2，默认 4 → 保留短暂过渡感）
  --dry-run       只报告每 episode 会删多少帧与速度分布变化，不写文件
退出码 0=成功。
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys

import h5py
import numpy as np

STATE_STREAM = "left_arm_state"  # 停顿判定流（7 维臂关节，排除夹爪 ratio 干扰）


def pause_delete_windows(state_t: np.ndarray, speed: np.ndarray,
                         thresh: float, min_pause: int, min_keep: int) -> list[tuple]:
    """返回待删除的停顿窗口 [(t_low, t_high), ...]（开区间，此时间窗内的帧删除）。

    speed[i] = state[i]→state[i+1] 的位移；低速帧 = speed < thresh。连续 >= min_pause
    个低速帧构成停顿段，段内保留首尾各 min_keep//2 帧（其余时间戳区间整段删除）。
    """
    low = speed < thresh
    windows = []
    i = 0
    n = len(state_t)
    while i < n - 1:
        if low[i]:
            j = i
            while j < n - 1 and low[j]:
                j += 1
            seg_len = j - i + 1  # 帧数（i..j 共 seg_len 帧，末帧速度未定义但算段长）
            if seg_len > min_keep:  # 段足够长才值得删
                keep_side = min_keep // 2
                # 保留前 keep_side 帧与后 keep_side 帧；删它们之间的时间窗（开区间）
                t_low = state_t[i + keep_side - 1]      # 保留到这一帧
                t_high = state_t[j - keep_side + 1]     # 从这一帧起保留
                if t_high > t_low:
                    windows.append((t_low, t_high))
            i = j
        else:
            i += 1
    return windows


def compress_episode(ep_dir: str, out_ep_dir: str, thresh: float,
                     min_pause: int, min_keep: int, dry_run: bool = False) -> dict:
    """压缩单个 episode 的 raw 数据：删停顿窗口内的帧，写入 out_ep_dir。

    dry_run=True 只统计不写（保留 out_ep_dir 未创建）。
    """
    robot_h5 = os.path.join(ep_dir, "robot_data.h5")
    cam_h5 = os.path.join(ep_dir, "camera_data.h5")
    if not dry_run:
        os.makedirs(out_ep_dir, exist_ok=True)

    stats: dict = {}
    with h5py.File(robot_h5, "r") as f:
        # 读 state 流算速度
        st_vals = np.asarray(f[f"streams/{STATE_STREAM}/values"])
        st_t = np.asarray(f[f"streams/{STATE_STREAM}/timestamps"])
    if len(st_t) < 4:
        if not dry_run:
            shutil.copy2(robot_h5, os.path.join(out_ep_dir, "robot_data.h5"))
            shutil.copy2(cam_h5, os.path.join(out_ep_dir, "camera_data.h5"))
        return {"episode": os.path.basename(ep_dir), "kept": len(st_t), "dropped": 0}

    speed = np.abs(np.diff(st_vals, axis=0)).max(axis=1)  # len = n-1
    windows = pause_delete_windows(st_t, speed, thresh, min_pause, min_keep)
    dropped_total = 0
    per_stream = {}

    def keep_mask(ts: np.ndarray) -> np.ndarray:
        """该流时间戳中，不在任何删除窗口内的帧为 True。"""
        if not windows:
            return np.ones(len(ts), dtype=bool)
        ts = np.asarray(ts)
        keep = np.ones(len(ts), dtype=bool)
        for t_lo, t_hi in windows:
            keep &= ~((ts > t_lo) & (ts < t_hi))
        return keep

    if not dry_run:
        # ---- 重写 robot_data.h5（所有流同步删帧；结构 = streams/<流>/values+timestamps）----
        def _copy_group(src_g, dst_g):
            """递归复制 group 树；dataset 层按同组 timestamps 的 keep_mask 删帧。"""
            for name in src_g:
                item = src_g[name]
                if isinstance(item, h5py.Group):
                    _copy_group(item, dst_g.create_group(name))
                else:
                    data = np.asarray(item)
                    out = data
                    if "timestamps" in src_g:
                        ts = np.asarray(src_g["timestamps"])
                        out = data[keep_mask(ts)]
                        if name == "timestamps":
                            per_stream.setdefault(
                                "/".join(src_g.name.strip("/").split("/")[1:]), 0
                            )
                    dst_g.create_dataset(name, data=out, dtype=item.dtype)

        with h5py.File(robot_h5, "r") as fsrc, h5py.File(
                os.path.join(out_ep_dir, "robot_data.h5"), "w") as fdst:
            for gname in fsrc.keys():
                g = fsrc[gname]
                if isinstance(g, h5py.Group):
                    _copy_group(g, fdst.create_group(gname))
                else:  # 顶层 dataset（罕见）：原样
                    fdst.create_dataset(gname, data=np.asarray(g))
            for k, v in fsrc.attrs.items():
                fdst.attrs[k] = v
        # per_stream 统计修正（上面 setdefault 占位）：重读实际长度
        with h5py.File(os.path.join(out_ep_dir, "robot_data.h5"), "r") as f:
            def count(name, obj):
                if isinstance(obj, h5py.Dataset) and name.endswith("/timestamps"):
                    key = name.rsplit("/", 2)[-2]
                    per_stream[key] = len(obj)
            f.visititems(count)

        # ---- 重写 camera_data.h5（各路同步删帧）----
        with h5py.File(cam_h5, "r") as fsrc, h5py.File(
                os.path.join(out_ep_dir, "camera_data.h5"), "w") as fdst:
            for cname in fsrc.keys():
                c = fsrc[cname]
                cdest = fdst.create_group(cname)
                ts = np.asarray(c["timestamps"])
                m = keep_mask(ts)
                per_stream[f"cam/{cname}"] = int(m.sum())
                for dname in c:
                    d = c[dname]
                    data = np.asarray(d)
                    if dname == "images" and data.dtype == object:
                        # vlen JPEG 字节：按掩码复制，用源 dataset 的 vlen dtype 创建
                        src = data[m]
                        out = np.empty(len(src), dtype=object)
                        for kk in range(len(src)):
                            out[kk] = src[kk]
                        cdest.create_dataset(dname, data=out, dtype=d.dtype)
                    else:
                        out = data[m]
                        cdest.create_dataset(dname, data=out)

        # 复制 meta.json（原样）
        meta_src = os.path.join(ep_dir, "meta.json")
        if os.path.exists(meta_src):
            shutil.copy2(meta_src, os.path.join(out_ep_dir, "meta.json"))
    else:
        # dry-run：只算 state 保留数，不写文件
        per_stream[STATE_STREAM] = int(keep_mask(st_t).sum())

    kept = int(per_stream.get(STATE_STREAM, len(st_t)))
    dropped_total = max(0, len(st_t) - kept)
    return {
        "episode": os.path.basename(ep_dir),
        "state_frames": len(st_t),
        "kept": kept,
        "dropped": dropped_total,
        "pct": (dropped_total / len(st_t) * 100) if len(st_t) else 0,
        "windows": len(windows),
        "streams": per_stream,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", required=True, help="raw session 目录（含 episode*/）")
    ap.add_argument("--out-session", required=True, help="输出新 session 目录")
    ap.add_argument("--speed-thresh", type=float, default=0.008, help="低速阈值 rad/帧（默认 0.008=只压完全停顿；0.02=连减速也压）")
    ap.add_argument("--min-pause", type=int, default=3, help="连续低速 >= 此帧数才算停顿（默认 3）")
    ap.add_argument("--min-keep", type=int, default=4, help="每段停顿保留帧数（首尾各半，默认 4）")
    ap.add_argument("--dry-run", action="store_true", help="只报告不写文件")
    args = ap.parse_args()

    if not os.path.isdir(args.session):
        print(f"session 不存在: {args.session}", file=sys.stderr)
        return 1
    eps = sorted(d for d in os.listdir(args.session)
                 if d.startswith("episode") and os.path.isdir(os.path.join(args.session, d)))
    if not eps:
        print(f"session 下无 episode 目录: {args.session}", file=sys.stderr)
        return 1

    out_root = args.session if args.dry_run else args.out_session
    if not args.dry_run:
        os.makedirs(out_root, exist_ok=True)

    total_dropped = 0
    total_frames = 0
    n_win = 0
    print(f"停顿压缩: 阈值={args.speed_thresh} rad/帧, 最少停顿={args.min_pause}帧, "
          f"每段保留={args.min_keep}帧, {'[DRY-RUN]' if args.dry_run else ''}")
    for ep in eps:
        ep_dir = os.path.join(args.session, ep)
        out_ep = os.path.join(out_root, ep)
        st = compress_episode(ep_dir, out_ep, args.speed_thresh,
                              args.min_pause, args.min_keep, dry_run=args.dry_run)
        total_dropped += st["dropped"]
        total_frames += st["state_frames"]
        n_win += st["windows"]
        print(f"  {ep}: {st['state_frames']}→{st['kept']} 帧 "
              f"(-{st['dropped']}, {st['pct']:.0f}%)  停顿段 {st['windows']} 个")

    if total_frames:
        print(f"\n合计: {total_frames}→{total_frames - total_dropped} 帧 "
              f"(-{total_dropped}, {total_dropped / total_frames * 100:.1f}%), 停顿段 {n_win} 个")
    print(f"{'输出到: ' + args.out_session if not args.dry_run else 'dry-run 结束，未写文件'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
