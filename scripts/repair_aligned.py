#!/usr/bin/env python3
"""对齐数据完整修复：弧长匀速化重采样（选帧）——消除静止帧/卡顿/速度不均。

动机：真机推理固定卡点 = 训练数据里操作员停顿/减速被模型复现。本脚本在
**aligned_data.h5 层**用**弧长均匀选帧**：把「快-停-快」的轨迹按累计运动量（弧长）
重新映射成**匀速**——静止帧在弧长空间不占长度→自然压缩，速度不均→均匀，时间戳→
严格单调（validate 的 W2 gap 消失）。**选帧**（取"弧长最近的原始帧"）保证 state 与
图像**严格同源**、零错位；插值会破坏这一点（图像只能取离散帧），故不用。

处理对象：session 下 episode*/aligned_data.h5（已对齐的数据）。输出新 session 的
aligned_data.h5（+ 原样复制 robot/camera/meta，供后续 vla_process 直接 convert——
align 检测到 aligned 已存在会跳过）。

方法（弧长匀速化选帧）：
  每帧弧长 = Σ_j|Δstate_j|，**仅臂关节维参与**（夹爪/末端 ratio 无量纲，与 rad 混算
  会污染速度参考——实测占全维弧长 15%、把 speed-ref 抬 16%）；目标每帧弧长 =
  --speed-ref（默认 session 臂维弧长**均值** = 总弧长/总帧数：重采样后帧数≈原始、
  总时长不变，只把静止/停滞帧的弧长匀到运动段——丝滑但**不加速**；旧默认 p90
  ≈1.9x 均值会整体提速并放大每帧跳变，实测每帧最大跳变 p50 0.019→0.043、>0.05 rad
  占比 16%→40%，故弃用）；每个重采样点取"弧长最近的原始帧"，静止段弧长≈0 被压缩、
  帧间位移≈speed-ref（速度近似均匀）；连续重复帧（快速帧弧长跨度>网格间距时的采样
  伪影）去重，避免在快跳后制造假停顿。

  --keep-intent（意图感知，可选）：默认把所有停顿一视同仁地压缩；加此开关后，用 raw
  *_cmd 流分型停顿——操作者**有意停顿**（cmd 同步停住，如抓握保持/放置等待）保留其
  时长（每帧推进 speed_ref → 停顿帧 1:1 进输出），只压缩**摩擦型停顿**（cmd 在动、
  state 被静摩擦卡住）。cmd/state 时间戳语义不同，分型前经互相关对齐（带方差/相关度
  守卫）。无 *_cmd 流的 episode 自动退化为全压缩。

用法:
  /usr/bin/python3 astral_ws/scripts/repair_aligned.py \
      --session ~/astral_data/raw/pick_place_merged \
      --out-session ~/astral_data/raw/pick_place_merged_repaired \
      [--speed-ref 0.0] [--fps 30] [--keep-intent] [--dry-run]

参数:
  --speed-ref rad/帧         目标每帧弧长（默认 0=自动取臂维弧长均值，保留总时长不加速）
  --fps                      输出时间戳网格（默认 30）
  --keep-intent              意图感知：保留操作者有意停顿的时长，只压缩摩擦型停顿
  --dry-run                  只报告不写文件
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

# 意图/摩擦分型复用 quantify_cmd_state 的对齐与互相关（同目录模块，无重依赖）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quantify_cmd_state import _cross_lag  # noqa: E402


CAM_KEYS = None  # 运行时探测 aligned 里的 camera 组名（video0/video8/...）


def arm_mask_from_meta(ep_dir: str, n_dim: int) -> np.ndarray | None:
    """返回臂维掩码（True=臂关节，参与弧长/速度判定）。

    读 meta.json 的 schema.state_blocks：name 含 "arm" 的块参与（left_arm/right_arm），
    ee/waist/head 排除（gripper 的 ratio 无量纲，与 rad 混算污染速度参考）。
    无 meta/schema 时回退 None=全维（历史行为）。
    """
    try:
        with open(os.path.join(ep_dir, "meta.json"), encoding="utf-8") as f:
            meta = json.load(f)
        mask = []
        for b in meta["schema"]["state_blocks"]:
            mask += ["arm" in b["name"]] * b["dim"]
        if len(mask) == n_dim:
            return np.asarray(mask, dtype=bool)
    except Exception:  # noqa: BLE001
        pass
    return None


def _camera_groups(f) -> list[str]:
    return [k for k in f.keys()
            if isinstance(f[k], h5py.Group) and "images" in f[k]]


def _load_cmd(ep_dir: str, arm_name: str | None = None) -> tuple[np.ndarray, np.ndarray] | None:
    """读 robot_data.h5 的臂 cmd 流（values, timestamps）；无则 None。

    arm_name（如 "left_arm"）给则优先取 {arm_name}_cmd（未来双臂时不会拿错侧），
    否则取第一个 *_cmd 流。
    """
    p = os.path.join(ep_dir, "robot_data.h5")
    if not os.path.exists(p):
        return None
    with h5py.File(p, "r") as f:
        if "streams" not in f:
            return None
        names = [k for k in f["streams"] if k.endswith("_cmd")]
        if not names:
            return None
        name = names[0]
        if arm_name and (arm_name + "_cmd") in f["streams"]:
            name = arm_name + "_cmd"
        g = f[f"streams/{name}"]
        return (np.asarray(g["values"], dtype=np.float64),
                np.asarray(g["timestamps"], dtype=np.float64))


def _intent_hold_mask(state: np.ndarray, ts: np.ndarray,
                      cmd: np.ndarray, cmd_ts: np.ndarray,
                      arm_mask: np.ndarray | None,
                      flat_v: float = 0.02, min_pause: float = 0.15,
                      intent_frac: float = 0.5) -> np.ndarray:
    """aligned 帧级意图停顿掩码（True=操作者有意停顿，repair 应保留其时长）。

    判据：state 全臂停住（每步 max|Δarm|/dt < flat_v）的停顿段里，cmd 对齐后同步也停
    （窗口内 cmd 平段占比 >= intent_frac）→ 意图型（保留）；cmd 在动而 state 停 → 摩擦
    型（不保留，照常压缩）。cmd 与 state 时间戳语义不同（~200ms 恒偏移），先经
    _cross_lag（带方差/相关度守卫）对齐。cmd 值在 state 时刻 t 处取 cmd_ts ≈ t + lag。
    """
    st = state[:, arm_mask] if arm_mask is not None else state
    n = len(st)
    mask = np.zeros(n, dtype=bool)
    if n < 4 or cmd is None or len(cmd) < 4 or len(ts) < 4:
        return mask
    lag = _cross_lag(st.mean(axis=1)[:, None], ts,
                     cmd.mean(axis=1)[:, None], cmd_ts)
    c_idx = np.clip(np.searchsorted(cmd_ts, ts + lag, side="right") - 1,
                    0, len(cmd_ts) - 1)
    c_arm = cmd[c_idx]   # cmd 流只有臂关节维，无需 arm_mask（无 ee/waist 污染）
    dt = np.maximum(np.diff(ts), 1e-6)
    cflat = np.abs(np.diff(c_arm, axis=0)).max(axis=1) / dt < flat_v
    sflat = np.abs(np.diff(st, axis=0)).max(axis=1) / dt < flat_v
    i, L = 0, len(sflat)
    while i < L:
        if sflat[i]:
            j = i
            while j < L and sflat[j]:
                j += 1
            if ts[j] - ts[i] >= min_pause:
                frac = float(cflat[i:j].mean()) if j > i else 0.0
                if frac >= intent_frac:
                    mask[i:j + 1] = True
            i = j
        else:
            i += 1
    return mask


def resample_frames(state: np.ndarray, quality: np.ndarray,
                    cam_images: dict, cam_offsets: dict, cam_ts: dict,
                    speed_ref: float, fps: int, state_ts0: float = 0.0,
                    arm_mask: np.ndarray | None = None,
                    intent_hold_mask: np.ndarray | None = None):
    """弧长均匀重采样（**选帧**，零错位）：按累计运动量均匀取**原始帧**。

    不用关节插值——插值必然造成"关节 vs 图像"错位（关节值在弧长空间插值、图像只能
    取离散原始帧，两者最多差半帧弧长）。这里每个重采样点取"弧长最近的原始帧"，
    state/action 与图像**严格同源**（同一原始帧），零错位；同时静止段弧长≈0 被压缩、
    帧间位移≈speed_ref（速度近似均匀）。代价：快慢不均（快速帧是原始的单大步、慢速
    帧原始离散），但绝无 state-图像错位、也无伪影重复帧（连续重复已去重）。

    弧长只算 arm_mask 的臂维（默认全维兼容）。speed_ref<=0 或 S_total<=0 → 返回 None
    （纯静止/退化 episode，调用方原样复制）。

    intent_hold_mask（--keep-intent）：操作者**有意停顿**帧（cmd 同步停住）——这些帧
    的"时长"必须保留（模型要学"在这里停住"），所以度量里每帧推进 speed_ref（= 输出
    一帧），停顿帧数 1:1 进输出；摩擦型停顿（cmd 在动、state 被静摩擦卡住）不在此
    掩码内，弧长≈0 照常被压缩。去重时豁免意图停顿帧（其连续重复 = 时长的正确表示）。
    """
    st = state[:, arm_mask] if arm_mask is not None else state
    d = np.abs(np.diff(st, axis=0)).sum(axis=1)          # (N-1,) 臂维每帧弧长
    if intent_hold_mask is not None and speed_ref > 0:
        # 意图停顿段内的步（i→i+1 都在掩码内）推进 speed_ref，保留其时长
        inside = intent_hold_mask[1:] & intent_hold_mask[:-1]
        d = np.where(inside, speed_ref, d)
    S = np.concatenate([[0.0], np.cumsum(d)])               # (N,) 累计弧长
    S_total = S[-1]
    if speed_ref <= 0 or S_total <= 0:
        return None
    N_new = max(2, int(round(S_total / speed_ref)))
    s_grid = np.linspace(0.0, S_total, N_new)
    # 弧长最近原始帧（argmin；精确对齐到原始帧，零错位）
    idx = np.array([int(np.argmin(np.abs(S - g))) for g in s_grid])
    # 去重：快速帧弧长跨度 > 网格间距时，多个网格点就近映射到同一原始帧 → 连续重复帧。
    # 这是纯采样伪影（状态/图像全同、无信息），留着会在每个快跳后制造假"停顿"帧
    # （实测默认均值速度下 30% 帧重复）。去重只删无信息副本，零错位不变。
    # 意图停顿帧豁免：其连续重复是"停顿时长"的正确表示，不删。
    if len(idx) > 1:
        keep = np.concatenate([[True], idx[1:] != idx[:-1]])
        if intent_hold_mask is not None:
            keep = keep | intent_hold_mask[idx]
        idx = idx[keep]
    N_new = len(idx)

    state_new = state[idx]                                  # 原始帧值（非插值）
    quality_new = quality[idx]
    cam_new = {c: cam_images[c][idx] for c in cam_images}
    off_new = {c: cam_offsets[c][idx] for c in cam_offsets}
    ts_cam_new = {c: cam_ts[c][idx] for c in cam_ts}

    # 匀速时间戳（帧间隔 = 1/fps 秒；严格单调）
    timestamps_new = float(state_ts0) + np.arange(N_new) / fps
    # next-state action
    action_new = np.vstack([state_new[1:], state_new[-1]])

    return state_new, action_new, quality_new, timestamps_new, idx, cam_new, off_new, ts_cam_new


def repair_episode(ep_dir: str, out_ep_dir: str,
                   speed_ref: float, fps: int, dry_run: bool,
                   keep_intent: bool = False) -> dict:
    src = os.path.join(ep_dir, "aligned_data.h5")
    if not os.path.exists(src):
        return {"episode": os.path.basename(ep_dir), "error": "无 aligned_data.h5"}
    if not dry_run:
        os.makedirs(out_ep_dir, exist_ok=True)

    with h5py.File(src, "r") as f:
        state = np.asarray(f["observation/state"], dtype=np.float64)
        quality = np.asarray(f["quality"], dtype=np.uint8)
        ts_all = np.asarray(f["timestamps"], dtype=np.float64)
        state_ts0 = float(ts_all[0])
        cams = _camera_groups(f)
        cam_images = {c: np.asarray(f[f"{c}/images"]) for c in cams}
        cam_offsets = {c: np.asarray(f[f"{c}/src_offsets"]) if f"{c}/src_offsets" in f[c] else np.zeros(len(cam_images[c]), dtype=np.int64) for c in cams}
        cam_ts = {c: np.asarray(f[f"{c}/src_timestamps"]) if f"{c}/src_timestamps" in f[c] else np.zeros(len(cam_images[c])) for c in cams}

    arm_mask = arm_mask_from_meta(ep_dir, state.shape[1])  # 弧长只用臂维

    # --keep-intent：用 raw cmd 分型意图停顿（cmd 同步停住=有意停，保留时长）；无 cmd 流则跳过
    intent_mask = None
    if keep_intent:
        arm_name = None
        try:
            with open(os.path.join(ep_dir, "meta.json"), encoding="utf-8") as mf:
                for b in json.load(mf)["schema"]["state_blocks"]:
                    if "arm" in b["name"]:
                        arm_name = b["name"]
                        break
        except Exception:  # noqa: BLE001
            pass
        cmd = _load_cmd(ep_dir, arm_name)
        if cmd is None:
            print(f"  {os.path.basename(ep_dir)}: --keep-intent 但无 *_cmd 流，跳过意图保留",
                  file=sys.stderr)
        else:
            intent_mask = _intent_hold_mask(state, ts_all, cmd[0], cmd[1], arm_mask)

    r = resample_frames(state, quality, cam_images, cam_offsets, cam_ts,
                        speed_ref, fps, state_ts0, arm_mask, intent_mask)
    if r is None:  # 纯静止/退化 episode：无法重采样，原样复制
        if not dry_run:
            for fname in ("aligned_data.h5", "robot_data.h5", "camera_data.h5", "meta.json"):
                p = os.path.join(ep_dir, fname)
                if os.path.exists(p):
                    shutil.copy2(p, os.path.join(out_ep_dir, fname))
        return {"episode": os.path.basename(ep_dir), "frames": len(state),
                "kept": len(state), "pct": 0.0}
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


def session_speed_ref(session: str, eps: list, arm_mask: np.ndarray | None) -> float:
    """session 全局 speed_ref：所有 episode 臂维每帧弧长合并后的**均值**。

    选均值而非 p90：均值 = 总弧长/总帧数，重采样后总帧数≈原始（总时长不变），
    只把静止/停滞帧的弧长匀到运动段——丝滑但**不加速**；旧默认 p90（≈1.9x 均值）
    会把整段数据提速、放大每帧跳变（实测每帧最大跳变 p50 0.019→0.043、
    >0.05 rad 占比 16%→40%）。各段统一速度尺度，避免模型学到不一致的节奏。
    """
    ds = []
    for ep in eps:
        p = os.path.join(session, ep, "aligned_data.h5")
        if not os.path.exists(p):
            continue
        try:
            with h5py.File(p, "r") as f:
                st = np.asarray(f["observation/state"], dtype=np.float64)
                if arm_mask is not None:
                    st = st[:, arm_mask]
            if len(st) < 3:
                continue
            d = np.abs(np.diff(st, axis=0)).sum(axis=1)
            ds.append(d)
        except Exception:  # noqa: BLE001
            continue
    if not ds:
        return 0.0
    all_d = np.concatenate(ds)
    return float(all_d.mean())


def _self_test() -> int:
    """合成数据自测意图感知重采样：意图停顿保留时长、摩擦停顿压缩、零错位保持。"""
    hz, T = 30.0, 5.0
    n = int(T * hz)
    t = np.arange(n) / hz
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}")
        ok = ok and cond

    def longest_run(vals):
        best = cur = 1
        for i in range(1, len(vals)):
            if abs(vals[i] - vals[i - 1]) < 1e-9:
                cur += 1
                best = max(best, cur)
            else:
                cur = 1
        return best

    vel = 0.04                                   # 每步位移 rad/步（≈1.2 rad/s，>> flat_v=0.02）
    cmd = vel * np.arange(n)                     # 匀速斜坡（每步 +vel，弧长=vel=speed_ref）
    cmd[(t >= 1.0) & (t < 1.5)] = cmd[int(1.0 * hz)]      # 意图停顿：cmd 也停 0.5s
    state = cmd.copy()
    i0, i1 = int(3.0 * hz), int(3.6 * hz)                 # 摩擦停顿：cmd 动、state 卡 0.6s
    state[i0:i1] = state[i0]
    st8 = np.stack([state, np.zeros(n)], axis=1)          # 8 维（1 臂关节 + 1 夹爪恒 0）
    arm = np.array([True, False])
    speed_ref = vel

    # 分型：意图段 1.0-1.5s 应标 True，摩擦段 3.0-3.6s 不应标 True
    intent = _intent_hold_mask(st8, t, cmd[:, None], t, arm, flat_v=0.02, min_pause=0.1)
    check("意图停顿被识别", intent[int(1.0 * hz):int(1.5 * hz)].mean() > 0.8,
          f"意图段覆盖={intent[int(1.0*hz):int(1.5*hz)].mean():.2f}")
    check("摩擦停顿不当意图", intent[int(3.0 * hz):int(3.6 * hz)].mean() < 0.2,
          f"摩擦段覆盖={intent[int(3.0*hz):int(3.6*hz)].mean():.2f}")

    # keep-intent：意图停顿保留 ~0.5s*hz=15 帧恒值段；摩擦段被压
    r = resample_frames(st8, np.zeros(n, np.uint8), {}, {}, {},
                        speed_ref, hz, 0.0, arm, intent)
    stn = r[0]
    check("意图停顿保留时长(~15帧)", longest_run(stn[:, 0]) >= 12,
          f"最长恒值run={longest_run(stn[:, 0])} 帧")
    check("摩擦停顿被压缩(帧数减少)", len(stn) < n * 0.98, f"{len(stn)}/{n}")
    z = all((np.abs(st8 - row).max(axis=1) <= 1e-9).any() for row in stn)
    check("零错位(state∈原始帧)", z)

    # 对照：无意图感知 → 意图段也被压（最长恒值 run 显著变短、输出更短）
    r2 = resample_frames(st8, np.zeros(n, np.uint8), {}, {}, {},
                         speed_ref, hz, 0.0, arm, None)
    st2 = r2[0]
    check("对照：无 keep-intent → 停顿也被压", longest_run(st2[:, 0]) < 6,
          f"最长恒值run={longest_run(st2[:, 0])}")
    check("对照：输出更短（意图段也被压）", len(st2) < len(stn), f"{len(st2)}/{len(stn)}")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", help="含 episode*/aligned_data.h5 的 session")
    ap.add_argument("--out-session", help="输出新 session 目录")
    ap.add_argument("--speed-ref", type=float, default=0.0,
                    help="目标每帧弧长 rad/帧（0=自动取 session 臂维弧长均值，保留总时长不加速；想更快给 p70~p90 量级，如 0.07）")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--keep-intent", action="store_true",
                    help="意图感知：用 raw *_cmd 流分型停顿，操作者有意停顿（cmd 同步停）保留其"
                         "时长，只压缩摩擦型停顿（cmd 在动 state 被静摩擦卡住）")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--self-test", action="store_true", help="合成数据自测")
    args = ap.parse_args()

    if args.self_test:
        return _self_test()
    if not args.session:
        ap.error("--session 必填（--self-test 除外）")
    if not args.dry_run and not args.out_session:
        ap.error("--out-session 必填（--dry-run 只需 --session）")

    if not os.path.isdir(args.session):
        print(f"session 不存在: {args.session}", file=sys.stderr)
        return 1
    eps = sorted(d for d in os.listdir(args.session)
                 if d.startswith("episode") and os.path.isdir(os.path.join(args.session, d)))
    if not eps:
        print("无 episode", file=sys.stderr)
        return 1
    # 臂维掩码（弧长只用臂关节；无 meta 则全维）
    arm_mask = None
    for ep0 in eps:
        ap = os.path.join(args.session, ep0, "aligned_data.h5")
        if os.path.exists(ap):
            with h5py.File(ap) as f:
                n_dim = f["observation/state"].shape[1]
            arm_mask = arm_mask_from_meta(os.path.join(args.session, ep0), n_dim)
            break
    # 全局 speed_ref（若未显式指定）：所有 episode 臂维弧长均值，统一速度尺度、不加速
    if args.speed_ref <= 0:
        args.speed_ref = session_speed_ref(args.session, eps, arm_mask)
    out_root = args.session if args.dry_run else args.out_session
    if not args.dry_run:
        os.makedirs(out_root, exist_ok=True)

    print(f"修复: 弧长匀速化选帧 speed_ref={'自动(臂维均值=%.4f)' % args.speed_ref if args.speed_ref > 0 else args.speed_ref} "
          f"{'意图感知(保留有意停顿)' if args.keep_intent else ''} "
          f"{'[DRY-RUN]' if args.dry_run else ''}")
    tot0 = tot1 = 0
    for ep in eps:
        st = repair_episode(os.path.join(args.session, ep), os.path.join(out_root, ep),
                            args.speed_ref, args.fps, args.dry_run, args.keep_intent)
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
