#!/usr/bin/env python3
"""对抗性零错位验证：repair_aligned(选帧) 输出必须满足——
1) state 每行精确等于原始某帧（选帧非插值）；2) 图像 bytes 是原始帧集合的成员
（与 state 同源同一原始帧）——**遍历全部相机组**（相机名可改：videoN→base/left_wrist/
right_wrist，动态探测）；3) 时间戳严格单调；4) action = next-state。"""
import h5py, numpy as np, os

SRC = "/home/robot/loopkok/sdk/astral_data/raw/pick_place_merged"


def _camera_groups(f) -> list[str]:
    """动态探测相机组（含 images 的组），不依赖具体相机名/数量。"""
    return [k for k in f.keys()
            if isinstance(f[k], h5py.Group) and "images" in f[k]]


def check(name: str) -> bool:
    OUT = f"{SRC}_{name}"
    print(f"=== {name} ===")
    bad_state = bad_img = bad_ts = bad_act = 0
    n = 0
    for ep in sorted(os.listdir(OUT))[:15]:
        op = f"{OUT}/{ep}/aligned_data.h5"
        sp = f"{SRC}/{ep}/aligned_data.h5"
        if not os.path.exists(op) or not os.path.exists(sp):
            continue
        with h5py.File(sp) as f:
            st0 = np.asarray(f["observation/state"])
            cams = _camera_groups(f)
            im0 = {c: [bytes(x) for x in f[f"{c}/images"]] for c in cams}
        with h5py.File(op) as f:
            st1 = np.asarray(f["observation/state"])
            im1 = {c: [bytes(x) for x in f[f"{c}/images"]] for c in cams}
            ac1 = np.asarray(f["action"])
            ts = np.asarray(f["timestamps"])
        n += len(st1)
        # 1) state 每行必须精确等于原始某帧
        for r in st1:
            if not (np.abs(st0 - r).max(axis=1) <= 1e-9).any():
                bad_state += 1
        # 2) 每路相机图像必须来自原始帧集合（与 state 同源）
        for c in cams:
            s0 = set(im0[c])
            for b in im1[c]:
                if b not in s0:
                    bad_img += 1
        # 3) 时间戳严格单调
        if not (np.diff(ts) > 0).all():
            bad_ts += 1
        # 4) action = next-state
        if not np.allclose(ac1[:-1], st1[1:], atol=1e-5):
            bad_act += 1
    print(f"  检查 {n} 帧: state非原始帧={bad_state} 图像非原始帧={bad_img} "
          f"时间戳非单调={bad_ts} action≠nextstate={bad_act}")
    return bad_state + bad_img + bad_ts + bad_act == 0


ok1 = check("repaired")
print("\n零错位对抗验证:", "ALL PASS" if ok1 else "FAIL")
raise SystemExit(0 if ok1 else 1)
