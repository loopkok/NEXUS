"""清洗校验：逐 episode 体检，产出报告并可隔离问题段。

规则（级别：fail=必须处理，warn=人工过目）：
  F1 fail  文件缺失/不可读（robot_data.h5 / camera_data.h5 / meta.json）
  F2 fail  schema 要求的流缺失或为空（配置与硬件不一致）
  F3 fail  数值流含 NaN/Inf
  F4 fail  时间戳非严格递增
  F5 fail  episode 时长 < min_duration_s（默认 2s）
  W1 warn  流实测频率低于期望下限（arm 45Hz / hand 100Hz / gripper 5Hz / 图像 0.8*fps）
  W2 warn  采样空洞 > max_gap_ms（按次数报告）
  W3 warn  关节相邻帧跳变 |Δ| > 0.5 rad（遥操限速 4rad/s@30fps → 0.5 为硬异常）
  W4 warn  图像帧 JPEG 解码失败（首/中/尾抽检）
  W5 warn  armed 覆盖率 < 50%（按事件时间线推算；无事件则跳过）
  W6 warn  对齐帧图像时间偏差 p95 > 1.5/fps（需已跑 align）

用法：
  python3 validate_data.py --session ~/astral_data/pick_place [--apply]
  --apply：把 fail 的 episode 移入 session/quarantine/（移动，不删除）
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from typing import Any

import h5py
import numpy as np

from astral_data_collect.align_data import ALIGNED_H5
from astral_data_collect.data_writer import CAMERA_H5, META_JSON, ROBOT_H5
from astral_data_collect.schema import CollectSchema

QUARANTINE_DIR = "quarantine"
REPORT_JSON = "validation_report.json"

# 各流实测频率下限（Hz）——低于则 W1
EXPECTED_MIN_HZ = {
    "body_state": 45.0,
    "left_arm_state": 45.0, "right_arm_state": 45.0,
    "left_arm_cmd": 45.0, "right_arm_cmd": 45.0,
    "left_hand_state": 100.0, "right_hand_state": 100.0,
    "left_hand_cmd": 100.0, "right_hand_cmd": 100.0,
    "left_gripper_ratio": 5.0, "right_gripper_ratio": 5.0,
    "left_gripper_rad": 5.0, "right_gripper_rad": 5.0,
    "head_state": 20.0, "head_cmd": 20.0,
}
MAX_JUMP_RAD = 0.5
MIN_DURATION_S = 2.0


class _Report:
    def __init__(self, episode_dir: str) -> None:
        self.episode_dir = episode_dir
        self.issues: list[dict[str, str]] = []

    def fail(self, rule: str, msg: str) -> None:
        self.issues.append({"level": "fail", "rule": rule, "msg": msg})

    def warn(self, rule: str, msg: str) -> None:
        self.issues.append({"level": "warn", "rule": rule, "msg": msg})

    @property
    def status(self) -> str:
        levels = {i["level"] for i in self.issues}
        return "fail" if "fail" in levels else ("warn" if "warn" in levels else "pass")

    def to_dict(self) -> dict[str, Any]:
        return {
            "episode": os.path.basename(self.episode_dir),
            "status": self.status,
            "issues": self.issues,
        }


def _stream_duration(ts: np.ndarray) -> float:
    return float(ts[-1] - ts[0]) if len(ts) > 1 else 0.0


def _check_stream(rep: _Report, name: str, ts: np.ndarray, values: np.ndarray,
                  max_gap_s: float) -> None:
    if len(ts) == 0:
        rep.fail("F2", f"stream {name} empty")
        return
    if np.any(~np.isfinite(values)):
        rep.fail("F3", f"stream {name} contains NaN/Inf")
    if len(ts) > 1 and np.any(np.diff(ts) <= 0):
        rep.fail("F4", f"stream {name} timestamps not strictly increasing")
    dur = _stream_duration(ts)
    if dur > 0.5:
        hz = len(ts) / dur
        low = EXPECTED_MIN_HZ.get(name)
        if low is not None and hz < low:
            rep.warn("W1", f"stream {name} rate {hz:.1f}Hz < {low}Hz")
        if len(ts) > 1:
            gaps = np.diff(ts)
            n_gap = int(np.count_nonzero(gaps > max_gap_s))
            if n_gap:
                rep.warn(
                    "W2",
                    f"stream {name}: {n_gap} gaps > {max_gap_s * 1000:.0f}ms "
                    f"(max {gaps.max() * 1000:.0f}ms)",
                )
        if len(ts) > 2 and values.ndim == 2 and values.shape[1] > 0:
            jump = np.max(np.abs(np.diff(values, axis=0)))
            if np.isfinite(jump) and jump > MAX_JUMP_RAD:
                rep.warn("W3", f"stream {name} max frame jump {jump:.2f} rad")


def _armed_coverage(events: list[dict], t0: float, t1: float) -> float | None:
    """从事件时间线推 teleop_armed 覆盖比例；无事件返回 None。"""
    ev = sorted(
        (e for e in events if e.get("name") in ("teleop_armed", "teleop_disarmed")),
        key=lambda e: e["t"],
    )
    if not ev:
        return None
    armed = False
    last = t0
    acc = 0.0
    for e in ev:
        t = min(max(float(e["t"]), t0), t1)
        if armed:
            acc += t - last
        armed = bool(e["data"]) if e["name"] == "teleop_armed" else not bool(e["data"])
        # teleop_armed False / teleop_disarmed True 都表示撤防
        if e["name"] == "teleop_armed":
            armed = bool(e["data"])
        else:
            armed = not bool(e["data"])
        last = t
    if armed:
        acc += t1 - last
    total = max(t1 - t0, 1e-9)
    return acc / total


def validate_episode(
    episode_dir: str,
    *,
    min_duration_s: float = MIN_DURATION_S,
    check_aligned: bool = True,
) -> _Report:
    rep = _Report(episode_dir)

    robot_h5 = os.path.join(episode_dir, ROBOT_H5)
    camera_h5 = os.path.join(episode_dir, CAMERA_H5)
    meta_json = os.path.join(episode_dir, META_JSON)
    for p in (robot_h5, camera_h5, meta_json):
        if not os.path.exists(p):
            rep.fail("F1", f"missing {os.path.basename(p)}")
    if any(i["rule"] == "F1" for i in rep.issues):
        return rep

    try:
        with open(meta_json, encoding="utf-8") as f:
            meta = json.load(f)
        schema = CollectSchema.from_dict(meta["schema"])
    except Exception as exc:
        rep.fail("F1", f"meta.json unreadable: {exc}")
        return rep

    max_gap_s = float(schema.max_gap_ms) / 1000.0

    try:
        with h5py.File(robot_h5, "r") as fr, h5py.File(camera_h5, "r") as fc:
            # -- 数值流 ------------------------------------------------------------
            for name, dim in schema.required_streams().items():
                if name not in fr["streams"]:
                    rep.fail("F2", f"stream {name} missing")
                    continue
                ts = fr[f"streams/{name}/timestamps"][:]
                values = fr[f"streams/{name}/values"][:]
                if values.ndim == 2 and values.shape[1] != dim and len(ts) > 0:
                    rep.warn("W2", f"stream {name} dim {values.shape[1]} != schema {dim}")
                _check_stream(rep, name, ts, values, max_gap_s)

            # -- 相机流 ------------------------------------------------------------
            for cam in schema.cameras:
                if cam not in fc or len(fc[f"{cam}/timestamps"]) == 0:
                    rep.fail("F2", f"camera {cam} missing/empty")
                    continue
                ts = fc[f"{cam}/timestamps"][:]
                if len(ts) > 1 and np.any(np.diff(ts) <= 0):
                    rep.fail("F4", f"camera {cam} timestamps not strictly increasing")
                dur = _stream_duration(ts)
                if dur > 0.5:
                    hz = len(ts) / dur
                    if hz < 0.8 * schema.dataset_fps:
                        rep.warn(
                            "W1",
                            f"camera {cam} rate {hz:.1f}Hz < {0.8 * schema.dataset_fps:.1f}Hz",
                        )
                # JPEG 解码抽检（首/中/尾）
                try:
                    import cv2

                    n = len(ts)
                    for i in {0, n // 2, n - 1}:
                        raw = fc[f"{cam}/images"][int(i)]
                        img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                        if img is None:
                            rep.warn("W4", f"camera {cam} frame {i} JPEG undecodable")
                except ImportError:
                    pass  # 无 cv2 时跳过抽检

            # -- 时长 ---------------------------------------------------------------
            dur = float(meta.get("duration_s", 0.0))
            if dur < min_duration_s:
                rep.fail("F5", f"duration {dur:.2f}s < {min_duration_s}s")

            # -- armed 覆盖率 ---------------------------------------------------------
            cov = _armed_coverage(
                meta.get("events", []),
                float(meta.get("start_time_wall", 0.0)),
                float(meta.get("end_time_wall", 0.0)),
            )
            if cov is not None and cov < 0.5:
                rep.warn("W5", f"teleop armed coverage {cov * 100:.0f}% < 50%")

        # -- 对齐产物（可选） ---------------------------------------------------------
        aligned = os.path.join(episode_dir, ALIGNED_H5)
        if check_aligned and os.path.exists(aligned):
            with h5py.File(aligned, "r") as fa:
                state = fa["observation/state"][:]
                if np.any(~np.isfinite(state)):
                    missing = json.loads(fa.attrs.get("missing_blocks", "[]"))
                    if missing:
                        rep.fail("F3", f"aligned state NaN (missing blocks: {missing})")
                    else:
                        rep.fail("F3", "aligned state contains NaN/Inf")
                fps = float(fa.attrs.get("fps", schema.dataset_fps))
                for cam in schema.cameras:
                    key = f"{cam}/src_offsets"
                    if key in fa:
                        off = np.abs(fa[key][:])
                        if len(off):
                            p95 = float(np.percentile(off, 95))
                            if p95 > 1.5 / fps:
                                rep.warn(
                                    "W6",
                                    f"camera {cam} align offset p95 {p95 * 1000:.1f}ms "
                                    f"> {1.5 / fps * 1000:.1f}ms",
                                )
    except Exception as exc:
        rep.fail("F1", f"h5 read error: {exc}")
    return rep


def validate_session(
    session_dir: str,
    *,
    apply: bool = False,
    min_duration_s: float = MIN_DURATION_S,
    log=print,
) -> dict[str, Any]:
    session_dir = os.path.abspath(session_dir)
    episode_dirs = sorted(
        os.path.join(session_dir, d)
        for d in os.listdir(session_dir)
        if d.startswith("episode")
        and os.path.isdir(os.path.join(session_dir, d))
    )
    reports = []
    for ep in episode_dirs:
        rep = validate_episode(ep, min_duration_s=min_duration_s)
        reports.append(rep)
        line = f"  [{rep.status.upper():4s}] {os.path.basename(ep)}"
        if rep.issues:
            line += ": " + "; ".join(
                f"{i['rule']} {i['msg']}" for i in rep.issues[:4]
            )
            if len(rep.issues) > 4:
                line += f" … (+{len(rep.issues) - 4})"
        log(line)

    n_fail = sum(1 for r in reports if r.status == "fail")
    n_warn = sum(1 for r in reports if r.status == "warn")
    n_pass = sum(1 for r in reports if r.status == "pass")

    if apply:
        qdir = os.path.join(session_dir, QUARANTINE_DIR)
        for rep in reports:
            if rep.status != "fail":
                continue
            os.makedirs(qdir, exist_ok=True)
            dst = os.path.join(qdir, os.path.basename(rep.episode_dir))
            shutil.move(rep.episode_dir, dst)
            log(f"  quarantined -> {dst}")

    summary = {
        "session": session_dir,
        "total": len(reports),
        "pass": n_pass,
        "warn": n_warn,
        "fail": n_fail,
        "quarantined": n_fail if apply else 0,
        "episodes": [r.to_dict() for r in reports],
    }
    with open(os.path.join(session_dir, REPORT_JSON), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    log(
        f"summary: {n_pass} pass / {n_warn} warn / {n_fail} fail"
        + (f" (fail moved to {QUARANTINE_DIR}/)" if apply else "")
        + f" — report: {os.path.join(session_dir, REPORT_JSON)}"
    )
    return summary


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--session", required=True, help="session 目录（含 episode*）")
    ap.add_argument("--apply", action="store_true",
                    help="把 fail 的 episode 移入 quarantine/（移动不删除）")
    ap.add_argument("--min-duration", type=float, default=MIN_DURATION_S)
    args = ap.parse_args(argv)
    summary = validate_session(
        os.path.expanduser(args.session),
        apply=args.apply,
        min_duration_s=args.min_duration,
    )
    sys.exit(1 if summary["fail"] and not args.apply else 0)


if __name__ == "__main__":
    main(sys.argv[1:])
