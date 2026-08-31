#!/usr/bin/env python3
"""Offline tests for geometric DH-free S/E/W arm-angle IK (``ik/geometric.py``).

Ground truth: Pinocchio FK via the production ``URDFNumericalIKSolver``.
The geometric solver must match URDF FK to float precision (no DH fit error)
and reproduce seed branches on FK->IK round-trips.

Usage:
  python3 -m astral_arm_teleop.test_geometric_ik
  ros2 run astral_arm_teleop test_geometric_ik
"""

from __future__ import annotations

import sys
import time

import numpy as np

from astral_arm_teleop.ik.factory import (
    default_astral_urdf_path,
    make_ik_solver,
    make_single_arm_ik,
)
from astral_arm_teleop.ik.geometric import GeometricIKSolver
from astral_arm_teleop.ik.urdf_solver import URDFNumericalIKSolver

ARMS = ("L", "R")


def _wrap(x: np.ndarray) -> np.ndarray:
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def _sample_q(rng, lower, upper, n, margin=0.05):
    """Random configs clear of generic SRS singularities.

    (shoulder PK2 double root at |q2|=pi/2, straight elbow q4=0, wrist q6=0 —
    no 7-DOF arm-angle solver is asserted exact at those.)
    """
    out = []
    lo = lower + margin
    hi = upper - margin
    while len(out) < n:
        q = rng.uniform(lo, hi)
        if abs(abs(q[1]) - 0.5 * np.pi) < 0.25:
            continue
        if q[3] > -0.30:
            continue
        if abs(q[5]) < 0.15:
            continue
        out.append(q)
    return out


class _GroundTruth:
    def __init__(self):
        self.solver = URDFNumericalIKSolver(urdf_path=default_astral_urdf_path())

    def fk(self, arm: str, q: np.ndarray) -> np.ndarray:
        return self.solver.fk_homogeneous(arm, q)


def _make(arm: str) -> GeometricIKSolver:
    return GeometricIKSolver("left" if arm == "L" else "right")


def test_fk_matches_urdf(gt: _GroundTruth) -> bool:
    print("\n  --- FK vs Pinocchio URDF (ground truth) ---")
    rng = np.random.default_rng(42)
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        max_pos = 0.0
        max_rot = 0.0
        for _ in range(100):
            q = rng.uniform(solver.lower_limits, solver.upper_limits)
            Tg = solver.fk(q)
            Tu = gt.fk(arm, q)
            max_pos = max(max_pos, float(np.linalg.norm(Tg[:3, 3] - Tu[:3, 3])))
            max_rot = max(max_rot, float(np.linalg.norm(Tg[:3, :3] - Tu[:3, :3])))
        ok = max_pos < 1e-8 and max_rot < 1e-8
        ok_all &= ok
        print(
            f"  arm {arm}: max pos err {max_pos * 1e9:.1f} nm, "
            f"max rot err {max_rot:.2e} -> {'PASS' if ok else 'FAIL'}"
        )
    return ok_all


def test_fk_ik_roundtrip() -> bool:
    print("\n  --- FK -> IK round-trip (both arms) ---")
    rng = np.random.default_rng(7)
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        seeds = _sample_q(rng, solver.lower_limits, solver.upper_limits, 8)
        fails = 0
        worst_pos_mm = 0.0
        worst_dq = 0.0
        for q_seed in seeds:
            solver.sync_state(q_seed)
            T = solver.fk(q_seed)
            sol = solver.solve(T)
            if sol is None:
                print(f"  arm {arm}: FAIL no solution for seed {np.round(q_seed, 2)}")
                fails += 1
                continue
            Terr = solver.fk(sol)
            pos_mm = float(np.linalg.norm(Terr[:3, 3] - T[:3, 3]) * 1000.0)
            rot_err = float(np.linalg.norm(Terr[:3, :3] - T[:3, :3]))
            dq = float(np.max(np.abs(_wrap(sol - q_seed))))
            worst_pos_mm = max(worst_pos_mm, pos_mm)
            worst_dq = max(worst_dq, dq)
            if pos_mm > 1.0 or rot_err > 0.02 or dq > 0.2:
                print(
                    f"  arm {arm}: FAIL seed {np.round(q_seed, 2)} "
                    f"pos={pos_mm:.3f}mm rot={rot_err:.4f} dq={dq:.3f}"
                )
                fails += 1
        ok = fails == 0
        ok_all &= ok
        print(
            f"  arm {arm}: {len(seeds)} seeds, worst pos {worst_pos_mm:.4f} mm, "
            f"worst |dq| {worst_dq:.4f} rad -> {'PASS' if ok else 'FAIL'}"
        )
    return ok_all


def test_solutions_within_limits() -> bool:
    print("\n  --- Solutions respect joint limits ---")
    rng = np.random.default_rng(11)
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        lo, hi = solver.lower_limits, solver.upper_limits
        seeds = _sample_q(rng, lo, hi, 5)
        ok = True
        for q_seed in seeds:
            solver.sync_state(q_seed)
            sol = solver.solve(solver.fk(q_seed))
            if sol is None:
                continue
            if not (np.all(sol >= lo - 1e-9) and np.all(sol <= hi + 1e-9)):
                print(f"  arm {arm}: FAIL out of limits {np.round(sol, 3)}")
                ok = False
        ok_all &= ok
        print(f"  arm {arm}: {'PASS' if ok else 'FAIL'}")
    return ok_all


def test_unreachable_returns_none() -> bool:
    print("\n  --- Unreachable target returns None ---")
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        solver.sync_state(np.zeros(7))
        T = np.eye(4)
        T[:3, 3] = np.array([1.5, 0.0, 0.0])  # reach ~0.48 m
        sol = solver.solve(T)
        ok = sol is None
        ok_all &= ok
        print(f"  arm {arm}: {'PASS' if ok else 'FAIL (returned a pose)'}")
    return ok_all


def test_smooth_trajectory() -> bool:
    print("\n  --- Smooth trajectory (continuity + timing) ---")
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        q0 = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00])
        q0 = np.clip(q0, solver.lower_limits + 0.02, solver.upper_limits - 0.02)
        solver.sync_state(q0)
        T0 = solver.fk(q0)
        center = T0[:3, 3].copy()
        base_rot = T0[:3, :3].copy()
        n = 60
        fails = 0
        times = []
        max_jump = 0.0
        q_prev = q0.copy()
        for step in range(n):
            ang = 2 * np.pi * step / n
            T = np.eye(4)
            T[:3, :3] = base_rot
            T[:3, 3] = center + 0.03 * np.array([np.cos(ang), np.sin(ang), 0.0])
            t0 = time.perf_counter()
            sol = solver.solve(T)
            times.append((time.perf_counter() - t0) * 1000)
            if sol is None:
                fails += 1
                continue
            max_jump = max(max_jump, float(np.max(np.abs(_wrap(sol - q_prev)))))
            q_prev = sol.copy()
        rate = (n - fails) / n * 100
        mean_t = float(np.mean(times[1:])) if len(times) > 1 else 0.0
        ok = rate >= 95.0 and max_jump < 0.3 and mean_t < 5.0
        ok_all &= ok
        print(
            f"  arm {arm}: rate={rate:.1f}% max_jump={max_jump:.3f} rad "
            f"mean_t={mean_t:.2f} ms -> {'PASS' if ok else 'FAIL'}"
        )
    return ok_all


def test_dual_arm_factory(gt: _GroundTruth) -> bool:
    print("\n  --- Dual-arm factory bridge ---")
    bridge = make_ik_solver("geometric")
    ok = True
    for arm in ARMS:
        Tg = bridge.fk(arm, np.zeros(7))
        Tu = gt.fk(arm, np.zeros(7))
        if float(np.linalg.norm(Tg - Tu)) > 1e-8:
            print(f"  arm {arm}: FAIL fk mismatch vs URDF")
            ok = False
    TL = bridge.fk("L", np.zeros(7))
    TR = bridge.fk("R", np.zeros(7))
    print(f"  L EE0={np.round(TL[:3, 3], 3).tolist()} R EE0={np.round(TR[:3, 3], 3).tolist()}")
    if not (TL[0, 3] > 0.05 and TR[0, 3] < -0.05):
        print("  FAIL: base_link arm separation wrong")
        ok = False
    # solve through the bridge on a bent seed
    for arm in ARMS:
        q0 = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0])
        lo, hi = bridge.joint_limits(arm)
        q0 = np.clip(q0, lo + 0.02, hi - 0.02)
        q14 = bridge.q_full
        q14[0:7 if arm == "L" else None] = 0.0  # placeholder, replaced below
        bridge.sync_state(np.concatenate([q0, np.zeros(7)]) if arm == "L" else np.concatenate([np.zeros(7), q0]))
        T = bridge.fk(arm, q0)
        sol = bridge.solve(arm, T)
        if sol is None or float(np.linalg.norm(bridge.fk(arm, sol)[:3, 3] - T[:3, 3])) > 1e-3:
            print(f"  arm {arm}: FAIL bridge solve")
            ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_single_arm_factory() -> bool:
    print("\n  --- Single-arm factory (teleop node API) ---")
    ok = True
    for side in ("left", "right"):
        s = make_single_arm_ik(side, "geometric")
        for attr in ("solve", "fk", "sync_state", "lower_limits", "upper_limits", "method_name"):
            if not hasattr(s, attr):
                print(f"  {side}: FAIL missing {attr}")
                ok = False
        if "geometric" not in str(s.method_name):
            print(f"  {side}: FAIL method_name={s.method_name}")
            ok = False
        if np.asarray(s.lower_limits).shape != (7,) or np.asarray(s.upper_limits).shape != (7,):
            print(f"  {side}: FAIL limits shape")
            ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    print("Astral Teleop — Geometric (DH-free) IK Test Suite")
    gt = _GroundTruth()
    results = {
        "fk_matches_urdf": test_fk_matches_urdf(gt),
        "fk_ik_roundtrip": test_fk_ik_roundtrip(),
        "within_limits": test_solutions_within_limits(),
        "unreachable_none": test_unreachable_returns_none(),
        "smooth_trajectory": test_smooth_trajectory(),
        "dual_arm_factory": test_dual_arm_factory(gt),
        "single_arm_factory": test_single_arm_factory(),
    }
    print("\n  SUMMARY:")
    for k, v in results.items():
        print(f"    {k}: {'PASS' if v else 'FAIL'}")
    ok = all(results.values())
    print(f"FINAL: {'ALL PASSED' if ok else 'SOME FAILED'}")
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
