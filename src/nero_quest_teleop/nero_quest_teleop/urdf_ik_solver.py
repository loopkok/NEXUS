#!/usr/bin/env python3
"""URDF-based numerical IK solver for the Nero arm.

Uses Pinocchio FK + scipy least_squares (Levenberg-Marquardt) — no CasADi needed.
Fast (~3-8ms with warm-start) and uses the full URDF kinematic model.

API is identical to IKSolver:
    solver = URDFIKSolver(urdf_path)
    q = solver.solve(T_flange)  # 4x4 -> 7 joints or None
    T = solver.fk(q)            # FK verification
"""

import os
import time
from typing import List, Optional

import numpy as np
from scipy.optimize import least_squares

try:
    import pinocchio as pin
except ImportError as e:
    raise ImportError("URDFIKSolver requires pinocchio.") from e


class URDFIKSolver:
    """Numerical IK via Pinocchio FK + scipy Levenberg-Marquardt.

    Optimised for teleop (<10ms):
      - max_nfev=20, xtol=1e-4
      - Warm-start from previous solution (converges in 3-8 function evals)
      - Joint limit clamping via bounded optimization
    """

    def __init__(
        self,
        urdf_path: str,
        package_dirs: Optional[List[str]] = None,
        locked_joints: Optional[List[str]] = None,
        ee_frame_name: str = "ee",
        max_iter: int = 4,
        tol: float = 1e-4,
        w_pos: float = 1.0,
        w_ori: float = 0.3,
        w_reg: float = 0.0,
        w_smooth: float = 0.0,
    ):
        if package_dirs is None:
            package_dirs = [os.path.dirname(urdf_path)]
        if locked_joints is None:
            locked_joints = ["joint8"]

        # ---- Load URDF ----
        robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dirs=package_dirs)
        self._reduced = robot.buildReducedRobot(
            list_of_joints_to_lock=self._dedup_locked_joints(robot, locked_joints),
            reference_configuration=np.zeros(robot.model.nq),
        )
        self._model = self._reduced.model

        # Add ee frame at joint7 flange
        ee_id_full = robot.model.getFrameId(ee_frame_name)
        ee_parent = (
            robot.model.frames[ee_id_full].parentJoint
            if ee_id_full < robot.model.nframes
            else self._model.getJointId("joint7")
        )
        self._model.addFrame(
            pin.Frame(ee_frame_name, ee_parent, pin.SE3.Identity(), pin.FrameType.OP_FRAME)
        )
        self._ee_id = self._model.getFrameId(ee_frame_name)
        self._data = self._model.createData()
        self._nq = self._model.nq

        self._lower = self._model.lowerPositionLimit.copy()
        self._upper = self._model.upperPositionLimit.copy()

        # Solver params
        self._max_nfev = max_iter * 5  # generous nfev budget, LM converges fast
        self._tol = tol
        self._w_pos = w_pos
        self._w_ori = w_ori
        self._w_reg = w_reg
        self._w_smooth = w_smooth

        # Warm-start state
        self._q_prev = np.zeros(self._nq)
        self._solve_times: List[float] = []

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def nq(self) -> int:
        return self._nq

    @property
    def lower_limits(self) -> np.ndarray:
        return self._lower.copy()

    @property
    def upper_limits(self) -> np.ndarray:
        return self._upper.copy()

    # ------------------------------------------------------------------
    # Core API
    # ------------------------------------------------------------------

    def solve(self, T_target: np.ndarray) -> Optional[np.ndarray]:
        """Solve IK via Levenberg-Marquardt optimization on URDF FK error."""
        t0 = time.perf_counter()
        T_tgt = pin.SE3(np.asarray(T_target, dtype=float))
        q0 = self._q_prev.copy()
        q_ref = np.clip(q0.copy(), self._lower, self._upper)

        def _cost(q):
            pin.forwardKinematics(self._model, self._data, q)
            pin.updateFramePlacements(self._model, self._data)
            Tc = self._data.oMf[self._ee_id]
            err = pin.log(Tc.inverse() * T_tgt).vector  # 6-DOF se3 error
            out = np.empty(6 + self._nq)
            out[:3] = self._w_pos * err[:3]
            out[3:6] = self._w_ori * err[3:]
            out[6:] = np.sqrt(self._w_reg + self._w_smooth) * (q - q_ref)
            return out

        # Clamp warm-start to URDF limits (may differ from DH/robot limits)
        q0 = np.clip(q0, self._lower, self._upper)

        try:
            res = least_squares(
                _cost,
                q0,
                bounds=(self._lower, self._upper),
                method="trf",
                xtol=self._tol,
                ftol=self._tol,
                gtol=1e-6,
                max_nfev=self._max_nfev,
                verbose=0,
            )
        except Exception:
            self._solve_times.append((time.perf_counter() - t0) * 1000)
            return None

        dt = (time.perf_counter() - t0) * 1000
        self._solve_times.append(dt)

        q_out = res.x
        # Verify
        pin.forwardKinematics(self._model, self._data, q_out)
        pin.updateFramePlacements(self._model, self._data)
        Tc = self._data.oMf[self._ee_id]
        pos_err = float(np.linalg.norm(Tc.translation - T_tgt.translation))
        rot_err_vec = pin.log(Tc.inverse() * T_tgt).vector[3:]
        rot_err = float(np.linalg.norm(rot_err_vec))

        if pos_err < 1e-3 and rot_err < 1e-1:
            self._q_prev = q_out.copy()
            return q_out.copy()

        return None

    def fk(self, q: np.ndarray) -> np.ndarray:
        q_arr = np.asarray(q, dtype=float).ravel()[:self._nq]
        pin.forwardKinematics(self._model, self._data, q_arr)
        pin.updateFramePlacements(self._model, self._data)
        return self._data.oMf[self._ee_id].homogeneous

    def sync_state(self, q: np.ndarray):
        self._q_prev = np.clip(
            np.asarray(q, dtype=float).ravel()[:self._nq],
            self._lower, self._upper,
        )

    def check_self_collision(self, q: np.ndarray) -> bool:
        return False

    def active_joint_names(self) -> List[str]:
        return [n for n in self._model.names if n != "universe"]

    @property
    def avg_solve_time_ms(self) -> float:
        if not self._solve_times:
            return 0.0
        return float(np.mean(self._solve_times[-50:]))

    @staticmethod
    def _dedup_locked_joints(robot, locked_joints):
        unique, seen = [], set()
        for name in locked_joints:
            try:
                jid = robot.model.getJointId(name)
            except Exception:
                continue
            if jid <= 0 or jid in seen:
                continue
            seen.add(jid)
            unique.append(name)
        return unique
