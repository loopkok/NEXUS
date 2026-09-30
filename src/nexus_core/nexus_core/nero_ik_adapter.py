"""Fast interactive Nero IK around the unchanged analytic core.

The core remains available for offline global searches. Teleop uses measured/
accepted joint continuity and a bounded local refinement, a worker for full-circle
scans outside the ROS executor thread.
"""
from __future__ import annotations
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from nero_quest_teleop.ik_solver import IKSolver, NeroParams, ContinuityParams, fk_all


class InteractiveNeroIK:
    def __init__(self, lower, upper, *, refinement_ms=8.0):
        params = NeroParams.default()
        params.joint_limits = np.column_stack((lower, upper)).astype(float)
        self.core = IKSolver(params, ContinuityParams(enable_global_fallback=False))
        self.params = params
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.refinement_ms = float(refinement_ms)
        if not 0 < self.refinement_ms <= 20:
            raise ValueError('IK refinement_ms must be in (0, 20]')
        self.seed = np.clip(np.zeros(7), self.lower, self.upper)
        self.last_report = {}
        self.global_core = IKSolver(params)

    def fk(self, q):
        return self.core.fk(q)

    def sync_state(self, q, reset_branch=True):
        self.seed = np.asarray(q, dtype=float).copy()
        self.core.sync_state(self.seed, reset_branch=reset_branch)
        # Recover the arm angle from the actual elbow instead of forcing the
        # first tick after every reanchor into a global 0.03-rad grid scan.
        Ts = fk_all(self.seed, self.params)
        S = np.array([0., 0., self.params.d_i[0]])
        W = Ts[5][:3, 3]
        E = Ts[3][:3, 3]
        sw = W - S
        distance = np.linalg.norm(sw)
        if distance < 1e-10:
            return
        u = sw / distance
        e1 = np.cross(S, u)
        if np.linalg.norm(e1) < 1e-10:
            e1 = np.cross([1., 0., 0.], u)
        if np.linalg.norm(e1) < 1e-10:
            e1 = np.cross([0., 1., 0.], u)
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(u, e1)
        a, b = abs(self.params.d_i[2]), abs(self.params.d_i[4])
        C = S + ((a*a - b*b + distance*distance)/(2*distance))*u
        self.core._state.theta0_prev = math.atan2(float(np.dot(E-C, e2)), float(np.dot(E-C, e1)))

    def geometry_reason(self, target):
        S = np.array([0., 0., self.params.d_i[0]])
        W = target[:3, 3] - (self.params.post_transform_d8 + self.params.d_i[6])*target[:3, 2]
        distance = float(np.linalg.norm(W-S))
        a, b = abs(self.params.d_i[2]), abs(self.params.d_i[4])
        reason = ''
        if distance > a+b+1e-8:
            reason = 'reach_outer'
        elif distance < abs(a-b)-1e-8:
            reason = 'reach_inner'
        else:
            angle = math.acos(float(np.clip((distance*distance-a*a-b*b)/(2*a*b), -1., 1.)))
            if not any(self.lower[3]-1e-8 <= q <= self.upper[3]+1e-8 for q in (angle, -angle)):
                reason = 'elbow_limit'
        return reason, distance

    @staticmethod
    def residual(current, target):
        return np.concatenate((target[:3, 3]-current[:3, 3],
            Rotation.from_matrix(target[:3, :3]@current[:3, :3].T).as_rotvec()))

    def _valid_solution(self, q, target):
        if q is None or not np.isfinite(q).all() or np.any(q<self.lower) or np.any(q>self.upper):
            return False
        e = self.residual(self.fk(q), target)
        return np.linalg.norm(e[:3]) <= .0005 and np.linalg.norm(e[3:]) <= .002

    def solve(self, target):
        target = np.asarray(target, dtype=float)
        reason, distance = self.geometry_reason(target)
        self.last_report = {'reason': reason, 'wrist_distance_m': distance,
                            'target_position_m': target[:3, 3].tolist(), 'method': 'geometry'}
        if reason:
            return None
        # Already at the desired pose is a valid solution, including singular
        # orientations where the analytic Euler extraction discards candidates.
        if self._valid_solution(self.seed, target):
            self.last_report['method'] = 'seed'
            return self.seed.copy()
        q = self.core.solve(target)
        if self._valid_solution(q, target):
            self.last_report['method'] = 'analytic_local'
            self.sync_state(q, reset_branch=False)
            return q
        # Bounded damped least squares uses the same unchanged FK/DH model.
        # Every returned candidate must match the full target pose and limits.
        q = self.seed.copy()
        deadline = time.perf_counter() + self.refinement_ms/1000.
        for _ in range(16):
            if time.perf_counter() >= deadline:
                break
            Ts = fk_all(q, self.params)
            T = self.fk(q)
            e = self.residual(T, target)
            J = np.empty((6, 7))
            for j in range(7):
                axis = Ts[j+1][:3, 2]
                J[:3, j] = np.cross(axis, T[:3, 3]-Ts[j+1][:3, 3])
                J[3:, j] = axis
            weights = np.array([1., 1., 1., .3, .3, .3])
            Jw, ew = J*weights[:, None], e*weights
            step = Jw.T@np.linalg.solve(Jw@Jw.T + .0001*np.eye(6), ew)
            step *= min(1., .12/max(float(np.max(np.abs(step))), 1e-12))
            cost = np.linalg.norm(ew)
            accepted = False
            for scale in (1., .5, .25):
                candidate = np.clip(q + scale*step, self.lower, self.upper)
                if np.linalg.norm(self.residual(self.fk(candidate), target)*weights) < cost:
                    q = candidate
                    accepted = True
                    break
            if self._valid_solution(q, target):
                self.last_report['method'] = 'bounded_refinement'
                self.sync_state(q, reset_branch=False)
                return q.copy()
            if not accepted:
                break
        # Retain the original global search as a final fallback so bounding the
        # fast path never removes reachable branches. The caller runs this
        # method on a latest-request worker, outside the ROS control executor.
        self.global_core.sync_state(self.seed)
        q = self.global_core.solve(target)
        if self._valid_solution(q, target):
            self.last_report['method'] = 'global_fallback'
            self.sync_state(q, reset_branch=False)
            return q
        self.last_report['reason'] = 'limits_or_search'
        self.last_report['method'] = 'global_fallback'
        return None
