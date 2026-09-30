"""Known-pose correctness and latest-request isolation for Nero interactive IK."""
import threading
import time
import unittest
from types import SimpleNamespace
import numpy as np
from nexus_core.nero_ik_adapter import InteractiveNeroIK
from nexus_core.latest_ik_worker import LatestIKWorker
from nero_quest_teleop.ik_solver import NeroParams

class NeroInteractiveIKTests(unittest.TestCase):
    def setUp(self):
        limits = NeroParams.default().joint_limits
        self.solver = InteractiveNeroIK(limits[:, 0], limits[:, 1])

    def test_known_targets_beyond_old_base_sphere_are_not_clipped(self):
        rng = np.random.default_rng(77)
        found = 0
        for _ in range(32):
            q = rng.uniform(self.solver.lower+.001, self.solver.upper-.001)
            target = self.solver.fk(q)
            self.solver.sync_state(q)
            out = self.solver.solve(target)
            self.assertIsNotNone(out)
            np.testing.assert_allclose(self.solver.fk(out), target, atol=1e-8)
            found += np.linalg.norm(target[:3, 3]) > .58
        self.assertGreater(found, 0)

    def test_near_home_motion_preserves_full_pose_and_limits(self):
        rng = np.random.default_rng(42)
        for home in [np.array([-.405,1.281,-.957,1.311,2.682,-.314,-.163]),
                     np.array([.405,1.281,.957,1.311,-2.682,.314,-.163])]:
            for _ in range(16):
                q = np.clip(home+rng.normal(0,.12,7),self.solver.lower+.001,self.solver.upper-.001)
                target = self.solver.fk(q)
                self.solver.sync_state(home)
                out = self.solver.solve(target)
                self.assertIsNotNone(out)
                e = self.solver.residual(self.solver.fk(out),target)
                self.assertLessEqual(np.linalg.norm(e[:3]),.0005)
                self.assertLessEqual(np.linalg.norm(e[3:]),.002)
                self.assertTrue(np.all(out>=self.solver.lower) and np.all(out<=self.solver.upper))

    def test_geometric_rejection_does_not_scan_core(self):
        self.solver.core.solve = lambda _: self.fail('outer reach must reject before scanning')
        target = np.eye(4); target[:3,3]=[2.,0.,.138]
        self.assertIsNone(self.solver.solve(target))
        self.assertEqual(self.solver.last_report['reason'],'reach_outer')

    def test_worker_coalesces_targets_without_blocking_submit(self):
        entered, release, done = threading.Event(), threading.Event(), threading.Event()
        seen = []
        class Solver:
            last_report = {}
            def sync_state(self, q): pass
            def solve(self, target):
                seen.append(float(target[0]))
                if len(seen)==1:
                    entered.set(); release.wait(2.)
                else: done.set()
                return target
        worker = LatestIKWorker(Solver())
        try:
            worker.submit(1,np.array([1.]));self.assertTrue(entered.wait(1.))
            worker.submit(1,np.array([2.]));worker.submit(1,np.array([3.]))
            release.set();self.assertTrue(done.wait(1.))
            self.assertEqual(seen,[1.,3.])
        finally:
            release.set();worker.close()
