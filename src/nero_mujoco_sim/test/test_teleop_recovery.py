"""Unreachable targets must hold and recover, while real input loss disarms."""
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
try:
    import rclpy
    from nexus_core.nero_teleop_node import NeroTeleopNode
except ImportError:
    rclpy = None

@unittest.skipIf(rclpy is None, 'requires ROS 2')
class TeleopRecoveryTests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[3]
        profile = root / 'src/nexus_core/profiles/nero_dual_xhand_mujoco.json'
        rclpy.init(args=['--ros-args', '-p', f'profile_file:={profile}'])
        self.node = NeroTeleopNode()
        n = self.node
        n._state = np.array([-.405, 1.281, -.957, 1.311, 2.682, -.314, -.163])
        n._vr = (np.zeros(3), Rotation.identity())
        n._state_time = n._vr_time = time.monotonic()
        self.assertTrue(n._start()[0])
        self.messages = []
        n._pub = SimpleNamespace(publish=self.messages.append)
        n._worker.close()
        n._worker = SimpleNamespace(
            take=lambda: (n._generation, n._anchor, n._solver.solve(None),
                          {'reason': 'test'}, time.monotonic(), 1.),
            submit=lambda *args: None, close=lambda: None)
        n._solver = SimpleNamespace(solve=lambda target: None,
                                   residual=lambda *args: np.zeros(6),
                                   fk=lambda q: np.eye(4),
                                   sync_state=lambda *args, **kwargs: None)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_failed_ik_holds_measured_then_recovers_without_rearming(self):
        self.node._tick()
        self.assertTrue(self.node._armed)
        np.testing.assert_allclose(self.messages[-1].position, self.node._state)
        target = self.node._state.copy()
        target[0] += .01
        self.node._solver.solve = lambda _: target
        self.node._tick()
        self.assertTrue(self.node._armed)
        np.testing.assert_allclose(self.messages[-1].position, target)

    def test_missing_input_disarms_and_publishes_no_fresh_hold(self):
        self.node._vr_time = time.monotonic() - 1.
        self.node._tick()
        self.assertFalse(self.node._armed)
        self.assertFalse(self.messages)

    def test_input_expiring_during_solve_does_not_publish_hold(self):
        def failed_solve(_):
            self.node._vr_time = time.monotonic() - 1.
            return None
        self.node._solver.solve = failed_solve
        self.node._tick()
        self.assertFalse(self.messages)

    def test_pre_reanchor_worker_result_is_discarded(self):
        n = self.node
        n._worker.take = lambda: (n._generation-1, n._anchor,
                                  n._state+.1, {}, time.monotonic(), 1.)
        n._tick()
        np.testing.assert_allclose(self.messages[-1].position, n._state)

    def test_worker_exception_disarms_without_refreshing_candidate(self):
        n = self.node
        n._worker.take = lambda: (n._generation, n._anchor, None,
                                  {'reason': 'solver_exception', 'detail': 'test'},
                                  time.monotonic(), 1.)
        n._tick()
        self.assertFalse(n._armed)
        self.assertFalse(self.messages)
