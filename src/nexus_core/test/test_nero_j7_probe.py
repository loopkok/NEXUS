"""J7 diagnostic boundaries with no ROS, CAN sockets or physical SDK factory."""
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('nero_j7_probe', ROOT / 'scripts/nexus_nero_j7_probe.py')
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)
PROFILE = ROOT / 'src/nexus_core/profiles/nero_dual_xhand.json'
COMPONENT = next(c for c in json.loads(PROFILE.read_text())['components'] if c['name'] == 'left_arm')


class Clock:
    now = 1.
    def monotonic(self): return self.now
    def time(self): return 1_700_000_000 + self.now
    def sleep(self, seconds): self.now += seconds


class Arm:
    def __init__(self, clock):
        self.clock = clock
        self.q = [-.405, 1.281, -.957, 1.311, 2.682, -.314, -.163]
        self.tx = []
        self.comm = SimpleNamespace(send=self.tx.append)
        self.callbacks = []
        self.disconnected = False
        self.callback_write = False
        self.mode = 4
        self.mode_stuck = False
        self.frozen = False
        self.drift = False
        self.enabled = True
        self.moves = []
        self.stop_count = 0

    def get_context(self): return self
    def register_parser_packet_fun(self, callback): self.callbacks.append(callback)
    def init_comm(self): return self.comm
    def feed(self):
        for can_id, indices in probe.JOINT_FRAMES.items():
            data = b''.join(round(math.degrees(self.q[i]) * 1000).to_bytes(4, 'big', signed=True) for i in indices)
            for callback in self.callbacks:
                callback(SimpleNamespace(arbitration_id=can_id, data=data.ljust(8, b'\0')))
    def connect(self):
        self.feed()
        if self.callback_write:
            self.comm.send(SimpleNamespace(arbitration_id=0x151, data=bytes(8)))
    def disconnect(self): self.disconnected = True
    def get_joint_angles(self): return SimpleNamespace(msg=list(self.q), timestamp=self.clock.time())
    def get_arm_status(self):
        self.feed()
        return SimpleNamespace(timestamp=self.clock.time(), msg=SimpleNamespace(
            ctrl_mode=self.mode, mode_feedback=1, arm_status=0, motion_status=0, err_code=0))
    def get_driver_states(self, joint):
        return SimpleNamespace(timestamp=self.clock.time(), msg=SimpleNamespace(foc_status=SimpleNamespace(
            driver_enable_status=self.enabled, driver_error_status=False, collision_status=False, stall_status=False)))
    def set_motion_mode(self, mode):
        if not self.mode_stuck:
            self.mode = 1
    def set_auto_set_motion_mode_enabled(self, value): pass
    def set_speed_percent(self, value): pass
    def move_j(self, target):
        self.moves.append(list(target))
        if not self.frozen:
            self.q = list(target)
        if self.drift:
            self.q[0] += math.radians(.6)
    def electronic_emergency_stop(self): self.stop_count += 1


class J7ProbeTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.patches = [patch.object(probe.time, name, getattr(self.clock, name))
                        for name in ('monotonic', 'time', 'sleep')]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        self.arm = Arm(self.clock)
        self.feedback = probe.Feedback()
        self.arm.register_parser_packet_fun(self.feedback.receive)
        self.arm.feed()
        self.events = []

    def read(self): return probe.snapshot(self.arm, self.feedback)
    def trial(self):
        return probe.trial(self.arm, self.read, COMPONENT, 1., 5, .8, self.events.append)

    def test_only_j7_changes_one_planned_command_no_return_or_stop_on_success(self):
        before = self.arm.q[:]
        result = self.trial()
        self.assertEqual(result['result'], 'passed')
        self.assertEqual(len(self.arm.moves), 1)
        for a, b in zip(self.arm.moves[0][:6], before[:6]):
            self.assertLess(abs(a-b), math.radians(.001))
        self.assertAlmostEqual(self.arm.moves[0][6] - before[6], math.radians(1), places=5)
        self.assertEqual(self.arm.stop_count, 0)

    def test_frozen_j7_fails_without_resend_or_js_fallback(self):
        self.arm.frozen = True
        with self.assertRaisesRegex(RuntimeError, 'did not reach'):
            self.trial()
        self.assertEqual(len(self.arm.moves), 1)
        self.assertEqual(self.arm.stop_count, 1)

    def test_first_six_deviation_aborts(self):
        self.arm.drift = True
        with self.assertRaisesRegex(RuntimeError, 'J1-J6 moved'):
            self.trial()
        self.assertEqual(self.arm.stop_count, 1)

    def test_disabled_joint_refuses_control_without_any_command(self):
        self.arm.enabled = False
        with self.assertRaisesRegex(RuntimeError, 'already be enabled'):
            self.trial()
        self.assertEqual(self.arm.mode, 4)
        self.assertEqual(self.arm.moves, [])
        self.assertEqual(self.arm.stop_count, 0)

    def test_wrong_mode_never_submits_joint_target(self):
        self.arm.mode_stuck = True
        with self.assertRaisesRegex(RuntimeError, 'mode not confirmed'):
            self.trial()
        self.assertEqual(self.arm.moves, [])
        self.assertEqual(self.arm.stop_count, 1)

    def test_stale_j7_cannot_hide_behind_fresh_other_position_frames(self):
        self.clock.sleep(.3)
        with self.feedback.lock:
            old = self.feedback.frames[0x2A9]
        self.arm.feed()
        with self.feedback.lock:
            self.feedback.frames[0x2A9] = old
        with self.assertRaisesRegex(RuntimeError, 'four position frames'):
            probe.validate_feedback(self.read())

    def test_target_rejects_wide_nonfinite_and_out_of_limit_displacement(self):
        for delta in (0, .1, -.1, 1.1, -1.1, float('nan')):
            with self.subTest(delta=delta), self.assertRaises(ValueError):
                probe.make_target(self.arm.q, COMPONENT, delta)
        q = self.arm.q[:]
        q[6] = COMPONENT['upper'][6]
        with self.assertRaises(ValueError):
            probe.make_target(q, COMPONENT, 1)
        with self.assertRaises(ValueError):
            probe.make_target(q[:6], COMPONENT, 1)

    def run_main(self, write=False, execute=False, conflicts=None):
        self.arm.callback_write = write
        fake = ModuleType('pyAgxArm')
        fake.__file__ = 'FAKE_SDK_NO_CAN'
        fake.AgxArmFactory = SimpleNamespace(create_arm=lambda config: self.arm)
        fake.ArmModel = SimpleNamespace(NERO='nero')
        fake.NeroFW = SimpleNamespace(DEFAULT='default')
        fake.create_agx_arm_config = lambda **kw: kw
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / 'probe.jsonl'
            args = ['probe', '--profile', str(PROFILE), '--duration', '.5', '--output', str(log)]
            if execute:
                args.append('--execute')
            with patch.dict(sys.modules, {'pyAgxArm': fake}), patch.object(sys, 'argv', args), \
                    patch.object(probe, 'active_nero_processes', return_value=conflicts or []), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                code = probe.main()
            return code, [json.loads(line) for line in log.read_text().splitlines()]

    def test_default_main_is_read_only_with_zero_actual_transmits(self):
        code, records = self.run_main()
        self.assertEqual(code, 0)
        self.assertEqual(records[-1]['result'], 'observed_only')
        self.assertEqual(records[-1]['tx_count'], 0)
        self.assertEqual(self.arm.tx, [])
        self.assertEqual(self.arm.moves, [])
        self.assertEqual(self.arm.stop_count, 0)
        self.assertTrue(self.arm.disconnected)

    def test_unexpected_sdk_send_is_blocked_in_read_only_mode(self):
        code, records = self.run_main(write=True)
        self.assertEqual(code, 1)
        self.assertTrue(any(r.get('result') == 'blocked_read_only' for r in records))
        self.assertEqual(self.arm.tx, [])
        self.assertTrue(self.arm.disconnected)

    def test_execute_refuses_running_nero_driver_before_opening_sdk(self):
        with self.assertRaises(SystemExit) as failure:
            self.run_main(execute=True, conflicts=[12345])
        self.assertEqual(failure.exception.code, 2)
        self.assertEqual(self.arm.moves, [])


if __name__ == '__main__':
    unittest.main()
