#!/usr/bin/env python3
"""Tests for non-ROS hardware I/O adapters (RobotIO / CameraIO).

Hermetic: no astral_robot_sdk hardware / no real cameras. RobotIO gets a
FakeRobot via ``robot_factory``; CameraIO gets synthetic providers via
``provider_factory``.
"""

import time
import unittest
from types import SimpleNamespace

import numpy as np

from astral_data_collect.schema import CollectSchema
from astral_policy_inference.hw_io import CameraIO, CameraSpec, RobotIO

try:
    from astral_robot_sdk import LEFT_ARM_IDS, RIGHT_ARM_IDS
except ImportError:  # pragma: no cover
    LEFT_ARM_IDS = [21, 22, 23, 24, 25, 26, 27]
    RIGHT_ARM_IDS = [31, 32, 33, 34, 35, 36, 37]


def schema(**kw):
    base = dict(
        arms=["left"],
        end_effector_left="gripper",
        end_effector_right="none",
        include_waist=False,
        include_head=False,
        cameras=["video8", "video0"],
        dataset_fps=30,
    )
    base.update(kw)
    return CollectSchema(**base)


LEFT7 = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]


def q18_with_left(left=None, right=None, waist=None, head=None):
    q = [0.0] * 18
    q[:7] = left if left is not None else LEFT7
    if right:
        q[7:14] = right
    if waist:
        q[14:16] = waist
    if head:
        q[16:18] = head
    return q


class FakeRobot:
    """Minimal fake of the astral_robot_sdk RobotDriver surface we use."""

    def __init__(self, q18=None):
        self.q18 = list(q18) if q18 is not None else [0.0] * 18
        self.calls = []
        self.motion_mode = None

    def connect(self):
        self.calls.append("connect")

    def one_click_ready(self, enable_timeout_s=3.0, seed_from_current=False):
        self.calls.append(("ready", bool(seed_from_current)))
        return True

    def set_lpf(self, enable, alpha):
        self.calls.append(("lpf", bool(enable), float(alpha)))

    def get_joint_angles(self):
        return SimpleNamespace(msg=SimpleNamespace(positions=list(self.q18)))

    def set_target_positions(self, targets):
        self.calls.append(("set_target_positions", dict(targets)))

    def set_gripper_angle(self, rad, right_hand=False):
        self.calls.append(("set_gripper_angle", float(rad), bool(right_hand)))

    def set_motion_mode(self, m):
        self.motion_mode = int(m)
        self.calls.append(("set_motion_mode", int(m)))

    def move_arm_js(self, left, right):
        self.calls.append(("move_arm_js", list(left), list(right)))

    def e_stop(self):
        self.calls.append("estop")

    def disconnect(self):
        self.calls.append("disconnect")


def make_io(**kw):
    kw.setdefault("robot_factory", lambda: FakeRobot(q18_with_left()))
    kw.setdefault("dry_run", False)
    kw.setdefault("auto_ready", False)
    io = RobotIO(schema(), **kw)
    io.connect()
    return io


class TestRobotIOState(unittest.TestCase):
    def test_read_state_assembles_left_arm_and_gripper_default(self):
        io = make_io()
        st = io.read_state()
        self.assertIsNotNone(st)
        self.assertEqual(st.shape[0], 8)
        np.testing.assert_allclose(st[:7], LEFT7)
        self.assertAlmostEqual(st[7], 0.0, msg="gripper echo defaults to open (0.0)")

    def test_gripper_echo_tracks_last_command(self):
        io = make_io()
        io.send_row(np.array(LEFT7 + [0.5]))
        st = io.read_state()
        self.assertAlmostEqual(st[7], 0.5)

    def test_read_state_includes_waist_head_via_body(self):
        io = RobotIO(
            schema(include_waist=True, include_head=True),
            robot_factory=lambda: FakeRobot(
                q18_with_left(waist=[0.2, -0.3], head=[0.4, 0.5])
            ),
            dry_run=False,
            auto_ready=False,
        )
        io.connect()
        st = io.read_state()
        self.assertEqual(st.shape[0], 12)  # 7 arm + 1 ee + 2 waist + 2 head
        np.testing.assert_allclose(st[:7], LEFT7)
        self.assertAlmostEqual(st[8], 0.2)    # waist_0
        self.assertAlmostEqual(st[9], -0.3)   # waist_1
        self.assertAlmostEqual(st[10], 0.4)   # head_yaw
        self.assertAlmostEqual(st[11], 0.5)   # head_pitch


class TestRobotIOCmd(unittest.TestCase):
    def test_send_row_single_arm_only_left_targets(self):
        io = make_io()
        io.send_row(np.array(LEFT7 + [0.0]))
        robot = io._robot
        arm = [c for c in robot.calls if c[0] == "set_target_positions"]
        self.assertEqual(len(arm), 1)
        self.assertEqual(arm[0][1], dict(zip(LEFT_ARM_IDS, LEFT7)),
                         "left arm commanded by motor id, no other side")
        for c in robot.calls:
            if c[0] == "set_target_positions":
                self.assertTrue(set(c[1].keys()) <= set(LEFT_ARM_IDS),
                                "single-arm session must never command right arm")
        grip = [c for c in robot.calls if c[0] == "set_gripper_angle"]
        self.assertEqual(len(grip), 1)
        self.assertAlmostEqual(grip[0][1], 0.8, places=6, msg="ratio 0 -> open_rad 0.8")
        self.assertFalse(grip[0][2], "left gripper (right_hand=False)")

    def test_gripper_ratio_to_rad_mapping(self):
        io = make_io()
        io.send_row(np.array(LEFT7 + [1.0]))
        grip = [c for c in io._robot.calls if c[0] == "set_gripper_angle"]
        self.assertAlmostEqual(grip[-1][1], 0.0, places=6, msg="ratio 1 -> closed_rad 0.0")
        io.send_row(np.array(LEFT7 + [0.5]))
        grip = [c for c in io._robot.calls if c[0] == "set_gripper_angle"]
        self.assertAlmostEqual(grip[-1][1], 0.4, places=6, msg="0.8 - 0.8*0.5")

    def test_ready_estop(self):
        io = make_io()
        io.ready()
        self.assertTrue(any(c == ("ready", True) for c in io._robot.calls))
        io.estop()
        self.assertIn("estop", io._robot.calls)


class TestRobotIOHitl(unittest.TestCase):
    def test_damping_position_seed(self):
        io = make_io()
        io.set_damping()
        self.assertEqual(io._robot.motion_mode, 0)
        io.set_position(seed_from_current=True)
        self.assertEqual(io._robot.motion_mode, 1)
        seeds = [c for c in io._robot.calls if c[0] == "move_arm_js"]
        self.assertEqual(len(seeds), 1)
        np.testing.assert_allclose(seeds[0][1], LEFT7)   # seed from current left arm
        np.testing.assert_allclose(seeds[0][2], [0.0] * 7)  # seed right with current pose

    def test_position_without_seed(self):
        io = make_io()
        io.set_position(seed_from_current=False)
        self.assertEqual(io._robot.motion_mode, 1)
        self.assertFalse(any(c[0] == "move_arm_js" for c in io._robot.calls))


class TestRobotIODryRun(unittest.TestCase):
    def test_dry_run_no_hardware(self):
        io = RobotIO(schema(), dry_run=True)
        io.connect()
        self.assertIsNone(io._robot)
        st = io.read_state()
        self.assertEqual(st.shape[0], 8)
        io.send_row(np.zeros(8))
        io.set_damping()
        io.set_position()
        io.estop()
        io.disconnect()  # all no-op, must not raise


class TestRobotIOConnect(unittest.TestCase):
    def test_auto_ready(self):
        robot = FakeRobot(q18_with_left())
        io = RobotIO(schema(), robot_factory=lambda: robot, dry_run=False, auto_ready=True)
        io.connect()
        self.assertIn("connect", robot.calls)
        self.assertTrue(any(c[0] == "ready" and c[1] for c in robot.calls),
                        "auto_ready calls one_click_ready(seed_from_current=True)")


class FakeCameraProvider:
    def __init__(self, frame=None, label="cam"):
        self._frame = frame if frame is not None else np.zeros((640, 480, 3), np.uint8)
        self.label = label
        self.started = False
        self.reads = 0

    def start(self):
        self.started = True

    def read(self):
        self.reads += 1
        return self._frame.copy()

    def stop(self):
        self.started = False


class TestCameraIO(unittest.TestCase):
    def test_read_images_letterboxed_to_image_size(self):
        specs = [
            CameraSpec(label="video0", kind="v4l2", device=0),
            CameraSpec(label="video8", kind="v4l2", device=8),
        ]
        cam = CameraIO(specs, image_size=480, provider_factory=lambda s: FakeCameraProvider(label=s.label))
        cam.start()
        try:
            time.sleep(0.15)
            imgs = cam.read_images()
            self.assertEqual(set(imgs), {"video0", "video8"})
            for k, img in imgs.items():
                self.assertEqual(img.shape, (480, 480, 3), f"{k} letterboxed to 480x480")
                self.assertEqual(img.dtype, np.uint8)
        finally:
            cam.stop()

    def test_no_labels_yet_returns_empty(self):
        cam = CameraIO([], image_size=480)
        cam.start()
        try:
            self.assertEqual(cam.read_images(), {})
        finally:
            cam.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)
