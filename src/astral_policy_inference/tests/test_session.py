#!/usr/bin/env python3
"""Tests for non-ROS RobotSession orchestration + HDF5 session recording.

Hermetic: stub backend + FakeRobot (robot_factory seam) + synthetic camera
frames. The recorder round-trip proves a recorded session is directly
replayable via ``load_replay``.
"""

import os
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace

import h5py
import numpy as np

from astral_data_collect.schema import CollectSchema
from astral_policy_inference.hw_io import CameraIO, CameraSpec, RobotIO
from astral_policy_inference.replay import load_replay
from astral_policy_inference.session import (
    Hdf5SessionRecorder,
    RobotSession,
    SessionConfig,
    load_session_config,
)


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


class FakeRobot:
    def __init__(self):
        q = [0.0] * 18
        q[:7] = LEFT7
        self.q18 = q
        self.calls = []
        self.motion_mode = None

    def connect(self):
        pass

    def one_click_ready(self, *a, **k):
        return True

    def set_lpf(self, *a):
        pass

    def get_joint_angles(self):
        return SimpleNamespace(msg=SimpleNamespace(positions=list(self.q18)))

    def set_target_positions(self, t):
        self.calls.append(("set_target_positions", dict(t)))

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


class FakeCamProvider:
    def __init__(self, spec=None):
        self.started = False

    def start(self):
        self.started = True

    def read(self):
        return np.full((64, 64, 3), 42, np.uint8)

    def stop(self):
        self.started = False


CAM_MAP = {"base_0_rgb": "video8", "left_wrist_0_rgb": "video0"}


def build_session(**kw):
    robot = FakeRobot()
    io = RobotIO(schema(), robot_factory=lambda: robot, dry_run=False, auto_ready=False)
    cam = CameraIO(
        [CameraSpec(label="video0"), CameraSpec(label="video8")],
        image_size=64,
        provider_factory=lambda s: FakeCamProvider(),
    )
    sess = RobotSession(
        schema=schema(),
        robot_io=io,
        camera_io=cam,
        recorder=kw.pop("recorder", None),
        backend_cfg={"backend_type": "stub", "camera_map": dict(CAM_MAP)},
        abs_action_min_scale=0.0,
        **kw,
    )
    return sess, io, cam, robot


def wait_for(pred, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.05)
    return pred()


class TestRecorderRoundTrip(unittest.TestCase):
    def test_records_replayable_h5(self):
        sc = schema()
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, "session.h5")
        rec = Hdf5SessionRecorder(path, sc, record_images=True)
        rec.open()
        n = 5
        for i in range(n):
            rec.append(
                state=np.arange(8, dtype=np.float64) + i,
                images={
                    "video0": np.full((64, 64, 3), i, np.uint8),
                    "video8": np.full((64, 64, 3), i + 1, np.uint8),
                },
                action=np.arange(8, dtype=np.float64) + 10 * i,
                prompt=f"p{i}",
            )
        rec.close()

        ep = load_replay(path)
        self.assertEqual(ep.action_dim, 8)
        self.assertEqual(ep.fps, 30)
        self.assertIsNotNone(ep.schema)
        self.assertEqual(ep.schema.state_names(), sc.state_names())
        np.testing.assert_allclose(ep.actions[2], np.arange(8) + 20)

        with h5py.File(path, "r") as f:
            self.assertEqual(f["action"].shape, (n, 8))
            self.assertEqual(f["state"].shape[0], n)
            self.assertEqual(f["streams/video0"].shape, (n, 64, 64, 3))
            self.assertEqual(f["streams/video8"].shape, (n, 64, 64, 3))
            self.assertEqual(f.attrs["fps"], 30)
            self.assertEqual(f.attrs["schema"], sc.to_json())
            p0 = f["prompt"][0]
            if isinstance(p0, bytes):  # h5py version quirk
                p0 = p0.decode("utf-8")
            self.assertEqual(p0, "p0")


class TestRobotSessionPolicy(unittest.TestCase):
    def test_policy_run_sends_actions_to_sdk(self):
        sess, _io, _cam, robot = build_session()
        sess.start()
        try:
            self.assertTrue(wait_for(lambda: sess.stats()["state"] == "IDLE"))
            time.sleep(0.4)  # obs pump + camera frames settle
            sess.request("policy")
            self.assertTrue(wait_for(lambda: sess.stats()["state"] == "POLICY"),
                            f"state={sess.stats()['state']}")
            self.assertTrue(
                wait_for(lambda: any(c[0] == "set_target_positions" for c in robot.calls)),
                "policy output must reach the SDK",
            )
            self.assertTrue(
                wait_for(lambda: any(c[0] == "set_gripper_angle" for c in robot.calls)),
                "gripper ratio must reach the SDK",
            )
        finally:
            sess.stop()


class TestRobotSessionHitl(unittest.TestCase):
    def test_takeover_damping_release_position_stop_hold(self):
        sess, _io, _cam, robot = build_session()
        sess.start()
        try:
            time.sleep(0.4)
            sess.request("policy")
            self.assertTrue(wait_for(lambda: sess.stats()["state"] == "POLICY"))
            self.assertTrue(
                wait_for(lambda: any(c[0] == "set_target_positions" for c in robot.calls))
            )

            sess.request("takeover")
            self.assertTrue(wait_for(lambda: sess.stats()["state"] == "HUMAN"))
            self.assertEqual(robot.motion_mode, 0, "takeover -> damping (human can drag)")

            sess.request("release")
            self.assertTrue(wait_for(lambda: sess.stats()["state"] == "POLICY"))
            self.assertEqual(robot.motion_mode, 1, "release -> position + replan")

            sess.request("stop")
            self.assertTrue(wait_for(lambda: sess.stats()["state"] == "IDLE"))
            self.assertEqual(robot.motion_mode, 1, "stop holds position, never damping")
        finally:
            sess.stop()


class TestRobotSessionRecording(unittest.TestCase):
    def test_session_records_policy_run(self):
        sc = schema()
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, "session.h5")
        rec = Hdf5SessionRecorder(path, sc, record_images=True)
        sess, _io, _cam, robot = build_session(recorder=rec)
        sess.start()
        try:
            time.sleep(0.4)
            sess.request("policy")
            self.assertTrue(wait_for(lambda: sess.stats()["state"] == "POLICY"))
            self.assertTrue(
                wait_for(lambda: any(c[0] == "set_target_positions" for c in robot.calls))
            )
            time.sleep(1.0)  # let ~30 action rows accumulate
        finally:
            sess.stop()
        # stop() closes the recorder -> file is replayable
        ep = load_replay(path)
        self.assertEqual(ep.action_dim, 8)
        self.assertGreaterEqual(ep.num_frames, 10)


class TestRobotSessionPrompt(unittest.TestCase):
    def test_set_prompt_reaches_runner(self):
        sess, _io, _cam, _robot = build_session(prompt="pick up the red bottle")
        sess.start()
        try:
            time.sleep(0.4)
            self.assertEqual(sess.stats()["prompt"], "pick up the red bottle")
        finally:
            sess.stop()


class TestSessionConfig(unittest.TestCase):
    def test_load_config_builds_schema_and_cameras(self):
        cfg = load_session_config(
            os.path.join(os.path.dirname(__file__), "..", "config", "robot_session.yaml")
        )
        self.assertIsInstance(cfg, SessionConfig)
        self.assertEqual(cfg.schema.state_dim, 8)
        labels = [c.label for c in cfg.cameras]
        self.assertEqual(set(labels), {"video8", "video0"})
        self.assertEqual(cfg.schema.cameras, ["video8", "video0"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
