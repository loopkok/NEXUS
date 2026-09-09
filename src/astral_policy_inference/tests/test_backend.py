#!/usr/bin/env python3
"""Tests for policy backends + factory (no network / no torch needed)."""

import json
import os
import sys
import tempfile
import types
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from astral_policy_inference.backend import (
    LerobotActBackend,
    ObsBatch,
    OpenPiServerBackend,
    PolicyError,
    StubBackend,
    _actions_from_response,
    make_backend,
)

ACT_DIM = 8
CAM_MAP = {"base_0_rgb": "video8", "left_wrist_0_rgb": "video0"}


class FakeClient:
    def __init__(self, host, port, response=None, raise_on_infer=False):
        self.host, self.port = host, port
        self.response = response if response is not None else {
            "actions": np.zeros((5, ACT_DIM)),
        }
        self.raise_on_infer = raise_on_infer
        self.payloads = []
        self.reset_calls = 0
        self.closed = False

    def infer(self, payload):
        if self.raise_on_infer:
            raise ConnectionError("boom")
        self.payloads.append(payload)
        return self.response

    def reset(self):
        self.reset_calls += 1

    def close(self):
        self.closed = True


class TestPayloadMapping(unittest.TestCase):
    def _make(self, client):
        return OpenPiServerBackend(
            host="h", port=8000, action_dim=ACT_DIM,
            slot_keys=CAM_MAP, client_factory=lambda *a, **k: client,
        )

    def test_openpi_payload_keys(self):
        client = FakeClient("h", 8000)
        bk = self._make(client)
        bk.open()
        obs = ObsBatch(
            state=np.arange(ACT_DIM, dtype=np.float64),
            images={"video8": np.zeros((8, 8, 3), np.uint8)},
            prompt="pick up",
        )
        out = bk.infer(obs)
        self.assertEqual(out.shape[1], ACT_DIM)
        payload = client.payloads[-1]
        self.assertTrue(np.array_equal(payload["observation/state"], obs.state))
        self.assertEqual(payload["prompt"], "pick up")
        self.assertIn("observation/camera/base_0_rgb", payload)
        # missing slot stays absent -> server zero-pads (AstralInputs.image_mask)
        self.assertNotIn("observation/camera/left_wrist_0_rgb", payload)

    def test_missing_actions_key_raises(self):
        client = FakeClient("h", 8000, response={"not_actions": []})
        bk = self._make(client)
        bk.open()
        with self.assertRaises(PolicyError):
            bk.infer(ObsBatch(state=np.zeros(ACT_DIM), images={}, prompt=""))

    def test_backend_raises_when_not_open(self):
        bk = self._make(FakeClient("h", 8000))
        with self.assertRaises(PolicyError):
            bk.infer(ObsBatch(state=np.zeros(ACT_DIM), images={}, prompt=""))

    def test_dim_mismatch_surfaces_policy_error(self):
        client = FakeClient("h", 8000,
                            response={"actions": np.zeros((3, ACT_DIM + 1))})
        bk = self._make(client)
        bk.open()
        with self.assertRaises(PolicyError):
            bk.infer(ObsBatch(state=np.zeros(ACT_DIM), images={}, prompt=""))

    def test_close_resets_client(self):
        client = FakeClient("h", 8000)
        bk = self._make(client)
        bk.open()
        bk.reset()
        bk.close()
        self.assertEqual(client.reset_calls, 1)


class TestActionsFromResponse(unittest.TestCase):
    def test_1d_and_2d(self):
        a = _actions_from_response({"actions": np.zeros(ACT_DIM)}, ACT_DIM, 50)
        self.assertEqual(a.shape, (1, ACT_DIM))
        b = _actions_from_response({"actions": np.zeros((4, ACT_DIM))}, ACT_DIM, 2)
        self.assertEqual(b.shape, (2, ACT_DIM))

    def test_dim_mismatch(self):
        with self.assertRaises(PolicyError):
            _actions_from_response({"actions": np.zeros((4, 9))}, ACT_DIM, 50)


class TestFactory(unittest.TestCase):
    def test_openpi_selected(self):
        bk = make_backend(backend_type="openpi", action_dim=ACT_DIM, camera_map=CAM_MAP)
        self.assertIsInstance(bk, OpenPiServerBackend)

    def test_act_requires_checkpoint(self):
        with self.assertRaises(PolicyError):
            make_backend(backend_type="act", action_dim=ACT_DIM, camera_map=CAM_MAP)

    def test_stub_and_unknown(self):
        bk = make_backend(backend_type="stub", action_dim=ACT_DIM, camera_map={})
        self.assertIsInstance(bk, StubBackend)
        with self.assertRaises(PolicyError):
            make_backend(backend_type="nope", action_dim=ACT_DIM, camera_map={})


class TestLerobotActBackendLoad(unittest.TestCase):
    """Regression: ``LerobotActBackend.open()`` must resolve the concrete policy
    class through the lerobot factory. In lerobot 0.6.2 (both this fork and
    VLA/lerobot) ``PreTrainedPolicy`` is the *abstract* base — calling
    ``PreTrainedPolicy.from_pretrained`` directly cannot instantiate a real
    ACT checkpoint (``Can't instantiate abstract class``). The factory's
    ``get_policy_class`` maps ``config.json["type"]`` ("act") -> ``ACTPolicy``.

    Hermetic: lerobot is faked via ``sys.modules`` so this runs without lerobot
    installed (matches the file's "no torch needed" contract)."""

    def _fake_lerobot(self, seen):
        fake = types.ModuleType("lerobot.policies")

        class FakePolicy:
            @classmethod
            def from_pretrained(cls, path):
                seen.append(("from_pretrained", path))
                return SimpleNamespace(
                    config=SimpleNamespace(type="act"), to=lambda *a, **k: None
                )

        fake.get_policy_class = (
            lambda t: seen.append(("get_policy_class", t)) or FakePolicy
        )
        fake.make_pre_post_processors = (
            lambda cfg, pretrained_path=None: ((lambda x: x), (lambda x: x))
        )
        return fake

    def test_open_resolves_concrete_policy_via_factory(self):
        seen = []
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "config.json"), "w") as f:
                json.dump({"type": "act"}, f)
            fake = self._fake_lerobot(seen)
            with mock.patch.dict(
                sys.modules,
                {"lerobot": types.ModuleType("lerobot"), "lerobot.policies": fake},
            ):
                bk = LerobotActBackend(checkpoint_dir=d, action_dim=8, image_keys={})
                bk.open()
        self.assertEqual(
            seen,
            [("get_policy_class", "act"), ("from_pretrained", d)],
        )
        self.assertIsNotNone(bk._policy)
        self.assertIsNotNone(bk._pre)
        self.assertIsNotNone(bk._post)

    def test_open_unknown_type_raises_policy_error(self):
        seen = []
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "config.json"), "w") as f:
                json.dump({"type": "nope"}, f)
            fake = self._fake_lerobot(seen)

            def boom(t):
                raise ValueError(f"unknown policy {t}")

            fake.get_policy_class = boom
            with mock.patch.dict(
                sys.modules,
                {"lerobot": types.ModuleType("lerobot"), "lerobot.policies": fake},
            ):
                bk = LerobotActBackend(checkpoint_dir=d, action_dim=8, image_keys={})
                with self.assertRaises(PolicyError):
                    bk.open()


if __name__ == "__main__":
    unittest.main(verbosity=2)
