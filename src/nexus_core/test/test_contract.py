import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "nexus_core"))
sys.path.insert(0, str(ROOT / "src" / "astral_data_collect"))

from nexus_core.arbiter import CommandArbiter
from nexus_core.profile import Profile, ProfileError, verify_model_manifest
from astral_data_collect.schema import CollectSchema, NexusCollectSchema


def profile(name):
    return Profile.load(ROOT / "src" / "nexus_core" / "profiles" / f"{name}.json")


class ProfileTests(unittest.TestCase):
    def test_builtins_and_schema_roundtrip(self):
        for name, dimension in (("astral_dual_gripper", 16),
                                ("astral_gripper_wuji", 35),
                                ("astral_gripper_wuji_glove", 35),
                                ("nero_dual_xhand", 38)):
            p = profile(name)
            self.assertEqual(p.dimension, dimension)
            schema = NexusCollectSchema(p)
            restored = CollectSchema.from_dict(json.loads(schema.to_json()))
            self.assertEqual(restored.state_names(), schema.state_names())
            self.assertEqual(restored.profile.digest, p.digest)
            self.assertEqual(len(restored.required_streams()), len(p.components) * 2)

    def test_bad_profile_fields(self):
        raw = copy.deepcopy(profile("nero_dual_xhand").raw)
        raw["components"][0]["joints"] = raw["components"][0]["joints"][::-1]
        with self.assertRaisesRegex(ProfileError, "order"):
            Profile(raw)
        raw = copy.deepcopy(profile("nero_dual_xhand").raw)
        raw["components"][0]["upper"] = [1.0]
        with self.assertRaisesRegex(ProfileError, "dimensions"):
            Profile(raw)
        raw = copy.deepcopy(profile("nero_dual_xhand").raw)
        raw["cameras"][1]["role"] = "base"
        with self.assertRaisesRegex(ProfileError, "camera"):
            Profile(raw)
        raw = copy.deepcopy(profile("nero_dual_xhand").raw)
        raw["teleop"]["left"]["arm_base_frame"] = ""
        with self.assertRaisesRegex(ProfileError, "frame"):
            Profile(raw)
        raw = copy.deepcopy(profile("nero_dual_xhand").raw)
        raw["teleop"]["left"]["vr_to_arm_rot"][0] = 0.5
        with self.assertRaisesRegex(ProfileError, "orthogonal"):
            Profile(raw)

    def test_manifest_gate(self):
        p = profile("nero_dual_xhand")
        manifest = p.frozen_schema()
        verify_model_manifest(p, manifest)
        manifest["components"][0]["joints"] = manifest["components"][0]["joints"][::-1]
        with self.assertRaisesRegex(ProfileError, "components"):
            verify_model_manifest(p, manifest)
        manifest = p.frozen_schema()
        manifest["camera_map"]["base_0_rgb"] = "left_wrist"
        with self.assertRaisesRegex(ProfileError, "camera_map"):
            verify_model_manifest(p, manifest)


class ArbiterTests(unittest.TestCase):
    def setUp(self):
        self.p = profile("astral_dual_gripper")
        self.m = CommandArbiter(self.p, startup_timeout=0.5)
        for spec in self.p.components:
            self.m.update_state(spec.name, list(spec.joints),
                                [min(hi, max(lo, 0.0)) for lo, hi in zip(spec.lower, spec.upper)], 1.0)

    def test_single_source_and_takeover_hold(self):
        self.m.select("TELEOP", 1.0)
        for spec in self.p.components:
            self.m.update_candidate("teleop", spec.name, list(spec.joints),
                                    [min(hi, max(lo, 0.2)) for lo, hi in zip(spec.lower, spec.upper)], 1.01)
        out = self.m.tick(1.02)
        self.assertEqual(set(out), {c.name for c in self.p.components})
        self.m.select("PAUSED", 1.03)
        hold = self.m.tick(1.04)
        self.assertNotEqual(out, hold)
        self.assertEqual(self.m.mode, "PAUSED")
        self.m.select("POLICY", 1.05)
        self.assertFalse(any(k[0] == "policy" for k in self.m.candidates))
        self.assertEqual(self.m.tick(1.06), hold)

    def test_legacy_lerobot_mapping_and_original_preservation(self):
        try:
            import h5py
            import numpy as np
            from nexus_core.import_nero_lerobot import import_dataset
        except ImportError:
            self.skipTest("h5py/numpy/Pillow are needed for the data import test")
        p = profile("nero_dual_xhand")
        camera_map = {"base": "observation.image",
                      "left_wrist": "observation.wrist_image_left",
                      "right_wrist": "observation.wrist_image_right"}
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        initial = np.zeros(38, dtype=np.float32)
        next_state = np.full(38, 0.1, dtype=np.float32)
        frames = []
        for i, (state, action) in enumerate(((initial, next_state),
                                             (next_state, next_state))):
            row = {"episode_index": 0, "timestamp": i / 30,
                   "observation.state": state, "action": action}
            row.update({feature: image for feature in camera_map.values()})
            frames.append(row)
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "legacy"
            source.mkdir()
            original = source / "sentinel.txt"
            original.write_text("unchanged", encoding="utf-8")
            output = Path(temp) / "nexus_session"
            report = import_dataset(source, output, p, camera_map, dataset=frames)
            self.assertEqual(original.read_text(encoding="utf-8"), "unchanged")
            self.assertEqual(len(report), 1)
            self.assertFalse(report[0]["action_mismatches"])
            episode = output / "episode_000000"
            with h5py.File(episode / "robot_data.h5") as robot:
                np.testing.assert_allclose(robot["streams/left_arm_state/values"][1],
                                           next_state[:7])
                np.testing.assert_allclose(robot["streams/right_ee_cmd/values"][0],
                                           next_state[-12:])
            with h5py.File(episode / "camera_data.h5") as cameras:
                self.assertEqual(len(cameras["left_wrist/images"]), 2)
            bad = [dict(frame) for frame in frames]
            bad[0]["action"] = initial
            with self.assertRaisesRegex(ValueError, "actions differ"):
                import_dataset(source, Path(temp) / "reject", p, camera_map, dataset=bad)

    def test_legacy_raw_hdf5_import(self):
        try:
            import h5py
            import numpy as np
            from nexus_core.import_nero import import_episode
            from astral_data_collect.align_data import align_session
            from astral_data_collect.validate_data import validate_session
        except ImportError:
            self.skipTest("h5py/numpy are needed for the raw import test")
        p = profile("nero_dual_xhand")
        count = 91
        timestamps = np.arange(count, dtype=np.float64) / 30 + 1.0
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "old_episode"
            source.mkdir()
            with h5py.File(source / "robot_data.h5", "w") as robot:
                robot.create_dataset("timestamps", data=timestamps)
                for field, dim in (("left_arm", 7), ("right_arm", 7),
                                   ("left_hand", 12), ("right_hand", 12)):
                    robot.create_dataset(f"{field}/joints", data=np.zeros((count, dim)))
            with h5py.File(source / "camera_data.h5", "w") as camera:
                for old_id in ("cam_0", "cam_1", "cam_2"):
                    group = camera.create_group(old_id)
                    group.create_dataset("timestamps", data=timestamps)
                    images = group.create_dataset("images", (count,),
                        dtype=h5py.vlen_dtype(np.dtype("uint8")))
                    for i in range(count):
                        images[i] = np.asarray([255, 216, 255, 217], dtype="uint8")
            original = (source / "robot_data.h5").read_bytes()
            output = Path(temp) / "new" / "episode_000001"
            report = import_episode(source, output, p,
                {"base": "cam_0", "left_wrist": "cam_1", "right_wrist": "cam_2"}, "test")
            self.assertEqual(original, (source / "robot_data.h5").read_bytes())
            self.assertEqual(report["streams"]["right_ee_state"], count)
            self.assertEqual(json.loads((output / "meta.json").read_text(encoding="utf-8"))["schema"]["profile_sha256"], p.digest)
            self.assertEqual(len(align_session(str(output.parent), log=lambda _: None)), 1)
            quality = validate_session(str(output.parent), apply=False, log=lambda _: None)
            self.assertEqual(quality["fail"], 0, quality)

    def test_bad_order_and_stale_command(self):
        spec = self.p.components[0]
        with self.assertRaisesRegex(ProfileError, "order"):
            self.m.update_candidate("teleop", spec.name, list(spec.joints)[::-1],
                                    [0.0] * spec.dim, 1.0)
        self.m.select("TELEOP", 1.0)
        self.m.tick(1.6)
        self.assertEqual(self.m.mode, "PAUSED")
        self.assertIn("stale command", self.m.fault)

    def test_estop_latched(self):
        self.m.select("ESTOP", 1.0)
        self.assertEqual(self.m.tick(1.1), {})
        with self.assertRaisesRegex(ProfileError, "latched"):
            self.m.select("TELEOP", 1.2)

    def test_stale_feedback_suppresses_driver_output(self):
        self.m.select("TELEOP", 1.0)
        for spec in self.p.components:
            self.m.update_candidate("teleop", spec.name, list(spec.joints),
                                    [min(hi, max(lo, 0.2)) for lo, hi in zip(spec.lower, spec.upper)], 1.01)
        self.assertTrue(self.m.tick(1.02))
        self.assertEqual(self.m.tick(1.6), {})
        self.assertEqual(self.m.mode, "PAUSED")


if __name__ == "__main__":
    unittest.main()
