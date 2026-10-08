"""Pure configuration checks; no ROS runtime, CAN or hardware SDK needed."""
import copy
import json
from pathlib import Path
import unittest

from nexus_core.nero_home_config import home_tolerances
from nexus_core.profile import Profile, ProfileError


class NeroHomeConfigTests(unittest.TestCase):
    def setUp(self):
        self.raw = json.loads((Path(__file__).resolve().parents[1]
                              / "profiles/nero_dual_xhand.json").read_text(encoding="utf-8"))

    def test_legacy_profile_without_setting_preserves_default(self):
        self.raw["adapter_config"]["nero_can"].pop("home_tolerance_rad", None)
        profile = Profile(self.raw)
        self.assertEqual(home_tolerances(profile.adapter_config("nero_can"), "left", 7), (.05,) * 7)

    def test_scalar_joint_list_and_independent_sides(self):
        for value, side, expected in (
                (.03, "right", (.03,) * 7),
                ([.05] * 6 + [.1], "left", (.05,) * 6 + (.1,)),
                ({"left": [.05] * 6 + [.1], "right": .02}, "left", (.05,) * 6 + (.1,)),
                ({"left": [.05] * 6 + [.1], "right": .02}, "right", (.02,) * 7)):
            with self.subTest(value=value, side=side):
                raw = copy.deepcopy(self.raw)
                raw["adapter_config"]["nero_can"]["home_tolerance_rad"] = value
                profile = Profile(raw)
                self.assertEqual(home_tolerances(profile.adapter_config("nero_can"), side, 7), expected)

    def test_invalid_settings_fail_profile_validation(self):
        for value in (0, -.1, True, "0.05", None, float("nan"), float("inf"),
                      [], [.05] * 6, [.05] * 8, [.05] * 6 + [False],
                      {}, {"lef": .05}, {"left": [.05] * 6}, {"left": {"j7": .1}}):
            with self.subTest(value=value):
                raw = copy.deepcopy(self.raw)
                raw["adapter_config"]["nero_can"]["home_tolerance_rad"] = value
                with self.assertRaisesRegex(ProfileError, "home_tolerance_rad"):
                    Profile(raw)

    def test_active_arm_requires_explicit_side_in_mapping(self):
        with self.assertRaisesRegex(ValueError, "missing right"):
            home_tolerances({"home_tolerance_rad": {"left": .05}}, "right", 7)


if __name__ == "__main__":
    unittest.main()
