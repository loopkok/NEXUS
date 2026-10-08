"""Exercise the actual SDK encoder/transmitter with an in-memory transport."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config


class NeroJointPacketTests(unittest.TestCase):
    def test_home_packets_include_joint_seven_for_both_sides_and_firmwares(self):
        path = Path(__file__).resolve().parents[1] / "profiles/nero_dual_xhand.json"
        targets = json.loads(path.read_text())["adapter_config"]["nero_can"]["home_pose"]
        for firmware in (NeroFW.DEFAULT, NeroFW.V111):
            for side, target in targets.items():
                with self.subTest(firmware=firmware, side=side):
                    cfg = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=firmware,
                                                interface="virtual", channel="nexus_packet_test")
                    arm = AgxArmFactory.create_arm(cfg)
                    frames = []
                    transport = SimpleNamespace(send=frames.append, get_channel=lambda: "in_memory")
                    # No connect, physical CAN, driver enable or real send.
                    with patch.object(arm._ctx, "get_comm", return_value=transport):
                        arm.set_motion_mode("j")
                        arm.set_auto_set_motion_mode_enabled(False)
                        arm.set_speed_percent(10)
                        arm.move_j(target[:])
                    packets = {frame.arbitration_id: frame for frame in frames}
                    self.assertEqual(set(packets), {0x151, 0x155, 0x156, 0x157, 0x170})
                    decoded = []
                    for can_id, words in ((0x155, 2), (0x156, 2), (0x157, 2), (0x170, 1)):
                        data = bytes(packets[can_id].data)
                        self.assertEqual(len(data), 8)
                        decoded.extend(int.from_bytes(data[4*i:4*i+4], "big", signed=True)
                                       * math.pi / 180_000 for i in range(words))
                    for actual, expected in zip(decoded, target):
                        self.assertLess(abs(actual - expected), math.pi / 180_000)
                    self.assertEqual(bytes(packets[0x170].data[4:]), bytes(4))


if __name__ == "__main__":
    unittest.main()
