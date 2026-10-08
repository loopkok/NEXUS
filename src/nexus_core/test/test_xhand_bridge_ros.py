"""ROS integration with synthetic XHand feedback; no serial driver or hardware.

Run with ROS_DOMAIN_ID=185 ROS_LOCALHOST_ONLY=1 and the Humble environment.
"""
import math
import json
import os
from pathlib import Path
import tempfile
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from xhand_control_interfaces.msg import XHandCommand, XHandState, XHandStateArray

from nexus_core.joint_bridge_node import JointBridgeNode
from nexus_core.profile import Profile
from astral_web_monitor.monitor_node import MonitorNode


class XHandBridgeTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[1] / "profiles/nero_dual_xhand.json"
        self.temp = tempfile.TemporaryDirectory()
        raw = dict(Profile.load(source).raw)
        raw["instance"] = f"test_xhand_{os.getpid()}"
        profile = Path(self.temp.name) / "profile.json"
        profile.write_text(json.dumps(raw))
        # Remap native channels too: these tests can never reach a real XHand
        # driver even if someone runs them in the robot's ROS domain.
        self.native_state = f"/test_xhand_{os.getpid()}/state"
        self.native_command = f"/test_xhand_{os.getpid()}/command"
        rclpy.init(args=["--ros-args", "-p", f"profile_file:={profile}",
                         "-p", "component:=left_ee", "-p", "side:=left", "-p", "adapter_mode:=xhand",
                         "-r", f"/left_hand/xhand_state:={self.native_state}",
                         "-r", f"/left_hand/xhand_command:={self.native_command}"], domain_id=185)
        self.bridge = JointBridgeNode()
        self.probe = Node("synthetic_xhand", use_global_arguments=False)
        self.monitor = MonitorNode()
        self.monitor.configure_nexus(self.bridge.profile)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.probe)
        self.executor.add_node(self.monitor)
        self.feedback = self.probe.create_publisher(XHandStateArray, self.native_state, qos_profile_sensor_data)
        self.final = self.probe.create_publisher(JointState, self.bridge.profile.topic("left_ee", "joint_commands"), qos_profile_sensor_data)
        self.candidates = self.probe.create_publisher(XHandCommand, f"{self.bridge.profile.namespace}/legacy/left_xhand_candidate", qos_profile_sensor_data)
        self.states, self.commands, self.targets = [], [], []
        self.subs = [
            self.probe.create_subscription(JointState, self.bridge.profile.topic("left_ee", "joint_states"), self.states.append, qos_profile_sensor_data),
            # Match the unmodified vendor driver's reliable subscription.
            self.probe.create_subscription(XHandCommand, self.native_command, self.commands.append,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)),
            self.probe.create_subscription(JointState, self.bridge.profile.candidate_topic("teleop", "left_ee"), self.targets.append, qos_profile_sensor_data),
        ]
        deadline = time.monotonic() + 4.0
        while self.feedback.get_subscription_count() < 1 and time.monotonic() < deadline:
            self.spin(0.02)
        self.assertGreater(self.feedback.get_subscription_count(), 0)
        self.spin(0.3)

    def tearDown(self):
        self.executor.shutdown()
        self.probe.destroy_node()
        self.monitor.destroy_node()
        self.bridge.destroy_node()
        rclpy.shutdown()
        self.temp.cleanup()

    def spin(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=0.002)

    def state(self):
        msg = XHandStateArray()
        msg.header.stamp = self.probe.get_clock().now().to_msg()
        msg.hand_id = [7]
        state = XHandState()
        state.name = [n.removeprefix("left_hand_") for n in self.bridge.spec.joints]
        state.position = [float((lo + hi) / 2) for lo, hi in zip(self.bridge.spec.lower, self.bridge.spec.upper)]
        msg.hand_states = [state]
        return msg

    def test_reordered_measured_state_and_command_qos_gate(self):
        msg = self.state()
        positions = list(msg.hand_states[0].position)
        msg.hand_states[0].name.reverse()
        msg.hand_states[0].position.reverse()
        self.feedback.publish(msg)
        self.spin(0.1)
        self.assertEqual(list(self.states[-1].name), list(self.bridge.spec.joints))
        self.assertFalse(self.monitor.nexus_snapshot()["joints"]["left_ee"]["stale"])
        for actual, expected in zip(self.states[-1].position, positions):
            self.assertAlmostEqual(actual, expected, places=5)
        command = JointState(name=list(self.bridge.spec.joints), position=positions)
        self.final.publish(command)
        self.spin(0.05)
        self.assertFalse(self.commands, "no vendor commands before explicit enable")
        self.assertTrue(self.bridge._enable(None, Trigger.Response()).success)
        self.final.publish(command)
        self.spin(0.1)
        self.assertEqual(len(self.commands), 1, "reliable vendor subscription must match")
        self.assertEqual(self.commands[0].hand_id, 7)
        self.assertEqual(self.commands[0].mode, 3)
        self.spin(0.55)
        self.final.publish(command)
        self.spin(0.05)
        self.assertEqual(len(self.commands), 1, "expired feedback must block commands")
        self.assertTrue(self.monitor.nexus_snapshot()["joints"]["left_ee"]["stale"])

    def test_invalid_feedback_never_refreshes_ready(self):
        for kind in ("stale", "future", "zero_stamp", "dimension", "duplicate_name", "nan", "limit", "devices"):
            msg = self.state()
            if kind == "stale":
                msg.header.stamp.sec -= 2
            elif kind == "future":
                msg.header.stamp.sec += 2
            elif kind == "zero_stamp":
                msg.header.stamp.sec = msg.header.stamp.nanosec = 0
            elif kind == "dimension":
                msg.hand_states[0].position.pop()
            elif kind == "duplicate_name":
                msg.hand_states[0].name.append(msg.hand_states[0].name[0])
            elif kind == "nan":
                msg.hand_states[0].position[0] = math.nan
            elif kind == "limit":
                msg.hand_states[0].position[0] = self.bridge.spec.upper[0] + 0.2
            elif kind == "devices":
                msg.hand_id.append(8)
            self.feedback.publish(msg)
            self.spin(0.04)
            self.assertEqual(self.bridge._last_feedback, 0.0, kind)
            self.assertFalse(self.bridge._ready(None, Trigger.Response()).success, kind)
        self.assertFalse(self.states)
        self.assertTrue(self.monitor.nexus_snapshot()["joints"]["left_ee"]["stale"])
        self.feedback.publish(self.state())
        self.spin(0.1)
        self.assertTrue(self.bridge._ready(None, Trigger.Response()).success)

    def test_feedback_stays_ready_under_candidate_load(self):
        begin, next_state, next_candidate, next_web_check = time.monotonic(), 0.0, 0.0, 0.0
        last_valid, max_gap = None, 0.0
        while time.monotonic() - begin < 3.0:
            now = time.monotonic()
            if now >= next_state:
                self.feedback.publish(self.state())
                next_state = now + 0.01
            if now >= next_candidate:
                measured = self.state().hand_states[0]
                self.candidates.publish(XHandCommand(name=measured.name, position=measured.position))
                next_candidate = now + 1 / 72
            self.executor.spin_once(timeout_sec=0.001)
            if last_valid != self.bridge._last_feedback:
                if last_valid:
                    max_gap = max(max_gap, self.bridge._last_feedback - last_valid)
                last_valid = self.bridge._last_feedback
            if now - begin > 0.2:
                self.assertTrue(self.bridge._ready(None, Trigger.Response()).success)
                if now >= next_web_check:
                    self.assertFalse(self.monitor.nexus_snapshot()["joints"]["left_ee"]["stale"])
                    next_web_check = now + 0.1
        self.assertGreater(len(self.states), 150)
        self.assertGreater(len(self.targets), 100)
        self.assertLess(max_gap, 0.5)
        self.assertFalse(self.commands)
        print(f"XHand synthetic feedback count={len(self.states)} candidates={len(self.targets)} max_gap_ms={max_gap * 1000:.1f}")


if __name__ == "__main__":
    unittest.main()
