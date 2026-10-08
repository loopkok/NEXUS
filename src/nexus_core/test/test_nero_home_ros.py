"""Measured homing with a fake CAN SDK, real ROS services and command mux.

All channels are in ROS domain 186 and a temporary instance namespace. The
pyAgxArm factory is replaced before driver construction; no CAN is opened.
"""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from nexus_core.command_mux_node import CommandMuxNode
from nexus_core.driver_manager_node import DriverManagerNode
from nexus_core.nero_driver_node import NeroDriverNode
from nexus_core.profile import Profile


class FakeArm:
    def __init__(self, initial):
        self.q = list(initial)
        self.duration = 0.1
        self.target = None
        self.enabled = False
        self.drop_feedback = False
        self.stale_feedback = False
        self.js_commands = []
        self.mode = "js"
        self.j_commands = 0
        self.frozen_joints = set()

    def connect(self): pass
    def disconnect(self): pass
    def set_auto_set_motion_mode_enabled(self, value): pass
    def set_speed_percent(self, value): self.speed = value
    def set_motion_mode(self, mode): self.mode = mode

    def enable(self):
        self.enabled = True
        return True

    def disable(self):
        self.enabled = False
        self.target = None

    def get_joint_angles(self):
        if self.drop_feedback:
            return None
        if self.target is not None and self.enabled:
            alpha = min(1.0, (time.monotonic() - self.started) / self.duration)
            self.q = [a if i in self.frozen_joints else a + alpha * (b - a)
                      for i, (a, b) in enumerate(zip(self.start, self.target))]
        return SimpleNamespace(msg=list(self.q), timestamp=time.time() - (2.0 if self.stale_feedback else 0.0))

    def get_arm_status(self):
        return SimpleNamespace(timestamp=time.time(), msg=SimpleNamespace(
            ctrl_mode=1, arm_status=0, mode_feedback=1, motion_status=0, err_code=0))

    def get_driver_states(self, index):
        return SimpleNamespace(timestamp=time.time(), msg=SimpleNamespace(foc_status=SimpleNamespace(
            driver_enable_status=self.enabled, driver_error_status=False, collision_status=False, stall_status=False)))

    def move_j(self, target):
        self.start, self.target = list(self.q), list(target)
        self.started = time.monotonic()
        self.j_commands += 1

    def move_js(self, target):
        self.js_commands.append((time.monotonic(), list(target)))
        self.q = [self.q[i] if i in self.frozen_joints else value
                  for i, value in enumerate(target)]
        self.target = None


class NeroHomeTests(unittest.TestCase):
    def setUp(self):
        raw = json.loads((Path(__file__).resolve().parents[1] / "profiles/nero_dual_xhand.json").read_text())
        raw["instance"] = f"test_nero_home_{os.getpid()}"
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / "profile.json"
        self.path.write_text(json.dumps(raw))
        self.profile = Profile.load(self.path)
        self.fakes = {}
        for side in ("left", "right"):
            spec = self.profile.component(f"{side}_arm")
            target = self.profile.adapter_config("nero_can")["home_pose"][side]
            initial = [v + 0.15 if v + 0.15 < hi else v - 0.15 for v, hi in zip(target, spec.upper)]
            self.fakes[side] = FakeArm(initial)
        sdk = ModuleType("pyAgxArm")
        sdk.create_agx_arm_config = lambda **kw: kw
        sdk.AgxArmFactory = SimpleNamespace(create_arm=lambda cfg: self.fakes["left" if "left" in cfg["channel"] else "right"])
        sdk.ArmModel = SimpleNamespace(NERO="fake_nero")
        sdk.NeroFW = SimpleNamespace(DEFAULT="fake")
        self.patch = patch.dict(sys.modules, {"pyAgxArm": sdk})
        self.patch.start()
        rclpy.init(args=["--ros-args", "-p", f"profile_file:={self.path}",
                         "-p", "home_timeout:=12.0", "-p", "service_timeout:=0.5"], domain_id=186)
        self.drivers = {}
        for side in self.fakes:
            self.drivers[side] = NeroDriverNode(parameter_overrides=[
                Parameter("component", value=f"{side}_arm"), Parameter("side", value=side)],
                cli_args=["--ros-args", "-r", f"__node:=test_nero_{side}"])
        self.mux = CommandMuxNode()
        self.manager = DriverManagerNode()
        self.probe = Node("nero_home_test_probe")
        self.feedback = {side: [] for side in self.fakes}
        self.modes = []
        self.subs, self.hand_pubs, self.services = [], [], []
        for side in self.fakes:
            self.subs.append(self.probe.create_subscription(JointState, self.profile.topic(f"{side}_arm", "joint_states"),
                lambda msg, s=side: self.feedback[s].append((time.monotonic(), list(msg.position))), qos_profile_sensor_data))
        self.subs.append(self.probe.create_subscription(String, self.profile.namespace + "/control/state",
            lambda msg: self.modes.append(json.loads(msg.data)["mode"]),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for spec in [c for c in self.profile.components if c.kind == "hand"]:
            self.hand_pubs.append((self.probe.create_publisher(JointState, self.profile.topic(spec.name, "joint_states"), qos_profile_sensor_data), spec))
            self.services.append(self.probe.create_service(Trigger, f"{self.profile.namespace}/drivers/{spec.name}/estop",
                lambda req, res: self.hand_estop(res)))
        self.probe.create_timer(0.02, self.publish_hands)
        self.home_client = self.probe.create_client(Trigger, self.profile.namespace + "/drivers/home")
        self.stop_client = self.probe.create_client(Trigger, self.profile.namespace + "/drivers/estop")
        self.executor = SingleThreadedExecutor()
        for node in [*self.drivers.values(), self.mux, self.probe]: self.executor.add_node(node)
        self.manager_executor = MultiThreadedExecutor(num_threads=8)
        self.manager_executor.add_node(self.manager)
        self.threads = [threading.Thread(target=e.spin) for e in (self.executor, self.manager_executor)]
        self.addCleanup(self.cleanup)
        for t in self.threads: t.start()
        self.wait(lambda: not self.manager._state_faults() and self.manager._mode == "IDLE")
        for driver in self.drivers.values():
            self.assertTrue(driver._enable(None, Trigger.Response()).success)
        self.wait(lambda: self.home_client.service_is_ready())

    @staticmethod
    def hand_estop(response):
        response.success = True
        return response

    def publish_hands(self):
        for pub, spec in self.hand_pubs:
            msg = JointState(name=list(spec.joints), position=[0.0] * spec.dim)
            msg.header.stamp = self.probe.get_clock().now().to_msg()
            pub.publish(msg)

    def wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate(): return
            time.sleep(0.01)
        self.fail(f"condition not reached; mode={self.manager._mode} feedback_faults={self.manager._state_faults()}")

    def cleanup(self):
        for driver in self.drivers.values():
            if driver._homing: driver._finish_home(False, "test cleanup")
        for e in (self.manager_executor, self.executor): e.shutdown(timeout_sec=3)
        for t in self.threads: t.join(timeout=3)
        for node in [self.manager, *self.drivers.values(), self.mux, self.probe]: node.destroy_node()
        rclpy.shutdown()
        self.patch.stop()
        self.temp.cleanup()

    def test_right_home_over_five_seconds_no_feedback_gap_or_recoil(self):
        self.fakes["right"].duration = 8.0
        before = time.monotonic()
        future = self.home_client.call_async(Trigger.Request())
        self.wait(future.done, timeout=10)
        self.assertTrue(future.result().success, future.result().message)
        self.assertGreater(time.monotonic() - before, 5)
        self.assertEqual(self.manager._mode, "IDLE")
        for side, rows in self.feedback.items():
            times = [t for t, _ in rows if t >= before]
            self.assertGreater(len(times), 100)
            gap = max(b - a for a, b in zip(times, times[1:]))
            self.assertLess(gap, 0.5, side)
            self.assertTrue(self.drivers[side]._enabled, side)
            print(f"home side={side} feedback_count={len(times)} max_gap_ms={gap*1000:.1f}")
        # An old hold still within the normal 0.5-s command timeout must not
        # pull the right arm away from the measured arrival position.
        target = self.profile.adapter_config("nero_can")["home_pose"]["right"]
        message = JointState(name=list(self.profile.component("right_arm").joints), position=[v + .1 for v in target])
        message.header.stamp = Time(nanoseconds=self.drivers["right"]._command_not_before_ns - 1).to_msg()
        mark = time.monotonic()
        # Simulate a message already taken from the mux's DDS history, without
        # creating a forbidden second publisher on the final command topic.
        self.drivers["right"]._on_command(message)
        time.sleep(.2)
        for t, command in self.fakes["right"].js_commands:
            if t >= mark: self.assertLess(max(abs(a-b) for a,b in zip(command,target)), .01)
        self.assertNotIn("ESTOP", self.modes)

    def test_unreachable_home_fails_and_never_returns_idle(self):
        self.fakes["right"].duration = 30
        self.drivers["right"]._home_timeout = .8
        future = self.home_client.call_async(Trigger.Request())
        self.wait(future.done)
        self.assertFalse(future.result().success)
        self.assertIn("max_joint_error", future.result().message)
        self.wait(lambda: self.manager._mode == "ESTOP")
        for driver in self.drivers.values(): self.assertFalse(driver._enabled)
        after_home = self.modes[self.modes.index("HOMING") + 1:]
        self.assertNotIn("IDLE", after_home)

    def test_estop_interrupts_pending_home(self):
        self.fakes["right"].duration = 30
        future = self.home_client.call_async(Trigger.Request())
        self.wait(lambda: self.drivers["right"]._homing)
        emergency = self.stop_client.call_async(Trigger.Request())
        self.wait(emergency.done)
        self.assertTrue(emergency.result().success, emergency.result().message)
        self.wait(future.done)
        self.assertFalse(future.result().success)
        self.assertEqual(self.manager._mode, "ESTOP")
        self.assertFalse(self.drivers["right"]._enabled)

    def test_stale_sdk_cache_cannot_complete_home(self):
        self.fakes["right"].duration = 30
        future = self.home_client.call_async(Trigger.Request())
        self.wait(lambda: self.drivers["right"]._homing)
        self.fakes["right"].stale_feedback = True
        future_deadline = time.monotonic()
        self.wait(future.done)
        self.assertFalse(future.result().success)
        self.assertIn("feedback stale", future.result().message)
        self.assertLess(time.monotonic() - future_deadline, 2)
        self.assertEqual(self.manager._mode, "ESTOP")

    def test_reported_joint_seven_residual_is_not_accepted_as_home(self):
        fake, driver = self.fakes["left"], self.drivers["left"]
        target = list(self.profile.adapter_config("nero_can")["home_pose"]["left"])
        fake.q = target[:]
        fake.q[6] += .0793
        fake.frozen_joints = {6}
        driver._home_timeout = .8
        self.wait(lambda: abs(driver._last_q[6] - fake.q[6]) < 1e-4)
        future = self.home_client.call_async(Trigger.Request())
        self.wait(future.done)
        self.assertFalse(future.result().success)
        name = driver.spec.joints[6]
        self.assertIn(f"{name}: 0.0793rad/4.54deg", future.result().message)
        snapshot = driver._last_home_diagnostic
        self.assertEqual(snapshot["pending_joints"], [name])
        self.assertTrue(snapshot["enabled"])
        self.assertTrue(snapshot["drivers"][6]["driver_enable_status"])
        self.assertFalse(fake.enabled)

    def test_diagnostic_service_sends_no_hardware_commands(self):
        driver, fake = self.drivers["right"], self.fakes["right"]
        # Keep normal mux hold traffic out of this service-specific assertion.
        driver._enabled = False
        client = self.probe.create_client(Trigger, f"{self.profile.namespace}/drivers/right_arm/diagnostics")
        self.wait(client.service_is_ready)
        before = (fake.j_commands, len(fake.js_commands), fake.enabled)
        future = client.call_async(Trigger.Request())
        self.wait(future.done)
        response = future.result()
        self.assertTrue(response.success)
        data = json.loads(response.message)
        self.assertEqual(len(data["joints"]), 7)
        self.assertEqual(len(data["drivers"]), 7)
        self.assertEqual(data["controller"]["arm_status"], 0)
        self.assertEqual((fake.j_commands, len(fake.js_commands), fake.enabled), before)


if __name__ == "__main__": unittest.main()
