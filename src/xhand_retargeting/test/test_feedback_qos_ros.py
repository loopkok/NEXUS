"""Actual best-effort feedback reception, without a hand driver or landmarks."""
import os
import time
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from xhand_control_interfaces.msg import XHandStateArray
from xhand_retargeting.xhand_dex_retargeting_node import XHandDexRetargetingNode


class HandFeedbackQoSTests(unittest.TestCase):
    def test_best_effort_native_feedback_reaches_both_latency_callbacks(self):
        prefix = f"/test_xhand_qos_{os.getpid()}"
        args = ["--ros-args", "-p", "viz:=false", "-p", "dry_run:=true"]
        for side in ("left", "right"):
            args += ["-r", f"/{side}_hand/xhand_state:={prefix}/state/{side}",
                     "-r", f"/{side}_hand/xhand_command:={prefix}/candidate/{side}"]
        rclpy.init(args=args, domain_id=187)
        with patch.object(XHandDexRetargetingNode, "make_config", return_value=Mock()):
            retargeter = XHandDexRetargetingNode()
        probe = Node("xhand_qos_probe", use_global_arguments=False)
        executor = SingleThreadedExecutor()
        executor.add_node(retargeter)
        executor.add_node(probe)
        publishers = {side: probe.create_publisher(XHandStateArray, f"{prefix}/state/{side}", qos_profile_sensor_data)
                      for side in ("left", "right")}
        try:
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and not all(p.get_subscription_count() for p in publishers.values()):
                executor.spin_once(timeout_sec=.02)
            self.assertTrue(all(p.get_subscription_count() for p in publishers.values()))
            now = time.time()
            retargeter._pending_hand_metrics = {side: (now, 0., 0., 0., now, now) for side in publishers}
            for publisher in publishers.values(): publisher.publish(XHandStateArray())
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and not all(retargeter._hand_metrics_window.values()):
                executor.spin_once(timeout_sec=.02)
            self.assertEqual([len(retargeter._hand_metrics_window[side]) for side in publishers], [1, 1])
        finally:
            executor.shutdown()
            retargeter.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__": unittest.main()
