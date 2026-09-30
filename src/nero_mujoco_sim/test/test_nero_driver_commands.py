"""Exercise the physical-profile command boundary with CAN disabled."""
import unittest
from pathlib import Path
from types import SimpleNamespace
try:
    import rclpy
    from sensor_msgs.msg import JointState
    from nexus_core.nero_driver_node import NeroDriverNode
except ImportError:
    rclpy = None

@unittest.skipIf(rclpy is None,'requires ROS 2')
class NeroDriverCommandTests(unittest.TestCase):
    def setUp(self):
        root=Path(__file__).resolve().parents[3]
        profile=root/'src/nexus_core/profiles/nero_dual_xhand.json'
        rclpy.init(args=['--ros-args','-p',f'profile_file:={profile}','-p','dry_run:=true'])
        self.node=NeroDriverNode()
        self.calls=[]
        self.node._robot=SimpleNamespace(move_js=self.calls.append,disconnect=lambda:None)
        self.node._enabled=True

    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown()

    def message(self,age_ns=0):
        msg=JointState();msg.name=list(self.node.spec.joints);msg.position=[.1]*7
        ns=self.node.get_clock().now().nanoseconds-age_ns
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,1_000_000_000)
        return msg

    def test_expired_command_cannot_touch_sdk_or_refresh_watchdog(self):
        before=self.node._last_cmd
        self.node._on_command(self.message(1_000_000_000))
        self.assertFalse(self.calls);self.assertEqual(before,self.node._last_cmd)
        self.node._on_command(self.message())
        self.assertEqual(len(self.calls),1)

    def test_invalid_order_and_limits_do_not_reach_sdk(self):
        msg=self.message();msg.name=list(reversed(msg.name));self.node._on_command(msg)
        msg=self.message();msg.position[0]=999.;self.node._on_command(msg)
        self.assertFalse(self.calls)
