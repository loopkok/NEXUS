"""ROS-enabled regression tests for the simulator's command safety boundary."""
import unittest
from pathlib import Path

try:
    import rclpy
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from nero_mujoco_sim.mujoco_sim_node import NeroMujocoSimNode
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'requires ROS 2 and installed simulator')
class RuntimeCommandTests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[3]
        profile = root / 'src/nexus_core/profiles/nero_dual_xhand_mujoco.json'
        rclpy.init(args=['--ros-args', '-p', f'profile_file:={profile}',
                        '-p', 'enable_viewer:=false'])
        self.node = NeroMujocoSimNode()
        self.node._enabled['left_arm'] = True

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def command(self, stamp_ns):
        msg = JointState()
        msg.name = list(self.node.specs['left_arm'].joints)
        msg.position = [.1, 0., 0., 0., 0., 0., 0.]
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        return msg

    def test_expired_command_does_not_refresh_watchdog_or_replace_target(self):
        before = self.node._targets['left_arm'].copy()
        now = self.node.get_clock().now().nanoseconds
        self.node._on_command('left_arm', self.command(now - 1_000_000_000))
        self.assertEqual(self.node._targets['left_arm'], before)
        self.assertEqual(self.node._last_commands['left_arm'], 0.)
        self.node._on_command('left_arm', self.command(self.node.get_clock().now().nanoseconds))
        self.assertEqual(self.node._targets['left_arm'][0], .1)
        self.assertGreater(self.node._last_commands['left_arm'], 0.)

    def test_queued_pre_home_command_cannot_cancel_homing(self):
        before = self.node.get_clock().now().nanoseconds
        response = self.node._home('left_arm', None, Trigger.Response())
        self.assertTrue(response.success)
        home_target = self.node._targets['left_arm'].copy()
        self.node._on_command('left_arm', self.command(before))
        self.assertEqual(self.node._targets['left_arm'], home_target)
        self.assertIn('left_arm', self.node._homing_started)

    def test_estop_ignores_subsequent_fresh_commands(self):
        self.node._estop('left_arm', None, Trigger.Response())
        before = self.node._targets['left_arm'].copy()
        self.node._on_command('left_arm', self.command(self.node.get_clock().now().nanoseconds))
        self.assertEqual(self.node._targets['left_arm'], before)
        self.assertFalse(any(self.node._enabled.values()))
