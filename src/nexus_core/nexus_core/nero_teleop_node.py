"""Quest wrist to Nero arm candidate commands; original IK remains untouched."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from nero_quest_teleop.ik_solver import IKSolver
from nero_quest_teleop.pose_processor import PoseProcessor
from nero_quest_teleop.safety_filter import SafetyFilter
from .profile import Profile


class NeroTeleopNode(Node):
    def __init__(self):
        super().__init__("nexus_nero_teleop")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("side", "left")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("input_timeout", 0.5)
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        self.side = str(self.get_parameter("side").value)
        self.spec = self.profile.component(f"{self.side}_arm")
        if self.spec.ik != "nero_analytic":
            raise ValueError("Nero teleop requires nero_analytic IK adapter")
        cfg = self.profile.raw["teleop"][self.side]
        self._solver = IKSolver()
        self._processor = PoseProcessor(
            np.array(cfg["vr_to_arm_rot"], dtype=float).reshape(3, 3),
            pos_smoothing=0.8, rot_smoothing=0.8,
            motion_scale=float(cfg["motion_scale"]), flip_pitch=False)
        self._safety = SafetyFilter(
            np.array(self.spec.lower), np.array(self.spec.upper),
            max_joint_vel=float(cfg.get("max_joint_step", 0.065)),
            workspace_radius=float(cfg.get("workspace_radius", 0.58)),
            workspace_z_min=-1.0, workspace_z_max=1.0)
        self._armed = False
        self._state = None
        self._state_time = 0.0
        self._vr = None
        self._vr_time = 0.0
        self._anchor = None
        self._last_step = time.monotonic()
        ns = self.profile.namespace
        self._pub = self.create_publisher(
            JointState, self.profile.candidate_topic("teleop", self.spec.name),
            qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f"{ns}/input/{self.side}/wrist_pose",
                                 self._on_vr, qos_profile_sensor_data)
        self.create_subscription(JointState, self.profile.topic(self.spec.name, "joint_states"),
                                 self._on_state, qos_profile_sensor_data)
        self.create_subscription(Bool, "/teleop/start", self._on_start, 10)
        self.create_subscription(Bool, "/teleop/disarm", self._on_disarm, 10)
        self.create_service(Trigger, "~/start", self._start_service)
        self.create_service(Trigger, "~/reanchor", self._start_service)
        self.create_timer(1.0 / max(1.0, float(self.get_parameter("control_rate").value)), self._tick)
        self.get_logger().info(f"Nero teleop {self.side}: IK core=nero_quest_teleop.ik_solver; "
                               f"profile_sha256={self.profile.digest}")

    def _on_vr(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        position = np.array([p.x, p.y, p.z], dtype=float)
        quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if not np.isfinite(position).all() or not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1.0) > 0.1:
            return
        self._vr = (position, Rotation.from_quat(quat))
        self._vr_time = time.monotonic()

    def _on_state(self, msg: JointState) -> None:
        if list(msg.name) != list(self.spec.joints) or len(msg.position) != self.spec.dim:
            return
        values = np.asarray(msg.position, dtype=float)
        if not np.isfinite(values).all() or np.any(values < np.asarray(self.spec.lower) - 0.05) or np.any(values > np.asarray(self.spec.upper) + 0.05):
            return
        self._state = values
        self._state_time = time.monotonic()

    def _start(self) -> tuple[bool, str]:
        now = time.monotonic()
        if self._state is None or now - self._state_time > 0.5:
            return False, "fresh Nero joint feedback required"
        if self._vr is None or now - self._vr_time > float(self.get_parameter("input_timeout").value):
            return False, "fresh wrist pose required"
        self._solver.sync_state(self._state)
        self._anchor = self._solver.fk(self._state)
        self._processor.set_vr_zero_point(*self._vr)
        self._processor.update_vr_pose(*self._vr)
        self._safety.set_initial_state(self._state, self._anchor[:3, 3])
        self._armed = True
        return True, "Nero teleop reanchored to measured joints"

    def _on_start(self, msg: Bool) -> None:
        if msg.data:
            ok, reason = self._start()
            (self.get_logger().info if ok else self.get_logger().error)(reason)

    def _on_disarm(self, msg: Bool) -> None:
        if msg.data:
            self._armed = False

    def _start_service(self, _request, response):
        response.success, response.message = self._start()
        return response

    def _tick(self) -> None:
        if not self._armed or self._vr is None or self._state is None or self._anchor is None:
            return
        now = time.monotonic()
        if now - self._vr_time > float(self.get_parameter("input_timeout").value) or now - self._state_time > 0.5:
            self._armed = False
            self.get_logger().error("Nero input/feedback timeout; teleop disarmed")
            return
        self._processor.update_vr_pose(*self._vr)
        delta_pos, delta_rot = self._processor.process()
        target = self._processor.compute_target_pose(
            delta_pos, delta_rot, self._anchor[:3, 3], self._anchor[:3, :3])
        target[:3, 3] = self._safety.check_workspace(target[:3, 3])
        solved = self._solver.solve(target)
        if solved is None:
            return
        safe, info = self._safety.filter(np.asarray(solved, dtype=float), now - self._last_step)
        self._last_step = now
        if info.get("collision"):
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.spec.joints)
        msg.position = safe.tolist()
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = NeroTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
