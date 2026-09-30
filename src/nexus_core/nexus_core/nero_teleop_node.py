"""Quest link7 flange poses to Nero arm candidate commands; IK stays untouched."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .nero_ik_adapter import InteractiveNeroIK
from .latest_ik_worker import LatestIKWorker
from nero_quest_teleop.pose_processor import PoseProcessor
from nero_quest_teleop.safety_filter import SafetyFilter
from .profile import Profile

# Candidate commands are latest-value control data: avoid reliable DDS
# backpressure and let the mux stale-command watchdog reject a stalled stream.
_CANDIDATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class NeroTeleopNode(Node):
    def __init__(self):
        super().__init__("nexus_nero_teleop")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("side", "left")
        self.declare_parameter("component", "")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("input_timeout", 0.5)
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        side_param = str(self.get_parameter("side").value)
        component = str(self.get_parameter("component").value)
        self.spec = self.profile.component(component) if component else self.profile.component(f"{side_param}_arm")
        self.side = self.spec.side or side_param
        if self.spec.ik != "nero_analytic":
            raise ValueError("Nero teleop requires nero_analytic IK adapter")
        cfg = self.profile.teleop_config(self.spec.name)
        self._solver = InteractiveNeroIK(self.spec.lower, self.spec.upper,
                                         refinement_ms=float(cfg.get("ik_refinement_ms", 8.0)))
        self._worker = LatestIKWorker(self._solver)
        self._generation = 0
        self._seed_pending = None
        self._last_ik_report = {}
        # The old base-origin sphere incorrectly clipped legal Nero poses.
        # Optional site workspace bounds remain explicit profile constraints;
        # the default uses the exact shoulder/wrist triangle and joint limits.
        self._explicit_workspace = "workspace_radius" in cfg
        self._processor = PoseProcessor(
            np.array(cfg["vr_to_arm_rot"], dtype=float).reshape(3, 3),
            pos_smoothing=float(cfg.get("pos_smoothing", 0.0 if self.spec.driver == "nero_mujoco" else 0.8)),
            rot_smoothing=float(cfg.get("rot_smoothing", 0.0 if self.spec.driver == "nero_mujoco" else 0.8)),
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
        self._metrics_since = time.monotonic()
        self._solve_ms = []
        self._input_age_ms = []
        self._failed_solves = 0
        self._workspace_clips = 0
        ns = self.profile.namespace
        self._pub = self.create_publisher(
            JointState, self.profile.candidate_topic("teleop", self.spec.name),
            _CANDIDATE_QOS)
        channel = self.spec.input_channel
        self.create_subscription(PoseStamped, f"{ns}/input/{channel}/wrist_pose",
                                 self._on_vr, _CANDIDATE_QOS)
        self.create_subscription(JointState, self.profile.topic(self.spec.name, "joint_states"),
                                 self._on_state, _CANDIDATE_QOS)
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
        self._generation += 1
        self._seed_pending = self._state.copy()
        # Quest3 already reports the Nero link7 flange pose. Anchor and solve
        # directly in the unchanged IK solver's link7 frame.
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
            self._generation += 1

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
        if self._explicit_workspace:
            unclipped = target[:3, 3].copy()
            target[:3, 3] = self._safety.check_workspace(target[:3, 3])
            self._workspace_clips += int(not np.allclose(unclipped, target[:3, 3]))
        result = self._worker.take()
        self._worker.submit(self._generation, target, self._seed_pending)
        self._seed_pending = None
        if result is None or result[0] != self._generation:
            # Never wait for the IK worker on the control callback.
            self._publish_candidate(self._state)
            return
        _, solved_target, solved, report, finished, duration_ms = result
        self._solve_ms.append(duration_ms)
        self._input_age_ms.append((now - self._vr_time) * 1000.)
        self._failed_solves += int(solved is None)
        self._last_ik_report = report
        self._report_metrics()
        if report.get('reason') == 'solver_exception':
            self._armed = False
            self.get_logger().error(f"Nero IK worker failed; teleop disarmed: {report.get('detail')}")
            return
        error = self._solver.residual(solved_target, target)
        obsolete = (now-finished > .1 or np.linalg.norm(error[:3]) > .02
                    or np.linalg.norm(error[3:]) > .1)
        if solved is None or obsolete:
            if solved is None:
                self.get_logger().warning(
                    f"Nero IK hold: reason={report.get('reason')} "
                    f"wrist_distance_m={report.get('wrist_distance_m')} "
                    f"target_position_m={report.get('target_position_m')} "
                    f"method={report.get('method')} solve_ms={duration_ms:.1f}",
                    throttle_duration_sec=2.0)
            if self._feedback_is_fresh():
                self._publish_candidate(self._state)
                self._safety.set_initial_state(self._state, self._solver.fk(self._state)[:3, 3])
            return
        safe, info = self._safety.filter(np.asarray(solved, dtype=float), now - self._last_step)
        self._last_step = now
        if info.get("collision"):
            return
        if self._feedback_is_fresh():
            self._publish_candidate(safe)

    def destroy_node(self):
        self._worker.close()
        return super().destroy_node()

    def _feedback_is_fresh(self) -> bool:
        now = time.monotonic()
        return (now - self._vr_time <= float(self.get_parameter("input_timeout").value)
                and now - self._state_time <= 0.5)

    def _publish_candidate(self, positions) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.spec.joints)
        msg.position = np.asarray(positions, dtype=float).tolist()
        self._pub.publish(msg)

    def _report_metrics(self) -> None:
        now = time.monotonic()
        elapsed = now - self._metrics_since
        if elapsed < 5.0:
            return
        self.get_logger().info(
            f"Nero IK side={self.side} solve_hz={len(self._solve_ms) / elapsed:.1f} "
            f"solve_p95_ms={np.percentile(self._solve_ms, 95):.1f} "
            f"input_age_p95_ms={np.percentile(self._input_age_ms, 95):.1f} "
            f"failed={self._failed_solves} workspace_clips={self._workspace_clips} "
            f"last_method={self._last_ik_report.get('method')} "
            f"last_reason={self._last_ik_report.get('reason')}")
        self._metrics_since = now
        self._solve_ms.clear()
        self._input_age_ms.clear()
        self._failed_solves = self._workspace_clips = 0


def main() -> None:
    rclpy.init()
    node = NeroTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
