"""NEXUS joint-command driver backed by a MuJoCo physics simulation."""

from __future__ import annotations

import math
import functools
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from nexus_core.profile import Profile

from .homing import is_pre_home_command
from .model_builder import build_mjcf_from_urdf


def _physics_locked(method):
    """Serialize ROS lifecycle callbacks with access to MuJoCo state."""
    @functools.wraps(method)
    def call(self, *args, **kwargs):
        with self._physics_lock:
            return method(self, *args, **kwargs)
    return call


class NeroMujocoSimNode(Node):
    """Physics-backed driver for every component in a NEXUS assembly profile."""

    def __init__(self):
        super().__init__("nero_mujoco_sim")
        self._physics_lock = threading.RLock()
        self._run_stop = threading.Event()
        self.declare_parameter("profile_file", "")
        self.declare_parameter("urdf_file", "")
        self.declare_parameter("enable_viewer", True)
        self.declare_parameter("realtime", True)
        self.declare_parameter("simulation_mode", "kinematic")
        self._simulation_mode = str(self.get_parameter("simulation_mode").value)
        if self._simulation_mode not in ("kinematic", "physics"):
            raise ValueError("simulation_mode must be kinematic or physics")
        self.declare_parameter("state_rate", 100.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("homing_timeout", 45.0)
        self.declare_parameter("timestep", 0.002)
        self.declare_parameter("viewer_rate", 30.0)
        self._viewer_rate = float(self.get_parameter("viewer_rate").value)
        if not math.isfinite(self._viewer_rate) or not 1.0 <= self._viewer_rate <= 120.0:
            raise ValueError("viewer_rate must be between 1 and 120 Hz")

        profile_path = str(self.get_parameter("profile_file").value).strip()
        if not profile_path:
            raise ValueError("profile_file is required")
        self.profile = Profile.load(profile_path)
        if not self.profile.components:
            raise ValueError("simulation profile has no components")
        if any(component.driver != "nero_mujoco" for component in self.profile.components):
            raise ValueError("all simulated components must select driver=nero_mujoco")
        self.namespace = self.profile.namespace
        self.specs = {component.name: component for component in self.profile.components}
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        if self._command_timeout <= 0:
            raise ValueError("command_timeout must be positive")
        self._homing_timeout = float(self.get_parameter("homing_timeout").value)
        if self._homing_timeout <= self._command_timeout:
            raise ValueError("homing_timeout must exceed command_timeout")

        urdf_file = str(self.get_parameter("urdf_file").value).strip()
        if not urdf_file:
            urdf_file = str(Path(get_package_share_directory("xhand_nero_description")) /
                            "urdf" / "xhand_nero_description.urdf")
        joint_limits = {
            joint: (lower, upper)
            for component in self.profile.components
            for joint, lower, upper in zip(component.joints, component.lower, component.upper)
        }
        sim_config = self.profile.adapter_config("nero_mujoco")
        joint_zero_offsets = {
            str(name): float(value)
            for name, value in sim_config.get("joint_zero_offsets", {}).items()
        }
        timestep = float(self.get_parameter("timestep").value)
        if not math.isfinite(timestep) or not 0.0005 <= timestep <= 0.01:
            raise ValueError("timestep must be between 0.0005 and 0.01 seconds")
        mjcf = build_mjcf_from_urdf(
            urdf_file, joint_limits, joint_zero_offsets=joint_zero_offsets,
            timestep=timestep)
        try:
            import mujoco
            import mujoco.viewer
        except ImportError as exc:
            raise RuntimeError("MuJoCo is missing; install it with `python3 -m pip install mujoco`") from exc
        self._mujoco = mujoco
        self._viewer_module = mujoco.viewer
        self.model = mujoco.MjModel.from_xml_string(mjcf)
        self.data = mujoco.MjData(self.model)

        self._joint_ids: dict[str, list[int]] = {}
        self._actuator_ids: dict[str, list[int]] = {}
        self._component_publishers = {}
        self._targets: dict[str, list[float]] = {}
        self._enabled = {name: False for name in self.specs}
        self._estopped = False
        self._homing_started: dict[str, float] = {}
        self._homing_started_ros_ns: dict[str, int] = {}
        self._last_commands = {name: 0.0 for name in self.specs}
        self._state_time = 0.0
        self._timeout_reported = set()
        for component in self.profile.components:
            joint_ids = []
            actuator_ids = []
            initial = []
            for name, lower, upper in zip(component.joints, component.lower, component.upper):
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                actuator_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{name}")
                if joint_id < 0 or actuator_id < 0:
                    raise RuntimeError(f"MuJoCo model lacks joint/position actuator for {name!r}")
                joint_ids.append(int(joint_id))
                actuator_ids.append(int(actuator_id))
                initial.append(min(upper, max(lower, 0.0)))
            self._joint_ids[component.name] = joint_ids
            self._actuator_ids[component.name] = actuator_ids
            self._targets[component.name] = initial.copy()
            self._component_publishers[component.name] = self.create_publisher(
                JointState, self.profile.topic(component.name, "joint_states"),
                qos_profile_sensor_data)
            self.create_subscription(
                JointState, self.profile.topic(component.name, "joint_commands"),
                lambda msg, name=component.name: self._on_command(name, msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self._create_lifecycle_services(component.name)

        for component in self.profile.components:
            for name, value in zip(component.joints, self._targets[component.name]):
                jid = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_JOINT, name)
                self.data.qpos[self.model.jnt_qposadr[jid]] = value
        self._mujoco.mj_forward(self.model, self.data)
        self._last_state_published = time.monotonic()
        self.create_timer(1.0 / max(1.0, float(self.get_parameter("state_rate").value)),
                          self._publish_states)
        self.get_logger().info(
            f"MuJoCo physics ready profile={self.profile.profile_id} "
            f"sha256={self.profile.digest} nq={self.model.nq} nv={self.model.nv} "
            f"components={list(self.specs)} urdf={urdf_file}")

    def _create_lifecycle_services(self, component: str) -> None:
        prefix = f"{self.namespace}/drivers/{component}"
        self.create_service(Trigger, f"{prefix}/ready",
                            lambda req, res, name=component: self._ready(name, req, res))
        self.create_service(Trigger, f"{prefix}/enable",
                            lambda req, res, name=component: self._enable(name, req, res))
        self.create_service(Trigger, f"{prefix}/home",
                            lambda req, res, name=component: self._home(name, req, res))
        self.create_service(Trigger, f"{prefix}/estop",
                            lambda req, res, name=component: self._estop(name, req, res))

    @_physics_locked
    def _state(self, component: str) -> list[float]:
        spec = self.specs[component]
        values = []
        for name, lower, upper, joint_id in zip(
                spec.joints, spec.lower, spec.upper, self._joint_ids[component]):
            address = int(self.model.jnt_qposadr[joint_id])
            value = float(self.data.qpos[address])
            values.append(min(upper, max(lower, value)))
        return values

    @_physics_locked
    def _on_command(self, component: str, msg: JointState) -> None:
        spec = self.specs[component]
        if self._estopped or not self._enabled[component]:
            return
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns > 0 and (self.get_clock().now().nanoseconds - stamp_ns) / 1e9 > self._command_timeout:
            self.get_logger().warning(f"{component}: expired command rejected",
                                      throttle_duration_sec=2.0)
            return
        homing_stamp = self._homing_started_ros_ns.get(component)
        if homing_stamp is not None and is_pre_home_command(msg, homing_stamp):
            # An IDLE hold can already be in DDS when the mux reports HOMING.
            # It predates this request and must not cancel the adapter-owned
            # trajectory when its callback arrives late.
            return
        if list(msg.name) != list(spec.joints) or len(msg.position) != spec.dim:
            self.get_logger().error(
                f"{component}: final command has wrong joint names/order/dimension",
                throttle_duration_sec=2.0)
            return
        values = [float(value) for value in msg.position]
        if any(not math.isfinite(value) or value < lower or value > upper
               for value, lower, upper in zip(values, spec.lower, spec.upper)):
            self.get_logger().error(f"{component}: final command outside configured limits",
                                    throttle_duration_sec=2.0)
            return
        self._targets[component] = values
        self._homing_started.pop(component, None)
        self._homing_started_ros_ns.pop(component, None)
        self._last_commands[component] = time.monotonic()
        self._timeout_reported.discard(component)

    @_physics_locked
    def _publish_states(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for component, spec in self.specs.items():
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "world"
            msg.name = list(spec.joints)
            msg.position = self._state(component)
            self._component_publishers[component].publish(msg)
        self._last_state_published = time.monotonic()

    @staticmethod
    def _result(response, success: bool, message: str):
        response.success = success
        response.message = message
        return response

    @_physics_locked
    def _ready(self, component: str, _request, response):
        fresh = time.monotonic() - self._last_state_published < 0.5
        return self._result(response, fresh,
                            "fresh MuJoCo state" if fresh else "MuJoCo state is stale")

    @_physics_locked
    def _enable(self, component: str, _request, response):
        if self._estopped:
            return self._result(response, False, "simulation estop is latched; restart to reset")
        if time.monotonic() - self._last_state_published >= 0.5:
            return self._result(response, False, "fresh MuJoCo feedback required")
        self._targets[component] = self._state(component)
        self._last_commands[component] = time.monotonic()
        self._enabled[component] = True
        return self._result(response, True, f"{component} enabled; holding measured pose")

    @_physics_locked
    def _home(self, component: str, _request, response):
        if self._estopped or not self._enabled[component]:
            return self._result(response, False, "enable the simulation component before home")
        spec = self.specs[component]
        home = [0.0] * spec.dim
        if spec.kind == "arm":
            config = self.profile.adapter_config("nero_mujoco")
            targets = config.get("home_pose", {})
            if spec.side in targets:
                home = [float(value) for value in targets[spec.side]]
        if len(home) != spec.dim or any(value < lo or value > hi
                                        for value, lo, hi in zip(home, spec.lower, spec.upper)):
            return self._result(response, False, f"{component} home pose does not match its profile")
        self._targets[component] = home
        self._homing_started[component] = time.monotonic()
        self._homing_started_ros_ns[component] = int(self.get_clock().now().nanoseconds)
        self._last_commands[component] = time.monotonic()
        return self._result(response, True, f"{component} homing target accepted; waiting for measured convergence")

    @_physics_locked
    def _estop(self, _component: str, _request, response):
        self._estopped = True
        for name in self.specs:
            self._enabled[name] = False
            self._targets[name] = self._state(name)
        return self._result(response, True, "simulation estop latched")

    @_physics_locked
    def _step_physics(self) -> None:
        now = time.monotonic()
        for component, actuator_ids in self._actuator_ids.items():
            if self._enabled[component] and not self._estopped:
                homing_since = self._homing_started.get(component)
                if homing_since is not None and now - homing_since > self._homing_timeout:
                    self._targets[component] = self._state(component)
                    self._homing_started.pop(component, None)
                    self._homing_started_ros_ns.pop(component, None)
                    self.get_logger().error(f"{component}: homing timed out; holding measured pose")
                elif homing_since is None and now - self._last_commands[component] > self._command_timeout:
                    self._targets[component] = self._state(component)
                    if component not in self._timeout_reported:
                        self.get_logger().error(
                            f"{component}: command timeout; holding measured pose")
                        self._timeout_reported.add(component)
            else:
                self._targets[component] = self._state(component)
            for actuator_id, target in zip(actuator_ids, self._targets[component]):
                self.data.ctrl[actuator_id] = target
        if self._simulation_mode == "kinematic":
            # Interactive teleop follows the accepted joint targets directly,
            # like Astral. No position-actuator settling or gravity response.
            for component, joint_ids in self._joint_ids.items():
                for joint_id, target in zip(joint_ids, self._targets[component]):
                    self.data.qpos[self.model.jnt_qposadr[joint_id]] = target
            self.data.qvel[:] = 0.0
            self.data.qacc[:] = 0.0
            self.data.time += float(self.model.opt.timestep)
            self._mujoco.mj_forward(self.model, self.data)
        else:
            self._mujoco.mj_step(self.model, self.data)

    def run(self) -> None:
        viewer = None
        viewer_threads = set()
        if bool(self.get_parameter("enable_viewer").value):
            threads_before = set(threading.enumerate())
            viewer = self._viewer_module.launch_passive(self.model, self.data)
            viewer_threads = set(threading.enumerate()) - threads_before
            viewer.cam.distance = 2.0
            viewer.cam.lookat[:] = [0.25, 0.0, 0.55]
            viewer.cam.elevation = -18
        realtime = bool(self.get_parameter("realtime").value)
        timestep = float(self.model.opt.timestep)
        report_wall = time.monotonic()
        report_sim = float(self.data.time)
        steps = 0
        syncs = 0
        sync_max_ms = 0.0
        next_sync = time.monotonic()
        next_step = time.monotonic()
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        def spin():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass
        ros_thread = threading.Thread(target=spin, name="nero-sim-ros", daemon=True)
        ros_thread.start()
        try:
            while rclpy.ok() and not self._run_stop.is_set() and (viewer is None or viewer.is_running()):
                # ROS reception must drain independently of rendering/physics;
                # one callback per step starved four 100 Hz command streams.
                self._step_physics()
                now = time.monotonic()
                if viewer is not None and now >= next_sync:
                    with self._physics_lock:
                        viewer.sync()
                    sync_max_ms = max(sync_max_ms, (time.monotonic() - now) * 1000.0)
                    syncs += 1
                    next_sync = time.monotonic() + 1.0 / self._viewer_rate
                steps += 1
                now = time.monotonic()
                if now - report_wall >= 5.0:
                    elapsed = now - report_wall
                    sim_rate = (float(self.data.time) - report_sim) / elapsed
                    self.get_logger().info(
                        f"MuJoCo mode={self._simulation_mode} physics_hz={steps / elapsed:.1f} "
                        f"real_time_factor={sim_rate:.2f} contacts={self.data.ncon} "
                        f"viewer_sync_hz={syncs / elapsed:.1f} viewer_sync_max_ms={sync_max_ms:.1f}")
                    report_wall, report_sim, steps = now, float(self.data.time), 0
                    syncs, sync_max_ms = 0, 0.0
                if realtime:
                    # Absolute deadlines recover short render stalls, with a
                    # bounded catch-up budget after long pauses.
                    next_step = max(next_step + timestep, time.monotonic() - 4 * timestep)
                    remaining = next_step - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
        finally:
            executor.shutdown(timeout_sec=2.0)
            ros_thread.join(timeout=2.0)
            executor.remove_node(self)
            if viewer is not None:
                viewer.close()
                # close() only requests native viewer exit. Join its Python
                # owner before interpreter teardown destroys GL resources.
                for thread in viewer_threads:
                    thread.join(timeout=5.0)

    def request_stop(self) -> None:
        self._run_stop.set()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NeroMujocoSimNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
