import unittest
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from unittest.mock import patch

import numpy as np
from nexus_core.adapter_registry import DRIVERS
from nexus_core.profile import Profile
from nero_quest_teleop.ik_solver import IKSolver

from nero_mujoco_sim.model_builder import build_mjcf_from_urdf
from nero_mujoco_sim.homing import is_pre_home_command
from nero_mujoco_sim.nexus_adapter import driver_adapter, launch_driver


ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / "src" / "xhand_nero_description" / "urdf" / "xhand_nero_description.urdf"
PROFILE_JOINTS = [
    *[f"{side}_joint{i}" for side in ("left", "right") for i in range(1, 8)],
    *[f"{side}_hand_{joint}" for side in ("left", "right") for joint in (
        "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
        "index_bend_joint", "index_joint1", "index_joint2", "mid_joint1",
        "mid_joint2", "ring_joint1", "ring_joint2", "pinky_joint1", "pinky_joint2")],
]


class ModelBuilderTests(unittest.TestCase):
    def test_old_hold_message_cannot_cancel_homing(self):
        from types import SimpleNamespace

        home_stamp_ns = 5_000_000_000
        delayed_hold = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
            sec=4, nanosec=999_999_999)))
        fresh_command = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
            sec=5, nanosec=1)))
        self.assertTrue(is_pre_home_command(delayed_hold, home_stamp_ns))
        self.assertFalse(is_pre_home_command(fresh_command, home_stamp_ns))

    def test_sim_profile_loads_through_the_driver_plugin(self):
        adapter = driver_adapter()
        with patch.dict(DRIVERS, {adapter.name: adapter}):
            profile = Profile.load(ROOT / "src" / "nexus_core" / "profiles" /
                                   "nero_dual_xhand_mujoco.json")
        self.assertEqual(profile.dimension, 38)
        self.assertEqual({component.driver for component in profile.components}, {"nero_mujoco"})
        self.assertEqual(profile.instance, "nero_sim")
        self.assertEqual(profile.adapter_config("nero_mujoco")["timestep"], 0.002)

    def test_viewer_override_only_changes_sim_process_parameter(self):
        adapter = driver_adapter()
        profile_path = ROOT / "src" / "nexus_core" / "profiles" / "nero_dual_xhand_mujoco.json"
        with patch.dict(DRIVERS, {adapter.name: adapter}):
            profile = Profile.load(profile_path)
        digest = profile.digest
        with patch.dict(os.environ, {"NEXUS_MUJOCO_VIEWER": "1"}):
            with patch("launch_ros.actions.Node") as node:
                launch_driver(profile, profile.components, str(profile_path), False)
        self.assertTrue(node.call_args_list[0].kwargs["parameters"][0]["enable_viewer"])
        self.assertEqual(profile.digest, digest)

    def test_builds_full_dual_arm_dual_hand_mjcf(self):
        xml = build_mjcf_from_urdf(URDF, {"left_joint2": (-1.74533, 1.74533)})
        root = ET.fromstring(xml)
        joints = {joint.get("name"): joint for joint in root.findall(".//joint")}
        actuators = {act.get("joint"): act for act in root.findall("./actuator/position")}
        self.assertTrue(set(PROFILE_JOINTS).issubset(joints))
        self.assertEqual(set(PROFILE_JOINTS), set(actuators))
        self.assertEqual(joints["left_joint2"].get("range"), "-1.74533 1.74533")
        self.assertEqual(actuators["left_joint5"].get("kp"), "100")
        self.assertEqual(actuators["left_joint5"].get("kv"), "10")
        self.assertIsNotNone(root.find(".//body[@name='pick_cube']"))
        self.assertIsNotNone(root.find(".//geom[@name='table_top']"))

    def test_muJoCo_compiles_and_steps_when_dependency_is_installed(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest("install mujoco to run the physics integration test")
        xml = build_mjcf_from_urdf(URDF, {joint: (-1.74533, 1.74533)
                                          for joint in PROFILE_JOINTS[:14]})
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        self.assertEqual(model.nu, len(PROFILE_JOINTS))
        mujoco.mj_forward(model, data)
        self.assertEqual(data.ncon, 0, "robot or props start in penetrating contact")
        for name in ("left_joint1", "left_hand_index_joint1"):
            actuator = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{name}")
            data.ctrl[actuator] = 0.25
        qpos_addresses = {
            name: int(model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])
            for name in ("left_joint1", "left_hand_index_joint1")
        }
        for _ in range(1000):
            mujoco.mj_step(model, data)
        for name, address in qpos_addresses.items():
            self.assertGreater(float(data.qpos[address]), 0.05, name)
        self.assertGreater(data.ncon, 0, "pick cube never contacted the table")

    def test_mirrored_wrist_servos_settle_without_a_joint_five_limit_cycle(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest("install mujoco to run the physics integration test")
        xml = build_mjcf_from_urdf(URDF)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        targets = {"left_joint5": (-0.294, 0.285), "right_joint5": (-0.342, 0.073)}
        addresses = {}
        for name, (target, initial) in targets.items():
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{name}")
            addresses[name] = int(model.jnt_qposadr[joint])
            data.qpos[addresses[name]] = initial
            data.ctrl[actuator] = target
        mujoco.mj_forward(model, data)
        for _ in range(250):
            mujoco.mj_step(model, data)
        for name, (target, _initial) in targets.items():
            error = abs(float(data.qpos[addresses[name]]) - target)
            self.assertLess(error, 0.02, f"{name} did not settle: error={error:.4f} rad")

    def test_mujoco_frames_match_unchanged_nero_ik_fk_and_xhand_tcp(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest("install mujoco to run the kinematic parity test")
        profile_path = ROOT / "src" / "nexus_core" / "profiles" / "nero_dual_xhand_mujoco.json"
        profile = Profile.load(profile_path, validate_plugins=False)
        sim = profile.adapter_config("nero_mujoco")
        offsets = sim["joint_zero_offsets"]
        limits = {joint: (low, high) for component in profile.components
                  for joint, low, high in zip(component.joints, component.lower, component.upper)}
        xml = build_mjcf_from_urdf(URDF, limits, joint_zero_offsets=offsets)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        poses = {
            "home": sim["home_pose"],
            "neutral": {
                "left": [0.0, 0.0, 0.0, 1.2, 0.0, 0.4, 0.0],
                "right": [0.0, 0.0, 0.0, 1.2, 0.0, 0.4, 0.0],
            },
            "probe": {
                "left": [0.3, -0.4, 0.2, 1.0, -0.3, 0.2, 0.4],
                "right": [-0.3, -0.4, -0.2, 1.0, 0.3, 0.2, -0.4],
            },
        }
        solvers = {side: IKSolver() for side in ("left", "right")}
        arm_component = {side: profile.component(f"{side}_arm") for side in ("left", "right")}

        def body_transform(body_id):
            transform = np.eye(4)
            transform[:3, :3] = data.xmat[body_id].reshape(3, 3)
            transform[:3, 3] = data.xpos[body_id]
            return transform

        def relative_transform(parent_id, child_id):
            return np.linalg.inv(body_transform(parent_id)) @ body_transform(child_id)

        def rotation_error_deg(first, second):
            delta = first[:3, :3].T @ second[:3, :3]
            return np.degrees(np.arccos(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0)))

        for side in ("left", "right"):
            component = arm_component[side]
            solver = solvers[side]
            base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_base_link")
            link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_link7")
            tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_hand_ee_link")
            self.assertGreaterEqual(base_id, 0)
            self.assertGreaterEqual(link_id, 0)
            self.assertGreaterEqual(tcp_id, 0)
            offset = np.asarray(
                component.frame_transforms["link7_to_xhand_palm"], dtype=float
            )
            solver_to_tcp = np.eye(4)
            from scipy.spatial.transform import Rotation
            solver_to_tcp[:3, :3] = Rotation.from_euler("xyz", offset[3:]).as_matrix()
            solver_to_tcp[:3, 3] = offset[:3]
            for pose_name, per_side in poses.items():
                q = np.asarray(per_side[side], dtype=float)
                self.assertTrue(np.all(q >= np.asarray(component.lower)))
                self.assertTrue(np.all(q <= np.asarray(component.upper)))
                for joint_name, value in zip(component.joints, q):
                    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    data.qpos[model.jnt_qposadr[joint_id]] = value
                mujoco.mj_forward(model, data)
                ik_fk = solver.fk(q)
                sim_link7 = relative_transform(base_id, link_id)
                sim_tcp = relative_transform(base_id, tcp_id)
                ik_tcp = ik_fk @ solver_to_tcp
                self.assertLess(np.linalg.norm(sim_link7[:3, 3] - ik_fk[:3, 3]), 3e-5,
                                f"{side} {pose_name} link7 FK position mismatch")
                self.assertLess(rotation_error_deg(sim_link7, ik_fk), 0.001,
                                f"{side} {pose_name} link7 FK orientation mismatch")
                self.assertLess(np.linalg.norm(sim_tcp[:3, 3] - ik_tcp[:3, 3]), 3e-5,
                                f"{side} {pose_name} XHand TCP position mismatch")
                self.assertLess(rotation_error_deg(sim_tcp, ik_tcp), 0.001,
                                f"{side} {pose_name} XHand TCP orientation mismatch")


if __name__ == "__main__":
    unittest.main()
