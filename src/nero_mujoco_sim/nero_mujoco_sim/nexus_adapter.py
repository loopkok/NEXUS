"""NEXUS plugin exposing the physics simulator as an assembly driver."""

from __future__ import annotations

import os

from nexus_core.adapter_registry import Adapter


def _joint_names(kind: str, side: str | None) -> list[str] | None:
    if kind == "arm":
        label = side or "arm"
        return [f"{label}_joint{i}" for i in range(1, 8)]
    if kind == "hand":
        label = side or "hand"
        names = (
            "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
            "index_bend_joint", "index_joint1", "index_joint2",
            "mid_joint1", "mid_joint2", "ring_joint1", "ring_joint2",
            "pinky_joint1", "pinky_joint2",
        )
        return [f"{label}_hand_{name}" for name in names]
    return None


def driver_adapter() -> Adapter:
    return Adapter(
        name="nero_mujoco",
        kinds=frozenset({"arm", "hand"}),
        scope="assembly",
        feedback=frozenset({"measured"}),
        joint_names=_joint_names,
        supports_home=True,
        launcher="nero_mujoco_sim.nexus_adapter:launch_driver",
        home_target=_home_target,
        home_tolerance=0.1,
    )


def _home_target(profile: dict, kind: str, side: str | None, dimension: int) -> list[float] | None:
    """Return the measured joint target the manager should wait for after home."""
    if kind == "hand":
        return [0.0] * dimension
    if kind != "arm" or side is None:
        return None
    return profile.get("adapter_config", {}).get("nero_mujoco", {}).get(
        "home_pose", {}).get(side)


def launch_driver(profile, components, path: str, _dry_run: bool):
    """Start one simulation process and XHand candidate bridges for a profile."""
    from launch_ros.actions import Node

    config = profile.adapter_config("nero_mujoco")
    # Presentation is a per-launch option, not part of the frozen robot layout.
    # The Web launcher sets this only for the simulation subprocess.
    viewer_env = os.environ.get("NEXUS_MUJOCO_VIEWER")
    if viewer_env not in (None, "0", "1"):
        raise ValueError("NEXUS_MUJOCO_VIEWER must be 0 or 1")
    enable_viewer = (bool(config.get("enable_viewer", True)) if viewer_env is None
                     else viewer_env == "1")
    parameters = {
        "profile_file": path,
        "enable_viewer": enable_viewer,
        "realtime": bool(config.get("realtime", True)),
        "simulation_mode": str(config.get("simulation_mode", "kinematic")),
        "state_rate": float(config.get("state_rate", 100.0)),
        "command_timeout": float(config.get("command_timeout", 0.5)),
        "homing_timeout": float(config.get("homing_timeout", 45.0)),
        "timestep": float(config.get("timestep", 0.002)),
        "viewer_rate": float(config.get("viewer_rate", 30.0)),
    }
    if config.get("urdf_file"):
        parameters["urdf_file"] = str(config["urdf_file"])
    actions = [Node(
        package="nero_mujoco_sim",
        executable="nero_mujoco_sim_node",
        name="nero_mujoco_sim",
        output="screen",
        parameters=[parameters],
    )]
    # The XHand retargeter emits the vendor XHandCommand message. Keep the
    # standard bridge at the adapter boundary so the mux only sees JointState.
    for component in components:
        if component.kind == "hand":
            actions.append(Node(
                package="nexus_core",
                executable="nexus_joint_bridge",
                name=f"mujoco_hand_bridge_{component.name}",
                output="screen",
                parameters=[{
                    "profile_file": path,
                    "component": component.name,
                    "side": component.side,
                    "adapter_mode": "xhand",
                    "candidate_only": True,
                }],
            ))
    return actions
