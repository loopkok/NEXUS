"""Compose a NEXUS assembly from installed adapter plugins and one profile."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nexus_core.adapter_registry import (
    CAMERA_ADAPTERS,
    DRIVERS,
    IK_ADAPTERS,
    INPUT_ADAPTERS,
    RETARGETERS,
    load_target,
)
from nexus_core.profile import Profile


def _plugin_call(target: str, *args):
    return load_target(target)(*args)


def _group_by(items, key):
    grouped = {}
    for item in items:
        value = key(item)
        if value is not None:
            grouped.setdefault(value, []).append(item)
    return grouped


def _input_channels(profile: Profile) -> dict[str, list[str]]:
    grouped: dict[str, list[str]] = {}
    for channel, row in profile.raw["inputs"].items():
        sources = {profile.input_spec(channel, semantic)["source"]
                   for semantic in row
                   if profile.input_spec(channel, semantic) is not None}
        for source in sources - {"none"}:
            grouped.setdefault(source, []).append(channel)
    return grouped


def _preflight(profile: Profile, dry_run: bool, with_cameras: bool) -> None:
    if dry_run:
        return
    drivers = _group_by(profile.components, lambda c: c.driver)
    for name, components in drivers.items():
        hook = DRIVERS[name].preflight
        if hook:
            _plugin_call(hook, profile, components)
    for source, channels in _input_channels(profile).items():
        hook = INPUT_ADAPTERS[source].preflight
        if hook:
            _plugin_call(hook, profile, channels)
    if with_cameras:
        cameras = _group_by(profile.raw["cameras"], lambda c: c["source"])
        for source, rows in cameras.items():
            hook = CAMERA_ADAPTERS[source].preflight
            if hook:
                _plugin_call(hook, profile, rows)


def _launch_adapters(profile: Profile, path: str, dry_run: bool, with_cameras: bool):
    actions = []

    # Drivers can share one assembly process or run one process per component.
    for name, components in _group_by(profile.components, lambda c: c.driver).items():
        adapter = DRIVERS[name]
        if adapter.scope == "assembly":
            actions.extend(_plugin_call(adapter.launcher, profile, components, path, dry_run))
        elif adapter.scope == "component":
            for component in components:
                actions.extend(_plugin_call(adapter.launcher, profile, [component], path, dry_run))
        else:
            raise RuntimeError(f"driver adapter {name} has invalid scope {adapter.scope!r}")

    for component in profile.components:
        if component.ik:
            adapter = IK_ADAPTERS[component.ik]
            actions.extend(_plugin_call(adapter.launcher, profile, [component], path, dry_run))

    for name, components in _group_by(profile.components, lambda c: c.retargeter).items():
        adapter = RETARGETERS[name]
        if adapter.scope == "assembly":
            actions.extend(_plugin_call(adapter.launcher, profile, components, path, dry_run))
        elif adapter.scope == "component":
            for component in components:
                actions.extend(_plugin_call(adapter.launcher, profile, [component], path, dry_run))
        else:
            raise RuntimeError(f"retargeter adapter {name} has invalid scope {adapter.scope!r}")

    for source, channels in _input_channels(profile).items():
        adapter = INPUT_ADAPTERS[source]
        actions.extend(_plugin_call(adapter.launcher, profile, channels, path, dry_run))

    if with_cameras:
        cameras = _group_by(profile.raw["cameras"], lambda c: c["source"])
        for source, rows in cameras.items():
            adapter = CAMERA_ADAPTERS[source]
            actions.extend(_plugin_call(adapter.launcher, profile, rows, path, dry_run))
    return actions


def _setup(context):
    profile_name = LaunchConfiguration("profile").perform(context).strip()
    if not profile_name or (Path(profile_name).name != profile_name and not Path(profile_name).is_file()):
        raise RuntimeError("profile must be a built-in name or an existing JSON file")
    path = Path(profile_name)
    if not path.is_file():
        path = Path(get_package_share_directory("nexus_core")) / "profiles" / f"{profile_name}.json"
    profile = Profile.load(path)
    if profile.raw.get("schema_version") != 2:
        raise RuntimeError("system launch requires a schema v2 assembly profile; v1 is for old data metadata")
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() in ("true", "1", "yes")
    with_cameras = LaunchConfiguration("with_cameras").perform(context).lower() in ("true", "1", "yes")
    with_recording = LaunchConfiguration("with_recording").perform(context).lower() in ("true", "1", "yes")
    with_policy = LaunchConfiguration("with_policy").perform(context).lower() in ("true", "1", "yes")
    _preflight(profile, dry_run, with_cameras)

    ns = profile.namespace
    actions = [
        LogInfo(msg=f"NEXUS profile={profile.profile_id} instance={profile.instance} "
                    f"sha256={profile.digest} dry_run={dry_run}"),
        Node(package="nexus_core", executable="nexus_command_mux", name="nexus_command_mux",
             output="screen", parameters=[{"profile_file": str(path)}]),
        Node(package="nexus_core", executable="nexus_driver_manager", name="nexus_driver_manager",
             output="screen", parameters=[{"profile_file": str(path)}]),
        Node(package="nexus_core", executable="nexus_input_bridge", name="nexus_input_bridge",
             output="screen", parameters=[{"profile_file": str(path)}]),
    ]
    actions.extend(_launch_adapters(profile, str(path), dry_run, with_cameras))

    if with_recording:
        actions.append(Node(
            package="astral_data_collect", executable="data_collect_node", name="nexus_data_collect",
            output="screen", parameters=[{
                "profile_file": str(path),
                "save_root": LaunchConfiguration("data_root").perform(context),
                "session": LaunchConfiguration("session").perform(context),
            }], remappings=[
                ("/teleop/start", f"{ns}/control/teleop_start"),
                ("/teleop/disarm", f"{ns}/control/teleop_disarm"),
                ("/data_collect/control", f"{ns}/data/collect/control"),
                ("/data_collect/task", f"{ns}/data/collect/task"),
                ("/data_collect/session", f"{ns}/data/collect/session"),
                ("/data_collect/state", f"{ns}/data/collect/state"),
            ]))
    if with_policy:
        actions.append(Node(
            package="nexus_core", executable="nexus_policy", name="nexus_policy",
            output="screen", parameters=[{
                "profile_file": str(path),
                "model_manifest": LaunchConfiguration("model_manifest").perform(context),
                "backend_type": LaunchConfiguration("backend_type").perform(context),
                "model": LaunchConfiguration("model").perform(context),
                "host": LaunchConfiguration("server_host").perform(context),
                "port": int(LaunchConfiguration("server_port").perform(context)),
                "replay_path": LaunchConfiguration("replay_path").perform(context),
            }], remappings=[
                ("/policy_inference/state", f"{ns}/policy/state"),
                ("/policy_inference/cmd", f"{ns}/policy/cmd"),
                ("/policy_inference/task", f"{ns}/policy/task"),
                ("/teleop/disarm", f"{ns}/control/teleop_disarm"),
            ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="astral_gripper_wuji"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("with_cameras", default_value="false"),
        DeclareLaunchArgument("with_recording", default_value="true"),
        DeclareLaunchArgument("with_policy", default_value="true"),
        DeclareLaunchArgument("data_root", default_value="~/nexus_data"),
        DeclareLaunchArgument("session", default_value="default_task"),
        DeclareLaunchArgument("model_manifest", default_value=""),
        DeclareLaunchArgument("backend_type", default_value="remote"),
        DeclareLaunchArgument("model", default_value="act"),
        DeclareLaunchArgument("server_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("server_port", default_value="8001"),
        DeclareLaunchArgument("replay_path", default_value=""),
        OpaqueFunction(function=_setup),
    ])
