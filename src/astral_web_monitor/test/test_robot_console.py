"""Control routing, concurrent operations and recording protection; no hardware."""
import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import HTTPException

from astral_web_monitor import web_server
from astral_web_monitor.robot_console import RobotConsole, capabilities
from nexus_core.profile import Profile

PROFILES = Path(__file__).resolve().parents[2] / "nexus_core" / "profiles"


def test_capabilities_follow_component_drivers():
    physical = capabilities(Profile.load(PROFILES / "nero_dual_xhand.json"))
    assert not physical["viewer"]
    assert [c["dim"] for c in physical["components"]] == [7, 7, 12, 12]
    assert physical["home"]
    assert not physical["components"][2]["home"]
    simulation = capabilities(Profile.load(PROFILES / "nero_dual_xhand_mujoco.json"))
    assert simulation["viewer"]


def test_async_operation_tracks_failure_and_rejects_duplicate():
    async def scenario():
        console = RobotConsole()
        first = console.submit("home", lambda: (False, "home target not reached"))
        assert first["status"] == "running"
        with pytest.raises(ValueError):
            console.submit("home", lambda: (True, "unexpected duplicate"))
        emergency = console.submit("estop", lambda: (True, "software gate latched"))
        await asyncio.gather(*console.tasks)
        assert console.operations[first["id"]]["status"] == "failed"
        assert console.operations[emergency["id"]]["status"] == "succeeded"
    asyncio.run(scenario())


def test_driver_home_uses_long_async_timeout(monkeypatch):
    calls = []
    console = RobotConsole()
    monkeypatch.setattr(web_server, "_robot_console", console)
    monkeypatch.setattr(web_server, "_nexus_profile", Profile.load(PROFILES / "nero_dual_xhand.json"))
    monkeypatch.setattr(web_server._launch_mgr, "_state", "paused")
    monkeypatch.setattr(web_server, "get_node", lambda: SimpleNamespace(
        call_trigger=lambda path, timeout_s: (calls.append((path, timeout_s)) or True, "home complete")))
    async def scenario():
        response = await web_server.nexus_driver("home")
        assert response.ok and response.data["status"] == "running"
        await asyncio.gather(*console.tasks)
        assert calls == [("/nexus/nero/drivers/home", 120.0)]
        assert console.operations[response.data["id"]]["status"] == "succeeded"
    asyncio.run(scenario())


def test_pause_resume_use_canonical_disarm_and_reanchor(monkeypatch):
    events = []
    monkeypatch.setattr(web_server, "_robot_console", RobotConsole())
    monkeypatch.setattr(web_server, "_nexus_profile", Profile.load(PROFILES / "nero_dual_xhand.json"))
    monkeypatch.setattr(web_server._launch_mgr, "_state", "running")
    node = SimpleNamespace(
        publish_nexus_bool=lambda key: events.append(key) or True,
        publish_nexus_teleop_start=lambda: events.append("reanchor") or True)
    monkeypatch.setattr(web_server, "get_node", lambda: node)
    async def scenario():
        await web_server.pause()
        assert web_server._launch_mgr.state == "paused"
        await web_server.resume()
        assert web_server._launch_mgr.state == "running"
        assert events == ["teleop_disarm", "reanchor"]
    asyncio.run(scenario())


def test_stop_refuses_active_recording(monkeypatch):
    monkeypatch.setattr(web_server, "_nexus_profile", Profile.load(PROFILES / "nero_dual_xhand.json"))
    monkeypatch.setattr(web_server._launch_mgr, "_state", "running")
    monkeypatch.setattr(web_server, "get_node", lambda: SimpleNamespace(
        nexus_snapshot=lambda: {"data_collect": {"state": "RECORDING"}}))
    with pytest.raises(HTTPException) as error:
        asyncio.run(web_server.stop())
    assert error.value.status_code == 409


def test_legacy_hardware_mode_cannot_control_active_nero(monkeypatch):
    monkeypatch.setattr(web_server, "_nexus_profile", Profile.load(PROFILES / "nero_dual_xhand.json"))
    monkeypatch.setattr(web_server._launch_mgr, "_state", "running")
    with pytest.raises(HTTPException) as error:
        asyncio.run(web_server.robot_damping())
    assert error.value.status_code == 409


def test_run_report_preserves_profile_and_events(tmp_path):
    profile = Profile.load(PROFILES / "nero_dual_xhand.json")
    console = RobotConsole()
    console.begin(profile, {"dry_run": True, "session": "test"}, tmp_path)
    console.event("pause")
    report = console.report({"control": {"mode": "IDLE"}}, ["test log"], [])
    assert report["profile_sha256"] == profile.digest
    assert report["profile"] == profile.raw
    assert report["events"][-1]["action"] == "pause"
    assert console.path.is_file()


def test_single_arm_profile_needs_no_frontend_robot_branch():
    import copy
    raw = copy.deepcopy(Profile.load(PROFILES / "astral_dual_gripper.json").raw)
    raw["profile_id"] = "astral_single_gripper_web_test"
    raw["components"] = [c for c in raw["components"] if c["name"] in ("left_arm", "left_ee")]
    single = capabilities(Profile(raw))
    assert [c["dim"] for c in single["components"]] == [7, 1]
    assert len(single["components"]) == 2
