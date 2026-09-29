"""The Web Nero simulation must launch real MuJoCo with a per-process viewer flag."""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path

import pytest
from fastapi import HTTPException

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from astral_web_monitor import web_server  # noqa: E402
from astral_web_monitor.launch_manager import STOPPED  # noqa: E402


PROFILE = (Path(__file__).resolve().parents[2] / "nexus_core" / "profiles" /
           "nero_dual_xhand_mujoco.json")


def _prepare(monkeypatch, tmp_path):
    launched = []
    monkeypatch.setattr(web_server, "_nexus_profile_file", lambda _name: PROFILE)
    monkeypatch.setattr(web_server, "get_node", lambda: None)
    monkeypatch.setattr(web_server._launch_mgr, "start", lambda preset: (launched.append(preset) or True, "started"))
    monkeypatch.setattr(web_server._collect_mgr, "_state", STOPPED)
    monkeypatch.setattr(web_server._policy_mgr, "_state", STOPPED)
    monkeypatch.setenv("NEXUS_DATA_ROOT", str(tmp_path))
    return launched


def test_nero_web_launch_uses_sim_and_desktop_viewer(monkeypatch, tmp_path):
    launched = _prepare(monkeypatch, tmp_path)
    monkeypatch.setenv("DISPLAY", ":0")
    result = asyncio.run(web_server.nexus_start({
        "profile": "nero_dual_xhand_mujoco", "session": "quest_live",
        "dry_run": False, "with_inputs": True, "with_cameras": False,
        "with_recording": False, "with_policy": False, "viewer": True,
    }))
    assert result.ok and len(launched) == 1
    preset = launched[0]
    assert preset.package == "nexus_core"
    assert preset.args["profile"] == str(PROFILE)
    assert preset.args["dry_run"] == "false"
    assert preset.args["with_inputs"] == "true"
    assert preset.args["with_recording"] == "false"
    assert preset.args["with_policy"] == "false"
    assert preset.env == {"NEXUS_MUJOCO_VIEWER": "1"}
    assert result.data["profile_sha256"] == web_server._nexus_profile.digest


def test_viewer_needs_desktop_and_sim_needs_real_sim_driver(monkeypatch, tmp_path):
    launched = _prepare(monkeypatch, tmp_path)
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    with pytest.raises(HTTPException) as no_display:
        asyncio.run(web_server.nexus_start({
            "profile": "nero_dual_xhand_mujoco", "viewer": True,
            "dry_run": False,
        }))
    assert no_display.value.status_code == 409
    with pytest.raises(HTTPException) as fake_driver:
        asyncio.run(web_server.nexus_start({
            "profile": "nero_dual_xhand_mujoco", "viewer": False,
            "dry_run": True,
        }))
    assert fake_driver.value.status_code == 400
    assert launched == []
