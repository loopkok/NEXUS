#!/usr/bin/env python3
"""Exercise the Web API using fake drivers only; never enable physical hardware.

Use a dedicated Web server, ROS domain and data root. Requires its source workspace
and synthetic input fixture to be available on this machine.
"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8091")
    parser.add_argument("--output", required=True)
    parser.add_argument("--include-simulation", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    events = []

    def api(path, body=None):
        req = Request(args.url + "/api/v1/" + path,
                      data=json.dumps(body).encode() if body is not None else None,
                      headers={"Content-Type": "application/json"})
        try:
            with urlopen(req, timeout=20) as response:
                data = json.load(response)
        except HTTPError as error:
            data = json.load(error)
            events.append({"path": path, "http_status": error.code, "response": data})
            raise
        events.append({"path": path, "response": data})
        assert data.get("ok", True), data
        return data.get("data")

    def wait(predicate, timeout=40):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = api("nexus/state")
            if predicate(state):
                return state
            time.sleep(.3)
        raise AssertionError("Web state transition timed out")

    def driver(verb):
        operation = api("nexus/driver/" + verb, {})
        state = wait(lambda s: any(o["id"] == operation["id"] and o["status"] != "running"
                                   for o in s["operations"]), timeout=125)
        result = next(o for o in state["operations"] if o["id"] == operation["id"])
        assert result["status"] == "succeeded", result

    try:
        assert api("nexus/state")["launch_state"] == "stopped", "Use an idle dedicated test server"
        for profile_id in (["astral_dual_gripper", "nero_dual_xhand"] + (["nero_dual_xhand_mujoco"] if args.include_simulation else [])):
            dry_run = not profile_id.endswith("_mujoco")
            api("nexus/validate", {"profile": profile_id})
            source_log = (output / (profile_id + "_source.log")).open("w")
            source = subprocess.Popen([
                "python3", str(root / "scripts/nexus_sim_input_source.py"),
                "--profile", str(root / "src/nexus_core/profiles" / (profile_id + ".json")),
                "--amplitude", "0.003", "--input-rate", "100"], stdout=source_log, stderr=subprocess.STDOUT)
            try:
                api("nexus/start", {"profile": profile_id, "session": profile_id + "_web_test",
                                    "dry_run": dry_run, "with_inputs": False, "with_cameras": False,
                                    "with_recording": True, "with_policy": False, "viewer": False})
                state = wait(lambda s: s["launch_state"] == "running" and
                             len(s["robot"]["joints"]) == 4 and
                             all(not j["stale"] for j in s["robot"]["joints"].values()))
                assert state["settings"]["dry_run"] is dry_run
                driver("ready")
                driver("enable")
                driver("home")
                api("teleop/start", {})
                wait(lambda s: s["robot"]["control"].get("mode") == "TELEOP")
                api("pause", {})
                wait(lambda s: s["launch_state"] == "paused" and s["robot"]["control"].get("mode") in ("IDLE", "PAUSED"))
                api("resume", {})
                wait(lambda s: s["launch_state"] == "running" and s["robot"]["control"].get("mode") == "TELEOP")
                api("collect/control", {"cmd": "start"})
                wait(lambda s: (s["robot"]["data_collect"] or {}).get("state") == "RECORDING")
                try:
                    api("stop", {})
                    raise AssertionError("Recording must block system shutdown")
                except HTTPError as error:
                    assert error.code == 409
                time.sleep(3)
                api("collect/control", {"cmd": "stop"})
                wait(lambda s: (s["robot"]["data_collect"] or {}).get("state") == "IDLE")
                state = api("nexus/state")
                publishers = state["robot"]["diagnostics"]["command_publishers"]
                assert all(len(p) == 1 for p in publishers.values()), publishers
                assert state["robot"]["diagnostics"]["streams"]
                api("pause", {})
                driver("estop")
                with urlopen(args.url + "/api/v1/nexus/report") as response:
                    report = json.load(response)
                assert report["settings"]["dry_run"] is dry_run
                (output / (profile_id + "_report.json")).write_text(
                    json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
                api("stop", {})
                wait(lambda s: s["launch_state"] == "stopped")
                print(profile_id + ": PASS", flush=True)
            finally:
                source.send_signal(signal.SIGINT)
                try:
                    source.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    source.kill()
                    source.wait()
                source_log.close()
    finally:
        (output / "api_events.json").write_text(json.dumps(events, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
