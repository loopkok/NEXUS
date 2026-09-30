"""Robot console metadata and asynchronous lifecycle operations (no hardware imports)."""
from __future__ import annotations

import asyncio
import json
import subprocess
import time
import uuid
from pathlib import Path

from nexus_core.adapter_registry import DRIVERS


def capabilities(profile):
    components = [{"name": c.name, "kind": c.kind, "dim": c.dim,
                   "driver": c.driver, "feedback": c.feedback,
                   "home": DRIVERS[c.driver].supports_home} for c in profile.components]
    # Viewer support is a driver property, independent of the robot's name.
    viewer = all(DRIVERS[c.driver].supports_viewer for c in profile.components)
    return {"components": components, "home": any(c["home"] for c in components),
            "viewer": viewer, "cameras": [c["role"] for c in profile.raw["cameras"]]}


class RobotConsole:
    def __init__(self):
        self.operations = {}
        self.tasks = set()
        self.run = None
        self.path = None

    def begin(self, profile, settings, root):
        try:
            revision = subprocess.check_output(["git", "rev-parse", "HEAD"],
                                               text=True, timeout=2).strip()
        except (OSError, subprocess.SubprocessError):
            revision = None
        self.operations = {}
        self.run = {"id": uuid.uuid4().hex, "started_at": time.time(),
                    "git_commit": revision, "profile": profile.raw,
                    "profile_sha256": profile.digest, "settings": settings,
                    "events": []}
        self.path = Path(root) / "runs" / self.run["id"] / "report.json"
        self.event("start")

    def event(self, action, **details):
        if self.run is None:
            return
        self.run["events"].append({"time": time.time(), "action": action, **details})
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.run, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(self.path)

    def submit(self, verb, call):
        if verb != "estop" and any(o["status"] == "running" for o in self.operations.values()):
            raise ValueError("驱动操作正在执行，请等待完成；急停仍可使用")
        operation = {"id": uuid.uuid4().hex, "verb": verb, "status": "running",
                     "started_at": time.time(), "message": "等待驱动响应"}
        self.operations[operation["id"]] = operation
        self.event("driver_request", verb=verb, operation_id=operation["id"])

        async def execute():
            try:
                ok, message = await asyncio.to_thread(call)
            except Exception as error:
                ok, message = False, str(error)
            operation.update(status="succeeded" if ok else "failed", message=message,
                             finished_at=time.time())
            self.event("driver_result", operation=dict(operation))

        task = asyncio.create_task(execute())
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)
        return dict(operation)

    def report(self, snapshot, logs, jobs):
        return {**(self.run or {}), "exported_at": time.time(), "robot": snapshot,
                "operations": list(self.operations.values()), "logs": logs, "jobs": jobs}
