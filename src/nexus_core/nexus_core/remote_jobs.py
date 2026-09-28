"""SSH/rsync GPU job runner shared by CLI and the NEXUS web operation page."""

from __future__ import annotations

import os
import re
import shlex
import signal
import subprocess
import threading
import time
import uuid
from collections import deque
from pathlib import Path

from .profile import Profile

_SAFE = re.compile(r"^[A-Za-z0-9_.-]+$")


class RemoteJob:
    def __init__(self, kind: str, profile: Profile, session: str, gpu_host: str,
                 gpu_root: str, gpu_python: str):
        if kind not in ("sync_process", "train_act", "train_pi05"):
            raise ValueError(f"unknown GPU job {kind}")
        if not _SAFE.fullmatch(session):
            raise ValueError("session must contain only letters, digits, _, . and -")
        if not re.fullmatch(r"[A-Za-z0-9_.@-]+", gpu_host):
            raise ValueError("invalid GPU SSH host")
        if not gpu_root.startswith("/") or not gpu_python.startswith("/"):
            raise ValueError("GPU root and Python path must be absolute")
        if not all(re.fullmatch(r"/[A-Za-z0-9_./-]+", value)
                   for value in (gpu_root, gpu_python)) or ".." in Path(gpu_root).parts or ".." in Path(gpu_python).parts:
            raise ValueError("GPU root and Python path must use safe absolute paths")
        self.id = uuid.uuid4().hex[:12]
        self.kind = kind
        self.profile = profile
        self.session = session
        self.gpu_host = gpu_host
        self.gpu_root = gpu_root.rstrip("/")
        self.gpu_python = gpu_python
        self.status = "queued"
        self.step = "queued"
        self.created_at = time.time()
        self.ended_at = None
        self.logs = deque(maxlen=500)
        self.artifacts = {}
        self._process = None
        self._remote_active = False
        self._cancel = threading.Event()
        self._lock = threading.Lock()
        self._remote_pidfile = f"{self.gpu_root}/jobs/{self.id}.pid"

    def snapshot(self):
        with self._lock:
            return {"id": self.id, "kind": self.kind, "status": self.status,
                    "step": self.step, "profile_id": self.profile.profile_id,
                    "profile_sha256": self.profile.digest, "session": self.session,
                    "created_at": self.created_at, "ended_at": self.ended_at,
                    "logs": list(self.logs), "artifacts": dict(self.artifacts)}

    def cancel(self):
        self._cancel.set()
        with self._lock:
            proc = self._process
            remote_active = self._remote_active
        if proc is not None and proc.poll() is None:
            proc.terminate()
        if self.status == "running" and remote_active:
            # Remote sh execs the actual worker after writing its PID. A
            # separate SSH connection terminates that exact process.
            remote = (f"test -r {shlex.quote(self._remote_pidfile)} && "
                      f"kill -TERM $(cat {shlex.quote(self._remote_pidfile)})")
            subprocess.run(["ssh", self.gpu_host, remote], timeout=8,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)

    def _run(self, step: str, args: list[str], remote: bool = False):
        if self._cancel.is_set():
            raise InterruptedError("job cancelled")
        with self._lock:
            self.step = step
            self.logs.append(f"[{step}] {' '.join(args[:3])}")
        if remote:
            command = shlex.join(args)
            wrapped = ("sh -c " + shlex.quote(
                f"mkdir -p {shlex.quote(self.gpu_root + '/jobs')}; "
                f"echo $$ > {shlex.quote(self._remote_pidfile)}; exec {command}"))
            args = ["ssh", self.gpu_host, wrapped]
        proc = subprocess.Popen(args, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True,
                                start_new_session=True)
        with self._lock:
            self._process = proc
            self._remote_active = remote
        try:
            for line in proc.stdout:
                with self._lock:
                    self.logs.append(line.rstrip())
            code = proc.wait()
            if self._cancel.is_set():
                raise InterruptedError("job cancelled")
            if code != 0:
                raise RuntimeError(f"{step} exited {code}")
        finally:
            with self._lock:
                self._process = None
                self._remote_active = False

    def execute(self, local_session: Path | None = None, *, steps: int = 1000,
                act_script: str = "", pi05_command: list[str] | None = None):
        with self._lock:
            self.status = "running"
        try:
            root = self.gpu_root
            slug = f"{self.session}_{self.profile.digest[:12]}"
            remote_session = f"{root}/sessions/{slug}"
            profile_path = f"{root}/profiles/{self.profile.profile_id}_{self.profile.digest[:12]}.json"
            pi_path = f"{root}/datasets/pi05/{slug}"
            act_path = f"{root}/datasets/act/{slug}"
            if self.kind == "sync_process":
                if local_session is None or not local_session.is_dir():
                    raise FileNotFoundError(local_session)
                self._run("create remote directories", ["mkdir", "-p", remote_session,
                    f"{root}/profiles", f"{root}/datasets/pi05", f"{root}/datasets/act",
                    f"{root}/jobs"], remote=True)
                self._run("sync frozen profile", ["rsync", "-a", str(Path(self.profile.path)),
                    f"{self.gpu_host}:{profile_path}"])
                self._run("sync raw episodes", ["rsync", "-a", "--partial",
                    str(local_session) + os.sep, f"{self.gpu_host}:{remote_session}/"])
                self._run("align, quality check and export", [self.gpu_python, "-m",
                    "nexus_core.pipeline", "--profile", profile_path,
                    "--session", remote_session, "--pi-output", pi_path,
                    "--act-output", act_path], remote=True)
                self.artifacts = {"pi05_dataset": pi_path, "act_dataset": act_path}
            elif self.kind == "train_act":
                if not act_script.startswith("/"):
                    raise ValueError("ACT training script path must be absolute")
                output = f"{root}/checkpoints/act/{slug}_{self.id}"
                self._run("verify ACT dataset layout", [self.gpu_python, "-m",
                    "nexus_core.cli", profile_path, "--model-manifest",
                    f"{act_path}/nexus_layout.json"], remote=True)
                self._run("ACT train", ["bash", act_script, "--mode", "from_scratch",
                    "--dataset", act_path, "--output-dir", output,
                    "--repo-id", f"nexus/{slug}", "--steps", str(steps),
                    "--no-wandb"], remote=True)
                self._run("stamp ACT checkpoint layout", [self.gpu_python, "-m",
                    "nexus_core.stamp_checkpoint", "--dataset", act_path,
                    "--checkpoint-root", output], remote=True)
                self.artifacts = {"checkpoint": output,
                                  "layout_manifest": f"{act_path}/nexus_layout.json"}
            else:
                if not pi05_command:
                    raise ValueError("pi0.5 GPU command must be configured for this profile")
                output = f"{root}/checkpoints/pi05/{slug}_{self.id}"
                self._run("verify pi0.5 dataset layout", [self.gpu_python, "-m",
                    "nexus_core.cli", profile_path, "--model-manifest",
                    f"{pi_path}/nexus_layout.json"], remote=True)
                substitutions = {"dataset": pi_path, "profile": profile_path,
                                 "output": output, "steps": str(steps),
                                 "profile_id": self.profile.profile_id}
                command = [part.format(**substitutions) for part in pi05_command]
                self._run("pi0.5 train", command, remote=True)
                self._run("stamp pi0.5 checkpoint layout", [self.gpu_python, "-m",
                    "nexus_core.stamp_checkpoint", "--dataset", pi_path,
                    "--checkpoint-root", output], remote=True)
                self.artifacts = {"checkpoint": output, "dataset": pi_path,
                                  "layout_manifest": f"{pi_path}/nexus_layout.json"}
            with self._lock:
                self.status = "succeeded"
                self.step = "done"
        except InterruptedError as exc:
            with self._lock:
                self.status = "cancelled"
                self.logs.append(str(exc))
        except Exception as exc:
            with self._lock:
                self.status = "failed"
                self.logs.append(f"ERROR: {exc}")
        finally:
            with self._lock:
                self.ended_at = time.time()


class JobRegistry:
    def __init__(self):
        self.jobs = {}
        self._lock = threading.Lock()

    def submit(self, job: RemoteJob, *args, **kwargs):
        with self._lock:
            self.jobs[job.id] = job
        threading.Thread(target=job.execute, args=args, kwargs=kwargs,
                         daemon=True, name=f"nexus-job-{job.id}").start()
        return job.id

    def get(self, job_id: str) -> RemoteJob:
        with self._lock:
            return self.jobs[job_id]

    def list(self):
        with self._lock:
            return [j.snapshot() for j in self.jobs.values()]


def main() -> None:
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("sync_process", "train_act", "train_pi05"))
    parser.add_argument("--profile", required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--data-root", default=os.environ.get("NEXUS_DATA_ROOT", "~/nexus_data"))
    parser.add_argument("--gpu-host", default=os.environ.get("NEXUS_GPU_HOST", ""))
    parser.add_argument("--gpu-root", default=os.environ.get("NEXUS_GPU_ROOT", ""))
    parser.add_argument("--gpu-python", default=os.environ.get("NEXUS_GPU_PYTHON", ""))
    parser.add_argument("--act-script", default=os.environ.get("NEXUS_GPU_ACT_SCRIPT", ""))
    parser.add_argument("--pi05-command", default="", help="quoted command template with {dataset}/{output}/{steps}")
    parser.add_argument("--steps", type=int, default=1000)
    args = parser.parse_args()
    profile = Profile.load(args.profile)
    job = RemoteJob(args.kind, profile, args.session, args.gpu_host,
                    args.gpu_root, args.gpu_python)
    command = shlex.split(args.pi05_command) if args.pi05_command else None
    job.execute(Path(args.data_root).expanduser() / args.session if args.kind == "sync_process" else None,
                steps=args.steps, act_script=args.act_script, pi05_command=command)
    print(json.dumps(job.snapshot(), ensure_ascii=False, indent=2))
    if job.status != "succeeded":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
