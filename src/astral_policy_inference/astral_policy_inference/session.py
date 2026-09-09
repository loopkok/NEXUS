"""Non-ROS real-robot session: orchestrate PolicyRunner + hardware I/O.

Wires the pure-Python core (:class:`~astral_policy_inference.runner.PolicyRunner`)
to the actual robot without ROS:

* :class:`RobotSession` — pumps obs (state from :class:`RobotIO`, images from
  :class:`CameraIO`) into the runner, sends safety-passed absolute actions back
  via ``RobotIO.send_row``, records the session to HDF5, and maps HITL
  takeover/release to damping/position modes on the board.
* :class:`Hdf5SessionRecorder` — writes an ``aligned_data.h5``-compatible file
  (``/action`` + attrs ``fps``/``schema``) so a recorded session is directly
  replayable via :func:`astral_policy_inference.replay.load_replay`; also
  records state / prompt / timestamps / camera images (optional).
* :class:`SessionConfig` / :func:`load_session_config` / :func:`build_robot_session`
  — YAML-driven assembly (schema section mirrors ``data_collect.yaml``).

``backend_cfg`` is passed straight through to the runner factory, so the same
session runs either remote openpi (large VLA on a GPU host) or in-process ACT
(py3.12 lerobot env on the robot machine) — only the config changes.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import os
import threading
import time
from typing import Optional

import h5py
import numpy as np

from astral_data_collect.schema import CollectSchema
from astral_policy_inference.hw_io import CameraIO, CameraSpec, RobotIO
from astral_policy_inference.runner import PolicyRunner

_LOG = logging.getLogger("astral_policy_inference.session")


# ------------------------------------------------------------------ config


@dataclasses.dataclass
class SessionConfig:
    """Parsed ``robot_session.yaml`` (schema is the layout single source)."""

    schema: CollectSchema
    robot: dict
    cameras: list[CameraSpec]
    model: dict
    session: dict


def load_session_config(path: str) -> SessionConfig:
    """Parse a robot_session.yaml into a SessionConfig."""
    import yaml

    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    top = raw.get("robot_session", raw)

    sd = dict(top.get("schema", {}))
    schema = CollectSchema(
        arms=[str(s) for s in sd.get("arms", ["left"])],
        end_effector_left=str(sd.get("end_effector_left", "gripper")),
        end_effector_right=str(sd.get("end_effector_right", "none")),
        include_waist=bool(sd.get("include_waist", False)),
        include_head=bool(sd.get("include_head", False)),
        cameras=[str(c) for c in sd.get("cameras", [])],
        dataset_fps=int(sd.get("dataset_fps", 30)),
    )
    cameras = [
        CameraSpec(label=str(label), **{k: v for k, v in spec.items()})
        for label, spec in dict(top.get("cameras", {})).items()
    ]
    return SessionConfig(
        schema=schema,
        robot=dict(top.get("robot", {})),
        cameras=cameras,
        model=dict(top.get("model", {})),
        session=dict(top.get("session", {})),
    )


def _parse_camera_map(raw) -> dict[str, str]:
    if isinstance(raw, dict):
        return {str(k): str(v) for k, v in raw.items()}
    text = str(raw or "").strip()
    if not text:
        return {}
    data = json.loads(text)
    return {str(k): str(v) for k, v in data.items()}


def build_robot_session(
    cfg: SessionConfig,
    *,
    record_dir: Optional[str] = None,
    recorder=None,
) -> "RobotSession":
    """Assemble a RobotSession from a SessionConfig (CLI / programmatic entry)."""
    robot_cfg = dict(cfg.robot)
    dry_run = bool(robot_cfg.pop("dry_run", False))
    stop_mode = str(robot_cfg.pop("stop_mode", "hold"))
    io = RobotIO(cfg.schema, dry_run=dry_run, **robot_cfg)
    cam = CameraIO(
        cfg.cameras,
        image_size=int(cfg.model.get("camera_image_size", 224)),
    )
    if recorder is None:
        target = record_dir or cfg.session.get("record_dir") or ""
        if target:
            os.makedirs(target, exist_ok=True)
            path = os.path.join(
                target, f"session_{time.strftime('%Y%m%d_%H%M%S')}.h5"
            )
            recorder = Hdf5SessionRecorder(
                path,
                cfg.schema,
                record_images=bool(cfg.session.get("record_images", True)),
            )
    backend_cfg = {
        # only make_backend()-accepted keys reach the runner's backend factory;
        # engine/safety/session knobs are passed explicitly below.
        "backend_type": str(cfg.model.get("backend_type", "openpi")),
        "host": str(cfg.model.get("host", "127.0.0.1")),
        "port": int(cfg.model.get("port", 8000)),
        "checkpoint_dir": str(cfg.model.get("checkpoint_dir", "") or ""),
        "default_prompt": str(cfg.model.get("default_prompt", "")),
        "camera_map": _parse_camera_map(cfg.model.get("camera_map", {})),
    }
    device = str(cfg.model.get("device", "") or "")
    if device:
        backend_cfg["device"] = device
    sess_cfg = cfg.session
    return RobotSession(
        schema=cfg.schema,
        robot_io=io,
        camera_io=cam,
        recorder=recorder,
        backend_cfg=backend_cfg,
        engine_mode=str(cfg.model.get("engine_mode", "queue_async")),
        action_chunk=int(cfg.model.get("action_chunk", 50)),
        control_interp=int(cfg.model.get("control_interp", 1)),
        abs_action_min_scale=float(cfg.model.get("abs_action_min_scale", 0.5)),
        default_prompt=str(cfg.model.get("default_prompt", "")),
        replay_source=str(sess_cfg.get("replay_source", "")),
        replay_episode=int(sess_cfg.get("replay_episode", 0)),
        replay_hold_s=float(sess_cfg.get("replay_hold_s", 1.0)),
        max_joint_vel=float(sess_cfg.get("max_joint_vel", 6.0)),
        obs_timeout_s=float(sess_cfg.get("obs_timeout_s", 0.5)),
        obs_stale_stop_s=float(sess_cfg.get("obs_stale_stop_s", 1.0)),
        stop_mode=stop_mode,
        prompt=str(backend_cfg.get("default_prompt", "")),
    )


# ------------------------------------------------------------- recorder


class Hdf5SessionRecorder:
    """Append-only HDF5 session recorder, replayable by ``load_replay``.

    Layout (``aligned_data.h5`` compatible, so the session can be replayed on
    the robot with ``request("playback:<path>")``):

      /action   (N, state_dim) float64  — safety-passed absolute rows sent
      /state    (N, state_dim) float64  — obs state used at each action tick
      /t_stamp  (N,) float64            — monotonic
      /wall_t   (N,) float64            — wall clock
      /prompt   (N,) vlen str           — per-row language instruction
      /streams/<label> (N,H,W,3) uint8  — camera images (if record_images)
      attrs: fps, schema, prompt_last
    """

    def __init__(
        self,
        path: str,
        schema: CollectSchema,
        *,
        record_images: bool = True,
        flush_rows: int = 30,
    ) -> None:
        self._path = os.path.abspath(path)
        self._schema = schema
        self._record_images = bool(record_images)
        self._flush_rows = max(1, int(flush_rows))
        self._f: Optional[h5py.File] = None
        self._n = 0
        self._buf = {
            "state": [],
            "action": [],
            "t_stamp": [],
            "wall_t": [],
            "prompt": [],
        }
        self._buf_images: dict[str, list] = {}

    @property
    def path(self) -> str:
        return self._path

    @property
    def rows(self) -> int:
        return self._n + len(self._buf["state"])

    def open(self) -> None:
        if self._f is not None:
            return
        os.makedirs(os.path.dirname(self._path), exist_ok=True)
        f = h5py.File(self._path, "w")
        dim = self._schema.state_dim
        f.attrs["fps"] = int(self._schema.dataset_fps)
        f.attrs["schema"] = self._schema.to_json()
        f.create_dataset("action", shape=(0, dim), maxshape=(None, dim),
                         dtype="f8", chunks=(512, dim))
        f.create_dataset("state", shape=(0, dim), maxshape=(None, dim),
                         dtype="f8", chunks=(512, dim))
        f.create_dataset("t_stamp", shape=(0,), maxshape=(None,), dtype="f8",
                         chunks=(2048,))
        f.create_dataset("wall_t", shape=(0,), maxshape=(None,), dtype="f8",
                         chunks=(2048,))
        f.create_dataset("prompt", shape=(0,), maxshape=(None,),
                         dtype=h5py.special_dtype(vlen=str), chunks=(2048,))
        self._f = f

    def append(
        self,
        *,
        state: np.ndarray,
        action: np.ndarray,
        prompt: str = "",
        images: dict[str, np.ndarray] | None = None,
    ) -> None:
        if self._f is None:
            return
        self._buf["state"].append(np.asarray(state, dtype=np.float64).reshape(-1))
        self._buf["action"].append(np.asarray(action, dtype=np.float64).reshape(-1))
        self._buf["t_stamp"].append(time.monotonic())
        self._buf["wall_t"].append(time.time())
        self._buf["prompt"].append(str(prompt or ""))
        if self._record_images and images:
            for label, img in images.items():
                self._buf_images.setdefault(label, []).append(
                    np.asarray(img, dtype=np.uint8)
                )
        if len(self._buf["state"]) >= self._flush_rows:
            self.flush()

    def flush(self) -> None:
        if self._f is None or not self._buf["state"]:
            return
        n = len(self._buf["state"])
        start = self._n
        for label, imgs in self._buf_images.items():
            try:
                arr = np.stack(imgs)
            except ValueError:  # frame size changed mid-session: drop this batch
                _LOG.warning("session recorder: image size changed for %s; skipped", label)
                continue
            key = f"streams/{label}"
            if key not in self._f:
                ds = self._f.create_dataset(
                    key,
                    shape=(0,) + arr.shape[1:],
                    maxshape=(None,) + arr.shape[1:],
                    dtype="u1",
                    chunks=(8,) + arr.shape[1:],
                    compression="lzf",
                )
            else:
                ds = self._f[key]
            ds.resize(start + n, axis=0)
            ds[start : start + n] = arr
        for k in ("state", "action", "t_stamp", "wall_t", "prompt"):
            ds = self._f[k]
            ds.resize(start + n, axis=0)
            ds[start : start + n] = self._buf[k]
        self._n += n
        self._buf = {k: [] for k in self._buf}
        self._buf_images = {}

    def close(self) -> None:
        if self._f is None:
            return
        self.flush()
        try:
            if self._n:
                last = self._f["prompt"][self._n - 1]
                if isinstance(last, bytes):
                    last = last.decode("utf-8", "replace")
                self._f.attrs["prompt_last"] = str(last)
        except Exception:  # noqa: BLE001
            pass
        self._f.close()
        self._f = None


# ------------------------------------------------------------- session


class RobotSession:
    """One non-ROS real-robot policy/replay session.

    * ``start()`` connects the board, starts cameras/recorder/runner, and pumps
      obs at ``dataset_fps``; ``stop()`` tears everything down (holding position
      unless ``stop_mode='damping'``).
    * ``request(verb)`` / ``set_prompt(text)`` are the programmable command
      surface (the CLI is a thin keyboard driver over these).
    * HITL: ``takeover`` flips FSM->HUMAN and puts the arm in damping (drag by
      hand); ``release`` re-plans/re-anchors from the live pose in position mode.

    ``backend_cfg`` mirrors ``policy_inference.yaml`` (backend_type/host/port/
    checkpoint_dir/camera_map/default_prompt/...). All SDK/camera calls are
    exception-safe from the runner's control thread (a hardware error is logged
    and surfaced via ``stats()['session_error']`` instead of killing the loop).
    """

    def __init__(
        self,
        *,
        schema: CollectSchema,
        robot_io: RobotIO,
        camera_io: CameraIO,
        recorder: Hdf5SessionRecorder | None = None,
        backend_cfg: dict,
        engine_mode: str = "queue_async",
        action_chunk: int = 50,
        control_interp: int = 1,
        abs_action_min_scale: float = 0.5,
        default_prompt: str = "",
        replay_source: str = "",
        replay_episode: int = 0,
        replay_hold_s: float = 1.0,
        max_joint_vel: float = 6.0,
        obs_timeout_s: float = 0.5,
        obs_stale_stop_s: float = 1.0,
        stop_mode: str = "hold",
        prompt: str = "",
        on_state=None,
    ) -> None:
        self.schema = schema
        self.io = robot_io
        self.cam = camera_io
        self.recorder = recorder
        self._stop_mode = str(stop_mode)
        self._prompt = str(prompt if prompt is not None else default_prompt)
        self._obs_lock = threading.Lock()
        self._last_state: Optional[np.ndarray] = None
        self._last_images: dict[str, np.ndarray] = {}
        self._last_prompt = self._prompt
        self._last_error: Optional[str] = None
        self._backend_cfg = dict(backend_cfg)
        self._ctrl_rate = float(schema.dataset_fps) * int(control_interp)
        self.runner = PolicyRunner(
            backend_cfg=dict(backend_cfg),
            schema=schema,
            engine_mode=engine_mode,
            action_chunk=int(action_chunk),
            control_interp=int(control_interp),
            abs_action_min_scale=float(abs_action_min_scale),
            replay_source=str(replay_source),
            replay_episode=int(replay_episode),
            replay_hold_s=float(replay_hold_s),
            max_joint_vel=float(max_joint_vel),
            obs_timeout_s=float(obs_timeout_s),
            obs_stale_stop_s=float(obs_stale_stop_s),
            on_action=self._on_action,
            on_state=on_state or (lambda _d: None),
            on_acquire_control=self._on_acquire,
            on_release_control=self._on_release,
        )
        self._pump_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        self.io.connect()
        self.cam.start()
        if self.recorder is not None:
            self.recorder.open()
        self.runner.start()
        self._stop_evt.clear()
        self._pump_thread = threading.Thread(
            target=self._pump, daemon=True, name="session-pump"
        )
        self._pump_thread.start()
        _LOG.info(
            "RobotSession up: %dD fps=%d backend_cfg=%s",
            self.schema.state_dim,
            self.schema.dataset_fps,
            {k: v for k, v in self._backend_cfg.items() if k != "camera_map"},
        )

    def stop(self) -> None:
        self._stop_evt.set()
        if self._pump_thread is not None:
            self._pump_thread.join(timeout=3.0)
            self._pump_thread = None
        self.runner.stop()
        self.cam.stop()
        try:
            if self.recorder is not None:
                self.recorder.close()
        finally:
            # disconnect must always run (board teardown is safety-relevant)
            self.io.disconnect()

    def estop(self) -> None:
        """True e-stop (power off) then tear down."""
        try:
            self.io.estop()
        except Exception as exc:  # noqa: BLE001
            _LOG.error("estop failed: %s", exc)
        self.stop()

    # ------------------------------------------------------- obs pump

    def _pump(self) -> None:
        dt = 1.0 / max(1.0, self._ctrl_rate)
        while not self._stop_evt.is_set():
            state = self.io.read_state()
            images = self.cam.read_images()
            with self._obs_lock:
                self._last_state = state
                self._last_images = images
                self._last_prompt = self._prompt
            if state is not None:
                self.runner.feed_observation(
                    state=state, images=images, prompt=self._prompt
                )
            time.sleep(dt)

    # --------------------------------------------------- runner callbacks

    def _on_action(self, row: np.ndarray) -> None:
        try:
            self.io.send_row(row)
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"send_row:{exc}"
            _LOG.error("send_row failed: %s", exc)
            return
        if self.recorder is None:
            return
        try:
            with self._obs_lock:
                st = self._last_state
                imgs = self._last_images
                pr = self._last_prompt
            self.recorder.append(
                state=st if st is not None else np.zeros(self.schema.state_dim),
                images=imgs,
                action=row,
                prompt=pr,
            )
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"record:{exc}"
            _LOG.error("session record failed: %s", exc)

    def _on_acquire(self) -> None:
        """Policy/playback owns the command stream -> position mode, re-seeded
        from the measured pose (no snap-back after a damping session)."""
        try:
            self.io.set_position(seed_from_current=True)
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"set_position:{exc}"
            _LOG.error("set_position failed: %s", exc)

    def _on_release(self) -> None:
        """Control released. HUMAN takeover -> damping (drag by hand); any other
        release (stop) -> hold position (never go limp unintentionally)."""
        try:
            if self.runner.controller.state == "HUMAN":
                self.io.set_damping()
            elif self._stop_mode == "damping":
                self.io.set_damping()
            else:
                self.io.set_position(seed_from_current=True)
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"release:{exc}"
            _LOG.error("release handler failed: %s", exc)

    # ------------------------------------------------------------ command

    def request(self, verb: str) -> None:
        self.runner.request(verb)

    def set_prompt(self, text: str) -> None:
        self._prompt = str(text or "")
        with self._obs_lock:
            self._last_prompt = self._prompt

    def stats(self) -> dict:
        st = self.runner.stats()
        if self._last_error:
            st["session_error"] = self._last_error
        return st


__all__ = [
    "Hdf5SessionRecorder",
    "RobotSession",
    "SessionConfig",
    "build_robot_session",
    "load_session_config",
]
