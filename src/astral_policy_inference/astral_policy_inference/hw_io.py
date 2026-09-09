"""Non-ROS hardware I/O adapters for a real-robot policy session.

Two adapters wire the pure-Python :class:`PolicyRunner` to the actual robot
without ROS:

* :class:`RobotIO` — ``astral_robot_sdk`` (UDP -> control board): reads the
  18-DoF joint feedback and assembles a schema-layout state vector; sends
  absolute action rows (arm rad + gripper ratio) back to the board; exposes
  the damping/position modes used for human-in-the-loop takeover.
* :class:`CameraIO` — one background capture thread per camera (OpenCV V4L2 /
  pyrealsense2), latest frame letterboxed to the model input size.

Semantics mirror ``astral_robot_control``'s driver node (so a session run
without ROS behaves exactly like the ROS deployment):

* gripper has no true feedback -> echo the last commanded ratio (config
  default before the first command),
* single-arm sessions command only that arm's motor ids (absent side untouched),
* damping->position transitions re-seed the board target from the measured pose
  so the arm does not snap back to the pre-damping target.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from astral_data_collect.schema import CollectSchema
from astral_policy_inference.robot_io import ObsLayout, letterbox

_LOG = logging.getLogger("astral_policy_inference.hw_io")

_ARM_DIM = 7
_BODY_DIM = 18


@dataclass(frozen=True)
class CameraSpec:
    """One camera: how to open it and at what geometry."""

    label: str
    kind: str = "v4l2"        # "v4l2" | "realsense"
    device: int | str = 0     # V4L2 index/path, or realsense serial
    width: int = 640
    height: int = 480
    fps: int = 30


class CameraProvider:
    """Structural base for camera capture; read() returns HxWx3 uint8 RGB."""

    def start(self) -> None: ...
    def read(self) -> Optional[np.ndarray]: ...
    def stop(self) -> None: ...


class V4l2CameraProvider(CameraProvider):
    """OpenCV V4L2 (MJPG) capture, e.g. a USB webcam (video0)."""

    def __init__(self, spec: CameraSpec) -> None:
        self._spec = spec
        self._cap = None

    def start(self) -> None:
        import cv2

        dev = self._spec.device
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        if not cap.isOpened():
            cap = cv2.VideoCapture(dev)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        if not cap.isOpened():
            raise RuntimeError(f"failed to open camera device {dev!r}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._spec.width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._spec.height))
        cap.set(cv2.CAP_PROP_FPS, float(self._spec.fps))
        self._cap = cap

    def read(self) -> Optional[np.ndarray]:
        import cv2

        if self._cap is None:
            return None
        ok, bgr = self._cap.read()
        if not ok or bgr is None:
            return None
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def stop(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class RealSenseCameraProvider(CameraProvider):
    """pyrealsense2 color stream (e.g. the D435i at video8).

    Requires ``pyrealsense2`` in the running interpreter; the Session falls
    back to ``V4l2CameraProvider`` on the UVC color endpoint when it is
    missing (caller decides via the provider factory).
    """

    def __init__(self, spec: CameraSpec) -> None:
        self._spec = spec
        self._pipeline = None

    def start(self) -> None:
        import pyrealsense2 as rs

        cfg = rs.config()
        dev = self._spec.device
        if isinstance(dev, str) and dev:
            cfg.enable_device(dev)
        cfg.enable_stream(
            rs.stream.color,
            self._spec.width,
            self._spec.height,
            rs.format.rgb8,
            self._spec.fps,
        )
        self._pipeline = rs.pipeline()
        self._pipeline.start(cfg)

    def read(self) -> Optional[np.ndarray]:
        import pyrealsense2 as rs

        if self._pipeline is None:
            return None
        frames = self._pipeline.wait_for_frames(timeout_ms=200)
        color = frames.get_color_frame()
        if color is None:
            return None
        return np.asanyarray(color.get_data()).copy()  # already RGB

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None


class CameraIO:
    """One capture thread per camera; ``read_images()`` returns the latest
    frames (letterboxed to ``image_size``), keyed by label."""

    def __init__(
        self,
        specs: list[CameraSpec],
        image_size: int,
        *,
        provider_factory: Optional[Callable[[CameraSpec], CameraProvider]] = None,
    ) -> None:
        self._specs = list(specs)
        self._image_size = int(image_size)
        self._factory = provider_factory or CameraIO._default_provider
        self._providers: dict[str, CameraProvider] = {}
        self._latest: dict[str, np.ndarray] = {}
        self._stamps: dict[str, float] = {}
        self._lock = threading.Lock()
        self._threads: list[threading.Thread] = []
        self._stop = threading.Event()

    @staticmethod
    def _default_provider(spec: CameraSpec) -> CameraProvider:
        if spec.kind == "realsense":
            return RealSenseCameraProvider(spec)
        return V4l2CameraProvider(spec)

    @property
    def labels(self) -> list[str]:
        return [s.label for s in self._specs]

    def start(self) -> None:
        self._stop.clear()
        for spec in self._specs:
            try:
                prov = self._factory(spec)
                prov.start()
            except Exception as exc:  # noqa: BLE001
                # One dead camera must not kill the session: policy can degrade
                # (openpi zero-pads missing slots) and other cameras keep going.
                _LOG.warning("camera %s failed to start: %s", spec.label, exc)
                continue
            self._providers[spec.label] = prov
            t = threading.Thread(
                target=self._capture_loop,
                args=(spec.label, prov),
                daemon=True,
                name=f"cam-{spec.label}",
            )
            self._threads.append(t)
            t.start()

    def _capture_loop(self, label: str, prov: CameraProvider) -> None:
        while not self._stop.is_set():
            frame = prov.read()
            if frame is None:
                time.sleep(0.005)
                continue
            try:
                img = letterbox(frame, self._image_size)
            except Exception:  # noqa: BLE001
                time.sleep(0.005)
                continue
            with self._lock:
                self._latest[label] = img
                self._stamps[label] = time.monotonic()

    def read_images(self) -> dict[str, np.ndarray]:
        """Latest frames per label (copies), HxWx3 uint8 RGB."""
        with self._lock:
            return {k: v.copy() for k, v in self._latest.items()}

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads = []
        for prov in self._providers.values():
            try:
                prov.stop()
            except Exception:  # noqa: BLE001
                pass
        self._providers = {}
        self._latest = {}


class RobotIO:
    """astral_robot_sdk adapter: schema-layout state in, absolute rows out.

    ``dry_run=True`` keeps the whole pipeline runnable with no board (default
    zero state, commands logged) — used for bring-up / sim.
    """

    def __init__(
        self,
        schema: CollectSchema,
        *,
        robot_factory: Optional[Callable[[], object]] = None,
        dry_run: bool = False,
        board_ip: str = "192.168.10.2",
        board_cmd_port: int = 5001,
        local_ip: str = "0.0.0.0",
        local_port: int = 8081,
        obs_hz: int = 50,
        ctrl_hz: int = 50,
        auto_ready: bool = True,
        lpf_enable: bool = True,
        lpf_alpha: float = 0.85,
        left_gripper_open_rad: float = 0.8,
        left_gripper_closed_rad: float = 0.0,
        right_gripper_open_rad: float = 0.8,
        right_gripper_closed_rad: float = 0.0,
        default_gripper_ratio: float = 0.0,
    ) -> None:
        self.schema = schema
        self.layout = ObsLayout(schema)
        self._robot_factory = robot_factory
        self._dry_run = bool(dry_run)
        self._board_ip = str(board_ip)
        self._board_cmd_port = int(board_cmd_port)
        self._local_ip = str(local_ip)
        self._local_port = int(local_port)
        self._obs_hz = int(obs_hz)
        self._ctrl_hz = int(ctrl_hz)
        self._auto_ready = bool(auto_ready)
        self._lpf_enable = bool(lpf_enable)
        self._lpf_alpha = float(lpf_alpha)
        self._grip_open_rad = {
            "left": float(left_gripper_open_rad),
            "right": float(right_gripper_open_rad),
        }
        self._grip_closed_rad = {
            "left": float(left_gripper_closed_rad),
            "right": float(right_gripper_closed_rad),
        }
        self._default_gripper_ratio = float(default_gripper_ratio)
        self._robot: object | None = None
        self._grip_echo: dict[str, float | None] = {"left": None, "right": None}
        self._sdk_lock = threading.Lock()

    @property
    def dry_run(self) -> bool:
        return self._dry_run

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> None:
        if self._dry_run:
            _LOG.warning(
                "RobotIO dry_run=True: no board; default state + commands logged"
            )
            return
        if self._robot_factory is not None:
            self._robot = self._robot_factory()
        else:
            from astral_robot_sdk import (
                RobotFactory,
                RobotModel,
                create_robot_config,
            )

            cfg = create_robot_config(
                robot=RobotModel.ROBOTMAIN,
                control_board_ip=self._board_ip,
                board_cmd_port=self._board_cmd_port,
                local_ip=self._local_ip,
                local_port=self._local_port,
                obs_hz=self._obs_hz,
                ctrl_hz=self._ctrl_hz,
            )
            self._robot = RobotFactory.create_robot(cfg)
        self._robot.connect()
        if self._auto_ready:
            self.ready()
        self._apply_lpf()

    def _apply_lpf(self) -> None:
        if self._robot is None:
            return
        try:
            self._robot.set_lpf(self._lpf_enable, self._lpf_alpha)
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("set_lpf failed: %s", exc)

    def ready(self) -> None:
        """one_click_ready: WORK -> POSITION -> enable -> zero, seeding the
        board target from the measured pose so damping->POSITION does not snap
        the arm back to the pre-damping target."""
        if self._robot is None:
            return
        with self._sdk_lock:
            self._robot.one_click_ready(enable_timeout_s=3.0, seed_from_current=True)

    def disconnect(self) -> None:
        with self._sdk_lock:
            if self._robot is not None:
                try:
                    self._robot.disconnect()
                except Exception as exc:  # noqa: BLE001
                    _LOG.warning("disconnect failed: %s", exc)
                self._robot = None

    # ------------------------------------------------------------ obs ingress

    def _read_q18(self) -> np.ndarray:
        if self._robot is None:
            return np.zeros(_BODY_DIM, dtype=np.float64)
        angles = self._robot.get_joint_angles()
        pos = list(angles.msg.positions)
        if len(pos) < _BODY_DIM:
            pos = pos + [0.0] * (_BODY_DIM - len(pos))
        return np.asarray(pos[:_BODY_DIM], dtype=np.float64)

    def read_state(self) -> np.ndarray | None:
        """Assemble the schema-layout absolute state vector (None if a required
        block cannot be filled — normally never for this schema)."""
        with self._sdk_lock:
            q = self._read_q18()
            vectors: dict[str, np.ndarray] = {}
            for side in self.schema.arms:
                start = 0 if side == "left" else _ARM_DIM
                vectors[f"{side}_arm_state"] = q[start : start + _ARM_DIM]
            for side in self.schema.arms:
                if self.schema.end_effector_for(side) != "gripper":
                    continue
                echo = self._grip_echo.get(side)
                vectors[f"{side}_gripper_ratio"] = np.asarray(
                    [self._default_gripper_ratio if echo is None else echo],
                    dtype=np.float64,
                )
            if self.schema.include_waist or self.schema.include_head:
                vectors["body_state"] = q
            state, _missing = self.layout.assemble_state(vectors)
            return state

    # ----------------------------------------------------------- action out

    def send_row(self, row: np.ndarray) -> None:
        """Split one absolute action row into per-block SDK calls, mirroring
        the driver's arbitration (only schema blocks, single-arm -> that side)."""
        with self._sdk_lock:
            targets = self.layout.split_action(
                np.asarray(row, dtype=np.float64).reshape(-1)
            )
            for t in targets:
                self._dispatch(t)

    def _dispatch(self, t: object) -> None:
        stream = str(t.stream)
        if stream.endswith("_arm_cmd"):
            side = stream.split("_arm_cmd")[0]
            if side not in self.schema.arms:
                raise RuntimeError(f"arm block {stream} not in schema arms")
            self._move_arm_side(side, np.asarray(t.values))
        elif stream.endswith("_ee_cmd"):
            side = stream.split("_ee_cmd")[0]
            if self.schema.end_effector_for(side) != "gripper":
                raise RuntimeError(
                    f"end-effector {self.schema.end_effector_for(side)!r} on {side} "
                    "not supported by RobotIO (gripper only)"
                )
            if str(t.kind) != "ratio":
                raise RuntimeError(f"gripper target kind {t.kind} unsupported")
            ratio = float(np.asarray(t.values).reshape(-1)[0])
            self._grip_echo[side] = ratio
            rad = self._grip_open_rad[side] + ratio * (
                self._grip_closed_rad[side] - self._grip_open_rad[side]
            )
            self._set_gripper_angle(side, rad)
        elif stream == "head_cmd":
            vals = np.asarray(t.values, dtype=np.float64).reshape(-1)
            if vals.shape[0] != 2:
                raise RuntimeError("head target must be 2-dim")
            self._move_head(vals)
        else:
            raise RuntimeError(f"unwritable block {stream}")

    def _move_arm_side(self, side: str, values: np.ndarray) -> None:
        if self._robot is None:
            _LOG.info("[dry_run] set_target_positions %s=%s", side,
                      np.round(values, 4).tolist())
            return
        from astral_robot_sdk import LEFT_ARM_IDS, RIGHT_ARM_IDS

        ids = {"left": LEFT_ARM_IDS, "right": RIGHT_ARM_IDS}[side]
        self._robot.set_target_positions(
            dict(zip(ids, [float(x) for x in values]))
        )

    def _set_gripper_angle(self, side: str, rad: float) -> None:
        if self._robot is None:
            _LOG.info("[dry_run] set_gripper_angle %s=%.3f rad", side, rad)
            return
        self._robot.set_gripper_angle(float(rad), right_hand=side == "right")

    def _move_head(self, values: np.ndarray) -> None:
        if self._robot is None:
            _LOG.info("[dry_run] move_head_js %s", np.round(values, 4).tolist())
            return
        self._robot.move_head_js(values[0], values[1])

    # ---------------------------------------------------------------- HITL

    def set_damping(self) -> None:
        """motion_mode=0 — arm relaxes, a human can drag it (takeover)."""
        with self._sdk_lock:
            if self._robot is None:
                _LOG.info("[dry_run] set_motion_mode(0) damping")
                return
            self._robot.set_motion_mode(0)

    def set_position(self, seed_from_current: bool = True) -> None:
        """motion_mode=1 — position hold; optionally re-seed the board target
        from the measured pose (no snap-back after a damping session)."""
        with self._sdk_lock:
            if self._robot is None:
                _LOG.info("[dry_run] set_motion_mode(1) position")
                return
            self._robot.set_motion_mode(1)
            if seed_from_current:
                q = self._read_q18()
                self._robot.move_arm_js(
                    left=list(q[:_ARM_DIM]), right=list(q[_ARM_DIM:14])
                )

    def estop(self) -> None:
        """True e-stop: power off, arm loses holding torque."""
        with self._sdk_lock:
            if self._robot is None:
                _LOG.warning("[dry_run] estop (no hardware)")
                return
            self._robot.e_stop()


__all__ = [
    "CameraIO",
    "CameraProvider",
    "CameraSpec",
    "RealSenseCameraProvider",
    "RobotIO",
    "V4l2CameraProvider",
]
