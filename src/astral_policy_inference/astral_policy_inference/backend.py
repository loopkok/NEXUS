"""Policy backends — swap a checkpoint directory / server, nothing else changes.

``PolicyBackend`` protocol + two concrete backends:

* :class:`OpenPiServerBackend` — talks to a remote ``openpi`` websocket policy
  server (``scripts/serve_policy.py``). The *server* owns the checkpoint, so
  changing models never touches the robot side; only the server command
  changes (``--policy.config`` + ``--policy.dir``).
* :class:`LerobotActBackend` — in-process ``PreTrainedPolicy.from_pretrained``
  + its serialized pre/post processor pipeline, so pointing at a different
  checkpoint directory swaps model, normalization, chunking — all at once.

Both emit **absolute** action rows (arm joint rad + gripper ratio 0..1) in the
dataset layout order, exactly what the driver/MuJoCo consumers take.

Heavy imports (websockets / torch / lerobot) happen lazily inside ``open()`` so
the module imports on machines that do not have them.
"""

from __future__ import annotations

import dataclasses
import time
from abc import ABC, abstractmethod
from typing import Callable

import numpy as np


class PolicyError(RuntimeError):
    """Raised when a backend cannot produce an action (network/policy/etc.)."""


@dataclasses.dataclass
class ObsBatch:
    """Neutral observation handed to any backend."""

    state: np.ndarray          # (state_dim,) unnormalized, dataset layout order
    images: dict[str, np.ndarray]  # logical key -> HxWx3 uint8
    prompt: str


class PolicyBackend(ABC):
    """Backend contract consumed by :mod:`engine`."""

    name: str = "backend"

    @abstractmethod
    def open(self) -> None:
        """Connect / load weights. May block for servers coming up."""

    @abstractmethod
    def close(self) -> None:
        """Release the connection / model."""

    @abstractmethod
    def infer(self, obs: ObsBatch) -> np.ndarray:
        """Return an absolute action chunk (>=1, action_dim); never None."""

    @abstractmethod
    def reset(self) -> None:
        """Drop any queued actions / language context on the policy side."""


def _actions_from_response(
    response: dict, action_dim: int, max_rows: int
) -> np.ndarray:
    actions = np.asarray(response["actions"], dtype=np.float64)
    if actions.ndim == 1:  # single row → make (1, dim)
        actions = actions.reshape(1, -1)
    if actions.shape[1] != action_dim:
        raise PolicyError(
            f"server action_dim {actions.shape[1]} != robot action_dim "
            f"{action_dim}"
        )
    return actions[:max_rows]


class OpenPiServerBackend(PolicyBackend):
    """Remote openpi websocket policy (pi0.5 served via ``serve_policy.py``)."""

    name = "openpi"

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 8000,
        action_dim: int,
        slot_keys: dict[str, str],  # camera_map: slot -> collect label
        state_key: str = "observation/state",
        image_prefix: str = "observation/camera",
        default_prompt: str = "",
        client_factory: Callable | None = None,  # test seam
    ):
        self.host = host
        self.port = int(port)
        self.action_dim = int(action_dim)
        self.slot_keys = dict(slot_keys)
        self.state_key = state_key
        self.image_prefix = image_prefix
        self.default_prompt = default_prompt
        self._client_factory = client_factory
        self._client = None
        self.last_infer_s = 0.0

    def open(self) -> None:
        if self._client is not None:
            return
        if self._client_factory is not None:
            self._client = self._client_factory(self.host, self.port)
            return
        try:
            from openpi_client.websocket_client_policy import WebsocketClientPolicy
        except ImportError as exc:  # pragma: no cover - env dependent
            raise PolicyError(
                "openpi_client not importable — run this node inside the openpi "
                "environment (uv) or `pip install -e packages/openpi-client`"
            ) from exc
        self._client = WebsocketClientPolicy(self.host, self.port)

    def close(self) -> None:
        ws = getattr(self._client, "_ws", None)
        if ws is not None:
            try:
                ws.close()
            except Exception:  # noqa: BLE001
                pass
        self._client = None

    def reset(self) -> None:
        if self._client is not None:
            try:
                self._client.reset()
            except Exception:  # noqa: BLE001
                pass

    def infer(self, obs: ObsBatch) -> np.ndarray:
        if self._client is None:
            raise PolicyError("OpenPiServerBackend not open()ed")
        payload: dict = {self.state_key: obs.state.astype(np.float32)}
        for slot, label in self.slot_keys.items():
            img = obs.images.get(label)
            if img is None:
                # AstralInputs zero-pads missing slots; server keeps image_mask.
                continue
            payload[f"{self.image_prefix}/{slot}"] = np.asarray(
                img, dtype=np.uint8
            )
        payload["prompt"] = obs.prompt or self.default_prompt
        t0 = time.perf_counter()
        try:
            response = self._client.infer(payload)
        except Exception as exc:  # noqa: BLE001
            raise PolicyError(f"openpi server infer failed: {exc}") from exc
        if "actions" not in (response or {}):
            raise PolicyError("openpi server response missing 'actions' key")
        actions = _actions_from_response(response, self.action_dim, max_rows=1 << 20)
        self.last_infer_s = time.perf_counter() - t0
        return actions


class LerobotActBackend(PolicyBackend):
    """In-process lerobot ACT policy loaded from a checkpoint directory."""

    name = "act"

    def __init__(
        self,
        *,
        checkpoint_dir: str,
        action_dim: int,
        image_keys: dict[str, str],  # feature label -> collect label (usually 1:1)
        device: str | None = None,
        default_prompt: str = "",
    ):
        self.checkpoint_dir = str(checkpoint_dir)
        self.action_dim = int(action_dim)
        self.image_keys = dict(image_keys)
        self.device = device
        self.default_prompt = default_prompt
        self._policy = None
        self._pre = None
        self._post = None
        self.last_infer_s = 0.0

    def open(self) -> None:
        if self._policy is not None:
            return
        try:
            from lerobot.policies import (
                PreTrainedPolicy,
                make_pre_post_processors,
            )
        except ImportError as exc:  # pragma: no cover
            raise PolicyError(
                "lerobot (this fork, v0.6.x) not importable — run this node in "
                "the lerobot environment or with VLA/lerobot/src on PYTHONPATH"
            ) from exc
        try:
            policy = PreTrainedPolicy.from_pretrained(self.checkpoint_dir)
            if self.device is not None:
                policy.to(self.device)
            pre, post = make_pre_post_processors(
                policy.config,
                pretrained_path=self.checkpoint_dir,
            )
        except Exception as exc:  # noqa: BLE001
            raise PolicyError(
                f"ACT checkpoint load failed from {self.checkpoint_dir}: {exc}"
            ) from exc
        self._policy = policy
        self._pre = pre
        self._post = post

    def close(self) -> None:
        self._policy = None
        self._pre = None
        self._post = None

    def reset(self) -> None:
        if self._policy is not None:
            try:
                self._policy.reset()
            except Exception:  # noqa: BLE001
                pass

    def infer(self, obs: ObsBatch) -> np.ndarray:
        if self._policy is None or self._pre is None or self._post is None:
            raise PolicyError("LerobotActBackend not open()ed")
        import torch

        raw: dict[str, np.ndarray] = {"observation.state": obs.state.astype(np.float32)}
        for label, cam in self.image_keys.items():
            img = obs.images.get(cam)
            if img is None:
                continue
            raw[f"observation.images.{label}"] = np.asarray(img, dtype=np.uint8)
        try:
            from lerobot.policies.utils import prepare_observation_for_inference

            with torch.inference_mode():
                obs_in = prepare_observation_for_inference(
                    raw,
                    next(self._policy.parameters()).device,
                    task=obs.prompt or self.default_prompt,
                    robot_type=None,
                )
                obs_in = self._pre(obs_in)
                t0 = time.perf_counter()
                action = self._policy.select_action(obs_in)
                action = self._post(action)
                self.last_infer_s = time.perf_counter() - t0
            arr = np.asarray(action.detach().cpu().squeeze(0), dtype=np.float64)
        except Exception as exc:  # noqa: BLE001
            raise PolicyError(f"ACT infer failed: {exc}") from exc
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.shape[1] != self.action_dim:
            raise PolicyError(
                f"ACT action_dim {arr.shape[1]} != robot action_dim {self.action_dim}"
            )
        return arr


class StubBackend(PolicyBackend):
    """Deterministic no-op backend for tests / MuJoCo smoke runs (no network).

    Inferences a short constant-ish chunk so engines/nodes can be exercised
    without openpi/lerobot installed. Not for real deployment.
    """

    name = "stub"

    def __init__(self, *, action_dim: int, camera_map: dict[str, str], **_: object):
        self.action_dim = int(action_dim)
        self.camera_map = dict(camera_map)
        self._n = 0
        self.last_infer_s = 0.0

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def dummy_obs(self, action_dim: int, camera: int = 8) -> ObsBatch:
        return ObsBatch(
            state=np.zeros(action_dim, dtype=np.float64),
            images={label: np.zeros((camera, camera, 3), np.uint8)
                    for label in self.camera_map.values()},
            prompt="",
        )

    def infer(self, obs: ObsBatch) -> np.ndarray:
        self._n += 1
        self.last_infer_s = 1e-3
        # a gentle absolute drift so ticks visibly differ (sim smoke checks).
        phase = float(self._n % 4)
        out = np.zeros((4, self.action_dim), dtype=np.float64)
        out[:] = 0.05 * np.cos(phase + np.arange(self.action_dim))
        return out


def make_backend(
    *,
    backend_type: str,
    action_dim: int,
    camera_map: dict[str, str],
    checkpoint_dir: str | None = None,
    host: str = "127.0.0.1",
    port: int = 8000,
    default_prompt: str = "",
    device: str | None = None,
) -> PolicyBackend:
    """Factory used by the node; backend_type in {openpi, act, stub}."""
    bt = (backend_type or "openpi").strip().lower()
    if bt in ("openpi", "pi0", "pi05", "pi"):
        return OpenPiServerBackend(
            host=host,
            port=port,
            action_dim=action_dim,
            slot_keys=camera_map,
            default_prompt=default_prompt,
        )
    if bt in ("act", "lerobot"):
        if not checkpoint_dir:
            raise PolicyError("act backend requires checkpoint_dir")
        return LerobotActBackend(
            checkpoint_dir=checkpoint_dir,
            action_dim=action_dim,
            image_keys={label: label for label in camera_map.values()},
            device=device,
            default_prompt=default_prompt,
        )
    if bt == "stub":
        return StubBackend(action_dim=action_dim, camera_map=camera_map)
    raise PolicyError(f"unknown backend_type={backend_type!r} (openpi|act|stub)")
