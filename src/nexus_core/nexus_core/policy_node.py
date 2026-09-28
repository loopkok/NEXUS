"""Profile-driven remote policy, replay and human takeover for NEXUS."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from astral_policy_inference.backend import ObsBatch, make_backend
from astral_policy_inference.engine import ActionEngine, EngineStateError
from astral_policy_inference.robot_io import decode_jpeg_rgb, letterbox
from .profile import Profile, verify_model_manifest


class NexusPolicyNode(Node):
    def __init__(self):
        super().__init__("nexus_policy")
        for key, value in {
            "profile_file": "", "model_manifest": "", "backend_type": "remote",
            "model": "act", "host": "127.0.0.1", "port": 8001,
            "prompt": "", "image_size": 224, "replay_path": "",
            "observation_timeout": 0.5, "image_timeout": 0.5,
            "infer_timeout": 3.0, "chunk": 50,
        }.items():
            self.declare_parameter(key, value)
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        manifest_file = str(self.get_parameter("model_manifest").value).strip()
        self.manifest = None
        if manifest_file:
            with open(manifest_file, encoding="utf-8") as fh:
                self.manifest = json.load(fh)
            verify_model_manifest(self.profile, self.manifest)
        self._mode = "IDLE"
        self._fault = ""
        self._state: dict[str, tuple[np.ndarray, float]] = {}
        self._images: dict[str, tuple[np.ndarray, float]] = {}
        self._engine = None
        self._pending: queue.Queue = queue.Queue()
        self._busy = False
        self._generation = 0
        self._transition_previous = "IDLE"
        self._takeover = None
        self._replay = None
        self._replay_index = 0
        self._replay_offset = np.zeros(self.profile.dimension, dtype=np.float64)
        self._interrupted = "IDLE"
        self._last_vector = None
        self._prompt = str(self.get_parameter("prompt").value)
        self._pubs = {}
        for spec in self.profile.components:
            self.create_subscription(JointState, self.profile.topic(spec.name, "joint_states"),
                                     lambda msg, n=spec.name: self._on_state(n, msg),
                                     qos_profile_sensor_data)
            self._pubs[spec.name] = self.create_publisher(
                JointState, self.profile.candidate_topic("policy", spec.name),
                qos_profile_sensor_data)
        self._replay_pubs = {spec.name: self.create_publisher(
            JointState, self.profile.candidate_topic("playback", spec.name),
            qos_profile_sensor_data) for spec in self.profile.components}
        for camera in self.profile.raw["cameras"]:
            role = camera["role"]
            self.create_subscription(CompressedImage,
                f"{self.profile.namespace}/camera/{role}/image/compressed",
                lambda msg, r=role: self._on_image(r, msg), qos_profile_sensor_data)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status = self.create_publisher(String, "/policy_inference/state", status_qos)
        self._disarm = self.create_publisher(Bool, "/teleop/disarm", 10)
        self.create_subscription(String, "/policy_inference/cmd", self._on_cmd, 10)
        self.create_subscription(String, "/policy_inference/task", self._on_task, 10)
        self._reanchor = {
            component.name: self.create_client(
                Trigger, f"/teleop_{component.name}/reanchor")
            for component in self.profile.components if component.kind == "arm"
        }
        self.create_timer(1.0 / self.profile.raw["dataset"]["fps"], self._tick)
        self.create_timer(0.05, self._poll)
        self.create_timer(1.0, self._publish_state)
        self._publish_state()
        self.get_logger().info(f"NEXUS policy profile_sha256={self.profile.digest} "
                               f"manifest={manifest_file or '<required for real model>'}")

    def _on_state(self, name, msg):
        spec = self.profile.component(name)
        if list(msg.name) != list(spec.joints) or len(msg.position) != spec.dim:
            return
        values = np.asarray(msg.position, dtype=np.float64)
        if not np.isfinite(values).all():
            return
        self._state[name] = values.copy(), time.monotonic()

    def _on_image(self, role, msg):
        frame = decode_jpeg_rgb(bytes(msg.data))
        if frame is None:
            return
        size = int(self.get_parameter("image_size").value)
        if size > 0:
            frame = letterbox(frame, size)
        self._images[role] = frame, time.monotonic()

    def _observation(self, require_images=True):
        now = time.monotonic()
        state_timeout = float(self.get_parameter("observation_timeout").value)
        image_timeout = float(self.get_parameter("image_timeout").value)
        missing = [c.name for c in self.profile.components
                   if c.name not in self._state or now - self._state[c.name][1] > state_timeout]
        roles = set(self.profile.raw["policy"]["camera_map"].values())
        if require_images:
            missing += [f"camera:{r}" for r in roles
                        if r not in self._images or now - self._images[r][1] > image_timeout]
        if missing:
            return None, missing
        vector = np.concatenate([self._state[c.name][0] for c in self.profile.components])
        images = {r: self._images[r][0] for r in roles if r in self._images}
        return ObsBatch(state=vector.astype(np.float32), images=images,
                        prompt=self._prompt), []

    def _set(self, mode, fault=""):
        self._mode = mode
        self._fault = fault
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = json.dumps({"state": self._mode, "fault": self._fault,
            "model": str(self.get_parameter("model").value),
            "profile_id": self.profile.profile_id,
            "profile_sha256": self.profile.digest,
            "ready": sorted(self._state), "cameras": sorted(self._images)},
            ensure_ascii=False)
        self._status.publish(msg)

    def _on_task(self, msg):
        self._prompt = msg.data

    def _on_cmd(self, msg):
        command = msg.data.strip().lower()
        try:
            if command == "policy":
                self._start_policy()
            elif command == "playback":
                self._start_playback()
            elif command == "takeover":
                self._start_takeover()
            elif command == "release":
                self._release()
            elif command == "pause" and self._mode in ("POLICY", "PLAYBACK"):
                self._set(self._mode + "_PAUSED")
            elif command == "resume" and self._mode in ("POLICY_PAUSED", "PLAYBACK_PAUSED"):
                self._set(self._mode.removesuffix("_PAUSED"))
            elif command == "stop":
                self._stop()
            else:
                raise ValueError(f"invalid command {command} from {self._mode}")
        except Exception as exc:
            self._fault = str(exc)
            self.get_logger().error(self._fault)
            self._publish_state()

    def _spawn(self, task):
        if self._busy:
            raise RuntimeError("policy transition already in progress")
        self._busy = True
        self._generation += 1
        generation = self._generation
        def run():
            try:
                result = task()
                self._pending.put((generation, True, result))
            except Exception as exc:
                self._pending.put((generation, False, exc))
        threading.Thread(target=run, daemon=True, name="nexus-policy-start").start()

    def _start_policy(self):
        if self._mode not in ("IDLE", "HUMAN", "POLICY_PAUSED"):
            raise RuntimeError("policy start requires IDLE, HUMAN or POLICY_PAUSED")
        if self.manifest is None and self.get_parameter("backend_type").value != "stub":
            raise RuntimeError("model_manifest required before enabling inference")
        obs, missing = self._observation()
        if obs is None:
            raise RuntimeError(f"cannot start policy: missing {missing}")
        previous = self._mode
        self._transition_previous = previous
        self._set("POLICY_PAUSED", "loading policy")
        self._disarm.publish(Bool(data=True))

        def start():
            if self._engine is not None:
                self._engine.stop()
            backend = make_backend(
                backend_type=str(self.get_parameter("backend_type").value),
                model=str(self.get_parameter("model").value),
                action_dim=self.profile.dimension,
                camera_map=self.profile.raw["policy"]["camera_map"],
                host=str(self.get_parameter("host").value),
                port=int(self.get_parameter("port").value),
                default_prompt=self._prompt,
                infer_timeout_s=float(self.get_parameter("infer_timeout").value),
                expected_profile_sha256=self.profile.digest if self.manifest else None)
            engine = ActionEngine(backend, mode="queue_async",
                action_dim=self.profile.dimension,
                chunk=int(self.get_parameter("chunk").value),
                policy_fps=self.profile.raw["dataset"]["fps"])
            engine.feed_obs(obs)
            engine.start()
            return ("POLICY", engine, previous)

        self._spawn(start)

    def _start_playback(self):
        if self._mode not in ("IDLE", "HUMAN"):
            raise RuntimeError("playback requires IDLE or HUMAN")
        path = Path(str(self.get_parameter("replay_path").value)).expanduser()
        if not path.is_file():
            raise FileNotFoundError(path)
        import h5py
        with h5py.File(path, "r") as fh:
            schema = json.loads(fh.attrs["schema"])
            if schema.get("profile_sha256") != self.profile.digest:
                raise RuntimeError("playback profile digest mismatch")
            if json.loads(fh.attrs["state_names"]) != [n for c in self.profile.components for n in c.joints]:
                raise RuntimeError("playback joint order mismatch")
            actions = np.asarray(fh["action"][:], dtype=np.float64)
        if actions.ndim != 2 or actions.shape[1] != self.profile.dimension or not np.isfinite(actions).all():
            raise RuntimeError("playback action shape/values invalid")
        self._replay, self._replay_index = actions, 0
        self._replay_offset[:] = 0
        self._disarm.publish(Bool(data=True))
        self._set("PLAYBACK")

    def _start_takeover(self):
        if self._mode not in ("POLICY", "POLICY_PAUSED", "PLAYBACK", "PLAYBACK_PAUSED"):
            raise RuntimeError("takeover requires policy or playback")
        obs, missing = self._observation(require_images=False)
        if obs is None:
            raise RuntimeError(f"takeover needs fresh measured joints: {missing}")
        self._interrupted = self._mode
        self._set("POLICY_PAUSED" if self._mode.startswith("POLICY") else "PLAYBACK_PAUSED")
        pending = {}
        for side, client in self._reanchor.items():
            if not client.service_is_ready():
                self._set(self._mode, f"reanchor service {side} unavailable")
                return
            pending[side] = client.call_async(Trigger.Request())
        self._takeover = (pending, time.monotonic() + 3.0)

    def _release(self):
        if self._mode != "HUMAN":
            raise RuntimeError("release requires HUMAN")
        obs, missing = self._observation(require_images=self._interrupted.startswith("POLICY"))
        if obs is None:
            raise RuntimeError(f"release needs fresh observation: {missing}")
        self._disarm.publish(Bool(data=True))
        if self._interrupted.startswith("PLAYBACK"):
            if self._replay is None or self._replay_index >= len(self._replay):
                self._set("IDLE")
                return
            self._replay_offset = obs.state - self._replay[self._replay_index]
            self._set("PLAYBACK")
        else:
            self._start_policy()

    def _stop(self):
        self._generation += 1
        self._busy = False
        self._takeover = None
        self._replay = None
        self._disarm.publish(Bool(data=False))
        if self._engine is not None:
            self._engine.set_enabled(False)
        self._set("IDLE")

    def _poll(self):
        if self._takeover is not None:
            pending, deadline = self._takeover
            if all(f.done() for f in pending.values()):
                failures = []
                for side, future in pending.items():
                    try:
                        result = future.result()
                        if not result.success:
                            failures.append(f"{side}: {result.message}")
                    except Exception as exc:
                        failures.append(f"{side}: {exc}")
                self._takeover = None
                if failures:
                    self._disarm.publish(Bool(data=True))
                    self._set(self._mode, f"takeover failed: {failures}")
                else:
                    if self._engine is not None:
                        self._engine.set_enabled(False)
                        self._engine.reset()
                    self._disarm.publish(Bool(data=False))
                    self._set("HUMAN")
            elif time.monotonic() > deadline:
                self._takeover = None
                self._disarm.publish(Bool(data=True))
                self._set(self._mode, "takeover reanchor timed out")
        try:
            generation, ok, result = self._pending.get_nowait()
        except queue.Empty:
            return
        if generation != self._generation:
            if ok and isinstance(result, tuple) and result[1] is not None:
                result[1].stop()
            return
        self._busy = False
        if ok:
            _mode, engine, _previous = result
            self._engine = engine
            self._set("POLICY")
        else:
            if self._transition_previous == "HUMAN":
                # Teleop was disarmed before connecting to the model. Do not
                # claim HUMAN control until a fresh takeover reanchors both
                # arms to measured state again.
                self._set("POLICY_PAUSED", f"policy start failed: {result}; request takeover")
            else:
                self._set(self._transition_previous, f"policy start failed: {result}")

    def _publish_vector(self, vector, source):
        values = np.asarray(vector, dtype=np.float64).reshape(-1)
        if len(values) != self.profile.dimension or not np.isfinite(values).all():
            raise ValueError("policy action dimension or value invalid")
        cursor = 0
        for spec in self.profile.components:
            part = values[cursor:cursor + spec.dim]
            cursor += spec.dim
            part = np.clip(part, spec.lower, spec.upper)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(spec.joints)
            msg.position = part.tolist()
            (self._pubs if source == "policy" else self._replay_pubs)[spec.name].publish(msg)
        self._last_vector = values

    def _tick(self):
        if self._mode == "POLICY" and self._engine is not None:
            obs, missing = self._observation()
            if obs is None:
                self._set("POLICY_PAUSED", f"observation timeout: {missing}")
                self._engine.set_enabled(False)
                return
            self._engine.feed_obs(obs)
            try:
                row = self._engine.tick()
                if row is not None:
                    self._publish_vector(row, "policy")
            except (EngineStateError, ValueError) as exc:
                self._engine.set_enabled(False)
                self._set("POLICY_PAUSED", f"inference failed: {exc}")
        elif self._mode == "PLAYBACK" and self._replay is not None:
            if self._replay_index >= len(self._replay):
                self._set("IDLE")
                return
            self._publish_vector(self._replay[self._replay_index] + self._replay_offset, "playback")
            self._replay_index += 1

    def destroy_node(self):
        if self._engine is not None:
            self._engine.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = NexusPolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
