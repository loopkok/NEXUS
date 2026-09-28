"""Single-writer command arbitration, independent of ROS and hardware SDKs."""

from __future__ import annotations

import math
from dataclasses import dataclass
from .profile import Profile, ProfileError

SOURCES = {"TELEOP": "teleop", "POLICY": "policy", "PLAYBACK": "playback"}


@dataclass(frozen=True)
class Sample:
    values: tuple[float, ...]
    received_at: float


class CommandArbiter:
    """One authoritative joint-command vector per configured component.

    A source change is seeded from measured state. No source can drive a
    component until its feedback and its own command have arrived. Loss of a
    selected command pauses the whole assembly and holds the last output.
    """

    def __init__(self, profile: Profile, command_timeout: float = 0.35,
                 state_timeout: float = 0.5, max_velocity: float = 6.0,
                 startup_timeout: float = 5.0):
        if min(command_timeout, state_timeout, max_velocity, startup_timeout) <= 0:
            raise ValueError("timeouts and max_velocity must be positive")
        self.profile = profile
        self.command_timeout = command_timeout
        self.state_timeout = state_timeout
        self.max_velocity = max_velocity
        self.startup_timeout = startup_timeout
        self.mode = "IDLE"
        self.resume_mode = "IDLE"
        self.fault = ""
        self.states: dict[str, Sample] = {}
        self.candidates: dict[tuple[str, str], Sample] = {}
        self.last_out: dict[str, tuple[float, ...]] = {}
        self.last_tick: float | None = None
        self.estopped = False
        self.mode_since = 0.0
        self.source_started = False

    def _normalize(self, component: str, names: list[str] | tuple[str, ...],
                   positions: list[float] | tuple[float, ...]) -> tuple[float, ...]:
        spec = self.profile.component(component)
        if len(positions) != spec.dim:
            raise ProfileError(f"{component}: expected {spec.dim} positions, got {len(positions)}")
        if list(names) != list(spec.joints):
            raise ProfileError(f"{component}: joint names/order do not match profile")
        values = tuple(float(v) for v in positions)
        if not all(math.isfinite(v) for v in values):
            raise ProfileError(f"{component}: non-finite joint value")
        return values

    def update_state(self, component: str, names: list[str], positions: list[float], now: float) -> None:
        values = self._normalize(component, names, positions)
        spec = self.profile.component(component)
        if any(v < lo - 0.05 or v > hi + 0.05
               for v, lo, hi in zip(values, spec.lower, spec.upper)):
            raise ProfileError(f"{component}: measured state outside joint limits")
        self.states[component] = Sample(values, now)

    def update_candidate(self, source: str, component: str, names: list[str],
                         positions: list[float], now: float) -> None:
        if source not in SOURCES.values():
            raise ProfileError(f"unknown source {source}")
        values = self._normalize(component, names, positions)
        self.candidates[source, component] = Sample(values, now)

    def select(self, mode: str, now: float) -> None:
        mode = mode.upper()
        if mode == "ESTOP":
            self.estopped = True
            self.mode = "ESTOP"
            self.fault = "emergency stop"
            return
        if self.estopped:
            raise ProfileError("emergency stop is latched; reset at the hardware adapter")
        if mode == "PAUSED":
            if self.mode in SOURCES:
                self.resume_mode = self.mode
            # Pause is a measured-pose hold, including the start of HITL
            # takeover. A queued target may be ahead of the physical robot.
            self.last_out = {**self.last_out, **{
                name: sample.values for name, sample in self.states.items()
                if now - sample.received_at <= self.state_timeout}}
            self.mode = "PAUSED"
            return
        if mode == "RESUME":
            mode = self.resume_mode
        if mode not in (*SOURCES, "IDLE"):
            raise ProfileError(f"invalid mode {mode}")
        if mode in SOURCES:
            missing = [c.name for c in self.profile.components
                       if c.name not in self.states or now - self.states[c.name].received_at > self.state_timeout]
            if missing:
                raise ProfileError(f"cannot enable {mode}: stale/missing feedback {missing}")
            if mode != self.mode:
                self.last_out = {c.name: self.states[c.name].values for c in self.profile.components}
                self.last_tick = now
                self.mode_since = now
                self.source_started = False
                # A prior run of the same source must not execute its last
                # candidate during release/restart before a fresh plan arrives.
                for key in list(self.candidates):
                    if key[0] == SOURCES[mode]:
                        del self.candidates[key]
        self.mode = mode
        self.fault = ""

    def tick(self, now: float) -> dict[str, tuple[float, ...]]:
        if self.estopped:
            return {}
        dt = 0.01 if self.last_tick is None else max(0.001, min(0.1, now - self.last_tick))
        self.last_tick = now
        if self.mode in SOURCES:
            source = SOURCES[self.mode]
            missing = [c.name for c in self.profile.components
                       if (source, c.name) not in self.candidates or
                       now - self.candidates[source, c.name].received_at > self.command_timeout]
            stale_state = [c.name for c in self.profile.components
                           if c.name not in self.states or now - self.states[c.name].received_at > self.state_timeout]
            if missing or stale_state:
                if missing and not stale_state and not self.source_started and now - self.mode_since < self.startup_timeout:
                    return self.last_out.copy()
                self.resume_mode = self.mode
                self.mode = "PAUSED"
                self.fault = f"stale command={missing} feedback={stale_state}"
                # A stale output must never be refreshed into a driver when
                # feedback has disappeared: let its local watchdog take over.
                if stale_state:
                    return {}
                self.last_out = {c.name: self.states[c.name].values
                                 for c in self.profile.components}
            else:
                self.source_started = True
                out: dict[str, tuple[float, ...]] = {}
                for spec in self.profile.components:
                    goal = self.candidates[source, spec.name].values
                    prev = self.last_out.get(spec.name, self.states[spec.name].values)
                    step = self.max_velocity * dt
                    out[spec.name] = tuple(min(hi, max(lo, p + min(step, max(-step, v - p))))
                                           for p, v, lo, hi in zip(prev, goal, spec.lower, spec.upper))
                self.last_out = out
                return out.copy()
        if self.last_out:
            return {name: values for name, values in self.last_out.items()
                    if name in self.states and now - self.states[name].received_at <= self.state_timeout}
        return {name: sample.values for name, sample in self.states.items()
                if now - sample.received_at <= self.state_timeout}

    def status(self) -> dict:
        return {"mode": self.mode, "fault": self.fault,
                "profile_id": self.profile.profile_id,
                "profile_sha256": self.profile.digest,
                "ready_components": sorted(self.states)}
