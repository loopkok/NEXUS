"""Host-side WebRTC sender for one outbound H.264/VP8 video track.

Self-contained (vendored, no hand-tracking-sdk dependency).

Includes a runtime patch that raises aiortc's H264/VP8 encoder bitrate floor so
that the Quest receiver's REMB feedback cannot drag the bitrate down to the
default ~0.5-1 Mbps (which makes 720p/1080p look blocky).
"""

from __future__ import annotations

import asyncio
import fractions
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from time import monotonic
from typing import Any


# ---------------------------------------------------------------------------
# Runtime bitrate patch.
#
# aiortc's H264 encoder defaults to 1 Mbps (DEFAULT_BITRATE), VP8 to 500 kbps,
# and both clamp to low ceilings (H264: 3 Mbps, VP8: 1.5 Mbps).  On a LAN the
# Quest's REMB feedback would otherwise cap the encoder at ~1.5 Mbps, making the
# image blocky.  We raise the floor and ceiling so the encoder stays crisp.
# ---------------------------------------------------------------------------
_BITRATE_MIN = 3_000_000      # 3 Mbps floor (720p clear)
_BITRATE_DEFAULT = 5_000_000  # 5 Mbps start (1080p clear)
_BITRATE_MAX = 12_000_000     # 12 Mbps ceiling (1080p crisp)

try:
    import aiortc.codecs.h264 as _h264
    _h264.DEFAULT_BITRATE = _BITRATE_DEFAULT
    _h264.MIN_BITRATE = _BITRATE_MIN
    _h264.MAX_BITRATE = _BITRATE_MAX
except Exception:
    pass

try:
    import aiortc.codecs.vpx as _vpx
    _vpx.DEFAULT_BITRATE = _BITRATE_DEFAULT
    _vpx.MIN_BITRATE = _BITRATE_MIN
    _vpx.MAX_BITRATE = _BITRATE_MAX
except Exception:
    pass


def install_bitrate_diagnostics(verbose: bool = False) -> None:
    """Optionally log encoder target_bitrate changes and the first codec.bit_rate.

    Only call this when you want the diagnostic lines; it wraps the encoder
    setter/encode so each REMB-driven change is visible.  Idempotent-ish: it
    replaces class attributes, so call once at startup.
    """
    if not verbose:
        return
    import logging

    _log = logging.getLogger("quest3_video_streamer")

    def _patch(cls, label):
        if cls is None or not hasattr(cls, "target_bitrate"):
            return
        prop = cls.target_bitrate
        if not isinstance(prop, property):
            return
        _orig_set = prop.fset

        def _logged_set(self, bitrate):
            _log.warning(f"[bitrate-diag][{label}] target_bitrate set to {bitrate}")
            if _orig_set is not None:
                _orig_set(self, bitrate)

        cls.target_bitrate = property(prop.fget, _logged_set)
        _orig_encode = cls.encode
        _done = {"v": False}

        def _logged_encode(self, frame, force_keyframe=False):
            codec = getattr(self, "codec", None)
            if not _done["v"] and codec is not None:
                _log.warning(
                    f"[bitrate-diag][{label}] first encode: codec.bit_rate="
                    f"{codec.bit_rate} target={self.target_bitrate} "
                    f"{codec.width}x{codec.height}"
                )
                _done["v"] = True
            return _orig_encode(self, frame, force_keyframe)

        cls.encode = _logged_encode

    try:
        _patch(__import__("aiortc.codecs.h264", fromlist=["H264Encoder"]).H264Encoder, "h264")
    except Exception:
        pass
    try:
        _patch(__import__("aiortc.codecs.vpx", fromlist=["Vp8Encoder"]).Vp8Encoder, "vp8")
    except Exception:
        pass


@dataclass(frozen=True, slots=True)
class TrackStats:
    index: int
    label: str
    fps: float
    bitrate_kbps: float
    frame_drops: int


@dataclass(frozen=True, slots=True)
class VideoSenderStats:
    fps: float
    bitrate_kbps: float
    frame_drops: int
    rtt_ms: float | None
    tracks: list[TrackStats] = None  # type: ignore[assignment]


class _AdapterVideoTrack:
    """Bridges a source adapter into the aiortc track API.

    Optional runtime gate: while ``gate.is_enabled(label)`` is false the track
    sends 2 fps black frames instead of camera frames.  This keeps the RTP
    stream (and the Quest panel) alive at negligible bandwidth without SDP
    renegotiation, and un-muting resumes the live feed instantly.

    Sources are opened lazily: a track only calls ``source.start()`` on its
    first un-muted frame, so cameras that are gated off (or whose device is
    missing) are never opened.  Open/read failures degrade to black frames
    with a 1 s retry backoff instead of killing the whole sender.
    """

    kind = "video"

    _MUTED_FPS = 2.0
    _RETRY_BACKOFF_S = 1.0

    def __init__(
        self,
        source: "VideoSourceAdapter",
        fps: int,
        gate: Any = None,
        label: str = "",
        log: Callable[[str], None] | None = None,
    ) -> None:
        self._source = source
        self._fps = max(1, fps)
        self._gate = gate
        self._label = label
        self._log = log or (lambda _msg: None)
        self._pts = 0
        self._time_base = fractions.Fraction(1, self._fps)
        self._started = False
        self._muted = False

    async def recv(self) -> Any:
        if self._gate is not None and not self._gate.is_enabled(self._label):
            await asyncio.sleep(1.0 / self._MUTED_FPS)
            frame = self._new_black_frame()
            self._muted = True
        else:
            self._muted = False
            frame = await self._live_frame()
        frame.pts = self._pts
        frame.time_base = self._time_base
        self._pts += 1
        return frame

    async def _live_frame(self) -> Any:
        if not self._started:
            try:
                await self._source.start()
                self._started = True
                self._log(f"[track {self._label}] source started (lazy open)")
            except Exception as exc:
                self._log(f"[track {self._label}] source open failed: {exc}; sending black")
                await asyncio.sleep(self._RETRY_BACKOFF_S)
                return self._new_black_frame()
        try:
            return await self._source.next_frame()
        except Exception as exc:
            self._log(f"[track {self._label}] next_frame failed: {exc}; sending black")
            try:
                await self._source.stop()
            except Exception:
                pass
            self._started = False  # retry open on the next recv
            await asyncio.sleep(self._RETRY_BACKOFF_S)
            return self._new_black_frame()

    def _new_black_frame(self) -> Any:
        """Fresh black frame at the source resolution (2 fps → cheap)."""
        import av  # lazy: only needed once a track is actually muted

        fmt = self._source.get_format()
        frame = av.VideoFrame(fmt.width, fmt.height, "yuv420p")
        # Limited-range black: Y=16, U=V=128.
        frame.planes[0].update(bytes([16]) * (fmt.width * fmt.height))
        cw, ch = fmt.width // 2, fmt.height // 2
        frame.planes[1].update(bytes([128]) * (cw * ch))
        frame.planes[2].update(bytes([128]) * (cw * ch))
        return frame


class VideoWebRTCSender:
    """One-to-one sender peer for host->Quest video.

    Supports one or more outbound video tracks (one per source adapter) so a
    single peer connection can carry multiple cameras (e.g. D435i + 2 wrist
    USB cams). The Quest offer must contain one recv-only video transceiver per
    source; the answer then carries one send track per source, in order.
    """

    def __init__(
        self,
        *,
        sources: "list[VideoSourceAdapter]",
        on_local_ice_candidate: Callable[[dict[str, Any]], Awaitable[None]] | None = None,
        log_hook: Callable[[str], None] | None = None,
        gate: Any = None,
    ) -> None:
        if not sources:
            raise ValueError("At least one video source is required.")
        self._sources = list(sources)
        self._on_local_ice_candidate = on_local_ice_candidate
        self._log_hook = log_hook
        # Optional runtime gate (StreamGate): muted tracks send 2 fps black.
        self._gate = gate
        self._pc: Any = None
        self._created_at = monotonic()
        self._frames_sent = 0
        self._bytes_sent = 0
        self._frame_drops = 0
        self._last_stats_lock = asyncio.Lock()
        self._h264_forced = False
        # Per-track accounting for per-camera bandwidth/fps stats.
        # track_id -> {index, label}; last_bytes/last_frames keyed by track_id.
        self._track_info: list[dict[str, Any]] = []
        self._last_bytes: dict[str, int] = {}
        self._last_frames: dict[str, int] = {}
        self._last_stats_time: float = monotonic()
        # Per-track frame counters incremented in AdapterTrack.recv(); aiortc's
        # outbound-rtp stats have no framesSent field, so fps is derived here.
        self._track_frame_counts: list[int] = []

    async def start(self) -> None:
        # Sources are NOT started here: each track lazily opens its source on
        # the first un-muted frame (see _AdapterVideoTrack), so gated-off or
        # missing cameras are never opened.
        self._pc = self._new_peer_connection()
        self._add_video_tracks()
        self._wire_ice_callbacks()

    async def stop(self) -> None:
        if self._pc is not None:
            await self._pc.close()
            self._pc = None
        for src in self._sources:
            try:
                await src.stop()  # adapters no-op when never started
            except Exception:
                pass

    async def apply_offer(self, *, sdp_offer: str) -> str:
        if self._pc is None:
            raise RuntimeError("Video sender not started.")
        rtc_session_description = self._import_aiortc_symbol("RTCSessionDescription")
        offer = rtc_session_description(sdp=sdp_offer, type="offer")
        await self._pc.setRemoteDescription(offer)
        self._force_h264_codec_if_possible()
        answer = await self._pc.createAnswer()
        await self._pc.setLocalDescription(answer)
        answer_sdp = str(self._pc.localDescription.sdp)
        try:
            video_codecs: list[str] = []
            in_video = False
            for line in answer_sdp.splitlines():
                if line.startswith("m=video"):
                    in_video = True
                    video_codecs.append("unknown")
                    continue
                if line.startswith("m="):
                    in_video = False
                if in_video and line.startswith("a=rtpmap:") and video_codecs and video_codecs[-1] == "unknown":
                    payload = line.split(":", 1)[1]
                    enc = payload.split(" ", 1)[1].split("/", 1)[0] if " " in payload else ""
                    if enc:
                        video_codecs[-1] = enc
            self._log(f"negotiated video codecs: {video_codecs} (h264_forced={self._h264_forced})")
        except Exception:
            pass
        for t in self._pc.getTransceivers():
            self._log(
                f"transceiver kind={t.kind} direction={t.direction} "
                f"currentDirection={t.currentDirection}"
            )
        return answer_sdp

    async def add_remote_ice_candidate(
        self,
        *,
        candidate: str,
        sdp_mid: str | None,
        sdp_mline_index: int | None,
    ) -> None:
        if self._pc is None or not candidate:
            return
        candidate_from_sdp = self._import_aiortc_sdp_symbol("candidate_from_sdp")
        parsed = candidate_from_sdp(candidate)
        parsed.sdpMid = sdp_mid
        parsed.sdpMLineIndex = sdp_mline_index
        await self._pc.addIceCandidate(parsed)

    async def get_stats(self) -> VideoSenderStats:
        if self._pc is None:
            return VideoSenderStats(fps=0.0, bitrate_kbps=0.0, frame_drops=0, rtt_ms=None, tracks=[])
        async with self._last_stats_lock:
            try:
                report = await self._pc.getStats()
            except Exception:
                report = {}
            now = monotonic()
            dt = max(0.001, now - self._last_stats_time)
            self._last_stats_time = now

            rtt_ms: float | None = None
            # aiortc's outbound-rtp stats only carry bytesSent (no framesSent),
            # so bitrate comes from bytesSent and fps from our own per-track
            # frame counters (incremented in AdapterTrack.recv). Collect bytes
            # per outbound stream; order is stable across calls for index mapping.
            ob_bytes: list[int] = []
            for stat in (report.values() if hasattr(report, "values") else []):
                stat_type = getattr(stat, "type", "")
                if stat_type == "outbound-rtp":
                    ob_bytes.append(int(getattr(stat, "bytesSent", 0)))
                if stat_type == "candidate-pair":
                    current_rtt = getattr(stat, "currentRoundTripTime", None)
                    if current_rtt is not None:
                        rtt_ms = float(current_rtt) * 1000.0

            n = len(self._track_info)
            # Per-track bitrate from bytesSent deltas (order-mapped).
            track_stats: list[TrackStats] = []
            total_bitrate_kbps = 0.0
            total_fps = 0.0
            for i in range(n):
                b = ob_bytes[i] if i < len(ob_bytes) else 0
                prev_b = self._last_bytes.get(str(i), 0)
                db = max(0, b - prev_b)
                self._last_bytes[str(i)] = b
                bitrate_kbps = (db * 8.0 / dt) / 1000.0
                # fps from our own frame counter.
                cur_f = self._track_frame_counts[i] if i < len(self._track_frame_counts) else 0
                prev_f = self._last_frames.get(str(i), 0)
                df = max(0, cur_f - prev_f)
                self._last_frames[str(i)] = cur_f
                fps = df / dt
                total_bitrate_kbps += bitrate_kbps
                total_fps += fps
                track_stats.append(TrackStats(
                    index=i, label=self._track_info[i]["label"],
                    fps=fps, bitrate_kbps=bitrate_kbps, frame_drops=0,
                ))

            return VideoSenderStats(
                fps=total_fps,
                bitrate_kbps=total_bitrate_kbps,
                frame_drops=self._frame_drops,
                rtt_ms=rtt_ms,
                tracks=track_stats,
            )

    # -- internals --------------------------------------------------------

    def _new_peer_connection(self) -> Any:
        rtc_peer_connection = self._import_aiortc_symbol("RTCPeerConnection")
        rtc_configuration = self._import_aiortc_symbol("RTCConfiguration")
        rtc_ice_server = self._import_aiortc_symbol("RTCIceServer")
        config = rtc_configuration(
            iceServers=[rtc_ice_server(urls=["stun:stun.l.google.com:19302"])]
        )
        pc = rtc_peer_connection(config)
        self._wire_connection_state(pc)
        return pc

    def _wire_connection_state(self, pc: Any) -> None:
        @pc.on("connectionstatechange")
        async def _on_state() -> None:
            self._log(f"connection state: {pc.connectionState}")

        @pc.on("iceconnectionstatechange")
        async def _on_ice_state() -> None:
            self._log(f"ICE connection state: {pc.iceConnectionState}")

        @pc.on("icegatheringstatechange")
        async def _on_ice_gathering() -> None:
            self._log(f"ICE gathering state: {pc.iceGatheringState}")

    def _log(self, message: str) -> None:
        if self._log_hook is not None:
            self._log_hook(message)

    def _add_video_tracks(self) -> None:
        if self._pc is None:
            raise RuntimeError("Peer connection not created.")
        video_stream_track = self._import_aiortc_symbol("VideoStreamTrack")

        class AdapterTrack(video_stream_track):  # type: ignore[misc, valid-type]
            def __init__(self, adapter: _AdapterVideoTrack, sender: VideoWebRTCSender, idx: int) -> None:
                super().__init__()
                self._adapter = adapter
                self._sender = sender
                self._idx = idx

            async def recv(self) -> Any:
                try:
                    frame = await self._adapter.recv()
                except Exception as exc:
                    import traceback

                    self._sender._log(f"recv() error: {exc}\n{''.join(traceback.format_exc())}")
                    raise
                self._sender._frames_sent += 1
                if frame is not None:
                    # Count actual frames delivered for per-track fps stats.
                    self._sender._track_frame_counts[self._idx] += 1
                else:
                    self._sender._frame_drops += 1
                return frame

        self._track_frame_counts = [0] * len(self._sources)
        for idx, source in enumerate(self._sources):
            video_format = source.get_format()
            track = AdapterTrack(
                _AdapterVideoTrack(
                    source, fps=video_format.fps,
                    gate=self._gate, label=video_format.label,
                    log=self._log,
                ),
                self, idx,
            )
            self._pc.addTrack(track)
            self._track_info.append({"index": idx, "label": video_format.label, "id": getattr(track, "id", str(idx))})
            self._log(
                f"added video track #{idx} label={video_format.label} "
                f"{video_format.width}x{video_format.height}@{video_format.fps} "
                f"fov_h={video_format.fov_h_deg}"
            )

    def _wire_ice_callbacks(self) -> None:
        if self._pc is None or self._on_local_ice_candidate is None:
            return

        @self._pc.on("icecandidate")
        async def _on_icecandidate(candidate: Any) -> None:
            if candidate is None:
                return
            candidate_str = str(getattr(candidate, "candidate", ""))
            self._log(f"local ICE candidate: {candidate_str[:80]}")
            await self._on_local_ice_candidate(
                {
                    "candidate": candidate_str,
                    "sdpMid": getattr(candidate, "sdpMid", None),
                    "sdpMLineIndex": getattr(candidate, "sdpMLineIndex", None),
                }
            )

    def _force_h264_codec_if_possible(self) -> None:
        if self._pc is None:
            return
        try:
            rtc_rtp_sender = self._import_aiortc_symbol("RTCRtpSender")
            capabilities = rtc_rtp_sender.getCapabilities("video")
            codecs = [
                codec
                for codec in getattr(capabilities, "codecs", [])
                if str(getattr(codec, "mimeType", "")).lower() == "video/h264"
            ]
            if not codecs:
                return
            for transceiver in self._pc.getTransceivers():
                if getattr(transceiver, "kind", "") == "video":
                    transceiver.setCodecPreferences(codecs)
                    self._h264_forced = True
        except Exception:
            return

    def _import_aiortc_symbol(self, symbol: str) -> Any:
        try:
            return getattr(__import__("aiortc", fromlist=[symbol]), symbol)
        except Exception as exc:
            raise RuntimeError(
                "aiortc is required for video streaming. "
                "Install with: python3 -m pip install aiortc av"
            ) from exc

    def _import_aiortc_sdp_symbol(self, symbol: str) -> Any:
        try:
            return getattr(__import__("aiortc.sdp", fromlist=[symbol]), symbol)
        except Exception as exc:
            raise RuntimeError("aiortc.sdp unavailable.") from exc


# Forward declaration for type hints (the real base lives in ros_source.py /
# webcam_source.py and is duck-typed here).
class VideoSourceAdapter:  # pragma: no cover - structural protocol
    async def start(self) -> None: ...
    async def stop(self) -> None: ...
    async def next_frame(self) -> Any: ...
    def get_format(self) -> "VideoFormat": ...


