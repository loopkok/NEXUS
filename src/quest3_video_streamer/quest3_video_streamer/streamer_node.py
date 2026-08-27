"""Quest 3 video streamer ROS 2 node.

Subscribes to one or more camera image sources (ROS ``sensor_msgs/Image``
topics from ``realsense2_camera`` / ``usb_cam``, or direct OpenCV webcams) and
pushes them to Quest 3 over WebRTC using a self-contained signaling + sender
pipeline. One outbound WebRTC video track is created per source.

Configuration is yaml-driven (``config/params.yaml``). The ``cameras`` array
lists which cameras to stream; each camera has a nested block
(``d435i``, ``wrist_left``, ...) with source/device/preset/fov/layout. Launch
files load the yaml and do not hardcode these tunables.

Threading model:
  * rclpy spins in a daemon thread (delivers image callbacks).
  * the asyncio WebRTC service runs in the main thread.
  * the ROS image callback hands frames to the asyncio loop thread-safely.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from quest3_video_streamer.gate import StreamGate
from quest3_video_streamer.service import (
    Quest3VideoService,
    VideoServiceConfig,
    parse_preset,
)
from quest3_video_streamer.source_base import VideoSourceAdapter
from quest3_video_streamer.webrtc_sender import install_bitrate_diagnostics


_LOG = logging.getLogger("quest3_video_streamer")


def _build_one_source(node: Node, spec: dict[str, Any]) -> VideoSourceAdapter:
    source_type = str(spec.get("type", spec.get("source_type", "ros"))).lower()
    width, height, fps = parse_preset(str(spec.get("preset", "720p30")))
    fov_h = float(spec.get("fov_h_deg", 69.0))
    label = str(spec.get("label", "camera"))

    if source_type == "webcam":
        from quest3_video_streamer.webcam_source import WebcamSourceAdapter

        device_index = int(spec.get("webcam_index", spec.get("device_index", 0)))
        force_mjpg = bool(spec.get("force_mjpg", True))
        _LOG.info(
            f"source=webcam device_index={device_index} {width}x{height}@{fps} "
            f"fov_h={fov_h} label={label} force_mjpg={force_mjpg}"
        )
        return WebcamSourceAdapter(
            device_index=device_index, width=width, height=height, fps=fps,
            fov_h_deg=fov_h, label=label, force_mjpg=force_mjpg,
        )

    # default: ros image topic
    from quest3_video_streamer.ros_source import RosImageSourceAdapter

    topic = str(spec.get("topic", spec.get("image_topic", "/camera/camera/color/image_raw")))
    _LOG.info(
        f"source=ros topic={topic} {width}x{height}@{fps} fov_h={fov_h} label={label}"
    )
    return RosImageSourceAdapter(
        node=node, topic=topic, width=width, height=height, fps=fps,
        fov_h_deg=fov_h, label=label,
    )


def _device_to_index(device: Any) -> int:
    """Accept '/dev/videoN' or an int and return the int index for cv2."""
    s = str(device)
    if s.startswith("/dev/video"):
        return int(s.replace("/dev/video", ""))
    return int(s)


def _build_sources(
    node: Node, params: dict[str, Any]
) -> tuple[list[VideoSourceAdapter], list[dict[str, Any]], list[dict[str, Any]]]:
    """Build the list of video sources + parallel display layouts + cam info.

    Priority:
      1. ``sources_json`` (advanced override) — one source per JSON entry.
      2. ``auto_scan: true`` — enumerate host capture devices (see scan.py);
         a yaml block matching the scanned label overrides individual fields.
         Falls back to the ``cameras`` list when no device is found.
      3. ``cameras`` list from yaml — build one source per named camera block.
    """
    sources_json = str(params.get("sources_json", "") or "").strip()
    if sources_json:
        try:
            specs = json.loads(sources_json)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid sources_json: {exc}") from exc
        if not isinstance(specs, list) or not specs:
            raise RuntimeError("sources_json must be a non-empty JSON array.")
        sources: list[VideoSourceAdapter] = []
        layouts: list[dict[str, Any]] = []
        infos: list[dict[str, Any]] = []
        for spec in specs:
            sources.append(_build_one_source(node, spec))
            layouts.append(dict(spec.get("layout", {})))
            infos.append({
                "label": str(spec.get("label", "camera")),
                "device": str(spec.get("device", spec.get("webcam_index", ""))),
                "source": str(spec.get("type", "ros")),
                "preset": str(spec.get("preset", "")),
                "sysfs_name": "",
            })
        _LOG.info(f"built {len(sources)} sources from sources_json")
        return sources, layouts, infos

    if bool(params.get("auto_scan", False)):
        from quest3_video_streamer.scan import enumerate_capture_devices

        found = enumerate_capture_devices()
        if found:
            n = len(found)
            sources = []
            layouts = []
            infos = []
            for i, dev in enumerate(found):
                spec, layout = _scan_spec(node, dev, i, n)
                sources.append(_build_one_source(node, spec))
                layouts.append(layout)
                infos.append({
                    "label": dev["label"],
                    "device": dev["device"],
                    "source": "webcam",
                    "preset": spec["preset"],
                    "sysfs_name": dev.get("sysfs_name", ""),
                })
            _LOG.info(
                f"auto_scan built {len(sources)} sources: "
                f"{[(d['label'], d['device'], d.get('sysfs_name', '')) for d in found]}"
            )
            return sources, layouts, infos
        _LOG.warning("auto_scan found no capture device; falling back to cameras list")

    cameras = params.get("cameras") or []
    if not cameras:
        raise RuntimeError(
            "No cameras configured. Enable auto_scan, set 'cameras' in params.yaml, "
            "or pass sources_json."
        )
    sources = []
    layouts = []
    infos = []
    for name in cameras:
        spec = _spec_from_camera_block(node, str(name))
        sources.append(_build_one_source(node, spec))
        layouts.append(spec["layout"])
        infos.append({
            "label": spec["label"],
            "device": str(_get_param(node, f"{name}.device", "")),
            "source": str(_get_param(node, f"{name}.source", "ros")),
            "preset": spec["preset"],
            "sysfs_name": "",
        })
    _LOG.info(f"built {len(sources)} sources from cameras={list(cameras)}")
    return sources, layouts, infos


def _scan_spec(
    node: Node, dev: dict[str, Any], index: int, count: int
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Spec for an auto-scanned device; a yaml block named after the label
    (e.g. ``video0.preset``) overrides individual fields."""
    label = dev["label"]
    preset = str(_get_param(node, f"{label}.preset", "720p30"))
    fov_h = float(_get_param(node, f"{label}.fov_h_deg", 60.0))
    force_mjpg = bool(_get_param(node, f"{label}.force_mjpg", True))
    pos = _get_param(node, f"{label}.layout.position", None)
    distance = float(_get_param(node, f"{label}.layout.distance", 1.8))
    size_mult = float(_get_param(node, f"{label}.layout.size_multiplier", 0.38))
    if pos is None:
        # Auto grid: spread panels in a single row in front of the eyes.
        x = (index - (count - 1) / 2.0) * 0.75
        pos = [x, 0.0, 1.8]
    layout = {
        "position": [float(p) for p in pos],
        "distance": distance,
        "size_multiplier": size_mult,
    }
    spec: dict[str, Any] = {
        "type": "webcam",
        "webcam_index": _device_to_index(dev["device"]),
        "preset": preset,
        "fov_h_deg": fov_h,
        "label": label,
        "force_mjpg": force_mjpg,
        "layout": layout,
    }
    return spec, layout


def _spec_from_camera_block(node: Node, name: str) -> dict[str, Any]:
    """Read a nested camera block (dotted params, e.g. 'd435i.preset')."""
    source = str(_get_param(node, f"{name}.source", "ros")).lower()
    device = _get_param(node, f"{name}.device", "/dev/video0")
    topic = str(_get_param(node, f"{name}.topic", "/camera/camera/color/image_raw"))
    preset = str(_get_param(node, f"{name}.preset", "720p30"))
    fov_h = float(_get_param(node, f"{name}.fov_h_deg", 69.0))
    label = str(_get_param(node, f"{name}.label", name))
    force_mjpg = bool(_get_param(node, f"{name}.force_mjpg", True))
    pos = _get_param(node, f"{name}.layout.position", [0.0, -0.1, 1.8])
    distance = float(_get_param(node, f"{name}.layout.distance", 1.8))
    size_mult = float(_get_param(node, f"{name}.layout.size_multiplier", 1.0))
    layout = {
        "position": [float(p) for p in pos],
        "distance": distance,
        "size_multiplier": size_mult,
    }
    spec: dict[str, Any] = {
        "preset": preset,
        "fov_h_deg": fov_h,
        "label": label,
        "layout": layout,
    }
    if source == "v4l2":
        # cv2 direct V4L2 read (low latency, no ROS/DDS).
        spec["type"] = "webcam"
        spec["webcam_index"] = _device_to_index(device)
        spec["force_mjpg"] = force_mjpg
    elif source == "webcam":
        spec["type"] = "webcam"
        spec["webcam_index"] = _device_to_index(device)
        spec["force_mjpg"] = force_mjpg
    else:  # ros
        spec["type"] = "ros"
        spec["topic"] = topic
    return spec


async def _run_telemetry_sink(host: str, port: int, verbose: bool) -> asyncio.AbstractServer:
    """Accept the Quest mocap TCP connection and drain/discard its lines.

    The VR app expects a listening TCP endpoint to enter the streaming phase.
    For a pure video-return host the mocap data is not needed, so we just
    drain it.
    """

    async def _handle(reader: asyncio.StreamReader, _writer: asyncio.StreamWriter) -> None:
        if verbose:
            _LOG.info(f"[telemetry-sink] client connected on {host}:{port}")
        try:
            while not reader.at_eof():
                await reader.readline()
        except (ConnectionError, asyncio.CancelledError):
            pass

    server = await asyncio.start_server(_handle, host, port)
    if verbose:
        _LOG.info(f"[telemetry-sink] listening on {host}:{port}")
    return server


def _get_param(node: Node, name: str, default: Any) -> Any:
    """Read a parameter, falling back to default if undeclared/missing."""
    try:
        val = node.get_parameter(name).value
        if val is None:
            return default
        return val
    except Exception:
        return default


def _safe_declare(node: Node, name: str, default: Any) -> None:
    """Declare a parameter, ignoring the case where an override already did."""
    try:
        node.declare_parameter(name, default)
    except rclpy.exceptions.ParameterAlreadyDeclaredException:
        pass


def _declare_params(node: Node) -> None:
    # Common params (declared with defaults; yaml/CLI overrides win and may
    # already have auto-declared them).
    _safe_declare(node, "sources_json", "")
    _safe_declare(node, "cameras", ["d435i"])
    _safe_declare(node, "signaling_host", "0.0.0.0")
    _safe_declare(node, "signaling_port", 8765)
    _safe_declare(node, "mocap_tcp_host", "0.0.0.0")
    _safe_declare(node, "mocap_tcp_port", 8000)
    _safe_declare(node, "enable_mocap_tcp", False)
    _safe_declare(node, "verbose", False)
    # Runtime push gate (see gate.py): master switch + initial active subset.
    _safe_declare(node, "push_enabled", True)
    _safe_declare(node, "active_cameras", "")  # comma labels; "" = all
    # Auto-scan host capture devices instead of the fixed `cameras` list.
    _safe_declare(node, "auto_scan", False)
    # Web preview (JPEG over CompressedImage on ~/preview/{label}).
    _safe_declare(node, "web_preview", True)
    _safe_declare(node, "web_preview_fps", 10.0)
    _safe_declare(node, "web_preview_width", 640)
    _safe_declare(node, "web_preview_quality", 65)
    # Legacy single-source params (kept for backward compatibility).
    _safe_declare(node, "source_type", "ros")
    _safe_declare(node, "image_topic", "/camera/camera/color/image_raw")
    _safe_declare(node, "webcam_index", 0)
    _safe_declare(node, "preset", "1080p30")
    _safe_declare(node, "fov_h_deg", 69.0)
    _safe_declare(node, "source_label", "d435i")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    rclpy.init()
    # Auto-declare nested per-camera params coming from the yaml overrides so
    # dotted names like 'd435i.preset' are readable without explicit declare.
    node = rclpy.create_node(
        "quest3_video_streamer",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    _declare_params(node)

    params = {name: _get_param(node, name, None) for name in [
        "sources_json", "cameras", "auto_scan",
        "signaling_host", "signaling_port", "mocap_tcp_host",
        "mocap_tcp_port", "enable_mocap_tcp", "verbose",
        "preset", "push_enabled", "active_cameras",
        "web_preview", "web_preview_fps", "web_preview_width",
        "web_preview_quality",
    ]}

    verbose = bool(params["verbose"])
    if verbose:
        logging.getLogger("quest3_video_streamer").setLevel(logging.DEBUG)

    # Spin rclpy in a daemon thread so image callbacks fire while the asyncio
    # WebRTC service runs in the main thread.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sources, layouts, cameras_info = _build_sources(node, params)
    gate = _setup_gate(node, sources, params, cameras_info)
    previews = _setup_previews(node, sources, gate, params)

    config = VideoServiceConfig(
        signaling_host=str(params["signaling_host"]),
        signaling_port=int(params["signaling_port"]),
        preset=str(params["preset"]),
        verbose=verbose,
        log_hook=lambda msg: _LOG.info(msg),
    )

    try:
        asyncio.run(_async_main(node, sources, layouts, config, params, verbose, gate))
    except KeyboardInterrupt:
        _LOG.info("interrupted")
    finally:
        for p in previews:
            p.stop()
        node.destroy_node()
        rclpy.shutdown()


def _setup_previews(
    node: Node,
    sources: list[VideoSourceAdapter],
    gate: StreamGate,
    params: dict[str, Any],
) -> list[Any]:
    """Attach a PreviewPublisher tap to each source (web live preview).

    No-op when ``web_preview`` is false. Frames are tapped from the producer
    threads (capture thread / rclpy callback) and JPEG-encoded in a dedicated
    thread per camera — never in the asyncio loop, so the Quest RTP pacing is
    unaffected.
    """
    if not bool(params.get("web_preview", True)):
        return []
    from quest3_video_streamer.preview import PreviewPublisher
    from quest3_video_streamer.ros_source import RosImageSourceAdapter

    fps = float(params.get("web_preview_fps", 10.0))
    width = int(params.get("web_preview_width", 640))
    quality = int(params.get("web_preview_quality", 65))
    previews: list[Any] = []
    for src in sources:
        label = str(src.get_format().label)
        pub = PreviewPublisher(
            node=node, label=label, gate=gate,
            fps=fps, width=width, quality=quality,
        )
        is_ros = isinstance(src, RosImageSourceAdapter)
        src.preview_hook = lambda f, p=pub, rgb=is_ros: p.submit(f, is_rgb=rgb)
        previews.append(pub)
    if previews:
        _LOG.info(
            f"web preview on ~/preview/<label>: {len(previews)} cams, "
            f"{fps}fps width={width} q{quality}"
        )
    return previews


_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _setup_gate(
    node: Node,
    sources: list[VideoSourceAdapter],
    params: dict[str, Any],
    cameras_info: list[dict[str, Any]] | None = None,
) -> StreamGate:
    """Create the runtime gate + its ROS control surface.

      * service ``~/set_push_enabled`` (std_srvs/SetBool): master switch.
      * subscription ``~/active_cameras`` (std_msgs/String, latched):
        comma-separated camera labels; empty = all configured cameras.
      * publisher ``~/gate_state`` (std_msgs/String, latched): JSON snapshot
        (gate state + per-camera device info), republished on every change so
        the web monitor can show live state.
    """
    labels = [str(src.get_format().label) for src in sources]
    initial_active = [
        x.strip() for x in str(params.get("active_cameras") or "").split(",")
        if x.strip()
    ]
    gate = StreamGate(
        all_labels=labels,
        push_enabled=bool(params.get("push_enabled", True)),
        active=initial_active,
    )
    state_pub = node.create_publisher(String, "~/gate_state", _LATCHED_QOS)

    def _publish_state() -> None:
        snap = gate.snapshot()
        snap["cameras"] = cameras_info or []
        msg = String()
        msg.data = json.dumps(snap, ensure_ascii=False)
        state_pub.publish(msg)

    def _on_set_push(req: SetBool.Request, resp: SetBool.Response) -> SetBool.Response:
        gate.set_push(req.data)
        resp.success = True
        resp.message = f"push_enabled={gate.snapshot()['push_enabled']}"
        _LOG.info(f"[gate] set_push_enabled -> {req.data}")
        _publish_state()
        return resp

    def _on_active_cameras(msg: String) -> None:
        labels_in = [x.strip() for x in msg.data.split(",") if x.strip()]
        effective = gate.set_active(labels_in)
        unknown = [x for x in labels_in if x not in labels]
        if unknown:
            _LOG.warning(
                f"[gate] active_cameras: unknown labels {unknown} ignored "
                f"(known: {labels}) — stale latched selection?"
            )
        _LOG.info(
            f"[gate] active_cameras <- {labels_in or 'ALL'} effective={effective}"
        )
        _publish_state()

    node.create_service(SetBool, "~/set_push_enabled", _on_set_push)
    node.create_subscription(String, "~/active_cameras", _on_active_cameras, _LATCHED_QOS)
    _publish_state()
    _LOG.info(
        f"[gate] push_enabled={gate.snapshot()['push_enabled']} "
        f"active={gate.snapshot()['active']} "
        f"(srv=~/set_push_enabled, topic=~/active_cameras, state=~/gate_state)"
    )
    return gate


async def _async_main(
    node: Node,
    sources: list[VideoSourceAdapter],
    layouts: list[dict[str, Any]],
    config: VideoServiceConfig,
    params: dict[str, Any],
    verbose: bool,
    gate: StreamGate | None = None,
) -> None:
    install_bitrate_diagnostics(verbose=verbose)

    sink_server: asyncio.AbstractServer | None = None
    if bool(params["enable_mocap_tcp"]):
        sink_server = await _run_telemetry_sink(
            str(params["mocap_tcp_host"]), int(params["mocap_tcp_port"]), verbose
        )

    service = Quest3VideoService(sources=sources, layouts=layouts, config=config, gate=gate)
    await service.start()
    _LOG.info(
        f"video service started host={config.signaling_host} port={config.signaling_port} "
        f"sources={len(sources)} preset={config.preset}"
    )
    _LOG.info(f"signaling endpoint (WebSocket): ws://<HOST_IP>:{config.signaling_port}")

    try:
        while rclpy.ok():
            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        pass
    finally:
        await service.stop()
        if sink_server is not None:
            sink_server.close()
            await sink_server.wait_closed()
        _LOG.info("shutdown complete")


if __name__ == "__main__":
    main()
