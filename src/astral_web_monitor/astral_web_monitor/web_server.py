"""FastAPI control plane + WebSocket telemetry + SPA static hosting.

Architecture follows rob_station's control_server:
  * REST  /api/v1/*  for control (start/stop/pause/resume/presets/health)
  * WS    /ws/telemetry  pushes a 30 Hz `ui_state` frame built from the ROS
    snapshot + launch state; broadcast is skipped when there are no clients
  * Static files served from web/dist (production) with SPA history fallback

The ROS node runs in a background thread (monitor_node.init_node); this
module runs uvicorn on the main thread.
"""
from __future__ import annotations

import asyncio
import json
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from . import config
from .launch_manager import (
    LaunchManager,
    PAUSED,
    RUNNING,
    load_presets,
)
from .monitor_node import get_node, init_node, shutdown_node
from .schemas import (
    ApiEnvelope,
    PresetInfo,
    StartRequest,
    VideoCamerasRequest,
    VideoPushRequest,
)


# --- globals ---------------------------------------------------------------
_launch_mgr = LaunchManager()
_presets = load_presets()
_web_dist = Path(os.environ.get("ASTRAL_WEB_MONITOR_DIST", ""))


def _build_ui_state() -> dict[str, Any]:
    """Aggregate one telemetry frame from ROS snapshot + launch state."""
    node = get_node()
    ros = node.snapshot() if node else {"joints": {}, "rates_hz": {}}
    return {
        "type": "ui_state",
        "ts": time.time(),
        "teleop": {
            "state": _launch_mgr.state,
            "preset": _launch_mgr.preset,
            "uptime_s": _launch_mgr.uptime_s(),
            "pid": _launch_mgr.pid,
        },
        "joints": ros.get("joints", {}),
        "rates_hz": ros.get("rates_hz", {}),
        "state_rates_hz": ros.get("state_rates_hz", {}),
        "health": ros.get("health", {"overall": "ok", "entities": {}}),
        "video_gate": ros.get("video_gate"),
        "log_tail": _launch_mgr.log_tail()[-50:],
    }


# --- connection manager ---------------------------------------------------
class _ConnectionManager:
    def __init__(self) -> None:
        self.active: list[WebSocket] = []

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self.active.append(ws)

    def disconnect(self, ws: WebSocket) -> None:
        if ws in self.active:
            self.active.remove(ws)

    @property
    def count(self) -> int:
        return len(self.active)

    async def broadcast(self, message: dict) -> None:
        text = json.dumps(message, ensure_ascii=False)
        dead: list[WebSocket] = []
        for ws in self.active:
            try:
                await ws.send_text(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)


_conn = _ConnectionManager()


# --- lifespan --------------------------------------------------------------
@asynccontextmanager
async def _lifespan(app: FastAPI):
    init_node()
    broadcaster = asyncio.create_task(_ui_state_broadcaster())
    try:
        yield
    finally:
        broadcaster.cancel()
        try:
            await broadcaster
        except asyncio.CancelledError:
            pass
        shutdown_node()


async def _ui_state_broadcaster() -> None:
    interval = 1.0 / config.WS_PUSH_HZ
    while True:
        await asyncio.sleep(interval)
        if _conn.count <= 0:
            continue
        msg = await asyncio.to_thread(_build_ui_state)
        await _conn.broadcast(msg)


# --- app -------------------------------------------------------------------
app = FastAPI(title="astral_web_monitor", version="0.1.0", lifespan=_lifespan)


@app.get("/api/v1/health")
async def health() -> ApiEnvelope:
    node = get_node()
    ros = node.snapshot() if node else {"health": {"overall": "down", "entities": {}}}
    return ApiEnvelope(
        ok=True,
        data={
            "ros_ok": node is not None,
            "launch_state": _launch_mgr.state,
            "launch_pid": _launch_mgr.pid,
            "uptime_s": _launch_mgr.uptime_s(),
            "ws_clients": _conn.count,
            "health": ros.get("health", {"overall": "down", "entities": {}}),
        },
    )


@app.get("/api/v1/presets", response_model=ApiEnvelope)
async def list_presets() -> ApiEnvelope:
    items = [
        PresetInfo(
            name=p.name, package=p.package, launch=p.launch,
            args=p.args, description=p.description,
        )
        for p in _presets.values()
    ]
    return ApiEnvelope(ok=True, data=[m.model_dump() for m in items])


@app.get("/api/v1/state")
async def get_state() -> ApiEnvelope:
    return ApiEnvelope(ok=True, data=_build_ui_state())


@app.post("/api/v1/start")
async def start(req: StartRequest) -> ApiEnvelope:
    preset = _presets.get(req.preset)
    if preset is None:
        raise HTTPException(status_code=404, detail=f"未知预设: {req.preset}")
    ok, msg = _launch_mgr.start(preset)
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=msg)


@app.post("/api/v1/stop")
async def stop() -> ApiEnvelope:
    ok, msg = _launch_mgr.stop()
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=msg)


@app.post("/api/v1/pause")
async def pause() -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    if _launch_mgr.state not in (RUNNING,):
        raise HTTPException(status_code=409, detail=f"当前状态 {_launch_mgr.state} 无法暂停")
    node.publish_disarm()
    _launch_mgr.mark_paused()
    return ApiEnvelope(ok=True, message="已暂停 (disarm)")


@app.post("/api/v1/resume")
async def resume() -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    if _launch_mgr.state != PAUSED:
        raise HTTPException(status_code=409, detail=f"当前状态 {_launch_mgr.state} 非暂停")
    node.publish_arm()
    _launch_mgr.mark_resumed()
    return ApiEnvelope(ok=True, message="已恢复 (arm)")


@app.post("/api/v1/teleop/start")
async def teleop_start() -> ApiEnvelope:
    """One-shot /teleop/start: arm nodes capture vr_init from current pose + arm.

    Use after the teleop nodes are running with require_start_signal:=true and
    the user has placed their hands at the desired initial pose. Re-sending
    re-captures the zero (re-center). The arm node is the authority: it ignores
    the trigger while homing or before any VR pose arrives (logs a warning), so
    this endpoint publishes unconditionally — it works whether the teleop launch
    was started via this monitor or from a separate CLI.
    """
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    node.publish_start()
    return ApiEnvelope(
        ok=True,
        message="已发送 /teleop/start（臂节点记 vr_init 并 arm；homing 中或无 VR 时会忽略并告警）",
    )


@app.post("/api/v1/restart")
async def restart() -> ApiEnvelope:
    """Restart the current preset: stop then start the same preset.

    Only valid for a launch started via this monitor's preset manager. If the
    teleop was started from a separate CLI, use stop+start manually instead.
    """
    name = _launch_mgr.preset
    if not name or name not in _presets:
        raise HTTPException(status_code=409, detail="无当前预设可重启（CLI 启动的遥操不支持重启）")
    ok_stop, _ = _launch_mgr.stop()
    # Wait briefly for the stop to take effect before restarting.
    await asyncio.sleep(1.0)
    ok_start, msg = _launch_mgr.start(_presets[name])
    if not ok_start:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=f"已重启预设: {name}")


# --- Robot hardware mode (driver services) ----------------------------------
# These call the driver's already-exposed Trigger services. The driver is the
# hardware authority; the monitor only invokes its services (non-intrusive).
# All run in a worker thread so the ROS spin thread can complete the future.

def _driver_call(name: str) -> tuple[bool, str]:
    node = get_node()
    if node is None:
        return False, "ROS 节点未就绪"
    return node.call_driver_service(name)


@app.post("/api/v1/robot/ready")
async def robot_ready() -> ApiEnvelope:
    ok, msg = await asyncio.to_thread(_driver_call, "ready")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "一键就绪 OK")


@app.post("/api/v1/robot/home")
async def robot_home() -> ApiEnvelope:
    ok, msg = await asyncio.to_thread(_driver_call, "home")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "全关节归零")


@app.post("/api/v1/robot/estop")
async def robot_estop() -> ApiEnvelope:
    """真急停：调 driver ~/estop → SDK disable()（断电）。"""
    ok, msg = await asyncio.to_thread(_driver_call, "estop")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "已断电 (e_stop)")


@app.post("/api/v1/robot/damping")
async def robot_damping() -> ApiEnvelope:
    """阻尼释放：调 driver ~/damping → motion_mode=0，可手动拖拽。"""
    ok, msg = await asyncio.to_thread(_driver_call, "damping")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "阻尼释放 (可手动拖拽)")


@app.post("/api/v1/robot/position")
async def robot_position() -> ApiEnvelope:
    """位置保持：调 driver ~/position → motion_mode=1，恢复位置保持。"""
    ok, msg = await asyncio.to_thread(_driver_call, "position")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "位置保持")


# --- Video return gate (quest3_video_streamer) ------------------------------
# Non-intrusive: the streamer owns the cameras; the monitor only reads its
# config/params.yaml (to list configured cameras), calls its SetBool master
# switch, and publishes the latched active-cameras subset.

def _sysfs_video_name(device: str) -> str:
    """Human-readable camera name for /dev/videoN from sysfs ('' if unknown)."""
    if not device.startswith("/dev/video"):
        return ""
    try:
        with open(f"/sys/class/video4linux/{device[5:]}/name", "r", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return ""


def _video_configured() -> list[dict[str, Any]]:
    """Configured camera blocks from quest3_video_streamer's params.yaml."""
    path = config.video_params_path()
    if not path or not os.path.isfile(path):
        return []
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return []
    params = data.get("quest3_video_streamer", {}).get("ros__parameters", data)
    names = params.get("cameras") or []
    out: list[dict[str, Any]] = []
    for name in names:
        block = params.get(str(name), {})
        if not isinstance(block, dict):
            block = {}
        device = str(block.get("device", ""))
        source = str(block.get("source", ""))
        exists = os.path.exists(device) if device.startswith("/dev/") else False
        out.append({
            "label": str(block.get("label", name)),
            "device": device,
            "source": source,
            "preset": str(block.get("preset", "")),
            "exists": exists,
            "sysfs_name": _sysfs_video_name(device) if exists else "",
        })
    return out


@app.get("/api/v1/video/status")
async def video_status() -> ApiEnvelope:
    node = get_node()
    gate = None
    if node is not None:
        gate = node.snapshot().get("video_gate")
    return ApiEnvelope(ok=True, data={
        "configured": _video_configured(),
        "gate": gate,  # {push_enabled, configured, active} or None when streamer offline
        "online": gate is not None,
    })


@app.post("/api/v1/video/push")
async def video_push(req: VideoPushRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    ok, msg = await asyncio.to_thread(node.set_video_push, req.enabled)
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or ("视频推送已开启" if req.enabled else "视频推送已关闭"))


@app.post("/api/v1/video/cameras")
async def video_cameras(req: VideoCamerasRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    node.publish_video_cameras(req.cameras)
    label = ", ".join(req.cameras) if req.cameras else "全部已配置相机"
    return ApiEnvelope(ok=True, message=f"已下发推送相机: {label}")


# --- WebSocket -------------------------------------------------------------
@app.websocket("/ws/telemetry")
async def ws_telemetry(websocket: WebSocket):
    await _conn.connect(websocket)
    # Send an immediate snapshot so the client doesn't wait for the next tick.
    snapshot = await asyncio.to_thread(_build_ui_state)
    await websocket.send_text(json.dumps(snapshot, ensure_ascii=False))
    try:
        while True:
            raw = await websocket.receive_text()
            if raw == "ping":
                await websocket.send_text(json.dumps({"type": "pong", "ts": time.time()}))
    except WebSocketDisconnect:
        _conn.disconnect(websocket)
    except Exception:
        _conn.disconnect(websocket)


# --- SPA static hosting ----------------------------------------------------
if _web_dist.is_dir():
    assets = _web_dist / "assets"
    if assets.is_dir():
        app.mount("/assets", StaticFiles(directory=str(assets)), name="assets")

    @app.get("/{full_path:path}")
    async def spa_fallback(full_path: str):
        # API and WS routes are matched first; anything else serves the SPA.
        if full_path.startswith(("api/", "ws/")):
            raise HTTPException(status_code=404)
        index = _web_dist / "index.html"
        if index.is_file():
            return FileResponse(str(index))
        raise HTTPException(status_code=404, detail="前端未构建")


def main() -> None:
    """Entry point (console_scripts). Runs uvicorn on the main thread."""
    import uvicorn
    uvicorn.run(
        app,
        host=config.WEB_HOST,
        port=config.WEB_PORT,
        log_level=os.environ.get("ASTRAL_WEB_MONITOR_LOG_LEVEL", "info"),
    )


if __name__ == "__main__":
    main()
