"""Direct OpenCV (V4L2) USB camera source adapter.

Used as a fallback / alternative when no ROS camera driver is desired for a USB
webcam (e.g. cheap MJPG-only USB cameras). Sets FOURCC=MJPG so high-res modes
(720p/1080p) are actually selected.

Capture runs in a background daemon thread so the blocking ``cv2.read()`` never
stalls the asyncio WebRTC event loop. ``next_frame()`` returns the freshest
frame non-blocking. This is important when multiple sources share one asyncio
loop: a blocking webcam read would starve the other (e.g. D435i) tracks.
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from typing import Any

from quest3_video_streamer.source_base import VideoFormat, VideoSourceAdapter

_LOG = logging.getLogger("quest3_video_streamer.capture")


class WebcamSourceAdapter(VideoSourceAdapter):
    def __init__(
        self,
        *,
        device_index: int = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        fov_h_deg: float = 60.0,
        label: str = "webcam",
        force_mjpg: bool = True,
    ) -> None:
        self._format = VideoFormat(
            width=width, height=height, fps=fps,
            fov_h_deg=fov_h_deg, label=label,
        )
        self._device_index = device_index
        self._force_mjpg = force_mjpg
        self._capture: Any = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest_rgb: Any = None
        self._frame_ready = asyncio.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        # Optional tap for the web preview pipeline: callable(bgr_frame).
        # Called from the capture thread; must be non-blocking.
        self.preview_hook: Any = None
        # Optional tap for the data-collection pipeline: callable(bgr_frame).
        # Same contract as preview_hook (capture thread, must not block).
        self.collect_hook: Any = None

    async def start(self) -> None:
        import cv2

        if self._thread is not None and self._thread.is_alive():
            return  # 幂等：eager start 后 track 的 lazy open 不再重复打开
        mjpg = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        capture = cv2.VideoCapture(self._device_index, cv2.CAP_V4L2)
        if self._force_mjpg:
            capture.set(cv2.CAP_PROP_FOURCC, mjpg)
        if not capture.isOpened():
            capture = cv2.VideoCapture(self._device_index)
            if self._force_mjpg:
                capture.set(cv2.CAP_PROP_FOURCC, mjpg)
        if not capture.isOpened():
            raise RuntimeError(f"Failed to open webcam index {self._device_index}.")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._format.width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._format.height))
        capture.set(cv2.CAP_PROP_FPS, float(self._format.fps))
        self._capture = capture
        self._loop = asyncio.get_running_loop()
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    async def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def get_format(self) -> VideoFormat:
        return self._format

    async def next_frame(self) -> Any:
        import av

        # Pace the consumer to the capture rate: block until the capture thread
        # has produced a NEW frame since the last consume. Without this, the
        # aiortc RTP send loop would spin at full speed re-encoding the same
        # latest frame, hogging the asyncio event loop and starving other
        # tracks (e.g. the D435i ROS source whose queue.get() would be delayed).
        if self._latest_rgb is None:
            # First frame: just wait for any frame to appear.
            while self._latest_rgb is None and not self._stop.is_set():
                await asyncio.sleep(0.005)
            if self._latest_rgb is None:
                return None
            self._frame_ready.clear()
        else:
            # Wait for a fresh frame (event set by the capture thread).
            await self._frame_ready.wait()
            if self._stop.is_set():
                return None
        self._frame_ready.clear()
        with self._lock:
            rgb = self._latest_rgb
        if rgb is None:
            return None
        return av.VideoFrame.from_ndarray(rgb, format="rgb24")

    # -- background capture thread (blocking cv2.read lives here) ------------

    def _capture_loop(self) -> None:
        import cv2

        # 驱动侧实率插桩：5s 窗口 INFO 日志。定位"采集帧率低"时分界用——
        # 这里低 = 捕获线程慢（驱动/CPU）；这里满 30 而 tap 低 = 抽头/发布段慢。
        frames = 0
        window_t0 = time.monotonic()
        while not self._stop.is_set():
            if self._capture is None:
                break
            ok, bgr = self._capture.read()
            if not ok or bgr is None:
                # Brief retry on transient read failure.
                continue
            frames += 1
            now = time.monotonic()
            if now - window_t0 >= 5.0:
                _LOG.info(
                    "[capture %s] driver-side %.1f fps",
                    self._format.label, frames / (now - window_t0),
                )
                frames = 0
                window_t0 = now
            hook = self.preview_hook
            if hook is not None:
                try:
                    hook(bgr)  # native BGR, before the RGB conversion
                except Exception:
                    pass
            collect_hook = self.collect_hook
            if collect_hook is not None:
                try:
                    collect_hook(bgr)  # native BGR, same contract as preview
                except Exception:
                    pass
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            with self._lock:
                self._latest_rgb = rgb
            # Notify the asyncio loop a frame is ready.
            loop = self._loop
            if loop is not None:
                try:
                    loop.call_soon_threadsafe(self._set_frame_ready)
                except Exception:
                    pass

    def _set_frame_ready(self) -> None:
        if not self._frame_ready.is_set():
            self._frame_ready.set()
