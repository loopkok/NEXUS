"""Full-quality frame tap for offline data collection (JPEG over CompressedImage).

Companion to ``preview.py``: same thread model, but full resolution at (near)
native frame rate, for consumption by ``astral_data_collect`` recording nodes.

Key differences from the web preview:

  * No downscale and high JPEG quality (default q90) — pixels are training data.
  * Encode-on-demand: frames are only queued when the topic has at least one
    subscriber, so a teleop-only run pays exactly zero encode cost.
  * Rate limit is a ceiling, not a throttle target: default 30 fps matches the
    dataset grid; set higher/lower per deployment.

Latency isolation (same contract as preview):

  * ``submit()`` runs in the producer thread (capture thread / rclpy
    callback). It only does gate + subscriber + rate checks and a
    ``put_nowait`` on a maxsize-1 queue — never blocks, drops when busy.
  * JPEG encoding runs in a dedicated daemon thread, NEVER in the capture
    thread and NEVER in the asyncio loop.
  * Publishing follows the runtime gate: a muted camera publishes nothing.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Any


class CollectTapPublisher:
    """Per-camera full-res JPEG publisher on ``~/collect/{label}``."""

    def __init__(
        self,
        *,
        node: Any,
        label: str,
        gate: Any = None,
        max_fps: float = 30.0,
        quality: int = 90,
    ) -> None:
        from sensor_msgs.msg import CompressedImage
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self._node = node
        self._label = label
        self._gate = gate
        self._period = 1.0 / max(0.5, max_fps)
        self._quality = int(quality)
        self._last_submit = 0.0
        self._queue: queue.Queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._msg_type = CompressedImage
        self._pub = node.create_publisher(
            CompressedImage, f"~/collect/{label}", qos
        )
        self._thread = threading.Thread(
            target=self._run, name=f"collect-tap-{label}", daemon=True
        )
        self._thread.start()

    def submit(self, frame: Any, *, is_rgb: bool) -> None:
        """Offer a frame (numpy HxWx3). Non-blocking; drops when busy."""
        if self._gate is not None and not self._gate.is_enabled(self._label):
            return
        if self._pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now - self._last_submit < self._period:
            return
        self._last_submit = now
        try:
            self._queue.put_nowait((frame, is_rgb))
        except queue.Full:
            pass

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self._node.destroy_publisher(self._pub)
        except Exception:
            pass

    # -- encoder thread ------------------------------------------------------

    def _run(self) -> None:
        import cv2

        while not self._stop.is_set():
            try:
                frame, is_rgb = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if is_rgb:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                ok, jpg = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality]
                )
                if not ok:
                    continue
                msg = self._msg_type()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.header.frame_id = self._label
                msg.format = "jpeg"
                msg.data = jpg.tobytes()
                self._pub.publish(msg)
            except Exception:
                continue
