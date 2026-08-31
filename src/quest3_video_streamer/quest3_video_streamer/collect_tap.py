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

import logging
import queue
import threading
import time
from typing import Any

_LOG = logging.getLogger("quest3_video_streamer.collect_tap")


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
        # 边界插桩：帧在哪一段丢的（准入/限流/队列/编码/发布），5s 窗口 INFO。
        # 实测事故定位用：driver 侧 30fps 而 collect 话题 3fps 时，看这里。
        self._n_submitted = 0
        self._n_rate_skip = 0
        self._n_queue_full = 0
        self._n_encoded = 0
        self._n_published = 0
        self._encode_ms_sum = 0.0
        self._publish_ms_sum = 0.0
        self._stat_t0 = time.monotonic()
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
        self._n_submitted += 1
        now = time.monotonic()
        if now - self._last_submit < self._period:
            self._n_rate_skip += 1
            return
        self._last_submit = now
        try:
            self._queue.put_nowait((frame, is_rgb))
        except queue.Full:
            self._n_queue_full += 1

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self._node.destroy_publisher(self._pub)
        except Exception:
            pass

    def _log_stats(self) -> None:
        """5s 窗口边界计数——帧到底丢在准入/限流/队列/编码/发布哪一段。"""
        now = time.monotonic()
        dt = now - self._stat_t0
        if dt < 5.0:
            return
        enc = self._encode_ms_sum / self._n_encoded if self._n_encoded else 0.0
        pub = self._publish_ms_sum / self._n_published if self._n_published else 0.0
        _LOG.info(
            "[tap %s] submit=%.1f/s rate_skip=%d queue_full=%d "
            "encoded=%.1f/s published=%.1f/s encode=%.1fms publish=%.1fms",
            self._label,
            self._n_submitted / dt,
            self._n_rate_skip,
            self._n_queue_full,
            self._n_encoded / dt,
            self._n_published / dt,
            enc,
            pub,
        )
        self._n_submitted = self._n_rate_skip = self._n_queue_full = 0
        self._n_encoded = self._n_published = 0
        self._encode_ms_sum = self._publish_ms_sum = 0.0
        self._stat_t0 = now

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
                t0 = time.monotonic()
                ok, jpg = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality]
                )
                if not ok:
                    continue
                self._n_encoded += 1
                self._encode_ms_sum += (time.monotonic() - t0) * 1000.0
                msg = self._msg_type()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.header.frame_id = self._label
                msg.format = "jpeg"
                msg.data = jpg.tobytes()
                t0 = time.monotonic()
                self._pub.publish(msg)
                self._n_published += 1
                self._publish_ms_sum += (time.monotonic() - t0) * 1000.0
            except Exception:
                continue
            self._log_stats()
