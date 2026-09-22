"""Structured capture/tap diagnostics remain cheap and machine-readable."""

from __future__ import annotations

import time
import queue

from quest3_video_streamer.collect_tap import CollectTapPublisher


def test_collect_tap_stamps_at_admission_before_encoding() -> None:
    stamp = object()

    class Pub:
        @staticmethod
        def get_subscription_count():
            return 1

    class Now:
        @staticmethod
        def to_msg():
            return stamp

    class Clock:
        @staticmethod
        def now():
            return Now()

    class Node:
        @staticmethod
        def get_clock():
            return Clock()

    tap = object.__new__(CollectTapPublisher)
    tap._pub = Pub()
    tap._node = Node()
    tap._queue = queue.Queue(maxsize=1)
    tap._max_fps = 30.0
    tap._tokens = 2.0
    tap._last_refill = time.monotonic()
    tap._n_submitted = tap._n_rate_skip = tap._n_queue_full = 0
    tap._last_submit_t = None
    tap._max_submit_gap_ms = 0.0

    frame = object()
    tap.submit(frame, is_rgb=False)

    queued_frame, is_rgb, queued_stamp = tap._queue.get_nowait()
    assert queued_frame is frame
    assert is_rgb is False
    assert queued_stamp is stamp


def test_collect_tap_emits_window_diagnostics_and_resets_counters() -> None:
    records: list[dict] = []
    tap = object.__new__(CollectTapPublisher)
    tap._label = "left_wrist"
    tap._diagnostic_hook = records.append
    tap._n_submitted = 145
    tap._n_rate_skip = 2
    tap._n_queue_full = 3
    tap._n_encoded = 140
    tap._n_published = 139
    tap._encode_ms_sum = 280.0
    tap._publish_ms_sum = 69.5
    tap._max_submit_gap_ms = 1100.0
    tap._max_publish_gap_ms = 1200.0
    tap._stat_t0 = time.monotonic() - 5.1

    tap._log_stats()

    assert len(records) == 1
    rec = records[0]
    assert rec["stage"] == "collect_tap"
    assert rec["label"] == "left_wrist"
    assert rec["rate_skip"] == 2
    assert rec["queue_full"] == 3
    assert rec["encoded_fps"] > 0
    assert rec["published_fps"] > 0
    assert rec["max_submit_gap_ms"] == 1100.0
    assert rec["max_publish_gap_ms"] == 1200.0
    assert rec["encode_ms"] == 2.0
    assert rec["publish_ms"] == 0.5
    assert tap._n_submitted == 0
    assert tap._n_published == 0
