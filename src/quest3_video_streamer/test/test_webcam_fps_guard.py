"""USB-speed-dependent UVC FPS negotiation safety tests."""

import asyncio

import cv2
import pytest

from quest3_video_streamer.webcam_source import (
    WebcamSourceAdapter,
    _fourcc_text,
    _fps_mismatch,
)


def test_usb3_120fps_is_not_accepted_as_30fps():
    assert _fps_mismatch(30.0, 120.0, 0.15)


def test_normal_camera_rounding_is_accepted():
    assert not _fps_mismatch(30.0, 29.97, 0.15)
    assert not _fps_mismatch(30.0, 27.9, 0.15)


def test_unknown_backend_fps_fails_open_for_runtime_diagnostics():
    assert not _fps_mismatch(30.0, 0.0, 0.15)


def test_fourcc_is_human_readable():
    packed = sum(ord(ch) << (8 * i) for i, ch in enumerate("MJPG"))
    assert _fourcc_text(float(packed)) == "MJPG"


def test_strict_start_releases_camera_and_rejects_120fps(monkeypatch):
    packed = float(sum(ord(ch) << (8 * i) for i, ch in enumerate("MJPG")))

    class FakeCapture:
        def __init__(self, *_args):
            self.released = False

        def isOpened(self):
            return True

        def set(self, _prop, _value):
            return True

        def get(self, prop):
            return {
                cv2.CAP_PROP_FRAME_WIDTH: 1280.0,
                cv2.CAP_PROP_FRAME_HEIGHT: 720.0,
                cv2.CAP_PROP_FPS: 120.0,
                cv2.CAP_PROP_FOURCC: packed,
            }.get(prop, 0.0)

        def release(self):
            self.released = True

    fake = FakeCapture()
    monkeypatch.setattr(cv2, "VideoCapture", lambda *_args: fake)
    source = WebcamSourceAdapter(
        device_index=8, width=1280, height=720, fps=30,
        label="right_wrist", strict_capture_fps=True,
    )
    with pytest.raises(RuntimeError, match="negotiated.*120"):
        asyncio.run(source.start())
    assert fake.released
    assert source._thread is None
