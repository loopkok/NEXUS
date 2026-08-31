"""Push-throttle (回传降载) 的纯逻辑测试：scaled_size 与 track fps 上限。

不触相机/网络：_AdapterVideoTrack 构造只调 source.get_format()。
"""

from __future__ import annotations

from quest3_video_streamer.source_base import VideoFormat
from quest3_video_streamer.webrtc_sender import _AdapterVideoTrack, scaled_size


def _fmt(w: int, h: int, fps: int) -> VideoFormat:
    return VideoFormat(width=w, height=h, fps=fps, fov_h_deg=60.0, label="t")


class _Src:
    def __init__(self, fmt: VideoFormat) -> None:
        self._fmt = fmt

    def get_format(self) -> VideoFormat:
        return self._fmt


def test_scaled_size_disabled_when_max_width_zero():
    assert scaled_size(1920, 1080, 0) == (1920, 1080)


def test_scaled_size_no_upscale():
    assert scaled_size(1280, 720, 1920) == (1280, 720)


def test_scaled_size_keeps_aspect_and_even():
    w, h = scaled_size(1920, 1080, 960)
    assert (w, h) == (960, 540)
    assert w % 2 == 0 and h % 2 == 0


def test_scaled_size_odd_max_width_rounds_even():
    w, h = scaled_size(1920, 1080, 961)
    assert w % 2 == 0 and h % 2 == 0
    assert w <= 960


def test_track_fps_capped_by_push_fps():
    t = _AdapterVideoTrack(_Src(_fmt(1920, 1080, 30)), fps=30, push_fps=15)
    assert t._fps == 15
    assert t._time_base.numerator == 1 and t._time_base.denominator == 15


def test_track_fps_untouched_when_push_fps_zero_or_higher():
    assert _AdapterVideoTrack(_Src(_fmt(1280, 720, 30)), fps=30, push_fps=0)._fps == 30
    assert _AdapterVideoTrack(_Src(_fmt(1280, 720, 30)), fps=30, push_fps=60)._fps == 30


def test_track_push_size_defaults_to_source_when_disabled():
    t = _AdapterVideoTrack(_Src(_fmt(1920, 1080, 30)), fps=30, push_max_width=0)
    assert (t._push_w, t._push_h) == (1920, 1080)


def test_track_push_size_scaled():
    t = _AdapterVideoTrack(_Src(_fmt(1920, 1080, 30)), fps=30, push_max_width=960)
    assert (t._push_w, t._push_h) == (960, 540)
