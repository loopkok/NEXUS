"""Host video-device auto-scan (no hardcoded device nodes).

Enumerates ``/dev/video*`` and keeps one **capture-capable** node per physical
device:

  * nodes without ``V4L2_CAP_VIDEO_CAPTURE`` (e.g. metadata nodes that USB
    cameras expose alongside the video node) are skipped;
  * nodes belonging to the same physical device (sysfs parent) are de-duped.

When a device exposes several capture nodes (RealSense D435i: depth / IR /
color), pick the **color-like** format (YUYV / MJPG / RGB) rather than the
lowest-numbered node. Lowest-number-first would take D435i ``/dev/video4``
(Z16 depth), which OpenCV cannot open as a webcam — color is ``/dev/video8``.

Label convention: ``video{N}`` (the device node basename).
"""

from __future__ import annotations

import fcntl
import glob
import os
import struct
import subprocess
from typing import Any

# VIDIOC_QUERYCAP = _IOR('V', 0, struct v4l2_capability[104])
_VIDIOC_QUERYCAP = 0x80685600
_V4L2_CAP_VIDEO_CAPTURE = 0x00000001
_V4L2_CAP_DEVICE_CAPS = 0x80000000

_COLOR_FOURCC = frozenset({"YUYV", "MJPG", "JPEG", "RGB3", "BGR3", "RGBP"})
_IR_FOURCC = frozenset({"GREY", "Y8  ", "Y8", "Y16 "})
_DEPTH_FOURCC = frozenset({"Z16 ", "Z16"})


def _is_capture_node(path: str) -> bool:
    """True when the v4l2 node supports video capture."""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        buf = bytearray(104)
        fcntl.ioctl(fd, _VIDIOC_QUERYCAP, buf, True)
        (capabilities,) = struct.unpack_from("<I", buf, 84)
        (device_caps,) = struct.unpack_from("<I", buf, 88)
        caps = device_caps if (capabilities & _V4L2_CAP_DEVICE_CAPS) else capabilities
        return bool(caps & _V4L2_CAP_VIDEO_CAPTURE)
    except OSError:
        return False
    finally:
        os.close(fd)


def _physical_group(node: str) -> str:
    """Group key = the physical device owning this video node (sysfs parent)."""
    real = os.path.realpath(f"/sys/class/video4linux/{node}/device")
    suffix = f"/video4linux/{node}"
    if real.endswith(suffix):
        real = real[: -len(suffix)]
    return os.path.dirname(real)


def sysfs_name(node: str) -> str:
    """Human-readable camera name ('' when unknown)."""
    try:
        with open(f"/sys/class/video4linux/{node}/name", "r", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return ""


def _node_index(node: str) -> int:
    try:
        return int(node[len("video"):])
    except ValueError:
        return 0


def _current_fourcc(path: str) -> str:
    """Current Pixel Format fourcc via v4l2-ctl, e.g. 'YUYV' / 'Z16' / 'MJPG'."""
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "-d", path, "--get-fmt-video"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=1.5,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    for line in out.splitlines():
        if "Pixel Format" not in line:
            continue
        if "'" in line:
            return line.split("'")[1].strip()
        return ""
    return ""


def _fourcc_score(fourcc: str) -> int:
    """Higher = better for RGB webcam streaming."""
    key = fourcc.strip()
    if key in _COLOR_FOURCC or fourcc in _COLOR_FOURCC:
        return 100
    if key in _IR_FOURCC or fourcc in _IR_FOURCC:
        return 10
    if key in _DEPTH_FOURCC or fourcc in _DEPTH_FOURCC:
        return 0
    return 1


def enumerate_capture_devices() -> list[dict[str, Any]]:
    """One entry per physical capture device, ordered by device node number."""
    by_group: dict[str, list[dict[str, Any]]] = {}
    for path in sorted(
        glob.glob("/dev/video*"), key=lambda p: int(p[len("/dev/video"):])
    ):
        node = os.path.basename(path)
        if not _is_capture_node(path):
            continue
        fourcc = _current_fourcc(path)
        rec = {
            "label": node,
            "device": path,
            "sysfs_name": sysfs_name(node),
            "fourcc": fourcc,
            "score": _fourcc_score(fourcc),
        }
        by_group.setdefault(_physical_group(node), []).append(rec)

    out: list[dict[str, Any]] = []
    for recs in by_group.values():
        recs.sort(key=lambda r: (-int(r["score"]), _node_index(str(r["label"]))))
        best = recs[0]
        fourcc = str(best.get("fourcc", ""))
        out.append({
            "label": best["label"],
            "device": best["device"],
            "sysfs_name": best["sysfs_name"],
            # D435i color is YUYV; forcing MJPG makes cv2 fail to open.
            "force_mjpg": fourcc.strip() in ("MJPG", "JPEG"),
        })
    out.sort(key=lambda d: _node_index(str(d["label"])))
    return out
