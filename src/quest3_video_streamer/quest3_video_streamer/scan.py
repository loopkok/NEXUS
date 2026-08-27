"""Host video-device auto-scan (no hardcoded device nodes).

Enumerates ``/dev/video*`` and keeps one **capture-capable** node per physical
device:

  * nodes without ``V4L2_CAP_VIDEO_CAPTURE`` (e.g. metadata nodes that USB
    cameras expose alongside the video node) are skipped;
  * nodes belonging to the same physical device (sysfs parent) are de-duped,
    keeping the lowest-numbered node.

Label convention: ``video{N}`` (the device node basename) — stable as long as
the host's device numbering is stable, and directly tells you which node it is.

Known limitation: multi-interface devices like the RealSense D435i expose
several capture-capable nodes (depth/IR/color); the first one is not
necessarily color. For D435i prefer an explicit yaml block (``auto_scan:
false`` or a matching label override) — see config/params.yaml.
"""

from __future__ import annotations

import fcntl
import glob
import os
import struct
from typing import Any

# VIDIOC_QUERYCAP = _IOR('V', 0, struct v4l2_capability[104])
_VIDIOC_QUERYCAP = 0x80685600
_V4L2_CAP_VIDEO_CAPTURE = 0x00000001
_V4L2_CAP_DEVICE_CAPS = 0x80000000


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


def enumerate_capture_devices() -> list[dict[str, Any]]:
    """One entry per physical capture device, ordered by device node number."""
    out: list[dict[str, Any]] = []
    seen_groups: set[str] = set()
    for path in sorted(glob.glob("/dev/video*"), key=lambda p: int(p[len("/dev/video"):])):
        node = os.path.basename(path)  # "video0"
        if not _is_capture_node(path):
            continue
        group = _physical_group(node)
        if group in seen_groups:
            continue
        seen_groups.add(group)
        out.append({
            "label": node,  # "video0"
            "device": path,  # "/dev/video0"
            "sysfs_name": sysfs_name(node),
        })
    return out
