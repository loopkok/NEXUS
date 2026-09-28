"""Convert Orbbec SDK color frames to OpenCV BGR without reference demos."""

import cv2
import numpy as np
from pyorbbecsdk import OBFormat


def frame_to_bgr_image(frame):
    width, height = frame.get_width(), frame.get_height()
    fmt = frame.get_format()
    raw = np.asanyarray(frame.get_data())
    if fmt == OBFormat.MJPG:
        return cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if fmt == OBFormat.RGB:
        return cv2.cvtColor(raw.reshape(height, width, 3), cv2.COLOR_RGB2BGR)
    if fmt == OBFormat.BGR:
        return raw.reshape(height, width, 3).copy()
    if fmt == OBFormat.YUYV:
        return cv2.cvtColor(raw.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUY2)
    if fmt == OBFormat.UYVY:
        return cv2.cvtColor(raw.reshape(height, width, 2), cv2.COLOR_YUV2BGR_UYVY)
    if fmt == OBFormat.I420:
        return cv2.cvtColor(raw.reshape(height * 3 // 2, width), cv2.COLOR_YUV2BGR_I420)
    if fmt == OBFormat.NV12:
        return cv2.cvtColor(raw.reshape(height * 3 // 2, width), cv2.COLOR_YUV2BGR_NV12)
    if fmt == OBFormat.NV21:
        return cv2.cvtColor(raw.reshape(height * 3 // 2, width), cv2.COLOR_YUV2BGR_NV21)
    raise ValueError(f"unsupported Orbbec color format: {fmt}")
