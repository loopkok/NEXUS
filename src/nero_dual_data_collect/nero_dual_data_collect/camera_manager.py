"""Multi-camera manager using multiprocessing for Gemini 335/305 cameras.

Orbbec SDK v2 API: Pipeline(device) per-device, Context for enumeration.
Cameras run in a separate process to avoid GIL contention with the ROS2 node.
"""

import os
import time
import threading
from typing import List, Dict, Optional

import cv2
import numpy as np
import h5py

try:
    from pyorbbecsdk import (
        Pipeline, Config, Context, AlignFilter,
        OBSensorType, OBStreamType, OBFormat,
    )
    HAS_ORBBEC = True
except ImportError:
    HAS_ORBBEC = False
    print("[CameraManager] WARNING: pyorbbecsdk not available. Camera capture disabled.")

try:
    from .orbbec_utils import frame_to_bgr_image
except ImportError:
    frame_to_bgr_image = None

CHUNK_SIZE = 1000


def jpeg_compress(img: np.ndarray, quality: int = 90) -> bytes:
    """Compress image to JPEG bytes."""
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    result, encimg = cv2.imencode(".jpg", img, encode_param)
    if not result:
        raise ValueError("JPEG compression failed")
    return encimg.tobytes()


def _get_device_list():
    """Enumerate all connected Orbbec devices. Must be called in the camera process."""
    ctx = Context()
    dev_list = ctx.query_devices()
    n = dev_list.get_count()
    devices = []
    for i in range(n):
        devices.append({
            "index": i,
            "serial": dev_list.get_device_serial_number_by_index(i),
            "name": dev_list.get_device_name_by_index(i),
            "pid": dev_list.get_device_pid_by_index(i),
            "uid": dev_list.get_device_uid_by_index(i),
        })
    return devices


def _find_device_for_config(devices: list, cfg: dict) -> Optional[dict]:
    """Match a camera config to a physical device.

    Priority: 1) cfg['serial'] exact match  2) cfg['id'] index match  3) first unused
    """
    # Exact serial match
    if "serial" in cfg and cfg["serial"]:
        for d in devices:
            if d["serial"] == cfg["serial"]:
                return d

    # Index-based: cam_0 → devices sorted by serial
    sorted_devs = sorted(devices, key=lambda x: x["serial"])
    idx = int(cfg.get("id", "cam_0").split("_")[-1])
    if 0 <= idx < len(sorted_devs):
        return sorted_devs[idx]

    return None


class GeminiCamera:
    """Wrapper for a single Gemini 335/305 camera using Orbbec SDK v2."""

    def __init__(self, device_info: dict, width: int = 640, height: int = 480, fps: int = 30):
        self.width = width
        self.height = height
        self.fps = fps
        self.device_info = device_info
        self.pipeline: Optional[Pipeline] = None
        self.config: Optional[Config] = None
        self.align_filter = None

    def initialize(self) -> bool:
        """Initialize the camera pipeline. Returns True on success."""
        if not HAS_ORBBEC:
            return False

        try:
            # Get Device by UID and create Pipeline for this specific device
            ctx = Context()
            dev_list = ctx.query_devices()
            device = dev_list.get_device_by_uid(self.device_info["uid"])
            self.pipeline = Pipeline(device)
            self.config = Config()

            # Align filter (depth to color)
            try:
                self.align_filter = AlignFilter(
                    align_to_stream=OBStreamType.COLOR_STREAM
                )
            except Exception:
                self.align_filter = None

            self._setup_streams()

            # Enable frame sync (optional)
            try:
                self.pipeline.enable_frame_sync()
            except Exception:
                pass

            self.pipeline.start(self.config)
            return True
        except Exception as e:
            print(f"[Camera] Init failed for {self.device_info['name']} SN={self.device_info['serial']}: {e}")
            return False

    def _setup_streams(self):
        """Configure color and depth streams."""
        # Color stream (prefer MJPG for compression efficiency)
        color_profile_list = self.pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR
        )
        color_profile = None
        try:
            color_profile = color_profile_list.get_video_stream_profile(
                self.width, self.height, OBFormat.MJPG, self.fps
            )
        except Exception:
            try:
                color_profile = color_profile_list.get_video_stream_profile(
                    self.width, self.height, OBFormat.RGB, self.fps
                )
            except Exception:
                color_profile = (
                    color_profile_list.get_default_video_stream_profile()
                )
        self.config.enable_stream(color_profile)

        # Depth stream
        depth_profile_list = self.pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR
        )
        depth_profile = None
        try:
            depth_profile = depth_profile_list.get_video_stream_profile(
                self.width, self.height, OBFormat.Y16, self.fps
            )
        except Exception:
            depth_profile = (
                depth_profile_list.get_default_video_stream_profile()
            )
        self.config.enable_stream(depth_profile)

    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture a single RGB frame. Returns BGR image or None."""
        if self.pipeline is None:
            return None

        try:
            frames = self.pipeline.wait_for_frames(100)
            if frames is None:
                return None

            # Align depth to color
            if self.align_filter:
                frames = self.align_filter.process(frames)
                if frames is None:
                    return None
                frames = frames.as_frame_set()

            color_frame = frames.get_color_frame()
            if not color_frame:
                return None

            # Convert to BGR
            if frame_to_bgr_image is not None:
                color_image = frame_to_bgr_image(color_frame)
            else:
                data = np.asanyarray(color_frame.get_data())
                fmt = color_frame.get_format()
                if fmt == OBFormat.MJPG:
                    color_image = cv2.imdecode(data, cv2.IMREAD_COLOR)
                elif fmt == OBFormat.RGB:
                    color_image = data.reshape(
                        (color_frame.get_height(), color_frame.get_width(), 3)
                    )
                    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                else:
                    color_image = data.reshape(
                        (color_frame.get_height(), color_frame.get_width(), 3)
                    )

            return color_image
        except Exception:
            return None

    def stop(self):
        """Stop the camera pipeline."""
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass


class CameraDataProcess(threading.Thread):
    """Thread-based multi-camera capture (avoids fork+threads deadlock with rclpy).

    Writes JPEG-compressed images directly to HDF5.
    """

    def __init__(
        self,
        start_event: threading.Event,
        stop_event: threading.Event,
        episode_path: str,
        camera_configs: List[Dict],
    ):
        """
        Args:
            start_event: Set to start recording.
            stop_event: Set to stop recording.
            episode_path: Directory to save camera_data.h5.
            camera_configs: List of dicts with keys:
                - id: str camera identifier (e.g., "cam_0")
                - serial: str optional serial number for exact matching
                - width: int (default 640)
                - height: int (default 480)
                - fps: int (default 30)
        """
        super().__init__()
        self.start_event = start_event
        self.stop_event = stop_event
        self.episode_path = episode_path
        self.camera_configs = camera_configs
        self.cameras_status = {}  # filled by run(): {cam_id: "online"/"offline"} for each config
        for cfg in camera_configs:
            self.cameras_status[cfg["id"]] = "offline"

    def run(self):
        if not HAS_ORBBEC:
            print("[Camera] Orbbec SDK not available. Camera process exiting.")
            return

        # ---- Enumerate devices in child process ----
        all_devices = _get_device_list()
        print(f"[Camera] Found {len(all_devices)} Orbbec device(s):")
        for d in all_devices:
            print(f"  [{d['index']}] {d['name']}  SN={d['serial']}  UID={d['uid']}")

        if not all_devices:
            print("[Camera] No devices found. Process exiting.")
            return

        # ---- Match configs to devices ----
        cameras: Dict[str, GeminiCamera] = {}
        used_uids = set()

        for cfg in self.camera_configs:
            dev = _find_device_for_config(all_devices, cfg)
            if dev is None or dev["uid"] in used_uids:
                # Fallback: pick first unused device of matching type
                for d in all_devices:
                    if d["uid"] not in used_uids:
                        dev = d
                        break
            if dev is None:
                print(f"[Camera] {cfg['id']}: no available device, skipping.")
                continue

            used_uids.add(dev["uid"])
            cam = GeminiCamera(
                device_info=dev,
                width=cfg.get("width", 640),
                height=cfg.get("height", 480),
                fps=cfg.get("fps", 30),
            )
            if cam.initialize():
                cameras[cfg["id"]] = cam
                self.cameras_status[cfg["id"]] = "online"
                print(f"[Camera] {cfg['id']} → {dev['name']} SN={dev['serial']} "
                      f"({cam.width}x{cam.height} @ {cam.fps}fps)")
            else:
                print(f"[Camera] {cfg['id']} initialization failed, skipping.")

        if not cameras:
            print("[Camera] No cameras initialized. Process exiting.")
            return

        active_ids = sorted(cameras.keys())

        # ---- Warm-up phase ----
        print("[Camera] Warming up (auto-exposure / white balance)...")
        while not self.start_event.is_set() and not self.stop_event.is_set():
            for cam_id in active_ids:
                try:
                    cameras[cam_id].capture_frame()
                except Exception:
                    pass
            time.sleep(0.01)

        if self.stop_event.is_set():
            for cam in cameras.values():
                cam.stop()
            return

        # ---- Recording phase ----
        print("[Camera] Start event received. Recording...")

        h5_path = os.path.join(self.episode_path, "camera_data.h5")
        cam_file = h5py.File(h5_path, "w")

        image_dsets = {}
        ts_dsets = {}
        dset_sizes = {}
        idxs = {}

        for cam_id in active_ids:
            image_dsets[cam_id] = cam_file.create_dataset(
                f"{cam_id}/images",
                shape=(0,),
                maxshape=(None,),
                dtype=h5py.special_dtype(vlen=np.dtype("uint8")),
            )
            ts_dsets[cam_id] = cam_file.create_dataset(
                f"{cam_id}/timestamps",
                shape=(0,),
                maxshape=(None,),
                dtype="f8",
            )
            dset_sizes[cam_id] = 0
            idxs[cam_id] = 0

        while not self.stop_event.is_set():
            for cam_id in active_ids:
                cam = cameras[cam_id]
                color_img = cam.capture_frame()
                timestamp = time.time()

                if color_img is None:
                    continue

                try:
                    jpeg_bytes = jpeg_compress(color_img, quality=90)
                except Exception:
                    continue

                i = idxs[cam_id]

                if i >= dset_sizes[cam_id]:
                    dset_sizes[cam_id] += CHUNK_SIZE
                    image_dsets[cam_id].resize((dset_sizes[cam_id],))
                    ts_dsets[cam_id].resize((dset_sizes[cam_id],))

                image_dsets[cam_id][i] = np.frombuffer(jpeg_bytes, dtype="uint8")
                ts_dsets[cam_id][i] = timestamp
                idxs[cam_id] += 1

        # ---- Cleanup ----
        print("[Camera] Stop event received. Saving...")
        for cam_id in active_ids:
            final_size = idxs[cam_id]
            image_dsets[cam_id].resize((final_size,))
            ts_dsets[cam_id].resize((final_size,))
            cameras[cam_id].stop()

        cam_file.close()
        counts = {cid: idxs[cid] for cid in active_ids}
        print(f"[Camera] Saved {counts} -> {h5_path}")
