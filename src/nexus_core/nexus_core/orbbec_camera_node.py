"""Single owner for Nero Gemini cameras, publishing ROS image contracts."""

from __future__ import annotations

import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .profile import Profile


class OrbbecCameraNode(Node):
    def __init__(self):
        super().__init__("nexus_orbbec_camera")
        self.declare_parameter("profile_file", "")
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        from nero_dual_data_collect.camera_manager import GeminiCamera, _get_device_list
        devices = {str(d["serial"]): d for d in _get_device_list()}
        cameras = [c for c in self.profile.raw["cameras"] if c["source"] == "orbbec"]
        if not cameras:
            raise RuntimeError("profile has no Orbbec camera")
        missing = [c["device"] for c in cameras if c["device"] not in devices]
        if missing:
            raise RuntimeError(f"missing camera serials {missing}; refusing index fallback")
        self._stop = threading.Event()
        self._threads = []
        self._cameras = []
        for row in cameras:
            role = row["role"]
            camera = GeminiCamera(devices[row["device"]],
                                  width=int(row.get("width", 640)),
                                  height=int(row.get("height", 480)),
                                  fps=int(row.get("fps", 30)))
            if not camera.initialize():
                raise RuntimeError(f"camera {role} ({row['device']}) failed to initialize")
            self._cameras.append(camera)
            jpg_pub = self.create_publisher(
                CompressedImage, f"{self.profile.namespace}/camera/{role}/image/compressed",
                qos_profile_sensor_data)
            raw_pub = self.create_publisher(
                Image, f"{self.profile.namespace}/camera/{role}/image/raw",
                qos_profile_sensor_data)
            thread = threading.Thread(target=self._capture, args=(role, camera, jpg_pub, raw_pub),
                                      name=f"orbbec-{role}", daemon=True)
            thread.start()
            self._threads.append(thread)
        self.get_logger().info(f"Orbbec cameras {[c['role'] for c in cameras]} "
                               f"profile_sha256={self.profile.digest}")

    def _capture(self, role, camera, jpg_pub, raw_pub):
        while not self._stop.is_set() and rclpy.ok():
            frame = camera.capture_frame()
            if frame is None:
                continue
            stamp = self.get_clock().now().to_msg()
            frame_id = f"{role}_optical_frame"
            if raw_pub.get_subscription_count() > 0:
                raw = Image()
                raw.header.stamp = stamp
                raw.header.frame_id = frame_id
                raw.height, raw.width = frame.shape[:2]
                raw.encoding = "bgr8"
                raw.is_bigendian = 0
                raw.step = raw.width * 3
                raw.data = frame.tobytes()
                raw_pub.publish(raw)
            if jpg_pub.get_subscription_count() > 0:
                ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                if ok:
                    msg = CompressedImage()
                    msg.header.stamp = stamp
                    msg.header.frame_id = frame_id
                    msg.format = "jpeg"
                    msg.data = encoded.tobytes()
                    jpg_pub.publish(msg)
        self.get_logger().info(f"camera {role} capture stopped")

    def destroy_node(self):
        self._stop.set()
        for camera in self._cameras:
            camera.stop()
        for thread in self._threads:
            thread.join(timeout=2.0)
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = OrbbecCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
