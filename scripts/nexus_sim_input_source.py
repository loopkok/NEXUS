#!/usr/bin/env python3
"""Publish synthetic Quest 3 and camera streams from a NEXUS profile.

This publisher-only fixture keeps performance probes lightweight: it does not
subscribe to the graph or open physical devices.
"""

from __future__ import annotations

import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CompressedImage, Joy
from std_msgs.msg import String

from nexus_core.profile import Profile
from quest3_hand_mocap.pose_mapping import rotate_pose


def _landmarks(curl: float) -> list[tuple[float, float, float]]:
    points = np.array([
        [0, 0, 0], [.008, -.002, -.004], [.016, -.004, -.006], [.024, -.005, -.005], [.032, -.006, -.003],
        [.004, .002, -.018], [.003, .002, -.036], [.002, .002, -.048], [.001, .001, -.056],
        [0, .001, -.020], [-.001, 0, -.040], [-.002, 0, -.052], [-.003, 0, -.060],
        [-.004, -.001, -.017], [-.006, -.001, -.035], [-.007, -.001, -.046], [-.008, -.001, -.054],
        [-.007, -.002, -.014], [-.010, -.003, -.028], [-.012, -.003, -.036], [-.014, -.003, -.042],
    ], dtype=float)
    for start in (5, 9, 13, 17):
        root = points[start].copy()
        for idx in range(start + 1, start + 4):
            k = curl * (idx - start) / 3.0 * 0.65
            original = points[idx].copy()
            points[idx] = [original[0], original[1] + k * .018,
                           root[2] + (original[2] - root[2]) * (1 - k)]
    return [tuple(map(float, point)) for point in points]


class SyntheticQuestSource(Node):
    def __init__(self, profile: Profile, amplitude: float, frequency: float,
                 image_rate: float, hand_curl: float, motion_axis: str = "y"):
        super().__init__("nexus_synthetic_quest_source")
        self.profile = profile
        self.amplitude = amplitude
        self.frequency = frequency
        self.hand_curl = hand_curl
        self.motion_axis = motion_axis
        self.wrist_mapping = profile.raw.get("input_settings", {}).get(
            "quest3_wrist_pose_mapping", {}
        )
        self.wrist_rotations = {
            side: np.asarray(
                self.wrist_mapping.get(f"{side}_rotation", np.eye(3).reshape(-1)),
                dtype=float,
            ).reshape(3, 3)
            for side in ("left", "right")
        }
        self.started = time.monotonic()
        self._rate_started = self.started
        self._last_input_at = 0.0
        self._max_input_gap = 0.0
        self._input_count = 0
        self.frame = 0
        self.wrists = {}
        self.hands = {}
        self.joys = {}
        for side in ("left", "right"):
            self.wrists[side] = self.create_publisher(
                PoseStamped, profile.input_spec(side, "wrist")["topic"], qos_profile_sensor_data)
            self.hands[side] = self.create_publisher(
                PoseArray, profile.input_spec(side, "hand")["topic"], qos_profile_sensor_data)
            joy_spec = profile.input_spec(side, "controller_joy")
            self.joys[side] = (self.create_publisher(Joy, joy_spec["topic"], qos_profile_sensor_data)
                               if joy_spec else None)
        self.body_pub = None
        body_spec = profile.input_spec("body", "body_joints")
        if body_spec:
            self.body_pub = self.create_publisher(PoseArray, body_spec["topic"], qos_profile_sensor_data)
        self.body_names_pub = None
        names_spec = profile.input_spec("body", "body_joint_names")
        if names_spec:
            names_qos = QoSProfile(
                depth=1, history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.body_names_pub = self.create_publisher(
                String, names_spec["topic"], names_qos)

        self.camera_publishers = {}
        self.camera_frames = {}
        for index, camera in enumerate(profile.raw["cameras"]):
            topic = camera.get("capture_topic")
            if not topic:
                continue
            image = np.zeros((120, 160, 3), dtype=np.uint8)
            image[:, :, :] = (25 + index * 45, 40, 70)
            cv2.rectangle(image, (40 + index * 17, 45), (60 + index * 17, 74), (240, 230, 40), -1)
            ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if not ok:
                raise RuntimeError(f"cannot create synthetic JPEG for camera {camera['role']}")
            self.camera_publishers[camera["role"]] = self.create_publisher(
                CompressedImage, topic, qos_profile_sensor_data)
            self.camera_frames[camera["role"]] = encoded.tobytes()
        self.create_timer(.02, self._publish_inputs)
        self.create_timer(1.0 / image_rate, self._publish_images)
        self.create_timer(5.0, self._log_input_rate)

    def _publish_inputs(self):
        now = time.monotonic()
        if self._last_input_at:
            self._max_input_gap = max(self._max_input_gap, now - self._last_input_at)
        self._last_input_at = now
        self._input_count += 1
        elapsed = now - self.started
        curl = self.hand_curl
        stamp = self.get_clock().now().to_msg()
        for side in ("left", "right"):
            wrist = PoseStamped()
            wrist.header.stamp = stamp
            wrist.header.frame_id = self.wrist_mapping.get(
                f"{side}_frame_id", f"quest3_{side}_wrist")
            raw_position = np.zeros(3, dtype=float)
            raw_position["xyz".index(self.motion_axis)] = (
                self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed))
            mapped_position, mapped_quat = rotate_pose(
                raw_position, np.array([0.0, 0.0, 0.0, 1.0]),
                self.wrist_rotations[side])
            wrist.pose.position.x = float(mapped_position[0])
            wrist.pose.position.y = float(mapped_position[1])
            wrist.pose.position.z = float(mapped_position[2])
            wrist.pose.orientation.x = float(mapped_quat[0])
            wrist.pose.orientation.y = float(mapped_quat[1])
            wrist.pose.orientation.z = float(mapped_quat[2])
            wrist.pose.orientation.w = float(mapped_quat[3])
            self.wrists[side].publish(wrist)

            hand = PoseArray()
            hand.header.stamp = stamp
            hand.header.frame_id = f"hand_{side}"
            hand.poses = [Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(w=1.0))
                          for x, y, z in _landmarks(curl)]
            self.hands[side].publish(hand)
            if self.joys[side]:
                joy = Joy()
                joy.header.stamp = stamp
                joy.axes = [0.0] * 4
                joy.buttons = [0] * 4
                self.joys[side].publish(joy)

        if self.body_pub:
            body = PoseArray()
            body.header.stamp = stamp
            body.header.frame_id = "quest3_body"
            body.poses = [Pose(orientation=Quaternion(w=1.0))]
            self.body_pub.publish(body)
        if self.body_names_pub:
            self.body_names_pub.publish(String(data='["root"]'))

    def _log_input_rate(self):
        now = time.monotonic()
        elapsed = max(1e-6, now - self._rate_started)
        self.get_logger().info(
            f"synthetic inputs hz={self._input_count / elapsed:.1f} "
            f"max_gap_ms={self._max_input_gap * 1000:.1f}")
        self._rate_started = now
        self._input_count = 0
        self._max_input_gap = 0.0

    def _publish_images(self):
        self.frame += 1
        stamp = self.get_clock().now().to_msg()
        for role, publisher in self.camera_publishers.items():
            message = CompressedImage()
            message.header.stamp = stamp
            message.header.frame_id = f"sim_camera_{role}"
            message.format = "jpeg"
            message.data = self.camera_frames[role]
            publisher.publish(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--amplitude", type=float, default=.008)
    parser.add_argument("--frequency", type=float, default=.15)
    parser.add_argument("--motion-axis", choices=("x", "y", "z"), default="y")
    parser.add_argument("--image-rate", type=float, default=30.0)
    parser.add_argument("--hand-curl", type=float, default=0.0,
                        help="fixed synthetic finger curl in [0, 1]")
    args = parser.parse_args()
    if (args.amplitude < 0 or args.frequency <= 0 or args.image_rate <= 0
            or not 0.0 <= args.hand_curl <= 1.0):
        parser.error("amplitude/rates must be positive and hand-curl must be in [0, 1]")
    rclpy.init()
    node = SyntheticQuestSource(Profile.load(args.profile), args.amplitude, args.frequency,
                                args.image_rate, args.hand_curl, args.motion_axis)
    node.get_logger().info(f"synthetic Quest source profile={node.profile.profile_id} "
                           f"sha256={node.profile.digest}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
