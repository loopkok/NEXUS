#!/usr/bin/env python3
"""Replay collected data on physical Nero arms (CAN) and XHands (ROS2).

Reads robot_data.h5 or aligned_data.h5 and replays joint trajectories
on the real robot hardware:
  - Nero arms: direct CAN control via pyAgxArm (MIT passthrough mode)
  - XHands: ROS2 XHandCommand topics → xhand_control_ros2 serial driver

Usage:
    # Replay raw robot data
    python replay_episode.py --episode_dir ~/xnero_data/Data/task/episode0

    # Replay aligned data
    python replay_episode.py --episode_dir ... --aligned

    # Half speed
    python replay_episode.py --episode_dir ... --speed 0.5
"""

import os
import sys
import time
import argparse
import signal

import h5py
import numpy as np
import rclpy
from rclpy.node import Node

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

XHAND_JOINT_NAMES = [
    "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
    "index_bend_joint", "index_joint1", "index_joint2",
    "mid_joint1", "mid_joint2",
    "ring_joint1", "ring_joint2",
    "pinky_joint1", "pinky_joint2",
]

XHAND_KP = 80.0
XHAND_KI = 0.0
XHAND_KD = 0.0
XHAND_EFFORT_LIMIT = 400.0
XHAND_MODE = 3

INIT_SPEED_PERCENT = 5  # 初始化移动到起始位姿的速度百分比
REPLAY_FRAME_DT = 0.02   # 默认回放帧间隔 (50Hz)


class NeroArmReplay:
    """Direct CAN control wrapper for a single Nero arm during replay."""

    def __init__(self, can_channel: str, side: str, fw_version: str = "default"):
        self.can_channel = can_channel
        self.side = side
        self.robot = None
        self._fw = NeroFW.V111 if fw_version == "v111" else NeroFW.DEFAULT

    def connect(self) -> bool:
        print(f"[{self.side.upper()}] Connecting on {self.can_channel}...")
        cfg = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=self._fw,
            channel=self.can_channel,
            interface="socketcan",
        )
        self.robot = AgxArmFactory.create_arm(cfg)
        self.robot.connect()

        # Enable arm
        print(f"[{self.side.upper()}] Enabling...")
        while not self.robot.enable():
            time.sleep(0.01)
        print(f"[{self.side.upper()}] Enabled.")

        # Switch to MIT passthrough mode
        self.robot.set_motion_mode("js")
        self.robot.set_auto_set_motion_mode_enabled(False)
        return True

    def move_to_start(self, target_joints: np.ndarray, slow: bool = True):
        """Move arm to starting pose before replay."""
        q = [float(v) for v in target_joints.ravel()[:7]]

        if slow:
            print(f"[{self.side.upper()}] Moving to start pose at {INIT_SPEED_PERCENT}% speed...")
            self.robot.set_speed_percent(INIT_SPEED_PERCENT)
            self.robot.set_motion_mode("j")
            self.robot.move_j(q)

            # Wait for arrival
            arrival_threshold = 0.05
            timeout = 15.0
            start = time.monotonic()
            while (time.monotonic() - start) < timeout:
                ja = self.robot.get_joint_angles()
                if ja is not None:
                    current = np.array(ja.msg[:7])
                    error = np.max(np.abs(current - np.array(q)))
                    if error < arrival_threshold:
                        break
                time.sleep(0.1)

            self.robot.set_speed_percent(100)
            self.robot.set_motion_mode("js")
            print(f"[{self.side.upper()}] Start pose reached.")
        else:
            self.robot.move_js(q)

    def send_joints(self, joints: np.ndarray):
        """Send joint angles via MIT passthrough (low-latency)."""
        q = [float(v) for v in joints.ravel()[:7]]
        self.robot.move_js(q)

    def disconnect(self):
        if self.robot:
            try:
                self.robot.set_normal_mode()
                time.sleep(0.1)
                self.robot.disconnect()
            except Exception:
                pass


class ReplayController:
    """Orchestrates replay of dual arms (CAN) + dual hands (ROS2 topics)."""

    def __init__(
        self,
        episode_dir: str,
        use_aligned: bool = False,
        speed: float = 1.0,
        loop_count: int = 1,
        left_can: str = "can_nero_left",
        right_can: str = "can_nero_right",
        fw_version: str = "default",
    ):
        self.episode_dir = episode_dir
        self.speed = speed
        self.loop_count = loop_count
        self.left_can = left_can
        self.right_can = right_can
        self.fw_version = fw_version

        # ---- Load data ----
        h5_name = "aligned_data.h5" if use_aligned else "robot_data.h5"
        h5_path = os.path.join(episode_dir, h5_name)

        if not os.path.exists(h5_path):
            raise FileNotFoundError(f"Data file not found: {h5_path}")

        print(f"Loading data from: {h5_path}")
        with h5py.File(h5_path, "r") as f:
            if use_aligned:
                self.la_joints = f["observation/left_arm_joints"][:]
                self.ra_joints = f["observation/right_arm_joints"][:]
                self.lh_joints = f["observation/left_hand_joints"][:]
                self.rh_joints = f["observation/right_hand_joints"][:]
            else:
                self.la_joints = f["left_arm/joints"][:]
                self.ra_joints = f["right_arm/joints"][:]
                self.lh_joints = (
                    f["left_hand/joints"][:]
                    if "left_hand/joints" in f
                    else np.zeros((self.la_joints.shape[0], 12))
                )
                self.rh_joints = (
                    f["right_hand/joints"][:]
                    if "right_hand/joints" in f
                    else np.zeros((self.ra_joints.shape[0], 12))
                )

        self.num_frames = self.la_joints.shape[0]
        print(
            f"Loaded {self.num_frames} frames. "
            f"Speed: {speed}x, Loops: {loop_count}"
        )

        # ---- ROS2 for hand control ----
        self._node: Node = None
        self._left_hand_pub = None
        self._right_hand_pub = None
        self._has_xhand = False

    def _init_ros2(self):
        """Initialize ROS2 for hand control."""
        self._node = Node("replay_controller")

        try:
            from xhand_control_interfaces.msg import XHandCommand
            self._has_xhand = True
            self._left_hand_pub = self._node.create_publisher(
                XHandCommand, "/left_hand/xhand_command", 10
            )
            self._right_hand_pub = self._node.create_publisher(
                XHandCommand, "/right_hand/xhand_command", 10
            )
            print("[Hand] XHandCommand publishers ready.")
        except ImportError:
            self._has_xhand = False
            print("[Hand] xhand_control_interfaces not available. Hand replay disabled.")

    def _build_hand_msg(self, joints: np.ndarray):
        """Build XHandCommand for hand replay."""
        try:
            from xhand_control_interfaces.msg import XHandCommand
        except ImportError:
            return None

        msg = XHandCommand()
        msg.hand_id = 0
        msg.name = XHAND_JOINT_NAMES
        jv = joints.ravel()[:12].tolist()
        while len(jv) < 12:
            jv.append(0.0)
        msg.position = [float(v) for v in jv]
        msg.kp = [XHAND_KP] * 12
        msg.ki = [XHAND_KI] * 12
        msg.kd = [XHAND_KD] * 12
        msg.effort_limit = [XHAND_EFFORT_LIMIT] * 12
        msg.mode = XHAND_MODE
        return msg

    def run(self):
        """Main replay execution."""
        # ---- Connect to arms via CAN ----
        left_arm = NeroArmReplay(self.left_can, "left", self.fw_version)
        right_arm = NeroArmReplay(self.right_can, "right", self.fw_version)

        try:
            left_arm.connect()
            right_arm.connect()

            # ---- Initialize ROS2 for hands ----
            self._init_ros2()

            # ---- Move to first frame pose (slow) ----
            print("\n--- Moving to start pose ---")
            left_arm.move_to_start(self.la_joints[0], slow=True)
            right_arm.move_to_start(self.ra_joints[0], slow=True)

            # Send first hand pose
            if self._has_xhand:
                lh_msg = self._build_hand_msg(self.lh_joints[0])
                rh_msg = self._build_hand_msg(self.rh_joints[0])
                if lh_msg:
                    self._left_hand_pub.publish(lh_msg)
                if rh_msg:
                    self._right_hand_pub.publish(rh_msg)
                # Spin a few times to deliver hand commands
                rclpy.spin_once(self._node, timeout_sec=0.1)

            print("\n--- Start pose reached. Ready for replay. ---")
            input(">>> Press Enter to start replay (Ctrl+C to abort)...\n")

            # ---- Replay loops ----
            for loop in range(self.loop_count):
                if loop > 0:
                    print(f"\n--- Loop {loop + 1}/{self.loop_count} ---")
                    # Move back to first frame
                    left_arm.send_joints(self.la_joints[0])
                    right_arm.send_joints(self.ra_joints[0])
                    time.sleep(0.5)

                start_time = time.time()

                for i in range(self.num_frames):
                    t_frame_start = time.time()

                    # ---- Arm control (CAN direct) ----
                    left_arm.send_joints(self.la_joints[i])
                    right_arm.send_joints(self.ra_joints[i])

                    # ---- Hand control (ROS2 topic) ----
                    if self._has_xhand:
                        lh_msg = self._build_hand_msg(self.lh_joints[i])
                        rh_msg = self._build_hand_msg(self.rh_joints[i])
                        if lh_msg:
                            self._left_hand_pub.publish(lh_msg)
                        if rh_msg:
                            self._right_hand_pub.publish(rh_msg)

                    # Spin ROS to deliver hand messages
                    rclpy.spin_once(self._node, timeout_sec=0.001)

                    # ---- Rate control ----
                    if i < self.num_frames - 1 and self.speed > 0:
                        sleep_time = REPLAY_FRAME_DT / self.speed
                        elapsed = time.time() - t_frame_start
                        if sleep_time > elapsed:
                            time.sleep(sleep_time - elapsed)

                    if i % 100 == 0 and i > 0:
                        elapsed = time.time() - start_time
                        print(f"  Frame {i}/{self.num_frames} "
                              f"({i / elapsed:.1f} fps)", end="\r")

                elapsed_total = time.time() - start_time
                print(f"\n  Loop {loop + 1} done: {self.num_frames} frames "
                      f"in {elapsed_total:.1f}s "
                      f"({self.num_frames / elapsed_total:.1f} fps avg)")

        except KeyboardInterrupt:
            print("\n\nReplay interrupted by user.")
        finally:
            print("\n--- Cleanup ---")
            left_arm.disconnect()
            right_arm.disconnect()
            if self._node:
                self._node.destroy_node()
            print("Arms disconnected. Done.")


def main():
    parser = argparse.ArgumentParser(
        description="Replay recorded episodes on physical Nero arms (CAN) + XHands (ROS2)."
    )
    parser.add_argument(
        "--episode_dir", type=str, required=True,
        help="Path to the episode directory containing robot_data.h5 or aligned_data.h5."
    )
    parser.add_argument(
        "--aligned", action="store_true",
        help="Use aligned_data.h5 instead of robot_data.h5."
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Replay speed multiplier (default: 1.0)."
    )
    parser.add_argument(
        "--loop_count", type=int, default=1,
        help="Number of replay loops (default: 1)."
    )
    parser.add_argument(
        "--left_can", type=str, default="can_nero_left",
        help="CAN channel for left arm (default: can_nero_left)."
    )
    parser.add_argument(
        "--right_can", type=str, default="can_nero_right",
        help="CAN channel for right arm (default: can_nero_right)."
    )
    parser.add_argument(
        "--fw_version", type=str, default="default",
        help="Nero firmware version: 'default' or 'v111'."
    )

    args = parser.parse_args()

    # Initialize rclpy for hand control
    rclpy.init(args=sys.argv)

    controller = ReplayController(
        episode_dir=args.episode_dir,
        use_aligned=args.aligned,
        speed=args.speed,
        loop_count=args.loop_count,
        left_can=args.left_can,
        right_can=args.right_can,
        fw_version=args.fw_version,
    )

    # Signal handling
    def sig_handler(signum, frame):
        print("\nSignal received, stopping replay...")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        controller.run()
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
    except KeyboardInterrupt:
        print("\nReplay stopped.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
