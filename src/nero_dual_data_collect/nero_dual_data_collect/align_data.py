#!/usr/bin/env python3
"""Post-processing data alignment for dual-arm dual-hand data collection.

Aligns robot data (arms + hands) with camera timestamps using
nearest-neighbor search. Produces frame-by-frame (state, action) pairs
suitable for behavior cloning / VLA training.

Usage:
    python align_data.py --data_dir ~/xnero_data/Data/default_task
    python align_data.py --data_dir ~/xnero_data/Data/default_task --start_episode 0
"""

import os
import sys
import argparse
import shutil
from glob import glob

import h5py
import numpy as np
import cv2


HOLD_FRAMES = 20  # Number of hold frames appended at episode end
DEFAULT_RESIZE = (640, 360)  # Default resize dimensions (W, H)


def find_nearest_idx(array: np.ndarray, value: float) -> int:
    """Find index of the nearest value in a sorted array (binary search)."""
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (
        idx == len(array)
        or abs(value - array[idx - 1]) < abs(value - array[idx])
    ):
        return idx - 1
    else:
        return idx


def post_process_append_hold(
    data_dict: dict,
    frames_dir: str,
    num_cams: int,
    n_repeat: int = HOLD_FRAMES,
):
    """Append hold frames at the end of an episode."""
    if n_repeat <= 0:
        return

    timestamps = data_dict["timestamps"]
    if len(timestamps) < 2:
        print("  Warning: too few frames, skipping hold append.")
        return

    dt = timestamps[-1] - timestamps[-2]

    # Get last frame data
    last_data = {
        "ts": timestamps[-1],
        "la_j": data_dict["left_arm_joints"][-1],
        "ra_j": data_dict["right_arm_joints"][-1],
        "lh_j": data_dict["left_hand_joints"][-1],
        "rh_j": data_dict["right_hand_joints"][-1],
        "la_j_a": data_dict["left_arm_joints_actions"][-1],
        "ra_j_a": data_dict["right_arm_joints_actions"][-1],
        "lh_j_a": data_dict["left_hand_joints_actions"][-1],
        "rh_j_a": data_dict["right_hand_joints_actions"][-1],
    }

    current_count = len(timestamps)
    last_img_idx = current_count - 1

    print(f"  - Appending {n_repeat} hold frames (dt={dt:.4f}s)")

    for i in range(1, n_repeat + 1):
        new_ts = last_data["ts"] + (dt * i)
        data_dict["timestamps"].append(new_ts)
        data_dict["left_arm_joints"].append(last_data["la_j"])
        data_dict["right_arm_joints"].append(last_data["ra_j"])
        data_dict["left_hand_joints"].append(last_data["lh_j"])
        data_dict["right_hand_joints"].append(last_data["rh_j"])
        data_dict["left_arm_joints_actions"].append(last_data["la_j_a"])
        data_dict["right_arm_joints_actions"].append(last_data["ra_j_a"])
        data_dict["left_hand_joints_actions"].append(last_data["lh_j_a"])
        data_dict["right_hand_joints_actions"].append(last_data["rh_j_a"])

        # Copy images
        new_img_idx = last_img_idx + i
        for cam_id in range(num_cams):
            src_path = os.path.join(
                frames_dir, f"cam{cam_id}", f"{last_img_idx:06d}.jpg"
            )
            dst_path = os.path.join(
                frames_dir, f"cam{cam_id}", f"{new_img_idx:06d}.jpg"
            )
            if os.path.exists(src_path):
                shutil.copy(src_path, dst_path)


def process_episode(
    episode_path: str,
    resize: tuple = DEFAULT_RESIZE,
    downsample: int = 1,
):
    """Process a single episode: align robot data with camera frames."""
    robot_h5 = os.path.join(episode_path, "robot_data.h5")
    camera_h5 = os.path.join(episode_path, "camera_data.h5")

    if not os.path.exists(robot_h5) or not os.path.exists(camera_h5):
        print(f"  Skip {episode_path}: missing data files.")
        return

    print(f"Processing: {episode_path}")

    try:
        f_cam = h5py.File(camera_h5, "r")
        f_robot = h5py.File(robot_h5, "r")
    except Exception as e:
        print(f"  Error reading HDF5: {e}")
        return

    # Find camera groups
    cam_groups = [
        k for k in f_cam.keys()
        if "images" in f_cam[k] if isinstance(f_cam[k], h5py.Group)
    ]
    # Fallback: check for old-style dataset naming
    if not cam_groups:
        cam_keys = [k for k in f_cam.keys() if k.startswith("images_")]
        # Convert old format check
        cam_groups = []
        for k in f_cam.keys():
            if isinstance(f_cam[k], h5py.Group) and "images" in f_cam[k]:
                cam_groups.append(k)

    if not cam_groups:
        # Try direct datasets
        img_keys = [k for k in f_cam.keys() if "image" in k.lower()]
        if not img_keys:
            print("  No camera data found.")
            f_cam.close()
            f_robot.close()
            return

    # Determine camera IDs
    cam_ids = sorted(cam_groups) if cam_groups else ["cam_0"]

    # --- Master clock: first camera timestamps ---
    master_cam = cam_ids[0]
    all_timestamps = f_cam[f"{master_cam}/timestamps"][:]

    if len(all_timestamps) == 0:
        print("  No camera timestamps found.")
        f_cam.close()
        f_robot.close()
        return

    # Downsampling
    target_timestamps = all_timestamps[::downsample]

    # Discard last frame (no t+1 action)
    num_valid_frames = len(target_timestamps) - 1
    if num_valid_frames < 1:
        print("  Not enough frames to build (state, action) pairs.")
        f_cam.close()
        f_robot.close()
        return

    # Prepare output directories
    frames_dir = os.path.join(episode_path, "frames")
    if os.path.exists(frames_dir):
        shutil.rmtree(frames_dir)
    for i in range(len(cam_ids)):
        os.makedirs(os.path.join(frames_dir, f"cam{i}"), exist_ok=True)

    # Data containers
    data_dict = {
        "timestamps": [],
        "left_arm_joints": [],
        "right_arm_joints": [],
        "left_hand_joints": [],
        "right_hand_joints": [],
        "left_arm_joints_actions": [],
        "right_arm_joints_actions": [],
        "left_hand_joints_actions": [],
        "right_hand_joints_actions": [],
    }

    # Robot data
    robot_ts = f_robot["timestamps"][:]

    # Helper to read robot dataset
    def _read_robot_ds(name):
        if name in f_robot:
            return f_robot[name][:]
        return np.zeros((len(robot_ts), 1))

    la_joints = _read_robot_ds("left_arm/joints")
    ra_joints = _read_robot_ds("right_arm/joints")
    lh_joints = _read_robot_ds("left_hand/joints")
    rh_joints = _read_robot_ds("right_hand/joints")

    # Handle shape issues
    for arr, name, expected_dim in [
        (la_joints, "left_arm/joints", 7),
        (ra_joints, "right_arm/joints", 7),
        (lh_joints, "left_hand/joints", 12),
        (rh_joints, "right_hand/joints", 12),
    ]:
        if arr.ndim == 1:
            arr = arr.reshape(-1, 1)
        if arr.shape[1] < expected_dim:
            pad = np.zeros((arr.shape[0], expected_dim - arr.shape[1]))
            arr = np.hstack([arr, pad])

    print(
        f"  Frames: {len(target_timestamps)} -> {num_valid_frames} valid "
        f"(downsample={downsample})"
    )

    # --- Main alignment loop ---
    for i in range(num_valid_frames):
        curr_ts = target_timestamps[i]
        next_ts = target_timestamps[i + 1]

        # Save images for each camera
        for cam_idx, cam_id in enumerate(cam_ids):
            ts_dset = f_cam[f"{cam_id}/timestamps"][:]
            img_idx = find_nearest_idx(ts_dset, curr_ts)

            try:
                raw_data = f_cam[f"{cam_id}/images"][img_idx]
                img_array = cv2.imdecode(
                    np.frombuffer(raw_data, np.uint8), cv2.IMREAD_COLOR
                )
            except Exception:
                img_array = None

            if img_array is None:
                img_resized = np.zeros(
                    (resize[1], resize[0], 3), dtype=np.uint8
                )
            else:
                img_resized = cv2.resize(
                    img_array, resize, interpolation=cv2.INTER_AREA
                )

            save_path = os.path.join(
                frames_dir, f"cam{cam_idx}", f"{i:06d}.jpg"
            )
            cv2.imwrite(save_path, img_resized)

        # Find nearest robot data for state and action
        r_idx_curr = find_nearest_idx(robot_ts, curr_ts)
        r_idx_next = find_nearest_idx(robot_ts, next_ts)

        r_idx_curr = min(max(r_idx_curr, 0), len(robot_ts) - 1)
        r_idx_next = min(max(r_idx_next, 0), len(robot_ts) - 1)

        # Append data
        data_dict["left_arm_joints"].append(la_joints[r_idx_curr])
        data_dict["right_arm_joints"].append(ra_joints[r_idx_curr])
        data_dict["left_hand_joints"].append(lh_joints[r_idx_curr])
        data_dict["right_hand_joints"].append(rh_joints[r_idx_curr])

        data_dict["left_arm_joints_actions"].append(la_joints[r_idx_next])
        data_dict["right_arm_joints_actions"].append(ra_joints[r_idx_next])
        data_dict["left_hand_joints_actions"].append(lh_joints[r_idx_next])
        data_dict["right_hand_joints_actions"].append(rh_joints[r_idx_next])

        data_dict["timestamps"].append(curr_ts)

        if i % 100 == 0 and i > 0:
            print(f"    Processed {i}/{num_valid_frames} frames", end="\r")

    # --- Append hold frames ---
    post_process_append_hold(
        data_dict=data_dict,
        frames_dir=frames_dir,
        num_cams=len(cam_ids),
        n_repeat=HOLD_FRAMES,
    )

    final_num_frames = len(data_dict["timestamps"])
    print(f"\n    Total frames after hold: {final_num_frames}")

    # --- Save aligned HDF5 ---
    output_h5 = os.path.join(episode_path, "aligned_data.h5")
    with h5py.File(output_h5, "w") as f_out:
        # Observations
        f_out.create_dataset(
            "observation/left_arm_joints",
            data=np.array(data_dict["left_arm_joints"], dtype=np.float32),
        )
        f_out.create_dataset(
            "observation/right_arm_joints",
            data=np.array(data_dict["right_arm_joints"], dtype=np.float32),
        )
        f_out.create_dataset(
            "observation/left_hand_joints",
            data=np.array(data_dict["left_hand_joints"], dtype=np.float32),
        )
        f_out.create_dataset(
            "observation/right_hand_joints",
            data=np.array(data_dict["right_hand_joints"], dtype=np.float32),
        )
        # Actions
        f_out.create_dataset(
            "action/left_arm_joints",
            data=np.array(data_dict["left_arm_joints_actions"], dtype=np.float32),
        )
        f_out.create_dataset(
            "action/right_arm_joints",
            data=np.array(data_dict["right_arm_joints_actions"], dtype=np.float32),
        )
        f_out.create_dataset(
            "action/left_hand_joints",
            data=np.array(data_dict["left_hand_joints_actions"], dtype=np.float32),
        )
        f_out.create_dataset(
            "action/right_hand_joints",
            data=np.array(data_dict["right_hand_joints_actions"], dtype=np.float32),
        )
        # Timestamps
        f_out.create_dataset(
            "timestamps",
            data=np.array(data_dict["timestamps"], dtype=np.float64),
        )
        # Metadata
        f_out.attrs["num_frames"] = final_num_frames
        f_out.attrs["fps"] = 30
        f_out.attrs["image_size"] = f"{resize[0]}x{resize[1]}"
        f_out.attrs["num_cameras"] = len(cam_ids)
        f_out.attrs["camera_ids"] = cam_ids
        f_out.attrs["description"] = (
            "Aligned dual-arm dual-hand data. "
            "Actions are states at t+1 (next-step behavior cloning)."
        )

    f_cam.close()
    f_robot.close()
    print(f"  Done! Saved {final_num_frames} aligned frames -> {output_h5}")


def main():
    parser = argparse.ArgumentParser(
        description="Align robot + camera data for dual-arm teleop episodes."
    )
    parser.add_argument(
        "--data_dir", type=str, required=True,
        help="Root directory containing episode* folders."
    )
    parser.add_argument(
        "--start_episode", type=int, default=0,
        help="Start processing from this episode index."
    )
    parser.add_argument(
        "--resize_width", type=int, default=640,
        help="Output image width (default: 640)."
    )
    parser.add_argument(
        "--resize_height", type=int, default=360,
        help="Output image height (default: 360)."
    )
    parser.add_argument(
        "--downsample", type=int, default=1,
        help="Frame downsampling factor (1 = no downsample, 2 = every other frame)."
    )
    args = parser.parse_args()

    resize = (args.resize_width, args.resize_height)

    # Find episode directories
    episode_dirs = sorted(
        glob(os.path.join(args.data_dir, "episode*")),
        key=lambda x: int(
            x.split("episode")[-1]
            if x.split("episode")[-1].isdigit()
            else 0
        ),
    )
    episode_dirs = episode_dirs[args.start_episode:]

    if not episode_dirs:
        print(f"No 'episode*' folders found in {args.data_dir}")
        return

    print(f"Found {len(episode_dirs)} episodes. Starting alignment...")
    print(f"  Resize: {resize[0]}x{resize[1]}, Downsample: {args.downsample}")

    for ep_dir in episode_dirs:
        process_episode(ep_dir, resize=resize, downsample=args.downsample)

    print("\nAll episodes processed.")


if __name__ == "__main__":
    main()
