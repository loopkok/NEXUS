#!/usr/bin/env python3
"""Convert aligned dual-arm dual-hand data to LeRobotDataset format.

Reads aligned_data.h5 + frames/ from each episode and creates a
LeRobotDataset suitable for training VLA models (e.g., pi0).

Output format matches the standard LeRobot convention:
  - observation.image          : overhead camera (Gemini 335)
  - observation.wrist_image_left  : left wrist camera (Gemini 305)
  - observation.wrist_image_right : right wrist camera (Gemini 305)
  - observation.state          : 38-dim joint state (7+7 arm + 12+12 hand)
  - action                     : 38-dim next-step action
  - task                       : per-frame language instruction

Usage:
    python convert_to_lerobot.py \
        --data_dir ~/xnero_data/Data/my_task \
        --repo_id my_task_lerobot \
        --task_description "pick and place with two arms"
"""

import os
import argparse
import re
import shutil
from pathlib import Path

import h5py
import cv2
import numpy as np
from tqdm import tqdm


def convert_dataset(
    data_dir: str,
    output_root: str,
    repo_id: str,
    task_description: str = "dual arm manipulation",
    fps: int = 30,
    robot_type: str = "nero_xhand",
):
    """Convert aligned episodes to LeRobotDataset format.

    State dim: 38 = 7 (left arm) + 7 (right arm) + 12 (left hand) + 12 (right hand)
    """
    # ---- Import LeRobot ----
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        try:
            from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        except ImportError:
            print("ERROR: lerobot not installed.")
            print("  pip install lerobot")
            return

    data_path = Path(data_dir)
    output_path = Path(output_root) / repo_id

    print(f"Source data: {data_path}")
    print(f"Output dataset: {output_path}")

    # ---- Clean old dataset ----
    if output_path.exists():
        print(f"Removing existing dataset at {output_path}")
        shutil.rmtree(output_path)

    # ---- Find episodes ----
    demo_paths = sorted(
        list(data_path.glob("episode*")),
        key=lambda p: int(re.search(r"(\d+)", p.name).group(1))
        if re.search(r"(\d+)", p.name) else 0,
    )

    if not demo_paths:
        print(f"No episode* folders found in {data_dir}")
        return

    print(f"Found {len(demo_paths)} episodes.")

    # ---- Detect image dimensions ----
    try:
        first_ep = demo_paths[0]
        first_img = sorted(
            list((first_ep / "frames" / "cam0").glob("*.jpg"))
        )[0]
        first_image = cv2.imread(str(first_img))
        if first_image is None:
            raise IOError(f"Cannot read {first_img}")
        img_shape = first_image.shape  # (H, W, 3)
        print(f"Detected image size: {img_shape}")
    except (IndexError, IOError) as e:
        print(f"ERROR detecting image size: {e}")
        return

    H, W = img_shape[:2]

    # ---- Define features (LeRobot standard format) ----
    features = {
        "observation.image": {
            "dtype": "image",
            "shape": (H, W, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.wrist_image_left": {
            "dtype": "image",
            "shape": (H, W, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.wrist_image_right": {
            "dtype": "image",
            "shape": (H, W, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (38,),
            "names": ["state"],
        },
        "action": {
            "dtype": "float32",
            "shape": (38,),
            "names": ["action"],
        },
    }

    # ---- Create empty dataset ----
    print("Creating LeRobotDataset...")
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=output_path,
        robot_type=robot_type,
        fps=fps,
        features=features,
        image_writer_threads=8,
        image_writer_processes=4,
    )
    print(f"Dataset created (robot_type={robot_type}, fps={fps}).")

    # ---- Camera mapping: frames/cam{0,1,2} -> LeRobot keys ----
    camera_feature_map = {
        "cam0": "observation.image",
        "cam1": "observation.wrist_image_left",
        "cam2": "observation.wrist_image_right",
    }
    # Also accept new naming from alignment
    camera_aliases = {"cam_0": "cam0", "cam_1": "cam1", "cam_2": "cam2"}

    # ---- Process each episode ----
    for episode_idx, demo_path in enumerate(
        tqdm(demo_paths, desc="Converting episodes")
    ):
        h5_path = demo_path / "aligned_data.h5"
        frames_dir = demo_path / "frames"

        if not h5_path.exists():
            print(f"  Skip {demo_path.name}: no aligned_data.h5")
            continue

        with h5py.File(h5_path, "r") as hf:
            # Read actions and observations
            la_actions = hf["action/left_arm_joints"][:]
            ra_actions = hf["action/right_arm_joints"][:]
            lh_actions = hf["action/left_hand_joints"][:]
            rh_actions = hf["action/right_hand_joints"][:]

            la_states = hf["observation/left_arm_joints"][:]
            ra_states = hf["observation/right_arm_joints"][:]
            lh_states = hf["observation/left_hand_joints"][:]
            rh_states = hf["observation/right_hand_joints"][:]

            num_frames = la_actions.shape[0]

            # Find camera directories and map them to LeRobot keys
            cam_dirs = sorted([d for d in frames_dir.iterdir() if d.is_dir()])
            cam_images = {}
            for cam_dir in cam_dirs:
                raw_name = cam_dir.name
                # Normalize cam_0 -> cam0 etc.
                name = camera_aliases.get(raw_name, raw_name)
                feature_key = camera_feature_map.get(name)
                if feature_key:
                    images = sorted(list(cam_dir.glob("*.jpg")))
                    cam_images[feature_key] = images

            available_cams = list(cam_images.keys())
            print(f"  {demo_path.name}: {num_frames} frames, "
                  f"cameras: {available_cams}")

            for i in range(num_frames):
                # Build 38-dim state and action
                state = np.concatenate([
                    la_states[i].ravel()[:7],
                    ra_states[i].ravel()[:7],
                    lh_states[i].ravel()[:12],
                    rh_states[i].ravel()[:12],
                ]).astype(np.float32)

                action = np.concatenate([
                    la_actions[i].ravel()[:7],
                    ra_actions[i].ravel()[:7],
                    lh_actions[i].ravel()[:12],
                    rh_actions[i].ravel()[:12],
                ]).astype(np.float32)

                # Ensure exact dimensions
                if len(state) != 38:
                    state = np.pad(state[:38], (0, max(0, 38 - len(state))))[:38]
                if len(action) != 38:
                    action = np.pad(action[:38], (0, max(0, 38 - len(action))))[:38]

                frame = {
                    "observation.state": state,
                    "action": action,
                    "task": task_description,
                }

                # Add camera images
                for feature_key, imgs in cam_images.items():
                    if i < len(imgs):
                        img = cv2.imread(str(imgs[i]))
                        if img is not None:
                            frame[feature_key] = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                # Fill missing cameras with black
                for feature_key in features:
                    if (feature_key not in frame
                            and feature_key not in ("observation.state", "action")
                            and features[feature_key].get("dtype") == "image"):
                        frame[feature_key] = np.zeros((H, W, 3), dtype=np.uint8)

                dataset.add_frame(frame)

            dataset.save_episode()

    print(f"\nAll {len(demo_paths)} episodes converted and saved.")
    print(f"LeRobot dataset at: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert aligned data to LeRobotDataset format."
    )
    parser.add_argument(
        "--data_dir", type=str, required=True,
        help="Directory containing aligned episode* folders."
    )
    parser.add_argument(
        "--output_dir", type=str, default=None,
        help="Root directory for LeRobot dataset (default: data_dir parent)."
    )
    parser.add_argument(
        "--repo_id", type=str, required=True,
        help="LeRobot dataset repository ID."
    )
    parser.add_argument(
        "--task_description", type=str,
        default="dual arm manipulation",
        help="Task description string for the dataset."
    )
    parser.add_argument(
        "--fps", type=int, default=30,
        help="Frames per second for the dataset."
    )
    parser.add_argument(
        "--robot_type", type=str, default="nero_xhand",
        help="Robot type string for the dataset (e.g., nero_xhand)."
    )
    args = parser.parse_args()

    output_dir = args.output_dir or os.path.dirname(args.data_dir)
    convert_dataset(
        data_dir=args.data_dir,
        output_root=output_dir,
        repo_id=args.repo_id,
        task_description=args.task_description,
        fps=args.fps,
        robot_type=args.robot_type,
    )


if __name__ == "__main__":
    main()
