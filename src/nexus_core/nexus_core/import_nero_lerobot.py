"""Import Nero's legacy LeRobot episodes into frozen NEXUS raw episodes.

The old Nero exporter used state order left arm, right arm, left XHand,
right XHand and camera features observation.image, wrist_image_left/right.
Every mapping is checked before a new episode is committed. Source files are
opened read-only and are never rewritten.
"""

from __future__ import annotations

import argparse
import io
import json
import math
import shutil
import tempfile
from pathlib import Path

import numpy as np

from .profile import Profile, ProfileError


LEGACY_CAMERA_FEATURES = (
    "observation.image",
    "observation.wrist_image_left",
    "observation.wrist_image_right",
)


def _scalar(value) -> float:
    return float(np.asarray(value).reshape(-1)[0])


def _jpeg(value) -> bytes:
    from PIL import Image

    image = np.asarray(value)
    if image.ndim != 3:
        raise ValueError(f"camera image must have three axes, got {image.shape}")
    if image.shape[0] in (1, 3, 4) and image.shape[-1] not in (1, 3, 4):
        image = np.moveaxis(image, 0, -1)
    if image.dtype.kind == "f":
        if not np.isfinite(image).all():
            raise ValueError("camera image contains NaN/Inf")
        image = np.uint8(np.clip(image * 255.0, 0, 255))
    else:
        image = np.uint8(image)
    if image.shape[2] not in (1, 3, 4):
        raise ValueError(f"unsupported image channel count {image.shape[2]}")
    with io.BytesIO() as buffer:
        Image.fromarray(image.squeeze(-1) if image.shape[2] == 1 else image).convert("RGB").save(
            buffer, format="JPEG", quality=95)
        return buffer.getvalue()


def import_dataset(source: Path, output_session: Path, profile: Profile,
                   camera_map: dict[str, str], *, task: str = "",
                   allow_action_rewrite: bool = False, dataset=None) -> list[dict]:
    from astral_data_collect.data_writer import CameraDataWriter, StreamDataWriter
    from astral_data_collect.schema import NexusCollectSchema

    if profile.raw["robot"] != "nero" or profile.dimension != 38:
        raise ProfileError("legacy Nero LeRobot import requires a 38D Nero profile")
    if set(camera_map) != set(profile.frozen_schema()["cameras"]):
        raise ProfileError("camera-map must cover every profile camera role")
    if set(camera_map.values()) != set(LEGACY_CAMERA_FEATURES):
        raise ProfileError("camera-map must use each legacy Nero image feature once")
    if output_session.exists() and any(output_session.iterdir()):
        raise FileExistsError(f"output session must be empty: {output_session}")
    if dataset is None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        dataset = LeRobotDataset(repo_id=source.name, root=source)
    schema = NexusCollectSchema(profile)
    order = [(component.name, component.dim) for component in profile.components]
    if order != [("left_arm", 7), ("right_arm", 7), ("left_ee", 12), ("right_ee", 12)]:
        raise ProfileError("legacy Nero LeRobot joint block order differs from profile")
    if len(dataset) < 2:
        raise ValueError("legacy LeRobot dataset has fewer than two frames")
    ranges: list[tuple[int, int, int]] = []
    last_episode = None
    last_time = -math.inf
    previous = None
    mismatch = []
    lower = np.asarray([value for component in profile.components for value in component.lower])
    upper = np.asarray([value for component in profile.components for value in component.upper])
    for index in range(len(dataset)):
        frame = dataset[index]
        episode = int(_scalar(frame["episode_index"]))
        if last_episode is not None and episode < last_episode:
            raise ValueError("legacy episodes must be ordered by episode_index")
        timestamp = _scalar(frame["timestamp"])
        if not math.isfinite(timestamp):
            raise ValueError(f"frame {index}: timestamp is not finite")
        if episode != last_episode:
            last_time = -math.inf
            ranges.append((episode, index, index + 1))
        else:
            ranges[-1] = (episode, ranges[-1][1], index + 1)
        if timestamp <= last_time:
            raise ValueError(f"frame {index}: timestamps are not strictly increasing")
        last_time, last_episode = timestamp, episode
        state = np.asarray(frame["observation.state"], dtype=np.float64).reshape(-1)
        action = np.asarray(frame["action"], dtype=np.float64).reshape(-1)
        if state.shape != (38,) or action.shape != (38,) or not np.isfinite(state).all() or not np.isfinite(action).all():
            raise ValueError(f"frame {index}: invalid 38D state/action")
        if np.any(state < lower - 0.05) or np.any(state > upper + 0.05) or np.any(action < lower - 0.05) or np.any(action > upper + 0.05):
            raise ValueError(f"frame {index}: state/action outside profile joint limits")
        if previous is not None and previous[0] == episode:
            delta = float(np.max(np.abs(previous[1] - state)))
            if delta > 0.05:
                mismatch.append({"frame": index - 1, "max_error": delta})
        previous = episode, action
    if mismatch and not allow_action_rewrite:
        raise ValueError(f"{len(mismatch)} legacy actions differ from next state; "
                         "pass --allow-action-rewrite to regenerate actions explicitly")
    if any(end - start < 2 for _, start, end in ranges):
        raise ValueError("each legacy LeRobot episode requires at least two frames")
    output_session.mkdir(parents=True, exist_ok=True)
    reports = []
    for episode, start, end in ranges:
        output = output_session / f"episode_{episode:06d}"
        if output.exists():
            raise FileExistsError(output)
        stage = Path(tempfile.mkdtemp(prefix=f".{output.name}_", dir=output_session))
        try:
            writer = StreamDataWriter(str(stage / "robot_data.h5"), schema.required_streams()).open()
            camera_writer = CameraDataWriter(str(stage / "camera_data.h5"), schema.cameras).open()
            first_timestamp = None
            final_timestamp = None
            for index in range(start, end):
                frame = dataset[index]
                timestamp = _scalar(frame["timestamp"])
                first_timestamp = timestamp if first_timestamp is None else first_timestamp
                final_timestamp = timestamp
                state = np.asarray(frame["observation.state"], dtype=np.float64).reshape(-1)
                action = np.asarray(frame["action"], dtype=np.float64).reshape(-1)
                cursor = 0
                for name, dim in order:
                    part = state[cursor:cursor + dim]
                    command = action[cursor:cursor + dim]
                    cursor += dim
                    writer.write(f"{name}_state", part, timestamp)
                    writer.write(f"{name}_cmd", command, timestamp)
                for role in schema.cameras:
                    camera_writer.write_image(role, _jpeg(frame[camera_map[role]]), timestamp)
            streams, cameras = writer.close(), camera_writer.close()
            report = {"format": "nexus_legacy_nero_lerobot_import_v1",
                      "source": str(source.resolve()), "source_episode_index": episode,
                      "frames": end - start, "profile_id": profile.profile_id,
                      "profile_sha256": profile.digest,
                      "state_action_order": [name for name, _ in order],
                      "camera_map": camera_map, "action_source": "next_state",
                      "action_mismatches": [m for m in mismatch if start <= m["frame"] < end],
                      "allow_action_rewrite": allow_action_rewrite,
                      "streams": streams, "cameras": cameras}
            (stage / "conversion_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            meta = {"package": "nexus_core", "format": "raw_hdf5_v1", "task": task,
                    "session": output_session.name, "schema": schema.to_dict(),
                    "duration_s": float(final_timestamp - first_timestamp),
                    "episode_index": episode,
                    "streams": {n: {"dim": d, "count": streams[n]}
                                for n, d in schema.required_streams().items()},
                    "cameras": {n: {"count": cameras[n]} for n in schema.cameras},
                    "import_report": "conversion_report.json"}
            (stage / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            stage.rename(output)
            reports.append(report)
        except BaseException:
            shutil.rmtree(stage)
            raise
    return reports


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--source", required=True, help="legacy LeRobot dataset root")
    parser.add_argument("--output-session", required=True, help="new empty NEXUS session directory")
    parser.add_argument("--camera-map", required=True, help="JSON role to legacy image feature")
    parser.add_argument("--task", default="")
    parser.add_argument("--allow-action-rewrite", action="store_true")
    args = parser.parse_args()
    reports = import_dataset(Path(args.source), Path(args.output_session),
                             Profile.load(args.profile), json.loads(args.camera_map),
                             task=args.task, allow_action_rewrite=args.allow_action_rewrite)
    print(json.dumps(reports, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
