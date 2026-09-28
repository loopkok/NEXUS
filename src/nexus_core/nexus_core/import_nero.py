"""Import legacy Nero raw HDF5 into frozen NEXUS episodes without modifying source."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
from pathlib import Path

import h5py
import numpy as np

from .profile import Profile, ProfileError


_FIELDS = {
    "left_arm": "left_arm/joints",
    "right_arm": "right_arm/joints",
    "left_ee": "left_hand/joints",
    "right_ee": "right_hand/joints",
}


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def import_episode(source: Path, output: Path, profile: Profile,
                   camera_map: dict[str, str], task: str) -> dict:
    from astral_data_collect.data_writer import CameraDataWriter, StreamDataWriter
    from astral_data_collect.schema import NexusCollectSchema

    if profile.raw["robot"] != "nero":
        raise ProfileError("legacy Nero import requires a Nero profile")
    if output.exists():
        raise FileExistsError(output)
    schema = NexusCollectSchema(profile)
    if set(camera_map) != set(schema.cameras) or len(set(camera_map.values())) != len(camera_map):
        raise ProfileError("camera map must name every profile role and distinct legacy camera ID")
    robot_path = source / "robot_data.h5"
    camera_path = source / "camera_data.h5"
    if not robot_path.is_file() or not camera_path.is_file():
        raise FileNotFoundError(f"missing legacy robot_data.h5/camera_data.h5 in {source}")
    # Refuse ambiguous or truncated source before creating any output.
    with h5py.File(robot_path, "r") as robot, h5py.File(camera_path, "r") as camera:
        if "timestamps" not in robot:
            raise ValueError("legacy robot timestamps absent")
        n = len(robot["timestamps"])
        if n < 2:
            raise ValueError("legacy robot episode has fewer than 2 frames")
        for component, field in _FIELDS.items():
            spec = profile.component(component)
            if field not in robot or robot[field].shape != (n, spec.dim):
                raise ValueError(f"legacy {field} shape mismatch: expected {(n, spec.dim)}")
        for role, old_id in camera_map.items():
            if old_id not in camera or "images" not in camera[old_id] or "timestamps" not in camera[old_id]:
                raise ValueError(f"legacy camera {old_id} for role {role} absent")
            if len(camera[old_id]["images"]) != len(camera[old_id]["timestamps"]):
                raise ValueError(f"legacy camera {old_id} images/timestamps mismatch")
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f".{output.name}_", dir=output.parent))
    try:
        writer = StreamDataWriter(str(stage / "robot_data.h5"), schema.required_streams()).open()
        camera_writer = CameraDataWriter(str(stage / "camera_data.h5"), schema.cameras).open()
        with h5py.File(robot_path, "r") as robot, h5py.File(camera_path, "r") as camera:
            ts = robot["timestamps"][:]
            if not np.isfinite(ts).all() or np.any(np.diff(ts) <= 0):
                raise ValueError("legacy robot timestamps invalid or not strictly increasing")
            for component, field in _FIELDS.items():
                samples = robot[field][:]
                if not np.isfinite(samples).all():
                    raise ValueError(f"legacy {field} has NaN/Inf")
                for t, values in zip(ts, samples):
                    writer.write(f"{component}_state", values, float(t))
                    # Legacy raw stores observations only. Duplicate into the
                    # unused command channel to keep raw schema complete; the
                    # frozen action_source=next_state never consumes this echo.
                    writer.write(f"{component}_cmd", values, float(t))
            for role, old_id in camera_map.items():
                group = camera[old_id]
                cam_ts = group["timestamps"][:]
                if len(cam_ts) < 2 or not np.isfinite(cam_ts).all() or np.any(np.diff(cam_ts) <= 0):
                    raise ValueError(f"legacy camera {old_id} timestamps invalid")
                for t, image in zip(cam_ts, group["images"]):
                    camera_writer.write_image(role, bytes(image), float(t))
        streams = writer.close()
        cameras = camera_writer.close()
        report = {
            "format": "nexus_legacy_nero_import_v1",
            "source": str(source.resolve()),
            "source_sha256": {"robot_data.h5": _sha(robot_path), "camera_data.h5": _sha(camera_path)},
            "profile_id": profile.profile_id, "profile_sha256": profile.digest,
            "field_map": _FIELDS, "camera_map": camera_map,
            "synthetic_command_streams": True,
            "streams": streams, "cameras": cameras,
        }
        (stage / "conversion_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        meta = {"package": "nexus_core", "format": "raw_hdf5_v1",
                "task": task, "session": output.parent.name,
                "duration_s": float(ts[-1] - ts[0]),
                "schema": schema.to_dict(), "episode_index": int(''.join(filter(str.isdigit, output.name)) or 0),
                "streams": {name: {"dim": dim, "count": streams[name]}
                            for name, dim in schema.required_streams().items()},
                "cameras": {name: {"count": cameras[name]} for name in schema.cameras},
                "import_report": "conversion_report.json"}
        (stage / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        stage.rename(output)
        return report
    except BaseException:
        shutil.rmtree(stage)
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--source", required=True, help="legacy episode directory")
    parser.add_argument("--output", required=True, help="new NEXUS episode directory")
    parser.add_argument("--task", default="")
    parser.add_argument("--camera-map", required=True,
                        help='JSON role->legacy ID, e.g. {"base":"cam_0","left_wrist":"cam_1","right_wrist":"cam_2"}')
    args = parser.parse_args()
    report = import_episode(Path(args.source), Path(args.output), Profile.load(args.profile),
                            json.loads(args.camera_map), args.task)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
