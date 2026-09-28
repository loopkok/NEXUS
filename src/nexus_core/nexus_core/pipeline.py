"""Offline NEXUS processing entry point, intended to run on the GPU host."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .profile import Profile, ProfileError


def verify_session(session: Path, profile: Profile) -> list[Path]:
    from astral_data_collect.schema import NexusCollectSchema

    episodes = sorted(p for p in session.iterdir() if p.is_dir() and p.name.startswith("episode"))
    if not episodes:
        raise ProfileError(f"no episodes in {session}")
    for ep in episodes:
        with (ep / "meta.json").open(encoding="utf-8") as fh:
            meta = json.load(fh)
        schema = meta["schema"]
        frozen = NexusCollectSchema.from_dict(schema)
        digest = schema.get("profile_sha256")
        if digest != profile.digest:
            raise ProfileError(f"{ep.name}: episode profile {digest} != selected {profile.digest}")
        if schema.get("state_dim") != profile.dimension:
            raise ProfileError(f"{ep.name}: state dimension mismatch")
        if schema.get("cameras") != [c["role"] for c in profile.raw["cameras"]]:
            raise ProfileError(f"{ep.name}: camera order mismatch")
        if frozen.state_names() != [joint for component in profile.components
                                   for joint in component.joints]:
            raise ProfileError(f"{ep.name}: joint order mismatch")
        if frozen.action_source != profile.raw["dataset"]["action_source"] or frozen.dataset_fps != profile.raw["dataset"]["fps"]:
            raise ProfileError(f"{ep.name}: action semantics or frame rate mismatch")
    return episodes


def process(session: Path, profile: Profile, pi_output: Path, act_output: Path,
            *, image_size: int = 224) -> dict:
    from astral_data_collect.align_data import align_session
    from astral_data_collect.validate_data import validate_session
    from astral_data_collect.convert_to_lerobot import convert_session
    from astral_data_collect.convert_to_act import run_pipeline

    episodes = verify_session(session, profile)
    align_session(str(session))
    report = validate_session(str(session), apply=True)
    healthy = sorted(p for p in session.iterdir() if p.is_dir() and p.name.startswith("episode")
                     and (p / "aligned_data.h5").is_file())
    if not healthy:
        raise RuntimeError("no healthy aligned episodes after quality quarantine; inspect "
                           f"{session / 'validation_report.json'}")
    pi_output.mkdir(parents=True, exist_ok=True)
    pi = convert_session(str(session), str(pi_output),
                         robot_type=profile.profile_id, image_size=image_size)
    act = run_pipeline(str(act_output), v21_root=str(pi_output), overwrite=False)
    manifest = profile.frozen_schema()
    manifest["model_family"] = ["act", "pi05"]
    for output in (pi_output, act_output):
        (output / "nexus_layout.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return {"episodes": len(healthy), "quarantined": report["quarantined"],
            "profile_sha256": profile.digest,
            "pi_dataset": str(pi_output), "act_dataset": str(act_output),
            "quality": report, "pi": pi, "act": act}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--pi-output", required=True)
    parser.add_argument("--act-output", required=True)
    parser.add_argument("--image-size", type=int, default=224)
    args = parser.parse_args()
    profile = Profile.load(args.profile)
    result = process(Path(args.session).expanduser().resolve(), profile,
                     Path(args.pi_output).expanduser().resolve(),
                     Path(args.act_output).expanduser().resolve(),
                     image_size=args.image_size)
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))


if __name__ == "__main__":
    main()
