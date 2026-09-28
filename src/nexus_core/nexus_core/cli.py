"""Offline profile inspection and model compatibility gate."""

import argparse
import json
from .profile import Profile, verify_model_manifest


def main() -> None:
    parser = argparse.ArgumentParser(prog="nexus_profile")
    parser.add_argument("profile")
    parser.add_argument("--model-manifest")
    args = parser.parse_args()
    profile = Profile.load(args.profile)
    if args.model_manifest:
        with open(args.model_manifest, encoding="utf-8") as fh:
            verify_model_manifest(profile, json.load(fh))
    print(json.dumps(profile.frozen_schema(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
