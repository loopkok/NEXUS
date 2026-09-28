"""Attach the frozen dataset layout to every trained checkpoint directory."""

import argparse
import json
import shutil
from pathlib import Path


def stamp(dataset: Path, root: Path) -> list[str]:
    source = dataset / "nexus_layout.json"
    with source.open(encoding="utf-8") as fh:
        layout = json.load(fh)
    if not layout.get("profile_sha256") or not layout.get("components"):
        raise ValueError("dataset NEXUS layout incomplete")
    if not root.is_dir():
        raise FileNotFoundError(root)
    targets = [root, *root.glob("**/pretrained_model")]
    for target in targets:
        path = target / "nexus_layout.json"
        if path.exists():
            with path.open(encoding="utf-8") as fh:
                existing = json.load(fh)
            if existing != layout:
                raise ValueError(f"checkpoint layout differs: {path}")
        else:
            shutil.copyfile(source, path)
    return [str(t / "nexus_layout.json") for t in targets]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--checkpoint-root", required=True)
    args = parser.parse_args()
    for path in stamp(Path(args.dataset), Path(args.checkpoint_root)):
        print(path)


if __name__ == "__main__":
    main()
