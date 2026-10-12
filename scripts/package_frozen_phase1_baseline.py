#!/usr/bin/env python3
"""Verify and package the immutable Phase 1 evaluator/numeric baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


ROOT = Path(__file__).resolve().parents[1]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--manifest", type=Path,
        default=ROOT / "FROZEN_PHASE1_EVALUATOR.json",
    )
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "deliverables/Phase1_Frozen_Evaluator_Baseline.zip",
    )
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    paths = [
        ROOT / "FROZEN_PHASE1_EVALUATOR.json",
        ROOT / "FROZEN_PHASE1_EVALUATOR.md",
    ]
    for group in ("source_files", "evidence_files"):
        for item in manifest[group]:
            path = ROOT / item["path"]
            if not path.is_file():
                raise FileNotFoundError(path)
            actual = sha256_file(path)
            if actual != item["sha256"]:
                raise RuntimeError(
                    f"Frozen hash mismatch for {item['path']}: "
                    f"expected {item['sha256']}, measured {actual}"
                )
            paths.append(path)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.output, "w", ZIP_DEFLATED, compresslevel=9) as archive:
        archive.writestr(
            "PACKAGE_CONTENTS.txt",
            "Immutable Phase 1 evaluator v1.0.0 source and numeric evidence.\n"
            "Raw DB3 recordings are not duplicated; their SHA-256 hashes are in "
            "FROZEN_PHASE1_EVALUATOR.json.\n",
        )
        seen = set()
        for path in paths:
            name = path.relative_to(ROOT).as_posix()
            if name not in seen:
                archive.write(path, name)
                seen.add(name)
    print(f"Saved {args.output} ({args.output.stat().st_size / 1024 / 1024:.2f} MiB)")


if __name__ == "__main__":
    main()
