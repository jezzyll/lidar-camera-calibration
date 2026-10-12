#!/usr/bin/env python3
"""Create an empty, non-overwriting Phase 1 physical-capture directory."""

from __future__ import annotations

import argparse
import hashlib
import re
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{2,63}$")
COPIES = {
    ROOT / "PHASE1_PHYSICAL_ACQUISITION_PROTOCOL.md": Path("protocol/PHASE1_PHYSICAL_ACQUISITION_PROTOCOL.md"),
    ROOT / "FROZEN_PHASE1_EVALUATOR.json": Path("protocol/FROZEN_PHASE1_EVALUATOR.json"),
    ROOT / "FROZEN_PHASE1_EVALUATOR.md": Path("protocol/FROZEN_PHASE1_EVALUATOR.md"),
    ROOT / "config/phase1_new_capture_protocol.json": Path("protocol/phase1_new_capture_protocol.json"),
    ROOT / "config/phase1_pose_schedule.csv": Path("protocol/phase1_pose_schedule.csv"),
    ROOT / "config/phase1_capture_sessions_template.csv": Path("manifests/capture_sessions.csv"),
    ROOT / "config/phase1_rig_measurement_template.csv": Path("manifests/rig_measurement.csv"),
    ROOT / "config/phase1_board_inventory_template.csv": Path("manifests/board_inventory.csv"),
    ROOT / "config/phase1_split_assignment_template.csv": Path("manifests/split_assignment.csv"),
    ROOT / "PHASE1_RAW_SUBMISSION_README_TEMPLATE.md": Path("RAW_SUBMISSION_README.md"),
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create a blank Phase 1 capture tree; existing targets are never overwritten."
    )
    parser.add_argument("experiment_id")
    parser.add_argument(
        "--parent", type=Path, default=ROOT / "data/new_capture",
        help="Parent directory for the new experiment root",
    )
    args = parser.parse_args()
    if not SAFE_ID.fullmatch(args.experiment_id):
        parser.error("experiment_id must be 3-64 letters, digits, underscores, or hyphens")
    target = (args.parent / args.experiment_id).resolve()
    if target.exists():
        raise FileExistsError(f"Refusing to overwrite existing capture root: {target}")

    directories = [
        "protocol", "manifests", "raw/preflight", "raw/calibration",
        "raw/validation", "photos/rig", "photos/boards", "checksums",
    ]
    for relative in directories:
        (target / relative).mkdir(parents=True, exist_ok=False)
    for source, relative in COPIES.items():
        shutil.copy2(source, target / relative)

    control_files = sorted((target / "protocol").iterdir())
    control_hashes = "".join(
        f"{sha256_file(path)}  {path.relative_to(target).as_posix()}\n"
        for path in control_files if path.is_file()
    )
    (target / "checksums/PROTOCOL_SHA256.txt").write_text(control_hashes)
    (target / "checksums/SHA256SUMS.txt").write_text(
        "# Populate with every raw, metadata, measurement, and photograph file "
        "after acquisition and before submission.\n"
    )
    print(target)
    print("Created blank raw-capture scaffold. Fill and freeze manifests before processing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
