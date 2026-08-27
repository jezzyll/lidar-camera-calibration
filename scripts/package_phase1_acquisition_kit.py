#!/usr/bin/env python3
"""Package the Phase 1 raw physical-acquisition handoff kit."""

from __future__ import annotations

import argparse
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


ROOT = Path(__file__).resolve().parents[1]
KIT_FILES = [
    Path("PHASE1_PHYSICAL_ACQUISITION_PROTOCOL.md"),
    Path("FROZEN_PHASE1_EVALUATOR.json"),
    Path("FROZEN_PHASE1_EVALUATOR.md"),
    Path("config/phase1_new_capture_protocol.json"),
    Path("config/phase1_pose_schedule.csv"),
    Path("config/phase1_capture_sessions_template.csv"),
    Path("config/phase1_rig_measurement_template.csv"),
    Path("config/phase1_board_inventory_template.csv"),
    Path("config/phase1_split_assignment_template.csv"),
    Path("PHASE1_RAW_SUBMISSION_README_TEMPLATE.md"),
    Path("scripts/create_phase1_capture_scaffold.py"),
    Path("scripts/validate_phase1_capture_submission.py"),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "deliverables/Phase1_Physical_Acquisition_Kit.zip",
    )
    args = parser.parse_args()
    missing = [path for path in KIT_FILES if not (ROOT / path).is_file()]
    if missing:
        raise FileNotFoundError("Missing kit files: " + ", ".join(map(str, missing)))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.output, "w", ZIP_DEFLATED, compresslevel=9) as archive:
        archive.writestr(
            "START_HERE.txt",
            "Phase 1 physical acquisition kit\n\n"
            "1. Read PHASE1_PHYSICAL_ACQUISITION_PROTOCOL.md.\n"
            "2. Freeze/copy the protocol and templates into a new experiment root.\n"
            "3. Record raw preflight, calibration, and validation bags.\n"
            "4. Fill all manifests and compute SHA-256 checksums.\n"
            "5. Submit raw recordings first. Do not claim a calibration result.\n"
            "6. Run validate_phase1_capture_submission.py as a metadata/checksum gate only.\n",
        )
        for relative in KIT_FILES:
            archive.write(ROOT / relative, relative.as_posix())
    print(f"Saved {args.output} ({args.output.stat().st_size / 1024:.1f} KiB)")


if __name__ == "__main__":
    main()
