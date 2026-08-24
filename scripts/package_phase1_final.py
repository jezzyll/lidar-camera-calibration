#!/usr/bin/env python3
"""Package the final Phase 1 report, measured outputs, and reproducible code."""

from __future__ import annotations

import argparse
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


REQUIRED_FILES = [
    Path("deliverables/Phase1_Final_Corrections_Evidence_Report.docx"),
    Path("PHASE1_FINAL_CORRECTIONS.md"),
    Path("MENTOR_HANDOFF.md"),
    Path("README.md"),
    Path("requirements.txt"),
    Path("scripts/calibrate.py"),
    Path("scripts/phase1_required_validation.py"),
    Path("scripts/phase1_final_experiments.py"),
    Path("scripts/checkerboard_mask_evaluation.py"),
    Path("scripts/build_phase1_report.py"),
    Path("scripts/build_phase1_final_report.py"),
    Path("scripts/package_phase1_final.py"),
    Path("scripts/validate.py"),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("deliverables/Phase1_Final_Corrections_Evidence_Package.zip"),
    )
    parser.add_argument(
        "--results",
        type=Path,
        default=Path("outputs/phase1_final"),
    )
    args = parser.parse_args()

    missing = [path for path in REQUIRED_FILES if not path.is_file()]
    if not args.results.is_dir():
        missing.append(args.results)
    if missing:
        raise FileNotFoundError(
            "Missing required package inputs: " + ", ".join(map(str, missing))
        )

    files = list(REQUIRED_FILES)
    files.extend(sorted(path for path in args.results.rglob("*") if path.is_file()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.output, "w", compression=ZIP_DEFLATED, compresslevel=6) as archive:
        archive.writestr(
            "PACKAGE_CONTENTS.txt",
            "Phase 1 final corrections evidence package\n\n"
            "Includes the Word report, all derived numeric/image outputs, data provenance, "
            "split/timestamp evidence, error-vector figures, and reproducible code.\n\n"
            "Raw internal ROS 2 bags and the large public archive are intentionally not "
            "duplicated. data_provenance.json records internal paths and the official public "
            "dataset/download links.\n",
        )
        seen = set()
        for path in files:
            archive_name = path.as_posix()
            if archive_name in seen:
                continue
            archive.write(path, archive_name)
            seen.add(archive_name)

    print(f"Saved {args.output} ({args.output.stat().st_size / 1024 / 1024:.1f} MiB)")


if __name__ == "__main__":
    main()
