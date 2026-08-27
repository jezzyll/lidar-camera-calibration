#!/usr/bin/env python3
"""Audit Phase 1 raw-capture manifests and checksums without processing data.

This tool intentionally performs no camera-LiDAR calibration, projection,
point selection, or reprojection-error calculation. It is a raw-submission
gate only.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROTOCOL = REPO_ROOT / "config/phase1_new_capture_protocol.json"
DEFAULT_FREEZE = REPO_ROOT / "FROZEN_PHASE1_EVALUATOR.json"

SESSION_COLUMNS = [
    "capture_id", "split", "session_id", "protocol_version",
    "split_manifest_sha256", "distance_target_ft", "distance_measured_m",
    "pose_id", "take_number", "position_label", "center_u_fraction",
    "center_v_fraction", "yaw_deg", "pitch_deg", "roll_deg", "board_id",
    "rig_id", "raw_db3_relative_path", "metadata_relative_path", "start_utc",
    "duration_s", "rgb_topic", "camera_info_topic", "lidar_topic",
    "image_width_px", "image_height_px", "time_sync_mode", "d455_serial",
    "livox_serial", "operator", "full_board_visible",
    "minimum_edge_margin_px", "minimum_observed_square_pitch_px",
    "complete_rgb_grid_detected_preflight", "camera_frame_count",
    "camera_dropped_frames", "lidar_packet_count", "lidar_dropped_packets",
    "sha256_db3", "sha256_metadata", "qa_decision", "qa_reason", "notes",
]

RIG_COLUMNS = [
    "rig_id", "measurement_repeat", "measurement_utc", "d455_serial",
    "livox_serial", "reference_frame", "baseline_x_m", "baseline_y_m",
    "baseline_z_m", "baseline_norm_m", "measurement_uncertainty_m",
    "measurement_tool", "mount_photo_relative_path", "photo_sha256",
    "witness_marks_ok", "operator", "notes",
]

BOARD_COLUMNS = [
    "board_id", "physical_square_columns", "physical_square_rows",
    "inner_corner_columns", "inner_corner_rows", "square_size_x_mean_m",
    "square_size_y_mean_m", "square_size_std_m", "square_size_min_m",
    "square_size_max_m", "board_width_m", "board_height_m",
    "flatness_max_deviation_m", "measurement_uncertainty_m", "material",
    "thickness_m", "finish", "orientation_marker",
    "mount_photo_relative_path", "photo_sha256", "notes",
]

SPLIT_COLUMNS = [
    "capture_id", "split", "session_id", "distance_target_ft", "pose_id",
    "take_number", "protocol_version",
]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_csv(path: Path, required_columns: list[str], errors: list[str]):
    if not path.is_file():
        errors.append(f"missing manifest: {path}")
        return []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        actual = reader.fieldnames or []
        missing = [column for column in required_columns if column not in actual]
        if missing:
            errors.append(f"{path}: missing columns {missing}")
        return list(reader)


def safe_relative(root: Path, value: str, label: str, errors: list[str]):
    if not value:
        errors.append(f"{label}: empty relative path")
        return None
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        errors.append(f"{label}: path must be relative and cannot contain '..': {value}")
        return None
    resolved = (root / relative).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError:
        errors.append(f"{label}: path escapes submission root: {value}")
        return None
    return resolved


def number(row: dict, field: str, label: str, errors: list[str]):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError):
        errors.append(f"{label}: {field} must be numeric")
        return None
    if not math.isfinite(value):
        errors.append(f"{label}: {field} must be finite")
        return None
    return value


def integer(row: dict, field: str, label: str, errors: list[str]):
    value = number(row, field, label, errors)
    if value is None:
        return None
    if not value.is_integer():
        errors.append(f"{label}: {field} must be an integer")
        return None
    return int(value)


def truth(value: str) -> bool:
    return str(value).strip().lower() in {"true", "yes", "1", "pass"}


def verify_declared_file(root: Path, relative: str, declared_hash: str,
                         label: str, errors: list[str]):
    path = safe_relative(root, relative, label, errors)
    if path is None:
        return
    if not path.is_file():
        errors.append(f"{label}: missing file {relative}")
        return
    actual = sha256_file(path)
    if actual.lower() != str(declared_hash).strip().lower():
        errors.append(
            f"{label}: SHA-256 mismatch for {relative}: expected "
            f"{declared_hash}, measured {actual}"
        )


def verify_frozen(manifest_path: Path, errors: list[str]):
    manifest = json.loads(manifest_path.read_text())
    for group in ("source_files", "evidence_files"):
        for item in manifest[group]:
            path = REPO_ROOT / item["path"]
            if not path.is_file():
                errors.append(f"frozen {group}: missing {item['path']}")
                continue
            actual = sha256_file(path)
            if actual != item["sha256"]:
                errors.append(
                    f"frozen {group}: hash mismatch for {item['path']}: "
                    f"expected {item['sha256']}, measured {actual}"
                )
    return manifest


def check_template_headers(errors: list[str]):
    templates = [
        (REPO_ROOT / "config/phase1_capture_sessions_template.csv", SESSION_COLUMNS),
        (REPO_ROOT / "config/phase1_rig_measurement_template.csv", RIG_COLUMNS),
        (REPO_ROOT / "config/phase1_board_inventory_template.csv", BOARD_COLUMNS),
        (REPO_ROOT / "config/phase1_split_assignment_template.csv", SPLIT_COLUMNS),
    ]
    for path, expected in templates:
        with path.open(newline="") as stream:
            actual = next(csv.reader(stream))
        if actual != expected:
            errors.append(f"template header mismatch: {path}")


def audit_rigs(root: Path, rows: list[dict], errors: list[str]):
    grouped = defaultdict(list)
    for index, row in enumerate(rows, start=2):
        label = f"rig row {index}"
        rig_id = row.get("rig_id", "").strip()
        if not rig_id:
            errors.append(f"{label}: empty rig_id")
            continue
        grouped[rig_id].append(row)
        x = number(row, "baseline_x_m", label, errors)
        y = number(row, "baseline_y_m", label, errors)
        z = number(row, "baseline_z_m", label, errors)
        norm = number(row, "baseline_norm_m", label, errors)
        uncertainty = number(row, "measurement_uncertainty_m", label, errors)
        if None not in (x, y, z, norm, uncertainty):
            measured = math.sqrt(x * x + y * y + z * z)
            if abs(measured - norm) > max(1e-6, uncertainty):
                errors.append(f"{label}: baseline norm is inconsistent with x/y/z")
        if not truth(row.get("witness_marks_ok", "")):
            errors.append(f"{label}: witness_marks_ok must be true")
        verify_declared_file(
            root, row.get("mount_photo_relative_path", ""),
            row.get("photo_sha256", ""), label + " mount photo", errors,
        )
    for rig_id, measurements in grouped.items():
        repeats = {row.get("measurement_repeat", "").strip() for row in measurements}
        if len(repeats) < 3:
            errors.append(f"rig {rig_id}: at least three baseline repeats are required")
    return set(grouped)


def audit_boards(root: Path, rows: list[dict], errors: list[str]):
    board_ids = set()
    for index, row in enumerate(rows, start=2):
        label = f"board row {index}"
        board_id = row.get("board_id", "").strip()
        if not board_id:
            errors.append(f"{label}: empty board_id")
            continue
        if board_id in board_ids:
            errors.append(f"{label}: duplicate board_id {board_id}")
        board_ids.add(board_id)
        columns = integer(row, "physical_square_columns", label, errors)
        rows_count = integer(row, "physical_square_rows", label, errors)
        inner_columns = integer(row, "inner_corner_columns", label, errors)
        inner_rows = integer(row, "inner_corner_rows", label, errors)
        sx = number(row, "square_size_x_mean_m", label, errors)
        sy = number(row, "square_size_y_mean_m", label, errors)
        width = number(row, "board_width_m", label, errors)
        height = number(row, "board_height_m", label, errors)
        uncertainty = number(row, "measurement_uncertainty_m", label, errors)
        if (columns, rows_count, inner_columns, inner_rows) != (12, 18, 11, 17):
            errors.append(f"{label}: expected 12x18 squares and 11x17 inner corners")
        if None not in (columns, rows_count, sx, sy, width, height, uncertainty):
            if abs(width - columns * sx) > max(1e-6, uncertainty):
                errors.append(f"{label}: board_width_m inconsistent with columns*square_x")
            if abs(height - rows_count * sy) > max(1e-6, uncertainty):
                errors.append(f"{label}: board_height_m inconsistent with rows*square_y")
        verify_declared_file(
            root, row.get("mount_photo_relative_path", ""),
            row.get("photo_sha256", ""), label + " board photo", errors,
        )
    return board_ids


def nearest_nominal(value: float, nominal: list[float], tolerance: float):
    closest = min(nominal, key=lambda candidate: abs(candidate - value))
    return closest if abs(closest - value) <= tolerance else None


def audit_split_manifest(path: Path, split_rows: list[dict], session_rows: list[dict],
                         protocol: dict, errors: list[str]):
    expected_protocol = protocol["schema_version"]
    nominal = protocol["nominal_distances_ft"]
    tolerance = protocol["distance_tolerance_ft"]
    planned = {}
    for index, row in enumerate(split_rows, start=2):
        label = f"split row {index}"
        capture_id = row.get("capture_id", "").strip()
        split = row.get("split", "").strip().lower()
        if not capture_id or capture_id in planned:
            errors.append(f"{label}: capture_id must be non-empty and unique")
            continue
        if split not in {"calibration", "validation"}:
            errors.append(f"{label}: split must be calibration or validation")
        if not row.get("session_id", "").strip():
            errors.append(f"{label}: session_id is required")
        if not row.get("pose_id", "").strip():
            errors.append(f"{label}: pose_id is required")
        if row.get("protocol_version", "").strip() != expected_protocol:
            errors.append(f"{label}: protocol_version must equal {expected_protocol}")
        distance = number(row, "distance_target_ft", label, errors)
        take = integer(row, "take_number", label, errors)
        if distance is not None and nearest_nominal(distance, nominal, tolerance) is None:
            errors.append(f"{label}: distance_target_ft not within declared tolerance")
        if take is not None and take < 1:
            errors.append(f"{label}: take_number must be positive")
        planned[capture_id] = row

    actual = {
        row.get("capture_id", "").strip(): row
        for row in session_rows
        if row.get("split", "").strip().lower() in {"calibration", "validation"}
    }
    missing_sessions = sorted(set(planned) - set(actual))
    missing_assignments = sorted(set(actual) - set(planned))
    if missing_sessions:
        errors.append(f"split manifest has capture IDs absent from sessions: {missing_sessions}")
    if missing_assignments:
        errors.append(f"session rows lack frozen split assignments: {missing_assignments}")

    for capture_id in sorted(set(planned) & set(actual)):
        plan = planned[capture_id]
        session = actual[capture_id]
        comparisons = [
            ("split", plan.get("split", "").strip().lower(), session.get("split", "").strip().lower()),
            ("session_id", plan.get("session_id", "").strip(), session.get("session_id", "").strip()),
            ("pose_id", plan.get("pose_id", "").strip(), session.get("pose_id", "").strip()),
            ("take_number", plan.get("take_number", "").strip(), session.get("take_number", "").strip()),
            ("protocol_version", plan.get("protocol_version", "").strip(), session.get("protocol_version", "").strip()),
        ]
        for field, expected, measured in comparisons:
            if expected != measured:
                errors.append(
                    f"capture {capture_id}: {field} differs from frozen split "
                    f"({expected!r} != {measured!r})"
                )
        try:
            distance_delta = abs(
                float(plan.get("distance_target_ft", ""))
                - float(session.get("distance_target_ft", ""))
            )
            if distance_delta > 1e-9:
                errors.append(f"capture {capture_id}: distance differs from frozen split")
        except ValueError:
            errors.append(f"capture {capture_id}: invalid distance in split/session comparison")

    actual_hash = sha256_file(path)
    declared_hashes = {
        row.get("split_manifest_sha256", "").strip().lower()
        for row in session_rows
        if row.get("split", "").strip().lower() in {"calibration", "validation"}
    }
    if declared_hashes != {actual_hash}:
        errors.append(
            "all calibration/validation session rows must declare the exact SHA-256 "
            f"of manifests/split_assignment.csv ({actual_hash})"
        )
    return {
        "split_manifest_sha256": actual_hash,
        "frozen_assignment_rows": len(split_rows),
    }


def audit_sessions(root: Path, rows: list[dict], protocol: dict,
                   rig_ids: set[str], board_ids: set[str], errors: list[str]):
    nominal = protocol["nominal_distances_ft"]
    tolerance = protocol["distance_tolerance_ft"]
    minimum_poses = protocol["minimum_unique_pose_ids_per_distance"]
    minimum_takes = int(protocol["minimum_takes_per_pose"])
    minimum_duration = float(protocol["minimum_stable_duration_s_per_take"])
    required_topics = protocol["required_topics"]
    required_width, required_height = protocol["required_image_resolution_wh"]
    expected_protocol = protocol["schema_version"]
    seen_capture = set()
    seen_raw = set()
    session_ids = defaultdict(set)
    pose_takes = defaultdict(lambda: defaultdict(set))
    distance_rows = Counter()

    for index, row in enumerate(rows, start=2):
        label = f"capture row {index}"
        capture_id = row.get("capture_id", "").strip()
        split = row.get("split", "").strip().lower()
        session_id = row.get("session_id", "").strip()
        if not capture_id or capture_id in seen_capture:
            errors.append(f"{label}: capture_id must be non-empty and unique")
        seen_capture.add(capture_id)
        if split not in {"preflight", "calibration", "validation"}:
            errors.append(f"{label}: split must be preflight/calibration/validation")
            continue
        if not session_id:
            errors.append(f"{label}: empty session_id")
        session_ids[split].add(session_id)
        if row.get("protocol_version", "").strip() != expected_protocol:
            errors.append(f"{label}: protocol_version must equal {expected_protocol}")
        target = number(row, "distance_target_ft", label, errors)
        measured_m = number(row, "distance_measured_m", label, errors)
        take = integer(row, "take_number", label, errors)
        duration = number(row, "duration_s", label, errors)
        if target is not None:
            mapped = nearest_nominal(target, nominal, tolerance)
            if mapped is None:
                errors.append(f"{label}: distance_target_ft not within declared tolerance")
            elif split in {"calibration", "validation"}:
                pose_id = row.get("pose_id", "").strip()
                if not pose_id:
                    errors.append(f"{label}: empty pose_id")
                elif take is not None:
                    pose_takes[(split, mapped)][pose_id].add(take)
                    distance_rows[(split, mapped)] += 1
        if measured_m is not None and measured_m <= 0:
            errors.append(f"{label}: distance_measured_m must be positive")
        if duration is not None and duration < minimum_duration:
            errors.append(f"{label}: duration_s must be at least {minimum_duration}")
        if row.get("rig_id", "").strip() not in rig_ids:
            errors.append(f"{label}: unknown rig_id")
        if row.get("board_id", "").strip() not in board_ids:
            errors.append(f"{label}: unknown board_id")
        if row.get("rgb_topic", "").strip() != required_topics[0]:
            errors.append(f"{label}: incorrect rgb_topic")
        if row.get("camera_info_topic", "").strip() != required_topics[1]:
            errors.append(f"{label}: incorrect camera_info_topic")
        if row.get("lidar_topic", "").strip() != required_topics[2]:
            errors.append(f"{label}: incorrect lidar_topic")
        if integer(row, "image_width_px", label, errors) != required_width:
            errors.append(f"{label}: image_width_px must equal {required_width}")
        if integer(row, "image_height_px", label, errors) != required_height:
            errors.append(f"{label}: image_height_px must equal {required_height}")
        if not truth(row.get("full_board_visible", "")):
            errors.append(f"{label}: full_board_visible must be true")
        if row.get("qa_decision", "").strip().upper() != "PASS":
            errors.append(f"{label}: qa_decision must be PASS before submission")
        raw_relative = row.get("raw_db3_relative_path", "").strip()
        if raw_relative in seen_raw:
            errors.append(f"{label}: raw DB3 path is reused")
        seen_raw.add(raw_relative)
        if split in {"calibration", "validation"} and split not in Path(raw_relative).parts:
            errors.append(f"{label}: raw path must be stored under raw/{split}/")
        verify_declared_file(
            root, raw_relative, row.get("sha256_db3", ""),
            label + " raw DB3", errors,
        )
        verify_declared_file(
            root, row.get("metadata_relative_path", ""),
            row.get("sha256_metadata", ""), label + " metadata", errors,
        )
        if split == "preflight":
            pitch = number(row, "minimum_observed_square_pitch_px", label, errors)
            if pitch is not None and pitch < 5.0:
                errors.append(f"{label}: preflight minimum square pitch must be >=5 px")
            if not truth(row.get("complete_rgb_grid_detected_preflight", "")):
                errors.append(f"{label}: preflight must resolve a complete 11x17 grid")

    if session_ids["calibration"] & session_ids["validation"]:
        errors.append("calibration and validation session IDs overlap")
    for split in ("calibration", "validation"):
        for distance in nominal:
            poses = pose_takes[(split, distance)]
            required_pose_count = int(minimum_poses[split])
            if len(poses) < required_pose_count:
                errors.append(
                    f"{split} {distance:g} ft: {len(poses)} unique poses; "
                    f"need {required_pose_count}"
                )
            for pose_id, takes in poses.items():
                if len(takes) < minimum_takes:
                    errors.append(
                        f"{split} {distance:g} ft {pose_id}: "
                        f"{len(takes)} takes; need {minimum_takes}"
                    )
    return {
        "capture_rows": len(rows),
        "calibration_session_ids": sorted(session_ids["calibration"]),
        "validation_session_ids": sorted(session_ids["validation"]),
        "rows_by_split_distance": {
            f"{split}:{distance:g}": count
            for (split, distance), count in sorted(distance_rows.items())
        },
    }


def main():
    parser = argparse.ArgumentParser(
        description="Audit raw Phase 1 capture manifests/checksums only; no processing."
    )
    parser.add_argument("submission_root", type=Path, nargs="?")
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--freeze-manifest", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--verify-frozen", action="store_true")
    parser.add_argument("--check-templates", action="store_true")
    parser.add_argument("--write-report", type=Path)
    args = parser.parse_args()
    errors: list[str] = []
    warnings: list[str] = []

    protocol = json.loads(args.protocol.read_text())
    freeze = None
    if args.verify_frozen:
        freeze = verify_frozen(args.freeze_manifest, errors)
    if args.check_templates:
        check_template_headers(errors)

    summary = {}
    if args.submission_root is not None:
        root = args.submission_root.resolve()
        if root == (REPO_ROOT / "data/raw").resolve():
            errors.append("new submission root must not overwrite data/raw")
        rig_rows = read_csv(root / "manifests/rig_measurement.csv", RIG_COLUMNS, errors)
        board_rows = read_csv(root / "manifests/board_inventory.csv", BOARD_COLUMNS, errors)
        session_rows = read_csv(root / "manifests/capture_sessions.csv", SESSION_COLUMNS, errors)
        split_path = root / "manifests/split_assignment.csv"
        split_rows = read_csv(split_path, SPLIT_COLUMNS, errors)
        if not rig_rows:
            errors.append("rig_measurement.csv has no data rows")
        if not board_rows:
            errors.append("board_inventory.csv has no data rows")
        if not session_rows:
            errors.append("capture_sessions.csv has no data rows")
        if not split_rows:
            errors.append("split_assignment.csv has no data rows")
        rig_ids = audit_rigs(root, rig_rows, errors) if rig_rows else set()
        board_ids = audit_boards(root, board_rows, errors) if board_rows else set()
        if session_rows:
            summary = audit_sessions(
                root, session_rows, protocol, rig_ids, board_ids, errors,
            )
        if split_rows and session_rows:
            summary.update(
                audit_split_manifest(
                    split_path, split_rows, session_rows, protocol, errors,
                )
            )
    elif not args.check_templates:
        parser.error("submission_root is required unless --check-templates is used")

    report = {
        "audit_type": "RAW_SUBMISSION_ONLY",
        "processing_performed": False,
        "calibration_or_accuracy_claim_generated": False,
        "protocol_schema_version": protocol["schema_version"],
        "frozen_evaluator": None if freeze is None else freeze["git_tag"],
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "warnings": warnings,
        "summary": summary,
    }
    rendered = json.dumps(report, indent=2) + "\n"
    if args.write_report:
        args.write_report.parent.mkdir(parents=True, exist_ok=True)
        args.write_report.write_text(rendered)
    print(rendered, end="")
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
