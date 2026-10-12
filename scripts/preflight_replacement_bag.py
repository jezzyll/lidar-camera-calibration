#!/usr/bin/env python3
"""Fail-closed preflight for replacement 5-ft and 22-ft ROS2 bags.

This command performs acquisition quality checks only.  It never estimates a
camera--LiDAR extrinsic and never reads checkerboard reprojection residuals.
The implementation contains its own 0.096 m decoding and background-stability
helpers so the active path cannot inherit obsolete checkerboard geometry from
historical experiment scripts.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import sqlite3
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
PITCH_M = 0.096
PATTERN = (6, 3)
PRINTED_CELLS = (7, 4)
MIN_OVERLAP_S = 15.0
MIN_RGB_RATE = 0.95
MAX_DRIFT_P95_DEG = 0.03
MAX_PREDICTED_SIGMA_PX = 1.0
REFERENCE_AUDIT = ROOT / "output/new_9p6_dataset_run3/audit.json"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RANGE = load_module("replacement_preflight_range", ROOT / "scripts/range_based_lidar_target.py")
BOARD_PLANE = load_module("replacement_preflight_board_plane", ROOT / "scripts/board_plane_lidar_extractor.py")
TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


def decode_rgb_message(data: bytes):
    """Decode the recorded rgb8/bgr8 ROS Image payload without resizing."""
    encoding_offset = data.find(b"rgb8")
    encoding = "rgb8"
    if encoding_offset == -1:
        encoding_offset = data.find(b"bgr8")
        encoding = "bgr8"
    if encoding_offset == -1:
        return None
    encoding_length = int.from_bytes(data[encoding_offset - 4:encoding_offset], "little")
    width = int.from_bytes(data[encoding_offset - 8:encoding_offset - 4], "little")
    height = int.from_bytes(data[encoding_offset - 12:encoding_offset - 8], "little")
    data_start = encoding_offset + encoding_length + 9
    raw = data[data_start:data_start + width * height * 3]
    if len(raw) != width * height * 3:
        return None
    image = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3).copy()
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else image


def declared_pointcloud_schema(message):
    datatype_names = {1: "int8", 2: "uint8", 3: "int16", 4: "uint16",
                      5: "int32", 6: "uint32", 7: "float32", 8: "float64"}
    return {
        "height": int(message.height), "width": int(message.width),
        "is_bigendian": bool(message.is_bigendian), "point_step": int(message.point_step),
        "row_step": int(message.row_step), "is_dense": bool(message.is_dense),
        "fields": [{"name": field.name, "offset": int(field.offset),
                    "datatype": datatype_names[int(field.datatype)],
                    "datatype_code": int(field.datatype), "count": int(field.count)}
                   for field in message.fields],
    }


def decode_pointcloud2(message):
    """Decode finite XYZI returns while preserving the recorded schema."""
    schema = declared_pointcloud_schema(message)
    endian = ">" if schema["is_bigendian"] else "<"
    dtype_map = {1: "i1", 2: "u1", 3: f"{endian}i2", 4: f"{endian}u2",
                 5: f"{endian}i4", 6: f"{endian}u4", 7: f"{endian}f4", 8: f"{endian}f8"}
    fields = {field.name: field for field in message.fields}
    required = ("x", "y", "z", "intensity")
    if any(name not in fields for name in required):
        raise ValueError(f"missing required PointCloud2 fields: {required}")
    count = int(message.width * message.height)
    raw = np.asarray(message.data, dtype=np.uint8).reshape(count, message.point_step)
    decoded = {}
    for name in required:
        field = fields[name]
        dtype = np.dtype(dtype_map[int(field.datatype)])
        decoded[name] = raw[:, field.offset:field.offset + dtype.itemsize].copy().view(dtype).ravel()
    points = np.column_stack([decoded[name] for name in required]).astype(float)
    return points[np.isfinite(points).all(axis=1)], schema


def downsample(points, voxel_m=.12):
    keys = np.floor(np.asarray(points) / voxel_m).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return np.asarray(points)[np.sort(indices)]


def background(points, distance_ft):
    """Mask the predeclared target neighborhood; never use reprojection."""
    xyz = np.asarray(points)[:, :3]
    good = np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > .5) & (np.linalg.norm(xyz, axis=1) < 15)
    xyz = xyz[good]
    expected_range = distance_ft * .3048
    radial = np.linalg.norm(xyz, axis=1)
    target = (np.abs(radial - expected_range) < .8) & (xyz[:, 2] > -1.3) & (xyz[:, 2] < 1.2)
    return downsample(xyz[~target])


def normals(points, neighbours=12):
    tree = cKDTree(points)
    indices = tree.query(points, k=min(neighbours, len(points)))[1]
    values = []
    for ids in indices:
        local = points[ids] - points[ids].mean(0)
        _, _, vectors = np.linalg.svd(local, full_matrices=False)
        values.append(vectors[-1])
    return np.asarray(values)


def icp_ptplane(source, destination, iterations=20):
    source = np.asarray(source, float).copy()
    transform = np.eye(4)
    tree = cKDTree(destination)
    destination_normals = normals(destination)
    used, rmse, condition = 0, np.inf, np.inf
    for _ in range(iterations):
        distances, indices = tree.query(source, k=1)
        accepted = distances < .35
        if accepted.sum() < 30:
            break
        p = source[accepted]; q = destination[indices[accepted]]; n = destination_normals[indices[accepted]]
        system = np.c_[np.cross(p, n), n]
        rhs = np.sum(n * (q - p), axis=1)
        singular = np.linalg.svd(system, compute_uv=False)
        condition = float(singular[0] / max(singular[-1], 1e-12))
        delta = np.linalg.lstsq(system, rhs, rcond=None)[0]
        rotation, _ = cv2.Rodrigues(delta[:3]); translation = delta[3:]
        source = (rotation @ source.T).T + translation
        update = np.eye(4); update[:3, :3] = rotation; update[:3, 3] = translation
        transform = update @ transform
        used = int(accepted.sum())
        rmse = float(np.sqrt(np.mean(np.sum(n * ((rotation @ p.T).T + translation - q), axis=1) ** 2)))
        if np.linalg.norm(delta) < 1e-7:
            break
    return transform, rmse, used / max(1, len(source)), condition


def rotation_angle_deg(rotation):
    return float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))))


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def resolve_bag_parts(value: str) -> list[Path]:
    """Resolve a ROS bag folder or one DB3 path, including metadata split files."""
    supplied = Path(value).expanduser().resolve()
    if not supplied.exists():
        raise FileNotFoundError(f"recording does not exist: {supplied}")
    folder = supplied if supplied.is_dir() else supplied.parent
    metadata = folder / "metadata.yaml"
    parts: list[Path] = []
    if metadata.exists():
        doc = yaml.safe_load(metadata.read_text()) or {}
        info = doc.get("rosbag2_bagfile_information", doc)
        names = info.get("relative_file_paths", [])
        parts = []
        for name in names:
            if not str(name).endswith(".db3"):
                continue
            candidate = (folder / name).resolve()
            # Some copied bags in the authoritative dataset had ':' removed by
            # the transfer filesystem while metadata.yaml retained it. Google
            # Drive also sanitizes '*' to '-' in downloaded file names.
            colon_stripped = (folder / str(name).replace(":", "")).resolve()
            drive_sanitized = (folder / str(name).replace(":", "").replace("*", "-")).resolve()
            if not candidate.exists() and colon_stripped.is_file():
                candidate = colon_stripped
            if not candidate.exists() and drive_sanitized.is_file():
                candidate = drive_sanitized
            parts.append(candidate)
        missing = [p for p in parts if not p.is_file()]
        if missing:
            raise FileNotFoundError(f"metadata references missing split files: {missing}")
        if supplied.is_file() and supplied.suffix == ".db3" and supplied not in parts:
            raise ValueError(f"supplied DB3 is not listed by {metadata}")
    if not parts:
        if supplied.is_file() and supplied.suffix == ".db3":
            parts = [supplied]
        elif supplied.is_dir():
            parts = sorted(supplied.glob("*.db3"))
    if not parts:
        raise ValueError(f"no DB3 files found for {supplied}")
    for part in parts:
        with sqlite3.connect(str(part)) as db:
            result = db.execute("PRAGMA integrity_check").fetchone()[0]
            if result != "ok":
                raise ValueError(f"SQLite integrity failure in {part}: {result}")
    return parts


def rows_for_topic(parts: list[Path], topic: str, with_data: bool = False):
    rows = []
    message_type = None
    for part in parts:
        with sqlite3.connect(str(part)) as db:
            found = db.execute("SELECT id,type FROM topics WHERE name=?", (topic,)).fetchone()
            if not found:
                continue
            tid, typ = found
            if message_type is not None and typ != message_type:
                raise ValueError(f"topic type changes across split files for {topic}")
            message_type = typ
            cols = "timestamp,data" if with_data else "timestamp"
            rows.extend(db.execute(f"SELECT {cols} FROM messages WHERE topic_id=?", (tid,)).fetchall())
    rows.sort(key=lambda x: x[0])
    return rows, message_type


def topic_inventory(parts: list[Path]) -> dict:
    out = {}
    for part in parts:
        with sqlite3.connect(str(part)) as db:
            for tid, name, typ in db.execute("SELECT id,name,type FROM topics ORDER BY id"):
                count = db.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,)).fetchone()[0]
                item = out.setdefault(name, {"type": typ, "count": 0})
                if item["type"] != typ:
                    raise ValueError(f"topic type changes across split files for {name}")
                item["count"] += int(count)
    return out


def detect_grid(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(gray, PATTERN, flags=flags)
    if not ok or corners is None or len(corners) != 18:
        return None
    corners = corners.reshape(-1, 2)
    grid = corners.reshape(3, 6, 2)
    adjacency = np.r_[np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel(),
                      np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel()]
    spacing = float(np.median(adjacency))
    if spacing <= 0 or adjacency.max() / spacing > 1.35 or adjacency.min() / spacing < 0.70:
        return None
    return corners


def decode_lidar(rows, message_type):
    decoded = []
    signature = None
    schema = None
    for index, (timestamp, raw) in enumerate(rows):
        msg = TYPESTORE.deserialize_cdr(raw, message_type)
        points, this_schema = decode_pointcloud2(msg)
        this_signature = {
            "is_bigendian": this_schema["is_bigendian"],
            "point_step": this_schema["point_step"],
            "fields": this_schema["fields"],
        }
        if signature is None:
            signature, schema = this_signature, this_schema
        elif signature != this_signature:
            raise ValueError("PointCloud2 schema changes within recording")
        decoded.append({"index": index, "timestamp_ns": int(timestamp), "points": points})
    return decoded, schema, signature


def status(value, evidence=True):
    if not evidence:
        return "UNRESOLVED"
    return "PASS" if bool(value) else "FAIL"


def _load_evidence_json(path: str | None, label: str):
    if not path:
        return None, {"status": "UNRESOLVED", "reason": f"no {label} supplied"}
    p = Path(path).expanduser().resolve()
    if not p.is_file():
        return None, {"status": "UNRESOLVED", "reason": f"{label} does not exist: {p}"}
    try:
        return json.loads(p.read_text()), {"path": str(p), "sha256": sha256(p)}
    except Exception as exc:
        return None, {"status": "UNRESOLVED", "reason": f"invalid {label}: {exc}"}


def board_evidence(path: str | None):
    """Validate independently measured full-board topology and pitch evidence."""
    data, meta = _load_evidence_json(path, "board evidence JSON")
    if data is None:
        return meta
    required = ("physical_cells", "inner_corners", "nominal_pitch_m",
                "horizontal_pitch_measurements_m", "vertical_pitch_measurements_m",
                "checker_width_m", "checker_height_m", "photo_paths")
    missing = [key for key in required if key not in data]
    if missing:
        return {"status": "UNRESOLVED", **meta, "missing_fields": missing,
                "reason": "full checker topology and measured pitch are not documented"}
    photo_paths = [Path(p).expanduser().resolve() for p in data.get("photo_paths", [])]
    missing_photos = [str(p) for p in photo_paths if not p.is_file()]
    h = np.asarray(data.get("horizontal_pitch_measurements_m", []), float)
    v = np.asarray(data.get("vertical_pitch_measurements_m", []), float)
    topology_ok = (list(data.get("physical_cells", [])) == list(PRINTED_CELLS) and
                   list(data.get("inner_corners", [])) == list(PATTERN))
    nominal_ok = abs(float(data.get("nominal_pitch_m", 0.0)) - PITCH_M) <= 1e-9
    measured = np.r_[h, v]
    measurements_ok = bool(len(h) >= 3 and len(v) >= 3 and
                           np.all(np.abs(measured - PITCH_M) <= 0.0005))
    extent_ok = (abs(float(data.get("checker_width_m", 0.0)) - 7 * PITCH_M) <= 0.003 and
                 abs(float(data.get("checker_height_m", 0.0)) - 4 * PITCH_M) <= 0.003)
    photos_ok = bool(photo_paths) and not missing_photos
    passed = topology_ok and nominal_ok and measurements_ok and extent_ok and photos_ok
    return {"status": "PASS" if passed else "FAIL", **meta,
            "physical_cells": data.get("physical_cells"), "inner_corners": data.get("inner_corners"),
            "nominal_pitch_m": data.get("nominal_pitch_m"),
            "horizontal_pitch_measurements_m": h.tolist(),
            "vertical_pitch_measurements_m": v.tolist(),
            "checker_width_m": data.get("checker_width_m"),
            "checker_height_m": data.get("checker_height_m"),
            "photo_paths": [str(p) for p in photo_paths], "missing_photos": missing_photos,
            "pitch_measurement_tolerance_m": 0.0005,
            "reason": ("independent full-board topology, pitch and photographs verified" if passed else
                       "evidence does not verify 7x4 cells, 6x3 inner corners, 0.096 m pitch and 0.672x0.384 m checker area")}


def mounting_evidence(path: str | None, reference_path: str | None = None):
    required = ("camera_serial", "lidar_serial", "rig_locked", "witness_marks_unchanged",
                "reference_configuration_id")
    data, meta = _load_evidence_json(path, "candidate mounting evidence JSON")
    reference, ref_meta = _load_evidence_json(reference_path, "frozen 10/15-ft mounting reference JSON")
    if data is None or reference is None:
        return {"status": "UNRESOLVED", "required_fields": list(required),
                "candidate": meta, "reference": ref_meta,
                "reason": "candidate evidence and an independently frozen 10/15-ft mounting reference are both required"}
    missing = [key for key in required if key not in data]
    ref_missing = [key for key in ("camera_serial", "lidar_serial", "reference_configuration_id") if key not in reference]
    placeholders = [key for key in required if isinstance(data.get(key), str) and
                    (not data.get(key).strip() or "ENTER_" in data.get(key))]
    ref_placeholders = [key for key in ("camera_serial", "lidar_serial", "reference_configuration_id")
                        if isinstance(reference.get(key), str) and
                        (not reference.get(key).strip() or "ENTER_" in reference.get(key))]
    if missing or ref_missing or placeholders or ref_placeholders:
        return {"status": "UNRESOLVED", "candidate": meta, "reference": ref_meta,
                "missing_fields": missing, "reference_missing_fields": ref_missing,
                "placeholder_fields": placeholders, "reference_placeholder_fields": ref_placeholders,
                "reason": "mounting evidence is incomplete or still contains placeholders"}
    matches = {
        "camera_serial": data["camera_serial"] == reference["camera_serial"],
        "lidar_serial": data["lidar_serial"] == reference["lidar_serial"],
        "reference_configuration_id": data["reference_configuration_id"] == reference["reference_configuration_id"],
    }
    passed = (all(matches.values()) and bool(data["rig_locked"]) and
              bool(data["witness_marks_unchanged"]))
    return {"status": "PASS" if passed else "FAIL", "candidate": meta, "reference": ref_meta,
            "matches": matches, "rig_locked": bool(data["rig_locked"]),
            "witness_marks_unchanged": bool(data["witness_marks_unchanged"]),
            "reason": ("candidate serials/configuration match the independently frozen reference and the rig/witness gates pass" if passed else
                       "candidate mounting differs from the frozen reference or lock/witness evidence failed")}


def make_rgb_image(image, corners, detected, total, path):
    canvas = image.copy() if image is not None else np.full((480, 640, 3), 245, np.uint8)
    if corners is not None:
        for index, point in enumerate(corners):
            p = tuple(np.rint(point).astype(int)); cv2.circle(canvas, p, 4, (0, 220, 0), -1)
            cv2.putText(canvas, str(index), (p[0] + 4, p[1] - 4), 0, .35, (0, 0, 180), 1)
    cv2.rectangle(canvas, (0, 0), (639, 48), (0, 0, 0), -1)
    cv2.putText(canvas, f"SOFTWARE PREFLIGHT: complete 6x3 detections {detected}/{total}",
                (6, 20), 0, .45, (255, 255, 255), 1)
    cv2.putText(canvas, "Not calibration evidence; no projection used", (6, 40), 0, .42, (80, 200, 255), 1)
    cv2.imwrite(str(path), canvas)


def make_drift_plot(rows, p95, path):
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    times = [r["time_s"] for r in rows]
    axes[0].plot(times, [r["rotation_deg"] for r in rows], "o-")
    axes[0].axhline(MAX_DRIFT_P95_DEG, color="red", linestyle="--", label="0.03° gate")
    axes[0].set_ylabel("background rotation (deg)"); axes[0].legend(); axes[0].grid(alpha=.3)
    axes[1].plot(times, [1000 * r["translation_m"] for r in rows], "o-")
    axes[1].set_ylabel("background translation (mm)"); axes[1].set_xlabel("recording time (s)"); axes[1].grid(alpha=.3)
    fig.suptitle(f"Target-masked LiDAR background drift • p95 rotation={p95:.4f}°\nNo RGB, intrinsics, extrinsic or reprojection used")
    fig.tight_layout(); fig.savefig(path, dpi=180); plt.close(fig)


def make_lidar_evidence(messages, corners, evidence, distance_ft, path):
    """Use the audited renderer when possible; render an honest empty ROI otherwise."""
    if messages:
        points = np.vstack([m["points"] for m in messages])
        expected = distance_ft * 0.3048
        roi = points[(abs(points[:, 0] - expected) < .5) & (abs(points[:, 1]) < .8) & (abs(points[:, 2]) < 1.2)]
    else:
        roi = np.empty((0, 4))
    if len(roi):
        canvas = np.full((800, 1200, 3), 250, np.uint8)
        centre = np.median(roi[:, :3], axis=0)
        centred = roi[:, :3] - centre
        _, _, axes = np.linalg.svd(centred, full_matrices=False)
        uv = centred @ axes[:2].T
        scale = max(np.ptp(uv[:, 0]), np.ptp(uv[:, 1]), .1)
        x = np.clip((600 + 900 * uv[:, 0] / scale).astype(int), 0, 1199)
        y = np.clip((400 - 600 * uv[:, 1] / scale).astype(int), 0, 799)
        colors = cv2.applyColorMap(np.clip(roi[:, 3], 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO).reshape(-1, 3)
        for xx, yy, color in zip(x, y, colors):
            canvas[yy, xx] = color
        cv2.putText(canvas, f"{distance_ft}ft LiDAR-only radial target evidence", (25, 38), 0, .8, (0, 0, 0), 2)
        cv2.putText(canvas, "Hypotheses only; boundary/origin/polarity not frozen", (25, 775), 0, .65, (0, 0, 210), 2)
        cv2.imwrite(str(path), canvas)
        return
    canvas = np.full((800, 1200, 3), 250, np.uint8)
    cv2.putText(canvas, f"{distance_ft}ft LiDAR-only target evidence", (25, 45), 0, .85, (0, 0, 0), 2)
    cv2.putText(canvas, "UNRESOLVED: zero usable returns in expected target ROI", (70, 365), 0, .8, (0, 0, 210), 2)
    cv2.putText(canvas, "No corners synthesized; no projection used", (70, 415), 0, .65, (0, 0, 0), 2)
    cv2.putText(canvas, str(evidence.get("status", "UNRESOLVED")), (25, 760), 0, .65, (0, 0, 210), 2)
    cv2.imwrite(str(path), canvas)


def rotation_invariant_candidate(points, distance_ft):
    """Diagnose candidates by radial range, without assigning origin/corner identity."""
    found = RANGE.discover_candidates(points, distance_ft * 0.3048)
    clusters = found["candidates"]
    # A candidate is deliberately UNRESOLVED: radial discovery cannot establish
    # physical boundary, pedestal origin, polarity, or mounting compatibility.
    return {"status": "UNRESOLVED" if clusters else "FAIL", "range_shell_points": found["range_shell_points"],
            "high_intensity_points": found["seed_points"], "clusters": clusters,
            "reason": "rotation-invariant candidate discovery only; no origin/corners accepted"}


def board_plane_extraction(messages, distance_ft):
    """Generate tilted-board hypotheses without assigning physical identity."""
    if not messages:
        return {"status": "UNRESOLVED_NO_LIDAR", "candidates": []}
    points = np.vstack([m["points"] for m in messages])
    radial = np.linalg.norm(points[:, :3], axis=1)
    shell = points[np.abs(radial - distance_ft * .3048) < .55]
    discovered = RANGE.discover_candidates(shell, distance_ft * .3048)
    records = []
    for candidate in discovered["candidates"]:
        geometry = BOARD_PLANE.board_plane_hypotheses(shell, candidate)
        hypotheses = []
        for hypothesis in geometry.get("hypotheses", [])[:3]:
            support = BOARD_PLANE.support_evidence(messages, hypothesis)
            hypotheses.append({k: v for k, v in hypothesis.items() if k != "local_points_uv"} | {
                "support_alternatives": [{k: v for k, v in item.items() if k != "point_masks"} for item in support]
            })
        records.append({"candidate": candidate, "geometry": {k: v for k, v in geometry.items() if k != "hypotheses"},
                        "hypotheses": hypotheses})
    return {"status": "UNRESOLVED", "reason": "board-plane hypotheses extracted; boundary/origin/polarity and fixed indices are not independently frozen",
            "physical_window_m": [BOARD_PLANE.BOARD_WIDTH_M, BOARD_PLANE.BOARD_HEIGHT_M],
            "candidate_count": len(records), "candidates": records}


def main(args):
    if args.distance_ft not in (5, 10, 15, 22):
        raise SystemExit("--distance-ft must be one of 5, 10, 15 or 22")
    out = Path(args.output_dir).expanduser().resolve()
    if out.exists():
        raise SystemExit(f"output directory already exists: {out}")
    out.mkdir(parents=True); images = out / "images"; images.mkdir()
    parts = resolve_bag_parts(args.bag)
    topics = topic_inventory(parts)
    image_topic = "/camera/camera/color/image_raw"
    info_topic = "/camera/camera/color/camera_info"
    lidar_topic = "/livox/lidar"
    image_rows, image_type = rows_for_topic(parts, image_topic, True)
    info_rows, info_type = rows_for_topic(parts, info_topic, True)
    lidar_rows, lidar_type = rows_for_topic(parts, lidar_topic, True)

    # Decode recorded camera model; missing evidence remains unresolved/failing.
    K = D = None; resolution = None; camera_frame = None
    if info_rows and info_type:
        msg = TYPESTORE.deserialize_cdr(info_rows[0][1], info_type)
        K = np.asarray(msg.k, float).reshape(3, 3); D = np.asarray(msg.d, float)
        resolution = [int(msg.width), int(msg.height)]; camera_frame = msg.header.frame_id

    decoded_images = []
    decoded_resolution_counts = {}
    detections = []
    for timestamp, raw in image_rows:
        image = decode_rgb_message(raw)
        if image is None:
            detections.append(None); continue
        image_resolution = (int(image.shape[1]), int(image.shape[0]))
        decoded_resolution_counts[image_resolution] = decoded_resolution_counts.get(image_resolution, 0) + 1
        if resolution is None:
            resolution = list(image_resolution)
        decoded_images.append((int(timestamp), image))
        detections.append(detect_grid(image))
    detected_count = sum(item is not None for item in detections)
    detection_rate = detected_count / len(image_rows) if image_rows else 0.0
    sample_index = next((i for i, item in enumerate(detections) if item is not None), None)
    sample_image = decoded_images[0][1] if decoded_images else None
    sample_corners = None
    if sample_index is not None:
        # decoded_images and detections have the same length when decoding succeeds (normal ROS Image input).
        sample_image = decode_rgb_message(image_rows[sample_index][1])
        sample_corners = detections[sample_index]
    make_rgb_image(sample_image, sample_corners, detected_count, len(image_rows), images / "rgb_grid_evidence.png")

    image_ts = np.asarray([r[0] for r in image_rows], np.int64)
    lidar_ts = np.asarray([r[0] for r in lidar_rows], np.int64)
    overlap_s = 0.0
    if len(image_ts) and len(lidar_ts):
        overlap_s = max(0.0, (min(image_ts[-1], lidar_ts[-1]) - max(image_ts[0], lidar_ts[0])) / 1e9)

    lidar_messages = []; lidar_schema = None; lidar_signature = None
    if lidar_rows and lidar_type:
        lidar_messages, lidar_schema, lidar_signature = decode_lidar(lidar_rows, lidar_type)

    # Audited drift metric: block backgrounds, target masked by predeclared distance slab,
    # point-to-plane ICP to a frozen middle reference block, then p95 rotation.
    block_size = max(4, int(args.block_messages))
    blocks = [lidar_messages[i:i + block_size] for i in range(0, len(lidar_messages), block_size)
              if len(lidar_messages[i:i + block_size]) >= 4]
    backgrounds = []
    for block in blocks:
        points = np.vstack([m["points"] for m in block])
        backgrounds.append(background(points, args.distance_ft))
    drift_rows = []; drift_p95 = None
    if backgrounds and len(backgrounds[len(backgrounds)//2]) >= 30:
        ref_index = len(backgrounds) // 2; ref = backgrounds[ref_index]
        t0 = lidar_messages[0]["timestamp_ns"]
        for index, (block, bg) in enumerate(zip(blocks, backgrounds)):
            if len(bg) < 30: continue
            transform, rmse, overlap, condition = icp_ptplane(bg, ref)
            mid = block[len(block)//2]["timestamp_ns"]
            drift_rows.append({"block": index, "time_s": (mid - t0) / 1e9,
                               "rotation_deg": rotation_angle_deg(transform[:3, :3]),
                               "translation_m": float(np.linalg.norm(transform[:3, 3])),
                               "icp_rmse_m": rmse, "overlap": overlap,
                               "condition_number": condition, "reference_block": index == ref_index})
        if drift_rows:
            drift_p95 = float(np.percentile([r["rotation_deg"] for r in drift_rows], 95))
            make_drift_plot(drift_rows, drift_p95, images / "background_drift.png")
            with (out / "background_drift.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=drift_rows[0]); writer.writeheader(); writer.writerows(drift_rows)

    # Rotation-aware board-plane extraction.  Candidate discovery and physical
    # identity acceptance remain separate; without independent asymmetric
    # evidence no fixed corner ordering is emitted.
    corners = None; origin_evidence = {"status": "UNRESOLVED_NO_LIDAR"}; subset_corners = []; invariant = {"status": "FAIL", "range_shell_points": 0, "clusters": []}
    if lidar_messages:
        all_lidar_points = np.vstack([m["points"] for m in lidar_messages])
        invariant = rotation_invariant_candidate(all_lidar_points, args.distance_ft)
        origin_evidence = board_plane_extraction(lidar_messages, args.distance_ft)
        make_lidar_evidence(lidar_messages, corners, origin_evidence, args.distance_ft,
                            images / "lidar_origin_coverage_evidence.png")

    lidar_sigma_m = predicted_sigma_px = None
    if len(subset_corners) >= 3 and K is not None:
        stack = np.asarray(subset_corners)
        per_corner = np.sqrt(np.trace(np.asarray([np.cov(stack[:, i, :].T) for i in range(18)]), axis1=1, axis2=2) / 3.0)
        lidar_sigma_m = float(np.nanmedian(per_corner))
        predicted_sigma_px = float(lidar_sigma_m * K[0, 0] / (args.distance_ft * 0.3048))

    reference_audit = Path(args.reference_audit).expanduser().resolve()
    reference = json.loads(reference_audit.read_text())["records"] if reference_audit.exists() else []
    reference = next((r for r in reference if int(r["distance_ft"]) == 10), None)
    intrinsics_match = bool(reference and K is not None and np.allclose(K, np.asarray(reference["K"]), atol=0.2)
                            and D is not None and np.allclose(D, np.asarray(reference["D"]), atol=1e-9))
    schema_match = bool(reference and lidar_signature and
                        lidar_signature["point_step"] == reference["lidar_schema"]["point_step"] and
                        lidar_signature["is_bigendian"] == reference["lidar_schema"]["is_bigendian"] and
                        lidar_signature["fields"] == reference["lidar_schema"]["fields"])
    mount = mounting_evidence(args.mounting_evidence, args.reference_mounting_evidence)
    board = board_evidence(args.board_evidence)
    decoded_resolutions = [{"resolution": list(key), "count": value}
                           for key, value in sorted(decoded_resolution_counts.items())]
    image_info_consistent = bool(decoded_resolution_counts and resolution is not None and
                                 set(decoded_resolution_counts) == {tuple(resolution)})

    checks = {
        "required_topics": {"status": status(bool(image_rows and info_rows and lidar_rows)),
                            "rgb_count": len(image_rows), "camera_info_count": len(info_rows), "lidar_count": len(lidar_rows)},
        "timestamp_overlap": {"status": status(overlap_s >= MIN_OVERLAP_S, bool(image_rows and lidar_rows)), "measured_s": overlap_s, "minimum_s": MIN_OVERLAP_S},
        "native_resolution": {"status": status(resolution == [640, 480] and image_info_consistent,
                                                        resolution is not None and bool(decoded_resolution_counts)),
                              "camera_info": resolution, "decoded_image_resolutions": decoded_resolutions,
                              "image_camera_info_consistent": image_info_consistent, "required": [640, 480]},
        "camera_intrinsics_recorded": {"status": status(K is not None and D is not None, K is not None or D is not None), "K": K.tolist() if K is not None else None, "D": D.tolist() if D is not None else None, "frame_id": camera_frame},
        "declared_geometry": {"status": status(abs(args.measured_pitch_m - PITCH_M) <= 1e-6), "pitch_m": args.measured_pitch_m, "printed_cells": list(PRINTED_CELLS), "inner_corners": list(PATTERN), "note": "command-line declaration only; physical_board_evidence must also pass"},
        "physical_board_evidence": board,
        "rgb_complete_detection": {"status": status(detection_rate >= MIN_RGB_RATE, bool(image_rows)), "detected": detected_count, "total": len(image_rows), "rate": detection_rate, "minimum": MIN_RGB_RATE},
        "background_drift": {"status": status(drift_p95 is not None and drift_p95 <= MAX_DRIFT_P95_DEG, drift_p95 is not None), "p95_rotation_deg": drift_p95, "maximum_deg": MAX_DRIFT_P95_DEG, "metric": "target-masked block-to-middle-reference point-to-plane ICP p95 rotation"},
        "lidar_origin_polarity": {"status": "PASS" if origin_evidence.get("status", "").startswith("ACCEPTED") else ("FAIL" if origin_evidence.get("status", "").startswith("REJECTED") else "UNRESOLVED"), "evidence": origin_evidence},
        # Candidate enumeration deliberately does not establish a unique
        # boundary, origin or index identity.  A legacy corner hypothesis must
        # never promote this independent discovery diagnostic to PASS.
        "rotation_invariant_candidate_discovery": {"status": invariant["status"], "diagnostic": invariant},
        "lidar_independent_blocks": {"status": status(len(subset_corners) >= 3, bool(lidar_messages)), "accepted_subsets": len(subset_corners), "required": 3},
        "predicted_corner_uncertainty": {"status": status(predicted_sigma_px is not None and predicted_sigma_px < MAX_PREDICTED_SIGMA_PX, predicted_sigma_px is not None), "lidar_sigma_m_median": lidar_sigma_m, "predicted_px": predicted_sigma_px, "maximum_px_strict": MAX_PREDICTED_SIGMA_PX},
        "reference_intrinsics": {"status": status(intrinsics_match, bool(reference and K is not None)), "tolerance_px": 0.2},
        "reference_lidar_schema": {"status": status(schema_match, bool(reference and lidar_signature))},
        "relative_mounting": mount,
    }
    required_names = list(checks)
    overall = "PASS" if all(checks[name]["status"] == "PASS" for name in required_names) else (
        "FAIL" if any(checks[name]["status"] == "FAIL" for name in required_names) else "UNRESOLVED")
    if any(checks[name]["status"] == "FAIL" for name in required_names): overall = "FAIL"

    result = {
        "purpose": "replacement acquisition preflight; not calibration evidence",
        "distance_ft": args.distance_ft, "overall_status": overall,
        "calibration_or_projection_run": False, "reprojection_used_for_decision": False,
        "bag_argument": str(Path(args.bag).expanduser()),
        "bag_parts": [{"path": str(p), "bytes": p.stat().st_size, "sha256": sha256(p)} for p in parts],
        "topics": topics, "checks": checks,
        "frame_explanation": {
            "lidar_world_background_alignment": "Target-masked static geometry is registered between time blocks. This measures LiDAR-frame/background stability only.",
            "camera_lidar_mounting": "Background ICP alone cannot prove unchanged camera-to-LiDAR extrinsics. PASS requires independent sensor serial, locked-rig and witness-mark evidence; otherwise this gate is UNRESOLVED."
        },
        "freeze_authorized": overall == "PASS",
        "next_action": "freeze observations before any projection" if overall == "PASS" else "correct/re-record failed gates; supply missing evidence for unresolved gates"
    }
    (out / "preflight.json").write_text(json.dumps(result, indent=2, sort_keys=True, default=lambda x: x.item() if hasattr(x, "item") else x.tolist()) + "\n")

    # Human report and dashboard.
    lines = [f"# {args.distance_ft}-ft replacement-bag preflight", "", f"**{overall}** — software preflight only; no calibration or reprojection was run.", "",
             "| Gate | Status | Measured/evidence |", "|---|---|---|"]
    for name, item in checks.items():
        measured = {k: v for k, v in item.items() if k not in ("status", "evidence", "K", "D", "values")}
        lines.append(f"| `{name}` | **{item['status']}** | `{json.dumps(measured, default=str)}` |")
    lines += ["", "## Frame interpretation", "",
              "LiDAR world/background alignment registers target-masked static scene geometry and measures LiDAR stability. It does **not** prove the camera–LiDAR bracket remained unchanged.", "",
              "Relative mounting is independently PASS only with matching sensor serials, a locked rig/configuration identifier and unchanged physical witness marks. Recorded K/D, schema and frame IDs are supporting metadata, not proof of mechanical preservation.", "",
              "## Failure interpretation", "",
              "- `FAIL`: the measured acquisition violates a gate; correct the physical setup and record again.",
              "- `UNRESOLVED`: required evidence is missing or cannot be established independently; never treat it as PASS.",
              "- Only an all-PASS report authorizes freezing. Projection and calibration remain forbidden until then."]
    (out / "PREFLIGHT_REPORT.md").write_text("\n".join(lines) + "\n")

    colors = {"PASS": (46, 125, 50), "FAIL": (198, 40, 40), "UNRESOLVED": (239, 108, 0)}
    fig, ax = plt.subplots(figsize=(15, 9)); ax.axis("off")
    ax.text(.03, .95, f"{args.distance_ft}-ft replacement preflight", fontsize=25, weight="bold")
    ax.text(.03, .90, f"Overall: {overall} • no reprojection/calibration used", fontsize=16, weight="bold", color=np.array(colors[overall])/255)
    y = .83
    for name, item in checks.items():
        ax.text(.04, y, name.replace("_", " "), fontsize=12)
        ax.text(.66, y, item["status"], fontsize=12, weight="bold", color=np.array(colors[item["status"]])/255)
        y -= .058
    ax.text(.03, .08, "Missing evidence never defaults to PASS. Background alignment ≠ proof of relative sensor mounting.", fontsize=13, weight="bold", color="#8b1d1d")
    fig.tight_layout(); fig.savefig(images / "preflight_dashboard.png", dpi=180); plt.close(fig)
    hashes = {str(p.relative_to(out)): sha256(p) for p in sorted(out.rglob("*")) if p.is_file() and p.name != "SHA256SUMS.json"}
    (out / "SHA256SUMS.json").write_text(json.dumps(hashes, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"output": str(out), "status": overall, "bag_parts": len(parts),
                      "failed": [k for k, v in checks.items() if v["status"] == "FAIL"],
                      "unresolved": [k for k, v in checks.items() if v["status"] == "UNRESOLVED"]}, indent=2))
    return 0 if overall == "PASS" else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="existing ROS2 bag folder or DB3 path; metadata split files are included")
    parser.add_argument("--distance-ft", type=int, required=True, choices=(5, 10, 15, 22))
    parser.add_argument("--measured-pitch-m", type=float, required=True,
                        help="independently measured physical cell pitch; required value is 0.096")
    parser.add_argument("--mounting-evidence", help="JSON containing sensor serials, rig lock, witness marks and reference ID")
    parser.add_argument("--reference-mounting-evidence", help="independently frozen 10/15-ft sensor serial and configuration reference JSON")
    parser.add_argument("--reference-audit", default=str(REFERENCE_AUDIT),
                        help="frozen 10/15-ft audit JSON used only for K/D and LiDAR-schema compatibility")
    parser.add_argument("--board-evidence", help="JSON documenting photographed 7x4 topology and repeated 0.096 m pitch measurements")
    parser.add_argument("--block-messages", type=int, default=8)
    parser.add_argument("--output-dir", required=True)
    raise SystemExit(main(parser.parse_args()))
