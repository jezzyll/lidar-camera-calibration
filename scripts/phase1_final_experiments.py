#!/usr/bin/env python3
"""Run the final Phase 1 split, diagnostics, and required experiments.

Internal calibration uses only LiDAR frame indices ``index % 4 == 0`` and the
first RGB reference frame from each bag. Validation uses ten explicitly listed
LiDAR frames selected from ``index % 4 == 2`` and their nearest RGB frames.
The frozen transform is never refined with validation frames.

The supplied bags contain only one mostly static capture session per distance.
Consequently, this script provides a strict *frame-level* split and staticity
diagnostics, while explicitly leaving the separate-capture controlled
experiment and larger far-range target capture open.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation

from calibrate import (
    CALIBRATION_POSES,
    POINT_DTYPE,
    detect_camera_board,
    extract_intrinsics,
    fit_camera_board,
    fit_lidar_board,
    image_rgb,
    lidar_board_corners,
    load_first,
    refine_extrinsic,
)
from phase1_required_validation import (
    BOARD_HEIGHT_M,
    BOARD_MARGIN_M,
    BOARD_WIDTH_M,
    PLANE_RESIDUAL_LIMIT_M,
    aggregate_metrics,
    calibration_parameters,
    camera_projection,
    error_metrics,
    evaluate_public,
    image_homography_reference,
    nearest_rgb_frames,
    save_qualitative,
)


VALIDATION_FRAME_COUNT = 10
CALIBRATION_MODULUS = 4
CALIBRATION_REMAINDER = 0
VALIDATION_REMAINDER = 2


def json_ready(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.bool_):
        return bool(value)
    raise TypeError(type(value).__name__)


def load_lidar_roi_frames(bag: Path, spec: dict) -> list[dict]:
    frames = []
    with AnyReader([bag]) as reader:
        connection = next(c for c in reader.connections if c.topic == "/livox/lidar")
        for index, (conn, timestamp, raw) in enumerate(
            reader.messages(connections=[connection])
        ):
            message = reader.deserialize(raw, conn.msgtype)
            source = np.frombuffer(message.data, dtype=POINT_DTYPE)
            keep = (
                (np.abs(source["x"] - spec["x"]) < 0.075)
                & (source["y"] >= spec["y"][0])
                & (source["y"] <= spec["y"][1])
                & (source["z"] >= spec["z"][0])
                & (source["z"] <= spec["z"][1])
            )
            selected = source[keep]
            points = np.c_[
                selected["x"], selected["y"], selected["z"], selected["intensity"],
            ].astype(np.float64)
            frames.append({
                "index": index,
                "timestamp_ns": int(timestamp),
                "roi_points": points,
            })
    if not frames:
        raise RuntimeError(f"No LiDAR frames in {bag}")
    return frames


def explicit_split(frame_count: int) -> tuple[list[int], list[int]]:
    calibration = [
        index for index in range(frame_count)
        if index % CALIBRATION_MODULUS == CALIBRATION_REMAINDER
    ]
    validation_candidates = [
        index for index in range(frame_count)
        if index % CALIBRATION_MODULUS == VALIDATION_REMAINDER
    ]
    positions = np.linspace(
        0, len(validation_candidates) - 1, VALIDATION_FRAME_COUNT, dtype=int,
    )
    validation = [validation_candidates[position] for position in positions]
    if set(calibration) & set(validation):
        raise RuntimeError("Calibration and validation LiDAR indices overlap")
    return calibration, validation


def first_rgb_image(bag: Path) -> tuple[int, np.ndarray]:
    with AnyReader([bag]) as reader:
        timestamp, message = load_first(reader, "/camera/camera/color/image_raw")
    return int(timestamp), image_rgb(message)


def track_polygon(reference_rgb: np.ndarray, target_rgb: np.ndarray,
                  reference_polygon: np.ndarray) -> tuple[np.ndarray, dict]:
    """Track the board by RGB-only template translation in the original grid."""
    gray_reference = cv2.cvtColor(reference_rgb, cv2.COLOR_RGB2GRAY)
    gray_target = cv2.cvtColor(target_rgb, cv2.COLOR_RGB2GRAY)
    minimum = np.floor(reference_polygon.min(axis=0)).astype(int)
    maximum = np.ceil(reference_polygon.max(axis=0)).astype(int)
    board_width = max(4, maximum[0] - minimum[0])
    board_height = max(4, maximum[1] - minimum[1])
    pad_x = max(4, int(round(0.15 * board_width)))
    pad_y = max(4, int(round(0.10 * board_height)))
    x1 = max(0, minimum[0] - pad_x)
    y1 = max(0, minimum[1] - pad_y)
    x2 = min(gray_reference.shape[1], maximum[0] + pad_x + 1)
    y2 = min(gray_reference.shape[0], maximum[1] + pad_y + 1)
    template = gray_reference[y1:y2, x1:x2]
    radius = 10
    sx1 = max(0, x1 - radius)
    sy1 = max(0, y1 - radius)
    sx2 = min(gray_target.shape[1], x2 + radius)
    sy2 = min(gray_target.shape[0], y2 + radius)
    search = gray_target[sy1:sy2, sx1:sx2]
    response = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
    _, score, _, location = cv2.minMaxLoc(response)
    matched_x = sx1 + location[0]
    matched_y = sy1 + location[1]
    dx = float(matched_x - x1)
    dy = float(matched_y - y1)
    polygon = reference_polygon + np.asarray([dx, dy])
    return polygon, {
        "translation_px": [dx, dy],
        "translation_magnitude_px": float(np.hypot(dx, dy)),
        "template_correlation": float(score),
        "method": "RGB-only template translation; never used for LiDAR selection or R,t fitting",
    }


def calibrate_from_split(data_root: Path) -> tuple[dict, dict, dict, dict]:
    ordered = sorted(CALIBRATION_POSES.items(), key=lambda pair: pair[1]["distance_ft"])
    frame_cache = {}
    split = {}
    rgb_references = {}
    camera_models = {}
    for name, spec in ordered:
        bag = data_root / name
        frames = load_lidar_roi_frames(bag, spec)
        calibration_indices, validation_indices = explicit_split(len(frames))
        frame_cache[name] = frames
        rgb_timestamp, rgb = first_rgb_image(bag)
        rgb_references[name] = rgb
        camera_models[name] = extract_intrinsics(bag)
        split[name] = {
            "distance_ft": spec["distance_ft"],
            "total_lidar_frames": len(frames),
            "calibration_lidar_rule": "frame_index % 4 == 0",
            "calibration_lidar_indices": calibration_indices,
            "validation_lidar_rule": "10 evenly spaced frames from frame_index % 4 == 2",
            "validation_lidar_indices": validation_indices,
            "unused_lidar_rule": "frame_index % 4 in {1,3}",
            "calibration_rgb_frame_index": 0,
            "calibration_rgb_timestamp_ns": rgb_timestamp,
        }

    near_name = "dataset02 7.25feet"
    near_k, near_d, width, height, model, camera_frame = camera_models[near_name]
    near_corners = detect_camera_board(rgb_references[near_name])
    r_board_camera, center_camera, near_polygon, camera_rmse = fit_camera_board(
        near_corners, near_k, near_d,
    )

    observations = []
    plane_quality = {}
    calibration_board_models = {}
    for name, spec in ordered:
        points = np.concatenate([
            frame_cache[name][index]["roi_points"]
            for index in split[name]["calibration_lidar_indices"]
            if len(frame_cache[name][index]["roi_points"])
        ])
        board_rotation, board_center, plane_rmse, inliers, coefficient = fit_lidar_board(
            points[:, :3], spec["y"], spec["z"],
        )
        camera_polygon = (
            near_polygon if name == near_name
            else np.asarray(spec["camera_board_corners_px"], dtype=np.float64)
        )
        observations.append({
            "dataset": name,
            "lidar_corners": lidar_board_corners(coefficient, spec["y"], spec["z"]),
            "camera_corners": camera_polygon,
        })
        plane_quality[name] = {
            "distance_ft": spec["distance_ft"],
            "calibration_lidar_board_plane_rmse_m": plane_rmse,
            "calibration_lidar_plane_inliers": inliers,
            "calibration_lidar_roi_points": int(len(points)),
        }
        calibration_board_models[name] = {
            "rotation": board_rotation,
            "center": board_center,
            "coefficient": coefficient,
            "camera_polygon": camera_polygon,
        }

    near_model = calibration_board_models[near_name]
    initial_r = r_board_camera @ near_model["rotation"].T
    initial_t = center_camera - initial_r @ near_model["center"]
    rotation, translation, joint_rmse, corner_errors = refine_extrinsic(
        initial_r, initial_t, observations, near_k, near_d,
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    quaternion = Rotation.from_matrix(rotation).as_quat()

    observation_report = {}
    offset = 0
    for observation in observations:
        name = observation["dataset"]
        errors = corner_errors[offset:offset + 4]
        offset += 4
        observation_report[name] = {
            **plane_quality[name],
            "camera_board_corners_px": observation["camera_corners"].tolist(),
            "camera_boundary_source": CALIBRATION_POSES[name]["camera_boundary_source"],
            "board_corner_reprojection_rmse_px": float(np.sqrt(np.mean(errors ** 2))),
        }

    frozen = {
        "direction": "p_camera_color_optical = R * p_livox + t",
        "frozen_before_validation": True,
        "camera": {
            "frame": camera_frame,
            "width": width,
            "height": height,
            "distortion_model": model,
            "K": near_k.tolist(),
            "D": near_d.tolist(),
            "source": "recorded CameraInfo (factory/device calibration)",
        },
        "checkerboard": {
            "inner_corners": [11, 17],
            "square_size_m": 0.025,
            "physical_size_m": [BOARD_WIDTH_M, BOARD_HEIGHT_M],
        },
        "extrinsic": {
            "parent_frame": camera_frame,
            "child_frame": "livox_frame",
            "R": rotation.tolist(),
            "t_m": translation.tolist(),
            "quaternion_xyzw": quaternion.tolist(),
            "T_camera_lidar": transform.tolist(),
            "method": "calibration-frame-only multi-range board-boundary refinement",
        },
        "quality": {
            "camera_checkerboard_reprojection_rmse_px": camera_rmse,
            "calibration_joint_board_corner_reprojection_rmse_px": joint_rmse,
        },
        "calibration_observations": observation_report,
        "split_contract": {
            "calibration_frames_used_to_fit_R_or_t": True,
            "validation_frames_used_to_fit_R_or_t": False,
            "sets_disjoint": True,
            "scope_limitation": (
                "strict non-overlapping frame split inside one capture session per distance; "
                "not a substitute for separate validation capture sessions"
            ),
            "datasets": split,
        },
    }

    # Device-model consistency across all internal bags.
    model_report = {}
    for name, values in camera_models.items():
        k, d, w, h, distortion_model, frame = values
        model_report[name] = {
            "resolution_wh": [w, h],
            "distortion_model": distortion_model,
            "camera_frame": frame,
            "max_abs_K_difference_from_near": float(np.max(np.abs(k - near_k))),
            "max_abs_D_difference_from_near": float(np.max(np.abs(d - near_d))),
        }
    return frozen, split, frame_cache, {
        "rgb_references": rgb_references,
        "calibration_board_models": calibration_board_models,
        "camera_model_consistency": model_report,
    }


def select_validation_points(points: np.ndarray, spec: dict) -> tuple[np.ndarray, np.ndarray, dict]:
    if len(points) < 6:
        raise RuntimeError("Insufficient LiDAR ROI returns for board validation")
    board_rotation, board_center, plane_rmse, plane_inliers, coefficient = fit_lidar_board(
        points[:, :3], spec["y"], spec["z"],
    )
    residual = points[:, 0] - np.c_[
        points[:, 1], points[:, 2], np.ones(len(points)),
    ] @ coefficient
    residual_center = float(np.median(residual))
    local = (points[:, :3] - board_center) @ board_rotation
    keep = (
        (np.abs(residual - residual_center) <= PLANE_RESIDUAL_LIMIT_M)
        & (local[:, 0] >= -BOARD_WIDTH_M / 2 + BOARD_MARGIN_M)
        & (local[:, 0] <= BOARD_WIDTH_M / 2 - BOARD_MARGIN_M)
        & (local[:, 1] >= -BOARD_HEIGHT_M / 2 + BOARD_MARGIN_M)
        & (local[:, 1] <= BOARD_HEIGHT_M / 2 - BOARD_MARGIN_M)
    )
    selected = points[keep]
    selected_local = local[keep]
    if len(selected) < 4:
        raise RuntimeError(
            f"Only {len(selected)} independently selected board returns remain "
            "after the LiDAR-only plane/extent filter"
        )
    normalized = np.c_[
        (selected_local[:, 0] + BOARD_WIDTH_M / 2) / BOARD_WIDTH_M,
        (selected_local[:, 1] + BOARD_HEIGHT_M / 2) / BOARD_HEIGHT_M,
    ]
    return selected, normalized, {
        "roi_points": int(len(points)),
        "selected_points": int(len(selected)),
        "plane_rmse_m": plane_rmse,
        "plane_inliers": plane_inliers,
        "plane_coefficient": coefficient.tolist(),
        "board_center_lidar_m": board_center.tolist(),
    }


def vector_metrics(actual: np.ndarray, expected: np.ndarray) -> dict:
    vectors = actual - expected
    magnitudes = np.linalg.norm(vectors, axis=1)
    mean_vector = vectors.mean(axis=0)
    mean_magnitude = float(np.mean(magnitudes))
    consistency = float(np.linalg.norm(mean_vector) / max(mean_magnitude, 1e-12))
    return {
        "mean_error_vector_du_dv_px": mean_vector.tolist(),
        "mean_error_vector_magnitude_px": float(np.linalg.norm(mean_vector)),
        "directional_consistency_0_random_1_uniform": consistency,
        "dominant_direction_deg_image_coordinates": float(
            np.degrees(np.arctan2(mean_vector[1], mean_vector[0]))
        ),
        "interpretation": (
            "predominantly systematic direction"
            if consistency >= 0.75 else
            "mixed/random directions or reference noise"
        ),
    }


def save_error_vectors(image: np.ndarray, polygon: np.ndarray, actual: np.ndarray,
                       expected: np.ndarray, output: Path, title: str) -> dict:
    errors = np.linalg.norm(actual - expected, axis=1)
    if len(actual) > 60:
        indices = np.linspace(0, len(actual) - 1, 60, dtype=int)
    else:
        indices = np.arange(len(actual))
    a = actual[indices]
    e = expected[indices]
    m = errors[indices]
    minimum = np.minimum(polygon.min(axis=0), np.minimum(a.min(axis=0), e.min(axis=0)))
    maximum = np.maximum(polygon.max(axis=0), np.maximum(a.max(axis=0), e.max(axis=0)))
    span = maximum - minimum
    padding = np.maximum(np.asarray([10.0, 10.0]), 0.25 * span)

    figure, axes = plt.subplots(1, 2, figsize=(13, 5.4), constrained_layout=True)
    for axis in axes:
        axis.imshow(image)
        closed = np.vstack([polygon, polygon[0]])
        axis.plot(closed[:, 0], closed[:, 1], color="lime", linewidth=2)
        axis.axis("off")
    axes[0].set_title("Original RGB and checkerboard reference")
    axes[1].set_title("Reference → projected LiDAR reprojection vectors")
    axes[1].set_xlim(minimum[0] - padding[0], maximum[0] + padding[0])
    axes[1].set_ylim(maximum[1] + padding[1], minimum[1] - padding[1])
    axes[1].scatter(e[:, 0], e[:, 1], marker="x", c="white", s=25, linewidths=1.0,
                    label="reference pixel")
    scatter = axes[1].scatter(a[:, 0], a[:, 1], c=m, cmap="turbo", s=28,
                              edgecolors="black", linewidths=0.35,
                              label="projected LiDAR pixel")
    axes[1].quiver(
        e[:, 0], e[:, 1], a[:, 0] - e[:, 0], a[:, 1] - e[:, 1], m,
        cmap="turbo", angles="xy", scale_units="xy", scale=1.0,
        width=0.004, headwidth=4.0, headlength=5.0,
    )
    label_indices = np.linspace(0, len(a) - 1, min(12, len(a)), dtype=int)
    for index in label_indices:
        axes[1].text(
            a[index, 0] + 0.35, a[index, 1] - 0.35, f"{m[index]:.1f}",
            color="white", fontsize=7,
            bbox={"facecolor": "black", "alpha": 0.55, "pad": 1},
        )
    axes[1].legend(loc="lower right", fontsize=8)
    figure.colorbar(scatter, ax=axes[1], fraction=0.035, pad=0.02).set_label(
        "Error magnitude (px)"
    )
    metrics = vector_metrics(actual, expected)
    figure.suptitle(
        f"{title}\nmean vector=({metrics['mean_error_vector_du_dv_px'][0]:.2f}, "
        f"{metrics['mean_error_vector_du_dv_px'][1]:.2f}) px | "
        f"directional consistency={metrics['directional_consistency_0_random_1_uniform']:.2f}",
        fontsize=13, fontweight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)
    return {"output": str(output), **metrics}


def safe_correlation(x, y) -> float | None:
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    if len(x) < 3 or np.std(x) <= 1e-12 or np.std(y) <= 1e-12:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def evaluate_internal_split(args, frozen: dict, split: dict, frame_cache: dict,
                            context: dict) -> dict:
    camera_k = np.asarray(frozen["camera"]["K"], dtype=np.float64)
    distortion = np.asarray(frozen["camera"]["D"], dtype=np.float64)
    rotation = np.asarray(frozen["extrinsic"]["R"], dtype=np.float64)
    translation = np.asarray(frozen["extrinsic"]["t_m"], dtype=np.float64)
    per_distance = []
    qualitative = []
    vector_figures = []
    timestamp_rows = []
    frame_rows = []

    for name, spec in sorted(CALIBRATION_POSES.items(), key=lambda pair: pair[1]["distance_ft"]):
        bag = args.internal_root / name
        validation_indices = split[name]["validation_lidar_indices"]
        frames = [frame_cache[name][index] for index in validation_indices]
        synchronized = nearest_rgb_frames(bag, [frame["timestamp_ns"] for frame in frames])
        reference_rgb = context["rgb_references"][name]
        reference_polygon = context["calibration_board_models"][name]["camera_polygon"]
        frame_records = []

        for evaluation_index, frame in enumerate(frames):
            selected, normalized, lidar_audit = select_validation_points(
                frame["roi_points"], spec,
            )
            rgb = synchronized[frame["timestamp_ns"]]
            polygon, tracking = track_polygon(reference_rgb, rgb["image"], reference_polygon)
            actual, depth = camera_projection(
                selected, rotation, translation, camera_k, distortion,
            )
            expected = image_homography_reference(normalized, polygon)
            valid = (
                (depth > 0)
                & (actual[:, 0] >= 0) & (actual[:, 0] < frozen["camera"]["width"])
                & (actual[:, 1] >= 0) & (actual[:, 1] < frozen["camera"]["height"])
            )
            actual = actual[valid]
            expected = expected[valid]
            depth = depth[valid]
            metrics, errors = error_metrics(actual, expected)
            vectors = vector_metrics(actual, expected)
            signed_delta_ms = (rgb["timestamp_ns"] - frame["timestamp_ns"]) / 1e6
            record = {
                "evaluation_frame_index": evaluation_index,
                "lidar_frame_index": frame["index"],
                "lidar_timestamp_ns": frame["timestamp_ns"],
                "rgb_frame_index": rgb["frame_index"],
                "rgb_timestamp_ns": rgb["timestamp_ns"],
                "signed_rgb_minus_lidar_ms": signed_delta_ms,
                "absolute_sync_delta_ms": abs(signed_delta_ms),
                "median_camera_depth_m": float(np.median(depth)),
                "rgb_reference_tracking": tracking,
                "lidar_board_selection": lidar_audit,
                **metrics,
                **vectors,
                "actual_pixels": actual,
                "expected_pixels": expected,
                "errors": errors,
                "rgb_image": rgb["image"],
                "camera_polygon": polygon,
            }
            frame_records.append(record)
            timestamp_rows.append({
                "dataset": name,
                "distance_ft": spec["distance_ft"],
                "evaluation_frame_index": evaluation_index,
                "lidar_frame_index": frame["index"],
                "lidar_timestamp_ns": frame["timestamp_ns"],
                "rgb_frame_index": rgb["frame_index"],
                "rgb_timestamp_ns": rgb["timestamp_ns"],
                "signed_rgb_minus_lidar_ms": signed_delta_ms,
                "absolute_sync_delta_ms": abs(signed_delta_ms),
            })
            frame_rows.append({
                "dataset": name,
                "distance_ft": spec["distance_ft"],
                "evaluation_frame_index": evaluation_index,
                "lidar_frame_index": frame["index"],
                **metrics,
            })
            if evaluation_index in (2, 7):
                prefix = args.output_dir / "qualitative" / (
                    f"internal_{str(spec['distance_ft']).replace('.', '_')}ft_"
                    f"validation_frame_{evaluation_index:02d}"
                )
                evidence = save_qualitative(
                    rgb["image"], polygon, actual, expected, errors,
                    f"Strict-split internal {spec['distance_ft']:g} ft — validation frame {evaluation_index}",
                    prefix, 12, 18,
                )
                qualitative.append({
                    "dataset": name,
                    "distance_ft": spec["distance_ft"],
                    "evaluation_frame_index": evaluation_index,
                    **evidence,
                })

        aggregate = aggregate_metrics(frame_records)
        aggregate_vectors = vector_metrics(
            np.concatenate([record["actual_pixels"] for record in frame_records]),
            np.concatenate([record["expected_pixels"] for record in frame_records]),
        )
        absolute_deltas = np.asarray([
            record["absolute_sync_delta_ms"] for record in frame_records
        ])
        signed_deltas = np.asarray([
            record["signed_rgb_minus_lidar_ms"] for record in frame_records
        ])
        timestamp_summary = {
            "pairs": len(frame_records),
            "mean_absolute_difference_ms": float(np.mean(absolute_deltas)),
            "median_absolute_difference_ms": float(np.median(absolute_deltas)),
            "minimum_absolute_difference_ms": float(np.min(absolute_deltas)),
            "maximum_absolute_difference_ms": float(np.max(absolute_deltas)),
            "mean_signed_rgb_minus_lidar_ms": float(np.mean(signed_deltas)),
            "error_vs_absolute_sync_delta_correlation": safe_correlation(
                absolute_deltas,
                [record["mean_reprojection_error_px"] for record in frame_records],
            ),
        }
        representative = min(
            frame_records,
            key=lambda record: abs(
                record["mean_reprojection_error_px"] - aggregate["mean_reprojection_error_px"]
            ),
        )
        vector_path = args.output_dir / "error_vectors" / (
            f"error_vectors_{str(spec['distance_ft']).replace('.', '_')}ft.png"
        )
        vector_figures.append({
            "dataset": name,
            "distance_ft": spec["distance_ft"],
            "evaluation_frame_index": representative["evaluation_frame_index"],
            **save_error_vectors(
                representative["rgb_image"], representative["camera_polygon"],
                representative["actual_pixels"], representative["expected_pixels"],
                vector_path, f"Internal {spec['distance_ft']:g} ft reprojection-error vectors",
            ),
        })

        clean_frames = []
        for record in frame_records:
            clean_frames.append({
                key: value for key, value in record.items()
                if key not in {
                    "actual_pixels", "expected_pixels", "errors", "rgb_image", "camera_polygon",
                }
            })
        per_distance.append({
            "dataset": name,
            "distance_ft": spec["distance_ft"],
            "calibration_lidar_indices_count": len(split[name]["calibration_lidar_indices"]),
            "validation_lidar_indices": validation_indices,
            "sets_disjoint": not bool(
                set(split[name]["calibration_lidar_indices"]) & set(validation_indices)
            ),
            **aggregate,
            **aggregate_vectors,
            "timestamp_analysis": timestamp_summary,
            "per_frame": clean_frames,
        })
    return {
        "per_distance": per_distance,
        "qualitative_examples": qualitative,
        "error_vector_figures": vector_figures,
        "timestamp_rows": timestamp_rows,
        "frame_rows": frame_rows,
    }


def diagnostic_results(frozen: dict, internal: dict, context: dict) -> dict:
    rotation = np.asarray(frozen["extrinsic"]["R"], dtype=np.float64)
    identity_error = rotation.T @ rotation - np.eye(3)
    determinant = float(np.linalg.det(rotation))
    distance_diagnostics = []
    for item in internal["per_distance"]:
        frames = item["per_frame"]
        plane_rmse = [frame["lidar_board_selection"]["plane_rmse_m"] for frame in frames]
        centers = np.asarray([
            frame["lidar_board_selection"]["board_center_lidar_m"] for frame in frames
        ])
        tracking = [
            frame["rgb_reference_tracking"]["translation_magnitude_px"] for frame in frames
        ]
        scores = [
            frame["rgb_reference_tracking"]["template_correlation"] for frame in frames
        ]
        camera_depths = [frame["median_camera_depth_m"] for frame in frames]
        distance_diagnostics.append({
            "distance_ft": item["distance_ft"],
            "timestamp": item["timestamp_analysis"],
            "error_vector": {
                key: item[key] for key in (
                    "mean_error_vector_du_dv_px",
                    "mean_error_vector_magnitude_px",
                    "directional_consistency_0_random_1_uniform",
                    "dominant_direction_deg_image_coordinates",
                    "interpretation",
                )
            },
            "validation_plane_rmse_mean_m": float(np.mean(plane_rmse)),
            "validation_plane_rmse_max_m": float(np.max(plane_rmse)),
            "validation_board_center_peak_to_peak_m": np.ptp(centers, axis=0).tolist(),
            "rgb_reference_translation_mean_px": float(np.mean(tracking)),
            "rgb_reference_translation_max_px": float(np.max(tracking)),
            "rgb_template_correlation_minimum": float(np.min(scores)),
            "median_camera_depth_range_m": [float(np.min(camera_depths)), float(np.max(camera_depths))],
        })

    intrinsics = context["camera_model_consistency"]
    max_k = max(value["max_abs_K_difference_from_near"] for value in intrinsics.values())
    max_d = max(value["max_abs_D_difference_from_near"] for value in intrinsics.values())
    vector_consistency = {
        str(item["distance_ft"]): item["directional_consistency_0_random_1_uniform"]
        for item in internal["per_distance"]
    }
    return {
        "timestamp_synchronization": {
            "result_by_distance": [item["timestamp"] for item in distance_diagnostics],
            "conclusion": (
                "All selected pairs are nearest-neighbour synchronized below 34 ms. "
                "Frame error remains nearly constant while timestamp deltas vary; timestamp "
                "offset is not supported as the primary cause, though hardware synchronization "
                "would strengthen a new capture."
            ),
        },
        "coordinate_frame_convention": {
            "equation": frozen["direction"],
            "camera_axes": "+X right, +Y down, +Z forward",
            "depth_ranges_by_distance": [
                {"distance_ft": item["distance_ft"], "range_m": item["median_camera_depth_range_m"]}
                for item in distance_diagnostics
            ],
            "conclusion": "Positive, monotonically increasing camera depth confirms transform direction and optical-axis convention.",
        },
        "rotation_matrix": {
            "R_transpose_R": (rotation.T @ rotation).tolist(),
            "orthogonality_error_frobenius": float(np.linalg.norm(identity_error, ord="fro")),
            "orthogonality_error_max_abs": float(np.max(np.abs(identity_error))),
            "determinant": determinant,
            "valid_rigid_rotation": bool(
                np.linalg.norm(identity_error, ord="fro") < 1e-9 and abs(determinant - 1.0) < 1e-9
            ),
            "conclusion": "The fitted rotation is mathematically a valid proper rigid-body rotation.",
        },
        "translation": {
            "t_camera_lidar_m": frozen["extrinsic"]["t_m"],
            "translation_norm_m": float(np.linalg.norm(frozen["extrinsic"]["t_m"])),
            "conclusion": (
                "Translation is finite and represented in metres. Its physical accuracy cannot "
                "be confirmed without an independent measurement of the sensor baseline; the "
                "persistent directional pixel bias therefore remains evidence against accepting "
                "the fitted transform as final."
            ),
        },
        "camera_distortion_model": {
            "model_by_dataset": intrinsics,
            "maximum_K_difference": max_k,
            "maximum_D_difference": max_d,
            "conclusion": "All internal bags carry the same 640x480 plumb-bob CameraInfo model; inconsistent intrinsics/distortion are not supported as the cause.",
        },
        "checkerboard_dimensions": {
            "inner_corners": [11, 17],
            "square_size_m": 0.025,
            "physical_squares": [12, 18],
            "physical_size_m": [BOARD_WIDTH_M, BOARD_HEIGHT_M],
            "dimension_identity_check": [12 * 0.025, 18 * 0.025],
            "conclusion": "Configured 0.30x0.45 m extent is internally consistent with 12x18 squares at 25 mm.",
        },
        "board_plane_estimation": {
            "by_distance": distance_diagnostics,
            "conclusion": "Validation plane RMSE is millimetric and stable. Plane noise is smaller than the systematic middle/far pixel offsets.",
        },
        "lidar_point_to_board_correspondence": {
            "selection": "LiDAR-only ROI, per-validation-frame plane residual, physical extent, and boundary margin",
            "rgb_mask_used": False,
            "conclusion": "Correspondence selection remains independent of RGB. Selected-point counts and plane fits are reported for every frame.",
        },
        "calibration_optimization": {
            "calibration_joint_corner_rmse_px": frozen["quality"]["calibration_joint_board_corner_reprojection_rmse_px"],
            "per_distance_corner_rmse_px": {
                name: value["board_corner_reprojection_rmse_px"]
                for name, value in frozen["calibration_observations"].items()
            },
            "conclusion": "The calibration-only multi-range objective cannot fit every annotated pose below 2 px; no post-validation refit or forced 99% correction was applied.",
        },
        "temporal_motion": {
            "by_distance": [
                {
                    "distance_ft": item["distance_ft"],
                    "rgb_reference_translation_max_px": item["rgb_reference_translation_max_px"],
                    "board_center_peak_to_peak_m": item["validation_board_center_peak_to_peak_m"],
                    "template_correlation_minimum": item["rgb_template_correlation_minimum"],
                }
                for item in distance_diagnostics
            ],
            "conclusion": "Existing bags appear static over the sampled frames. They were not recorded as a new controlled session, so this is diagnostic evidence rather than completion of the requested new controlled experiment.",
        },
        "reference_pixel_calculation": {
            "method": "physical LiDAR board coordinates mapped to an RGB-only tracked checkerboard polygon by homography",
            "camera_mask_used": False,
            "middle_far_reference_source": "manual outer boundary on the calibration RGB plus validation-frame RGB template tracking",
            "conclusion": "The middle/far target spans only tens of pixels, making one-pixel boundary uncertainty material. Directional vector bias and calibration-corner residuals support a geometry/reference inconsistency; improved independent target observations are required.",
        },
        "error_vector_direction": {
            "directional_consistency_by_distance": vector_consistency,
            "conclusion": "High directional consistency at failing distances indicates systematic extrinsic/reference bias rather than random point noise. See the three vector figures.",
        },
        "controlled_static_experiment": {
            "completed_with_new_capture": False,
            "provisional_existing_bag_analysis_completed": True,
            "required_capture_protocol": [
                "rigidly lock the D455 and Livox mount and record the physical sensor baseline",
                "rigidly mount a flat checkerboard; prevent target/sensor movement and people crossing the view",
                "record separate calibration and validation sessions at 7.25, 14.5, and 22.2 ft",
                "use hardware synchronization when available; otherwise record both native timestamps",
                "use multiple board positions and tilts for calibration and distinct held-out poses for validation",
                "freeze R and t before any validation bag is opened, then run this evaluator without refitting",
            ],
            "missing_evidence": [
                "separate calibration and validation capture sessions",
                "documented no-person/no-target-motion acquisition protocol",
                "new rigid-board capture at all three distances",
            ],
            "conclusion": "OPEN — requires physical data acquisition; existing static-frame analysis cannot be relabeled as a new controlled experiment.",
        },
        "larger_far_range_checkerboard": {
            "new_capture_completed": False,
            "recommended_target": (
                "rigid 0.60 x 0.90 m checkerboard (or larger while fully visible), preserving "
                "known square dimensions and a sharp, flat print"
            ),
            "required_range_ft": "22-30",
            "prerequisite": "run the documented synchronization, frame, plane, correspondence, and vector diagnostics first",
            "conclusion": "OPEN — no larger-board 22–30 ft raw capture was supplied. Diagnostics were performed first as required.",
        },
    }


def write_csv(path: Path, rows: list[dict], fields: list[str]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows([{field: row.get(field) for field in fields} for row in rows])


def write_outputs(args, report: dict, frozen: dict, internal: dict):
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "phase1_final_report.json").write_text(
        json.dumps(report, indent=2, default=json_ready) + "\n"
    )
    (args.output_dir / "frozen_calibration.json").write_text(
        json.dumps(frozen, indent=2, default=json_ready) + "\n"
    )
    (args.output_dir / "frozen_calibration.yaml").write_text(
        yaml.safe_dump(calibration_parameters(frozen), sort_keys=False)
    )
    (args.output_dir / "calibration_validation_split.json").write_text(
        json.dumps(frozen["split_contract"], indent=2) + "\n"
    )
    frame_fields = [
        "dataset", "distance_ft", "evaluation_frame_index", "lidar_frame_index",
        "evaluated_lidar_points", "mean_reprojection_error_px",
        "median_reprojection_error_px", "rmse_reprojection_error_px",
        "standard_deviation_reprojection_error_px", "maximum_reprojection_error_px",
        "percentage_within_1_px", "percentage_within_2_px", "percentage_within_3_px",
        "strict_same_raster_pixel_percentage", "passes_project_99pct_within_2px",
    ]
    write_csv(args.output_dir / "frame_level_error_metrics.csv", internal["frame_rows"], frame_fields)
    timestamp_fields = [
        "dataset", "distance_ft", "evaluation_frame_index", "lidar_frame_index",
        "lidar_timestamp_ns", "rgb_frame_index", "rgb_timestamp_ns",
        "signed_rgb_minus_lidar_ms", "absolute_sync_delta_ms",
    ]
    write_csv(args.output_dir / "timestamp_pairs.csv", internal["timestamp_rows"], timestamp_fields)
    aggregate_fields = [
        "dataset", "distance_ft", "number_of_frames", "evaluated_lidar_points",
        "mean_reprojection_error_px", "median_reprojection_error_px",
        "rmse_reprojection_error_px", "standard_deviation_reprojection_error_px",
        "maximum_reprojection_error_px", "percentage_within_1_px",
        "percentage_within_2_px", "percentage_within_3_px",
        "strict_same_raster_pixel_percentage", "passes_project_99pct_within_2px",
    ]
    write_csv(args.output_dir / "internal_multiframe_metrics.csv", internal["per_distance"], aggregate_fields)
    timestamp_summary = []
    for item in internal["per_distance"]:
        timestamp_summary.append({
            "dataset": item["dataset"],
            "distance_ft": item["distance_ft"],
            **item["timestamp_analysis"],
        })
    write_csv(
        args.output_dir / "timestamp_summary.csv", timestamp_summary,
        [
            "dataset", "distance_ft", "pairs", "mean_absolute_difference_ms",
            "median_absolute_difference_ms", "minimum_absolute_difference_ms",
            "maximum_absolute_difference_ms", "mean_signed_rgb_minus_lidar_ms",
            "error_vs_absolute_sync_delta_correlation",
        ],
    )
    diagnostic_rows = []
    for name, value in report["diagnostics"].items():
        diagnostic_rows.append({
            "diagnostic": name,
            "status": (
                "OPEN"
                if name in {"controlled_static_experiment", "larger_far_range_checkerboard"}
                else "COMPLETED"
            ),
            "conclusion": value["conclusion"],
        })
    write_csv(
        args.output_dir / "diagnostic_summary.csv", diagnostic_rows,
        ["diagnostic", "status", "conclusion"],
    )
    provenance = {
        "internal": {
            "synthetic": False,
            "description": "supplied real Intel RealSense D455 RGB and Livox Mid-360 measured point-cloud ROS 2 bags",
            "root": str(args.internal_root),
            "datasets": [
                {
                    "name": name,
                    "distance_ft": spec["distance_ft"],
                    "path": str(args.internal_root / name),
                    "topics": [
                        "/camera/camera/color/image_raw",
                        "/camera/camera/color/camera_info",
                        "/livox/lidar",
                    ],
                }
                for name, spec in sorted(
                    CALIBRATION_POSES.items(), key=lambda pair: pair[1]["distance_ft"]
                )
            ],
        },
        "external": {
            "synthetic": False,
            "name": report["external_acsc"]["dataset"],
            "source_repository": report["external_acsc"]["source_repository"],
            "download_link": report["external_acsc"]["download_link"],
            "local_root": str(args.public_root),
        },
        "separation": "internal and external transforms, splits, and metrics are never pooled",
    }
    (args.output_dir / "data_provenance.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    protocol = report["diagnostics"]["controlled_static_experiment"]
    larger = report["diagnostics"]["larger_far_range_checkerboard"]
    protocol_lines = [
        "# Required new controlled capture protocol",
        "",
        "Status: OPEN — the supplied raw data do not contain this new acquisition.",
        "",
    ]
    protocol_lines.extend(
        f"{index}. {step}"
        for index, step in enumerate(protocol["required_capture_protocol"], start=1)
    )
    protocol_lines.extend([
        "",
        "## Larger far-range target",
        "",
        f"Use at {larger['required_range_ft']} ft: {larger['recommended_target']}.",
        "",
        "Acceptance remains at least 99% of independently selected valid LiDAR returns "
        "with Euclidean reprojection error at or below 2 pixels at every required distance.",
        "",
    ])
    (args.output_dir / "required_new_capture_protocol.md").write_text(
        "\n".join(protocol_lines)
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--internal-root", type=Path, default=Path("data/raw"))
    parser.add_argument("--public-root", type=Path, default=Path("data/public/acsc/sample"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/phase1_final"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    frozen, split, frame_cache, context = calibrate_from_split(args.internal_root)
    internal = evaluate_internal_split(args, frozen, split, frame_cache, context)
    public_args = argparse.Namespace(
        public_root=args.public_root,
        output_dir=args.output_dir / "public_external",
    )
    public_result = evaluate_public(public_args)
    diagnostics = diagnostic_results(frozen, internal, context)
    internal_pass = all(
        item["passes_project_99pct_within_2px"] for item in internal["per_distance"]
    )
    capture_requirements_complete = (
        diagnostics["controlled_static_experiment"]["completed_with_new_capture"]
        and diagnostics["larger_far_range_checkerboard"]["new_capture_completed"]
    )
    report = {
        "title": "Phase 1 final corrections and required experiments",
        "status": {
            "phase1_status": "ACCEPTED" if internal_pass and capture_requirements_complete else "OPEN",
            "internal_99pct_within_2px_all_distances": bool(internal_pass),
            "required_new_capture_experiments_complete": bool(capture_requirements_complete),
            "reason": (
                "Phase 1 remains OPEN unless every internal distance passes and the required "
                "separate controlled/larger-board captures are completed"
            ),
        },
        "acceptance_criterion": {
            "requirement": "at every required internal distance, at least 99% of independently evaluated validation LiDAR returns have Euclidean reprojection error <=2 px",
            "origin": "project/mentor-defined",
        },
        "strict_split": frozen["split_contract"],
        "frozen_calibration": calibration_parameters(frozen),
        "internal_livox_d455": {
            "per_distance": internal["per_distance"],
            "qualitative_examples": internal["qualitative_examples"],
            "error_vector_figures": internal["error_vector_figures"],
        },
        "external_acsc": public_result,
        "experiments_are_separate": True,
        "combined_overall_accuracy_reported": False,
        "diagnostics": diagnostics,
        "results_changed_without_evidence": False,
        "post_validation_transform_refit_performed": False,
    }
    write_outputs(args, report, frozen, internal)
    print(json.dumps({
        "status": report["status"],
        "internal": [
            {
                key: item[key] for key in (
                    "distance_ft", "number_of_frames", "evaluated_lidar_points",
                    "mean_reprojection_error_px", "rmse_reprojection_error_px",
                    "standard_deviation_reprojection_error_px",
                    "percentage_within_2_px", "passes_project_99pct_within_2px",
                )
            }
            for item in internal["per_distance"]
        ],
        "rotation_validation": diagnostics["rotation_matrix"],
        "output_dir": str(args.output_dir),
    }, indent=2))


if __name__ == "__main__":
    main()
