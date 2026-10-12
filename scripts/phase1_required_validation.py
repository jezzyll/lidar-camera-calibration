#!/usr/bin/env python3
"""Run the mentor-requested non-circular Phase 1 calibration validation.

Evaluation returns are selected only from LiDAR geometry and reflectivity. RGB
checkerboard locations are used afterward as reference correspondences; RGB
masks never select or reject evaluation points. Source RGB arrays are neither
resized nor warped.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml
from rosbags.highlevel import AnyReader

from calibrate import (
    CALIBRATION_POSES,
    POINT_DTYPE,
    fit_lidar_board,
    image_rgb,
)
from checkerboard_mask_evaluation import (
    PUBLIC_DOWNLOAD_URL,
    PUBLIC_SOURCE_URL,
    camera_outer_polygon,
    checker_mask,
    detect_camera_corners,
    detect_lidar_board,
    solve_public_extrinsic,
)


INTERNAL_FRAME_COUNT = 10
BOARD_WIDTH_M = 0.30
BOARD_HEIGHT_M = 0.45
BOARD_MARGIN_M = 0.025 * 0.15
PLANE_RESIDUAL_LIMIT_M = 0.015
EXACT_EPSILON_PX = 1e-9


def pixel_sha256(image: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(image).tobytes()).hexdigest()


def camera_projection(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray,
                      camera_k: np.ndarray, distortion: np.ndarray):
    camera = points[:, :3] @ rotation.T + translation
    pixels = cv2.projectPoints(
        np.ascontiguousarray(points[:, :3], dtype=np.float64),
        cv2.Rodrigues(rotation)[0], translation, camera_k, distortion,
    )[0].reshape(-1, 2)
    return pixels, camera[:, 2]


def image_homography_reference(normalized_xy: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    unit_square = np.asarray(
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(unit_square, polygon.astype(np.float32))
    return cv2.perspectiveTransform(
        np.ascontiguousarray(normalized_xy, dtype=np.float64).reshape(-1, 1, 2),
        homography,
    ).reshape(-1, 2)


def planar_corner_reference(points_xyz: np.ndarray, lidar_corners: np.ndarray,
                            camera_corners: np.ndarray) -> np.ndarray:
    """Map physical plane coordinates to RGB pixels using 3D/2D board corners."""
    center = lidar_corners.mean(axis=0)
    _, _, basis = np.linalg.svd(lidar_corners - center, full_matrices=False)
    lidar_uv = (lidar_corners - center) @ basis[:2].T
    point_uv = (points_xyz - center) @ basis[:2].T
    homography, _ = cv2.findHomography(lidar_uv, camera_corners, method=0)
    if homography is None:
        raise RuntimeError("Could not form checkerboard plane reference homography")
    return cv2.perspectiveTransform(
        np.ascontiguousarray(point_uv, dtype=np.float64).reshape(-1, 1, 2), homography,
    ).reshape(-1, 2)


def error_metrics(actual: np.ndarray, expected: np.ndarray) -> tuple[dict, np.ndarray]:
    errors = np.linalg.norm(actual - expected, axis=1)
    same_raster_pixel = np.all(np.rint(actual) == np.rint(expected), axis=1)
    within_1 = errors <= 1.0
    within_2 = errors <= 2.0
    within_3 = errors <= 3.0
    metrics = {
        "evaluated_lidar_points": int(len(errors)),
        "mean_reprojection_error_px": float(np.mean(errors)),
        "median_reprojection_error_px": float(np.median(errors)),
        "rmse_reprojection_error_px": float(np.sqrt(np.mean(errors ** 2))),
        "standard_deviation_reprojection_error_px": float(np.std(errors)),
        "maximum_reprojection_error_px": float(np.max(errors)),
        "count_within_1_px": int(np.count_nonzero(within_1)),
        "count_within_2_px": int(np.count_nonzero(within_2)),
        "count_within_3_px": int(np.count_nonzero(within_3)),
        "percentage_within_1_px": float(100.0 * np.mean(within_1)),
        "percentage_within_2_px": float(100.0 * np.mean(within_2)),
        "percentage_within_3_px": float(100.0 * np.mean(within_3)),
        "strict_same_raster_pixel_percentage": float(100.0 * np.mean(same_raster_pixel)),
        "continuous_zero_error_percentage": float(100.0 * np.mean(errors <= EXACT_EPSILON_PX)),
        "passes_project_99pct_within_2px": bool(np.mean(within_2) >= 0.99),
    }
    return metrics, errors


def aggregate_metrics(records: list[dict]) -> dict:
    actual = np.concatenate([record["actual_pixels"] for record in records])
    expected = np.concatenate([record["expected_pixels"] for record in records])
    metrics, _ = error_metrics(actual, expected)
    metrics["number_of_frames"] = len(records)
    return metrics


def raw_lidar_roi_frames(bag: Path, spec: dict) -> tuple[list[dict], dict]:
    frames = []
    all_points = []
    with AnyReader([bag]) as reader:
        connection = next(c for c in reader.connections if c.topic == "/livox/lidar")
        for frame_index, (conn, timestamp, raw) in enumerate(
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
                "lidar_frame_index": frame_index,
                "lidar_timestamp_ns": int(timestamp),
                "roi_points": points,
            })
            if len(points):
                all_points.append(points)
    if not all_points:
        raise RuntimeError(f"No LiDAR checkerboard ROI points in {bag}")

    stacked = np.concatenate(all_points)
    board_rotation, board_center, plane_rmse, plane_inliers, coefficient = fit_lidar_board(
        stacked[:, :3], spec["y"], spec["z"],
    )
    design = np.c_[stacked[:, 1], stacked[:, 2], np.ones(len(stacked))]
    global_residual = stacked[:, 0] - design @ coefficient
    median = float(np.median(global_residual))

    valid_frames = []
    for frame in frames:
        points = frame["roi_points"]
        if not len(points):
            continue
        residual = points[:, 0] - np.c_[
            points[:, 1], points[:, 2], np.ones(len(points)),
        ] @ coefficient
        local = (points[:, :3] - board_center) @ board_rotation
        independent_keep = (
            (np.abs(residual - median) <= PLANE_RESIDUAL_LIMIT_M)
            & (local[:, 0] >= -BOARD_WIDTH_M / 2 + BOARD_MARGIN_M)
            & (local[:, 0] <= BOARD_WIDTH_M / 2 - BOARD_MARGIN_M)
            & (local[:, 1] >= -BOARD_HEIGHT_M / 2 + BOARD_MARGIN_M)
            & (local[:, 1] <= BOARD_HEIGHT_M / 2 - BOARD_MARGIN_M)
        )
        selected = points[independent_keep]
        selected_local = local[independent_keep]
        if len(selected) >= 4:
            normalized = np.c_[
                (selected_local[:, 0] + BOARD_WIDTH_M / 2) / BOARD_WIDTH_M,
                (selected_local[:, 1] + BOARD_HEIGHT_M / 2) / BOARD_HEIGHT_M,
            ]
            valid_frames.append({
                **frame,
                "selected_points": selected,
                "normalized_board_xy": normalized,
            })

    if len(valid_frames) < INTERNAL_FRAME_COUNT:
        raise RuntimeError(
            f"Only {len(valid_frames)} independently valid frames in {bag}; "
            f"need {INTERNAL_FRAME_COUNT}"
        )
    selected_indices = np.linspace(
        0, len(valid_frames) - 1, INTERNAL_FRAME_COUNT, dtype=int,
    )
    chosen = [valid_frames[index] for index in selected_indices]
    audit = {
        "selection_domain": "LiDAR coordinates only; no RGB pixels or masks",
        "initial_roi": {
            "abs_x_minus_center_lt_m": 0.075,
            "y_bounds_m": list(spec["y"]),
            "z_bounds_m": list(spec["z"]),
        },
        "plane_model": "x = a*y + b*z + c, robust LiDAR-only fit",
        "plane_coefficients": coefficient.tolist(),
        "maximum_plane_residual_m": PLANE_RESIDUAL_LIMIT_M,
        "physical_board_size_m": [BOARD_WIDTH_M, BOARD_HEIGHT_M],
        "interior_boundary_margin_m": BOARD_MARGIN_M,
        "eligible_lidar_frames": len(valid_frames),
        "evaluated_lidar_frames": len(chosen),
        "frame_sampling": "10 evenly spaced eligible LiDAR frames over the recording",
        "aggregate_board_roi_points": int(len(stacked)),
        "aggregate_plane_rmse_m": plane_rmse,
        "aggregate_plane_inliers": plane_inliers,
    }
    return chosen, audit


def nearest_rgb_frames(bag: Path, targets: list[int]) -> dict[int, dict]:
    best = {
        target: {"delta_ns": None, "timestamp_ns": None, "image": None, "frame_index": None}
        for target in targets
    }
    with AnyReader([bag]) as reader:
        connection = next(
            c for c in reader.connections if c.topic == "/camera/camera/color/image_raw"
        )
        for frame_index, (conn, timestamp, raw) in enumerate(
            reader.messages(connections=[connection])
        ):
            for target in targets:
                delta = abs(int(timestamp) - target)
                if best[target]["delta_ns"] is None or delta < best[target]["delta_ns"]:
                    message = reader.deserialize(raw, conn.msgtype)
                    best[target] = {
                        "delta_ns": delta,
                        "timestamp_ns": int(timestamp),
                        "image": image_rgb(message),
                        "frame_index": frame_index,
                    }
    if any(record["image"] is None for record in best.values()):
        raise RuntimeError(f"Could not synchronize RGB frames in {bag}")
    return best


def save_qualitative(image_rgb_array: np.ndarray, polygon: np.ndarray,
                     actual: np.ndarray, expected: np.ndarray, errors: np.ndarray,
                     title: str, prefix: Path, columns: int, rows: int) -> dict:
    prefix.parent.mkdir(parents=True, exist_ok=True)
    rgb_path = prefix.with_name(prefix.name + "_rgb.png")
    mask_path = prefix.with_name(prefix.name + "_mask.png")
    projection_path = prefix.with_name(prefix.name + "_projection.png")
    cv2.imwrite(str(rgb_path), cv2.cvtColor(image_rgb_array, cv2.COLOR_RGB2BGR))
    mask = checker_mask(
        cv2.cvtColor(image_rgb_array, cv2.COLOR_RGB2BGR), polygon, columns, rows,
    )
    cv2.imwrite(str(mask_path), mask)

    if len(actual) > 1200:
        indices = np.linspace(0, len(actual) - 1, 1200, dtype=int)
    else:
        indices = np.arange(len(actual))
    shown_actual = actual[indices]
    shown_expected = expected[indices]
    shown_errors = errors[indices]
    figure, axes = plt.subplots(1, 2, figsize=(13, 5.5), constrained_layout=True)
    axes[0].imshow(image_rgb_array)
    axes[0].plot(*np.vstack([polygon, polygon[0]]).T, color="lime", linewidth=2)
    axes[0].set_title("Untouched source RGB + checkerboard reference")
    axes[1].imshow(image_rgb_array)
    axes[1].plot(*np.vstack([polygon, polygon[0]]).T, color="lime", linewidth=2)
    line_step = max(1, len(shown_actual) // 150)
    for actual_point, expected_point in zip(
        shown_actual[::line_step], shown_expected[::line_step]
    ):
        axes[1].plot(
            [expected_point[0], actual_point[0]],
            [expected_point[1], actual_point[1]],
            color="white", linewidth=0.35, alpha=0.65,
        )
    axes[1].scatter(
        shown_expected[:, 0], shown_expected[:, 1], marker="x", s=12,
        c="white", linewidths=0.6, label="reference pixel",
    )
    scatter = axes[1].scatter(
        shown_actual[:, 0], shown_actual[:, 1], c=shown_errors, cmap="turbo",
        vmin=0.0, vmax=max(3.0, float(np.percentile(shown_errors, 99))),
        s=11, linewidths=0.15, edgecolors="black", label="projected LiDAR",
    )
    axes[1].set_title("Independent LiDAR returns colored by reprojection error")
    axes[1].legend(loc="lower right", fontsize=8)
    figure.colorbar(scatter, ax=axes[1], fraction=0.035, pad=0.015).set_label(
        "Euclidean reprojection error (px)"
    )
    for axis in axes:
        axis.axis("off")
    figure.suptitle(title, fontsize=14, fontweight="bold")
    figure.savefig(projection_path, dpi=170, bbox_inches="tight")
    plt.close(figure)
    return {
        "source_rgb": str(rgb_path),
        "source_rgb_pixel_sha256": pixel_sha256(image_rgb_array),
        "separate_visualization_mask": str(mask_path),
        "mask_used_for_point_selection_or_metrics": False,
        "projection_figure": str(projection_path),
        "source_resolution_wh": [int(image_rgb_array.shape[1]), int(image_rgb_array.shape[0])],
        "rgb_resized_or_warped_before_validation": False,
    }


def evaluate_internal(args, calibration: dict) -> dict:
    camera_k = np.asarray(calibration["camera"]["K"], dtype=np.float64)
    distortion = np.asarray(calibration["camera"]["D"], dtype=np.float64)
    rotation = np.asarray(calibration["extrinsic"]["R"], dtype=np.float64)
    translation = np.asarray(calibration["extrinsic"]["t_m"], dtype=np.float64)
    per_distance = []
    qualitative = []
    for name, spec in sorted(CALIBRATION_POSES.items(), key=lambda pair: pair[1]["distance_ft"]):
        bag = args.internal_root / name
        frames, selection_audit = raw_lidar_roi_frames(bag, spec)
        rgb_by_lidar_time = nearest_rgb_frames(
            bag, [frame["lidar_timestamp_ns"] for frame in frames],
        )
        polygon = np.asarray(
            calibration["calibration_observations"][name]["camera_board_corners_px"],
            dtype=np.float64,
        )
        frame_records = []
        for evaluation_index, frame in enumerate(frames):
            actual, depth = camera_projection(
                frame["selected_points"], rotation, translation, camera_k, distortion,
            )
            expected = image_homography_reference(frame["normalized_board_xy"], polygon)
            valid = (
                (depth > 0)
                & (actual[:, 0] >= 0) & (actual[:, 0] < calibration["camera"]["width"])
                & (actual[:, 1] >= 0) & (actual[:, 1] < calibration["camera"]["height"])
            )
            actual = actual[valid]
            expected = expected[valid]
            metrics, errors = error_metrics(actual, expected)
            synchronized = rgb_by_lidar_time[frame["lidar_timestamp_ns"]]
            record = {
                "evaluation_frame_index": evaluation_index,
                "lidar_frame_index": frame["lidar_frame_index"],
                "lidar_timestamp_ns": frame["lidar_timestamp_ns"],
                "rgb_frame_index": synchronized["frame_index"],
                "rgb_timestamp_ns": synchronized["timestamp_ns"],
                "absolute_sync_delta_ms": synchronized["delta_ns"] / 1e6,
                **metrics,
                "actual_pixels": actual,
                "expected_pixels": expected,
                "errors": errors,
            }
            frame_records.append(record)
            if evaluation_index in (2, 7):
                prefix = args.output_dir / "qualitative" / (
                    f"internal_{str(spec['distance_ft']).replace('.', '_')}ft_"
                    f"frame_{evaluation_index:02d}"
                )
                evidence = save_qualitative(
                    synchronized["image"], polygon, actual, expected, errors,
                    f"Internal {spec['distance_ft']:g} ft — synchronized frame {evaluation_index}",
                    prefix, 12, 18,
                )
                qualitative.append({
                    "experiment": "internal",
                    "dataset": name,
                    "distance_ft": spec["distance_ft"],
                    "evaluation_frame_index": evaluation_index,
                    **evidence,
                })
        aggregate = aggregate_metrics(frame_records)
        public_frame_records = [
            {key: value for key, value in record.items() if key not in {
                "actual_pixels", "expected_pixels", "errors",
            }}
            for record in frame_records
        ]
        per_distance.append({
            "dataset": name,
            "distance_ft": spec["distance_ft"],
            "camera_reference_source": calibration["calibration_observations"][name][
                "camera_boundary_source"
            ],
            "selection_audit": selection_audit,
            **aggregate,
            "per_frame": public_frame_records,
        })
    return {"per_distance": per_distance, "qualitative_examples": qualitative}


def evaluate_public(args) -> dict:
    root = args.public_root
    camera_k = np.loadtxt(root / "intrinsic").astype(np.float64)
    distortion = np.loadtxt(root / "distortion").reshape(-1).astype(np.float64)
    frames = []
    for image_path in sorted((root / "images").glob("*.png")):
        index = int(image_path.stem)
        image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        point_path = root / "pcds" / f"{index:06d}.npy"
        roi_path = root / "ROIs" / f"{index:06d}.txt"
        points = np.load(point_path)
        lidar = detect_lidar_board(points, np.loadtxt(roi_path))
        frames.append({
            "index": index,
            "image_path": image_path,
            "point_path": point_path,
            "image_bgr": image_bgr,
            "camera_corners": detect_camera_corners(image_bgr),
            "lidar": lidar,
            "camera_k": camera_k,
            "distortion": distortion,
        })
    test_indices = {2, 5, 8, 11, 14, 17, 20}
    train_indices = {item["index"] for item in frames} - test_indices
    rotation, translation, pnp_inliers = solve_public_extrinsic(frames, train_indices)
    records = []
    qualitative = []
    for item in frames:
        points = item["lidar"]["points"]
        actual, depth = camera_projection(points, rotation, translation, camera_k, distortion)
        expected = planar_corner_reference(
            points[:, :3], item["lidar"]["corners"], item["camera_corners"],
        )
        valid = (
            (depth > 0)
            & (actual[:, 0] >= 0) & (actual[:, 0] < item["image_bgr"].shape[1])
            & (actual[:, 1] >= 0) & (actual[:, 1] < item["image_bgr"].shape[0])
        )
        actual = actual[valid]
        expected = expected[valid]
        metrics, errors = error_metrics(actual, expected)
        record = {
            "frame_index": item["index"],
            "split": "held_out_test" if item["index"] in test_indices else "calibration_train",
            **metrics,
            "actual_pixels": actual,
            "expected_pixels": expected,
            "errors": errors,
        }
        records.append(record)
        if item["index"] in (0, 10, 20):
            polygon = camera_outer_polygon(item["camera_corners"])
            prefix = args.output_dir / "qualitative" / f"public_acsc_{item['index']:06d}"
            evidence = save_qualitative(
                cv2.cvtColor(item["image_bgr"], cv2.COLOR_BGR2RGB), polygon,
                actual, expected, errors, f"Public ACSC frame {item['index']:06d}",
                prefix, 8, 6,
            )
            qualitative.append({
                "experiment": "public_external",
                "frame_index": item["index"],
                "split": record["split"],
                **evidence,
            })
    train_records = [record for record in records if record["split"] == "calibration_train"]
    test_records = [record for record in records if record["split"] == "held_out_test"]
    clean_records = [
        {key: value for key, value in record.items() if key not in {
            "actual_pixels", "expected_pixels", "errors",
        }}
        for record in records
    ]
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return {
        "dataset": "ACSC public real checkerboard sample",
        "source_repository": PUBLIC_SOURCE_URL,
        "download_link": PUBLIC_DOWNLOAD_URL,
        "synthetic": False,
        "data_description": "real RGB PNG images and measured N×4 XYZ-reflectance arrays",
        "selection_independence": (
            "point-cloud ROI, plane, physical size, and LiDAR reflectivity only; "
            "RGB masks are not used"
        ),
        "calibration_train_indices": sorted(train_indices),
        "held_out_test_indices": sorted(test_indices),
        "camera": {
            "resolution_wh": [
                int(frames[0]["image_bgr"].shape[1]), int(frames[0]["image_bgr"].shape[0]),
            ],
            "K": camera_k.tolist(),
            "D": distortion.tolist(),
        },
        "extrinsic": {
            "direction": "p_camera = R_camera_lidar * p_lidar + t_camera_lidar",
            "R_camera_lidar": rotation.tolist(),
            "t_camera_lidar_m": translation.tolist(),
            "T_camera_lidar": transform.tolist(),
            "training_pnp_inlier_corners": pnp_inliers,
        },
        "training_aggregate": aggregate_metrics(train_records),
        "held_out_test_aggregate": aggregate_metrics(test_records),
        "per_frame": clean_records,
        "qualitative_examples": qualitative,
    }


def calibration_parameters(calibration: dict) -> dict:
    return {
        "camera_intrinsic_matrix_K": calibration["camera"]["K"],
        "distortion_model": calibration["camera"]["distortion_model"],
        "distortion_coefficients_D": calibration["camera"]["D"],
        "image_resolution_wh": [
            calibration["camera"]["width"], calibration["camera"]["height"],
        ],
        "lidar_to_camera_rotation_R": calibration["extrinsic"]["R"],
        "lidar_to_camera_translation_m": calibration["extrinsic"]["t_m"],
        "T_camera_lidar": calibration["extrinsic"]["T_camera_lidar"],
        "coordinate_frame_convention": {
            "equation": "p_camera_color_optical = R_camera_lidar * p_livox + t_camera_lidar",
            "camera_optical_axes": "+X right, +Y down, +Z forward",
            "translation_unit": "metres",
            "pixel_origin": "top-left; u right, v down",
        },
    }


def write_internal_csv(path: Path, rows: list[dict]):
    fields = [
        "dataset", "distance_ft", "number_of_frames", "evaluated_lidar_points",
        "mean_reprojection_error_px", "rmse_reprojection_error_px",
        "median_reprojection_error_px", "standard_deviation_reprojection_error_px",
        "percentage_within_1_px",
        "percentage_within_2_px", "percentage_within_3_px",
        "maximum_reprojection_error_px", "strict_same_raster_pixel_percentage",
        "passes_project_99pct_within_2px",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows([{field: row[field] for field in fields} for row in rows])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--public-root", type=Path, default=Path("data/public/acsc/sample"))
    parser.add_argument("--internal-root", type=Path, default=Path("data/raw"))
    parser.add_argument("--calibration", type=Path, default=Path("outputs/calibration_result.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/phase1_validation"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    calibration = json.loads(args.calibration.read_text())

    public_result = evaluate_public(args)
    internal_result = evaluate_internal(args, calibration)
    internal_pass = all(
        row["passes_project_99pct_within_2px"]
        for row in internal_result["per_distance"]
    )
    public_pass = public_result["held_out_test_aggregate"][
        "passes_project_99pct_within_2px"
    ]
    report = {
        "title": "Phase 1 calibration required corrections and validation",
        "status": {
            "phase1_accepted": bool(internal_pass),
            "reason": (
                "all internal distances pass the project criterion"
                if internal_pass else
                "one or more internal distances fail the project-defined 99% within 2 px criterion"
            ),
            "public_external_held_out_pass": bool(public_pass),
        },
        "definitions": {
            "reprojection_error": (
                "e_i = sqrt((u_i-u_i*)^2 + (v_i-v_i*)^2), where (u_i,v_i) is "
                "the calibrated projection and (u_i*,v_i*) is the independently "
                "derived camera-checkerboard reference for the same physical board coordinate"
            ),
            "strict_alignment": (
                "same nearest integer raster pixel: round(u_i)=round(u_i*) and "
                "round(v_i)=round(v_i*)"
            ),
            "continuous_exact_match": f"e_i <= {EXACT_EPSILON_PX:g} px numerical epsilon",
            "threshold_percentages": "Euclidean e_i <= 1, 2, or 3 pixels",
        },
        "acceptance_criterion": {
            "requirement": (
                "at least 99% of independently selected valid LiDAR returns must have "
                "Euclidean reprojection error <= 2 pixels at every internal distance"
            ),
            "origin": "project/mentor-defined acceptance criterion",
            "claimed_as_external_standard_or_paper_threshold": False,
        },
        "anti_circularity": {
            "rgb_mask_used_to_select_evaluation_lidar_points": False,
            "selection": (
                "LiDAR-only metric ROI, robust board plane residual, known physical board "
                "dimensions, and interior margin; public fitting additionally uses LiDAR reflectivity"
            ),
            "rgb_usage_after_selection": (
                "camera checkerboard corners/boundary provide reference pixels only after "
                "the LiDAR evaluation set is frozen"
            ),
        },
        "image_integrity": {
            "source_rgb_resized": False,
            "source_rgb_warped": False,
            "source_rgb_regenerated": False,
            "synthetic_images_or_lidar_points": False,
            "masks_saved_separately_and_used_only_for_visualization_annotation": True,
        },
        "calibration_parameters_internal": calibration_parameters(calibration),
        "public_external_experiment": public_result,
        "internal_d455_livox_experiment": internal_result,
        "experiment_separation": (
            "public and internal results are reported separately; no combined overall accuracy is calculated"
        ),
        "limitations": [
            "The internal bags contain one mostly static board pose per distance; multiple frames are repeated measurements, not independent poses.",
            "The 14.5 ft and 22.2 ft camera checkerboards are too small for reliable automatic 11x17 corner detection, so their original-frame outer boundaries are the camera references.",
            "All three internal distances participated in the saved calibration refinement, so these measurements test in-recording alignment rather than unseen-rig generalization.",
        ],
    }
    report_path = args.output_dir / "phase1_validation_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    csv_path = args.output_dir / "internal_multiframe_metrics.csv"
    write_internal_csv(csv_path, internal_result["per_distance"])
    yaml_path = args.output_dir / "internal_calibration_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(
        report["calibration_parameters_internal"], sort_keys=False,
    ))
    print(json.dumps({
        "status": report["status"],
        "internal_multiframe": internal_result["per_distance"],
        "public_held_out": public_result["held_out_test_aggregate"],
        "report": str(report_path),
        "csv": str(csv_path),
        "calibration_yaml": str(yaml_path),
    }, indent=2))


if __name__ == "__main__":
    main()
