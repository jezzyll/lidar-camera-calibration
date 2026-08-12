#!/usr/bin/env python3
"""Deterministic extrinsic sensitivity analysis on the calibrated board points.

This adopts the supplied PDF's perturbation model but measures pixel drift on
this project's real LiDAR checkerboard returns. It does not infer detector mAP.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from calibrate_reflectivity import aggregate_board_points, robust_plane


ROTATION_LEVELS_DEG = (0.1, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0)
TRANSLATION_LEVELS_M = (0.01, 0.02, 0.05, 0.10, 0.20)


def project(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray,
            camera_k: np.ndarray, distortion: np.ndarray):
    pixels = cv2.projectPoints(
        np.ascontiguousarray(points), cv2.Rodrigues(rotation)[0], translation, camera_k, distortion
    )[0].reshape(-1, 2)
    camera_points = (rotation @ points.T).T + translation
    return pixels, camera_points[:, 2] > 0


def perturbation(parameter: str, magnitude: float):
    delta_rotation = np.eye(3)
    delta_translation = np.zeros(3)
    if parameter in ("roll", "pitch", "yaw"):
        axis_index = {"roll": 0, "pitch": 1, "yaw": 2}[parameter]
        angles = np.zeros(3)
        angles[axis_index] = magnitude
        delta_rotation = Rotation.from_euler("xyz", angles, degrees=True).as_matrix()
    else:
        delta_translation[{"tx": 0, "ty": 1, "tz": 2}[parameter]] = magnitude
    return delta_rotation, delta_translation


def bbox_iou(reference: np.ndarray, candidate: np.ndarray):
    ref_min, ref_max = reference.min(axis=0), reference.max(axis=0)
    candidate_min, candidate_max = candidate.min(axis=0), candidate.max(axis=0)
    intersection = np.maximum(0.0, np.minimum(ref_max, candidate_max) - np.maximum(ref_min, candidate_min))
    intersection_area = float(np.prod(intersection))
    ref_area = float(np.prod(ref_max - ref_min))
    candidate_area = float(np.prod(candidate_max - candidate_min))
    union = ref_area + candidate_area - intersection_area
    return intersection_area / union if union > 0 else 0.0


def signed_measurement(points, baseline_uv, rotation, translation, camera_k, distortion,
                       parameter, signed_magnitude):
    delta_rotation, delta_translation = perturbation(parameter, signed_magnitude)
    # Matches the supplied reference: X'_c = (Delta R R) X_l + (t + Delta t).
    changed_rotation = delta_rotation @ rotation
    changed_translation = translation + delta_translation
    changed_uv, in_front = project(points, changed_rotation, changed_translation, camera_k, distortion)
    finite = in_front & np.all(np.isfinite(changed_uv), axis=1)
    errors = np.linalg.norm(changed_uv[finite] - baseline_uv[finite], axis=1)
    return {
        "valid_points": int(finite.sum()),
        "mean_pixel_drift": float(errors.mean()),
        "reprojection_rmse_px": float(np.sqrt(np.mean(errors ** 2))),
        "projected_bbox_iou": float(bbox_iou(baseline_uv[finite], changed_uv[finite])),
    }


def analyze(points, rotation, translation, camera_k, distortion):
    baseline_uv, in_front = project(points, rotation, translation, camera_k, distortion)
    valid = in_front & np.all(np.isfinite(baseline_uv), axis=1)
    points, baseline_uv = points[valid], baseline_uv[valid]
    rows = []
    for parameter in ("pitch", "roll", "yaw", "tx", "ty", "tz"):
        levels = ROTATION_LEVELS_DEG if parameter in ("pitch", "roll", "yaw") else TRANSLATION_LEVELS_M
        unit = "deg" if parameter in ("pitch", "roll", "yaw") else "m"
        for magnitude in levels:
            signed = [
                signed_measurement(
                    points, baseline_uv, rotation, translation, camera_k, distortion,
                    parameter, sign * magnitude,
                )
                for sign in (-1.0, 1.0)
            ]
            rows.append({
                "parameter": parameter,
                "magnitude": magnitude,
                "unit": unit,
                "mean_pixel_drift": float(np.mean([item["mean_pixel_drift"] for item in signed])),
                "reprojection_rmse_px": float(np.mean([item["reprojection_rmse_px"] for item in signed])),
                "projected_bbox_iou": float(np.mean([item["projected_bbox_iou"] for item in signed])),
                "valid_points": int(min(item["valid_points"] for item in signed)),
            })
    return rows, len(points)


def first_failure(rows, threshold):
    failures = {}
    for parameter in ("pitch", "roll", "yaw", "tx", "ty", "tz"):
        candidates = [row for row in rows if row["parameter"] == parameter]
        failure = next((row for row in candidates if row["projected_bbox_iou"] < threshold), None)
        failures[parameter] = failure
    return failures


def save_csv(rows, path):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_plot(rows, threshold, path):
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    colors = {"pitch": "#d62728", "roll": "#ff7f0e", "yaw": "#1f77b4",
              "tx": "#9467bd", "ty": "#2ca02c", "tz": "#17becf"}
    for parameter in ("pitch", "roll", "yaw"):
        selected = [row for row in rows if row["parameter"] == parameter]
        x = [row["magnitude"] for row in selected]
        axes[0, 0].plot(x, [row["reprojection_rmse_px"] for row in selected], "o-",
                        label=parameter, color=colors[parameter])
        axes[0, 1].plot(x, [row["projected_bbox_iou"] for row in selected], "o-",
                        label=parameter, color=colors[parameter])
    for parameter in ("tx", "ty", "tz"):
        selected = [row for row in rows if row["parameter"] == parameter]
        x = [row["magnitude"] for row in selected]
        axes[1, 0].plot(x, [row["reprojection_rmse_px"] for row in selected], "o-",
                        label=parameter, color=colors[parameter])
        axes[1, 1].plot(x, [row["projected_bbox_iou"] for row in selected], "o-",
                        label=parameter, color=colors[parameter])
    axes[0, 0].set_title("Rotation perturbation: pixel RMSE")
    axes[0, 1].set_title("Rotation perturbation: projected-board IoU")
    axes[1, 0].set_title("Translation perturbation: pixel RMSE")
    axes[1, 1].set_title("Translation perturbation: projected-board IoU")
    for axis in axes[:, 0]:
        axis.set_ylabel("pixels")
    for axis in axes[0]:
        axis.set_xlabel("absolute perturbation (degrees)")
    for axis in axes[1]:
        axis.set_xlabel("absolute perturbation (metres)")
    for axis in axes[:, 1]:
        axis.set_ylabel("IoU")
        axis.axhline(threshold, color="black", linestyle="--", linewidth=1, label=f"failure < {threshold:.2f}")
        axis.set_ylim(-0.02, 1.02)
    for axis in axes.flat:
        axis.grid(alpha=0.25)
        axis.legend()
    fig.suptitle("Measured Extrinsic Sensitivity — Livox/D455 Checkerboard", fontsize=16, fontweight="bold")
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=Path("data/raw_new/new1"))
    parser.add_argument("--calibration", type=Path, default=Path("outputs/calibration_reflectivity_result.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/error_analysis"))
    parser.add_argument("--failure-iou", type=float, default=0.50)
    parser.add_argument("--max-points", type=int, default=50000)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    calibration = json.loads(args.calibration.read_text())
    camera_k = np.asarray(calibration["camera"]["K"], dtype=np.float64)
    distortion = np.asarray(calibration["camera"]["D"], dtype=np.float64)
    rotation = np.asarray(calibration["extrinsic"]["R"], dtype=np.float64)
    translation = np.asarray(calibration["extrinsic"]["t_m"], dtype=np.float64)
    cloud = aggregate_board_points(args.bag)
    _, keep, _ = robust_plane(cloud)
    points = cloud[keep, :3]
    if len(points) > args.max_points:
        indices = np.linspace(0, len(points) - 1, args.max_points, dtype=int)
        points = points[indices]

    rows, point_count = analyze(points, rotation, translation, camera_k, distortion)
    failures = first_failure(rows, args.failure_iou)
    csv_path = args.output_dir / "dense_perturbation.csv"
    plot_path = args.output_dir / "calibration_sensitivity.png"
    report_path = args.output_dir / "calibration_error_report.json"
    save_csv(rows, csv_path)
    save_plot(rows, args.failure_iou, plot_path)
    report = {
        "source_bag": str(args.bag),
        "calibration": str(args.calibration),
        "perturbation_model": "R_changed = DeltaR @ R; t_changed = t + DeltaT",
        "points_evaluated": point_count,
        "failure_definition": f"projected checkerboard bounding-box IoU < {args.failure_iou}",
        "first_tested_failure": failures,
        "rows": rows,
        "limitations": [
            "Metrics measure board-projection sensitivity on calibration data, not vehicle detection accuracy.",
            "IoU is the axis-aligned projected checkerboard bounding box, not a detector-predicted vehicle box.",
            "Positive and negative perturbation metrics are averaged at each magnitude.",
        ],
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({
        "points_evaluated": point_count,
        "first_tested_failure": failures,
        "saved": [str(csv_path), str(plot_path), str(report_path)],
    }, indent=2))


if __name__ == "__main__":
    main()
