#!/usr/bin/env python3
"""Neural-network-assisted checkerboard reflectivity visualization.

The geometric calibration remains the saved LiDAR-to-camera solvePnP result.
This script trains a small MLP on LiDAR-only reflectivity features to classify
checkerboard cells as reflective white or absorptive black, evaluates it with
held-out cells, and uses its predictions to select measured LiDAR returns for
a full-board projection.  Camera brightness is used only for final validation;
it is never an input feature or a display-selection mask.
"""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from sklearn.metrics import accuracy_score, confusion_matrix, f1_score, precision_score, recall_score
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.neural_network import MLPClassifier
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

from calibrate_reflectivity import (
    BOARD_LIMITS,
    PATTERN,
    RASTER_RESOLUTION_M,
    aggregate_board_points,
    camera_data,
    detect_camera_corners,
    make_reflectivity_raster,
    robust_plane,
)


CELL_PIXELS = 32
PATCH_MARGIN_PIXELS = 5
FEATURE_NAMES = [
    "mean", "std", "p05", "p10", "p25", "p50", "p75", "p90", "p95",
    "fraction_gt_100", "fraction_gt_140", "fraction_gt_180",
    "hist_0_16", "hist_16_32", "hist_32_48", "hist_48_64",
    "hist_64_80", "hist_80_96", "hist_96_112", "hist_112_128",
    "hist_128_160", "hist_160_192", "hist_192_224", "hist_224_256",
]


def detect_lidar_corners(raster: np.ndarray):
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCornersSB(raster, PATTERN, flags)
    if not ok:
        raise RuntimeError("12x19 checkerboard was not detected in the LiDAR reflectivity raster")
    return corners.reshape(-1, 2).astype(np.float32)


def canonical_reflectivity(raster: np.ndarray, lidar_corners: np.ndarray):
    rows, cols = PATTERN[1], PATTERN[0]
    ideal_grid = np.stack(
        np.meshgrid(np.arange(cols), np.arange(rows)), axis=-1
    ).reshape(-1, 2).astype(np.float32)
    # One-cell offset includes the physical squares outside the inner corners.
    canonical_corners = (ideal_grid + 1.0) * CELL_PIXELS
    raster_to_canonical, _ = cv2.findHomography(lidar_corners, canonical_corners)
    if raster_to_canonical is None:
        raise RuntimeError("Could not rectify the LiDAR checkerboard raster")
    canonical = cv2.warpPerspective(
        raster,
        raster_to_canonical,
        ((cols + 1) * CELL_PIXELS, (rows + 1) * CELL_PIXELS),
        flags=cv2.INTER_LINEAR,
    )
    return canonical, raster_to_canonical


def patch_features(values: np.ndarray):
    values = values.astype(np.float64).ravel()
    histogram = np.histogram(
        values, bins=[0, 16, 32, 48, 64, 80, 96, 112, 128, 160, 192, 224, 256]
    )[0].astype(np.float64) / len(values)
    return np.r_[
        values.mean(),
        values.std(),
        np.percentile(values, [5, 10, 25, 50, 75, 90, 95]),
        [(values > level).mean() for level in (100, 140, 180)],
        histogram,
    ]


def cell_dataset(canonical: np.ndarray):
    rows, cols = PATTERN[1] + 1, PATTERN[0] + 1
    features = []
    parity = []
    coordinates = []
    margin = PATCH_MARGIN_PIXELS
    for row in range(rows):
        for col in range(cols):
            patch = canonical[
                row * CELL_PIXELS + margin:(row + 1) * CELL_PIXELS - margin,
                col * CELL_PIXELS + margin:(col + 1) * CELL_PIXELS - margin,
            ]
            features.append(patch_features(patch))
            parity.append((row + col) % 2)
            coordinates.append((row, col))
    features = np.asarray(features, dtype=np.float64)
    parity = np.asarray(parity, dtype=int)
    # The checker alternates exactly. Determine which parity is reflective-white
    # using only the LiDAR raster's mean reflectivity, not camera brightness.
    parity_means = [float(features[parity == value, 0].mean()) for value in (0, 1)]
    white_parity = int(np.argmax(parity_means))
    labels = (parity == white_parity).astype(int)
    return features, labels, coordinates, white_parity, parity_means


def make_model(random_state: int):
    return make_pipeline(
        StandardScaler(),
        MLPClassifier(
            hidden_layer_sizes=(16, 8),
            activation="relu",
            solver="lbfgs",
            alpha=0.02,
            max_iter=3000,
            random_state=random_state,
        ),
    )


def evaluate_model(features: np.ndarray, labels: np.ndarray, random_state: int):
    folds = StratifiedKFold(n_splits=5, shuffle=True, random_state=random_state)
    predictions = cross_val_predict(make_model(random_state), features, labels, cv=folds)
    matrix = confusion_matrix(labels, predictions, labels=[0, 1])
    return predictions, {
        "method": "5-fold stratified held-out-cell cross-validation",
        "cells_evaluated": int(len(labels)),
        "accuracy": float(accuracy_score(labels, predictions)),
        "precision_white": float(precision_score(labels, predictions)),
        "recall_white": float(recall_score(labels, predictions)),
        "f1_white": float(f1_score(labels, predictions)),
        "confusion_matrix_rows_true_black_white": matrix.tolist(),
    }


def point_cell_coordinates(points: np.ndarray, raster_to_canonical: np.ndarray):
    y_max = BOARD_LIMITS["y"][1]
    z_max = BOARD_LIMITS["z"][1]
    raster_uv = np.c_[
        (y_max - points[:, 1]) / RASTER_RESOLUTION_M,
        (z_max - points[:, 2]) / RASTER_RESOLUTION_M,
    ].astype(np.float32)
    canonical_uv = cv2.perspectiveTransform(
        raster_uv.reshape(-1, 1, 2), raster_to_canonical
    ).reshape(-1, 2)
    columns = np.floor(canonical_uv[:, 0] / CELL_PIXELS).astype(int)
    rows = np.floor(canonical_uv[:, 1] / CELL_PIXELS).astype(int)
    fractions = canonical_uv / CELL_PIXELS - np.c_[columns, rows]
    return rows, columns, fractions


def select_nn_white_returns(
    points: np.ndarray,
    raster_to_canonical: np.ndarray,
    cell_predictions: np.ndarray,
    intensity_threshold: float,
    boundary_margin: float,
):
    rows, columns, fractions = point_cell_coordinates(points, raster_to_canonical)
    max_row, max_col = cell_predictions.shape
    valid = (
        (rows >= 0) & (rows < max_row) & (columns >= 0) & (columns < max_col)
        & (fractions[:, 0] >= boundary_margin)
        & (fractions[:, 0] <= 1.0 - boundary_margin)
        & (fractions[:, 1] >= boundary_margin)
        & (fractions[:, 1] <= 1.0 - boundary_margin)
        & (points[:, 3] >= intensity_threshold)
    )
    valid_indices = np.flatnonzero(valid)
    selected = np.zeros(len(points), dtype=bool)
    selected[valid_indices] = cell_predictions[rows[valid_indices], columns[valid_indices]] == 1
    return points[selected]


def project(points: np.ndarray, calibration: dict, camera_k: np.ndarray,
            distortion: np.ndarray, image_shape: tuple[int, ...]):
    rotation = np.asarray(calibration["extrinsic"]["R"], dtype=np.float64)
    translation = np.asarray(calibration["extrinsic"]["t_m"], dtype=np.float64)
    camera_points = (rotation @ points[:, :3].T).T + translation
    pixels = cv2.projectPoints(
        np.ascontiguousarray(points[:, :3]), cv2.Rodrigues(rotation)[0], translation,
        camera_k, distortion,
    )[0].reshape(-1, 2)
    valid = (
        (camera_points[:, 2] > 0)
        & (pixels[:, 0] >= 0) & (pixels[:, 0] < image_shape[1])
        & (pixels[:, 1] >= 0) & (pixels[:, 1] < image_shape[0])
    )
    return pixels[valid], points[valid]


def unique_pixels(pixels: np.ndarray, intensity: np.ndarray, width: int):
    rounded = np.rint(pixels).astype(int)
    order = np.argsort(intensity)[::-1]
    keys = rounded[order, 1] * width + rounded[order, 0]
    _, first = np.unique(keys, return_index=True)
    chosen = order[first]
    return pixels[chosen], intensity[chosen]


def save_projection(image: np.ndarray, pixels: np.ndarray, intensity: np.ndarray,
                    metrics: dict, corner_rmse: float, output: Path):
    shown_pixels, shown_intensity = unique_pixels(pixels, intensity, image.shape[1])
    figure, axis = plt.subplots(figsize=(13, 8), constrained_layout=True)
    axis.imshow(image)
    scatter = axis.scatter(
        shown_pixels[:, 0], shown_pixels[:, 1], c=shown_intensity, s=10,
        cmap="turbo", vmin=metrics["selection"]["intensity_threshold"],
        vmax=max(25.0, float(shown_intensity.max())), linewidths=0, alpha=0.92,
    )
    axis.set_xlim(540, 840)
    axis.set_ylim(480, 130)
    axis.set_title(
        "Neural-network-assisted full-board LiDAR projection\n"
        f"MLP held-out cell accuracy {metrics['cross_validation']['accuracy'] * 100:.2f}%  |  "
        f"white cells {metrics['projection_validation']['white_cells_reached']}/130  |  "
        f"corner RMSE {corner_rmse:.3f} px",
        fontsize=14,
        fontweight="bold",
    )
    axis.text(
        0.5, -0.025,
        "LiDAR-only MLP cell selection; no camera-white display mask. "
        "Accuracy is held-out cells from this calibration recording.",
        transform=axis.transAxes, ha="center", va="top", fontsize=9.5,
        color="#7a1f1f",
    )
    axis.axis("off")
    colorbar = figure.colorbar(scatter, ax=axis, fraction=0.035, pad=0.02)
    colorbar.set_label("measured LiDAR reflectivity / intensity")
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)
    return int(len(shown_pixels))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=Path("data/raw_new/new1"))
    parser.add_argument(
        "--calibration", type=Path,
        default=Path("outputs/calibration_reflectivity_result.json"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/nn_calibration"))
    parser.add_argument("--intensity-threshold", type=float, default=20.0)
    parser.add_argument("--cell-boundary-margin", type=float, default=0.22)
    parser.add_argument("--random-state", type=int, default=42)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    image, camera_k, distortion, _ = camera_data(args.bag)
    camera_grid = detect_camera_corners(image)
    cloud = aggregate_board_points(args.bag)
    _, plane_keep, plane_rmse = robust_plane(cloud)
    board = cloud[plane_keep]
    raster = make_reflectivity_raster(board)
    lidar_corners = detect_lidar_corners(raster)
    canonical, raster_to_canonical = canonical_reflectivity(raster, lidar_corners)
    features, labels, coordinates, white_parity, parity_means = cell_dataset(canonical)

    _, evaluation = evaluate_model(features, labels, args.random_state)
    model = make_model(args.random_state)
    model.fit(features, labels)
    predictions = model.predict(features)
    probabilities = model.predict_proba(features)[:, 1]
    cell_predictions = predictions.reshape(PATTERN[1] + 1, PATTERN[0] + 1)

    selected = select_nn_white_returns(
        board, raster_to_canonical, cell_predictions,
        args.intensity_threshold, args.cell_boundary_margin,
    )
    calibration = json.loads(args.calibration.read_text())
    pixels, selected = project(selected, calibration, camera_k, distortion, image.shape)

    grayscale = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    rounded = np.rint(pixels).astype(int)
    on_camera_white = grayscale[rounded[:, 1], rounded[:, 0]] > 128
    camera_homography, _ = cv2.findHomography(
        camera_grid.reshape(-1, 2),
        np.stack(
            np.meshgrid(np.arange(PATTERN[0]), np.arange(PATTERN[1])), axis=-1
        ).reshape(-1, 2).astype(np.float64),
    )
    checker_xy = cv2.perspectiveTransform(
        pixels.reshape(-1, 1, 2), camera_homography
    ).reshape(-1, 2)
    hit_cells = {
        (int(np.floor(y)), int(np.floor(x))) for x, y in checker_xy
        if -1 <= x < PATTERN[0] and -1 <= y < PATTERN[1]
    }
    predicted_white_cells = {
        (row - 1, col - 1)
        for index, (row, col) in enumerate(coordinates) if predictions[index] == 1
    }
    white_cells_reached = len(hit_cells & predicted_white_cells)

    mlp = model.named_steps["mlpclassifier"]
    parameter_count = int(sum(weights.size for weights in mlp.coefs_)
                          + sum(bias.size for bias in mlp.intercepts_))
    report = {
        "purpose": "NN-assisted LiDAR reflectivity cell classification and visualization",
        "geometric_calibration": {
            "source": str(args.calibration),
            "method": "checker correspondences plus iterative solvePnP; unchanged by the MLP",
            "checker_corner_reprojection_rmse_px": calibration["quality"][
                "checker_corner_reprojection_rmse_px"
            ],
            "lidar_plane_rmse_m": plane_rmse,
        },
        "model": {
            "type": "scikit-learn MLPClassifier (feed-forward neural network)",
            "input_features": FEATURE_NAMES,
            "input_feature_count": int(features.shape[1]),
            "architecture": [int(features.shape[1]), 16, 8, 1],
            "activation": "ReLU hidden layers; logistic binary output",
            "optimizer": "L-BFGS",
            "l2_alpha": 0.02,
            "trainable_parameters": parameter_count,
            "training_cells": int(len(labels)),
            "label_source": (
                "known alternating checker pattern; reflective-white parity selected by "
                "LiDAR-only mean raster reflectivity"
            ),
            "white_parity": white_parity,
            "mean_lidar_reflectivity_by_parity": parity_means,
        },
        "cross_validation": evaluation,
        "all_cell_fit": {
            "accuracy": float(accuracy_score(labels, predictions)),
            "predicted_white_cells": int(predictions.sum()),
            "predicted_black_cells": int(len(predictions) - predictions.sum()),
            "minimum_predicted_white_probability_on_true_white": float(probabilities[labels == 1].min()),
            "maximum_predicted_white_probability_on_true_black": float(probabilities[labels == 0].max()),
        },
        "selection": {
            "input_modality": "LiDAR reflectivity raster and measured LiDAR points only",
            "intensity_threshold": args.intensity_threshold,
            "cell_boundary_margin_fraction": args.cell_boundary_margin,
            "camera_white_mask_used": False,
            "selected_measured_returns": int(len(selected)),
        },
        "projection_validation": {
            "camera_brightness_used_for_validation_only": True,
            "returns_on_camera_white_percent": float(on_camera_white.mean() * 100.0),
            "white_cells_reached": white_cells_reached,
            "predicted_white_cells_total": int(predictions.sum()),
        },
        "limitations": [
            "The MLP classifies checker reflectivity; it does not estimate camera intrinsics or the 6-DoF extrinsic transform.",
            "Cross-validation holds out checker cells, but all cells come from one stationary calibration recording.",
            "Independent checkerboard poses and road scenes are required to establish generalization.",
            "A neural calibration regressor such as CalibNet or LCCNet requires a substantially larger training dataset.",
        ],
    }

    image_path = args.output_dir / "nn_white_square_projection.png"
    report_path = args.output_dir / "nn_reflectivity_report.json"
    model_path = args.output_dir / "mlp_reflectivity_classifier.pkl"
    report["projection_validation"]["unique_display_pixels"] = save_projection(
        image, pixels, selected[:, 3], report,
        calibration["quality"]["checker_corner_reprojection_rmse_px"], image_path,
    )
    with model_path.open("wb") as handle:
        pickle.dump(
            {
                "model": model,
                "feature_names": FEATURE_NAMES,
                "cell_pixels": CELL_PIXELS,
                "patch_margin_pixels": PATCH_MARGIN_PIXELS,
            },
            handle,
        )
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    cv2.imwrite(str(args.output_dir / "canonical_lidar_reflectivity.png"), canonical)
    print(json.dumps(report, indent=2))
    print(f"Saved {image_path}")
    print(f"Saved {report_path}")
    print(f"Saved {model_path}")


if __name__ == "__main__":
    main()
