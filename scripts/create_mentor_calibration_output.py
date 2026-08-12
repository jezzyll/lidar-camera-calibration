#!/usr/bin/env python3
"""Create the mentor-facing four-stage calibration summary figure."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from calibrate_reflectivity import (
    aggregate_board_points,
    camera_data,
    detect_camera_corners,
    lidar_corners_3d,
    make_reflectivity_raster,
    robust_plane,
    unique_bright_points,
)


def draw_camera_detection(axis, image, grid):
    axis.imshow(image)
    rows, cols = grid.shape[:2]
    for row in range(rows):
        axis.plot(grid[row, :, 0], grid[row, :, 1], color="lime", linewidth=0.55, alpha=0.8)
    for col in range(cols):
        axis.plot(grid[:, col, 0], grid[:, col, 1], color="magenta", linewidth=0.55, alpha=0.8)
    axis.scatter(grid[:, :, 0], grid[:, :, 1], s=3, color="cyan", linewidths=0)
    minimum = grid.reshape(-1, 2).min(axis=0)
    maximum = grid.reshape(-1, 2).max(axis=0)
    axis.add_patch(plt.Rectangle(minimum, *(maximum - minimum), fill=False, color="lime", linewidth=1.8))
    axis.set_title("2. Camera checkerboard detection")
    axis.axis("off")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=Path("data/raw_new/new1"))
    parser.add_argument(
        "--calibration", type=Path, default=Path("outputs/calibration_reflectivity_result.json")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("outputs/mentor_calibration_pipeline.png")
    )
    args = parser.parse_args()
    calibration = json.loads(args.calibration.read_text())
    quality = calibration["quality"]
    threshold = float(quality["intensity_threshold"])

    image, camera_k, distortion, _ = camera_data(args.bag)
    camera_grid = detect_camera_corners(image)
    cloud = aggregate_board_points(args.bag)
    plane, plane_keep, _ = robust_plane(cloud)
    board = cloud[plane_keep]
    raster = make_reflectivity_raster(board)
    # Re-run corner detection on the LiDAR-only raster to prove the target is
    # independently observable by the second sensor.
    lidar_corners_3d(raster, plane)

    rotation = np.asarray(calibration["extrinsic"]["R"], dtype=np.float64)
    translation = np.asarray(calibration["extrinsic"]["t_m"], dtype=np.float64)
    camera_points = (rotation @ board[:, :3].T).T + translation
    pixels = cv2.projectPoints(
        np.ascontiguousarray(board[:, :3]), cv2.Rodrigues(rotation)[0], translation,
        camera_k, distortion,
    )[0].reshape(-1, 2)
    valid = (
        (camera_points[:, 2] > 0)
        & (pixels[:, 0] >= 0) & (pixels[:, 0] < image.shape[1])
        & (pixels[:, 1] >= 0) & (pixels[:, 1] < image.shape[0])
    )
    pixels, board = pixels[valid], board[valid]
    projected, projected_intensity = unique_bright_points(
        pixels, board[:, 3], threshold, image.shape[1]
    )

    figure = plt.figure(figsize=(18, 8.8), constrained_layout=True)
    grid = figure.add_gridspec(2, 4, height_ratios=(5.5, 1.35))
    axes = [figure.add_subplot(grid[0, index]) for index in range(4)]
    status_axis = figure.add_subplot(grid[1, :])

    axes[0].imshow(image)
    axes[0].set_title("1. Synchronized raw RGB")
    axes[0].axis("off")
    draw_camera_detection(axes[1], image, camera_grid)

    axes[2].imshow(raster, cmap="gray")
    axes[2].set_title("3. LiDAR reflectivity board extraction")
    axes[2].axis("off")

    axes[3].imshow(image)
    scatter = axes[3].scatter(
        projected[:, 0], projected[:, 1], c=projected_intensity, s=3.5,
        cmap="turbo", vmin=threshold, vmax=max(25, float(projected_intensity.max())),
        linewidths=0, alpha=0.9,
    )
    axes[3].set_title("4. Calibrated LiDAR projection")
    axes[3].axis("off")
    figure.colorbar(scatter, ax=axes[3], fraction=0.04, pad=0.01, label="LiDAR intensity")

    for axis in (axes[1], axes[3]):
        axis.set_xlim(540, 840)
        axis.set_ylim(500, 125)

    status_axis.axis("off")
    metrics = (
        f"Extrinsic corner RMSE: {quality['checker_corner_reprojection_rmse_px']:.3f} px     |     "
        f"LiDAR plane RMSE: {quality['lidar_plane_rmse_m'] * 1000:.2f} mm     |     "
        f"White-return precision: {quality['high_reflectivity_returns_on_camera_white_percent']:.2f}% "
        f"({quality['high_reflectivity_raw_returns']:,} returns)\n"
        f"White-cell coverage: {quality['white_checker_cells_hit']}/{quality['white_checker_cells_total']} "
        f"({quality['white_checker_cell_coverage_percent']:.1f}%)     |     "
        f"Black cells hit: {quality['black_checker_cells_hit']}     |     "
        "Result: checkerboard calibration validated"
    )
    status_axis.text(
        0.5, 0.56, metrics, ha="center", va="center", fontsize=13,
        bbox={"facecolor": "#eef6ee", "edgecolor": "#408040", "boxstyle": "round,pad=0.8"},
        linespacing=1.7,
    )
    status_axis.text(
        0.5, 0.02,
        "Calibration validation metrics — not vehicle-detection mAP or overall system accuracy",
        ha="center", va="bottom", fontsize=10.5, color="#8b1a1a", fontweight="bold",
    )
    figure.suptitle(
        "Livox Mid-360 + RealSense D455 — Checkerboard Calibration Pipeline",
        fontsize=19, fontweight="bold",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, bbox_inches="tight")
    plt.close(figure)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
