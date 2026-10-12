#!/usr/bin/env python3
"""Create mentor-facing LiDAR-on-checkerboard projection evidence.

The geometric transform is loaded from ``calibration_reflectivity_result.json``;
this script does not refit it.  Measured returns are selected in LiDAR space by
the checkerboard geometry and reflectivity parity, then projected into the RGB
image.  Camera pixels are used only for the reported validation metrics, never
to choose which points are displayed.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

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
from calibrate_reflectivity_nn import (
    CELL_PIXELS,
    canonical_reflectivity,
    cell_dataset,
    detect_lidar_corners,
    point_cell_coordinates,
)


def select_lidar_white_returns(
    board: np.ndarray,
    raster_to_canonical: np.ndarray,
    white_parity: int,
    intensity_threshold: float,
    boundary_margin: float,
):
    """Select measured returns from reflective checker cells using LiDAR only."""
    rows, columns, fractions = point_cell_coordinates(board, raster_to_canonical)
    cell_rows, cell_columns = PATTERN[1] + 1, PATTERN[0] + 1
    keep = (
        (rows >= 0) & (rows < cell_rows)
        & (columns >= 0) & (columns < cell_columns)
        & ((rows + columns) % 2 == white_parity)
        & (fractions[:, 0] >= boundary_margin)
        & (fractions[:, 0] <= 1.0 - boundary_margin)
        & (fractions[:, 1] >= boundary_margin)
        & (fractions[:, 1] <= 1.0 - boundary_margin)
        & (board[:, 3] >= intensity_threshold)
    )
    return board[keep]


def project(points, rotation, translation, camera_k, distortion, image_shape):
    camera_points = (rotation @ points[:, :3].T).T + translation
    pixels = cv2.projectPoints(
        np.ascontiguousarray(points[:, :3]), cv2.Rodrigues(rotation)[0],
        translation, camera_k, distortion,
    )[0].reshape(-1, 2)
    valid = (
        (camera_points[:, 2] > 0)
        & (pixels[:, 0] >= 0) & (pixels[:, 0] < image_shape[1])
        & (pixels[:, 1] >= 0) & (pixels[:, 1] < image_shape[0])
    )
    return pixels[valid], camera_points[valid, 2], points[valid]


def strongest_unique_pixels(pixels, depth, intensity, image_width):
    """Keep one real return per rendered pixel without fabricating coverage."""
    rounded = np.rint(pixels).astype(int)
    order = np.argsort(intensity)[::-1]
    keys = rounded[order, 1] * image_width + rounded[order, 0]
    _, first = np.unique(keys, return_index=True)
    chosen = order[first]
    return pixels[chosen], depth[chosen], intensity[chosen]


def camera_board_polygon(camera_grid: np.ndarray):
    """Extrapolate the four physical outer board corners from inner corners."""
    rows, columns = PATTERN[1], PATTERN[0]
    grid_xy = np.stack(
        np.meshgrid(np.arange(columns), np.arange(rows)), axis=-1
    ).reshape(-1, 2).astype(np.float64)
    image_to_grid, _ = cv2.findHomography(camera_grid.reshape(-1, 2), grid_xy)
    if image_to_grid is None:
        raise RuntimeError("Could not estimate the camera checkerboard homography")
    grid_to_image = np.linalg.inv(image_to_grid)
    outer_grid = np.asarray(
        [[[-1.0, -1.0], [columns, -1.0], [columns, rows], [-1.0, rows]]],
        dtype=np.float64,
    )
    polygon = cv2.perspectiveTransform(outer_grid, grid_to_image)[0]
    return polygon, image_to_grid


def lidar_board_polygon(
    raster_to_canonical: np.ndarray,
    plane: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
    camera_k: np.ndarray,
    distortion: np.ndarray,
):
    """Project the LiDAR-derived physical outer board corners."""
    canonical_to_raster = np.linalg.inv(raster_to_canonical)
    rows, columns = PATTERN[1] + 1, PATTERN[0] + 1
    canonical_outer = np.asarray(
        [[[0.0, 0.0], [columns * CELL_PIXELS, 0.0],
          [columns * CELL_PIXELS, rows * CELL_PIXELS], [0.0, rows * CELL_PIXELS]]],
        dtype=np.float64,
    )
    raster_uv = cv2.perspectiveTransform(canonical_outer, canonical_to_raster)[0]
    y = BOARD_LIMITS["y"][1] - raster_uv[:, 0] * RASTER_RESOLUTION_M
    z = BOARD_LIMITS["z"][1] - raster_uv[:, 1] * RASTER_RESOLUTION_M
    x = plane[0] * y + plane[1] * z + plane[2]
    corners = np.c_[x, y, z]
    return cv2.projectPoints(
        corners, cv2.Rodrigues(rotation)[0], translation, camera_k, distortion
    )[0].reshape(-1, 2)


def projection_metrics(
    image: np.ndarray,
    camera_grid_to_checker: np.ndarray,
    pixels: np.ndarray,
    white_parity: int,
):
    rounded = np.rint(pixels).astype(int)
    grayscale = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    camera_white = grayscale[rounded[:, 1], rounded[:, 0]] > 128

    checker_xy = cv2.perspectiveTransform(
        pixels.astype(np.float64).reshape(-1, 1, 2), camera_grid_to_checker
    ).reshape(-1, 2)
    columns = np.floor(checker_xy[:, 0] + 1.0).astype(int)
    rows = np.floor(checker_xy[:, 1] + 1.0).astype(int)
    valid = (
        (rows >= 0) & (rows < PATTERN[1] + 1)
        & (columns >= 0) & (columns < PATTERN[0] + 1)
    )
    hit_cells = {(int(row), int(column)) for row, column in zip(rows[valid], columns[valid])}
    expected_white = {
        (row, column)
        for row in range(PATTERN[1] + 1)
        for column in range(PATTERN[0] + 1)
        if (row + column) % 2 == white_parity
    }
    unexpected_black_hits = {
        cell for cell in hit_cells if (cell[0] + cell[1]) % 2 != white_parity
    }
    return {
        "returns_on_camera_white_count": int(camera_white.sum()),
        "returns_on_camera_white_percent": float(camera_white.mean() * 100.0),
        "expected_white_cells_total": len(expected_white),
        "expected_white_cells_hit": len(hit_cells & expected_white),
        "white_cell_coverage_percent": float(len(hit_cells & expected_white) / len(expected_white) * 100.0),
        "camera_checker_black_cells_hit": len(unexpected_black_hits),
    }


def add_bounds(axis, camera_polygon, lidar_polygon):
    axis.add_patch(plt.Polygon(
        camera_polygon, closed=True, fill=False, color="lime", linewidth=3.2,
    ))
    axis.add_patch(plt.Polygon(
        lidar_polygon, closed=True, fill=False, color="yellow", linewidth=1.8,
        linestyle="--",
    ))
    camera_x = float(camera_polygon[:, 0].min())
    camera_y = float(camera_polygon[:, 1].min())
    lidar_x = float(lidar_polygon[:, 0].min())
    lidar_y = float(lidar_polygon[:, 1].min())
    label_style = {"facecolor": "black", "alpha": 0.58, "pad": 2}
    axis.text(
        camera_x, camera_y - 8, "camera checkerboard boundary", color="lime",
        fontsize=9.5, fontweight="bold", bbox=label_style,
    )
    axis.text(
        lidar_x, lidar_y + 12, "projected LiDAR board boundary", color="yellow",
        fontsize=9.5, fontweight="bold", bbox=label_style,
    )


def save_projection_figure(
    image, pixels, depth, camera_polygon, lidar_polygon, metrics, corner_rmse,
    output, closeup,
):
    figure, axis = plt.subplots(figsize=(14, 8), constrained_layout=True)
    axis.imshow(image)
    scatter = axis.scatter(
        pixels[:, 0], pixels[:, 1], c=depth, s=9 if closeup else 5,
        cmap="turbo", linewidths=0, alpha=0.9,
    )
    add_bounds(axis, camera_polygon, lidar_polygon)
    if closeup:
        minimum = camera_polygon.min(axis=0)
        maximum = camera_polygon.max(axis=0)
        padding = np.array([55.0, 45.0])
        axis.set_xlim(minimum[0] - padding[0], maximum[0] + padding[0])
        axis.set_ylim(maximum[1] + padding[1], minimum[1] - padding[1])
    title_prefix = "Calibrated LiDAR points falling on checkerboard"
    axis.set_title(
        f"{title_prefix}\n"
        f"white-square projection {metrics['returns_on_camera_white_percent']:.2f}%  |  "
        f"white cells {metrics['expected_white_cells_hit']}/{metrics['expected_white_cells_total']}  |  "
        f"corner RMSE {corner_rmse:.3f} px",
        fontsize=14, fontweight="bold",
    )
    axis.axis("off")
    colorbar = figure.colorbar(scatter, ax=axis, fraction=0.035, pad=0.02)
    colorbar.set_label("camera depth Z (m)")
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=Path("data/raw_new/new1"))
    parser.add_argument(
        "--calibration", type=Path,
        default=Path("outputs/calibration_reflectivity_result.json"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--intensity-threshold", type=float, default=20.0)
    parser.add_argument("--cell-boundary-margin", type=float, default=0.22)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    calibration = json.loads(args.calibration.read_text())
    image, camera_k, distortion, _ = camera_data(args.bag)
    saved_camera_k = np.asarray(calibration["camera"]["K"], dtype=np.float64)
    saved_distortion = np.asarray(calibration["camera"]["D"], dtype=np.float64)
    intrinsic_difference = float(np.max(np.abs(saved_camera_k - camera_k)))
    distortion_difference = float(np.max(np.abs(saved_distortion - distortion)))
    if intrinsic_difference > 1e-8 or distortion_difference > 1e-8:
        raise RuntimeError(
            "The selected calibration belongs to a different camera session: "
            f"max K difference={intrinsic_difference:g}, "
            f"max D difference={distortion_difference:g}"
        )
    camera_grid = detect_camera_corners(image)
    camera_polygon, camera_grid_to_checker = camera_board_polygon(camera_grid)

    cloud = aggregate_board_points(args.bag)
    plane, plane_keep, plane_rmse = robust_plane(cloud)
    board = cloud[plane_keep]
    raster = make_reflectivity_raster(board)
    lidar_corners = detect_lidar_corners(raster)
    canonical, raster_to_canonical = canonical_reflectivity(raster, lidar_corners)
    _, _, _, white_parity, parity_means = cell_dataset(canonical)
    selected = select_lidar_white_returns(
        board, raster_to_canonical, white_parity,
        args.intensity_threshold, args.cell_boundary_margin,
    )

    rotation = np.asarray(calibration["extrinsic"]["R"], dtype=np.float64)
    translation = np.asarray(calibration["extrinsic"]["t_m"], dtype=np.float64)
    pixels, depth, selected = project(
        selected, rotation, translation, camera_k, distortion, image.shape,
    )
    metrics = projection_metrics(image, camera_grid_to_checker, pixels, white_parity)
    shown_pixels, shown_depth, _ = strongest_unique_pixels(
        pixels, depth, selected[:, 3], image.shape[1],
    )
    lidar_polygon = lidar_board_polygon(
        raster_to_canonical, plane, rotation, translation, camera_k, distortion,
    )

    corner_rmse = float(calibration["quality"]["checker_corner_reprojection_rmse_px"])
    closeup_path = args.output_dir / "lidar_projection_checkerboard.png"
    full_path = args.output_dir / "lidar_projection_full_scene.png"
    save_projection_figure(
        image, shown_pixels, shown_depth, camera_polygon, lidar_polygon,
        metrics, corner_rmse, closeup_path, closeup=True,
    )
    save_projection_figure(
        image, shown_pixels, shown_depth, camera_polygon, lidar_polygon,
        metrics, corner_rmse, full_path, closeup=False,
    )

    report = {
        "purpose": "primary mentor-facing camera-LiDAR checkerboard projection",
        "source_bag": str(args.bag),
        "calibration": str(args.calibration),
        "projection": "p_camera = R * p_livox + t; distorted camera projection with recorded K and D",
        "selection": {
            "input_modality": "LiDAR geometry and reflectivity only",
            "method": "detected LiDAR checker grid, reflective parity, intensity, and cell-interior margin",
            "camera_white_mask_used": False,
            "intensity_threshold": args.intensity_threshold,
            "cell_boundary_margin_fraction": args.cell_boundary_margin,
            "measured_returns_selected": int(len(selected)),
            "unique_display_pixels": int(len(shown_pixels)),
            "mean_lidar_reflectivity_by_checker_parity": parity_means,
        },
        "calibration_quality": {
            "checker_corner_reprojection_rmse_px": corner_rmse,
            "lidar_plane_rmse_m": plane_rmse,
            "saved_vs_recorded_camera_K_max_abs_difference": intrinsic_difference,
            "saved_vs_recorded_distortion_max_abs_difference": distortion_difference,
        },
        "projection_validation": metrics,
        "outputs": [str(closeup_path), str(full_path)],
        "interpretation": (
            "The checker pattern is preserved after projecting independently selected measured "
            "LiDAR returns, which is strong visual evidence of correct calibration. It is not, by "
            "itself, proof of perfect calibration or independent long-range accuracy."
        ),
    }
    report_path = args.output_dir / "lidar_projection_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Saved {closeup_path}")
    print(f"Saved {full_path}")
    print(f"Saved {report_path}")


if __name__ == "__main__":
    main()
