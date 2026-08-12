#!/usr/bin/env python3
"""High-precision checkerboard calibration using camera and LiDAR reflectivity.

This pipeline targets the 1280x720 ``new1`` ROS 2 bag.  It independently detects
the checker grid in:

* the RGB image; and
* a metric LiDAR reflectivity raster made by accumulating stationary scans.

Metric LiDAR checker corners are paired with RGB checker pixels and passed to
solvePnP, yielding the direct ``livox_frame -> camera_color_optical_frame``
transform.  High-reflectivity returns are then shown separately so black squares
remain visible, as requested by the mentor.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader

from calibrate import POINT_DTYPE, image_rgb, load_first


PATTERN = (12, 19)  # inner corners: columns, rows
CAMERA_ROI = (580, 170, 810, 460)
BOARD_LIMITS = {"y": (-0.226, 0.099), "z": (0.042, 0.542)}
RASTER_RESOLUTION_M = 0.00075


def camera_data(bag: Path):
    with AnyReader([bag]) as reader:
        _, info = load_first(reader, "/camera/camera/color/camera_info")
        _, image = load_first(reader, "/camera/camera/color/image_raw")
    K = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    D = np.asarray(info.d, dtype=np.float64)
    return image_rgb(image), K, D, info.header.frame_id


def detect_camera_corners(rgb: np.ndarray):
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    x1, y1, x2, y2 = CAMERA_ROI
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCornersSB(gray[y1:y2, x1:x2], PATTERN, flags)
    if not ok:
        raise RuntimeError("12x19 RGB checkerboard was not detected")
    return corners.reshape(PATTERN[1], PATTERN[0], 2) + np.array([x1, y1])


def aggregate_board_points(bag: Path):
    batches = []
    y0, y1 = BOARD_LIMITS["y"]
    z0, z1 = BOARD_LIMITS["z"]
    with AnyReader([bag]) as reader:
        conn = next(c for c in reader.connections if c.topic == "/livox/lidar")
        for connection, _, raw in reader.messages(connections=[conn]):
            msg = reader.deserialize(raw, connection.msgtype)
            p = np.frombuffer(msg.data, dtype=POINT_DTYPE)
            keep = (
                (p["x"] > 0.85) & (p["x"] < 1.15)
                & (p["y"] > y0) & (p["y"] < y1)
                & (p["z"] > z0) & (p["z"] < z1)
            )
            q = p[keep]
            batches.append(np.c_[q["x"], q["y"], q["z"], q["intensity"]])
    return np.concatenate(batches).astype(np.float64)


def robust_plane(points: np.ndarray):
    """Fit x = a*y + b*z + c and reject non-board depth returns."""
    A = np.c_[points[:, 1], points[:, 2], np.ones(len(points))]
    coef = np.linalg.lstsq(A, points[:, 0], rcond=None)[0]
    keep = np.ones(len(points), dtype=bool)
    for _ in range(5):
        residual = points[:, 0] - A @ coef
        median = np.median(residual)
        mad = np.median(np.abs(residual - median)) * 1.4826
        keep = np.abs(residual - median) < max(0.025, 2.5 * float(mad))
        coef = np.linalg.lstsq(A[keep], points[keep, 0], rcond=None)[0]
    residual = points[keep, 0] - A[keep] @ coef
    return coef, keep, float(np.sqrt(np.mean(residual ** 2)))


def make_reflectivity_raster(points: np.ndarray):
    y0, y1 = BOARD_LIMITS["y"]
    z0, z1 = BOARD_LIMITS["z"]
    resolution = RASTER_RESOLUTION_M
    width = round((y1 - y0) / resolution)
    height = round((z1 - z0) / resolution)
    # Raster right is decreasing LiDAR y; raster down is decreasing LiDAR z.
    u = np.floor((y1 - points[:, 1]) / resolution).astype(int)
    v = np.floor((z1 - points[:, 2]) / resolution).astype(int)
    valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, intensity = u[valid], v[valid], points[valid, 3]
    sums = np.zeros((height, width), np.float32)
    counts = np.zeros((height, width), np.float32)
    np.add.at(sums, (v, u), intensity)
    np.add.at(counts, (v, u), 1)
    # Smooth numerator and denominator separately; this fills Mid-360 scan gaps
    # without treating missing samples as zero reflectivity.
    sums = cv2.GaussianBlur(sums, (0, 0), 1.0)
    counts = cv2.GaussianBlur(counts, (0, 0), 1.0)
    average = np.divide(sums, counts, out=np.zeros_like(sums), where=counts > 0.01)
    occupied = counts > 0.1
    lo, hi = np.percentile(average[occupied], [1, 99])
    raster = np.clip((average - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
    raster[~occupied] = 0
    return raster


def lidar_corners_3d(raster: np.ndarray, plane: np.ndarray):
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCornersSB(raster, PATTERN, flags)
    if not ok:
        raise RuntimeError("12x19 LiDAR reflectivity checkerboard was not detected")
    grid = corners.reshape(PATTERN[1], PATTERN[0], 2)
    u, v = grid[:, :, 0], grid[:, :, 1]
    y = BOARD_LIMITS["y"][1] - u * RASTER_RESOLUTION_M
    z = BOARD_LIMITS["z"][1] - v * RASTER_RESOLUTION_M
    x = plane[0] * y + plane[1] * z + plane[2]
    return np.stack([x, y, z], axis=-1)


def solve_extrinsic(lidar_grid: np.ndarray, camera_grid: np.ndarray, K: np.ndarray, D: np.ndarray):
    """Resolve the four planar grid orientations by minimum corner RMSE."""
    candidates = []
    for flip_rows in (False, True):
        for flip_cols in (False, True):
            source = lidar_grid
            if flip_rows:
                source = source[::-1]
            if flip_cols:
                source = source[:, ::-1]
            source = np.ascontiguousarray(source.reshape(-1, 3))
            ok, rvec, tvec = cv2.solvePnP(
                source, camera_grid.reshape(-1, 2), K, D, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not ok:
                continue
            projected = cv2.projectPoints(source, rvec, tvec, K, D)[0].reshape(-1, 2)
            rmse = float(np.sqrt(np.mean(np.sum((projected - camera_grid.reshape(-1, 2)) ** 2, axis=1))))
            candidates.append((rmse, rvec, tvec, flip_rows, flip_cols))
    if not candidates:
        raise RuntimeError("Extrinsic solve failed")
    return min(candidates, key=lambda item: item[0])


def unique_bright_points(uv: np.ndarray, intensity: np.ndarray, threshold: float, width: int):
    keep = intensity >= threshold
    uv, intensity = uv[keep], intensity[keep]
    pixels = np.rint(uv).astype(int)
    # Retain only the strongest return at each rendered pixel.
    order = np.argsort(intensity)[::-1]
    _, first = np.unique(pixels[order, 1] * width + pixels[order, 0], return_index=True)
    chosen = order[first]
    return uv[chosen], intensity[chosen]


def checker_cell_metrics(rgb: np.ndarray, camera_grid: np.ndarray, uv: np.ndarray):
    """Count physical checker cells touched by retained projected returns."""
    rows, cols = PATTERN[1], PATTERN[0]
    grid_xy = np.stack(np.meshgrid(np.arange(cols), np.arange(rows)), axis=-1).reshape(-1, 2).astype(np.float64)
    homography, _ = cv2.findHomography(camera_grid.reshape(-1, 2), grid_xy)
    inverse = np.linalg.inv(homography)
    grayscale = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    white_cells = set()
    black_cells = set()
    # Inner corners [0..11] x [0..18] bound 13 x 20 physical squares.
    for row in range(-1, rows):
        for col in range(-1, cols):
            center = np.array([[[col + 0.5, row + 0.5]]], dtype=np.float64)
            pixel = cv2.perspectiveTransform(center, inverse)[0, 0]
            brightness = grayscale[round(pixel[1]), round(pixel[0])]
            (white_cells if brightness > 128 else black_cells).add((row, col))
    checker_xy = cv2.perspectiveTransform(uv.reshape(-1, 1, 2), homography).reshape(-1, 2)
    cells = np.floor(checker_xy).astype(int)
    valid = (
        (cells[:, 0] >= -1) & (cells[:, 0] < cols)
        & (cells[:, 1] >= -1) & (cells[:, 1] < rows)
    )
    hit_cells = {(row, col) for col, row in cells[valid]}
    return {
        "white_checker_cells_total": len(white_cells),
        "white_checker_cells_hit": len(hit_cells & white_cells),
        "white_checker_cell_coverage_percent": len(hit_cells & white_cells) / len(white_cells) * 100.0,
        "black_checker_cells_hit": len(hit_cells & black_cells),
    }


def save_figures(rgb, uv, intensity, threshold, white_precision, reprojection_rmse, output_dir):
    shown_uv, shown_intensity = unique_bright_points(uv, intensity, threshold, rgb.shape[1])
    title = (
        f"High-reflectivity LiDAR returns on checkerboard (intensity ≥ {threshold:g})\n"
        f"white-square precision {white_precision * 100:.2f}%  |  corner RMSE {reprojection_rmse:.3f} px"
    )

    fig, ax = plt.subplots(figsize=(13, 8), constrained_layout=True)
    ax.imshow(rgb)
    scatter = ax.scatter(
        shown_uv[:, 0], shown_uv[:, 1], c=shown_intensity, s=8, cmap="turbo",
        vmin=threshold, vmax=max(25, float(shown_intensity.max())), linewidths=0, alpha=0.9,
    )
    ax.set_xlim(540, 840)
    ax.set_ylim(480, 130)
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.axis("off")
    colorbar = fig.colorbar(scatter, ax=ax, fraction=0.035, pad=0.02)
    colorbar.set_label("LiDAR reflectivity / intensity")
    closeup = output_dir / "reflectivity_white_square_projection.png"
    fig.savefig(closeup, dpi=180, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 8), constrained_layout=True)
    ax.imshow(rgb)
    ax.scatter(
        shown_uv[:, 0], shown_uv[:, 1], c=shown_intensity, s=5, cmap="turbo",
        vmin=threshold, vmax=max(25, float(shown_intensity.max())), linewidths=0, alpha=0.9,
    )
    ax.set_title("New dataset — calibrated reflective-white LiDAR returns", fontsize=15, fontweight="bold")
    ax.axis("off")
    full = output_dir / "reflectivity_full_scene.png"
    fig.savefig(full, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return closeup, full, len(shown_uv)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=Path("data/raw_new/new1"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--intensity-threshold", type=float, default=23.0)
    args = parser.parse_args()
    if not args.bag.exists():
        raise SystemExit(f"Missing extracted new bag: {args.bag}")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rgb, K, D, camera_frame = camera_data(args.bag)
    camera_grid = detect_camera_corners(rgb)
    points = aggregate_board_points(args.bag)
    plane, plane_keep, plane_rmse = robust_plane(points)
    points = points[plane_keep]
    raster = make_reflectivity_raster(points)
    raster_path = args.output_dir / "lidar_reflectivity_checkerboard.png"
    cv2.imwrite(str(raster_path), raster)
    lidar_grid = lidar_corners_3d(raster, plane)

    corner_rmse, rvec, tvec, flip_rows, flip_cols = solve_extrinsic(lidar_grid, camera_grid, K, D)
    R = cv2.Rodrigues(rvec)[0]
    t = tvec[:, 0]
    camera_points = (R @ points[:, :3].T).T + t
    uv = cv2.projectPoints(np.ascontiguousarray(points[:, :3]), rvec, t, K, D)[0].reshape(-1, 2)
    valid = (
        (camera_points[:, 2] > 0)
        & (uv[:, 0] >= 0) & (uv[:, 0] < rgb.shape[1])
        & (uv[:, 1] >= 0) & (uv[:, 1] < rgb.shape[0])
    )
    uv, points = uv[valid], points[valid]
    grayscale = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    pixel_xy = np.rint(uv).astype(int)
    image_brightness = grayscale[pixel_xy[:, 1], pixel_xy[:, 0]]
    bright = points[:, 3] >= args.intensity_threshold
    white = image_brightness > 128
    white_precision = float(np.mean(white[bright]))
    correlation = float(np.corrcoef(points[:, 3], image_brightness)[0, 1])
    cell_metrics = checker_cell_metrics(rgb, camera_grid, uv[bright])

    closeup, full, displayed = save_figures(
        rgb, uv, points[:, 3], args.intensity_threshold, white_precision, corner_rmse, args.output_dir
    )
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    report = {
        "direction": "p_camera_color_optical = R * p_livox + t",
        "source_bag": str(args.bag),
        "camera": {
            "frame": camera_frame,
            "resolution": [rgb.shape[1], rgb.shape[0]],
            "K": K.tolist(),
            "D": D.tolist(),
        },
        "checkerboard": {"inner_corners": list(PATTERN)},
        "extrinsic": {"R": R.tolist(), "t_m": t.tolist(), "T_camera_lidar": T.tolist()},
        "quality": {
            "checker_corner_reprojection_rmse_px": corner_rmse,
            "lidar_plane_rmse_m": plane_rmse,
            "reflectivity_vs_image_brightness_pearson_r": correlation,
            "intensity_threshold": args.intensity_threshold,
            "high_reflectivity_raw_returns": int(bright.sum()),
            "high_reflectivity_unique_display_pixels": int(displayed),
            "high_reflectivity_returns_on_camera_white_fraction": white_precision,
            "high_reflectivity_returns_on_camera_white_percent": white_precision * 100.0,
            **cell_metrics,
        },
        "orientation_resolution": {"flip_rows": flip_rows, "flip_cols": flip_cols},
        "metric_note": (
            "White-square percentage is precision/purity after the stated reflectivity threshold; "
            "it is not total calibration accuracy and is measured on the calibration recording."
        ),
    }
    result_path = args.output_dir / "calibration_reflectivity_result.json"
    result_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Saved {closeup}")
    print(f"Saved {full}")
    print(f"Saved {raster_path}")
    print(f"Saved {result_path}")


if __name__ == "__main__":
    main()
