#!/usr/bin/env python3
"""Evaluate checkerboard masking on public and internal RGB–LiDAR data.

Every visualization remains in the source camera pixel coordinate system.  The
program never rectifies, resizes, deblurs, inpaints, or regenerates an RGB
frame.  Masks are separate label arrays; figures only draw contours and real
measured LiDAR returns over a copy of the source image.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize
from sklearn.decomposition import PCA
from sklearn.linear_model import LinearRegression, RANSACRegressor

from calibrate import CALIBRATION_POSES
from project_lidar_three_distances import (
    aggregate_board_points,
    keep_board_plane,
    lidar_board_polygon,
    project,
    representative_white_returns,
    sharpest_camera_frame,
)


PUBLIC_PATTERN = (7, 5)
PUBLIC_GRID_M = 0.08
PUBLIC_CELLS = (8, 6)
PUBLIC_SOURCE_URL = "https://github.com/HViktorTsoi/ACSC"
PUBLIC_DOWNLOAD_URL = (
    "https://drive.google.com/file/d/1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K/view"
)
INTERNAL_CELLS = (12, 18)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def pixel_sha256(image: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(image).tobytes()).hexdigest()


def detect_camera_corners(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray, PUBLIC_PATTERN)
    if not found:
        raise RuntimeError("Public checkerboard was not detected")
    corners = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    # Match ACSC's published lower-corner-first convention.
    if corners[0, 1] < corners[-1, 1] or (
        corners[0, 1] == corners[-1, 1] and corners[0, 0] > corners[-1, 0]
    ):
        corners = corners[::-1]
    return corners


def public_model_corners() -> np.ndarray:
    width_corners, height_corners = PUBLIC_PATTERN
    xx, yy = np.meshgrid(
        np.arange(1, height_corners + 1) * PUBLIC_GRID_M,
        np.arange(1, width_corners + 1) * PUBLIC_GRID_M,
    )
    xx, yy = xx.reshape(-1), yy.reshape(-1)
    order = yy / PUBLIC_GRID_M - 1 + width_corners * (xx / PUBLIC_GRID_M - 1)
    corners = np.c_[xx, yy, np.zeros_like(xx), order]
    corners = corners[np.argsort(order)]
    corners[:, :2] -= np.asarray(
        [(height_corners + 1) / 2, (width_corners + 1) / 2]
    ) * PUBLIC_GRID_M
    return corners


def fit_height_window(points: np.ndarray, height_m: float) -> np.ndarray:
    """Select the densest board-height interval and reject the supporting stand."""
    z = points[:, 2]
    step = 0.01
    edges = np.arange(float(z.min()) - step, float(z.max()) + 2 * step, step)
    hist, _ = np.histogram(z, bins=edges)
    width = max(1, int(round(height_m / step)))
    scores = np.convolve(hist, np.ones(width, dtype=np.int64), mode="valid")
    start = int(np.argmax(scores))
    low = edges[start] - 0.025
    high = edges[min(start + width, len(edges) - 1)] + 0.025
    return points[(z >= low) & (z <= high)]


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """Robustly fit x = a*y + b*z + c to the approximately vertical board."""
    features = points[:, 1:3]
    target = points[:, 0]
    if len(points) > 40000:
        indices = np.linspace(0, len(points) - 1, 40000, dtype=int)
    else:
        indices = np.arange(len(points))
    estimator = RANSACRegressor(
        estimator=LinearRegression(), min_samples=0.25,
        residual_threshold=0.018, max_trials=120, random_state=7,
    )
    estimator.fit(features[indices], target[indices])
    coefficient = np.r_[estimator.estimator_.coef_, estimator.estimator_.intercept_]
    residual = target - np.c_[features, np.ones(len(points))] @ coefficient
    median = float(np.median(residual))
    mad = 1.4826 * float(np.median(np.abs(residual - median)))
    limit = max(0.012, min(0.030, 3.0 * mad))
    keep = np.abs(residual - median) <= limit
    rmse = float(np.sqrt(np.mean(residual[keep] ** 2)))
    return points[keep], coefficient, rmse


def ray_project_to_plane(points: np.ndarray, coefficient: np.ndarray) -> np.ndarray:
    a, b, c = coefficient
    # x - a*y - b*z - c = 0, intersected with the ray from LiDAR origin.
    denominator = points[:, 0] - a * points[:, 1] - b * points[:, 2]
    scale = np.divide(c, denominator, out=np.ones_like(denominator), where=np.abs(denominator) > 1e-9)
    result = points.copy()
    result[:, :3] *= scale[:, None]
    return result


def transfer_by_pca(points_xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pca = PCA(n_components=3).fit(points_xyz)
    transform = pca.components_.copy()
    transform[[0, 1]] = transform[[1, 0]]
    transform[2] = np.cross(transform[0], transform[1])
    sign_vertical = np.sign(np.dot(transform[1], np.asarray([0.0, 0.0, 1.0]))) or 1.0
    transform[[0, 1]] *= sign_vertical
    sign_normal = np.sign(np.dot(transform[2], -points_xyz.mean(axis=0))) or 1.0
    transform[[0, 2]] *= sign_normal
    return transform, points_xyz @ transform.T


def otsu_intensity(values: np.ndarray) -> float:
    clipped = np.clip(values, 0, 255).astype(np.uint8)
    threshold, _ = cv2.threshold(clipped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return float(threshold)


def deterministic_voxel(points: np.ndarray, size: float = 0.004) -> np.ndarray:
    keys = np.floor(points[:, :2] / size).astype(np.int64)
    order = np.lexsort((points[:, 3], keys[:, 1], keys[:, 0]))
    sorted_keys = keys[order]
    first = np.r_[True, np.any(np.diff(sorted_keys, axis=0), axis=1)]
    return points[order[first]]


def checker_matching_loss(parameters: np.ndarray, points: np.ndarray, pivot: float) -> float:
    theta, offset_x, offset_y = parameters
    cosine, sine = np.cos(theta), np.sin(theta)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    xy = (points[:, :2] + np.asarray([offset_x, offset_y])) @ rotation.T
    half_size = np.asarray(PUBLIC_CELLS, dtype=np.float64) * PUBLIC_GRID_M / 2
    cell = np.floor((xy + half_size) / PUBLIC_GRID_M).astype(int)
    expected = (cell[:, 0] % 2) != (cell[:, 1] % 2)
    measured = points[:, 3] > pivot
    outside = np.any((xy < -half_size) | (xy > half_size), axis=1)
    mismatch = expected != measured
    grid_x = (np.arange(PUBLIC_CELLS[0] + 1) - PUBLIC_CELLS[0] / 2) * PUBLIC_GRID_M
    grid_y = (np.arange(PUBLIC_CELLS[1] + 1) - PUBLIC_CELLS[1] / 2) * PUBLIC_GRID_M
    distance = (
        np.min(np.abs(xy[:, 0, None] - grid_x), axis=1)
        + np.min(np.abs(xy[:, 1, None] - grid_y), axis=1)
    )
    return float(np.sum(distance * (outside | mismatch)))


def detect_lidar_board(points: np.ndarray, roi: np.ndarray) -> dict:
    low = roi[[0, 2, 4]]
    high = roi[[1, 3, 5]]
    roi_points = points[np.all((points[:, :3] >= low) & (points[:, :3] <= high), axis=1)]
    board_height = PUBLIC_CELLS[1] * PUBLIC_GRID_M
    height_points = fit_height_window(roi_points, board_height)
    plane_points, coefficient, plane_rmse = fit_plane(height_points)
    plane_points = ray_project_to_plane(plane_points, coefficient)
    rotation_3d, transformed = transfer_by_pca(plane_points[:, :3])
    translation_3d = transformed.mean(axis=0)
    local = plane_points.copy()
    local[:, :3] = transformed - translation_3d
    pivot = otsu_intensity(local[:, 3])
    fit_points = deterministic_voxel(local)
    result = minimize(
        checker_matching_loss, np.zeros(3), args=(fit_points, pivot),
        method="L-BFGS-B", bounds=[(-0.25, 0.25), (-0.12, 0.12), (-0.12, 0.12)],
        options={"maxiter": 1200, "ftol": 1e-12},
    )
    theta, offset_x, offset_y = result.x
    cosine, sine = np.cos(theta), np.sin(theta)
    rotation_2d = np.asarray(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
    )
    translation_2d = np.asarray([offset_x, offset_y, 0.0])

    corners = public_model_corners()
    corners[:, :3] = corners[:, :3] @ np.linalg.inv(rotation_2d.T) - translation_2d
    corners[:, :3] = (corners[:, :3] + translation_3d) @ np.linalg.inv(rotation_3d.T)
    if corners[0, 2] > corners[-1, 2] or (
        corners[0, 2] == corners[-1, 2] and corners[0, 1] < corners[-1, 1]
    ):
        corners = corners[::-1]

    aligned_xyz = (local[:, :3] + translation_2d) @ rotation_2d.T
    half_size = np.asarray(PUBLIC_CELLS, dtype=np.float64) * PUBLIC_GRID_M / 2
    inside = np.all(np.abs(aligned_xyz[:, :2]) <= half_size + 0.008, axis=1)
    board_points = plane_points[inside]
    board_aligned = aligned_xyz[inside]
    return {
        "corners": corners[:, :3],
        "points": board_points,
        "aligned": board_aligned,
        "roi_count": int(len(roi_points)),
        "height_count": int(len(height_points)),
        "plane_count": int(len(plane_points)),
        "board_count": int(len(board_points)),
        "plane_rmse_m": plane_rmse,
        "intensity_threshold": pivot,
        "matching_loss": float(result.fun / max(len(fit_points), 1)),
        "optimization_success": bool(result.success),
    }


def solve_public_extrinsic(frames: list[dict], train_indices: set[int]) -> tuple[np.ndarray, np.ndarray, int]:
    train = [item for item in frames if item["index"] in train_indices]
    world = np.concatenate([item["lidar"]["corners"] for item in train])
    image = np.concatenate([item["camera_corners"] for item in train])
    camera_k = frames[0]["camera_k"]
    distortion = frames[0]["distortion"]
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        world, image, camera_k, distortion, flags=cv2.SOLVEPNP_ITERATIVE,
        reprojectionError=6.0, iterationsCount=1000, confidence=0.999,
    )
    if not ok:
        raise RuntimeError("Could not solve public-dataset LiDAR-to-camera extrinsic")
    cv2.solvePnP(
        world[inliers[:, 0]], image[inliers[:, 0]], camera_k, distortion,
        rvec, tvec, True, flags=cv2.SOLVEPNP_ITERATIVE,
    )
    # A checkerboard has a left/right ordering ambiguity in 3D. Resolve each
    # pose against the RANSAC consensus transform, then refit the single rigid
    # sensor transform. This changes correspondence order, never measurements.
    for item in frames:
        base = item["lidar"]["corners"].reshape(5, 7, 3)
        variants = [
            base,
            base[::-1, :, :],
            base[:, ::-1, :],
            base[::-1, ::-1, :],
        ]
        scored = []
        for variant in variants:
            candidate = np.ascontiguousarray(variant.reshape(-1, 3))
            projected = cv2.projectPoints(
                candidate, rvec, tvec, camera_k, distortion,
            )[0].reshape(-1, 2)
            error = float(np.sqrt(np.mean((projected - item["camera_corners"]) ** 2)))
            scored.append((error, candidate))
        item["lidar"]["corners"] = min(scored, key=lambda pair: pair[0])[1]
    world = np.concatenate([item["lidar"]["corners"] for item in train])
    image = np.concatenate([item["camera_corners"] for item in train])
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        world, image, camera_k, distortion, rvec, tvec, True,
        iterationsCount=1000, reprojectionError=4.0, confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("Public robust refit failed after checkerboard ordering")
    cv2.solvePnP(
        world[inliers[:, 0]], image[inliers[:, 0]], camera_k, distortion,
        rvec, tvec, True, flags=cv2.SOLVEPNP_ITERATIVE,
    )
    return cv2.Rodrigues(rvec)[0], tvec.reshape(3), int(len(inliers))


def camera_outer_polygon(corners: np.ndarray) -> np.ndarray:
    width, height = PUBLIC_PATTERN
    canonical = np.stack(np.meshgrid(np.arange(width), np.arange(height)), axis=-1).reshape(-1, 2)
    # The ACSC ordering is reversed after lower-corner normalization.
    canonical = canonical[::-1].astype(np.float64)
    image_to_grid, _ = cv2.findHomography(corners, canonical)
    grid_to_image = np.linalg.inv(image_to_grid)
    outer = np.asarray([[[-1.0, -1.0], [width, -1.0], [width, height], [-1.0, height]]])
    return cv2.perspectiveTransform(outer.astype(np.float64), grid_to_image)[0]


def checker_mask(image: np.ndarray, polygon: np.ndarray, columns: int, rows: int) -> np.ndarray:
    destination = np.asarray(
        [[0.0, 0.0], [float(columns), 0.0], [float(columns), float(rows)], [0.0, float(rows)]],
        dtype=np.float32,
    )
    canonical_to_image = cv2.getPerspectiveTransform(destination, polygon.astype(np.float32))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cell_records = []
    for row in range(rows):
        for column in range(columns):
            cell = np.asarray([[[column, row], [column + 1, row], [column + 1, row + 1], [column, row + 1]]], dtype=np.float32)
            pixels = cv2.perspectiveTransform(cell, canonical_to_image)[0]
            cell_mask = np.zeros_like(mask)
            cv2.fillConvexPoly(cell_mask, np.rint(pixels).astype(np.int32), 1)
            values = gray[cell_mask.astype(bool)]
            cell_records.append((row, column, pixels, float(np.mean(values)) if len(values) else 0.0))
    brightness = np.asarray([record[3] for record in cell_records], dtype=np.uint8)
    threshold, _ = cv2.threshold(brightness, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    for _, _, pixels, mean_value in cell_records:
        cv2.fillConvexPoly(mask, np.rint(pixels).astype(np.int32), 1 if mean_value > threshold else 2)
    return mask


def project_points(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray,
                   camera_k: np.ndarray, distortion: np.ndarray, shape: tuple[int, ...]):
    camera = points[:, :3] @ rotation.T + translation
    pixels = cv2.projectPoints(
        np.ascontiguousarray(points[:, :3], dtype=np.float64),
        cv2.Rodrigues(rotation)[0], translation, camera_k, distortion
    )[0].reshape(-1, 2)
    keep = (
        (camera[:, 2] > 0) & (pixels[:, 0] >= 0) & (pixels[:, 0] < shape[1])
        & (pixels[:, 1] >= 0) & (pixels[:, 1] < shape[0])
    )
    return pixels[keep], camera[keep, 2], points[keep]


def point_mask_metrics(pixels: np.ndarray, points: np.ndarray, mask: np.ndarray,
                       intensity_threshold: float | None = None,
                       pixel_tolerance: int = 2) -> dict:
    rounded = np.rint(pixels).astype(int)
    labels = mask[rounded[:, 1], rounded[:, 0]]
    inside = labels > 0
    kernel_size = 2 * pixel_tolerance + 1
    tolerant_mask = cv2.dilate(
        (mask > 0).astype(np.uint8), np.ones((kernel_size, kernel_size), np.uint8)
    )
    tolerant_inside = tolerant_mask[rounded[:, 1], rounded[:, 0]] > 0
    result = {
        "projected_measured_returns": int(len(pixels)),
        "inside_camera_checkerboard": int(inside.sum()),
        "strict_board_containment_pct": float(100.0 * inside.mean()) if len(inside) else 0.0,
        "pixel_tolerance": pixel_tolerance,
        "within_tolerant_camera_checkerboard": int(tolerant_inside.sum()),
        "tolerant_board_containment_pct": (
            float(100.0 * tolerant_inside.mean()) if len(tolerant_inside) else 0.0
        ),
    }
    if intensity_threshold is not None and len(points):
        usable = inside
        lidar_white = points[:, 3] > intensity_threshold
        camera_white = labels == 1
        agreement = lidar_white[usable] == camera_white[usable]
        result.update({
            "reflectivity_cell_samples": int(usable.sum()),
            "reflectivity_vs_rgb_cell_accuracy_pct": float(100.0 * agreement.mean()) if len(agreement) else 0.0,
        })
    return result


def display_subset(pixels: np.ndarray, depth: np.ndarray, points: np.ndarray, maximum: int = 900):
    if len(pixels) <= maximum:
        return pixels, depth, points
    order = np.lexsort((-points[:, 3], np.rint(pixels[:, 1]), np.rint(pixels[:, 0])))
    chosen = order[np.linspace(0, len(order) - 1, maximum, dtype=int)]
    return pixels[chosen], depth[chosen], points[chosen]


def draw_result(image: np.ndarray, mask: np.ndarray, polygon: np.ndarray,
                pixels: np.ndarray, depth: np.ndarray, title: str, output: Path):
    figure, axes = plt.subplots(1, 3, figsize=(19, 6), constrained_layout=True)
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    axes[0].imshow(rgb)
    axes[0].set_title("Untouched source RGB")
    axes[1].imshow(mask, cmap="gray", vmin=0, vmax=2)
    axes[1].set_title("Separate checker-cell mask")
    axes[2].imshow(rgb)
    axes[2].plot(*np.vstack([polygon, polygon[0]]).T, color="lime", linewidth=2)
    scatter = axes[2].scatter(
        pixels[:, 0], pixels[:, 1], c=depth, cmap="turbo", s=8,
        linewidths=0.15, edgecolors="black",
    )
    axes[2].set_title("Original pixels + measured LiDAR projection")
    figure.colorbar(scatter, ax=axes[2], fraction=0.035, pad=0.015, label="camera depth (m)")
    for axis in axes:
        axis.axis("off")
    figure.suptitle(title, fontsize=15, fontweight="bold")
    figure.savefig(output, dpi=160, bbox_inches="tight")
    plt.close(figure)


def evaluate_public(root: Path, output_dir: Path) -> dict:
    camera_k = np.loadtxt(root / "intrinsic").astype(np.float64)
    distortion = np.loadtxt(root / "distortion").reshape(-1).astype(np.float64)
    frames = []
    for image_path in sorted((root / "images").glob("*.png")):
        index = int(image_path.stem)
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        points = np.load(root / "pcds" / f"{index:06d}.npy")
        roi = np.loadtxt(root / "ROIs" / f"{index:06d}.txt")
        frames.append({
            "index": index,
            "image_path": image_path,
            "image": image,
            "camera_corners": detect_camera_corners(image),
            "lidar": detect_lidar_board(points, roi),
            "camera_k": camera_k,
            "distortion": distortion,
        })
        print(f"Public frame {index:02d}: {frames[-1]['lidar']['board_count']} measured board returns")

    test_indices = {2, 5, 8, 11, 14, 17, 20}
    train_indices = {item["index"] for item in frames} - test_indices
    rotation, translation, pnp_inliers = solve_public_extrinsic(frames, train_indices)
    per_frame = []
    for item in frames:
        projected = cv2.projectPoints(
            np.ascontiguousarray(item["lidar"]["corners"], dtype=np.float64),
            cv2.Rodrigues(rotation)[0], translation,
            camera_k, distortion,
        )[0].reshape(-1, 2)
        errors = np.linalg.norm(projected - item["camera_corners"], axis=1)
        per_frame.append({
            "index": item["index"],
            "split": "test" if item["index"] in test_indices else "train",
            "corner_rmse_px": float(np.sqrt(np.mean(errors ** 2))),
            **{key: value for key, value in item["lidar"].items() if key not in {"corners", "points", "aligned"}},
        })

    representative_indices = [0, 10, 20]
    outputs = []
    representative_metrics = []
    for index in representative_indices:
        item = next(frame for frame in frames if frame["index"] == index)
        polygon = camera_outer_polygon(item["camera_corners"])
        mask = checker_mask(item["image"], polygon, *PUBLIC_CELLS)
        pixels, depth, measured = project_points(
            item["lidar"]["points"], rotation, translation, camera_k, distortion,
            item["image"].shape,
        )
        metrics = point_mask_metrics(
            pixels, measured, mask, item["lidar"]["intensity_threshold"]
        )
        metrics.update({
            "index": index,
            "source_image": str(item["image_path"]),
            "source_image_sha256": file_sha256(item["image_path"]),
            "source_pixel_sha256_bgr": pixel_sha256(item["image"]),
            "source_shape_hwc": list(item["image"].shape),
            "rgb_resized_or_warped": False,
            "median_camera_depth_m": float(np.median(depth)),
        })
        representative_metrics.append(metrics)
        high_reflectivity = measured[:, 3] > item["lidar"]["intensity_threshold"]
        display_pixels, display_depth, _ = display_subset(
            pixels[high_reflectivity], depth[high_reflectivity], measured[high_reflectivity]
        )
        metrics["displayed_high_reflectivity_measured_returns"] = int(len(display_pixels))
        output = output_dir / f"public_acsc_{index:06d}_real_mask_projection.png"
        mask_output = output_dir / f"public_acsc_{index:06d}_checker_mask.png"
        cv2.imwrite(str(mask_output), mask)
        metrics.update({
            "mask_output": str(mask_output),
            "mask_pixel_sha256": pixel_sha256(mask),
            "mask_label_counts": {
                str(label): int(np.sum(mask == label)) for label in (0, 1, 2)
            },
        })
        draw_result(
            item["image"], mask, polygon, display_pixels, display_depth,
            f"Public ACSC frame {index:06d} — raw-pixel masking and real LiDAR returns",
            output,
        )
        outputs.extend([str(output), str(mask_output)])

    train_rmse = [record["corner_rmse_px"] for record in per_frame if record["split"] == "train"]
    test_rmse = [record["corner_rmse_px"] for record in per_frame if record["split"] == "test"]
    return {
        "dataset": "ACSC public sample",
        "source_repository": PUBLIC_SOURCE_URL,
        "download": PUBLIC_DOWNLOAD_URL,
        "sensor_data": "real paired RGB images and N×4 XYZ-reflectance point clouds",
        "frames_total": len(frames),
        "train_indices": sorted(train_indices),
        "held_out_test_indices": sorted(test_indices),
        "extrinsic_direction": "p_camera = R * p_lidar + t",
        "R": rotation.tolist(),
        "t_m": translation.tolist(),
        "pnp_inlier_corners": pnp_inliers,
        "train_corner_rmse_px_mean": float(np.mean(train_rmse)),
        "train_corner_rmse_px_median": float(np.median(train_rmse)),
        "held_out_corner_rmse_px_mean": float(np.mean(test_rmse)),
        "held_out_corner_rmse_px_median": float(np.median(test_rmse)),
        "corner_fit_interpretation": (
            "3D grid-corner recovery is weak in several public poses; measured-return "
            "board containment is reported separately as visualization only"
        ),
        "per_frame": per_frame,
        "representative_mask_results": representative_metrics,
        "outputs": outputs,
    }


def evaluate_internal(data_root: Path, calibration_path: Path, output_dir: Path) -> dict:
    calibration = json.loads(calibration_path.read_text())
    camera_k = np.asarray(calibration["camera"]["K"], dtype=np.float64)
    distortion = np.asarray(calibration["camera"]["D"], dtype=np.float64)
    rotation = np.asarray(calibration["extrinsic"]["R"], dtype=np.float64)
    translation = np.asarray(calibration["extrinsic"]["t_m"], dtype=np.float64)
    records = []
    outputs = []
    for name, spec in sorted(CALIBRATION_POSES.items(), key=lambda pair: pair[1]["distance_ft"]):
        bag = data_root / name
        polygon = np.asarray(
            calibration["calibration_observations"][name]["camera_board_corners_px"],
            dtype=np.float64,
        )
        image_rgb, audit = sharpest_camera_frame(bag, polygon)
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        mask = checker_mask(image, polygon, *INTERNAL_CELLS)
        points, source_audit = aggregate_board_points(bag, spec)
        board, coefficient, plane_rmse, _ = keep_board_plane(points, spec)
        representatives, represented_cells = representative_white_returns(board, spec, 3, 0.15)
        pixels, depth, measured = project(
            representatives, rotation, translation, camera_k, distortion, image.shape,
        )
        metrics = point_mask_metrics(pixels, measured, mask)
        projected_polygon = lidar_board_polygon(
            coefficient, spec, rotation, translation, camera_k, distortion,
        )
        output = output_dir / f"internal_{str(spec['distance_ft']).replace('.', '_')}ft_real_mask_projection.png"
        mask_output = output_dir / f"internal_{str(spec['distance_ft']).replace('.', '_')}ft_checker_mask.png"
        cv2.imwrite(str(mask_output), mask)
        draw_result(
            image, mask, polygon, pixels, depth,
            f"Internal {spec['distance_ft']:g} ft — raw-pixel masking and real LiDAR returns",
            output,
        )
        records.append({
            "dataset": name,
            "distance_ft": spec["distance_ft"],
            **metrics,
            "raw_board_roi_returns": int(len(points)),
            "plane_filtered_returns": int(len(board)),
            "displayed_measured_returns": int(len(measured)),
            "represented_known_white_cells": int(len(represented_cells)),
            "camera_frame": audit,
            "lidar_source": source_audit,
            "camera_polygon_px": polygon.tolist(),
            "projected_lidar_polygon_px": projected_polygon.tolist(),
            "source_shape_hwc": list(image.shape),
            "source_pixel_sha256_bgr": pixel_sha256(image),
            "rgb_resized_or_warped": False,
            "median_camera_depth_m": float(np.median(depth)),
            "mask_output": str(mask_output),
            "mask_pixel_sha256": pixel_sha256(mask),
            "mask_label_counts": {
                str(label): int(np.sum(mask == label)) for label in (0, 1, 2)
            },
            "plane_rmse_m": plane_rmse,
            "output": str(output),
        })
        outputs.extend([str(output), str(mask_output)])
    return {
        "dataset": "internal D455 + Livox Mid-360 ROS bags",
        "calibration": str(calibration_path),
        "results": records,
        "outputs": outputs,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--public-root", type=Path, default=Path("data/public/acsc/sample"))
    parser.add_argument("--internal-root", type=Path, default=Path("data/raw"))
    parser.add_argument("--calibration", type=Path, default=Path("outputs/calibration_result.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/masking_evaluation"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    public = evaluate_public(args.public_root, args.output_dir)
    internal = evaluate_internal(args.internal_root, args.calibration, args.output_dir)
    report = {
        "purpose": "public-first then internal checkerboard masking and LiDAR projection evaluation",
        "authenticity_contract": {
            "source_rgb_pixels_preserved": True,
            "perspective_rectification_used": False,
            "rgb_resize_used": False,
            "deblurring_or_super_resolution_used": False,
            "synthetic_or_regenerated_images_used": False,
            "synthetic_lidar_points_used": False,
            "mask_representation": "separate uint8 label image; 0 background, 1 camera-bright cell, 2 camera-dark cell",
            "overlay_representation": "contours and projected measured returns drawn only for visualization",
        },
        "legacy_visualization_metric": {
            "metric": "measured board returns inside the RGB checkerboard mask dilated by the declared tolerance",
            "pixel_tolerance": 2,
            "used_for_phase1_acceptance": False,
            "reason": (
                "containment is not a point-wise reprojection error; use "
                "phase1_required_validation.py for the 99%-within-2px decision"
            ),
        },
        "public_evaluation": public,
        "internal_evaluation": internal,
    }
    report_path = args.output_dir / "checkerboard_mask_evaluation_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({
        "public_train_corner_rmse_px_mean": public["train_corner_rmse_px_mean"],
        "public_held_out_corner_rmse_px_mean": public["held_out_corner_rmse_px_mean"],
        "public_representative_mask_results": public["representative_mask_results"],
        "internal_results": internal["results"],
        "report": str(report_path),
    }, indent=2))


if __name__ == "__main__":
    main()
