#!/usr/bin/env python3
"""Generate readable checkerboard LiDAR projections at three distances.

The original 640x480 recordings contain the same 12x18-square physical board at
7.25, 14.5, and 22.2 ft.  Dense accumulated returns obscure the small distant
checkerboards, so this visualization chooses a fixed number of *measured* points
nearest the center of each known reflective-white cell.  No synthetic points or
camera-white per-pixel display mask are used.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from rosbags.highlevel import AnyReader

from calibrate import (
    CALIBRATION_POSES,
    POINT_DTYPE,
    fit_lidar_board,
    image_rgb,
    load_first,
)


SQUARE_SIZE_M = 0.025
CELL_COLUMNS = 12
CELL_ROWS = 18
WHITE_PARITY = 0


def aggregate_board_points(bag: Path, spec: dict):
    batches = []
    lidar_message_count = 0
    total_lidar_returns = 0
    first_bag_timestamp_ns = None
    last_bag_timestamp_ns = None
    lidar_frame = None
    with AnyReader([bag]) as reader:
        connection = next(c for c in reader.connections if c.topic == "/livox/lidar")
        for conn, bag_timestamp, raw in reader.messages(connections=[connection]):
            message = reader.deserialize(raw, conn.msgtype)
            points = np.frombuffer(message.data, dtype=POINT_DTYPE)
            lidar_message_count += 1
            total_lidar_returns += len(points)
            if first_bag_timestamp_ns is None:
                first_bag_timestamp_ns = int(bag_timestamp)
                lidar_frame = message.header.frame_id
            last_bag_timestamp_ns = int(bag_timestamp)
            keep = (
                (np.abs(points["x"] - spec["x"]) < 0.075)
                & (points["y"] >= spec["y"][0])
                & (points["y"] <= spec["y"][1])
                & (points["z"] >= spec["z"][0])
                & (points["z"] <= spec["z"][1])
            )
            selected = points[keep]
            batches.append(np.c_[
                selected["x"], selected["y"], selected["z"], selected["intensity"],
            ])
    if not batches or not any(len(batch) for batch in batches):
        raise RuntimeError(f"No checkerboard returns found in {bag}")
    return np.concatenate(batches).astype(np.float64), {
        "source_bag": str(bag),
        "lidar_topic": "/livox/lidar",
        "lidar_frame": lidar_frame,
        "lidar_messages_accumulated": lidar_message_count,
        "raw_lidar_returns_before_board_roi": total_lidar_returns,
        "first_bag_timestamp_ns": first_bag_timestamp_ns,
        "last_bag_timestamp_ns": last_bag_timestamp_ns,
        "accumulation_is_valid_because": "sensor rig and checkerboard are stationary throughout this recording",
    }


def sharpest_camera_frame(bag: Path, camera_polygon: np.ndarray):
    """Audit every RGB frame and return the sharpest board ROI from the real bag."""
    minimum = np.floor(camera_polygon.min(axis=0) - 3).astype(int)
    maximum = np.ceil(camera_polygon.max(axis=0) + 3).astype(int)
    best_score = -np.inf
    best_index = -1
    best_stamp = -1
    best_image = None
    frame_count = 0
    with AnyReader([bag]) as reader:
        connection = next(
            c for c in reader.connections if c.topic == "/camera/camera/color/image_raw"
        )
        for index, (conn, stamp, raw) in enumerate(reader.messages(connections=[connection])):
            message = reader.deserialize(raw, conn.msgtype)
            image = image_rgb(message)
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            x1, y1 = np.maximum(minimum, 0)
            x2 = min(int(maximum[0]), gray.shape[1])
            y2 = min(int(maximum[1]), gray.shape[0])
            roi = gray[y1:y2, x1:x2]
            score = float(cv2.Laplacian(roi, cv2.CV_64F).var())
            if score > best_score:
                best_score, best_index, best_stamp = score, index, int(stamp)
                best_image = image
            frame_count += 1
    if best_image is None:
        raise RuntimeError(f"No RGB frames found in {bag}")
    return best_image, {
        "selection_method": "maximum variance of Laplacian in fixed checkerboard ROI",
        "frames_audited": frame_count,
        "selected_frame_index": best_index,
        "selected_timestamp_ns": best_stamp,
        "sharpness_score": best_score,
        "super_resolution_or_deblurring_used": False,
    }


def keep_board_plane(points: np.ndarray, spec: dict):
    _, _, plane_rmse, inliers, coefficient = fit_lidar_board(
        points[:, :3], spec["y"], spec["z"],
    )
    design = np.c_[points[:, 1], points[:, 2], np.ones(len(points))]
    residual = points[:, 0] - design @ coefficient
    median = np.median(residual)
    mad = 1.4826 * np.median(np.abs(residual - median))
    keep = np.abs(residual - median) <= max(0.015, 3.0 * float(mad))
    return points[keep], coefficient, plane_rmse, inliers


def checker_coordinates(points: np.ndarray, spec: dict):
    # Board coordinates start at the camera-visible top-left and advance right/down.
    checker_x = (spec["y"][1] - points[:, 1]) / SQUARE_SIZE_M
    checker_y = (spec["z"][1] - points[:, 2]) / SQUARE_SIZE_M
    columns = np.floor(checker_x).astype(int)
    rows = np.floor(checker_y).astype(int)
    fractions = np.c_[checker_x - columns, checker_y - rows]
    return rows, columns, fractions


def representative_white_returns(
    points: np.ndarray,
    spec: dict,
    points_per_cell: int,
    boundary_margin: float,
):
    """Choose real returns nearest each white-cell center in LiDAR coordinates."""
    rows, columns, fractions = checker_coordinates(points, spec)
    valid = (
        (rows >= 0) & (rows < CELL_ROWS)
        & (columns >= 0) & (columns < CELL_COLUMNS)
        & ((rows + columns) % 2 == WHITE_PARITY)
        & (fractions[:, 0] >= boundary_margin)
        & (fractions[:, 0] <= 1.0 - boundary_margin)
        & (fractions[:, 1] >= boundary_margin)
        & (fractions[:, 1] <= 1.0 - boundary_margin)
    )
    valid_indices = np.flatnonzero(valid)
    chosen = []
    hit_cells = set()
    for row in range(CELL_ROWS):
        for column in range(CELL_COLUMNS):
            if (row + column) % 2 != WHITE_PARITY:
                continue
            matches = valid_indices[
                (rows[valid_indices] == row) & (columns[valid_indices] == column)
            ]
            if not len(matches):
                continue
            center_distance = np.sum((fractions[matches] - 0.5) ** 2, axis=1)
            count = min(points_per_cell, len(matches))
            chosen.extend(matches[np.argsort(center_distance)[:count]])
            hit_cells.add((row, column))
    return points[np.asarray(chosen, dtype=int)], hit_cells


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


def lidar_board_polygon(coefficient, spec, rotation, translation, camera_k, distortion):
    a, b, c = coefficient
    y_min, y_max = spec["y"]
    z_min, z_max = spec["z"]
    # TL, TR, BR, BL, matching the saved camera polygon order.
    yz = ((y_max, z_max), (y_min, z_max), (y_min, z_min), (y_max, z_min))
    corners = np.asarray([[a * y + b * z + c, y, z] for y, z in yz])
    return cv2.projectPoints(
        corners, cv2.Rodrigues(rotation)[0], translation, camera_k, distortion,
    )[0].reshape(-1, 2)


def polygon_accuracy(pixels, camera_polygon, tolerance):
    contour = np.asarray(camera_polygon, dtype=np.float32)
    signed_distance = np.asarray([
        cv2.pointPolygonTest(contour, (float(point[0]), float(point[1])), True)
        for point in pixels
    ])
    strict = float(np.mean(signed_distance >= 0.0) * 100.0)
    tolerant = float(np.mean(signed_distance >= -tolerance) * 100.0)
    return strict, tolerant


def cell_pixel_size(camera_polygon):
    top_width = np.linalg.norm(camera_polygon[1] - camera_polygon[0])
    bottom_width = np.linalg.norm(camera_polygon[2] - camera_polygon[3])
    left_height = np.linalg.norm(camera_polygon[3] - camera_polygon[0])
    right_height = np.linalg.norm(camera_polygon[2] - camera_polygon[1])
    horizontal = (top_width + bottom_width) / (2.0 * CELL_COLUMNS)
    vertical = (left_height + right_height) / (2.0 * CELL_ROWS)
    return float(horizontal), float(vertical)


def add_projection(axis, item, closeup, marker_size):
    axis.imshow(item["image"])
    scatter = axis.scatter(
        item["pixels"][:, 0], item["pixels"][:, 1], c=item["depth"],
        s=marker_size, cmap="turbo", vmin=item["depth_scale"][0],
        vmax=item["depth_scale"][1], linewidths=0.25, edgecolors="black", alpha=0.95,
    )
    axis.add_patch(plt.Polygon(
        item["camera_polygon"], closed=True, fill=False, color="lime", linewidth=2.5,
    ))
    axis.add_patch(plt.Polygon(
        item["lidar_polygon"], closed=True, fill=False, color="yellow",
        linewidth=1.7, linestyle="--",
    ))
    if closeup:
        minimum = item["camera_polygon"].min(axis=0)
        maximum = item["camera_polygon"].max(axis=0)
        width, height = maximum - minimum
        padding = np.array([max(8.0, 0.30 * width), max(8.0, 0.20 * height)])
        axis.set_xlim(minimum[0] - padding[0], maximum[0] + padding[0])
        axis.set_ylim(maximum[1] + padding[1], minimum[1] - padding[1])
    status = "PASS" if item["accuracy_pass"] else "FAIL"
    axis.set_title(
        f"{item['distance_ft']:g} ft\n"
        f"{item['represented_cells']}/108 white cells | "
        f"{item['tolerant_accuracy_pct']:.2f}% within ±{item['tolerance_px']:g}px ({status})",
        fontsize=12, fontweight="bold",
    )
    axis.axis("off")
    return scatter


def save_individual(item, output):
    figure, axis = plt.subplots(figsize=(10, 8), constrained_layout=True)
    scatter = add_projection(axis, item, closeup=True, marker_size=20)
    axis.set_title(
        "Measured LiDAR returns projected on checkerboard\n" + axis.get_title(),
        fontsize=14, fontweight="bold",
    )
    colorbar = figure.colorbar(scatter, ax=axis, fraction=0.035, pad=0.02)
    colorbar.set_label("camera depth Z (m)")
    figure.savefig(output, dpi=200, bbox_inches="tight")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=Path("data/raw"))
    parser.add_argument(
        "--calibration", type=Path, default=Path("outputs/calibration_result.json"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--points-per-cell", type=int, default=3)
    parser.add_argument("--cell-boundary-margin", type=float, default=0.15)
    parser.add_argument("--pixel-tolerance", type=float, default=2.0)
    parser.add_argument("--required-accuracy-pct", type=float, default=99.0)
    args = parser.parse_args()
    if args.points_per_cell < 1:
        raise SystemExit("--points-per-cell must be at least 1")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    calibration = json.loads(args.calibration.read_text())
    camera_k = np.asarray(calibration["camera"]["K"], dtype=np.float64)
    distortion = np.asarray(calibration["camera"]["D"], dtype=np.float64)
    rotation = np.asarray(calibration["extrinsic"]["R"], dtype=np.float64)
    translation = np.asarray(calibration["extrinsic"]["t_m"], dtype=np.float64)

    items = []
    ordered_poses = sorted(
        CALIBRATION_POSES.items(), key=lambda item: item[1]["distance_ft"],
    )
    for name, spec in ordered_poses:
        bag = args.data_root / name
        camera_polygon = np.asarray(
            calibration["calibration_observations"][name]["camera_board_corners_px"],
            dtype=np.float64,
        )
        image, image_audit = sharpest_camera_frame(bag, camera_polygon)
        points, lidar_audit = aggregate_board_points(bag, spec)
        board, coefficient, plane_rmse, plane_inliers = keep_board_plane(points, spec)
        representatives, represented_cells = representative_white_returns(
            board, spec, args.points_per_cell, args.cell_boundary_margin,
        )
        pixels, depth, representatives = project(
            representatives, rotation, translation, camera_k, distortion, image.shape,
        )
        projected_polygon = lidar_board_polygon(
            coefficient, spec, rotation, translation, camera_k, distortion,
        )
        strict_accuracy, tolerant_accuracy = polygon_accuracy(
            pixels, camera_polygon, args.pixel_tolerance,
        )
        horizontal_cell_px, vertical_cell_px = cell_pixel_size(camera_polygon)
        items.append({
            "dataset": name,
            "distance_ft": spec["distance_ft"],
            "image": image,
            "pixels": pixels,
            "depth": depth,
            "camera_polygon": camera_polygon,
            "lidar_polygon": projected_polygon,
            "selected_measured_returns": int(len(representatives)),
            "raw_board_roi_returns": int(len(points)),
            "plane_filtered_board_returns": int(len(board)),
            "represented_cells": len(represented_cells),
            "strict_inside_board_pct": strict_accuracy,
            "tolerant_accuracy_pct": tolerant_accuracy,
            "tolerance_px": args.pixel_tolerance,
            "accuracy_pass": bool(tolerant_accuracy + 1e-12 >= args.required_accuracy_pct),
            "horizontal_pixels_per_cell": horizontal_cell_px,
            "vertical_pixels_per_cell": vertical_cell_px,
            "lidar_plane_rmse_m": plane_rmse,
            "lidar_plane_inliers": plane_inliers,
            "camera_frame_audit": image_audit,
            "lidar_source_audit": lidar_audit,
        })

    for item in items:
        item["depth_scale"] = (
            float(item["depth"].min()), float(item["depth"].max()),
        )

    figure, axes = plt.subplots(1, 3, figsize=(18, 6.4), constrained_layout=True)
    for axis, item in zip(axes, items):
        scatter = add_projection(axis, item, closeup=True, marker_size=18)
        colorbar = figure.colorbar(scatter, ax=axis, fraction=0.040, pad=0.015)
        colorbar.set_label("camera depth Z (m)")
    legend = [
        Line2D([0], [0], color="lime", lw=3, label="camera checkerboard boundary"),
        Line2D([0], [0], color="yellow", lw=2, ls="--", label="projected LiDAR board boundary"),
    ]
    figure.legend(handles=legend, loc="lower center", ncol=2, frameon=True)
    figure.suptitle(
        "Camera–LiDAR Checkerboard Projection at Three Distances\n"
        "real measured returns sampled from known white-cell interiors",
        fontsize=17, fontweight="bold",
    )
    closeups_path = args.output_dir / "three_distance_lidar_projection.png"
    figure.savefig(closeups_path, dpi=200, bbox_inches="tight")
    plt.close(figure)

    figure, axes = plt.subplots(1, 3, figsize=(18, 5.5), constrained_layout=True)
    for axis, item in zip(axes, items):
        scatter = add_projection(axis, item, closeup=False, marker_size=9)
        colorbar = figure.colorbar(scatter, ax=axis, fraction=0.040, pad=0.015)
        colorbar.set_label("camera depth Z (m)")
    figure.suptitle(
        "Full-Scene LiDAR Projection Evidence — 7.25 ft, 14.5 ft, and 22.2 ft",
        fontsize=16, fontweight="bold",
    )
    full_path = args.output_dir / "three_distance_lidar_projection_full_scene.png"
    figure.savefig(full_path, dpi=180, bbox_inches="tight")
    plt.close(figure)

    individual_paths = []
    for item in items:
        distance_label = str(item["distance_ft"]).replace(".", "_")
        path = args.output_dir / f"lidar_projection_{distance_label}ft.png"
        save_individual(item, path)
        individual_paths.append(path)

    report_items = []
    for item in items:
        report_items.append({key: value for key, value in item.items() if key not in {
            "image", "pixels", "depth", "camera_polygon", "lidar_polygon", "depth_scale",
        }})
    report = {
        "purpose": "readable measured LiDAR checkerboard projection at all three recorded distances",
        "calibration": str(args.calibration),
        "checkerboard": {
            "physical_cells": [CELL_COLUMNS, CELL_ROWS],
            "square_size_m": SQUARE_SIZE_M,
            "known_white_parity": WHITE_PARITY,
        },
        "display_selection": {
            "points_per_available_white_cell": args.points_per_cell,
            "cell_boundary_margin_fraction": args.cell_boundary_margin,
            "camera_white_pixel_mask_used": False,
            "synthetic_points_used": False,
            "reason": "fixed measured-point sampling prevents distant boards from becoming opaque rectangles",
        },
        "model_usage": {
            "deep_learning_model_used": False,
            "machine_learning_model_used": False,
            "super_resolution_model_used": False,
            "method": (
                "classical calibrated pinhole projection with recorded K/D/R/t, robust least-squares "
                "plane fitting, known 12x18 checker geometry, and deterministic measured-point sampling"
            ),
        },
        "accuracy_requirement": {
            "metric": "selected returns inside camera board polygon with stated pixel tolerance",
            "required_accuracy_pct": args.required_accuracy_pct,
            "pixel_tolerance": args.pixel_tolerance,
            "all_distances_pass": bool(all(item["accuracy_pass"] for item in items)),
        },
        "datasets": report_items,
        "outputs": [str(closeups_path), str(full_path), *map(str, individual_paths)],
        "limitations": [
            "The 14.5 ft and 22.2 ft camera cells are below three pixels per square, so square-level color agreement is not independently measurable from these images.",
            "Known checker geometry selects alternating cells; camera pixels are not used as a per-point display mask.",
            "Board containment plus boundary overlap demonstrates projection alignment, not mathematically perfect calibration.",
        ],
    }
    report_path = args.output_dir / "three_distance_lidar_projection_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Saved {closeups_path}")
    print(f"Saved {full_path}")
    for path in individual_paths:
        print(f"Saved {path}")
    print(f"Saved {report_path}")


if __name__ == "__main__":
    main()
