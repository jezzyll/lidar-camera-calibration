#!/usr/bin/env python3
"""Project checkerboard LiDAR returns and measure board-alignment accuracy."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader

from calibrate import (
    CALIBRATION_POSES,
    POINT_DTYPE,
    TARGET_DATASET,
    TARGET_RANGE_FT,
    image_rgb,
    load_first,
)


RANGES = CALIBRATION_POSES


def aggregate_board(bag: Path, spec: dict):
    batches = []
    with AnyReader([bag]) as reader:
        conn = next(c for c in reader.connections if c.topic == "/livox/lidar")
        for index, (connection, _, raw) in enumerate(reader.messages(connections=[conn])):
            if index % 10:
                continue
            msg = reader.deserialize(raw, connection.msgtype)
            p = np.frombuffer(msg.data, dtype=POINT_DTYPE)
            keep = (
                (np.abs(p["x"] - spec["x"]) < 0.05)
                & (p["y"] >= spec["y"][0])
                & (p["y"] <= spec["y"][1])
                & (p["z"] >= spec["z"][0])
                & (p["z"] <= spec["z"][1])
            )
            p = p[keep]
            batches.append(np.c_[p["x"], p["y"], p["z"], p["intensity"]])
    return np.concatenate(batches).astype(np.float64)


def polygon_distances(points: np.ndarray, polygon: np.ndarray):
    contour = np.asarray(polygon, dtype=np.float32)
    return np.asarray([
        cv2.pointPolygonTest(contour, (float(point[0]), float(point[1])), True)
        for point in points
    ])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=Path("data/raw"))
    parser.add_argument("--calibration", type=Path, default=Path("outputs/calibration_result.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--required-accuracy-pct", type=float, default=99.0)
    parser.add_argument(
        "--pixel-tolerance", type=float, default=2.0,
        help="Allowed distance outside the annotated board boundary for the accuracy metric.",
    )
    args = parser.parse_args()
    result = json.loads(args.calibration.read_text())
    K = np.asarray(result["camera"]["K"])
    D = np.asarray(result["camera"]["D"])
    R = np.asarray(result["extrinsic"]["R"])
    t = np.asarray(result["extrinsic"]["t_m"])
    rvec = cv2.Rodrigues(R)[0]
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.4), constrained_layout=True)
    target_data = None
    for ax, (name, spec) in zip(axes, RANGES.items()):
        bag = args.data_root / name
        with AnyReader([bag]) as reader:
            _, msg = load_first(reader, "/camera/camera/color/image_raw")
        image = image_rgb(msg)
        points = aggregate_board(bag, spec)
        xyz = np.ascontiguousarray(points[:, :3])
        camera_points = (R @ xyz.T).T + t
        uv = cv2.projectPoints(xyz, rvec, t, K, D)[0].reshape(-1, 2)
        valid = (
            (camera_points[:, 2] > 0)
            & (uv[:, 0] >= 0)
            & (uv[:, 0] < image.shape[1])
            & (uv[:, 1] >= 0)
            & (uv[:, 1] < image.shape[0])
        )
        uv = uv[valid]
        depth = camera_points[valid, 2]
        observation = result["calibration_observations"][name]
        camera_polygon = np.asarray(observation["camera_board_corners_px"], dtype=np.float64)
        signed_distance = polygon_distances(uv, camera_polygon)
        strict_count = int(np.count_nonzero(signed_distance >= 0.0))
        tolerance_count = int(np.count_nonzero(signed_distance >= -args.pixel_tolerance))
        strict_pct = 100.0 * strict_count / len(uv)
        accuracy_pct = 100.0 * tolerance_count / len(uv)
        accuracy_pass = bool(accuracy_pct + 1e-12 >= args.required_accuracy_pct)
        in_required_range = bool(TARGET_RANGE_FT[0] <= spec["distance_ft"] <= TARGET_RANGE_FT[1])
        ax.imshow(image)
        ax.scatter(uv[:, 0], uv[:, 1], c=depth, s=6, cmap="turbo", linewidths=0, alpha=0.80)
        closed_polygon = np.vstack([camera_polygon, camera_polygon[0]])
        ax.plot(closed_polygon[:, 0], closed_polygon[:, 1], color="lime", linewidth=1.5)
        status = "PASS" if accuracy_pass else "FAIL"
        range_label = " | 22–30 ft target" if in_required_range else ""
        ax.set_title(
            f"{name}{range_label}\n"
            f"alignment accuracy: {accuracy_pct:.2f}% ({status}, ±{args.pixel_tolerance:g}px)\n"
            f"{len(uv):,} projected board returns"
        )
        ax.axis("off")
        rows.append({
            "dataset": name,
            "distance_ft": spec["distance_ft"],
            "in_required_22_to_30_ft_range": in_required_range,
            "points": int(len(uv)),
            "depth_min_m": float(depth.min()),
            "depth_max_m": float(depth.max()),
            "u_min": float(uv[:, 0].min()),
            "u_max": float(uv[:, 0].max()),
            "v_min": float(uv[:, 1].min()),
            "v_max": float(uv[:, 1].max()),
            "camera_board_corners_px": camera_polygon.tolist(),
            "strict_inside_board_count": strict_count,
            "strict_inside_board_pct": strict_pct,
            "within_tolerance_count": tolerance_count,
            "alignment_accuracy_pct": accuracy_pct,
            "pixel_tolerance": args.pixel_tolerance,
            "required_accuracy_pct": args.required_accuracy_pct,
            "accuracy_requirement_pass": accuracy_pass,
        })
        if name == TARGET_DATASET:
            target_data = (image, uv, depth, camera_polygon, accuracy_pct, accuracy_pass)

    fig.suptitle(
        "Checkerboard Camera–LiDAR Alignment Validation — 99% Requirement",
        fontsize=16, fontweight="bold", y=1.08,
    )
    figure_path = args.output_dir / "calibration_validation.png"
    fig.savefig(figure_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    # The mentor-facing close-up now shows the required 22--30 ft calibration
    # range rather than the original 7.25 ft initialization view.
    image, uv, depth, camera_polygon, accuracy_pct, accuracy_pass = target_data
    step = max(1, len(uv) // 650)
    shown_uv, shown_depth = uv[::step], depth[::step]
    fig, ax = plt.subplots(figsize=(11, 8), constrained_layout=True)
    ax.imshow(image)
    scatter = ax.scatter(
        shown_uv[:, 0], shown_uv[:, 1], c=shown_depth, s=24, cmap="turbo",
        linewidths=0.45, edgecolors="black", alpha=0.82,
    )
    lidar_bbox = (uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max())
    ax.add_patch(plt.Polygon(camera_polygon, closed=True, fill=False, color="lime", linewidth=2.2))
    x1, y1, x2, y2 = lidar_bbox
    ax.add_patch(plt.Rectangle(
        (x1, y1), x2 - x1, y2 - y1, fill=False, color="yellow", linewidth=2.2
    ))
    label_x, label_y = camera_polygon[0]
    ax.text(label_x, label_y - 3, "camera checkerboard boundary", color="lime", fontsize=10,
            fontweight="bold", bbox={"facecolor": "black", "alpha": 0.55, "pad": 2})
    ax.text(x1, y1 + 8, "projected LiDAR board bbox", color="yellow", fontsize=10,
            fontweight="bold", bbox={"facecolor": "black", "alpha": 0.55, "pad": 2})
    status = "PASS" if accuracy_pass else "FAIL"
    ax.set_title(
        f"Target-Range Calibration — 22.2 ft | {accuracy_pct:.2f}% alignment ({status})",
        fontsize=15, fontweight="bold",
    )
    ax.axis("off")
    colorbar = fig.colorbar(scatter, ax=ax, fraction=0.035, pad=0.02)
    colorbar.set_label("camera depth Z (m)")
    closeup_path = args.output_dir / "calibration_closeup_validation.png"
    fig.savefig(closeup_path, dpi=180, bbox_inches="tight")
    target_path = args.output_dir / "calibration_target_range_validation.png"
    fig.savefig(target_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    target_rows = [row for row in rows if row["in_required_22_to_30_ft_range"]]
    target_range_pass = bool(
        target_rows and all(row["accuracy_requirement_pass"] for row in target_rows)
    )
    report = {
        "calibration": str(args.calibration),
        "accuracy_metric": {
            "name": "checkerboard projected-point alignment accuracy",
            "definition": (
                "percentage of selected physical-board LiDAR returns whose projected pixel is "
                "inside, or no more than the stated tolerance outside, the camera-annotated board boundary"
            ),
            "required_accuracy_pct": args.required_accuracy_pct,
            "pixel_tolerance": args.pixel_tolerance,
            "numerator": "projected board returns within the tolerated camera board polygon",
            "denominator": "all selected physical-board LiDAR returns that project into the image",
        },
        "range_requirement": {
            "required_min_ft": TARGET_RANGE_FT[0],
            "required_max_ft": TARGET_RANGE_FT[1],
            "calibration_dataset": TARGET_DATASET,
            "target_range_accuracy_pass": target_range_pass,
        },
        "all_recorded_ranges_accuracy_pass": bool(
            all(row["accuracy_requirement_pass"] for row in rows)
        ),
        "datasets": rows,
        "limitations": [
            "This percentage is board-point alignment accuracy, not camera intrinsic accuracy, object-detection accuracy, or 3D mAP.",
            "The three displayed poses participate in calibration refinement, so the percentage is an in-sample fit metric.",
            "A separate 22--30 ft capture is required to claim independent 99% generalization accuracy.",
        ],
    }
    report_path = args.output_dir / "validation_metrics.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Saved {figure_path}")
    print(f"Saved {closeup_path}")
    print(f"Saved {target_path}")
    print(f"Saved {report_path}")


if __name__ == "__main__":
    main()
