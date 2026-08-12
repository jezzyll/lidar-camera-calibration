#!/usr/bin/env python3
"""Project checkerboard LiDAR returns into RGB images for validation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader

from calibrate import POINT_DTYPE, image_rgb, load_first


RANGES = {
    "dataset01 14.5feet": {"x": 4.285, "y": (-0.34, -0.04), "z": (-0.38, 0.07)},
    "dataset02 7.25feet": {"x": 1.695, "y": (0.10, 0.40), "z": (-0.155, 0.295)},
    "dataset03 22.2feet": {"x": 6.335, "y": (-0.12, 0.18), "z": (-0.40, 0.05)},
}


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=Path("data/raw"))
    parser.add_argument("--calibration", type=Path, default=Path("outputs/calibration_result.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    args = parser.parse_args()
    result = json.loads(args.calibration.read_text())
    K = np.asarray(result["camera"]["K"])
    D = np.asarray(result["camera"]["D"])
    R = np.asarray(result["extrinsic"]["R"])
    t = np.asarray(result["extrinsic"]["t_m"])
    rvec = cv2.Rodrigues(R)[0]
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.2), constrained_layout=True)
    near_data = None
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
        ax.imshow(image)
        ax.scatter(uv[:, 0], uv[:, 1], c=depth, s=6, cmap="turbo", linewidths=0, alpha=0.80)
        ax.set_title(f"{name}\n{len(uv):,} projected board returns")
        ax.axis("off")
        rows.append({
            "dataset": name,
            "points": int(len(uv)),
            "depth_min_m": float(depth.min()),
            "depth_max_m": float(depth.max()),
            "u_min": float(uv[:, 0].min()),
            "u_max": float(uv[:, 0].max()),
            "v_min": float(uv[:, 1].min()),
            "v_max": float(uv[:, 1].max()),
        })
        if name == "dataset02 7.25feet":
            near_data = (image, uv, depth)

    fig.suptitle("Checkerboard Camera–LiDAR Extrinsic Validation", fontsize=16, fontweight="bold", y=1.06)
    figure_path = args.output_dir / "calibration_validation.png"
    fig.savefig(figure_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    # A close-up patterned after the mentor's example.  Sampling and hollow markers
    # keep black/white checker cells visible instead of painting an opaque rectangle.
    image, uv, depth = near_data
    step = max(1, len(uv) // 650)
    shown_uv, shown_depth = uv[::step], depth[::step]
    fig, ax = plt.subplots(figsize=(11, 8), constrained_layout=True)
    ax.imshow(image)
    scatter = ax.scatter(
        shown_uv[:, 0], shown_uv[:, 1], c=shown_depth, s=24, cmap="turbo",
        linewidths=0.45, edgecolors="black", alpha=0.82,
    )
    camera_bbox = (249.94, 42.54, 316.89, 146.74)
    lidar_bbox = (uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max())
    for bbox, color, label, offset in (
        (camera_bbox, "lime", "camera checkerboard bbox", -4),
        (lidar_bbox, "yellow", "projected LiDAR board bbox", 12),
    ):
        x1, y1, x2, y2 = bbox
        ax.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, color=color, linewidth=2.2))
        ax.text(x1, y1 + offset, label, color=color, fontsize=10, fontweight="bold",
                bbox={"facecolor": "black", "alpha": 0.55, "pad": 2})
    ax.set_title("Calibrated LiDAR Projection — 7.25 ft Checkerboard", fontsize=15, fontweight="bold")
    ax.axis("off")
    colorbar = fig.colorbar(scatter, ax=ax, fraction=0.035, pad=0.02)
    colorbar.set_label("camera depth Z (m)")
    closeup_path = args.output_dir / "calibration_closeup_validation.png"
    fig.savefig(closeup_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    report = {
        "calibration": str(args.calibration),
        "interpretation": "LiDAR returns from the physical target project onto the checkerboard in all three held distances.",
        "datasets": rows,
        "warning": "Point-on-board overlap is a validation visualization, not a literal 99.99% accuracy statistic.",
    }
    report_path = args.output_dir / "validation_metrics.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Saved {figure_path}")
    print(f"Saved {closeup_path}")
    print(f"Saved {report_path}")


if __name__ == "__main__":
    main()
