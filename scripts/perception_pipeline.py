#!/usr/bin/env python3
"""Auditable LiDAR preprocessing and model-stage orchestrator.

Implemented stages:
  ROS 2 RGB/Livox extraction -> nearest-timestamp synchronization ->
  RANSAC ground removal -> statistical outlier removal -> DBSCAN ->
  geometric ROI proposals -> float32 .bin conversion -> oriented proposal boxes.

CenterPoint, BEVFusion, and fine-tuning are *gated* real model stages. They run
only when MMDetection3D plus user-supplied configs/checkpoints/annotations are
available; this program never relabels DBSCAN proposals as neural detections.
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
from dataclasses import dataclass
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Rectangle
from rosbags.highlevel import AnyReader
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors

from calibrate import POINT_DTYPE, image_rgb


@dataclass
class OrientedBox:
    label: int
    center: np.ndarray
    dimensions: np.ndarray  # length, width, height
    yaw: float
    point_count: int
    corners_xy: np.ndarray


def first_camera_info(reader: AnyReader, topic: str):
    conn = next(c for c in reader.connections if c.topic == topic)
    connection, _, raw = next(reader.messages(connections=[conn]))
    return reader.deserialize(raw, connection.msgtype)


def nearest_synchronized_pair(bag: Path, topics: dict, requested_time_s: float | None):
    with AnyReader([bag]) as reader:
        image_conn = next(c for c in reader.connections if c.topic == topics["image"])
        lidar_conn = next(c for c in reader.connections if c.topic == topics["lidar"])
        images = []
        lidars = []
        for connection, stamp, raw in reader.messages(connections=[image_conn, lidar_conn]):
            if connection.topic == topics["image"]:
                images.append((stamp, raw, connection))
            else:
                lidars.append((stamp, raw, connection))
        if not images or not lidars:
            raise RuntimeError("Bag does not contain both RGB and LiDAR messages")
        image_stamps = np.asarray([x[0] for x in images], dtype=np.int64)
        lidar_stamps = np.asarray([x[0] for x in lidars], dtype=np.int64)
        start = min(int(image_stamps[0]), int(lidar_stamps[0]))
        target = start + int(requested_time_s * 1e9) if requested_time_s is not None else int(lidar_stamps[len(lidars) // 2])
        lidar_index = int(np.argmin(np.abs(lidar_stamps - target)))
        image_index = int(np.argmin(np.abs(image_stamps - lidar_stamps[lidar_index])))
        lidar_stamp, lidar_raw, lidar_connection = lidars[lidar_index]
        image_stamp, image_raw, image_connection = images[image_index]
        lidar_msg = reader.deserialize(lidar_raw, lidar_connection.msgtype)
        image_msg = reader.deserialize(image_raw, image_connection.msgtype)
        info = first_camera_info(reader, topics["camera_info"])
    delta_ms = abs(image_stamp - lidar_stamp) / 1e6
    points = np.frombuffer(lidar_msg.data, dtype=POINT_DTYPE)
    xyzi = np.c_[points["x"], points["y"], points["z"], points["intensity"]].astype(np.float64)
    return image_rgb(image_msg), xyzi, int(image_stamp), int(lidar_stamp), float(delta_ms), info


def crop_roi(points: np.ndarray, roi: dict):
    keep = np.ones(len(points), dtype=bool)
    for column, key in enumerate(("x", "y", "z")):
        lo, hi = roi[key]
        keep &= (points[:, column] >= lo) & (points[:, column] <= hi)
    return points[keep]


def validate_calibration(path: Path, image: np.ndarray, info, tolerance: float):
    """Load calibration and prevent silent cross-session intrinsic misuse."""
    if not path.exists():
        raise FileNotFoundError(f"Calibration result does not exist: {path}")
    calibration = json.loads(path.read_text())
    camera = calibration["camera"]
    resolution = camera.get("resolution", [camera.get("width"), camera.get("height")])
    expected = [int(image.shape[1]), int(image.shape[0])]
    if list(resolution) != expected:
        raise RuntimeError(
            f"Calibration resolution {resolution} does not match synchronized RGB frame {expected}"
        )
    calibration_k = np.asarray(camera["K"], dtype=np.float64).reshape(3, 3)
    recorded_k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    intrinsic_delta = float(np.max(np.abs(calibration_k - recorded_k)))
    if intrinsic_delta > tolerance:
        raise RuntimeError(
            f"Calibration K differs from recorded CameraInfo by {intrinsic_delta:.6f}, "
            f"above tolerance {tolerance:.6f}"
        )
    transform = np.asarray(calibration["extrinsic"]["T_camera_lidar"], dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise RuntimeError("Calibration T_camera_lidar must be a finite 4x4 matrix")
    return calibration, transform, intrinsic_delta


def ransac_ground(points: np.ndarray, cfg: dict, seed: int = 7):
    """Fit an approximately horizontal plane from low points."""
    if len(points) < 3:
        return np.zeros(len(points), dtype=bool), None
    rng = np.random.default_rng(seed)
    max_z = min(float(np.percentile(points[:, 2], 35)), float(cfg["max_candidate_height_m"]))
    candidates = np.flatnonzero(points[:, 2] <= max_z)
    if len(candidates) < 3:
        candidates = np.arange(len(points))
    best = np.zeros(len(points), dtype=bool)
    best_plane = None
    threshold = float(cfg["distance_threshold_m"])
    for _ in range(int(cfg["iterations"])):
        ids = rng.choice(candidates, 3, replace=False)
        p0, p1, p2 = points[ids, :3]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        if normal[2] < 0:
            normal = -normal
        if normal[2] < float(cfg["min_up_normal_z"]):
            continue
        d = -float(np.dot(normal, p0))
        distance = np.abs(points[:, :3] @ normal + d)
        inliers = distance <= threshold
        if inliers.sum() > best.sum():
            best, best_plane = inliers, np.r_[normal, d]
    return best, best_plane


def statistical_outlier_filter(points: np.ndarray, cfg: dict):
    if len(points) <= 3:
        return points, np.ones(len(points), dtype=bool), {"mean_neighbor_distance_m": 0.0, "cutoff_m": 0.0}
    neighbors = min(int(cfg["neighbors"]), len(points) - 1)
    model = NearestNeighbors(n_neighbors=neighbors + 1, algorithm="auto", n_jobs=-1).fit(points[:, :3])
    distances, _ = model.kneighbors(points[:, :3])
    mean_distance = distances[:, 1:].mean(axis=1)
    cutoff = float(mean_distance.mean() + float(cfg["std_ratio"]) * mean_distance.std())
    keep = mean_distance <= cutoff
    return points[keep], keep, {"mean_neighbor_distance_m": float(mean_distance.mean()), "cutoff_m": cutoff}


def cluster_points(points: np.ndarray, cfg: dict):
    if not len(points):
        return np.empty(0, dtype=int)
    return DBSCAN(eps=float(cfg["eps_m"]), min_samples=int(cfg["min_samples"]), n_jobs=-1).fit_predict(points[:, :3])


def oriented_box(points: np.ndarray, label: int):
    xy = points[:, :2]
    center_xy = xy.mean(axis=0)
    covariance = np.cov((xy - center_xy).T)
    values, vectors = np.linalg.eigh(covariance)
    main = vectors[:, np.argmax(values)]
    yaw = math.atan2(float(main[1]), float(main[0]))
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, s], [-s, c]])
    local = (xy - center_xy) @ rotation.T
    local_min, local_max = local.min(axis=0), local.max(axis=0)
    local_center = (local_min + local_max) / 2
    center_xy = center_xy + local_center @ rotation
    length, width = local_max - local_min
    z_min, z_max = float(points[:, 2].min()), float(points[:, 2].max())
    corners_local = np.array([
        [local_min[0], local_min[1]], [local_max[0], local_min[1]],
        [local_max[0], local_max[1]], [local_min[0], local_max[1]],
    ])
    corners_xy = center_xy + (corners_local - local_center) @ rotation
    return OrientedBox(label, np.r_[center_xy, (z_min + z_max) / 2],
                       np.array([length, width, z_max - z_min]), yaw, len(points), corners_xy)


def vehicle_proposals(points: np.ndarray, labels: np.ndarray, cfg: dict):
    proposals = []
    for label in sorted(set(labels.tolist()) - {-1}):
        cluster = points[labels == label]
        if len(cluster) < int(cfg["min_points"]):
            continue
        box = oriented_box(cluster, int(label))
        length, width, height = box.dimensions
        if (
            cfg["length_m"][0] <= length <= cfg["length_m"][1]
            and cfg["width_m"][0] <= width <= cfg["width_m"][1]
            and cfg["height_m"][0] <= height <= cfg["height_m"][1]
        ):
            proposals.append(box)
    return proposals


def save_bins(points: np.ndarray, xyzi_path: Path, model_path: Path, intensity_scale: float):
    output = points[:, :4].astype(np.float32).copy()
    output[:, 3] = np.clip(output[:, 3] / intensity_scale, 0.0, 1.0)
    output.tofile(xyzi_path)
    # MMDetection3D's published nuScenes CenterPoint/BEVFusion pipelines load
    # five fields: x, y, z, intensity, and sweep time lag. A single scan has
    # zero time lag. Do not silently feed four-field records to that config.
    model_input = np.c_[output, np.zeros(len(output), dtype=np.float32)].astype(np.float32)
    model_input.tofile(model_path)
    return output, model_input


def save_undistorted_rgb(image: np.ndarray, path: Path, camera_k: np.ndarray, distortion: np.ndarray):
    # MMDetection3D's projection metadata is a pinhole matrix; lens distortion
    # must therefore be removed from the exported fusion image first.
    undistorted = cv2.undistort(image, camera_k, distortion, None, camera_k)
    if not cv2.imwrite(str(path), cv2.cvtColor(undistorted, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"Failed to save synchronized RGB image: {path}")
    return undistorted


def save_mmdet3d_info(path: Path, image_path: Path, lidar_path: Path, image: np.ndarray,
                      camera_k: np.ndarray, camera_lidar: np.ndarray):
    """Write the calibration handoff used by multimodal inference adapters."""
    cam2img = np.eye(4, dtype=np.float32)
    cam2img[:3, :3] = camera_k.astype(np.float32)
    lidar2cam = camera_lidar.astype(np.float32)
    lidar2img = cam2img @ lidar2cam
    item = {
        "sample_id": 0,
        "images": {
            "CAM_FRONT": {
                "img_path": image_path.name,
                "height": int(image.shape[0]),
                "width": int(image.shape[1]),
                "cam2img": cam2img.tolist(),
                "lidar2cam": lidar2cam.tolist(),
                "lidar2img": lidar2img.tolist(),
            }
        },
        "lidar_points": {"num_pts_feats": 5, "lidar_path": lidar_path.name},
        "instances": [],
    }
    with path.open("wb") as handle:
        pickle.dump({"metainfo": {"DATASET": "LivoxD455"}, "data_list": [item]}, handle)


def box_json(box: OrientedBox):
    return {
        "cluster_label": box.label,
        "center_xyz_m": box.center.tolist(),
        "dimensions_lwh_m": box.dimensions.tolist(),
        "yaw_rad": box.yaw,
        "point_count": box.point_count,
        "semantic_status": "geometric proposal; not a CenterPoint detection",
    }


def save_visualization(image, raw, nonground, filtered, labels, boxes, output: Path):
    fig = plt.figure(figsize=(16, 9), constrained_layout=True)
    grid = fig.add_gridspec(2, 3)
    ax_image = fig.add_subplot(grid[0, 0])
    ax_raw = fig.add_subplot(grid[0, 1])
    ax_ground = fig.add_subplot(grid[0, 2])
    ax_cluster = fig.add_subplot(grid[1, :2])
    ax_stats = fig.add_subplot(grid[1, 2])
    ax_image.imshow(image)
    ax_image.set_title("Nearest synchronized RGB frame")
    ax_image.axis("off")
    ax_raw.scatter(raw[:, 0], raw[:, 1], s=0.7, c=raw[:, 2], cmap="viridis")
    ax_raw.set_title(f"ROI point cloud ({len(raw):,})")
    ax_ground.scatter(nonground[:, 0], nonground[:, 1], s=0.7, c=nonground[:, 2], cmap="viridis")
    ax_ground.set_title(f"After RANSAC ground removal ({len(nonground):,})")
    noise = labels == -1
    ax_cluster.scatter(filtered[noise, 0], filtered[noise, 1], s=0.5, c="#bbbbbb", alpha=0.35)
    good = ~noise
    ax_cluster.scatter(filtered[good, 0], filtered[good, 1], s=1.5, c=labels[good], cmap="tab20")
    for box in boxes:
        closed = np.vstack([box.corners_xy, box.corners_xy[0]])
        ax_cluster.plot(closed[:, 0], closed[:, 1], color="red", linewidth=2)
    ax_cluster.set_title("DBSCAN clusters and dimension-filtered oriented proposals")
    for ax in (ax_raw, ax_ground, ax_cluster):
        ax.set_xlabel("x forward (m)")
        ax.set_ylabel("y left (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.25)
    ax_stats.axis("off")
    ax_stats.text(0, 1, "Pipeline status", fontsize=15, fontweight="bold", va="top")
    ax_stats.text(0, 0.87, f"Statistical-filter points: {len(filtered):,}\n"
                  f"DBSCAN clusters: {len(set(labels.tolist()) - {-1})}\n"
                  f"Vehicle-size proposals: {len(boxes)}\n\n"
                  "CenterPoint/BEVFusion:\nrequires real config + checkpoint +\ncompatible CUDA environment",
                  fontsize=11, va="top", linespacing=1.5)
    fig.suptitle("Livox Perception Preprocessing - Auditable Geometry Stages", fontsize=17, fontweight="bold")
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


def model_stage_status(name: str, cfg: dict):
    config = Path(cfg.get("config", "")) if cfg.get("config") else None
    checkpoint = Path(cfg.get("checkpoint", "")) if cfg.get("checkpoint") else None
    if config is None or checkpoint is None:
        return {"status": "blocked", "reason": "config/checkpoint not configured"}
    if not config.exists() or not checkpoint.exists():
        return {"status": "blocked", "reason": "configured model file does not exist"}
    try:
        import torch  # noqa: F401
        import mmdet3d  # noqa: F401
    except ImportError:
        return {"status": "blocked", "reason": "PyTorch/MMDetection3D not installed"}
    return {
        "status": "ready_for_adapter",
        "reason": "run scripts/mmdet3d_adapter.py; dataset-specific fine-tuning is still required",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path("config/perception_pipeline.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/perception"))
    parser.add_argument("--time-s", type=float)
    parser.add_argument("--calibration", type=Path)
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    args.output_dir.mkdir(parents=True, exist_ok=True)

    image, points, image_stamp, lidar_stamp, delta_ms, info = nearest_synchronized_pair(
        args.bag, cfg["topics"], args.time_s
    )
    if delta_ms > float(cfg["synchronization"]["max_delta_ms"]):
        raise RuntimeError(f"Nearest RGB/LiDAR delta {delta_ms:.2f} ms exceeds configured tolerance")
    calibration_path = args.calibration or Path(cfg["calibration"]["result"])
    calibration, camera_lidar, intrinsic_delta = validate_calibration(
        calibration_path,
        image,
        info,
        float(cfg["calibration"]["intrinsic_abs_tolerance"]),
    )
    roi = crop_roi(points, cfg["preprocessing"]["roi_m"])
    ground_mask, plane = ransac_ground(roi, cfg["preprocessing"]["ground_ransac"])
    nonground = roi[~ground_mask]
    filtered, _, outlier_stats = statistical_outlier_filter(nonground, cfg["preprocessing"]["statistical_outlier"])
    labels = cluster_points(filtered, cfg["preprocessing"]["dbscan"])
    boxes = vehicle_proposals(filtered, labels, cfg["preprocessing"]["vehicle_filter"])
    bin_path = args.output_dir / "frame_xyzi.bin"
    model_bin_path = args.output_dir / "frame_centerpoint_xyzi_time.bin"
    converted, model_input = save_bins(
        filtered, bin_path, model_bin_path, float(cfg["conversion"]["intensity_scale"])
    )
    image_path = args.output_dir / "rgb_undistorted.png"
    info_path = args.output_dir / "mmdet3d_info.pkl"
    camera_k = np.asarray(calibration["camera"]["K"], dtype=np.float64)
    distortion = np.asarray(calibration["camera"]["D"], dtype=np.float64)
    fusion_image = save_undistorted_rgb(image, image_path, camera_k, distortion)
    save_mmdet3d_info(
        info_path,
        image_path,
        model_bin_path,
        fusion_image,
        camera_k,
        camera_lidar,
    )
    visualization = args.output_dir / "preprocessing_visualization.png"
    save_visualization(image, roi, nonground, filtered, labels, boxes, visualization)

    cluster_count = len(set(labels.tolist()) - {-1})
    report = {
        "source_bag": str(args.bag),
        "synchronization": {
            "method": "nearest timestamp",
            "image_timestamp_ns": image_stamp,
            "lidar_timestamp_ns": lidar_stamp,
            "absolute_delta_ms": delta_ms,
            "within_tolerance": True,
        },
        "camera": {"resolution": [int(info.width), int(info.height)], "frame": info.header.frame_id},
        "calibration": {
            "status": "validated",
            "path": str(calibration_path),
            "direction": calibration["direction"],
            "max_intrinsic_abs_delta": intrinsic_delta,
            "T_camera_lidar": camera_lidar.tolist(),
            "quality": calibration.get("quality", {}),
        },
        "counts": {
            "raw_lidar": len(points), "roi": len(roi), "ground": int(ground_mask.sum()),
            "non_ground": len(nonground), "statistical_filtered": len(filtered),
            "dbscan_clusters": cluster_count, "dbscan_noise": int((labels == -1).sum()),
            "vehicle_size_proposals": len(boxes),
        },
        "ground_plane_abcd": plane.tolist() if plane is not None else None,
        "statistical_outlier": outlier_stats,
        "proposals": [box_json(box) for box in boxes],
        "model_inputs": {
            "xyzi_bin": {
                "path": str(bin_path), "dtype": "float32", "shape": list(converted.shape),
                "fields": ["x_m", "y_m", "z_m", "normalized_intensity"],
            },
            "centerpoint_bevfusion_bin": {
                "path": str(model_bin_path), "dtype": "float32", "shape": list(model_input.shape),
                "fields": ["x_m", "y_m", "z_m", "normalized_intensity", "time_lag_s"],
            },
            "rgb": str(image_path),
            "rgb_note": "undistorted pinhole image matching mmdet3d projection metadata",
            "mmdet3d_info": str(info_path),
        },
        "model_stages": {
            "centerpoint": model_stage_status("centerpoint", cfg["models"]["centerpoint"]),
            "bevfusion": model_stage_status("bevfusion", cfg["models"]["bevfusion"]),
            "fine_tuned": {
                "status": "blocked" if not cfg["models"]["fine_tuned"].get("annotations") else "not_run",
                "reason": "labeled training/validation annotations are required",
            },
        },
        "accuracy_note": (
            "DBSCAN boxes are geometric proposals, not semantic detections. Detection mAP/NDS cannot be "
            "reported until a real checkpoint is run and predictions are evaluated against held-out labels."
        ),
    }
    report_path = args.output_dir / "pipeline_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Saved {bin_path}")
    print(f"Saved {model_bin_path}")
    print(f"Saved {image_path}")
    print(f"Saved {info_path}")
    print(f"Saved {visualization}")
    print(f"Saved {report_path}")


if __name__ == "__main__":
    main()
