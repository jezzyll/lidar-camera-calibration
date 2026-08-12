#!/usr/bin/env python3
"""Checkerboard camera-LiDAR calibration for the supplied ROS 2 bags.

The recordings contain a fixed checkerboard at three ranges.  Camera intrinsics
are read from CameraInfo.  The near bag supplies enough pixels/points to solve
the checkerboard pose in the camera and its plane/bounds in LiDAR.  These two
poses give the rigid transform from livox_frame to camera_color_optical_frame.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation


POINT_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
        "offsets": [0, 4, 8, 12, 16, 17, 18],
        "itemsize": 26,
    }
)


def load_first(reader: AnyReader, topic: str):
    connection = next(c for c in reader.connections if c.topic == topic)
    conn, stamp, raw = next(reader.messages(connections=[connection]))
    return stamp, reader.deserialize(raw, conn.msgtype)


def image_rgb(msg) -> np.ndarray:
    if msg.encoding != "rgb8":
        raise ValueError(f"Expected rgb8, got {msg.encoding}")
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()


def extract_intrinsics(bag: Path):
    with AnyReader([bag]) as reader:
        _, msg = load_first(reader, "/camera/camera/color/camera_info")
    return (
        np.asarray(msg.k, dtype=np.float64).reshape(3, 3),
        np.asarray(msg.d, dtype=np.float64),
        int(msg.width),
        int(msg.height),
        msg.distortion_model,
        msg.header.frame_id,
    )


def detect_camera_board(rgb: np.ndarray):
    """Detect the 11x17 inner-corner grid in the high-resolution near pose."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    # A tight ROI prevents the tripod/floor corners from confusing SB detection.
    x1, y1, x2, y2 = 238, 30, 326, 160
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCornersSB(gray[y1:y2, x1:x2], (11, 17), flags)
    if not ok:
        raise RuntimeError("11x17 checkerboard was not detected in the near bag")
    grid = corners.reshape(17, 11, 2) + np.array([x1, y1])
    # SB ordering is bottom-right to top-left in this recording.
    return grid[::-1, ::-1].copy()


def fit_camera_board(corners: np.ndarray, K: np.ndarray, D: np.ndarray):
    square = 0.025
    obj = np.zeros((17, 11, 3), np.float64)
    obj[:, :, 0] = np.arange(11)[None, :] * square
    obj[:, :, 1] = np.arange(17)[:, None] * square
    ok, rvec, tvec = cv2.solvePnP(obj.reshape(-1, 3), corners.reshape(-1, 2), K, D)
    if not ok:
        raise RuntimeError("Camera checkerboard PnP failed")
    R_board_camera = cv2.Rodrigues(rvec)[0]
    t_board_camera = tvec[:, 0]
    reproj = cv2.projectPoints(obj.reshape(-1, 3), rvec, tvec, K, D)[0].reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum((reproj - corners.reshape(-1, 2)) ** 2, axis=1))))
    # Center of the 0.30 x 0.45 m board (12 x 18 squares).
    center_camera = R_board_camera @ np.array([0.125, 0.200, 0.0]) + t_board_camera
    outer = np.array(
        [[-square, -square, 0.0], [0.275, -square, 0.0],
         [0.275, 0.425, 0.0], [-square, 0.425, 0.0]],
        dtype=np.float64,
    )
    outer_uv = cv2.projectPoints(outer, rvec, tvec, K, D)[0].reshape(-1, 2)
    outer_bbox = np.r_[outer_uv.min(axis=0), outer_uv.max(axis=0)]
    return R_board_camera, center_camera, outer_bbox, rmse


def aggregate_near_board(bag: Path):
    batches = []
    with AnyReader([bag]) as reader:
        conn = next(c for c in reader.connections if c.topic == "/livox/lidar")
        for index, (connection, _, raw) in enumerate(reader.messages(connections=[conn])):
            if index % 2:
                continue
            msg = reader.deserialize(raw, connection.msgtype)
            points = np.frombuffer(msg.data, dtype=POINT_DTYPE)
            keep = (
                (points["x"] > 1.62)
                & (points["x"] < 1.77)
                & (points["y"] > 0.099)
                & (points["y"] < 0.399)
                & (points["z"] > -0.155)
                & (points["z"] < 0.295)
            )
            p = points[keep]
            batches.append(np.c_[p["x"], p["y"], p["z"]])
    return np.concatenate(batches).astype(np.float64)


def fit_lidar_board(points: np.ndarray):
    # Robustly fit x = a*y + b*z + c.
    A = np.c_[points[:, 1], points[:, 2], np.ones(len(points))]
    coef = np.linalg.lstsq(A, points[:, 0], rcond=None)[0]
    keep = np.ones(len(points), dtype=bool)
    for _ in range(3):
        residual = points[:, 0] - A @ coef
        mad = np.median(np.abs(residual - np.median(residual))) * 1.4826
        keep = np.abs(residual) < 3.0 * max(float(mad), 0.002)
        coef = np.linalg.lstsq(A[keep], points[keep, 0], rcond=None)[0]
    a, b, c = coef
    rmse = float(np.sqrt(np.mean((points[keep, 0] - A[keep] @ coef) ** 2)))

    normal = np.array([1.0, -a, -b])
    normal /= np.linalg.norm(normal)
    right = np.array([0.0, -1.0, 0.0])
    right -= normal * np.dot(normal, right)
    right /= np.linalg.norm(right)
    down = np.cross(normal, right)
    down /= np.linalg.norm(down)
    R_board_lidar = np.column_stack([right, down, normal])

    # Board boundary found from the long aggregate: y=[0.099,0.399],
    # z=[-0.155,0.295].  Center is robust to boundary sampling density.
    y_center, z_center = 0.249, 0.070
    center_lidar = np.array([a * y_center + b * z_center + c, y_center, z_center])
    return R_board_lidar, center_lidar, rmse, int(keep.sum())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=Path("data/raw"))
    parser.add_argument("--output", type=Path, default=Path("outputs/calibration_result.json"))
    args = parser.parse_args()
    near = args.data_root / "dataset02 7.25feet"
    if not near.exists():
        raise SystemExit(f"Missing extracted near bag: {near}")

    K, D, width, height, model, camera_frame = extract_intrinsics(near)
    with AnyReader([near]) as reader:
        _, image_msg = load_first(reader, "/camera/camera/color/image_raw")
    corners = detect_camera_board(image_rgb(image_msg))
    R_bc, center_c, camera_bbox, camera_rmse = fit_camera_board(corners, K, D)
    lidar_points = aggregate_near_board(near)
    R_bl, center_l, plane_rmse, inliers = fit_lidar_board(lidar_points)

    R = R_bc @ R_bl.T
    t = center_c - R @ center_l
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    quat = Rotation.from_matrix(R).as_quat()

    # Compare the independently extracted LiDAR boundary with the camera boundary.
    boundary_lidar = np.array(
        [[1.693, y, z] for y in (0.099, 0.399) for z in (-0.155, 0.295)],
        dtype=np.float64,
    )
    boundary_uv = cv2.projectPoints(boundary_lidar, cv2.Rodrigues(R)[0], t, K, D)[0].reshape(-1, 2)
    lidar_bbox = np.r_[boundary_uv.min(axis=0), boundary_uv.max(axis=0)]
    bbox_rmse = float(np.sqrt(np.mean((lidar_bbox - camera_bbox) ** 2)))

    result = {
        "direction": "p_camera_color_optical = R * p_livox + t",
        "camera": {
            "frame": camera_frame,
            "width": width,
            "height": height,
            "distortion_model": model,
            "K": K.tolist(),
            "D": D.tolist(),
            "source": "recorded CameraInfo (factory/device calibration)",
        },
        "checkerboard": {"inner_corners": [11, 17], "square_size_m": 0.025},
        "extrinsic": {
            "parent_frame": camera_frame,
            "child_frame": "livox_frame",
            "R": R.tolist(),
            "t_m": t.tolist(),
            "quaternion_xyzw": quat.tolist(),
            "T_camera_lidar": T.tolist(),
        },
        "quality": {
            "camera_checkerboard_reprojection_rmse_px": camera_rmse,
            "lidar_board_plane_rmse_m": plane_rmse,
            "lidar_plane_inliers": inliers,
            "near_pose_board_bbox_rmse_px": bbox_rmse,
        },
        "limitations": [
            "Each bag contains one fixed board pose; these data cannot independently re-estimate camera intrinsics.",
            "Near pose drives the extrinsic; middle/far bags are held out for visual validation.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(yaml.safe_dump(result, sort_keys=False))
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
