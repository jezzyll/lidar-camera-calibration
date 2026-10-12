#!/usr/bin/env python3
"""Checkerboard camera-LiDAR calibration for the supplied ROS 2 bags.

Camera intrinsics are read from CameraInfo.  The high-resolution near view gives
the initial transform, then independently annotated board boundaries from all
three recordings refine it.  The refinement is required to include the 22.2 ft
recording, so the final solve contains a calibration constraint in the requested
22--30 ft working range instead of being driven only by the 7.25 ft view.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import re
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


POINT_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
        "offsets": [0, 4, 8, 12, 16, 17, 18],
        "itemsize": 26,
    }
)


# The middle and far boards are too small for reliable 11x17 internal-corner
# detection (the far board is only about 19x30 pixels).  These outer boundaries
# were annotated once on the unmodified first RGB frames.  Keeping the reference
# pixels explicit makes the long-range constraint and its uncertainty auditable.
CALIBRATION_POSES = {
    "dataset01 14.5feet": {
        "distance_ft": 14.5,
        "x": 4.285,
        "y": (-0.34, -0.04),
        "z": (-0.38, 0.07),
        "camera_board_corners_px": (
            (323.5, 122.0), (351.5, 122.5), (351.5, 165.0), (323.5, 165.0),
        ),
        "camera_boundary_source": "manual outer-boundary annotation on first RGB frame",
    },
    "dataset02 7.25feet": {
        "distance_ft": 7.25,
        "x": 1.695,
        "y": (0.10, 0.40),
        "z": (-0.155, 0.295),
        "camera_board_corners_px": None,
        "camera_boundary_source": "11x17 subpixel checker corners plus known one-square border",
    },
    "dataset03 22.2feet": {
        "distance_ft": 22.2,
        "x": 6.335,
        "y": (-0.12, 0.18),
        "z": (-0.40, 0.05),
        "camera_board_corners_px": (
            (304.0, 121.5), (323.0, 121.5), (323.0, 151.5), (304.0, 151.5),
        ),
        "camera_boundary_source": "manual outer-boundary annotation on first RGB frame",
    },
}
TARGET_RANGE_FT = (22.0, 30.0)
TARGET_DATASET = "dataset03 22.2feet"


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
    return R_board_camera, center_camera, outer_uv, rmse


def aggregate_board(bag: Path, spec: dict, sample_stride: int = 2):
    batches = []
    with AnyReader([bag]) as reader:
        conn = next(c for c in reader.connections if c.topic == "/livox/lidar")
        for index, (connection, _, raw) in enumerate(reader.messages(connections=[conn])):
            if index % sample_stride:
                continue
            msg = reader.deserialize(raw, connection.msgtype)
            points = np.frombuffer(msg.data, dtype=POINT_DTYPE)
            keep = (
                (np.abs(points["x"] - spec["x"]) < 0.075)
                & (points["y"] >= spec["y"][0])
                & (points["y"] <= spec["y"][1])
                & (points["z"] >= spec["z"][0])
                & (points["z"] <= spec["z"][1])
            )
            p = points[keep]
            batches.append(np.c_[p["x"], p["y"], p["z"]])
    if not batches or not any(len(batch) for batch in batches):
        raise RuntimeError(f"No checkerboard LiDAR returns found in {bag}")
    return np.concatenate(batches).astype(np.float64)


def aggregate_near_board(bag: Path):
    """Compatibility wrapper for the original near-board helper."""
    return aggregate_board(bag, CALIBRATION_POSES["dataset02 7.25feet"])


def fit_lidar_board(points: np.ndarray, y_bounds=(0.10, 0.40), z_bounds=(-0.155, 0.295)):
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

    # Known physical board extents are more stable than extrema from a single
    # sparse scan, especially in the 22--30 ft working range.
    y_center = float(np.mean(y_bounds))
    z_center = float(np.mean(z_bounds))
    center_lidar = np.array([a * y_center + b * z_center + c, y_center, z_center])
    return R_board_lidar, center_lidar, rmse, int(keep.sum()), coef


def lidar_board_corners(coef: np.ndarray, y_bounds, z_bounds):
    """Return TL, TR, BR, BL physical board corners in the LiDAR frame."""
    a, b, c = coef
    y_min, y_max = y_bounds
    z_min, z_max = z_bounds
    yz = ((y_max, z_max), (y_min, z_max), (y_min, z_min), (y_max, z_min))
    return np.asarray([[a * y + b * z + c, y, z] for y, z in yz], dtype=np.float64)


def project(points: np.ndarray, R: np.ndarray, t: np.ndarray, K: np.ndarray, D: np.ndarray):
    return cv2.projectPoints(
        np.ascontiguousarray(points), cv2.Rodrigues(R)[0], t, K, D
    )[0].reshape(-1, 2)


def refine_extrinsic(initial_R, initial_t, observations, K, D,
                     return_diagnostics=False):
    """Jointly fit the transform to the board boundaries at every range.

    When requested, return the exact objective/solver configuration and the
    convergence counters from this solve. SciPy 1.13 does not expose its
    internal trust-region iteration counter in ``OptimizeResult``, so the
    iteration number is captured from the solver's own verbose iteration table.
    """
    object_points = np.concatenate([item["lidar_corners"] for item in observations])
    image_points = np.concatenate([item["camera_corners"] for item in observations])
    initial = np.r_[cv2.Rodrigues(initial_R)[0].ravel(), initial_t]

    def residual(parameters):
        projected = cv2.projectPoints(
            object_points, parameters[:3], parameters[3:], K, D
        )[0].reshape(-1, 2)
        return (projected - image_points).ravel()

    trace = io.StringIO()
    with contextlib.redirect_stdout(trace):
        fitted = least_squares(
            residual, initial, method="trf", jac="2-point",
            loss="soft_l1", f_scale=1.0, max_nfev=5000,
            xtol=1e-13, ftol=1e-13, gtol=1e-13,
            verbose=2 if return_diagnostics else 0,
        )
    if not fitted.success:
        raise RuntimeError(f"Multi-range extrinsic refinement failed: {fitted.message}")
    R = cv2.Rodrigues(fitted.x[:3])[0]
    t = fitted.x[3:]
    projected = project(object_points, R, t, K, D)
    errors = np.linalg.norm(projected - image_points, axis=1)
    result = (R, t, float(np.sqrt(np.mean(errors ** 2))), errors)
    if not return_diagnostics:
        return result

    iteration_numbers = []
    for line in trace.getvalue().splitlines():
        match = re.match(r"\s*(\d+)\s+\d+\s+", line)
        if match:
            iteration_numbers.append(int(match.group(1)))

    def robust_cost(values):
        squared = np.asarray(values, dtype=np.float64) ** 2
        rho = 2.0 * (np.sqrt(1.0 + squared) - 1.0)
        return float(0.5 * np.sum(rho))

    diagnostics = {
        "objective": (
            "minimize F(r,t) = 1/2 * sum_j rho(f_j(r,t)^2), where each "
            "f_j is a projected-minus-observed pixel component (du or dv), "
            "rho(z)=2*(sqrt(1+z)-1), and f_scale=1 px"
        ),
        "residual_definition": (
            "for each calibration board corner k: q_hat_k = project(K,D,R(r),t,P_k); "
            "residual components are [q_hat_k.u-q_k.u, q_hat_k.v-q_k.v]"
        ),
        "optimized_variables": {
            "rotation": "3-component Rodrigues/axis-angle vector r",
            "translation": "3-component translation vector t in metres",
            "total_scalar_variables": 6,
        },
        "fixed_variables": {
            "camera_intrinsic_matrix_K": True,
            "distortion_coefficients_D": True,
            "lidar_board_corner_coordinates": True,
            "camera_board_corner_observations": True,
        },
        "initialization": (
            "near-pose board-frame alignment: R0 = R_board_camera * "
            "R_board_lidar^T; t0 = center_camera - R0*center_lidar"
        ),
        "algorithm": {
            "library": "scipy.optimize.least_squares",
            "method": "trust-region reflective (trf)",
            "jacobian": "2-point finite difference",
            "robust_loss": "soft_l1",
            "f_scale_px": 1.0,
        },
        "convergence_configuration": {
            "xtol": 1e-13,
            "ftol": 1e-13,
            "gtol": 1e-13,
            "maximum_function_evaluations": 5000,
        },
        "convergence_result": {
            "success": bool(fitted.success),
            "status": int(fitted.status),
            "message": str(fitted.message),
            "trust_region_iterations": (
                max(iteration_numbers) if iteration_numbers else None
            ),
            "function_evaluations": int(fitted.nfev),
            "jacobian_evaluations": (
                None if fitted.njev is None else int(fitted.njev)
            ),
            "first_order_optimality": float(fitted.optimality),
            "initial_robust_cost": robust_cost(residual(initial)),
            "final_robust_cost": float(fitted.cost),
        },
        "calibration_corner_observations": int(len(object_points)),
        "scalar_pixel_residuals": int(2 * len(object_points)),
    }
    return (*result, diagnostics)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=Path("data/raw"))
    parser.add_argument("--output", type=Path, default=Path("outputs/calibration_result.json"))
    args = parser.parse_args()
    bags = {name: args.data_root / name for name in CALIBRATION_POSES}
    missing = [str(bag) for bag in bags.values() if not bag.exists()]
    if missing:
        raise SystemExit(f"Missing extracted calibration bag(s): {', '.join(missing)}")
    target_distance = CALIBRATION_POSES[TARGET_DATASET]["distance_ft"]
    if not TARGET_RANGE_FT[0] <= target_distance <= TARGET_RANGE_FT[1]:
        raise SystemExit(
            f"Required target-range observation is outside {TARGET_RANGE_FT[0]:g}--"
            f"{TARGET_RANGE_FT[1]:g} ft: {target_distance:g} ft"
        )

    near = bags["dataset02 7.25feet"]

    K, D, width, height, model, camera_frame = extract_intrinsics(near)
    with AnyReader([near]) as reader:
        _, image_msg = load_first(reader, "/camera/camera/color/image_raw")
    corners = detect_camera_board(image_rgb(image_msg))
    R_bc, center_c, near_camera_corners, camera_rmse = fit_camera_board(corners, K, D)
    near_spec = CALIBRATION_POSES["dataset02 7.25feet"]
    lidar_points = aggregate_board(near, near_spec)
    R_bl, center_l, plane_rmse, inliers, _ = fit_lidar_board(
        lidar_points, near_spec["y"], near_spec["z"]
    )

    initial_R = R_bc @ R_bl.T
    initial_t = center_c - initial_R @ center_l

    observations = []
    plane_quality = {}
    for name, spec in CALIBRATION_POSES.items():
        points = aggregate_board(bags[name], spec)
        _, _, pose_plane_rmse, pose_inliers, coef = fit_lidar_board(
            points, spec["y"], spec["z"]
        )
        camera_corners = (
            near_camera_corners if name == "dataset02 7.25feet"
            else np.asarray(spec["camera_board_corners_px"], dtype=np.float64)
        )
        observations.append({
            "dataset": name,
            "lidar_corners": lidar_board_corners(coef, spec["y"], spec["z"]),
            "camera_corners": camera_corners,
        })
        plane_quality[name] = {
            "distance_ft": spec["distance_ft"],
            "lidar_board_plane_rmse_m": pose_plane_rmse,
            "lidar_plane_inliers": pose_inliers,
        }

    R, t, joint_corner_rmse, corner_errors = refine_extrinsic(
        initial_R, initial_t, observations, K, D
    )
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    quat = Rotation.from_matrix(R).as_quat()

    observation_report = {}
    offset = 0
    for item in observations:
        name = item["dataset"]
        errors = corner_errors[offset:offset + 4]
        offset += 4
        distance_ft = plane_quality[name]["distance_ft"]
        observation_report[name] = {
            **plane_quality[name],
            "in_required_22_to_30_ft_range": bool(
                TARGET_RANGE_FT[0] <= distance_ft <= TARGET_RANGE_FT[1]
            ),
            "camera_board_corners_px": item["camera_corners"].tolist(),
            "camera_boundary_source": CALIBRATION_POSES[name]["camera_boundary_source"],
            "board_corner_reprojection_rmse_px": float(np.sqrt(np.mean(errors ** 2))),
        }

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
            "method": "near_pose_initialization_plus_multi_range_board_boundary_refinement",
        },
        "quality": {
            "camera_checkerboard_reprojection_rmse_px": camera_rmse,
            "lidar_board_plane_rmse_m": plane_rmse,
            "lidar_plane_inliers": inliers,
            "joint_board_corner_reprojection_rmse_px": joint_corner_rmse,
        },
        "range_requirement": {
            "required_min_ft": TARGET_RANGE_FT[0],
            "required_max_ft": TARGET_RANGE_FT[1],
            "calibration_dataset": TARGET_DATASET,
            "calibration_distance_ft": target_distance,
            "pass": True,
        },
        "calibration_observations": observation_report,
        "limitations": [
            "Each bag contains one fixed board pose; these data cannot independently re-estimate camera intrinsics.",
            "The 22.2 ft board is only about 19 x 30 camera pixels, so its outer boundary is annotated rather than claiming unreliable internal-corner detections.",
            "All three poses participate in the final refinement; validation on these same poses measures alignment fit, not independent generalization.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(yaml.safe_dump(result, sort_keys=False))
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
