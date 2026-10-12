#!/usr/bin/env python3
"""Rotation-aware, LiDAR-only checker/backing hypothesis extraction.

This module deliberately separates geometric hypothesis generation from
physical identity acceptance.  It never reads camera data or calibration
parameters and never returns a single frozen corner ordering.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

PITCH_M = 0.096
BOARD_WIDTH_M = 7 * PITCH_M
BOARD_HEIGHT_M = 4 * PITCH_M
SUPPORT_MEDIAN_GATE = 20.0
SUPPORT_CONNECTION_GATE = 0.80
SUPPORT_DIRECTION_GATE = 0.90


def _unit(x):
    x = np.asarray(x, float); n = np.linalg.norm(x)
    return x / n if n > 1e-12 else x * np.nan


def _box_sum(h, rows, cols):
    """All metric-window sums. No padding/cropping or dimension mutation."""
    if rows < 1 or cols < 1 or h.shape[0] < rows or h.shape[1] < cols:
        return None
    ii = np.pad(h, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    return ii[rows:, cols:] - ii[:-rows, cols:] - ii[rows:, :-cols] + ii[:-rows, :-cols]


def fit_candidate_plane(points, candidate, plane_gate_m=.045, radius_m=.85):
    p = np.asarray(points, float)
    centre = np.asarray(candidate["centre_xyz_m"], float)
    axes = np.asarray(candidate["axes"], float)
    normal = _unit(axes[2])
    rel = p[:, :3] - centre
    keep = (np.linalg.norm(rel, axis=1) <= radius_m) & (np.abs(rel @ normal) <= plane_gate_m)
    q = p[keep]
    if len(q) < 100:
        return None
    # Deterministic robust SVD refinement, retaining low-intensity returns.
    for _ in range(4):
        c = np.median(q[:, :3], axis=0)
        _, _, vh = np.linalg.svd(q[:, :3] - c, full_matrices=False)
        n = _unit(vh[-1]); residual = (q[:, :3] - c) @ n
        med = np.median(residual); mad = 1.4826 * np.median(np.abs(residual - med)) + 1e-6
        good = np.abs(residual - med) <= max(.012, 3 * mad)
        if good.all(): break
        q = q[good]
    if n @ (-c) < 0: n = -n
    seed_u = axes[0] - n * (axes[0] @ n); seed_u = _unit(seed_u)
    seed_v = _unit(np.cross(n, seed_u))
    return {"points": q, "centre": c, "normal_toward_lidar": n,
            "seed_u": seed_u, "seed_v": seed_v,
            "plane_rmse_m": float(np.sqrt(np.mean(((q[:, :3] - c) @ n) ** 2)))}


def _window_evidence(q, centre, u, v, step_m):
    xy = np.c_[(q[:, :3] - centre) @ u, (q[:, :3] - centre) @ v]
    margin = .03
    lo = xy.min(0) - margin; hi = xy.max(0) + margin
    edges_u = np.arange(lo[0], hi[0] + step_m, step_m)
    edges_v = np.arange(lo[1], hi[1] + step_m, step_m)
    if len(edges_u) < 2 or len(edges_v) < 2: return None
    h, _, _ = np.histogram2d(xy[:, 0], xy[:, 1], bins=[edges_u, edges_v])
    ku = int(round(BOARD_WIDTH_M / step_m)); kv = int(round(BOARD_HEIGHT_M / step_m))
    sums = _box_sum(h, ku, kv)
    if sums is None: return None
    i, j = np.unravel_index(np.argmax(sums), sums.shape)
    cu = edges_u[i] + ku * step_m / 2; cv = edges_v[j] + kv * step_m / 2
    local = xy - [cu, cv]
    inside = (np.abs(local[:, 0]) <= BOARD_WIDTH_M / 2) & (np.abs(local[:, 1]) <= BOARD_HEIGHT_M / 2)
    edge_band = .018
    edge = [
        inside & (np.abs(local[:, 0] + BOARD_WIDTH_M / 2) <= edge_band),
        inside & (np.abs(local[:, 0] - BOARD_WIDTH_M / 2) <= edge_band),
        inside & (np.abs(local[:, 1] + BOARD_HEIGHT_M / 2) <= edge_band),
        inside & (np.abs(local[:, 1] - BOARD_HEIGHT_M / 2) <= edge_band),
    ]
    cell_counts = np.zeros((4, 7), int); cell_medians = np.full((4, 7), np.nan)
    col = np.floor((local[:, 0] + BOARD_WIDTH_M / 2) / PITCH_M).astype(int)
    row = np.floor((local[:, 1] + BOARD_HEIGHT_M / 2) / PITCH_M).astype(int)
    valid = inside & (col >= 0) & (col < 7) & (row >= 0) & (row < 4)
    for rr in range(4):
        for cc in range(7):
            z = valid & (row == rr) & (col == cc)
            cell_counts[rr, cc] = int(z.sum())
            if z.any(): cell_medians[rr, cc] = float(np.median(q[z, 3]))
    even = cell_medians[(np.indices((4, 7)).sum(0) % 2) == 0]
    odd = cell_medians[(np.indices((4, 7)).sum(0) % 2) == 1]
    finite_even = even[np.isfinite(even)]; finite_odd = odd[np.isfinite(odd)]
    sep = 0.0
    if len(finite_even) and len(finite_odd):
        pooled = np.nanstd(cell_medians) + 1e-9
        sep = float(abs(np.median(finite_even) - np.median(finite_odd)) / pooled)
    edge_counts = [int(x.sum()) for x in edge]
    # Geometry score chooses hypotheses only; it never establishes identity.
    score = float(inside.sum() + 3 * min(edge_counts) + 10 * np.count_nonzero(cell_counts) + 20 * sep)
    world_centre = centre + cu * u + cv * v
    return {"centre_xyz_m": world_centre.tolist(), "point_count": int(inside.sum()),
            "edge_counts": edge_counts, "occupied_cells": int(np.count_nonzero(cell_counts)),
            "checker_separation_abs_std": sep, "score": score,
            "cell_counts": cell_counts.tolist(), "u_axis": u.tolist(), "v_axis": v.tolist(),
            "normal_toward_lidar": None, "local_points_uv": local[inside].tolist()}


def board_plane_hypotheses(points, candidate, angle_step_deg=3.0, step_m=.012, keep=8):
    fit = fit_candidate_plane(points, candidate)
    if fit is None:
        return {"status": "UNRESOLVED", "reason": "fewer than 100 robust local-plane returns", "hypotheses": []}
    raw = []
    for angle_deg in np.arange(0.0, 180.0, angle_step_deg):
        a = math.radians(float(angle_deg))
        u = math.cos(a) * fit["seed_u"] + math.sin(a) * fit["seed_v"]
        v = _unit(np.cross(fit["normal_toward_lidar"], u))
        e = _window_evidence(fit["points"], fit["centre"], u, v, step_m)
        if e is not None:
            e.update({"angle_deg": float(angle_deg), "plane_rmse_m": fit["plane_rmse_m"],
                      "normal_toward_lidar": fit["normal_toward_lidar"].tolist()})
            raw.append(e)
    raw.sort(key=lambda x: x["score"], reverse=True)
    distinct = []
    for h in raw:
        c = np.asarray(h["centre_xyz_m"])
        if any(np.linalg.norm(c - np.asarray(k["centre_xyz_m"])) < .04 and
               min(abs(h["angle_deg"]-k["angle_deg"]), 180-abs(h["angle_deg"]-k["angle_deg"])) < 8 for k in distinct):
            continue
        h = dict(h); h["hypothesis_id"] = f"plane_rect_{len(distinct):02d}"
        distinct.append(h)
        if len(distinct) >= keep: break
    return {"status": "UNRESOLVED", "reason": "geometric rectangles are hypotheses; polarity/index identity not assigned",
            "metric_step_m": step_m, "physical_window_m": [BOARD_WIDTH_M, BOARD_HEIGHT_M],
            "plane_point_count": int(len(fit["points"])), "plane_rmse_m": fit["plane_rmse_m"],
            "hypotheses": distinct}


def support_evidence(messages, hypothesis):
    centre = np.asarray(hypothesis["centre_xyz_m"]); u0 = np.asarray(hypothesis["u_axis"])
    v0 = np.asarray(hypothesis["v_axis"]); n = np.asarray(hypothesis["normal_toward_lidar"])
    alternatives = []
    # Both physical lower-edge directions remain until pedestal evidence resolves one.
    for bottom_sign in (-1, 1):
        down = bottom_sign * v0; right = _unit(np.cross(down, n))
        counts=[]; connected=[]; centroids=[]; masks=[]
        for msg in messages:
            p=np.asarray(msg["points"]); rel=p[:,:3]-centre
            x=rel@right; y=rel@down; z=rel@n
            mask=(np.abs(x)<=.18)&(y>=BOARD_HEIGHT_M/2-.02)&(y<=1.2)&(np.abs(z)<=.08)
            q=p[mask]; masks.append(mask); counts.append(int(len(q)))
            connected.append(bool(len(q) and np.min(y[mask])<=BOARD_HEIGHT_M/2+.09))
            if len(q): centroids.append(q[:,:3].mean(0))
        direction=0.0
        if centroids:
            direction=float(np.mean((np.asarray(centroids)-centre)@down>0))
        med=float(np.median(counts)) if counts else 0.0; conn=float(np.mean(connected)) if connected else 0.0
        passes=bool(med>=SUPPORT_MEDIAN_GATE and conn>=SUPPORT_CONNECTION_GATE and direction>=SUPPORT_DIRECTION_GATE)
        alternatives.append({"bottom_sign":bottom_sign,"right_axis":right.tolist(),"down_axis":down.tolist(),
                             "support_count_total":int(sum(counts)),"support_count_per_scan_median":med,
                             "connection_frequency":conn,"bottom_direction_stability":direction,
                             "passes_frozen_support_gate":passes,"message_counts":counts,"point_masks":masks})
    return alternatives


def inner_corner_hypotheses(hypothesis, support):
    """Return both left/right index polarities; neither is automatically frozen."""
    centre=np.asarray(hypothesis["centre_xyz_m"]); down=np.asarray(support["down_axis"]); right=np.asarray(support["right_axis"])
    out=[]
    for left_sign in (-1,1):
        axis=left_sign*right
        corners=np.asarray([centre+axis*((col-2.5)*PITCH_M)+down*((row-1)*PITCH_M)
                            for row in range(3) for col in range(6)])
        out.append({"left_right_sign":left_sign,"corners_xyz_m":corners.tolist(),
                    "identity_status":"HYPOTHESIS_ONLY"})
    return out
