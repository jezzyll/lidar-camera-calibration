#!/usr/bin/env python3
"""Rotation-invariant LiDAR target candidate discovery.

This module deliberately stops at candidate discovery.  It never uses camera
data and never chooses a board origin, polarity, or indexed corners.
"""
from __future__ import annotations

import numpy as np
from sklearn.cluster import DBSCAN


def discover_candidates(points, expected_range_m, shell_half_width_m=.55,
                        intensity_floor=100., eps_m=.08, min_samples=10):
    p = np.asarray(points, float)
    finite = np.isfinite(p[:, :4]).all(1)
    p = p[finite]
    radial = np.linalg.norm(p[:, :3], axis=1)
    shell = p[np.abs(radial - expected_range_m) < shell_half_width_m]
    seed = shell[shell[:, 3] >= intensity_floor]
    candidates = []
    if len(seed) >= min_samples:
        labels = DBSCAN(eps=eps_m, min_samples=min_samples).fit_predict(seed[:, :3])
        for label in sorted(set(labels)):
            if label < 0:
                continue
            q = seed[labels == label]
            if len(q) < 20:
                continue
            centre = q[:, :3].mean(0)
            _, singular, axes = np.linalg.svd(q[:, :3] - centre, full_matrices=False)
            uv = (q[:, :3] - centre) @ axes[:2].T
            span = np.ptp(np.percentile(uv, [1, 99], axis=0), axis=0)
            candidates.append({
                "candidate_id": f"range_cluster_{int(label):03d}",
                "label": int(label), "point_count": int(len(q)),
                "centre_xyz_m": centre.tolist(),
                "radial_distance_m": float(np.linalg.norm(centre)),
                "azimuth_deg": float(np.degrees(np.arctan2(centre[1], centre[0]))),
                "plane_sigma_m": float(singular[2] / np.sqrt(len(q))),
                "p01_p99_plane_span_m": span.tolist(),
                "axes": axes.tolist(),
            })
    return {
        "method": "radial range shell followed by intensity-seeded 3-D DBSCAN",
        "selection_policy": "enumerate every cluster; never rank/select using camera data",
        "range_shell_points": int(len(shell)),
        "seed_points": int(len(seed)),
        "candidates": candidates,
        "identity_status": "UNRESOLVED",
    }


def rotate_points(points, rotation):
    p = np.asarray(points, float).copy()
    p[:, :3] = p[:, :3] @ np.asarray(rotation, float).T
    return p
