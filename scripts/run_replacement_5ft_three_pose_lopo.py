#!/usr/bin/env python3
"""Run strict 5/10/15-ft LOPO only from a frozen, all-PASS replacement manifest.

This command is intentionally unavailable until an independently accepted
replacement 5-ft observation has been frozen before projection.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
AUTH = ROOT / "output/background_frame_normalization_delivery/authoritative"
AUDIT = ROOT / "output/new_9p6_dataset_run3/audit.json"
REFERENCE_OBS = AUTH / "frozen_normalized_observations.csv"
THRESHOLD_PX = 2.0
EXPECTED_COUNT = 18


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def rigid(a, b):
    ac, bc = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ac).T @ (b - bc))
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    return r, bc - r @ ac


def model():
    pitch = 0.096
    return np.array([[(c - 2.5) * pitch, -(r - 1) * pitch, 0.0]
                     for r in range(3) for c in range(6)], float)


def pose_initial(rgb, lidar, K, D):
    board = model()
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(board, rgb, K, D, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise RuntimeError("training-only PnP initialization failed")
    choices = []
    for rv, tv in zip(rvecs, tvecs):
        rc, _ = cv2.Rodrigues(rv)
        pc = (rc @ board.T).T + tv.ravel()
        if np.all(pc[:, 2] > 0):
            choices.append((rc, tv.ravel()))
    if not choices:
        raise RuntimeError("training-only PnP initialization has no positive-depth solution")
    rc, tc = choices[0]
    rl, tl = rigid(board, lidar)
    r = rc @ rl.T
    t = tc - r @ tl
    rv, _ = cv2.Rodrigues(r)
    return np.r_[rv.ravel(), t]


def project(params, xyz, K, D):
    r, _ = cv2.Rodrigues(params[:3])
    pc = (r @ xyz.T).T + params[3:]
    uv, _ = cv2.projectPoints(xyz, params[:3], params[3:], K, D)
    return uv.reshape(-1, 2), pc


def fit(training, K, D):
    init = pose_initial(training[0]["rgb"], training[0]["lidar"], K, D)

    def residual(params):
        values = []
        for pose in training:
            uv, pc = project(params, pose["lidar"], K, D)
            if np.any(pc[:, 2] <= 0):
                return np.full(sum(len(x["rgb"]) for x in training) * 2, 1e6)
            values.append(((uv - pose["rgb"]) / pose["sigma_px"][:, None]).ravel())
        return np.concatenate(values)

    solution = least_squares(residual, init, loss="linear", max_nfev=5000,
                             ftol=1e-13, xtol=1e-13, gtol=1e-13)
    singular = np.linalg.svd(solution.jac, compute_uv=False)
    return solution.x, {"rank": int(np.linalg.matrix_rank(solution.jac)),
                        "singular_values": singular.tolist(),
                        "condition_number": float(singular[0] / singular[-1]),
                        "iterations": int(solution.nfev), "success": bool(solution.success)}


def metrics(errors, positive):
    e = np.asarray(errors, float)
    return {"count": int(len(e)), "mean_px": float(e.mean()),
            "median_px": float(np.median(e)), "rmse_px": float(np.sqrt(np.mean(e * e))),
            "max_px": float(e.max()), "within2_count": int(np.sum(e <= THRESHOLD_PX)),
            "within2_pct": float(100 * np.mean(e <= THRESHOLD_PX)),
            "positive_depth_count": int(np.sum(positive))}


def read_pose_rows(path, distance):
    rows = [r for r in csv.DictReader(open(path)) if int(r["distance_ft"]) == distance]
    if len(rows) != EXPECTED_COUNT or sorted(int(r["index"]) for r in rows) != list(range(EXPECTED_COUNT)):
        raise ValueError(f"{distance}-ft frozen observation must contain exactly indices 0..17")
    rows.sort(key=lambda r: int(r["index"]))
    rgb = np.array([[float(r["rgb_u"]), float(r["rgb_v"])] for r in rows])
    lidar = np.array([[float(r["lidar_x_L10"]), float(r["lidar_y_L10"]), float(r["lidar_z_L10"])] for r in rows])
    sigma = np.array([float(r["training_sigma_px"]) for r in rows])
    return rgb, lidar, sigma


def overlay(image_path, rgb, uv, errors, distance, path):
    image = cv2.imread(str(image_path))
    if image is None or image.shape[:2] != (480, 640):
        raise ValueError(f"frozen {distance}-ft image must be native 640x480: {image_path}")
    for i, (a, b, err) in enumerate(zip(rgb, uv, errors)):
        aa = tuple(np.rint(a).astype(int)); bb = tuple(np.rint(b).astype(int))
        color = (0, 180, 0) if err <= THRESHOLD_PX else (0, 0, 220)
        cv2.circle(image, aa, 5, (255, 255, 0), 1)
        cv2.circle(image, bb, 4, color, -1)
        cv2.line(image, aa, bb, color, 1)
        cv2.putText(image, str(i), (bb[0] + 4, bb[1] - 4), 0, .35, color, 1)
    cv2.rectangle(image, (0, 0), (639, 45), (0, 0, 0), -1)
    cv2.putText(image, f"{distance}ft STRICT LOPO | fixed index | no rematching", (8, 18), 0, .45, (255, 255, 255), 1)
    cv2.putText(image, f"{int(np.sum(errors <= 2))}/18 <=2px | RMSE={np.sqrt(np.mean(errors**2)):.3f}px", (8, 38), 0, .45, (255, 255, 255), 1)
    cv2.imwrite(str(path), image)


def validate_manifest(path):
    p = Path(path).expanduser().resolve()
    data = json.loads(p.read_text())
    required = ("distance_ft", "preflight_json", "observations_csv", "rgb_image",
                "canonical_lidar_frame", "frozen_before_projection",
                "physical_identity_frozen", "source_hashes")
    missing = [k for k in required if k not in data]
    if missing:
        raise ValueError(f"replacement freeze manifest missing: {missing}")
    if int(data["distance_ft"]) != 5 or data["canonical_lidar_frame"] != "L10":
        raise ValueError("replacement must be 5 ft and independently normalized to canonical L10")
    if not data["frozen_before_projection"] or not data["physical_identity_frozen"]:
        raise ValueError("replacement identity and observations were not frozen before projection")
    preflight = Path(data["preflight_json"]).expanduser().resolve()
    obs = Path(data["observations_csv"]).expanduser().resolve()
    image = Path(data["rgb_image"]).expanduser().resolve()
    for artifact in (preflight, obs, image):
        if not artifact.is_file():
            raise FileNotFoundError(artifact)
    pf = json.loads(preflight.read_text())
    if pf.get("overall_status") != "PASS" or not pf.get("freeze_authorized"):
        raise ValueError("replacement preflight is not all-PASS/freeze-authorized")
    expected = data["source_hashes"]
    actual = {"preflight_json": sha256(preflight), "observations_csv": sha256(obs), "rgb_image": sha256(image)}
    if any(expected.get(k) != v for k, v in actual.items()):
        raise ValueError(f"replacement freeze hash mismatch: expected {expected}, actual {actual}")
    return p, data, obs, image


def run_once(manifest_path, out, reference_observations, reference_audit, reference_image_dir):
    out.mkdir(parents=True, exist_ok=False); images = out / "images"; images.mkdir()
    manifest_file, manifest, replacement_obs, replacement_image = validate_manifest(manifest_path)
    audit = json.load(open(reference_audit))["records"]
    reference = next(r for r in audit if int(r["distance_ft"]) == 10)
    K, D = np.asarray(reference["K"], float), np.asarray(reference["D"], float)
    poses = []
    for distance in (5, 10, 15):
        source = replacement_obs if distance == 5 else reference_observations
        rgb, lidar, sigma = read_pose_rows(source, distance)
        image = replacement_image if distance == 5 else reference_image_dir / f"{distance}ft.png"
        poses.append({"distance_ft": distance, "rgb": rgb, "lidar": lidar,
                      "sigma_px": sigma, "image": image})
    folds, records = [], []
    for held in poses:
        training = [p for p in poses if p is not held]
        params, condition = fit(training, K, D)
        uv, pc = project(params, held["lidar"], K, D)
        delta = uv - held["rgb"]; errors = np.linalg.norm(delta, axis=1); positive = pc[:, 2] > 0
        m = metrics(errors, positive)
        m.update({"distance_ft": held["distance_ft"],
                  "training_distances_ft": [p["distance_ft"] for p in training], **condition})
        folds.append(m)
        overlay(held["image"], held["rgb"], uv, errors, held["distance_ft"],
                images / f"{held['distance_ft']}ft_strict_lopo_overlay.png")
        for i in range(EXPECTED_COUNT):
            records.append({"distance_ft": held["distance_ft"], "training_distances_ft": "+".join(map(str, m["training_distances_ft"])),
                            "index": i, "row": i // 6, "column": i % 6,
                            "rgb_u": held["rgb"][i, 0], "rgb_v": held["rgb"][i, 1],
                            "projected_u": uv[i, 0], "projected_v": uv[i, 1],
                            "dx_px": delta[i, 0], "dy_px": delta[i, 1], "error_px": errors[i],
                            "camera_depth_m": pc[i, 2], "positive_depth": bool(positive[i])})
    aggregate = metrics([r["error_px"] for r in records], [r["positive_depth"] for r in records])
    accepted = bool(aggregate["within2_count"] == 54 and aggregate["positive_depth_count"] == 54 and
                    all(f["within2_count"] == 18 and f["positive_depth_count"] == 18 and f["rank"] == 6 for f in folds))
    with (out / "strict_lopo_correspondences.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0]); writer.writeheader(); writer.writerows(records)
    # Deployment fit is strictly conditional on complete independent acceptance.
    deployment = None; in_sample = None
    if accepted:
        params, condition = fit(poses, K, D)
        rv = params[:3]; r, _ = cv2.Rodrigues(rv); t = params[3:]
        all_errors, all_positive = [], []
        for pose in poses:
            uv, pc = project(params, pose["lidar"], K, D)
            all_errors.extend(np.linalg.norm(uv - pose["rgb"], axis=1)); all_positive.extend(pc[:, 2] > 0)
        deployment = {"direction": "P_C = R_C_L10 P_L10 + t_C_L10", "R_C_L10": r.tolist(),
                      "t_C_L10": t.tolist(), "T_C_L10": np.block([[r, t[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]).tolist(),
                      "training_distances_ft": [5, 10, 15], "conditioning": condition}
        in_sample = metrics(all_errors, all_positive)
    result = {"scope": "replacement 5 ft plus frozen 10/15 ft", "seed": 0,
              "replacement_manifest": str(manifest_file), "replacement_manifest_sha256": sha256(manifest_file),
              "frozen_10_15_observations": str(reference_observations), "frozen_10_15_sha256": sha256(reference_observations),
              "rules": {"fixed_index": True, "positive_depth": True, "inclusive_threshold_px": 2.0,
                        "held_out_excluded_from_fit_and_tuning": True},
              "folds": folds, "aggregate": aggregate, "acceptance": "ACCEPTED" if accepted else "NOT ACCEPTED",
              "deployment_transform_released": accepted, "deployment_transform": deployment,
              "deployment_in_sample_metrics": in_sample}
    (out / "results.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    # Residual summary image.
    fig, ax = plt.subplots(figsize=(12, 6))
    for distance, marker in zip((5, 10, 15), ("o", "s", "^")):
        e = [r["error_px"] for r in records if r["distance_ft"] == distance]
        ax.plot(range(18), e, marker=marker, label=f"{distance} ft")
    ax.axhline(2.0, color="red", linestyle="--", label="inclusive 2 px threshold")
    ax.set(xlabel="fixed corner index", ylabel="Euclidean error (px)", title="Strict entire-pose LOPO residuals")
    ax.grid(alpha=.3); ax.legend(); fig.tight_layout(); fig.savefig(images / "strict_lopo_residuals.png", dpi=180); plt.close(fig)
    hashes = {str(p.relative_to(out)): sha256(p) for p in sorted(out.rglob("*")) if p.is_file() and p.name != "SHA256SUMS.json"}
    (out / "SHA256SUMS.json").write_text(json.dumps(hashes, indent=2, sort_keys=True) + "\n")
    return result


def main(args):
    root = Path(args.output_dir).expanduser().resolve()
    if root.exists():
        raise SystemExit(f"output directory already exists: {root}")
    root.mkdir(parents=True)
    reference_observations = Path(args.reference_observations).expanduser().resolve()
    reference_audit = Path(args.reference_audit).expanduser().resolve()
    reference_image_dir = Path(args.reference_image_dir).expanduser().resolve()
    required = [reference_observations, reference_audit,
                reference_image_dir / "10ft.png", reference_image_dir / "15ft.png"]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"missing frozen reference artifacts: {missing}")
    run1 = run_once(args.replacement_freeze_manifest, root / "run1",
                    reference_observations, reference_audit, reference_image_dir)
    run2 = run_once(args.replacement_freeze_manifest, root / "run2",
                    reference_observations, reference_audit, reference_image_dir)
    keys = ("results.json", "strict_lopo_correspondences.csv")
    comparisons = {key: {"run1": sha256(root / "run1" / key), "run2": sha256(root / "run2" / key)} for key in keys}
    deterministic = all(v["run1"] == v["run2"] for v in comparisons.values())
    summary = {"deterministic": deterministic, "comparisons": comparisons,
               "acceptance": run1["acceptance"], "transform_released": bool(run1["deployment_transform_released"] and deterministic)}
    if not deterministic:
        summary["acceptance"] = "NOT ACCEPTED"
        summary["transform_released"] = False
    (root / "REPRODUCIBILITY.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2))
    return 0 if summary["acceptance"] == "ACCEPTED" and deterministic else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replacement-freeze-manifest", required=True)
    parser.add_argument("--reference-observations", default=str(REFERENCE_OBS))
    parser.add_argument("--reference-audit", default=str(AUDIT))
    parser.add_argument("--reference-image-dir", default=str(ROOT / "output/new_9p6_dataset_run3/selected"))
    parser.add_argument("--output-dir", required=True)
    raise SystemExit(main(parser.parse_args()))
