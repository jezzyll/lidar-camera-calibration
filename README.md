# RealSense D455–Livox Mid-360 calibration

Tools for checkerboard-based Camera–LiDAR calibration with independent acquisition preflight and strict, fixed-index, entire-pose leave-one-pose-out (LOPO) validation.

## Current status

**Calibration is not accepted and no deployment transform is released.**

The current mentor-approved scope is:

- preserve the frozen 10-ft and 15-ft observations and their hashes;
- acquire and independently qualify one replacement 5-ft recording;
- pause 22-ft expansion;
- run three-pose LOPO only after the replacement observation passes every preflight gate and its physical corner identities are frozen before projection.

Historical pose-specific held-out results from
`output/background_frame_normalization_delivery/authoritative/results.json` were:

| Held-out pose | Result within inclusive 2 px | RMSE |
|---|---:|---:|
| 10 ft | 18/18 | 1.1246 px |
| 15 ft | 18/18 | 1.5004 px |

These are historical pose-specific diagnostics from that run. They are not a three-distance acceptance result and do not authorize deployment. The previous 5-ft observation was classified as not calibration grade.

## Geometry and conventions

- physical checkerboard: 7×4 cells;
- inner-corner topology: 6×3, exactly 18 corners;
- measured cell pitch: 0.096 m;
- RGB protocol: native D455 640×480 with matching recorded `CameraInfo`;
- correspondence: row-major fixed index, `index = row * 6 + column`;
- transform: `P_C = R_C_L P_L + t_C_L`;
- validation: positive camera depth and Euclidean pixel error `<= 2.0 px`;
- acceptance: 18/18 per held-out pose and 54/54 overall.

LiDAR candidate discovery is radial and rotation invariant. Tilted-board hypotheses are fitted in the estimated board-plane frame with metric windows. Candidate discovery does not establish checker boundaries, origin, polarity, or fixed corner indices; those must be resolved independently from LiDAR-observable physical evidence.

## Install

Python 3.11 is recommended.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

The main dependencies are NumPy, SciPy, OpenCV, scikit-learn, Matplotlib, PyYAML and `rosbags`.

## Replacement 5-ft preflight

Copy and complete the sanitized templates under `config/templates/`. The board evidence must document the full 7×4 topology, at least three horizontal and three vertical pitch measurements, the 0.672×0.384 m checker extent, and evidence-photo paths. Placeholder or missing evidence never passes.

```bash
BAG_5FT="/absolute/path/to/new_replacement_5ft_bag_folder_or_db3"
OUT="output/replacement_preflight_5ft_$(date +%Y%m%d_%H%M%S)"

test -e "$BAG_5FT" || exit 1
.venv/bin/python scripts/preflight_replacement_bag.py "$BAG_5FT" \
  --distance-ft 5 \
  --measured-pitch-m 0.096 \
  --board-evidence config/templates/board_evidence.completed.json \
  --mounting-evidence config/templates/replacement_mounting_evidence.completed.json \
  --reference-mounting-evidence config/templates/frozen_mounting_reference.completed.json \
  --block-messages 8 \
  --output-dir "$OUT"
```

The command verifies all metadata-referenced DB3 segments and SQLite integrity, hashes the inputs, inspects topics and timing, decodes native images and `CameraInfo`, measures complete-grid detection, evaluates target-masked background drift, enumerates rotation-invariant LiDAR candidates, creates tilted board-plane hypotheses, and validates independent board/mounting evidence. It does not fit an extrinsic or use reprojection error.

Environmental normalization and relative mounting are different claims:

- target-masked LiDAR background registration measures LiDAR-frame/background consistency;
- it cannot prove that the camera remained fixed relative to the LiDAR;
- relative-extrinsic compatibility requires matching sensor identities and independent locked-mount/witness evidence tied to the frozen 10/15-ft configuration.

An all-PASS report authorizes a separate review/freeze step. Geometric hypotheses must not be promoted automatically to fixed indices.

## Strict three-pose LOPO

After the accepted replacement observation and its extraction rules have been frozen and hashed:

```bash
.venv/bin/python scripts/run_replacement_5ft_three_pose_lopo.py \
  --replacement-freeze-manifest /absolute/path/to/replacement_5ft_freeze_manifest.json \
  --output-dir "output/replacement_5ft_three_pose_lopo_$(date +%Y%m%d_%H%M%S)"
```

The runner executes two fresh deterministic runs and these fixed folds:

1. train 10/15 ft, hold out 5 ft;
2. train 5/15 ft, hold out 10 ft;
3. train 5/10 ft, hold out 15 ft.

It uses only training poses to estimate each fold transform. There is no nearest-neighbour matching, permutation search, post-projection flip, manual offset, or held-out tuning. Native 640×480 overlays, all 54 correspondence records, per-fold conditioning and aggregate metrics are written to the chosen output directory.

Only if every fold achieves 18/18 and the aggregate is 54/54 with positive depth does the runner fit and release a separate all-data deployment transform. Its in-sample residuals are labeled separately from LOPO validation.

## Tests

```bash
.venv/bin/pytest -q \
  tests/test_range_based_lidar_target.py \
  tests/test_board_plane_lidar_extractor.py \
  tests/test_preflight_replacement_bag.py \
  tests/test_replacement_5ft_lopo_gate.py
```

## Data policy

Raw DB3 bags, archives, virtual environments, credentials, completed private evidence records and generated outputs are intentionally excluded from Git. Commit only reusable code, documentation and sanitized blank templates.
