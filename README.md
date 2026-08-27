# RealSense D455 ↔ Livox Mid-360 checkerboard calibration

## Authoritative Phase 1 status

The authoritative implementation is
`scripts/phase1_final_experiments.py`; the human-readable technical record is
`PHASE1_FINAL_CORRECTIONS.md`.

Phase 1 is **OPEN** under the project/mentor-defined rule that at every
required internal distance at least 99% of independently evaluated valid LiDAR
returns must have Euclidean reprojection error at or below 2 pixels.

Current Phase 1 status: **OPEN — software/documentation complete; new physical
acquisition and final independent calibration validation pending.**

The accepted software evaluator is frozen as `phase1-evaluator-v1.0.0` at
commit `50eb04fb8ed732e4dd8cc01092658bdf8fa08295`. Its protected source,
runtime, raw-input, and evidence hashes are recorded in
`FROZEN_PHASE1_EVALUATOR.json`.

| Distance | Frames | Points | Mean px | Median px | RMSE px | Std px | Max px | ≤1 px | ≤2 px | ≤3 px | Result |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 7.25 ft | 10 | 1,777 | 0.217 | 0.206 | 0.237 | 0.095 | 0.572 | 100.00% | 100.00% | 100.00% | pass |
| 14.5 ft | 10 | 349 | 3.945 | 3.869 | 4.009 | 0.717 | 5.397 | 0.00% | 0.00% | 3.44% | fail |
| 22.2 ft | 10 | 170 | 2.026 | 2.090 | 2.111 | 0.590 | 3.068 | 2.35% | 45.88% | 98.24% | fail |

These are threshold-satisfaction rates, not a percentage calibration-accuracy
claim. The transform is not refitted after validation to force the target.
The exact ≤2 px counts are 1,777/1,777, 0/349, and 78/170 respectively;
therefore the accepted 22.2-ft result is `78/170 = 45.8823529%`.

The unresolved issue is **systematic reprojection bias whose root cause cannot
yet be uniquely separated into extrinsic calibration error versus
point/reference correspondence uncertainty using the current middle/far
observations.**

## What the final correction adds

- disjoint calibration and validation LiDAR frame sets;
- explicit frame rules and validation indices;
- frozen `R,t` before validation;
- LiDAR-only validation-return selection without an RGB mask;
- all 30 LiDAR/RGB timestamp pairs and per-distance summary statistics;
- all 30 frame-level error rows, including standard deviation;
- every evaluated internal return with projected/reference pixels, error, and
  exact ≤1/2/3 px flags;
- three representative reprojection-error vector plots;
- a 5,610-row independent physical-corner attempt audit and three corner
  observability figures;
- an RGB-only checker-grid regularity gate that rejects complete raw detector
  grids with duplicated/skipped rows before any LiDAR error is calculated;
- complete optimization mathematics and actual convergence counters;
- `RᵀR`, determinant, and rotation-validity evidence;
- documented checks for synchronization, frame convention, translation,
  intrinsics/distortion/resolution, target dimensions, plane fit,
  correspondence, optimization, temporal motion, and reference pixels;
- separate public ACSC and internal Livox+D455 experiments;
- an explicit OPEN status for the unavailable new controlled capture and
  larger-board 22–30 ft experiment.

## Real data

Place the supplied internal ROS 2 bags at:

```text
data/raw/dataset01 14.5feet/
data/raw/dataset02 7.25feet/
data/raw/dataset03 22.2feet/
```

The external experiment uses the non-synthetic
[ACSC camera–LiDAR calibration dataset](https://github.com/HViktorTsoi/ACSC).
Its [official sample archive](https://drive.google.com/file/d/1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K/view)
contains 21 real 2048×1536 RGB frames, paired measured XYZ-reflectance point
clouds, intrinsics, distortion, and LiDAR ROIs.

Extract the sample so it contains:

```text
data/public/acsc/sample/images/
data/public/acsc/sample/pcds/
data/public/acsc/sample/ROIs/
data/public/acsc/sample/intrinsic
data/public/acsc/sample/distortion
```

Public and internal transforms, splits, metrics, and conclusions are never
pooled. The current public held-out result is also a failure: 25.01% of 59,764
returns are within 2 px.

## Reproduce

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python scripts/phase1_final_experiments.py
.venv/bin/python scripts/build_phase1_final_report.py
.venv/bin/python scripts/validate_phase1_capture_submission.py \
  --check-templates --verify-frozen
.venv/bin/python scripts/package_frozen_phase1_baseline.py
.venv/bin/python scripts/package_phase1_acquisition_kit.py
.venv/bin/python scripts/package_phase1_final.py
```

`scripts/validate.py` is a compatibility launcher for the same strict final
experiment.

## Main outputs

Numeric and audit evidence:

- `outputs/phase1_final/phase1_final_report.json` — canonical full report;
- `outputs/phase1_final/frozen_calibration.json` and `.yaml` — camera and
  transform parameters;
- `outputs/phase1_final/calibration_validation_split.json` — explicit split;
- `outputs/phase1_final/frame_level_error_metrics.csv` — 30 frame rows;
- `outputs/phase1_final/return_individual_errors.csv` — every evaluated
  internal return and exact threshold flags;
- `outputs/phase1_final/internal_multiframe_metrics.csv` — aggregate result;
- `outputs/phase1_final/timestamp_pairs.csv` and `timestamp_summary.csv`;
- `outputs/phase1_final/diagnostic_summary.csv`;
- `outputs/phase1_final/corner_metrics.csv` — aggregate independent-corner
  result;
- `outputs/phase1_final/corner_detection_status.csv` — all 30 detection
  attempts;
- `outputs/phase1_final/corner_individual_errors.csv` — 30×187 attempted
  physical corners;
- `outputs/phase1_final/data_provenance.json`;
- `outputs/phase1_final/required_new_capture_protocol.md`.

Image evidence:

- `outputs/phase1_final/error_vectors/` — three vector plots;
- `outputs/phase1_final/corner_validation/` — three corner-observability plots;
- `outputs/phase1_final/qualitative/` — six internal original-RGB/mask/
  projection sets;
- `outputs/phase1_final/public_external/qualitative/` — three public sets.

Shareable deliverables:

- `deliverables/Phase1_Final_Corrections_Evidence_Report.docx` — formal report
  with required numerical tables and measured-output figures embedded;
- `deliverables/Phase1_Final_Corrections_Evidence_Package.zip` — report,
  complete derived outputs, documentation, and reproducible code.
- `deliverables/Phase1_Frozen_Evaluator_Baseline.zip` — hash-verified frozen
  v1.0.0 evaluator source and accepted numeric evidence;
- `deliverables/Phase1_Physical_Acquisition_Kit.zip` — frozen physical-capture
  protocol, pose schedule, blank manifests, and raw-submission validator.

Raw internal bags and the large public archive are not duplicated in the ZIP.
Their paths and official external source links are in `data_provenance.json`.

## Evaluation method

The internal split is deterministic:

```text
calibration: frame_index % 4 == 0
validation:  10 evenly spaced frames where frame_index % 4 == 2
unused:      every frame not selected above (including unselected % 4 == 2)
```

For each validation frame, returns are selected only from LiDAR geometry: a
fixed metric ROI, robust plane fit, known 0.30×0.45 m physical extent, and an
interior boundary margin. The RGB mask never selects or rejects a point.

For projected pixel `(u_i,v_i)` and independent checkerboard reference
`(u_i*,v_i*)`:

```text
e_i = sqrt((u_i-u_i*)^2 + (v_i-v_i*)^2)
```

Original RGB arrays stay in the recorded coordinate grid; they are not
resized, warped, deblurred, super-resolved, or regenerated for validation.
Masks are separate visualization outputs only.

## Calibration model

No ML or DL model estimates the rigid transform. The pipeline uses measured
checkerboard geometry, D455 `CameraInfo`, robust Livox board-plane fitting,
OpenCV camera geometry, and nonlinear multi-range rigid-transform refinement.

```text
p_camera_color_optical = R_camera_lidar * p_livox + t_camera_lidar
[u,v] = project(K, D, p_camera_color_optical)
```

Optional ML experiments elsewhere in the repository do not participate in the
strict Phase 1 decision.

## Remaining physical acquisition

The supplied bags provide one mostly static session per distance. To close
Phase 1:

1. rigidly mount the D455 and Mid-360, add witness marks, and measure/document
   the three-component physical sensor baseline;
2. use a measured rigid, flat, matte checkerboard; the far-range planning
   target is approximately 130 mm squares and a 1.56×2.34 m 12×18 board,
   subject to a separate 30-ft preflight observability check;
3. record separate calibration and validation sessions at 7.25, 14.5, 22,
   25, and 30 ft, with multiple positions and orientations/tilts at every
   distance;
4. submit the unmodified raw calibration and validation recordings first.
   Before any calibration, evaluation, or claimed result is produced,
   document and freeze the acquisition protocol, raw-file checksums, session
   roles, and calibration/validation split;
5. estimate `R,t` only from calibration captures and freeze/hash the transform
   before releasing validation captures to evaluation;
6. run the same selection, scoring, and acceptance methodology without
   validation refit and pass 99% within 2 px independently at every distance.

The frozen v1.0.0 command-line runner is specific to the present three
same-session bags. After raw-data and split sign-off, a separately reviewed,
versioned input adapter may be added for the five-distance separate-session
layout; it must not change the frozen scoring functions or acceptance rule.

The full field protocol is `PHASE1_PHYSICAL_ACQUISITION_PROTOCOL.md`.

Older visualization scripts and `outputs/phase1_validation/` remain for
historical comparison but are superseded for Phase 1 acceptance by
`scripts/phase1_final_experiments.py`.
