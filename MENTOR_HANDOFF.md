# Mentor handoff: Phase 1 final corrections

## Decision

Phase 1 is **OPEN**. The final implementation now contains a disjoint
calibration/validation frame split, explicit RGB/LiDAR timestamp evidence, all
30 frame-level metric rows, standard deviation, three error-vector plots,
rotation-matrix validation, systematic-error diagnostics, original-grid
qualitative outputs, and a reproducible evidence package.

The current transform still fails the project-defined acceptance requirement
at 14.5 ft and 22.2 ft. A new controlled capture and larger target at 22–30 ft
also require physical data acquisition and are not falsely marked complete.

## Internal strict-split result

| Distance | Frames | Points | Mean px | Median px | RMSE px | Std px | Max px | ≤1 px | ≤2 px | ≤3 px | Result |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 7.25 ft | 10 | 1,777 | 0.217 | 0.206 | 0.237 | 0.095 | 0.572 | 100.00% | 100.00% | 100.00% | pass |
| 14.5 ft | 10 | 349 | 3.945 | 3.869 | 4.009 | 0.717 | 5.397 | 0.00% | 0.00% | 3.44% | fail |
| 22.2 ft | 10 | 170 | 2.026 | 2.090 | 2.111 | 0.590 | 3.068 | 2.35% | 45.88% | 98.24% | fail |

Correct reporting statement: 100.00% of 1,777 independently evaluated returns
at 7.25 ft have error at or below 2 px. This is not called 100% calibration
accuracy.

## Anti-circularity and split

- `frame_index % 4 == 0`: calibration LiDAR frames;
- ten evenly spaced `frame_index % 4 == 2` frames: validation;
- remainders 1 and 3: unused;
- validation frames never refine `R` or `t`;
- validation returns are selected by LiDAR-only ROI, plane residual, known
  physical extent, and boundary margin;
- RGB masks are separate visualization files and never participate in point
  selection or metric calculation.

The explicit lists are in
`outputs/phase1_final/calibration_validation_split.json`.

## Main diagnostic conclusion

The 14.5 ft and 22.2 ft errors are systematic. Error-vector directional
consistency is 0.990 at each failing distance. Intrinsics/distortion are
identical across bags, `R` is a valid proper rotation (`det(R)` approximately
1 and Frobenius orthogonality error `5.690e-16`), plane fits are millimetric,
and timestamp-error correlations do not support synchronization as the primary
cause.

The remaining evidence points to extrinsic/reference geometry inconsistency.
The small middle/far targets use manual original-grid boundaries, so improved
independent target observations are required before attributing all error to
`R` or `t` alone.

## Public external result

The public experiment uses the real
[ACSC dataset](https://github.com/HViktorTsoi/ACSC) and its
[official sample download](https://drive.google.com/file/d/1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K/view).
The held-out result is 25.01% of 59,764 returns within 2 px: fail. Public and
internal results are separate and are never pooled.

## Files to share

Send these two files together:

1. `deliverables/Phase1_Final_Corrections_Evidence_Report.docx` — the full
   technical report with all required tables and 12 measured-output figures
   embedded.
2. `deliverables/Phase1_Final_Corrections_Evidence_Package.zip` — the report,
   derived dataset outputs, 30-frame CSV, timestamps, calibration files,
   diagnostic/vector evidence, original RGB copies, masks, projections,
   provenance, documentation, and reproducible code.

Raw internal ROS 2 bags and the large public archive are not duplicated in the
ZIP. Their paths and official links are recorded in
`outputs/phase1_final/data_provenance.json`.

## Remaining physical work

- record new fixed-sensor, rigid-board, no-motion calibration and validation
  sessions at all three distances;
- keep the calibration and validation sessions separate before fitting;
- capture a sharp rigid checkerboard of approximately 0.60×0.90 m or larger at
  22–30 ft;
- freeze the transform and rerun the same evaluator without validation refit;
- accept Phase 1 only if every distance independently reaches at least 99%
  within 2 px.

Full definitions, parameters, evidence, and commands are in
`PHASE1_FINAL_CORRECTIONS.md`.
