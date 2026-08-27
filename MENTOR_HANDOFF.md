# Mentor handoff: Phase 1 final corrections

## Decision

Phase 1 is **OPEN**. The final implementation now contains a disjoint
calibration/validation frame split, explicit RGB/LiDAR timestamp evidence, all
30 frame-level metric rows, standard deviation, three error-vector plots,
rotation-matrix validation, systematic-error diagnostics, original-grid
qualitative outputs, and a reproducible evidence package.

Current Phase 1 status: **OPEN — software/documentation complete; new physical
acquisition and final independent calibration validation pending.**

The accepted evaluator is frozen as `phase1-evaluator-v1.0.0` at commit
`50eb04fb8ed732e4dd8cc01092658bdf8fa08295`; the protected hashes are in
`FROZEN_PHASE1_EVALUATOR.json`.

The final methodological addition is also implemented: independent projection
of all 187 known physical inner corners per validation frame, camera-only RGB
corner detection, per-corner output, and full optimization mathematics.

The current transform still fails the project-defined acceptance requirement
at 14.5 ft and 22.2 ft. A new controlled capture and larger target at 22–30 ft
also require physical data acquisition and are not falsely marked complete.

## Internal strict-split result

| Distance | Frames | Points | Mean px | Median px | RMSE px | Std px | Max px | ≤1 px | ≤2 px | ≤3 px | Result |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 7.25 ft | 10 | 1,777 | 0.217 | 0.206 | 0.237 | 0.095 | 0.572 | 100.00% | 100.00% | 100.00% | pass |
| 14.5 ft | 10 | 349 | 3.945 | 3.869 | 4.009 | 0.717 | 5.397 | 0.00% | 0.00% | 3.44% | fail |
| 22.2 ft | 10 | 170 | 2.026 | 2.090 | 2.111 | 0.590 | 3.068 | 2.35% | 45.88% | 98.24% | fail |

The exact ≤2 px numerators are 1,777/1,777, 0/349, and 78/170. Thus the
accepted frozen 22.2-ft run evaluates to `78/170 = 45.8823529%`. The
acceptance decision remains fail.

Correct reporting statement: 100.00% of 1,777 independently evaluated returns
at 7.25 ft have error at or below 2 px. This is not called 100% calibration
accuracy.

## Anti-circularity and split

- calibration set C: every `frame_index % 4 == 0` frame;
- validation set V: ten explicitly listed, evenly spaced
  `frame_index % 4 == 2` frames;
- unused set U: every other frame, including remainders 1 and 3 and unselected
  remainder-2 candidates;
- validation frames never refine `R` or `t`;
- validation returns are selected by LiDAR-only ROI, plane residual, known
  physical extent, and boundary margin;
- RGB masks are separate visualization files and never participate in point
  selection or metric calculation.

The report gives C and U as exact set expressions/counts and lists every V
index. The complete integer arrays for C, V, and U are in
`outputs/phase1_final/calibration_validation_split.json`.

## Main diagnostic conclusion

The 14.5 ft and 22.2 ft errors are systematic. Error-vector directional
consistency is 0.990 at each failing distance. Intrinsics/distortion are
identical across bags, `R` is a valid proper rotation (`det(R)` approximately
1 and Frobenius orthogonality error `5.690e-16`), plane fits are millimetric,
and timestamp-error correlations do not support synchronization as the primary
cause.

The unresolved issue is **systematic reprojection bias whose root cause cannot
yet be uniquely separated into extrinsic calibration error versus
point/reference correspondence uncertainty using the current middle/far
observations.**

## Independent corner result

| Distance | RGB frames | Rejected grids | Valid grids | Valid corners | Mean px | RMSE px | Max px | ≤1 px | ≤2 px | ≤3 px |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.25 ft | 10 | 1 | 2 | 374 | 0.300279 | 0.356403 | 0.856734 | 100.00% | 100.00% | 100.00% |
| 14.5 ft | 10 | 0 | 0 | 0 | N/A | N/A | N/A | N/A | N/A | N/A |
| 22.2 ft | 10 | 0 | 0 | 0 | N/A | N/A | N/A | N/A | N/A | N/A |

The original 98.04% value was not a transform failure. All 11 apparent
failures were in the final row returned for RGB frame 987. That final row was
10.38 px from its predecessor while the grid's median vertical neighbour
spacing was 5.69 px (maximum neighbour ratio 1.879), proving that the raw
detector skipped a physical row. A declared RGB-only regularity gate now
requires every neighbour spacing to be 0.50–1.50× its orientation median. It
rejects the entire irregular grid before comparison with LiDAR; it does not
filter individual corners by reprojection error and it does not alter `R` or
`t`.

At 14.5 ft the square pitch is only about 2.33×2.38 px; at 22.2 ft it is
about 1.58×1.67 px. The complete 11×17 RGB grid is not independently resolved
in any middle/far frame, so corner statistics cannot honestly decide between
extrinsic error and correspondence uncertainty from these recordings.

`corner_individual_errors.csv` contains all 5,610 attempted physical corners;
374 have independently accepted RGB coordinates/errors, all 187 corners in
the irregular raw grid are marked `rgb_grid_quality_rejected`, and unresolved
rows are explicitly marked. `corner_detection_status.csv` supplies the raw
detection flag and exact minimum/maximum spacing ratios for every frame.

## Optimization documentation

The report now provides the exact soft-L1 objective, 6 optimized variables,
fixed `K,D`, near-pose initialization, trust-region-reflective algorithm,
finite-difference Jacobian, tolerances, calibration-frame rule, and actual
convergence: 8 iterations, 12 function evaluations, 9 Jacobian evaluations,
and `ftol` termination. No validation frame is used and no post-validation
transform adjustment is performed.

## Public external result

The public experiment uses the real
[ACSC dataset](https://github.com/HViktorTsoi/ACSC) and its
[official sample download](https://drive.google.com/file/d/1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K/view).
The held-out result is 25.01% of 59,764 returns within 2 px: fail. Public and
internal results are separate and are never pooled.

## Files to share

Send these files together:

1. `deliverables/Phase1_Final_Corrections_Evidence_Report.docx` — the full
   technical report with all required tables and 15 measured-output figures
   embedded.
2. `deliverables/Phase1_Final_Corrections_Evidence_Package.zip` — the report,
   derived dataset outputs, 30-frame CSV, all 2,296 evaluated return rows,
   timestamps, calibration files, diagnostic/vector evidence, original RGB
   copies, masks, projections, provenance, documentation, and reproducible
   code.
3. `deliverables/Phase1_Frozen_Evaluator_Baseline.zip` — the immutable
   evaluator source and accepted numerical evidence after SHA-256 verification.
4. `deliverables/Phase1_Physical_Acquisition_Kit.zip` — the field protocol,
   pose schedule, manifest templates, and raw-submission integrity validator.

Raw internal ROS 2 bags and the large public archive are not duplicated in the
ZIP. Their paths and official links are recorded in
`outputs/phase1_final/data_provenance.json`.

## Remaining physical work

- rigidly mount the D455 and Mid-360, photograph witness marks, and measure the
  physical baseline vector with at least three repeated measurements;
- use a measured rigid, flat checkerboard; the recommended far-range planning
  target is 130 mm squares on a 1.56×2.34 m 12×18 board, verified by a
  disposable 30-ft preflight;
- record new calibration and validation sessions at approximately 7.25, 14.5,
  22, 25, and 30 ft, with multiple positions and orientations/tilts at every
  distance;
- keep calibration and validation session IDs, poses, paths, and recordings
  completely separate before fitting;
- submit the unmodified raw calibration and validation recordings first.
  Before any calibration, evaluation, or claimed result is produced, document
  and freeze the acquisition protocol, raw-file checksums, session roles, and
  calibration/validation split;
- freeze the transform and rerun the same evaluator without validation refit;
- accept Phase 1 only if every distance independently reaches at least 99%
  within 2 px.

The v1.0.0 CLI remains preserved for the present three bags. A separately
reviewed, versioned routing adapter may be added only after raw-data sign-off;
it may not change selection, scoring, thresholds, or the no-refit rule.

Full definitions, parameters, evidence, and commands are in
`PHASE1_FINAL_CORRECTIONS.md`; the field checklist is in
`PHASE1_PHYSICAL_ACQUISITION_PROTOCOL.md`.
