# Phase 1 calibration: final corrections and required experiments

## Final status

Phase 1 remains **OPEN**. The strict internal validation passes at 7.25 ft and
fails at 14.5 ft and 22.2 ft. The required new controlled static capture and
larger-board 22–30 ft capture are also open because those raw recordings were
not supplied. Existing bags are not relabeled as new experiments.

Acceptance is project/mentor-defined:

> At every required internal distance, at least 99% of independently selected
> valid LiDAR returns must have Euclidean reprojection error at or below 2 px.

No value was changed and the transform was not refitted after validation to
force this requirement.

## Strict calibration/validation split

The final experiment freezes the transform before scoring validation frames:

- calibration LiDAR frames: `frame_index % 4 == 0`;
- validation LiDAR frames: ten evenly spaced frames from
  `frame_index % 4 == 2`;
- unused LiDAR frames: `frame_index % 4 in {1,3}`;
- calibration RGB observation: RGB frame 0 from each internal bag;
- validation RGB observation: nearest timestamp to each validation LiDAR
  frame;
- validation frames never refine `R` or `t`.

| Distance | Total LiDAR frames | Calibration frames | Validation LiDAR indices |
|---:|---:|---:|:---|
| 7.25 ft | 991 | 248 | 2, 110, 218, 330, 438, 550, 658, 770, 878, 990 |
| 14.5 ft | 884 | 221 | 2, 98, 194, 294, 390, 490, 586, 686, 782, 882 |
| 22.2 ft | 1,091 | 273 | 2, 122, 242, 362, 482, 606, 726, 846, 966, 1,090 |

This is a genuinely disjoint frame-level split within one supplied capture
session per distance. It is not a substitute for the requested new, separate
calibration and validation capture sessions.

## Independent LiDAR return selection

For each validation frame, points are selected before any RGB reference is
consulted:

1. Apply the predeclared metric LiDAR ROI around the target.
2. Fit `x = a*y + b*z + c` using that frame's LiDAR XYZ values.
3. Keep returns with absolute plane residual at or below 0.015 m.
4. Keep returns inside the known 0.30 × 0.45 m board extent, excluding a
   0.00375 m boundary margin.
5. Freeze the selected returns.
6. Only then map each physical board coordinate to its RGB reference pixel and
   project the measured return with the frozen `K`, `D`, `R`, and `t`.

The RGB checkerboard mask is never used to select, reject, or score LiDAR
returns. It is a separate visualization/annotation file.

## Strict alignment definition

For projected pixel `q_i=(u_i,v_i)` and independently calculated checkerboard
reference pixel `q_i*=(u_i*,v_i*)` for the same physical board coordinate:

```text
e_i = ||q_i - q_i*||_2
    = sqrt((u_i-u_i*)^2 + (v_i-v_i*)^2)
```

- strict raster alignment:
  `round(u_i)=round(u_i*)` and `round(v_i)=round(v_i*)`;
- continuous exact match: `e_i <= 1e-9 px` numerical tolerance;
- acceptance match: `e_i <= 2 px`.

The practical decision uses the 2 px percentage only.

## Internal aggregate result

| Distance | Frames | Points | Mean px | Median px | RMSE px | Std px | Max px | ≤1 px | ≤2 px | ≤3 px | Result |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 7.25 ft | 10 | 1,777 | 0.217 | 0.206 | 0.237 | 0.095 | 0.572 | 100.00% | 100.00% | 100.00% | pass |
| 14.5 ft | 10 | 349 | 3.945 | 3.869 | 4.009 | 0.717 | 5.397 | 0.00% | 0.00% | 3.44% | fail |
| 22.2 ft | 10 | 170 | 2.026 | 2.090 | 2.111 | 0.590 | 3.068 | 2.35% | 45.88% | 98.24% | fail |

These are threshold-satisfaction percentages for independently evaluated
returns. They are not described as percentage calibration accuracy.

All 30 frame rows are in
`outputs/phase1_final/frame_level_error_metrics.csv` and are embedded in the
Word report.

## Timestamp evidence

Every pair's native LiDAR and RGB timestamps, frame indices, signed difference,
and absolute difference are recorded in
`outputs/phase1_final/timestamp_pairs.csv`.

| Distance | Pairs | Mean abs ms | Median abs ms | Minimum abs ms | Maximum abs ms | Mean signed RGB−LiDAR ms | Error/delta correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.25 ft | 10 | 19.392 | 20.470 | 4.172 | 31.351 | 2.949 | −0.133 |
| 14.5 ft | 10 | 14.645 | 8.836 | 0.502 | 33.462 | 4.380 | −0.468 |
| 22.2 ft | 10 | 17.337 | 19.708 | 3.766 | 24.783 | −1.552 | −0.093 |

The selected nearest-neighbor pairs are below 34 ms. Frame error is stable as
timestamp offset varies, and the correlations do not support timestamp offset
as the primary cause. Hardware synchronization remains preferable for the new
controlled capture.

## Calibration parameters

```text
K = [[386.76702881,   0.00000000, 322.92172241],
     [  0.00000000, 386.23919678, 240.03582764],
     [  0.00000000,   0.00000000,   1.00000000]]

D = [-0.05702033, 0.06783563, -0.00031395,
      0.00094203, -0.02286913]

distortion model = plumb_bob
resolution = 640 x 480

R_camera_lidar =
[[-0.03659460, -0.99855644,  0.03931761],
 [-0.26905672, -0.02804615, -0.96271589],
 [ 0.96242886, -0.04580888, -0.26764199]]

t_camera_lidar_m = [0.12406065, -0.13618098, 0.19254702]

T_camera_lidar =
[[-0.03659460, -0.99855644,  0.03931761,  0.12406065],
 [-0.26905672, -0.02804615, -0.96271589, -0.13618098],
 [ 0.96242886, -0.04580888, -0.26764199,  0.19254702],
 [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]]
```

Convention:

```text
p_camera_color_optical = R_camera_lidar * p_livox + t_camera_lidar
camera axes: +X right, +Y down, +Z forward
translation: metres
pixel origin: top-left; u right, v down
```

## Rotation validation

```text
R^T R =
[[ 1.000000000000, -0.000000000000,  0.000000000000],
 [-0.000000000000,  1.000000000000, -0.000000000000],
 [ 0.000000000000, -0.000000000000,  1.000000000000]]
```

- Frobenius orthogonality error: `5.690e-16`;
- maximum absolute orthogonality error: `2.411e-16`;
- determinant: `0.9999999999999999`;
- proper rigid rotation: pass.

This confirms mathematical rotation validity, not calibration acceptance.

## Systematic-error diagnosis

All requested causes were checked:

- **Timestamp synchronization:** not supported as the primary cause by the
  paired-offset statistics and stable frame errors.
- **Coordinate frames and transform direction:** all selected returns have
  positive camera depth, increasing from approximately 1.80 m to 6.34 m.
- **Rotation representation:** valid proper rotation, as shown above.
- **Translation:** finite, norm approximately 0.266 m; its physical accuracy
  still needs an independent sensor-baseline measurement.
- **Intrinsics, distortion, and resolution:** every internal bag contains the
  same 640×480 plumb-bob CameraInfo; maximum `K` and `D` differences are zero.
- **Checkerboard dimensions:** 12×18 physical squares at 25 mm give the
  configured 0.30×0.45 m extent.
- **Board-plane estimation:** validation plane RMSE is approximately 3.9–4.0
  mm on average and 5.46 mm maximum.
- **Correspondence selection:** LiDAR-only ROI, plane, physical extent, and
  boundary margin; no RGB mask.
- **Optimization:** calibration-only multi-range fitting has 0.526 px corner
  RMSE at 7.25 ft, 4.329 px at 14.5 ft, and 2.240 px at 22.2 ft. It cannot fit
  every observed pose below 2 px.
- **Temporal motion:** sampled RGB tracking translation is 0 px and board-plane
  center variation is only a few millimetres, but this is not a newly recorded
  controlled session.
- **Reference pixels:** near range uses detected subpixel checker corners;
  middle/far use manual original-grid outer boundaries on targets only tens of
  pixels wide. One-pixel annotation uncertainty is therefore material.
- **Error direction:** vector consistency is about 0.99 at both failing
  distances, which supports systematic geometry/reference bias instead of
  random point noise.

| Distance | Mean vector `(du,dv)` px | Vector magnitude px | Direction consistency | Dominant image angle |
|---:|:---:|---:|---:|---:|
| 7.25 ft | (0.04, 0.19) | 0.190 | 0.876 | 78.0° |
| 14.5 ft | (−1.33, −3.67) | 3.906 | 0.990 | −109.8° |
| 22.2 ft | (0.67, 1.89) | 2.006 | 0.990 | 70.4° |

The three vector plots are in `outputs/phase1_final/error_vectors/` and are
embedded in the Word report.

## Controlled and larger-board experiments

### New controlled static validation: open

The supplied bags contain only one mostly static capture session per distance.
A legitimate new experiment requires:

1. rigidly lock the D455 and Livox mount and measure the physical baseline;
2. rigidly mount a flat board and prevent sensor/target movement and people
   crossing the view;
3. record separate calibration and validation sessions at all three distances;
4. use hardware synchronization if available, otherwise preserve both native
   timestamps;
5. use several board positions/tilts for calibration and different held-out
   poses for validation;
6. freeze `R,t` before opening validation bags and run the validator without
   refitting.

### Larger target at 22–30 ft: open

After the diagnostics above, capture a rigid 0.60×0.90 m checkerboard or larger
while keeping it fully visible. The print must be sharp/flat with known square
dimensions. The same independent acceptance test applies.

## Separate public ACSC result

The external dataset is the non-synthetic
[ACSC camera–LiDAR calibration dataset](https://github.com/HViktorTsoi/ACSC).
Its [official sample download](https://drive.google.com/file/d/1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K/view)
contains real RGB images and measured point-cloud arrays.

The seven-frame held-out result remains separate: 59,764 returns, 16.387 px
mean, 10.367 px median, 24.214 px RMSE, 17.826 px standard deviation, 53.526 px
maximum, 11.82% within 1 px, 25.01% within 2 px, and 30.70% within 3 px: fail.
This public failure is retained rather than hidden or combined with the
internal result.

## Reproduce and inspect outputs

```bash
.venv/bin/python scripts/phase1_final_experiments.py
.venv/bin/python scripts/build_phase1_final_report.py
.venv/bin/python scripts/package_phase1_final.py
```

Primary outputs:

- `outputs/phase1_final/phase1_final_report.json`;
- `outputs/phase1_final/frozen_calibration.json` and `.yaml`;
- `outputs/phase1_final/calibration_validation_split.json`;
- `outputs/phase1_final/frame_level_error_metrics.csv`;
- `outputs/phase1_final/internal_multiframe_metrics.csv`;
- `outputs/phase1_final/timestamp_pairs.csv` and `timestamp_summary.csv`;
- `outputs/phase1_final/diagnostic_summary.csv`;
- `outputs/phase1_final/data_provenance.json`;
- `outputs/phase1_final/error_vectors/`;
- `outputs/phase1_final/qualitative/`;
- `outputs/phase1_final/public_external/qualitative/`;
- `outputs/phase1_final/required_new_capture_protocol.md`;
- `deliverables/Phase1_Final_Corrections_Evidence_Report.docx`;
- `deliverables/Phase1_Final_Corrections_Evidence_Package.zip`.

The evidence package includes all derived dataset outputs, original RGB copies,
separate masks, projection figures, tables, JSON/YAML, documentation, and code.
Raw internal bags and the large public archive are not duplicated; their local
paths and official public links are recorded in `data_provenance.json`.
