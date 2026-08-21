# Mentor handoff: Phase 1 calibration validation

## Decision

Phase 1 is **not yet accepted** under the project/mentor-defined requirement:

> At every internal distance, at least 99% of independently selected valid
> LiDAR returns must have Euclidean reprojection error ≤ 2 pixels.

This is a project acceptance criterion. It is not claimed to be a universal
threshold taken from a paper or standard.

## Internal multi-frame result

| Distance | Frames | Evaluated returns | Mean px | RMSE px | Median px | ≤1 px | ≤2 px | ≤3 px | Max px | Result |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 7.25 ft | 10 | 1,786 | 0.217 | 0.237 | 0.205 | 100.00% | 100.00% | 100.00% | 0.752 | pass |
| 14.5 ft | 10 | 345 | 3.928 | 3.991 | 3.846 | 0.00% | 0.00% | 4.06% | 5.365 | fail |
| 22.2 ft | 10 | 176 | 2.113 | 2.192 | 2.154 | 2.27% | 43.18% | 97.16% | 3.064 | fail |

No "100% calibration accuracy" claim is made. The percentages above are the
fractions of evaluated returns satisfying specific Euclidean pixel-error
thresholds.

## Independent point selection

The internal evaluation set is selected before any RGB reference is consulted:

1. read a single raw Livox frame;
2. apply the predeclared metric LiDAR ROI;
3. fit `x = a*y + b*z + c` robustly using LiDAR coordinates;
4. keep returns with absolute plane residual ≤ 0.015 m;
5. keep returns inside the known 0.30×0.45 m board with a 0.00375 m
   boundary margin;
6. freeze those returns;
7. only then use the camera checkerboard geometry to obtain reference pixels.

The RGB mask is saved only as a separate visualization/annotation file. It is
never used to select, reject, or score a LiDAR return.

## Mathematical definition

For the calibrated projection \((u_i,v_i)\) and independent checkerboard
reference \((u_i^*,v_i^*)\) for the same physical point:

```text
e_i = sqrt((u_i-u_i*)^2 + (v_i-v_i*)^2)
```

- strict raster match: `round(u_i)=round(u_i*)` and
  `round(v_i)=round(v_i*)`;
- continuous exact match: `e_i ≤ 1e-9 px`;
- acceptance match: `e_i ≤ 2 px`.

Mean, median, RMSE, maximum, and the percentages at 1, 2, and 3 pixels are all
reported. The JSON also includes strict-raster and continuous-exact rates.

## Public external experiment

The non-synthetic external baseline is the [ACSC real camera–LiDAR
checkerboard dataset](https://github.com/HViktorTsoi/ACSC), with the
[official sample download](https://drive.google.com/file/d/1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K/view).
Fourteen frames are used for the public extrinsic fit and seven distinct frames
are held out.

The held-out public aggregate is 59,764 evaluated returns, 16.387 px mean,
10.367 px median, 24.214 px RMSE, 53.526 px maximum, 11.82% within 1 px,
25.01% within 2 px, and 30.70% within 3 px: **fail**. This failure is kept in
the report. The current simple LiDAR grid-corner extractor is not reliable
enough across all ACSC poses and must be improved before this can be claimed as
a successful external baseline.

Public and internal experiments are never combined into one percentage.

## Calibration parameters

```text
K = [[386.76702881,   0.00000000, 322.92172241],
     [  0.00000000, 386.23919678, 240.03582764],
     [  0.00000000,   0.00000000,   1.00000000]]

D = [-0.05702033, 0.06783563, -0.00031395, 0.00094203, -0.02286913]
resolution = 640 x 480

R_camera_lidar =
[[-0.03659380, -0.99855630,  0.03932181],
 [-0.26905772, -0.02805037, -0.96271549],
 [ 0.96242861, -0.04580926, -0.26764282]]

t_camera_lidar_m = [0.12405766, -0.13617702, 0.19254424]

T_camera_lidar =
[[-0.03659380, -0.99855630,  0.03932181,  0.12405766],
 [-0.26905772, -0.02805037, -0.96271549, -0.13617702],
 [ 0.96242861, -0.04580926, -0.26764282,  0.19254424],
 [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]]
```

Convention: `p_camera_color_optical = R_camera_lidar * p_livox +
t_camera_lidar`; camera optical axes are +X right, +Y down, +Z forward; metres;
pixel origin at top-left, u right and v down.

## Evidence to show

Primary report and tables:

- `PHASE1_VALIDATION.md`;
- `outputs/phase1_validation/phase1_validation_report.json`;
- `outputs/phase1_validation/internal_multiframe_metrics.csv`;
- `outputs/phase1_validation/internal_calibration_parameters.yaml`.

Nine qualitative projection figures are in
`outputs/phase1_validation/qualitative/`: three public examples and two
synchronized examples at each internal distance. Every example includes:

- an original-resolution, losslessly stored RGB image;
- a separate mask that is not used for scoring;
- the checkerboard reference, measured projected LiDAR returns, point-wise
  reference markers/error lines, and an error color scale.

## Reproduce

```bash
.venv/bin/python scripts/calibrate.py
.venv/bin/python scripts/phase1_required_validation.py
```

The full raw-data directory layout and public download instructions are in
`README.md`. The JSON is the canonical record; it contains train/test indices,
all per-frame metrics and timestamps, RGB pixel hashes, LiDAR selection audits,
full transforms, integrity declarations, and limitations.
