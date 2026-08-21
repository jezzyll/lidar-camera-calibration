# Phase 1 calibration: required corrections and validation

## Outcome

The current calibration is visually promising but **does not yet satisfy** the
mentor-defined acceptance criterion at all internal distances. It passes at
7.25 ft and fails at 14.5 ft and 22.2 ft.

The corrected evaluation uses real images and measured LiDAR returns. It does
not generate, resize, warp, deblur, or super-resolve RGB data, and it never uses
an RGB checkerboard mask to choose the LiDAR returns that are scored.

## Acceptance rule and exact definitions

The project/mentor-defined rule is:

> At every internal distance, at least 99% of independently selected valid
> LiDAR returns must have Euclidean reprojection error ≤ 2 pixels.

This threshold is a project acceptance criterion, not a claimed universal
standard or a value taken from a particular paper.

For projected pixel \(q_i=(u_i,v_i)\) and checkerboard reference pixel
\(q_i^*=(u_i^*,v_i^*)\) for the same physical board coordinate:

```text
e_i = ||q_i - q_i*||_2
    = sqrt((u_i-u_i*)^2 + (v_i-v_i*)^2)
```

The report defines:

- strict raster alignment: `round(u_i)=round(u_i*)` and
  `round(v_i)=round(v_i*)`;
- continuous exact match: `e_i ≤ 1e-9 px` numerical tolerance;
- threshold matches: `e_i ≤ 1 px`, `e_i ≤ 2 px`, and `e_i ≤ 3 px`.

The practical project decision uses only the 2-pixel percentage. "Strict" and
continuous exact are diagnostic metrics, not alternative pass criteria.

## How evaluation LiDAR returns are selected

### Internal Livox + D455

Point selection happens entirely in the LiDAR coordinate frame before the RGB
checkerboard reference is consulted:

1. Read each raw `/livox/lidar` frame.
2. Apply a fixed metric 3D ROI around the known physical board location.
3. Fit the vertical board plane robustly as `x = a*y + b*z + c` using only
   LiDAR XYZ coordinates accumulated from the recording.
4. For the evaluated frame, keep returns whose absolute plane residual is at
   most 0.015 m.
5. Keep returns inside the known 0.30×0.45 m board extent, excluding a
   0.00375 m margin at its boundary.
6. Freeze the resulting evaluation set.
7. Map each frozen return's physical plane coordinate to the independently
   observed camera checkerboard geometry and compute its pixel error.

The RGB mask is never queried in steps 1–6 or during metric calculation. Masks
are separate files for visualization/annotation only.

Ten eligible LiDAR frames are sampled evenly across each bag, and every one is
matched to the nearest original RGB timestamp. The largest selected sync offset
in the current run is below 37 ms.

### Public ACSC

The public selection uses the dataset's LiDAR point-cloud ROI followed by
LiDAR-only height, robust plane, known physical board size, and reflectivity
processing. Fourteen frames estimate one transform; seven distinct frames
(`2, 5, 8, 11, 14, 17, 20`) are held out. RGB masks do not select points.

## Internal multi-frame result

| Distance | Frames | Evaluated LiDAR points | Mean px | Median px | RMSE px | Maximum px | ≤1 px | ≤2 px | ≤3 px | Strict raster | Result |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 7.25 ft | 10 | 1,786 | 0.217 | 0.205 | 0.237 | 0.752 | 100.00% | 100.00% | 100.00% | 75.42% | pass |
| 14.5 ft | 10 | 345 | 3.928 | 3.846 | 3.991 | 5.365 | 0.00% | 0.00% | 4.06% | 0.00% | fail |
| 22.2 ft | 10 | 176 | 2.113 | 2.154 | 2.192 | 3.064 | 2.27% | 43.18% | 97.16% | 1.14% | fail |

Continuous exact match is 0.00% at all three distances, as expected for
floating-point measurements. No result is described as "100% calibration
accuracy." For example, the valid statement at 7.25 ft is: **100.00% of 1,786
evaluated returns have reprojection error ≤ 2 pixels.**

## Separate public held-out result

Dataset: [ACSC real camera–LiDAR checkerboard
dataset](https://github.com/HViktorTsoi/ACSC).

Official sample:
[Google Drive download](https://drive.google.com/file/d/1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K/view).

The sample is non-synthetic: it contains 21 real 2048×1536 RGB PNG images and
21 paired measured N×4 XYZ-reflectance point-cloud arrays, plus calibration and
LiDAR ROI files.

| Split | Frames | Evaluated LiDAR points | Mean px | Median px | RMSE px | Maximum px | ≤1 px | ≤2 px | ≤3 px | Result |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| Held-out test | 7 | 59,764 | 16.387 | 10.367 | 24.214 | 53.526 | 11.82% | 25.01% | 30.70% | fail |

This result is intentionally not pooled with the internal result. The current
simple ACSC LiDAR grid-corner recovery is weak across several poses, so the
external implementation requires improvement. The failure remains visible in
the report rather than being replaced by a board-containment percentage.

## Internal calibration parameters

Camera intrinsic matrix:

```text
K = [[386.76702881,   0.00000000, 322.92172241],
     [  0.00000000, 386.23919678, 240.03582764],
     [  0.00000000,   0.00000000,   1.00000000]]
```

Plumb-bob distortion coefficients:

```text
D = [-0.05702033, 0.06783563, -0.00031395, 0.00094203, -0.02286913]
```

Recorded RGB resolution: `640×480`.

LiDAR-to-camera rotation and translation:

```text
R = [[-0.03659380, -0.99855630,  0.03932181],
     [-0.26905772, -0.02805037, -0.96271549],
     [ 0.96242861, -0.04580926, -0.26764282]]

t = [0.12405766, -0.13617702, 0.19254424] m
```

Complete transform:

```text
T_camera_lidar =
[[-0.03659380, -0.99855630,  0.03932181,  0.12405766],
 [-0.26905772, -0.02805037, -0.96271549, -0.13617702],
 [ 0.96242861, -0.04580926, -0.26764282,  0.19254424],
 [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]]
```

Coordinate convention:

```text
p_camera_color_optical = R_camera_lidar * p_livox + t_camera_lidar
camera optical axes: +X right, +Y down, +Z forward
translation unit: metres
pixel origin: top-left; u right, v down
```

## Original-image and qualitative evidence

The validation never changes the RGB grid used for measurement. Each example
stores the original RGB pixel array losslessly as PNG and records its SHA-256
pixel hash. The mask is a different PNG and is explicitly marked as unused for
point selection and scoring.

Nine projection examples are generated:

- public ACSC frames `000000`, `000010`, and held-out `000020`;
- internal synchronized frames 2 and 7 at 7.25 ft;
- internal synchronized frames 2 and 7 at 14.5 ft;
- internal synchronized frames 2 and 7 at 22.2 ft.

For each prefix under `outputs/phase1_validation/qualitative/`:

- `*_rgb.png` is the original-pixel RGB copy;
- `*_mask.png` is the separate visualization mask;
- `*_projection.png` shows the original scene, checkerboard reference,
  projected measured LiDAR points, reference pixels/error lines, and an error
  color bar.

The JSON lists every exact filename and pixel hash.

## Reproducible commands and outputs

After extracting the raw datasets as described in `README.md`:

```bash
.venv/bin/python scripts/calibrate.py
.venv/bin/python scripts/phase1_required_validation.py
```

Generated outputs:

- `outputs/calibration_result.json` — transform used for internal validation;
- `outputs/phase1_validation/phase1_validation_report.json` — canonical full
  report, public split, per-frame timestamps, per-frame metrics, selection
  audit, parameters, integrity declarations, and limitations;
- `outputs/phase1_validation/internal_multiframe_metrics.csv` — requested
  aggregate table;
- `outputs/phase1_validation/internal_calibration_parameters.yaml` — complete
  calibration configuration;
- `outputs/phase1_validation/qualitative/` — nine evidence sets.

## Interpretation and required next work

The 14.5 ft error stays near 4 px over all ten synchronized frames, while the
22.2 ft error stays near 2–3 px. That repeatability indicates a systematic
geometry/reference problem rather than a single blurred RGB frame. The
existing multi-range corner fit itself has about 2.83 px joint corner RMSE, so
it cannot defensibly support a universal ≤2 px claim.

The current bags provide one mostly static pose per distance, and all three
poses participated in the transform refinement. A defensible completion needs
new synchronized captures with a larger sharp target at several positions and
tilts, especially at 22–30 ft. Calibration and held-out poses must be separated,
and the frozen transform must then pass this same script without refitting on
the held-out captures.
