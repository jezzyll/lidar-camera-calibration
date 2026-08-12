# RealSense D455 ↔ Livox Mid-360 checkerboard calibration

This repository has a completed **calibration milestone**:

1. obtain the RGB camera intrinsic model;
2. estimate the rigid LiDAR-to-camera extrinsic transform with the checkerboard;
3. validate it by projecting LiDAR returns onto the checkerboard in RGB frames.

The requested downstream preprocessing flow is now implemented separately in
`scripts/perception_pipeline.py`; see `PIPELINE_IMPLEMENTATION.md` and the
concise `MENTOR_HANDOFF.md`. Real
CenterPoint/BEVFusion execution is deliberately gated on compatible pretrained
artifacts, CUDA, road-scene data, and 3D labels. The calibration recordings do
not contain evidence from which vehicle-detection accuracy can be computed.

## Recommended mentor-facing result (new high-resolution bag)

`new1-002.zip` contains a closer 1280×720 recording. It has a different camera
intrinsic matrix from the earlier 640×480 bags, so it is processed as a separate
calibration session by `scripts/calibrate_reflectivity.py`.

The improved pipeline detects the 12×19 grid independently in the RGB image and
in a metric LiDAR reflectivity raster. It then solves the direct LiDAR-to-camera
transform from those corner correspondences. Unlike the original dense depth
overlay, the final figure displays only high-reflectivity returns, leaving the
absorptive black squares clear.

With the documented default intensity threshold of 23:

- checker-corner reprojection RMSE: approximately **0.40 px**;
- LiDAR reflectivity vs camera brightness correlation: approximately **0.78**;
- **100.00%** of retained high-reflectivity returns land on camera-white pixels;
- 2,278 raw high-reflectivity returns support that percentage.
- retained returns touch **65 of 130 white cells** and **0 black cells**.

The 100.00% value is specifically **white-square point precision after thresholding**.
It is not overall calibration accuracy, and it was measured on the calibration
recording rather than an independent test capture.

Run after extracting the ZIP to `data/raw_new/new1/`:

```bash
.venv/bin/python scripts/calibrate_reflectivity.py
```

Recommended files to show the mentor:

- `outputs/reflectivity_white_square_projection.png` — main close-up;
- `outputs/reflectivity_full_scene.png` — full-scene proof;
- `outputs/lidar_reflectivity_checkerboard.png` — what LiDAR alone observes;
- `outputs/calibration_reflectivity_result.json` — transform and honest metrics.

## What the requested output means

The supplied example image is a calibration validation overlay. For every Livox point
`p_livox`, the program computes:

```text
p_camera = R_camera_lidar * p_livox + t_camera_lidar
[u, v] = project(K, distortion, p_camera)
```

When calibration is correct, returns from the physical checkerboard land on that same
checkerboard in the camera image. “Points on black should be clear” means inspecting
this pixel alignment. A dense colored rectangle by itself is not proof of 99.99% accuracy.

## Dataset audit

The three archives are ROS 2 sqlite bags:

| Recording | Duration | RGB | LiDAR | Board range found in LiDAR |
|---|---:|---:|---:|---:|
| dataset02 7.25feet | 99.1 s | 1,486 | 991 | 1.695 m |
| dataset01 14.5feet | 88.4 s | 1,326 | 884 | 4.285 m |
| dataset03 22.2feet | 110.8 s | 1,658 | 1,091 | 6.335 m |

Topics used are `/camera/camera/color/image_raw`,
`/camera/camera/color/camera_info`, and `/livox/lidar`.

Important limitation: the board and sensors do not move within a recording. Repeating
one pose for many frames does not create the geometrically diverse observations needed
to re-estimate camera intrinsics. The intrinsic matrix and distortion coefficients in
this project are therefore extracted from the D455 `CameraInfo` stream (the correct
device calibration) and validated, rather than falsely reported as a new multi-view
checkerboard solve.

## Result

The transform direction is:

```text
p_camera_color_optical = R * p_livox + t
```

```text
R = [-0.05337164  -0.99786028   0.03776672]
    [-0.28107325  -0.02127949  -0.95945037]
    [ 0.95820107  -0.06182265  -0.27933612]

t = [ 0.15050517, -0.12079831, 0.21021229 ] metres
```

Measured fit diagnostics:

- checkerboard PnP reprojection RMSE: **0.163 px** on the near calibration image;
- LiDAR checkerboard plane RMSE: **3.95 mm** after robust fitting;
- projected-vs-camera board boundary RMSE: approximately **0.66 px** on the near pose;
- the independent middle and far recordings visually land on their boards.

These are meaningful calibration metrics. Do not claim “99.99% accuracy”; calibration
quality should be reported in pixels, millimetres, and independent held-out overlays.

The machine-readable result is `outputs/calibration_result.json`; a copy suitable for
ROS/config use is `config/calibration.yaml`.

## Run from scratch

Extract the three ZIP files into `data/raw/` so the paths are:

```text
data/raw/dataset01 14.5feet/
data/raw/dataset02 7.25feet/
data/raw/dataset03 22.2feet/
```

Then run:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python scripts/calibrate.py
.venv/bin/python scripts/validate.py
```

Outputs:

- `outputs/calibration_result.json` — intrinsics, distortion, transform, fit quality;
- `outputs/calibration_validation.png` — the mentor-facing three-distance overlay;
- `outputs/calibration_closeup_validation.png` — close-up with both board bounds;
- `outputs/validation_metrics.json` — projected point/depth/bounds summary.

## What is needed for a stronger second capture

For a genuine checkerboard-only intrinsic calibration, record at least 15–25 sharp
camera views with the board filling different parts of the image, including strong tilt
and distance changes. For stronger extrinsics, record 6–10 simultaneous camera/LiDAR
board poses (different positions and tilts), keep the sensor rig rigid, measure the exact
square size, and avoid reflective-floor returns. Do not proceed to models until these
held-out projection overlays pass.
