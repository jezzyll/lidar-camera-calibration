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

## Primary LiDAR projection result

The main mentor-facing output is now
`outputs/lidar_projection_checkerboard.png`. It projects 16,730 real measured
LiDAR returns through the saved extrinsic and recorded camera model. The points
are selected from reflective checker cells using only LiDAR geometry and
reflectivity; there is no camera-white display mask.

Measured projection result:

- checker corner reprojection RMSE: **0.403 px**;
- returns landing on camera-white checker squares: **16,730/16,730 = 100.00%**;
- white checker cells reached: **130/130**;
- camera checker black cells hit: **0**;
- unique rendered LiDAR pixels: **6,576**.

The output is colored by camera depth and includes both the camera-derived and
LiDAR-projected board boundaries, matching the requested projection format.
`outputs/lidar_projection_full_scene.png` provides the uncropped scene context,
while `outputs/calibration_target_range_validation.png` provides the separate
22.2 ft range evidence.

Run the projection after the high-resolution calibration:

```bash
.venv/bin/python scripts/calibrate_reflectivity.py
.venv/bin/python scripts/project_lidar_checkerboard.py
```

The preserved checker pattern is strong visual calibration evidence, but it
does not mathematically establish “perfect” calibration. The RMSE, measured
point counts, no-mask declaration, and a held-out 22–30 ft capture remain
necessary for a defensible accuracy statement.

### Projection at all three recorded distances

`scripts/project_lidar_three_distances.py` produces the same readable projection
format at **7.25 ft, 14.5 ft, and 22.2 ft**. Because dense accumulation paints an
opaque rectangle over the distant boards, it displays three real measured
returns nearest the center of each available known white cell. It does not
create synthetic points and does not use a camera-white pixel mask.

Results with the locked ±2 px board-boundary criterion:

- 7.25 ft: **108/108 cells represented, 100.00% PASS**;
- 14.5 ft: **103/108 cells represented, 100.00% PASS**;
- 22.2 ft: **108/108 cells represented, 100.00% PASS**.

Run:

```bash
.venv/bin/python scripts/calibrate.py
.venv/bin/python scripts/project_lidar_three_distances.py
```

Outputs:

- `outputs/three_distance_lidar_projection.png` — primary three-distance close-up;
- `outputs/three_distance_lidar_projection_full_scene.png` — physical-scene proof;
- `outputs/lidar_projection_7_25ft.png`;
- `outputs/lidar_projection_14_5ft.png`;
- `outputs/lidar_projection_22_2ft.png`;
- `outputs/three_distance_lidar_projection_report.json`.

At 14.5 ft a cell is about 2.3 camera pixels; at 22.2 ft it is only about
1.6 pixels. The magnified overlays can show measured point placement and board
containment, but those recordings cannot independently prove white-vs-black
square accuracy. A higher-resolution 22–30 ft capture or a larger checkerboard
is required for that stronger claim.

The script audits every RGB frame and selects the sharpest checkerboard ROI by
variance of Laplacian: frame 1267/1486 at 7.25 ft, frame 54/1326 at 14.5 ft,
and frame 425/1658 at 22.2 ft. No ML/DL, deblurring, or super-resolution model is
used. The projection itself is classical geometry using the recorded camera
intrinsics/distortion and calibrated `R,t`. Exact LiDAR provenance and
raw→ROI→plane→display counts are saved in the JSON report.

Current point audit:

| Distance | Board-ROI returns | Plane-filtered | Displayed measured returns |
|---:|---:|---:|---:|
| 7.25 ft | 182,287 | 181,834 | 324 |
| 14.5 ft | 31,106 | 30,848 | 307 |
| 22.2 ft | 20,014 | 19,720 | 324 |

The optional MLP elsewhere in this repository is used only for LiDAR
reflectivity-cell classification in the separate high-resolution `new1`
experiment. It is not used for these three-distance projections or for the
geometric calibration.

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
- `outputs/nn_calibration/nn_white_square_projection.png` — neural-network-assisted
  full-board close-up using LiDAR-only MLP cell classification;
- `outputs/reflectivity_full_scene.png` — full-scene proof;
- `outputs/lidar_reflectivity_checkerboard.png` — what LiDAR alone observes;
- `outputs/calibration_reflectivity_result.json` — transform and honest metrics.

## Neural-network-assisted reflectivity result

The optional neural stage uses a scikit-learn `MLPClassifier` with architecture
`24 -> 16 -> 8 -> 1`. Its 24 inputs are LiDAR reflectivity statistics and
histogram bins extracted independently from each rectified checker cell. It
does not use camera pixels as input and does not replace the intrinsic or
extrinsic geometric solve.

On five-fold held-out-cell validation within `new1`, the MLP classified all 260
white/black checker cells correctly. Its full-board projection selects 16,730
measured returns, reaches all 130 white cells, and places 100.00% of those
selected returns on camera-white pixels in this recording. No camera-white
display mask is used. These figures are same-recording checker classification
and projection metrics, not independent calibration or detector accuracy.

Run:

```bash
.venv/bin/python scripts/calibrate_reflectivity_nn.py
```

Outputs:

- `outputs/nn_calibration/nn_white_square_projection.png`;
- `outputs/nn_calibration/nn_reflectivity_report.json`;
- `outputs/nn_calibration/mlp_reflectivity_classifier.pkl`.

## What the requested output means

The supplied example image is a calibration validation overlay. For every Livox point
`p_livox`, the program computes:

```text
p_camera = R_camera_lidar * p_livox + t_camera_lidar
[u, v] = project(K, distortion, p_camera)
```

When calibration is correct, returns from the physical checkerboard land on that same
checkerboard in the camera image. The validation now reports a defined percentage in
addition to RMSE: the fraction of selected physical-board returns that project inside,
or no more than 2 px outside, the annotated camera board boundary. The required
threshold is 99%. This is checkerboard alignment accuracy, not detector accuracy.

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

The final refinement includes `dataset03 22.2feet`, which passes the requested
22–30 ft calibration-range requirement. The 7.25 ft checker corner detection is used
for a stable initialization, while the outer checkerboard boundaries at all three
ranges constrain the final transform. The far board occupies only about 19×30 camera
pixels, so its outer boundary is explicitly annotated instead of reporting unreliable
internal-corner detections.

The transform direction is:

```text
p_camera_color_optical = R * p_livox + t
```

```text
R = [-0.03659380  -0.99855630   0.03932181]
    [-0.26905772  -0.02805037  -0.96271549]
    [ 0.96242861  -0.04580926  -0.26764282]

t = [ 0.12405766, -0.13617702, 0.19254424 ] metres
```

Measured fit diagnostics:

- checkerboard PnP reprojection RMSE: **0.163 px** on the near calibration image;
- near LiDAR checkerboard plane RMSE: **3.96 mm** after robust fitting;
- joint board-corner reprojection RMSE: **2.83 px** across all three poses;
- 7.25 ft board-point alignment: **100.00%** within the predeclared ±2 px tolerance;
- 14.5 ft board-point alignment: **99.29%** within ±2 px;
- 22.2 ft target-range board-point alignment: **100.00%** within ±2 px.

The percentage has an explicit numerator, denominator, and tolerance in
`outputs/validation_metrics.json`. These are in-sample alignment-fit measurements
because all three poses constrain the refinement. A new, separate 22–30 ft recording
is still required before claiming independent 99% generalization accuracy.

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
- `outputs/calibration_validation.png` — three-distance overlay with 99% pass/fail results;
- `outputs/calibration_target_range_validation.png` — mentor-facing 22.2 ft result;
- `outputs/calibration_closeup_validation.png` — compatibility copy of the target-range view;
- `outputs/validation_metrics.json` — metric definition, counts, percentages, and range checks.

## What is needed for a stronger second capture

For a genuine checkerboard-only intrinsic calibration, record at least 15–25 sharp
camera views with the board filling different parts of the image, including strong tilt
and distance changes. For stronger extrinsics, record 6–10 simultaneous camera/LiDAR
board poses at 22–30 ft (different positions and tilts), keep the sensor rig rigid,
measure the exact square size, and avoid reflective-floor returns. Reserve at least one
additional 22–30 ft capture as a held-out test and apply the same locked ±2 px / 99%
criterion without refitting.
