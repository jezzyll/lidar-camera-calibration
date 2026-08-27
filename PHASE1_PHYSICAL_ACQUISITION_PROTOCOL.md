# Phase 1 physical acquisition protocol

## Status and scope

Current Phase 1 status: **OPEN — software/documentation complete; new physical
acquisition and final independent calibration validation pending.**

The accepted evaluator is frozen as `phase1-evaluator-v1.0.0` at commit
`50eb04fb8ed732e4dd8cc01092658bdf8fa08295`. This protocol governs raw data
acquisition only. It does not authorize changing the evaluator, refitting on
validation data, or reporting a new calibration result.

The unresolved issue is **systematic reprojection bias whose root cause cannot
yet be uniquely separated into extrinsic calibration error versus
point/reference correspondence uncertainty using the current middle/far
observations.**

## Non-negotiable raw-first gate

Submit the unmodified raw calibration and validation recordings first. Before
any calibration, evaluation, or claimed result is produced, document and
freeze:

1. this acquisition protocol and its version;
2. SHA-256 checksums for every raw bag, metadata file, rig photograph, and
   target-measurement photograph;
3. every recording's session role (`calibration` or `validation`);
4. the complete bag-level calibration/validation split;
5. sensor serial numbers, rig ID, board ID, measured baseline, board geometry,
   actual distance, pose, timestamps, and synchronization mode.

Do not overwrite `data/raw`. Store the new experiment under a new immutable
session directory. Do not open validation recordings in calibration code until
the calibration transform has been frozen and hashed.

## 1. Frozen evaluator boundary

- Do not change `R`, `t`, `K`, or `D` merely to force a result above 99%.
- RGB masks remain excluded from LiDAR selection and scoring.
- LiDAR point selection, reprojection-error definition, thresholds, timestamp
  analysis, and aggregation remain unchanged.
- Estimate `R,t` only from calibration-session recordings.
- Freeze and hash `R,t` before releasing validation recordings to the
  evaluator.
- Never refit on validation recordings.

Operational limitation: the frozen CLI currently routes three hard-coded,
same-session bags. It is not yet a drop-in five-distance, separate-session
runner. Do not change it before raw submission. After protocol and raw-data
sign-off, create a separately reviewed, versioned input adapter if required.
That adapter may route declared board geometry and separate bag roots, but it
must not alter the frozen scoring functions or acceptance rule.

## 2. Rigid sensor rig

1. Rigidly mount the Intel RealSense D455 and Livox Mid-360 to one stiff plate.
2. Eliminate movable joints, flexible tape, loose clamps, and cable forces that
   can shift either sensor.
3. Assign a permanent `rig_id`; mark fasteners with witness marks and
   photograph the rig from front, rear, left, right, top, and bottom.
4. Record both sensor serial numbers and firmware/driver settings.
5. Do not loosen, remount, or bump either sensor between calibration and
   validation sessions. If movement is suspected, invalidate all later bags.

### Physical baseline measurement

Measure the vector from the declared Livox reference origin to the D455 color
optical-center reference in a documented rig coordinate frame. Record
`baseline_x_m`, `baseline_y_m`, `baseline_z_m`, Euclidean norm, measurement
tool, and estimated uncertainty. Include annotated photographs showing the two
reference points and the positive axes. A single unlabeled distance is not
sufficient.

## 3. Checkerboard construction and measurement

- Use a rigid, flat, matte checkerboard with accurately measured dimensions.
- Declare physical square columns/rows, inner-corner columns/rows, square size,
  total width/height, material, surface finish, planarity measurement, and
  uncertainty under a permanent `board_id`.
- Measure multiple squares across both axes; do not rely only on the print
  dialog or nominal paper size.
- Keep the board fully visible, sharply focused, well illuminated, and free of
  glare, occlusion, motion blur, and people crossing the target.

### Far-range sizing from the recorded D455 intrinsics

Using `fx = 386.767 px`, native 640×480 RGB, range `Z`, and physical square
size `s`, the fronto-parallel pinhole estimate is:

```text
square_pitch_px ≈ fx * s / Z
s_required ≈ desired_pitch_px * Z / fx
```

At 30 ft (`Z = 9.144 m`):

| Desired native pitch | Required square size | 12×18-square board size |
|---:|---:|---:|
| 4 px | 94.6 mm | 1.135 × 1.702 m |
| 5 px | 118.2 mm | 1.419 × 2.128 m |

A 50 mm-square, 0.60×0.90 m board is predicted to provide only about 2.11 px
per square at 30 ft and is therefore not a reliable default for complete
11×17 corner detection. Use approximately 130 mm squares
(`1.56×2.34 m` for 12×18 squares) for 22–30 ft when physically feasible; the
predicted frontal pitch at 30 ft is about 5.50 px and about 5.17 px under a
simple 20° cosine estimate. A 120 mm-square board (`1.44×2.16 m`) is the
minimum fallback and must pass the separate preflight observability gate
below. Smaller declared boards may be used at 7.25 and 14.5 ft to preserve
image margin, but every bag must identify its measured `board_id`.

Perspective tilt reduces apparent pitch. These calculations are planning
estimates, not calibration results.

## 4. Preflight capture — excluded from all results

Use separate `preflight` bags that will never enter calibration or validation.
At the worst-case 30-ft range and intended maximum tilts, verify:

- native RGB is exactly 640×480 and unwarped;
- the complete target stays inside the frame with margin;
- a complete 11×17 RGB grid is observable in several sharp frames;
- all required ROS topics and CameraInfo are present;
- LiDAR returns reach the target with adequate density;
- exposure, focus, lighting, and synchronization settings are recorded.

This is a go/no-go acquisition check only. Do not compute reprojection error or
claim calibration accuracy from preflight bags.

## 5. Required distance and pose matrix

Capture actual measured ranges near 7.25, 14.5, 22, 25, and 30 ft. Record the
actual distance in metres; the nominal label is not a substitute for
measurement.

For each distance, acquire at least:

- eight distinct calibration poses with two independent 15-second takes per
  pose; and
- eight distinct validation poses with two independent 15-second takes per
  pose, recorded in a completely separate session and never reused for
  fitting.

Each take must contain at least 15 seconds of stable RGB, CameraInfo, and LiDAR
streams after the target stops moving. Restart recording between takes. Use different combinations of
left/center/right and low/center/high positions, fronto-parallel views, positive
and negative yaw, positive and negative pitch, and modest combined tilts.
Validation poses must not duplicate calibration poses.

The minimum plan is 160 raw take bags (`5 distances × 2 roles × 8 poses × 2
takes`). Additional poses are allowed if their role is declared before
processing. The frozen nominal schedule is in
`config/phase1_pose_schedule.csv`; record measured angles because nominal
values are planning targets only.

## 6. Separate sessions and recording order

1. Freeze this protocol and prepare empty manifests.
2. Record preflight bags; pass or replace the target/setup.
3. Record all calibration-session bags and stop the recorder.
4. Record validation-session bags under different session IDs and raw paths.
5. Compute checksums and submit all raw data plus manifests.
6. Audit completeness only; do not calibrate or score yet.
7. After raw/protocol/split sign-off, estimate `R,t` using calibration bags.
8. Save and hash the frozen transform and calibration log.
9. Release validation bags to the frozen evaluator/input adapter.
10. Evaluate once without refitting and report every required metric.

## 7. Required ROS streams and settings

Every pose bag must contain:

- `/camera/camera/color/image_raw`;
- `/camera/camera/color/camera_info`;
- `/livox/lidar`.

Use hardware synchronization when available. Otherwise preserve both native
timestamps and document the synchronization mode. Record D455 resolution,
frame rate, exposure/gain/auto-exposure state, Livox mode/rate, host clock, ROS
distribution, driver versions, and any dropped-message warnings. Do not resize,
warp, deblur, super-resolve, or regenerate RGB images.

## 8. Required immutable directory structure

```text
data/new_capture/<experiment_id>/
  protocol/
    phase1_new_capture_protocol.json
  manifests/
    rig_measurement.csv
    board_inventory.csv
    capture_sessions.csv
    split_assignment.csv
  raw/
    preflight/<distance>/<capture_id>/
    calibration/<distance>/<capture_id>/
    validation/<distance>/<capture_id>/
  photos/
    rig/
    boards/
  checksums/
    SHA256SUMS.txt
```

Raw bags and metadata are immutable. Derived images, masks, projections,
calibration files, and reports must go outside `raw/`.

Create the non-overwriting directory scaffold with:

```bash
.venv/bin/python scripts/create_phase1_capture_scaffold.py \
  ACQ_YYYYMMDD_RIG01
```

Fill `split_assignment.csv` with every planned calibration and validation
`capture_id`, role, session ID, distance, pose, and take before processing.
Hash the completed file and copy that SHA-256 into every corresponding
`capture_sessions.csv` row. The submission validator checks exact one-to-one
agreement and rejects any split change or missing assignment.

## 9. Submission gate

The raw submission is accepted for later processing only if:

- every required manifest field is complete;
- every declared raw/photo file exists and matches its SHA-256 checksum;
- calibration and validation session IDs and paths are disjoint;
- the five distances and minimum pose counts are present in both roles;
- rig and board IDs resolve to measured inventory rows;
- no validation bag is labeled or stored as calibration;
- the frozen evaluator hashes still match;
- the protocol and split are approved before any claimed result.

Run the metadata/checksum validator only; it performs no calibration and
produces no accuracy claim:

```bash
.venv/bin/python scripts/validate_phase1_capture_submission.py \
  data/new_capture/<experiment_id> --verify-frozen
```

## 10. Final acceptance after acquisition

Phase 1 can be accepted only if the frozen transform, evaluated on completely
separate validation captures without refitting, satisfies the project-defined
criterion at every required distance:

> At least 99% of independently selected valid LiDAR returns have Euclidean
> reprojection error at or below 2 pixels.

Until that evidence exists, the status remains OPEN.
