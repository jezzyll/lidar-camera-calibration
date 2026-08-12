# Mentor handoff: Livox Mid-360 + RealSense D455 pipeline

## One-sentence status

Calibration and all classical preprocessing/data-conversion algorithms are
implemented and verified on `new1`; CenterPoint, BEVFusion, and fine-tuning are
connected through real MMDetection3D adapters but cannot produce a valid
detection result until synchronized road scenes, 3D labels, compatible model
configs/checkpoints, and an NVIDIA/CUDA environment are supplied.

## What to show now

1. `outputs/reflectivity_white_square_projection.png` — primary calibration
   validation: reflected points lie on camera-white checker squares and black
   squares remain clear.
2. `outputs/calibration_reflectivity_result.json` — intrinsic matrix,
   distortion, LiDAR-to-camera 4x4 transform, and measured quality.
3. `outputs/perception/new1/preprocessing_visualization.png` — synchronized RGB,
   ROI, RANSAC removal, statistical filtering, and DBSCAN stages.
4. `outputs/perception/new1/pipeline_report.json` — exact counts and status of
   every implemented/model stage.
5. `outputs/error_analysis/calibration_sensitivity.png` — measured response to
   roll, pitch, yaw, Tx, Ty, and Tz perturbations.

Do not present a CenterPoint/BEVFusion vehicle result from `new1`: it is an
indoor checkerboard recording and has no vehicles or ground-truth 3D boxes.

## Measured results

- RGB/LiDAR timestamp difference for the tested frame: **17.706 ms**, within
  the configured **50 ms** tolerance.
- Calibration intrinsic consistency: **0.0 maximum matrix difference** between
  the saved calibration and the recording's `CameraInfo`.
- Checker-corner reprojection RMSE: **0.403 px**.
- LiDAR checker plane RMSE: **8.84 mm**.
- High-reflectivity returns landing on camera-white squares: **100.00%** for
  threshold `intensity >= 23`, supported by **2,278 raw returns**.
- White checker-cell coverage: **65/130 = 50%**; black cells hit: **0**.
- Preprocessing test: **19,872 raw points -> 6,895 ROI -> 5,986 non-ground ->
  5,794 statistically filtered points -> 4 DBSCAN clusters**.
- Vehicle-size proposals: **0**, which is correct for this non-vehicle scene.

The 100% figure is white-return precision on the calibration recording, not
overall system/detector accuracy. Detection AP/mAP/NDS is currently **not
measurable**, not “under 10,” because the supplied data contain no labeled
vehicles and no detector checkpoint has been run.

## Algorithms implemented

1. Nearest-neighbour timestamp synchronization with an explicit maximum delta.
2. Camera intrinsic validation from D455 `CameraInfo`.
3. Checkerboard/reflectivity LiDAR-to-camera extrinsic estimation with PnP.
4. RANSAC approximately-horizontal ground-plane removal.
5. K-nearest-neighbour statistical outlier removal.
6. DBSCAN point-cloud clustering.
7. Configurable spatial ROI and vehicle-dimension proposal filtering.
8. PCA-based oriented geometric proposal boxes.
9. Float32 `[x,y,z,intensity]` and
   `[x,y,z,intensity,time_lag]` `.bin` export.
10. Undistorted synchronized RGB and 3D-to-2D projection metadata export.
11. Real MMDetection3D `LidarDet3DInferencer` adapter for CenterPoint.
12. Real MMDetection3D `MultiModalityDet3DInferencer` adapter for BEVFusion.
13. Upstream MMDetection3D trainer launcher for supervised fine-tuning.
14. Upstream MMDetection3D held-out test/evaluation launcher.
15. Deterministic extrinsic perturbation analysis with pixel drift, RMSE, and
    projected-board bounding-box IoU.

## Commands already verified

```bash
.venv/bin/python scripts/calibrate_reflectivity.py
.venv/bin/python scripts/perception_pipeline.py \
  --bag data/raw_new/new1 \
  --output-dir outputs/perception/new1
.venv/bin/python scripts/calibration_error_analysis.py
```

## What is still required for the final vehicle visualization

- Synchronized road-driving ROS bags containing actual vehicles.
- Oriented 3D labels for cars/pedestrians/cyclists and train/validation/test
  splits.
- A CenterPoint configuration/checkpoint adapted to Livox coordinates and the
  five-field point format.
- A **single-camera** BEVFusion configuration adapted to the D455. A stock
  nuScenes BEVFusion configuration assumes its own dataset and multiple cameras.
- Linux with an NVIDIA GPU/CUDA-compatible PyTorch/MMCV/MMDetection3D stack.
- Evaluation with held-out class-wise AP/mAP, orientation/translation errors,
  and projected 3D-box IoU. No accuracy claim should be made on training data.

## Short explanation for the mentor

> First I calibrated the D455 intrinsics and the rigid Livox-to-camera
> transform on the checkerboard. I verified that the saved calibration exactly
> matches the recording's CameraInfo and obtained 0.403-pixel corner RMSE. Then
> I synchronized RGB and LiDAR by timestamp, cropped the LiDAR ROI, removed the
> ground by RANSAC, removed sparse outliers statistically, clustered with
> DBSCAN, formed oriented geometric proposals, and exported model-compatible
> XYZI-time data plus the undistorted RGB/projection metadata. CenterPoint and
> BEVFusion are connected to MMDetection3D, but this checkerboard recording has
> no vehicles or 3D labels, so a valid detection accuracy and final vehicle-box
> visualization require road data, checkpoints, CUDA, and held-out labels.
