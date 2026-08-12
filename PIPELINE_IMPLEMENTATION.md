# Livox + RGB perception pipeline: implementation status

## Requested flow

```text
Livox + RGB capture
  -> nearest-timestamp synchronization
  -> checkerboard calibration
  -> RANSAC ground removal
  -> statistical outlier removal
  -> DBSCAN clustering
  -> geometric vehicle-size ROI proposals
  -> float32 XYZI .bin
  -> CenterPoint
  -> oriented 3D detection boxes
  -> BEVFusion
  -> supervised fine-tuning
  -> final visualization and held-out evaluation
```

## What is executable now

`scripts/perception_pipeline.py` implements the data and classical-geometry stages.
It reads the original ROS 2 messages, selects the nearest RGB frame for a LiDAR
scan, checks the timestamp delta, filters the point cloud, writes model-ready
float32 XYZI data, produces PCA-oriented geometric proposals, and saves an
auditable visualization/report.

It also validates that the calibration resolution and intrinsic matrix match
the selected RGB frame, undistorts that frame, and exports a five-field
`[x,y,z,intensity,time_lag]` point file plus projection metadata for the model
adapters. This prevents accidental use of the 640x480 calibration with the
1280x720 recording.

Run an indoor/calibration example:

```bash
.venv/bin/python scripts/perception_pipeline.py \
  --bag "data/raw_new/new1" \
  --output-dir outputs/perception/new1
```

Run a road-scene bag only if it contains the same topics:

```bash
.venv/bin/python scripts/perception_pipeline.py \
  --bag /path/to/road_scene_bag \
  --output-dir outputs/perception/road_scene
```

## Model stages and accuracy

CenterPoint and BEVFusion are trained neural networks, not generic names for
DBSCAN or PCA boxes. A defensible run requires:

1. Linux/NVIDIA CUDA environment compatible with PyTorch, MMCV, and MMDetection3D;
2. matching CenterPoint and BEVFusion config files and checkpoints;
3. dataset coordinate/class/range conversion;
4. camera calibration and image augmentation matrices expected by the model;
5. labeled 3D boxes for fine-tuning and a separate validation/test split;
6. evaluation with class-wise AP/mAP and, where compatible, NDS/mATE/mAOE.

The supplied hardware has one D455 RGB view. A stock nuScenes BEVFusion config
expects the nuScenes camera layout, so its image pipeline/config must be adapted
and fine-tuned for this single-camera rig; a downloaded checkpoint alone does
not make it dataset-compatible.

The current Apple M1 environment has no installed PyTorch/MMDetection3D model
stack, and the supplied checkerboard bags do not contain vehicles or ground-truth
3D annotations. Consequently, the repository reports those stages as blocked
instead of fabricating CenterPoint/BEVFusion accuracy.

After installing MMDetection3D on a compatible Linux/NVIDIA machine and adding
the selected config/checkpoint, run real LiDAR-only inference with:

```bash
python scripts/mmdet3d_adapter.py centerpoint \
  --points outputs/perception/new1/frame_centerpoint_xyzi_time.bin \
  --config /path/to/centerpoint_config.py \
  --checkpoint /path/to/centerpoint_checkpoint.pth \
  --output-dir outputs/centerpoint
```

Run camera-LiDAR inference with:

```bash
python scripts/mmdet3d_adapter.py bevfusion \
  --points outputs/perception/new1/frame_centerpoint_xyzi_time.bin \
  --image outputs/perception/new1/rgb_undistorted.png \
  --infos outputs/perception/new1/mmdet3d_info.pkl \
  --config /path/to/bevfusion_config.py \
  --checkpoint /path/to/bevfusion_checkpoint.pth \
  --output-dir outputs/bevfusion
```

The adapter uses MMDetection3D's `LidarDet3DInferencer` for CenterPoint and
`MultiModalityDet3DInferencer` for BEVFusion. A stock nuScenes checkpoint is a
pipeline smoke test, not a valid accuracy result on this Livox dataset. The
vehicle data must be labeled and the model fine-tuned/evaluated on separate
splits before reporting accuracy.

Fine-tuning is launched through the upstream trainer only after the custom
dataset config and annotations exist:

```bash
python scripts/mmdet3d_adapter.py fine-tune \
  --mmdet3d-root /path/to/mmdetection3d \
  --config /path/to/livox_finetune_config.py \
  --output-dir outputs/training
```

Evaluate the resulting checkpoint on the held-out dataset defined in the config:

```bash
python scripts/mmdet3d_adapter.py evaluate \
  --mmdet3d-root /path/to/mmdetection3d \
  --config /path/to/livox_finetune_config.py \
  --checkpoint outputs/training/best_checkpoint.pth \
  --output-dir outputs/evaluation
```

## Relationship to the supplied PDF

The PDF is an error-analysis reference, not a pretrained implementation. Its
useful requirements have been adopted:

- report reprojection RMSE and mean pixel drift for calibration;
- report projected-box IoU for fusion alignment;
- perturb roll, pitch, yaw, Tx, Ty, and Tz to study sensitivity;
- use real detector reruns where available;
- treat IoU below 0.50 as a fusion failure threshold;
- report missing classes and missing labels rather than inventing scores.

The PDF itself states that its measured detector subset contains only 10 car
instances, pedestrian/cyclist have zero measured instances, and much of the
broad degradation sweep is projection-modeled. Its figures therefore cannot be
used as a claimed target detector accuracy for this different dataset.

The comparable, dataset-specific perturbation study is executable with:

```bash
.venv/bin/python scripts/calibration_error_analysis.py
```

It perturbs roll, pitch, yaw, Tx, Ty, and Tz in positive and negative directions
and reports mean pixel drift, pixel RMSE, and projected-checkerboard box IoU. The
IoU is an alignment metric, not vehicle-detection accuracy.

## Next data required

To complete the neural stages, obtain or record synchronized road scenes with
cars/pedestrians/cyclists, retain the rigid calibration, label oriented 3D boxes,
define train/validation/test splits, and provide the intended CenterPoint and
BEVFusion checkpoints/configs. Calibration-board recordings should remain the
calibration set, not the detection-training set.
