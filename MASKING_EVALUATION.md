# Legacy checkerboard masking visualization

This document is retained only to explain the older masking outputs produced by
`scripts/checkerboard_mask_evaluation.py`.

Those outputs use real source RGB frames and measured LiDAR returns. They do not
resize, warp, deblur, super-resolve, regenerate, or synthesize the input data.
The masks are saved separately in the original camera pixel grid.

However, the old percentage asks whether LiDAR returns already cropped to the
physical board land inside a camera board polygon, optionally dilated by two
pixels. That is a **board-containment visualization metric**, not a point-wise
reprojection error. It therefore must not be used as the mentor's 99%
calibration acceptance result.

The authoritative non-circular evaluation, public dataset link, equations,
per-distance metrics, parameters, and evidence are in `PHASE1_VALIDATION.md`
and `outputs/phase1_validation/phase1_validation_report.json`. Reproduce them
with:

```bash
.venv/bin/python scripts/phase1_required_validation.py
```

The public source remains the non-synthetic [ACSC sample
dataset](https://github.com/HViktorTsoi/ACSC), available from its
[official sample download](https://drive.google.com/file/d/1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K/view).
