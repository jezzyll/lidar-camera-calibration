#!/usr/bin/env python3
"""Strict MMDetection3D adapter for CenterPoint, BEVFusion, and fine-tuning.

This file intentionally fails when model artifacts or CUDA are unavailable.
It never reports geometric DBSCAN proposals as neural-network detections.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def existing_file(value: str) -> Path:
    path = Path(value)
    if not path.is_file():
        raise argparse.ArgumentTypeError(f"file does not exist: {path}")
    return path


def require_inference_environment(device: str):
    try:
        import torch
        import mmdet3d  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "PyTorch/MMDetection3D is not installed. Use the project's documented "
            "Linux/NVIDIA environment before running a neural model."
        ) from exc
    if device.startswith("cuda") and not torch.cuda.is_available():
        raise SystemExit("A CUDA device was requested but torch.cuda.is_available() is false")


def check_float32_records(path: Path, features: int):
    byte_count = path.stat().st_size
    record_bytes = 4 * features
    if byte_count == 0 or byte_count % record_bytes:
        raise SystemExit(
            f"{path} is not a non-empty float32 point cloud with {features} fields per point"
        )


def run_centerpoint(args):
    require_inference_environment(args.device)
    check_float32_records(args.points, args.features)
    from mmdet3d.apis import LidarDet3DInferencer

    inferencer = LidarDet3DInferencer(
        model=str(args.config), weights=str(args.checkpoint), device=args.device
    )
    result = inferencer(
        inputs={"points": str(args.points)},
        out_dir=str(args.output_dir),
        pred_score_thr=args.score_threshold,
        show=False,
        print_result=True,
    )
    print(result)


def run_bevfusion(args):
    require_inference_environment(args.device)
    check_float32_records(args.points, args.features)
    from mmdet3d.apis import MultiModalityDet3DInferencer

    inferencer = MultiModalityDet3DInferencer(
        model=str(args.config), weights=str(args.checkpoint), device=args.device
    )
    result = inferencer(
        inputs={
            "points": str(args.points),
            "img": str(args.image),
            "infos": str(args.infos),
        },
        out_dir=str(args.output_dir),
        pred_score_thr=args.score_threshold,
        show=False,
        print_result=True,
    )
    print(result)


def run_train(args):
    require_inference_environment("cuda:0")
    train_script = args.mmdet3d_root / "tools" / "train.py"
    if not train_script.is_file():
        raise SystemExit(f"MMDetection3D train script not found: {train_script}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        str(train_script),
        str(args.config),
        "--work-dir",
        str(args.output_dir),
    ]
    if args.resume:
        command.append("--resume")
    subprocess.run(command, cwd=args.mmdet3d_root, check=True)


def run_evaluate(args):
    require_inference_environment(args.device)
    test_script = args.mmdet3d_root / "tools" / "test.py"
    if not test_script.is_file():
        raise SystemExit(f"MMDetection3D test script not found: {test_script}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        str(test_script),
        str(args.config),
        str(args.checkpoint),
        "--work-dir",
        str(args.output_dir),
    ]
    subprocess.run(command, cwd=args.mmdet3d_root, check=True)


def common_inference(parser: argparse.ArgumentParser):
    parser.add_argument("--points", type=existing_file, required=True)
    parser.add_argument("--config", type=existing_file, required=True)
    parser.add_argument("--checkpoint", type=existing_file, required=True)
    parser.add_argument("--features", type=int, default=5)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--score-threshold", type=float, default=0.30)
    parser.add_argument("--output-dir", type=Path, required=True)


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="stage", required=True)

    centerpoint = subparsers.add_parser("centerpoint")
    common_inference(centerpoint)
    centerpoint.set_defaults(run=run_centerpoint)

    bevfusion = subparsers.add_parser("bevfusion")
    common_inference(bevfusion)
    bevfusion.add_argument("--image", type=existing_file, required=True)
    bevfusion.add_argument("--infos", type=existing_file, required=True)
    bevfusion.set_defaults(run=run_bevfusion)

    train = subparsers.add_parser("fine-tune")
    train.add_argument("--mmdet3d-root", type=Path, required=True)
    train.add_argument("--config", type=existing_file, required=True)
    train.add_argument("--output-dir", type=Path, required=True)
    train.add_argument("--resume", action="store_true")
    train.set_defaults(run=run_train)

    evaluate = subparsers.add_parser("evaluate")
    evaluate.add_argument("--mmdet3d-root", type=Path, required=True)
    evaluate.add_argument("--config", type=existing_file, required=True)
    evaluate.add_argument("--checkpoint", type=existing_file, required=True)
    evaluate.add_argument("--device", default="cuda:0")
    evaluate.add_argument("--output-dir", type=Path, required=True)
    evaluate.set_defaults(run=run_evaluate)

    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.run(args)


if __name__ == "__main__":
    main()
