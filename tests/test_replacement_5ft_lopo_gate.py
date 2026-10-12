import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/run_replacement_5ft_three_pose_lopo.py"
SPEC = importlib.util.spec_from_file_location("replacement_5ft_lopo_gate_test", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_missing_freeze_manifest_fields_fail_closed(tmp_path):
    manifest = tmp_path / "freeze.json"
    manifest.write_text(json.dumps({"distance_ft": 5}))
    with pytest.raises(ValueError, match="missing"):
        MODULE.validate_manifest(manifest)


def test_nonpassing_preflight_cannot_enter_lopo(tmp_path):
    preflight = tmp_path / "preflight.json"
    observations = tmp_path / "observations.csv"
    image = tmp_path / "image.png"
    preflight.write_text(json.dumps({"overall_status": "FAIL", "freeze_authorized": False}))
    observations.write_text("distance_ft,index\n")
    image.write_bytes(b"not needed because preflight must fail first")
    manifest = tmp_path / "freeze.json"
    manifest.write_text(json.dumps({
        "distance_ft": 5,
        "preflight_json": str(preflight),
        "observations_csv": str(observations),
        "rgb_image": str(image),
        "canonical_lidar_frame": "L10",
        "frozen_before_projection": True,
        "physical_identity_frozen": True,
        "source_hashes": {
            "preflight_json": MODULE.sha256(preflight),
            "observations_csv": MODULE.sha256(observations),
            "rgb_image": MODULE.sha256(image),
        },
    }))
    with pytest.raises(ValueError, match="not all-PASS"):
        MODULE.validate_manifest(manifest)


def test_unfrozen_identity_cannot_enter_lopo(tmp_path):
    manifest = tmp_path / "freeze.json"
    manifest.write_text(json.dumps({
        "distance_ft": 5,
        "preflight_json": "unused",
        "observations_csv": "unused",
        "rgb_image": "unused",
        "canonical_lidar_frame": "L10",
        "frozen_before_projection": False,
        "physical_identity_frozen": False,
        "source_hashes": {},
    }))
    with pytest.raises(ValueError, match="not frozen"):
        MODULE.validate_manifest(manifest)
