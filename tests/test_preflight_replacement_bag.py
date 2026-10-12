import importlib.util
import json
import sqlite3
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/preflight_replacement_bag.py"
SPEC = importlib.util.spec_from_file_location("replacement_preflight_test", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_db(path):
    with sqlite3.connect(path) as db:
        db.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT,offered_qos_profiles TEXT)")
        db.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)")


def test_resolve_folder_includes_all_metadata_split_files(tmp_path):
    first = tmp_path / "capture_0.db3"
    second = tmp_path / "capture_1.db3"
    make_db(first); make_db(second)
    (tmp_path / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  relative_file_paths:\n"
        "    - capture_0.db3\n"
        "    - capture_1.db3\n"
    )
    assert MODULE.resolve_bag_parts(str(tmp_path)) == [first.resolve(), second.resolve()]
    assert MODULE.resolve_bag_parts(str(first)) == [first.resolve(), second.resolve()]


def test_nonexistent_recording_is_rejected(tmp_path):
    with pytest.raises(FileNotFoundError):
        MODULE.resolve_bag_parts(str(tmp_path / "missing.db3"))


def test_metadata_colon_removed_by_transfer_is_resolved(tmp_path):
    actual = tmp_path / "checker9.6cm9.6cm_0.db3"
    make_db(actual)
    (tmp_path / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  relative_file_paths:\n"
        "    - checker9.6cm:9.6cm_0.db3\n"
    )
    assert MODULE.resolve_bag_parts(str(tmp_path)) == [actual.resolve()]


def test_missing_mounting_evidence_is_unresolved():
    result = MODULE.mounting_evidence(None)
    assert result["status"] == "UNRESOLVED"


def test_mounting_evidence_requires_frozen_reference(tmp_path):
    path = tmp_path / "mount.json"
    path.write_text(json.dumps({
        "camera_serial": "D455-example",
        "lidar_serial": "Mid360-example",
        "rig_locked": True,
        "witness_marks_unchanged": True,
        "reference_configuration_id": "locked-10-15-reference",
    }))
    assert MODULE.mounting_evidence(str(path))["status"] == "UNRESOLVED"


def test_complete_matching_mounting_evidence_passes(tmp_path):
    candidate = tmp_path / "candidate.json"
    reference = tmp_path / "reference.json"
    candidate.write_text(json.dumps({
        "camera_serial": "D455-example",
        "lidar_serial": "Mid360-example",
        "rig_locked": True,
        "witness_marks_unchanged": True,
        "reference_configuration_id": "locked-10-15-reference",
    }))
    reference.write_text(json.dumps({
        "camera_serial": "D455-example",
        "lidar_serial": "Mid360-example",
        "reference_configuration_id": "locked-10-15-reference",
    }))
    assert MODULE.mounting_evidence(str(candidate), str(reference))["status"] == "PASS"


def test_board_evidence_requires_full_topology_and_measured_pitch(tmp_path):
    photo = tmp_path / "board.jpg"
    photo.write_bytes(b"evidence")
    evidence = tmp_path / "board.json"
    evidence.write_text(json.dumps({
        "physical_cells": [7, 4],
        "inner_corners": [6, 3],
        "nominal_pitch_m": 0.096,
        "horizontal_pitch_measurements_m": [0.096, 0.0961, 0.0959],
        "vertical_pitch_measurements_m": [0.096, 0.0962, 0.0958],
        "checker_width_m": 0.672,
        "checker_height_m": 0.384,
        "photo_paths": [str(photo)],
    }))
    assert MODULE.board_evidence(str(evidence))["status"] == "PASS"


def test_three_column_sketch_does_not_pass_board_evidence(tmp_path):
    photo = tmp_path / "sketch.jpg"
    photo.write_bytes(b"evidence")
    evidence = tmp_path / "board.json"
    evidence.write_text(json.dumps({
        "physical_cells": [3, 2],
        "inner_corners": [2, 1],
        "nominal_pitch_m": 0.096,
        "horizontal_pitch_measurements_m": [0.096, 0.096, 0.096],
        "vertical_pitch_measurements_m": [0.096, 0.096, 0.096],
        "checker_width_m": 0.70,
        "checker_height_m": 0.425,
        "photo_paths": [str(photo)],
    }))
    assert MODULE.board_evidence(str(evidence))["status"] == "FAIL"


def test_declared_thresholds_are_frozen():
    assert MODULE.PITCH_M == 0.096
    assert MODULE.PATTERN == (6, 3)
    assert MODULE.PRINTED_CELLS == (7, 4)
    assert MODULE.MAX_DRIFT_P95_DEG == 0.03
    assert MODULE.MAX_PREDICTED_SIGMA_PX == 1.0


def test_empty_lidar_roi_generates_unresolved_image(tmp_path):
    import numpy as np
    path = tmp_path / "empty.png"
    MODULE.make_lidar_evidence(
        [{"points": np.array([[1.0, 20.0, 20.0, 5.0]])}], None,
        {"status": "INSUFFICIENT_LIDAR"}, 22, path,
    )
    assert path.is_file() and path.stat().st_size > 0


def test_rotation_invariant_candidate_does_not_claim_identity():
    import numpy as np
    rng = np.random.default_rng(0)
    centre = np.array([1.5, -6.33, -0.35])
    points = np.c_[centre + rng.normal(0, [0.12, 0.03, 0.08], (300, 3)), np.full(300, 110.0)]
    result = MODULE.rotation_invariant_candidate(points, 22)
    assert result["status"] == "UNRESOLVED"
    assert result["clusters"]
    assert "no origin/corners accepted" in result["reason"]
