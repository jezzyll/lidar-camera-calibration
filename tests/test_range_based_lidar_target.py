import importlib.util
from pathlib import Path
import numpy as np

P = Path(__file__).resolve().parents[1] / "scripts/range_based_lidar_target.py"
s = importlib.util.spec_from_file_location("range_target", P)
m = importlib.util.module_from_spec(s); s.loader.exec_module(m)


def cloud(seed=0):
    rng = np.random.default_rng(seed)
    y = rng.uniform(-.45, .45, 500); z = rng.uniform(-.19, .19, 500)
    x = 6.5 + rng.normal(0, .003, 500)
    return np.c_[x, y, z, rng.uniform(105, 130, 500)]


def test_rotation_invariant_candidate_geometry():
    p = cloud(); a = np.deg2rad(73); R = np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    x = m.discover_candidates(p, 6.5); y = m.discover_candidates(m.rotate_points(p, R), 6.5)
    assert len(x["candidates"]) == len(y["candidates"]) == 1
    assert x["range_shell_points"] == y["range_shell_points"]
    assert np.allclose(sorted(x["candidates"][0]["p01_p99_plane_span_m"]),
                       sorted(y["candidates"][0]["p01_p99_plane_span_m"]), atol=1e-9)


def test_discovery_never_claims_identity():
    result = m.discover_candidates(cloud(), 6.5)
    assert result["identity_status"] == "UNRESOLVED"
    assert "selection_policy" in result
