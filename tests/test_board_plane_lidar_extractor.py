import importlib.util
from pathlib import Path

import numpy as np

P=Path(__file__).resolve().parents[1]/"scripts/board_plane_lidar_extractor.py"
s=importlib.util.spec_from_file_location("bp",P);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)


def synthetic(angle_deg=0, ambiguous=False):
    rng=np.random.default_rng(4); a=np.deg2rad(angle_deg)
    n=np.array([np.cos(a),np.sin(a),0.15]); n/=np.linalg.norm(n)
    u=np.cross([0,0,1],n); u/=np.linalg.norm(u); v=np.cross(n,u)
    c=np.array([6.6,-.25,-.2]); pts=[]
    for r in range(4):
      for col in range(7):
       for _ in range(45):
        du=(col-3+rng.random())*.096; dv=(r-1.5+rng.random())*.096
        x=c+du*u+dv*v+rng.normal(0,.002)*n
        pts.append([*x,40+100*((r+col)%2)])
    if ambiguous:
      for p in list(pts):
       x=np.asarray(p[:3])+.75*u;pts.append([*x,p[3]])
    axes=np.vstack([u,v,n]); cand={"centre_xyz_m":c.tolist(),"axes":axes.tolist()}
    return np.asarray(pts),cand


def test_rotated_board_metric_hypothesis_is_recovered():
    for angle in (0,37,91,143):
      p,c=synthetic(angle);result=m.board_plane_hypotheses(p,c,angle_step_deg=3)
      assert result["hypotheses"]
      best=result["hypotheses"][0]
      assert best["occupied_cells"]>=26
      assert best["identity_status"] if "identity_status" in best else result["status"]=="UNRESOLVED"


def test_symmetric_grid_does_not_assign_polarity():
    p,c=synthetic(52);h=m.board_plane_hypotheses(p,c)["hypotheses"][0]
    fake={"down_axis":h["v_axis"],"right_axis":h["u_axis"]}
    ids=m.inner_corner_hypotheses(h,fake)
    assert len(ids)==2
    assert all(x["identity_status"]=="HYPOTHESIS_ONLY" for x in ids)
    assert not np.allclose(ids[0]["corners_xyz_m"],ids[1]["corners_xyz_m"])


def test_ambiguous_scene_keeps_multiple_physical_hypotheses():
    p,c=synthetic(28,ambiguous=True);result=m.board_plane_hypotheses(p,c,keep=8)
    assert result["status"]=="UNRESOLVED"
    assert len(result["hypotheses"])>=2


def test_metric_kernel_fails_closed_without_padding_or_axis_swap():
    histogram=np.ones((31,57))
    assert m._box_sum(histogram,32,56) is None
    assert m._box_sum(histogram,30,58) is None
    result=m._box_sum(histogram,30,56)
    assert result is not None and result.shape==(2,2)
