"""Tests for the chassis fit/clearance module using synthetic tube frames."""
import numpy as np, trimesh, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from suspension import SuspensionKinematics, Hardpoints
from suspension import chassis as ch


def _tube(p, q, r=12):
    p, q = np.array(p, float), np.array(q, float)
    s = trimesh.creation.cylinder(radius=r, segments=16, height=np.linalg.norm(q - p))
    d = (q - p); L = np.linalg.norm(d); d /= L; z = np.array([0, 0, 1.])
    v = np.cross(z, d); c = np.dot(z, d)
    if np.linalg.norm(v) < 1e-9:
        R = np.eye(3) if c > 0 else np.diag([1, -1, -1.])
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = (p + q) / 2
    s.apply_transform(T); return s


def _frame(hp):
    return trimesh.util.concatenate([
        _tube(hp.upper_front_inner, hp.upper_rear_inner),
        _tube(hp.lower_front_inner, hp.lower_rear_inner),
        _tube(hp.upper_front_inner, hp.lower_front_inner)])


def test_pickups_on_frame_pass_fit():
    hp = Hardpoints.default()
    res = ch.fit_check(hp, _frame(hp), tol_mm=15)
    on_frame = [r for r in res if "front" in r["point"] or "rear" in r["point"]]
    assert any(r["mountable"] for r in on_frame)


def test_clean_frame_is_clear():
    hp = Hardpoints.default()
    kin = SuspensionKinematics(hp)
    res = ch.clearance_check(kin, _frame(hp), warn_mm=8)
    assert res["verdict"] in ("CLEAR", "TIGHT")
    assert not any(v["collision"] for v in res["per_link"].values())


def test_intruding_tube_detected_as_collision():
    hp = Hardpoints.default()
    kin = SuspensionKinematics(hp)
    mid = 0.5 * (np.array(hp.lower_front_inner) + kin.static.lower_outer)
    mesh = trimesh.util.concatenate([_frame(hp),
                                     _tube(mid + [-50, 0, 0], mid + [50, 0, 0], r=18)])
    res = ch.clearance_check(kin, mesh, warn_mm=8)
    assert res["verdict"] == "COLLISION"


def test_manufacturing_sheet_has_pickups():
    hp = Hardpoints.default()
    kin = SuspensionKinematics(hp)
    sheet = ch.manufacturing_sheet(hp, kin)
    assert "lower_front_inner" in sheet and "upright_length" in sheet


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    p = 0
    for fn in fns:
        try:
            fn(); print(f"  PASS  {fn.__name__}"); p += 1
        except AssertionError as e:
            print(f"  FAIL  {fn.__name__}: {e}")
    print(f"\n{p}/{len(fns)} chassis tests passed")
