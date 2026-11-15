# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Direct-acting dampers (no rocker): motion ratio from the damper's own length change, damper force from statics
checked against virtual work, round trip through the manifests, and the InverseGenesis rocker/direct selector."""
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import genesis_analysis as ga, genesis_repro as gr, loadpath as lp     # noqa: E402
from suspension.kinematics import Hardpoints, SuspensionKinematics                    # noqa: E402

LOADS = [lp.WheelLoad(Fz=1200.0), lp.WheelLoad(Fy=-1800.0, Fz=1200.0, Mz=-47000.0),
         lp.WheelLoad(Fx=1933.0, Fz=1161.0), lp.WheelLoad(Fx_wc=-1560.0, Fy=-1400.0, Fz=1300.0, Mz=-37000.0)]


def _direct(body="lower"):
    hp = Hardpoints.default()
    if body == "lower":
        out, inn = ga.default_direct_damper(hp)
    elif body == "upper":
        out = 0.3 * np.asarray(hp.upper_front_inner) + 0.7 * np.asarray(hp.upper_outer) + [0, 0, 10.0]
        inn = out + np.array([10.0, -120.0, 180.0])
    else:
        out = np.asarray(hp.lower_outer) + np.array([0.0, -25.0, 40.0])
        inn = out + np.array([5.0, -200.0, 250.0])
    return replace(hp, actuation="direct", pushrod_attach=body, pushrod_outer=np.asarray(out, float),
                   spring_inner=np.asarray(inn, float))


def _damper_len(hp, state):
    return float(np.linalg.norm(np.asarray(hp.spring_inner) - np.asarray(state.pushrod_outer)))


def test_flags_and_the_chassis_end():
    hp = _direct()
    assert hp.has_direct_damper() and not hp.has_rocker()
    k = SuspensionKinematics(hp)
    assert k._has_direct and not k._has_rocker and k.motion_ratio_is_real()
    assert np.array_equal(k.actuation_inner(), hp.spring_inner)
    r = SuspensionKinematics(Hardpoints.default())
    assert np.array_equal(r.actuation_inner(), Hardpoints.default().rocker_pushrod) and not r._has_direct
    bare = replace(Hardpoints.default(), pushrod_outer=None)
    kb = SuspensionKinematics(bare)
    assert kb.actuation_inner() is None and not kb.motion_ratio_is_real()
    with pytest.raises(ValueError, match="^No pushrod/rocker or direct-acting damper defined on these hardpoints.$"):
        kb.spring_length_at(kb.static)


def test_rocker_points_are_ignored_when_direct():
    """A direct corner keeps any leftover rocker points but does not use them."""
    hp = _direct()
    assert hp.rocker_pivot is not None and not hp.has_rocker()
    assert not replace(hp, spring_inner=None).has_direct_damper()
    assert not replace(hp, pushrod_outer=None).has_direct_damper()
    assert not replace(hp, actuation="rocker").has_direct_damper()


@pytest.mark.parametrize("body", ["lower", "upper", "upright"])
def test_motion_ratio_is_the_damper_length_change(body):
    hp = _direct(body)
    k = SuspensionKinematics(hp)
    up, dn = k.solve_at_travel(5.0), k.solve_at_travel(-5.0)
    ref = abs((_damper_len(hp, up) - _damper_len(hp, dn)) / (up.wheel_center[2] - dn.wheel_center[2]))
    assert k.motion_ratio() == pytest.approx(ref, rel=1e-9)
    L, th, ok = k.spring_length_at(k.static)
    assert ok and th == 0.0 and L == pytest.approx(np.linalg.norm(hp.spring_inner - hp.pushrod_outer), abs=1e-9)
    trv, mr = k.motion_ratio_curve(-20, 20, 5)
    assert np.all(np.isfinite(mr)) and len(set(np.round(mr, 6))) > 1      # a real curve, not the constant proxy
    s = k.solve_at_travel(1.0); s2 = k.solve_at_travel(-1.0)                 # noqa: E702
    assert mr[2] == pytest.approx(abs((_damper_len(hp, s) - _damper_len(hp, s2))
                                      / (s.wheel_center[2] - s2.wheel_center[2])), rel=1e-9)


@pytest.mark.parametrize("body", ["lower", "upper", "upright"])
def test_damper_force_equals_virtual_work(body):
    """Damper force times its length rate equals the power of the wheel load, for each mounting body."""
    hp = _direct(body)
    k = SuspensionKinematics(hp)
    s0 = k.static
    h = 0.02
    sp, sm = k.solve_at_travel(h), k.solve_at_travel(-h)
    dL = (_damper_len(hp, sp) - _damper_len(hp, sm)) / (2 * h)

    def fit(A, B):
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
        return R, cb - R @ ca
    pts = lambda s: np.array([s.upper_outer, s.lower_outer, s.tie_rod_outer], float)   # noqa: E731
    Rp, tp = fit(pts(s0), pts(sp)); Rm, tm = fit(pts(s0), pts(sm))                     # noqa: E702
    vel = lambda p: ((Rp @ p + tp) - (Rm @ p + tm)) / (2 * h)                           # noqa: E731
    W = (Rp - Rm) / (2 * h)
    om = np.array([W[2, 1], W[0, 2], W[1, 0]])
    for L in LOADS:
        P = (np.dot([L.Fx, L.Fy, L.Fz], vel(np.asarray(s0.contact_patch))) + L.Fx_wc
             * vel(np.asarray(s0.wheel_center))[0] + L.Mz * om[2])
        mf = lp.solve_member_forces(k, s0, L)
        assert mf.has_pushrod and mf.forces["PR"] == pytest.approx(P / dL, rel=2e-3, abs=0.5)
        assert np.allclose(mf.axes["PR"], (hp.spring_inner - s0.pushrod_outer)
                           / np.linalg.norm(hp.spring_inner - s0.pushrod_outer))


def test_manifest_round_trip_keeps_rocker_hashes():
    d = gr.hp_to_dict(_direct())
    assert d["actuation"] == "direct"
    back = gr.hp_from_dict(d)
    assert back.actuation == "direct" and back.has_direct_damper()
    assert "actuation" not in gr.hp_to_dict(Hardpoints.default())            # rocker manifests unchanged
    assert Hardpoints.from_dict(_direct().as_dict()).actuation == "direct"


def test_actuation_summary_reports_the_damper():
    a = ga.actuation_summary(_direct())
    assert a["ok"] and a["actuation"] == "direct"
    assert a["pushrod_length_mm"] == pytest.approx(a["damper_static_mm"], abs=1e-6)
    r = ga.actuation_summary(replace(_direct(), spring_inner=None))
    assert not r["ok"] and "damper" in r["reason"]
    assert ga.actuation_summary(Hardpoints.default())["actuation"] == "rocker"


@pytest.mark.parametrize("body", ["lower", "upper", "upright"])
def test_carried_point_rides_with_its_body(body):
    """Move the whole corner rigidly: the carried point moves with it. Reshape another body: the point stays put."""
    seed = _direct(body)
    th = np.radians(4.0)
    R = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1.0]])
    t = np.array([3.0, -7.0, 4.0])
    moved = seed.copy()
    for k, v in seed.as_dict().items():
        if isinstance(v, list) and len(v) == 3 and k != "rocker_axis":
            setattr(moved, k, R @ np.asarray(v, float) + t)
    assert np.allclose(ga.carry_actuation_point(seed, moved), R @ seed.pushrod_outer + t, atol=1e-9)
    assert np.allclose(ga.carry_actuation_point(seed, seed), seed.pushrod_outer, atol=1e-9)
    other = {"lower": "upper_outer", "upper": "lower_outer", "upright": "upper_front_inner"}[body]
    reshaped = seed.copy()
    setattr(reshaped, other, np.asarray(getattr(seed, other)) + np.array([5.0, 5.0, 5.0]))
    assert np.allclose(ga.carry_actuation_point(seed, reshaped), seed.pushrod_outer, atol=1e-9)
    assert ga.carry_actuation_point(seed, moved, "rocker_spring") is not None
    assert ga.carry_actuation_point(replace(seed, pushrod_outer=None), moved) is None


def test_fullcar_flags_a_direct_corner_as_solved():
    from suspension import inverse_genesis_fullcar as igf
    src = Path(igf.__file__).read_text()
    assert "solved from direct-acting damper" in src


class _St:
    """Just enough of streamlit for the selector: widgets return their defaults or a forced choice."""
    def __init__(self, choice=None, body=None):
        self.choice, self.body, self.notes = choice, body, []

    def radio(self, label, opts, index=0, **k):
        return self.choice if self.choice is not None else opts[index]

    def selectbox(self, label, opts, index=0, **k):
        return self.body if self.body is not None else opts[index]

    def number_input(self, label, lo, hi, value, step, **k):
        return value

    def columns(self, n):
        return [self] * n

    def caption(self, t, **k):
        self.notes.append(t)
    warning = info = caption


def test_selector_switches_a_rocker_corner_to_direct_and_back():
    from ui import inverse_genesis as uig
    st = _St(choice=uig._ACT_LABELS[1])
    hp = uig._actuation_selector(st, Hardpoints.default(), "t")
    assert hp.actuation == "direct" and hp.has_direct_damper() and hp.pushrod_attach == "lower"
    o, i = ga.default_direct_damper(Hardpoints.default())
    assert np.allclose(hp.pushrod_outer, o) and np.allclose(hp.spring_inner, i)
    assert any("motion ratio" in n for n in st.notes)
    kept = uig._actuation_selector(_St(body="upright"), hp, "t")         # direct stays direct, ends kept
    assert kept.actuation == "direct" and kept.pushrod_attach == "upright"
    assert np.allclose(kept.pushrod_outer, hp.pushrod_outer)
    back = uig._actuation_selector(_St(choice=uig._ACT_LABELS[0]), hp, "t")
    assert back.actuation == "rocker" and back.has_rocker()
    assert hp.actuation == "direct"                                          # the input is not mutated
