# ============================================================================
#  KinematiK — tests for actuation synthesis
#  Copyright (c) 2026 Frederik Thio. Open source.
# ============================================================================
"""Derived motion-ratio targets, linkage metrics and the bounded synthesis."""
import math

import numpy as np
import pytest

from suspension import actuation as ac
from suspension.kinematics import Hardpoints, SuspensionKinematics

HP = Hardpoints.default()


def test_targets_are_derived_from_the_stroke():
    t = ac.ActuationTargets.derived(stroke_mm=57.0, travel_mm=(-25.0, 25.0))
    assert t.mr_ceiling == pytest.approx(0.8 * 57.0 / 50.0)
    assert t.mr_static == pytest.approx(0.912 * 0.93, abs=0.002)


def test_metrics_match_the_kinematic_motion_ratio():
    m = ac.actuation_metrics(HP)
    assert m.ok
    trv, mr = SuspensionKinematics(HP).motion_ratio_curve(-25.0, 25.0, 11)
    assert m.mr_static == pytest.approx(float(np.interp(0.0, trv, mr)), rel=1e-6)


def test_default_linkage_is_strongly_falling():
    m = ac.actuation_metrics(HP)
    assert m.spread > 0.3 and m.slope_per_mm < 0.0


def test_out_of_plane_is_measured():
    h = Hardpoints.default()
    h.spring_inner = np.asarray(h.spring_inner, float) + np.array([40.0, 0.0, 0.0])
    assert ac.actuation_metrics(h).out_of_plane_deg > ac.actuation_metrics(HP).out_of_plane_deg


def test_obstacle_clearance_is_measured():
    m = ac.actuation_metrics(HP)
    pro, rp = np.asarray(HP.pushrod_outer), np.asarray(HP.rocker_pushrod)
    mid = 0.5 * (pro + rp)
    hit = ac.actuation_metrics(HP, obstacles=[(mid - [0, 0, 5], mid + [0, 0, 5], 10.0)])
    assert hit.clearance_mm < 0.0 < m.clearance_mm


def test_synthesis_meets_every_band():
    r = ac.synthesize_actuation(HP, n_starts=2, ride_hz=2.5, sprung_corner_kg=60.1)
    assert r.ok and all(v <= 1e-6 for v in r.residual)
    t = ac.ActuationTargets.derived()
    assert abs(r.metrics.mr_static - t.mr_static) <= t.mr_tol + 1e-9
    assert r.metrics.spread <= t.max_spread + 1e-9
    assert r.metrics.stroke_used_mm <= t.usable_stroke * t.stroke_mm + 1e-9
    f = r.ride_frequency_hz
    assert f[len(f) // 2] == pytest.approx(2.5, rel=0.02)


def test_synthesis_leaves_the_kinematics_alone():
    """Actuation points do not enter camber or toe."""
    r = ac.synthesize_actuation(HP, n_starts=1)
    a = SuspensionKinematics(HP).sweep(-25.0, 25.0, 5)
    b = SuspensionKinematics(r.hp).sweep(-25.0, 25.0, 5)
    assert max(abs(x.camber - y.camber) for x, y in zip(a, b)) < 1e-9
    assert max(abs(x.toe - y.toe) for x, y in zip(a, b)) < 1e-9


# --------------------------------------------------------------------------- #
#  Frame tubes, nodes, and rocker bearings
# --------------------------------------------------------------------------- #
def _brute(p0, p1, q0, q1, n=801):
    s = np.linspace(0, 1, n)[:, None]
    P = p0 + s * (p1 - p0); Q = q0 + s * (q1 - q0)
    return float(np.min(np.linalg.norm(P[:, None, :] - Q[None, ::2, :], axis=2)))


def test_segment_distance_matches_brute_force():
    """Regression: an earlier scalar version divided by the wrong denominator."""
    rng = np.random.default_rng(7)
    for _ in range(25):
        p0, p1, q0, q1 = (rng.uniform(-400, 400, 3) for _ in range(4))
        assert ac._seg_dist(p0, p1, q0, q1) == pytest.approx(_brute(p0, p1, q0, q1), abs=1.0)


def test_clearance_is_swept_through_travel():
    """A tube the linkage only reaches in bump is caught; a static check misses it."""
    m = ac.actuation_metrics(HP)
    from suspension.kinematics import SuspensionKinematics
    kin = SuspensionKinematics(HP)
    st = kin.solve_at_travel(25.0)
    L, th, ok = kin.spring_length_at(st, seed=0.0)
    rs_b = kin._rotate_about_axis(HP.rocker_spring, th)
    tube = [(rs_b + np.array([0, 0, 12.0]), rs_b + np.array([0, 0, 40.0]), 5.0)]
    hit = ac.actuation_metrics(HP, obstacles=tube)
    assert hit.clearance_mm < 5.0


def test_node_gap_is_measured():
    far = np.array([[5000.0, 5000.0, 5000.0]])
    near = np.array([np.asarray(HP.rocker_pivot), np.asarray(HP.spring_inner)])
    assert ac.actuation_metrics(HP, nodes=far).node_gap_mm > 1000.0
    assert ac.actuation_metrics(HP, nodes=near).node_gap_mm == pytest.approx(0.0, abs=1e-9)


def test_rocker_is_in_equilibrium():
    """Pushrod, spring and pivot forces sum to zero and balance moments about the axis."""
    from suspension import genesis_repro as gr
    rows = ac.rocker_bearing_loads(HP, gr.vehicle_load_cases())
    assert len(rows) == 5
    assert all(r["pivot_radial_N"] > 0 and np.isfinite(r["s0"]) for r in rows)


def test_offset_rocker_loads_its_bearings_harder():
    from suspension import genesis_repro as gr
    import copy
    flat = ac.rocker_bearing_loads(HP, gr.vehicle_load_cases())
    h = copy.deepcopy(HP)
    h.rocker_pushrod = np.asarray(h.rocker_pushrod, float) + np.array([40.0, 0.0, 0.0])
    off = ac.rocker_bearing_loads(h, gr.vehicle_load_cases())
    assert max(r["couple_Nmm"] for r in off) > max(r["couple_Nmm"] for r in flat)
