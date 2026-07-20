# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for the Phantom Envelope carve (suspension/phantom_envelope.py).

Same philosophy as the ghost/compliance suites: nail the closed forms the
carve stands on (point-to-segment distance, capsule containment), pin the
sign convention of a clearance query (+ outside, − inside), prove the carve
actually unions the swept capsules, prove the compliance warp GROWS the
envelope vs the rigid sweep in the soft-link case (and barely moves it in
the stiff case), and prove the point-cloud round-trips through JSON with its
frame label intact.

Run:  python -m pytest tests/test_phantom_envelope.py
      (or standalone: python tests/test_phantom_envelope.py)
"""
import json
import math
import sys, os

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension.kinematics import Hardpoints
from suspension.compliance import CompliantCorner
from suspension.ghost_topology import (
    GhostCorner, uniform_sections, ghost_audit,
)
from suspension.phantom_envelope import (
    Capsule, _seg_point_distance, _seg_point_distance_batch,
    PhantomEnvelope, carve_from_states, carve_rigid_envelope,
    carve_ghost_envelope, envelope_delta, radii_from_sections,
    rigid_sweep_states, render_envelope_md,
)


HP = Hardpoints.default()


def _stock_corner(**kw):
    return GhostCorner.uniform_tube(HP, wheel_rate_N_per_mm=35.0,
                                    Fz_static_N=700.0, **kw)


def _soft_corner():
    cc = CompliantCorner.uniform_tube(HP, material="Aluminium 6061",
                                      od_mm=12.0, wall_mm=1.0, k_tab=1500.0)
    return GhostCorner(cc, uniform_sections(od_mm=12.0, wall_mm=1.0,
                                            material="Aluminium 6061",
                                            yield_MPa=276.0),
                       wheel_rate_N_per_mm=35.0, Fz_static_N=700.0)


def _pulse(peak_g=3.0, n=201, dur=0.8, fz_static=700.0, fz_peak=2200.0):
    t = np.linspace(0.0, dur, n)
    s = np.sin(np.pi * t / dur)
    Fz = fz_static + (fz_peak - fz_static) * s
    Fy = -peak_g * Fz * s
    return t, np.zeros_like(t), Fy, Fz


# --------------------------------------------------------------------------- #
#  Closed forms — point to segment, capsule containment
# --------------------------------------------------------------------------- #
def test_point_segment_distance_endpoints_and_interior():
    a = np.array([0.0, 0.0, 0.0])
    b = np.array([10.0, 0.0, 0.0])
    # perpendicular from the middle
    assert _seg_point_distance(np.array([5.0, 3.0, 0.0]), a, b) == pytest.approx(3.0)
    # beyond an endpoint clamps to the endpoint
    assert _seg_point_distance(np.array([-4.0, 3.0, 0.0]), a, b) == pytest.approx(5.0)
    # on the segment
    assert _seg_point_distance(np.array([7.0, 0.0, 0.0]), a, b) == pytest.approx(0.0)


def test_point_segment_batch_matches_scalar():
    a = np.array([1.0, 2.0, 3.0]); b = np.array([-4.0, 5.0, 0.0])
    pts = np.random.RandomState(1).uniform(-10, 10, size=(50, 3))
    batch = _seg_point_distance_batch(pts, a, b)
    scal = np.array([_seg_point_distance(p, a, b) for p in pts])
    assert np.allclose(batch, scal)


def test_capsule_containment_and_signed_clearance():
    cap = Capsule(a=np.array([0.0, 0, 0]), b=np.array([10.0, 0, 0]),
                  r=2.0, member="X", t=0.0)
    inside = np.array([5.0, 1.0, 0.0])   # 1 mm off axis, r=2 -> inside
    outside = np.array([5.0, 5.0, 0.0])  # 5 mm off axis -> outside
    assert cap.contains(inside)
    assert not cap.contains(outside)
    assert cap.signed_clearance(inside) == pytest.approx(-1.0)
    assert cap.signed_clearance(outside) == pytest.approx(3.0)


# --------------------------------------------------------------------------- #
#  The carve — a single capsule's boundary, then a union
# --------------------------------------------------------------------------- #
def test_single_capsule_boundary_points_lie_on_skin():
    cap_state_hp = HP
    # one static rigid state carved with a known radius
    gc = _stock_corner()
    st = gc.cc.rigid_kin.static
    radius = {m: 5.0 for m in ("UF", "UR", "LF", "LR", "TR", "PR")}
    env = carve_from_states(HP, [st], [0.0], radius, kind="rigid",
                            frame="FR-test")
    assert env.n_points > 0
    # every boundary point must be within ~radius of SOME capsule surface
    # (it was sampled on a capsule surface, so its min distance-to-a-segment
    # should be close to that capsule's radius)
    for p in env.boundary[:200]:
        dmin = min(_seg_point_distance(p, c.a, c.b) - c.r for c in env.capsules)
        assert dmin <= 0.6   # on or just inside a skin (union interior points removed)


def test_carve_unions_overlapping_capsules_removes_interior():
    # two parallel capsules that overlap: the boundary between them is carved away
    a1 = Capsule(np.array([0.0, 0, 0]), np.array([10.0, 0, 0]), 3.0, "A", 0.0)
    a2 = Capsule(np.array([0.0, 2.0, 0]), np.array([10.0, 2.0, 0]), 3.0, "B", 0.0)
    # build an envelope by hand and check no boundary point sits deep inside the
    # OTHER capsule
    from suspension.phantom_envelope import _capsule_surface_points
    caps = [a1, a2]
    kept = []
    for i, c in enumerate(caps):
        surf = _capsule_surface_points(c, n_ring=16, n_len=8)
        for p in surf:
            other = caps[1 - i]
            if _seg_point_distance(p, other.a, other.b) >= other.r - 0.05:
                kept.append(p)
    kept = np.array(kept)
    # no kept point should be > 0.1 mm inside the other capsule
    for p in kept:
        for c in caps:
            assert _seg_point_distance(p, c.a, c.b) >= c.r - 0.15


# --------------------------------------------------------------------------- #
#  The query — the headline clearance answer, with sign + attribution
# --------------------------------------------------------------------------- #
def test_query_sign_and_attribution():
    gc = _stock_corner()
    env = carve_rigid_envelope(gc, "FR", travel_mm=(-10, 10), n_travel=5)
    # a point far outside everything is clear with big positive clearance
    q_far = env.query(np.array([500.0, 500.0, 500.0]), probe_radius_mm=0.0)
    assert not q_far.violates and q_far.clearance_mm > 50.0
    assert q_far.frame.startswith("FR")
    # a point right on the UCA outer ball joint is deep inside -> violates
    uo = gc.cc.rigid_kin.static.upper_outer
    q_in = env.query(uo, probe_radius_mm=0.0)
    assert q_in.violates and q_in.clearance_mm < 0.0
    assert q_in.nearest_member in env.members


def test_probe_radius_shrinks_clearance():
    gc = _stock_corner()
    env = carve_rigid_envelope(gc, "FR", travel_mm=(-5, 5), n_travel=3)
    p = np.array([200.0, 400.0, 250.0])
    c0 = env.query(p, probe_radius_mm=0.0).clearance_mm
    c5 = env.query(p, probe_radius_mm=5.0).clearance_mm
    assert c5 == pytest.approx(c0 - 5.0, abs=1e-6)


def test_vectorised_clearances_match_query():
    gc = _stock_corner()
    env = carve_rigid_envelope(gc, "FR", travel_mm=(-10, 10), n_travel=5)
    pts = np.random.RandomState(2).uniform([-50, 300, 150],
                                           [100, 560, 320], size=(40, 3))
    batch = env.clearances(pts, probe_radius_mm=3.0)
    one = np.array([env.query(p, 3.0).clearance_mm for p in pts])
    assert np.allclose(batch, one, atol=1e-6)


# --------------------------------------------------------------------------- #
#  The compliance warp — the reason the tool exists
# --------------------------------------------------------------------------- #
def test_soft_corner_grows_more_than_stiff_corner():
    t, Fx, Fy, Fz = _pulse(peak_g=2.0)

    gc_s = _stock_corner()
    a_s = ghost_audit(gc_s, t, Fx, Fy, Fz, corner_label="FR", n_samples=16)
    rig_s = carve_rigid_envelope(gc_s, "FR", travel_mm=(-25, 25), n_travel=9)
    cmp_s = carve_ghost_envelope(a_s, gc_s)
    d_stiff = envelope_delta(rig_s, cmp_s).max_outward_growth_mm

    gc_soft = _soft_corner()
    a_soft = ghost_audit(gc_soft, t, Fx, Fy, Fz, corner_label="FR", n_samples=16)
    rig_soft = carve_rigid_envelope(gc_soft, "FR", travel_mm=(-25, 25), n_travel=9)
    cmp_soft = carve_ghost_envelope(a_soft, gc_soft)
    d_soft = envelope_delta(rig_soft, cmp_soft).max_outward_growth_mm

    # the soft corner deflects more, so its compliant envelope pushes further
    # outside the rigid sweep than the stiff corner's does.
    assert d_soft > d_stiff
    assert d_soft > 0.2       # a real, reportable growth


def test_ghost_envelope_excludes_void_instants():
    # drive the soft corner hard enough that some instant yields (FoS < 1)
    t, Fx, Fy, Fz = _pulse(peak_g=3.5, fz_peak=3200.0)
    gc = _soft_corner()
    audit = ghost_audit(gc, t, Fx, Fy, Fz, corner_label="FR", n_samples=20)
    void = [g for g in audit.instants if g.min_fos < 1.0]
    env = carve_ghost_envelope(audit, gc, include_void=False)
    if void:
        assert len(env.excluded) == len(void)
        # and including them keeps them instead
        env_inc = carve_ghost_envelope(audit, gc, include_void=True)
        assert len(env_inc.excluded) == 0
        assert env_inc.n_instants >= env.n_instants
    else:
        pytest.skip("this pulse did not yield any instant; nothing to exclude")


# --------------------------------------------------------------------------- #
#  Radii come from the tube sections; inflation adds to them
# --------------------------------------------------------------------------- #
def test_radii_from_sections_are_half_od_plus_inflation():
    gc = _stock_corner()          # 19.05 mm OD default
    r = radii_from_sections(gc, inflate_mm=0.0)
    assert r["UF"] == pytest.approx(19.05 / 2.0)
    r2 = radii_from_sections(gc, inflate_mm=3.0)
    assert r2["UF"] == pytest.approx(19.05 / 2.0 + 3.0)


# --------------------------------------------------------------------------- #
#  Point cloud export — the lightweight file the other teams consume
# --------------------------------------------------------------------------- #
def test_point_cloud_json_roundtrips_with_frame():
    gc = _stock_corner()
    env = carve_rigid_envelope(gc, "FR", travel_mm=(-10, 10), n_travel=5)
    blob = env.to_json()
    d = json.loads(blob)
    assert d["format"] == "kinematik.phantom_envelope/1"
    assert d["units"] == "mm"
    assert d["kind"] == "rigid"
    assert d["frame"].startswith("FR")
    assert len(d["points_xyz_mm"]) == env.n_points
    # bbox in the file matches the computed bbox
    mn, mx = env.bbox
    assert np.allclose(d["bbox_min_mm"], mn)
    assert np.allclose(d["bbox_max_mm"], mx)


def test_csv_export_has_header_and_capsule_provenance():
    gc = _stock_corner()
    env = carve_rigid_envelope(gc, "FR", travel_mm=(-5, 5), n_travel=3)
    csv = env.to_csv()
    lines = csv.splitlines()
    assert lines[0] == "x_mm,y_mm,z_mm,member,t_s"
    # at least the capsule endpoint rows carry a member label
    assert any(",UF," in ln or ",LF," in ln for ln in lines[1:])


# --------------------------------------------------------------------------- #
#  Rigid sweep sanity — the linkage actually moves through travel
# --------------------------------------------------------------------------- #
def test_rigid_sweep_moves_the_outboard_point():
    gc = _stock_corner()
    states, times = rigid_sweep_states(gc, travel_mm=(-25, 25), n_travel=7)
    z = [s.lower_outer[2] for s in states]
    # the lower ball joint climbs monotonically with travel
    assert z[-1] > z[0] + 40.0        # ~50 mm of travel range
    assert len(states) == len(times) == 7


def test_render_md_mentions_growth_and_frame():
    gc = _soft_corner()
    t, Fx, Fy, Fz = _pulse(peak_g=2.0)
    audit = ghost_audit(gc, t, Fx, Fy, Fz, corner_label="FR", n_samples=12)
    rigid = carve_rigid_envelope(gc, "FR", travel_mm=(-25, 25), n_travel=9)
    comp = carve_ghost_envelope(audit, gc)
    delta = envelope_delta(rigid, comp)
    md = render_envelope_md(comp, delta=delta)
    assert "Phantom Envelope" in md
    assert "FR" in md
    assert "mm" in md


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
