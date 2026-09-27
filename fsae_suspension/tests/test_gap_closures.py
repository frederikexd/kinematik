# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
Tests for the gaps the inverse-synthesis paper's review exposed and the code
now closes:

* shaft-reacted longitudinal force at the wheel centre (inboard drive / brakes)
  and the missing traction cases for a driven axle;
* load-amplification, joint-lash and frame-twist bounds inside the search;
* tire compliance in series and aero heave in the target chain;
* correlated clusters and systematic offsets in the tolerance field;
* the tool-access wall;
* the band sweep.

Every new input is optional and serialised only when used, so each test of a
default also guards the input hash of every manifest published before.
"""
from dataclasses import replace

import numpy as np
import pytest

from suspension.kinematics import Hardpoints, SuspensionKinematics
from suspension import loadpath as lp
from suspension import elastokinematics as ek
from suspension import inverse_genesis as ig
from suspension import genesis_repro as gr
from suspension import target_derivation as td
from suspension.kinematik_stochastic import (ToleranceField, ToleranceCluster,
                                             ToleranceSpec, _perturbed)


@pytest.fixture(scope="module")
def hp():
    return Hardpoints.default()


@pytest.fixture(scope="module")
def solved(hp):
    kin = SuspensionKinematics(hp)
    return kin, kin.solve_at_travel(0.0)


# --------------------------------------------------------------------------- #
#  Drive torque: traction through an inboard final drive
# --------------------------------------------------------------------------- #
def _member_moment_about(mf, point):
    """Moment (N*mm) of all member forces on the upright about a point."""
    M = np.zeros(3)
    for m, T in mf.forces.items():
        u = np.asarray(mf.axes[m], float)
        p = np.asarray(mf.outboard[m], float)
        M += np.cross(p - point, T * u)
    return M


def test_wheel_centre_force_puts_no_moment_about_the_wheel_centre(solved):
    kin, st = solved
    mf = lp.solve_member_forces(kin, st, lp.WheelLoad(Fx_wc=-1500.0))
    assert mf.residual < 1e-6
    M = _member_moment_about(mf, np.asarray(st.wheel_center, float))
    assert np.linalg.norm(M) < 1e-3 * 1500.0          # N*mm, numerically zero


def test_patch_force_puts_the_wheel_radius_moment_through_the_links(solved):
    kin, st = solved
    Fx = -1500.0
    mf = lp.solve_member_forces(kin, st, lp.WheelLoad(Fx=Fx))
    M = _member_moment_about(mf, np.asarray(st.wheel_center, float))
    lever = np.asarray(st.contact_patch, float) - np.asarray(st.wheel_center, float)
    # the links must react the patch force's moment about the wheel centre
    expect = -np.cross(lever, np.array([Fx, 0.0, 0.0]))
    assert np.allclose(M, expect, atol=1e-3 * abs(Fx) * 228.0)


def test_default_wheel_load_is_unchanged(solved):
    kin, st = solved
    a = lp.solve_member_forces(kin, st, lp.WheelLoad(Fx=800, Fy=-1500, Fz=1300))
    b = lp.solve_member_forces(kin, st, lp.WheelLoad(Fx=800, Fy=-1500, Fz=1300,
                                                     Fx_wc=0.0))
    assert a.forces == b.forces


def test_wheel_load_from_corner_routes_traction_to_the_wheel_centre():
    w = lp.wheel_load_from_corner(1200.0, mu_lateral=1.0, mu_long=-1.0,
                                  inboard_drive=True)
    assert w.Fx == 0.0 and w.Fx_wc == pytest.approx(-1200.0)
    b = lp.wheel_load_from_corner(1200.0, mu_long=1.0, inboard_drive=True)
    assert b.Fx == pytest.approx(1200.0) and b.Fx_wc == 0.0   # braking stays
    c = lp.wheel_load_from_corner(1200.0, mu_long=1.0, inboard_brakes=True)
    assert c.Fx == 0.0 and c.Fx_wc == pytest.approx(1200.0)


def test_screening_defaults_are_the_five_cases_and_traction_adds_two():
    base = gr.vehicle_load_cases(axle="rear")
    assert len(base) == 5 and all(c.Fx_wc == 0.0 for c in base)
    drv = gr.vehicle_load_cases(axle="rear", traction_g=1.06)
    assert len(drv) == 7
    trac = drv[5:]
    assert all(c.Fx == 0.0 and c.Fx_wc < 0.0 for c in trac)
    assert trac[0].Fz > base[3].Fz / 3.0          # load moves onto the rear
    out = gr.vehicle_load_cases(axle="rear", traction_g=1.06, inboard_drive=False)
    assert all(c.Fx < 0.0 and c.Fx_wc == 0.0 for c in out[5:])


def test_screening_sees_the_traction_case(hp):
    res_in = gr.structural_screening(hp, axle="rear", traction_g=1.06)
    res_out = gr.structural_screening(hp, axle="rear", traction_g=1.06,
                                      inboard_drive=False)
    assert res_in != res_out


def test_rocker_bearing_screen_reads_the_wheel_centre_force(hp):
    from suspension import actuation as act
    kin = SuspensionKinematics(hp)
    if not kin.motion_ratio_is_real():
        pytest.skip("reference corner has no rocker")
    patch = gr.LoadCaseSpec("patch", Fz=1500.0, Fx=-1200.0)
    shaft = gr.LoadCaseSpec("shaft", Fz=1500.0, Fx_wc=-1200.0)
    a, b = act.rocker_bearing_loads(hp, [patch, shaft])
    assert a["pushrod_N"] != pytest.approx(b["pushrod_N"])


# --------------------------------------------------------------------------- #
#  New walls: load amplification, joint lash, frame twist
# --------------------------------------------------------------------------- #
def test_elasto_spec_hash_stable_and_round_trips():
    plain = ek.ElastoSpec(Fy=-1500.0, Fz=1300.0)
    assert set(plain.to_dict()) == {"stiffness", "Fx", "Fy", "Fz", "Mz", "n_steps"}
    full = ek.ElastoSpec(Fy=-1500.0, Fz=1300.0, Fx_wc=-600.0,
                         joint_lash_mm={"TR": 0.05},
                         twist=ek.FrameTwist(2000.0, 618.0, 250.0))
    assert ek.ElastoSpec.from_dict(full.to_dict()).to_dict() == full.to_dict()


def test_lash_deadband_is_linear_in_clearance(hp):
    a = ek.lash_deadband(hp, {"TR": 0.025})["toe"]
    b = ek.lash_deadband(hp, {"TR": 0.05})["toe"]
    assert a > 0.0 and b == pytest.approx(2.0 * a, rel=1e-6)


def test_twist_toe_scales_inversely_with_frame_stiffness(hp):
    t1 = ek.FrameTwist(2000.0, 618.0, 250.0).toe_deg(hp)
    t2 = ek.FrameTwist(4000.0, 618.0, 250.0).toe_deg(hp)
    assert t1 == pytest.approx(2.0 * t2, rel=1e-9)


def test_new_bounds_are_walls(hp):
    spec = ek.ElastoSpec(Fy=-1500.0, Fz=1300.0, joint_lash_mm={"TR": 0.05},
                         twist=ek.FrameTwist(2000.0, 618.0, 250.0))
    vals = ig.properties_of(hp, ig.SolvedPropertyBounds(
        bounds=[ig.PropertyBound("member_force_ratio", hi=10.0),
                ig.PropertyBound("lash_toe_deg", hi=1.0),
                ig.PropertyBound("twist_toe_deg", hi=1.0)], elasto=spec))
    assert all(np.isfinite(v) and v > 0.0 for v in vals.values())
    tight = ig.SolvedPropertyBounds(
        bounds=[ig.PropertyBound("member_force_ratio",
                                 hi=0.5 * vals["member_force_ratio"]),
                ig.PropertyBound("lash_toe_deg", hi=0.5 * vals["lash_toe_deg"]),
                ig.PropertyBound("twist_toe_deg", hi=0.5 * vals["twist_toe_deg"])],
        elasto=spec)
    assert {v[0] for v in tight.violations(hp)} == {
        "member_force_ratio", "lash_toe_deg", "twist_toe_deg"}


def test_lash_and_twist_bounds_refuse_without_their_inputs():
    spec = ek.ElastoSpec(Fy=-1500.0, Fz=1300.0)
    with pytest.raises(ValueError):
        ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("lash_toe_deg", hi=0.1)],
                                elasto=spec)
    with pytest.raises(ValueError):
        ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("twist_toe_deg", hi=0.1)],
                                elasto=spec)


def test_member_force_ratio_includes_the_pushrod(hp):
    r = ek.solve_elastokinematic(hp, ek.ElastoSpec(Fz=3000.0), jacobian=False)
    assert "PR" in r.all_member_forces and "PR" not in r.member_forces


# --------------------------------------------------------------------------- #
#  Target chain: tires in series, aero heave
# --------------------------------------------------------------------------- #
def test_rigid_tire_default_leaves_every_target_unchanged():
    v = td.Vehicle()
    assert td.tire_roll_deg(v, 1.5) == 0.0
    assert td.roll_to_road(v, 1.5) == td.roll(v, 1.5)


def test_tire_roll_matches_closed_form():
    v = replace(td.Vehicle(), tire_rate_n_per_mm=120.0)
    M = 300.0 * 9.81 * 1.5 * 0.280
    assert td.tire_roll_deg(v, 1.5) == pytest.approx(
        np.degrees(M / (120e3 * 1.21 ** 2)), rel=1e-12)
    # moves the static camber target by exactly that much, no gain change
    t = td.Tire()
    d = (td.static_camber_needed(v, t, -0.037)
         - td.static_camber_needed(td.Vehicle(), t, -0.037))
    assert d == pytest.approx(-td.tire_roll_deg(v, 1.5))


def test_series_share_moves_toward_half():
    r = td.series_roll_stiffness(454.0, 405.0, 120.0, 1210.0)
    assert 0.5 < r["share_series"] < r["share_rigid"]
    assert r["total"] < 454.0 + 405.0


def test_aero_heave_comes_out_of_the_budget():
    assert td.aero_heave_mm(2.6, 60.0, 14.848) == pytest.approx(7.448, abs=1e-3)
    v = td.Vehicle()
    a = td.design_accelerations(td.Tire())["combined"]
    f0 = td.f_min(v, 61.2, a, a, "front")
    f1 = td.f_min(replace(v, aero_heave_mm=5.0), 61.2, a, a, "front")
    assert f1 > f0


# --------------------------------------------------------------------------- #
#  Tolerance field: correlated clusters, systematic offsets
# --------------------------------------------------------------------------- #
def test_field_without_clusters_samples_as_before():
    a = ToleranceField.preset("jig_weld").sample(500, seed=4)
    f = ToleranceField.preset("jig_weld")
    assert np.array_equal(a, f.sample(500, seed=4))
    assert "clusters" not in gr.field_to_dict(f)


def test_cluster_is_correlated_and_round_trips():
    f = ToleranceField.preset("jig_weld")
    f.clusters = [ToleranceCluster(("upper_front_inner", "upper_rear_inner"),
                                   ToleranceSpec.symmetric(1.0))]
    f.__post_init__()
    s = f.sample(20000, seed=1)
    o = sorted(f.specs)
    i, j = 3 * o.index("upper_front_inner"), 3 * o.index("upper_rear_inner")
    # shared variance 1/3 against own 0.25/3: correlation 0.8
    assert np.corrcoef(s[:, i], s[:, j])[0, 1] == pytest.approx(0.8, abs=0.02)
    g = gr.field_from_dict(gr.field_to_dict(f))
    assert g.clusters[0].points == ("upper_front_inner", "upper_rear_inner")


def test_shifted_field_biases_by_the_offset():
    f = ToleranceField.preset("jig_weld").shifted(
        {"lower_front_inner": [0.4, 0.0, -0.2]})
    o = sorted(f.specs)
    k = 3 * o.index("lower_front_inner")
    assert np.allclose(f.mean_vec()[k:k + 3], [0.4, 0.0, -0.2])


# --------------------------------------------------------------------------- #
#  Tool access
# --------------------------------------------------------------------------- #
def _box_around(p, dx0, dx1):
    p = np.asarray(p, float)
    return ig.KeepOutBox(lo=p + [dx0, -30, -30], hi=p + [dx1, 30, 30])


def test_tool_access_needs_only_one_clear_end(hp):
    ta = ig.ToolAccess(points=("lower_front_inner",))
    front = _box_around(hp.lower_front_inner, -80, -40)   # forward of the bolt
    assert ta.clearance(hp, [front])["lower_front_inner"] > 0.0
    rear = _box_around(hp.lower_rear_inner, 40, 80)       # behind the partner
    assert ta.clearance(hp, [front, rear])["lower_front_inner"] < 0.0


def test_tool_access_is_a_wall_and_round_trips(hp):
    blocks = [_box_around(hp.lower_front_inner, -80, -40),
              _box_around(hp.lower_rear_inner, 40, 80)]
    vol = ig.LegalVolume.around(hp, 5.0, points=["upper_front_inner"])
    vol.keep_out = blocks
    vol.tool_access = ig.ToolAccess(points=("lower_front_inner",))
    assert any("bolt access" in v[0] for v in vol.keepout_violations(hp))
    d = gr.volume_to_dict(vol)
    assert gr.volume_from_dict(d).tool_access.points == ("lower_front_inner",)
    vol.tool_access = None
    assert "tool_access" not in gr.volume_to_dict(vol)


# --------------------------------------------------------------------------- #
#  Band sweep
# --------------------------------------------------------------------------- #
def test_band_sweep_reports_declared_first_and_is_deterministic(hp):
    st = np.array([-25.0, 0.0, 25.0])
    truth, ok = ig.curves_of(_perturbed(hp, {
        "upper_front_inner": np.array([0.0, -4.0, 5.0]),
        "upper_rear_inner": np.array([0.0, -4.0, 5.0])}), st)
    assert ok
    tg = ig.GenesisTargets(curves=[
        ig.TargetCurve("camber_deg", st, truth["camber_deg"], np.full(3, 0.15)),
        ig.TargetCurve("toe_deg", st, truth["toe_deg"], np.full(3, 0.08))])
    vol = ig.LegalVolume.around(hp, 8.0, points=["upper_front_inner",
                                                 "upper_rear_inner"])
    a = ig.band_sweep(hp, tg, vol, scales=(0.8, 1.25), channels=["toe_deg"],
                      n_starts=2, n_yield=100)
    b = ig.band_sweep(hp, tg, vol, scales=(0.8, 1.25), channels=["toe_deg"],
                      n_starts=2, n_yield=100)
    assert a[0]["channel"] == "(declared)" and len(a) == 3
    assert a == b
