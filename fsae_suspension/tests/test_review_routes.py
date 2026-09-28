# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Design routes for the trade-offs the review left open: steering ratio
window at full caster, inboard-brake anti-lift, rocker bearing rating against
spacing, and rod-end shank fatigue."""
import math

import numpy as np
import pytest

from suspension import steering_feel as sf
from suspension import actuation as act
from suspension import genesis_repro as gr
from suspension import inverse_genesis as ig
from suspension.joints import shank_fatigue_check, thread_endurance_amplitude_mpa
from suspension.kinematics import Hardpoints, SuspensionKinematics
from suspension.tiremodel import default_tire


def test_ratio_window_at_full_caster_and_its_trail_limit():
    t = default_tire()
    w = sf.ratio_window(t, 1247.7, 164.9, 14.6)
    assert w["feasible"] and 4.0 < w["ratio_min"] < 6.0
    assert w["worst_alpha_deg"] < 8.5          # torque peaks before the force
    lim = sf.trail_limit_mm(t, 1247.7, 164.9, 14.6)
    assert sf.ratio_window(t, 1247.7, 164.9, 14.6, t0_outer_mm=lim - 0.5)["feasible"]
    assert not sf.ratio_window(t, 1247.7, 164.9, 14.6, t0_outer_mm=lim + 0.5)["feasible"]


def test_inboard_brake_anti_lift_shares_anti_squat_reference():
    k = SuspensionKinematics(Hardpoints.default())
    sq = k.anti_squat_pct(280.0, 1630.0, 1.0)
    li = k.anti_lift_pct(280.0, 1630.0, 0.40, inboard_brakes=True)
    assert li == pytest.approx(0.40 * sq, rel=1e-12)
    assert k.anti_lift_pct(280.0, 1630.0, 0.40) != pytest.approx(li)


def test_property_bound_context_uses_inboard_reference():
    hp = Hardpoints.default()
    out = ig.properties_of(hp, ig.SolvedPropertyBounds(
        bounds=[ig.PropertyBound("anti_lift_pct", lo=-500.0)],
        rear_brakes_inboard=True))
    k = SuspensionKinematics(hp)
    assert out["anti_lift_pct"] == pytest.approx(
        k.anti_lift_pct(280.0, 1630.0, 0.40, state=k.solve_at_travel(0.0),
                        inboard_brakes=True), rel=1e-9)


def test_required_bearing_rating_falls_with_spacing_when_offset():
    import copy
    hp = Hardpoints.default()
    kin = SuspensionKinematics(hp)
    if not kin.motion_ratio_is_real():
        pytest.skip("reference corner has no rocker")
    cases = gr.vehicle_load_cases()
    flat = [x["required_C0_N"] for x in
            act.required_rating_vs_spacing(hp, cases, [30.0, 60.0])]
    assert flat[0] == pytest.approx(flat[1])      # in-plane: no couple
    off = copy.deepcopy(hp)
    k = np.asarray(kin._rocker_axis, float)
    off.rocker_pushrod = np.asarray(hp.rocker_pushrod, float) + 45.0 * k
    r = [x["required_C0_N"] for x in
         act.required_rating_vs_spacing(off, cases, [20.0, 30.0, 60.0])]
    assert r[0] > r[1] > r[2] > flat[0]


def test_vdi_endurance_and_shank_check():
    assert thread_endurance_amplitude_mpa(10.0) == pytest.approx(0.85 * (15.0 + 45.0))
    small = shank_fatigue_check(4799.0, 8.0, 7.9375, 24, 0.15)
    big = shank_fatigue_check(4799.0, 8.0, 9.525, 24, 0.05)
    assert small["root_dia_mm"] == pytest.approx(7.9375 - 1.226869 * 25.4 / 24)
    assert small["fos"] < 1.0 < big["fos"]


# --------------------------------------------------------------------------- #
#  Closures: the routes as enforced checks
# --------------------------------------------------------------------------- #
from suspension import halfshaft as hs
from suspension import calibration as cal


def test_mechanical_trail_matches_kingpin_ground_intercept():
    hp = Hardpoints.default()
    st = SuspensionKinematics(hp).solve_at_travel(0.0)
    lo, up, cp = (np.asarray(v, float) for v in (st.lower_outer, st.upper_outer,
                                                  st.contact_patch))
    t = (cp[2] - lo[2]) / (up[2] - lo[2])
    assert ig.mechanical_trail_mm(st) == pytest.approx(cp[0] - (lo + t * (up - lo))[0])


def test_steer_ratio_bound_is_a_wall_and_needs_its_spec():
    hp = Hardpoints.default()
    se = ig.SteerEffort()
    val = ig.properties_of(hp, ig.SolvedPropertyBounds(
        bounds=[ig.PropertyBound("steer_ratio_min", hi=99.0)], steer_effort=se))
    tight = ig.SolvedPropertyBounds(
        bounds=[ig.PropertyBound("steer_ratio_min", hi=0.5 * val["steer_ratio_min"])],
        steer_effort=se)
    assert tight.violations(hp)
    with pytest.raises(ValueError):
        ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("steer_ratio_min", hi=6.0)])


def test_new_context_fields_round_trip_and_keep_old_hashes():
    ctx = ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("caster_deg", lo=0.0)])
    d = gr._property_bounds_to_dict(ctx)
    assert "rear_brakes_inboard" not in d and "steer_effort" not in d
    ctx2 = ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("caster_deg", lo=0.0)],
                                   rear_brakes_inboard=True,
                                   steer_effort=ig.SteerEffort(t0_outer_mm=20.0))
    back = gr._property_bounds_from_dict(gr._property_bounds_to_dict(ctx2))
    assert back.rear_brakes_inboard and back.steer_effort.t0_outer_mm == 20.0


def test_inboard_brakes_reverse_the_halfshaft_duty():
    t = hs.shaft_torques_nm()
    assert t["brake_Nm"] < t["drive_Nm"]
    assert t["reversing_range_Nm"] == pytest.approx(t["drive_Nm"] + t["brake_Nm"])
    assert hs.shaft_shear_mpa(100.0, 20.0) == pytest.approx(
        100e3 * 10.0 / (math.pi * 20.0 ** 4 / 32.0))


def test_shank_fatigue_reported_by_the_screen():
    hp = Hardpoints.default()
    r = gr.structural_screening(hp, rod_end={"nominal_dia_mm": 7.9375,
                                             "threads_per_inch": 24,
                                             "ball_radius_mm": 8.0, "mu_ball": 0.15})
    assert set(r["shank_fatigue"]) == set(r["worst_fos_per_member"])
    assert "shank_fatigue" not in gr.structural_screening(hp)


def test_bearing_selection_ranks_candidates():
    hp = Hardpoints.default()
    if not SuspensionKinematics(hp).motion_ratio_is_real():
        pytest.skip("reference corner has no rocker")
    cands = [act.RockerBearing(label="small", C0_N=2360.0),
             act.RockerBearing(label="big", C0_N=20000.0)]
    r = act.select_rocker_bearing(hp, gr.vehicle_load_cases(), cands)
    assert r[0]["label"] == "big" and r[0]["s0"] > r[1]["s0"]


def test_calibration_recovers_a_known_transducer():
    rng = np.random.default_rng(1)
    loads = np.linspace(0.0, 5000.0, 6)
    F, V, D, R = [], [], [], []
    for run in range(3):
        for d, seq in ((1, loads), (-1, loads[::-1])):
            for L in seq:
                F.append(L); D.append(d); R.append(run)
                V.append(0.02 + 2e-4 * L + (1e-4 if d < 0 and 0 < L < 5000 else 0.0)
                         + rng.normal(0, 1e-5))
    c = cal.calibrate(F, V, D, R, full_scale_n=5000.0)
    assert c["sensitivity_mvv_per_n"] == pytest.approx(2e-4, rel=0.01)
    assert c["hysteresis_pct_fs"] > c["repeatability_pct_fs"]
    b = cal.bending_rejection(apparent_axial_n=10.0, full_scale_n=5000.0,
                              bending_strain_ue=189.0, full_scale_strain_ue=559.0)
    assert b["required_ratio"] == pytest.approx(189.0 / 559.0 / 0.005)
    assert b["passes"]
    assert cal.temperature_coeff([20, 30, 40], [0.0, 1.0, 2.0])["coeff_n_per_c"] == pytest.approx(0.1)
    assert cal.installation_offset(120.0, 100.0, 15.0)["significant"]


# --------------------------------------------------------------------------- #
#  Steering walls in the loop; bridge thermal cancellation
# --------------------------------------------------------------------------- #
from suspension import strain_thermal as sth


def test_kinematic_ratio_and_lock_follow_the_rack():
    hp = Hardpoints.default()
    a, b = ig.SteerEffort(rack_c_mm_per_rev=76.2), ig.SteerEffort(rack_c_mm_per_rev=91.0)
    assert a.kinematic_ratio(hp) == pytest.approx(b.kinematic_ratio(hp) * 91.0 / 76.2, rel=1e-9)
    assert a.lock_sw_deg(hp) == pytest.approx(b.lock_sw_deg(hp) * 91.0 / 76.2, rel=1e-6)
    assert ig.SteerEffort(lock_road_wheel_deg=89.0).lock_sw_deg(hp) == float("inf")


def test_steering_walls_refuse_instead_of_returning_an_illegal_front():
    from suspension.kinematik_stochastic import _perturbed
    hp = Hardpoints.default()
    se = ig.SteerEffort(rack_c_mm_per_rev=91.0, effort_limit_nm=7.45)
    wc = np.asarray(hp.wheel_center); arm = np.asarray(hp.tie_rod_outer) - wc
    arm[1] = 0.0; arm /= np.linalg.norm(arm)
    truth = _perturbed(hp, {"tie_rod_outer": 12.0 * arm,
                            "tie_rod_inner": np.array([12.0 * arm[0], 0.0, 0.0])})
    st = np.array([-25.0, 0.0, 25.0]); cur, _ = ig.curves_of(truth, st)
    tg = ig.GenesisTargets(curves=[
        ig.TargetCurve("toe_deg", st, cur["toe_deg"], np.full(3, 0.05)),
        ig.TargetCurve("camber_deg", st, cur["camber_deg"], np.full(3, 0.15))])
    walls = ig.SolvedPropertyBounds(bounds=[
        ig.PropertyBound("caster_deg", lo=3.67),
        ig.PropertyBound("steer_effort_margin", lo=0.0),
        ig.PropertyBound("lock_sw_deg", hi=120.0)], steer_effort=se)
    free = ig.LegalVolume.around(hp, 15.0, points=["tie_rod_outer", "tie_rod_inner"])
    rf = ig.inverse_genesis(hp, tg, free, n_starts=2, n_yield=50)
    ctx = ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("caster_deg", lo=0.0)],
                                  steer_effort=se)
    margin = ig.properties_of(rf.winner_hp, ctx, only=("steer_effort_margin",))
    assert margin["steer_effort_margin"] < 0.0        # unwalled: fails effort
    walled = ig.LegalVolume.around(hp, 15.0, points=["tie_rod_outer", "tie_rod_inner"])
    walled.properties = walls
    rw = ig.inverse_genesis(hp, tg, walled, n_starts=2, n_yield=50)
    if rw.winner_hp is not None:
        assert not walls.violations(rw.winner_hp)
    else:
        assert "solved-property bound refused" in rw.reason


def test_poisson_bridge_cancels_antisymmetric_heat():
    assert sth.poisson_bridge_apparent_ue((0.5, -0.5), (0.0, 0.0)) == pytest.approx(0.0)
    assert sth.poisson_bridge_apparent_ue((1.0, 1.0), (1.0, 1.0)) == pytest.approx(0.0)
    q = sth.radiant_flux(400.0, 40.0, 0.1)
    beside = sth.bridge_thermal_error(q, 15.875, 0.889, transverse_at_deg=0.0)
    quarter = sth.bridge_thermal_error(q, 15.875, 0.889, transverse_at_deg=90.0)
    assert beside["apparent_ue"] == pytest.approx(0.0, abs=1e-12)
    assert 0.0 < abs(quarter["apparent_ue"]) < 0.1 * 1.8 * beside["side_to_side_c"]


# --------------------------------------------------------------------------- #
#  Tire vertical load: every term
# --------------------------------------------------------------------------- #
from suspension import target_derivation as td


def test_corner_loads_sum_to_the_normal_force_and_match_the_paper():
    v = td.Vehicle()
    r = td.corner_loads(v, a_lat_g=1.5)
    assert r["check_sum_N"] == pytest.approx(300.0 * 9.81)
    assert r["total"]["FR"] == pytest.approx(1247.7, abs=0.5)      # paper's outer front
    a = td.corner_loads(v, a_lat_g=1.5, downforce_n=400.0)
    assert a["check_sum_N"] == pytest.approx(300.0 * 9.81 + 400.0)


def test_bank_grade_and_warp_terms():
    v = td.Vehicle()
    flat = td.corner_loads(v)["total"]
    up = td.corner_loads(v, grade_deg=5.0)["total"]
    assert up["RL"] > flat["RL"] and up["FL"] < flat["FL"]           # uphill loads the rear
    b = td.corner_loads(v, a_lat_g=1.0, bank_deg=3.0)
    assert b["normal_N"] > 300.0 * 9.81 * math.cos(math.radians(3.0))
    w = td.corner_loads(v, steer_warp_n=20.0)["total"]     # left steer loads FL and RR
    assert w["FL"] - flat["FL"] == pytest.approx(20.0)
    assert w["FR"] - flat["FR"] == pytest.approx(-20.0)
    assert sum(w.values()) == pytest.approx(sum(flat.values()))


def test_steer_warp_is_antisymmetric_and_zero_at_centre():
    hp = Hardpoints.default()
    z = td.steer_warp(hp, 0.0, 454.0, 405.0, 1210.0, 1210.0)
    assert z["front_pair_N"] == pytest.approx(0.0, abs=1e-9)
    a = td.steer_warp(hp, 10.0, 454.0, 405.0, 1210.0, 1210.0)
    b = td.steer_warp(hp, 20.0, 454.0, 405.0, 1210.0, 1210.0)
    assert abs(b["front_pair_N"]) > abs(a["front_pair_N"]) > 0.0


def test_steering_left_at_standstill_unloads_the_right_front():
    """Rouelle's question: turn left at 0 km/h, what happens to the RF load?"""
    from suspension.kinematics import SuspensionKinematics
    hp = Hardpoints.default()
    s0 = SuspensionKinematics(hp).solve_at_travel(0.0)
    s1 = SuspensionKinematics(hp, pickup_deltas={"tie_rod_inner": np.array([0.0, 10.0, 0.0])}).solve_at_travel(0.0)
    assert s1.toe < s0.toe                          # +rack toes the right wheel in: a left steer
    w = td.steer_warp(hp, 10.0, 454.0, 405.0, 1210.0, 1210.0)
    assert w["dz_right_mm"] > 0 > w["dz_left_mm"]   # outside patch rises, inside drops
    ls = w["left_steer_N"]
    assert ls["FR"] < 0 < ls["FL"] and ls["RR"] > 0 > ls["RL"]
    v = td.Vehicle()
    base = td.corner_loads(v, a_lat_g=1.5)["total"]
    into = td.corner_loads(v, a_lat_g=1.5, steer_warp_n=ls["FL"])["total"]
    assert into["FR"] < base["FR"]                  # steering into the turn unloads the outer front
