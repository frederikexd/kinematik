# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
Tests for the last four gaps: halfshaft secondary loads, bracket and tube
permanent set, assembly sequence and damper synthesis. Invariants and closed
forms, not snapshots.
"""
import math

import numpy as np
import pytest

from suspension import assembly_plan as ap
from suspension import contact_patch as cp
from suspension import damper_synthesis as ds
from suspension import genesis_repro as gr
from suspension import halfshaft as hs
from suspension import loadpath as lp
from suspension import permanent_set as ps
from suspension.kinematics import Hardpoints, SuspensionKinematics


@pytest.fixture(scope="module")
def solved():
    hp = Hardpoints.default()
    kin = SuspensionKinematics(hp)
    return hp, kin, kin.solve_at_travel(0.0)


# --------------------------------------------------------------------------- #
#  Halfshaft
# --------------------------------------------------------------------------- #
def test_plunge_and_couple_follow_their_formulas(solved):
    _, _, st = solved
    spec = hs.HalfshaftSpec((0.0, 120.0, 240.0), tripod_pitch_radius_mm=20.0,
                            mu_plunge=0.03)
    r = hs.halfshaft_loads(spec, st, 300.0)
    assert r["plunge_force_N"] == pytest.approx(0.03 * 300e3 / 20.0)
    assert r["couple_Nmm"] == pytest.approx(
        300e3 * math.tan(math.radians(r["shaft_angle_deg"]) / 2.0))
    a, b = (np.asarray(c["F_wc_extra"]) for c in r["cases"])
    assert np.allclose(a, -b)


def test_hub_extras_keep_equilibrium_and_default_is_unchanged(solved):
    _, kin, st = solved
    base = lp.solve_member_forces(kin, st, lp.WheelLoad(Fz=1500.0, Fx_wc=-1500.0))
    same = lp.solve_member_forces(kin, st, lp.WheelLoad(
        Fz=1500.0, Fx_wc=-1500.0, F_wc_extra=(0, 0, 0), M_wc_extra=(0, 0, 0)))
    assert base.forces == same.forces
    ex = lp.solve_member_forces(kin, st, lp.WheelLoad(
        Fz=1500.0, Fx_wc=-1500.0, F_wc_extra=(0.0, 300.0, 0.0),
        M_wc_extra=(0.0, 0.0, 5000.0)))
    assert ex.residual < 1e-6 and ex.forces != base.forces


def test_screen_splits_each_driven_case_by_plunge_direction(solved):
    hp, kin, st = solved
    cases = gr.vehicle_load_cases(axle="rear", traction_g=1.06)
    out = gr.with_halfshaft(cases, hs.HalfshaftSpec((0.0, 120.0, 240.0)), st)
    assert len(out) == len(cases) + 2
    assert sum("plunge" in c.name for c in out) == 4


# --------------------------------------------------------------------------- #
#  Permanent set
# --------------------------------------------------------------------------- #
E, SY = 205000.0, 435.0


def test_rect_permanent_curvature_matches_closed_form():
    sec = ps.Section("rect", b_mm=40.0, h_mm=6.0, n=160)
    My = SY * 40.0 * 36.0 / 6.0
    for m in (1.1, 1.3):
        r = ps.permanent_curvature(sec, E, SY, m * My)
        ky = 2.0 * SY / (E * 6.0)
        exact = ky / math.sqrt(3.0 - 2.0 * m) - m * My / (E * sec.I_mm4())
        assert r["kappa_perm"] == pytest.approx(exact, rel=0.01)


def test_no_set_below_yield_and_collapse_past_plastic_moment():
    sec = ps.Section("rect", b_mm=40.0, h_mm=6.0, n=80)
    My = SY * 40.0 * 36.0 / 6.0
    assert ps.cantilever_set(sec, E, SY, 30.0, 0.95 * My / 30.0, n_seg=10)[
        "permanent_tip_mm"] == 0.0
    assert ps.cantilever_set(sec, E, SY, 30.0, 1.6 * My / 30.0, n_seg=10)["collapsed"]


def test_weld_residual_stress_gives_shakedown_set_below_nominal_yield():
    sec = ps.Section("rect", b_mm=40.0, h_mm=6.0, n=80)
    My = SY * 40.0 * 36.0 / 6.0
    res = lambda y: 0.6 * SY * (2.0 * y / 6.0) ** 2
    r = ps.cantilever_set(sec, E, SY, 30.0, 0.9 * My / 30.0, residual=res, n_seg=10)
    assert r["yielded"] and r["permanent_tip_mm"] > 0.0


def test_tube_section_inertia_matches_formula():
    sec = ps.Section("tube", od_mm=25.4, t_mm=1.65, n=2000)
    ro, ri = 12.7, 12.7 - 1.65
    assert sec.I_mm4() == pytest.approx(math.pi / 4.0 * (ro ** 4 - ri ** 4), rel=0.01)


def test_channel_drift_is_linear_in_offset(solved):
    hp, _, _ = solved
    a = ps.channel_drift(hp, "lower_front_inner", [0.0, 0.0, 0.1])
    b = ps.channel_drift(hp, "lower_front_inner", [0.0, 0.0, 0.2])
    assert b["toe"] == pytest.approx(2.0 * a["toe"])


# --------------------------------------------------------------------------- #
#  Assembly sequence
# --------------------------------------------------------------------------- #
def _cell(diff_in_the_way=True):
    bolt = ap.Part("bolt", [((-20, 0, 0), (20, 0, 0), 4)],
                   directions=[(1, 0, 0), (-1, 0, 0)], travel_mm=60)
    clevis = ap.Part("clevis", [((-10, 0, -15), (-10, 0, 15), 3),
                                ((10, 0, -15), (10, 0, 15), 3)], fixed=True)
    tube = ap.Part("frame tube", [((-40, -100, 0), (-40, 100, 0), 12.7)], fixed=True)
    x = 45 if diff_in_the_way else 300
    diff = ap.Part("differential", [((x, -30, 0), (x, 30, 0), 20)],
                   directions=[(0, 0, 1)], travel_mm=200)
    return [bolt, clevis, diff, tube]


def test_bolt_slides_out_of_its_own_clevis():
    parts = _cell(diff_in_the_way=False)
    assert ap.blockers(parts[0], parts, (1, 0, 0)) == []
    assert ap.parts_to_free(parts, "bolt") == []


def test_boxed_in_bolt_needs_the_differential_out_first():
    parts = _cell()
    assert ap.parts_to_free(parts, "bolt") == ["differential"]
    d = ap.disassembly(parts)
    assert d["ok"] and d["order"].index("differential") < d["order"].index("bolt")
    assert d["assembly_order"] == list(reversed(d["order"]))


def test_deadlock_is_named():
    a = ap.Part("a", [((0, 0, 0), (10, 0, 0), 3)], directions=[(0, 0, 1)])
    b = ap.Part("b", [((0, 0, 8), (10, 0, 8), 3)], directions=[(0, 0, -1)])
    d = ap.disassembly([a, b])
    assert not d["ok"] and d["deadlock"]["a"] == ["b"]


# --------------------------------------------------------------------------- #
#  Damper synthesis
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def front():
    return cp.QuarterCar(60.1, 11.9, 14848.0, 0.0, 120e3)


def test_load_variation_has_an_interior_damping_optimum(front):
    z = ds.grip_optimal_zeta(front)
    sweep = ds.zeta_sweep(front, [0.1, z, 1.2])
    assert 0.1 < z < 1.2
    assert sweep[1]["ratio"] < sweep[0]["ratio"] and sweep[1]["ratio"] < sweep[2]["ratio"]


def test_synthesised_curve_refers_rates_to_the_shaft(front):
    r = ds.synthesize(front, motion_ratio=0.8)
    c = r["curve"]
    assert c.c_bump_low == pytest.approx(r["c_wheel_low"] / 0.64)
    assert c.c_reb_low == pytest.approx(1.5 * c.c_bump_low)
    assert not c.is_calibrated and c.v_knee > 0.0


def test_front_rate_for_share_inverts_the_share():
    cf = ds.front_rate_for_share(0.53, 1500.0, 1210.0, 1180.0)
    assert ds.transient_front_share(cf, 1500.0, 1210.0, 1180.0) == pytest.approx(0.53)


# --------------------------------------------------------------------------- #
#  Steering feel, hot inflation, harshness, radiant gradient, clevis
# --------------------------------------------------------------------------- #
from suspension import steering_feel as sf
from suspension import strain_thermal as sth
from suspension import target_derivation as td
from suspension.joints import clevis_pin_tilt_deg, rod_end_shank_bending
from suspension.tiremodel import default_tire


def test_torque_at_limit_is_mechanical_trail_alone():
    t = default_tire()
    r = sf.limit_feel(t, 1248.0, 14.6)
    from suspension.dynamic_ackermann import peak_slip_deg
    fy = abs(float(t.fy(math.radians(peak_slip_deg(t, 1248.0)), 1248.0)))
    assert r["torque_at_limit_Nm"] == pytest.approx(fy * 14.6 / 1000.0, rel=1e-3)


def test_less_caster_lightens_more_and_leaves_less_torque():
    t = default_tire()
    hi, lo = sf.limit_feel(t, 1248.0, 14.6), sf.limit_feel(t, 1248.0, 4.4)
    assert lo["torque_at_limit_Nm"] < hi["torque_at_limit_Nm"]
    assert lo["lightening"] > hi["lightening"]
    assert 0.0 < hi["alpha_torque_max_deg"] < hi["alpha_peak_deg"]


def test_hot_pressure_and_rate():
    p = td.hot_pressure_kpa(83.0, 25.0, 25.0)
    assert p == pytest.approx(83.0)
    ph = td.hot_pressure_kpa(83.0, 25.0, 70.0)
    assert ph > 83.0
    assert td.tire_rate_at_pressure(120.0, 83.0, 83.0) == pytest.approx(120.0)
    assert td.tire_rate_at_pressure(120.0, 83.0, ph, carcass_share=1.0) == pytest.approx(120.0)


def test_anti_path_adds_body_harshness():
    k = 14848.0
    c = 2 * 0.3 * math.sqrt(k * 60.1)
    a = cp.body_accel_rms(cp.QuarterCar(60.1, 11.9, k, c, 120e3), 15.0)
    b = cp.body_accel_rms(cp.QuarterCar(60.1, 11.9, k, c, 120e3, kappa=0.26), 15.0)
    assert b > a > 0.0


def test_ring_gradient_scales_with_flux_and_three_sensors_resolve_it():
    g1 = sth.ring_gradient(500.0, 25.4, 1.65)["delta_T_c"]
    g2 = sth.ring_gradient(1000.0, 25.4, 1.65)["delta_T_c"]
    assert g2 == pytest.approx(2.0 * g1)
    dT, phi = 1.4, 73.0
    T = [dT / 2 * math.cos(math.radians(a - phi)) for a in (0.0, 120.0, 240.0)]
    r = sth.first_harmonic_from_three(*T)
    assert r["delta_T_c"] == pytest.approx(dT) and r["direction_deg"] == pytest.approx(phi)
    assert sth.two_sensor_reading(dT, 60.0) == pytest.approx(dT / 2.0)
    assert 0.0 < sth.shield_factor() < 0.1


def test_clevis_tilt_and_shank_bending():
    assert clevis_pin_tilt_deg(4799.0, 35000.0, 35000.0, 20.0) == pytest.approx(0.0)
    assert clevis_pin_tilt_deg(4799.0, 35000.0, 43000.0, 20.0) > 0.0
    r = rod_end_shank_bending(4799.0, 7.9, 6.6, mu_ball=0.1)
    assert r["stress_MPa"] == pytest.approx(0.1 * 4799 * 7.9 / (math.pi * 6.6 ** 3 / 32))
    assert rod_end_shank_bending(4799.0, 7.9, 6.6, pin_tilt_deg=14.0)["binds"]
