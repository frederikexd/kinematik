# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
Tests for the dynamic gaps the inverse-synthesis review left: split-mu
transients, contact-patch load variation with the anti bypass, joint
stiction and steering lag, dynamic Ackermann, hydraulic brake bias drift and
joint wear. Each test pins a physical invariant or a limiting case, not an
output a later edit could silently move.
"""
import math

import numpy as np
import pytest

from suspension import contact_patch as cp
from suspension import steering_response as sr
from suspension import dynamic_ackermann as da
from suspension import brake_bias as bb
from suspension import joint_wear as jw
from suspension import transient as tr
from suspension.joints import stiction_describing_function as sdf
from suspension.kinematics import Hardpoints
from suspension.tiremodel import default_tire
from suspension.brake_thermal import BrakeThermalParams


# --------------------------------------------------------------------------- #
#  Split mu
# --------------------------------------------------------------------------- #
def test_uniform_mu_hook_changes_nothing():
    drv, road, t_end, u0, _ = tr.step_steer_maneuver(t_end=1.0)
    a = tr.TransientSolver(None).run(t_end, driver=drv, road=road, u0=u0)
    road2 = tr.RoadInput(mu=lambda t: np.ones(4))
    b = tr.TransientSolver(None).run(t_end, driver=drv, road=road2, u0=u0)
    assert np.allclose(a.r, b.r, atol=1e-9) and np.allclose(a.beta, b.beta, atol=1e-9)


def test_rear_grip_loss_oversteers_front_grip_loss_understeers():
    out = {}
    for ax in ("front", "rear"):
        r = tr.run_maneuver(None, "mu_step", axle=ax, d_mu=-0.15, t_end=2.4,
                            t_step=1.5)
        assert r.ok
        out[ax] = tr.mu_step_response(r, 1.5)["yaw_rate_peak_change_deg_s"]
    assert out["rear"] > 0.0 > out["front"]


# --------------------------------------------------------------------------- #
#  Contact-patch load variation
# --------------------------------------------------------------------------- #
@pytest.fixture
def corner():
    k = 14848.0
    return cp.QuarterCar(60.1, 11.9, k, 2 * 0.3 * math.sqrt(k * 60.1), 120e3)


def test_quarter_car_limits(corner):
    h = corner.load_frf([1e-4, 200.0])
    # at very low frequency the whole corner follows the road: no load change
    assert abs(h[0]) < 1e-2 * corner.k_t
    # far above wheel hop the unsprung mass stays put: dFz -> k_t z_r
    assert abs(h[1]) == pytest.approx(corner.k_t, rel=0.05)


def test_kappa_from_anti_matches_paper_front():
    k = cp.kappa_from_anti(61.2, 280.0, 1630.0, 0.60, 1.5)
    assert k == pytest.approx(0.612 * 280 / (1630 * 0.6) * 1.5)


def test_anti_bypass_raises_load_variation(corner):
    base = cp.load_variation(corner, 15.0, "B")
    corner.kappa = cp.kappa_from_anti(61.2, 280.0, 1630.0, 0.60, 1.5)
    anti = cp.load_variation(corner, 15.0, "B")
    assert anti["stable"] and anti["ratio"] > base["ratio"]


def test_heavier_unsprung_mass_varies_load_more(corner):
    light = cp.load_variation(cp.QuarterCar(60.1, 9.0, corner.k, corner.c, 120e3), 15.0)
    heavy = cp.load_variation(cp.QuarterCar(60.1, 15.0, corner.k, corner.c, 120e3), 15.0)
    assert heavy["ratio"] > light["ratio"]


def test_rougher_road_scales_rms_by_root_of_psd(corner):
    a = cp.load_variation(corner, 15.0, "A")["sigma_dFz_N"]
    b = cp.load_variation(corner, 15.0, "B")["sigma_dFz_N"]
    assert b / a == pytest.approx(2.0, rel=1e-6)          # PSD x4


# --------------------------------------------------------------------------- #
#  Stiction and steering lag
# --------------------------------------------------------------------------- #
def test_stiction_locks_below_breakout_and_fades_at_large_amplitude():
    assert sdf(30000.0, 150.0, 100.0)["locked"]
    small = sdf(30000.0, 150.0, 300.0)
    large = sdf(30000.0, 150.0, 5000.0)
    assert small["phase_lag_deg"] > large["phase_lag_deg"] > 0.0
    assert large["k_eff_n_per_mm"] == pytest.approx(30000.0, rel=0.05)


def test_relaxation_adds_lag_and_vanishing_it_removes_its_share():
    car = sr.SingleTrack(speed_ms=15.0)
    b = sr.lag_budget(car, 2.5)
    assert b["relaxation_ms"] > 0.0
    slow = sr.lag(sr.SingleTrack(speed_ms=15.0, sigma_m=0.9), 2.5)["lag_ms"]
    assert slow > b["total_ms"]


def test_steady_state_gain_matches_the_bicycle_formula():
    car = sr.SingleTrack(speed_ms=15.0)
    L = car.a_m + car.b_m
    K = car.mass_kg / L * (car.b_m / car.cf_n_per_rad - car.a_m / car.cr_n_per_rad)
    ay_per_rad = car.speed_ms ** 2 / (L + K * car.speed_ms ** 2)
    h = sr.lateral_accel_frf(car, [1e-4])[0]
    assert h.real == pytest.approx(ay_per_rad, rel=1e-3)


# --------------------------------------------------------------------------- #
#  Dynamic Ackermann
# --------------------------------------------------------------------------- #
def test_equal_loads_give_geometric_ackermann():
    t = default_tire()
    case = da.AxleCase(700.0, 700.0)
    assert da.dynamic_target(t, case)["dynamic_ackermann_pct"] == pytest.approx(100.0, abs=0.5)


def test_load_transfer_pulls_target_below_geometric_and_optimum_costs_nothing():
    t = default_tire()
    case = da.AxleCase.from_vehicle()
    tg = da.dynamic_target(t, case)
    assert tg["peak_slip_outer_deg"] > tg["peak_slip_inner_deg"]
    assert tg["dynamic_ackermann_pct"] < 100.0
    at_opt = da.axle_force_at_split(t, case, tg["dynamic_difference_deg"])
    at_geo = da.axle_force_at_split(t, case, tg["geometric_difference_deg"])
    assert at_opt["loss_pct"] == pytest.approx(0.0, abs=0.01)
    assert at_geo["loss_pct"] > at_opt["loss_pct"]


def test_kinematic_split_is_symmetric_in_rack_direction():
    hp = Hardpoints.default()
    a = da.kinematic_split(hp, 20.0)
    b = da.kinematic_split(hp, -20.0)
    assert a["difference_deg"] == pytest.approx(b["difference_deg"], abs=1e-9)
    assert da.ackermann_pct(22.709, 17.708, 1630.0, 1210.0) == pytest.approx(100.0, abs=0.2)


# --------------------------------------------------------------------------- #
#  Brake bias
# --------------------------------------------------------------------------- #
def test_bias_is_independent_of_pedal_force_and_hits_target():
    s = bb.BrakeHydraulics()
    s = bb.BrakeHydraulics(**{**s.__dict__, "bar_front": s.bar_for_bias(0.60)})
    assert s.bias() == pytest.approx(0.60, abs=1e-6)
    tf1, tr1 = s.axle_torques_Nm(500.0); tf2, tr2 = s.axle_torques_Nm(1000.0)
    assert tf1 / (tf1 + tr1) == pytest.approx(tf2 / (tf2 + tr2))


def test_front_fade_moves_bias_rearward_and_anti_dive_with_it():
    fade = BrakeThermalParams(enable_fade=True)
    s = bb.BrakeHydraulics(fade_front=fade, fade_rear=BrakeThermalParams(enable_fade=True))
    s = bb.BrakeHydraulics(**{**s.__dict__, "bar_front": s.bar_for_bias(0.60)})
    d = bb.bias_drift(s, [300.0, 600.0], [200.0, 400.0])
    assert d["cold"] == pytest.approx(0.60, abs=1e-6) and d["min"] < 0.60
    lo, hi = bb.anti_dive_window(61.2, 0.60, 51.0, 70.0)
    assert lo == pytest.approx(0.50, abs=1e-9) and hi == pytest.approx(0.6863, abs=1e-4)


# --------------------------------------------------------------------------- #
#  Joint wear
# --------------------------------------------------------------------------- #
def test_wear_grows_clearance_linearly_and_fit_recovers_rate():
    s = jw.JointWearSpec("TR", duty=[jw.JointDuty(500.0, 10.0, 400.0)])
    c = s.clearance_at_km([0.0, 100.0, 200.0])
    assert c[2] - c[1] == pytest.approx(c[1] - c[0])
    K = jw.fit_wear_rate(s, [100.0, 300.0], s.clearance_at_km([100.0, 300.0]))
    assert K == pytest.approx(s.wear_rate_mm3_per_Nm, rel=1e-9)


def test_km_to_budget_scales_inversely_with_wear_rate():
    hp = Hardpoints.default()
    duty = [jw.JointDuty(500.0, 10.0, 400.0)]
    a = jw.km_to_budget(hp, [jw.JointWearSpec("TR", duty=duty)], "toe", 0.04)
    b = jw.km_to_budget(hp, [jw.JointWearSpec("TR", duty=duty,
                                              wear_rate_mm3_per_Nm=1e-6)], "toe", 0.04)
    assert a == pytest.approx(2.0 * b, rel=1e-6)
