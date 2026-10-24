# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""suspension/brake_thermal.py against energy balances: the endurance equilibrium and its inverse (the duct gain)
agree, the equilibrium satisfies its own heat balance, pad fade, the fade-test and lap power series carry the kinetic
energy they claim, lap braking energy, pad wear, and the rotor optimiser's best and Pareto front by brute force.
SI units (W, J, kg, m/s) with temperatures in degrees C and rotor dimensions in mm."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import brake_thermal as bt                                    # noqa: E402

SIGMA = 5.670374419e-8


# --------------------------------------------------------------------------- #
#  Equilibrium and its inverse
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("calibrated", [False, True])
def test_the_duct_gain_brings_the_equilibrium_to_its_target(calibrated):
    """required_duct_gain inverts equilibrium: a duct of the returned gain makes the rotor settle at the target,
    and with no duct the unducted equilibrium needs a gain of exactly one."""
    p = bt.BrakeThermalParams(calibrated=calibrated)
    m = bt.BrakeThermalModel(p)
    eq = m.equilibrium(p_brake_avg_w=2000.0, v_work_ms=14.0, diameter_mm=220.0)
    r = bt.required_duct_gain(p_brake_avg_w=2000.0, v_work_ms=14.0, diameter_mm=220.0, T_target_c=eq.T_equilibrium_c,
                              params=p)
    assert r["required_duct_gain"] == pytest.approx(1.0, rel=1e-6) and r["achievable_unducted"]
    r2 = bt.required_duct_gain(p_brake_avg_w=2000.0, v_work_ms=14.0, diameter_mm=220.0, T_target_c=300.0, params=p)
    eq2 = m.equilibrium(p_brake_avg_w=2000.0, v_work_ms=14.0, diameter_mm=220.0,
                        duct_gain=r2["required_duct_gain"])
    assert eq2.T_equilibrium_c == pytest.approx(300.0, abs=1e-3) and not r2["achievable_unducted"]


def test_the_convection_is_the_rotating_disc_correlation_unless_calibrated():
    p = bt.BrakeThermalParams()
    assert bt.convection_h(p, 220.0, 14.0) == pytest.approx(bt.h_rotating_disc(220.0, 14.0))
    cal = bt.BrakeThermalParams(calibrated=True, h_air0=25.0, h_air1=3.0)
    assert bt.convection_h(cal, 220.0, 14.0, duct_gain=2.0) == pytest.approx(2.0 * (25.0 + 3.0 * 14.0))
    assert cal.h_air(-14.0) == cal.h_air(14.0)


def test_the_equilibrium_balances_convection_and_radiation():
    p = bt.BrakeThermalParams()
    eq = bt.BrakeThermalModel(p).equilibrium(p_brake_avg_w=2500.0, v_work_ms=12.0, diameter_mm=200.0)
    T, Ta = eq.T_equilibrium_c + 273.15, p.T_ambient_c + 273.15
    out = eq.h_air * eq.area_m2 * (T - Ta) + p.emissivity * SIGMA * eq.area_m2 * (T ** 4 - Ta ** 4)
    assert out == pytest.approx(2500.0, rel=1e-6)
    no_rad = bt.BrakeThermalModel(bt.BrakeThermalParams(enable_radiation=False)).equilibrium(
        p_brake_avg_w=2500.0, v_work_ms=12.0, diameter_mm=200.0)
    assert no_rad.T_equilibrium_c == pytest.approx(p.T_ambient_c + 2500.0 / (no_rad.h_air * no_rad.area_m2))
    assert no_rad.T_equilibrium_c > eq.T_equilibrium_c and eq.synthesized


@pytest.mark.parametrize("power", [1.0, 2.0e3, 1.0e6, 1.0e15])
def test_the_equilibrium_balances_at_any_power(power):
    """From a trickle to an absurd 1e15 W the solve lands on the balance; it used to stop short at high power."""
    p = bt.BrakeThermalParams()
    eq = bt.BrakeThermalModel(p).equilibrium(p_brake_avg_w=power, v_work_ms=12.0, diameter_mm=200.0)
    T, Ta = eq.T_equilibrium_c + 273.15, p.T_ambient_c + 273.15
    out = eq.h_air * eq.area_m2 * (T - Ta) + p.emissivity * SIGMA * eq.area_m2 * (T ** 4 - Ta ** 4)
    assert out == pytest.approx(power, rel=1e-9)


def test_pad_fade_is_off_by_default_then_linear_above_onset_down_to_a_floor():
    p = bt.BrakeThermalParams()
    assert p.mu_fade(900.0) == 1.0
    f = bt.BrakeThermalParams(enable_fade=True, T_fade_onset_c=450.0, fade_per_C=0.0012, fade_floor=0.45)
    assert f.mu_fade(400.0) == 1.0 and f.mu_fade(550.0) == pytest.approx(1 - 0.12) and f.mu_fade(2000.0) == 0.45
    hot = bt.BrakeThermalModel(f).equilibrium(p_brake_avg_w=6000.0, v_work_ms=8.0, diameter_mm=180.0)
    assert hot.faded and hot.mu_fade_mult == pytest.approx(f.mu_fade(hot.T_equilibrium_c))
    assert bt.default_brake_thermal_params().calibrated is False


def test_the_mass_temperature_sweep_is_the_single_stop_at_each_diameter():
    m = bt.BrakeThermalModel()
    rows = m.mass_temp_sweep(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, thickness_mm=5.0, n=5)
    assert [r["diameter_mm"] for r in rows] == [180.0, 210.0, 240.0, 270.0, 300.0]
    for r in rows:
        s = m.single_stop(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, diameter_mm=r["diameter_mm"], thickness_mm=5.0)
        assert r["T_peak_c"] == s.T_peak_c and r["rotor_mass_kg"] == s.rotor_mass_kg
    assert all(a["T_peak_c"] > b["T_peak_c"] for a, b in zip(rows, rows[1:]))       # more mass, cooler
    assert len(m.mass_temp_sweep(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, thickness_mm=5.0, n=1)) == 2
    assert m.provenance()["calibrated"] is False and "triage" in m.provenance()["note"]


# --------------------------------------------------------------------------- #
#  Power series carry the energy they claim
# --------------------------------------------------------------------------- #
def test_a_fade_test_puts_each_stops_kinetic_energy_into_the_rotor():
    """Per stop, the energy into one front rotor is 1/2 m v0^2 x bias x share-to-rotor / 2 (up to the last partial
    time step); the gaps carry no power at 60% of v0."""
    m, v0, bias, dt = 300.0, 25.0, 0.65, 0.01
    pw, vs, dt_out = bt.fade_test_power_series(mass_kg=m, v0_ms=v0, front_bias=bias, n_stops=3, dt=dt)
    e = 0.5 * m * v0 ** 2 * bias * 0.90 / 2
    assert dt_out == dt and np.sum(pw) * dt == pytest.approx(3 * e, rel=0.01)
    assert vs[0] == v0 and min(v for v, p in zip(vs, pw) if p == 0.0) == pytest.approx(0.6 * v0)


def test_a_lap_trace_is_resampled_without_losing_braking_energy():
    """One straight, one braking zone from 30 to 10 m/s over 80 m: the resampled power integrates to the energy the
    lap-energy routine counts for one front rotor."""
    d = np.linspace(0.0, 400.0, 401)
    v = np.where(d < 300.0, 30.0, np.maximum(30.0 - (d - 300.0) * 0.25, 10.0))
    g = np.where(np.diff(v, append=v[-1]) < 0, -1.0, 0.0)
    pw, vs, dt = bt.lap_power_series(d, v, mass_kg=300.0, front_bias=0.65, dt=0.005, long_g=g)
    le = bt.lap_brake_energy(d, v, mass_kg=300.0, front_bias=0.65, lap_time_s=20.0, long_g=g)
    assert le.q_lap_j == pytest.approx(0.5 * 300.0 * (30.0 ** 2 - 10.0 ** 2))
    assert np.sum(pw) * dt == pytest.approx(le.q_front_lap_j / 2.0, rel=0.02)
    assert le.p_front_avg_w == pytest.approx(le.q_front_lap_j / 2 / 20.0) and le.n_brake_samples == 80
    assert le.v_brake_mean_ms == pytest.approx(20.0, rel=0.01) and le.p_front_peak_w == pytest.approx(max(pw), rel=1e-9)


def test_coasting_is_not_braking_when_the_longitudinal_g_says_so():
    d = np.linspace(0, 100, 101)
    v = np.linspace(30, 20, 101)
    coast = bt.lap_brake_energy(d, v, mass_kg=300.0, front_bias=0.6, lap_time_s=5.0, long_g=np.zeros(101))
    assert coast.q_lap_j == 0.0 and coast.v_brake_mean_ms == 0.0
    pw, _, _ = bt.lap_power_series(d, v, mass_kg=300.0, front_bias=0.6, long_g=np.zeros(101))
    assert max(pw) == 0.0
    assert bt.lap_brake_energy(d, v, mass_kg=300.0, front_bias=0.6, lap_time_s=5.0).q_lap_j > 0
    up = bt.lap_brake_energy(d, v[::-1], mass_kg=300.0, front_bias=0.6, lap_time_s=5.0)
    assert up.q_lap_j == 0.0
    still = bt.lap_brake_energy([0.0, 0.0], [10.0, 5.0], mass_kg=300.0, front_bias=0.6, lap_time_s=5.0)
    assert still.q_lap_j > 0 and still.p_front_peak_w == 0.0                   # no distance, no time: no peak power


def test_a_trace_too_short_to_brake_on_returns_nothing():
    assert bt.lap_power_series([0.0], [10.0], mass_kg=300.0, front_bias=0.6) == ([0.0], [0.0], 0.1)
    e = bt.lap_brake_energy([0.0], [10.0], mass_kg=300.0, front_bias=0.6, lap_time_s=5.0)
    assert e.q_lap_j == 0.0 and e.n_brake_samples == 0 and e.lap_time_s == 5.0


def test_pad_wear_index_is_its_stated_formula():
    w = bt.pad_wear_index(q_lap_j=2.0e6, T_pad_peak_c=500.0, front_bias=0.6)
    assert w["q_per_front_pad_kj"] == pytest.approx(600.0) and w["temp_acceleration"] == pytest.approx(2.0)
    assert w["wear_index"] == pytest.approx(6.0e5 / 5.0e5 * 2.0)
    assert bt.pad_wear_index(q_lap_j=2.0e6, T_pad_peak_c=200.0, front_bias=0.6)["temp_acceleration"] == 1.0


# --------------------------------------------------------------------------- #
#  The rotor optimiser
# --------------------------------------------------------------------------- #
def _small_search(**kw):
    base = dict(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, T_constraint_c=500.0, dia_steps=3, th_steps=3,
                vent_steps=2, n_stops=3, gap_s=4.0)
    base.update(kw)
    return bt.optimise_rotor(**base)


def test_the_optimiser_returns_the_lightest_feasible_rotor_and_a_true_pareto_front():
    r = _small_search()
    assert r.n_evaluated == 3 * 3 * 2 and len(r.candidates) == r.n_evaluated
    assert all(c.feasible == (c.margin_c >= 0.0) for c in r.candidates)
    assert r.best is min(r.feasible, key=lambda c: c.rotor_mass_kg)
    for c in r.pareto:
        assert not any(o.rotor_mass_kg <= c.rotor_mass_kg and o.T_rotor_peak_c <= c.T_rotor_peak_c
                       and (o.rotor_mass_kg < c.rotor_mass_kg or o.T_rotor_peak_c < c.T_rotor_peak_c)
                       for o in r.feasible)
    assert [c.rotor_mass_kg for c in r.pareto] == sorted(c.rotor_mass_kg for c in r.pareto)
    assert r.synthesized and r.constraint_c == 500.0


def test_with_nothing_feasible_the_front_spans_every_candidate():
    r = _small_search(T_constraint_c=40.0, materials=["grey_cast_iron", "carbon_carbon"], vent_steps=1)
    assert r.best is None and not r.feasible and r.n_evaluated == 2 * 3 * 3
    assert all(c in r.candidates for c in r.pareto) and r.pareto
    assert {c.vented_fraction for c in r.candidates} == {0.0}


def test_a_high_crack_risk_rules_a_rotor_out_even_when_it_runs_cool_enough():
    r = _small_search(screen_cracks=True, T_constraint_c=2000.0, dia_steps=2, th_steps=2, vent_steps=1)
    for c in r.candidates:
        assert c.crack_risk in ("low", "elevated", "high")
        assert c.feasible == (c.margin_c >= 0.0 and c.crack_risk != "high")
        assert c.T_surface_peak_c >= c.T_rotor_peak_c - 1e-6
