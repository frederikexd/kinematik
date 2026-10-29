"""suspension/joints.py against closed forms: each force law and its inverse at points on every segment, the tangent
and secant rates, the energy lost per cycle (pi c omega X^2 + pi eta k_sec X^2), the linearisation (c_eq = c + eta k /
omega), every refusal message, the named presets, and the clevis, shank and thread formulas (VDI 2230)."""
import math

import numpy as np
import pytest

from suspension.joints import (JointCompliance as J, clevis_pin_tilt_deg, rod_end_shank_bending, shank_fatigue_check,
                               stiction_describing_function, thread_endurance_amplitude_mpa)


@pytest.mark.parametrize("make,message", [
    (lambda: J.linear(0.0), "linear joint needs a positive finite rate k (N/mm)."),
    (lambda: J.linear(math.inf), "linear joint needs a positive finite rate k (N/mm)."),
    (lambda: J.cubic(0.0, 1.0), "cubic joint needs a positive linear rate k1 (N/mm)."),
    (lambda: J.cubic(math.nan, 1.0), "cubic joint needs a positive linear rate k1 (N/mm)."),
    (lambda: J.cubic(1.0, -1e-9), "cubic hardening k3 must be ≥ 0 (a softening joint is not monotone / not physical "
                                  "here)."),
    (lambda: J.bilinear(0.0, 1.0, 1.0), "bilinear joint needs positive rate k1 (N/mm)."),
    (lambda: J.bilinear(1.0, math.inf, 1.0), "bilinear joint needs positive rate k2 (N/mm)."),
    (lambda: J.bilinear(1.0, 1.0, 0.0), "bilinear knee_mm must be a positive displacement (mm)."),
    (lambda: J.bilinear(1.0, 1.0, math.inf), "bilinear knee_mm must be a positive displacement (mm)."),
    (lambda: J.freeplay(-1e-9, 1.0), "freeplay lash_mm must be ≥ 0 (mm)."),
    (lambda: J.freeplay(math.inf, 1.0), "freeplay lash_mm must be ≥ 0 (mm)."),
    (lambda: J.freeplay(0.1, 0.0), "freeplay needs a positive engaged rate k (N/mm)."),
    (lambda: J.freeplay(0.1, math.inf), "freeplay needs a positive engaged rate k (N/mm)."),
    (lambda: J.freeplay(0.1, 1.0, k_lash=-1e-9), "freeplay in-band stiffness k_lash must be ≥ 0."),
    (lambda: J.tabular([0, 1], [0, 1]), "tabular joint needs matching 1-D disp/force arrays of length ≥ 3."),
    (lambda: J.tabular([0, 1, 2], [0, 1]), "tabular joint needs matching 1-D disp/force arrays of length ≥ 3."),
    (lambda: J.tabular([[0, 1, 2]], [[0, 1, 2]]), "tabular joint needs matching 1-D disp/force arrays of length ≥ 3."),
    (lambda: J.tabular([0, 1, 1], [0, 1, 2]), "tabular displacement samples must be strictly increasing."),
    (lambda: J.tabular([0, 1, 2], [0, 1, 1]), "tabular force samples must be strictly increasing (a monotone, "
                                              "invertible joint)."),
])
def test_every_refusal_says_what_is_wrong(make, message):
    with pytest.raises(ValueError) as e:
        make()
    assert str(e.value) == message


def test_the_smallest_legal_values_are_accepted():
    assert J.cubic(1e-6, 0.0).k3 == 0.0
    assert J.freeplay(0.0, 1e-6, k_lash=0.0).lash == 0.0
    J.tabular([0.0, 1.0, 2.0], [0.0, 1.0, 2.0])


def test_constructors_store_what_they_are_given():
    for j, kind, label in [(J.linear(3, 0.5, 0.2), "linear", "linear"), (J.cubic(3, 4, 0.5, 0.2), "cubic", "cubic"),
                           (J.bilinear(3, 5, 0.4, 0.5, 0.2), "bilinear", "bilinear"),
                           (J.freeplay(0.1, 3, 2, 0.5, 0.2), "freeplay", "freeplay"),
                           (J.tabular([-1, 0, 1], [-3, 0, 3], 0.5, 0.2), "tabular", "tabular")]:
        assert (j.kind, j.label, j.c_viscous, j.loss_factor) == (kind, label, 0.5, 0.2)
        assert type(j.c_viscous) is float and type(j.loss_factor) is float
    assert (J.linear(3).c_viscous, J.linear(3).loss_factor) == (0.0, 0.0)
    for j in (J.cubic(3, 4), J.bilinear(3, 5, 0.4), J.freeplay(0.1, 3), J.tabular([-1, 0, 1], [-3, 0, 3])):
        assert (j.c_viscous, j.loss_factor) == (0.0, 0.0)
    assert (J.cubic(3, 4).k1, J.cubic(3, 4).k3) == (3.0, 4.0)
    b = J.bilinear(3, 5, 0.4)
    assert (b.k1, b.k2, b.knee) == (3.0, 5.0, 0.4)
    f = J.freeplay(0.1, 3, 2)
    assert (f.lash, f.k1, f.k_lash) == (0.1, 3.0, 2.0) and J.freeplay(0.1, 3).k_lash == 0.0
    assert J.linear(3, label="x").label == "x" and J.tabular([-1, 0, 1], [-3, 0, 3], label="t").label == "t"
    for make in (J.cubic, ):
        assert make(3, 4, label="y").label == "y"
    assert J.bilinear(3, 5, 0.4, label="b").label == "b" and J.freeplay(0.1, 3, label="f").label == "f"


def test_the_presets_are_the_documented_parts():
    r = J.rubber_bushing()
    assert (r.kind, r.k1, r.k3, r.loss_factor, r.label) == ("cubic", 1500.0, 12000.0, 0.12, "rubber bushing")
    r = J.rubber_bushing(k_radial=1000.0, hardening=2.0, loss_factor=0.3, label="mine")
    assert (r.k1, r.k3, r.loss_factor, r.label) == (1000.0, 2000.0, 0.3, "mine")
    p = J.polyurethane_bushing()
    assert (p.kind, p.k1, p.k3, p.loss_factor, p.label) == ("cubic", 6000.0, 24000.0, 0.05, "polyurethane bushing")
    p = J.polyurethane_bushing(k_radial=2000.0, hardening=3.0, loss_factor=0.07, label="pu")
    assert (p.k1, p.k3, p.loss_factor, p.label) == (2000.0, 6000.0, 0.07, "pu")
    s = J.spherical_bearing()
    assert (s.kind, s.lash, s.k1, s.k_lash, s.loss_factor, s.label) == \
        ("freeplay", 0.05, 120000.0, 2000.0, 0.01, "spherical bearing")
    s = J.spherical_bearing(lash_mm=0.1, k=5e4, k_lash=500.0, loss_factor=0.02, label="worn")
    assert (s.lash, s.k1, s.k_lash, s.loss_factor, s.label) == (0.1, 5e4, 500.0, 0.02, "worn")


@pytest.mark.parametrize("j,points", [
    (J.linear(2000.0), [(0.5, 1000.0), (-0.25, -500.0)]),
    (J.cubic(1000.0, 4000.0), [(0.5, 500.0 + 500.0), (-0.5, -1000.0), (0.1, 100.0 + 4.0)]),
    (J.bilinear(1000.0, 5000.0, 0.2), [(0.1, 100.0), (0.2, 200.0), (0.3, 200.0 + 500.0), (-0.3, -700.0)]),
    (J.freeplay(0.1, 10000.0, k_lash=200.0), [(0.05, 10.0), (0.1, 20.0), (0.3, 20.0 + 2000.0), (-0.3, -2020.0)]),
])
def test_each_force_law_and_its_inverse(j, points):
    for d, f in points:
        assert j.force(d) == pytest.approx(f, rel=1e-12)
        assert j.displacement(f) == pytest.approx(d, rel=1e-9)
    assert j.displacement(0.0) == 0.0


def test_tangent_and_secant_rates_on_every_segment():
    c = J.cubic(1000.0, 4000.0)
    assert c.tangent_stiffness(0.5) == pytest.approx(1000.0 + 3 * 4000.0 * 0.25)
    b = J.bilinear(1000.0, 5000.0, 0.2)
    assert (b.tangent_stiffness(0.2), b.tangent_stiffness(-0.25)) == (1000.0, 5000.0)
    f = J.freeplay(0.1, 10000.0, k_lash=200.0)
    assert (f.tangent_stiffness(0.1), f.tangent_stiffness(-0.11)) == (200.0, 10000.0)
    assert J.linear(7.0).tangent_stiffness(123.0) == 7.0
    assert b.secant_stiffness(700.0) == pytest.approx(700.0 / 0.3)
    assert b.secant_stiffness(0.0) == 1000.0
    assert J.freeplay(0.1, 1000.0).secant_stiffness(0.0) == 0.0
    loose = J.freeplay(0.1, 1000.0)                       # no in-band rate: any in-band force reads no motion
    assert loose.displacement(0.0) == 0.0 and loose.force(0.05) == 0.0
    assert loose.displacement(100.0) == pytest.approx(0.2)


def test_the_cubic_inverse_is_newton_to_the_last_digit():
    j = J.cubic(10.0, 1e6)                                 # far from the linear seed
    d = j.displacement(5000.0)
    assert j.force(d) == pytest.approx(5000.0, rel=1e-12)
    assert d == pytest.approx(np.roots([1e6, 0.0, 10.0, -5000.0])[2].real, rel=1e-12)


def test_a_measured_table_is_interpolated_and_inverted_monotonically():
    d = [-1.0, -0.2, 0.0, 0.3, 1.0]
    f = [-3000.0, -300.0, 0.0, 400.0, 2500.0]
    j = J.tabular(d, f)
    for di, fi in zip(d, f):
        assert j.force(di) == pytest.approx(fi)
    for F in (-2000.0, -10.0, 250.0, 1999.0):
        assert j.force(j.displacement(F)) == pytest.approx(F, abs=1e-6)
    slope_hi = j._f_of_d.derivative()(1.0)
    assert j.displacement(3000.0) == pytest.approx(1.0 + 500.0 / slope_hi)          # beyond the table: end slope
    assert j.displacement(2500.0) == pytest.approx(1.0)
    slope_lo = j._f_of_d.derivative()(-1.0)
    assert j.displacement(-3500.0) == pytest.approx(-1.0 - 500.0 / slope_lo)
    assert j.displacement(-3000.0) == pytest.approx(-1.0)
    assert j.tangent_stiffness(0.0) == pytest.approx(float(j._f_of_d.derivative()(0.0)))
    assert (j._d_lo, j._d_hi, j._f_lo, j._f_hi) == (-1.0, 1.0, -3000.0, 2500.0)
    assert j.table_disp.dtype == np.float64 and j.table_force.dtype == np.float64
    assert J(kind="tabular")._f_of_d is None                    # nothing to build without a table


def test_an_unknown_kind_is_refused_by_every_law():
    j = J(kind="magic")
    for call in (lambda: j.force(1.0), lambda: j.displacement(1.0), lambda: j.tangent_stiffness(1.0)):
        with pytest.raises(ValueError) as e:
            call()
        assert str(e.value) == "unknown joint kind 'magic'"


def test_energy_per_cycle_is_viscous_plus_structural():
    j = J.bilinear(1000.0, 5000.0, 0.2, c_viscous=2.0, loss_factor=0.1)
    X, f = 0.3, 12.0
    k_sec = (200.0 + 500.0) / X
    want = math.pi * 2.0 * (2 * math.pi * f) * X ** 2 + math.pi * 0.1 * k_sec * X ** 2
    assert j.energy_loss_per_cycle(X, f) == pytest.approx(want, rel=1e-12)
    assert j.energy_loss_per_cycle(-X, f) == pytest.approx(want, rel=1e-12)
    assert j.energy_loss_per_cycle(0.0, f) == 0.0


def test_the_linearisation_folds_the_loss_factor_into_a_viscous_rate():
    j = J.cubic(1000.0, 4000.0, c_viscous=1.5, loss_factor=0.2)
    d = j.displacement(800.0)
    k_t = 1000.0 + 3 * 4000.0 * d * d
    lin = j.linearize(about_force_N=800.0, freq_hz=10.0)
    assert lin == pytest.approx({"tangent_stiffness_N_per_mm": k_t, "viscous_c_N_s_per_mm": 1.5,
                                 "equivalent_viscous_c_N_s_per_mm": 1.5 + 0.2 * k_t / (2 * math.pi * 10.0),
                                 "loss_factor": 0.2, "about_force_N": 800.0, "displacement_mm": d})
    for kw in (dict(), dict(freq_hz=0.0), dict(freq_hz=None)):
        assert j.linearize(800.0, **kw)["equivalent_viscous_c_N_s_per_mm"] == 1.5
    assert J.cubic(1000.0, 4000.0, c_viscous=1.5).linearize(800.0, 10.0)["equivalent_viscous_c_N_s_per_mm"] == 1.5
    zero = j.linearize()
    assert (zero["about_force_N"], zero["displacement_mm"], zero["tangent_stiffness_N_per_mm"]) == (0.0, 0.0, 1000.0)


def test_the_summary_names_every_coefficient():
    j = J(kind="bilinear", k1=1.0, k3=2.0, k2=3.0, knee=4.0, lash=5.0, k_lash=6.0, c_viscous=7.0, loss_factor=8.0,
          label="L")
    assert j.summary() == {"kind": "bilinear", "label": "L", "k1_N_per_mm": 1.0, "k3_N_per_mm3": 2.0,
                           "k2_N_per_mm": 3.0, "knee_mm": 4.0, "lash_mm": 5.0, "k_lash_N_per_mm": 6.0,
                           "c_viscous_N_s_per_mm": 7.0, "loss_factor": 8.0}


def test_stiction_locks_below_break_out_and_lags_above_it():
    assert stiction_describing_function(1000.0, 200.0, 150.0) == {
        "locked": True, "k_eff_n_per_mm": math.inf, "phase_lag_deg": 0.0, "hysteresis_mm": 0.0}
    assert stiction_describing_function(1000.0, -200.0, 200.0)["locked"] is True       # break-out by magnitude
    for bad in ((0.0, 1.0, 2.0), (1.0, 1.0, 0.0)):
        with pytest.raises(ValueError) as e:
            stiction_describing_function(*bad)
        assert str(e.value) == "k and force amplitude must be > 0."
    r = stiction_describing_function(1000.0, 200.0, 1000.0)
    assert r["locked"] is False and r["hysteresis_mm"] == pytest.approx(0.4)
    assert r["phase_lag_deg"] > 0.0 and r["k_eff_n_per_mm"] > 1000.0
    # the settled loop, built here: x follows (F - Fs)/k up the slope and (F + Fs)/k down it
    th = np.linspace(0.0, 2 * np.pi, 20000, endpoint=False)
    F = 1000.0 * np.sin(th)
    x = np.empty_like(F)
    up = (th <= np.pi / 2) | (th > 3 * np.pi / 2)
    x[:] = np.nan
    hold_hi, hold_lo = 0.8, -0.8                                        # (F0 - Fs)/k at the turning points
    x = np.where(up & (F - 200.0 >= 1000.0 * hold_lo), (F - 200.0) / 1000.0, np.nan)
    x = np.where(~up & (F + 200.0 <= 1000.0 * hold_hi), (F + 200.0) / 1000.0, x)
    x = np.where(np.isnan(x) & up, hold_lo, x)
    x = np.where(np.isnan(x) & ~up, hold_hi, x)
    a, b = 2 * np.mean(x * np.sin(th)), 2 * np.mean(x * np.cos(th))
    assert r["k_eff_n_per_mm"] == pytest.approx(1000.0 / math.hypot(a, b), rel=2e-3)
    assert r["phase_lag_deg"] == pytest.approx(-math.degrees(math.atan2(b, a)), abs=0.05)
    assert r == stiction_describing_function(1000.0, 200.0, 1000.0, n=4000)
    free = stiction_describing_function(1000.0, 0.0, 1000.0)
    assert free["k_eff_n_per_mm"] == pytest.approx(1000.0, rel=1e-9) and abs(free["phase_lag_deg"]) < 1e-9
    assert stiction_describing_function(1000.0, 200.0, -1000.0) == r


def test_clevis_tilt_is_the_tab_deflection_difference_over_the_spacing():
    assert clevis_pin_tilt_deg(1000.0, 5000.0, 5000.0, 20.0) == 0.0
    fa, fb = 1000.0 * (0.5 + 2.0 / 20.0), 1000.0 * (0.5 - 2.0 / 20.0)
    want = math.degrees(math.atan2(fa / 4000.0 - fb / 6000.0, 20.0))
    assert clevis_pin_tilt_deg(1000.0, 4000.0, 6000.0, 20.0, load_offset_mm=2.0) == pytest.approx(want, rel=1e-12)
    assert clevis_pin_tilt_deg(1000.0, 4000.0, 6000.0, 20.0) == pytest.approx(
        math.degrees(math.atan2(500.0 / 4000.0 - 500.0 / 6000.0, 20.0)), rel=1e-12)


def test_shank_bending_is_the_ball_friction_moment():
    r = rod_end_shank_bending(-3000.0, 8.0, 6.0)
    M = 0.1 * 3000.0 * 8.0
    assert r == pytest.approx({"moment_Nmm": M, "stress_MPa": M / (math.pi * 216.0 / 32.0), "binds": False})
    assert rod_end_shank_bending(3000.0, 8.0, 6.0, mu_ball=0.2)["moment_Nmm"] == pytest.approx(2 * M)
    assert rod_end_shank_bending(3000.0, 8.0, 6.0, pin_tilt_deg=-13.0)["binds"] is True
    assert rod_end_shank_bending(3000.0, 8.0, 6.0, pin_tilt_deg=12.9)["binds"] is False
    assert rod_end_shank_bending(3000.0, 8.0, 6.0, pin_tilt_deg=10.0, misalignment_rating_deg=10.0)["binds"] is True


def test_thread_endurance_follows_vdi_2230():
    sv = 0.85 * (150.0 / 10.0 + 45.0)
    assert thread_endurance_amplitude_mpa(10.0) == pytest.approx(sv)
    assert thread_endurance_amplitude_mpa(10.0, rolled_after_heat_treatment=True) == pytest.approx(1.5 * sv)
    assert thread_endurance_amplitude_mpa(10.0, True, mean_to_proof=0.3) == pytest.approx(1.7 * sv)


def test_the_shank_fatigue_check_uses_the_thread_root():
    r = shank_fatigue_check(2000.0, 7.0, 9.525, 24.0)
    d3 = 9.525 - 1.226869 * 25.4 / 24.0
    sa = 0.15 * 2000.0 * 7.0 / (math.pi * d3 ** 3 / 32.0)
    se = 0.85 * (150.0 / 9.525 + 45.0)
    assert r == pytest.approx({"root_dia_mm": d3, "stress_amp_MPa": sa, "endurance_amp_MPa": se, "fos": se / sa})
    assert shank_fatigue_check(-2000.0, 7.0, 9.525, 24.0, mu_ball=0.3)["stress_amp_MPa"] == pytest.approx(2 * sa)
    assert shank_fatigue_check(0.0, 7.0, 9.525, 24.0)["fos"] == math.inf


def test_soft_rates_and_unit_points_are_ordinary_values():
    j = J.linear(0.5)
    assert j.kind == "linear" and j.force(2.0) == pytest.approx(1.0)
    assert J.linear(100.0).secant_stiffness(100.0) == pytest.approx(100.0)          # d = 1 mm exactly
    c = J.linear(80.0, c_viscous=0.3, loss_factor=0.1)
    assert c.energy_loss_per_cycle(1.0, 5.0) == pytest.approx(
        math.pi * 0.3 * 2 * math.pi * 5.0 + math.pi * 0.1 * 80.0, rel=1e-12)
    lin = c.linearize(10.0, freq_hz=0.5)
    assert lin["equivalent_viscous_c_N_s_per_mm"] == pytest.approx(0.3 + 0.1 * 80.0 / (2 * math.pi * 0.5), rel=1e-12)


def test_the_tabular_curve_extrapolates_and_needs_pchip(monkeypatch):
    import suspension.joints as jm
    from scipy.interpolate import PchipInterpolator
    d, f = [-1.0, 0.0, 0.5, 2.0], [-300.0, 0.0, 120.0, 900.0]
    t = J.tabular(d, f)
    assert t.force(3.0) == pytest.approx(float(PchipInterpolator(d, f)(3.0)), rel=1e-12) and math.isfinite(t.force(3.0))
    monkeypatch.setattr(jm, "_HAVE_PCHIP", False)
    with pytest.raises(RuntimeError) as e:
        J.tabular(d, f)
    assert str(e.value) == "tabular joints need scipy.interpolate.PchipInterpolator."


def test_stiction_with_a_soft_path_and_a_small_force():
    r = stiction_describing_function(0.5, 0.2, 0.8)
    assert r["locked"] is False and r["hysteresis_mm"] == pytest.approx(0.8)
    assert stiction_describing_function(0.5, 0.0, 0.8)["k_eff_n_per_mm"] == pytest.approx(0.5, rel=1e-3)


def test_the_binding_default_and_a_tiny_fatigue_amplitude():
    assert rod_end_shank_bending(1000.0, 6.0, 8.0, misalignment_rating_deg=0.5)["binds"] is False
    r = shank_fatigue_check(1.0, 6.0, 9.525, 24.0)
    assert 0 < r["stress_amp_MPa"] < 1 and r["fos"] == pytest.approx(r["endurance_amp_MPa"] / r["stress_amp_MPa"])
