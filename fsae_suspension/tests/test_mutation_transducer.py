"""suspension/transducer.py: tube section and surface strains, the bridge equation for each arrangement, the
self-heating limit and its inverse, Euler buckling, the shunt that simulates a reading, lead-resistance loss, the
dynamic amplitude error, the exact Poisson bridge, and the gauge-layout Monte Carlo (a perfect layout leaks nothing; a
toleranced one is rebuilt here from the stated model with the same random draws)."""
import math

import numpy as np
import pytest

from suspension import transducer as td


def test_tube_section_and_strains():
    s = td.tube_section(19.05, 0.9)
    i = 19.05 - 1.8
    assert s == pytest.approx({"A_mm2": math.pi / 4 * (19.05 ** 2 - i ** 2), "I_mm4": math.pi / 64 * (19.05 ** 4 - i ** 4),
                               "J_mm4": math.pi / 32 * (19.05 ** 4 - i ** 4), "c_mm": 9.525})
    e = td.tube_strains(10000.0, 2e5, 1e5, 19.05, 0.9)
    G = 205000.0 / 2.58
    assert e == pytest.approx({"axial_ue": 10000.0 / (205000.0 * s["A_mm2"]) * 1e6,
                               "bending_ue": 2e5 * 9.525 / s["I_mm4"] / 205000.0 * 1e6,
                               "torsion_shear_ue": 1e5 * 9.525 / s["J_mm4"] / G * 1e6})
    e2 = td.tube_strains(10000.0, 2e5, 1e5, 19.05, 0.9, E=70000.0, nu=0.33)
    assert e2["torsion_shear_ue"] == pytest.approx(1e5 * 9.525 / s["J_mm4"] / (70000.0 / 2.66) * 1e6)
    assert e2["axial_ue"] == pytest.approx(10000.0 / (70000.0 * s["A_mm2"]) * 1e6)


def test_the_bridge_equation_for_each_arrangement():
    assert td.bridge_output_mvv(500.0) == pytest.approx(2.1 / 4 * 2 * 1.29 * 500e-6 * 1e3)
    assert td.bridge_output_mvv(500.0, 2.0, 0.3, "poisson_half") == pytest.approx(0.5 * 1.3 * 500e-6 * 1e3)
    assert td.bridge_output_mvv(500.0, 2.0, 0.3, "quarter") == pytest.approx(0.5 * 500e-6 * 1e3)
    with pytest.raises(KeyError):
        td.bridge_output_mvv(500.0, arrangement="tee")


def test_self_heating_limit_and_its_inverse():
    v = td.excitation_limit_v(350.0, 6.0, 2.0)
    assert v == pytest.approx(2 * math.sqrt(350.0 * 6e-6 * 2000.0))
    assert td.grid_power_density(v, 350.0, 6.0) == pytest.approx(2.0)
    v3 = td.excitation_limit_v(350.0, 6.0, 2.0, n_series=3)
    assert v3 == pytest.approx(3 * v) and td.grid_power_density(v3, 350.0, 6.0, n_series=3) == pytest.approx(2.0)
    assert td.grid_power_density(10.0, 350.0, 6.0) == pytest.approx(25.0 / (350.0 * 6e-6) / 1e3)
    assert td.POWER_DENSITY_KW_M2[("thin_steel", "dynamic", "high")] == (3.0, 8.0)
    assert len(td.POWER_DENSITY_KW_M2) == 5 and td.POWER_DENSITY_KW_M2[("aluminium", "static", "high")] == (3.0, 8.0)


def test_euler_load_and_shunt_calibration():
    I = td.tube_section(15.875, 0.889)["I_mm4"]
    assert td.euler_load_n(300.0, 15.875, 0.889) == pytest.approx(math.pi ** 2 * 205000.0 * I / 9e4)
    assert td.euler_load_n(300.0, 15.875, 0.889, E=70000.0) == pytest.approx(math.pi ** 2 * 70000.0 * I / 9e4)
    rs = td.shunt_resistor_ohm(350.0, 1.0)
    assert rs == pytest.approx(350.0 * (1 / 0.004 - 1))
    dR = 350.0 ** 2 / (350.0 + rs)                       # the arm's drop with the shunt across it
    assert dR / 350.0 / 4 == pytest.approx(1e-3, rel=1e-12)


def test_lead_resistance_loss_and_its_drift():
    r = td.four_wire_loss(350.0, 0.5)
    assert r == pytest.approx({"loss_pct": 100 * 1.0 / 351.0, "drift_pct": 0.0})
    r = td.four_wire_loss(350.0, 0.5, dT_c=40.0)
    rl = 0.5 * (1 + 0.00393 * 40)
    assert r["drift_pct"] == pytest.approx(100 * (2 * rl / (350 + 2 * rl) - 1.0 / 351.0))
    r = td.four_wire_loss(350.0, 0.5, dT_c=40.0, alpha_cu=0.004)
    rl = 0.5 * 1.16
    assert r["drift_pct"] == pytest.approx(100 * (2 * rl / (350 + 2 * rl) - 1.0 / 351.0))


def test_dynamic_error_exact_bridge_and_shunt_check():
    assert td.dynamic_amplitude_error(30.0, 120.0) == pytest.approx(100 * (1 / (1 - 1 / 16) - 1))
    e = td.poisson_bridge_exact_mvv(2000.0)
    x = 2.1 * 2e-3
    assert e == pytest.approx({"exact_mvv": 1.29 * x / (2 + 0.71 * x) * 1e3, "linear_mvv": 1.29 * x / 2 * 1e3,
                               "nonlinearity_pct": 100 * ((1.29 * x / (2 + 0.71 * x)) / (1.29 * x / 2) - 1)})
    e = td.poisson_bridge_exact_mvv(2000.0, 2.0, 0.3)
    x = 4e-3
    assert e["exact_mvv"] == pytest.approx(1.3 * x / (2 + 0.7 * x) * 1e3)
    assert td.shunt_ratio_check(1.0012, 1.0) == pytest.approx({"shift_pct": 0.12, "inspect": True})
    assert td.shunt_ratio_check(0.9992, 1.0)["inspect"] is False
    assert td.shunt_ratio_check(0.9992, 1.0, limit_pct=0.05)["inspect"] is True


def test_a_perfect_layout_leaks_nothing():
    r = td.rejection_monte_carlo(pos_tol_deg=0.0, align_tol_deg=0.0, gf_tol=0.0, n=200)
    assert r["bending_p95_ue"] == pytest.approx(0.0, abs=1e-9) and r["torsion_p95_ue"] == pytest.approx(0.0, abs=1e-9)
    assert r["pass_fraction"] == 1.0 and r["limit_ue"] == pytest.approx(0.0025 * 559.0)


def _mc(n_axial=2, pos=1.0, align=0.5, gf=0.002, bend=190.0, tors=246.0, fs=559.0, lim=0.25, nu=0.29, n=2000, seed=0):
    rng = np.random.default_rng(seed)
    th = np.radians(np.arange(n_axial) * 360.0 / n_axial + rng.uniform(-pos, pos, (n, n_axial)))
    pa = np.radians(rng.uniform(-align, align, (n, n_axial)))
    pt = np.radians(90.0 + rng.uniform(-align, align, (n, n_axial)))
    ga = 1 + rng.uniform(-gf, gf, (n, n_axial))
    gt = 1 + rng.uniform(-gf, gf, (n, n_axial))
    read = lambda ex, ey, g, ph: ex * np.cos(ph) ** 2 + ey * np.sin(ph) ** 2 + g * np.sin(ph) * np.cos(ph)
    br = lambda ex, ey, g: (ga * read(ex, ey, g, pa)).mean(1) - (gt * read(ex, ey, g, pt)).mean(1)
    sens = br(1.0, -nu, 0.0)
    psi = rng.uniform(0, 2 * np.pi, (n, 1))
    b = np.abs(br(np.cos(th - psi), -nu * np.cos(th - psi), 0.0)) / sens * bend
    t = np.abs(br(0.0, 0.0, 1.0)) / sens * tors
    L = lim / 100 * fs
    pb, ptq = np.percentile(b, 95), np.percentile(t, 95)
    return {"bending_p95_ue": pb, "torsion_p95_ue": ptq, "bending_rejection_p95": bend / pb,
            "torsion_rejection_p95": tors / ptq, "limit_ue": L, "pass_fraction": np.mean((b < L) & (t < L))}


@pytest.mark.parametrize("kw", [dict(), dict(n_axial=4, pos=2.0, align=1.0, gf=0.005, nu=0.33, seed=3),
                                dict(bend=100.0, tors=300.0, fs=800.0, lim=0.5, seed=1)])
def test_the_toleranced_layout_follows_the_stated_model(kw):
    names = dict(n_axial="n_axial", pos="pos_tol_deg", align="align_tol_deg", gf="gf_tol", bend="bending_ue",
                 tors="torsion_ue", fs="axial_fs_ue", lim="limit_pct_fs", nu="nu", seed="seed")
    r = td.rejection_monte_carlo(n=2000, **{names[k]: v for k, v in kw.items()})
    assert r == pytest.approx(_mc(**kw))
    assert 0.0 < r["pass_fraction"] <= 1.0


def test_the_default_monte_carlo_size_and_seed():
    a = td.rejection_monte_carlo()
    assert a == pytest.approx(_mc(n=100000))


def test_the_shunt_shift_is_relative_to_the_reference():
    assert td.shunt_ratio_check(2.003, 2.0)["shift_pct"] == pytest.approx(0.15, rel=1e-9)


def test_the_rejection_ratios_without_and_with_tiny_leakage():
    z = td.rejection_monte_carlo(bending_ue=0.0, torsion_ue=0.0, n=200)
    assert z["bending_rejection_p95"] == math.inf and z["torsion_rejection_p95"] == math.inf
    t = td.rejection_monte_carlo(pos_tol_deg=0.05, align_tol_deg=0.02, gf_tol=0.0001, n=2000)
    assert 0 < t["bending_p95_ue"] < 1 and 0 < t["torsion_p95_ue"] < 1
    assert t["bending_rejection_p95"] == pytest.approx(190.0 / t["bending_p95_ue"])
    assert t["torsion_rejection_p95"] == pytest.approx(246.0 / t["torsion_p95_ue"])
