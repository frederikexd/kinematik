"""suspension/contact_patch.py and suspension/damper_synthesis.py. The quarter car's responses are recomputed here by
Cramer's rule on the equations of motion written out in the module docstring (not by the module's matrix solve), with
the limits that pin them (no load change for a slow road, the tire alone at high frequency), the ISO 8608 PSD in
closed form, and the synthesis steps (golden-section optimum, knee, transient share) against a grid search and their
formulas."""
import math
from dataclasses import replace

import numpy as np
import pytest

from suspension import contact_patch as cp
from suspension import damper_synthesis as ds

QC = cp.QuarterCar(m_s=65.0, m_u=11.0, k=30000.0, c=1500.0, k_t=120000.0, c_t=80.0, kappa=0.12)


def _cramer(qc, f):
    s = 1j * 2 * math.pi * f
    kt, ks, K = qc.k_t + qc.c_t * s, qc.k + qc.c * s, qc.kappa
    # m_s s^2 z_s = -ks (z_s - z_u) + K kt (1 - z_u);  m_u s^2 z_u = ks (z_s - z_u) + (1 - K) kt (1 - z_u)
    a11, a12, b1 = qc.m_s * s * s + ks, -ks + K * kt, K * kt
    a21, a22, b2 = -ks, qc.m_u * s * s + ks + (1 - K) * kt, (1 - K) * kt
    det = a11 * a22 - a12 * a21
    return (b1 * a22 - a12 * b2) / det, (a11 * b2 - a21 * b1) / det, kt, s


def test_the_quarter_car_refuses_what_is_not_a_corner():
    for n in ("m_s", "m_u", "k", "k_t"):
        with pytest.raises(ValueError) as e:
            replace(QC, **{n: 0.0})
        assert str(e.value) == f"QuarterCar.{n} must be > 0."
    for kw in (dict(c=-1.0), dict(c_t=-1.0)):
        with pytest.raises(ValueError) as e:
            replace(QC, **kw)
        assert str(e.value) == "QuarterCar damping must be >= 0."
    replace(QC, c=0.0, c_t=0.0)
    q = cp.QuarterCar(1.0, 1.0, 1.0, 0.0, 1.0)
    assert (q.c_t, q.kappa) == (0.0, 0.0)


def test_static_load_and_the_two_natural_frequencies():
    assert QC.static_load_N == pytest.approx(76.0 * 9.81)
    assert QC.wheel_hop_hz() == pytest.approx(math.sqrt(150000.0 / 11.0) / (2 * math.pi))
    assert QC.body_hz() == pytest.approx(math.sqrt(30000.0 * 120000.0 / 150000.0 / 65.0) / (2 * math.pi))


def test_the_load_response_by_cramers_rule_and_its_limits():
    f = np.array([0.7, 2.0, 9.5, 17.0, 40.0])
    H = QC.load_frf(f)
    for fi, h in zip(f, H):
        _, zu, kt, _ = _cramer(QC, fi)
        assert h == pytest.approx(kt * (1 - zu), rel=1e-10)
    plain = replace(QC, kappa=0.0, c_t=0.0)
    assert abs(plain.load_frf(1e-4)[0]) < 1e-3                       # a slow road lifts the whole corner: no load
    assert plain.load_frf(1e4)[0] == pytest.approx(plain.k_t, rel=1e-2)   # a fast one only squeezes the tire
    assert QC.load_frf(3.0).shape == (1,)


def test_stability_of_the_coupled_corner():
    assert QC.is_stable() is True
    assert replace(QC, c=0.0, c_t=0.0, kappa=0.0).is_stable() is False      # undamped: the modes never decay
    assert replace(QC, kappa=0.87).is_stable() is True and replace(QC, kappa=0.89).is_stable() is False
    for K in (0.3, 0.87, 0.89, 1.4):
        q = replace(QC, kappa=K)
        M = np.diag([q.m_s, q.m_u])
        Km = np.array([[q.k, -q.k + K * q.k_t], [-q.k, q.k + (1 - K) * q.k_t]])
        Cm = np.array([[q.c, -q.c + K * q.c_t], [-q.c, q.c + (1 - K) * q.c_t]])
        A = np.block([[np.zeros((2, 2)), np.eye(2)], [-np.linalg.solve(M, Km), -np.linalg.solve(M, Cm)]])
        assert q.is_stable() is bool(np.all(np.linalg.eigvals(A).real < 0))


def test_the_anti_bypass_gain():
    assert cp.kappa_from_anti(30.0, 300.0, 1550.0, 0.6, 1.4) == pytest.approx(0.3 * 300 / (1550 * 0.6) * 1.4)
    for share in (0.0, -0.1):
        with pytest.raises(ValueError) as e:
            cp.kappa_from_anti(30.0, 300.0, 1550.0, share, 1.4)
        assert str(e.value) == "axle_force_share must be > 0."


def test_the_iso_8608_road():
    assert cp.ISO8608_GD0 == {"A": 16e-6, "B": 64e-6, "C": 256e-6, "D": 1024e-6} and cp.N0 == 0.1
    f = np.array([1.0, 5.0])
    want = 64e-6 * ((f / 20.0) / 0.1) ** -2 / 20.0
    assert cp.iso8608_psd(f, 20.0) == pytest.approx(want)
    assert cp.iso8608_psd(f, 20.0, "c") == pytest.approx(4 * want)
    assert cp.iso8608_psd(f, 20.0, "A", waviness=2.5) == pytest.approx(16e-6 * ((f / 20.0) / 0.1) ** -2.5 / 20.0)


def _integral(fun, f):
    return float(np.trapezoid(fun, f))


def test_load_variation_integrates_the_load_psd():
    f = np.geomspace(0.5, 50.0, 2000)
    S = np.array([abs(_cramer(QC, fi)[2] * (1 - _cramer(QC, fi)[1])) ** 2 for fi in f]) * cp.iso8608_psd(f, 18.0, "C")
    var = _integral(S, f)
    hi = f >= 10.0
    r = cp.load_variation(QC, 18.0, "C")
    assert r["sigma_dFz_N"] == pytest.approx(math.sqrt(var), rel=1e-9)
    assert r["ratio"] == pytest.approx(math.sqrt(var) / (76.0 * 9.81), rel=1e-9)
    assert r["peak_hz"] == pytest.approx(f[np.argmax(S)])
    assert r["share_above_10hz"] == pytest.approx(_integral(S[hi], f[hi]) / var, rel=1e-9)
    assert r["stable"] is True
    assert cp.load_variation(QC, 18.0)["sigma_dFz_N"] == pytest.approx(r["sigma_dFz_N"] / 2, rel=1e-9)  # B vs C
    low = cp.load_variation(QC, 18.0, "C", f_lo=0.5, f_hi=9.0, n=300)
    assert low["share_above_10hz"] == 0.0
    edge = cp.load_variation(QC, 18.0, "C", f_lo=0.5, f_hi=10.0, n=300)     # one sample at 10 Hz: no band to integrate
    assert edge["share_above_10hz"] == 0.0
    wide = cp.load_variation(QC, 18.0, "C", f_lo=0.2, f_hi=80.0, n=500)
    assert wide["sigma_dFz_N"] > r["sigma_dFz_N"] * 0.99


def test_body_acceleration_integrates_the_sprung_response():
    f = np.geomspace(0.5, 50.0, 2000)
    acc = []
    for fi in f:
        zs, _, _, s = _cramer(QC, fi)
        acc.append(abs(s * s * zs) ** 2)
    want = math.sqrt(_integral(np.array(acc) * cp.iso8608_psd(f, 18.0, "B"), f))
    assert cp.body_accel_rms(QC, 18.0) == pytest.approx(want, rel=1e-9)
    f2 = np.geomspace(1.0, 20.0, 300)
    acc2 = [abs(_cramer(QC, fi)[3] ** 2 * _cramer(QC, fi)[0]) ** 2 for fi in f2]
    assert cp.body_accel_rms(QC, 18.0, "A", f_lo=1.0, f_hi=20.0, n=300) == pytest.approx(
        math.sqrt(_integral(np.array(acc2) * cp.iso8608_psd(f2, 18.0, "A"), f2)), rel=1e-9)


def test_the_damper_speed_distribution_is_gaussian_about_the_rms_shaft_speed():
    q = replace(QC, kappa=0.0, c_t=0.0)
    r = cp.damper_speed_distribution(q, 18.0, knee_ms=0.05, motion_ratio=0.8)
    f = np.linspace(0.1, 50.0, 4000)
    H = []
    for fi in f:
        zs, zu, _, s = _cramer(q, fi)
        H.append(abs(s * (zs - zu) * 0.8) ** 2)
    sigma = math.sqrt(_integral(np.array(H) * cp.iso8608_psd(f, 18.0, "B"), f))
    assert r["shaft_v_rms_ms"] == pytest.approx(sigma, rel=1e-9)
    assert r["share_below_knee"] == pytest.approx(math.erf(0.05 / (sigma * math.sqrt(2))), rel=1e-12)
    assert r["share_above_knee"] == pytest.approx(1 - r["share_below_knee"])
    one = cp.damper_speed_distribution(q, 18.0, knee_ms=0.05)
    assert one["shaft_v_rms_ms"] == pytest.approx(sigma / 0.8, rel=1e-9)
    c = cp.damper_speed_distribution(q, 18.0, 0.05, road_class="C", f_lo=0.5, f_hi=30.0, n=1000)
    f3 = np.linspace(0.5, 30.0, 1000)
    H3 = [abs(_cramer(q, fi)[3] * (_cramer(q, fi)[0] - _cramer(q, fi)[1])) ** 2 for fi in f3]
    assert c["shaft_v_rms_ms"] == pytest.approx(math.sqrt(_integral(np.array(H3) * cp.iso8608_psd(f3, 18.0, "C"),
                                                                   f3)), rel=1e-9)


# ------------------------------------------------------------------ damper synthesis
Q0 = replace(QC, kappa=0.0)


def test_critical_damping_is_on_the_ride_rate():
    assert ds._crit(Q0) == pytest.approx(2 * math.sqrt(30000.0 * 120000.0 / 150000.0 * 65.0))


def test_the_zeta_sweep_sets_damping_from_the_ratio():
    out = ds.zeta_sweep(Q0, [0.2, 0.5], speed_ms=12.0, road_class="C")
    for row, z in zip(out, (0.2, 0.5)):
        r = cp.load_variation(replace(Q0, c=z * ds._crit(Q0)), 12.0, "C")
        assert row == {"zeta": z, "ratio": r["ratio"], "stable": r["stable"]}
    assert ds.zeta_sweep(Q0, [0.3])[0]["ratio"] == cp.load_variation(replace(Q0, c=0.3 * ds._crit(Q0)), 15.0,
                                                                     "B")["ratio"]


def test_the_grip_optimum_is_the_minimum_of_the_load_variation():
    zs = np.linspace(0.05, 1.5, 300)
    ratio = [cp.load_variation(replace(Q0, c=z * ds._crit(Q0)), 15.0, "B")["ratio"] for z in zs]
    z_opt = ds.grip_optimal_zeta(Q0)
    assert z_opt == pytest.approx(zs[int(np.argmin(ratio))], abs=0.01)
    assert ds.grip_optimal_zeta(Q0, lo=0.05, hi=0.1) == pytest.approx(0.1, abs=1e-6)   # clamps to its bracket
    assert ds.grip_optimal_zeta(Q0, lo=1.2, hi=1.5) == pytest.approx(1.2, abs=1e-6)
    assert ds.grip_optimal_zeta(Q0, speed_ms=25.0, road_class="C") != z_opt


def test_body_band_velocity_is_the_suspension_speed_below_four_hertz():
    f = np.geomspace(0.3, 4.0, 400)
    rel = [abs(_cramer(Q0, fi)[3] * (_cramer(Q0, fi)[1] - _cramer(Q0, fi)[0])) ** 2 for fi in f]
    want = math.sqrt(_integral(np.array(rel) * cp.iso8608_psd(f, 15.0, "B"), f))
    assert ds.body_band_velocity_rms(Q0) == pytest.approx(want, rel=1e-9)
    f2 = np.geomspace(0.3, 8.0, 400)
    rel2 = [abs(_cramer(QC, fi)[3] * (_cramer(QC, fi)[1] - _cramer(QC, fi)[0])) ** 2 for fi in f2]
    assert ds.body_band_velocity_rms(QC, 20.0, "C", f_hi=8.0) == pytest.approx(
        math.sqrt(_integral(np.array(rel2) * cp.iso8608_psd(f2, 20.0, "C"), f2)), rel=1e-9)


def test_the_synthesised_damper():
    r = ds.synthesize(Q0, motion_ratio=0.8)
    cc = ds._crit(Q0)
    zg = ds.grip_optimal_zeta(Q0)
    lo, hi = 0.65 * cc, zg * cc
    v = ds.body_band_velocity_rms(replace(Q0, c=lo))
    knee = max(1.5 * v * 0.8, 0.005)
    c = r["curve"]
    assert (c.c_bump_low, c.c_bump_high, c.c_reb_low, c.c_reb_high) == pytest.approx(
        (lo / 0.64, hi / 0.64, 1.5 * lo / 0.64, 1.5 * hi / 0.64))
    assert (c.v_knee, c.is_calibrated, c.source) == (pytest.approx(knee), False, "synthesised (damper_synthesis)")
    assert r == pytest.approx({"curve": c, "zeta_grip": zg, "zeta_body": 0.65, "c_wheel_low": lo, "c_wheel_high": hi,
                               "knee_shaft_ms": knee, "body_band_v_rms_ms": v,
                               "ratio_at_grip": cp.load_variation(replace(Q0, c=hi), 15.0, "B")["ratio"]})
    r2 = ds.synthesize(Q0, 1.0, speed_ms=20.0, road_class="A", zeta_body=0.7, knee_factor=0.001, rebound_ratio=2.0)
    assert r2["knee_shaft_ms"] == 0.005 and r2["curve"].c_reb_low == pytest.approx(2.0 * r2["curve"].c_bump_low)
    assert r2["zeta_grip"] == pytest.approx(ds.grip_optimal_zeta(Q0, 20.0, "A"))
    assert r2["c_wheel_low"] == pytest.approx(0.7 * cc)
    assert r2["ratio_at_grip"] == pytest.approx(cp.load_variation(replace(Q0, c=r2["c_wheel_high"]), 20.0, "A")["ratio"])
    assert r2["body_band_v_rms_ms"] == pytest.approx(ds.body_band_velocity_rms(replace(Q0, c=0.7 * cc), 20.0, "A"))


def test_the_transient_share_and_the_front_rate_that_gives_a_target():
    s = ds.transient_front_share(3000.0, 2500.0, 1250.0, 1200.0)
    f, r = 3000.0 * 1.25 ** 2, 2500.0 * 1.2 ** 2
    assert s == pytest.approx(f / (f + r))
    c = ds.front_rate_for_share(0.53, 2500.0, 1250.0, 1200.0)
    assert ds.transient_front_share(c, 2500.0, 1250.0, 1200.0) == pytest.approx(0.53)
    assert c == pytest.approx(0.53 / 0.47 * r / 1.25 ** 2)


def test_tire_damping_split_by_the_bypass_decides_stability():
    q = cp.QuarterCar(m_s=70.0, m_u=12.0, k=30000.0, c=50.0, k_t=150000.0, c_t=500.0, kappa=0.8)
    k, c, kt, ct, K = q.k, q.c, q.k_t, q.c_t, q.kappa
    Mi = np.diag([1 / 70.0, 1 / 12.0])
    Km = np.array([[k, -k + K * kt], [-k, k + (1 - K) * kt]])
    Cm = np.array([[c, -c + K * ct], [-c, c + (1 - K) * ct]])
    A = np.block([[np.zeros((2, 2)), np.eye(2)], [-Mi @ Km, -Mi @ Cm]])
    assert q.is_stable() is bool(np.all(np.linalg.eigvals(A).real < 0)) is True


def test_the_synthesis_reports_the_ratio_on_its_own_road_class():
    r = ds.synthesize(QC, 0.9, speed_ms=14.0, road_class="C")
    want = cp.load_variation(replace(QC, c=r["c_wheel_high"]), 14.0, "C")["ratio"]
    assert r["ratio_at_grip"] == pytest.approx(want)
    assert r["curve"].is_calibrated is False and r["zeta_grip"] < ds.ZETA_HI
    lv = cp.load_variation(QC, 15.0, "C")
    assert lv["sigma_dFz_N"] ** 2 > 0 and 0 < lv["share_above_10hz"] < 1
