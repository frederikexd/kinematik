"""suspension/measurement_fits.py: each fit recovers the parameter that generated its synthetic test data (a cosine
trail, a linear rate law, an ISO 8608 road built from random-phase harmonics, a fade law, a plunge law), and the
hole-drilling profile mirrors or truncates as documented."""
import math

import numpy as np
import pytest

from suspension import measurement_fits as mf
from suspension.contact_patch import ISO8608_GD0


def test_the_pneumatic_trail_fit_recovers_the_cosine_law():
    a = np.linspace(-10.0, 10.0, 41)
    fy = -1800.0 * np.sin(1.3 * np.arctan(0.25 * a))
    tp = 32.0 * np.cos(0.5 * np.pi * np.abs(a) / 7.0)
    mz = -tp * fy / 1000.0
    r = mf.fit_pneumatic_trail(a, fy, mz)
    assert r["t0_mm"] == pytest.approx(32.0, rel=5e-3) and r["alpha0_deg"] == pytest.approx(7.0, abs=0.03)
    assert r["rms_mm"] < 0.2
    # a near-zero force point would wreck the ratio: it is dropped
    a2, fy2, mz2 = np.append(a, 0.05), np.append(fy, 1.0), np.append(mz, 50.0)
    assert mf.fit_pneumatic_trail(a2, fy2, mz2)["t0_mm"] == pytest.approx(r["t0_mm"], rel=1e-9)
    edge = mf.fit_pneumatic_trail(a2, np.append(fy, 0.0501 * 1800.0 * math.sin(1.3 * math.atan(2.5))), mz2)
    assert edge["rms_mm"] > r["rms_mm"]                  # just above 5% of the peak: kept
    small = mf.fit_pneumatic_trail([0.2, 1.0, 2.0, 3.0], [100.0, 500.0, 900.0, 1200.0],
                                   [-2.0, -10.0, -17.0, -20.0])
    assert 0.5 <= small["alpha0_deg"] <= 6.0


def test_the_carcass_share_from_rates_at_two_or_more_pressures():
    p = [70.0, 85.0, 100.0]
    k = [90.0 + 0.5 * x for x in p]
    r = mf.fit_carcass_share(p, k, 83.0)
    assert r == pytest.approx({"carcass_share": 90.0 / 131.5, "rate_at_ref_n_per_mm": 131.5, "rms_n_per_mm": 0.0},
                              abs=1e-9)
    kn = [k[0] + 1, k[1] - 2, k[2] + 1]
    noisy = mf.fit_carcass_share(p, kn, 83.0)
    b_, a_ = np.polyfit(p, kn, 1)
    assert noisy == pytest.approx({"carcass_share": a_ / (a_ + 83 * b_), "rate_at_ref_n_per_mm": a_ + 83 * b_,
                                   "rms_n_per_mm": math.sqrt(np.mean((np.array(kn) - (a_ + b_ * np.array(p))) ** 2))})
    assert noisy["rms_n_per_mm"] > 0
    for bad in (([80.0], [100.0]), ([80.0, 80.0], [100.0, 101.0])):
        with pytest.raises(ValueError) as e:
            mf.fit_carcass_share(*bad, 83.0)
        assert str(e.value) == "fit_carcass_share needs rates at two or more pressures."


def _road(gd0, w=2.0, length=2000.0, dx=0.05, seed=1):
    """A profile whose one-sided PSD is gd0 (n / 0.1)^-w: random-phase harmonics, summed by inverse FFT."""
    rng = np.random.default_rng(seed)
    N = int(length / dx)
    n = np.fft.rfftfreq(N, dx)
    dn = n[1] - n[0]
    amp = np.zeros(len(n))
    amp[1:] = np.sqrt(2 * gd0 * (n[1:] / 0.1) ** -w * dn)            # cosine amplitude of each harmonic
    X = amp * np.exp(1j * rng.uniform(0, 2 * np.pi, len(n))) * N / 2
    return np.fft.irfft(X, N)


def test_the_road_class_from_a_profile_with_a_known_spectrum():
    z = _road(64e-6)
    r = mf.fit_road_class(z + 0.3, 0.05)                  # an offset is removed first
    assert r["road_class"] == "B" and r["Gd_n0_m3"] == pytest.approx(64e-6, rel=0.25)
    assert r["waviness"] == pytest.approx(2.0, abs=0.15)
    c = mf.fit_road_class(_road(256e-6, w=2.5, seed=2), 0.05, n_lo=0.1, n_hi=1.5, segment=512)
    assert c["road_class"] == "C" and c["waviness"] == pytest.approx(2.5, abs=0.2)
    assert set(ISO8608_GD0) >= {c["road_class"]}
    short = mf.fit_road_class(_road(16e-6, length=40.0, seed=3), 0.05, segment=4096)    # segment longer than the run
    assert short["road_class"] in ("A", "B")


def test_the_fade_law_fit():
    T = np.linspace(100.0, 700.0, 61)
    r = np.where(T > 380.0, np.maximum(1.0 - 0.0012 * (T - 380.0), 0.0), 1.0)
    f = mf.fit_fade_law(T, r)
    assert f["T_fade_onset_c"] == pytest.approx(380.0, abs=3.0) and f["fade_per_C"] == pytest.approx(0.0012, rel=0.02)
    assert f["fade_floor"] == pytest.approx(r.min()) and f["rms"] < 5e-3
    rising = mf.fit_fade_law(T, 1.0 + 0.0005 * np.maximum(T - 300.0, 0.0))
    assert rising["fade_per_C"] == 0.0
    with pytest.raises(ValueError) as e:
        mf.fit_fade_law([300.0, 300.0], [1.0, 0.9])
    assert str(e.value) == "fit_fade_law needs temperatures above an onset."


def test_plunge_friction_from_axial_force_against_torque():
    T = np.array([50.0, 150.0, 300.0])
    F = 0.04 * T * 1000.0 / 20.0
    assert mf.fit_plunge_friction(T, F, 20.0) == pytest.approx({"mu_plunge": 0.04, "rms_n": 0.0}, abs=1e-12)
    r = mf.fit_plunge_friction(T, F + [5.0, -5.0, 0.0], 18.0)
    k = np.sum(T * 1e3 * (F + [5, -5, 0])) / np.sum((T * 1e3) ** 2)
    assert r == pytest.approx({"mu_plunge": k * 18.0,
                               "rms_n": math.sqrt(np.mean((F + [5, -5, 0] - k * T * 1e3) ** 2))})


def test_the_hole_drilling_profile():
    prof = mf.residual_profile([1.0, 0.0, 2.0], [100.0, 300.0, -50.0], thickness_mm=8.0)
    assert prof(4.0) == pytest.approx(300.0) and prof(3.0) == pytest.approx(100.0)       # depths 0 and 1 from the top
    assert prof(2.5) == pytest.approx(25.0)                                                # halfway from 1 to 2 mm
    assert prof(0.0) == pytest.approx(-50.0)                                               # held below 2 mm
    assert prof(-4.0) == pytest.approx(300.0) and prof(-3.0) == pytest.approx(100.0)      # mirrored on the far face
    one = mf.residual_profile([0.0, 1.0, 2.0], [300.0, 100.0, -50.0], 8.0, symmetric=False)
    assert one(4.0) == pytest.approx(300.0) and one(-3.0) == 0.0 and one(1.0) == 0.0
    assert one(2.0) == pytest.approx(-50.0)                                                # exactly the deepest point


def test_two_pressures_half_a_kilopascal_apart_are_enough():
    from suspension import measurement_fits as mfit
    r = mfit.fit_carcass_share([100.0, 100.5], [200.0, 201.0], 100.0)
    assert r["rate_at_ref_n_per_mm"] == pytest.approx(200.0) and r["carcass_share"] == pytest.approx(0.0, abs=1e-6)


def test_a_fade_onset_seen_by_only_two_hot_points():
    from suspension import measurement_fits as mfit
    T = np.array([100.0, 200.0, 300.0, 400.0, 500.0])
    r = 1.0 - 0.001 * np.maximum(T - 380.0, 0.0)
    f = mfit.fit_fade_law(T, r)
    assert f["T_fade_onset_c"] == pytest.approx(380.0, abs=1.4) and f["fade_per_C"] == pytest.approx(0.001, rel=0.05)
    noisy = r + np.array([0.0, 0.004, -0.003, 0.002, -0.002])
    g = mfit.fit_fade_law(T, noisy)
    on, s = g["T_fade_onset_c"], g["fade_per_C"]
    pred = np.where(T > on, 1.0 - s * (T - on), 1.0)
    assert g["rms"] == pytest.approx(math.sqrt(np.mean((noisy - pred) ** 2)), rel=1e-9) and g["rms"] > 0


def test_the_trail_fit_scale_and_threshold():
    from suspension import measurement_fits as mfit
    a = np.array([0.2, 0.4, 0.6, 0.8, 1.2, 1.6])
    fy = 1000.0 * np.sin(np.radians(a) * 20)
    tp = 30.0 * np.cos(0.5 * np.pi * a / 1.0)
    r = mfit.fit_pneumatic_trail(a, fy, -tp * fy / 1000.0)
    assert r["t0_mm"] == pytest.approx(30.0, rel=1e-3) and r["alpha0_deg"] == pytest.approx(1.0, abs=0.02)
    fy2 = np.array([1000.0, 50.0, 800.0, 600.0])
    tp2 = np.array([20.0, 999.0, 15.0, 10.0])
    a2 = np.array([1.0, 2.0, 3.0, 4.0])
    exact = mfit.fit_pneumatic_trail(a2, fy2, -tp2 * fy2 / 1000.0)                 # 50 N is exactly 5%: dropped
    keep = mfit.fit_pneumatic_trail(a2[[0, 2, 3]], fy2[[0, 2, 3]], -(tp2 * fy2)[[0, 2, 3]] / 1000.0)
    assert exact == pytest.approx(keep)


def test_the_road_psd_against_welch():
    from scipy.signal import welch
    from suspension import measurement_fits as mfit
    from suspension.contact_patch import ISO8608_GD0, N0
    dx = 1 / 25.6                                       # bins at k/40 cycles/m: 0.05 and 2.0 are bins exactly
    rng = np.random.default_rng(3)
    for n in (1535, 4096):
        z = np.cumsum(rng.normal(0.0, 1e-3, n)) + 0.3
        r = mfit.fit_road_class(z, dx)
        f, G = welch(z - z.mean(), fs=1 / dx, window=np.hanning(1024), nperseg=1024, noverlap=512, detrend=False,
                     scaling="density")
        m = (f >= 0.05) & (f <= 2.0)
        slope, icpt = np.polyfit(np.log(f[m] / N0), np.log(G[m]), 1)
        assert r["Gd_n0_m3"] == pytest.approx(math.exp(icpt), rel=1e-6) and r["waviness"] == pytest.approx(-slope)
        assert r["road_class"] == min(ISO8608_GD0, key=lambda c: abs(math.log(ISO8608_GD0[c] / math.exp(icpt))))
