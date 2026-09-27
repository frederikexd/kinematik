# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Each fitter must recover the parameters that generated its synthetic test."""
import math

import numpy as np
import pytest

from suspension import measurement_fits as mf
from suspension import target_derivation as td
from suspension import contact_patch as cp
from suspension.brake_thermal import BrakeThermalParams


def test_pneumatic_trail_recovered():
    a = np.linspace(0.5, 12.0, 40)
    fy = -1500.0 * np.sin(1.3 * np.arctan(0.25 * a))
    tp = 14.0 * np.cos(0.5 * np.pi * a / 8.5)
    mz = -fy * tp / 1000.0
    r = mf.fit_pneumatic_trail(a, fy, mz)
    assert r["t0_mm"] == pytest.approx(14.0, rel=0.01)
    assert r["alpha0_deg"] == pytest.approx(8.5, abs=0.05)


def test_carcass_share_recovered_and_consistent_with_rate_model():
    p = np.array([70.0, 83.0, 100.0, 110.0])
    k = np.array([td.tire_rate_at_pressure(120.0, 83.0, x, 0.3) for x in p])
    r = mf.fit_carcass_share(p, k, 83.0)
    assert r["carcass_share"] == pytest.approx(0.3, abs=1e-9)
    assert r["rate_at_ref_n_per_mm"] == pytest.approx(120.0)


def test_road_class_recovered_from_a_synthetic_iso_profile():
    rng = np.random.default_rng(3)
    dx, N = 0.05, 2 ** 16
    n = np.fft.rfftfreq(N, dx)
    Gd = np.zeros_like(n)
    Gd[1:] = cp.ISO8608_GD0["B"] * (n[1:] / cp.N0) ** -2.0
    amp = np.sqrt(Gd * (n[1] - n[0]) / 2.0)
    phase = rng.uniform(0, 2 * np.pi, len(n))
    spec = amp * np.exp(1j * phase) * N
    z = np.fft.irfft(spec, N)
    r = mf.fit_road_class(z, dx)
    assert r["road_class"] == "B"
    assert r["waviness"] == pytest.approx(2.0, abs=0.15)


def test_fade_law_recovered():
    p = BrakeThermalParams(enable_fade=True, T_fade_onset_c=450.0, fade_per_C=0.0012)
    T = np.linspace(200.0, 700.0, 60)
    r = mf.fit_fade_law(T, [p.mu_fade(x) for x in T])
    assert r["T_fade_onset_c"] == pytest.approx(450.0, abs=3.0)
    assert r["fade_per_C"] == pytest.approx(0.0012, rel=0.03)


def test_plunge_friction_recovered():
    T = np.array([50.0, 150.0, 250.0, 350.0])
    F = 0.04 * T * 1000.0 / 20.0
    assert mf.fit_plunge_friction(T, F, 20.0)["mu_plunge"] == pytest.approx(0.04)


def test_residual_profile_interpolates_and_mirrors():
    prof = mf.residual_profile([0.0, 1.0, 2.0], [300.0, 100.0, -50.0], 6.0)
    assert prof(3.0) == pytest.approx(300.0)        # top surface
    assert prof(-3.0) == pytest.approx(300.0)       # mirrored bottom surface
    assert prof(2.0) == pytest.approx(100.0)        # 1 mm deep
    one = mf.residual_profile([0.0, 1.0], [300.0, 0.0], 6.0, symmetric=False)
    assert one(-3.0) == 0.0
