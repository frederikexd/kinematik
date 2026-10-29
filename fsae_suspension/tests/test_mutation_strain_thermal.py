"""suspension/strain_thermal.py: Stefan-Boltzmann flux, the ring's Fourier modes against the flux's Fourier coefficients
integrated numerically here, the shield factor from series radiative resistances, the three-sensor harmonic fit on a
field built from a known amplitude and direction, and the Poisson bridge's sum over a field it must cancel."""
import math

import numpy as np
import pytest

from suspension import strain_thermal as st


def test_materials_and_the_radiant_flux():
    assert st.MATERIALS == {"4130": (42.0, 7850.0, 460.0), "6061": (167.0, 2700.0, 896.0),
                            "7075": (130.0, 2810.0, 960.0), "cfrp": (5.0, 1550.0, 900.0)}
    assert st._SIGMA == 5.670374419e-8
    q = st.radiant_flux(400.0, 40.0, 0.3)
    assert q == pytest.approx(0.8 * 5.670374419e-8 * 0.3 * (673.15 ** 4 - 313.15 ** 4))
    assert st.radiant_flux(400.0, 40.0, 0.3, emissivity_source=0.5) == pytest.approx(q * 0.5 / 0.8)


def test_the_ring_gradient_and_its_time_constant():
    r = st.ring_gradient(2000.0, 25.4, 1.65)
    cond = 42.0 * 1.65e-3 / 0.0127 ** 2
    assert r == pytest.approx({"delta_T_c": 2 * (0.8 * 2000.0 / 2) / (100.0 + cond),
                               "tau_s": 7850.0 * 460.0 * 1.65e-3 / (100.0 + cond), "conduction_w_m2k": cond})
    r = st.ring_gradient(2000.0, 25.4, 1.65, material="cfrp", h_w_m2k=20.0, absorptivity=0.9)
    cond = 5.0 * 1.65e-3 / 0.0127 ** 2
    assert r["delta_T_c"] == pytest.approx(2 * (0.9 * 1000.0) / (40.0 + cond))
    assert r["tau_s"] == pytest.approx(1550.0 * 900.0 * 1.65e-3 / (40.0 + cond))


def test_a_shield_adds_two_surface_resistances():
    assert st.shield_factor() == pytest.approx((1 / 0.8 + 1 / 0.8 - 1) / ((1 / 0.8 + 10 - 1) + (10 + 1 / 0.8 - 1)))
    assert st.shield_factor(0.05, 0.9, 0.7) == pytest.approx(
        (1 / 0.9 + 1 / 0.7 - 1) / ((1 / 0.9 + 20 - 1) + (20 + 1 / 0.7 - 1)))


def test_apparent_strain_and_two_misaligned_sensors():
    assert st.apparent_microstrain(3.0) == pytest.approx(5.4) and st.apparent_microstrain(3.0, 1.2) == pytest.approx(3.6)
    assert st.two_sensor_reading(4.0, 60.0) == pytest.approx(2.0)


def test_three_sensors_recover_amplitude_and_direction():
    for amp, phi, base in ((3.0, 35.0, 20.0), (1.2, 250.0, -4.0), (0.5, 0.0, 0.0)):
        T = [base + 0.5 * amp * math.cos(math.radians(t - phi)) for t in (0.0, 120.0, 240.0)]
        r = st.first_harmonic_from_three(*T)
        assert r["delta_T_c"] == pytest.approx(amp) and r["direction_deg"] == pytest.approx(phi % 360.0, abs=1e-9)


def test_ring_modes_carry_the_fourier_coefficients_of_the_absorbed_flux():
    th = np.linspace(-math.pi, math.pi, 400001)
    flux = np.maximum(np.cos(th), 0.0)
    h = st.ring_harmonics(1500.0, 25.4, 1.65, n_max=6)
    assert set(h) == {1, 2, 3, 4, 5, 6}
    cond = 42.0 * 1.65e-3 / 0.0127 ** 2
    for n, a in h.items():
        cn = np.trapezoid(flux * np.cos(n * th), th) / math.pi
        assert a * (100.0 + n * n * cond) / (0.8 * 1500.0) == pytest.approx(cn, abs=1e-8)
    assert set(st.ring_harmonics(1500.0, 25.4, 1.65)) == {1, 2, 3, 4}
    h2 = st.ring_harmonics(1500.0, 25.4, 1.65, material="6061", h_w_m2k=10.0, absorptivity=0.5, n_max=2)
    c6 = 167.0 * 1.65e-3 / 0.0127 ** 2
    assert h2[2] == pytest.approx(0.5 * 1500.0 * 2 / (3 * math.pi) / (20.0 + 4 * c6))


def test_the_ring_temperature_is_the_cosine_series_about_the_source():
    h = {1: 2.0, 2: -0.5, 3: 0.1}
    t = st.ring_temperature(h, 70.0, source_deg=30.0)
    assert t == pytest.approx(sum(a * math.cos(n * math.radians(40.0)) for n, a in h.items()))
    assert st.ring_temperature(h, 0.0) == pytest.approx(1.6)


def test_the_poisson_bridge_cancels_what_it_should():
    assert st.poisson_bridge_apparent_ue((5.0, 5.0), (5.0, 5.0)) == 0.0
    assert st.poisson_bridge_apparent_ue((6.0, 4.0), (5.0, 5.0)) == pytest.approx(0.0)
    assert st.poisson_bridge_apparent_ue((6.0, 6.0), (5.0, 5.0), 1.5, 0.25) == pytest.approx(1.5 * 2 / 2.5)
    r = st.bridge_thermal_error(2500.0, 25.4, 1.65)
    h = st.ring_harmonics(2500.0, 25.4, 1.65)
    # odd modes and the fourth cancel across the bridge; the second reads 4 a2 / (2 (1 + nu)) per unit mismatch
    assert r["side_to_side_c"] == pytest.approx(2 * h[1]) and r["second_harmonic_c"] == pytest.approx(h[2])
    assert r["apparent_ue"] == pytest.approx(1.8 * 4 * h[2] / 2.6)
    beside = st.bridge_thermal_error(2500.0, 25.4, 1.65, transverse_at_deg=0.0)
    assert beside["apparent_ue"] == pytest.approx(0.0, abs=1e-12)
    turned = st.bridge_thermal_error(2500.0, 25.4, 1.65, source_deg=90.0, material="6061", mismatch_ue_per_c=1.0,
                                     h_w_m2k=30.0)
    h6 = st.ring_harmonics(2500.0, 25.4, 1.65, "6061", 30.0)
    ta = (st.ring_temperature(h6, 0.0, 90.0), st.ring_temperature(h6, 180.0, 90.0))
    tt = (st.ring_temperature(h6, 90.0, 90.0), st.ring_temperature(h6, 270.0, 90.0))
    assert turned["apparent_ue"] == pytest.approx(1.0 * (sum(ta) - sum(tt)) / 2.6)
