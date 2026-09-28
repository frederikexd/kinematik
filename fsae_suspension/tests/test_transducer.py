# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Transducer design and calibration reduction: closed forms, the seminar's
worked relations, and reproduction of a published calibration certificate."""
import math

import numpy as np
import pytest

from suspension import transducer as td
from suspension.calibration import certificate_reduction, loading_schedule, uncertainty_budget


def test_tube_strains_match_the_paper():
    s = td.tube_strains(4799.0, 5800.0, 5800.0, 15.875, 0.889)
    assert s["axial_ue"] == pytest.approx(559.0, abs=1.0)
    assert s["bending_ue"] == pytest.approx(190.0, abs=1.0)
    assert s["torsion_shear_ue"] == pytest.approx(246.0, abs=1.5)


def test_bridge_output_and_arrangements():
    full = td.bridge_output_mvv(559.0, 2.1, 0.29)
    assert full == pytest.approx(2.1 / 4 * 2 * 1.29 * 559e-6 * 1e3)
    assert td.bridge_output_mvv(559.0, 2.1, 0.29, "poisson_half") == pytest.approx(full / 2)


def test_excitation_limit_inverts_power_density():
    v = td.excitation_limit_v(1000.0, 2.25, 0.8)
    assert td.grid_power_density(v, 1000.0, 2.25) == pytest.approx(0.8)
    assert td.grid_power_density(5.0, 350.0, 2.25) > 1.6          # 5 V overcooks 350 ohm


def test_six_wire_case_and_lead_drift():
    assert td.four_wire_loss(1000.0, 0.0)["loss_pct"] == 0.0
    w = td.four_wire_loss(350.0, 1.07, dT_c=20.0)
    assert 0.5 < w["loss_pct"] < 0.7 and w["drift_pct"] > 0.0


def test_dynamic_error_matches_the_paper():
    assert td.dynamic_amplitude_error(200.0, 1830.0) == pytest.approx(1.21, abs=0.01)
    assert td.dynamic_amplitude_error(200.0, 2830.0) == pytest.approx(0.50, abs=0.01)


def test_rejection_perfect_gauges_and_averaging():
    perfect = td.rejection_monte_carlo(2, 0.0, 0.0, 0.0, n=2000)
    assert perfect["bending_p95_ue"] < 1e-9 and perfect["torsion_p95_ue"] < 1e-9
    two = td.rejection_monte_carlo(2, 1.0, 1.0, 0.002, n=20000)
    four = td.rejection_monte_carlo(4, 1.0, 1.0, 0.002, n=20000)
    assert four["pass_fraction"] > two["pass_fraction"]


def test_loading_schedule_follows_the_seminar_cycle():
    s = loading_schedule(5000.0)
    assert sum(p["phase"] == "preload" for p in s) == 5              # 3 + 1 per rotation
    rec = [p for p in s if p["recorded"]]
    assert len(rec) == 33                                            # 11 points x 3 orientations
    assert [p["load_n"] for p in rec[:11]] == [0, 1000, 2000, 3000, 4000, 5000, 4000, 3000, 2000, 1000, 0]
    assert {p["orientation_deg"] for p in rec} == {0.0, 120.0, 240.0}


LOADS = [0, 1.32, 2.64, 3.96, 5.28, 6.6]
INC = [[0.0012, 0.1885, 0.3663, 0.5451, 0.7262, 0.9081], [0.0012, 0.1886, 0.3662, 0.5451, 0.7260, 0.9079],
       [0.0012, 0.1885, 0.3662, 0.5450, 0.7262, 0.9078]]
DEC = [[0.0012, 0.1901, 0.3749, 0.5562, 0.7301, 0.9081], [0.0012, 0.1901, 0.3749, 0.5563, 0.7299, 0.9079],
       [0.0011, 0.1901, 0.3750, 0.5561, 0.7299, 0.9078]]


def test_reproduces_the_seminar_calibration_certificate():
    r = certificate_reduction(LOADS, INC, DEC)
    assert r["nonlinearity_pct"][1:] == pytest.approx([-0.485, -0.009, 0.239, 0.139, -0.154], abs=0.005)
    assert r["hysteresis_pct"][2:5] == pytest.approx([-0.958, -1.227, -0.425], abs=0.005)
    assert r["poly_B"][1:] == pytest.approx([0.1830, 0.3650, 0.5461, 0.7262, 0.9054], abs=0.0002)
    assert r["response_coeffs"][1] == pytest.approx(0.1390, abs=0.0002)
    assert r["force_coeffs"][1] == pytest.approx(7.1934, abs=0.005)


def test_uncertainty_budget_rss():
    b = uncertainty_budget({"a": (0.2, "normal_k2"), "b": (math.sqrt(3) * 0.1, "rectangular")})
    assert b["combined_pct"] == pytest.approx(math.sqrt(0.1 ** 2 + 0.1 ** 2))
    assert b["expanded_pct"] == pytest.approx(2 * b["combined_pct"])


def test_series_arms_quarter_the_gauge_voltage():
    one = td.grid_power_density(2.5, 1000.0, 2.25, n_series=1)
    two = td.grid_power_density(5.0, 1000.0, 2.25, n_series=2)
    assert two == pytest.approx(one)
    assert td.excitation_limit_v(1000.0, 2.25, 0.8, 2) == pytest.approx(2 * td.excitation_limit_v(1000.0, 2.25, 0.8))


def test_euler_and_shunt():
    s = td.tube_section(15.875, 0.889)
    assert td.euler_load_n(486.0, 15.875, 0.889) == pytest.approx(math.pi ** 2 * 205000 * s["I_mm4"] / 486 ** 2)
    rs = td.shunt_resistor_ohm(2000.0, 0.607)
    r_eff = 2000.0 * rs / (2000.0 + rs)
    assert (2000.0 - r_eff) / 2000.0 / 4 * 1e3 == pytest.approx(0.607, rel=0.01)
