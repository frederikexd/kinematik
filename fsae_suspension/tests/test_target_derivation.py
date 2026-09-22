# ============================================================================
#  KinematiK — tests for the section 4 target derivation and tire sensitivity
#  Copyright (c) 2026 Frederik Thio. Open source.
# ============================================================================
"""The target chain must reproduce the paper's section 4 and table 3, and the
tire-sensitivity study must say which targets the tire inputs actually move."""
from dataclasses import replace

import pytest

from suspension import target_derivation as td

V, T = td.Vehicle(), td.Tire()


@pytest.mark.parametrize("anti,ax,ay,axle,paper", [
    (68.0, 1.06, 0.0, "front", 1.43), (68.0, 1.06, 1.06, "front", 2.02),
    (-25.7, 1.06, 0.0, "rear", 2.72), (-25.7, 1.06, 1.06, "rear", 3.83),
    (39.1, 1.06, 1.06, "rear", 2.66)])
def test_equation_1_reproduces_table_3(anti, ax, ay, axle, paper):
    assert td.f_min(V, anti, ax, ay, axle) == pytest.approx(paper, abs=0.015)


def test_camber_chain_reproduces_section_4():
    assert td.camber_gain_needed(V, T, -2.5) == pytest.approx(-0.040, abs=0.001)
    assert td.camber_gain_needed(V, T, -1.5) == pytest.approx(-0.12, abs=0.002)
    phi, z = td.roll(V, 1.5)
    assert phi == pytest.approx(1.17, abs=0.01) and z == pytest.approx(12.3, abs=0.1)


def test_derived_floors_match_the_paper():
    t = td.targets(V, T)
    assert t["anti_dive_floor_pct"] == pytest.approx(39.0, abs=0.5)
    assert t["anti_squat_floor_pct"] == pytest.approx(23.0, abs=0.5)


def test_floor_and_frequency_are_inverse():
    a = td.anti_floor_pct(V, 3.0, 1.06, 1.06, "rear")
    assert td.f_min(V, a, 1.06, 1.06, "rear") == pytest.approx(3.0, abs=1e-9)


def test_static_camber_absorbs_a_new_optimum():
    """A different camber optimum is an alignment change, not a hardpoint one."""
    g = -0.0357
    for opt in (-1.37, -1.83, -2.29):
        t = replace(T, camber_opt_deg=opt)
        s = td.static_camber_needed(V, t, g)
        assert td.loaded_camber_error(V, t, s, g) == pytest.approx(0.0, abs=1e-12)


def test_camber_sensitivity_moves_no_target():
    a = td.targets(V, T)
    b = td.targets(V, replace(T, camber_sensitivity_pct_per_deg=1.8))
    assert a == b


def test_higher_grip_raises_the_anti_floors():
    lo = td.targets(V, replace(T, peak_mu=1.16))
    hi = td.targets(V, replace(T, peak_mu=1.94))
    assert lo["anti_squat_floor_pct"] < 23.0 < hi["anti_squat_floor_pct"]
    assert lo["anti_dive_floor_pct"] < 39.0 < hi["anti_dive_floor_pct"]


def test_sensitivity_study_shape():
    c = [td.Corner("rear", -2.6, -0.0238, 34.28, "rear")]
    rows = td.tire_sensitivity(c)
    assert rows[0]["case"] == "reference" and len(rows) == 7
    assert all("rear" in r for r in rows)


def test_derived_ride_frequencies():
    assert td.ride_frequency_derived(V, T, 61.17, "front") == pytest.approx(2.47, abs=0.01)
    assert td.ride_frequency_derived(V, T, 34.28, "rear") == pytest.approx(3.07, abs=0.01)
    assert td.ride_frequency_derived(V, T, 34.28, "rear", 0.0) == pytest.approx(2.77, abs=0.01)


def test_roll_gradient_ceilings():
    assert td.roll_gradient_ceiling(V, T, 34.28, "rear", 3.0) == pytest.approx(0.894, abs=0.002)
    assert td.roll_gradient_ceiling(V, T, 34.28, "rear", 3.0, 0.10) == pytest.approx(0.751, abs=0.002)


def test_roll_centre_band_as_load_transfer():
    assert 18.0 * td.share_per_mm_front_rc(V) == pytest.approx(0.0111, abs=0.0003)


def test_balance_target_is_a_mild_understeer_margin():
    assert td.neutral_front_share(V) == pytest.approx(0.511, abs=0.002)
    assert td.axle_capacity_ratio(V, 0.53) == pytest.approx(0.993, abs=0.001)
    assert td.axle_capacity_ratio(V, 0.48) > 1.0 > td.axle_capacity_ratio(V, 0.53)
