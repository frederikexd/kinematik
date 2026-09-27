# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Section 6 checks, halfshaft Goodman sizing, bridge exactness, shunt and
calibration-header guards, and the twelve-source GUM budget."""
import math

import pytest

from suspension import setup_checks as sc
from suspension import transducer as td
from suspension.calibration import check_calibration_header, gum_budget_s74
from suspension.halfshaft import goodman_min_diameter
from suspension.kinematics import Hardpoints


def test_thermal_fit_matches_the_closed_form():
    r = sc.thermal_fit(80.0, 180.0, 9.525)
    assert r["span_growth_um"] == pytest.approx(171.36, abs=0.01)
    assert r["bore_opening_um"] == pytest.approx(9.068, abs=0.001)
    assert r["slot_margin"] == pytest.approx(2.918, abs=0.001)


def test_shims_and_thread_on_the_linkage():
    hp = Hardpoints.default()
    lat, vert = sc.camber_per_shim(hp, 0.5, "lateral"), sc.camber_per_shim(hp, 0.5, "vertical")
    assert abs(lat) > abs(vert)                      # lateral shims are the camber adjuster
    assert abs(sc.camber_per_shim(hp, 1.0, "lateral")) == pytest.approx(2 * abs(lat), rel=0.02)
    arm = sc.toe_arm_mm(hp)
    assert 20.0 < arm < 200.0
    r = sc.toe_thread_resolution(66.0)
    assert r["toe_per_flat_deg"] == pytest.approx(0.153, abs=0.001)
    assert r["backlash_toe_deg"] == pytest.approx(0.0217, abs=0.0005)


def test_mass_bom_and_buckling():
    hp = Hardpoints.default()
    m = sc.tube_mass_g(hp)
    assert m["g_per_m"] == pytest.approx(328.6, abs=0.5)
    assert sc.bom_cost(32, 4.50, 36, 12.0, 4, 16.0)["joints_usd"] == pytest.approx(496.0)
    e = sc.euler_margins(hp, {"TR": 1000.0})
    L = e["TR"]["length_mm"]
    I = math.pi / 64 * (15.875 ** 4 - (15.875 - 1.778) ** 4)
    assert e["TR"]["euler_n"] == pytest.approx(math.pi ** 2 * 205000 * I / L ** 2)
    assert e["UF"]["fos"] == math.inf


def test_hand_torque_and_twist_scaling():
    r = sc.hand_torque_at_collapse(1934.0, 255.6, 12.67, 5.907)
    assert r["hand_nm"] == pytest.approx((1934.0 + 255.6) * 0.01267 / 5.907)
    assert sc.frame_twist_toe_scaled(0.17, 1.5, 2.0) == pytest.approx(0.2267, abs=1e-4)


def test_goodman_counts_torsion_once():
    good = goodman_min_diameter(355.63, -201.30, kc=1.0, von_mises=True)
    double = goodman_min_diameter(355.63, -201.30, kc=0.59, von_mises=True)
    assert 22.0 < good["d_min_mm"] < 24.5
    assert double["d_min_mm"] > good["d_min_mm"] + 3.0          # the double count inflates it
    assert good["T_alt_nm"] == pytest.approx(278.47, abs=0.01)
    assert good["ka"] == pytest.approx(0.723, abs=0.001)


def test_bridge_exact_and_shunt_monitor():
    b = td.poisson_bridge_exact_mvv(559.0)
    assert b["nonlinearity_pct"] == pytest.approx(-0.0417, abs=0.0005)
    assert td.shunt_ratio_check(0.6320, 0.6315)["inspect"] is False
    assert td.shunt_ratio_check(0.6330, 0.6315)["inspect"] is True


def test_calibration_header_guard():
    cal = {"channel_id": "FL_LF", "transducer_serial": "T07", "valid_until": "2027-03-01",
           "coeffs": {"tension": (0.1, 7.2), "compression": (0.1, 7.1)}}
    assert check_calibration_header(cal, {"channel_id": "FL_LF", "transducer_serial": "T07", "date": "2026-10-01"})["ok"]
    bad = check_calibration_header(cal, {"channel_id": "FL_LR", "transducer_serial": "T07", "date": "2027-04-01"})
    assert not bad["ok"] and len(bad["problems"]) == 2


def test_gum_budget_s74_reference_row():
    stated = gum_budget_s74(reference_u_pct=0.050)
    correct = gum_budget_s74(reference_u_pct=0.025)
    assert stated["expanded_pct"] == pytest.approx(0.456, abs=0.002)   # the draft's arithmetic
    assert correct["expanded_pct"] == pytest.approx(0.448, abs=0.002)  # with U 0.05% k 2 -> u 0.025
    assert correct["expanded_pct"] < 0.5
