"""suspension/setup_checks.py: differential expansion, the shim and tie-rod checks re-solved on the linkage, thread
resolution, link lengths and tube mass from the hardpoints, the bill of materials, Euler margins and hand torque in
closed form."""
import math

import numpy as np
import pytest

from suspension import setup_checks as sc
from suspension.kinematics import Hardpoints, SuspensionKinematics

HP = Hardpoints.default()


def _solve(deltas=None):
    return SuspensionKinematics(HP, pickup_deltas=deltas or {}).solve_at_travel(0.0)


def test_differential_expansion():
    r = sc.thermal_fit(60.0, 120.0, 10.0)
    d = 23.6e-6 - 11.7e-6
    assert r == pytest.approx({"span_growth_um": d * 120 * 60 * 1000, "bore_opening_um": d * 10 * 60 * 1000,
                               "slot_margin": 500.0 / (d * 120 * 60 * 1000)})
    r = sc.thermal_fit(60.0, 120.0, 10.0, alpha_upright=12e-6, alpha_pin=12e-6)
    assert (r["span_growth_um"], r["slot_margin"]) == (0.0, math.inf)
    r = sc.thermal_fit(40.0, 100.0, 8.0, alpha_upright=20e-6, alpha_pin=10e-6, slot_clearance_mm=0.2)
    assert r["slot_margin"] == pytest.approx(200.0 / 40.0)


def test_a_shim_at_the_upper_pivots_moves_the_camber_on_the_linkage():
    for direction, v in (("lateral", [0.0, 0.5, 0.0]), ("vertical", [0.0, 0.0, 0.5])):
        v = np.array(v)
        want = _solve({"upper_front_inner": v, "upper_rear_inner": v}).camber - _solve().camber
        assert sc.camber_per_shim(HP, direction=direction) == pytest.approx(want)
    v = np.array([0.0, 1.5, 0.0])
    assert sc.camber_per_shim(HP, 1.5) == pytest.approx(
        _solve({"upper_front_inner": v, "upper_rear_inner": v}).camber - _solve().camber)
    with pytest.raises(KeyError):
        sc.camber_per_shim(HP, direction="diagonal")


def test_upper_arm_span_and_the_toe_arm():
    mid = 0.5 * (np.asarray(HP.upper_front_inner) + np.asarray(HP.upper_rear_inner))
    assert sc.upper_arm_span_mm(HP) == pytest.approx(abs(HP.upper_outer[1] - mid[1]))
    ti, to = np.asarray(HP.tie_rod_inner, float), np.asarray(HP.tie_rod_outer, float)
    u = (ti - to) / np.linalg.norm(ti - to)
    for dl in (0.05, 0.2):
        dt = _solve({"tie_rod_inner": u * dl}).toe - _solve().toe
        assert sc.toe_arm_mm(HP, dl) == pytest.approx(dl / math.tan(math.radians(abs(dt))))
    assert sc.toe_arm_mm(HP) == pytest.approx(sc.toe_arm_mm(HP, 0.05))


def test_thread_resolution():
    r = sc.toe_thread_resolution(120.0)
    p = 25.4 / 24
    assert r == pytest.approx({"pitch_mm": p, "toe_per_flat_deg": math.degrees(math.atan(p / 6 / 120.0)),
                               "band_rod_travel_mm": 120.0 * math.tan(math.radians(0.08)),
                               "band_in_flats": 120.0 * math.tan(math.radians(0.08)) / (p / 6),
                               "backlash_toe_deg": math.degrees(math.atan(0.025 / 120.0))})
    r = sc.toe_thread_resolution(100.0, tpi=28.0, band_deg=0.1, backlash_mm=0.05, flats=4)
    p = 25.4 / 28
    assert r == pytest.approx({"pitch_mm": p, "toe_per_flat_deg": math.degrees(math.atan(p / 4 / 100.0)),
                               "band_rod_travel_mm": 100.0 * math.tan(math.radians(0.1)),
                               "band_in_flats": 100.0 * math.tan(math.radians(0.1)) / (p / 4),
                               "backlash_toe_deg": math.degrees(math.atan(0.05 / 100.0))})


def test_link_lengths_and_tube_mass():
    L = sc.link_lengths(HP)
    assert set(L) == {"UF", "UR", "LF", "LR", "TR", "PR"}
    assert L["TR"] == pytest.approx(np.linalg.norm(np.asarray(HP.tie_rod_inner) - np.asarray(HP.tie_rod_outer)))
    assert L["PR"] == pytest.approx(np.linalg.norm(np.asarray(HP.pushrod_outer) - np.asarray(HP.rocker_pushrod)))
    assert L["UR"] == pytest.approx(np.linalg.norm(np.asarray(HP.upper_rear_inner) - np.asarray(HP.upper_outer)))
    m = sc.tube_mass_g(HP)
    lin = math.pi / 4 * (15.875 ** 2 - (15.875 - 1.778) ** 2) * 7.85
    assert m["g_per_m"] == pytest.approx(lin) and m["lengths_mm"] == L
    assert m["total_g"] == pytest.approx(lin * sum(L.values()) / 1000)
    m = sc.tube_mass_g(HP, od_mm=19.05, wall_mm=0.9, rho_g_cm3=2.7)
    assert m["g_per_m"] == pytest.approx(math.pi / 4 * (19.05 ** 2 - 17.25 ** 2) * 2.7)


def test_bill_of_materials():
    assert sc.bom_cost(10.0, 8.0, 12, 15.0) == {"tube_usd": 80.0, "joints_usd": 180.0, "total_usd": 260.0}
    assert sc.bom_cost(10.0, 8.0, 12, 15.0, ptfe_joints=4, usd_ptfe=30.0, other_usd=50.0) == {
        "tube_usd": 80.0, "joints_usd": 300.0, "total_usd": 430.0}


def test_euler_margins():
    r = sc.euler_margins(HP, {"TR": -1500.0, "UF": 800.0})
    idm = 15.875 - 1.778
    I = math.pi / 64 * (15.875 ** 4 - idm ** 4)
    L = sc.link_lengths(HP)
    for k in L:
        pc = math.pi ** 2 * 205000.0 * I / L[k] ** 2
        assert r[k]["length_mm"] == pytest.approx(L[k]) and r[k]["euler_n"] == pytest.approx(pc)
    assert r["TR"]["fos"] == pytest.approx(r["TR"]["euler_n"] / 1500.0)
    assert r["UF"]["fos"] == pytest.approx(r["UF"]["euler_n"] / 800.0) and r["LR"]["fos"] == math.inf
    r2 = sc.euler_margins(HP, {}, od_mm=19.05, wall_mm=1.2, E=70000.0)
    I2 = math.pi / 64 * (19.05 ** 4 - 16.65 ** 4)
    assert r2["TR"]["euler_n"] == pytest.approx(math.pi ** 2 * 70000.0 * I2 / L["TR"] ** 2)


def test_hand_torque_and_frame_twist_scaling():
    r = sc.hand_torque_at_collapse(2000.0, 800.0, 20.0, 5.0)
    assert r == pytest.approx({"road_wheel_nm": 56.0, "hand_nm": 11.2, "budget_left_nm": -1.2})
    assert sc.hand_torque_at_collapse(2000.0, 800.0, 20.0, 5.0, budget_nm=15.0)["budget_left_nm"] == pytest.approx(3.8)
    assert sc.frame_twist_toe_scaled(0.02, 1.5, 2.0) == pytest.approx(0.02 * 2.0 / 1.5)


def test_camber_in_steer():
    r = sc.camber_in_steer(HP, 15.0)
    out = _solve({"tie_rod_inner": np.array([0.0, 15.0, 0.0])})
    ins = _solve({"tie_rod_inner": np.array([0.0, -15.0, 0.0])})
    assert r == pytest.approx({"static_deg": _solve().camber, "outside_deg": out.camber, "inside_deg": ins.camber,
                               "outside_road_wheel_deg": abs(out.toe), "inside_road_wheel_deg": abs(ins.toe)})


def test_tiny_compressions_and_spans_still_give_finite_margins():
    I = math.pi / 64 * (15.875 ** 4 - (15.875 - 2 * 0.889) ** 4)
    r = sc.euler_margins(HP, {"LF": 0.5})["LF"]
    assert r["fos"] == pytest.approx(math.pi ** 2 * 205000.0 * I / r["length_mm"] ** 2 / 0.5)
    t = sc.thermal_fit(0.01, 50.0, 10.0)
    span = (23.6e-6 - 11.7e-6) * 50.0 * 0.01 * 1000.0
    assert 0 < span < 1 and t["slot_margin"] == pytest.approx(500.0 / span)

