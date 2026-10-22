# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The combustion powertrain: closed-form checks of the restrictor, the gearbox, the launch, the fuel integral and
engine braking, then the gate's honesty (undeclared is MISSING, never a pass) and the lap and comparison plumbing."""
import math
import os
import sys
from dataclasses import replace
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension import Hardpoints, SuspensionKinematics, VehicleDynamics, VehicleParams  # noqa: E402
from suspension import ice_powertrain as ice                                              # noqa: E402
from suspension import laptime as lt                                                      # noqa: E402
from suspension import tiremodel as tm                                                    # noqa: E402


def _vd(mass=300.0):
    kin = SuspensionKinematics(Hardpoints.default())
    return VehicleDynamics(VehicleParams(mass=mass), front_kin=kin, rear_kin=kin, tire=tm.default_tire())


ENG = ice.EngineMap.representative_600_i4()
GB = ice.Gearbox()


# ------------------------------------------------------------------ restrictor
def test_choked_flow_matches_the_isentropic_closed_form():
    d, p0, t0, g, r = 20.0, 101325.0, 298.15, 1.4, 287.0
    area = math.pi * 0.020 ** 2 / 4
    expect = area * p0 * math.sqrt(g / (r * t0)) * (2 / (g + 1)) ** ((g + 1) / (2 * (g - 1)))
    assert ice.choked_mass_flow_kg_s(d) == pytest.approx(expect, rel=1e-12)
    assert ice.choked_mass_flow_kg_s(d) == pytest.approx(0.0745, abs=5e-4)


def test_choked_flow_scales_with_throat_area():
    assert ice.choked_mass_flow_kg_s(19.0) / ice.choked_mass_flow_kg_s(20.0) == pytest.approx((19 / 20) ** 2)


def test_power_ceiling_is_air_over_afr_times_lhv_times_bte():
    f = ice.FUELS["gasoline"]
    m_air = ice.choked_mass_flow_kg_s(20.0, cd=0.95)
    assert ice.restrictor_power_ceiling_kw("gasoline", bte=0.3) == pytest.approx(
        m_air / f.afr_stoich * f.lhv_mj_kg * 1e3 * 0.3, rel=1e-12)


def test_rule_diameters_and_co2_factors():
    assert ice.FUELS["gasoline"].restrictor_mm == 20.0 and ice.FUELS["e85"].restrictor_mm == 19.0
    assert ice.FUELS["gasoline"].co2_kg_per_l == 2.31 and ice.FUELS["e85"].co2_kg_per_l == 1.65
    assert ice.co2_ev_kg(10.0) == pytest.approx(6.5)


def test_restricted_curve_never_exceeds_the_ceiling():
    big = replace(ENG, torque_nm=[t * 2 for t in ENG.torque_nm])
    assert big.restrictor_check()["exceeds"]
    capped = big.restricted()
    assert capped.peak()["power_kw"] <= ice.restrictor_power_ceiling_kw() * 1.0001


# ------------------------------------------------------------------ gearbox
def test_shift_speed_matches_the_closed_form_on_a_linear_torque_curve():
    # T = a - b rpm. In gear k the force is (a - b k c v) k e / r with c = 60 / (2 pi r), so gears g and g+1 give
    # equal force at v = a / (b c (k_g + k_g+1)).
    a, b = 100.0, 0.004
    eng = ice.EngineMap(rpm=[1000.0, 25000.0], torque_nm=[a - b * 1000, a - b * 25000], redline_rpm=24000.0)
    gb = ice.Gearbox(primary=2.0, ratios=(2.5, 1.8), final_drive=3.0)
    c = 60.0 / (2 * math.pi * gb.tire_radius_m)
    k1, k2 = gb.overall(0), gb.overall(1)
    assert gb.shift_speeds(eng)[0] == pytest.approx(a / (b * c * (k1 + k2)), rel=1e-6)


def test_flat_torque_runs_each_gear_to_redline():
    eng = ice.EngineMap(rpm=[1000.0, 14000.0], torque_nm=[50.0, 50.0], redline_rpm=13000.0)
    sv = GB.shift_speeds(eng)
    for g, v in enumerate(sv):
        assert v == pytest.approx(GB.speed_at(13000.0, g), rel=1e-3)


def test_gear_ratios_must_fall():
    with pytest.raises(ValueError):
        ice.Gearbox(ratios=(1.5, 2.0))


# ------------------------------------------------------------------ launch
def test_launch_matches_constant_acceleration_with_no_losses():
    eng = ice.EngineMap(rpm=[500.0, 20000.0], torque_nm=[50.0, 50.0], redline_rpm=20000.0, inertia_kgm2=0.0)
    gb = ice.Gearbox(primary=1.0, ratios=(4.0,), final_drive=1.0, efficiency=1.0)
    veh = ice.ICEVehicle(mass_kg=250.0, mu=100.0, cda_m2=0.0, cla_m2=0.0, crr=0.0, wheel_inertia_kgm2=0.0)
    res = ice.simulate_launch(eng, gb, veh, launch_rpm=500.0, dt=1e-4)
    a = 50.0 * 4.0 / gb.tire_radius_m / 250.0
    assert res.time_s == pytest.approx(math.sqrt(2 * 75.0 / a), rel=2e-3)


def test_rotating_inertia_reflects_through_the_square_of_the_ratio():
    veh = ice.ICEVehicle(mass_kg=300.0, wheel_inertia_kgm2=0.6)
    r, k = 0.225, 17.4
    assert veh.m_eff(k, ENG, 0.92, r) == pytest.approx(300.0 + (0.6 + ENG.inertia_kgm2 * k * k * 0.92) / r ** 2)
    assert veh.m_eff(None, ENG, 0.92, r) == pytest.approx(300.0 + 0.6 / r ** 2)


def test_launch_shifts_where_the_gearbox_says_and_finishes():
    res = ice.simulate_launch(ENG, GB)
    assert res.finished and 3.5 < res.time_s < 6.0
    sv = GB.shift_speeds(ENG)
    for (_, v, g_from, _) in res.shifts:
        assert v == pytest.approx(sv[g_from - 1], abs=0.05)
    assert res.slipping[0] and not res.slipping[-1]


def test_an_instant_shift_keeps_driving_through_every_gear():
    gb = replace(GB, shift_time_s=0.0)
    res = ice.simulate_launch(ENG, gb, distance_m=200.0)
    assert res.finished and len(res.shifts) >= 3
    assert res.gear.max() == len(res.shifts) + 1 and (res.gear > 0).all()


def test_traction_limits_the_launch():
    res = ice.simulate_launch(ENG, GB)
    assert (res.wheel_force_n[:200] <= res.traction_limit_n[:200] + 1e-9).all()


def test_final_drive_sweep_has_an_interior_optimum():
    sw = ice.sweep_final_drive(ENG, GB)
    assert sw.ratios[0] < sw.best_ratio < sw.ratios[-1]


# ------------------------------------------------------------------ fuel
def test_fuel_at_constant_speed_matches_the_force_balance():
    v0, L = 20.0, 1000.0
    lap = SimpleNamespace(s=[0.0, L / 2, L], v=[v0, v0, v0])
    veh = ice.ICEVehicle()
    fr = ice.fuel_for_lap(lap, ENG, GB, veh, bte=0.3)
    f = ice.FUELS["gasoline"]
    p = (0.5 * veh.rho * veh.cda_m2 * v0 ** 2 + veh.crr * veh.mass_kg * ice.G) * v0 / GB.efficiency
    assert fr.fuel_kg == pytest.approx(p / (0.3 * f.lhv_mj_kg * 1e6) * L / v0, rel=1e-9)
    assert fr.co2_kg == pytest.approx(fr.fuel_l * 2.31)
    assert fr.l_per_100km == pytest.approx(fr.fuel_l / L * 1e5)


def test_coasting_burns_only_the_overrun_flow():
    lap = SimpleNamespace(s=[0.0, 100.0], v=[30.0, 10.0])          # hard deceleration: no drive
    fr = ice.fuel_for_lap(lap, ENG, GB, overrun_fuel_g_s=0.05)
    assert fr.fuel_kg == pytest.approx(0.05e-3 * 100.0 / 20.0)


def test_endurance_fuel_and_tank_margin():
    fr = SimpleNamespace(fuel_l=0.08, lap_m=1000.0)
    end = ice.endurance_fuel(fr, tank_l=2.0)
    assert end.laps == pytest.approx(22.0)
    assert end.fuel_l == pytest.approx(1.76)
    assert end.tank_margin_l == pytest.approx(2.0 - 1.76 * 1.1)
    assert end.l_per_100km == pytest.approx(8.0)


# ------------------------------------------------------------------ engine braking
def test_engine_braking_force_and_balance():
    f = ice.engine_braking_force_n(20.0, ENG, GB, gear=2)
    assert f == pytest.approx(ENG.overrun_torque_nm * GB.overall(2) / GB.tire_radius_m)
    b = ice.brake_balance_with_engine_braking(1.5, 0.6, 20.0, ENG, GB, gear=2)
    assert b["front_n"] == pytest.approx(0.6 * (b["total_n"] - f))
    assert b["effective_front_share"] < 0.6


# ------------------------------------------------------------------ gate
def _fuel_res():
    lap = SimpleNamespace(s=[0.0, 500.0, 1000.0], v=[10.0, 25.0, 12.0])
    return ice.fuel_for_lap(lap, ENG, GB)


def test_undeclared_inputs_are_missing_never_pass():
    st = {f.check: f.status for f in ice.feasibility_gate(ENG, GB, _fuel_res())}
    for k in ("restrictor diameter", "fuel tank", "fuel pump", "injector duty", "cooling"):
        assert st[k] == "MISSING"


def test_oversize_restrictor_and_small_tank_fail():
    st = {f.check: f.status for f in ice.feasibility_gate(ENG, GB, _fuel_res(), restrictor_mm=20.5, tank_l=0.1)}
    assert st["restrictor diameter"] == "FAIL" and st["fuel tank"] == "FAIL"
    st = {f.check: f.status for f in ice.feasibility_gate(ENG, GB, _fuel_res(), fuel_key="e85", restrictor_mm=19.5)}
    assert st["restrictor diameter"] == "FAIL"


def test_injector_duty_and_pump():
    fr = _fuel_res()
    st = {f.check: f.status for f in ice.feasibility_gate(ENG, GB, fr, injector_cc_min=1.0, fuel_pump_lph=1.0)}
    assert st["injector duty"] == "FAIL" and st["fuel pump"] == "FAIL"
    st = {f.check: f.status for f in ice.feasibility_gate(ENG, GB, fr, injector_cc_min=500.0, fuel_pump_lph=200.0)}
    assert st["injector duty"] == "PASS" and st["fuel pump"] == "PASS"


def test_representative_curve_is_flagged():
    assert any(f.check == "torque curve" and f.status == "WARN" for f in ice.feasibility_gate(ENG, GB, _fuel_res()))


# ------------------------------------------------------------------ import, lap, comparison, cooling
def test_csv_import_reads_torque_or_power():
    a = ice.EngineMap.from_csv("rpm,torque_nm\n4000,40\n8000,55\n12000,45\n")
    assert a.torque_at(8000) == pytest.approx(55.0)
    b = ice.EngineMap.from_csv("rpm,power_kw\n4000,10\n8000,40\n")
    assert b.torque_at(8000) == pytest.approx(40e3 / (8000 * 2 * math.pi / 60))
    c = ice.EngineMap.from_csv("RPM,Torque (lbft)\n5000,30\n9000,40\n")
    assert c.torque_at(9000) == pytest.approx(40 * 1.3558179)


def test_lap_runs_shifts_and_more_torque_is_faster():
    track = lt.default_autocross()
    base = ice.simulate_lap(_vd(), track, ENG, GB)
    strong = ice.simulate_lap(_vd(), track, replace(ENG, torque_nm=[t * 1.3 for t in ENG.torque_nm]), GB)
    assert base.lap.ok and base.shifts > 0 and base.shift_penalty_s >= 0
    assert strong.lap_time_s < base.lap_time_s


def test_lap_force_carries_the_rotating_inertia():
    veh = ice.ICEVehicle()
    pt = ice.lap_powertrain(ENG, GB, veh)
    v = 8.0
    g = GB.best_gear(v, ENG)
    m_eff = veh.m_eff(GB.overall(g), ENG, GB.efficiency, GB.tire_radius_m)
    assert pt.tractive_force(v) == pytest.approx(GB.envelope_force(v, ENG) * veh.mass_kg / m_eff)


def test_heavier_engine_option_is_slower():
    opts = [ice.EngineOption("base", ENG, GB), ice.EngineOption("heavy", ENG, GB, mass_delta_kg=30.0)]
    rows = ice.compare_engines(opts, _vd(), lt.default_autocross())
    assert all(r["error"] == "" for r in rows)
    assert rows[1]["t_75m_s"] > rows[0]["t_75m_s"] and rows[1]["lap_s"] > rows[0]["lap_s"]


def test_cooling_uses_the_engine_heat():
    fr = _fuel_res()
    res = ice.size_cooling(fr)
    assert res.spec.heat_w == pytest.approx(fr.mean_coolant_heat_w)
