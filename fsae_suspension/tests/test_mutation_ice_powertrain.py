"""suspension/ice_powertrain.py against oracles written here: the choked-orifice flow and the restrictor power
ceiling, the torque curve with its limiter and power cap, CSV import in every unit, the gearbox ratios and the
upshift crossover found independently, the standing-start integration written out step by step, fuel and coolant
heat from the wheel power, the endurance tank, engine braking, and every feasibility finding word for word."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.optimize import brentq

from suspension import ice_powertrain as ice
from suspension import interfaces as ifm

GAM, RAIR = 1.4, 287.0


def _choked(d_mm, p0=101325.0, t0=298.15, cd=1.0):
    A = math.pi * (d_mm / 1000) ** 2 / 4
    return cd * A * p0 / math.sqrt(RAIR * t0) * math.sqrt(GAM) * (2 / (GAM + 1)) ** ((GAM + 1) / (2 * (GAM - 1)))


def test_constants_and_fuels():
    assert (ice.G, ice.P_ISA, ice.T_ISA, ice.CO2_KG_PER_KWH_EV, ice.FUEL_LIMIT_L_PER_100KM, ice.ENDURANCE_KM) == (
        9.81, 101325.0, 298.15, 0.65, 26.0, 22.0)
    g, e = ice.fuel("gasoline"), ice.fuel("e85")
    assert (g.key, g.name, g.density_kg_l, g.lhv_mj_kg, g.afr_stoich, g.co2_kg_per_l, g.restrictor_mm) == (
        "gasoline", "Gasoline (100 octane)", 0.745, 43.0, 14.7, 2.31, 20.0)
    assert g.source == "density, LHV, AFR: typical published values; CO2 factor and restrictor: FSAE rules"
    assert (e.key, e.name, e.density_kg_l, e.lhv_mj_kg, e.afr_stoich, e.co2_kg_per_l, e.restrictor_mm) == (
        "e85", "E85", 0.781, 29.2, 9.8, 1.65, 19.0)
    assert e.source == "density, LHV, AFR: typical published values for E85; CO2 factor and restrictor: FSAE rules"
    assert ice.Fuel("k", "n", 1, 2, 3, 4, 5).source == ""
    with pytest.raises(ValueError) as err:
        ice.fuel("diesel")
    assert str(err.value) == "unknown fuel 'diesel'; one of ['e85', 'gasoline']"
    assert ice.co2_ev_kg(4.0) == pytest.approx(2.6) and ice.co2_ev_kg(-3.0) == 0.0


def test_the_choked_restrictor():
    assert ice.choked_mass_flow_kg_s(20.0) == pytest.approx(_choked(20.0), rel=1e-12)
    assert ice.choked_mass_flow_kg_s(20.0) == pytest.approx(0.0745, abs=5e-5)
    assert ice.choked_mass_flow_kg_s(19.0, 90000.0, 310.0, 0.9) == pytest.approx(_choked(19.0, 90000.0, 310.0, 0.9),
                                                                                  rel=1e-12)
    assert ice.choked_mass_flow_kg_s(20.0, cd=1.0) == pytest.approx(_choked(20.0))
    for bad in (dict(d_mm=0.0), dict(d_mm=-1.0), dict(d_mm=20.0, p0_pa=0.0), dict(d_mm=20.0, t0_k=0.0),
                dict(d_mm=20.0, cd=0.0), dict(d_mm=20.0, cd=1.0000001)):
        with pytest.raises(ValueError) as err:
            ice.choked_mass_flow_kg_s(**bad)
        assert str(err.value) == "restrictor diameter, stagnation state and Cd must be positive (Cd <= 1)"
    p = ice.restrictor_power_ceiling_kw()
    assert p == pytest.approx(_choked(20.0, cd=0.95) / 14.7 * 43.0 * 1e3 * 0.30, rel=1e-12)
    p = ice.restrictor_power_ceiling_kw("e85", d_mm=18.0, bte=0.33, cd=0.9, p0_pa=95000.0, t0_k=300.0)
    assert p == pytest.approx(_choked(18.0, 95000.0, 300.0, 0.9) / 9.8 * 29.2 * 1e3 * 0.33, rel=1e-12)
    assert ice.restrictor_power_ceiling_kw("e85") == pytest.approx(_choked(19.0, cd=0.95) / 9.8 * 29.2 * 300,
                                                                   rel=1e-12)
    for bte in (0.0, 1.0, -0.1, 1.5):
        with pytest.raises(ValueError) as err:
            ice.restrictor_power_ceiling_kw(bte=bte)
        assert str(err.value) == "brake thermal efficiency must be between 0 and 1"


# ------------------------------------------------------------------ the torque curve
def ENG(**kw):
    base = dict(rpm=[2000.0, 6000.0, 10000.0], torque_nm=[40.0, 60.0, 50.0], redline_rpm=11000.0)
    base.update(kw)
    return ice.EngineMap(**base)


def test_the_engine_map():
    e = ENG()
    assert (e.idle_rpm, e.inertia_kgm2, e.overrun_torque_nm, e.displacement_cc, e.name, e.source, e.power_cap_kw) == (
        1300.0, 0.025, 8.0, 599.0, "", "user", None)
    assert e.torque_at(1000.0) == 40.0 and e.torque_at(4000.0) == 50.0 and e.torque_at(10500.0) == 50.0
    assert e.torque_at(11000.0) == 50.0 and e.torque_at(11000.01) == 0.0
    assert e.power_kw_at(6000.0) == pytest.approx(60.0 * 6000 * 2 * math.pi / 60 / 1e3, rel=1e-12)
    capped = ENG(power_cap_kw=30.0)
    w = 6000 * 2 * math.pi / 60
    assert capped.torque_at(6000.0) == pytest.approx(30e3 / w, rel=1e-12) and capped.torque_at(2000.0) == 40.0
    assert capped.torque_at(0.0) == 40.0
    for bad, msg in ((dict(rpm=[1.0], torque_nm=[1.0]), "the torque curve needs at least two (rpm, torque) points "
                                                          "of equal length"),
                     (dict(rpm=[1.0, 2.0], torque_nm=[1.0]), "the torque curve needs at least two (rpm, torque) "
                                                              "points of equal length"),
                     (dict(rpm=[1.0, 1.0, 2.0], torque_nm=[1.0] * 3), "rpm must increase strictly along the curve"),
                     (dict(rpm=[3.0, 2.0], torque_nm=[1.0] * 2), "rpm must increase strictly along the curve"),
                     (dict(redline_rpm=2000.0), "redline must lie above the first rpm point")):
        with pytest.raises(ValueError) as err:
            ENG(**bad)
        assert str(err.value) == msg
    assert ENG(redline_rpm=2000.5).redline_rpm == 2000.5


def test_the_peak_and_the_restrictor():
    e = ENG()
    grid = np.linspace(2000.0, 10000.0, 400)
    p = np.array([e.torque_at(x) * x * 2 * math.pi / 60 / 1e3 for x in grid])
    pk = e.peak()
    assert pk == {"power_kw": pytest.approx(p.max()), "power_rpm": pytest.approx(grid[p.argmax()]),
                  "torque_nm": pytest.approx(max(e.torque_at(x) for x in grid)), "torque_rpm": pytest.approx(grid[np.argmax(
                      [e.torque_at(x) for x in grid])])}
    short = ENG(redline_rpm=8000.0)
    assert short.peak()["power_rpm"] <= 8000.0 and short.peak()["power_rpm"] == pytest.approx(8000.0)
    r = e.restricted("e85", bte=0.25, cd=0.9, d_mm=15.0)
    assert r.power_cap_kw == pytest.approx(ice.restrictor_power_ceiling_kw("e85", 15.0, bte=0.25, cd=0.9))
    assert r.source == "user (restrictor-capped)" and e.power_cap_kw is None and r.rpm == e.rpm
    assert ENG().restricted().power_cap_kw == pytest.approx(ice.restrictor_power_ceiling_kw())
    c = e.restrictor_check("gasoline", 0.2, 0.9, d_mm=10.0)
    ceil = ice.restrictor_power_ceiling_kw("gasoline", 10.0, bte=0.2, cd=0.9)
    assert c == {"ceiling_kw": ceil, "peak_kw": pk["power_kw"], "peak_rpm": pk["power_rpm"], "exceeds": True,
                 "bte": 0.2}
    assert e.restrictor_check()["exceeds"] is False
    assert e.restrictor_check()["ceiling_kw"] == pytest.approx(ice.restrictor_power_ceiling_kw())


def test_the_representative_curve():
    r = ice.EngineMap.representative_600_i4()
    assert r.rpm == [3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 13500]
    assert r.torque_nm == [38, 42, 46, 50, 54, 57, 58, 57, 53, 47, 41, 38]
    assert (r.redline_rpm, r.idle_rpm, r.displacement_cc, r.name, r.source) == (
        13500.0, 1300.0, 599.0, "600 cc inline-four, restricted (representative)", "representative")


def test_reading_a_dyno_csv():
    e = ice.EngineMap.from_csv("rpm,torque_nm\n6000,60\n\n2000,40\n10000,50\n")
    assert e.rpm == [2000.0, 6000.0, 10000.0] and e.torque_nm == [40.0, 60.0, 50.0]
    assert e.redline_rpm == 10000.0 and e.source == "dyno"
    lb = ice.EngineMap.from_csv("RPM;LbFt;extra\n2000;30;9\n4000;40;9", redline_rpm=4500.0, name="x", source="sim")
    assert lb.torque_nm == pytest.approx([30 * 1.3558179, 40 * 1.3558179]) and lb.redline_rpm == 4500.0
    assert lb.name == "x" and lb.source == "sim"
    hp = ice.EngineMap.from_csv("rpm\thp\n3000\t20\n6000\t50")
    assert hp.torque_nm == pytest.approx([20 * 745.69987 / (3000 * 2 * math.pi / 60),
                                          50 * 745.69987 / (6000 * 2 * math.pi / 60)])
    kw = ice.EngineMap.from_csv("rpm,power_kw\n3000,20\n6000,50")
    assert kw.torque_nm == pytest.approx([20e3 / (3000 * 2 * math.pi / 60), 50e3 / (6000 * 2 * math.pi / 60)])
    k2 = ice.EngineMap.from_csv("engine rpm,kW\n3000,20\n6000,50")
    assert k2.torque_nm == pytest.approx(kw.torque_nm)
    zero = ice.EngineMap.from_csv("rpm,power\n0,5\n6000,50")
    assert zero.torque_nm[0] == pytest.approx(5e3 / 1e-6)
    both = ice.EngineMap.from_csv("rpm,power_kw,torque\n3000,99,40\n6000,99,60")
    assert both.torque_nm == [40.0, 60.0]
    first = ice.EngineMap.from_csv("rpm,torque,hp\n3000,40,99\n6000,60,99")
    assert first.torque_nm == [40.0, 60.0]
    for h in ("nm", "tq"):
        assert ice.EngineMap.from_csv(f"rpm,{h}\n3000,40\n6000,60").torque_nm == [40.0, 60.0]
    hp_after_kw = ice.EngineMap.from_csv("rpm,kw,hp\n3000,20,99\n6000,50,99")
    assert hp_after_kw.torque_nm == pytest.approx(kw.torque_nm)
    with pytest.raises(ValueError) as err:
        ice.EngineMap.from_csv("rpm,boost\n3000,1\n6000,2")
    assert str(err.value) == ("no torque or power column found (headers: rpm plus torque_nm, lbft, power_kw or hp)")


# ------------------------------------------------------------------ the gearbox
GB = dict(primary=2.0, ratios=(2.5, 1.8, 1.3), final_drive=3.0, shift_time_s=0.05, efficiency=0.9, tire_radius_m=0.25)


def test_the_gearbox():
    d = ice.Gearbox()
    assert (d.primary, d.ratios, d.final_drive, d.shift_time_s, d.efficiency, d.tire_radius_m) == (
        2.111, (2.750, 2.000, 1.667, 1.444, 1.304, 1.208), 3.0, 0.08, 0.92, 0.225)
    assert d.source == "representative: CBR600RR (2007+) primary and gear ratios; check your engine's service manual"
    g = ice.Gearbox(**GB)
    assert g.n == 3 and g.overall(1) == pytest.approx(2.0 * 1.8 * 3.0)
    assert g.rpm_at(10.0, 0) == pytest.approx(10.0 / 0.25 * 15.0 * 60 / (2 * math.pi))
    assert g.rpm_at(-5.0, 0) == 0.0
    assert g.speed_at(6000.0, 2) == pytest.approx(6000 * 2 * math.pi / 60 / 7.8 * 0.25)
    e = ENG()
    assert g.wheel_force(1.0, 0, e) == pytest.approx(40.0 * 15.0 * 0.9 / 0.25)            # clutch held at 2000 rpm
    v = g.speed_at(6000.0, 0)
    assert g.wheel_force(v, 0, e) == pytest.approx(60.0 * 15.0 * 0.9 / 0.25)
    assert g.wheel_force(g.speed_at(12000.0, 0), 0, e) == 0.0
    f = [g.wheel_force(20.0, k, e) for k in range(3)]
    assert g.best_gear(20.0, e) == int(np.argmax(f)) and g.envelope_force(20.0, e) == max(f)
    assert g.top_speed_at_redline(e) == pytest.approx(g.speed_at(11000.0, 2))
    for bad, msg in ((dict(ratios=()), "every ratio must be positive"),
                     (dict(ratios=(2.0, 0.0)), "every ratio must be positive"),
                     (dict(primary=0.0), "every ratio must be positive"),
                     (dict(final_drive=-1.0), "every ratio must be positive"),
                     (dict(ratios=(2.0, 2.0)), "gear ratios must fall from first gear to top gear"),
                     (dict(ratios=(1.5, 2.0)), "gear ratios must fall from first gear to top gear")):
        with pytest.raises(ValueError) as err:
            ice.Gearbox(**{**GB, **bad})
        assert str(err.value) == msg


def _shift_oracle(g, e):
    out = []
    for k in range(g.n - 1):
        vr = g.speed_at(e.redline_rpm, k)
        lo = max(g.speed_at(e.rpm[0], k), 0.1)
        d = lambda v: g.wheel_force(v, k, e) - g.wheel_force(v, k + 1, e)
        if d(vr * 0.999) > 0:
            out.append(vr)
        elif d(lo) <= 0:
            out.append(lo)
        else:
            out.append(brentq(d, lo, vr * 0.999, xtol=1e-12))
    return out


def test_shift_speeds():
    g = ice.Gearbox(**GB)
    e = ENG(rpm=[2000.0, 6000.0, 10000.0], torque_nm=[40.0, 60.0, 20.0])
    s = g.shift_speeds(e)
    assert s == pytest.approx(_shift_oracle(g, e), abs=1e-8)
    assert all(g.speed_at(2000.0, k) < s[k] < g.speed_at(11000.0, k) for k in range(2))
    flat = ice.Gearbox(**{**GB, "ratios": (2.0, 1.99)})
    assert flat.shift_speeds(ENG()) == pytest.approx([flat.speed_at(11000.0, 0)])
    dead = ENG(torque_nm=[0.0, 0.0, 0.0])
    assert g.shift_speeds(dead) == pytest.approx([max(g.speed_at(2000.0, k), 0.1) for k in range(2)])
    low = ENG(rpm=[1.0, 6000.0, 10000.0], torque_nm=[0.0, 0.0, 0.0])
    assert g.shift_speeds(low) == pytest.approx([0.1, 0.1])


# ------------------------------------------------------------------ the vehicle
def test_the_vehicle():
    v = ice.ICEVehicle()
    assert (v.mass_kg, v.driven_weight_frac, v.cg_height_m, v.wheelbase_m, v.mu, v.cda_m2, v.cla_m2, v.rear_aero_frac,
            v.crr, v.rho, v.wheel_inertia_kgm2) == (300.0, 0.55, 0.28, 1.55, 1.4, 1.10, 2.5, 0.5, 0.015, 1.225, 0.6)
    e = ENG()
    assert v.m_eff(None, e) == pytest.approx(300 + 0.6 / 0.225 ** 2)
    assert v.m_eff(10.0, None) == pytest.approx(300 + 0.6 / 0.225 ** 2)
    assert v.m_eff(10.0, e, 0.9, 0.25) == pytest.approx(300 + (0.6 + 0.025 * 100 * 0.9) / 0.0625)
    assert v.m_eff(10.0, e) == pytest.approx(300 + (0.6 + 2.5) / 0.225 ** 2)
    assert v.resistance_n(20.0) == pytest.approx(0.5 * 1.225 * 1.1 * 400 + 0.015 * 300 * 9.81)


# ------------------------------------------------------------------ the standing start
def _launch(eng, gb, veh, fuel_key="gasoline", bte=0.30, launch_rpm=9000.0, cap=math.inf, distance=75.0, dt=1e-3,
            t_max=20.0):
    f = ice.fuel(fuel_key)
    r = gb.tire_radius_m
    sv = gb.shift_speeds(eng)
    v = x = t = ap = 0.0
    gear, slip, left, nxt = 0, True, 0.0, 0
    fk = cj = 0.0
    shifts, rows = [], []
    n_cap = int(t_max / dt) + 2
    i = 0
    while x < distance and i < n_cap - 1:
        ov = gb.overall(gear)
        rin = gb.rpm_at(v, gear)
        if slip and rin >= launch_rpm:
            slip = False
        if left > 0:
            fd, re, cpl = 0.0, gb.rpm_at(v, nxt), False
            left -= dt
            if left <= 0:
                gear = nxt
        elif not slip and gear < gb.n - 1 and (v >= sv[gear] or rin >= eng.redline_rpm):
            shifts.append((t, v, gear + 1, gear + 2))
            if gb.shift_time_s > 0:
                nxt, left = gear + 1, gb.shift_time_s
                fd, re, cpl = 0.0, rin, False
            else:
                gear += 1
                ov = gb.overall(gear)
                re = gb.rpm_at(v, gear)
                fd, cpl = eng.torque_at(re) * ov * gb.efficiency / r, True
        elif slip:
            re = launch_rpm
            fd, cpl = min(eng.torque_at(launch_rpm), cap) * ov * gb.efficiency / r, False
        else:
            re = rin
            fd, cpl = eng.torque_at(re) * ov * gb.efficiency / r, True
        nr = veh.mass_kg * 9.81 * veh.driven_weight_frac + veh.mass_kg * ap * veh.cg_height_m / veh.wheelbase_m
        ft = veh.mu * max(nr + 0.5 * veh.rho * veh.cla_m2 * v * v * veh.rear_aero_frac, 0.0)
        fw = min(fd, ft)
        me = veh.mass_kg + (veh.wheel_inertia_kgm2 + (eng.inertia_kgm2 * ov ** 2 * gb.efficiency if cpl else 0.0)) / r ** 2
        a = (fw - 0.5 * veh.rho * veh.cda_m2 * v * v - veh.crr * veh.mass_kg * 9.81) / me
        if fd > 0:
            tu = fw * r / (ov * gb.efficiency)
            fk += tu * re * 2 * math.pi / 60 / (bte * f.lhv_mj_kg * 1e6) * dt
            if slip:
                cj += tu * (launch_rpm - rin) * 2 * math.pi / 60 * dt
        rows.append((t, x, v, a, re, 0 if left > 0 else gear + 1, fw, ft, slip))
        v = max(v + a * dt, 0.0)
        x += v * dt
        t += dt
        ap = a
        i += 1
    return np.array(rows, dtype=object), shifts, fk * 1e3, cj / 1e3, x >= distance


def _same_launch(res, want):
    rows, shifts, fuel_g, clutch_kj, fin = want
    cols = [res.t_s, res.x_m, res.v_ms, res.a_ms2, res.rpm, res.gear, res.wheel_force_n, res.traction_limit_n,
            res.slipping]
    assert len(res.t_s) == len(rows)
    for k, c in enumerate(cols):
        assert np.asarray(c, float) == pytest.approx(np.asarray(rows[:, k], float), rel=1e-9, abs=1e-9), k
    assert res.gear.dtype.kind == "i" and res.slipping.dtype == bool
    assert [(pytest.approx(a), pytest.approx(b), c, d) for a, b, c, d in shifts] == res.shifts
    assert res.fuel_g == pytest.approx(fuel_g, rel=1e-9) and res.clutch_heat_kj == pytest.approx(clutch_kj, rel=1e-9)
    assert res.finished is fin


def test_the_launch_integration():
    e = ENG(rpm=[2000.0, 6000.0, 10000.0], torque_nm=[40.0, 60.0, 30.0])
    g = ice.Gearbox(**GB)
    veh = ice.ICEVehicle(mass_kg=250.0, mu=1.6, cla_m2=3.0)
    res = ice.simulate_launch(e, g, veh, launch_rpm=7000.0, dt=2e-3, bte=0.28, fuel_key="e85",
                              clutch_capacity_nm=55.0)
    _same_launch(res, _launch(e, g, veh, "e85", 0.28, 7000.0, 55.0, 75.0, 2e-3))
    assert res.distance_m == 75.0 and len(res.shifts) >= 1 and res.warnings == []
    assert res.time_s == res.t_s[-1] and res.trap_speed_kmh == pytest.approx(res.v_ms[-1] * 3.6)
    assert res.clutch_heat_kj > 0 and res.slipping[0] and not res.slipping[-1]
    inst = ice.Gearbox(**{**GB, "shift_time_s": 0.0})
    r2 = ice.simulate_launch(e, inst, veh, launch_rpm=7000.0, dt=2e-3)
    _same_launch(r2, _launch(e, inst, veh, launch_rpm=7000.0, dt=2e-3))
    assert len(r2.shifts) >= 1 and 0 not in set(r2.gear.tolist())


def test_a_limiter_shift_and_a_short_run():
    e = ENG(rpm=[2000.0, 6000.0, 10000.0], torque_nm=[40.0, 60.0, 60.0], redline_rpm=10000.0)
    g = ice.Gearbox(**{**GB, "ratios": (2.5, 2.45, 2.4)})
    veh = ice.ICEVehicle()
    r = ice.simulate_launch(e, g, veh, distance_m=60.0, dt=2e-3)
    _same_launch(r, _launch(e, g, veh, distance=60.0, dt=2e-3))
    short = ice.simulate_launch(e, g, veh, distance_m=500.0, dt=5e-3, t_max_s=1.0)
    _same_launch(short, _launch(e, g, veh, distance=500.0, dt=5e-3, t_max=1.0))
    assert short.finished is False and short.time_s == math.inf
    assert short.warnings == ["did not reach 500 m in 1 s; check the curve and the ratios"]
    rep = ice.simulate_launch(ice.EngineMap.representative_600_i4(), ice.Gearbox(), dt=5e-3, distance_m=20.0)
    assert rep.warnings == ["the torque curve is a representative shape, not your engine's dyno curve"]
    assert len(ice.LaunchResult(*(np.zeros(1),) * 9).shifts) == 0
    lr = ice.LaunchResult(*(np.zeros(1),) * 9)
    assert (lr.fuel_g, lr.clutch_heat_kj, lr.distance_m, lr.finished, lr.warnings) == (0.0, 0.0, 75.0, True, [])
    none = ice.simulate_launch(e, g, veh, distance_m=0.0)
    assert len(none.t_s) == 1 and none.t_s[0] == 0.0 and none.finished is True


def test_the_final_drive_sweep():
    e = ENG(rpm=[2000.0, 6000.0, 10000.0], torque_nm=[40.0, 60.0, 30.0])
    g = ice.Gearbox(**GB)
    s = ice.sweep_final_drive(e, g, None, ratios=[2.0, 3.0, 4.0], dt=4e-3, distance_m=40.0)
    for k, fd in enumerate((2.0, 3.0, 4.0)):
        g2 = ice.Gearbox(**{**GB, "final_drive": fd})
        r = ice.simulate_launch(e, g2, None, dt=4e-3, distance_m=40.0)
        assert s.times_s[k] == pytest.approx(r.time_s) and s.trap_kmh[k] == pytest.approx(r.trap_speed_kmh)
        assert s.top_speed_kmh[k] == pytest.approx(g2.top_speed_at_redline(e) * 3.6)
    k = int(np.argmin(s.times_s))
    assert s.best_ratio == [2.0, 3.0, 4.0][k] and s.best.time_s == s.times_s[k]
    assert s.ratios.tolist() == [2.0, 3.0, 4.0]
    assert s.as_table() == [{"final_drive": r, "t_75m_s": t, "trap_kmh": v, "top_speed_kmh": p} for r, t, v, p in
                            zip([2.0, 3.0, 4.0], s.times_s.tolist(), s.trap_kmh.tolist(), s.top_speed_kmh.tolist())]
    d = ice.sweep_final_drive(e, g, dt=0.02, distance_m=10.0)
    assert d.ratios == pytest.approx(np.linspace(1.8, 4.2, 25))
    never = ice.sweep_final_drive(e, g, ratios=[2.5], dt=0.01, t_max_s=0.05, distance_m=500.0)
    assert never.best is None and never.best_ratio == 2.5


# ------------------------------------------------------------------ the lap
def test_the_lap_powertrain():
    e = ENG()
    g = ice.Gearbox(**GB)
    veh = ice.ICEVehicle(mass_kg=280.0, cda_m2=1.2, cla_m2=3.1, rho=1.2, crr=0.02)
    pt = ice.lap_powertrain(e, g, veh)
    assert (pt.power_kw, pt.drivetrain_eff, pt.cda, pt.cla, pt.rho, pt.crr, pt.drive) == (
        pytest.approx(e.peak()["power_kw"]), 0.9, 1.2, 3.1, 1.2, 0.02, "rwd")
    assert pt.engine is e and pt.gearbox is g
    for v in (0.0, 8.0, 25.0):
        vv = max(v, 0.05)
        k = g.best_gear(vv, e)
        me = veh.m_eff(g.overall(k), e, 0.9, 0.25)
        assert pt.tractive_force(v) == pytest.approx(g.envelope_force(vv, e) * 280.0 / me, rel=1e-12)
    over = ice.lap_powertrain(e, g, veh, power_kw=10.0, drive="awd")
    assert over.power_kw == 10.0 and over.drive == "awd"
    assert ice.lap_powertrain(e, g).cda == 1.10


class FakeLap:
    def __init__(self, s, v, ok=True, t=50.0, warning=""):
        self.s, self.v, self.ok, self.lap_time_s, self.warning = s, v, ok, t, warning


def _penalty(g, e, s, v):
    gears = [g.best_gear(x, e) for x in v]
    acc = np.append(np.diff(np.asarray(v) ** 2) / (2 * np.maximum(np.diff(s), 1e-6)), 0.0)
    n, pen = 0, 0.0
    for k in range(1, len(v)):
        if gears[k] > gears[k - 1] and acc[k] > 0:
            n += 1
            j = k
            while j < len(v) - 1 and acc[j] > 0:
                j += 1
            pen += min(g.shift_time_s, (s[j] - s[k]) * acc[k] * g.shift_time_s / max(v[k] ** 2, 1.0))
    return n, pen, [x + 1 for x in gears]


def test_the_lap_and_its_shift_penalty(monkeypatch):
    from suspension import laptime as lt
    e = ENG()
    g = ice.Gearbox(**GB)
    s = np.linspace(0, 400, 81)
    v = np.r_[np.linspace(5, 30, 40), np.linspace(30, 12, 20), np.linspace(12, 33, 21)]
    seen = {}

    def fake(vdyn, track, pt):
        seen.update(vdyn=vdyn, track=track, pt=pt)
        return FakeLap(s, v, t=61.0)
    monkeypatch.setattr(lt, "simulate_lap", fake)
    r = ice.simulate_lap("VD", "TR", e, g)
    n, pen, gears = _penalty(g, e, s, v)
    assert (seen["vdyn"], seen["track"]) == ("VD", "TR") and seen["pt"].engine is e
    assert r.shifts == n and n >= 2 and r.shift_penalty_s == pytest.approx(pen, rel=1e-12)
    assert r.lap_time_s == pytest.approx(61.0 + pen) and r.gear_trace == gears and r.lap.lap_time_s == 61.0
    assert r.warnings == ["the car reaches the limiter in top gear on this track; a longer final drive may help"] if \
        33.0 > g.top_speed_at_redline(e) * 0.999 else r.warnings == []
    slow = np.linspace(5, 12, 81)
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: FakeLap(s, slow))
    assert ice.simulate_lap(None, None, e, g).warnings == []
    fast = np.full(81, g.top_speed_at_redline(e))
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: FakeLap(s, fast))
    assert ice.simulate_lap(None, None, e, g).warnings == [
        "the car reaches the limiter in top gear on this track; a longer final drive may help"]
    rep = ice.EngineMap.representative_600_i4()
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: FakeLap(s, slow))
    assert ice.simulate_lap(None, None, rep, ice.Gearbox()).warnings[0] == (
        "the torque curve is a representative shape, not your engine's dyno curve")
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: FakeLap([0.0], [5.0], t=9.0))
    one = ice.simulate_lap(None, None, e, g)
    assert (one.shifts, one.shift_penalty_s, one.lap_time_s, one.gear_trace, one.warnings) == (0, 0.0, 9.0, [],
                                                                                               ["lap failed"])
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: FakeLap(s, v, ok=False, t=1.0, warning="spun"))
    assert ice.simulate_lap(None, None, e, g).warnings == ["spun"]


def test_a_shift_penalty_capped_at_the_shift_time(monkeypatch):
    from suspension import laptime as lt
    e = ENG()
    g = ice.Gearbox(**{**GB, "shift_time_s": 0.001})
    s = np.r_[np.linspace(0, 50, 30), np.linspace(52, 3000, 30)]
    v = np.r_[np.linspace(2, 20, 30), np.linspace(20.5, 40, 30)]
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: FakeLap(s, v))
    r = ice.simulate_lap(None, None, e, g)
    n, pen, _ = _penalty(g, e, s, v)
    assert r.shifts == n and r.shift_penalty_s == pytest.approx(pen) and 0 < pen <= 0.001 * n
    slowv = np.r_[np.linspace(0.2, 0.9, 30), np.linspace(0.95, 0.99, 30)]
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: FakeLap(np.linspace(0, 1, 60), slowv))
    r = ice.simulate_lap(None, None, e, ice.Gearbox(**{**GB, "ratios": (2.5, 0.1, 0.05)}))
    assert r.shift_penalty_s == pytest.approx(_penalty(ice.Gearbox(**{**GB, "ratios": (2.5, 0.1, 0.05)}), e,
                                                       np.linspace(0, 1, 60), slowv)[1])


# ------------------------------------------------------------------ fuel, tank, engine braking, cooling
def _fuel(lap, eng, gb, veh, fuel_key="gasoline", bte=0.30, over=0.05, cf=0.28):
    f = ice.fuel(fuel_key)
    s, v = np.asarray(lap.s, float), np.asarray(lap.v, float)
    ds = np.diff(s)
    vm = 0.5 * (v[1:] + v[:-1])
    dt = ds / np.maximum(vm, 0.05)
    a = np.diff(v * v) / (2 * np.maximum(ds, 1e-6))
    me = np.array([veh.m_eff(gb.overall(gb.best_gear(x, eng)), eng, gb.efficiency, gb.tire_radius_m) for x in vm])
    pw = (me * a + 0.5 * veh.rho * veh.cda_m2 * vm ** 2 + veh.crr * veh.mass_kg * 9.81) * vm
    pw = np.minimum(pw, np.array([gb.envelope_force(x, eng) for x in vm]) * vm)
    pe = np.where(pw > 0, pw / gb.efficiency, 0.0)
    rate = np.where(pw > 0, pe / (bte * f.lhv_mj_kg * 1e6), over / 1e3)
    kg = rate @ dt
    chem = rate * f.lhv_mj_kg * 1e6
    t = dt.sum()
    lit = kg / f.density_kg_l
    L = s[-1] - s[0]
    return dict(fuel_kg=kg, fuel_l=lit, co2_kg=lit * f.co2_kg_per_l, engine_work_mj=(pe * dt).sum() / 1e6,
                lap_time_s=t, lap_m=L, peak_fuel_g_s=rate.max() * 1e3, mean_coolant_heat_w=(chem * dt).sum() / t * cf,
                peak_coolant_heat_w=chem.max() * cf, l_per_100km=lit / L * 1e5)


LAP = NS(s=np.r_[0.0, 20.0, 40.0, 60.0, 60.0, 90.0, 140.0], v=np.r_[8.0, 15.0, 20.0, 18.0, 18.0, 0.0, 25.0])


def test_fuel_for_a_lap():
    e = ENG()
    g = ice.Gearbox(**GB)
    veh = ice.ICEVehicle(mass_kg=260.0)
    r = ice.fuel_for_lap(LAP, e, g, veh, fuel_key="e85", bte=0.27, overrun_fuel_g_s=0.08, coolant_frac=0.3)
    want = _fuel(LAP, e, g, veh, "e85", 0.27, 0.08, 0.3)
    for k, w in want.items():
        assert getattr(r, k) == pytest.approx(w, rel=1e-12), k
    assert r.warnings == []
    d = ice.fuel_for_lap(LAP, e, g)
    for k, w in _fuel(LAP, e, g, ice.ICEVehicle()).items():
        assert getattr(d, k) == pytest.approx(w, rel=1e-12), k
    rep = ice.fuel_for_lap(LAP, ice.EngineMap.representative_600_i4(), ice.Gearbox())
    assert rep.warnings == ["fuel rests on a declared BTE and a representative curve; replace with a BSFC map from "
                            "the dyno"]
    with pytest.raises(ValueError) as err:
        ice.fuel_for_lap(NS(s=[0.0], v=[1.0]), e, g)
    assert str(err.value) == "the lap has no speed trace"


def _fr(fuel_l=0.5, lap_m=1000.0, peak=3.0, mean_heat=9000.0):
    return ice.FuelResult(0.0, fuel_l, 0.0, 0.0, 60.0, lap_m, peak, mean_heat, 2 * mean_heat, 0.0)


def test_the_endurance_tank():
    e = ice.endurance_fuel(_fr())
    assert (e.laps, e.fuel_l, e.co2_kg, e.l_per_100km, e.tank_l, e.tank_margin_l, e.within_fuel_limit) == (
        pytest.approx(22.0), pytest.approx(11.0), pytest.approx(11.0 * 2.31), pytest.approx(50.0), None, None, False)
    t = ice.endurance_fuel(_fr(fuel_l=0.1), distance_km=10.0, tank_l=2.0, fuel_key="e85", reserve_frac=0.2)
    assert t.fuel_l == pytest.approx(1.0) and t.tank_margin_l == pytest.approx(2.0 - 1.2)
    assert t.co2_kg == pytest.approx(1.65) and t.within_fuel_limit is True and t.l_per_100km == pytest.approx(10.0)
    z = ice.endurance_fuel(_fr(lap_m=0.0), distance_km=1e-6)
    assert z.laps == pytest.approx(1000.0)                        # a zero-length lap counts as 1 um
    assert ice.endurance_fuel(_fr(fuel_l=0.26), distance_km=22.0).within_fuel_limit is True


def test_engine_braking_and_the_brake_balance():
    e = ENG(overrun_torque_nm=10.0)
    g = ice.Gearbox(**GB)
    assert ice.engine_braking_force_n(20.0, e, g) == pytest.approx(10.0 * g.overall(g.best_gear(20.0, e)) / 0.25)
    assert ice.engine_braking_force_n(20.0, e, g, gear=0) == pytest.approx(10.0 * 15.0 / 0.25)
    veh = ice.ICEVehicle(mass_kg=250.0)
    b = ice.brake_balance_with_engine_braking(1.2, 0.6, 20.0, e, g, veh, gear=1)
    tot = 1.2 * 9.81 * 250.0
    eb = 10.0 * g.overall(1) / 0.25
    assert b == {"total_n": pytest.approx(tot), "engine_braking_n": pytest.approx(eb),
                 "front_n": pytest.approx(0.6 * (tot - eb)), "rear_n": pytest.approx(tot - 0.6 * (tot - eb)),
                 "effective_front_share": pytest.approx(0.6 * (tot - eb) / tot), "declared_bias": 0.6}
    tiny = ice.brake_balance_with_engine_braking(0.01, 0.6, 5.0, e, g)
    assert tiny["engine_braking_n"] == pytest.approx(0.01 * 9.81 * 300) and tiny["front_n"] == pytest.approx(0.0)
    zero = ice.brake_balance_with_engine_braking(0.0, 0.6, 5.0, e, g)
    assert zero["effective_front_share"] == 0.0 and zero["total_n"] == 0.0


def test_size_cooling_hands_over_the_heat(monkeypatch):
    from suspension import cooling
    got = []
    monkeypatch.setattr(cooling, "size_loop", lambda spec: got.append(spec) or "sized")
    assert ice.size_cooling(_fr(mean_heat=8000.0)) == "sized"
    s = got[-1]
    assert (s.heat_w, s.coolant_key, s.flow_lpm, s.ambient_c, s.coolant_in_c, s.max_coolant_c, s.radiator_ua_w_per_k,
            s.loop_volume_l, s.cap_pressure_bar) == (8000.0, "eg50", 25.0, 35.0, 85.0, 105.0, 700.0, 2.5, 1.1)
    ice.size_cooling(_fr(mean_heat=8000.0), flow_lpm=30.0, ambient_c=30.0, coolant_in_c=80.0, max_coolant_c=100.0,
                     radiator_ua_w_per_k=650.0, loop_volume_l=3.0, cap_pressure_bar=1.3, coolant_key="water",
                     use_peak=True)
    s = got[-1]
    assert (s.heat_w, s.coolant_key, s.flow_lpm, s.ambient_c, s.coolant_in_c, s.max_coolant_c, s.radiator_ua_w_per_k,
            s.loop_volume_l, s.cap_pressure_bar) == (16000.0, "water", 30.0, 30.0, 80.0, 100.0, 650.0, 3.0, 1.3)


# ------------------------------------------------------------------ the feasibility gate
def _gate(**kw):
    return [(f.check, f.status, f.detail) for f in ice.feasibility_gate(ENG(), ice.Gearbox(**GB), _fr(), **kw)]


def test_an_undeclared_car_is_missing_everything():
    e = ENG()
    rc = e.restrictor_check("gasoline", 0.30)
    end = ice.endurance_fuel(_fr())
    assert _gate() == [
        ("restrictor diameter", "MISSING", "declare it; the rule maximum for Gasoline (100 octane) is 20.0 mm"),
        ("curve against the restrictor", "PASS",
         f"peak {rc['peak_kw']:.1f} kW at {rc['peak_rpm']:.0f} rpm against a {rc['ceiling_kw']:.1f} kW ceiling at BTE "
         "0.30"),
        ("fuel tank", "MISSING", f"endurance needs {end.fuel_l:.2f} L plus 10% reserve; declare the usable tank volume"),
        ("fuel limit", "WARN", f"{end.l_per_100km:.1f} L/100 km against 26 L/100 km (2024 efficiency proposal; check "
                               f"the current rules); CO2 {end.co2_kg:.2f} kg"),
        ("fuel pump", "MISSING", f"peak demand {3.0 * 3.6 / 0.745:.1f} L/h; declare the pump's flow at regulated "
                                 "pressure"),
        ("injector duty", "MISSING", "declare the injector flow (cc/min at rail pressure)"),
        ("cooling", "MISSING", "size the loop with size_cooling()")]


def test_every_declared_check():
    peak_lph = 3.0 * 3.6 / 0.781
    end = ice.endurance_fuel(_fr(), tank_l=15.0, fuel_key="e85")
    cool = NS(ok=False, spec=NS(heat_w=9100.0), ua_margin=0.83, steady_coolant_c=111.4)
    vmax = ice.Gearbox(**GB).top_speed_at_redline(ENG())
    out = _gate(fuel_key="e85", bte=0.2, restrictor_mm=19.5, tank_l=15.0, fuel_pump_lph=1.2 * peak_lph - 0.01,
                injector_cc_min=100.0, n_injectors=2, lap_top_speed_ms=vmax + 1.0, cooling=cool)
    rc = ENG().restrictor_check("e85", 0.2, d_mm=19.5)
    assert rc["exceeds"] is True
    duty = 3.0 / (100.0 / 60 * 0.781 * 2)
    assert out == [
        ("restrictor diameter", "FAIL", "19.50 mm exceeds the 19.0 mm maximum for E85"),
        ("curve against the restrictor", "WARN",
         f"peak {rc['peak_kw']:.1f} kW at {rc['peak_rpm']:.0f} rpm against a {rc['ceiling_kw']:.1f} kW ceiling at BTE "
         "0.20; the curve is unrestricted or the BTE is low"),
        ("fuel tank", "PASS", f"{end.fuel_l:.2f} L for 22 km plus 10% reserve against 15.00 L "
                              f"(margin {end.tank_margin_l:+.2f} L)"),
        ("fuel limit", "WARN", f"{end.l_per_100km:.1f} L/100 km against 26 L/100 km (2024 efficiency proposal; check "
                               f"the current rules); CO2 {end.co2_kg:.2f} kg"),
        ("fuel pump", "FAIL", f"{1.2 * peak_lph - 0.01:.0f} L/h against {peak_lph:.1f} L/h peak x 1.2"),
        ("injector duty", "FAIL", f"{duty:.0%} at peak demand (limit 85%)"),
        ("top gear", "WARN", f"redline in top gear at {vmax * 3.6:.0f} km/h; the lap reaches {(vmax + 1) * 3.6:.0f} km/h"),
        ("cooling", "FAIL", "9.1 kW to coolant; radiator UA margin 0.83, steady coolant 111 C")]


def test_the_gate_edges():
    end = ice.endurance_fuel(_fr(), tank_l=1.0)
    exact_tank = end.fuel_l * (1 + 0.10)
    peak_lph = 3.0 * 3.6 / 0.745
    vmax = ice.Gearbox(**GB).top_speed_at_redline(ENG())
    out = dict((c, (s, d)) for c, s, d in _gate(restrictor_mm=20.0, tank_l=exact_tank, fuel_pump_lph=1.2 * peak_lph,
                                                injector_cc_min=1000.0, lap_top_speed_ms=vmax,
                                                cooling=NS(ok=True, spec=NS(heat_w=5000.0), ua_margin=1.5,
                                                           steady_coolant_c=95.0)))
    assert out["restrictor diameter"] == ("PASS", "20.00 mm within 20.0 mm")
    assert out["fuel tank"][0] == "PASS" and out["fuel pump"][0] == "PASS" and out["top gear"][0] == "PASS"
    assert out["injector duty"] == ("PASS", f"{3.0 / (1000.0 / 60 * 0.745 * 4):.0%} at peak demand (limit 85%)")
    assert out["cooling"] == ("PASS", "5.0 kW to coolant; radiator UA margin 1.50, steady coolant 95 C")
    small = dict((c, s) for c, s, _ in _gate(tank_l=exact_tank - 1e-6))
    assert small["fuel tank"] == "FAIL"
    lean = ice.feasibility_gate(ENG(), ice.Gearbox(**GB), _fr(fuel_l=0.1))
    assert [f.status for f in lean if f.check == "fuel limit"] == ["PASS"]
    rep = ice.feasibility_gate(ice.EngineMap.representative_600_i4(), ice.Gearbox(), _fr())
    assert (rep[-1].check, rep[-1].status, rep[-1].detail) == ("torque curve", "WARN", "representative shape; "
                                                                "declare the dyno or engine-code curve")
    zero_cap = dict((c, (s, d)) for c, s, d in _gate(injector_cc_min=0.0))
    assert zero_cap["injector duty"][0] == "FAIL"


# ------------------------------------------------------------------ comparing options, the ledger
def test_compare_engines(monkeypatch):
    calls = []

    def launch(eng, gb, veh, fuel_key, bte):
        calls.append(("launch", eng, veh.mass_kg, fuel_key, bte))
        return NS(time_s=4.1)

    def lap(vd, track, eng, gb, veh):
        calls.append(("lap", vd.p.mass, track, veh.mass_kg))
        return NS(lap="L", lap_time_s=55.0)

    def fl(l, eng, gb, veh, fuel_key, bte):
        calls.append(("fuel", l, veh.mass_kg, fuel_key, bte))
        return _fr(fuel_l=0.2)
    monkeypatch.setattr(ice, "simulate_launch", launch)
    monkeypatch.setattr(ice, "simulate_lap", lap)
    monkeypatch.setattr(ice, "fuel_for_lap", fl)
    from dataclasses import dataclass

    @dataclass
    class P:
        mass: float = 290.0
    vd = NS(p=P())
    e = ENG()
    opts = [ice.EngineOption("a", e, ice.Gearbox(**GB)),
            ice.EngineOption("b", e, ice.Gearbox(**GB), mass_delta_kg=-12.0, fuel_key="e85", bte=0.33),
            ice.EngineOption("bad", e, ice.Gearbox(**GB), fuel_key="diesel")]
    o = ice.EngineOption("x", e, ice.Gearbox())
    assert (o.mass_delta_kg, o.fuel_key, o.bte) == (0.0, "gasoline", 0.30)
    monkeypatch.setattr(ice, "endurance_fuel", lambda fr, fuel_key: (_ for _ in ()).throw(ValueError("nope"))
                        if fuel_key == "diesel" else NS(fuel_l=4.4, co2_kg=9.9))
    rows = ice.compare_engines(opts, vd, "T", ice.ICEVehicle(mass_kg=300.0))
    pk = e.peak()["power_kw"]
    assert rows[0] == {"option": "a", "t_75m_s": 4.1, "lap_s": 55.0, "endurance_fuel_l": 4.4, "co2_kg": 9.9,
                       "mass_kg": 300.0, "peak_kw": pk, "error": ""}
    assert rows[1]["mass_kg"] == 288.0 and rows[2] == {"option": "bad", "error": "nope"}
    assert ("launch", e, 288.0, "e85", 0.33) in calls and ("lap", 278.0, "T", 288.0) in calls
    assert ("fuel", "L", 288.0, "e85", 0.33) in calls and vd.p.mass == 290.0
    default = ice.compare_engines(opts[:1], vd, "T")
    assert default[0]["mass_kg"] == 300.0


def test_publishing_to_the_ledger():
    L = ifm.IntegrationLedger()
    e = ENG()
    pk = e.peak()
    assert ice.publish_to_ledger(L, e) is L
    pt = L.get("powertrain")
    assert (pt.peak_torque_nm, pt.peak_power_kw, pt.updated_by, pt.is_estimate) == (
        pk["torque_nm"], pk["power_kw"], "suspension.ice_powertrain", True)
    assert L.get("cooling") is None
    L.set(ifm.SubsystemInterface(name="cooling", heat_reject_w=1.0))
    ice.publish_to_ledger(L, e, _fr(mean_heat=7000.0), updated_by="me")
    c = L.get("cooling")
    assert (c.heat_reject_w, c.updated_by, c.is_estimate) == (7000.0, "me", True)
    assert L.get("powertrain").updated_by == "me"
    fresh = ifm.IntegrationLedger()
    ice.publish_to_ledger(fresh, e, _fr(mean_heat=500.0))
    assert fresh.get("cooling").heat_reject_w == 500.0
    assert ice.publish_to_ledger(None, e) is None
    odd = object()
    assert ice.publish_to_ledger(odd, e) is odd

    class Put:
        def __init__(self):
            self.d = {}

        def get(self, k):
            return self.d.get(k)

        def put(self, it):
            self.d[it.name] = it
    p = Put()
    ice.publish_to_ledger(p, e, _fr())
    assert set(p.d) == {"powertrain", "cooling"}

    class NoSet:
        def get(self, k):
            return None
    assert isinstance(ice.publish_to_ledger(NoSet(), e, _fr()), NoSet)

    class Boom:
        def get(self, k):
            raise RuntimeError
    b = Boom()
    assert ice.publish_to_ledger(b, e) is b


def test_pass_through_and_edges(monkeypatch):
    """Non-default objects everywhere one is handed on, so a dropped or swapped argument changes the answer."""
    from suspension import laptime as lt
    e = ENG()
    g = ice.Gearbox(**dict(GB, efficiency=0.85))
    veh = ice.ICEVehicle(mass_kg=255.0)
    # small but legal restrictor inputs
    assert ice.choked_mass_flow_kg_s(0.5, 0.5, 0.5, 0.9) == pytest.approx(_choked(0.5, 0.5, 0.5, 0.9), rel=1e-9)
    # the gear handed to engine braking is the one used
    v = 20.0
    for gear in (0, 2):
        bb = ice.brake_balance_with_engine_braking(0.8, 0.6, v, e, g, veh, gear=gear)
        assert bb["engine_braking_n"] == pytest.approx(min(ice.engine_braking_force_n(v, e, g, gear),
                                                           0.8 * ice.G * 255.0))
    assert ice.brake_balance_with_engine_braking(0.8, 0.6, v, e, g, veh, gear=0)["engine_braking_n"] != \
        ice.brake_balance_with_engine_braking(0.8, 0.6, v, e, g, veh, gear=2)["engine_braking_n"]
    # the declared tank is carried
    assert ice.endurance_fuel(_fr(), tank_l=7.0).tank_l == 7.0
    # the lap distance is end minus start
    off = NS(s=np.r_[100.0, 120.0, 150.0], v=np.r_[10.0, 14.0, 16.0])
    assert ice.fuel_for_lap(off, e, g).lap_m == 50.0
    two = NS(s=np.r_[0.0, 30.0], v=np.r_[10.0, 14.0])
    assert ice.fuel_for_lap(two, e, g).lap_m == 30.0
    # the powertrain carries the gearbox efficiency and the vehicle
    pt = ice.lap_powertrain(e, g, veh)
    assert pt.drivetrain_eff == 0.85 and pt.drive == "rwd"
    seen = {}

    def fake(vdyn, track, p):
        seen["pt"] = p
        return FakeLap(np.linspace(0, 100, 11), np.linspace(30, 8, 11), t=20.0)
    monkeypatch.setattr(lt, "simulate_lap", fake)
    r = ice.simulate_lap(None, None, e, g, veh)
    assert seen["pt"].tractive_force(12.0) == pytest.approx(pt.tractive_force(12.0), rel=1e-12)
    assert seen["pt"].tractive_force(12.0) != pytest.approx(ice.lap_powertrain(e, g).tractive_force(12.0), rel=1e-6)
    assert r.shifts == 0                     # slowing down through the gears is no upshift
    s = np.r_[0.0, 5.0, 40.0, 80.0]
    up = np.r_[g.speed_at(9000.0, 0) * 0.98, g.speed_at(9000.0, 0) * 1.6, g.speed_at(9000.0, 1) * 1.3,
               g.speed_at(9000.0, 1) * 1.35]
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: FakeLap(s, up, t=10.0))
    first = ice.simulate_lap(None, None, e, g)
    n, pen, gears = _penalty(g, e, s, up)
    assert gears[1] > gears[0] and first.shifts == n and first.shift_penalty_s == pytest.approx(pen, rel=1e-12)
    bad = FakeLap(s, up, ok=False, t=3.0, warning="x")
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: bad)
    assert ice.simulate_lap(None, None, e, g).lap is bad
    pair = FakeLap(np.r_[0.0, 10.0], np.r_[5.0, 9.0], t=2.0)
    monkeypatch.setattr(lt, "simulate_lap", lambda *a: pair)
    assert ice.simulate_lap(None, None, e, g).lap is pair
    # a shorter run, the default step, a custom vehicle in the sweep
    short = ice.simulate_launch(e, g, veh, distance_m=40.0, dt=2e-3)
    assert short.finished and short.distance_m == 40.0
    d = ice.simulate_launch(e, g, veh, distance_m=10.0)
    assert d.t_s[1] - d.t_s[0] == pytest.approx(1e-3)
    sw = ice.sweep_final_drive(e, g, veh, ratios=[2.5, 3.5], distance_m=30.0, dt=2e-3)
    for r_, t_ in zip(sw.ratios, sw.times_s):
        g2 = ice.Gearbox(**dict(GB, efficiency=0.85, final_drive=float(r_)))
        assert t_ == pytest.approx(ice.simulate_launch(e, g2, veh, distance_m=30.0, dt=2e-3).time_s, rel=1e-12)
    # fine rpm grids and ratios under one are legal; a zero final drive is not
    ice.EngineMap(rpm=[1000.0, 1000.5, 1001.0], torque_nm=[1.0, 1.0, 1.0], redline_rpm=1001.0)
    ice.Gearbox(primary=0.5, ratios=(2.0,), final_drive=0.5)
    with pytest.raises(ValueError):
        ice.Gearbox(primary=2.0, ratios=(2.0,), final_drive=0.0)
    # publishing keeps what the ledger already knew about each subsystem
    L = ifm.IntegrationLedger()
    L.set(ifm.SubsystemInterface(name="powertrain", mass_kg=50.0))
    L.set(ifm.SubsystemInterface(name="cooling", cooling_airflow_cms=0.3))
    ice.publish_to_ledger(L, e, _fr())
    assert L.get("powertrain").mass_kg == 50.0 and L.get("cooling").cooling_airflow_cms == 0.3


def test_compare_engines_hands_on_its_own_objects(monkeypatch):
    e = ENG()
    g = ice.Gearbox(**GB)
    fr = _fr(fuel_l=0.3)
    got = {}
    monkeypatch.setattr(ice, "simulate_launch", lambda eng, gb, veh, fuel_key, bte: got.update(launch=(eng, gb))
                        or NS(time_s=4.0))
    monkeypatch.setattr(ice, "simulate_lap", lambda vd, track, eng, gb, veh: got.update(lap=(eng, gb))
                        or NS(lap="L", lap_time_s=50.0))
    monkeypatch.setattr(ice, "fuel_for_lap", lambda l, eng, gb, veh, fuel_key, bte: got.update(fuel=(eng, gb)) or fr)
    monkeypatch.setattr(ice, "endurance_fuel", lambda f, fuel_key: got.update(end=f) or NS(fuel_l=1.0, co2_kg=2.0))

    from dataclasses import dataclass

    @dataclass
    class P:
        mass: float = 290.0
    ice.compare_engines([ice.EngineOption("a", e, g)], NS(p=P()), "T")
    assert got["launch"][0] is e and got["launch"][1] is g and got["lap"][0] is e and got["lap"][1] is g
    assert got["fuel"][0] is e and got["fuel"][1] is g and got["end"] is fr
