# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""suspension/ev_electrical_check.py against closed forms: road-load power, pack and cell current and energy at
constant speed and constant acceleration, the fuse-limited top speed, every verdict and warning, and reading the
electrics lead's workbook. SI units (m, s, N, W, A, V) except where a name says kW, kWh, mph or inches."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import ev_electrical_check as ec                              # noqa: E402

openpyxl = pytest.importorskip("openpyxl")
P = ec.ElecParams()
M, CDA, CRR, RHO, ETA = 230.0, 1.10, 0.018, 1.225, 0.90


def test_derived_pack_figures_are_volts_times_amps():
    assert P.pack_energy_kwh == pytest.approx(504.0 * 15.0 / 1000.0)
    assert P.usable_energy_kwh == pytest.approx(0.92 * P.pack_energy_kwh)
    assert P.max_power_from_fuse_kw == pytest.approx(50.0 * 504.0 / 1000.0)


def test_constant_speed_draws_road_load_power_and_its_energy():
    """At constant v: P = (rho*CdA*v^2/2 + Crr*m*g)*v / eta; I = P/V; energy = P * L/v."""
    v0, L = 15.0, 300.0
    s = np.linspace(0.0, L, 301)
    r = ec.check_lap_electrical(np.full_like(s, v0), s, P)
    Pw = (0.5 * RHO * CDA * v0 ** 2 + CRR * M * 9.81) * v0 / ETA
    assert np.allclose(r.power_profile_kw, Pw / 1000.0) and np.allclose(r.current_profile_a, Pw / 504.0)
    assert r.peak_current_a == pytest.approx(Pw / 504.0) and r.avg_current_a == pytest.approx(Pw / 504.0)
    assert r.peak_cell_current_a == pytest.approx(Pw / 504.0 / 3)
    assert r.energy_used_kwh == pytest.approx(Pw * L / v0 / 3.6e6, rel=1e-12)
    assert r.ok and r.summary.startswith("✅") and r.warnings == []


def test_constant_acceleration_adds_m_a_to_the_road_load():
    """v^2 = v0^2 + 2 a s: the tractive force is the road load plus m*a at every point."""
    a, v0 = 2.0, 5.0
    s = np.linspace(0.0, 100.0, 4001)
    v = np.sqrt(v0 ** 2 + 2 * a * s)
    r = ec.check_lap_electrical(v, s, P)
    F = 0.5 * RHO * CDA * v ** 2 + CRR * M * 9.81 + M * a
    assert np.allclose(r.power_profile_kw * 1000.0, F * v / ETA, rtol=1e-3)
    assert r.peak_current_speed_ms == pytest.approx(v[-1])


def test_braking_draws_nothing_from_the_pack():
    """Decelerating harder than the road load alone would: the brakes do it, the motor is off.

    The old model clamped only the m*a term and kept drag and rolling power on the
    pack through every braking zone.
    """
    s = np.linspace(0.0, 100.0, 101)
    v = np.linspace(30.0, 10.0, 101)                     # a = v dv/ds = -0.2 v, up to -6 m/s^2
    r = ec.check_lap_electrical(v, s, P)
    assert np.all(r.power_profile_kw == 0.0) and r.peak_current_a == 0.0


def test_road_load_power_counts_the_net_force():
    v = np.array([0.0, 10.0, 10.0, 10.0])
    road = 0.5 * RHO * CDA * 100 + CRR * M * 9.81
    a = np.array([3.0, 0.0, 1.0, -road / M])             # launch, cruise, accel, coast
    F, Pw, Pp = ec.traction_power_w(v, a)
    assert F == pytest.approx([M * 3 + CRR * M * 9.81, road, road + M, 0.0])
    assert Pw == pytest.approx(F * v) and Pp[0] == 0.0     # P = F v: nothing at standstill
    assert Pp[1:] == pytest.approx(np.maximum(F[1:] * 10, 0) / ETA)
    assert ec.traction_power_w([10.0], [-5.0])[2][0] == 0.0
    assert ec.fuse_limited_speed(1e9) == 100.0


def test_the_fuse_limited_speed_solves_the_road_load_power_balance():
    s = np.linspace(0.0, 100.0, 11)
    r = ec.check_lap_electrical(np.full(11, 10.0), s, P)
    P_cap = P.max_power_from_fuse_kw * 1000.0 * ETA
    road = lambda v: 0.5 * RHO * CDA * v ** 3 + CRR * M * 9.81 * v
    assert road(r.max_safe_speed_ms) >= P_cap > road(r.max_safe_speed_ms - 0.03)
    big = ec.check_lap_electrical(np.full(11, 10.0), s, ec.ElecParams(fuse_max_a=5000.0))
    assert big.max_safe_speed_ms == 100.0                                    # the search ceiling


def test_every_limit_trips_its_flag_and_names_the_numbers():
    s = np.linspace(0.0, 2000.0, 401)
    v = np.minimum(np.sqrt(4.0 + 2 * 6.0 * s), 45.0)
    tight = ec.ElecParams(pack_capacity_ah=0.2, fuse_max_a=20.0, nominal_cell_current_a=5.0)
    r = ec.check_lap_electrical(v, s, tight)
    assert r.fuse_blown and r.cell_overcurrent and r.energy_empty and not r.ok
    assert f"{float(np.max(r.power_profile_kw)):.1f} kW at its peak" in r.fuse_message
    assert f"caps you at {tight.max_power_from_fuse_kw:.1f} kW" in r.fuse_message
    assert "Cell overcurrent" in r.cell_message and "PACK RUNS DRY" in r.energy_message
    assert "fuse over-current, cell over-current, energy deficit" in r.summary
    assert any("Top speed" in w for w in r.warnings) and any("below 10 Ah" in w for w in r.warnings)


def test_an_empty_pack_reports_zero_percent_used():
    s = np.linspace(0.0, 10.0, 11)
    r = ec.check_lap_electrical(np.ones(11), s, ec.ElecParams(pack_capacity_ah=0.0, fuse_max_a=1e6,
                                                                nominal_cell_current_a=1e6))
    assert r.energy_empty and "PACK RUNS DRY" in r.energy_message
    z = ec.ElecParams(pack_voltage_v=0.0, pack_capacity_ah=15.0, fuse_max_a=1e9, nominal_cell_current_a=1e9)
    q = ec.check_lap_electrical(np.ones(11), s, z)
    assert q.energy_empty or "(0.0% of" in q.energy_message


def test_too_little_data_is_refused_politely():
    r = ec.check_lap_electrical([10.0], [0.0], P)
    assert not r.ok and "Not enough speed data" in r.summary


def test_a_time_trace_in_mph_is_the_same_lap():
    t = np.linspace(0.0, 20.0, 201)
    mph = np.full_like(t, 30.0)
    r = ec.check_lap_from_speed_csv(mph, t, P)
    v = 30.0 * 0.44704
    direct = ec.check_lap_electrical(np.full_like(t, v), v * t, P)
    assert r.energy_used_kwh == pytest.approx(direct.energy_used_kwh) and r.peak_current_a == direct.peak_current_a


# --------------------------------------------------------------------------- #
#  The workbook
# --------------------------------------------------------------------------- #
def _workbook(path, pack_name="Battery Pack Calcs", ep_name="ElecPropulsion", svt_name="SpeedVsTime", ep_extra=20,
              **over):
    wb = openpyxl.Workbook()
    wb.remove(wb.active)
    pack = {"Parrallel Battery Count": 4, "Series Battery Count": 120, "Capacity Battery Cell (Ah)": 4.0,
            "Internal Resistance Battery Cell (Ohms)": 0.015, "Battery Pack Nominal Voltage (V)": 432.0,
            "Pack Capacity (Ah)": 16.0, "Fuse Max (A)": 80.0, "Current Across a Single Cell (A)": 20.0}
    pack.update(over)
    ws = wb.create_sheet(pack_name)
    for k, val in pack.items():
        ws.append([k, val])
    ws.append(["Blank label", None])
    ep = wb.create_sheet(ep_name)
    for k, val in {"Motor Peak Power (kW)": 80.0, "Motor Peak Torque (Nm)": 140.0, "Motor Efficiency ": 0.94,
                   "Motor Max DC Voltage (V)": 600.0}.items():
        ep.append([k, val])
    ep.append([None, None])                                       # a spacer row the reader skips
    ep.append(["Wheel Diameter (in)", 16.0])
    for i in range(ep_extra):                                     # rows past the first 12 are not parameters
        ep.append([f"later row {i}", i])
    sv = wb.create_sheet(svt_name)
    sv.append(["time (s)", "speed (mph)"])
    for t in range(5):
        sv.append([float(t), 20.0 + t])
    sv.append(["note", "x"])
    wb.save(path)
    return path


def test_the_workbook_is_read_by_label(tmp_path):
    p = ec.ElecParams.from_excel(str(_workbook(tmp_path / "a.xlsx")))
    assert (p.n_parallel, p.n_series, p.cell_capacity_ah, p.cell_r_ohm) == (4, 120, 4.0, 0.015)
    assert (p.pack_voltage_v, p.pack_capacity_ah, p.fuse_max_a, p.nominal_cell_current_a) == (432.0, 16.0, 80.0, 20.0)
    assert (p.motor_peak_power_kw, p.motor_peak_torque_nm, p.motor_efficiency) == (80.0, 140.0, 0.94)
    assert (p.motor_max_dc_v, p.wheel_diameter_in) == (600.0, 16.0)
    assert p.pack_energy_kwh == pytest.approx(432.0 * 16.0 / 1000.0)


def test_renamed_sheets_are_still_found(tmp_path):
    path = _workbook(tmp_path / "b.xlsx", pack_name="BatteryPackConfig", ep_name="Elec Propulsion",
                     svt_name="Speed Vs Time", ep_extra=0)
    assert ec.ElecParams.from_excel(str(path)).fuse_max_a == 80.0
    t, mph = ec.load_speed_vs_time(str(path))
    assert list(t) == [0.0, 1.0, 2.0, 3.0, 4.0] and list(mph) == [20.0, 21.0, 22.0, 23.0, 24.0]


def test_missing_values_fall_back_to_what_the_others_imply(tmp_path):
    path = _workbook(tmp_path / "c.xlsx", **{"Battery Pack Nominal Voltage (V)": "n/a", "Pack Capacity (Ah)": None,
                                            "Current Across a Single Cell (A)": None})
    p = ec.ElecParams.from_excel(str(path))
    assert p.pack_voltage_v == pytest.approx(120 * 3.6) and p.pack_capacity_ah == pytest.approx(4.0 * 4)
    assert p.nominal_cell_current_a == pytest.approx(80.0 / 4)             # the fuse shared by the strings


def test_an_unreadable_workbook_gives_the_defaults(tmp_path, monkeypatch):
    assert ec.ElecParams.from_excel(str(tmp_path / "missing.xlsx")) == ec.ElecParams()
    assert ec.ElecParams.from_excel_bytes(b"not a workbook") == ec.ElecParams()
    good = _workbook(tmp_path / "d.xlsx").read_bytes()
    assert ec.ElecParams.from_excel_bytes(good).fuse_max_a == 80.0
    empty = tmp_path / "e.xlsx"
    wb = openpyxl.Workbook()
    wb.save(empty)
    assert ec.ElecParams.from_excel(str(empty)) == ec.ElecParams(pack_voltage_v=140 * 3.6, nominal_cell_current_a=50.0 / 3)
    monkeypatch.setitem(sys.modules, "openpyxl", None)
    assert ec.ElecParams.from_excel(str(empty)) == ec.ElecParams()
    assert ec.ElecParams.from_excel_bytes(good) == ec.ElecParams()
    with pytest.raises(ImportError, match="openpyxl is required"):
        ec.load_speed_vs_time(str(empty))


def test_the_speed_trace_must_exist_and_have_two_points(tmp_path):
    good = _workbook(tmp_path / "f.xlsx")
    t, mph = ec.load_speed_vs_time_from_bytes(good.read_bytes())
    assert len(t) == 5
    wb = openpyxl.load_workbook(good)
    del wb["SpeedVsTime"]
    wb.save(tmp_path / "g.xlsx")
    with pytest.raises(ValueError, match="No speed-vs-time sheet"):
        ec.load_speed_vs_time(str(tmp_path / "g.xlsx"))
    wb = openpyxl.load_workbook(good)
    ws = wb["SpeedVsTime"]
    ws.delete_rows(3, 10)
    ws.append([7.0])
    wb.save(tmp_path / "h.xlsx")
    with pytest.raises(ValueError, match="only 1 numeric data row"):
        ec.load_speed_vs_time(str(tmp_path / "h.xlsx"))
    one_col = openpyxl.Workbook()
    one_col.active.title = "SpeedVsTime"
    for x in (1.0, 2.0, 3.0):
        one_col.active.append([x])                                # a single column: no speed at all
    one_col.save(tmp_path / "i.xlsx")
    with pytest.raises(ValueError, match="only 0 numeric data row"):
        ec.load_speed_vs_time(str(tmp_path / "i.xlsx"))
    with pytest.raises(ValueError, match="Could not open"):
        ec.load_speed_vs_time(str(tmp_path / "nope.xlsx"))
