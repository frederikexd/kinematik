"""Characterization suite for suspension/ev_electrical_check.py (mutation gate); see tests/_golden.py."""
import math
import sys

import numpy as np
import openpyxl
import pytest

from suspension import ev_electrical_check as ec
from tests._golden import check, plain

PACK = [("Parrallel Battery Count", 4), ("Series Battery Count", 120), ("Capacity Battery Cell (Ah)", 4.5),
        ("Internal Resistance Battery Cell (Ohms)", 0.02), ("Battery Pack Nominal Voltage (V)", 430.0),
        ("Pack Capacity (Ah)", 18.0), ("Fuse Max (A)", 60.0), ("Current Across a Single Cell (A)", 14.0)]
EP = [("Motor Peak Power (kW)", 80.0), ("Motor Peak Torque (Nm)", 200.0), ("Motor Efficiency ", 0.9),
      ("Motor Max DC Voltage (V)", 600.0), ("Wheel Diameter (in)", 16.0)]


def _wb(path, sheets):
    wb = openpyxl.Workbook()
    wb.remove(wb.active)
    for name, rows in sheets.items():
        ws = wb.create_sheet(name)
        for r in rows:
            ws.append(list(r))
    if not wb.worksheets:
        wb.create_sheet("x")
    wb.save(path)
    return str(path)


def _ep_rows(extra_at=None):
    rows = [("pad", None)] * 6 + EP
    if extra_at:
        rows = rows + [("pad", None)] * (extra_at - len(rows) - 1) + [("Wheel Diameter (in)", 99.0)]
    return rows


def test_from_excel_cases(tmp_path, monkeypatch):
    cases = {
        "full": {"BatteryPackConfig": PACK + [(None, 1), ("blank", None), ("  Fuse Max (A)  ", 70.0)],
                 "ElecPropulsion": EP},
        "aliases": {"BatteryPackConfig": [("parallel battery count", 2), (" series battery count ", 100),
                                           ("Fuse Max (A)", "abc")],
                    "ElecPropulsion": [("motor efficiency", 0.8)]},
        "defaults": {"Battery Pack Calcs": [("Fuse Max (A)", 0)], "Other": [("a", 1)]},
        "one_col": {"BatteryPackConfig": [("Series Battery Count",), ("Fuse Max (A)",)],
                    "ElecPropulsion": [("Motor Peak Power (kW)",)]},
        "ep_row12": {"BatteryPackConfig": PACK, "ElecPropulsion": _ep_rows(12)},
        "ep_row13": {"BatteryPackConfig": PACK, "ElecPropulsion": _ep_rows(13)},
        "ep_late": {"BatteryPackConfig": PACK, "ElecPropulsion": [("pad", 0)] * 11 + EP},
        "zero_par": {"BatteryPackConfig": [("Parrallel Battery Count", 0.4), ("Fuse Max (A)", 30.0)]},
        "empty": {},
    }
    res = {}
    for k, sheets in cases.items():
        p = _wb(tmp_path / f"{k}.xlsx", sheets)
        res[k] = ec.ElecParams.from_excel(p)
        res[k + "_b"] = ec.ElecParams.from_excel_bytes(open(p, "rb").read())
    res["missing"] = ec.ElecParams.from_excel(str(tmp_path / "nope.xlsx"))
    res["garbage"] = ec.ElecParams.from_excel_bytes(b"not a workbook")
    res["direct"] = ec.ElecParams(pack_voltage_v=300.0, pack_capacity_ah=8.0, fuse_max_a=40.0)
    check("ev_elec_params", res)
    p = _wb(tmp_path / "full2.xlsx", cases["full"])
    data = open(p, "rb").read()
    monkeypatch.setitem(sys.modules, "openpyxl", None)
    assert plain(ec.ElecParams.from_excel(p)) == plain(ec.ElecParams())
    assert plain(ec.ElecParams.from_excel_bytes(data)) == plain(ec.ElecParams())
    with pytest.raises(ImportError, match="^openpyxl is required to read Excel files. Install it with: pip"):
        ec.load_speed_vs_time(p)


@pytest.mark.filterwarnings("ignore::suspension.plausibility.PlausibilityWarning")   # textbook numbers, not a car
def test_power_helpers():
    res = {}
    v = np.array([0.0, 5.0, 20.0, 35.0])
    a = np.array([3.0, -8.0, 0.5, 0.0])
    res["tp"] = [ec.traction_power_w(v, a), ec.traction_power_w(v, a, vehicle_mass_kg=300.0, drag_cda=0.9,
                                                                crr=0.02, rho_air=1.1, drivetrain_eff=0.3),
                 ec.traction_power_w(12.0, 1.0, drivetrain_eff=0.8)]
    res["fls"] = [ec.fuse_limited_speed(p) for p in (0.0, 5e3, 25e3, 1e8)]
    res["fls_kw"] = [ec.fuse_limited_speed(25e3, vehicle_mass_kg=300.0, drag_cda=0.8, crr=0.03, rho_air=1.0,
                                           drivetrain_eff=0.95),
                     ec.fuse_limited_speed(25e3, drivetrain_eff=0.2)]
    check("ev_elec_power", res)
    b = np.array([-1.0, -8.0, -3.0, -0.2])
    check("ev_elec_regen", {"r": [ec.traction_power_w(v, b, regen_eff=0.7),
                                  ec.traction_power_w(v, b, regen_eff=0.6, regen_cap_w=8e3, drivetrain_eff=0.8),
                                  ec.traction_power_w(v, a, regen_eff=0.5, rot_mass_factor=1.1),
                                  ec.traction_power_w(v, b, rot_mass_factor=1.07),
                                  ec.traction_power_w(v, b, regen_eff=0.7, aux_power_w=250.0)]})
    pt = np.array([-60e3, -5e3, 0.0, 5e3, 60e3, 400e3])
    check("ev_elec_ir", {"b": [ec.battery_internal_power_w(pt, 355.0, 0.09),
                               ec.battery_internal_power_w(pt, 400.0, 0.2),
                               ec.battery_internal_power_w(pt, 355.0, 0.0),
                               ec.battery_internal_power_w(12e3, 300.0, -0.1)]})


def _lap(n=60):
    s = np.linspace(0.0, 600.0, n)
    v = 12.0 + 10.0 * np.sin(s / 60.0) ** 2
    return v, s


def test_lap_checks():
    v, s = _lap()
    P = ec.ElecParams
    cases = {
        "default": (v, s, P(), {}),
        "fuse": (v * 2.2, s, P(), {}),
        "cell": (v, s, P(nominal_cell_current_a=5.0), {}),
        "energy": (v, s * 40.0, P(pack_capacity_ah=1.0), {}),
        "all": (v * 2.5, s * 30.0, P(pack_capacity_ah=0.5, nominal_cell_current_a=2.0), {}),
        "small_pack": (v, s, P(pack_capacity_ah=9.9), {}),
        "cap10": (v, s, P(pack_capacity_ah=10.0), {}),
        "stopped": (np.zeros(5), np.linspace(0, 4, 5), P(pack_capacity_ah=0.0), {}),
        "crawl": (np.array([0.0, 0.2, 0.4, 0.3, 0.0]), np.array([0.0, 0.1, 0.3, 0.5, 0.6]), P(), {}),
        "lowv": (v, s, P(pack_voltage_v=0.5, n_parallel=0), {}),
        "kw": (v, s, P(fuse_max_a=20.0), dict(drivetrain_eff=0.3, vehicle_mass_kg=320.0, drag_cda=1.4,
                                              crr=0.03, rho_air=1.0)),
        "short": ([5.0], [0.0], P(), {}),
        "empty": ([], [], P(), {}),
        "coast": (np.linspace(30.0, 10.0, 20), np.linspace(0.0, 300.0, 20), P(), {}),
    }
    res = {k: ec.check_lap_electrical(a, b, p, **kw) for k, (a, b, p, kw) in cases.items()}
    t = np.linspace(0.0, 30.0, 31)
    mph = 20.0 + 15.0 * np.sin(t / 5.0)
    res["csv"] = ec.check_lap_from_speed_csv(mph, t, P())
    res["csv_kw"] = ec.check_lap_from_speed_csv(mph, t, P(), drivetrain_eff=0.7, vehicle_mass_kg=280.0,
                                                drag_cda=1.3, crr=0.025)
    res["csv_one"] = ec.check_lap_from_speed_csv([3.0], [0.0], P())
    check("ev_elec_lap", res)


def test_speed_vs_time_loader(tmp_path):
    res = {}
    rows = [("Time (s)", "Speed (mph)"), (0, 10), (1, 12.5), ("x", 3), (2, None), (3.5, 20)]
    ok = _wb(tmp_path / "ok.xlsx", {"SpeedVsTime": rows})
    res["ok"] = ec.load_speed_vs_time(ok)
    res["ok_b"] = ec.load_speed_vs_time_from_bytes(open(ok, "rb").read())
    res["alias"] = ec.load_speed_vs_time(_wb(tmp_path / "al.xlsx", {"Speed Vs Time": [(0, 1), (1, 2)]}))
    one = _wb(tmp_path / "one.xlsx", {"SpeedVsTime": [(0,), (1,), (2,)]})
    few = _wb(tmp_path / "few.xlsx", {"SpeedVsTime": [(0, 1), ("a", "b")]})
    none = _wb(tmp_path / "none.xlsx", {"A": [(1, 2)], "B": [(1, 2)]})
    for k, p in (("one", one), ("few", few), ("none", none), ("bad", str(tmp_path / "zz.xlsx"))):
        try:
            ec.load_speed_vs_time(p)
            res[k] = "no error"
        except ValueError as e:
            res[k] = str(e).replace(str(tmp_path), "<T>")
    try:
        ec.load_speed_vs_time_from_bytes(open(none, "rb").read())
    except ValueError as e:
        res["none_b"] = str(e)
    check("ev_elec_svt", res)


def test_more_lap_edges():
    P = ec.ElecParams
    res = {"two": ec.check_lap_electrical([10.0, 12.0], [0.0, 20.0], P()),
           "tiny_pack": ec.check_lap_electrical(np.linspace(5, 8, 6), np.linspace(0, 50, 6), P(pack_capacity_ah=1.0)),
           "csv_half": ec.check_lap_from_speed_csv([10.0, 14.0, 20.0, 18.0, 12.0], [0.0, 0.5, 1.0, 1.5, 2.0], P())}
    check("ev_elec_edges", res)


def test_a_motor_map_sets_the_drive_efficiency():
    v, a = np.r_[5.0, 10.0, 20.0, 30.0, 25.0], np.r_[2.0, 0.5, -3.0, 0.2, -0.05]
    tab = ((0.0, 1e4, 4e4), (0.80, 0.90, 0.95))
    F, pw, pp = ec.traction_power_w(v, a, vehicle_mass_kg=300.0, drag_cda=1.0, crr=0.015, rho_air=1.2,
                                    drivetrain_eff=0.95, regen_eff=0.6, regen_cap_w=2e4, aux_power_w=150.0,
                                    motor_eff_map=tab)
    f = 300.0 * a + 0.6 * v ** 2 + 0.015 * 300.0 * 9.81
    shaft = np.maximum(f * v, 0.0) / 0.95
    eta = np.interp(shaft, tab[0], tab[1])
    assert F == pytest.approx(f, rel=1e-12) and pw == pytest.approx(f * v, rel=1e-12)
    assert pp == pytest.approx(shaft / eta + 0.6 * np.maximum(np.minimum(f * v, 0.0), -2e4) + 150.0, rel=1e-12)
    plain_pp = ec.traction_power_w(v, a, vehicle_mass_kg=300.0, drag_cda=1.0, crr=0.015, rho_air=1.2,
                                   drivetrain_eff=0.95)[2]
    assert plain_pp == pytest.approx(shaft, rel=1e-12)                 # no map: the gearing alone


def test_motor_efficiency_from_a_table():
    tab = ((0.0, 1e4, 4e4), (0.80, 0.90, 1.0))
    p = np.r_[0.0, 5e3, 1e4, 2.5e4, 4e4, 9e4]
    assert ec.motor_efficiency(p, tab) == pytest.approx([0.80, 0.85, 0.90, 0.95, 1.0, 1.0], rel=1e-12)
    assert ec.motor_efficiency(-2.5e4, tab) == pytest.approx(0.95)  # regenerating: by the size of the power
    assert ec.motor_efficiency(5e3, ((1e4, 2e4), (0.7, 0.9))) == pytest.approx(0.7)   # flat below the table
    assert ec.motor_efficiency(0.25, ((0.0, 0.5), (0.7, 0.9))) == pytest.approx(0.8)   # points under 1 W apart
    msg_pts = "the motor map needs at least two (power, efficiency) points of equal length"
    for tab_bad, msg in [(((1e4,), (0.9,)), msg_pts), (((0.0, 1e4), (0.9,)), msg_pts),
                         (([[0.0, 1e4]], [[0.8, 0.9]]), msg_pts),
                         (((1e4, 1e4), (0.8, 0.9)), "motor map power must rise strictly"),
                         (((0.0, 1e4), (0.0, 0.9)), "motor map efficiencies must lie in (0, 1]"),
                         (((0.0, 1e4), (0.8, 1.01)), "motor map efficiencies must lie in (0, 1]")]:
        with pytest.raises(ValueError) as err:
            ec.motor_efficiency(1e3, tab_bad)
        assert str(err.value) == msg


MAP = dict(rpm=(0.0, 1000.0, 3000.0), torque_nm=(0.0, 50.0, 100.0),
           efficiency=((0.80, 0.85, 0.88), (0.82, 0.90, 0.92), (0.84, 0.93, 0.95)))


def _bilin(n, t, rpm=MAP["rpm"], tq=MAP["torque_nm"], e=MAP["efficiency"]):
    """Bilinear interpolation written out cell by cell, held flat at the table's edges."""
    n = min(max(n, rpm[0]), rpm[-1])
    t = min(max(t, tq[0]), tq[-1])
    i = max(k for k in range(len(rpm) - 1) if rpm[k] <= n)
    j = max(k for k in range(len(tq) - 1) if tq[k] <= t)
    fx = (n - rpm[i]) / (rpm[i + 1] - rpm[i])
    fy = (t - tq[j]) / (tq[j + 1] - tq[j])
    return (e[i][j] * (1 - fx) * (1 - fy) + e[i + 1][j] * fx * (1 - fy) + e[i][j + 1] * (1 - fx) * fy
            + e[i + 1][j + 1] * fx * fy)


def test_a_speed_and_torque_motor_map():
    m = ec.MotorEfficiencyMap(**MAP, n_motors=4, gear_ratio=2.0, wheel_radius_m=0.25)
    v = np.r_[0.0, 5.0, 10.0, 20.0, 40.0, -10.0]
    p = np.r_[0.0, 8e3, 3e4, 2e3, 9e4, 3e4]
    want = []
    for vi, pi in zip(v, p):
        w = abs(vi) / 0.25 * 2.0
        want.append(_bilin(w * 30 / math.pi, abs(pi) / (4 * max(w, 1e-9))))
    assert m.efficiency_at(p, v) == pytest.approx(want, rel=1e-12)
    assert m.efficiency_at(-3e4, 10.0) == pytest.approx(m.efficiency_at(3e4, 10.0))   # by the size of the power
    one = ec.MotorEfficiencyMap(**MAP)
    assert (one.n_motors, one.gear_ratio, one.wheel_radius_m) == (1, 1.0, 0.225)
    w1 = 10.0 / 0.225
    assert one.efficiency_at(2e3, 10.0) == pytest.approx(_bilin(w1 * 30 / math.pi, 2e3 / w1), rel=1e-12)
    assert one.efficiency_at(1e3, 0.0) == pytest.approx(0.88)          # standing: the torque runs off the table
    assert one.efficiency_at(0.0, 0.0) == pytest.approx(0.80)
    late = ec.MotorEfficiencyMap((1000.0, 3000.0), (20.0, 60.0), ((0.7, 0.8), (0.9, 0.95)))
    assert late.efficiency_at(1.0, 0.1) == pytest.approx(0.7)          # below both tables: flat at the first entry
    tiny = ec.MotorEfficiencyMap((0.0, 0.5), (0.0, 0.5), ((0.5, 0.6), (0.7, 0.8)))
    w = 0.25 * math.pi / 30
    assert tiny.efficiency_at(0.25 * w, 0.225 * w) == pytest.approx(0.65)   # grid steps under 1 still interpolate
    msg = ("the motor map needs at least two speeds and two torques, and one efficiency per pair (rows: speed, "
           "columns: torque)")
    for kw in [dict(rpm=(0.0,), torque_nm=(0.0, 1.0), efficiency=((0.9, 0.9),)),
               dict(rpm=(0.0, 1.0), torque_nm=(0.0,), efficiency=((0.9,), (0.9,))),
               dict(rpm=(0.0, 1.0), torque_nm=(0.0, 1.0), efficiency=((0.9, 0.9),)),
               dict(rpm=((0.0, 1.0),), torque_nm=(0.0, 1.0), efficiency=((0.9, 0.9), (0.9, 0.9))),
               dict(rpm=(0.0, 1.0), torque_nm=((0.0, 1.0),), efficiency=((0.9, 0.9), (0.9, 0.9)))]:
        with pytest.raises(ValueError) as err:
            ec.MotorEfficiencyMap(**kw)
        assert str(err.value) == msg
    good = dict(rpm=(0.0, 1.0), torque_nm=(0.0, 1.0), efficiency=((0.9, 0.9), (0.9, 1.0)))
    ec.MotorEfficiencyMap(**good)                                    # 1.0 is allowed
    for kw, m2 in [(dict(good, rpm=(1.0, 1.0)), "motor map speeds and torques must rise strictly"),
                   (dict(good, torque_nm=(1.0, 1.0)), "motor map speeds and torques must rise strictly"),
                   (dict(good, efficiency=((0.0, 0.9), (0.9, 0.9))), "motor map efficiencies must lie in (0, 1]"),
                   (dict(good, efficiency=((1.01, 0.9), (0.9, 0.9))), "motor map efficiencies must lie in (0, 1]"),
                   (dict(good, n_motors=0), "motor count, gear ratio and wheel radius must be positive"),
                   (dict(good, gear_ratio=0.0), "motor count, gear ratio and wheel radius must be positive"),
                   (dict(good, wheel_radius_m=0.0), "motor count, gear ratio and wheel radius must be positive")]:
        with pytest.raises(ValueError) as err:
            ec.MotorEfficiencyMap(**kw)
        assert str(err.value) == m2
    ec.MotorEfficiencyMap(**good, n_motors=1, gear_ratio=1e-3, wheel_radius_m=1e-3)   # small but positive


TIR = """[MODEL]
LONGVL                   = 16.9              $Nominal speed
FNOMIN                   = 6000              $Nominal wheel load
QSY1                     = 0.00702
QSY3                     = 0.001515
QSY4                     = 8.514E-5
QSY7                     = 0.9008
PKY1                     = -15.314
  PKY2 = 1.7044
PKY4                     = 1.9896
"""


def test_tire_losses_from_a_tir_file():
    t = ec.TireLosses.from_tir(TIR)
    assert (t.qsy1, t.qsy3, t.qsy4, t.qsy7, t.fz_nom_n, t.v_ref_ms, t.pky1, t.pky2, t.pky4, t.n_tires) == (
        0.00702, 0.001515, 8.514e-5, 0.9008, 6000.0, 16.9, -15.314, 1.7044, 1.9896, 4)
    d = ec.TireLosses.from_tir("QSY1 = 0.02\n", n_tires=2)
    assert (d.qsy1, d.qsy3, d.qsy4, d.qsy7, d.fz_nom_n, d.v_ref_ms, d.pky1, d.pky2, d.pky4, d.n_tires) == (
        0.02, 0.0, 0.0, 1.0, 1000.0, 16.7, -15.0, 1.7, 2.0, 2)
    assert ec.TireLosses.from_tir("XQSY1 = 0.5\nQSY1X = 0.6\n").qsy1 == 0.01      # whole keys only
    fz, v = np.r_[4000.0, 4800.0, -10.0], np.r_[10.0, -25.0, 5.0]
    f = np.maximum(fz, 0.0) / 4
    u = v / 16.9
    want = 4 * f * (0.00702 + 0.001515 * np.abs(u) + 8.514e-5 * u ** 4) * (np.maximum(f, 1e-9) / 6000) ** (0.9008 - 1)
    assert t.rolling_force_n(fz, v) == pytest.approx(want, rel=1e-12) and want[2] == 0.0
    assert t.rolling_force_n(2.0, 10.0) == pytest.approx(                      # a light load is not floored
        2.0 * (0.00702 + 0.001515 * 10 / 16.9 + 8.514e-5 * (10 / 16.9) ** 4) * (0.5 / 6000) ** (0.9008 - 1))
    assert ec.TireLosses.from_tir(TIR.lower()) == ec.TireLosses()              # keys are capitals
    assert t.cornering_stiffness_n_rad(fz) == pytest.approx(
        4 * 15.314 * 6000 * np.sin(1.9896 * np.arctan(f / (1.7044 * 6000))), rel=1e-12)
    assert t.cornering_drag_n(np.r_[2000.0, -3000.0, 0.0], fz[[0, 1, 0]]) == pytest.approx(
        np.r_[2000.0, 3000.0, 0.0] ** 2 / t.cornering_stiffness_n_rad(fz[[0, 1, 0]]), rel=1e-12)
    assert t.cornering_drag_n(100.0, 0.0) == pytest.approx(100.0 ** 2 / 1e-9)   # no load: the floor, not 1/0
    plain = ec.TireLosses()
    assert plain.rolling_force_n(4000.0, 30.0) == pytest.approx(0.01 * 4000.0)    # the defaults: Crr 0.01
    for kw in [dict(fz_nom_n=0.0), dict(v_ref_ms=0.0), dict(n_tires=0), dict(pky2=0.0)]:
        with pytest.raises(ValueError) as err:
            ec.TireLosses(**kw)
        assert str(err.value) == "nominal load, reference speed, PKY2 and the tire count must be positive"
    ec.TireLosses(fz_nom_n=1e-3, v_ref_ms=1e-3, n_tires=1, pky2=1e-3)


def test_traction_power_with_downforce_tires_cornering_and_a_motor_map():
    v, a = np.r_[8.0, 15.0, 22.0, 18.0], np.r_[2.0, 0.3, -2.5, 0.0]
    alat = np.r_[3.0, -9.0, 5.0, 12.0]
    tire = ec.TireLosses.from_tir(TIR)
    mm = ec.MotorEfficiencyMap(**MAP, n_motors=4, gear_ratio=2.0, wheel_radius_m=0.25)
    m, rho = 300.0, 1.2
    F, pw, pp = ec.traction_power_w(v, a, vehicle_mass_kg=m, drag_cda=1.1, crr=0.02, rho_air=rho, drivetrain_eff=0.95,
                                    rot_mass_factor=1.3, aux_power_w=50.0, cla_m2=3.0, tire=tire, a_lat=alat,
                                    motor_eff_map=mm, regen_eff=0.5)
    fz = m * 9.81 + 0.5 * rho * 3.0 * v ** 2
    f = 1.3 * m * a + 0.5 * rho * 1.1 * v ** 2 + tire.rolling_force_n(fz, v) + (m * alat) ** 2 / \
        tire.cornering_stiffness_n_rad(fz)
    shaft = np.maximum(f * v, 0.0) / 0.95
    assert F == pytest.approx(f, rel=1e-12) and pw == pytest.approx(f * v, rel=1e-12)
    assert pp == pytest.approx(shaft / mm.efficiency_at(shaft, v) + 0.5 * np.minimum(f * v, 0.0) + 50.0, rel=1e-12)
    # downforce alone: Crr on weight plus downforce
    F2, _, _ = ec.traction_power_w(v, a, vehicle_mass_kg=m, drag_cda=1.1, crr=0.02, rho_air=rho, cla_m2=3.0)
    assert F2 == pytest.approx(m * a + 0.5 * rho * 1.1 * v ** 2 + 0.02 * fz, rel=1e-12)
    # the tire without a_lat: rolling resistance only
    F3, _, _ = ec.traction_power_w(v, a, vehicle_mass_kg=m, drag_cda=1.1, rho_air=rho, tire=tire)
    assert F3 == pytest.approx(m * a + 0.5 * rho * 1.1 * v ** 2 + tire.rolling_force_n(m * 9.81, v), rel=1e-12)
    with pytest.raises(ValueError) as err:
        ec.traction_power_w(v, a, a_lat=alat)
    assert str(err.value) == "cornering drag needs the tires' cornering stiffness: pass tire=TireLosses(...)"


def test_toe_scrub_costs_c_alpha_tan_sin_per_tire():
    t = ec.TireLosses.from_tir(TIR)
    fz = np.r_[3000.0, 4500.0]
    toe = np.radians([0.3, -0.3, 2.0, 2.1])
    c1 = t.cornering_stiffness_n_rad(fz) / 4
    want = c1 * float(np.sum(np.tan(toe) * np.sin(toe)))
    assert t.toe_drag_n(toe, fz) == pytest.approx(want, rel=1e-12)
    assert t.toe_drag_n(-toe, fz) == pytest.approx(want, rel=1e-12)        # toe-in or toe-out: the same scrub
    assert t.toe_drag_n(np.zeros(4), fz) == pytest.approx([0.0, 0.0])
    two = ec.TireLosses(n_tires=2)
    assert two.toe_drag_n((0.01, 0.02), 2000.0) == pytest.approx(
        two.cornering_stiffness_n_rad(2000.0) / 2 * (math.tan(0.01) * math.sin(0.01) + math.tan(0.02) * math.sin(0.02)))
    for bad in ((0.01, 0.01, 0.01), ((0.01, 0.01), (0.01, 0.01))):
        with pytest.raises(ValueError) as err:
            t.toe_drag_n(bad, 3000.0)
        assert str(err.value) == "give one toe angle per tire (4)"
    v, a = np.r_[10.0, 20.0], np.r_[0.0, 0.5]
    F0, _, _ = ec.traction_power_w(v, a, vehicle_mass_kg=280.0, tire=t, cla_m2=3.0)
    F1, pw, _ = ec.traction_power_w(v, a, vehicle_mass_kg=280.0, tire=t, cla_m2=3.0, toe_rad=toe)
    fzv = 280.0 * 9.81 + 0.5 * 1.225 * 3.0 * v ** 2
    assert F1 - F0 == pytest.approx(t.toe_drag_n(toe, fzv), rel=1e-12) and pw == pytest.approx(F1 * v, rel=1e-12)
    with pytest.raises(ValueError) as err:
        ec.traction_power_w(v, a, toe_rad=toe)
    assert str(err.value) == "toe scrub needs the tires' cornering stiffness: pass tire=TireLosses(...)"


def _pack_oracle(p, t, soc_pts, v_pts, soc0, ah, r):
    soc, out_v, out_i, i_prev = soc0, [], [], 0.0
    for k in range(len(p)):
        dt = t[k] - t[k - 1] if k else 0.0
        soc -= i_prev * dt / (3600.0 * ah)
        ocv = float(np.interp(soc, soc_pts, v_pts))
        disc = max(ocv * ocv - 4 * r * p[k], 0.0)
        i = (ocv - math.sqrt(disc)) / (2 * r)
        out_v.append(ocv - i * r)
        out_i.append(i)
        i_prev = i
    return np.array(out_v), np.array(out_i), soc


def test_a_pack_whose_voltage_falls_as_it_discharges():
    t = np.arange(0.0, 600.0, 2.0)
    p = 20e3 + 15e3 * np.sin(t / 30.0) - np.where(t > 400, 30e3, 0.0)      # regenerating at the end
    tab = ((0.0, 0.5, 1.0), (250.0, 280.0, 300.0))
    v, i, soc = ec.pack_along_lap(p, t, ocv_soc=tab, soc0=0.8, capacity_ah=5.0, r_pack_ohm=0.1)
    wv, wi, wsoc = _pack_oracle(p, t, *tab, 0.8, 5.0, 0.1)
    assert v == pytest.approx(wv, rel=1e-12) and i == pytest.approx(wi, rel=1e-12)
    assert soc[-1] == pytest.approx(wsoc, rel=1e-12) and soc[0] == 0.8
    assert soc[300 // 2] < soc[0] and soc[-1] > soc[220]                   # drains, then the regen puts some back
    flat, fi, _ = ec.pack_along_lap(p, t, ocv_soc=((0, 1), (280.0, 280.0)), soc0=0.8, capacity_ah=5.0, r_pack_ohm=0.1)
    _, ci = ec.battery_internal_power_w(p, 280.0, 0.1)
    assert fi == pytest.approx(ci, rel=1e-12) and flat == pytest.approx(280.0 - ci * 0.1, rel=1e-12)
    one, oi, os_ = ec.pack_along_lap(np.r_[1e4], np.r_[5.0], ocv_soc=tab, soc0=0.5, capacity_ah=1.0, r_pack_ohm=0.1)
    assert os_[0] == 0.5 and one[0] == pytest.approx(280.0 - oi[0] * 0.1)
    _, si, small = ec.pack_along_lap(np.r_[1e3, 1e3], np.r_[0.0, 0.5], ocv_soc=((0.0, 0.5), (200.0, 300.0)),
                                     soc0=0.25, capacity_ah=1e-3, r_pack_ohm=0.1)
    assert small[1] == pytest.approx(0.25 - si[0] * 0.5 / 3.6, rel=1e-12)    # Ah: I dt / 3600
    for kw, msg in [(dict(capacity_ah=0.0), "capacity must be positive"),
                    (dict(ocv_soc=((0.0,), (250.0,))), "the OCV table needs at least two (state of charge, V) points, "
                                                       "charge rising strictly"),
                    (dict(ocv_soc=((0.0, 1.0), (250.0,))), "the OCV table needs at least two (state of charge, V) "
                                                          "points, charge rising strictly"),
                    (dict(ocv_soc=((0.5, 0.5), (250.0, 260.0))), "the OCV table needs at least two (state of charge, "
                                                                "V) points, charge rising strictly")]:
        args = dict(ocv_soc=tab, soc0=0.8, capacity_ah=5.0, r_pack_ohm=0.1) | kw
        with pytest.raises(ValueError) as err:
            ec.pack_along_lap(p, t, **args)
        assert str(err.value) == msg
    with pytest.raises(ValueError) as err:
        ec.pack_along_lap(p, t[:-1], ocv_soc=tab, soc0=0.8, capacity_ah=5.0, r_pack_ohm=0.1)
    assert str(err.value) == "one power per time point"
    with pytest.raises(ValueError) as err:
        ec.pack_along_lap(np.array([]), np.array([]), ocv_soc=tab, soc0=0.8, capacity_ah=5.0, r_pack_ohm=0.1)
    assert str(err.value) == "one power per time point"
    ec.pack_along_lap(p, t, ocv_soc=tab, soc0=0.8, capacity_ah=1e-6, r_pack_ohm=0.1)      # tiny but positive


def test_a_pack_that_heats_and_whose_resistance_follows_charge_and_temperature():
    t = np.arange(0.0, 300.0, 2.0)                       # 2 s steps: the heating scales with dt, not 1/dt
    p = 25e3 + 10e3 * np.sin(t / 20.0)
    ocv = ((0.0, 0.5, 1.0), (250.0, 280.0, 300.0))
    rtab = ((0.0, 1.0), (290.0, 320.0), ((0.10, 0.20), (0.08, 0.16)))
    temps = []
    v, i, soc = ec.pack_along_lap(p, t, ocv_soc=ocv, soc0=0.8, capacity_ah=10.0, r_pack_ohm=0.5, r_soc_temp=rtab,
                                  heat_capacity_j_k=20e3, temp0_k=300.0, temps_out=temps)
    q, T, r_prev, wi, wv, wt = 0.8, 300.0, None, [], [], []
    for k in range(t.size):
        if k:
            q -= wi[-1] * 2.0 / 36000.0
            T += wi[-1] ** 2 * r_prev * 2.0 / 20e3
        fx, fy = min(max(q, 0.0), 1.0), min(max((T - 290.0) / 30.0, 0.0), 1.0)   # held flat beyond the table
        r = 0.10 * (1 - fx) * (1 - fy) + 0.08 * fx * (1 - fy) + 0.20 * (1 - fx) * fy + 0.16 * fx * fy
        o = float(np.interp(q, ocv[0], ocv[1]))
        ik = (o - math.sqrt(o * o - 4 * r * p[k])) / (2 * r)
        wi.append(ik)
        wv.append(o - ik * r)
        wt.append(T)
        r_prev = r
    assert i == pytest.approx(wi, rel=1e-12) and v == pytest.approx(wv, rel=1e-12) and temps == pytest.approx(wt)
    assert temps[0] == 300.0 and temps[-1] > 301.0
    cold = ec.pack_along_lap(p, t, ocv_soc=ocv, soc0=0.8, capacity_ah=10.0, r_pack_ohm=0.5, r_soc_temp=rtab)
    assert cold[0][-1] > v[-1]                          # no heating: 298.15 K all lap, a lower resistance
    t2 = []
    ec.pack_along_lap(p[:3], t[:3], ocv_soc=ocv, soc0=0.8, capacity_ah=10.0, r_pack_ohm=0.5, temps_out=t2)
    assert t2 == [298.15] * 3                           # no heat capacity: the temperature does not move
    flat = ec.pack_along_lap(p, t, ocv_soc=ocv, soc0=0.8, capacity_ah=10.0, r_pack_ohm=0.12, heat_capacity_j_k=1.0)
    plain = ec.pack_along_lap(p, t, ocv_soc=ocv, soc0=0.8, capacity_ah=10.0, r_pack_ohm=0.12)
    assert flat[1] == pytest.approx(plain[1], rel=1e-12)   # heating alone changes nothing without a table
    msg = "the resistance table needs at least two charges and two temperatures, one value each"
    for bad in [((0.0,), (290.0, 320.0), ((0.1, 0.2),)), ((0.0, 1.0), (290.0,), ((0.1,), (0.1,))),
                ((0.0, 1.0), (290.0, 320.0), ((0.1, 0.2),))]:
        with pytest.raises(ValueError) as err:
            ec.pack_along_lap(p, t, ocv_soc=ocv, soc0=0.8, capacity_ah=10.0, r_pack_ohm=0.5, r_soc_temp=bad)
        assert str(err.value) == msg
    for c in (0.0, -1.0):
        with pytest.raises(ValueError) as err:
            ec.pack_along_lap(p, t, ocv_soc=ocv, soc0=0.8, capacity_ah=10.0, r_pack_ohm=0.5, heat_capacity_j_k=c)
        assert str(err.value) == "heat capacity must be positive"
    ec.pack_along_lap(p[:2], t[:2], ocv_soc=ocv, soc0=0.8, capacity_ah=10.0, r_pack_ohm=0.5, heat_capacity_j_k=1e-3)


def test_a_lumped_pack_heats_by_i2r_and_cools_to_ambient():
    t = np.r_[0.0, 2.0, 3.0, 7.0, 7.5]
    i = np.r_[100.0, 50.0, 0.0, 200.0, 10.0]
    T = ec.pack_temperature_along(i, t, r_pack_ohm=0.1, heat_capacity_j_k=1e4, temp0_k=300.0, cooling_w_k=3.0,
                                  ambient_k=295.0)
    want = [300.0]
    for k in range(1, 5):
        want.append(want[-1] + (i[k - 1] ** 2 * 0.1 - 3.0 * (want[-1] - 295.0)) * (t[k] - t[k - 1]) / 1e4)
    assert T == pytest.approx(want, rel=1e-12)
    adi = ec.pack_temperature_along(i, t, r_pack_ohm=0.1, heat_capacity_j_k=1e4, temp0_k=300.0)
    assert adi[-1] == pytest.approx(300.0 + float(np.sum(i[:-1] ** 2 * 0.1 * np.diff(t))) / 1e4, rel=1e-12)
    assert list(ec.pack_temperature_along(np.r_[5.0], np.r_[1.0], r_pack_ohm=0.1, heat_capacity_j_k=1.0,
                                          temp0_k=290.0)) == [290.0]
    for kw, msg in [(dict(heat_capacity_j_k=0.0), "heat capacity must be positive and cooling not negative"),
                    (dict(cooling_w_k=-1.0, ambient_k=290.0), "heat capacity must be positive and cooling not negative"),
                    (dict(cooling_w_k=1.0), "cooling needs an ambient temperature")]:
        args = dict(r_pack_ohm=0.1, heat_capacity_j_k=1e4, temp0_k=300.0) | kw
        with pytest.raises(ValueError) as err:
            ec.pack_temperature_along(i, t, **args)
        assert str(err.value) == msg
    for bad_i, bad_t in [(i, t[:-1]), (np.array([]), np.array([]))]:
        with pytest.raises(ValueError) as err:
            ec.pack_temperature_along(bad_i, bad_t, r_pack_ohm=0.1, heat_capacity_j_k=1e4, temp0_k=300.0)
        assert str(err.value) == "one current per time point"
    ec.pack_temperature_along(i, t, r_pack_ohm=0.1, heat_capacity_j_k=1e-6, temp0_k=300.0, cooling_w_k=0.0)


# ---- untrusted inputs: each guard fires at its exact boundary and says exactly what is wrong ----
def _says(exc_info, text):
    assert str(exc_info.value) == text


def test_tir_tire_count_boundaries_and_message():
    assert ec.TireLosses.from_tir("", n_tires=1).n_tires == 1
    assert ec.TireLosses.from_tir("", n_tires=2).n_tires == 2
    for bad in (0, True, 2.0):
        with pytest.raises(ValueError) as e:
            ec.TireLosses.from_tir("", n_tires=bad)
        _says(e, f"n_tires must be a whole number of tires, got {bad!r}")


def test_tir_value_out_of_float_range_is_refused():
    with pytest.raises(ValueError) as e:
        ec.TireLosses.from_tir("FNOMIN = 1e999\n")
    _says(e, ".tir FNOMIN: inf is not a finite number")


_P = ec.ElecParams()
_T = [0.0, 1.0, 2.0]
_V = [10.0, 20.0, 30.0]


def test_speed_trace_guards_say_exactly_what_is_wrong():
    cases = [
        (([10.0, 20.0], _T), {}, "the speed trace needs one speed per time, got 2 and 3"),
        ((_V, [0.0, 1.0, 1.0]), {}, "speed trace time must rise from one sample to the next"),
        ((_V, [0.0, 2.0, 1.0]), {}, "speed trace time must rise from one sample to the next"),
        ((_V, [0.0, float("nan"), 2.0]), {}, "speed trace time (s): 1 of 3 value(s) are not finite numbers (nan or inf)"),
        (([10.0, float("inf"), 30.0], _T), {}, "speed trace speed (mph): 1 of 3 value(s) are not finite numbers (nan or inf)"),
        (([10.0, 1000.5, 30.0], _T), {}, "a speed above 1000 mph in the trace: check the column and its unit"),
        ((_V, _T), {"vehicle_mass_kg": 0.0}, "vehicle mass (kg): must be above zero, got 0"),
        ((_V, _T), {"drivetrain_eff": float("nan")}, "drivetrain efficiency: nan is not a finite number"),
        ((_V, _T), {"drag_cda": float("nan")}, "drag CdA (m²): nan is not a finite number"),
        ((_V, _T), {"crr": float("inf")}, "rolling resistance: inf is not a finite number"),
    ]
    for (speed, time), kw, msg in cases:
        with pytest.raises(ValueError) as e:
            ec.check_lap_from_speed_csv(speed, time, _P, **kw)
        _says(e, msg)


def test_speed_trace_edges_that_are_accepted():
    assert ec.check_lap_from_speed_csv([10.0, 1000.0, 30.0], _T, _P) is not None     # exactly 1000 mph passes
    one = ec.check_lap_from_speed_csv([10.0], [0.0], _P)                            # one sample is too little data,
    assert not one.ok                                                               # reported, not refused


def test_a_non_finite_electrical_result_is_refused(monkeypatch):
    good = ec.check_lap_from_speed_csv(_V, _T, _P)
    bad = ec.check_lap_from_speed_csv(_V, _T, _P)
    bad.energy_used_kwh = float("nan")
    monkeypatch.setattr(ec, "check_lap_electrical", lambda *a, **k: bad)
    with pytest.raises(ValueError) as e:
        ec.check_lap_from_speed_csv(_V, _T, _P)
    _says(e, "the electrical check gave a result that is not a finite number: one of the pack, motor "
             "or vehicle parameters is nan, infinite or far out of range")
    monkeypatch.setattr(ec, "check_lap_electrical", lambda *a, **k: good)
    assert ec.check_lap_from_speed_csv(_V, _T, _P) is good
