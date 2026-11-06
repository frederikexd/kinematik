"""The powertrain benchmark kit (docs/POWERTRAIN_BENCHMARK_PLAN.md): KinematiK's side runs on the shared inputs, a
reference file is graded against the limits, and MoTeC i2 exports are read and compared metric by metric. The MoTeC
tests write logs whose physics is known exactly, so every metric must come back at the value that was put in."""
import csv
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location("powertrain_benchmark", ROOT / "scripts" / "powertrain_benchmark.py")
pb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pb)
INP = json.loads((pb.BENCH / "inputs.json").read_text())


def test_every_case_runs_on_the_shared_inputs():
    res = {(c, m): v for c, m, v in pb.run()}
    assert len(res) == 5 and all(math.isfinite(v) and v > 0 for v in res.values())
    assert math.isclose(res[("ice_restrictor", "choked_flow_kg_s")],
                        0.95 * pb.ice.choked_mass_flow_kg_s(20.0), rel_tol=1e-12)
    assert 3.0 < res[("ev_75m", "time_s")] < 6.0 and 3.0 < res[("ice_75m", "time_s")] < 6.0


def test_compare_passes_within_the_limit_and_fails_outside_it(tmp_path):
    ours = {(c, m): v for c, m, v in pb.run()}
    f = tmp_path / "ref.csv"
    with open(f, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["tool", "case", "metric", "value"])
        w.writerow(["Simulink", "ev_75m", "time_s", ours[("ev_75m", "time_s")] * 1.015])
        w.writerow(["Amesim", "ice_75m", "time_s", ""])                     # not run yet: skipped, not failed
    assert pb.compare(str(f)) == 0
    with open(f, "a", newline="") as fh:
        csv.writer(fh).writerow(["Amesim", "ice_75m", "time_s", ours[("ice_75m", "time_s")] * 1.05])
    assert pb.compare(str(f)) == 1


def _motec(path, cols: dict, units: dict):
    """A file laid out as i2 Pro's CSV export: a metadata block, the channel names, their units, then the data."""
    names = list(cols)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh, quoting=csv.QUOTE_ALL)
        w.writerow(["Format", "MoTeC CSV File"])
        w.writerow(["Venue", "benchmark"])
        w.writerow([])
        w.writerow(names)
        w.writerow([units[n] for n in names])
        w.writerow([])
        for k in range(len(cols[names[0]])):
            w.writerow([f"{cols[n][k]:.6f}" for n in names])
    return str(path)


def _speed_trace(n=4000, dt=0.02):
    t = np.arange(n) * dt
    v = 14.0 + 6.0 * np.sin(t / 3.0) + 2.0 * np.sin(t / 0.9)          # an endurance-like 8-22 m/s trace
    return t, v


def test_the_reader_takes_i2_layout_units_and_overrides(tmp_path):
    t = np.arange(50) * 0.1
    p = _motec(tmp_path / "a.csv", {"Time": t, "Wheel Speed RL": np.full(50, 36.0), "Odometer": t * 0.01},
               {"Time": "s", "Wheel Speed RL": "km/h", "Odometer": "km"})
    log = pb.read_motec_csv(p, {"speed": "Wheel Speed RL"})
    assert np.allclose(log["speed"], 10.0) and math.isclose(log["distance"][-1], 49.0)
    p2 = _motec(tmp_path / "b.csv", {"Time": t, "GPS Speed": np.full(50, 10.0)}, {"Time": "s", "GPS Speed": "m/s"})
    assert math.isclose(pb.read_motec_csv(p2)["distance"][-1], 49.0)                  # integrated when not logged


def test_ev_metrics_come_back_at_the_values_put_in(tmp_path):
    ev, veh = INP["ev"], INP["vehicle"]
    # acceleration: KinematiK's own 75 m trace, logged
    car = pb._point_mass_car(veh, 1.0 - ev["rear_static_share"])
    acc = pb.lt.acceleration_time(car, pb._ev_powertrain(veh, ev))
    s, v = np.asarray(acc.s, float), np.asarray(acc.v, float)
    t = np.concatenate([[0.0], np.cumsum(np.diff(s) / np.maximum(0.5 * (v[1:] + v[:-1]), 1e-3))])
    t, s, v = np.append(t, t[-1] + 0.5), np.append(s, s[-1] + 0.5 * v[-1]), np.append(v, v[-1])   # a log runs on
    tt = np.arange(0.0, t[-1], 0.01)
    accel = _motec(tmp_path / "acc.csv", {"Time": tt, "Ground Speed": np.interp(tt, t, v) * 3.6,
                                          "Distance": np.interp(tt, t, s)},
                   {"Time": "s", "Ground Speed": "km/h", "Distance": "m"})
    # drive: a pack of the declared resistance feeding exactly the model's pack power on a speed trace
    t, v = _speed_trace()
    eta = ev["driveline_eff"] * ev["motor_eff"] * ev["inverter_eff"]
    from suspension import ev_electrical_check as ec
    _, _, P = ec.traction_power_w(v, pb._smooth_accel(t, v), vehicle_mass_kg=veh["mass_kg"], drag_cda=veh["cda_m2"],
                                  crr=veh["crr"], rho_air=veh["rho_kg_m3"], drivetrain_eff=eta)
    pk = ev["pack"]
    R, Voc = pk["series"] * pk["cell_ohm"] / pk["parallel"], pk["series"] * 3.9
    I = (Voc - np.sqrt(Voc * Voc - 4 * R * P)) / (2 * R)
    motor = v / veh["tire_radius_m"] * ev["final_drive"] * 60 / (2 * math.pi)
    drive = _motec(tmp_path / "drive.csv", {"Time": t, "Ground Speed": v * 3.6, "Pack Voltage": Voc - I * R,
                                            "Pack Current": I, "Motor Speed": motor},
                   {"Time": "s", "Ground Speed": "km/h", "Pack Voltage": "V", "Pack Current": "A", "Motor Speed": "rpm"})
    assert pb.motec("ev", str(pb.BENCH / "inputs.json"), accel, drive) == 0
    # a car that is not the one declared fails: the same drive log against a pack of twice the resistance
    bad = json.loads(json.dumps(INP))
    bad["ev"]["pack"]["cell_ohm"] *= 2
    for k in ("motor_curve_csv", "engine_curve_csv"):
        bad["ev" if k.startswith("motor") else "ice"][k] = str(pb.BENCH / INP["ev" if k.startswith("motor") else "ice"][k])
    f = tmp_path / "bad.json"
    f.write_text(json.dumps(bad))
    assert pb.motec("ev", str(f), None, drive) == 1


def test_ice_metrics_come_back_at_the_values_put_in(tmp_path):
    ic, veh = INP["ice"], INP["vehicle"]
    eng, gb, iv = pb._ice_parts(veh, ic)
    la = pb.ice.simulate_launch(eng, gb, iv, fuel_key=ic["fuel"], bte=ic["bte"], launch_rpm=ic["launch_rpm"])
    run_on = lambda a, b: np.append(a, b)  # noqa: E731  (a log runs on past 75 m)
    accel = _motec(tmp_path / "acc.csv", {"Time": run_on(la.t_s, la.t_s[-1] + 0.5),
                                          "Ground Speed": run_on(la.v_ms, la.v_ms[-1]) * 3.6,
                                          "Distance": run_on(la.x_m, la.x_m[-1] + 0.5 * la.v_ms[-1]),
                                          "Engine RPM": run_on(la.rpm, la.rpm[-1])},
                   {"Time": "s", "Ground Speed": "km/h", "Distance": "m", "Engine RPM": "rpm"})
    # drive: engine rpm in the declared gears; fuel used from the declared BTE on the same trace
    t, v = _speed_trace()
    overall = [gb.primary * g * gb.final_drive for g in gb.ratios]
    gear = np.clip(np.searchsorted([8.0, 12.0, 15.5, 18.5, 21.0], v), 0, 5)
    rpm = v / veh["tire_radius_m"] * np.array(overall)[gear] * 60 / (2 * math.pi)
    f = pb.ice.fuel(ic["fuel"])
    p_eng = np.maximum((veh["mass_kg"] * pb._smooth_accel(t, v) + pb._road_load(veh, v)) * v, 0.0) / gb.efficiency
    e = np.concatenate([[0.0], np.cumsum(0.5 * (p_eng[1:] + p_eng[:-1]) * np.diff(t))])
    fuel_l = e / (ic["bte"] * f.lhv_mj_kg * 1e6) / f.density_kg_l
    drive = _motec(tmp_path / "drive.csv", {"Time": t, "Ground Speed": v * 3.6, "Engine RPM": rpm, "Fuel Used": fuel_l},
                   {"Time": "s", "Ground Speed": "km/h", "Engine RPM": "rpm", "Fuel Used": "l"})
    assert pb.motec("ice", str(pb.BENCH / "inputs.json"), accel, drive) == 0


def _simulink_lap(tmp_path, scale_r=1.0, energy_scale=1.0):
    """A lap written as simulink_lap.m writes it: a closed circle driven on a speed trace, with a pack of the declared
    open-circuit voltage and resistance feeding exactly KinematiK's pack power (times energy_scale)."""
    ev, veh = INP["ev"], INP["vehicle"]
    from suspension import ev_electrical_check as ec
    t = np.arange(0.0, 60.0, 0.01)
    v = 15.0 + 5.0 * np.sin(2 * np.pi * t / 30.0)
    d = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))])
    th = 2 * np.pi * d / d[-1]
    x, y = d[-1] / (2 * np.pi) * np.cos(th), d[-1] / (2 * np.pi) * np.sin(th)
    eta = ev["driveline_eff"] * ev["motor_eff"] * ev["inverter_eff"]
    _, _, P = ec.traction_power_w(v, pb._smooth_accel(t, v), vehicle_mass_kg=veh["mass_kg"], drag_cda=veh["cda_m2"],
                                  crr=veh["crr"], rho_air=veh["rho_kg_m3"], drivetrain_eff=eta,
                                  rot_mass_factor=pb.ROT_FACTOR_EV)
    P = P * energy_scale
    pk = ev["pack"]
    R, Voc = scale_r * pk["series"] * pk["cell_ohm"] / pk["parallel"], pk["series"] * pk["cell_v"]
    I = (Voc - np.sqrt(Voc * Voc - 4 * R * P)) / (2 * R)
    I = np.where(P < 0, P / Voc, I)                       # regenerating: a little charge current
    motor = v / veh["tire_radius_m"] * ev["final_drive"] * 60 / (2 * math.pi)
    return _motec(tmp_path / "simulink_lap.csv",
                  {"Time": t, "Ground Speed": v, "Distance": d, "X": x, "Y": y, "Pack Voltage": Voc - I * R,
                   "Pack Current": I, "Motor Speed": motor},
                  {"Time": "s", "Ground Speed": "m/s", "Distance": "m", "X": "m", "Y": "m", "Pack Voltage": "V",
                   "Pack Current": "A", "Motor Speed": "rpm"})


def test_a_simulink_lap_comes_back_at_the_values_put_in(tmp_path, capsys):
    inputs = str(pb.BENCH / "inputs.json")
    lap = _simulink_lap(tmp_path)
    r = pb.simulink_lap_rows(inputs, lap)
    got = {m: (k, s) for m, k, s in r["rows"]}
    assert set(got) == set(pb.SIMULINK_LAP_LIMIT)
    for m in ("energy_kwh_per_km", "peak_current_a", "rms_current_a", "min_pack_voltage_v"):
        k, s = got[m]
        assert abs(k / s - 1) < 2e-3, (m, k, s)
    assert abs(got["pack_resistance_ohm"][1] / got["pack_resistance_ohm"][0] - 1) < 0.02
    assert math.isclose(r["lap_time_simulink_s"], 59.99, rel_tol=1e-9) and r["lap_ok"]
    assert abs(r["ratio_measured"] / INP["ev"]["final_drive"] - 1) < 1e-3
    assert abs(r["rolling_radius_m"] / INP["vehicle"]["tire_radius_m"] - 1) < 1e-3
    assert r["model"].startswith("rotating-inertia factor 1.050, one motor efficiency")
    assert 0.0 < r["lap_time_kinematik_s"] < r["lap_time_simulink_s"]   # the limit lap beats a driven trace
    assert abs(r["track_length_m"] / r["distance_m"] - 1) < 0.02
    assert pb.simulink_lap(inputs, lap) == 0
    assert "lap time (not graded)" in capsys.readouterr().out


def test_a_simulink_lap_from_another_car_fails(tmp_path):
    inputs = str(pb.BENCH / "inputs.json")
    assert pb.simulink_lap(inputs, _simulink_lap(tmp_path, energy_scale=1.05)) == 1
    assert pb.simulink_lap(inputs, _simulink_lap(tmp_path, scale_r=2.0)) == 1


def test_a_simulink_lap_needs_the_pack_channels(tmp_path):
    t = np.arange(0.0, 5.0, 0.1)
    p = _motec(tmp_path / "no_pack.csv", {"Time": t, "Ground Speed": np.full(t.size, 10.0)},
               {"Time": "s", "Ground Speed": "m/s"})
    with pytest.raises(ValueError, match="needs a pack volts channel"):
        pb.simulink_lap_rows(str(pb.BENCH / "inputs.json"), p)


def test_the_lateral_acceleration_of_a_circle_is_v_squared_over_r():
    t = np.arange(0.0, 30.0, 0.01)
    v = np.full(t.size, 12.0)
    th = v * t / 40.0
    alat = pb.lateral_accel(t, v, 40.0 * np.cos(th), 40.0 * np.sin(th))
    assert np.allclose(alat[100:-100], 12.0 ** 2 / 40.0, rtol=1e-3)
    assert np.allclose(pb.lateral_accel(t, v, 40.0 * np.cos(th), -40.0 * np.sin(th))[100:-100], -3.6, rtol=1e-3)


def test_a_simulink_lap_with_the_motor_map_tires_and_inertia(tmp_path):
    """The car's own map, tires and rotating inertia in its inputs: KinematiK uses them, and a pack fed exactly
    that model's power comes back at the values put in."""
    from suspension import ev_electrical_check as ec
    inp = json.loads(json.dumps(INP))
    inp["ev"]["motor_curve_csv"] = str(pb.BENCH / INP["ev"]["motor_curve_csv"])
    inp["ev"]["motor_eff_map"] = {"rpm": [0, 5000, 20000], "torque_nm": [0, 50, 150],
                                  "efficiency": [[0.8, 0.85, 0.85], [0.85, 0.93, 0.95], [0.85, 0.92, 0.94]],
                                  "n_motors": 2}
    inp["vehicle"]["tire"] = {"qsy1": 0.008, "qsy3": 0.001, "qsy4": 1e-4, "qsy7": 0.9, "fz_nom_n": 1500.0,
                              "v_ref_ms": 16.7, "pky1": -20.0, "pky2": 1.5, "pky4": 2.0, "n_tires": 4}
    inp["vehicle"]["rot_mass_factor"] = 1.2
    f = tmp_path / "car.json"
    f.write_text(json.dumps(inp))
    ev, veh = inp["ev"], inp["vehicle"]
    t = np.arange(0.0, 60.0, 0.01)
    v = 15.0 + 5.0 * np.sin(2 * np.pi * t / 30.0)
    d = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))])
    th = 2 * np.pi * d / d[-1]
    x, y = d[-1] / (2 * np.pi) * np.cos(th), d[-1] / (2 * np.pi) * np.sin(th)
    mm = ec.MotorEfficiencyMap((0, 5000, 20000), (0, 50, 150), tuple(map(tuple, ev["motor_eff_map"]["efficiency"])),
                               2, ev["final_drive"], veh["tire_radius_m"])
    tire = ec.TireLosses(**veh["tire"])
    _, _, P = ec.traction_power_w(v, pb._smooth_accel(t, v), vehicle_mass_kg=veh["mass_kg"], drag_cda=veh["cda_m2"],
                                  crr=veh["crr"], rho_air=veh["rho_kg_m3"], rot_mass_factor=1.2,
                                  cla_m2=veh["cla_m2"], tire=tire, a_lat=pb.lateral_accel(t, v, x, y),
                                  drivetrain_eff=ev["driveline_eff"] * ev["inverter_eff"], motor_eff_map=mm)
    pk = ev["pack"]
    R, Voc = pk["series"] * pk["cell_ohm"] / pk["parallel"], pk["series"] * pk["cell_v"]
    I = (Voc - np.sqrt(Voc * Voc - 4 * R * P)) / (2 * R)
    lap = _motec(tmp_path / "lap.csv", {"Time": t, "Ground Speed": v, "Distance": d, "X": x, "Y": y,
                                        "Pack Voltage": Voc - I * R, "Pack Current": I},
                 {"Time": "s", "Ground Speed": "m/s", "Distance": "m", "X": "m", "Y": "m", "Pack Voltage": "V",
                  "Pack Current": "A"})
    r = pb.simulink_lap_rows(str(f), lap)
    assert r["model"] == ("rotating-inertia factor 1.200, motor map (3 speeds x 3 torques, 2 motors), tire rolling "
                          "resistance under weight and downforce, cornering drag")
    for m, k, s in r["rows"]:
        if m != "pack_resistance_ohm":
            assert abs(k / s - 1) < 2e-3, (m, k, s)
    plain = pb.simulink_lap_rows(str(pb.BENCH / "inputs.json"), lap)      # the same lap without them: well off
    assert {m: (k, s) for m, k, s in plain["rows"]}["energy_kwh_per_km"][0] < 0.95 * r["rows"][0][2]


HOCK = pb.BENCH / "simulink_hockenheim"


def test_the_stored_hockenheim_lap_is_pinned_and_reproduces_its_result():
    """MathWorks' Formula Student car over Hockenheim (docs/POWERTRAIN_BENCHMARK_PLAN.md): the stored run still
    matches its checksums and grades as written up, energy 3.3 % under Simulink with the car's map, tires and
    inertia, 21.7 % under without them."""
    import hashlib
    for line in (HOCK / "MANIFEST.sha256").read_text().splitlines():
        digest, name = line.split()
        assert hashlib.sha256((HOCK / name).read_bytes()).hexdigest() == digest, name
    r = pb.simulink_lap_rows(str(HOCK / "inputs.json"), str(HOCK / "simulink_lap.csv"))
    got = {m: k / s - 1 for m, k, s in r["rows"]}
    assert got["energy_kwh_per_km"] == pytest.approx(-0.033, abs=0.002)
    assert got["peak_current_a"] == pytest.approx(-0.056, abs=0.002)
    assert got["rms_current_a"] == pytest.approx(-0.036, abs=0.002)
    assert got["min_pack_voltage_v"] == pytest.approx(0.003, abs=0.002)
    assert r["rolling_radius_m"] == pytest.approx(0.2261, abs=2e-4)
    bare = json.loads((HOCK / "inputs.json").read_text())
    for k in ("tire", "rot_mass_factor"):
        bare["vehicle"].pop(k)
    bare["ev"].pop("motor_eff_map")
    bare["ev"]["motor_curve_csv"] = str(HOCK / "motor_curve.csv")
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        f = Path(d) / "bare.json"
        f.write_text(json.dumps(bare))
        b = pb.simulink_lap_rows(str(f), str(HOCK / "simulink_lap.csv"))
    assert {m: k / s - 1 for m, k, s in b["rows"]}["energy_kwh_per_km"] == pytest.approx(-0.217, abs=0.002)


def test_the_sealed_cases_are_the_ones_matlab_runs():
    """kinematik_blind_runs.m and BLIND_CASES list the same cases with the same settings."""
    import re
    m = (pb.BENCH / "matlab" / "kinematik_blind_runs.m").read_text()
    rows = re.findall(r"^\s*'(\w+)',\s*'(\w+)',\s*'([^']+)',\s*(true|false),\s*([\d.]+|NaN)\s*$", m, re.M)
    assert [(c["name"], c["vehicle_data"], c["event"], c["fs_car"], c["vGain"]) for c in pb.BLIND_CASES] == [
        (n, d, e, f == "true", None if g == "NaN" else float(g)) for n, d, e, f, g in rows]
    assert {c["kind"] for c in pb.BLIND_CASES} == {"lap", "launch"}
    assert all((c["kind"] == "launch") == (c["event"] == "WOT Braking") for c in pb.BLIND_CASES)


def _launch_log(tmp_path, scale=1.0):
    """A standing start written as simulink_lap.m writes it, from KinematiK's own 75 m run (time scaled by scale)."""
    ev, veh = INP["ev"], INP["vehicle"]
    acc = pb.lt.acceleration_time(pb._point_mass_car(veh, 1.0 - ev["rear_static_share"]), pb._ev_powertrain(veh, ev))
    s, v = np.asarray(acc.s, float), np.asarray(acc.v, float)
    t = np.concatenate([[0.0], np.cumsum(np.diff(s) / np.maximum(0.5 * (v[1:] + v[:-1]), 1e-3))]) * scale
    v = v / scale
    t, s, v = np.append(t, t[-1] + 1.0), np.append(s, s[-1] + v[-1]), np.append(v, v[-1])
    tt = np.arange(0.0, t[-1], 0.01)
    return _motec(tmp_path / "simulink_lap.csv", {"Time": tt, "Ground Speed": np.interp(tt, t, v),
                                                  "Distance": np.interp(tt, t, s)},
                  {"Time": "s", "Ground Speed": "m/s", "Distance": "m"}), acc


def test_a_launch_is_predicted_not_given(tmp_path):
    lap, acc = _launch_log(tmp_path)
    rows = {m: (k, s) for m, k, s in pb.blind_launch_rows(pb.BENCH / "inputs.json", lap)}
    k, s = rows["launch_75m_time_s"]
    assert k == pytest.approx(s, rel=2e-3) and k == pytest.approx(acc.lap_time_s, rel=0.03)   # same clock both sides
    assert rows["launch_75m_speed_ms"][0] == pytest.approx(rows["launch_75m_speed_ms"][1], rel=2e-3)
    assert rows["launch_75m_speed_ms"][0] == pytest.approx(acc.top_speed_ms, rel=2e-3)
    (tmp_path / "slow").mkdir()
    slow, _ = _launch_log(tmp_path / "slow", scale=1.1)
    rows = {m: (k, s) for m, k, s in pb.blind_launch_rows(pb.BENCH / "inputs.json", slow)}
    assert rows["launch_75m_time_s"][1] == pytest.approx(1.1 * rows["launch_75m_time_s"][0], rel=5e-3)


def test_blind_grading_refuses_changed_code_and_reports_every_case(tmp_path, monkeypatch, capsys):
    import shutil
    root = tmp_path / "repo"
    for f in pb.SEALED_CODE:
        (root / f).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(pb.ROOT / f, root / f)
    monkeypatch.setattr(pb, "BLIND", tmp_path / "blind")
    rec = pb.seal_blind(fitness=pb.FITNESS2)
    assert rec["code_sha256"] == pb.code_fingerprint() and len(rec["cases"]) == len(pb.BLIND_CASES)
    assert rec["fitness_for_use"] == pb.FITNESS2
    with pytest.raises(ValueError, match="written once"):
        pb.seal_blind()
    runs = tmp_path / "runs"
    (runs / "launch_fs_car").mkdir(parents=True)
    lap, _ = _launch_log(runs / "launch_fs_car")
    shutil.copy(pb.BENCH / "inputs.json", runs / "launch_fs_car" / "inputs.json")
    shutil.copy(pb.BENCH / "motor_curve.csv", runs / "launch_fs_car" / "motor_curve.csv")
    (runs / "hock_fs_car").mkdir()
    shutil.copy(pb.BENCH / "simulink_hockenheim" / "simulink_lap.csv", runs / "hock_fs_car" / "simulink_lap.csv")
    for f in ("inputs.json", "motor_curve.csv"):
        shutil.copy(pb.BENCH / "simulink_hockenheim" / f, runs / "hock_fs_car" / f)
    r = pb.grade_blind(runs, root=root)
    status = {c["name"]: c["status"] for c in r["cases"]}
    assert r["changed"] == [] and status["launch_fs_car"] == "graded" and status["hock_fs_car"] == "graded"
    assert status["skidpad_fs_car"] == "not run"
    launch = next(c for c in r["cases"] if c["name"] == "launch_fs_car")
    assert all(ok for *_, ok in launch["rows"])
    hock = {m: ok for m, *_, ok in next(c for c in r["cases"] if c["name"] == "hock_fs_car")["rows"]}
    assert hock["min_pack_voltage_v"] and not hock["energy_kwh_per_km"]
    fit = next(c for c in r["cases"] if c["name"] == "hock_fs_car")["fitness"]
    assert set(fit) == set(hock) and fit["min_pack_voltage_v"] == (True, False)
    monkeypatch.setattr(pb, "ROOT", root)
    pb.report_blind(runs)
    printed = capsys.readouterr().out
    assert "within the sealed fitness-for-use bars" in printed and "fit for use" in printed
    (root / "suspension/laptime.py").write_text((root / "suspension/laptime.py").read_text() + "\n# changed\n")
    assert pb.grade_blind(runs, root=root)["changed"] == ["suspension/laptime.py"]
    assert pb.report_blind(runs) == 2
    assert "differs from the sealed code" in capsys.readouterr().out


def test_the_preregistration_is_in_the_repo_and_matches_the_cases():
    """The sealed record: written once, before the runs. Its code hashes are checked when grading, not here (the code
    may move on after the runs; grading is done from the sealed commit)."""
    rec = json.loads((pb.BLIND / "PREREGISTRATION.json").read_text())
    assert rec["cases"] == [dict(c) for c in pb.BLIND_CASES]
    assert rec["limits"] == {"lap": pb.SIMULINK_LAP_LIMIT, "launch": pb.LAUNCH_LIMIT}
    assert set(rec["code_sha256"]) == set(pb.SEALED_CODE) and rec["created_utc"].endswith("Z")


def test_the_blind_runs_are_stored_and_grade_as_reported():
    """The six sealed cases as run on 5 Nov 2026: four ran, both launches failed in Simscape, 2 of 20 graded metrics
    within the limits (docs/POWERTRAIN_BENCHMARK_PLAN.md). Re-graded only while the code is still the sealed code."""
    import hashlib
    runs = pb.BLIND / "runs"
    for line in (runs / "MANIFEST.sha256").read_text().splitlines():
        digest, name = line.split(maxsplit=1)
        assert hashlib.sha256((runs / name).read_bytes()).hexdigest() == digest, name
    r = pb.grade_blind(runs)
    if r["changed"]:
        pytest.skip("the code has moved on from the sealed commit; the stored GRADED.txt is the record")
    status = {c["name"]: c["status"] for c in r["cases"]}
    assert status == {"hock_fs_car": "graded", "hock_fs_car_fast": "graded", "hock_pushrod_fast": "graded",
                      "skidpad_fs_car": "graded", "launch_fs_car": "not run", "launch_stock": "not run"}
    oks = [ok for c in r["cases"] for *_, ok in c.get("rows", [])]
    assert (sum(oks), len(oks)) == (2, 20)


def _check_manifest(runs):
    import hashlib
    for line in (runs / "MANIFEST.sha256").read_text().splitlines():
        digest, name = line.split(maxsplit=1)
        assert hashlib.sha256((runs / name).read_bytes()).hexdigest() == digest, name


@pytest.mark.parametrize("folder, research, fit, unsafe", [("runs", (28, 39), 34, 0),
                                                           ("runs_old_exporter", (8, 29), 15, 5)])
def test_the_round_two_runs_are_stored_and_grade_as_reported(folder, research, fit, unsafe):
    """Round 2 as run on 5 Nov 2026 after the seal (commit 788a137): `runs` with the sealed MATLAB files, 28 of 39
    within 3 %; `runs_old_exporter` with outdated simulink_lap.m and export_achilles.m (a protocol deviation, kept on
    record), 8 of 29. Re-graded only while the code is still the sealed code."""
    runs = pb.BLIND2 / folder
    _check_manifest(runs)
    r = pb.grade_blind(runs, pb.BLIND2 / "PREREGISTRATION.json")
    if r["changed"]:
        pytest.skip("the code has moved on from the sealed commit; the stored GRADED.txt is the record")
    assert all(c["status"] == "graded" for c in r["cases"]) and len(r["cases"]) == len(pb.BLIND2_CASES)
    oks = [ok for c in r["cases"] for *_, ok in c["rows"]]
    fits = [f for c in r["cases"] for f in c["fitness"].values()]
    assert (sum(oks), len(oks)) == research
    assert sum(bool(ok) for ok, _ in fits) == fit and sum(u for _, u in fits) == unsafe


def test_round_two_cases_are_the_ones_matlab_runs():
    import re
    m = (pb.BENCH / "matlab" / "kinematik_blind2_runs.m").read_text()
    rows = re.findall(r"^\s*'(\w+)',\s*'(\w+)',\s*'([^']+)',\s*(true|false),\s*([\d.]+|NaN)\s*$", m, re.M)
    assert [(c["name"], c["vehicle_data"], c["event"], c["fs_car"], c["speed"]) for c in pb.BLIND2_CASES] == [
        (n, d, e, f == "true", None if g == "NaN" else float(g)) for n, d, e, f, g in rows]
    assert re.search(r"^kb_pedal = 0\.10;", m, re.M) and all(c["pedal"] == 0.10 for c in pb.BLIND2_CASES
                                                              if c["kind"] == "launch")
    assert all(c["skip_s"] == 3.0 for c in pb.BLIND2_CASES if c["kind"] == "lap")
    assert pb.SEALED_CODE2[4] == "benchmarks/powertrain/matlab/kinematik_blind2_runs.m" and len(pb.SEALED_CODE2) == 7


def test_the_grading_window_and_a_part_pedal_launch(tmp_path):
    inputs = str(pb.BENCH / "inputs.json")
    lap = _simulink_lap(tmp_path)
    full = pb.simulink_lap_rows(inputs, lap)
    cut = pb.simulink_lap_rows(inputs, lap, skip_s=3.0)
    assert cut["lap_time_simulink_s"] == pytest.approx(full["lap_time_simulink_s"] - 3.0, abs=0.011)
    assert cut["distance_m"] < full["distance_m"]
    for m, k, s in cut["rows"]:
        if m != "pack_resistance_ohm":
            assert abs(k / s - 1) < 2e-3, m
    log = pb.read_motec_csv(lap)
    part = pb._from(log, 10.0)
    assert part["time"][0] >= 10.0 and part["distance"][0] == 0.0 and len(part["speed"]) == len(part["time"])
    ev, veh = INP["ev"], INP["vehicle"]
    cap = 0.10 * 400.0                                      # one motor in the shared inputs
    acc = pb.lt.acceleration_time(pb._point_mass_car(veh, 1.0 - ev["rear_static_share"]),
                                  pb._ev_powertrain(veh, ev, torque_cap_nm=cap))
    trace = pb.launch_trace(acc)
    p = _motec(tmp_path / "launch.csv", {"Time": trace["time"], "Ground Speed": trace["speed"],
                                         "Distance": trace["distance"]}, {"Time": "s", "Ground Speed": "m/s", "Distance": "m"})
    rows = {m: (k, s) for m, k, s in pb.blind_launch_rows(pb.BENCH / "inputs.json", p, pedal=0.10,
                                                          torque_per_pedal_nm=400.0)}
    assert rows["launch_75m_time_s"][0] == pytest.approx(rows["launch_75m_time_s"][1], rel=1e-3)
    full_pedal = {m: k for m, k, _ in pb.blind_launch_rows(pb.BENCH / "inputs.json", p)}
    assert full_pedal["launch_75m_time_s"] < 0.8 * rows["launch_75m_time_s"][0]     # the cap slows it a lot
    pt = pb._ev_powertrain(veh, ev, torque_cap_nm=50.0)
    assert max(pt.motor_map.torque_nm) == 50.0


def test_toe_and_a_discharging_pack_in_the_lap_grading(tmp_path):
    inp = json.loads((pb.BENCH / "simulink_hockenheim" / "inputs.json").read_text())
    inp["ev"]["motor_curve_csv"] = str(pb.BENCH / "simulink_hockenheim" / "motor_curve.csv")
    inp["vehicle"]["toe_deg"] = [0.3, 0.3, -0.2, -0.2]
    inp["ev"]["pack"].update(soc_init=0.8, ocv_soc={"soc": [0, 0.2, 0.4, 0.6, 0.8, 1],
                                                    "v": [201.6, 258.768, 264.96, 272.952, 286.128, 303.912]})
    f = tmp_path / "car.json"
    f.write_text(json.dumps(inp))
    lap = pb.BENCH / "simulink_hockenheim" / "simulink_lap.csv"
    r = pb.simulink_lap_rows(str(f), str(lap))
    assert "toe scrub" in r["model"] and r["model"].endswith("pack voltage falling with charge")
    base = pb.simulink_lap_rows(str(pb.BENCH / "simulink_hockenheim" / "inputs.json"), str(lap))
    e = {m: k for m, k, _ in r["rows"]}
    e0 = {m: k for m, k, _ in base["rows"]}
    assert e["energy_kwh_per_km"] > e0["energy_kwh_per_km"]          # the scrub costs energy
    assert e["min_pack_voltage_v"] < e0["min_pack_voltage_v"]        # the voltage sags as the charge goes


def test_the_round_two_preregistration_matches_its_cases():
    rec = json.loads((pb.BLIND2 / "PREREGISTRATION.json").read_text())
    assert rec["cases"] == [dict(c) for c in pb.BLIND2_CASES]
    assert rec["limits"] == {"lap": pb.SIMULINK_LAP_LIMIT2, "launch": pb.LAUNCH_LIMIT2}
    assert set(rec["limits"]["lap"].values()) | set(rec["limits"]["launch"].values()) == {0.03}
    assert set(rec["code_sha256"]) == set(pb.SEALED_CODE2) and rec["created_utc"].endswith("Z")
    assert rec["fitness_for_use"] == pb.FITNESS2
    graded = set(rec["limits"]["lap"]) | set(rec["limits"]["launch"])
    assert set(pb.FITNESS2) <= graded


def test_fitness_verdict_uses_the_larger_bar_and_flags_the_unsafe_side():
    f = pb.FITNESS2
    assert pb.fitness_verdict("energy_kwh_per_km", 1.049, 1.0, f) == (True, False)
    assert pb.fitness_verdict("energy_kwh_per_km", 0.94, 1.0, f) == (False, False)
    assert pb.fitness_verdict("peak_current_a", 120.0, 100.0, f) == (False, False)   # over: costs weight only
    assert pb.fitness_verdict("peak_current_a", 85.0, 100.0, f) == (False, True)     # under: unsafe
    assert pb.fitness_verdict("peak_current_a", 91.0, 100.0, f) == (True, False)
    assert pb.fitness_verdict("pack_temp_rise_k", 2.45, 2.0, f) == (True, False)     # 0.5 K floor beats 10 %
    assert pb.fitness_verdict("pack_temp_rise_k", 2.6, 2.0, f) == (False, False)
    assert pb.fitness_verdict("pack_temp_rise_k", 10.9, 10.0, f) == (True, False)
    assert pb.fitness_verdict("lap_time_s", 1.0, 2.0, f) == (None, False)
    assert pb.fitness_verdict("energy_kwh_per_km", 1.0, 0.0, f) == (None, False)
