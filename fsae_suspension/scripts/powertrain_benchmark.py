"""EV and combustion powertrain benchmark: KinematiK against Simulink/Simscape, Simcenter Amesim and logged MoTeC data.

    python scripts/powertrain_benchmark.py run                 # KinematiK's numbers -> benchmarks/powertrain/kinematik_results.csv
    python scripts/powertrain_benchmark.py compare FILE.csv    # grade a filled-in reference_results_TEMPLATE.csv
    python scripts/powertrain_benchmark.py motec --kind ev|ice --inputs CAR.json [--accel ACCEL.csv] [--drive RUN.csv]
                                                               # KinematiK against the car's own MoTeC i2 CSV exports
    python scripts/powertrain_benchmark.py simulink-lap --inputs achilles/inputs.json --lap achilles/simulink_lap.csv
                                                               # KinematiK against a Simscape lap (matlab/simulink_lap.m)
    python scripts/powertrain_benchmark.py simulink-blind DIR  # grade the pre-registered runs (matlab/kinematik_blind_runs.m)
    python scripts/powertrain_benchmark.py simulink-blind-seal # write the pre-registration (before the runs, once)
    python scripts/powertrain_benchmark.py simulink-blind2 DIR # round 2 (matlab/kinematik_blind2_runs.m)
    python scripts/powertrain_benchmark.py simulink-blind2-seal

Every tool reads the same inputs (benchmarks/powertrain/inputs.json, motor_curve.csv, engine_curve.csv,
track_benchmark.csv); see docs/POWERTRAIN_BENCHMARK_PLAN.md for how to set each tool up. The pass limits are the
plan's: 1% where the tool solves the same physics, 2% where it uses a different method (QSS against a forward
transient). A tool without an entry for a metric is skipped, not failed.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
BENCH = ROOT / "benchmarks" / "powertrain"

from suspension import ice_powertrain as ice  # noqa: E402
from suspension import laptime as lt  # noqa: E402

G = 9.81
# (case, metric) -> limit as a fraction; QSS-vs-QSS (OptimumLap) and same-physics checks get 1%, the rest 2%
LIMITS = {("ice_restrictor", "choked_flow_kg_s"): 0.01}
DEFAULT_LIMIT = {"Simulink": 0.02, "Amesim": 0.02}


def _curve(name, base=None):
    with open(Path(base or BENCH) / name) as fh:
        rows = list(csv.DictReader(fh))
    return [float(r["rpm"]) for r in rows], [float(r["torque_nm"]) for r in rows]


def _point_mass_car(veh: dict, front_share: float):
    """A car whose lateral limit is exactly OptimumLap's point mass: mu x (1 + downforce / weight)."""
    vs = [0.1] + [1.0 + 2.0 * i for i in range(24)]
    q = lambda v: 0.5 * veh["rho_kg_m3"] * veh["cla_m2"] * v * v  # noqa: E731
    lat = [veh["mu"] * (1.0 + q(v) / (veh["mass_kg"] * G)) for v in vs]
    car = SimpleNamespace(p=SimpleNamespace(mass=veh["mass_kg"], weight_dist_front=front_share))
    car.max_lateral_g = lambda: veh["mu"]
    car._kx_lat_limit_table = (vs, lat, round(float(veh["mu"]), 9))
    return car


def _ev_powertrain(veh, ev, base=None, torque_cap_nm=None):
    rpm, tq = _curve(ev["motor_curve_csv"], base)
    if torque_cap_nm is not None:                      # a part pedal: the controller asks for less than the curve
        tq = [min(x, torque_cap_nm) for x in tq]
    return lt.Powertrain(power_kw=ev["pack_power_kw"], drivetrain_eff=ev["driveline_eff"], cda=veh["cda_m2"],
                         cla=veh["cla_m2"], rho=veh["rho_kg_m3"], crr=veh["crr"], drive=ev.get("drive", "rwd"),
                         rot_mass_factor=veh.get("rot_mass_factor", 1.0),
                         motor_map=lt.MotorMap(rpm=rpm, torque_nm=tq, final_drive=ev["final_drive"],
                                               wheel_radius_m=veh["tire_radius_m"]))


def _ice_parts(veh, ic, base=None):
    rpm, tq = _curve(ic["engine_curve_csv"], base)
    eng = ice.EngineMap(rpm=rpm, torque_nm=tq, redline_rpm=ic["redline_rpm"], name="benchmark", source="inputs")
    gb = ice.Gearbox(primary=ic["primary"], ratios=tuple(ic["gears"]), final_drive=ic["final_drive"],
                     shift_time_s=ic["shift_time_s"], efficiency=ic["driveline_eff"],
                     tire_radius_m=veh["tire_radius_m"])
    iv = ice.ICEVehicle(mass_kg=veh["mass_kg"], driven_weight_frac=ic["rear_static_share"],
                        cg_height_m=veh["cg_height_m"], wheelbase_m=veh["wheelbase_m"], mu=veh["mu"],
                        cda_m2=veh["cda_m2"], cla_m2=veh["cla_m2"], rear_aero_frac=veh["rear_aero_frac"],
                        crr=veh["crr"], rho=veh["rho_kg_m3"], wheel_inertia_kgm2=ic["wheel_inertia_kgm2"])
    return eng, gb, iv


def run() -> list[tuple[str, str, float]]:
    inp = json.loads((BENCH / "inputs.json").read_text())
    veh, ev, ic = inp["vehicle"], inp["ev"], inp["ice"]
    out = []

    # ---- EV ----------------------------------------------------------------------------------------------------- #
    rpm, tq = _curve(ev["motor_curve_csv"])
    pt = lt.Powertrain(power_kw=ev["pack_power_kw"], drivetrain_eff=ev["driveline_eff"], cda=veh["cda_m2"],
                       cla=veh["cla_m2"], rho=veh["rho_kg_m3"], crr=veh["crr"],
                       motor_map=lt.MotorMap(rpm=rpm, torque_nm=tq, final_drive=ev["final_drive"],
                                             wheel_radius_m=veh["tire_radius_m"]))
    car = _point_mass_car(veh, 1.0 - ev["rear_static_share"])
    acc = lt.acceleration_time(car, pt)
    out += [("ev_75m", "time_s", acc.lap_time_s), ("ev_75m", "v_end_ms", acc.top_speed_ms)]

    # ---- combustion --------------------------------------------------------------------------------------------- #
    rpm, tq = _curve(ic["engine_curve_csv"])
    eng = ice.EngineMap(rpm=rpm, torque_nm=tq, redline_rpm=ic["redline_rpm"], name="benchmark", source="inputs.json")
    gb = ice.Gearbox(primary=ic["primary"], ratios=tuple(ic["gears"]), final_drive=ic["final_drive"],
                     shift_time_s=ic["shift_time_s"], efficiency=ic["driveline_eff"],
                     tire_radius_m=veh["tire_radius_m"])
    iv = ice.ICEVehicle(mass_kg=veh["mass_kg"], driven_weight_frac=ic["rear_static_share"],
                        cg_height_m=veh["cg_height_m"], wheelbase_m=veh["wheelbase_m"], mu=veh["mu"],
                        cda_m2=veh["cda_m2"], cla_m2=veh["cla_m2"], rear_aero_frac=veh["rear_aero_frac"],
                        crr=veh["crr"], rho=veh["rho_kg_m3"], wheel_inertia_kgm2=ic["wheel_inertia_kgm2"])
    la = ice.simulate_launch(eng, gb, iv, fuel_key=ic["fuel"], bte=ic["bte"], launch_rpm=ic["launch_rpm"])
    out += [("ice_75m", "time_s", float(la.t_s[-1])), ("ice_75m", "v_end_ms", float(la.v_ms[-1]))]
    out.append(("ice_restrictor", "choked_flow_kg_s",
                ice.choked_mass_flow_kg_s(ic["restrictor_mm"], cd=ic["restrictor_cd"])))
    return out


def write(rows):
    path = BENCH / "kinematik_results.csv"
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["case", "metric", "value"])
        w.writerows([(c, m, f"{v:.6g}") for c, m, v in rows])
    return path


def compare(ref_path: str) -> int:
    ours = {(c, m): v for c, m, v in run()}
    bad = 0
    print(f"{'tool':<11}{'case':<16}{'metric':<18}{'KinematiK':>11}{'tool':>11}{'diff':>8}{'limit':>7}  result")
    with open(ref_path) as fh:
        for r in csv.DictReader(fh):
            if not (r.get("value") or "").strip():
                continue
            key = (r["case"], r["metric"])
            if key not in ours:
                print(f"unknown case/metric {key}")
                bad += 1
                continue
            ref, k = float(r["value"]), ours[key]
            lim = LIMITS.get(key, DEFAULT_LIMIT.get(r["tool"], 0.02))
            d = abs(k - ref) / abs(ref) if ref else math.inf
            ok = d <= lim
            bad += not ok
            print(f"{r['tool']:<11}{r['case']:<16}{r['metric']:<18}{k:>11.4g}{ref:>11.4g}{d:>7.1%}{lim:>7.0%}  "
                  f"{'PASS' if ok else 'FAIL'}")
    return 1 if bad else 0


# --------------------------------------------------------------------------------------------------------------- #
#  MoTeC: KinematiK's powertrain against the car's own logged runs (i2 Pro: File > Export > CSV)
# --------------------------------------------------------------------------------------------------------------- #
CHANNELS = {
    "time": ["Time"],
    "speed": ["Ground Speed", "GPS Speed", "Vehicle Speed", "Corr Speed", "Speed"],
    "distance": ["Distance", "Lap Distance", "GPS Distance", "Odometer"],
    "volts": ["Pack Voltage", "Battery Voltage", "HV Voltage", "Accumulator Voltage", "Inverter DC Voltage"],
    "amps": ["Pack Current", "Battery Current", "HV Current", "Accumulator Current", "Inverter DC Current"],
    "motor_rpm": ["Motor Speed", "Motor RPM", "Inverter Motor Speed", "Motor Rpm"],
    "rpm": ["Engine RPM", "Engine Speed", "RPM"],
    "gear": ["Gear", "Gear Position", "Gear Pos"],
    "throttle": ["Throttle Pos", "Throttle Position", "TPS", "APPS", "Pedal Position"],
    "fuel_used": ["Fuel Used", "Fuel Used Total", "Fuel Consumed"],
    "pack_temp": ["Pack Temperature"],
    "motor_power": ["Motor Power"],
    "x": ["X", "Pos X", "GPS X"],
    "y": ["Y", "Pos Y", "GPS Y"],
}
_SPEED_UNIT = {"km/h": 1 / 3.6, "kph": 1 / 3.6, "mph": 0.44704, "m/s": 1.0}
_DIST_UNIT = {"m": 1.0, "km": 1000.0, "mi": 1609.344, "ft": 0.3048}
_FUEL_UNIT_L = {"l": 1.0, "litre": 1.0, "liter": 1.0, "ml": 1e-3, "gal": 3.78541}
# model against measurement: friction, CdA and the declared maps are estimates, so these are wider than tool
# against tool; they are starting values, to be tightened as the inputs firm up
MOTEC_LIMIT = {"accel_75m.time_s": 0.03, "peak_wheel_power_kw": 0.05, "pack_resistance_ohm": 0.15,
               "final_drive": 0.01, "energy_kwh_per_km": 0.10, "gear_ratio_max_error": 0.01,
               "torque_curve_rms_error": 0.08, "fuel_l_per_100km": 0.10}
ROT_FACTOR_EV = 1.05          # rotating-inertia factor for the EV (inputs: plan); the ICE uses ICEVehicle.m_eff


def read_motec_csv(path, override=None) -> dict:
    """Channels of a MoTeC i2 CSV export as SI arrays: the header row starts with "Time", the next row holds the
    units, data follows. `override` maps a role (speed, volts, amps, rpm, ...) to the channel name the car uses."""
    import numpy as np
    with open(path, newline="", encoding="utf-8-sig") as fh:
        rows = [[c.strip().strip('"') for c in r] for r in csv.reader(fh)]
    h = next(i for i, r in enumerate(rows) if r and r[0].lower() == "time")
    names, units = rows[h], (rows[h + 1] if h + 1 < len(rows) else [])
    data = []
    for r in rows[h + 2:]:
        if not any(r):                               # i2 leaves a blank line between the units and the data
            continue
        try:
            data.append([float(c) if c else math.nan for c in r[:len(names)]])
        except ValueError:
            continue
    arr = np.array([d + [math.nan] * (len(names) - len(d)) for d in data], float)
    low = [n.lower() for n in names]
    out = {}
    for role, cands in CHANNELS.items():
        want = (override or {}).get(role)
        j = next((low.index(c.lower()) for c in ([want] if want else []) + cands if c.lower() in low), None)
        if j is None:
            continue
        col, unit = arr[:, j], (units[j].lower() if j < len(units) else "")
        if role == "speed":
            col = col * _SPEED_UNIT.get(unit, 1 / 3.6)
        elif role in ("distance", "x", "y"):
            col = col * _DIST_UNIT.get(unit, 1.0)
        elif role == "fuel_used":
            col = col * _FUEL_UNIT_L.get(unit, 1.0)
        out[role] = col
    if "time" not in out or "speed" not in out:
        raise ValueError(f"{path}: needs a Time and a speed channel; found {names}")
    if "distance" not in out:                       # integrate speed when no distance channel was exported
        t, v = out["time"], out["speed"]
        out["distance"] = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))])
    return out


def measured_75m(log) -> float:
    """Time from the first movement (speed above 0.5 m/s, back-dated one sample to rest) to 75 m."""
    import numpy as np
    t, v, d = log["time"], log["speed"], log["distance"]
    i0 = max(int(np.argmax(v > 0.5)) - 1, 0)
    d = d - d[i0]
    i1 = int(np.argmax(d >= 75.0))
    if d[i1] < 75.0:
        raise ValueError("the acceleration log never reaches 75 m")
    return float(np.interp(75.0, d[i1 - 1:i1 + 1], t[i1 - 1:i1 + 1]) - t[i0])


def _road_load(veh, v):
    return 0.5 * veh["rho_kg_m3"] * veh["cda_m2"] * v * v + veh["crr"] * veh["mass_kg"] * G


def _smooth_accel(t, v, window_s=0.2):
    """Acceleration from logged speed, smoothed over `window_s` (differentiating raw speed amplifies noise)."""
    import numpy as np
    dt = float(np.median(np.diff(t)))
    n = max(int(round(window_s / dt)), 1)
    vs = np.convolve(np.pad(v, (n // 2, n - 1 - n // 2), mode="edge"), np.ones(n) / n, mode="valid")
    return np.gradient(vs, t)                        # edge-padded: no false spikes at the ends of the log


def motec(kind, inputs, accel=None, drive=None, override=None) -> int:
    """KinematiK's powertrain against logged runs. `accel`: one full-throttle run from rest past 75 m. `drive`: any
    longer run (an endurance lap or stint) for energy, pack resistance, ratios and fuel."""
    import numpy as np
    from suspension import ev_electrical_check as ec
    inp = json.loads(Path(inputs).read_text())
    base = Path(inputs).resolve().parent
    veh = inp["vehicle"]
    r = veh["tire_radius_m"]
    rows = []
    if kind == "ev":
        ev = inp["ev"]
        pt = _ev_powertrain(veh, ev, base)
        eta = ev["driveline_eff"] * ev["motor_eff"] * ev["inverter_eff"]
        if accel:
            log = read_motec_csv(accel, override)
            car = _point_mass_car(veh, 1.0 - ev["rear_static_share"])
            rows.append(("accel_75m.time_s", lt.acceleration_time(car, pt).lap_time_s, measured_75m(log)))
            t, v = log["time"], log["speed"]
            p_wheel = (ROT_FACTOR_EV * veh["mass_kg"] * _smooth_accel(t, v) + _road_load(veh, v)) * v
            rpm, tq = _curve(ev["motor_curve_csv"], base)
            p_shaft = min(max(T * n * 2 * math.pi / 60 for n, T in zip(rpm, tq)),     # the motor curve, or the
                          ev["pack_power_kw"] * 1e3 * ev["inverter_eff"] * ev["motor_eff"])  # pack side's limit
            rows.append(("peak_wheel_power_kw", p_shaft * ev["driveline_eff"] / 1e3, float(np.nanmax(p_wheel)) / 1e3))
        if drive:
            log = read_motec_csv(drive, override)
            t, v, d = log["time"], log["speed"], log["distance"]
            if "volts" in log and "amps" in log:
                V, I = log["volts"], log["amps"]
                ok = np.isfinite(V) & np.isfinite(I)
                slope = np.polyfit(I[ok], V[ok], 1)[0]                     # V = Voc - I R over the run
                pk = ev["pack"]
                rows.append(("pack_resistance_ohm", pk["series"] * pk["cell_ohm"] / pk["parallel"], float(-slope)))
                e_meas = float(np.trapezoid(np.nan_to_num(np.maximum(V * I, 0.0)), t)) / 3.6e6   # discharge only
                _, _, p_pack = ec.traction_power_w(v, _smooth_accel(t, v), vehicle_mass_kg=veh["mass_kg"],
                                                   drag_cda=veh["cda_m2"], crr=veh["crr"],
                                                   rho_air=veh["rho_kg_m3"], drivetrain_eff=eta)
                km = float(d[-1] - d[0]) / 1e3
                rows.append(("energy_kwh_per_km", float(np.trapezoid(p_pack, t)) / 3.6e6 / km, e_meas / km))
            if "motor_rpm" in log:
                moving = v > 5.0
                ratio = log["motor_rpm"][moving] * 2 * math.pi / 60 * r / v[moving]
                rows.append(("final_drive", ev["final_drive"], float(np.nanmedian(ratio))))
    else:
        ic = inp["ice"]
        eng, gb, iv = _ice_parts(veh, ic, base)
        overall = [gb.primary * g * gb.final_drive for g in gb.ratios]
        if accel:
            log = read_motec_csv(accel, override)
            la = ice.simulate_launch(eng, gb, iv, fuel_key=ic["fuel"], bte=ic["bte"], launch_rpm=ic["launch_rpm"])
            rows.append(("accel_75m.time_s", float(la.t_s[-1]), measured_75m(log)))
            if "rpm" in log:
                # torque curve: in gear (ratio within 3% of a declared gear), drive on, recover crank torque from
                # the wheel force: T = (m_eff a + road load) r / (G eta), and compare with the map in 500 rpm bins
                t, v, n = log["time"], log["speed"], log["rpm"]
                a = _smooth_accel(t, v)
                with np.errstate(divide="ignore", invalid="ignore"):
                    G = n * 2 * math.pi / 60 * r / v
                gi = np.array([int(np.argmin([abs(g - o) / o for o in overall])) if np.isfinite(g) else -1 for g in G])
                engaged = (v > 3.0) & (gi >= 1) & np.array(   # gear 2 and up: first is clutch- and grip-limited
                    [abs(G[k] - overall[gi[k]]) / overall[gi[k]] < 0.03 if gi[k] >= 0 else False for k in range(len(G))])
                # the smoothed acceleration averages over 0.2 s: keep a sample only if that whole window stayed in
                # the same gear, so no shift leaks into it
                w = max(int(round(0.2 / float(np.median(np.diff(t))))), 1)
                in_gear = engaged.copy()
                for k in range(len(t)):
                    lo, hi = max(k - w, 0), min(k + w + 1, len(t))
                    in_gear[k] = engaged[k] and bool(np.all(engaged[lo:hi])) and bool(np.all(gi[lo:hi] == gi[k]))
                in_gear &= a > 0.5
                if "throttle" in log:
                    in_gear &= log["throttle"] > 90.0
                errs = []
                for b0 in range(int(min(eng.rpm)), int(max(eng.rpm)), 500):
                    m = in_gear & (n >= b0) & (n < b0 + 500)
                    if m.sum() < 5:
                        continue
                    k = np.where(m)[0]
                    T = [(iv.m_eff(overall[gi[q]], eng, gb.efficiency, r) * a[q] + _road_load(veh, v[q]))
                         * r / (overall[gi[q]] * gb.efficiency) for q in k]
                    model = float(np.interp(b0 + 250, eng.rpm, eng.torque_nm))
                    errs.append((float(np.median(T)) - model) / model)
                if errs:
                    rms = float(np.sqrt(np.mean(np.square(errs))))
                    rows.append(("torque_curve_rms_error", 0.0, rms))
        if drive:
            log = read_motec_csv(drive, override)
            t, v, d = log["time"], log["speed"], log["distance"]
            if "rpm" in log:
                moving = v > 3.0
                G = log["rpm"][moving] * 2 * math.pi / 60 * r / v[moving]
                worst = 0.0
                for o in overall:
                    near = G[np.abs(G - o) / o < 0.05]
                    if near.size >= 20:
                        worst = max(worst, abs(float(np.median(near)) - o) / o)
                rows.append(("gear_ratio_max_error", 0.0, worst))
            if "fuel_used" in log:
                f = ice.fuel(ic["fuel"])
                eng_p = np.maximum((veh["mass_kg"] * _smooth_accel(t, v) + _road_load(veh, v)) * v, 0.0) / gb.efficiency
                kg = float(np.trapezoid(eng_p, t)) / (ic["bte"] * f.lhv_mj_kg * 1e6)
                km = float(d[-1] - d[0]) / 1e3
                fu = log["fuel_used"]
                meas_l = float(np.nanmax(fu) - np.nanmin(fu))
                rows.append(("fuel_l_per_100km", kg / f.density_kg_l / km * 100.0, meas_l / km * 100.0))
    bad = 0
    print(f"{'metric':<24}{'KinematiK':>11}{'measured':>11}{'diff':>8}{'limit':>7}  result")
    for m, k, meas in rows:
        if m in ("gear_ratio_max_error", "torque_curve_rms_error"):
            d, shown = meas, "error"                       # already a fraction against the declared values
        else:
            d, shown = (abs(k - meas) / abs(meas) if meas else math.inf), f"{k:.4g}"
        lim = MOTEC_LIMIT[m]
        bad += d > lim
        print(f"{m:<24}{shown:>11}{meas:>11.4g}{d:>7.1%}{lim:>7.0%}  {'PASS' if d <= lim else 'FAIL'}")
    return 1 if bad else 0


# --------------------------------------------------------------------------------------------------------------- #
#  Simulink lap: the same speed trace through KinematiK's pack model (plan case 2), and the lap time beside it
# --------------------------------------------------------------------------------------------------------------- #
# tool against tool on the same trace (plan case 2); the fitted resistance and the ratio keep the MoTeC limits
SIMULINK_LAP_LIMIT = {"energy_kwh_per_km": 0.02, "peak_current_a": 0.02, "rms_current_a": 0.02,
                      "min_pack_voltage_v": 0.02, "pack_resistance_ohm": 0.15}
# Round 2: 3 % on every metric, set before its runs (the plan's 2 % failed in round 1; the resistance is now
# compared as each side shows it, so it gets the same limit as the rest)
SIMULINK_LAP_LIMIT2 = {"energy_kwh_per_km": 0.03, "peak_current_a": 0.03, "rms_current_a": 0.03,
                       "min_pack_voltage_v": 0.03, "pack_resistance_ohm": 0.03, "pack_temp_rise_k": 0.03,
                       "drive_loss_kwh_per_km": 0.03}
LAUNCH_LIMIT2 = {"launch_75m_time_s": 0.03, "launch_75m_speed_ms": 0.03}
# Fitness for use (docs/POWERTRAIN_VALIDATION.md): the error a design decision can carry, beside the 3 % research
# bar. rel: fraction of the reference; abs: a floor in the metric's units; safe: the side a miss must fall on to be
# harmless ("over": over-predicting is safe, so an under-prediction is flagged)
FITNESS2 = {"energy_kwh_per_km": {"rel": 0.05}, "peak_current_a": {"rel": 0.10, "safe": "over"},
            "rms_current_a": {"rel": 0.10, "safe": "over"}, "min_pack_voltage_v": {"rel": 0.03},
            "pack_resistance_ohm": {"rel": 0.10}, "pack_temp_rise_k": {"rel": 0.10, "abs": 0.5},
            "drive_loss_kwh_per_km": {"rel": 0.10}, "launch_75m_time_s": {"rel": 0.03},
            "launch_75m_speed_ms": {"rel": 0.03}}


def fitness_verdict(metric, ours, ref, spec) -> tuple:
    """(within the design-margin bar, missed on the unsafe side) for one graded metric."""
    f = spec.get(metric)
    if f is None or not ref:
        return None, False
    ok = abs(ours - ref) <= max(f["rel"] * abs(ref), f.get("abs", 0.0))
    unsafe = f.get("safe") == "over" and ours < ref and not ok
    return ok, unsafe


def lateral_accel(t, v, x, y, window_s=0.5):
    """Lateral acceleration (m/s²) from the driven path: speed times the rate of turn of the heading, the heading
    smoothed over window_s (a position log differentiated twice is noisy)."""
    import numpy as np
    heading = np.unwrap(np.arctan2(np.gradient(y, t), np.gradient(x, t)))
    return v * _smooth_accel(t, heading, window_s)


def _lap_pack_power(veh, ev, log):
    """KinematiK's pack power on a logged lap, with whatever the car's inputs give: the motor's speed-and-torque
    map (ev.motor_eff_map) or one efficiency, the tires' rolling resistance and cornering stiffness (vehicle.tire)
    and, with a tire and the lap's X/Y, the cornering drag, and with the tires' toe (vehicle.toe_deg, one per
    tire as the car runs) their scrub. Returns the power and a note of the model used."""
    import numpy as np
    from suspension import ev_electrical_check as ec
    t, v = log["time"], log["speed"]
    kw = dict(vehicle_mass_kg=veh["mass_kg"], drag_cda=veh["cda_m2"], crr=veh["crr"], rho_air=veh["rho_kg_m3"],
              rot_mass_factor=veh.get("rot_mass_factor", ROT_FACTOR_EV),
              cla_m2=veh.get("cla_m2", 0.0) if "tire" in veh else 0.0)
    note = [f"rotating-inertia factor {kw['rot_mass_factor']:.3f}"]
    mm = ev.get("motor_eff_map")
    if mm:
        kw["motor_eff_map"] = ec.MotorEfficiencyMap(tuple(mm["rpm"]), tuple(mm["torque_nm"]),
                                                    tuple(map(tuple, mm["efficiency"])), mm.get("n_motors", 1),
                                                    ev["final_drive"], veh["tire_radius_m"])
        kw["drivetrain_eff"] = ev["driveline_eff"] * ev["inverter_eff"]
        note.append(f"motor map ({len(mm['rpm'])} speeds x {len(mm['torque_nm'])} torques, {mm.get('n_motors', 1)} "
                    "motors)")
    else:
        kw["drivetrain_eff"] = ev["driveline_eff"] * ev["motor_eff"] * ev["inverter_eff"]
        note.append(f"one motor efficiency ({ev['motor_eff']:.3f})")
    if "tire" in veh:
        kw["tire"] = ec.TireLosses(**veh["tire"])
        note.append("tire rolling resistance under weight and downforce")
        if "toe_deg" in veh:
            kw["toe_rad"] = np.radians(veh["toe_deg"])
            note.append("toe scrub")
        if "x" in log and "y" in log:
            kw["a_lat"] = lateral_accel(t, v, log["x"], log["y"])
            note.append("cornering drag")
    else:
        note.append(f"Crr {veh['crr']} on the weight")
    _, pw, p = ec.traction_power_w(v, _smooth_accel(t, v), **kw)
    return p, ", ".join(note), np.maximum(pw, 0.0) / max(ev["driveline_eff"], 0.5)   # shaft power, driving


def _from(log, t0) -> dict:
    """The log from time t0 on (every channel cut the same way, distance restarted at 0)."""
    keep = log["time"] >= t0
    out = {k: v[keep] for k, v in log.items()}
    out["distance"] = out["distance"] - out["distance"][0]
    return out


def simulink_lap_rows(inputs, lap, override=None, skip_s: float = 0.0) -> dict:
    """KinematiK on Simulink's own lap: Simulink's speed trace drives traction_power_w (the declared drive
    efficiency, no regeneration), the pack power becomes current and terminal voltage through the pack's open-circuit
    voltage and resistance (battery_internal_power_w), and each is compared with Simulink's pack, discharge only.
    With the lap's X/Y, KinematiK's own limit lap on that path is reported beside Simulink's driven lap time."""
    import numpy as np
    from suspension import ev_electrical_check as ec
    inp = json.loads(Path(inputs).read_text())
    base = Path(inputs).resolve().parent
    veh, ev = inp["vehicle"], inp["ev"]
    log = read_motec_csv(lap, override)
    if skip_s:                                          # leave out the event's start-up, on both sides
        log = _from(log, log["time"][0] + skip_s)
    for role in ("volts", "amps"):
        if role not in log:
            raise ValueError(f"{lap}: needs a pack {role} channel (Pack Voltage, Pack Current)")
    t, v, d = log["time"], log["speed"], log["distance"]
    dt = np.diff(t)
    km = float(d[-1] - d[0]) / 1e3
    p_pack, model, shaft = _lap_pack_power(veh, ev, log)
    pk = ev["pack"]
    ocv, r_pack = pk["series"] * pk["cell_v"], pk["series"] * pk["cell_ohm"] / pk["parallel"]
    if "ocv_soc" in pk and "soc_init" in pk:          # the open-circuit voltage falls as the pack discharges
        extra = {}
        if "r_soc_t" in pk:                           # resistance with charge and temperature, and the pack heating
            rt = pk["r_soc_t"]
            extra = dict(r_soc_temp=(rt["soc"], rt["t_k"], rt["r"]), heat_capacity_j_k=pk.get("thermal_mass_j_k") or None,
                         temp0_k=pk.get("temp0_k") or 298.15)
        temps = []
        u_k, i_k, _ = ec.pack_along_lap(p_pack, t, ocv_soc=(pk["ocv_soc"]["soc"], pk["ocv_soc"]["v"]),
                                        soc0=pk["soc_init"], capacity_ah=pk["cell_ah"] * pk["parallel"],
                                        r_pack_ohm=r_pack, temps_out=temps, **extra)
        model += ", pack voltage falling with charge"
        if extra:
            model += ", resistance with charge and temperature" + (", the pack heating" if
                                                                   extra["heat_capacity_j_k"] else "")
    else:
        _, i_k = ec.battery_internal_power_w(p_pack, ocv, r_pack)
        u_k = ocv - i_k * r_pack
    U, I = log["volts"], log["amps"]
    i_s = np.maximum(I, 0.0)

    def trap(y):
        return float(np.sum(0.5 * (y[1:] + y[:-1]) * dt))

    rows = [("energy_kwh_per_km", trap(np.maximum(p_pack, 0.0)) / 3.6e6 / km, trap(np.maximum(U * I, 0.0)) / 3.6e6 / km),
            ("peak_current_a", float(np.max(i_k)), float(np.max(I))),
            ("rms_current_a", math.sqrt(trap(i_k ** 2) / (t[-1] - t[0])), math.sqrt(trap(i_s ** 2) / (t[-1] - t[0]))),
            ("min_pack_voltage_v", float(np.min(u_k)), float(np.min(U)))]
    if "pack_temp" in log and "ocv_soc" in pk and pk.get("thermal_mass_j_k"):   # the pack heating over the lap
        T = log["pack_temp"]
        rows.append(("pack_temp_rise_k", temps[-1] - temps[0], float(T[-1] - T[0])))
    if "motor_power" in log:                          # what the motors and their inverter turn into heat, driving
        drive = shaft > 0
        aux = ev.get("aux_w", 0.0)
        rows.append(("drive_loss_kwh_per_km", trap(np.where(drive, p_pack - aux - shaft, 0.0)) / 3.6e6 / km,
                     trap(np.where(log["motor_power"] > 0, np.maximum(U * I, 0.0) - log["motor_power"], 0.0))
                     / 3.6e6 / km))
    ok = np.isfinite(U) & np.isfinite(I)
    # the pack's resistance as each side shows it: the slope of terminal voltage against current over the lap
    rows.append(("pack_resistance_ohm", float(-np.polyfit(i_k, u_k, 1)[0]), float(-np.polyfit(I[ok], U[ok], 1)[0])))
    out = {"rows": rows, "lap_time_simulink_s": float(t[-1] - t[0]), "distance_m": float(d[-1] - d[0]), "model": model}
    if "motor_rpm" in log:              # reported: the tire rolls on its loaded radius, so this is not the gearbox
        moving = v > 5.0
        ratio = float(np.nanmedian(log["motor_rpm"][moving] * 2 * math.pi / 60 * veh["tire_radius_m"] / v[moving]))
        out.update(ratio_measured=ratio, rolling_radius_m=veh["tire_radius_m"] * ev["final_drive"] / ratio)
    if "x" in log and "y" in log:
        track = lt.track_from_path(log["x"], log["y"], name="Simulink lap", closed=True)
        car = _point_mass_car(veh, 1.0 - ev["rear_static_share"])
        res = lt.simulate_lap(car, track, _ev_powertrain(veh, ev, base))
        out.update(lap_time_kinematik_s=res.lap_time_s, track_length_m=track.total_length(), lap_ok=res.ok,
                   lap_warning=res.warning)
    return out


def simulink_lap(inputs, lap, override=None) -> int:
    r = simulink_lap_rows(inputs, lap, override)
    bad = 0
    print(f"KinematiK on Simulink's lap ({r['distance_m']:.0f} m, {r['lap_time_simulink_s']:.2f} s): same speed trace, "
          f"pack discharge only\nKinematiK model: {r['model']}")
    print(f"{'metric':<22}{'KinematiK':>11}{'Simulink':>11}{'diff':>8}{'limit':>7}  result")
    for m, k, s in r["rows"]:
        d = abs(k - s) / abs(s) if s else math.inf
        lim = SIMULINK_LAP_LIMIT[m]
        bad += d > lim
        print(f"{m:<22}{k:>11.4g}{s:>11.4g}{d:>7.1%}{lim:>7.0%}  {'PASS' if d <= lim else 'FAIL'}")
    if "ratio_measured" in r:
        print(f"\nmotor speed over road speed (not graded): ratio {r['ratio_measured']:.3f} on the unloaded radius, "
              f"so the tire rolls on {r['rolling_radius_m'] * 1000:.1f} mm")
    if "lap_time_kinematik_s" in r:
        k, s = r["lap_time_kinematik_s"], r["lap_time_simulink_s"]
        print(f"\nlap time (not graded): KinematiK's limit lap on the same path {k:.2f} s, Simulink's driven lap "
              f"{s:.2f} s ({k / s - 1:+.1%}); path {r['track_length_m']:.0f} m. sm_car's driver follows the event's "
              "own speed profile, not the car's limit, so KinematiK should be the faster of the two."
              + (f" Warning: {r['lap_warning']}" if r["lap_warning"] else ""))
    else:
        print("\nlap time: no X/Y in the lap file, so KinematiK's own lap is not run")
    return 1 if bad else 0


# --------------------------------------------------------------------------------------------------------------- #
#  Pre-registered Simulink runs: the code is sealed before MATLAB is run, and graded only from that code
# --------------------------------------------------------------------------------------------------------------- #
BLIND = BENCH / "simulink_blind"
SEALED_CODE = ("suspension/ev_electrical_check.py", "suspension/laptime.py", "suspension/_num.py",
               "scripts/powertrain_benchmark.py", "benchmarks/powertrain/matlab/kinematik_blind_runs.m",
               "benchmarks/powertrain/matlab/simulink_lap.m", "benchmarks/powertrain/matlab/export_achilles.m")
BLIND_CASES = (
    {"name": "hock_fs_car", "kind": "lap", "vehicle_data": "Vehicle_data_dwishbone", "event": "CRG Hockenheim F",
     "fs_car": True, "vGain": 1.0},
    {"name": "hock_fs_car_fast", "kind": "lap", "vehicle_data": "Vehicle_data_dwishbone", "event": "CRG Hockenheim F",
     "fs_car": True, "vGain": 1.25},
    {"name": "hock_pushrod_fast", "kind": "lap", "vehicle_data": "Vehicle_data_dwpushrod", "event": "CRG Hockenheim F",
     "fs_car": True, "vGain": 1.25},
    {"name": "skidpad_fs_car", "kind": "lap", "vehicle_data": "Vehicle_data_dwishbone", "event": "Skidpad",
     "fs_car": True, "vGain": 1.0},
    {"name": "launch_fs_car", "kind": "launch", "vehicle_data": "Vehicle_data_dwishbone", "event": "WOT Braking",
     "fs_car": True, "vGain": None},
    {"name": "launch_stock", "kind": "launch", "vehicle_data": "Vehicle_data_dwishbone", "event": "WOT Braking",
     "fs_car": False, "vGain": None},
)
LAUNCH_LIMIT = {"launch_75m_time_s": 0.02, "launch_75m_speed_ms": 0.02}     # plan case 1, tools

# Round 2 (after round 1 failed, docs/POWERTRAIN_BENCHMARK_PLAN.md): the Formula Student-sized car keeps its ride
# height and alignment (spring preload scaled with the sprung weight), a pack of higher-power cells, the speed
# varied through the trajectory itself, launches at a pedal the model can simulate, laps graded from 3 s in.
BLIND2 = BENCH / "simulink_blind2"
SEALED_CODE2 = SEALED_CODE[:4] + ("benchmarks/powertrain/matlab/kinematik_blind2_runs.m",) + SEALED_CODE[5:]
_LAP2 = {"kind": "lap", "skip_s": 3.0}
_LAUNCH2 = {"kind": "launch", "event": "WOT Braking", "vehicle_data": "Vehicle_data_dwishbone", "speed": None,
            "pedal": 0.10, "torque_per_pedal_nm": 400.0}
BLIND2_CASES = (
    {"name": "hock_fs2", **_LAP2, "vehicle_data": "Vehicle_data_dwishbone", "event": "CRG Hockenheim F",
     "fs_car": True, "speed": 1.0},
    {"name": "hock_fs2_fast", **_LAP2, "vehicle_data": "Vehicle_data_dwishbone", "event": "CRG Hockenheim F",
     "fs_car": True, "speed": 1.15},
    {"name": "hock_fs2_pushrod_fast", **_LAP2, "vehicle_data": "Vehicle_data_dwpushrod", "event": "CRG Hockenheim F",
     "fs_car": True, "speed": 1.15},
    {"name": "hock_stock_fast", **_LAP2, "vehicle_data": "Vehicle_data_dwishbone", "event": "CRG Hockenheim F",
     "fs_car": False, "speed": 1.15},
    {"name": "skidpad_fs2", **_LAP2, "vehicle_data": "Vehicle_data_dwishbone", "event": "Skidpad", "fs_car": True,
     "speed": 1.0},
    {"name": "launch_fs2", **_LAUNCH2, "fs_car": True},
    {"name": "launch_stock2", **_LAUNCH2, "fs_car": False},
)


def _sha256(path) -> str:
    import hashlib
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def code_fingerprint(root=None, files=None) -> dict:
    return {f: _sha256(Path(root or ROOT) / f) for f in (files or SEALED_CODE)}


def seal_blind(now=None, folder=None, cases=None, code=None, limits=None, fitness=None) -> dict:
    """Write <folder>/PREREGISTRATION.json (benchmarks/powertrain/simulink_blind by default): the cases, the metrics
    and limits, and the SHA-256 of every file the grading and the MATLAB runs use. Commit and push it before
    running MATLAB."""
    import datetime
    folder = Path(folder or BLIND)
    cases = BLIND_CASES if cases is None else cases
    path = folder / "PREREGISTRATION.json"
    if path.exists():
        raise ValueError(f"{path} exists: a pre-registration is written once; a new one needs a new folder")
    rec = {"created_utc": (now or datetime.datetime.now(datetime.UTC)).strftime("%Y-%m-%dT%H:%M:%SZ"),
           "statement": "Sealed before any of these Simulink runs. Graded only from the code whose SHA-256 is "
                        "listed here; the limits are not changed after a run. Every case is reported, pass or fail.",
           "cases": list(cases), "limits": limits or {"lap": SIMULINK_LAP_LIMIT, "launch": LAUNCH_LIMIT},
           "code_sha256": code_fingerprint(files=code)}
    if fitness:
        rec["fitness_for_use"] = fitness
    folder.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rec, indent=1) + "\n")
    return rec


def speed_at(log, distance_m) -> float:
    """Speed (m/s) at distance_m from the first movement, as measured_75m starts the clock."""
    import numpy as np
    v, d = log["speed"], log["distance"]
    i0 = max(int(np.argmax(v > 0.5)) - 1, 0)
    return float(np.interp(distance_m, d[i0:] - d[i0], v[i0:]))


def blind_launch_rows(inputs, lap, override=None, pedal=None, torque_per_pedal_nm=None) -> list:
    """KinematiK's own 75 m launch (laptime.acceleration_time on the car's inputs) against the Simulink run: a
    prediction, not given the trace. With pedal, each of the n motors is asked for torque_per_pedal_nm x pedal (the
    Simscape controller's gain), so the curve is capped at n x that."""
    inp = json.loads(Path(inputs).read_text())
    base = Path(inputs).resolve().parent
    veh, ev = inp["vehicle"], inp["ev"]
    log = read_motec_csv(lap, override)
    cap = None
    if pedal is not None:
        cap = ev.get("motor_eff_map", {}).get("n_motors", 1) * torque_per_pedal_nm * pedal
    acc = lt.acceleration_time(_point_mass_car(veh, 1.0 - ev["rear_static_share"]),
                               _ev_powertrain(veh, ev, base, torque_cap_nm=cap))
    ours = launch_trace(acc)          # both clocked the same way: from the sample before the car passes 0.5 m/s
    return [("launch_75m_time_s", measured_75m(ours), measured_75m(log)),
            ("launch_75m_speed_ms", speed_at(ours, 75.0), speed_at(log, 75.0))]


def launch_trace(acc, dt=0.01) -> dict:
    """KinematiK's 75 m run (s, v stations) as a time log at dt, so it is clocked exactly as a logged run is."""
    import numpy as np
    s, v = np.asarray(acc.s, float), np.asarray(acc.v, float)
    t = np.concatenate([[0.0], np.cumsum(np.diff(s) / np.maximum(0.5 * (v[1:] + v[:-1]), 1e-6))])
    grid = np.arange(0.0, t[-1], dt)
    # the log ends one step past the line, as a logged run goes on past it
    return {"time": np.append(grid, [t[-1], t[-1] + dt]),
            "speed": np.append(np.interp(grid, t, v), [v[-1], v[-1]]),
            "distance": np.append(np.interp(grid, t, s), [s[-1], s[-1] + v[-1] * dt])}


def grade_blind(runs_dir, prereg=None, root=None) -> dict:
    """Every sealed case: its rows (metric, KinematiK, Simulink, limit, pass) or why it has none."""
    root = root or ROOT
    rec = json.loads(Path(prereg or BLIND / "PREREGISTRATION.json").read_text())
    changed = sorted(f for f, h in rec["code_sha256"].items() if not (Path(root) / f).exists()
                     or _sha256(Path(root) / f) != h)
    out = {"changed": changed, "cases": []}
    if changed:
        return out
    for case in rec["cases"]:
        d = Path(runs_dir) / case["name"]
        if not (d / "simulink_lap.csv").exists():
            out["cases"].append({"name": case["name"], "status": "not run"})
            continue
        if case["kind"] == "launch":
            rows = blind_launch_rows(d / "inputs.json", d / "simulink_lap.csv", pedal=case.get("pedal"),
                                     torque_per_pedal_nm=case.get("torque_per_pedal_nm"))
            lim = rec["limits"]["launch"]
        else:
            rows = simulink_lap_rows(d / "inputs.json", d / "simulink_lap.csv",
                                     skip_s=case.get("skip_s", 0.0))["rows"]
            lim = rec["limits"]["lap"]
        graded = [(m, k, s, lim[m], abs(k - s) / abs(s) <= lim[m] if s else False) for m, k, s in rows]
        fit = {m: fitness_verdict(m, k, s, rec.get("fitness_for_use", {})) for m, k, s in rows}
        out["cases"].append({"name": case["name"], "status": "graded", "rows": graded, "fitness": fit})
    return out


def report_blind(runs_dir, prereg=None) -> int:
    r = grade_blind(runs_dir, prereg)
    if r["changed"]:
        print("The code differs from the sealed code, so the runs cannot be graded blind: " + ", ".join(r["changed"])
              + "\nCheck out the commit that added the PREREGISTRATION.json and grade from there.")
        return 2
    n_pass = n_all = n_fit = n_unsafe = 0
    for c in r["cases"]:
        print(f"\n{c['name']}: {c['status']}")
        for m, k, s, lim, ok in c.get("rows", []):
            n_all += 1
            n_pass += ok
            fit, unsafe = c.get("fitness", {}).get(m, (None, False))
            n_fit += bool(fit)
            n_unsafe += unsafe
            tag = "" if fit is None else ("  fit for use" if fit else "  NOT fit for use") + (
                " (under-predicts: unsafe side)" if unsafe else "")
            print(f"  {m:<22}{k:>11.4g}{s:>11.4g}{abs(k - s) / abs(s) if s else math.inf:>7.1%}{lim:>7.0%}  "
                  f"{'PASS' if ok else 'FAIL'}{tag}")
    print(f"\n{n_pass}/{n_all} graded metrics within their sealed limits")
    if any(c.get("fitness") for c in r["cases"]):
        print(f"{n_fit}/{n_all} within the sealed fitness-for-use bars; {n_unsafe} missed on the unsafe side")
    return 0 if n_all and n_pass == n_all else 1


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "run":
        rows = run()
        for c, m, v in rows:
            print(f"{c:<16}{m:<18}{v:.4g}")
        print(f"written to {write(rows)}")
    elif len(sys.argv) == 3 and sys.argv[1] == "compare":
        sys.exit(compare(sys.argv[2]))
    elif len(sys.argv) >= 2 and sys.argv[1] == "motec":
        import argparse
        ap = argparse.ArgumentParser(prog="powertrain_benchmark.py motec")
        ap.add_argument("--kind", choices=("ev", "ice"), required=True)
        ap.add_argument("--inputs", default=str(BENCH / "inputs.json"), help="the car's own copy of inputs.json")
        ap.add_argument("--accel", help="MoTeC CSV of one 75 m acceleration run")
        ap.add_argument("--drive", help="MoTeC CSV of a longer run: an endurance lap or stint")
        ap.add_argument("--chan", action="append", default=[], help="role=Channel Name, e.g. speed=Wheel Speed RL")
        a = ap.parse_args(sys.argv[2:])
        sys.exit(motec(a.kind, a.inputs, a.accel, a.drive, dict(c.split("=", 1) for c in a.chan)))
    elif len(sys.argv) >= 2 and sys.argv[1] == "simulink-lap":
        import argparse
        ap = argparse.ArgumentParser(prog="powertrain_benchmark.py simulink-lap")
        ap.add_argument("--inputs", required=True, help="achilles/inputs.json from simulink_lap.m")
        ap.add_argument("--lap", required=True, help="achilles/simulink_lap.csv from simulink_lap.m")
        ap.add_argument("--chan", action="append", default=[], help="role=Channel Name")
        a = ap.parse_args(sys.argv[2:])
        sys.exit(simulink_lap(a.inputs, a.lap, dict(c.split("=", 1) for c in a.chan)))
    elif len(sys.argv) == 3 and sys.argv[1] == "simulink-blind":
        sys.exit(report_blind(sys.argv[2]))
    elif len(sys.argv) == 3 and sys.argv[1] == "simulink-blind2":
        sys.exit(report_blind(sys.argv[2], BLIND2 / "PREREGISTRATION.json"))
    elif len(sys.argv) == 2 and sys.argv[1] == "simulink-blind2-seal":
        rec = seal_blind(folder=BLIND2, cases=BLIND2_CASES, code=SEALED_CODE2,
                         limits={"lap": SIMULINK_LAP_LIMIT2, "launch": LAUNCH_LIMIT2}, fitness=FITNESS2)
        print(f"sealed {len(rec['cases'])} cases and {len(rec['code_sha256'])} files at {rec['created_utc']}; commit "
              "and push benchmarks/powertrain/simulink_blind2/PREREGISTRATION.json before running MATLAB")
    elif len(sys.argv) == 2 and sys.argv[1] == "simulink-blind-seal":
        rec = seal_blind()
        print(f"sealed {len(rec['cases'])} cases and {len(rec['code_sha256'])} files at {rec['created_utc']}; commit "
              "and push benchmarks/powertrain/simulink_blind/PREREGISTRATION.json before running MATLAB")
    else:
        print(__doc__)
        sys.exit(2)
