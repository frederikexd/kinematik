"""KinematiK's EV and ICE powertrain modules against published GT-SUITE models and Argonne dynamometer data.

    python scripts/gt_suite_benchmark.py                       # the GT-SUITE comparison; writes results.csv
    python scripts/gt_suite_benchmark.py --seal                # also print the SHA-256 of results.csv (blind runs)
    python scripts/gt_suite_benchmark.py argonne               # against Argonne's measured BMW i3
    python scripts/gt_suite_benchmark.py argonne-extract RAW   # re-cut benchmarks/gt_suite/argonne from raw D3 files

Everything the comparison reads is listed with its SHA-256 in benchmarks/gt_suite/MANIFEST.sha256; the script refuses
to run when a file no longer matches, so a reference value cannot drift without the manifest changing in review.

EV (SAE 2021-01-1245, BMW i3, GT-SUITE, 25 degC): suspension.ev_electrical_check.traction_power_w on the EPA cycles,
with regeneration through the drive capped at the i3's 50 kW, and without it (KinematiK's default, an upper bound).
The motor and inverter efficiency is not in the paper: every result is also computed across its assumed band, and the
band is printed next to the nominal value.

ICE (SAE 2012-36-0247, 12 t diesel truck, GT-SUITE, constant speed, no accessories): suspension.ice_powertrain
.fuel_for_lap. The paper gives neither Crr nor its BSFC map as numbers, so both are set from one speed and the other
five are predicted; this is repeated with every speed as the calibration point, and the worst error is graded.

Argonne (D3, 2014 BMW i3 BEV): the same EV functions on the measured speed traces, road load from Argonne's
coast-down targets, auxiliaries measured at standstill, drive and regeneration efficiencies fitted on all graded
segments but the one predicted.

Exit code 0 when every graded row is within the tolerance in references.json.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from suspension import ev_electrical_check as ec  # noqa: E402
from suspension import ice_powertrain as ice  # noqa: E402

BENCH = ROOT / "benchmarks" / "gt_suite"
MANIFEST = BENCH / "MANIFEST.sha256"
G = 9.81


# ---------------------------------------------------------------------------------------------------------------------
#  Provenance
# ---------------------------------------------------------------------------------------------------------------------
def sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_manifest() -> list[str]:
    """Files whose SHA-256 no longer matches MANIFEST.sha256 (empty when everything matches)."""
    bad = []
    for line in MANIFEST.read_text().splitlines():
        if line.strip():
            digest, name = line.split(maxsplit=1)
            f = BENCH / name.strip()
            if not f.exists() or sha256(f) != digest:
                bad.append(name.strip())
    return bad


def _refs():
    return json.loads((BENCH / "references.json").read_text())


def _cycle(name):
    with open(BENCH / name, newline="") as fh:
        rows = list(csv.DictReader(fh))
    return np.array([float(r["time_s"]) for r in rows]), np.array([float(r["speed_ms"]) for r in rows])


def stats(errors) -> dict:
    """Mean absolute error, bias (mean signed error) and worst case of a list of relative errors."""
    e = np.asarray(list(errors), float)
    return {"mae": float(np.mean(np.abs(e))), "bias": float(np.mean(e)), "worst": float(e[np.argmax(np.abs(e))])}


# ---------------------------------------------------------------------------------------------------------------------
#  EV
# ---------------------------------------------------------------------------------------------------------------------
def ev_energy_wh_km(t, v, veh, eta, rho, regen=True, r_pack=0.0):
    """KinematiK's battery energy per km on a speed trace (Wh/km), with the pack's I²R loss, and its cumulative
    energy (Wh)."""
    a = np.gradient(v, t)
    k = 1.0 + veh["axle_inertia_kgm2"] / (veh["tire_radius_m"] ** 2 * veh["mass_kg"])
    _, _, p_pack = ec.traction_power_w(v, a, vehicle_mass_kg=veh["mass_kg"],
                                       drag_cda=veh["cd"] * veh["frontal_area_m2"], crr=veh["crr"], rho_air=rho,
                                       drivetrain_eff=eta, regen_eff=eta if regen else 0.0,
                                       regen_cap_w=veh["regen_cap_kw"] * 1e3, rot_mass_factor=k)
    p_pack, _ = ec.battery_internal_power_w(p_pack, veh["pack_ocv_v"], r_pack)
    e_cum = np.concatenate([[0.0], np.cumsum(0.5 * (p_pack[1:] + p_pack[:-1]) * np.diff(t))]) / 3600
    km = np.trapezoid(v, t) / 1e3
    return float(e_cum[-1] / km), e_cum


def ev_rows(ref=None):
    """(cycle, Wh/km nominal, band low, band high, without regen, GT-SUITE Wh/km)."""
    e = (ref or _refs())["ev"]
    veh, asm = e["vehicle"], e["assumed"]
    d_eff = veh["differential_eff"]
    out = []
    for key, fname in e["cycles"].items():
        if key.startswith("x_"):
            continue
        t, v = _cycle(fname)
        m, r, rp = asm["motor_inverter_eff"], asm["rho_kg_m3"], asm["pack_r_ohm"]
        nominal = ev_energy_wh_km(t, v, veh, d_eff * m["nominal"], r["nominal"], r_pack=rp["nominal"])[0]
        corners = [ev_energy_wh_km(t, v, veh, d_eff * me, rh, r_pack=rr)[0]
                   for me in (m["low"], m["high"]) for rh in (r["low"], r["high"]) for rr in (rp["low"], rp["high"])]
        no_regen = ev_energy_wh_km(t, v, veh, d_eff * m["nominal"], r["nominal"], regen=False,
                                   r_pack=rp["nominal"])[0]
        out.append((key, nominal, min(corners), max(corners), no_regen, float(e["wh_per_km_25c"][key])))
    return out


# ---------------------------------------------------------------------------------------------------------------------
#  ICE
# ---------------------------------------------------------------------------------------------------------------------
def _rho(veh):
    return veh["ambient_bar"] * 1e5 / (287.05 * (veh["ambient_c"] + 273.15))


def _truck(ic, crr):
    veh = ic["vehicle"]
    v = ice.ICEVehicle(mass_kg=veh["mass_kg"], cda_m2=veh["cd"] * veh["frontal_area_m2"], crr=crr, rho=_rho(veh),
                       wheel_inertia_kgm2=0.0, driven_weight_frac=0.7, cg_height_m=0.88, wheelbase_m=4.7, mu=0.9)
    # neither the torque curve nor the gear ratios are in the paper as numbers; at constant speed only the engine's
    # power reserve matters (fuel_for_lap caps wheel power at the curve), so a generous curve and a 6-speed suffice
    eng = ice.EngineMap(rpm=[800, 1200, 1600, 2000, 2400, 2600], torque_nm=[400, 540, 540, 540, 500, 450],
                        redline_rpm=2600, idle_rpm=500, inertia_kgm2=1.7, displacement_cc=4800,
                        name="MaxxForce 4.8H (shape assumed)", source="user")
    gb = ice.Gearbox(primary=1.0, ratios=(6.6, 3.8, 2.3, 1.4, 1.0, 0.79), final_drive=4.1, shift_time_s=0.5,
                     efficiency=veh["gearbox_eff"] * veh["axle_eff"], tire_radius_m=veh["tire_radius_m"],
                     source="ratios assumed")
    return v, eng, gb


_EXTRA_FUELS: dict = {}


def _with_fuels(fn, *args, **kw):
    """Run fn with the benchmark's own fuels added to ice.FUELS, and take them out again afterwards, so the
    module's fuel list is the same before and after (other tests check it)."""
    saved = {k: ice.FUELS.get(k) for k in _EXTRA_FUELS}
    ice.FUELS.update(_EXTRA_FUELS)
    try:
        return fn(*args, **kw)
    finally:
        for k, f in saved.items():
            if f is None:
                ice.FUELS.pop(k, None)
            else:
                ice.FUELS[k] = f


def _diesel(ic):
    veh = ic["vehicle"]
    _EXTRA_FUELS["diesel"] = ice.Fuel("diesel", "Diesel", veh["diesel_density_kg_l"],
                                      ic["assumed"]["diesel_lhv_mj_kg"], 14.5, 2.68, 0.0, "benchmark")


def _cruise(ic, crr, bte, kmh):
    """KinematiK's fuel_for_lap over 1 km at constant speed: (engine kW, km/l)."""
    veh, eng, gb = _truck(ic, crr)
    lap = SimpleNamespace(s=np.linspace(0.0, 1000.0, 51), v=np.full(51, kmh / 3.6))
    r = _with_fuels(ice.fuel_for_lap, lap, eng, gb, veh, fuel_key="diesel", bte=bte, overrun_fuel_g_s=0.0)
    return r.engine_work_mj * 1e3 / r.lap_time_s, r.lap_m / 1e3 / r.fuel_l


def calibrate(ic, kmh):
    """Crr and BTE that make the truck match the paper exactly at one speed."""
    veh = ic["vehicle"]
    k = ic["speeds_kmh"].index(kmh)
    v0 = kmh / 3.6
    p0 = ic["dynamics_power_kw"][k] * 1e3
    eta = veh["gearbox_eff"] * veh["axle_eff"]
    crr = (p0 * eta / v0 - 0.5 * _rho(veh) * veh["cd"] * veh["frontal_area_m2"] * v0 ** 2) / (veh["mass_kg"] * G)
    fuel_kg_s = v0 / 1e3 / ic["km_per_l_without_accessories"][k] * veh["diesel_density_kg_l"]
    return crr, p0 / (fuel_kg_s * ic["assumed"]["diesel_lhv_mj_kg"] * 1e6)


def ice_rows(ref=None, calibrate_at=None):
    """(speed km/h, KinematiK kW, GT-SUITE kW, KinematiK km/l, GT-SUITE km/l), plus the Crr and BTE used."""
    ic = (ref or _refs())["ice"]
    _diesel(ic)
    crr, bte = calibrate(ic, calibrate_at or ic["calibrate_at_kmh"])
    out = [(kmh, *_cruise(ic, crr, bte, kmh), fe_ref)
           for kmh, fe_ref in zip(ic["speeds_kmh"], ic["km_per_l_without_accessories"])]
    out = [(kmh, p, p_ref, fe, fe_ref) for (kmh, p, fe, fe_ref), p_ref in zip(out, ic["dynamics_power_kw"])]
    return out, crr, bte


def ice_leave_one_out(ref=None):
    """Calibrate at every speed in turn and predict the rest: per calibration speed, the worst power and fuel error."""
    ic = (ref or _refs())["ice"]
    res = {}
    for k in ic["speeds_kmh"]:
        rows, _, _ = ice_rows(ref, calibrate_at=k)
        pred = [r for r in rows if r[0] != k]
        res[k] = (stats(p / pr - 1 for _, p, pr, _, _ in pred)["worst"],
                  stats(f / fr - 1 for *_, f, fr in pred)["worst"])
    return res


# ---------------------------------------------------------------------------------------------------------------------
#  More GT-SUITE EV papers: an electric two-wheeler and a converted Toyota Avanza
# ---------------------------------------------------------------------------------------------------------------------
def ev_2w_rows(ref=None) -> dict:
    """Top speed of the two-wheeler from KinematiK's road-load power balance at the motor's 5 kW, and the motor speed
    there from the gear ratio and wheel radius. Every input is in the paper."""
    e = (ref or _refs())["ev_2w"]
    v = e["vehicle"]
    top = ec.fuse_limited_speed(v["motor_kw"] * 1e3, vehicle_mass_kg=v["mass_kg"],
                                drag_cda=v["cd"] * v["frontal_area_m2"], crr=v["crr"], rho_air=v["rho_kg_m3"],
                                drivetrain_eff=v["driveline_eff"])
    rpm = e["top_speed_kmh"] / 3.6 / (2 * np.pi * v["wheel_radius_m"]) * 60 * v["gear_ratio"]
    return {"top_kmh": top * 3.6, "motor_rpm_at_gt_top": float(rpm)}


def ev_avanza_rows(ref=None) -> dict:
    """The single-gear converted Avanza: 0-100 km/h with KinematiK's EV launch (powertrain.engine.simulate_launch),
    top speed from the road-load power balance, and NEDC energy across the assumed rolling resistance, with and
    without regeneration (the paper does not say which)."""
    from powertrain import engine as pe
    e = (ref or _refs())["ev_avanza"]
    v, a = e["vehicle"], e["assumed"]
    cda = v["cd"] * v["frontal_area_m2"]

    def run(crr):
        mot = pe.MotorCurve.from_peak(v["motor_peak_torque_nm"], v["motor_peak_kw"], v["motor_max_rpm"],
                                      eta_motor=v["motor_eff"], voltage_v=v["pack_v"])
        p = pe.DrivetrainParams(motor=mot, tire_radius_m=a["tire_radius_m"], gear_ratio=v["final_drive"],
                                mass_kg=v["mass_kg"], drivetrain_eff=a["driveline_eff"], mu=1.0,
                                driven_weight_frac=0.6, cg_height_m=0.6, wheelbase_m=2.655, cda_m2=cda, crr=crr,
                                rot_inertia_factor=a["rot_inertia_factor"])
        res = pe.simulate_launch(p, distance_m=400.0, t_max_s=40.0)
        i = int(np.argmax(res.v_ms >= 100 / 3.6))
        t100 = float(np.interp(100 / 3.6, res.v_ms[i - 1:i + 1], res.t_s[i - 1:i + 1]))
        top = ec.fuse_limited_speed(v["motor_peak_kw"] * 1e3, vehicle_mass_kg=v["mass_kg"], drag_cda=cda, crr=crr,
                                    rho_air=a["rho_kg_m3"], drivetrain_eff=a["driveline_eff"])
        t, sp = _cycle("nedc.csv")
        eta = v["motor_eff"] * a["driveline_eff"]
        nedc = {}
        for regen in (True, False):
            _, _, pp = ec.traction_power_w(sp, np.gradient(sp, t), vehicle_mass_kg=v["mass_kg"], drag_cda=cda,
                                           crr=crr, rho_air=a["rho_kg_m3"], drivetrain_eff=eta,
                                           regen_eff=eta if regen else 0.0, regen_cap_w=v["motor_peak_kw"] * 1e3,
                                           rot_mass_factor=a["rot_inertia_factor"])
            nedc[regen] = float(np.trapezoid(pp, t) / 3600 / (np.trapezoid(sp, t) / 1e3))
        return t100, top * 3.6, nedc

    c = a["crr"]
    nom = run(c["nominal"])
    corners = [run(c["low"]), run(c["high"])]
    band = [x[2][r] for x in [nom, *corners] for r in (True, False)]
    return {"t_0_100": nom[0], "top_kmh": nom[1], "nedc_regen": nom[2][True], "nedc_no_regen": nom[2][False],
            "nedc_band": (min(band), max(band)), "gt_nedc_wh_km": 1000.0 / e["single_gear"]["nedc_km_per_kwh"]}


# ---------------------------------------------------------------------------------------------------------------------
#  ICE gearing: the 0-402 m time of 25 gear sets (GT-SUITE, Proton Saga)
# ---------------------------------------------------------------------------------------------------------------------
def gear_launch_rows(ref=None) -> dict:
    """KinematiK's simulate_launch over 402 m for every gear set of the paper, with the torque curve scaled once so
    the standard set matches the paper's base time. Graded: each time, and the change from the base set, which is
    what a gearing study is for (the absolute time is mostly set by the calibration)."""
    from scipy.optimize import brentq
    from scipy.stats import spearmanr
    g = (ref or _refs())["ice_gears"]
    v, a = g["vehicle"], g["assumed"]
    veh = ice.ICEVehicle(mass_kg=v["mass_kg"], cda_m2=v["cd"] * a["frontal_area_m2"], cla_m2=0.0, crr=v["crr"],
                         rho=1.2, mu=1.0, driven_weight_frac=0.6, cg_height_m=0.55, wheelbase_m=2.465,
                         wheel_inertia_kgm2=3.2)

    def t402(gears, scale, schedule=True):
        eng = ice.EngineMap(rpm=a["torque_curve_rpm"], torque_nm=[x * scale for x in a["torque_curve_nm"]],
                            redline_rpm=a["redline_rpm"], idle_rpm=750, inertia_kgm2=0.12, displacement_cc=1332)
        gb = ice.Gearbox(primary=1.0, ratios=tuple(gears), final_drive=a["final_drive"],
                         shift_time_s=a["shift_time_s"], efficiency=a["gearbox_eff"],
                         tire_radius_m=v["tire_radius_m"], source="paper",
                         upshift_kmh=tuple(a["upshift_kmh"]) if schedule else None)
        return ice.simulate_launch(eng, gb, veh, launch_rpm=a["launch_rpm"], distance_m=402.0, t_max_s=60.0).time_s

    ref_t = np.array(g["time_402m_s"])
    d_ref = ref_t - g["base_402m_s"]
    out = {}
    for schedule in (True, False):
        scale = brentq(lambda k: t402(g["base_gears"], k, schedule) - g["base_402m_s"], 0.3, 3.0, xtol=1e-6)
        ours = np.array([t402(s, scale, schedule) for s in g["gear_sets"]])
        out[schedule] = {"scale": float(scale), "ours": ours, "rank": float(spearmanr(ours, ref_t)[0]),
                         "sensitivity": float(np.polyfit(d_ref, ours - g["base_402m_s"], 1)[0]),
                         "spread_ours": float(np.ptp(ours)), "best_ours": int(np.argmin(ours))}
    return {**out[True], "ref": ref_t, "spread_ref": float(np.ptp(ref_t)), "best_ref": int(np.argmin(ref_t)),
            "max_force": out[False]}


# ---------------------------------------------------------------------------------------------------------------------
#  Argonne D3 dynamometer data (measured BMW i3)
# ---------------------------------------------------------------------------------------------------------------------
ARGONNE = BENCH / "argonne"
_SPEED_UNITS = {"mph": 0.44704, "km/h": 1 / 3.6, "kph": 1 / 3.6, "m/s": 1.0}


def read_d3(path) -> dict:
    """Time (s), speed (m/s), battery power (W, discharge positive) and test phase from an Argonne D3 text export
    (tab- or comma-separated, units in brackets after each channel name, e.g. "Dyno_Speed[mph]")."""
    lines = Path(path).read_text(errors="replace").splitlines()
    h = next(i for i, ln in enumerate(lines) if re.match(r"\s*\"?time", ln, re.I))
    sep = "\t" if "\t" in lines[h] else ","
    names = [c.strip().strip('"') for c in lines[h].split(sep)]
    rows = []
    for ln in lines[h + 1:]:
        try:
            rows.append([float(c) if c.strip() else np.nan for c in ln.split(sep)[:len(names)]])
        except ValueError:
            continue
    arr = np.array([r + [np.nan] * (len(names) - len(r)) for r in rows], float)

    def col(pattern, exclude=None, required=True):
        for j, n in enumerate(names):
            if re.search(pattern, n, re.I) and not (exclude and re.search(exclude, n, re.I)):
                unit = (re.search(r"\[(.*?)\]", n) or [None, ""])[1].lower()
                return arr[:, j], unit
        if required:
            raise ValueError(f"{path}: no channel matching {pattern!r}; channels are {names}")
        return np.zeros(len(arr)), ""

    t, _ = col(r"^time")
    spd, unit = col(r"(dyno|veh|vehicle).*sp(ee)?d", r"wheel|motor|eng|whl")
    volts, _ = col(r"(hv|batt|bat|pack).*volt", r"12\s*v|lv|aux")
    amps, _ = col(r"(hv|batt|bat|pack).*curr", r"12\s*v|lv|aux")
    phase, _ = col(r"phase", required=False)
    ok = np.isfinite(t) & np.isfinite(spd) & np.isfinite(volts) & np.isfinite(amps)
    v = spd[ok] * _SPEED_UNITS.get(unit, 0.44704)
    p = volts[ok] * amps[ok]
    accel = np.gradient(v, t[ok]) > 0.3                 # discharge counts positive, whichever sign the logger used
    if accel.any() and np.median(p[accel]) < 0:
        p = -p
    return {"time": t[ok], "speed": v, "power": p, "phase": phase[ok]}


def segment_name(seg) -> str:
    ph = seg["phases"]
    return f"{seg['test']}_{seg['cycle']}_ph" + (f"{ph[0]}" if len(ph) == 1 else f"{ph[0]}-{ph[-1]}")


def extract(raw_dir, ref=None) -> list[tuple]:
    """Cut every segment of references.json out of the raw D3 files in raw_dir ("<test> Test Data.txt") and write it
    as 1 s block means (energy and distance are kept exactly). Returns (segment, extracted Wh, Argonne's Wh)."""
    ARGONNE.mkdir(exist_ok=True)
    out, cache = [], {}
    for seg in (ref or _refs())["argonne"]["segments"]:
        f = Path(raw_dir) / f"{seg['test']} Test Data.txt"
        log = cache.setdefault(seg["test"], read_d3(f))
        sel = np.isin(np.round(log["phase"], 6), seg["phases"])
        t, v, p = log["time"][sel], log["speed"][sel], log["power"][sel]
        dt = float(np.median(np.diff(t)))
        n = int(round(1.0 / dt))
        m = len(t) // n * n
        vb, pb = v[:m].reshape(-1, n).mean(1), p[:m].reshape(-1, n).mean(1)
        with open(ARGONNE / f"{segment_name(seg)}.csv", "w", newline="") as fh:
            w = csv.writer(fh, lineterminator="\n")
            w.writerow(["time_s", "speed_ms", "batt_power_w"])
            w.writerows((k, f"{a:.4f}", f"{b:.1f}") for k, (a, b) in enumerate(zip(vb, pb)))
        out.append((segment_name(seg), float(np.sum(p) * dt / 3600), float(seg["argonne_wh"])))
    return out


def load_segment(seg) -> dict:
    with open(ARGONNE / f"{segment_name(seg)}.csv", newline="") as fh:
        rows = list(csv.DictReader(fh))
    return {k: np.array([float(r[c]) for r in rows]) for k, c in
            (("time", "time_s"), ("speed", "speed_ms"), ("power", "batt_power_w"))}


def coastdown_road_load(arg) -> tuple[float, float]:
    """Crr and CdA (at 1.184 kg/m³) of KinematiK's road-load form fitted to Argonne's coast-down targets."""
    m = arg["test_weight_lb"] * 0.45359237
    v = np.linspace(0.0, 36.0, 200)
    lbf, mph = 4.4482216, 0.44704
    f = arg["target_a_lb"] * lbf + arg["target_b_lb_per_mph"] * lbf * v / mph + arg["target_c_lb_per_mph2"] * lbf * (v / mph) ** 2
    c2, c0 = np.polyfit(v * v, f, 1)
    return float(c0 / (m * G)), float(2 * c2 / 1.184)


def measured_wh_km(log) -> tuple[float, np.ndarray]:
    e = np.cumsum(log["power"]) / 3600               # 1 s block means
    return float(e[-1] / (np.sum(log["speed"]) / 1e3)), e


def model_wh_km(log, veh, crr, cda, eta_d, eta_r, aux_w, r_pack) -> tuple[float, np.ndarray]:
    """KinematiK on a measured speed trace with drive and regeneration efficiencies, auxiliaries and pack loss."""
    t, v = log["time"], log["speed"]
    m = veh["mass_kg"]
    k = 1.0 + veh["axle_inertia_kgm2"] / (veh["tire_radius_m"] ** 2 * m)
    _, _, p = ec.traction_power_w(v, np.gradient(v, t), vehicle_mass_kg=m, drag_cda=cda, crr=crr, rho_air=1.184,
                                  drivetrain_eff=eta_d, regen_eff=eta_r, regen_cap_w=veh["regen_cap_kw"] * 1e3,
                                  rot_mass_factor=k, aux_power_w=aux_w)
    p, _ = ec.battery_internal_power_w(p, veh["pack_ocv_v"], r_pack)
    e = np.cumsum(p) / 3600
    return float(e[-1] / (np.sum(v) / 1e3)), e


def standstill_power_w(logs) -> float:
    """The auxiliaries' draw: median battery power while the car stands still (measured, not fitted)."""
    return float(np.median(np.concatenate([lg["power"][lg["speed"] < 0.05] for lg in logs])))


def fit_drive(logs, veh, crr, cda, aux_w, r_pack) -> tuple[float, float]:
    """Drive and regeneration efficiencies that best match a set of measured logs (least squares on Wh/km)."""
    from scipy.optimize import least_squares
    meas = np.array([measured_wh_km(lg)[0] for lg in logs])
    res = least_squares(lambda x: np.array([model_wh_km(lg, veh, crr, cda, x[0], x[1], aux_w, r_pack)[0]
                                            for lg in logs]) / meas - 1, x0=[0.9, 0.7], bounds=([0.5, 0.0], [1.0, 1.0]))
    return float(res.x[0]), float(res.x[1])


def argonne_rows(ref=None) -> dict:
    """Leave one out over the graded segments: drive and regeneration efficiencies are fitted on all graded segments
    but one, the auxiliaries are measured at standstill on the same training set, and the left-out segment is
    predicted. Ungraded segments are predicted from a fit on every graded one."""
    ref = ref or _refs()
    e, arg = ref["ev"], ref["argonne"]
    veh, r_pack = e["vehicle"], e["assumed"]["pack_r_ohm"]["nominal"]
    crr, cda = coastdown_road_load(arg)
    segs = arg["segments"]
    logs = {segment_name(s): load_segment(s) for s in segs}
    graded = [segment_name(s) for s in segs if s["graded"]]
    out = {"crr": crr, "cda": cda, "graded": [], "reported": []}
    for name in graded:
        train = [logs[n] for n in graded if n != name]
        aux = standstill_power_w(train)
        eta_d, eta_r = fit_drive(train, veh, crr, cda, aux, r_pack)
        meas, em = measured_wh_km(logs[name])
        ours, eo = model_wh_km(logs[name], veh, crr, cda, eta_d, eta_r, aux, r_pack)
        rms = float(np.sqrt(np.mean((eo - em) ** 2)) / em[-1])
        out["graded"].append((name, meas, ours, ours / meas - 1, rms, eta_d, eta_r, aux))
    aux = standstill_power_w([logs[n] for n in graded])
    eta_d, eta_r = fit_drive([logs[n] for n in graded], veh, crr, cda, aux, r_pack)
    out["all"] = (eta_d, eta_r, aux)
    for s in segs:
        if not s["graded"]:
            name = segment_name(s)
            meas, _ = measured_wh_km(logs[name])
            ours, _ = model_wh_km(logs[name], veh, crr, cda, eta_d, eta_r, aux, r_pack)
            out["reported"].append((name, s["ambient_c"], s["start"], s["hvac"], meas, ours, ours / meas - 1))
    # the GT-SUITE paper and KinematiK on the paper's own inputs, against the measured 25 degC hot-start means
    by_cycle = {}
    for s in segs:
        if s["graded"]:
            by_cycle.setdefault(s["cycle"], []).append(measured_wh_km(logs[segment_name(s)])[0])
    out["cycle_means"] = {c: float(np.mean(v)) for c, v in by_cycle.items()}
    return out


# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
#  Argonne D3, ICE: 2019 Chevrolet Blazer (measured)
# ---------------------------------------------------------------------------------------------------------------------
ARGONNE_ICE = BENCH / "argonne_ice"
_TDMS = {"bag": "Exhaust_Bag", "speed": "Dyno_Spd", "fuel": "Eng_FuelFlow_Direct2", "rpm": "Eng_Spd_ECM",
         "torque": "Eng_torque_TCM"}


def _read_tdms(path) -> dict:
    from nptdms import TdmsFile                       # only needed to re-extract from Argonne's raw files
    g = TdmsFile.read(path).groups()[0]
    return {k: np.asarray(g[c][:], float) for k, c in _TDMS.items()}


def extract_ice(raw_dir, ref=None) -> list[tuple]:
    """Cut the Blazer segments out of Argonne's TDMS files ("<test> Analysis.tdms") as 1 s block means, and the
    full-load torque curve (highest logged torque per 500 rpm band) out of the wide-open-throttle test."""
    ice_ref = (ref or _refs())["argonne_ice"]
    ARGONNE_ICE.mkdir(exist_ok=True)
    cache, out = {}, []
    for seg in ice_ref["segments"]:
        d = cache.setdefault(seg["test"], _read_tdms(Path(raw_dir) / f"{seg['test']} Analysis.tdms"))
        sel = np.isin(np.round(d["bag"]), seg["phases"])
        n = sel.sum() // 10 * 10
        cols = {k: d[k][sel][:n].reshape(-1, 10).mean(1) for k in ("speed", "fuel", "rpm", "torque")}
        with open(ARGONNE_ICE / f"{segment_name(seg)}.csv", "w", newline="") as fh:
            w = csv.writer(fh, lineterminator="\n")
            w.writerow(["time_s", "speed_ms", "fuel_g_s", "engine_rpm", "engine_torque_nm"])
            for k in range(len(cols["speed"])):
                w.writerow([k, f"{cols['speed'][k] * 0.44704:.4f}", f"{cols['fuel'][k]:.4f}", f"{cols['rpm'][k]:.0f}",
                            f"{cols['torque'][k]:.1f}"])
        out.append((segment_name(seg), float(cols["fuel"].sum()), float(seg["bag_fuel_g"])))
    cal = ice_ref["calibration"]
    d = _read_tdms(Path(raw_dir) / f"{cal['test']} Analysis.tdms")
    sel = np.isin(np.round(d["bag"]), cal["phases"])
    n = sel.sum() // 10 * 10
    cols = {k: d[k][sel][:n].reshape(-1, 10).mean(1) for k in ("speed", "fuel", "rpm", "torque")}
    phase = np.round(d["bag"][sel][:n].reshape(-1, 10).max(1))
    with open(ARGONNE_ICE / f"{cal['test']}_calibration.csv", "w", newline="") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(["time_s", "speed_ms", "fuel_g_s", "engine_rpm", "engine_torque_nm", "phase"])
        for k in range(len(phase)):
            w.writerow([k, f"{cols['speed'][k] * 0.44704:.4f}", f"{cols['fuel'][k]:.4f}", f"{cols['rpm'][k]:.0f}",
                        f"{cols['torque'][k]:.1f}", f"{phase[k]:.0f}"])
    d = _read_tdms(Path(raw_dir) / f"{ice_ref['torque_curve_test']} Analysis.tdms")
    with open(ARGONNE_ICE / "torque_curve.csv", "w", newline="") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(["rpm", "torque_nm"])
        for lo in range(750, 7250, 500):
            s = (d["rpm"] >= lo) & (d["rpm"] < lo + 500)
            if s.any():
                w.writerow([lo + 250, f"{np.nanmax(d['torque'][s]):.1f}"])
    return out


def _load_csv(path) -> dict:
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    return {c: np.array([float(r[c]) for r in rows]) for c in rows[0]}


def _blazer(ice_ref):
    m = ice_ref["test_weight_lb"] * 0.45359237
    crr, cda = coastdown_road_load(ice_ref)
    tc = _load_csv(ARGONNE_ICE / "torque_curve.csv")
    veh = ice.ICEVehicle(mass_kg=m, cda_m2=cda, crr=crr, rho=1.184, wheel_inertia_kgm2=0.0, cla_m2=0.0, mu=1.0,
                         cg_height_m=0.7, wheelbase_m=2.86, driven_weight_frac=0.6)
    eng = ice.EngineMap(rpm=list(tc["rpm"]), torque_nm=list(tc["torque_nm"]), redline_rpm=7000, idle_rpm=650,
                        inertia_kgm2=0.2, displacement_cc=3649, name="GM LGX 3.6 L V6", source="dyno")
    gb = ice.Gearbox(primary=1.0, ratios=tuple(ice_ref["gear_ratios"]), final_drive=ice_ref["final_drive"],
                     shift_time_s=0.3, efficiency=0.90, tire_radius_m=ice_ref["tire_radius_m"], source="9T65")
    lhv = ice_ref["fuel_net_hv_btu_lbm"] * 2.326e-3                        # BTU/lbm -> MJ/kg
    _EXTRA_FUELS["cert_gasoline"] = ice.Fuel("cert_gasoline", "Certification gasoline", ice_ref["fuel_density_g_ml"],
                                             lhv, 14.7, 2.31, 0.0, "Argonne")
    return veh, eng, gb, crr, cda


def _rates(logs, veh) -> tuple[float, float]:
    """Measured idle and overrun fuel rates (g/s): mean logged flow standing still (stop-start included), and while
    braking."""
    idle, over = [], []
    for lg in logs:
        v, f = lg["speed_ms"], lg["fuel_g_s"]
        p = (veh.mass_kg * np.gradient(v) + veh.resistance_n(0.0) + 0.5 * veh.rho * veh.cda_m2 * v * v) * v
        idle.append(f[v < 0.1])
        over.append(f[(v > 3) & (p < -2000)])
    return float(np.mean(np.concatenate(idle))), float(np.mean(np.concatenate(over)))


def ice_fuel_g(log, parts, bte, idle_g_s, over_g_s, willans=None, gearbox=None) -> float:
    """KinematiK's fuel_for_lap on a measured speed trace. fuel_for_lap works along distance, so time standing
    still is added at the measured idle rate. With a Willans line the logged engine speed is passed in, since a
    road car's automatic picks its gears for economy, not for drive force."""
    veh, eng, gb = parts[:3]
    gb = gearbox or gb
    v = log["speed_ms"]
    moving = v >= 0.1
    s = np.cumsum(v)[moving]
    keep = np.concatenate([[True], np.diff(s) > 1e-6])
    lap = SimpleNamespace(s=s[keep], v=v[moving][keep], rpm=log["engine_rpm"][moving][keep])
    r = _with_fuels(ice.fuel_for_lap, lap, eng, gb, veh, fuel_key="cert_gasoline", bte=bte, overrun_fuel_g_s=over_g_s,
                    willans=willans)
    return r.fuel_kg * 1e3 + idle_g_s * float(np.sum(~moving))


def calibrate_willans(ice_ref, parts) -> tuple:
    """From the stairs test alone: the Willans line (engine torque, speed and fuel flow, 5 s means) and the driveline
    efficiency against the coast-down road load (level-road steady speeds: road-load power over brake power)."""
    veh = parts[0]
    cal = _load_csv(ARGONNE_ICE / f"{ice_ref['calibration']['test']}_calibration.csv")
    n = len(cal["time_s"]) // 5 * 5
    b = {k: c[:n].reshape(-1, 5).mean(1) for k, c in cal.items()}
    lhv = ice_ref["fuel_net_hv_btu_lbm"] * 2.326e-3
    wl = ice.fit_willans(b["engine_torque_nm"], b["engine_rpm"], b["fuel_g_s"], parts[1].displacement_cc, lhv,
                         source=f"Argonne D3 test {ice_ref['calibration']['test']}")
    v = b["speed_ms"]
    acc = np.gradient(v, 5.0)
    p_brake = b["engine_torque_nm"] * b["engine_rpm"] * np.pi / 30
    steady = (b["phase"] == 1) & (v > 3) & (np.abs(acc) < 0.05) & (p_brake > 2e3)
    p_road = np.array([veh.resistance_n(x) for x in v[steady]]) * v[steady]
    return wl, float(np.sum(p_road) / np.sum(p_brake[steady]))


def argonne_ice_rows(ref=None) -> dict:
    """Two models on the graded Blazer segments.
    Willans (graded): the Willans line and driveline efficiency from the stairs test alone, idle and overrun fuel
    measured on the other segments; each segment predicted blind.
    One BTE (reported, for comparison): fitted on the other segments, the left-out one predicted.
    Also the engine's own efficiency against its power, from the logs."""
    from dataclasses import replace
    from scipy.optimize import minimize_scalar
    ice_ref = (ref or _refs())["argonne_ice"]
    parts = _blazer(ice_ref)
    wl, eta_drv = calibrate_willans(ice_ref, parts)
    gb_w = replace(parts[2], efficiency=eta_drv)
    segs = ice_ref["segments"]
    logs = {segment_name(s): _load_csv(ARGONNE_ICE / f"{segment_name(s)}.csv") for s in segs}
    graded = [s for s in segs if s["graded"]]

    def fit_bte(train):
        idle, over = _rates([logs[segment_name(s)] for s in train], parts[0])
        b = minimize_scalar(lambda b: sum((ice_fuel_g(logs[segment_name(s)], parts, b, idle, over) / s["bag_fuel_g"]
                                           - 1) ** 2 for s in train), bounds=(0.1, 0.5), method="bounded").x
        return float(b), idle, over

    out = {"crr": parts[3], "cda": parts[4], "willans": wl, "driveline_eff": eta_drv, "graded": [], "bte": [],
           "reported": []}
    for s in graded:
        name, meas = segment_name(s), s["bag_fuel_g"]
        others = [x for x in graded if x is not s]
        idle, over = _rates([logs[segment_name(x)] for x in others], parts[0])
        ours = ice_fuel_g(logs[name], parts, 0.3, idle, over, willans=wl, gearbox=gb_w)
        out["graded"].append((name, meas, ours, ours / meas - 1))
        out.setdefault("meter", []).append((name, float(logs[name]["fuel_g_s"].sum()), ours))
        b, idle, over = fit_bte(others)
        ours_b = ice_fuel_g(logs[name], parts, b, idle, over)
        out["bte"].append((name, meas, ours_b, ours_b / meas - 1, b))
    idle, over = _rates([logs[segment_name(s)] for s in graded], parts[0])
    out["rates"] = (idle, over)
    for s in segs:
        if not s["graded"]:
            ours = ice_fuel_g(logs[segment_name(s)], parts, 0.3, idle, over, willans=wl, gearbox=gb_w)
            out["reported"].append((segment_name(s), s.get("note", ""), s["bag_fuel_g"], ours,
                                    ours / s["bag_fuel_g"] - 1))
    lhv = ice_ref["fuel_net_hv_btu_lbm"] * 2.326                            # BTU/lbm -> J/g
    p = np.concatenate([lg["engine_torque_nm"] * lg["engine_rpm"] * np.pi / 30 for lg in logs.values()])
    f = np.concatenate([lg["fuel_g_s"] for lg in logs.values()])
    ok = (f > 0.2) & (p > 1e3)
    out["bte_by_power"] = [(lo, hi, float(np.median(p[ok][(p[ok] >= lo * 1e3) & (p[ok] < hi * 1e3)]
                                                    / (f[ok][(p[ok] >= lo * 1e3) & (p[ok] < hi * 1e3)] * lhv))))
                           for lo, hi in ((1, 5), (5, 10), (10, 20), (20, 40), (40, 200))]
    return out


def report_argonne_ice(ref, limit) -> int:
    r = argonne_ice_rows(ref)
    wl = r["willans"]
    print(f"ICE 2019 Chevrolet Blazer 3.6 L V6 against Argonne D3 (25 degC, hot start); coast-down road load: "
          f"Crr {r['crr']:.4f}, CdA {r['cda']:.3f} m2; reference: Argonne's bag fuel")
    print(f"  Willans line from the stairs test alone: eta_indicated {wl.eta_indicated:.3f}, FMEP {wl.fmep_kpa:.0f} kPa "
          f"+ {wl.fmep_kpa_per_krpm:.1f} kPa/krpm; driveline efficiency {r['driveline_eff']:.3f}")
    print(f"  {'segment':22} {'measured g':>10} {'Willans':>9} {'error':>7}    {'one BTE':>9} {'error':>7}")
    for (name, meas, ours, err), (_, _, ob, eb, _) in zip(r["graded"], r["bte"]):
        print(f"  {name:22} {meas:10.1f} {ours:9.1f} {err:+7.1%} {'PASS' if abs(err) <= limit else 'FAIL'}"
              f" {ob:9.1f} {eb:+7.1%}")
    for label, rows in (("Willans", r["graded"]), ("one BTE", r["bte"])):
        s = stats(g[3] for g in rows)
        print(f"  {label:8} MAE {s['mae']:.1%}, bias {s['bias']:+.1%}, worst {s['worst']:+.1%}")
    s = stats(ours / m - 1 for _, m, ours in r["meter"])
    print(f"  Willans against the logged fuel meter it was calibrated with (bag reads 1.5-3.8 % higher): "
          f"MAE {s['mae']:.1%}, bias {s['bias']:+.1%}, worst {s['worst']:+.1%}")
    idle, over = r["rates"]
    print(f"\n  idle {idle:.3f} g/s, overrun {over:.3f} g/s (measured on the graded segments)")
    for name, note, meas, ours, err in r["reported"]:
        print(f"    not graded: {name:22} {meas:7.1f} g  Willans {ours:7.1f} ({err:+.1%})  {note}")
    print("\n  the engine's own efficiency from the logs (torque x speed / fuel power), by engine power:")
    for lo, hi, e in r["bte_by_power"]:
        print(f"    {lo:3}-{hi:3} kW  BTE {e:.3f}")
    return 0 if all(abs(g[3]) <= limit for g in r["graded"]) else 1


# ---------------------------------------------------------------------------------------------------------------------
#  Formula Student Korea 2023 endurance (Team A-FA): the pack model at Formula Student scale
# ---------------------------------------------------------------------------------------------------------------------
def read_fsk(path) -> dict:
    """Inverter DC bus voltage and current and the BMS state of charge, on one time base (s from midnight), from the
    A-FA telemetry msgpack (a list of records with datetime, key and parsed fields)."""
    import datetime as dt
    import msgpack
    recs = msgpack.unpackb(Path(path).read_bytes(), raw=False)
    t0 = dt.datetime(2000, 1, 1)

    def series(key, field):
        rows = []
        for r in recs:
            if r.get("key") == key and r.get("parsed"):
                d = dt.datetime.strptime(r["datetime"], "%Y-%m-%d %H:%M:%S.%f")
                rows.append(((d - dt.datetime.combine(d.date(), t0.time())).total_seconds(), r["parsed"][field]))
        rows.sort()
        return np.array([a for a, _ in rows]), np.array([b for _, b in rows], float)

    t, v = series("CAN_INV_VOLTAGE", "dc_bus_voltage")
    ti, i = series("CAN_INV_CURRENT", "dc_bus_current")
    ts, soc = series("CAN_BMS_CORE", "soc")
    out = {"time": t, "volts": v, "amps": np.interp(t, ti, i), "soc": np.interp(t, ts, soc)}
    rows = []                                           # the BMS's hottest and coolest cell, 1 degC steps
    for r in recs:
        if r.get("key") == "CAN_BMS_TEMP" and r.get("parsed"):
            d = dt.datetime.strptime(r["datetime"], "%Y-%m-%d %H:%M:%S.%f")
            tt = r["parsed"]["temperature"]
            rows.append(((d - dt.datetime.combine(d.date(), t0.time())).total_seconds(),
                         0.5 * (tt["max"]["value"] + tt["min"]["value"]) + 273.15))
    if rows:
        rows.sort()
        out["temp_time"] = np.array([a for a, _ in rows])
        out["temp_k"] = np.array([b for _, b in rows], float)
    return out


def fsk_rows(log, ref=None) -> dict:
    """Fit the pack on stint 1, predict stint 2 with KinematiK's battery_internal_power_w."""
    fs = (ref or _refs())["fsk2023"]
    t, v, i, soc = log["time"], log["volts"], log["amps"], log["soc"]
    live = v > 0.5 * np.max(v)                          # the high-voltage bus is up
    gaps = np.where(np.diff(t) > fs["stint_gap_s"])[0]
    stint = np.zeros(len(t), int)
    for g in gaps:
        stint[g + 1:] += 1
    s1, s2 = live & (stint == 0), live & (stint == 1)
    x = np.column_stack([np.ones(s1.sum()), soc[s1], -i[s1]])
    (a, b, r_pack), *_ = np.linalg.lstsq(x, v[s1], rcond=None)
    rest = s2 & (np.abs(i) < fs["rest_current_a"])
    bins = np.round(soc[rest])
    levels = np.unique(bins)
    ocv_rest = np.array([np.median(v[rest][bins == u]) for u in levels])
    ocv = np.interp(soc[s2], levels, ocv_rest)
    p = v[s2] * i[s2]
    p_int, i_pred = ec.battery_internal_power_w(p, ocv, r_pack)
    v_pred = ocv - i_pred * r_pack
    load = i[s2] > fs["loaded_current_a"]
    dt2 = np.gradient(t[s2])
    sag_m, sag_p = float(np.mean((ocv - v[s2])[load])), float(np.mean((ocv - v_pred)[load]))
    loss_m = float(np.sum(np.maximum((ocv - v[s2]) * i[s2], 0.0) * dt2) / 3600)
    loss_p = float(np.sum((p_int - p) * dt2) / 3600)
    r2 = -np.polyfit(i[s2], v[s2] - ocv, 1)[0]
    return {"r_stint1": float(r_pack), "r_stint2": float(r2), "ocv_fit": (float(a), float(b)),
            "sag_measured": sag_m, "sag_predicted": sag_p, "loss_measured_wh": loss_m, "loss_predicted_wh": loss_p,
            "v_rms_loaded": float(np.sqrt(np.mean((v_pred - v[s2])[load] ** 2))),
            "i_rms": float(np.sqrt(np.mean((i_pred - i[s2]) ** 2))), "n_loaded": int(load.sum()),
            "soc_range": (float(soc[s1].max()), float(soc[s2].min())), "peak_kw": float(np.max(p) / 1e3),
            "energy_kwh": float(np.sum(v[live] * i[live] * np.gradient(t)[live]) / 3.6e6)}


def fsk_thermal_rows(log, ref=None) -> dict:
    """Pack temperature, measured: the mean of the BMS's hottest and coolest cell. KinematiK's lumped pack
    (pack_temperature_along: C dT/dt = I² R − hA (T − T_ambient), R the pack resistance fitted on stint 1, the ambient
    the pack's temperature at the start) has its heat capacity C and cooling hA fitted on stint 1 alone; stint 2 is
    then predicted from its own current and its first temperature reading."""
    from scipy.optimize import minimize
    fs = (ref or _refs())["fsk2023"]
    r_pack = fsk_rows(log, ref)["r_stint1"]
    t, i, tt, temp = log["time"], log["amps"], log["temp_time"], log["temp_k"]
    gaps = np.where(np.diff(t) > fs["stint_gap_s"])[0]
    s1 = (t >= t[0]) & (t <= t[gaps[0]])
    s2 = t > t[gaps[0] + 1] - 1e-9
    m1 = (tt >= t[s1][0]) & (tt <= t[s1][-1])
    m2 = (tt >= t[s2][0]) & (tt <= t[s2][-1])
    ambient = float(temp[m1][0])

    def model(c, ha, sel, start):
        temps = ec.pack_temperature_along(i[sel], t[sel], r_pack_ohm=r_pack, heat_capacity_j_k=c, temp0_k=start,
                                          cooling_w_k=ha, ambient_k=ambient)
        return temps

    def cost(x):
        c, ha = np.exp(x)
        return float(np.mean((np.interp(tt[m1], t[s1], model(c, ha, s1, ambient)) - temp[m1]) ** 2))

    fit = minimize(cost, np.log([20e3, 2.0]), method="Nelder-Mead", options={"xatol": 1e-6, "fatol": 1e-10})
    c, ha = (float(x) for x in np.exp(fit.x))
    start2 = float(temp[m2][0])
    pred2 = np.interp(tt[m2], t[s2], model(c, ha, s2, start2))
    return {"r_pack": r_pack, "heat_capacity_j_k": c, "cooling_w_k": ha, "ambient_k": ambient,
            "fit_rms_k": float(np.sqrt(fit.fun)), "rise_measured_k": float(temp[m2][-1] - start2),
            "rise_predicted_k": float(pred2[-1] - start2), "rms_k": float(np.sqrt(np.mean((pred2 - temp[m2]) ** 2))),
            "max_abs_k": float(np.max(np.abs(pred2 - temp[m2]))), "n": int(m2.sum())}


def report_fsk(path, ref, limit) -> int:
    fs = ref["fsk2023"]
    if sha256(path) != fs["file_sha256"]:
        print(f"{path} is not the pinned A-FA dataset (SHA-256 differs from references.json): results would not "
              "be comparable")
        return 2
    r = fsk_rows(read_fsk(path), ref)
    e_sag = r["sag_predicted"] / r["sag_measured"] - 1
    e_loss = r["loss_predicted_wh"] / r["loss_measured_wh"] - 1
    print("EV pack at Formula Student scale: Team A-FA, Formula Student Korea 2023 endurance (measured)")
    print(f"  {r['energy_kwh']:.2f} kWh over two stints, state of charge {r['soc_range'][0]:.0f} -> "
          f"{r['soc_range'][1]:.0f} %, peak {r['peak_kw']:.1f} kW")
    print(f"  stint 1 fit: pack resistance {r['r_stint1']:.4f} ohm (stint 2 on its own: {r['r_stint2']:.4f})")
    print(f"  stint 2, predicted by battery_internal_power_w ({r['n_loaded']} samples above "
          f"{fs['loaded_current_a']:.0f} A):")
    print(f"    voltage sag under load  measured {r['sag_measured']:.2f} V  KinematiK {r['sag_predicted']:.2f} V  "
          f"{e_sag:+.1%} {'PASS' if abs(e_sag) <= limit else 'FAIL'}")
    print(f"    I2R loss over stint 2   measured {r['loss_measured_wh']:.1f} Wh  KinematiK "
          f"{r['loss_predicted_wh']:.1f} Wh  {e_loss:+.1%} {'PASS' if abs(e_loss) <= limit else 'FAIL'}")
    print(f"    loaded terminal voltage RMS error {r['v_rms_loaded']:.2f} V; current RMS error {r['i_rms']:.2f} A")
    th = fsk_thermal_rows(read_fsk(path), ref)
    e_rise = th["rise_predicted_k"] / th["rise_measured_k"] - 1
    print(f"  pack temperature (BMS, mean of hottest and coolest cell, 1 degC steps); stint 1 fit: heat capacity "
          f"{th['heat_capacity_j_k'] / 1e3:.1f} kJ/K, cooling {th['cooling_w_k']:.2f} W/K (RMS {th['fit_rms_k']:.2f} K)")
    print(f"    stint 2 rise            measured {th['rise_measured_k']:.2f} K  KinematiK {th['rise_predicted_k']:.2f} K  "
          f"{e_rise:+.1%} {'PASS' if abs(e_rise) <= limit else 'FAIL'}; RMS {th['rms_k']:.2f} K, worst "
          f"{th['max_abs_k']:.2f} K over {th['n']} readings")
    ok = abs(e_sag) <= limit and abs(e_loss) <= limit and abs(e_rise) <= limit
    return 0 if ok else 1


# ---------------------------------------------------------------------------------------------------------------------
#  Pre-registration: papers graded only by predictions sealed before their results were entered
# ---------------------------------------------------------------------------------------------------------------------
PAPERS = BENCH / "papers.json"
SEALED = BENCH / "preregistered"
REGISTER = BENCH / "preregistrations.json"


def _canonical(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _papers() -> dict:
    return json.loads(PAPERS.read_text()) if PAPERS.exists() else {}


def _register() -> list:
    return json.loads(REGISTER.read_text()) if REGISTER.exists() else []


def _git(*args) -> str:
    import subprocess
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()


def predict_ev_cycle(inp) -> dict:
    """Battery energy per km on standard cycles. Inputs: mass_kg, cd, frontal_area_m2, crr, rho_kg_m3,
    tire_radius_m, axle_inertia_kgm2, drive_eff, regen_eff, regen_cap_kw, aux_w, pack_ocv_v, pack_r_ohm, cycles
    (names of the cycle files in benchmarks/gt_suite, e.g. "udds.csv")."""
    out = {}
    for name in inp["cycles"]:
        t, v = _cycle(name)
        m = inp["mass_kg"]
        k = 1.0 + inp.get("axle_inertia_kgm2", 0.0) / (inp["tire_radius_m"] ** 2 * m)
        _, _, p = ec.traction_power_w(v, np.gradient(v, t), vehicle_mass_kg=m,
                                      drag_cda=inp["cd"] * inp["frontal_area_m2"], crr=inp["crr"],
                                      rho_air=inp["rho_kg_m3"], drivetrain_eff=inp["drive_eff"],
                                      regen_eff=inp.get("regen_eff", 0.0),
                                      regen_cap_w=inp.get("regen_cap_kw", 1e9) * 1e3, rot_mass_factor=k,
                                      aux_power_w=inp.get("aux_w", 0.0))
        p, _ = ec.battery_internal_power_w(p, inp["pack_ocv_v"], inp.get("pack_r_ohm", 0.0))
        out[name] = float(np.trapezoid(p, t) / 3600 / (np.trapezoid(v, t) / 1e3))
    return out


def predict_fuel_map_holdout(inp) -> dict:
    """Fuel flow (g/s) at held-out dyno points from a fuel map fitted on calibration points. Inputs:
    displacement_cc, lhv_mj_kg, calibration (rows of rpm, torque_nm, fuel_g_s), predict (rows of rpm, torque_nm)."""
    cal = np.asarray(inp["calibration"], float)
    line = ice.fit_willans(cal[:, 1], cal[:, 0], cal[:, 2], inp["displacement_cc"], inp["lhv_mj_kg"])
    out = {}
    for n, t in inp["predict"]:
        p = t * n * np.pi / 30
        out[f"{n:g}rpm_{t:g}Nm"] = float(line.fuel_power_w(p, n, inp["displacement_cc"]) / (inp["lhv_mj_kg"] * 1e3))
    return out


PREDICTORS = {"ev_cycle": predict_ev_cycle, "fuel_map_holdout": predict_fuel_map_holdout}


def preregister(case_id, allow_dirty=False, now=None) -> dict:
    """Seal KinematiK's predictions for a paper whose results are not yet entered. Refuses when the case already has
    results, or when the working tree has uncommitted changes (a prediction must come from a committed release)."""
    import datetime as dt
    case = _papers()[case_id]
    if case.get("results"):
        raise ValueError(f"{case_id} already has results: pre-registration must come before them")
    dirty = bool(_git("status", "--porcelain"))
    if dirty and not allow_dirty:
        raise ValueError("uncommitted changes: commit KinematiK first, so the prediction names a release")
    record = {"case": case_id, "type": case["type"], "kinematik_commit": _git("rev-parse", "HEAD"), "dirty": dirty,
              "created_utc": (now or dt.datetime.now(dt.UTC)).isoformat(timespec="seconds"),
              "inputs_sha256": _canonical(case["inputs"]), "predictions": PREDICTORS[case["type"]](case["inputs"])}
    SEALED.mkdir(exist_ok=True)
    path = SEALED / f"{case_id}.json"
    path.write_text(json.dumps(record, indent=2) + "\n")
    reg = [r for r in _register() if r["case"] != case_id] + [
        {"case": case_id, "file": f"preregistered/{case_id}.json", "sha256": sha256(path),
         "created_utc": record["created_utc"], "kinematik_commit": record["kinematik_commit"]}]
    REGISTER.write_text(json.dumps(reg, indent=2) + "\n")
    return record


def check_preregistered(case_id) -> list[str]:
    """Problems with a graded paper's seal (empty when it holds): a register entry, a sealed file matching its
    SHA-256, the same inputs as now, and no results inside the sealed file."""
    case = _papers()[case_id]
    entry = next((r for r in _register() if r["case"] == case_id), None)
    if entry is None:
        return [f"{case_id}: no pre-registration"]
    path = BENCH / entry["file"]
    if not path.exists():
        return [f"{case_id}: sealed file {entry['file']} is missing"]
    problems = []
    if sha256(path) != entry["sha256"]:
        problems.append(f"{case_id}: sealed file changed after registration")
    sealed = json.loads(path.read_text())
    if sealed["inputs_sha256"] != _canonical(case["inputs"]):
        problems.append(f"{case_id}: inputs changed after registration")
    if "results" in sealed:
        problems.append(f"{case_id}: the sealed file holds results")
    return problems


def grade_paper(case_id, limit) -> tuple[list, list[str]]:
    """Sealed and current-model predictions against the paper's results: rows of (key, result, sealed, current)."""
    case = _papers()[case_id]
    problems = check_preregistered(case_id)
    if not case.get("results"):
        return [], problems + [f"{case_id}: no results entered yet"]
    sealed = json.loads((BENCH / f"preregistered/{case_id}.json").read_text())["predictions"] if not problems else {}
    current = PREDICTORS[case["type"]](case["inputs"])
    rows = [(k, float(v), sealed.get(k), current.get(k)) for k, v in case["results"].items()]
    return rows, problems


def report_paper(case_id, limit) -> int:
    rows, problems = grade_paper(case_id, limit)
    for p in problems:
        print("  " + p)
    if problems:
        return 2
    case = _papers()[case_id]
    print(f"{case_id}: {case.get('source', '')}")
    print(f"  {'point':28} {'paper':>10} {'sealed':>10} {'error':>7}   {'current model':>13} {'error':>7}")
    for k, res, s, c in rows:
        print(f"  {k:28} {res:10.4g} {s:10.4g} {s / res - 1:+7.1%} {'PASS' if abs(s / res - 1) <= limit else 'FAIL'}"
              f" {c:13.4g} {c / res - 1:+7.1%}")
    e = stats(s / res - 1 for _, res, s, _ in rows)
    print(f"  sealed: MAE {e['mae']:.1%}, bias {e['bias']:+.1%}, worst {e['worst']:+.1%}")
    return 0 if all(abs(s / res - 1) <= limit for _, res, s, _ in rows) else 1


def extract_launches(raw_dir, ref=None) -> list[str]:
    """Cut the BMW i3's wide-open-throttle standing starts out of Argonne test 61505016 at the logger's 10 Hz."""
    la = (ref or _refs())["argonne"]["launches"]
    path = Path(raw_dir) / f"{la['test']} Test Data.txt"
    names = path.read_text(errors="replace").splitlines()[0].split("\t")
    arr = np.genfromtxt(path, delimiter="\t", skip_header=1)

    def col(prefix):
        return arr[:, next(i for i, n in enumerate(names) if n.startswith(prefix))]

    t, v = col("Time"), col("Dyno_Speed") * 0.44704
    ped, rpm = col("Accelerator_Pedal_Position"), col("Motor_Speed")
    p = col("HV_Battery_Voltage") * col("HV_Battery_Current")
    wot = ped > la["min_pedal_pct"]
    starts = np.where(np.diff(wot.astype(int)) == 1)[0] + 1
    ARGONNE.mkdir(exist_ok=True)
    out = []
    for s in starts:
        e = s + int(np.argmax(~wot[s:]))
        if v[s - 1] > 0.5 or v[s:e].max() < la["min_speed_ms"]:
            continue
        a = max(s - 10, 0)
        name = f"{la['test']}_wot_{len(out) + 1}.csv"
        with open(ARGONNE / name, "w", newline="") as fh:
            w = csv.writer(fh, lineterminator="\n")
            w.writerow(["time_s", "speed_ms", "motor_rpm", "batt_power_w", "pedal_pct"])
            for k in range(a, e):
                w.writerow([f"{t[k] - t[a]:.2f}", f"{v[k]:.4f}", f"{rpm[k]:.0f}", f"{p[k]:.0f}", f"{ped[k]:.1f}"])
        out.append(name)
    return out


def _launch_logs(ref):
    la = ref["argonne"]["launches"]
    return [_load_csv(ARGONNE / f.name) for f in sorted(ARGONNE.glob(f"{la['test']}_wot_*.csv"))]


def _speed_times(t, v, marks=(13.4112, 26.8224)):
    """Times from the first movement to each speed (default 30 and 60 mph)."""
    i0 = max(int(np.argmax(v > 0.2)) - 1, 0)
    out = []
    for x in marks:
        j = int(np.argmax(v >= x))
        out.append(float(np.interp(x, v[j - 1:j + 1], t[j - 1:j + 1]) - t[i0]))
    return out


def ev_launch_rows(ref=None) -> dict:
    """KinematiK's EV launch (powertrain.engine.simulate_launch) against the i3's measured standing starts, two
    ways: from the spec sheet (peak torque and power, constant power above base speed), and from a motor curve
    measured on one run predicting the other. Road load from Argonne's coast-down targets; the motor-to-road speed
    ratio from the logged motor speed."""
    from powertrain import engine as pe
    ref = ref or _refs()
    e, arg = ref["ev"], ref["argonne"]
    la, asm = arg["launches"], arg["launches"]["assumed"]
    veh = e["vehicle"]
    m, g_ratio, eta, kin = veh["mass_kg"], veh["final_drive"], asm["driveline_eff"], asm["rot_inertia_factor"]
    lbf, mph = 4.4482216, 0.44704
    a_n, b_n, c_n = arg["target_a_lb"] * lbf, arg["target_b_lb_per_mph"] * lbf / mph, \
        arg["target_c_lb_per_mph2"] * lbf / mph ** 2
    crr, cda = coastdown_road_load(arg)
    logs = _launch_logs(ref)
    ratios = [np.median(lg["motor_rpm"][lg["speed_ms"] > 5] / lg["speed_ms"][lg["speed_ms"] > 5]) for lg in logs]
    r_eff = g_ratio * 60 / (2 * np.pi) / float(np.mean(ratios))      # rolling radius that matches the logged ratio

    def simulate(motor):
        p = pe.DrivetrainParams(motor=motor, tire_radius_m=r_eff, gear_ratio=g_ratio, mass_kg=m, drivetrain_eff=eta,
                                mu=2.0, driven_weight_frac=0.53, cg_height_m=0.55, wheelbase_m=2.57, cda_m2=cda,
                                crr=crr, rot_inertia_factor=kin)
        res = pe.simulate_launch(p, distance_m=400.0, t_max_s=30.0)
        return _speed_times(res.t_s, res.v_ms)

    def measured_curve(lg):
        t, v = lg["time_s"], lg["speed_ms"]
        acc = np.gradient(np.convolve(v, np.ones(5) / 5, mode="same"), t)
        tq = (m * kin * acc + a_n + b_n * v + c_n * v * v) * r_eff / (g_ratio * eta)
        rpm = v / r_eff * g_ratio * 60 / (2 * np.pi)
        keep = (lg["pedal_pct"] > la["min_pedal_pct"]) & (v > 0.5) & (acc > 0)
        bins = np.arange(0.0, la["motor"]["max_rpm"], 500.0)
        pts = [(b + 250, float(np.median(tq[keep][(rpm[keep] >= b) & (rpm[keep] < b + 500)])))
               for b in bins if np.sum(keep & (rpm >= b) & (rpm < b + 500)) > 2]
        r_, t_ = zip(*pts)
        return pe.MotorCurve(rpm=np.r_[0.0, r_], torque_nm=np.r_[t_[0], t_], redline_rpm=la["motor"]["max_rpm"],
                             eta_motor=0.92, voltage_v=veh["pack_ocv_v"])

    meas = [_speed_times(lg["time_s"], lg["speed_ms"]) for lg in logs]
    spec = simulate(pe.MotorCurve.from_peak(la["motor"]["peak_torque_nm"], la["motor"]["peak_power_kw"],
                                            la["motor"]["max_rpm"], eta_motor=0.92, voltage_v=veh["pack_ocv_v"]))
    if len(logs) < 2:
        raise ValueError("the held-out check needs at least two launches")
    curves = [measured_curve(lg) for lg in logs]
    held_out = [simulate(curves[(k + 1) % len(logs)]) for k in range(len(logs))]
    return {"measured": meas, "spec": spec, "held_out": held_out, "r_eff": r_eff, "curves": curves,
            "repeatability": [float(np.ptp([x[i] for x in meas]) / np.mean([x[i] for x in meas])) for i in (0, 1)]}


def report_argonne(ref, limit) -> int:
    r = argonne_rows(ref)
    print(f"EV  BMW i3 against Argonne D3 (25 degC, HVAC off, hot start); road load from the coast-down targets: "
          f"Crr {r['crr']:.4f}, CdA {r['cda']:.3f} m2")
    print("  drive and regen efficiency fitted on the other segments, auxiliaries measured at standstill")
    print(f"  {'segment':24} {'measured':>8} {'KinematiK':>9} {'error':>7} {'trace RMS':>9} {'eta drive':>9} "
          f"{'eta regen':>9} {'aux W':>6}")
    for name, meas, ours, err, rms, ed, er, aux in r["graded"]:
        print(f"  {name:24} {meas:8.1f} {ours:9.1f} {err:+7.1%} {rms:9.1%} {ed:9.3f} {er:9.3f} {aux:6.0f} "
              f"{'PASS' if abs(err) <= limit else 'FAIL'}")
    s = stats(g[3] for g in r["graded"])
    print(f"  MAE {s['mae']:.1%}, bias {s['bias']:+.1%}, worst {s['worst']:+.1%}")
    ed, er, aux = r["all"]
    print(f"\n  fitted on all graded segments: drive {ed:.3f}, regen {er:.3f}, auxiliaries {aux:.0f} W")
    print("  not graded (KinematiK has no thermal model), predicted with those values:")
    for name, amb, start, hvac, meas, ours, err in r["reported"]:
        print(f"    {name:24} {amb:4} degC {start:4} start, HVAC {hvac:18} measured {meas:6.1f} "
              f"KinematiK {ours:6.1f} ({err:+.1%})")
    lr = ev_launch_rows(ref)
    print(f"\n  launches (Argonne test {ref['argonne']['launches']['test']}, wide-open throttle, KinematiK's "
          f"powertrain.engine.simulate_launch):")
    launch_ok = True
    for k, (meas, held) in enumerate(zip(lr["measured"], lr["held_out"])):
        for label, mv, hv, sv in (("0-30 mph", meas[0], held[0], lr["spec"][0]),
                                  ("0-60 mph", meas[1], held[1], lr["spec"][1])):
            err = hv / mv - 1
            launch_ok &= abs(err) <= limit
            print(f"    run {k + 1} {label}: measured {mv:5.2f} s, motor curve from the other run {hv:5.2f} s "
                  f"{err:+6.1%} {'PASS' if abs(err) <= limit else 'FAIL'}; spec sheet only {sv:5.2f} s "
                  f"({sv / mv - 1:+.1%})")
    print(f"    run-to-run spread of the car itself: 0-30 mph {lr['repeatability'][0]:.1%}, 0-60 mph "
          f"{lr['repeatability'][1]:.1%}")
    rows = {c: (nom, gt) for c, nom, _, _, _, gt in ev_rows(ref)}
    print("\n  the GT-SUITE paper and KinematiK on the paper's inputs, against the measured means (Wh/km):")
    for c, meas in r["cycle_means"].items():
        nom, gt = rows[c]
        print(f"    {c:5} measured {meas:6.1f}   GT-SUITE {gt:6.1f} ({gt / meas - 1:+.1%})   "
              f"KinematiK, paper inputs {nom:6.1f} ({nom / meas - 1:+.1%})")
    return 0 if all(abs(g[3]) <= limit for g in r["graded"]) and launch_ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seal", action="store_true", help="print the SHA-256 of results.csv")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("argonne", help="grade against Argonne's measured BMW i3 (benchmarks/gt_suite/argonne)")
    x = sub.add_parser("argonne-extract", help="cut the segments out of raw D3 files (writes benchmarks/gt_suite/argonne)")
    x.add_argument("raw_dir")
    sub.add_parser("argonne-ice", help="grade the ICE against Argonne's measured 2019 Chevrolet Blazer")
    xi = sub.add_parser("argonne-ice-extract", help="cut benchmarks/gt_suite/argonne_ice from Argonne's TDMS files")
    xi.add_argument("raw_dir")
    fk = sub.add_parser("fsk2023", help="the pack model against Team A-FA's Formula Student Korea 2023 telemetry")
    fk.add_argument("path", help="data.msgpack from a clone of luftaquila/a-fa-telemetry-endurance-viewer")
    pr = sub.add_parser("preregister", help="seal KinematiK's predictions for a paper in papers.json before its "
                                            "results are entered")
    pr.add_argument("case")
    pr.add_argument("--allow-dirty", action="store_true", help="seal from uncommitted code (recorded as dirty)")
    gp = sub.add_parser("grade", help="grade a pre-registered paper once its results are in papers.json")
    gp.add_argument("case")
    args = ap.parse_args(argv)
    bad_files = verify_manifest()
    if bad_files:
        print("MANIFEST.sha256 does not match: " + ", ".join(bad_files) + "\nA reference changed: review it, then "
              "regenerate the manifest (sha256sum, see docs/GT_SUITE_BENCHMARK.md).")
        return 2
    ref = _refs()
    limit = ref["tolerance"]["limit"]
    if args.cmd == "argonne-extract":
        for name in extract_launches(args.raw_dir, ref):
            print(f"  launch: {name}")
        print("segment                         extracted Wh  Argonne Wh  difference")
        for name, wh, ref_wh in extract(args.raw_dir, ref):
            print(f"  {name:30} {wh:10.1f} {ref_wh:11.1f} {wh / ref_wh - 1:+10.2%}")
        print("Now regenerate MANIFEST.sha256 (docs/GT_SUITE_BENCHMARK.md).")
        return 0
    if args.cmd == "argonne":
        return report_argonne(ref, limit)
    if args.cmd == "argonne-ice-extract":
        for name, g, ref_g in extract_ice(args.raw_dir, ref):
            print(f"  {name:24} direct meter {g:8.1f} g  bag {ref_g:8.1f} g  {g / ref_g - 1:+.2%}")
        print("Now regenerate MANIFEST.sha256 (docs/GT_SUITE_BENCHMARK.md).")
        return 0
    if args.cmd == "argonne-ice":
        return report_argonne_ice(ref, limit)
    if args.cmd == "fsk2023":
        return report_fsk(args.path, ref, limit)
    if args.cmd in ("preregister", "grade") and args.case not in _papers():
        print(f"no case {args.case!r} in benchmarks/gt_suite/papers.json")
        return 2
    if args.cmd == "preregister":
        try:
            rec = preregister(args.case, allow_dirty=args.allow_dirty)
        except ValueError as e:
            print(e)
            return 2
        print(f"sealed {len(rec['predictions'])} predictions for {args.case} from {rec['kinematik_commit'][:10]}"
              f"{' (dirty)' if rec['dirty'] else ''} at {rec['created_utc']}.\nNow commit and push "
              f"benchmarks/gt_suite/preregistered/{args.case}.json and preregistrations.json, before entering the "
              "paper's results.")
        return 0
    if args.cmd == "grade":
        return report_paper(args.case, limit)
    graded = []
    print("EV  BMW i3, GT-SUITE (SAE 2021-01-1245), Wh/km at 25 degC; band: assumed efficiency, air density, pack resistance")
    print(f"  {'cycle':6} {'KinematiK':>9} {'band':>15} {'GT-SUITE':>9} {'error':>7}      {'no regen':>8}")
    for cyc, nom, lo, hi, nr, gt in ev_rows(ref):
        err = nom / gt - 1
        graded.append(("ev", cyc, "wh_per_km", nom, gt, err))
        inside = "in band" if lo <= gt <= hi else "outside band"
        print(f"  {cyc:6} {nom:9.1f} {lo:7.1f}-{hi:6.1f} {gt:9.1f} {err:+7.1%} {'PASS' if abs(err) <= limit else 'FAIL'}"
              f" {nr:8.1f} ({nr / gt - 1:+.1%})  {inside}")

    w2 = ev_2w_rows(ref)
    e2 = ref["ev_2w"]
    err = w2["top_kmh"] / e2["top_speed_kmh"] - 1
    print(f"\nEV  electric two-wheeler, 220 kg, 5 kW, GT-SUITE (Materials Research Proceedings 49, 2025)")
    print(f"  top speed {w2['top_kmh']:.1f} km/h against {e2['top_speed_kmh']} {err:+.1%} "
          f"{'PASS' if abs(err) <= limit else 'FAIL'}; motor speed at that speed {w2['motor_rpm_at_gt_top']:.0f} rpm "
          f"against {e2['motor_rpm_at_top']:.0f}")
    graded.append(("ev", "two_wheeler", "top_speed_kmh", w2["top_kmh"], e2["top_speed_kmh"], err))
    av = ev_avanza_rows(ref)
    sg = ref["ev_avanza"]["single_gear"]
    e_t, e_v = av["t_0_100"] / sg["t_0_100_s"] - 1, av["top_kmh"] / sg["top_speed_kmh"] - 1
    print(f"\nEV  converted Toyota Avanza, single gear, GT-SUITE (Cylinder 11(1), 2025); tire radius, Crr assumed")
    print(f"  0-100 km/h {av['t_0_100']:.2f} s against {sg['t_0_100_s']} {e_t:+.1%} "
          f"{'PASS' if abs(e_t) <= limit else 'FAIL'}")
    print(f"  top speed  {av['top_kmh']:.1f} km/h against {sg['top_speed_kmh']} {e_v:+.1%} "
          f"{'PASS' if abs(e_v) <= limit else 'FAIL'}")
    lo, hi = av["nedc_band"]
    print(f"  NEDC {av['nedc_regen']:.1f} Wh/km with regeneration, {av['nedc_no_regen']:.1f} without, against "
          f"{av['gt_nedc_wh_km']:.1f} ({'inside' if lo <= av['gt_nedc_wh_km'] <= hi else 'outside'} the band "
          f"{lo:.0f}-{hi:.0f}; not graded: the paper does not say whether regeneration is on)")
    graded += [("ev", "avanza", "t_0_100_s", av["t_0_100"], sg["t_0_100_s"], e_t),
               ("ev", "avanza", "top_speed_kmh", av["top_kmh"], sg["top_speed_kmh"], e_v)]

    irows, crr, bte = ice_rows(ref)
    print(f"\nICE 12 t diesel truck, GT-SUITE (SAE 2012-36-0247), constant speed, no accessories"
          f"\n  set at {ref['ice']['calibrate_at_kmh']} km/h: Crr {crr:.5f}, BTE {bte:.3f}")
    print(f"  {'km/h':>4} {'kW ours':>8} {'kW GT':>7} {'error':>7}  {'km/l ours':>9} {'km/l GT':>8} {'error':>7}")
    for kmh, p, pg, fe, fg in irows:
        ep, ef = p / pg - 1, fe / fg - 1
        tag = "set" if kmh == ref["ice"]["calibrate_at_kmh"] else ""
        print(f"  {kmh:4} {p:8.2f} {pg:7.2f} {ep:+7.1%} {tag:4} {fe:9.2f} {fg:8.2f} {ef:+7.1%} {tag}")
    loo = ice_leave_one_out(ref)
    print("  calibrated at each speed in turn, worst error on the other five:")
    for k, (wp, wf) in loo.items():
        print(f"    set at {k:2} km/h: power {wp:+6.1%}, fuel {wf:+6.1%}")
        graded += [("ice", f"set_{k}kmh", "engine_kw_worst", wp, 0.0, wp),
                   ("ice", f"set_{k}kmh", "km_per_l_worst", wf, 0.0, wf)]

    gr = gear_launch_rows(ref)
    print(f"\nICE gearing: Proton Saga 1.3 L, GT-SUITE (JAITA 2024), 0-402 m for 25 gear sets; a 4-speed automatic's "
          f"upshifts at {ref['ice_gears']['assumed']['upshift_kmh']} km/h; torque curve scaled x{gr['scale']:.3f} so "
          f"the standard set gives the base {ref['ice_gears']['base_402m_s']} s")
    for k, (gs, o, r) in enumerate(zip(ref["ice_gears"]["gear_sets"], gr["ours"], gr["ref"])):
        graded.append(("ice", "gears_" + "-".join(f"{x:g}" for x in gs), "time_402m_s", o, r, o / r - 1))
    e = stats(o / r - 1 for o, r in zip(gr["ours"], gr["ref"]))
    print(f"  402 m time: MAE {e['mae']:.1%}, worst {e['worst']:+.1%} (narrow: GT-SUITE's sets span only "
          f"{gr['spread_ref']:.2f} s)")
    print(f"  the gear effect: KinematiK's change from the base set is {gr['sensitivity']:.2f} x GT-SUITE's "
          f"(spread {gr['spread_ours']:.2f} s against {gr['spread_ref']:.2f} s), rank correlation {gr['rank']:.2f}; "
          f"fastest set: GT-SUITE #{gr['best_ref'] + 1}, KinematiK #{gr['best_ours'] + 1}")
    mf = gr["max_force"]
    print(f"  shifting at maximum drive force instead (a race driver): gear effect {mf['sensitivity']:.2f} x, rank "
          f"{mf['rank']:.2f} (diagnostic: the paper's car is an automatic)")
    graded.append(("ice", "gears", "sensitivity", gr["sensitivity"], 1.0, gr["sensitivity"] - 1))

    with open(BENCH / "results.csv", "w", newline="") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(["module", "case", "metric", "kinematik", "reference", "error"])
        w.writerows((m, c, k, f"{x:.6g}", f"{y:.6g}", f"{e:+.4f}") for m, c, k, x, y, e in graded)
    for mod in ("ev", "ice"):
        s = stats(r[5] for r in graded if r[0] == mod)
        print(f"\n{mod.upper()}: MAE {s['mae']:.1%}, bias {s['bias']:+.1%}, worst {s['worst']:+.1%}", end="")
    bad = [r for r in graded if abs(r[5]) > limit]
    print(f"\n\n{len(graded) - len(bad)}/{len(graded)} graded rows within {limit:.0%}"
          + ("; outside: " + ", ".join(f"{m} {c} {k}" for m, c, k, *_ in bad) if bad else ""))
    if args.seal:
        print(f"results.csv SHA-256: {sha256(BENCH / 'results.csv')}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
