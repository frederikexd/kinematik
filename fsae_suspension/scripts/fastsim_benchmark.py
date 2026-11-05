"""KinematiK's EV and ICE powertrain modules against NREL FASTSim 3.1.0.

    python scripts/fastsim_benchmark.py            # grade; FASTSim itself is not needed, its runs are stored
    python scripts/fastsim_benchmark.py extract    # rerun FASTSim and rewrite benchmarks/fastsim
                                                   # (pip install fastsim==3.1.0)

FASTSim (Future Automotive Systems Technology Simulator, National Renewable Energy Laboratory, Apache 2.0) is the
U.S. Department of Energy's vehicle energy model, validated by NREL against Argonne dynamometer data. Its bundled
vehicles are driven over the same cycle files as the GT-SUITE benchmark (UDDS, HWFET, US06, NEDC). KinematiK is
given each vehicle's parameters as FASTSim states them and FASTSim's achieved speed trace; it then computes the
energy on its own, with suspension.ev_electrical_check.traction_power_w (EV, battery terminals) and
suspension.ice_powertrain.fuel_for_lap (ICE, fuel energy).

Graded: energy per km, within the tolerance in benchmarks/gt_suite/references.json. Reported: the tractive energy
alone (the road-load layer), and for the petrol cars a Willans line fitted to FASTSim's engine map with
fit_willans, to show what the straight line costs against a curved map.

Everything read is listed with its SHA-256 in benchmarks/fastsim/MANIFEST.sha256.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from suspension import ev_electrical_check as ec  # noqa: E402
from suspension import ice_powertrain as ice  # noqa: E402

BENCH = ROOT / "benchmarks" / "fastsim"
CYCLE_DIR = ROOT / "benchmarks" / "gt_suite"
MANIFEST = BENCH / "MANIFEST.sha256"
FASTSIM_VERSION = "3.1.0"
CYCLES = {"udds": "udds.csv", "hwy": "hwfet.csv", "us06": "us06.csv", "nedc": "nedc.csv"}
# key: (FASTSim resource, powertrain, how it is set up)
VEHICLES = {
    "zoe": ("2022_Renault_Zoe_ZE50_R135.yaml", "ev", "as bundled"),
    "leaf": ("2016 Nissan Leaf 30 kWh thrml.yaml", "ev", "cabin, HVAC and battery thermal models off"),
    "model3": ("2022 Tesla Model 3 RWD thrml.yaml", "ev", "cabin, HVAC and battery thermal models off"),
    "fusion": ("2012_Ford_Fusion.yaml", "ice", "as bundled"),
    "pacifica": ("2026_Chrysler_Pacifica_Select.yaml", "ice", "stop-start and deceleration fuel cut-off off"),
}
LHV_MJ_KG = 43.0          # any value: FASTSim works in fuel power, and fuel_for_lap's kg are turned back into joules
WILLANS_RPM = 3000.0      # FASTSim's map has no engine speed; the Willans line is fitted and run at this one


def sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_manifest() -> list[str]:
    bad = []
    for line in MANIFEST.read_text().splitlines():
        if line.strip():
            digest, name = line.split(maxsplit=1)
            f = ROOT / "benchmarks" / name.strip()
            if not f.exists() or sha256(f) != digest:
                bad.append(name.strip())
    return bad


def _runs() -> dict:
    return json.loads((BENCH / "fastsim_runs.json").read_text())


def _trace(name) -> tuple[np.ndarray, np.ndarray]:
    with open(BENCH / f"{name}.csv", newline="") as fh:
        rows = list(csv.DictReader(fh))
    return np.array([float(r["time_s"]) for r in rows]), np.array([float(r["speed_ms"]) for r in rows])


def _steps(t, v):
    """FASTSim's integration: each step at the mean of its end speeds, acceleration the change over the step."""
    dt = np.diff(t)
    return dt, 0.5 * (v[1:] + v[:-1]), np.diff(v) / dt


def _rot(p) -> float:
    return 1.0 + p["wheel_inertia_total_kgm2"] / (p["wheel_radius_m"] ** 2 * p["mass_kg"])


# ---------------------------------------------------------------------------------------------------------------------
#  KinematiK on FASTSim's vehicles
# ---------------------------------------------------------------------------------------------------------------------
def tractive_wh_km(p, t, v) -> float:
    dt, vm, a = _steps(t, v)
    _, pw, _ = ec.traction_power_w(vm, a, vehicle_mass_kg=p["mass_kg"], drag_cda=p["cda_m2"], crr=p["crr"],
                                   rho_air=p["rho"], rot_mass_factor=_rot(p))
    return float(np.sum(np.maximum(pw, 0.0) * dt)) / 3.6e3 / (float(np.sum(vm * dt)) / 1e3)


def _linear_table(frac, eff, strategy):
    """A FASTSim map as a table KinematiK reads linearly. A step map (each efficiency held up to the next point)
    becomes a staircase: every step ends a hair before the next point."""
    frac, eff = np.asarray(frac, float), np.asarray(eff, float)
    if strategy == "Linear":
        return frac, eff
    if strategy != "Step-Lower":
        raise ValueError(f"FASTSim interpolation {strategy!r} is not handled")
    x = np.column_stack([frac[:-1], frac[1:] * (1 - 1e-9)]).ravel()
    return np.append(x, frac[-1]), np.append(np.repeat(eff[:-1], 2), eff[-1])


def ev_wh_km(p, t, v) -> float:
    """Battery-terminal energy per km: drive through the gearing and the motor map, regeneration through the gearing
    and the motor's efficiency at zero load (FASTSim 3.1 evaluates it there), capped at the motor's rating."""
    dt, vm, a = _steps(t, v)
    tr, pmax = p["transmission_eff"], p["motor_max_w"]
    frac, eff = _linear_table(p["motor_power_frac"], p["motor_eff"], p["motor_interp"])
    _, _, pp = ec.traction_power_w(
        vm, a, vehicle_mass_kg=p["mass_kg"], drag_cda=p["cda_m2"], crr=p["crr"], rho_air=p["rho"],
        drivetrain_eff=tr, regen_eff=tr * p["motor_eff"][0], regen_cap_w=pmax / tr, rot_mass_factor=_rot(p),
        aux_power_w=p["aux_w"], motor_eff_map=(frac * pmax, eff))
    return float(np.sum(pp * dt)) / 3.6e3 / (float(np.sum(vm * dt)) / 1e3)


def engine_map(p) -> ice.PowerEfficiencyMap:
    frac, eff = _linear_table(p["engine_power_frac"], p["engine_eff"], p["engine_interp"])
    return ice.PowerEfficiencyMap(tuple(frac * p["engine_max_w"]), tuple(eff),
                                  idle_fuel_w=p["idle_fuel_w"], source=f"FASTSim {FASTSIM_VERSION}")


def engine_willans(p) -> ice.WillansLine:
    """fit_willans on FASTSim's map points, as a user would on a dyno sweep (at one engine speed)."""
    pw = np.array(p["engine_power_frac"]) * p["engine_max_w"]
    eff = np.array(p["engine_eff"])
    ok = (pw > 0) & (eff > 0)
    tq = pw[ok] / (WILLANS_RPM * np.pi / 30)
    fuel_g_s = pw[ok] / eff[ok] / (LHV_MJ_KG * 1e3)
    return ice.fit_willans(tq, np.full(tq.size, WILLANS_RPM), fuel_g_s, p["displacement_cc"], LHV_MJ_KG)


def ice_wh_km(p, t, v, fuel_model) -> float:
    """Fuel energy per km from fuel_for_lap on the moving part of the trace; standing still the engine idles,
    driving the auxiliaries (fuel_for_lap works along distance, so standstill time is added here)."""
    dt, vm, _ = _steps(t, v)
    s = np.concatenate([[0.0], np.cumsum(vm * dt)])
    pmax = p["engine_max_w"]
    veh = ice.ICEVehicle(mass_kg=p["mass_kg"], cda_m2=p["cda_m2"], crr=p["crr"], rho=p["rho"],
                         wheel_inertia_kgm2=p["wheel_inertia_total_kgm2"])
    eng = ice.EngineMap(rpm=[500.0, 8000.0], torque_nm=[1e4, 1e4], redline_rpm=8000.0, idle_rpm=500.0,
                        inertia_kgm2=0.0, displacement_cc=p["displacement_cc"], power_cap_kw=pmax / 1e3,
                        source=f"FASTSim {FASTSIM_VERSION}")
    gb = ice.Gearbox(primary=1.0, ratios=(1.0,), final_drive=5.0, shift_time_s=0.0,
                     efficiency=p["transmission_eff"], tire_radius_m=p["wheel_radius_m"], source="FASTSim")
    lap = SimpleNamespace(s=s, v=v, rpm=np.full(v.size, WILLANS_RPM))
    r = ice.fuel_for_lap(lap, eng, gb, veh, fuel_key="gasoline", overrun_fuel_g_s=0.0, willans=fuel_model,
                         aux_power_w=p["aux_w"])
    lhv = ice.fuel("gasoline").lhv_mj_kg * 1e6
    standing = float(np.sum(dt[vm == 0]))
    idle_w = float(np.ravel(fuel_model.fuel_power_w(p["aux_w"], WILLANS_RPM, p["displacement_cc"]))[0])
    return (r.fuel_kg * lhv + idle_w * standing) / 3.6e3 / (float(np.sum(vm * dt)) / 1e3)


def rows(runs=None) -> list[dict]:
    runs = runs or _runs()
    out = []
    for key, p in runs["vehicles"].items():
        for cyc in CYCLES:
            name = f"{key}_{cyc}"
            fsr = runs["results"][name]
            t, v = _trace(name)
            row = {"vehicle": key, "cycle": cyc, "powertrain": p["powertrain"],
                   "tractive_k": tractive_wh_km(p, t, v), "tractive_fs": fsr["tractive_wh_km"]}
            if p["powertrain"] == "ev":
                row["energy_k"] = ev_wh_km(p, t, v)
            else:
                row["energy_k"] = ice_wh_km(p, t, v, engine_map(p))
                row["willans_k"] = ice_wh_km(p, t, v, engine_willans(p))
            row["energy_fs"] = fsr["energy_wh_km"]
            row["err"] = row["energy_k"] / row["energy_fs"] - 1
            out.append(row)
    return out


def report(limit) -> int:
    rs = rows()
    runs = _runs()
    print(f"KinematiK against NREL FASTSim {runs['fastsim_version']}: energy per km, same vehicle, same speed trace")
    print(f"  {'vehicle':9} {'cycle':5} {'tractive K':>10} {'FASTSim':>8} {'diff':>7}   "
          f"{'energy K':>9} {'FASTSim':>8} {'error':>7}")
    fails = 0
    for r in rs:
        unit = "battery" if r["powertrain"] == "ev" else "fuel"
        ok = abs(r["err"]) <= limit
        fails += not ok
        extra = ""
        if "willans_k" in r:
            extra = f"   Willans line {r['willans_k']:7.1f} ({r['willans_k'] / r['energy_fs'] - 1:+.1%}, not graded)"
        print(f"  {r['vehicle']:9} {r['cycle']:5} {r['tractive_k']:10.1f} {r['tractive_fs']:8.1f} "
              f"{r['tractive_k'] / r['tractive_fs'] - 1:+7.2%}   {r['energy_k']:9.1f} {r['energy_fs']:8.1f} "
              f"{r['err']:+7.2%} {'PASS' if ok else 'FAIL'} ({unit}){extra}")
    e = np.array([r["err"] for r in rs])
    print(f"\n{len(rs) - fails}/{len(rs)} within {limit:.0%}; mean absolute error {np.mean(np.abs(e)):.2%}, "
          f"bias {np.mean(e):+.2%}, worst {e[np.argmax(np.abs(e))]:+.2%}")
    return 0 if fails == 0 else 1


# ---------------------------------------------------------------------------------------------------------------------
#  Extraction (needs FASTSim)
# ---------------------------------------------------------------------------------------------------------------------
def _setup(d, key):
    if "thermal" in VEHICLES[key][2]:
        d["hvac"] = "None"
        d["cabin"] = "None"
        d["pt_type"]["BEV"]["res"]["thrml"] = "None"
    if "stop-start" in VEHICLES[key][2]:
        c = d["pt_type"]["Conv"]
        c["pt_cntrl"]["StartStop"]["time_delay_after_stop_until_fc_can_turn_off_seconds"] = 1e9
        c["dfco_cntrl"]["dfco_enabled"] = False
    return d


def _strategy(interp) -> str:
    st = interp.get("strategy", "Linear")
    return st if isinstance(st, str) else "-".join(f"{k}-{v}" for k, v in st.items())


def _params(fs, d, key) -> dict:
    ch = d["chassis"]
    pt = d["pt_type"]["BEV" if VEHICLES[key][1] == "ev" else "Conv"]
    p = {"fastsim_resource": VEHICLES[key][0], "powertrain": VEHICLES[key][1], "setup": VEHICLES[key][2],
         "mass_kg": d["mass_kilograms"], "cda_m2": ch["drag_coef"] * ch["frontal_area_square_meters"],
         "crr": ch["wheel_rr_coef"], "wheel_radius_m": ch["wheel_radius_meters"],
         "wheel_inertia_total_kgm2": ch["num_wheels"] * ch["wheel_inertia_kilogram_square_meters"],
         "rho": fs.Air.get_density(None), "aux_w": d["pwr_aux_base_watts"],
         "transmission_eff": pt["transmission"]["eff_interp"]}
    if VEHICLES[key][1] == "ev":
        em = pt["em"]["eff_interp_achieved"]
        p.update(motor_max_w=pt["em"]["pwr_out_max_watts"], motor_power_frac=em["data"]["grid"][0]["data"],
                 motor_eff=em["data"]["values"]["data"], motor_interp=_strategy(em))
    else:
        fc = pt["fc"]
        m = fc["eff_interp_from_pwr_out"]
        p.update(engine_max_w=fc["pwr_out_max_watts"], engine_power_frac=m["data"]["grid"][0]["data"],
                 engine_eff=m["data"]["values"]["data"], engine_interp=_strategy(m), idle_fuel_w=fc["pwr_idle_fuel_watts"],
                 displacement_cc=2000.0)
    return p


def extract() -> None:
    import fastsim as fs
    if fs.__version__ != FASTSIM_VERSION:
        raise SystemExit(f"FASTSim {FASTSIM_VERSION} is pinned; this is {fs.__version__}")
    BENCH.mkdir(parents=True, exist_ok=True)
    runs = {"fastsim_version": fs.__version__, "air_density_note": "FASTSim's constant air density, kg/m3",
            "vehicles": {}, "results": {}}
    for key, (res, kind, _) in VEHICLES.items():
        stock = fs.Vehicle.from_resource(res).to_dict()
        d = _setup(json.loads(json.dumps(stock)), key)
        p = _params(fs, d, key)
        p["fastsim_vehicle_sha256"] = hashlib.sha256(json.dumps(stock, sort_keys=True).encode()).hexdigest()
        runs["vehicles"][key] = p
        for cyc, fname in CYCLES.items():
            with open(CYCLE_DIR / fname, newline="") as fh:
                rr = list(csv.DictReader(fh))
            t = [float(r["time_s"]) for r in rr]
            v = [float(r["speed_ms"]) for r in rr]
            c = fs.Cycle.from_resource("udds.csv").to_dict()
            c.update({"time_seconds": t, "speed_meters_per_second": v, "grade": [0.0] * len(t)})
            c = {k: x for k, x in c.items() if not isinstance(x, list) or len(x) == len(t)}
            veh = fs.Vehicle.from_dict(d)
            veh.set_save_interval(1)
            sp = fs.SimParams.default().to_dict()
            sp["trace_miss_opts"] = "Allow"
            sd = fs.SimDrive(veh, fs.Cycle.from_dict(c), fs.SimParams.from_dict(sp))
            sd.run()
            df = sd.to_dataframe()
            ts = df["cyc.time_seconds"].to_numpy()
            dt = np.diff(ts, prepend=ts[0])
            vs = df["veh.history.speed_ach_meters_per_second"].to_numpy()
            km = df["veh.history.dist_meters"].to_numpy()[-1] / 1e3
            if kind == "ev":
                pe = df["veh.pt_type.BEV.res.history.pwr_out_electrical_watts"].to_numpy()
                em_eff = df["veh.pt_type.BEV.em.history.eff"].to_numpy()
                regen = df["veh.pt_type.BEV.em.history.pwr_mech_prop_out_watts"].to_numpy() < 0
                if regen.any() and not np.allclose(em_eff[regen], p["motor_eff"][0]):
                    raise SystemExit(f"{key} {cyc}: FASTSim's regeneration efficiency is not the zero-load one")
            else:
                pe = df["veh.pt_type.Conv.fc.history.pwr_fuel_watts"].to_numpy()
            name = f"{key}_{cyc}"
            runs["results"][name] = {
                "energy_wh_km": float(np.sum(pe * dt)) / 3.6e3 / km,
                "tractive_wh_km": float(np.sum(np.maximum(df["veh.history.pwr_tractive_watts"].to_numpy(), 0) * dt))
                / 3.6e3 / km,
                "distance_km": km, "trace_missed_max_ms": float(np.max(np.abs(vs - np.array(v))))}
            with open(BENCH / f"{name}.csv", "w", newline="") as fh:
                w = csv.writer(fh, lineterminator="\n")
                w.writerow(["time_s", "speed_ms"])
                w.writerows([f"{a:g}", repr(float(b))] for a, b in zip(ts, vs))
            print(f"  {name:16} {runs['results'][name]['energy_wh_km']:8.1f} Wh/km")
    (BENCH / "fastsim_runs.json").write_text(json.dumps(runs, indent=1) + "\n")
    files = sorted([f"fastsim/{f.name}" for f in BENCH.glob("*.csv")] + ["fastsim/fastsim_runs.json"]
                   + [f"gt_suite/{c}" for c in CYCLES.values()])
    MANIFEST.write_text("".join(f"{sha256(ROOT / 'benchmarks' / f)}  {f}\n" for f in files))
    print("wrote benchmarks/fastsim and its MANIFEST.sha256")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", nargs="?", choices=["extract"])
    args = ap.parse_args(argv)
    if args.cmd == "extract":
        extract()
        return 0
    bad = verify_manifest()
    if bad:
        print("MANIFEST.sha256 does not match: " + ", ".join(bad))
        return 2
    limit = json.loads((CYCLE_DIR / "references.json").read_text())["tolerance"]["limit"]
    return report(limit)


if __name__ == "__main__":
    sys.exit(main())
