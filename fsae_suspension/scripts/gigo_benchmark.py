#!/usr/bin/env python3
"""Garbage-in benchmark (docs/GIGO_BENCHMARK_PLAN.md): does KinematiK catch wrong powertrain inputs?

Two gates, measured on realistic input errors applied one at a time to real decks:

  Gate 1, at entry       the wrong value is given to KinematiK: does it refuse it or warn (beyond what it says on the
                         clean deck)?
  Gate 2, second opinion KinematiK holds the right deck; the established tool (FASTSim 3.1.0) was given the wrong
                         one: does the tool's result disagree with KinematiK's by more than GATE2_REL?

Harm is measured by the tool (FASTSim) on the five FASTSim vehicles, and by KinematiK's own Simscape-validated model
on the Formula Student car (no tool to rerun there, so Gate 2 does not apply). Three corruption sets: systematic
(every field x every confusion its dimension allows), random typing errors (seeded by the commit hash of the pushed
seal, so the sample cannot be chosen) and curated real-world errors.

    python scripts/gigo_benchmark.py seal                  # writes benchmarks/gigo/PREREGISTRATION.json (once)
    python scripts/gigo_benchmark.py run COMMIT            # after the seal is pushed: COMMIT is its commit hash;
                                                           # runs FASTSim, writes benchmarks/gigo/runs.json
    python scripts/gigo_benchmark.py report                # grades the stored runs (no FASTSim needed)
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import random
import shutil
import sys
import tempfile
import warnings
from datetime import datetime, UTC
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import fastsim_benchmark as fb  # noqa: E402
import powertrain_benchmark as pb  # noqa: E402

BENCH = ROOT / "benchmarks" / "gigo"
SEAL = BENCH / "PREREGISTRATION.json"
RUNS = BENCH / "runs.json"
GATE2_REL = 0.01            # KinematiK and FASTSim agree to 0.04 % on clean decks
HARM_REL = 0.05             # the fitness-for-use bar for energy and fuel (POWERTRAIN_VALIDATION.md)
N_RANDOM = 60
RANDOM_VERSION = "1"        # bump if the typo generator changes (a new seal)
SEALED_CODE = ("scripts/gigo_benchmark.py", "scripts/fastsim_benchmark.py", "scripts/powertrain_benchmark.py",
               "suspension/ev_electrical_check.py", "suspension/ice_powertrain.py", "suspension/laptime.py",
               "suspension/_num.py")
FS_DECK = ROOT / "benchmarks" / "powertrain" / "simulink_blind2" / "runs"
FS_LAP, FS_LAUNCH = "hock_fs2", "launch_fs2"
CLEAN_FS_DECKS = tuple(c["name"] for c in pb.BLIND2_CASES) + ("../../simulink_hockenheim",)
PREDICTIONS = {"gate2_catch_harmful_min": 0.95, "gate1_catch_harmful_max": 0.30, "gate1_false_alarms": 0,
               "gate2_false_alarms": 0}

# ---------------------------------------------------------------------------------------------------------------------
#  Confusions by dimension: (key, what happened, factor) or a named transform
# ---------------------------------------------------------------------------------------------------------------------
CONFUSIONS = {
    "mass": (("lb_into_kg", "pounds typed into a kg field", 2.20462), ("kg_into_lb", "kg typed into a lb field", 1 / 2.20462),
             ("g_into_kg", "grams typed into a kg field", 1e3), ("t_into_kg", "tonnes typed into a kg field", 1e-3)),
    "area": (("ft2_into_m2", "ft² typed into an m² field", 10.7639), ("m2_into_ft2", "m² typed into a ft² field", 1 / 10.7639)),
    "coefficient": (("percent", "typed in percent", 100.0), ("per_mille", "typed per thousand (kg/t)", 1e3)),
    "length": (("diameter", "diameter typed as radius", 2.0), ("inch", "inches typed into a metre field", 39.3701),
               ("mm", "millimetres typed into a metre field", 1e3), ("cm", "centimetres typed into a metre field", 1e2)),
    "power": (("kilo_low", "W typed into a kW field (or kW into W)", 1e-3), ("kilo_high", "kW typed into a W field", 1e3),
              ("hp_into_kw", "hp typed into a kW field", 1.34102), ("kw_into_hp", "kW typed into an hp field", 1 / 1.34102)),
    "efficiency": (("percent", "efficiency typed in percent", 100.0), ("applied_twice", "the same efficiency applied twice", "square")),
    "torque": (("lbft_into_nm", "lb·ft typed into an N·m field", 1 / 1.35582), ("nm_into_lbft", "N·m typed into a lb·ft field", 1.35582)),
    "ratio": (("inverse", "ratio entered as its inverse", "inverse"),),
    "speed": (("rads_into_rpm", "rad/s typed into an rpm field", 1 / 9.5493),),
    "voltage": (("cell_into_pack", "a cell's 3.6 V typed where the pack voltage belongs", "cell_3v6"),),
    "charge": (("mah_into_ah", "mAh typed into an Ah field", 1e3),),
    "resistance": (("mohm_into_ohm", "milliohms typed into an ohm field", 1e3),),
}


def _tr(x, how):
    if how == "square":
        return x * x
    if how == "inverse":
        return 1.0 / x
    return x * how


# FASTSim decks: field -> (dimension, applies to, path into FASTSim's vehicle dict)
def _pt(d):
    return d["pt_type"]["BEV" if "BEV" in d["pt_type"] else "Conv"]


FS_FIELDS = {
    "mass_kg": ("mass", "all", lambda d: (d, "mass_kilograms")),
    "cda_m2": ("area", "all", lambda d: (d["chassis"], "drag_coef")),
    "crr": ("coefficient", "all", lambda d: (d["chassis"], "wheel_rr_coef")),
    "wheel_radius_m": ("length", "all", lambda d: (d["chassis"], "wheel_radius_meters")),
    "aux_power_w": ("power", "all", lambda d: (d, "pwr_aux_base_watts")),
    "transmission_eff": ("efficiency", "all", lambda d: (_pt(d)["transmission"], "eff_interp")),
    "motor_power_w": ("power", "ev", lambda d: (_pt(d)["em"], "pwr_out_max_watts")),
    "motor_eff_map": ("efficiency", "ev", lambda d: (_pt(d)["em"]["eff_interp_achieved"]["data"]["values"], "data")),
    "engine_power_w": ("power", "ice", lambda d: (_pt(d)["fc"], "pwr_out_max_watts")),
    "engine_eff_map": ("efficiency", "ice", lambda d: (_pt(d)["fc"]["eff_interp_from_pwr_out"]["data"]["values"], "data")),
    "idle_fuel_w": ("power", "ice", lambda d: (_pt(d)["fc"], "pwr_idle_fuel_watts")),
}

# The Formula Student deck (inputs.json): field -> (dimension, path)
FSAE_FIELDS = {
    "mass_kg": ("mass", ("vehicle", "mass_kg")),
    "cda_m2": ("area", ("vehicle", "cda_m2")),
    "cla_m2": ("area", ("vehicle", "cla_m2")),
    "tire_radius_m": ("length", ("vehicle", "tire_radius_m")),
    "cg_height_m": ("length", ("vehicle", "cg_height_m")),
    "wheelbase_m": ("length", ("vehicle", "wheelbase_m")),
    "mu": ("coefficient", ("vehicle", "mu")),
    "final_drive": ("ratio", ("ev", "final_drive")),
    "motor_peak_torque_nm": ("torque", ("ev", "motor_peak_torque_nm")),
    "motor_max_rpm": ("speed", ("ev", "motor_max_rpm")),
    "pack_power_kw": ("power", ("ev", "pack_power_kw")),
    "inverter_eff": ("efficiency", ("ev", "inverter_eff")),
    "driveline_eff": ("efficiency", ("ev", "driveline_eff")),
    "cell_v": ("voltage", ("ev", "pack", "cell_v")),
    "cell_ah": ("charge", ("ev", "pack", "cell_ah")),
    "cell_ohm": ("resistance", ("ev", "pack", "cell_ohm")),
}

# Curated real-world errors: (key, story, deck kind, field, transform)
CURATED = (
    ("lb_for_kg", "the mass from a US scale, in pounds", "all", "mass_kg", 2.20462),
    ("driver_dropped", "the 75 kg driver left out of the mass", "all", "mass_kg", "minus_75"),
    ("cd_for_cda", "the drag coefficient typed where CdA belongs", "fastsim", "cda_m2", "cd_for_cda"),
    ("crr_percent", "rolling resistance typed in percent", "fastsim", "crr", 100.0),
    ("dia_for_radius", "wheel diameter typed as the radius", "all", "radius", 2.0),
    ("hp_for_kw", "a horsepower rating typed as kW", "all", "power", 1.34102),
    ("aux_kilo", "300 W of auxiliaries typed as 0.3 (kW into a W field)", "fastsim", "aux_power_w", 1e-3),
    ("eff_percent", "drive efficiency typed in percent", "all", "drive_eff", 100.0),
    ("eff_twice", "the drive efficiency applied twice", "all", "drive_eff", "square"),
    ("lbft_for_nm", "motor torque from a US datasheet, in lb·ft", "fsae", "motor_peak_torque_nm", 1 / 1.35582),
    ("ratio_inverted", "gear ratio entered as its inverse", "fsae", "final_drive", "inverse"),
    ("rpm_for_rads", "motor top speed in rad/s typed into rpm", "fsae", "motor_max_rpm", 1 / 9.5493),
    ("cell_for_pack_v", "a cell's 3.6 V typed where the pack voltage belongs", "fsae", "cell_v", "cell_3v6"),
    ("mah_for_ah", "cell capacity in mAh typed into Ah", "fsae", "cell_ah", 1e3),
)


def _sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def code_fingerprint(root=None) -> dict:
    root = Path(root or ROOT)
    return {f: _sha256(root / f) for f in SEALED_CODE}


# ---------------------------------------------------------------------------------------------------------------------
#  The corruption list
# ---------------------------------------------------------------------------------------------------------------------
def _fastsim_kinds():
    return {k: v[1] for k, v in fb.VEHICLES.items()}


def systematic() -> list[dict]:
    out = []
    for veh, kind in _fastsim_kinds().items():
        for field, (dim, scope, _) in FS_FIELDS.items():
            if scope in ("all", kind):
                for key, story, how in CONFUSIONS[dim]:
                    out.append({"set": "systematic", "deck": f"fastsim:{veh}", "field": field, "key": key,
                                "story": story, "how": how})
    for field, (dim, _) in FSAE_FIELDS.items():
        for key, story, how in CONFUSIONS[dim]:
            out.append({"set": "systematic", "deck": "fsae", "field": field, "key": key, "story": story, "how": how})
    return out


_ALIAS = {"radius": {"ev": "wheel_radius_m", "ice": "wheel_radius_m", "fsae": "tire_radius_m"},
          "power": {"ev": "motor_power_w", "ice": "engine_power_w", "fsae": "pack_power_kw"},
          "drive_eff": {"ev": "transmission_eff", "ice": "transmission_eff", "fsae": "inverter_eff"}}


def curated() -> list[dict]:
    out = []
    for key, story, scope, field, how in CURATED:
        targets = []
        if scope in ("all", "fastsim"):
            targets += [(f"fastsim:{v}", kind) for v, kind in _fastsim_kinds().items()]
        if scope in ("all", "fsae"):
            targets.append(("fsae", "fsae"))
        for deck, kind in targets:
            out.append({"set": "curated", "deck": deck, "field": _ALIAS.get(field, {}).get(kind, field), "key": key,
                        "story": story, "how": how})
    return out


def _typo(value: float, kind: str, rng: random.Random) -> float:
    """A typing error on the value as written to four significant figures."""
    if kind == "decimal_right":
        return value * 10.0
    if kind == "decimal_left":
        return value / 10.0
    mant, exp = f"{abs(value):.3e}".split("e")
    digits = mant.replace(".", "")
    if kind == "swap":
        i = rng.randrange(len(digits) - 1)
        digits = digits[:i] + digits[i + 1] + digits[i] + digits[i + 2:]
    else:                                   # "drop": one digit left out, the rest shift up
        i = rng.randrange(len(digits))
        digits = digits[:i] + digits[i + 1:] + "0"
    return math.copysign(float(f"{digits[0]}.{digits[1:]}e{exp}"), value)


TYPOS = ("decimal_right", "decimal_left", "swap", "drop")


def random_set(seed_commit: str) -> list[dict]:
    """N_RANDOM typing errors on scalar fields, drawn with the seal commit's hash as the seed."""
    rng = random.Random(int(hashlib.sha256(seed_commit.encode()).hexdigest(), 16))
    pairs = [(f"fastsim:{v}", f) for v, kind in _fastsim_kinds().items() for f, (_, scope, _) in FS_FIELDS.items()
             if scope in ("all", kind) and not f.endswith("_map")]
    pairs += [("fsae", f) for f in FSAE_FIELDS]
    out = []
    for i in range(N_RANDOM):
        deck, field = rng.choice(pairs)
        kind = rng.choice(TYPOS)
        out.append({"set": "random", "deck": deck, "field": field, "key": f"typo_{i:02d}_{kind}", "story": kind,
                    "how": {"typo": kind, "draw": rng.getrandbits(32)}})
    return out


def apply_value(x, how, field=None, deck=None):
    if isinstance(how, dict):
        return _typo(x, how["typo"], random.Random(how["draw"]))
    if how == "minus_75":
        return x - 75.0
    if how == "cell_3v6":                   # the deck's pack is one lumped cell (series 1): cell_v is the pack's
        return 3.6
    if how == "cd_for_cda":
        raise AssertionError("handled on the FASTSim deck")
    return _tr(x, how)


# ---------------------------------------------------------------------------------------------------------------------
#  FASTSim decks
# ---------------------------------------------------------------------------------------------------------------------
def fastsim_dict(veh: str) -> dict:
    import fastsim as fs
    return fb._setup(json.loads(json.dumps(fs.Vehicle.from_resource(fb.VEHICLES[veh][0]).to_dict())), veh)


def corrupt_fastsim(d: dict, c: dict) -> dict:
    d = copy.deepcopy(d)
    obj, k = FS_FIELDS[c["field"]][2](d)
    if c["how"] == "cd_for_cda":            # CdA' = Cd: the coefficient divided by the frontal area once more
        obj[k] = obj[k] / d["chassis"]["frontal_area_square_meters"]
    elif isinstance(obj[k], list):
        obj[k] = [apply_value(x, c["how"]) for x in obj[k]]
    else:
        obj[k] = apply_value(obj[k], c["how"])
    return d


def fastsim_run(d: dict, veh: str) -> dict:
    """FASTSim's energy per km on each cycle, or why it refused."""
    import numpy as np
    import fastsim as fs
    out = {}
    for cyc, fname in fb.CYCLES.items():
        try:
            import csv
            with open(fb.CYCLE_DIR / fname, newline="") as fh:
                rr = list(csv.DictReader(fh))
            t = [float(r["time_s"]) for r in rr]
            v = [float(r["speed_ms"]) for r in rr]
            c = fs.Cycle.from_resource("udds.csv").to_dict()
            c.update({"time_seconds": t, "speed_meters_per_second": v, "grade": [0.0] * len(t)})
            c = {k: x for k, x in c.items() if not isinstance(x, list) or len(x) == len(t)}
            vh = fs.Vehicle.from_dict(d)
            vh.set_save_interval(1)
            sp = fs.SimParams.default().to_dict()
            sp["trace_miss_opts"] = "Allow"
            sd = fs.SimDrive(vh, fs.Cycle.from_dict(c), fs.SimParams.from_dict(sp))
            sd.run()
            df = sd.to_dataframe()
            ts = df["cyc.time_seconds"].to_numpy()
            dt = np.diff(ts, prepend=ts[0])
            km = df["veh.history.dist_meters"].to_numpy()[-1] / 1e3
            col = ("veh.pt_type.BEV.res.history.pwr_out_electrical_watts" if fb.VEHICLES[veh][1] == "ev"
                   else "veh.pt_type.Conv.fc.history.pwr_fuel_watts")
            e = float(np.sum(df[col].to_numpy() * dt)) / 3.6e3 / km
            out[cyc] = {"energy_wh_km": e} if math.isfinite(e) and km > 0 else {"refused": "no finite result"}
        except Exception as exc:  # noqa: BLE001 — FASTSim refusing a deck is a result
            out[cyc] = {"refused": f"{type(exc).__name__}: {str(exc).splitlines()[0][:160] if str(exc) else ''}"}
    return out


def _kinematik_fastsim(d: dict, veh: str) -> tuple:
    """KinematiK on the deck (energy per cycle), and whether it refused or warned."""
    import fastsim as fs
    p = fb._params(fs, d, veh)
    flags, res = [], {}
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        for cyc in fb.CYCLES:
            t, v = fb._trace(f"{veh}_{cyc}")
            try:
                res[cyc] = (fb.ev_wh_km(p, t, v) if p["powertrain"] == "ev"
                            else fb.ice_wh_km(p, t, v, fb.engine_map(p)))
            except ValueError as exc:
                flags.append(f"refused: {exc}")
                res[cyc] = None
            except Exception as exc:  # noqa: BLE001
                flags.append(f"crashed: {type(exc).__name__}")
                res[cyc] = None
    flags += sorted({f"warned: {x.message}" for x in w})
    return res, flags


# ---------------------------------------------------------------------------------------------------------------------
#  The Formula Student deck
# ---------------------------------------------------------------------------------------------------------------------
def _fsae_eval(inp: dict, curve_dir: Path) -> tuple:
    """KinematiK's lap metrics (on hock_fs2's lap) and its 75 m launch on a deck, and whether it refused or warned.
    curve_dir holds the deck's motor_curve.csv."""
    flags, out = [], {}
    with tempfile.TemporaryDirectory() as tmp, warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        tmp = Path(tmp)
        shutil.copy(curve_dir / "motor_curve.csv", tmp / "motor_curve.csv")
        (tmp / "inputs.json").write_text(json.dumps(inp))
        try:
            rows = pb.simulink_lap_rows(tmp / "inputs.json", FS_DECK / FS_LAP / "simulink_lap.csv", skip_s=3.0)["rows"]
            out.update({m: k for m, k, _ in rows})
            for m, k, _ in pb.blind_launch_rows(tmp / "inputs.json", FS_DECK / FS_LAUNCH / "simulink_lap.csv"):
                out[m] = k
        except ValueError as exc:
            flags.append(f"refused: {exc}")
        except Exception as exc:  # noqa: BLE001
            flags.append(f"crashed: {type(exc).__name__}")
    flags += sorted({f"warned: {x.message}" for x in w})
    return out, flags


def _fsae_deck() -> dict:
    return json.loads((FS_DECK / FS_LAP / "inputs.json").read_text())


def corrupt_fsae(inp: dict, c: dict) -> dict:
    inp = copy.deepcopy(inp)
    obj = inp
    path = FSAE_FIELDS[c["field"]][1]
    for k in path[:-1]:
        obj = obj[k]
    obj[path[-1]] = apply_value(obj[path[-1]], c["how"], deck=inp)
    return inp


FSAE_HARM = {"energy_kwh_per_km": 0.05, "peak_current_a": 0.10, "rms_current_a": 0.10, "min_pack_voltage_v": 0.03,
             "launch_75m_time_s": 0.03}


# ---------------------------------------------------------------------------------------------------------------------
#  Seal, run, grade
# ---------------------------------------------------------------------------------------------------------------------
def seal(now=None) -> dict:
    if SEAL.exists():
        raise SystemExit(f"{SEAL} is written once; a new seal is a new file and a new round")
    BENCH.mkdir(parents=True, exist_ok=True)
    rec = {"created_utc": (now or datetime.now(UTC)).strftime("%Y-%m-%dT%H:%M:%SZ"),
           "plan": "docs/GIGO_BENCHMARK_PLAN.md",
           "gates": {"gate1": "KinematiK refuses (ValueError) or warns beyond the clean deck's warnings",
                     "gate2_rel": GATE2_REL, "harm_rel_fastsim": HARM_REL, "harm_rel_fsae": FSAE_HARM},
           "random": {"n": N_RANDOM, "version": RANDOM_VERSION, "seed": "the commit hash that adds this file"},
           "confusions": {d: [list(x[:2]) + [x[2] if not isinstance(x[2], float) else repr(x[2])] for x in v]
                          for d, v in CONFUSIONS.items()},
           "fastsim_fields": {f: list(v[:2]) for f, v in FS_FIELDS.items()},
           "fsae_fields": {f: [v[0], "/".join(v[1])] for f, v in FSAE_FIELDS.items()},
           "corruptions": systematic() + curated(),
           "fastsim_version": fb.FASTSIM_VERSION, "fastsim_vehicles": {k: v[0] for k, v in fb.VEHICLES.items()},
           "fsae_deck": {f"{n}/inputs.json": _sha256(FS_DECK / n / "inputs.json") for n in (FS_LAP, FS_LAUNCH)},
           "clean_fsae_decks": list(CLEAN_FS_DECKS),
           "predictions": PREDICTIONS, "code_sha256": code_fingerprint()}
    SEAL.write_text(json.dumps(rec, indent=1, ensure_ascii=False) + "\n")
    return rec


def run(seed_commit: str) -> dict:
    import fastsim as fs
    rec = json.loads(SEAL.read_text())
    if fs.__version__ != rec["fastsim_version"]:
        raise SystemExit(f"FASTSim {rec['fastsim_version']} is sealed; this is {fs.__version__}")
    changed = [f for f, h in rec["code_sha256"].items() if _sha256(ROOT / f) != h]
    if changed:
        raise SystemExit("the code differs from the sealed code: " + ", ".join(changed))
    corruptions = rec["corruptions"] + random_set(seed_commit)
    out = {"seed_commit": seed_commit, "run_utc": datetime.now(UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
           "fastsim_version": fs.__version__, "clean": {}, "rows": []}
    decks = {v: fastsim_dict(v) for v in fb.VEHICLES}
    for v, d in decks.items():
        out["clean"][f"fastsim:{v}"] = {"fastsim": fastsim_run(d, v), "kinematik": _kinematik_fastsim(d, v)}
    inp = _fsae_deck()
    out["clean"]["fsae"] = {"kinematik": _fsae_eval(inp, FS_DECK / FS_LAP)}
    for name in CLEAN_FS_DECKS:            # false alarms on every Formula Student and stock Simscape deck
        cd = (FS_DECK / name).resolve()
        out["clean"][f"fsae_clean:{name}"] = {"kinematik": _fsae_eval(json.loads((cd / "inputs.json").read_text()),
                                                                      cd)}
    for c in corruptions:
        row = {k: c[k] for k in ("set", "deck", "field", "key", "story")}
        try:
            if c["deck"].startswith("fastsim:"):
                v = c["deck"].split(":")[1]
                d = corrupt_fastsim(decks[v], c)
                if json.dumps(d) == json.dumps(decks[v]):
                    row["no_change"] = True     # a zero scaled, an efficiency of 1 squared, a typo giving the same number
                else:
                    row["fastsim"] = fastsim_run(d, v)
                    row["kinematik"] = _kinematik_fastsim(d, v)
            else:
                ci = corrupt_fsae(inp, c)
                if json.dumps(ci) == json.dumps(inp):
                    row["no_change"] = True
                else:
                    row["kinematik"] = _fsae_eval(ci, FS_DECK / FS_LAP)
        except Exception as exc:  # noqa: BLE001 — a corruption that cannot be applied is reported
            row["not_applied"] = f"{type(exc).__name__}: {exc}"
        out["rows"].append(row)
    RUNS.write_text(json.dumps(out, indent=1, ensure_ascii=False) + "\n")
    (BENCH / "MANIFEST.sha256").write_text(f"{_sha256(RUNS)}  runs.json\n")
    return out


def _new_flags(flags, clean_flags):
    return [f for f in flags if f not in clean_flags]


def grade(runs=None) -> dict:
    """Every row judged: harm, Gate 1 (refused or new warning) and Gate 2 (FASTSim decks)."""
    runs = runs or json.loads(RUNS.read_text())
    graded = []
    for r in runs["rows"]:
        g = {k: r[k] for k in ("set", "deck", "field", "key", "story")}
        if "not_applied" in r or r.get("no_change"):
            g["status"] = "not applied" if "not_applied" in r else "no change"
            graded.append(g)
            continue
        clean = runs["clean"][r["deck"]]
        res, flags = r["kinematik"]
        new = _new_flags(flags, clean["kinematik"][1])
        g["gate1"] = bool([f for f in new if f.startswith(("refused", "warned"))])
        g["crashed"] = any(f.startswith("crashed") for f in new)
        g["flags"] = new
        if r["deck"].startswith("fastsim:"):
            k_clean = clean["kinematik"][0]
            harm, gate2, refused = [], [], []
            for cyc, fsr in r["fastsim"].items():
                if "refused" in fsr:
                    refused.append(cyc)
                    continue
                e0 = clean["fastsim"][cyc]["energy_wh_km"]
                harm.append(abs(fsr["energy_wh_km"] / e0 - 1))
                gate2.append(abs(fsr["energy_wh_km"] / k_clean[cyc] - 1) > GATE2_REL)
            g["tool_refused"] = len(refused) == len(r["fastsim"])
            g["harm"] = max(harm) if harm else None
            g["harmful"] = bool(harm) and max(harm) > HARM_REL
            g["gate2"] = bool(gate2) and any(gate2)
        else:
            c0 = clean["kinematik"][0]
            moves = {m: abs(res[m] / c0[m] - 1) for m in FSAE_HARM if m in res and m in c0 and c0[m]}
            g["harm"] = max(moves.values()) if moves else None
            g["harmful"] = any(moves[m] > FSAE_HARM[m] for m in moves)
            g["gate2"] = None
            g["tool_refused"] = False
        graded.append(g)
    fa1 = [f"{k}: {f}" for k, v in runs["clean"].items() for f in v["kinematik"][1]
           if f.startswith(("refused", "warned", "crashed"))]
    fa2 = [f"{k} {cyc}" for k, v in runs["clean"].items() if k.startswith("fastsim:")
           for cyc, fsr in v["fastsim"].items()
           if abs(fsr["energy_wh_km"] / v["kinematik"][0][cyc] - 1) > GATE2_REL]
    return {"rows": graded, "false_alarms_gate1": sorted(set(fa1)), "false_alarms_gate2": fa2}


def summarize(gr: dict) -> dict:
    out = {}
    for s in ("systematic", "random", "curated", "all"):
        rows = [r for r in gr["rows"] if (s == "all" or r["set"] == s) and not r.get("status")]
        harmful = [r for r in rows if r["harmful"]]
        fsh = [r for r in harmful if r["gate2"] is not None]
        out[s] = {"corruptions": len(rows), "harmful": len(harmful),
                  "tool_refused": sum(r["tool_refused"] for r in rows),
                  "gate1_harmful": sum(r["gate1"] for r in harmful),
                  "gate2_harmful": sum(r["gate2"] for r in fsh), "gate2_applicable": len(fsh),
                  "either_harmful": sum(r["gate1"] or bool(r["gate2"]) for r in harmful),
                  "silent_harmful": [f"{r['deck']} {r['field']} {r['key']} ({r['harm']:.0%})" for r in harmful
                                     if not r["gate1"] and not r["gate2"]],
                  "crashed": sum(r["crashed"] for r in rows),
                  "not_applied": sum(1 for r in gr["rows"] if (s == "all" or r["set"] == s)
                                     and r.get("status") == "not applied"),
                  "no_change": sum(1 for r in gr["rows"] if (s == "all" or r["set"] == s)
                                   and r.get("status") == "no change")}
    return out


def report() -> int:
    rec = json.loads(SEAL.read_text())
    gr = grade()
    sm = summarize(gr)
    pct = lambda a, b: f"{a}/{b} ({a / b:.0%})" if b else "n/a"  # noqa: E731
    for s, v in sm.items():
        print(f"\n{s}: {v['corruptions']} corruptions, {v['harmful']} harmful (beyond the fitness bar), "
              f"{v['tool_refused']} refused by FASTSim itself")
        print(f"  Gate 1, at entry:        {pct(v['gate1_harmful'], v['harmful'])} of the harmful ones")
        print(f"  Gate 2, second opinion:  {pct(v['gate2_harmful'], v['gate2_applicable'])} of the harmful ones on "
              "FASTSim decks")
        print(f"  either gate:             {pct(v['either_harmful'], v['harmful'])}")
        if v["crashed"]:
            print(f"  KinematiK crashed (not counted as caught): {v['crashed']}")
        if v["not_applied"]:
            print(f"  not applied: {v['not_applied']}")
        if v["no_change"]:
            print(f"  left the value unchanged (not counted): {v['no_change']}")
        if s != "all":
            for x in v["silent_harmful"]:
                print(f"    silent and harmful: {x}")
    print(f"\nfalse alarms, Gate 1: {gr['false_alarms_gate1'] or 'none'}")
    print(f"false alarms, Gate 2: {gr['false_alarms_gate2'] or 'none'}")
    a = sm["all"]
    p = rec["predictions"]
    g2 = a["gate2_harmful"] / a["gate2_applicable"] if a["gate2_applicable"] else math.nan
    g1 = a["gate1_harmful"] / a["harmful"] if a["harmful"] else math.nan
    print("\nsealed predictions:")
    print(f"  Gate 2 >= {p['gate2_catch_harmful_min']:.0%}: {g2:.0%} -> {'held' if g2 >= p['gate2_catch_harmful_min'] else 'MISSED'}")
    print(f"  Gate 1 < {p['gate1_catch_harmful_max']:.0%}: {g1:.0%} -> {'held' if g1 < p['gate1_catch_harmful_max'] else 'MISSED'}")
    print(f"  false alarms Gate 1 = 0: {len(gr['false_alarms_gate1'])}; Gate 2 = 0: {len(gr['false_alarms_gate2'])}")
    return 0


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["seal"]:
        rec = seal()
        print(f"sealed {len(rec['corruptions'])} corruptions (+{N_RANDOM} random, seeded by the seal's commit) at "
              f"{rec['created_utc']}; commit and push benchmarks/gigo/PREREGISTRATION.json, then run with its hash")
        return 0
    if len(argv) == 2 and argv[0] == "run":
        out = run(argv[1])
        print(f"{len(out['rows'])} corruptions run; wrote {RUNS.relative_to(ROOT)}")
        return report()
    if argv[:1] == ["report"]:
        return report()
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
