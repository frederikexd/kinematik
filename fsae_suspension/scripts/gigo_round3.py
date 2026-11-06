#!/usr/bin/env python3
"""Garbage-in benchmark, round 3 (docs/GIGO_BENCHMARK_PLAN.md): the entry checks on decks they were not written
against, with 95 % confidence intervals.

Held-out decks, none used in rounds 1 and 2:

  fastsim:bolt   the 2020 Chevrolet Bolt EV bundled with FASTSim 3.1.0 (thermal models off), with FASTSim in the loop
                 for harm and the second opinion
  fsae_ev        the Formula Student EV of the powertrain benchmark's own deck (benchmarks/powertrain/inputs.json,
                 written before any of this: 300 kg, a 140s3p 504 V pack, 80 kW, a 7:1 reduction)
  fsae_ice       the Formula Student combustion car of the same deck (a restricted 600 cc four, six gears)

Harm on the Formula Student decks is measured by KinematiK's own models, validated against Simscape (energy,
current, voltage and the 75 m launch for the EV; fuel per km on the Hockenheim lap and the 75 m launch for the
combustion car). The corruptions are rounds 1 and 2's, applied to these decks, plus 60 random typing errors seeded by
this round's seal commit.

    python scripts/gigo_round3.py seal          # benchmarks/gigo/round3/PREREGISTRATION.json (once)
    python scripts/gigo_round3.py run COMMIT    # after the seal is pushed; COMMIT is its commit hash
    python scripts/gigo_round3.py report        # grades the stored run
"""
from __future__ import annotations

import copy
import csv
import subprocess
from contextlib import contextmanager
import hashlib
import json
import math
import random
import shutil
import sys
import tempfile
import warnings
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import fastsim_benchmark as fb  # noqa: E402
import gigo_benchmark as gb  # noqa: E402
import powertrain_benchmark as pb  # noqa: E402
from suspension import ice_powertrain as ice  # noqa: E402
from suspension.plausibility import PlausibilityWarning as PW  # noqa: E402

BOLT = ("2020 Chevrolet Bolt EV thrml.yaml", "ev", "as bundled")


@contextmanager
def _with_bolt():
    """The Bolt among FASTSim's vehicles only while it is used, so rounds 1 and 2's catalogs are untouched."""
    fb.VEHICLES["bolt"] = BOLT
    try:
        yield
    finally:
        fb.VEHICLES.pop("bolt", None)
BENCH = ROOT / "benchmarks" / "gigo" / "round3"
SEAL, RUNS = BENCH / "PREREGISTRATION.json", BENCH / "runs.json"
DECK = ROOT / "benchmarks" / "powertrain"
N_RANDOM = 60
SEALED_CODE = gb.SEALED_CODE + ("scripts/gigo_round3.py",)
PREDICTIONS = {"gate1_fsae_catch_harmful_min": 0.75, "gate1_catch_harmful_min": 0.50, "gate1_catch_harmful_max": 0.95,
               "gate2_catch_harmful_min": 0.95, "gate1_false_alarms": 0, "gate2_false_alarms": 0}
ICE_HARM = {"fuel_kg_per_km": 0.05, "launch_75m_time_s": 0.03}

CONFUSIONS = dict(gb.CONFUSIONS)
CONFUSIONS["voltage"] = gb.CONFUSIONS["voltage"] + (("pack_into_cell", "the pack voltage typed as the cell voltage",
                                                     "times_series"),)
EV_FIELDS = dict(gb.FSAE_FIELDS)
ICE_FIELDS = {**{k: v for k, v in gb.FSAE_FIELDS.items() if v[1][0] == "vehicle"},
              "crr": ("coefficient", ("vehicle", "crr")),
              "final_drive": ("ratio", ("ice", "final_drive")), "primary": ("ratio", ("ice", "primary")),
              "driveline_eff": ("efficiency", ("ice", "driveline_eff")), "bte": ("efficiency", ("ice", "bte")),
              "redline_rpm": ("speed", ("ice", "redline_rpm")), "launch_rpm": ("speed", ("ice", "launch_rpm")),
              "engine_torque": ("torque", ("_curve", "torque"))}
EV_FIELDS["crr"] = ("coefficient", ("vehicle", "crr"))
FIELDS = {"fsae_ev": EV_FIELDS, "fsae_ice": ICE_FIELDS}
CURATED_TARGETS = {          # curated key -> {deck: field}
    "lb_for_kg": {"fastsim:bolt": "mass_kg", "fsae_ev": "mass_kg", "fsae_ice": "mass_kg"},
    "driver_dropped": {"fastsim:bolt": "mass_kg", "fsae_ev": "mass_kg", "fsae_ice": "mass_kg"},
    "cd_for_cda": {"fastsim:bolt": "cda_m2"},
    "crr_percent": {"fastsim:bolt": "crr", "fsae_ev": "crr", "fsae_ice": "crr"},
    "dia_for_radius": {"fastsim:bolt": "wheel_radius_m", "fsae_ev": "tire_radius_m", "fsae_ice": "tire_radius_m"},
    "hp_for_kw": {"fastsim:bolt": "motor_power_w", "fsae_ev": "pack_power_kw"},
    "aux_kilo": {"fastsim:bolt": "aux_power_w"},
    "eff_percent": {"fastsim:bolt": "transmission_eff", "fsae_ev": "inverter_eff", "fsae_ice": "driveline_eff"},
    "eff_twice": {"fastsim:bolt": "transmission_eff", "fsae_ev": "inverter_eff", "fsae_ice": "driveline_eff"},
    "lbft_for_nm": {"fsae_ev": "motor_peak_torque_nm", "fsae_ice": "engine_torque"},
    "ratio_inverted": {"fsae_ev": "final_drive", "fsae_ice": "final_drive"},
    "rpm_for_rads": {"fsae_ev": "motor_max_rpm", "fsae_ice": "redline_rpm"},
    "cell_for_pack_v": {"fsae_ev": "cell_v"},
    "mah_for_ah": {"fsae_ev": "cell_ah"},
}


def wilson(k: int, n: int, z: float = 1.96) -> tuple:
    """95 % Wilson score interval for k successes in n trials (fractions)."""
    if n == 0:
        return math.nan, math.nan
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def _decks() -> list[str]:
    return ["fastsim:bolt", "fsae_ev", "fsae_ice"]


def systematic() -> list[dict]:
    out = []
    for field, (dim, scope, _) in gb.FS_FIELDS.items():
        if scope in ("all", "ev"):
            out += [{"set": "systematic", "deck": "fastsim:bolt", "field": field, "key": k, "story": s, "how": h}
                    for k, s, h in CONFUSIONS[dim]]
    for deck, fields in FIELDS.items():
        for field, (dim, _) in fields.items():
            out += [{"set": "systematic", "deck": deck, "field": field, "key": k, "story": s, "how": h}
                    for k, s, h in CONFUSIONS[dim]]
    return out


def curated() -> list[dict]:
    stories = {k: (s, h) for k, s, _scope, _f, h in gb.CURATED}
    return [{"set": "curated", "deck": deck, "field": field, "key": key, "story": stories[key][0],
             "how": stories[key][1]} for key, targets in CURATED_TARGETS.items() for deck, field in targets.items()]


def random_set(seed_commit: str) -> list[dict]:
    rng = random.Random(int(hashlib.sha256(("round3:" + seed_commit).encode()).hexdigest(), 16))
    pairs = [("fastsim:bolt", f) for f, (_, scope, _) in gb.FS_FIELDS.items()
             if scope in ("all", "ev") and not f.endswith("_map")]
    pairs += [(d, f) for d, fields in FIELDS.items() for f in fields]
    out = []
    for i in range(N_RANDOM):
        deck, field = rng.choice(pairs)
        kind = rng.choice(gb.TYPOS)
        out.append({"set": "random", "deck": deck, "field": field, "key": f"typo_{i:02d}_{kind}", "story": kind,
                    "how": {"typo": kind, "draw": rng.getrandbits(32)}})
    return out


# ---------------------------------------------------------------------------------------------------------------------
#  Decks and how each is evaluated
# ---------------------------------------------------------------------------------------------------------------------
def fs_deck(kind: str) -> dict:
    d = json.loads((DECK / "inputs.json").read_text())
    return {"vehicle": d["vehicle"], ("ev" if kind == "fsae_ev" else "ice"): d["ev" if kind == "fsae_ev" else "ice"]}


def _engine_curve():
    with open(DECK / "engine_curve.csv") as fh:
        rows = list(csv.DictReader(fh))
    return [float(r["rpm"]) for r in rows], [float(r["torque_nm"]) for r in rows]


def corrupt_fs(inp: dict, c: dict, kind: str) -> dict:
    inp = copy.deepcopy(inp)
    dim, path = FIELDS[kind][c["field"]]
    if path[0] == "_curve":                               # the engine's torque curve, every point
        inp["_torque_scale"] = gb.apply_value(1.0, c["how"]) if not isinstance(c["how"], dict) else None
        if isinstance(c["how"], dict):
            inp["_torque_typo"] = c["how"]
        return inp
    obj = inp
    for k in path[:-1]:
        obj = obj[k]
    if c["how"] == "times_series":
        obj[path[-1]] = obj[path[-1]] * inp["ev"]["pack"]["series"]
    else:
        obj[path[-1]] = gb.apply_value(obj[path[-1]], c["how"], deck=inp)
    return inp


def _lap():
    log = pb._from(pb.read_motec_csv(gb.FS_DECK / gb.FS_LAP / "simulink_lap.csv"), 0.0)
    log = pb._from(log, log["time"][0] + 3.0)
    return log


def _ice_eval(inp: dict) -> tuple:
    """KinematiK's fuel per km on the Hockenheim lap and its 75 m launch for the combustion deck, and whether it
    refused or warned."""
    flags, out = [], {}
    rpm, tq = _engine_curve()
    if "_torque_typo" in inp:
        tq = [gb.apply_value(t, inp["_torque_typo"]) for t in tq]
    elif inp.get("_torque_scale"):
        tq = [t * inp["_torque_scale"] for t in tq]
    deck = {k: v for k, v in inp.items() if not k.startswith("_")}
    with tempfile.TemporaryDirectory() as tmp, warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        tmp = Path(tmp)
        with open(tmp / "engine_curve.csv", "w", newline="") as fh:
            wr = csv.writer(fh, lineterminator="\n")
            wr.writerow(["rpm", "torque_nm"])
            wr.writerows(zip(rpm, tq))
        try:
            pb.check_deck(deck, where="inputs.json", base=tmp)
            veh, ic = deck["vehicle"], deck["ice"]
            eng, gbx, iv = pb._ice_parts(veh, ic, tmp)
            la = ice.simulate_launch(eng, gbx, iv, fuel_key=ic["fuel"], bte=ic["bte"], launch_rpm=ic["launch_rpm"])
            out["launch_75m_time_s"] = float(la.t_s[-1]) if la.finished else math.inf
            lap = _lap()
            s = lap["distance"]
            r = ice.fuel_for_lap(SimpleNamespace(s=s, v=lap["speed"]), eng, gbx, iv, fuel_key=ic["fuel"],
                                 bte=ic["bte"])
            out["fuel_kg_per_km"] = r.fuel_kg / (float(s[-1] - s[0]) / 1e3)
        except ValueError as exc:
            flags.append(f"refused: {exc}")
        except Exception as exc:  # noqa: BLE001
            flags.append(f"crashed: {type(exc).__name__}")
    flags += sorted({f"warned: {x.message}" for x in w if issubclass(x.category, PW)})
    return out, flags


def _ev_eval(inp: dict) -> tuple:
    """KinematiK's lap metrics (on hock_fs2's Hockenheim lap) and 75 m launch for the EV deck, and whether it
    refused or warned. As round 2's evaluation, but only KinematiK's own PlausibilityWarnings count as warnings."""
    flags, out = [], {}
    with tempfile.TemporaryDirectory() as tmp, warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        tmp = Path(tmp)
        shutil.copy(DECK / "motor_curve.csv", tmp / "motor_curve.csv")
        (tmp / "inputs.json").write_text(json.dumps(inp))
        try:
            rows = pb.simulink_lap_rows(tmp / "inputs.json", gb.FS_DECK / gb.FS_LAP / "simulink_lap.csv",
                                        skip_s=3.0)["rows"]
            out.update({m: k for m, k, _ in rows})
            try:
                for m, k, _ in pb.blind_launch_rows(tmp / "inputs.json", gb.FS_DECK / gb.FS_LAUNCH / "simulink_lap.csv"):
                    out[m] = k
            except ValueError as exc:
                if "never reaches 75 m" not in str(exc):
                    raise
                out["launch_75m_time_s"] = math.inf
                flags.append("result: the launch never reached 75 m")
        except ValueError as exc:
            flags.append(f"refused: {exc}")
        except Exception as exc:  # noqa: BLE001
            flags.append(f"crashed: {type(exc).__name__}")
    flags += sorted({f"warned: {x.message}" for x in w if issubclass(x.category, PW)})
    return out, flags


def bolt_clean(d: dict) -> dict:
    """FASTSim on the clean Bolt: energy per km and the achieved speed trace per cycle (KinematiK drives that
    trace, as in FASTSIM_BENCHMARK.md)."""
    import numpy as np
    import fastsim as fs
    out = {}
    for cyc, fname in fb.CYCLES.items():
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
        e = float(np.sum(df["veh.pt_type.BEV.res.history.pwr_out_electrical_watts"].to_numpy() * dt)) / 3.6e3 / km
        out[cyc] = {"energy_wh_km": e, "t": [float(x) for x in ts],
                    "v": [float(x) for x in df["veh.history.speed_ach_meters_per_second"].to_numpy()]}
    return out


def bolt_dict() -> dict:
    with _with_bolt():
        return gb.fastsim_dict("bolt")


def fastsim_run_bolt(d: dict) -> dict:
    """FASTSim on a (corrupted) Bolt in a process of its own, as round 2 runs its decks: a crash of FASTSim's
    compiled core is FASTSim refusing the deck."""
    with tempfile.TemporaryDirectory() as tmp:
        f = Path(tmp) / "deck.json"
        f.write_text(json.dumps(d))
        try:
            proc = subprocess.run([sys.executable, "-I", str(Path(__file__).resolve()), "_fastsim", str(f)],
                                  capture_output=True, text=True, timeout=600)
        except subprocess.TimeoutExpired:
            return {cyc: {"refused": "FASTSim did not finish in 600 s"} for cyc in fb.CYCLES}
    if proc.returncode != 0:
        return {cyc: {"refused": f"FASTSim crashed (exit {proc.returncode})"} for cyc in fb.CYCLES}
    return json.loads(proc.stdout.strip().splitlines()[-1])


def _kinematik_bolt(d: dict, traces: dict) -> tuple:
    import numpy as np
    import fastsim as fs
    with _with_bolt():
        p = fb._params(fs, d, "bolt")
    flags, res = [], {}
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        for cyc, tr in traces.items():
            try:
                res[cyc] = fb.ev_wh_km(p, np.asarray(tr["t"]), np.asarray(tr["v"]))
            except ValueError as exc:
                flags.append(f"refused: {exc}")
                res[cyc] = None
            except Exception as exc:  # noqa: BLE001
                flags.append(f"crashed: {type(exc).__name__}")
                res[cyc] = None
    flags += sorted({f"warned: {x.message}" for x in w if issubclass(x.category, PW)})
    return res, flags


# ---------------------------------------------------------------------------------------------------------------------
#  Seal, run, grade
# ---------------------------------------------------------------------------------------------------------------------
def code_fingerprint(root=None) -> dict:
    root = Path(root or ROOT)
    return {f: gb._sha256(root / f) for f in SEALED_CODE}


def seal(now=None) -> dict:
    if SEAL.exists():
        raise SystemExit(f"{SEAL} is written once")
    BENCH.mkdir(parents=True, exist_ok=True)
    rec = {"created_utc": (now or datetime.now(UTC)).strftime("%Y-%m-%dT%H:%M:%SZ"),
           "plan": "docs/GIGO_BENCHMARK_PLAN.md (round 3)", "decks": _decks(),
           "deck_files": {"benchmarks/powertrain/inputs.json": gb._sha256(DECK / "inputs.json"),
                          "benchmarks/powertrain/motor_curve.csv": gb._sha256(DECK / "motor_curve.csv"),
                          "benchmarks/powertrain/engine_curve.csv": gb._sha256(DECK / "engine_curve.csv")},
           "fastsim_vehicle": BOLT[0], "fastsim_version": fb.FASTSIM_VERSION,
           "harm": {"fastsim": gb.HARM_REL, "fsae_ev": gb.FSAE_HARM, "fsae_ice": ICE_HARM}, "gate2_rel": gb.GATE2_REL,
           "random": {"n": N_RANDOM, "seed": "the commit hash that adds this file, prefixed 'round3:'"},
           "corruptions": systematic() + curated(), "predictions": PREDICTIONS, "code_sha256": code_fingerprint()}
    SEAL.write_text(json.dumps(rec, indent=1, ensure_ascii=False) + "\n")
    return rec


def run(seed_commit: str) -> dict:
    import fastsim as fs
    rec = json.loads(SEAL.read_text())
    changed = [f for f, h in rec["code_sha256"].items() if gb._sha256(ROOT / f) != h]
    if changed or fs.__version__ != rec["fastsim_version"]:
        raise SystemExit(f"not the sealed code or FASTSim: {changed} {fs.__version__}")
    d = bolt_dict()
    clean = bolt_clean(d)
    traces = {c: {"t": x["t"], "v": x["v"]} for c, x in clean.items()}
    out = {"seed_commit": seed_commit, "run_utc": datetime.now(UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
           "clean": {"fastsim:bolt": {"fastsim": {c: {"energy_wh_km": x["energy_wh_km"]} for c, x in clean.items()},
                                      "kinematik": _kinematik_bolt(d, traces)},
                     "fsae_ev": {"kinematik": _ev_eval(fs_deck("fsae_ev"))},
                     "fsae_ice": {"kinematik": _ice_eval(fs_deck("fsae_ice"))}},
           "rows": []}
    for c in rec["corruptions"] + random_set(seed_commit):
        row = {k: c[k] for k in ("set", "deck", "field", "key", "story")}
        try:
            if c["deck"] == "fastsim:bolt":
                dc = gb.corrupt_fastsim(d, c)
                if json.dumps(dc) == json.dumps(d):
                    row["no_change"] = True
                else:
                    row["fastsim"] = fastsim_run_bolt(dc)
                    row["kinematik"] = _kinematik_bolt(dc, traces)
            else:
                base = fs_deck(c["deck"])
                ci = corrupt_fs(base, c, c["deck"])
                if json.dumps(ci) == json.dumps(base) or ci.get("_torque_scale") == 1.0:
                    row["no_change"] = True
                else:
                    row["kinematik"] = (_ev_eval if c["deck"] == "fsae_ev" else _ice_eval)(ci)
        except Exception as exc:  # noqa: BLE001
            row["not_applied"] = f"{type(exc).__name__}: {exc}"
        out["rows"].append(row)
    RUNS.write_text(json.dumps(out, indent=1, ensure_ascii=False) + "\n")
    (BENCH / "MANIFEST.sha256").write_text(f"{gb._sha256(RUNS)}  runs.json\n")
    return out


def grade(runs=None) -> dict:
    runs = runs or json.loads(RUNS.read_text())
    rows = []
    for r in runs["rows"]:
        g = {k: r[k] for k in ("set", "deck", "field", "key", "story")}
        if "not_applied" in r or r.get("no_change"):
            g["status"] = "not applied" if "not_applied" in r else "no change"
            rows.append(g)
            continue
        clean = runs["clean"][r["deck"]]
        res, flags = r["kinematik"]
        new = [f for f in flags if f not in clean["kinematik"][1]]   # where= names a fixed file, so messages compare
        g.update(gate1=any(f.startswith(("refused", "warned")) for f in new),
                 crashed=any(f.startswith("crashed") for f in new), flags=new, gate2=None, tool_refused=False)
        if r["deck"] == "fastsim:bolt":
            harm, g2 = [], []
            for cyc, fsr in r["fastsim"].items():
                if "refused" in fsr:
                    continue
                harm.append(abs(fsr["energy_wh_km"] / clean["fastsim"][cyc]["energy_wh_km"] - 1))
                g2.append(abs(fsr["energy_wh_km"] / clean["kinematik"][0][cyc] - 1) > gb.GATE2_REL)
            g.update(tool_refused=not harm, harm=max(harm) if harm else None, harmful=bool(harm) and max(harm) > gb.HARM_REL,
                     gate2=any(g2))
        else:
            bars = gb.FSAE_HARM if r["deck"] == "fsae_ev" else ICE_HARM
            c0 = clean["kinematik"][0]
            moves = {m: abs(res[m] / c0[m] - 1) for m in bars if m in res and c0.get(m)}
            g.update(harm=max(moves.values()) if moves else None, harmful=any(moves[m] > bars[m] for m in moves))
        rows.append(g)
    fa1 = [f"{d}: {f}" for d, v in runs["clean"].items() for f in v["kinematik"][1]]
    fa2 = [f"bolt {c}" for c, x in runs["clean"]["fastsim:bolt"]["fastsim"].items()
           if abs(x["energy_wh_km"] / runs["clean"]["fastsim:bolt"]["kinematik"][0][c] - 1) > gb.GATE2_REL]
    return {"rows": rows, "false_alarms_gate1": fa1, "false_alarms_gate2": fa2}


def summarize(gr: dict) -> dict:
    out = {}
    live = [r for r in gr["rows"] if not r.get("status")]
    for name, sel in (("all", lambda r: True), ("formula student", lambda r: r["deck"] != "fastsim:bolt"),
                      ("fsae_ev", lambda r: r["deck"] == "fsae_ev"), ("fsae_ice", lambda r: r["deck"] == "fsae_ice"),
                      ("fastsim:bolt", lambda r: r["deck"] == "fastsim:bolt")):
        rows = [r for r in live if sel(r)]
        h = [r for r in rows if r["harmful"]]
        g2 = [r for r in h if r["gate2"] is not None]
        k1, k2 = sum(r["gate1"] for r in h), sum(r["gate2"] for r in g2)
        out[name] = {"errors": len(rows), "harmful": len(h), "gate1": k1, "gate1_ci": wilson(k1, len(h)),
                     "gate2": k2, "gate2_n": len(g2), "gate2_ci": wilson(k2, len(g2)),
                     "either": sum(r["gate1"] or bool(r["gate2"]) for r in h),
                     "tool_refused": sum(r["tool_refused"] for r in rows), "crashed": sum(r["crashed"] for r in rows),
                     "flagged_harmless": sum(r["gate1"] for r in rows if not r["harmful"]),
                     "silent_harmful": [f"{r['deck']} {r['field']} {r['key']} ({r['harm']:.0%})" for r in h
                                        if not r["gate1"] and not r["gate2"]]}
    out["no_change"] = sum(r.get("status") == "no change" for r in gr["rows"])
    out["not_applied"] = sum(r.get("status") == "not applied" for r in gr["rows"])
    return out


def report() -> int:
    rec = json.loads(SEAL.read_text())
    gr = grade()
    sm = summarize(gr)
    ci = lambda c: f"[{c[0]:.0%}, {c[1]:.0%}]"  # noqa: E731
    for name in ("all", "formula student", "fsae_ev", "fsae_ice", "fastsim:bolt"):
        v = sm[name]
        print(f"\n{name}: {v['errors']} errors, {v['harmful']} harmful")
        if v["harmful"]:
            print(f"  Gate 1, at entry:       {v['gate1']}/{v['harmful']} ({v['gate1'] / v['harmful']:.0%}, 95 % CI "
                  f"{ci(v['gate1_ci'])})")
        if v["gate2_n"]:
            print(f"  Gate 2, second opinion: {v['gate2']}/{v['gate2_n']} ({v['gate2'] / v['gate2_n']:.0%}, 95 % CI "
                  f"{ci(v['gate2_ci'])})")
        print(f"  either: {v['either']}/{v['harmful']}; also flagged {v['flagged_harmless']} harmless wrong values; "
              f"FASTSim refused {v['tool_refused']}; KinematiK crashed {v['crashed']}")
        if name not in ("all", "formula student"):
            for x in v["silent_harmful"]:
                print(f"    silent and harmful: {x}")
    print(f"\nleft unchanged (not counted): {sm['no_change']}; not applied: {sm['not_applied']}")
    print(f"false alarms, Gate 1: {gr['false_alarms_gate1'] or 'none'}")
    print(f"false alarms, Gate 2: {gr['false_alarms_gate2'] or 'none'}")
    p, a, f = rec["predictions"], sm["all"], sm["formula student"]
    g1, g1f = a["gate1"] / a["harmful"], f["gate1"] / f["harmful"] if f["harmful"] else math.nan
    g2 = a["gate2"] / a["gate2_n"] if a["gate2_n"] else math.nan
    ok = lambda c: "held" if c else "MISSED"  # noqa: E731
    print("\nsealed predictions:")
    print(f"  Gate 1 on the Formula Student decks >= {p['gate1_fsae_catch_harmful_min']:.0%}: {g1f:.0%} -> "
          f"{ok(g1f >= p['gate1_fsae_catch_harmful_min'])}")
    print(f"  Gate 1 overall in [{p['gate1_catch_harmful_min']:.0%}, {p['gate1_catch_harmful_max']:.0%}): {g1:.0%} -> "
          f"{ok(p['gate1_catch_harmful_min'] <= g1 < p['gate1_catch_harmful_max'])}")
    print(f"  Gate 2 >= {p['gate2_catch_harmful_min']:.0%}: {g2:.0%} -> {ok(g2 >= p['gate2_catch_harmful_min'])}")
    print(f"  false alarms Gate 1 = 0: {len(gr['false_alarms_gate1'])}; Gate 2 = 0: {len(gr['false_alarms_gate2'])}")
    return 0


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["seal"]:
        rec = seal()
        print(f"sealed {len(rec['corruptions'])} corruptions (+{N_RANDOM} random) at {rec['created_utc']}")
        return 0
    if len(argv) == 2 and argv[0] == "run":
        run(argv[1])
        return report()
    if argv[:1] == ["report"]:
        return report()
    if len(argv) == 2 and argv[0] == "_fastsim":           # one Bolt deck, in its own process (fastsim_run_bolt)
        with _with_bolt():
            print(json.dumps(gb._fastsim_run_here(json.loads(Path(argv[1]).read_text()), "bolt")))
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
