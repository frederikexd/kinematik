# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The prediction ledger: KinematiK's predictions for real cars, sealed before the results are used, scored after.

Why: benchmarks show the logic matches other tools; they do not show that KinematiK describes real cars. The only
way to show that is to predict real cars and compare with measurements nobody here controls. Formula Student
competitions publish both halves: each team's car data (formulastudent.de team pages: mass split, wheelbase, track,
tyres, motors, ratio) and official skidpad and 75 m acceleration times measured by the organisers.

How a ledger entry works:
  1. specs: one row per car, copied from the public team page with its URL. Nothing else about the car is used.
  2. predict: every input the spec does not give is filled by the fixed PROTOCOL below (KinematiK's own defaults
     and the competition rules), never chosen per car and never tuned to results. Each input is labelled
     "published", "rule" or "assumed".
  3. seal: the predictions, the inputs behind them, a SHA-256 fingerprint of the code that made them, and the time
     are written to a record whose own SHA-256 is stored with it. Push the record to GitHub before the results are
     used; the commit time is then public proof of when the predictions existed.
  4. score: the official results are loaded and each prediction is compared: error, bias, how many within 5 %, and
     how well the predicted order matches the real order (Spearman rank correlation). Misses are listed by name.

Two kinds of entry, never mixed up (the record says which):
  * "blind": sealed before the competition ran. This is the real test.
  * "hindcast": sealed after the results were public (a past event, or a paper). The protocol still forbids
    per-car choices and tuning, but a reader has only our word that the results did not shape the protocol, so a
    hindcast is development evidence, not proof.

Pure Python + KinematiK's own models (dynamics, tiremodel, laptime): no Streamlit, no network.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import re
from dataclasses import asdict, dataclass
from datetime import datetime, UTC
from pathlib import Path

from .dynamics import VehicleDynamics, VehicleParams
from .laptime import SKIDPAD_RADIUS_M, MotorMap, Powertrain, acceleration_time, skidpad_time
from .tiremodel import default_tire

# ---------------------------------------------------------------------------
#  PROTOCOL: how a public spec becomes a prediction. Fixed for every car; changing anything here is a new
#  protocol version, and a record states the version it was made with.
# ---------------------------------------------------------------------------
PROTOCOL_VERSION = "1"
ACCEL_DISTANCE_M = 75.0                    # FS acceleration event length
LOADED_RADIUS_FACTOR = 0.97                # loaded / unloaded tyre radius
POWER_CAP_KW_BY_YEAR = {2012: 85.0}        # FS rules: accumulator power limit (85 kW in 2012, 80 kW since)
POWER_CAP_KW_DEFAULT = 80.0
TRACTION_TORQUE_MARGIN_G = 2.0             # motor torque assumed enough for this many g at the wheels, so a
                                           # launch is grip-limited (FS EVs are; no team publishes peak torque)
EXCLUDE_LR_SPLIT = 0.10                    # skidpad: left/right laps more than 10 % apart = an incident, not pace
EXCLUDE_PENALTY = True                     # a best time that includes a penalty is not a clean measurement
PROTOCOL_TEXT = {
    "mass": "published: front + rear weight with a 68 kg driver (FSG car data)",
    "weight_dist_front": "published: front weight / total",
    "wheelbase, tracks": "published",
    "cg_height": "assumed: KinematiK default (VehicleParams.cg_height), never per car",
    "tyre grip": "assumed: KinematiK default_tire(), never per car",
    "aero (ClA, CdA)": "assumed: KinematiK Powertrain defaults, never per car",
    "power": "rule: min(motors x published power per motor, FS accumulator power limit for the year)",
    "drive": "published: 4 motors = all-wheel drive, fewer = rear-wheel drive",
    "overall ratio": ("published: the primary ratio; primary x secondary only when the primary is below 5; "
                      "when front and rear ratios differ, the rear one"),
    "redline": "published max motor rpm",
    "wheel radius": f"published tyre diameter / 2 x {LOADED_RADIUS_FACTOR} (loaded)",
    "motor torque": f"assumed: enough for {TRACTION_TORQUE_MARGIN_G} g at the wheels (grip-limited launch)",
    "skidpad": f"KinematiK laptime.skidpad_time, radius {SKIDPAD_RADIUS_M} m",
    "acceleration": f"KinematiK laptime.acceleration_time over {ACCEL_DISTANCE_M} m from rest",
    "scoring exclusions": (f"skidpad: left/right laps more than {EXCLUDE_LR_SPLIT:.0%} apart, or any penalty in "
                           "the best time; listed but left out of the summary"),
}

_CODE_FILES = ("ledger.py", "laptime.py", "dynamics.py", "tiremodel.py")


@dataclass
class CarSpec:
    """One car as published (mm, kg, kW, rpm; ratios unitless). None = not published."""
    event: str
    car: str
    team: str
    year: int
    source_url: str
    wheelbase_mm: float | None = None
    track_f_mm: float | None = None
    track_r_mm: float | None = None
    weight_f_kg: float | None = None
    weight_r_kg: float | None = None
    tyres: str = ""
    n_motors: int | None = None
    power_per_motor_kw: float | None = None
    max_rpm: float | None = None
    ratio_primary: float | None = None
    ratio_secondary: float | None = None


# ---------------------------------------------------------------------------
#  Spec -> model
# ---------------------------------------------------------------------------
def tyre_radius_m(text: str) -> float | None:
    """Unloaded tyre radius (m) from a published size: '16.0x7.5-10', '18x6 R10', '20.0x7.0-13' (diameter in
    inches first) or metric '185/40 R13' (width mm / aspect % R rim inches). None if no size is recognised."""
    s = (text or "").replace(",", ".")
    m = re.search(r"(\d{3})\s*/\s*(\d{2})\s*Z?R?\s*(\d{2})", s)
    if m:
        width, aspect, rim = float(m.group(1)), float(m.group(2)), float(m.group(3))
        return (rim * 25.4 + 2.0 * width * aspect / 100.0) / 2000.0
    for a, b in re.findall(r"(\d+(?:\.\d+)?)\s*[xX]\s*(\d+(?:\.\d+)?)", s):
        for d_in in (float(a), float(b)):          # '16x7.5' (diameter first) or '7.5x16' (width first)
            if 15.0 <= d_in <= 24.0:
                return d_in * 25.4 / 2000.0
    return None


def overall_ratio(primary: float | None, secondary: float | None) -> float | None:
    """Motor-to-wheel ratio (unitless) per the protocol: the primary, or primary x secondary when the primary is
    below 5 (then it is clearly only the first stage)."""
    if primary is None or not primary > 0:
        return None
    if primary < 5.0 and secondary and secondary > 0:
        return primary * secondary
    return primary


def build(spec: CarSpec):
    """(VehicleDynamics, Powertrain, inputs) for a spec. `inputs` maps each model input to
    {"value", "unit", "basis"} with basis "published", "rule" or "assumed"; units as labelled (mm, kg, kW, m,
    rpm, N·m)."""
    d = VehicleParams()
    inputs = {}

    def _put(name, value, unit, basis):
        inputs[name] = {"value": value, "unit": unit, "basis": basis}
        return value

    wf, wr = spec.weight_f_kg, spec.weight_r_kg
    if wf and wr and wf > 0 and wr > 0:
        mass = _put("mass", wf + wr, "kg", "published")
        wdf = _put("weight_dist_front", wf / (wf + wr), "-", "published")
    else:
        mass = _put("mass", d.mass, "kg", "assumed")
        wdf = _put("weight_dist_front", d.weight_dist_front, "-", "assumed")
    wb = _put("wheelbase", spec.wheelbase_mm or d.wheelbase, "mm", "published" if spec.wheelbase_mm else "assumed")
    tf = _put("track_front", spec.track_f_mm or d.track_front, "mm", "published" if spec.track_f_mm else "assumed")
    tr = _put("track_rear", spec.track_r_mm or d.track_rear, "mm", "published" if spec.track_r_mm else "assumed")
    cg = _put("cg_height", d.cg_height, "mm", "assumed")
    params = VehicleParams(mass=mass, cg_height=cg, wheelbase=wb, track_front=tf, track_rear=tr,
                           weight_dist_front=wdf)
    veh = VehicleDynamics(params, tire=default_tire())

    pt0 = Powertrain()
    cap = POWER_CAP_KW_BY_YEAR.get(int(spec.year), POWER_CAP_KW_DEFAULT)
    if spec.n_motors and spec.power_per_motor_kw:
        power = _put("power", min(spec.n_motors * spec.power_per_motor_kw, cap), "kW", "rule")
    else:
        power = _put("power", cap, "kW", "rule")
    drive = _put("drive", "awd" if (spec.n_motors or 0) >= 4 else "rwd", "-",
                "published" if spec.n_motors else "assumed")
    r_unloaded = tyre_radius_m(spec.tyres)
    r = _put("wheel_radius", (r_unloaded * LOADED_RADIUS_FACTOR) if r_unloaded else 0.20, "m",
            "published" if r_unloaded else "assumed")
    _put("cla", pt0.cla, "m^2", "assumed")
    _put("cda", pt0.cda, "m^2", "assumed")
    ratio = overall_ratio(spec.ratio_primary, spec.ratio_secondary)
    motor_map = None
    if ratio and spec.max_rpm:
        _put("overall_ratio", ratio, "-", "published")
        _put("redline", spec.max_rpm, "rpm", "published")
        torque = TRACTION_TORQUE_MARGIN_G * mass * params.g * r / ratio / max(pt0.drivetrain_eff, 0.05)
        _put("motor_torque", torque, "N·m", "assumed")
        motor_map = MotorMap.from_peak(torque, power, spec.max_rpm, final_drive=ratio, wheel_radius_m=r)
    pt = Powertrain(power_kw=power, max_tractive_n=math.inf, drive=drive, motor_map=motor_map)
    return veh, pt, inputs


def predict(spec: CarSpec) -> dict:
    """KinematiK's predictions for one car: skidpad time (s, one timed circle) and 75 m acceleration time (s),
    with the inputs behind them and any model warnings."""
    veh, pt, inputs = build(spec)
    sk = skidpad_time(veh, pt)
    ac = acceleration_time(veh, pt, distance_m=ACCEL_DISTANCE_M)
    warnings = [w for w in (sk.warning, ac.warning) if w]
    return {"car": spec.car, "team": spec.team, "skidpad_s": round(float(sk.lap_time_s), 4),
            "accel_s": round(float(ac.lap_time_s), 4), "inputs": inputs, "warnings": warnings}


# ---------------------------------------------------------------------------
#  Files
# ---------------------------------------------------------------------------
_NUM = ("wheelbase_mm", "track_f_mm", "track_r_mm", "weight_f_kg", "weight_r_kg", "power_per_motor_kw", "max_rpm",
        "ratio_primary", "ratio_secondary")


def _num(v):
    v = (v or "").strip()
    if not v or v.upper() == "NA":
        return None
    return float(v.replace(",", "."))


def load_specs_csv(path) -> list:
    """CarSpec rows from a CSV with the CarSpec field names as headers (mm, kg, kW, rpm). NA or blank = None."""
    out = []
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            kw = {k: _num(row.get(k)) for k in _NUM}
            n = _num(row.get("n_motors"))
            out.append(CarSpec(event=row["event"].strip(), car=row["car"].strip(), team=row["team"].strip(),
                               year=int(row["year"]), source_url=row["source_url"].strip(),
                               tyres=(row.get("tyres") or "").strip(), n_motors=int(n) if n else None, **kw))
    return out


def load_results_csv(path) -> dict:
    """{car: row} from an official-results CSV with columns car, event (skidpad|accel), best_s, and optionally
    left_s, right_s, penalty_s, source_url. Times in seconds (s); the record itself is unitless."""
    out: dict = {}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            car, ev = row["car"].strip(), row["event"].strip()
            out.setdefault(car, {})[ev] = {
                "best_s": _num(row.get("best_s")), "left_s": _num(row.get("left_s")),
                "right_s": _num(row.get("right_s")), "penalty_s": _num(row.get("penalty_s")) or 0.0,
                "source_url": (row.get("source_url") or "").strip()}
    return out


def code_fingerprint(root=None) -> dict:
    """{file: sha256 hex} of the code that makes the predictions (unitless)."""
    base = Path(root) if root else Path(__file__).resolve().parent
    return {f: hashlib.sha256((base / f).read_bytes()).hexdigest() for f in _CODE_FILES}


def _canon(obj) -> bytes:
    return json.dumps(obj, sort_keys=True, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


# ---------------------------------------------------------------------------
#  Seal / verify
# ---------------------------------------------------------------------------
def seal(specs, event: str, kind: str, note: str = "", now: datetime | None = None) -> dict:
    """A sealed record of predictions for `specs`. kind is "blind" (sealed before the competition ran) or
    "hindcast" (results were already public). The record's own SHA-256 is stored under "record_sha256".
    Predicted times in seconds (s); hashes unitless."""
    if kind not in ("blind", "hindcast"):
        raise ValueError("kind must be 'blind' or 'hindcast'")
    rec = {
        "ledger": "KinematiK prediction ledger",
        "protocol_version": PROTOCOL_VERSION,
        "protocol": PROTOCOL_TEXT,
        "event": event,
        "kind": kind,
        "note": note,
        "sealed_utc": (now or datetime.now(UTC)).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "code_sha256": code_fingerprint(),
        "specs": [asdict(s) for s in specs],
        "predictions": [predict(s) for s in specs],
    }
    rec["record_sha256"] = hashlib.sha256(_canon(rec)).hexdigest()
    return rec


def verify(rec: dict) -> dict:
    """Checks on a sealed record: is it unaltered, and does today's code reproduce it? Returns
    {"intact", "same_code", "reproduced"} booleans (unitless)."""
    body = {k: v for k, v in rec.items() if k != "record_sha256"}
    intact = hashlib.sha256(_canon(body)).hexdigest() == rec.get("record_sha256")
    same_code = code_fingerprint() == rec.get("code_sha256")
    again = [predict(CarSpec(**s)) for s in rec.get("specs", [])]
    reproduced = [(p["skidpad_s"], p["accel_s"]) for p in again] == \
                 [(p["skidpad_s"], p["accel_s"]) for p in rec.get("predictions", [])]
    return {"intact": intact, "same_code": same_code, "reproduced": reproduced}


# ---------------------------------------------------------------------------
#  Score
# ---------------------------------------------------------------------------
def _ranks(xs):
    order = sorted(range(len(xs)), key=lambda i: xs[i])
    r = [0.0] * len(xs)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and xs[order[j + 1]] == xs[order[i]]:
            j += 1
        for k in range(i, j + 1):
            r[order[k]] = (i + j) / 2.0 + 1.0
        i = j + 1
    return r


def spearman(a, b) -> float:
    """Spearman rank correlation of two equal-length sequences (unitless, -1..1); NaN if fewer than 3 or no
    spread."""
    if len(a) != len(b) or len(a) < 3:
        return math.nan
    ra, rb = _ranks(list(a)), _ranks(list(b))
    ma, mb = sum(ra) / len(ra), sum(rb) / len(rb)
    cov = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    va = math.sqrt(sum((x - ma) ** 2 for x in ra))
    vb = math.sqrt(sum((y - mb) ** 2 for y in rb))
    return cov / (va * vb) if va > 0 and vb > 0 else math.nan


def _excluded(event, res) -> str:
    if res.get("best_s") is None:
        return "no valid official time"
    if EXCLUDE_PENALTY and (res.get("penalty_s") or 0.0) > 0:
        return "best time includes a penalty"
    if event == "skidpad":
        left, right = res.get("left_s"), res.get("right_s")
        if left and right and abs(left - right) / min(left, right) > EXCLUDE_LR_SPLIT:
            return f"left/right laps {left:.3f} / {right:.3f} s differ by more than {EXCLUDE_LR_SPLIT:.0%}"
    return ""


def score(rec: dict, results: dict) -> dict:
    """Compare a sealed record with official results. Per car and event: predicted and official time (s), error
    (s and %), and whether it is excluded (and why). Per event: n, mean absolute error (%), median absolute error
    (%), bias (%, + = predicted slower), within-5 % count, and Spearman rank correlation of predicted vs official
    order."""
    rows = []
    for p in rec["predictions"]:
        for event, key in (("skidpad", "skidpad_s"), ("accel", "accel_s")):
            res = results.get(p["car"], {}).get(event)
            if res is None:
                continue
            pred, off = p[key], res.get("best_s")
            why = _excluded(event, res)
            err = (pred - off) if off else math.nan
            rows.append({"car": p["car"], "team": p["team"], "event": event, "predicted_s": pred,
                         "official_s": off, "error_s": err,
                         "error_pct": 100.0 * err / off if off else math.nan, "excluded": why})
    summary = {}
    for event in ("skidpad", "accel"):
        used = [r for r in rows if r["event"] == event and not r["excluded"]]
        if not used:
            continue
        errs = sorted(abs(r["error_pct"]) for r in used)
        n = len(errs)
        med = errs[n // 2] if n % 2 else 0.5 * (errs[n // 2 - 1] + errs[n // 2])
        summary[event] = {
            "n": n,
            "excluded": sum(1 for r in rows if r["event"] == event and r["excluded"]),
            "mae_pct": sum(errs) / n,
            "median_abs_pct": med,
            "bias_pct": sum(r["error_pct"] for r in used) / n,
            "within_5pct": sum(1 for e in errs if e <= 5.0),
            "spearman": spearman([r["predicted_s"] for r in used], [r["official_s"] for r in used]),
        }
    return {"event": rec["event"], "kind": rec["kind"], "record_sha256": rec.get("record_sha256"),
            "rows": rows, "summary": summary}


def scorecard_markdown(sc: dict) -> str:
    """The scorecard as Markdown (text only, unitless): headline per event, then every car, misses included."""
    names = {"skidpad": "Skidpad", "accel": "Acceleration (75 m)"}
    lines = [f"# Prediction ledger scorecard: {sc['event']}", "",
             f"Kind: **{sc['kind']}**" + (" (sealed before the competition ran)" if sc["kind"] == "blind" else
                                          " (sealed after the results were public: development evidence, not proof)"),
             f"Record SHA-256: `{sc['record_sha256']}`", ""]
    for event, s in sc["summary"].items():
        rho = "n/a" if math.isnan(s["spearman"]) else format(s["spearman"], ".2f")
        lines += [f"## {names[event]}", "",
                  f"- Cars scored: {s['n']} ({s['excluded']} excluded, listed below)",
                  f"- Mean absolute error: {format(s['mae_pct'], '.1f')} %; median {format(s['median_abs_pct'], '.1f')} %",
                  f"- Bias: {format(s['bias_pct'], '+.1f')} % (+ = predicted slower than official)",
                  f"- Within 5 %: {s['within_5pct']} of {s['n']}",
                  f"- Rank correlation (Spearman, predicted vs official order): {rho}", ""]
    lines += ["## Every car", "", "| Car | Team | Event | Predicted (s) | Official (s) | Error | Note |",
              "|---|---|---|---|---|---|---|"]
    for r in sorted(sc["rows"], key=lambda r: (r["event"], r["official_s"] or 1e9)):
        off = "–" if r["official_s"] is None else format(r["official_s"], ".3f")
        err = "–" if math.isnan(r["error_pct"]) else format(r["error_pct"], "+.1f") + " %"
        lines.append(f"| {r['car']} | {r['team']} | {names[r['event']]} | {format(r['predicted_s'], '.3f')} | "
                     f"{off} | {err} | {r['excluded'] or ''} |")
    return "\n".join(lines) + "\n"
