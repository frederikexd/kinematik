# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Prediction ledger, protocol version 2: the same physics, calibrated on one competition and tested on another.

Protocol v1 (ledger.py) fills every unpublished input with KinematiK's defaults and nothing else. On FSG 2026 that
left a shared offset (skidpad about 10 % optimistic, acceleration about 5 %) and a few misses caused by the teams'
own data. Version 2 keeps v1's physics and adds five fixed rules. Each one is the same for every car and is fitted
only on a training competition, never on the one being predicted:

  1. Shared calibration. One time factor for skidpad per tyre compound group (R20, LC0, other; a group with fewer
     than MIN_GROUP training cars uses the pooled factor) and one for acceleration, each the median of
     official / predicted over the training competition's clean runs.
  2. Data that contradicts itself is set aside. A published value outside the plausible Formula Student range
     (PLAUSIBLE) is treated as not published. A redline and ratio that cap the car below MIN_REDLINE_SPEED_MS are
     treated as a data error: the motor map is dropped and the power limit applies.
  3. Published information v1 ignored: the tyre compound (step 1 groups by it) and the tyre size (the wheel radius,
     including '175/518 R13' style sizes where 518 is the diameter in mm). Aero is NOT used: teams' aero mentions
     proved unreliable (TU Delft and GFR run wings in 2026 but their entries don't say so).
  4. Team track record. A team's ratio of official to calibrated prediction in the previous competition, shrunk
     halfway to 1 (PRIOR_SHRINK), multiplies this year's prediction. It uses only results public before the event.
     Kept for skidpad only (TEAM_RECORD_EVENTS): in the cross-year tests it helped skidpad both ways and
     acceleration only one way.
  5. Skidpad as driven: the path radius is the rules' inner circle (7.625 m) plus a fixed clearance plus half the
     car's own track, not v1's fixed 9.125 m, so a wide car runs a longer lap. The faster of the left and right laps
     is scored alongside the official average.

Pure Python + KinematiK's own models. The physics is v1's (ledger.build), so v1 records stay reproducible.
"""
from __future__ import annotations

import hashlib
import math
import re
import statistics
from dataclasses import asdict, replace
from datetime import UTC, datetime
from pathlib import Path

from . import ledger as L
from .laptime import acceleration_time, skidpad_time

PROTOCOL_VERSION = "2"
SKIDPAD_INNER_RADIUS_M = 7.625          # FS rules: inner circle 15.25 m in diameter
SKIDPAD_CLEARANCE_M = 0.25              # inner tyre to the cones, fixed for every car
MIN_REDLINE_SPEED_MS = 25.0             # an 80 kW FS car passes ~30 m/s within 75 m; a cap below 25 m/s is a typo
MIN_GROUP = 4                           # training cars a compound group needs for its own skidpad factor
PRIOR_SHRINK = 0.5                      # weight of last year's team ratio (0 = ignore, 1 = trust fully)
ACCEL_QUANTILE = 0.5                    # v2: the acceleration factor is the median ratio, as declared and tested
CHALLENGER_ACCEL_QUANTILE = 0.25        # pre-registered challenger: calibrate to the fastest quarter of the field
                                        # (cars that ran to their spec). Proposed after the 2026 test was seen, so
                                        # it is NOT adopted: both are sealed blind for FSG 2027 and that decides.
TEAM_RECORD_EVENTS = ("skidpad",)       # rule 4 kept only where it cut the error in BOTH cross-year tests:
                                        # skidpad yes; acceleration no (forward 6.8 -> 7.7 %), so not used there
PLAUSIBLE = {"wheelbase_mm": (1525.0, 2000.0), "track_f_mm": (900.0, 1600.0), "track_r_mm": (900.0, 1600.0),
             "weight_f_kg": (50.0, 250.0), "weight_r_kg": (50.0, 250.0), "power_per_motor_kw": (3.0, 150.0),
             "max_rpm": (2000.0, 30000.0), "ratio_primary": (1.0, 30.0)}
PROTOCOL_TEXT = {
    "physics": "protocol v1 (ledger.build): published spec, FS power limit, KinematiK defaults for the rest",
    "1 calibration": ("time factor = median(official / predicted) on the training competition's clean runs; skidpad "
                      f"per compound group (R20, LC0, other; pooled below {MIN_GROUP} cars), acceleration pooled"),
    "2 data checks": (f"published values outside {PLAUSIBLE} are treated as not published; a redline and ratio "
                      f"capping the car below {MIN_REDLINE_SPEED_MS} m/s drop the motor map"),
    "3 published inputs": "tyre compound (calibration group) and tyre diameter; aero not used (unreliable)",
    "4 team record": (f"prediction x (1 + {PRIOR_SHRINK} x (last competition's official / calibrated prediction"
                      f" - 1)), same team (FSG team id), clean runs only; events {TEAM_RECORD_EVENTS}"),
    "5 skidpad": (f"path radius {SKIDPAD_INNER_RADIUS_M} m + {SKIDPAD_CLEARANCE_M} m + half the mean track; the "
                  "faster of left and right laps is scored as a second channel"),
}
_CODE_FILES = ("ledger_v2.py",) + L._CODE_FILES


# ---------------------------------------------------------------------------
#  Rules 2, 3 and 5: spec -> physics prediction
# ---------------------------------------------------------------------------
def tyre_radius_m(text: str) -> float | None:
    """Unloaded tyre radius (m). Adds '175 / 518 R13' sizes (width mm / diameter mm) to ledger.tyre_radius_m."""
    m = re.search(r"\d{3}\s*/\s*(\d{3})\s*R?\s*\d{2}", (text or ""))
    if m and 400 <= float(m.group(1)) <= 650:
        return float(m.group(1)) / 2000.0
    return L.tyre_radius_m(text)


def compound(text: str) -> str:
    """Tyre compound group from the published tyre text: 'R20', 'LC0' or 'other' (unitless)."""
    t = (text or "").upper()
    r20, lc0 = "R20" in t, "LC0" in t
    if r20 and not lc0:
        return "R20"
    if lc0 and not r20:
        return "LC0"
    return "other"


def team_id(spec) -> str:
    """The team's identity across years (unitless): the FSG team id from its page URL, else the team name."""
    m = re.search(r"/tid/(\d+)", spec.source_url or "")
    return m.group(1) if m else spec.team


def sanitize(spec):
    """(spec with implausible or self-contradicting published values removed, {field: rejected value}).
    Values in mm, kg, kW, rpm."""
    rejected, fixes = {}, {}
    for f, (lo, hi) in PLAUSIBLE.items():
        v = getattr(spec, f)
        if v is not None and not (lo <= v <= hi):
            rejected[f], fixes[f] = v, None
    s = replace(spec, **fixes)
    ratio = L.overall_ratio(s.ratio_primary, s.ratio_secondary)
    r = tyre_radius_m(s.tyres)
    if ratio and s.max_rpm:
        v_cap = s.max_rpm * 2.0 * math.pi / 60.0 * (r or 0.20) * L.LOADED_RADIUS_FACTOR / ratio
        if v_cap < MIN_REDLINE_SPEED_MS:
            rejected["redline_speed_ms"] = round(v_cap, 2)
            s = replace(s, max_rpm=None, ratio_primary=None, ratio_secondary=None)
    if r and L.tyre_radius_m(s.tyres) is None:      # size only this module reads: hand it on in inch form
        s = replace(s, tyres=f"{s.tyres} ({r * 2000.0 / 25.4:.2f}x0)")
    return s, rejected


def physics(spec) -> dict:
    """v2 physics for one car before calibration: skidpad time (s) on the car's own path radius (m) and 75 m
    acceleration time (s), with the inputs, rejected values, compound group and team id."""
    s, rejected = sanitize(spec)
    veh, pt, inputs = L.build(s)
    track_m = (inputs["track_front"]["value"] + inputs["track_rear"]["value"]) / 2000.0
    radius = SKIDPAD_INNER_RADIUS_M + SKIDPAD_CLEARANCE_M + track_m / 2.0
    inputs["skidpad_radius"] = {"value": round(radius, 4), "unit": "m", "basis": "rule"}
    sk = skidpad_time(veh, pt, radius_m=radius)
    ac = acceleration_time(veh, pt, distance_m=L.ACCEL_DISTANCE_M)
    return {"car": spec.car, "team": spec.team, "team_id": team_id(spec), "compound": compound(spec.tyres),
            "skidpad_s": float(sk.lap_time_s), "accel_s": float(ac.lap_time_s), "inputs": inputs,
            "rejected": rejected, "warnings": [w for w in (sk.warning, ac.warning) if w]}


# ---------------------------------------------------------------------------
#  Rules 1 and 4: calibration and team record
# ---------------------------------------------------------------------------
def _clean_pairs(phys, results):
    out = []
    for p in phys:
        for ev, key in (("skidpad", "skidpad_s"), ("accel", "accel_s")):
            res = results.get(p["car"], {}).get(ev)
            if res and not L._excluded(ev, res) and p[key] > 0:
                out.append((p, ev, res["best_s"] / p[key]))
    return out


def _quantile(xs, q: float) -> float:
    """Linear-interpolation quantile (unitless q in 0..1) of a non-empty list, as numpy's default."""
    xs = sorted(xs)
    pos = q * (len(xs) - 1)
    lo = math.floor(pos)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def fit(phys, results, source: str, accel_quantile: float = ACCEL_QUANTILE) -> dict:
    """Shared time factors (unitless) from one or more training competitions: {"skidpad": {group: factor},
    "skidpad_pooled", "accel", "accel_quantile", "n": {...}, "source"}. `phys` are physics() rows, `results` the
    matching official results keyed by car. The acceleration factor is the `accel_quantile` of official /
    predicted (0.5 = median, the v2 rule)."""
    pairs = _clean_pairs(phys, results)
    sk = [(p["compound"], r) for p, ev, r in pairs if ev == "skidpad"]
    ac = [r for _p, ev, r in pairs if ev == "accel"]
    pooled = statistics.median(r for _g, r in sk) if sk else 1.0
    groups = {}
    for g in ("R20", "LC0", "other"):
        rs = [r for gg, r in sk if gg == g]
        groups[g] = statistics.median(rs) if len(rs) >= MIN_GROUP else pooled
    return {"skidpad": groups, "skidpad_pooled": pooled, "accel": _quantile(ac, accel_quantile) if ac else 1.0,
            "accel_quantile": accel_quantile,
            "n": {"skidpad": len(sk), "accel": len(ac),
                  **{f"skidpad_{g}": sum(1 for gg, _r in sk if gg == g) for g in ("R20", "LC0", "other")}},
            "source": source}


def calibrated(p: dict, calib: dict) -> dict:
    """Physics times (s) times the shared factors (unitless): {"skidpad_s", "accel_s"}."""
    return {"skidpad_s": p["skidpad_s"] * calib["skidpad"].get(p["compound"], calib["skidpad_pooled"]),
            "accel_s": p["accel_s"] * calib["accel"]}


def team_record(phys, results, calib) -> dict:
    """{team_id: {"skidpad": ratio, "accel": ratio}}: official / calibrated prediction (unitless) on clean runs."""
    out: dict = {}
    for p, ev, _r in _clean_pairs(phys, results):
        c = calibrated(p, calib)[f"{ev}_s"]
        out.setdefault(p["team_id"], {})[ev] = results[p["car"]][ev]["best_s"] / c
    return out


def predict(p: dict, calib: dict, record: dict | None = None, record_events=TEAM_RECORD_EVENTS) -> dict:
    """A v2 prediction (s; team-record ratios unitless) from a physics() row: calibrated, then the team's record applied (for the events in
    record_events) if it has one."""
    c = calibrated(p, calib)
    used = {}
    for ev in record_events:
        r = ((record or {}).get(p["team_id"]) or {}).get(ev)
        if r is not None:
            c[f"{ev}_s"] *= 1.0 + PRIOR_SHRINK * (r - 1.0)
            used[ev] = round(r, 4)
    return {"car": p["car"], "team": p["team"], "skidpad_s": round(c["skidpad_s"], 4),
            "accel_s": round(c["accel_s"], 4), "physics_skidpad_s": round(p["skidpad_s"], 4),
            "physics_accel_s": round(p["accel_s"], 4), "compound": p["compound"], "team_record": used,
            "rejected": p["rejected"], "inputs": p["inputs"], "warnings": p["warnings"]}


# ---------------------------------------------------------------------------
#  Seal / verify / best-lap channel
# ---------------------------------------------------------------------------
def code_fingerprint() -> dict:
    """{file: sha256 hex} of the v2 code and the v1 physics it uses (unitless)."""
    base = Path(__file__).resolve().parent
    return {f: hashlib.sha256((base / f).read_bytes()).hexdigest() for f in _CODE_FILES}


def seal(specs, event: str, kind: str, calib: dict, record: dict | None, note: str = "",
         now: datetime | None = None, record_events=TEAM_RECORD_EVENTS) -> dict:
    """A sealed v2 record (times in s; hashes unitless). The calibration and team record go into the record, so
    it can be reproduced and checked without anything else."""
    if kind not in ("blind", "hindcast"):
        raise ValueError("kind must be 'blind' or 'hindcast'")
    rec = {"ledger": "KinematiK prediction ledger", "protocol_version": PROTOCOL_VERSION,
           "protocol": PROTOCOL_TEXT, "event": event, "kind": kind, "note": note,
           "sealed_utc": (now or datetime.now(UTC)).strftime("%Y-%m-%dT%H:%M:%SZ"),
           "code_sha256": code_fingerprint(), "calibration": calib, "team_record": record or {},
           "team_record_events": list(record_events), "specs": [asdict(s) for s in specs],
           "predictions": [predict(physics(s), calib, record, record_events) for s in specs]}
    rec["record_sha256"] = hashlib.sha256(L._canon(rec)).hexdigest()
    return rec


def verify(rec: dict) -> dict:
    """{"intact", "same_code", "reproduced"} for a v2 record (unitless booleans)."""
    body = {k: v for k, v in rec.items() if k != "record_sha256"}
    intact = hashlib.sha256(L._canon(body)).hexdigest() == rec.get("record_sha256")
    events = tuple(rec.get("team_record_events", TEAM_RECORD_EVENTS))
    again = [predict(physics(L.CarSpec(**s)), rec["calibration"], rec.get("team_record"), events)
             for s in rec.get("specs", [])]
    reproduced = [(p["skidpad_s"], p["accel_s"]) for p in again] == \
                 [(p["skidpad_s"], p["accel_s"]) for p in rec.get("predictions", [])]
    return {"intact": intact, "same_code": code_fingerprint() == rec.get("code_sha256"), "reproduced": reproduced}


def best_lap_summary(rec: dict, results: dict) -> dict:
    """Skidpad scored against the faster of the left and right laps (s), clean runs only: {"n", "mae_pct",
    "bias_pct", "within_5pct"}; empty if no run has both laps."""
    errs = []
    for p in rec["predictions"]:
        res = results.get(p["car"], {}).get("skidpad")
        if not res or L._excluded("skidpad", res) or not (res.get("left_s") and res.get("right_s")):
            continue
        best = min(res["left_s"], res["right_s"])
        errs.append(100.0 * (p["skidpad_s"] - best) / best)
    if not errs:
        return {}
    return {"n": len(errs), "mae_pct": sum(abs(e) for e in errs) / len(errs), "bias_pct": sum(errs) / len(errs),
            "within_5pct": sum(1 for e in errs if abs(e) <= 5.0)}
