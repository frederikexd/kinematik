# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Prediction ledger, autocross relative pace (test B in benchmarks/ledger/protocol-v2/REGISTRATION.md).

The autocross track changes every year and is not published as a map, so an absolute lap time can't be predicted
from a spec. What can be tested is whether the spec ranks and spaces the cars correctly, which is the comparison a
team makes in prevalidation ("does design A lap faster than design B, and by how much?").

  physics   each car's lap on laptime.default_autocross() with the v1 build of its sanitized spec (ledger_v2).
  scale     each prediction times the median of official / predicted over the OTHER scored cars at the same
            event (leave one out): one shared number per event that carries the track length, never the car's
            own time.
  baseline  every car predicted at the median official time of the other cars. The spec only adds information
            if the model beats it.

Kept separate from ledger_v2.py so the frozen v2 code (and its registered SHA-256) is unchanged.
"""
from __future__ import annotations

import math
import statistics

from . import ledger as L
from . import ledger_v2 as V
from .laptime import default_autocross, simulate_lap


def autocross_lap_s(spec) -> float:
    """Unscaled lap time (s) of one car on KinematiK's representative autocross, from its sanitized spec."""
    s, _rejected = V.sanitize(spec)
    veh, pt, _inputs = L.build(s)
    return float(simulate_lap(veh, default_autocross(), pt).lap_time_s)


def _summary(errs) -> dict:
    a = [abs(e) for e in errs]
    return {"n": len(errs), "mae_pct": sum(a) / len(a), "median_abs_pct": statistics.median(a),
            "bias_pct": sum(errs) / len(errs), "within_5pct": sum(1 for e in a if e <= 5.0)}


def score_pace(lap_s: dict, results: dict) -> dict:
    """Leave-one-out pace score (errors in %, times in s). `lap_s` is {car: autocross_lap_s}, `results` official
    results keyed by car with an "autocross" entry. Returns {"rows", "model", "baseline"}; the model summary also
    carries the Spearman rank correlation (unitless). Empty summaries if fewer than 3 cars can be scored."""
    rows, clean = [], {}
    for car, t in lap_s.items():
        res = results.get(car, {}).get("autocross")
        why = "no official result" if not res else L._excluded("autocross", res)
        if why:
            rows.append({"car": car, "excluded": why})
        else:
            clean[car] = (t, res["best_s"])
    if len(clean) < 3:
        return {"rows": rows, "model": {}, "baseline": {}}
    err_m, err_b, pred, off = [], [], [], []
    for car, (t, o) in clean.items():
        others = [(tt, oo) for c, (tt, oo) in clean.items() if c != car]
        scale = statistics.median(oo / tt for tt, oo in others)
        base = statistics.median(oo for _tt, oo in others)
        p = t * scale
        e, eb = 100.0 * (p - o) / o, 100.0 * (base - o) / o
        rows.append({"car": car, "physics_s": round(t, 3), "predicted_s": round(p, 3), "official_s": o,
                     "error_pct": round(e, 2), "baseline_error_pct": round(eb, 2), "excluded": ""})
        err_m.append(e)
        err_b.append(eb)
        pred.append(t)
        off.append(o)
    model = _summary(err_m)
    rho = L.spearman(pred, off)
    model["spearman"] = rho if not math.isnan(rho) else None
    return {"rows": rows, "model": model, "baseline": _summary(err_b)}
