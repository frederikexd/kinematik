# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The torque-distribution allocation search (suspension/torque_distribution.py: state, steady_limit) against a
global search of the same model with SciPy. Limits: benchmarks/torque_distribution/limits.json, registered before
the run.

For each instant the left/right split d is fixed at each of 2,001 points across its allowed range (the module's own
_d_range, re-evaluated in each fixed-point pass) and the best is refined by Brent's method; KinematiK's answer must
be within 0.002 of that global optimum in peak tyre utilisation. Each steady limit must be feasible at KinematiK's
speed and infeasible 0.1 % above it, by the global search.

    python scripts/torque_distribution_benchmark.py      # exit 1 on any failure (about two minutes)
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import minimize_scalar

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import torque_distribution as td  # noqa: E402

BENCH = ROOT / "benchmarks" / "torque_distribution"
REG = json.loads((BENCH / "limits.json").read_text())
_ORIG = td._d_range


def _umax_at(frac: float, V, kappa, at, option) -> float:
    """Peak utilisation with the split at a fraction (0..1) of its allowed range, in every fixed-point pass."""
    def fixed(opt, driving, FR, car, fo, fi, k, tt):
        lo, hi = _ORIG(opt, driving, FR, car, fo, fi, k, tt)
        d = lo + frac * (hi - lo)
        return (d, d)
    td._d_range = fixed
    try:
        r = td.state(V, kappa, 0.0, at, option)
    finally:
        td._d_range = _ORIG
    return float(r["umax"])


def global_umax(V, kappa, at, option) -> float:
    grid = np.linspace(0.0, 1.0, 2001)
    u = np.array([_umax_at(f, V, kappa, at, option) for f in grid])
    i = int(np.argmin(u))
    lo, hi = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
    if hi > lo:
        res = minimize_scalar(lambda f: _umax_at(f, V, kappa, at, option), bounds=(lo, hi), method="bounded",
                              options={"xatol": 1e-9})
        return float(min(u[i], res.fun))
    return float(u[i])


def run() -> dict:
    lim = REG["limits"]
    out, ok = {"instants": [], "steady": []}, True
    for option in REG["cases"]["options"]:
        for V, R, at in ((8.0, 6.0, 4.0), (11.0, 9.0, -6.0), (14.0, 15.0, 2.0), (16.0, 15.0, -8.0),
                         (18.0, 25.0, 6.0), (20.0, 25.0, 0.0)):
            k = td.state(V, 1.0 / R, 0.0, at, option)
            g = global_umax(V, 1.0 / R, at, option)
            if g > 1.0:          # phase 2: an infeasible instant's overshoot is graded on the exact search
                k = td.state(V, 1.0 / R, 0.0, at, option, exact_overshoot=True)
            diff = float(k["umax"]) - g
            p = bool(diff <= lim["umax_abs"])          # KinematiK may not be worse than the global optimum
            out["instants"].append({"option": option, "V": V, "R": R, "at": at, "kinematik_umax": float(k["umax"]),
                                    "global_umax": g, "diff": diff, "passes": p})
            ok = ok and p
        for R in REG["cases"]["steady_limit_radius_m"]:
            s = td.steady_limit(R, option)
            V = s["V"]
            at_v = global_umax(V, 1.0 / R, 0.0, option)
            above = global_umax(V * (1 + lim["steady_speed_relative"]), 1.0 / R, 0.0, option)
            p = bool(at_v <= 1.0 + 1e-9 and above > 1.0)
            out["steady"].append({"option": option, "R": R, "V": V, "global_umax_at_V": at_v,
                                  "global_umax_0.1pct_above": above, "passes": p})
            ok = ok and p
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    w = max(res["instants"], key=lambda r: r["diff"])
    print(f"instants: {sum(r['passes'] for r in res['instants'])}/{len(res['instants'])} pass; worst KinematiK - "
          f"global {w['diff']:+.4f} ({w['option']}, V {w['V']}, R {w['R']}, at {w['at']})")
    print(f"steady limits: {sum(r['passes'] for r in res['steady'])}/{len(res['steady'])} pass")
    for r in res["instants"] + res["steady"]:
        if not r["passes"]:
            print("  FAIL", r)
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
