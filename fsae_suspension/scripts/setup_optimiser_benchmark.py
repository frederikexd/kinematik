# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The setup optimiser's coordinate search (suspension/setup.py, optimise) against a global search of the same
objective by SciPy's differential evolution. Limits: benchmarks/setup_optimiser/limits.json, registered before the run.

    python scripts/setup_optimiser_benchmark.py      # exit 1 on any failure (about two minutes)
"""
from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.dynamics import VehicleParams  # noqa: E402
from suspension.kinematics import Hardpoints, SuspensionKinematics  # noqa: E402
from suspension.setup import PARAM_KNOBS, _prepare, evaluate, optimise  # noqa: E402
from suspension.tiremodel import default_tire  # noqa: E402

BENCH = ROOT / "benchmarks" / "setup_optimiser"
REG = json.loads((BENCH / "limits.json").read_text())
TARGET, TOL = 0.04, 0.06


def run() -> dict:
    kin, tire = SuspensionKinematics(Hardpoints.default()), default_tire()
    knobs = list(PARAM_KNOBS)
    base = _prepare(VehicleParams(), kin, kin)

    def score(x):
        p = replace(base, **{k: float(v) for k, v in zip(knobs, x)})
        e = evaluate(p, kin, kin, tire)
        pen = 0.5 * max(abs(e["balance"] - TARGET) - TOL, 0.0)       # the optimiser's own penalty
        return e["max_g"] - pen, e

    o = optimise(VehicleParams(), front_kin=kin, rear_kin=kin, tire=tire, target_balance=TARGET, balance_tol=TOL)
    x_coord = [o["best_params"].get(k, getattr(base, k)) for k in knobs]
    s_coord, e_coord = score(x_coord)
    de = differential_evolution(lambda x: -score(x)[0], [(PARAM_KNOBS[k]["lo"], PARAM_KNOBS[k]["hi"]) for k in knobs],
                                seed=1, popsize=15, maxiter=200, tol=1e-8, polish=True)
    s_de, e_de = score(de.x)
    gap = s_de - s_coord
    return {"coordinate_search": {"score_g": s_coord, "eval": e_coord, "setup": dict(zip(knobs, map(float, x_coord)))},
            "differential_evolution": {"score_g": s_de, "eval": e_de, "setup": dict(zip(knobs, map(float, de.x))),
                                       "evaluations": int(de.nfev), "converged": bool(de.success)},
            "gap_g": gap, "passed": bool(gap <= REG["limits"]["score_gap_g"])}


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    c, d = res["coordinate_search"], res["differential_evolution"]
    print(f"coordinate search {c['score_g']:.4f} g   global (DE, {d['evaluations']} evals) {d['score_g']:.4f} g   "
          f"gap {res['gap_g']:+.4f} g")
    for k in c["setup"]:
        print(f"  {k:22s} coord {c['setup'][k]:8.3f}   DE {d['setup'][k]:8.3f}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
