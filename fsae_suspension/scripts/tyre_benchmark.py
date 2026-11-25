# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""KinematiK's Magic Formula lateral force (suspension/tiremodel.py, PacejkaLateral) against MFeval 4.3.1.

Reference output: benchmarks/tyre/mfeval_4.3.1/, made in MATLAB by benchmarks/tyre/mfeval_reference.m from the .tir
files that ship with MFeval. Limits and grading scope: benchmarks/tyre/limits.json, registered before the output
was seen. A point passes if |dFy| <= max(1 N, 0.1 % of the peak |Fy| at that load and camber).

    python scripts/tyre_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import csv
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.tiremodel import PacejkaLateral  # noqa: E402

BENCH = ROOT / "benchmarks" / "tyre"
REF = BENCH / "mfeval_4.3.1"
REG = json.loads((BENCH / "limits.json").read_text())
LAT = ["PCY1", "PDY1", "PDY2", "PDY3", "PEY1", "PEY2", "PEY3", "PEY4", "PKY1", "PKY2", "PKY3",
       "PHY1", "PHY2", "PHY3", "PVY1", "PVY2", "PVY3", "PVY4"]
SCALE = ["LFZO", "LCY", "LMUY", "LEY", "LKY", "LHY", "LVY", "LGAY"]


def read_tir(path: Path) -> dict:
    out = {}
    for line in path.read_text(errors="replace").splitlines():
        m = re.match(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*([-+0-9.eE]+)", line)
        if m:
            out[m.group(1).upper()] = float(m.group(2))
    return out


def scope(t: dict) -> str:
    fittyp = int(t.get("FITTYP", 0))
    if fittyp in (6, 21):
        return "all"
    if fittyp in (61, 62) and t.get("PKY4", 2.0) == 2.0:
        return "camber0"
    return "not graded"


def run() -> dict:
    lim = REG["limits"]
    out, ok = {}, True
    for csv_path in sorted(REF.glob("mfeval_ref_*.csv")):
        name = csv_path.stem.replace("mfeval_ref_", "")
        t = read_tir(csv_path.with_suffix(".tir"))
        tire = PacejkaLateral(coeffs={k: t.get(k, 0.0) for k in LAT}, FNOMIN=t["FNOMIN"],
                              scaling={k: t.get(k, 1.0) for k in SCALE})
        rows = list(csv.DictReader(csv_path.open()))
        groups = defaultdict(list)
        for r in rows:
            fz, a, g, fy = (float(r[k]) for k in ("Fz_N", "alpha_rad", "gamma_rad", "Fy_N"))
            groups[(fz, g)].append((a, fy, float(tire.fy(a, fz, g))))
        sc = scope(t)
        res = {"fittyp": int(t.get("FITTYP", 0)), "scope": sc, "groups": []}
        for (fz, g), pts in sorted(groups.items()):
            peak = max(abs(p[1]) for p in pts)
            tol = max(lim["fy_abs_N"], lim["fy_relative_to_peak"] * peak)
            worst = max(abs(p[2] - p[1]) for p in pts)
            graded = sc == "all" or (sc == "camber0" and g == 0.0)
            passes = (worst <= tol) if graded else None
            res["groups"].append({"Fz_N": fz, "gamma_deg": math.degrees(g), "peak_Fy_N": peak, "worst_dFy_N": worst,
                                  "tolerance_N": tol, "graded": graded, "passes": passes})
            if graded:
                ok = ok and passes
        out[name] = res
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for name, r in res.items():
        if name == "passed":
            continue
        g = [x for x in r["groups"] if x["graded"]]
        worst = max((x["worst_dFy_N"] / x["peak_Fy_N"] for x in r["groups"]), default=0)
        gw = max((x["worst_dFy_N"] / x["peak_Fy_N"] for x in g), default=float("nan"))
        print(f"{name:28s} FITTYP {r['fittyp']:2d} {r['scope']:10s} graded groups {len(g):2d}  "
              f"worst graded {100 * gw:.4f} % of peak  (worst any {100 * worst:.3f} %)  "
              f"{'PASS' if all(x['passes'] for x in g) else 'FAIL' if g else '-'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
