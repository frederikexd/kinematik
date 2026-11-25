# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Radiator effectiveness (suspension/cooling.py, crossflow_effectiveness) against an exact finite-difference
solution of the crossflow exchanger and a published automobile-radiator case. Limits:
benchmarks/radiator/limits.json, registered before the model was written.

    python scripts/radiator_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.cooling import crossflow_effectiveness  # noqa: E402

BENCH = ROOT / "benchmarks" / "radiator"
REG = json.loads((BENCH / "limits.json").read_text())


def _fd(ntu: float, cr: float, arrangement: str, n: int) -> float:
    """Fluid A (C_min = 1, enters at 1) flows along x, fluid B (C_max = 1/Cr, enters at 0) along y."""
    ca, cb = 1.0, (1.0 / cr if cr > 0 else 1e12)
    if arrangement == "both_unmixed":
        ta = np.ones(n)                         # A streams (one per row j), marching over columns i
        q = 0.0
        tb_cols = np.zeros(n)                   # B temperature entering each column... B marches over rows
        tb = np.zeros((n,))                     # B streams (one per column i), current row
        ta_grid = np.ones(n)
        tb = np.zeros(n)
        # march rows j (B direction) outer, columns i (A direction) inner, cell by cell with exact pair exchange
        ta_row_in = np.ones(n)                  # A inlet temperature of each row
        a = np.empty((n, n))
        for j in range(n):
            t_a = 1.0
            for i in range(n):
                ua = ntu / (n * n)
                c1, c2 = ca / n, cb / n          # each cell sees 1/n of each stream
                # exact counter-free exchange of two well-mixed parcels over the cell (two-node, no direction)
                k = ua * (1.0 / c1 + 1.0 / c2)
                d = (t_a - tb[i]) * (1.0 - math.exp(-k)) / (1.0 / c1 + 1.0 / c2)   # heat over the cell
                t_a -= d / c1
                tb[i] += d / c2
                q += d
        return q / (ca * 1.0)
    if arrangement == "cmax_mixed":             # B (C_max) mixed: uniform across each row
        t_b, q = 0.0, 0.0
        for _ in range(n):
            t_out = t_b + (1.0 - t_b) * math.exp(-ntu / ca)     # each A stream crosses one row at uniform t_b
            dq = (ca / n) * (1.0 - t_out)
            q += dq
            t_b += dq / cb
        return q / ca
    if arrangement == "cmin_mixed":             # A (C_min) mixed: uniform across each column
        t_a, q = 1.0, 0.0
        for _ in range(n):
            t_out = t_a - t_a * math.exp(-ntu / cb) if cb < 1e11 else t_a     # B streams warm towards t_a
            dq = (cb / n) * (t_out - 0.0) if cb < 1e11 else (ca / n) * 0.0
            if cb >= 1e11:                        # Cr = 0: B never warms; A loses NTU/n of its excess per column
                dq = t_a * (1.0 - math.exp(-ntu / n)) * ca
            q += dq
            t_a -= dq / ca
        return q / ca
    raise ValueError(arrangement)


def exact(ntu: float, cr: float, arrangement: str) -> float:
    """Richardson-extrapolated finite-difference effectiveness."""
    n1 = 150 if arrangement == "both_unmixed" else 2000
    e1, e2 = _fd(ntu, cr, arrangement, n1), _fd(ntu, cr, arrangement, 2 * n1)
    return 2 * e2 - e1


def run() -> dict:
    lim = REG["limits"]
    out, ok = {"grid": []}, True
    for arr in ("both_unmixed", "cmax_mixed", "cmin_mixed"):
        tol = lim["both_unmixed_vs_exact_abs"] if arr == "both_unmixed" else lim["one_mixed_vs_exact_abs"]
        for ntu in REG["grid"]["NTU"]:
            for cr in REG["grid"]["Cr"]:
                k, x = crossflow_effectiveness(ntu, cr, arr), exact(ntu, cr, arr)
                row = {"arrangement": arr, "NTU": ntu, "Cr": cr, "kinematik": k, "exact": x, "diff": k - x,
                       "passes": bool(abs(k - x) <= tol)}
                out["grid"].append(row)
                ok = ok and row["passes"]
    k = crossflow_effectiveness(0.59, 0.455, "both_unmixed")
    out["published"] = {"kinematik": k, "target": 0.40, "diff": k - 0.40,
                        "passes": bool(abs(k - 0.40) <= lim["published_abs"])}
    ok = ok and out["published"]["passes"]
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for arr in ("both_unmixed", "cmax_mixed", "cmin_mixed"):
        rows = [r for r in res["grid"] if r["arrangement"] == arr]
        w = max(rows, key=lambda r: abs(r["diff"]))
        print(f"{arr:13s} worst {w['diff']:+.4f} at NTU {w['NTU']}, Cr {w['Cr']} "
              f"({'PASS' if all(r['passes'] for r in rows) else 'FAIL'})")
    p = res["published"]
    print(f"published     {p['kinematik']:.4f} vs {p['target']}  {'PASS' if p['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
