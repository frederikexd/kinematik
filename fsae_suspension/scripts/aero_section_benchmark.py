# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The 2D section solvers (suspension/aero/panel2d.py) against the exact Joukowski aerofoil, and the thick and thin
solvers against each other in ground effect as the thickness goes to zero. Limits: benchmarks/aero_section/limits.json.

    python scripts/aero_section_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.aero import panel2d as p2  # noqa: E402

BENCH = ROOT / "benchmarks" / "aero_section"
REG = json.loads((BENCH / "limits.json").read_text())


def run() -> dict:
    lim = REG["limits"]
    out, ok = {"joukowski": [], "thin_limit": []}, True
    x, z, exact = p2.joukowski_section(0.1, 0.08, 800)
    for a in (0.0, 4.0, 8.0):
        k, e = p2.thick_cl(x, z, math.radians(a)), exact(math.radians(a))
        r = {"alpha_deg": a, "kinematik": k, "exact": e, "rel": k / e - 1, "passes": bool(abs(k / e - 1) <= lim["joukowski_relative"])}
        out["joukowski"].append(r); ok = ok and r["passes"]
    th = np.linspace(0, 2 * math.pi, 401)
    ex, ez = 0.5 + 0.5 * np.cos(th), -0.0025 * np.sin(th)           # clockwise from the trailing edge
    xc = 0.5 * (1 - np.cos(np.linspace(0, math.pi, 201))); zc = np.zeros_like(xc)
    a = math.radians(4.0)
    for hc in (0.1, 0.3, 1.0, None):
        # both pitched about the leading edge, the plate's lowest point (its trailing edge, nose down) at h
        rot = lambda X, Z: (X * math.cos(a) + Z * math.sin(a), -X * math.sin(a) + Z * math.cos(a))
        X1, Z1 = rot(ex, ez); X2, Z2 = rot(xc, zc)
        if hc is not None:
            Z1 = Z1 + hc - Z2.min(); Z2 = Z2 + hc - Z2.min()
        k, t = p2.thick_cl(X1, Z1, 0.0, hc), p2.thin_cl(X2, Z2, 0.0, hc)
        r = {"h_c": hc, "thick": k, "thin": t, "rel": k / t - 1, "passes": bool(abs(k / t - 1) <= lim["thin_limit_relative"])}
        out["thin_limit"].append(r); ok = ok and r["passes"]
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for r in res["joukowski"]:
        print(f"Joukowski {r['alpha_deg']:3.0f} deg  panel {r['kinematik']:.4f}  exact {r['exact']:.4f}  "
              f"{100 * r['rel']:+.2f} % {'PASS' if r['passes'] else 'FAIL'}")
    for r in res["thin_limit"]:
        print(f"thin limit h/c {str(r['h_c']):5s} thick {r['thick']:.4f}  thin {r['thin']:.4f}  "
              f"{100 * r['rel']:+.2f} % {'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
