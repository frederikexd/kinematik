# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The quasi-1D underfloor model (suspension/aero/underfloor.py) against Southampton's diffuser bluff body in a
moving-ground wind tunnel (Ruhrmann & Zhang 2003). Limits: benchmarks/underbody_diffuser/limits.json.

    python scripts/underbody_diffuser_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.aero import underfloor as uf  # noqa: E402

BENCH = ROOT / "benchmarks" / "underbody_diffuser"
REG = json.loads((BENCH / "limits.json").read_text())
S = REG["source"]
D = S["body_d_mm"] / 1000.0
A_REF = (S["height_over_d"] * S["width_over_d"] - 2 * (1 - math.pi / 4) * 0.35 ** 2) * D * D


def duct(theta_deg: float, h_over_d: float, nx: int = 400):
    L, x0 = S["length_over_d"] * D, S["ramp_start_over_d"] * D
    xs = np.linspace(0.0, L, nx)
    nose_l, nose_h = 2.5 * D, 0.5 * S["height_over_d"] * D
    relief = np.where(xs < nose_l, nose_h * (1 - np.sqrt(np.clip(1 - ((nose_l - xs) / nose_l) ** 2, 0, 1))), 0.0)
    relief = relief + np.where(xs > x0, (xs - x0) * math.tan(math.radians(theta_deg)), 0.0)
    gap = h_over_d * D + relief
    width = np.full_like(xs, S["width_over_d"] * D)
    return xs, gap * width, width


def cl(theta_deg: float, h_over_d: float) -> float:
    xs, area, width = duct(theta_deg, h_over_d)
    r = uf._solve_channel(xs, area, width, 30.0, A_REF, 1.225, uf.DEFAULT_DISCHARGE)
    return -r.c_lift                       # downforce positive, as the paper


def run() -> dict:
    lim = REG["limits"]["relative_each_graded_point"]
    rows, ok = [], True
    for th, pts in S["CL_measured_fig2a"].items():
        for h, m in pts.items():
            k = cl(float(th), float(h))
            graded = th in ("5", "10")
            rel = k / m - 1
            if graded:
                ok = ok and abs(rel) <= lim
            rows.append({"theta_deg": float(th), "h_over_d": float(h), "measured": m, "kinematik": k, "rel": rel,
                         "graded": graded})
    return {"A_ref_m2": A_REF, "rows": rows, "passed": bool(ok)}


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for r in res["rows"]:
        print(f"  {r['theta_deg']:4.0f} deg h/d {r['h_over_d']:.2f}: measured {r['measured']:.2f}, KinematiK "
              f"{r['kinematik']:.2f} ({100 * r['rel']:+.0f} %){'' if r['graded'] else '  (not graded)'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
