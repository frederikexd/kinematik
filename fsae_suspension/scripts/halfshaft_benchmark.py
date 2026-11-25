# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The halfshaft fatigue screen (suspension/halfshaft.py) against Shigley: the DE-Goodman shaft diameter (Eq. 7-8)
in torsion, and the stresses of the published solution to problem 6-56. Limits: benchmarks/halfshaft/limits.json,
registered before the run.

    python scripts/halfshaft_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import halfshaft as hs  # noqa: E402

BENCH = ROOT / "benchmarks" / "halfshaft"
REG = json.loads((BENCH / "limits.json").read_text())
KPSI = 6.894757                                  # MPa per kpsi
LBF_IN = 0.1129848                               # N m per lbf in


def eq_7_8(c: dict) -> float:
    """Shigley Eq. 7-8 in pure torsion, solved by fixed-point iteration on d (mm)."""
    ab = {"ground": (1.58, -0.085), "machined": (4.51, -0.265), "hot_rolled": (57.7, -0.718)}[c["surface"]]
    sut = c["sut_mpa"]
    ka = ab[0] * sut ** ab[1]
    tm, ta = 0.5 * (c["t_max_nm"] + c["t_min_nm"]), 0.5 * (c["t_max_nm"] - c["t_min_nm"])
    d = 25.0
    for _ in range(200):
        kb = 1.24 * d ** -0.107 if d <= 51 else 1.51 * d ** -0.157
        se = ka * kb * 0.504 * min(sut, 1400.0)
        d = (16 * c["fos"] * math.sqrt(3) * c["kf"] / math.pi * (abs(ta) / se + abs(tm) / sut) * 1000.0) ** (1 / 3)
    return d


def run() -> dict:
    lim = REG["limits"]
    out, ok = {"eq_7_8": {}}, True
    for name, c in REG["cases_eq_7_8"].items():
        got = hs.goodman_min_diameter(c["t_max_nm"], c["t_min_nm"], sut_mpa=c["sut_mpa"], fos=c["fos"],
                                      surface=c["surface"], kf=c["kf"])["d_min_mm"]
        ref = eq_7_8(c)
        rel = got / ref - 1
        out["eq_7_8"][name] = {"kinematik_mm": got, "shigley_mm": ref, "rel": rel,
                               "passes": bool(abs(rel) <= lim["d_min_relative"])}
        ok = ok and out["eq_7_8"][name]["passes"]
    p = REG["sources"]["problem_6_56"]
    d_mm = p["data"]["d_in"] * 25.4
    tmax = hs.shaft_shear_mpa(p["data"]["T_max_lbf_in"] * LBF_IN, d_mm) * p["data"]["Kfs"] / KPSI
    tmin = hs.shaft_shear_mpa(p["data"]["T_min_lbf_in"] * LBF_IN, d_mm) * p["data"]["Kfs"] / KPSI
    rows = {}
    for q, v in (("tau_a_kpsi", (tmax - tmin) / 2), ("tau_m_kpsi", (tmax + tmin) / 2), ("tau_max_kpsi", tmax)):
        rel = v / p["target"][q] - 1
        rows[q] = {"kinematik": v, "target": p["target"][q], "rel": rel, "passes": bool(abs(rel) <= lim["stress_relative"])}
        ok = ok and rows[q]["passes"]
    out["problem_6_56"] = rows
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for n, r in res["eq_7_8"].items():
        print(f"Eq 7-8 {n:15s} KinematiK {r['kinematik_mm']:7.3f} mm  Shigley {r['shigley_mm']:7.3f} mm  "
              f"{100 * r['rel']:+6.2f} % {'PASS' if r['passes'] else 'FAIL'}")
    for q, r in res["problem_6_56"].items():
        print(f"6-56   {q:15s} KinematiK {r['kinematik']:7.3f}     target {r['target']:7.3f}     "
              f"{100 * r['rel']:+6.2f} % {'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
