# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Brake hydraulics to torque (suspension/pedal_box.py, CircuitSpec) against a published disk-brake worked example
(Penn State ME 360, Shigley's method). Limits: benchmarks/brake_hydraulics/limits.json, registered before the run.

    python scripts/brake_hydraulics_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.pedal_box import CircuitSpec  # noqa: E402

BENCH = ROOT / "benchmarks" / "brake_hydraulics"
REG = json.loads((BENCH / "limits.json").read_text())


def run() -> dict:
    d, t, lim = REG["source"]["data"], REG["source"]["target"], REG["limits"]
    # the pad's clamp at its pressure limit, uniform wear: F = theta p_max r_i (r_o - r_i)
    F = math.radians(d["theta_deg"]) * d["p_max_mpa"] * 1e6 * d["ri_mm"] * 1e-3 * (d["ro_mm"] - d["ri_mm"]) * 1e-3
    c = CircuitSpec(mc_bore_mm=19.05, caliper_piston_dia_mm=40.0, pistons_per_side=1, pad_mu=d["mu"],
                    rotor_dia_mm=2 * d["ro_mm"], pad_inner_radius_mm=d["ri_mm"], n_corners=1)
    p_bar = F / (c.clamp_area_mm2 * 1e-6) / 1e5
    T = c.axle_torque_Nm(p_bar)
    ro, ri = d["ro_mm"] * 1e-3, d["ri_mm"] * 1e-3
    T_up = 2 / 3 * d["faces"] * d["mu"] * F * (ro ** 3 - ri ** 3) / (ro ** 2 - ri ** 2)
    rows = {"F_max_N": (F, t["F_max_N"], lim["F_relative"]), "T_uniform_wear_Nm": (T, t["T_uniform_wear_Nm"], lim["T_relative"])}
    out, ok = {}, True
    for q, (k, tgt, l) in rows.items():
        rel = k / tgt - 1
        out[q] = {"kinematik": k, "target": tgt, "rel": rel, "passes": bool(abs(rel) <= l)}
        ok = ok and out[q]["passes"]
    out["T_uniform_pressure_Nm"] = {"reference_formula": T_up, "target": t["T_uniform_pressure_Nm"], "graded": False,
                                    "note": "KinematiK uses uniform wear; uniform pressure would read this much higher"}
    out["line_pressure_bar"] = p_bar
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for q in ("F_max_N", "T_uniform_wear_Nm"):
        r = res[q]
        print(f"{q:20s} KinematiK {r['kinematik']:9.2f}  target {r['target']:9.2f}  {100 * r['rel']:+.3f} % "
              f"{'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
