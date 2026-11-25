# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The bracket weld check (suspension/bracket_fos.py) against Shigley: direct throat shear (problem 9-1) and fillet
welds in bending (Table 9-2, one line and two parallel lines). Limits: benchmarks/weld/limits.json, registered
before the run.

    python scripts/weld_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import inspect
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import bracket_fos as bf  # noqa: E402

BENCH = ROOT / "benchmarks" / "weld"
REG = json.loads((BENCH / "limits.json").read_text())


def _weld_stress(P, lever, h, total_len, lines):
    kw = dict(name="w", material="Steel 4130 (as-welded, no PWHT)", width_mm=200.0, thickness_mm=50.0, P_N=P, lever_arm_mm=lever,
              weld_leg_mm=h, weld_length_mm=total_len)
    if "weld_lines" in inspect.signature(bf.Bracket).parameters:
        kw["weld_lines"] = lines
    r = bf.screen_bracket(bf.Bracket(**kw))
    return next(m.stress_MPa for m in r.modes if m.mode == "weld throat shear")


def shigley(c):
    d, h, n = c["line_mm"], c["h_mm"], c["lines"]
    iu = d ** 3 / 12 if n == 1 else d ** 3 / 6
    t1 = c["P_N"] / (0.707 * h * n * d)
    t2 = c["P_N"] * c["lever_mm"] * (d / 2) / (0.707 * h * iu)
    return math.hypot(t1, t2)


def run() -> dict:
    lim = REG["limits"]["relative"]
    out, ok = {}, True
    p = REG["sources"]["problem_9_1"]
    tau = _weld_stress(p["target"]["F_kN"] * 1000.0, 0.0, p["data"]["h_mm"], p["data"]["l_mm"], 1)
    rel = tau / p["data"]["tau_allow_mpa"] - 1
    out["problem_9_1"] = {"kinematik_mpa": tau, "target_mpa": p["data"]["tau_allow_mpa"], "rel": rel,
                          "passes": bool(abs(rel) <= 0.002)}     # 49.5 kN is printed to 3 figures
    ok = ok and out["problem_9_1"]["passes"]
    for name, c in REG["cases"].items():
        k = _weld_stress(c["P_N"], c["lever_mm"], c["h_mm"], c["line_mm"] * c["lines"], c["lines"])
        s = shigley(c)
        rel = k / s - 1
        out[name] = {"kinematik_mpa": k, "shigley_mpa": s, "rel": rel, "passes": bool(abs(rel) <= lim)}
        ok = ok and out[name]["passes"]
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for n, r in res.items():
        if n != "passed":
            ref = r.get("shigley_mpa", r.get("target_mpa"))
            print(f"{n:12s} KinematiK {r['kinematik_mpa']:8.2f} MPa  Shigley {ref:8.2f} MPa  {100 * r['rel']:+7.2f} % "
                  f"{'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
