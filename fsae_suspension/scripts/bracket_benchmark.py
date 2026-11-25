# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The bracket screen's bolted modes (suspension/bracket_fos.py): member bearing and net-section tension against
Shigley's published solutions (8-68, 8-70, 8-71), and hole tear-out against AISC 360 J3.10. Stresses are compared
(the factors of safety use KinematiK's own material yields). Limits: benchmarks/bracket/limits.json, registered
before the run.

    python scripts/bracket_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import inspect
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import bracket_fos as bf  # noqa: E402

BENCH = ROOT / "benchmarks" / "bracket"
REG = json.loads((BENCH / "limits.json").read_text())
MAT = "Steel 4130 (normalized tube)"


def _modes(**kw) -> dict:
    if "holes_in_section" in kw and "holes_in_section" not in inspect.signature(bf.Bracket).parameters:
        kw.pop("holes_in_section")
    r = bf.screen_bracket(bf.Bracket(name="b", material=MAT, load_is_shear=False, **kw))
    return {m.mode: m.stress_MPa for m in r.modes}


def run() -> dict:
    lim = REG["limits"]["relative"]
    out, ok = {}, True
    for name in ("8-68", "8-70", "8-71"):
        c = REG["cases"][name]
        m = _modes(width_mm=c["w_mm"], thickness_mm=c["t_mm"], P_N=c["F_N"], hole_dia_mm=c["d_mm"],
                   n_bolts=c["n_bolts"], holes_in_section=c["holes_in_section"], edge_dist_mm=10 * c["d_mm"])
        rows = {}
        for q, mode in (("member_bearing_mpa", "bolt-hole bearing"), ("net_tension_mpa", "direct tension (net section at hole)")):
            k, t = m[mode], c["target"][q]
            rel = k / t - 1
            rows[q] = {"kinematik": k, "target": t, "rel": rel, "passes": bool(abs(rel) <= lim)}
            ok = ok and rows[q]["passes"]
        out[name] = rows
    c = REG["cases"]["aisc_tearout"]
    lc = c["edge_mm"] - c["hole_mm"] / 2
    Rn = 1.2 * lc * c["t_mm"] * c["Fu_mpa"]
    m = _modes(width_mm=200.0, thickness_mm=c["t_mm"], P_N=Rn, hole_dia_mm=c["hole_mm"], edge_dist_mm=c["edge_mm"],
               n_bolts=c["n_bolts"])
    k, t = m["hole tear-out (shear)"], 0.6 * c["Fu_mpa"]
    out["aisc_tearout"] = {"tearout_shear_mpa": {"kinematik": k, "target": t, "rel": k / t - 1,
                                                 "passes": bool(abs(k / t - 1) <= lim)}}
    ok = ok and out["aisc_tearout"]["tearout_shear_mpa"]["passes"]
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for n, rows in res.items():
        if n != "passed":
            for q, r in rows.items():
                print(f"{n:13s} {q:20s} KinematiK {r['kinematik']:8.2f}  target {r['target']:8.2f}  "
                      f"{100 * r['rel']:+7.2f} % {'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
