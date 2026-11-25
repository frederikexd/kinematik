# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The bolted-joint check (suspension/bolted_joint.py) against Shigley's published worked solutions.

  8-22  M14 x 2 bolt through 40 mm of steel: bolt stiffness, member stiffness, joint constant C
  8-25  the same joint, member stiffness by Wileman's fit
  8-46, 8-47  preload from assembly torque, T = K F d

Problem data, targets and limits: benchmarks/bolted_joint/limits.json, registered before the run.
Results: benchmarks/bolted_joint/results.json.

    python scripts/bolted_joint_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import inspect
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import bolted_joint as bj  # noqa: E402

BENCH = ROOT / "benchmarks" / "bolted_joint"
REG = json.loads((BENCH / "limits.json").read_text())
LIM = REG["limits"]


def _fastener(d, At, ld):
    kw = dict(grade="8.8", nominal_d_mm=d, stress_area_mm2=At)
    if "shank_in_grip_mm" in inspect.signature(bj.Fastener).parameters:
        kw["shank_in_grip_mm"] = ld          # the unthreaded shank inside the grip
    return bj.Fastener(**kw)


def run() -> dict:
    out, ok = {}, True

    def row(name, q, got, tgt, lim):
        nonlocal ok
        rel = got / tgt - 1
        passes = bool(abs(rel) <= lim)
        ok = ok and passes
        out.setdefault(name, {})[q] = {"kinematik": got, "target": tgt, "rel": rel, "limit": lim, "passes": passes}

    p = REG["problems"]["8-22_M14"]
    d = p["data"]
    E = d["E_GPa"] * 1000.0
    f = _fastener(d["d_mm"], d["At_mm2"], d["ld_mm"])
    kb = bj._bolt_stiffness(f, d["grip_mm"], E_bolt_MPa=E) / 1000.0          # N/mm -> MN/m
    km = bj._member_stiffness(E, d["d_mm"], d["grip_mm"]) / 1000.0
    row("8-22_M14", "k_b_MN_per_m", kb, p["target"]["k_b_MN_per_m"], LIM["k_b_relative"])
    row("8-22_M14", "C", kb / (kb + km), p["target"]["C"], LIM["C_relative"])

    p = REG["problems"]["8-25_wileman"]
    d = p["data"]
    km = bj._member_stiffness(d["E_GPa"] * 1000.0, d["d_mm"], d["grip_mm"]) / 1000.0
    row("8-25_wileman", "k_m_wileman_MN_per_m", km, p["target"]["k_m_wileman_MN_per_m"], LIM["k_m_wileman_relative"])

    for name in ("8-46a_torque", "8-47a_torque"):
        p = REG["problems"][name]
        d = p["data"]
        # preload through the full check: a joint with no external load, the stiffnesses irrelevant here
        fz = bj.Fastener(grade="10.9", nominal_d_mm=d["d_mm"], stress_area_mm2=300.0, K_factor=d["K"])
        r = bj.analyze_joint(fz, bj.ClampedStack(k_bolt_N_per_mm=1e5, k_member_N_per_mm=4e5), d["T_Nm"], 0.0)
        row(name, "F_i_kN", r.F_preload / 1000.0, p["target"]["F_i_kN"], LIM["F_i_relative"])

    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for name, rows in res.items():
        if name == "passed":
            continue
        for q, r in rows.items():
            print(f"{name:14s} {q:22s} KinematiK {r['kinematik']:10.4g}  target {r['target']:10.4g}  "
                  f"{100 * r['rel']:+6.2f} % (limit {100 * r['limit']:.1f} %) {'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
