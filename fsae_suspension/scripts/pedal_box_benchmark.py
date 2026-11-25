# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The balance bar (suspension/pedal_box.py, balance_bar_bias) against an OpenSeesPy static solve of the bar.
Limits: benchmarks/pedal_box/limits.json, registered before the run. Needs openseespy.

    python scripts/pedal_box_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.pedal_box import CircuitSpec, balance_bar_bias  # noqa: E402

BENCH = ROOT / "benchmarks" / "pedal_box"
REG = json.loads((BENCH / "limits.json").read_text())


def opensees(c: dict) -> tuple[float, float]:
    import openseespy.opensees as ops
    L, e = c["bar_length_mm"], c["bar_offset_mm"]
    F = c["pedal_force_N"] * c["pedal_ratio"]
    ops.wipe(); ops.model("basic", "-ndm", 2, "-ndf", 3)
    ops.node(1, 0.0, 0.0)                    # front clevis
    ops.node(2, L / 2.0 - e, 0.0)            # pivot: offset is positive toward the front clevis
    ops.node(3, L, 0.0)                      # rear clevis
    ops.fix(1, 0, 1, 0); ops.fix(3, 1, 1, 0)
    ops.geomTransf("Linear", 1)
    for k, (i, j) in enumerate(((1, 2), (2, 3)), 1):
        ops.element("elasticBeamColumn", k, i, j, 100.0, 200000.0, 1000.0, 1)
    ops.timeSeries("Constant", 1); ops.pattern("Plain", 1, 1); ops.load(2, 0.0, -F, 0.0)
    ops.system("FullGeneral"); ops.numberer("Plain"); ops.constraints("Plain")
    ops.integrator("LoadControl", 1.0); ops.algorithm("Linear"); ops.analysis("Static")
    assert ops.analyze(1) == 0
    ops.reactions()
    out = ops.nodeReaction(1, 2), ops.nodeReaction(3, 2)
    ops.wipe()
    return out


def run() -> dict:
    lim = REG["limits"]
    out, ok = {}, True
    for name, c in REG["cases"].items():
        front = CircuitSpec(mc_bore_mm=c["mc_front_mm"], caliper_piston_dia_mm=25.4)
        rear = CircuitSpec(mc_bore_mm=c["mc_rear_mm"], caliper_piston_dia_mm=25.4)
        r = balance_bar_bias(pedal_force_N=c["pedal_force_N"], pedal_ratio=c["pedal_ratio"], front=front, rear=rear,
                             bar_length_mm=c["bar_length_mm"], bar_offset_mm=c["bar_offset_mm"])
        ff, fr = opensees(c)
        pf = ff / (math.pi / 4 * c["mc_front_mm"] ** 2) * 10.0      # N/mm^2 -> bar
        pr = fr / (math.pi / 4 * c["mc_rear_mm"] ** 2) * 10.0
        rows = {}
        for q, k, ref, l in (("F_front_N", r.force_front_N, ff, lim["force_relative"]),
                             ("F_rear_N", r.force_rear_N, fr, lim["force_relative"]),
                             ("p_front_bar", r.pressure_front_bar, pf, lim["pressure_relative"]),
                             ("p_rear_bar", r.pressure_rear_bar, pr, lim["pressure_relative"])):
            rel = k / ref - 1
            rows[q] = {"kinematik": k, "opensees": ref, "rel": rel, "passes": bool(abs(rel) <= l)}
            ok = ok and rows[q]["passes"]
        out[name] = rows
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for n, rows in res.items():
        if n != "passed":
            for q, r in rows.items():
                print(f"{n:13s} {q:12s} KinematiK {r['kinematik']:10.4f}  OpenSees {r['opensees']:10.4f}  "
                      f"{r['rel']:+.1e} {'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
