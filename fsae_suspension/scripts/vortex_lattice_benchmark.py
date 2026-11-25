# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The vortex lattice (suspension/aero/vortex_lattice.py) against Tuck's (1993) accurate lifting-surface solutions.

Lift-curve slope of flat rectangular wings at aspect ratios 1, 2, 4 and 8, from C_L at +1 and -1 deg. Reference
values, case and limit: benchmarks/vortex_lattice/limits.json. Results: benchmarks/vortex_lattice/results.json.

    python scripts/vortex_lattice_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import os
import sys
import tempfile
from pathlib import Path

import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.aero.backends import Attitude, CaseSpec  # noqa: E402
from suspension.aero.vortex_lattice import VortexLatticeModel  # noqa: E402

BENCH = ROOT / "benchmarks" / "vortex_lattice"
REG = json.loads((BENCH / "limits.json").read_text())


def slope(ar: float, ns: int = 40, nc: int = 10, chord: float = 0.2) -> float:
    span = ar * chord
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "wing.stl")
        trimesh.creation.box(extents=[chord, span, 0.002]).export(path)
        cl = {}
        for a in (1.0, -1.0):
            r = VortexLatticeModel(ns, nc).solve(CaseSpec(attitude=Attitude(pitch_deg=a, ride_height_mm=50000.0),
                                                          geometry_path=path, reference_area_m2=span * chord))
            cl[a] = r.c_lift
    return abs(cl[1.0] - cl[-1.0]) / (2 * math.radians(1.0))


def run() -> dict:
    out, ok = {"cases": {}}, True
    for ar, ref in REG["reference"]["lift_slope_per_rad"].items():
        k = slope(float(ar))
        rel = k / ref - 1
        passes = bool(abs(rel) <= REG["limit_relative"])
        out["cases"][ar] = {"kinematik": k, "tuck": ref, "rel": rel, "passes": passes}
        ok = ok and passes
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for ar, r in res["cases"].items():
        print(f"AR {ar:>2s}: KinematiK {r['kinematik']:.4f} /rad  Tuck {r['tuck']:.5f}  {100 * r['rel']:+.2f} %  "
              f"{'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
