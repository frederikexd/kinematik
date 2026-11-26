# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Absolute downforce of the vortex lattice (suspension/aero/vortex_lattice.py) against wind-tunnel measurements of
a single-element wing in ground effect (Zerihan & Zhang; Zerihan's thesis, Southampton 2001). Limits:
benchmarks/aero_ground_effect/limits.json, registered before the run.

    python scripts/aero_ground_effect_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.aero.backends import Attitude, CaseSpec  # noqa: E402
from suspension.aero.vortex_lattice import VortexLatticeModel  # noqa: E402

BENCH = ROOT / "benchmarks" / "aero_ground_effect"
REG = json.loads((BENCH / "limits.json").read_text())
C, SPAN = 0.2234, 1.100
# Thesis Table 2: x/c, suction y/c, pressure y/c, at the reference incidence (y up, ground below)
TABLE2 = [(0.0000, 0.0000, 0.0000), (0.0010, -0.0076, 0.0079), (0.0020, -0.0107, 0.0109), (0.0050, -0.0168, 0.0173),
          (0.0100, -0.0228, 0.0232), (0.0150, -0.0266, 0.0271), (0.0200, -0.0294, 0.0300), (0.0250, -0.0320, 0.0313),
          (0.0300, -0.0345, 0.0322), (0.0350, -0.0369, 0.0330), (0.0400, -0.0393, 0.0338), (0.0450, -0.0416, 0.0346),
          (0.0500, -0.0438, 0.0354), (0.0550, -0.0460, 0.0361), (0.0600, -0.0481, 0.0369), (0.0700, -0.0520, 0.0382),
          (0.0800, -0.0557, 0.0395), (0.0900, -0.0591, 0.0407), (0.1000, -0.0622, 0.0417), (0.1200, -0.0676, 0.0436),
          (0.1400, -0.0718, 0.0451), (0.1600, -0.0750, 0.0463), (0.1800, -0.0769, 0.0472), (0.2000, -0.0778, 0.0480),
          (0.2500, -0.0762, 0.0498), (0.3000, -0.0732, 0.0515), (0.3500, -0.0692, 0.0527), (0.4000, -0.0645, 0.0534),
          (0.4500, -0.0590, 0.0537), (0.5000, -0.0526, 0.0535), (0.5500, -0.0454, 0.0529), (0.6000, -0.0373, 0.0518),
          (0.6500, -0.0285, 0.0503), (0.7000, -0.0188, 0.0482), (0.7500, -0.0083, 0.0456), (0.8000, 0.0031, 0.0438),
          (0.8500, 0.0152, 0.0443), (0.9000, 0.0282, 0.0479), (0.9200, 0.0336, 0.0502), (0.9400, 0.0392, 0.0530),
          (0.9600, 0.0449, 0.0562), (0.9800, 0.0507, 0.0599), (0.9900, 0.0537, 0.0619), (1.0000, 0.0567, 0.0640)]
HEIGHTS = [None, 0.671, 0.448, 0.313, 0.224, 0.134, 0.082]


def wing_stl(path: str) -> float:
    """Closed extruded wing; returns the depth of the lowest suction point below the leading edge (m)."""
    lower = [(x * C, s * C) for x, s, _ in TABLE2]
    upper = [(x * C, p * C) for x, _, p in TABLE2][::-1]
    loop = lower + upper[:-1]                      # suction LE->TE, pressure TE->LE (LE once)
    n = len(loop)
    v = [(x, -SPAN / 2, z) for x, z in loop] + [(x, SPAN / 2, z) for x, z in loop]
    cx, cz = np.mean([p[0] for p in loop]), np.mean([p[1] for p in loop])
    v += [(cx, -SPAN / 2, cz), (cx, SPAN / 2, cz)]
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [(i, j, n + j), (i, n + j, n + i)]
        f += [(2 * n, j, i), (2 * n + 1, n + i, n + j)]
    m = trimesh.Trimesh(vertices=np.array(v), faces=np.array(f), process=True)
    trimesh.repair.fix_normals(m)
    m.export(path)
    return -min(z for _, z in loop)


def section_loop(n: int = 120):
    """Table 2 as one closed loop in chord units, cosine-spaced: TE -> suction (lower) -> LE -> pressure -> TE."""
    b = 0.5 * (1 - np.cos(np.linspace(0, np.pi, n + 1)))
    xs = [r[0] for r in TABLE2]
    s = np.interp(b, xs, [r[1] for r in TABLE2])
    p = np.interp(b, xs, [r[2] for r in TABLE2])
    return list(zip(np.r_[b[::-1], b[1:]].tolist(), np.r_[s[::-1], p[1:]].tolist()))


def _sweep(path, drop, endplates=None, heights=HEIGHTS, section=None):
    pts = []
    for hc in heights:
        # the lattice is placed by its leading edge; the thesis measures to the lowest suction point
        h_mm = 50000.0 if hc is None else (hc * C + drop) * 1000.0
        extra = {"endplates": endplates} if endplates else {}
        if section:
            extra["section_profile"] = section
        r = VortexLatticeModel(60, 16).solve(CaseSpec(attitude=Attitude(ride_height_mm=h_mm), geometry_path=path,
                                                      reference_area_m2=SPAN * C, extra=extra))
        pts.append({"h_c": hc, "downforce_CL": abs(float(r.c_lift)), "sign": float(np.sign(r.c_lift))})
    return pts


def run(phase: int = 3) -> dict:
    """Phase 1: wing alone (the registered run). Phase 2: with endplates as assumed in limits.json. Phase 3: and
    the section thickness as a 2D strip factor."""
    lim = REG["limits"]["relative"]
    out = {"phase": phase}
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "wing.stl")
        drop = wing_stl(path)
        ep = None
        if phase >= 2:
            ep = dict(length_m=0.25, height_m=0.10, x_start_m=-(0.25 - C) / 2, z_bottom_m=-drop, n_height=12,
                      n_length=16)
            alt = dict(length_m=0.10, height_m=0.25, x_start_m=(C - 0.10) / 2, z_bottom_m=-drop, n_height=24,
                       n_length=8)
            sec = section_loop() if phase >= 3 else None
            out["bracket_other_orientation"] = _sweep(path, drop, alt, heights=[None, 0.671], section=sec)
        out["points"] = _sweep(path, drop, ep, section=sec if phase >= 3 else None)
    g = REG["graded"]
    pts = {p["h_c"]: p["downforce_CL"] for p in out["points"]}
    rows, ok = {}, True
    for q, k in (("freestream_CL", pts[None]), ("h_c_0.671_CL", pts[0.671])):
        rel = k / g[q] - 1
        rows[q] = {"kinematik": k, "measured": g[q], "rel": rel, "passes": bool(abs(rel) <= lim)}
        ok = ok and rows[q]["passes"]
    out["graded"] = rows
    out["reported"] = {"h_c_0.082_CL": pts[0.082], "measured_max_CL_at_0.082": 1.72}
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for p in res["points"]:
        print(f"h/c {str(p['h_c']):6s} downforce CL {p['downforce_CL']:.3f}  (sign {p['sign']:+.0f})")
    for q, r in res["graded"].items():
        print(f"{q:14s} KinematiK {r['kinematik']:.3f}  measured {r['measured']:.2f}  {100 * r['rel']:+.1f} % "
              f"{'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
