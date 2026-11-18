"""MorphMesh plane-stress kernel against beam theory (closed form).

A slender strip, clamped along its base, loaded at the boss ring like a chassis tab, solved with MorphMesh's own
finite-element kernel (``_PlateFE``) on the full material field. Three numbers are compared with their closed forms:
axial displacement P·a/(E·A), transverse displacement P·a³/(3EI) + P·a/(κGA) (Timoshenko, κ = 5/6), and the bending
stress σ = M·x/I at the outermost element centroid half-way up. Limits: benchmarks/morphmesh/limits.json,
registered before the run. Results: benchmarks/morphmesh/results.json.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from suspension import morphmesh as mm  # noqa: E402

OUT = HERE / "benchmarks" / "morphmesh"
W, T, H, A_ARM, P = 10.0, 4.0, 100.0, 90.0, 1000.0


def run(h: float) -> dict:
    dom = mm.PlateDomain(width_mm=W, height_mm=H, h_mm=h, thickness_mm=T, load_bore=(W / 2, A_ARM, 1.0),
                         ring_mm=1.0, name="benchmark strip")
    fe = mm._PlateFE(dom)
    x = np.ones((fe.ny, fe.nx))
    E, nu = fe.E0, fe.nu
    G, A, I = E / (2 * (1 + nu)), W * T, T * W ** 3 / 12.0
    (Ua, Ub), _ = fe.solve_cases(x, [mm.LoadCase(np.array(d), P, 1.0, 1, 0.0, 0.0, a) for d, a in (((0.0, 1.0), 90.0), ((1.0, 0.0), 0.0))])
    n = fe.load_nodes
    ua, ub = float(np.mean(Ua[2 * n + 1])), float(np.mean(Ub[2 * n]))
    ua_cf = P * A_ARM / (E * A)
    ub_cf = P * A_ARM ** 3 / (3 * E * I) + P * A_ARM / (5 / 6 * G * A)
    sig = fe.stress_tensor(Ub).reshape(fe.ny, fe.nx, 3)
    iy = int(round(A_ARM / 2 / h - 0.5))
    yc, xc = (iy + 0.5) * h, W - h / 2
    s_fe = float(sig[iy, -1, 1])
    s_cf = -P * (A_ARM - yc) * (xc - W / 2) / I          # load +x bends the strip: the +x side is in compression
    return {"h_mm": h, "elements_across": fe.nx,
            "axial_mm": ua, "axial_closed_mm": ua_cf, "axial_rel": abs(ua / ua_cf - 1),
            "transverse_mm": ub, "transverse_closed_mm": ub_cf, "transverse_rel": abs(ub / ub_cf - 1),
            "stress_MPa": s_fe, "stress_closed_MPa": s_cf, "stress_rel": abs(s_fe / s_cf - 1)}


def main() -> dict:
    rows = [run(h) for h in (1.0, 0.5, 0.25)]
    lim = json.loads((OUT / "limits.json").read_text())["limits_at_finest_mesh"]
    f = rows[-1]
    passes = {"axial": f["axial_rel"] <= lim["axial_displacement_rel"],
              "transverse": f["transverse_rel"] <= lim["transverse_displacement_rel"],
              "stress": f["stress_rel"] <= lim["bending_stress_rel"],
              "converges": all(b["transverse_rel"] < a["transverse_rel"] for a, b in zip(rows, rows[1:]))}
    res = {"rows": rows, "passes": passes}
    (OUT / "results.json").write_text(json.dumps(res, indent=1) + "\n")
    return res


if __name__ == "__main__":
    r = main()
    for row in r["rows"]:
        print({k: (round(v, 5) if isinstance(v, float) else v) for k, v in row.items()})
    print(r["passes"])
