"""Bracket FoS root bending against an independent 3D finite-element model (scikit-fem).

A welded tab is modelled as a 3D solid (27-node hexahedra, scikit-fem, no KinematiK code), root face fixed, the load as
a uniform traction on the free end. At each section the FEA top-surface bending stress is compared with KinematiK's
screen_bracket root-bending stress for a lever arm equal to the section's distance from the load: the width average
and the peak across the width. The root itself is reported, not judged: a fully fixed face concentrates stress at its
edges, which hand screening does not claim to capture. Limits: benchmarks/bracket_fea/limits.json, registered before
the run. Results: benchmarks/bracket_fea/results.json. Needs ``pip install scikit-fem``.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from suspension import bracket_fos as bf  # noqa: E402

OUT = HERE / "benchmarks" / "bracket_fea"
L, W, T, P, E, NU = 40.0, 30.0, 4.0, 500.0, 205000.0, 0.29
MAT = "Steel 4130 (normalized tube)"


def kinematik_stress(arm_mm: float) -> float:
    res = bf.screen_bracket(bf.Bracket("tab", MAT, W, T, P, lever_arm_mm=arm_mm))
    return next(m.stress_MPa for m in res.modes if m.mode == "root bending")


def fea(nx: int = 40, ny: int = 12, nz: int = 4):
    from skfem import (Basis, BilinearForm, ElementHex2, ElementVector, FacetBasis, LinearForm, MeshHex,
                       condense, solve)
    from skfem.models.elasticity import lame_parameters, linear_elasticity
    m = MeshHex.init_tensor(np.linspace(0, L, nx + 1), np.linspace(0, W, ny + 1), np.linspace(0, T, nz + 1))
    e = ElementVector(ElementHex2())
    ib = Basis(m, e, intorder=4)
    lam, mu = lame_parameters(E, NU)
    K = BilinearForm(linear_elasticity(lam, mu)).assemble(ib)
    fb = FacetBasis(m, e, facets=m.facets_satisfying(lambda x: x[0] > L - 1e-9), intorder=4)
    q = P / (W * T)

    @LinearForm
    def trac(v, w):
        return -q * v[2]

    f = trac.assemble(fb)
    D = ib.get_dofs(lambda x: x[0] < 1e-9).all()
    u = solve(*condense(K, f, D=D))
    g = ib.interpolate(u).grad
    exx, eyy, ezz = g[0, 0], g[1, 1], g[2, 2]
    sxx = 2 * mu * exx + lam * (exx + eyy + ezz)
    sb = ib.with_element(ElementHex2())
    s_nodal = sb.project(sxx)
    return sb, s_nodal


def main() -> dict:
    sb, s = fea()
    ys = np.linspace(0, W, 61)
    rows = []
    for x in (0.0, 2 * T, 12.0, 16.0, 20.0, 24.0, L - 2 * T):
        pts = np.vstack([np.full_like(ys, x), ys, np.full_like(ys, T)])
        top = sb.probes(pts) @ s
        k = kinematik_stress(L - x)
        avg = float(np.trapezoid(top, ys) / W)
        peak = float(np.max(np.abs(top)))
        rows.append({"x_from_root_mm": x, "kinematik_MPa": k, "fea_width_average_MPa": avg, "fea_peak_MPa": peak,
                     "average_rel": abs(abs(avg) / k - 1), "peak_rel": abs(peak / k - 1), "judged": x >= 2 * T})
    lim = json.loads((OUT / "limits.json").read_text())["limits"]
    judged = [r for r in rows if r["judged"]]
    passes = {"width_average": all(r["average_rel"] <= lim["width_average_rel"] for r in judged),
              "peak_across_width": all(r["peak_rel"] <= lim["peak_across_width_rel"] for r in judged)}
    res = {"tool": "scikit-fem 12.0.2", "rows": rows, "passes": passes}
    (OUT / "results.json").write_text(json.dumps(res, indent=1) + "\n")
    return res


if __name__ == "__main__":
    r = main()
    for row in r["rows"]:
        print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()})
    print(r["passes"])
