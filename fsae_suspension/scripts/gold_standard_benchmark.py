"""Bracket FoS and MorphMesh against the published results of commercial FE tools (Ansys, MSC Nastran, Creo Simulate,
Autodesk Nastran, Abaqus), taken from their own verification manuals.

Every problem, published value and limit is in benchmarks/gold_standard/limits.json, registered before KinematiK was
run on these problems. A result passes if it is within the limit of the tool's published value, or at least as close to
the manual's theory value as the tool itself is. Results: benchmarks/gold_standard/results.json.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from suspension import bracket_fos as bf  # noqa: E402
from suspension import morphmesh as mm  # noqa: E402

OUT = HERE / "benchmarks" / "gold_standard"


def bracket_stress(width: float, thickness: float, P: float, lever: float) -> float:
    res = bf.screen_bracket(bf.Bracket("check", "Steel 4130 (normalized tube)", width, thickness, P, lever_arm_mm=lever))
    return next(m.stress_MPa for m in res.modes if m.mode == "root bending")


def morphmesh_cantilever(nu: float, n_across: int = 8, L: float = 6.0, d: float = 0.2, t: float = 0.1,
                         E: float = 1e7) -> dict:
    """Tip displacement (in) of the MacNeal-Harder straight cantilever under unit axial and unit in-plane shear."""
    h = d / n_across
    dom = mm.PlateDomain(width_mm=d, height_mm=L, h_mm=h, thickness_mm=t, load_bore=(d / 2, L / 2, h),
                         ring_mm=h, name="MacNeal-Harder cantilever")
    fe = mm._PlateFE(dom)
    fe.E0, fe.nu = E, nu
    fe.KE, fe.B0, fe.D1 = mm._q4_ke_and_b(nu, h)
    top = [ix * (fe.ny + 1) + fe.ny for ix in range(fe.nx + 1)]
    w = np.ones(len(top))
    w[[0, -1]] = 0.5
    w /= w.sum()
    fa, fs = np.zeros(fe.ndof), np.zeros(fe.ndof)
    fa[[2 * n + 1 for n in top]] = w                                        # unit axial
    fs[[2 * n for n in top]] = w                                            # unit in-plane shear
    cases = [mm.LoadCase(np.array((0.0, 1.0)), 1.0, 1.0, 1, 0.0, 0.0, 90.0),
             mm.LoadCase(np.array((1.0, 0.0)), 1.0, 1.0, 1, 0.0, 0.0, 0.0)]
    loads = {id(cases[0]): fa, id(cases[1]): fs}
    fe.force_vector = lambda case: loads[id(case)]
    (Ua, Us), _ = fe.solve_cases(np.ones((fe.ny, fe.nx)), cases)
    return {"axial_in": float(np.mean(Ua[[2 * n + 1 for n in top]])),
            "inplane_shear_in": float(np.mean(Us[[2 * n for n in top]]))}


def _judge(k: float, pub: float, theory: float, lim: float) -> dict:
    rel = abs(k / pub - 1)
    return {"kinematik": k, "published": pub, "theory": theory, "rel_to_published": rel,
            "kinematik_rel_to_theory": abs(k / theory - 1), "tool_rel_to_theory": abs(pub / theory - 1),
            "passes": rel <= lim or abs(k / theory - 1) <= abs(pub / theory - 1)}


def main() -> dict:
    reg = json.loads((OUT / "limits.json").read_text())
    rows = []
    for p in reg["bracket_root_bending"]:
        k = bracket_stress(p["width"], p["thickness"], p["P"], p["lever"])
        rows.append({"feature": "Bracket FoS", "tool": p["tool"], "source": p["source"], "quantity": "root bending psi",
                     **_judge(k, p["published_psi"], p["theory_psi"], reg["bracket_limit_rel"])})
    mc = reg["morphmesh_cantilever"]
    cache = {}
    for p in mc["published"]:
        r = cache.setdefault(p["nu"], morphmesh_cantilever(p["nu"]))
        for q, th in (("axial_in", "theory_axial"), ("inplane_shear_in", "theory_shear")):
            if q in p:
                rows.append({"feature": "MorphMesh", "tool": p["tool"], "source": p["source"], "quantity": q,
                             **_judge(r[q], p[q], p[th], mc["limit_rel"])})
    tools = {(r["feature"], r["tool"]) for r in rows}
    res = {"rows": rows, "all_pass": all(r["passes"] for r in rows),
           "tools_per_feature": {f: sorted(t for g, t in tools if g == f) for f in ("Bracket FoS", "MorphMesh")}}
    (OUT / "results.json").write_text(json.dumps(res, indent=1) + "\n")
    return res


if __name__ == "__main__":
    res = main()
    for r in res["rows"]:
        print(f'{r["feature"]:12} {r["tool"][:34]:34} {r["quantity"]:18} K={r["kinematik"]:.6g} pub={r["published"]:.6g} '
              f'th={r["theory"]:.6g} rel={r["rel_to_published"]:.4f} pass={r["passes"]}')
    print(res["all_pass"], res["tools_per_feature"])
