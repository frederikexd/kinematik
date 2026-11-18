"""MorphMesh and Bracket FoS against Ansys Verification Manual VM16 (Bending of a Solid Beam, Plane Elements).

The published Mechanical APDL results (PLANE182 and PLANE42) are the reference; the problem, the values and the limits
are in benchmarks/ansys_vm16/limits.json, registered before the run. Results: benchmarks/ansys_vm16/results.json.
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

OUT = HERE / "benchmarks" / "ansys_vm16"
IN, LBF, PSI = 25.4, 4.4482216152605, 0.00689475729
L, HB, T, D = 10 * IN, 2 * IN, 1 * IN, 1 * IN
E = 30e6 * PSI
M_END, F_END = 2000 * LBF * IN, 300 * LBF


def morphmesh_vm16(nx: int = 16) -> dict:
    h = HB / nx
    dom = mm.PlateDomain(width_mm=HB, height_mm=L, h_mm=h, thickness_mm=T, load_bore=(HB / 2, L / 2, 1.0),
                         ring_mm=1.0, name="VM16 beam")
    fe = mm._PlateFE(dom)
    fe.E0, fe.nu = E, 0.0
    fe.KE, fe.B0, fe.D1 = mm._q4_ke_and_b(0.0, h)
    top = [ix * (fe.ny + 1) + fe.ny for ix in range(fe.nx + 1)]           # free-end nodes, x = 0 … h
    case_f = {"moment": np.zeros(fe.ndof), "shear": np.zeros(fe.ndof)}
    case_f["moment"][2 * top[0] + 1] = -M_END / HB                         # couple: down at x = 0, up at x = h
    case_f["moment"][2 * top[-1] + 1] = +M_END / HB
    w = np.full(len(top), 1.0)
    w[[0, -1]] = 0.5                                                       # uniform shear, consistent nodal split
    case_f["shear"][[2 * n for n in top]] = F_END * w / w.sum()
    cases = [mm.LoadCase(np.array((0.0, 1.0)), 1.0, 1.0, 1, 0.0, 0.0, 90.0),
             mm.LoadCase(np.array((1.0, 0.0)), 1.0, 1.0, 1, 0.0, 0.0, 0.0)]
    loads = {id(cases[0]): case_f["moment"], id(cases[1]): case_f["shear"]}
    fe.force_vector = lambda case: loads[id(case)]                         # VM16's end loads, not a boss ring
    (Um, Us), _ = fe.solve_cases(np.ones((fe.ny, fe.nx)), cases)
    out = {}
    iy = int(round(D / h)) - 1                                             # rows iy and iy+1 straddle y = d
    for name, U in (("case1", Um), ("case2", Us)):
        defl = float(np.mean(np.abs(U[[2 * n for n in top]])))
        s = fe.stress_tensor(U).reshape(fe.ny, fe.nx, 3)[:, :, 1]
        rows = s[iy:iy + 2]
        outer = rows[:, 0] + 0.5 * (rows[:, 0] - rows[:, 1])               # extrapolate to the x = 0 fibre
        out[name] = {"deflection_in": defl / IN, "stress_psi": float(abs(outer.mean())) / PSI}
    return out


def bracket_vm16() -> float:
    res = bf.screen_bracket(bf.Bracket("VM16", "Steel 4130 (normalized tube)", T, HB, F_END, lever_arm_mm=L - D))
    return next(m.stress_MPa for m in res.modes if m.mode == "root bending") / PSI


def main() -> dict:
    reg = json.loads((OUT / "limits.json").read_text())
    ref, lim = reg["ansys_published"]["PLANE182"], reg["limits_vs_ansys"]
    mmr, br = morphmesh_vm16(), bracket_vm16()
    rows = []
    for c in ("case1", "case2"):
        for q, key in (("deflection_in", "MorphMesh_deflection_rel"), ("stress_psi", "MorphMesh_stress_rel")):
            a = ref[f"{c}_{q}"]
            k = mmr[c][q]
            rows.append({"feature": "MorphMesh", "quantity": f"{c} {q}", "ansys": a, "kinematik": k,
                         "rel": abs(k / a - 1), "passes": abs(k / a - 1) <= lim[key]})
    a = ref["case2_stress_psi"]
    rows.append({"feature": "Bracket FoS", "quantity": "case2 stress_psi", "ansys": a, "kinematik": br,
                 "rel": abs(br / a - 1), "passes": abs(br / a - 1) <= lim["Bracket_stress_rel"]})
    res = {"rows": rows, "all_pass": all(r["passes"] for r in rows)}
    (OUT / "results.json").write_text(json.dumps(res, indent=1) + "\n")
    return res


if __name__ == "__main__":
    for r in main()["rows"]:
        print(r)
