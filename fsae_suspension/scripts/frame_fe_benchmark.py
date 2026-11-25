# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The chassis frame solver (suspension/frame_fe.py) against Ansys Mechanical APDL's published verification problems.

  VM18   Out-of-plane bending of a curved bar: bending and torsion together, the load pattern of a frame in twist
  VM217  Portal frame under symmetric loading: a statically indeterminate frame with stiff and soft members

Problem data, targets, Mechanical APDL's published values and limits: benchmarks/frame_fe/limits.json, registered
before the run. Results: benchmarks/frame_fe/results.json.

frame_fe's elements read only a section's A, I and J, so VM217's I-beams go in as a section object carrying those.
Member end forces (for stresses and moments) are recovered from the solved displacements with frame_fe's own element
matrices: f = k_local T u, minus the fixed-end forces of a member's distributed load.

    python scripts/frame_fe_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import frame_fe as ff  # noqa: E402

BENCH = ROOT / "benchmarks" / "frame_fe"
REG = json.loads((BENCH / "limits.json").read_text())


@dataclass
class _Sec:
    """A section by its properties, for shapes that are not round tubes."""
    A: float
    I: float
    J: float


def _end_forces(fr: ff.Frame, u: np.ndarray, e: int) -> np.ndarray:
    i, j, s, _ = fr.elems[e]
    a, b = fr.nodes[i], fr.nodes[j]
    T = fr._T(a, b)
    ue = np.r_[u[i], u[j]]
    return fr._k_local(float(np.linalg.norm(b - a)), s) @ (T @ ue)


def vm18() -> dict:
    p = REG["problems"]["VM18"]["data"]
    r, F, E, nu, d = p["r_in"], p["F_lb"], p["E_psi"], p["nu"], p["d_in"]
    fr = ff.Frame(E=E, G=E / (2 * (1 + nu)))
    sec = ff.Section(od_mm=d, wall_mm=d / 2)                 # a solid bar: the units are inches, the class is unit-free
    n = 90
    th = np.radians(np.linspace(0.0, p["arc_deg"], n + 1))
    for t in th:
        fr.add_node((r * np.cos(t), r * np.sin(t), 0.0))
    for k in range(n):
        fr.add_elem(k, k + 1, sec)
    u = fr.solve({n: (0, 0, -F, 0, 0, 0)}, {0: range(6)})
    f0 = _end_forces(fr, u, 0)                                # element 0 at the built-in end A
    R = fr._T(fr.nodes[0], fr.nodes[1])[:3, :3]
    m = R.T @ f0[3:6]                                         # end moment at A in global axes
    tangent = np.array([0.0, 1.0, 0.0])                       # the ring's tangent at A (theta = 0)
    c = d / 2
    Tq = float(abs(m @ tangent))                              # torsion: the moment along the bar's tangent
    M = float(np.linalg.norm(m - (m @ tangent) * tangent))    # bending: the rest
    # (A first run split the moment along element 0's chord, 0.5 deg off the tangent, and missed by tan 0.5 deg.)
    return {"deflection_in": float(u[n][2]), "bending_stress_psi": M * c / sec.I, "torsion_shear_psi": Tq * c / sec.J}


def vm217() -> dict:
    p = REG["problems"]["VM217"]["data"]
    a, L, w, E = p["a_in"], p["L_in"], p["w_lb_per_in"], p["E_psi"]
    Ic = p["I_col_in4"]
    col = _Sec(A=1e6, I=Ic, J=2 * Ic)
    span = _Sec(A=1e6, I=p["I_span_over_I_col"] * Ic, J=2 * p["I_span_over_I_col"] * Ic)
    fr = ff.Frame(E=E, G=E / 2.6)
    b0, b1 = fr.add_node((0, 0, 0)), fr.add_node((L, 0, 0))
    c0, c1 = fr.add_node((0, a, 0)), fr.add_node((L, a, 0))
    fr.add_elem(b0, c0, col, "col", max_el_mm=a / 4)
    fr.add_elem(b1, c1, col, "col", max_el_mm=a / 4)
    first_span = len(fr.elems)
    fr.add_elem(c0, c1, span, "span", max_el_mm=L / 16)
    loads, fe = {}, {}
    for e in range(first_span, len(fr.elems)):                # consistent nodal loads of the uniform load, -y
        i, j, _, _ = fr.elems[e]
        le = float(np.linalg.norm(fr.nodes[j] - fr.nodes[i]))
        for nd, sg in ((i, 1.0), (j, -1.0)):
            v = loads.setdefault(nd, np.zeros(6))
            v[1] += -w * le / 2
            v[5] += -sg * w * le ** 2 / 12
        fe[e] = (le, w)
    u = fr.solve(loads, {b0: range(6), b1: range(6)})
    moments = []
    for e in range(first_span, len(fr.elems)):
        f = _end_forces(fr, u, e)
        le, ww = fe[e]
        # remove the fixed-end forces of the member load to get the true end moments (local z moment, DOF 5 and 11)
        m_i = f[5] - (-ww * le ** 2 / 12)
        m_j = f[11] - (ww * le ** 2 / 12)
        moments += [abs(m_i), abs(m_j)]
    i_mid = [k for k, nd in enumerate(fr.nodes) if abs(nd[0] - L / 2) < 1e-6 and abs(nd[1] - a) < 1e-6][0]
    return {"max_rotation_rad": float(abs(u[c0][5])), "max_bending_moment_lb_in": float(max(moments)),
            "midspan_deflection_in": float(u[i_mid][1])}


def run() -> dict:
    out, ok = {}, True
    for name, fn in (("VM18", vm18), ("VM217", vm217)):
        P = REG["problems"][name]
        got = fn()
        row = {}
        for q, tgt in P["target"].items():
            k, apdl = got[q], P["mechanical_apdl"][q]
            rel = abs(abs(k) / abs(tgt) - 1)
            passes = bool(rel <= P["limit_relative"] or rel <= abs(abs(apdl) / abs(tgt) - 1))
            row[q] = {"kinematik": k, "target": tgt, "mechanical_apdl": apdl, "rel_to_target": rel,
                      "apdl_rel_to_target": abs(abs(apdl) / abs(tgt) - 1), "passes": passes}
            ok = ok and passes
        out[name] = row
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for name in ("VM18", "VM217"):
        for q, r in res[name].items():
            print(f"{name:6s} {q:26s} KinematiK {r['kinematik']:+.5g}  target {r['target']:+.5g}  "
                  f"APDL {r['mechanical_apdl']:+.5g}  ({100 * r['rel_to_target']:.3f} % vs APDL "
                  f"{100 * r['apdl_rel_to_target']:.2f} %) {'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
