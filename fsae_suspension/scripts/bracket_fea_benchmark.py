"""Bracket FoS root bending against three independent 3D finite-element models (scikit-fem, SfePy, OpenSees).

A welded tab is modelled as a 3D solid in each tool (no KinematiK code), root face fixed, the load as
a uniform traction on the free end. At each section the FEA top-surface bending stress is compared with KinematiK's
screen_bracket root-bending stress for a lever arm equal to the section's distance from the load: the width average
and the peak across the width. The root itself is reported, not judged: a fully fixed face concentrates stress at its
edges, which hand screening does not claim to capture. Limits: benchmarks/bracket_fea/limits.json, registered before
the run. Results: benchmarks/bracket_fea/results.json. Needs ``pip install scikit-fem sfepy openseespy``. SfePy and OpenSees
stresses are element averages, extrapolated linearly to the top surface from the top two element layers.
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


def sfepy_model(nx: int = 40, ny: int = 12, nz: int = 4):
    """SfePy 2026.3, 27-node hexahedra: element centroids and their average sxx, MPa."""
    from sfepy.base.base import output
    from sfepy.discrete import Equation, Equations, FieldVariable, Integral, Integrals, Material, Problem
    from sfepy.discrete.conditions import Conditions, EssentialBC
    from sfepy.discrete.fem import FEDomain, Field
    from sfepy.mechanics.matcoefs import stiffness_from_youngpoisson
    from sfepy.mesh.mesh_generators import gen_block_mesh
    from sfepy.solvers.ls import ScipyDirect
    from sfepy.solvers.nls import Newton
    from sfepy.terms import Term
    output.set_output(quiet=True)
    mesh = gen_block_mesh([L, W, T], [nx + 1, ny + 1, nz + 1], [L / 2, W / 2, T / 2], name="tab", verbose=False)
    dom = FEDomain("d", mesh)
    om = dom.create_region("Omega", "all")
    root = dom.create_region("Root", "vertices in (x < 1e-6)", "facet")
    end = dom.create_region("End", f"vertices in (x > {L - 1e-6:.6f})", "facet")
    fld = Field.from_args("u", np.float64, "vector", om, approx_order=2)
    u = FieldVariable("u", "unknown", fld)
    v = FieldVariable("v", "test", fld, primary_var_name="u")
    m = Material("m", D=stiffness_from_youngpoisson(3, E, NU))
    tr = Material("tr", val=np.array([[0.0], [0.0], [-P / (W * T)]]))
    i = Integral("i", order=4)
    t1 = Term.new("dw_lin_elastic(m.D, v, u)", i, om, m=m, v=v, u=u)
    t2 = Term.new("dw_surface_ltr(tr.val, v)", i, end, tr=tr, v=v)
    pb = Problem("tab", equations=Equations([Equation("eq", t1 - t2)]))
    pb.set_solver(Newton({"i_max": 1, "eps_a": 1e-10}, lin_solver=ScipyDirect({})))
    pb.set_bcs(ebcs=Conditions([EssentialBC("fix", root, {"u.all": 0.0})]))
    st = pb.solve(save_results=False)
    s = pb.evaluate("ev_cauchy_stress.i.Omega(m.D, u)", mode="el_avg", m=m, u=st["u"], copy_materials=False,
                    integrals=Integrals([i]))
    return mesh.cmesh.get_centroids(3), np.asarray(s).reshape(-1, 6)[:, 0], T / nz


def opensees_model(nx: int = 40, ny: int = 12, nz: int = 8):
    """OpenSeesPy, SSPbrick (8-node, stabilised single point): element centroids and their sxx, MPa."""
    import openseespy.opensees as ops
    ops.wipe()
    ops.model("basic", "-ndm", 3, "-ndf", 3)
    xs, ys, zs = np.linspace(0, L, nx + 1), np.linspace(0, W, ny + 1), np.linspace(0, T, nz + 1)

    def nid(i, j, k):
        return 1 + i * (ny + 1) * (nz + 1) + j * (nz + 1) + k

    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, z in enumerate(zs):
                ops.node(nid(i, j, k), float(x), float(y), float(z))
                if i == 0:
                    ops.fix(nid(i, j, k), 1, 1, 1)
    ops.nDMaterial("ElasticIsotropic", 1, E, NU)
    cents, tags = [], []
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                tag = 1 + len(tags)
                ops.element("SSPbrick", tag, nid(i, j, k), nid(i + 1, j, k), nid(i + 1, j + 1, k), nid(i, j + 1, k),
                            nid(i, j, k + 1), nid(i + 1, j, k + 1), nid(i + 1, j + 1, k + 1), nid(i, j + 1, k + 1), 1)
                tags.append(tag)
                cents.append(((xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2, (zs[k] + zs[k + 1]) / 2))
    ops.timeSeries("Linear", 1)
    ops.pattern("Plain", 1, 1)
    ay, az = np.zeros(ny + 1), np.zeros(nz + 1)
    ay[:-1] += np.diff(ys) / 2
    ay[1:] += np.diff(ys) / 2
    az[:-1] += np.diff(zs) / 2
    az[1:] += np.diff(zs) / 2
    for j in range(ny + 1):                     # the end traction as consistent nodal loads on bilinear faces
        for k in range(nz + 1):
            ops.load(nid(nx, j, k), 0.0, 0.0, -P / (W * T) * ay[j] * az[k])
    ops.system("UmfPack")
    ops.numberer("RCM")
    ops.constraints("Plain")
    ops.integrator("LoadControl", 1.0)
    ops.algorithm("Linear")
    ops.analysis("Static")
    ops.analyze(1)
    s = np.array([ops.eleResponse(t, "stress")[0] for t in tags])
    ops.wipe()
    return np.array(cents), s, T / nz


def _element_rows(cents, s, dz) -> list:
    rows = []
    for x in sorted(set(np.round(cents[:, 0], 6))):
        if x not in (0.5, 8.5, 12.5, 16.5, 20.5, 24.5, 32.5):
            continue
        a = (np.abs(cents[:, 0] - x) < 1e-6) & (np.abs(cents[:, 2] - (T - dz / 2)) < 1e-6)
        b = (np.abs(cents[:, 0] - x) < 1e-6) & (np.abs(cents[:, 2] - (T - 1.5 * dz)) < 1e-6)
        sa, sb_ = s[a][np.argsort(cents[a, 1])], s[b][np.argsort(cents[b, 1])]
        top = sa + (sa - sb_) * 0.5                       # linear extrapolation to the surface
        rows.append((float(x), float(np.mean(top)), float(np.max(np.abs(top)))))
    return rows


def _skfem_rows() -> list:
    sb, s = fea()
    ys = np.linspace(0, W, 61)
    rows = []
    for x in (0.0, 2 * T, 12.0, 16.0, 20.0, 24.0, L - 2 * T):
        top = sb.probes(np.vstack([np.full_like(ys, x), ys, np.full_like(ys, T)])) @ s
        rows.append((x, float(np.trapezoid(top, ys) / W), float(np.max(np.abs(top)))))
    return rows


TOOLS = {"scikit-fem 12.0.2 (27-node hexahedra)": _skfem_rows,
         "SfePy 2026.3 (27-node hexahedra)": lambda: _element_rows(*sfepy_model()),
         "OpenSeesPy 3.8.0.0 (SSPbrick)": lambda: _element_rows(*opensees_model())}


def main(tools: dict | None = None) -> dict:
    lim = json.loads((OUT / "limits.json").read_text())["limits"]
    out = {}
    for name, fn in (tools or TOOLS).items():
        rows = []
        for x, avg, peak in fn():
            k = kinematik_stress(L - x)
            rows.append({"x_from_root_mm": x, "kinematik_MPa": k, "fea_width_average_MPa": avg,
                         "fea_peak_MPa": peak, "average_rel": abs(abs(avg) / k - 1), "peak_rel": abs(peak / k - 1),
                         "judged": x >= 2 * T})
        judged = [r for r in rows if r["judged"]]
        out[name] = {"rows": rows, "passes": {
            "width_average": all(r["average_rel"] <= lim["width_average_rel"] for r in judged),
            "peak_across_width": all(r["peak_rel"] <= lim["peak_across_width_rel"] for r in judged)}}
    res = {"tools": out}
    (OUT / "results.json").write_text(json.dumps(res, indent=1) + "\n")
    return res


if __name__ == "__main__":
    for name, r in main()["tools"].items():
        print(name, r["passes"])
        for row in r["rows"]:
            print("  ", {k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()})
