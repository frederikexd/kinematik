"""MorphMesh plane-stress kernel against beam theory (closed form).

A slender strip, clamped along its base, loaded at the boss ring like a chassis tab, solved with MorphMesh's own
finite-element kernel (``_PlateFE``) on the full material field. Three numbers are compared with their closed forms:
axial displacement P·a/(E·A), transverse displacement P·a³/(3EI) + P·a/(κGA) (Timoshenko, κ = 5/6), and the bending
stress σ = M·x/I at the outermost element centroid half-way up. Limits: benchmarks/morphmesh/limits.json,
registered before the run. Results: benchmarks/morphmesh/results.json.

The same strip, grid and loaded nodes are also solved in three other finite-element programs (scikit-fem and SfePy with
9-node quads, OpenSees with its 4-node plane-stress quad) and compared with MorphMesh (``main(tools=True)``; needs
``pip install scikit-fem sfepy openseespy``).
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


# --- three other tools on the same strip, same grid, same loaded nodes --------------------------------------------- #
def _setup(h: float = 0.25):
    dom = mm.PlateDomain(width_mm=W, height_mm=H, h_mm=h, thickness_mm=T, load_bore=(W / 2, A_ARM, 1.0),
                         ring_mm=1.0, name="benchmark strip")
    fe = mm._PlateFE(dom)
    ld = np.array([((n // (fe.ny + 1)) * h, (n % (fe.ny + 1)) * h) for n in fe.load_nodes])
    iy = int(round(A_ARM / 2 / h - 0.5))
    probe = (W - h / 2, (iy + 0.5) * h)
    return fe, ld, probe


def kinematik_strip(h: float = 0.25) -> dict:
    fe, ld, probe = _setup(h)
    x = np.ones((fe.ny, fe.nx))
    (Ua, Ub), _ = fe.solve_cases(x, [mm.LoadCase(np.array(d), P, 1.0, 1, 0.0, 0.0, a)
                                     for d, a in (((0.0, 1.0), 90.0), ((1.0, 0.0), 0.0))])
    n = fe.load_nodes
    sig = fe.stress_tensor(Ub).reshape(fe.ny, fe.nx, 3)
    iy = int(round(A_ARM / 2 / h - 0.5))
    return {"axial_mm": float(np.mean(Ua[2 * n + 1])), "transverse_mm": float(np.mean(Ub[2 * n])),
            "stress_MPa": float(sig[iy, -1, 1])}


def _match(coords: np.ndarray, pts: np.ndarray) -> np.ndarray:
    idx = [int(np.argmin(np.hypot(coords[:, 0] - x, coords[:, 1] - y))) for x, y in pts]
    assert np.allclose(coords[idx], pts, atol=1e-9)
    return np.array(idx)


def skfem_strip(h: float = 0.25) -> dict:
    from skfem import Basis, BilinearForm, ElementQuad2, ElementVector, MeshQuad, condense, solve
    from skfem.models.elasticity import lame_parameters, linear_elasticity
    fe, ld, probe = _setup(h)
    E, nu = fe.E0, fe.nu
    m = MeshQuad.init_tensor(np.linspace(0, W, fe.nx + 1), np.linspace(0, H, fe.ny + 1))
    ib = Basis(m, ElementVector(ElementQuad2()))
    lam, mu = lame_parameters(E, nu)
    K = BilinearForm(linear_elasticity(2 * lam * mu / (lam + 2 * mu), mu)).assemble(ib) * T   # plane stress
    vid = _match(m.p.T, ld)
    D = ib.get_dofs(lambda x: x[1] < 1e-9).all()
    out = {}
    for key, (cx, cy) in (("axial", (0, 1)), ("bend", (1, 0))):
        f = np.zeros(ib.N)
        f[ib.nodal_dofs[0, vid]] += P * cx / len(vid)
        f[ib.nodal_dofs[1, vid]] += P * cy / len(vid)
        u = solve(*condense(K, f, D=D))
        out[key] = u
    ua = float(np.mean(out["axial"][ib.nodal_dofs[1, vid]]))
    ub = float(np.mean(out["bend"][ib.nodal_dofs[0, vid]]))
    g = ib.interpolate(out["bend"]).grad
    lam_s = 2 * lam * mu / (lam + 2 * mu)
    syy = 2 * mu * g[1, 1] + lam_s * (g[0, 0] + g[1, 1])
    sb = ib.with_element(ElementQuad2())
    s = sb.probes(np.array([[probe[0]], [probe[1]]])) @ sb.project(syy)
    return {"axial_mm": ua, "transverse_mm": ub, "stress_MPa": float(s[0])}


def sfepy_strip(h: float = 0.25) -> dict:
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
    fe, ld, probe = _setup(h)
    mesh = gen_block_mesh([W, H], [fe.nx + 1, fe.ny + 1], [W / 2, H / 2], name="strip", verbose=False)
    vid = _match(mesh.coors, ld)
    dom = FEDomain("d", mesh)
    om = dom.create_region("Omega", "all")
    base = dom.create_region("Base", "vertices in (y < 1e-6)", "facet")
    lr = dom.create_region("Load", "vertex " + ", ".join(str(i) for i in vid), "vertex")
    fld = Field.from_args("u", np.float64, "vector", om, approx_order=2)
    u = FieldVariable("u", "unknown", fld)
    v = FieldVariable("v", "test", fld, primary_var_name="u")
    m = Material("m", D=stiffness_from_youngpoisson(2, fe.E0, fe.nu, plane="stress"))
    i = Integral("i", order=4)
    res = {}
    for key, (cx, cy) in (("axial", (0, 1)), ("bend", (1, 0))):
        ld_m = Material("ld", values={".val": [P * cx / len(vid) / T, P * cy / len(vid) / T]})
        t1 = Term.new("dw_lin_elastic(m.D, v, u)", i, om, m=m, v=v, u=u)
        t2 = Term.new("dw_point_load(ld.val, v)", Integral("i0", order=0), lr, ld=ld_m, v=v)
        pb = Problem("strip", equations=Equations([Equation("eq", t1 - t2)]))
        pb.set_solver(Newton({"i_max": 1, "eps_a": 1e-12}, lin_solver=ScipyDirect({})))
        pb.set_bcs(ebcs=Conditions([EssentialBC("fix", base, {"u.all": 0.0})]))
        st = pb.solve(save_results=False)
        vals = st["u"]()
        res[key] = (vals.reshape(-1, 2), pb, m, st, i)
    va = res["axial"][0][: mesh.n_nod]
    vb = res["bend"][0][: mesh.n_nod]
    _, pb, m_, st, i_ = res["bend"]
    s = pb.evaluate("ev_cauchy_stress.i.Omega(m.D, u)", mode="el_avg", m=m_, u=st["u"], copy_materials=False,
                    integrals=Integrals([i_]))
    s = np.asarray(s).reshape(-1, 3)[:, 1]
    c = mesh.cmesh.get_centroids(2)
    e = int(np.argmin(np.hypot(c[:, 0] - probe[0], c[:, 1] - probe[1])))
    return {"axial_mm": float(np.mean(va[vid, 1])), "transverse_mm": float(np.mean(vb[vid, 0])),
            "stress_MPa": float(s[e])}


def opensees_strip(h: float = 0.25) -> dict:
    import openseespy.opensees as ops
    fe, ld, probe = _setup(h)
    nx, ny = fe.nx, fe.ny
    out = {}
    for key, (cx, cy) in (("axial", (0, 1)), ("bend", (1, 0))):
        ops.wipe()
        ops.model("basic", "-ndm", 2, "-ndf", 2)
        for ix in range(nx + 1):
            for iy in range(ny + 1):
                ops.node(1 + ix * (ny + 1) + iy, ix * h, iy * h)
                if iy == 0:
                    ops.fix(1 + ix * (ny + 1) + iy, 1, 1)
        ops.nDMaterial("ElasticIsotropic", 1, fe.E0, fe.nu)
        tag = 0
        etag = {}
        for ix in range(nx):
            for iy in range(ny):
                tag += 1
                n1 = 1 + ix * (ny + 1) + iy
                ops.element("quad", tag, n1, n1 + ny + 1, n1 + ny + 2, n1 + 1, T, "PlaneStress", 1)
                etag[(ix, iy)] = tag
        ops.timeSeries("Linear", 1)
        ops.pattern("Plain", 1, 1)
        for n in fe.load_nodes:
            ops.load(int(n) + 1, P * cx / len(fe.load_nodes), P * cy / len(fe.load_nodes))
        ops.system("UmfPack")
        ops.numberer("RCM")
        ops.constraints("Plain")
        ops.integrator("LoadControl", 1.0)
        ops.algorithm("Linear")
        ops.analysis("Static")
        ops.analyze(1)
        d = 1 if key == "axial" else 0
        out[key] = float(np.mean([ops.nodeDisp(int(n) + 1, d + 1) for n in fe.load_nodes]))
        if key == "bend":
            iy = int(round(A_ARM / 2 / h - 0.5))
            st = np.array(ops.eleResponse(etag[(nx - 1, iy)], "stresses")).reshape(4, 3)
            out["stress"] = float(st[:, 1].mean())
        ops.wipe()
    return {"axial_mm": out["axial"], "transverse_mm": out["bend"], "stress_MPa": out["stress"]}


TOOLS = {"scikit-fem 12.0.2 (9-node quads)": skfem_strip, "SfePy 2026.3 (9-node quads)": sfepy_strip,
         "OpenSeesPy 3.8.0.0 (quad, PlaneStress)": opensees_strip}


def compare_tools(tools: dict | None = None) -> dict:
    """Each other tool against MorphMesh on the 0.25 mm strip, relative differences, and pass/fail on the limits."""
    lim = json.loads((OUT / "limits.json").read_text())["other_tool_limits"]
    ref = kinematik_strip()
    out = {"morphmesh": ref}
    for name, fn in (tools or TOOLS).items():
        r = fn()
        rel = {k: abs(r[k] / ref[k] - 1) for k in ref}
        out[name] = {"values": r, "rel": rel, "passes": rel["axial_mm"] <= lim["axial_displacement_rel"]
                     and rel["transverse_mm"] <= lim["transverse_displacement_rel"]
                     and rel["stress_MPa"] <= lim["bending_stress_rel"]}
    return out


def main(tools: bool = False) -> dict:
    rows = [run(h) for h in (1.0, 0.5, 0.25)]
    lim = json.loads((OUT / "limits.json").read_text())["limits_at_finest_mesh"]
    f = rows[-1]
    passes = {"axial": f["axial_rel"] <= lim["axial_displacement_rel"],
              "transverse": f["transverse_rel"] <= lim["transverse_displacement_rel"],
              "stress": f["stress_rel"] <= lim["bending_stress_rel"],
              "converges": all(b["transverse_rel"] < a["transverse_rel"] for a, b in zip(rows, rows[1:]))}
    res = {"rows": rows, "passes": passes}
    path = OUT / "results.json"
    if tools:
        res["other_tools"] = compare_tools()
    elif path.exists():                                   # keep the recorded other-tool run
        old = json.loads(path.read_text()).get("other_tools")
        if old:
            res["other_tools"] = old
    (OUT / "results.json").write_text(json.dumps(res, indent=1) + "\n")
    return res


if __name__ == "__main__":
    r = main(tools=True)
    for name, t in r["other_tools"].items():
        print(name, t)
    for row in r["rows"]:
        print({k: (round(v, 5) if isinstance(v, float) else v) for k, v in row.items()})
    print(r["passes"])
