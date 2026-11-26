# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Whole-frame torsion, bending and mount stiffness of an FSAE-style space frame: frame_fe.py against OpenSeesPy.
Limits: benchmarks/frame_torsion_opensees/limits.json.

    python scripts/frame_torsion_opensees_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import frame_fe as ff  # noqa: E402

BENCH = ROOT / "benchmarks" / "frame_torsion_opensees"
REG = json.loads((BENCH / "limits.json").read_text())
MAIN, SIDE, BRACE = (25.4, 2.4), (25.4, 1.65), (25.4, 1.25)
F = 1000.0
MAX_EL = 50.0


def frame_points():
    """Nodes (x lateral, y up, z longitudinal; mm) and tubes (a, b, section) of a generic FSAE space frame."""
    P = {}
    for s, sx in (("L", -1), ("R", 1)):
        P["fb_lo" + s] = (sx * 150, 100, 0); P["fb_hi" + s] = (sx * 150, 400, 0)          # front bulkhead
        P["fh_lo" + s] = (sx * 250, 100, 600); P["fh_hi" + s] = (sx * 200, 650, 650)        # front hoop
        P["mid_lo" + s] = (sx * 300, 100, 1150); P["mid_hi" + s] = (sx * 300, 350, 1150)    # side impact station
        P["mh_lo" + s] = (sx * 300, 100, 1500); P["mh_hi" + s] = (sx * 250, 1050, 1500)     # main hoop
        P["mh_si" + s] = (sx * 300, 350, 1500)
        P["rb_lo" + s] = (sx * 250, 100, 2100); P["rb_hi" + s] = (sx * 250, 400, 2100)      # rear box
        P["fm" + s] = (sx * 200, 250, 300)                                                   # front mounts
        P["rm" + s] = (sx * 250, 250, 2100)
    T = []
    for s in ("L", "R"):
        T += [("fb_lo" + s, "fb_hi" + s, SIDE), ("fb_lo" + s, "fm" + s, SIDE), ("fb_hi" + s, "fm" + s, SIDE),
              ("fm" + s, "fh_lo" + s, SIDE), ("fm" + s, "fh_hi" + s, BRACE),
              ("fb_lo" + s, "fh_lo" + s, SIDE), ("fb_hi" + s, "fh_hi" + s, SIDE),
              ("fh_lo" + s, "fh_hi" + s, MAIN),
              ("fh_lo" + s, "mid_lo" + s, SIDE), ("mid_lo" + s, "mh_lo" + s, SIDE),
              ("fh_hi" + s, "mid_hi" + s, SIDE), ("mid_hi" + s, "mh_si" + s, SIDE),
              ("fh_lo" + s, "mid_hi" + s, SIDE), ("mid_lo" + s, "mh_si" + s, SIDE),
              ("mid_lo" + s, "mid_hi" + s, BRACE),
              ("mh_lo" + s, "mh_si" + s, MAIN), ("mh_si" + s, "mh_hi" + s, MAIN),
              ("mh_hi" + s, "rb_hi" + s, SIDE), ("mh_lo" + s, "rb_lo" + s, SIDE), ("mh_si" + s, "rb_hi" + s, SIDE),
              ("mh_lo" + s, "rb_hi" + s, BRACE),
              ("rb_lo" + s, "rm" + s, SIDE), ("rm" + s, "rb_hi" + s, SIDE)]
    T += [("fb_loL", "fb_loR", SIDE), ("fb_hiL", "fb_hiR", SIDE), ("fb_loL", "fb_hiR", BRACE),
          ("fh_loL", "fh_loR", MAIN), ("fh_hiL", "fh_hiR", MAIN), ("fmL", "fmR", BRACE),
          ("mid_loL", "mid_loR", SIDE), ("mid_loL", "fh_loR", BRACE), ("mid_loR", "mh_loL", BRACE),
          ("mh_loL", "mh_loR", MAIN), ("mh_hiL", "mh_hiR", MAIN), ("mh_siL", "mh_siR", SIDE),
          ("rb_loL", "rb_loR", SIDE), ("rb_hiL", "rb_hiR", SIDE), ("rb_loL", "rb_hiR", BRACE),
          ("rmL", "rmR", SIDE), ("mh_loL", "rb_loR", BRACE), ("mh_siL", "rb_hiR", BRACE)]
    return P, T


def build_kinematik():
    P, T = frame_points()
    fr = ff.Frame()
    idx = {k: fr.add_node(v) for k, v in P.items()}
    for a, b, (od, w) in T:
        fr.add_elem(idx[a], idx[b], ff.Section(od, w), max_el_mm=MAX_EL)
    return fr, idx


def cases(fr, idx):
    """{name: (loads, fixed, (node, dof) list to read)} on node indices of this frame."""
    fixed = {idx["rmL"]: range(6), idx["rmR"]: range(6)}
    floor = ["mid_loL", "mid_loR", "mh_loL", "mh_loR"]
    return {
        "torsion": ({idx["fmR"]: [0, F, 0, 0, 0, 0], idx["fmL"]: [0, -F, 0, 0, 0, 0]}, fixed),
        "bending": ({idx[k]: [0, -F, 0, 0, 0, 0] for k in floor}, fixed),
        "mount": ({idx["fmR"]: [0, F, 0, 0, 0, 0]}, fixed),
    }


def quantities(u, fr, idx) -> dict:
    """torsion N m/deg, bending max |uy| mm, mount N/mm, from a displacement field per case."""
    xL, xR = fr.nodes[idx["fmL"]][0], fr.nodes[idx["fmR"]][0]
    ut = u["torsion"]
    theta = math.degrees(math.atan2(ut[idx["fmR"]][1] - ut[idx["fmL"]][1], xR - xL))
    return {"torsion_Nm_per_deg": F * (xR - xL) / 1000.0 / theta,
            "bending_max_deflection_mm": float(np.abs(u["bending"][:, 1]).max()),
            "mount_N_per_mm": F / u["mount"][idx["fmR"]][1]}


def solve_kinematik() -> dict:
    fr, idx = build_kinematik()
    u = {n: fr.solve(l, f) for n, (l, f) in cases(fr, idx).items()}
    return quantities(u, fr, idx), fr, idx


def solve_opensees(fr, idx) -> dict:
    """The same nodes and elements in OpenSeesPy: elasticBeamColumn, one linear static solve per case."""
    import openseespy.opensees as ops
    u = {}
    for name, (loads, fixed) in cases(fr, idx).items():
        ops.wipe()
        ops.model("basic", "-ndm", 3, "-ndf", 6)
        for k, p in enumerate(fr.nodes):
            ops.node(k + 1, *map(float, p))
        for nd, dofs in fixed.items():
            flags = [1 if d in set(dofs) else 0 for d in range(6)]
            ops.fix(nd + 1, *flags)
        for e, (i, j, s, _) in enumerate(fr.elems):
            a, b = fr.nodes[i], fr.nodes[j]
            ex = (b - a) / np.linalg.norm(b - a)
            vz = np.array([0.0, 0.0, 1.0]) if abs(ex[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
            ops.geomTransf("Linear", e + 1, *map(float, np.cross(ex, np.cross(vz, ex))))
            ops.element("elasticBeamColumn", e + 1, i + 1, j + 1, s.A, ff.E_STEEL, ff.G_STEEL, s.J, s.I, s.I, e + 1)
        ops.timeSeries("Linear", 1)
        ops.pattern("Plain", 1, 1)
        for nd, v in loads.items():
            ops.load(nd + 1, *map(float, v))
        ops.system("UmfPack"); ops.numberer("RCM"); ops.constraints("Plain")
        ops.integrator("LoadControl", 1.0); ops.algorithm("Linear"); ops.analysis("Static")
        if ops.analyze(1) != 0:
            raise RuntimeError("OpenSees failed on " + name)
        u[name] = np.array([ops.nodeDisp(k + 1) for k in range(len(fr.nodes))])
    ops.wipe()
    return quantities(u, fr, idx)


def run() -> dict:
    k, fr, idx = solve_kinematik()
    o = solve_opensees(fr, idx)
    lim = REG["limits"]["relative_each_quantity"]
    rows = {q: {"kinematik": k[q], "opensees": o[q], "rel": k[q] / o[q] - 1} for q in k}
    return {"n_nodes": len(fr.nodes), "n_elements": len(fr.elems), "n_tubes": len(frame_points()[1]),
            "rows": rows, "passed": bool(all(abs(r["rel"]) <= lim for r in rows.values()))}


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    print(f"{res['n_tubes']} tubes, {res['n_elements']} elements")
    for q, r in res["rows"].items():
        print(f"  {q}: KinematiK {r['kinematik']:.4f}, OpenSees {r['opensees']:.4f}, {100 * r['rel']:+.5f} %")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
