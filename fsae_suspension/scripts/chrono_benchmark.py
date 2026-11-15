# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Project Chrono (open-source multibody engine, BSD) runs of the link-load and direct-acting benchmarks.

Shares no code with KinematiK: it reads only the corner hardpoints (manifests, benchmarks/direct_acting/corners) and the
load cases (limits.json), builds each corner as Chrono bodies and joints, and writes its results as the CSV files the
compare scripts read (benchmarks/link_loads/chrono_results.csv, benchmarks/direct_acting/chrono_damper.csv and
chrono_loads.csv). Run it in an environment with Chrono's Python module from the projectchrono conda channel (the
`pychrono` package on PyPI is an unrelated project):

    micromamba create -p ./chrono -c projectchrono -c conda-forge python=3.12 pychrono=9.0.1 numpy
    ./chrono/bin/python scripts/chrono_benchmark.py

Models, mm / N / N*mm, no gravity, unit masses (a zero-DOF static solve does not depend on them):
  member forces   upright as a free body held by ChLinkDistance members (two-force links: the pushrod, or the damper of
                  a direct-acting corner, to its chassis point); for a damper on the lower wishbone, the wishbone is a
                  body on a revolute pivot with a spherical joint to the upright and the damper on it. Loads applied as
                  body force/torque accumulators; DoStaticLinear; axial force = link reaction on the moving body,
                  projected on the outer-to-inner axis (+ tension). A one-link closed-form case is checked first.
  damper length   both wishbones on revolute pivots (the lower one driven in angle by a rotation motor), spherical ball
                  joints, the tie rod as a distance link; DoAssembly at each angle; wheel-centre travel and damper
                  length read off the solved bodies and interpolated onto -25..+25 mm of wheel-centre travel.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pychrono as ch

ROOT = Path(__file__).resolve().parents[1]
LL = ROOT / "benchmarks" / "link_loads"
DA = ROOT / "benchmarks" / "direct_acting"
CASES = json.loads((LL / "limits.json").read_text())["load_cases"]
LL_CORNERS = ("front_v11_final", "rear_v11_final", "front_v11_joints50")
DA_CORNERS = ("direct_upright", "direct_lower")
ALL6 = ("UF", "UR", "LF", "LR", "TR", "PR")
ENDS = {"UF": ("upper_outer", "upper_front_inner"), "UR": ("upper_outer", "upper_rear_inner"),
        "LF": ("lower_outer", "lower_front_inner"), "LR": ("lower_outer", "lower_rear_inner"),
        "TR": ("tie_rod_outer", "tie_rod_inner")}
TRAVEL = np.arange(-25.0, 26.0, 1.0)


def V(p):
    return ch.ChVector3d(float(p[0]), float(p[1]), float(p[2]))


def A(v):
    return np.array([v.x, v.y, v.z])


def _body(sys_, fixed=False):
    b = ch.ChBody()
    b.SetMass(1.0)
    b.SetInertiaXX(ch.ChVector3d(1.0, 1.0, 1.0))
    b.SetFixed(fixed)
    sys_.Add(b)
    return b


def _system():
    s = ch.ChSystemNSC()
    s.SetGravitationalAcceleration(ch.ChVector3d(0, 0, 0))
    s.SetSolver(ch.ChSolverSparseQR())                   # direct solver: the default iterative one (PSOR) is not exact
    return s


def _frame_z_along(p, axis):
    """A frame at p whose z axis points along axis (revolute and motor joints rotate about z)."""
    z = np.asarray(axis, float) / np.linalg.norm(axis)
    x = np.cross([0.0, 0.0, 1.0], z) if abs(z[2]) < 0.9 else np.cross([1.0, 0.0, 0.0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = ch.ChMatrix33d()
    R.SetFromDirectionAxes(V(x), V(y), V(z))
    return ch.ChFramed(V(p), R.GetQuaternion())


def _distance(sys_, moving, ground, po, pi):
    lk = ch.ChLinkDistance()
    lk.Initialize(moving, ground, False, V(po), V(pi), True)
    sys_.Add(lk)
    return lk


def _tension(lk, po, pi):
    """Axial force (+ tension): the link's reaction on the moving body, absolute, along outer -> inner."""
    r, fr = lk.GetReaction1(), lk.GetFrame1Abs()      # hold both: the binding returns temporaries
    f = fr.TransformDirectionLocalToParent(r.force)
    u = (np.asarray(pi, float) - np.asarray(po, float))
    return float(A(f) @ (u / np.linalg.norm(u)))


def member_forces(w: dict, case: dict, members=ALL6) -> dict:
    """One static solve: axial force per member for one load case."""
    P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
    s = _system()
    ground = _body(s, fixed=True)
    up = _body(s)
    direct = w.get("actuation") == "direct"
    on_lca = direct and w.get("pushrod_attach") == "lower"
    pr_inner = P["spring_inner"] if direct else P["rocker_pushrod"]
    links = {}
    for m, (o, i) in ENDS.items():
        if on_lca and m in ("LF", "LR"):
            continue
        links[m] = (_distance(s, up, ground, P[o], P[i]), P[o], P[i])
    if on_lca:
        lca = _body(s)
        rev = ch.ChLinkLockRevolute()
        rev.Initialize(lca, ground, _frame_z_along(P["lower_front_inner"],
                                                   P["lower_rear_inner"] - P["lower_front_inner"]))
        sph = ch.ChLinkLockSpherical()
        sph.Initialize(up, lca, ch.ChFramed(V(P["lower_outer"])))
        s.Add(rev); s.Add(sph)                                                    # noqa: E702
        links["PR"] = (_distance(s, lca, ground, P["pushrod_outer"], pr_inner), P["pushrod_outer"], pr_inner)
    else:
        links["PR"] = (_distance(s, up, ground, P["pushrod_outer"], pr_inner), P["pushrod_outer"], pr_inner)
    i = up.AddAccumulator()
    up.AccumulateForce(i, V([case["Fx"], case["Fy"], case["Fz"]]), V(P["contact_patch"]), False)
    up.AccumulateForce(i, V([case["Fx_wc"], 0.0, 0.0]), V(P["wheel_center"]), False)
    up.AccumulateTorque(i, V([0.0, 0.0, case["Mz"]]), False)
    if not s.DoStaticLinear():
        raise RuntimeError("Chrono static solve failed")
    return {m: _tension(*links[m]) for m in members if m in links}


def calibrate():
    """Closed form first: a body held by six orthogonal links, 100 N down at the point the vertical link holds.
    The vertical link carries 100 N tension, the others nothing."""
    s = _system()
    g, b = _body(s, fixed=True), _body(s)
    o = np.zeros(3)
    spec = [(o, [0, 0, 100.0]), (o, [100.0, 0, 0]), (o, [0, 100.0, 0]),
            ([50.0, 0, 0], [50.0, 0, 100.0]), ([0, 50.0, 0], [0, 50.0, 100.0]), ([0, 50.0, 0], [100.0, 50.0, 0])]
    lks = [(_distance(s, b, g, np.asarray(p, float), np.asarray(q, float)), p, q) for p, q in spec]
    i = b.AddAccumulator()
    b.AccumulateForce(i, V([0, 0, -100.0]), V(o), False)
    assert s.DoStaticLinear()
    t = [_tension(*x) for x in lks]
    assert abs(t[0] - 100.0) < 1e-6 and max(abs(x) for x in t[1:]) < 1e-6, t


def damper_curve(w: dict) -> np.ndarray:
    """Damper length (mm) at TRAVEL of the wheel centre."""
    P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
    s = _system()
    ground = _body(s, fixed=True)
    lca, uca, up = _body(s), _body(s), _body(s)
    mot = ch.ChLinkMotorRotationAngle()
    mot.Initialize(lca, ground, _frame_z_along(P["lower_front_inner"], P["lower_rear_inner"] - P["lower_front_inner"]))
    fn = ch.ChFunctionConst(0.0)
    mot.SetAngleFunction(fn)
    rev = ch.ChLinkLockRevolute()
    rev.Initialize(uca, ground, _frame_z_along(P["upper_front_inner"], P["upper_rear_inner"] - P["upper_front_inner"]))
    s1, s2 = ch.ChLinkLockSpherical(), ch.ChLinkLockSpherical()
    s1.Initialize(up, lca, ch.ChFramed(V(P["lower_outer"])))
    s2.Initialize(up, uca, ch.ChFramed(V(P["upper_outer"])))
    for x in (mot, rev, s1, s2):
        s.Add(x)
    _distance(s, up, ground, P["tie_rod_outer"], P["tie_rod_inner"])
    on = up if w["pushrod_attach"] == "upright" else lca
    out_local, wc_local = P["pushrod_outer"].copy(), P["wheel_center"].copy()    # bodies start at the origin
    angles = np.radians(np.linspace(-7.0, 7.0, 281))
    order = np.argsort(np.abs(angles), kind="stable")
    res = {}
    for k in order:                                    # walk out from ride height in both directions
        if k == order[0] or np.sign(angles[k]) != np.sign(angles[order[list(order).index(k) - 1]]):
            for b in (lca, uca, up):
                b.SetPos(ch.ChVector3d(0, 0, 0)); b.SetRot(ch.ChQuaterniond(1, 0, 0, 0))  # noqa: E702
        fn.SetConstant(float(angles[k]))
        for _ in range(3):
            s.DoAssembly(ch.POSITION, 100, 1e-10, 1e-14, 1e-12)
        # closure: rigid arms keep their lengths, the tie rod its length, else the assembly did not converge
        for body, pt, piv in ((lca, "lower_outer", "lower_front_inner"), (uca, "upper_outer", "upper_front_inner"),
                              (up, "tie_rod_outer", "tie_rod_inner")):
            q = A(body.TransformPointLocalToParent(V(P[pt])))
            err = abs(np.linalg.norm(q - P[piv]) - np.linalg.norm(P[pt] - P[piv]))
            if err > 1e-6:
                raise RuntimeError(f"Chrono assembly did not close ({pt}: {err:.2e} mm)")
        wc = A(up.TransformPointLocalToParent(V(wc_local)))
        dout = A(on.TransformPointLocalToParent(V(out_local)))
        res[k] = (wc[2] - P["wheel_center"][2], float(np.linalg.norm(P["spring_inner"] - dout)))
    dz = np.array([res[k][0] for k in range(len(angles))])
    L = np.array([res[k][1] for k in range(len(angles))])
    srt = np.argsort(dz)
    if dz[srt][0] > -25 or dz[srt][-1] < 25:
        raise RuntimeError("sweep does not cover -25..+25 mm of wheel travel")
    return np.interp(TRAVEL, dz[srt], L[srt])


def main():
    calibrate()
    man = {n: json.loads((ROOT / "manifests" / f"{n}.recorded.genesis.json").read_text())["recorded"]
           ["winner_hardpoints"] for n in LL_CORNERS}
    rows = ["# chrono: Project Chrono " + ch.__name__ + " 9.0.1, scripts/chrono_benchmark.py. Axial force, N, + tension.",
            "corner,case," + ",".join(ALL6)]
    for n in LL_CORNERS:
        for c, d in CASES.items():
            f = member_forces(man[n], d)
            rows.append(f"{n},{c}," + ",".join(f"{f[m]:.6f}" for m in ALL6))
    (LL / "chrono_results.csv").write_text("\n".join(rows) + "\n")
    dl = ["# chrono: damper length (mm) at wheel-centre travel (mm), scripts/chrono_benchmark.py.",
          "corner,travel_mm,damper_mm"]
    dlo = ["# chrono: axial force, N, + tension; direct_lower: UF, UR, TR and the damper PR.",
           "corner,case," + ",".join(ALL6)]
    for n in DA_CORNERS:
        w = json.loads((DA / "corners" / f"{n}.json").read_text())
        L = damper_curve(w)
        dl += [f"{n},{t:g},{x:.9f}" for t, x in zip(TRAVEL, L)]
        for c, d in CASES.items():
            f = member_forces(w, d)
            dlo.append(f"{n},{c}," + ",".join(f"{f[m]:.6f}" if m in f else "" for m in ALL6))
    (DA / "chrono_damper.csv").write_text("\n".join(dl) + "\n")
    (DA / "chrono_loads.csv").write_text("\n".join(dlo) + "\n")
    print("wrote chrono_results.csv, chrono_damper.csv, chrono_loads.csv")


if __name__ == "__main__":
    sys.exit(main())
