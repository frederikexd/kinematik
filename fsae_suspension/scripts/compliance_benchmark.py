# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Compliance steer and camber against two independent solvers, which are themselves checked against Ansys VM4.

KinematiK's CompliantCorner (suspension/compliance.py) deflects each link by its axial force over its stiffness and
re-solves the corner. Here the same corner, links and loads go to:

  independent  a rigid upright on six axial springs, solved for its six displacements with the full nonlinear geometry
               (SciPy fsolve); no KinematiK code. The damper is a rigid member.
  opensees     OpenSeesPy 3.8: truss elements for the links, a rigid upright (rigidLink beam constraints to the wheel
               centre), a stiff truss for the damper, linear static.

Both references first reproduce Ansys Mechanical APDL's published VM4 (Deflection of a Hinged Support): bar stress
10,000 psi and joint deflection -0.120 in, Mechanical APDL ratio 1.000. That checks their truss model, not KinematiK.

Corner, stiffnesses, loads and limits: benchmarks/compliance/limits.json, registered before KinematiK was run.
Results: benchmarks/compliance/results.json.

    python scripts/compliance_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import fsolve

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
BENCH = ROOT / "benchmarks" / "compliance"
REG = json.loads((BENCH / "limits.json").read_text())
CORNER = json.loads((ROOT / "benchmarks" / "direct_acting" / "corners" / "direct_upright.json").read_text())
CASES = json.loads((ROOT / "benchmarks" / "link_loads" / "limits.json").read_text())["load_cases"]
K = REG["link_stiffness_N_per_mm"]
RIGID = 1e9                                            # N/mm, the damper in OpenSees
ENDS = {"UF": ("upper_outer", "upper_front_inner"), "UR": ("upper_outer", "upper_rear_inner"),
        "LF": ("lower_outer", "lower_front_inner"), "LR": ("lower_outer", "lower_rear_inner"),
        "TR": ("tie_rod_outer", "tie_rod_inner"), "PR": ("pushrod_outer", "spring_inner")}


# --------------------------------------------------------------------------- angles, written here, not imported
def _angles(spin):
    """(camber, toe) in deg of a spin axis pointing outboard: camber negative when the top leans inboard, toe + out."""
    s = spin / np.linalg.norm(spin)
    return -np.degrees(np.arctan2(s[2], abs(s[1]))), np.degrees(np.arctan2(s[0], abs(s[1])))


def _static_spin(camber, toe):
    c, t = np.radians(camber), np.radians(toe)
    return np.array([np.cos(c) * np.sin(t), np.cos(c) * np.cos(t), -np.sin(c)])


def _rot(r):
    th = float(np.linalg.norm(r))
    if th < 1e-15:
        return np.eye(3)
    k = r / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * Kx + (1 - np.cos(th)) * Kx @ Kx


def _points():
    return {k: np.asarray(v, float) for k, v in CORNER.items() if isinstance(v, list) and len(v) == 3}


def _result(R, t, P, ref):
    R, t = np.asarray(R, float), np.asarray(t, float)
    s0 = _static_spin(CORNER["static_camber"], CORNER["static_toe"])
    c0, t0 = _angles(s0)
    c1, t1 = _angles(R @ s0)
    cp = ref + R @ (P["contact_patch"] - ref) + t
    return {"compliance_toe_deg": float(t1 - t0), "compliance_camber_deg": float(c1 - c0),
            "contact_patch_lateral_mm": float(cp[1] - P["contact_patch"][1])}


# --------------------------------------------------------------------------- reference 1: nonlinear springs
def independent(case: dict) -> dict:
    """Six displacements of the rigid upright plus the damper force, with the damper held at its length exactly
    (a constraint, not a stiff spring, so the system stays well conditioned)."""
    P = _points()
    ref = P["wheel_center"]
    L0 = {m: np.linalg.norm(P[i] - P[o]) for m, (o, i) in ENDS.items()}
    loads = [(P["contact_patch"], np.array([case["Fx"], case["Fy"], case["Fz"]], float)),
             (P["wheel_center"], np.array([case.get("Fx_wc", 0.0), 0.0, 0.0]))]
    torque = np.array([0.0, 0.0, case.get("Mz", 0.0)])
    scale = max(np.abs(np.r_[case["Fx"], case["Fy"], case["Fz"], case.get("Fx_wc", 0.0)]))

    def residual(q):
        R, t, lam = _rot(q[3:6] * 1e-4), q[:3] * 1e-2, q[6] * scale
        F, M, gap = np.zeros(3), np.zeros(3), 0.0
        for m, (o, i) in ENDS.items():
            po = ref + R @ (P[o] - ref) + t
            d = P[i] - po
            L = np.linalg.norm(d)
            if m == "PR":
                f, gap = lam * d / L, (L - L0[m]) * 100.0
            else:
                f = K[m] * (L - L0[m]) * d / L                 # a stretched link pulls the upright inboard
            F, M = F + f, M + np.cross(po - ref, f)
        for pt, f in loads:
            F, M = F + f, M + np.cross(ref + R @ (pt - ref) + t - ref, f)
        return np.concatenate([F / scale, (M + torque) / (100.0 * scale), [gap]])

    q, _, ier, msg = fsolve(residual, np.zeros(7), xtol=1e-14, full_output=True)
    if np.max(np.abs(residual(q))) > 1e-9:
        raise RuntimeError(f"independent compliance solve did not converge: {msg}")
    return _result(_rot(q[3:6] * 1e-4), q[:3] * 1e-2, P, ref)


# --------------------------------------------------------------------------- reference 2: OpenSees
def opensees(case: dict) -> dict:
    import openseespy.opensees as ops
    P = _points()
    ops.wipe()
    ops.model("basic", "-ndm", 3, "-ndf", 6)
    names = ["wheel_center", "contact_patch"] + sorted({o for o, _ in ENDS.values()})
    tag = {n: i + 1 for i, n in enumerate(names)}
    for n in names:
        ops.node(tag[n], *P[n])
    for j, (m, (o, i)) in enumerate(ENDS.items()):
        it = 100 + j
        ops.node(it, *P[i])
        ops.fix(it, 1, 1, 1, 1, 1, 1)
        L = np.linalg.norm(P[i] - P[o])
        ops.uniaxialMaterial("Elastic", it, 1.0)
        ops.element("Truss", it, it, tag[o], (K.get(m, RIGID)) * L, it)   # E = 1, A = k L, so EA/L = k
    for n in names[1:]:
        ops.rigidLink("beam", tag["wheel_center"], tag[n])
    ops.timeSeries("Constant", 1)
    ops.pattern("Plain", 1, 1)
    ops.load(tag["contact_patch"], case["Fx"], case["Fy"], case["Fz"], 0.0, 0.0, case.get("Mz", 0.0))
    ops.load(tag["wheel_center"], case.get("Fx_wc", 0.0), 0.0, 0.0, 0.0, 0.0, 0.0)
    ops.constraints("Transformation")
    ops.numberer("RCM")
    ops.system("FullGeneral")
    ops.test("NormDispIncr", 1e-12, 10)
    ops.algorithm("Linear")
    ops.integrator("LoadControl", 1.0)
    ops.analysis("Static")
    if ops.analyze(1) != 0:
        raise RuntimeError("OpenSees analysis failed")
    u = np.array(ops.nodeDisp(tag["wheel_center"]))
    ops.wipe()
    return _result(_rot(u[3:]), u[:3], P, P["wheel_center"])


# --------------------------------------------------------------------------- KinematiK
def kinematik(case: dict) -> dict:
    from suspension import loadpath as lp
    from suspension.compliance import CompliantCorner, MemberStiffness
    from suspension.genesis_repro import hp_from_dict
    cc = CompliantCorner(hp_from_dict(CORNER), {m: MemberStiffness(k_direct=v) for m, v in K.items()})
    r = cc.solve(lp.WheelLoad(Fx=case["Fx"], Fy=case["Fy"], Fz=case["Fz"], Mz=case.get("Mz", 0.0),
                              Fx_wc=case.get("Fx_wc", 0.0)))
    return {"compliance_toe_deg": float(r.compliance_toe), "compliance_camber_deg": float(r.compliance_camber),
            "contact_patch_lateral_mm": float(r.contact_patch_lateral_shift_mm), "converged": bool(r.converged)}


# --------------------------------------------------------------------------- Ansys VM4 on both references
def vm4() -> dict:
    d = REG["ansys_vm4"]["data"]
    L, th, A, E, F = d["L_in"], np.radians(d["theta_deg"]), d["A_in2"], d["E_psi"], d["F_lb"]
    a, b = L * np.cos(th), L * np.sin(th)
    sup = [np.array([0.0, 0.0]), np.array([2 * a, 0.0])]
    joint = np.array([a, -b])

    # direct stiffness: each bar adds (EA/L) n n^T at the joint, n the unit vector from its support to the joint
    Kj = np.zeros((2, 2))
    for sp in sup:
        n = (joint - sp) / np.linalg.norm(joint - sp)
        Kj += E * A / L * np.outer(n, n)
    u = np.linalg.solve(Kj, np.array([0.0, -F]))
    n0 = (joint - sup[0]) / np.linalg.norm(joint - sup[0])
    ind = {"stress_psi": float(E * (n0 @ u) / L), "deflection_in": float(u[1])}   # + tension

    import openseespy.opensees as ops
    ops.wipe()
    ops.model("basic", "-ndm", 2, "-ndf", 2)
    ops.node(1, 0.0, 0.0), ops.node(2, a, -b), ops.node(3, 2 * a, 0.0)
    ops.fix(1, 1, 1), ops.fix(3, 1, 1)
    ops.uniaxialMaterial("Elastic", 1, E)
    ops.element("Truss", 1, 1, 2, A, 1), ops.element("Truss", 2, 2, 3, A, 1)
    ops.timeSeries("Constant", 1), ops.pattern("Plain", 1, 1), ops.load(2, 0.0, -F)
    ops.system("BandSPD"), ops.numberer("RCM"), ops.constraints("Plain")
    ops.integrator("LoadControl", 1.0), ops.algorithm("Linear"), ops.analysis("Static"), ops.analyze(1)
    os_ = {"stress_psi": float(ops.eleResponse(1, "axialForce")[0] / A), "deflection_in": float(ops.nodeDisp(2, 2))}
    ops.wipe()
    pub, lim = REG["ansys_vm4"]["published"], REG["ansys_vm4"]["limit_relative"]
    out = {"published": pub}
    for name, r in (("independent", ind), ("opensees", os_)):
        out[name] = {**r, "passes": bool(all(abs(r[q] / pub[q] - 1) <= lim for q in ("stress_psi", "deflection_in")))}
    return out


def _judge(k: float, ref: float, lim: dict) -> bool:
    return abs(k - ref) <= lim["absolute"] or abs(k - ref) <= lim["relative"] * abs(ref)


def run() -> dict:
    lims = REG["limits"]
    out = {"vm4": vm4(), "cases": {}}
    ok = out["vm4"]["independent"]["passes"] and out["vm4"]["opensees"]["passes"]
    for c, d in CASES.items():
        kk, ind, os_ = kinematik(d), independent(d), opensees(d)
        row = {"kinematik": kk, "independent": ind, "opensees": os_, "passes": {}}
        for q in ("compliance_toe_deg", "compliance_camber_deg", "contact_patch_lateral_mm"):
            lim = lims["contact_patch_lateral_mm"] if q.endswith("_mm") else lims["angle_deg"]
            row["passes"][q] = bool(_judge(kk[q], ind[q], lim) and _judge(kk[q], os_[q], lim))
            ok = ok and row["passes"][q]
        out["cases"][c] = row
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    v = res["vm4"]
    print(f"VM4 published 10000 psi / -0.120 in: independent {v['independent']['stress_psi']:.1f} psi "
          f"{v['independent']['deflection_in']:.5f} in; OpenSees {v['opensees']['stress_psi']:.1f} psi "
          f"{v['opensees']['deflection_in']:.5f} in")
    for c, r in res["cases"].items():
        k, i, o = r["kinematik"], r["independent"], r["opensees"]
        print(f"{c:13s} toe K {k['compliance_toe_deg']:+.5f} I {i['compliance_toe_deg']:+.5f} O {o['compliance_toe_deg']:+.5f} | "
              f"camber K {k['compliance_camber_deg']:+.5f} I {i['compliance_camber_deg']:+.5f} O {o['compliance_camber_deg']:+.5f} | "
              f"cp_y K {k['contact_patch_lateral_mm']:+.4f} I {i['contact_patch_lateral_mm']:+.4f} O {o['contact_patch_lateral_mm']:+.4f} "
              f"{'PASS' if all(r['passes'].values()) else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
