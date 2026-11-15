# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Link loads from an independent static solve: no KinematiK code, a different formulation.

KinematiK's loadpath.py is a force method: it writes the upright's six equilibrium equations in the six unknown member
forces and solves that 6x6 system. This file is a displacement method. Each member is an axial spring of its own
stiffness, the upright is a rigid body with six unknown displacements, and SciPy's fsolve finds the pose where the
springs balance the wheel load, with the full nonlinear geometry (rotation matrix, stretched lengths). The member force
is k times the change in length. For a statically determinate corner the result cannot depend on the stiffnesses, so
they are set unequal on purpose (1e7 to 6e7 N/mm). The movement is about 1e-4 mm, so the geometry stays the design
geometry.

Inputs are the raw manifest JSON (recorded winner hardpoints) and a load case: forces in N at the contact patch, a
shaft-reacted Fx_wc at the wheel centre, an aligning torque Mz in N*mm. Output: axial force per member, N, + tension.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.optimize import fsolve

MEMBERS = ("UF", "UR", "LF", "LR", "TR", "PR")
ENDS = {"UF": ("upper_outer", "upper_front_inner"), "UR": ("upper_outer", "upper_rear_inner"),
        "LF": ("lower_outer", "lower_front_inner"), "LR": ("lower_outer", "lower_rear_inner"),
        "TR": ("tie_rod_outer", "tie_rod_inner"), "PR": ("pushrod_outer", "rocker_pushrod")}
STIFFNESS = dict(zip(MEMBERS, (1e7, 2e7, 3e7, 4e7, 5e7, 6e7)))      # N/mm, unequal on purpose


def _rot(r):
    """Rotation matrix of the rotation vector r (rad), Rodrigues' formula."""
    th = float(np.linalg.norm(r))
    if th < 1e-15:
        return np.eye(3)
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def member_forces(hp: dict, case: dict) -> dict:
    """Axial force (N, + tension) in each member for one load case; hp is a manifest's winner_hardpoints."""
    if hp.get("pushrod_attach", "upright") != "upright":
        raise ValueError("this check covers the pushrod on the upright (six two-force members)")
    P = {k: np.asarray(v, float) for k, v in hp.items() if isinstance(v, list) and len(v) == 3}
    ref = P["wheel_center"]                               # the upright rotates about the wheel centre
    loads = [(P["contact_patch"], np.array([case["Fx"], case["Fy"], case["Fz"]], float)),
             (P["wheel_center"], np.array([case.get("Fx_wc", 0.0), 0.0, 0.0]))]
    torque = np.array([0.0, 0.0, case.get("Mz", 0.0)])
    L0 = {m: np.linalg.norm(P[i] - P[o]) for m, (o, i) in ENDS.items()}

    def tensions(q):
        R, t = _rot(q[3:] * 1e-4), q[:3] * 1e-4            # scaled unknowns: 0.1 micron and 0.1 milliradian
        out = {}
        for m, (o, i) in ENDS.items():
            po = ref + R @ (P[o] - ref) + t
            d = P[i] - po
            L = np.linalg.norm(d)
            out[m] = (STIFFNESS[m] * (L - L0[m]), d / L, po)  # stretched spring pulls the upright inboard
        return out, R, t

    def residual(q):
        tens, R, t = tensions(q)
        F = sum(T * u for T, u, _ in tens.values())
        M = sum(np.cross(po - ref, T * u) for T, u, po in tens.values())
        for p, f in loads:
            pp = ref + R @ (p - ref) + t
            F = F + f
            M = M + np.cross(pp - ref, f)
        M = M + torque
        return np.concatenate([F, M / 100.0])

    q, info, ier, msg = fsolve(residual, np.zeros(6), xtol=1e-13, full_output=True)
    scale = max(np.abs(np.r_[case["Fx"], case["Fy"], case["Fz"], case.get("Fx_wc", 0.0)]))
    if np.max(np.abs(residual(q))) > 1e-7 * scale:
        raise RuntimeError(f"independent solve did not converge: {msg}")
    tens, _, _ = tensions(q)
    return {m: float(tens[m][0]) for m in MEMBERS}


def from_manifest(path: Path, case: dict) -> dict:
    return member_forces(json.loads(Path(path).read_text())["recorded"]["winner_hardpoints"], case)
