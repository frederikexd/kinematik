# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Lateral load transfer of the final corners through the links, from forces rather than from the roll-centre point
(supplement S6.35).

For a corner whose wheel moves by dz (up, body held) the contact patch moves dy = q dz sideways (q = dy_cp/dz, the
scrub rate, corner frame: y outboard). By virtual work, of the vertical load Fz and lateral force Fy at the patch,
the spring path sees Fz + q Fy and the links carry the rest straight into the body: V = -q Fy (positive up). That
link-borne vertical force is the geometric (non-elastic) load transfer and its sum over an axle the jacking force; the
height at which the axle's lateral force acts on the body (the force-based roll centre) is the moment of the
link-borne forces about the ground centre point over the axle's lateral force. Only when both wheels share one scrub
rate does it equal the geometric roll centre (the CP-IC line intercept); in roll, or with unequal tire forces, the two
differ and the force-based one is what transfers load.

Checks built in (the physics is proved, not assumed): q from the contact-patch motion equals the slope of the
contact-patch-to-instant-centre line; the pushrod force from loadpath equals the virtual-work prediction
(Fz + q Fy + q_x Fx) / (dL_pushrod/dz); the link-borne vertical force from loadpath's member forces equals -q Fy.
Lengths mm, forces N, angles deg.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.kinematics import SuspensionKinematics                                  # noqa: E402
from suspension.genesis_repro import GenesisManifest, hp_from_dict                      # noqa: E402
from suspension import loadpath as lp, tire_utilisation as tu                           # noqa: E402

MAN = ROOT / "manifests"
TRACK = 1210.0
ROLL_GRAD = 0.78           # deg/g, the final corners (paper Table 3)
ROLL_SHARE = 0.522         # front roll-stiffness share of the final corners (paper section 3.1)


def hp_of(name: str):
    return hp_from_dict(GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())
                        .recorded["winner_hardpoints"])


def scrub_rate(kin, z: float, h: float = 0.05) -> tuple:
    """(q_y, q_x) = d(contact patch y, x)/d(travel) at travel z (dimensionless)."""
    a, b = kin.solve_at_travel(z + h), kin.solve_at_travel(z - h)
    d = (np.asarray(a.contact_patch) - np.asarray(b.contact_patch)) / (2 * h)
    dz = (a.contact_patch[2] - b.contact_patch[2]) / (2 * h)
    return float(d[1] / (1.0 + (dz - 1.0))), float(d[0])       # per mm of travel (travel = wheel-centre z)


def ic_slope(st) -> float:
    """Front-view slope of the contact-patch -> instant-centre line, |dz/dy|, the geometric construction."""
    cp, ic = np.asarray(st.contact_patch, float), np.asarray(st.instant_center, float)
    return float((ic[1] - cp[2]) / (cp[1] - ic[0]))


def pushrod_length(kin, z: float) -> float:
    st = kin.solve_at_travel(z)
    return float(np.linalg.norm(np.asarray(st.pushrod_outer) - np.asarray(kin.hp.rocker_pushrod)))


def proofs(name: str, Fy: float = -1800.0, Fz: float = 1200.0, Fx: float = 0.0) -> dict:
    """Virtual-work checks of kinematics against statics on one corner: the scrub rate from the contact patch's
    motion against the slope of the CP-IC line, and the link-borne vertical force from loadpath (the load less what
    the pushrod carries, T dL/dz in wheel terms) against -q Fy - q_x Fx."""
    hp = hp_of(name)
    kin = SuspensionKinematics(hp)
    st = kin.solve_at_travel(0.0)
    h = 0.05
    a, b = kin.solve_at_travel(h), kin.solve_at_travel(-h)
    v = (np.asarray(a.contact_patch) - np.asarray(b.contact_patch)) / (2 * h)        # material point, per mm of travel
    dLp = (pushrod_length(kin, h) - pushrod_length(kin, -h)) / (2 * h)
    mf = lp.solve_member_forces(kin, st, lp.WheelLoad(Fx=Fx, Fy=Fy, Fz=Fz))
    spring_wheel = mf.forces["PR"] * dLp                                              # wheel-equivalent spring load
    q_ic = ic_slope(st)
    return {"q_cp_motion": float(v[1] / v[2]), "q_ic_line": float(q_ic),
            "links_vertical_loadpath": float(Fz * v[2] - spring_wheel),
            "links_vertical_virtual_work": float(-(Fy * v[1] + Fx * v[0])),
            "pushrod_T": float(mf.forces["PR"]), "residual_N": float(mf.residual)}


def axle(name: str, fy_out: float, fz_out: float, fy_in: float, fz_in: float, roll_deg: float) -> dict:
    """Force-based roll centre, link-borne transfer and jacking of one axle. Forces: magnitudes toward the turn
    centre (fy) and vertical loads (fz), outer and inner; roll in deg (outer wheel in bump)."""
    hp = hp_of(name)
    kin = SuspensionKinematics(hp)
    z = 0.5 * TRACK * math.tan(math.radians(roll_deg))
    out = {"bump_mm": z}
    sides = {}
    for side, travel, fy, fz in (("outer", z, fy_out, fz_out), ("inner", -z, fy_in, fz_in)):
        st = kin.solve_at_travel(travel)
        a, b = kin.solve_at_travel(travel + 0.05), kin.solve_at_travel(travel - 0.05)
        dcp = (np.asarray(a.contact_patch) - np.asarray(b.contact_patch)) / 0.1
        q = float(dcp[1] / dcp[2])                       # d y_cp / d z_cp, corner frame (y outboard)
        Fy_c = -fy if side == "outer" else +fy           # corner frame: outer force inboard, inner force outboard
        V = -q * Fy_c                                    # link-borne vertical force on the body, + up
        sides[side] = {"q": q, "Fy_corner": Fy_c, "V_links": V, "cp_y": float(st.contact_patch[1]),
                       "rc_intercept_mm": float(st.contact_patch[2] + q * st.contact_patch[1])}
    # vehicle frame (left turn, outer = right at y_v = -cp_y, inner = left at +cp_y; z up; lateral + to the left)
    yo, yi = -sides["outer"]["cp_y"], sides["inner"]["cp_y"]
    Fl = fy_out + fy_in                                   # total lateral force toward the turn centre (+ left)
    Vo, Vi = sides["outer"]["V_links"], sides["inner"]["V_links"]
    # moment about the x axis through the ground centre point of the link-borne forces (lateral at ground, vertical)
    Mx = yo * Vo + yi * Vi
    h_frc = -Mx / Fl                                     # height at which the axle's lateral force acts on the body
    out.update(sides)
    out["force_based_rc_mm"] = h_frc
    out["kinematic_rc_mm"] = float(_kinematic_rc(kin, z))
    out["jacking_N"] = Vo + Vi
    out["geometric_transfer_N"] = Fl * h_frc / TRACK
    return out


def _kinematic_rc(kin, z: float) -> float:
    """Intersection height of the two sides' CP-IC lines in roll (the geometric roll centre)."""
    lines = []
    for travel, sgn in ((z, -1.0), (-z, +1.0)):           # outer (right) at -y, inner (left) at +y, vehicle frame
        st = kin.solve_at_travel(travel)
        cp, ic = np.asarray(st.contact_patch, float), np.asarray(st.instant_center, float)
        p0 = np.array([sgn * cp[1], cp[2]]); p1 = np.array([sgn * ic[0], ic[1]])
        lines.append((p0, p1 - p0))
    (a, da), (b, db) = lines
    M = np.array([da, -db]).T
    t = np.linalg.solve(M, b - a)
    return float((a + t[0] * da)[1])


def run(out_path: str | None = None, ay_g: float = 1.5) -> dict:
    s = tu.tire_state(0.0, ay_g)
    roll = ROLL_GRAD * ay_g
    out = {"ay_g": ay_g, "roll_deg": roll, "proofs": {}, "axles": {}}
    for ax, name, (o, i) in (("front", "front_v11_final", ("FR", "FL")), ("rear", "rear_v11_final", ("RR", "RL"))):
        out["proofs"][ax] = proofs(name)
        for state, r in (("static", 0.0), ("rolled", roll)):
            out["axles"][f"{ax} {state}"] = axle(name, abs(s[o]["Fy"]), s[o]["Fz"], abs(s[i]["Fy"]), s[i]["Fz"], r)
    m, g, h = 300.0, 9.81, 280.0
    total = m * g * ay_g * h / TRACK
    for state in ("static", "rolled"):
        f, r = out["axles"][f"front {state}"], out["axles"][f"rear {state}"]
        out[f"geometric_share_of_total_transfer_{state}"] = (f["geometric_transfer_N"] + r["geometric_transfer_N"]) / total
    out["total_lateral_transfer_N"] = total
    from suspension import target_derivation as td
    for state in ("static", "rolled"):                 # front share of lateral load transfer at the 52.2% stiffness share
        out[f"front_share_force_{state}"] = td.front_lltd_share(out["axles"][f"front {state}"]["force_based_rc_mm"],
                                                                 out["axles"][f"rear {state}"]["force_based_rc_mm"], ROLL_SHARE)
        out[f"front_share_geometric_{state}"] = td.front_lltd_share(out["axles"][f"front {state}"]["kinematic_rc_mm"],
                                                                     out["axles"][f"rear {state}"]["kinematic_rc_mm"], ROLL_SHARE)
    if out_path:
        Path(out_path).write_text(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    r = run(sys.argv[1] if len(sys.argv) > 1 else None)
    print(json.dumps(r, indent=1)[:4000])
