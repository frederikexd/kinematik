# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Frame torsion and pickup stiffness derived from the ChassisV10 tubes, and the compliance of the final corners on
them (supplement S5.2 and S14).

The frame is the beam model of suspension/frame_fe.py on the tube axes recovered from ChassisV10.STEP
(data/chassisv10_tube_axes.json). ChassisV11 does not exist yet; its brief puts every inboard pickup on a node, so
each pickup of the final corners (and each rocker pivot and spring mount) is modelled as a node joined by
``BAY_TUBES`` tubes of ``BAY_SECTION`` to the ChassisV10 junctions that triangulate it best. On that frame:

torsional_rate():      rear pickups held, a couple on the front pickups: N*m/deg (and ChassisV10 alone, between
                       its axle stations, for comparison).
corner_flexibility():  the 15 x 15 flexibility of one corner's five pickups, corner frame (mm/N), with the frame
                       free and every load balanced by the car's inertia (inertia relief), measured from the body's
                       own rigid motion; the StiffnessField(coupled=...) the elastokinematic solve and the
                       compliance wall take.
full_vehicle():        every corner's link, pushrod and rocker forces on the frame at once, balanced by inertia;
                       each corner's camber, toe, caster and KPI change from its pickups' motion. This includes the
                       frame twist between rack and pickups and the other corners' loads.
Lengths mm, forces N, stiffness N/mm, angles deg.
"""
from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import frame_fe as ff                                                   # noqa: E402
from suspension import elastokinematics as ek, loadpath as lp, tire_utilisation as tu   # noqa: E402
from suspension.kinematics import SuspensionKinematics                                  # noqa: E402
from suspension.genesis_repro import GenesisManifest, hp_from_dict                      # noqa: E402

TUBES = ROOT / "data" / "chassisv10_tube_axes.json"
MAN = ROOT / "manifests"
CORNERS = {"front": "front_v11_final", "rear": "rear_v11_final"}
AXLE_Z = {"front": 950.0, "rear": -680.0}            # CAD Z of the axle stations (mm)
GROUND_Y = -76.0                                     # CAD Y of the ground plane (mm), node brief
PTS = list(ek.PICKUPS)
ACT = ["rocker_pivot", "spring_inner"]
BAY_TUBES = 4                                        # tubes per pickup node
BAY_SECTION = ff.Section(25.4, 1.65)                 # 1 x 0.065 in, the frame's main size
SPRUNG_KG = 300.0 - 4 * 11.9                         # car less four 11.9 kg unsprung corners
CG_CAD = (0.0, 280.0 + GROUND_Y, AXLE_Z["rear"] + 0.48 * 1630.0)   # 280 mm high, 48% front
OUTER = {"upper_front_inner": "upper_outer", "upper_rear_inner": "upper_outer", "lower_front_inner": "lower_outer",
         "lower_rear_inner": "lower_outer", "tie_rod_inner": "tie_rod_outer"}


#: hardpoints declared before their manifest is run (axle -> dict), e.g. by final_corners_v2 while it builds the
#: compliance wall that needs this flexibility
HP_OVERRIDE: dict = {}


def hp_of(axle: str):
    if axle in HP_OVERRIDE:
        return hp_from_dict(HP_OVERRIDE[axle])
    return hp_from_dict(GenesisManifest.from_json((MAN / f"{CORNERS[axle]}.recorded.genesis.json").read_text())
                        .recorded["winner_hardpoints"])


def to_cad(p, axle: str, side: int) -> np.ndarray:
    """Corner point (x rearward of the axle, y outboard, z up from ground) to CAD; side +1 right, -1 left."""
    x, y, z = p
    return np.array([side * y, z + GROUND_Y, AXLE_Z[axle] - x])


def cad_vec(v, side: int) -> np.ndarray:
    x, y, z = v
    return np.array([side * y, z, -x])


def corner_vec(v, side: int) -> np.ndarray:
    X, Y, Z = v
    return np.array([-Z, side * X, Y])


def chassis_v10() -> tuple:
    d = json.loads(TUBES.read_text())
    axes = [(np.array(t["p0"]), np.array(t["p1"]), t["wall_mm"]) for t in d["tubes"]]
    return ff.from_step_axes(axes, [t["od_mm"] for t in d["tubes"]])


def junctions(fr) -> list:
    deg: dict = {}
    for i, j, _, _ in fr.elems:
        deg[i] = deg.get(i, 0) + 1
        deg[j] = deg.get(j, 0) + 1
    return sorted(k for k, v in deg.items() if v >= 3)


BAY_MAX_MM = 400.0                                   # longest bay tube


def best_bay(fr, P, cand, n: int = BAY_TUBES, near: int = 14, rmax: float = BAY_MAX_MM):
    """The n junctions (of the ``near`` nearest within ``rmax`` mm) that triangulate P best: largest smallest
    singular value of the unit directions per mean tube length."""
    ks = sorted((k for k in cand if float(np.linalg.norm(fr.nodes[k] - P)) <= rmax),
                key=lambda k: float(np.linalg.norm(fr.nodes[k] - P)))[:near]
    best = None
    for combo in itertools.combinations(ks, n):
        U = np.array([(fr.nodes[k] - P) / np.linalg.norm(fr.nodes[k] - P) for k in combo])
        L = float(np.mean([np.linalg.norm(fr.nodes[k] - P) for k in combo]))
        sv = float(np.linalg.svd(U, compute_uv=False).min())
        if best is None or sv / L > best[0]:
            best = (sv / L, combo, sv, L)
    return best


def chassis_v11(n: int = BAY_TUBES, sec: ff.Section = BAY_SECTION) -> tuple:
    """ChassisV10 with a node bay at every pickup of the final corners, both sides. Returns (frame, nodes, report)."""
    fr, rep = chassis_v10()
    J = junctions(fr)
    nodes, bays = {}, {}
    for axle in ("front", "rear"):
        hp = hp_of(axle)
        for side in (+1, -1):
            for pt in PTS + ACT:
                P = to_cad(getattr(hp, pt), axle, side)
                b = best_bay(fr, P, J, n)
                nodes[(axle, side, pt)] = fr.attach(P, list(b[1]), sec)
                bays[f"{axle} {'right' if side > 0 else 'left'} {pt}"] = {"tube_lengths_mm": [float(np.linalg.norm(fr.nodes[k] - P)) for k in b[1]],
                                                                          "smallest_singular_value": b[2]}
    rep["bays"] = bays
    return fr, nodes, rep


def mass_model(fr) -> np.ndarray:
    own = fr.lumped_mass(0.0).sum()
    return fr.lumped_mass(SPRUNG_KG - own, cg=CG_CAD)


def torsional_rate(fr, nodes, force_n: float = 1000.0) -> float:
    """Rear pickups held, front pickups pushed up on the right and down on the left; N*m/deg."""
    fixed = {k: range(6) for key, k in nodes.items() if key[0] == "rear"}
    front = [key for key in nodes if key[0] == "front"]
    u = fr.solve({nodes[key]: [0, key[1] * force_n, 0, 0, 0, 0] for key in front}, fixed)
    X = np.array([fr.nodes[nodes[k]][0] for k in front])
    Y = np.array([u[nodes[k], 1] for k in front])
    T = sum(k[1] * force_n * fr.nodes[nodes[k]][0] for k in front)            # N*mm
    return T / 1000.0 / np.degrees(np.polyfit(X, Y, 1)[0])


def torsional_rate_v10(fr, half_mm: float = 100.0, force_n: float = 100.0) -> float:
    """ChassisV10 alone: nodes within ``half_mm`` of the rear axle station held, those near the front one twisted."""
    rear = [k for k, p in enumerate(fr.nodes) if abs(p[2] - AXLE_Z["rear"]) <= half_mm]
    front = [k for k, p in enumerate(fr.nodes) if abs(p[2] - AXLE_Z["front"]) <= half_mm and abs(p[0]) > 50]
    u = fr.solve({k: [0, np.sign(fr.nodes[k][0]) * force_n, 0, 0, 0, 0] for k in front}, {k: range(6) for k in rear})
    X = np.array([fr.nodes[k][0] for k in front]); Y = np.array([u[k, 1] for k in front])
    T = sum(abs(fr.nodes[k][0]) * force_n for k in front)
    return T / 1000.0 / np.degrees(np.polyfit(X, Y, 1)[0])


def corner_flexibility(fr, nodes, mass, axle: str, side: int = +1) -> np.ndarray:
    """15 x 15 flexibility of the corner's five pickups in its own corner frame (mm/N), inertia relief."""
    C = fr.inertia_relief_flexibility([nodes[(axle, side, p)] for p in PTS], mass)
    Rc = np.array([corner_vec(e, side) for e in np.eye(3)]).T          # corner <- CAD
    out = np.zeros_like(C)
    for a in range(5):
        for b in range(5):
            out[3 * a:3 * a + 3, 3 * b:3 * b + 3] = Rc @ C[3 * a:3 * a + 3, 3 * b:3 * b + 3] @ Rc.T
    return 0.5 * (out + out.T)


def axial_stiffness(C: np.ndarray, hp) -> dict:
    """Stiffness along each link at its own pickup, others free (kN/mm)."""
    out = {}
    for i, p in enumerate(PTS):
        u = np.asarray(getattr(hp, OUTER[p]), float) - np.asarray(getattr(hp, p), float)
        u /= np.linalg.norm(u)
        out[p] = 1.0 / float(u @ C[3 * i:3 * i + 3, 3 * i:3 * i + 3] @ u) / 1000.0
    return out


# ------------------------------------------------------------------ the whole car
def wheel_loads(ax_g: float, ay_g: float, bias: float = 0.73, mz_per_fy: float = 50000.0 / 1902.25) -> tuple:
    """Each tire's load in its own corner frame from tire_utilisation's quasi-steady allocation (ay > 0 a left turn).

    The ground force on each tire points to the turn centre: inboard on the right (outer) wheels, outboard on the
    left; the aligning torque is the pneumatic-trail moment Mz = t * Fy, the trail behind the contact centre,
    50 N*m at the 1902 N of the declared case. Front brakes act at the contact patch, rear brakes and drive at the
    wheel centre (inboard)."""
    s = tu.tire_state(ax_g, ay_g, brake_front=bias)
    out = {}
    for t in tu.TIRES:
        axle = "front" if t[0] == "F" else "rear"
        side = +1 if t[1] == "R" else -1
        Fy = -side * s[t]["Fy"]
        Fx = -s[t]["Fx"]
        kw = dict(Fy=Fy, Fz=s[t]["Fz"], Mz=mz_per_fy * Fy)
        out[t] = lp.WheelLoad(Fx_wc=Fx, **kw) if axle == "rear" else lp.WheelLoad(Fx=Fx, **kw)
    return out, bool(s["feasible"])


def rocker_reactions(hp, mf) -> dict:
    """Forces (and the pivot's moment) the rocker puts on the frame, corner frame (N, N*mm)."""
    Fp = -mf.forces["PR"] * np.asarray(mf.axes["PR"], float)                 # pushrod on rocker
    piv = np.asarray(hp.rocker_pivot, float)
    a = np.asarray(hp.rocker_axis, float); a /= np.linalg.norm(a)
    rp, rs, si = (np.asarray(getattr(hp, k), float) for k in ("rocker_pushrod", "rocker_spring", "spring_inner"))
    e = (si - rs) / np.linalg.norm(si - rs)                                   # spring force on rocker = S e
    S = -(a @ np.cross(rp - piv, Fp)) / (a @ np.cross(rs - piv, e))
    Fs = S * e
    Fb, Mb = -(Fp + Fs), -(np.cross(rp - piv, Fp) + np.cross(rs - piv, Fs))   # bearing on rocker
    return {"rocker_pivot": (-Fb, -Mb), "spring_inner": (-Fs, np.zeros(3)), "spring_N": S}


def frame_loads(fr, nodes, loads: dict) -> tuple:
    f, forces = {}, {}
    for t, L in loads.items():
        axle = "front" if t[0] == "F" else "rear"
        side = +1 if t[1] == "R" else -1
        hp = hp_of(axle)
        kin = SuspensionKinematics(hp)
        mf = lp.solve_member_forces(kin, kin.solve_at_travel(0.0), L)
        forces[t] = {m: float(v) for m, v in mf.forces.items()}
        for m, p in ek.MEMBER_PICKUP.items():
            v = f.setdefault(nodes[(axle, side, p)], np.zeros(6))
            fp = (np.asarray(mf.pickup_forces[p], float) if mf.pickup_forces
                  else -mf.forces[m] * np.asarray(mf.axes[m], float))
            v[:3] += cad_vec(fp, side)
        for p, (Fv, Mv) in ((k, v) for k, v in rocker_reactions(hp, mf).items() if k != "spring_N"):
            v = f.setdefault(nodes[(axle, side, p)], np.zeros(6))
            v[:3] += cad_vec(Fv, side)
            v[3:] += cad_vec(Mv, side) * side             # an axial vector picks up the mirror's determinant
    return f, forces


def solve_free(fr, mass, f: dict) -> tuple:
    """Free-free solve: loads balanced by the inertia of ``mass``; displacements from the body's rigid motion."""
    n = 6 * len(fr.nodes)
    K = fr.stiffness()
    Md = np.repeat(np.c_[mass, mass, mass, np.zeros((len(mass), 3))].ravel()[None, :], 1, 0).ravel()
    R = fr.rigid_modes(np.average(np.array(fr.nodes), axis=0, weights=mass))
    A = R.T @ (Md[:, None] * R)
    Ai = np.linalg.inv(A)
    F = np.zeros(n)
    for k, v in f.items():
        F[6 * k:6 * k + 6] += v
    res = R.T @ F
    Fb = F - Md * (R @ (Ai @ res))
    X = np.array(fr.nodes)
    a, b = int(np.argmin(X[:, 2])), int(np.argmax(X[:, 2]))
    c = int(np.argmax(np.abs(np.cross(X - X[a], X[b] - X[a])).sum(1)))
    fixed = [6 * a, 6 * a + 1, 6 * a + 2, 6 * b, 6 * b + 1, 6 * c + 1]
    free = np.setdiff1d(np.arange(n), fixed)
    U = np.zeros(n)
    U[free] = np.linalg.solve(K[np.ix_(free, free)], Fb[free])
    reaction = float(np.abs((K @ U - Fb)[fixed]).max())
    Ue = U - R @ (Ai @ (R.T @ (Md * U)))
    return Ue.reshape(-1, 6), res, reaction


def full_vehicle(fr, nodes, mass, ax_g: float, ay_g: float, jac: dict, bias: float = 0.73) -> dict:
    loads, feasible = wheel_loads(ax_g, ay_g, bias)
    f, forces = frame_loads(fr, nodes, loads)
    U, res, reaction = solve_free(fr, mass, f)
    out = {"feasible": feasible, "resultant_N": res[:3].tolist(), "resultant_Nmm": res[3:].tolist(),
           "max_support_reaction_N": reaction, "corners": {}}
    for t in loads:
        axle = "front" if t[0] == "F" else "rear"
        side = +1 if t[1] == "R" else -1
        d = np.concatenate([corner_vec(U[nodes[(axle, side, p)], :3], side) for p in PTS])
        out["corners"][t] = {"channels": dict(zip(ek.CHANNELS, map(float, jac[axle] @ d))),
                             "pickup_mm": {p: d[3 * i:3 * i + 3].tolist() for i, p in enumerate(PTS)},
                             "member_N": forces[t]}
    return out


def run(out_path: str | None = None) -> dict:
    fr0, rep0 = chassis_v10()
    fr, nodes, rep = chassis_v11()
    K = fr.stiffness()
    w = np.linalg.eigvalsh(K)
    mass = mass_model(fr)
    out = {"chassis_v10": {k: v for k, v in rep0.items()}, "bays": rep["bays"],
           "rigid_body_modes": int(np.sum(np.abs(w) < 1e-3)), "first_elastic_eig": float(w[6]),
           "mass_kg": float(mass.sum()), "torsion": {}, "corners": {}, "full_vehicle": {}}
    out["torsion"]["v11_bays_Nm_per_deg"] = torsional_rate(fr, nodes)
    out["torsion"]["v10_alone_Nm_per_deg"] = {str(h): torsional_rate_v10(fr0, h) for h in (100.0, 150.0)}
    jac = {a: ek.compliance_jacobian(hp_of(a)) for a in ("front", "rear")}
    for axle in ("front", "rear"):
        C = corner_flexibility(fr, nodes, mass, axle)
        out["corners"][axle] = {"flexibility_mm_per_N": C.tolist(), "axial_kN_per_mm": axial_stiffness(C, hp_of(axle)),
                                "symmetry_error": float(np.abs(C - C.T).max() / np.abs(C).max())}
    for name, (ax, ay) in (("1.5g corner", (0.0, 1.5)), ("1.5g braking", (1.5, 0.0)),
                           ("combined 1.06g", (1.06, 1.06)), ("combined 1.06g traction", (-1.06, 1.06))):
        out["full_vehicle"][name] = full_vehicle(fr, nodes, mass, ax, ay, jac)
    if out_path:
        Path(out_path).write_text(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    r = run(sys.argv[1] if len(sys.argv) > 1 else None)
    print("rigid modes", r["rigid_body_modes"], "K_T", round(r["torsion"]["v11_bays_Nm_per_deg"]), r["torsion"]["v10_alone_Nm_per_deg"])
    for a, c in r["corners"].items():
        print(a, {k[:12]: round(v, 1) for k, v in c["axial_kN_per_mm"].items()})
    for n, c in r["full_vehicle"].items():
        print(n, c["feasible"], np.round(c["resultant_N"], 1), {t: (round(v["channels"]["camber"], 3), round(v["channels"]["toe"], 3))
                                                              for t, v in c["corners"].items()})
