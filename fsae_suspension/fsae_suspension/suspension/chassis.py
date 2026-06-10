"""
Chassis integration: check a suspension corner against the real chassis CAD.

This is the module that turns KinematiK from a kinematics toy into something the
team can actually gate manufacturing on. It answers the two questions that decide
whether you can cut tube and weld brackets:

    FIT       — do the inboard pickup points land on/near the chassis where a
                bracket can actually be mounted? (a pickup floating 40 mm off any
                tube is a packaging problem you want to find before fabrication)

    CLEARANCE — as the wheel moves through full bump/droop AND the steering sweeps
                lock to lock, does any moving link (wishbone, upright, tie rod,
                wheel/tire envelope) collide with or come dangerously close to the
                chassis tubes?

The clearance check is the important one. A static geometry can look perfectly
clear and still smash a lower wishbone into a frame tube at full bump — so we
sweep the linkage, build the swept volume of each link as a cloud of segments,
and query the minimum distance to the chassis mesh over the whole motion.

Chassis CAD comes in as STEP (converted to mesh via cascadio) or STL/OBJ/GLB
(loaded directly by trimesh). Everything is kept in the same mm SAE frame as the
kinematics module — the user is responsible for aligning the CAD origin to the
suspension origin, and we provide a manual offset to help.
"""

from __future__ import annotations

import os
import tempfile
import numpy as np
import trimesh

from .kinematics import SuspensionKinematics, Hardpoints


# --------------------------------------------------------------------------- #
#  Loading chassis geometry
# --------------------------------------------------------------------------- #
def load_chassis(path: str, offset=(0.0, 0.0, 0.0), scale=1.0) -> trimesh.Trimesh:
    """
    Load a chassis mesh from STEP / STL / OBJ / GLB. Returns a single Trimesh in
    the suspension coordinate frame after applying `scale` then `offset` (mm).

    STEP files are tessellated to a mesh via cascadio. Multi-body assemblies are
    concatenated into one mesh — we only need the surface for distance queries.
    """
    ext = os.path.splitext(path)[1].lower()

    if ext in (".step", ".stp"):
        import cascadio
        with tempfile.NamedTemporaryFile(suffix=".glb", delete=False) as tmp:
            glb_path = tmp.name
        cascadio.step_to_glb(path, glb_path, tol_linear=0.5, tol_angular=0.5)
        scene = trimesh.load(glb_path, force="scene")
        os.unlink(glb_path)
    else:
        scene = trimesh.load(path, force="scene")

    if isinstance(scene, trimesh.Scene):
        if len(scene.geometry) == 0:
            raise ValueError("No geometry found in chassis file.")
        mesh = trimesh.util.concatenate(
            [g for g in scene.geometry.values() if isinstance(g, trimesh.Trimesh)])
    else:
        mesh = scene

    if scale != 1.0:
        mesh.apply_scale(scale)
    mesh.apply_translation(np.asarray(offset, float))
    return mesh


def mesh_summary(mesh: trimesh.Trimesh) -> dict:
    lo, hi = mesh.bounds
    return {
        "triangles": int(len(mesh.faces)),
        "bbox_min": lo.tolist(),
        "bbox_max": hi.tolist(),
        "size_mm": (hi - lo).tolist(),
        "watertight": bool(mesh.is_watertight),
    }


# --------------------------------------------------------------------------- #
#  Swept linkage geometry
# --------------------------------------------------------------------------- #
def _link_segments(state, hp: Hardpoints):
    """
    Return the list of line segments (p, q) representing every moving link at one
    suspension state. These are the things that must not hit the chassis.
    """
    return [
        ("upper_wishbone_front", hp.upper_front_inner, state.upper_outer),
        ("upper_wishbone_rear",  hp.upper_rear_inner,  state.upper_outer),
        ("lower_wishbone_front", hp.lower_front_inner, state.lower_outer),
        ("lower_wishbone_rear",  hp.lower_rear_inner,  state.lower_outer),
        ("upright",              state.lower_outer,    state.upper_outer),
        ("tie_rod",              hp.tie_rod_inner,     state.tie_rod_outer),
        ("wheel_spindle",        state.lower_outer,    state.wheel_center),
    ]


def _sample_segment(p, q, n=12, skip_start_mm=0.0):
    """
    Discretise a segment into n points for distance querying. skip_start_mm drops
    samples within that distance of p (the inboard/mount end) so a link isn't
    flagged as colliding with the very tube it bolts to.
    """
    t = np.linspace(0, 1, n)[:, None]
    pts = p[None, :] * (1 - t) + q[None, :] * t
    if skip_start_mm > 0:
        d = np.linalg.norm(pts - p[None, :], axis=1)
        pts = pts[d >= skip_start_mm]
    return pts


def sweep_link_points(kin: SuspensionKinematics,
                      travel_min=-30.0, travel_max=30.0, n_travel=15,
                      samples_per_link=12, mount_exclude_mm=25.0):
    """
    Build the swept point cloud of every moving link across the travel range,
    tagged by link name. Returns (points Nx3, names list aligned to points).

    `mount_exclude_mm` drops samples near each link's inboard mount so a wishbone
    isn't reported as colliding with the chassis tube it actually bolts to. The
    outboard (upright) end is the moving end we care about for clearance.

    Steering sweep: the static tie-rod-inner is the rack position; vertical travel
    is swept here. A steering sweep can be layered by translating tie_rod_inner.
    """
    pts, names = [], []
    for tv in np.linspace(travel_min, travel_max, n_travel):
        st = kin.solve_at_travel(tv)
        for name, p, q in _link_segments(st, kin.hp):
            # exclude the mount end only for links that bolt to the chassis
            skip = mount_exclude_mm if "wishbone" in name or name == "tie_rod" else 0.0
            seg = _sample_segment(np.asarray(p, float), np.asarray(q, float),
                                  samples_per_link, skip_start_mm=skip)
            if len(seg) == 0:
                continue
            pts.append(seg)
            names.extend([name] * len(seg))
    return np.vstack(pts), names


# --------------------------------------------------------------------------- #
#  Fit check — do inboard pickups land on the chassis?
# --------------------------------------------------------------------------- #
INBOARD_POINTS = [
    ("upper_front_inner", "Upper wishbone front"),
    ("upper_rear_inner",  "Upper wishbone rear"),
    ("lower_front_inner", "Lower wishbone front"),
    ("lower_rear_inner",  "Lower wishbone rear"),
    ("tie_rod_inner",     "Tie rod inner (rack)"),
]


def fit_check(hp: Hardpoints, mesh: trimesh.Trimesh, tol_mm=10.0):
    """
    For each inboard pickup, distance to the nearest chassis surface. A pickup is
    'mountable' if it sits within tol_mm of a tube (close enough to weld a tab).
    Returns list of dicts with point name, distance, and pass/fail.
    """
    from trimesh.proximity import closest_point
    pts = np.array([getattr(hp, k) for k, _ in INBOARD_POINTS], float)
    closest, dist, _ = closest_point(mesh, pts)
    out = []
    for (key, label), d in zip(INBOARD_POINTS, dist):
        out.append({
            "point": key, "label": label,
            "distance_mm": float(d),
            "mountable": bool(d <= tol_mm),
        })
    return out


# --------------------------------------------------------------------------- #
#  Clearance check — does the moving linkage hit the chassis?
# --------------------------------------------------------------------------- #
def clearance_check(kin: SuspensionKinematics, mesh: trimesh.Trimesh,
                    travel_min=-30.0, travel_max=30.0, n_travel=15,
                    warn_mm=8.0):
    """
    Minimum distance from each moving link to the chassis surface, evaluated over
    the full travel sweep. Negative distance = penetration (the swept link passes
    inside the chassis surface = hard collision). Distance below warn_mm = too
    close, flag it.

    Returns dict: per-link min distance + worst offender + overall verdict.
    """
    from trimesh.proximity import ProximityQuery
    pq = ProximityQuery(mesh)

    pts, names = sweep_link_points(kin, travel_min, travel_max, n_travel)
    # signed_distance: + inside the mesh (collision), - outside (clear gap).
    # We invert so that + = clear gap in mm, - = penetration depth.
    signed = pq.signed_distance(pts)        # + inside solid
    clearance = -signed                     # + outside = real gap

    names_arr = np.array(names)
    per_link = {}
    for link in np.unique(names_arr):
        m = names_arr == link
        min_clear = float(np.min(clearance[m]))
        per_link[link] = {
            "min_clearance_mm": min_clear,
            "collision": bool(min_clear < 0),
            "warning": bool(0 <= min_clear < warn_mm),
        }

    worst = min(per_link.items(), key=lambda kv: kv[1]["min_clearance_mm"])
    any_collision = any(v["collision"] for v in per_link.values())
    any_warning = any(v["warning"] for v in per_link.values())
    verdict = ("COLLISION" if any_collision
               else "TIGHT" if any_warning
               else "CLEAR")
    return {
        "per_link": per_link,
        "worst_link": worst[0],
        "worst_clearance_mm": worst[1]["min_clearance_mm"],
        "verdict": verdict,
    }


# --------------------------------------------------------------------------- #
#  Manufacturing export — locked hardpoints for the fab team
# --------------------------------------------------------------------------- #
def manufacturing_sheet(hp: Hardpoints, kin: SuspensionKinematics) -> str:
    """
    A plain-text pickup schedule the fab team can work from: every hardpoint in
    mm, plus the derived A-arm lengths and the static alignment. CSV-ish so it
    drops into a spreadsheet or a drawing note.
    """
    lines = ["KinematiK manufacturing pickup schedule",
             "coordinates in mm, SAE axes (x rear, y right, z up)", ""]
    lines.append("point,x,y,z")
    for key, _ in INBOARD_POINTS:
        v = getattr(hp, key)
        lines.append(f"{key},{v[0]:.1f},{v[1]:.1f},{v[2]:.1f}")
    for key in ("upper_outer", "lower_outer", "tie_rod_outer"):
        v = getattr(hp, key)
        lines.append(f"{key},{v[0]:.1f},{v[1]:.1f},{v[2]:.1f}")
    lines += ["", "derived link lengths (mm)"]
    lines.append(f"upper_wishbone_front,{kin.L_upper_f:.1f}")
    lines.append(f"upper_wishbone_rear,{kin.L_upper_r:.1f}")
    lines.append(f"lower_wishbone_front,{kin.L_lower_f:.1f}")
    lines.append(f"lower_wishbone_rear,{kin.L_lower_r:.1f}")
    lines.append(f"upright_length,{kin.L_upright:.1f}")
    lines.append(f"tie_rod_length,{kin.L_tie:.1f}")
    s = kin.static
    lines += ["", "static alignment",
              f"camber_deg,{s.camber:.2f}", f"toe_deg,{s.toe:.2f}",
              f"caster_deg,{s.caster:.2f}", f"kpi_deg,{s.kpi:.2f}"]
    return "\n".join(lines)
