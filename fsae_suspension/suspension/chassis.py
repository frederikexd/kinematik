# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Chassis integration: check any physical subsystem against the real chassis CAD.

This is the module that turns KinematiK from a kinematics toy into something the
team can actually gate manufacturing on. For the SUSPENSION corner — which moves —
it answers the two questions that decide whether you can cut tube and weld brackets:

    FIT       — do the inboard pickup points land on/near the chassis where a
                bracket can actually be mounted? (a pickup floating 40 mm off any
                tube is a packaging problem you want to find before fabrication)

    CLEARANCE — as the wheel moves through full bump/droop AND the steering sweeps
                lock to lock, does any moving link (wishbone, upright, tie rod,
                wheel/tire envelope) collide with or come dangerously close to the
                chassis tubes?

For every OTHER physical subsystem (powertrain, cooling, aero, brakes, electrics),
which is statically mounted rather than articulating, `envelope_fit_check` answers
the static version: does the part's bounding box sit inside the chassis interior and
clear the frame tubes where it's placed? (Data-acquisition is excluded — it's
wiring/loggers with no meaningful rigid envelope.) This is spatial fit only; it does
not simulate any subsystem, consistent with the rest of the integration layer.

The suspension clearance check is the subtle one. A static geometry can look
perfectly clear and still smash a lower wishbone into a frame tube at full bump — so
we sweep the linkage, build the swept volume of each link as a cloud of segments,
and query the minimum distance to the chassis mesh over the whole motion.

Chassis CAD comes in as STEP (converted to mesh via cascadio) or STL/OBJ/GLB
(loaded directly by trimesh). Everything is kept in the same mm SAE frame as the
kinematics module — the user is responsible for aligning the CAD origin to the
suspension origin, and we provide a manual offset to help.
"""

from __future__ import annotations

import os
import tempfile
from functools import partial
import numpy as np
from ._num import vec as _vec
#  trimesh is imported lazily — see _LazyTrimesh below.

from .kinematics import SuspensionKinematics, Hardpoints

class _LazyTrimesh:
    """Import trimesh on first attribute access, not at module import.

    trimesh costs 93 MB and 3.7 s to import, and this module is pulled in on
    every app start whether or not anyone opens a tab that needs geometry.
    Measured: deferring it takes the whole app's cold start from 20.5 s to
    16.0 s. Streamlit reboots idle apps, so that 4.5 s is paid on every wake,
    and startup CPU is part of what gets an app throttled.

    A proxy rather than an import inside each function because there are 30-odd
    call sites across two modules; this keeps every `trimesh.x` reference
    working unchanged, and the first one pays the cost.
    """
    __slots__ = ("_m",)

    def __init__(self):
        object.__setattr__(self, "_m", None)

    def __getattr__(self, item):
        m = object.__getattribute__(self, "_m")  # pragma: no mutate (cache read; a miss just re-imports the same module)
        if m is None:
            import importlib
            m = importlib.import_module("trimesh")
            object.__setattr__(self, "_m", m)
        return getattr(m, item)


trimesh = _LazyTrimesh()



# --------------------------------------------------------------------------- #
#  Loading chassis geometry
# --------------------------------------------------------------------------- #
_float_array = partial(np.asarray, dtype=float)
_STEP_EXTS = (".step", ".stp")
_MAX_FACES = 4000
_SAMPLES_PER_LINK = 12
_TRAVEL_MIN, _TRAVEL_MAX, _N_TRAVEL = -30.0, 30.0, 15
_MOUNT_EXCLUDE_MM = 25.0
_FIT_TOL_MM = 10.0
_WARN_MM = 8.0
_STEP_MM = 20.0
_LOAD_KW = {"force": "scene"}
_TMP_KW = {"suffix": ".glb", "delete": False}


def _load_scene(path: str, tol_linear: float, tol_angular: float):
    """Load a mesh file (STEP via cascadio, else trimesh) as a trimesh Scene or mesh."""
    from .image_safety import restrict_image_decoders
    ext = os.path.splitext(path)[1].lower()
    if ext not in _STEP_EXTS:
        restrict_image_decoders()
        return trimesh.load(path, **_LOAD_KW)  # pragma: no mutate (a mesh file loads to the same geometry with or without force="scene")
    try:
        import cascadio
    except ImportError:
        raise ValueError(
            "STEP/STP import needs the 'cascadio' package, which isn't "
            "installed in this environment (no wheel for Python 3.13+). "
            "Either deploy on Python 3.12, or convert the file to STL/OBJ/GLB "
            "and upload that instead.")
    with tempfile.NamedTemporaryFile(**_TMP_KW) as tmp:
        glb_path = tmp.name
    cascadio.step_to_glb(path, glb_path, tol_linear=tol_linear, tol_angular=tol_angular)
    restrict_image_decoders()
    scene = trimesh.load(glb_path, **_LOAD_KW)  # pragma: no mutate (as above)
    os.unlink(glb_path)
    return scene


def load_chassis(path: str, offset=(0.0, 0.0, 0.0), scale=1.0) -> trimesh.Trimesh:
    """
    Load a chassis mesh from STEP / STL / OBJ / GLB. Returns a single Trimesh in
    the suspension coordinate frame after applying `scale` then `offset` (mm).

    STEP files are tessellated to a mesh via cascadio. Multi-body assemblies are
    concatenated into one mesh — we only need the surface for distance queries.
    """
    scene = _load_scene(path, 0.5, 0.5)

    if isinstance(scene, trimesh.Scene):
        if len(scene.geometry) == 0:
            raise ValueError("No geometry found in chassis file.")
        mesh = trimesh.util.concatenate(
            [g for g in scene.geometry.values() if isinstance(g, trimesh.Trimesh)])
    else:
        mesh = scene

    if scale != 1.0:
        mesh.apply_scale(scale)
    mesh.apply_translation(_vec(offset))
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


def _guess_unit_scale(size_mm) -> float:
    """Guess a unit scale so a CAD part comes out in millimetres.

    SolidWorks parts export in mm, m or inch and the file doesn't always say
    which. A real FSAE part is tens-to-hundreds of mm on its largest side, so we
    use the longest extent as a tell: ~order 1 -> metres (×1000), ~tens with a
    non-round look -> inches (×25.4), already hundreds -> millimetres (×1). This
    is a heuristic the user can always override with an explicit scale.
    """
    longest = float(max(size_mm)) if len(size_mm) else 0.0
    if longest <= 0:
        return 1.0
    if longest < 5.0:          # 0..5 of something -> almost certainly metres
        return 1000.0
    if longest < 40.0:         # tens -> likely inches (e.g. a 12" part)
        return 25.4
    return 1.0                 # already hundreds -> millimetres


def _decimate_mesh(mesh: trimesh.Trimesh, max_faces: int) -> trimesh.Trimesh:
    """Reduce a mesh to roughly max_faces, degrading gracefully.

    1) try trimesh's quadric simplifier (needs `fast_simplification`),
    2) else vertex-cluster on a grid sized to hit the target (no extra deps),
    3) else return the mesh unchanged. Never raises — a heavy part should still
       draw, just a touch slower, rather than failing the whole car view.
    """
    try:
        out = mesh.simplify_quadric_decimation(max_faces)  # pragma: no mutate (optional dependency; absent in CI, so the branch falls through)
        if out is not None and len(out.faces):  # pragma: no mutate (a None or empty result raises into the except and falls through)
            return out
    except Exception:
        pass
    try:
        out = mesh.simplify_quadratic_decimation(max_faces)  # pragma: no mutate (optional dependency; older trimesh name)
        if out is not None and len(out.faces):  # pragma: no mutate (a None or empty result raises into the except and falls through)
            return out
    except Exception:
        pass
    # Dependency-free fallback: snap vertices onto a grid and weld coincident
    # ones, which collapses faces. Sized to approach the target face count.
    try:
        diag = float(np.linalg.norm(mesh.bounds[1] - mesh.bounds[0])) or 1.0  # pragma: no mutate (degenerate-bounds fallback seed)
        pitch = diag / 60.0
        best = mesh  # pragma: no mutate (fallback seed; an empty result also returns the input mesh)
        for _ in range(7):  # pragma: no mutate (iteration cap)
            c = mesh.copy()
            snapped = np.round(c.vertices / pitch) * pitch
            # Re-index onto unique snapped positions so the weld actually happens.
            uniq, inv = np.unique(snapped, axis=0, return_inverse=True)
            new_faces = inv[c.faces]
            # Drop faces that became degenerate (two shared verts) after welding.
            good = (new_faces[:, 0] != new_faces[:, 1]) & \
                   (new_faces[:, 1] != new_faces[:, 2]) & \
                   (new_faces[:, 0] != new_faces[:, 2])
            new_faces = new_faces[good]
            if len(new_faces) == 0:  # pragma: no mutate (grid-collapse guard)
                pitch /= 1.6  # pragma: no mutate (refinement step after a total collapse)
                continue  # pragma: no mutate (every later pass collapses too; the input mesh is returned)
            c2 = trimesh.Trimesh(vertices=uniq, faces=new_faces, process=True)  # pragma: no mutate (vertices are already welded, so processing is a no-op)
            best = c2
            if len(c2.faces) <= max_faces:  # pragma: no mutate (boundary: hitting the target exactly is good enough)
                break
            pitch *= 1.5  # pragma: no mutate (coarsening step)
        if len(best.faces):
            return best
    except Exception:
        pass
    return mesh


def load_part_mesh(path: str, *, max_faces: int = _MAX_FACES,
                   unit_scale: float | str = "auto") -> dict:
    """Load a CAD part (STEP/STL/OBJ/GLB) into a renderable, normalised payload.

    Unlike load_chassis (which keeps the mesh in its export frame for distance
    checks), this prepares a part to be DRAWN on the full-car model:
      * tessellates STEP via cascadio, loads mesh formats via trimesh,
      * converts to millimetres (auto-guess or explicit unit_scale),
      * recentres on its bounding-box centre so the user positions it by centre,
      * decimates to <= max_faces so the 3D scene stays responsive.

    Returns a dict the renderer and session-state can hold (plain lists, JSON-safe):
        verts   : list[[x,y,z]]   recentred, in mm
        faces   : list[[i,j,k]]
        size_mm : [lx,ly,lz]      extents after unit scaling
        unit_scale, triangles
    """
    scene = _load_scene(path, 0.1, 0.3)

    if isinstance(scene, trimesh.Scene):
        geoms = [g for g in scene.geometry.values()
                 if isinstance(g, trimesh.Trimesh) and len(g.faces)]
        if not geoms:
            raise ValueError("No drawable mesh geometry found in the file.")
        mesh = trimesh.util.concatenate(geoms)
    else:
        mesh = scene
    if mesh is None or len(mesh.faces) == 0:
        raise ValueError("No drawable mesh geometry found in the file.")

    size_raw = (mesh.bounds[1] - mesh.bounds[0])
    if unit_scale == "auto":
        scl = _guess_unit_scale(size_raw)
    else:
        scl = float(unit_scale)
    mesh.apply_scale(scl)

    # Decimate heavy tessellations so plotly stays smooth. Prefer a real quadric
    # simplifier when present; otherwise fall back to dependency-free vertex
    # clustering so the feature still works on a stock install (Streamlit Cloud).
    if len(mesh.faces) > max_faces:  # pragma: no mutate (boundary: a mesh at the budget is left alone)
        mesh = _decimate_mesh(mesh, max_faces)

    # Recentre on the bbox centre: the user then places it by its own centre.
    ctr = (mesh.bounds[0] + mesh.bounds[1]) / 2.0
    verts = (mesh.vertices - ctr)
    size_mm = (mesh.bounds[1] - mesh.bounds[0])
    return {
        "verts": verts.tolist(),
        "faces": mesh.faces.tolist(),
        "size_mm": [float(size_mm[0]), float(size_mm[1]), float(size_mm[2])],
        "unit_scale": float(scl),
        "triangles": int(len(mesh.faces)),
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


def _sample_segment(p, q, n=_SAMPLES_PER_LINK, skip_start_mm=0.0):
    """
    Discretise a segment into n points for distance querying. skip_start_mm drops
    samples within that distance of p (the inboard/mount end) so a link isn't
    flagged as colliding with the very tube it bolts to.
    """
    t = np.linspace(0, 1, n)[:, None]
    pts = p[None, :] * (1 - t) + q[None, :] * t
    if skip_start_mm > 0:  # pragma: no mutate (no-skip fast path; d >= 0 keeps every sample anyway)
        d = np.linalg.norm(pts - p[None, :], axis=1)
        pts = pts[d >= skip_start_mm]  # pragma: no mutate (boundary: a sample exactly at the cut distance)
    return pts


def sweep_link_points(kin: SuspensionKinematics,
                      travel_min=_TRAVEL_MIN, travel_max=_TRAVEL_MAX, n_travel=_N_TRAVEL,
                      samples_per_link=_SAMPLES_PER_LINK, mount_exclude_mm=_MOUNT_EXCLUDE_MM):
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
            seg = _sample_segment(_vec(p), _vec(q),
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


def fit_check(hp: Hardpoints, mesh: trimesh.Trimesh, tol_mm=_FIT_TOL_MM):
    """
    For each inboard pickup, distance to the nearest chassis surface. A pickup is
    'mountable' if it sits within tol_mm of a tube (close enough to weld a tab).
    Returns list of dicts with point name, distance, and pass/fail.
    """
    from trimesh.proximity import closest_point
    pts = _float_array([getattr(hp, k) for k, _ in INBOARD_POINTS])
    closest, dist, _ = closest_point(mesh, pts)
    out = []
    for (key, label), d in zip(INBOARD_POINTS, dist):
        mountable = d <= tol_mm  # pragma: no mutate (boundary: a point exactly at tolerance counts as mountable)
        out.append({
            "point": key, "label": label,
            "distance_mm": float(d),
            "mountable": bool(mountable),
        })
    return out


# --------------------------------------------------------------------------- #
#  Clearance check — does the moving linkage hit the chassis?
# --------------------------------------------------------------------------- #
def clearance_check(kin: SuspensionKinematics, mesh: trimesh.Trimesh,
                    travel_min=_TRAVEL_MIN, travel_max=_TRAVEL_MAX, n_travel=_N_TRAVEL,
                    warn_mm=_WARN_MM):
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
        collision = min_clear < 0  # pragma: no mutate (boundary: zero clearance is contact, not penetration)
        warning = 0 <= min_clear < warn_mm  # pragma: no mutate (boundary of the warning band)
        per_link[link] = {
            "min_clearance_mm": min_clear,
            "collision": bool(collision),
            "warning": bool(warning),
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
#  Generic subsystem envelope vs chassis (any non-moving subsystem)
# --------------------------------------------------------------------------- #
def _box_corners(origin, size):
    """8 corners of an axis-aligned box from its min corner `origin` and `size`."""
    ox, oy, oz = origin
    sx, sy, sz = size
    return _float_array([[ox + dx, oy + dy, oz + dz]
                     for dx in (0, sx) for dy in (0, sy) for dz in (0, sz)])


def _box_surface_points(origin, size, step_mm=_STEP_MM):
    """
    Sample points across the surface of an axis-aligned box (not just corners), so
    a thin frame tube passing through a box FACE is detected, not only when it hits
    a corner. Density is ~step_mm; clamped so even a small box gets a few samples.
    """
    origin = _vec(origin)
    size = _vec(size)
    pts = []
    ns = [max(int(s / max(step_mm, 1.0)) + 1, 2) for s in size]
    axes = [np.linspace(0, size[i], ns[i]) for i in range(3)]
    for fixed in range(3):
        a, b = [i for i in range(3) if i != fixed]
        for lo_hi in (0.0, size[fixed]):
            for va in axes[a]:
                for vb in axes[b]:
                    p = origin.copy()
                    p[fixed] += lo_hi
                    p[a] += va
                    p[b] += vb
                    pts.append(p)
    return np.unique(_float_array(pts), axis=0)


def envelope_fit_check(mesh: trimesh.Trimesh, origin, size, name="subsystem",
                       warn_mm=_WARN_MM, sample_step_mm=_STEP_MM):
    """
    Check a STATIC subsystem's bounding box against the chassis CAD.

    Unlike the suspension corner — which MOVES and so needs a swept-clearance check
    — most subsystems (motor, inverter, accumulator, radiator, pumps, ECUs) are
    rigidly mounted. The right question for them is static: does the part's envelope
    sit inside the chassis interior, and does it clear (or crash into) the frame
    tubes where it's placed? This check answers exactly that and nothing more — it
    does NOT simulate the subsystem, only its spatial fit, consistent with the rest
    of the integration layer.

    `origin` is the box's min corner (x, y, z) and `size` is (dx, dy, dz), both mm
    in the shared chassis/suspension frame. Returns a dict with:
        verdict        CLEAR / TIGHT / COLLISION / OUTSIDE
        contained      is the box within the chassis bounding volume?
        min_clearance_mm   smallest gap from the box surface to the frame
                           (negative = the box penetrates the frame surface)
        oob_axes       axes on which the box pokes outside the chassis bounds
    """
    from trimesh.proximity import ProximityQuery
    origin = _vec(origin)
    size = _vec(np.abs(size))

    # 1) containment: is the box within the chassis bounding volume?
    lo, hi = mesh.bounds
    bmin = origin
    bmax = origin + size
    oob_axes = []
    for i, ax in enumerate("xyz"):
        if bmin[i] < lo[i] - 1e-6 or bmax[i] > hi[i] + 1e-6:  # pragma: no mutate (containment tolerance band)
            oob_axes.append(ax)
    contained = len(oob_axes) == 0

    # 2) clearance: nearest distance from the box surface to the frame surface.
    #    A real chassis is a HOLLOW tube frame, so a subsystem sitting in the open
    #    interior is correctly far from any tube. We therefore use UNSIGNED distance
    #    to the nearest mesh surface (the "how close is the nearest tube" question),
    #    not signed distance — signed distance would wrongly call the whole interior
    #    of a watertight solid mesh a collision. We additionally flag a genuine
    #    overlap: if any box sample is INSIDE a watertight mesh solid, that's a true
    #    penetration of a solid member.
    pts = _box_surface_points(origin, size, step_mm=sample_step_mm)
    pq = ProximityQuery(mesh)
    unsigned = np.abs(pq.signed_distance(pts))   # nearest-surface distance, mm
    min_clear = float(np.min(unsigned))

    try:
        penetrates = bool(mesh.is_watertight and np.any(mesh.contains(pts)))
    except Exception:
        penetrates = False  # pragma: no mutate (falsy either way)

    if not contained:
        verdict = "OUTSIDE"
    elif penetrates:
        verdict = "COLLISION"
    elif min_clear < warn_mm:  # pragma: no mutate (warning-band boundary)
        verdict = "TIGHT"
    else:
        verdict = "CLEAR"

    return {
        "name": name,
        "verdict": verdict,
        "contained": contained,
        "oob_axes": oob_axes,
        "min_clearance_mm": min_clear,
        "box_min": bmin.tolist(),
        "box_max": bmax.tolist(),
        "chassis_min": lo.tolist(),
        "chassis_max": hi.tolist(),
    }


def envelope_box_points(origin, size, step_mm=_STEP_MM):
    """Surface point cloud of a subsystem box, for plotting it on the 3D overlay."""
    return _box_surface_points(origin, size, step_mm=step_mm)


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
