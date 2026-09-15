# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/genesis_analysis.py — corner and vehicle analyses that sit next
#  to the InverseGenesis engine: frame recovery, packaging, steering effort,
#  actuation, brackets and link compliance.
# ============================================================================
"""
Deterministic analyses fed by the InverseGenesis corner and the shared
vehicle declaration. Nothing here is fitted or searched.

WHAT LIVES HERE
---------------
* ``parse_step_tubes`` / ``frame_stats`` — tube axes, wall thicknesses and
  bend arcs straight from a STEP file's B-rep entities (no CAD kernel), and
  the tube count, straight run, bend arc, node count per clustering tolerance
  and vertex accounting of the recovered frame.
* ``wheelbase_check`` / ``static_clearance`` — axle stations against the
  rules minimum; frame clearance above the declared ground plane.
* ``steering_torque`` — trail, per-wheel kingpin torque and steering-wheel
  torque against steering ratio.
* ``tyre_table`` — peak µ, peak slip and optimum camber at stated loads.
* ``actuation_summary`` / ``roll_stiffness_from_spring`` /
  ``ride_frequency_hz`` — motion ratio across travel, pushrod and damper
  lengths, damper travel used, wheel-rate spread, ride frequency across
  travel, and the placeholder-motion-ratio roll-stiffness error.
* ``bracket_fos`` — double-shear clevis root bending.
* ``axial_budget`` — link axial strain + rod-end lash, and any deflection as
  a fraction of an acceptance band.
* ``ball_joint_envelope`` — ball-joint heights against the rim envelope.

Every function states its units. The bracket stress-concentration factor is
an explicit input; its 1.28 default is a declared assumption, not derived.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field as _dcfield

import numpy as np

from .kinematics import Hardpoints, SuspensionKinematics


# =========================================================================== #
#  STEP (ISO 10303-21) B-rep reader — just enough for tube frames
# =========================================================================== #
_ENTITY = re.compile(r"#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.*)\)\s*$", re.S)


def _split_entities(text: str):
    """Yield raw entity strings from the DATA section, splitting on ';'
    outside quoted strings."""
    m = re.search(r"\bDATA\s*;(.*?)\bENDSEC\s*;", text, re.S)
    body = m.group(1) if m else text
    buf, in_str = [], False
    for ch in body:
        if ch == "'":
            in_str = not in_str
        if ch == ";" and not in_str:
            yield "".join(buf).strip()
            buf = []
        else:
            buf.append(ch)


def _parse_args(s: str):
    """STEP argument list → nested Python lists (refs as ('#', id), numbers as
    float, enums/strings as str)."""
    out, stack, tok, in_str = [], [], "", False
    cur = out

    def flush():
        nonlocal tok
        t = tok.strip()
        tok = ""
        if not t:
            return
        if t.startswith("#"):
            cur.append(("#", int(t[1:])))
            return
        try:
            cur.append(float(t))
        except ValueError:
            cur.append(t)

    for ch in s:
        if in_str:
            tok += ch
            if ch == "'":
                in_str = False
            continue
        if ch == "'":
            in_str = True
            tok += ch
        elif ch == "(":
            new = []
            cur.append(new)
            stack.append(cur)
            cur = new
        elif ch == ")":
            flush()
            cur = stack.pop()
        elif ch == ",":
            flush()
        else:
            tok += ch
    flush()
    return out


def _refs(x):
    if isinstance(x, tuple) and x and x[0] == "#":
        return [x[1]]
    if isinstance(x, list):
        r = []
        for v in x:
            r += _refs(v)
        return r
    return []


@dataclass
class StepTubes:
    """What ``parse_step_tubes`` recovers, all lengths in mm."""
    axes: list = _dcfield(default_factory=list)      # [(p0, p1, wall_mm|None)]
    bends: list = _dcfield(default_factory=list)     # [(major_mm, angle_deg)]
    radii_mm: dict = _dcfield(default_factory=dict)  # cylinder radius -> faces
    vertices: np.ndarray = _dcfield(
        default_factory=lambda: np.zeros((0, 3)))
    rejected: int = 0
    units_note: str = ""


def parse_step_tubes(text: str, outer_radius_mm: float = 12.70,
                     radius_tol_mm: float = 0.02,
                     axis_tol_mm: float = 0.5,
                     min_length_mm: float = 1.0) -> StepTubes:
    """Tube axis lines, walls and bends from STEP text (lengths in mm).

    Method: every CYLINDRICAL_SURFACE of the
    outer radius is collapsed into unique axis lines; each axis is bounded by
    the vertices of ITS OWN faces (not by every vertex within the radius —
    that envelope rule over-extends at every joint); the wall is the outer
    radius minus the radius of a coaxial inner cylinder; each
    TOROIDAL_SURFACE whose minor radius is the outer radius is a bend whose
    arc is major radius × swept angle, the angle read from its face vertices.
    Assumes the file's length unit is mm (SolidWorks default); the returned
    ``units_note`` says what the header declared.
    """
    E = {}
    for raw in _split_entities(text):
        m = _ENTITY.match(raw)
        if m:
            E[int(m.group(1))] = (m.group(2), _parse_args(m.group(3)))

    def pt(i):
        """Point or vertex entity → xyz in the file's length unit (mm)."""
        k, a = E[i]
        if k == "CARTESIAN_POINT":
            return np.array(a[1], float)
        if k == "VERTEX_POINT":
            return pt(_refs(a[1])[0])
        raise KeyError(k)

    def direction(i):
        """DIRECTION entity → unit vector (dimensionless)."""
        k, a = E[i]
        v = np.array(a[1], float)
        return v / np.linalg.norm(v)

    def placement(i):
        """AXIS2_PLACEMENT_3D → (origin in mm, unit axis, dimensionless)."""
        _, a = E[i]
        r = _refs(a)
        o = pt(r[0])
        z = direction(r[1]) if len(r) > 1 else np.array([0, 0, 1.0])
        return o, z

    def face_vertices(face_id):
        """Vertex ids bounding a face (ids, no unit; points are in mm)."""
        _, a = E[face_id]
        vs = set()
        for b in _refs(a[1]):                         # FACE_(OUTER_)BOUND
            loop = _refs(E[b][1][1])[0]
            kind, la = E[loop]
            if kind == "VERTEX_LOOP":
                vs.add(_refs(la[1])[0])
                continue
            for oe in _refs(la[1]):                   # ORIENTED_EDGE
                ec = _refs(E[oe][1])[-1]              # EDGE_CURVE
                ea = E[ec][1]
                r = _refs(ea)
                vs.update(r[:2])
        return vs

    # faces grouped by surface
    faces_of = {}
    for i, (k, a) in E.items():
        if k in ("ADVANCED_FACE", "FACE_SURFACE"):
            surf = _refs(a[2])[0]
            faces_of.setdefault(surf, []).append(i)

    out = StepTubes()
    unit = re.search(r"SI_UNIT\s*\(\s*\.(\w+)\.\s*,\s*\.METRE\.", text)
    out.units_note = ("length unit: " + (unit.group(1).lower() + "metre"
                                          if unit else "not declared (mm assumed)"))

    all_v = {i for i, (k, _) in E.items() if k == "VERTEX_POINT"}
    out.vertices = (np.array([pt(i) for i in sorted(all_v)])
                    if all_v else np.zeros((0, 3)))

    outer, inner = [], []
    for sid, fids in faces_of.items():
        k, a = E[sid]
        if k != "CYLINDRICAL_SURFACE":
            continue
        rad = float(a[2])
        o, d = placement(_refs(a[1])[0])
        key = round(rad, 3)
        out.radii_mm[key] = out.radii_mm.get(key, 0) + len(fids)
        vids = set()
        for f in fids:
            vids |= face_vertices(f)
        rec = (o, d, vids, rad)
        if abs(rad - outer_radius_mm) <= radius_tol_mm:
            outer.append(rec)
        elif rad < outer_radius_mm:
            inner.append(rec)

    def same_line(o1, d1, o2, d2):
        """True if two axis lines coincide within axis_tol_mm (mm)."""
        if abs(abs(float(d1 @ d2)) - 1.0) > 1e-6:
            return False
        w = o2 - o1
        return np.linalg.norm(w - (w @ d1) * d1) <= axis_tol_mm

    lines = []                                  # [o, d, vids]
    for o, d, vids, _ in outer:
        for L in lines:
            if same_line(L[0], L[1], o, d):
                L[2] |= vids
                break
        else:
            lines.append([o, d, set(vids)])

    for o, d, vids in lines:
        if len(vids) < 2:
            out.rejected += 1
            continue
        s = [float((pt(v) - o) @ d) for v in vids]
        if max(s) - min(s) < min_length_mm:
            out.rejected += 1
            continue
        p0, p1 = o + min(s) * d, o + max(s) * d
        wall = None
        for io, idir, _, irad in inner:
            if same_line(o, d, io, idir):
                wall = round(outer_radius_mm - irad, 3)
                break
        out.axes.append((p0, p1, wall))

    for sid, fids in faces_of.items():
        k, a = E[sid]
        if k != "TOROIDAL_SURFACE":
            continue
        major, minor = float(a[2]), float(a[3])
        if abs(minor - outer_radius_mm) > radius_tol_mm:
            continue
        c, ax = placement(_refs(a[1])[0])
        vids = set()
        for f in fids:
            vids |= face_vertices(f)
        ref = np.cross(ax, [1.0, 0, 0])
        if np.linalg.norm(ref) < 1e-6:
            ref = np.cross(ax, [0, 1.0, 0])
        ref /= np.linalg.norm(ref)
        ref2 = np.cross(ax, ref)
        angs = []
        for v in vids:
            w = pt(v) - c
            w = w - (w @ ax) * ax
            if np.linalg.norm(w) > 1e-9:
                angs.append(math.atan2(w @ ref2, w @ ref))
        if len(angs) < 2:
            continue
        angs = sorted(set(round(x, 9) for x in angs))
        gaps = [angs[i + 1] - angs[i] for i in range(len(angs) - 1)]
        gaps.append(2 * math.pi - (angs[-1] - angs[0]))
        span = 2 * math.pi - max(gaps)
        out.bends.append((major, math.degrees(span)))
    return out


def _cluster_count(points: np.ndarray, tol_mm: float) -> tuple[int, np.ndarray]:
    """Single-linkage clusters within tol_mm; returns (count, label per point)."""
    n = len(points)
    parent = list(range(n))

    def find(i):
        """Union-find root of a point index (index, dimensionless)."""
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        d = np.linalg.norm(points[i + 1:] - points[i], axis=1)
        for j in np.nonzero(d <= tol_mm)[0]:
            a, b = find(i), find(i + 1 + int(j))
            if a != b:
                parent[a] = b
    labels = np.array([find(i) for i in range(n)])
    return len(set(labels.tolist())), labels


def frame_stats(axes, bends=(), cluster_tols_mm=(20, 30, 37, 40, 50),
                vertices=None, outer_radius_mm: float = 12.70,
                small_radii_mm=()) -> dict:
    """Frame summary from tube axes (mm) and bends (major mm, angle deg).

    Returns tube count, straight run (m), bend arc (m), total (m), node count
    and node-to-node centreline length (m) per clustering tolerance (mm),
    wall-thickness census (mm), and — given the model's vertices (mm) —
    how many lie in a tube envelope, on a bend, or elsewhere.
    """
    axes = list(axes)
    p = [(np.asarray(a[0], float), np.asarray(a[1], float)) for a in axes]
    straight = sum(float(np.linalg.norm(b - a)) for a, b in p)
    arc = sum(float(r) * math.radians(float(t)) for r, t in bends)
    ends = (np.array([q for ab in p for q in ab])
            if p else np.zeros((0, 3)))
    nodes = {}
    for tol in cluster_tols_mm:
        cnt, lab = _cluster_count(ends, float(tol))
        cent = {l: ends[lab == l].mean(axis=0) for l in set(lab.tolist())}
        n2n = sum(float(np.linalg.norm(cent[lab[2 * i + 1]] - cent[lab[2 * i]]))
                  for i in range(len(p)))
        nodes[float(tol)] = {"nodes": cnt, "node_to_node_m": n2n / 1000.0}
    walls = {}
    for a in axes:
        w = a[2] if len(a) > 2 else None
        walls[w] = walls.get(w, 0) + 1
    out = {"tubes": len(p), "straight_m": straight / 1000.0,
           "bends": len(bends), "bend_arc_m": arc / 1000.0,
           "total_m": (straight + arc) / 1000.0, "nodes": nodes,
           "walls_mm": walls}
    if vertices is not None and len(vertices):
        V = np.asarray(vertices, float)
        inside = np.zeros(len(V), bool)
        for a, b in p:
            d = b - a
            L = np.linalg.norm(d)
            if L < 1e-9:
                continue
            u = d / L
            s = (V - a) @ u
            perp = np.linalg.norm((V - a) - np.outer(s, u), axis=1)
            inside |= (s >= -1e-6) & (s <= L + 1e-6) & \
                (perp <= outer_radius_mm + 1e-3)
        out["vertices"] = {"total": int(len(V)),
                           "in_tube_envelope": int(inside.sum()),
                           "outside_envelopes": int((~inside).sum())}
    return out


# =========================================================================== #
#  Vehicle declarations
# =========================================================================== #
def wheelbase_check(front_station_mm: float, rear_station_mm: float,
                    rules_min_mm: float = 1525.0) -> dict:
    """Wheelbase (mm) from two longitudinal axle stations (mm) and its margin
    (mm) against the rules minimum (mm)."""
    wb = float(front_station_mm) - float(rear_station_mm)
    return {"wheelbase_mm": wb, "margin_mm": wb - rules_min_mm,
            "passes": wb >= rules_min_mm}


def static_clearance(lowest_member_y_mm: float, ground_y_mm: float) -> dict:
    """Clearance (mm) from the declared ground plane to the lowest frame
    member, both as CAD heights (mm)."""
    return {"clearance_mm": float(lowest_member_y_mm) - float(ground_y_mm)}


def tyre_table(loads_N=(550.0, 1100.0, 1650.0), tire=None) -> list:
    """Peak µ (dimensionless), peak slip (deg) and optimum camber (deg) at
    each vertical load (N) for the declared tyre model."""
    if tire is None:
        from .tiremodel import default_tire
        tire = default_tire()
    rows = []
    for fz in loads_N:
        cam, mu_at = tire.optimal_camber(float(fz))
        rows.append({"Fz_N": float(fz),
                     "mu_peak": float(tire.mu_peak(float(fz))),
                     "alpha_peak_deg": float(tire.alpha_peak(float(fz))),
                     "optimal_camber_deg": float(cam),
                     "mu_at_optimal_camber": float(mu_at)})
    return rows


# =========================================================================== #
#  Steering effort
# =========================================================================== #
def steering_torque(caster_deg: float, tire_radius_mm: float,
                    fz_outer_N: float, fz_inner_N: float, mu: float,
                    mz_outer_Nm: float, mz_inner_Nm: float = 0.0,
                    ratios=(4.0, 5.0, 6.0, 8.0),
                    target_Nm: float = 10.0) -> dict:
    """Mechanical trail (mm), kingpin torque per wheel (N·m), road-wheel total
    (N·m) and steering-wheel torque (N·m) per steering ratio (dimensionless).

    trail = R · tan(caster); per wheel T = µ·Fz·trail + Mz.
    """
    trail = float(tire_radius_mm) * math.tan(math.radians(caster_deg))
    fy_o, fy_i = mu * fz_outer_N, mu * fz_inner_N
    t_o = fy_o * trail / 1000.0 + mz_outer_Nm
    t_i = fy_i * trail / 1000.0 + mz_inner_Nm
    total = t_o + t_i
    rows = [{"ratio": float(r), "steering_wheel_Nm": total / float(r),
             "meets_target": total / float(r) <= target_Nm}
            for r in ratios]
    return {"trail_mm": trail, "fy_outer_N": fy_o, "fy_inner_N": fy_i,
            "trail_torque_outer_Nm": fy_o * trail / 1000.0,
            "outer_Nm": t_o, "inner_Nm": t_i, "road_wheel_total_Nm": total,
            "ratio_needed_for_target": total / target_Nm, "rows": rows}


# =========================================================================== #
#  Actuation, ride and roll
# =========================================================================== #
def roll_stiffness_from_spring(spring_rate_N_mm: float, motion_ratio: float,
                               track_mm: float,
                               placeholder_mr: float = 1.0) -> dict:
    """Axle roll stiffness (N·m/deg) from coil rate (N/mm), motion ratio
    (dimensionless) and track (mm), and the error (%) a placeholder motion
    ratio would introduce. K = k·MR²·t²/2 per radian."""
    def k(mr):
        """Roll stiffness (N·m/deg) at motion ratio ``mr`` (dimensionless)."""
        return spring_rate_N_mm * mr ** 2 * track_mm ** 2 / 2 \
            * math.pi / 180.0 / 1000.0
    true_k, ph = k(motion_ratio), k(placeholder_mr)
    return {"wheel_rate_N_mm": spring_rate_N_mm * motion_ratio ** 2,
            "roll_stiffness_Nm_deg": true_k,
            "placeholder_Nm_deg": ph,
            "placeholder_error_pct": 100.0 * (ph / true_k - 1.0)
            if true_k else math.nan}


def ride_frequency_hz(spring_rate_N_mm: float, motion_ratio: float,
                      sprung_corner_mass_kg: float) -> float:
    """Ride frequency (Hz) from coil rate (N/mm), motion ratio (dimensionless)
    and sprung corner mass (kg)."""
    k_n_m = spring_rate_N_mm * motion_ratio ** 2 * 1000.0
    return math.sqrt(k_n_m / sprung_corner_mass_kg) / (2 * math.pi)


def actuation_summary(hp: Hardpoints, travel_mm: float = 25.0, n: int = 11,
                      spring_rate_N_mm: float | None = None,
                      sprung_corner_mass_kg: float | None = None,
                      damper_stroke_mm: float | None = None) -> dict:
    """Actuation summary: motion ratio (dimensionless) static and at ±travel,
    pushrod length (mm), attachment fraction along the arm (dimensionless),
    damper static length (mm), damper travel used (mm), wheel-rate spread
    (%), and — given coil rate (N/mm) and sprung mass (kg) — ride frequency
    (Hz) at droop / static / bump."""
    kin = SuspensionKinematics(hp)
    if not kin.motion_ratio_is_real():
        return {"ok": False,
                "reason": "pushrod_outer, rocker_pivot, rocker_axis, "
                          "rocker_pushrod, rocker_spring and spring_inner "
                          "must all be defined"}
    trv, mr = kin.motion_ratio_curve(-travel_mm, travel_mm, n)
    mr = np.asarray(mr, float)
    i0 = int(np.argmin(np.abs(np.asarray(trv))))
    s0 = kin.solve_at_travel(0.0)
    lengths, seed = [], 0.0
    for t in trv:
        L, seed, ok = kin.spring_length_at(kin.solve_at_travel(float(t)),
                                           seed=seed)
        lengths.append(L if ok else math.nan)
    lengths = np.asarray(lengths)
    pro = np.asarray(s0.pushrod_outer if s0.pushrod_outer is not None
                     else hp.pushrod_outer, float)
    push_len = float(np.linalg.norm(np.asarray(hp.rocker_pushrod) - pro))
    body = getattr(hp, "pushrod_attach", "lower")
    if body in ("lower", "upper"):
        fi = np.asarray(getattr(hp, f"{body}_front_inner"), float)
        ri = np.asarray(getattr(hp, f"{body}_rear_inner"), float)
        bj = np.asarray(getattr(s0, f"{body}_outer"), float)
        mid = 0.5 * (fi + ri)
        arm = bj - mid
        frac = float((pro - mid) @ arm / (arm @ arm))
    else:
        frac = math.nan
    wr = mr ** 2
    out = {"ok": True,
           "travel_mm": [float(t) for t in trv],
           "motion_ratio": mr.tolist(),
           "mr_static": float(mr[i0]),
           "mr_droop": float(mr[0]), "mr_bump": float(mr[-1]),
           "rate_character": ("rising" if mr[-1] > mr[i0] > mr[0]
                              else "falling" if mr[-1] < mr[i0] < mr[0]
                              else "peaks/dips at ride height"),
           "pushrod_length_mm": push_len,
           "pushrod_attach": body,
           "attachment_fraction": frac,
           "damper_static_mm": float(lengths[i0]),
           "damper_travel_used_mm": float(np.nanmax(lengths)
                                          - np.nanmin(lengths)),
           "mr_spread_pct": 100 * float(np.nanmax(mr) - np.nanmin(mr))
           / float(np.nanmin(mr)),
           "wheel_rate_spread_pct": 100 * float(np.nanmax(wr) - np.nanmin(wr))
           / float(np.nanmin(wr))}
    if damper_stroke_mm:
        out["stroke_fraction_used"] = out["damper_travel_used_mm"] \
            / float(damper_stroke_mm)
    if spring_rate_N_mm and sprung_corner_mass_kg:
        f = [ride_frequency_hz(spring_rate_N_mm, float(m),
                               sprung_corner_mass_kg) for m in mr]
        out["ride_hz_droop"] = f[0]
        out["ride_hz_static"] = f[i0]
        out["ride_hz_bump"] = f[-1]
    return out


# =========================================================================== #
#  Brackets and compliance
# =========================================================================== #
def bracket_fos(load_N: float, reach_mm: float, plate_width_mm: float,
                plate_thickness_mm: float, n_plates: int = 2,
                allowable_mpa: float = 460.0, kt: float = 1.28) -> dict:
    """Double-shear clevis root-bending factor of safety (dimensionless).

    Each plate carries load/n_plates (N) at the reach (mm) as a cantilever
    bent about its thin axis: σ = Kt·M/Z, Z = w·t²/6 (mm³), allowable in MPa.
    ``kt`` is a declared stress-concentration / combined-load factor
    (dimensionless); 1.28 is the default declaration.
    """
    m = load_N / n_plates * reach_mm
    z = plate_width_mm * plate_thickness_mm ** 2 / 6.0
    sigma = kt * m / z
    return {"moment_Nmm": m, "section_modulus_mm3": z,
            "stress_mpa": sigma, "fos": allowable_mpa / sigma}


def axial_budget(force_N: float, length_mm: float, od_mm: float = 15.88,
                 wall_mm: float = 0.889, E_gpa: float = 205.0,
                 lash_mm: float = 0.025, n_joints: int = 2,
                 reported_extension_mm: float | None = None) -> dict:
    """Link axial strain (mm) = F·L/(E·A), rod-end lash (mm) per joint, their
    sum (mm), and the unexplained remainder (mm) of a reported extension."""
    ri = od_mm / 2 - wall_mm
    area = math.pi * ((od_mm / 2) ** 2 - ri ** 2)
    strain = force_N * length_mm / (E_gpa * 1e3 * area)
    lash = lash_mm * n_joints
    out = {"area_mm2": area, "axial_strain_mm": strain, "lash_mm": lash,
           "sum_mm": strain + lash}
    if reported_extension_mm is not None:
        out["unexplained_mm"] = reported_extension_mm - strain - lash
    return out


def band_fractions(deflections: dict, bands: dict) -> dict:
    """Each deflection (deg or mm) as a fraction (dimensionless) of the
    matching acceptance band half-width (same unit)."""
    return {k: abs(float(v)) / float(bands[k])
            for k, v in deflections.items() if k in bands and bands[k]}


def ball_joint_envelope(hp: Hardpoints, max_offset_mm: float = 115.0) -> dict:
    """Upper and lower ball-joint heights relative to the wheel centre (mm)
    against the rim envelope half-height (mm)."""
    s0 = SuspensionKinematics(hp).solve_at_travel(0.0)
    wc = float(s0.wheel_center[2])
    up = float(s0.upper_outer[2]) - wc
    lo = float(s0.lower_outer[2]) - wc
    return {"upper_above_wc_mm": up, "lower_below_wc_mm": -lo,
            "max_offset_mm": max_offset_mm,
            "inside": abs(up) <= max_offset_mm and abs(lo) <= max_offset_mm}
