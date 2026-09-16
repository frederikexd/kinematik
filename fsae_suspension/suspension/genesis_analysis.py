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
    outside quoted strings (linear time: split on quotes first)."""
    m = re.search(r"\bDATA\s*;(.*?)\bENDSEC\s*;", text, re.S)
    body = m.group(1) if m else text
    buf = []
    for i, chunk in enumerate(body.split("'")):
        if i % 2:                      # inside a string literal
            buf.append("'" + chunk + "'")
            continue
        parts = chunk.split(";")
        buf.append(parts[0])
        for part in parts[1:]:
            yield "".join(buf).strip()
            buf = [part]
    tail = "".join(buf).strip()
    if tail:
        yield tail


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
    od_mm: list = _dcfield(default_factory=list)     # outer diameter per axis
    bends: list = _dcfield(default_factory=list)     # [(major_mm, angle_deg)]
    radii_mm: dict = _dcfield(default_factory=dict)  # cylinder radius -> faces
    tube_radii_mm: list = _dcfield(default_factory=list)
    vertices: np.ndarray = _dcfield(
        default_factory=lambda: np.zeros((0, 3)))
    rejected: int = 0
    units_note: str = ""
    unit_scale: float = 1.0


_UNIT_SCALE = {"MILLI": 1.0, "CENTI": 10.0, "DECI": 100.0, "": 1000.0,
               "MICRO": 1e-3}


def parse_step_tubes(text: str, outer_radius_mm=None,
                     radius_tol_mm: float = 0.02,
                     axis_tol_mm: float = 0.5,
                     min_length_mm: float = 1.0,
                     detect_range_mm=(4.0, 40.0)) -> StepTubes:
    """Tube axis lines, walls and bends from STEP text (lengths in mm).

    Method: every CYLINDRICAL_SURFACE of a tube's outer radius is collapsed
    into unique axis lines; each axis is bounded by the vertices of ITS OWN
    faces (not by every vertex within the radius — that envelope rule
    over-extends at every joint); the wall is the outer radius minus the
    radius of a coaxial inner cylinder; each TOROIDAL_SURFACE whose minor
    radius is a tube radius is a bend whose arc is major radius × swept
    angle, the angle read from its face vertices.

    ``outer_radius_mm`` may be one radius (mm), several, or None to detect
    the tube sizes: radii within ``detect_range_mm`` (mm) that occur on at
    least two faces and are not only ever the bore of a larger coaxial
    cylinder. The file's declared length unit is converted to mm.
    """
    E = {}
    for raw in _split_entities(text):
        m = _ENTITY.match(raw)
        if m:
            E[int(m.group(1))] = (m.group(2), _parse_args(m.group(3)))

    out = StepTubes()
    unit = re.search(r"SI_UNIT\s*\(\s*(?:\.(\w+)\.|\$)\s*,\s*\.METRE\.",
                     text)
    prefix = (unit.group(1) or "") if unit else "MILLI"
    scale = _UNIT_SCALE.get(prefix, 1.0)
    out.unit_scale = scale
    out.units_note = ("length unit: " + (prefix.lower() + "metre"
                                          if unit else "not declared (mm assumed)")
                      + ("" if scale == 1.0 else " — converted to mm"))

    def pt(i):
        """Point or vertex entity → xyz in mm."""
        k, a = E[i]
        if k == "CARTESIAN_POINT":
            return np.array(a[1], float) * scale
        if k == "VERTEX_POINT":
            return pt(_refs(a[1])[0])
        raise KeyError(k)

    def direction(i):
        """DIRECTION entity → unit vector (dimensionless)."""
        _, a = E[i]
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
        for b in _refs(a[1]):
            loop = _refs(E[b][1][1])[0]
            kind, la = E[loop]
            if kind == "VERTEX_LOOP":
                vs.add(_refs(la[1])[0])
                continue
            for oe in _refs(la[1]):
                ec = _refs(E[oe][1])[-1]
                vs.update(_refs(E[ec][1])[:2])
        return vs

    def same_line(o1, d1, o2, d2):
        """True if two axis lines coincide within axis_tol_mm (mm)."""
        if abs(abs(float(d1 @ d2)) - 1.0) > 1e-6:
            return False
        w = o2 - o1
        return np.linalg.norm(w - (w @ d1) * d1) <= axis_tol_mm

    faces_of = {}
    for i, (k, a) in E.items():
        if k in ("ADVANCED_FACE", "FACE_SURFACE"):
            try:
                faces_of.setdefault(_refs(a[2])[0], []).append(i)
            except (IndexError, TypeError):
                continue

    all_v = [i for i, (k, _) in E.items() if k == "VERTEX_POINT"]
    out.vertices = (np.array([pt(i) for i in sorted(all_v)])
                    if all_v else np.zeros((0, 3)))

    cyls = []                                  # (radius, o, d, vids)
    for sid, fids in faces_of.items():
        k, a = E[sid]
        if k != "CYLINDRICAL_SURFACE":
            continue
        rad = float(a[2]) * scale
        o, d = placement(_refs(a[1])[0])
        vids = set()
        for f in fids:
            vids |= face_vertices(f)
        key = round(rad, 3)
        out.radii_mm[key] = out.radii_mm.get(key, 0) + len(fids)
        cyls.append((rad, o, d, vids))

    if outer_radius_mm is None:
        lo, hi = detect_range_mm
        cand = sorted(r for r, n in out.radii_mm.items()
                      if lo <= r <= hi and n >= 2)
        radii = []
        for r in cand:
            mine = [c for c in cyls if abs(c[0] - r) <= radius_tol_mm]
            outer_somewhere = any(
                not any(c2[0] > r + radius_tol_mm
                        and same_line(c[1], c[2], c2[1], c2[2])
                        for c2 in cyls)
                for c in mine)
            if outer_somewhere:
                radii.append(r)
    elif isinstance(outer_radius_mm, (int, float)):
        radii = [float(outer_radius_mm)]
    else:
        radii = [float(r) for r in outer_radius_mm]
    out.tube_radii_mm = radii

    for R in radii:
        lines = []
        for rad, o, d, vids in cyls:
            if abs(rad - R) > radius_tol_mm:
                continue
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
            sproj = [float((pt(v) - o) @ d) for v in vids]
            if max(sproj) - min(sproj) < min_length_mm:
                out.rejected += 1
                continue
            p0, p1 = o + min(sproj) * d, o + max(sproj) * d
            wall, best = None, -1.0
            for rad, io, idir, _ in cyls:
                if best < rad < R - radius_tol_mm and \
                        same_line(o, d, io, idir):
                    best = rad
                    wall = round(R - rad, 3)
            out.axes.append((p0, p1, wall))
            out.od_mm.append(round(2 * R, 3))

    for sid, fids in faces_of.items():
        k, a = E[sid]
        if k != "TOROIDAL_SURFACE":
            continue
        major, minor = float(a[2]) * scale, float(a[3]) * scale
        if not any(abs(minor - R) <= radius_tol_mm for R in radii):
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
        for vid in vids:
            w = pt(vid) - c
            w = w - (w @ ax) * ax
            if np.linalg.norm(w) > 1e-9:
                angs.append(math.atan2(w @ ref2, w @ ref))
        if len(angs) < 2:
            continue
        angs = sorted(set(round(x, 9) for x in angs))
        gaps = [angs[i + 1] - angs[i] for i in range(len(angs) - 1)]
        gaps.append(2 * math.pi - (angs[-1] - angs[0]))
        out.bends.append((major, math.degrees(2 * math.pi - max(gaps))))
    return out


def frame_graph_from_step(res: StepTubes, cluster_tol_mm: float = 20.0):
    """FrameGraph (node coordinates in mm, CAD axes) from recovered tubes:
    ends clustered within ``cluster_tol_mm`` (mm) become nodes, and each
    tube keeps its own OD and wall (mm) as a size-table entry."""
    from .tubeframe import FrameGraph, TubeSpec as FrameTubeSpec
    sizes = {}
    for od, (_, _, w) in zip(res.od_mm, res.axes):
        key = f"STEP {od:g}x{(w if w is not None else 0):g}"
        sizes[key] = FrameTubeSpec(key, float(od),
                                   float(w) if w is not None else 0.0)
    g = FrameGraph(size_table=sizes or None)
    if not res.axes:
        return g
    ends = np.array([q for a, b, _ in res.axes for q in (a, b)])
    _, lab = _cluster_count(ends, float(cluster_tol_mm))
    for l in sorted(set(lab.tolist())):
        g.add_node(f"N{l}", tuple(ends[lab == l].mean(axis=0)))
    for i, (od, (_, _, w)) in enumerate(zip(res.od_mm, res.axes)):
        a, b = f"N{lab[2 * i]}", f"N{lab[2 * i + 1]}"
        if a != b:
            key = f"STEP {od:g}x{(w if w is not None else 0):g}"
            g.add_tube(f"T{i + 1:02d}", a, b, size=key)
    return g


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
                small_radii_mm=(), od_mm=None) -> dict:
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
    sizes = {}
    for i, a in enumerate(axes):
        w = a[2] if len(a) > 2 else None
        walls[w] = walls.get(w, 0) + 1
        if od_mm:
            k = (float(od_mm[i]), w)
            sizes[k] = sizes.get(k, 0) + 1
    out = {"tubes": len(p), "straight_m": straight / 1000.0,
           "bends": len(bends), "bend_arc_m": arc / 1000.0,
           "total_m": (straight + arc) / 1000.0, "nodes": nodes,
           "walls_mm": walls, "sizes_mm": sizes}
    if vertices is not None and len(vertices):
        V = np.asarray(vertices, float)
        inside = np.zeros(len(V), bool)
        radii = ([0.5 * float(x) for x in od_mm] if od_mm
                 else [outer_radius_mm] * len(p))
        for (a, b), rad in zip(p, radii):
            d = b - a
            L = np.linalg.norm(d)
            if L < 1e-9:
                continue
            u = d / L
            s = (V - a) @ u
            perp = np.linalg.norm((V - a) - np.outer(s, u), axis=1)
            inside |= (s >= -1e-6) & (s <= L + 1e-6) & \
                (perp <= rad + 1e-3)
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


# =========================================================================== #
#  Design review — the checks a reviewer asks about, in one table
# =========================================================================== #
#: Starting ranges only (typical FSAE practice). Every team should edit them;
#: the UI exposes this table for exactly that.
DEFAULT_REVIEW_LIMITS = [
    {"key": "caster_deg", "check": "Caster", "unit": "deg", "lo": 2.0, "hi": 8.0,
     "why": "Sets self-centring and steering weight; negative caster makes the "
            "car wander.",
     "fix": "Move the upper ball joint rearward relative to the lower (or bound "
            "the outboard x-coordinates)."},
    {"key": "kpi_deg", "check": "Kingpin inclination", "unit": "deg",
     "lo": 0.0, "hi": 10.0,
     "why": "High KPI adds camber loss with steer and jacking.",
     "fix": "Move the upper ball joint outboard or the lower inboard."},
    {"key": "scrub_mm", "check": "Scrub radius", "unit": "mm", "lo": -5.0,
     "hi": 30.0,
     "why": "Large scrub feeds braking and bump forces into the steering.",
     "fix": "Move the kingpin axis toward the contact patch (upright/offset)."},
    {"key": "camber_gain_deg_per_mm", "check": "Camber gain", "unit": "deg/mm",
     "lo": -0.08, "hi": -0.005,
     "why": "Negative gain keeps the outside tyre upright as the body rolls.",
     "fix": "Make the upper arm shorter or more inclined than the lower."},
    {"key": "bump_steer_abs", "check": "Bump steer (magnitude)",
     "unit": "deg/mm", "lo": 0.0, "hi": 0.01,
     "why": "Toe change over bumps makes the car dart and costs tyre life.",
     "fix": "Put the tie-rod inner on the line of the wishbone instant axis."},
    {"key": "toe_change_deg", "check": "Toe change over travel",
     "unit": "deg", "lo": 0.0, "hi": 0.10,
     "why": "The whole-travel toe range the driver feels.",
     "fix": "Tie-rod inner height and length (see bump steer)."},
    {"key": "rc_height_mm", "check": "Roll-centre height", "unit": "mm",
     "lo": 0.0, "hi": 100.0,
     "why": "Sets how much lateral load goes through the links vs the springs.",
     "fix": "Change the front-view angle of the arms."},
    {"key": "rc_migration_abs", "check": "Roll-centre migration (magnitude)",
     "unit": "mm/mm", "lo": 0.0, "hi": 1.0,
     "why": "A roll centre that moves a lot makes the balance change "
            "mid-corner. Not a solver channel — nothing bounds it unless you do.",
     "fix": "Tighten the RC-height band or make the arms more parallel."},
    {"key": "anti_pct", "check": "Anti-dive / anti-squat", "unit": "%",
     "lo": 0.0, "hi": 50.0,
     "why": "Too much stiffens the car in braking/drive; negative adds "
            "pitch.",
     "fix": "Tilt the wishbone pivot axes in side view."},
    {"key": "joints_in_rim", "check": "Ball joints inside the rim", "unit": "",
     "lo": 1.0, "hi": 1.0,
     "why": "The solver has no wheel: a joint outside the rim is unbuildable.",
     "fix": "Move the ball joints toward the wheel centre."},
    {"key": "worst_fos", "check": "Worst link factor of safety", "unit": "",
     "lo": 1.5, "hi": 1e9,
     "why": "Below this a link can yield or buckle in the screened cases.",
     "fix": "Bigger tube, shorter link, or reduce the governing load."},
    {"key": "steering_ratio_needed", "check": "Steering ratio needed for the "
     "torque target", "unit": ":1", "lo": 0.0, "hi": 6.0,
     "why": "Above ~6:1 the steering is slow; the torque target then needs "
            "less caster/trail or assistance.",
     "fix": "Reduce caster (trail) or scrub, or accept a slower rack."},
    {"key": "stroke_used", "check": "Damper stroke used", "unit": "fraction",
     "lo": 0.0, "hi": 0.8,
     "why": "Leave stroke for kerbs and bottoming protection.",
     "fix": "Lower the motion ratio or use a longer-stroke damper."},
    {"key": "mr_solved", "check": "Motion ratio solved from a linkage",
     "unit": "", "lo": 1.0, "hi": 1.0,
     "why": "An assumed motion ratio makes every roll-stiffness number "
            "provisional.",
     "fix": "Add pushrod and rocker points to the corner."},
    {"key": "toe_band_share", "check": "Compliance steer, share of toe band",
     "unit": "fraction", "lo": 0.0, "hi": 0.5,
     "why": "Deflection under load uses the same band as build scatter.",
     "fix": "Stiffer tie rod / brackets, or a wider toe band."},
    {"key": "wheelbase_margin_mm", "check": "Wheelbase margin to rules",
     "unit": "mm", "lo": 0.0, "hi": 1e9,
     "why": "Below the rules minimum the car is not legal.",
     "fix": "Move the axle stations apart."},
    {"key": "ground_clearance_mm", "check": "Static clearance under frame",
     "unit": "mm", "lo": 25.0, "hi": 1e9,
     "why": "Too little clearance and the frame strikes over kerbs.",
     "fix": "Raise ride height (ground plane) or the lowest member."},
]


def design_review(values: dict, limits=None, margin: float = 0.10) -> list:
    """Compare each available value (units as named in the limit rows: deg,
    mm, deg/mm, %, dimensionless fractions) with its range.

    Status is "pass" inside [lo, hi], "watch" outside by at most ``margin``
    of the span (or of the bound for one-sided ranges), "fail" beyond, and
    "n/a" when the value is missing. Boolean checks use lo = hi = 1.
    """
    out = []
    for lim in (limits if limits is not None else DEFAULT_REVIEW_LIMITS):
        k = lim["key"]
        v = values.get(k)
        row = dict(lim)
        row["value"] = v
        if v is None or (isinstance(v, float) and math.isnan(v)):
            row["status"] = "n/a"
            out.append(row)
            continue
        v = float(v)
        lo, hi = float(lim["lo"]), float(lim["hi"])
        if lo <= v <= hi:
            row["status"] = "pass"
        elif lo == hi:
            row["status"] = "fail"
        else:
            finite = [b for b in (lo, hi) if abs(b) < 1e8]
            span = (hi - lo) if len(finite) == 2 else max(abs(finite[0]), 1.0)
            miss = (lo - v) if v < lo else (v - hi)
            row["status"] = "watch" if miss <= margin * span else "fail"
        out.append(row)
    return out


def declared_mr_summary(mr_droop: float, mr_static: float, mr_bump: float,
                        spring_rate_N_mm: float | None = None,
                        sprung_corner_mass_kg: float | None = None) -> dict:
    """Actuation summary from a DECLARED motion-ratio curve (dimensionless,
    at full droop / ride height / full bump) when no rocker is defined:
    rate character, motion-ratio and wheel-rate spread (%), and — given coil
    rate (N/mm) and sprung corner mass (kg) — ride frequency (Hz) at each."""
    mr = [float(mr_droop), float(mr_static), float(mr_bump)]
    wr = [m ** 2 for m in mr]
    out = {"mr_droop": mr[0], "mr_static": mr[1], "mr_bump": mr[2],
           "rate_character": ("rising" if mr[2] > mr[1] > mr[0]
                              else "falling" if mr[2] < mr[1] < mr[0]
                              else "peaks/dips at ride height"),
           "mr_spread_pct": 100 * (max(mr) - min(mr)) / min(mr),
           "wheel_rate_spread_pct": 100 * (max(wr) - min(wr)) / min(wr)}
    if spring_rate_N_mm and sprung_corner_mass_kg:
        f = [ride_frequency_hz(spring_rate_N_mm, m, sprung_corner_mass_kg)
             for m in mr]
        out.update(ride_hz_droop=f[0], ride_hz_static=f[1], ride_hz_bump=f[2])
    return out
