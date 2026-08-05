# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Simulation handoff — the layer between a KinematiK section and a solver.

The plain DXF export answers "what shape is it". This module answers "what IS
it", which is the part a member otherwise re-derives by hand at the far end of
the handoff, usually at 1 a.m., usually wrong.

Four artefacts, one vocabulary:

  * a DXF whose entities carry their ROLE on named layers and in XDATA, so a
    bolt hole arrives as a bolt hole and a motor register does not,
  * a mesh shortlist sized from the section's own thinnest feature rather than
    a fixed millimetre value someone remembered from another part,
  * a study spec whose constraints and loads point at those same names,
  * a manifest tying the three together, embedded INSIDE the DXF as well as
    shipped beside it, because sidecars get separated from their files.

Three invariants this module exists to hold:

  1. GEOMETRY IS MILLIMETRES. Always, whatever the UI is displaying. The old
     exporter scaled with the display setting, which is invisible to a human
     reading a drawing and a silent 25.4x error to anything automated.
  2. GLOBAL MESH SIZE FOLLOWS MATERIAL THINNESS, never hole diameter. A hole
     drives LOCAL refinement only. Conflating the two is how a 3 mm hole
     dictates a 200k-element mesh on a plate that needed 5 mm elements.
  3. A MISSING NUMBER STAYS MISSING. An undeclared load is null and marked
     required; it never quietly becomes a guess. Starter values stay labelled
     as starter values.

Stdlib only, by design: this must stay importable and unit-testable with no
numpy, no scipy, and no Streamlit.

Specified by tests/test_sim_handoff.py — read that first if you change
anything here.
"""
from __future__ import annotations

import hashlib
import io
import json
import math
import re
import zipfile
from dataclasses import dataclass, field

# --------------------------------------------------------------------------- #
#  Contract identifiers
# --------------------------------------------------------------------------- #
SCHEMA = "kinematik.sim-handoff/v1"

#: XDATA written under an undeclared APPID is invalid and silently dropped by
#: some readers — the roles would vanish without anything erroring. The APPID
#: table entry and this name must always agree.
XDATA_APPID = "KINEMATIK"

LAYER_OUTER = "KK_OUTER"
LAYER_HOLE = "KK_HOLE"
LAYER_CAVITY = "KK_CAVITY"
LAYER_ANNOTATION = "KK_NOTE"

ROLE_OUTER = "outer_profile"
ROLE_CAVITY = "cavity"
ROLE_HOLE_BOLT = "bolt_hole"
ROLE_HOLE_BORE = "bore"

#: Named selections. Every constraint and load target must resolve to one of
#: these, or to a documented placeholder the member picks themselves.
NS_BODY = "KK_BODY"
NS_OUTER_EDGE = "KK_OUTER_EDGE"
NS_BORE = "KK_BORE"

DISCLAIMER = (
    "Screening-level pre-validation. Sizes, loads and material values here are "
    "starting points derived from declared KinematiK inputs, not a validated "
    "analysis. Confirm every number against your own certs, test data and "
    "solver before any of it reaches a design decision."
)

# --------------------------------------------------------------------------- #
#  Fastener reference data
# --------------------------------------------------------------------------- #
#  (nominal, major_d_mm, tensile_stress_area_mm2, {fit: clearance_hole_d_mm})
#  Clearance holes per ISO 273. Stress areas per ISO 898-1.
_THREADS = (
    ("M3", 3.0, 5.03, {"close": 3.2, "medium": 3.4, "free": 3.6}),
    ("M4", 4.0, 8.78, {"close": 4.3, "medium": 4.5, "free": 4.8}),
    ("M5", 5.0, 14.2, {"close": 5.3, "medium": 5.5, "free": 5.8}),
    ("M6", 6.0, 20.1, {"close": 6.4, "medium": 6.6, "free": 7.0}),
    ("M8", 8.0, 36.6, {"close": 8.4, "medium": 9.0, "free": 10.0}),
    ("M10", 10.0, 58.0, {"close": 10.5, "medium": 11.0, "free": 12.0}),
    ("M12", 12.0, 84.3, {"close": 13.0, "medium": 13.5, "free": 14.5}),
    ("M14", 14.0, 115.0, {"close": 15.0, "medium": 15.5, "free": 16.5}),
    ("M16", 16.0, 157.0, {"close": 17.0, "medium": 17.5, "free": 18.5}),
)

#: Proof strength, MPa (ISO 898-1). Preload is quoted against proof, not UTS.
_GRADE_PROOF_MPA = {"8.8": 640.0, "10.9": 830.0, "12.9": 970.0,
                    "grade5": 634.0, "grade8": 896.0}

#: A hole this far from a tabulated clearance size is not a clearance hole.
_CLEARANCE_TOL_MM = 0.25

# --------------------------------------------------------------------------- #
#  Starter material — clearly labelled as such everywhere it surfaces
# --------------------------------------------------------------------------- #
_MATERIALS = {
    "suspension": dict(name="AISI 4130 normalised", E_MPa=205000.0, nu=0.29,
                       rho_kg_m3=7850.0, yield_MPa=460.0),
    "chassis": dict(name="AISI 4130 normalised", E_MPa=205000.0, nu=0.29,
                    rho_kg_m3=7850.0, yield_MPa=460.0),
    "powertrain": dict(name="6061-T6 aluminium", E_MPa=68900.0, nu=0.33,
                       rho_kg_m3=2700.0, yield_MPa=276.0),
    "electrics": dict(name="6061-T6 aluminium", E_MPa=68900.0, nu=0.33,
                      rho_kg_m3=2700.0, yield_MPa=276.0),
    "aerodynamics": dict(name="Carbon/epoxy laminate (quasi-isotropic)",
                         E_MPa=45000.0, nu=0.31, rho_kg_m3=1550.0,
                         yield_MPa=400.0),
    "cooling": dict(name="3003-H14 aluminium", E_MPa=68900.0, nu=0.33,
                    rho_kg_m3=2730.0, yield_MPa=145.0),
    "brakes": dict(name="AISI 4130 normalised", E_MPa=205000.0, nu=0.29,
                   rho_kg_m3=7850.0, yield_MPa=460.0),
}
_MATERIAL_DEFAULT = dict(name="6061-T6 aluminium", E_MPa=68900.0, nu=0.33,
                         rho_kg_m3=2700.0, yield_MPa=276.0)


# --------------------------------------------------------------------------- #
#  Small geometric helpers (stdlib only — no numpy in this layer)
# --------------------------------------------------------------------------- #
def _pts(raw) -> list:
    """Coerce anything point-list-shaped into [(float, float), ...]."""
    out = []
    for p in (raw or []):
        try:
            x, y = float(p[0]), float(p[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            out.append((x, y))
    return out


def _area(pts) -> float:
    """Shoelace area, unsigned."""
    n = len(pts)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        s += x0 * y1 - x1 * y0
    return abs(s) / 2.0


def _centroid(pts):
    if not pts:
        return (0.0, 0.0)
    return (sum(p[0] for p in pts) / len(pts),
            sum(p[1] for p in pts) / len(pts))


def _bbox_of(pts):
    if not pts:
        return (0.0, 0.0, 0.0, 0.0)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def _dist_point_segment(p, a, b) -> float:
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _dist_point_polygon(p, pts) -> float:
    if len(pts) < 2:
        return math.inf
    return min(_dist_point_segment(p, pts[i], pts[(i + 1) % len(pts)])
               for i in range(len(pts)))


def _floor_to(value: float, places: int = 3) -> float:
    """Round DOWN.

    Mesh sizes are rounded down, never to-nearest: rounding a global size UP
    can drop the element count across a thin wall below the level's own
    declared minimum, which is the one thing the ladder promises.
    """
    if not math.isfinite(value) or value <= 0:
        return 0.0
    f = 10.0 ** places
    return math.floor(value * f) / f


def _ascii(text) -> str:
    """The app's strings are full of ⌀, · and em dashes; R12 is 7-bit."""
    s = str(text)
    for bad, good in (("⌀", "dia "), ("×", "x"), ("·", "."), ("—", "-"),
                      ("–", "-"), ("’", "'"), ("“", '"'), ("”", '"'),
                      ("°", "deg"), ("±", "+/-"), ("²", "^2"), ("³", "^3"),
                      ("µ", "u"), ("Ω", "ohm")):
        s = s.replace(bad, good)
    return s.encode("ascii", "replace").decode("ascii")


def _slug(text) -> str:
    s = re.sub(r"[^a-z0-9]+", "_", _ascii(text).lower()).strip("_")
    return (s or "section")[:48]


def _num(value) -> float | None:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


# --------------------------------------------------------------------------- #
#  Section model
# --------------------------------------------------------------------------- #
@dataclass
class Loop:
    """A closed profile — the outer boundary or an internal cavity."""
    pts: list
    role: str = ROLE_OUTER
    closed: bool = True

    @property
    def area_mm2(self) -> float:
        return _area(self.pts)

    @property
    def bbox(self):
        return _bbox_of(self.pts)


@dataclass
class Hole:
    """A circular feature. Its ROLE is the whole point of this class."""
    c: tuple
    d_mm: float
    role: str = ROLE_HOLE_BOLT
    group: str | None = None


@dataclass
class FastenerGroup:
    """A recognised pattern of like-sized holes."""
    id: str
    pattern: str                 # circular / rectangular / irregular
    count: int
    d_mm: float
    pcd_mm: float = 0.0
    pitch_x_mm: float = 0.0
    pitch_y_mm: float = 0.0
    centre: tuple = (0.0, 0.0)
    bolt: dict = field(default_factory=dict)

    @property
    def selection(self) -> str:
        return f"KK_BOLT_{self.id}"

    def as_dict(self) -> dict:
        return {"id": self.id, "pattern": self.pattern, "count": self.count,
                "hole_d_mm": round(self.d_mm, 3),
                "pcd_mm": round(self.pcd_mm, 3),
                "pitch_x_mm": round(self.pitch_x_mm, 3),
                "pitch_y_mm": round(self.pitch_y_mm, 3),
                "named_selection": self.selection, "bolt": self.bolt}


@dataclass
class Section:
    """A 2D section plus the intent needed to simulate it."""
    label: str = ""
    subsystem: str = ""
    outer: Loop | None = None
    cavities: list = field(default_factory=list)
    holes: list = field(default_factory=list)
    fastener_groups: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)
    wall_mm: float = 0.0
    extrude_mm: float = 0.0
    extrude_source: str = ""
    frame_tag: str = ""

    # -- derived ----------------------------------------------------------- #
    @property
    def bbox(self):
        pts = list(self.outer.pts) if self.outer else []
        for cav in self.cavities:
            pts += list(cav.pts)
        for h in self.holes:
            r = h.d_mm / 2.0
            pts += [(h.c[0] - r, h.c[1] - r), (h.c[0] + r, h.c[1] + r)]
        return _bbox_of(pts)

    @property
    def width_mm(self) -> float:
        x0, _, x1, _ = self.bbox
        return x1 - x0

    @property
    def height_mm(self) -> float:
        _, y0, _, y1 = self.bbox
        return y1 - y0

    @property
    def max_dim_mm(self) -> float:
        return max(self.width_mm, self.height_mm, 0.0)

    @property
    def area_mm2(self) -> float:
        a = self.outer.area_mm2 if self.outer else 0.0
        a -= sum(c.area_mm2 for c in self.cavities)
        a -= sum(math.pi * (h.d_mm / 2.0) ** 2 for h in self.holes)
        return max(a, 0.0)

    @property
    def bore(self) -> Hole | None:
        for h in self.holes:
            if h.role == ROLE_HOLE_BORE:
                return h
        return None

    def digest(self) -> str:
        """Stable over the GEOMETRY only.

        Not the label, not the metadata, and emphatically not the clock: two
        exports of the same shape must produce the same digest, so a manifest
        and a DXF can be checked against each other after they have been
        separated, renamed and emailed around.
        """
        parts = []
        if self.outer:
            parts.append("O:" + ";".join(f"{x:.4f},{y:.4f}"
                                         for x, y in self.outer.pts))
        for cav in self.cavities:
            parts.append("C:" + ";".join(f"{x:.4f},{y:.4f}"
                                         for x, y in cav.pts))
        for h in sorted(self.holes, key=lambda z: (z.c[0], z.c[1], z.d_mm)):
            parts.append(f"H:{h.c[0]:.4f},{h.c[1]:.4f},{h.d_mm:.4f},{h.role}")
        blob = "|".join(parts).encode("utf-8")
        return hashlib.sha256(blob).hexdigest()[:16]


# --------------------------------------------------------------------------- #
#  Bolt inference
# --------------------------------------------------------------------------- #
def infer_bolt(hole_d_mm: float, grade: str = "8.8",
               preload_fraction: float = 0.65,
               nut_factor: float = 0.20) -> dict:
    """Read a thread back from the clearance hole that was drilled for it.

    Returns ``nominal=None`` when the diameter matches no tabulated clearance
    hole — a 75 mm motor register is not an M75 bolt, and handing it a
    pretension is exactly the failure this layer exists to prevent.

    ``install_torque_nm`` is always consistent with ``pretension_n`` via
    T = K.F.d. If those two ever disagree, the number a member torques to is
    not the number the model was preloaded with.
    """
    d = _num(hole_d_mm) or 0.0
    best = None
    for nominal, major_d, area, fits in _THREADS:
        for fit, clear_d in fits.items():
            err = abs(d - clear_d)
            if err <= _CLEARANCE_TOL_MM and (best is None or err < best[0]):
                best = (err, nominal, major_d, area, fit)

    if best is None:
        return {"nominal": None, "fit": None, "hole_d_mm": round(d, 3),
                "note": ("no tabulated clearance hole at this diameter - "
                         "treated as a bore or a non-fastener feature, and "
                         "given no thread, preload or torque")}

    _, nominal, major_d, area, fit = best
    proof = _GRADE_PROOF_MPA.get(str(grade), _GRADE_PROOF_MPA["8.8"])
    frac = _num(preload_fraction) or 0.65
    k = _num(nut_factor) or 0.20

    pretension = round(frac * proof * area, 1)                    # N
    torque = round(k * pretension * (major_d / 1000.0), 2)        # N.m

    return {
        "nominal": nominal, "fit": fit, "hole_d_mm": round(d, 3),
        "nominal_d_mm": major_d, "stress_area_mm2": area,
        "grade": str(grade), "proof_MPa": proof,
        "preload_fraction": frac, "nut_factor": k,
        "pretension_n": pretension, "install_torque_nm": torque,
        "note": ("preload is a STARTING value - K (the nut factor) is where "
                 "the real scatter lives, so measure or specify yours"),
    }


# --------------------------------------------------------------------------- #
#  Pattern recognition
# --------------------------------------------------------------------------- #
def _classify_pattern(centres) -> dict:
    """Circular / rectangular / irregular, and the dimensions a drawing uses.

    A rectangular pattern's four corners DO lie on a circle, so equal radii
    alone is not enough — a bolt circle also has uniform angular spacing. No
    drawing dimensions four corner holes by PCD, so quoting one would be a
    number nobody could check against the part.
    """
    n = len(centres)
    cx, cy = _centroid(centres)
    radii = [math.hypot(x - cx, y - cy) for x, y in centres]
    mean_r = sum(radii) / n if n else 0.0

    if n >= 3 and mean_r > 1e-9:
        r_tol = max(0.05, 0.005 * mean_r)
        if max(radii) - min(radii) <= r_tol:
            angles = sorted(math.degrees(math.atan2(y - cy, x - cx)) % 360.0
                            for x, y in centres)
            gaps = [(angles[(i + 1) % n] - angles[i]) % 360.0
                    for i in range(n)]
            if max(gaps) - min(gaps) <= 1.0:          # uniform spacing
                return {"pattern": "circular", "pcd_mm": 2.0 * mean_r,
                        "pitch_x_mm": 0.0, "pitch_y_mm": 0.0,
                        "centre": (cx, cy)}

    xs = sorted({round(x, 3) for x, _ in centres})
    ys = sorted({round(y, 3) for _, y in centres})
    if n == 4 and len(xs) == 2 and len(ys) == 2:
        return {"pattern": "rectangular", "pcd_mm": 0.0,
                "pitch_x_mm": xs[1] - xs[0], "pitch_y_mm": ys[1] - ys[0],
                "centre": (cx, cy)}

    x0, y0, x1, y1 = _bbox_of(centres)
    return {"pattern": "irregular", "pcd_mm": 0.0,
            "pitch_x_mm": x1 - x0, "pitch_y_mm": y1 - y0, "centre": (cx, cy)}


def pattern_phrase(group: FastenerGroup) -> str:
    """One human sentence describing a recognised pattern."""
    bolt = group.bolt or {}
    thread = bolt.get("nominal") or f"{group.d_mm:g} mm clearance"
    fit = f" ({bolt['fit']} fit)" if bolt.get("fit") else ""
    if group.pattern == "circular":
        where = f"on a {group.pcd_mm:.1f} mm PCD"
    elif group.pattern == "rectangular":
        where = (f"on a {group.pitch_x_mm:.1f} x {group.pitch_y_mm:.1f} mm "
                 "rectangular pitch")
    else:
        where = "in an irregular pattern"
    return f"{group.count} x {thread}{fit} {where}"


def _resolve_roles(sec: Section) -> None:
    """Assign every hole a role and gather like holes into fastener groups."""
    if not sec.holes:
        return

    buckets: dict = {}
    for h in sec.holes:
        buckets.setdefault(round(h.d_mm, 2), []).append(h)

    grouped_d = []
    idx = 0
    for d_key in sorted(buckets):
        members = buckets[d_key]
        if len(members) < 3:                 # not a pattern; decide later
            continue
        idx += 1
        info = _classify_pattern([m.c for m in members])
        gid = f"G{idx}"
        grp = FastenerGroup(id=gid, count=len(members), d_mm=d_key,
                            bolt=infer_bolt(d_key), **info)
        sec.fastener_groups.append(grp)
        grouped_d.append(d_key)
        for m in members:
            m.role = ROLE_HOLE_BOLT
            m.group = gid

    # Leftovers: a large, central, ungrouped hole is a bore, not a bolt hole.
    # A motor register must never be handed a pretension.
    origin = _centroid(sec.outer.pts) if sec.outer else (0.0, 0.0)
    centre_tol = max(1.0, 0.02 * sec.max_dim_mm)
    biggest_grouped = max(grouped_d) if grouped_d else 0.0
    for h in sec.holes:
        if h.group is not None:
            continue
        central = math.hypot(h.c[0] - origin[0], h.c[1] - origin[1]) <= centre_tol
        dominant = h.d_mm > 1.5 * biggest_grouped
        if central and dominant:
            h.role, h.group = ROLE_HOLE_BORE, None
        else:
            h.role, h.group = ROLE_HOLE_BOLT, None


# --------------------------------------------------------------------------- #
#  Section construction
# --------------------------------------------------------------------------- #
#: Meta keys that declare a real extrusion depth. "wall" is deliberately NOT
#: here — a box's wall thickness is not its extrusion depth, and conflating
#: them silently halves the modelled part.
_EXTRUDE_KEY_RE = re.compile(r"thick|thk|depth|extrud|gauge", re.I)

#: Meta keys whose value is a KinematiK starter, not a measured input.
_STARTER_KEY_RE = re.compile(r"plate size|outline|blank|envelope|starter|"
                             r"assumed|default", re.I)


def _extrusion(sec: Section) -> None:
    for key, val in (sec.meta or {}).items():
        if _EXTRUDE_KEY_RE.search(str(key)):
            v = _num(val)
            if v and v > 0:
                sec.extrude_mm = v
                sec.extrude_source = f"declared - {_ascii(key)}"
                return
    # No declared thickness. Scale the starter with the part so the mesh ladder
    # stays a RATIO rather than a fixed millimetre value that only suits parts
    # of one size.
    starter = 0.05 * sec.max_dim_mm
    sec.extrude_mm = min(60.0, max(1.5, starter))
    sec.extrude_source = ("assumed - 5% of the section's largest dimension "
                          "(starter, not a declared thickness)")


def section_from_candidate(subsystem_key, candidate) -> Section:
    """Build a Section from one of the app's own candidate dicts.

    Tolerant by contract: empty, partial and legacy ``profile_mm`` candidates
    must all produce a usable Section rather than an exception. An export that
    raises is an export nobody gets.
    """
    cand = dict(candidate or {})
    meta = dict(cand.get("meta") or {})
    sec = Section(label=_ascii(cand.get("label", "") or "section"),
                  subsystem=str(subsystem_key or "").strip() or "general",
                  meta=meta,
                  notes=[_ascii(n) for n in (cand.get("notes") or [])])

    kwargs = dict(cand.get("dxf_kwargs") or {})
    raw_lines = list(kwargs.get("polylines") or [])
    if not raw_lines and cand.get("profile_mm"):
        raw_lines = [{"pts": cand["profile_mm"], "closed": True}]   # legacy

    loops = []
    for pl in raw_lines:
        if isinstance(pl, dict):
            pts, closed = _pts(pl.get("pts")), bool(pl.get("closed", True))
        else:
            pts, closed = _pts(pl), True
        if pts:
            loops.append(Loop(pts=pts, closed=closed))

    if loops:
        loops.sort(key=lambda lp: lp.area_mm2, reverse=True)
        sec.outer = loops[0]
        sec.outer.role = ROLE_OUTER
        for lp in loops[1:]:
            lp.role = ROLE_CAVITY
            sec.cavities.append(lp)

    for c in (kwargs.get("circles") or []):
        try:
            cx, cy = float(c["c"][0]), float(c["c"][1])
            r = float(c["r"])
        except (TypeError, ValueError, KeyError, IndexError):
            continue
        if r > 0 and math.isfinite(cx) and math.isfinite(cy):
            sec.holes.append(Hole(c=(cx, cy), d_mm=2.0 * r))

    # A cavity is a wall, not a hole: the thinnest gap between the outer
    # boundary and the largest cavity is the feature that drives the mesh.
    if sec.outer and sec.cavities:
        ox0, oy0, ox1, oy1 = sec.outer.bbox
        gaps = []
        for cav in sec.cavities:
            cx0, cy0, cx1, cy1 = cav.bbox
            gaps += [cx0 - ox0, ox1 - cx1, cy0 - oy0, oy1 - cy1]
        positive = [g for g in gaps if g > 1e-9]
        if positive:
            sec.wall_mm = min(positive)

    _resolve_roles(sec)
    _extrusion(sec)
    return sec


# --------------------------------------------------------------------------- #
#  Mesh shortlist
# --------------------------------------------------------------------------- #
@dataclass
class MeshLevel:
    name: str
    global_size_mm: float
    local_size_mm: float
    elems_across_thin: int
    growth_rate: float
    est_dof: int
    quality: dict = field(default_factory=dict)
    purpose: str = ""

    def as_dict(self) -> dict:
        return {"name": self.name, "global_size_mm": self.global_size_mm,
                "local_size_mm": self.local_size_mm,
                "elems_across_thin": self.elems_across_thin,
                "growth_rate": self.growth_rate, "est_dof": self.est_dof,
                "quality": dict(self.quality), "purpose": self.purpose}


@dataclass
class MeshShortlist:
    levels: list
    basis: dict
    convergence: dict

    def as_dict(self) -> dict:
        return {"levels": [lv.as_dict() for lv in self.levels],
                "basis": dict(self.basis),
                "convergence": dict(self.convergence)}


#: (name, elements across the thinnest feature, growth, max skewness, hole
#:  refinement divisor, purpose)
_LEVELS = (
    ("screening", 2, 1.30, 0.90, 8.0,
     "shape and load-path sanity; cheap enough to run on every change"),
    ("production", 3, 1.20, 0.85, 12.0,
     "the level results are reported from"),
    ("convergence", 4, 1.15, 0.80, 16.0,
     "the refinement production is checked against"),
)


def _min_ligament_mm(sec: Section) -> float:
    """Thinnest strip of material between a hole and anything else."""
    vals = []
    if sec.outer and len(sec.outer.pts) >= 2:
        for h in sec.holes:
            d = _dist_point_polygon(h.c, sec.outer.pts) - h.d_mm / 2.0
            if d > 1e-9:
                vals.append(d)
    for i, a in enumerate(sec.holes):
        for b in sec.holes[i + 1:]:
            d = (math.hypot(a.c[0] - b.c[0], a.c[1] - b.c[1])
                 - a.d_mm / 2.0 - b.d_mm / 2.0)
            if d > 1e-9:
                vals.append(d)
    return min(vals) if vals else 0.0


def mesh_shortlist(sec: Section) -> MeshShortlist:
    """Three levels sized from the section's own thinnest feature.

    The global size follows MATERIAL THINNESS. Hole diameter drives the LOCAL
    size only. Shrinking the holes on a plate leaves more material, not less,
    so it must never make the global mesh finer.
    """
    candidates = []
    if sec.wall_mm > 0:
        candidates.append((sec.wall_mm, "cavity wall"))
    if sec.extrude_mm > 0:
        candidates.append((sec.extrude_mm, "extrusion depth"))
    lig = _min_ligament_mm(sec)
    if lig > 0:
        candidates.append((lig, "ligament between holes / to the free edge"))

    if candidates:
        min_feature_mm, min_feature_from = min(candidates, key=lambda z: z[0])
    else:
        min_feature_mm, min_feature_from = 1.0, "fallback (no usable geometry)"

    hole_ds = [h.d_mm for h in sec.holes if h.d_mm > 0]
    min_hole = min(hole_ds) if hole_ds else 0.0

    planar = max(sec.max_dim_mm, 1e-9)
    thin = (sec.extrude_mm / planar) < 0.05 if sec.extrude_mm > 0 else False
    bolted = bool(sec.fastener_groups) or any(
        h.role == ROLE_HOLE_BOLT for h in sec.holes)

    if thin and not bolted:
        method = "shell (mid-surface, second-order)"
        rationale = ("the section is thin relative to its plan dimensions and "
                     "carries no fastener, so a mid-surface shell captures it "
                     "at a fraction of the cost.")
        shell_alt = ""
    else:
        method = "solid (second-order tetrahedra)"
        if bolted:
            rationale = ("solid, because the part is bolted: a shell has no "
                         "through-thickness, so it cannot carry a pretension, "
                         "a bearing stress at a hole, or a washer footprint.")
        else:
            rationale = ("solid, because the section is not thin enough for a "
                         "mid-surface shell to be defensible.")
        shell_alt = ("" if bolted else
                     "A shell would be cheaper if the fastener detail is "
                     "removed and only global stiffness is wanted.")

    area = sec.area_mm2
    if area <= 0:
        w, h = sec.width_mm, sec.height_mm
        area = w * h if (w > 0 and h > 0) else 0.0
    volume = max(area * max(sec.extrude_mm, 1e-6), 1e-6)

    levels, prev_dof = [], 0
    for name, across, growth, skew, hole_div, purpose in _LEVELS:
        g = _floor_to(min_feature_mm / across) or 0.001
        local = g / 2.0 if min_hole <= 0 else min(g, min_hole / hole_div)
        local = _floor_to(local) or min(g, 0.001)

        n_elem = volume / (g ** 3)
        dof = int(max(n_elem * 3.0, 1.0))
        dof = max(dof, prev_dof + 1)        # the ladder must always cost more
        prev_dof = dof

        levels.append(MeshLevel(
            name=name, global_size_mm=g, local_size_mm=local,
            elems_across_thin=across, growth_rate=growth, est_dof=dof,
            quality={"skewness_max": skew,
                     "aspect_ratio_max": round(20.0 - 4.0 * len(levels), 1),
                     "orthogonal_quality_min": round(0.10 + 0.05 * len(levels), 2)},
            purpose=purpose))

    prod = levels[1]
    basis = {
        "min_feature_mm": round(min_feature_mm, 3),
        "min_feature_from": min_feature_from,
        "method": method,
        "method_rationale": rationale,
        "defeature_mm": round(max(prod.global_size_mm * 0.5, 0.05), 3),
        "estimate_note": ("DOF figures are order-of-magnitude estimates from "
                          "volume / element size, for picking a level - not a "
                          "solver's own count."),
        "hole_refinement_from_mm": round(min_hole, 3) if min_hole else None,
    }
    if shell_alt:
        basis["shell_alternative"] = shell_alt

    convergence = {
        "compare": ["production", "convergence"],
        "metric": "peak von Mises stress in the governing ligament",
        "accept_change_pct": 5.0,
        "note": ("If the metric moves more than the acceptance band between "
                 "these two levels, the result is not converged - refine "
                 "again. Exclude re-entrant corners and load-application "
                 "points from the comparison: stress at a geometric "
                 "singularity rises without limit as the mesh refines, so it "
                 "never converges and must not be read as a peak stress."),
    }
    return MeshShortlist(levels=levels, basis=basis, convergence=convergence)


def mesh_rows(mesh: MeshShortlist) -> list:
    """Table-shaped view of the ladder, for the UI."""
    return [{
        "Level": lv.name,
        "Global size (mm)": lv.global_size_mm,
        "Local at holes (mm)": lv.local_size_mm,
        "Elements across thinnest": lv.elems_across_thin,
        "Growth rate": lv.growth_rate,
        "Max skewness": lv.quality.get("skewness_max"),
        "Est. DOF": lv.est_dof,
        "Purpose": lv.purpose,
    } for lv in mesh.levels]


# --------------------------------------------------------------------------- #
#  Named selections and the study
# --------------------------------------------------------------------------- #
def named_selections(sec: Section) -> list:
    """Every entity name a constraint or load is allowed to point at.

    A constraint targeting a name nothing defines is the exact failure this
    whole layer exists to prevent, so the study is built only from this list.
    """
    names = [NS_BODY, NS_OUTER_EDGE]
    names += [g.selection for g in sec.fastener_groups]
    if sec.bore is not None:
        names.append(NS_BORE)
    loose = [h for h in sec.holes if h.group is None and h.role == ROLE_HOLE_BOLT]
    if loose:
        names.append("KK_BOLT_LOOSE")
    for i, _ in enumerate(sec.cavities, start=1):
        names.append(f"KK_CAVITY_{i}")
    return names


@dataclass
class Study:
    analysis_type: str
    material: dict
    constraints: list
    load_cases: list
    outputs: list = field(default_factory=list)
    notes: list = field(default_factory=list)

    def as_dict(self) -> dict:
        return {"analysis_type": self.analysis_type,
                "material": dict(self.material),
                "constraints": [dict(c) for c in self.constraints],
                "load_cases": [dict(lc) for lc in self.load_cases],
                "outputs": list(self.outputs), "notes": list(self.notes)}


_LOAD_KEY_RE = re.compile(r"peak load|max load|design load|load \(n\)", re.I)
_TORQUE_KEY_RE = re.compile(r"torque", re.I)


def _meta_number(meta: dict, pattern) -> float | None:
    for key, val in (meta or {}).items():
        if pattern.search(str(key)):
            v = _num(val)
            if v is not None:
                return v
    return None


def study_spec(sec: Section) -> Study:
    """The load cases and constraints, bound to the DXF's own names.

    A load nobody declared stays ``None`` and ``required``. It never becomes a
    plausible-looking number, because a plausible-looking number is the one
    thing nobody re-checks.
    """
    ns = named_selections(sec)
    material = dict(_MATERIALS.get(sec.subsystem, _MATERIAL_DEFAULT))
    material["source"] = "KinematiK starter - confirm against your own cert"

    if sec.fastener_groups:
        held = [g.selection for g in sec.fastener_groups]
        hold_note = "fixed at the recognised fastener pattern"
    else:
        held = [NS_OUTER_EDGE]
        hold_note = ("no fastener pattern was recognised - pick the mounting "
                     "edge yourself before running")

    constraints = [{"name": "mounting_restraint", "type": "fixed_support",
                    "targets": held, "note": hold_note}]

    react = NS_BORE if sec.bore is not None else NS_BODY
    peak = _meta_number(sec.meta, _LOAD_KEY_RE)
    load_cases = [{
        "name": "corner_peak_load",
        "type": "force",
        "magnitude_n": peak,
        "direction": "worst-case in-plane; set the vector from your own case",
        "targets": [react],
        "required": peak is None,
        "note": ("declared in KinematiK" if peak is not None else
                 "NOT DECLARED - the run should stop here rather than "
                 "continue with an assumed value"),
    }]

    torque = _meta_number(sec.meta, _TORQUE_KEY_RE)
    if torque is not None and sec.bore is not None:
        load_cases.append({
            "name": "drive_torque_reaction", "type": "moment",
            "magnitude_nm": torque, "targets": [NS_BORE], "required": False,
            "note": "declared in KinematiK",
        })

    for g in sec.fastener_groups:
        bolt = g.bolt or {}
        if bolt.get("pretension_n"):
            load_cases.append({
                "name": f"bolt_pretension_{g.id.lower()}",
                "type": "bolt_pretension",
                "magnitude_n": bolt["pretension_n"],
                "targets": [g.selection], "required": False,
                "note": (f"starter: {bolt.get('preload_fraction')} of "
                         f"{bolt.get('grade')} proof; install torque "
                         f"{bolt.get('install_torque_nm')} N.m at K="
                         f"{bolt.get('nut_factor')}"),
            })

    # Belt and braces: never emit a target that is not a defined selection.
    for item in constraints + load_cases:
        item["targets"] = [t for t in item.get("targets", []) if t in ns]
        if not item["targets"]:
            item["targets"] = [NS_OUTER_EDGE]

    return Study(
        analysis_type="static_structural",
        material=material,
        constraints=constraints,
        load_cases=load_cases,
        outputs=["von Mises stress", "directional deformation",
                 "bearing stress at each hole", "reaction at the restraint"],
        notes=[("Geometry is millimetres. Forces are newtons. Nothing here is "
                "scaled to a display unit."),
               ("Every target above is a named selection carried on a DXF "
                "layer and in XDATA under " + XDATA_APPID + ".")],
    )


# --------------------------------------------------------------------------- #
#  Trust map — starter values stay separated from measured ones
# --------------------------------------------------------------------------- #
def trust_map(sec: Section) -> dict:
    """Which numbers came from somewhere real, and which KinematiK made up."""
    measured, starter = {}, {}
    for key, val in (sec.meta or {}).items():
        target = starter if _STARTER_KEY_RE.search(str(key)) else measured
        target[str(key)] = val

    if sec.extrude_source.startswith("assumed"):
        starter["extrude_mm"] = sec.extrude_mm
    else:
        measured["extrude_mm"] = sec.extrude_mm

    return {
        "measured": measured,
        "starter": starter,
        "note": ("'starter' values are KinematiK defaults shaped to the part, "
                 "not measurements. Replace them before any result is "
                 "reported, and never quote one as an input."),
    }


# --------------------------------------------------------------------------- #
#  Manifest
# --------------------------------------------------------------------------- #
def _dxf_name(sec: Section) -> str:
    return f"kinematik_{_slug(sec.subsystem)}_{_slug(sec.label)}.dxf"


def handoff_manifest(sec: Section, frame_tag: str = "") -> dict:
    """The document that makes the other three artefacts one handoff."""
    mesh = mesh_shortlist(sec)
    study = study_spec(sec)
    x0, y0, x1, y1 = sec.bbox

    return {
        "schema": SCHEMA,
        "generated_by": "KinematiK",
        "units": {
            "length": "mm", "force": "N", "stress": "MPa", "moment": "N.m",
            "mass": "kg", "temperature": "degC", "angle": "deg",
            "canonical": True,
            "note": ("Geometry is millimetres regardless of what the app was "
                     "displaying. Display units never reach this file."),
        },
        "part": {
            "label": sec.label,
            "subsystem": sec.subsystem,
            "geometry_digest": sec.digest(),
            "dxf": _dxf_name(sec),
            "frame": _ascii(frame_tag or sec.frame_tag or "KinematiK vehicle frame"),
        },
        "geometry": {
            "bbox_mm": [round(x0, 4), round(y0, 4), round(x1, 4), round(y1, 4)],
            "outer_area_mm2": round(sec.outer.area_mm2, 3) if sec.outer else 0.0,
            "net_area_mm2": round(sec.area_mm2, 3),
            "wall_mm": round(sec.wall_mm, 4),
            "extrude_mm": round(sec.extrude_mm, 4),
            "extrude_source": sec.extrude_source,
            "cavities": len(sec.cavities),
            "holes": [{"x_mm": round(h.c[0], 4), "y_mm": round(h.c[1], 4),
                       "d_mm": round(h.d_mm, 4), "role": h.role,
                       "group": h.group} for h in sec.holes],
            "fastener_groups": [g.as_dict() for g in sec.fastener_groups],
        },
        "named_selections": named_selections(sec),
        "mesh": mesh.as_dict(),
        "study": study.as_dict(),
        "trust": trust_map(sec),
        "layers": {"outer": LAYER_OUTER, "hole": LAYER_HOLE,
                   "cavity": LAYER_CAVITY, "annotation": LAYER_ANNOTATION},
        "xdata_appid": XDATA_APPID,
        "disclaimer": DISCLAIMER,
    }


# --------------------------------------------------------------------------- #
#  DXF writer (R12, 7-bit ASCII)
# --------------------------------------------------------------------------- #
_LAYER_COLOURS = ((LAYER_OUTER, 7), (LAYER_HOLE, 1), (LAYER_CAVITY, 3),
                  (LAYER_ANNOTATION, 4))


def _xdata(out: list, role: str, extra: str = "") -> None:
    """XDATA under the APPID declared in the TABLES section — never a bare one."""
    out += ["1001", XDATA_APPID, "1000", f"role={role}"]
    if extra:
        out += ["1000", _ascii(extra)]


def section_dxf(sec: Section) -> bytes:
    """A DXF that carries its own intent.

    Self-sufficient on purpose: the manifest is embedded as 999 comments at the
    head of the file as well as shipped beside it, because sidecars get
    emailed on their own and dropped into shared folders constantly.
    """
    man = handoff_manifest(sec)
    o: list = []

    def w(code, value):
        o.append(str(code))
        o.append(_ascii(value))

    def wf(code, value):
        o.append(str(code))
        o.append(f"{float(value):.6f}")

    # ---- embedded manifest, first thing in the file ----------------------- #
    w(999, f"KinematiK simulation handoff - {SCHEMA}")
    w(999, "Geometry is MILLIMETRES. Roles are on layers and in XDATA.")
    w(999, "BEGIN KK-MANIFEST-JSON")
    for line in json.dumps(man, indent=1, ensure_ascii=True,
                           default=str).split("\n"):
        w(999, line)
    w(999, "END KK-MANIFEST-JSON")

    # ---- HEADER ----------------------------------------------------------- #
    x0, y0, x1, y1 = sec.bbox
    w(0, "SECTION")
    w(2, "HEADER")
    w(9, "$INSUNITS")
    w(70, 4)                       # 4 = millimetres. Must be the first code 70.
    w(9, "$MEASUREMENT")
    w(70, 1)                       # 1 = metric
    w(9, "$EXTMIN")
    wf(10, x0)
    wf(20, y0)
    wf(30, 0.0)
    w(9, "$EXTMAX")
    wf(10, x1)
    wf(20, y1)
    wf(30, 0.0)
    w(0, "ENDSEC")

    # ---- TABLES ----------------------------------------------------------- #
    w(0, "SECTION")
    w(2, "TABLES")
    w(0, "TABLE")
    w(2, "APPID")
    w(70, 1)
    w(0, "APPID")                  # the entry XDATA below is written under
    w(2, XDATA_APPID)
    w(70, 0)
    w(0, "ENDTAB")
    w(0, "TABLE")
    w(2, "LAYER")
    w(70, len(_LAYER_COLOURS))
    for name, colour in _LAYER_COLOURS:
        w(0, "LAYER")
        w(2, name)
        w(70, 0)
        w(62, colour)
        w(6, "CONTINUOUS")
    w(0, "ENDTAB")
    w(0, "ENDSEC")

    # ---- ENTITIES --------------------------------------------------------- #
    w(0, "SECTION")
    w(2, "ENTITIES")

    def polyline(loop: Loop, layer: str, role: str, extra: str = ""):
        w(0, "POLYLINE")
        w(8, layer)
        w(66, 1)
        w(70, 1 if loop.closed else 0)
        wf(10, 0.0)
        wf(20, 0.0)
        wf(30, 0.0)
        _xdata(o, role, extra)
        for px, py in loop.pts:
            w(0, "VERTEX")
            w(8, layer)
            wf(10, px)
            wf(20, py)
            wf(30, 0.0)
        w(0, "SEQEND")
        w(8, layer)

    if sec.outer:
        polyline(sec.outer, LAYER_OUTER, ROLE_OUTER,
                 f"area_mm2={sec.outer.area_mm2:.3f}")
    for i, cav in enumerate(sec.cavities, start=1):
        polyline(cav, LAYER_CAVITY, ROLE_CAVITY, f"cavity={i}")

    for h in sec.holes:
        w(0, "CIRCLE")
        w(8, LAYER_HOLE)
        wf(10, h.c[0])
        wf(20, h.c[1])
        wf(30, 0.0)
        wf(40, h.d_mm / 2.0)
        bolt = infer_bolt(h.d_mm) if h.role == ROLE_HOLE_BOLT else {}
        extra = f"d_mm={h.d_mm:.3f}"
        if h.group:
            extra += f";group={h.group}"
        if bolt.get("nominal"):
            extra += f";thread={bolt['nominal']};fit={bolt['fit']}"
        _xdata(o, h.role, extra)

    # An annotation entity is always emitted: the note layer must exist even
    # for a degenerate section, or a reader cannot tell an empty drawing from
    # a drawing that lost its notes.
    w(0, "TEXT")
    w(8, LAYER_ANNOTATION)
    wf(10, x0)
    wf(20, y1 + max(6.0, 0.03 * max(sec.max_dim_mm, 1.0)))
    wf(30, 0.0)
    wf(40, max(2.5, 0.02 * max(sec.max_dim_mm, 1.0)))
    w(1, f"{sec.label} | mm | {SCHEMA} | digest {sec.digest()}")
    _xdata(o, "annotation", f"digest={sec.digest()}")

    w(0, "ENDSEC")
    w(0, "EOF")

    # No trailing newline: a DXF is a stream of (code, value) PAIRS, and a
    # trailing blank makes the line count odd, which breaks strict readers.
    return "\n".join(o).encode("ascii")


# --------------------------------------------------------------------------- #
#  Bundle
# --------------------------------------------------------------------------- #
def _mesh_markdown(sec: Section, mesh: MeshShortlist) -> str:
    b = mesh.basis
    lines = [
        f"# Mesh shortlist - {sec.label}",
        "",
        f"Sizing basis: thinnest material **{b['min_feature_mm']:g} mm** "
        f"({b['min_feature_from']}).",
        f"Method: **{b['method']}** - {b['method_rationale']}",
        f"Defeature below {b['defeature_mm']:g} mm.",
        "",
        "| Level | Global (mm) | Local at holes (mm) | Across thinnest | "
        "Growth | Max skew | Est. DOF |",
        "|---|---|---|---|---|---|---|",
    ]
    for lv in mesh.levels:
        lines.append(
            f"| {lv.name} | {lv.global_size_mm:g} | {lv.local_size_mm:g} | "
            f"{lv.elems_across_thin} | {lv.growth_rate:g} | "
            f"{lv.quality['skewness_max']:g} | {lv.est_dof:,} |")
    lines += ["", f"_{b['estimate_note']}_", "",
              "## Convergence", "",
              f"Compare **{mesh.convergence['compare'][0]}** against "
              f"**{mesh.convergence['compare'][1]}** on "
              f"{mesh.convergence['metric']}; accept a change of up to "
              f"{mesh.convergence['accept_change_pct']:g}%.", "",
              mesh.convergence["note"], "", "---", "", DISCLAIMER]
    return "\n".join(lines)


def _readme(sec: Section, study: Study) -> str:
    need = [lc["name"] for lc in study.load_cases if lc.get("required")]
    lines = [
        f"KinematiK simulation handoff - {sec.label}",
        f"Schema: {SCHEMA}",
        f"Geometry digest: {sec.digest()}",
        "",
        "Contents",
        "--------",
        f"  {_dxf_name(sec)}  geometry, roles on layers + XDATA, manifest",
        "                     embedded as 999 comments",
        "  manifest.json      the whole handoff in one document",
        "  study.json         load cases and constraints, bound to the",
        "                     named selections carried in the DXF",
        "  mesh_shortlist.md  three mesh levels and the convergence pair",
        "",
        "Units",
        "-----",
        "  Millimetres, newtons, MPa. Always. Whatever the app was showing.",
        "",
    ]
    if need:
        lines += ["Before you run", "--------------",
                  "  These loads are NOT declared anywhere yet and are marked",
                  "  required rather than filled in with a guess:",
                  *[f"    - {n}" for n in need],
                  "  A run should stop on them.", ""]
    lines += ["", DISCLAIMER]
    return "\n".join(lines)


def bundle_bytes(sec: Section, frame_tag: str = "") -> bytes:
    """The four artefacts as one zip, internally consistent by construction."""
    man = handoff_manifest(sec, frame_tag=frame_tag)
    mesh = mesh_shortlist(sec)
    study = study_spec(sec)
    name = man["part"]["dxf"]

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr(name, section_dxf(sec))
        z.writestr("manifest.json",
                   json.dumps(man, indent=2, ensure_ascii=False, default=str))
        z.writestr("study.json",
                   json.dumps(study.as_dict(), indent=2, ensure_ascii=False,
                              default=str))
        z.writestr("mesh_shortlist.md", _mesh_markdown(sec, mesh))
        z.writestr("README.txt", _readme(sec, study))
    return buf.getvalue()


# --------------------------------------------------------------------------- #
#  One call for the UI
# --------------------------------------------------------------------------- #
def handoff_for_candidate(subsystem_key, candidate, frame_tag: str = "") -> dict:
    """Everything the export UI needs, from one of the app's candidate dicts."""
    sec = section_from_candidate(subsystem_key, candidate)
    sec.frame_tag = _ascii(frame_tag or "")
    mesh = mesh_shortlist(sec)
    study = study_spec(sec)
    man = handoff_manifest(sec, frame_tag=frame_tag)
    return {
        "section": sec,
        "mesh": mesh,
        "study": study,
        "manifest": man,
        "dxf": section_dxf(sec),
        "dxf_name": man["part"]["dxf"],
        "bundle": bundle_bytes(sec, frame_tag=frame_tag),
        "bundle_name": f"kinematik_{_slug(sec.subsystem)}_"
                       f"{_slug(sec.label)}_handoff.zip",
    }
