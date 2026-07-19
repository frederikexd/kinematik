# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Phantom Envelope — the exact 3D volume a moving corner CLAIMS across its whole
range of motion, carved by swept capsules and warped by Ghost Topology's
real-time compliance deflection.

WHY THIS MODULE EXISTS
----------------------
Packaging conflicts between the suspension and everything the chassis and
powertrain teams want to bolt near it (motor mounts, inverter cases, headers,
harness runs, ducting) are found one of two ways in industry:

    1. a manual CAD interference check — someone loads both assemblies into the
       big seat, sweeps the suspension through travel and steer by hand, and
       eyeballs the clearance. It is slow, it is a person's afternoon, and it is
       run against the RIGID sweep — the geometry the links have when nothing is
       loaded;
    2. brute-force 3D mesh collision across the motion range — correct in
       principle, but it melts a workstation and nobody runs it per design
       iteration, so the powertrain lead waits days for an answer or ships the
       mount and hopes.

Neither closes the loop that actually bites: under a 1.8 g corner the upper
control arm does not sit on its rigid sweep. It DEFLECTS — compliance camber,
compliance steer, the outboard ball joint walking inboard by millimetres — and
the volume the loaded link needs is a warped copy of the volume the CAD sweep
drew. The mount that clears the rigid sweep by 2 mm can foul the compliant one.

KinematiK already owns every piece of the honest answer as tested standalone
parts: the rigid corner solver (kinematics.py) sweeps the links through travel
and steer; the Ghost Topology engine (ghost_topology.py) solves the DEFORMED
geometry at each instant of a transient load history, so it already knows where
every outboard point actually is under load, per member, per instant. This
module is the geometric join: it takes those solved poses, treats each link as
a moving CAPSULE (a line segment with the member's tube radius), and carves the
union of every capsule across the whole motion — rigid sweep, compliance-warped
sweep, or both — into a lightweight point cloud the other teams can query.

WHY NO SUPERCOMPUTER IS NEEDED (the honest trick)
-------------------------------------------------
Same pragmatism as the rest of the repo, stated and priced:

  * A suspension link is a two-force member on spherical joints — a straight
    tube between two points. Its exact solid at any instant is a CAPSULE
    (a cylinder of the tube radius capped by two hemispheres). Capsule geometry
    has a closed-form point-to-segment distance, so "is this point inside the
    swept solid?" is a min over segments of (distance − radius) — arithmetic,
    not a mesh boolean. No triangles, no BVH, no GPU.

  * The sweep is the union of the capsules at the audited instants. We do NOT
    voxelise the whole bounding box (that IS the brute-force cost). We carve the
    SURFACE: sample points on each capsule at each instant, keep the ones no
    OTHER capsule swallows, and that hull-shell point cloud is the forbidden
    boundary. O(instants × members × samples), a few thousand points — a JSON
    file, not an FEA queue.

  * The compliance warp is FREE because Ghost Topology already solved it. We
    read the outboard point straight off each instant's `ghost` CornerState;
    the deflection is baked in. No extra structural solve happens here.

The limit is stated too: this is the volume of the LINKS (and optionally the
wheel/tyre disc and the upright), swept through the instants the ghost audit
chose to solve. It is as dense in motion as the audit is — feed it a fine sweep
for a fine envelope. It is a screening boundary to hand the packaging seat and a
fast "does my mount fit" gate, NOT a certified CAD interference sign-off. The
same spirit as Ghost Topology handing the FEA seat a load case: this hands the
packaging review a forbidden volume and a pass criterion, and says so.

HONEST SCOPE (what this is NOT)
-------------------------------
  * The swept solid of straight two-force links as capsules of the tube OD,
    plus optional wheel/tyre disc and a spherical upright blob. Brackets, tabs,
    the actual A-arm gusset webs, and any non-tubular geometry are NOT modelled
    — inflate the radius or add a member if you need them covered.
  * The sweep is only as continuous as the instants sampled. Between two audited
    instants the capsule is linearly bridged (the honest thing for a monotone
    sweep segment); a wildly non-monotone motion between two samples can leave a
    gap. Sample density is yours to set; the carve reports how many instants fed
    it.
  * "Rigid", "compliant", or "both" envelopes are all offered. The compliant one
    is the point of the tool, but it inherits Ghost Topology's scope: quasi-
    static compliance, no plasticity — a ghost instant flagged void (FoS < 1) is
    excluded from the carve and SAID SO, because a yielded link's position is not
    a number this model owns.

Units: mm throughout. Corner axes: SAE-style right-side corner (x rear+, y
right+, z up+), identical to kinematics.py and ghost_topology.py — the point
cloud a downstream team queries is in the SAME corner frame the geometry was
defined in. A `frame` label rides on every output so nobody queries a right-
corner cloud with a left-corner mount by accident.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from typing import Optional, Iterable

import numpy as np

from .kinematics import Hardpoints, CornerState
from .compliance import _MEMBER_ENDPOINTS
from .ghost_topology import GhostCorner, GhostAudit, GhostInstant, _MARGIN_MEMBERS


# The inboard (chassis-fixed) attribute on Hardpoints for each member, and the
# CornerState attribute holding the solved outboard point. Mirrors
# compliance._MEMBER_ENDPOINTS but resolved to what we actually read here.
_OUTBOARD_STATE_ATTR = {
    "UF": "upper_outer", "UR": "upper_outer",
    "LF": "lower_outer", "LR": "lower_outer",
    "TR": "tie_rod_outer", "PR": "pushrod_outer",
}


# --------------------------------------------------------------------------- #
#  Capsule geometry — the closed forms the carve stands on
# --------------------------------------------------------------------------- #
def _seg_point_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Shortest distance from point p to the segment a→b (mm)."""
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return float(np.linalg.norm(p - a))
    t = float((p - a) @ ab) / denom
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    proj = a + t * ab
    return float(np.linalg.norm(p - proj))


def _seg_point_distance_batch(pts: np.ndarray, a: np.ndarray,
                              b: np.ndarray) -> np.ndarray:
    """Distance from every row of `pts` (N×3) to segment a→b. Vectorised."""
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return np.linalg.norm(pts - a, axis=1)
    t = (pts - a) @ ab / denom
    t = np.clip(t, 0.0, 1.0)
    proj = a[None, :] + t[:, None] * ab[None, :]
    return np.linalg.norm(pts - proj, axis=1)


@dataclass
class Capsule:
    """A moving link's solid at one instant: segment a→b, radius r (mm)."""
    a: np.ndarray
    b: np.ndarray
    r: float
    member: str
    t: float          # event time of the instant that produced it (s)

    def contains(self, p: np.ndarray, skin_mm: float = 0.0) -> bool:
        return _seg_point_distance(p, self.a, self.b) <= self.r + skin_mm

    def signed_clearance(self, p: np.ndarray) -> float:
        """+ outside the capsule surface, − inside. Distance to the tube skin."""
        return _seg_point_distance(p, self.a, self.b) - self.r


# --------------------------------------------------------------------------- #
#  The carved envelope — a queryable forbidden volume
# --------------------------------------------------------------------------- #
@dataclass
class ClearanceResult:
    """The answer to 'does my point/sphere fit?', fully attributed."""
    point_mm: list
    probe_radius_mm: float
    clearance_mm: float          # + clear, − penetrating (skin-to-skin)
    violates: bool
    nearest_member: str
    nearest_t_s: float           # event time of the governing instant
    nearest_load_N: Optional[dict]   # the wheel load at that instant, if known
    frame: str

    def summary(self) -> dict:
        return dict(asdict(self))


@dataclass
class PhantomEnvelope:
    """
    The swept, compliance-warped forbidden volume of one corner, as a point
    cloud of the boundary plus the capsule set it was carved from (so a query
    is exact against the capsules, not just the sampled shell).

    kind        : "rigid", "compliant", or "both" — which sweep was carved.
    frame       : corner frame label, e.g. "FR (right-side corner axes)".
    capsules    : every Capsule across every audited instant (the exact solid).
    boundary    : N×3 hull-shell points (the lightweight cloud to ship).
    members     : the member set carved.
    n_instants  : instants that fed the carve (post-exclusion).
    excluded    : list[(t, reason)] for instants dropped (e.g. FoS-void).
    radius_mm   : member -> capsule radius used.
    load_at_t   : t(s) -> {'Fx','Fy','Fz'} for query attribution, if available.
    bbox        : (min_xyz, max_xyz) of the boundary cloud.
    """
    kind: str
    frame: str
    capsules: list
    boundary: np.ndarray
    members: tuple
    n_instants: int
    excluded: list
    radius_mm: dict
    load_at_t: dict = field(default_factory=dict)
    note: str = ""

    # ---------------------------------------------------------------- #
    @property
    def bbox(self) -> tuple:
        if len(self.boundary) == 0:
            z = np.zeros(3)
            return z, z
        return (self.boundary.min(axis=0), self.boundary.max(axis=0))

    @property
    def n_points(self) -> int:
        return int(len(self.boundary))

    # ---------------------------------------------------------------- #
    def query(self, point, probe_radius_mm: float = 0.0) -> ClearanceResult:
        """
        The headline: does a sphere of `probe_radius_mm` centred at `point`
        (a motor-mount corner, an inverter face, a harness clip) clear the
        swept volume? Exact against the capsule set — the sampled boundary is
        for drawing, the capsules are for the answer.

        clearance_mm > 0 : the sphere clears the nearest tube skin by that gap.
        clearance_mm ≤ 0 : it penetrates the phantom envelope by |clearance|.
        The governing member, the instant (event time), and the load at that
        instant are attributed, so the reply reads like the pitch:
        'the UCA compliance-deflects here and your mount violates by 1.1 mm at
         this instant of the corner'.
        """
        p = np.asarray(point, float)
        best_clear = math.inf
        best_cap: Optional[Capsule] = None
        for cap in self.capsules:
            c = cap.signed_clearance(p) - probe_radius_mm
            if c < best_clear:
                best_clear, best_cap = c, cap
        if best_cap is None:
            return ClearanceResult(p.tolist(), float(probe_radius_mm),
                                   float("inf"), False, "", float("nan"),
                                   None, self.frame)
        load = self.load_at_t.get(round(best_cap.t, 6))
        return ClearanceResult(
            point_mm=[float(v) for v in p],
            probe_radius_mm=float(probe_radius_mm),
            clearance_mm=float(best_clear),
            violates=bool(best_clear <= 0.0),
            nearest_member=best_cap.member,
            nearest_t_s=float(best_cap.t),
            nearest_load_N=load,
            frame=self.frame,
        )

    def clearances(self, points, probe_radius_mm: float = 0.0) -> np.ndarray:
        """
        Vectorised clearance for a whole cloud of candidate points at once
        (N×3 in, length-N array out). Skin-to-skin clearance to the nearest
        capsule; + clear, − penetrating. This is the fast path for checking a
        mount's entire surface against the envelope in one shot.
        """
        pts = np.asarray(points, float)
        if pts.ndim == 1:
            pts = pts[None, :]
        best = np.full(len(pts), math.inf)
        for cap in self.capsules:
            d = _seg_point_distance_batch(pts, cap.a, cap.b) - cap.r - probe_radius_mm
            best = np.minimum(best, d)
        return best

    def query_many(self, points, probe_radius_mm: float = 0.0) -> list:
        """Query a cloud of candidate points (e.g. a mount's own surface)."""
        return [self.query(p, probe_radius_mm) for p in points]

    def worst_violation(self, points, probe_radius_mm: float = 0.0):
        """The deepest-penetrating point of a candidate part, or None if all clear."""
        res = self.query_many(points, probe_radius_mm)
        worst = min(res, key=lambda r: r.clearance_mm) if res else None
        return worst if (worst and worst.violates) else None

    # ---------------------------------------------------------------- #
    def to_point_cloud(self) -> dict:
        """The lightweight matrix the other teams consume. JSON-ready."""
        mn, mx = self.bbox
        return {
            "format": "kinematik.phantom_envelope/1",
            "kind": self.kind,
            "frame": self.frame,
            "units": "mm",
            "members": list(self.members),
            "radius_mm": {k: float(v) for k, v in self.radius_mm.items()},
            "n_instants": self.n_instants,
            "n_points": self.n_points,
            "bbox_min_mm": [float(v) for v in mn],
            "bbox_max_mm": [float(v) for v in mx],
            "excluded_instants": [
                {"t_s": float(t), "reason": r} for t, r in self.excluded],
            "note": self.note,
            "points_xyz_mm": self.boundary.round(3).tolist(),
        }

    def to_json(self, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_point_cloud(), indent=indent)

    def to_csv(self) -> str:
        """A plain x,y,z point cloud (+ member/t provenance) for the CAD seat."""
        lines = ["x_mm,y_mm,z_mm,member,t_s"]
        # boundary points don't carry provenance individually; the capsules do,
        # so emit the boundary as x,y,z and append a provenance block? Keep it
        # simple and honest: one flat cloud, the header names the frame.
        for row in self.boundary:
            lines.append(f"{row[0]:.3f},{row[1]:.3f},{row[2]:.3f},,")
        # capsule endpoints carry member+t — append them so the CSV is
        # self-describing without the JSON.
        for cap in self.capsules:
            for pt in (cap.a, cap.b):
                lines.append(f"{pt[0]:.3f},{pt[1]:.3f},{pt[2]:.3f},"
                             f"{cap.member},{cap.t:.4f}")
        return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  Building capsules from solved geometry
# --------------------------------------------------------------------------- #
def _capsules_for_state(hp: Hardpoints, state: CornerState, t: float,
                        members: Iterable[str], radius_mm: dict) -> list:
    """One capsule per member: chassis-fixed inboard point → solved outboard."""
    caps = []
    for m in members:
        out_attr = _OUTBOARD_STATE_ATTR.get(m)
        _, in_field = _MEMBER_ENDPOINTS[m]
        p_out = getattr(state, out_attr, None) if out_attr else None
        if p_out is None:
            # e.g. no rocker geometry -> no pushrod outer; skip that member.
            continue
        p_in = np.asarray(getattr(hp, in_field), float)
        caps.append(Capsule(a=np.asarray(p_out, float), b=p_in,
                            r=float(radius_mm.get(m, 10.0)), member=m, t=t))
    return caps


def _capsule_surface_points(cap: Capsule, n_ring: int = 10,
                            n_len: int = 6) -> np.ndarray:
    """
    Sample points on a capsule's surface: `n_len` rings of `n_ring` points along
    the tube, plus the two hemispherical caps as their pole rings. Cheap and
    dense enough for a screening boundary.
    """
    a, b, r = cap.a, cap.b, cap.r
    axis = b - a
    L = float(np.linalg.norm(axis))
    if L < 1e-9:
        # degenerate: a sphere. Return a small ring.
        return a[None, :] + r * _sphere_dirs(n_ring)
    z = axis / L
    # two vectors spanning the plane normal to the axis
    tmp = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = tmp - (tmp @ z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    ang = np.linspace(0.0, 2.0 * math.pi, n_ring, endpoint=False)
    ring = np.cos(ang)[:, None] * x[None, :] + np.sin(ang)[:, None] * y[None, :]
    pts = []
    for s in np.linspace(0.0, 1.0, n_len):
        centre = a + s * axis
        pts.append(centre[None, :] + r * ring)
    # cap poles (hemisphere tips)
    pts.append((a - r * z)[None, :])
    pts.append((b + r * z)[None, :])
    return np.vstack(pts)


def _sphere_dirs(n: int) -> np.ndarray:
    """n roughly-even directions on the unit sphere (Fibonacci)."""
    i = np.arange(n) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n)
    gold = math.pi * (1.0 + 5.0 ** 0.5)
    theta = gold * i
    return np.column_stack([np.sin(phi) * np.cos(theta),
                            np.sin(phi) * np.sin(theta),
                            np.cos(phi)])


# --------------------------------------------------------------------------- #
#  The carve — union of swept capsules, boundary shell extracted
# --------------------------------------------------------------------------- #
def carve_from_states(hp: Hardpoints, states: list, times: list,
                      radius_mm: dict, members: tuple = _MARGIN_MEMBERS,
                      kind: str = "rigid",
                      frame: str = "corner",
                      load_at_t: Optional[dict] = None,
                      ring: int = 10, along: int = 6,
                      skin_mm: float = 0.05,
                      excluded: Optional[list] = None,
                      note: str = "") -> PhantomEnvelope:
    """
    Carve a PhantomEnvelope from a list of solved CornerStates (one per instant).

    The forbidden solid is the UNION of every member's capsule across every
    state. The boundary cloud is the subset of sampled capsule-surface points
    that no OTHER capsule swallows — the outer shell of that union, which is all
    a packaging query needs to see and all that needs shipping.
    """
    caps: list[Capsule] = []
    for st, t in zip(states, times):
        caps.extend(_capsules_for_state(hp, st, t, members, radius_mm))

    # sample every capsule's surface, then reject points strictly inside any
    # OTHER capsule (union boundary). skin_mm keeps a point on its own tube from
    # being eaten by a neighbour it merely touches.
    boundary_pts: list[np.ndarray] = []
    for i, cap in enumerate(caps):
        surf = _capsule_surface_points(cap, n_ring=ring, n_len=along)
        keep = np.ones(len(surf), bool)
        for j, other in enumerate(caps):
            if j == i:
                continue
            d = _seg_point_distance_batch(surf, other.a, other.b)
            keep &= d >= (other.r - skin_mm)
        if keep.any():
            boundary_pts.append(surf[keep])
    boundary = (np.vstack(boundary_pts) if boundary_pts
                else np.zeros((0, 3), float))

    return PhantomEnvelope(
        kind=kind, frame=frame, capsules=caps, boundary=boundary,
        members=tuple(members), n_instants=len(states),
        excluded=list(excluded or []), radius_mm=dict(radius_mm),
        load_at_t=dict(load_at_t or {}), note=note)


# --------------------------------------------------------------------------- #
#  The kinematic (rigid) sweep — motion range, no load
# --------------------------------------------------------------------------- #
def rigid_sweep_states(gc: GhostCorner, travel_mm=(-25.0, 25.0),
                       n_travel: int = 9, steer_mm=(0.0,), ) -> tuple:
    """
    Sweep the RIGID linkage through a travel range (and optional tie-rod steer
    offsets, applied as extra states) to get the no-load motion envelope. Uses
    the same solver the whole repo trusts (kinematics.solve_at_travel). Returns
    (states, times) with a synthetic 'time' = travel index so the boundary carve
    treats them as distinct instants.
    """
    kin = gc.cc.rigid_kin
    lo, hi = float(travel_mm[0]), float(travel_mm[1])
    travels = np.linspace(lo, hi, max(2, int(n_travel)))
    states, times = [], []
    seed = None
    for k, tr in enumerate(travels):
        st = kin.solve_at_travel(float(tr), seed=seed)
        if getattr(st, "converged", True):
            seed = np.concatenate([st.lower_outer, st.upper_outer,
                                   st.tie_rod_outer])
        states.append(st)
        times.append(float(k))     # synthetic ordinal 'time'
    return states, times


def radii_from_sections(gc: GhostCorner,
                        inflate_mm: float = 0.0) -> dict:
    """Capsule radius per member = tube OD/2 (+ optional inflation for tabs/webs)."""
    out = {}
    for m in _MARGIN_MEMBERS:
        sec = gc.sections.get(m)
        od = sec.od_mm if sec is not None else 19.05
        out[m] = 0.5 * od + float(inflate_mm)
    return out


# --------------------------------------------------------------------------- #
#  Public entry points
# --------------------------------------------------------------------------- #
def _frame_label(corner_label: str) -> str:
    cl = (corner_label or "corner").upper()
    side = "right-side" if cl.endswith("R") else (
        "left-side" if cl.endswith("L") else "corner")
    return f"{cl} ({side} corner axes, x rear+, y right+, z up+)"


def carve_rigid_envelope(gc: GhostCorner, corner_label: str = "corner",
                         travel_mm=(-25.0, 25.0), n_travel: int = 9,
                         members: tuple = _MARGIN_MEMBERS,
                         inflate_mm: float = 0.0,
                         ring: int = 10, along: int = 6) -> PhantomEnvelope:
    """
    The rigid motion envelope: the volume the links sweep through travel with NO
    load. This is what a manual CAD interference check draws — offered here as
    the honest baseline the compliant envelope is compared against.
    """
    radius = radii_from_sections(gc, inflate_mm)
    states, times = rigid_sweep_states(gc, travel_mm=travel_mm, n_travel=n_travel)
    return carve_from_states(
        gc.cc.rigid_kin.hp, states, times, radius, members=members,
        kind="rigid", frame=_frame_label(corner_label),
        ring=ring, along=along,
        note=(f"rigid travel sweep {travel_mm[0]:+.0f}..{travel_mm[1]:+.0f} mm, "
              f"{len(states)} poses, no load."))


def carve_ghost_envelope(audit: GhostAudit, gc: GhostCorner,
                         members: tuple = _MARGIN_MEMBERS,
                         inflate_mm: float = 0.0,
                         ring: int = 10, along: int = 6,
                         include_void: bool = False) -> PhantomEnvelope:
    """
    The COMPLIANT envelope — the point of the tool. Reads the DEFORMED geometry
    Ghost Topology already solved at each audited instant of the transient and
    carves the volume the loaded links actually claim. Instants flagged FoS-void
    (elastic model invalid past yield) are excluded unless `include_void`, and
    the exclusion is recorded on the envelope, not hidden.
    """
    radius = radii_from_sections(gc, inflate_mm)
    states, times, load_at_t, excluded = [], [], {}, []
    for g in audit.instants:
        if (not include_void) and g.min_fos < 1.0:
            excluded.append((g.t, f"{g.min_fos_member} FoS {g.min_fos:.2f} < 1.0 "
                                  "— elastic geometry void past yield"))
            continue
        states.append(g.ghost)
        times.append(g.t)
        load_at_t[round(g.t, 6)] = {"Fx": float(g.load.Fx),
                                    "Fy": float(g.load.Fy),
                                    "Fz": float(g.load.Fz)}
    note = (f"compliance-warped sweep over {len(states)} audited instants of "
            f"'{audit.corner_label}'; deflection from Ghost Topology.")
    if excluded:
        note += f" {len(excluded)} instant(s) excluded (FoS-void)."
    return carve_from_states(
        gc.cc.rigid_kin.hp, states, times, radius, members=members,
        kind="compliant", frame=_frame_label(audit.corner_label),
        load_at_t=load_at_t, ring=ring, along=along,
        excluded=excluded, note=note)


# --------------------------------------------------------------------------- #
#  Rigid vs compliant — the delta that IS the pitch
# --------------------------------------------------------------------------- #
@dataclass
class EnvelopeDelta:
    """How much the compliance warp GREW the forbidden volume vs the rigid sweep."""
    corner_label: str
    max_outward_growth_mm: float     # deepest a compliant point sits outside rigid
    growth_member: str
    growth_t_s: float
    growth_load_N: Optional[dict]
    mean_growth_mm: float
    frac_points_grown: float         # share of compliant boundary outside rigid
    note: str = ""

    def summary(self) -> dict:
        return dict(asdict(self))


def envelope_delta(rigid: PhantomEnvelope, compliant: PhantomEnvelope
                   ) -> EnvelopeDelta:
    """
    Measure how far the compliant envelope pushes OUTSIDE the rigid one: for each
    compliant boundary point, its signed clearance to the rigid capsule union
    (negative = inside rigid, positive = outside → new forbidden ground the CAD
    sweep never drew). The max positive is the headline 'the loaded arm needs
    N mm the rigid check missed'.
    """
    if compliant.n_points == 0 or not rigid.capsules:
        return EnvelopeDelta(compliant.frame, 0.0, "", float("nan"), None,
                             0.0, 0.0, note="empty envelope(s)")
    growth = np.full(compliant.n_points, -math.inf)
    # for each compliant point, distance OUTSIDE the nearest rigid capsule skin
    for cap in rigid.capsules:
        d = _seg_point_distance_batch(compliant.boundary, cap.a, cap.b) - cap.r
        # a point's clearance to the rigid UNION is the min over capsules; we
        # want the min distance-to-skin (closest rigid tube), so track min.
        growth = np.maximum(growth, -math.inf)  # no-op guard
    # min over capsules = closest rigid tube for each point
    dists = np.full(compliant.n_points, math.inf)
    for cap in rigid.capsules:
        d = _seg_point_distance_batch(compliant.boundary, cap.a, cap.b) - cap.r
        dists = np.minimum(dists, d)
    grown = dists > 0.0
    imax = int(np.argmax(dists))
    # attribute the worst-growth point to the compliant capsule it lies on
    worst_pt = compliant.boundary[imax]
    best_member, best_t, best_gap = "", float("nan"), -math.inf
    for cap in compliant.capsules:
        on = cap.signed_clearance(worst_pt)
        if abs(on) < abs(best_gap) or best_member == "":
            if abs(on) <= cap.r + 1e-3:
                best_member, best_t, best_gap = cap.member, cap.t, on
    load = compliant.load_at_t.get(round(best_t, 6)) if best_t == best_t else None
    return EnvelopeDelta(
        corner_label=compliant.frame,
        max_outward_growth_mm=float(max(0.0, dists[imax])),
        growth_member=best_member, growth_t_s=float(best_t),
        growth_load_N=load,
        mean_growth_mm=float(dists[grown].mean()) if grown.any() else 0.0,
        frac_points_grown=float(grown.mean()),
        note=("compliant boundary points measured against the rigid capsule "
              "union; positive = forbidden ground the rigid sweep missed."))


# --------------------------------------------------------------------------- #
#  Markdown report — the sheet that goes to the powertrain / packaging lead
# --------------------------------------------------------------------------- #
def render_envelope_md(env: PhantomEnvelope,
                       delta: Optional[EnvelopeDelta] = None,
                       queries: Optional[list] = None) -> str:
    L: list[str] = []
    mn, mx = env.bbox
    L.append(f"# Phantom Envelope — {env.frame}")
    L.append("")
    L.append(f"**Sweep:** `{env.kind}` · {env.n_instants} instants · "
             f"{env.n_points} boundary points · members "
             f"{', '.join(env.members)}")
    L.append("")
    L.append(f"Bounding box (mm): "
             f"x [{mn[0]:.1f}, {mx[0]:.1f}], "
             f"y [{mn[1]:.1f}, {mx[1]:.1f}], "
             f"z [{mn[2]:.1f}, {mx[2]:.1f}].")
    L.append("")
    if env.excluded:
        L.append("**Excluded instants:**")
        for t, r in env.excluded:
            L.append(f"- t = {t*1000:.0f} ms — {r}")
        L.append("")
    if delta is not None:
        L.append("## Compliance warp vs rigid sweep")
        L.append("")
        if delta.max_outward_growth_mm > 0.0:
            ld = delta.growth_load_N or {}
            ldtxt = (f" at Fy {ld.get('Fy', 0):.0f} N, Fz {ld.get('Fz', 0):.0f} N"
                     if ld else "")
            L.append(
                f"🟠 Under load the corner claims **{delta.max_outward_growth_mm:.2f} mm** "
                f"beyond the rigid sweep, governed by **{delta.growth_member}** at "
                f"t = {delta.growth_t_s*1000:.0f} ms{ldtxt}. "
                f"{delta.frac_points_grown*100:.0f}% of the compliant boundary lies "
                f"outside the rigid one (mean growth {delta.mean_growth_mm:.2f} mm). "
                "A packaging check run against the rigid CAD sweep alone would "
                "miss this ground.")
        else:
            L.append("🟢 The compliant envelope stays within the rigid sweep — "
                     "for this event the rigid CAD interference check is a "
                     "faithful stand-in.")
        L.append("")
    if queries:
        L.append("## Clearance queries")
        L.append("")
        L.append("| point (mm) | probe r (mm) | clearance (mm) | verdict | member | t (ms) |")
        L.append("|---|---|---|---|---|---|")
        for q in queries:
            p = q.point_mm
            v = "🔴 VIOLATES" if q.violates else "🟢 clear"
            tt = f"{q.nearest_t_s*1000:.0f}" if q.nearest_t_s == q.nearest_t_s else "—"
            L.append(f"| ({p[0]:.0f}, {p[1]:.0f}, {p[2]:.0f}) | "
                     f"{q.probe_radius_mm:.1f} | {q.clearance_mm:+.2f} | {v} | "
                     f"{q.nearest_member} | {tt} |")
        L.append("")
    L.append("_The forbidden volume of the LINKS swept as tube-radius capsules, "
             "warped by Ghost Topology's compliance deflection. A screening "
             "boundary and a fast clearance gate for the packaging seat — not a "
             "certified CAD interference sign-off. Same corner frame the "
             "geometry was defined in._")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  Self-test (python3 -m suspension.phantom_envelope)
# --------------------------------------------------------------------------- #
if __name__ == "__main__":   # pragma: no cover
    import numpy as np
    from suspension.ghost_topology import ghost_audit

    hp = Hardpoints.default()
    gc = GhostCorner.uniform_tube(hp, wheel_rate_N_per_mm=35.0, Fz_static_N=700.0)

    print("=== rigid motion envelope (±25 mm travel) ===")
    rigid = carve_rigid_envelope(gc, corner_label="FR", travel_mm=(-25, 25),
                                 n_travel=9)
    print(render_envelope_md(rigid))
    print()

    # a 3 g cornering pulse -> ghost audit -> compliant envelope
    t = np.linspace(0.0, 0.8, 401)
    s = np.sin(np.pi * t / 0.8)
    Fz = 700.0 + 1500.0 * s
    Fy = -3.0 * Fz * s
    Fx = np.zeros_like(t)
    audit = ghost_audit(gc, t, Fx, Fy, Fz, corner_label="FR", n_samples=16)
    comp = carve_ghost_envelope(audit, gc)
    delta = envelope_delta(rigid, comp)

    print("=== compliant (ghost) envelope + delta ===")
    # a candidate motor-mount corner point near the UCA outer, probe r = 6 mm
    probe = np.array([15.0, 545.0, 305.0])
    q = comp.query(probe, probe_radius_mm=6.0)
    print(render_envelope_md(comp, delta=delta, queries=[q]))
    print()
    print("point-cloud JSON keys:", list(comp.to_point_cloud().keys()))
