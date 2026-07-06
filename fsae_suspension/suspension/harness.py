# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Harness layer — the *physical wiring harness* in 3-D car space, the half of the
electrical job `electronics.py` deliberately does not touch.

`electronics.py` owns the copper on the *board*: a trace's width, its IPC-2221
heating, its Onderdonk fusing, the diff-pair impedance. That is the PCB. This
module owns the copper *between* boxes — the loom the electrical member lays into
the chassis the afternoon before they cut a single wire:

    1. route every individual conductor as a 3-D polyline through the same car
       coordinates the suspension mount-points and keep-outs already live in, so a
       wire that would foul a wishbone, an A-arm sweep, or the accumulator box
       shows up as a clearance FAIL on the *same* integration board a mount clash
       does — both owners named,
    2. check the two things that actually scrap a loom on the bench: a bend
       *tighter than the wire can take* (kinks the conductor, work-hardens the
       strands, breaks shielding) and a connector entry with *no strain relief*
       straight length (the wire flexes at the crimp and fatigues the contact),
    3. and then — because you want the answer *before* you cut — derive the
       manufacturing artefacts straight off the route geometry:
         * the exact CUT LENGTH of every conductor to the millimetre
           (3-D path arc-length + service loops + a standards-style bend
           allowance + strip/termination stock),
         * a 1:1 FORMBOARD: the harness unfolded flat to a 2-D nail-board layout
           that preserves every branch length exactly, the drawing a fabricator
           pins the loom out on,
         * the automated BILL OF MATERIALS (wire by gauge, connectors, contacts,
           backshells, conduit/loom tube) rolled straight out of the declared
           parts,
         * the exact COPPER MASS of every conductor and the harness mass
           *distribution* (per-branch CG and the loom's combined CG in car
           coordinates), so the harness weight and where it sits is known to the
           gram before fab.

Same honesty rules the rest of KinematiK keeps:
  * This is NOT a CAD kernel and NOT a wire-flex FEA. The route is the polyline
    the electrical team declares (or imports from the CAD centreline); we measure
    it, we do not solve cable dynamics. Anything that truly needs a flexible-body
    solver — the real sag of an unsupported run under vibration, the true
    minimum-energy drape — is returned as `None` with a stated reason, never an
    invented number.
  * Cross-section / strand data is real (AWG copper areas, insulation build), so
    resistance, voltage drop and *mass* are exact, not guessed. Where a wire
    re-uses a board net that `electronics.py` already sized, the gauge carries the
    current; we do not re-type the current here.
  * Provenance is preserved exactly like `MountPoint`/`Trace`: every `WireRun`
    and `Connector` carries who placed it and whether the route is still an
    estimate, and a finding on estimated geometry says so.

The unit the rest of the app renders is the same typed `Finding` from
`interfaces.py`, so a kinked wire or a loom fouling the accumulator lands on the
integration board next to a melted trace and a suspension clash.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Optional

import numpy as np

from .interfaces import Finding, Severity


# --------------------------------------------------------------------------- #
#  Copper / wire physical constants
# --------------------------------------------------------------------------- #
RHO_CU_20C = 1.724e-8          # ohm·m  (same value electronics.py uses)
ALPHA_CU = 3.93e-3             # 1/°C
CU_DENSITY_KG_M3 = 8960.0      # density of copper
# A representative insulation density (PVC/XLPE/PTFE all land near here for the
# thin builds FSAE looms use); used only for the jacket-mass contribution, which
# is reported separately from the authoritative copper mass.
INSUL_DENSITY_KG_M3 = 1400.0

# Standard solid-copper cross-section per AWG (mm^2). These are the *conductor*
# areas the IPC / NEC tables list; stranded wire of the same AWG matches to <1 %.
# Keyed by AWG number (smaller AWG = thicker wire).
AWG_AREA_MM2 = {
    8: 8.366, 10: 5.261, 12: 3.309, 14: 2.081, 16: 1.309,
    18: 0.823, 20: 0.518, 22: 0.326, 24: 0.205, 26: 0.129,
    28: 0.0810, 30: 0.0509,
}
# Nominal finished outside diameter (mm) of a typical thin-wall stranded wire of
# each AWG (conductor + a TXL/MIL-thin insulation build). Used for bend-radius
# and bundle-OD geometry. These are representative, not a specific part number;
# a WireRun may override `od_mm` with the exact part's spec.
AWG_NOMINAL_OD_MM = {
    8: 4.5, 10: 3.6, 12: 3.0, 14: 2.5, 16: 2.1,
    18: 1.8, 20: 1.5, 22: 1.3, 24: 1.1, 26: 1.0,
    28: 0.9, 30: 0.8,
}


def awg_area_mm2(awg: int) -> float:
    """Conductor cross-section (mm^2) for an AWG, interpolated in the log-area
    domain for gauges not in the table (the AWG scale is geometric)."""
    if awg in AWG_AREA_MM2:
        return AWG_AREA_MM2[awg]
    # AWG area halves roughly every 3 gauges: A(awg) = A_ref * 2^((ref-awg)/3)
    ref = 10
    return AWG_AREA_MM2[ref] * (2.0 ** ((ref - awg) / 3.0))


def awg_nominal_od_mm(awg: int) -> float:
    if awg in AWG_NOMINAL_OD_MM:
        return AWG_NOMINAL_OD_MM[awg]
    # OD scales ~ sqrt(area); anchor on AWG 10.
    return AWG_NOMINAL_OD_MM[10] * np.sqrt(awg_area_mm2(awg) / AWG_AREA_MM2[10])


# --------------------------------------------------------------------------- #
#  Geometry helpers — 3-D polyline arc length, turn angles, AABB clearance
# --------------------------------------------------------------------------- #
def polyline_length_mm(path: np.ndarray) -> float:
    """Total 3-D arc length of an (N,3) polyline in mm."""
    if path.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def segment_lengths_mm(path: np.ndarray) -> np.ndarray:
    """Per-segment lengths of an (N,3) polyline."""
    if path.shape[0] < 2:
        return np.zeros((0,))
    return np.linalg.norm(np.diff(path, axis=0), axis=1)


def turn_angles_deg(path: np.ndarray) -> np.ndarray:
    """
    Interior turn angle (deg) at every *internal* vertex of the polyline — the
    deflection from straight (0° = dead straight, 180° = folded back on itself).
    These are the corners a bend radius has to be checked against.
    """
    if path.shape[0] < 3:
        return np.zeros((0,))
    angs = []
    for i in range(1, path.shape[0] - 1):
        a = path[i] - path[i - 1]
        b = path[i + 1] - path[i]
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-9 or nb < 1e-9:
            angs.append(0.0)
            continue
        cosang = float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))
        angs.append(float(np.degrees(np.arccos(cosang))))
    return np.asarray(angs)


def vertex_bend_radius_mm(path: np.ndarray) -> np.ndarray:
    """
    Effective bend radius (mm) the route geometry implies at each internal vertex.
    A polyline has zero radius at a hard corner, so we estimate the radius the
    fabricator would actually form from the corner's turn angle and the length of
    the two adjacent segments — the radius of the largest circular arc that fits
    inside the corner and is tangent to both legs:

        R = min(leg_in, leg_out)/2 * tan( (180° - turn) / 2 )

    A gentle sweep (small turn over long legs) gives a large R; a sharp kink over
    short legs gives a small R. This is the standard "fillet that fits the corner"
    construction — a screening estimate of the formed radius, not a cable-FEA
    drape.
    """
    if path.shape[0] < 3:
        return np.zeros((0,))
    seglen = segment_lengths_mm(path)
    turns = turn_angles_deg(path)
    radii = []
    for i, turn in enumerate(turns):
        leg_in = seglen[i]
        leg_out = seglen[i + 1]
        half = np.radians((180.0 - turn) / 2.0)
        # tan(half) is large for a nearly-straight corner -> large radius
        usable = min(leg_in, leg_out) / 2.0
        if turn < 1e-6:
            radii.append(float("inf"))
        else:
            radii.append(float(usable * np.tan(half)))
    return np.asarray(radii)


# --------------------------------------------------------------------------- #
#  Declaration primitives — connectors, wire runs, the routed harness
# --------------------------------------------------------------------------- #
@dataclass
class Connector:
    """
    A harness end-point / branch node: an ECU header, a sensor plug, a splice, a
    motor-controller connector. Lives in the same car coordinates as everything
    else. Wires terminate here; strain relief is measured from here.

        xyz_mm        : (x,y,z) location of the connector face in car coordinates
        owner_subsystem: who owns the box this plugs into
        cavities      : number of contact cavities (for the BOM contact count)
        part_number   : the connector PN (rolled into the BOM)
        strain_relief_mm: required straight run of wire leaving this connector
                          before the first bend (the backshell / clamp zone)
        mass_g        : mass of the connector body itself (housing + backshell),
                        for the harness mass roll-up; None = not declared
    """
    name: str
    owner_subsystem: str
    xyz_mm: tuple = (0.0, 0.0, 0.0)
    cavities: int = 1
    part_number: str = ""
    strain_relief_mm: float = 25.0
    mass_g: Optional[float] = None
    is_estimate: bool = True
    set_by: str = ""
    notes: str = ""

    def as_array(self) -> np.ndarray:
        return np.asarray(self.xyz_mm, dtype=float)

    def as_dict(self):
        return asdict(self)

    @staticmethod
    def from_dict(d) -> "Connector":
        d = dict(d)
        if isinstance(d.get("xyz_mm"), list):
            d["xyz_mm"] = tuple(d["xyz_mm"])
        valid = Connector.__dataclass_fields__.keys()
        return Connector(**{k: v for k, v in d.items() if k in valid})


@dataclass
class WireRun:
    """
    One conductor routed in 3-D from one connector to another. The route is the
    polyline centreline (car coordinates, mm) the electrical team lays through the
    CAD — the same coordinate frame the suspension geometry uses, so a wire can be
    clearance-checked against the very keep-outs the mount-points are.

        gauge_awg     : wire gauge -> conductor area, mass, OD, resistance
        path_mm       : [(x,y,z), ...] routed centreline through the car
        from_conn/to_conn: connector names this conductor terminates into
        net           : logical net (ties back to a board net if it continues a
                        trace electronics.py already sized — the gauge then has to
                        carry that net's current)
        od_mm         : finished outside diameter; None -> AWG nominal
        bundle_min_radius_mult: minimum bend radius as a multiple of OD. Industry
                        practice for a single building wire is ~4-6x OD; a shielded
                        / coax / fibre-stiffened run wants 8-10x. Default 6.
        service_loop_mm: slack deliberately left at install (per the route),
                        included in cut length but NOT in the taut clearance/bend
                        geometry.
        strip_mm      : conductor stripped + terminated stock added to each end
                        for the crimp (added to cut length, both ends).
        carries_current_a: the worst-case current this conductor must carry, if
                        the electrical team pins it here; None = read from the net
                        elsewhere / not declared (an ampacity finding is then
                        MISSING, never invented).
    """
    name: str
    owner_subsystem: str
    gauge_awg: int = 20
    path_mm: list = field(default_factory=list)        # [(x,y,z), ...]
    from_conn: str = ""
    to_conn: str = ""
    net: str = ""
    od_mm: Optional[float] = None
    bundle_min_radius_mult: float = 6.0
    service_loop_mm: float = 0.0
    strip_mm: float = 8.0
    carries_current_a: Optional[float] = None
    is_estimate: bool = True
    set_by: str = ""
    notes: str = ""

    # ---- geometry -------------------------------------------------------- #
    def as_polyline(self) -> np.ndarray:
        if not self.path_mm:
            return np.zeros((0, 3))
        return np.asarray(self.path_mm, dtype=float)

    @property
    def outside_diameter_mm(self) -> float:
        return float(self.od_mm) if self.od_mm else awg_nominal_od_mm(self.gauge_awg)

    @property
    def min_bend_radius_mm(self) -> float:
        """Allowed minimum formed bend radius = multiplier × outside diameter."""
        return self.bundle_min_radius_mult * self.outside_diameter_mm

    def routed_length_mm(self) -> float:
        """Taut 3-D path length of the centreline (no loops / strip stock)."""
        return polyline_length_mm(self.as_polyline())

    def cut_length_mm(self) -> float:
        """
        The number the fabricator cuts to, to the millimetre:

            routed 3-D arc length
          + service loop slack the route asks for
          + bend allowance (extra wire consumed forming each corner vs the sharp
            polyline corner — the standard "arc is longer than the mitre" term:
            for a corner turned by angle θ formed at radius R, the formed arc
            exceeds the two tangent legs by  R*(θ_rad - 2*tan(θ/2)) ... which is
            negative, i.e. a real radius is SHORTER than a hard mitre; we instead
            add the conservative neutral-axis allowance R*θ_rad for wire pulled
            around the bend, the term loom shops actually add)
          + strip / termination stock at both ends.

        All terms are explicit and summed in mm; nothing is rounded away until the
        caller chooses to.
        """
        path = self.as_polyline()
        base = polyline_length_mm(path)
        # bend allowance: extra wire pulled around each formed corner. The wire is
        # formed at roughly its own minimum bend radius at a real corner, NOT at
        # the large geometric fillet that happens to fit a gentle sweep — a gentle
        # sweep over long legs adds essentially no extra wire. So cap the radius
        # used for the allowance at a realistic forming radius (its min bend
        # radius), and only count corners that actually deflect.
        turns = np.radians(turn_angles_deg(path))
        radii = vertex_bend_radius_mm(path)
        r_form_cap = self.min_bend_radius_mm
        bend_allow = 0.0
        for theta, R in zip(turns, radii):
            if np.isfinite(R) and theta > 0:
                r_eff = min(R, r_form_cap)
                bend_allow += float(r_eff * theta)
        return base + self.service_loop_mm + bend_allow + 2.0 * self.strip_mm

    # ---- electrical (exact from AWG, same physics as electronics.py) ----- #
    def resistance_ohm(self, temp_c: float = 20.0) -> float:
        rho = RHO_CU_20C * (1.0 + ALPHA_CU * (temp_c - 20.0))
        L = self.cut_length_mm() * 1e-3
        A = awg_area_mm2(self.gauge_awg) * 1e-6
        if A <= 0:
            return float("nan")
        return rho * L / A

    def voltage_drop_v(self, current_a: float, temp_c: float = 20.0) -> float:
        return current_a * self.resistance_ohm(temp_c)

    # ---- mass (exact copper, this is the authoritative number) ----------- #
    def copper_mass_g(self) -> float:
        """Exact copper mass of the conductor: area × cut length × density."""
        A = awg_area_mm2(self.gauge_awg) * 1e-6      # m^2
        L = self.cut_length_mm() * 1e-3              # m
        return CU_DENSITY_KG_M3 * A * L * 1000.0     # -> grams

    def insulation_mass_g(self) -> float:
        """Approx jacket mass (annulus between conductor and OD). Reported
        separately from copper so the copper number stays authoritative."""
        od = self.outside_diameter_mm * 1e-3
        a_out = np.pi * (od / 2.0) ** 2
        a_cu = awg_area_mm2(self.gauge_awg) * 1e-6
        a_ins = max(a_out - a_cu, 0.0)
        L = self.cut_length_mm() * 1e-3
        return INSUL_DENSITY_KG_M3 * a_ins * L * 1000.0

    def mass_g(self) -> float:
        """Total conductor mass = copper + insulation."""
        return self.copper_mass_g() + self.insulation_mass_g()

    def centroid_mm(self) -> Optional[np.ndarray]:
        """Length-weighted centroid of the taut route in car coordinates — the
        point the wire's mass acts through (uniform linear density)."""
        path = self.as_polyline()
        if path.shape[0] < 2:
            return None
        seg = segment_lengths_mm(path)
        mids = (path[:-1] + path[1:]) / 2.0
        total = float(np.sum(seg))
        if total <= 0:
            return None
        return np.sum(mids * seg[:, None], axis=0) / total

    def as_dict(self):
        return asdict(self)

    @staticmethod
    def from_dict(d) -> "WireRun":
        d = dict(d)
        if isinstance(d.get("path_mm"), list):
            d["path_mm"] = [tuple(p) for p in d["path_mm"]]
        valid = WireRun.__dataclass_fields__.keys()
        return WireRun(**{k: v for k, v in d.items() if k in valid})


# --------------------------------------------------------------------------- #
#  Clearance vs the same keep-outs the mount-points use
# --------------------------------------------------------------------------- #
def _point_aabb_signed_dist_mm(p: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> float:
    """Exact signed distance point->AABB (same construction as KeepOut). >0 out,
    <0 inside (penetration depth)."""
    d_out = np.maximum(np.maximum(lo - p, p - hi), 0.0)
    outside = float(np.linalg.norm(d_out))
    if outside > 0.0:
        return outside
    inside = float(np.min(np.minimum(p - lo, hi - p)))
    return -inside


def _signed_dist_batch(pts: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Vectorised signed point→AABB distance for an (N,3) array of points.
    >0 outside, <0 inside (negative penetration depth), 0 on the surface."""
    d_out = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
    outside = np.linalg.norm(d_out, axis=1)
    inside = np.min(np.minimum(pts - lo, hi - pts), axis=1)
    return np.where(outside > 0.0, outside, -np.maximum(inside, 0.0))


def _polyline_aabb_clearance_detail(path: np.ndarray, lo: np.ndarray,
                                    hi: np.ndarray):
    """
    Minimum signed clearance between a routed polyline and an axis-aligned
    keep-out box, plus WHERE on the route it occurs.

    The signed distance to a convex set is a convex function, so along each
    straight segment d(t) is convex in t — its minimum is found *exactly* (to
    machine precision) by golden-section search, run vectorised across every
    segment at once. This replaces the old 5 mm point-sampling walk: it is both
    exact (no sample-spacing error — a wire that just grazes a corner between
    two samples can no longer slip through) and orders of magnitude faster
    (a few dozen fully-vectorised numpy evaluations instead of a Python loop
    per 5 mm of route).

    Returns (worst_signed_mm, worst_point_xyz or None). Negative => the wire
    passes through the box; the point is the deepest penetration / closest
    approach on the centreline.
    """
    if path.shape[0] < 2:
        return float("inf"), None
    P0 = path[:-1].astype(float)                 # (M,3) segment starts
    D = path[1:].astype(float) - P0              # (M,3) segment vectors
    M = P0.shape[0]
    invphi = (np.sqrt(5.0) - 1.0) / 2.0          # 0.618...
    a = np.zeros(M)
    b = np.ones(M)
    c = b - invphi * (b - a)
    d = a + invphi * (b - a)
    fc = _signed_dist_batch(P0 + D * c[:, None], lo, hi)
    fd = _signed_dist_batch(P0 + D * d[:, None], lo, hi)
    # 72 golden steps shrink the bracket by 0.618^72 ≈ 8e-16 of the segment.
    for _ in range(72):
        take_left = fc < fd            # True: min lies in [a, d]
        fc_old, fd_old = fc, fd
        b = np.where(take_left, d, b)
        a = np.where(take_left, a, c)
        c_next = np.where(take_left, b - invphi * (b - a), d)
        d_next = np.where(take_left, c, a + invphi * (b - a))
        probe = np.where(take_left, c_next, d_next)
        f_probe = _signed_dist_batch(P0 + D * probe[:, None], lo, hi)
        fc = np.where(take_left, f_probe, fd_old)
        fd = np.where(take_left, fc_old, f_probe)
        c, d = c_next, d_next
    t = (a + b) / 2.0
    pts = P0 + D * t[:, None]
    vals = _signed_dist_batch(pts, lo, hi)
    # segment endpoints are candidates too (convex min can sit at t=0/1;
    # golden converges there as well, but include the exact vertices for free)
    vend = _signed_dist_batch(path.astype(float), lo, hi)
    i_seg = int(np.argmin(vals))
    i_end = int(np.argmin(vend))
    if vend[i_end] < vals[i_seg]:
        return float(vend[i_end]), path[i_end].astype(float)
    return float(vals[i_seg]), pts[i_seg]


def _polyline_aabb_clearance_mm(path: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                                step_mm: float = 5.0) -> float:
    """Minimum signed clearance polyline→AABB (kept for API compatibility;
    `step_mm` is ignored — the answer is now exact, not sampled).
    Negative => the wire passes through the box."""
    gap, _ = _polyline_aabb_clearance_detail(path, lo, hi)
    return gap


# --------------------------------------------------------------------------- #
#  Formboard (1:1 unfolded 2-D layout) data
# --------------------------------------------------------------------------- #
@dataclass
class FormboardBranch:
    """One conductor as it appears on the flat nail-board: an ordered list of 2-D
    (x,y) points whose *segment lengths exactly equal the 3-D route's* segment
    lengths, so the board is a true 1:1 manufacturing layout.

    `corner_radii_mm` is the formed bend radius the 3-D route implies at each
    internal vertex (same order as the drawn points), and `min_bend_radius_mm`
    the wire's allowed minimum — so the drawing can flag, on the board itself,
    exactly which nail position holds a bend the conductor cannot take."""
    wire: str
    net: str
    gauge_awg: int
    points_mm: list                 # [(x,y), ...] flat
    cut_length_mm: float
    from_conn: str = ""
    to_conn: str = ""
    corner_radii_mm: list = field(default_factory=list)   # per internal vertex
    min_bend_radius_mm: float = 0.0
    reversed: bool = False          # drawn to→from instead of from→to

    def as_dict(self):
        return asdict(self)


@dataclass
class Formboard:
    """
    The whole harness unfolded flat. `branches` are the per-wire 2-D polylines,
    `nodes` are the connector positions on the board (each connector appears
    exactly ONCE), `ties` mark where a second length-true branch terminating at
    an already-placed connector necessarily ends elsewhere on the board (drawn
    as a dashed 'same plug' link — honest, not warped), and `extent_mm` is the
    bounding box of the drawing (the physical board size the shop needs).
    """
    branches: list = field(default_factory=list)
    nodes: dict = field(default_factory=dict)       # connector -> (x,y)
    extent_mm: tuple = (0.0, 0.0)
    ties: list = field(default_factory=list)        # [{connector, a_xy, b_xy}]

    def as_dict(self):
        return dict(branches=[b.as_dict() for b in self.branches],
                    nodes={k: list(v) for k, v in self.nodes.items()},
                    extent_mm=list(self.extent_mm),
                    ties=[dict(connector=t["connector"],
                               a_xy=list(t["a_xy"]), b_xy=list(t["b_xy"]))
                          for t in self.ties])


def _ang_diff(a: float, b: float) -> float:
    """Smallest absolute difference between two angles (radians)."""
    d = (a - b) % (2.0 * np.pi)
    return min(d, 2.0 * np.pi - d)


def _unfold_branch_2d(path3d: np.ndarray, origin2d: np.ndarray,
                      heading_deg: float,
                      target_heading_deg: Optional[float] = None) -> np.ndarray:
    """
    Unfold a 3-D centreline into a flat 2-D polyline that PRESERVES every segment
    length exactly. The branch launches along `heading_deg` and turns at each
    vertex by the *same interior angle* the 3-D route turns through — an
    isometric (length-true) unrolling: the bends are real, only the out-of-plane
    component is removed.

    Only the *sign* of each flat turn is free (the 3-D route fixes the
    magnitude). The old implementation blindly alternated the sign, which made
    real routes zig-zag, curl back over themselves and cross other branches —
    the 'drawing doesn't go right' failure. Here each turn's sign is chosen to
    STEER the running heading toward `target_heading_deg` (default: the launch
    heading), so the branch flows outward in its own lane while every segment
    length and every bend angle stay exactly true.
    """
    seg = segment_lengths_mm(path3d)
    if seg.shape[0] == 0:
        return origin2d.reshape(1, 2)
    turns = turn_angles_deg(path3d)        # interior deflection at each vertex
    target = np.radians(heading_deg if target_heading_deg is None
                        else target_heading_deg)
    pts = [origin2d.copy()]
    heading = np.radians(heading_deg)
    cur = origin2d.copy()
    for i, L in enumerate(seg):
        d = np.array([np.cos(heading), np.sin(heading)])
        cur = cur + d * L
        pts.append(cur.copy())
        if i < turns.shape[0]:
            t = np.radians(turns[i])
            plus, minus = heading + t, heading - t
            heading = plus if _ang_diff(plus, target) <= _ang_diff(minus, target) \
                else minus
    return np.asarray(pts)


# --------------------------------------------------------------------------- #
#  The harness ledger
# --------------------------------------------------------------------------- #
@dataclass
class HarnessLedger:
    """
    The routed loom: connectors + wire runs, with the bench checks (bend radius,
    strain relief, 3-D clearance vs keep-outs) and the manufacturing roll-ups
    (cut length, formboard, BOM, copper mass + distribution).

    Keep-outs are passed IN from the geometry ledger at check time (same boxes the
    mount-point clash uses) rather than duplicated here — one source of truth for
    "what volume is reserved".
    """
    connectors: dict = field(default_factory=dict)   # name -> Connector
    wires: dict = field(default_factory=dict)         # name -> WireRun
    ambient_c: float = 40.0
    clearance_warn_mm: float = 10.0                    # gap that triggers WARN
    clearance_fail_mm: float = 0.0                     # gap that triggers FAIL (touch/through)

    # ---- mutators -------------------------------------------------------- #
    def set_connector(self, c: Connector):
        self.connectors[c.name] = c

    def set_wire(self, w: WireRun):
        self.wires[w.name] = w

    def remove_connector(self, name: str):
        self.connectors.pop(name, None)

    def remove_wire(self, name: str):
        self.wires.pop(name, None)

    # ---- bend-radius + strain-relief bench checks ------------------------ #
    def check_bends(self) -> list:
        """
        For every wire: does any formed corner bend tighter than the conductor's
        minimum bend radius (kinks it), and does each terminated end leave the
        connector with the required straight strain-relief length before its first
        bend?
        """
        out: list = []
        for w in self.wires.values():
            path = w.as_polyline()
            est = w.is_estimate
            tag = " (estimated route)" if est else ""
            owners = sorted({w.owner_subsystem,
                             *(self.connectors[c].owner_subsystem
                               for c in (w.from_conn, w.to_conn)
                               if c in self.connectors)})
            if path.shape[0] < 2:
                out.append(Finding(
                    "harness-route", Severity.MISSING,
                    f"Wire '{w.name}' has no routed path — cannot check bends, "
                    f"clearance, or cut length yet.",
                    subsystems=owners or [w.owner_subsystem]))
                continue

            # --- minimum bend radius at every corner --- #
            radii = vertex_bend_radius_mm(path)
            r_min_allowed = w.min_bend_radius_mm
            if radii.size:
                worst_r = float(np.min(radii))
                if worst_r < r_min_allowed:
                    out.append(Finding(
                        "harness-bend", Severity.FAIL,
                        f"Wire '{w.name}' (AWG{w.gauge_awg}, {w.outside_diameter_mm:.1f} mm OD) "
                        f"is bent to {worst_r:.0f} mm radius at its sharpest corner, "
                        f"tighter than its {r_min_allowed:.0f} mm minimum "
                        f"({w.bundle_min_radius_mult:.0f}× OD) — this kinks the "
                        f"conductor / cracks shield. Ease the corner or add a "
                        f"sweep{tag}.",
                        subsystems=owners,
                        detail=dict(wire=w.name, formed_radius_mm=worst_r,
                                    min_radius_mm=r_min_allowed, od_mm=w.outside_diameter_mm,
                                    estimate=est)))
                elif worst_r < 1.5 * r_min_allowed:
                    out.append(Finding(
                        "harness-bend", Severity.WARN,
                        f"Wire '{w.name}' bends to {worst_r:.0f} mm radius — within "
                        f"1.5× of its {r_min_allowed:.0f} mm minimum; thin margin "
                        f"on the tightest corner{tag}.",
                        subsystems=owners,
                        detail=dict(wire=w.name, formed_radius_mm=worst_r,
                                    min_radius_mm=r_min_allowed, estimate=est)))

            # --- strain relief: straight run leaving each connector --- #
            seg = segment_lengths_mm(path)
            for end_label, conn_name, first_seg in (
                    ("from", w.from_conn, seg[0] if seg.size else 0.0),
                    ("to", w.to_conn, seg[-1] if seg.size else 0.0)):
                conn = self.connectors.get(conn_name)
                if conn is None:
                    continue
                req = conn.strain_relief_mm
                if first_seg < req - 1e-6:
                    out.append(Finding(
                        "harness-strain-relief", Severity.WARN,
                        f"Wire '{w.name}' leaves connector '{conn_name}' with only "
                        f"{first_seg:.0f} mm straight before its first bend — less "
                        f"than the {req:.0f} mm strain-relief / backshell zone. The "
                        f"wire will flex at the crimp and fatigue the contact{tag}.",
                        subsystems=sorted({w.owner_subsystem, conn.owner_subsystem}),
                        detail=dict(wire=w.name, connector=conn_name, end=end_label,
                                    straight_mm=float(first_seg),
                                    required_mm=req, estimate=est)))
        return out

    # ---- route anchoring: does the polyline actually reach its plugs? ---- #
    def check_anchoring(self, tol_mm: float = 1.0) -> list:
        """
        Every derived number — cut length, formboard, clearance — is measured off
        the declared polyline. If the polyline doesn't actually start/end at the
        connector face it claims to plug into, all of those numbers are measured
        off the wrong geometry (the classic 'wire floats 80 mm off the ECU in the
        3-D view' symptom). Flag any terminated end whose route endpoint sits more
        than `tol_mm` from its connector's declared position.
        """
        out: list = []
        for w in self.wires.values():
            path = w.as_polyline()
            if path.shape[0] < 2:
                continue
            est = w.is_estimate
            tag = " (estimated route)" if est else ""
            for end_label, conn_name, pt in ((w.from_conn, w.from_conn, path[0]),
                                             (w.to_conn, w.to_conn, path[-1])):
                conn = self.connectors.get(conn_name)
                if conn is None:
                    continue
                gap = float(np.linalg.norm(pt - conn.as_array()))
                if gap > tol_mm:
                    out.append(Finding(
                        "harness-anchor", Severity.WARN,
                        f"Wire '{w.name}' claims to terminate at connector "
                        f"'{conn_name}' but its route endpoint sits {gap:.0f} mm "
                        f"away from the connector face — the cut length, "
                        f"formboard and clearance are being measured off a route "
                        f"that never reaches the plug. Snap the endpoint to the "
                        f"connector{tag}.",
                        subsystems=sorted({w.owner_subsystem,
                                           conn.owner_subsystem}),
                        detail=dict(wire=w.name, connector=conn_name,
                                    offset_mm=round(gap, 1),
                                    endpoint_mm=[round(float(v), 1) for v in pt],
                                    connector_mm=[round(float(v), 1)
                                                  for v in conn.as_array()],
                                    estimate=est)))
        return out

    # ---- 3-D clearance vs keep-outs -------------------------------------- #
    def check_clearance(self, keepouts: Optional[list] = None) -> list:
        """
        Route every wire past the keep-out volumes the rest of the car reserves
        (the same AABB boxes the mount-point clash checks). A wire through a box is
        a FAIL with both owners named; a wire within the warn band is a WARN.

        `keepouts` is a list of objects exposing `.lo_mm`, `.hi_mm`,
        `.owner_subsystem`, `.name` — i.e. the geometry ledger's KeepOut objects,
        passed straight in. None / empty => a single MISSING note (cannot check
        clearance with nothing to clear).
        """
        out: list = []
        keepouts = keepouts or []
        if not self.wires:
            return out
        if not keepouts:
            out.append(Finding(
                "harness-clearance", Severity.MISSING,
                "No keep-out volumes supplied — cannot check the harness route "
                "against reserved space. Declare chassis / suspension / "
                "accumulator keep-outs in the geometry ledger.",
                subsystems=["electrics"]))
            return out
        for w in self.wires.values():
            path = w.as_polyline()
            if path.shape[0] < 2:
                continue
            est = w.is_estimate
            tag = " (estimated route)" if est else ""
            for ko in keepouts:
                lo = np.asarray(ko.lo_mm, float)
                hi = np.asarray(ko.hi_mm, float)
                # inflate the box by half the wire OD: the centreline must clear by
                # the conductor's own radius before its surface touches.
                pad = w.outside_diameter_mm / 2.0
                gap, at = _polyline_aabb_clearance_detail(path, lo - pad, hi + pad)
                at_mm = None if at is None else [round(float(v), 1) for v in at]
                ko_owner = getattr(ko, "owner_subsystem", "?")
                pair = sorted({w.owner_subsystem, ko_owner})
                if gap <= self.clearance_fail_mm:
                    out.append(Finding(
                        "harness-clearance", Severity.FAIL,
                        f"Wire '{w.name}' passes through {ko_owner}'s reserved "
                        f"volume '{ko.name}' (penetration {-gap:.0f} mm past the "
                        f"wire surface) — the loom fouls it. Re-route{tag}.",
                        subsystems=pair,
                        detail=dict(wire=w.name, keepout=ko.name,
                                    penetration_mm=float(-gap), at_mm=at_mm,
                                    estimate=est)))
                elif gap < self.clearance_warn_mm:
                    out.append(Finding(
                        "harness-clearance", Severity.WARN,
                        f"Wire '{w.name}' clears {ko_owner}'s '{ko.name}' by only "
                        f"{gap:.0f} mm (wants {self.clearance_warn_mm:.0f} mm) — "
                        f"tight against reserved space{tag}.",
                        subsystems=pair,
                        detail=dict(wire=w.name, keepout=ko.name,
                                    gap_mm=float(gap), at_mm=at_mm,
                                    estimate=est)))
        if not any(f.severity in (Severity.FAIL, Severity.WARN) for f in out) and out:
            out.append(Finding(
                "harness-clearance", Severity.OK,
                "Every routed wire clears all reserved keep-out volumes with "
                "margin.", subsystems=["electrics"]))
        return out

    # ---- cut length to the millimetre ------------------------------------ #
    def cut_list(self) -> list:
        """
        The cut list the fabricator works to: every wire, its exact cut length to
        the mm, broken into its terms so the number is auditable.
        """
        rows = []
        for w in self.wires.values():
            path = w.as_polyline()
            base = polyline_length_mm(path)
            turns = np.radians(turn_angles_deg(path))
            radii = vertex_bend_radius_mm(path)
            r_cap = w.min_bend_radius_mm
            bend = sum(float(min(R, r_cap) * th) for th, R in zip(turns, radii)
                       if np.isfinite(R) and th > 0)
            rows.append(dict(
                wire=w.name, net=w.net, gauge_awg=w.gauge_awg,
                from_conn=w.from_conn, to_conn=w.to_conn,
                routed_mm=round(base, 1),
                bend_allowance_mm=round(bend, 1),
                service_loop_mm=round(w.service_loop_mm, 1),
                strip_both_ends_mm=round(2.0 * w.strip_mm, 1),
                cut_length_mm=round(w.cut_length_mm(), 1),
            ))
        rows.sort(key=lambda r: r["wire"])
        return rows

    # ---- automated BOM --------------------------------------------------- #
    def bom(self) -> dict:
        """
        The automated bill of materials, rolled straight off the declared parts:
          * wire: total length per gauge (m) and copper mass per gauge (g),
          * connectors: count per part number, with cavity (contact) totals,
          * contacts: total crimp contacts = sum of wire terminations actually
            landed in connectors.
        Everything here is counted, not estimated.
        """
        wire_by_gauge: dict = {}
        for w in self.wires.values():
            g = w.gauge_awg
            agg = wire_by_gauge.setdefault(g, dict(length_mm=0.0, copper_g=0.0,
                                                   insul_g=0.0, count=0))
            agg["length_mm"] += w.cut_length_mm()
            agg["copper_g"] += w.copper_mass_g()
            agg["insul_g"] += w.insulation_mass_g()
            agg["count"] += 1

        conn_by_pn: dict = {}
        contacts = 0
        for w in self.wires.values():
            for c in (w.from_conn, w.to_conn):
                if c in self.connectors:
                    contacts += 1
        for c in self.connectors.values():
            pn = c.part_number or f"(unspecified:{c.name})"
            agg = conn_by_pn.setdefault(pn, dict(count=0, cavities=0,
                                                 mass_g=0.0, mass_known=True))
            agg["count"] += 1
            agg["cavities"] += c.cavities
            if c.mass_g is None:
                agg["mass_known"] = False
            else:
                agg["mass_g"] += c.mass_g

        wire_rows = [dict(gauge_awg=g,
                          length_m=round(v["length_mm"] / 1000.0, 3),
                          copper_g=round(v["copper_g"], 1),
                          insulation_g=round(v["insul_g"], 1),
                          conductors=v["count"])
                     for g, v in sorted(wire_by_gauge.items())]
        conn_rows = [dict(part_number=pn, qty=v["count"], cavities=v["cavities"],
                          mass_g=(round(v["mass_g"], 1) if v["mass_known"] else None))
                     for pn, v in sorted(conn_by_pn.items())]
        return dict(
            wire=wire_rows,
            connectors=conn_rows,
            contacts_total=contacts,
            total_wire_m=round(sum(r["length_m"] for r in wire_rows), 3),
            total_copper_g=round(sum(r["copper_g"] for r in wire_rows), 1),
        )

    # ---- copper mass + distribution (CG) --------------------------------- #
    def mass_distribution(self) -> dict:
        """
        The exact harness mass and where it sits. Per wire: copper mass, total
        mass, and its centroid in car coordinates. Combined: total copper mass,
        total harness mass (wire + connector bodies where declared), and the
        mass-weighted CG of the loom — the number that tells you how the harness
        loads the car's balance, known before a wire is cut.

        Connector-body mass is folded into the CG when declared; connectors with
        unknown mass are listed so the number is never silently completed.
        """
        per_wire = []
        m_acc = 0.0                       # total mass for CG (g)
        moment = np.zeros(3)              # mass-weighted position (g·mm)
        cu_total = 0.0
        for w in self.wires.values():
            cu = w.copper_mass_g()
            m = w.mass_g()
            cu_total += cu
            cen = w.centroid_mm()
            per_wire.append(dict(
                wire=w.name, net=w.net, gauge_awg=w.gauge_awg,
                copper_g=round(cu, 2), total_g=round(m, 2),
                centroid_mm=(None if cen is None else [round(float(x), 1) for x in cen]),
            ))
            if cen is not None:
                m_acc += m
                moment += m * cen

        unknown_conn = []
        for c in self.connectors.values():
            if c.mass_g is None:
                unknown_conn.append(c.name)
            else:
                m_acc += c.mass_g
                moment += c.mass_g * c.as_array()

        cg = (moment / m_acc).tolist() if m_acc > 0 else None
        return dict(
            per_wire=sorted(per_wire, key=lambda r: r["wire"]),
            total_copper_g=round(cu_total, 1),
            total_harness_g=round(m_acc, 1),
            harness_cg_mm=(None if cg is None else [round(v, 1) for v in cg]),
            connectors_without_declared_mass=sorted(unknown_conn),
            note=("harness_cg includes every connector whose mass is declared; "
                  "connectors_without_declared_mass are excluded from the CG until "
                  "their mass is given — never assumed."),
        )

    # ---- formboard (1:1 unfolded) ---------------------------------------- #
    def formboard(self) -> Formboard:
        """
        Build the 1:1 flat formboard as a topology-aware tree layout:

          * the harness connector graph is traversed from each component's hub
            connector outward, so every connector appears at exactly ONE board
            position (previously a shared plug was silently drawn wherever the
            last branch happened to end);
          * each branch is unrolled length-true from its start connector's board
            position, launched into its own angular lane and STEERED (turn signs
            chosen toward the lane) instead of blindly zig-zagged, so branches
            flow outward and stop curling back over each other;
          * a second length-true branch that terminates at an already-placed
            connector gets an explicit dashed `tie` (same plug, drawn honestly
            where its true length puts it — never warped to fit);
          * disconnected sub-harnesses are tiled side by side.

        Segment lengths and bend magnitudes remain exactly those of the 3-D
        route — the 1:1 manufacturing guarantee is unchanged. The returned
        extent is the physical board size.
        """
        routed = [w for w in sorted(self.wires.values(), key=lambda x: x.name)
                  if w.as_polyline().shape[0] >= 2]
        branches: list = []
        ties: list = []
        all_pts: list = []
        placed: dict = {}        # node key -> np.array([x, y]) on the board
        node_head: dict = {}     # node key -> outgoing base heading (deg)
        fan_count: dict = {}     # node key -> how many branches already launched
        # symmetric fan: successive branches from the same node take these
        # offsets (deg) around the node's base heading, so siblings get lanes
        # instead of piling onto one line
        _FAN = [0.0, 38.0, -38.0, 76.0, -76.0, 114.0, -114.0, 152.0, -152.0]

        # degree of each named connector, to root each component at its hub
        deg: dict = {}
        for w in routed:
            for cn in (w.from_conn, w.to_conn):
                if cn:
                    deg[cn] = deg.get(cn, 0) + 1

        remaining = list(routed)
        while remaining:
            # grow the tree: prefer a wire touching an already-placed connector
            idx = next((i for i, w in enumerate(remaining)
                        if (w.from_conn and w.from_conn in placed)
                        or (w.to_conn and w.to_conn in placed)), None)
            if idx is None:
                # new disconnected component: seed it at the hub connector of
                # its own wires (highest degree endpoint), tiled to the right
                # of everything drawn so far so components never overlap
                w0 = remaining[0]
                cands = [c for c in (w0.from_conn, w0.to_conn) if c]
                seed = (max(cands, key=lambda c: deg.get(c, 0))
                        if cands else f"\u00b7{w0.name}")
                ox = (float(np.vstack(all_pts)[:, 0].max()) + 300.0
                      if all_pts else 0.0)
                placed[seed] = np.array([ox, 0.0])
                node_head[seed] = 0.0
                idx = 0
                w = remaining.pop(0)
                start_key = seed
                rev = bool(w.to_conn and w.to_conn == seed
                           and w.from_conn != seed)
            else:
                w = remaining.pop(idx)
                if w.from_conn and w.from_conn in placed:
                    start_key, rev = w.from_conn, False
                else:
                    start_key, rev = w.to_conn, True

            path = w.as_polyline()
            if rev:
                path = path[::-1]
            other = (w.from_conn if rev else w.to_conn) or ""

            k = fan_count.get(start_key, 0)
            fan_count[start_key] = k + 1
            heading = node_head.get(start_key, 0.0) + _FAN[k % len(_FAN)]
            flat = _unfold_branch_2d(path, placed[start_key], heading)

            radii = vertex_bend_radius_mm(path)
            branches.append(FormboardBranch(
                wire=w.name, net=w.net, gauge_awg=w.gauge_awg,
                points_mm=[tuple(round(float(v), 1) for v in p) for p in flat],
                cut_length_mm=round(w.cut_length_mm(), 1),
                from_conn=w.from_conn, to_conn=w.to_conn,
                corner_radii_mm=[(None if not np.isfinite(r)
                                  else round(float(r), 1)) for r in radii],
                min_bend_radius_mm=round(w.min_bend_radius_mm, 1),
                reversed=rev))
            all_pts.append(flat)

            end = flat[-1]
            if other:
                if other in placed:
                    # a second length-true branch reaching this plug cannot be
                    # bent to land on it — record the honest 'same connector'
                    # tie instead of silently overwriting the node position
                    if float(np.linalg.norm(end - placed[other])) > 0.5:
                        ties.append(dict(connector=other,
                                         a_xy=tuple(float(v) for v in end),
                                         b_xy=tuple(float(v)
                                                    for v in placed[other])))
                else:
                    placed[other] = end.copy()
                    # children of this node continue outward along the arrival
                    # direction of the branch that placed it
                    tail = flat[-1] - flat[-2] if flat.shape[0] >= 2 \
                        else np.array([1.0, 0.0])
                    node_head[other] = float(np.degrees(
                        np.arctan2(tail[1], tail[0])))

        nodes = {k: tuple(float(v) for v in p) for k, p in placed.items()
                 if not k.startswith("\u00b7")}
        if all_pts:
            stacked = np.vstack(all_pts)
            mn = stacked.min(axis=0)
            mx = stacked.max(axis=0)
            extent = tuple(float(v) for v in (mx - mn))
            # shift everything positive so the board origin is the corner
            for b in branches:
                b.points_mm = [tuple(round(float(p[i] - mn[i]), 1)
                                     for i in range(2)) for p in b.points_mm]
            nodes = {k: tuple(round(float(v[i] - mn[i]), 1) for i in range(2))
                     for k, v in nodes.items()}
            ties = [dict(connector=t["connector"],
                         a_xy=tuple(round(float(t["a_xy"][i] - mn[i]), 1)
                                    for i in range(2)),
                         b_xy=tuple(round(float(t["b_xy"][i] - mn[i]), 1)
                                    for i in range(2))) for t in ties]
        else:
            extent = (0.0, 0.0)
        return Formboard(branches=branches, nodes=nodes, extent_mm=extent,
                         ties=ties)

    # ---- persistence ----------------------------------------------------- #
    def as_dict(self):
        return dict(
            connectors={k: v.as_dict() for k, v in self.connectors.items()},
            wires={k: v.as_dict() for k, v in self.wires.items()},
            ambient_c=self.ambient_c,
            clearance_warn_mm=self.clearance_warn_mm,
            clearance_fail_mm=self.clearance_fail_mm,
        )

    @staticmethod
    def from_dict(d) -> "HarnessLedger":
        d = d or {}
        hl = HarnessLedger()
        for k, v in (d.get("connectors") or {}).items():
            hl.set_connector(Connector.from_dict(v))
        for k, v in (d.get("wires") or {}).items():
            hl.set_wire(WireRun.from_dict(v))
        for sk in ("ambient_c", "clearance_warn_mm", "clearance_fail_mm"):
            if d.get(sk) is not None:
                setattr(hl, sk, d[sk])
        return hl


# --------------------------------------------------------------------------- #
#  One-call harness gate — the "before you cut a wire" check
# --------------------------------------------------------------------------- #
@dataclass
class HarnessCheckResult:
    """Bundles the harness findings + the derived manufacturing artefacts."""
    findings: list = field(default_factory=list)
    cut_list: list = field(default_factory=list)
    bom: dict = field(default_factory=dict)
    mass: dict = field(default_factory=dict)
    formboard: Optional[Formboard] = None

    def has_hard_fail(self) -> bool:
        return any(f.severity == Severity.FAIL for f in self.findings)

    def summary(self) -> str:
        from .interfaces import summarize
        s = summarize(self.findings)
        cu = self.mass.get("total_copper_g")
        wire_m = self.bom.get("total_wire_m")
        head = (f"{s['worst'].upper()}: "
                f"{s['counts'].get('fail',0)} fail / "
                f"{s['counts'].get('warning',0)} warn / "
                f"{s['counts'].get('ok',0)} ok across {len(self.findings)} harness checks")
        if cu is not None and wire_m is not None:
            head += f" · {wire_m:.2f} m wire · {cu:.0f} g copper"
        return head


def check_harness(harness: HarnessLedger,
                  keepouts: Optional[list] = None) -> HarnessCheckResult:
    """
    Run the full pre-cut harness gate: bend-radius + strain-relief on every wire,
    3-D clearance against the supplied keep-out volumes (the same boxes the
    mount-point clash uses), and roll up the cut list, BOM, copper-mass
    distribution and the 1:1 formboard — everything you want to know before
    cutting the first wire, on the same typed `Finding` surface the rest of the
    integration board renders.
    """
    findings = []
    findings += harness.check_bends()
    findings += harness.check_anchoring()
    findings += harness.check_clearance(keepouts=keepouts)
    return HarnessCheckResult(
        findings=findings,
        cut_list=harness.cut_list(),
        bom=harness.bom(),
        mass=harness.mass_distribution(),
        formboard=harness.formboard(),
    )
