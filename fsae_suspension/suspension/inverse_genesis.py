# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/inverse_genesis.py — 🧬 InverseGenesis: the stochastic inverse
#  engine. Draw the kinematic curves you want inside acceptance bands, declare
#  the legal volume each hardpoint may occupy, and the engine generates the
#  geometry — then rejects every knife-edge optimum the shop can't hold,
#  keeping the coordinates that survive the Stochastic Inversion error field.
# ============================================================================
"""
InverseGenesis — the curves are the anchor; the points get pulled into place.

WHY THIS MODULE EXISTS
----------------------
Every kinematics tool in the chain — this repo's own forward solver included —
runs the design loop in the one direction nobody actually wants: guess
coordinates, solve, read the curves, wince, guess again. Engineers spend days
of that loop translating "I want ~1° of camber gain and dead bump steer" into
x/y/z millimetres by hand, because the tools only speak coordinates.

This module runs the loop backwards. The engineer states the INTENT — target
kinematic curves over wheel travel, each with an acceptance band ("camber at
full bump: −2.4° ± 0.25°") — plus the LEGAL VOLUME each movable hardpoint may
physically occupy (a per-point box, optionally minus the keep-out volumes the
headers, mounts and bodywork already claim). The engine treats the curves as
the fixed anchor and pulls the coordinates into alignment. Three stages:

  1. THE PHYSICS-INFORMED BOUNDARY FILTER — candidates are never free points
     in space. Every step of the search is clamped to the declared per-point
     legal boxes; every accepted geometry is screened against keep-out
     volumes queried through the exact Phantom Envelope capsule arithmetic
     (any object exposing ``clearances(points, probe_radius_mm)`` works: a
     carved PhantomEnvelope of a neighbouring assembly, or the KeepOutBox
     declared here for "the header lives in this box"). A coordinate that
     hits the curves from inside an exhaust primary is not a solution; the
     filter makes it unrepresentable rather than merely penalised. The
     same wall carries SOLVED-PROPERTY BOUNDS (``PropertyBound``): anti-dive,
     anti-squat, roll-centre migration rate, caster, kingpin inclination and
     static scrub are evaluated on every trial geometry, and a step that
     leaves a declared range is refused exactly as a keep-out violation is.
     This is the difference between a search that steers around a bound and
     a screen that discards candidates afterwards: a screen can only tell
     you that no candidate it happened to produce survived, while a bound
     inside the loop lets the solver walk the feasible boundary and report
     honestly when there is nothing behind it.

  2. THE DETERMINISTIC REVERSE GRADIENTS — the inverse solve itself. Each
     iteration builds the Jacobian of the band-weighted curve residual with
     respect to the free hardpoint coordinates (central differences through
     the full nonlinear corner solver — the exact same reverse sensitivities
     backpropagation would produce, computed honestly, because the forward
     solver is fast enough to differentiate numerically) and takes a damped
     Gauss–Newton step: solve (JᵀJ + λD)Δx = −Jᵀr, clamp to the legal boxes,
     reject on keep-out contact, adapt λ on failure. A fast linear model
     proposes; the full nonlinear solver disposes. Every step is checkable
     arithmetic — no stochastic optimiser, no population magic, and the same
     seed gives byte-identical geometry every run.

  3. THE BUILD-YIELD CO-OPTIMIZER — the stage that separates this from every
     textbook inverse-kinematics routine, all of which assume the machinist
     is perfect. Multiple deterministic starts inside the legal volume yield
     a family of curve-hitting candidates, and each is then charged for its
     manufacturing fragility: the Stochastic Inversion error field (the
     asymmetric per-point per-axis weld/jig tolerances the shop actually
     holds) is propagated through the candidate's own sensitivity matrix,
     and the BUILD YIELD — the fraction of as-built cars still inside the
     SAME acceptance bands — is computed per candidate. The coupling is the
     point: a candidate's curve-fit residual consumes band width, and only
     the leftover headroom is available to absorb weld scatter. A geometry
     that nails the target dead-centre but sits on a sensitivity knife-edge
     (yield collapses when a welder pulls a tab 1.5 mm) is verdicted
     KNIFE_EDGE and REJECTED in favour of a slightly-off-centre candidate
     the shop can actually hold. The engine optimises for the car that gets
     built, not the car on screen.

THE HONEST TRICK, STATED AND PRICED
-----------------------------------
The yield per candidate is first-order propagation through one sensitivity
matrix — thousands of sampled cars in microseconds — exactly the priced
linearisation Stochastic Inversion ships. The price is MEASURED here the same
way: the winning candidate's linear yield is verified by a subsample of full
nonlinear re-solves and the pass/fail agreement is printed with the result;
below the honesty threshold the result demotes itself and says to rerun in
full. And when the declared bands, boxes and error field are JOINTLY
unsatisfiable — every curve-hitting geometry is knife-edge, or no legal
geometry reaches the curves at all — the engine says exactly that, names the
binding constraint (the limiting band, the clamped box face, the violated
keep-out), and refuses to fabricate an optimum. "Your targets and your shop
disagree" is a result, not a failure.

SCOPE, HONESTLY
---------------
* One corner, the rigid double-wishbone solver. Compliance under load is
  Ghost Topology's job; the generated geometry should be fed there next.
* Channels are interpolated from a dense warm-started travel sweep; station
  spacing finer than the sweep grid buys nothing (the grid density is set
  from the station count and reported).
* Keep-out screening tests the HARDPOINT (a sphere of ``probe_radius_mm``
  at the pickup), not the bracket around it — inflate the probe to cover
  the tab. And if the keep-out object is this corner's OWN envelope, the
  points that are endpoints of its capsules will always violate; carve the
  obstacle envelope from the neighbouring assemblies, not from the corner
  being designed. Stated here and in the report footer.
* The error field is Stochastic Inversion's, with its scope: independent
  per-point errors, build-to-fit links.
* Solved-property bounds are evaluated at the STATIC state (and, for
  roll-centre migration, as the least-squares slope of roll-centre height
  over the declared travel range). They therefore bound the property as the
  nominal geometry delivers it, not as every as-built car delivers it; the
  build-yield stage still prices only the declared curve channels. Bounding
  a property costs solver time — each trial geometry pays one extra short
  sweep — so only the properties actually bounded are computed.
* Deterministic end to end: fixed seeds drive the multi-start sampler and
  the yield sampler, so the same inputs give byte-identical geometry,
  yields and markdown.

Self-test: ``python3 -m suspension.inverse_genesis``
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field as _dcfield
from collections.abc import Sequence

from .kinematics import Hardpoints, SuspensionKinematics
from .ghost_topology import _rc_height_mm
from .kinematik_stochastic import ToleranceField, _perturbed

_AXES = ("x", "y", "z")

# --------------------------------------------------------------------------- #
#  The curve channels — the language the intent is drawn in.
# --------------------------------------------------------------------------- #
CHANNELS: tuple[str, ...] = (
    "camber_deg",     # camber vs travel (the gain curve)
    "toe_deg",        # toe vs travel (bump steer, drawn as the whole curve)
    "rc_height_mm",   # roll-centre height vs travel (migration)
    "scrub_mm",       # scrub radius vs travel
)

_CHANNEL_LABELS = {
    "camber_deg":   "camber (°)",
    "toe_deg":      "toe (°)",
    "rc_height_mm": "roll-centre height (mm)",
    "scrub_mm":     "scrub radius (mm)",
}

# The hardpoints the engine is allowed to treat as design freedoms. Wheel
# centre / contact patch are excluded on purpose — they are the tyre's
# geometry, not the linkage's; moving them changes the question, not the
# answer.
DESIGNABLE_POINTS: tuple[str, ...] = (
    "upper_front_inner", "upper_rear_inner",
    "lower_front_inner", "lower_rear_inner",
    "upper_outer", "lower_outer",
    "tie_rod_inner", "tie_rod_outer",
)


# --------------------------------------------------------------------------- #
#  The forward map: geometry → curve samples at the declared stations.
# --------------------------------------------------------------------------- #
def curves_of(hp: Hardpoints, stations_mm: np.ndarray,
              track_mm: float = 1200.0,
              n_sweep: int | None = None
              ) -> tuple[dict[str, np.ndarray], bool]:
    """Every channel sampled at the requested travel stations.

    One dense warm-started sweep covers the station range; channels are
    linearly interpolated onto the stations. A single non-converged state
    anywhere in the sweep fails the whole geometry — a corner the solver
    cannot follow is not a candidate, it's a cliff.
    """
    stations = np.asarray(stations_mm, float)
    lo, hi = float(stations.min()), float(stations.max())
    if hi - lo < 1e-9:                       # single station: give it width
        lo, hi = lo - 1.0, hi + 1.0
    n = n_sweep or max(15, 3 * len(stations) + 1)
    try:
        kin = SuspensionKinematics(hp)
        states = kin.sweep(travel_min=lo, travel_max=hi, n=n)
    except Exception:
        return {}, False
    if not states or any(not getattr(s, "converged", True) for s in states):
        return {}, False
    tr = np.array([s.travel for s in states])
    raw = {
        "camber_deg":   np.array([s.camber for s in states]),
        "toe_deg":      np.array([s.toe for s in states]),
        "rc_height_mm": np.array([_rc_height_mm(s, track_mm=track_mm)
                                  for s in states]),
        "scrub_mm":     np.array([s.scrub_radius for s in states]),
    }
    if not all(np.all(np.isfinite(v)) for v in raw.values()):
        return {}, False
    out = {ch: np.interp(stations, tr, v) for ch, v in raw.items()}
    return out, True


# --------------------------------------------------------------------------- #
#  The intent — target curves drawn inside acceptance bands.
# --------------------------------------------------------------------------- #
@dataclass
class TargetCurve:
    """One drawn curve: channel values at travel stations, each ± a band.

    ``band`` is the acceptance HALF-WIDTH per station (same units as the
    channel, > 0). The engine's definition of success is every station of
    every curve inside its band — and the band is also the currency the
    build-yield spends: fit residual consumes it, weld scatter must fit in
    what's left.
    """
    channel: str
    travel_mm: np.ndarray
    target: np.ndarray
    band: np.ndarray

    def __post_init__(self):
        if self.channel not in CHANNELS:
            raise ValueError(f"Unknown channel '{self.channel}'. "
                             f"Channels: {', '.join(CHANNELS)}.")
        self.travel_mm = np.asarray(self.travel_mm, float).ravel()
        self.target = np.asarray(self.target, float).ravel()
        self.band = np.asarray(self.band, float).ravel()
        if not (len(self.travel_mm) == len(self.target) == len(self.band)):
            raise ValueError(f"TargetCurve '{self.channel}': travel, target "
                             "and band must have equal length.")
        if len(self.travel_mm) == 0:
            raise ValueError(f"TargetCurve '{self.channel}' is empty.")
        if np.any(self.band <= 0):
            raise ValueError(f"TargetCurve '{self.channel}': every band must "
                             "be > 0 — a zero band asks for a probability-zero "
                             "car and the yield would honestly be 0.")
        order = np.argsort(self.travel_mm)
        self.travel_mm = self.travel_mm[order]
        self.target = self.target[order]
        self.band = self.band[order]


@dataclass
class GenesisTargets:
    """The full drawn intent: one or more TargetCurves."""
    curves: list[TargetCurve]
    track_mm: float = 1200.0

    def __post_init__(self):
        if not self.curves:
            raise ValueError("GenesisTargets needs at least one TargetCurve.")
        seen = set()
        for c in self.curves:
            if c.channel in seen:
                raise ValueError(f"Channel '{c.channel}' declared twice — "
                                 "merge its stations into one curve.")
            seen.add(c.channel)

    # ---- residual layout: one row per (channel, station) ------------------ #
    def rows(self) -> list[tuple[str, float]]:
        """Returns (channel, travel) pairs with travel in mm."""
        return [(c.channel, float(t)) for c in self.curves
                for t in c.travel_mm]

    def stations(self) -> np.ndarray:
        """Travel stations in mm."""
        return np.unique(np.concatenate([c.travel_mm for c in self.curves]))

    def target_vec(self) -> np.ndarray:
        """Target values in deg or mm, channel-native units."""
        return np.concatenate([c.target for c in self.curves])

    def band_vec(self) -> np.ndarray:
        """Acceptance half-widths in deg or mm, channel-native units."""
        return np.concatenate([c.band for c in self.curves])

    def residual(self, hp: Hardpoints) -> tuple[np.ndarray, bool]:
        """Band-weighted residual r: |r_i| ≤ 1 means station i is inside its
        band. NaNs (with ok=False) when the geometry doesn't solve.

        Rows normalised by their band, so the result is dimensionless.
        """
        vals, ok = curves_of(hp, self.stations(), track_mm=self.track_mm)
        if not ok:
            return np.full(len(self.rows()), np.nan), False
        parts = []
        for c in self.curves:
            v = np.interp(c.travel_mm, self.stations(), vals[c.channel])
            parts.append((v - c.target) / c.band)
        return np.concatenate(parts), True

    def row_labels(self) -> list[str]:
        """Labels carry the channel unit and the station in mm."""
        return [f"{_CHANNEL_LABELS[ch]} @ {t:+.1f} mm" for ch, t in self.rows()]


# --------------------------------------------------------------------------- #
#  The physics-informed boundary filter — legal volume + keep-out.
# --------------------------------------------------------------------------- #
@dataclass
class KeepOutBox:
    """An axis-aligned obstacle in corner axes (mm) — "the header lives here".

    Speaks the same query dialect as PhantomEnvelope: ``clearances(points,
    probe_radius_mm)`` returns signed skin clearance, + clear / − penetrating,
    so the boundary filter treats a hand-declared box and a carved envelope
    identically.
    """
    lo: np.ndarray
    hi: np.ndarray
    label: str = "keep-out box"

    def __post_init__(self):
        self.lo = np.asarray(self.lo, float).reshape(3)
        self.hi = np.asarray(self.hi, float).reshape(3)
        if np.any(self.hi <= self.lo):
            raise ValueError(f"KeepOutBox '{self.label}': hi must exceed lo "
                             "on every axis.")

    def clearances(self, points, probe_radius_mm: float = 0.0) -> np.ndarray:
        """Signed skin clearance in mm, positive clear and negative penetrating."""
        pts = np.asarray(points, float)
        if pts.ndim == 1:
            pts = pts[None, :]
        # outside: Euclidean distance to the box; inside: −(distance to the
        # nearest face). The standard signed AABB distance, closed form.
        d_out = np.maximum(np.maximum(self.lo - pts, pts - self.hi), 0.0)
        outside = np.linalg.norm(d_out, axis=1)
        d_in = np.minimum(pts - self.lo, self.hi - pts).min(axis=1)
        inside = np.where(np.all((pts >= self.lo) & (pts <= self.hi), axis=1),
                          -d_in, 0.0)
        return outside + inside - probe_radius_mm


@dataclass
class PointSpacing:
    """A relation BETWEEN two hardpoints that no per-point box can express.

    Requires ``coord(b) - coord(a) >= min_gap_mm`` along ``axis`` ("x", "y"
    or "z"), or — with ``axis="dist"`` — the Euclidean distance |b - a| to be
    at least ``min_gap_mm``. Along an axis this is an ORDERING as well as a
    base length: "lower_front_inner ahead of lower_rear_inner by >= 150 mm"
    is ``PointSpacing("lower_front_inner", "lower_rear_inner", "x", 150)``
    in corner axes (x rearward). Enforced exactly like a keep-out: a step
    that breaks it is refused, not penalised.
    """
    a: str
    b: str
    axis: str = "x"
    min_gap_mm: float = 0.0
    label: str = ""

    def __post_init__(self):
        for n in (self.a, self.b):
            if n not in PERTURBABLE_OR_FIXED:
                raise ValueError(f"PointSpacing: unknown hardpoint '{n}'.")
        if self.axis not in ("x", "y", "z", "dist"):
            raise ValueError("PointSpacing.axis must be x, y, z or dist.")
        self.min_gap_mm = float(self.min_gap_mm)
        if not self.label:
            rel = ("|b-a|" if self.axis == "dist"
                   else f"{self.b}.{self.axis} - {self.a}.{self.axis}")
            self.label = f"spacing {rel} >= {self.min_gap_mm:g} mm"

    def gap(self, hp: Hardpoints) -> float:
        """Signed gap in mm along the declared axis."""
        pa = np.asarray(getattr(hp, self.a), float)
        pb = np.asarray(getattr(hp, self.b), float)
        if self.axis == "dist":
            return float(np.linalg.norm(pb - pa))
        k = _AXES.index(self.axis)
        return float(pb[k] - pa[k])


PERTURBABLE_OR_FIXED: tuple[str, ...] = DESIGNABLE_POINTS + (
    "wheel_center", "contact_patch")


# --------------------------------------------------------------------------- #
#  Solved-property bounds — the properties the curve channels do not carry.
# --------------------------------------------------------------------------- #
#  A channel is a curve the engineer draws. A solved property is a number the
#  geometry happens to produce. Anti-squat, roll-centre migration and caster
#  are all in the second class, and an inverse solver is indifferent to any
#  property it is not told about: it will happily return a corner that hits
#  every drawn curve with -5 deg of caster, -102% anti-squat, or a roll centre
#  that runs away under heave. Declaring a PropertyBound moves that property
#  onto the same wall as the keep-out volumes.
SOLVED_PROPERTIES: tuple[str, ...] = (
    "anti_dive_pct",           # front, at the static state, %
    "anti_squat_pct",          # rear, at the static state, %
    "caster_deg",              # static caster angle, deg
    "kpi_deg",                 # static kingpin inclination, deg
    "scrub_static_mm",         # static scrub radius, mm
    "rc_migration_mm_per_mm",  # d(roll-centre height)/d(travel), chassis frame
)

_PROPERTY_LABELS = {
    "anti_dive_pct":          "anti-dive (%)",
    "anti_squat_pct":         "anti-squat (%)",
    "caster_deg":             "caster (deg)",
    "kpi_deg":                "kingpin inclination (deg)",
    "scrub_static_mm":        "scrub radius, static (mm)",
    "rc_migration_mm_per_mm": "roll-centre migration (mm/mm)",
}

#: properties that need the (more expensive) side-view path slope
_PITCH_PROPERTIES = frozenset({"anti_dive_pct", "anti_squat_pct"})

#: how many random shifts to draw per requested start when hunting for one
#: that already satisfies the declared property bounds
_MAX_START_DRAWS_PER_START = 40


@dataclass
class PropertyBound:
    """A declared range on a SOLVED property — a wall, not a penalty.

    ``lo``/``hi`` are inclusive; either may be left at infinity for a
    one-sided bound. A trial geometry whose property falls outside the range,
    or whose property is not finite (a side-view instant centre at infinity,
    a degenerate front-view IC), refuses the step exactly as a keep-out
    violation does.

    Declaring only a lower bound is the common mistake and the module says so
    rather than silently allowing it: ``PropertyBound("anti_squat_pct",
    lo=23.0)`` lets the solver walk to +116% because nothing stops it. Bound
    the band you actually want.
    """
    prop: str
    lo: float = float("-inf")
    hi: float = float("inf")
    label: str = ""

    def __post_init__(self):
        if self.prop not in SOLVED_PROPERTIES:
            raise ValueError(
                f"PropertyBound: unknown property '{self.prop}'. Allowed: "
                f"{', '.join(SOLVED_PROPERTIES)}.")
        self.lo = float(self.lo)
        self.hi = float(self.hi)
        if self.hi < self.lo:
            raise ValueError(f"PropertyBound '{self.prop}': hi < lo.")
        if self.lo == float("-inf") and self.hi == float("inf"):
            raise ValueError(
                f"PropertyBound '{self.prop}': declare at least one side; an "
                "unbounded bound is not a constraint.")
        if not self.label:
            name = _PROPERTY_LABELS[self.prop]
            if self.lo == float("-inf"):
                self.label = f"{name} <= {self.hi:g}"
            elif self.hi == float("inf"):
                self.label = f"{name} >= {self.lo:g}"
            else:
                self.label = f"{self.lo:g} <= {name} <= {self.hi:g}"

    def margin(self, value: float) -> float:
        """Signed slack: >= 0 satisfied, < 0 by how much it is violated.

        A non-finite property is reported as a violation of -inf rather than
        quietly passing, because "this geometry has no side-view instant
        centre" is not the same as "this geometry meets your anti-squat".

        Units follow the bound: mm/mm for migration, deg for angles, percent for anti-effects.
        """
        v = float(value)
        if not np.isfinite(v):
            return float("-inf")
        return float(min(v - self.lo, self.hi - v))


def properties_of(hp: Hardpoints, ctx: "SolvedPropertyBounds",
                  only: Sequence[str] | None = None
                  ) -> dict[str, float] | None:
    """Every solved property of one geometry, or None if it does not solve.

    ``only`` restricts the evaluation to the named properties; the default is
    the set actually bounded by ``ctx``, which is what keeps this affordable
    inside the search loop.

    Lengths in mm, angles in deg, anti-effects in dimensionless percent.
    """
    want = set(only) if only is not None else ctx.needed()
    if not want:
        return {}
    lo, hi = float(ctx.travel_mm[0]), float(ctx.travel_mm[1])
    n = max(3, int(ctx.n_nodes))
    try:
        kin = SuspensionKinematics(hp)
        states = kin.sweep(travel_min=lo, travel_max=hi, n=n)
    except Exception:
        return None
    if not states or any(not getattr(st, "converged", True) for st in states):
        return None

    tr = np.array([st.travel for st in states], float)
    i0 = int(np.argmin(np.abs(tr)))
    s0 = states[i0]

    out: dict[str, float] = {}
    if "caster_deg" in want:
        out["caster_deg"] = float(s0.caster)
    if "kpi_deg" in want:
        out["kpi_deg"] = float(s0.kpi)
    if "scrub_static_mm" in want:
        out["scrub_static_mm"] = float(s0.scrub_radius)
    if "rc_migration_mm_per_mm" in want:
        rc = np.array([_rc_height_mm(st, track_mm=ctx.track_mm)
                       for st in states], float)
        if np.all(np.isfinite(rc)) and np.ptp(tr) > 1e-9:
            A = np.vstack([tr, np.ones_like(tr)]).T
            out["rc_migration_mm_per_mm"] = float(
                np.linalg.lstsq(A, rc, rcond=None)[0][0])
        else:
            out["rc_migration_mm_per_mm"] = float("nan")
    if want & _PITCH_PROPERTIES:
        try:
            if "anti_dive_pct" in want:
                out["anti_dive_pct"] = float(kin.anti_dive_pct(
                    ctx.cg_height_mm, ctx.wheelbase_mm,
                    ctx.brake_bias_front, state=s0))
            if "anti_squat_pct" in want:
                out["anti_squat_pct"] = float(kin.anti_squat_pct(
                    ctx.cg_height_mm, ctx.wheelbase_mm,
                    ctx.drive_bias_rear, state=s0))
        except Exception:
            return None
    return out


@dataclass
class SolvedPropertyBounds:
    """The bounds, plus the vehicle context the pitch properties need.

    Anti-dive and anti-squat are not properties of the linkage alone: they
    need the wheelbase, the CG height and the brake/drive bias. Those are
    declared here so that a bound cannot be enforced against an undeclared
    vehicle, and so that the same manifest that reproduces a run reproduces
    the numbers the bound was checked against.
    """
    bounds: list[PropertyBound] = _dcfield(default_factory=list)
    cg_height_mm: float = 280.0
    wheelbase_mm: float = 1630.0
    track_mm: float = 1200.0
    brake_bias_front: float = 0.60
    drive_bias_rear: float = 1.0
    travel_mm: tuple[float, float] = (-25.0, 25.0)
    n_nodes: int = 5

    def __post_init__(self):
        self.bounds = list(self.bounds)
        for b in self.bounds:
            if not isinstance(b, PropertyBound):
                raise TypeError("SolvedPropertyBounds.bounds takes "
                                "PropertyBound instances.")
        for name in ("cg_height_mm", "wheelbase_mm", "track_mm"):
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"SolvedPropertyBounds.{name} must be > 0.")
        lo, hi = (float(self.travel_mm[0]), float(self.travel_mm[1]))
        if hi <= lo:
            raise ValueError("SolvedPropertyBounds.travel_mm must be "
                             "(min, max) with max > min.")
        self.travel_mm = (lo, hi)

    def needed(self) -> set[str]:
        """Returns property-name strings; dimensionless keys, not measurements."""
        return {b.prop for b in self.bounds}

    def evaluate(self, hp: Hardpoints) -> dict[str, float] | None:
        """Values in mm, deg or percent depending on the property."""
        return properties_of(hp, self)

    def violations(self, hp: Hardpoints) -> list[tuple[str, str, float]]:
        """(property, bound label, margin) for every violated bound.

        Margin in mm/mm, deg or percent depending on the property.
        """
        if not self.bounds:
            return []
        vals = self.evaluate(hp)
        if vals is None:
            return [("(sweep)", "geometry does not solve over the bound "
                                "evaluation range", float("-inf"))]
        out: list[tuple[str, str, float]] = []
        for b in self.bounds:
            m = b.margin(vals.get(b.prop, float("nan")))
            if m < -1e-12:
                out.append((b.prop, b.label, m))
        return out


@dataclass
class LegalVolume:
    """Where each movable hardpoint is ALLOWED to exist.

    boxes            : point name → (lo, hi) absolute corner-frame bounds, mm.
                       Only listed points are design freedoms; everything
                       else stays welded to its nominal.
    keep_out         : obstacle volumes — PhantomEnvelope instances (carved
                       from NEIGHBOURING assemblies) and/or KeepOutBoxes.
                       Queried through ``clearances(points, probe)``.
    probe_radius_mm  : the sphere tested at each movable point (inflate to
                       cover the physical tab/bracket, not just the pickup).
    min_clearance_mm : required skin gap to every obstacle.
    properties       : optional SolvedPropertyBounds — ranges on anti-dive,
                       anti-squat, roll-centre migration, caster, KPI and
                       scrub that the SEARCH enforces, refusing any step that
                       leaves them exactly as it refuses a keep-out. None
                       (the default) reproduces the pre-bound behaviour
                       exactly: the properties are neither computed nor
                       enforced and the solver pays nothing for them.
    """
    boxes: dict[str, tuple[np.ndarray, np.ndarray]]
    keep_out: list[object] = _dcfield(default_factory=list)
    probe_radius_mm: float = 0.0
    min_clearance_mm: float = 0.0
    #: relations between points (minimum wishbone base, fore/aft ordering)
    spacings: list[PointSpacing] = _dcfield(default_factory=list)
    #: ranges on properties the curve channels do not carry
    properties: SolvedPropertyBounds | None = None

    def __post_init__(self):
        if not self.boxes:
            raise ValueError("LegalVolume: declare at least one movable "
                             "point's box — with zero freedoms there is "
                             "nothing to generate.")
        norm = {}
        for name, (lo, hi) in self.boxes.items():
            if name not in DESIGNABLE_POINTS:
                raise ValueError(
                    f"'{name}' is not a designable hardpoint. Allowed: "
                    f"{', '.join(DESIGNABLE_POINTS)}.")
            lo = np.asarray(lo, float).reshape(3)
            hi = np.asarray(hi, float).reshape(3)
            if np.any(hi < lo):
                raise ValueError(f"LegalVolume box for '{name}': hi < lo.")
            norm[name] = (lo, hi)
        self.boxes = norm

    @staticmethod
    def around(hp: Hardpoints, half_mm: dict[str, float] | float,
               points: Sequence[str] | None = None,
               **kw) -> LegalVolume:
        """Boxes of ± half_mm around the nominal — the common declaration.

        half_mm is the box half-width in mm.
        """
        if points is None:
            points = list(half_mm) if isinstance(half_mm, dict) \
                else list(DESIGNABLE_POINTS)
        boxes = {}
        for p in points:
            h = abs(float(half_mm[p] if isinstance(half_mm, dict)
                          else half_mm))
            c = np.asarray(getattr(hp, p), float)
            boxes[p] = (c - h, c + h)
        return LegalVolume(boxes=boxes, **kw)

    # ---- coordinate bookkeeping ------------------------------------------- #
    def points(self) -> list[str]:
        """Returns hardpoint-name strings; dimensionless keys, not measurements."""
        return sorted(self.boxes)

    def coords(self) -> list[tuple[str, int]]:
        """Returns (point name, axis index) pairs; dimensionless indices, not measurements."""
        return [(p, a) for p in self.points() for a in range(3)]

    def coord_labels(self) -> list[str]:
        """Labels name a coordinate measured in mm."""
        return [f"{p}.{_AXES[a]}" for p, a in self.coords()]

    def bounds_vec(self, hp: Hardpoints) -> tuple[np.ndarray, np.ndarray]:
        """Shift bounds (lo, hi) per flattened coordinate, RELATIVE to hp.

        Shift bounds in mm, relative to hp.
        """
        lo, hi = [], []
        for p in self.points():
            c = np.asarray(getattr(hp, p), float)
            blo, bhi = self.boxes[p]
            lo.append(blo - c)
            hi.append(bhi - c)
        return np.concatenate(lo), np.concatenate(hi)

    def clamp(self, hp: Hardpoints, shift: np.ndarray
              ) -> tuple[np.ndarray, list[str]]:
        """Clamp a flattened shift into the boxes; name clamped coordinates.

        Shift in mm.
        """
        lo, hi = self.bounds_vec(hp)
        clamped = [lab for lab, s, l, h in
                   zip(self.coord_labels(), shift, lo, hi)
                   if s < l - 1e-12 or s > h + 1e-12]
        return np.clip(shift, lo, hi), clamped

    def keepout_violations(self, hp: Hardpoints
                           ) -> list[tuple[str, str, float]]:
        """(point, obstacle label, clearance) for every filtered violation.

        Clearance in mm.
        """
        out: list[tuple[str, str, float]] = []
        pts = np.array([np.asarray(getattr(hp, p), float)
                        for p in self.points()])
        for obs in self.keep_out:
            cl = np.asarray(obs.clearances(pts, self.probe_radius_mm), float)
            lab = getattr(obs, "label", None) or \
                getattr(obs, "kind", None) or obs.__class__.__name__
            for p, c in zip(self.points(), cl):
                if c < self.min_clearance_mm - 1e-12:
                    out.append((p, str(lab), float(c)))
        for sp in self.spacings:
            g = sp.gap(hp)
            if g < sp.min_gap_mm - 1e-12:
                out.append((f"{sp.a}->{sp.b}", sp.label,
                            float(g - sp.min_gap_mm)))
        return out

    # ---- the solved-property wall ----------------------------------------- #
    def has_property_bounds(self) -> bool:
        return bool(self.properties is not None and self.properties.bounds)

    def property_violations(self, hp: Hardpoints
                            ) -> list[tuple[str, str, float]]:
        """(property, bound label, margin) for every violated bound.

        Kept separate from ``keepout_violations`` on purpose: a step refused
        because a pickup sits inside the exhaust and a step refused because
        the corner would deliver -51% anti-squat are different facts about
        the design, and the report has to be able to say which happened.

        Margin in mm/mm, deg or percent depending on the property.
        """
        if not self.has_property_bounds():
            return []
        return self.properties.violations(hp)

    def evaluate_properties(self, hp: Hardpoints) -> dict[str, float] | None:
        """Solved properties of one geometry, or None when none are bounded.

        Values in mm, deg or percent depending on the property.
        """
        if self.properties is None or not self.properties.bounds:
            return None
        return self.properties.evaluate(hp)


# --------------------------------------------------------------------------- #
#  Flatten / unflatten between the solver's vector and named point shifts.
# --------------------------------------------------------------------------- #
def _unflatten(vec: np.ndarray, coords: list[tuple[str, int]]
               ) -> dict[str, np.ndarray]:
    offs: dict[str, np.ndarray] = {}
    for (p, a), v in zip(coords, vec):
        offs.setdefault(p, np.zeros(3))[a] = v
    return {p: v for p, v in offs.items()}


def _shifted(hp: Hardpoints, volume: LegalVolume,
             shift: np.ndarray) -> Hardpoints:
    return _perturbed(hp, _unflatten(shift, volume.coords()))


# --------------------------------------------------------------------------- #
#  The reverse gradients — Jacobian of the weighted residual, by full solves.
# --------------------------------------------------------------------------- #
def _jacobian(hp: Hardpoints, targets: GenesisTargets,
              coords: list[tuple[str, int]],
              step_mm: float = 0.25) -> np.ndarray | None:
    """Central-difference d(weighted residual)/d(coordinate). None when any
    probe fails to solve — the caller treats that as a cliff, not a number."""
    J = np.zeros((len(targets.rows()), len(coords)))
    for j, (p, a) in enumerate(coords):
        off = np.zeros(3)
        off[a] = step_mm
        rp, okp = targets.residual(_perturbed(hp, {p: off}))
        rm, okm = targets.residual(_perturbed(hp, {p: -off}))
        if not (okp and okm):
            return None
        J[:, j] = (rp - rm) / (2.0 * step_mm)
    return J


# --------------------------------------------------------------------------- #
#  One inverse solve — damped Gauss–Newton inside the boundary filter.
# --------------------------------------------------------------------------- #
@dataclass
class Candidate:
    """One geometry the reverse solve produced, before/after yield pricing."""
    ok: bool
    hit: bool                       # every station inside its band
    shifts: dict[str, np.ndarray]   # point → shift from nominal, mm
    shift_vec: np.ndarray
    residual: np.ndarray            # band-weighted; |r| ≤ 1 is inside
    max_band_frac: float            # max |r| — the fit's worst station
    worst_row: str                  # which (channel, station) governs
    iterations: int
    clamped: list[str]              # coordinates pinned to a box face
    keepout_rejections: int         # steps the keep-out filter refused
    # solved-property wall (empty / None when no bounds were declared):
    property_rejections: int = 0    # steps a PropertyBound refused
    properties: dict[str, float] | None = None
    # co-optimizer stage:
    yield_frac: float | None = None
    yield_warnings: list[str] = _dcfield(default_factory=list)
    verdict: str = ""               # RESILIENT | TEMPERED | KNIFE_EDGE | NO_FIT


def genesis_solve(hp: Hardpoints, targets: GenesisTargets,
                  volume: LegalVolume,
                  start_shift: np.ndarray | None = None,
                  max_iter: int = 30, step_mm: float = 0.25,
                  lam0: float = 1e-2) -> Candidate:
    """Pull the movable points until the curves land inside their bands.

    Levenberg-damped Gauss–Newton on the band-weighted residual: solve
    (JᵀJ + λ·diag(JᵀJ))Δx = −Jᵀr, clamp Δx into the legal boxes, reject the
    step outright if any moved point violates a keep-out volume OR leaves a
    declared solved-property bound (raise λ and retry — the filter is a
    constraint, not a penalty), accept on cost decrease. Deterministic: no
    randomness anywhere in this function.

    Coordinates in mm; camber, toe and caster in deg; roll-centre height and scrub in mm.
    """
    coords = volume.coords()
    x, _ = volume.clamp(hp, np.zeros(len(coords))
                        if start_shift is None
                        else np.asarray(start_shift, float))
    hp_x = _shifted(hp, volume, x)
    r, ok = targets.residual(hp_x)
    if not ok:
        # a start the solver can't follow is discarded honestly
        return Candidate(ok=False, hit=False, shifts={}, shift_vec=x,
                         residual=r, max_band_frac=float("inf"),
                         worst_row="(nominal/start does not solve)",
                         iterations=0, clamped=[], keepout_rejections=0)
    if volume.keepout_violations(hp_x):
        return Candidate(ok=False, hit=False, shifts={}, shift_vec=x,
                         residual=r, max_band_frac=float(np.max(np.abs(r))),
                         worst_row="(start violates a keep-out volume)",
                         iterations=0, clamped=[], keepout_rejections=1)
    start_pvio = volume.property_violations(hp_x)
    if start_pvio:
        return Candidate(ok=False, hit=False, shifts={}, shift_vec=x,
                         residual=r, max_band_frac=float(np.max(np.abs(r))),
                         worst_row=("(start violates "
                                    f"{start_pvio[0][1]})"),
                         iterations=0, clamped=[], keepout_rejections=0,
                         property_rejections=1)

    cost = float(r @ r)
    lam = lam0
    clamped_last: list[str] = []
    rejections = 0
    prop_rejections = 0
    it = 0
    for it in range(1, max_iter + 1):
        if np.max(np.abs(r)) <= 1.0:        # every station inside its band
            break
        J = _jacobian(hp_x, targets, coords, step_mm=step_mm)
        if J is None:                        # sitting at a solver cliff
            break
        JtJ = J.T @ J
        diag = np.diag(np.maximum(np.diag(JtJ), 1e-12))
        g = J.T @ r
        stepped = False
        for _ in range(8):                   # λ ladder within one iteration
            try:
                dx = np.linalg.solve(JtJ + lam * diag, -g)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            x_try, clamped = volume.clamp(hp, x + dx)
            hp_try = _shifted(hp, volume, x_try)
            vio = volume.keepout_violations(hp_try)
            if vio:
                rejections += 1
                lam *= 10.0                  # shorter step, away from the wall
                continue
            if volume.property_violations(hp_try):
                prop_rejections += 1
                lam *= 10.0                  # same wall, different surface
                continue
            r_try, ok_try = targets.residual(hp_try)
            if not ok_try:
                lam *= 10.0
                continue
            c_try = float(r_try @ r_try)
            if c_try < cost - 1e-12 or np.max(np.abs(r_try)) <= 1.0:
                x, hp_x, r, cost = x_try, hp_try, r_try, c_try
                clamped_last = clamped
                lam = max(lam / 3.0, 1e-6)
                stepped = True
                break
            lam *= 10.0
        if not stepped:
            break                            # stalled: best legal point stands

    worst = int(np.argmax(np.abs(r)))
    return Candidate(
        ok=True,
        hit=bool(np.max(np.abs(r)) <= 1.0),
        shifts={p: v for p, v in _unflatten(x, coords).items()
                if np.any(np.abs(v) > 1e-9)},
        shift_vec=x,
        residual=r,
        max_band_frac=float(np.max(np.abs(r))),
        worst_row=targets.row_labels()[worst],
        iterations=it,
        clamped=clamped_last,
        keepout_rejections=rejections,
        property_rejections=prop_rejections,
        properties=volume.evaluate_properties(hp_x),
    )


# --------------------------------------------------------------------------- #
#  The build-yield co-optimizer.
# --------------------------------------------------------------------------- #
@dataclass
class GenesisThresholds:
    resilient_yield: float = 0.95    # ≥ this → RESILIENT
    tempered_yield: float = 0.80     # ≥ this → TEMPERED, below → KNIFE_EDGE
    verify_agreement: float = 0.98   # linear-vs-full pass/fail honesty floor


def build_yield(hp_candidate: Hardpoints, targets: GenesisTargets,
                fld: ToleranceField, r_fit: np.ndarray,
                n: int = 4000, seed: int = 0, step_mm: float = 0.25
                ) -> tuple[float | None, list[str]]:
    """P(as-built curves stay inside the bands), first order.

    The coupling that makes the co-optimizer honest: the candidate's own fit
    residual ``r_fit`` (band units) is added to the propagated weld scatter
    before judging — the fit has already spent part of the band, and only
    the headroom left absorbs the shop's error field.
    """
    warns: list[str] = []
    J = _jacobian(hp_candidate, targets, fld.coords(), step_mm=step_mm)
    if J is None:
        return None, ["Sensitivity probes at the candidate fail to solve — "
                      "the geometry sits near a kinematic singularity; its "
                      "yield is not a number this model owns (treated as "
                      "knife-edge)."]
    samples = fld.sample(n, seed=seed)          # (n, coords) mm
    dr = samples @ J.T                          # (n, rows), band units
    passed = np.all(np.abs(r_fit[None, :] + dr) <= 1.0, axis=1)
    return float(np.mean(passed)), warns


def _verify_yield_full(hp_candidate: Hardpoints, targets: GenesisTargets,
                       fld: ToleranceField, r_fit: np.ndarray,
                       J: np.ndarray | None,
                       n_verify: int, seed: int) -> tuple[float, float]:
    """(full-solve yield, linear-vs-full pass/fail agreement) on a subsample.

    Perturbed geometries that fail to solve are charged as fails — the same
    accounting Stochastic Inversion uses."""
    samples = fld.sample(n_verify, seed=seed + 1)
    coords = fld.coords()
    full_pass = np.zeros(n_verify, bool)
    for i in range(n_verify):
        hp_i = _perturbed(hp_candidate, _unflatten(samples[i], coords))
        r_i, ok = targets.residual(hp_i)
        full_pass[i] = bool(ok and np.max(np.abs(r_i)) <= 1.0)
    if J is not None:
        lin_pass = np.all(np.abs(r_fit[None, :] + samples @ J.T) <= 1.0,
                          axis=1)
        agree = float(np.mean(lin_pass == full_pass))
    else:
        agree = 0.0
    return float(np.mean(full_pass)), agree


@dataclass
class GenesisResult:
    ok: bool
    reason: str
    candidates: list[Candidate]      # every distinct solve, best first
    winner: Candidate | None
    winner_hp: Hardpoints | None
    best_fit: Candidate | None    # the pure curve-fit optimum (may lose!)
    resilience_premium: float | None   # winner yield − best-fit yield
    n_starts: int
    seed: int
    verify_yield: float | None    # full-solve check on the winner
    verify_agreement: float | None
    thresholds: GenesisThresholds
    warnings: list[str]
    #: the bounds the search enforced, echoed for the report and the manifest
    property_bounds: SolvedPropertyBounds | None = None


def inverse_genesis(hp: Hardpoints, targets: GenesisTargets,
                    volume: LegalVolume,
                    fld: ToleranceField | None = None,
                    n_starts: int = 6, n_yield: int = 4000,
                    n_verify_full: int = 0, seed: int = 0,
                    thresholds: GenesisThresholds | None = None,
                    max_iter: int = 30, step_mm: float = 0.25
                    ) -> GenesisResult:
    """The full engine: multi-start reverse solve + build-yield co-optimizer.

    Starts: the nominal itself plus (n_starts − 1) deterministic samples
    inside the legal boxes. Each converged candidate that HITS the curves is
    priced for build yield against ``fld``; the winner is the highest-yield
    hit, NOT the best fit — the knife-edge optimum loses on purpose, and the
    yield it forfeited is printed as the resilience premium. With no field
    declared, the engine degrades honestly to pure inverse kinematics and
    says the buildability question went unasked.

    Hardpoint coordinates in mm; channels in deg or mm; build yield dimensionless.
    """
    th = thresholds or GenesisThresholds()
    warnings: list[str] = []

    r0, ok0 = targets.residual(hp)
    if not ok0:
        return GenesisResult(
            ok=False,
            reason="The NOMINAL geometry does not solve over the requested "
                   "travel stations — fix the hardpoints (or the stations) "
                   "before asking for their inverse.",
            candidates=[], winner=None, winner_hp=None, best_fit=None,
            resilience_premium=None, n_starts=0, seed=seed,
            verify_yield=None, verify_agreement=None, thresholds=th,
            warnings=warnings, property_bounds=volume.properties)

    # ---- deterministic multi-start ---------------------------------------- #
    #  The property wall is absolute, exactly like the keep-out wall: the
    #  search refuses a step that leaves a bound, so it cannot walk INTO the
    #  feasible set from outside it. With bounds declared, the random starts
    #  are therefore drawn until they are property-feasible (a bounded number
    #  of draws, still deterministic from the seed) instead of being thrown
    #  away by the first check. The nominal is always offered as a start; if
    #  it violates a bound, that is reported rather than hidden, because "your
    #  current geometry is already outside the band you just declared" is a
    #  design answer.
    rng = np.random.default_rng(int(seed))
    lo, hi = volume.bounds_vec(hp)
    starts: list[np.ndarray] = [np.zeros(len(lo))]
    want = max(0, int(n_starts) - 1)
    if volume.has_property_bounds():
        draws = 0
        max_draws = max(1, want) * _MAX_START_DRAWS_PER_START
        while len(starts) - 1 < want and draws < max_draws:
            draws += 1
            cand_shift = rng.uniform(lo, hi)
            if not volume.property_violations(
                    _shifted(hp, volume, cand_shift)):
                starts.append(cand_shift)
        if len(starts) - 1 < want:
            warnings.append(
                f"Only {len(starts) - 1} of {want} random starts landed "
                f"inside the declared solved-property bounds after "
                f"{draws} draws. The bounds are tight relative to the legal "
                "volume, so the candidate field is thinner than requested "
                "and the resilience premium is measured over fewer basins.")
    else:
        for _ in range(want):
            starts.append(rng.uniform(lo, hi))

    cands: list[Candidate] = []
    n_start_prop_refused = 0
    n_start_keepout_refused = 0
    for s in starts:
        c = genesis_solve(hp, targets, volume, start_shift=s,
                          max_iter=max_iter, step_mm=step_mm)
        if not c.ok:
            if c.property_rejections:
                n_start_prop_refused += 1
            elif c.keepout_rejections:
                n_start_keepout_refused += 1
            continue
        if any(np.linalg.norm(c.shift_vec - c2.shift_vec) < 0.05
               for c2 in cands):
            continue                          # same basin, keep one
        cands.append(c)

    if not cands:
        if n_start_prop_refused and volume.has_property_bounds():
            names = "; ".join(b.label for b in volume.properties.bounds)
            reason = (
                f"No start survived the declared solved-property bounds "
                f"({names}). {n_start_prop_refused} of {len(starts)} starts "
                "were refused on a property, not on the curves, so the "
                "search never moved. The wall is absolute by design — it "
                "refuses a step out of the band, it does not walk in from "
                "outside — so this means the bound is unreachable FROM THIS "
                "LEGAL VOLUME with these starts, not that no geometry "
                "anywhere satisfies it. Widen the band, move or enlarge the "
                "boxes, or seed from a geometry that already meets the "
                "bound. No optimum was fabricated.")
        else:
            reason = ("No start inside the legal volume produced a solvable "
                      "geometry — the declared boxes reach past the solver's "
                      "kinematic range. Shrink or move the boxes.")
        return GenesisResult(
            ok=False, reason=reason,
            candidates=[], winner=None, winner_hp=None, best_fit=None,
            resilience_premium=None, n_starts=len(starts), seed=seed,
            verify_yield=None, verify_agreement=None, thresholds=th,
            warnings=warnings, property_bounds=volume.properties)

    hits = [c for c in cands if c.hit]

    # ---- nobody reached the curves: name the binding constraint ----------- #
    if not hits:
        best = min(cands, key=lambda c: c.max_band_frac)
        for c in cands:
            c.verdict = "NO_FIT"
        limit = []
        if best.clamped:
            limit.append("the legal box is binding on "
                         + ", ".join(best.clamped))
        if best.keepout_rejections:
            limit.append(f"the keep-out filter refused {best.keepout_rejections} "
                         "step(s) toward the curves")
        if best.property_rejections:
            names = ", ".join(b.label for b in volume.properties.bounds) \
                if volume.has_property_bounds() else "a solved-property bound"
            limit.append(f"a solved-property bound refused "
                         f"{best.property_rejections} step(s) toward the "
                         f"curves ({names})")
        if not limit:
            limit.append("the linkage itself cannot produce these curves in "
                         "this volume")
        reason = (f"No legal geometry reaches the drawn curves. Closest "
                  f"approach: {best.max_band_frac:.2f}× the band, governed by "
                  f"{best.worst_row}; {'; '.join(limit)}. The bands and the "
                  "legal volume are mutually unsatisfiable AS DECLARED — "
                  "widen that band, free that coordinate, or accept the "
                  "closest legal curve below. No optimum was fabricated.")
        cands.sort(key=lambda c: c.max_band_frac)
        return GenesisResult(ok=False, reason=reason, candidates=cands,
                             winner=None,
                             winner_hp=_shifted(hp, volume, best.shift_vec),
                             best_fit=best, resilience_premium=None,
                             n_starts=len(starts), seed=seed,
                             verify_yield=None, verify_agreement=None,
                             thresholds=th, warnings=warnings,
            property_bounds=volume.properties)

    best_fit = min(hits, key=lambda c: c.max_band_frac)

    # ---- no error field: pure inverse kinematics, honestly labelled ------- #
    if fld is None or not fld.specs:
        for c in cands:
            c.verdict = "NO_FIT" if not c.hit else "TEMPERED"
        warnings.append("No tolerance field declared — the buildability "
                        "question was not asked. This is textbook inverse "
                        "kinematics: the winner is the best FIT, which may "
                        "be a knife-edge your shop cannot hold. Declare a "
                        "field (Stochastic Inversion presets work) to "
                        "co-optimize for yield.")
        hits.sort(key=lambda c: c.max_band_frac)
        others = sorted((c for c in cands if not c.hit),
                        key=lambda c: c.max_band_frac)
        return GenesisResult(
            ok=True,
            reason="Curves reached (fit-only — no yield pricing).",
            candidates=hits + others, winner=best_fit,
            winner_hp=_shifted(hp, volume, best_fit.shift_vec),
            best_fit=best_fit, resilience_premium=None,
            n_starts=len(starts), seed=seed,
            verify_yield=None, verify_agreement=None,
            thresholds=th, warnings=warnings,
            property_bounds=volume.properties)

    # ---- the co-optimizer: price every hit for build yield ---------------- #
    for c in cands:
        if not c.hit:
            c.verdict = "NO_FIT"
            continue
        hp_c = _shifted(hp, volume, c.shift_vec)
        y, w = build_yield(hp_c, targets, fld, c.residual,
                           n=n_yield, seed=seed, step_mm=step_mm)
        c.yield_warnings = w
        if y is None:
            c.yield_frac, c.verdict = 0.0, "KNIFE_EDGE"
            continue
        c.yield_frac = y
        c.verdict = ("RESILIENT" if y >= th.resilient_yield else
                     "TEMPERED" if y >= th.tempered_yield else
                     "KNIFE_EDGE")

    hits.sort(key=lambda c: (-(c.yield_frac or 0.0), c.max_band_frac))
    winner = hits[0]
    premium = float((winner.yield_frac or 0.0) - (best_fit.yield_frac or 0.0))

    if winner.verdict == "KNIFE_EDGE":
        reason = (f"Every geometry that hits the drawn curves is KNIFE_EDGE "
                  f"under the declared field (best yield "
                  f"{(winner.yield_frac or 0):.1%}) — the bands, the legal "
                  "volume and the shop's error field are JOINTLY "
                  "unsatisfiable. The levers, in order of cheapness: jig the "
                  "dominant tab (shrink the field), widen the governing "
                  "band, or free another coordinate. The best knife-edge is "
                  "reported below, clearly labelled — building it is a "
                  "gamble this engine prices, not one it recommends.")
        ok = False
    else:
        if winner is not best_fit and premium > 1e-9:
            reason = (f"Manufacturing-resilient geometry found. The pure "
                      f"curve-fit optimum was REJECTED: it fits "
                      f"{best_fit.max_band_frac:.2f}× band vs the winner's "
                      f"{winner.max_band_frac:.2f}×, but its build yield is "
                      f"{(best_fit.yield_frac or 0):.1%} against the "
                      f"winner's {(winner.yield_frac or 0):.1%} — a "
                      f"{premium:+.1%} yield premium bought by moving off "
                      "the knife edge. The engine designs the car that gets "
                      "built.")
        else:
            reason = (f"Manufacturing-resilient geometry found: the best "
                      f"fit is also the most buildable "
                      f"({(winner.yield_frac or 0):.1%} yield).")
        ok = True

    # ---- price the linearisation on the winner ---------------------------- #
    verify_y = verify_a = None
    if n_verify_full > 0:
        hp_w = _shifted(hp, volume, winner.shift_vec)
        Jw = _jacobian(hp_w, targets, fld.coords(), step_mm=step_mm)
        verify_y, verify_a = _verify_yield_full(
            hp_w, targets, fld, winner.residual, Jw, n_verify_full, seed)
        if verify_a < th.verify_agreement:
            warnings.append(
                f"Linear/full pass-fail agreement {verify_a:.1%} is below "
                f"the {th.verify_agreement:.0%} honesty floor — the winning "
                "yield is DEMOTED to the full-solve figure "
                f"({verify_y:.1%}); trust that number, and rerun the "
                "co-optimizer with more full verification samples.")

    others = sorted((c for c in cands if not c.hit),
                    key=lambda c: c.max_band_frac)
    return GenesisResult(
        ok=ok, reason=reason, candidates=hits + others, winner=winner,
        winner_hp=_shifted(hp, volume, winner.shift_vec),
        best_fit=best_fit, resilience_premium=premium,
        n_starts=len(starts), seed=seed,
        verify_yield=verify_y, verify_agreement=verify_a,
        thresholds=th, warnings=warnings,
            property_bounds=volume.properties)


# --------------------------------------------------------------------------- #
#  Report
# --------------------------------------------------------------------------- #
_VERDICT_ICON = {"RESILIENT": "🟢", "TEMPERED": "🟡",
                 "KNIFE_EDGE": "🔴", "NO_FIT": "⚪"}


def render_genesis_md(res: GenesisResult,
                      targets: GenesisTargets | None = None) -> str:
    """The one-page markdown the design review reads."""
    L: list[str] = ["# 🧬 InverseGenesis — stochastic inverse report", ""]
    L.append(("✅ " if res.ok else "❌ ") + res.reason)
    L.append("")
    if res.winner is not None:
        w = res.winner
        L.append("## The generated geometry")
        L.append(f"- verdict: {_VERDICT_ICON.get(w.verdict, '')} "
                 f"**{w.verdict}**"
                 + (f" — build yield **{w.yield_frac:.1%}**"
                    if w.yield_frac is not None else ""))
        L.append(f"- worst station: {w.max_band_frac:.2f}× band "
                 f"({w.worst_row})")
        L.append(f"- converged in {w.iterations} Gauss–Newton iterations"
                 + (f"; keep-out filter refused {w.keepout_rejections} "
                    "step(s)" if w.keepout_rejections else "")
                 + (f"; solved-property bounds refused "
                    f"{w.property_rejections} step(s)"
                    if w.property_rejections else ""))
        if w.clamped:
            L.append(f"- pinned to the legal box: {', '.join(w.clamped)}")
        L.append("")
        if w.properties:
            L.append("## Solved properties, bounded inside the search")
            L.append("| property | bound | delivered | margin |")
            L.append("|---|---|---|---|")
            bounds = (res.property_bounds.bounds
                      if res.property_bounds is not None else [])
            for b in bounds:
                v = w.properties.get(b.prop, float("nan"))
                m = b.margin(v)
                L.append(f"| {_PROPERTY_LABELS[b.prop]} | {b.label} | "
                         f"{v:.3f} | {m:+.3f} |")
            extra = sorted(set(w.properties) - {b.prop for b in bounds})
            for k in extra:
                L.append(f"| {_PROPERTY_LABELS[k]} | (reported only) | "
                         f"{w.properties[k]:.3f} | — |")
            L.append("")
        L.append("| hardpoint | Δx (mm) | Δy (mm) | Δz (mm) |")
        L.append("|---|---|---|---|")
        for p in sorted(res.winner.shifts):
            v = res.winner.shifts[p]
            L.append(f"| {p} | {v[0]:+.2f} | {v[1]:+.2f} | {v[2]:+.2f} |")
        if not res.winner.shifts:
            L.append("| (nominal already satisfies the curves) | — | — | — |")
        L.append("")
    if res.resilience_premium is not None and res.best_fit is not None \
            and res.winner is not res.best_fit:
        L.append(f"**Resilience premium:** the rejected best-fit candidate "
                 f"yields {(res.best_fit.yield_frac or 0):.1%}; the winner "
                 f"pays {res.winner.max_band_frac - res.best_fit.max_band_frac:+.2f}× "
                 f"band of fit for **{res.resilience_premium:+.1%}** yield.")
        L.append("")
    if len(res.candidates) > 1:
        L.append("## Candidate family")
        L.append("| # | verdict | fit (×band) | build yield | governed by |")
        L.append("|---|---|---|---|---|")
        for i, c in enumerate(res.candidates, 1):
            y = f"{c.yield_frac:.1%}" if c.yield_frac is not None else "—"
            L.append(f"| {i} | {_VERDICT_ICON.get(c.verdict, '')} "
                     f"{c.verdict} | {c.max_band_frac:.2f} | {y} | "
                     f"{c.worst_row} |")
        L.append("")
    if res.verify_yield is not None:
        L.append(f"**Linearisation, priced:** full-solve verification yield "
                 f"{res.verify_yield:.1%}; linear/full pass-fail agreement "
                 f"{res.verify_agreement:.1%} "
                 f"(floor {res.thresholds.verify_agreement:.0%}).")
        L.append("")
    for wmsg in res.warnings:
        L.append(f"⚠️ {wmsg}")
    if res.warnings:
        L.append("")
    L.append("---")
    L.append(f"*{res.n_starts} deterministic starts, seed {res.seed}. "
             "Scope: rigid corner solver; independent per-point errors; "
             "keep-out screening tests each movable pickup as a probe "
             "sphere, not the bracket around it — and an obstacle envelope "
             "must be carved from NEIGHBOURING assemblies, never from the "
             "corner being designed. "
             + ("Solved-property bounds are enforced on every trial geometry "
                "at the static state (migration as the least-squares slope "
                "over the declared travel range), so they bound the nominal "
                "corner, not every as-built one. "
                if res.property_bounds is not None else "")
             + "Validate the generated geometry in "
             "Ghost Topology and full simulation before manufacturing.*")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  Self-test — python3 -m suspension.inverse_genesis
# --------------------------------------------------------------------------- #
if __name__ == "__main__":   # pragma: no cover
    hp = Hardpoints.default()

    # The ground truth: a KNOWN geometry, shifted off the default, whose own
    # curves become the drawn target. The engine must find its way back to
    # curves it has provably never seen the coordinates of.
    truth_shift = {"upper_front_inner": np.array([0.0, -4.0, 5.0]),
                   "upper_rear_inner":  np.array([0.0, -4.0, 5.0])}
    hp_truth = _perturbed(hp, truth_shift)
    stations = np.array([-25.0, -12.5, 0.0, 12.5, 25.0])
    truth, ok = curves_of(hp_truth, stations)
    assert ok, "self-test ground truth must solve"

    targets = GenesisTargets(curves=[
        TargetCurve("camber_deg", stations, truth["camber_deg"],
                    np.full(5, 0.15)),
        TargetCurve("toe_deg", stations, truth["toe_deg"],
                    np.full(5, 0.08)),
        TargetCurve("rc_height_mm", stations, truth["rc_height_mm"],
                    np.full(5, 6.0)),
    ])
    volume = LegalVolume.around(
        hp, 8.0, points=["upper_front_inner", "upper_rear_inner"])

    print("=== 1 · reverse gradients recover a hidden geometry ===")
    r_nom, _ = targets.residual(hp)
    print(f"  nominal misses the drawn curves by "
          f"{np.max(np.abs(r_nom)):.2f}× band")
    c = genesis_solve(hp, targets, volume)
    assert c.ok and c.hit, f"inverse solve must hit (got {c.max_band_frac:.2f}×)"
    print(f"  solved in {c.iterations} iterations → "
          f"{c.max_band_frac:.2f}× band; shifts:")
    for p, v in sorted(c.shifts.items()):
        print(f"    {p}: [{v[0]:+.2f}, {v[1]:+.2f}, {v[2]:+.2f}] mm")

    print()
    print("=== 2 · the boundary filter is a wall, not a penalty ===")
    # a keep-out box sitting exactly on the truth's upper-front tab position
    tgt = np.asarray(hp_truth.upper_front_inner, float)
    ko = KeepOutBox(tgt - 3.0, tgt + 3.0, label="exhaust primary (test)")
    volume_ko = LegalVolume.around(
        hp, 8.0, points=["upper_front_inner", "upper_rear_inner"],
        keep_out=[ko], min_clearance_mm=1.0)
    c_ko = genesis_solve(hp, targets, volume_ko)
    assert c_ko.ok
    hp_ko = _shifted(hp, volume_ko, c_ko.shift_vec)
    assert not volume_ko.keepout_violations(hp_ko), \
        "no generated point may sit inside a keep-out volume"
    print(f"  with the truth position walled off: "
          f"{'still hit' if c_ko.hit else 'closest legal'} at "
          f"{c_ko.max_band_frac:.2f}× band, "
          f"{c_ko.keepout_rejections} step(s) refused — zero violations")

    print()
    print("=== 3 · the co-optimizer prices the knife edge ===")
    fld = ToleranceField.preset("hand_weld", weld_pull_mm=1.0, pull_axis="z")
    res = inverse_genesis(hp, targets, volume, fld=fld,
                          n_starts=5, n_yield=3000, n_verify_full=60, seed=0)
    print(render_genesis_md(res, targets))
    assert res.winner is not None
    assert res.winner.yield_frac is not None
    assert 0.0 <= res.winner.yield_frac <= 1.0
    assert res.winner.hit or not res.ok
    # the winner is never out-yielded by another hit
    for cc in res.candidates:
        if cc.hit and cc.yield_frac is not None:
            assert res.winner.yield_frac >= cc.yield_frac - 1e-12

    print()
    print("=== 4 · determinism: same inputs, byte-identical report ===")
    res2 = inverse_genesis(hp, targets, volume, fld=fld,
                           n_starts=5, n_yield=3000, n_verify_full=60, seed=0)
    assert render_genesis_md(res) == render_genesis_md(res2)
    print("  identical ✓")

    print()
    print("=== 5 · unsatisfiable intent is named, not papered over ===")
    impossible = GenesisTargets(curves=[
        TargetCurve("camber_deg", stations,
                    truth["camber_deg"] + 25.0,       # 25° away: not happening
                    np.full(5, 0.1))])
    res_no = inverse_genesis(hp, impossible, volume, fld=fld,
                             n_starts=3, n_yield=500, seed=0)
    assert not res_no.ok and res_no.winner is None
    print("  " + res_no.reason.split(".")[0] + ".")

    print()
    print("=== 6 · the channels do not carry anti-squat; a bound does ===")
    wide = LegalVolume.around(
        hp, 20.0, points=["upper_front_inner", "upper_rear_inner",
                          "lower_front_inner", "lower_rear_inner"])
    ctx_kw = dict(cg_height_mm=280.0, wheelbase_mm=1630.0,
                  track_mm=1200.0, drive_bias_rear=1.0)
    free = inverse_genesis(hp, targets, wide, fld=None, n_starts=6, seed=0)
    probe = SolvedPropertyBounds(bounds=[], **ctx_kw)
    spread = [properties_of(_shifted(hp, wide, c.shift_vec), probe,
                            only=["anti_squat_pct"])["anti_squat_pct"]
              for c in free.candidates if c.hit]
    print(f"  {len(spread)} candidates all hit every channel at every "
          f"station, with anti-squat from {min(spread):+.1f}% to "
          f"{max(spread):+.1f}% — the channels never saw it")

    bounded_vol = LegalVolume.around(
        hp, 20.0, points=["upper_front_inner", "upper_rear_inner",
                          "lower_front_inner", "lower_rear_inner"],
        properties=SolvedPropertyBounds(
            bounds=[PropertyBound("anti_squat_pct", lo=23.0, hi=60.0)],
            **ctx_kw))
    bounded = inverse_genesis(hp, targets, bounded_vol, fld=fld,
                              n_starts=6, n_yield=1500, seed=0)
    assert bounded.ok and bounded.winner is not None
    for c in bounded.candidates:
        if c.hit:
            assert 23.0 - 1e-9 <= c.properties["anti_squat_pct"] \
                <= 60.0 + 1e-9, "a bounded run returned an out-of-band corner"
    print(f"  with the bound inside the search every surviving candidate "
          f"lands in [23, 60]%; the winner delivers "
          f"{bounded.winner.properties['anti_squat_pct']:+.1f}% at "
          f"{bounded.winner.max_band_frac:.2f}x band and "
          f"{bounded.winner.yield_frac:.1%} yield")

    unreachable = LegalVolume.around(
        hp, 20.0, points=["upper_front_inner", "upper_rear_inner",
                          "lower_front_inner", "lower_rear_inner"],
        properties=SolvedPropertyBounds(
            bounds=[PropertyBound("rc_migration_mm_per_mm",
                                  lo=-0.15, hi=0.15)], **ctx_kw))
    res_un = inverse_genesis(hp, targets, unreachable, fld=fld,
                             n_starts=6, n_yield=500, seed=0)
    assert not res_un.ok and res_un.winner is None
    assert "solved-property" in res_un.reason
    print("  an unreachable bound is named as unreachable FROM THIS VOLUME, "
          "not silently dropped and not fabricated")

    print()
    print("=== 7 · declaring no bounds changes nothing ===")
    plain = LegalVolume.around(
        hp, 8.0, points=["upper_front_inner", "upper_rear_inner"],
        properties=SolvedPropertyBounds(bounds=[], **ctx_kw))
    a = inverse_genesis(hp, targets, volume, fld=fld, n_starts=5,
                        n_yield=3000, n_verify_full=60, seed=0)
    b = inverse_genesis(hp, targets, plain, fld=fld, n_starts=5,
                        n_yield=3000, n_verify_full=60, seed=0)
    assert np.allclose(a.winner.shift_vec, b.winner.shift_vec)
    print("  empty bounds reproduce the unbounded winner exactly ✓")

    print()
    print("self-test passed ✓")

