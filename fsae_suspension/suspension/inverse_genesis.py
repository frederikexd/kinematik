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
     legal boxes, and every accepted geometry is screened against keep-out
     volumes queried through the exact Phantom Envelope capsule arithmetic
     (any object exposing ``clearances(points, probe_radius_mm)`` works: a
     carved PhantomEnvelope of a neighbouring assembly, or the KeepOutBox
     declared here for "the header lives in this box"). A coordinate that
     hits the curves from inside an exhaust primary is not a solution; the
     filter makes it unrepresentable rather than merely penalised.

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
* Deterministic end to end: fixed seeds drive the multi-start sampler and
  the yield sampler, so the same inputs give byte-identical geometry,
  yields and markdown.

Self-test: ``python3 -m suspension.inverse_genesis``
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field as _dcfield
from typing import Dict, List, Optional, Sequence, Tuple

from .kinematics import Hardpoints, SuspensionKinematics
from .ghost_topology import _rc_height_mm
from .kinematik_stochastic import ToleranceField, _perturbed

_AXES = ("x", "y", "z")

# --------------------------------------------------------------------------- #
#  The curve channels — the language the intent is drawn in.
# --------------------------------------------------------------------------- #
CHANNELS: Tuple[str, ...] = (
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
DESIGNABLE_POINTS: Tuple[str, ...] = (
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
              n_sweep: Optional[int] = None
              ) -> Tuple[Dict[str, np.ndarray], bool]:
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
    curves: List[TargetCurve]
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
    def rows(self) -> List[Tuple[str, float]]:
        return [(c.channel, float(t)) for c in self.curves
                for t in c.travel_mm]

    def stations(self) -> np.ndarray:
        return np.unique(np.concatenate([c.travel_mm for c in self.curves]))

    def target_vec(self) -> np.ndarray:
        return np.concatenate([c.target for c in self.curves])

    def band_vec(self) -> np.ndarray:
        return np.concatenate([c.band for c in self.curves])

    def residual(self, hp: Hardpoints) -> Tuple[np.ndarray, bool]:
        """Band-weighted residual r: |r_i| ≤ 1 means station i is inside its
        band. NaNs (with ok=False) when the geometry doesn't solve."""
        vals, ok = curves_of(hp, self.stations(), track_mm=self.track_mm)
        if not ok:
            return np.full(len(self.rows()), np.nan), False
        parts = []
        for c in self.curves:
            v = np.interp(c.travel_mm, self.stations(), vals[c.channel])
            parts.append((v - c.target) / c.band)
        return np.concatenate(parts), True

    def row_labels(self) -> List[str]:
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
    """
    boxes: Dict[str, Tuple[np.ndarray, np.ndarray]]
    keep_out: List[object] = _dcfield(default_factory=list)
    probe_radius_mm: float = 0.0
    min_clearance_mm: float = 0.0

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
    def around(hp: Hardpoints, half_mm: Dict[str, float] | float,
               points: Optional[Sequence[str]] = None,
               **kw) -> "LegalVolume":
        """Boxes of ± half_mm around the nominal — the common declaration."""
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
    def points(self) -> List[str]:
        return sorted(self.boxes)

    def coords(self) -> List[Tuple[str, int]]:
        return [(p, a) for p in self.points() for a in range(3)]

    def coord_labels(self) -> List[str]:
        return [f"{p}.{_AXES[a]}" for p, a in self.coords()]

    def bounds_vec(self, hp: Hardpoints) -> Tuple[np.ndarray, np.ndarray]:
        """Shift bounds (lo, hi) per flattened coordinate, RELATIVE to hp."""
        lo, hi = [], []
        for p in self.points():
            c = np.asarray(getattr(hp, p), float)
            blo, bhi = self.boxes[p]
            lo.append(blo - c)
            hi.append(bhi - c)
        return np.concatenate(lo), np.concatenate(hi)

    def clamp(self, hp: Hardpoints, shift: np.ndarray
              ) -> Tuple[np.ndarray, List[str]]:
        """Clamp a flattened shift into the boxes; name clamped coordinates."""
        lo, hi = self.bounds_vec(hp)
        clamped = [lab for lab, s, l, h in
                   zip(self.coord_labels(), shift, lo, hi)
                   if s < l - 1e-12 or s > h + 1e-12]
        return np.clip(shift, lo, hi), clamped

    def keepout_violations(self, hp: Hardpoints
                           ) -> List[Tuple[str, str, float]]:
        """(point, obstacle label, clearance) for every filtered violation."""
        out: List[Tuple[str, str, float]] = []
        pts = np.array([np.asarray(getattr(hp, p), float)
                        for p in self.points()])
        for obs in self.keep_out:
            cl = np.asarray(obs.clearances(pts, self.probe_radius_mm), float)
            lab = getattr(obs, "label", None) or \
                getattr(obs, "kind", None) or obs.__class__.__name__
            for p, c in zip(self.points(), cl):
                if c < self.min_clearance_mm - 1e-12:
                    out.append((p, str(lab), float(c)))
        return out


# --------------------------------------------------------------------------- #
#  Flatten / unflatten between the solver's vector and named point shifts.
# --------------------------------------------------------------------------- #
def _unflatten(vec: np.ndarray, coords: List[Tuple[str, int]]
               ) -> Dict[str, np.ndarray]:
    offs: Dict[str, np.ndarray] = {}
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
              coords: List[Tuple[str, int]],
              step_mm: float = 0.25) -> Optional[np.ndarray]:
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
    shifts: Dict[str, np.ndarray]   # point → shift from nominal, mm
    shift_vec: np.ndarray
    residual: np.ndarray            # band-weighted; |r| ≤ 1 is inside
    max_band_frac: float            # max |r| — the fit's worst station
    worst_row: str                  # which (channel, station) governs
    iterations: int
    clamped: List[str]              # coordinates pinned to a box face
    keepout_rejections: int         # steps the boundary filter refused
    # co-optimizer stage:
    yield_frac: Optional[float] = None
    yield_warnings: List[str] = _dcfield(default_factory=list)
    verdict: str = ""               # RESILIENT | TEMPERED | KNIFE_EDGE | NO_FIT


def genesis_solve(hp: Hardpoints, targets: GenesisTargets,
                  volume: LegalVolume,
                  start_shift: Optional[np.ndarray] = None,
                  max_iter: int = 30, step_mm: float = 0.25,
                  lam0: float = 1e-2) -> Candidate:
    """Pull the movable points until the curves land inside their bands.

    Levenberg-damped Gauss–Newton on the band-weighted residual: solve
    (JᵀJ + λ·diag(JᵀJ))Δx = −Jᵀr, clamp Δx into the legal boxes, reject the
    step outright if any moved point violates a keep-out volume (raise λ and
    retry — the filter is a constraint, not a penalty), accept on cost
    decrease. Deterministic: no randomness anywhere in this function.
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

    cost = float(r @ r)
    lam = lam0
    clamped_last: List[str] = []
    rejections = 0
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
                ) -> Tuple[Optional[float], List[str]]:
    """P(as-built curves stay inside the bands), first order.

    The coupling that makes the co-optimizer honest: the candidate's own fit
    residual ``r_fit`` (band units) is added to the propagated weld scatter
    before judging — the fit has already spent part of the band, and only
    the headroom left absorbs the shop's error field.
    """
    warns: List[str] = []
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
                       J: Optional[np.ndarray],
                       n_verify: int, seed: int) -> Tuple[float, float]:
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
    candidates: List[Candidate]      # every distinct solve, best first
    winner: Optional[Candidate]
    winner_hp: Optional[Hardpoints]
    best_fit: Optional[Candidate]    # the pure curve-fit optimum (may lose!)
    resilience_premium: Optional[float]   # winner yield − best-fit yield
    n_starts: int
    seed: int
    verify_yield: Optional[float]    # full-solve check on the winner
    verify_agreement: Optional[float]
    thresholds: GenesisThresholds
    warnings: List[str]


def inverse_genesis(hp: Hardpoints, targets: GenesisTargets,
                    volume: LegalVolume,
                    fld: Optional[ToleranceField] = None,
                    n_starts: int = 6, n_yield: int = 4000,
                    n_verify_full: int = 0, seed: int = 0,
                    thresholds: Optional[GenesisThresholds] = None,
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
    """
    th = thresholds or GenesisThresholds()
    warnings: List[str] = []

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
            warnings=warnings)

    # ---- deterministic multi-start ---------------------------------------- #
    rng = np.random.default_rng(int(seed))
    lo, hi = volume.bounds_vec(hp)
    starts: List[np.ndarray] = [np.zeros(len(lo))]
    for _ in range(max(0, int(n_starts) - 1)):
        starts.append(rng.uniform(lo, hi))

    cands: List[Candidate] = []
    for s in starts:
        c = genesis_solve(hp, targets, volume, start_shift=s,
                          max_iter=max_iter, step_mm=step_mm)
        if not c.ok:
            continue
        if any(np.linalg.norm(c.shift_vec - c2.shift_vec) < 0.05
               for c2 in cands):
            continue                          # same basin, keep one
        cands.append(c)

    if not cands:
        return GenesisResult(
            ok=False,
            reason="No start inside the legal volume produced a solvable "
                   "geometry — the declared boxes reach past the solver's "
                   "kinematic range. Shrink or move the boxes.",
            candidates=[], winner=None, winner_hp=None, best_fit=None,
            resilience_premium=None, n_starts=len(starts), seed=seed,
            verify_yield=None, verify_agreement=None, thresholds=th,
            warnings=warnings)

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
                             thresholds=th, warnings=warnings)

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
            thresholds=th, warnings=warnings)

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
        thresholds=th, warnings=warnings)


# --------------------------------------------------------------------------- #
#  Report
# --------------------------------------------------------------------------- #
_VERDICT_ICON = {"RESILIENT": "🟢", "TEMPERED": "🟡",
                 "KNIFE_EDGE": "🔴", "NO_FIT": "⚪"}


def render_genesis_md(res: GenesisResult,
                      targets: Optional[GenesisTargets] = None) -> str:
    """The one-page markdown the design review reads."""
    L: List[str] = ["# 🧬 InverseGenesis — stochastic inverse report", ""]
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
                 + (f"; boundary filter refused {w.keepout_rejections} "
                    "step(s)" if w.keepout_rejections else ""))
        if w.clamped:
            L.append(f"- pinned to the legal box: {', '.join(w.clamped)}")
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
             "corner being designed. Validate the generated geometry in "
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
    print("self-test passed ✓")
