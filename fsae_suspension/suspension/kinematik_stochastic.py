# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/kinematik_stochastic.py — 🎲🛡️ Stochastic Inversion: the
#  metrology-informed digital twin. Monte Carlo tolerance sweep over the
#  hardpoints, robust re-centring of the nominal against an ASYMMETRIC
#  manufacturing error field, and the as-built → shim-pack Alignment
#  Prescription that turns weld error into an arithmetic problem.
# ============================================================================
"""
Stochastic Inversion — design the corner your shop can actually build.

WHY THIS MODULE EXISTS
----------------------
Every kinematics tool in the industry — including the rest of this repo, until
now — takes hardpoints as EXACT coordinates. Out on the shop floor the car is
never built exactly: welds pull tabs by millimetres, jigs stack error, rod
ends carry play. The deck describes one car; the welder builds another; the
solver has never once been asked which of the two it was solving. A geometry
optimised to a knife-edge peak on screen degrades the moment a tab lands
1.5 mm off — and nobody finds out until the tyre-temperature stickers come
back wrong at the first test day.

This module inverts the question. Instead of "what does THIS geometry do?",
it asks three questions no siloed CAD → kinematics → FEA chain can:

  1. THE TOLERANCE SWEEP — given the error field your shop actually produces
     (per-point, per-axis, ASYMMETRIC bounds, because a weld pulls the tab
     toward the bead, it doesn't scatter symmetrically), what fraction of the
     cars you could build still meet the kinematic intent? That fraction is
     the YIELD — the number that separates a robust geometry from a fragile
     one that only exists on screen.

  2. THE ROBUST NUDGE — an asymmetric field means the EXPECTED as-built car
     is biased off the design intent before the first cut: E[Δmetric] =
     J·E[δ] ≠ 0. Shifting the NOMINAL by the solution of J·x ≈ −J·μ
     re-centres the whole error cloud inside the acceptance bands: you aim
     up-wind of the weld pull, and the population of buildable cars lands on
     target. When the field is symmetric the module says so out loud — no
     nominal shift can raise a linear yield around a centred cloud, and the
     honest fixes are named (better jigging, wider bands) instead of a
     fabricated "optimised" coordinate.

  3. THE ALIGNMENT PRESCRIPTION — after welding, measure the as-built points
     (calipers or a CMM arm), paste them in, and the module solves the shim /
     rod-end adjustments — over the adjusters the car ACTUALLY has, each with
     its axis, range and step — that restore the kinematic intent. The
     residual that no shim can reach is printed, with the missing adjustment
     direction named, instead of being rounded away.

THE HONEST TRICK, STATED AND PRICED (why no cluster is needed)
--------------------------------------------------------------
A full Monte Carlo of N geometries × a travel sweep × a Levenberg–Marquardt
solve per point is real work (~15 ms per geometry on a laptop). The shortcut
is FIRST-ORDER PROPAGATION: a central-difference sensitivity matrix J
(metric × hardpoint-coordinate, built from ~2·coords full solves, once) maps
the sampled error cloud to metric deltas by pure matrix arithmetic — 5,000
samples in microseconds. The price of the linearisation is MEASURED, not
assumed: a verification subsample of full nonlinear solves is run alongside,
and the pass/fail agreement between the linear model and the true solver is
reported with every result. Agreement below the honesty threshold demotes the
result and tells you to run the full sweep. Both modes ship; neither lies
about which one ran.

SCOPE, HONESTLY
---------------
* Errors are INDEPENDENT per point. A jig that shifts a whole tab cluster
  together is correlated error — model it by moving the nominal, not the
  field. Stated here and in the report footer.
* The links are assumed built-to-fit: rod ends thread to length against the
  as-built pickups, so a shifted pickup changes GEOMETRY, not preload. This
  is manufacturing error, not compliance — compliance.py owns the loaded
  deflection, and the two compose downstream (Ghost Topology).
* The yield judges KINEMATIC intent (camber in bump, bump steer, roll-centre
  height, scrub, caster) against the nominal design values. It does not judge
  clearances (Phantom Envelope), thermal windows (ThermicPatch) or structural
  margins (Ghost Topology) — it hands them a population instead of a point.
* Deterministic end to end: a fixed seed drives the sampler, so the same
  inputs give byte-identical yields, attributions and markdown.

Self-test: ``python3 -m suspension.kinematik_stochastic``
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field as _dcfield
from typing import Dict, List, Optional, Tuple

from .kinematics import Hardpoints, SuspensionKinematics
from .ghost_topology import _rc_height_mm


# --------------------------------------------------------------------------- #
#  The metric vector — the kinematic intent the yield is judged against.
# --------------------------------------------------------------------------- #
METRICS: Tuple[str, ...] = (
    "camber_bump_deg",   # camber at full bump travel (the tyre's loaded lean)
    "bump_steer_deg",    # toe change full droop → full bump (the wander)
    "rc_height_mm",      # roll-centre height at static ride
    "scrub_mm",          # scrub radius at static ride
    "caster_deg",        # caster at static ride
)

_METRIC_LABELS = {
    "camber_bump_deg": "camber at full bump (°)",
    "bump_steer_deg":  "bump steer, droop→bump (°)",
    "rc_height_mm":    "roll-centre height (mm)",
    "scrub_mm":        "scrub radius (mm)",
    "caster_deg":      "caster (°)",
}

# The hardpoint fields a tolerance field may perturb. Wheel centre / contact
# patch are excluded on purpose: they are located by the upright machining and
# hub, not by shop-floor welds; include them by editing the field explicitly.
PERTURBABLE_POINTS: Tuple[str, ...] = (
    "upper_front_inner", "upper_rear_inner",
    "lower_front_inner", "lower_rear_inner",
    "upper_outer", "lower_outer",
    "tie_rod_inner", "tie_rod_outer",
)

_AXES = ("x", "y", "z")


def metrics_of(hp: Hardpoints, travel_mm: float = 25.0,
               n_travel: int = 5) -> Tuple[np.ndarray, bool]:
    """The metric vector for one geometry, plus a converged flag.

    A short warm-started sweep (droop → bump) keeps the solver on the correct
    branch; a single non-converged state anywhere in the sweep marks the whole
    sample failed — a geometry the solver cannot follow is a geometry the
    yield must charge for, not paper over.
    """
    try:
        kin = SuspensionKinematics(hp)
        states = kin.sweep(travel_min=-travel_mm, travel_max=travel_mm,
                           n=n_travel)
    except Exception:
        return np.full(len(METRICS), np.nan), False
    if not states or any(not getattr(s, "converged", True) for s in states):
        return np.full(len(METRICS), np.nan), False
    by_travel = {round(s.travel, 6): s for s in states}
    bump = states[-1]
    droop = states[0]
    static = by_travel.get(0.0, states[len(states) // 2])
    vec = np.array([
        bump.camber,
        bump.toe - droop.toe,
        _rc_height_mm(static, track_mm=1200.0),
        static.scrub_radius,
        static.caster,
    ], dtype=float)
    if not np.all(np.isfinite(vec)):
        return np.full(len(METRICS), np.nan), False
    return vec, True


def _perturbed(hp: Hardpoints, offsets: Dict[str, np.ndarray]) -> Hardpoints:
    """A copy of hp with each named point shifted by its 3-vector offset."""
    new = hp.copy() if hasattr(hp, "copy") else Hardpoints(**{
        k: (np.asarray(v, float).copy() if isinstance(v, np.ndarray) else v)
        for k, v in hp.__dict__.items()})
    for name, off in offsets.items():
        cur = getattr(new, name, None)
        if cur is not None:
            setattr(new, name, np.asarray(cur, float) + np.asarray(off, float))
    return new


# --------------------------------------------------------------------------- #
#  The manufacturing tolerance field — asymmetric, per point, per axis.
# --------------------------------------------------------------------------- #
@dataclass
class ToleranceSpec:
    """Error bounds for ONE point: lo ≤ error ≤ hi per axis, mm.

    Asymmetry is the whole point: a hand weld pulls the tab TOWARD the bead,
    so a realistic spec is e.g. lo=(-0.3,-0.3,-0.3), hi=(+1.5,+0.3,+0.3) —
    biased +x. ``dist`` is "uniform" (worst-case honest box) or "normal"
    (truncated at the bounds, σ = span/4 — a shop that mostly lands mid-box).
    """
    lo: np.ndarray            # (3,) mm, each ≤ 0 expected but not required
    hi: np.ndarray            # (3,) mm
    dist: str = "uniform"

    def __post_init__(self):
        self.lo = np.asarray(self.lo, float).reshape(3)
        self.hi = np.asarray(self.hi, float).reshape(3)
        if np.any(self.hi < self.lo):
            raise ValueError("ToleranceSpec: hi < lo on some axis.")
        if self.dist not in ("uniform", "normal"):
            raise ValueError(f"Unknown distribution '{self.dist}'.")

    @staticmethod
    def symmetric(r_mm: float, dist: str = "uniform") -> "ToleranceSpec":
        r = abs(float(r_mm))
        return ToleranceSpec(lo=np.full(3, -r), hi=np.full(3, r), dist=dist)

    @property
    def mean(self) -> np.ndarray:
        """E[error] per axis. Uniform: mid-box. Truncated normal centred on
        the mid-box: also the mid-box (symmetric truncation) — exact, not
        approximate, because the sampler centres the normal there."""
        return 0.5 * (self.lo + self.hi)

    @property
    def var(self) -> np.ndarray:
        """Var[error] per axis (first-order attribution uses this)."""
        span = self.hi - self.lo
        if self.dist == "uniform":
            return span ** 2 / 12.0
        # normal, σ = span/4, truncated at ±2σ: Var = σ²(1 − 4φ(2)/(Φ(2)−Φ(−2)))
        sig = span / 4.0
        shrink = 1.0 - 4.0 * _phi(2.0) / (2.0 * _Phi(2.0) - 1.0)
        return (sig ** 2) * shrink


def _phi(x: float) -> float:
    return float(np.exp(-0.5 * x * x) / np.sqrt(2.0 * np.pi))


def _Phi(x: float) -> float:
    from math import erf, sqrt
    return 0.5 * (1.0 + erf(x / sqrt(2.0)))


@dataclass
class ToleranceField:
    """point name → ToleranceSpec. Only listed points are perturbed."""
    specs: Dict[str, ToleranceSpec] = _dcfield(default_factory=dict)

    def __post_init__(self):
        for name in self.specs:
            if name not in PERTURBABLE_POINTS:
                raise ValueError(
                    f"'{name}' is not a perturbable hardpoint. Allowed: "
                    f"{', '.join(PERTURBABLE_POINTS)}.")

    # ---- shop presets — representative, not measured; edit to your shop ----
    @staticmethod
    def preset(shop: str = "hand_weld",
               weld_pull_mm: float = 0.0,
               pull_axis: str = "x") -> "ToleranceField":
        """Seed a field from a shop class.

        hand_weld : hand-welded tabs ±1.5 mm, machined outers ±0.2 mm
        jig_weld  : jigged tabs ±0.5 mm, machined outers ±0.15 mm
        cnc       : machined everything ±0.05 mm (the aerospace fantasy —
                    useful as the control case that shows what tolerance
                    actually costs you)

        ``weld_pull_mm`` biases every CHASSIS tab +pull_axis by that much
        (hi += pull) — the systematic draw of a weld bead laid on one side.
        """
        tabs = dict(hand_weld=1.5, jig_weld=0.5, cnc=0.05).get(shop)
        if tabs is None:
            raise ValueError(f"Unknown shop preset '{shop}'.")
        outers = dict(hand_weld=0.2, jig_weld=0.15, cnc=0.05)[shop]
        ax = {"x": 0, "y": 1, "z": 2}[pull_axis]
        specs: Dict[str, ToleranceSpec] = {}
        for p in ("upper_front_inner", "upper_rear_inner",
                  "lower_front_inner", "lower_rear_inner"):
            spec = ToleranceSpec.symmetric(tabs)
            if weld_pull_mm:
                hi = spec.hi.copy()
                hi[ax] += abs(float(weld_pull_mm))
                spec = ToleranceSpec(lo=spec.lo, hi=hi, dist=spec.dist)
            specs[p] = spec
        # the tie-rod inner rides the machined rack, and the outboard points
        # are located by the machined upright — machining class, not weld class
        for p in ("tie_rod_inner", "upper_outer", "lower_outer",
                  "tie_rod_outer"):
            specs[p] = ToleranceSpec.symmetric(outers)
        return ToleranceField(specs)

    # ---- flattening for the linear algebra --------------------------------
    def coords(self) -> List[Tuple[str, int]]:
        """Deterministic (point, axis) coordinate order."""
        return [(p, a) for p in sorted(self.specs) for a in range(3)]

    def sample(self, n: int, seed: int = 0) -> np.ndarray:
        """(n, n_coords) error samples, mm — deterministic for a given seed."""
        rng = np.random.default_rng(int(seed))
        cols = []
        for p in sorted(self.specs):
            s = self.specs[p]
            if s.dist == "uniform":
                u = rng.uniform(s.lo, s.hi, size=(n, 3))
            else:
                mid, sig = s.mean, (s.hi - s.lo) / 4.0
                u = rng.normal(mid, np.where(sig > 0, sig, 1.0), size=(n, 3))
                u = np.clip(u, s.lo, s.hi)
                u = np.where((s.hi - s.lo) > 0, u, mid)
            cols.append(u)
        return np.hstack(cols) if cols else np.zeros((n, 0))

    def mean_vec(self) -> np.ndarray:
        return np.concatenate([self.specs[p].mean for p in sorted(self.specs)]) \
            if self.specs else np.zeros(0)

    def var_vec(self) -> np.ndarray:
        return np.concatenate([self.specs[p].var for p in sorted(self.specs)]) \
            if self.specs else np.zeros(0)


# --------------------------------------------------------------------------- #
#  The acceptance bands — what "still the car you designed" means, in numbers.
# --------------------------------------------------------------------------- #
@dataclass
class YieldSpec:
    """Max acceptable |as-built − nominal| per metric. Defaults are a sane
    FSAE tuning envelope; every band is a design decision — edit them."""
    camber_bump_deg: float = 0.25
    bump_steer_deg: float = 0.10
    rc_height_mm: float = 8.0
    scrub_mm: float = 4.0
    caster_deg: float = 0.30

    def bands(self) -> np.ndarray:
        return np.array([abs(getattr(self, m)) for m in METRICS], float)


@dataclass
class StochasticThresholds:
    robust_yield: float = 0.95      # ≥ this → ROBUST
    marginal_yield: float = 0.80    # ≥ this → MARGINAL, below → FRAGILE
    solver_fail_frac: float = 0.02  # > this non-converged → SOLVER_LIMITED
    verify_agreement: float = 0.98  # linear-vs-full pass/fail agreement floor


# --------------------------------------------------------------------------- #
#  The sensitivity matrix — one honest Jacobian, priced.
# --------------------------------------------------------------------------- #
@dataclass
class Sensitivity:
    coords: List[Tuple[str, int]]      # (point, axis) per column
    J: np.ndarray                      # (n_metrics, n_coords), unit per mm
    nominal: np.ndarray                # metric vector at the nominal geometry
    step_mm: float

    def coord_labels(self) -> List[str]:
        return [f"{p}.{_AXES[a]}" for p, a in self.coords]


def sensitivity(hp: Hardpoints, fld: ToleranceField,
                step_mm: float = 0.25) -> Sensitivity:
    """Central-difference dMetric/dCoordinate at the nominal geometry."""
    nominal, ok = metrics_of(hp)
    if not ok:
        raise ValueError("The NOMINAL geometry does not solve — fix the "
                         "hardpoints before asking about its tolerance.")
    coords = fld.coords()
    J = np.zeros((len(METRICS), len(coords)))
    for j, (p, a) in enumerate(coords):
        off = np.zeros(3)
        off[a] = step_mm
        mp, okp = metrics_of(_perturbed(hp, {p: off}))
        mm_, okm = metrics_of(_perturbed(hp, {p: -off}))
        if not (okp and okm):
            raise ValueError(
                f"Sensitivity probe failed at {p}.{_AXES[a]} ±{step_mm} mm — "
                "the nominal sits at the edge of the solvable region; the "
                "tolerance question is already answered (FRAGILE).")
        J[:, j] = (mp - mm_) / (2.0 * step_mm)
    return Sensitivity(coords=coords, J=J, nominal=nominal, step_mm=step_mm)


# --------------------------------------------------------------------------- #
#  The stochastic sweep.
# --------------------------------------------------------------------------- #
@dataclass
class StochasticResult:
    ok: bool
    mode: str                       # "linear" | "full"
    n: int
    seed: int
    yield_frac: float
    verdict: str                    # ROBUST | MARGINAL | FRAGILE | SOLVER_LIMITED
    fail_frac_per_metric: np.ndarray        # (n_metrics,)
    solver_fail_frac: float
    bias: np.ndarray                # E[Δmetric] = J·μ  (the asymmetry bill)
    p05: np.ndarray                 # 5th / 95th percentile Δmetric
    p95: np.ndarray
    attribution: np.ndarray         # (n_metrics, n_coords) variance share, linear
    sens: Sensitivity
    bands: np.ndarray
    verify_agreement: Optional[float]       # linear mode only
    verify_worst_err: Optional[np.ndarray]  # worst |linear−full| per metric
    warnings: List[str]

    def worst_metric(self) -> Tuple[str, float]:
        i = int(np.argmax(self.fail_frac_per_metric))
        return METRICS[i], float(self.fail_frac_per_metric[i])

    def dominant_coord(self, metric: str) -> Tuple[str, float]:
        i = METRICS.index(metric)
        j = int(np.argmax(self.attribution[i]))
        return self.sens.coord_labels()[j], float(self.attribution[i, j])


def _judge(deltas: np.ndarray, bands: np.ndarray) -> np.ndarray:
    """(n,) bool pass — every metric inside its band."""
    return np.all(np.abs(deltas) <= bands[None, :], axis=1)


def stochastic_sweep(hp: Hardpoints, fld: ToleranceField,
                     yspec: Optional[YieldSpec] = None,
                     n: int = 5000, seed: int = 0, mode: str = "linear",
                     n_verify: int = 120,
                     thresholds: Optional[StochasticThresholds] = None,
                     sens: Optional[Sensitivity] = None,
                     target: Optional[np.ndarray] = None) -> StochasticResult:
    """Run the tolerance sweep and return the yield with its full anatomy.

    mode="linear": first-order propagation of ``n`` samples through J, plus
        ``n_verify`` full nonlinear solves to PRICE the linearisation — the
        pass/fail agreement and worst metric error are reported, and poor
        agreement demotes the result with a warning instead of hiding.
    mode="full": every sample is a full solve. Slow (~15 ms/sample) and exact.

    ``target``: the metric vector the deviation is judged against. Default is
    this geometry's own nominal metrics. Pass the ORIGINAL design intent when
    sweeping a shifted nominal (the robust nudge does) — otherwise the judge
    quietly moves with the goalposts.
    """
    yspec = yspec or YieldSpec()
    th = thresholds or StochasticThresholds()
    bands = yspec.bands()
    warnings: List[str] = []
    if not fld.specs:
        raise ValueError("Empty tolerance field — declare at least one point.")
    if mode not in ("linear", "full"):
        raise ValueError(f"mode must be 'linear' or 'full', got '{mode}'.")

    sens = sens or sensitivity(hp, fld)
    tgt = sens.nominal if target is None else np.asarray(target, float)
    shift0 = sens.nominal - tgt                     # nominal's own offset from
    samples = fld.sample(n, seed=seed)              # the judged target (n, nc)
    bias = sens.J @ fld.mean_vec() + shift0

    solver_fail = 0
    verify_agreement = None
    verify_worst = None

    if mode == "full":
        deltas = np.zeros((n, len(METRICS)))
        okmask = np.ones(n, bool)
        for i in range(n):
            offs = _unflatten(samples[i], sens.coords)
            m, ok = metrics_of(_perturbed(hp, offs))
            okmask[i] = ok
            deltas[i] = (m - tgt) if ok else np.inf
        solver_fail = int(np.sum(~okmask))
        passed = _judge(np.where(okmask[:, None], deltas, np.inf), bands)
        deltas_stat = deltas[okmask] if okmask.any() else np.zeros((0, len(METRICS)))
    else:
        deltas = samples @ sens.J.T + shift0[None, :]   # (n, n_metrics)
        passed = _judge(deltas, bands)
        deltas_stat = deltas
        # ---- price the linearisation on a deterministic subsample ---------
        nv = min(int(n_verify), n)
        if nv > 0:
            worst = np.zeros(len(METRICS))
            agree = 0
            nv_ok = 0
            for i in range(nv):                     # first nv samples: deterministic
                offs = _unflatten(samples[i], sens.coords)
                m, ok = metrics_of(_perturbed(hp, offs))
                if not ok:
                    solver_fail += 1
                    continue
                nv_ok += 1
                true_d = m - tgt
                worst = np.maximum(worst, np.abs(true_d - deltas[i]))
                if bool(np.all(np.abs(true_d) <= bands)) == bool(passed[i]):
                    agree += 1
            verify_agreement = (agree / nv_ok) if nv_ok else 0.0
            verify_worst = worst
            solver_fail_frac_v = solver_fail / nv
            if verify_agreement < th.verify_agreement:
                warnings.append(
                    f"Linear model agrees with the full solver on only "
                    f"{verify_agreement*100:.0f}% of verified samples "
                    f"(floor {th.verify_agreement*100:.0f}%) — run mode='full' "
                    "before trusting this yield.")
            solver_fail = int(round(solver_fail_frac_v * n))  # extrapolated, stated
            if solver_fail_frac_v > 0:
                warnings.append(
                    "Solver failures in the verification subsample were "
                    "extrapolated to the full sample count — an estimate, "
                    "stated as one.")

    yield_frac = float(np.sum(passed)) / n if n else 0.0
    if mode == "linear":
        # solver failures found in verification reduce the honest yield:
        yield_frac = max(0.0, yield_frac - solver_fail / n)

    fail_pm = np.array([
        float(np.mean(np.abs(deltas_stat[:, i]) > bands[i]))
        if len(deltas_stat) else 1.0
        for i in range(len(METRICS))])

    p05 = (np.percentile(deltas_stat, 5, axis=0) if len(deltas_stat)
           else np.full(len(METRICS), np.nan))
    p95 = (np.percentile(deltas_stat, 95, axis=0) if len(deltas_stat)
           else np.full(len(METRICS), np.nan))

    # ---- first-order variance attribution (linear by construction, said so)
    var = fld.var_vec()
    contrib = (sens.J ** 2) * var[None, :]
    tot = np.sum(contrib, axis=1, keepdims=True)
    attribution = np.where(tot > 0, contrib / np.where(tot > 0, tot, 1.0), 0.0)

    sf_frac = solver_fail / n if n else 0.0
    if sf_frac > th.solver_fail_frac:
        verdict = "SOLVER_LIMITED"
        warnings.append(
            f"{sf_frac*100:.1f}% of perturbed geometries failed to solve — "
            "the nominal sits near a kinematic singularity; the yield below "
            "is a lower bound and the geometry is fragile in a way no shim "
            "fixes.")
    elif yield_frac >= th.robust_yield:
        verdict = "ROBUST"
    elif yield_frac >= th.marginal_yield:
        verdict = "MARGINAL"
    else:
        verdict = "FRAGILE"

    return StochasticResult(
        ok=True, mode=mode, n=n, seed=seed, yield_frac=yield_frac,
        verdict=verdict, fail_frac_per_metric=fail_pm,
        solver_fail_frac=sf_frac, bias=bias, p05=p05, p95=p95,
        attribution=attribution, sens=sens, bands=bands,
        verify_agreement=verify_agreement, verify_worst_err=verify_worst,
        warnings=warnings)


def _unflatten(vec: np.ndarray, coords: List[Tuple[str, int]]
               ) -> Dict[str, np.ndarray]:
    offs: Dict[str, np.ndarray] = {}
    for (p, a), v in zip(coords, vec):
        offs.setdefault(p, np.zeros(3))[a] = v
    return offs


# --------------------------------------------------------------------------- #
#  The robust nudge — aim up-wind of the weld pull.
# --------------------------------------------------------------------------- #
@dataclass
class RobustNudge:
    ok: bool
    reason: str
    shifts: Dict[str, np.ndarray]           # point -> nominal shift, mm
    predicted_yield: float                  # linear model, after the nudge
    baseline_yield: float                   # linear model, before
    verified_yield: Optional[float]         # full-solve check on the nudged
    clamped: List[str]                      # coordinates that hit the freedom box


def robust_nudge(hp: Hardpoints, fld: ToleranceField,
                 res: StochasticResult,
                 freedom_mm: float | Dict[str, float] = 3.0,
                 seed: int = 0, n_verify_full: int = 0) -> RobustNudge:
    """Re-centre the nominal against an asymmetric field.

    Linear model: as-built Δmetric = J·(δ + x) where x is the nominal shift.
    Choosing x s.t. J·x ≈ −J·μ (band-weighted least squares over the coords
    the field owns) centres E[Δ] on zero — the design aims where the weld
    pull will land it. ``freedom_mm`` bounds |x| per coordinate (one number,
    or per-point); coordinates that clamp are named. If the field is already
    centred (|bias| negligible against the bands) the honest answer is
    returned: no nominal shift can raise a linear yield — improve the shop
    (shrink the field) or renegotiate the bands.

    ``n_verify_full`` > 0 additionally re-runs the FULL sweep on the nudged
    nominal with that many samples — the linear prediction, priced.
    """
    bands, J = res.bands, res.sens.J
    mu = fld.mean_vec()
    bias = J @ mu
    if np.all(np.abs(bias) <= 0.02 * bands):
        return RobustNudge(
            ok=False,
            reason=("The error field is centred on the design intent "
                    "(|E[Δmetric]| ≤ 2% of every band): no nominal shift can "
                    "raise a first-order yield around a centred cloud. The "
                    "levers that CAN: shrink the field (jig the tabs, "
                    "measure-and-sort rod ends) or widen the acceptance "
                    "bands — both design decisions, neither free."),
            shifts={}, predicted_yield=res.yield_frac,
            baseline_yield=res.yield_frac, verified_yield=None, clamped=[])

    # band-weighted least squares: minimise ||W(Jx + bias)||, W = 1/band
    W = 1.0 / np.where(bands > 0, bands, 1.0)
    A = J * W[:, None]
    b = -(bias * W)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)

    # clamp to the freedom boxes, honestly listing clamped coordinates
    labels = res.sens.coord_labels()
    clamped: List[str] = []
    for j, (p, a) in enumerate(res.sens.coords):
        lim = abs(float(freedom_mm[p] if isinstance(freedom_mm, dict)
                        else freedom_mm))
        if abs(x[j]) > lim:
            x[j] = np.sign(x[j]) * lim
            clamped.append(labels[j])

    # predicted linear yield after the nudge — same samples, shifted cloud
    samples = fld.sample(res.n, seed=seed)
    deltas = (samples + x[None, :]) @ J.T
    predicted = float(np.mean(_judge(deltas, bands)))

    shifts = {p: v for p, v in _unflatten(x, res.sens.coords).items()
              if np.any(np.abs(v) > 1e-9)}

    verified = None
    if n_verify_full > 0:
        hp2 = _perturbed(hp, shifts)
        try:
            r2 = stochastic_sweep(hp2, fld, YieldSpec(**{
                m: float(bands[i]) for i, m in enumerate(METRICS)}),
                n=n_verify_full, seed=seed, mode="full",
                target=res.sens.nominal)     # judged vs the ORIGINAL intent
            verified = r2.yield_frac
        except ValueError:
            verified = 0.0

    reason = ("Asymmetric field: the expected as-built car is biased "
              + ", ".join(f"{_METRIC_LABELS[m]} {bias[i]:+.3g}"
                          for i, m in enumerate(METRICS)
                          if abs(bias[i]) > 0.02 * bands[i])
              + " before the first cut. The nudge below aims the nominal "
                "up-wind of that pull.")
    return RobustNudge(ok=True, reason=reason, shifts=shifts,
                       predicted_yield=predicted,
                       baseline_yield=res.yield_frac,
                       verified_yield=verified, clamped=clamped)


# --------------------------------------------------------------------------- #
#  The metrology feedback loop — as-built in, shim pack out.
# --------------------------------------------------------------------------- #
@dataclass
class Adjuster:
    """One physical adjustment the built car actually has.

    point : the hardpoint it moves (e.g. "tie_rod_inner")
    axis  : "x"|"y"|"z" or a 3-vector direction (normalised internally)
    lo/hi : travel limits, mm (shim stack range, rod-end thread range)
    step  : quantisation, mm (shim thickness; 0 = continuous)
    label : how the prescription names it ("front-left lower aft shim pack")
    """
    point: str
    axis: object = "z"
    lo: float = -3.0
    hi: float = 3.0
    step: float = 0.5
    label: str = ""

    def direction(self) -> np.ndarray:
        if isinstance(self.axis, str):
            d = np.zeros(3)
            d[{"x": 0, "y": 1, "z": 2}[self.axis]] = 1.0
            return d
        d = np.asarray(self.axis, float).reshape(3)
        n = np.linalg.norm(d)
        if n < 1e-12:
            raise ValueError("Adjuster axis has zero length.")
        return d / n

    def name(self) -> str:
        ax = self.axis if isinstance(self.axis, str) else "custom-axis"
        return self.label or f"{self.point} ({ax})"


@dataclass
class Prescription:
    ok: bool
    verdict: str                    # RESTORED | PARTIAL | UNSHIMMABLE
    moves_mm: List[float]           # per adjuster, rounded to its step
    adjusters: List[Adjuster]
    delta_before: np.ndarray        # as-built − nominal metrics
    delta_after: np.ndarray         # full-solve residual after the moves
    bands: np.ndarray
    unreachable: List[str]          # metrics no declared adjuster can move
    warnings: List[str]

    def lines(self) -> List[str]:
        out = []
        for adj, mv in zip(self.adjusters, self.moves_mm):
            if abs(mv) < 1e-9:
                continue
            out.append(f"{adj.name()}: {mv:+.2f} mm")
        return out or ["no adjustment required"]


def alignment_prescription(hp_nominal: Hardpoints,
                           as_built: Dict[str, np.ndarray],
                           adjusters: List[Adjuster],
                           yspec: Optional[YieldSpec] = None) -> Prescription:
    """Turn measured as-built coordinates into a shim-pack prescription.

    1. The as-built geometry (nominal + measured shifts) is solved in FULL —
       the deviation the car really has, not a linear estimate of it.
    2. Adjuster sensitivities are finite-differenced AT THE AS-BUILT geometry
       (the car the wrenches will touch), band-weighted least squares picks
       the moves, each move is rounded to its shim step and clamped to its
       range.
    3. The result is VERIFIED by a final full solve of (as-built + moves);
       the residual printed is the residual the real car will carry.

    Metrics that live in the null space of the declared adjusters are listed
    as unreachable — the honest sentence is "you have no adjuster for this",
    never a rounded-away zero.
    """
    yspec = yspec or YieldSpec()
    bands = yspec.bands()
    warnings: List[str] = []
    for name in as_built:
        if not hasattr(hp_nominal, name) or getattr(hp_nominal, name) is None:
            raise ValueError(f"Measured point '{name}' is not a hardpoint.")
    if not adjusters:
        raise ValueError("Declare at least one Adjuster — a prescription "
                         "with nothing to turn is a wish.")

    m_nom, ok0 = metrics_of(hp_nominal)
    if not ok0:
        raise ValueError("Nominal geometry does not solve.")
    shifts = {k: np.asarray(v, float) - np.asarray(getattr(hp_nominal, k), float)
              for k, v in as_built.items()}
    # ---- plausibility gate (the Saboteur lesson): a measured point tens of
    # millimetres from nominal is not a weld error, it is a units slip
    # (metres-as-mm), a frame slip (Z-down sheet) or a mis-typed row. The LM
    # solver will often still CONVERGE on such geometry — to a car that does
    # not exist — so the gate refuses loudly instead of shimming garbage.
    for k, dv in shifts.items():
        d = float(np.linalg.norm(dv))
        if d > 25.0:
            raise ValueError(
                f"As-built '{k}' sits {d:.0f} mm from nominal — no weld "
                "pulls that far. Check units (metres pasted as mm?), the "
                "frame/datum (Z-down sheet in a Z-up deck?), and the row "
                "itself before asking for shims.")
    hp_built = _perturbed(hp_nominal, shifts)
    m_built, ok1 = metrics_of(hp_built)
    if not ok1:
        raise ValueError("The AS-BUILT geometry does not solve — re-check the "
                         "measured coordinates (frame? units? a swapped axis?) "
                         "before shimming anything.")
    d0 = m_built - m_nom

    # adjuster Jacobian at the as-built geometry
    h = 0.25
    Ja = np.zeros((len(METRICS), len(adjusters)))
    for j, adj in enumerate(adjusters):
        d = adj.direction() * h
        mp, okp = metrics_of(_perturbed(hp_built, {adj.point: d}))
        mm_, okm = metrics_of(_perturbed(hp_built, {adj.point: -d}))
        if not (okp and okm):
            warnings.append(f"Adjuster '{adj.name()}' probe failed to solve; "
                            "treated as immovable.")
            continue
        Ja[:, j] = (mp - mm_) / (2.0 * h)

    W = 1.0 / np.where(bands > 0, bands, 1.0)
    x, *_ = np.linalg.lstsq(Ja * W[:, None], -(d0 * W), rcond=None)

    moves: List[float] = []
    for j, adj in enumerate(adjusters):
        v = float(np.clip(x[j], adj.lo, adj.hi))
        if adj.step and adj.step > 0:
            v = round(v / adj.step) * adj.step
        moves.append(v)

    # unreachable metrics: row of Ja ~ 0 → no declared adjuster moves it
    reach = np.max(np.abs(Ja), axis=1)          # |dM/dx| per metric row
    unreachable = [METRICS[i] for i in range(len(METRICS))
                   if reach[i] < 1e-6 and abs(d0[i]) > bands[i]]

    # verify with a full solve
    move_offs: Dict[str, np.ndarray] = {}
    for adj, mv in zip(adjusters, moves):
        move_offs[adj.point] = move_offs.get(adj.point, np.zeros(3)) \
            + adj.direction() * mv
    m_after, ok2 = metrics_of(_perturbed(hp_built, move_offs))
    if not ok2:
        warnings.append("The shimmed geometry failed to solve; prescription "
                        "withdrawn — residuals shown are the UN-shimmed car.")
        m_after = m_built
        moves = [0.0] * len(adjusters)
    d1 = m_after - m_nom

    inside = np.abs(d1) <= bands
    improved = np.abs(d1) <= np.abs(d0) + 1e-9
    if np.all(inside):
        verdict = "RESTORED"
    elif unreachable or not np.all(improved):
        verdict = "UNSHIMMABLE" if unreachable else "PARTIAL"
    else:
        verdict = "PARTIAL"

    return Prescription(ok=True, verdict=verdict, moves_mm=moves,
                        adjusters=adjusters, delta_before=d0, delta_after=d1,
                        bands=bands, unreachable=unreachable,
                        warnings=warnings)


# --------------------------------------------------------------------------- #
#  Markdown reports.
# --------------------------------------------------------------------------- #
def render_stochastic_md(res: StochasticResult,
                         nudge: Optional[RobustNudge] = None,
                         title: str = "corner") -> str:
    L: List[str] = []
    L.append(f"# 🎲🛡️ Stochastic Inversion — {title}")
    L.append("")
    L.append(f"**Verdict: {res.verdict}** — manufacturing yield "
             f"**{res.yield_frac*100:.1f}%** over {res.n} sampled builds "
             f"(mode: {res.mode}, seed {res.seed}).")
    if res.mode == "linear" and res.verify_agreement is not None:
        L.append(f"Linearisation priced: pass/fail agreement with the full "
                 f"solver {res.verify_agreement*100:.0f}% on the verification "
                 "subsample.")
    L.append("")
    L.append("| metric | band ± | E[Δ] (bias) | Δ p5 | Δ p95 | fail % | "
             "dominant error source |")
    L.append("|---|---|---|---|---|---|---|")
    for i, m in enumerate(METRICS):
        lab, share = res.dominant_coord(m)
        L.append(f"| {_METRIC_LABELS[m]} | {res.bands[i]:.3g} | "
                 f"{res.bias[i]:+.3g} | {res.p05[i]:+.3g} | "
                 f"{res.p95[i]:+.3g} | {res.fail_frac_per_metric[i]*100:.1f} | "
                 f"{lab} ({share*100:.0f}%) |")
    L.append("")
    if nudge is not None:
        L.append("## Robust nudge")
        L.append("")
        L.append(nudge.reason)
        L.append("")
        if nudge.ok:
            for p, v in sorted(nudge.shifts.items()):
                L.append(f"- shift **{p}** by "
                         f"[{v[0]:+.2f}, {v[1]:+.2f}, {v[2]:+.2f}] mm")
            L.append(f"- predicted yield {nudge.baseline_yield*100:.1f}% → "
                     f"**{nudge.predicted_yield*100:.1f}%** (linear model"
                     + (f"; full-solve verified {nudge.verified_yield*100:.1f}%"
                        if nudge.verified_yield is not None else "") + ")")
            if nudge.clamped:
                L.append("- freedom-box clamps: " + ", ".join(nudge.clamped))
        L.append("")
    for w in res.warnings:
        L.append(f"- ⚠️ {w}")
    L.append("")
    L.append("_Independent per-point errors, links built-to-fit, first-order "
             "attribution — the scope notes live in the module docstring. "
             "This audits kinematic intent; hand the surviving population to "
             "Ghost Topology / Phantom Envelope / ThermicPatch for loads, "
             "clearance and temperature._")
    return "\n".join(L)


def render_prescription_md(rx: Prescription,
                           title: str = "corner") -> str:
    L: List[str] = []
    L.append(f"# 🔧 Alignment Prescription — {title}")
    L.append("")
    L.append(f"**Verdict: {rx.verdict}**")
    L.append("")
    L.append("## Do this")
    for line in rx.lines():
        L.append(f"- {line}")
    L.append("")
    L.append("| metric | band ± | as-built Δ | after shims Δ |")
    L.append("|---|---|---|---|")
    for i, m in enumerate(METRICS):
        mark = "" if abs(rx.delta_after[i]) <= rx.bands[i] else " ⚠️"
        L.append(f"| {_METRIC_LABELS[m]} | {rx.bands[i]:.3g} | "
                 f"{rx.delta_before[i]:+.3g} | {rx.delta_after[i]:+.3g}{mark} |")
    L.append("")
    if rx.unreachable:
        L.append("**Unreachable with the declared adjusters:** "
                 + ", ".join(_METRIC_LABELS[m] for m in rx.unreachable)
                 + " — no shim direction you declared moves this metric; the "
                   "fix is a different adjuster (or accepting the residual), "
                   "not more turns of the ones you have.")
        L.append("")
    for w in rx.warnings:
        L.append(f"- ⚠️ {w}")
    L.append("")
    L.append("_Residuals are from a full nonlinear re-solve of the shimmed "
             "as-built geometry — the numbers the real car will carry, not a "
             "linear promise._")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  Self-test (python3 -m suspension.kinematik_stochastic)
# --------------------------------------------------------------------------- #
if __name__ == "__main__":   # pragma: no cover
    hp = Hardpoints.default()

    print("=== hand-welded shop, symmetric field ===")
    fld = ToleranceField.preset("hand_weld")
    res = stochastic_sweep(hp, fld, n=5000, seed=0, mode="linear")
    print(render_stochastic_md(res, robust_nudge(hp, fld, res)))
    print()

    print("=== the weld-pull case: tabs drawn +1.2 mm up (weld draw) ===")
    fld2 = ToleranceField.preset("hand_weld", weld_pull_mm=1.2, pull_axis="z")
    res2 = stochastic_sweep(hp, fld2, n=5000, seed=0, mode="linear")
    nud = robust_nudge(hp, fld2, res2, n_verify_full=150)
    print(render_stochastic_md(res2, nud))
    print()

    print("=== metrology feedback: FL upper tab pulled 1.4 mm aft ===")
    as_built = {"upper_front_inner":
                np.asarray(hp.upper_front_inner, float) + np.array([1.4, 0, 0])}
    rx = alignment_prescription(
        hp, as_built,
        adjusters=[Adjuster("tie_rod_inner", "z", -3, 3, 0.25,
                            "tie-rod inner shim"),
                   Adjuster("upper_rear_inner", "x", -3, 3, 0.5,
                            "upper aft shim pack"),
                   Adjuster("lower_rear_inner", "x", -3, 3, 0.5,
                            "lower aft shim pack")])
    print(render_prescription_md(rx))
