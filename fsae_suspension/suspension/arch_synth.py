# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/arch_synth.py — Architecture Synthesis: mixed discrete-continuous
#  multi-objective co-optimization over the vehicle's ARCHITECTURE (discrete
#  switches: wheel size, motor count, pack voltage, inboard vs outboard damper)
#  AND its continuous corner geometry (a subset of hardpoint coordinates),
#  producing a Pareto front instead of a single point.
# ============================================================================
"""
Architecture Synthesis — discrete + continuous, together, honestly.

WHAT THIS DOES
--------------
KinematiK's existing InverseGenesis solves the continuous problem: given a fixed
architecture, pull hardpoint coordinates to hit target kinematic curves. It
cannot answer "13-inch or 10-inch? one motor or four? 400V or 600V?" because
those are DISCRETE switches, and a gradient step has nowhere to go on a binary
choice. This module wraps a mixed-variable multi-objective optimizer around the
same real kinematics solver so the discrete switches and the continuous geometry
are searched at the same time, and returns the Pareto-optimal *set* of
architectures rather than pretending one winner exists.

WHAT IT IS NOT
--------------
This is NOT a magic points predictor. A Pareto front is only as trustworthy as
the objective models feeding it, and this module is scrupulous about which parts
of the objective are real and which are placeholders you MUST calibrate to your
own car before quoting a number to a design judge:

  * PHYSICS-GROUNDED (trustworthy): every kinematic objective — camber gain,
    bump steer, scrub, motion-ratio linearity — is computed by running the real
    ``SuspensionKinematics`` solver from ``kinematics.py`` on the candidate's
    actual hardpoints. These numbers are as good as the solver, which is the
    same solver the rest of KinematiK trusts.

  * PARAMETRIC ESTIMATES (calibrate before quoting): mass, and the mapping from
    engineering metrics to "competition points", use transparent coefficient
    tables declared in ``PointsModel`` / ``MassModel`` below. They are
    defensible first-order estimates, NOT measurements. Every one is labelled,
    every coefficient is exposed, and the report prints which objective terms
    are physics vs parametric so nobody mistakes a tuned guess for a lap time.

The honest posture: the OPTIMIZER is exact (real NSGA-II, real non-dominated
sorting, real crowding distance), the KINEMATICS are exact, and the ECONOMIC
LAYER (points/mass coefficients) is an editable model you own. That separation
is the whole design.

NO NEW DEPENDENCIES
-------------------
The multi-objective engine (NSGA-II) is implemented here in ~200 lines on numpy
alone — no pymoo, no platypus — matching KinematiK's import-light rule so it runs
on the Streamlit Cloud base image and stays unit-testable in isolation.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field, asdict
from typing import Callable

from .kinematics import Hardpoints, SuspensionKinematics


# ===================================================================== #
#  1.  THE ARCHITECTURE SPACE  (discrete switches + continuous vars)
# ===================================================================== #
@dataclass
class DiscreteChoice:
    """One discrete architectural switch and its allowed options."""
    name: str
    options: list          # e.g. [10, 13] or ["inboard", "outboard"]
    label: str = ""        # human label for reports

    def __post_init__(self):
        if not self.label:
            self.label = self.name


@dataclass
class ContinuousVar:
    """One continuous design variable, with bounds, that perturbs geometry.

    ``apply`` receives (Hardpoints copy, value) and mutates it in place. This is
    how a continuous coordinate reaches the real solver: the optimizer proposes a
    number in [lo, hi], ``apply`` writes it into the hardpoint set, and the
    kinematics solver runs on the result. Keeping the mutation in a closure means
    the optimizer never needs to know the geometry's internal layout.
    """
    name: str
    lo: float
    hi: float
    apply: Callable[[Hardpoints, float], None]
    label: str = ""

    def __post_init__(self):
        if not self.label:
            self.label = self.name


def default_discrete_space() -> list[DiscreteChoice]:
    """The four canonical FSAE architectural switches from the feature brief.

    These are the switches a lead actually agonises over and that force a CAD
    restart today. Options are the common FSAE choices; edit freely.
    """
    return [
        DiscreteChoice("wheel_in", [10, 13], "Wheel size (in)"),
        DiscreteChoice("motors", [1, 2, 4], "Motor count"),
        DiscreteChoice("pack_v", [400, 600], "Pack voltage (V)"),
        DiscreteChoice("damper", ["inboard", "outboard"], "Damper layout"),
    ]


def default_continuous_space() -> list[ContinuousVar]:
    """A safe, meaningful subset of movable hardpoint coordinates.

    We deliberately expose a SMALL set of high-leverage coordinates rather than
    all 30-odd, because (a) it keeps the search honest and fast, and (b) these
    are the ones with the strongest, best-understood effect on the kinematic
    objectives. Each mutates the real Hardpoints the solver consumes. Bounds are
    generous but physically sane (mm).
    """
    def set_z(attr):
        return lambda hp, v: setattr(hp, attr, _with(getattr(hp, attr), 2, v))

    def set_y(attr):
        return lambda hp, v: setattr(hp, attr, _with(getattr(hp, attr), 1, v))

    return [
        ContinuousVar("uo_z", 270.0, 320.0, set_z("upper_outer"),
                      "Upper ball-joint height"),
        ContinuousVar("lo_z", 95.0, 130.0, set_z("lower_outer"),
                      "Lower ball-joint height"),
        ContinuousVar("ufi_z", 255.0, 305.0, set_z("upper_front_inner"),
                      "Upper front pickup height"),
        ContinuousVar("tro_z", 130.0, 175.0, set_z("tie_rod_outer"),
                      "Tie-rod outer height"),
        ContinuousVar("uo_y", 500.0, 560.0, set_y("upper_outer"),
                      "Upper ball-joint lateral"),
    ]


def _with(vec, idx, val):
    """Return a copy of vec with component idx set to val (keeps arrays pure)."""
    out = np.array(vec, float)
    out[idx] = val
    return out


# ===================================================================== #
#  2.  THE ECONOMIC LAYER  (PARAMETRIC — calibrate before quoting)
# ===================================================================== #
@dataclass
class MassModel:
    """First-order corner/architecture mass deltas, kg. PARAMETRIC — edit these.

    Every number is a transparent, editable estimate relative to a baseline
    (13-inch, single-motor, 400V, inboard). They are plausible FSAE ballparks,
    NOT weighed parts. Swap in your own BOM numbers to make the front real.
    """
    base_kg: float = 210.0                       # baseline full-car mass
    wheel_kg: dict = field(default_factory=lambda: {10: -3.2, 13: 0.0})
    motor_kg: dict = field(default_factory=lambda: {1: 0.0, 2: 6.5, 4: 14.0})
    pack_kg: dict = field(default_factory=lambda: {400: 0.0, 600: 4.5})
    damper_kg: dict = field(default_factory=lambda: {"inboard": 2.0, "outboard": 0.0})

    def mass(self, arch: dict) -> float:
        return (self.base_kg
                + self.wheel_kg.get(arch["wheel_in"], 0.0)
                + self.motor_kg.get(arch["motors"], 0.0)
                + self.pack_kg.get(arch["pack_v"], 0.0)
                + self.damper_kg.get(arch["damper"], 0.0))


@dataclass
class PointsModel:
    """Map engineering metrics -> estimated FSAE dynamic-event points. PARAMETRIC.

    This is the layer a design judge will (rightly) probe hardest, so it is kept
    deliberately simple and fully exposed. Points here are a MONOTONE PROXY for
    dynamic performance, not an official scoring formula. Treat differences
    between candidates as directional, and calibrate ``per_kg`` etc. against your
    own lap-sim before putting a number on a slide.

    Terms:
      * lighter car  -> more points          (per_kg, points/kg)
      * more drive motors -> more usable traction, with diminishing return
      * 600V -> lower I^2R loss, small efficiency/endurance credit
      * smaller wheel -> lower unsprung + CG, small credit, but less contact
        patch (a penalty), net per the two coefficients below
      * kinematic quality (camber gain, bump steer) -> grip consistency credit,
        fed straight from the real solver
    """
    base_points: float = 500.0
    per_kg: float = -0.55                       # each kg over baseline costs pts
    baseline_kg: float = 210.0
    motor_pts: dict = field(default_factory=lambda: {1: 0.0, 2: 8.0, 4: 12.0})
    pack_pts: dict = field(default_factory=lambda: {400: 0.0, 600: 3.5})
    wheel_cg_pts: dict = field(default_factory=lambda: {10: 5.0, 13: 0.0})
    wheel_patch_pts: dict = field(default_factory=lambda: {10: -3.2, 13: 0.0})
    # kinematic-quality credits (physics-fed inputs, parametric weights)
    camber_gain_target_deg: float = 1.0         # |deg| camber per 25mm bump wanted
    camber_pts_per_deg_err: float = -6.0        # miss the target -> lose pts
    bumpsteer_pts_per_deg: float = -9.0         # total toe swing over travel

    def points(self, arch: dict, mass_kg: float, kin: dict) -> float:
        p = self.base_points
        p += self.per_kg * (mass_kg - self.baseline_kg)
        p += self.motor_pts.get(arch["motors"], 0.0)
        p += self.pack_pts.get(arch["pack_v"], 0.0)
        p += self.wheel_cg_pts.get(arch["wheel_in"], 0.0)
        p += self.wheel_patch_pts.get(arch["wheel_in"], 0.0)
        # kinematic quality (from the REAL solver)
        p += self.camber_pts_per_deg_err * abs(
            kin["camber_gain_deg"] - self.camber_gain_target_deg)
        p += self.bumpsteer_pts_per_deg * kin["bumpsteer_deg"]
        return p


# ===================================================================== #
#  3.  KINEMATIC EVALUATION  (PHYSICS — the real solver)
# ===================================================================== #
def evaluate_kinematics(hp: Hardpoints, travel_mm: float = 25.0,
                        n: int = 9) -> dict:
    """Run the REAL corner solver and extract the tunable kinematic metrics.

    Returns physics-grounded numbers (or a large-penalty sentinel dict if the
    geometry is degenerate and the solver cannot converge — a failed build is a
    bad candidate, not a crash).
    """
    try:
        kin = SuspensionKinematics(hp)
        states = kin.sweep(travel_min=-travel_mm, travel_max=travel_mm, n=n)
        cambers = np.array([s.camber for s in states])
        toes = np.array([s.toe for s in states])
        travels = np.array([s.travel for s in states])
        if not all(getattr(s, "converged", True) for s in states):
            raise RuntimeError("non-converged sweep")
        # camber gain: change in camber per 25 mm bump (positive = gains negative
        # camber into bump, the usual want). Report magnitude of the slope*25.
        slope = np.polyfit(travels, cambers, 1)[0]     # deg/mm
        camber_gain = abs(slope) * 25.0
        # bump steer: peak-to-peak toe swing across the travel range (deg)
        bumpsteer = float(np.max(toes) - np.min(toes))
        # scrub radius at static
        static = states[len(states) // 2]
        return {
            "ok": True,
            "camber_gain_deg": float(camber_gain),
            "bumpsteer_deg": float(bumpsteer),
            "scrub_mm": float(abs(static.scrub_radius)),
            "static_camber_deg": float(static.camber),
        }
    except Exception as e:                              # degenerate geometry
        return {
            "ok": False, "err": str(e),
            "camber_gain_deg": 99.0, "bumpsteer_deg": 99.0,
            "scrub_mm": 999.0, "static_camber_deg": 0.0,
        }


# ===================================================================== #
#  4.  THE MIXED-VARIABLE PROBLEM  (glue: genome -> objectives)
# ===================================================================== #
@dataclass
class Candidate:
    arch: dict                       # discrete choices, resolved
    cont: dict                       # continuous var name -> value
    hp: Hardpoints                   # the geometry actually evaluated
    kin: dict                        # physics metrics
    mass_kg: float                   # parametric
    points: float                    # parametric (to MAXIMISE)
    objectives: np.ndarray           # what NSGA-II minimises
    feasible: bool

    def summary(self) -> dict:
        return {
            **{k: self.arch[k] for k in self.arch},
            "mass_kg": round(self.mass_kg, 2),
            "points": round(self.points, 1),
            "camber_gain_deg": round(self.kin["camber_gain_deg"], 3),
            "bumpsteer_deg": round(self.kin["bumpsteer_deg"], 3),
            "scrub_mm": round(self.kin["scrub_mm"], 1),
            "feasible": self.feasible,
        }


class ArchitectureProblem:
    """Binds the architecture space + economic models + real solver into an
    objective vector for the optimizer.

    Objectives (ALL minimised by NSGA-II; we negate points):
        f0 = -points          (maximise estimated points)
        f1 =  mass_kg         (minimise mass)
        f2 =  bumpsteer_deg   (minimise, physics — a pure quality axis)

    Keeping mass AND points both as axes (rather than folding mass into points
    only) lets a lead see the genuine trade surface: the lightest car and the
    highest-points car need not be the same architecture.
    """

    def __init__(self, discrete=None, continuous=None,
                 base_hp: Hardpoints | None = None,
                 points_model: PointsModel | None = None,
                 mass_model: MassModel | None = None,
                 travel_mm: float = 25.0):
        self.discrete = discrete if discrete is not None else default_discrete_space()
        self.continuous = continuous if continuous is not None else default_continuous_space()
        self.base_hp = base_hp if base_hp is not None else Hardpoints.default()
        self.points_model = points_model or PointsModel()
        self.mass_model = mass_model or MassModel()
        self.travel_mm = travel_mm
        self.n_disc = len(self.discrete)
        self.n_cont = len(self.continuous)

    # --- genome encoding: [disc indices... , cont values...] -----------
    def random_genome(self, rng: np.random.Generator) -> np.ndarray:
        disc = [rng.integers(0, len(d.options)) for d in self.discrete]
        cont = [rng.uniform(c.lo, c.hi) for c in self.continuous]
        return np.array(disc + cont, float)

    def decode(self, g: np.ndarray) -> tuple[dict, dict, Hardpoints]:
        arch = {}
        for i, d in enumerate(self.discrete):
            idx = int(round(g[i])) % len(d.options)
            arch[d.name] = d.options[idx]
        cont = {}
        hp = self.base_hp.copy()
        for j, c in enumerate(self.continuous):
            val = float(np.clip(g[self.n_disc + j], c.lo, c.hi))
            cont[c.name] = val
            c.apply(hp, val)
        return arch, cont, hp

    def evaluate(self, g: np.ndarray) -> Candidate:
        arch, cont, hp = self.decode(g)
        kin = evaluate_kinematics(hp, travel_mm=self.travel_mm)
        mass = self.mass_model.mass(arch)
        pts = self.points_model.points(arch, mass, kin)
        feasible = bool(kin["ok"]) and kin["scrub_mm"] < 60.0
        # infeasible geometries are pushed to the back of the front, not crashed
        pen = 0.0 if feasible else 1e4
        obj = np.array([
            -pts + pen,                 # f0: maximise points
            mass + pen,                 # f1: minimise mass
            kin["bumpsteer_deg"] + pen  # f2: minimise bump steer (physics)
        ])
        return Candidate(arch, cont, hp, kin, mass, pts, obj, feasible)


# ===================================================================== #
#  5.  NSGA-II  (dependency-free: non-dominated sort + crowding + SBX/PM)
# ===================================================================== #
def _dominates(a: np.ndarray, b: np.ndarray) -> bool:
    """True iff a Pareto-dominates b (all <= and at least one <)."""
    return bool(np.all(a <= b) and np.any(a < b))


def fast_non_dominated_sort(objs: np.ndarray) -> list[list[int]]:
    """Classic Deb 2002 non-dominated sort. Returns fronts as index lists."""
    n = len(objs)
    S = [[] for _ in range(n)]
    ndom = np.zeros(n, int)
    fronts = [[]]
    for p in range(n):
        for q in range(n):
            if p == q:
                continue
            if _dominates(objs[p], objs[q]):
                S[p].append(q)
            elif _dominates(objs[q], objs[p]):
                ndom[p] += 1
        if ndom[p] == 0:
            fronts[0].append(p)
    i = 0
    while fronts[i]:
        nxt = []
        for p in fronts[i]:
            for q in S[p]:
                ndom[q] -= 1
                if ndom[q] == 0:
                    nxt.append(q)
        i += 1
        fronts.append(nxt)
    return fronts[:-1]


def crowding_distance(objs: np.ndarray, front: list[int]) -> np.ndarray:
    """Deb crowding distance within one front (larger = more isolated = keep)."""
    m = objs.shape[1]
    dist = np.zeros(len(front))
    pts = objs[front]
    for k in range(m):
        order = np.argsort(pts[:, k])
        dist[order[0]] = dist[order[-1]] = np.inf
        lo, hi = pts[order[0], k], pts[order[-1], k]
        span = hi - lo if hi > lo else 1.0
        for r in range(1, len(front) - 1):
            dist[order[r]] += (pts[order[r + 1], k] - pts[order[r - 1], k]) / span
    return dist


def _tournament(ranks, crowd, rng):
    a, b = rng.integers(0, len(ranks), 2)
    if ranks[a] < ranks[b]:
        return a
    if ranks[b] < ranks[a]:
        return b
    return a if crowd[a] > crowd[b] else b


def _crossover_mutate(p1, p2, prob, rng, eta=15.0, pm=None):
    """SBX crossover + polynomial mutation on the whole genome. Discrete slots
    (leading n_disc entries) are handled by the caller's rounding at decode; here
    they're treated as reals and re-quantised on decode, which is a standard and
    effective way to keep one operator for a mixed genome."""
    n = len(p1)
    pm = pm if pm is not None else 1.0 / n
    c1, c2 = p1.copy(), p2.copy()
    # SBX
    for i in range(n):
        if rng.random() <= 0.5 and abs(p1[i] - p2[i]) > 1e-12:
            u = rng.random()
            beta = (2 * u) ** (1 / (eta + 1)) if u <= 0.5 else \
                   (1 / (2 * (1 - u))) ** (1 / (eta + 1))
            c1[i] = 0.5 * ((1 + beta) * p1[i] + (1 - beta) * p2[i])
            c2[i] = 0.5 * ((1 - beta) * p1[i] + (1 + beta) * p2[i])
    # polynomial mutation
    for c in (c1, c2):
        for i in range(n):
            if rng.random() < pm:
                u = rng.random()
                delta = (2 * u) ** (1 / (eta + 1)) - 1 if u < 0.5 else \
                        1 - (2 * (1 - u)) ** (1 / (eta + 1))
                c[i] += delta * max(abs(c[i]), 1.0) * 0.1
    return c1, c2


@dataclass
class SynthResult:
    pareto: list[Candidate]          # non-dominated set (front 0)
    population: list[Candidate]      # final generation (for inspection)
    history: list                    # per-gen hypervolume-ish proxy
    problem: ArchitectureProblem
    generations: int
    pop_size: int


def synthesize(problem: ArchitectureProblem | None = None,
               pop_size: int = 40, generations: int = 25,
               seed: int = 0, progress: Callable[[int, int], None] | None = None
               ) -> SynthResult:
    """Run NSGA-II over the mixed architecture/geometry space.

    Deterministic for a given seed. Returns the Pareto front of Candidates plus
    the final population and a convergence trace. Defaults (40x25 = 1000 real
    kinematic solves) run in a few seconds and give a stable front for the
    default 4-switch / 5-continuous space.
    """
    problem = problem or ArchitectureProblem()
    rng = np.random.default_rng(seed)

    genomes = [problem.random_genome(rng) for _ in range(pop_size)]
    cands = [problem.evaluate(g) for g in genomes]
    objs = np.array([c.objectives for c in cands])
    history = []

    for gen in range(generations):
        # rank current pop
        fronts = fast_non_dominated_sort(objs)
        ranks = np.zeros(len(cands), int)
        crowd = np.zeros(len(cands))
        for r, fr in enumerate(fronts):
            cd = crowding_distance(objs, fr)
            for k, idx in enumerate(fr):
                ranks[idx] = r
                crowd[idx] = cd[k]
        # make offspring
        children_g = []
        while len(children_g) < pop_size:
            i = _tournament(ranks, crowd, rng)
            j = _tournament(ranks, crowd, rng)
            c1, c2 = _crossover_mutate(genomes[i], genomes[j], 0.9, rng)
            children_g += [c1, c2]
        children_g = children_g[:pop_size]
        child_c = [problem.evaluate(g) for g in children_g]
        # merge parents+children, elitist truncation
        all_g = genomes + children_g
        all_c = cands + child_c
        all_o = np.array([c.objectives for c in all_c])
        fronts = fast_non_dominated_sort(all_o)
        new_idx = []
        for fr in fronts:
            if len(new_idx) + len(fr) <= pop_size:
                new_idx += fr
            else:
                cd = crowding_distance(all_o, fr)
                room = pop_size - len(new_idx)
                new_idx += [fr[k] for k in np.argsort(-cd)[:room]]
                break
        genomes = [all_g[i] for i in new_idx]
        cands = [all_c[i] for i in new_idx]
        objs = np.array([c.objectives for c in cands])
        # convergence proxy: best (min) of each objective among feasible
        feas = [c for c in cands if c.feasible]
        if feas:
            fo = np.array([c.objectives for c in feas])
            history.append({"gen": gen,
                            "best_points": float(-fo[:, 0].min()),
                            "min_mass": float(fo[:, 1].min()),
                            "min_bumpsteer": float(fo[:, 2].min())})
        if progress:
            progress(gen + 1, generations)

    # final Pareto front (feasible only, front 0)
    final_o = np.array([c.objectives for c in cands])
    fronts = fast_non_dominated_sort(final_o)
    pareto = [cands[i] for i in fronts[0] if cands[i].feasible]
    # sort the front by points descending for readability
    pareto.sort(key=lambda c: -c.points)
    return SynthResult(pareto, cands, history, problem, generations, pop_size)


# ===================================================================== #
#  6.  TRADE-OFF REPORTING  (the "why" a design judge asks for)
# ===================================================================== #
def tradeoff_table(result: SynthResult) -> list[dict]:
    """Flatten the Pareto front into rows for a table/plot."""
    return [c.summary() for c in result.pareto]


def compare_architectures(result: SynthResult) -> list[dict]:
    """For each distinct DISCRETE architecture on the Pareto front, report its
    best continuous realisation. This is the "10-inch+400V vs 13-inch+600V"
    answer, each row being a genuinely non-dominated architecture with its best
    geometry — the honest version of the brief's trade-off matrix.
    """
    best: dict = {}
    for c in result.pareto:
        key = tuple(sorted(c.arch.items()))
        if key not in best or c.points > best[key].points:
            best[key] = c
    rows = [c.summary() for c in best.values()]
    rows.sort(key=lambda r: -r["points"])
    return rows


PROVENANCE = {
    "physics_grounded": [
        "camber_gain_deg", "bumpsteer_deg", "scrub_mm", "static_camber_deg",
    ],
    "parametric_estimate": [
        "mass_kg (MassModel coefficients)",
        "points (PointsModel coefficients)",
    ],
    "note": (
        "Kinematic axes come from the real SuspensionKinematics solver and are "
        "trustworthy. Mass and points come from editable coefficient tables and "
        "are first-order estimates — calibrate MassModel/PointsModel to your own "
        "BOM and lap-sim before quoting a points delta to a judge."
    ),
}
