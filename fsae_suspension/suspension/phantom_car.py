# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  Module: phantom_car — the margin audit (conservatism as a ledger)
# ============================================================================
"""
Phantom Car — the car your margins actually designed.

THE PROBLEM THIS SOLVES
-----------------------
The Proof Engine measures how uncertain the deck is. The Saboteur catches a
deck that is corrupted. Neither touches the third failure mode, and it is the
one that makes formula cars fat and DNFs endurance anyway:

    EVERY SUBSYSTEM HEDGES THE SAME UNCERTAINTY SEPARATELY, IN SECRET,
    AND NOBODY ADDS IT UP.

The brakes lead sizes for the car "if it comes in heavy" — quietly designing
to 250 kg. The structures lead takes the declared mount load, adds "a bit for
safety", then applies FoS 1.5 on top of a load case that was already
worst-case: margin stacked on margin, on a number whose evidence grade was
GUESS to begin with — so the bracket is defending a car that is a 4σ
statistical event, and the mass bill for defending that impossible car lands
on the real one. Meanwhile the energy budget — the one number that actually
DNFs you at endurance — consumes the SAME mass at its optimistic target
value, naked, because "we'll get the weight down". The input deck now
describes at least two mutually exclusive cars, everyone believes they were
prudent, and the total conservatism of the design is a number nobody has ever
computed, because it lives smeared across eight private spreadsheets.

Aerospace primes manage this with dedicated margin-management processes and a
staff to run them. No CAE, PLM, or requirements tool computes it, and nothing
a student team can afford even names it. This module does both, and it costs
zero new physics: the σ that prices every hedge is the SAME σ the Proof
Engine already derives from evidence grades and staleness. One ledger, third
consumer.

WHAT IT DOES
------------
  1. MARGIN DECLARATIONS. Each consumer of a deck number states, in the open,
     the design value it actually uses (which its spreadsheet already
     contains — this is disclosure, not new work) and any design factor
     applied on top. The declaration is priced in the only honest currency
     available: HOW MANY SIGMA of the quantity's own evidence-graded
     uncertainty the hedge buys. "Assumes 250 kg" is opinion; "hedged +2.1σ
     on an ESTIMATE-grade mass" is arithmetic.

  2. THE MARGIN CHARTER, SEALED. The team declares ONE design percentile for
     the whole car (e.g. "we design to the 95th-percentile car") and seals it
     sha256, like a validation contract. Every declaration is then judged
     against the charter:
         ALIGNED       covered within tolerance of the charter — the intent
         STACKED       covered far beyond it — margin paid twice; the excess
                       is reported as RELEASABLE, in the quantity's own units
         UNDER-COVERED some hedge, but short of the charter
         NAKED         consumed at nominal while the charter demands cover —
                       and the verdict names the evidence grade it is naked on
         ANTI-HEDGED   consumed at a value BETTER than the deck claims — the
                       consumer is designing to a car the ledger says does
                       not exist yet
     Editing a sealed charter breaks the seal, and a broken seal refuses to
     judge, out loud. "We'll just design to the target weight" can never be
     decided quietly after the hedges were priced.

  3. THE TWO-CARS DETECTOR. For every quantity, the spread of assumed design
     values across its consumers is measured in σ. Beyond 1σ of spread the
     deck provably describes MORE THAN ONE CAR — brakes stopping a 250 kg
     car while the energy budget feeds a 228 kg one — and the audit names
     both consumers and the width of their disagreement. This is the exact
     contradiction the Integration ledger was built to kill for VALUES,
     applied for the first time to ASSUMPTIONS.

  4. β — THE RELIABILITY INDEX OF EACH LOAD CASE. A consumer that stacks
     worst-case assumptions on several inputs at once is designing to their
     JOINT worst case. The distance to that design point in sigma space,
     β = sqrt(Σ z_i²), is the same first-order reliability index (FORM) that
     professional reliability engineering uses — computed here from the
     σ your evidence grades already imply, with the odds stated in English:
     "this bracket load case is a 1-in-2,300,000 car."

  5. THE PHANTOM, THE COHERENT CAR, AND THE GAP. For each objective (lap
     time, endurance energy, thermal margin, mass) the audit evaluates three
     cars: the NOMINAL car (the deck as declared), the COHERENT car (every
     touched channel at the charter percentile, hedged once, in the same
     direction), and the PHANTOM (every channel at the most adverse value
     any consumer assumed — the union of everyone's private fears). The gap
     between phantom and coherent is the envelope currently spent defending
     cars the deck itself says are statistically impossible. It is reported
     as ENVELOPE, honestly — not as promised savings — because releasing it
     is a design decision; pricing it is this module's job.

WHY NOBODY HAS BUILT THIS
-------------------------
Margin stacking is invisible to every tool in the chain by construction:
solvers see one load case at a time and cannot know it was already hedged
upstream; PLM sees files; requirements tools see targets, not the private
conservatism between a declared number and the value a consumer's
spreadsheet actually uses. The information needed to add margins up — who
consumes what, at what assumed value, against what uncertainty — has never
lived in one place before. In KinematiK it already does: the Integration
ledger knows the numbers, the Proof Engine knows their σ, and the coupling
graph knows who consumes them. This module is the missing join, and it is
only buildable HERE.

HONESTY CONTRACT (same as the rest of KinematiK)
------------------------------------------------
Every number is deterministic and reproducible with a calculator: hedges are
priced by (assumed − value) / σ with σ from the Proof Engine's evidence
grades; design factors are converted to the same currency by the documented
linearization (f − 1)·|value| / σ; β is a plain root-sum-square; the normal
tail comes from math.erf; the charter percentile inverts Φ by bisection.
Joint probabilities assume independence between quantities and say so — the
odds are a first-order statement, exactly as FORM's are. The phantom/coherent
gap is envelope, never promised mass savings, and the docket says so. The
audit judges only DECLARED consumptions: a hedge nobody disclosed is a blind
spot, listed out loud as an unaudited consumer, never absorbed into a green
board.

No streamlit / pandas / plotly imports. Pure stdlib. Unit-testable headless.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
from dataclasses import dataclass, field, asdict
from typing import Optional

from .proof_engine import (
    Objective, Quantity, DEFAULT_OBJECTIVES, aggregate,
    _CHANNEL_META, _SUM_CHANNELS,
)


# --------------------------------------------------------------------------- #
#  Documented constants — every threshold named, none folklore
# --------------------------------------------------------------------------- #
# A declaration is ALIGNED when its total cover is within this many sigma of
# the charter's target. Half a sigma: tight enough that a stacked FoS shows,
# loose enough that rounding a design value never triggers a verdict.
ALIGN_TOL_SIGMA = 0.5
# Below this much cover a declaration is NAKED rather than merely UNDER —
# a quarter sigma is indistinguishable from consuming the nominal value.
NAKED_SIGMA = 0.25
# An assumed value this far on the FAVORABLE side is an anti-hedge — the
# consumer is designing to a better car than the deck declares.
ANTI_SIGMA = 0.25
# Two consumers whose assumed design values differ by more than this many
# sigma of the quantity's own band are provably designing different cars.
TWO_CARS_SIGMA = 1.0
# Odds are capped here: beyond ~6σ the independence assumption carries more
# error than the tail, and "essentially impossible" is the honest phrasing.
_MAX_BETA_FOR_ODDS = 6.0

_HIGH, _LOW = "high", "low"


# --------------------------------------------------------------------------- #
#  Normal-distribution helpers — stdlib, deterministic, hand-checkable
# --------------------------------------------------------------------------- #
def phi(z: float) -> float:
    """Standard normal CDF via math.erf — exact to double precision."""
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def z_from_percentile(p: float) -> float:
    """
    Invert Φ by bisection on [0, 8] — deterministic, no rational-approximation
    magic numbers to trust. p in percent, (50, 100). 95 → 1.6449, 97.72 → 2.0.
    """
    if not (50.0 < p < 100.0):
        raise ValueError("Design percentile must be in (50, 100) — designing "
                         "to the median car (or worse) is not a margin "
                         "policy, it is a coin flip.")
    lo, hi, target = 0.0, 8.0, p / 100.0
    for _ in range(60):                       # 8 / 2^60 ≪ double epsilon
        mid = 0.5 * (lo + hi)
        if phi(mid) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# --------------------------------------------------------------------------- #
#  Car-level quantities — the aggregates consumers actually design against
# --------------------------------------------------------------------------- #
@dataclass
class CarQuantity:
    """
    One car-level channel with a value AND a σ. σ is computed by the same
    one-at-a-time perturbation the Proof Engine uses for objectives: each
    contributing subsystem Quantity is pushed ± its own band, the aggregate
    re-evaluated, contributions combined root-sum-square. Deterministic;
    for a plain sum channel it reduces to the RSS of the contributors' bands,
    which you can check by hand.
    """
    key: str            # "car.mass_kg"
    channel: str        # "mass_kg"
    label: str
    value: float
    unit: str
    sigma: float
    worst_grade: str    # the worst evidence grade among contributors


def car_quantities(quantities: list[Quantity],
                   today: Optional[_dt.date] = None) -> dict[str, CarQuantity]:
    """Build every car-level channel present in the deck, with priced σ."""
    base = aggregate(quantities)
    out: dict[str, CarQuantity] = {}
    grade_rank = {}
    for q in quantities:
        grade_rank.setdefault(q.channel, []).append(q)
    for ch, val in base.items():
        contributors = grade_rank.get(ch, [])
        var = 0.0
        for q in [x for x in quantities if x.channel == ch]:
            u = q.abs_unc(today)
            if u == 0.0:
                continue
            hi = [Quantity.from_dict(x.as_dict()) for x in quantities]
            lo = [Quantity.from_dict(x.as_dict()) for x in quantities]
            # locate q within the copies by key (stable, unique)
            for arr, sign in ((hi, +1.0), (lo, -1.0)):
                for x in arr:
                    if x.key == q.key:
                        x.value = q.value + sign * u
            d = abs(aggregate(hi).get(ch, val) - aggregate(lo).get(ch, val)) / 2.0
            var += d * d
        lab, unit = _CHANNEL_META.get(ch, (ch.replace("_", " "), ""))
        worst = "estimate"
        if contributors:
            worst = max(contributors, key=lambda q: q.rel_unc(today)).grade.value
        out[f"car.{ch}"] = CarQuantity(
            key=f"car.{ch}", channel=ch, label=f"car {lab}",
            value=float(val), unit=unit, sigma=math.sqrt(var),
            worst_grade=worst)
    return out


def resolve_quantity(key: str, quantities: list[Quantity],
                     cars: dict[str, CarQuantity],
                     today: Optional[_dt.date] = None
                     ) -> Optional[CarQuantity]:
    """
    A declaration may target a car-level aggregate ("car.mass_kg") or one
    subsystem's channel ("accumulator.mass_kg"). Both come back in the same
    shape; a key that resolves to nothing returns None and the audit reports
    it as unresolved instead of silently dropping it.
    """
    if key in cars:
        return cars[key]
    for q in quantities:
        if q.key == key:
            return CarQuantity(key=q.key, channel=q.channel, label=q.label,
                               value=q.value, unit=q.unit,
                               sigma=q.abs_unc(today),
                               worst_grade=q.grade.value)
    return None


# --------------------------------------------------------------------------- #
#  The declarations — disclosure of what each consumer actually assumes
# --------------------------------------------------------------------------- #
@dataclass
class MarginDeclaration:
    """
    One consumer's stated consumption of one deck quantity. `assumed_value`
    is the number the consumer's own sizing actually uses — the disclosure
    this whole module exists to obtain. `design_factor` is any multiplicative
    factor applied ON TOP (an FoS, a "×1.2 to be safe"). `adverse` states
    which direction of the quantity hurts this consumer, so a hedge and an
    anti-hedge can be told apart.
    """
    consumer: str            # "brake sizing", "energy budget", ...
    quantity_key: str        # "car.mass_kg" or "accumulator.mass_kg"
    adverse: str             # "high" | "low"
    assumed_value: float
    design_factor: float = 1.0
    rationale: str = ""

    def as_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "MarginDeclaration":
        valid = MarginDeclaration.__dataclass_fields__.keys()
        return MarginDeclaration(**{k: v for k, v in d.items() if k in valid})


# The built-in consumption map: who, in a normal FSAE-EV design flow, sizes
# against which car-level number, and which direction hurts them. This seeds
# the disclosure form — it invents no assumed values (seeds are nominal, so
# a fresh audit shows honest NAKED verdicts, not fabricated prudence).
DEFAULT_CONSUMPTION: list[tuple[str, str, str, str]] = [
    ("brake sizing", "car.mass_kg", _HIGH,
     "pad energy and lock-up margin scale with the mass the brakes must stop"),
    ("brake sizing", "car.cg_z_mm", _HIGH,
     "forward transfer under braking ∝ CG height → front-line demand"),
    ("structures / mounts", "car.mount_load_n", _HIGH,
     "the peak load the bracket must survive"),
    ("energy budget", "car.mass_kg", _HIGH,
     "endurance kWh grows with mass — the number that DNFs you"),
    ("energy budget", "car.power_draw_w", _HIGH,
     "LV parasitic drain integrates over the full endurance"),
    ("pack cooling sizing", "car.heat_reject_w", _HIGH,
     "the heat the radiator must dump"),
    ("pack cooling sizing", "car.cooling_airflow_cms", _LOW,
     "the airflow the core can count on at low speed"),
    ("tractive fusing", "car.peak_current_a", _HIGH,
     "fuse and conductor sizing against the peak the pack can deliver"),
    ("lap-time target", "car.mass_kg", _HIGH,
     "the mass the season's lap-time promise is made at"),
]


def seed_declarations(quantities: list[Quantity],
                      today: Optional[_dt.date] = None
                      ) -> list[MarginDeclaration]:
    """
    Seed the disclosure form from the consumption map — assumed values start
    at NOMINAL on purpose. Fabricating hedges the team never made would be
    the unearned green board; a fresh audit should say NAKED wherever the
    charter demands cover and nobody has disclosed any.
    """
    cars = car_quantities(quantities, today)
    out = []
    for consumer, key, adverse, why in DEFAULT_CONSUMPTION:
        cq = cars.get(key)
        if cq is None:
            continue                      # channel not declared in the deck
        out.append(MarginDeclaration(consumer=consumer, quantity_key=key,
                                     adverse=adverse,
                                     assumed_value=cq.value,
                                     design_factor=1.0, rationale=why))
    return out


# --------------------------------------------------------------------------- #
#  The Margin Charter — one percentile for the whole car, sealed
# --------------------------------------------------------------------------- #
@dataclass
class MarginCharter:
    """
    The team's single answer to "how bad a car do we design for?". Sealed at
    creation like a validation contract; the audit refuses to judge against
    a charter whose sealed fields moved. `fos_rule` records where a design
    factor is allowed to live — the default "once" means a chain may carry
    EITHER a sigma-hedge to the charter percentile OR an explicit factor,
    never both silently stacked; the verdicts enforce the arithmetic of that
    sentence and this field documents the intent for next year's cohort.
    """
    percentile: float
    fos_rule: str = "once"
    note: str = ""
    author: str = ""
    created_on: str = ""
    seal: str = ""

    _SEALED = ("percentile", "fos_rule", "note", "author", "created_on")

    def _payload(self) -> str:
        d = asdict(self)
        return json.dumps({k: d[k] for k in self._SEALED},
                          sort_keys=True, separators=(",", ":"))

    def compute_seal(self) -> str:
        return hashlib.sha256(self._payload().encode()).hexdigest()

    def verify_seal(self) -> bool:
        return bool(self.seal) and self.seal == self.compute_seal()

    @property
    def z(self) -> float:
        return z_from_percentile(self.percentile)

    def as_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "MarginCharter":
        valid = MarginCharter.__dataclass_fields__.keys()
        return MarginCharter(**{k: v for k, v in d.items() if k in valid})


def create_charter(percentile: float, note: str = "", author: str = "",
                   fos_rule: str = "once",
                   today: Optional[_dt.date] = None) -> MarginCharter:
    """Seal the charter BEFORE the audit — the percentile provably never
    moved to flatter a verdict."""
    c = MarginCharter(percentile=float(percentile), fos_rule=fos_rule,
                      note=note, author=author,
                      created_on=(today or _dt.date.today()).isoformat())
    c.seal = c.compute_seal()
    return c


# --------------------------------------------------------------------------- #
#  The audit — pricing every hedge in the deck's own sigma currency
# --------------------------------------------------------------------------- #
@dataclass
class DeclarationFinding:
    """One declaration, priced and judged against the charter."""
    consumer: str
    quantity_key: str
    quantity_label: str
    unit: str
    nominal: float
    sigma: float
    worst_grade: str
    adverse: str
    assumed_value: float
    design_factor: float
    z_assumed: float          # (assumed − nominal)/σ, signed toward adverse
    z_factor: float           # (f − 1)·|nominal|/σ — the factor, in σ currency
    z_total: float
    coverage_pct: float       # Φ(z_total)·100 — the percentile actually bought
    verdict: str              # ALIGNED | STACKED | UNDER-COVERED | NAKED | ANTI-HEDGED
    releasable: float         # units of the quantity, when STACKED
    exposure: float           # units of the quantity, when short of the charter
    rationale: str = ""


@dataclass
class QuantityCoverage:
    """All consumers of one quantity, and whether they agree on the car."""
    quantity_key: str
    label: str
    unit: str
    nominal: float
    sigma: float
    worst_grade: str
    consumers: list = field(default_factory=list)   # DeclarationFinding refs
    spread_sigma: float = 0.0
    contradictory: bool = False
    lo_consumer: str = ""
    hi_consumer: str = ""
    lo_assumed: float = 0.0
    hi_assumed: float = 0.0


@dataclass
class ConsumerPhantom:
    """The joint improbability of one consumer's stacked assumptions."""
    consumer: str
    beta: float               # FORM reliability index sqrt(Σ max(z_assumed,0)²)
    n_hedged: int
    odds_one_in: float        # 1 / (1 − Φ(β)); 0.0 means beyond the odds cap
    inputs: list = field(default_factory=list)      # (label, z_assumed)


@dataclass
class ObjectiveGap:
    """
    Nominal vs coherent-percentile vs phantom, per objective. `gap` is
    normalized so the sign means one thing regardless of the objective's
    direction: POSITIVE = the phantom over-defends beyond the charter
    (envelope spent on statistically impossible cars); NEGATIVE = the union
    of everyone's declared fears still falls SHORT of the charter car — the
    collective hedging under-defends the percentile the team swore to.
    """
    objective_key: str
    label: str
    unit: str
    better: str
    nominal: float
    coherent: float
    phantom: float
    gap: float                # +over-defence / −under-defence vs the charter


@dataclass
class MarginAudit:
    """The full docket. `refused=True` means the charter seal was broken and
    NOTHING below it was computed — a broken seal judges nothing."""
    charter: dict
    charter_z: float
    refused: bool
    refusal_reason: str
    generated_on: str
    findings: list = field(default_factory=list)
    quantity_coverage: list = field(default_factory=list)
    consumer_phantoms: list = field(default_factory=list)
    objective_gaps: list = field(default_factory=list)
    unresolved: list = field(default_factory=list)    # keys that matched nothing
    unaudited_consumers: list = field(default_factory=list)


def _judge_declaration(dec: MarginDeclaration, cq: CarQuantity,
                       z_star: float) -> DeclarationFinding:
    d = +1.0 if dec.adverse == _HIGH else -1.0
    sigma = cq.sigma if cq.sigma > 0 else 1e-12
    z_assumed = d * (dec.assumed_value - cq.value) / sigma
    z_factor = (dec.design_factor - 1.0) * abs(cq.value or 1.0) / sigma
    z_total = z_assumed + z_factor
    # Verdict order is fixed and documented: an anti-hedge is called out even
    # if a big FoS technically drags z_total back over the line — designing
    # to a flattering number and papering it with a factor is two mistakes.
    if z_assumed < -ANTI_SIGMA:
        verdict = "ANTI-HEDGED"
    elif z_total < NAKED_SIGMA:
        verdict = "NAKED"
    elif z_total > z_star + ALIGN_TOL_SIGMA:
        verdict = "STACKED"
    elif z_total < z_star - ALIGN_TOL_SIGMA:
        verdict = "UNDER-COVERED"
    else:
        verdict = "ALIGNED"
    releasable = max(z_total - z_star, 0.0) * sigma if verdict == "STACKED" else 0.0
    exposure = (max(z_star - z_total, 0.0) * sigma
                if verdict in ("NAKED", "UNDER-COVERED", "ANTI-HEDGED") else 0.0)
    return DeclarationFinding(
        consumer=dec.consumer, quantity_key=cq.key, quantity_label=cq.label,
        unit=cq.unit, nominal=cq.value, sigma=cq.sigma,
        worst_grade=cq.worst_grade, adverse=dec.adverse,
        assumed_value=dec.assumed_value, design_factor=dec.design_factor,
        z_assumed=z_assumed, z_factor=z_factor, z_total=z_total,
        coverage_pct=phi(z_total) * 100.0, verdict=verdict,
        releasable=releasable, exposure=exposure, rationale=dec.rationale)


def _objective_adverse_dir(obj: Objective, base_car: dict, ch: str,
                           sigma: float) -> float:
    """+1 if pushing the channel UP worsens the objective, else −1.
    Determined by a probe perturbation, never by folklore."""
    hi = dict(base_car)
    hi[ch] = base_car.get(ch, 0.0) + (sigma if sigma > 0 else 1.0)
    f0, f1 = obj.fn(base_car), obj.fn(hi)
    worse_up = (f1 > f0) if obj.better == "lower" else (f1 < f0)
    if f1 == f0:
        return 0.0
    return +1.0 if worse_up else -1.0


def _channel_of(key: str) -> str:
    return key.split(".", 1)[1] if "." in key else key


def _as_car_delta(dec: MarginDeclaration, cq: CarQuantity,
                  cars: dict[str, CarQuantity]) -> tuple[str, float]:
    """
    Translate a declaration into (channel, implied car-level value). A
    subsystem-level hedge on a SUM channel shifts the car total by the same
    delta; on any other channel it is taken at face value only when the key
    is already car-level (documented approximation — the docket lists the
    declaration either way, this only feeds the phantom evaluation).
    """
    ch = _channel_of(dec.quantity_key)
    car = cars.get(f"car.{ch}")
    if car is None:
        return ch, dec.assumed_value
    if dec.quantity_key.startswith("car."):
        return ch, dec.assumed_value
    if ch in _SUM_CHANNELS:
        return ch, car.value + (dec.assumed_value - cq.value)
    return ch, car.value                       # non-sum subsystem hedge: neutral


def audit(charter: MarginCharter,
          declarations: list[MarginDeclaration],
          quantities: list[Quantity],
          objectives: Optional[list[Objective]] = None,
          today: Optional[_dt.date] = None) -> MarginAudit:
    """
    The whole docket, deterministically: same charter + declarations + ledger
    in, same audit out. A charter with a broken seal refuses to judge — the
    refusal IS the result, and nothing else is computed.
    """
    stamp = (today or _dt.date.today()).isoformat()
    if not charter.verify_seal():
        return MarginAudit(charter=charter.as_dict(), charter_z=0.0,
                           refused=True,
                           refusal_reason=(
                               "The charter's sealed fields changed after "
                               "sealing (or it was never sealed). A margin "
                               "verdict against a movable percentile is "
                               "worthless — re-seal the charter first."),
                           generated_on=stamp)
    z_star = charter.z
    cars = car_quantities(quantities, today)
    findings: list[DeclarationFinding] = []
    unresolved: list[str] = []
    for dec in declarations:
        cq = resolve_quantity(dec.quantity_key, quantities, cars, today)
        if cq is None or cq.sigma <= 0.0:
            unresolved.append(dec.quantity_key)
            continue
        findings.append(_judge_declaration(dec, cq, z_star))

    # ---- per-quantity coverage & the two-cars detector -------------------- #
    by_q: dict[str, list[DeclarationFinding]] = {}
    for f in findings:
        by_q.setdefault(f.quantity_key, []).append(f)
    coverage: list[QuantityCoverage] = []
    for key in sorted(by_q):
        fs = by_q[key]
        qc = QuantityCoverage(
            quantity_key=key, label=fs[0].quantity_label, unit=fs[0].unit,
            nominal=fs[0].nominal, sigma=fs[0].sigma,
            worst_grade=fs[0].worst_grade,
            consumers=[f.consumer for f in fs])
        if len(fs) >= 2:
            lo = min(fs, key=lambda f: f.assumed_value)
            hi = max(fs, key=lambda f: f.assumed_value)
            qc.spread_sigma = (hi.assumed_value - lo.assumed_value) / qc.sigma
            qc.contradictory = qc.spread_sigma > TWO_CARS_SIGMA
            qc.lo_consumer, qc.hi_consumer = lo.consumer, hi.consumer
            qc.lo_assumed, qc.hi_assumed = lo.assumed_value, hi.assumed_value
        coverage.append(qc)

    # ---- β per consumer --------------------------------------------------- #
    by_c: dict[str, list[DeclarationFinding]] = {}
    for f in findings:
        by_c.setdefault(f.consumer, []).append(f)
    phantoms: list[ConsumerPhantom] = []
    for consumer in sorted(by_c):
        fs = by_c[consumer]
        zs = [(f.quantity_label, f.z_assumed) for f in fs]
        beta = math.sqrt(sum(max(z, 0.0) ** 2 for _, z in zs))
        n = sum(1 for _, z in zs if z > 0.0)
        if beta <= 0.0:
            odds = 1.0
        elif beta > _MAX_BETA_FOR_ODDS:
            odds = 0.0                         # "beyond the odds cap"
        else:
            tail = 1.0 - phi(beta)
            odds = (1.0 / tail) if tail > 0 else 0.0
        phantoms.append(ConsumerPhantom(consumer=consumer, beta=beta,
                                        n_hedged=n, odds_one_in=odds,
                                        inputs=zs))

    # ---- phantom vs coherent vs nominal, per objective -------------------- #
    base_car = aggregate(quantities)
    touched: dict[str, list[tuple[MarginDeclaration, CarQuantity]]] = {}
    for dec in declarations:
        cq = resolve_quantity(dec.quantity_key, quantities, cars, today)
        if cq is None:
            continue
        touched.setdefault(_channel_of(dec.quantity_key), []).append((dec, cq))
    gaps: list[ObjectiveGap] = []
    for obj in (objectives or DEFAULT_OBJECTIVES):
        coherent_car = dict(base_car)
        phantom_car_ = dict(base_car)
        for ch, pairs in touched.items():
            car = cars.get(f"car.{ch}")
            if car is None or car.sigma <= 0.0:
                continue
            adverse = _objective_adverse_dir(obj, base_car, ch, car.sigma)
            if adverse == 0.0:
                continue                       # objective blind to this channel
            coherent_car[ch] = car.value + adverse * z_star * car.sigma
            implied = [_as_car_delta(d, cq, cars)[1] for d, cq in pairs]
            worst = max(implied) if adverse > 0 else min(implied)
            # the phantom never sits INSIDE nominal — fears only push outward
            if adverse > 0:
                phantom_car_[ch] = max(worst, car.value)
            else:
                phantom_car_[ch] = min(worst, car.value)
        nom = obj.fn(base_car)
        coh = obj.fn(coherent_car)
        pha = obj.fn(phantom_car_)
        raw = pha - coh
        gap = raw if obj.better == "lower" else -raw
        gaps.append(ObjectiveGap(objective_key=obj.key, label=obj.label,
                                 unit=obj.unit, better=obj.better,
                                 nominal=nom, coherent=coh, phantom=pha,
                                 gap=gap))

    # ---- unaudited consumers — the honest blind spot ---------------------- #
    declared = {(d.consumer, d.quantity_key) for d in declarations}
    unaudited = []
    for consumer, key, adverse, why in DEFAULT_CONSUMPTION:
        if key in cars and (consumer, key) not in declared:
            unaudited.append({"consumer": consumer, "quantity_key": key,
                              "why": why})

    return MarginAudit(charter=charter.as_dict(), charter_z=z_star,
                       refused=False, refusal_reason="",
                       generated_on=stamp,
                       findings=findings, quantity_coverage=coverage,
                       consumer_phantoms=phantoms, objective_gaps=gaps,
                       unresolved=sorted(set(unresolved)),
                       unaudited_consumers=unaudited)


# --------------------------------------------------------------------------- #
#  The docket — judge-ready markdown
# --------------------------------------------------------------------------- #
def _odds_en(p: ConsumerPhantom) -> str:
    if p.beta <= 0.0:
        return "no stacked worst cases"
    if p.odds_one_in == 0.0:
        return f"β = {p.beta:.2f} — beyond 1 in 10⁹; essentially impossible"
    return f"β = {p.beta:.2f} — a 1-in-{p.odds_one_in:,.0f} car"


def render_docket_md(a: MarginAudit, frame_note: str = "") -> str:
    """The Margin Docket: everything above, as a pinnable one-pager."""
    L: list[str] = []
    L.append("# 👻 Phantom Car — Margin Docket")
    L.append("")
    ch = a.charter
    if a.refused:
        L.append(f"**REFUSED** — {a.refusal_reason}")
        return "\n".join(L)
    L.append(f"Charter: design to the **{ch.get('percentile', 0):.1f}th-"
             f"percentile car** (z* = {a.charter_z:.2f}σ), design factor "
             f"rule: *{ch.get('fos_rule', 'once')}*. Sealed "
             f"`{ch.get('seal', '')[:16]}…` on {ch.get('created_on', '?')}."
             + (f" Frame: {frame_note}." if frame_note else ""))
    L.append("")
    L.append("Every hedge below is priced in σ of the quantity's own "
             "evidence-graded uncertainty (Proof Engine ledger). "
             "Deterministic: same deck, same docket.")
    L.append("")
    L.append("## Verdicts")
    L.append("")
    L.append("| Consumer | Quantity | Assumes | Hedge | +Factor | Cover | "
             "Verdict |")
    L.append("|---|---|---|---|---|---|---|")
    for f in a.findings:
        extra = ""
        if f.releasable:
            extra = f" — **{f.releasable:.3g} {f.unit} releasable**"
        if f.exposure:
            extra = f" — **{f.exposure:.3g} {f.unit} exposed**"
        L.append(f"| {f.consumer} | {f.quantity_label} | "
                 f"{f.assumed_value:.4g} {f.unit} | {f.z_assumed:+.2f}σ | "
                 f"{f.z_factor:+.2f}σ (×{f.design_factor:.2f}) | "
                 f"{f.coverage_pct:.1f}% | {f.verdict}{extra} |")
    L.append("")
    two = [q for q in a.quantity_coverage if q.contradictory]
    if two:
        L.append("## ⚠️ The deck describes more than one car")
        L.append("")
        for q in two:
            L.append(f"- **{q.label}**: *{q.hi_consumer}* designs to "
                     f"{q.hi_assumed:.4g} {q.unit} while *{q.lo_consumer}* "
                     f"designs to {q.lo_assumed:.4g} {q.unit} — "
                     f"**{q.spread_sigma:.1f}σ apart**. Both cannot be the "
                     "same physical car; one of these sizings is wrong "
                     "today.")
        L.append("")
    L.append("## β — the improbability each consumer defends against")
    L.append("")
    for p in a.consumer_phantoms:
        L.append(f"- **{p.consumer}** ({p.n_hedged} stacked worst case"
                 f"{'s' if p.n_hedged != 1 else ''}): {_odds_en(p)}")
    L.append("")
    L.append("## The three cars, per objective")
    L.append("")
    L.append("| Objective | Nominal | Coherent (charter) | Phantom | "
             "Over-defence vs charter |")
    L.append("|---|---|---|---|---|")
    for g in a.objective_gaps:
        L.append(f"| {g.label} | {g.nominal:.4g} {g.unit} | "
                 f"{g.coherent:.4g} {g.unit} | {g.phantom:.4g} {g.unit} | "
                 f"{g.gap:+.3g} {g.unit} |")
    L.append("")
    L.append("_Positive over-defence is design ENVELOPE currently spent "
             "defending cars the deck's own σ says are statistically "
             "impossible — not promised savings; releasing it is a design "
             "decision, this docket only prices it. Negative means the union "
             "of everyone's declared fears still under-defends the charter "
             "car._")
    if a.unresolved:
        L.append("")
        L.append("**Unresolved declarations** (keys matching nothing in the "
                 "deck — fix, don't ignore): " + ", ".join(a.unresolved))
    if a.unaudited_consumers:
        L.append("")
        L.append("**Unaudited consumers** — known sizing paths with no "
                 "disclosed assumption; their hedges are invisible to this "
                 "docket until declared:")
        for u in a.unaudited_consumers:
            L.append(f"- {u['consumer']} ← {u['quantity_key']}")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  Demo declarations — the classic pathology, for the self-test and the UI
# --------------------------------------------------------------------------- #
def demo_declarations(quantities: list[Quantity],
                      today: Optional[_dt.date] = None
                      ) -> list[MarginDeclaration]:
    """
    The textbook failure set, built from whatever the deck declares:
      * structures hedges the mount load +2σ AND applies FoS 1.5 (STACKED),
      * brakes design to a heavy car (+2σ mass),
      * the energy budget assumes the optimistic target mass (ANTI-HEDGED —
        and therefore CONTRADICTORY with brakes on the same quantity),
      * cooling consumes a guess-grade heat load naked (NAKED).
    """
    cars = car_quantities(quantities, today)
    out: list[MarginDeclaration] = []

    def add(consumer, key, adverse, dz, factor=1.0, why=""):
        cq = cars.get(key)
        if cq is None or cq.sigma <= 0:
            return
        d = +1.0 if adverse == _HIGH else -1.0
        out.append(MarginDeclaration(
            consumer=consumer, quantity_key=key, adverse=adverse,
            assumed_value=cq.value + d * dz * cq.sigma,
            design_factor=factor, rationale=why))

    add("structures / mounts", "car.mount_load_n", _HIGH, 2.0, 1.5,
        "worst-case load, then FoS 1.5 on top")
    add("brake sizing", "car.mass_kg", _HIGH, 2.0, 1.0,
        "sized for the car 'if it comes in heavy'")
    add("energy budget", "car.mass_kg", _HIGH, -1.5, 1.0,
        "assumes the target weight — 'we'll get it down'")
    add("pack cooling sizing", "car.heat_reject_w", _HIGH, 0.0, 1.0,
        "takes the declared heat load at face value")
    return out


# --------------------------------------------------------------------------- #
#  Self-test (python3 -m suspension.phantom_car)
# --------------------------------------------------------------------------- #
if __name__ == "__main__":  # pragma: no cover
    from .interfaces import SubsystemInterface, blank_ledger
    from .proof_engine import build_uncertainty_ledger
    led = blank_ledger()
    led.set(SubsystemInterface(name="chassis", mass_kg=45, cg_z_mm=300,
                               mount_load_n=4200, is_estimate=True))
    led.set(SubsystemInterface(name="powertrain", mass_kg=60, cg_z_mm=320,
                               peak_power_kw=68, peak_torque_nm=180,
                               heat_reject_w=3200, is_estimate=True))
    led.set(SubsystemInterface(name="accumulator", mass_kg=55,
                               peak_current_a=180, power_draw_w=350,
                               is_estimate=True))
    led.set(SubsystemInterface(name="cooling", mass_kg=6,
                               cooling_airflow_cms=0.14, is_estimate=True))
    qs = build_uncertainty_ledger(led)
    charter = create_charter(95.0, note="one phantom, not eight",
                             author="self-test")
    decs = demo_declarations(qs)
    a = audit(charter, decs, qs)
    print(render_docket_md(a))
