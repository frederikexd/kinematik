# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  Module: fusebox — the failure-order audit
# ============================================================================
"""
Fusebox — every load path fails somewhere. Did the car choose where?

THE PROBLEM THIS SOLVES
-----------------------
Electrical engineering answered this question a century and a half ago: when
the overload comes, a FUSE — a cheap, sacrificial, stocked-in-the-box element
— is DESIGNED to be the thing that dies, so nothing expensive does. Every
circuit has a chosen victim, and choosing it is a first-class design act.

Mechanical load paths on a formula car answer the same question by accident.
A front-corner curb strike sends one load through a chain: tie rod, wishbone
leg, upright, chassis tab. SOMETHING in that chain fails first — that is not
a risk, it is a certainty conditional on a big enough hit, and formula cars
hit curbs, cones, and each other every season. Which element goes first is
decided by whichever capacity happens to be lowest. Nobody chose it. Nobody
even knows it, because:

  1. THE ORDER IS INVISIBLE TO EVERY TOOL IN THE CHAIN. FEA checks one part
     against one load case and reports ITS margin; it cannot see that the
     upright's 1.8 and the tie rod's 1.35 are an ORDERING, let alone whether
     it is the right one. DFMEA ranks failure modes by RPN folklore, not by
     which physically happens first on a shared path. Requirements tools see
     targets. The ordering lives between the tools, so no tool owns it.

  2. UNDER THE DECK'S OWN UNCERTAINTY, THE ORDER ISN'T EVEN DETERMINED.
     FoS 1.35 on the tie rod (MODELLED, ±10 %) vs FoS 1.8 on the upright
     (GUESS from last year's hand calc, ±40 %) is not "tie rod first". Price
     both capacities with the same evidence-graded σ the Proof Engine already
     maintains and the upright fails first in roughly one curb strike in
     four. The team believes it has a cheap fuse; it is actually flipping a
     weighted coin between a $45 rod-end afternoon and a $900, six-week
     billet upright — a competition-ending part, lost to a Tuesday curb.

  3. ON AN EV, SOME ELEMENTS MUST NEVER BE FIRST. The accumulator container,
     its mounts, the cell-stack restraint, the firewall: the entire point of
     the surrounding structure is that these are LAST in every ordering. That
     is exactly the kind of claim that is believed by construction and
     verified by nobody — a GUESS-grade container-mount capacity can put a
     few percent of first-failure probability on the one element whose
     failure is not a repair bill but a safety event and an instant DNF.

WHAT IT DOES
------------
  * THE PECKING ORDER. For each declared overload path (a chain of elements
    sharing one rising load), every element's capacity is a random variable:
    mean = its FoS at the path's reference load, σ = FoS × the SAME
    evidence-graded, staleness-inflated relative band the Proof Engine
    assigns that grade — one pedigree law, fifth consumer, zero new physics.
    The probability each element fails FIRST is the standard first-order
    statistics of the minimum of independent normals,
        P(i first) = ∫ φ_i(x) · Π_{j≠i} [1 − Φ_j(x)] dx,
    computed by fixed-grid trapezoid quadrature — deterministic, same deck
    in, same order out, and for two elements it collapses to the closed form
    Φ((μ_j−μ_i)/√(σ_i²+σ_j²)) a stubborn lead can check on a napkin.
  * VERDICTS AGAINST A SEALED CHARTER. The team designates the intended fuse
    per path and one confidence level for the car, sha256-sealed. Each path
    judges: FUSED (the designated S1 element is first at ≥ the charter
    confidence) / COIN-FLIP (the ordering is undetermined by the deck's own
    σ — the contenders are named) / INVERTED (the most likely first failure
    is a structural part, with the inversion priced in $ and days against
    the intended fuse) / UNFUSED (no fuse-grade element exists on the path
    at all — the chain has no cheap victim by construction) / BREACH-RISK
    (a forbidden element carries more first-failure probability than the
    charter tolerates — this verdict outranks every other).
  * FIX ARITHMETIC, NOT FIX FOLKLORE. For every rival that threatens the
    designated fuse, three levers are solved EXACTLY from the pairwise
    normal formula: (a) lower the fuse's FoS to the printed value (floored
    — a fuse that pops at 1.0 pops in normal driving); (b) raise the
    rival's FoS to the printed value; (c) SHARPEN the rival's evidence grade
    — the printed grade whose tighter band alone restores the ordering.
    Lever (c) is the one no redesign meeting ever tables: a strain-gauge
    pull test on the upright can buy the same ordering certainty as three
    weeks of re-machining, because half the coin-flip was never mechanics —
    it was a GUESS-grade capacity band doing the flipping.
  * THE OVERLOAD BILL. Conditional on the hit arriving, the expected bill of
    the CURRENT ordering (Σ P(i first) × its cost, and the same in days of
    downtime) next to the bill of the intended fuse. The difference is the
    price of the unmanaged pecking order — an expected value, stated as
    conditional on the event, never as a promised saving.
  * INCIDENT JUDGING — THE FREE DATUM. When something actually breaks, the
    sealed charter judges the incident: AS-DESIGNED / SURPRISE / BREACH. A
    SURPRISE carries the one consolation prize of every breakage: the part
    that failed first just delivered a free capacity measurement (its true
    capacity is bounded by the event), and the verdict says to bank it —
    re-grade that capacity and the pecking order sharpens for free. An
    edited charter refuses to judge, out loud.

WHY NO ONE HAS BUILT IT
-----------------------
Fuse coordination is a solved discipline in electrical protection and a
staffed process (frangibility, designed break points) at aerospace primes —
and it does not exist in any tool a student team can afford, because the
computation needs every element's capacity, the σ implied by its evidence
quality, its replacement cost and lead time, and the map of which elements
share a path, joined in one place. A solver vendor sees one part per run.
In KinematiK the σ law, the costs, and the chain already live together, so
Fusebox is a join plus one honest declaration per path — not a new deck.

HONESTY CONTRACT (same as the rest of KinematiK)
------------------------------------------------
Deterministic end to end: no Monte Carlo, no fitted constants. The quadrature
grid is fixed; the pairwise closed form is printed so any verdict can be
spot-checked by hand. Assumptions said out loud: capacities are treated as
independent normals (the σ of a GUESS puts <1 % mass below zero — stated,
not hidden); the chain is treated as series (every element sees the path
load scaled by its own declared FoS reference — an element loaded at 40 % of
the path load simply carries that in its FoS). Elements without a severity
class, and paths without elements, are reported as blind spots — never
absorbed into a green board.

No streamlit / pandas / plotly imports. Pure stdlib. Unit-testable headless.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
from dataclasses import dataclass, field, asdict
from enum import Enum
from statistics import NormalDist
from typing import Optional

from .proof_engine import EvidenceGrade, effective_rel_unc

_N = NormalDist()          # standard normal — stdlib, exact enough, no scipy

# Fixed quadrature grid size. Deterministic by construction: same inputs,
# same grid, same probabilities, forever.
GRID_POINTS = 4001

# A designated fuse may be softened, but never below this FoS at the path's
# reference load — a fuse that pops at 1.0 pops in normal driving.
MIN_FUSE_FOS = 1.10

# Default charter constants (documented; the charter can override).
DEFAULT_CONFIDENCE = 0.90     # P(designated fuse first) required for FUSED
DEFAULT_FORBIDDEN_P = 0.01    # max tolerated P(first) on any S3 element


# --------------------------------------------------------------------------- #
#  Severity classes — what kind of victim each element would be
# --------------------------------------------------------------------------- #
class Severity(str, Enum):
    S1_FUSE_GRADE = "S1"   # bolt-on, spare in the box, hours to swap
    S2_STRUCTURAL = "S2"   # custom / long-lead / frame repair
    S3_FORBIDDEN = "S3"    # safety-critical: must never fail first


SEVERITY_META = {
    Severity.S1_FUSE_GRADE: (
        "Fuse-grade", "Bolt-on sacrificial part. A spare lives in the box; "
        "the car is back out the same day."),
    Severity.S2_STRUCTURAL: (
        "Structural", "Custom or long-lead part, or frame damage. Losing it "
        "first costs weeks, not hours."),
    Severity.S3_FORBIDDEN: (
        "Forbidden-first", "Safety-critical (accumulator container/mounts, "
        "cell restraint, firewall, driver cell). Failing FIRST is a safety "
        "event, not a repair bill — the surrounding structure exists so "
        "this element is last in every ordering."),
}


# --------------------------------------------------------------------------- #
#  Path model
# --------------------------------------------------------------------------- #
@dataclass
class PathElement:
    """One element of a load chain.

    fos:  capacity / the load THIS ELEMENT sees when the path carries the
          archetype reference load. An element that only sees 40 % of the
          path load carries that share inside its FoS — the chain is then a
          clean series race in one load multiplier.
    grade / age_days: pedigree of the CAPACITY number, priced with the exact
          Proof Engine band law (evidence grade → relative σ, inflated by
          staleness). A checkbox can't shrink σ; only a better grade can.
    """
    key: str
    label: str
    fos: float
    grade: EvidenceGrade = EvidenceGrade.ESTIMATE
    severity: Optional[Severity] = None
    replace_cost_usd: float = 0.0
    downtime_days: float = 0.0
    age_days: float = 0.0
    note: str = ""

    def __post_init__(self):
        if self.fos <= 0:
            raise ValueError(f"{self.key}: FoS must be > 0")

    @property
    def rel_unc(self) -> float:
        return effective_rel_unc(self.grade, self.age_days)

    @property
    def mu(self) -> float:
        return float(self.fos)

    @property
    def sigma(self) -> float:
        return self.fos * self.rel_unc

    def as_dict(self) -> dict:
        d = asdict(self)
        d["grade"] = self.grade.value
        d["severity"] = self.severity.value if self.severity else None
        return d

    @staticmethod
    def from_dict(d: dict) -> "PathElement":
        d = dict(d)
        d["grade"] = EvidenceGrade(d.get("grade", "estimate"))
        sev = d.get("severity")
        d["severity"] = Severity(sev) if sev else None
        valid = PathElement.__dataclass_fields__.keys()
        return PathElement(**{k: v for k, v in d.items() if k in valid})


@dataclass
class OverloadPath:
    """A credible overload archetype and the chain that carries it."""
    key: str
    label: str
    story: str
    elements: list[PathElement] = field(default_factory=list)
    designated_fuse_key: str = ""

    def element(self, key: str) -> Optional[PathElement]:
        for e in self.elements:
            if e.key == key:
                return e
        return None

    def as_dict(self) -> dict:
        return {"key": self.key, "label": self.label, "story": self.story,
                "designated_fuse_key": self.designated_fuse_key,
                "elements": [e.as_dict() for e in self.elements]}

    @staticmethod
    def from_dict(d: dict) -> "OverloadPath":
        return OverloadPath(
            key=d["key"], label=d.get("label", d["key"]),
            story=d.get("story", ""),
            designated_fuse_key=d.get("designated_fuse_key", ""),
            elements=[PathElement.from_dict(x) for x in d.get("elements", [])])


# --------------------------------------------------------------------------- #
#  First-failure statistics — deterministic, checkable
# --------------------------------------------------------------------------- #
def pairwise_first(mu_i: float, sig_i: float,
                   mu_j: float, sig_j: float) -> float:
    """P(C_i < C_j) for independent normals — the napkin formula.

    Φ((μ_j − μ_i) / √(σ_i² + σ_j²)). Exact; every multi-way verdict below
    can be spot-checked against this two-element collapse.
    """
    denom = math.sqrt(sig_i * sig_i + sig_j * sig_j)
    if denom <= 0.0:
        return 1.0 if mu_i < mu_j else (0.5 if mu_i == mu_j else 0.0)
    return _N.cdf((mu_j - mu_i) / denom)


def first_failure_probs(elements: list[PathElement],
                        grid_points: int = GRID_POINTS) -> dict[str, float]:
    """P(element fails first) = P(its capacity is the minimum of the chain).

    Fixed-grid trapezoid quadrature of ∫ φ_i(x) Π_{j≠i} (1−Φ_j(x)) dx over
    [min(μ−6σ), max(μ+6σ)]. Deterministic; renormalised (analytically the
    probabilities sum to 1; the residual is quadrature error and is tiny at
    the fixed grid).
    """
    if not elements:
        return {}
    if len(elements) == 1:
        return {elements[0].key: 1.0}
    dists = [(e.key, NormalDist(e.mu, max(e.sigma, 1e-12))) for e in elements]
    lo = min(d.mean - 6.0 * d.stdev for _, d in dists)
    hi = max(d.mean + 6.0 * d.stdev for _, d in dists)
    n = max(int(grid_points), 101)
    dx = (hi - lo) / (n - 1)
    raw = {k: 0.0 for k, _ in dists}
    for step in range(n):
        x = lo + step * dx
        w = 0.5 if step in (0, n - 1) else 1.0
        surv = [(k, 1.0 - d.cdf(x)) for k, d in dists]
        prod_all = 1.0
        for _, s in surv:
            prod_all *= s
        for idx, (k, d) in enumerate(dists):
            s_k = surv[idx][1]
            others = prod_all / s_k if s_k > 1e-300 else _prod_except(surv, idx)
            raw[k] += w * d.pdf(x) * others * dx
    total = sum(raw.values())
    if total <= 0.0:
        return {k: 1.0 / len(raw) for k in raw}
    return {k: v / total for k, v in raw.items()}


def _prod_except(surv: list[tuple[str, float]], skip: int) -> float:
    p = 1.0
    for i, (_, s) in enumerate(surv):
        if i != skip:
            p *= s
    return p


# --------------------------------------------------------------------------- #
#  The path audit
# --------------------------------------------------------------------------- #
class PathVerdict(str, Enum):
    FUSED = "FUSED"                 # designated S1 fuse first at ≥ confidence
    COIN_FLIP = "COIN-FLIP"         # ordering undetermined by the deck's own σ
    INVERTED = "INVERTED"           # a structural part is the likely victim
    UNFUSED = "UNFUSED"             # no fuse-grade element on the path at all
    BREACH_RISK = "BREACH-RISK"     # a forbidden element can plausibly be first


@dataclass
class PathAudit:
    path_key: str
    verdict: PathVerdict
    probs: dict[str, float]                 # element key → P(first)
    leader_key: str
    leader_p: float
    fuse_key: str                           # designated ("" if none)
    fuse_p: float
    contenders: list[str]                   # keys with P(first) ≥ 0.10
    forbidden_hits: list[tuple[str, float]]  # (S3 key, P) above threshold
    expected_cost_usd: float                # Σ p·cost — conditional on the hit
    expected_downtime_days: float
    fuse_cost_usd: float                    # bill if the intended fuse goes
    fuse_downtime_days: float
    blind_spots: list[str]
    headline: str


def audit_path(path: OverloadPath,
               confidence: float = DEFAULT_CONFIDENCE,
               forbidden_p: float = DEFAULT_FORBIDDEN_P) -> PathAudit:
    """Judge one overload path against the charter constants.

    Verdict precedence (worst wins): BREACH-RISK > UNFUSED > INVERTED >
    COIN-FLIP > FUSED. All thresholds are the documented module constants
    unless a sealed charter overrides them.
    """
    blind: list[str] = []
    if not path.elements:
        return PathAudit(path.key, PathVerdict.UNFUSED, {}, "", 0.0,
                         path.designated_fuse_key, 0.0, [], [], 0.0, 0.0,
                         0.0, 0.0,
                         [f"path '{path.label}' has no declared elements — "
                          "nothing here is audited"],
                         "No elements declared: this chain's first failure "
                         "is completely unexamined.")
    for e in path.elements:
        if e.severity is None:
            blind.append(f"'{e.label}' has no severity class — treated as "
                         "structural for verdicts, listed here so the gap "
                         "is never silent")
    probs = first_failure_probs(path.elements)
    leader_key = max(probs, key=lambda k: probs[k])
    leader_p = probs[leader_key]
    leader = path.element(leader_key)
    fuse = path.element(path.designated_fuse_key)
    fuse_p = probs.get(path.designated_fuse_key, 0.0)
    contenders = sorted([k for k, p in probs.items() if p >= 0.10],
                        key=lambda k: -probs[k])
    forbidden_hits = sorted(
        [(e.key, probs[e.key]) for e in path.elements
         if e.severity is Severity.S3_FORBIDDEN
         and probs[e.key] > forbidden_p],
        key=lambda t: -t[1])
    exp_cost = sum(probs[e.key] * e.replace_cost_usd for e in path.elements)
    exp_days = sum(probs[e.key] * e.downtime_days for e in path.elements)
    fuse_cost = fuse.replace_cost_usd if fuse else 0.0
    fuse_days = fuse.downtime_days if fuse else 0.0

    has_s1 = any(e.severity is Severity.S1_FUSE_GRADE for e in path.elements)
    lead_sev = leader.severity if (leader and leader.severity) \
        else Severity.S2_STRUCTURAL

    if forbidden_hits:
        v = PathVerdict.BREACH_RISK
        k0, p0 = forbidden_hits[0]
        el0 = path.element(k0)
        head = (f"'{el0.label}' is forbidden-first (S3) yet carries "
                f"{p0:.1%} first-failure probability — above the "
                f"{forbidden_p:.0%} the charter tolerates. Its capacity is "
                f"{el0.grade.value.upper()} grade (±{el0.rel_unc:.0%}); "
                "sharpening that band may retire the breach without metal.")
    elif not has_s1:
        v = PathVerdict.UNFUSED
        head = ("No fuse-grade (S1) element exists on this chain — whatever "
                "breaks first is expensive by construction. Designate or "
                "add a sacrificial element.")
    elif fuse is not None and leader_key == path.designated_fuse_key \
            and fuse_p >= confidence:
        v = PathVerdict.FUSED
        head = (f"'{fuse.label}' fails first with {fuse_p:.0%} probability "
                f"(charter asks {confidence:.0%}). The car has chosen its "
                "victim.")
    elif leader_key != path.designated_fuse_key \
            and lead_sev is not Severity.S1_FUSE_GRADE:
        v = PathVerdict.INVERTED
        head = (f"Most likely first failure is '{leader.label}' "
                f"({lead_sev.value}, {leader_p:.0%}) — not the designated "
                f"fuse ({fuse_p:.0%}). Conditional on the hit, that swaps a "
                f"${fuse_cost:,.0f}/{fuse_days:.1f}-day repair for "
                f"${leader.replace_cost_usd:,.0f}/"
                f"{leader.downtime_days:.1f} days.")
    else:
        v = PathVerdict.COIN_FLIP
        names = ", ".join(
            f"'{path.element(k).label}' {probs[k]:.0%}" for k in contenders)
        head = (f"The ordering is undetermined at the deck's own σ: "
                f"{names}. The designated fuse holds {fuse_p:.0%} against a "
                f"{confidence:.0%} charter — the first failure is a "
                "weighted coin flip, not a decision.")

    return PathAudit(path.key, v, probs, leader_key, leader_p,
                     path.designated_fuse_key, fuse_p, contenders,
                     forbidden_hits, exp_cost, exp_days, fuse_cost,
                     fuse_days, blind, head)


# --------------------------------------------------------------------------- #
#  Fix arithmetic — three exact levers per rival
# --------------------------------------------------------------------------- #
@dataclass
class Prescription:
    rival_key: str
    pair_p_now: float            # P(fuse first vs this rival), pairwise
    lower_fuse_fos_to: Optional[float]   # None ⇒ lever infeasible
    lower_fuse_note: str
    raise_rival_fos_to: Optional[float]
    raise_rival_note: str
    sharpen_rival_to: Optional[EvidenceGrade]
    sharpen_note: str


def _solve_lower_fuse(mu_j: float, sig_j: float, r_f: float,
                      z: float) -> Optional[float]:
    """Smallest-change fuse mean m < μ_j with Φ((μ_j−m)/√(r_f²m²+σ_j²)) = c.

    Squares to  m²(1−z²r_f²) − 2μ_j m + (μ_j² − z²σ_j²) = 0; smaller root.
    """
    a = 1.0 - z * z * r_f * r_f
    if a <= 0.0:
        return None
    disc = z * z * (mu_j * mu_j * r_f * r_f + a * sig_j * sig_j)
    m = (mu_j - math.sqrt(disc)) / a
    return m if m > 0.0 else None


def _solve_raise_rival(mu_f: float, sig_f: float, r_j: float,
                       z: float) -> Optional[float]:
    """Smallest rival mean m > μ_f with Φ((m−μ_f)/√(σ_f²+r_j²m²)) = c.

    Squares to  m²(1−z²r_j²) − 2μ_f m + (μ_f² − z²σ_f²) = 0; larger root.
    If z·r_j ≥ 1 the rival's σ grows as fast as its margin — no finite FoS
    restores the ordering; the honest answer is lever (c).
    """
    a = 1.0 - z * z * r_j * r_j
    if a <= 0.0:
        return None
    disc = z * z * (mu_f * mu_f * r_j * r_j + a * sig_f * sig_f)
    return (mu_f + math.sqrt(disc)) / a


def prescribe(path: OverloadPath,
              confidence: float = DEFAULT_CONFIDENCE) -> list[Prescription]:
    """For every rival beating the pairwise confidence, the three exact fixes.

    Pairwise: the joint P(fuse first) is also reported by audit_path; pairwise
    dominance against every rival is the necessary condition a lead can act
    on one rival at a time. Fresh grades (age 0) are assumed for lever (c) —
    a sharpened band that is allowed to go stale was never sharpened.
    """
    fuse = path.element(path.designated_fuse_key)
    if fuse is None:
        return []
    z = _N.inv_cdf(confidence)
    out: list[Prescription] = []
    for e in path.elements:
        if e.key == fuse.key:
            continue
        p_now = pairwise_first(fuse.mu, fuse.sigma, e.mu, e.sigma)
        if p_now >= confidence:
            continue
        # (a) lower the fuse
        m_low = _solve_lower_fuse(e.mu, e.sigma, fuse.rel_unc, z)
        if m_low is None:
            low_note = ("infeasible: the fuse's own band is too wide for any "
                        "FoS to dominate — sharpen the fuse's grade first")
            m_low_out = None
        elif m_low < MIN_FUSE_FOS:
            low_note = (f"needs FoS {m_low:.2f} < the {MIN_FUSE_FOS:.2f} "
                        "floor — a fuse that soft pops in normal driving; "
                        "use another lever")
            m_low_out = None
        else:
            low_note = (f"soften '{fuse.label}' to FoS {m_low:.2f} at this "
                        "path's reference load")
            m_low_out = round(m_low, 3)
        # (b) raise the rival
        m_hi = _solve_raise_rival(fuse.mu, fuse.sigma, e.rel_unc, z)
        if m_hi is None:
            hi_note = (f"infeasible: '{e.label}' is {e.grade.value.upper()} "
                       f"grade (±{e.rel_unc:.0%}) — its band grows as fast "
                       "as any FoS you add; no amount of metal fixes an "
                       "unknown. Sharpen the grade instead")
            m_hi_out = None
        else:
            hi_note = (f"stiffen '{e.label}' to FoS {m_hi:.2f}")
            m_hi_out = round(m_hi, 3)
        # (c) sharpen the rival's evidence grade — certainty instead of metal
        sharpen_to: Optional[EvidenceGrade] = None
        for g in (EvidenceGrade.MODELLED, EvidenceGrade.MEASURED,
                  EvidenceGrade.VERIFIED):
            unc_g = effective_rel_unc(g, 0.0)
            if unc_g >= e.rel_unc:
                continue        # not actually a sharpening
            if pairwise_first(fuse.mu, fuse.sigma,
                              e.mu, e.mu * unc_g) >= confidence:
                sharpen_to = g
                break
        if sharpen_to is not None:
            sh_note = (f"measure, don't machine: re-grade '{e.label}' "
                       f"capacity to {sharpen_to.value.upper()} "
                       f"(±{effective_rel_unc(sharpen_to, 0.0):.0%}) and the "
                       "ordering is restored with zero new metal")
        else:
            sh_note = ("even a VERIFIED band can't restore the ordering "
                       "alone — the means are genuinely too close; combine "
                       "with lever (a) or (b)")
        out.append(Prescription(e.key, p_now, m_low_out, low_note,
                                m_hi_out, hi_note, sharpen_to, sh_note))
    return sorted(out, key=lambda p: p.pair_p_now)


# --------------------------------------------------------------------------- #
#  The sealed Fuse Charter
# --------------------------------------------------------------------------- #
class IncidentVerdict(str, Enum):
    AS_DESIGNED = "AS-DESIGNED"
    SURPRISE = "SURPRISE"
    BREACH = "BREACH"


_CHARTER_SEALED = ("confidence", "forbidden_p", "designations",
                   "created_utc", "note")


def _charter_payload(ch: dict) -> str:
    return json.dumps({k: ch.get(k) for k in _CHARTER_SEALED},
                      sort_keys=True, separators=(",", ":"))


def create_charter(designations: dict[str, str],
                   confidence: float = DEFAULT_CONFIDENCE,
                   forbidden_p: float = DEFAULT_FORBIDDEN_P,
                   note: str = "") -> dict:
    """Seal the team's chosen victims. designations: path_key → fuse key."""
    if not (0.5 < confidence < 1.0):
        raise ValueError("confidence must be in (0.5, 1.0)")
    if not (0.0 < forbidden_p < 0.5):
        raise ValueError("forbidden_p must be in (0, 0.5)")
    ch = {"confidence": float(confidence), "forbidden_p": float(forbidden_p),
          "designations": dict(sorted(designations.items())),
          "created_utc": _dt.datetime.now(_dt.timezone.utc)
          .strftime("%Y-%m-%d %H:%M UTC"),
          "note": note}
    ch["seal"] = hashlib.sha256(_charter_payload(ch).encode()).hexdigest()
    return ch


def charter_intact(ch: dict) -> bool:
    return bool(ch.get("seal")) and \
        hashlib.sha256(_charter_payload(ch).encode()).hexdigest() == ch["seal"]


def judge_incident(charter: dict, path: OverloadPath,
                   broke_first_key: str,
                   event_note: str = "") -> tuple[IncidentVerdict, str]:
    """Judge what actually broke against what was sealed.

    Raises on a broken seal — an edited charter refuses to judge, out loud.
    Every SURPRISE ships the free datum: the element that failed first just
    had its capacity measured by reality; bank it.
    """
    if not charter_intact(charter):
        raise ValueError(
            "Fuse Charter seal is broken — a sealed designation was edited "
            "after sealing. Re-seal a new charter; this one refuses to "
            "judge.")
    el = path.element(broke_first_key)
    if el is None:
        raise ValueError(f"'{broke_first_key}' is not an element of path "
                         f"'{path.key}'")
    designated = charter.get("designations", {}).get(path.key, "")
    datum = (f"Free datum: reality just measured '{el.label}' — its true "
             f"capacity is bounded by this event. Re-grade that capacity "
             f"to MEASURED with the event magnitude and the whole pecking "
             f"order sharpens at zero cost.")
    if el.severity is Severity.S3_FORBIDDEN:
        return (IncidentVerdict.BREACH,
                f"BREACH: forbidden-first element '{el.label}' failed first. "
                "This is a safety event, not a repair bill — the load path "
                "must be redesigned so an S1/S2 element yields before it. "
                + datum + (f" [{event_note}]" if event_note else ""))
    if broke_first_key == designated:
        return (IncidentVerdict.AS_DESIGNED,
                f"AS-DESIGNED: the designated fuse '{el.label}' took the "
                "hit, exactly as sealed. Swap the spare, log the event "
                "load if known, race on. "
                + datum + (f" [{event_note}]" if event_note else ""))
    return (IncidentVerdict.SURPRISE,
            f"SURPRISE: '{el.label}' failed first, not the designated fuse. "
            "The sealed ordering was wrong about reality — audit whether "
            "the miss was capacity (this element weaker than declared) or "
            "load share (it saw more of the path load than its FoS "
            "assumed). " + datum + (f" [{event_note}]" if event_note else ""))


# --------------------------------------------------------------------------- #
#  Seed paths — a representative FSAE-EV fuse map, stories included
# --------------------------------------------------------------------------- #
def seed_paths() -> list[OverloadPath]:
    """Four archetypes every FSAE-EV season actually meets.

    Numbers are representative and editable; the grades are chosen to tell
    the true story — the upright capacity that is still last year's hand
    calc, the accumulator mount nobody has pull-tested.
    """
    G, E, M = EvidenceGrade.GUESS, EvidenceGrade.ESTIMATE, EvidenceGrade.MODELLED
    S1, S2, S3 = (Severity.S1_FUSE_GRADE, Severity.S2_STRUCTURAL,
                  Severity.S3_FORBIDDEN)
    return [
        OverloadPath(
            "front_curb", "Front-corner curb strike",
            "AutoX exit curb taken a wheel-width wide: one lateral spike "
            "through the whole front corner chain.",
            [PathElement("tie_rod", "Tie rod (outer)", 1.35, M, S1, 45, 0.2,
                         note="the classic motorsport fuse — cheap, stocked"),
             PathElement("lca_front", "LCA front leg", 1.60, E, S2, 220, 4.0),
             PathElement("upright", "Upright (billet)", 1.80, G, S2, 900,
                         21.0, note="capacity is last year's hand calc"),
             PathElement("chassis_tab", "Chassis pickup tab", 2.20, E, S2,
                         350, 10.0, note="frame repair, jig time")],
            designated_fuse_key="tie_rod"),
        OverloadPath(
            "wing_strike", "Front-wing cone/kerb strike",
            "Endurance cone under the front wing at speed: drag load "
            "through the wing into the chassis.",
            [PathElement("wing_skin", "Wing element skin", 1.20, E, S1, 120,
                         0.5),
             PathElement("shear_pin", "Mount shear pin", 1.30, M, S1, 4, 0.1,
                         note="designed break point"),
             PathElement("wing_bracket", "Wing mount bracket", 1.70, E, S2,
                         160, 5.0),
             PathElement("nose_pickup", "Nose/chassis pickup", 2.10, E, S2,
                         300, 9.0)],
            designated_fuse_key="shear_pin"),
        OverloadPath(
            "tow_yank", "Recovery tow at 30° off-axis",
            "A gravel-trap recovery yank — the marshal's truck does not "
            "read your load cases.",
            [PathElement("tow_hook", "Tow hook", 1.40, M, S1, 25, 0.2),
             PathElement("tow_tab", "Tow tab weld", 1.90, E, S2, 90, 3.0),
             PathElement("frame_node", "Frame node", 2.40, E, S2, 400, 12.0)],
            designated_fuse_key="tow_hook"),
        OverloadPath(
            "side_accu", "Side load into accumulator bay",
            "Side impact / heavy side curb into the accumulator region — "
            "the one chain where the ordering is a rules-and-safety matter, "
            "not a lead-time matter.",
            [PathElement("side_panel", "Side impact panel", 1.50, E, S1, 260,
                         1.0, note="sacrificial by rule intent"),
             PathElement("accu_mount", "Accumulator container mount", 2.25,
                         G, S3, 700, 28.0,
                         note="the FoS gap is designed in — but the "
                              "capacity was never pull-tested, so the "
                              "grade is a GUESS and the ±40% band eats "
                              "the gap"),
             PathElement("cell_restraint", "Cell-stack restraint", 2.50, M,
                         S3, 500, 28.0,
                         note="rules-driven sizing, CAD-modelled")],
            designated_fuse_key="side_panel"),
    ]


# --------------------------------------------------------------------------- #
#  Markdown export — the pinnable Fuse Map
# --------------------------------------------------------------------------- #
def render_fusebox_md(paths: list[OverloadPath], audits: list[PathAudit],
                      charter: Optional[dict] = None,
                      frame_tag: str = "") -> str:
    lines = ["# ⛓️ Fusebox — the failure-order audit", ""]
    if charter and charter_intact(charter):
        lines.append(f"*Charter sealed {charter['created_utc']} · "
                     f"confidence {charter['confidence']:.0%} · forbidden ≤ "
                     f"{charter['forbidden_p']:.0%} · sha256 "
                     f"`{charter['seal'][:16]}…`*")
    elif charter:
        lines.append("*⚠️ CHARTER SEAL BROKEN — verdicts below are advisory "
                     "only; re-seal before judging any incident.*")
    else:
        lines.append("*No sealed charter — audits use module defaults "
                     f"(confidence {DEFAULT_CONFIDENCE:.0%}, forbidden ≤ "
                     f"{DEFAULT_FORBIDDEN_P:.0%}).*")
    if frame_tag:
        lines.append(f"*Coordinate convention: {frame_tag}*")
    lines.append("")
    amap = {a.path_key: a for a in audits}
    for p in paths:
        a = amap.get(p.key)
        if a is None:
            continue
        lines += [f"## {p.label} — **{a.verdict.value}**", "",
                  f"> {p.story}", "", a.headline, "",
                  "| element | severity | FoS | grade (±) | P(first) | "
                  "cost | days |",
                  "|---|---|---|---|---|---|---|"]
        for e in p.elements:
            mark = " ⛓️" if e.key == p.designated_fuse_key else ""
            lines.append(
                f"| {e.label}{mark} | "
                f"{e.severity.value if e.severity else '—'} | "
                f"{e.fos:.2f} | {e.grade.value} ±{e.rel_unc:.0%} | "
                f"{a.probs.get(e.key, 0.0):.1%} | "
                f"${e.replace_cost_usd:,.0f} | {e.downtime_days:.1f} |")
        lines += ["",
                  f"Expected overload bill (conditional on the hit): "
                  f"**${a.expected_cost_usd:,.0f} · "
                  f"{a.expected_downtime_days:.1f} days** — the intended "
                  f"fuse alone would cost ${a.fuse_cost_usd:,.0f} · "
                  f"{a.fuse_downtime_days:.1f} days.", ""]
        for b in a.blind_spots:
            lines.append(f"- ⚠️ {b}")
        if a.blind_spots:
            lines.append("")
    lines.append("*Probabilities are first-order statistics of the minimum "
                 "of independent normal capacities, σ from the Proof "
                 "Engine's evidence-grade band law. Deterministic: same "
                 "deck in, same order out.*")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  Self-test — python3 -m suspension.fusebox
# --------------------------------------------------------------------------- #
def _self_test() -> None:
    n = [0]

    def ok(cond, msg):
        n[0] += 1
        if not cond:
            raise AssertionError(f"self-test #{n[0]} failed: {msg}")
        print(f"  ✓ {msg}")

    # pairwise closed form matches quadrature (two elements)
    a = PathElement("a", "A", 1.35, EvidenceGrade.MODELLED,
                    Severity.S1_FUSE_GRADE)
    b = PathElement("b", "B", 1.80, EvidenceGrade.GUESS,
                    Severity.S2_STRUCTURAL)
    p = first_failure_probs([a, b])
    closed = pairwise_first(a.mu, a.sigma, b.mu, b.sigma)
    ok(abs(p["a"] - closed) < 1e-4,
       f"quadrature matches napkin formula ({p['a']:.4f} vs {closed:.4f})")
    ok(abs(sum(p.values()) - 1.0) < 1e-9, "probabilities sum to 1")
    ok(0.20 < p["b"] < 0.30,
       f"GUESS-grade FoS 1.8 beats MODELLED 1.35 to failure "
       f"{p['b']:.0%} of the time — the flagship coin flip")

    # seeded paths audit deterministically; side_accu breaches on the GUESS
    paths = seed_paths()
    audits = [audit_path(pp) for pp in paths]
    amap = {aud.path_key: aud for aud in audits}
    ok(amap["front_curb"].verdict is PathVerdict.COIN_FLIP,
       "front curb chain is a COIN-FLIP at defaults")
    ok(amap["side_accu"].verdict is PathVerdict.BREACH_RISK,
       "GUESS-grade accumulator mount triggers BREACH-RISK")
    ok(amap["side_accu"].forbidden_hits[0][0] == "accu_mount",
       "the breach names the accumulator mount")
    a2 = [audit_path(pp) for pp in seed_paths()]
    ok(all(x.probs == y.probs for x, y in zip(audits, a2)),
       "audit is deterministic (byte-identical probabilities)")

    # sharpening the mount's grade retires the breach — certainty, not metal
    fixed = seed_paths()
    fixed[3].element("accu_mount").grade = EvidenceGrade.MEASURED
    ok(audit_path(fixed[3]).verdict is not PathVerdict.BREACH_RISK,
       "re-grading GUESS→MEASURED retires the breach with zero new metal")

    # prescriptions solve the pairwise equation exactly
    pres = prescribe(paths[0], 0.95)
    ok(any(pr.rival_key == "upright" for pr in pres),
       "the upright is prescribed against")
    pu = next(pr for pr in pres if pr.rival_key == "upright")
    ok(pu.sharpen_rival_to is not None,
       f"measure-don't-machine lever exists (→ {pu.sharpen_rival_to})")
    if pu.raise_rival_fos_to:
        el = paths[0].element("upright")
        fuse = paths[0].element("tie_rod")
        chk = pairwise_first(fuse.mu, fuse.sigma, pu.raise_rival_fos_to,
                             pu.raise_rival_fos_to * el.rel_unc)
        ok(abs(chk - 0.95) < 5e-4,   # prescribed FoS is display-rounded to 3 dp
           f"raise-rival root satisfies the pairwise equation ({chk:.6f})")

    # charter seal + incident judging
    ch = create_charter({pp.key: pp.designated_fuse_key for pp in paths})
    ok(charter_intact(ch), "fresh charter seal verifies")
    v, msg = judge_incident(ch, paths[0], "tie_rod")
    ok(v is IncidentVerdict.AS_DESIGNED and "Free datum" in msg,
       "designated fuse breaking judges AS-DESIGNED with the free datum")
    v, _ = judge_incident(ch, paths[0], "upright")
    ok(v is IncidentVerdict.SURPRISE, "wrong victim judges SURPRISE")
    v, _ = judge_incident(ch, paths[3], "accu_mount")
    ok(v is IncidentVerdict.BREACH, "S3 first judges BREACH")
    ch["confidence"] = 0.51
    try:
        judge_incident(ch, paths[0], "tie_rod")
        ok(False, "tampered charter must refuse")
    except ValueError:
        ok(True, "tampered charter refuses to judge, out loud")

    md = render_fusebox_md(paths, audits, create_charter(
        {pp.key: pp.designated_fuse_key for pp in paths}))
    ok("BREACH-RISK" in md and "sha256" in md and "⛓️" in md,
       "markdown fuse map renders")
    print(f"\nfusebox self-test: all {n[0]} checks passed.")


if __name__ == "__main__":
    _self_test()
