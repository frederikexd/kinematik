# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  Module: earshot — the test-day power audit
# ============================================================================
"""
Earshot — is the answer within earshot of the session you actually have?

THE PROBLEM THIS SOLVES
-----------------------
The Proof Engine ranks evidence actions by uncertainty retired per hour, and
the top of that list is almost always a physical test: an A-B track session,
a coast-down, a tilt test. Every one of those rankings silently assumes the
test WORKS — that a coast-down delivers ±3 %, that an afternoon of A-B laps
can hear a 0.3 s wing. Reality disagrees, in three specific ways, and each
one wastes the scarcest resource a student team has (a track day, a driver,
a charged pack) on an experiment that was mathematically incapable of
answering its question before anyone loaded the trailer:

  1. THE DEAF A-B TEST. The team fits the new wing, runs some laps, swaps
     back, runs some more. The predicted gain is 0.3 s; the driver's
     lap-to-lap sigma is 0.8 s. Detecting that effect at 80 % power needs
     ~112 laps PER CONFIGURATION. The session has 40 laps of pack. The
     experiment was dead at breakfast — and the inevitable "inconclusive"
     result gets read as "the wing doesn't work", which is worse than not
     testing, because now a real gain has been falsely buried. Clinical
     trials have refused to start without this power calculation for fifty
     years; no engineering tool has ever run it for a test day.

  2. THE CONFOUNDED ORDERING. Tires wear, the track rubbers in, the pack
     sags — every session has drift. Run all the A laps then all the B laps
     (which is what tired teams do, because swapping the wing takes twenty
     minutes) and a 0.03 s/lap tire-wear drift puts a 0.6 s bias straight
     into the A−B estimate: TWICE the effect being hunted. The fix is free
     — ABBA blocks cancel linear drift exactly — but only if someone runs
     the arithmetic before the session instead of discovering the confound
     in the data afterwards, when it can no longer be removed.

  3. THE MEASUREMENT THAT TEACHES NOTHING. A tilt test at 8 degrees with a
     half-degree angle error puts ±6 % on CG height from the angle term
     alone — while the ledger's current ESTIMATE band might be ±20 % but an
     upgraded MODELLED CAD rollup is ±10 %. Run the weak version of the test
     and the checkbox culture still logs "CG height: MEASURED ±3 %" — an
     unearned grade upgrade that the whole Proof Engine chain then trusts.
     The instrument arithmetic decides what grade a test can EARN, and a
     planned test whose delivered band is wider than the ledger's current
     band is MOOT: it cannot teach the team anything it doesn't know.

WHAT IT DOES
------------
  * A/B DETECTABILITY. From the predicted effect (the ledger's own objective
    delta, or a declared one) and the noise floor (driver lap sigma — itself
    an evidence-graded quantity that starts as a GUESS and gets measured from
    baseline laps), computes the laps-per-config needed at a chosen
    significance and power, and the MINIMUM DETECTABLE EFFECT of the session
    the team actually has. Session capacity can be derived from the pack:
    usable kWh over kWh per lap — the EV twist on "how long is the day".
    Verdicts: RESOLVABLE / UNDERPOWERED / SWAMPED.
  * THE ORDERING AUDIT. For AABB / ABAB / ABBA the drift-induced bias in the
    A−B estimate is computed EXACTLY for a linear drift (mean lap index of A
    minus mean lap index of B, times drift per lap) and weighed against the
    swap cost each ordering pays. CONFOUNDED is declared when the bias
    rivals what the session can hear.
  * INSTRUMENT RESOLUTION. First-order error propagation from instrument
    specs through the test's own equation (tilt: h = ΔW·L/(W·tanθ);
    coast-down: two-band decel separation of A + B·v²; corner scales) to the
    band the test will actually deliver — and therefore the evidence grade
    it EARNS, which is the grade the Proof Engine ledger is allowed to
    record. SHARPENS when the delivered band beats the ledger's current
    band; MOOT when it doesn't.
  * THE SEALED SESSION SHEET. Ordering, laps per config, alpha, power, the
    sigma and delta used, the MDE, and the abort criterion are sha256-sealed
    before the trailer loads — same discipline as a validation contract.
    Judging afterwards: DETECTED / NOT-DETECTED (with the honest line: at
    your sealed power, a real effect still had a stated chance of hiding —
    a non-detection is priced, never shrugged into "doesn't work") / VOID
    when the session broke its own sealed minimums, said out loud instead
    of quietly judging anyway.

WHY NO ONE HAS BUILT IT
-----------------------
Power analysis lives in statistics tools that have never heard of a car;
data-logger vendors sell hindsight — analysis after the session; CAE vendors
sell solver hours. The a-priori question — CAN this session hear the answer —
needs the predicted effect (the lap model), the noise floor (driver sigma as
an evidence-graded quantity), the session budget (the accumulator), and the
current uncertainty bands (the Proof Engine ledger) in one place. In
KinematiK they already are, so Earshot is a join, not a data-entry burden:
one new number (driver sigma) buys the whole audit.

HONESTY CONTRACT (same as the rest of KinematiK)
------------------------------------------------
Every formula here is the standard textbook form, named in its docstring,
computed deterministically — no Monte Carlo, no fitted constants — so a
stubborn lead can check every verdict by hand. The lap-count formula is the
two-sample normal-approximation power calculation; the drift bias is exact
for a linear drift and says so; instrument propagation is first-order and
lists its partials. Where reality is messier than the model (non-linear
drift, non-normal lap times), the sheet says the assumption out loud.

No streamlit / pandas / plotly imports. Pure stdlib. Unit-testable headless.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Optional

from .proof_engine import EvidenceGrade, Quantity, effective_rel_unc
from .phantom_car import phi, z_from_percentile


# --------------------------------------------------------------------------- #
#  Normal helpers — reuse the Phantom Car's bisection inverse (one Φ, four
#  consumers, on purpose), plus the two-sided alpha form power analysis needs.
# --------------------------------------------------------------------------- #
def z_two_sided(alpha: float) -> float:
    """z such that P(|Z| > z) = alpha. alpha=0.05 → 1.95996…"""
    if not (0.0 < alpha < 1.0):
        raise ValueError("alpha must be in (0, 1)")
    return z_from_percentile((1.0 - alpha / 2.0) * 100.0)


def z_power(power: float) -> float:
    """z such that Φ(z) = power. power=0.80 → 0.84162…"""
    if not (0.5 < power < 1.0):
        raise ValueError("power must be in (0.5, 1) — asking for a coin-flip "
                         "chance of hearing a real effect is not a test plan")
    return z_from_percentile(power * 100.0)


# --------------------------------------------------------------------------- #
#  A/B detectability — the deaf-test detector
# --------------------------------------------------------------------------- #
class ABVerdict(str, Enum):
    RESOLVABLE = "RESOLVABLE"      # the session can hear the predicted effect
    UNDERPOWERED = "UNDERPOWERED"  # a longer session could; this one can't
    SWAMPED = "SWAMPED"            # effect below noise floor; no sane session can


# A session needing more laps than this per config is not a track test, it is
# a research program — the effect is declared SWAMPED and the honest advice is
# to shrink the noise (better driver consistency, a steadier reference lap) or
# grow the effect (test a bigger change), not to book more track days.
SWAMPED_LAP_LIMIT = 400


@dataclass
class ABDesign:
    """
    Two-configuration comparison design, normal approximation, two-sided.

        n_per_config = 2 * ((z_{1-α/2} + z_power) * σ / δ)²        [ceil]
        MDE(n)       = (z_{1-α/2} + z_power) * σ * sqrt(2 / n)

    σ is the lap-to-lap standard deviation of the objective (driver + track
    micro-variation), δ the true effect to detect. Both formulas are the
    standard two-sample z forms found in any experimental-design text; the
    normal approximation is stated as an assumption on the sheet.
    """
    effect_predicted: float          # δ, objective units (e.g. seconds)
    noise_sigma: float               # σ, same units
    alpha: float = 0.05
    power: float = 0.80
    laps_available_per_config: int = 0   # what the session actually offers

    def __post_init__(self):
        if self.noise_sigma <= 0:
            raise ValueError("noise sigma must be positive — a zero-noise "
                             "driver has not been born")

    @property
    def _z_sum(self) -> float:
        return z_two_sided(self.alpha) + z_power(self.power)

    @property
    def laps_needed_per_config(self) -> int:
        d = abs(self.effect_predicted)
        if d == 0.0:
            return SWAMPED_LAP_LIMIT + 1
        return math.ceil(2.0 * (self._z_sum * self.noise_sigma / d) ** 2)

    def mde(self, n_per_config: Optional[int] = None) -> float:
        """Minimum detectable effect for a given session length."""
        n = self.laps_available_per_config if n_per_config is None else n_per_config
        if n <= 0:
            return math.inf
        return self._z_sum * self.noise_sigma * math.sqrt(2.0 / n)

    @property
    def verdict(self) -> ABVerdict:
        need = self.laps_needed_per_config
        if need > SWAMPED_LAP_LIMIT:
            return ABVerdict.SWAMPED
        if self.laps_available_per_config >= need:
            return ABVerdict.RESOLVABLE
        return ABVerdict.UNDERPOWERED

    @property
    def miss_probability(self) -> float:
        """
        Probability a REAL effect of the predicted size goes undetected in the
        session as booked — the number that turns "inconclusive" from a shrug
        into a price. 1 − achieved power, where achieved power is
        Φ(δ / (σ·√(2/n)) − z_{1-α/2}); exact under the same normal model.
        """
        n = self.laps_available_per_config
        if n <= 0:
            return 1.0
        ncp = abs(self.effect_predicted) / (self.noise_sigma * math.sqrt(2.0 / n))
        achieved = phi(ncp - z_two_sided(self.alpha))
        return max(0.0, min(1.0, 1.0 - achieved))

    def as_dict(self) -> dict:
        d = asdict(self)
        d.update(laps_needed_per_config=self.laps_needed_per_config,
                 mde_at_available=self.mde(),
                 verdict=self.verdict.value,
                 miss_probability=self.miss_probability)
        return d


def laps_from_pack(pack_kwh: float, usable_frac: float, kwh_per_lap: float,
                   reserve_kwh: float = 0.0, configs: int = 2) -> int:
    """
    The EV session budget: laps per configuration the pack can actually feed.
    floor((pack·usable − reserve) / kWh-per-lap / configs). kWh per lap is the
    same channel the endurance energy budget runs on — one ledger, and the
    test plan is spent in the same currency the race is.
    """
    if kwh_per_lap <= 0 or configs <= 0:
        return 0
    usable = pack_kwh * max(0.0, min(1.0, usable_frac)) - max(0.0, reserve_kwh)
    if usable <= 0:
        return 0
    return int(usable / kwh_per_lap / configs)


# --------------------------------------------------------------------------- #
#  The ordering audit — drift confounds, computed instead of folklore
# --------------------------------------------------------------------------- #
@dataclass
class DriftSource:
    """A session drift, linear in lap count, in objective units per lap."""
    key: str
    label: str
    rate_per_lap: float          # objective units / lap (sign = direction)
    note: str = ""


DEFAULT_DRIFTS: list[DriftSource] = [
    DriftSource("tire_wear", "Tire wear", +0.030,
                "Autocross-compound falloff; a hot set gives up lap time "
                "steadily. Measure from a long baseline stint to upgrade."),
    DriftSource("track_rubber", "Track rubbering in", -0.020,
                "A green lot gains grip as rubber goes down — laps get "
                "FASTER, which flatters whichever config runs second."),
    DriftSource("pack_sag", "Pack voltage sag", +0.010,
                "Lower open-circuit voltage late in the session costs "
                "straightline speed at the power limit."),
    DriftSource("driver_learning", "Driver learning the course", -0.040,
                "The strongest early-session drift there is; decays after "
                "~10 familiarisation laps, which is why the sheet insists "
                "on burn-in laps that count for nobody."),
]


def ordering_bias(ordering: str, n_per_config: int, drift_per_lap: float) -> float:
    """
    EXACT bias a linear drift injects into the A−B mean difference, for the
    three orderings teams actually use. Bias = drift · (mean lap index of A −
    mean lap index of B). Hand-checkable:

      AABB : A on laps 1..n, B on n+1..2n  → index gap −n   → bias −d·n
      ABAB : strict alternation           → index gap −1   → bias −d
      ABBA : blocks of 4 (A,B,B,A)        → gap 0 in every block → bias 0

    Only linear drift cancels exactly in ABBA; the sheet states that
    assumption. A negative bias means the drift flatters B, positive
    flatters A — the sign is reported because it tells you WHICH config the
    session lied for.
    """
    o = ordering.upper()
    n = int(n_per_config)
    if n <= 0:
        return 0.0
    if o == "AABB":
        return -drift_per_lap * n
    if o == "ABAB":
        return -drift_per_lap * 1.0
    if o == "ABBA":
        return 0.0
    raise ValueError(f"unknown ordering {ordering!r} (use AABB, ABAB, ABBA)")


def swap_count(ordering: str, n_per_config: int) -> int:
    """
    Configuration changes the ordering costs — the honest price of drift
    cancellation, because a wing swap is twenty minutes of daylight.
    AABB pays 1; ABAB pays 2n−1; ABBA pays 2 per block and nothing at the
    A→A block boundaries (which is exactly why it cancels drift cheaply).
    Computed by literally walking the sequence, so it cannot be wrong.
    """
    seq = build_sequence(ordering, n_per_config)
    return sum(1 for i in range(1, len(seq)) if seq[i] != seq[i - 1])


def build_sequence(ordering: str, n_per_config: int) -> str:
    """The literal lap sequence, e.g. ABBA·n/2 blocks → 'ABBAABBA…'."""
    o, n = ordering.upper(), int(n_per_config)
    if o == "AABB":
        return "A" * n + "B" * n
    if o == "ABAB":
        return "AB" * n
    if o == "ABBA":
        blocks, rem = divmod(n, 2)
        seq = "ABBA" * blocks
        if rem:                       # odd n: one trailing AB pair, declared
            seq += "AB"
        return seq
    raise ValueError(f"unknown ordering {ordering!r}")


class OrderingVerdict(str, Enum):
    CLEAN = "CLEAN"                # bias ≪ what the session can hear
    BIASED = "BIASED"              # bias comparable to the MDE — noted
    CONFOUNDED = "CONFOUNDED"      # bias rivals or exceeds the hunted effect


@dataclass
class OrderingFinding:
    ordering: str
    net_bias: float                # objective units, sum over drift sources
    per_source: dict               # key -> bias contribution
    swaps: int
    verdict: OrderingVerdict
    note: str = ""


def audit_orderings(design: ABDesign,
                    drifts: Optional[list[DriftSource]] = None,
                    n_per_config: Optional[int] = None) -> list[OrderingFinding]:
    """
    Weigh every standard ordering against the session's own drift model and
    the effect being hunted. Verdict thresholds are stated, not tuned:
    CONFOUNDED when |bias| ≥ half the predicted effect (the estimate can be
    moved to the wrong side of zero by drift alone); BIASED when it exceeds
    a fifth of the MDE; CLEAN below that.
    """
    drifts = DEFAULT_DRIFTS if drifts is None else drifts
    n = design.laps_available_per_config if n_per_config is None else n_per_config
    n = max(1, int(n))
    d_eff = abs(design.effect_predicted)
    mde = design.mde(n)
    out: list[OrderingFinding] = []
    for o in ("AABB", "ABAB", "ABBA"):
        per = {s.key: ordering_bias(o, n, s.rate_per_lap) for s in drifts}
        net = sum(per.values())
        if d_eff > 0 and abs(net) >= 0.5 * d_eff:
            v = OrderingVerdict.CONFOUNDED
            note = (f"Drift alone moves the A−B estimate by {net:+.3f} — "
                    f"{abs(net) / d_eff:.1f}× half the effect being hunted. "
                    "This ordering cannot testify.")
        elif math.isfinite(mde) and abs(net) >= 0.2 * mde:
            v = OrderingVerdict.BIASED
            note = (f"Bias {net:+.3f} is a visible fraction of the MDE "
                    f"({mde:.3f}); report it alongside any result.")
        else:
            v = OrderingVerdict.CLEAN
            note = "Linear drift cancels to below reporting threshold."
        out.append(OrderingFinding(o, net, per, swap_count(o, n), v, note))
    return out


# --------------------------------------------------------------------------- #
#  Instrument resolution — the band a parameter test will actually deliver
# --------------------------------------------------------------------------- #
class ResolutionVerdict(str, Enum):
    SHARPENS = "SHARPENS"   # delivered band beats the ledger's current band
    MOOT = "MOOT"           # it doesn't — the test cannot teach you anything


@dataclass
class DeliveredResolution:
    """
    What a planned parameter test will really hand back, from instrument
    arithmetic — and therefore the evidence grade it EARNS. The earned grade
    is the tightest grade whose base band covers the delivered one; a
    checkbox may not claim better.
    """
    test_key: str
    test_label: str
    parameter_key: str            # ledger channel it measures
    delivered_rel_unc: float      # fraction, e.g. 0.027 = ±2.7 %
    terms: dict                   # name -> rel-unc contribution (RSS members)
    earned_grade: EvidenceGrade
    current_rel_unc: Optional[float] = None   # ledger band, for the verdict
    verdict: Optional[ResolutionVerdict] = None
    note: str = ""

    def judge_against(self, current_rel_unc: float) -> "DeliveredResolution":
        self.current_rel_unc = current_rel_unc
        if self.delivered_rel_unc < current_rel_unc:
            self.verdict = ResolutionVerdict.SHARPENS
            self.note = (f"±{self.delivered_rel_unc * 100:.1f} % beats the "
                         f"ledger's ±{current_rel_unc * 100:.1f} % — worth "
                         f"the trip; record as {self.earned_grade.value}.")
        else:
            self.verdict = ResolutionVerdict.MOOT
            self.note = (f"±{self.delivered_rel_unc * 100:.1f} % is no better "
                         f"than the ±{current_rel_unc * 100:.1f} % already on "
                         "the ledger. As planned, this test cannot teach the "
                         "team anything — fix the dominant term "
                         f"({max(self.terms, key=self.terms.get)}) or skip it.")
        return self


def _earned_grade(rel_unc: float) -> EvidenceGrade:
    """Tightest grade whose base band covers the delivered precision."""
    for g in (EvidenceGrade.VERIFIED, EvidenceGrade.MEASURED,
              EvidenceGrade.MODELLED, EvidenceGrade.ESTIMATE):
        if rel_unc <= g.base_rel_unc:
            return g
    return EvidenceGrade.GUESS


def resolve_tilt_cg(mass_kg: float, wheelbase_mm: float, cg_height_mm: float,
                    tilt_deg: float, scale_res_kg: float,
                    angle_sigma_deg: float, repeats: int = 1) -> DeliveredResolution:
    """
    Tilt / axle-lift CG-height test:  h = ΔW · L / (W · tanθ)
    where ΔW = W · h · tanθ / L is the expected weight transfer read off the
    scales. First-order relative band, terms RSS'd:

        angle term : σθ / (sinθ · cosθ)   [d(ln tanθ)/dθ = 1/(sinθ cosθ)]
        scale term : σ_ΔW / ΔW, with σ_ΔW = scale resolution (per reading;
                     both axles read, so √2 · res is used)

    The angle term is why a shallow tilt cannot measure CG height: at 8° it
    contributes ~7.3 × σθ[rad]; at 20°, ~3.1 ×. Repeats divide by √repeats
    (independent re-lifts, both operators re-zeroing).
    """
    th = math.radians(tilt_deg)
    if not (0 < tilt_deg < 45):
        raise ValueError("tilt angle must be in (0, 45) degrees")
    dW = mass_kg * cg_height_mm * math.tan(th) / wheelbase_mm
    term_angle = math.radians(angle_sigma_deg) / (math.sin(th) * math.cos(th))
    term_scale = (math.sqrt(2.0) * scale_res_kg) / dW if dW > 0 else math.inf
    rel = math.sqrt(term_angle ** 2 + term_scale ** 2) / math.sqrt(max(1, repeats))
    return DeliveredResolution(
        "tilt_cg", f"Tilt test at {tilt_deg:.0f}°", "cg_z_mm", rel,
        {"angle": term_angle / math.sqrt(max(1, repeats)),
         "scale_resolution": term_scale / math.sqrt(max(1, repeats))},
        _earned_grade(rel))


def resolve_coastdown(mass_kg: float, cda_m2: float, air_density: float,
                      v_high_ms: float, v_low_ms: float,
                      speed_sigma_ms: float, band_seconds: float,
                      runs_paired: int = 1) -> DeliveredResolution:
    """
    Coast-down CdA by two-band decel separation of F = A + B·v²:

        B = m·(a_h − a_l) / (v_h² − v_l²),   CdA = 2B / ρ

    Each band's mean decel is Δv over the band duration T; with speed noise
    σ_v at each end, σ_a = √2·σ_v / T. Bands are independent, so

        σ_B / B = √(σ_ah² + σ_al²) / (a_h − a_l)

    evaluated at the drag decel the declared CdA itself implies (the sheet is
    judging the PLAN, so the plan's own numbers set the scale). Paired
    opposite-direction runs cancel wind and grade to first order AND divide
    the band by √runs; unpaired runs cancel nothing, and the sheet says so.
    """
    if v_high_ms <= v_low_ms:
        raise ValueError("high band must be faster than low band")
    a_h = 0.5 * air_density * cda_m2 * v_high_ms ** 2 / mass_kg
    a_l = 0.5 * air_density * cda_m2 * v_low_ms ** 2 / mass_kg
    sep = a_h - a_l
    sigma_a = math.sqrt(2.0) * speed_sigma_ms / max(band_seconds, 1e-9)
    rel = (math.sqrt(2.0) * sigma_a / sep) / math.sqrt(max(1, runs_paired)) \
        if sep > 0 else math.inf
    return DeliveredResolution(
        "coastdown", "Coast-down (two-band, paired directions)",
        "cda_m2", rel,
        {"speed_noise": rel,        # single dominant term in this surrogate
         },
        _earned_grade(rel))


def resolve_corner_scales(mass_kg: float, pad_res_kg: float,
                          repeats: int = 1) -> DeliveredResolution:
    """
    Total mass from four pads: σ_total = √4 · pad resolution (independent
    pad quantisation, RSS), over √repeats for full re-zeroed re-weighs.
    """
    sigma = 2.0 * pad_res_kg / math.sqrt(max(1, repeats))
    rel = sigma / mass_kg if mass_kg > 0 else math.inf
    return DeliveredResolution(
        "corner_scales", "Corner scales (4 pads)", "mass_kg", rel,
        {"pad_resolution": rel}, _earned_grade(rel))


# --------------------------------------------------------------------------- #
#  The sealed session sheet — pre-registration for a track day
# --------------------------------------------------------------------------- #
class SessionVerdict(str, Enum):
    DETECTED = "DETECTED"
    NOT_DETECTED = "NOT_DETECTED"
    VOID = "VOID"


@dataclass
class SessionSheet:
    """
    Everything that must not move after the result exists, sealed before the
    trailer loads: the design (δ, σ, α, power, ordering, laps), the MDE, the
    burn-in laps that count for nobody, and the abort criterion. Result
    fields are filled at judging and never touch sealed ones — editing a
    sealed field breaks the seal, and a broken seal refuses judgment out
    loud, exactly like a validation contract.
    """
    title: str
    objective_label: str
    unit: str
    effect_predicted: float
    noise_sigma: float
    noise_sigma_grade: str            # evidence grade of σ itself — stated
    alpha: float
    power: float
    ordering: str
    laps_per_config: int
    burn_in_laps: int
    mde: float
    drift_bias_declared: float        # net linear-drift bias of the ordering
    abort_note: str = ""
    assumption_note: str = ("Normal-approximation two-sample design; drift "
                            "cancellation exact only for linear drift.")
    created_utc: str = ""
    seal: str = ""
    # ---- result block (mutable after sealing) ----
    mean_a: Optional[float] = None
    mean_b: Optional[float] = None
    laps_run_per_config: Optional[int] = None
    judged_verdict: Optional[str] = None
    judged_note: str = ""

    _SEALED = ("title", "objective_label", "unit", "effect_predicted",
               "noise_sigma", "noise_sigma_grade", "alpha", "power",
               "ordering", "laps_per_config", "burn_in_laps", "mde",
               "drift_bias_declared", "abort_note", "assumption_note",
               "created_utc")

    def _payload(self) -> str:
        return json.dumps({k: getattr(self, k) for k in self._SEALED},
                          sort_keys=True, separators=(",", ":"))

    def compute_seal(self) -> str:
        return hashlib.sha256(self._payload().encode()).hexdigest()

    def verify_seal(self) -> bool:
        return bool(self.seal) and self.seal == self.compute_seal()

    def as_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "SessionSheet":
        return SessionSheet(**{k: v for k, v in d.items()
                               if k in SessionSheet.__dataclass_fields__})


def create_sheet(title: str, design: ABDesign, ordering: str,
                 objective_label: str = "lap time", unit: str = "s",
                 burn_in_laps: int = 3, sigma_grade: str = "estimate",
                 drifts: Optional[list[DriftSource]] = None,
                 abort_note: str = "") -> SessionSheet:
    n = design.laps_available_per_config
    bias = sum(ordering_bias(ordering, n, s.rate_per_lap)
               for s in (DEFAULT_DRIFTS if drifts is None else drifts))
    sheet = SessionSheet(
        title=title, objective_label=objective_label, unit=unit,
        effect_predicted=design.effect_predicted,
        noise_sigma=design.noise_sigma, noise_sigma_grade=sigma_grade,
        alpha=design.alpha, power=design.power, ordering=ordering.upper(),
        laps_per_config=n, burn_in_laps=int(burn_in_laps),
        mde=design.mde(n), drift_bias_declared=bias, abort_note=abort_note,
        created_utc=_dt.datetime.now(_dt.timezone.utc)
                        .isoformat(timespec="seconds").replace("+00:00", "Z"))
    sheet.seal = sheet.compute_seal()
    return sheet


def judge_session(sheet: SessionSheet, mean_a: float, mean_b: float,
                  laps_run_per_config: int) -> SessionSheet:
    """
    Fill the result block, never the sealed one. VOID when the seal is broken
    or the session ran fewer laps than sealed (a shortened session has a
    larger MDE than it promised — judging it as sealed would be quietly
    moving the goalposts in the session's favour). Detection is the sealed
    two-sided z test; a NOT_DETECTED verdict carries the sealed miss
    probability so "inconclusive" arrives priced.
    """
    sheet.mean_a, sheet.mean_b = mean_a, mean_b
    sheet.laps_run_per_config = int(laps_run_per_config)
    if not sheet.verify_seal():
        sheet.judged_verdict = SessionVerdict.VOID.value
        sheet.judged_note = ("Seal broken — a sealed field was edited after "
                            "creation. This sheet refuses to judge.")
        return sheet
    if laps_run_per_config < sheet.laps_per_config:
        sheet.judged_verdict = SessionVerdict.VOID.value
        sheet.judged_note = (
            f"Session ran {laps_run_per_config}/{sheet.laps_per_config} sealed "
            "laps per config. Its real MDE is wider than the sealed one; "
            "re-seal a design for the session you can actually run.")
        return sheet
    diff = mean_a - mean_b
    se = sheet.noise_sigma * math.sqrt(2.0 / laps_run_per_config)
    z = abs(diff) / se if se > 0 else math.inf
    if z >= z_two_sided(sheet.alpha):
        sheet.judged_verdict = SessionVerdict.DETECTED.value
        sheet.judged_note = (
            f"|A−B| = {abs(diff):.3f} {sheet.unit} (z = {z:.2f}) clears the "
            f"sealed two-sided threshold at α = {sheet.alpha}. Declared drift "
            f"bias of this ordering: {sheet.drift_bias_declared:+.3f} "
            f"{sheet.unit} — report it next to the result.")
    else:
        design = ABDesign(sheet.effect_predicted, sheet.noise_sigma,
                          sheet.alpha, sheet.power, laps_run_per_config)
        sheet.judged_verdict = SessionVerdict.NOT_DETECTED.value
        sheet.judged_note = (
            f"|A−B| = {abs(diff):.3f} {sheet.unit} (z = {z:.2f}) does not "
            f"clear α = {sheet.alpha}. NOT 'the change does nothing': at the "
            f"sealed power, a real {sheet.effect_predicted:+.3f} {sheet.unit} "
            f"effect still had a {design.miss_probability * 100:.0f} % chance "
            "of hiding in this session. Absence of evidence, priced.")
    return sheet


# --------------------------------------------------------------------------- #
#  Markdown export — the pinnable run sheet
# --------------------------------------------------------------------------- #
def render_session_md(sheet: SessionSheet,
                      orderings: Optional[list[OrderingFinding]] = None,
                      frame_note: str = "") -> str:
    seq = build_sequence(sheet.ordering, sheet.laps_per_config)
    lines = [
        f"# 🎙️ Earshot session sheet — {sheet.title}",
        "",
        f"*Sealed {sheet.created_utc} · sha256 `{sheet.seal[:16]}…`*"
        + (f" · frame: {frame_note}" if frame_note else ""),
        "",
        f"**Question:** does the change move {sheet.objective_label} by the "
        f"predicted {sheet.effect_predicted:+.3f} {sheet.unit}?",
        "",
        f"| | |",
        f"|---|---|",
        f"| Noise floor σ ({sheet.noise_sigma_grade}) | "
        f"{sheet.noise_sigma:.3f} {sheet.unit}/lap |",
        f"| Design | α = {sheet.alpha}, power = {sheet.power:.0%} |",
        f"| Laps per config (sealed minimum) | {sheet.laps_per_config} |",
        f"| Burn-in laps (count for nobody) | {sheet.burn_in_laps} |",
        f"| Minimum detectable effect | {sheet.mde:.3f} {sheet.unit} |",
        f"| Ordering | {sheet.ordering} — declared linear-drift bias "
        f"{sheet.drift_bias_declared:+.3f} {sheet.unit} |",
        "",
        f"**Run order:** `{seq}`",
        "",
        f"**Abort criterion (sealed):** {sheet.abort_note or '—'}",
        "",
        f"> {sheet.assumption_note}",
    ]
    if orderings:
        lines += ["", "## Ordering audit", "",
                  "| Ordering | Net drift bias | Swaps | Verdict |",
                  "|---|---|---|---|"]
        for f in orderings:
            lines.append(f"| {f.ordering} | {f.net_bias:+.3f} {sheet.unit} | "
                         f"{f.swaps} | {f.verdict.value} |")
    if sheet.judged_verdict:
        lines += ["", "## Judgment", "",
                  f"**{sheet.judged_verdict}** — {sheet.judged_note}"]
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  Self-test — every number checkable by hand
# --------------------------------------------------------------------------- #
def _self_test() -> None:
    ok = lambda c, m: (_ for _ in ()).throw(AssertionError(m)) if not c else None

    # --- power arithmetic, the flagship number -----------------------------
    d = ABDesign(effect_predicted=0.30, noise_sigma=0.80,
                 alpha=0.05, power=0.80, laps_available_per_config=20)
    ok(abs(z_two_sided(0.05) - 1.959964) < 1e-4, "z_alpha")
    ok(abs(z_power(0.80) - 0.841621) < 1e-4, "z_power")
    ok(d.laps_needed_per_config == 112,
       f"0.3 s vs 0.8 s should need 112 laps/config, got {d.laps_needed_per_config}")
    ok(abs(d.mde(20) - 0.70875) < 1e-3, f"MDE(20) {d.mde(20)}")
    ok(d.verdict is ABVerdict.UNDERPOWERED, "verdict")
    # ncp = 0.3/(0.8·√(2/20)) = 1.1858; Φ(1.1858−1.95996) = 0.2194 → miss 78 %
    ok(abs(d.miss_probability - 0.78057) < 1e-3,
       f"miss prob {d.miss_probability}")

    big = ABDesign(1.0, 0.8, 0.05, 0.80, 20)
    ok(big.laps_needed_per_config == 11 and big.verdict is ABVerdict.RESOLVABLE,
       "1.0 s effect should be resolvable in 20 laps")
    tiny = ABDesign(0.05, 0.8, 0.05, 0.80, 20)
    ok(tiny.verdict is ABVerdict.SWAMPED, "0.05 s effect is swamped")

    # --- pack-limited session budget ---------------------------------------
    ok(laps_from_pack(7.0, 0.9, 0.14, reserve_kwh=0.5, configs=2) == 20,
       "pack laps")

    # --- ordering bias, exact ----------------------------------------------
    ok(abs(ordering_bias("AABB", 20, 0.03) + 0.60) < 1e-12, "AABB bias")
    ok(abs(ordering_bias("ABAB", 20, 0.03) + 0.03) < 1e-12, "ABAB bias")
    ok(ordering_bias("ABBA", 20, 0.03) == 0.0, "ABBA cancels")
    ok(swap_count("AABB", 20) == 1, "AABB swaps")
    ok(swap_count("ABAB", 20) == 39, "ABAB swaps")
    ok(build_sequence("ABBA", 4) == "ABBAABBA", "ABBA sequence")
    finds = {f.ordering: f for f in audit_orderings(d)}
    ok(finds["AABB"].verdict is OrderingVerdict.CONFOUNDED, "AABB confounded")
    ok(finds["ABBA"].verdict is OrderingVerdict.CLEAN, "ABBA clean")

    # --- instrument resolution ---------------------------------------------
    shallow = resolve_tilt_cg(250, 1550, 300, 8.0, 0.5, 0.5)
    steep = resolve_tilt_cg(250, 1550, 300, 20.0, 0.5, 0.5)
    ok(steep.delivered_rel_unc < shallow.delivered_rel_unc,
       "steeper tilt must resolve better")
    # hand-check the angle term at 20°: 0.008727 / (sin20·cos20) = 0.02715
    ok(abs(steep.terms["angle"] - 0.027153) < 1e-3, "angle term at 20°")
    moot = shallow.judge_against(0.05)
    ok(moot.verdict is ResolutionVerdict.MOOT, "8° tilt vs ±5 % ledger: MOOT")
    sharp = steep.judge_against(0.10)
    ok(sharp.verdict is ResolutionVerdict.SHARPENS, "20° tilt vs ±10 %: SHARPENS")
    scales = resolve_corner_scales(250, 0.5)
    ok(abs(scales.delivered_rel_unc - 0.004) < 1e-9, "scales 2·0.5/250")
    ok(scales.earned_grade is EvidenceGrade.VERIFIED, "±0.4 % earns verified")

    cd = resolve_coastdown(250, 1.1, 1.204, 22.0, 10.0, 0.14, 3.0, runs_paired=4)
    ok(0 < cd.delivered_rel_unc < 1.0, "coastdown returns a finite band")

    # --- seal + judge -------------------------------------------------------
    d2 = ABDesign(0.9, 0.8, 0.05, 0.80, 14)
    ok(d2.verdict is ABVerdict.RESOLVABLE, "0.9 s in 14 laps resolvable")
    sheet = create_sheet("Rear-wing Gurney A/B", d2, "ABBA",
                         abort_note="Abort if rain or σ(first 5 A laps) > 1.2 s")
    ok(sheet.verify_seal(), "fresh seal verifies")
    judged = judge_session(sheet, mean_a=54.90, mean_b=54.00,
                           laps_run_per_config=14)
    ok(judged.judged_verdict == "DETECTED", f"judged {judged.judged_note}")

    short = create_sheet("short", ABDesign(0.9, 0.8, 0.05, 0.8, 14), "ABBA")
    judge_session(short, 54.9, 54.0, laps_run_per_config=9)
    ok(short.judged_verdict == "VOID", "short session is VOID")

    tampered = create_sheet("t", ABDesign(0.9, 0.8, 0.05, 0.8, 14), "ABBA")
    tampered.effect_predicted = 0.2          # move the goalposts…
    judge_session(tampered, 54.9, 54.0, 14)
    ok(tampered.judged_verdict == "VOID", "tampered sheet refuses to judge")

    md = render_session_md(judged, audit_orderings(d2))
    ok("DETECTED" in md and "ABBA" in md and "sha256" in md, "markdown render")

    print("earshot self-test: ALL CHECKS PASSED")
    print(f"  the flagship number: 0.30 s effect vs 0.80 s driver sigma "
          f"needs {d.laps_needed_per_config} laps per config; "
          f"20 laps can hear {d.mde(20):.2f} s; "
          f"AABB drift bias {ordering_bias('AABB', 20, 0.03):+.2f} s; "
          f"miss probability as booked {d.miss_probability:.0%}")


if __name__ == "__main__":
    _self_test()
