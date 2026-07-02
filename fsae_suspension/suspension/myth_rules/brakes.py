# ============================================================================
#  KinematiK — Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
# ============================================================================
"""Brakes myth rules. Context (optional): dict with brake-thermal numbers.
Most claims here are about energy and thermal physics that hold regardless of
the specific rotor, with honest pointers to the brake_thermal model for sizing."""
from __future__ import annotations
from typing import Any, Optional
from ..mythbuster import CheckOutcome, ParsedClaim, Rule, Verdict, register


# Bigger rotor = more braking force
def _r_bigger_rotor_force(claim: ParsedClaim, context: Any) -> Optional[CheckOutcome]:
    if not (claim.has("bigger rotor", "larger rotor", "bigger disc", "bigger disk",
                      "rotor size", "rotor")
            and claim.has("more braking", "stops faster", "stop faster", "more force",
                          "shorter stop", "brake harder", "faster", "quicker")):
        return None
    return CheckOutcome(
        Verdict.MYTH,
        ("Peak braking is limited by TYRE grip, not rotor size. Once the tyres are at "
         "the friction limit (or ABS/threshold), a bigger rotor can't shorten the "
         "stop. What a bigger/heavier rotor buys is THERMAL capacity \u2014 more mass "
         "and area to absorb and reject heat over repeated stops without fade. So size "
         "rotors for the endurance heat load (use the brake-thermal model), not for "
         "peak deceleration, which your tyres already cap."),
        provenance="decel capped by tyre \u03bc; rotor size = thermal capacity")
_r_bigger_rotor_force.reference_claim = "A bigger brake rotor makes the car stop faster."


# Braking energy / heat
def _r_brake_energy(claim: ParsedClaim, context: Any) -> Optional[CheckOutcome]:
    if not (claim.has("brake", "braking") and
            claim.has("heat", "energy", "temperature", "thermal", "fade")):
        return None
    return CheckOutcome(
        Verdict.DEPENDS,
        ("Braking converts the car's kinetic energy (\u00bdmv\u00b2) to heat in the "
         "rotors each stop \u2014 so heat scales with the SQUARE of entry speed and "
         "linearly with mass. The sizing question (will it fade over an endurance "
         "stint?) depends on rotor mass, area, cooling and stop frequency, which is "
         "exactly what the brake-thermal model computes. On a regen-braking EV, "
         "energy the motor recovers never reaches the rotors \u2014 size for the heat "
         "that remains AFTER regen, not total. Run the thermal model with your stop "
         "schedule."),
        provenance="Q = \u00bdmv\u00b2 per stop; needs brake_thermal sizing")
_r_brake_energy.reference_claim = "Brake heat is about the same at any speed."


# Brake bias
def _r_brake_bias(claim: ParsedClaim, context: Any) -> Optional[CheckOutcome]:
    if not (claim.has("brake bias", "brake balance", "bias") and
            claim.has("50", "even", "equal", "centre", "center", "middle")):
        return None
    return CheckOutcome(
        Verdict.MYTH,
        ("Brake bias is NOT 50/50. Under braking the car pitches forward and load "
         "transfers onto the front tyres, so the fronts can carry more braking force "
         "before locking \u2014 FSAE cars typically run roughly 60\u201370% front. The "
         "exact split depends on CG height, wheelbase and decel (the same load-"
         "transfer physics as cornering). Set it from your weight distribution and "
         "transfer, then fine-tune so front and rear approach lock together."),
        provenance="forward load transfer \u2192 front-biased; ~60-70% front typical")
_r_brake_bias.reference_claim = "Brake bias should be 50/50 front to rear."


# --------------------------------------------------------------------------- #
#  Throttle return-spring redundancy — the exact assumption the pedal-box lead
#  must never make silently: "the two springs are identical, so if one lets go
#  the other is fine." The FSAE rule is single-fault tolerance, and identical
#  PART is not identical DUTY (arms/preloads differ), nor does it survive a
#  weaker or differently-mounted backup. When the app passes live throttle-return
#  context (a ReturnRedundancyResult, or the springs+resistance to build one),
#  this checks the CLAIM against the real numbers instead of reciting theory.
# --------------------------------------------------------------------------- #
def _throttle_result_from_context(context: Any):
    """Return a ReturnRedundancyResult from whatever the app handed us, or None.

    Accepts (in order):
      * a ReturnRedundancyResult directly (has .verdict + .worst_case),
      * a dict slice with {"return_result": <result>},
      * a dict slice with {"springs": [...], "resistance": <ReturnResistance>,
        "margin_target": <float>} to compute one on the spot.
    Never fabricates springs — if there's nothing real to check, returns None and
    the rule answers on physics alone (honestly, without pretending to have data).
    """
    if context is None:
        return None
    # unwrap a discipline-keyed bundle if the app passed the whole thing
    ctx = context
    if isinstance(ctx, dict) and "brakes" in ctx and not (
            "return_result" in ctx or "springs" in ctx):
        ctx = ctx["brakes"]

    # a result object passed straight through
    if hasattr(ctx, "verdict") and hasattr(ctx, "worst_case"):
        return ctx
    if isinstance(ctx, dict):
        if ctx.get("return_result") is not None:
            rr = ctx["return_result"]
            if hasattr(rr, "verdict") and hasattr(rr, "worst_case"):
                return rr
        springs = ctx.get("springs")
        if springs:
            try:
                from ..throttle_return import (check_return_redundancy,
                                               ReturnResistance)
                res = ctx.get("resistance")
                if isinstance(res, dict):
                    res = ReturnResistance(**res)
                return check_return_redundancy(
                    springs, res,
                    margin_target=float(ctx.get("margin_target", 1.0)))
            except Exception:
                return None
    return None


def _r_throttle_identical_backup(claim: ParsedClaim, context: Any) -> Optional[CheckOutcome]:
    # Fire on claims that the redundancy is fine BECAUSE the springs are the same,
    # or that a single/duplicate spring is enough.
    if not claim.has("throttle", "return spring", "return springs", "pedal spring"):
        # allow "backup spring identical" phrasing without the word throttle
        if not claim.has("backup spring", "second spring", "return"):
            return None
    talks_redundancy = claim.has(
        "identical", "same spring", "same as", "matches", "match the primary",
        "one is enough", "single spring", "duplicate", "copy", "backup is fine",
        "still closes", "still returns", "if one", "one fails", "one unhooks",
        "one breaks", "one spring")
    if not talks_redundancy:
        return None

    rr = _throttle_result_from_context(context)
    if rr is not None:
        # Check the CLAIM against the live model.
        verdict = str(getattr(rr, "verdict", "")).upper()
        worst = getattr(rr, "worst_case", "the worst single-failure case")
        margin = getattr(rr, "worst_margin", float("nan"))
        # Translate the internal case label into plain English: "without 'primary'"
        # -> "if the primary spring fails".
        plain = worst
        if isinstance(worst, str) and worst.startswith("without '") and worst.endswith("'"):
            plain = f"if the {worst[len('without \''):-1]} spring lets go"
        if verdict == "PASS":
            return CheckOutcome(
                Verdict.TRUE,
                (f"Yes — your throttle still closes with either spring gone. Even in "
                 f"the worst case ({plain}) the remaining spring has enough force to "
                 f"shut it, against friction and cable drag. One thing to know: this "
                 f"works because each spring was checked on its own real mounting, not "
                 f"because they're 'identical' — so if you change either spring's arm "
                 f"or preload, run this again."),
                provenance=f"check_return_redundancy PASS; worst {worst} margin {margin:.2f}")
        if verdict in ("TIGHT", "FAIL"):
            v = Verdict.MYTH if verdict == "FAIL" else Verdict.DEPENDS
            if verdict == "FAIL":
                lead = (f"No — and this matters. With your current numbers, {plain} "
                        f"the throttle would NOT close on its own. ")
            else:
                lead = (f"Barely — it closes {plain}, but with almost no margin, so "
                        f"a sticky pivot in the car could hang it. ")
            return CheckOutcome(
                v,
                (lead
                 + "The catch with 'identical springs': the tool doesn't trust that. "
                 "It removes each spring in turn and checks whether the one left can "
                 "still shut the throttle against friction and cable drag, on its "
                 "actual arm and preload. Here the surviving spring comes up short. "
                 "Fix it by adding preload or a stiffer spring, or by cutting friction "
                 "and cable drag — then re-run."),
                provenance=f"check_return_redundancy {verdict}; worst {worst} margin {margin:.2f}")

    # No live data — answer on the rule + physics, honestly labelled.
    return CheckOutcome(
        Verdict.DEPENDS,
        ("The FSAE requirement is single-fault tolerance: the throttle must return to "
         "closed with ANY ONE component failed, including one return spring unhooking. "
         "'The springs are identical so the backup is fine' is not sufficient reasoning "
         "\u2014 two springs of the same part still differ in moment arm and preload as "
         "mounted, and a backup that's weaker, shorter-armed or draggier can fail to "
         "return the throttle alone. Don't assume; CHECK each single-spring-failure "
         "case. KinematiK's Brakes \u25b8 Pedal box & throttle view computes exactly "
         "this \u2014 net closing torque with each spring removed \u2014 and won't pass a "
         "return that only closes with both springs healthy."),
        provenance="FSAE two-return-spring single-fault rule; needs per-spring check")
_r_throttle_identical_backup.reference_claim = (
    "The two throttle return springs are identical, so if one fails the other is fine.")


def _r_throttle_sensor_is_spring(claim: ParsedClaim, context: Any) -> Optional[CheckOutcome]:
    if not claim.has("throttle", "return", "tps", "apps", "sensor"):
        return None
    if not (claim.has("sensor", "tps", "apps", "position sensor")
            and claim.has("spring", "return", "counts", "count", "instead of")):
        return None
    return CheckOutcome(
        Verdict.MYTH,
        ("A throttle/accelerator position sensor (TPS/APPS) does NOT count as a return "
         "spring. FSAE requires at least two return springs that mechanically close the "
         "throttle if any one component fails; a sensor only measures position, it "
         "applies no closing force. You need two independent springs regardless of how "
         "many sensors you run."),
        provenance="FSAE rule: TPS/APPS explicitly not a return spring")
_r_throttle_sensor_is_spring.reference_claim = (
    "The throttle position sensor can act as one of the two required return springs.")


RULES = [
    Rule("brakes.bigger_rotor_force", "brakes", _r_bigger_rotor_force,
         keywords_any=("bigger rotor", "larger rotor", "bigger disc", "bigger disk",
                       "rotor size", "rotor"), priority=10),
    Rule("brakes.brake_bias", "brakes", _r_brake_bias,
         keywords_any=("brake bias", "brake balance", "bias"), priority=20),
    Rule("brakes.brake_energy", "brakes", _r_brake_energy,
         keywords_any=("brake", "braking"), priority=30),
    Rule("brakes.throttle_identical_backup", "brakes", _r_throttle_identical_backup,
         keywords_any=("throttle", "return spring", "return springs", "pedal spring",
                       "backup spring", "second spring", "return", "one spring",
                       "one unhooks", "one fails"), priority=5),
    Rule("brakes.throttle_sensor_is_spring", "brakes", _r_throttle_sensor_is_spring,
         keywords_any=("throttle", "tps", "apps", "sensor"), priority=6),
]
for _rule in RULES:
    register(_rule)
