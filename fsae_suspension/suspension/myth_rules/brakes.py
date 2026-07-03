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


RULES = [
    Rule("brakes.bigger_rotor_force", "brakes", _r_bigger_rotor_force,
         keywords_any=("bigger rotor", "larger rotor", "bigger disc", "bigger disk",
                       "rotor size", "rotor"), priority=10),
    Rule("brakes.brake_bias", "brakes", _r_brake_bias,
         keywords_any=("brake bias", "brake balance", "bias"), priority=20),
    Rule("brakes.brake_energy", "brakes", _r_brake_energy,
         keywords_any=("brake", "braking"), priority=30),
]
for _rule in RULES:
    register(_rule)
