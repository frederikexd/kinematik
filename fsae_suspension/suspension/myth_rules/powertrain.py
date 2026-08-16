# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
Powertrain myth rules.

These are the seven rules the original ``pt_integration.check_assumption``
shipped, re-expressed as engine ``Rule`` objects. Behaviour is preserved: same
verdicts, same thresholds, same live-number explanations. Splitting them out of
the 250-line if/elif chain is what lets the other seven disciplines add their
own rules without touching a giant function.

Context expected: a ``PowertrainContext`` (or a bare ``MotorEnvelope``). A rule
that needs gearing it wasn't given returns DEPENDS with an honest reason rather
than guessing.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..mythbuster import preliminary, CheckOutcome, ParsedClaim, Rule, Verdict, register


# --------------------------------------------------------------------------- #
#  Context                                                                     #
# --------------------------------------------------------------------------- #
@dataclass
class PowertrainContext:
    """What powertrain rules check against. ``env`` is the live MotorEnvelope;
    gearing is optional (some rules need it, and say so when it's missing)."""
    env: Any                              # MotorEnvelope (duck-typed to avoid heavy import)
    gear_final_drive: float | None = None
    wheel_r_m: float = 0.228


def _as_ctx(context: Any) -> PowertrainContext | None:
    """Accept a PowertrainContext, a bare MotorEnvelope, or a dict; return a
    PowertrainContext or None if there's nothing usable."""
    if context is None:
        return None
    if isinstance(context, PowertrainContext):
        return context
    # bare envelope?
    if hasattr(context, "peak_power_kw") and hasattr(context, "redline_rpm"):
        return PowertrainContext(env=context)
    if isinstance(context, dict):
        env = context.get("env") or context.get("powertrain")
        if env is not None:
            return PowertrainContext(
                env=env,
                gear_final_drive=context.get("gear_final_drive"),
                wheel_r_m=context.get("wheel_r_m", 0.228),
            )
    return None


#  Each rule now gives its own QUALITATIVE answer when the motor envelope is
#  absent, instead of the single shared "I can't check that" refusal this used
#  to be. The direction of every one of these follows from P = T*omega and needs
#  no motor data; what the envelope adds is the user's numbers. See
#  mythbuster.preliminary().
_NEED_ENV = ("The live motor envelope is not loaded, so there are no numbers "
             "for YOUR motor here — open the EV Powertrain tab for peak "
             "torque/power, base speed and redline.")


# --------------------------------------------------------------------------- #
#  RULE 1 — "power cap limits / sets RPM"                                      #
# --------------------------------------------------------------------------- #
def _r_power_caps_rpm(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not (claim.has("cap", "limit", "restrict", "max", "ceiling", "cannot exceed",
                      "can't exceed", "can not exceed")
            and claim.has("rpm", "redline", "rev", "speed")
            and claim.has("kw", "kilowatt", "power", "watt")):
        return None
    ctx = _as_ctx(context)
    if ctx is None:
        return preliminary(
            Verdict.MYTH,
            ("Power and rpm are independent limits. A power cap limits TORQUE at "
             "a given speed, not the speed itself: with P = T*omega the "
             "controller simply pulls torque down as revs rise, so the motor "
             "keeps spinning to whatever redline its mechanical limit and your "
             "gearing set. Capping power does not cap rpm."),
            need=_NEED_ENV)
    env = ctx.env
    user_rpm = claim.num("rpm")
    claimed = f"~{user_rpm:.0f} rpm" if user_rpm else "some RPM limit"
    if user_rpm and abs(user_rpm - env.redline_rpm) < 200:
        return CheckOutcome(
            Verdict.DEPENDS,
            (f"The {env.redline_rpm:.0f} rpm figure is right but the reasoning is "
             f"wrong. The motor reaches {env.peak_power_kw:.0f} kW at base speed "
             f"({env.base_speed_rpm:.0f} rpm) and holds it to redline — power "
             f"doesn't set redline. Redline comes from the motor's mechanical "
             f"limit and your gearing, independently of the power cap."),
            grounding="physics",
            provenance=f"redline={env.redline_rpm:.0f} rpm, base={env.base_speed_rpm:.0f} rpm")
    return CheckOutcome(
        Verdict.MYTH,
        (f"Power and RPM are independent limits. This motor reaches "
         f"{env.peak_power_kw:.0f} kW at {env.base_speed_rpm:.0f} rpm (base speed) "
         f"and holds that power to {env.redline_rpm:.0f} rpm. The controller caps "
         f"torque — not rpm — where T\u00d7\u03c9 = {env.peak_power_kw:.0f} kW. "
         f"Redline is {env.redline_rpm:.0f} rpm regardless of the cap; {claimed} "
         f"is not implied."),
        grounding="physics",
        provenance=f"peak={env.peak_power_kw:.0f} kW @ {env.base_speed_rpm:.0f}-{env.redline_rpm:.0f} rpm")
_r_power_caps_rpm.reference_claim = "Capping power to 80 kW means we can't rev past 7000 rpm."


# --------------------------------------------------------------------------- #
#  RULE 2 — continuous vs peak power                                          #
# --------------------------------------------------------------------------- #
def _r_continuous_vs_peak(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not (claim.has("continuous", "sustained", "steady") and claim.has("kw", "power")):
        return None
    ctx = _as_ctx(context)
    if ctx is None:
        return preliminary(
            Verdict.MYTH,
            ("Continuous power cannot exceed peak power. Peak is what the "
             "machine will deliver briefly before something thermal gives; "
             "continuous is what it will hold indefinitely once temperatures "
             "settle. Continuous is therefore always the LOWER number, and the "
             "gap between them is a cooling question."),
            need=_NEED_ENV)
    env = ctx.env
    user_kw = claim.num("kw")
    if user_kw and user_kw > env.peak_power_kw + 0.5:
        return CheckOutcome(
            Verdict.MYTH,
            (f"Continuous power can never exceed peak. You stated {user_kw:.0f} kW "
             f"continuous but this motor's peak is {env.peak_power_kw:.0f} kW "
             f"(capped at {env.rule_cap_kw:.0f} kW by the FSAE rule). Continuous "
             f"({env.continuous_power_kw:.0f} kW here) is what the cooling sustains "
             f"\u2014 always \u2264 peak."),
            grounding="physics",
            provenance=f"peak={env.peak_power_kw:.0f} kW, continuous={env.continuous_power_kw:.0f} kW")
    if user_kw and abs(user_kw - env.continuous_power_kw) < env.peak_power_kw * 0.05:
        return CheckOutcome(
            Verdict.TRUE,
            (f"Correct. {user_kw:.0f} kW matches the continuous rating "
             f"({env.continuous_power_kw:.0f} kW), which is "
             f"{env.continuous_power_kw/env.peak_power_kw*100:.0f}% of the "
             f"{env.peak_power_kw:.0f} kW peak \u2014 the thermal limit cooling can "
             f"sustain indefinitely."),
            grounding="physics",
            provenance=f"continuous={env.continuous_power_kw:.0f} kW")
    kw_str = f"{user_kw:.0f} kW " if user_kw else ""
    return CheckOutcome(
        Verdict.DEPENDS,
        (f"Continuous {kw_str}power is set by cooling capacity, not a rule. Here "
         f"the continuous limit is {env.continuous_power_kw:.0f} kW "
         f"({env.continuous_power_kw/env.peak_power_kw*100:.0f}% of peak). It is "
         f"always \u2264 peak ({env.peak_power_kw:.0f} kW) and \u2264 the "
         f"{env.rule_cap_kw:.0f} kW FSAE cap."),
        grounding="physics",
        provenance=f"continuous={env.continuous_power_kw:.0f} kW, peak={env.peak_power_kw:.0f} kW")
_r_continuous_vs_peak.reference_claim = "Continuous power can be higher than peak power."


# --------------------------------------------------------------------------- #
#  RULE 3 — base speed / where peak power hits                                #
# --------------------------------------------------------------------------- #
def _r_base_speed(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not (claim.has("base speed", "corner speed", "peak power", "max power",
                      "full power") and claim.has("rpm", "redline")):
        return None
    ctx = _as_ctx(context)
    if ctx is None:
        return preliminary(
            Verdict.MYTH,
            ("Peak power happens at BASE SPEED, not at redline. Below base speed "
             "the motor holds roughly constant torque, so power climbs with rpm; "
             "above it the controller holds power flat and torque falls as "
             "P/omega. Peak power is reached at the corner between those two "
             "regions and is merely maintained to redline."),
            need=_NEED_ENV)
    env = ctx.env
    user_rpm = claim.num("rpm")
    if claim.has("redline", "max rpm", "maximum rpm"):
        return CheckOutcome(
            Verdict.MYTH,
            (f"Peak power is reached at base speed ({env.base_speed_rpm:.0f} rpm), "
             f"not at redline ({env.redline_rpm:.0f} rpm). Above base speed the "
             f"motor holds {env.peak_power_kw:.0f} kW while torque falls as 1/\u03c9 "
             f"\u2014 power at redline is only {env.power_at_redline_kw():.0f} kW."),
            grounding="physics",
            provenance=f"base={env.base_speed_rpm:.0f} rpm, redline={env.redline_rpm:.0f} rpm")
    if user_rpm:
        err = abs(user_rpm - env.base_speed_rpm) / env.base_speed_rpm * 100
        if err < 5:
            return CheckOutcome(
                Verdict.TRUE,
                (f"Correct. Base speed is {env.base_speed_rpm:.0f} rpm \u2014 where "
                 f"{env.peak_power_kw:.0f} kW is first reached (T\u00d7\u03c9 = P). "
                 f"Above it, torque falls; power stays flat."),
                grounding="physics",
                provenance=f"base={env.base_speed_rpm:.0f} rpm")
        return CheckOutcome(
            Verdict.MYTH,
            (f"Base speed is {env.base_speed_rpm:.0f} rpm, not {user_rpm:.0f} rpm "
             f"\u2014 where peak power ({env.peak_power_kw:.0f} kW) is first reached. "
             f"Your figure is {err:.0f}% off."),
            grounding="physics",
            provenance=f"base={env.base_speed_rpm:.0f} rpm")
    return CheckOutcome(
        Verdict.DEPENDS,
        (f"Base speed for this motor is {env.base_speed_rpm:.0f} rpm. Below it, "
         f"torque is flat at {env.peak_torque_nm:.0f} N\u00b7m and power rises "
         f"linearly. Above it, torque falls and power holds {env.peak_power_kw:.0f} "
         f"kW to redline."),
        grounding="physics",
        provenance=f"base={env.base_speed_rpm:.0f} rpm")
_r_base_speed.reference_claim = "Peak power happens at redline."


# --------------------------------------------------------------------------- #
#  RULE 4 — top speed from redline + gearing                                  #
# --------------------------------------------------------------------------- #
def _r_top_speed(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    # Only a genuine top-speed claim: an explicit speed phrase or a speed unit.
    # Bare "fast"/"faster" is too ambiguous (brakes, aero, "more power = faster")
    # and is handled by the more specific rules instead.
    if not claim.has("top speed", "max speed", "maximum speed", "km/h", "kmh",
                     "kph", "velocity"):
        return None
    ctx = _as_ctx(context)
    if ctx is None:
        return preliminary(
            Verdict.DEPENDS,
            ("Top speed is set by whichever comes first: the rpm your gearing "
             "runs out at, or the speed where drag absorbs all available "
             "power. FSAE cars are almost always the FORMER on an autocross "
             "course \u2014 you hit redline in the tallest usable gear long "
             "before drag limits you \u2014 which means top speed is a gearing "
             "choice, not a power one. Check which limit binds before buying "
             "power you cannot use."),
            need=_NEED_ENV)
    env = ctx.env
    if not ctx.gear_final_drive:
        return CheckOutcome(
            Verdict.DEPENDS,
            (f"Top speed depends on gearing, which isn't set here. With redline "
             f"{env.redline_rpm:.0f} rpm and a {ctx.wheel_r_m*1000:.0f} mm wheel, "
             f"give me the final-drive ratio and I'll give you the number."),
            grounding="physics",
            provenance="no final-drive ratio in context")
    v_redline = (env.redline_rpm * 2 * math.pi / 60.0) * ctx.wheel_r_m / ctx.gear_final_drive * 3.6
    user_kmh = claim.num("kmh")
    if user_kmh:
        err = abs(user_kmh - v_redline)
        if err < 5:
            return CheckOutcome(
                Verdict.TRUE,
                (f"Correct. With {env.redline_rpm:.0f} rpm redline and "
                 f"{ctx.gear_final_drive:.2f}:1 final drive "
                 f"({ctx.wheel_r_m*1000:.0f} mm wheel) top speed is {v_redline:.0f} "
                 f"km/h \u2014 your {user_kmh:.0f} km/h matches."),
                grounding="physics",
                provenance=f"v_redline={v_redline:.0f} km/h")
        return CheckOutcome(
            Verdict.MYTH,
            (f"With {env.redline_rpm:.0f} rpm and {ctx.gear_final_drive:.2f}:1 final "
             f"drive top speed is {v_redline:.0f} km/h, not {user_kmh:.0f} km/h "
             f"(off by {err:.0f} km/h). Check wheel radius "
             f"({ctx.wheel_r_m*1000:.0f} mm) and final drive."),
            grounding="physics",
            provenance=f"v_redline={v_redline:.0f} km/h")
    return CheckOutcome(
        Verdict.DEPENDS,
        (f"With {env.redline_rpm:.0f} rpm redline and {ctx.gear_final_drive:.2f}:1 "
         f"final drive ({ctx.wheel_r_m*1000:.0f} mm wheel) top speed is "
         f"{v_redline:.0f} km/h. Give me your expected figure and I'll verify it."),
        grounding="physics",
        provenance=f"v_redline={v_redline:.0f} km/h")


# --------------------------------------------------------------------------- #
#  RULE 5 — torque at a specific RPM                                          #
# --------------------------------------------------------------------------- #
def _r_torque_at_rpm(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not (claim.has("torque") and claim.num("rpm") and claim.num("nm")):
        return None
    ctx = _as_ctx(context)
    if ctx is None:
        #  NO qualitative answer exists here, and that is not a gap in the
        #  rule. This claim asserts a SPECIFIC torque at a SPECIFIC rpm, which
        #  can only be checked against the actual curve — first principles have
        #  nothing to say about whether YOUR motor makes 180 N.m at 4000 rpm.
        #  UNVERIFIED rather than UNKNOWN so the reader can tell "we will not
        #  guess" from "we did not understand the question".
        return CheckOutcome(
            Verdict.UNVERIFIED,
            ("Checking a specific torque at a specific rpm needs the actual "
             "motor curve \u2014 no general principle can confirm or deny a "
             "number for your machine. Open the EV Powertrain tab so the "
             "envelope is loaded, then ask again and this will interpolate the "
             "real curve at that rpm and tell you the error."),
            grounding="asserted",
            sources=("your motor datasheet / dyno curve",),
            provenance="claim is machine-specific; no envelope in context")
    env = ctx.env
    user_rpm = claim.num("rpm")
    user_nm = claim.num("nm")
    actual_nm = float(np.interp(user_rpm, env.rpm, env.torque_nm))
    err = abs(user_nm - actual_nm) / max(actual_nm, 1.0) * 100
    if err < 8:
        return CheckOutcome(
            Verdict.TRUE,
            (f"Correct. At {user_rpm:.0f} rpm this envelope gives {actual_nm:.0f} "
             f"N\u00b7m \u2014 your {user_nm:.0f} N\u00b7m is within {err:.1f}%."),
            grounding="physics",
            provenance=f"interp torque={actual_nm:.0f} N\u00b7m @ {user_rpm:.0f} rpm")
    region = ("below base speed (flat torque)" if user_rpm < env.base_speed_rpm
              else "above base speed (power-limited)")
    tail = (f"Below {env.base_speed_rpm:.0f} rpm torque is flat at "
            f"{env.peak_torque_nm:.0f} N\u00b7m."
            if user_rpm < env.base_speed_rpm
            else f"Above {env.base_speed_rpm:.0f} rpm torque falls as P/\u03c9.")
    return CheckOutcome(
        Verdict.MYTH,
        (f"At {user_rpm:.0f} rpm ({region}) this motor produces {actual_nm:.0f} "
         f"N\u00b7m, not {user_nm:.0f} N\u00b7m ({err:.0f}% error). {tail}"),
        grounding="physics",
        provenance=f"interp torque={actual_nm:.0f} N\u00b7m @ {user_rpm:.0f} rpm")


# --------------------------------------------------------------------------- #
#  RULE 6 — FSAE cap compliance                                              #
# --------------------------------------------------------------------------- #
def _r_cap_compliance(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not claim.has("80 kw", "80kw", "fsae cap", "fsae limit", "rule", "compliant",
                     "comply", "legal", "within the cap", "under the cap"):
        return None
    ctx = _as_ctx(context)
    if ctx is None:
#  NUMERIC CLAIMS DEFER TO THE GENERAL REASONER. When the user states a
        #  figure ("our pack draws 75 kW peak") the cap question is arithmetic
        #  against a fixed rulebook constant and needs no motor model — and the
        #  reasoner already has tested logic for the part that is genuinely
        #  subtle: telling a DRAW from a RATING ("inverter rated 120 kW, draw
        #  capped to 80"). Declining here reuses that instead of reimplementing
        #  it. An earlier attempt to reimplement it inline called a legal 80 kW
        #  draw a scrutineering fail because a 120 kW rating appeared in the
        #  same sentence.
        if claim.num("kw"):
            return None
        return preliminary(
            Verdict.DEPENDS,
            ("Compliance with the FSAE 80 kW cap depends on the trace, not the "
             "rating. The cap applies to tractive power measured at the "
             "ACCUMULATOR terminals over a rolling window, so an inverter or "
             "motor rated above 80 kW is perfectly legal as long as delivered "
             "power never exceeds it. What matters is the measured peak of "
             "your run, including transient overshoot on corner exit."),
            need=_NEED_ENV)
    env = ctx.env
    if env.over_cap:
        return CheckOutcome(
            Verdict.MYTH,
            (f"This envelope's declared peak ({env.peak_power_kw:.0f} kW) exceeds the "
             f"FSAE {env.rule_cap_kw:.0f} kW tractive cap \u2014 not compliant without "
             f"an electronic limiter. The envelope is shown clamped to "
             f"{env.rule_cap_kw:.0f} kW."),
            grounding="physics",
            provenance=f"peak={env.peak_power_kw:.0f} kW vs cap {env.rule_cap_kw:.0f} kW")
    return CheckOutcome(
        Verdict.TRUE,
        (f"Correct. Peak power ({env.peak_power_kw:.0f} kW) is within the FSAE "
         f"{env.rule_cap_kw:.0f} kW cap. Continuous ({env.continuous_power_kw:.0f} "
         f"kW) is also under the cap."),
        grounding="physics",
        provenance=f"peak={env.peak_power_kw:.0f} kW vs cap {env.rule_cap_kw:.0f} kW")
_r_cap_compliance.reference_claim = "We're compliant with the 80 kW FSAE rule."


# --------------------------------------------------------------------------- #
#  RULE 7 — "more power = more speed"                                         #
# --------------------------------------------------------------------------- #
def _r_more_power_more_speed(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not (claim.has("more power", "higher power", "increase power", "bigger motor",
                      "more kw", "higher kw")
            and claim.has("faster", "more speed", "higher speed", "quicker",
                          "better acceleration", "higher top speed")):
        return None
    ctx = _as_ctx(context)
    if ctx is None:
        # this rule is qualitative; still answerable without exact numbers
        return CheckOutcome(
            Verdict.DEPENDS,
            ("It depends on the binding constraint. Traction-limited (low speed): "
             "more torque helps. Power-limited (high speed): more kW helps. Past "
             "redline you need a different gear ratio, not more power. And the FSAE "
             "80 kW cap rules out power above that entirely."),
            grounding="physics",
            provenance="qualitative; no live envelope")
    env = ctx.env
    return CheckOutcome(
        Verdict.DEPENDS,
        (f"It depends on which constraint is binding. With {env.peak_power_kw:.0f} kW "
         f"and {env.peak_torque_nm:.0f} N\u00b7m: traction-limited (low speed) \u2192 "
         f"more torque helps; power-limited (high speed) \u2192 more kW helps. Above "
         f"{env.redline_rpm:.0f} rpm you need different gearing, not more power. The "
         f"FSAE {env.rule_cap_kw:.0f} kW cap rules out power above that."),
        grounding="physics",
        provenance=f"peak={env.peak_power_kw:.0f} kW / {env.peak_torque_nm:.0f} N\u00b7m")
_r_more_power_more_speed.reference_claim = "More power always makes the car faster."


# --------------------------------------------------------------------------- #
#  Registration                                                               #
# --------------------------------------------------------------------------- #
RULES = [
    Rule("powertrain.power_caps_rpm", "powertrain", _r_power_caps_rpm,
         keywords_any=("rpm", "redline", "rev", "speed"), priority=10),
    Rule("powertrain.continuous_vs_peak", "powertrain", _r_continuous_vs_peak,
         keywords_any=("continuous", "sustained", "steady"), priority=20),
    Rule("powertrain.base_speed", "powertrain", _r_base_speed,
         keywords_any=("base speed", "corner speed", "peak power", "max power",
                       "full power"), priority=30),
    Rule("powertrain.top_speed", "powertrain", _r_top_speed,
         keywords_any=("top speed", "max speed", "maximum speed", "km/h", "kmh",
                       "kph", "velocity"), priority=40),
    Rule("powertrain.torque_at_rpm", "powertrain", _r_torque_at_rpm,
         keywords_any=("torque",), priority=50),
    Rule("powertrain.cap_compliance", "powertrain", _r_cap_compliance,
         keywords_any=("80 kw", "80kw", "fsae cap", "fsae limit", "rule",
                       "compliant", "comply", "legal", "cap"), priority=60),
    Rule("powertrain.more_power_more_speed", "powertrain", _r_more_power_more_speed,
         keywords_any=("more power", "higher power", "increase power",
                       "bigger motor", "more kw", "higher kw"), priority=70),
]

for _rule in RULES:
    register(_rule)
