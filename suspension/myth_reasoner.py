# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
myth_reasoner.py — deterministic general-knowledge fallback for the myth-buster
===============================================================================

Why this exists
---------------
The myth-buster's registered rules are *exact*: each one checks a specific claim
against the live models with real arithmetic. That is the gold standard, but it
only covers the claims someone has written a rule for. Every other assumption a
lead types fell through to a bare "no registered rule could check that", which
reads like the tool is broken even for perfectly reasonable engineering claims.

This module is the safety net UNDER the registered rules. When no exact rule
matches, ``assess()`` runs the claim against a broad, hand-curated knowledge
base of physics, engineering and FSAE relationships and returns a reasoned
verdict — the same kind of substantive answer a knowledgeable lead would give,
produced entirely in Python.

The honesty contract (unchanged)
--------------------------------
    * **No AI, no LLM, no network.** Every answer is deterministic: the same
      claim always yields the same verdict and reasoning. It is all keyword
      routing + encoded domain relationships, which you can read below.
    * **Confidence is earned, not faked.** A claim only gets a hard MYTH/TRUE
      verdict when it maps to a relationship the knowledge base actually
      encodes. Anything vaguer comes back as DEPENDS with the governing physics
      and the tradeoffs named — never a confident guess.
    * **FSAE rule claims are flagged for verification.** The rulebook changes
      yearly; encoded limits are the stable, long-standing ones, and every
      rule-flavoured answer tells the user to confirm against the current
      season's official rulebook rather than trusting a hardcoded number.

Public API
----------
    assess(claim, *, discipline=None) -> ReasonedVerdict | None

``claim`` is the ``ParsedClaim`` the engine already built (so we reuse its
number/unit extraction). Returns ``None`` only when even the general reasoner
has nothing relevant to say — in practice that is rare, because the generic
"engineering tradeoff" responder catches broad claims. The caller maps a
returned verdict onto a ``MythResult``.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Callable, Optional

# We depend only on the parsed-claim shape, not the engine, to avoid import
# cycles. Verdict strings are duplicated as plain literals ("myth"/"true"/
# "depends") so this module never imports the engine.


# --------------------------------------------------------------------------- #
#  Result shape                                                                #
# --------------------------------------------------------------------------- #
@dataclass
class ReasonedVerdict:
    """One reasoned answer from the general knowledge base.

    ``verdict`` is a plain string matching the engine's vocabulary
    ("myth" | "true" | "depends"). ``explanation`` is the plain-language
    reasoning. ``discipline`` and ``provenance`` mirror the engine's MythResult
    fields so the caller can build one directly. ``fsae_rule`` set True appends
    the "verify against the current rulebook" note.
    """
    verdict: str
    explanation: str
    discipline: str = ""
    provenance: str = "General engineering knowledge base (no live model)."
    fsae_rule: bool = False


# --------------------------------------------------------------------------- #
#  Knowledge-base entry                                                        #
# --------------------------------------------------------------------------- #
@dataclass
class _Topic:
    """One encoded relationship.

    ``any_of`` — the claim must contain at least one phrase from EACH inner
    group (an AND of ORs), so a topic only fires when the claim is really about
    it. ``respond`` receives the parsed claim and returns a ReasonedVerdict.
    ``priority`` orders topics; the first matching topic that returns non-None
    wins. Higher priority runs first.
    """
    name: str
    any_of: list[list[str]]
    respond: Callable[["object"], Optional[ReasonedVerdict]]
    discipline: str = ""
    priority: int = 0

    def matches(self, lower: str) -> bool:
        return all(any(p in lower for p in group) for group in self.any_of)


_TOPICS: list[_Topic] = []


def _topic(name, any_of, *, discipline="", priority=0):
    def _wrap(fn):
        _TOPICS.append(_Topic(name=name, any_of=any_of, respond=fn,
                              discipline=discipline, priority=priority))
        return fn
    return _wrap


def _v(verdict, explanation, *, discipline="", fsae_rule=False,
       provenance=None):
    kw = {}
    if provenance is not None:
        kw["provenance"] = provenance
    return ReasonedVerdict(verdict=verdict, explanation=explanation,
                           discipline=discipline, fsae_rule=fsae_rule, **kw)


# ===========================================================================
#  VEHICLE DYNAMICS
# ===========================================================================
@_topic("vd.downforce_always_faster",
        [["downforce", "aero"], ["always", "faster", "quicker", "lap"]],
        discipline="aerodynamics", priority=60)
def _downforce_faster(c):
    return _v(
        "depends",
        "More downforce is not unconditionally faster. Downforce grows grip in "
        "corners but its partner — drag — costs you on the straights, and it "
        "scales with velocity squared, so the balance shifts with the track. On "
        "a tight, low-speed autocross the cornering gain usually dominates and "
        "more wing is faster; on a straight-heavy layout the drag penalty can "
        "make you slower. The honest answer is track- and speed-dependent: "
        "evaluate lap time on the actual course (the Lap Time tab), don't assume "
        "a monotonic 'more is better'.",
        discipline="aerodynamics")


@_topic("vd.load_sensitivity",
        [["load", "vertical", "weight on"], ["grip", "traction", "friction"]],
        discipline="suspension", priority=55)
def _load_sensitivity(c):
    return _v(
        "myth",
        "Grip does not rise proportionally with vertical load. Tyres are "
        "load-sensitive: the friction coefficient falls as normal load rises, "
        "so doubling the load gives noticeably LESS than double the lateral "
        "force. This is exactly why lateral load transfer hurts an axle — the "
        "heavily loaded outer tyre gains less than the unloaded inner tyre "
        "loses. Use the tyre model (Suspension/Tyre tab) for the real curve; "
        "the 'twice the load, twice the grip' intuition is the classic myth.",
        discipline="suspension")


@_topic("vd.stiffer_always_better",
        [["stiffer", "stiff", "spring rate", "roll stiffness"],
         ["faster", "better", "more grip", "handling"]],
        discipline="suspension", priority=45)
def _stiffer_better(c):
    return _v(
        "depends",
        "Stiffer is not automatically better. Raising rate reduces body roll "
        "and pitch and can sharpen response, but it also cuts mechanical grip "
        "over bumps and kerbs and shifts the balance toward whichever end you "
        "stiffened (stiffen the front and you add understeer). The optimum is a "
        "compromise set by track roughness, tyre load-sensitivity and the "
        "balance you want — not a 'more is always better' axis.",
        discipline="suspension")


@_topic("vd.lower_cg",
        [["lower", "low"], ["center of gravity", "centre of gravity", "cg",
                            "cog", "roll centre", "roll center"]],
        discipline="suspension", priority=40)
def _lower_cg(c):
    return _v(
        "true",
        "Lowering the centre of gravity is one of the few near-free wins in "
        "vehicle dynamics: it cuts lateral load transfer for a given track "
        "width and corner, which — because tyres are load-sensitive — raises "
        "total axle grip and reduces roll. The only caveats are packaging, "
        "ground clearance and driveline angles; the dynamics themselves favour "
        "a lower CG almost every time.",
        discipline="suspension")


@_topic("vd.weight_always_bad",
        [["lighter", "less weight", "reduce weight", "lower mass", "heavier"],
         ["faster", "better", "quicker", "grip", "accelerat"]],
        discipline="chassis", priority=42)
def _weight(c):
    return _v(
        "depends",
        "Less mass helps acceleration, braking and tyre life and is almost "
        "always worth chasing — but 'lighter is always faster' skips the "
        "tradeoffs. Cornering grip rises with load (though less than "
        "proportionally, since tyres are load-sensitive), so shedding mass "
        "gains less in sustained corners than it does in the straights and "
        "transitions. And weight cut at the cost of stiffness, reliability or "
        "legal minimums is a net loss. Direction: right. Unconditional: no.",
        discipline="chassis")


@_topic("vd.wider_tyre_more_grip",
        [["wider", "bigger tyre", "bigger tire", "tyre width", "tire width"],
         ["grip", "traction", "faster"]],
        discipline="suspension", priority=40)
def _wider_tyre(c):
    return _v(
        "depends",
        "A wider tyre often grips more, but not for the schoolbook reason. "
        "Classic friction says contact area doesn't change friction force; real "
        "tyres beat that because a bigger contact patch lowers pressure and "
        "temperature per unit area and exploits load-sensitivity, raising the "
        "effective grip. But wider costs mass, aero drag, warm-up and possibly "
        "compound availability, and only helps if you can actually load and "
        "heat it. Conditional win, not a law.",
        discipline="suspension")


@_topic("vd.suspension_architecture",
        [["double wishbone", "wishbone", "macpherson", "mcpherson",
          "trailing arm", "multilink", "multi-link", "swing axle",
          "solid axle", "live axle", "pushrod", "pullrod"],
         ["better", "worse", "best", "than", "vs", "versus", "superior",
          "prefer", "should we", "use"]],
        discipline="suspension", priority=58)
def _suspension_architecture(c):
    lower = getattr(c, "lower", "")
    dw = ("double wishbone" in lower or "wishbone" in lower)
    mac = ("macpherson" in lower or "mcpherson" in lower)
    if dw and mac:
        return _v(
            "depends",
            "Double-wishbone vs MacPherson is a constraint tradeoff, not a "
            "ranking. Double wishbones give the designer near-full control of "
            "camber gain, roll centre and scrub through the corner, which is "
            "why nearly every FSAE and racing car uses them — the kinematics "
            "are simply better. MacPherson is cheaper, lighter in part count "
            "and packages well in a road car with a tall strut tower, but it "
            "compromises camber control under roll. For an FSAE car with room "
            "to package upper and lower arms, double wishbone is the usual "
            "choice; 'better' still depends on your packaging and cost "
            "constraints, so state those and it becomes a clear decision. Use "
            "the Kinematics tab to compare camber curves for your actual "
            "geometry.",
            discipline="suspension")
    return _v(
        "depends",
        "Suspension-architecture 'X is better than Y' claims depend on what you "
        "need from the kinematics: camber control through roll, roll-centre "
        "placement, anti-features, packaging, unsprung mass and cost all trade "
        "against each other. There's no universally best linkage — the right "
        "one is set by your track, tyre and packaging. Compare the actual "
        "camber and roll-centre curves in the Kinematics tab rather than "
        "ranking layouts in the abstract.",
        discipline="suspension")


@_topic("vd.understeer_oversteer",
        [["understeer", "oversteer", "push", "loose"],
         ["safe", "faster", "slower", "better", "grip"]],
        discipline="suspension", priority=35)
def _balance(c):
    return _v(
        "depends",
        "Neither understeer nor oversteer is inherently faster — the quick "
        "setup is the one that puts both axles at their peak slip at the same "
        "time (neutral, or a hair of the balance the driver can exploit). Mild "
        "understeer is more stable and forgiving; a touch of oversteer can "
        "rotate the car on entry. Which is 'better' depends on the corner, the "
        "driver and the tyres, so tune balance to peak both axles rather than "
        "chasing one label.",
        discipline="suspension")


# ===========================================================================
#  BRAKES
# ===========================================================================
@_topic("brk.bigger_rotor_stops_faster",
        [["bigger rotor", "larger rotor", "bigger disc", "bigger brake",
          "more braking", "bigger caliper", "bigger calliper"],
         ["stop", "faster", "shorter", "more grip", "deceler"]],
        discipline="brakes", priority=55)
def _bigger_brakes(c):
    return _v(
        "myth",
        "Bigger brakes do not shorten stopping distance on their own. Peak "
        "deceleration is set by tyre grip and load, not caliper size — once you "
        "can lock the wheels (or hit the ABS/limit), more clamping does nothing "
        "for distance. What bigger rotors buy is thermal capacity and "
        "fade-resistance over repeated stops, and pedal feel. For a single "
        "stop, tyres and weight decide it, not brake size.",
        discipline="brakes")


@_topic("brk.brake_bias",
        [["brake bias", "brake balance", "front brake", "rear brake",
          "bias forward", "bias rearward"],
         ["faster", "better", "shorter", "lock", "stable"]],
        discipline="brakes", priority=40)
def _brake_bias(c):
    return _v(
        "depends",
        "Optimal brake bias tracks the dynamic load distribution, which shifts "
        "forward under deceleration. Too much front bias locks the fronts "
        "(understeer, lost steering); too much rear locks the rears "
        "(instability). The fastest bias puts both axles near lock "
        "simultaneously for the given deceleration and grip — it's a tuned "
        "compromise, not a fixed 'more front is safer' rule.",
        discipline="brakes")


# ===========================================================================
#  POWERTRAIN / EV
# ===========================================================================
@_topic("pt.more_power_faster",
        [["more power", "more kw", "more horsepower", "more torque",
          "bigger motor"],
         ["faster", "quicker", "lower lap", "better lap", "win"]],
        discipline="powertrain", priority=45)
def _more_power(c):
    return _v(
        "depends",
        "More power helps only where you can put it down. On an FSAE autocross, "
        "traction, corner exit and driveability often cap usable power well "
        "below peak — extra kW past the traction limit just spins tyres and "
        "adds heat and mass. And the accumulator power draw is capped by rule "
        "(historically 80 kW), so 'more power' may not even be legal. Gains are "
        "real on longer straights, marginal in tight sections.",
        discipline="powertrain", fsae_rule=True)


@_topic("pt.power_limit",
        [["accumulator", "tractive", "battery", "power limit", "power cap",
          "kw limit", "80 kw", "80kw", "draw"],
         ["kw", "power", "draw", "limit", "cap", "legal", "allowed", "rule"]],
        discipline="powertrain", priority=70)
def _power_limit(c):
    kw = c.numbers.get("kw")
    lower = getattr(c, "lower", "")
    # Only answer as a rules point when the claim is really about pack power
    # draw, not any sentence that merely contains the word "power".
    if not any(k in lower for k in ("accumulator", "tractive", "battery",
                                    "power limit", "power cap", "kw limit",
                                    "80 kw", "80kw", "draw")):
        return None
    over = ""
    if kw is not None and kw > 80:
        over = (f" Your figure of {kw:g} kW is above the historical 80 kW cap, "
                "so as stated it would be non-compliant.")
    elif kw is not None and kw <= 80:
        over = f" Your figure of {kw:g} kW is within the historical 80 kW cap."
    return _v(
        "true",
        "FSAE Electric has long capped the power drawn from the accumulator at "
        "80 kW, enforced by the energy meter — you cannot legally exceed it "
        "regardless of what the motor could deliver." + over + " Treat this as "
        "the stable limit, but confirm the exact figure and enforcement in the "
        "current season's rulebook before relying on it.",
        discipline="powertrain", fsae_rule=True)


@_topic("pt.regen",
        [["regen", "regeneration", "recuper"],
         ["free", "always", "faster", "range", "energy", "better"]],
        discipline="powertrain", priority=35)
def _regen(c):
    return _v(
        "depends",
        "Regen recovers braking energy and helps endurance energy budget, but "
        "it isn't free lap time: it adds control complexity, shifts brake bias "
        "(the tyres still do most of the stopping), and is subject to rule "
        "limits on regen at low speed. Worth it for the energy score; not a "
        "straight speed upgrade.",
        discipline="powertrain", fsae_rule=True)


# ===========================================================================
#  STRUCTURES / MATERIALS
# ===========================================================================
@_topic("str.stronger_stiffer",
        [["stronger", "strength"], ["stiffer", "stiff", "stiffness", "rigid"]],
        discipline="chassis", priority=60)
def _strength_vs_stiffness(c):
    return _v(
        "myth",
        "Strength and stiffness are different properties and don't move "
        "together. Stiffness resists deflection (set by geometry and elastic "
        "modulus E); strength resists failure (set by yield/ultimate stress). "
        "You can have a stiff part that's brittle, or a strong part that flexes "
        "a lot. Steel and aluminium differ ~3x in both density and modulus, so "
        "an 'equally stiff' aluminium part is bulkier but can weigh less — "
        "conflating the two is a common and costly design error.",
        discipline="chassis")


@_topic("str.thicker_stronger",
        [["thicker", "more material", "add material", "bigger tube",
          "thicker wall"],
         ["stronger", "stiffer", "better", "safe"]],
        discipline="chassis", priority=40)
def _thicker(c):
    return _v(
        "depends",
        "Adding material usually raises stiffness and strength, but where you "
        "add it dominates how much. Bending stiffness scales with the second "
        "moment of area, so moving material away from the neutral axis (larger "
        "diameter, thinner wall) is far more mass-efficient than simply "
        "thickening a wall. 'Thicker = better' ignores that a bigger, thinner "
        "section often beats a small, thick one at lower weight — the FSAE "
        "chassis game is stiffness per kilogram, not raw thickness.",
        discipline="chassis")


@_topic("str.carbon_always_better",
        [["carbon", "composite", "carbon fibre", "carbon fiber"],
         ["better", "stronger", "lighter", "always", "stiffer"]],
        discipline="chassis", priority=40)
def _carbon(c):
    return _v(
        "depends",
        "Carbon composite offers excellent stiffness- and strength-to-weight, "
        "but 'always better' ignores cost, manufacturing repeatability, "
        "damage-tolerance, joints/inserts and — for FSAE — the extra rules and "
        "testing that apply to composite structures. A well-designed steel "
        "spaceframe can beat a poorly executed composite on cost, schedule and "
        "reliability. Material choice is a systems decision, not a material "
        "ranking.",
        discipline="chassis", fsae_rule=True)


# ===========================================================================
#  THERMAL / COOLING
# ===========================================================================
@_topic("cool.bigger_radiator",
        [["bigger radiator", "larger radiator", "more cooling", "bigger rad"],
         ["cooler", "better", "always", "temperature", "overheat"]],
        discipline="cooling", priority=45)
def _radiator(c):
    return _v(
        "depends",
        "A bigger radiator adds heat-rejection capacity, but cooling is limited "
        "by airflow and temperature difference, not just core area. Past a "
        "point you're adding mass, frontal area and drag for little gain, and a "
        "poorly ducted large core can flow worse than a well-ducted small one. "
        "Fix airflow and ducting first; size the core to the actual heat load "
        "and worst-case ambient, not by 'bigger is cooler'.",
        discipline="cooling")


# ===========================================================================
#  ELECTRICAL
# ===========================================================================
@_topic("elec.higher_voltage",
        [["higher voltage", "more voltage", "raise voltage", "increase voltage"],
         ["faster", "better", "more power", "efficient", "current"]],
        discipline="electrics", priority=40)
def _voltage(c):
    return _v(
        "depends",
        "Higher pack voltage lets you deliver the same power at lower current, "
        "cutting I\u00b2R losses and cable/conductor mass — a real efficiency and "
        "packaging win. But it raises insulation, isolation and safety "
        "requirements, and FSAE caps the maximum tractive-system voltage. So "
        "higher voltage is often the better engineering choice up to the legal "
        "and safety ceiling, not an unconditional 'more is better'.",
        discipline="electrics", fsae_rule=True)


# ===========================================================================
#  GENERIC ENGINEERING RESPONDERS (broadest net, lowest priority)
# ===========================================================================
_COMPARATIVES = ("better", "worse", "faster", "slower", "stronger", "stiffer",
                 "lighter", "heavier", "more", "less", "always", "never",
                 "best", "worst", "increase", "decrease", "improve", "reduce")

# The generic responders are the broadest net, but they must not manufacture a
# confident-sounding answer for a claim that isn't actually about vehicles or
# engineering (that would be false confidence — the one thing this tool must
# never do). A claim only reaches the generic responders if it mentions at
# least one recognisable engineering / vehicle / FSAE term. Everything else
# falls through to an honest UNKNOWN.
_DOMAIN_TERMS = (
    # dynamics
    "grip", "traction", "tyre", "tire", "downforce", "drag", "aero", "lap",
    "corner", "understeer", "oversteer", "balance", "roll", "pitch", "camber",
    "caster", "toe", "slip", "load transfer", "cg", "center of gravity",
    "centre of gravity", "suspension", "spring", "damper", "shock", "arb",
    "anti-roll", "wheelbase", "track width", "ackermann", "steering",
    "wishbone", "macpherson", "pushrod", "pullrod", "motion ratio",
    # brakes
    "brake", "rotor", "disc", "caliper", "calliper", "pedal", "bias",
    "deceleration", "stopping",
    # powertrain / ev
    "power", "torque", "kw", "horsepower", "motor", "engine", "rpm", "gear",
    "accumulator", "battery", "voltage", "current", "cell", "regen", "tractive",
    "inverter", "energy", "efficiency", "drivetrain", "powertrain",
    # structure / materials
    "chassis", "frame", "stiffness", "strength", "stress", "strain", "modulus",
    "aluminium", "aluminum", "steel", "carbon", "composite", "material", "tube",
    "weld", "fatigue", "yield", "mass", "weight", "kg", "unsprung",
    # thermal
    "cooling", "radiator", "coolant", "temperature", "heat", "thermal", "fan",
    "duct", "airflow",
    # general vehicle / FSAE
    "car", "vehicle", "wheel", "speed", "acceleration", "force", "friction",
    "fsae", "formula", "autocross", "endurance", "skidpad", "rule", "legal",
    "cost", "reliability", "setup", "handling", "performance", "wing",
    "diffuser", "splitter", "undertray",
)


def _is_domain_relevant(lower: str) -> bool:
    return any(term in lower for term in _DOMAIN_TERMS)


@_topic("gen.absolute_claim",
        [["always", "never", "guarantee", "impossible", "definitely",
          "no matter", "in all cases", "every time"]],
        priority=10)
def _absolute(c):
    if not _is_domain_relevant(getattr(c, "lower", "")):
        return None
    return _v(
        "depends",
        "Engineering claims with 'always', 'never' or 'guaranteed' are almost "
        "always too strong. Real systems trade off against each other — grip vs "
        "drag, stiffness vs weight, power vs traction, cooling vs mass — so a "
        "change that helps one metric usually costs another, and the net result "
        "depends on the operating point (track, speed, temperature, load). "
        "Name the specific quantities and the operating condition and the "
        "answer usually becomes checkable; as an absolute, treat it with "
        "suspicion.",
        provenance="General engineering principle (tradeoffs dominate absolutes).")


@_topic("gen.comparative_tradeoff",
        [list(_COMPARATIVES)],
        priority=5)
def _comparative(c):
    if not _is_domain_relevant(getattr(c, "lower", "")):
        return None
    return _v(
        "depends",
        "This is a comparative engineering claim, and the honest answer is "
        "'it depends on the constraint that's actually binding'. Almost every "
        "'more X gives more Y' relationship saturates or reverses once a "
        "different limit takes over (traction, thermal, structural, aero drag, "
        "or an FSAE rule). Pin down the specific quantities and the operating "
        "point — then it can be checked against the live models or a hand "
        "calculation instead of argued. If it touches a rules limit, confirm "
        "against the current season's rulebook.",
        provenance="General engineering knowledge base (no live model).")


# --------------------------------------------------------------------------- #
#  Public entry                                                                #
# --------------------------------------------------------------------------- #
_FSAE_NOTE = (" \u26a0\ufe0f This touches an FSAE rules point. Encoded limits are the "
              "stable, long-standing ones \u2014 always confirm the exact figure and "
              "wording against the current season's official rulebook before "
              "relying on it.")


def assess(claim, *, discipline: Optional[str] = None) -> Optional[ReasonedVerdict]:
    """Reason about a claim the registered rules couldn't match.

    Deterministic: routes on encoded topic keywords, most-specific first, and
    returns the first topic that produces a verdict. Returns None only if even
    the generic responders find nothing (e.g. an empty or number-only claim).
    """
    lower = getattr(claim, "lower", "") or ""
    if not lower.strip():
        return None

    # If a discipline was explicitly picked, prefer topics from it, then fall
    # back to the rest — so "Brakes" narrows before the generic responders.
    ordered = sorted(_TOPICS, key=lambda t: t.priority, reverse=True)
    if discipline:
        ordered = ([t for t in ordered if t.discipline == discipline]
                   + [t for t in ordered if t.discipline != discipline])

    for topic in ordered:
        if not topic.matches(lower):
            continue
        try:
            out = topic.respond(claim)
        except Exception:
            continue
        if out is None:
            continue
        if out.fsae_rule and _FSAE_NOTE.strip() not in out.explanation:
            out.explanation = out.explanation + _FSAE_NOTE
        return out

    return None


def topic_count() -> int:
    """Number of encoded relationships — surfaced in tests/diagnostics."""
    return len(_TOPICS)
