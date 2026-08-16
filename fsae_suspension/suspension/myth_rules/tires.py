# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
Tyre myth rules — the discipline you can only test once, so the cost of a wrong
assumption is highest. Every rule checks against the live Pacejka model
(``tiremodel``) when present; with no fitted tyre loaded it answers against the
shipped FSAE default and SAYS SO, never implying the absolute number is measured.

Context: a ``PacejkaLateral`` tyre, or a dict ``{"tire": tyre, "Fz_N": load}``.
"""
from __future__ import annotations

from typing import Any

from ..mythbuster import preliminary, CheckOutcome, ParsedClaim, Rule, Verdict, register


def _tire_and_load(context: Any):
    """Return (tire, Fz_N, is_default_hint). tire may be None."""
    if context is None:
        return None, None, None
    tire = None
    Fz = None
    if hasattr(context, "mu_peak") and hasattr(context, "FNOMIN"):
        tire = context
    elif isinstance(context, dict):
        tire = context.get("tire") or context.get("tires")
        Fz = context.get("Fz_N") or context.get("load_n")
    if tire is not None and Fz is None:
        Fz = tire.FNOMIN
    return tire, Fz, None


#  Each tyre rule now answers QUALITATIVELY without a fitted tyre instead of
#  refusing. Load sensitivity and the camber/grip trade-off are universal tyre
#  behaviour; what the fitted model adds is the magnitude for YOUR compound.
_NEED_TIRE = ("No fitted tyre is loaded, so these are the general shapes rather "
              "than your compound's numbers \u2014 load your TTC fit (or the "
              "shipped FSAE default) for the actual curve.")


# --- RULE: load sensitivity ("twice the load = twice the grip") --------------
def _r_load_sensitivity(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not (claim.has("load") and claim.has("grip", "friction", "mu", "lateral",
                                            "cornering force", "more grip")):
        return None
    tire, Fz, _ = _tire_and_load(context)
    if tire is None:
        return preliminary(
            Verdict.MYTH,
            ("Tyre grip is LOAD SENSITIVE: the friction coefficient falls as "
             "vertical load rises, so doubling the load buys noticeably less "
             "than double the lateral force \u2014 typically more like "
             "1.7-1.8x on a racing tyre. This is the whole reason weight "
             "transfer costs grip: the loaded tyre gains less than the "
             "unloaded one gives up, so the axle nets out worse. It is also "
             "why lower CG and managing transfer matter at all."),
            need=_NEED_TIRE)
    mu_nom = tire.mu_peak(tire.FNOMIN)
    mu_2x = tire.mu_peak(2.0 * tire.FNOMIN)
    # grip force ratio at 2x load vs linear expectation
    fy_ratio = (mu_2x * 2.0) / (mu_nom * 1.0)
    drop_pct = (1.0 - mu_2x / mu_nom) * 100.0
    return CheckOutcome(
        Verdict.MYTH,
        (f"Tyre grip is load-sensitive: \u03bc FALLS as vertical load rises, so "
         f"doubling load does NOT double cornering force. On this model \u03bc drops "
         f"from {mu_nom:.2f} at {tire.FNOMIN:.0f} N to {mu_2x:.2f} at "
         f"{2*tire.FNOMIN:.0f} N ({drop_pct:.0f}% lower). Double the load buys only "
         f"~{fy_ratio:.2f}\u00d7 the lateral force, not 2\u00d7. This is exactly why "
         f"weight transfer costs you grip on the loaded tyre \u2014 and why lower CG "
         f"and managing load transfer matter."),
        #  COMPUTED: this quotes mu at FNOMIN and 2*FNOMIN from the live tyre
        #  model rather than restating a general truth, so the verdict moves if
        #  the model or the fit moves. The sources say where the underlying
        #  load-sensitivity data comes from, because the MODEL is only as good
        #  as the coefficients behind it.
        grounding="computed",
        sources=("Tire Test Consortium (TTC) data — verify for YOUR compound",
                 "Milliken, Race Car Vehicle Dynamics, ch. 2"),
        provenance=f"\u03bc({tire.FNOMIN:.0f}N)={mu_nom:.2f}, \u03bc({2*tire.FNOMIN:.0f}N)={mu_2x:.2f}")
_r_load_sensitivity.reference_claim = "Twice the vertical load gives twice the grip."


# --- RULE: camber always helps -----------------------------------------------
def _r_more_camber(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not (claim.has("camber") and claim.has("more grip", "always", "best",
                                              "maximi", "increase grip", "helps")):
        return None
    tire, Fz, _ = _tire_and_load(context)
    if tire is None:
        return preliminary(
            Verdict.DEPENDS,
            ("More negative camber is a trade, not a gain. It puts the tyre "
             "flatter on the road once the body rolls and the tyre deflects, "
             "which raises PEAK lateral grip up to a point \u2014 past that "
             "point you are standing the contact patch on its inner edge, "
             "losing grip, and giving away straight-line braking and traction "
             "as well. There is an optimum per compound and it is found on "
             "track or in tyre data, not by adding more."),
            need=_NEED_TIRE)
    opt_cam, _ = tire.optimal_camber(Fz or tire.FNOMIN)
    return CheckOutcome(
        Verdict.MYTH,
        (f"There's an OPTIMUM camber, not 'more is better'. For this tyre at "
         f"{(Fz or tire.FNOMIN):.0f} N the peak-grip camber is about "
         f"{opt_cam:.1f}\u00b0. Beyond it, contact patch pressure distribution "
         f"worsens and lateral grip falls again, while straight-line braking/"
         f"traction and tyre temperature also suffer. Tune toward "
         f"{opt_cam:.1f}\u00b0, then verify with your own thermal and TTC data."),
        grounding="physics",
        provenance=f"optimal camber \u2248 {opt_cam:.1f}\u00b0 @ {(Fz or tire.FNOMIN):.0f} N")
_r_more_camber.reference_claim = "More negative camber always means more grip."


# --- RULE: hot pressure = cold pressure (tyre pressure rises with temp) -------
def _r_pressure_temp(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not (claim.has("pressure") and claim.has("temp", "hot", "cold", "warm")):
        return None
    # This is a gas-law fact, answerable without a fitted model.
    return CheckOutcome(
        Verdict.TRUE,
        ("Correct in direction: hot tyre pressure is higher than cold. By Gay-"
         "Lussac (P/T constant at fixed volume), a tyre going from ~20\u00b0C cold to "
         "~80\u00b0C operating gains roughly (353/293\u22121) \u2248 20% absolute "
         "pressure \u2014 about +0.25\u20130.30 bar over a typical cold set. Set COLD "
         "pressures to land on your hot target; chasing a hot number directly will "
         "leave you cold-overinflated. Confirm the rise with your own tyre-temp "
         "logger."),
        grounding="physics",
        provenance="Gay-Lussac P/T; 293K\u219253K \u2248 +20% abs pressure")
_r_pressure_temp.reference_claim = "Cold and hot tyre pressures are basically the same."


# --- RULE: wider tyre always grips more --------------------------------------
def _r_wider_more_grip(claim: ParsedClaim, context: Any) -> CheckOutcome | None:
    if not (claim.has("wider", "width", "bigger tyre", "bigger tire", "fatter")
            and claim.has("grip", "faster", "more grip", "better")):
        return None
    return CheckOutcome(
        Verdict.DEPENDS,
        ("Wider isn't automatically more grip. A wider tyre spreads the same "
         "vertical load over more area, lowering contact pressure \u2014 which helps "
         "via load sensitivity (\u03bc rises as pressure/temp drop) only if the tyre "
         "still reaches operating temperature. On a light FSAE car a too-wide tyre "
         "can run cold and grip LESS, while adding unsprung mass, inertia and drag. "
         "The decision is a tyre-temperature and load-sensitivity trade, not a width "
         "contest \u2014 check it against your thermal model and TTC data."),
        grounding="physics",
        provenance="load-sensitivity + thermal trade-off; needs tyre-temp data")
_r_wider_more_grip.reference_claim = "A wider tyre always gives more grip."


RULES = [
    Rule("tires.load_sensitivity", "suspension", _r_load_sensitivity,
         keywords_any=("load",), priority=10),
    Rule("tires.more_camber", "suspension", _r_more_camber,
         keywords_any=("camber",), priority=20),
    Rule("tires.pressure_temp", "suspension", _r_pressure_temp,
         keywords_any=("pressure",), priority=30),
    Rule("tires.wider_more_grip", "suspension", _r_wider_more_grip,
         keywords_any=("wider", "width", "fatter", "bigger tyre", "bigger tire"),
         priority=40),
]
for _rule in RULES:
    register(_rule)
