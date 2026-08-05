# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Pedal-box PACKAGING, BALANCE-BAR bias and PEDAL-TRAVEL budget — the three things
that decide whether the pedal assembly actually fits, actually reaches the bias
you designed for, and actually stops before the pedal hits the floor.

WHY THIS MODULE EXISTS
----------------------
The Brakes tab already sizes the hydraulic chain: pedal ratio, master-cylinder
bore, caliper area, rotor radius -> line pressure -> brake torque. That answers
"can we make enough torque". It does not answer the three questions that stall a
pedal-box design review:

  1. IT DOESN'T FIT.  The assembly is longer than the bay it has to live in, and
     the team is staring at a CAD screenshot trying to find 50-100 mm. Shortening
     the pushrod buys a few mm and then runs out of thread. `stack_up()` breaks
     the installed length into named segments so the length has an OWNER, and
     `shorten_options()` enumerates every lever that buys X-length, each with the
     millimetres it returns AND what it costs in pedal force or pedal travel.
     `plan_shortening()` then assembles the cheapest combination that clears a
     stated deficit. This is the "any ideas?" question, answered with numbers.

  2. THE BIAS IS A SLIDER, NOT A PART.  A dual-master-cylinder balance bar sets
     bias with hardware: the two bores, the caliper areas, the rotor radii, and
     where the bar pivot sits. Picking "65% front" on a slider is a wish. The bar
     may be physically unable to reach it, or may reach it only at the very end of
     its adjustment, with no authority left to trim the car at a test day.
     `balance_bar_bias()` computes the bias the ASSEMBLED hardware makes,
     `bias_authority()` sweeps the bar through its travel to report the reachable
     bias band and the change per turn of the adjuster.

  3. THE PEDAL GOES TO THE FLOOR.  Every fix in (1) that shrinks the master
     cylinder or raises the pedal ratio is paid for in TRAVEL, and travel is the
     one budget nobody writes down. `pedal_travel()` adds up the fluid every part
     of the circuit swallows -- knockback, pad compression, caliper deflection,
     hose expansion, fluid compressibility, trapped air -- converts it to master-
     cylinder stroke and then to travel at the pedal pad. This is also what
     "brake lines mapped and calculated" actually needs: `line_volume_cc()` turns
     a routed line length into the volume it adds.

The three are COUPLED, which is the whole point. A bigger bore shortens the
cylinder and cuts travel but raises pedal effort by (d_new/d_old)^2. A higher
pedal ratio cuts effort and shortens the pivot-to-clevis arm but multiplies
travel. Answering any one in isolation is how a pedal box ends up needing a
re-spin. Every option this module returns carries its side effects with it.

THE HONESTY CONTRACT, APPLIED
-----------------------------
  * The EQUATIONS are statics and volume bookkeeping -- lever moment balance,
    area ratios, bulk-modulus compression. They are safe.
  * The PARAMETERS are not. Hose expansion per metre, pad compression under load,
    caliper housing deflection and pad knockback vary by part and by how well the
    system is bled. Every default here is a REPRESENTATIVE value that gives the
    right SHAPE and the right SENSITIVITY, and every result that used one is
    flagged `is_estimate=True`, exactly like `throttle_return` and `brakes`.
  * Measured beats modelled, always. `TravelParams.from_measured_stroke()` takes
    one bench measurement -- pump the pedal, measure the travel to a firm pedal --
    and back-calculates the total system compliance, which turns every subsequent
    trade study on this car from representative into calibrated.
  * Master-cylinder body lengths in `MC_FAMILIES` are placeholders for getting the
    stack-up SHAPE right before the parts land. A caliper on the actual cylinder
    always wins; `stack_up()` says so in a finding whenever a catalogue length is
    used instead of a measured one.

UNITS: mm for lengths (that is how a pedal box is drawn and measured), N for
force, bar for line pressure in the human-facing fields and Pa internally, cc for
fluid volume. Angles in degrees at the API, radians internally.

REFERENCES: standard lever statics for the balance bar (moment balance about each
clevis); Shigley for the compressibility relation dV = V*dP/K; FSAE / Formula
Student rules, Brake System (single pedal acting on all four wheels, two
independent hydraulic circuits, brake pedal shall withstand 2000 N).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

from .interfaces import Finding, Severity


# --------------------------------------------------------------------------- #
#  Representative parameter libraries.
#  Every number in this section is a REPRESENTATIVE value chosen to give the
#  right order of magnitude and the right sensitivity. None of them is measured
#  on your car. Results built on them carry is_estimate=True.
# --------------------------------------------------------------------------- #

#  Bulk modulus of brake fluid, Pa. Falls with temperature and with absorbed
#  water, which is exactly why a hot, wet system feels soft.
FLUID_BULK_MODULUS_PA = {
    "DOT 3": 1.45e9,
    "DOT 4": 1.50e9,
    "DOT 5.1": 1.50e9,
    "DOT 5 (silicone)": 1.10e9,   # noticeably more compressible; racers avoid it
}

#  Volumetric expansion of a pressurised line, cc per metre per bar.
#  Rubber hose is an order of magnitude worse than PTFE braided, which is why a
#  soft pedal is so often just the hoses.
HOSE_EXPANSION_CC_PER_M_PER_BAR = {
    "rubber hose": 3.0e-3,
    "PTFE braided (steel)": 8.0e-4,
    "hardline (steel tube)": 1.0e-4,
    "hardline (Cu-Ni tube)": 1.6e-4,
}

#  Fitting stack at the master-cylinder outlet: how much X-length the line exit
#  costs, INCLUDING the bend radius the hardline needs to turn away. This is the
#  "we also need to account for the lines coming out the rear of the MCs" item --
#  it is real length and it is routinely left out of a CAD envelope check.
FITTING_STACK_MM = {
    "straight fitting + hardline bend": 42.0,
    "45 deg fitting": 28.0,
    "90 deg fitting": 18.0,
    "90 deg banjo": 14.0,
    "bulkhead elbow at the MC face": 11.0,
}

#  Master-cylinder families. body_mm is the cylinder body length EXCLUDING the
#  pushrod and excluding anything at the outlet; stroke_mm is the usable stroke.
#  PLACEHOLDERS for getting the stack-up shape right before parts arrive -- always
#  replace with a caliper measurement of the cylinder you actually bought.
MC_FAMILIES = {
    "compact racing (short body)": dict(body_mm=88.0, stroke_mm=19.0,
                                        bores_mm=(15.9, 17.5, 19.0, 20.6)),
    "standard racing":             dict(body_mm=112.0, stroke_mm=25.4,
                                        bores_mm=(15.9, 17.5, 19.0, 20.6, 22.2)),
    "long stroke / large reservoir": dict(body_mm=134.0, stroke_mm=31.8,
                                          bores_mm=(17.5, 19.0, 20.6, 22.2, 25.4)),
}

#  Minimum thread engagement for an adjustable pushrod, expressed as a multiple
#  of the thread diameter. Below this the rod end is a liability, not an
#  adjustment -- which is the real reason "shorten the pushrod" runs out so fast.
MIN_THREAD_ENGAGEMENT_D = 1.5

#  Pushrod angular misalignment beyond which a plain clevis binds and you must be
#  on spherical rod ends at both ends, degrees.
PUSHROD_SPHERICAL_REQUIRED_DEG = 3.0
#  Beyond this the side load into the cylinder bore starts scoring the seal.
PUSHROD_MAX_SANE_DEG = 12.0


def _mm2_of_dia(d_mm: float) -> float:
    """Circle area in mm^2 from a diameter in mm."""
    return math.pi * (float(d_mm) ** 2) / 4.0


# =========================================================================== #
#  1.  LONGITUDINAL STACK-UP  --  where the length actually goes
# =========================================================================== #
@dataclass
class StackSegment:
    """One named contributor to the pedal box's installed X-length.

    `adjustable_mm` is how much of this segment could in principle be removed
    without changing the part -- pushrod thread, say. It is what separates "we can
    trim this" from "this is the part and the part is that long".
    """
    name: str
    length_mm: float
    adjustable_mm: float = 0.0
    measured: bool = False        # True if this came off a real part, not a catalogue
    note: str = ""

    def as_dict(self):
        return asdict(self)


@dataclass
class StackUpResult:
    """The installed length, segment by segment, against the space available."""
    segments: list[StackSegment]
    installed_mm: float
    available_mm: float
    deficit_mm: float             # >0 means it does NOT fit, by this much
    verdict: str                  # FITS / TIGHT / DOES NOT FIT
    is_estimate: bool
    findings: list[Finding] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def fits(self) -> bool:
        return self.deficit_mm <= 0.0

    def waterfall(self) -> list[dict]:
        """Segments sorted longest-first -- the order to go hunting in."""
        return [s.as_dict() for s in
                sorted(self.segments, key=lambda s: -s.length_mm)]

    def as_dict(self):
        return dict(
            segments=[s.as_dict() for s in self.segments],
            installed_mm=self.installed_mm, available_mm=self.available_mm,
            deficit_mm=self.deficit_mm, verdict=self.verdict,
            is_estimate=self.is_estimate,
            findings=[f.as_dict() for f in self.findings],
            notes=list(self.notes))

    def summary(self) -> str:
        head = (f"Pedal box stack-up: {self.installed_mm:.1f} mm installed vs "
                f"{self.available_mm:.1f} mm available -> {self.verdict}")
        if self.deficit_mm > 0:
            head += f" (need to find {self.deficit_mm:.1f} mm)"
        lines = [head]
        for s in sorted(self.segments, key=lambda s: -s.length_mm):
            tag = "measured" if s.measured else "catalogue/estimate"
            adj = (f", {s.adjustable_mm:.1f} mm of it adjustable"
                   if s.adjustable_mm > 0 else "")
            lines.append(f"    {s.length_mm:7.1f} mm  {s.name}  [{tag}{adj}]")
        return "\n".join(lines)


def stack_up(*,
             available_mm: float,
             pedal_lever_mm: float = 90.0,
             pedal_ratio: float = 5.0,
             pedal_rest_angle_deg: float = 25.0,
             balance_bar_stack_mm: float = 38.0,
             pushrod_mm: float = 55.0,
             pushrod_thread_dia_mm: float = 8.0,
             pushrod_engaged_mm: float | None = None,
             mc_body_mm: float | None = None,
             mc_family: str = "standard racing",
             mc_body_measured: bool = False,
             mc_outlet: str = "straight fitting + hardline bend",
             mc_outlet_mm: float | None = None,
             mount_plate_mm: float = 8.0,
             bulkhead_clearance_mm: float = 12.0,
             pedal_arc_clearance_mm: float = 10.0,
             tilt_deg: float = 0.0,
             tight_band_mm: float = 10.0) -> StackUpResult:
    """Break the pedal box's installed longitudinal length into owned segments.

    The chain runs from the pedal pad face rearward (in the car's -X sense, toward
    the front bulkhead) to the last thing behind the master cylinders:

        pad -> pivot  |  pivot -> bar clevis  |  balance bar hardware  |
        pushrod  |  MC body  |  MC outlet fitting + line bend  |
        mount plate  |  clearance to the bulkhead

    plus the arc clearance the pedal pad sweeps as it is pressed.

    Parameters worth knowing about
    ------------------------------
    pedal_rest_angle_deg : the pedal arm's angle from the X axis at rest. A more
        upright pedal (bigger angle) projects LESS length onto X, which is one of
        the cheapest ways to buy room -- at the cost of the driver's ankle angle.
    tilt_deg : if the master cylinders are mounted tilted rather than along X, the
        cylinder + pushrod stack only costs cos(tilt) of its true length.
    mc_body_mm : the measured cylinder body length. Leave it None to fall back to
        the `mc_family` catalogue, which flags the whole result as an estimate --
        a catalogue length is a placeholder, not your part.
    pushrod_engaged_mm : how much thread is currently engaged in the rod end. Used
        to work out how much of the pushrod is genuinely adjustable before the
        joint is unsafe. Defaults to a nominal 1.5*d already engaged.

    Returns a StackUpResult whose `deficit_mm` is the number the team is hunting
    for -- feed it straight to `shorten_options()`.
    """
    theta = math.radians(float(pedal_rest_angle_deg))
    tilt = math.radians(max(float(tilt_deg), 0.0))
    cos_tilt = math.cos(tilt)

    findings: list[Finding] = []
    notes: list[str] = []
    is_estimate = False

    # --- pedal geometry ----------------------------------------------------
    # X projection of the pedal arm at rest. This is the part of the box that
    # exists purely because the pedal is a lever.
    pad_to_pivot_x = float(pedal_lever_mm) * math.cos(theta)
    # The bar clevis sits at (lever / ratio) from the pivot, on the same arm.
    clevis_arm_mm = float(pedal_lever_mm) / max(float(pedal_ratio), 1e-6)
    pivot_to_clevis_x = clevis_arm_mm * math.cos(theta)

    # --- master cylinder ---------------------------------------------------
    if mc_body_mm is None:
        fam = MC_FAMILIES.get(mc_family) or MC_FAMILIES["standard racing"]
        mc_body_mm = float(fam["body_mm"])
        mc_body_measured = False
        is_estimate = True
        findings.append(Finding(
            check="mc_body_length_source", severity=Severity.MISSING,
            message=(f"Master-cylinder body length came from the '{mc_family}' "
                     f"catalogue placeholder ({mc_body_mm:.1f} mm), not from your "
                     f"part. Put a caliper on the cylinder and pass mc_body_mm -- "
                     f"body lengths differ by 20.0 mm to 40.0 mm across families and "
                     f"that is most of the deficit you are chasing."),
            subsystems=["brakes"]))
    else:
        mc_body_mm = float(mc_body_mm)
        if mc_body_measured:
            findings.append(Finding(
                check="mc_body_length_source", severity=Severity.OK,
                message=f"Master-cylinder body length {mc_body_mm:.1f} mm is a "
                        f"measured value.",
                subsystems=["brakes"]))
        else:
            is_estimate = True

    # --- the outlet: the thing the CAD envelope forgets ---------------------
    if mc_outlet_mm is None:
        mc_outlet_mm = FITTING_STACK_MM.get(mc_outlet)
        if mc_outlet_mm is None:
            mc_outlet_mm = FITTING_STACK_MM["straight fitting + hardline bend"]
            notes.append(f"Unknown outlet type '{mc_outlet}'; used the straight-"
                         f"fitting stack.")
        is_estimate = True
        findings.append(Finding(
            check="mc_outlet_stack", severity=Severity.WARN,
            message=(f"The line exit behind the cylinders is carrying "
                     f"{mc_outlet_mm:.1f} mm of the stack ('{mc_outlet}'), which "
                     f"includes the bend radius the hardline needs to turn away "
                     f"from the cylinder. This is the segment most often missing "
                     f"from a CAD envelope check, because the fitting is modelled "
                     f"but the bend is not."),
            subsystems=["brakes"]))
    else:
        mc_outlet_mm = float(mc_outlet_mm)

    # --- pushrod adjustability --------------------------------------------
    # Default to a typical as-built engagement of 2.5 thread diameters. That is
    # the state the deck describes: there IS some adjustment, it is just nowhere
    # near enough on its own.
    engaged = (float(pushrod_engaged_mm) if pushrod_engaged_mm is not None
               else 2.5 * float(pushrod_thread_dia_mm))
    min_engagement = MIN_THREAD_ENGAGEMENT_D * float(pushrod_thread_dia_mm)
    # Only thread engaged BEYOND the minimum can be wound out and still be a joint.
    pushrod_adjustable = max(engaged - min_engagement, 0.0)
    if pushrod_adjustable < 3.0:
        findings.append(Finding(
            check="pushrod_adjustment_exhausted", severity=Severity.WARN,
            message=(f"The pushrod has only {pushrod_adjustable:.1f} mm of length "
                     f"left before thread engagement drops under the "
                     f"{MIN_THREAD_ENGAGEMENT_D:.1f}x{pushrod_thread_dia_mm:.1f} mm "
                     f"= {min_engagement:.1f} mm minimum. Winding it out past that "
                     f"is not an adjustment, it is a failure mode -- the deficit has "
                     f"to come from somewhere else."),
            subsystems=["brakes"]))

    segments = [
        StackSegment("Pedal pad to pivot (X projection of the lever)",
                     pad_to_pivot_x, 0.0, False,
                     f"{pedal_lever_mm:.1f} mm arm at {pedal_rest_angle_deg:.0f} deg "
                     f"from X. A more upright pedal projects less."),
        StackSegment("Pedal arc clearance (pad sweep under braking)",
                     float(pedal_arc_clearance_mm), 0.0, False,
                     "Room the pad needs to travel without touching anything."),
        StackSegment("Pivot to balance-bar clevis",
                     pivot_to_clevis_x, 0.0, False,
                     f"Lever/ratio = {clevis_arm_mm:.1f} mm arm. Raising the pedal "
                     f"ratio pulls this in."),
        StackSegment("Balance bar + clevises + spherical bearing",
                     float(balance_bar_stack_mm), 0.0, False,
                     "Bar hardware between the pedal and the two pushrods."),
        StackSegment("Pushrod",
                     float(pushrod_mm) * cos_tilt, pushrod_adjustable * cos_tilt,
                     False,
                     f"{pushrod_adjustable:.1f} mm can be wound out before thread "
                     f"engagement hits the {min_engagement:.1f} mm floor."),
        StackSegment("Master cylinder body",
                     mc_body_mm * cos_tilt, 0.0, bool(mc_body_measured),
                     "The part. Only a different part changes this."),
        StackSegment("MC outlet fitting + hardline bend radius",
                     mc_outlet_mm, 0.0, False,
                     f"'{mc_outlet}'. A 90 deg banjo or a bulkhead elbow is the "
                     f"cheapest length in the whole stack."),
        StackSegment("Mount plate / bulkhead flange",
                     float(mount_plate_mm), 0.0, False, ""),
        StackSegment("Clearance to the front bulkhead",
                     float(bulkhead_clearance_mm), 0.0, False,
                     "Assembly and tolerance gap. Do not spend this one."),
    ]

    installed = sum(s.length_mm for s in segments)
    deficit = installed - float(available_mm)

    if deficit > 0:
        verdict = "DOES NOT FIT"
        findings.append(Finding(
            check="pedal_box_envelope", severity=Severity.FAIL,
            message=(f"The assembly is {deficit:.1f} mm longer than the "
                     f"{available_mm:.1f} mm available. Run shorten_options() "
                     f"against this deficit -- the pushrod alone can only return "
                     f"{pushrod_adjustable:.1f} mm of it."),
            subsystems=["brakes", "chassis"]))
    elif -deficit < float(tight_band_mm):
        verdict = "TIGHT"
        findings.append(Finding(
            check="pedal_box_envelope", severity=Severity.WARN,
            message=(f"It fits with only {-deficit:.1f} mm to spare, inside the "
                     f"{tight_band_mm:.1f} mm tight band. One fitting change or one "
                     f"thicker mount plate puts it over."),
            subsystems=["brakes", "chassis"]))
    else:
        verdict = "FITS"
        findings.append(Finding(
            check="pedal_box_envelope", severity=Severity.OK,
            message=f"Fits with {-deficit:.1f} mm to spare.",
            subsystems=["brakes", "chassis"]))

    if tilt_deg > 0:
        notes.append(f"Cylinders tilted {tilt_deg:.0f} deg: the pushrod and body "
                     f"only cost cos({tilt_deg:.0f} deg) = {cos_tilt:.3f} of their "
                     f"true length along X.")
    if is_estimate:
        notes.append("Some segments came from catalogue placeholders. The RANKING "
                     "of segments is trustworthy; the absolute total is provisional "
                     "until the real parts are measured.")

    return StackUpResult(segments=segments, installed_mm=installed,
                         available_mm=float(available_mm), deficit_mm=deficit,
                         verdict=verdict, is_estimate=is_estimate,
                         findings=findings, notes=notes)


# =========================================================================== #
#  2.  SHORTENING OPTIONS  --  the "any ideas?" question, with numbers
# =========================================================================== #
@dataclass
class ShortenOption:
    """One way to buy X-length, with what it returns and what it costs.

    `cost_rank` orders the menu: 0 is free (an adjustment you already own), 1 is a
    parts change with no performance penalty, 2 trades pedal force or travel, 3
    needs a design re-spin or re-analysis. `side_effects` carries the quantified
    consequences so nothing is bought blind.
    """
    name: str
    gain_mm: float
    cost_rank: int
    cost: str
    side_effects: dict = field(default_factory=dict)
    feasible: bool = True
    requires_recheck: list[str] = field(default_factory=list)

    def as_dict(self):
        return asdict(self)

    def label(self) -> str:
        return f"{self.gain_mm:+.1f} mm  {self.name}  ({self.cost})"


@dataclass
class ShortenPlan:
    """A combination of options that clears (or fails to clear) the deficit."""
    deficit_mm: float
    chosen: list[ShortenOption]
    total_gain_mm: float
    remaining_mm: float           # >0 means still too long after everything chosen
    solved: bool
    findings: list[Finding] = field(default_factory=list)

    def as_dict(self):
        return dict(deficit_mm=self.deficit_mm,
                    chosen=[o.as_dict() for o in self.chosen],
                    total_gain_mm=self.total_gain_mm,
                    remaining_mm=self.remaining_mm, solved=self.solved,
                    findings=[f.as_dict() for f in self.findings])

    def summary(self) -> str:
        lines = [f"Need {self.deficit_mm:.1f} mm. "
                 + ("SOLVED" if self.solved
                    else f"SHORT BY {self.remaining_mm:.1f} mm")
                 + f" with {len(self.chosen)} change(s):"]
        for o in self.chosen:
            lines.append("    " + o.label())
            for k, v in o.side_effects.items():
                lines.append(f"         -> {k}: {v}")
        if not self.solved:
            lines.append("    -> Every listed lever is spent and it is still too "
                         "long. This is a layout problem, not an adjustment "
                         "problem: move the bulkhead, go to a floor-mounted pedal, "
                         "or accept a different cylinder family.")
        return "\n".join(lines)


def shorten_options(stack: StackUpResult, *,
                    pedal_lever_mm: float = 90.0,
                    pedal_ratio: float = 5.0,
                    pedal_ratio_max: float = 6.5,
                    pedal_rest_angle_deg: float = 25.0,
                    pedal_rest_angle_max_deg: float = 40.0,
                    mc_bore_mm: float = 15.875,
                    mc_bore_up_mm: float | None = None,
                    mc_family: str = "standard racing",
                    mc_family_short: str = "compact racing (short body)",
                    mc_outlet: str = "straight fitting + hardline bend",
                    mc_outlet_alt: str = "90 deg banjo",
                    tilt_deg: float = 0.0,
                    tilt_max_deg: float = 10.0) -> list[ShortenOption]:
    """Enumerate every lever that buys longitudinal length, quantified.

    Each option reports the millimetres it returns and, where the physics gives a
    number, exactly what it costs -- pedal force as a ratio, pedal travel as a
    ratio, or a re-check that has to be run. Options are returned sorted by
    cost_rank then by gain, so the free ones come first and the ones that make you
    re-do work come last.

    Nothing here is applied automatically. It is a menu, priced.
    """
    opts: list[ShortenOption] = []
    seg = {s.name: s for s in stack.segments}

    # --- 0. free: adjustment you already own -------------------------------
    push = seg.get("Pushrod")
    if push is not None and push.adjustable_mm > 0.1:
        opts.append(ShortenOption(
            name="Wind the pushrod in to minimum safe thread engagement",
            gain_mm=push.adjustable_mm, cost_rank=0,
            cost="free, but it is then spent",
            side_effects={
                "pedal free play": "goes to zero -- re-set the pedal stop so the "
                                   "cylinder still returns fully to its rest port, "
                                   "or the brakes will drag as they heat",
                "further adjustment": "none left in this direction",
            },
            requires_recheck=["pedal free play / rest-port return"]))

    # --- 1. parts change, no performance penalty ---------------------------
    cur_outlet = FITTING_STACK_MM.get(mc_outlet,
                                      FITTING_STACK_MM["straight fitting + hardline bend"])
    alt_outlet = FITTING_STACK_MM.get(mc_outlet_alt)
    if alt_outlet is not None and alt_outlet < cur_outlet:
        opts.append(ShortenOption(
            name=f"Change the MC outlet from '{mc_outlet}' to '{mc_outlet_alt}'",
            gain_mm=cur_outlet - alt_outlet, cost_rank=1,
            cost="one fitting per cylinder, no performance penalty",
            side_effects={
                "pedal feel": "unchanged -- this removes packaging length, not fluid",
                "assembly": "one more sealing joint per circuit to bleed and "
                            "leak-check before the brake test",
            },
            requires_recheck=["leak check", "line routing clearance"]))

    # The cheapest length in the whole stack is usually here, so if the team is
    # already on the shortest fitting say so rather than staying silent.
    if alt_outlet is not None and alt_outlet >= cur_outlet:
        best = min(FITTING_STACK_MM.items(), key=lambda kv: kv[1])
        if best[1] < cur_outlet:
            opts.append(ShortenOption(
                name=f"Change the MC outlet to '{best[0]}' (shortest available)",
                gain_mm=cur_outlet - best[1], cost_rank=1,
                cost="one fitting per cylinder",
                side_effects={"pedal feel": "unchanged"},
                requires_recheck=["leak check"]))

    fam_now = MC_FAMILIES.get(mc_family)
    fam_short = MC_FAMILIES.get(mc_family_short)
    if fam_now and fam_short and fam_short["body_mm"] < fam_now["body_mm"]:
        lost_stroke = fam_now["stroke_mm"] - fam_short["stroke_mm"]
        opts.append(ShortenOption(
            name=f"Move to a '{mc_family_short}' cylinder body",
            gain_mm=fam_now["body_mm"] - fam_short["body_mm"], cost_rank=1,
            cost="buy different cylinders",
            side_effects={
                "usable stroke": f"drops {lost_stroke:.1f} mm to "
                                 f"{fam_short['stroke_mm']:.1f} mm -- only safe if "
                                 f"pedal_travel() says the circuit needs less than "
                                 f"that with a reserve",
                "lead time": "a parts order sits on the critical path",
            },
            feasible=True,
            requires_recheck=["pedal_travel() stroke demand vs the new stroke limit"]))

    # --- 2. trades pedal force or pedal travel -----------------------------
    if tilt_max_deg > tilt_deg:
        t_new = math.radians(float(tilt_max_deg))
        t_old = math.radians(float(tilt_deg))
        rod_plus_body = 0.0
        for nm in ("Pushrod", "Master cylinder body"):
            if nm in seg:
                # segments are already stored post-tilt; recover the true length
                rod_plus_body += seg[nm].length_mm / max(math.cos(t_old), 1e-9)
        gain = rod_plus_body * (math.cos(t_old) - math.cos(t_new))
        force_loss = 1.0 - math.cos(t_new - t_old)
        sane = tilt_max_deg <= PUSHROD_MAX_SANE_DEG
        opts.append(ShortenOption(
            name=f"Tilt the cylinders to {tilt_max_deg:.0f} deg off the X axis",
            gain_mm=gain, cost_rank=2,
            cost="geometry change; needs spherical rod ends",
            side_effects={
                "pushrod force reaching the piston":
                    f"-{force_loss*100:.1f}% (cosine loss), so pedal effort rises "
                    f"by about the same fraction",
                "side load into the bore":
                    f"{math.sin(t_new)*100:.0f}% of the rod load pushes sideways on "
                    f"the piston seal -- spherical rod ends at BOTH ends are "
                    f"mandatory past {PUSHROD_SPHERICAL_REQUIRED_DEG:.0f} deg",
                "sanity": ("within the usual limit" if sane else
                           f"past the {PUSHROD_MAX_SANE_DEG:.0f} deg point where "
                           f"seal scoring starts -- do not"),
            },
            feasible=sane,
            requires_recheck=["rod-end articulation through full pedal travel",
                              "hydraulic sizing at the higher pedal effort"]))

    if pedal_ratio_max > pedal_ratio:
        clevis_now = float(pedal_lever_mm) / max(pedal_ratio, 1e-6)
        clevis_new = float(pedal_lever_mm) / max(pedal_ratio_max, 1e-6)
        cosang = math.cos(math.radians(float(pedal_rest_angle_deg)))
        gain = (clevis_now - clevis_new) * cosang
        travel_mult = pedal_ratio_max / pedal_ratio
        force_mult = pedal_ratio / pedal_ratio_max
        opts.append(ShortenOption(
            name=f"Raise the pedal ratio from {pedal_ratio:.1f} to "
                 f"{pedal_ratio_max:.1f} (clevis moves toward the pivot)",
            gain_mm=gain, cost_rank=2,
            cost="pays for length and effort in travel",
            side_effects={
                "pedal effort to lock": f"x{force_mult:.2f} (lower -- this is the "
                                        f"good half of the trade)",
                "pedal travel to lock": f"x{travel_mult:.2f} (higher -- this is what "
                                        f"it costs, and it is the half that gets "
                                        f"forgotten)",
                "clevis load": f"x{travel_mult:.2f} into the bar and pushrod",
            },
            requires_recheck=["pedal_travel() against the available travel",
                              "brake pedal 2000 N check at the new clevis position"]))

    bore_up = (float(mc_bore_up_mm) if mc_bore_up_mm is not None
               else float(mc_bore_mm) * 1.10)
    if bore_up > mc_bore_mm:
        # A bigger bore needs less stroke for the same swept volume, which lets a
        # shorter-stroke (and so shorter-bodied) cylinder do the job.
        area_ratio = (bore_up / float(mc_bore_mm)) ** 2
        stroke_ratio = 1.0 / area_ratio
        fam = MC_FAMILIES.get(mc_family, MC_FAMILIES["standard racing"])
        stroke_saved = float(fam["stroke_mm"]) * (1.0 - stroke_ratio)
        # Body length scales with stroke roughly one-for-one over the stroke range.
        opts.append(ShortenOption(
            name=f"Go up in bore, {mc_bore_mm:.1f} -> {bore_up:.1f} mm, and take "
                 f"the shorter-stroke cylinder it allows",
            gain_mm=stroke_saved, cost_rank=2,
            cost="pedal effort rises as the square of the bore ratio",
            side_effects={
                "line pressure at a given pedal force": f"x{stroke_ratio:.2f} "
                    f"(lower)",
                "pedal force to reach lock-up": f"x{area_ratio:.2f} -- check this "
                    f"against what a driver can actually push before you commit",
                "pedal travel to lock": f"x{stroke_ratio:.2f} (shorter, the one "
                    f"genuinely free win here)",
            },
            requires_recheck=["Hydraulic sizing: torque made vs needed at the new "
                              "bore", "pedal effort against driver capability"]))

    # --- 3. layout re-spin -------------------------------------------------
    if pedal_rest_angle_max_deg > pedal_rest_angle_deg:
        c_now = math.cos(math.radians(float(pedal_rest_angle_deg)))
        c_new = math.cos(math.radians(float(pedal_rest_angle_max_deg)))
        gain = float(pedal_lever_mm) * (c_now - c_new)
        opts.append(ShortenOption(
            name=f"Stand the pedal up: rest angle {pedal_rest_angle_deg:.0f} -> "
                 f"{pedal_rest_angle_max_deg:.0f} deg from X",
            gain_mm=gain, cost_rank=3,
            cost="ergonomics and a pedal-face re-design",
            side_effects={
                "driver ankle angle": "steeper -- run it past every driver, not just "
                                      "the tallest one, before it is welded",
                "foot force direction": "less of the driver's push lines up with the "
                                        "pushrod, so effective pedal effort rises",
                "heel position": "the floor/heel rest almost certainly moves too",
            },
            requires_recheck=["driver fit for every driver",
                              "brake pedal 2000 N check on the new pedal",
                              "throttle and clutch pedal plane alignment"]))

    opts.append(ShortenOption(
        name="Move the balance bar behind the pedal (overlap the bar into the "
             "pedal's swept plane)",
        gain_mm=max(seg.get("Balance bar + clevises + spherical bearing",
                            StackSegment("", 0.0)).length_mm * 0.45, 0.0),
        cost_rank=3,
        cost="a genuine re-layout of the pedal box plate",
        side_effects={
            "clearance": "the bar and the pedal now share a volume -- this only "
                         "works if the swept-path check is clean through FULL "
                         "travel, not just at rest",
            "adjuster access": "the bias adjuster cable/knob has to still reach the "
                               "driver",
        },
        feasible=True,
        requires_recheck=["swept-path interference through full pedal travel",
                          "bias adjuster routing"]))

    opts.sort(key=lambda o: (o.cost_rank, -o.gain_mm))
    return opts


def plan_shortening(stack: StackUpResult,
                    options: list[ShortenOption] | None = None,
                    *, max_cost_rank: int = 2,
                    **option_kwargs) -> ShortenPlan:
    """Assemble the cheapest set of options that clears the stack-up deficit.

    Walks the menu cheapest-first and takes options until the deficit is covered,
    skipping anything marked infeasible or above `max_cost_rank`. This is a greedy
    pick by cost, not a global optimum -- deliberately, because the ordering that
    matters to a team is "what can I do without buying anything, then without
    changing the design", and a greedy walk down that order IS that answer.

    If the whole affordable menu still does not cover the deficit, `solved` is
    False and the plan says so rather than quietly returning a partial fix.
    """
    if options is None:
        options = shorten_options(stack, **option_kwargs)

    deficit = max(stack.deficit_mm, 0.0)
    findings: list[Finding] = []

    if deficit <= 0:
        return ShortenPlan(deficit_mm=0.0, chosen=[], total_gain_mm=0.0,
                           remaining_mm=0.0, solved=True,
                           findings=[Finding(
                               check="shorten_plan", severity=Severity.OK,
                               message="Nothing to do -- the assembly already fits.",
                               subsystems=["brakes"])])

    chosen: list[ShortenOption] = []
    total = 0.0
    for o in options:
        if total >= deficit:
            break
        if not o.feasible or o.cost_rank > max_cost_rank:
            continue
        if o.gain_mm <= 0.1:
            continue
        chosen.append(o)
        total += o.gain_mm

    remaining = max(deficit - total, 0.0)
    solved = remaining <= 0.0

    if solved:
        findings.append(Finding(
            check="shorten_plan", severity=Severity.OK,
            message=(f"{total:.1f} mm recovered against a {deficit:.1f} mm deficit "
                     f"using {len(chosen)} change(s), none above cost rank "
                     f"{max_cost_rank}. Every option carries a re-check -- run them "
                     f"before this counts as closed."),
            subsystems=["brakes"]))
    else:
        findings.append(Finding(
            check="shorten_plan", severity=Severity.FAIL,
            message=(f"The affordable menu returns {total:.1f} mm of the "
                     f"{deficit:.1f} mm needed, leaving {remaining:.1f} mm. This is "
                     f"no longer an adjustment problem. The honest options are a "
                     f"cylinder family with a shorter body, a floor-mounted pedal, "
                     f"or moving the bulkhead -- all of which are chassis "
                     f"conversations, so start them now rather than at the next "
                     f"design review."),
            subsystems=["brakes", "chassis"]))

    # A plan built entirely on estimated segment lengths is a plan built on sand.
    if stack.is_estimate:
        findings.append(Finding(
            check="shorten_plan_provenance", severity=Severity.WARN,
            message=("This plan is priced against catalogue segment lengths. "
                     "Measure the real cylinders and fittings before ordering "
                     "anything -- the deficit itself could move by tens of mm."),
            subsystems=["brakes"]))

    return ShortenPlan(deficit_mm=deficit, chosen=chosen, total_gain_mm=total,
                       remaining_mm=remaining, solved=solved, findings=findings)


# =========================================================================== #
#  3.  BALANCE BAR  --  the bias the hardware actually makes
# =========================================================================== #
@dataclass
class CircuitSpec:
    """One hydraulic circuit: its cylinder, its calipers, its rotor.

    `pistons_per_side` drives CLAMP force (only one side's area presses the pad);
    `opposed` drives VOLUME (in an opposed caliper every piston moves, so the
    fluid demand is double). Getting these two confused is the classic reason a
    hand-calculated pedal travel comes out half of what the car does.
    """
    mc_bore_mm: float
    caliper_piston_dia_mm: float
    pistons_per_side: int = 2
    opposed: bool = True
    pad_mu: float = 0.45
    rotor_dia_mm: float = 220.0
    effective_radius_frac: float = 0.92   # r_eff as a fraction of the outer radius
    n_corners: int = 2                    # corners this circuit feeds

    @property
    def mc_area_mm2(self) -> float:
        return _mm2_of_dia(self.mc_bore_mm)

    @property
    def clamp_area_mm2(self) -> float:
        """Area that generates clamp force at one caliper (one side's pistons)."""
        return _mm2_of_dia(self.caliper_piston_dia_mm) * max(int(self.pistons_per_side), 1)

    @property
    def swept_area_mm2(self) -> float:
        """Total piston area that MOVES at one caliper -- the fluid demand area."""
        n = max(int(self.pistons_per_side), 1) * (2 if self.opposed else 1)
        return _mm2_of_dia(self.caliper_piston_dia_mm) * n

    @property
    def r_eff_mm(self) -> float:
        return 0.5 * float(self.rotor_dia_mm) * float(self.effective_radius_frac)

    def axle_torque_Nm(self, pressure_bar: float) -> float:
        """Brake torque this circuit's whole axle makes at a line pressure."""
        p_pa = float(pressure_bar) * 1e5
        clamp_N = p_pa * (self.clamp_area_mm2 * 1e-6)     # per caliper
        torque_per_corner = 2.0 * clamp_N * float(self.pad_mu) * (self.r_eff_mm * 1e-3)
        return torque_per_corner * max(int(self.n_corners), 1)


@dataclass
class BalanceBarResult:
    """What the assembled balance bar and cylinders actually deliver."""
    bar_offset_mm: float          # + = pivot toward the FRONT clevis
    force_front_N: float
    force_rear_N: float
    pressure_front_bar: float
    pressure_rear_bar: float
    torque_front_Nm: float
    torque_rear_Nm: float
    bias_front: float             # torque bias, 0-1
    findings: list[Finding] = field(default_factory=list)

    def as_dict(self):
        d = asdict(self)
        d["findings"] = [f.as_dict() for f in self.findings]
        return d


def balance_bar_bias(*, pedal_force_N: float, pedal_ratio: float,
                     front: CircuitSpec, rear: CircuitSpec,
                     bar_length_mm: float = 60.0,
                     bar_offset_mm: float = 0.0) -> BalanceBarResult:
    """Bias, pressures and torques from the actual hardware, not from a slider.

    The bar is a lever loaded at its pivot with a reaction at each clevis. Moment
    balance about each clevis gives the split: the clevis NEARER the pivot carries
    the larger share, so moving the pivot toward the front cylinder raises front
    pressure. With `a` = pivot-to-front-clevis and `b` = pivot-to-rear-clevis,
    a + b = L:

        F_front = F_rod * b / L        F_rear = F_rod * a / L

    Those forces divide by their OWN cylinder areas, so two different bores give
    two different pressures from the same bar position -- which is precisely why
    bias is a hardware question and not a preference.

    `bar_offset_mm` is measured positive toward the FRONT clevis.
    """
    L = max(float(bar_length_mm), 1e-6)
    e = float(bar_offset_mm)
    a = L / 2.0 - e          # pivot -> front clevis
    b = L / 2.0 + e          # pivot -> rear clevis

    findings: list[Finding] = []
    if a <= 0 or b <= 0:
        findings.append(Finding(
            check="balance_bar_offset", severity=Severity.FAIL,
            message=(f"A {e:+.1f} mm offset puts the pivot outside the "
                     f"{L:.1f} mm clevis span. The bar cannot be assembled like "
                     f"that."),
            subsystems=["brakes"]))
        a = max(a, 1e-6)
        b = max(b, 1e-6)

    F_rod = float(pedal_force_N) * float(pedal_ratio)
    F_f = F_rod * b / L
    F_r = F_rod * a / L

    p_f_bar = (F_f / (front.mc_area_mm2 * 1e-6)) / 1e5
    p_r_bar = (F_r / (rear.mc_area_mm2 * 1e-6)) / 1e5

    T_f = front.axle_torque_Nm(p_f_bar)
    T_r = rear.axle_torque_Nm(p_r_bar)
    bias = T_f / max(T_f + T_r, 1e-9)

    # A bar sitting hard against one end has no trim authority left in that
    # direction -- a test-day problem that is free to catch at the design stage.
    frac_off_centre = abs(e) / (L / 2.0)
    if frac_off_centre > 0.75:
        findings.append(Finding(
            check="bar_authority", severity=Severity.WARN,
            message=(f"The bar is {frac_off_centre*100:.0f}% of the way to its end "
                     f"stop to make {bias*100:.0f}% front. There is almost no trim "
                     f"left in that direction, so the driver cannot dial bias "
                     f"further at a test day. Change a BORE to re-centre the bar "
                     f"instead -- that is what the bore choice is for."),
            subsystems=["brakes"]))
    elif frac_off_centre < 0.35:
        findings.append(Finding(
            check="bar_authority", severity=Severity.OK,
            message=(f"The bar sits {frac_off_centre*100:.0f}% off centre, so there "
                     f"is trim authority in both directions."),
            subsystems=["brakes"]))

    return BalanceBarResult(
        bar_offset_mm=e, force_front_N=F_f, force_rear_N=F_r,
        pressure_front_bar=p_f_bar, pressure_rear_bar=p_r_bar,
        torque_front_Nm=T_f, torque_rear_Nm=T_r, bias_front=bias,
        findings=findings)


@dataclass
class BiasAuthority:
    """The bias band the hardware can reach, and how fast the adjuster moves it."""
    bias_min: float
    bias_max: float
    bias_at_centre: float
    target_bias: float | None
    target_reachable: bool
    offset_for_target_mm: float | None
    bias_per_turn: float          # change in front bias per full turn of the adjuster
    thread_pitch_mm: float
    sweep: list[dict] = field(default_factory=list)
    findings: list[Finding] = field(default_factory=list)

    def as_dict(self):
        d = asdict(self)
        d["findings"] = [f.as_dict() for f in self.findings]
        return d

    def summary(self) -> str:
        lines = [f"Reachable front bias: {self.bias_min*100:.1f}% to "
                 f"{self.bias_max*100:.1f}% "
                 f"(centred bar = {self.bias_at_centre*100:.1f}%)"]
        lines.append(f"Adjuster: {self.bias_per_turn*100:.2f}% bias per turn "
                     f"({self.thread_pitch_mm:.2f} mm pitch)")
        if self.target_bias is not None:
            if self.target_reachable:
                lines.append(f"Target {self.target_bias*100:.0f}% front is reachable "
                             f"at {self.offset_for_target_mm:+.2f} mm bar offset.")
            else:
                lines.append(f"Target {self.target_bias*100:.0f}% front is NOT "
                             f"reachable with these bores -- change a bore.")
        return "\n".join(lines)


def bias_authority(*, pedal_force_N: float, pedal_ratio: float,
                   front: CircuitSpec, rear: CircuitSpec,
                   bar_length_mm: float = 60.0,
                   max_offset_mm: float | None = None,
                   target_bias: float | None = None,
                   thread_pitch_mm: float = 1.058,   # 3/8"-24 UNF
                   n: int = 41) -> BiasAuthority:
    """Sweep the bar through its travel: what bias CAN this hardware make?

    Answers the three questions a bias slider cannot:
      * what band of bias do these two bores and calipers actually cover,
      * is my target inside it, and at what bar position,
      * how much does one turn of the adjuster move the bias (the number a driver
        needs when they ask for "a bit more front" between runs).

    `max_offset_mm` defaults to 40% of the half-span, which is about where a real
    bar runs out of usable articulation.
    """
    L = float(bar_length_mm)
    e_max = (float(max_offset_mm) if max_offset_mm is not None
             else 0.40 * (L / 2.0))
    n = max(int(n), 3)

    sweep = []
    for i in range(n):
        e = -e_max + (2.0 * e_max) * i / (n - 1)
        r = balance_bar_bias(pedal_force_N=pedal_force_N, pedal_ratio=pedal_ratio,
                             front=front, rear=rear, bar_length_mm=L,
                             bar_offset_mm=e)
        sweep.append(dict(offset_mm=e, bias_front=r.bias_front,
                          pressure_front_bar=r.pressure_front_bar,
                          pressure_rear_bar=r.pressure_rear_bar))

    biases = [s["bias_front"] for s in sweep]
    b_min, b_max = min(biases), max(biases)
    centre = balance_bar_bias(pedal_force_N=pedal_force_N, pedal_ratio=pedal_ratio,
                              front=front, rear=rear, bar_length_mm=L,
                              bar_offset_mm=0.0).bias_front

    findings: list[Finding] = []
    reachable = False
    e_target: float | None = None
    if target_bias is not None:
        t = float(target_bias)
        reachable = (b_min - 1e-9) <= t <= (b_max + 1e-9)
        if reachable:
            # bias is monotonic in offset for fixed hardware -- bisect for the offset
            lo, hi = -e_max, e_max
            f_lo = sweep[0]["bias_front"]
            for _ in range(80):
                mid = 0.5 * (lo + hi)
                bm = balance_bar_bias(pedal_force_N=pedal_force_N,
                                      pedal_ratio=pedal_ratio, front=front,
                                      rear=rear, bar_length_mm=L,
                                      bar_offset_mm=mid).bias_front
                if (bm < t) == (f_lo < t):
                    lo = mid
                    f_lo = bm
                else:
                    hi = mid
            e_target = 0.5 * (lo + hi)
            near_end = abs(e_target) > 0.75 * e_max
            findings.append(Finding(
                check="target_bias_reachable",
                severity=Severity.WARN if near_end else Severity.OK,
                message=(f"{t*100:.0f}% front needs the bar at "
                         f"{e_target:+.2f} mm"
                         + (f" -- that is {abs(e_target)/e_max*100:.0f}% of the way "
                            f"to the stop, so there is little trim left. Re-centre "
                            f"it by changing a bore."
                            if near_end else
                            f", comfortably inside the bar's travel.")),
                subsystems=["brakes"]))
        else:
            # which bore to change, and which way
            direction = ("larger front bore or smaller rear bore"
                         if t < b_min else
                         "smaller front bore or larger rear bore")
            findings.append(Finding(
                check="target_bias_reachable", severity=Severity.FAIL,
                message=(f"{t*100:.0f}% front is outside everything these bores can "
                         f"make ({b_min*100:.1f}%-{b_max*100:.1f}%). No bar position "
                         f"gets there. Fit a {direction}. Bias is set by the "
                         f"hardware; the bar only trims it."),
                subsystems=["brakes"]))

    # d(bias)/d(turn) about the centre -- the number to quote to a driver.
    half = 0.5 * float(thread_pitch_mm)
    b_hi = balance_bar_bias(pedal_force_N=pedal_force_N, pedal_ratio=pedal_ratio,
                            front=front, rear=rear, bar_length_mm=L,
                            bar_offset_mm=+half).bias_front
    b_lo = balance_bar_bias(pedal_force_N=pedal_force_N, pedal_ratio=pedal_ratio,
                            front=front, rear=rear, bar_length_mm=L,
                            bar_offset_mm=-half).bias_front
    per_turn = b_hi - b_lo

    if abs(per_turn) > 0.04:
        findings.append(Finding(
            check="bias_adjuster_sensitivity", severity=Severity.WARN,
            message=(f"One turn of the adjuster moves bias {per_turn*100:.1f}% -- "
                     f"that is coarse. A driver asking for 'a little more front' "
                     f"will overshoot. A finer thread or a longer bar gives finer "
                     f"trim."),
            subsystems=["brakes"]))

    return BiasAuthority(
        bias_min=b_min, bias_max=b_max, bias_at_centre=centre,
        target_bias=(float(target_bias) if target_bias is not None else None),
        target_reachable=reachable, offset_for_target_mm=e_target,
        bias_per_turn=per_turn, thread_pitch_mm=float(thread_pitch_mm),
        sweep=sweep, findings=findings)


# =========================================================================== #
#  4.  PEDAL TRAVEL  --  the budget nobody writes down
# =========================================================================== #
@dataclass
class TravelParams:
    """Compliance parameters for the travel budget.

    Every default is REPRESENTATIVE. The shape and the sensitivities are right;
    the absolute travel is provisional until `calibrated` is set from a bench
    measurement (see `from_measured_stroke`).
    """
    # --- per-piston geometry consumed before any pressure is made ---
    knockback_mm: float = 0.15        # running clearance the pistons retract to
    pad_compression_mm: float = 0.10  # pad material squash at working pressure
    # --- pressure-proportional compliance ---
    caliper_cc_per_bar: float = 2.0e-3    # housing spread, per caliper
    hose_type: str = "PTFE braided (steel)"
    hardline_type: str = "hardline (steel tube)"
    fluid: str = "DOT 4"
    # --- trapped air: the single biggest cause of a long pedal ---
    air_cc: float = 0.0
    # --- mechanical free play before the cylinder starts working ---
    free_play_mm: float = 1.5         # at the pushrod, not at the pad
    # --- global compliance scale, set by calibration ---
    # One bench measurement cannot separate hose expansion from pad compression
    # from knockback. What it CAN do is scale them all together so the model
    # reproduces the real pedal. This is that single honest multiplier: 1.0 means
    # "straight off the representative values", and `calibrate_travel_params`
    # solves for the value that matches a measured travel.
    compliance_scale: float = 1.0
    calibrated: bool = False
    fitted_to: str = ""

    def bulk_modulus_pa(self) -> float:
        return FLUID_BULK_MODULUS_PA.get(self.fluid,
                                         FLUID_BULK_MODULUS_PA["DOT 4"])

    def hose_cc_per_m_per_bar(self) -> float:
        return HOSE_EXPANSION_CC_PER_M_PER_BAR.get(
            self.hose_type, HOSE_EXPANSION_CC_PER_M_PER_BAR["PTFE braided (steel)"])

    def hardline_cc_per_m_per_bar(self) -> float:
        return HOSE_EXPANSION_CC_PER_M_PER_BAR.get(
            self.hardline_type, HOSE_EXPANSION_CC_PER_M_PER_BAR["hardline (steel tube)"])


@dataclass
class VolumeItem:
    """One consumer of fluid, in cc, with why it exists."""
    name: str
    volume_cc: float
    pressure_dependent: bool
    note: str = ""

    def as_dict(self):
        return asdict(self)


@dataclass
class TravelResult:
    """Fluid demand -> cylinder stroke -> travel at the pedal pad."""
    items: list[VolumeItem]
    total_cc: float
    mc_stroke_mm: float
    pedal_travel_mm: float           # at the pad, including free play
    available_travel_mm: float
    mc_stroke_limit_mm: float
    stroke_utilisation: float        # stroke demanded / stroke available
    verdict: str                     # PASS / TIGHT / FAIL
    is_estimate: bool
    findings: list[Finding] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def as_dict(self):
        return dict(items=[i.as_dict() for i in self.items],
                    total_cc=self.total_cc, mc_stroke_mm=self.mc_stroke_mm,
                    pedal_travel_mm=self.pedal_travel_mm,
                    available_travel_mm=self.available_travel_mm,
                    mc_stroke_limit_mm=self.mc_stroke_limit_mm,
                    stroke_utilisation=self.stroke_utilisation,
                    verdict=self.verdict, is_estimate=self.is_estimate,
                    findings=[f.as_dict() for f in self.findings],
                    notes=list(self.notes))

    def biggest(self, k: int = 3) -> list[VolumeItem]:
        """The k largest consumers -- where to go if the pedal is long."""
        return sorted(self.items, key=lambda i: -i.volume_cc)[:k]

    def summary(self) -> str:
        lines = [f"Pedal travel: {self.pedal_travel_mm:.1f} mm at the pad of "
                 f"{self.available_travel_mm:.1f} mm available -> {self.verdict}",
                 f"  {self.total_cc:.3f} cc demanded, "
                 f"{self.mc_stroke_mm:.1f} mm of cylinder stroke "
                 f"({self.stroke_utilisation*100:.0f}% of the "
                 f"{self.mc_stroke_limit_mm:.1f} mm limit)"]
        for i in self.biggest(4):
            lines.append(f"    {i.volume_cc:6.3f} cc  {i.name}")
        return "\n".join(lines)


def line_volume_cc(length_mm: float, inner_dia_mm: float) -> float:
    """Internal volume of a run of line, cc. The 'lines mapped and calculated' bit.

    Routing length is not free: it is fluid you have to move, it is fluid that
    compresses, and in flexible hose it is wall that expands. Measure the routed
    length off the CAD (not the straight-line distance) and put it in.
    """
    return _mm2_of_dia(inner_dia_mm) * float(length_mm) / 1000.0


def pedal_travel(*, circuit: CircuitSpec,
                 line_pressure_bar: float,
                 pedal_ratio: float,
                 hose_length_m: float = 0.6,
                 hardline_length_m: float = 1.8,
                 line_inner_dia_mm: float = 3.2,
                 params: TravelParams | None = None,
                 available_travel_mm: float = 60.0,
                 mc_stroke_limit_mm: float = 25.4,
                 tight_frac: float = 0.80) -> TravelResult:
    """Add up every consumer of fluid and convert it to travel at the pedal pad.

    The chain, in order of how much it usually costs:

      knockback      pistons retract to a running clearance and have to be pushed
                     back out before any pad touches anything. Scales with TOTAL
                     piston area, so an opposed caliper costs double.
      pad compression the friction material squashes under clamp load.
      caliper spread  the housing opens up; small per bar but every bar counts.
      hose expansion  the wall stretches. Rubber is ~4x PTFE braided, per metre.
      fluid compression the fluid itself, over the WHOLE circuit volume.
      trapped air     the killer. A single cc of air is worth more travel than
                     everything else on this list put together.

    `line_pressure_bar` should be the pressure at the design stop -- take it from
    `balance_bar_bias()` so the travel is computed at the pressure the bias
    actually produces, not at a round number.

    Returns travel AT THE PAD, which is what a driver feels and what the pedal box
    has to physically allow.
    """
    p = params or TravelParams()
    P = max(float(line_pressure_bar), 0.0)
    n_cal = max(int(circuit.n_corners), 1)

    items: list[VolumeItem] = []

    # --- geometric, pressure-independent ----------------------------------
    swept_mm2 = circuit.swept_area_mm2 * n_cal
    v_knock = swept_mm2 * float(p.knockback_mm) / 1000.0
    items.append(VolumeItem(
        "Piston knockback / running clearance", v_knock, False,
        f"{swept_mm2:.1f} mm2 of moving piston across {n_cal} caliper(s) x "
        f"{p.knockback_mm:.2f} mm. Wheel-bearing and hub compliance make this "
        f"worse on track than on the bench."))

    v_pad = swept_mm2 * float(p.pad_compression_mm) / 1000.0
    items.append(VolumeItem(
        "Pad compression", v_pad, False,
        f"{p.pad_compression_mm:.2f} mm of material squash. New pads and bedded "
        f"pads differ; a soft compound is worse."))

    # --- pressure-proportional --------------------------------------------
    v_cal = float(p.caliper_cc_per_bar) * n_cal * P
    items.append(VolumeItem(
        "Caliper housing deflection", v_cal, True,
        f"{p.caliper_cc_per_bar*1000:.3f} cc/kbar per caliper at {P:.0f} bar."))

    v_hose = p.hose_cc_per_m_per_bar() * float(hose_length_m) * n_cal * P
    items.append(VolumeItem(
        f"Hose expansion ({p.hose_type})", v_hose, True,
        f"{hose_length_m:.2f} m per corner x {n_cal} corner(s) at {P:.0f} bar. "
        f"Switching rubber for PTFE braided is the cheapest firm-pedal fix there "
        f"is."))

    v_hard = p.hardline_cc_per_m_per_bar() * float(hardline_length_m) * P
    items.append(VolumeItem(
        f"Hardline expansion ({p.hardline_type})", v_hard, True,
        f"{hardline_length_m:.2f} m of routed hardline at {P:.0f} bar."))

    # Whole-circuit fluid volume: lines + the swept caliper volume.
    v_lines = (line_volume_cc(float(hardline_length_m) * 1000.0, line_inner_dia_mm)
               + line_volume_cc(float(hose_length_m) * 1000.0 * n_cal,
                                line_inner_dia_mm))
    v_fluid_total = v_lines + v_knock + v_pad
    K = p.bulk_modulus_pa()
    v_comp = v_fluid_total * (P * 1e5) / K
    items.append(VolumeItem(
        "Fluid compressibility", v_comp, True,
        f"{v_fluid_total:.3f} cc of {p.fluid} at {P:.0f} bar, K={K/1e9:.2f} GPa. "
        f"Hot or water-contaminated fluid is softer than this."))

    # --- air ---------------------------------------------------------------
    if p.air_cc > 0:
        # Isothermal: trapped air at ~1 bar abs squeezes to V*(1/(1+P_gauge)).
        v_air = float(p.air_cc) * (1.0 - 1.0 / (1.0 + P))
        items.append(VolumeItem(
            "Trapped air (compressed)", v_air, True,
            f"{p.air_cc:.3f} cc of air at atmospheric collapses to almost nothing "
            f"at {P:.0f} bar, and every cc of that collapse is pedal travel you "
            f"paid for. Bleed it out."))

    # Apply the single global compliance scale. Before calibration this is 1.0
    # and changes nothing; after calibration it is the one multiplier that makes
    # the whole budget reproduce a measured pedal.
    scale = float(p.compliance_scale)
    if scale != 1.0:
        items = [VolumeItem(i.name, i.volume_cc * scale, i.pressure_dependent,
                            i.note) for i in items]

    total_cc = sum(i.volume_cc for i in items)

    # --- to stroke and to travel ------------------------------------------
    A_mc_cm2 = circuit.mc_area_mm2 / 100.0
    stroke_mm = (total_cc / max(A_mc_cm2, 1e-9)) * 10.0
    travel_mm = (stroke_mm + float(p.free_play_mm)) * float(pedal_ratio)

    util = stroke_mm / max(float(mc_stroke_limit_mm), 1e-9)

    findings: list[Finding] = []
    notes: list[str] = []

    if travel_mm > float(available_travel_mm) or util > 1.0:
        verdict = "FAIL"
        if util > 1.0:
            findings.append(Finding(
                check="mc_stroke", severity=Severity.FAIL,
                message=(f"The circuit demands {stroke_mm:.1f} mm of cylinder "
                         f"stroke against a {mc_stroke_limit_mm:.1f} mm limit. The "
                         f"piston bottoms before the brakes are fully applied -- "
                         f"this is a pedal that goes to the floor, not a soft one."),
                subsystems=["brakes"]))
        if travel_mm > float(available_travel_mm):
            findings.append(Finding(
                check="pedal_travel", severity=Severity.FAIL,
                message=(f"{travel_mm:.1f} mm of travel at the pad against "
                         f"{available_travel_mm:.1f} mm available. Go after the "
                         f"largest consumers first: "
                         + ", ".join(i.name for i in
                                     sorted(items, key=lambda x: -x.volume_cc)[:2])
                         + "."),
                subsystems=["brakes"]))
    elif (travel_mm > tight_frac * float(available_travel_mm)
          or util > tight_frac):
        verdict = "TIGHT"
        findings.append(Finding(
            check="pedal_travel", severity=Severity.WARN,
            message=(f"{travel_mm:.1f} mm of travel uses "
                     f"{travel_mm/max(available_travel_mm,1e-9)*100:.0f}% of what "
                     f"is available, and the cylinder is at {util*100:.0f}% of its "
                     f"stroke. That leaves nothing for pad wear, a hot circuit, or "
                     f"a bad bleed -- all of which only ever make it longer."),
            subsystems=["brakes"]))
    else:
        verdict = "PASS"
        findings.append(Finding(
            check="pedal_travel", severity=Severity.OK,
            message=(f"{travel_mm:.1f} mm at the pad, cylinder at {util*100:.0f}% "
                     f"of stroke. Reserve for wear and heat."),
            subsystems=["brakes"]))

    if p.air_cc > 0:
        findings.append(Finding(
            check="trapped_air", severity=Severity.WARN,
            message=(f"This budget includes {p.air_cc:.3f} cc of trapped air. If "
                     f"you are modelling a badly-bled system to see the effect, "
                     f"good. If that is your real state, bleed it before you "
                     f"conclude anything about the cylinder bore."),
            subsystems=["brakes"]))

    if not p.calibrated:
        findings.append(Finding(
            check="travel_provenance", severity=Severity.MISSING,
            message=("Knockback, pad compression, hose and caliper compliance are "
                     "representative values, not measured on this car. The RANKING "
                     "of consumers and the sensitivity to bore and ratio are "
                     "trustworthy; the absolute travel is provisional. One bench "
                     "measurement -- pump to a firm pedal, measure the travel -- "
                     "fixes that via TravelParams.from_measured_stroke()."),
            subsystems=["brakes"]))
        notes.append("Uncalibrated: use for trades, not as an absolute pedal "
                     "travel.")

    return TravelResult(
        items=items, total_cc=total_cc, mc_stroke_mm=stroke_mm,
        pedal_travel_mm=travel_mm, available_travel_mm=float(available_travel_mm),
        mc_stroke_limit_mm=float(mc_stroke_limit_mm), stroke_utilisation=util,
        verdict=verdict, is_estimate=not p.calibrated,
        findings=findings, notes=notes)


def calibrate_travel_params(*, measured_pedal_travel_mm: float,
                            circuit: CircuitSpec,
                            line_pressure_bar: float,
                            pedal_ratio: float,
                            base: TravelParams | None = None,
                            fitted_to: str = "bench measurement",
                            **travel_kwargs) -> TravelParams:
    """Scale the compliance parameters so the model reproduces ONE real measurement.

    Pump the pedal to a firm stop at a known line pressure, measure the travel at
    the pad, pass it in. This solves for the single multiplier on the compliance
    terms that makes the model match, and returns a `calibrated=True` parameter
    set. Every trade study run afterwards -- bore, ratio, hose type -- is then
    anchored to this car instead of to a handbook.

    It deliberately scales ALL the compliance terms together rather than pretending
    one measurement can separate hose expansion from pad compression. It cannot.
    What it can do is stop the absolute number being a guess.
    """
    p0 = base or TravelParams()
    ref = pedal_travel(circuit=circuit, line_pressure_bar=line_pressure_bar,
                       pedal_ratio=pedal_ratio, params=p0, **travel_kwargs)
    target = float(measured_pedal_travel_mm)
    # Free play is mechanical and separately known; only the FLUID part of the
    # travel scales with compliance, so the scale is solved on that part alone.
    free = p0.free_play_mm * float(pedal_ratio)
    modelled_fluid = max(ref.pedal_travel_mm - free, 1e-9)
    measured_fluid = max(target - free, 1e-9)
    k = (measured_fluid / modelled_fluid) * float(p0.compliance_scale)

    if target <= free:
        raise ValueError(
            f"A measured travel of {target:.1f} mm is at or below the "
            f"{free:.1f} mm of pure free play implied by free_play_mm x "
            f"pedal_ratio. Either the measurement or the free play is wrong -- "
            f"there would be no fluid travel left to calibrate against.")

    return TravelParams(
        knockback_mm=p0.knockback_mm,
        pad_compression_mm=p0.pad_compression_mm,
        caliper_cc_per_bar=p0.caliper_cc_per_bar,
        hose_type=p0.hose_type, hardline_type=p0.hardline_type, fluid=p0.fluid,
        air_cc=p0.air_cc, free_play_mm=p0.free_play_mm,
        compliance_scale=k,
        calibrated=True,
        fitted_to=f"{fitted_to} ({target:.1f} mm at "
                  f"{line_pressure_bar:.0f} bar, compliance scale x{k:.3f})")


# Attach as a classmethod-style constructor without changing the dataclass shape.
TravelParams.from_measured_stroke = staticmethod(calibrate_travel_params)


# =========================================================================== #
#  5.  THE COUPLED VIEW  --  one call that keeps the three honest together
# =========================================================================== #
@dataclass
class PedalBoxStudy:
    """Packaging, bias and travel evaluated as ONE design, because they are one."""
    stack: StackUpResult
    plan: ShortenPlan | None
    bias: BalanceBarResult
    authority: BiasAuthority
    travel_front: TravelResult
    travel_rear: TravelResult
    findings: list[Finding] = field(default_factory=list)

    @property
    def verdict(self) -> str:
        """Worst of the three -- a pedal box is only as good as its worst axis."""
        order = {"FAIL": 3, "DOES NOT FIT": 3, "TIGHT": 2, "PASS": 1, "FITS": 1}
        worst = max(order.get(self.stack.verdict, 1),
                    order.get(self.travel_front.verdict, 1),
                    order.get(self.travel_rear.verdict, 1))
        if not self.authority.target_reachable and self.authority.target_bias is not None:
            worst = 3
        return {3: "FAIL", 2: "TIGHT", 1: "PASS"}[worst]

    def as_dict(self):
        return dict(verdict=self.verdict, stack=self.stack.as_dict(),
                    plan=(self.plan.as_dict() if self.plan else None),
                    bias=self.bias.as_dict(), authority=self.authority.as_dict(),
                    travel_front=self.travel_front.as_dict(),
                    travel_rear=self.travel_rear.as_dict(),
                    findings=[f.as_dict() for f in self.findings])

    def summary(self) -> str:
        parts = [f"PEDAL BOX STUDY -> {self.verdict}", "", self.stack.summary()]
        if self.plan is not None and self.plan.deficit_mm > 0:
            parts += ["", self.plan.summary()]
        parts += ["", self.authority.summary(), "",
                  "Front circuit: " + self.travel_front.summary(),
                  "Rear circuit:  " + self.travel_rear.summary()]
        return "\n".join(parts)


def study(*, available_mm: float,
          front: CircuitSpec, rear: CircuitSpec,
          pedal_force_N: float = 500.0,
          pedal_ratio: float = 5.0,
          pedal_lever_mm: float = 90.0,
          target_bias: float | None = 0.65,
          bar_length_mm: float = 60.0,
          bar_offset_mm: float = 0.0,
          travel_params: TravelParams | None = None,
          available_travel_mm: float = 60.0,
          mc_stroke_limit_mm: float = 25.4,
          stack_kwargs: dict | None = None,
          travel_kwargs: dict | None = None) -> PedalBoxStudy:
    """Run packaging, bias authority and travel as one coupled study.

    This exists because the three fight each other and evaluating them in
    separate sessions is how a pedal box gets designed twice. The travel budget is
    computed at the pressure the balance bar ACTUALLY produces at the requested
    pedal force, so a bias change shows up in the travel, and the shortening plan
    is priced against the same geometry.

    Cross-cutting findings -- the ones no single check can see -- are raised here.
    """
    sk = dict(stack_kwargs or {})
    tk = dict(travel_kwargs or {})

    stack_res = stack_up(available_mm=available_mm, pedal_ratio=pedal_ratio,
                         pedal_lever_mm=pedal_lever_mm, **sk)
    plan = (plan_shortening(stack_res, pedal_ratio=pedal_ratio,
                            pedal_lever_mm=pedal_lever_mm,
                            mc_bore_mm=front.mc_bore_mm)
            if stack_res.deficit_mm > 0 else None)

    bias_res = balance_bar_bias(pedal_force_N=pedal_force_N,
                                pedal_ratio=pedal_ratio, front=front, rear=rear,
                                bar_length_mm=bar_length_mm,
                                bar_offset_mm=bar_offset_mm)
    auth = bias_authority(pedal_force_N=pedal_force_N, pedal_ratio=pedal_ratio,
                          front=front, rear=rear, bar_length_mm=bar_length_mm,
                          target_bias=target_bias)

    tp = travel_params or TravelParams()
    tf = pedal_travel(circuit=front, line_pressure_bar=bias_res.pressure_front_bar,
                      pedal_ratio=pedal_ratio, params=tp,
                      available_travel_mm=available_travel_mm,
                      mc_stroke_limit_mm=mc_stroke_limit_mm, **tk)
    tr = pedal_travel(circuit=rear, line_pressure_bar=bias_res.pressure_rear_bar,
                      pedal_ratio=pedal_ratio, params=tp,
                      available_travel_mm=available_travel_mm,
                      mc_stroke_limit_mm=mc_stroke_limit_mm, **tk)

    findings: list[Finding] = []

    # The coupling that catches teams out: the two circuits share ONE pedal, so
    # the pedal travel is set by the LONGER of the two. A rear circuit that needs
    # more stroke than the front drags the whole pedal down with it.
    if abs(tf.mc_stroke_mm - tr.mc_stroke_mm) > 2.0:
        longer = "front" if tf.mc_stroke_mm > tr.mc_stroke_mm else "rear"
        findings.append(Finding(
            check="circuit_stroke_mismatch", severity=Severity.WARN,
            message=(f"The two circuits want different strokes "
                     f"({tf.mc_stroke_mm:.1f} mm front vs {tr.mc_stroke_mm:.1f} mm "
                     f"rear). They share one pedal, so the {longer} circuit sets "
                     f"the pedal travel and the balance bar tilts to take up the "
                     f"difference. A large mismatch eats the bar's articulation "
                     f"and shifts bias as the pedal is pressed -- which is felt as "
                     f"a bias that changes with pedal effort."),
            subsystems=["brakes"]))

    # Packaging fixes that would make travel worse are worth flagging BEFORE they
    # are chosen, not after.
    if plan is not None and plan.chosen:
        travel_costly = [o.name for o in plan.chosen
                         if "pedal travel" in str(o.side_effects).lower()
                         and "higher" in str(o.side_effects).lower()]
        if travel_costly and tf.verdict in ("TIGHT", "FAIL"):
            findings.append(Finding(
                check="packaging_vs_travel_conflict", severity=Severity.FAIL,
                message=("The shortening plan leans on changes that cost pedal "
                         "travel (" + "; ".join(travel_costly) + "), and travel is "
                         "already " + tf.verdict + ". Those two cannot both be paid "
                         "for. Take the length out of the fittings and the cylinder "
                         "body instead, or find it in the chassis."),
                subsystems=["brakes", "chassis"]))

    return PedalBoxStudy(stack=stack_res, plan=plan, bias=bias_res, authority=auth,
                         travel_front=tf, travel_rear=tr, findings=findings)


def provenance() -> dict:
    """What this module is and is not, for the report generator."""
    return {
        "model": ("pedal-box longitudinal stack-up, balance-bar lever statics, "
                  "and a fluid-volume pedal-travel budget"),
        "safe": ("the equations: lever moment balance, area ratios, "
                 "dV = V*dP/K compressibility, volume bookkeeping"),
        "provisional": ("the parameters: knockback, pad compression, caliper and "
                        "hose compliance, and catalogue cylinder body lengths"),
        "note": ("Use for TRADES -- which lever buys length, which bore reaches the "
                 "bias, what a ratio change costs in travel. Calibrate against one "
                 "bench measurement before quoting an absolute pedal travel, and "
                 "measure the real cylinders before ordering parts off a stack-up."),
    }
