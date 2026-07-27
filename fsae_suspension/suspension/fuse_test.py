# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/fuse_test.py — the TIME axis of fuse protection. fusebox.py
#  answers "which element in the chain loses the race" in current magnitude;
#  it has no clock. But a fuse does not protect a wire by being smaller — it
#  protects it by being FASTER, and whether it is faster is a question about
#  two curves crossing. This module draws both curves, finds where they cross,
#  and designs the destructive bench test that confirms the fuse curve is real.
# ============================================================================
r"""
Fuse time-current coordination, and the rig that measures it.

WHY THIS MODULE EXISTS
----------------------
`fusebox.py` models an overload path as a series race in one load multiplier:
each element has a failure current with a mean and a spread, and the audit
reports which one is most likely to go first. That is the right model for a
slow overload, and it is silent about the case that actually destroys harnesses.

A dead short is not a slow overload. Nothing in it is decided by current
magnitude alone, because both the fuse and the wire survive enormous currents
for short enough times. What decides it is ENERGY — the I²t each part absorbs
before it opens — and that means both parts have to be described as curves in
time, and the only question worth asking is which curve is lower at the current
the fault actually produces.

    grep -ri "i2t\|time.current\|blow\|melting" suspension/*.py    # nothing

So the toolkit could tell you your fuse was rated below your wire and still not
tell you the wire would cook first at 400 A.

The second half of this module exists because a fuse's published curve is a
manufacturer's typical, the fuses a team actually buys are frequently
counterfeit or a different series than the label claims, and the only way to
know is to blow some on a bench. Blowing them is easy. MEASURING them is where
a proof-of-concept rig quietly produces numbers that are entirely instrument
and no fuse.

WHAT IS PHYSICS AND WHAT IS DECLARED
------------------------------------
The wire withstand curve is the standard adiabatic short-circuit equation,
I²t = k²S², with k derived from the conductor and insulation temperatures
rather than looked up. The derivation reproduces the published IEC 60364-5-54
table values for copper PVC (115), copper XLPE (143) and aluminium PVC (76) to
within a rounding digit, so an automotive insulation the table does not list
gets a real derived k rather than a guessed one.

The fuse curve is a power law fitted through anchor points YOU read off the
datasheet. Nothing here invents a fuse characteristic. A fuse with no declared
anchors gets no curve, and coordination against it is refused rather than
estimated, because a made-up fuse curve compared against a real wire curve
produces a confident-looking protected range that means nothing.

THE HARD RULE
-------------
A measurement whose uncertainty is dominated by the instrument is not reported
as a measurement. Given the timer resolution and the detection latency of the
rig, this module computes the shortest blow time the rig can actually resolve,
and REFUSES the test points below it — because a rig that reports "47 ms" when
its own detection path takes 250 ms has measured the operator, and the number
it prints is indistinguishable in format from a real one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from .interfaces import Severity, Finding


# ===================================================================== #
#  1.  CONDUCTOR PHYSICS — the adiabatic withstand curve
# ===================================================================== #
#: Material constants for the IEC 60949 adiabatic equation.
#:   qc     volumetric heat capacity at 20 degC, J/(degC*mm^3)
#:   beta   reciprocal of the temperature coefficient of resistance at 0 degC
#:   rho20  electrical resistivity at 20 degC, ohm*mm
CONDUCTOR_MATERIALS: dict[str, dict] = {
    "copper":    {"qc": 3.45e-3, "beta": 234.5, "rho20": 1.7241e-5},
    "aluminium": {"qc": 2.50e-3, "beta": 228.0, "rho20": 2.8264e-5},
}

#: Insulation, as (continuous conductor temperature, permitted short-circuit
#: temperature). These two temperatures are the whole of what the insulation
#: contributes to the withstand curve, which is why deriving k beats looking it
#: up: an automotive insulation absent from the standard's table still has a
#: datasheet with these two numbers on it.
INSULATIONS: dict[str, tuple[float, float]] = {
    "PVC 70":        (70.0, 160.0),     # IEC table k = 115 (Cu)
    "PVC 90":        (90.0, 160.0),     # IEC table k = 100
    "XLPE / EPR 90": (90.0, 250.0),     # IEC table k = 143
    "Rubber 60":     (60.0, 200.0),     # IEC table k = 141
    "TXL / GXL 125": (125.0, 250.0),    # automotive XLPE, derived not tabled
    "SXL 125":       (125.0, 250.0),
    "PTFE 200":      (200.0, 350.0),    # derived
    "Silicone 180":  (180.0, 350.0),    # derived
}

#: AWG -> conductor area in mm^2. Mirrors harness.py's table for the gauges a
#: low-voltage harness actually uses; kept local so this module stays free of
#: the numpy import harness.py pulls in.
AWG_AREA_MM2: dict[int, float] = {
    0: 53.48, 2: 33.62, 4: 21.15, 6: 13.30, 8: 8.37, 10: 5.26, 12: 3.31,
    14: 2.08, 16: 1.31, 18: 0.823, 20: 0.518, 22: 0.326, 24: 0.205,
}


def awg_area_mm2(awg: int) -> float:
    """Conductor cross-section for an AWG, geometric between tabled gauges."""
    if awg in AWG_AREA_MM2:
        return AWG_AREA_MM2[awg]
    return AWG_AREA_MM2[10] * (2.0 ** ((10 - awg) / 3.0))


def adiabatic_k(material: str, t_initial_c: float, t_final_c: float) -> float:
    """The k factor of I²t = k²S², derived rather than looked up.

        k = sqrt( qc*(beta+20)/rho20 * ln((beta + tf) / (beta + ti)) )

    The derivation assumes all the fault energy stays in the conductor, which
    is true for the short times that matter and conservative for the long ones.

    Verified against the published IEC 60364-5-54 values: copper PVC 70/160
    gives 114.8 against a tabled 115, copper XLPE 90/250 gives 142.9 against
    143, aluminium PVC gives 76.1 against 76.
    """
    m = CONDUCTOR_MATERIALS.get(material)
    if m is None:
        raise KeyError(f"unknown conductor material '{material}'. "
                       f"Known: {sorted(CONDUCTOR_MATERIALS)}")
    if t_final_c <= t_initial_c:
        raise ValueError("short-circuit temperature must exceed the "
                         "continuous conductor temperature")
    return math.sqrt(
        m["qc"] * (m["beta"] + 20.0) / m["rho20"]
        * math.log((m["beta"] + t_final_c) / (m["beta"] + t_initial_c)))


@dataclass
class WireSpec:
    """The conductor the fuse is there to protect.

    `t_initial_c` defaults to the insulation's continuous rating, which is the
    conservative assumption: a wire already at its rated temperature when the
    fault arrives has the least headroom, and a harness in an FSAE engine bay
    in August is not at 20 degC.
    """
    label: str = "GLV feed"
    awg: Optional[int] = 16
    area_mm2: Optional[float] = None
    material: str = "copper"
    insulation: str = "TXL / GXL 125"
    t_initial_c: Optional[float] = None
    t_final_c: Optional[float] = None
    continuous_rating_a: Optional[float] = None   # ampacity in its bundle

    def area(self) -> float:
        if self.area_mm2 is not None:
            return float(self.area_mm2)
        if self.awg is None:
            raise ValueError(f"{self.label}: declare either awg or area_mm2")
        return awg_area_mm2(self.awg)

    def temperatures(self) -> tuple[float, float]:
        ti, tf = INSULATIONS.get(self.insulation, (70.0, 160.0))
        return (self.t_initial_c if self.t_initial_c is not None else ti,
                self.t_final_c if self.t_final_c is not None else tf)

    def k(self) -> float:
        ti, tf = self.temperatures()
        return adiabatic_k(self.material, ti, tf)

    def i2t_withstand(self) -> float:
        """A²·s the conductor absorbs before the insulation reaches its limit."""
        return (self.k() * self.area()) ** 2

    def withstand_time_s(self, current_a: float) -> float:
        """How long this wire survives `current_a`. Infinite at zero current."""
        if current_a <= 0:
            return math.inf
        return self.i2t_withstand() / (current_a ** 2)

    def damage_current_a(self, t_s: float) -> float:
        """The current that damages this wire in exactly `t_s`."""
        if t_s <= 0:
            return math.inf
        return math.sqrt(self.i2t_withstand() / t_s)


# ===================================================================== #
#  2.  FUSE MODEL — a power law through YOUR datasheet anchors
# ===================================================================== #
class FuseClass(str, Enum):
    FAST = "fast"
    SLOW_BLOW = "slow_blow"
    BLADE_ATO = "blade_ato"
    MIDI_MEGA = "midi_mega"
    SEMICONDUCTOR = "semiconductor"
    UNKNOWN = "unknown"


@dataclass
class CurveAnchor:
    """One point read off the datasheet's log-log time-current plot.

    `current_mult` is a multiple of the fuse's rated current, which is how the
    plots are drawn and how fuse families are compared. Reading two of these
    off the page is a two-minute job and it is the only input this module will
    accept in place of a real curve.
    """
    current_mult: float
    time_s: float
    note: str = ""


@dataclass
class FuseSpec:
    """A fuse, described only as well as its datasheet was actually read."""
    label: str = "main GLV fuse"
    rating_a: Optional[float] = None
    fuse_class: FuseClass = FuseClass.UNKNOWN
    anchors: list = field(default_factory=list)      # list[CurveAnchor]
    #: Manufacturer's clearing I²t, when the datasheet states one directly.
    declared_i2t: Optional[float] = None
    #: Blade fuses are rated in still air at 25 degC and must be derated for
    #: continuous duty in a hot, enclosed fusebox.
    continuous_derate: float = 0.75
    part_number: str = ""
    source: str = ""
    is_estimate: bool = False

    # ------------------------------------------------------------------ #
    def has_curve(self) -> bool:
        return self.rating_a is not None and len(self.anchors) >= 2

    def power_law(self) -> Optional[tuple[float, float]]:
        """Fit t = a * (I/I_rated)^(-b) through the declared anchors.

        Two anchors give an exact fit; more than two are least-squares fitted
        in log-log, which is the space the datasheet plot is drawn in and the
        space the scatter is symmetric in.
        """
        if not self.has_curve():
            return None
        xs = [math.log(a.current_mult) for a in self.anchors]
        ys = [math.log(a.time_s) for a in self.anchors]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx <= 0:
            return None
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        slope = sxy / sxx
        return math.exp(my - slope * mx), -slope      # (a, b)

    def anchor_range(self) -> Optional[tuple[float, float]]:
        """(lowest, highest) current multiple the datasheet anchors cover."""
        if not self.anchors:
            return None
        ms = [a.current_mult for a in self.anchors]
        return min(ms), max(ms)

    def blow_time_s(self, current_a: float) -> Optional[float]:
        """Expected time to clear at `current_a`, or None with no curve.

        Above the fastest declared anchor the model switches from the fitted
        power law to CONSTANT I²t, because that is what a fuse actually does:
        once the element melts faster than heat can leave it, clearing energy
        stops falling and the curve flattens to t proportional to I^-2.

        This matters and it is not a refinement. A blade fuse fitted between 2x
        and 10x commonly returns b near 4, and extrapolating I^-4 out to a
        dead-short current predicts clearing times orders of magnitude shorter
        than the fuse can deliver. That error is in the dangerous direction: it
        makes the protection look fast enough when it is not.
        """
        pl = self.power_law()
        if pl is None or not self.rating_a or current_a <= 0:
            return None
        a, b = pl
        rng = self.anchor_range()
        mult = current_a / self.rating_a
        if rng is not None and mult > rng[1]:
            # constant-I²t continuation, matched at the fastest anchor
            i_hi = rng[1] * self.rating_a
            t_hi = a * rng[1] ** (-b)
            return (i_hi ** 2 * t_hi) / (current_a ** 2)
        return a * mult ** (-b)

    def extrapolated(self, current_a: float) -> bool:
        """True when `current_a` sits outside the declared anchor range."""
        rng = self.anchor_range()
        if rng is None or not self.rating_a:
            return True
        return not (rng[0] <= current_a / self.rating_a <= rng[1])

    def clearing_i2t(self, current_a: float) -> Optional[float]:
        t = self.blow_time_s(current_a)
        return None if t is None else current_a ** 2 * t

    def continuous_limit_a(self) -> Optional[float]:
        if self.rating_a is None:
            return None
        return self.rating_a * self.continuous_derate


# ===================================================================== #
#  3.  COORDINATION — where the two curves cross
# ===================================================================== #
@dataclass
class CoordinationResult:
    fuse: FuseSpec
    wire: WireSpec
    protected: Optional[bool]           # None when it cannot be determined
    crossover_a: Optional[float]        # current above which the wire loses
    parallel_curves: bool               # b == 2: pure I²t comparison, no crossing
    margin_at: dict                     # current -> (t_fuse, t_wire, ratio)
    findings: list = field(default_factory=list)


#: Currents at which coordination is reported. Spans slow overload through
#: the dead-short region a low-impedance accumulator can actually deliver.
_PROBE_MULTS = (1.5, 2.0, 3.0, 5.0, 10.0, 20.0)


def coordinate(fuse: FuseSpec, wire: WireSpec,
               *, prospective_fault_a: Optional[float] = None
               ) -> CoordinationResult:
    """Does this fuse clear before this wire is damaged, and up to what current?

    The crossover is the analytic solution of t_fuse(I) = t_wire(I):

        a*(I/Ir)^-b = k²S²/I²   ->   I^(2-b) = k²S² / (a * Ir^b)

    When b == 2 the two curves are parallel in log-log — the fuse is behaving
    as a pure I²t device over the fitted range — and there is no crossover at
    all. That case is not a degenerate nuisance; it is the clean one, because
    coordination then reduces to a single comparison of two energies that holds
    at every current.
    """
    findings: list[Finding] = []
    who = ["electrics", "dataacq"]

    if not fuse.has_curve():
        findings.append(Finding(
            "fuse-curve-undeclared", Severity.MISSING,
            f"{fuse.label} has no declared time-current curve (needs a rating "
            f"and at least two anchor points off the datasheet plot). "
            f"Coordination is REFUSED rather than estimated: a made-up fuse "
            f"curve compared against a real wire curve produces a protected "
            f"range that looks authoritative and means nothing. Read two "
            f"points off the log-log plot — a slow one near 2x rating and a "
            f"fast one near 10x — and this will do the rest.",
            subsystems=who))
        return CoordinationResult(fuse, wire, None, None, False, {}, findings)

    a, b = fuse.power_law()
    ir = fuse.rating_a
    i2t_wire = wire.i2t_withstand()

    findings.append(Finding(
        "wire-withstand", Severity.INFO,
        f"{wire.label}: {wire.area():.2f} mm² {wire.material}, "
        f"{wire.insulation}, k = {wire.k():.0f} → withstand "
        f"{i2t_wire:,.0f} A²s. That is {wire.damage_current_a(1.0):.0f} A for "
        f"one second, or {wire.damage_current_a(0.01):.0f} A for ten "
        f"milliseconds.",
        subsystems=who,
        detail={"i2t": i2t_wire, "k": wire.k(), "area_mm2": wire.area()}))

    # ---- crossover ------------------------------------------------------- #
    parallel = abs(b - 2.0) < 1e-6
    crossover = None
    protected: Optional[bool] = None

    if parallel:
        # Both curves go as I^-2; compare energies once and it holds everywhere.
        i2t_fuse = a * ir ** 2
        protected = i2t_fuse < i2t_wire
        findings.append(Finding(
            "coordination-parallel", Severity.OK if protected else Severity.FAIL,
            f"The fitted fuse curve is a pure I²t law over this range, so it "
            f"never crosses the wire curve. Fuse lets through "
            f"{i2t_fuse:,.0f} A²s against the wire's {i2t_wire:,.0f} A²s — "
            f"{'protected at every current' if protected else 'the wire cooks first at EVERY current, not just high ones'}.",
            subsystems=who, detail={"i2t_fuse": i2t_fuse}))
    else:
        try:
            crossover = (i2t_wire / (a * ir ** b)) ** (1.0 / (2.0 - b))
        except (ValueError, ZeroDivisionError, OverflowError):
            crossover = None

        if crossover is None or crossover <= 0 or not math.isfinite(crossover):
            findings.append(Finding(
                "coordination-indeterminate", Severity.MISSING,
                "The fitted curves do not cross at any physical current, which "
                "usually means the anchor points are too close together to "
                "define a slope. Take a second reading further along the "
                "datasheet plot.",
                subsystems=who))
        elif b > 2.0:
            # fuse curve steeper: fuse wins at high current, loses at low
            protected = True
            findings.append(Finding(
                "coordination-crossover-low", Severity.WARN,
                f"The fuse clears first above {crossover:.0f} A, and the WIRE "
                f"reaches its limit first below it. Below the crossover both "
                f"times are long, so this is a thermal-overload question "
                f"rather than a short-circuit one — but it means a sustained "
                f"fault just under {crossover:.0f} A damages insulation while "
                f"the fuse holds. Check that nothing in this circuit can stall "
                f"at that current.",
                subsystems=who, detail={"crossover_a": crossover}))
        else:
            # fuse curve shallower: fuse wins at low current, loses at high
            protected = False
            findings.append(Finding(
                "coordination-fails-high", Severity.FAIL,
                f"Above {crossover:.0f} A the WIRE reaches its damage limit "
                f"before this fuse clears. That is the short-circuit region — "
                f"exactly where a fuse is supposed to earn its place — and a "
                f"dead short across an accumulator delivers far more than "
                f"{crossover:.0f} A. Either a faster fuse or a larger "
                f"conductor; a lower-rated fuse of the same family will not "
                f"fix it, because the curves are the wrong shape relative to "
                f"each other, not merely offset.",
                subsystems=who, detail={"crossover_a": crossover}))

    # ---- the short-circuit region: both curves are I^-2, compare energies -- #
    rng = fuse.anchor_range()
    if rng is not None and not parallel:
        i_hi = rng[1] * ir
        t_hi = fuse.blow_time_s(i_hi)
        if t_hi:
            i2t_fuse_sc = i_hi ** 2 * t_hi
            sc_ok = i2t_fuse_sc < i2t_wire
            findings.append(Finding(
                "short-circuit-energy",
                Severity.OK if sc_ok else Severity.FAIL,
                f"Above {i_hi:,.0f} A ({rng[1]:g}x rating) the fuse is "
                f"energy-limited, so both curves fall as I^-2 and never cross "
                f"again: it is one comparison of let-through "
                f"{i2t_fuse_sc:,.0f} A²s against the wire's "
                f"{i2t_wire:,.0f} A²s. "
                + ("The wire survives any fault current, however large."
                   if sc_ok else
                   "The wire is damaged at EVERY short-circuit current — "
                   "raising the fault current does not help the fuse catch up, "
                   "because both parts scale together."),
                subsystems=who,
                detail={"i2t_fuse_sc": i2t_fuse_sc, "i2t_wire": i2t_wire}))

    if crossover is not None and rng is not None and ir:
        if not (rng[0] <= crossover / ir <= rng[1]):
            findings.append(Finding(
                "crossover-extrapolated", Severity.WARN,
                f"The {crossover:,.0f} A crossover lies outside the "
                f"{rng[0]:g}x–{rng[1]:g}x range your anchor points cover, so "
                f"it is extrapolation rather than datasheet. Read one more "
                f"point near {crossover/ir:.1f}x rating to place it on real "
                f"data.",
                subsystems=who, detail={"crossover_a": crossover}))

    # ---- margin table ------------------------------------------------------ #
    margin: dict[float, tuple] = {}
    for m in _PROBE_MULTS:
        i = ir * m
        tf = fuse.blow_time_s(i)
        tw = wire.withstand_time_s(i)
        if tf is None:
            continue
        margin[i] = (tf, tw, (tw / tf) if tf > 0 else math.inf)

    # ---- the actual prospective fault -------------------------------------- #
    if prospective_fault_a:
        tf = fuse.blow_time_s(prospective_fault_a)
        tw = wire.withstand_time_s(prospective_fault_a)
        if tf is not None:
            ratio = tw / tf if tf > 0 else math.inf
            if ratio < 1.0:
                findings.append(Finding(
                    "fault-not-cleared", Severity.FAIL,
                    f"At the declared prospective fault current of "
                    f"{prospective_fault_a:,.0f} A the wire reaches its limit "
                    f"in {tw*1000:.1f} ms and the fuse clears in "
                    f"{tf*1000:.1f} ms. The harness is damaged before the "
                    f"protection operates.",
                    subsystems=who, detail={"ratio": ratio}))
            elif ratio < 2.0:
                findings.append(Finding(
                    "fault-margin-thin", Severity.WARN,
                    f"At {prospective_fault_a:,.0f} A the fuse clears in "
                    f"{tf*1000:.1f} ms against a wire limit of "
                    f"{tw*1000:.1f} ms — a factor of {ratio:.1f}. Fuse blow "
                    f"times scatter by tens of percent unit to unit, so this "
                    f"is not a margin, it is an overlap.",
                    subsystems=who, detail={"ratio": ratio}))
            else:
                findings.append(Finding(
                    "fault-cleared", Severity.OK,
                    f"At {prospective_fault_a:,.0f} A the fuse clears in "
                    f"{tf*1000:.1f} ms, {ratio:.0f}x inside the wire's "
                    f"{tw*1000:.1f} ms limit.",
                    subsystems=who, detail={"ratio": ratio}))

    return CoordinationResult(fuse, wire, protected, crossover, parallel,
                              margin, findings)


def nuisance_check(fuse: FuseSpec, *, continuous_load_a: Optional[float],
                   inrush_a: Optional[float] = None,
                   inrush_ms: Optional[float] = None,
                   ambient_c: float = 25.0) -> list[Finding]:
    """Will this fuse hold the load it is supposed to hold?

    The failure this catches is the opposite of the one everybody plans for:
    a fuse chosen small enough to protect everything, which then opens on a
    cold-start inrush in the rain on the last lap.
    """
    out: list[Finding] = []
    who = ["electrics"]

    if fuse.rating_a is None:
        out.append(Finding(
            "fuse-rating-undeclared", Severity.MISSING,
            f"{fuse.label} has no declared rating; nuisance-trip margin cannot "
            f"be checked.", subsystems=who))
        return out

    lim = fuse.continuous_limit_a()
    if continuous_load_a is None:
        out.append(Finding(
            "load-undeclared", Severity.MISSING,
            f"{fuse.label}: continuous load current not declared, so nothing "
            f"confirms this fuse can hold the circuit it protects.",
            subsystems=who))
    elif continuous_load_a > lim:
        out.append(Finding(
            "nuisance-trip-likely", Severity.FAIL,
            f"{fuse.label} carries {continuous_load_a:.1f} A continuously "
            f"against a derated limit of {lim:.1f} A "
            f"({fuse.continuous_derate*100:.0f}% of {fuse.rating_a:.0f} A). "
            f"Blade fuses are rated in still air at 25 degC; inside a hot "
            f"sealed fusebox this one will open on a normal lap.",
            subsystems=who, detail={"load_a": continuous_load_a}))
    elif continuous_load_a > 0.9 * lim:
        out.append(Finding(
            "nuisance-trip-marginal", Severity.WARN,
            f"{fuse.label} runs at {continuous_load_a/lim*100:.0f}% of its "
            f"derated limit. Fine on the bench, marginal at {ambient_c:.0f} "
            f"degC with the bodywork on.",
            subsystems=who))
    else:
        out.append(Finding(
            "continuous-ok", Severity.OK,
            f"{fuse.label}: {continuous_load_a:.1f} A against a derated "
            f"{lim:.1f} A limit.", subsystems=who))

    if inrush_a and inrush_ms:
        t = fuse.blow_time_s(inrush_a)
        if t is None:
            out.append(Finding(
                "inrush-uncheckable", Severity.MISSING,
                "Inrush declared but the fuse has no curve to check it "
                "against.", subsystems=who))
        elif t < inrush_ms / 1000.0:
            out.append(Finding(
                "inrush-opens-fuse", Severity.FAIL,
                f"A {inrush_a:.0f} A inrush lasting {inrush_ms:.0f} ms opens "
                f"this fuse, which clears that current in {t*1000:.0f} ms. "
                f"Either a slow-blow part or a soft-start.",
                subsystems=who, detail={"inrush_a": inrush_a}))
        else:
            out.append(Finding(
                "inrush-survived", Severity.OK,
                f"Inrush {inrush_a:.0f} A for {inrush_ms:.0f} ms is inside the "
                f"fuse's {t*1000:.0f} ms clearing time at that current.",
                subsystems=who))
    return out


# ===================================================================== #
#  4.  THE INSTRUMENT — what the rig can actually resolve
# ===================================================================== #
@dataclass
class Instrument:
    """The measuring chain of the bench rig, described honestly.

    The proof-of-concept default is the rig as first written: a human watching
    for the fuse to go and pressing a key, timed with `millis()`. Its detection
    latency is human reaction time, which is not a property of the fuse.
    """
    label: str = "proof-of-concept (operator keypress, millis)"
    timer_resolution_s: float = 1e-3            # millis()
    detection_latency_s: float = 0.25           # human reaction
    latency_jitter_s: float = 0.08              # spread of that reaction
    adc_sample_rate_hz: Optional[float] = None  # None = no automatic detection
    serial_in_timing_path: bool = True          # blocking prints between marks
    serial_baud: int = 9600
    serial_chars_in_path: int = 30

    # ------------------------------------------------------------------ #
    def serial_blocking_s(self) -> float:
        """Time a blocking print steals from the measurement.

        At 8N1 each character is 10 bits. A print sitting between the start
        mark and the stop mark does not slow the fuse down; it delays the
        clock, and it does so by an amount nobody subtracts afterwards.
        """
        if not self.serial_in_timing_path or self.serial_baud <= 0:
            return 0.0
        return self.serial_chars_in_path * 10.0 / self.serial_baud

    def sampling_latency_s(self) -> float:
        """Worst-case delay between the event and the sample that sees it."""
        if not self.adc_sample_rate_hz:
            return 0.0
        return 1.0 / self.adc_sample_rate_hz

    def total_latency_s(self) -> float:
        return (self.detection_latency_s + self.serial_blocking_s()
                + self.sampling_latency_s())

    def uncertainty_s(self) -> float:
        """Combined standard uncertainty on a single blow-time measurement.

        Quantisation contributes resolution/sqrt(12) as a uniform distribution;
        the latency jitter adds in quadrature. A CONSTANT latency is a bias
        rather than an uncertainty and could in principle be subtracted — but
        only if someone measured it, and nobody ever does, so it is reported
        separately as a bias rather than quietly assumed away.
        """
        quant = self.timer_resolution_s / math.sqrt(12.0)
        return math.sqrt(quant ** 2 + self.latency_jitter_s ** 2)

    def resolvable_time_s(self, max_rel_error: float = 0.10) -> float:
        """Shortest blow time this rig can measure to `max_rel_error`."""
        u = self.uncertainty_s()
        return u / max_rel_error if max_rel_error > 0 else math.inf

    def required_sample_rate_hz(self, target_time_s: float,
                                max_rel_error: float = 0.05) -> float:
        """Sample rate needed to catch an event of `target_time_s`."""
        if target_time_s <= 0:
            return math.inf
        return 1.0 / (max_rel_error * target_time_s)


#: A rig rebuilt around a shunt, a comparator-style threshold in software and
#: microsecond timing, with every print moved out of the timing path.
def instrumented_rig(adc_sample_rate_hz: float = 9600.0) -> Instrument:
    return Instrument(
        label="instrumented (shunt + ADC threshold, micros)",
        timer_resolution_s=1e-6,
        detection_latency_s=1.0 / adc_sample_rate_hz,
        latency_jitter_s=0.5 / adc_sample_rate_hz,
        adc_sample_rate_hz=adc_sample_rate_hz,
        serial_in_timing_path=False)


def instrument_findings(inst: Instrument,
                        expected_times_s: list[float]) -> list[Finding]:
    """Can this rig measure these events, or is it measuring itself?"""
    out: list[Finding] = []
    who = ["electrics", "dataacq"]
    u = inst.uncertainty_s()
    bias = inst.total_latency_s()
    floor = inst.resolvable_time_s(0.10)

    out.append(Finding(
        "instrument-budget", Severity.INFO,
        f"{inst.label}: timer resolution {inst.timer_resolution_s*1e6:.0f} us, "
        f"detection latency {bias*1000:.1f} ms, combined uncertainty "
        f"±{u*1000:.1f} ms. Shortest event measurable to 10% is "
        f"{floor*1000:.0f} ms.",
        subsystems=who,
        detail={"uncertainty_s": u, "bias_s": bias, "floor_s": floor}))

    if inst.serial_in_timing_path and inst.serial_blocking_s() > 0:
        out.append(Finding(
            "serial-in-timing-path", Severity.WARN,
            f"Blocking serial output sits between the start and stop marks: "
            f"{inst.serial_chars_in_path} characters at {inst.serial_baud} "
            f"baud is {inst.serial_blocking_s()*1000:.0f} ms added to every "
            f"reading, in one direction only. Buffer the results and print "
            f"after the test, not during it.",
            subsystems=who))

    if inst.adc_sample_rate_hz is None:
        out.append(Finding(
            "manual-detection", Severity.FAIL,
            f"Blow detection is manual, so the {bias*1000:.0f} ms in this "
            f"measurement is the operator's reaction time and not a property "
            f"of the fuse. Every reading is biased long by roughly that much, "
            f"and the bias is larger than most of the blow times worth "
            f"measuring. Detect the current collapse on a shunt instead.",
            subsystems=who))

    for t in sorted(expected_times_s):
        rel = u / t if t > 0 else math.inf
        if rel > 0.5:
            sev, verdict = Severity.FAIL, "is noise, not a measurement"
        elif rel > 0.2:
            sev, verdict = Severity.FAIL, "is dominated by the instrument"
        elif rel > 0.1:
            sev, verdict = Severity.WARN, "is marginal"
        else:
            continue
        need = inst.required_sample_rate_hz(t, 0.05)
        out.append(Finding(
            "test-point-unmeasurable", sev,
            f"An expected blow time of {t*1000:.1f} ms {verdict}: ±{u*1000:.1f} "
            f"ms is {rel*100:.0f}% of it. Resolving this point to 5% needs "
            f"detection at {need:,.0f} Hz or faster.",
            subsystems=who, detail={"time_s": t, "rel": rel, "need_hz": need}))
    return out


# ===================================================================== #
#  5.  THE TEST PLAN — destructive, so it has to be designed
# ===================================================================== #
@dataclass
class TestPoint:
    current_mult: float
    current_a: float
    expected_time_s: Optional[float]
    samples: int
    measurable: bool
    rel_uncertainty: Optional[float]


@dataclass
class TestPlan:
    fuse: FuseSpec
    instrument: Instrument
    points: list
    fuses_consumed: int
    estimated_cost: float
    scatter: Optional[ScatterEstimate] = None
    findings: list = field(default_factory=list)

    def measurable_points(self) -> list:
        return [p for p in self.points if p.measurable]

    def sized_on_measurement(self) -> bool:
        """True when the fuse count came from your bench, not from the prior."""
        return bool(self.scatter and self.scatter.measured)


#: Prior log-normal spread of blow time within one fuse family, as a standard
#: deviation of ln(t). Manufacturers publish +/-20% bands and real parts scatter
#: wider. This is a PRIOR, used only until the team has measured its own — see
#: `pooled_log_scatter` and `refine_from_pilot`, which replace it with a number
#: that came off your bench. Nothing in this module treats it as known.
PRIOR_LOG_SCATTER = 0.30

#: Backwards-compatible alias. Prefer PRIOR_LOG_SCATTER: the name matters,
#: because "default" reads as a property of fuses and "prior" reads as a
#: placeholder waiting to be displaced, which is what it is.
DEFAULT_LOG_SCATTER = PRIOR_LOG_SCATTER


def _chi2_lower_quantile(dof: int, p: float = 0.05) -> float:
    """Lower-tail chi-square quantile by the Wilson-Hilferty approximation.

    Used to put an upper confidence bound on an estimated standard deviation
    without dragging scipy into a module that is otherwise pure stdlib. The
    approximation is within a fraction of a percent by dof = 10 and errs
    slightly LOW at small dof, which inflates the bound — the conservative
    direction for sizing a test.
    """
    if dof <= 0:
        return float("nan")
    z = {0.01: -2.326, 0.05: -1.645, 0.10: -1.282}.get(p, -1.645)
    t = 1.0 - 2.0 / (9.0 * dof) + z * math.sqrt(2.0 / (9.0 * dof))
    return dof * max(t, 1e-9) ** 3


@dataclass
class ScatterEstimate:
    """The unit-to-unit spread of blow time, and how well it is known.

    `measured` is the whole point of this object. A scatter that came off a
    bench and a scatter that came out of this module's prior produce the same
    kind of number and license completely different confidence, so they are not
    allowed to look alike.
    """
    sigma_ln: float
    dof: int = 0                    # degrees of freedom behind the estimate
    measured: bool = False
    n_levels: int = 0
    source: str = "prior"

    # ------------------------------------------------------------------ #
    def rel_uncertainty(self) -> float:
        """Relative uncertainty on sigma itself: approximately 1/sqrt(2*dof).

        This is the number that makes a small pilot honest. Five fuses give
        dof = 4 and sigma known to about 35% — and since the sample count goes
        as sigma squared, the count implied by that pilot is uncertain by
        roughly 70%.
        """
        if not self.measured or self.dof <= 0:
            return float("inf")
        return 1.0 / math.sqrt(2.0 * self.dof)

    def upper_bound(self, confidence: float = 0.95) -> float:
        """Upper confidence bound on sigma_ln.

        Sizing a destructive test on a point estimate of sigma under-sizes it
        half the time, which is the half where you finish the session without
        the precision you went in for and the fuses are already gone. The bound
        is the cost of not doing that twice.
        """
        if not self.measured or self.dof <= 0:
            return self.sigma_ln
        chi2 = _chi2_lower_quantile(self.dof, 1.0 - confidence)
        if not math.isfinite(chi2) or chi2 <= 0:
            return self.sigma_ln
        return self.sigma_ln * math.sqrt(self.dof / chi2)

    def as_dict(self) -> dict:
        return {"sigma_ln": self.sigma_ln, "dof": self.dof,
                "measured": self.measured, "n_levels": self.n_levels,
                "source": self.source}


def prior_scatter(sigma_ln: float = PRIOR_LOG_SCATTER) -> ScatterEstimate:
    """The placeholder, explicitly labelled as one."""
    return ScatterEstimate(sigma_ln=sigma_ln, dof=0, measured=False,
                           source="module prior (not your fuses)")


def pooled_log_scatter(measurements: list) -> Optional[ScatterEstimate]:
    """Pool the observed spread of ln(t) across current levels.

    Pooling across levels is the right move because the spread of a fuse
    family is a property of the parts rather than of the current you tested
    at, so every level contributes degrees of freedom to one estimate:

        s_p² = sum (n_i - 1) s_i² / sum (n_i - 1)

    Returns None when no level has at least two samples — a spread needs two
    parts to be a spread, and reporting one from a single reading would be the
    same failure this module rejects everywhere else.
    """
    num = 0.0
    dof = 0
    levels = 0
    for m in measurements:
        ts = [t for t in m.times_s if t > 0]
        if len(ts) < 2:
            continue
        ls = [math.log(t) for t in ts]
        mean = sum(ls) / len(ls)
        var = sum((l - mean) ** 2 for l in ls) / (len(ls) - 1)
        num += (len(ts) - 1) * var
        dof += len(ts) - 1
        levels += 1
    if dof <= 0:
        return None
    return ScatterEstimate(sigma_ln=math.sqrt(num / dof), dof=dof,
                           measured=True, n_levels=levels,
                           source=f"measured, {levels} level(s), {dof} dof")


def _as_scatter(s) -> ScatterEstimate:
    """Accept a ScatterEstimate, a bare float, or None."""
    if s is None:
        return prior_scatter()
    if isinstance(s, ScatterEstimate):
        return s
    return ScatterEstimate(sigma_ln=float(s), dof=0, measured=False,
                           source="caller-supplied scalar (unverified)")


def samples_needed(rel_precision: float = 0.20,
                   log_scatter=None,
                   confidence_z: float = 1.96,
                   *, use_upper_bound: bool = True,
                   scatter_confidence: float = 0.95) -> int:
    """Fuses per current level to pin the median blow time to +/-rel_precision.

    Blow times are log-normally distributed, so the interval is multiplicative
    and the count is n = (z * sigma_ln / ln(1 + p))².

    When `log_scatter` is a MEASURED `ScatterEstimate`, the sizing uses the
    upper confidence bound on sigma rather than the point estimate, because
    sigma enters squared: a pilot that happened to come out tight would
    otherwise licence a test too small to deliver the precision it promised.
    Pass `use_upper_bound=False` to size on the point estimate anyway.
    """
    if rel_precision <= 0:
        raise ValueError("rel_precision must be > 0")
    est = _as_scatter(log_scatter)
    sigma = (est.upper_bound(scatter_confidence)
             if (use_upper_bound and est.measured) else est.sigma_ln)
    return max(3, math.ceil((confidence_z * sigma
                             / math.log1p(rel_precision)) ** 2))


#: Degrees of freedom below which an upper-bound sizing is dominated by not
#: knowing sigma rather than by sigma itself. At dof = 2 the 95% bound on a
#: standard deviation is roughly five times the estimate, and since the count
#: goes as sigma squared, sizing rigorously on a three-fuse pilot demands
#: hundreds per level — arithmetically correct and practically useless.
MIN_USEFUL_DOF = 8


def recommended_pilot_n(target_rel_uncertainty: float = 0.25) -> int:
    """Pilot size so sigma itself is known to `target_rel_uncertainty`.

    Inverts sd(ln s) ~ 1/sqrt(2*dof). Wanting sigma to 25% needs dof = 8, so
    nine fuses at one current — which is the honest price of the two-stage
    procedure and considerably cheaper than the session it saves.
    """
    if target_rel_uncertainty <= 0:
        raise ValueError("target_rel_uncertainty must be > 0")
    return max(3, math.ceil(1.0 / (2.0 * target_rel_uncertainty ** 2)) + 1)


def refine_from_pilot(measurements: list, *, rel_precision: float = 0.20,
                      confidence_z: float = 1.96,
                      pilot_n: Optional[int] = None) -> dict:
    """Two-stage sizing: what the pilot says the real test needs.

    The standard answer to "sigma is assumed" is to stop assuming it. Run a
    small pilot at one current, measure the spread, and size the real test from
    that. This reports the count the pilot implies, how many more fuses that
    means, and how much the count moved against the prior — because a pilot
    that doubles the requirement has just saved a session that would have
    produced a confidently wrong median.

    A pilot can also be too small to be worth pooling rigorously, and this says
    so instead of returning the arithmetically correct but unusable number that
    a two-fuse spread implies.
    """
    est = pooled_log_scatter(measurements)
    if est is None:
        return {
            "scatter": None,
            "n_required": None,
            "finding": Finding(
                "pilot-too-thin", Severity.MISSING,
                f"The pilot has no current level with two or more samples, so "
                f"it measures no spread at all and cannot size anything. A "
                f"pilot of one fuse per level is a set of anecdotes; put at "
                f"least {recommended_pilot_n()} on one level.",
                subsystems=["electrics", "dataacq"]),
        }

    n_prior = samples_needed(rel_precision, prior_scatter(), confidence_z)
    n_point = samples_needed(rel_precision, est, confidence_z,
                             use_upper_bound=False)
    n_req = samples_needed(rel_precision, est, confidence_z)
    have = pilot_n if pilot_n is not None else (est.dof + est.n_levels)
    more = max(0, n_req - have)

    # ---- is the pilot big enough to bound sigma usefully? ---------------- #
    if est.dof < MIN_USEFUL_DOF:
        want = recommended_pilot_n()
        return {
            "scatter": est,
            "n_required": n_point,          # fall back to the point estimate
            "n_point_estimate": n_point,
            "n_prior": n_prior,
            "n_upper_bound": n_req,
            "additional": max(0, n_point - have),
            "pilot_adequate": False,
            "finding": Finding(
                "pilot-underpowered", Severity.WARN,
                f"The pilot gives sigma_ln = {est.sigma_ln:.3f} but only "
                f"{est.dof} degrees of freedom, so sigma is itself uncertain "
                f"by ±{est.rel_uncertainty()*100:.0f}%. Sizing rigorously on "
                f"the upper bound would demand {n_req} per level, which is not "
                f"a real recommendation — it is the cost of not knowing sigma, "
                f"not the cost of the scatter. Use the point estimate "
                f"({n_point} per level) with that caveat, or spend "
                f"{max(0, want - have)} more fuses to reach a "
                f"{want}-sample pilot and get a bound worth acting on.",
                subsystems=["electrics", "dataacq"],
                detail={"sigma_ln": est.sigma_ln, "dof": est.dof,
                        "n_point": n_point, "n_upper": n_req,
                        "recommended_pilot": want}),
        }

    ratio = est.sigma_ln / PRIOR_LOG_SCATTER
    if ratio > 1.3:
        sev = Severity.WARN
        note = (f"Your parts scatter {ratio:.1f}x WIDER than the prior, so the "
                f"test needs {n_req} per level, not {n_prior}. Sizing on the "
                f"prior would have finished the session believing a median it "
                f"had not actually pinned down.")
    elif ratio < 0.7:
        sev = Severity.OK
        note = (f"Your parts scatter {1/ratio:.1f}x TIGHTER than the prior: "
                f"{n_req} per level instead of {n_prior}, which is "
                f"{n_prior - n_req} fewer fuses per level for the same "
                f"precision.")
    else:
        sev = Severity.OK
        note = (f"Measured scatter is close to the prior; {n_req} per level "
                f"against {n_prior}.")

    return {
        "scatter": est,
        "n_required": n_req,
        "n_point_estimate": n_point,
        "n_prior": n_prior,
        "n_upper_bound": n_req,
        "additional": more,
        "pilot_adequate": True,
        "finding": Finding(
            "scatter-measured", sev,
            f"Pilot gives sigma_ln = {est.sigma_ln:.3f} ({est.dof} dof, "
            f"itself uncertain by ±{est.rel_uncertainty()*100:.0f}%). Sizing "
            f"on the {est.upper_bound():.3f} upper bound rather than the point "
            f"estimate costs {n_req - n_point} extra fuses per level and is "
            f"what stops the test being under-sized half the time. {note}",
            subsystems=["electrics", "dataacq"],
            detail={"sigma_ln": est.sigma_ln, "dof": est.dof,
                    "n_required": n_req, "additional": more}),
    }
    if ratio > 1.3:
        sev = Severity.WARN
        note = (f"Your parts scatter {ratio:.1f}x WIDER than the prior, so the "
                f"test needs {n_req} per level, not {n_prior}. Sizing on the "
                f"prior would have finished the session believing a median it "
                f"had not actually pinned down.")
    elif ratio < 0.7:
        sev = Severity.OK
        note = (f"Your parts scatter {1/ratio:.1f}x TIGHTER than the prior: "
                f"{n_req} per level instead of {n_prior}, which is "
                f"{n_prior - n_req} fewer fuses per level for the same "
                f"precision.")
    else:
        sev = Severity.OK
        note = (f"Measured scatter is close to the prior; {n_req} per level "
                f"against {n_prior}.")

    return {
        "scatter": est,
        "n_required": n_req,
        "n_point_estimate": n_point,
        "n_prior": n_prior,
        "additional": more,
        "finding": Finding(
            "scatter-measured", sev,
            f"Pilot gives sigma_ln = {est.sigma_ln:.3f} ({est.dof} dof, "
            f"itself uncertain by ±{est.rel_uncertainty()*100:.0f}%). Sizing "
            f"on the {est.upper_bound():.3f} upper bound rather than the point "
            f"estimate costs {n_req - n_point} extra fuses per level and is "
            f"what stops the test being under-sized half the time. {note}",
            subsystems=["electrics", "dataacq"],
            detail={"sigma_ln": est.sigma_ln, "dof": est.dof,
                    "n_required": n_req, "additional": more}),
    }


def build_test_plan(fuse: FuseSpec, instrument: Instrument, *,
                    multipliers: tuple = (2.0, 3.0, 5.0, 10.0),
                    rel_precision: float = 0.20,
                    unit_cost: float = 1.50,
                    log_scatter=None) -> TestPlan:
    """Design the destructive test, and say which points the rig cannot take.

    Every point in this plan destroys `samples` fuses. That is the reason the
    plan exists as an object with a cost on it rather than as an intention:
    a team that walks to the bench with four fuses and no plan measures four
    unrelated numbers and concludes nothing.

    `log_scatter` accepts a measured `ScatterEstimate` from `pooled_log_scatter`
    or `refine_from_pilot`. Left as None it falls back to the module prior and
    SAYS SO in a finding, because a fuse count derived from a placeholder and
    one derived from your own pilot are the same kind of number carrying
    entirely different weight.
    """
    findings: list[Finding] = []
    who = ["electrics", "dataacq"]
    est = _as_scatter(log_scatter)

    if not fuse.has_curve():
        findings.append(Finding(
            "plan-without-curve", Severity.MISSING,
            "No declared fuse curve, so the expected blow times are unknown "
            "and the plan cannot say which points the rig can resolve. Testing "
            "blind is still possible — but start at 2x rating and work up, "
            "because a point that clears faster than the rig can see returns a "
            "number that looks fine.",
            subsystems=who))

    n = samples_needed(rel_precision, est)
    u = instrument.uncertainty_s()
    points: list[TestPoint] = []

    for m in multipliers:
        i = (fuse.rating_a or 0.0) * m
        t = fuse.blow_time_s(i) if fuse.has_curve() else None
        rel = (u / t) if (t and t > 0) else None
        points.append(TestPoint(
            current_mult=m, current_a=i, expected_time_s=t, samples=n,
            measurable=(rel is None or rel <= 0.20),
            rel_uncertainty=rel))

    consumed = sum(p.samples for p in points)
    cost = consumed * unit_cost

    # ---- where the fuse count actually came from ------------------------- #
    if est.measured:
        findings.append(Finding(
            "scatter-source-measured", Severity.OK,
            f"Sized on YOUR measured spread: sigma_ln = {est.sigma_ln:.3f} "
            f"from {est.dof} degrees of freedom across {est.n_levels} level(s), "
            f"used at its {est.upper_bound():.3f} upper bound. The count below "
            f"is a consequence of your parts rather than of an assumption.",
            subsystems=who, detail=est.as_dict()))
    else:
        n_hi = samples_needed(rel_precision,
                              ScatterEstimate(est.sigma_ln * 1.5, source="x1.5"))
        findings.append(Finding(
            "scatter-source-assumed", Severity.MISSING,
            f"Sized on an ASSUMED spread (sigma_ln = {est.sigma_ln:.2f}, "
            f"{est.source}), which nothing here has verified for the parts in "
            f"your drawer. The count is quadratic in that assumption: if your "
            f"fuses scatter half again as widely, this becomes {n_hi} per level "
            f"rather than {n}. Run five at one current, pass them to "
            f"refine_from_pilot(), and this stops being a guess.",
            subsystems=who, detail=est.as_dict()))

    findings.append(Finding(
        "plan-size", Severity.INFO,
        f"{len(points)} current levels x {n} samples = {consumed} fuses "
        f"(about {cost:,.0f} at {unit_cost:.2f} each) to pin the median blow "
        f"time to ±{rel_precision*100:.0f}% at each level. {n} per level "
        f"comes from the log-normal scatter of blow times, not from a round "
        f"number — one fuse per level is an anecdote.",
        subsystems=who, detail={"fuses": consumed, "cost": cost, "n": n}))

    bad = [p for p in points if not p.measurable]
    if bad:
        findings.append(Finding(
            "plan-points-unmeasurable", Severity.FAIL,
            f"{len(bad)} of {len(points)} planned points are below what this "
            f"rig can resolve ("
            + ", ".join(f"{p.current_mult:g}x → {p.expected_time_s*1000:.1f} ms"
                        for p in bad if p.expected_time_s) +
            f"). Those are the short-circuit points — the ones that decide "
            f"whether the harness survives — so a rig that can only take the "
            f"slow points measures the region that matters least.",
            subsystems=who))

    if fuse.has_curve():
        findings.extend(instrument_findings(
            instrument, [p.expected_time_s for p in points
                         if p.expected_time_s]))

    return TestPlan(fuse, instrument, points, consumed, cost, est, findings)


# ===================================================================== #
#  6.  INGEST — what the rig actually measured
# ===================================================================== #
@dataclass
class Measurement:
    current_a: float
    times_s: list                 # every sample at this current


@dataclass
class FitResult:
    a: Optional[float]
    b: Optional[float]
    per_point: dict               # current -> {median, geo_sd, n, declared, ratio}
    in_family: Optional[bool]
    #: The unit-to-unit spread these very measurements imply. Feed it straight
    #: back into build_test_plan(log_scatter=...) so the next session is sized
    #: on your parts rather than on this module's prior.
    scatter: Optional[ScatterEstimate] = None
    findings: list = field(default_factory=list)


def _log_stats(xs: list[float]) -> tuple[float, float]:
    """Geometric mean and geometric standard deviation."""
    ls = [math.log(x) for x in xs if x > 0]
    if not ls:
        return float("nan"), float("nan")
    m = sum(ls) / len(ls)
    if len(ls) < 2:
        return math.exp(m), float("nan")
    var = sum((l - m) ** 2 for l in ls) / (len(ls) - 1)
    return math.exp(m), math.exp(math.sqrt(var))


def fit_measurements(measurements: list[Measurement], fuse: FuseSpec,
                     instrument: Instrument,
                     *, tolerance: float = 0.5) -> FitResult:
    """Fit the measured curve and compare it to the declared one.

    REFUSES to fit points the instrument could not resolve. A rig that reports
    milliseconds with a quarter-second detection latency has measured its
    operator, and folding those readings into a regression launders them into
    something that looks like a curve.
    """
    findings: list[Finding] = []
    who = ["electrics", "dataacq"]
    u = instrument.uncertainty_s()

    usable: list[Measurement] = []
    for m in measurements:
        med, _ = _log_stats(m.times_s)
        if med != med or med <= 0:
            continue
        rel = u / med
        if rel > 0.20:
            findings.append(Finding(
                "measurement-rejected", Severity.FAIL,
                f"{m.current_a:.0f} A: median {med*1000:.1f} ms against an "
                f"instrument uncertainty of ±{u*1000:.1f} ms "
                f"({rel*100:.0f}%). Rejected from the fit — this reading is "
                f"the rig, not the fuse.",
                subsystems=who, detail={"current_a": m.current_a}))
            continue
        if len(m.times_s) < 3:
            findings.append(Finding(
                "measurement-thin", Severity.WARN,
                f"{m.current_a:.0f} A has only {len(m.times_s)} sample(s). "
                f"Blow times scatter by tens of percent; a median of two is "
                f"not a median.",
                subsystems=who))
        usable.append(m)

    # The spread these measurements imply — the number that displaces the
    # module prior when sizing the next session.
    scatter = pooled_log_scatter(usable)
    if scatter is not None:
        findings.append(Finding(
            "scatter-observed", Severity.INFO,
            f"Observed spread sigma_ln = {scatter.sigma_ln:.3f} "
            f"({scatter.dof} dof across {scatter.n_levels} level(s)), itself "
            f"uncertain by ±{scatter.rel_uncertainty()*100:.0f}%. Pass this "
            f"into build_test_plan(log_scatter=...) and the next test is sized "
            f"on these parts instead of on an assumption.",
            subsystems=who, detail=scatter.as_dict()))

    per: dict[float, dict] = {}
    for m in usable:
        med, gsd = _log_stats(m.times_s)
        declared = fuse.blow_time_s(m.current_a)
        per[m.current_a] = {
            "median": med, "geo_sd": gsd, "n": len(m.times_s),
            "declared": declared,
            "ratio": (med / declared) if declared else None,
        }

    if len(usable) < 2 or not fuse.rating_a:
        findings.append(Finding(
            "fit-refused", Severity.MISSING,
            f"Only {len(usable)} usable current level(s) — a power law needs "
            f"at least two to have a slope. No fitted curve is produced.",
            subsystems=who))
        return FitResult(None, None, per, None,
                         scatter=scatter, findings=findings)

    xs = [math.log(m.current_a / fuse.rating_a) for m in usable]
    ys = [math.log(_log_stats(m.times_s)[0]) for m in usable]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        findings.append(Finding(
            "fit-refused", Severity.MISSING,
            "All usable measurements are at one current; no slope.",
            subsystems=who))
        return FitResult(None, None, per, None,
                         scatter=scatter, findings=findings)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    a_fit, b_fit = math.exp(my - slope * mx), -slope

    findings.append(Finding(
        "fit-result", Severity.INFO,
        f"Measured curve: t = {a_fit:.3g} x (I/I_rated)^-{b_fit:.2f} over "
        f"{len(usable)} current levels.",
        subsystems=who, detail={"a": a_fit, "b": b_fit}))

    ratios = [d["ratio"] for d in per.values() if d.get("ratio")]
    in_family = None
    if ratios and fuse.has_curve():
        worst = max(max(r, 1.0 / r) for r in ratios)
        in_family = worst <= (1.0 + tolerance)
        if in_family:
            findings.append(Finding(
                "in-family", Severity.OK,
                f"Measured blow times sit within {(worst-1)*100:.0f}% of the "
                f"declared curve at every level — these parts behave like the "
                f"datasheet says.",
                subsystems=who, detail={"worst_ratio": worst}))
        else:
            fast = [c for c, d in per.items()
                    if d.get("ratio") and d["ratio"] < 1.0 / (1 + tolerance)]
            slow = [c for c, d in per.items()
                    if d.get("ratio") and d["ratio"] > 1 + tolerance]
            direction = ("SLOWER" if slow else "faster")
            findings.append(Finding(
                "out-of-family", Severity.FAIL,
                f"Measured blow times differ from the declared curve by up to "
                f"{(worst-1)*100:.0f}%, {direction} than the datasheet. "
                + ("A fuse slower than its published curve invalidates every "
                   "coordination result computed from that curve — the wire "
                   "protection you signed off does not exist. "
                   if slow else
                   "Faster than published is safer for the wire and worse for "
                   "nuisance trips; re-check inrush. ")
                + "Counterfeit or mislabelled blade fuses are common enough "
                  "that this is the expected explanation, not an exotic one.",
                subsystems=who,
                detail={"worst_ratio": worst, "slow": slow, "fast": fast}))

    for c, d in per.items():
        if d["geo_sd"] == d["geo_sd"] and d["geo_sd"] > 1.5:
            findings.append(Finding(
                "high-scatter", Severity.WARN,
                f"{c:.0f} A: geometric SD {d['geo_sd']:.2f} — the spread "
                f"between nominally identical fuses is wider than the margin "
                f"most coordination decisions assume.",
                subsystems=who, detail={"current_a": c}))

    return FitResult(a_fit, b_fit, per, in_family,
                     scatter=scatter, findings=findings)


# ===================================================================== #
#  7.  THE RIG FIRMWARE — generated from the plan
# ===================================================================== #
def emit_arduino_sketch(plan: TestPlan, *, shunt_mohm: float = 1.0,
                        adc_ref_v: float = 5.0, adc_bits: int = 10,
                        gain: float = 50.0,
                        trip_fraction: float = 0.5) -> str:
    """Generate the rig firmware, with the timing faults designed out.

    Four changes against a keypress-and-millis proof of concept, each of which
    is the difference between measuring a fuse and measuring the bench:

      1. The stop mark comes from the current collapsing through a threshold on
         a shunt, not from an operator noticing. Reaction time leaves the
         measurement entirely.
      2. `micros()` instead of `millis()` — a 1 ms tick is 20% of a 5 ms event.
      3. No serial output between the marks. Results go to a buffer and print
         after the run, because a blocking print at 9600 baud is tens of
         milliseconds added in one direction.
      4. Repeats are counted and the statistics are computed on the rig, so
         nobody transcribes single readings into a spreadsheet and treats the
         first one as the answer.

    The threshold is derived from the shunt and the amplifier gain, so the
    number in the sketch corresponds to a stated current rather than to an ADC
    count somebody tuned until it looked right.
    """
    counts = float((1 << adc_bits) - 1)
    fuse = plan.fuse
    rating = fuse.rating_a or 0.0
    pts = plan.points or []
    i_max = max((p.current_a for p in pts), default=rating * 10.0)

    def adc_for(current_a: float) -> int:
        v = current_a * (shunt_mohm / 1000.0) * gain
        return int(round(min(counts, max(0.0, v / adc_ref_v * counts))))

    trip_a = i_max * trip_fraction
    trip_counts = adc_for(trip_a)
    full_scale_a = adc_ref_v / gain / (shunt_mohm / 1000.0)
    n = pts[0].samples if pts else 10

    warn = ""
    if adc_for(i_max) >= counts:
        warn = (f"//  !! WARNING: {i_max:.0f} A saturates this front end "
                f"(full scale {full_scale_a:.0f} A). Lower the gain or the "
                f"shunt.\n")

    levels = "\n".join(
        f"//    {p.current_mult:g}x = {p.current_a:8.1f} A   expected "
        f"{'%.1f ms' % (p.expected_time_s*1000) if p.expected_time_s else '?'}"
        f"{'' if p.measurable else '   <-- BELOW RIG RESOLUTION'}"
        for p in pts) or "//    (no levels planned)"

    return f"""// ===========================================================================
//  Fuse time-to-blow rig  —  generated by KinematiK suspension/fuse_test.py
//
//  Fuse under test : {fuse.label} ({fuse.rating_a or 0:.0f} A{', ' + fuse.part_number if fuse.part_number else ''})
//  Front end       : {shunt_mohm:g} mOhm shunt, gain {gain:g}, {adc_bits}-bit ADC
//                    full scale {full_scale_a:.0f} A, {full_scale_a/counts:.2f} A per count
//  Trip threshold  : {trip_counts} counts ~ {trip_a:.0f} A (current collapse)
//  Samples/level   : {n}
{warn}//
//  Planned levels:
{levels}
//
//  TIMING NOTES — the reason this is not the proof-of-concept sketch:
//    * The stop mark is the current collapsing past a threshold, NOT a
//      keypress. Operator reaction time (~250 ms) is larger than most blow
//      times worth measuring and biases every reading in one direction.
//    * micros(), not millis(). A 1 ms tick is 20% of a 5 ms event.
//    * Nothing prints between the marks. Serial.println() blocks; 30 chars
//      at 9600 baud is 31 ms silently added to the measurement.
//    * analogRead() on an AVR takes ~104 us, so detection latency is about
//      0.1 ms. Events faster than ~2 ms need a comparator, not this loop.
// ===========================================================================

const uint8_t  PIN_SENSE   = A0;
const uint16_t TRIP_COUNTS = {trip_counts};   // current-collapse threshold
const uint16_t ARM_COUNTS  = {max(1, trip_counts // 2)};   // must exceed this to arm
const uint8_t  N_SAMPLES   = {n};
const uint32_t TIMEOUT_US  = 30000000UL;      // 30 s: fuse did not blow

uint32_t results[N_SAMPLES];
uint8_t  count = 0;

void setup() {{
  Serial.begin(115200);                 // faster than 9600: prints hurt less
  while (!Serial) {{ ; }}
  analogReference(DEFAULT);
  Serial.println(F("Fuse rig ready. Apply current; detection is automatic."));
  Serial.print(F("Trip threshold: ")); Serial.print(TRIP_COUNTS);
  Serial.println(F(" counts"));
}}

// Blocking measurement of one blow event. No I/O inside the timed region.
bool measure_one(uint32_t *out) {{
  // --- arm: wait for current to establish -------------------------------- //
  while (analogRead(PIN_SENSE) < ARM_COUNTS) {{ ; }}
  uint32_t t0 = micros();

  // --- wait for the current to collapse ---------------------------------- //
  while (analogRead(PIN_SENSE) >= TRIP_COUNTS) {{
    if (micros() - t0 > TIMEOUT_US) return false;
  }}
  *out = micros() - t0;                 // wraps correctly on unsigned math
  return true;
}}

void report() {{
  // Sort a copy for the median. n is small; insertion sort is fine.
  uint32_t s[N_SAMPLES];
  for (uint8_t i = 0; i < count; i++) s[i] = results[i];
  for (uint8_t i = 1; i < count; i++) {{
    uint32_t v = s[i]; int8_t j = i - 1;
    while (j >= 0 && s[j] > v) {{ s[j+1] = s[j]; j--; }}
    s[j+1] = v;
  }}
  uint32_t med = (count & 1) ? s[count/2] : (s[count/2 - 1] + s[count/2]) / 2;

  Serial.println(F("--------------------------------"));
  Serial.print(F("n = ")); Serial.println(count);
  for (uint8_t i = 0; i < count; i++) {{
    Serial.print(F("  ")); Serial.print(results[i] / 1000.0, 3);
    Serial.println(F(" ms"));
  }}
  Serial.print(F("min    ")); Serial.print(s[0] / 1000.0, 3);
  Serial.println(F(" ms"));
  Serial.print(F("median ")); Serial.print(med / 1000.0, 3);
  Serial.println(F(" ms"));
  Serial.print(F("max    ")); Serial.print(s[count-1] / 1000.0, 3);
  Serial.println(F(" ms"));
  Serial.print(F("spread ")); Serial.print((float)s[count-1] / (float)s[0], 2);
  Serial.println(F("x  <-- report this, not a single reading"));
  Serial.println(F("--------------------------------"));
}}

void loop() {{
  if (count >= N_SAMPLES) {{ report(); count = 0; delay(5000); return; }}

  uint32_t us;
  if (measure_one(&us)) {{
    results[count++] = us;              // print AFTER the timed region
    Serial.print(F("blow ")); Serial.print(us / 1000.0, 3);
    Serial.print(F(" ms   (")); Serial.print(count);
    Serial.print(F("/")); Serial.print(N_SAMPLES); Serial.println(F(")"));
  }} else {{
    Serial.println(F("TIMEOUT - fuse did not open. Raise the current."));
  }}
  Serial.println(F("Fit the next fuse; measurement re-arms automatically."));
  delay(2000);
}}
"""


# ===================================================================== #
#  8.  PROVENANCE
# ===================================================================== #
PROVENANCE = {
    "physics_grounded": [
        "wire withstand from the IEC 60949 adiabatic equation I²t = k²S², "
        "with k derived from conductor and insulation temperatures — the "
        "derivation reproduces the published IEC 60364-5-54 values for copper "
        "PVC (115), copper PVC-90 (100), copper XLPE (143), copper rubber "
        "(141) and aluminium PVC (76)",
        "fuse/wire crossover as the analytic solution of t_fuse(I) = t_wire(I) "
        "for a power-law fuse against an I^-2 conductor curve",
        "constant-I²t continuation above the fastest declared anchor, because "
        "a fuse becomes energy-limited once the element melts faster than heat "
        "can leave it — extrapolating the power law instead overstates how "
        "fast the fuse is, in the dangerous direction",
        "sample count from the log-normal confidence interval on a median",
        "pooled within-family variance of ln(t) across current levels",
        "upper confidence bound on the estimated scatter via the "
        "Wilson-Hilferty chi-square approximation, verified against published "
        "quantiles (within 0.25% by 10 dof, erring low and therefore "
        "conservative below that)",
        "instrument uncertainty as quantisation (resolution/sqrt(12)) combined "
        "in quadrature with latency jitter",
        "UART blocking time from 8N1 byte framing",
    ],
    "declared_not_invented": [
        "the fuse time-current curve, fitted only through anchor points read "
        "off a datasheet; a fuse with no anchors gets no curve and "
        "coordination against it is refused",
    ],
    "measured_when_available": [
        "the unit-to-unit blow-time scatter that sizes the test. "
        "PRIOR_LOG_SCATTER = 0.30 is a PLACEHOLDER, used only until a pilot "
        "displaces it: pooled_log_scatter() estimates it from your own parts, "
        "refine_from_pilot() re-sizes the test from that, and build_test_plan() "
        "emits a MISSING finding whenever the count still rests on the prior. "
        "The count is quadratic in this quantity, so the difference between "
        "assumed and measured is the difference between 5 and 50 fuses per "
        "level, not a refinement.",
    ],
    "estimate_flagged": [
        "the 0.75 continuous derate for blade fuses in a hot enclosure",
        "human reaction time of 250 ms in the proof-of-concept instrument",
        "adiabatic k for automotive insulations (TXL/GXL, PTFE, silicone) is "
        "derived from the same verified formula but from datasheet "
        "temperatures rather than a standard's table",
    ],
    "hard_rule": (
        "A measurement whose uncertainty is dominated by the instrument is "
        "REJECTED from the fit rather than folded in. A rig with a 250 ms "
        "detection latency reporting a 47 ms blow time has measured its "
        "operator, and averaging such readings launders them into something "
        "shaped like a curve. Likewise coordination against an undeclared "
        "fuse curve is refused, because a protected range computed from an "
        "invented characteristic is indistinguishable in presentation from a "
        "real one — and a test sized on an assumed scatter says so, because a "
        "fuse count carries the authority of whatever produced it."
    ),
}
