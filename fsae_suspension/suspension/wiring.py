# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/wiring.py — conductor sizing, derated for a car and not a wall
# ============================================================================
"""
Wire sizing that starts from the NEC table and then leaves the building.

THE TABLE, AND WHY IT IS ONLY A STARTING POINT

The base ampacities here are the standard NEC building-wire figures (2017
edition, as published in Cerrowire's ampacity chart). They are good numbers
and they are the ones every team reaches for, because they are the ones a
search engine returns. They are also derived for conditions an FSAE car does
not have, and the publisher says so plainly: the values carry no temperature
correction and no bundling adjustment.

Four mismatches, in rough order of how much trouble they cause:

  AMBIENT. The table assumes 30 °C. An accumulator interior on a July grid,
  or a loom routed past a motor housing, sits at 50–70 °C. The NEC's own
  correction factors take a 90 °C conductor down to 71 % at 60 °C ambient and
  41 % at 80 °C. A 6 AWG cable "rated 75 A" is a 53 A cable where you put it.

  BUNDLING. The table is one conductor in free air or a small raceway. A car
  loom is twenty conductors in a sleeve. The NEC adjustment for 10–20
  current-carrying conductors is 50 %. Bundling and ambient MULTIPLY, and a
  hot bundle lands near a third of the table value.

  CONSTRUCTION. FSAE runs fine-strand silicone or Tefzel (M22759), rated 150
  or 200 °C, not 7-strand THHN. For those the 90 °C column is CONSERVATIVE —
  sometimes by a lot — and using it wastes copper and mass. This module will
  not silently apply a 200 °C rating to a wire you have not named, but it
  will tell you when the table is the thing holding you back.

  DUTY. Ampacity is a continuous rating. A tractive-system cable sees 80 kW
  for a few seconds and a much lower RMS across a lap. Sizing on the peak
  wastes mass; sizing on the average melts things. The honest input is the
  RMS current from your own log, which `size_from_log` computes.

AND THE THING THAT IS USUALLY NOT AMPACITY AT ALL

Long low-voltage runs are almost never limited by heat. They are limited by
VOLTAGE DROP: a conductor that is thermally comfortable can still drop enough
volts over a six-metre round trip to brown out an ECU on a starter crank.
`check_run` reports both and tells you which one governs, because the answer
is frequently the one nobody checked.

FUSES PROTECT WIRE, NOT LOADS

A fuse is chosen so the conductor never becomes the fuse. If the fuse rating
exceeds the derated ampacity of the wire it feeds, the wire is the weakest
element in the path and it will fail somewhere inaccessible, on track. That
check is one line and it is the one most often skipped.

Nothing here is a substitute for the rules, the NEC, or a qualified engineer,
and none of it has been validated against your loom. Print the derated number,
then measure a conductor's temperature under real load before you trust it.

Pure Python + NumPy. Self-test: python3 -m suspension.wiring
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field as _dcfield
from collections.abc import Sequence

import numpy as np


__all__ = ["Conductor", "AWG", "SOURCE", "ampacity", "derate",
           "DerateResult", "RunCheck", "check_run", "recommend_gauge",
           "size_from_log", "LogSizing", "fuse_coordination",
           "TEMP_CORRECTION", "BUNDLE_ADJUSTMENT", "INSULATIONS"]


SOURCE = ("NEC 2017 building-wire ampacities as published in Cerrowire's "
          "ampacity chart (cerrowire.com). Temperature correction and "
          "bundling adjustment are NOT included in those figures and are "
          "applied separately here.")


# =========================================================================== #
#  The table
# =========================================================================== #
@dataclass(frozen=True)
class Conductor:
    awg: str
    area_mm2: float
    r_ohm_per_km_20c: float      # copper, 20 °C
    a_60c: float | None       # NM-B, UF-B
    a_75c: float | None       # THW, THWN, XHHW
    a_90c: float | None       # THHN, XHHW-2, THWN-2
    in_nec_table: bool = True

    def base(self, insulation_c: int) -> float | None:
        return {60: self.a_60c, 75: self.a_75c, 90: self.a_90c}.get(
            insulation_c)


#  Areas and DC resistances are physical constants of the gauge. Ampacities
#  are the published NEC copper columns. Gauges below 14 AWG are NOT in that
#  table — they are carried here for area, resistance and voltage drop, with
#  ampacity left as None rather than invented, because a made-up ampacity on
#  a signal wire is exactly the kind of confident wrong number this toolkit
#  exists to refuse.
AWG: dict[str, Conductor] = {
    "22":  Conductor("22", 0.326, 52.96, None, None, None, False),
    "20":  Conductor("20", 0.518, 33.31, None, None, None, False),
    "18":  Conductor("18", 0.823, 20.95, None, None, None, False),
    "16":  Conductor("16", 1.31,  13.17, None, None, None, False),
    "14":  Conductor("14", 2.08,   8.286, 15.0,  20.0,  25.0),
    "12":  Conductor("12", 3.31,   5.211, 20.0,  25.0,  30.0),
    "10":  Conductor("10", 5.26,   3.277, 30.0,  35.0,  40.0),
    "8":   Conductor("8",  8.37,   2.061, 40.0,  50.0,  55.0),
    "6":   Conductor("6", 13.3,    1.296, 55.0,  65.0,  75.0),
    "4":   Conductor("4", 21.2,    0.8152, 70.0, 85.0,  95.0),
    "3":   Conductor("3", 26.7,    0.6465, 85.0, 100.0, 115.0),
    "2":   Conductor("2", 33.6,    0.5127, 95.0, 115.0, 130.0),
    "1":   Conductor("1", 42.4,    0.4065, None, 130.0, 145.0),
    "1/0": Conductor("1/0", 53.5,  0.3224, None, 150.0, 170.0),
    "2/0": Conductor("2/0", 67.4,  0.2557, None, 175.0, 195.0),
    "3/0": Conductor("3/0", 85.0,  0.2028, None, 200.0, 225.0),
    "4/0": Conductor("4/0", 107.2, 0.1608, None, 230.0, 260.0),
}

_ORDER = ["22", "20", "18", "16", "14", "12", "10", "8", "6", "4", "3", "2",
          "1", "1/0", "2/0", "3/0", "4/0"]

#  The NEC's published ambient-correction factors, kept ONLY as a validation
#  reference. They are not used to compute anything — see `correction_factor`,
#  which derives them — and the self-test checks the derivation against every
#  one of these 19 numbers.
TEMP_CORRECTION: dict[int, list[tuple[float, float]]] = {
    75: [(35, 0.94), (40, 0.88), (45, 0.82), (50, 0.75), (55, 0.67),
         (60, 0.58), (70, 0.33)],
    90: [(35, 0.96), (40, 0.91), (45, 0.87), (50, 0.82), (55, 0.76),
         (60, 0.71), (70, 0.58), (80, 0.41)],
    60: [(35, 0.91), (40, 0.82), (45, 0.71), (50, 0.58)],
}

#  The NEC tables are built on a 30 °C ambient.
_NEC_AMBIENT_C = 30.0
#  Copper temperature coefficient of resistance, per K about 20 °C.
_ALPHA_CU = 0.00393


def correction_factor(rating_c: float, ambient_c: float, *,
                      include_resistance_shift: bool = False) -> float:
    """The NEC ambient-correction factor for a published column. DERIVED.

    A conductor's rating is a permitted temperature RISE above ambient, and
    the heat it can shed is proportional to that rise. The heat is I²R, so
    the current scales as the square root:

        I ∝ √( T_rating − T_ambient )

    This is not a model invented here — it is the relation the NEC's own
    tables are built from. `_selftest` checks it against all nineteen
    published factors and reproduces every one within 0.005, which is the
    rounding of the published table itself.

    NO RESISTANCE SHIFT HERE — the parameter is refused rather than ignored.
    Both endpoints of THIS ratio have the conductor sitting at the same rating
    temperature (that is what a rating means); only the ambient differs. Copper
    resistance is therefore identical top and bottom and cancels exactly. There
    is nothing to correct.

    The term used to be applied anyway, copied from `ampacity_scale` where it IS
    right (there the two endpoints really are at different conductor
    temperatures — a published column's rating versus the conductor's own). Here
    it reduced to a function of rating_c alone, exactly 1.0 at 90 C and
    increasingly wrong below it: against the published table it overshot by
    0.048 at the 60 C column, ten times the accuracy this docstring claims. The
    sign matters more than the size. Every error was POSITIVE, i.e. it said the
    conductor could carry more current than the NEC allows, which is the one
    direction an ampacity tool must never be wrong in.

    Raising beats silently ignoring: a caller passing True believed they were
    getting a refinement, and should find out they were not.
    """
    rise = float(rating_c) - float(ambient_c)
    base = float(rating_c) - _NEC_AMBIENT_C
    if rise <= 0 or base <= 0:
        return 0.0
    if include_resistance_shift:
        raise ValueError(
            "correction_factor() has no resistance shift to apply — see the note "
            "in its docstring. Use ampacity_scale() if you are scaling ACROSS "
            "rating temperatures.")
    return math.sqrt(rise / base)


#  Published columns, richest first. A conductor rated above 90 °C is scaled
#  FROM the 90 °C column, which is the highest the table publishes.
_COLUMNS: list[tuple[int, str]] = [(90, "a_90c"), (75, "a_75c"),
                                   (60, "a_60c")]


def _column_for(effective_rating_c: float) -> tuple[int, str]:
    """The published column an effective rating should be scaled from."""
    for col, attr in _COLUMNS:
        if effective_rating_c >= col:
            return col, attr
    return _COLUMNS[-1]


def ampacity_scale(effective_rating_c: float, ambient_c: float, *,
                   include_resistance_shift: bool = True
                   ) -> tuple[float, int]:
    """Scale factor from the appropriate published column, and which column.

    Same square-root law, but anchored on a REAL published number rather than
    on the rating's own hypothetical 30 °C value. That distinction is what
    makes the extrapolation to 150 and 200 °C wire meaningful instead of
    circular: a 200 °C conductor is compared against the 90 °C column it is
    genuinely better than, not against an imaginary 200 °C-at-30 °C entry.
    """
    col, _attr = _column_for(effective_rating_c)
    rise = float(effective_rating_c) - float(ambient_c)
    base = float(col) - _NEC_AMBIENT_C
    if rise <= 0 or base <= 0:
        return 0.0, col
    f = math.sqrt(rise / base)
    if include_resistance_shift:
        #  Copper resistance rises with temperature, so a conductor allowed to
        #  sit at 200 °C burns more per amp than the 90 °C column assumes.
        #  The NEC's published factors omit this; it costs about 8 % at
        #  150 °C and 13 % at 200 °C, and omitting it would be the optimistic
        #  direction — which is the wrong one to be wrong in.
        f *= math.sqrt((1.0 + _ALPHA_CU * (float(col) - 20.0))
                       / (1.0 + _ALPHA_CU * (float(effective_rating_c) - 20.0)))
    return f, col


#  NEC 310.15(C)(1) adjustment for current-carrying conductors bundled
#  together. A car loom is squarely in the 10–20 band.
BUNDLE_ADJUSTMENT: list[tuple[int, float]] = [
    (3, 1.00), (6, 0.80), (9, 0.70), (20, 0.50), (30, 0.45), (40, 0.40),
    (999, 0.35),
]

#  insulation key → (continuous conductor rating °C, note)
INSULATIONS: dict[str, tuple[int, str]] = {
    "nmb":      (60,  "NM-B / UF-B building wire."),
    "thw":      (75,  "THW / THWN building wire."),
    "thhn":     (90,  "THHN / THWN-2 / XHHW-2 building wire — the column the "
                      "NEC table is anchored on."),
    "xhhw2":    (90,  "XHHW-2 building wire."),
    "tefzel":   (150, "M22759/16 ETFE (Tefzel), 150 °C continuous. The "
                      "aerospace-derived wire most FSAE teams actually run."),
    "silicone": (200, "Fine-strand silicone, 200 °C continuous. Very "
                      "flexible, poor abrasion resistance — sleeve it."),
    "ptfe":     (200, "M22759/11 PTFE, 200 °C continuous."),
    "xlpe125":  (125, "Cross-linked polyolefin, 125 °C continuous."),
}

#  Common termination temperature ratings. A conductor is only as good as
#  what it lands in, and this is the limit that actually bites.
TERMINATIONS: dict[str, tuple[int, str]] = {
    "ring_lug_105":  (105, "Typical crimp ring lug / heat-shrink boot."),
    "connector_125": (125, "Deutsch DTM / Autosport-style contact."),
    "connector_150": (150, "High-temperature contact system."),
    "busbar_200":    (200, "Bolted busbar with high-temperature hardware."),
}


def conductor(awg: str) -> Conductor:
    k = str(awg).strip().upper().replace("AWG", "").strip()
    if k in AWG:
        return AWG[k]
    raise KeyError(f"unknown gauge '{awg}' — have {', '.join(_ORDER)}")


def _lookup(bands: Sequence[tuple[float, float]], value: float) -> float:
    for upper, factor in bands:
        if value <= upper:
            return factor
    return bands[-1][1]


def rating_of(insulation: str) -> int:
    return INSULATIONS.get(insulation.lower(), (90, ""))[0]


def ampacity(awg: str, insulation: str = "thhn",
             ambient_c: float = _NEC_AMBIENT_C) -> float | None:
    """Single-conductor ampacity for any insulation rating and ambient.

    Anchored on the published 90 °C column and scaled by `correction_factor`.
    Passing thhn at 30 °C returns the table value exactly.
    """
    c = conductor(awg)
    if c.a_90c is None:
        return None
    rating = rating_of(insulation)
    f, col = ampacity_scale(rating, ambient_c)
    base = getattr(c, _column_for(rating)[1])
    return base * f if base is not None else None


# =========================================================================== #
#  Derating
# =========================================================================== #
@dataclass
class DerateResult:
    conductor: Conductor
    insulation: str
    rating_c: int
    base_a: float | None
    temp_factor: float
    bundle_factor: float
    allowed_a: float | None
    notes: list[str] = _dcfield(default_factory=list)

    @property
    def total_factor(self) -> float:
        return self.temp_factor * self.bundle_factor


def derate(awg: str, *, insulation: str = "thhn", ambient_c: float = 30.0,
           n_bundled: int = 1,
           termination_c: float | None = None) -> DerateResult:
    """What the conductor may actually carry where you put it.

    Three independent limits, and the third is the one that bites:

      AMBIENT — derived, and it now works for 150 and 200 °C wire instead of
      flooring everything at 90.

      BUNDLING — NEC 310.15(C)(1). Multiplies with ambient.

      TERMINATION — a 200 °C conductor landing in a 105 °C ring lug is a
      105 °C circuit. The wire is almost never the limit on a well-built car;
      the crimp, the boot and the connector body are. Leaving this unstated
      is how a correctly-specified cable still melts at its end.
    """
    c = conductor(awg)
    ins_rating = rating_of(insulation)
    ins_note = INSULATIONS.get(insulation.lower(), (90, "unknown"))[1]

    effective = float(ins_rating)
    term_limited = False
    if termination_c is not None and float(termination_c) < effective:
        effective = float(termination_c)
        term_limited = True

    tf, col = ampacity_scale(effective, ambient_c)
    base = getattr(c, _column_for(effective)[1])
    bf = _lookup([(float(a), f) for a, f in BUNDLE_ADJUSTMENT],
                 float(max(n_bundled, 1)))
    allowed = base * tf * bf if base is not None else None

    notes = [f"{ins_note} Continuous rating {ins_rating} °C."]
    if base is None:
        notes.append(
            f"{c.awg} AWG is not in the NEC building-wire table, which starts "
            f"at 14 AWG. Area and resistance below are physical constants and "
            f"are valid; the ampacity is left blank rather than invented. Use "
            f"the wire manufacturer's chassis-wiring rating for this gauge.")
    notes.append(
        f"Ampacity derived from the published {col} °C column, scaled by "
        f"√((T_rating − T_ambient)/(T_column − 30)) with the copper "
        f"resistance shift included: {effective:g} °C conductor in a "
        f"{ambient_c:g} °C ambient → ×{tf:.3f}.")
    if term_limited:
        notes.append(
            f"**Termination-limited.** The conductor is rated {ins_rating} °C "
            f"but it lands in a {termination_c:g} °C termination, so the "
            f"circuit is a {termination_c:g} °C circuit. Every amp above this "
            f"is bought by upgrading the lug, not the cable.")
    elif termination_c is None:
        notes.append(
            "No termination rating stated, so the conductor's own rating was "
            "used. On a real car this is usually optimistic — crimps, boots "
            "and connector bodies are typically 105–125 °C, and they, not the "
            "wire, set the limit.")
    if n_bundled > 3:
        notes.append(
            f"{n_bundled} current-carrying conductors bundled → ×{bf:.2f}. "
            f"Only conductors actually carrying current count; a spare or a "
            f"signal pair does not.")
    if base is not None and allowed is not None and allowed < base * 0.6:
        notes.append(
            f"Combined derate is ×{tf*bf:.2f}: this conductor may carry "
            f"{allowed:.0f} A where the {col} °C chart says {base:.0f} A.")
    if base is not None and allowed is not None and allowed > base:
        notes.append(
            f"This conductor beats the {col} °C chart value "
            f"({allowed:.0f} A vs "
            f"{base:.0f} A) because its insulation is rated well above the "
            f"column the chart is built on. That headroom is real and it is "
            f"mass you do not have to carry — but it is only collectable if "
            f"the terminations are rated for it too.")
    return DerateResult(c, insulation, int(effective), base, tf, bf, allowed,
                        notes)


# =========================================================================== #
#  A run: heat AND volts
# =========================================================================== #
@dataclass
class RunCheck:
    awg: str
    current_a: float
    length_m: float
    system_v: float
    derated_a: float | None
    thermal_margin: float | None
    resistance_ohm: float
    drop_v: float
    drop_pct: float
    loss_w: float
    governing: str
    ok: bool
    notes: list[str] = _dcfield(default_factory=list)


def check_run(awg: str, *, current_a: float, length_m: float,
              system_v: float, insulation: str = "thhn",
              ambient_c: float = 30.0, n_bundled: int = 1,
              conductor_temp_c: float | None = None,
              termination_c: float | None = None,
              max_drop_pct: float = 3.0) -> RunCheck:
    """Both failure modes, and which one governs.

    `length_m` is the ONE-WAY run; the resistance used is the round trip,
    because the return path is a conductor too and half of every voltage-drop
    error in a paddock comes from forgetting it.
    """
    d = derate(awg, insulation=insulation, ambient_c=ambient_c,
               n_bundled=n_bundled, termination_c=termination_c)
    c = d.conductor
    t_op = conductor_temp_c if conductor_temp_c is not None \
        else max(ambient_c + 20.0, 40.0)
    #  copper temperature coefficient, 0.00393 per K about 20 °C
    r_km = c.r_ohm_per_km_20c * (1.0 + 0.00393 * (t_op - 20.0))
    r = r_km / 1000.0 * (2.0 * float(length_m))
    drop = float(current_a) * r
    pct = 100.0 * drop / max(float(system_v), 1e-9)
    loss = float(current_a) ** 2 * r

    thermal = (d.allowed_a / current_a) if (d.allowed_a and current_a > 0) \
        else None
    volt_margin = max_drop_pct / pct if pct > 0 else float("inf")
    if thermal is None:
        governing = "voltage drop (no ampacity available for this gauge)"
    elif thermal < volt_margin:
        governing = "ampacity"
    else:
        governing = "voltage drop"

    notes = list(d.notes)
    notes.append(
        f"Round trip {2*length_m:g} m of {c.awg} AWG at {t_op:.0f} °C → "
        f"{r*1000:.2f} mΩ. At {current_a:g} A that is {drop:.2f} V "
        f"({pct:.2f} % of {system_v:g} V) and {loss:.0f} W burned in the "
        f"cable.")
    if governing == "voltage drop":
        notes.append(
            "**Voltage drop governs, not heat.** The conductor is thermally "
            "comfortable and still losing volts. This is the usual answer for "
            "long low-voltage runs and it is the one nobody checks — a "
            "browned-out ECU on a starter crank is this, every time.")
    if loss > 25.0:
        notes.append(
            f"{loss:.0f} W into the loom is also {loss:.0f} W the cooling "
            f"system never budgeted for, deposited along a cable that is "
            f"probably cable-tied to something you care about.")
    #  UNKNOWN IS NOT PASS. The first version treated a missing ampacity as
    #  "no thermal objection" and let voltage drop alone decide — which
    #  recommended 16 AWG for 132 A RMS, because 16 AWG is not in the NEC
    #  table and so raised no objection at all. A conductor whose ampacity
    #  this module does not know is a conductor this module will not clear.
    ok = bool(thermal is not None and thermal >= 1.0
              and pct <= max_drop_pct)
    if thermal is None:
        notes.append(
            f"**Not cleared.** {c.awg} AWG has no ampacity in the NEC "
            f"building-wire table, so the thermal check could not be made. "
            f"Unknown is not a pass — get the manufacturer's chassis-wiring "
            f"rating for this gauge before using it at {current_a:g} A.")
    return RunCheck(c.awg, float(current_a), float(length_m),
                    float(system_v), d.allowed_a, thermal, r, drop, pct,
                    loss, governing, ok, notes)


def parallel_needed(current_a: float, *, insulation: str = "thhn",
                    ambient_c: float = 30.0, n_bundled: int = 1,
                    awg: str = "4/0") -> tuple[int, float]:
    """How many conductors of `awg` in parallel would carry the current.

    When a single conductor cannot do the job, the answer is not a bigger
    number that does not exist — it is parallel runs, a higher-temperature
    wire, or a busbar. Returns (count, allowed_per_conductor)."""
    d = derate(awg, insulation=insulation, ambient_c=ambient_c,
               n_bundled=n_bundled)
    if not d.allowed_a:
        return 0, 0.0
    return int(math.ceil(current_a / d.allowed_a)), d.allowed_a


def recommend_gauge(*, current_a: float, length_m: float, system_v: float,
                    insulation: str = "thhn", ambient_c: float = 30.0,
                    n_bundled: int = 1, termination_c: float | None = None,
                    max_drop_pct: float = 3.0,
                    fos: float = 1.25) -> tuple[str | None, list[RunCheck]]:
    """Smallest gauge that clears BOTH tests, plus the whole ladder.

    Returns None for the pick when nothing in the table clears it. That is a
    real answer — see `parallel_needed` — and it is emphatically not the same
    as the smallest gauge that raised no objection."""
    ladder: list[RunCheck] = []
    pick = None
    for awg in _ORDER:
        r = check_run(awg, current_a=current_a * fos, length_m=length_m,
                      system_v=system_v, insulation=insulation,
                      ambient_c=ambient_c, n_bundled=n_bundled,
                      termination_c=termination_c,
                      max_drop_pct=max_drop_pct)
        ladder.append(r)
        if pick is None and r.ok:
            pick = awg
    return pick, ladder


# =========================================================================== #
#  Fuses protect wire
# =========================================================================== #
def fuse_coordination(awg: str, fuse_a: float, *, insulation: str = "thhn",
                      ambient_c: float = 30.0, n_bundled: int = 1
                      ) -> tuple[str, str]:
    """Does the fuse protect the conductor, or does the conductor protect the
    fuse? Returns (severity, message)."""
    d = derate(awg, insulation=insulation, ambient_c=ambient_c,
               n_bundled=n_bundled)
    if d.allowed_a is None:
        return ("unknown",
                f"{awg} AWG has no table ampacity, so fuse coordination "
                f"cannot be checked from this data. Get the manufacturer's "
                f"rating before trusting the {fuse_a:g} A fuse.")
    if fuse_a > d.allowed_a:
        return ("violation",
                f"**The {fuse_a:g} A fuse does not protect {awg} AWG.** "
                f"Derated ampacity is {d.allowed_a:.0f} A, so the wire is the "
                f"weakest element in this path — it becomes the fuse, "
                f"somewhere inside a loom, on track. Either drop the fuse to "
                f"{d.allowed_a:.0f} A or step the wire up.")
    if fuse_a > d.allowed_a * 0.9:
        return ("watch",
                f"{fuse_a:g} A fuse against {d.allowed_a:.0f} A of derated "
                f"wire — protected, but with under 10 % margin. Fuse curves "
                f"are slow and hot fuses drift; leave more room.")
    return ("ok",
            f"{fuse_a:g} A fuse against {d.allowed_a:.0f} A of derated wire — "
            f"the fuse opens first, which is the point.")


# =========================================================================== #
#  Sizing from a real log
# =========================================================================== #
@dataclass
class LogSizing:
    rms_a: float
    peak_a: float
    mean_a: float
    crest: float
    duty_over_rms: float
    recommended_awg: str | None
    peak_awg: str | None
    notes: list[str] = _dcfield(default_factory=list)


def size_from_log(current_a: Sequence[float], *, length_m: float,
                  system_v: float, insulation: str = "thhn",
                  ambient_c: float = 30.0, n_bundled: int = 1,
                  max_drop_pct: float = 3.0) -> LogSizing:
    """Size on the RMS the car actually draws, not on the nameplate.

    Ampacity is a continuous rating and a race car's current is anything but.
    Sizing on the peak buys copper you carry for a whole endurance run to
    survive four seconds of it; sizing on the mean melts. RMS is the value
    that produces the same heating, so RMS is the value to size on — with the
    peak checked separately for voltage drop, because a sag at full power is
    a real problem even if it is a brief one.
    """
    i = np.asarray(current_a, float)
    i = i[np.isfinite(i)]
    if i.size == 0:
        return LogSizing(float("nan"), float("nan"), float("nan"),
                         float("nan"), float("nan"), None, None,
                         ["No usable current samples."])
    rms = float(np.sqrt(np.mean(i ** 2)))
    peak = float(np.max(np.abs(i)))
    mean = float(np.mean(np.abs(i)))
    crest = peak / rms if rms > 0 else float("nan")
    duty = float(np.mean(np.abs(i) > rms))

    rec, _l = recommend_gauge(current_a=rms, length_m=length_m,
                              system_v=system_v, insulation=insulation,
                              ambient_c=ambient_c, n_bundled=n_bundled,
                              max_drop_pct=max_drop_pct)
    pk, _l2 = recommend_gauge(current_a=peak, length_m=length_m,
                              system_v=system_v, insulation=insulation,
                              ambient_c=ambient_c, n_bundled=n_bundled,
                              max_drop_pct=max_drop_pct)
    notes = [
        f"RMS **{rms:.1f} A**, peak **{peak:.1f} A**, mean {mean:.1f} A — "
        f"crest factor {crest:.2f}.",
        f"The car spends {duty:.0%} of the log above its own RMS.",
    ]
    if rec != pk:
        notes.append(
            f"Sizing on RMS gives {rec} AWG; sizing on the peak gives {pk} "
            f"AWG. The difference is copper you would carry for the whole "
            f"event to survive the worst few seconds — worth it for voltage "
            f"drop at full power, not for heat.")
    else:
        notes.append(
            f"RMS and peak both land on {rec} AWG, so there is no trade to "
            f"make here.")
    if crest > 3.0:
        notes.append(
            f"A crest factor of {crest:.1f} is high. Check that the peak is "
            f"real current and not a sensor artefact before sizing anything "
            f"around it — one bad sample sets this number.")
    return LogSizing(rms, peak, mean, crest, duty, rec, pk, notes)


# =========================================================================== #
#  Self-test
# =========================================================================== #
def _selftest() -> int:
    fails = 0

    def chk(name, cond, detail=""):
        nonlocal fails
        print(f"  {'PASS' if cond else 'FAIL'}  {name}"
              + (f"   {detail}" if detail and not cond else ""))
        if not cond:
            fails += 1

    print("· the derivation IS the published table")
    worst = 0.0
    n_checked = 0
    for rating, rows in TEMP_CORRECTION.items():
        for amb, published in rows:
            derived = correction_factor(rating, amb,
                                        include_resistance_shift=False)
            worst = max(worst, abs(derived - published))
            n_checked += 1
    chk(f"reproduces all {n_checked} published NEC factors within 0.005",
        worst < 0.005, f"worst deviation {worst:.4f}")
    chk("and recovers the anchor exactly",
        abs(correction_factor(90, 30, include_resistance_shift=False) - 1.0)
        < 1e-12)

    print("· high-temperature wire now gets a number, not an apology")
    a90 = ampacity("6", "thhn", 60.0)
    a150 = ampacity("6", "tefzel", 60.0)
    a200 = ampacity("6", "silicone", 60.0)
    chk("150 C tefzel beats 90 C thhn at the same ambient", a150 > a90 * 1.5,
        f"{a90:.1f} vs {a150:.1f}")
    chk("200 C silicone beats 150 C tefzel", a200 > a150,
        f"{a150:.1f} vs {a200:.1f}")
    chk("tefzel at 60 C ambient beats the 30 C chart value",
        a150 > conductor("6").a_90c, f"{a150:.1f} vs {conductor('6').a_90c}")

    print("· terminations are the real limit")
    free = derate("6", insulation="silicone", ambient_c=60.0)
    lug = derate("6", insulation="silicone", ambient_c=60.0,
                 termination_c=105.0)
    chk("a 105 C lug throttles a 200 C wire", lug.allowed_a < free.allowed_a,
        f"{free.allowed_a:.1f} vs {lug.allowed_a:.1f}")
    chk("and the report says which", any("Termination-limited" in n
                                          for n in lug.notes))
    chk("an unstated termination is flagged as optimistic",
        any("usually optimistic" in n for n in free.notes))

    print("· table")
    chk("6 AWG 90 C is 75 A", ampacity("6", "thhn") == 75.0)
    chk("2 AWG 75 C is 115 A", conductor("2").a_75c == 115.0)
    chk("small gauges have no invented ampacity",
        ampacity("20", "thhn") is None)

    print("· derating")
    hot = derate("6", ambient_c=60.0, n_bundled=12)
    chk("ambient and bundling multiply",
        abs(hot.total_factor - hot.temp_factor * 0.50) < 1e-12)
    chk("75 A chart becomes ~25 A in a hot loom",
        23.0 < hot.allowed_a < 28.0, str(hot.allowed_a))
    cold = derate("6", ambient_c=30.0, n_bundled=1)
    chk("thhn at the NEC basis recovers the table exactly",
        abs(cold.allowed_a - 75.0) < 1e-9, str(cold.allowed_a))

    print("· runs")
    r = check_run("14", current_a=15.0, length_m=4.0, system_v=12.0)
    chk("long LV run is governed by volts", r.governing == "voltage drop",
        r.governing)
    chk("and it fails the 3 % limit", not r.ok, f"{r.drop_pct}")
    hv = check_run("2", current_a=100.0, length_m=1.5, system_v=400.0)
    chk("short HV run passes", hv.ok, f"{hv.drop_pct} {hv.thermal_margin}")

    print("· recommendation")
    pick, ladder = recommend_gauge(current_a=15.0, length_m=4.0,
                                   system_v=12.0)
    chk("recommends something", pick is not None, str(pick))
    chk("recommendation actually passes",
        next(x for x in ladder if x.awg == pick).ok)

    print("· fuses")
    sev, _m = fuse_coordination("6", 60.0, ambient_c=60.0, n_bundled=12)
    chk("over-fused hot wire is a violation", sev == "violation", sev)
    sev2, _m2 = fuse_coordination("6", 20.0, ambient_c=60.0, n_bundled=12)
    chk("properly fused is ok", sev2 == "ok", sev2)

    print("· unknown is not pass")
    r16 = check_run("16", current_a=130.0, length_m=1.5, system_v=400.0)
    chk("a gauge with no ampacity is never cleared", not r16.ok)
    chk("and it says why", any("Unknown is not a pass" in n for n in r16.notes))
    pick_big, _l = recommend_gauge(current_a=200.0, length_m=1.5,
                                   system_v=400.0, ambient_c=60.0,
                                   n_bundled=12)
    chk("impossible current returns no gauge", pick_big is None,
        str(pick_big))
    n_par, per = parallel_needed(200.0, ambient_c=60.0, n_bundled=12)
    chk("parallel count is offered instead", n_par >= 2,
        f"{n_par} x {per:.0f} A")

    print("· from a log")
    t = np.linspace(0, 60, 6000)
    cur = 60.0 + 120.0 * np.clip(np.sin(2 * np.pi * t / 7.0), 0, 1)
    ls = size_from_log(cur, length_m=1.5, system_v=400.0)
    chk("rms sits between mean and peak",
        ls.mean_a < ls.rms_a < ls.peak_a,
        f"{ls.mean_a} {ls.rms_a} {ls.peak_a}")
    chk("a gauge comes back", ls.recommended_awg is not None)

    print(f"\n{'ALL PASS' if not fails else f'{fails} FAILURES'}")
    return 1 if fails else 0


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
