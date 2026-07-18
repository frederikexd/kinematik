# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  Module: saboteur — adversarial pre-flight (mutation testing for input decks)
# ============================================================================
"""
The Saboteur — KinematiK corrupts your car before ANSYS can.

THE PROBLEM THIS SOLVES
-----------------------
The Proof Engine's DISCREPANT verdict catches a garbage run when the result
lands OUTSIDE the plausibility envelope. That leaves exactly one class of
error uncovered, and it is the worst one:

    THE GARBAGE THAT FLATTERS THE ENVELOPE.

A pounds-into-kilograms slip on one subsystem, a dropped term in the mass
roll-up, lb·ft typed into an N·m field — each of these can move the answer
by an amount that still LOOKS plausible. The run comes back, the number is
believable, the contract says PASS, and the team acts on it. Nobody audits a
result that confirms what they expected. Confirmation bias does the rest.

Software engineering solved the mirror-image problem decades ago with
MUTATION TESTING: deliberately inject known bug classes into the code and
check whether the test suite notices. A test suite that passes on mutated
code is a test suite with holes — and now you know exactly where they are.
No CAE, PLM, or requirements tool has ever applied this to the input deck.
This module does.

WHAT IT DOES
------------
  1. THE SABOTAGE SWEEP. A catalog of named mutations — each one a real,
     documented failure class from the prevalidation bottleneck map (unit
     thousandfold slips, imperial-into-SI, coordinate-frame Z flips, dropped
     and double-counted roll-up terms) — is applied one at a time to a shadow
     copy of the uncertainty ledger. For every (mutation, target) pair the
     objective is re-evaluated and the question is asked: WOULD ANYONE
     NOTICE? A corruption that moves the objective beyond its own 3-sigma
     band is caught by the Proof Engine's existing envelope. One that does
     not is a SILENT KILLER: the exact error that would come back from ANSYS
     wearing a plausible face.

  2. TRIPWIRES, CHOSEN BY ARITHMETIC. A tripwire is a cheap secondary
     observable — total model mass, specific power, torque-per-power,
     implied pack voltage — that the team asks the sim to report ALONGSIDE
     the primary result. The comparison a tripwire makes is DECK vs RUN,
     and that distinction carries the whole design: the Proof Engine asks
     whether the deck matches REALITY (its bands come from evidence grades,
     and a validation contract judges that question); a tripwire asks
     whether the run consumed THE DECK IT WAS GIVEN. A solver that meshed
     the declared geometry must reproduce the deck's own arithmetic —
     its mass printout must equal the declared roll-up — to within a tight
     CONSISTENCY TOLERANCE (meshing simplification, rounding), no matter
     how uncertain the declared numbers are about the real car. So each
     wire carries a documented per-wire tolerance, not a ledger-derived
     band; otherwise the wires would go blind exactly when the deck is
     most uncertain, which is exactly when garbage is most likely. For
     every silent killer, the sweep computes which tripwires deviate
     beyond their tolerance under that corruption — i.e. which cheap check
     would have exposed it. A greedy set-cover then picks the SMALLEST set
     of tripwires that catches the most silent killers. The output is a
     PRE-FLIGHT SHEET: 3-6 numbers to record with every run, each with the
     band it must land in, chosen not by folklore but by detectability.

  3. HONEST BLIND SPOTS. Any mutation that neither the envelope nor any
     tripwire in the catalog can detect is reported OUT LOUD as a blind
     spot, with the only remaining defence named (measure that input
     directly). A coverage number that silently excludes the undetectable
     would be exactly the unearned green board this platform exists to end.

  4. THE SEALED SHEET. Like a validation contract, the pre-flight sheet is
     sha256-sealed at creation: the tripwire set, the clean values, and the
     bands are fixed before the run. Judging real readings against a
     tampered sheet is refused.

  5. FINGERPRINTING. When a run comes back and tripwires have fired, the
     pattern of WHICH wires moved and in WHICH direction is compared
     against the predicted signature of every catalogued corruption
     (cosine similarity on band-normalised deviations — deterministic,
     reproducible by hand). The verdict is not "something is wrong" but
     "this signature matches pounds-into-kg on the accumulator mass".
     The audit that used to take an evening now starts with a named
     suspect.

WHY NOBODY HAS BUILT THIS
-------------------------
Solver vendors sell answers; a tool that tells you which of their answers
would be undetectably wrong is not in their interest to build. Checklist
culture (pre-flight checks, sanity checks) has the right instinct but picks
its checks by folklore — nobody computes which check actually separates a
corrupted deck from a clean one, so teams check total mass out of habit and
miss the torque-unit slip that total mass cannot see. This module replaces
folklore with detectability arithmetic, and it reuses the exact uncertainty
ledger the Proof Engine already maintains, so it costs the team zero new
data entry.

HONESTY CONTRACT (same as the rest of KinematiK)
------------------------------------------------
Every number here is deterministic: same ledger in, same kill board, same
sheet, same fingerprints out. Mutations are documented transforms you can
apply with a calculator; tripwire bands are the same symmetric one-at-a-time
perturbations as the Proof Engine; set cover is greedy with fixed,
documented tie-breaks. Tripwire evaluators are closed forms over declared
channels, tagged with how to read the real-world counterpart. The sweep
covers the CATALOGUED corruption classes only — it never claims to certify
a deck against errors it does not model, and the export says so.

No streamlit / pandas / plotly imports. Pure stdlib. Unit-testable headless.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
from dataclasses import dataclass, field, asdict
from typing import Callable, Optional

from .proof_engine import (
    Objective, Quantity, aggregate, analyze_objective,
)

CATALOG_VERSION = "1"          # bump when mutation/tripwire catalogs change:
                               # a sealed sheet records the catalog it was
                               # built against, so an old sheet can never be
                               # silently judged with new arithmetic.

_SIGMA = 3.0                   # detection threshold, in units of a channel's
                               # own band — the same 3-sigma the Proof
                               # Engine's plausibility envelope uses, so
                               # "caught" means the same thing everywhere.

_SIGMA_CAP = 1e6               # ceiling on any reported separation. Keeps a
                               # divide-by-nothing (or an unevaluable wire,
                               # which IS a detection) finite, so signatures
                               # stay valid JSON and cosine fingerprints can
                               # never go NaN. 1e6 sigma is "certain" by any
                               # standard; nothing downstream needs more.


# --------------------------------------------------------------------------- #
#  Mutations — the catalog of ways a deck actually goes wrong
# --------------------------------------------------------------------------- #
# Each mutation names a real failure class from docs/BOTTLENECKS.md and the
# Frames & Datums war stories. `channels` limits where it can strike (an
# imperial mass slip cannot corrupt an airflow), `factor` is the value
# transform for scaling mutations, and `structural` marks the two roll-up
# mutations that add/remove a term instead of scaling one.

@dataclass(frozen=True)
class Mutation:
    key: str
    label: str
    story: str                     # the real-world failure this reproduces
    channels: tuple                # channel names it can strike ("*_mm" glob ok)
    factor: float = 1.0            # value' = value * factor   (scaling class)
    negate: bool = False           # value' = -value           (frame class)
    structural: str = ""           # "" | "drop" | "double"    (roll-up class)

    def applies(self, q: Quantity) -> bool:
        for pat in self.channels:
            if pat == q.channel:
                return True
            if pat.startswith("*") and q.channel.endswith(pat[1:]):
                return True
        return False

    def apply(self, quantities: list[Quantity], i: int) -> list[Quantity]:
        """Return a corrupted shadow copy; never touches the originals."""
        out = [Quantity.from_dict(q.as_dict()) for q in quantities]
        if self.structural == "drop":
            del out[i]
        elif self.structural == "double":
            dup = Quantity.from_dict(out[i].as_dict())
            dup.subsystem = out[i].subsystem + "~dup"
            dup.key = out[i].key + "~dup"
            out.append(dup)
        elif self.negate:
            out[i].value = -out[i].value
        else:
            out[i].value = out[i].value * self.factor
        return out


_LEN = ("*_mm",)
_MASS = ("mass_kg",)
_W = ("heat_reject_w", "power_draw_w")
_NM = ("*_nm",)
_SUM = ("mass_kg", "heat_reject_w", "power_draw_w")

DEFAULT_MUTATIONS: list[Mutation] = [
    Mutation("len_x1000", "metres typed into a mm field",
             "A CAD export in metres pasted into a millimetre deck: every "
             "length is 1000x off. The classic Shark import killer.",
             _LEN, factor=1000.0),
    Mutation("len_div1000", "mm typed into a metres field",
             "The same slip in the other direction — a 280 mm CG height "
             "arrives as 0.28 in a deck that wanted mm.",
             _LEN, factor=1.0 / 1000.0),
    Mutation("len_inch", "inches typed into a mm field",
             "A US supplier drawing dimensioned in inches, transcribed "
             "unconverted. Off by 25.4x — small enough to look like a "
             "different design instead of an error.",
             _LEN, factor=1.0 / 25.4),
    Mutation("mass_lb", "pounds typed into a kg field",
             "A cell datasheet in lb, a scale set to lb, a US catalogue "
             "value — the deck's mass silently grows 2.2x on one subsystem.",
             _MASS, factor=2.20462),
    Mutation("z_flip", "coordinate-frame Z flip on CG height",
             "SAE J670 is Z-down, ISO 8855 is Z-up. One hardpoint sheet in "
             "the wrong frame and the CG is BELOW the ground plane — the "
             "exact war story the Frames & Datums tab was built on.",
             ("cg_z_mm",), negate=True),
    Mutation("power_kilo", "the kilo prefix slips on a power channel",
             "3200 W of pack heat typed as 3.2 (kW into a W field), or a "
             "68 kW motor entered as 68000. The thousandfold twin of the "
             "length slip, on the thermal side.",
             _W, factor=1.0 / 1000.0),
    Mutation("torque_lbft", "lb-ft typed into an N-m field",
             "A US motor datasheet quotes lb-ft; the number lands in an "
             "N-m field unconverted, 26% low. Small enough to survive a "
             "glance, big enough to move a driveline sizing.",
             _NM, factor=1.0 / 1.35582),
    Mutation("force_lbf", "lbf typed into an N field",
             "A load case from an imperial-era reference used raw: mount "
             "loads 4.4x low, and the bracket that passes FEA is the one "
             "that fails at the car.",
             ("mount_load_n",), factor=1.0 / 4.44822),
    Mutation("drop_term", "one subsystem missing from a roll-up",
             "The deck was built from a six-week-old email and one "
             "subsystem's contribution never made it in — the "
             "eight-spreadsheet failure, entering a sim.",
             _SUM, structural="drop"),
    Mutation("double_count", "one subsystem counted twice in a roll-up",
             "Two leads both declared the coolant mass because the "
             "interface boundary was ambiguous. The roll-up is high by "
             "exactly one honest number.",
             _SUM, structural="double"),
]


# --------------------------------------------------------------------------- #
#  Tripwires — cheap observables with real-world read instructions
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Tripwire:
    """
    A secondary number recorded ALONGSIDE the primary result. `fn` evaluates
    it from the aggregated car dict; `needs` lists the channels it consumes
    (a wire is offered only when its inputs are declared); `how` tells a
    human where the real-world counterpart comes from — because a tripwire
    the team cannot actually read is theatre.
    """
    key: str
    label: str
    unit: str
    fn: Callable[[dict], float]
    needs: tuple
    how: str
    tol: float = 0.05              # consistency tolerance, relative: the run's
                                   # reading must land within ±tol of the
                                   # deck's own arithmetic. This is deck-vs-run
                                   # agreement (mesh simplification, rounding),
                                   # NOT deck-vs-reality — reality is the Proof
                                   # Engine's question, priced by evidence
                                   # grades over in proof_engine.py.
    doc: str = ""


def _tw_total_mass(car: dict) -> float:
    return car.get("mass_kg", float("nan"))


def _tw_cg_height(car: dict) -> float:
    return car.get("cg_z_mm", float("nan"))


def _tw_specific_power(car: dict) -> float:
    m = car.get("mass_kg", 0.0)
    return (car.get("peak_power_kw", float("nan")) * 1000.0 / m) if m else float("nan")


def _tw_heat_frac(car: dict) -> float:
    p = car.get("peak_power_kw", 0.0) * 1000.0
    return (car.get("heat_reject_w", float("nan")) / p) if p else float("nan")


def _tw_cooling_load(car: dict) -> float:
    v = car.get("cooling_airflow_cms", 0.0)
    return (car.get("heat_reject_w", float("nan")) / v) if v else float("nan")


def _tw_torque_per_power(car: dict) -> float:
    p = car.get("peak_power_kw", 0.0)
    return (car.get("peak_torque_nm", float("nan")) / p) if p else float("nan")


def _tw_brake_ratio(car: dict) -> float:
    t = car.get("peak_torque_nm", 0.0)
    return (car.get("brake_torque_nm", float("nan")) / t) if t else float("nan")


def _tw_pack_voltage(car: dict) -> float:
    a = car.get("peak_current_a", 0.0)
    return (car.get("peak_power_kw", float("nan")) * 1000.0 / a) if a else float("nan")


def _tw_mount_g(car: dict) -> float:
    m = car.get("mass_kg", 0.0)
    return (car.get("mount_load_n", float("nan")) / (9.81 * m)) if m else float("nan")


def _tw_lv_frac(car: dict) -> float:
    p = car.get("peak_power_kw", 0.0) * 1000.0
    return (car.get("power_draw_w", float("nan")) / p) if p else float("nan")


DEFAULT_TRIPWIRES: list[Tripwire] = [
    Tripwire("total_mass_kg", "Total rolled-up mass", "kg", _tw_total_mass,
             ("mass_kg",),
             "ANSYS/mesher mass-properties printout; the deck's own roll-up. "
             "The single cheapest checksum in engineering.", tol=0.02),
    Tripwire("cg_z_mm", "Rolled-up CG height", "mm", _tw_cg_height,
             ("cg_z_mm",),
             "CAD mass-properties Z of the meshed model. Negative means a "
             "frame flip, full stop.", tol=0.05),
    Tripwire("specific_power_wkg", "Specific power", "W/kg",
             _tw_specific_power, ("peak_power_kw", "mass_kg"),
             "Deck peak power over deck mass. FSAE-EV cars live near "
             "200-400 W/kg; a thousandfold or lb slip throws this outside "
             "any real car.", tol=0.05),
    Tripwire("heat_frac", "Heat rejected / peak electrical power", "-",
             _tw_heat_frac, ("heat_reject_w", "peak_power_kw"),
             "Pack+inverter+motor loss fraction implied by the deck. Physics "
             "pins it near 0.03-0.15; a kilo-prefix slip on either side "
             "leaves that window immediately.", tol=0.10),
    Tripwire("cooling_load_wcms", "Air-side loading", "W per m3/s",
             _tw_cooling_load, ("heat_reject_w", "cooling_airflow_cms"),
             "Heat over airflow — the number the radiator was sized to. "
             "Compare against the cooling tab's sizing point.", tol=0.10),
    Tripwire("torque_per_power", "Peak torque per peak power", "N·m/kW",
             _tw_torque_per_power, ("peak_torque_nm", "peak_power_kw"),
             "Implies motor base speed (9549/this = rpm). A lb-ft slip "
             "implies a base speed no catalogued motor has.", tol=0.05),
    Tripwire("brake_ratio", "Brake torque / drive torque", "-",
             _tw_brake_ratio, ("brake_torque_nm", "peak_torque_nm"),
             "Brakes must out-torque the motor with margin; the brakes tab "
             "computes the same ratio from the hydraulic side.", tol=0.05),
    Tripwire("pack_voltage_v", "Implied pack voltage", "V", _tw_pack_voltage,
             ("peak_power_kw", "peak_current_a"),
             "P/I from the deck. Must land near the accumulator's nominal "
             "(FSAE-EV: under 600 V). A current-unit slip implies an "
             "impossible pack.", tol=0.05),
    Tripwire("mount_g", "Mount load in vehicle-weight g", "g", _tw_mount_g,
             ("mount_load_n", "mass_kg"),
             "Peak mount load over m·g. Corner loads live in fractions of a "
             "g to a few g; an lbf slip parks this at an impossible "
             "value.", tol=0.10),
    Tripwire("lv_frac", "Continuous LV draw / peak power", "-", _tw_lv_frac,
             ("power_draw_w", "peak_power_kw"),
             "Housekeeping power fraction, sub-percent on a real car. A "
             "kilo slip on the LV budget is naked here.", tol=0.10),
]


def wire_band(w: Tripwire, clean: float) -> float:
    """
    The wire's 1-sigma-equivalent band, defined so the 3-sigma detection
    threshold equals the wire's stated consistency tolerance exactly:
    a reading counts as tripped when it deviates from the deck's own
    arithmetic by more than ±tol. The floor guards a legitimately-zero
    clean value (e.g. an undeclared-as-zero draw) from producing a zero
    band that would fire on numerical dust.
    """
    return w.tol * max(abs(clean), 1e-9) / _SIGMA


def available_tripwires(quantities: list[Quantity],
                        wires: Optional[list[Tripwire]] = None,
                        ) -> list[Tripwire]:
    """Wires whose every input channel is declared and whose clean value is
    finite. A wire that cannot be evaluated is not offered — an unreadable
    checksum is worse than none, because it looks like coverage."""
    car = aggregate(quantities)
    declared = set(car.keys())
    out = []
    for w in (wires if wires is not None else DEFAULT_TRIPWIRES):
        if not set(w.needs) <= declared:
            continue
        v = w.fn(car)
        if isinstance(v, float) and math.isfinite(v):
            out.append(w)
    return out


# --------------------------------------------------------------------------- #
#  The sweep — every mutation, every target, would anyone notice?
# --------------------------------------------------------------------------- #
@dataclass
class SabotageFinding:
    """One (mutation, target) pair and everything the sweep learned about it."""
    mutation_key: str
    mutation_label: str
    target_key: str                # Quantity.key it struck
    target_label: str
    delta_objective: float         # objective shift the corruption causes
    objective_sigmas: float        # |shift| / objective 1-sigma band
    envelope_catches: bool         # Proof Engine plausibility would flag it
    fakes_pass: Optional[bool]     # would land inside a given acceptance band
    wire_sigmas: dict = field(default_factory=dict)   # wire key -> signed sigmas
    caught_by: list = field(default_factory=list)     # wires beyond _SIGMA

    @property
    def silent(self) -> bool:
        """Invisible to the primary result — the envelope would not fire."""
        return not self.envelope_catches

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass
class SweepReport:
    objective_key: str
    objective_label: str
    unit: str
    nominal: float
    objective_unc: float                       # 1-sigma band on the objective
    findings: list = field(default_factory=list)
    wire_meta: dict = field(default_factory=dict)   # key -> {clean, band, ...}

    @property
    def silent_killers(self) -> list:
        return [f for f in self.findings if f.silent]

    @property
    def blind_spots(self) -> list:
        """Silent AND no catalogued wire sees them. Reported, never hidden."""
        return [f for f in self.findings if f.silent and not f.caught_by]


def run_sweep(objective: Objective, quantities: list[Quantity],
              mutations: Optional[list[Mutation]] = None,
              wires: Optional[list[Tripwire]] = None,
              pass_band: Optional[tuple] = None,
              today: Optional[_dt.date] = None) -> SweepReport:
    """
    The core loop. Deterministic: catalog order x quantity order, symmetric
    OAT bands, fixed 3-sigma threshold. Every number reproducible by hand.

    `pass_band` (lo, hi) on the objective, if given, additionally answers the
    scariest question: could this corruption hand the team a PASS? True only
    when the corrupted objective sits inside the acceptance band — i.e. the
    sealed contract itself would smile at the garbage.
    """
    mutations = mutations if mutations is not None else DEFAULT_MUTATIONS
    live_wires = available_tripwires(quantities, wires)

    clean_car = aggregate(quantities)
    rep = analyze_objective(objective, quantities, today)
    obj_band = rep.total_unc

    wire_meta: dict = {}
    for w in live_wires:
        clean = w.fn(clean_car)
        wire_meta[w.key] = {
            "label": w.label, "unit": w.unit, "how": w.how,
            "clean": clean, "band": wire_band(w, clean), "tol": w.tol,
        }

    findings: list[SabotageFinding] = []
    for m in mutations:
        for i, q in enumerate(quantities):
            if not m.applies(q):
                continue
            mutated = m.apply(quantities, i)
            bad_car = aggregate(mutated)
            f_bad = objective.fn(bad_car)
            d_obj = f_bad - rep.nominal
            sig = min(abs(d_obj) / obj_band, _SIGMA_CAP) if obj_band > 0 \
                else (_SIGMA_CAP if d_obj != 0.0 else 0.0)
            fakes = None
            if pass_band is not None:
                lo, hi = pass_band
                fakes = bool(lo <= f_bad <= hi)
            wire_sig: dict = {}
            caught: list = []
            for w in live_wires:
                meta = wire_meta[w.key]
                v_bad = w.fn(bad_car)
                if not (isinstance(v_bad, float) and math.isfinite(v_bad)):
                    # the corruption makes the wire unevaluable — that IS a
                    # detection (a NaN checksum never goes unnoticed)
                    wire_sig[w.key] = _SIGMA_CAP
                    caught.append(w.key)
                    continue
                band = meta["band"]
                s = (v_bad - meta["clean"]) / band if band > 0 else 0.0
                s = max(-_SIGMA_CAP, min(_SIGMA_CAP, s))
                wire_sig[w.key] = s
                if abs(s) >= _SIGMA:
                    caught.append(w.key)
            findings.append(SabotageFinding(
                mutation_key=m.key, mutation_label=m.label,
                target_key=q.key, target_label=q.label,
                delta_objective=d_obj, objective_sigmas=sig,
                envelope_catches=sig >= _SIGMA, fakes_pass=fakes,
                wire_sigmas=wire_sig, caught_by=caught))
    return SweepReport(objective.key, objective.label, objective.unit,
                       rep.nominal, obj_band, findings, wire_meta)


# --------------------------------------------------------------------------- #
#  Sheet selection — the smallest set of checks that catches the most
# --------------------------------------------------------------------------- #
def select_tripwires(report: SweepReport,
                     max_wires: Optional[int] = None) -> list[str]:
    """
    Greedy set cover over the SILENT killers (the envelope already owns the
    loud ones). Each round picks the wire that newly catches the most
    still-uncaught silent findings; ties break on (a) larger worst-case
    separation over its newly-caught set — a wire that barely clears 3 sigma
    is a worse sentry than one that clears 30 — then (b) catalog order.
    Deterministic.

    By default the cover runs until NOTHING CATCHABLE remains uncaught — a
    hard cap that quietly leaves a catchable killer uncovered would be the
    unearned green board wearing a new hat. `max_wires` exists for callers
    who accept a shorter sheet, and build_sheet charges any resulting
    uncovered-but-catchable finding to the blind-spot list so the truncation
    is visible, never silent.
    """
    silent = report.silent_killers
    uncaught = set(range(len(silent)))
    order = list(report.wire_meta.keys())
    chosen: list[str] = []
    while uncaught and (max_wires is None or len(chosen) < max_wires):
        best_key, best_new, best_sep = "", set(), -1.0
        for k in order:
            if k in chosen:
                continue
            new = {i for i in uncaught
                   if abs(silent[i].wire_sigmas.get(k, 0.0)) >= _SIGMA}
            if not new:
                continue
            sep = min(abs(silent[i].wire_sigmas[k]) for i in new)
            if (len(new), sep) > (len(best_new), best_sep):
                best_key, best_new, best_sep = k, new, sep
        if not best_key:
            break
        chosen.append(best_key)
        uncaught -= best_new
    return chosen


# --------------------------------------------------------------------------- #
#  The sealed pre-flight sheet
# --------------------------------------------------------------------------- #
@dataclass
class PreflightSheet:
    """
    The 3-6 checksum numbers to record with a run, sealed like a validation
    contract: wire set, clean values, and bands are fixed and sha256-hashed
    at creation. Judging real readings against an edited sheet is refused —
    a tripwire whose band moved after the run is not a tripwire.
    """
    id: str
    created_on: str
    author: str
    objective_key: str
    catalog_version: str
    wires: list                    # [{key,label,unit,clean,band,how}] in order
    coverage_before: float         # fraction of catalog caught by envelope only
    coverage_after: float          # ... caught by envelope + this sheet
    blind_spots: list              # [{mutation_label, target_label}] honest list
    signatures: dict               # finding id -> {wire key -> sigmas} for
                                   # fingerprinting silent killers later
    seal: str = ""

    _SEALED = ("id", "created_on", "author", "objective_key",
               "catalog_version", "wires", "coverage_before",
               "coverage_after", "blind_spots", "signatures")

    def _payload(self) -> str:
        d = asdict(self)
        return json.dumps({k: d[k] for k in self._SEALED},
                          sort_keys=True, separators=(",", ":"))

    def compute_seal(self) -> str:
        return hashlib.sha256(self._payload().encode()).hexdigest()

    def verify_seal(self) -> bool:
        return bool(self.seal) and self.seal == self.compute_seal()

    def as_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "PreflightSheet":
        valid = PreflightSheet.__dataclass_fields__.keys()
        return PreflightSheet(**{k: v for k, v in d.items() if k in valid})


def build_sheet(report: SweepReport, author: str = "",
                max_wires: Optional[int] = None,
                today: Optional[_dt.date] = None) -> PreflightSheet:
    """Select wires, compute honest coverage, seal. Deterministic."""
    chosen = select_tripwires(report, max_wires)
    n = len(report.findings)
    caught_env = sum(1 for f in report.findings if f.envelope_catches)
    caught_all = sum(1 for f in report.findings
                     if f.envelope_catches
                     or any(abs(f.wire_sigmas.get(k, 0.0)) >= _SIGMA
                            for k in chosen))
    blind = [{"mutation_label": f.mutation_label,
              "target_label": f.target_label}
             for f in report.findings
             if not f.envelope_catches
             and not any(abs(f.wire_sigmas.get(k, 0.0)) >= _SIGMA
                         for k in chosen)]
    sigs = {}
    for f in report.findings:
        if f.silent and any(abs(f.wire_sigmas.get(k, 0.0)) >= _SIGMA
                            for k in chosen):
            fid = f"{f.mutation_key}::{f.target_key}"
            sigs[fid] = {k: round(f.wire_sigmas.get(k, 0.0), 6)
                         for k in chosen}
    day = (today or _dt.date.today()).isoformat()
    sheet = PreflightSheet(
        id="pf_" + hashlib.sha256(
            (report.objective_key + day + ",".join(chosen)).encode()
        ).hexdigest()[:10],
        created_on=day, author=author,
        objective_key=report.objective_key,
        catalog_version=CATALOG_VERSION,
        wires=[{"key": k, **report.wire_meta[k]} for k in chosen],
        coverage_before=(caught_env / n) if n else 1.0,
        coverage_after=(caught_all / n) if n else 1.0,
        blind_spots=blind, signatures=sigs)
    sheet.seal = sheet.compute_seal()
    return sheet


# --------------------------------------------------------------------------- #
#  Fingerprinting — the garbage names itself
# --------------------------------------------------------------------------- #
@dataclass
class WireVerdict:
    status: str                    # "clean" | "tripped" | "incomplete"
    tripped: list                  # wire keys beyond _SIGMA
    deviations: dict               # wire key -> signed sigmas observed
    suspects: list                 # [{finding, cosine, magnitude_ratio}] ranked
    note: str = ""


def _cosine(a: dict, b: dict, keys: list) -> float:
    num = sum(a.get(k, 0.0) * b.get(k, 0.0) for k in keys)
    na = math.sqrt(sum(a.get(k, 0.0) ** 2 for k in keys))
    nb = math.sqrt(sum(b.get(k, 0.0) ** 2 for k in keys))
    return num / (na * nb) if na > 0 and nb > 0 else 0.0


def judge_readings(sheet: PreflightSheet, readings: dict,
                   match_threshold: float = 0.8) -> WireVerdict:
    """
    Compare the run's actual tripwire readings against the sealed sheet.

    CLEAN: every wire inside its 3-sigma band — the catalogued corruption
    classes are excluded (and only those; the note says so).
    TRIPPED: at least one wire outside its band. The observed deviation
    pattern is matched against every silent-killer signature the sweep
    predicted, by cosine similarity on band-normalised deviations, ranked
    with a magnitude sanity ratio. The top matches are named suspects — the
    audit starts at "check the accumulator mass for a pound-kg slip", not
    at "something is wrong somewhere".
    Refuses a broken seal: a sheet edited after the run judges nothing.
    """
    if not sheet.verify_seal():
        raise ValueError(
            "Pre-flight sheet seal is broken — a sealed field changed after "
            "creation. Re-build the sheet; a moved tripwire band cannot "
            "judge a run.")
    keys = [w["key"] for w in sheet.wires]
    missing = [k for k in keys if k not in readings]
    if missing:
        return WireVerdict("incomplete", [], {}, [],
                           note="Missing readings for: " + ", ".join(missing)
                                + ". A skipped tripwire is not a passed one.")
    dev: dict = {}
    tripped: list = []
    for w in sheet.wires:
        band = w["band"]
        d = ((float(readings[w["key"]]) - w["clean"]) / band) if band > 0 \
            else 0.0
        dev[w["key"]] = d
        if abs(d) >= _SIGMA:
            tripped.append(w["key"])
    if not tripped:
        return WireVerdict(
            "clean", [], dev, [],
            note="All tripwires inside their sealed bands. The catalogued "
                 "corruption classes are excluded for this run — errors "
                 "outside the catalog remain possible, as always.")
    obs_norm = math.sqrt(sum(v * v for v in dev.values()))
    suspects = []
    for fid, sig in sheet.signatures.items():
        c = _cosine(dev, sig, keys)
        if c < match_threshold:
            continue
        sig_norm = math.sqrt(sum(v * v for v in sig.values()))
        ratio = (obs_norm / sig_norm) if sig_norm > 0 else 0.0
        suspects.append({"finding": fid, "cosine": round(c, 4),
                         "magnitude_ratio": round(ratio, 4)})
    # closest direction first; among equals, magnitude closest to 1x; then
    # finding id — fully deterministic ranking
    suspects.sort(key=lambda s: (-s["cosine"],
                                 abs(math.log(s["magnitude_ratio"]))
                                 if s["magnitude_ratio"] > 0 else math.inf,
                                 s["finding"]))
    note = ("Tripwire(s) fired: " + ", ".join(tripped) + ". ")
    if suspects:
        top = suspects[0]
        note += (f"Deviation signature matches `{top['finding']}` "
                 f"(cosine {top['cosine']:.2f}, magnitude "
                 f"{top['magnitude_ratio']:.2f}x predicted). Start the "
                 "audit there.")
    else:
        note += ("No catalogued corruption matches this signature — the "
                 "error is real but outside the catalog. Audit units, "
                 "frame, BCs and geometry version in that order.")
    return WireVerdict("tripped", tripped, dev, suspects, note)


# --------------------------------------------------------------------------- #
#  Markdown export — the sheet that gets taped next to the ANSYS seat
# --------------------------------------------------------------------------- #
def render_preflight_md(sheet: PreflightSheet, report: SweepReport,
                        frame_note: str = "") -> str:
    L: list[str] = []
    L.append(f"# Pre-flight Sheet — {report.objective_label}")
    L.append("")
    L.append(f"*Sealed {sheet.created_on}"
             + (f" by {sheet.author}" if sheet.author else "")
             + f". Seal:* `{sheet.seal[:16]}…` *— verify in KinematiK "
               "before judging readings.*")
    if frame_note:
        L.append(f"*Coordinate convention:* {frame_note}")
    L.append("")
    L.append(f"KinematiK injected **{len(report.findings)}** catalogued "
             "corruptions into a shadow copy of this deck. "
             f"**{len(report.silent_killers)}** of them would have come back "
             "looking plausible — inside the result's own 3σ envelope. "
             "Record the numbers below with every run; they were chosen "
             "because they expose exactly those corruptions.")
    L.append("")
    L.append(f"**Detection coverage:** {sheet.coverage_before * 100:.0f}% "
             "(result envelope alone) → "
             f"**{sheet.coverage_after * 100:.0f}%** (envelope + this sheet)")
    L.append("")
    L.append("## Record these with the run")
    L.append("")
    L.append("| # | Checksum | Expected | Must land in | Where to read it |")
    L.append("|---|---|---|---|---|")
    for i, w in enumerate(sheet.wires, 1):
        lo = w["clean"] - _SIGMA * w["band"]
        hi = w["clean"] + _SIGMA * w["band"]
        L.append(f"| {i} | {w['label']} | {w['clean']:.4g} {w['unit']} | "
                 f"[{lo:.4g}, {hi:.4g}] (±{w.get('tol', 0.0) * 100:.0f}%) | "
                 f"{w['how']} |")
    L.append("")
    if sheet.blind_spots:
        L.append("## Honest blind spots")
        L.append("")
        L.append("These catalogued corruptions are invisible to the result "
                 "AND to every available tripwire. The only defence is "
                 "measuring the input directly:")
        L.append("")
        for b in sheet.blind_spots:
            L.append(f"- **{b['mutation_label']}** striking "
                     f"*{b['target_label']}*")
        L.append("")
    L.append("_A reading outside its band means the run and the deck "
             "disagree — enter the readings in the Saboteur tab and the "
             "deviation pattern will name the most likely corruption class. "
             "This sheet covers the catalogued error classes only; it is a "
             "tripwire net, not a certificate._")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  Self-test (python3 -m suspension.saboteur)
# --------------------------------------------------------------------------- #
if __name__ == "__main__":  # pragma: no cover
    from .interfaces import SubsystemInterface, blank_ledger
    from .proof_engine import DEFAULT_OBJECTIVES, build_uncertainty_ledger
    led = blank_ledger()
    led.set(SubsystemInterface(name="chassis", mass_kg=45, cg_z_mm=300,
                               is_estimate=True))
    led.set(SubsystemInterface(name="powertrain", mass_kg=60, cg_z_mm=320,
                               peak_power_kw=68, peak_torque_nm=180,
                               heat_reject_w=3200, is_estimate=True))
    led.set(SubsystemInterface(name="accumulator", mass_kg=55,
                               peak_current_a=180, is_estimate=True))
    led.set(SubsystemInterface(name="cooling", mass_kg=6,
                               cooling_airflow_cms=0.14, is_estimate=True))
    qs = build_uncertainty_ledger(led)
    obj = DEFAULT_OBJECTIVES[1]
    rep = run_sweep(obj, qs)
    sheet = build_sheet(rep)
    print(render_preflight_md(sheet, rep))
