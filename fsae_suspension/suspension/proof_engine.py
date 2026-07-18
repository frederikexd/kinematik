# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  Module: proof_engine — the Burden-of-Proof planner
# ============================================================================
"""
Proof Engine — "KinematiK gets you to the right question" made literal.

THE PROBLEM THIS SOLVES
-----------------------
Every FSAE-EV team faces the same three failures in the week before they open
ANSYS / ADAMS / SolidWorks Simulation:

  1. THEY VALIDATE THE WRONG THING FIRST. The upright gets an 8-hour FEA
     campaign because FEA is what the team knows how to do — while the lap-time
     prediction the whole season is being tuned against is ±2 s, dominated by a
     CG height nobody has ever measured. Solver hours go where the skills are,
     not where the uncertainty is.

  2. THEY DECIDE WHAT "PASS" MEANS *AFTER* SEEING THE RESULT. The sim says
     FoS = 1.05, and 1.05 is retroactively declared fine, because the part is
     already drawn. Experimental science solved this decades ago with
     pre-registration: state the acceptance criterion BEFORE the run, in
     writing, sealed. No engineering tool does this. This one does.

  3. THEY CAN'T TELL A FAILED DESIGN FROM A GARBAGE RUN. A sim result that
     contradicts everything upstream is treated the same as one that lands
     slightly out of band. But a result outside the *plausibility envelope*
     implied by the ledger means the sim and the ledger disagree about reality
     — someone's units, frame, or boundary conditions are wrong — and acting on
     either number before finding out which is how garbage propagates.

WHAT IT DOES
------------
  * Builds a QUANTIFIED uncertainty ledger from the IntegrationLedger: every
    declared channel gets a ± band derived from an evidence grade
    (guess / estimate / modelled / measured / verified) that INFLATES WITH AGE
    (staleness decay — last season's corner-weighing is not this season's).
  * Propagates those bands through documented objective models (lap time,
    endurance energy, cooling margin, mass roll-up) using deterministic
    one-at-a-time perturbation — no Monte Carlo, no randomness, every number
    reproducible and auditable by hand, matching the release-gate ethos.
  * Attributes the objective's uncertainty to its inputs ("61 % of your ±1.9 s
    lap-time band is CG height, and CG height is a judgement call").
  * Ranks a catalog of EVIDENCE ACTIONS — corner scales, tilt test, coast-down,
    dyno pull, flow bench, TTC fit, torsion rig, an ANSYS study — by
    uncertainty retired PER HOUR of effort on the objective the team chose.
    This is value-of-information planning: the output is an ordered proof plan,
    i.e. the literal list of questions worth asking the expensive tools.
  * Writes a PRE-REGISTERED VALIDATION CONTRACT for any planned action: the
    prediction, the acceptance band, and the 3-sigma plausibility envelope are
    fixed and SEALED (sha256 over the canonical payload) before the run
    happens. Judging a returned result gives one of three verdicts:
        PASS        inside the acceptance band → the channel's grade upgrades
        FAIL        outside the band but physically plausible → design problem
        DISCREPANT  outside the plausibility envelope → the run and the ledger
                    disagree about reality; audit inputs before trusting either
    Editing the band after sealing breaks the seal, and the broken seal is
    reported out loud. Goalposts cannot silently move.

HONESTY CONTRACT (same as the rest of KinematiK)
------------------------------------------------
The objective models here are documented closed-form surrogates — every
sensitivity names its mechanism and its source in the docstring, and results
carry the "coupled" confidence class, never "measured". Where KinematiK owns a
real solver (mass roll-up), the objective is exact and says so. The engine
accepts injected objective functions, so the UI can substitute the real lap sim
where it is configured; the surrogates exist so the planner works on day one of
a season, with nothing but the ledger.

No streamlit / pandas / plotly imports. Pure stdlib. Unit-testable headless.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Callable, Optional

from .interfaces import IntegrationLedger


# --------------------------------------------------------------------------- #
#  Evidence grades — how good is the number, really?
# --------------------------------------------------------------------------- #
class EvidenceGrade(str, Enum):
    """
    The five honest answers to "where did this number come from?", each with a
    default relative uncertainty. The defaults are deliberately conservative
    round numbers a design judge can interrogate, not false precision:

      GUESS     ±40 %  someone said a number in a meeting
      ESTIMATE  ±20 %  scaled from last year's car / a reference design
      MODELLED  ±10 %  a CAD mass rollup, a closed-form calc, a KinematiK solver
      MEASURED   ±3 %  an instrument touched the real hardware
      VERIFIED   ±1 %  measured twice, independently (two scales, two operators)
    """
    GUESS = "guess"
    ESTIMATE = "estimate"
    MODELLED = "modelled"
    MEASURED = "measured"
    VERIFIED = "verified"

    @property
    def base_rel_unc(self) -> float:
        return _GRADE_UNC[self.value]

    @property
    def rank(self) -> int:
        return _GRADE_RANK[self.value]

    @property
    def label(self) -> str:
        return _GRADE_LABEL[self.value]


_GRADE_UNC = {
    "guess": 0.40, "estimate": 0.20, "modelled": 0.10,
    "measured": 0.03, "verified": 0.01,
}
_GRADE_RANK = {"guess": 0, "estimate": 1, "modelled": 2,
               "measured": 3, "verified": 4}
_GRADE_LABEL = {
    "guess": "guess (someone said it in a meeting)",
    "estimate": "estimate (scaled from a reference)",
    "modelled": "modelled (CAD / closed-form / solver)",
    "measured": "measured (instrument on real hardware)",
    "verified": "verified (independently measured twice)",
}

# Staleness half-life, days: after this many days the grade's uncertainty has
# doubled. A guess doesn't rot (it's already maximally uncertain); a
# measurement rots fastest in relative terms because the car it was taken on
# keeps changing under it. Uncertainty never inflates past the GUESS floor.
_STALENESS_HALFLIFE_D = {
    "guess": math.inf, "estimate": 365.0, "modelled": 270.0,
    "measured": 180.0, "verified": 180.0,
}


def effective_rel_unc(grade: EvidenceGrade, age_days: float = 0.0) -> float:
    """Grade's base uncertainty, inflated linearly with age (doubling at the
    grade's half-life), capped at the GUESS ceiling. Deterministic."""
    base = grade.base_rel_unc
    hl = _STALENESS_HALFLIFE_D[grade.value]
    if not math.isfinite(hl) or age_days <= 0:
        return base
    return min(base * (1.0 + age_days / hl), _GRADE_UNC["guess"])


# --------------------------------------------------------------------------- #
#  A quantity with a quantified pedigree
# --------------------------------------------------------------------------- #
@dataclass
class Quantity:
    """One channel of one subsystem, with its value AND its pedigree."""
    key: str                 # "chassis.mass_kg"
    subsystem: str
    channel: str             # "mass_kg"
    label: str
    value: float
    unit: str
    grade: EvidenceGrade = EvidenceGrade.ESTIMATE
    measured_on: str = ""    # ISO date the evidence was produced ("" = unknown)
    source: str = ""         # free text: "corner scales 2026-05-02", "guess"

    def age_days(self, today: Optional[_dt.date] = None) -> float:
        if not self.measured_on:
            return 0.0
        try:
            d = _dt.date.fromisoformat(self.measured_on[:10])
        except ValueError:
            return 0.0
        return max(0.0, ((today or _dt.date.today()) - d).days)

    def rel_unc(self, today: Optional[_dt.date] = None) -> float:
        return effective_rel_unc(self.grade, self.age_days(today))

    def abs_unc(self, today: Optional[_dt.date] = None) -> float:
        # A zero value still deserves a nonzero band; fall back to unit-scale 1.
        scale = abs(self.value) if self.value else 1.0
        return scale * self.rel_unc(today)

    def as_dict(self) -> dict:
        d = asdict(self)
        d["grade"] = self.grade.value
        return d

    @staticmethod
    def from_dict(d: dict) -> "Quantity":
        d = dict(d)
        d["grade"] = EvidenceGrade(d.get("grade", "estimate"))
        valid = Quantity.__dataclass_fields__.keys()
        return Quantity(**{k: v for k, v in d.items() if k in valid})


# Channels the engine reasons about, with labels/units. Anything else declared
# in the ledger is carried but has no objective sensitivity (superset-safe,
# same policy as risk_propagation.CHANNEL_LABELS).
_CHANNEL_META = {
    "mass_kg":            ("mass", "kg"),
    "cg_z_mm":            ("CG height", "mm"),
    "cg_x_mm":            ("CG longitudinal position", "mm"),
    "peak_torque_nm":     ("peak motor torque", "N·m"),
    "peak_power_kw":      ("peak power", "kW"),
    "heat_reject_w":      ("heat rejected", "W"),
    "cooling_airflow_cms": ("cooling airflow", "m³/s"),
    "mount_load_n":       ("peak mount load", "N"),
    "brake_torque_nm":    ("brake torque", "N·m"),
    "power_draw_w":       ("continuous power draw", "W"),
    "peak_current_a":     ("peak current", "A"),
}


def build_uncertainty_ledger(ledger: IntegrationLedger,
                             overrides: Optional[dict] = None) -> list[Quantity]:
    """
    Turn the IntegrationLedger into a list of Quantities with pedigrees.

    Seeding rule: `is_estimate=True` seeds ESTIMATE, `False` seeds MODELLED —
    the binary flag the ledger already carries maps onto the two middle grades,
    and never onto MEASURED, because a checkbox is not an instrument. Anything
    better than MODELLED must be claimed explicitly (via `overrides` — the
    persisted pedigree map the UI maintains, keyed by Quantity.key).
    """
    overrides = overrides or {}
    out: list[Quantity] = []
    for iface in ledger.interfaces.values():
        for ch, val in iface.numeric_values().items():
            if not isinstance(val, (int, float)):
                continue                      # tuples (downforce@v) skipped here
            lab, unit = _CHANNEL_META.get(ch, (ch.replace("_", " "), ""))
            key = f"{iface.name}.{ch}"
            q = Quantity(
                key=key, subsystem=iface.name, channel=ch,
                label=f"{iface.name} {lab}", value=float(val), unit=unit,
                grade=(EvidenceGrade.ESTIMATE if iface.is_estimate
                       else EvidenceGrade.MODELLED),
                measured_on=iface.updated_on[:10] if iface.updated_on else "",
                source=("declared as estimate" if iface.is_estimate
                        else "declared (not an estimate)"),
            )
            ov = overrides.get(key)
            if ov:
                q = Quantity.from_dict({**q.as_dict(), **ov})
            out.append(q)
    return out


# --------------------------------------------------------------------------- #
#  Objectives — the top-level numbers the season is actually judged on
# --------------------------------------------------------------------------- #
# An objective consumes AGGREGATED car-level channels (summed mass, mass-
# weighted CG, summed heat, min airflow, max torque...) so per-subsystem
# quantities roll up before evaluation.

_SUM_CHANNELS = {"mass_kg", "heat_reject_w", "power_draw_w"}
_MAX_CHANNELS = {"peak_torque_nm", "peak_power_kw", "peak_current_a",
                 "mount_load_n", "brake_torque_nm"}
_MIN_CHANNELS = {"cooling_airflow_cms"}
_MASS_WEIGHTED = {"cg_z_mm", "cg_x_mm"}


def aggregate(quantities: list[Quantity]) -> dict:
    """Car-level channel values from subsystem quantities. Deterministic."""
    car: dict = {}
    masses = {q.subsystem: q.value for q in quantities if q.channel == "mass_kg"}
    total_mass = sum(masses.values())
    for ch in _SUM_CHANNELS | _MAX_CHANNELS | _MIN_CHANNELS | _MASS_WEIGHTED:
        vals = [(q.subsystem, q.value) for q in quantities if q.channel == ch]
        if not vals:
            continue
        if ch in _SUM_CHANNELS:
            car[ch] = sum(v for _, v in vals)
        elif ch in _MAX_CHANNELS:
            car[ch] = max(v for _, v in vals)
        elif ch in _MIN_CHANNELS:
            car[ch] = min(v for _, v in vals)
        else:  # mass-weighted mean; falls back to plain mean without masses
            if total_mass > 0 and any(s in masses for s, _ in vals):
                num = sum(masses.get(s, 0.0) * v for s, v in vals)
                den = sum(masses.get(s, 0.0) for s, _ in vals)
                car[ch] = num / den if den > 0 else sum(v for _, v in vals) / len(vals)
            else:
                car[ch] = sum(v for _, v in vals) / len(vals)
    return car


@dataclass
class Objective:
    """
    A top-level number with a documented evaluator. `fn` maps the aggregated
    car dict to a scalar; `confidence` states honestly what class of model the
    evaluator is (roll-up = exact, surrogate = coupled). `better` says which
    direction is good, so the plan can phrase risk correctly.
    """
    key: str
    label: str
    unit: str
    fn: Callable[[dict], float]
    confidence: str            # "exact" | "coupled"
    better: str                # "lower" | "higher"
    doc: str = ""


# ---- default objective surrogates (documented, injectable, replaceable) ---- #

def _obj_total_mass(car: dict) -> float:
    """Exact roll-up: the sum of declared subsystem masses."""
    return car.get("mass_kg", 0.0)


def _obj_laptime(car: dict) -> float:
    """
    Endurance-lap surrogate around a 75 s FSAE reference lap at 230 kg all-up.
    Mechanisms and sensitivities (all named, all challengeable):
      * mass:      +0.030 s/lap per kg — the number KinematiK's own coupling
                   graph already uses for the chassis-mass edge.
      * CG height: lateral load transfer ∝ h; grip loss ≈ 0.5·ΔW/W of lateral
                   capacity; at ~40 % of the lap cornering that lands near
                   +0.011 s/lap per mm above the 280 mm reference.
      * power:     laps are traction/corner-limited below ~60 kW but power
                   still buys straights: −0.15 s/lap per kW above 60, saturated.
    This is a COUPLED surrogate for planning, not a lap sim; the UI swaps in
    the real lap sim where one is configured.
    """
    t = 75.0
    t += 0.030 * (car.get("mass_kg", 230.0) - 230.0)
    t += 0.011 * (car.get("cg_z_mm", 280.0) - 280.0)
    p = car.get("peak_power_kw", 60.0)
    t -= 0.15 * max(min(p, 80.0) - 60.0, -20.0)
    return t


def _obj_endurance_energy(car: dict) -> float:
    """
    Endurance energy surrogate, kWh for a 22 km FSAE endurance:
      * baseline 6.0 kWh at 230 kg (rolling + aero + accel cycles),
      * +0.012 kWh per kg (rolling resistance + kinetic energy per stop-start),
      * continuous LV draw over ~25 min adds power_draw_w · 25/60 /1000.
    COUPLED surrogate; the EV tab's energy budget replaces it when present.
    """
    e = 6.0 + 0.012 * (car.get("mass_kg", 230.0) - 230.0)
    e += car.get("power_draw_w", 0.0) * (25.0 / 60.0) / 1000.0
    return e


def _obj_cooling_margin(car: dict) -> float:
    """
    Cooling margin surrogate, °C below the 60 °C pack ceiling:
    air-side capacity ≈ ρ·cp·V̇·ΔT_air with ρcp ≈ 1200 J/(m³·K) and 20 °C
    allowable air rise ⇒ capacity_W ≈ 24 000 · airflow. Pack temperature rise
    over ambient ≈ 25 · (heat / capacity) °C. Margin = 60 − 25 − rise.
    COUPLED surrogate; pack_thermal's transient result replaces it when run.
    """
    q = car.get("heat_reject_w", 0.0)
    v = max(car.get("cooling_airflow_cms", 0.05), 1e-6)
    cap = 24000.0 * v
    rise = 25.0 * (q / cap) if cap > 0 else 999.0
    return 60.0 - 25.0 - rise


DEFAULT_OBJECTIVES: list[Objective] = [
    Objective("total_mass_kg", "Total car mass", "kg", _obj_total_mass,
              "exact", "lower", _obj_total_mass.__doc__ or ""),
    Objective("laptime_s", "Endurance lap time", "s", _obj_laptime,
              "coupled", "lower", _obj_laptime.__doc__ or ""),
    Objective("endurance_kwh", "Endurance energy", "kWh", _obj_endurance_energy,
              "coupled", "lower", _obj_endurance_energy.__doc__ or ""),
    Objective("cooling_margin_c", "Pack thermal margin", "°C", _obj_cooling_margin,
              "coupled", "higher", _obj_cooling_margin.__doc__ or ""),
]


# --------------------------------------------------------------------------- #
#  Uncertainty propagation & attribution — deterministic, auditable
# --------------------------------------------------------------------------- #
@dataclass
class Attribution:
    """One input's contribution to one objective's uncertainty band."""
    quantity_key: str
    label: str
    grade: str
    input_unc: float           # ± on the input, in its own unit
    delta_out: float           # ± it alone induces on the objective
    share: float               # fraction of total variance [0..1]


@dataclass
class UncertaintyReport:
    objective_key: str
    objective_label: str
    unit: str
    nominal: float
    total_unc: float           # ± (1-sigma-equivalent, root-sum-square)
    confidence: str            # evaluator class, honestly restated
    attributions: list = field(default_factory=list)   # sorted, largest first


def analyze_objective(objective: Objective, quantities: list[Quantity],
                      today: Optional[_dt.date] = None) -> UncertaintyReport:
    """
    One-at-a-time symmetric perturbation: each quantity is pushed to
    value ± its uncertainty, everything re-aggregated, the objective
    re-evaluated, and the induced half-spread recorded. Contributions combine
    root-sum-square (independence linearization). Same inputs ⇒ same report,
    always — every attribution can be reproduced by hand with a calculator,
    which is the whole point.
    """
    base_car = aggregate(quantities)
    nominal = objective.fn(base_car)
    attrs: list[Attribution] = []
    for i, q in enumerate(quantities):
        u = q.abs_unc(today)
        if u == 0.0:
            continue
        hi = [Quantity.from_dict(x.as_dict()) for x in quantities]
        lo = [Quantity.from_dict(x.as_dict()) for x in quantities]
        hi[i].value = q.value + u
        lo[i].value = q.value - u
        f_hi = objective.fn(aggregate(hi))
        f_lo = objective.fn(aggregate(lo))
        d = abs(f_hi - f_lo) / 2.0
        if d > 0.0:
            attrs.append(Attribution(q.key, q.label, q.grade.value, u, d, 0.0))
    total_var = sum(a.delta_out ** 2 for a in attrs)
    total = math.sqrt(total_var)
    for a in attrs:
        a.share = (a.delta_out ** 2 / total_var) if total_var > 0 else 0.0
    attrs.sort(key=lambda a: a.delta_out, reverse=True)
    return UncertaintyReport(objective.key, objective.label, objective.unit,
                             nominal, total, objective.confidence, attrs)


# --------------------------------------------------------------------------- #
#  Evidence actions — the catalog of ways to buy certainty
# --------------------------------------------------------------------------- #
@dataclass
class EvidenceAction:
    """
    A concrete thing a team can do to upgrade a channel's evidence grade.
    `channels` are channel names it improves (any subsystem); `hours` is honest
    student-hours including setup; `brief` is the how-to a lead can hand over.
    """
    key: str
    label: str
    tool: str                  # "workshop" | "track" | "rig" | "ANSYS" | ...
    hours: float
    channels: list
    resulting_grade: EvidenceGrade
    brief: str = ""


DEFAULT_ACTIONS: list[EvidenceAction] = [
    EvidenceAction(
        "corner_scales", "Corner-scale the car (or subassembly)", "workshop",
        2.0, ["mass_kg"], EvidenceGrade.MEASURED,
        "Four corner scales, level floor, driver ballast in seat. Record per-"
        "corner and total; two operators re-zero and repeat for VERIFIED."),
    EvidenceAction(
        "tilt_cg", "CG height by axle-lift / tilt method", "workshop",
        4.0, ["cg_z_mm"], EvidenceGrade.MEASURED,
        "Raise one axle a known height on scales, record weight transfer, "
        "solve h from ΔW·wheelbase/(W·tanθ). Repeat at two angles."),
    EvidenceAction(
        "cad_rollup", "CAD mass-properties rollup with densities audited",
        "CAD", 3.0, ["mass_kg", "cg_z_mm", "cg_x_mm"], EvidenceGrade.MODELLED,
        "Assign real material densities to every body, include fasteners and "
        "fluids, export mass properties from the master assembly."),
    EvidenceAction(
        "coastdown", "Coast-down test for drag + rolling resistance", "track",
        5.0, ["power_draw_w"], EvidenceGrade.MEASURED,
        "Two directions, calm air, log speed-vs-time from 60 km/h; fit "
        "F = A + B·v². Feeds the energy budget the pack is sized on."),
    EvidenceAction(
        "dyno_pull", "Motor dyno / rolling-road pull", "rig",
        6.0, ["peak_torque_nm", "peak_power_kw"], EvidenceGrade.MEASURED,
        "Torque and power vs rpm at competition voltage and current limits; "
        "log inverter derates. The driveline check inherits the real number."),
    EvidenceAction(
        "pack_thermal_log", "Instrumented pack discharge (thermal)", "rig",
        8.0, ["heat_reject_w"], EvidenceGrade.MEASURED,
        "Endurance-profile discharge with per-segment thermocouples; heat "
        "rejected from ΔT and coolant/air flow. BMS log is the record."),
    EvidenceAction(
        "flow_bench", "Duct flow measurement (anemometer / flow bench)", "rig",
        4.0, ["cooling_airflow_cms"], EvidenceGrade.MEASURED,
        "Measure actual duct throughput at fan voltage / representative speed; "
        "compare against the CFD or vendor-curve assumption."),
    EvidenceAction(
        "strain_mount", "Strain-gauge a mount in a static load test", "rig",
        10.0, ["mount_load_n"], EvidenceGrade.MEASURED,
        "Instrument the mount, apply the design load case through a lever/"
        "jack, record strain vs load to the design point."),
    EvidenceAction(
        "ansys_static", "ANSYS static structural study (from a sealed brief)",
        "ANSYS", 8.0, ["mount_load_n", "brake_torque_nm"],
        EvidenceGrade.MODELLED,
        "Run exactly the sealed contract: geometry version from the Registry, "
        "loads and frame from the ledger, mesh convergence stated. The pre-"
        "registered band decides pass/fail — not the mood in the room."),
    EvidenceAction(
        "brake_dyno", "Brake torque measurement (decel test / dyno)", "track",
        5.0, ["brake_torque_nm"], EvidenceGrade.MEASURED,
        "Instrumented stops from 60 km/h; back out per-corner torque from "
        "decel and known mass (which is why corner scales come first)."),
    EvidenceAction(
        "current_log", "Log peak current at competition limits", "track",
        3.0, ["peak_current_a"], EvidenceGrade.MEASURED,
        "DAQ the DC bus through accel + one hot lap at the derate settings "
        "you will actually race with."),
]


@dataclass
class PlanItem:
    action_key: str
    action_label: str
    tool: str
    hours: float
    affected: list                      # quantity keys it upgrades
    unc_before: float
    unc_after: float
    unc_retired: float                  # objective units
    value_per_hour: float               # unc_retired / hours
    note: str = ""


@dataclass
class ProofPlan:
    objective_key: str
    objective_label: str
    unit: str
    nominal: float
    unc_now: float
    unc_floor: float                    # if EVERY listed action were done
    items: list = field(default_factory=list)     # ranked, best value first


def plan_proofs(objective: Objective, quantities: list[Quantity],
                actions: Optional[list[EvidenceAction]] = None,
                today: Optional[_dt.date] = None) -> ProofPlan:
    """
    Value-of-information ranking. For each action: clone the quantity set,
    upgrade every channel the action covers to the action's resulting grade
    (fresh, so staleness resets), re-run analyze_objective, and record the
    uncertainty retired per hour. An action that would DOWNGRADE a channel
    (re-modelling something already measured) contributes nothing for that
    channel and is ranked honestly low — the engine never recommends
    re-proving what is already better proven.
    """
    actions = actions if actions is not None else DEFAULT_ACTIONS
    base = analyze_objective(objective, quantities, today)
    items: list[PlanItem] = []
    floor_qs = [Quantity.from_dict(q.as_dict()) for q in quantities]

    for act in actions:
        after_qs = []
        touched: list[str] = []
        for q in quantities:
            q2 = Quantity.from_dict(q.as_dict())
            if q.channel in act.channels and act.resulting_grade.rank > q.grade.rank:
                q2.grade = act.resulting_grade
                q2.measured_on = (today or _dt.date.today()).isoformat()
                q2.source = act.label
                touched.append(q.key)
            after_qs.append(q2)
        if not touched:
            continue
        after = analyze_objective(objective, after_qs, today)
        retired = max(base.total_unc - after.total_unc, 0.0)
        items.append(PlanItem(
            act.key, act.label, act.tool, act.hours, touched,
            base.total_unc, after.total_unc, retired,
            retired / act.hours if act.hours > 0 else 0.0,
            note=("upgrades " + ", ".join(touched))))
        # accumulate the floor
        for i, q in enumerate(floor_qs):
            if q.channel in act.channels and act.resulting_grade.rank > q.grade.rank:
                floor_qs[i].grade = act.resulting_grade
                floor_qs[i].measured_on = (today or _dt.date.today()).isoformat()

    floor = analyze_objective(objective, floor_qs, today)
    items.sort(key=lambda it: it.value_per_hour, reverse=True)
    return ProofPlan(objective.key, objective.label, objective.unit,
                     base.nominal, base.total_unc, floor.total_unc, items)


# --------------------------------------------------------------------------- #
#  Pre-registered Validation Contracts — sealed before the run
# --------------------------------------------------------------------------- #
class Verdict(str, Enum):
    OPEN = "open"
    PASS = "pass"
    FAIL = "fail"
    DISCREPANT = "discrepant"


_PLAUSIBILITY_SIGMA = 3.0   # outside prediction ± 3·unc ⇒ the run and the
                            # ledger disagree about reality


@dataclass
class ValidationContract:
    """
    The pre-registration record. Everything above `seal` is fixed at creation
    and hashed; `seal` is sha256 over the canonical JSON of those fields.
    Judging never mutates sealed fields — it only fills the result block — so
    verify_seal() passing after judgment proves the goalposts never moved.
    """
    id: str
    created_on: str
    author: str
    action_key: str            # which evidence action / sim this contracts
    quantity_key: str          # the channel under test ("" for objective-level)
    title: str
    predicted: float           # the ledger's current answer
    predicted_unc: float       # ± on that answer, from the uncertainty ledger
    unit: str
    pass_lo: float             # acceptance band — the requirement, stated NOW
    pass_hi: float
    plaus_lo: float            # plausibility envelope = predicted ± 3σ
    plaus_hi: float
    criterion_note: str        # WHY this band (the design judge's question)
    seal: str = ""
    # ---- result block (filled by judge_result, never sealed) -------------- #
    status: str = Verdict.OPEN.value
    result_value: Optional[float] = None
    judged_on: str = ""
    judgment_note: str = ""

    _SEALED = ("id", "created_on", "author", "action_key", "quantity_key",
               "title", "predicted", "predicted_unc", "unit",
               "pass_lo", "pass_hi", "plaus_lo", "plaus_hi", "criterion_note")

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
    def from_dict(d: dict) -> "ValidationContract":
        valid = ValidationContract.__dataclass_fields__.keys()
        return ValidationContract(**{k: v for k, v in d.items() if k in valid})


def create_contract(action: EvidenceAction, quantity: Quantity,
                    pass_lo: float, pass_hi: float,
                    criterion_note: str, author: str = "",
                    today: Optional[_dt.date] = None,
                    contract_id: str = "") -> ValidationContract:
    """
    Seal a contract BEFORE the run. The plausibility envelope is computed from
    the quantity's current value and uncertainty — the team does not choose it,
    which is what makes DISCREPANT an honest verdict rather than an excuse.
    Raises ValueError on an inverted band, because a contract that can't be
    failed isn't a contract.
    """
    if pass_hi < pass_lo:
        raise ValueError("Acceptance band is inverted (pass_hi < pass_lo). "
                         "A contract that cannot be failed is not a contract.")
    u = quantity.abs_unc(today)
    c = ValidationContract(
        id=contract_id or f"vc_{hashlib.sha256((quantity.key + (today or _dt.date.today()).isoformat() + criterion_note).encode()).hexdigest()[:10]}",
        created_on=(today or _dt.date.today()).isoformat(),
        author=author,
        action_key=action.key,
        quantity_key=quantity.key,
        title=f"{action.label} — {quantity.label}",
        predicted=quantity.value,
        predicted_unc=u,
        unit=quantity.unit,
        pass_lo=float(pass_lo), pass_hi=float(pass_hi),
        plaus_lo=quantity.value - _PLAUSIBILITY_SIGMA * u,
        plaus_hi=quantity.value + _PLAUSIBILITY_SIGMA * u,
        criterion_note=criterion_note,
    )
    c.seal = c.compute_seal()
    return c


def judge_result(contract: ValidationContract, measured: float,
                 today: Optional[_dt.date] = None) -> ValidationContract:
    """
    Three-way verdict, checked in this order:

      broken seal → refuse (ValueError). If the band was edited after sealing,
                    no verdict is honest; the tamper is the finding.
      DISCREPANT  → outside the plausibility envelope: the run and the ledger
                    disagree by more than 3σ. Do not act on either number.
                    Audit: units, coordinate frame, boundary conditions,
                    geometry version, load magnitude — in that order (the
                    Frames tab exists because frame flips top this list).
      PASS        → inside the acceptance band.
      FAIL        → plausible but out of band: a real design problem, found
                    before manufacturing. That is the tool working.
    """
    if not contract.verify_seal():
        raise ValueError(
            "Contract seal is broken — a sealed field changed after creation. "
            "Re-create the contract; a moved goalpost cannot be judged.")
    c = ValidationContract.from_dict(contract.as_dict())
    c.result_value = float(measured)
    c.judged_on = (today or _dt.date.today()).isoformat()
    if not (c.plaus_lo <= measured <= c.plaus_hi):
        c.status = Verdict.DISCREPANT.value
        c.judgment_note = (
            f"Result {measured:g} {c.unit} is outside the plausibility "
            f"envelope [{c.plaus_lo:g}, {c.plaus_hi:g}] implied by the ledger "
            f"({c.predicted:g} ± {_PLAUSIBILITY_SIGMA:g}×{c.predicted_unc:g}). "
            "The run and the ledger disagree about reality — one of them has "
            "wrong units, frame, BCs, or geometry. Audit before trusting either.")
    elif c.pass_lo <= measured <= c.pass_hi:
        c.status = Verdict.PASS.value
        c.judgment_note = (
            f"Result {measured:g} {c.unit} inside the pre-registered band "
            f"[{c.pass_lo:g}, {c.pass_hi:g}]. Upgrade the channel's evidence "
            "grade with this result as the source.")
    else:
        c.status = Verdict.FAIL.value
        c.judgment_note = (
            f"Result {measured:g} {c.unit} is plausible but outside the "
            f"pre-registered band [{c.pass_lo:g}, {c.pass_hi:g}]. This is a "
            "design finding, not a bad run — it was caught before the first cut.")
    return c


# --------------------------------------------------------------------------- #
#  Markdown exports — the handover artifacts
# --------------------------------------------------------------------------- #
def render_proof_plan_md(plan: ProofPlan, report: UncertaintyReport,
                         frame_note: str = "") -> str:
    """The one-page proof plan a lead pins in the channel."""
    L: list[str] = []
    L.append(f"# Proof Plan — {plan.objective_label}")
    L.append("")
    L.append(f"**Current answer:** {plan.nominal:.3g} ± {plan.unc_now:.3g} "
             f"{plan.unit}  (evaluator class: {report.confidence})")
    L.append(f"**Floor if every action below is done:** ± {plan.unc_floor:.3g} "
             f"{plan.unit}")
    if frame_note:
        L.append(f"**Coordinate convention:** {frame_note}")
    L.append("")
    L.append("## Where the uncertainty comes from")
    L.append("")
    L.append("| Input | Grade | ± (input) | ± ({u}) | Share |"
             .format(u=plan.unit))
    L.append("|---|---|---|---|---|")
    for a in report.attributions[:12]:
        L.append(f"| {a.label} | {a.grade} | {a.input_unc:.3g} | "
                 f"{a.delta_out:.3g} | {a.share * 100:.0f}% |")
    L.append("")
    L.append("## What to prove next (ranked by certainty bought per hour)")
    L.append("")
    L.append("| # | Action | Tool | Hours | ± retired ({u}) | per hour |"
             .format(u=plan.unit))
    L.append("|---|---|---|---|---|---|")
    for i, it in enumerate(plan.items, 1):
        L.append(f"| {i} | {it.action_label} | {it.tool} | {it.hours:g} | "
                 f"{it.unc_retired:.3g} | {it.value_per_hour:.3g} |")
    L.append("")
    L.append("_Every number above is deterministic: same ledger in, same plan "
             "out. Attributions are one-at-a-time perturbations you can "
             "reproduce by hand._")
    return "\n".join(L)


def render_contract_brief_md(contract: ValidationContract,
                             action: Optional[EvidenceAction] = None,
                             frame_note: str = "") -> str:
    """
    The sealed brief handed to whoever runs the sim or test. It states the
    acceptance band, the plausibility envelope, and the seal hash — so the
    person at the ANSYS seat knows the criterion was fixed before they hit
    Solve, and everyone else can verify it stayed fixed after.
    """
    c = contract
    L: list[str] = []
    L.append(f"# Validation Contract — {c.title}")
    L.append("")
    L.append(f"*Sealed {c.created_on}"
             + (f" by {c.author}" if c.author else "") + ".*")
    L.append(f"*Seal:* `{c.seal[:16]}…` — verify in KinematiK before judging.")
    if frame_note:
        L.append(f"*Coordinate convention:* {frame_note}")
    L.append("")
    L.append(f"**Channel under test:** `{c.quantity_key}`")
    L.append(f"**Ledger prediction:** {c.predicted:g} ± {c.predicted_unc:g} "
             f"{c.unit}")
    L.append(f"**Pre-registered acceptance band:** [{c.pass_lo:g}, "
             f"{c.pass_hi:g}] {c.unit}")
    L.append(f"**Plausibility envelope (±{_PLAUSIBILITY_SIGMA:g}σ):** "
             f"[{c.plaus_lo:g}, {c.plaus_hi:g}] {c.unit}")
    L.append("")
    L.append(f"**Why this band:** {c.criterion_note}")
    if action and action.brief:
        L.append("")
        L.append(f"**How to run it:** {action.brief}")
    L.append("")
    L.append("**Verdict rules (fixed):** inside the band = PASS. Outside the "
             "band but inside the envelope = FAIL (a design finding). Outside "
             "the envelope = DISCREPANT — the run and the ledger disagree; "
             "audit units, frame, BCs and geometry version before acting on "
             "either number.")
    if c.status != Verdict.OPEN.value:
        L.append("")
        L.append(f"## Verdict: **{c.status.upper()}**")
        L.append(f"Result: {c.result_value:g} {c.unit} on {c.judged_on}.")
        L.append("")
        L.append(c.judgment_note)
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  Self-test (python3 -m suspension.proof_engine)
# --------------------------------------------------------------------------- #
if __name__ == "__main__":  # pragma: no cover
    from .interfaces import SubsystemInterface, blank_ledger
    led = blank_ledger()
    led.set(SubsystemInterface(name="chassis", mass_kg=45, cg_z_mm=300,
                                  is_estimate=True))
    led.set(SubsystemInterface(name="powertrain", mass_kg=60, cg_z_mm=320,
                                  peak_power_kw=68, heat_reject_w=3200,
                                  is_estimate=True))
    led.set(SubsystemInterface(name="cooling", cooling_airflow_cms=0.14,
                                  mass_kg=6, is_estimate=True))
    qs = build_uncertainty_ledger(led)
    obj = DEFAULT_OBJECTIVES[1]
    rep = analyze_objective(obj, qs)
    plan = plan_proofs(obj, qs)
    print(render_proof_plan_md(plan, rep))
