# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/express.py — ⚡ The Express Lane
# ============================================================================
"""
The Express Lane — "short for time? two sentences and your data, we run it."

The briefing questionnaire is five taps and it is worth every one of them: it
teaches a new member WHICH tool to open and WHY. But a member at 01:40 the
night before a design review does not want to be taught. They want the files.

So this is the other door. Two sentences and a data drop in; a ZIP of finished
reports, CSVs and a manifest out. Same engines, same physics, no questionnaire.

It is a PIPELINE OF FOUR DETERMINISTIC STAGES, and not one of them is a
language model:

  1. THE GRAMMAR (`parse_request`) — a keyword table maps words to tool ids,
     and a nearest-quantity binder attaches numbers to the parameter they sit
     next to ("240 kg car, cg 290 mm, 62% front bias"). Units are converted to
     the toolkit's storage units and the conversion is printed. Ambiguity is
     resolved by DIMENSION, not by guessing: 'g' beside a mass word is grams,
     'g' beside a cornering word is gravities, and the receipt says which
     reading it took. Same sentence in, same spec out, forever.

  2. THE SNIFFER (`sniff_files`) — CSV/TSV delimiter and header detection via
     the stdlib sniffer, then column names are matched against a synonym table
     to canonical channels. Scale is inferred from the data itself and
     disclosed: a speed column with a median of 63 is km/h and is divided by
     3.6; a lateral column peaking at 1.8 is gravities and is left alone.
     Sample rate, gaps, flatlined channels and NaN runs come back as flags,
     not as silence.

  3. THE PLANNER (`plan`) — a job registry. Every job DECLARES the channels
     and parameters it needs. A job runs if the sentence asked for its tool,
     or if the uploaded data satisfies it outright (the data is a request too:
     drop a log with steering and lateral-g and the event finder runs whether
     or not anyone typed the word). A job that cannot run is SKIPPED WITH A
     NAMED REASON, never silently dropped, and the reason names the missing
     channel or parameter so the fix is one column away.

  4. THE RUNNER + BUNDLER (`run_express`, `bundle_zip`) — jobs are executed
     flagged-not-raised: one bad job returns a FAILED artifact with its
     traceback line, the other eleven still ship. The ZIP is byte-deterministic
     (fixed timestamps, sorted entries) so two identical requests produce two
     identical files — which is the whole point of a design-review artifact.

What this is NOT: it is not a shortcut past the engineering. Every number in
the bundle carries the same fidelity note and the same caveats it carries in
its own tab, and the README says, per tool, which tab to open when there IS
time. The Express Lane gets you a defensible starting artifact in ninety
seconds; it does not get you out of understanding it.

Pure Python + NumPy, headless, no network, no external services. Self-test:
    python3 -m suspension.express
UI in ui/express_lane.py.
"""

from __future__ import annotations

import csv
import io
import json
import math
import re
import time
import traceback
import zipfile
from dataclasses import dataclass, field as _dcfield, asdict
from collections.abc import Callable, Iterable, Sequence

import numpy as np


__all__ = [
    "Ask", "Channel", "DataBundle", "Job", "PlannedJob", "Artifact",
    "ExpressRun", "parse_request", "sniff_files", "plan", "run_express",
    "bundle_zip", "register_job", "JOBS", "validate_param_table",
]


# =========================================================================== #
#  0 · Units — every conversion the pipeline is allowed to make, in one table
# =========================================================================== #
#  unit token → (dimension, factor to storage unit)
#  storage units: length mm · mass kg · accel g · speed m/s · angle deg
#                 stiffness N/mm · fraction 0..1 · money USD · time s
_UNITS: dict[str, tuple[str, float]] = {
    # length → mm
    "mm": ("length", 1.0), "millimetre": ("length", 1.0),
    "millimeter": ("length", 1.0), "cm": ("length", 10.0),
    "m": ("length", 1000.0), "in": ("length", 25.4),
    "inch": ("length", 25.4), '"': ("length", 25.4),
    # mass → kg
    "kg": ("mass", 1.0), "kgs": ("mass", 1.0), "kilo": ("mass", 1.0),
    "lb": ("mass", 0.45359237), "lbs": ("mass", 0.45359237),
    "gram": ("mass", 0.001), "grams": ("mass", 0.001),
    # acceleration → g
    "g": ("accel", 1.0), "gs": ("accel", 1.0),
    "m/s2": ("accel", 1.0 / 9.80665), "m/s^2": ("accel", 1.0 / 9.80665),
    # speed → m/s
    "m/s": ("speed", 1.0), "mps": ("speed", 1.0),
    "kph": ("speed", 1.0 / 3.6), "km/h": ("speed", 1.0 / 3.6),
    "kmh": ("speed", 1.0 / 3.6), "mph": ("speed", 0.44704),
    # angle → deg
    "deg": ("angle", 1.0), "degs": ("angle", 1.0), "degree": ("angle", 1.0),
    "degrees": ("angle", 1.0), "°": ("angle", 1.0),
    "rad": ("angle", 180.0 / math.pi),
    # stiffness → N/mm
    "n/mm": ("stiffness", 1.0), "nmm": ("stiffness", 1.0),
    "lbf/in": ("stiffness", 0.175126835), "lb/in": ("stiffness", 0.175126835),
    # fraction
    "%": ("fraction", 0.01), "pct": ("fraction", 0.01),
    # money
    "$": ("money", 1.0), "usd": ("money", 1.0), "eur": ("money", 1.0),
    # time
    "s": ("time", 1.0), "sec": ("time", 1.0), "secs": ("time", 1.0),
    "ms": ("time", 0.001), "min": ("time", 60.0),
    # torque → Nm.  Its absence was a live bug: "140 Nm motor torque" parsed
    # the 140 as a bare number, which then lost a tie-break to the "6.5 kWh"
    # four characters further away and became the motor torque.
    "nm": ("torque", 1.0), "n-m": ("torque", 1.0), "n·m": ("torque", 1.0),
    "ftlb": ("torque", 1.35582), "ft-lb": ("torque", 1.35582),
    "lbft": ("torque", 1.35582),
    # temperature → °C
    "°c": ("temperature", 1.0), "degc": ("temperature", 1.0),
    "celsius": ("temperature", 1.0),
    # current → A
    "amps": ("current", 1.0), "amp": ("current", 1.0), "a": ("current", 1.0),
    "ma": ("current", 0.001),
    # pressure → bar
    "bar": ("pressure", 1.0), "psi": ("pressure", 0.0689476),
    "kpa": ("pressure", 0.01), "mpa": ("pressure", 10.0),
    # volumetric flow → L/min
    "lpm": ("flow", 1.0), "l/min": ("flow", 1.0), "lmin": ("flow", 1.0),
    "gpm": ("flow", 3.78541),
    # voltage → V
    "v": ("voltage", 1.0), "volt": ("voltage", 1.0),
    "volts": ("voltage", 1.0), "vdc": ("voltage", 1.0),
    "kv": ("voltage", 1000.0),
    # energy → kWh
    "kwh": ("energy", 1.0), "wh": ("energy", 0.001), "mj": ("energy", 1.0 / 3.6),
    # power → kW
    "kw": ("power", 1.0), "hp": ("power", 0.7457), "bhp": ("power", 0.7457),
    # a count is a dimension too — "22 laps" is a number with a meaning
    "lap": ("count", 1.0), "laps": ("count", 1.0),
    "cell": ("count", 1.0), "cells": ("count", 1.0),
}

#  'g' is the only genuinely ambiguous token in FSAE prose (gram vs gravity).
#  It is resolved by the DIMENSION of the parameter it binds to, never by a
#  prior — and whichever reading is taken lands on the receipt.
_AMBIGUOUS = {"g": {"accel": 1.0, "mass": 0.001}}

_NUM = r"[-+]?(?:\d{1,3}(?:,\d{3})+(?:\.\d+)?|\d*\.\d+|\d+)"
_UNIT_ALT = "|".join(sorted((re.escape(u) for u in _UNITS), key=len,
                            reverse=True))
#  The unit group needs a trailing boundary. Without one the alternation
#  matches a unit that is merely the FIRST LETTERS of the next word: adding
#  "a" for amps made "47 and a 280 kg car" parse the 47 as 47 amps, because
#  "a" matched the "a" of "and" — and the fraction that wanted it then
#  rejected it as the wrong dimension. "m" inside "motor" was the same bug
#  sitting latent.
_QTY_RE = re.compile(
    rf"(?P<cur>[\$€£])?\s*(?P<num>{_NUM})\s*(?P<k>k\b)?\s*"
    rf"(?:(?P<unit>{_UNIT_ALT})(?![a-z0-9]))?",
    re.IGNORECASE)


@dataclass
class Quantity:
    """One number found in the prose, with where it was and what it meant."""
    raw: str
    value: float
    unit: str | None
    dim: str | None
    start: int
    end: int
    claimed_by: str | None = None

    @property
    def mid(self) -> float:
        return 0.5 * (self.start + self.end)


def _scan_quantities(text: str) -> list[Quantity]:
    """Every number-shaped thing in the text, with its unit and character
    span. Bare numbers are kept — a parameter word can still claim one, and
    the assumption gets receipted."""
    out: list[Quantity] = []
    for m in _QTY_RE.finditer(text):
        raw_num = m.group("num")
        if raw_num is None:
            continue
        try:
            val = float(raw_num.replace(",", ""))
        except ValueError:
            continue
        if m.group("k"):
            val *= 1000.0
        unit = (m.group("unit") or "").lower() or None
        if m.group("cur"):
            unit, dim = "$", "money"
        else:
            dim = _UNITS[unit][0] if unit in _UNITS else None
        out.append(Quantity(raw=m.group(0).strip(), value=val, unit=unit,
                            dim=dim, start=m.start(), end=m.end()))
    return out


def _convert(q: Quantity, want_dim: str) -> tuple[float | None, str]:
    """Value in the storage unit for `want_dim`, plus a receipt fragment.
    Returns (None, reason) when the quantity's dimension cannot be honoured."""
    if q.unit is None:
        #  "weight distribution 47" means 47 %, not 47. Storing it raw put a
        #  47.0 into a 0..1 parameter and produced load transfers off by two
        #  orders of magnitude — silently, because 47 is a perfectly ordinary
        #  looking number.
        if want_dim == "fraction":
            if 1.0 < q.value <= 100.0:
                return q.value / 100.0, ("bare number answering a fraction — "
                                         "read as a percentage, ÷100")
            if q.value > 100.0:
                return None, (f"{q.value:g} cannot be a fraction or a "
                              f"percentage")
        return q.value, "no unit given — read in the toolkit's own unit"
    u = q.unit.lower()
    if u in _AMBIGUOUS and want_dim in _AMBIGUOUS[u]:
        f = _AMBIGUOUS[u][want_dim]
        return q.value * f, (f"'{u}' read as {want_dim} "
                             f"(the parameter's dimension decided it)")
    if u not in _UNITS:
        return None, f"unit '{u}' not in the conversion table"
    dim, fac = _UNITS[u]
    if dim == want_dim:
        note = "" if fac == 1.0 else f"×{fac:g} → storage unit"
        return q.value * fac, note
    #  a percentage answering a fraction, and a bare fraction answering a
    #  percentage, are the same honest intent
    if want_dim == "fraction" and dim == "fraction":
        return q.value * fac, "×0.01"
    return None, (f"'{u}' is a {dim}, but this parameter is a {want_dim} "
                  f"— not applied")


# =========================================================================== #
#  1 · The grammar — tools, parameters, deliverables
# =========================================================================== #
#  tool id → the words that ask for it. Ids match _TAB_META in the shell, so
#  the README can name the tab to open when there IS time.
_TOOL_WORDS: dict[str, tuple[str, ...]] = {
    "kinematics": ("kinematic", "bump steer", "bumpsteer", "camber",
                   "toe", "caster", "kpi", "scrub", "hardpoint", "geometry",
                   "pickup", "instant centre", "instant center"),
    "roll": ("roll", "load transfer", "roll centre", "roll center",
             "lateral load", "jacking", "weight transfer", "arb",
             "anti-roll", "antiroll"),
    "compliance": ("compliance", "flex", "deflect", "stiffness", "stiff",
                   "member load", "chassis torsion"),
    "brakes": ("brake", "braking", "bias", "lock-up", "lockup", "rotor",
               "pedal", "caliper", "fade"),
    "tire": ("tire", "tyre", "grip", "pacejka", "friction", "mu",
             "slip angle", "ggv", "g-g-v", "envelope", "traction circle",
             "friction circle"),
    "setup": ("setup", "spring", "damper", "damping", "wheel rate",
              "motion ratio", "ride frequency", "corner weight"),
    "laptime": ("lap", "laptime", "lap time", "track", "endurance",
                "autocross", "skidpad", "acceleration event"),
    "daq": ("telemetry", "daq", "log", "logger", "channel", "sample rate",
            "data acquisition", "run data"),
    "ev": ("ev", "powertrain", "motor", "inverter", "pack", "battery",
           "accumulator", "energy", "state of charge", "tractive",
           "tractive system", "precharge", "shutdown circuit", "tsal",
           "bspd", "contactor", "imd", "hv", "high voltage"),
    "weight": ("mass", "weight", "lightweight", "gram", "budget mass"),
    "cooling": ("cooling", "coolant", "radiator", "rad", "heat rejection",
                "water pump", "test rig", "cooling rig", "glycol",
                "heat load", "overheating"),
    "printing": ("printed", "3d print", "3-d print", "filament", "onyx",
                 "paht", "additive", "layer adhesion", "print orientation",
                 "markforged", "manifold"),
    "powertrain": ("powertrain", "gear ratio", "final drive", "sprocket",
                   "driveline", "motor mount", "diff mount", "chain",
                   "acceleration event"),
    #  NB: "harness" belongs to the frames vocabulary — in FSAE it means the
    #  driver restraint, not the wiring loom. Using it here would fire the
    #  wrong engine on the more safety-critical of the two meanings.
    "wiring": ("wiring", "wire", "wires", "gauge", "awg", "cable", "loom",
               "conductor", "ampacity", "voltage drop", "fuse", "fusing",
               "wire size", "cable size", "gauge chart", "tefzel",
               "silicone", "thhn", "m22759", "insulation"),
    "rules": ("rules", "rulebook", "legal", "rule check", "tech inspection",
              "technical inspection", "scrutineering", "penalty",
              "violation", "energy meter", "compliant with the rules"),
    "aero": ("aero", "aerodynamic", "downforce", "drag", "wing", "undertray",
             "diffuser", "cla", "cda", "l/d"),
    "dfmea": ("dfmea", "fmea", "failure mode", "rpn", "risk", "severity",
              "occurrence", "detection"),
    "fusebox": ("fusebox", "fuse", "overload path", "sacrificial",
                "first failure", "breaks first", "weak link"),
    "pcb": ("pcb", "board", "kicad", "altium", "pcbdoc", "trace", "via",
            "copper", "net", "layout"),
    "electronics": ("electronics", "wiring", "loom", "connector", "ecu",
                    "integration ledger", "lv", "grounding"),
    "thermal": ("thermal", "temperature", "cooling", "overheat", "fan",
                "cell temp", "heat"),
    "transient": ("transient", "step steer", "yaw response", "settling",
                  "simulforge", "mechatronic", "actuator", "bus latency"),
    "omnicore": ("omnicore", "pareto", "trade study", "trade-off", "knee",
                 "configuration sweep", "co-optimise", "co-optimize"),
    "genesis": ("genesis", "inverse design", "solve for hardpoints",
                "synthesise", "synthesize"),
    "ghost": ("ghost", "ghost topology", "erosion", "rigid model",
              "is the rigid model honest", "faithful", "deflection budget"),
    "morph": ("morphmesh", "morph", "topology optimisation",
              "topology optimization", "bracket shape"),
    "frames": ("frame", "chassis", "tube", "harness", "seat", "belt",
               "roll hoop", "bulkhead", "panel", "impact attenuator",
               "mounting", "bracket"),
}

#  parameter → (storage field, dimension, default, the words that name it)
#  Ordered: earlier entries claim their quantity first, so the more specific
#  phrase ("front track") wins over the general one ("track").
_PARAM_WORDS: list[tuple[str, str, float, tuple[str, ...]]] = [
    #  Module limits are per-module and the pack figure cannot satisfy them,
    #  so the module phrases must claim their quantity first.
    ("module_kwh", "energy", None, ("module energy", "per module",
                                    "each module")),
    ("module_kg", "mass", None, ("module mass", "module weighs")),
    ("module_v", "voltage", None, ("module voltage", "segment voltage")),
    ("pack_v_max", "voltage", None, ("system voltage", "pack voltage",
                                     "ts voltage", "bus voltage",
                                     "maximum voltage", "tractive")),
    ("track_front_mm", "length", 1200.0, ("front track", "track front")),
    ("track_rear_mm", "length", 1180.0, ("rear track", "track rear")),
    ("wheelbase_mm", "length", 1550.0, ("wheelbase", "wheel base")),
    ("cg_height_mm", "length", 300.0, ("cg height", "cg", "c.g.", "cog",
                                       "centre of gravity",
                                       "center of gravity")),
    ("weight_dist_front", "fraction", 0.47, ("weight distribution",
                                             "front weight", "weight split",
                                             "distribution")),
    ("mass_kg", "mass", 280.0, ("mass", "weight", "kerb", "curb", "heavier",
                                "lighter", "car weighs", "vehicle mass")),
    ("lateral_g", "accel", 1.4, ("lateral", "cornering", "corner", "skidpad",
                                 "lat g", "steady state")),
    ("long_g", "accel", 1.5, ("braking", "decel", "longitudinal",
                              "brake g", "stopping")),
    ("brake_bias_front", "fraction", 0.62, ("bias", "brake bias",
                                            "front bias")),
    ("spring_rate_N_per_mm", "stiffness", 35.0, ("spring rate", "spring",
                                                 "wheel rate")),
    ("travel_mm", "length", 25.0, ("travel", "bump travel", "droop",
                                   "wheel travel")),
    ("mu_peak", None, 1.55, ("mu", "peak mu", "friction coefficient")),
    #  Cooling and printed-part duty. All specific phrases, placed after the
    #  chassis lengths so 'wall thickness' cannot be claimed by a track or
    #  wheelbase word (validate_param_table enforces the rest).
    ("coolant_c", "temperature", 80.0, ("coolant temperature", "coolant temp",
                               "service temperature", "loop temperature")),
    ("flow_lpm", "flow", 12.0, ("flow rate", "coolant flow", "pump flow",
                                "flow")),
    ("cap_bar", "pressure", 1.1, ("cap pressure", "system pressure",
                                  "rad cap", "coolant pressure")),
    ("wall_mm", "length", 3.0, ("wall thickness", "wall")),
    ("bore_mm", "length", 25.0, ("manifold bore", "bore", "inner diameter",
                                 "internal diameter")),
    ("rad_ua", None, 120.0, ("radiator ua", "ua")),
    #  Wiring. `run_length_mm` is stored in mm like every other length and
    #  divided at the point of use — one storage unit per dimension, always.
    ("run_length_mm", "length", 1500.0, ("cable run", "cable length",
                                         "run length", "wire run",
                                         "loom length")),
    ("ambient_c", "temperature", 30.0, ("ambient temperature", "ambient",
                                        "loom temperature",
                                        "bay temperature")),
    ("n_bundled", None, 1.0, ("conductors in the loom", "bundled conductors",
                              "conductors", "bundled")),
    ("termination_c", "temperature", None, ("termination", "lug", "crimp",
                                            "connector rating",
                                            "terminal rating")),
    ("fuse_a", "current", None, ("fuse", "fuse rating", "breaker")),
    ("lv_current_a", "current", None, ("pump current", "fan current",
                                       "lv current")),
    ("final_drive", None, None, ("final drive", "gear ratio")),
    ("motor_torque_nm", "torque", 140.0, ("motor torque", "peak torque")),
    ("wheel_radius_mm", "length", 228.0, ("wheel radius", "tyre radius",
                                          "tire radius", "rolling radius")),
    ("budget_usd", "money", None, ("budget", "cost", "spend")),
    ("pack_kwh", "energy", 6.5, ("pack", "accumulator", "battery",
                                 "pack energy")),
    ("power_kw", "power", 60.0, ("power", "motor", "inverter", "tractive")),
    #  Must precede endurance_laps: its phrases are the specific ones, and
    #  the generic "laps" would otherwise claim the quantity first. One is
    #  how far the car must go, the other is how far this log went.
    ("logged_laps", "count", None, ("logged laps", "laps in the log",
                                    "log covers", "this log is",
                                    "log is")),
    ("endurance_laps", "count", 22.0, ("endurance", "laps", "lap count")),
    ("rotor_dia_mm", "length", 220.0, ("rotor", "disc", "disk")),
]

#  dimension → the one parameter it can only be. Ambiguous dimensions are
#  absent on purpose (see the unit-implied binding pass in parse_request).
_UNIT_IMPLIES: dict[str, str] = {
    "mass": "mass_kg",
    "stiffness": "spring_rate_N_per_mm",
    "money": "budget_usd",
    "energy": "pack_kwh",
    "voltage": "pack_v_max",
    "torque": "motor_torque_nm",
    "power": "power_kw",
    "count": "endurance_laps",
}

#  deliverable words — what the member wants OUT. Absence means "everything".
_DELIVERABLE_WORDS: dict[str, tuple[str, ...]] = {
    "md": ("report", "write-up", "writeup", "summary", "memo", "document"),
    "csv": ("csv", "table", "spreadsheet", "raw numbers", "data out"),
    "json": ("json", "machine readable", "api", "handover"),
}

#  Consumed but deliberately inert: recognised so they do not land in
#  "ignored", disclosed as changing nothing.
_ACK_WORDS = ("please", "quick", "quickly", "asap", "tonight", "tomorrow",
              "review", "design review", "need", "want", "give", "run",
              "generate", "files", "everything", "short", "time", "help",
              # structural qualifiers and output nouns: real words, but not
              # domain words, so surfacing them as "not understood" is noise
              "front", "rear", "left", "right", "outer", "inner", "each",
              "numbers", "loads", "times", "values", "results", "figures",
              "plot", "plots", "graph", "graphs", "curve", "curves",
              # budget vocabulary — parse_budget consumed these already
              "have", "minute", "minutes", "second", "seconds", "hour",
              "hours", "wait", "budget", "rush", "hurry", "night",
              # ordinary verbs and connectives, not domain words
              "check", "checks", "checking", "system", "only", "not",
              "instead", "stocks", "carry", "carries", "make", "made",
              "using", "use", "used", "look", "looks")

_STOP = {"the", "a", "an", "and", "for", "with", "our", "your", "my", "we",
         "us", "is", "are", "was", "were", "of", "on", "in", "to", "at",
         "it", "its", "that", "this", "from", "by", "as", "be", "car",
         "team", "just", "got", "get", "all", "can", "you", "i"}



def _find_word(low: str, phrase: str) -> int:
    """Index of `phrase` in `low` as a whole word, else -1.

    Substring matching is how a keyword grammar quietly embarrasses itself:
    plain `"ev" in text` fires on "review", "lap" on "overlap". The boundary
    is letters only, so hyphens, digits and punctuation still delimit — and a
    trailing plural is tolerated, because "in the corners" means cornering.
    """
    pat = _WORD_CACHE.get(phrase)
    if pat is None:
        #  A trailing plural is the same word: "in the corners" is the
        #  cornering case, "spring rates" is the spring rate.
        pat = _WORD_CACHE[phrase] = re.compile(
            r"(?<![a-z])" + re.escape(phrase) + r"(?:e?s)?(?![a-z])")
    m = pat.search(low)
    return m.start() if m else -1


_WORD_CACHE: dict[str, re.Pattern] = {}

# =========================================================================== #
#  1b · The time budget — the lane's promise, made explicit
# =========================================================================== #
#  The express lane promises files "while you wait". That promise is a NUMBER,
#  and until it is written down every expensive engine is a judgement call
#  made at 01:40 by whoever is adding a job. So: 90 seconds by default, which
#  is the outer edge of a member willing to watch a spinner, and the member
#  can say otherwise in the same sentence as everything else.
_DEFAULT_BUDGET_S = 90.0

_BUDGET_PHRASES: list[tuple[str, float]] = [
    ("overnight", 8 * 3600.0), ("all night", 8 * 3600.0),
    ("over lunch", 45 * 60.0), ("take your time", 30 * 60.0),
    ("no rush", 15 * 60.0), ("i can wait", 10 * 60.0),
    ("while i wait", 90.0), ("right now", 60.0), ("asap", 45.0),
    ("in a hurry", 45.0), ("short on time", 45.0), ("quick", 45.0),
]
_BUDGET_NUM_RE = re.compile(
    r"(?<![a-z])(\d+(?:\.\d+)?)\s*(hours?|hrs?|minutes?|mins?|seconds?|secs?)"
    r"(?![a-z])", re.IGNORECASE)
_BUDGET_UNIT_S = {"hour": 3600.0, "hr": 3600.0, "minute": 60.0, "min": 60.0,
                  "second": 1.0, "sec": 1.0}


def parse_budget(text: str) -> tuple[float, str | None]:
    """How long the member is willing to wait, and what said so.

    An explicit duration always beats a mood word: "I'm in a hurry but I can
    give it five minutes" is five minutes, not forty-five seconds.
    """
    low = (text or "").lower()
    m = _BUDGET_NUM_RE.search(low)
    if m:
        unit = m.group(2).rstrip("s").rstrip(".")
        for k, mult in _BUDGET_UNIT_S.items():
            if unit.startswith(k):
                return float(m.group(1)) * mult, m.group(0).strip()
    for phrase, secs in _BUDGET_PHRASES:
        if phrase in low:
            return secs, phrase
    return _DEFAULT_BUDGET_S, None


# ---------------------------------------------------------------------------
#  Engines that are deliberately NOT in the lane, and the reason, by name.
#  A missing tool with no explanation reads as an oversight; a missing tool
#  with a reason reads as a decision. These are decisions.
_NOT_IN_LANE: dict[str, str] = {
    "genesis": (
        "Inverse Genesis needs a **legal volume** — the keep-out boxes your "
        "packaging actually allows — before it can solve for hardpoints. Two "
        "sentences cannot supply a packaging envelope, and a lane that "
        "invented one would be inventing the answer. Open the 🧬 Inverse "
        "Genesis tab, draw the volume once, and it is yours from then on."),
}

#  How far from a parameter word a number may sit and still belong to it, in
#  characters, and what a number with no unit at all costs against one that
#  carries the right unit. Both are declared knobs, not folklore: widen the
#  window and the binder gets greedier, raise the penalty and it trusts units
#  more than proximity. ~12 characters is about three words.
_BIND_WINDOW = 45.0
_UNITLESS_PENALTY = 12.0


def _dims_of(q: Quantity) -> set:
    """Every dimension a quantity could legitimately be.

    Normally one. For the genuinely ambiguous tokens ('g' = gram or gravity)
    it is the whole ambiguity set, so the binder can still consider the
    quantity for either reading and let `_convert` record which it took.
    """
    if q.unit and q.unit.lower() in _AMBIGUOUS:
        return set(_AMBIGUOUS[q.unit.lower()]) | ({q.dim} if q.dim else set())
    return {q.dim} if q.dim else set()


@dataclass
class Ask:
    """The typed request + the parse receipt."""
    text: str
    tools: list[str] = _dcfield(default_factory=list)
    params: dict[str, float] = _dcfield(default_factory=dict)
    deliverables: list[str] = _dcfield(default_factory=list)
    n_sentences: int = 0
    budget_s: float = _DEFAULT_BUDGET_S
    budget_source: str | None = None
    consumed: list[str] = _dcfield(default_factory=list)
    assumptions: list[str] = _dcfield(default_factory=list)
    ignored: list[str] = _dcfield(default_factory=list)

    def param(self, key: str, fallback: float | None = None) -> float:
        """A parameter the member gave, else the DECLARED default.

        The declared default always wins over a caller's fallback, because
        the README prints the declared one — a job quietly using a different
        number than the receipt claims is the exact dishonesty this module
        is built to prevent. `fallback` covers keys not in the table at all.
        """
        if key in self.params:
            return self.params[key]
        for f, _d, dflt, _w in _PARAM_WORDS:
            if f == key and dflt is not None:
                return dflt
        return fallback  # type: ignore[return-value]

    def summary(self) -> dict:
        return {k: v for k, v in asdict(self).items() if k != "text"}


def _sentences(text: str) -> list[str]:
    return [s.strip() for s in re.split(r"(?<=[.!?])\s+|\n+", text or "")
            if s.strip()]


def validate_param_table() -> list[str]:
    """The parameter table's one ordering contract, checked instead of trusted.

    If a phrase belonging to an EARLIER entry matches inside a phrase
    belonging to a LATER one, the earlier (generic) entry will claim the
    quantity that the later (specific) one was reaching for. That is not a
    hypothetical: it has cost three separate bugs — 'laps' stealing from
    'log covers', 'mass' stealing from 'module mass', and 'weight' stealing
    from 'weight distribution' — each found only after the fact by a test
    written in response to it.

    Uses `_find_word` itself rather than a substring check, so the plural
    tolerance and word boundaries are exactly the ones the binder applies.
    """
    problems: list[str] = []
    for i, (fi, _di, _vi, wi) in enumerate(_PARAM_WORDS):
        for j, (fj, _dj, _vj, wj) in enumerate(_PARAM_WORDS):
            if j <= i:
                continue
            for pi in wi:
                for pj in wj:
                    if pi != pj and _find_word(pj, pi) >= 0:
                        problems.append(
                            f"'{pi}' ({fi}, #{i}) matches inside '{pj}' "
                            f"({fj}, #{j}) — move {fj} above {fi}")
    return problems


def parse_request(text: str) -> Ask:
    """Deterministic grammar over the member's two sentences.

    Same text in, same Ask out, forever. Every token the grammar consumed,
    every default it assumed and every word it did not understand is on the
    receipt — the parser's ignorance is printed, never papered over.
    """
    raw = (text or "").strip()
    low = raw.lower()
    ask = Ask(text=raw, n_sentences=len(_sentences(raw)))
    ask.budget_s, ask.budget_source = parse_budget(raw)

    # ---- 1 · quantities, then parameters claim the nearest one ------------ #
    qtys = _scan_quantities(raw)
    for field, dim, default, words in _PARAM_WORDS:
        hit_pos, hit_word = None, None
        for w in words:
            i = _find_word(low, w)
            if i >= 0 and (hit_pos is None or i < hit_pos):
                hit_pos, hit_word = i, w
        if hit_pos is None:
            continue
        anchor = hit_pos + len(hit_word) / 2.0
        #  Nearest is not enough. In "6.5 kWh pack, 22 endurance laps" the
        #  bare 22 sits closer to "pack" than the kWh figure does, and a
        #  distance-only binder hands the pack a lap count. So a quantity
        #  carrying the dimension the parameter wants gets a head start, a
        #  bare number pays a declared penalty, and a quantity of the WRONG
        #  dimension is not a candidate at all.
        best: Quantity | None = None
        best_score = float("inf")
        for q in qtys:
            if q.claimed_by is not None:
                continue
            dist = abs(q.mid - anchor)
            if dist > _BIND_WINDOW:
                continue
            if dim is None:
                #  A parameter with no dimension still prefers a number with
                #  no unit. Letting it take a kWh figure for free is how
                #  "motor torque" ended up holding a pack energy.
                penalty = 0.0 if q.dim is None else _UNITLESS_PENALTY
            elif dim in _dims_of(q):
                penalty = 0.0
            elif q.dim is None:
                penalty = _UNITLESS_PENALTY
            else:
                continue                     # wrong dimension: not a candidate
            if dist + penalty < best_score:
                best, best_score = q, dist + penalty
        if best is None:
            continue
        if dim is None:                          # dimensionless parameter
            val, note = best.value, "dimensionless"
        else:
            val, note = _convert(best, dim)
        if val is None:
            ask.assumptions.append(
                f"'{hit_word}' saw '{best.raw}' but {note} — using the "
                f"default {default}")
            continue
        best.claimed_by = field
        ask.params[field] = float(val)
        ask.consumed.append(
            f"'{best.raw}' beside '{hit_word}' → {field} = {val:g}"
            + (f"  ({note})" if note else ""))

    # ---- 1b · unit-implied binding --------------------------------------- #
    #  A unit can name its own parameter when nothing else claimed it and the
    #  dimension maps to exactly ONE parameter in the table: "245 kg car" is a
    #  mass whether or not the word "mass" appears. Dimensions that map to
    #  several parameters (length → cg / wheelbase / track) are deliberately
    #  NOT inferred — guessing which length was meant is exactly the kind of
    #  invention this grammar exists to avoid.
    for dim, field in _UNIT_IMPLIES.items():
        if field in ask.params:
            continue
        for q in qtys:
            if q.claimed_by is not None or q.dim != dim:
                continue
            q.claimed_by = field
            ask.params[field] = float(q.value * _UNITS[q.unit][1])
            ask.consumed.append(
                f"'{q.raw}' → {field} = {ask.params[field]:g} "
                f"(inferred from the unit alone — no parameter word nearby)")
            break

    # ---- 2 · tools ------------------------------------------------------- #
    for tool, words in _TOOL_WORDS.items():
        hits = [w for w in words if _find_word(low, w) >= 0]
        if hits:
            ask.tools.append(tool)
            ask.consumed.append(f"'{hits[0]}' → the {tool} engine")

    # ---- 3 · deliverables ------------------------------------------------ #
    for kind, words in _DELIVERABLE_WORDS.items():
        if any(_find_word(low, w) >= 0 for w in words):
            ask.deliverables.append(kind)
    if not ask.deliverables:
        ask.deliverables = ["md", "csv", "json"]
        ask.assumptions.append(
            "no output format named — shipping all three (report, CSV, JSON)")

    # ---- 4 · what it could not place ------------------------------------- #
    claimed_spans = [(q.start, q.end) for q in qtys if q.claimed_by]
    used_words = set()
    for tool, words in _TOOL_WORDS.items():
        if tool in ask.tools:
            for w in words:
                if _find_word(low, w) >= 0:
                    used_words.update(w.split())
    for _f, _d, _dv, words in _PARAM_WORDS:
        for w in words:
            if _find_word(low, w) >= 0:
                used_words.update(w.split())
    for kind, words in _DELIVERABLE_WORDS.items():
        for w in words:
            if _find_word(low, w) >= 0:
                used_words.update(w.split())
    used_words.update(_ACK_WORDS)
    #  A unit is never an unknown word — it was read, it just was not a noun.
    used_words.update(_UNITS)

    tokens = re.findall(r"[a-zA-Z][a-zA-Z\-']{2,}", low)

    def _understood(tok: str) -> bool:
        #  A hyphenated compound counts as understood if any of its parts was:
        #  'paht-cf' should not be reported as unknown when 'paht' summoned
        #  the printed-parts engine.
        if tok in used_words or tok in _STOP:
            return True
        parts = [x for x in tok.split("-") if x]
        return any(x in used_words or x in _STOP for x in parts)

    ask.ignored = sorted({t for t in tokens if not _understood(t)})[:24]

    # ---- 5 · the honest defaults ----------------------------------------- #
    if not ask.tools:
        ask.assumptions.append(
            "no tool words recognised — the plan falls back to whatever the "
            "uploaded data can feed, plus the geometry baseline")
    unclaimed = [q for q in qtys if q.claimed_by is None and q.unit]
    if unclaimed:
        ask.assumptions.append(
            "numbers seen but not bound to a parameter (no parameter word "
            "within the 45-character binding window): "
            + ", ".join(q.raw for q in unclaimed[:6]))
    if ask.budget_source:
        ask.consumed.append(
            f"'{ask.budget_source}' → a time budget of {ask.budget_s:g} s")
    else:
        ask.assumptions.append(
            f"no time budget stated — using the lane default of "
            f"{_DEFAULT_BUDGET_S:g} s; say 'I have five minutes' and the "
            f"expensive engines become available")
    if ask.n_sentences > 3:
        ask.assumptions.append(
            f"{ask.n_sentences} sentences given — the grammar read all of "
            "them; the two-sentence guidance is about YOUR time, not a limit")
    if claimed_spans:
        ask.consumed.append(
            f"{len(claimed_spans)} of {len(qtys)} numbers bound to parameters")
    return ask



# =========================================================================== #
#  2 · The sniffer — columns to canonical channels, scale inferred and printed
# =========================================================================== #
#  canonical → (label, storage unit, aliases…)
_CHANNELS: dict[str, tuple[str, str, tuple[str, ...]]] = {
    "time":        ("time", "s", ("time", "t", "timestamp", "elapsed",
                                  "secs", "seconds", "utc")),
    "distance":    ("distance", "m", ("distance", "dist", "lapdist",
                                      "lap_distance", "odo", "s_m")),
    "speed":       ("speed", "m/s", ("speed", "velocity", "vehicle_speed",
                                     "gps_speed", "vcar", "v")),
    "ax":          ("longitudinal acceleration", "g",
                    ("ax", "accel_x", "acc_x", "longitudinal", "long_accel",
                     "glong", "g_long", "accx")),
    "ay":          ("lateral acceleration", "g",
                    ("ay", "accel_y", "acc_y", "lateral", "lat_accel",
                     "glat", "g_lat", "accy")),
    "az":          ("vertical acceleration", "g",
                    ("az", "accel_z", "acc_z", "vertical", "accz")),
    "yaw_rate":    ("yaw rate", "deg/s", ("yaw", "yaw_rate", "yawrate",
                                          "gyro_z", "r_deg")),
    "steer":       ("steering angle", "deg", ("steer", "steering",
                                              "steer_angle", "sas", "swa",
                                              "handwheel")),
    "throttle":    ("throttle", "%", ("throttle", "tps", "apps", "pedal_pos",
                                      "accelerator")),
    "brake_front": ("front brake pressure", "bar",
                    ("brake_pressure_front", "brake_front", "brake_f", "bpf",
                     "p_brake_f", "front_brake", "brakepressfront")),
    "brake_rear":  ("rear brake pressure", "bar",
                    ("brake_pressure_rear", "brake_rear", "brake_r", "bpr",
                     "p_brake_r", "rear_brake", "brakepressrear")),
    "damper_fl":   ("damper FL", "mm", ("damper_fl", "susp_fl", "pot_fl",
                                        "shock_fl", "damperposfl")),
    "damper_fr":   ("damper FR", "mm", ("damper_fr", "susp_fr", "pot_fr",
                                        "shock_fr", "damperposfr")),
    "damper_rl":   ("damper RL", "mm", ("damper_rl", "susp_rl", "pot_rl",
                                        "shock_rl", "damperposrl")),
    "damper_rr":   ("damper RR", "mm", ("damper_rr", "susp_rr", "pot_rr",
                                        "shock_rr", "damperposrr")),
    "rpm":         ("engine / motor speed", "rpm", ("rpm", "engine_speed",
                                                    "motor_rpm", "n_motor")),
    #  NB: bare "voltage" and "current" are deliberately NOT aliases. They
    #  are too generic — a column called "Widget Voltage" is not the pack, and
    #  a channel claimed wrongly is worse than one left unmatched, because the
    #  unmatched column at least gets listed for a human to look at.
    "pack_v":      ("pack voltage", "V", ("pack_voltage", "ts_voltage",
                                          "battery_voltage", "vbatt",
                                          "bus_voltage")),
    "pack_i":      ("pack current", "A", ("pack_current", "ts_current",
                                          "battery_current", "ibatt",
                                          "bus_current")),
    "temp_motor":  ("motor temperature", "degC", ("motor_temp", "t_motor",
                                                  "temp_motor")),
    "temp_pack":   ("cell temperature", "degC", ("cell_temp", "pack_temp",
                                                 "t_cell", "temp_cell")),
}

#  the brake-pressure aliases must beat the generic 'brake' word, and the
#  per-corner damper aliases must beat 'damper'; longest alias wins.
_ALIAS_INDEX: list[tuple[str, str]] = sorted(
    ((alias, canon) for canon, (_l, _u, aliases) in _CHANNELS.items()
     for alias in aliases),
    key=lambda ac: len(ac[0]), reverse=True)


def _norm_header(h: str) -> str:
    """'Lat Accel [g]' → 'lat_accel'. Units in brackets are dropped (they are
    re-inferred from the data, which cannot lie about its own scale)."""
    h = re.sub(r"[\(\[\{].*?[\)\]\}]", " ", str(h or ""))
    h = re.sub(r"[^0-9a-zA-Z]+", "_", h).strip("_").lower()
    return re.sub(r"_+", "_", h)


#  Tokens that describe the MEASUREMENT rather than the quantity. Real logs
#  are full of them — "Damper Pos FL", "Brake Press Front", "Speed Filtered" —
#  and a synonym table that does not strip them matches nothing on the first
#  file a team actually drops.
_FILLER = frozenset((
    "pos", "position", "press", "pressure", "sensor", "raw", "filt",
    "filtered", "ch", "chan", "channel", "signal", "val", "value", "avg",
    "mean", "calc", "calculated", "data", "log", "actual", "meas",
    "measured", "left", "right",
))


def _strip_filler(n: str) -> str:
    parts = [t for t in n.split("_") if t and t not in _FILLER]
    return "_".join(parts)


def _lookup(n: str) -> str | None:
    if not n:
        return None
    for alias, canon in _ALIAS_INDEX:
        if n == alias:
            return canon
    for alias, canon in _ALIAS_INDEX:
        if len(alias) >= 3 and (n.startswith(alias + "_")
                                or n.endswith("_" + alias)
                                or f"_{alias}_" in f"_{n}_"):
            return canon
    return None


def _match_channel(header: str) -> str | None:
    n = _norm_header(header)
    return _lookup(n) or _lookup(_strip_filler(n))


@dataclass
class Channel:
    """One recognised column: what it is, what it was, and what we did to it."""
    canon: str
    label: str
    unit: str
    source_column: str
    n: int
    n_finite: int
    vmin: float
    vmax: float
    mean: float
    std: float
    scale_note: str = ""
    source_file: str = ""
    flags: list[str] = _dcfield(default_factory=list)


@dataclass
class DataBundle:
    """Everything the sniffer made of the upload."""
    channels: dict[str, Channel] = _dcfield(default_factory=dict)
    series: dict[str, np.ndarray] = _dcfield(default_factory=dict)
    hardpoints: object | None = None
    #  Uploads that are neither a table nor a hardpoint set, keyed by kind —
    #  a routed board ("board_file", KiCad or Altium ASCII), a DFMEA export.
    #  Jobs declare these via `needs_extra`.
    extras: dict[str, str] = _dcfield(default_factory=dict)
    files: list[str] = _dcfield(default_factory=list)
    unmatched: list[str] = _dcfield(default_factory=list)
    sample_rate_hz: float | None = None
    duration_s: float | None = None
    receipts: list[str] = _dcfield(default_factory=list)
    warnings: list[str] = _dcfield(default_factory=list)

    def has(self, *canon: str) -> bool:
        return all(c in self.series for c in canon)

    def summary(self) -> dict:
        return {
            "files": self.files,
            "channels": {k: asdict(v) for k, v in self.channels.items()},
            "extras": sorted(self.extras),
            "unmatched_columns": self.unmatched,
            "sample_rate_hz": self.sample_rate_hz,
            "duration_s": self.duration_s,
            "warnings": self.warnings,
        }


def _rescale(canon: str, arr: np.ndarray) -> tuple[np.ndarray, str]:
    """Infer the column's scale FROM THE DATA and disclose what was done.

    The header can lie about units; the numbers cannot lie about magnitude.
    A lateral column peaking at 14 is m/s², one peaking at 1.8 is gravities.
    Every rescale here is reversible and printed — none is silent.
    """
    fin = arr[np.isfinite(arr)]
    if fin.size == 0:
        return arr, ""
    peak = float(np.nanmax(np.abs(fin)))
    med = float(np.nanmedian(np.abs(fin)))
    if canon in ("ax", "ay", "az"):
        if peak > 5.0:
            return arr / 9.80665, (f"peak |{canon}| = {peak:.1f} → read as "
                                   f"m/s², divided by 9.80665 to gravities")
        return arr, f"peak |{canon}| = {peak:.2f} → already in gravities"
    if canon == "speed":
        if med > 40.0:
            return arr / 3.6, (f"median speed {med:.0f} → read as km/h, "
                               f"divided by 3.6 to m/s")
        if 25.0 < med <= 40.0:
            return arr * 0.44704, (f"median speed {med:.0f} → read as mph, "
                                   f"×0.44704 to m/s")
        return arr, f"median speed {med:.1f} → already in m/s"
    if canon == "throttle":
        if peak <= 1.5:
            return arr * 100.0, "peak ≤ 1.5 → read as 0–1, ×100 to percent"
        return arr, "already in percent"
    return arr, ""


def _flag(canon: str, arr: np.ndarray) -> list[str]:
    """Quality flags. A flatlined potentiometer is the single most expensive
    thing to discover AFTER the design review, so it is discovered here."""
    flags: list[str] = []
    fin = arr[np.isfinite(arr)]
    n_nan = int(arr.size - fin.size)
    if n_nan:
        flags.append(f"{n_nan} non-finite samples ({100.0*n_nan/max(arr.size,1):.1f} %)")
    if fin.size and float(np.std(fin)) < 1e-9:
        flags.append("FLATLINED — zero variance across the whole log; "
                     "suspect an unplugged sensor before trusting any job "
                     "that used it")
    if fin.size > 10:
        span = float(np.max(fin) - np.min(fin))
        if span > 0 and float(np.std(fin)) / span < 0.005:
            flags.append("near-constant with isolated spikes — likely dropouts")
    return flags


def _read_table(name: str, blob: bytes) -> tuple[list[str], list[list[str]], str]:
    """Delimiter + header detection via the stdlib sniffer, with a declared
    fallback. Returns (header, rows, receipt)."""
    text = blob.decode("utf-8", errors="replace")
    text = text.lstrip("\ufeff")
    sample = text[:8192]
    delim, note = ",", "comma (the fallback — sniffer was not confident)"
    try:
        dial = csv.Sniffer().sniff(sample, delimiters=",;\t|")
        delim = dial.delimiter
        note = f"delimiter '{delim}' (stdlib sniffer)"
    except Exception:                                        # noqa: BLE001
        if sample.count("\t") > sample.count(","):
            delim, note = "\t", "tab (counted, sniffer declined)"
    rows = [r for r in csv.reader(io.StringIO(text), delimiter=delim) if r]
    if not rows:
        return [], [], f"{name}: empty"
    header, body = rows[0], rows[1:]
    #  a units row is a second line that is non-numeric while the third is
    #  numeric — the AiM/MoTeC habit. Dropped, and said so.
    def _numericish(r: Sequence[str]) -> bool:
        ok = 0
        for c in r[:12]:
            try:
                float(str(c).strip())
                ok += 1
            except ValueError:
                pass
        return ok >= max(1, len(r[:12]) // 2)
    if len(body) >= 2 and not _numericish(body[0]) and _numericish(body[1]):
        note += "; second row looked like units and was dropped"
        body = body[1:]
    return header, body, f"{name}: {note}, {len(body)} data rows"


def _as_float_col(body: list[list[str]], idx: int) -> np.ndarray:
    out = np.full(len(body), np.nan, dtype=float)
    for i, row in enumerate(body):
        if idx < len(row):
            try:
                out[i] = float(str(row[idx]).strip().replace(",", ""))
            except ValueError:
                pass
    return out


def sniff_files(files: Iterable[tuple[str, bytes]] | None) -> DataBundle:
    """Turn an arbitrary upload into canonical channels — or say why not.

    Accepts (name, bytes) pairs. CSV/TSV become channels; a JSON file holding
    a hardpoint dict becomes the geometry the kinematics jobs run on. Anything
    unrecognised is listed, not swallowed.
    """
    db = DataBundle()
    for name, blob in list(files or []):
        db.files.append(name)
        low = name.lower()
        try:
            if low.endswith(".json"):
                obj = json.loads(blob.decode("utf-8", errors="replace"))
                hp = _hardpoints_from_json(obj)
                if hp is not None:
                    db.hardpoints = hp
                    db.receipts.append(f"{name}: read as a hardpoint set")
                else:
                    db.receipts.append(
                        f"{name}: JSON read, but no hardpoint keys found "
                        f"— ignored")
                continue
            if low.endswith((".kicad_pcb", ".pcbdoc", ".pcb")):
                text = blob.decode("utf-8", errors="replace")
                # Identify by content, not extension: an Altium ASCII export
                # renamed to .txt is still an Altium board, and a *binary*
                # .PcbDoc is not readable at all — say which, rather than
                # storing bytes the PCB job will choke on later.
                try:
                    from .pcb_doctor import sniff_format as _sniff
                    kind = _sniff(blob, name)
                except Exception:                            # noqa: BLE001
                    kind = "unknown"
                if kind == "altium_binary":
                    db.receipts.append(
                        f"{name}: native binary Altium .PcbDoc — not readable. "
                        f"Re-save it from Altium as 'PCB ASCII File' and "
                        f"re-drop it.")
                    continue
                if kind == "unknown":
                    db.receipts.append(
                        f"{name}: not recognised as a KiCad or Altium ASCII "
                        f"board — ignored")
                    continue
                db.extras["board_file"] = text
                db.receipts.append(
                    f"{name}: read as "
                    f"{'a KiCad' if kind == 'kicad' else 'an Altium ASCII'} "
                    f"board")
                continue
            if low.endswith((".csv", ".tsv", ".txt", ".dat", ".log")):
                header, body, note = _read_table(name, blob)
                db.receipts.append(note)
                if not header or not body:
                    continue
                for idx, col in enumerate(header):
                    canon = _match_channel(col)
                    if canon is None or canon in db.series:
                        if canon is None:
                            db.unmatched.append(col.strip() or f"col{idx}")
                        continue
                    arr = _as_float_col(body, idx)
                    if not np.isfinite(arr).any():
                        db.unmatched.append(col.strip() or f"col{idx}")
                        continue
                    arr, scale_note = _rescale(canon, arr)
                    label, unit, _al = _CHANNELS[canon]
                    fin = arr[np.isfinite(arr)]
                    db.series[canon] = arr
                    db.channels[canon] = Channel(
                        canon=canon, label=label, unit=unit,
                        source_column=col.strip(), source_file=name,
                        n=int(arr.size),
                        n_finite=int(fin.size),
                        vmin=float(np.min(fin)), vmax=float(np.max(fin)),
                        mean=float(np.mean(fin)), std=float(np.std(fin)),
                        scale_note=scale_note, flags=_flag(canon, arr))
                continue
            db.receipts.append(f"{name}: extension not handled — ignored")
        except Exception as err:                             # noqa: BLE001
            db.warnings.append(f"{name}: could not be read ({type(err).__name__}"
                               f": {err}) — skipped, the run continues")

    # ---- one timebase per bundle ------------------------------------------- #
    #  Channels from two files with different row counts cannot share a mask,
    #  and merging them into one flat namespace produced an IndexError the
    #  first time anyone dropped two logs at once. The express lane runs ONE
    #  timebase per bundle: the file contributing the most recognised channels
    #  wins (ties broken by name, so the choice is deterministic), and
    #  channels from any other length are unmerged and SAID SO rather than
    #  silently kept to blow up three jobs later.
    if db.channels:
        by_file: dict[str, list[str]] = {}
        for canon, ch in db.channels.items():
            by_file.setdefault(ch.source_file, []).append(canon)
        #  The primary MUST be whichever file supplied the timebase. Picking
        #  by channel count instead let a second file win and take the `time`
        #  channel down with it, leaving a bundle with more channels than the
        #  first attempt and no clock to read them against.
        if "time" in db.channels:
            primary = db.channels["time"].source_file
        else:
            primary = sorted(by_file, key=lambda f: (-len(by_file[f]), f))[0]
        n_primary = db.channels.get(
            "time", db.channels[by_file[primary][0]]).n
        dropped: dict[str, list[str]] = {}
        for canon in list(db.channels):
            ch = db.channels[canon]
            if ch.source_file != primary and ch.n != n_primary:
                dropped.setdefault(ch.source_file, []).append(ch.label)
                del db.channels[canon]
                db.series.pop(canon, None)
        for fname, labels in sorted(dropped.items()):
            db.warnings.append(
                f"{fname}: {len(labels)} channel(s) not merged — that file "
                f"has a different row count than {primary}, and one bundle "
                f"runs one timebase. Drop them separately to analyse both: "
                + ", ".join(sorted(labels)[:8]))
        if len(by_file) > 1:
            db.receipts.append(
                f"timebase taken from {primary} ({n_primary} rows) — the "
                f"file that supplied the clock; every other channel must "
                f"share it")

    if "time" in db.series:
        t = db.series["time"]
        fin = t[np.isfinite(t)]
        if fin.size > 2:
            dt = np.diff(fin)
            dt = dt[dt > 0]
            if dt.size:
                med = float(np.median(dt))
                db.sample_rate_hz = 1.0 / med if med > 0 else None
                db.duration_s = float(fin[-1] - fin[0])
                jit = float(np.percentile(dt, 95) / med) if med > 0 else 1.0
                if jit > 1.5:
                    db.warnings.append(
                        f"timebase jitter: the 95th-percentile sample gap is "
                        f"{jit:.1f}× the median — any rate-based number below "
                        f"is a screening value")
    elif db.series:
        db.warnings.append(
            "no time column recognised — every time-domain job is skipped; "
            "name the column 'time' and re-drop to unlock them")
    for c in db.channels.values():
        for f in c.flags:
            if "FLATLINED" in f:
                db.warnings.append(f"{c.label}: {f}")
    return db


def _hardpoints_from_json(obj) -> object | None:
    """A hardpoint dict from any of the shapes the toolkit exports."""
    try:
        from .kinematics import Hardpoints
    except Exception:                                        # noqa: BLE001
        return None
    if not isinstance(obj, dict):
        return None
    for candidate in (obj, obj.get("hardpoints"), obj.get("hp")):
        if not isinstance(candidate, dict):
            continue
        if "upper_front_inner" not in candidate:
            continue
        try:
            return Hardpoints.from_dict(candidate)
        except Exception:                                    # noqa: BLE001
            try:
                kw = {k: (np.asarray(v, float)
                          if isinstance(v, (list, tuple)) else v)
                      for k, v in candidate.items()}
                return Hardpoints(**kw)
            except Exception:                                # noqa: BLE001
                continue
    return None


# =========================================================================== #
#  3 · Artifacts and the job registry
# =========================================================================== #
@dataclass
class Artifact:
    """One file that will land in the ZIP."""
    path: str
    data: bytes
    kind: str = "md"
    note: str = ""


@dataclass
class Ctx:
    """What a job is handed. Everything it needs, nothing global.

    `results` is the one channel between jobs: a job that another depends on
    stashes its output under its own jid. It is deliberately not a general
    blackboard — a job reading a key it did not declare in `needs_jobs` is a
    hidden edge, and hidden edges are how a plan stops being a plan.
    """
    ask: Ask
    data: DataBundle
    hardpoints: object
    hp_source: str
    results: dict[str, object] = _dcfield(default_factory=dict)
    #  Structured findings, raised by any job, harvested by the DFMEA job.
    #  This is the one piece of shared state that is deliberately open: a
    #  failure mode belongs to the car, not to the analysis that noticed it.
    findings: list[dict] = _dcfield(default_factory=list)
    #  Stamped by the runner so a finding knows which job raised it without
    #  every flag() call having to repeat itself.
    current_job: str = ""

    def flag(self, *, subsystem: str, item: str, mode: str, effect: str,
             severity: int, status: str, evidence: str, action: str,
             cause: str = "") -> None:
        """Raise a failure mode from inside a job.

        `status` is one of ok · watch · violation · unknown, and `evidence`
        is how the finding was reached — 'measured', 'modelled' or 'unchecked'
        — because that is precisely what DFMEA calls detectability.
        """
        self.findings.append(dict(
            subsystem=subsystem, item=item, mode=mode, effect=effect,
            cause=cause, severity=int(severity), status=status,
            evidence=evidence, action=action, source=self.current_job))


@dataclass
class Job:
    """A unit of work that declares its own preconditions."""
    jid: str
    title: str
    tool: str
    fn: Callable[[Ctx], list[Artifact]]
    needs_channels: tuple[str, ...] = ()   # ALL of these
    needs_any: tuple[str, ...] = ()        # and at least ONE of these
    needs_extra: tuple[str, ...] = ()      # non-tabular uploads, by kind
    needs_jobs: tuple[str, ...] = ()       # other jobs whose output it uses
    data_activated: bool = False           # runs on data alone, unasked
    cost_s: float = 0.5                    # DECLARED, measured once, not timed
    runs_last: bool = False                # harvests what other jobs raised
    note: str = ""

    @property
    def tier(self) -> str:
        """fast · slow · deep. The thresholds are the lane's, not physics'."""
        if self.cost_s < 2.0:
            return "fast"
        return "slow" if self.cost_s < 30.0 else "deep"


JOBS: dict[str, Job] = {}


def register_job(job: Job) -> Job:
    JOBS[job.jid] = job
    return job


@dataclass
class PlannedJob:
    job: Job
    reason: str                     # why it is in the plan
    skipped: str | None = None   # the named reason it CANNOT run
    deferred: str | None = None  # or the named reason it WOULD NOT FIT


def _order_by_dependency(planned: list[PlannedJob]) -> list[PlannedJob]:
    """Stable topological sort: dependencies before dependents, cost order
    preserved everywhere it does not conflict. Cycles are impossible to
    express usefully here, so one is left in place and reported by the runner
    rather than silently reordered."""
    runnable = [p for p in planned if p.skipped is None and p.deferred is None]
    other = [p for p in planned if p.skipped is not None or p.deferred is not None]
    #  A harvester must see every finding, so it is pulled out of the
    #  dependency graph and appended after it. Expressing that as a
    #  dependency on all 34 other jobs would be true and useless.
    last = [p for p in runnable if p.job.runs_last]
    runnable = [p for p in runnable if not p.job.runs_last]
    done: set = set()
    ordered: list[PlannedJob] = []
    pending = list(runnable)
    while pending:
        progressed = False
        for p in list(pending):
            if all(d in done for d in p.job.needs_jobs):
                ordered.append(p)
                done.add(p.job.jid)
                pending.remove(p)
                progressed = True
        if not progressed:                    # a cycle: keep source order
            ordered.extend(pending)
            break
    return ordered + last + other


def plan(ask: Ask, data: DataBundle,
         budget_s: float | None = None) -> list[PlannedJob]:
    """Which jobs run, which cannot, which would not fit — and why, by name.

    Three doors into the plan: the SENTENCE asked for the tool, the DATA
    satisfies the job outright, or — for the expensive engines — both, plus
    room in the time budget. A dropped log with steering and lateral-g gets
    the event finder whether or not anyone typed 'telemetry'; a thirty-second
    optimiser does not get to ambush someone who asked for bump steer.

    ADMISSION IS COMPUTED FROM DECLARED COSTS, NEVER FROM THE CLOCK. That is
    deliberate and it is the whole reason the budget works at all: if the
    runner dropped jobs when it noticed it was running late, the same request
    on a loaded laptop would produce a different ZIP than on an idle one, and
    the bundle would stop being citable. So the plan is a pure function of
    (ask, data, budget) and stays byte-deterministic; an overrun is REPORTED
    afterwards, never acted on. The cost of that choice is that a bad
    estimate overruns silently — which is why `ExpressRun` carries the
    measured time per job for the UI, so the estimates can be corrected in
    source rather than guessed at forever.
    """
    out: list[PlannedJob] = []
    asked = set(ask.tools)
    budget = float(ask.budget_s if budget_s is None else budget_s)
    committed = 0.0
    admitted: set = set()

    #  Cheapest first within each tier, so a small budget buys the most work,
    #  and jid last so the order never depends on dict insertion.
    _TIER_RANK = {"fast": 0, "slow": 1, "deep": 2}
    for jid in sorted(JOBS, key=lambda k: (_TIER_RANK[JOBS[k].tier],
                                           JOBS[k].cost_s, k)):
        job = JOBS[jid]
        by_ask = job.tool in asked
        satisfied = ((not job.needs_channels or data.has(*job.needs_channels))
                     and (not job.needs_any
                          or any(c in data.series for c in job.needs_any))
                     and all(k in data.extras for k in job.needs_extra))
        by_data = bool(job.needs_channels or job.needs_any
                       or job.needs_extra) and satisfied
        if not (by_ask or (job.data_activated and by_data)):
            continue
        missing = [c for c in job.needs_channels if c not in data.series]
        why = []
        if missing:
            why.append("needs " + ", ".join(
                _CHANNELS[c][0] if c in _CHANNELS else c for c in missing)
                + " — not found in the upload")
        if job.needs_any and not any(c in data.series
                                     for c in job.needs_any):
            why.append("needs at least one of " + ", ".join(
                _CHANNELS[c][0] if c in _CHANNELS else c
                for c in job.needs_any) + " — none is in the upload")
        missing_x = [k for k in job.needs_extra if k not in data.extras]
        if missing_x:
            why.append("needs an upload of kind "
                       + ", ".join(missing_x) + " — none was given")
        #  A dependency that is not itself running makes this job impossible,
        #  and the reason must point AT the dependency — "morph needs the
        #  ghost audit, and the ghost audit needs a log" is a chain the member
        #  can act on; "morph could not run" is not.
        blocked = [d for d in job.needs_jobs if d not in admitted]
        if blocked:
            why.append("needs " + ", ".join(
                JOBS[d].title if d in JOBS else d for d in blocked)
                + " to run first, and it is not in this plan")
        if why:
            out.append(PlannedJob(
                job, "asked for by name",
                skipped="; ".join(why)))
            continue

        #  The expensive tiers must be asked for. A thirty-second optimiser
        #  that fires because a log happened to contain a steering trace is
        #  not a feature, it is a surprise.
        if job.tier == "deep" and not by_ask:
            continue
        if job.tier != "fast":
            if committed + job.cost_s > budget:
                out.append(PlannedJob(
                    job, "asked for by name" if by_ask else "data-activated",
                    deferred=(
                        f"~{job.cost_s:g} s of work, and {committed:.0f} s of "
                        f"your {budget:g} s budget is already committed. Say "
                        f"'I have five minutes' and it runs, or open the tab "
                        f"and give it the attention it wants.")))
                continue
            committed += job.cost_s
        else:
            committed += job.cost_s

        admitted.add(jid)
        out.append(PlannedJob(
            job, "asked for by name" if by_ask else
                 "activated by the data you dropped"))

    #  Cost order is what admission wants; dependency order is what execution
    #  needs. A stable topological pass over the admitted set gives both.
    out = _order_by_dependency(out)
    if not any(p.skipped is None and p.deferred is None for p in out):
        # Nothing survived. The geometry baseline always can — it needs no
        # data at all — so the member never gets an empty ZIP.
        base = JOBS.get("geometry_baseline")
        if base is not None:
            out.insert(0, PlannedJob(
                base, "the fallback — nothing else could run, and an empty "
                      "bundle helps nobody"))
    return out


# =========================================================================== #
#  4 · The jobs themselves — real engines, screening fidelity, printed
# =========================================================================== #
def _md(title: str, lines: Sequence[str]) -> bytes:
    body = "\n".join(lines).rstrip() + "\n"
    return (f"# {title}\n\n{body}").encode()


def _csv_bytes(header: Sequence[str], rows: Iterable[Sequence]) -> bytes:
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(header)
    for r in rows:
        w.writerow(["" if v is None else
                    (f"{v:.6g}" if isinstance(v, float) else v) for v in r])
    return buf.getvalue().encode("utf-8")


def _fnum(v, fmt="{:.4g}") -> str:
    try:
        return "—" if v is None or not np.isfinite(float(v)) else fmt.format(float(v))
    except Exception:                                        # noqa: BLE001
        return "—"


# --- geometry baseline ------------------------------------------------------ #
def _job_geometry(ctx: Ctx) -> list[Artifact]:
    from .kinematics import SuspensionKinematics
    kin = SuspensionKinematics(ctx.hardpoints)
    tmax = float(ctx.ask.param("travel_mm", 25.0))
    n = 41
    states = kin.sweep(-tmax, tmax, n)
    rows = [(float(s.travel), float(s.camber), float(s.toe),
             float(s.caster), float(s.kpi), float(s.scrub_radius),
             float(s.roll_center_height), bool(s.converged))
            for s in states]
    trav = np.array([r[0] for r in rows])
    camb = np.array([r[1] for r in rows])
    toe = np.array([r[2] for r in rows])
    #  gradients about ride height, by central difference over ±10 mm
    def _grad(y):
        i0 = int(np.argmin(np.abs(trav + 10.0)))
        i1 = int(np.argmin(np.abs(trav - 10.0)))
        return (y[i1] - y[i0]) / (trav[i1] - trav[i0]) * 10.0
    cam_grad, toe_grad = _grad(camb), _grad(toe)
    n_bad = sum(1 for r in rows if not r[7])

    md = _md("Geometry baseline — camber, toe, caster, KPI, scrub", [
        f"Hardpoints: **{ctx.hp_source}**.",
        f"Sweep: ±{tmax:g} mm wheel travel, {n} stations.",
        "",
        "| quantity | at ride | gradient per 10 mm bump |",
        "|---|---|---|",
        f"| camber (deg) | {_fnum(np.interp(0.0, trav, camb))} | "
        f"{_fnum(cam_grad)} |",
        f"| toe (deg) | {_fnum(np.interp(0.0, trav, toe))} | "
        f"{_fnum(toe_grad)} |",
        "",
        f"**Bump steer** is the toe gradient: **{toe_grad:+.4f} deg per 10 mm "
        f"of bump**. Anything past roughly ±0.05 deg/10 mm is worth a "
        "tie-rod-height iteration before you spend tunnel or rig time on it.",
        "",
        f"**Camber gain** is **{cam_grad:+.4f} deg per 10 mm**.",
        "",
        (f"⚠️ {n_bad} of {n} stations did not converge — treat the ends of "
         f"the sweep as indicative." if n_bad else
         "All stations converged."),
        "",
        "---",
        "*Screening fidelity: 41-station sweep, default solver seeds. Open "
        "the 📐 Kinematics tab to drag the outer tie-rod pickup and watch "
        "bump steer zero itself live.*",
    ])
    csvb = _csv_bytes(
        ["travel_mm", "camber_deg", "toe_deg", "caster_deg", "kpi_deg",
         "scrub_radius_mm", "roll_centre_height_mm", "converged"], rows)
    return [
        Artifact("kinematics/geometry_baseline.md", md, "md"),
        Artifact("kinematics/geometry_sweep.csv", csvb, "csv"),
    ]


register_job(Job("geometry_baseline", "Geometry baseline sweep", "kinematics",
                 _job_geometry,
                 note="Runs on the default corner when no hardpoints are "
                      "uploaded — so the bundle is never empty."))


# --- motion ratio / rates --------------------------------------------------- #
def _job_rates(ctx: Ctx) -> list[Artifact]:
    from .kinematics import SuspensionKinematics
    from .damper import default_damper, damping_ratio
    kin = SuspensionKinematics(ctx.hardpoints)
    k_spring = float(ctx.ask.param("spring_rate_N_per_mm", 35.0))
    mass = float(ctx.ask.param("mass_kg", 280.0))
    wdf = float(ctx.ask.param("weight_dist_front", 0.47))
    corner_mass = mass * wdf / 2.0

    mr = float(kin.motion_ratio())
    mr_real = bool(kin.motion_ratio_is_real())
    wheel_rate = float(kin.wheel_rate(k_spring))
    f_ride = (math.sqrt(wheel_rate * 1000.0 / corner_mass) / (2 * math.pi)
              if corner_mass > 0 and wheel_rate > 0 else float("nan"))
    curve = kin.motion_ratio_curve(-25.0, 25.0, 21)

    zeta_b = damping_ratio(default_damper(), corner_mass, wheel_rate, mr,
                           region="bump")
    zeta_r = damping_ratio(default_damper(), corner_mass, wheel_rate, mr,
                           region="rebound")

    mr_note = ("(solved from the linkage)" if mr_real else
               "(**assumed** — this corner has no rocker in the model, so "
               "the ratio is a placeholder and every number below inherits "
               "that assumption)")
    md = _md("Rates — motion ratio, wheel rate, ride frequency, damping", [
        f"Spring rate **{k_spring:g} N/mm** · corner mass **"
        f"{corner_mass:.1f} kg** (from {mass:g} kg at {wdf:.0%} front).",
        "",
        "| quantity | value |",
        "|---|---|",
        f"| motion ratio | {_fnum(mr)} {mr_note} |",
        f"| wheel rate | {_fnum(wheel_rate)} N/mm |",
        f"| ride frequency | {_fnum(f_ride)} Hz |",
        f"| damping ratio, low-speed bump | {_fnum(zeta_b)} |",
        f"| damping ratio, low-speed rebound | {_fnum(zeta_r)} |",
        "",
        "Damping ratios come from the **representative** damper curve, not "
        "your dyno sheet. Load a real curve in the 🎛️ Setup Optimiser before "
        "you quote these to a judge.",
        "",
        "*Ride frequency is the undamped single-mass value "
        "`f = √(k_wheel/m)/2π` — no tyre spring in series.*",
    ])
    csvb = _csv_bytes(["travel_mm", "motion_ratio"],
                      [(float(t), float(v)) for t, v in zip(*curve)]
                      if isinstance(curve, tuple) else
                      [(float(a), float(b)) for a, b in curve])
    return [Artifact("setup/rates.md", md, "md"),
            Artifact("setup/motion_ratio_curve.csv", csvb, "csv")]


register_job(Job("rates", "Motion ratio, wheel rate, damping", "setup",
                 _job_rates))


# --- load transfer ---------------------------------------------------------- #
def _job_load_transfer(ctx: Ctx) -> list[Artifact]:
    from .kinematics import SuspensionKinematics
    from .dynamics import VehicleDynamics, VehicleParams
    kin = SuspensionKinematics(ctx.hardpoints)
    p = VehicleParams(
        mass=float(ctx.ask.param("mass_kg", 280.0)),
        cg_height=float(ctx.ask.param("cg_height_mm", 300.0)),
        wheelbase=float(ctx.ask.param("wheelbase_mm", 1550.0)),
        track_front=float(ctx.ask.param("track_front_mm", 1200.0)),
        track_rear=float(ctx.ask.param("track_rear_mm", 1180.0)),
        weight_dist_front=float(ctx.ask.param("weight_dist_front", 0.47)),
        mu_peak=float(ctx.ask.param("mu_peak", 1.55)))
    vd = VehicleDynamics(p, front_kin=kin, rear_kin=kin)
    g_lat = float(ctx.ask.param("lateral_g", 1.4))
    g_lon = float(ctx.ask.param("long_g", 1.5))

    loads, detail = vd.lateral_load_transfer(g_lat)
    #  the longitudinal solver returns the AXLE transfer (N), not corners —
    #  the corner split is done here, evenly across each axle, and said so.
    dW, _lon_info = vd.longitudinal_load_transfer(g_lon)
    dW = float(dW)
    w_static = p.mass * p.g
    f_axle = w_static * p.weight_dist_front
    r_axle = w_static * (1.0 - p.weight_dist_front)
    lon_f = (f_axle + dW) / 2.0
    lon_r = (r_axle - dW) / 2.0
    rows = [("lateral", g_lat, loads.fl, loads.fr, loads.rl, loads.rr),
            ("longitudinal", g_lon, lon_f, lon_f, lon_r, lon_r)]

    ltd_f = float(detail.get("ltd_front", float("nan")))
    ltd_r = float(detail.get("ltd_rear", float("nan")))
    total = ltd_f + ltd_r
    tlltd = ltd_f / total if total else float("nan")

    md = _md("Load transfer and balance", [
        f"At **{g_lat:g} g** lateral, **{g_lon:g} g** longitudinal.",
        "",
        "| corner | lateral (N) | longitudinal (N) |",
        "|---|---|---|",
        f"| FL | {_fnum(loads.fl)} | {_fnum(lon_f)} |",
        f"| FR | {_fnum(loads.fr)} | {_fnum(lon_f)} |",
        f"| RL | {_fnum(loads.rl)} | {_fnum(lon_r)} |",
        f"| RR | {_fnum(loads.rr)} | {_fnum(lon_r)} |",
        "",
        f"Longitudinal: **{_fnum(dW)} N** transferred front↔rear at "
        f"{g_lon:g} g, split evenly across each axle (point-mass; no "
        "anti-dive geometry in this number).",
        "",
        f"- Roll angle: **{_fnum(detail.get('roll_angle'))} deg**",
        f"- Roll-centre height, front / rear: "
        f"{_fnum(detail.get('rc_front'))} / {_fnum(detail.get('rc_rear'))} mm",
        f"- Lateral load-transfer distribution (**TLLTD**): "
        f"**{tlltd:.1%} front**",
        "",
        "TLLTD is the single number that moves balance fastest. Above about "
        "50 % front the car understeers at the limit; the ARB rates in the "
        "🎚️ Roll tab are the cheapest lever on it.",
        "",
        "*Quasi-static: no transient, no tyre relaxation, no aero. "
        "For the transient picture, promote this to the ⚡🔩 SimulForge tab.*",
    ])
    csvb = _csv_bytes(["case", "g", "FL_N", "FR_N", "RL_N", "RR_N"], rows)
    return [Artifact("roll/load_transfer.md", md, "md"),
            Artifact("roll/load_transfer.csv", csvb, "csv")]


register_job(Job("load_transfer", "Lateral + longitudinal load transfer",
                 "roll", _job_load_transfer))


# --- roll-centre migration --------------------------------------------------- #
def _job_rc_migration(ctx: Ctx) -> list[Artifact]:
    from .kinematics import SuspensionKinematics
    from .dynamics import VehicleDynamics, VehicleParams
    kin = SuspensionKinematics(ctx.hardpoints)
    p = VehicleParams(track_front=float(ctx.ask.param("track_front_mm", 1200.0)))
    vd = VehicleDynamics(p, front_kin=kin, rear_kin=kin)
    res = vd.roll_center_migration(kin, p.track_front, -25.0, 25.0, 21)
    if isinstance(res, tuple) and len(res) == 2:
        trav, rc = np.asarray(res[0], float), np.asarray(res[1], float)
    else:
        arr = np.asarray(res, float)
        trav, rc = arr[:, 0], arr[:, 1]
    fin = np.isfinite(rc)
    swing = (float(np.nanmax(rc[fin]) - np.nanmin(rc[fin]))
             if fin.any() else float("nan"))
    md = _md("Roll-centre migration", [
        f"Roll-centre height across ±25 mm of travel, {trav.size} stations.",
        "",
        f"- Height at ride: **{_fnum(np.interp(0.0, trav, rc))} mm**",
        f"- Total migration across the sweep: **{_fnum(swing)} mm**",
        "",
        "A roll centre that migrates more than roughly its own height across "
        "the working travel makes the car's balance travel-dependent — the "
        "setup that works on a smooth skidpad stops working over kerbs.",
        "",
        "*Kinematic roll centre from the instant-centre construction; it is a "
        "geometric artefact, not a force-based one.*",
    ])
    csvb = _csv_bytes(["travel_mm", "roll_centre_height_mm"],
                      [(float(a), float(b)) for a, b in zip(trav, rc)])
    return [Artifact("roll/rc_migration.md", md, "md"),
            Artifact("roll/rc_migration.csv", csvb, "csv")]


register_job(Job("rc_migration", "Roll-centre migration", "roll",
                 _job_rc_migration))


# --- telemetry summary ------------------------------------------------------- #
def _job_telemetry(ctx: Ctx) -> list[Artifact]:
    db = ctx.data
    rows = []
    for canon, ch in sorted(db.channels.items()):
        rows.append((ch.canon, ch.label, ch.unit, ch.source_column, ch.n,
                     ch.n_finite, ch.vmin, ch.vmax, ch.mean, ch.std,
                     "; ".join(ch.flags)))
    lines = [
        f"Files read: {', '.join(db.files) or '—'}",
        f"Recognised channels: **{len(db.channels)}**"
        f" · unmatched columns: {len(db.unmatched)}",
        (f"Sample rate: **{_fnum(db.sample_rate_hz)} Hz** · duration "
         f"**{_fnum(db.duration_s)} s**" if db.sample_rate_hz
         else "No usable timebase."),
        "",
        "| channel | unit | from column | min | max | mean | std | flags |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for _c, lab, unit, src, _n, _nf, vmin, vmax, mean, std, flags in rows:
        lines.append(f"| {lab} | {unit} | `{src}` | {_fnum(vmin)} | "
                     f"{_fnum(vmax)} | {_fnum(mean)} | {_fnum(std)} | "
                     f"{flags or '—'} |")
    scale = [f"- **{ch.label}**: {ch.scale_note}"
             for ch in db.channels.values() if ch.scale_note]
    if scale:
        lines += ["", "### Scale decisions the sniffer made", ""] + scale
    if db.unmatched:
        lines += ["", "### Columns not recognised", "",
                  "These were left alone. If one of them is a channel you "
                  "need, rename it to a name in the synonym table and "
                  "re-drop:", "",
                  ", ".join(f"`{c}`" for c in sorted(set(db.unmatched))[:60])]
    if db.warnings:
        lines += ["", "### Warnings", ""] + [f"- ⚠️ {w}" for w in db.warnings]
    csvb = _csv_bytes(
        ["channel", "label", "unit", "source_column", "n", "n_finite",
         "min", "max", "mean", "std", "flags"], rows)
    return [Artifact("daq/telemetry_summary.md",
                     _md("Telemetry summary", lines), "md"),
            Artifact("daq/channel_stats.csv", csvb, "csv")]


register_job(Job("telemetry", "Telemetry channel summary", "daq",
                 _job_telemetry, needs_channels=("time",),
                 data_activated=True))


# --- event finder ------------------------------------------------------------ #
def _segments(mask: np.ndarray, t: np.ndarray, min_s: float
              ) -> list[tuple[float, float, int, int]]:
    """Contiguous True runs of at least `min_s` seconds."""
    out = []
    i = 0
    n = mask.size
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            if t[j] - t[i] >= min_s:
                out.append((float(t[i]), float(t[j]), i, j))
            i = j + 1
        else:
            i += 1
    return out


def _job_events(ctx: Ctx) -> list[Artifact]:
    db = ctx.data
    t = db.series["time"]
    ok = np.isfinite(t)
    t = t[ok]
    rows = []
    thresholds = {"cornering": ("ay", 0.6, 0.30),
                  "braking": ("ax", 0.5, 0.25),
                  "acceleration": ("ax", 0.35, 0.25)}
    for name, (chan, thr, min_s) in thresholds.items():
        if chan not in db.series:
            continue
        y = db.series[chan][ok]
        if name == "braking":
            m = y < -thr
        elif name == "acceleration":
            m = y > thr
        else:
            m = np.abs(y) > thr
        m = np.nan_to_num(m, nan=False).astype(bool)
        for t0, t1, i0, i1 in _segments(m, t, min_s):
            seg = y[i0:i1 + 1]
            peak = float(np.nanmax(np.abs(seg)))
            spd = ""
            if "speed" in db.series:
                s = db.series["speed"][ok][i0:i1 + 1]
                if np.isfinite(s).any():
                    spd = float(np.nanmean(s))
            rows.append((name, round(t0, 3), round(t1, 3),
                         round(t1 - t0, 3), round(peak, 3), spd))
    rows.sort(key=lambda r: (r[1], r[0]))
    by_kind: dict[str, list] = {}
    for r in rows:
        by_kind.setdefault(r[0], []).append(r)

    lines = [f"Found **{len(rows)} events** in "
             f"{_fnum(db.duration_s)} s of logging.", "",
             "Thresholds are declared, not learned: cornering |ay| > 0.6 g "
             "for ≥ 0.30 s, braking ax < −0.5 g for ≥ 0.25 s, acceleration "
             "ax > 0.35 g for ≥ 0.25 s. Change them and the count changes — "
             "that is a threshold choice, not a discovery.", "",
             "| kind | count | longest (s) | peak (g) |", "|---|---|---|---|"]
    for kind, rs in sorted(by_kind.items()):
        lines.append(f"| {kind} | {len(rs)} | "
                     f"{max(r[3] for r in rs):.2f} | "
                     f"{max(r[4] for r in rs):.2f} |")
    if not rows:
        lines += ["", "No segment cleared the thresholds. Either the log is "
                       "an installation lap, or the acceleration channels are "
                       "scaled differently than the sniffer inferred — check "
                       "the scale decisions in the telemetry summary."]
    csvb = _csv_bytes(["kind", "t_start_s", "t_end_s", "duration_s",
                       "peak_g", "mean_speed_ms"], rows)
    return [Artifact("daq/events.md", _md("Event finder", lines), "md"),
            Artifact("daq/events.csv", csvb, "csv")]


register_job(Job("events", "Braking / cornering event finder", "daq",
                 _job_events, needs_channels=("time", "ay"),
                 data_activated=True))


# --- measured vs model ------------------------------------------------------- #
def _job_measured_vs_model(ctx: Ctx) -> list[Artifact]:
    from .kinematics import SuspensionKinematics
    from .dynamics import VehicleDynamics, VehicleParams
    db = ctx.data
    ay = db.series["ay"]
    fin = ay[np.isfinite(ay)]
    #  the 99.5th percentile, not the max — one spike is a bump, not a limit
    meas = float(np.percentile(np.abs(fin), 99.5)) if fin.size else float("nan")
    kin = SuspensionKinematics(ctx.hardpoints)
    p = VehicleParams(
        mass=float(ctx.ask.param("mass_kg", 280.0)),
        cg_height=float(ctx.ask.param("cg_height_mm", 300.0)),
        mu_peak=float(ctx.ask.param("mu_peak", 1.55)))
    vd = VehicleDynamics(p, front_kin=kin, rear_kin=kin)
    try:
        model = vd.max_lateral_g()
        model = float(model[0] if isinstance(model, tuple) else model)
    except Exception as err:                                 # noqa: BLE001
        model = float("nan")
    ratio = meas / model if model and np.isfinite(model) and model else float("nan")
    if not np.isfinite(ratio):
        verdict = "The model could not produce a limit to compare against."
    elif ratio < 0.80:
        verdict = ("The car is leaving more than 20 % of the model's grip on "
                   "the table. Before blaming the model, check the obvious "
                   "three: driver confidence, tyre pressures/temps, and "
                   "whether the log contains a genuine limit corner at all.")
    elif ratio > 1.10:
        verdict = ("Measured exceeds the model by more than 10 %. Either "
                   "μ_peak is set conservatively, or the accelerometer is "
                   "reading roll-induced gravity component — check the "
                   "sensor's mounting angle before celebrating.")
    else:
        verdict = ("Measured and modelled agree within 10 %. That is a "
                   "correlated model — the number to quote in a design "
                   "review, with this comparison as the evidence.")
    lines = [
        "| source | peak lateral (g) |", "|---|---|",
        f"| measured, 99.5th percentile | **{_fnum(meas)}** |",
        f"| model, `max_lateral_g()` | **{_fnum(model)}** |",
        f"| ratio measured/model | **{_fnum(ratio, '{:.2f}')}** |", "",
        verdict, "",
        f"*Model inputs: {p.mass:g} kg, cg {p.cg_height:g} mm, "
        f"μ_peak {p.mu_peak:g}. Change any of them in your sentence and this "
        "comparison moves — which is exactly why the inputs are printed.*",
    ]
    return [Artifact("validation/measured_vs_model.md",
                     _md("Measured vs modelled lateral grip", lines), "md")]


register_job(Job("measured_vs_model", "Measured vs modelled grip",
                 "laptime", _job_measured_vs_model,
                 needs_channels=("ay",), data_activated=True))


# =========================================================================== #
#  5 · The runner
# =========================================================================== #
@dataclass
class ExpressRun:
    ask: Ask
    data: DataBundle
    planned: list[PlannedJob]
    artifacts: list[Artifact] = _dcfield(default_factory=list)
    ran: list[str] = _dcfield(default_factory=list)
    failed: list[tuple[str, str]] = _dcfield(default_factory=list)
    skipped: list[tuple[str, str]] = _dcfield(default_factory=list)
    deferred: list[tuple[str, str]] = _dcfield(default_factory=list)
    #  measured per job, for the UI and for correcting the DECLARED costs in
    #  source. Never enters the bundle — see the note in plan().
    timings: dict[str, float] = _dcfield(default_factory=dict)
    elapsed_s: float = 0.0
    hp_source: str = ""

    def manifest(self) -> dict:
        return {
            "generator": "KinematiK Express Lane",
            "request": self.ask.summary(),
            "data": self.data.summary(),
            "hardpoints": self.hp_source,
            "jobs_run": self.ran,
            "jobs_failed": [{"job": j, "error": e} for j, e in self.failed],
            "jobs_skipped": [{"job": j, "reason": r}
                             for j, r in self.skipped],
            "jobs_deferred": [{"job": j, "reason": r}
                              for j, r in self.deferred],
            "budget_s": self.ask.budget_s,
            "files": [a.path for a in self.artifacts],
            #  Deliberately NOT the wall clock. A design-review artifact that
            #  differs between two identical runs cannot be cited, and a
            #  timestamp is the classic way to lose that property for nothing.
            #  Timing lives on the run object for the UI to show.
        }


def run_express(text: str,
                files: Iterable[tuple[str, bytes]] | None = None,
                *,
                hardpoints: object | None = None,
                budget_s: float | None = None,
                progress: Callable[[str], None] | None = None
                ) -> ExpressRun:
    """Parse → sniff → plan → run. Never raises; failures become artifacts."""
    t0 = time.time()
    say = progress or (lambda _s: None)

    say("Reading the request…")
    ask = parse_request(text)

    say("Sniffing the upload…")
    data = sniff_files(files)

    hp, hp_src = data.hardpoints, "uploaded hardpoint set"
    if hp is None and hardpoints is not None:
        hp, hp_src = hardpoints, "live hardpoints from the Kinematics tab"
    if hp is None:
        from .kinematics import Hardpoints
        hp, hp_src = Hardpoints.default(), \
            "default FSAE front corner (nothing uploaded, nothing live)"

    say("Planning…")
    planned = plan(ask, data, budget_s)
    run = ExpressRun(ask=ask, data=data, planned=planned, hp_source=hp_src)
    ctx = Ctx(ask=ask, data=data, hardpoints=hp, hp_source=hp_src)

    for p in planned:
        if p.skipped:
            run.skipped.append((p.job.jid, p.skipped))
            continue
        if p.deferred:
            run.deferred.append((p.job.jid, p.deferred))
            continue
        unmet = [d for d in p.job.needs_jobs if d not in run.ran]
        if unmet:
            run.skipped.append((p.job.jid, "depends on " + ", ".join(
                JOBS[d].title if d in JOBS else d for d in unmet)
                + ", which did not complete"))
            continue
        say(f"Running {p.job.title}…"
            + (f" (~{p.job.cost_s:g} s)" if p.job.tier != "fast" else ""))
        t_job = time.time()
        ctx.current_job = p.job.jid
        try:
            arts = p.job.fn(ctx) or []
            run.artifacts.extend(arts)
            run.ran.append(p.job.jid)
            run.timings[p.job.jid] = time.time() - t_job
        except Exception as err:                             # noqa: BLE001
            tb = traceback.format_exc(limit=3)
            run.failed.append((p.job.jid, f"{type(err).__name__}: {err}"))
            run.artifacts.append(Artifact(
                f"_failed/{p.job.jid}.md",
                _md(f"FAILED — {p.job.title}", [
                    f"This job raised `{type(err).__name__}: {err}`.",
                    "", "The rest of the bundle is unaffected — that is why "
                    "you are still reading a report and not a stack trace.",
                    "", "```", tb.strip(), "```"]), "md"))

    run.elapsed_s = time.time() - t0
    say(f"Done in {run.elapsed_s:.1f} s — {len(run.artifacts)} files.")
    return run


# =========================================================================== #
#  6 · The bundle
# =========================================================================== #
_TAB_NAMES = {
    "kinematics": "📐 Kinematics", "roll": "🎚️ Roll & Load Transfer",
    "setup": "🎛️ Setup Optimiser", "daq": "📡 Data Acquisition",
    "laptime": "🏁 Track Testing", "brakes": "🛑 Brakes",
    "compliance": "🧬 Compliance (Flex)", "tire": "🛞 Tire & Grip",
    "ev": "⚡ EV Powertrain", "weight": "⚖️ Weight & Handover",
    "frames": "🧭 Frames & Datums", "setup": "🎛️ Setup Optimiser",
    "aero": "🪂 Aero", "dfmea": "🧯 DFMEA", "fusebox": "🔌 Fusebox",
    "pcb": "🩺 PCB Doctor", "electronics": "🔗 Electronics Integration",
    "thermal": "🌡️ Pack Thermal", "transient": "⚡🔩 SimulForge",
    "omnicore": "🧠 OmniCore", "genesis": "🧬 Inverse Genesis",
    "ghost": "👻 Ghost Topology", "rules": "📋 Rules Check",
    "cooling": "❄️ Cooling Loop", "printing": "🖨️ Printed Parts",
    "powertrain": "⚙️ Powertrain", "wiring": "🔌 Wiring & Ampacity",
    "morph": "🕸️ MorphMesh",
}


def render_readme(run: ExpressRun) -> bytes:
    ask, db = run.ask, run.data
    L: list[str] = []
    L.append("You asked for this in two sentences. Here is what was run, on "
             "what, and what every number is worth.")
    L.append("")
    L.append("> **Nothing in this pipeline is a language model.** The request "
             "was read by a keyword grammar and a nearest-quantity binder; "
             "the data by a delimiter sniffer and a synonym table. Both print "
             "their receipts below, including what they did not understand.")
    L.append("")
    L.append(f"**{len(run.artifacts) + 2} files** · {len(run.ran)} jobs run, "
             f"{len(run.deferred)} deferred, {len(run.skipped)} skipped, "
             f"{len(run.failed)} failed.")
    L.append("")
    L.append(f"Time budget: **{ask.budget_s:g} s**"
             + (f", from '{ask.budget_source}' in your sentence."
                if ask.budget_source else
                " (the lane default — nothing in your sentence set one)."))
    L.append("")

    L.append("## What you said")
    L.append("")
    L.append(f"> {ask.text or '(nothing typed — the data drove the whole plan)'}")
    L.append("")
    L.append("### The parse receipt")
    L.append("")
    for c in ask.consumed:
        L.append(f"- ✅ {c}")
    for a in ask.assumptions:
        L.append(f"- ➖ assumed: {a}")
    if ask.ignored:
        L.append(f"- 🕳️ not understood (a grammar, not a language model): "
                 f"{', '.join(ask.ignored)}")
    if not ask.consumed and not ask.assumptions:
        L.append("- (nothing to report — no text was given)")
    L.append("")

    L.append("### Parameters the run used")
    L.append("")
    L.append("| parameter | value | source |")
    L.append("|---|---|---|")
    for field, _dim, default, _w in _PARAM_WORDS:
        if default is None and field not in ask.params:
            continue
        v = ask.params.get(field, default)
        src = "**from your sentence**" if field in ask.params else "default"
        L.append(f"| `{field}` | {v:g} | {src} |")
    L.append("")
    L.append("Every default above is a declared constant, not a hidden one. "
             "If one is wrong for your car, put the right number in the "
             "sentence and re-run — the binder will pick it up.")
    L.append("")

    L.append("## What you dropped")
    L.append("")
    if db.files:
        for r in db.receipts:
            L.append(f"- {r}")
        L.append("")
        L.append(f"- Hardpoints used: **{run.hp_source}**")
        if db.sample_rate_hz:
            L.append(f"- Timebase: **{db.sample_rate_hz:.1f} Hz** over "
                     f"**{db.duration_s:.1f} s**")
        if db.channels:
            L.append(f"- Channels recognised: "
                     + ", ".join(f"`{c}`" for c in sorted(db.channels)))
        if db.unmatched:
            L.append(f"- Columns left unmatched: {len(db.unmatched)} "
                     f"(listed in `daq/telemetry_summary.md`)")
    else:
        L.append("No files were dropped, so every job ran on declared "
                 "defaults and the built-in FSAE front corner. The geometry "
                 "is real; the vehicle numbers are placeholders until you "
                 "give it yours.")
    for w in db.warnings:
        L.append(f"- ⚠️ {w}")
    L.append("")

    L.append("## What was run")
    L.append("")
    L.append("| job | tool | why it ran |")
    L.append("|---|---|---|")
    for p in run.planned:
        if p.skipped:
            continue
        L.append(f"| {p.job.title} | {_TAB_NAMES.get(p.job.tool, p.job.tool)} "
                 f"| {p.reason} |")
    L.append("")
    if run.deferred:
        L.append("### Deferred — would not fit the time budget")
        L.append("")
        for jid, reason in run.deferred:
            job = JOBS[jid]
            L.append(f"- **{job.title}** ({job.tier}, ~{job.cost_s:g} s) — "
                     f"{reason}")
        L.append("")
        L.append("Deferred is not skipped. Nothing is missing from these "
                 "jobs — they simply cost more than the lane promised you, "
                 "and quietly spending four minutes of your evening on a "
                 "job you did not price is not a favour. Re-run with a "
                 "bigger budget in the sentence, or take them into their "
                 "tabs where the wait comes with controls.")
        L.append("")
    blocked = [t for t in ask.tools if t in _NOT_IN_LANE]
    if blocked:
        L.append("### Asked for, and deliberately not in this lane")
        L.append("")
        for t in blocked:
            L.append(f"- **{_TAB_NAMES.get(t, t)}** — {_NOT_IN_LANE[t]}")
        L.append("")
    if run.skipped:
        L.append("### Skipped — and exactly why")
        L.append("")
        for jid, reason in run.skipped:
            L.append(f"- **{JOBS[jid].title}** — {reason}")
        L.append("")
        L.append("Each of those is one column or one number away from "
                 "running. Nothing was dropped silently.")
        L.append("")
    if run.failed:
        L.append("### Failed")
        L.append("")
        for jid, err in run.failed:
            L.append(f"- **{JOBS[jid].title}** — `{err}` "
                     f"(traceback in `_failed/{jid}.md`)")
        L.append("")

    L.append("## Read this before you quote any of it")
    L.append("")
    L.append("This bundle is a **screening artifact**. Every job ran at "
             "coarse, declared fidelity so it could finish while you waited. "
             "That makes it an excellent starting point for a design review "
             "and a poor place to stop:")
    L.append("")
    tools = sorted({p.job.tool for p in run.planned if not p.skipped})
    if tools:
        L.append("Take these past screening fidelity in their own tabs: "
                 + ", ".join(f"**{_TAB_NAMES.get(t, t)}**" for t in tools)
                 + ".")
        L.append("")
    if ask.deliverables and set(ask.deliverables) != {"md", "csv", "json"}:
        L.append("You asked specifically for "
                 + ", ".join(sorted(ask.deliverables))
                 + " — those are the files to open first. Everything else is "
                 "in here anyway, because deleting your data over an "
                 "adjective would be a strange thing for a toolkit to do.")
        L.append("")
    L.append("And the standing rule this toolkit exists for: do this here "
             "first so that ANSYS / MATLAB / ADAMS spend their time "
             "**validating** your design instead of debugging your inputs.")
    L.append("")
    L.append("## Files in this bundle")
    L.append("")
    for a in sorted(run.artifacts, key=lambda x: x.path):
        L.append(f"- `{a.path}`")
    L.append("- `manifest.json` — the whole run, machine-readable")
    L.append("")
    L.append("---")
    L.append("")
    L.append("*KinematiK Express Lane · deterministic: the same sentence and "
             "the same files produce a byte-identical ZIP, every time.*")
    return _md("Your KinematiK bundle", L)


def bundle_zip(run: ExpressRun) -> bytes:
    """A byte-deterministic ZIP: fixed timestamps, sorted entries.

    Determinism is not fussiness. Two members running the same request must
    get the same artifact, or the bundle cannot be cited in a design review.
    """
    arts = list(run.artifacts)
    arts.append(Artifact("README.md", render_readme(run), "md"))
    arts.append(Artifact(
        "manifest.json",
        json.dumps(run.manifest(), indent=2, sort_keys=True,
                   default=str).encode("utf-8"), "json"))

    #  The deliverable words ("report", "csv", "json") set EMPHASIS in the
    #  README — they do not filter the ZIP. A member who types "just give me
    #  a report" has not asked us to throw away their data, and a bundle whose
    #  contents silently depend on an adjective is a support ticket waiting to
    #  happen.
    keep = arts

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for a in sorted(keep, key=lambda x: x.path):
            zi = zipfile.ZipInfo(a.path, date_time=(1980, 1, 1, 0, 0, 0))
            zi.compress_type = zipfile.ZIP_DEFLATED
            zi.external_attr = 0o644 << 16
            z.writestr(zi, a.data)
    return buf.getvalue()


def express(text: str,
            files: Iterable[tuple[str, bytes]] | None = None,
            **kw) -> tuple[ExpressRun, bytes]:
    """The whole lane in one call: sentences in, (run, zip bytes) out."""
    run = run_express(text, files, **kw)
    return run, bundle_zip(run)


# =========================================================================== #
#  7 · The second tranche of jobs
# =========================================================================== #
#  Imported for its side effect: every job in it calls register_job at import
#  time. Kept at the bottom so the registry, Job and the helpers all exist
#  first, and wrapped so a broken job module degrades the lane to its core
#  jobs instead of taking the whole import down.
try:
    from . import express_jobs as _express_jobs      # noqa: F401
except Exception as _jobs_err:                       # noqa: BLE001
    import warnings as _warnings
    _warnings.warn(f"express_jobs failed to load ({_jobs_err}) — the express "
                   f"lane is running on its core jobs only", RuntimeWarning,
                   stacklevel=2)


# =========================================================================== #
#  8 · Self-test
# =========================================================================== #
def _demo_csv(n: int = 600) -> bytes:
    """A synthetic log with deliberately awkward headers and units — so the
    self-test exercises the sniffer's inference, not a happy path."""
    t = np.linspace(0.0, 30.0, n)
    ay = 1.35 * np.sin(2 * np.pi * t / 7.0) * 9.80665      # m/s², not g
    ax = -1.2 * np.clip(np.sin(2 * np.pi * t / 5.0 + 1.0), -1, 0) * 9.80665
    spd = (60.0 + 25.0 * np.cos(2 * np.pi * t / 7.0))       # km/h, not m/s
    steer = 40.0 * np.sin(2 * np.pi * t / 7.0)
    out = io.StringIO()
    w = csv.writer(out, lineterminator="\n")
    w.writerow(["Time", "Lat Accel [m/s2]", "Long Accel", "GPS Speed",
                "Steering Angle", "MysteryChannel"])
    w.writerow(["s", "m/s2", "m/s2", "kph", "deg", "-"])    # units row
    for i in range(n):
        w.writerow([f"{t[i]:.4f}", f"{ay[i]:.4f}", f"{ax[i]:.4f}",
                    f"{spd[i]:.3f}", f"{steer[i]:.3f}", "7"])
    return out.getvalue().encode("utf-8")


def _selftest() -> int:
    fails = 0

    def check(name, cond, detail=""):
        nonlocal fails
        print(f"  {'PASS' if cond else 'FAIL'}  {name}"
              + (f"   {detail}" if detail and not cond else ""))
        if not cond:
            fails += 1

    print("· parameter table")
    _probs = validate_param_table()
    check("ordering contract holds", not _probs, "; ".join(_probs[:3]))

    print("· grammar")
    a = parse_request(
        "Our 245 kg car with a cg height of 285 mm keeps understeering on "
        "the skidpad at 1.5 lateral g. Give us the roll and load transfer "
        "numbers and the bump steer, front track 1210 mm, 62% front bias.")
    check("mass bound", a.params.get("mass_kg") == 245.0, str(a.params))
    check("cg bound", a.params.get("cg_height_mm") == 285.0, str(a.params))
    check("front track bound (beats bare 'track')",
          a.params.get("track_front_mm") == 1210.0, str(a.params))
    check("lateral g bound", abs(a.params.get("lateral_g", 0) - 1.5) < 1e-9,
          str(a.params))
    check("bias read as a fraction",
          abs(a.params.get("brake_bias_front", 0) - 0.62) < 1e-9,
          str(a.params))
    check("roll tool asked", "roll" in a.tools, str(a.tools))
    check("kinematics tool asked", "kinematics" in a.tools, str(a.tools))
    check("deterministic", parse_request(a.text).params == a.params)

    b = parse_request("Car is 320 g heavier than target.")
    check("'g' beside a mass word reads as grams",
          abs(b.params.get("mass_kg", 0) - 0.320) < 1e-9, str(b.params))

    print("· sniffer")
    db = sniff_files([("run7.csv", _demo_csv())])
    check("time found", "time" in db.series)
    check("ay found", "ay" in db.series)
    check("ay rescaled to g", db.channels["ay"].vmax < 3.0,
          str(db.channels["ay"].vmax))
    check("speed rescaled to m/s", db.channels["speed"].mean < 30.0,
          str(db.channels["speed"].mean))
    check("units row dropped", db.channels["ay"].n_finite > 500,
          str(db.channels["ay"].n_finite))
    check("sample rate ~20 Hz",
          db.sample_rate_hz and 15 < db.sample_rate_hz < 25,
          str(db.sample_rate_hz))
    check("mystery column reported, not swallowed",
          any("mystery" in u.lower() for u in db.unmatched), str(db.unmatched))

    print("· plan + run")
    run = run_express(
        "Short on time — 245 kg car, cg 290 mm, need roll numbers and the "
        "telemetry from this log before the review.",
        [("run7.csv", _demo_csv())])
    check("ran something", len(run.ran) >= 3, str(run.ran))
    check("no failures", not run.failed, str(run.failed))
    check("events data-activated even though unasked",
          "events" in run.ran, str(run.ran))
    check("artifacts produced", len(run.artifacts) >= 6,
          str(len(run.artifacts)))

    print("· bundle")
    z1 = bundle_zip(run)
    run2 = run_express(run.ask.text, [("run7.csv", _demo_csv())])
    z2 = bundle_zip(run2)
    check("zip is valid", zipfile.is_zipfile(io.BytesIO(z1)))
    with zipfile.ZipFile(io.BytesIO(z1)) as z:
        names = z.namelist()
    check("README present", "README.md" in names)
    check("manifest present", "manifest.json" in names)
    check("byte-deterministic across runs", z1 == z2,
          f"{len(z1)} vs {len(z2)}")

    print("· empty request never yields an empty bundle")
    run3 = run_express("", None)
    check("fallback job ran", len(run3.artifacts) >= 2, str(run3.ran))

    print(f"\n{'ALL PASS' if not fails else f'{fails} FAILURES'}")
    return 1 if fails else 0


def main(argv: list[str] | None = None) -> int:
    """Command line: the same lane, without a browser.

        python3 -m suspension.express "245 kg car, roll numbers please" \
                run7.csv -o bundle.zip
        python3 -m suspension.express --selftest

    Same engine, same determinism, so CI can diff a bundle across commits.
    """
    import argparse
    import sys

    ap = argparse.ArgumentParser(
        prog="python3 -m suspension.express",
        description="KinematiK Express Lane — two sentences and your data "
                    "in, a bundle out. No language model involved.")
    ap.add_argument("request", nargs="?", default="",
                    help="what you need, in a sentence or two")
    ap.add_argument("files", nargs="*", help="CSV/TSV logs, hardpoint JSON")
    ap.add_argument("-o", "--out", default="kinematik_express.zip",
                    help="output ZIP path")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and the receipts, write nothing")
    ap.add_argument("--selftest", action="store_true",
                    help="run the module self-test and exit")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()

    payload = []
    for path in args.files:
        try:
            with open(path, "rb") as fh:
                payload.append((path.split("/")[-1], fh.read()))
        except OSError as err:
            print(f"! {path}: {err}", file=sys.stderr)

    run = run_express(args.request, payload,
                      progress=lambda msg: print(f"  {msg}", file=sys.stderr))

    print(f"\nrequest    : {run.ask.text or '(none)'}")
    print(f"tools      : {', '.join(run.ask.tools) or '(none recognised)'}")
    print(f"params     : "
          + (", ".join(f"{k}={v:g}" for k, v in sorted(run.ask.params.items()))
             or "(all defaults)"))
    if run.ask.ignored:
        print(f"not read   : {', '.join(run.ask.ignored)}")
    print(f"channels   : {', '.join(sorted(run.data.channels)) or '(none)'}")
    print(f"ran        : {', '.join(run.ran) or '(none)'}")
    print(f"budget     : {run.ask.budget_s:g} s"
          + (f" (from '{run.ask.budget_source}')" if run.ask.budget_source
             else " (lane default)"))
    for jid, why in run.deferred:
        print(f"deferred   : {jid} — {why}")
    for jid, why in run.skipped:
        print(f"skipped    : {jid} — {why}")
    for jid, err in run.failed:
        print(f"FAILED     : {jid} — {err}")

    if args.dry_run:
        print("\n(dry run — nothing written)")
        return 0

    blob = bundle_zip(run)
    with open(args.out, "wb") as fh:
        fh.write(blob)
    print(f"\nwrote {args.out}  ({len(blob) / 1024:.0f} kB, "
          f"{len(run.artifacts) + 2} files)")
    return 1 if run.failed else 0


if __name__ == "__main__":
    import sys
    #  Under `python3 -m suspension.express` this file is __main__, and the
    #  `from .express import ...` inside express_jobs imports a SECOND copy of
    #  it under its real name — so the extra jobs register into that copy's
    #  registry and the CLI silently plans against an empty one. Delegating to
    #  the properly-named module is the fix; it is also why the CLI is a thin
    #  main() rather than inline code.
    from suspension.express import main as _main
    sys.exit(_main())
