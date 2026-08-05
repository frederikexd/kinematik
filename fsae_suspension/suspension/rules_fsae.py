# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/rules_fsae.py — the ruleset, with its provenance attached
# ============================================================================
"""
The FSAE electric-vehicle limits, as data, with a loud label on the tin.

WHY THIS MODULE IS SHAPED THE WAY IT IS

A rulebook encoded into an engineering tool is a liability unless three things
travel with every number it emits:

  1. WHICH DOCUMENT it came from, by title and revision.
  2. WHETHER THAT DOCUMENT IS BINDING. The ruleset loaded here is the **2027
     DRAFT circulated for public comment**. Its own cover page says it is not
     valid for competition and may or may not be adopted in whole or in part.
     A tool that quietly checked a car against a draft — and passed it — would
     be manufacturing false confidence about the one thing a team cannot
     afford to be wrong about.
  3. WHAT THE CHECK COULD NOT SEE. Most of a rulebook is not a number. It is
     "must be operable by an untrained person in ten seconds" and "must be
     visible from all angles". Those cannot be checked from a CSV, and a
     report that lists only the numeric checks reads like a clean bill of
     health. So every check run here is counted against the rules that were
     NOT checked, and that ratio is printed.

So: `RULESET.binding` is False, every renderer refuses to drop the draft
banner, and `check_*` returns findings that name their rule by id.

WHAT IS ACTUALLY CHECKABLE

Two kinds:

  DECLARED — the numbers a member typed. Pack energy, module mass, system
  voltage, motor power. Cheap, and catches the design-stage mistakes.

  MEASURED — the numbers in their own log. This is the one worth having: the
  power and voltage limits are not "don't exceed 80 kW", they are "don't
  exceed it continuously for 100 ms, or on a 500 ms moving average" (EV.3.4.1).
  That is an algorithm, and it is the same algorithm the event's Energy Meter
  runs. A team that logs pack voltage and current can therefore be told, from
  last week's endurance run, exactly how many 60-second penalties they would
  have taken — before the competition, not after.

Rule text is PARAPHRASED throughout, deliberately and tightly. The ids and the
numeric limits are facts and are reproduced exactly; the prose is not the
toolkit's to redistribute. Always read the actual rule before acting.

Pure Python + NumPy. Self-test: python3 -m suspension.rules_fsae
"""

from __future__ import annotations

from dataclasses import dataclass, field as _dcfield
from collections.abc import Sequence

import numpy as np


__all__ = ["RULESET", "Ruleset", "Rule", "Finding", "RULES",
           "check_declared", "check_measured", "banner", "rules_by_section"]


# =========================================================================== #
#  Provenance
# =========================================================================== #
@dataclass(frozen=True)
class Ruleset:
    title: str
    revision: str
    published: str
    binding: bool
    note: str
    comment_url: str = ""

    def label(self) -> str:
        return f"{self.title} (rev {self.revision}, {self.published})"


RULESET = Ruleset(
    title="Formula SAE Rules 2027 — DRAFT for Public Comment",
    revision="0.0",
    published="21 July 2026",
    binding=False,
    note=("This revision is circulated for comment. Its own cover page states "
          "it is not valid for any competition and may or may not be adopted, "
          "in whole or in part. Every check below is therefore provisional: "
          "it tells you where your car stands against a proposal, not against "
          "the rules you will be inspected under."),
    comment_url="https://sae.qualtrics.com/jfe/form/SV_cwLNBpNuY2vxAH4",
)


def banner() -> list[str]:
    """The label that must appear on every artifact carrying a rules verdict.

    Not optional and not configurable. A rules check whose draft status can be
    turned off is a rules check that will eventually be quoted without it.
    """
    return [
        f"> ⚠️ **{RULESET.title}** · revision {RULESET.revision}, "
        f"{RULESET.published}.",
        ">",
        f"> {RULESET.note}",
        ">",
        "> Check every finding against the final published rules for your "
        "competition year before acting on it, and read the actual rule text "
        "— what follows is a paraphrase written to be checkable, not to be "
        "authoritative.",
    ]


# =========================================================================== #
#  The rules, as data
# =========================================================================== #
@dataclass(frozen=True)
class Rule:
    rid: str
    section: str
    title: str
    requirement: str            # paraphrase — see the module docstring
    limit: float | None = None
    unit: str = ""
    checkable: bool = False     # can this be checked from numbers we hold?


RULES: tuple[Rule, ...] = (
    # ---- EV.3 Electrical Limitations ------------------------------------- #
    Rule("EV.3.3.1", "EV.3", "Maximum power",
         "Power measured by the event Energy Meter must stay at or below the "
         "limit.", 80.0, "kW", True),
    Rule("EV.3.3.2", "EV.3", "Maximum voltage",
         "Voltage between any two points must stay at or below the limit.",
         600.0, "V DC", True),
    Rule("EV.3.3.3", "EV.3", "Low-speed regeneration",
         "The powertrain must not regenerate below the speed threshold.",
         5.0, "km/h", True),
    Rule("EV.3.4.1", "EV.3", "What counts as a violation",
         "A power or voltage excursion counts once it persists continuously "
         "for the dwell time, or once a moving average over the averaging "
         "window exceeds the limit.", 0.100, "s continuous", True),
    Rule("EV.3.4.1b", "EV.3", "Violation moving-average window",
         "The second test applies a moving average over this window.",
         0.500, "s window", True),
    Rule("EV.3.5.2", "EV.3", "Endurance penalty",
         "Each violation during Endurance carries a time penalty.",
         60.0, "s per violation", True),
    Rule("EV.3.2.4", "EV.3", "Energy Meter in the path",
         "All tractive-system power must flow through the event Energy "
         "Meter.", None, "", False),
    Rule("EV.3.1.1", "EV.3", "Reverse",
         "Driving the vehicle in reverse under motor power is prohibited.",
         None, "", False),
    Rule("EV.3.1.3", "EV.3", "Torque algorithms",
         "Any control unit adjusting requested wheel torque may only reduce "
         "the driver's request, never increase it.", None, "", False),

    # ---- EV.4 Components -------------------------------------------------- #
    Rule("EV.4.2", "EV.4", "Motor controller",
         "Motors must connect to the tractive battery through a motor "
         "controller; no direct connection.", None, "", False),
    Rule("EV.4.7.1", "EV.4", "APPS / brake plausibility trip",
         "Plausibility fails when the mechanical brakes are engaged while "
         "the accelerator signals more than this share of pedal travel.",
         25.0, "% pedal travel", True),
    Rule("EV.4.7.2", "EV.4", "APPS / brake plausibility reset",
         "After a trip, motor power stays shut down until the accelerator "
         "falls below this share of pedal travel.", 5.0, "% pedal travel",
         True),
    Rule("EV.4.3.6", "EV.4", "Container labelling",
         "Each tractive battery container must carry the school name and "
         "vehicle number, the ISO 7010-W012 warning symbol at the minimum "
         "triangle size, and the required text.", 100.0, "mm triangle side",
         False),

    # ---- EV.5 Energy Storage ---------------------------------------------- #
    Rule("EV.5.1.1a", "EV.5", "Module static voltage",
         "Each module's static voltage must stay at or below the limit.",
         120.0, "V DC", True),
    Rule("EV.5.1.1b", "EV.5", "Module energy",
         "Each module's contained energy must stay at or below the limit, "
         "computed as maximum stack voltage times nominal cell capacity.",
         6.0, "MJ", True),
    Rule("EV.5.1.1c", "EV.5", "Module mass",
         "Each module's mass must stay at or below the limit.", 12.0, "kg",
         True),
    Rule("EV.5.4.1", "EV.5", "Isolation relays and fuse",
         "Every tractive battery pack needs at least one fuse and two or more "
         "normally-open isolation relays opening both poles.", 2.0,
         "relays minimum", True),
    Rule("EV.5.4.5", "EV.5", "Relay hold-up",
         "A capacitor may hold the isolation relays closed after shutdown for "
         "no longer than this.", 250.0, "ms", False),
    Rule("EV.5.5.1b", "EV.5", "MSD height",
         "The manual service disconnect must sit above this height from the "
         "ground.", 350.0, "mm minimum", True),
    Rule("EV.5.5.1d", "EV.5", "MSD operation time",
         "An untrained person must be able to operate the manual service "
         "disconnect within this time, without tools or removing bodywork.",
         10.0, "s maximum", False),
    Rule("EV.5.6.1a", "EV.5", "Precharge target",
         "The precharge circuit must bring the intermediate circuit to at "
         "least this share of tractive-system voltage before the second "
         "isolation relay closes.", 90.0, "%", True),
    Rule("EV.5.6.3d", "EV.5", "Discharge circuit rating",
         "The discharge circuit must handle maximum tractive-system voltage "
         "for at least this long.", 15.0, "s minimum", False),
    Rule("EV.5.6.5", "EV.5", "Precharge relay type",
         "The precharge relay must be a mechanical type.", None, "", False),

    # ---- EV.8 Shutdown system --------------------------------------------- #
    Rule("EV.8.6.3", "EV.8", "IMD response value",
         "The insulation monitoring device's response value must be set at or "
         "above this, relative to the maximum tractive-system voltage.",
         500.0, "ohm/V minimum", True),
    Rule("EV.8.7.2", "EV.8", "BSPD trip",
         "The brake-system plausibility device must open the shutdown circuit "
         "when hard braking coincides with tractive-system current "
         "corresponding to this DC power at nominal pack voltage, for longer "
         "than the dwell.", 5.0, "kW", True),
    Rule("EV.8.7.2b", "EV.8", "BSPD dwell",
         "The BSPD trip condition must persist longer than this before the "
         "shutdown circuit opens.", 0.5, "s", True),
    Rule("EV.8.1.4", "EV.8", "Shutdown branches normally open",
         "The BMS, IMD and BSPD branches of the shutdown circuit must be "
         "normally open and mutually independent.", None, "", False),

    # ---- T.9 Electrical (shared) ------------------------------------------ #
    Rule("T.9.1.1", "T.9", "High voltage threshold",
         "Voltage above this counts as high voltage and pulls in the "
         "labelling and isolation requirements.", 60.0, "V DC", True),
)

_BY_ID: dict[str, Rule] = {r.rid: r for r in RULES}


def rules_by_section() -> dict[str, list[Rule]]:
    out: dict[str, list[Rule]] = {}
    for r in RULES:
        out.setdefault(r.section, []).append(r)
    return out


def limit(rid: str) -> float | None:
    r = _BY_ID.get(rid)
    return r.limit if r else None


# =========================================================================== #
#  Findings
# =========================================================================== #
@dataclass
class Finding:
    rid: str
    severity: str               # ok · watch · violation · unknown
    message: str
    value: float | None = None
    limit: float | None = None
    unit: str = ""

    @property
    def rule(self) -> Rule | None:
        return _BY_ID.get(self.rid)


def _cmp(rid: str, value: float | None, *, at_most: bool = True,
         watch_frac: float = 0.95, what: str = "") -> Finding:
    """One numeric comparison, with a 'watch' band below the limit.

    The band exists because a car at 79.4 kW against an 80 kW limit has not
    broken a rule and is not safe either: the limit is enforced on a meter
    that is not yours, calibrated on a day that is not today.
    """
    r = _BY_ID[rid]
    lim = r.limit
    if value is None or lim is None or not np.isfinite(value):
        return Finding(rid, "unknown",
                       f"{r.title}: nothing to check it against — "
                       f"{what or 'the value was not given'}.",
                       None, lim, r.unit)
    if at_most:
        if value > lim:
            return Finding(rid, "violation",
                           f"{r.title}: {value:g} {r.unit} exceeds the "
                           f"{lim:g} {r.unit} limit.", value, lim, r.unit)
        if value >= lim * watch_frac:
            return Finding(rid, "watch",
                           f"{r.title}: {value:g} {r.unit} is within "
                           f"{100*(1-watch_frac):.0f} % of the {lim:g} "
                           f"{r.unit} limit — no margin for a meter that is "
                           f"not yours.", value, lim, r.unit)
        return Finding(rid, "ok",
                       f"{r.title}: {value:g} {r.unit} against a {lim:g} "
                       f"{r.unit} limit.", value, lim, r.unit)
    if value < lim:
        return Finding(rid, "violation",
                       f"{r.title}: {value:g} {r.unit} is below the required "
                       f"{lim:g} {r.unit}.", value, lim, r.unit)
    return Finding(rid, "ok",
                   f"{r.title}: {value:g} {r.unit} meets the {lim:g} "
                   f"{r.unit} minimum.", value, lim, r.unit)


# =========================================================================== #
#  Declared checks — the numbers a member typed
# =========================================================================== #
def check_declared(params: dict[str, float]) -> list[Finding]:
    """Check what the member stated. Absent values become `unknown`, never
    `ok` — an unchecked rule is not a passed rule."""
    out: list[Finding] = []
    out.append(_cmp("EV.3.3.1", params.get("power_kw"),
                    what="no motor power stated"))
    out.append(_cmp("EV.3.3.2", params.get("pack_v_max"),
                    what="no maximum system voltage stated"))
    out.append(_cmp("EV.5.1.1a", params.get("module_v"),
                    what="no per-module static voltage stated"))
    out.append(_cmp("EV.5.1.1c", params.get("module_kg"),
                    what="no per-module mass stated"))

    #  Module energy is stated in kWh by every team and limited in MJ by the
    #  rules, which is exactly the kind of unit seam that produces a confident
    #  wrong answer. Converted here once, and the conversion is printed.
    mod_kwh = params.get("module_kwh")
    if mod_kwh is not None and np.isfinite(mod_kwh):
        f = _cmp("EV.5.1.1b", float(mod_kwh) * 3.6)
        f.message += f"  (converted: {mod_kwh:g} kWh × 3.6 = " \
                     f"{mod_kwh * 3.6:g} MJ)"
        out.append(f)
    else:
        out.append(_cmp("EV.5.1.1b", None,
                        what="no per-module energy stated"))

    #  A whole-pack figure cannot pass or fail a per-module rule, but it can
    #  tell you how many modules the pack must split into — which is the
    #  question the member actually has.
    pack_kwh = params.get("pack_kwh")
    if pack_kwh is not None and np.isfinite(pack_kwh):
        n_min = int(np.ceil(float(pack_kwh) * 3.6 / limit("EV.5.1.1b")))
        out.append(Finding(
            "EV.5.1.1b", "watch" if n_min > 1 else "ok",
            f"Module count: a {pack_kwh:g} kWh pack "
            f"({pack_kwh * 3.6:.2f} MJ) must be divided into at least "
            f"**{n_min} modules** to keep each one under the "
            f"{limit('EV.5.1.1b'):g} MJ limit — before any voltage or mass "
            f"limit is considered, which may force more.",
            float(pack_kwh) * 3.6, limit("EV.5.1.1b"), "MJ"))
    return out


# =========================================================================== #
#  Measured checks — the team's own log, scored the way the meter scores it
# =========================================================================== #
def _runs_over(mask: np.ndarray, t: np.ndarray, min_s: float
               ) -> list[tuple[float, float]]:
    """Contiguous True runs lasting at least `min_s`."""
    out: list[tuple[float, float]] = []
    i, n = 0, int(mask.size)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            if float(t[j] - t[i]) >= min_s:
                out.append((float(t[i]), float(t[j])))
            i = j + 1
        else:
            i += 1
    return out


def _moving_average(y: np.ndarray, win: int) -> np.ndarray:
    if win <= 1 or y.size < win:
        return y.astype(float)
    k = np.ones(int(win), float) / float(win)
    return np.convolve(y.astype(float), k, mode="same")


@dataclass
class MeasuredResult:
    findings: list[Finding] = _dcfield(default_factory=list)
    power_events: list[tuple[float, float, str]] = _dcfield(
        default_factory=list)
    voltage_events: list[tuple[float, float, str]] = _dcfield(
        default_factory=list)
    regen_events: list[tuple[float, float, str]] = _dcfield(
        default_factory=list)
    peak_kw: float = float("nan")
    peak_v: float = float("nan")
    sample_rate_hz: float = float("nan")
    rate_adequate: bool = False

    @property
    def n_violations(self) -> int:
        return len(self.power_events) + len(self.voltage_events)

    @property
    def endurance_penalty_s(self) -> float:
        return self.n_violations * (limit("EV.3.5.2") or 60.0)


def check_measured(t: Sequence[float], pack_v: Sequence[float],
                   pack_i: Sequence[float],
                   speed_ms: Sequence[float] | None = None
                   ) -> MeasuredResult:
    """Score a log the way EV.3.4.1 defines a violation.

    Two independent tests, both of which count: a continuous excursion of at
    least the dwell time, and a moving average over the averaging window that
    crosses the limit. Overlapping detections are merged so one excursion is
    not counted twice.
    """
    res = MeasuredResult()
    t = np.asarray(t, float)
    v = np.asarray(pack_v, float)
    i = np.asarray(pack_i, float)
    good = np.isfinite(t) & np.isfinite(v) & np.isfinite(i)
    t, v, i = t[good], v[good], i[good]
    if t.size < 8:
        res.findings.append(Finding("EV.3.4.1", "unknown",
                                    "Too few usable samples to score."))
        return res

    dt = np.diff(t)
    dt = dt[dt > 0]
    med = float(np.median(dt)) if dt.size else float("nan")
    res.sample_rate_hz = 1.0 / med if med > 0 else float("nan")
    dwell = limit("EV.3.4.1") or 0.100
    window = limit("EV.3.4.1b") or 0.500
    #  A 10 Hz log cannot resolve a 100 ms dwell. Saying so is the finding;
    #  scoring it anyway and reporting zero violations would be worse than
    #  saying nothing.
    res.rate_adequate = bool(np.isfinite(res.sample_rate_hz)
                             and res.sample_rate_hz >= 2.0 / dwell)

    kw = v * i / 1000.0
    res.peak_kw = float(np.nanmax(kw))
    res.peak_v = float(np.nanmax(v))
    p_lim = limit("EV.3.3.1") or 80.0
    v_lim = limit("EV.3.3.2") or 600.0
    win_n = max(1, int(round(window / med))) if med > 0 else 1

    for label, series, lim, sink in (
            ("power", kw, p_lim, res.power_events),
            ("voltage", v, v_lim, res.voltage_events)):
        cont = _runs_over(series > lim, t, dwell)
        avg = _runs_over(_moving_average(series, win_n) > lim, t, 0.0)
        merged: list[tuple[float, float, str]] = [
            (a, b, f"continuous > {dwell*1000:.0f} ms") for a, b in cont]
        for a, b in avg:
            if not any(not (b < ea or a > eb) for ea, eb, _k in merged):
                merged.append((a, b, f"{window*1000:.0f} ms moving average"))
        merged.sort()
        sink.extend(merged)

    res.findings.append(_cmp("EV.3.3.1", res.peak_kw))
    res.findings.append(_cmp("EV.3.3.2", res.peak_v))

    if speed_ms is not None:
        s = np.asarray(speed_ms, float)[good] if np.asarray(
            speed_ms, float).size == good.size else np.asarray(speed_ms, float)
        n = min(s.size, kw.size)
        if n > 8:
            thresh = (limit("EV.3.3.3") or 5.0) / 3.6      # km/h → m/s
            bad = (kw[:n] < -0.1) & (s[:n] >= 0) & (s[:n] < thresh)
            for a, b in _runs_over(bad, t[:n], 0.0):
                res.regen_events.append((a, b, "regen below the speed floor"))
            res.findings.append(Finding(
                "EV.3.3.3",
                "violation" if res.regen_events else "ok",
                (f"{len(res.regen_events)} regeneration episodes below "
                 f"{limit('EV.3.3.3'):g} km/h."
                 if res.regen_events else
                 f"No regeneration recorded below "
                 f"{limit('EV.3.3.3'):g} km/h."),
                float(len(res.regen_events)), 0.0, "episodes"))
    else:
        res.findings.append(Finding(
            "EV.3.3.3", "unknown",
            "No speed channel, so low-speed regeneration was not checked."))
    return res


# =========================================================================== #
#  Coverage — what was NOT checked matters as much as what was
# =========================================================================== #
def coverage() -> tuple[int, int]:
    """(checkable rules encoded, total rules encoded). Both are a fraction of
    the real rulebook, which is the point of printing them."""
    return sum(1 for r in RULES if r.checkable), len(RULES)


def _selftest() -> int:
    fails = 0

    def chk(name, cond, detail=""):
        nonlocal fails
        print(f"  {'PASS' if cond else 'FAIL'}  {name}"
              + (f"   {detail}" if detail and not cond else ""))
        if not cond:
            fails += 1

    print("· provenance")
    chk("ruleset is not binding", RULESET.binding is False)
    chk("banner names the draft", any("DRAFT" in b for b in banner()))

    print("· declared")
    f = {x.rid: x for x in check_declared({"power_kw": 95.0})}
    chk("over-power flagged", f["EV.3.3.1"].severity == "violation")
    f = {x.rid: x for x in check_declared({"power_kw": 79.0})}
    chk("just-under flagged as watch", f["EV.3.3.1"].severity == "watch")
    f = {x.rid: x for x in check_declared({})}
    chk("absent value is unknown, never ok",
        f["EV.3.3.1"].severity == "unknown")

    print("· measured")
    t = np.arange(0, 20, 0.01)
    v = np.full_like(t, 400.0)
    i = np.full_like(t, 150.0)                    # 60 kW baseline
    i[(t > 5.0) & (t < 5.3)] = 250.0              # 100 kW for 300 ms
    r = check_measured(t, v, i, speed_ms=np.full_like(t, 20.0))
    chk("power violation caught", len(r.power_events) >= 1,
        str(r.power_events))
    chk("penalty is 60 s each",
        r.endurance_penalty_s == 60.0 * r.n_violations)
    chk("no false voltage violation", not r.voltage_events)
    chk("rate adequacy detected", r.rate_adequate)

    i2 = np.full_like(t, 150.0)                   # clean run
    r2 = check_measured(t, v, i2, speed_ms=np.full_like(t, 20.0))
    chk("clean run is clean", r2.n_violations == 0, str(r2.n_violations))

    slow_t = np.arange(0, 20, 0.2)                # 5 Hz
    r3 = check_measured(slow_t, np.full_like(slow_t, 400.0),
                        np.full_like(slow_t, 150.0))
    chk("slow log flagged as unresolvable", not r3.rate_adequate)

    print("· coverage")
    ck, tot = coverage()
    chk("coverage reports both numbers", 0 < ck <= tot)

    print(f"\n{'ALL PASS' if not fails else f'{fails} FAILURES'}")
    return 1 if fails else 0


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
