# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/worthwhile.py — "Is it worthwhile once assembled?" The piece that
#  closes the loop the IntegrationLedger leaves open: it pushes the RECONCILED
#  (real, not per-team-optimistic) mass / CG / downforce into the actual vehicle
#  model, runs the real lap sim, and reports the POINTS gap between the paper car
#  and the car the team will actually build — while REFUSING to produce a
#  reassuring number when a hard contradiction makes the car not-buildable.
# ============================================================================
"""
Worthwhileness — does the assembly still score, and is it even buildable?

WHY THIS MODULE EXISTS
----------------------
interfaces.py already does the hard, unglamorous half: it reconciles what every
subsystem DECLARES (mass, CG, envelope, loads, power, heat) against the shared
budgets and flags MISSING data and FAIL-level incompatibilities. But it stops one
step short of the question a lead actually asks — "if I build exactly this, is it
worth it?" — because the reconciled numbers never reach the physics. The ledger
even leaves a literal note ("feed CG height into the vehicle model") that today a
human has to action by hand. This module actions it, and adds the two things a
static ledger structurally cannot do:

  1. THE PAPER-vs-REAL POINTS GAP.
     Every subsystem sizes itself on optimistic inputs — suspension tunes for a
     210 kg car, the pack comes in at 240, aero designed for a CG the ballast
     blew. Each subsystem is individually "correct" and the assembled car is
     slower than any of their slides claim. This module runs the real lap sim
     TWICE: once on the optimistic paper baseline, once on the RECONCILED build,
     and reports the points delta between them per event. That delta is the
     honest cost of the estimates catching up with reality.

  2. THE ASSUMPTION-CONTRADICTION CHECK.
     The ledger compares each subsystem to a shared TARGET. It does not catch
     when subsystem A ASSUMED something about subsystem B that B now contradicts
     — the real assembly-killer. If suspension declares it "assumes 210 kg" and
     the reconciled mass is 240 kg, that is a contradiction with an owner, not a
     vague budget overrun. We surface those as first-class findings naming both
     sides.

THE HARD RULE (THE POINT OF THE WHOLE THING)
--------------------------------------------
An integration tool earns its keep by REFUSING TO AVERAGE AWAY A CONTRADICTION.
The failure mode is a smooth "92% integrated ✓" printed over a broken mass
budget or a part that doesn't fit. So this module has a NO-GO gate: if any
buildability-blocking FAIL is present (the parts do not physically go together,
or required data is missing to even attempt the physics), it returns a NOT
BUILDABLE verdict and DECLINES to print a points score — because a points number
for a car that can't be assembled is a lie. Only when the assembly is buildable
does it compute and report the worthwhileness delta.

WHAT IS PHYSICS vs ESTIMATE HERE
--------------------------------
The lap sim, load transfer and points curves are the same real models the rest
of KinematiK uses and tests. The mass/CG roll-up is exact arithmetic on declared
numbers. What stays an ESTIMATE is any subsystem interface still flagged
is_estimate — and the verdict says so, refusing to imply more certainty than the
teams actually have. Nothing here fabricates a number that wasn't declared or
solved.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional

from . import interfaces as ifc
from .interfaces import Severity, Finding, IntegrationLedger
from .dynamics import VehicleParams, VehicleDynamics
from . import lapsim


# ===================================================================== #
#  1.  ASSUMPTION CONTRADICTIONS  (A assumed X about B; B says not-X)
# ===================================================================== #
@dataclass
class Assumption:
    """One subsystem's stated assumption about a shared or another-subsystem value.

    e.g. suspension: assumes total car mass = 210 kg; aero: assumes cg_z = 300 mm.
    These are the design premises a subsystem sized itself on. When the reconciled
    reality differs, THAT is the contradiction that sinks the assembly — and it
    has a clear owner (the subsystem that assumed wrong, or the one that came in
    heavy), unlike a faceless budget overrun.
    """
    by: str                 # subsystem making the assumption
    field: str              # what it assumed about (e.g. "total_mass_kg", "cg_z_mm")
    value: float            # the value it assumed
    tol: float = 0.0        # acceptable band around the assumption
    about: str = "car"      # which subsystem/shared quantity it's about
    rationale: str = ""


def check_assumptions(ledger: IntegrationLedger,
                      assumptions: list[Assumption]) -> list[Finding]:
    """Compare each declared assumption against the reconciled reality.

    Reconciled reality is computed from the ledger's own mass roll-up (exact
    arithmetic on declared masses/CGs). Each violated assumption becomes a
    Finding naming both the assumer and, where identifiable, the subsystem whose
    real number broke it.
    """
    out: list[Finding] = []
    roll = ledger.mass_rollup()
    real = {
        "total_mass_kg": roll["total_kg"] if roll["declared"] else None,
        "cg_x_mm": roll["cg_mm"][0] if roll["cg_mm"] else None,
        "cg_y_mm": roll["cg_mm"][1] if roll["cg_mm"] else None,
        "cg_z_mm": roll["cg_mm"][2] if roll["cg_mm"] else None,
    }
    for a in assumptions:
        actual = real.get(a.field)
        if actual is None:
            out.append(Finding(
                "assumption-uncheckable", Severity.MISSING,
                f"{a.by} assumes {a.field} = {a.value:g}, but the reconciled "
                f"value isn't computable yet (missing subsystem data). This "
                f"assumption is riding unverified.",
                subsystems=[a.by], detail=dict(assumed=a.value, field=a.field)))
            continue
        if abs(actual - a.value) > a.tol:
            # who broke it? the heaviest declared subsystem for a mass miss, else
            # just name the assumer and 'car'.
            culprit = _largest_contributor(ledger, a.field)
            subs = [a.by] + ([culprit] if culprit and culprit != a.by else [])
            out.append(Finding(
                "assumption-contradiction", Severity.FAIL,
                f"{a.by} designed around {a.field} = {a.value:g}"
                f"{f' ±{a.tol:g}' if a.tol else ''}, but the reconciled build is "
                f"{actual:g} ({actual - a.value:+.1f}). {a.by}'s work is sized for "
                f"a car that doesn't exist"
                + (f"; {culprit} is the largest contributor to the gap." if culprit
                   and culprit != a.by else "."),
                subsystems=subs,
                detail=dict(assumed=a.value, actual=actual,
                            delta=actual - a.value, about=a.about,
                            rationale=a.rationale)))
    return out


def _largest_contributor(ledger: IntegrationLedger, field_name: str) -> Optional[str]:
    """For a mass/CG assumption miss, name the subsystem contributing most to it —
    so the finding has a concrete owner, not just a budget."""
    if not field_name.startswith(("total_mass", "cg_")):
        return None
    best, best_val = None, -1.0
    for name, it in ledger.interfaces.items():
        m = getattr(it, "mass_kg", None)
        if m is None:
            continue
        # for CG_z misses, weight by mass*height; for mass, by mass
        if field_name == "cg_z_mm" and getattr(it, "cg_z_mm", None) is not None:
            contrib = m * abs(it.cg_z_mm)
        else:
            contrib = m
        if contrib > best_val:
            best, best_val = name, contrib
    return best


# ===================================================================== #
#  2.  BUILD A VEHICLE FROM THE RECONCILED LEDGER  (the missing handoff)
# ===================================================================== #
def vehicle_from_ledger(ledger: IntegrationLedger,
                        base: VehicleParams | None = None,
                        front_kin=None, rear_kin=None, tire=None) -> VehicleDynamics:
    """Push the RECONCILED mass + CG height from the ledger into a VehicleParams,
    keeping everything else from `base`. This is the handoff interfaces.py only
    describes in a note. Returns a VehicleDynamics ready for the lap sim.

    Mass: reconciled subsystem total, plus the ledger's declared driver
    allowance (so the vehicle mass is the real all-up mass the tyres carry).
    CG height: the reconciled combined CG_z when available, else base.
    """
    p = base or VehicleParams()
    roll = ledger.mass_rollup()
    total = roll["total_kg"] if roll["declared"] else p.mass
    # add driver allowance if the target was declared to include one and the
    # subsystem masses don't already carry it
    total_all_up = total + max(ledger.includes_driver_kg, 0.0)
    cg_z = roll["cg_mm"][2] if roll["cg_mm"] else p.cg_height

    # shallow-copy params with reconciled mass/CG
    from dataclasses import replace
    p2 = replace(p, mass=float(total_all_up), cg_height=float(cg_z))
    return VehicleDynamics(p2, front_kin=front_kin, rear_kin=rear_kin, tire=tire)


# ===================================================================== #
#  3.  THE WORTHWHILENESS VERDICT
# ===================================================================== #
@dataclass
class WorthwhileVerdict:
    buildable: bool                     # False => NO-GO, points withheld
    blocking: list                      # the FAIL findings that block the build
    findings: list                      # all integration + assumption findings
    paper_points: Optional[dict]        # per-event points on optimistic baseline
    real_points: Optional[dict]         # per-event points on reconciled build
    points_delta: Optional[dict]        # real - paper, per event and total
    any_estimate: bool                  # reconciled numbers still contain estimates
    verdict_text: str

    def total_delta(self) -> Optional[float]:
        if self.points_delta is None:
            return None
        return self.points_delta.get("total")


def _event_times(veh: VehicleDynamics,
                 params: lapsim.LapSimParams | None = None,
                 endurance_laps: int = 1) -> dict:
    """Run the standard events and return {event: event_time_s or None}."""
    results = lapsim.simulate_events(veh, params=params,
                                     endurance_laps=endurance_laps)
    out = {}
    for ev, res in results.items():
        t = getattr(res, "event_time", None)
        ok = getattr(res, "ok", False) and t is not None and np.isfinite(t) and t > 0
        out[ev] = float(t) if ok else None
    return out


def _score_against_reference(times: dict, ref_times: dict) -> dict:
    """Score a car's event times against a SHARED reference (the paper car's
    times). This is the honest fix: event_points with no reference scores every
    car at max against itself, so paper and real would tie regardless of mass.
    Scoring BOTH cars against the same reference (paper as Tmin) makes a heavier,
    slower real car actually lose points — the whole point of the comparison.
    """
    pts = {}
    for ev, t in times.items():
        ref = ref_times.get(ev)
        if t is None or ref is None or ref <= 0:
            pts[ev] = 0.0
        else:
            # paper car is the benchmark (Tmin=ref); a slower real time scores less
            pts[ev] = lapsim.event_points(ev, t, best_time=ref)
    pts["total"] = float(sum(v for k, v in pts.items() if k != "total"))
    return pts


def assess(ledger: IntegrationLedger,
           assumptions: list[Assumption] | None = None,
           paper_baseline: VehicleParams | None = None,
           front_kin=None, rear_kin=None, tire=None,
           sim_params: lapsim.LapSimParams | None = None,
           endurance_laps: int = 1) -> WorthwhileVerdict:
    """The full worthwhileness assessment.

    Steps:
      1. Run the ledger's own integration checks (mass, CG, envelope, thermal,
         electrical, driveline, mounts, estimate-flagging).
      2. Run the assumption-contradiction check.
      3. If any BUILDABILITY-BLOCKING FAIL exists (a part doesn't fit, an
         assumption is contradicted, or data needed for the physics is missing),
         return NOT BUILDABLE and WITHHOLD the points number.
      4. Otherwise build the reconciled vehicle, run the lap sim on both the
         optimistic paper baseline and the reconciled build, and report the
         per-event and total points delta.
    """
    assumptions = assumptions or []
    findings = list(ledger.check_all())
    findings += check_assumptions(ledger, assumptions)

    # what blocks a build: physical non-fit, contradicted assumptions, and the
    # specific MISSING data the physics cannot proceed without (mass roll-up).
    roll = ledger.mass_rollup()
    blocking = [f for f in findings if f.severity == Severity.FAIL]
    physics_blocked = not roll["declared"] or roll["cg_mm"] is None
    if physics_blocked:
        blocking = blocking + [Finding(
            "physics-input-missing", Severity.MISSING,
            "Cannot run the lap sim on the reconciled build: mass and/or combined "
            "CG are not yet computable from declared subsystem data. The "
            "worthwhileness number is withheld rather than faked.",
            subsystems=roll["missing"])]

    any_estimate = bool(roll.get("any_estimate"))

    if blocking:
        names = ", ".join(sorted({s for f in blocking for s in f.subsystems})) or "—"
        txt = (f"NOT BUILDABLE as declared. {len(blocking)} blocking issue(s) "
               f"involving: {names}. Points withheld — a worthwhileness score for "
               f"a car that can't be assembled would be misleading. Resolve the "
               f"blocking findings, then re-run.")
        return WorthwhileVerdict(
            buildable=False, blocking=blocking, findings=findings,
            paper_points=None, real_points=None, points_delta=None,
            any_estimate=any_estimate, verdict_text=txt)

    # buildable: run the physics on both the paper baseline and the reconciled car
    paper_params = paper_baseline or VehicleParams()
    paper_veh = VehicleDynamics(paper_params, front_kin=front_kin,
                                rear_kin=rear_kin, tire=tire)
    real_veh = vehicle_from_ledger(ledger, base=paper_params,
                                   front_kin=front_kin, rear_kin=rear_kin, tire=tire)

    paper_times = _event_times(paper_veh, sim_params, endurance_laps)
    real_times = _event_times(real_veh, sim_params, endurance_laps)
    # score BOTH against the paper car's times as the shared benchmark, so the
    # heavier/slower real car actually loses points (see _score_against_reference)
    paper_pts = _score_against_reference(paper_times, paper_times)  # ~max by def.
    real_pts = _score_against_reference(real_times, paper_times)
    delta = {k: round(real_pts[k] - paper_pts.get(k, 0.0), 1) for k in real_pts}

    dm = real_veh.p.mass - paper_params.mass
    dcg = real_veh.p.cg_height - paper_params.cg_height
    est_note = (" Some reconciled inputs are still ESTIMATES — treat the delta as "
                "directional until they're confirmed.") if any_estimate else ""
    txt = (f"BUILDABLE. Reconciled build is {dm:+.1f} kg and {dcg:+.0f} mm CG vs "
           f"the paper baseline, costing an estimated {delta['total']:+.1f} points "
           f"across the dynamic events. That gap is the price of the estimates "
           f"meeting reality — close it by attacking the heaviest over-budget "
           f"subsystem, not by re-optimising suspension in a vacuum.{est_note}")

    return WorthwhileVerdict(
        buildable=True, blocking=[], findings=findings,
        paper_points=paper_pts, real_points=real_pts, points_delta=delta,
        any_estimate=any_estimate, verdict_text=txt)


PROVENANCE = {
    "physics_grounded": [
        "lap times & load transfer (real LapSimulator / VehicleDynamics)",
        "points curves (published-form FSAE event_points)",
        "mass & CG roll-up (exact arithmetic on declared numbers)",
    ],
    "estimate_flagged": [
        "any subsystem interface still marked is_estimate — the verdict says so",
    ],
    "hard_rule": (
        "When a blocking FAIL (non-fit, contradicted assumption, or missing "
        "physics input) is present, the points number is WITHHELD, not averaged "
        "away. A worthwhileness score for an unbuildable car is a lie."
    ),
}
