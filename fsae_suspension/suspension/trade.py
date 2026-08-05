# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  suspension/trade.py — "is this specific part worth buying?"
#
#  worthwhile.py answers "does the car we are actually building still score?"
#  It cannot answer "should we spend $2,600 on THIS part?", because it compares a
#  paper car to a reconciled car, not option A to option B. This module does the
#  A/B: it runs the real lap sim on each option, converts the time delta to
#  points, prices the points — and REFUSES to name a winner when the difference
#  is smaller than the model's own uncertainty.
# ============================================================================
"""
Component trade study with an honest resolution floor.

THE FAILURE MODE THIS IS BUILT AGAINST
--------------------------------------
Point a lap sim at two parts, get 0.14 s, multiply by a points curve, write
"worth 4.2 points" in the design brief. The number is fiction: it is smaller
than the effect of the mu you assumed, the torque bias ratio you copied off a
forum, and the corner-exit condition you picked. The sim will hand you that
number to three decimals every time, and nothing in it tells you the sign is
not reliable.

So this module never returns a single delta. It sweeps the parameters the
answer is actually sensitive to, and applies one rule:

    IF THE SIGN OF THE POINTS DELTA IS NOT STABLE ACROSS THE SWEPT BAND,
    NO WINNER IS DECLARED AND NO $/POINT IS PRINTED.

This mirrors the hard rule in worthwhile.py — refusing to average away a
contradiction — applied to a purchasing decision instead of an assembly.

WHAT GETS SWEPT (and why each one matters)
------------------------------------------
  mu_scale        absolute grip is a placeholder unless you have TTC data;
                  everything traction-limited scales with it
  tbr             a quoted torque bias ratio is nominal, and real bias moves
                  with preload state, oil and wear
  preload_nm      shim/spring dependent; the whole point of an adjustable unit
  lock_k          the yaw/scrub penalty for a locked axle — the one coefficient
                  in driveline.py that is not solved physics
  exit_lateral_g  which corner exit you evaluate at changes the load ratio the
                  diff is working against, which is the entire mechanism

THE THREE CURRENCIES
--------------------
A part is not "worth it" on points alone. This reports, separately and without
blending them into one score:
  * POINTS      from the lap sim, with a band
  * DOLLARS     purchase + the parts you must buy to run it. Note the FSAE Cost
                event is a real points category, so money is partly convertible
                — but only if you pass the reference costs; otherwise withheld.
  * TEAM HOURS  the labour a listing hides. "No sprocket adapter" is not a
                discount, it is a work order.
Nothing here converts hours into points. If your team is time-limited rather
than cash-limited, the hours column is the one that decides, and only you know
which constraint binds.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from collections.abc import Sequence

import numpy as np

from . import lapsim
from .dynamics import VehicleParams, VehicleDynamics
from .driveline import (
    DifferentialSpec, ExitCondition, axle_traction, apply_lock_derate,
    LOCK_PENALTY_K_DEFAULT, LOCK_PENALTY_K_BAND,
)


# --------------------------------------------------------------------------- #
#  1.  What gets swept
# --------------------------------------------------------------------------- #
@dataclass
class Uncertainty:
    """The bands the answer is allowed to move inside. Widen them when your
    inputs are guesses; narrow them when you have measured data. A narrow band
    you cannot defend is how a fictional number gets a confidence interval."""
    mu_scale: tuple = (0.88, 1.12)          # multiplier on peak tyre friction
    tbr_rel: tuple = (0.75, 1.30)           # multiplier on each spec's TBR
    preload_rel: tuple = (0.4, 1.8)         # multiplier on each spec's preload
    lock_k: tuple = LOCK_PENALTY_K_BAND     # yaw/scrub penalty coefficient
    exit_lateral_frac: tuple = (0.45, 0.88)  # share of the car's lateral limit
                                             # being used at the corner exit
    n_draws: int = 300
    seed: int = 12345

    def sample(self, rng) -> dict:
        u = rng.uniform
        return dict(
            mu_scale=u(*self.mu_scale),
            tbr_rel=u(*self.tbr_rel),
            preload_rel=u(*self.preload_rel),
            lock_k=u(*self.lock_k),
            exit_lateral_frac=u(*self.exit_lateral_frac),
        )


@dataclass
class Option:
    """One purchasable configuration under consideration."""
    label: str
    spec: DifferentialSpec
    mass_offset_kg: float = 0.0     # anything else that changes with this choice
                                    # (mounts, chain, carrier) — signed, kg
    notes: str = ""

    def acquisition_usd(self) -> float | None:
        return self.spec.total_acquisition_usd()

    def fab_hours(self) -> float:
        return float(self.spec.extra_fab_hours)


# --------------------------------------------------------------------------- #
#  2.  One evaluation
# --------------------------------------------------------------------------- #
def _event_times(veh, sim_params, endurance_laps: int) -> dict:
    res = lapsim.simulate_events(veh, params=sim_params,
                                 endurance_laps=endurance_laps)
    out = {}
    for ev, r in res.items():
        t = getattr(r, "event_time", None)
        ok = getattr(r, "ok", False) and t is not None and np.isfinite(t) and t > 0
        out[ev] = float(t) if ok else None
    return out


def _field_reference(all_times: Sequence[dict]) -> dict:
    """Per-event Tmin taken as the FASTEST option in the comparison — the same
    convention FSAE itself uses, where Tmin is the best time in the field.

    This matters more than it looks. Scoring every option against the BASELINE's
    time instead is subtly one-sided: event_points clamps at maximum for any time
    faster than Tmin, so an option that is quicker than the baseline scores
    identically to it, while an option that is slower is penalised in full. The
    delta can then only ever come out negative or zero, no matter how much
    quicker the candidate really is. (The same asymmetry is present in
    worthwhile._score_against_reference, where it is harmless only because a
    reconciled car is never lighter than the paper car it is compared to.)"""
    ref = {}
    for ev in {e for t in all_times for e in t}:
        vals = [t[ev] for t in all_times if t.get(ev)]
        ref[ev] = min(vals) if vals else None
    return ref


def _score_against(times: dict, ref_times: dict) -> dict:
    """Score a set of event times against a shared per-event Tmin."""
    pts = {}
    for ev, t in times.items():
        ref = ref_times.get(ev)
        pts[ev] = 0.0 if (t is None or not ref or ref <= 0) else \
            lapsim.event_points(ev, t, best_time=ref)
    pts["total"] = float(sum(v for k, v in pts.items() if k != "total"))
    return pts


def evaluate_option(opt: Option, base_params: VehicleParams,
                    sim_params: lapsim.LapSimParams,
                    cond: ExitCondition,
                    draw: dict | None = None,
                    front_kin=None, rear_kin=None, tire=None,
                    endurance_laps: int = 1) -> dict:
    """Run one option once, at one point in the swept space. Returns the event
    times plus the diagnostics that explain them."""
    draw = draw or dict(mu_scale=1.0, tbr_rel=1.0, preload_rel=1.0,
                        lock_k=LOCK_PENALTY_K_DEFAULT,
                        exit_lateral_frac=cond.lateral_g_frac or 0.70)
    spec = replace(opt.spec,
                   tbr=opt.spec.tbr * draw["tbr_rel"]
                   if np.isfinite(opt.spec.tbr) else opt.spec.tbr,
                   preload_nm=opt.spec.preload_nm * draw["preload_rel"])
    c = replace(cond, lateral_g=None,
                lateral_g_frac=draw["exit_lateral_frac"], cl_a=sim_params.cl_a)

    # mass: the part plus anything that changes with it
    dm = (spec.mass_kg or 0.0) + opt.mass_offset_kg
    p = replace(base_params,
                mass=base_params.mass + dm,
                mu_peak=base_params.mu_peak * draw["mu_scale"])
    veh_for_traction = VehicleDynamics(p, front_kin=front_kin,
                                       rear_kin=rear_kin, tire=tire)
    tr = axle_traction(veh_for_traction, spec, c)

    # the cost of locking: derate lateral grip, then re-build the vehicle
    p_run = apply_lock_derate(p, tr.lock_ratio, draw["lock_k"])
    veh = VehicleDynamics(p_run, front_kin=front_kin, rear_kin=rear_kin, tire=tire)
    lp = replace(sim_params, mass=p_run.mass,
                 drive_grip_frac=tr.drive_grip_frac)

    times = _event_times(veh, lp, endurance_laps)
    return dict(times=times, drive_grip_frac=tr.drive_grip_frac,
                lock_ratio=tr.lock_ratio, mass_kg=p_run.mass,
                t_total_nm=tr.t_total_nm, fz_inside_n=tr.fz_inside_n,
                fz_outside_n=tr.fz_outside_n, lateral_util=tr.lateral_util,
                notes=tr.notes)


# --------------------------------------------------------------------------- #
#  3.  The verdict
# --------------------------------------------------------------------------- #
@dataclass
class OptionResult:
    label: str
    delta_points_median: float | None
    delta_points_p05: float | None
    delta_points_p95: float | None
    sign_stable: bool                 # does the band stay on one side of zero?
    acquisition_usd: float | None
    fab_hours: float
    usd_per_point: float | None    # withheld unless sign_stable
    nominal: dict = field(default_factory=dict)   # centre-of-band diagnostics
    notes: list = field(default_factory=list)
    actionable: bool = False          # sign-stable AND bigger than the floor
                                      # below which a real driver would bury it


@dataclass
class TradeVerdict:
    baseline: str
    results: list                      # list[OptionResult]
    decidable: bool                    # is ANY option separable from baseline?
    resolution_points: float           # the width of the noise floor we found
    verdict_text: str
    pairwise: dict = field(default_factory=dict)   # (a,b) -> paired delta band
    cost_event: dict | None = None
    provenance: dict = field(default_factory=dict)

    def best(self) -> OptionResult | None:
        cand = [r for r in self.results if r.actionable
                and r.delta_points_median is not None
                and r.delta_points_median > 0]
        return max(cand, key=lambda r: r.delta_points_median) if cand else None


def compare(options: Sequence[Option], baseline: Option,
            base_params: VehicleParams | None = None,
            sim_params: lapsim.LapSimParams | None = None,
            cond: ExitCondition | None = None,
            unc: Uncertainty | None = None,
            practical_floor_points: float = 5.0,
            front_kin=None, rear_kin=None, tire=None,
            endurance_laps: int = 1) -> TradeVerdict:
    """A/B (or A/B/C) the options against the baseline, with the sweep.

    For every draw, ALL options are evaluated at the SAME draw — same mu, same
    corner, same lock coefficient — and scored against that draw's baseline.
    That is what makes the comparison meaningful: the shared nuisance parameters
    cancel, and what survives is the difference between the parts.

    ``practical_floor_points`` is the second gate, and it exists because sign
    stability alone is not enough. A sweep can report a rock-solid +0.4 points,
    and +0.4 points is not a decision: driver run-to-run variation at a real
    competition routinely swamps it, as do track temperature, a scruffy launch
    and traffic on an autocross run. A difference the model can resolve but a
    driver would bury is reported as NOT ACTIONABLE, and no $/point is printed
    for it. Set the floor from your own data-logged run-to-run spread if you
    have it — the 5-point default is a placeholder, not a measurement.
    """
    base_params = base_params or VehicleParams()
    sim_params = sim_params or lapsim.LapSimParams(mass=base_params.mass)
    cond = cond or ExitCondition(cl_a=sim_params.cl_a)
    unc = unc or Uncertainty()
    rng = np.random.default_rng(unc.seed)

    totals: dict[str, list] = {o.label: [] for o in list(options) + [baseline]}
    nominal: dict[str, dict] = {}
    all_notes: dict[str, set] = {o.label: set() for o in list(options) + [baseline]}

    # centre-of-band nominal run, for the diagnostics a human reads
    for o in list(options) + [baseline]:
        nominal[o.label] = evaluate_option(
            o, base_params, sim_params, cond, None,
            front_kin, rear_kin, tire, endurance_laps)

    for _ in range(int(unc.n_draws)):
        d = unc.sample(rng)
        # every option sees the SAME draw — the shared nuisance parameters
        # cancel, and what survives is the difference between the parts
        runs = {}
        for o in list(options) + [baseline]:
            runs[o.label] = evaluate_option(o, base_params, sim_params, cond, d,
                                            front_kin, rear_kin, tire,
                                            endurance_laps)
            all_notes[o.label].update(runs[o.label]["notes"])
        ref = _field_reference([r["times"] for r in runs.values()])
        for lab, r in runs.items():
            totals[lab].append(_score_against(r["times"], ref)["total"])

    def _band(arr):
        a = np.asarray(arr, dtype=float)
        a = a[np.isfinite(a)]
        if a.size == 0:
            return None, None, None, False
        med = float(np.median(a))
        p05, p95 = float(np.percentile(a, 5)), float(np.percentile(a, 95))
        return med, p05, p95, bool(p05 > 0.0 or p95 < 0.0)

    base_arr = np.asarray(totals[baseline.label], dtype=float)

    results: list[OptionResult] = []
    res_floor = 0.0
    for o in options:
        arr = np.asarray(totals[o.label], dtype=float) - base_arr
        med, p05, p95, stable = _band(arr)
        if med is None:
            results.append(OptionResult(o.label, None, None, None, False,
                                        o.acquisition_usd(), o.fab_hours(), None,
                                        nominal.get(o.label, {}),
                                        ["Lap sim produced no usable result."]))
            continue
        res_floor = max(res_floor, p95 - p05)
        actionable = bool(stable and abs(med) >= practical_floor_points)
        usd = o.acquisition_usd()
        upp = (usd / med) if (actionable and med > 0 and usd) else None
        results.append(OptionResult(
            label=o.label, delta_points_median=med,
            delta_points_p05=p05, delta_points_p95=p95, sign_stable=stable,
            acquisition_usd=usd, fab_hours=o.fab_hours(), usd_per_point=upp,
            nominal=nominal.get(o.label, {}), notes=sorted(all_notes[o.label]),
            actionable=actionable))

    # ---- PAIRWISE: the comparison that usually decides the purchase ------ #
    #  Beating the baseline is easy and often not the question. The question is
    #  whether the expensive candidate is separable from the cheap one. Doing
    #  this per-draw (paired) rather than by comparing two bands is the point:
    #  paired differences cancel the shared uncertainty, so this is a STRICTER
    #  test than "do their error bars overlap", not a looser one.
    pairwise: dict = {}
    labels = [o.label for o in options]
    for i, a in enumerate(labels):
        for b in labels[i + 1:]:
            diff = np.asarray(totals[a], dtype=float) - np.asarray(totals[b], dtype=float)
            med, p05, p95, stable = _band(diff)
            pairwise[(a, b)] = dict(
                median=med, p05=p05, p95=p95, sign_stable=stable,
                separable=bool(stable and med is not None
                               and abs(med) >= practical_floor_points))

    decidable = any(r.actionable for r in results)
    winners = [r for r in results
               if r.actionable and (r.delta_points_median or 0) > 0]

    if not decidable:
        txt = (f"NOT SEPARABLE — no winner declared. Across the swept band every "
               f"option's points delta versus '{baseline.label}' changes sign, "
               f"i.e. the difference between these parts is smaller than the "
               f"uncertainty in the inputs (mu, torque bias ratio, preload, lock "
               f"penalty, which corner you evaluate). The honest reading is that "
               f"THIS MODEL CANNOT TELL THEM APART at a magnitude worth acting "
               f"on (floor: {practical_floor_points:g} points), so no $/point is "
               f"printed. "
               f"Narrow the band with real data — tyre data, the vendor's own TBR "
               f"figure, a measured corner-exit case — or decide on the grounds "
               f"the model is not disputing: reliability, adjustability, "
               f"packaging, lead time and team hours.")
    elif not winners:
        l = min((r for r in results if r.actionable),
                key=lambda r: r.delta_points_median)
        txt = (f"SEPARABLE, AND NEGATIVE. Every separable option is WORSE than "
               f"'{baseline.label}'; '{l.label}' loses "
               f"{l.delta_points_median:+.1f} points. Do not buy on lap time.")
    else:
        w = max(winners, key=lambda r: r.delta_points_median)
        cost_bit = f" at ${w.usd_per_point:,.0f} per point" if w.usd_per_point else ""
        hrs_bit = f", plus {w.fab_hours:.0f} team-hours of fabrication" if w.fab_hours else ""
        txt = (f"SEPARABLE FROM THE BASELINE. '{w.label}' beats "
               f"'{baseline.label}' by {w.delta_points_median:+.1f} points "
               f"(90% band {w.delta_points_p05:+.1f} … {w.delta_points_p95:+.1f}); "
               f"the sign holds across the whole swept band{cost_bit}{hrs_bit}.")
        # is the winner actually distinguishable from the cheaper candidates?
        ties = [b for (a, b), v in pairwise.items()
                if a == w.label and not v["separable"]]
        ties += [a for (a, b), v in pairwise.items()
                 if b == w.label and not v["separable"]]
        if ties:
            cheaper = []
            for t in ties:
                r = next((x for x in results if x.label == t), None)
                if r and r.acquisition_usd and w.acquisition_usd and \
                        r.acquisition_usd < w.acquisition_usd:
                    cheaper.append(f"{t} (${r.acquisition_usd:,.0f}, "
                                   f"{r.fab_hours:.0f} h)")
            txt += (f"\n\nBUT IT IS NOT SEPARABLE FROM: {'; '.join(ties)}. "
                    f"Paired across every draw, the points difference between "
                    f"them changes sign, so this model cannot say the winner is "
                    f"actually the better part.")
            if cheaper:
                txt += (f" Cheaper and equally indistinguishable: "
                        f"{'; '.join(cheaper)}. On points alone there is no case "
                        f"for the more expensive one — the case has to be made on "
                        f"what the model is not simulating.")

    return TradeVerdict(
        baseline=baseline.label, results=results, decidable=decidable,
        resolution_points=float(res_floor), verdict_text=txt,
        pairwise=pairwise, provenance=PROVENANCE)


# --------------------------------------------------------------------------- #
#  4.  The FSAE Cost event — the only place dollars become points
# --------------------------------------------------------------------------- #
def cost_event_points(your_cost_usd: float,
                      min_cost_usd: float | None = None,
                      max_cost_usd: float | None = None,
                      pts_max: float = 100.0) -> float | None:
    """Published-form cost score. Returns None when the reference costs are not
    supplied — WITHHELD rather than guessed, because the year's Cmin/Cmax are
    the whole scale and inventing them would invent the answer."""
    if min_cost_usd is None or max_cost_usd is None:
        return None
    if not np.isfinite(your_cost_usd) or max_cost_usd <= min_cost_usd:
        return None
    frac = (max_cost_usd - your_cost_usd) / (max_cost_usd - min_cost_usd)
    return float(pts_max * min(max(frac, 0.0), 1.0))


def cost_event_delta(car_cost_usd: float, part_delta_usd: float,
                     min_cost_usd: float | None = None,
                     max_cost_usd: float | None = None,
                     pts_max: float = 100.0) -> float | None:
    """How many Cost-event points a price difference on ONE part moves. This is
    usually a fraction of a point, and seeing that is the point: it stops the
    Cost event being used to justify a decision it cannot carry."""
    a = cost_event_points(car_cost_usd, min_cost_usd, max_cost_usd, pts_max)
    b = cost_event_points(car_cost_usd + part_delta_usd,
                          min_cost_usd, max_cost_usd, pts_max)
    return None if (a is None or b is None) else float(b - a)


PROVENANCE = {
    "physics_grounded": [
        "lap times (real LapSimulator over the standard events)",
        "points curves (published-form FSAE event_points)",
        "rear-axle torque split (driveline.py, exact for a given TBR/preload)",
        "mass effect (exact — the part's mass enters the vehicle model)",
    ],
    "estimate_flagged": [
        "every band in Uncertainty — these are judgements, not measurements",
        "the lock/yaw penalty coefficient",
        "any vendor figure not confirmed against the vendor's own data",
        "Cost-event points, unless the year's Cmin/Cmax are supplied",
    ],
    "hard_rule": (
        "No winner and no $/point when the points delta changes sign across the "
        "swept band. A trade study that always produces a ranking is not a "
        "trade study, it is a random number generator with a house style."
    ),
}
