# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
genesis_seed.py — find a start inside the walls when none of the drawn ones is.

InverseGenesis's walls are absolute: a start that violates one is refused,
and a search never walks in from outside (inverse_genesis, PropertyBound).
When every drawn start is refused, the honest answer is NO_FIT. This module
is the step an engineer takes next, and a manifest can declare it
(GenesisManifest.start_search) so that a re-run repeats it: a deterministic pattern search over the
same free pickups, inside the same boxes, on a cost that is the worst band
fraction plus a penalty for every wall the geometry breaks, so it can move
through infeasible geometry toward the walls' inside. Its result is only a
start: InverseGenesis is then re-run from it with every wall absolute, and
that run is the manifest.

The settings and the evaluation count are returned so the manifest context
can carry them, and the search is deterministic for a given seed.
Lengths in mm; the cost is dimensionless.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

from . import inverse_genesis as ig

__all__ = ["SeedSearch", "seed_cost", "seed_search", "walls_broken"]


@dataclass
class SeedSearch:
    """Pattern-search settings (mm, counts)."""
    seed: int = 1
    max_evals: int = 600
    step0_mm: float = 16.0
    step_min_mm: float = 0.5
    #: wheel clearance beyond the envelope's own wall below which a penalty applies
    clearance_target_mm: float = 2.0
    #: stop once every wall holds and the worst band fraction is at most this
    stop_band_frac: float = 0.6
    #: walls only: the cost is the wall penalty alone and the search stops as soon as every wall holds, wherever the
    #: curves are, so InverseGenesis has to bring the curves into their bands with every wall absolute
    walls_only: bool = False
    #: keep-outs, spacings and bolt access penalised by depth (1 per 10 mm, at most 10) so the search can walk out of
    #: them; False gives every breach a flat 10, the setting of the searches published before this option existed
    graded_keepouts: bool = True


def _bound_scale(b) -> float:
    vals = [abs(v) for v in (b.lo, b.hi) if math.isfinite(v)]
    return max(0.1 * max(vals), 1e-3) if vals else 1.0


def seed_cost(hp, targets, volume, clearance_target_mm: float = 2.0,
              graded_keepouts: bool = True) -> tuple[float, dict]:
    """(cost, detail) of one geometry: worst band fraction plus wall penalties (dimensionless; clearance in mm).

    Each violated property bound adds its violation over a tenth of the bound
    value (capped at 10); a non-finite property adds 10; wheel clearance below
    the target adds the shortfall over the target; keep-out, spacing and bolt-
    access breaches add their summed depth over 10 mm (at least 0.0001 each,
    capped at 10), or a flat 10 with ``graded_keepouts=False``.
    """
    r, ok = targets.residual(hp)
    if not ok:
        return math.inf, {"reason": "does not solve"}
    band = float(np.max(np.abs(r)))
    pen, why = 0.0, []
    ko = volume.keepout_violations(hp)
    if ko:
        if graded_keepouts:
            depth = 0.0
            for p, _lab, c in ko:
                own = "->" in p or p.endswith("(bolt access)")      # spacing gap or access clearance: 0 is the wall
                depth += max((-c) if own else (volume.min_clearance_mm - c), 1e-3)
            pen += min(depth / 10.0, 10.0)
        else:
            pen += 10.0
        why.append("keep-out")
    if volume.has_property_bounds():
        vals = volume.properties.evaluate(hp)
        if vals is None:
            return math.inf, {"reason": "properties do not solve"}
        for b in volume.properties.bounds:
            m = b.margin(vals.get(b.prop, float("nan")))
            if m < 0:
                pen += 10.0 if not math.isfinite(m) else min(-m / _bound_scale(b), 10.0)
                why.append(b.prop)
    if volume.has_wheel_envelope() and volume.wheel_envelope.joint_swing_limit_deg is not None:
        lim = float(volume.wheel_envelope.joint_swing_limit_deg)
        for item, _, m in volume.envelope_violations(hp):
            if item.endswith("joint swing") or item == "(joints)":
                pen += 10.0 if not math.isfinite(m) else min(-m / (0.1 * lim), 10.0)
                why.append(item)
    em = volume.envelope_margin(hp)
    if em is not None and em < clearance_target_mm:
        pen += min((clearance_target_mm - em) / clearance_target_mm, 10.0)
        why.append(f"clearance {em:.2f} mm")
    return band + pen, {"band": band, "penalty": pen, "violated": why,
                        "clearance_mm": em}


def walls_broken(hp, targets, volume) -> list[str]:
    """The walls a start breaks (empty when it is inside all of them): the properties out of their bounds, keep-outs,
    joint swing and wheel clearance (mm) below the envelope's own wall. Used to decide whether a run needs a start
    search before InverseGenesis, which refuses such a start."""
    c, d = seed_cost(hp, targets, volume, clearance_target_mm=0.0)
    if not math.isfinite(c):
        return [d.get("reason", "does not solve")]
    return list(d["violated"]) if d["penalty"] > 0.0 else []


def seed_search(hp, targets, volume, settings: SeedSearch = SeedSearch(), log=None):
    """Pattern search for a start that meets every wall.

    Returns (hardpoints, report). The report carries the settings, evaluation
    count, start and final cost and whether every wall holds at the end.
    """
    coords = volume.coords()
    rng = np.random.default_rng(settings.seed)
    x = np.zeros(len(coords))

    def _cost(h):
        c, d = seed_cost(h, targets, volume, settings.clearance_target_mm, settings.graded_keepouts)
        if settings.walls_only and math.isfinite(c):
            c = d["penalty"]
        return c, d

    best, det = _cost(ig._shifted(hp, volume, x))
    start = (best, det)
    n_eval, step = 1, float(settings.step0_mm)

    def _feasible(d):
        if settings.walls_only:
            return d.get("penalty", 1.0) == 0.0
        return d.get("penalty", 1.0) == 0.0 and d.get("band", 2.0) <= 1.0

    def _done(d):
        return _feasible(d) and (settings.walls_only or d["band"] <= settings.stop_band_frac)

    while step >= settings.step_min_mm and n_eval < settings.max_evals and not _done(det):   # a start already done stays
        improved = False
        for i in rng.permutation(len(coords)):
            for sgn in (1.0, -1.0):
                if n_eval >= settings.max_evals:
                    break
                trial = x.copy(); trial[i] += sgn * step
                trial, _ = volume.clamp(hp, trial)
                if np.allclose(trial, x):
                    continue
                c, d = _cost(ig._shifted(hp, volume, trial))
                n_eval += 1
                if c < best - 1e-9:
                    x, best, det, improved = trial, c, d, True
                    if log:
                        log(f"eval {n_eval}: cost {best:.4f} band {d['band']:.3f} "
                            f"pen {d['penalty']:.3f} step {step:g} {d['violated']}")
                    break
            if _done(det):
                break
        if _done(det):
            break
        if not improved:
            step *= 0.5
    report = {"settings": asdict(settings), "evaluations": n_eval,
              "start_cost": start[0], "start_violated": start[1].get("violated"),
              "final_cost": best, "final_band": det.get("band"),
              "final_violated": det.get("violated"),
              "final_clearance_mm": det.get("clearance_mm"),
              "walls_hold": _feasible(det),             # every wall holds and (unless walls_only) every curve in band
              "walls_met": det.get("penalty", 1.0) == 0.0,   # every wall holds, wherever the curves are
              "shift_mm": {f"{p}.{'xyz'[a]}": float(v) for (p, a), v in zip(coords, x) if v}}
    return ig._shifted(hp, volume, x), report
