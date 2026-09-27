# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Limit feel: aligning torque with pneumatic trail that collapses at the limit.

What the driver feels through the wheel near the grip limit is the aligning
torque about the steering axis, Fy times the total trail. The total is the
mechanical trail the caster sets plus the tire's pneumatic trail, and the
pneumatic trail DECAYS with slip: it is largest at small slip and falls
through zero near the slip of peak lateral force. So as the tire saturates
the torque first peaks and then lightens, and that lightening is the limit
cue. How much torque is left AT the limit is set by the mechanical trail
alone.

MODEL
-----
Pneumatic trail t_p(alpha) = t0 cos((pi / 2) alpha / alpha0), going negative
past alpha0, with alpha0 the slip of peak lateral force at the load (the
usual approximation that the trail crosses zero at the force peak) and t0
from brush theory, one sixth of the contact-patch length, l = Fz / (p w).
Everything declared: a measured Mz curve replaces it. Aligning torque
M(alpha) = |Fy(alpha)| (t_mech + t_p(alpha)).

METRICS
-------
torque at the limit M(alpha0) = |Fy_peak| t_mech; the peak torque and its
slip; the WARNING MARGIN alpha0 - alpha(M max), how many degrees before the
limit the wheel starts to lighten; and the LIGHTENING 1 - M(alpha0) / M max.
A small mechanical trail gives a large, early lightening (a clear cue) but
little torque left at the limit (a light wheel to hold); a large one gives
more torque at the limit and a weaker cue. Units: mm, N, deg, N*m.
"""
from __future__ import annotations

import math

import numpy as np


def brush_trail_mm(fz_n: float, inflation_kpa: float = 83.0,
                   patch_width_mm: float = 180.0) -> float:
    """Zero-slip pneumatic trail (mm) from brush theory: patch length / 6."""
    length = fz_n / (inflation_kpa * 1e-3 * patch_width_mm)      # mm
    return length / 6.0


def mechanical_trail_mm(caster_deg: float, loaded_radius_mm: float = 228.0,
                        caster_offset_mm: float = 0.0) -> float:
    """Ground-level mechanical trail (mm): R tan(caster) minus spindle offset."""
    return loaded_radius_mm * math.tan(math.radians(caster_deg)) - caster_offset_mm


def aligning_curve(tire, fz_n: float, t_mech_mm: float,
                   t0_mm: float | None = None, alpha_max_deg: float = 14.0,
                   n: int = 281) -> dict:
    """Aligning torque about the steering axis (N*m) against slip (deg)."""
    from .dynamic_ackermann import peak_slip_deg
    a0 = peak_slip_deg(tire, fz_n)
    t0 = brush_trail_mm(fz_n) if t0_mm is None else float(t0_mm)
    a = np.linspace(0.0, alpha_max_deg, n)
    fy = np.abs(tire.fy(np.radians(a), fz_n))
    tp = t0 * np.cos(0.5 * np.pi * a / a0)
    M = fy * (t_mech_mm + tp) / 1000.0
    return {"alpha_deg": a, "M_Nm": M, "Fy_N": fy, "t_pneu_mm": tp,
            "alpha_peak_deg": a0, "t0_mm": t0}


def limit_feel(tire, fz_n: float, t_mech_mm: float,
               t0_mm: float | None = None) -> dict:
    """The feel metrics of the module docstring for one wheel."""
    c = aligning_curve(tire, fz_n, t_mech_mm, t0_mm)
    a, M = c["alpha_deg"], c["M_Nm"]
    k = int(np.argmax(M))
    a0 = c["alpha_peak_deg"]
    M_lim = float(np.interp(a0, a, M))
    return {"alpha_peak_deg": a0, "t0_mm": c["t0_mm"],
            "torque_max_Nm": float(M[k]), "alpha_torque_max_deg": float(a[k]),
            "torque_at_limit_Nm": M_lim,
            "warning_margin_deg": float(a0 - a[k]),
            "lightening": float(1.0 - M_lim / M[k]) if M[k] > 0 else float("nan")}


def worst_axle_torque(tire, fz_outer_n: float, fz_inner_n: float,
                      t_mech_mm: float, t0_outer_mm: float | None = None) -> dict:
    """Largest combined aligning torque of both front wheels (N*m) over slip.

    Both wheels at the same slip; each with its own load and a brush-theory
    trail (the inner scaled by load when ``t0_outer_mm`` is given).
    """
    t0o = brush_trail_mm(fz_outer_n) if t0_outer_mm is None else float(t0_outer_mm)
    t0i = t0o * fz_inner_n / fz_outer_n
    co = aligning_curve(tire, fz_outer_n, t_mech_mm, t0o)
    ci = aligning_curve(tire, fz_inner_n, t_mech_mm, t0i)
    tot = co["M_Nm"] + np.interp(co["alpha_deg"], ci["alpha_deg"], ci["M_Nm"])
    k = int(np.argmax(tot))
    return {"torque_Nm": float(tot[k]), "alpha_deg": float(co["alpha_deg"][k])}


def ratio_window(tire, fz_outer_n: float, fz_inner_n: float, t_mech_mm: float,
                 effort_limit_Nm: float = 10.0, lock_ratio_max: float = 6.0,
                 t0_outer_mm: float | None = None) -> dict:
    """The fixed steering ratios that meet both driver effort and lock.

    Effort needs ratio >= worst aligning torque / effort limit (the worst over
    slip, not at the limit, because pneumatic trail peaks before the force
    does); lock needs ratio <= ``lock_ratio_max``. Feasible when the first is
    below the second.
    """
    w = worst_axle_torque(tire, fz_outer_n, fz_inner_n, t_mech_mm, t0_outer_mm)
    lo = w["torque_Nm"] / effort_limit_Nm
    return {"ratio_min": lo, "ratio_max": lock_ratio_max,
            "feasible": lo <= lock_ratio_max, "worst_torque_Nm": w["torque_Nm"],
            "worst_alpha_deg": w["alpha_deg"]}


def trail_limit_mm(tire, fz_outer_n: float, fz_inner_n: float, t_mech_mm: float,
                   effort_limit_Nm: float = 10.0, lock_ratio_max: float = 6.0,
                   hi: float = 80.0) -> float:
    """Zero-slip pneumatic trail (mm, outer wheel) at which the window closes."""
    lo = 0.0
    if ratio_window(tire, fz_outer_n, fz_inner_n, t_mech_mm, effort_limit_Nm,
                    lock_ratio_max, hi)["feasible"]:
        return float("inf")
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        ok = ratio_window(tire, fz_outer_n, fz_inner_n, t_mech_mm, effort_limit_Nm,
                          lock_ratio_max, mid)["feasible"]
        lo, hi = (mid, hi) if ok else (lo, mid)
    return 0.5 * (lo + hi)
