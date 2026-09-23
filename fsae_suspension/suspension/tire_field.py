# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Tire inputs from tests a team can run on its own car, without tire data.

The target chain uses three tire numbers: peak friction, camber optimum and
camber sensitivity. The sensitivity study shows that only peak friction can
move a target past a reported corner, and that the camber optimum only moves
the alignment. Both can be measured at car level, on a skidpad, with no
tire-test data:

* ``mu_from_skidpad``: steady lateral acceleration on a circle of known radius,
  from lap time or from a logged accelerometer, corrected for any downforce.
  This is the vehicle-level friction that actually sets the design
  accelerations, which is what the target chain needs.
* ``camber_optimum_from_sweep``: lap time at three or more static cambers,
  fitted with a parabola; the minimum is the static camber at which the loaded
  outside tire sits at its optimum, and the corner's own gain and roll carry it
  to the tire's loaded optimum.
* ``tire_rate_from_load_test``: vertical load against deflection on the car's
  own tire, the slope is the tire rate, which sets the wheel-hop band.

``update_tire`` turns those results into a ``target_derivation.Tire`` and
re-runs the targets, reporting which moved and whether the reported corners
still meet them. Camber sensitivity stays declared: it sets no target.

Units: m and s for the skidpad, g for accelerations, deg, N, mm, N/mm, Hz.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Sequence

import numpy as np

from . import target_derivation as td

G0 = 9.81


def mu_from_skidpad(radius_m: float, lap_times_s: Sequence[float] | None = None,
                    ay_g: Sequence[float] | None = None, mass_kg: float = 300.0,
                    downforce_ClA_m2: float = 0.0, rho: float = 1.225) -> dict:
    """Vehicle-level peak friction (dimensionless) from steady skidpad running.

    Give ``lap_times_s`` (s per lap on the driven radius ``radius_m``) or logged
    steady ``ay_g`` (g). Downforce, if any, is removed so the result is friction
    rather than grip per unit weight: mu = m a_y / (m g + 0.5 rho ClA v^2).
    Uses the best (fastest or highest) run, and reports the spread.
    """
    if lap_times_s:
        t = np.asarray(lap_times_s, float)
        if np.any(t <= 0.0):
            raise ValueError("lap times must be > 0 s")
        v = 2.0 * math.pi * radius_m / t
        ay = v ** 2 / radius_m / G0
    elif ay_g:
        ay = np.asarray(ay_g, float)
        v = np.sqrt(ay * G0 * radius_m)
    else:
        raise ValueError("give lap_times_s or ay_g")
    down = 0.5 * rho * downforce_ClA_m2 * v ** 2
    mu = mass_kg * ay * G0 / (mass_kg * G0 + down)
    i = int(np.argmax(mu))
    return {"mu": float(mu[i]), "ay_g": float(ay[i]), "speed_m_s": float(v[i]),
            "mu_spread": float(mu.max() - mu.min()), "n_runs": int(mu.size)}


def camber_optimum_from_sweep(static_camber_deg: Sequence[float],
                              lap_times_s: Sequence[float]) -> dict:
    """Static camber (deg) that minimises skidpad lap time, by parabola fit.

    Needs three or more settings that bracket the minimum; refuses a fit whose
    minimum lies outside the tested range, since that is an extrapolation.
    """
    c = np.asarray(static_camber_deg, float)
    t = np.asarray(lap_times_s, float)
    if c.size < 3:
        raise ValueError("need at least three camber settings")
    a, b, k = np.polyfit(c, t, 2)
    if a <= 0.0:
        raise ValueError("lap time has no minimum in this sweep (not convex)")
    c_opt = -b / (2.0 * a)
    if not (c.min() <= c_opt <= c.max()):
        raise ValueError(f"fitted optimum {c_opt:.2f} deg lies outside the tested "
                         f"range {c.min():.2f} to {c.max():.2f}; widen the sweep")
    resid = t - np.polyval([a, b, k], c)
    return {"static_camber_opt_deg": float(c_opt),
            "lap_time_at_opt_s": float(np.polyval([a, b, k], c_opt)),
            "fit_rms_s": float(np.sqrt(np.mean(resid ** 2)))}


def tire_rate_from_load_test(loads_N: Sequence[float],
                             deflections_mm: Sequence[float],
                             unsprung_corner_kg: float | None = None,
                             wheel_rate_N_per_mm: float = 0.0) -> dict:
    """Tire vertical rate (N/mm) as the slope of load against deflection.

    With ``unsprung_corner_kg`` the wheel-hop frequency (Hz) follows, using the
    wheel rate (N/mm) in parallel.
    """
    f = np.asarray(loads_N, float); d = np.asarray(deflections_mm, float)
    if f.size < 2:
        raise ValueError("need at least two load points")
    k, c = np.polyfit(d, f, 1)
    out = {"tire_rate_N_per_mm": float(k),
           "fit_rms_N": float(np.sqrt(np.mean((f - np.polyval([k, c], d)) ** 2)))}
    if unsprung_corner_kg:
        out["wheel_hop_hz"] = float(math.sqrt((k + wheel_rate_N_per_mm) * 1000.0
                                              / unsprung_corner_kg) / (2 * math.pi))
    return out


@dataclass
class FieldTire:
    """The measured car-level tire inputs; None where a test was not run."""
    peak_mu: float | None = None
    static_camber_opt_deg: float | None = None
    camber_gain_deg_per_mm: float | None = None   # of the tested corner
    skidpad_ay_g: float | None = None


def update_tire(field: FieldTire, base: td.Tire = td.Tire(),
                veh: td.Vehicle = td.Vehicle()) -> td.Tire:
    """A ``target_derivation.Tire`` with the measured inputs substituted.

    Peak mu is dimensionless; cambers in deg; gain in deg/mm; lateral
    acceleration in g.

    Peak mu replaces the synthetic value directly. The camber optimum comes
    from the sweep: at the skidpad's lateral acceleration the loaded outside
    tire sat at static + gain * z + phi, and at the lap-time minimum that is
    the optimum. Camber sensitivity stays declared; it sets no target.
    """
    t = base
    if field.peak_mu is not None:
        t = replace(t, peak_mu=float(field.peak_mu))
    if field.static_camber_opt_deg is not None:
        if field.camber_gain_deg_per_mm is None or field.skidpad_ay_g is None:
            raise ValueError("the camber optimum needs the tested corner's gain "
                             "and the skidpad lateral acceleration")
        phi, z = td.roll(veh, field.skidpad_ay_g)
        t = replace(t, camber_opt_deg=float(field.static_camber_opt_deg
                                            + field.camber_gain_deg_per_mm * z + phi))
    return t


def compare_targets(new: td.Tire, corners: Sequence[td.Corner],
                    base: td.Tire = td.Tire(), veh: td.Vehicle = td.Vehicle()) -> dict:
    """Targets under the synthetic and the measured tire, and each corner's standing.

    Anti floors and margins in percent; camber error and restoring static camber
    in deg.
    """
    a, b = td.targets(veh, base), td.targets(veh, new)
    out = {"targets": {k: {"synthetic": a[k], "measured": b[k]} for k in a},
           "corners": {}}
    for c in corners:
        floor = b["anti_dive_floor_pct"] if c.axle == "front" else b["anti_squat_floor_pct"]
        err = td.loaded_camber_error(veh, new, c.static_camber_deg, c.camber_gain_deg_per_mm)
        out["corners"][c.name] = {
            "anti_margin_pct": c.anti_pct - floor,
            "camber_error_deg": err,
            "static_to_restore_deg": td.static_camber_needed(veh, new, c.camber_gain_deg_per_mm)}
    return out


def mu_threshold_for(axle_key: str, anti_pct: float, veh: td.Vehicle = td.Vehicle(),
                     base: td.Tire = td.Tire()) -> float:
    """Peak mu (dimensionless) above which a corner's anti no longer clears its floor."""
    lo, hi = 0.5, 3.0
    for _ in range(80):
        m = 0.5 * (lo + hi)
        f = td.targets(veh, replace(base, peak_mu=m))[axle_key]
        lo, hi = (m, hi) if f < anti_pct else (lo, m)
    return lo
