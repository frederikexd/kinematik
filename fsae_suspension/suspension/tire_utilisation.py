# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Instantaneous tire utilisation: how much of its traction ellipse each tire uses.

For a quasi-steady state (longitudinal and lateral acceleration in g) every
tire carries a vertical load Fz (target_derivation.corner_loads), a
longitudinal force Fx and a lateral force Fy. Its utilisation is

    u = sqrt((Fx / (kx mu Fz))^2 + (Fy / (mu Fz))^2)

with mu the load-sensitive peak friction of the tire model and kx the ratio
of longitudinal to lateral peak friction (declared: the synthetic tire is
pure-lateral). u = 1 is the edge of the ellipse; 1 - u is the margin; the
spare grip is the extra force the tire could take in its present direction,
|F| (1/u - 1). Units: forces in N, accelerations in g, angles in deg,
utilisation and margins dimensionless.

Force allocation:
- lateral: steady-state yaw balance splits m a_y between the axles by the
  CG position; within an axle both tires run a common slip angle, solved on
  the tire model, with each tire's lateral force scaled by the friction
  ellipse for its own Fx (combined-slip approximation);
- braking: the declared front share, split equally left and right (one line
  pressure per axle);
- traction: rear only, split equally (spool, declared).
"""
from __future__ import annotations

import math

import numpy as np

from .target_derivation import Vehicle, corner_loads
from .tiremodel import default_tire

G = 9.81
TIRES = ("FL", "FR", "RL", "RR")


def _fy_combined(tire, alpha_deg: float, fz: float, fx: float, kx: float, gamma: float) -> float:
    """Lateral force magnitude (N) at slip angle alpha (deg), reduced by the friction ellipse for Fx.

    The tire model follows the SAE sign (negative Fy for positive slip); the
    magnitude is used here and the sign restored by the caller.
    """
    mu = tire.mu_peak(fz, gamma)
    r = min(1.0, abs(fx) / (kx * mu * fz)) if fz > 0 else 1.0
    return abs(float(tire.fy(math.radians(alpha_deg), fz, gamma))) * math.sqrt(max(0.0, 1.0 - r * r))


def _axle_split(tire, fy_req: float, fz: tuple, fx: tuple, kx: float, gamma: float):
    """Common slip angle (deg) and per-tire Fy (N) meeting an axle lateral force (N).

    Returns None when the axle cannot carry ``fy_req`` at any slip angle.
    """
    sgn = 1.0 if fy_req >= 0 else -1.0; need = abs(fy_req)
    tot = lambda a: sum(_fy_combined(tire, a, z, x, kx, gamma) for z, x in zip(fz, fx))
    grid = np.linspace(0.0, 20.0, 201); vals = [tot(a) for a in grid]
    k = int(np.argmax(vals))
    if vals[k] < need: return None
    lo, hi = 0.0, float(grid[k])
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if tot(mid) < need: lo = mid
        else: hi = mid
    a = 0.5 * (lo + hi)
    return sgn * a, [sgn * _fy_combined(tire, a, z, x, kx, gamma) for z, x in zip(fz, fx)]


def tire_state(ax_g: float, ay_g: float, veh: Vehicle | None = None, tire=None, brake_front: float = 0.60,
               kx: float = 1.0, gamma: float = 0.0, **load_kw) -> dict:
    """Per-tire loads, forces, utilisation, margin and spare grip at one instant.

    ``ax_g`` > 0 is braking, < 0 traction; ``ay_g`` > 0 a left turn (load to
    the right-hand tires). Returns a dict per tire plus ``feasible`` (False
    when an axle cannot carry its share: the car is past its limit).
    """
    veh = veh or Vehicle(); tire = tire or default_tire()
    m = veh.mass_kg if hasattr(veh, "mass_kg") else 300.0
    wf = load_kw.pop("weight_front", 0.48)
    fz = corner_loads(veh, a_lat_g=ay_g, a_long_g=ax_g, weight_front=wf, **load_kw)["total"]
    Fx_tot = m * G * ax_g
    if ax_g >= 0:
        fx = {"FL": -brake_front * Fx_tot / 2, "FR": -brake_front * Fx_tot / 2, "RL": -(1 - brake_front) * Fx_tot / 2, "RR": -(1 - brake_front) * Fx_tot / 2}
    else:
        fx = {"FL": 0.0, "FR": 0.0, "RL": -Fx_tot / 2, "RR": -Fx_tot / 2}
    Fy_tot = m * G * ay_g
    fy_axle = {"F": Fy_tot * wf, "R": Fy_tot * (1.0 - wf)}          # steady yaw balance: front carries b/L = wf of m a_y
    out, feasible = {}, True
    for ax, pair in (("F", ("FL", "FR")), ("R", ("RL", "RR"))):
        res = _axle_split(tire, fy_axle[ax], tuple(fz[t] for t in pair), tuple(fx[t] for t in pair), kx, gamma)
        if res is None: feasible = False; fys = [fy_axle[ax] / 2] * 2; alpha = float("nan")
        else: alpha, fys = res
        for t, fyv in zip(pair, fys):
            mu = tire.mu_peak(fz[t], gamma); u = math.hypot(fx[t] / (kx * mu * fz[t]), fyv / (mu * fz[t]))
            F = math.hypot(fx[t], fyv)
            out[t] = {"Fz": fz[t], "Fx": fx[t], "Fy": fyv, "mu": mu, "alpha_deg": alpha, "u": u,
                      "margin": 1.0 - u, "spare_n": F * (1.0 / u - 1.0) if u > 0 else kx * mu * fz[t]}
    out["feasible"] = feasible
    out["vehicle_u"] = max(out[t]["u"] for t in TIRES)
    out["grip_used_n"] = sum(math.hypot(out[t]["Fx"], out[t]["Fy"]) for t in TIRES)
    out["grip_spare_n"] = sum(out[t]["spare_n"] for t in TIRES)
    return out


def limit_envelope(n_dir: int = 72, veh: Vehicle | None = None, tire=None, **kw) -> list[dict]:
    """The vehicle's quasi-steady g-g limit: the largest acceleration in each
    direction before the first tire reaches the edge of its ellipse, with every
    tire's utilisation there. Directions are angles from pure braking (0 deg)
    through left cornering (90 deg) to traction (180 deg); g, deg, dimensionless.
    """
    tire = tire or default_tire(); rows = []
    for k in range(n_dir + 1):
        th = math.radians(180.0 * k / n_dir)
        lo, hi = 0.0, 2.5
        for _ in range(40):
            mid = 0.5 * (lo + hi); s = tire_state(mid * math.cos(th), mid * math.sin(th), veh, tire, **kw)
            if s["feasible"] and s["vehicle_u"] <= 1.0: lo = mid
            else: hi = mid
        s = tire_state(lo * math.cos(th), lo * math.sin(th), veh, tire, **kw)
        rows.append({"theta_deg": math.degrees(th), "a_g": lo, "ax_g": lo * math.cos(th), "ay_g": lo * math.sin(th),
                     **{f"u_{t}": s[t]["u"] for t in TIRES}, "limiting": max(TIRES, key=lambda t: s[t]["u"])})
    return rows
