# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Dynamic Ackermann: the steer split that maximises front axle force.

Geometric Ackermann points both front wheels at one turn centre, which is
right at zero slip. At the limit the two tires carry different loads, and a
load-sensitive tire reaches its peak force at a different slip angle on each:
a real tire's peak slip rises with load, so the heavily loaded outer tire
wants MORE slip than the inner one. In a steady turn both wheels share the
axle's sideslip and yaw terms, so the steer DIFFERENCE between them is

    delta_i - delta_o = (geometric difference) + (alpha_i - alpha_o)

and the split that puts each tire at its own peak is the geometric one plus
the difference of the peak slips. Expressed as an Ackermann percentage of the
geometric difference, that is the dynamic target. This module computes it
from the tire model and the axle loads at the design case, reads the split a
corner's linkage actually gives at a rack position, and prices the gap as
front axle force lost against the optimum.

Sign conventions: steer angles positive toward the turn; the inner wheel is
the one on the turn side. For a right-side corner in KinematiK's frame
(x rearward, y outboard), positive toe points the wheel outboard, which for
the right wheel is a right turn.

Scope: steady state at one radius and lateral acceleration, pure lateral
slip, the tire model as declared (synthetic until measured). Units: mm, N,
deg unless named.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .kinematics import Hardpoints, SuspensionKinematics


def geometric_split(delta_outer_deg: float, wheelbase_mm: float,
                    kingpin_track_mm: float) -> float:
    """Inner steer (deg) that geometric (100%) Ackermann gives for an outer steer."""
    do = math.radians(abs(delta_outer_deg))
    if do < 1e-9:
        return 0.0
    cot_i = 1.0 / math.tan(do) - kingpin_track_mm / wheelbase_mm
    return math.degrees(math.atan(1.0 / cot_i))


def ackermann_pct(delta_inner_deg: float, delta_outer_deg: float,
                  wheelbase_mm: float, kingpin_track_mm: float) -> float:
    """Ackermann percentage of a steer pair: 100 geometric, 0 parallel."""
    gi = geometric_split(delta_outer_deg, wheelbase_mm, kingpin_track_mm)
    geo = gi - abs(delta_outer_deg)
    if abs(geo) < 1e-12:
        return float("nan")
    return 100.0 * (abs(delta_inner_deg) - abs(delta_outer_deg)) / geo


def peak_slip_deg(tire, fz_n: float, camber_deg: float = 0.0,
                  lo: float = 0.5, hi: float = 20.0) -> float:
    """Slip angle (deg) of peak |Fy|, refined to 0.01 deg."""
    a = np.radians(np.linspace(lo, hi, 391))
    fy = np.abs(tire.fy(a, fz_n, math.radians(camber_deg)))
    k = int(np.argmax(fy))
    a2 = np.linspace(a[max(k - 1, 0)], a[min(k + 1, len(a) - 1)], 201)
    fy2 = np.abs(tire.fy(a2, fz_n, math.radians(camber_deg)))
    return float(np.degrees(a2[int(np.argmax(fy2))]))


@dataclass
class AxleCase:
    """The front axle at the design case. Loads N, lengths mm, angles deg."""
    fz_outer_n: float
    fz_inner_n: float
    radius_mm: float = 4500.0          # path radius of the CG
    wheelbase_mm: float = 1630.0
    kingpin_track_mm: float = 1210.0
    camber_outer_deg: float = 0.0
    camber_inner_deg: float = 0.0

    @staticmethod
    def from_vehicle(mass_kg: float = 300.0, weight_front: float = 0.48,
                     cg_height_mm: float = 280.0, track_mm: float = 1210.0,
                     front_share: float = 0.53, a_lat_g: float = 1.5,
                     **kw) -> "AxleCase":
        """Outer and inner front loads from the balance target of the paper."""
        g = 9.81
        stat = mass_kg * g * weight_front / 2.0
        dfz = front_share * mass_kg * g * a_lat_g * cg_height_mm / track_mm
        return AxleCase(stat + dfz, max(stat - dfz, 1.0),
                        kingpin_track_mm=track_mm, **kw)


def dynamic_target(tire, case: AxleCase) -> dict:
    """The steer split that puts each front tire at its own peak slip.

    Returns the peak slips (deg), the geometric outer/inner steer for the
    radius, the dynamic inner/outer steer difference and the Ackermann
    percentage it amounts to (below 100 when the outer tire wants more slip).
    """
    ao = peak_slip_deg(tire, case.fz_outer_n, case.camber_outer_deg)
    ai = peak_slip_deg(tire, case.fz_inner_n, case.camber_inner_deg)
    L, t, R = case.wheelbase_mm, case.kingpin_track_mm, case.radius_mm
    do_geo = math.degrees(math.atan(L / (R + t / 2.0)))
    di_geo = math.degrees(math.atan(L / (R - t / 2.0)))
    geo = di_geo - do_geo
    dyn = geo + (ai - ao)
    return {"peak_slip_outer_deg": ao, "peak_slip_inner_deg": ai,
            "geometric_outer_deg": do_geo, "geometric_inner_deg": di_geo,
            "geometric_difference_deg": geo, "dynamic_difference_deg": dyn,
            "dynamic_ackermann_pct": 100.0 * dyn / geo}


def axle_force_at_split(tire, case: AxleCase, split_deg: float,
                        n: int = 400) -> dict:
    """Best front axle lateral force (N) when the steer difference is fixed.

    With the inner-minus-outer steer fixed at ``split_deg``, the slips obey
    alpha_i - alpha_o = split - geometric difference; the axle's sideslip
    sets their common level, so the best force is the maximum over it.
    Returns that force, the unconstrained optimum and the loss in percent.
    """
    t = dynamic_target(tire, case)
    d = split_deg - t["geometric_difference_deg"]
    ao = np.linspace(0.5, 20.0, n)
    ai = ao + d
    ok = ai > 0.0
    co, ci = math.radians(case.camber_outer_deg), math.radians(case.camber_inner_deg)
    F = (np.abs(tire.fy(np.radians(ao[ok]), case.fz_outer_n, co))
         + np.abs(tire.fy(np.radians(ai[ok]), case.fz_inner_n, ci)))
    best = float(np.max(F))
    opt = float(abs(tire.fy(math.radians(t["peak_slip_outer_deg"]), case.fz_outer_n, co))
                + abs(tire.fy(math.radians(t["peak_slip_inner_deg"]), case.fz_inner_n, ci)))
    return {"axle_force_n": best, "optimum_n": opt,
            "loss_pct": 100.0 * (1.0 - best / opt) if opt > 0 else float("nan")}


def kinematic_split(hp: Hardpoints, rack_mm: float) -> dict:
    """Outer and inner road-wheel steer (deg) a mirrored axle gives at a rack.

    The right corner steers by its toe change at +rack; the mirrored left
    corner at the same rack sees its tie-rod inner move inboard, which is the
    right corner at -rack, with the sign flipped. The wheel on the turn side
    is the inner one.
    """
    toe0 = SuspensionKinematics(hp).solve_at_travel(0.0).toe

    def toe_at(r):
        """Road-wheel toe in deg at a rack travel in mm."""
        k = SuspensionKinematics(hp, pickup_deltas={
            "tie_rod_inner": np.array([0.0, r, 0.0])})
        return k.solve_at_travel(0.0).toe

    dR = toe_at(rack_mm) - toe0                    # right wheel, + = right turn
    dL = -(toe_at(-rack_mm) - toe0)                # left wheel, + = right turn
    if dR + dL >= 0.0:                             # right turn: right is inner
        inner, outer = dR, dL
    else:                                          # left turn: left is inner
        inner, outer = -dL, -dR
    return {"inner_deg": float(inner), "outer_deg": float(outer),
            "difference_deg": float(inner - outer)}
