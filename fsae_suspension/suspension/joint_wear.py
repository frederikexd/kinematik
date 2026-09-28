# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Joint wear: how the clearance grows, and when it eats the band.

A new car's joints have their catalogue clearance. Every suspension cycle
slides the ball against its liner under load, and the clearance grows. This
module predicts that growth with Archard's law and turns it into the channel
deadband of ``elastokinematics.lash_deadband``, so the question "after how
many kilometres does joint wear consume the toe band" has a number, and the
inspection interval can be set from it rather than from habit.

    wear depth per cycle  h = K * p * s,  p = F / (d * w),  s = r * theta
    clearance growth      = 2 h per cycle (the ball wears on both sides of
                            its travel as the load reverses)

with K the specific wear rate of the liner (mm^3 / (N m)), F the joint load
amplitude (N), d x w the projected bearing area (mm^2), r the ball radius
(mm) and theta the swing per cycle (rad). Cycles come from a declared count
per kilometre at that amplitude; a duty cycle of several amplitudes is a list.

PROVENANCE. The wear rate is the whole answer and it is not a property this
module can know: PTFE-lined spherical bearings are quoted over roughly
1e-7 to 1e-6 mm^3/(N m), bare steel-on-steel an order higher, and
contamination and missing boots move it further. The default is flagged
DECLARED. Measuring clearance at intervals (the paper's CMM re-inspection)
and fitting K is what turns this into a prediction; ``fit_wear_rate`` does
that from two or more measurements.

Scope: sliding wear only; no fretting, corrosion, bracket set or tube
relaxation, which are measured, not predicted. Units mm, N, km.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import elastokinematics as _ek
from .kinematics import Hardpoints


@dataclass
class JointDuty:
    """One load and swing amplitude a joint sees, and how often per km."""
    load_n: float
    swing_deg: float
    cycles_per_km: float


@dataclass
class JointWearSpec:
    """One member's joints. Lengths mm; K in mm^3/(N m)."""
    member: str                                # "TR", "UF", ... (loadpath names)
    ball_radius_mm: float = 7.9                # 5/16 in rod end
    bearing_width_mm: float = 8.0
    clearance_new_mm: float = 0.025            # both joints of the member, total
    n_joints: int = 2
    wear_rate_mm3_per_Nm: float = 5e-7
    provenance: str = "declared"
    duty: list = field(default_factory=list)   # list[JointDuty]

    def growth_per_km_mm(self) -> float:
        """Clearance growth of the member (all its joints), mm per km."""
        d = 2.0 * self.ball_radius_mm
        area = d * self.bearing_width_mm
        g = 0.0
        for du in self.duty:
            p = abs(du.load_n) / area                            # N/mm^2
            s_m = self.ball_radius_mm * math.radians(abs(du.swing_deg)) / 1000.0
            h = self.wear_rate_mm3_per_Nm * p * s_m              # mm per cycle
            g += 2.0 * h * du.cycles_per_km
        return g * self.n_joints

    def clearance_at_km(self, km) -> np.ndarray:
        return self.clearance_new_mm + self.growth_per_km_mm() * np.asarray(km, float)


def deadband_over_life(hp: Hardpoints, specs: list[JointWearSpec], km) -> dict:
    """Channel deadband (deg) at each distance, from worn clearances."""
    km = np.atleast_1d(np.asarray(km, float))
    out = {ch: np.zeros(len(km)) for ch in _ek.CHANNELS}
    for i, d in enumerate(km):
        lash = {s.member: float(s.clearance_at_km(d)) for s in specs}
        db = _ek.lash_deadband(hp, lash)
        for ch in out:
            out[ch][i] = db[ch]
    out["km"] = km
    return out


def km_to_budget(hp: Hardpoints, specs: list[JointWearSpec], channel: str,
                 budget_deg: float, km_max: float = 1e5) -> float:
    """Distance (km) at which the worn deadband of ``channel`` reaches budget.

    The deadband is linear in clearance and clearance linear in distance, so
    two evaluations fix the line. Returns 0 if the new joints already exceed
    the budget and inf if wear never reaches it within ``km_max``.
    """
    d0 = deadband_over_life(hp, specs, [0.0])[channel][0]
    if d0 >= budget_deg:
        return 0.0
    d1 = deadband_over_life(hp, specs, [1000.0])[channel][0]
    rate = (d1 - d0) / 1000.0
    if rate <= 0.0:
        return float("inf")
    km = (budget_deg - d0) / rate
    return float(km) if km <= km_max else float("inf")


def fit_wear_rate(spec: JointWearSpec, km, measured_clearance_mm) -> float:
    """Wear rate K (mm^3/(N m)) that best fits measured member clearances.

    Least squares on clearance = c0 + g(K) km, with g linear in K; c0 is taken
    as the spec's new clearance. Needs the duty declared.
    """
    km = np.asarray(km, float); c = np.asarray(measured_clearance_mm, float)
    unit = JointWearSpec(**{**spec.__dict__, "wear_rate_mm3_per_Nm": 1.0})
    g1 = unit.growth_per_km_mm()
    if g1 <= 0.0 or np.all(km == 0):
        raise ValueError("fit_wear_rate needs a declared duty and km > 0.")
    y = c - spec.clearance_new_mm
    return float(np.sum(km * y) / (g1 * np.sum(km * km)))
