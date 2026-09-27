# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Brake bias from the hydraulics, and how it drifts as the brakes heat.

The rest of KinematiK takes brake bias as a declared fraction. This module
derives it from the parts (balance bar, two master cylinders, caliper
pistons, effective rotor radii, pad friction) and follows it through a stint
with each axle's pad friction set by its own rotor temperature, so the anti-
geometry that depends on it can be checked across the run, not only cold.

    pressure      P_ax  = F_pedal * ratio * bar_ax / A_mc,ax
    clamp force   N_ax  = P_ax * A_piston,ax           (per caliper, one side)
    axle torque   T_ax  = n_cal * 2 * mu_ax(T) * N_ax * r_eff,ax
    bias          T_f / (T_f + T_r)

The factor 2 is the two pad faces of an opposed caliper (A_piston is the
piston area on ONE side). mu_ax(T) = mu_nominal * fade(T), with the fade law
of ``brake_thermal.BrakeThermalParams.mu_fade`` for each axle.

What does NOT move the bias, and why: caliper-bridge deflection, fluid
thermal expansion and hose swell add volume the pedal must displace, so they
move pedal travel and feel, but at equilibrium each circuit's pressure is
still pedal force over master-cylinder area; with separate front and rear
circuits the split of force at the balance bar, and so the bias, is
unchanged. ``pedal_travel_mm`` prices the compliance separately; fluid
expansion returns to the reservoir at rest and costs no travel. The bias moves
through pad friction, and through the bar if the pedal travel runs a master
cylinder to its stop, which the travel check flags.

Scope: static hydraulics, no residual-pressure valves or proportioning valves
(add their gain to ``bar_front`` if fitted), fade law as declared
(uncalibrated until fitted). Units: N, mm, bar, deg C.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .brake_thermal import BrakeThermalParams


def _area(d_mm: float) -> float:
    return math.pi * d_mm * d_mm / 4.0


@dataclass
class BrakeHydraulics:
    """The declared brake system. Lengths mm, forces N.

    ``bar_front`` is the fraction of the pushrod force the balance bar sends
    to the front master cylinder. Piston diameters are per piston; a caliper
    with two pistons per side lists ``pistons_per_side=2``.
    """
    pedal_ratio: float = 4.0
    bar_front: float = 0.55
    mc_bore_front_mm: float = 15.875        # 5/8 in
    mc_bore_rear_mm: float = 17.78          # 0.7 in
    piston_front_mm: float = 25.4
    piston_rear_mm: float = 25.4
    pistons_per_side_front: int = 2
    pistons_per_side_rear: int = 1
    r_eff_front_mm: float = 85.0
    r_eff_rear_mm: float = 75.0
    mu_pad_front: float = 0.45
    mu_pad_rear: float = 0.45
    fade_front: BrakeThermalParams = field(default_factory=BrakeThermalParams)
    fade_rear: BrakeThermalParams = field(default_factory=BrakeThermalParams)
    calipers_per_axle: int = 2

    def axle_torques_Nm(self, pedal_force_n: float, t_front_c: float = 30.0,
                        t_rear_c: float = 30.0) -> tuple[float, float]:
        """(front, rear) axle brake torque, N*m, at rotor temperatures (deg C)."""
        F = pedal_force_n * self.pedal_ratio
        pf = F * self.bar_front / _area(self.mc_bore_front_mm)          # N/mm^2
        pr = F * (1.0 - self.bar_front) / _area(self.mc_bore_rear_mm)
        af = self.pistons_per_side_front * _area(self.piston_front_mm)
        ar = self.pistons_per_side_rear * _area(self.piston_rear_mm)
        muf = self.mu_pad_front * self.fade_front.mu_fade(t_front_c)
        mur = self.mu_pad_rear * self.fade_rear.mu_fade(t_rear_c)
        tf = self.calipers_per_axle * 2.0 * muf * pf * af * self.r_eff_front_mm
        tr = self.calipers_per_axle * 2.0 * mur * pr * ar * self.r_eff_rear_mm
        return tf / 1000.0, tr / 1000.0

    def bias(self, t_front_c: float = 30.0, t_rear_c: float = 30.0) -> float:
        """Front share of brake torque (fraction). Independent of pedal force. Units: dimensionless fraction; temperatures in °C."""
        tf, tr = self.axle_torques_Nm(1000.0, t_front_c, t_rear_c)
        return tf / (tf + tr)

    def bar_for_bias(self, target: float, t_front_c: float = 30.0,
                     t_rear_c: float = 30.0) -> float:
        """Balance-bar front fraction that gives ``target`` bias (bisection)."""
        lo, hi = 0.05, 0.95
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            b = BrakeHydraulics(**{**self.__dict__, "bar_front": mid}).bias(
                t_front_c, t_rear_c)
            lo, hi = (mid, hi) if b < target else (lo, mid)
        return 0.5 * (lo + hi)


def bias_drift(system: BrakeHydraulics, t_front_c, t_rear_c) -> dict:
    """Bias along paired rotor-temperature traces (deg C), e.g. from
    ``brake_thermal`` over an endurance stint. Returns the trace and its range."""
    tf = np.atleast_1d(np.asarray(t_front_c, float))
    tr = np.atleast_1d(np.asarray(t_rear_c, float))
    b = np.array([system.bias(a, c) for a, c in zip(tf, tr)])
    return {"bias": b, "min": float(b.min()), "max": float(b.max()),
            "cold": float(system.bias(float(tf[0]), float(tr[0])))}


def anti_dive_window(anti_at_ref_pct: float, bias_ref: float,
                     lo_pct: float, hi_pct: float) -> tuple[float, float]:
    """Front-bias range (fractions) inside which anti-dive stays in [lo, hi].

    With outboard front brakes anti-dive is proportional to the front brake
    fraction, so anti(b) = anti_ref * b / b_ref.
    Units: anti-dive in %, bias as a front fraction (dimensionless).
    """
    k = anti_at_ref_pct / bias_ref
    return lo_pct / k, hi_pct / k


def anti_dive_over_bias(anti_at_ref_pct: float, bias_ref: float, bias) -> np.ndarray:
    """Anti-dive (%) at each bias along a trace (outboard front brakes). Units: anti-dive in %, bias as a front fraction (dimensionless)."""
    return anti_at_ref_pct * np.asarray(bias, float) / bias_ref


def pedal_travel_mm(system: BrakeHydraulics, pressure_bar: float,
                    caliper_compliance_mm3_per_bar: float = 4.0,
                    line_compliance_mm3_per_bar: float = 2.0) -> dict:
    """Pedal travel (mm) each circuit's compliance costs at a line pressure.

    Volume = (calipers x caliper compliance + line compliance) x pressure,
    over the master-cylinder area, times the pedal ratio. Caliper compliance
    rises as the caliper heats, which is how a hot caliper lengthens the pedal
    without moving the bias. Fluid thermal expansion is not a travel term: at
    rest it returns to the reservoir through the compensating port. Declared
    compliance defaults; replace with the caliper data sheet.
    """
    out = {}
    for ax, d in (("front", system.mc_bore_front_mm), ("rear", system.mc_bore_rear_mm)):
        v = (system.calipers_per_axle * caliper_compliance_mm3_per_bar
             + line_compliance_mm3_per_bar) * pressure_bar
        out[ax] = system.pedal_ratio * v / _area(d)
    return out
