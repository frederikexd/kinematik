# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Radiant heat on a gauged tube: how big the gradient is, how fast, and what
it reads as.

A full (Poisson) bridge cancels temperature only when all four arms share it.
A brake disc or motor controller radiates onto one side of a thin-walled
link, so the side facing it runs hotter, and self-compensated gauges cancel
their OWN temperature but not a difference between arms: the residual is the
gauges' compensation mismatch times the temperature difference between arms.
This module sizes that gradient, its time constant, and the apparent strain,
and tells the protocol how many temperature sensors resolve it.

RING MODEL
----------
A thin ring (radius r, wall t, conductivity k) absorbs a radiant flux on one
side, q_abs(theta) = a q max(cos theta, 0), and loses heat by convection h on
both faces. Circumferential conduction k t / r^2 couples the sides. The
first Fourier harmonic carries the side-to-side difference: its amplitude is
T1 = (a q / 2) / (2 h + k t / r^2), so the difference between the hot and the
cold side is about 2 T1, and it forms with time constant
tau = rho c t / (2 h + k t / r^2). Higher harmonics add little across the
diameter and are neglected.

SENSORS
-------
The gradient is a cosine whose direction is the direction of the source.
Two sensors on one diameter read its full amplitude only when that diameter
points at the source, and cos(phi) of it when misaligned by phi, so the
check can pass while the gauges see more. Three sensors at 120 deg resolve
amplitude and direction of the first harmonic whatever the source's
direction; ``first_harmonic_from_three`` does that.

The apparent strain is mismatch x (difference between the arms' temperatures).
Self-compensated gauges are matched to steel or aluminium within about 1 to
2 microstrain per deg C; the value is DECLARED from the gauge data sheet.
Units: mm, W/m^2, deg C, microstrain.
"""
from __future__ import annotations

import math

import numpy as np

_SIGMA = 5.670374419e-8

MATERIALS = {   # k W/mK, rho kg/m^3, c J/kgK
    "4130": (42.0, 7850.0, 460.0),
    "6061": (167.0, 2700.0, 896.0),
    "7075": (130.0, 2810.0, 960.0),
    "cfrp": (5.0, 1550.0, 900.0),
}


def radiant_flux(t_source_c: float, t_tube_c: float, view_factor: float,
                 emissivity_source: float = 0.8) -> float:
    """Net radiant flux (W/m^2) onto the tube from a hot source."""
    ts, tt = t_source_c + 273.15, t_tube_c + 273.15
    return emissivity_source * _SIGMA * view_factor * (ts ** 4 - tt ** 4)


def ring_gradient(q_w_m2: float, od_mm: float, wall_mm: float,
                  material: str = "4130", h_w_m2k: float = 50.0,
                  absorptivity: float = 0.8) -> dict:
    """Hot-to-cold side difference (deg C) and its time constant (s)."""
    k, rho, c = MATERIALS[material]
    r = od_mm / 2000.0; t = wall_mm / 1000.0
    cond = k * t / (r * r)
    T1 = (absorptivity * q_w_m2 / 2.0) / (2.0 * h_w_m2k + cond)
    tau = rho * c * t / (2.0 * h_w_m2k + cond)
    return {"delta_T_c": 2.0 * T1, "tau_s": tau, "conduction_w_m2k": cond}


def shield_factor(emissivity_shield: float = 0.1, emissivity_source: float = 0.8,
                  emissivity_tube: float = 0.8) -> float:
    """Fraction of radiant exchange a single thin shield passes (parallel plates).

    Unshielded resistance 1/e1 + 1/e2 - 1; with one shield of emissivity es on
    both faces it becomes (1/e1 + 1/es - 1) + (1/es + 1/e2 - 1). The ratio is
    the factor. A polished foil (es ~ 0.05 to 0.1) passes 4 to 7%.
    """
    e1, e2, es = emissivity_source, emissivity_tube, emissivity_shield
    bare = 1.0 / e1 + 1.0 / e2 - 1.0
    shielded = (1.0 / e1 + 1.0 / es - 1.0) + (1.0 / es + 1.0 / e2 - 1.0)
    return bare / shielded


def apparent_microstrain(delta_T_arms_c: float, mismatch_ue_per_c: float = 1.8
                         ) -> float:
    """Apparent strain (microstrain) from a temperature difference between arms."""
    return mismatch_ue_per_c * delta_T_arms_c


def two_sensor_reading(delta_T_c: float, misalign_deg: float) -> float:
    """Difference two diametral sensors read, misaligned from the source."""
    return delta_T_c * math.cos(math.radians(misalign_deg))


def first_harmonic_from_three(t0: float, t120: float, t240: float) -> dict:
    """Amplitude (hot-to-cold difference) and direction of the gradient from
    three sensors at 0, 120 and 240 deg."""
    th = np.radians([0.0, 120.0, 240.0])
    T = np.array([t0, t120, t240], float)
    a = 2.0 / 3.0 * np.sum(T * np.cos(th))
    b = 2.0 / 3.0 * np.sum(T * np.sin(th))
    return {"delta_T_c": 2.0 * math.hypot(a, b),
            "direction_deg": math.degrees(math.atan2(b, a)) % 360.0}


# --------------------------------------------------------------------------- #
#  What the full bridge actually reads
# --------------------------------------------------------------------------- #
def ring_harmonics(q_w_m2: float, od_mm: float, wall_mm: float,
                   material: str = "4130", h_w_m2k: float = 50.0,
                   absorptivity: float = 0.8, n_max: int = 4) -> dict:
    """Fourier amplitudes (deg C) of the ring temperature field, n = 1..n_max.

    The absorbed flux max(cos theta, 0) has cosine coefficients 1/pi,
    1/2, 2/(3 pi), 0, -2/(15 pi) ...; mode n is opposed by convection 2h and
    circumferential conduction n^2 k t / r^2, so T_n = a q c_n / (2 h + n^2 k t / r^2).
    """
    k, rho, c = MATERIALS[material]
    r = od_mm / 2000.0; t = wall_mm / 1000.0
    def cn(n):
        if n == 1:
            return 0.5
        if n % 2 == 1:
            return 0.0
        return 2.0 / math.pi * (-1) ** (n // 2 + 1) / (n * n - 1)
    return {n: absorptivity * q_w_m2 * cn(n) / (2.0 * h_w_m2k + n * n * k * t / (r * r))
            for n in range(1, n_max + 1)}


def ring_temperature(harmonics: dict, theta_deg: float, source_deg: float = 0.0) -> float:
    """Temperature rise (deg C, less the uniform part) at an angle on the ring."""
    th = math.radians(theta_deg - source_deg)
    return sum(a * math.cos(n * th) for n, a in harmonics.items())


def poisson_bridge_apparent_ue(t_axial, t_transverse, mismatch_ue_per_c: float = 1.8,
                               poisson: float = 0.3) -> float:
    """Apparent axial strain (microstrain) a Poisson full bridge reads from
    the temperatures at its gauges.

    The bridge sums the two axial arms and subtracts the two transverse arms;
    each gauge's thermal output is the compensation mismatch times its own
    temperature. Its axial sensitivity is 2 (1 + nu), so the apparent axial
    strain is mismatch (Ta1 + Ta2 - Tt1 - Tt2) / (2 (1 + nu)). A uniform rise
    cancels; so does any field antisymmetric across the tube, as bending does,
    when the axial gauges sit on opposite faces.
    """
    ta1, ta2 = t_axial; tt1, tt2 = t_transverse
    return mismatch_ue_per_c * (ta1 + ta2 - tt1 - tt2) / (2.0 * (1.0 + poisson))


def bridge_thermal_error(q_w_m2: float, od_mm: float, wall_mm: float,
                         source_deg: float = 0.0, transverse_at_deg: float = 90.0,
                         material: str = "4130", mismatch_ue_per_c: float = 1.8,
                         h_w_m2k: float = 50.0) -> dict:
    """Apparent strain a one-sided radiant field gives a Poisson bridge with
    axial gauges at 0 and 180 deg and transverse gauges at
    ``transverse_at_deg`` and +180 (90 = the usual quarter-turn layout, 0 =
    beside the axial gauges), for a source at ``source_deg``."""
    h = ring_harmonics(q_w_m2, od_mm, wall_mm, material, h_w_m2k)
    ta = (ring_temperature(h, 0.0, source_deg), ring_temperature(h, 180.0, source_deg))
    tt = (ring_temperature(h, transverse_at_deg, source_deg),
          ring_temperature(h, transverse_at_deg + 180.0, source_deg))
    return {"side_to_side_c": 2.0 * h[1], "second_harmonic_c": h[2],
            "apparent_ue": poisson_bridge_apparent_ue(ta, tt, mismatch_ue_per_c)}
