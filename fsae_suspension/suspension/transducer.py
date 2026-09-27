# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Strain-gauged link transducer: bridge, signal chain, dynamics, rejection.

The design side of the measurement architecture in the paper's section 7,
following the practice set out in J. Chester's Formula Student training
seminar for Procter & Chester Measurements ("The Hidden Engineering Behind
Every Load Measurement", 2026). Every function is a closed-form or sampled
calculation on declared inputs; nothing here is a measurement.

    tube_strains            axial, bending and torsional strain in the link
    bridge_output_mvv       full-bridge output, (GF/4)(e1 - e2 + e3 - e4)
    excitation_limit_v      V = 2 sqrt(R A P'), the self-heating limit
    grid_power_density      P' = V^2 / (4 R A) at a chosen excitation
    four_wire_loss          sensitivity lost to lead resistance without sense
    dynamic_amplitude_error 1/(1 - r^2) - 1 for a lightly damped member
    rejection_monte_carlo   bending and torsion leakage for a gauge layout
                            under bonding tolerances

Units: N, mm, MPa, microstrain, V, ohm, kW/m^2 unless named.
"""
from __future__ import annotations

import math

import numpy as np


def tube_section(od_mm: float, wall_mm: float) -> dict:
    """Area, second moment and polar moment of a round tube. Units: diameter and wall in mm; area in mm², moments in mm⁴."""
    idm = od_mm - 2.0 * wall_mm
    A = math.pi / 4.0 * (od_mm ** 2 - idm ** 2)
    I = math.pi / 64.0 * (od_mm ** 4 - idm ** 4)
    return {"A_mm2": A, "I_mm4": I, "J_mm4": 2.0 * I, "c_mm": od_mm / 2.0}


def tube_strains(force_n: float, bend_nmm: float, torque_nmm: float,
                 od_mm: float, wall_mm: float, E: float = 205000.0,
                 nu: float = 0.29) -> dict:
    """Surface strains (microstrain) from axial force, bending and torsion. Units: force in N, moments in N·mm, dimensions in mm, E in MPa; strains in microstrain (dimensionless)."""
    s = tube_section(od_mm, wall_mm)
    G = E / (2.0 * (1.0 + nu))
    return {"axial_ue": force_n / (E * s["A_mm2"]) * 1e6,
            "bending_ue": bend_nmm * s["c_mm"] / s["I_mm4"] / E * 1e6,
            "torsion_shear_ue": torque_nmm * s["c_mm"] / s["J_mm4"] / G * 1e6}


def bridge_output_mvv(axial_ue: float, gauge_factor: float = 2.1,
                      nu: float = 0.29, arrangement: str = "poisson_full") -> float:
    """Bridge output (mV/V) for axial strain.

    V_o/V_i = (GF/4)(e1 - e2 + e3 - e4). A full Poisson bridge has two axial
    arms (+e) and two transverse arms (-nu e), so V_o/V_i = (GF/4) 2 (1+nu) e.
    A half bridge (one axial, one transverse) gives half that; a quarter
    bridge a quarter of the full-axial value.
    Units: strain in microstrain, output in mV/V (both dimensionless ratios).
    """
    e = axial_ue * 1e-6
    k = {"poisson_full": 2.0 * (1.0 + nu), "poisson_half": 1.0 + nu,
         "quarter": 1.0}[arrangement]
    return gauge_factor / 4.0 * k * e * 1e3


def excitation_limit_v(r_ohm: float, grid_area_mm2: float,
                       power_density_kw_m2: float, n_series: int = 1) -> float:
    """Highest excitation (V) before self-heating: V = 2 n sqrt(R A P').

    Each bridge arm carries half the excitation; with ``n_series`` gauges in
    series in an arm, each gauge carries V / (2 n). For n = 1 this is the
    seminar's V = 2 sqrt(R A P').
    """
    return 2.0 * n_series * math.sqrt(r_ohm * grid_area_mm2 * 1e-6 * power_density_kw_m2 * 1e3)


def grid_power_density(v_exc: float, r_ohm: float, grid_area_mm2: float,
                       n_series: int = 1) -> float:
    """Grid power density (kW/m^2): (V / 2n)^2 / (R A), = V^2/(4 R A) for n = 1."""
    return (v_exc / (2.0 * n_series)) ** 2 / (r_ohm * grid_area_mm2 * 1e-6) / 1e3


def euler_load_n(length_mm: float, od_mm: float, wall_mm: float,
                 E: float = 205000.0) -> float:
    """Pinned-pinned Euler load (N) of a link between spherical joints."""
    return math.pi ** 2 * E * tube_section(od_mm, wall_mm)["I_mm4"] / length_mm ** 2


def shunt_resistor_ohm(r_arm_ohm: float, target_mvv: float) -> float:
    """Shunt across one arm that simulates ``target_mvv`` of output.

    A shunt Rs across an arm of resistance R lowers it by R^2/(R + Rs); the
    bridge output is about a quarter of that fractional change, so
    Rs = R (1 / (4 target) - 1) with target in V/V.
    """
    t = target_mvv * 1e-3
    return r_arm_ohm * (1.0 / (4.0 * t) - 1.0)


#: power-density guide (kW/m^2) by spring-element material and accuracy,
#: from the seminar's excitation table (units as in the table it follows)
POWER_DENSITY_KW_M2 = {
    ("thin_steel", "static", "high"): (0.8, 1.6),
    ("thin_steel", "static", "moderate"): (1.5, 3.0),
    ("thin_steel", "dynamic", "high"): (3.0, 8.0),
    ("thick_steel", "static", "high"): (1.5, 3.0),
    ("aluminium", "static", "high"): (3.0, 8.0),
}


def four_wire_loss(r_bridge_ohm: float, r_lead_ohm: float,
                   dT_c: float = 0.0, alpha_cu: float = 0.00393) -> dict:
    """Sensitivity lost to lead resistance in a four-wire connection.

    Without remote sensing the bridge sees V r/(r + 2 R_lead); the loss moves
    with lead temperature through copper's coefficient. A six-wire connection
    senses excitation at the bridge and removes both terms.
    Units: resistances in ohm, temperature change in degrees Celsius, losses in percent.
    """
    loss = 2.0 * r_lead_ohm / (r_bridge_ohm + 2.0 * r_lead_ohm)
    rl2 = r_lead_ohm * (1.0 + alpha_cu * dT_c)
    loss2 = 2.0 * rl2 / (r_bridge_ohm + 2.0 * rl2)
    return {"loss_pct": 100.0 * loss, "drift_pct": 100.0 * (loss2 - loss)}


def dynamic_amplitude_error(f_hz: float, fn_hz: float) -> float:
    """Amplitude error (%) of a lightly damped member: 1/(1 - r^2) - 1. Units: frequencies in Hz, error in percent."""
    r = f_hz / fn_hz
    return 100.0 * (1.0 / (1.0 - r * r) - 1.0)


def rejection_monte_carlo(n_axial: int = 2, pos_tol_deg: float = 1.0,
                          align_tol_deg: float = 0.5, gf_tol: float = 0.002,
                          bending_ue: float = 190.0, torsion_ue: float = 246.0,
                          axial_fs_ue: float = 559.0, limit_pct_fs: float = 0.25,
                          nu: float = 0.29, n: int = 100000, seed: int = 0) -> dict:
    """Bending and torsion leakage of a series Poisson bridge under bonding tolerances.

    ``n_axial`` axial gauges sit evenly round the tube, each with a
    transverse gauge beside it (tee rosettes); all axial arms are in series
    in one bridge arm pair and all transverse in the other. Each gauge gets
    an independent uniform circumferential position error, angular
    misalignment and gauge-factor error. Bending acts in a random plane,
    torsion as a uniform shear strain. The apparent axial strain each
    produces is compared with ``limit_pct_fs`` of the full-scale axial
    strain. Returns 95th-percentile apparent strains, the rejection ratios
    they imply, and the fraction of builds passing both.
    """
    rng = np.random.default_rng(seed)
    th = np.radians(np.arange(n_axial) * 360.0 / n_axial
                    + rng.uniform(-pos_tol_deg, pos_tol_deg, (n, n_axial)))
    phA = np.radians(rng.uniform(-align_tol_deg, align_tol_deg, (n, n_axial)))
    phT = np.radians(90.0 + rng.uniform(-align_tol_deg, align_tol_deg, (n, n_axial)))
    gA = 1.0 + rng.uniform(-gf_tol, gf_tol, (n, n_axial))
    gT = 1.0 + rng.uniform(-gf_tol, gf_tol, (n, n_axial))

    def gauge(ex, ey, gxy, ph):
        """Strain a grid reads (dimensionless) at an orientation in rad."""
        return ex * np.cos(ph) ** 2 + ey * np.sin(ph) ** 2 + gxy * np.sin(ph) * np.cos(ph)

    def bridge(ex, ey, gxy):
        """Bridge output per unit strain (dimensionless)."""
        return (gA * gauge(ex, ey, gxy, phA)).mean(1) - (gT * gauge(ex, ey, gxy, phT)).mean(1)

    one = np.ones_like(th)
    sens = bridge(one, -nu * one, 0.0 * th)
    psi = rng.uniform(0.0, 2.0 * np.pi, (n, 1))
    ab = np.abs(bridge(np.cos(th - psi), -nu * np.cos(th - psi), 0.0 * th)) / sens * bending_ue
    at = np.abs(bridge(0.0 * th, 0.0 * th, one)) / sens * torsion_ue
    lim = limit_pct_fs / 100.0 * axial_fs_ue
    pb, pt = float(np.percentile(ab, 95)), float(np.percentile(at, 95))
    return {"bending_p95_ue": pb, "torsion_p95_ue": pt,
            "bending_rejection_p95": bending_ue / pb if pb > 0 else math.inf,
            "torsion_rejection_p95": torsion_ue / pt if pt > 0 else math.inf,
            "limit_ue": lim, "pass_fraction": float(np.mean((ab < lim) & (at < lim)))}


def poisson_bridge_exact_mvv(axial_ue: float, gauge_factor: float = 2.1, nu: float = 0.29) -> dict:
    """Exact output of a full Poisson bridge and its departure from linear.

    Axial arms change by x = GF e and transverse by -nu x, so the exact
    output is (1 + nu) x / (2 + (1 - nu) x), against the linear (1 + nu) x / 2.
    Units: strain in microstrain, output in mV/V (dimensionless ratios), non-linearity in %.
    """
    x = gauge_factor * axial_ue * 1e-6
    exact = (1.0 + nu) * x / (2.0 + (1.0 - nu) * x)
    lin = (1.0 + nu) * x / 2.0
    return {"exact_mvv": exact * 1e3, "linear_mvv": lin * 1e3, "nonlinearity_pct": 100.0 * (exact - lin) / lin}


def shunt_ratio_check(measured_mvv: float, reference_mvv: float, limit_pct: float = 0.1) -> dict:
    """Compare a shunt-calibration reading with its bench reference.

    A shift beyond ``limit_pct`` points at the connection or the bridge
    (contact resistance, a damaged gauge) and triggers inspection.
    Units: readings in mV/V (dimensionless), shift and limit in %.
    """
    shift = 100.0 * (measured_mvv - reference_mvv) / reference_mvv
    return {"shift_pct": shift, "inspect": abs(shift) > limit_pct}
