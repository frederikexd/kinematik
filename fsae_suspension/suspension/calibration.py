# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Data reduction for the load-link measurement protocol (paper, section 7).

Section 7 pre-registers what each bench and trackside check measures and what
it must meet. This module is the arithmetic that turns the raw readings of
those checks into the reported numbers, fixed before any data exist so that
the reduction cannot be tuned to the result. Nothing here measures anything.

    calibrate            applied load vs bridge output over ascending and
                         descending runs: sensitivity, zero, non-linearity,
                         hysteresis, repeatability (percent of full scale) and
                         an expanded uncertainty, k = 2
    bending_rejection    apparent axial output from an applied bending moment,
                         against the rejection the error budget needs
    zero_drift           zero readings through a session: shift and rate
    temperature_coeff    zero against temperature: the coefficient to correct
    installation_offset  installed reading at rest against the load predicted
                         from scaled corner weights: the offset to record

Units: N, mV/V, microstrain, deg C, s.
"""
from __future__ import annotations

import math

import numpy as np


def calibrate(applied_n, output_mvv, direction, run, full_scale_n: float,
              u_reference_rel: float = 0.001) -> dict:
    """Linear calibration and its error components.

    ``direction`` is +1 for ascending points and -1 for descending; ``run``
    labels repeats. The best-fit line through every point gives the
    sensitivity and zero. Non-linearity is the largest deviation of the
    ascending mean from the line; hysteresis the largest ascending-descending
    difference at one load; repeatability the largest spread between runs at
    one load and direction; each as percent of full scale. The standard
    uncertainty combines them as rectangular distributions (half-width / sqrt 3)
    with the reference cell's relative uncertainty at full scale, and the
    expanded uncertainty is k = 2 of that.
    """
    F = np.asarray(applied_n, float); V = np.asarray(output_mvv, float)
    d = np.asarray(direction); r = np.asarray(run)
    sens, zero = np.polyfit(F, V, 1)
    Fhat = (V - zero) / sens                       # reading in newtons
    err = Fhat - F
    loads = np.unique(F)
    nl = hy = rp = 0.0
    for L in loads:
        m = F == L
        up, dn = m & (d > 0), m & (d < 0)
        if np.any(up):
            nl = max(nl, abs(float(np.mean(err[up]))))
        if np.any(up) and np.any(dn):
            hy = max(hy, abs(float(np.mean(err[up]) - np.mean(err[dn]))))
        for sel in (up, dn):
            if np.count_nonzero(sel) > 1:
                rp = max(rp, float(np.ptp(err[sel])))
    pct = lambda x: 100.0 * x / full_scale_n
    u = math.sqrt((nl / math.sqrt(3)) ** 2 + (hy / 2 / math.sqrt(3)) ** 2
                  + (rp / 2 / math.sqrt(3)) ** 2
                  + (u_reference_rel * full_scale_n) ** 2)
    return {"sensitivity_mvv_per_n": float(sens), "zero_mvv": float(zero),
            "nonlinearity_pct_fs": pct(nl), "hysteresis_pct_fs": pct(hy),
            "repeatability_pct_fs": pct(rp), "u_std_n": u,
            "U_expanded_n": 2.0 * u, "U_expanded_pct_fs": pct(2.0 * u)}


def bending_rejection(apparent_axial_n: float, full_scale_n: float,
                      bending_strain_ue: float, full_scale_strain_ue: float,
                      budget_pct_fs: float = 0.5) -> dict:
    """Measured bending rejection against the ratio the budget needs.

    Rejection = (bending surface strain / full-scale axial strain) divided by
    (apparent axial reading / full scale). The budget needs the apparent
    reading inside ``budget_pct_fs``, so the required ratio is the strain
    ratio over that fraction (the paper's 68:1 at 189 and 559 microstrain).
    """
    strain_ratio = bending_strain_ue / full_scale_strain_ue
    apparent = abs(apparent_axial_n) / full_scale_n
    measured = strain_ratio / apparent if apparent > 0 else math.inf
    required = strain_ratio / (budget_pct_fs / 100.0)
    return {"measured_ratio": measured, "required_ratio": required,
            "passes": measured >= required}


def zero_drift(t_s, zero_n, limit_n: float) -> dict:
    """Zero shift across a session and its rate; flags a shift past the limit."""
    t = np.asarray(t_s, float); z = np.asarray(zero_n, float)
    rate = float(np.polyfit(t, z, 1)[0]) if len(t) > 1 else 0.0
    shift = float(np.ptp(z))
    return {"shift_n": shift, "rate_n_per_s": rate, "within_limit": shift <= limit_n}


def temperature_coeff(temp_c, zero_n) -> dict:
    """Zero-versus-temperature coefficient (N per deg C) and fit residual."""
    T = np.asarray(temp_c, float); z = np.asarray(zero_n, float)
    a, b = np.polyfit(T, z, 1)
    return {"coeff_n_per_c": float(a), "zero_at_0c_n": float(b),
            "rms_n": float(np.sqrt(np.mean((z - (a * T + b)) ** 2)))}


def installation_offset(reading_n: float, predicted_n: float, u_n: float) -> dict:
    """Installed reading at rest against the predicted static member load.

    The difference is the fit-up preload to record, never to tare away; it
    is significant when it exceeds the expanded uncertainty.
    Units: readings, prediction, offset and uncertainty all in N.
    """
    off = float(reading_n - predicted_n)
    return {"offset_n": off, "significant": abs(off) > u_n}


# --------------------------------------------------------------------------- #
#  Loading schedule, certificate reduction and uncertainty budget
# --------------------------------------------------------------------------- #
def loading_schedule(rated_n: float, n_steps: int = 5, n_preloads: int = 3,
                     orientations_deg=(0.0, 120.0, 240.0), preload_hold_s: float = 60.0,
                     zero_hold_s: float = 60.0, step_dwell_s: float = 30.0) -> list[dict]:
    """The calibration loading cycle, as a list of held load points.

    Three preloads to rated load, each held and returned to zero; then a
    measurement cycle of ``n_steps`` equal steps up and back down (2 n + 1
    points, 11 for five steps) with a dwell at every step. The transducer is
    rotated about its own axis between measurement cycles, with one further
    preload before each, so each orientation carries a full cycle. Every
    point is a dict of orientation, phase, load (N), minimum hold (s) and
    whether it is a recorded measurement point.
    """
    pts = []
    def add(o, ph, load, hold, rec=False):
        """Units: load in N, hold in seconds, orientation in deg."""
        pts.append({"orientation_deg": o, "phase": ph, "load_n": load,
                    "min_hold_s": hold, "recorded": rec})
    for k, o in enumerate(orientations_deg):
        for _ in range(n_preloads if k == 0 else 1):
            add(o, "preload", rated_n, preload_hold_s); add(o, "zero", 0.0, zero_hold_s)
        up = [rated_n * i / n_steps for i in range(n_steps + 1)]
        for L in up + up[-2::-1]:
            add(o, "measure", L, step_dwell_s, True)
    return pts


def certificate_reduction(loads, runs_inc, runs_dec) -> dict:
    """Reduce a multi-run calibration the way a calibration certificate does.

    ``loads`` are the applied forces from zero to rated; ``runs_inc`` and
    ``runs_dec`` are lists (one per run or orientation) of the responses at
    those loads increasing and decreasing (the decreasing value at rated load
    is the increasing one). Returns, per load: mean increasing and decreasing
    response, standard deviation across runs, hysteresis as percent of span
    (mean increasing minus mean decreasing, over the zero-corrected response
    at rated load); the span response A (zero-corrected mean increasing); the
    second-order polynomial through zero fitted to A against force, y = m1
    x^2 + m2 x, and its response B; non-linearity as (B - A) in percent of
    span; and the inverse polynomial, force as m1' y^2 + m2' y, used in
    service. These are the columns of the certificate reviewed in the seminar.
    """
    x = np.asarray(loads, float)
    inc = np.asarray(runs_inc, float); dec = np.asarray(runs_dec, float)
    mi, md = inc.mean(0), dec.mean(0)
    sd = np.vstack([inc, dec]).std(0, ddof=1) if inc.shape[0] > 1 else np.zeros_like(mi)
    A = mi - mi[0]
    span = A[-1]
    hyst = 100.0 * (mi - md) / span
    M = np.column_stack([x ** 2, x])
    m1, m2 = np.linalg.lstsq(M, A, rcond=None)[0]
    B = m1 * x ** 2 + m2 * x
    nl = 100.0 * (B - A) / span
    Mi = np.column_stack([A ** 2, A])
    f1, f2 = np.linalg.lstsq(Mi, x, rcond=None)[0]
    return {"mean_inc": mi, "mean_dec": md, "std_dev": sd, "hysteresis_pct": hyst,
            "span_A": A, "poly_B": B, "delta": B - A, "nonlinearity_pct": nl,
            "response_coeffs": (float(m1), float(m2)), "force_coeffs": (float(f1), float(f2)),
            "max_nonlinearity_pct": float(nl[np.argmax(np.abs(nl))]),
            "max_hysteresis_pct": float(hyst[np.argmax(np.abs(hyst))])}


def uncertainty_budget(components: dict, k: float = 2.0) -> dict:
    """Combine standard uncertainties (percent of full scale) by root sum of squares.

    ``components`` maps a name to (value, distribution, divisor-note) where
    value is the quoted limit in percent of full scale and distribution is
    "normal_k2" (a k = 2 expanded value, divided by 2), "rectangular" (a
    half-width, divided by sqrt 3) or "std" (already a standard uncertainty).
    Returns each standard uncertainty, the combined value and the expanded
    uncertainty at coverage factor k, following the GUM.
    """
    u = {}
    for name, (val, dist) in components.items():
        if dist == "normal_k2":
            u[name] = val / 2.0
        elif dist == "rectangular":
            u[name] = val / math.sqrt(3.0)
        elif dist == "std":
            u[name] = val
        else:
            raise ValueError(f"unknown distribution {dist}")
    uc = math.sqrt(sum(v * v for v in u.values()))
    return {"standard": u, "combined_pct": uc, "expanded_pct": k * uc, "k": k}


def check_calibration_header(cal: dict, channel: dict) -> dict:
    """Refuse to log a channel whose calibration record does not match it.

    ``cal`` is the active calibration record (at least ``channel_id``,
    ``transducer_serial``, ``mode`` coefficients and ``valid_until``);
    ``channel`` the logger's channel metadata (``channel_id``,
    ``transducer_serial`` and optionally ``date``). Returns the mismatches;
    an empty list means the header matches and data may be written.
    Compares identifiers and dates only, so it carries no physical units (dimensionless).
    """
    problems = []
    for key in ("channel_id", "transducer_serial"):
        if cal.get(key) != channel.get(key):
            problems.append(f"{key}: calibration {cal.get(key)!r} vs channel {channel.get(key)!r}")
    if "date" in channel and "valid_until" in cal and str(channel["date"]) > str(cal["valid_until"]):
        problems.append(f"calibration expired {cal['valid_until']}")
    for mode in ("tension", "compression"):
        if mode not in cal.get("coeffs", {}):
            problems.append(f"no {mode} coefficients")
    return {"ok": not problems, "problems": problems}


def gum_budget_s74(reference_u_pct: float = 0.025, include_bridge_nl: bool = True,
                   dynamic_ci: float = 0.1) -> dict:
    """The twelve-source budget of supplement table S7.4 for one gauged link.

    Standard uncertainties in percent of full scale. ``reference_u_pct`` is
    the reference cell's standard uncertainty: a U = 0.05% (k = 2) cell gives
    0.025. The bridge's own non-linearity is absorbed by the second-order
    calibration, so ``include_bridge_nl`` counts it only as a conservative
    extra. ``dynamic_ci`` is the sensitivity to content above 200 Hz.
    """
    comp = {"Calibration non-linearity": (0.25 / 2, "std"),
            "Calibration hysteresis": (0.25 / 2, "std"),
            "Repeatability (3 repeats)": (0.10 / math.sqrt(3), "std"),
            "Creep (20 min)": (0.02 / 2, "std"),
            "Reference standard": (reference_u_pct, "std"),
            "Thermal zero shift (corrected)": (0.006, "std"),
            "Bending contamination": (0.25 / (2 * math.sqrt(3)), "std"),
            "Torsion contamination": (0.25 / (2 * math.sqrt(3)), "std"),
            "DAQ resolution": (0.002, "std"),
            "Excitation stability": (0.05 / math.sqrt(3) / 2, "std"),
            "Dynamic bandwidth": (1.21 * dynamic_ci * 0.5, "std")}
    if include_bridge_nl:
        comp["Bridge non-linearity (absorbed by the fit)"] = (0.042 / 2, "std")
    return uncertainty_budget(comp)
