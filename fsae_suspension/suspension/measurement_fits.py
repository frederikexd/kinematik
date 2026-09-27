# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Measured inputs: turn a test's raw data into the parameter a model declares.

Every dynamic model added for the inverse-synthesis review runs on declared
values until a team measures them. This module is the replacement step: one
fitting function per declared input, taking the numbers the named test
produces and returning the parameter in the form the model consumes, with
the fit residual so a poor test is visible. Nothing here invents data.

    input                 test                                 function
    pneumatic trail       Mz and Fy against slip (flat belt)   fit_pneumatic_trail
    carcass share         vertical rate at two+ pressures      fit_carcass_share
    damper rates          force against velocity (dyno)        damper.DamperCurve.from_dyno_points
    road class            measured road profile                fit_road_class
    pad fade law          mu against rotor temperature (dyno)  fit_fade_law
    liner wear rate       clearance at intervals               joint_wear.fit_wear_rate
    plunge friction       axial force against torque (bench)   fit_plunge_friction
    weld residual stress  hole drilling, stress against depth  residual_profile

Units as in the consuming modules: mm, N, deg, kPa, deg C, m for profiles.
"""
from __future__ import annotations

import math

import numpy as np


def fit_pneumatic_trail(alpha_deg, fy_n, mz_nm) -> dict:
    """t0 (mm) and zero-crossing slip alpha0 (deg) of t_p = t0 cos(pi/2 a/a0).

    Pneumatic trail is -Mz / Fy at each slip (sign so it is positive at small
    slip). The cosine law of ``steering_feel`` is fitted by a grid over alpha0
    and linear least squares for t0. Points with |Fy| under 5% of its maximum
    are dropped, where the ratio is ill-conditioned.
    """
    a = np.asarray(alpha_deg, float); fy = np.asarray(fy_n, float)
    mz = np.asarray(mz_nm, float) * 1000.0
    keep = np.abs(fy) > 0.05 * np.max(np.abs(fy))
    a, tp = a[keep], -mz[keep] / fy[keep]
    best = None
    for a0 in np.linspace(max(np.min(np.abs(a)), 0.5), 2.0 * np.max(np.abs(a)), 400):
        c = np.cos(0.5 * np.pi * np.abs(a) / a0)
        t0 = float(np.sum(c * tp) / np.sum(c * c))
        r = float(np.sqrt(np.mean((tp - t0 * c) ** 2)))
        if best is None or r < best[2]:
            best = (t0, a0, r)
    return {"t0_mm": best[0], "alpha0_deg": float(best[1]), "rms_mm": best[2]}


def fit_carcass_share(pressure_kpa, rate_n_per_mm, p_ref_kpa: float) -> dict:
    """Carcass share s and reference rate from rates at several pressures.

    k(p) = a + b p is fitted by least squares; at the reference pressure the
    rate is k0 = a + b p_ref and the carcass share s = a / k0, the form
    ``target_derivation.tire_rate_at_pressure`` uses.
    """
    p = np.asarray(pressure_kpa, float); k = np.asarray(rate_n_per_mm, float)
    if len(p) < 2 or np.ptp(p) <= 0:
        raise ValueError("fit_carcass_share needs rates at two or more pressures.")
    b, a = np.polyfit(p, k, 1)
    k0 = a + b * p_ref_kpa
    return {"carcass_share": float(a / k0), "rate_at_ref_n_per_mm": float(k0),
            "rms_n_per_mm": float(np.sqrt(np.mean((k - (a + b * p)) ** 2)))}


def fit_road_class(profile_m, dx_m: float, n_lo: float = 0.05,
                   n_hi: float = 2.0, segment: int = 1024) -> dict:
    """ISO 8608 G_d(n0) and class from a measured road profile.

    The displacement PSD is estimated by averaged periodograms (Hann window,
    half-overlapped segments), fitted with the ISO form G_d(n0)(n/n0)^-w in
    log space over [n_lo, n_hi] cycles/m, and the class is the one whose
    geometric-mean G_d(n0) is nearest in log terms. Returns G_d(n0) (m^3),
    the fitted waviness w and the class letter.
    """
    from .contact_patch import ISO8608_GD0, N0
    z = np.asarray(profile_m, float) - np.mean(profile_m)
    seg = min(int(segment), len(z))
    step = seg // 2
    win = np.hanning(seg)
    norm = np.sum(win ** 2)
    acc, count = None, 0
    for i in range(0, len(z) - seg + 1, step):
        X = np.fft.rfft(win * z[i:i + seg])
        P = 2.0 * np.abs(X) ** 2 * dx_m / norm       # one-sided, m^2/(cycle/m)
        acc = P if acc is None else acc + P
        count += 1
    G = acc / count
    n = np.fft.rfftfreq(seg, dx_m)
    m = (n >= n_lo) & (n <= n_hi)
    slope, icpt = np.polyfit(np.log(n[m] / N0), np.log(G[m]), 1)
    gd0 = float(math.exp(icpt))
    cls = min(ISO8608_GD0, key=lambda c: abs(math.log(ISO8608_GD0[c] / gd0)))
    return {"Gd_n0_m3": gd0, "waviness": float(-slope), "road_class": cls}


def fit_fade_law(temp_c, mu_ratio) -> dict:
    """Onset (deg C), fade per deg C and floor of the pad fade law.

    ``mu_ratio`` is measured pad friction over its cold value. The law of
    ``brake_thermal.BrakeThermalParams.mu_fade`` (1 to the onset, then linear,
    floored) is fitted by a grid over onset with least squares on the slope;
    the floor is the lowest measured ratio if the data reach it.
    """
    T = np.asarray(temp_c, float); r = np.asarray(mu_ratio, float)
    best = None
    for onset in np.linspace(T.min(), T.max(), 300):
        hot = T > onset
        if np.count_nonzero(hot) < 2:
            continue
        x = T[hot] - onset
        s = float(np.sum(x * (1.0 - r[hot])) / np.sum(x * x))
        pred = np.where(hot, 1.0 - s * np.maximum(T - onset, 0.0), 1.0)
        err = float(np.sqrt(np.mean((r - pred) ** 2)))
        if best is None or err < best[2]:
            best = (onset, s, err)
    if best is None:
        raise ValueError("fit_fade_law needs temperatures above an onset.")
    return {"T_fade_onset_c": float(best[0]), "fade_per_C": max(best[1], 0.0),
            "fade_floor": float(np.min(r)), "rms": best[2]}


def fit_plunge_friction(torque_nm, axial_force_n,
                        pitch_radius_mm: float) -> dict:
    """Effective plunge friction mu from a bench test: F = mu T / r."""
    T = np.asarray(torque_nm, float) * 1000.0; F = np.asarray(axial_force_n, float)
    k = float(np.sum(T * F) / np.sum(T * T))
    return {"mu_plunge": k * pitch_radius_mm,
            "rms_n": float(np.sqrt(np.mean((F - k * T) ** 2)))}


def residual_profile(depth_mm, stress_mpa, thickness_mm: float,
                     symmetric: bool = True):
    """A residual-stress profile function y -> MPa from hole-drilling data.

    Hole drilling measures stress against depth from the surface. The profile
    for ``permanent_set`` is a function of the fiber height y (0 at the
    mid-plane): with ``symmetric`` the same surface profile is mirrored onto
    both faces (a plate welded or cooled alike on both), otherwise the far
    face is taken as stress-free. Between measured depths it interpolates
    linearly and holds the deepest value to the mid-plane; ``permanent_set``
    then removes the net force and moment so the profile is self-equilibrated.
    """
    d = np.asarray(depth_mm, float); s = np.asarray(stress_mpa, float)
    order = np.argsort(d); d, s = d[order], s[order]
    h2 = thickness_mm / 2.0

    def prof(y: float) -> float:
        depth_top = h2 - y
        depth_bot = h2 + y
        v = float(np.interp(depth_top, d, s))
        if symmetric and depth_bot < depth_top:
            v = float(np.interp(depth_bot, d, s))
        elif not symmetric and depth_top > d.max():
            v = 0.0
        return v
    return prof
