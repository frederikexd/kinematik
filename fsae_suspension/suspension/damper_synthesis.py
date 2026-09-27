# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Damper synthesis: slopes from grip and body control, split from balance.

Springs and bars set the steady load transfer; during turn-in the dampers
split it, and over bumps they decide how much the tire load varies. This
module turns those three requirements into a damper, the same way the rest
of KinematiK turns requirements into geometry:

1. HIGH-SPEED SLOPE, for grip. Over a rough road the tire load variation of
   ``contact_patch.load_variation`` has a minimum in damping: too little and
   the wheel hops, too much and the damper drives road input into the tire.
   The damping ratio at that minimum is the grip optimum, and it sets the
   slope above the knee, where wheel-hop velocities live.
2. LOW-SPEED SLOPE, for body control. Body modes run at low shaft speed and
   want a higher ratio (0.65 to 0.7 is the usual transient target) for a
   fast, settled response to steering and braking. It sets the slope below
   the knee.
3. KNEE, from the road. Set at a declared multiple of the RMS suspension
   velocity in the body band (below 4 Hz) on the design road, so that body
   motion stays on the low-speed slope and wheel motion reaches the high one.
4. FRONT/REAR SPLIT, for transient balance. While the body is rolling the
   damper roll moments carry a share of the load transfer, front share
   c_f t_f^2 / (c_f t_f^2 + c_r t_r^2) with c the wheel-referred rates. Matching
   it to the steady front share (53% in the paper) keeps the balance through
   turn-in the same as in the steady corner; ``front_rate_for_share`` solves it.

Rebound is set at a declared multiple of bump. Wheel-referred rates go to the
shaft through the motion ratio squared. The result is a
``damper.DamperCurve`` flagged as synthesised, not calibrated: dyno the real
damper and fit it. Units SI unless named.
"""
from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from .contact_patch import QuarterCar, load_variation, iso8608_psd
from .damper import DamperCurve


def _crit(qc: QuarterCar) -> float:
    ke = qc.k * qc.k_t / (qc.k + qc.k_t)
    return 2.0 * math.sqrt(ke * qc.m_s)


def zeta_sweep(qc: QuarterCar, zetas, speed_ms: float = 15.0,
               road_class: str = "B") -> list[dict]:
    """Load-variation ratio at each body damping ratio (wheel-referred). Units: damping ratios dimensionless, load variation as a fraction (dimensionless)."""
    out = []
    for z in zetas:
        q = replace(qc, c=float(z) * _crit(qc))
        r = load_variation(q, speed_ms, road_class)
        out.append({"zeta": float(z), "ratio": r["ratio"], "stable": r["stable"]})
    return out


def grip_optimal_zeta(qc: QuarterCar, speed_ms: float = 15.0,
                      road_class: str = "B", lo: float = 0.05,
                      hi: float = 1.5) -> float:
    """Damping ratio minimising tire load variation (golden section). Units: damping ratio (dimensionless), load variation as a fraction (dimensionless)."""
    g = (math.sqrt(5.0) - 1.0) / 2.0
    f = lambda z: load_variation(replace(qc, c=z * _crit(qc)), speed_ms,
                                 road_class)["ratio"]
    a, b = lo, hi
    c, d = b - g * (b - a), a + g * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(40):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - g * (b - a); fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + g * (b - a); fd = f(d)
    return 0.5 * (a + b)


def body_band_velocity_rms(qc: QuarterCar, speed_ms: float = 15.0,
                           road_class: str = "B", f_hi: float = 4.0) -> float:
    """RMS suspension (wheel-to-body) velocity below ``f_hi``, m/s."""
    f = np.geomspace(0.3, f_hi, 400)
    rel = np.empty(len(f))
    for i, fi in enumerate(f):
        s = 1j * 2.0 * math.pi * fi
        kt = qc.k_t + qc.c_t * s
        ks = qc.k + qc.c * s
        A = np.array([[qc.m_s * s * s + ks, -ks + qc.kappa * kt],
                      [-ks, qc.m_u * s * s + ks + (1.0 - qc.kappa) * kt]], complex)
        b = np.array([qc.kappa * kt, (1.0 - qc.kappa) * kt], complex)
        zs, zu = np.linalg.solve(A, b)
        rel[i] = abs(s * (zu - zs)) ** 2
    fn = getattr(np, "trapezoid", None) or np.trapz
    return math.sqrt(float(fn(rel * iso8608_psd(f, speed_ms, road_class), f)))


def synthesize(qc: QuarterCar, motion_ratio: float, speed_ms: float = 15.0,
               road_class: str = "B", zeta_body: float = 0.65,
               knee_factor: float = 1.5, rebound_ratio: float = 1.5) -> dict:
    """A bump/rebound damper curve (shaft-referred) and how it was set. Units: rates in N/m and N·s/m, masses in kg, damping ratios dimensionless."""
    z_grip = grip_optimal_zeta(qc, speed_ms, road_class)
    cc = _crit(qc)
    c_low_w, c_high_w = zeta_body * cc, z_grip * cc
    mr2 = motion_ratio * motion_ratio
    v_rms = body_band_velocity_rms(replace(qc, c=c_low_w), speed_ms, road_class)
    knee_shaft = max(knee_factor * v_rms * motion_ratio, 0.005)
    curve = DamperCurve(c_bump_low=c_low_w / mr2, c_bump_high=c_high_w / mr2,
                        c_reb_low=rebound_ratio * c_low_w / mr2,
                        c_reb_high=rebound_ratio * c_high_w / mr2,
                        v_knee=knee_shaft, is_calibrated=False,
                        source="synthesised (damper_synthesis)")
    return {"curve": curve, "zeta_grip": z_grip, "zeta_body": zeta_body,
            "c_wheel_low": c_low_w, "c_wheel_high": c_high_w,
            "knee_shaft_ms": knee_shaft, "body_band_v_rms_ms": v_rms,
            "ratio_at_grip": load_variation(replace(qc, c=c_high_w), speed_ms,
                                            road_class)["ratio"]}


def transient_front_share(c_front_wheel: float, c_rear_wheel: float,
                          track_front_mm: float, track_rear_mm: float) -> float:
    """Front share of the damper roll moment while the body rolls. Units: damping rates in N·s/m, tracks in mm, share dimensionless."""
    f = c_front_wheel * (track_front_mm / 1000.0) ** 2
    r = c_rear_wheel * (track_rear_mm / 1000.0) ** 2
    return f / (f + r)


def front_rate_for_share(target: float, c_rear_wheel: float,
                         track_front_mm: float, track_rear_mm: float) -> float:
    """Front wheel-referred low-speed rate giving ``target`` transient share. Units: damping rates in N·s/m, tracks in mm, share dimensionless."""
    r = c_rear_wheel * (track_rear_mm / 1000.0) ** 2
    return target / (1.0 - target) * r / (track_front_mm / 1000.0) ** 2
