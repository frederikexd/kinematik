# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Steering bandwidth: the lag from road-wheel steer to lateral acceleration.

A static compliance-steer number says how far the wheel moves; a driver feels
WHEN the car answers. This module gives the frequency response of lateral
acceleration to road-wheel steer for a linear single-track car and splits its
phase lag into the three terms the suspension design controls or inherits:

* tire relaxation: each axle's lateral force builds over a relaxation length
  sigma, a first-order lag of time constant sigma / V;
* compliance steer: the front lateral force steers the wheels back by
  e_f (rad per N of front axle force), the understeer path the pickup and
  link stiffness set;
* joint stiction: on the compliance path the joints stick and release, whose
  describing function (``joints.stiction_describing_function``) multiplies
  e_f by (k / k_eff) exp(-j phi): a secant stiffness and a phase lag.

Equations (unknowns v, r, Fyf, Fyr at each frequency, steer delta = 1):
    m (s v + V r) = Fyf + Fyr
    Iz s r        = a Fyf - b Fyr
    s Fyf = (V / sigma) (Cf (delta - e_f G Fyf - (v + a r) / V) - Fyf)
    s Fyr = (V / sigma) (Cr (-(v - b r) / V) - Fyr)
    a_y = (Fyf + Fyr) / m
The lag is reported in degrees and as time, -phase / (360 f).

Scope: linear, small-angle, constant speed; steering-system compliance
between the driver's hands and the rack is a separate term the team can add
as a further lag. It ranks designs and attributes the lag; telemetry of
steering angle against lateral acceleration is what confirms it. SI units.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np


@dataclass
class SingleTrack:
    """Linear single-track car. SI units; e_f in rad per N of front axle force."""
    mass_kg: float = 300.0
    a_m: float = 0.8476            # CG to front axle (52% of 1.630 m)
    b_m: float = 0.7824            # CG to rear axle
    iz_kgm2: float | None = None   # None = m a b
    speed_ms: float = 15.0
    cf_n_per_rad: float = 38000.0  # front axle cornering stiffness
    cr_n_per_rad: float = 40000.0  # rear axle cornering stiffness
    sigma_m: float = 0.47          # relaxation length (paper, section 3.2)
    e_f_rad_per_n: float = 0.0     # compliance steer, understeer sign
    #: stiction on the compliance path: (k_lin / k_eff, phase lag deg), from
    #: joints.stiction_describing_function; (1, 0) = no stiction
    stiction: tuple[float, float] = (1.0, 0.0)

    @property
    def iz(self) -> float:
        """Yaw inertia in kg·m²."""
        return self.iz_kgm2 if self.iz_kgm2 else self.mass_kg * self.a_m * self.b_m


def lateral_accel_frf(car: SingleTrack, f_hz) -> np.ndarray:
    """a_y / delta ((m/s^2)/rad), complex, at each frequency (Hz)."""
    f = np.atleast_1d(np.asarray(f_hz, float))
    V, m, a, b = car.speed_ms, car.mass_kg, car.a_m, car.b_m
    g_ratio, lag = car.stiction
    G = g_ratio * np.exp(-1j * math.radians(lag))
    tau = car.sigma_m / V if car.sigma_m > 0 else 0.0
    out = np.empty(len(f), complex)
    for i, fi in enumerate(f):
        s = 1j * 2.0 * math.pi * fi
        # rows: lateral, yaw, front force, rear force; unknowns v, r, Fyf, Fyr
        A = np.array([
            [m * s, m * V, -1.0, -1.0],
            [0.0, car.iz * s, -a, b],
            [car.cf_n_per_rad / V, car.cf_n_per_rad * a / V,
             tau * s + 1.0 + car.cf_n_per_rad * car.e_f_rad_per_n * G, 0.0],
            [car.cr_n_per_rad / V, -car.cr_n_per_rad * b / V, 0.0, tau * s + 1.0],
        ], complex)
        rhs = np.array([0.0, 0.0, car.cf_n_per_rad, 0.0], complex)
        v, r, fyf, fyr = np.linalg.solve(A, rhs)
        out[i] = (fyf + fyr) / m
    return out


def lag(car: SingleTrack, f_hz: float) -> dict:
    """Phase lag of a_y behind steer at f_hz: degrees and milliseconds."""
    h = lateral_accel_frf(car, [1e-3, f_hz])
    ph = math.degrees(np.angle(h[1] / h[0]))
    ph = (ph + 180.0) % 360.0 - 180.0
    return {"phase_deg": -ph, "lag_ms": -ph / (360.0 * f_hz) * 1000.0,
            "gain_ratio": float(abs(h[1]) / abs(h[0]))}


def lag_budget(car: SingleTrack, f_hz: float = 2.5) -> dict:
    """Total lag at f_hz and each term's share (ms), by removing it in turn.

    The shares are the lag lost when that term is removed from the full car,
    so they need not sum to the total exactly; the chassis term is what is
    left with all three removed (yaw and sideslip dynamics of a rigid car).
    Units: lag in milliseconds, frequency in Hz, speed in m/s.
    """
    full = lag(car, f_hz)["lag_ms"]
    no_relax = lag(replace(car, sigma_m=0.0), f_hz)["lag_ms"]
    no_stic = lag(replace(car, stiction=(1.0, 0.0)), f_hz)["lag_ms"]
    no_comp = lag(replace(car, e_f_rad_per_n=0.0), f_hz)["lag_ms"]
    bare = lag(replace(car, sigma_m=0.0, e_f_rad_per_n=0.0,
                       stiction=(1.0, 0.0)), f_hz)["lag_ms"]
    return {"total_ms": full, "relaxation_ms": full - no_relax,
            "compliance_ms": full - no_comp, "stiction_ms": full - no_stic,
            "chassis_ms": bare}


def axle_cornering_stiffness(tire, fz_per_wheel_n: float,
                             alpha_deg: float = 1.0) -> float:
    """Axle cornering stiffness (N/rad) from a tire model, two wheels, secant."""
    a = math.radians(alpha_deg)
    return 2.0 * abs(float(tire.fy(a, fz_per_wheel_n))) / a


def yaw_mode(car: "SingleTrack", speeds_ms=(10.0, 15.0, 20.0, 25.0)) -> list[dict]:
    """Natural frequency (Hz) and damping ratio (dimensionless) of the yaw mode.

    The linear single-track car in lateral velocity v and yaw rate r (tire
    lag left out, so this is the rigid-tire yaw mode). Damping terms scale
    with 1/V, so yaw damping falls as speed rises: the answer to "does yaw
    velocity damping increase or decrease with speed?" computed, not
    recalled. Speeds in m/s. Returns one dict per speed; zeta >= 1 means the
    mode does not overshoot, and None marks an oversteering car past its
    critical speed, where the mode is unstable.
    """
    from dataclasses import replace
    out = []
    for V in speeds_ms:
        c = replace(car, speed_ms=V)
        m, a, b, cf, cr, iz = c.mass_kg, c.a_m, c.b_m, c.cf_n_per_rad, c.cr_n_per_rad, c.iz
        A = np.array([[-(cf + cr) / (m * V), -(a * cf - b * cr) / (m * V) - V],
                      [-(a * cf - b * cr) / (iz * V), -(a * a * cf + b * b * cr) / (iz * V)]])
        ev = np.linalg.eigvals(A)
        det, tr = float(np.linalg.det(A)), float(np.trace(A))
        wn = float(np.sqrt(det)) if det > 0 else float("nan")
        zeta = -tr / (2.0 * wn) if det > 0 else None       # >= 1: no overshoot; None: unstable
        out.append({"speed_ms": V, "fn_hz": wn / (2 * np.pi), "zeta": zeta,
                    "yaw_damping_nms_per_rad": (a * a * cf + b * b * cr) / V})
    return out
