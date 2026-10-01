# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Contact-patch load variation: the four-post question, answered analytically.

A kinematic target says nothing about whether the tire stays on the road. The
number that does is the variation of the vertical tire load over a rough
surface, sigma(dFz) / Fz_static: grip follows load, and a tire whose load
swings loses grip on average because the curve of force against load bends
over. A four-post rig measures it; this module predicts it from the same
quarter-car a rig excites, so a design can be ranked before a rig is booked.

THE MODEL
---------
Two masses (sprung corner share m_s, unsprung m_u), the wheel rate k and a
linear damper c between them, the tire as a spring k_t (and optional damping
c_t) to the road. Road height z_r is the input; dFz = k_t (z_r - z_u) +
c_t (z_r' - z_u') is the output. Frequency responses are exact for the linear
model.

THE ANTI-GEOMETRY BYPASS
------------------------
Under braking or traction the longitudinal force is tied to the vertical load,
Fx = mu_x Fz, and anti-geometry turns a share of Fx into a vertical force
between body and wheel carried by the links, not the spring or damper:
F_link = tan(theta) Fx, with tan(theta) the side-view swing-arm slope. Over a
bump under braking the dynamic part is F_link = kappa dFz with kappa =
tan(theta) mu_x, pushing the body up and the wheel down. The load variation
therefore feeds a force back into the corner that no damper sees. For outboard
brakes tan(theta) follows from the anti percentage:
tan(theta) = (anti / 100) h / (L bias), so ``kappa_from_anti`` needs only the
numbers the paper already has. kappa = 0 is the classical quarter car.

ROADS
-----
ISO 8608 displacement PSDs, G_d(n) = G_d(n0) (n / n0)^-w, n0 = 0.1 cycle/m,
w = 2, with the class geometric means (A 16e-6, B 64e-6, C 256e-6 m^3). At
speed V the temporal PSD is G(f) = G_d(f / V) / V.

SCOPE
-----
Linear, one corner, vertical only apart from the anti feedback. It ranks
designs and exposes the trends (unsprung mass, damping, anti) a rig measures;
it does not replace the rig, and damper non-linearity, bump-stop contact and
tire lift-off (dFz < -Fz_static) are outside it. Units SI unless named.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

ISO8608_GD0 = {"A": 16e-6, "B": 64e-6, "C": 256e-6, "D": 1024e-6}   # m^3
N0 = 0.1                                                             # cycle/m


@dataclass
class QuarterCar:
    """One corner. Masses kg, rates N/m, damping N*s/m.

    ``kappa`` is the anti-geometry bypass gain (dimensionless), see the
    module docstring; 0 without anti or without a longitudinal force.
    """
    m_s: float
    m_u: float
    k: float
    c: float
    k_t: float
    c_t: float = 0.0
    kappa: float = 0.0

    def __post_init__(self):
        for n in ("m_s", "m_u", "k", "k_t"):
            if float(getattr(self, n)) <= 0.0:
                raise ValueError(f"QuarterCar.{n} must be > 0.")
        if self.c < 0.0 or self.c_t < 0.0:
            raise ValueError("QuarterCar damping must be >= 0.")

    @property
    def static_load_N(self) -> float:
        return (self.m_s + self.m_u) * 9.81

    def wheel_hop_hz(self) -> float:
        """Uncoupled wheel-hop frequency sqrt((k + k_t) / m_u) / 2 pi, Hz."""
        return math.sqrt((self.k + self.k_t) / self.m_u) / (2.0 * math.pi)

    def body_hz(self) -> float:
        """Uncoupled ride frequency of the corner, sprung on k and k_t in series."""
        ke = self.k * self.k_t / (self.k + self.k_t)
        return math.sqrt(ke / self.m_s) / (2.0 * math.pi)

    def load_frf(self, f_hz) -> np.ndarray:
        """dFz / z_r (N/m), complex, at each frequency (Hz)."""
        f = np.atleast_1d(np.asarray(f_hz, float))
        out = np.empty(len(f), complex)
        for i, fi in enumerate(f):
            w = 2.0 * math.pi * fi
            s = 1j * w
            kt = self.k_t + self.c_t * s          # tire dynamic stiffness
            ks = self.k + self.c * s              # suspension dynamic stiffness
            # unknowns z_s, z_u; dFz = kt (z_r - z_u); link force kappa dFz
            # m_s s^2 z_s = -ks (z_s - z_u) + kappa kt (z_r - z_u)
            # m_u s^2 z_u =  ks (z_s - z_u) + (1 - kappa) kt (z_r - z_u)
            A = np.array([[self.m_s * s * s + ks, -ks + self.kappa * kt],
                          [-ks, self.m_u * s * s + ks + (1.0 - self.kappa) * kt]],
                         complex)
            b = np.array([self.kappa * kt, (1.0 - self.kappa) * kt], complex)
            zs, zu = np.linalg.solve(A, b)
            out[i] = kt * (1.0 - zu)
        return out

    def is_stable(self) -> bool:
        """True when every mode of the coupled corner decays."""
        k, c, kt, ct, K = self.k, self.c, self.k_t, self.c_t, self.kappa
        M = np.diag([self.m_s, self.m_u])
        Kmat = np.array([[k, -k + K * kt], [-k, k + (1 - K) * kt]])
        Cmat = np.array([[c, -c + K * ct], [-c, c + (1 - K) * ct]])
        Minv = np.linalg.inv(M)
        A = np.block([[np.zeros((2, 2)), np.eye(2)],
                      [-Minv @ Kmat, -Minv @ Cmat]])
        return bool(np.all(np.linalg.eigvals(A).real < 0.0))


def kappa_from_anti(anti_pct: float, cg_height_mm: float, wheelbase_mm: float,
                    axle_force_share: float, mu_x: float) -> float:
    """Anti bypass gain from an anti percentage (outboard brakes / hub drive).

    tan(theta) = (anti/100) * h / (L * share), share the axle's fraction of
    the longitudinal force (the brake bias for anti-dive), and kappa =
    tan(theta) * mu_x with mu_x the longitudinal force over vertical load.
    """
    if axle_force_share <= 0.0:
        raise ValueError("axle_force_share must be > 0.")
    tan_t = (anti_pct / 100.0) * cg_height_mm / (wheelbase_mm * axle_force_share)
    return float(tan_t * mu_x)


def iso8608_psd(f_hz, speed_ms: float, road_class: str = "B",
                waviness: float = 2.0) -> np.ndarray:
    """Temporal road displacement PSD (m^2/Hz) at frequency f (Hz) and speed."""
    gd0 = ISO8608_GD0[road_class.upper()]
    n = np.asarray(f_hz, float) / float(speed_ms)
    return gd0 * (n / N0) ** (-waviness) / float(speed_ms)


def load_variation(qc: QuarterCar, speed_ms: float, road_class: str = "B",
                   f_lo: float = 0.5, f_hi: float = 50.0, n: int = 2000) -> dict:
    """RMS tire load variation over an ISO 8608 road.

    Returns sigma(dFz) in N, its ratio to the static load, the frequency of
    the largest contribution (Hz) and the share of the variance above 10 Hz,
    which is the wheel-hop band the unsprung mass governs.
    """
    f = np.geomspace(f_lo, f_hi, n)
    H = qc.load_frf(f)
    S = np.abs(H) ** 2 * iso8608_psd(f, speed_ms, road_class)
    fn = getattr(np, "trapezoid", None) or np.trapz
    var = float(fn(S, f))
    hi = f >= 10.0
    var_hi = float(fn(S[hi], f[hi])) if np.count_nonzero(hi) > 1 else 0.0
    sig = math.sqrt(max(var, 0.0))
    return {"sigma_dFz_N": sig, "ratio": sig / qc.static_load_N,
            "peak_hz": float(f[int(np.argmax(S))]),
            "share_above_10hz": var_hi / var if var > 0 else 0.0,
            "stable": qc.is_stable()}


def body_accel_rms(qc: QuarterCar, speed_ms: float, road_class: str = "B",
                   f_lo: float = 0.5, f_hi: float = 50.0, n: int = 2000) -> float:
    """RMS sprung (body) vertical acceleration over an ISO 8608 road, m/s^2.

    The harshness a driver feels through the seat. With ``kappa`` set it
    includes the anti path, which reaches the body without passing the
    damper, so anti-geometry's cost in harshness under braking is priced
    directly against the same corner with kappa = 0.
    """
    f = np.geomspace(f_lo, f_hi, n)
    acc = np.empty(len(f))
    for i, fi in enumerate(f):
        s = 1j * 2.0 * math.pi * fi
        kt = qc.k_t + qc.c_t * s
        ks = qc.k + qc.c * s
        A = np.array([[qc.m_s * s * s + ks, -ks + qc.kappa * kt],
                      [-ks, qc.m_u * s * s + ks + (1.0 - qc.kappa) * kt]], complex)
        b = np.array([qc.kappa * kt, (1.0 - qc.kappa) * kt], complex)
        zs, _ = np.linalg.solve(A, b)
        acc[i] = abs(s * s * zs) ** 2
    fn = getattr(np, "trapezoid", None) or np.trapz
    return math.sqrt(float(fn(acc * iso8608_psd(f, speed_ms, road_class), f)))


def damper_speed_distribution(qc: "QuarterCar", speed_ms: float, knee_ms: float,
                              motion_ratio: float = 1.0, road_class: str = "B",
                              f_lo: float = 0.1, f_hi: float = 50.0, n: int = 4000) -> dict:
    """Share of time the damper spends below its low/high-speed knee.

    Suspension (wheel-to-body) velocity from the quarter-car response to an
    ISO 8608 road, times the motion ratio for the damper shaft; with a
    Gaussian road the shaft velocity is Gaussian, so the share below the
    knee is erf(knee / (sigma sqrt 2)). Velocities in m/s, speed in m/s,
    frequencies in Hz; shares dimensionless. A distribution, not a number:
    it says where the damper curve matters on this road.
    """
    import math as _m
    f = np.linspace(f_lo, f_hi, n); w = 2 * np.pi * f; s = 1j * w
    ms, mu, k, c, kt = qc.m_s, qc.m_u, qc.k, qc.c, qc.k_t
    # [ms s^2 + c s + k, -(c s + k); -(c s + k), mu s^2 + c s + k + kt] [zs; zu] = [0; kt zr]
    det = (ms * s ** 2 + c * s + k) * (mu * s ** 2 + c * s + k + kt) - (c * s + k) ** 2
    zs = (c * s + k) * kt / det; zu = (ms * s ** 2 + c * s + k) * kt / det
    H = s * (zs - zu) * motion_ratio
    G = iso8608_psd(f, speed_ms, road_class)
    sigma = float(_m.sqrt(np.trapezoid(np.abs(H) ** 2 * G, f)))
    low = _m.erf(knee_ms / (sigma * _m.sqrt(2.0)))
    return {"shaft_v_rms_ms": sigma, "share_below_knee": low, "share_above_knee": 1.0 - low}
