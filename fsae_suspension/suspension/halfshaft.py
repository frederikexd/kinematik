# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Halfshaft secondary loads on the upright: plunge friction and the CV couple.

An inboard final drive sends its torque down a halfshaft, and the torque
itself goes to the chassis (loadpath.WheelLoad.Fx_wc). Two smaller loads do
reach the upright through the hub, and a structural screen of a driven corner
should carry them:

* PLUNGE FRICTION. The shaft length changes as the wheel moves, and the
  plunging joint (a tripod at the diff, usually) resists that sliding with a
  friction force along the shaft. For a tripod carrying torque T on a roller
  pitch radius r, the three rollers carry T / (3 r) each and the axial
  resistance is mu T / r, with mu the joint's effective sliding or rolling
  friction. It acts along the shaft axis in either direction, whichever
  opposes the plunge, so both signs are returned as separate cases.
* SECONDARY COUPLE. A constant-velocity joint running at an angle beta
  transmits torque with a secondary moment T tan(beta / 2) about the axis
  normal to the plane of the two shafts. At the outboard joint that moment is
  reacted by the hub bearing into the upright.

Plunge friction is DECLARED: a needle-roller tripod is roughly 0.01 to 0.05,
a plain one higher, and wear and grease move it. The generated axial force a
tripod produces cyclically at speed and angle is not modelled. Units: mm, N,
N*m (torque in), N*mm (moments out).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _spin_axis(camber_deg: float, toe_deg: float) -> np.ndarray:
    v = np.array([math.tan(math.radians(toe_deg)), 1.0,
                  -math.tan(math.radians(camber_deg))])
    return v / np.linalg.norm(v)


@dataclass
class HalfshaftSpec:
    """The driven corner's shaft. Point in the corner frame (mm)."""
    inboard_joint_mm: tuple              # plunging joint centre, at the diff
    tripod_pitch_radius_mm: float = 20.0
    mu_plunge: float = 0.03
    provenance: str = "declared"

    def __post_init__(self):
        self.inboard_joint_mm = tuple(float(c) for c in self.inboard_joint_mm)
        if self.tripod_pitch_radius_mm <= 0.0 or self.mu_plunge < 0.0:
            raise ValueError("HalfshaftSpec: pitch radius > 0 and mu >= 0 needed.")


def halfshaft_loads(spec: HalfshaftSpec, state, torque_Nm: float) -> dict:
    """Plunge force (N, both signs) and outboard CV couple (N*mm) on the upright.

    ``state`` is a solved CornerState (wheel centre, camber, toe). Returns the
    shaft angle (deg), the plunge force magnitude, the couple magnitude and
    the two load vectors (force at the wheel centre, moment) for each plunge
    direction, ready for WheelLoad.F_wc_extra / M_wc_extra.
    """
    wc = np.asarray(state.wheel_center, float)
    u = wc - np.asarray(spec.inboard_joint_mm, float)
    L = np.linalg.norm(u)
    if L < 1e-6:
        raise ValueError("halfshaft: inboard joint coincides with the wheel centre.")
    u /= L
    s = _spin_axis(state.camber, state.toe)
    beta = math.acos(float(np.clip(abs(u @ s), -1.0, 1.0)))
    T = abs(float(torque_Nm)) * 1000.0                      # N*mm
    f_plunge = spec.mu_plunge * T / spec.tripod_pitch_radius_mm
    m2 = T * math.tan(beta / 2.0)
    n = np.cross(s, u)
    nn = np.linalg.norm(n)
    M = (m2 * n / nn) if nn > 1e-12 else np.zeros(3)
    cases = [{"F_wc_extra": tuple(sgn * f_plunge * u), "M_wc_extra": tuple(M)}
             for sgn in (1.0, -1.0)]
    return {"shaft_angle_deg": math.degrees(beta), "plunge_force_N": f_plunge,
            "couple_Nmm": m2, "cases": cases}


def shaft_torques_nm(mass_kg: float = 300.0, wheel_radius_mm: float = 228.0,
                     traction_g: float = 1.06, drive_share: float = 1.0,
                     braking_g: float = 1.5, rear_brake_share: float = 0.40) -> dict:
    """Per-halfshaft torque (N*m) in drive and, with inboard brakes, braking.

    With the rear brakes inboard the halfshaft carries brake torque the other
    way, so its duty goes from pulsating (drive only) to fully reversed; the
    torsional fatigue check then uses the reversing range, which is the sum
    of the two magnitudes, not the larger one.
    """
    g = 9.81
    t_drive = drive_share * mass_kg * g * traction_g / 2.0 * wheel_radius_mm / 1000.0
    t_brake = rear_brake_share * mass_kg * g * braking_g / 2.0 * wheel_radius_mm / 1000.0
    return {"drive_Nm": t_drive, "brake_Nm": t_brake,
            "peak_Nm": max(t_drive, t_brake), "reversing_range_Nm": t_drive + t_brake}


def shaft_shear_mpa(torque_nm: float, od_mm: float, id_mm: float = 0.0) -> float:
    """Torsional shear stress (MPa) at the surface of a round or tubular shaft."""
    J = math.pi * (od_mm ** 4 - id_mm ** 4) / 32.0
    return abs(torque_nm) * 1000.0 * (od_mm / 2.0) / J


def goodman_min_diameter(t_max_nm: float, t_min_nm: float, sut_mpa: float = 1000.0,
                         fos: float = 1.5, surface: str = "machined", kc: float = 1.0,
                         von_mises: bool = True, d_lo: float = 10.0, d_hi: float = 60.0) -> dict:
    """Smallest solid shaft (mm) passing a Goodman screen, Shigley Marin factors.

    Mean and alternating torque come from the duty's extremes. Shear stress
    16T/(pi d^3) is converted to an equivalent normal stress by sqrt(3) when
    ``von_mises`` (then ``kc`` should be 1: the conversion already accounts
    for torsion, and a torsion load factor kc = 0.59 on top of it counts
    torsion twice). The endurance limit is 0.5 Sut (Sut <= 1400 MPa) times the
    surface factor ka = a Sut^b and the size factor kb = 1.24 d^-0.107 for
    2.79 <= d <= 51 mm, 1.51 d^-0.157 above. Criterion: s_a/Se + s_m/Sut <= 1/n.
    """
    ab = {"ground": (1.58, -0.085), "machined": (4.51, -0.265), "hot_rolled": (57.7, -0.718)}[surface]
    ka = ab[0] * sut_mpa ** ab[1]
    se0 = 0.5 * min(sut_mpa, 1400.0) if sut_mpa <= 1400 else 700.0
    tm, ta = 0.5 * (t_max_nm + t_min_nm), 0.5 * (t_max_nm - t_min_nm)
    k = math.sqrt(3.0) if von_mises else 1.0

    def util(d):
        kb = 1.24 * d ** -0.107 if d <= 51 else 1.51 * d ** -0.157
        se = ka * kb * kc * se0
        s = lambda T: k * 16.0 * abs(T) * 1000.0 / (math.pi * d ** 3)
        return s(ta) / se + s(tm) / sut_mpa, kb, se

    lo, hi = d_lo, d_hi
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if util(mid)[0] * fos <= 1.0: hi = mid
        else: lo = mid
    u, kb, se = util(hi)
    return {"d_min_mm": hi, "T_mean_nm": tm, "T_alt_nm": ta, "ka": ka, "kb": kb, "kc": kc, "Se_mpa": se}
