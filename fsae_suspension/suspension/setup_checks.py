# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Build-and-run checks behind the paper's additional results (section 6).

Every function works on a solved corner or on declared inputs; nothing here
is a measurement.

    thermal_fit            differential expansion of an aluminium upright
                           against steel pins: span growth and bore clearance
    camber_per_shim        camber change for a shim at the upper inner pivots,
                           solved on the linkage (lateral or vertical shim)
    toe_arm_mm             effective arm between tie-rod length and toe,
                           solved on the linkage
    toe_thread_resolution  toe per flat, band in rod travel, backlash
    link_lengths / tube_mass / bom_cost
    euler_margins          pinned-pinned buckling factor per link
    hand_torque_at_collapse steering-wheel torque when pneumatic trail is gone
    frame_twist_toe_scaled linear scaling of frame-twist toe with load
"""
from __future__ import annotations

import math

import numpy as np

from .kinematics import Hardpoints, SuspensionKinematics

LINKS = {"UF": ("upper_front_inner", "upper_outer"), "UR": ("upper_rear_inner", "upper_outer"),
         "LF": ("lower_front_inner", "lower_outer"), "LR": ("lower_rear_inner", "lower_outer"),
         "TR": ("tie_rod_inner", "tie_rod_outer"), "PR": ("pushrod_outer", "rocker_pushrod")}


def thermal_fit(dT_c: float, span_mm: float, pin_d_mm: float,
                alpha_upright: float = 23.6e-6, alpha_pin: float = 11.7e-6,
                slot_clearance_mm: float = 0.5) -> dict:
    """Differential growth of upright against pin over a temperature rise.

    The pickup span grows by (a_u - a_p) L dT relative to the steel pin, and
    the bore diameter by (a_u - a_p) d dT relative to it: a positive bore
    change means the clearance opens.
    """
    d = alpha_upright - alpha_pin
    span_um = d * span_mm * dT_c * 1000.0
    return {"span_growth_um": span_um, "bore_opening_um": d * pin_d_mm * dT_c * 1000.0,
            "slot_margin": slot_clearance_mm * 1000.0 / span_um if span_um > 0 else math.inf}


def _camber(hp, deltas=None):
    return float(SuspensionKinematics(hp, pickup_deltas=deltas or {}).solve_at_travel(0.0).camber)


def _toe(hp, deltas=None):
    return float(SuspensionKinematics(hp, pickup_deltas=deltas or {}).solve_at_travel(0.0).toe)


def camber_per_shim(hp: Hardpoints, shim_mm: float = 0.5, direction: str = "lateral") -> float:
    """Camber change (deg) from a shim moving both upper inner pivots.

    ``lateral`` shims move the pivots inboard/outboard (y), the usual camber
    adjustment; ``vertical`` shims move them in z. Solved on the linkage at
    zero travel, so the answer carries the real arm geometry.
    """
    v = {"lateral": np.array([0.0, shim_mm, 0.0]), "vertical": np.array([0.0, 0.0, shim_mm])}[direction]
    return _camber(hp, {"upper_front_inner": v, "upper_rear_inner": v}) - _camber(hp)


def upper_arm_span_mm(hp: Hardpoints) -> float:
    """Lateral distance from the upper pivot axis midpoint to the upper ball joint."""
    mid = 0.5 * (np.asarray(hp.upper_front_inner, float) + np.asarray(hp.upper_rear_inner, float))
    return float(abs(np.asarray(hp.upper_outer, float)[1] - mid[1]))


def toe_arm_mm(hp: Hardpoints, dl_mm: float = 0.05) -> float:
    """Effective arm (mm) relating tie-rod length change to toe.

    Lengthens the tie rod by ``dl_mm`` along its own axis (moving the inner
    joint) and solves the toe change: arm = dl / tan(d toe).
    """
    ti, to = np.asarray(hp.tie_rod_inner, float), np.asarray(hp.tie_rod_outer, float)
    u = (ti - to) / np.linalg.norm(ti - to)
    dt = _toe(hp, {"tie_rod_inner": u * dl_mm}) - _toe(hp)
    return dl_mm / math.tan(math.radians(abs(dt)))


def toe_thread_resolution(arm_mm: float, tpi: float = 24.0, band_deg: float = 0.08,
                          backlash_mm: float = 0.025, flats: int = 6) -> dict:
    """Toe per flat of a threaded rod end, the band in rod travel, backlash."""
    pitch = 25.4 / tpi
    flat = pitch / flats
    return {"pitch_mm": pitch, "toe_per_flat_deg": math.degrees(math.atan(flat / arm_mm)),
            "band_rod_travel_mm": arm_mm * math.tan(math.radians(band_deg)),
            "band_in_flats": arm_mm * math.tan(math.radians(band_deg)) / flat,
            "backlash_toe_deg": math.degrees(math.atan(backlash_mm / arm_mm))}


def link_lengths(hp: Hardpoints) -> dict:
    """Pin-to-pin length (mm) of each link."""
    return {k: float(np.linalg.norm(np.asarray(getattr(hp, a), float) - np.asarray(getattr(hp, b), float)))
            for k, (a, b) in LINKS.items()}


def tube_mass_g(hp: Hardpoints, od_mm: float = 15.875, wall_mm: float = 0.889,
                rho_g_cm3: float = 7.85) -> dict:
    """Linear density (g/m) and total tube mass (g) of one corner's links."""
    idm = od_mm - 2.0 * wall_mm
    lin = math.pi / 4.0 * (od_mm ** 2 - idm ** 2) * rho_g_cm3          # mm^2 * g/cm^3 = g/m
    L = link_lengths(hp)
    return {"g_per_m": lin, "lengths_mm": L, "total_g": lin * sum(L.values()) / 1000.0}


def bom_cost(tube_ft: float, usd_per_ft: float, rod_ends: int, usd_rod_end: float,
             ptfe_joints: int = 0, usd_ptfe: float = 0.0, other_usd: float = 0.0) -> dict:
    """Structural bill of materials from declared quantities and prices."""
    tube = tube_ft * usd_per_ft
    joints = rod_ends * usd_rod_end + ptfe_joints * usd_ptfe
    return {"tube_usd": tube, "joints_usd": joints, "total_usd": tube + joints + other_usd}


def euler_margins(hp: Hardpoints, member_forces_n: dict, od_mm: float = 15.875,
                  wall_mm: float = 0.889, E: float = 205000.0) -> dict:
    """Pinned-pinned Euler load and buckling factor for each link.

    ``member_forces_n`` maps a link name to its largest compressive force (N,
    positive). A link with no compression entry gets an infinite factor.
    """
    idm = od_mm - 2.0 * wall_mm
    I = math.pi / 64.0 * (od_mm ** 4 - idm ** 4)
    out = {}
    for k, L in link_lengths(hp).items():
        pc = math.pi ** 2 * E * I / L ** 2
        f = abs(member_forces_n.get(k, 0.0))
        out[k] = {"length_mm": L, "euler_n": pc, "fos": pc / f if f > 0 else math.inf}
    return out


def hand_torque_at_collapse(fy_outer_n: float, fy_inner_n: float, mech_trail_mm: float,
                            ratio: float, budget_nm: float = 10.0) -> dict:
    """Steering-wheel torque once the pneumatic trail has collapsed at the limit.

    Only the mechanical trail is left, so the road-wheel torque is
    (Fy_outer + Fy_inner) t_m and the hand torque that over the ratio.
    """
    road = (fy_outer_n + fy_inner_n) * mech_trail_mm / 1000.0
    hand = road / ratio
    return {"road_wheel_nm": road, "hand_nm": hand, "budget_left_nm": budget_nm - hand}


def frame_twist_toe_scaled(toe_ref_deg: float, load_ref_g: float, load_g: float) -> float:
    """Frame-twist toe scales linearly with the twisting load in the uniform model."""
    return toe_ref_deg * load_g / load_ref_g
