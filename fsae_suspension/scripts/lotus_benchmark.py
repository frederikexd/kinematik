# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""External benchmark against Lotus Suspension Analysis (supplement S6.30): the published hardpoints of
Arevalo, Medina and Valladolid, Ingenius 20 (2018) 95-106, table 3, solved by KinematiK.

Frame conversion: x_K = -(x - x_axle) with the axles at x = 160 and 1760 mm, y unchanged,
z_K = z - (310 - 247.65) mm (ground under the 310 mm wheel centre of a 19.5 in tire). Bump is matched on
wheel-centre travel, as Lotus defines it; steer is solved with the contact-patch point held on the ground
("on the ground") and with the body and travel held ("ball joint held"). Lengths mm, angles deg."""
from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from suspension.kinematics import Hardpoints, SuspensionKinematics          # noqa: E402

CORNERS = {
    160.0: {"lower_front_inner": (15, 250, 203.59), "lower_rear_inner": (335, 250, 203.59),
            "upper_front_inner": (15, 290, 347.59), "upper_rear_inner": (335, 290, 347.59),
            "lower_outer": (148.83, 555, 219), "upper_outer": (171.17, 529, 401),
            "tie_rod_outer": (220, 549.14, 259.83), "tie_rod_inner": (220, 258.5, 234.2), "wheel_center": (160, 600, 310)},
    1760.0: {"lower_front_inner": (1443.9, 212, 227.59), "lower_rear_inner": (1732, 212, 227.59),
             "upper_front_inner": (1446.6, 256, 377.59), "upper_rear_inner": (1732, 256, 377.59),
             "lower_outer": (1730, 520, 218.83), "upper_outer": (1760, 520, 400.83),
             "tie_rod_outer": (1790, 520, 218.83), "tie_rod_inner": (1750, 190, 227.59), "wheel_center": (1760, 590, 310)},
}


def corner(x_axle: float, tire_radius_mm: float = 247.65) -> Hardpoints:
    """The source corner at x_axle (mm) in KinematiK's frame, static camber and toe zero; ground under a tire of
    the given radius (247.65 mm, 19.5 in, for camber and toe; 230 mm reproduces the source's roll-centre figure)."""
    conv = lambda p: np.array([-(p[0] - x_axle), p[1], p[2] - (310.0 - tire_radius_mm)], float)
    kw = {k: conv(v) for k, v in CORNERS[x_axle].items()}
    w = kw["wheel_center"]
    kw["contact_patch"] = np.array([w[0], w[1], 0.0])
    return replace(Hardpoints.default(), static_camber=0.0, static_toe=0.0, **kw)


def bump(x_axle: float, wheel_centre_travel=(-30, -20, -10, 0, 10, 20, 30)) -> dict:
    """Camber and toe change (deg) at wheel-centre travel (mm), relative to ride height."""
    k = SuspensionKinematics(corner(x_axle))
    T = np.linspace(-45, 45, 181)
    S = [k.solve_at_travel(float(t)) for t in T]
    wz = np.array([float(s.wheel_center[2]) for s in S])
    cam = np.array([float(s.camber) for s in S]); toe = np.array([float(s.toe) for s in S])
    z0, c0, t0 = np.interp(0, T, wz), np.interp(0, T, cam), np.interp(0, T, toe)
    out = {}
    for d in wheel_centre_travel:
        t = np.interp(z0 + d, wz, T)
        out[d] = (float(np.interp(t, T, cam) - c0), float(np.interp(t, T, toe) - t0))
    return out


def roll_centre(x_axle: float, track_mm: float, wheel_centre_travel=(-30, -20, -10, 0, 10, 20, 30),
                tire_radius_mm: float = 230.0) -> dict:
    """Roll-centre height above ground (mm) in parallel bump: the front-view roll centre of the solved corner
    (ghost_topology._rc_height_mm) less the body's travel toward the ground, R = 230 mm as in the source's figure 7."""
    from suspension.ghost_topology import _rc_height_mm
    k = SuspensionKinematics(corner(x_axle, tire_radius_mm))
    T = np.linspace(-45, 45, 901)
    S = [k.solve_at_travel(float(t)) for t in T]
    wz = np.array([float(s.wheel_center[2]) for s in S]); rc = np.array([_rc_height_mm(s, track_mm) for s in S])
    z0 = np.interp(0, T, wz)
    return {d: float(np.interp(np.interp(z0 + d, wz, T), T, rc)) - d for d in wheel_centre_travel}


def roll(x_axle: float, track_mm: float, roll_deg=(-3, -2, -1, 0, 1, 2, 3)) -> dict:
    """Camber to ground (deg) against body roll (deg): the wheel moves (track/2)·tan(roll) on its wheel centre and
    the body's roll is added back, tires on the ground."""
    k = SuspensionKinematics(corner(x_axle))
    T = np.linspace(-60, 60, 241)
    S = [k.solve_at_travel(float(t)) for t in T]
    wz = np.array([float(s.wheel_center[2]) for s in S]); cam = np.array([float(s.camber) for s in S])
    z0, c0 = np.interp(0, T, wz), np.interp(0, T, cam)
    out = {}
    for phi in roll_deg:
        t = np.interp(z0 - 0.5 * track_mm * np.tan(np.radians(phi)), wz, T)
        out[phi] = float(np.interp(t, T, cam) - c0) - phi
    return out


def steer(x_axle: float, steer_deg: float) -> dict:
    """Camber (deg) at a road-wheel steer angle: ball joints held, and with the patch held on the ground."""
    hp = corner(x_axle)
    def st(rack, t=0.0):
        return SuspensionKinematics(hp, pickup_deltas={"tie_rod_inner": np.array([0.0, rack, 0.0])}).solve_at_travel(float(t))
    def rack_for(t):
        return brentq(lambda r: float(st(r, t).toe) - steer_deg, -80.0, 80.0)
    held = float(st(rack_for(0.0)).camber)
    tt = brentq(lambda t: float(np.asarray(st(rack_for(t), t).contact_patch)[2]), -40.0, 40.0)
    return {"held": held, "on_ground": float(st(rack_for(tt), tt).camber), "travel_mm": tt}


if __name__ == "__main__":
    for xa in CORNERS:
        print(xa, {d: tuple(round(v, 4) for v in cv) for d, cv in bump(xa).items()})
    for xa, tr in ((160.0, 1200.0), (1760.0, 1180.0)):
        print('roll', xa, {k: round(v, 3) for k, v in roll(xa, tr).items()})
        print('rc', xa, {k: round(v, 1) for k, v in roll_centre(xa, tr).items()})
    for a in (-31.5, -30, -20, -10, 10, 20, 30, 31.5):
        s = steer(160.0, a)
        print(a, round(s["on_ground"], 3), round(s["held"], 3))
