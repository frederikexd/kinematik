# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
steering_margin.py — how much the steering-effort wall has to spare, and what
restores it, for any corner a manifest declares.

The effort wall (inverse_genesis.SteerEffort) is met on one tire with one
estimate of the zero-slip pneumatic trail. Its margin is what matters before a
part is made: this module states it against any set of tires and an assumed
larger trail, and pre-checks the change that restores the wall without moving
a pickup, a spindle offset in the upright (the hub moved forward or rearward
relative to the kingpin axis), by re-evaluating every wall and band of the manifest on the
offset geometry.

    margin(manifest, tires, t0_assumed_mm)      hand torque, trail limit and the mechanical trail
                                                 each tire needs at the assumed trail
    offset_check(manifest, dx_mm)                every wall and band on the offset geometry
    offset_for_trail(manifest, trail_mm)         the offset that gives a mechanical trail

The design-case wheel loads, effort limit and steering ratio come from the
manifest's own SteerEffort and geometry; nothing here is specific to one car.
Loads N, lengths mm, angles deg, torques N*m.
"""
from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
from ._num import vec as _vec

from . import inverse_genesis as ig
from . import steering_feel as sf
from .kinematics import SuspensionKinematics

__all__ = ["lock_check", "standstill_scrub_torque", "corner", "mechanical_trail", "steer_effort", "hand_torque", "trail_limit", "trail_for_effort", "spindle_offset",
           "offset_for_trail", "offset_check", "margin"]


def mechanical_trail(hp) -> float:
    """Ground-level mechanical trail (mm) of the static corner."""
    return ig.mechanical_trail_mm(SuspensionKinematics(hp).solve_at_travel(0.0))


def steer_effort(manifest) -> ig.SteerEffort:
    """The manifest's declared SteerEffort (loads in N, effort limit in N·m, rack in mm per turn), or the default
    when it declares none."""
    vol = manifest.objects()[2]
    se = getattr(getattr(vol, "properties", None), "steer_effort", None)
    return se if se is not None else ig.SteerEffort()


def hand_torque(tire, trail_mm: float, ratio: float, fz_outer_n: float, fz_inner_n: float,
                t0_outer_mm: float | None = None, alpha_max_deg: float = 22.0) -> float:
    """Worst combined aligning torque of both front wheels over slip, at the steering wheel (N*m).

    The outer wheel's zero-slip trail is ``t0_outer_mm`` (brush theory when None); the inner keeps its brush-theory
    trail."""
    co = sf.aligning_curve(tire, fz_outer_n, trail_mm, t0_outer_mm, alpha_max_deg=alpha_max_deg, n=441)
    ci = sf.aligning_curve(tire, fz_inner_n, trail_mm, alpha_max_deg=alpha_max_deg, n=441)    # brush trail
    return float(np.max(np.asarray(co["M_Nm"]) + np.asarray(ci["M_Nm"]))) / ratio


def trail_limit(tire, trail_mm: float, ratio: float, fz_outer_n: float, fz_inner_n: float,
                effort_limit_nm: float = 10.0) -> float:
    """Zero-slip pneumatic trail of the outer wheel (mm) at which the hand torque reaches the limit."""
    lo, hi = 2.0, 80.0
    for _ in range(60):  # pragma: no mutate
        m = 0.5 * (lo + hi)
        lo, hi = (m, hi) if hand_torque(tire, trail_mm, ratio, fz_outer_n, fz_inner_n, m) < effort_limit_nm \
            else (lo, m)
    return lo


def trail_for_effort(tire, ratio: float, fz_outer_n: float, fz_inner_n: float, t0_outer_mm: float | None,
                     hi_mm: float, effort_limit_nm: float = 10.0) -> float:
    """Largest mechanical trail (mm, at most ``hi_mm``) that keeps the hand torque at the limit."""
    lo = -20.0  # pragma: no mutate
    for _ in range(50):  # pragma: no mutate
        m = 0.5 * (lo + hi_mm)
        h = hand_torque(tire, m, ratio, fz_outer_n, fz_inner_n, t0_outer_mm)
        below = h < effort_limit_nm  # pragma: no mutate
        lo, hi_mm = (m, hi_mm) if below else (lo, m)
    return lo


def spindle_offset(hp, dx_mm: float):
    """The corner with the wheel centre and contact patch moved by the offset (mm) along x; caster, kingpin and every pickup
    unchanged. The corner frame's x points rearward, so a negative offset moves the wheel forward of the kingpin
    axis and shortens the mechanical trail."""
    w, c = _vec(hp.wheel_center), _vec(hp.contact_patch)
    return replace(hp, wheel_center=tuple(w + [dx_mm, 0.0, 0.0]), contact_patch=tuple(c + [dx_mm, 0.0, 0.0]))


def offset_for_trail(hp, trail_mm: float) -> float:
    """Spindle offset (mm along x) that gives the mechanical trail ``trail_mm``."""
    lo, hi = -40.0, 40.0
    for _ in range(60):  # pragma: no mutate
        mid = 0.5 * (lo + hi)
        longer = mechanical_trail(spindle_offset(hp, mid)) > trail_mm  # pragma: no mutate
        lo, hi = (lo, mid) if longer else (mid, hi)
    return 0.5 * (lo + hi)


def corner(manifest):
    """The manifest's recorded winner, or its declared start when it has none (Hardpoints, mm)."""
    hp = manifest.recorded.get("winner_hardpoints") if manifest.recorded else None
    from .genesis_repro import hp_from_dict
    return hp_from_dict(hp) if hp else manifest.objects()[0]


def offset_check(manifest, dx_mm: float, hp=None) -> dict:
    """Every wall and band of ``manifest`` on its winner (or ``hp``) before and after a spindle offset.

    Returns {"dx_mm", "trail_mm": (before, after), "before", "after", "walls_hold", "bands_hold"} (lengths in mm,
    angles in deg); the property
    values are those the manifest's walls bound, "clearance_mm" is the wheel-envelope margin beyond the wall and
    "band" the worst fraction of the curve bands used."""
    hp = corner(manifest) if hp is None else hp
    _, tg, vol, _ = manifest.objects()
    new = spindle_offset(hp, dx_mm)

    def _props(h):
        v = dict(vol.properties.evaluate(h) or {}) if vol.has_property_bounds() else {}
        r, ok = tg.residual(h)
        v["band"] = float(np.max(np.abs(r))) if ok else math.nan
        em = vol.envelope_margin(h)
        v["clearance_mm"] = None if em is None else float(em)
        return {k: (None if x is None else float(x)) for k, x in v.items()}
    before, after = _props(hp), _props(new)
    walls = (not vol.keepout_violations(new)) and (not vol.has_property_bounds() or not vol.property_violations(new)) \
        and (after["clearance_mm"] is None or after["clearance_mm"] >= 0.0)
    return {"dx_mm": float(dx_mm), "trail_mm": (mechanical_trail(hp), mechanical_trail(new)), "before": before,
            "after": after, "walls_hold": bool(walls), "bands_hold": bool(after["band"] <= 1.0)}


def margin(manifest, tires: dict, t0_assumed_mm: float | None = None, hp=None) -> dict:
    """The effort wall's margin on each tire (name -> tire model) for the manifest's winner (or ``hp``); torques in
    N·m, trails in mm.

    Per tire: hand torque at the estimated (brush-theory) trail and, when given, at ``t0_assumed_mm``; the trail
    limit; and the mechanical trail that would hold the limit at the assumed trail."""
    hp = corner(manifest) if hp is None else hp
    se = steer_effort(manifest)
    ratio, trail = float(se.kinematic_ratio(hp)), mechanical_trail(hp)
    out = {"trail_mm": trail, "ratio": ratio, "fz_outer_n": se.fz_outer_n, "fz_inner_n": se.fz_inner_n,
           "effort_limit_nm": se.effort_limit_nm, "t0_assumed_mm": t0_assumed_mm, "tires": {}}
    for name, t in tires.items():
        r = {"hand_brush_Nm": hand_torque(t, trail, ratio, se.fz_outer_n, se.fz_inner_n),
             "t0_limit_mm": trail_limit(t, trail, ratio, se.fz_outer_n, se.fz_inner_n, se.effort_limit_nm)}
        if t0_assumed_mm is not None:
            r["hand_assumed_Nm"] = hand_torque(t, trail, ratio, se.fz_outer_n, se.fz_inner_n, t0_assumed_mm)
            r["trail_for_limit_mm"] = trail_for_effort(t, ratio, se.fz_outer_n, se.fz_inner_n, t0_assumed_mm, trail,
                                                       se.effort_limit_nm)
        out["tires"][name] = r
    return out


def _steered(hp, rack_mm: float):
    k = SuspensionKinematics(hp, pickup_deltas={"tie_rod_inner": np.array([0.0, float(rack_mm), 0.0])})
    return k.solve_at_travel(0.0)


def _lateral_arm(st) -> float:
    """Moment arm (mm) of a lateral tire force at the contact patch about the steering axis, the wheel steered."""
    U, L, cp = (_vec(v) for v in (st.upper_outer, st.lower_outer, st.contact_patch))
    k = (U - L) / np.linalg.norm(U - L)
    t = np.radians(float(st.toe))
    lat = np.array([np.sin(t), np.cos(t), 0.0])        # the axle direction turns with the wheel (x rearward)
    return float(abs(np.dot(np.cross(cp - L, lat), k)))


def lock_check(manifest, hp=None, tire=None, h_mm: float = 0.25) -> dict:
    """The steering at full lock: what changes from the straight-ahead effort check.

    At the rack travel that reaches the declared lock (SteerEffort.lock_sw_deg), with the manifest's design-case
    wheel loads and with both wheels at the static load (their mean):

    * the moment arm of each wheel's lateral force about its steering axis (mm), against straight ahead;
    * weight jacking: the rack force (N) the wheel loads put on the steering as the steered contact patches rise or
      fall relative to the body (kingpin inclination and caster), by virtual work, F = -sum Fz dz/dx, and the hand
      torque it needs (N*m, positive resisting the steer);
    * the straight-ahead worst hand torque (N*m) scaled by the larger arm ratio, plus the jacking torque.

    The left wheel is this corner at the opposite rack travel (a left-right symmetric car)."""
    hp = corner(manifest) if hp is None else hp
    se = steer_effort(manifest)
    from .tiremodel import default_tire
    tire = default_tire() if tire is None else tire
    c = float(se.rack_c_mm_per_rev)
    x = float(se.lock_sw_deg(hp)) * c / 360.0
    sgn = 1.0 if _steered(hp, 1.0).toe > _steered(hp, 0.0).toe else -1.0      # rack direction that steers this
    xi, xo = sgn * x, -sgn * x                                                 # corner inward (inner wheel)

    def _dz(xr):
        return (float(_steered(hp, xr + h_mm).contact_patch[2]) - float(_steered(hp, xr - h_mm).contact_patch[2])) \
            / (2.0 * h_mm)
    # the steering moves the rack by +x toward the turn; this corner (inner) sees rack sgn*x, the mirror (outer) -sgn*x
    dzi_dx, dzo_dx = sgn * _dz(xi), -sgn * _dz(xo)
    lever = c / (2.0 * np.pi) / 1000.0                                         # m of rack per rad of wheel

    def _jack(fi, fo):
        F = -(fi * dzi_dx + fo * dzo_dx)                                       # N at the rack
        return {"rack_N": float(F), "hand_Nm": float(F * lever)}
    a0 = _lateral_arm(_steered(hp, 0.0))
    ai, ao = _lateral_arm(_steered(hp, xi)), _lateral_arm(_steered(hp, xo))
    ratio = float(se.kinematic_ratio(hp))
    base = hand_torque(tire, mechanical_trail(hp), ratio, se.fz_outer_n, se.fz_inner_n, se.t0_outer_mm)
    jd = _jack(se.fz_inner_n, se.fz_outer_n)
    st = 0.5 * (se.fz_inner_n + se.fz_outer_n)
    js = _jack(st, st)
    return {"rack_mm": x, "steer_inner_deg": float(_steered(hp, xi).toe), "steer_outer_deg": float(_steered(hp, xo).toe),
            "arm_straight_mm": a0, "arm_inner_mm": ai, "arm_outer_mm": ao,
            "jacking_design": jd, "jacking_static": js,
            "hand_straight_Nm": base,
            "hand_lock_Nm": base * max(ai, ao) / a0 + max(jd["hand_Nm"], js["hand_Nm"], 0.0)}


def standstill_scrub_torque(fz_n: float, mu: float, inflation_kpa: float = 83.0) -> float:
    """Torque (N*m) to twist a tire on the ground at standstill, as uniform pressure on a circular patch of the same
    area: (2/3) mu Fz a, a = sqrt(Fz / (p pi)). An order-of-magnitude estimate: a rolling tire needs far less."""
    a = float(np.sqrt(fz_n / (inflation_kpa * 1e-3 * np.pi)))                 # mm
    return 2.0 / 3.0 * mu * fz_n * a / 1000.0
