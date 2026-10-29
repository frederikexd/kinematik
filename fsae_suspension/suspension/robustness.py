# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Robustness of the synthesised corner to tire heat, compliance and service.

    balance_under_tire_change   how far the neutral front load-transfer share
                                moves when one axle's tires lose (or gain) grip,
                                and whether restoring balance needs a hardpoint
                                (roll centre) or only a set-up change (bar)
    clevis_sensitivity          what a deflection of each chassis pickup along
                                its own link does to static camber and toe, bump
                                steer and camber gain, re-solved on the linkage
    preset_tie_rod_toe          toe error of a spare tie rod pre-set to length on
                                the bench and fitted without re-alignment
Units: loads N, lengths mm, angles deg, shares and scales dimensionless.
"""
from __future__ import annotations

import math

import numpy as np
from ._num import vec as _vec

from .kinematics import Hardpoints, SuspensionKinematics
from . import target_derivation as td

G0 = 9.81


def _capacity_ratio(veh, share: float, a_lat_g: float, kf: float, kr: float, wfrac: float = 0.48) -> float:
    """Front over rear normalised lateral capacity (dimensionless) with axle µ scales kf, kr."""
    lt = veh.mass_kg * G0 * a_lat_g * veh.cg_height_mm / veh.track_mm
    wf = veh.mass_kg * G0 * wfrac / 2.0; wr = veh.mass_kg * G0 * (1.0 - wfrac) / 2.0
    cf = kf * sum(td.mu_of_load(f) * f for f in (wf + share * lt, wf - share * lt)) / (2 * wf)
    cr = kr * sum(td.mu_of_load(f) * f for f in (wr + (1 - share) * lt, wr - (1 - share) * lt)) / (2 * wr)
    return cf / cr


def balance_under_tire_change(front_mu_scale: float = 1.0, rear_mu_scale: float = 1.0, margin: float = 0.02,
                              a_lat_g: float = 1.5, k_rear_nm_deg: float = 405.0, k_front_nm_deg: float = 454.0,
                              rc_front_mm: float = 58.5, rc_rear_mm: float = 39.5) -> dict:
    """Neutral front load-transfer share (dimensionless) after a tire-grip change, and what restores the margin.

    The design keeps the front ``margin`` above neutral (53% against 51%). When
    hot front tires lose grip, neutral moves down; keeping the same margin needs
    either a lower front roll centre (mm, a hardpoint change) or a softer front
    bar (N·m/deg, a set-up change) — both are reported.
    """
    veh = td.Vehicle(); lo, hi = 0.3, 0.8  # pragma: no mutate (bracket: the neutral share sits well inside it; checked in the tests)
    for _ in range(80):  # pragma: no mutate (bisection count)
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if _capacity_ratio(veh, mid, a_lat_g, front_mu_scale, rear_mu_scale) > 1.0 else (lo, mid)  # pragma: no mutate (float threshold: the two comparisons differ only on exact equality)
    neutral = lo; target = neutral + margin
    per_mm = td.share_per_mm_front_rc(veh)
    s_needed = td.stiffness_share_for_lltd(target, rc_front_mm, rc_rear_mm)
    kf_needed = s_needed / (1.0 - s_needed) * k_rear_nm_deg
    lltd_now = td.front_lltd_share(rc_front_mm, rc_rear_mm, k_front_nm_deg / (k_front_nm_deg + k_rear_nm_deg))
    return {"neutral_share": neutral, "target_share": target, "current_share": lltd_now,
            "share_error_points": 100.0 * (lltd_now - target),
            "front_rc_change_mm": (target - lltd_now) / per_mm, "front_bar_change_nm_deg": kf_needed - k_front_nm_deg}


LINKS = {"upper_front_inner": "upper_outer", "upper_rear_inner": "upper_outer", "lower_front_inner": "lower_outer",
         "lower_rear_inner": "lower_outer", "tie_rod_inner": "tie_rod_outer"}


def _curves(hp, deltas=None, travel=25.0):
    """Static camber and toe (deg), bump-steer range (deg) and mean camber gain (deg/mm) over ±travel mm."""
    k = SuspensionKinematics(hp, pickup_deltas=deltas or {})
    s0, sb, sr = k.solve_at_travel(0.0), k.solve_at_travel(travel), k.solve_at_travel(-travel)
    toes = [k.solve_at_travel(t).toe for t in np.linspace(-travel, travel, 11)]  # pragma: no mutate (sampling: toe is monotonic over travel on any sane corner, so the ends decide)
    return {"camber": float(s0.camber), "toe": float(s0.toe), "bump_steer": float(max(toes) - min(toes)),
            "camber_gain": float((sb.camber - sr.camber) / (2 * travel))}


def clevis_sensitivity(hp: Hardpoints, deflection_mm: float = 1.0) -> dict:
    """Effect of moving each chassis pickup ``deflection_mm`` (mm) along its own link, toward the upright.

    Axial link load deflects a clevis along the link, so that is the direction
    tested. Returns per pickup the change in static camber and toe (deg), in the
    bump-steer range over ±25 mm (deg) and in mean camber gain (deg/mm).
    """
    base = _curves(hp); out = {}
    for inner, outer in LINKS.items():
        a = _vec(getattr(hp, inner)); b = _vec(getattr(hp, outer))
        u = (b - a) / np.linalg.norm(b - a)
        c = _curves(hp, {inner: u * deflection_mm})
        out[inner] = {k: c[k] - base[k] for k in base}
    return out


def preset_tie_rod_toe(arm_mm: float, length_tol_mm: float = 0.05, tpi: float = 24.0, flats: int = 6) -> dict:
    """Toe error (deg) of a spare tie rod pre-set to length and fitted without re-alignment.

    A bench length gauge sets the spare to ±length_tol_mm; an untrimmed thread
    engagement error is half a flat. Arm in mm; angles in deg.
    """
    flat = 25.4 / tpi / flats
    return {"preset_error_deg": math.degrees(math.atan(length_tol_mm / arm_mm)),
            "half_flat_error_deg": math.degrees(math.atan(0.5 * flat / arm_mm)),
            "band_deg": 0.08}


_MEMBER_ENDS = {"UF": ("upper_front_inner", "upper_outer"), "UR": ("upper_rear_inner", "upper_outer"),
                "LF": ("lower_front_inner", "lower_outer"), "LR": ("lower_rear_inner", "lower_outer"),
                "TR": ("tie_rod_inner", "tie_rod_outer"), "PR": ("rocker_pushrod", "pushrod_outer")}


def member_margins(hp: Hardpoints, loads: dict, od_mm: float = 15.875, wall_mm: float = 0.889,
                   e_mpa: float = 205000.0, sy_mpa: float = 460.0, crook: float = 1.0 / 1000.0) -> dict:
    """Euler and Perry–Robertson margins of every link under named wheel loads.

    ``loads`` maps case names to loadpath.WheelLoad (N at the contact patch).
    Compression: buckling factor P_cr/P with P_cr = π²EI/L² (pinned ends), and a
    yield factor from the Perry–Robertson peak stress P/A·[1 + e c/r² /(1 − P/P_cr)]
    with initial crookedness e = crook·L. Tension: σ_y A / F. A leg of the wishbone that
    carries the pushrod also bends (loadpath's two-body solution reports the force across it
    and the arm to the mount): its moment adds M/Z to the peak stress, amplified by
    1/(1 − P/P_cr) in compression. Forces N, lengths mm, stresses MPa, factors dimensionless.
    """
    from .loadpath import solve_member_forces
    A = math.pi / 4 * (od_mm ** 2 - (od_mm - 2 * wall_mm) ** 2); I = math.pi / 64 * (od_mm ** 4 - (od_mm - 2 * wall_mm) ** 4)
    r2, c = I / A, od_mm / 2.0
    Z = I / c                                       # section modulus, mm^3
    kin = SuspensionKinematics(hp); st = kin.solve_at_travel(0.0); out = {}
    for case, ld in loads.items():
        mf = solve_member_forces(kin, st, ld)
        trans = mf.transverse or {}
        arm = mf.bending_arm_mm or {}
        for mem, F in mf.forces.items():
            p0, p1 = (_vec(getattr(hp, e)) for e in _MEMBER_ENDS[mem]); L = float(np.linalg.norm(p1 - p0))
            pcr = math.pi ** 2 * e_mpa * I / L ** 2
            # a leg of the wishbone that carries the pushrod is pinned at its pivot and loaded across by the pivot
            # reaction's transverse part: bending moment V * a at the pushrod mount, a its distance from the pivot
            Mb = float(trans.get(mem, 0.0)) * float(arm.get(mem, 0.0))
            if F < 0:  # pragma: no mutate (a member at exactly zero force is checked in tension; at 0 < F < 1 N either branch gives no failure)
                P = -F
                amp = 1.0 / (1 - P / pcr) if P < pcr else math.inf  # pragma: no mutate (float threshold: the two comparisons differ only on exact equality)
                smax = P / A * (1 + crook * L * c / r2 * amp) + Mb / Z * amp if P < pcr else math.inf  # pragma: no mutate (float threshold: the two comparisons differ only on exact equality)
                fb, fy = pcr / P, sy_mpa / smax
            else:
                fb, fy = math.inf, sy_mpa / max(F / A + Mb / Z, 1e-9)
            r = out.setdefault(mem, {"length_mm": L, "p_cr_n": pcr, "worst_n": 0.0, "worst_case": "", "fos_buckling": math.inf, "fos_yield": math.inf})
            if abs(F) > abs(r["worst_n"]): r["worst_n"], r["worst_case"] = float(F), case
            r["fos_buckling"] = min(r["fos_buckling"], fb); r["fos_yield"] = min(r["fos_yield"], fy)
    return out


def overtravel_check(hp: Hardpoints, bump_mm: float = 50.0, droop_mm: float = 25.0, stroke_mm: float = 57.0, n: int = 31,
                     damper_extended_mm: float | None = None) -> dict:
    """Pushrod-rocker behaviour to ``bump_mm`` of jounce (mm): transmission angles (deg), motion ratio, stroke.

    A linkage near toggle shows a transmission angle approaching 0 or 180 deg and
    a motion ratio that reverses; a binding one fails to solve. The damper end of
    travel is where the stroke used from full droop reaches ``stroke_mm``; with the
    damper's extended eye-to-eye length ``damper_extended_mm`` (mm) it is where the
    spring reaches the damper's closed length, ``damper_extended_mm - stroke_mm``,
    so the margin to top-out at full droop is not counted as stroke.
    """
    from .actuation import actuation_metrics
    m = actuation_metrics(hp, travel_mm=(-droop_mm, bump_mm), n=n)
    if not m.ok: return {"ok": False, "reason": m.reason}
    ta = _vec(m.transmission_deg); mr = _vec(m.mr); trv = _vec(m.travel)
    mono = bool(np.all(np.diff(mr) <= 1e-9) or np.all(np.diff(mr) >= -1e-9))  # pragma: no mutate (float threshold: the two comparisons differ only on exact equality)
    used = np.concatenate([[0.0], np.cumsum(0.5 * (mr[1:] + mr[:-1]) * np.diff(trv))])
    avail = stroke_mm if damper_extended_mm is None else float(m.spring_length_mm[0] - (damper_extended_mm - stroke_mm))
    bottom = float(np.interp(avail, used, trv)) if used[-1] >= avail else None
    return {"ok": True, "transmission_min_deg": float(np.nanmin(ta)), "transmission_max_deg": float(np.nanmax(ta)),
            "mr_min": float(np.nanmin(mr)), "mr_max": float(np.nanmax(mr)), "monotonic": mono,
            "toggle_margin_deg": float(min(np.nanmin(ta), 180.0 - np.nanmax(ta))),
            "all_steps_solved": bool(np.all(np.isfinite(mr))),
            "stroke_used_mm": float(used[-1]), "stroke_available_mm": avail, "damper_bottoms_at_mm": bottom}


def hand_roll_centre(hp: Hardpoints) -> dict:
    """Front-view instant centre and roll-centre height (mm) by classical vector mechanics, at static.

    Each ball joint moves perpendicular to its wishbone's pivot axis a (unit vector
    from the front to the rear inner pickup): v = a × (j − p_front). Projected onto
    the front (y-z) view, the instant centre lies on the line through j
    perpendicular to v; the two lines meet at the IC. The roll centre is where the
    line from the contact patch through the IC crosses the centreline y = 0.
    """
    def line(f, r, j):
        """Front-view point and direction (mm) of the perpendicular to ball joint j's projected velocity."""
        f, r, j = (_vec(x) for x in (f, r, j))
        v = np.cross(r - f, j - f); return j[1:], np.array([-v[2], v[1]])     # only the direction of v matters
    pu, nu = line(hp.upper_front_inner, hp.upper_rear_inner, hp.upper_outer)
    pl, nl = line(hp.lower_front_inner, hp.lower_rear_inner, hp.lower_outer)
    s, _ = np.linalg.solve(np.array([nu, -nl]).T, pl - pu); ic = pu + s * nu  # pragma: no mutate (the sign of the second direction only flips the unused parameter)
    cp = _vec(hp.contact_patch); slope = (ic[1] - cp[2]) / (ic[0] - cp[1])
    return {"ic_y_mm": float(ic[0]), "ic_z_mm": float(ic[1]), "rc_height_mm": float(cp[2] - slope * cp[1])}


def shock_cases(axle: str, brake_front: float = 0.73, kick: float = 0.3, mass_kg: float = 300.0,
                inboard_drive: bool = True, inboard_brakes: bool = True) -> dict:
    """The declared shock load cases at one axle's outer wheel (loadpath.WheelLoad, N, SAE axes).

    braking 1.8 g      the axle's share of a 1.8 g stop at ``brake_front``, on the
                       dynamic vertical load
    curb strike 3 g    three times the static corner load with a longitudinal and
                       lateral kick of ``kick`` times that load (declared), the lateral
                       kick outboard and, as a second case, inboard: a kerb can push
                       the wheel either way, and the two load different legs
    curb + braking     the curb strike during the 1.8 g stop (front), or during a
                       1.06 g traction exit (rear)
    Units N; the cases are declared design loads, not measurements.
    """
    from .loadpath import WheelLoad
    v = td.Vehicle(); g = G0; wf = 0.48
    if axle == "front":
        st = mass_kg * g * wf / 2.0; fx = brake_front * mass_kg * g * 1.8 / 2.0
        fz = td.corner_loads(v, a_long_g=1.8)["total"]["FL"]
        return {"braking 1.8 g": WheelLoad(Fx=fx, Fz=fz), "curb strike 3 g + kick": WheelLoad(Fx=kick * 3 * st, Fy=kick * 3 * st, Fz=3 * st),
                "curb strike 3 g + inboard kick": WheelLoad(Fx=kick * 3 * st, Fy=-kick * 3 * st, Fz=3 * st),
                "curb strike while braking": WheelLoad(Fx=fx, Fz=3 * st)}
    st = mass_kg * g * (1 - wf) / 2.0; fxb = (1 - brake_front) * mass_kg * g * 1.8 / 2.0
    fzb = td.corner_loads(v, a_long_g=1.8)["total"]["RL"]
    # a chassis-mounted final drive (and inboard rear brakes) react the shaft torque, so their longitudinal force
    # reaches the upright at the wheel centre (loadpath.WheelLoad.Fx_wc), as the anti-squat and bearing cases declare
    fxt = -mass_kg * g * 1.06 / 2.0
    brake = WheelLoad(Fx_wc=fxb, Fz=fzb) if inboard_brakes else WheelLoad(Fx=fxb, Fz=fzb)
    trac = WheelLoad(Fx_wc=fxt, Fz=3 * st) if inboard_drive else WheelLoad(Fx=fxt, Fz=3 * st)
    return {"braking 1.8 g": brake, "curb strike 3 g + kick": WheelLoad(Fx=kick * 3 * st, Fy=kick * 3 * st, Fz=3 * st),
            "curb strike 3 g + inboard kick": WheelLoad(Fx=kick * 3 * st, Fy=-kick * 3 * st, Fz=3 * st),
            "traction 1.06 g + curb": trac}


def upright_brake_requirement(brake_front: float = 0.73, decel_g: float = 1.8, mass_kg: float = 300.0,
                              wheel_radius_mm: float = 228.0, caster_limit_deg: float = 0.1,
                              toe_limit_deg: float = 0.027) -> dict:
    """Upright stiffness the caliper torque of a hard stop demands.

    The front caliper reacts brake_front x m x a x R / 2 (N·m) per wheel. Holding
    the caster change under ``caster_limit_deg`` needs a rotational stiffness of
    torque / limit (N·m/deg); ``toe_limit_deg`` (a third of the +/- 0.08 deg band)
    is the bench acceptance for the toe the same torque induces.
    """
    torque = brake_front * mass_kg * G0 * decel_g / 2.0 * wheel_radius_mm / 1000.0
    return {"torque_Nm": torque, "caster_stiffness_Nm_per_deg": torque / caster_limit_deg,
            "toe_limit_deg": toe_limit_deg}
