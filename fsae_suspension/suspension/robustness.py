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
    veh = td.Vehicle(); lo, hi = 0.3, 0.8
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if _capacity_ratio(veh, mid, a_lat_g, front_mu_scale, rear_mu_scale) > 1.0 else (lo, mid)
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
    toes = [k.solve_at_travel(t).toe for t in np.linspace(-travel, travel, 11)]
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
        a = np.asarray(getattr(hp, inner), float); b = np.asarray(getattr(hp, outer), float)
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


def structural_margins(hp: Hardpoints, cases: dict, od_mm: float = 15.875, wall_mm: float = 0.889,
                       E_mpa: float = 205000.0, sy_mpa: float = 435.0) -> dict:
    """Worst axial load in each link over the load cases, with buckling and yield factors.

    ``cases`` maps a case name to (Fx, Fy, Fz) at the contact patch, N, SAE axes.
    Each link is a pin-jointed two-force member (loadpath.solve_member_forces).
    Euler: pinned-pinned on the tube; yield: axial stress on the tube section
    against the declared 4130 yield. Forces N, lengths mm, stresses MPa.
    """
    from .loadpath import solve_member_forces, WheelLoad
    kin = SuspensionKinematics(hp); st = kin.solve_at_travel(0.0)
    idm = od_mm - 2 * wall_mm; A = math.pi / 4 * (od_mm ** 2 - idm ** 2); I = math.pi / 64 * (od_mm ** 4 - idm ** 4)
    worst = {}
    for name, (fx, fy, fz) in cases.items():
        mf = solve_member_forces(kin, st, WheelLoad(Fx=fx, Fy=fy, Fz=fz))
        for m, f in mf.forces.items():
            w = worst.setdefault(m, {"comp_n": 0.0, "comp_case": "", "tens_n": 0.0, "tens_case": ""})
            if f < w["comp_n"]: w["comp_n"], w["comp_case"] = f, name
            if f > w["tens_n"]: w["tens_n"], w["tens_case"] = f, name
    from .setup_checks import link_lengths
    L = link_lengths(hp); key = {"UF": "UF", "UR": "UR", "LF": "LF", "LR": "LR", "TR": "TR"}
    for m, w in worst.items():
        Lm = L.get(key.get(m, m))
        w["euler_n"] = math.pi ** 2 * E_mpa * I / Lm ** 2 if Lm else math.nan
        w["buckling_fos"] = w["euler_n"] / abs(w["comp_n"]) if w["comp_n"] < 0 and Lm else math.inf
        peak = max(abs(w["comp_n"]), w["tens_n"]); w["yield_fos"] = sy_mpa * A / peak if peak > 0 else math.inf
        w["length_mm"] = Lm
    return worst


def overtravel_check(hp: Hardpoints, bump_max_mm: float = 75.0, droop_mm: float = -25.0, n: int = 41) -> dict:
    """Pushrod-rocker behaviour from full droop to ``bump_max_mm`` (mm) of bump.

    A linkage near toggle has a transmission angle near 0 or 180 deg, where the
    motion ratio collapses or diverges and the wheel locks; a bind shows as a
    failed solve. Returns the transmission-angle range (deg), the worst margin
    to toggle (deg), the motion-ratio range (dimensionless) and whether every
    step solved.
    """
    from .actuation import actuation_metrics
    m = actuation_metrics(hp, travel_mm=(droop_mm, bump_max_mm), n=n)
    if not m.ok: return {"ok": False, "reason": m.reason}
    lo, hi = m.transmission_deg
    mr = [x for x in m.mr if x == x]
    return {"ok": True, "transmission_deg": (lo, hi), "toggle_margin_deg": min(lo, 180.0 - hi),
            "mr_min": min(mr), "mr_max": max(mr), "mr_static": m.mr_static, "all_steps_solved": len(mr) == len(m.mr),
            "stroke_used_mm": m.stroke_used_mm}


_MEMBER_ENDS = {"UF": ("upper_front_inner", "upper_outer"), "UR": ("upper_rear_inner", "upper_outer"),
                "LF": ("lower_front_inner", "lower_outer"), "LR": ("lower_rear_inner", "lower_outer"),
                "TR": ("tie_rod_inner", "tie_rod_outer"), "PR": ("rocker_pushrod", "pushrod_outer")}


def member_margins(hp: Hardpoints, loads: dict, od_mm: float = 15.875, wall_mm: float = 0.889,
                   e_mpa: float = 205000.0, sy_mpa: float = 460.0, crook: float = 1.0 / 1000.0) -> dict:
    """Euler and Perry–Robertson margins of every link under named wheel loads.

    ``loads`` maps case names to loadpath.WheelLoad (N at the contact patch).
    Compression: buckling factor P_cr/P with P_cr = π²EI/L² (pinned ends), and a
    yield factor from the Perry–Robertson peak stress P/A·[1 + e c/r² /(1 − P/P_cr)]
    with initial crookedness e = crook·L. Tension: σ_y A / F. Forces N, lengths mm,
    stresses MPa, factors dimensionless.
    """
    from .loadpath import solve_member_forces
    A = math.pi / 4 * (od_mm ** 2 - (od_mm - 2 * wall_mm) ** 2); I = math.pi / 64 * (od_mm ** 4 - (od_mm - 2 * wall_mm) ** 4)
    r2, c = I / A, od_mm / 2.0
    kin = SuspensionKinematics(hp); st = kin.solve_at_travel(0.0); out = {}
    for case, ld in loads.items():
        mf = solve_member_forces(kin, st, ld)
        for mem, F in mf.forces.items():
            p0, p1 = (np.asarray(getattr(hp, e), float) for e in _MEMBER_ENDS[mem]); L = float(np.linalg.norm(p1 - p0))
            pcr = math.pi ** 2 * e_mpa * I / L ** 2
            if F < 0:
                P = -F; smax = P / A * (1 + crook * L * c / r2 / (1 - P / pcr)) if P < pcr else math.inf
                fb, fy = pcr / P, sy_mpa / smax
            else:
                fb, fy = math.inf, sy_mpa * A / max(F, 1e-9)
            r = out.setdefault(mem, {"length_mm": L, "p_cr_n": pcr, "worst_n": 0.0, "worst_case": "", "fos_buckling": math.inf, "fos_yield": math.inf})
            if abs(F) > abs(r["worst_n"]): r["worst_n"], r["worst_case"] = float(F), case
            r["fos_buckling"] = min(r["fos_buckling"], fb); r["fos_yield"] = min(r["fos_yield"], fy)
    return out


def overtravel_check(hp: Hardpoints, bump_mm: float = 50.0, droop_mm: float = 25.0, stroke_mm: float = 57.0, n: int = 31) -> dict:
    """Pushrod-rocker behaviour to ``bump_mm`` of jounce (mm): transmission angles (deg), motion ratio, stroke.

    A linkage near toggle shows a transmission angle approaching 0 or 180 deg and
    a motion ratio that reverses; a binding one fails to solve. The damper end of
    travel is where the stroke used from full droop reaches ``stroke_mm``.
    """
    from .actuation import actuation_metrics
    m = actuation_metrics(hp, travel_mm=(-droop_mm, bump_mm), n=n)
    if not m.ok: return {"ok": False, "reason": m.reason}
    ta = np.asarray(m.transmission_deg, float); mr = np.asarray(m.mr, float); trv = np.asarray(m.travel, float)
    mono = bool(np.all(np.diff(mr) <= 1e-9) or np.all(np.diff(mr) >= -1e-9))
    used = np.concatenate([[0.0], np.cumsum(0.5 * (mr[1:] + mr[:-1]) * np.diff(trv))])
    bottom = float(np.interp(stroke_mm, used, trv)) if used[-1] >= stroke_mm else None
    return {"ok": True, "transmission_min_deg": float(np.nanmin(ta)), "transmission_max_deg": float(np.nanmax(ta)),
            "mr_min": float(mr.min()), "mr_max": float(mr.max()), "monotonic": mono,
            "stroke_used_mm": float(used[-1]), "damper_bottoms_at_mm": bottom}


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
        f, r, j = (np.asarray(x, float) for x in (f, r, j)); a = (r - f) / np.linalg.norm(r - f)
        v = np.cross(a, j - f); return j[1:], np.array([-v[2], v[1]])
    pu, nu = line(hp.upper_front_inner, hp.upper_rear_inner, hp.upper_outer)
    pl, nl = line(hp.lower_front_inner, hp.lower_rear_inner, hp.lower_outer)
    s, _ = np.linalg.solve(np.array([nu, -nl]).T, pl - pu); ic = pu + s * nu
    cp = np.asarray(hp.contact_patch, float); slope = (ic[1] - cp[2]) / (ic[0] - cp[1])
    return {"ic_y_mm": float(ic[0]), "ic_z_mm": float(ic[1]), "rc_height_mm": float(cp[2] - slope * cp[1])}
