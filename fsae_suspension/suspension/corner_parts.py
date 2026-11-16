# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The parts the links load: pickup brackets, the upright, the hub bearings, and fatigue of links and rod ends.

The link screen stops at the links. A design judge asks where their forces go: into a bracket on the frame at the
inboard end, and into the upright at the outboard end, which carries them to the hub bearings and the tyre. This
module screens those parts with standard hand calculations, the same load cases as the links, so the load path is
closed from the contact patch to the frame. It screens; it does not replace FEA. Where a part's geometry has not
been declared it returns what the part must have (section modulus, bearing static rating) instead of a factor.

Methods (all closed form):
  * upright pickup loads: each member's axial force along its axis, applied at its outboard point, summed per
    joint; their sum with the contact-patch load is checked for equilibrium. Where the pushrod loads the lower
    wishbone, the lower ball joint's force is the balance of the upright instead;
  * upright arms: each arm from the wheel centre to a joint as a cantilever, M = |r x F| (N*mm), sigma = M / Z;
  * hub bearings: two bearings a declared distance apart on the spin axis share the contact-patch force and its
    moment about the wheel centre; P0 = X0 Fr + Y0 Fa (ISO 76), s0 = C0 / P0;
  * pickup brackets: double-shear clevis root bending (genesis_analysis.bracket_fos) at each member's worst force;
  * link weld fatigue: the axial stress range over the repeated service cases against the declared IIW detail
    class at the design life (FAT at 2e6 cycles, slope 3, constant-amplitude limit 0.585 FAT at 1e7);
  * rod-end shank fatigue: the ball's friction moment against the thread's VDI 2230 endurance (joints.py), raised
    for a design life under 2e6 cycles by the same slope-3 line (VDI 2230 finite life).
A fatigue factor is the allowable stress at the design life over the stress: 1 means the part just reaches it.
Units: forces in N, lengths in mm, moments in N*mm, stresses in MPa, factors dimensionless.
"""
from __future__ import annotations

import math

import numpy as np

#: Joints on the upright: the members that meet there.
JOINTS = {"upper ball joint": ("UF", "UR"), "lower ball joint": ("LF", "LR"),
          "tie-rod end": ("TR",), "pushrod": ("PR",)}
_JOINT_POINT = {"upper ball joint": "upper_outer", "lower ball joint": "lower_outer",
                "tie-rod end": "tie_rod_outer", "pushrod": "pushrod_outer"}

#: Load cases whose names mark a one-off shock, left out of the fatigue (repeated service) range.
SHOCK_WORDS = ("bump", "kerb", "curb", "strike", "pothole")

#: IIW constant-amplitude fatigue limit as a fraction of FAT (slope m = 3 from 2e6 to 1e7 cycles).
CAFL_FRACTION = (2e6 / 1e7) ** (1.0 / 3.0)


def upright_loads(hp, load_cases):
    """Force (N, SAE axes) on the upright at each joint, per load case, and the equilibrium residual (N).

    Returns [{"load_case", "joint", "point" (mm), "F" (N, 3-vector), "residual_N"}]. A member in tension pulls the
    upright toward its inboard end, so it acts along the member's axis (outboard to inboard) with its signed force.
    """
    from . import loadpath as lp
    from .kinematics import SuspensionKinematics
    kin = SuspensionKinematics(hp)
    state = kin.solve_at_travel(0.0)
    out = []
    for lc in load_cases:
        load = lc.wheel_load()
        mf = lp.solve_member_forces(kin, state, load)
        total = np.zeros(3)
        rows = []
        on_wishbone = getattr(mf, "model", "two-force") != "two-force"
        for joint, members in JOINTS.items():
            if on_wishbone and joint in ("lower ball joint", "pushrod"):
                continue                      # the pushrod loads the lower wishbone, not the upright
            got = [m for m in members if m in mf.forces and m in mf.axes]
            if not got:
                continue
            F = sum(float(mf.forces[m]) * np.asarray(mf.axes[m], float) for m in got)
            total += F
            pt = getattr(state, _JOINT_POINT[joint], None)
            rows.append({"load_case": lc.name, "joint": joint,
                         "point": None if pt is None else np.asarray(pt, float), "F": F})
        if on_wishbone:
            # the lower ball joint carries what the upper joint and tie rod do not: the upright's force balance
            F = -(load.force() + total)
            total += F
            pt = getattr(state, "lower_outer", None)
            rows.append({"load_case": lc.name, "joint": "lower ball joint",
                         "point": None if pt is None else np.asarray(pt, float), "F": F})
        resid = float(np.linalg.norm(total + load.force()))
        for r in rows:
            r["residual_N"] = resid
        out += rows
    return out


def upright_arms(loads, wheel_center, yield_mpa, section_modulus_mm3=None, fos_target=1.5):
    """Each upright arm (wheel centre to joint) as a cantilever at its worst case.

    Returns [{"joint", "arm_mm", "moment_Nmm", "load_case", "Z_required_mm3", "fos"}]: the section modulus the arm
    needs at its root for ``fos_target`` on ``yield_mpa`` (MPa), and, where ``section_modulus_mm3`` maps the joint to
    the arm's declared Z (mm^3), its factor of safety."""
    wc = np.asarray(wheel_center, float)
    worst = {}
    for r in loads:
        if r["point"] is None:
            continue
        arm = r["point"] - wc
        M = float(np.linalg.norm(np.cross(arm, r["F"])))
        if M > worst.get(r["joint"], {}).get("moment_Nmm", -1.0):
            worst[r["joint"]] = {"joint": r["joint"], "arm_mm": float(np.linalg.norm(arm)), "moment_Nmm": M,
                                 "load_case": r["load_case"]}
    out = []
    for j, w in worst.items():
        w["Z_required_mm3"] = fos_target * w["moment_Nmm"] / float(yield_mpa)
        z = (section_modulus_mm3 or {}).get(j)
        w["fos"] = (float(yield_mpa) * float(z) / w["moment_Nmm"]) if z and w["moment_Nmm"] > 0 else None
        out.append(w)
    return out


def hub_bearings(load_cases, tyre_radius_mm, spacing_mm, wheel_offset_mm=0.0, x0=0.6, y0=0.5, c0_kN=None,
                 s0_target=2.0):
    """Static load on the more loaded of two hub bearings, per load case.

    The bearings sit ``spacing_mm`` apart on the spin axis, centred on the wheel centre; the contact patch is
    ``tyre_radius_mm`` below it and ``wheel_offset_mm`` outboard. The patch force (N) and its moment about the
    wheel centre split between the bearings as a beam on two supports; one bearing takes the axial force. P0 = x0
    Fr + y0 Fa (ISO 76; 0.6 and 0.5 for deep-groove bearings). Returns [{"load_case", "Fr_N", "Fa_N", "P0_N",
    "C0_required_kN", "s0"}] with s0 only where ``c0_kN`` (kN) is declared."""
    R, s, e = float(tyre_radius_mm), float(spacing_mm), float(wheel_offset_mm)
    out = []
    for lc in load_cases:
        Fx, Fy, Fz = float(lc.Fx + getattr(lc, "Fx_wc", 0.0)), float(lc.Fy), float(lc.Fz)
        # moments about the wheel centre that the bearing pair reacts as a couple along the spin axis:
        # vertical plane: Fz at the patch offset e outboard, Fy at the patch R below the centre
        m_vert = Fz * e + Fy * R
        m_long = Fx * e
        fz = [Fz / 2 + m_vert / s, Fz / 2 - m_vert / s]
        fx = [Fx / 2 + m_long / s, Fx / 2 - m_long / s]
        fr = [math.hypot(a, b) for a, b in zip(fz, fx)]
        i = int(np.argmax(fr))
        P0 = max(x0 * fr[i] + y0 * abs(Fy), fr[i])
        row = {"load_case": lc.name, "Fr_N": fr[i], "Fa_N": abs(Fy), "P0_N": P0,
               "C0_required_kN": s0_target * P0 / 1000.0, "s0": None}
        if c0_kN:
            row["s0"] = float(c0_kN) * 1000.0 / P0
        out.append(row)
    return out


def brackets(screen_rows, reach_mm, plate_width_mm, plate_thickness_mm, n_plates, allowable_mpa, kt):
    """Clevis root bending (genesis_analysis.bracket_fos) at each member's worst force. ``screen_rows`` are the
    link screen's rows (force in N). Returns [{"member", "force_N", "load_case", "stress_mpa", "fos"}]."""
    from .genesis_analysis import bracket_fos
    worst = {}
    for r in screen_rows:
        if abs(r["force_N"]) > abs(worst.get(r["member"], {}).get("force_N", 0.0)):
            worst[r["member"]] = r
    out = []
    for m, r in worst.items():
        b = bracket_fos(abs(float(r["force_N"])), reach_mm, plate_width_mm, plate_thickness_mm, int(n_plates),
                        allowable_mpa, kt)
        out.append({"member": m, "force_N": float(r["force_N"]), "load_case": r["load_case"],
                    "stress_mpa": b["stress_mpa"], "fos": b["fos"]})
    return out


def life_factor(cycles, knee=2e6):
    """Allowable stress at ``cycles`` over the allowable at ``knee`` cycles on a slope-3 S-N line (dimensionless)."""
    return (knee / float(cycles)) ** (1.0 / 3.0)


def link_weld_fatigue(screen_rows, tube_area_mm2, fat_mpa, cycles=1e5):
    """Each link's tube-to-insert weld over the repeated service cases (no shock cases) at a design life.

    The stress range is the axial force range over those cases, unloaded included (N), over the tube area (mm^2);
    the allowable range is FAT (MPa) on the IIW slope-3 line at ``cycles``, never below the constant-amplitude limit
    (CAFL_FRACTION x FAT at 1e7 cycles and beyond). ``tube_area_mm2`` maps a member to its tube area. Returns
    [{"member", "range_MPa", "limit_MPa", "fos"}]."""
    by = {}
    for r in screen_rows:
        if any(w in r["load_case"].lower() for w in SHOCK_WORDS):
            continue
        by.setdefault(r["member"], [0.0]).append(float(r["force_N"]))
    limit = float(fat_mpa) * max(life_factor(cycles), CAFL_FRACTION) if cycles < 1e7 else CAFL_FRACTION * float(fat_mpa)
    out = []
    for m, fs in by.items():
        rng = (max(fs) - min(fs)) / float(tube_area_mm2[m])
        out.append({"member": m, "range_MPa": rng, "limit_MPa": limit,
                    "fos": limit / rng if rng > 0 else math.inf})
    return out


def rod_end_fatigue(screen_rows, nominal_dia_mm, threads_per_inch, ball_radius_mm, mu_ball=0.1, cycles=1e5):
    """Rod-end shank bending fatigue (joints.shank_fatigue_check) at each member's largest service force, with the
    endurance raised for a design life under 2e6 cycles (VDI 2230 finite life, slope 3). ``mu_ball``: 0.05 to 0.1
    PTFE-lined, up to 0.2 metal on metal. Diameter and ball radius in mm, forces in N, stresses in MPa. Returns
    [{"member", "force_N", "stress_amp_MPa", "endurance_amp_MPa", "fos"}]."""
    from .joints import shank_fatigue_check
    peak = {}
    for r in screen_rows:
        if any(w in r["load_case"].lower() for w in SHOCK_WORDS):
            continue
        peak[r["member"]] = max(peak.get(r["member"], 0.0), abs(float(r["force_N"])))
    out = []
    for m, F in peak.items():
        c = shank_fatigue_check(F, ball_radius_mm, nominal_dia_mm, threads_per_inch, mu_ball)
        se = c["endurance_amp_MPa"] * max(1.0, life_factor(cycles))
        out.append({"member": m, "force_N": F, "stress_amp_MPa": c["stress_amp_MPa"], "endurance_amp_MPa": se,
                    "fos": se / c["stress_amp_MPa"] if c["stress_amp_MPa"] > 0 else math.inf})
    return out
