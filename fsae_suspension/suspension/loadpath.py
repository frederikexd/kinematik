# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Suspension member load path — the axial force in every link under a wheel load.

To know how much a control arm or tie rod DEFLECTS, you first need the force it
carries. This module resolves the wheel-contact load into the axial force in each
suspension member, using the standard motorsport idealisation: every link is a
PIN-JOINTED TWO-FORCE MEMBER, so it carries load only along its own axis (e.g.
RCVD / Pashley load-path analysis). The outboard corner assembly (upright + the
instantaneously-rigid wishbones) is then a free body held by a determinate set of
those axial force lines, and a 6×6 equilibrium gives every member force at once.

MEMBERS (double wishbone, modelling each A-arm as two independent legs):
    UF, UR  upper wishbone front / rear legs   (force at the upper ball joint)
    LF, LR  lower wishbone front / rear legs   (force at the lower ball joint)
    TR      tie rod                            (force at the tie-rod outer)
    PR      pushrod                            (force at the pushrod outer)

Six members, six equilibrium equations (3 force + 3 moment) → statically determinate.
The pushrod is the member that carries the vertical/spring load path; without a
defined pushrod the vertical reaction is not uniquely determined by the five
remaining links, so a clearly-labelled vertical strut proxy is substituted and the
result is flagged, rather than returning a confidently wrong number.

SIGN CONVENTION
    Member force T > 0  ⇒ TENSION (the link is being stretched, ball joint pulled
    inboard); T < 0 ⇒ compression. The force a member exerts on the upright is
    T · û, with û the unit vector from the member's outboard point toward its
    inboard (chassis) point.

All coordinates mm, forces N, moments N·mm, in the same SAE vehicle axes as the
kinematics solver (x rear, y right, z up).
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass


# Canonical member order used throughout.
MEMBERS = ["UF", "UR", "LF", "LR", "TR", "PR"]


@dataclass
class WheelLoad:
    """
    Load applied at the contact patch, SAE axes (N), plus an optional aligning
    torque about the vertical (N·mm).

      Fx : longitudinal (+rearward) — braking/traction reaction
      Fy : lateral (+toward +y/right) — cornering force from the ground on the tyre
      Fz : vertical (+up) — the tyre's share of vehicle weight + load transfer
      Mz : aligning torque about z (N·mm), optional
    """
    Fx: float = 0.0
    Fy: float = 0.0
    Fz: float = 0.0
    Mz: float = 0.0
    #: longitudinal force (N, +rearward) whose torque about the spin axis is
    #: carried by a SHAFT to the chassis rather than by the upright: traction
    #: through an inboard final drive, or braking through inboard brakes. The
    #: ground applies it at the contact patch, but the wheel and hub pass it to
    #: the upright through the bearing at the wheel centre with no moment about
    #: the spin axis, because the halfshaft reacts Fx x R. It is therefore
    #: applied at the wheel centre here. Outboard brakes (caliper on the
    #: upright) and hub motors react their torque through the upright: put
    #: those in ``Fx``, at the contact patch.
    Fx_wc: float = 0.0
    #: further force (N) and moment (N*mm) the hub puts on the upright at the
    #: wheel centre, SAE axes: halfshaft plunge friction and the secondary
    #: couple of an angled CV joint (see halfshaft.py). Zero by default.
    F_wc_extra: tuple = (0.0, 0.0, 0.0)
    M_wc_extra: tuple = (0.0, 0.0, 0.0)

    def force(self) -> np.ndarray:
        """Total force on the corner, N: contact-patch and wheel-centre parts."""
        return (np.array([self.Fx + self.Fx_wc, self.Fy, self.Fz])
                + _vec(self.F_wc_extra))


@dataclass
class MemberForces:
    """Axial force (N, + tension) in each member, plus solver diagnostics."""
    forces: dict                      # member -> axial force (N)
    axes: dict                        # member -> unit axis (outboard->inboard)
    outboard: dict                    # member -> outboard application point (mm)
    condition: float                  # condition number of the equilibrium matrix
    residual: float                   # ‖A·f − b‖ (N), should be ~0 when determinate
    has_pushrod: bool                 # False ⇒ vertical strut proxy was used
    note: str = ""
    #: force each inboard pickup puts on the chassis (N, corner axes). For two-force members it is -T·û; for the legs
    #: of a wishbone that carries the pushrod it is the full pivot reaction, which need not lie along the leg.
    pickup_forces: dict | None = None
    #: force across each such leg at its inner pivot (N): it bends the leg; zero for two-force members
    transverse: dict | None = None
    #: distance along each such leg from its pivot to where the pushrod load enters (mm), for the bending moment
    bending_arm_mm: dict | None = None
    #: "two-force" (pushrod on the upright: every member axial) or "wishbone-pushrod" (two-body solution)
    model: str = "two-force"

    def tension(self, m: str) -> float:
        return float(self.forces.get(m, 0.0))

    def as_dict(self):
        return {
            "forces_N": {k: float(v) for k, v in self.forces.items()},
            "condition": float(self.condition),
            "residual_N": float(self.residual),
            "has_pushrod": self.has_pushrod,
            "note": self.note,
        }


def _vec(v) -> np.ndarray:
    """A point or vector as a float array. The one place the conversion happens."""
    return np.asarray(v, float)


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v  # pragma: no mutate (float threshold: < and <= differ only on exact equality)


def _member_geometry(kin, state):
    """
    Build (outboard_point, inboard_point) for each member from the solved corner
    state and the hardpoints. Returns a dict member -> (p_out, p_in). PR is omitted
    when no rocker/pushrod is defined.
    """
    hp = kin.hp
    g = {
        "UF": (state.upper_outer, _vec(hp.upper_front_inner)),
        "UR": (state.upper_outer, _vec(hp.upper_rear_inner)),
        "LF": (state.lower_outer, _vec(hp.lower_front_inner)),
        "LR": (state.lower_outer, _vec(hp.lower_rear_inner)),
        "TR": (state.tie_rod_outer, _vec(hp.tie_rod_inner)),
    }
    if kin._has_rocker and state.pushrod_outer is not None:
        g["PR"] = (_vec(state.pushrod_outer),
                   _vec(hp.rocker_pushrod))
    return g


def solve_member_forces(kin, state, load: WheelLoad) -> MemberForces:
    """
    Resolve a contact-patch load into the axial force in each suspension member.

    kin   : a SuspensionKinematics (for hardpoints + rocker info)
    state : a solved CornerState (geometry the loads act through)
    load  : the WheelLoad at the contact patch

    Returns MemberForces. When no pushrod is defined, a vertical strut proxy from
    the lower ball joint carries the vertical path and `has_pushrod` is False.
    """
    if (kin._has_rocker and state.pushrod_outer is not None
            and kin._pushrod_body in ("lower", "upper")):
        return _solve_wishbone_pushrod(kin, state, load)
    geom = _member_geometry(kin, state)
    members = list(geom.keys())
    cp = _vec(state.contact_patch)

    has_pushrod = "PR" in geom
    note = ""
    if not has_pushrod:
        # Substitute a vertical strut at the lower ball joint so the system stays
        # determinate. This models a direct-acting spring reacting vertical load;
        # it is a proxy, not the real bell-crank path, and we say so.
        lo = _vec(state.lower_outer)
        geom["STRUT"] = (lo, lo + np.array([0.0, 0.0, 200.0]))  # pragma: no mutate (only the direction is used)
        members.append("STRUT")
        note = ("No pushrod/rocker defined — a direct-acting vertical strut proxy "
                "was used for the vertical load path. Define the rocker for the real "
                "pushrod force.")

    if len(members) != 6:          # only a modified member set (a redundant leg) gets here
        note += (f" (Expected 6 force members for a determinate solve, got "
                 f"{len(members)}.)")

    # Assemble the 6×N equilibrium matrix about the contact patch.
    A = np.zeros((6, len(members)))
    axes = {}
    outb = {}
    for j, m in enumerate(members):
        p_out, p_in = geom[m]
        u = _unit(p_in - p_out)              # force on assembly = T·u
        axes[m] = u
        outb[m] = p_out
        A[0:3, j] = u
        A[3:6, j] = np.cross(p_out - cp, u)  # pragma: no mutate (moment about the contact patch; A has 6 rows)

    F = load.force()
    M_applied = _applied_moment(state, load, cp)
    if np.any(load.F_wc_extra) or np.any(load.M_wc_extra):
        note += " Hub force/moment at the wheel centre included (halfshaft)."
    if load.Fx_wc:
        note += (f" {load.Fx_wc:+.0f} N of longitudinal force applied at the wheel "
                 "centre (torque reacted by a shaft, not the upright).")
    b = np.concatenate([-F, -M_applied])

    cond = float(np.linalg.cond(A)) if A.shape[1] == 6 else np.inf
    if np.isfinite(cond) and cond < 1e10:  # pragma: no mutate (threshold; above 1e8 both routes agree and the note flags it)
        f = np.linalg.solve(A, b)
    else:
        # Degenerate / non-square: least-squares so a result still comes back, but
        # the high condition number in the report flags it as untrustworthy.
        f, *_ = np.linalg.lstsq(A, b, rcond=None)  # pragma: no mutate
    residual = float(np.linalg.norm(A @ f - b))

    forces = {m: float(f[j]) for j, m in enumerate(members) if m != "STRUT"}
    # keep STRUT out of the public member set but record it in the note
    if "STRUT" in members:
        forces.setdefault("PR", 0.0)         # no real pushrod force
    if cond > 1e8:  # pragma: no mutate (float threshold: < and <= differ only on exact equality)
        note += (f" Equilibrium matrix is poorly conditioned (cond≈{cond:.1e}); the "
                 "linkage may be near a singular configuration — treat member loads "
                 "with caution.")

    names = {"UF": "upper_front_inner", "UR": "upper_rear_inner", "LF": "lower_front_inner",
             "LR": "lower_rear_inner", "TR": "tie_rod_inner"}
    pick = {names[m]: -forces[m] * axes[m] for m in names if m in forces}   # each link's pull on the chassis
    return MemberForces(forces=forces, axes=axes, outboard=outb,
                        condition=cond, residual=residual,
                        has_pushrod=has_pushrod, note=note.strip(), pickup_forces=pick)


def _applied_moment(state, load, cp):
    """Moment (N*mm) about the contact patch of the load's non-patch parts: aligning torque, shaft-reacted Fx and hub
    extras at the wheel centre."""
    wc = _vec(state.wheel_center)
    # the shaft-reacted Fx acts at the wheel centre, so about the patch it carries
    # only the lever (wc - cp) x F, like the hub extras
    f_wc = np.array([float(load.Fx_wc), 0.0, 0.0]) + _vec(load.F_wc_extra)
    return (np.array([0.0, 0.0, float(load.Mz)]) + np.cross(wc - cp, f_wc)
            + _vec(load.M_wc_extra))


def _solve_wishbone_pushrod(kin, state, load: WheelLoad) -> MemberForces:
    """Two-body statics when the pushrod acts on a wishbone, not on the upright.

    The upright is held by the other wishbone's two legs and the tie rod (two-force members) and by the ball joint
    of the pushrod-carrying wishbone, which can push in any direction: six unknowns, the upright's six equilibrium
    equations. That wishbone pivots about its inner axis, so its moment about that axis fixes the pushrod force
    (virtual work gives the same number: tests/test_load_case_physics.py). Its two pivot reactions carry the rest;
    their split along the pivot axis is statically indeterminate and is taken as the one closest to two-force legs
    (least force across the legs), which is exact when every load reaches the wishbone at its ball joint. The force
    across each leg is reported (``transverse``) with the arm to the pushrod mount, so the leg can be checked in
    bending. Units N, mm.
    """
    hp = kin.hp
    which = kin._pushrod_body
    cp = _vec(state.contact_patch)
    uo, lo, tro = (_vec(getattr(state, k)) for k in ("upper_outer", "lower_outer", "tie_rod_outer"))
    if which == "lower":
        bj, (fn, rn), (fi, ri) = lo, ("LF", "LR"), (hp.lower_front_inner, hp.lower_rear_inner)
        others = {"UF": (uo, hp.upper_front_inner), "UR": (uo, hp.upper_rear_inner)}
    else:
        bj, (fn, rn), (fi, ri) = uo, ("UF", "UR"), (hp.upper_front_inner, hp.upper_rear_inner)
        others = {"LF": (lo, hp.lower_front_inner), "LR": (lo, hp.lower_rear_inner)}
    others["TR"] = (tro, hp.tie_rod_inner)
    fi, ri = _vec(fi), _vec(ri)
    A = np.zeros((6, 6)); axes, outb = {}, {}
    for j, (m, (po, pi)) in enumerate(others.items()):
        u = _unit(_vec(pi) - po)
        axes[m], outb[m] = u, po
        A[:3, j] = u; A[3:, j] = np.cross(po - cp, u)
    for k in range(3):
        e = np.eye(3)[k]
        A[:3, 3 + k] = e; A[3:, 3 + k] = np.cross(bj - cp, e)
    b = np.concatenate([-load.force(), -_applied_moment(state, load, cp)])
    cond = float(np.linalg.cond(A))
    x = np.linalg.solve(A, b)
    B = x[3:]                                              # force of the wishbone on the upright, at the ball joint
    P = _vec(state.pushrod_outer)
    u_pr = _unit(_vec(hp.rocker_pushrod) - P)
    ax = _unit(ri - fi)
    T_pr = -float(ax @ np.cross(bj - fi, -B)) / float(ax @ np.cross(P - fi, u_pr))
    # wishbone: -B at the ball joint, T_pr u_pr at the mount, pivot reactions RF at fi and RR at ri
    Fw = -B + T_pr * u_pr
    Mw = np.cross(bj - fi, -B) + np.cross(P - fi, T_pr * u_pr)
    d = ri - fi
    # Any two rows of d x fix RR up to a multiple of d, and the split chosen below removes that
    # multiple, so a change to one row cannot change the answer.
    Sd = np.array([[0, -d[2], d[1]], [d[2], 0, -d[0]], [-d[1], d[0], 0]])  # pragma: no mutate
    # particular solution: RR from (d x RR = -Mw) (minimum norm, perpendicular to d), RF = -Fw - RR
    RR0 = np.linalg.lstsq(Sd, -Mw, rcond=None)[0]  # pragma: no mutate
    RF0 = -Fw - RR0
    eF, eR = _unit(fi - bj), _unit(ri - bj)                # leg axes, ball joint -> pivot (the û of each leg)
    PF, PR_ = np.eye(3) - np.outer(eF, eF), np.eye(3) - np.outer(eR, eR)
    # RF = RF0 + lam*ax, RR = RR0 - lam*ax: choose lam minimising the force across both legs
    num = -(ax @ PF @ RF0) + (ax @ PR_ @ RR0)
    den = ax @ PF @ ax + ax @ PR_ @ ax
    lam = num / den if den > 1e-12 else 0.0  # pragma: no mutate (den > 0 unless the pivot axis lies along both legs)
    RF, RR = RF0 + lam * ax, RR0 - lam * ax
    forces = {m: float(x[j]) for j, m in enumerate(others)}
    forces[fn], forces[rn] = float(RF @ eF), float(RR @ eR)
    forces["PR"] = T_pr
    axes[fn], axes[rn], outb[fn], outb[rn] = eF, eR, bj, bj
    axes["PR"], outb["PR"] = u_pr, P
    pick = {}
    names = {"UF": "upper_front_inner", "UR": "upper_rear_inner", "LF": "lower_front_inner",
             "LR": "lower_rear_inner", "TR": "tie_rod_inner"}
    for m in others:
        pick[names[m]] = -forces[m] * axes[m]
    pick[names[fn]], pick[names[rn]] = -RF, -RR
    trans = {fn: float(np.linalg.norm(PF @ RF)), rn: float(np.linalg.norm(PR_ @ RR))}
    arm = {fn: float(np.clip((P - fi) @ (-eF), 0.0, np.linalg.norm(bj - fi))),
           rn: float(np.clip((P - ri) @ (-eR), 0.0, np.linalg.norm(bj - ri)))}
    # residual: the upright's six equations and the wishbone's own balance
    res_up = float(np.linalg.norm(A @ x - b))
    # both terms are zero when the wishbone balances, so their signs cannot be seen on a balanced solve
    res_wb = float(np.linalg.norm(Fw + RF + RR)) + float(np.linalg.norm(Mw + np.cross(ri - fi, RR))) / 1000.0  # pragma: no mutate
    residual = res_up + res_wb  # pragma: no mutate (a sum of two zeros on a balanced solve)
    return MemberForces(forces=forces, axes=axes, outboard=outb, condition=cond,
                        residual=residual, has_pushrod=True,
                        note=f"pushrod on the {which} wishbone: two-body solution", pickup_forces=pick,
                        transverse=trans, bending_arm_mm=arm, model="wishbone-pushrod")


def wheel_load_from_corner(Fz: float, mu_lateral: float = 0.0,
                           mu_long: float = 0.0,
                           lateral_sign: float = -1.0,
                           inboard_drive: bool = False,
                           inboard_brakes: bool = False) -> WheelLoad:
    """
    Convenience: build a WheelLoad from a vertical tyre load and friction fractions.

      Fz          : vertical load on this tyre (N)
      mu_lateral  : lateral force as a fraction of Fz (e.g. 1.4 for a 1.4-g-capable
                    outer tyre at the limit). Fy = lateral_sign · mu_lateral · Fz.
      mu_long     : longitudinal fraction (braking/traction). Fx = mu_long · Fz.
      lateral_sign: −1 puts the cornering force toward the centreline (the inboard,
                    centripetal direction for a RIGHT-side outer wheel in a left
                    turn) — the usual compliance-steer load case.
      inboard_drive / inboard_brakes : when the traction (Fx < 0, forward) or
                  braking (Fx > 0) torque is reacted by a shaft to the chassis,
                  that force goes to ``Fx_wc`` and acts at the wheel centre.
    """
    fx = mu_long * Fz
    shaft = (fx < 0.0 and inboard_drive) or (fx > 0.0 and inboard_brakes)  # pragma: no mutate (at fx == 0 both branches give a zero load)
    return WheelLoad(Fx=0.0 if shaft else fx,
                     Fy=lateral_sign * mu_lateral * Fz,
                     Fz=Fz, Fx_wc=fx if shaft else 0.0)
