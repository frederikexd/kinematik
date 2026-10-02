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
        return (np.array([self.Fx + self.Fx_wc, self.Fy, self.Fz], float)
                + np.asarray(self.F_wc_extra, float))


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


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def _member_geometry(kin, state):
    """
    Build (outboard_point, inboard_point) for each member from the solved corner
    state and the hardpoints. Returns a dict member -> (p_out, p_in). PR is omitted
    when no rocker/pushrod is defined.
    """
    hp = kin.hp
    g = {
        "UF": (state.upper_outer, np.asarray(hp.upper_front_inner, float)),
        "UR": (state.upper_outer, np.asarray(hp.upper_rear_inner, float)),
        "LF": (state.lower_outer, np.asarray(hp.lower_front_inner, float)),
        "LR": (state.lower_outer, np.asarray(hp.lower_rear_inner, float)),
        "TR": (state.tie_rod_outer, np.asarray(hp.tie_rod_inner, float)),
    }
    if getattr(kin, "_has_rocker", False) and state.pushrod_outer is not None:
        g["PR"] = (np.asarray(state.pushrod_outer, float),
                   np.asarray(hp.rocker_pushrod, float))
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
    geom = _member_geometry(kin, state)
    members = list(geom.keys())
    cp = np.asarray(state.contact_patch, float)

    has_pushrod = "PR" in geom
    note = ""
    if not has_pushrod:
        # Substitute a vertical strut at the lower ball joint so the system stays
        # determinate. This models a direct-acting spring reacting vertical load;
        # it is a proxy, not the real bell-crank path, and we say so.
        lo = np.asarray(state.lower_outer, float)
        geom["STRUT"] = (lo, lo + np.array([0.0, 0.0, 200.0]))
        members.append("STRUT")
        note = ("No pushrod/rocker defined — a direct-acting vertical strut proxy "
                "was used for the vertical load path. Define the rocker for the real "
                "pushrod force.")

    if len(members) != 6:
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
        A[3:6, j] = np.cross(p_out - cp, u)  # moment about the contact patch

    F = load.force()
    M_applied = np.array([0.0, 0.0, load.Mz])
    fx_wc = float(getattr(load, "Fx_wc", 0.0))
    f_ex = np.asarray(getattr(load, "F_wc_extra", (0.0, 0.0, 0.0)), float)
    m_ex = np.asarray(getattr(load, "M_wc_extra", (0.0, 0.0, 0.0)), float)
    if np.any(f_ex) or np.any(m_ex):
        wc = np.asarray(state.wheel_center, float)
        M_applied = M_applied + np.cross(wc - cp, f_ex) + m_ex
        note += " Hub force/moment at the wheel centre included (halfshaft)."
    if fx_wc:
        # shaft-reacted longitudinal force acts at the wheel centre, so about
        # the contact patch it carries only the lever (wc - cp) x F
        wc = np.asarray(state.wheel_center, float)
        M_applied = M_applied + np.cross(wc - cp, np.array([fx_wc, 0.0, 0.0]))
        note += (f" {fx_wc:+.0f} N of longitudinal force applied at the wheel "
                 "centre (torque reacted by a shaft, not the upright).")
    b = np.concatenate([-F, -M_applied])

    cond = float(np.linalg.cond(A)) if A.shape[1] == 6 else np.inf
    if A.shape[1] == 6 and np.isfinite(cond) and cond < 1e10:
        f = np.linalg.solve(A, b)
    else:
        # Degenerate / non-square: least-squares so a result still comes back, but
        # the high condition number in the report flags it as untrustworthy.
        f, *_ = np.linalg.lstsq(A, b, rcond=None)
    residual = float(np.linalg.norm(A @ f - b))

    forces = {m: float(f[j]) for j, m in enumerate(members) if m != "STRUT"}
    # keep STRUT out of the public member set but record it in the note
    if "STRUT" in members:
        forces.setdefault("PR", 0.0)         # no real pushrod force
    if cond > 1e8:
        note += (f" Equilibrium matrix is poorly conditioned (cond≈{cond:.1e}); the "
                 "linkage may be near a singular configuration — treat member loads "
                 "with caution.")

    return MemberForces(forces=forces, axes=axes, outboard=outb,
                        condition=cond, residual=residual,
                        has_pushrod=has_pushrod, note=note.strip())


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
    shaft = (fx < 0.0 and inboard_drive) or (fx > 0.0 and inboard_brakes)
    return WheelLoad(Fx=0.0 if shaft else fx,
                     Fy=lateral_sign * mu_lateral * Fz,
                     Fz=Fz, Fx_wc=fx if shaft else 0.0)
