# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Ghost Topology — the deformed geometry the car actually has mid-event, and
whether it is sabotaging the geometry you designed.

WHY THIS MODULE EXISTS
----------------------
The industry treats these as three different tools that cannot talk:

    1. a multibody/kinematics solver assumes every link is RIGID and gives
       you the motion and the loads;
    2. those loads get exported as a static load case into an FEA solver,
       which tells you whether the part bends;
    3. nobody closes the loop: the bent part changes the geometry, the
       changed geometry changes the tyre force, the changed force changes the
       bending. Co-simulating full nonlinear FEA against multibody dynamics
       is the enterprise-priced, workstation-melting answer, so teams throw a
       safety factor at it and hope.

KinematiK already owns every piece of that loop as a tested standalone part:
the rigid corner solver (kinematics.py), the member load-path resolver
(loadpath.py), the quasi-static compliance coupling (compliance.py), the
FoS screening rules (bracket_fos.py's 1.5-on-yield gate), and a transient
integrator producing per-corner load histories at millisecond resolution
(transient.py). This module is the join: it walks a transient overload
event, and at each audited instant solves the GHOST TOPOLOGY — the
instantaneous deformed suspension geometry under that instant's loads — and
reports three things the siloed workflow structurally cannot see:

    * how the deformation moves the kinematic metrics (camber, toe, the
      front-view instant centre, the roll-centre height, the contact patch)
      away from the rigid design intent, sampled through the event;
    * how the member LOAD PATHS redistribute because the deformed geometry
      reacts the same wheel load through different force lines — the load-
      share shift between the rigid and the ghost topology;
    * the TRANSIENT structural margin of every member — FoS on yield in
      tension, yield AND pinned-pinned Euler buckling in compression —
      traced through the event instead of evaluated at one hand-picked
      static case, judged against the team's standing 1.5 rule.

And it closes the loop the siloed tools cannot: the TYRE-FORCE FEEDBACK.
Compliance camber and compliance steer change the tyre's operating point,
which changes the lateral force, which changes the deflection. Per instant
this is a scalar fixed point, and its contraction ratio is measured directly:

    loop_gain = d(feedback force) / d(applied force)

|gain| < 1 : the loop is a contraction — the feedback converges, and the
             closed-loop force is the geometric-series sum, confirmed by a
             final full solve. The gain IS the stability margin, reported.
|gain| ≥ 1 : COMPLIANCE-INDUCED INSTABILITY — each increment of deflection
             recruits more force than the increment that caused it. There is
             no quasi-static equilibrium on this branch; the event verdict
             says so out loud instead of printing a fixed point that does
             not exist.

WHY NO SUPERCOMPUTER IS NEEDED (the honest trick)
-------------------------------------------------
Time-scale separation, stated and priced. A suspension link's structural
modes live at hundreds of Hz to kHz; the chassis dynamics the transient
solver integrates live at ~1–20 Hz. Across that gap the structure tracks its
load quasi-statically: the deflection at instant t is an ALGEBRAIC function
of the load at instant t, so the "co-simulation" collapses to the already-
tested compliance solve evaluated along the load history — a few
Levenberg-Marquardt corner solves per audited instant, cached across
near-identical load states. That is laptop arithmetic, not an FEA queue.

The same statement prices the limit: for events comparable to the link
modes themselves — a sub-5 ms curb impact edge, a shock load — structural
inertia matters and the quasi-static condensation is WRONG. The audit
measures the load slew rate of the event it was given and flags instants
that violate the separation instead of quietly answering anyway.

HONEST SCOPE (what this is NOT)
-------------------------------
  * Quasi-static structural response along a transient load history — not a
    structural-dynamics (modal / impact) solve. Fast edges are flagged.
  * Tangent-stiffness elasticity via compliance.py: no plasticity. The
    moment any member's transient FoS crosses 1.0 the geometry answers are
    void beyond that instant — a yielded link does not spring back — and the
    verdict machinery treats them that way rather than plotting through it.
  * One corner at a time, like compliance.py: no cross-corner chassis
    compliance coupling. Per-corner tab stiffness is in scope (k_tab).
  * The rigid baseline at travel is superposed with the compliance delta
    solved about static ride (delta-at-travel ≈ delta-at-static for the
    few-degree, few-mm range this covers). Stated, not hidden.
  * The tyre feedback is a first-order sensitivity model (∂Fy/∂camber,
    ∂Fy/∂toe at the operating point) — the honest linearisation of the Magic
    Formula about the instant, not a re-run of the tyre model, because the
    corner model does not know the wheel's slip state. Sensitivities default
    to representative FSAE magnitudes and should be overridden from the
    team's own tyre fit (tirefit.py) when one exists.

Everything downstream of ANSYS's job stays ANSYS's job: this is the tool
that tells you WHICH instant of WHICH event to give the expensive solver,
and what the pass criterion should be — in the same spirit as the rest of
the repo. Units: mm, N, MPa, deg, s. Corner axes: SAE-style right-side
corner (x rear+, y right+, z up+), same as kinematics.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from typing import Optional

import numpy as np

from .kinematics import SuspensionKinematics, Hardpoints, CornerState
from .compliance import CompliantCorner, MemberStiffness, CompliantResult
from . import loadpath as lp
from . import flex as flexmod


# Canonical member set audited for margins (the pushrod carries the spring
# path; it gets a margin row too even though it has no wishbone length key).
_MARGIN_MEMBERS = ("UF", "UR", "LF", "LR", "TR", "PR")

# Verdicts, worst-first. A ghost audit reports ONE governing verdict plus the
# full flag list, in the same spirit as bracket_fos's PASS/TIGHT/FAIL and the
# Proof Engine's three-way call.
VERDICTS = (
    "FEEDBACK_DIVERGENT",    # tyre-force loop gain ≥ 1: no quasi-static equilibrium
    "COMPLIANCE_INVERTED",   # deflection flips the SIGN of a kinematic intent
    "MARGIN_BREACHED",       # a member's transient FoS dips below the limit
    "COMPLIANCE_DEGRADED",   # measurable erosion, no inversion, margins hold
    "RIGID_FAITHFUL",        # deflections below thresholds — the rigid model is honest here
)


# --------------------------------------------------------------------------- #
#  Member structural sections — the margin side of the audit
# --------------------------------------------------------------------------- #
@dataclass
class MemberSection:
    """
    The structural identity of one link for margin purposes: a thin-walled
    round tube (the FSAE reality) with a yield strength.

      material  : key into flex.MATERIALS (for E, used by Euler buckling)
      od_mm, wall_mm : tube section
      yield_MPa : 0.2% yield. Default is the same normalized-4130 tube figure
                  bracket_fos.py carries (460 MPa), so a margin here and a
                  bracket margin there divide by the same number.

    Buckling is pinned-pinned Euler (K = 1): a two-force member on spherical
    joints IS the pinned-pinned column, so this is the honest closed form,
    not a conservatism knob. Same screening formula as tubeframe.py.
    """
    material: str = "Steel 4130"
    od_mm: float = 19.05
    wall_mm: float = 0.9
    yield_MPa: float = 460.0

    @property
    def area_mm2(self) -> float:
        idia = self.od_mm - 2.0 * self.wall_mm
        return math.pi / 4.0 * (self.od_mm ** 2 - idia ** 2)

    @property
    def I_mm4(self) -> float:
        idia = self.od_mm - 2.0 * self.wall_mm
        return math.pi / 64.0 * (self.od_mm ** 4 - idia ** 4)

    def euler_pcr_N(self, length_mm: float) -> float:
        """Pinned-pinned Euler critical load (N). Screening, same as tubeframe."""
        if length_mm <= 0:
            return float("inf")
        mat = flexmod.MATERIALS.get(self.material)
        E = mat.E if mat is not None else 205000.0
        return math.pi ** 2 * E * self.I_mm4 / (length_mm ** 2)

    def margins(self, tension_N: float, length_mm: float) -> dict:
        """
        FoS of this member carrying axial force `tension_N` (+ tension) over
        `length_mm`. Tension is checked on yield; compression on yield AND
        Euler, governing = the smaller. FoS is inf at zero load.
        """
        T = float(tension_N)
        A = self.area_mm2
        stress = abs(T) / A if A > 0 else float("inf")
        fos_yield = self.yield_MPa / stress if stress > 0 else float("inf")
        fos_buckle = float("inf")
        if T < 0:
            pcr = self.euler_pcr_N(length_mm)
            fos_buckle = pcr / abs(T) if abs(T) > 0 else float("inf")
        fos = min(fos_yield, fos_buckle)
        mode = ("buckling" if fos_buckle < fos_yield else
                ("yield (compression)" if T < 0 else "yield (tension)"))
        return {"force_N": T, "stress_MPa": stress, "fos_yield": fos_yield,
                "fos_buckle": fos_buckle, "fos": fos, "mode": mode,
                "length_mm": float(length_mm)}


def uniform_sections(od_mm: float = 19.05, wall_mm: float = 0.9,
                     material: str = "Steel 4130", yield_MPa: float = 460.0,
                     tie_od_mm: Optional[float] = None,
                     tie_wall_mm: Optional[float] = None) -> dict:
    """Every member the same tube (the common FSAE case); tie rod optionally its own."""
    out = {}
    for m in _MARGIN_MEMBERS:
        od = tie_od_mm if (m == "TR" and tie_od_mm is not None) else od_mm
        wall = tie_wall_mm if (m == "TR" and tie_wall_mm is not None) else wall_mm
        out[m] = MemberSection(material=material, od_mm=od, wall_mm=wall,
                               yield_MPa=yield_MPa)
    return out


# --------------------------------------------------------------------------- #
#  Tyre-force feedback — the loop the siloed tools cannot close
# --------------------------------------------------------------------------- #
@dataclass
class TireSensitivity:
    """
    First-order sensitivity of the tyre lateral force to the compliance
    deltas, at the operating point:

        dFy = dFy_dcamber_N_per_deg · Δcamber + dFy_dtoe_N_per_deg · Δtoe

    with Δcamber, Δtoe the COMPLIANCE deltas in degrees (kinematics.py sign
    conventions: camber negative = top inboard; toe positive = toe-out) and
    dFy applied to the SIGNED Fy of the corner model (cornering pull on the
    outer right-side wheel is −y).

    `representative(load)` builds typical FSAE magnitudes, scaled by Fz:
      * camber term: losing camber (Δcamber toward positive on a loaded
        outer wheel) sheds grip ⇒ pushes Fy toward zero. ~45 N/deg per kN.
        This is NEGATIVE feedback (grip loss unloads the links).
      * toe term: compliance toe-out on the outer wheel grows its slip angle
        ⇒ pushes Fy further in its own direction. ~300 N/deg per kN — the
        cornering stiffness, and the POSITIVE-feedback path that can go
        unstable.
    Override both from a real tyre fit; these are stand-ins and say so.
    """
    dFy_dcamber_N_per_deg: float = 0.0
    dFy_dtoe_N_per_deg: float = 0.0
    note: str = ""

    @staticmethod
    def representative(load: lp.WheelLoad,
                       camber_N_per_deg_per_kN: float = 45.0,
                       toe_N_per_deg_per_kN: float = 300.0) -> "TireSensitivity":
        Fz_kN = max(load.Fz, 0.0) / 1000.0
        direction = -1.0 if load.Fy < 0 else (1.0 if load.Fy > 0 else 0.0)
        return TireSensitivity(
            # grip loss opposes the current Fy direction:
            dFy_dcamber_N_per_deg=-direction * camber_N_per_deg_per_kN * Fz_kN,
            # slip-angle growth reinforces it:
            dFy_dtoe_N_per_deg=direction * toe_N_per_deg_per_kN * Fz_kN,
            note="representative FSAE magnitudes scaled by Fz — override "
                 "from your tyre fit for a final number")

    def dFy(self, d_camber_deg: float, d_toe_deg: float) -> float:
        return (self.dFy_dcamber_N_per_deg * d_camber_deg
                + self.dFy_dtoe_N_per_deg * d_toe_deg)


# --------------------------------------------------------------------------- #
#  One audited instant
# --------------------------------------------------------------------------- #
@dataclass
class GhostInstant:
    """The ghost topology at one instant of the event, fully attributed."""
    t: float                          # event time (s)
    load: lp.WheelLoad                # the CLOSED-LOOP load actually solved
    load_open: lp.WheelLoad           # the load before tyre feedback
    travel_mm: float                  # rigid baseline travel used (from Fz)
    # kinematic states
    rigid: CornerState                # rigid geometry at `travel_mm` (design intent)
    ghost: CornerState                # deformed geometry (compliance delta applied)
    d_camber: float                   # ghost − rigid intent, deg
    d_toe: float                      # deg (+ = compliance toe-out)
    d_rc_height_mm: float             # roll-centre height shift, mm
    d_ic_mm: np.ndarray               # instant-centre migration (y, z), mm
    d_cp_lateral_mm: float            # contact-patch lateral shift, mm
    # load-path shift: member -> {rigid_N, ghost_N, delta_N, share_shift}
    load_path_shift: dict
    # margins: member -> MemberSection.margins() dict
    margins: dict
    min_fos: float
    min_fos_member: str
    # feedback
    loop_gain: float                  # measured feedback contraction ratio
    feedback_converged: bool
    feedback_dFy_N: float             # closed-loop Fy − open-loop Fy
    # bookkeeping
    compliance: CompliantResult
    warnings: list = field(default_factory=list)

    def summary(self) -> dict:
        return {
            "t_s": self.t,
            "load_N": {"Fx": self.load.Fx, "Fy": self.load.Fy, "Fz": self.load.Fz},
            "travel_mm": self.travel_mm,
            "camber_rigid_deg": self.rigid.camber,
            "camber_ghost_deg": self.rigid.camber + self.d_camber,
            "d_camber_deg": self.d_camber, "d_toe_deg": self.d_toe,
            "d_rc_height_mm": self.d_rc_height_mm,
            "d_ic_mm": [float(v) for v in self.d_ic_mm],
            "d_cp_lateral_mm": self.d_cp_lateral_mm,
            "min_fos": self.min_fos, "min_fos_member": self.min_fos_member,
            "loop_gain": self.loop_gain,
            "feedback_converged": self.feedback_converged,
            "feedback_dFy_N": self.feedback_dFy_N,
            "warnings": list(self.warnings),
        }


def _rc_height_mm(state: CornerState, track_mm: float) -> float:
    """
    Front-view roll-centre height from one corner's IC (same construction as
    dynamics.roll_center_height): the line contact-patch → IC, intersected
    with the centreline y = 0. Returns nan for a degenerate (parallel-link) IC.
    """
    cp = state.contact_patch
    ic = state.instant_center
    if not np.all(np.isfinite(ic)):
        return float("nan")
    dy = ic[0] - cp[1]
    if abs(dy) < 1e-9:
        return float("nan")
    slope = (ic[1] - cp[2]) / dy
    return float(cp[2] + slope * (0.0 - cp[1]))


# --------------------------------------------------------------------------- #
#  The corner-level engine
# --------------------------------------------------------------------------- #
class GhostCorner:
    """
    One suspension corner with flexible links, structural sections, and the
    tyre-force feedback: everything needed to solve the ghost topology at an
    instant. Wraps a CompliantCorner (the tested compliance stack) and a
    MemberSection map (the margin side).
    """

    def __init__(self, cc: CompliantCorner, sections: Optional[dict] = None,
                 wheel_rate_N_per_mm: Optional[float] = None,
                 Fz_static_N: Optional[float] = None,
                 track_mm: float = 1200.0):
        """
        cc        : the compliant corner (geometry + per-member stiffness).
        sections  : member -> MemberSection for margins; default uniform 4130
                    matching the compliance default tube.
        wheel_rate_N_per_mm, Fz_static_N : when both given, the rigid baseline
                    at each instant is solved at travel = (Fz − Fz_static)/k,
                    so the design-intent camber/toe the ghost is judged
                    against is the RIGID VALUE AT THAT TRAVEL, not at static
                    ride. Omit either and the baseline stays at static ride
                    (travel 0) — stated in the result, never guessed.
        track_mm  : axle track for the roll-centre construction.
        """
        self.cc = cc
        self.sections = sections if sections is not None else uniform_sections()
        self.wheel_rate = wheel_rate_N_per_mm
        self.Fz_static = Fz_static_N
        self.track_mm = float(track_mm)
        self._travel_clamp = 40.0     # solver-honest travel range, mm

    # ------------------------------------------------------------------ #
    @staticmethod
    def uniform_tube(hp: Hardpoints, material: str = "Steel 4130",
                     od_mm: float = 19.05, wall_mm: float = 0.9,
                     yield_MPa: float = 460.0,
                     k_tab: Optional[float] = None,
                     wheel_rate_N_per_mm: Optional[float] = None,
                     Fz_static_N: Optional[float] = None,
                     track_mm: float = 1200.0, **compliance_kw) -> "GhostCorner":
        """The zero-FEA path: same tube everywhere, stiffness AND section from it."""
        cc = CompliantCorner.uniform_tube(hp, material=material, od_mm=od_mm,
                                          wall_mm=wall_mm, k_tab=k_tab,
                                          **compliance_kw)
        sec = uniform_sections(od_mm=od_mm, wall_mm=wall_mm, material=material,
                               yield_MPa=yield_MPa)
        return GhostCorner(cc, sec, wheel_rate_N_per_mm=wheel_rate_N_per_mm,
                           Fz_static_N=Fz_static_N, track_mm=track_mm)

    # ------------------------------------------------------------------ #
    def _rigid_at(self, travel_mm: float) -> CornerState:
        if abs(travel_mm) < 1e-9:
            return self.cc.rigid_kin.static
        st = self.cc.rigid_kin.solve_at_travel(travel_mm)
        return st if getattr(st, "converged", True) else self.cc.rigid_kin.static

    def _travel_for(self, Fz: float) -> tuple:
        """Baseline travel (mm) from the wheel rate; clamped, with a note if clamped."""
        if self.wheel_rate is None or self.Fz_static is None or self.wheel_rate <= 0:
            return 0.0, None
        tr = (Fz - self.Fz_static) / self.wheel_rate
        if abs(tr) > self._travel_clamp:
            return math.copysign(self._travel_clamp, tr), (
                f"baseline travel {tr:.1f} mm clamped to ±{self._travel_clamp:.0f} mm "
                "(outside the solver-honest range)")
        return float(tr), None

    # ------------------------------------------------------------------ #
    def solve_instant(self, load: lp.WheelLoad, t: float = 0.0,
                      tire: Optional[TireSensitivity] = None,
                      fos_limit: float = 1.5,
                      feedback_tol_N: float = 1.0,
                      max_feedback_iter: int = 8) -> GhostInstant:
        """
        Solve the ghost topology at one instant.

        The tyre-force loop is closed by MEASURED contraction: solve the
        compliance at the open-loop load, read the geometry deltas, convert
        to a feedback force dFy₁; solve again at Fy+dFy₁ and read dFy₂. The
        loop gain is g = (dFy₂ − dFy₁)/dFy₁ — the actual local derivative of
        feedback force w.r.t. applied force. |g| < 1 ⇒ the closed-loop force
        is the geometric-series limit Fy + dFy₁/(1−g), confirmed with a final
        solve (and polished by ordinary iteration if the linear estimate
        isn't inside tolerance). |g| ≥ 1 ⇒ no quasi-static equilibrium: the
        instant is returned solved at open loop, flagged FEEDBACK divergent,
        and the audit verdict machinery takes it from there.
        """
        warnings: list[str] = []
        tire = tire if tire is not None else TireSensitivity.representative(load)
        travel, tw = self._travel_for(load.Fz)
        if tw:
            warnings.append(tw)
        rigid = self._rigid_at(travel)

        def solve_at(Fy: float) -> CompliantResult:
            return self.cc.solve(lp.WheelLoad(Fx=load.Fx, Fy=Fy, Fz=load.Fz,
                                              Mz=load.Mz))

        # --- closed loop by measured contraction --------------------------- #
        res1 = solve_at(load.Fy)
        dFy1 = tire.dFy(res1.compliance_camber, res1.compliance_toe)
        loop_gain = 0.0
        converged = True
        Fy_closed = load.Fy
        res = res1
        if abs(dFy1) > feedback_tol_N:
            res2 = solve_at(load.Fy + dFy1)
            dFy2 = tire.dFy(res2.compliance_camber, res2.compliance_toe)
            loop_gain = (dFy2 - dFy1) / dFy1 if abs(dFy1) > 1e-12 else 0.0
            if abs(loop_gain) >= 1.0:
                converged = False
                res = res1                    # open-loop numbers, honestly flagged
                warnings.append(
                    f"tyre-force feedback loop gain {loop_gain:+.2f} (|g| ≥ 1): "
                    "compliance-induced instability — each deflection increment "
                    "recruits more force than caused it; no quasi-static "
                    "equilibrium on this branch. Values shown are OPEN-LOOP.")
            else:
                # geometric-series limit, then polish by iteration if needed
                Fy_closed = load.Fy + dFy1 / (1.0 - loop_gain)
                res = solve_at(Fy_closed)
                for _ in range(max_feedback_iter):
                    dFy = tire.dFy(res.compliance_camber, res.compliance_toe)
                    Fy_next = load.Fy + dFy
                    if abs(Fy_next - Fy_closed) <= feedback_tol_N:
                        break
                    Fy_closed = Fy_closed + 0.7 * (Fy_next - Fy_closed)
                    res = solve_at(Fy_closed)
                else:
                    warnings.append("feedback polish hit its iteration cap — "
                                    "closed-loop force carries ± a few N.")
        closed_load = lp.WheelLoad(Fx=load.Fx, Fy=Fy_closed, Fz=load.Fz, Mz=load.Mz)
        if not res.converged:
            warnings.append("inner compliance loop did not fully converge at this "
                            "instant — treat the last digit with care.")

        # --- ghost vs rigid intent (superposition about static, stated) ---- #
        d_cam = res.compliance_camber
        d_toe = res.compliance_toe
        ghost = res.compliant
        rc_rigid = _rc_height_mm(rigid, self.track_mm)
        rc_ghost_delta = (_rc_height_mm(res.compliant, self.track_mm)
                          - _rc_height_mm(res.rigid, self.track_mm))
        d_rc = rc_ghost_delta if np.isfinite(rc_ghost_delta) else float("nan")
        d_ic = np.asarray(res.compliant.instant_center, float) \
            - np.asarray(res.rigid.instant_center, float)
        d_cp = res.contact_patch_lateral_shift_mm

        # --- load-path shift: same load, rigid vs ghost geometry ----------- #
        mf_rigid = lp.solve_member_forces(self.cc.rigid_kin, res.rigid, closed_load)
        mf_ghost = lp.solve_member_forces(self.cc.rigid_kin, res.compliant, closed_load)
        tot_r = sum(abs(mf_rigid.tension(m)) for m in _MARGIN_MEMBERS) or 1.0
        tot_g = sum(abs(mf_ghost.tension(m)) for m in _MARGIN_MEMBERS) or 1.0
        shift = {}
        for m in _MARGIN_MEMBERS:
            Tr, Tg = mf_rigid.tension(m), mf_ghost.tension(m)
            shift[m] = {"rigid_N": Tr, "ghost_N": Tg, "delta_N": Tg - Tr,
                        "share_shift": abs(Tg) / tot_g - abs(Tr) / tot_r}

        # --- transient margins --------------------------------------------- #
        margins = {}
        min_fos, min_m = float("inf"), ""
        for m in _MARGIN_MEMBERS:
            sec = self.sections.get(m)
            if sec is None:
                continue
            L = self.cc._member_length(m, self.cc.rigid_kin, res.compliant)
            mg = sec.margins(mf_ghost.tension(m), L)
            margins[m] = mg
            if mg["fos"] < min_fos:
                min_fos, min_m = mg["fos"], m
        if min_fos < 1.0:
            warnings.append(
                f"{min_m} transient FoS {min_fos:.2f} < 1.0: yield onset — the "
                "elastic model (and every geometry number) is VOID beyond this "
                "instant; a yielded link does not spring back.")
        elif min_fos < fos_limit:
            warnings.append(f"{min_m} transient FoS {min_fos:.2f} is under the "
                            f"{fos_limit:.2f} rule during this event, though the "
                            "static case may pass.")

        return GhostInstant(
            t=float(t), load=closed_load, load_open=load, travel_mm=travel,
            rigid=rigid, ghost=ghost, d_camber=d_cam, d_toe=d_toe,
            d_rc_height_mm=d_rc, d_ic_mm=d_ic, d_cp_lateral_mm=d_cp,
            load_path_shift=shift, margins=margins,
            min_fos=min_fos, min_fos_member=min_m,
            loop_gain=float(loop_gain), feedback_converged=converged,
            feedback_dFy_N=float(Fy_closed - load.Fy),
            compliance=res, warnings=warnings)


# --------------------------------------------------------------------------- #
#  Verdict thresholds — the audit's rulebook, visible and overridable
# --------------------------------------------------------------------------- #
@dataclass
class GhostThresholds:
    """
    What each verdict requires. Every number is a knob, none is hidden.

      fos_limit          : the team's standing FoS rule (1.5 on yield, same
                           gate as bracket_fos.py).
      camber_inversion   : COMPLIANCE_INVERTED when the loaded camber crosses
                           to the OPPOSITE SIGN of the rigid intent by at
                           least this margin (deg) — e.g. a designed-negative
                           wheel driven positive: the grip-losing inversion.
      toe_flip_deg       : same idea for toe — compliance steer overwhelming
                           and reversing the kinematic toe by ≥ this much.
      degraded_camber/toe/rc/cp : the "measurable erosion" thresholds for
                           COMPLIANCE_DEGRADED vs RIGID_FAITHFUL.
      slew_limit_ms      : quasi-static validity — an audited instant whose
                           local load history swings more than half its own
                           peak-to-peak inside this window is flagged as too
                           fast for the time-scale separation.
    """
    fos_limit: float = 1.5
    camber_inversion: float = 0.05
    toe_flip_deg: float = 0.05
    degraded_camber: float = 0.5
    degraded_toe: float = 0.2
    degraded_rc_mm: float = 10.0
    degraded_cp_mm: float = 2.0
    slew_limit_ms: float = 5.0


# --------------------------------------------------------------------------- #
#  The event-level audit
# --------------------------------------------------------------------------- #
@dataclass
class GhostAudit:
    """The ghost audit of one transient event at one corner."""
    corner_label: str
    verdict: str                      # governing, from VERDICTS
    flags: list                       # every verdict condition that fired
    instants: list                    # list[GhostInstant], time-ordered
    findings: list                    # list[dict] — per-condition, attributed
    thresholds: GhostThresholds
    n_solves: int                     # compliance solves actually run
    n_cache_hits: int
    note: str = ""

    # -- convenience traces (arrays over the audited instants) ------------- #
    def trace(self, key: str) -> np.ndarray:
        pulls = {
            "t": lambda g: g.t,
            "d_camber": lambda g: g.d_camber,
            "d_toe": lambda g: g.d_toe,
            "camber_ghost": lambda g: g.rigid.camber + g.d_camber,
            "camber_rigid": lambda g: g.rigid.camber,
            "d_rc": lambda g: g.d_rc_height_mm,
            "d_cp": lambda g: g.d_cp_lateral_mm,
            "min_fos": lambda g: g.min_fos,
            "loop_gain": lambda g: g.loop_gain,
            "Fy": lambda g: g.load.Fy,
            "Fz": lambda g: g.load.Fz,
        }
        return np.array([pulls[key](g) for g in self.instants], float)

    def summary(self) -> dict:
        worst = min(self.instants, key=lambda g: g.min_fos) if self.instants else None
        return {
            "corner": self.corner_label, "verdict": self.verdict,
            "flags": list(self.flags),
            "n_instants": len(self.instants),
            "n_solves": self.n_solves, "n_cache_hits": self.n_cache_hits,
            "worst_fos": (worst.min_fos if worst else float("nan")),
            "worst_fos_member": (worst.min_fos_member if worst else ""),
            "worst_fos_t_s": (worst.t if worst else float("nan")),
            "max_d_camber_deg": (float(np.max(np.abs(self.trace("d_camber"))))
                                 if self.instants else 0.0),
            "max_d_toe_deg": (float(np.max(np.abs(self.trace("d_toe"))))
                              if self.instants else 0.0),
            "max_loop_gain": (float(np.max(np.abs(self.trace("loop_gain"))))
                              if self.instants else 0.0),
            "findings": [dict(f) for f in self.findings],
            "note": self.note,
        }


def _pick_sample_indices(t: np.ndarray, Fx: np.ndarray, Fy: np.ndarray,
                         Fz: np.ndarray, n_samples: int) -> np.ndarray:
    """
    The instants worth solving: the |Fy| peak, |Fx| peak, Fz max and min (the
    extreme load states, where compliance and margins are worst), plus a
    uniform comb over the event so the traces have shape. Deduped, sorted.
    """
    n = len(t)
    idx = {0, n - 1, int(np.argmax(np.abs(Fy))), int(np.argmax(np.abs(Fx))),
           int(np.argmax(Fz)), int(np.argmin(Fz))}
    if n_samples > 0:
        idx.update(int(i) for i in np.linspace(0, n - 1, n_samples).round())
    return np.array(sorted(idx), int)


def ghost_audit(gc: GhostCorner, t, Fx, Fy, Fz,
                corner_label: str = "corner",
                n_samples: int = 24,
                tire: Optional[TireSensitivity] = None,
                thresholds: Optional[GhostThresholds] = None,
                cache_quantum_N: float = 25.0) -> GhostAudit:
    """
    Audit one corner's ghost topology through a transient load history.

      t, Fx, Fy, Fz : the event, in CORNER-MODEL axes (x rear+, y right+,
                      z up+, N). Use `ghost_audit_transient` to feed a
                      TransientResult with the sign mapping done for you.
      n_samples     : uniform comb size on top of the always-included load
                      extremes.
      tire          : feedback sensitivities; default = representative,
                      rebuilt per instant from that instant's load.
      cache_quantum_N : instants whose (Fx,Fy,Fz) round to the same
                      `cache_quantum_N` grid share one solve — the trick that
                      keeps a 3000-step event at laptop cost.

    Never raises for a bad event: an empty or non-finite history returns an
    empty RIGID_FAITHFUL audit with the reason in `note`.
    """
    th = thresholds if thresholds is not None else GhostThresholds()
    t = np.asarray(t, float)
    Fx = np.asarray(Fx, float)
    Fy = np.asarray(Fy, float)
    Fz = np.asarray(Fz, float)
    if t.size == 0 or not (np.all(np.isfinite(Fx)) and np.all(np.isfinite(Fy))
                           and np.all(np.isfinite(Fz))):
        return GhostAudit(corner_label, "RIGID_FAITHFUL", [], [], [], th, 0, 0,
                          note="empty or non-finite load history — nothing audited.")

    idx = _pick_sample_indices(t, Fx, Fy, Fz, n_samples)

    # quasi-static validity: local swing inside the slew window
    slew_flags = set()
    if t.size > 2:
        win = th.slew_limit_ms / 1000.0
        span = float(np.max(np.abs(Fy)) - np.min(np.abs(Fy))) or 1.0
        ptp = float(np.ptp(Fy)) or 1.0
        for i in idx:
            lo = np.searchsorted(t, t[i] - win / 2.0)
            hi = np.searchsorted(t, t[i] + win / 2.0)
            if hi - lo >= 2 and np.ptp(Fy[lo:hi]) > 0.5 * abs(ptp):
                slew_flags.add(int(i))

    instants: list[GhostInstant] = []
    cache: dict = {}
    n_solves = n_hits = 0
    for i in idx:
        key = (round(Fx[i] / cache_quantum_N), round(Fy[i] / cache_quantum_N),
               round(Fz[i] / cache_quantum_N))
        if key in cache:
            base = cache[key]
            n_hits += 1
            g = GhostInstant(**{**base.__dict__, "t": float(t[i]),
                                "warnings": list(base.warnings)})
        else:
            load = lp.WheelLoad(Fx=float(Fx[i]), Fy=float(Fy[i]), Fz=float(Fz[i]))
            sens = tire if tire is not None else TireSensitivity.representative(load)
            g = gc.solve_instant(load, t=float(t[i]), tire=sens,
                                 fos_limit=th.fos_limit)
            cache[key] = g
            n_solves += 1
        if int(i) in slew_flags:
            g.warnings.append(
                f"load swings >50% of the event's range within ±{th.slew_limit_ms/2:.1f} ms "
                "of this instant — faster than the quasi-static separation is "
                "honest about; treat this instant as a flag for a structural-"
                "dynamics check, not an answer.")
        instants.append(g)

    # ---- verdict machinery ------------------------------------------------ #
    flags: list[str] = []
    findings: list[dict] = []

    def find(check, severity, message, detail):
        findings.append({"check": check, "severity": severity,
                         "message": message, "detail": detail})

    diverged = [g for g in instants if not g.feedback_converged]
    if diverged:
        flags.append("FEEDBACK_DIVERGENT")
        g0 = diverged[0]
        find("tyre-feedback stability", "fail",
             f"loop gain reached {g0.loop_gain:+.2f} at t = {g0.t*1000:.0f} ms "
             "(|g| ≥ 1): compliance-induced instability — deflection recruits "
             "force faster than force causes deflection. No quasi-static "
             "equilibrium exists on this branch.",
             {"t_s": g0.t, "loop_gain": g0.loop_gain})

    inverted = []
    for g in instants:
        cam_rigid = g.rigid.camber
        cam_ghost = cam_rigid + g.d_camber
        if (abs(cam_rigid) > 1e-6 and cam_rigid * cam_ghost < 0
                and abs(cam_ghost) >= th.camber_inversion):
            inverted.append((g, "camber", cam_rigid, cam_ghost))
        toe_rigid = g.rigid.toe
        toe_ghost = toe_rigid + g.d_toe
        if (abs(toe_rigid) > 1e-6 and toe_rigid * toe_ghost < 0
                and abs(toe_ghost) >= th.toe_flip_deg):
            inverted.append((g, "toe", toe_rigid, toe_ghost))
    if inverted:
        flags.append("COMPLIANCE_INVERTED")
        g0, what, r, gh = inverted[0]
        find("kinematic-intent inversion", "fail",
             f"{what} INVERTED at t = {g0.t*1000:.0f} ms: rigid intent "
             f"{r:+.2f}°, ghost topology {gh:+.2f}° — structural deflection "
             "under load is actively reversing the kinematic design.",
             {"t_s": g0.t, "metric": what, "rigid_deg": r, "ghost_deg": gh,
              "n_instants_inverted": len(inverted)})

    breached = [g for g in instants if g.min_fos < th.fos_limit]
    if breached:
        flags.append("MARGIN_BREACHED")
        gworst = min(breached, key=lambda g: g.min_fos)
        sev = "fail" if gworst.min_fos < 1.0 else "warning"
        find("transient structural margin", sev,
             f"{gworst.min_fos_member} FoS fell to {gworst.min_fos:.2f} "
             f"({gworst.margins[gworst.min_fos_member]['mode']}) at "
             f"t = {gworst.t*1000:.0f} ms — under the {th.fos_limit:.2f} rule "
             "mid-event"
             + (" and past yield onset: elastic answers beyond this instant "
                "are void." if gworst.min_fos < 1.0 else "."),
             {"t_s": gworst.t, "member": gworst.min_fos_member,
              "fos": gworst.min_fos,
              "mode": gworst.margins[gworst.min_fos_member]["mode"]})

    degraded = [g for g in instants if (
        abs(g.d_camber) > th.degraded_camber or abs(g.d_toe) > th.degraded_toe
        or (np.isfinite(g.d_rc_height_mm)
            and abs(g.d_rc_height_mm) > th.degraded_rc_mm)
        or abs(g.d_cp_lateral_mm) > th.degraded_cp_mm)]
    if degraded and "COMPLIANCE_INVERTED" not in flags:
        flags.append("COMPLIANCE_DEGRADED")
        gd = max(degraded, key=lambda g: abs(g.d_camber))
        find("compliance erosion", "warning",
             f"compliance moved the geometry measurably (worst: "
             f"Δcamber {gd.d_camber:+.2f}°, Δtoe {gd.d_toe:+.2f}° at "
             f"t = {gd.t*1000:.0f} ms) without inverting the intent.",
             {"t_s": gd.t, "d_camber_deg": gd.d_camber, "d_toe_deg": gd.d_toe})

    slewed = [g for g in instants if any("faster than the quasi-static" in w
                                         for w in g.warnings)]
    if slewed:
        find("quasi-static validity", "warning",
             f"{len(slewed)} audited instant(s) sit on load edges faster than "
             f"the {th.slew_limit_ms:.0f} ms separation window — those instants "
             "flag a structural-dynamics question, they do not answer it.",
             {"n_instants": len(slewed)})

    verdict = next((v for v in VERDICTS if v in flags), "RIGID_FAITHFUL")
    if verdict == "RIGID_FAITHFUL":
        find("ghost topology", "ok",
             "deflections stayed below every erosion threshold through the "
             "event — the rigid model is an honest stand-in for this event at "
             "this corner.", {})

    return GhostAudit(corner_label=corner_label, verdict=verdict, flags=flags,
                      instants=instants, findings=findings, thresholds=th,
                      n_solves=n_solves, n_cache_hits=n_hits)


# --------------------------------------------------------------------------- #
#  Feeding a TransientResult — the sign mapping, done once, out loud
# --------------------------------------------------------------------------- #
# transient.py body axes: x forward+, y LEFT+, z up+.
# corner model axes:      x rear+,    y RIGHT+, z up+ (a right-side corner;
#                         left corners are the y-mirror).
# Mapping per corner:
#   Fx_corner = −Fx_body   (x flips for every corner)
#   Fz_corner = +Fz_body
#   Fy_corner = −Fy_body for RIGHT-side corners (FR, RR): body-left+ → corner
#               right+ is a straight y flip.
#             = +Fy_body for LEFT-side corners (FL, RL): the mirror maps the
#               body's y-left+ onto the mirrored corner's own y+.
_CORNER_INDEX = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}
_RIGHT_SIDE = {"FR", "RR"}


def ghost_audit_transient(gc: GhostCorner, result, corner: str = "FR",
                          **audit_kw) -> GhostAudit:
    """
    Run the ghost audit on one corner of a transient.TransientResult, with the
    body→corner sign mapping applied (documented above). `gc` must be built
    from that axle's hardpoints. A failed transient (`result.ok` False)
    returns an empty audit that says so rather than auditing zeros.
    """
    corner = corner.upper()
    if corner not in _CORNER_INDEX:
        raise ValueError(f"corner must be one of {sorted(_CORNER_INDEX)}")
    if not getattr(result, "ok", False):
        return GhostAudit(corner, "RIGID_FAITHFUL", [], [], [],
                          GhostThresholds(), 0, 0,
                          note="transient run was flagged failed — nothing to audit: "
                               + "; ".join(getattr(result, "warnings", [])[:2]))
    ci = _CORNER_INDEX[corner]
    ysign = -1.0 if corner in _RIGHT_SIDE else 1.0
    return ghost_audit(gc, result.t,
                       Fx=-result.Fx[:, ci],
                       Fy=ysign * result.Fy[:, ci],
                       Fz=result.Fz[:, ci],
                       corner_label=corner, **audit_kw)


# --------------------------------------------------------------------------- #
#  Markdown report — the sheet that goes to the design judge / the FEA seat
# --------------------------------------------------------------------------- #
def render_ghost_md(audit: GhostAudit) -> str:
    L: list[str] = []
    s = audit.summary()
    L.append(f"# Ghost Topology audit — corner {audit.corner_label}")
    L.append("")
    L.append(f"**Verdict: `{audit.verdict}`**"
             + (f"  (flags: {', '.join(audit.flags)})" if audit.flags else ""))
    L.append("")
    if audit.note:
        L.append(f"_{audit.note}_")
        L.append("")
        return "\n".join(L)
    L.append(f"{len(audit.instants)} instants audited "
             f"({audit.n_solves} compliance solves, {audit.n_cache_hits} cache hits). "
             f"Worst transient FoS **{s['worst_fos']:.2f}** "
             f"({s['worst_fos_member']}, t = {s['worst_fos_t_s']*1000:.0f} ms). "
             f"Peak compliance: Δcamber {s['max_d_camber_deg']:.2f}°, "
             f"Δtoe {s['max_d_toe_deg']:.2f}°. "
             f"Peak tyre-feedback loop gain {s['max_loop_gain']:.2f} "
             "(1.0 is the instability boundary).")
    L.append("")
    L.append("## Findings")
    L.append("")
    for f in audit.findings:
        badge = {"fail": "🔴", "warning": "🟡", "ok": "🟢"}.get(f["severity"], "•")
        L.append(f"- {badge} **{f['check']}** — {f['message']}")
    L.append("")
    L.append("## Instants")
    L.append("")
    L.append("| t (ms) | Fy (N) | Fz (N) | Δcamber (°) | Δtoe (°) | ΔRC (mm) | "
             "min FoS | member | loop gain |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for g in audit.instants:
        rc = f"{g.d_rc_height_mm:+.1f}" if np.isfinite(g.d_rc_height_mm) else "—"
        L.append(f"| {g.t*1000:.0f} | {g.load.Fy:.0f} | {g.load.Fz:.0f} | "
                 f"{g.d_camber:+.2f} | {g.d_toe:+.2f} | {rc} | "
                 f"{g.min_fos:.2f} | {g.min_fos_member} | {g.loop_gain:+.2f} |")
    L.append("")
    L.append("_Quasi-static compliance along a transient load history — not a "
             "structural-dynamics solve, not plasticity, one corner at a time. "
             "A red instant here is the load case and pass criterion to hand "
             "the FEA seat, not a substitute for it._")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  Self-test (python3 -m suspension.ghost_topology)
# --------------------------------------------------------------------------- #
if __name__ == "__main__":   # pragma: no cover
    hp = Hardpoints.default()

    # A 3 g cornering pulse: half-sine Fy over 0.8 s on a loaded outer corner,
    # Fz rising with load transfer. Corner-model axes (cornering pull is −y).
    t = np.linspace(0.0, 0.8, 801)
    shape = np.sin(np.pi * t / 0.8)
    Fz = 700.0 + 1500.0 * shape                  # N, static → loaded
    Fy = -3.0 * Fz * shape                       # up to ~3 g utilisation
    Fx = np.zeros_like(t)

    print("=== stock 4130 corner (19.05 × 0.9) ===")
    gc = GhostCorner.uniform_tube(hp, wheel_rate_N_per_mm=35.0, Fz_static_N=700.0)
    audit = ghost_audit(gc, t, Fx, Fy, Fz, corner_label="FR-demo", n_samples=12)
    print(render_ghost_md(audit))
    print()

    print("=== the sabotage case: soft links + soft tabs ===")
    cc = CompliantCorner.uniform_tube(hp, material="Aluminium 6061",
                                      od_mm=12.0, wall_mm=1.0, k_tab=1500.0)
    gc2 = GhostCorner(cc, uniform_sections(od_mm=12.0, wall_mm=1.0,
                                           material="Aluminium 6061",
                                           yield_MPa=276.0),
                      wheel_rate_N_per_mm=35.0, Fz_static_N=700.0)
    audit2 = ghost_audit(gc2, t, Fx, Fy, Fz, corner_label="FR-soft", n_samples=12)
    print(render_ghost_md(audit2))
