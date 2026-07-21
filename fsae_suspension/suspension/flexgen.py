# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
FlexGen Engine — compliant (flexure-based) suspension synthesis.

WHY THIS EXISTS
---------------
A traditional corner locates the upright through ball joints: spherical
bearings that wear, develop play, need boots and grease, and cost mass. A
COMPLIANT corner replaces the revolute freedom of a ball joint with a
monolithic flexure blade that BENDS on purpose: zero friction, zero slop,
nothing to lubricate, and the "joint" is part of the arm. The price is that
the blade is now a structural spring in the load path — it stores strain
energy every mm of travel, it can fatigue, and under the axial component of a
cornering load it can buckle. None of KinematiK's existing solvers can see
any of that: the rigid kinematics stack (kinematics.py) treats every joint as
an ideal sphere, and the flexible-body stack (flex.py) linearises about zero
— correct for a wishbone that deflects tenths of a millimetre, WRONG for a
flexure that is DESIGNED to sweep through 20–40 degrees of rotation-by-
bending, where the load–deflection law is honestly non-linear.

WHAT THE ENGINE IS
------------------
A pseudo-rigid-body (PRB) / discretized-elastica model of the blade: the
flexure is a chain of ``n`` rigid segments of length ℓ = L/n joined by
torsional springs k_i = E·I(s_i)/ℓ.  As n grows this converges to the planar
non-linear Euler–Bernoulli elastica (the classic PRB model of Howell is the
n = 1..2 member of the same family); n = 16 is inside 1 % of the analytic
small-deflection limits and captures the large-deflection stiffening and the
axial-load softening that the linear model cannot.  Equilibrium is found by
Newton's method on the exact gradient of the total potential

    Π(φ) = Σ ½ k_i (φ_i − φ_{i−1})²  −  F·u_tip(φ)  −  M·φ_tip

and — this is the "differentiable beam mechanics" part — the analytic Hessian
H of Π is the tangent stiffness of the loaded blade, so:

  * the NON-LINEAR COMPLIANCE MATRIX at the tip is C(p) = J H⁻¹ Jᵀ
    (J = ∂u_tip/∂φ), exact at the current operating point, no finite
    differences, symmetric by construction;
  * BUCKLING falls out for free: the geometric (load) term lives inside H,
    so the axial load at which λ_min(H) → 0 is the buckling load of the
    discrete blade — computed, not looked up from an effective-length chart
    (the Euler chart value is used only as a cross-check in the tests).

The engine tracks the ELASTIC STRAIN ENERGY U(z) = Σ ½ k_i Δφ_i² stored per
mm of bump travel; its second derivative d²U/dz² = dF/dz is the wheel-rate
contribution of the flexure itself — the "equivalent air spring". A team
running blades gets that rate whether they want it or not, so the physical
coilover must be DOWNSIZED by exactly that amount to hit the target wheel
rate; `coilover_downsize` does the arithmetic and refuses (out loud) when the
blades alone already exceed the target.

WHAT THE ENGINE IS NOT — read this before trusting a number
-----------------------------------------------------------
* Planar. Bending is solved in the blade's compliant plane. The stiff
  (width) direction and torsion are reported as LINEAR parasitic-stiffness
  ratios, adequate for sizing, not for a 6-DOF corner model.
* The fatigue lint is a Goodman screen on smooth-specimen data (Shigley) with
  the surface/size factors left at 1 — a REAL flexure lives or dies on its
  edge finish, and the lint says so in its own findings. It is a "worth
  taking to FEA + test coupon" gate, not a life prediction.
* Composite blades: there is no honest isotropic fatigue law for a laminate.
  Asking the lint to certify a carbon blade returns a BLOCKER telling you to
  test coupons, and the export gives a layup ORIENTATION map (fibre axis per
  station), not a laminate schedule.
* The STEP export is a faceted planar B-rep of the tapered blade blank
  (opens as a solid in FreeCAD / SolidWorks / NX); the printable STL is the
  same solid triangulated. Fillets, mounting bosses and lattice infill are
  downstream CAD/CAM work — exporting a fake organic lattice we never
  stress-checked would be the false-confidence failure this codebase refuses.

Units: mm, N, MPa (N/mm²) throughout, energies in N·mm (1000 N·mm = 1 J) —
the same consistent set as flex.py, whose Material table this module reuses.

Self-test:  python3 -m suspension.flexgen
"""

from __future__ import annotations

import io
import math
from dataclasses import dataclass
from datetime import date
from typing import Optional

import numpy as np

from .flex import MATERIALS, Material


# --------------------------------------------------------------------------- #
#  Static-strength & fatigue properties for the flexure-relevant materials.
#  (S_ut, S_y, S_e) in MPa. S_e is the fully-reversed endurance/fatigue
#  strength of a SMOOTH specimen: for steels ≈ 0.5·S_ut (Shigley), capped at
#  700 MPa; aluminium has NO endurance limit, so the value is the 5·10⁸-cycle
#  fatigue strength; None means "no honest isotropic law exists" (laminates).
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class FatigueProps:
    S_ut: float               # ultimate tensile strength, MPa
    S_y: float                # 0.2 % yield strength, MPa
    S_e: Optional[float]      # fully-reversed fatigue strength, MPa (None: n/a)
    note: str = ""


FATIGUE = {
    # normalized 4130; heat-treated blades should override via custom props
    "Steel 4130":  FatigueProps(670.0, 435.0, 335.0, "normalized; HT raises all three"),
    "Steel mild":  FatigueProps(400.0, 250.0, 200.0, "A36-class"),
    "Aluminium 6061": FatigueProps(310.0, 276.0, 96.0,
                                   "T6; NO endurance limit — 5e8-cycle strength"),
    "Aluminium 7075": FatigueProps(572.0, 503.0, 159.0,
                                   "T6; NO endurance limit — 5e8-cycle strength"),
    "Titanium Ti-6Al-4V": FatigueProps(950.0, 880.0, 510.0,
                                       "annealed; printed Ti needs HIP + own data"),
    "Carbon (axial, representative)": FatigueProps(600.0, 600.0, None,
                                                   "laminate — coupon test required"),
}


# --------------------------------------------------------------------------- #
#  Geometry
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class BladeSection:
    """Rectangular flexure cross-section, optionally thickness-tapered.

    ``width_mm`` (b) is the STIFF direction — in a suspension blade it points
    along the wheelbase so the blade carries braking/accel loads rigidly.
    ``t`` is the COMPLIANT direction (bending thickness); a linear taper from
    root to tip moves peak strain away from the clamped root.
    """
    width_mm: float
    t_root_mm: float
    t_tip_mm: Optional[float] = None       # None -> uniform (= t_root)

    def __post_init__(self):
        if self.width_mm <= 0 or self.t_root_mm <= 0:
            raise ValueError("BladeSection: width and root thickness must be > 0")
        tt = self.t_root_mm if self.t_tip_mm is None else self.t_tip_mm
        if tt <= 0:
            raise ValueError("BladeSection: tip thickness must be > 0")
        if tt > self.t_root_mm * (1 + 1e-12):
            raise ValueError("BladeSection: reverse taper (tip thicker than "
                             "root) puts peak strain at the clamp — refused.")

    # s_frac in [0, 1] measured root -> tip
    def t_at(self, s_frac: float) -> float:
        tt = self.t_root_mm if self.t_tip_mm is None else self.t_tip_mm
        return self.t_root_mm + (tt - self.t_root_mm) * float(s_frac)

    def A_at(self, s_frac: float) -> float:
        return self.width_mm * self.t_at(s_frac)

    def I_at(self, s_frac: float) -> float:
        """Second moment about the COMPLIANT (bending) axis: b·t³/12."""
        return self.width_mm * self.t_at(s_frac) ** 3 / 12.0

    def I_stiff_at(self, s_frac: float) -> float:
        """Second moment about the STIFF axis: t·b³/12 (parasitic direction)."""
        return self.t_at(s_frac) * self.width_mm ** 3 / 12.0


@dataclass(frozen=True)
class FlexureBlade:
    """One monolithic flexure blade: clamped at the chassis, loaded at the
    outboard (upright) end.

    boundary:
      * "fixed-free"   — tip rotation free (a single blade replacing one ball
                         joint; Euler column K = 2 for the axial direction).
      * "fixed-guided" — tip rotation held at 0 (the parallel-blade / double-
                         blade flexure that keeps the upright square; Euler
                         column K = 1).
    """
    length_mm: float
    section: BladeSection
    material: Material
    boundary: str = "fixed-guided"
    n_seg: int = 16
    name: str = "blade"

    def __post_init__(self):
        if self.length_mm <= 0:
            raise ValueError("FlexureBlade: length must be > 0")
        if self.boundary not in ("fixed-free", "fixed-guided"):
            raise ValueError(f"FlexureBlade: unknown boundary {self.boundary!r}")
        if self.n_seg < 4:
            raise ValueError("FlexureBlade: n_seg >= 4 (discretized elastica)")

    # --------------------------------------------------------------- derived
    @property
    def seg_len(self) -> float:
        return self.length_mm / self.n_seg

    def spring_stations(self) -> np.ndarray:
        """s_frac of joint i, i = 1..n. Joints sit at MIDPOINT stations
        s_i = (i - 1/2)·ℓ — midpoint quadrature makes the discrete cantilever
        deflection exact to O(1/n²) instead of O(1/n)."""
        n = self.n_seg
        return (np.arange(n) + 0.5) / n

    def spring_k(self) -> np.ndarray:
        """Torsional spring constants k_i = E·I(s_i)/ℓ, N·mm/rad, i = 1..n.

        I is evaluated at the joint's midpoint station — where the discrete
        model concentrates the curvature of its tributary length ℓ.
        """
        s = self.spring_stations()
        E = self.material.E
        return np.array([E * self.section.I_at(si) / self.seg_len for si in s])

    def mass_g(self) -> float:
        """Blade blank mass, grams (rho in kg/m³, volume in mm³)."""
        n = 64
        vol = 0.0
        for i in range(n):
            vol += self.section.A_at((i + 0.5) / n) * (self.length_mm / n)
        return vol * self.material.rho * 1e-6

    # linear parasitic stiffnesses (small-deflection, root section — the
    # honest label for these is "sizing numbers", see module docstring)
    def k_compliant_linear(self) -> float:
        """Small-deflection transverse rate in the COMPLIANT plane, N/mm."""
        E, L = self.material.E, self.length_mm
        I = self.section.I_at(0.25)  # representative station for a taper
        return (12.0 if self.boundary == "fixed-guided" else 3.0) * E * I / L ** 3

    def k_stiff_linear(self) -> float:
        """Small-deflection transverse rate in the STIFF plane, N/mm."""
        E, L = self.material.E, self.length_mm
        I = self.section.I_stiff_at(0.25)
        return (12.0 if self.boundary == "fixed-guided" else 3.0) * E * I / L ** 3

    def k_axial_linear(self) -> float:
        """Axial (length-direction) rate, N/mm."""
        n = 64
        c = 0.0
        for i in range(n):
            c += (self.length_mm / n) / (self.material.E *
                                         self.section.A_at((i + 0.5) / n))
        return 1.0 / c


# --------------------------------------------------------------------------- #
#  The PRB chain solver
# --------------------------------------------------------------------------- #
@dataclass
class BladeState:
    """One solved equilibrium of the blade under a tip load."""
    fx_n: float                 # axial tip force (+ = tension along blade axis)
    fy_n: float                 # transverse tip force in the compliant plane
    m_nmm: float                # tip moment
    phis: np.ndarray            # absolute segment angles, rad (n,)
    tip_dx_mm: float            # axial foreshortening (negative = shortened)
    tip_dy_mm: float            # transverse tip travel
    tip_rot_rad: float
    strain_energy_nmm: float
    compliance: np.ndarray      # 3x3 tangent compliance [ux,uy,rot]/[Fx,Fy,M]
    curvature_per_mm: np.ndarray  # κ at each joint, 1/mm (n,)
    min_eig_h: float            # min eigenvalue of the tangent Hessian


class PRBChain:
    """Discretized-elastica / multi-spring pseudo-rigid-body solver for one
    :class:`FlexureBlade`. Planar; see module docstring for scope."""

    def __init__(self, blade: FlexureBlade):
        self.blade = blade
        self.k = blade.spring_k()                       # (n,)
        self.l = blade.seg_len
        self.n = blade.n_seg
        # Midpoint-joint chain: a rigid ℓ/2 stub from the clamp to joint 1,
        # links of ℓ between joints, and ℓ/2 from the last joint to the tip.
        self.lens = np.full(self.n, self.l)
        self.lens[-1] = self.l / 2.0
        self.x0 = self.l / 2.0
        self.guided = blade.boundary == "fixed-guided"
        # DOFs: φ_1..φ_n ; fixed-guided pins φ_n = 0 (removed from the system)
        self.ndof = self.n - 1 if self.guided else self.n

    # ---------------------------------------------------------------- utils
    def _full_phi(self, q: np.ndarray) -> np.ndarray:
        if self.guided:
            return np.concatenate([q, [0.0]])
        return q

    def _tip(self, phis: np.ndarray):
        x = self.x0 + float(np.sum(self.lens * np.cos(phis)))
        y = float(np.sum(self.lens * np.sin(phis)))
        return x, y, phis[-1]

    def _spring_energy(self, phis: np.ndarray) -> float:
        d = np.diff(np.concatenate([[0.0], phis]))
        return float(0.5 * np.sum(self.k * d * d))

    def _grad_hess(self, q: np.ndarray, fx: float, fy: float, m: float):
        """Analytic gradient & Hessian of Π over the free DOFs."""
        phis = self._full_phi(q)
        n = self.n
        # spring part over full φ, tridiagonal
        d = np.diff(np.concatenate([[0.0], phis]))
        g_full = np.zeros(n)
        for j in range(n):
            g_full[j] = self.k[j] * d[j]
            if j + 1 < n:
                g_full[j] -= self.k[j + 1] * d[j + 1]
        H_full = np.zeros((n, n))
        for j in range(n):
            H_full[j, j] += self.k[j]
            if j + 1 < n:
                H_full[j, j] += self.k[j + 1]
                H_full[j, j + 1] -= self.k[j + 1]
                H_full[j + 1, j] -= self.k[j + 1]
        # load part: −F·u − M·φ_n
        s, c = np.sin(phis), np.cos(phis)
        g_full += fx * self.lens * s - fy * self.lens * c
        g_full[-1] -= m
        H_full[np.diag_indices(n)] += fx * self.lens * c + fy * self.lens * s
        if self.guided:
            return g_full[:-1], H_full[:-1, :-1]
        return g_full, H_full

    def _jac_tip(self, phis: np.ndarray) -> np.ndarray:
        """J = ∂(x, y, φ_tip)/∂q over the FREE DOFs, (3, ndof)."""
        s, c = np.sin(phis), np.cos(phis)
        J = np.zeros((3, self.n))
        J[0, :] = -self.lens * s
        J[1, :] = self.lens * c
        J[2, -1] = 1.0
        if self.guided:
            return J[:, :-1]
        return J

    # --------------------------------------------------------------- solving
    def solve(self, fx_n: float = 0.0, fy_n: float = 0.0, m_nmm: float = 0.0,
              n_steps: int = 12, q0: Optional[np.ndarray] = None) -> BladeState:
        """Equilibrium under a tip load, by load-stepped damped Newton.

        Raises RuntimeError with an explicit message if no stable equilibrium
        is found — which for a compressive axial load past buckling is the
        physically correct answer, not a numerical accident.
        """
        q = np.zeros(self.ndof) if q0 is None else q0.astype(float).copy()
        for step in range(1, n_steps + 1):
            f = step / n_steps
            fx, fy, m = fx_n * f, fy_n * f, m_nmm * f
            for _ in range(80):
                g, H = self._grad_hess(q, fx, fy, m)
                gn = float(np.max(np.abs(g))) if g.size else 0.0
                if gn < 1e-9 * max(1.0, np.max(self.k) * 1e-6):
                    break
                mu = 0.0
                for _try in range(30):
                    try:
                        dq = np.linalg.solve(H + mu * np.eye(self.ndof), -g)
                    except np.linalg.LinAlgError:
                        dq = None
                    if dq is not None and np.all(np.isfinite(dq)):
                        # keep steps physical (< ~20 deg per joint per iter)
                        mx = float(np.max(np.abs(dq))) if dq.size else 0.0
                        if mx > 0.35:
                            dq *= 0.35 / mx
                        break
                    mu = 10.0 * mu if mu else float(np.max(self.k))
                else:
                    raise RuntimeError("FlexGen: Newton could not regularise "
                                       "the tangent stiffness — blade unstable "
                                       "under this load.")
                q = q + dq
            else:
                raise RuntimeError("FlexGen: no convergence at load fraction "
                                   f"{f:.2f} of ({fx_n:.1f} N, {fy_n:.1f} N, "
                                   f"{m_nmm:.1f} N·mm) — treat as unstable/"
                                   "post-buckled; do not use this blade here.")
        phis = self._full_phi(q)
        x, y, rot = self._tip(phis)
        g, H = self._grad_hess(q, fx_n, fy_n, m_nmm)
        eig_min = float(np.min(np.linalg.eigvalsh(H))) if self.ndof else 0.0
        if eig_min <= 0:
            # Converged onto a saddle of Π — an UNSTABLE (post-buckled)
            # configuration. Returning a state here would hand the caller a
            # geometry the real blade snaps away from; refusing is the truth.
            raise RuntimeError(
                "FlexGen: equilibrium under "
                f"({fx_n:.1f} N, {fy_n:.1f} N, {m_nmm:.1f} N·mm) is UNSTABLE "
                f"(min tangent-stiffness eigenvalue {eig_min:.1f} ≤ 0) — the "
                "blade has buckled; do not use it at this load.")
        J = self._jac_tip(phis)
        C = J @ np.linalg.solve(H, J.T)
        d = np.diff(np.concatenate([[0.0], phis]))
        return BladeState(
            fx_n=fx_n, fy_n=fy_n, m_nmm=m_nmm, phis=phis,
            tip_dx_mm=float(x - self.blade.length_mm), tip_dy_mm=float(y),
            tip_rot_rad=float(rot),
            strain_energy_nmm=self._spring_energy(phis),
            compliance=(C + C.T) / 2.0,
            curvature_per_mm=d / self.l,
            min_eig_h=eig_min,
        )

    def critical_axial_load_n(self) -> float:
        """Buckling load of the straight blade, from the discrete tangent
        Hessian: H(P) = K_spring − P·diag(len) on the free DOFs, so P_cr is
        the smallest generalized eigenvalue of (K_spring, diag(len)).
        Converges to Euler π²EI/(K·L)² with K = 2 (fixed-free) / K = 1
        (fixed-guided) as n_seg grows."""
        _, K = self._grad_hess(np.zeros(self.ndof), 0.0, 0.0, 0.0)
        d = self.lens[:self.ndof]
        Dinv = np.diag(1.0 / np.sqrt(d))
        return float(np.min(np.linalg.eigvalsh(Dinv @ K @ Dinv)))

    def solve_travel(self, dy_mm: float, fx_n: float = 0.0,
                     tol: float = 1e-6) -> BladeState:
        """Displacement control: find the transverse force that puts the tip
        at ``dy_mm`` (with axial preload ``fx_n``), by monotone bisection."""
        if abs(dy_mm) < 1e-12:
            return self.solve(fx_n, 0.0, 0.0)
        k0 = self.blade.k_compliant_linear()
        lo, hi = 0.0, math.copysign(max(1e-6, k0 * abs(dy_mm)), dy_mm)
        st = self.solve(fx_n, hi)
        # expand the bracket until we straddle the target
        for _ in range(60):
            if abs(st.tip_dy_mm) >= abs(dy_mm):
                break
            lo, hi = hi, hi * 1.6
            st = self.solve(fx_n, hi)
        else:
            raise RuntimeError("FlexGen: could not bracket the requested "
                               f"travel {dy_mm} mm — blade too soft/unstable.")
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            st = self.solve(fx_n, mid)
            if abs(st.tip_dy_mm - dy_mm) < tol:
                break
            if abs(st.tip_dy_mm) < abs(dy_mm):
                lo = mid
            else:
                hi = mid
        return st


# --------------------------------------------------------------------------- #
#  Strain energy → equivalent spring
# --------------------------------------------------------------------------- #
@dataclass
class EquivalentSpring:
    """The flexure's own contribution to wheel rate over the travel sweep."""
    z_mm: np.ndarray            # tip travel samples (symmetric about 0)
    f_n: np.ndarray             # transverse restoring force at each sample
    energy_nmm: np.ndarray      # stored strain energy at each sample
    k_n_mm: np.ndarray          # tangent rate dF/dz at each sample
    k_at_ride_n_mm: float       # rate at z = 0
    axial_preload_n: float

    @property
    def energy_at_full_j(self) -> float:
        return float(np.max(self.energy_nmm)) / 1000.0


def equivalent_spring(blade: FlexureBlade, travel_mm: float,
                      axial_preload_n: float = 0.0,
                      n_pts: int = 13) -> EquivalentSpring:
    """Sweep the blade tip through ±travel and read force, energy and tangent
    rate. n_pts is per side including 0; the sweep is symmetric because a
    corner sees both bump and droop."""
    chain = PRBChain(blade)
    z = np.linspace(-travel_mm, travel_mm, 2 * n_pts - 1)
    f = np.zeros_like(z)
    u = np.zeros_like(z)
    for i, zi in enumerate(z):
        st = chain.solve_travel(float(zi), fx_n=axial_preload_n)
        f[i] = st.fy_n
        u[i] = st.strain_energy_nmm
    k = np.gradient(f, z)
    i0 = n_pts - 1
    return EquivalentSpring(z_mm=z, f_n=f, energy_nmm=u, k_n_mm=k,
                            k_at_ride_n_mm=float(k[i0]),
                            axial_preload_n=axial_preload_n)


def coilover_downsize(k_wheel_target_n_mm: float,
                      k_flex_wheel_n_mm: float,
                      motion_ratio: float = 1.0) -> dict:
    """How much physical spring the coilover still needs once the blades are
    supplying ``k_flex_wheel_n_mm`` at the wheel.

    motion_ratio = wheel travel / spring travel; spring rate transforms by
    the square of it. Returns a dict with the residual spring rate and the
    fraction of the target the flexure already covers; ``feasible`` is False
    when the blades alone exceed the target (the corner would be over-sprung
    with NO coilover fitted — a design error the caller must hear about)."""
    if k_wheel_target_n_mm <= 0 or motion_ratio <= 0:
        raise ValueError("coilover_downsize: target rate and motion ratio > 0")
    resid_wheel = k_wheel_target_n_mm - k_flex_wheel_n_mm
    return {
        "k_flex_wheel_n_mm": k_flex_wheel_n_mm,
        "k_wheel_target_n_mm": k_wheel_target_n_mm,
        "flex_share": k_flex_wheel_n_mm / k_wheel_target_n_mm,
        "k_spring_residual_n_mm": max(0.0, resid_wheel) / motion_ratio ** 2,
        "feasible": resid_wheel >= 0.0,
    }


# --------------------------------------------------------------------------- #
#  Fatigue & buckling linter
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class BladeLoadCase:
    """One corner load state expressed at the blade tip.

    axial_n:  along the blade axis, + tension, − compression (the component
              of the cornering/braking load that tries to buckle the blade).
    shear_n:  transverse force in the compliant plane ON TOP of the travel-
              imposed bending (e.g. damper reaction routed through the blade).
    travel_mm: tip travel the case holds the blade at (max bump for the
              cornering-plus-bump worst case).
    """
    name: str
    axial_n: float = 0.0
    shear_n: float = 0.0
    travel_mm: float = 0.0


@dataclass(frozen=True)
class Finding:
    level: str        # "ok" | "warn" | "blocker"
    code: str
    detail: str


def _case_stress(blade: FlexureBlade, chain: PRBChain,
                 case: BladeLoadCase) -> tuple[float, float, float]:
    """(σ_bend_max, σ_axial, τ_max) MPa for one solved case."""
    st = chain.solve_travel(case.travel_mm, fx_n=case.axial_n) \
        if abs(case.travel_mm) > 1e-12 else chain.solve(case.axial_n,
                                                        case.shear_n)
    if abs(case.travel_mm) > 1e-12 and abs(case.shear_n) > 1e-12:
        # additional shear on top of the travel-imposed state
        st = chain.solve(case.axial_n, st.fy_n + case.shear_n, 0.0)
    E = blade.material.E
    s = chain.blade.spring_stations()
    sig_b = 0.0
    for kap, sf in zip(st.curvature_per_mm, s):
        sig_b = max(sig_b, E * (blade.section.t_at(sf) / 2.0) * abs(kap))
    A_min = min(blade.section.A_at(0.0), blade.section.A_at(1.0))
    sig_ax = case.axial_n / A_min
    v = abs(st.fy_n) + abs(case.shear_n)
    tau = 1.5 * v / A_min                      # rectangular-section peak shear
    return sig_b, sig_ax, tau


def flexgen_lint(blade: FlexureBlade, cases: list[BladeLoadCase],
                 fos_yield: float = 1.5, fos_fatigue: float = 1.5,
                 fos_buckling: float = 2.0,
                 min_stiff_ratio: float = 30.0) -> list[Finding]:
    """The fatigue & buckling margin linter.

    Screens every declared load case for (a) first-pass von Mises yield at
    ``fos_yield``, (b) high-cycle fatigue by the Goodman line — alternating
    stress from the ±travel bending, mean stress from the axial/static part —
    at ``fos_fatigue``, and (c) axial buckling of the blade at
    ``fos_buckling`` against the COMPUTED discrete critical load. Also flags
    a parasitic-stiffness ratio (stiff/compliant plane) below
    ``min_stiff_ratio`` — a blade that soft sideways is a bump-steer
    mechanism, not a bearing replacement.
    """
    out: list[Finding] = []
    fat = FATIGUE.get(blade.material.name)
    chain = PRBChain(blade)

    if fat is None:
        out.append(Finding("warn", "NO_FATIGUE_DATA",
                           f"No fatigue table entry for {blade.material.name!r}"
                           " — fatigue screen skipped; add coupon data."))
    elif fat.S_e is None:
        out.append(Finding("blocker", "LAMINATE_FATIGUE",
                           f"{blade.material.name}: no isotropic fatigue law "
                           "exists for a laminate. This lint refuses to "
                           "certify it — get coupon fatigue data for YOUR "
                           "layup, then screen against that."))

    ratio = blade.k_stiff_linear() / max(blade.k_compliant_linear(), 1e-12)
    if ratio < min_stiff_ratio:
        out.append(Finding("warn", "PARASITIC_SOFT",
                           f"Stiff/compliant plane stiffness ratio {ratio:.0f}"
                           f" < {min_stiff_ratio:.0f} — the blade will comply "
                           "in the direction it is supposed to locate."))

    p_cr = chain.critical_axial_load_n()
    for case in cases:
        # buckling: only compressive axial loads threaten it
        if case.axial_n < 0:
            margin = p_cr / abs(case.axial_n)
            if margin < 1.0:
                out.append(Finding("blocker", "BUCKLED",
                                   f"[{case.name}] axial {case.axial_n:.0f} N "
                                   f"exceeds the computed critical load "
                                   f"{p_cr:.0f} N — the blade buckles."))
                continue
            if margin < fos_buckling:
                # a declared FoS is a requirement, not a mood — failing it
                # blocks, exactly like yield and HCF do. A blade run just
                # under its buckling load also loses nearly all transverse
                # rate (axial softening), which is its own failure mode.
                out.append(Finding("blocker", "BUCKLING_MARGIN",
                                   f"[{case.name}] buckling margin "
                                   f"{margin:.2f} < required FoS "
                                   f"{fos_buckling:.1f} (P_cr = {p_cr:.0f} "
                                   "N) — and near P_cr the blade's "
                                   "transverse rate collapses too."))
            elif margin < 1.25 * fos_buckling:
                out.append(Finding("warn", "BUCKLING_NEAR",
                                   f"[{case.name}] buckling margin "
                                   f"{margin:.2f} within 25 % of the FoS "
                                   f"{fos_buckling:.1f} floor."))
        try:
            sig_b, sig_ax, tau = _case_stress(blade, chain, case)
        except RuntimeError as exc:
            out.append(Finding("blocker", "UNSTABLE",
                               f"[{case.name}] no stable equilibrium: {exc}"))
            continue
        sig_max = abs(sig_ax) + sig_b
        vm = math.sqrt(sig_max ** 2 + 3.0 * tau ** 2)
        if fat is not None:
            if vm * fos_yield > fat.S_y:
                out.append(Finding("blocker", "YIELD",
                                   f"[{case.name}] von Mises {vm:.0f} MPa × "
                                   f"FoS {fos_yield:.1f} exceeds S_y "
                                   f"{fat.S_y:.0f} MPa."))
            if fat.S_e is not None:
                # Goodman: bending from ±travel is the fully-reversed
                # alternating part; the axial/static part is the mean.
                sig_a, sig_m = sig_b, max(0.0, sig_ax)
                goodman = sig_a / fat.S_e + sig_m / fat.S_ut
                if goodman * fos_fatigue > 1.0:
                    out.append(Finding("blocker", "HCF",
                                       f"[{case.name}] Goodman utilisation "
                                       f"{goodman:.2f} × FoS {fos_fatigue:.1f}"
                                       f" > 1 (σ_a={sig_a:.0f}, "
                                       f"σ_m={sig_m:.0f} MPa) — high-cycle "
                                       "fatigue risk."))
                elif goodman * fos_fatigue > 0.8:
                    out.append(Finding("warn", "HCF_NEAR",
                                       f"[{case.name}] Goodman utilisation "
                                       f"{goodman:.2f} within 20 % of the "
                                       f"FoS {fos_fatigue:.1f} limit."))
    if fat is not None and fat.S_e is not None:
        out.append(Finding("ok", "SMOOTH_SPECIMEN",
                           "Fatigue screen uses smooth-specimen data with "
                           "surface/size factors = 1 — a machined edge, a "
                           "print layer line or a weld HAZ halves S_e. "
                           "Coupon-test the real edge finish."))
    if not any(f.level == "blocker" for f in out):
        out.append(Finding("ok", "PASS",
                           f"All {len(cases)} case(s) pass yield/HCF/buckling "
                           f"at FoS {fos_yield:.1f}/{fos_fatigue:.1f}/"
                           f"{fos_buckling:.1f}."))
    return out


# --------------------------------------------------------------------------- #
#  Synthesis — smallest blade that passes the lint
# --------------------------------------------------------------------------- #
@dataclass
class SynthesisResult:
    blade: Optional[FlexureBlade]
    mass_g: float
    findings: list[Finding]
    searched: int
    feasible: bool
    message: str = ""


def synthesize_blade(material_name: str, width_mm: float,
                     travel_mm: float, cases: list[BladeLoadCase],
                     length_range_mm: tuple[float, float] = (40.0, 120.0),
                     t_range_mm: tuple[float, float] = (0.8, 6.0),
                     boundary: str = "fixed-guided",
                     taper: float = 0.7,
                     n_length: int = 7, n_t: int = 12,
                     fos_yield: float = 1.5, fos_fatigue: float = 1.5,
                     fos_buckling: float = 2.0) -> SynthesisResult:
    """Deterministic grid synthesis: the lightest (L, t_root) blade of the
    given material/width whose lint comes back with no blockers, with every
    case's ``travel_mm`` overridden to the requested travel. ``taper`` is
    t_tip/t_root. Grid search on purpose — reproducible by hand, no seed."""
    if material_name not in MATERIALS:
        raise ValueError(f"Unknown material {material_name!r}")
    mat = MATERIALS[material_name]
    cases = [BladeLoadCase(c.name, c.axial_n, c.shear_n, travel_mm)
             for c in cases]
    best: Optional[FlexureBlade] = None
    best_mass = math.inf
    best_findings: list[Finding] = []
    tried = 0
    for L in np.linspace(*length_range_mm, n_length):
        for t in np.linspace(*t_range_mm, n_t):
            tried += 1
            blade = FlexureBlade(float(L),
                                 BladeSection(width_mm, float(t),
                                              float(t) * taper),
                                 mat, boundary=boundary)
            m = blade.mass_g()
            if m >= best_mass:
                continue
            try:
                f = flexgen_lint(blade, cases, fos_yield, fos_fatigue,
                                 fos_buckling)
            except RuntimeError:
                continue
            if any(x.level == "blocker" for x in f):
                continue
            best, best_mass, best_findings = blade, m, f
    if best is None:
        return SynthesisResult(None, math.nan, [], tried, False,
                               "No (L, t) in the search box passes the lint "
                               "for this material/width/travel/loads — widen "
                               "the blade, change material, or shorten travel.")
    return SynthesisResult(best, best_mass, best_findings, tried, True,
                           f"{best.name}: L={best.length_mm:.1f} mm, "
                           f"t={best.section.t_root_mm:.2f}"
                           f"→{best.section.t_at(1.0):.2f} mm, "
                           f"{best_mass:.1f} g")


# --------------------------------------------------------------------------- #
#  Geometry export — STL / STEP / layup orientation map
# --------------------------------------------------------------------------- #
def _blade_solid(blade: FlexureBlade):
    """(verts (8,3), quads) of the tapered blade blank.
    x along length, y = compliant/thickness direction, z = width."""
    L = blade.length_mm
    b = blade.section.width_mm
    tr = blade.section.t_root_mm
    tt = blade.section.t_at(1.0)
    v = np.array([
        [0, -tr / 2, -b / 2], [0,  tr / 2, -b / 2],
        [0,  tr / 2,  b / 2], [0, -tr / 2,  b / 2],
        [L, -tt / 2, -b / 2], [L,  tt / 2, -b / 2],
        [L,  tt / 2,  b / 2], [L, -tt / 2,  b / 2],
    ], dtype=float)
    # quads with outward-facing winding (right-hand rule)
    quads = [
        (0, 3, 2, 1),   # root face (−x)
        (4, 5, 6, 7),   # tip face (+x)
        (0, 1, 5, 4),   # −z side
        (3, 7, 6, 2),   # +z side
        (1, 2, 6, 5),   # +y face
        (0, 4, 7, 3),   # −y face
    ]
    return v, quads


def export_stl(blade: FlexureBlade) -> str:
    """ASCII STL of the blade blank — watertight, directly sliceable."""
    v, quads = _blade_solid(blade)
    out = io.StringIO()
    out.write(f"solid {blade.name}\n")
    for q in quads:
        for tri in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
            p0, p1, p2 = v[tri[0]], v[tri[1]], v[tri[2]]
            nrm = np.cross(p1 - p0, p2 - p0)
            nn = np.linalg.norm(nrm)
            nrm = nrm / nn if nn > 0 else nrm
            out.write(f"  facet normal {nrm[0]:.6e} {nrm[1]:.6e} {nrm[2]:.6e}\n")
            out.write("    outer loop\n")
            for p in (p0, p1, p2):
                out.write(f"      vertex {p[0]:.6e} {p[1]:.6e} {p[2]:.6e}\n")
            out.write("    endloop\n  endfacet\n")
    out.write(f"endsolid {blade.name}\n")
    return out.getvalue()


def export_step(blade: FlexureBlade) -> str:
    """Minimal AP214 STEP file of the blade blank: a faceted planar B-rep
    (MANIFOLD_SOLID_BREP over a CLOSED_SHELL of six planar ADVANCED_FACEs),
    with mm length units declared. Opens as a solid in FreeCAD/SolidWorks."""
    v, quads = _blade_solid(blade)
    ents: list[str] = []

    def add(txt: str) -> int:
        ents.append(txt)
        return len(ents)          # 1-based entity id

    # --- product / context boilerplate ------------------------------------ #
    app = add("APPLICATION_CONTEXT('automotive design')")
    add("APPLICATION_PROTOCOL_DEFINITION('international standard',"
        f"'automotive_design',2010,#{app})")
    pctx = add(f"PRODUCT_CONTEXT('',#{app},'mechanical')")
    prod = add(f"PRODUCT('{blade.name}','{blade.name}','',(#{pctx}))")
    pdf = add(f"PRODUCT_DEFINITION_FORMATION('','',#{prod})")
    pdc = add(f"PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')")
    pd = add(f"PRODUCT_DEFINITION('design','',#{pdf},#{pdc})")
    pds = add(f"PRODUCT_DEFINITION_SHAPE('','',#{pd})")
    lu = add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
    au = add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    su = add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    unc = add(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-3),#{lu},"
              "'distance_accuracy_value','')")
    ctx = add(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)"
              f"GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc}))"
              f"GLOBAL_UNIT_ASSIGNED_CONTEXT((#{lu},#{au},#{su}))"
              "REPRESENTATION_CONTEXT('','3D'))")

    def pt(p) -> int:
        return add(f"CARTESIAN_POINT('',({p[0]:.6f},{p[1]:.6f},{p[2]:.6f}))")

    def dirn(d) -> int:
        return add(f"DIRECTION('',({d[0]:.9f},{d[1]:.9f},{d[2]:.9f}))")

    cps = [pt(p) for p in v]
    vps = [add(f"VERTEX_POINT('',#{c})") for c in cps]

    # unique edges (unordered vertex pairs) shared between faces
    edge_of: dict[tuple[int, int], int] = {}

    def edge(a: int, bb: int) -> tuple[int, bool]:
        key = (min(a, bb), max(a, bb))
        if key not in edge_of:
            i, j = key
            d = v[j] - v[i]
            d = d / np.linalg.norm(d)
            vec = add(f"VECTOR('',#{dirn(d)},1.0)")
            line = add(f"LINE('',#{cps[i]},#{vec})")
            edge_of[key] = add(f"EDGE_CURVE('',#{vps[i]},#{vps[j]},#{line},.T.)")
        return edge_of[key], (a, bb) == (min(a, bb), max(a, bb))

    faces = []
    for q in quads:
        oes = []
        for k in range(4):
            a, bb = q[k], q[(k + 1) % 4]
            ec, fwd = edge(a, bb)
            oes.append(add(f"ORIENTED_EDGE('',*,*,#{ec},"
                           f"{'.T.' if fwd else '.F.'})"))
        loop = add("EDGE_LOOP('',(" + ",".join(f"#{e}" for e in oes) + "))")
        bound = add(f"FACE_OUTER_BOUND('',#{loop},.T.)")
        p0, p1, p2 = v[q[0]], v[q[1]], v[q[2]]
        nrm = np.cross(p1 - p0, p2 - p0)
        nrm = nrm / np.linalg.norm(nrm)
        ref = p1 - p0
        ref = ref / np.linalg.norm(ref)
        ax = add(f"AXIS2_PLACEMENT_3D('',#{cps[q[0]]},#{dirn(nrm)},#{dirn(ref)})")
        plane = add(f"PLANE('',#{ax})")
        faces.append(add(f"ADVANCED_FACE('',(#{bound}),#{plane},.T.)"))

    shell = add("CLOSED_SHELL('',(" + ",".join(f"#{f}" for f in faces) + "))")
    brep = add(f"MANIFOLD_SOLID_BREP('{blade.name}',#{shell})")
    origin = add("CARTESIAN_POINT('',(0.,0.,0.))")
    zd = add("DIRECTION('',(0.,0.,1.))")
    xd = add("DIRECTION('',(1.,0.,0.))")
    wax = add(f"AXIS2_PLACEMENT_3D('',#{origin},#{zd},#{xd})")
    rep = add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{wax},#{brep}),#{ctx})")
    add(f"SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})")

    body = "\n".join(f"#{i + 1}={t};" for i, t in enumerate(ents))
    today = date.today().isoformat()
    return (
        "ISO-10303-21;\nHEADER;\n"
        "FILE_DESCRIPTION(('KinematiK FlexGen blade blank'),'2;1');\n"
        f"FILE_NAME('{blade.name}.step','{today}',('KinematiK'),"
        "('KinematiK FlexGen'),'flexgen.py','','');\n"
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n"
        "ENDSEC;\nDATA;\n" + body + "\nENDSEC;\nEND-ISO-10303-21;\n"
    )


def layup_map(blade: FlexureBlade, n_stations: int = 11,
              process: str = "auto") -> str:
    """Fibre/build ORIENTATION map along the blade, as CSV.

    process:
      * "carbon_layup" — per-station 0°-ply axis (unit vector along the blade
        centreline) plus a suggested ±45 fraction that grows toward the root
        where interlaminar shear peaks.
      * "additive_ti"  — recommended build orientation (layer normal in the
        blade's WIDTH direction so layer lines never run across the bending
        surface) and per-station contour count from local thickness.
      * "auto" — carbon material -> layup, otherwise additive.

    This is an ORIENTATION map, not a certified schedule — the header row of
    the CSV says so, on purpose.
    """
    if process == "auto":
        process = ("carbon_layup" if "Carbon" in blade.material.name
                   else "additive_ti")
    rows = ["# KinematiK FlexGen orientation map — guidance, not a certified "
            "laminate schedule / build file",
            f"# blade={blade.name} L={blade.length_mm:.1f}mm "
            f"b={blade.section.width_mm:.1f}mm material={blade.material.name} "
            f"process={process}"]
    if process == "carbon_layup":
        rows.append("s_mm,t_mm,axis_x,axis_y,axis_z,zero_deg_frac,pm45_frac")
        for i in range(n_stations):
            sf = i / (n_stations - 1)
            pm45 = 0.4 - 0.25 * sf         # more ±45 at the root
            rows.append(f"{sf * blade.length_mm:.2f},"
                        f"{blade.section.t_at(sf):.3f},1,0,0,"
                        f"{1 - pm45:.2f},{pm45:.2f}")
    elif process == "additive_ti":
        rows.append("s_mm,t_mm,build_normal_x,build_normal_y,build_normal_z,"
                    "contours")
        for i in range(n_stations):
            sf = i / (n_stations - 1)
            t = blade.section.t_at(sf)
            rows.append(f"{sf * blade.length_mm:.2f},{t:.3f},0,0,1,"
                        f"{max(2, int(round(t / 0.4)))}")
    else:
        raise ValueError(f"layup_map: unknown process {process!r}")
    return "\n".join(rows) + "\n"


# --------------------------------------------------------------------------- #
#  Report
# --------------------------------------------------------------------------- #
def render_flexgen_md(blade: FlexureBlade, spring: EquivalentSpring,
                      findings: list[Finding],
                      downsize: Optional[dict] = None) -> str:
    """One-page markdown summary for the design review / handover."""
    p_cr = PRBChain(blade).critical_axial_load_n()
    lines = [
        f"# FlexGen blade — {blade.name}",
        "",
        f"* Material: **{blade.material.name}**, boundary **{blade.boundary}**",
        f"* L = {blade.length_mm:.1f} mm, b = {blade.section.width_mm:.1f} mm, "
        f"t = {blade.section.t_root_mm:.2f} → {blade.section.t_at(1.0):.2f} mm,"
        f" mass {blade.mass_g():.1f} g",
        f"* Rate at ride: **{spring.k_at_ride_n_mm:.2f} N/mm** at the tip "
        f"(axial preload {spring.axial_preload_n:.0f} N); strain energy at "
        f"full travel {spring.energy_at_full_j * 1000:.0f} N·mm",
        f"* Computed axial buckling load: **{p_cr:.0f} N** "
        f"(discrete tangent-stiffness eigenvalue, not a chart)",
        f"* Parasitic stiffness ratio (stiff/compliant plane): "
        f"{blade.k_stiff_linear() / blade.k_compliant_linear():.0f}",
        "",
        "## Lint",
    ]
    for f in findings:
        badge = {"ok": "✅", "warn": "⚠️", "blocker": "⛔"}[f.level]
        lines.append(f"* {badge} `{f.code}` — {f.detail}")
    if downsize is not None:
        lines += [
            "",
            "## Coilover downsizing",
            f"* Flexure supplies {downsize['flex_share'] * 100:.0f} % of the "
            f"{downsize['k_wheel_target_n_mm']:.1f} N/mm wheel-rate target.",
            (f"* Residual physical spring: "
             f"**{downsize['k_spring_residual_n_mm']:.1f} N/mm** at the spring."
             if downsize["feasible"] else
             "* ⛔ The blades ALONE exceed the wheel-rate target — the corner "
             "is over-sprung with no coilover fitted. Soften the blade."),
        ]
    lines += [
        "",
        "_Pre-validation output. Validate the blade in ANSYS (non-linear, "
        "large deflection) and fatigue-test a coupon with the production edge "
        "finish before cutting metal._",
    ]
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- #
#  Self-test
# --------------------------------------------------------------------------- #
def _selftest():                                       # pragma: no cover
    mat = MATERIALS["Steel 4130"]
    blade = FlexureBlade(80.0, BladeSection(30.0, 2.0), mat,
                         boundary="fixed-free", n_seg=24)
    chain = PRBChain(blade)
    E, L = mat.E, blade.length_mm
    I = blade.section.I_at(0.0)
    # small-load limit vs Euler-Bernoulli
    F = 2.0
    st = chain.solve(0.0, F)
    ref = F * L ** 3 / (3 * E * I)
    assert abs(st.tip_dy_mm - ref) / ref < 0.01, (st.tip_dy_mm, ref)
    rot_ref = F * L ** 2 / (2 * E * I)
    assert abs(st.tip_rot_rad - rot_ref) / rot_ref < 0.01
    # compliance matrix symmetric, dy/dFy matches
    assert abs(st.compliance[1, 1] - ref / F) / (ref / F) < 0.02
    # buckling vs Euler fixed-free
    pe = math.pi ** 2 * E * I / (4 * L ** 2)
    pc = chain.critical_axial_load_n()
    assert abs(pc - pe) / pe < 0.02, (pc, pe)
    # energy consistency on a sweep
    sp = equivalent_spring(blade, 8.0, n_pts=9)
    zpos = sp.z_mm >= 0
    _trap = getattr(np, "trapezoid", None) or np.trapz   # numpy 1/2 compat
    w = _trap(sp.f_n[zpos], sp.z_mm[zpos])
    u = sp.energy_nmm[-1]
    assert abs(w - u) / u < 0.02, (w, u)
    # exports parse-shaped
    stl = export_stl(blade)
    assert stl.count("facet normal") == 12
    step = export_step(blade)
    assert step.startswith("ISO-10303-21;") and "MANIFOLD_SOLID_BREP" in step
    print("flexgen self-test OK — "
          f"tip {st.tip_dy_mm:.3f} mm vs {ref:.3f} mm analytic, "
          f"P_cr {pc:.0f} N vs Euler {pe:.0f} N")


if __name__ == "__main__":                             # pragma: no cover
    _selftest()
