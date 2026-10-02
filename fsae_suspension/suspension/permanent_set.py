# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Permanent set: how far a bracket or tube stays bent after an overload.

A kerb strike that pushes a pickup bracket or a clevis tube past yield leaves
it bent when the load goes, and the pickup has moved. Whether that happens,
and by how much, follows from elastic-plastic bending: this module computes
it with a fiber model of the section, so the answer covers rectangles
(brackets, tabs) and tubes alike, and a weld's residual stress can be
included, which is how a member that never saw a nominal overload still
shakes down and moves on its first high-g cycles.

METHOD
------
The section is cut into fibers at heights y with areas A. For a strain field
eps = eps0 + kappa * y (plane sections), each fiber's stress is its residual
stress plus E * (eps - eps_p), clipped to +/- yield (elastic-perfectly
plastic). Loading: at each station along the member the curvature and axial
strain that carry the applied moment with zero axial force are found, and
each fiber's plastic strain is recorded. Unloading is elastic: the
curvature at zero moment and zero force follows from a 2 x 2 linear solve.
The permanent curvature, integrated along a cantilever of length L under a
tip load, gives the permanent tip deflection.

Residual stress from welding is DECLARED (a profile through the depth,
self-equilibrated here by removing its force and moment resultants); a
measured profile (hole drilling) makes the shakedown prediction real.
Strain hardening is neglected, which over-predicts set slightly; stress
relaxation by creep is negligible in steel and aluminium at chassis
temperature and is not modelled. Units: mm, N, MPa, N*mm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np


@dataclass
class Section:
    """A bending section as fibers. ``kind`` "rect" (b x h) or "tube" (od, t)."""
    kind: str
    b_mm: float = 0.0
    h_mm: float = 0.0
    od_mm: float = 0.0
    t_mm: float = 0.0
    n: int = 200

    def fibers(self) -> tuple[np.ndarray, np.ndarray]:
        """(y mm, area mm^2) of each fiber, bending about the horizontal axis."""
        if self.kind == "rect":
            y = (np.arange(self.n) + 0.5) / self.n * self.h_mm - self.h_mm / 2.0
            return y, np.full(self.n, self.b_mm * self.h_mm / self.n)
        if self.kind == "tube":
            ro, ri = self.od_mm / 2.0, self.od_mm / 2.0 - self.t_mm
            y = (np.arange(self.n) + 0.5) / self.n * 2 * ro - ro
            dy = 2 * ro / self.n
            wo = 2 * np.sqrt(np.clip(ro * ro - y * y, 0.0, None))
            wi = 2 * np.sqrt(np.clip(ri * ri - y * y, 0.0, None))
            return y, (wo - wi) * dy
        raise ValueError("Section.kind must be 'rect' or 'tube'.")

    def I_mm4(self) -> float:
        y, A = self.fibers()
        return float(np.sum(A * y * y))


def _residual(sec: Section, profile: Callable | None) -> np.ndarray:
    y, A = sec.fibers()
    if profile is None:
        return np.zeros_like(y)
    s = np.asarray([float(profile(v)) for v in y])
    # self-equilibrate: remove net force and moment
    s = s - np.sum(s * A) / np.sum(A)
    s = s - y * np.sum(s * A * y) / np.sum(A * y * y)
    return s


def _stress(y, sr, E, sy, eps0, kappa, epp):
    trial = sr + E * (eps0 + kappa * y - epp)
    return np.clip(trial, -sy, sy), trial


def _load(sec, E, sy, M, sr, epp):
    """Curvature and axial strain carrying M (N*mm) at zero axial force."""
    y, A = sec.fibers()

    def forces(e0, k):
        """Section axial force in N and moment in N·mm."""
        s, _ = _stress(y, sr, E, sy, e0, k, epp)
        return float(np.sum(s * A)), float(np.sum(s * A * y))

    k_el = M / (E * sec.I_mm4())
    lo, hi = 0.0, max(abs(k_el) * 2.0, 1e-9)
    sgn = 1.0 if M >= 0 else -1.0

    def e0_for(k):
        """Reference-fibre strain (dimensionless) for a curvature in 1/mm."""
        a, b = -1.0, 1.0
        for _ in range(60):
            m = 0.5 * (a + b)
            (a, b) = (m, b) if forces(m, k)[0] < 0 else (a, m)
        return 0.5 * (a + b)

    # grow the bracket until the moment is reached, or the section collapses
    for _ in range(60):
        if abs(forces(e0_for(sgn * hi), sgn * hi)[1]) >= abs(M):
            break
        hi *= 2.0
        if hi > 1.0:                                   # 1/mm: plastic hinge
            return None
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        (lo, hi) = (mid, hi) if abs(forces(e0_for(sgn * mid), sgn * mid)[1]) < abs(M) else (lo, mid)
    k = sgn * 0.5 * (lo + hi)
    return e0_for(k), k


def permanent_curvature(sec: Section, E: float, sy: float, M: float,
                        residual: Callable | None = None) -> dict:
    """Permanent curvature (1/mm) left after moment M is applied and removed."""
    y, A = sec.fibers()
    sr = _residual(sec, residual)
    epp = np.zeros_like(y)
    sol = _load(sec, E, sy, M, sr, epp)
    if sol is None:
        return {"collapsed": True, "kappa_perm": float("inf"), "yielded": True}
    e0, k = sol
    s, trial = _stress(y, sr, E, sy, e0, k, epp)
    epp = (trial - s) / E                              # plastic strain taken
    # elastic unload to zero force and moment: solve for (e0', k')
    Mat = E * np.array([[np.sum(A), np.sum(A * y)], [np.sum(A * y), np.sum(A * y * y)]])
    rhs = -np.array([np.sum((sr - E * epp) * A), np.sum((sr - E * epp) * A * y)])
    e0u, ku = np.linalg.solve(Mat, rhs)
    return {"collapsed": False, "kappa_perm": float(ku),
            "yielded": bool(np.any(np.abs(epp) > 1e-12))}


def cantilever_set(sec: Section, E: float, sy: float, length_mm: float,
                   tip_load_n: float, residual: Callable | None = None,
                   n_seg: int = 40) -> dict:
    """Permanent tip deflection (mm) of a cantilever after a tip load.

    Moment F (L - x) along the member; the permanent curvature at each station
    integrates to the tip as the integral of kappa_perm (L - x) dx.
    """
    x = (np.arange(n_seg) + 0.5) / n_seg * length_mm
    dx = length_mm / n_seg
    tip, yielded = 0.0, False
    for xi in x:
        r = permanent_curvature(sec, E, sy, tip_load_n * (length_mm - xi), residual)
        if r["collapsed"]:
            return {"collapsed": True, "permanent_tip_mm": float("inf"), "yielded": True}
        yielded |= r["yielded"]
        tip += r["kappa_perm"] * (length_mm - xi) * dx
    return {"collapsed": False, "permanent_tip_mm": float(tip), "yielded": yielded}


def channel_drift(hp, pickup: str, offset_mm) -> dict:
    """Camber, toe, caster, kpi change (deg) from a pickup moved by offset_mm."""
    from . import elastokinematics as _ek
    J = _ek.compliance_jacobian(hp, points=(pickup,))
    d = J @ np.asarray(offset_mm, float).reshape(3)
    return {ch: float(v) for ch, v in zip(_ek.CHANNELS, d)}
