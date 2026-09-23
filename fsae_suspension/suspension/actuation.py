# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Actuation synthesis: design the pushrod, rocker and spring, not just read them.

The kinematic synthesis places the wishbones and the toe link. The spring and
damper are driven through a pushrod and a rocker (bell-crank), and until now
those six points were whatever the defaults were. On the default linkage the
motion ratio falls from about 1.0 in droop to 0.5 in bump, so the wheel rate
varies by nearly 300% across travel and every ride frequency is valid only at
static height. This module designs the linkage instead.

The actuation points do not enter camber, toe or roll centre (the pushrod rides
on the lower wishbone and only reads its motion), so actuation is a clean second
stage on the corner the kinematic synthesis returns: it cannot spoil a
kinematic result.

Targets are derived, not declared:

* Static motion ratio: as high as the damper stroke allows, because a higher
  ratio gives the damper more travel and velocity per millimetre of wheel travel
  and so more usable damping resolution. With usable stroke fraction u, stroke S
  and total wheel travel T, the ceiling is MR_max = u S / T; the target sits a
  margin below it.
* Progression: the spread of motion ratio across travel is bounded, so the
  wheel rate, which goes with MR squared, stays near constant and the ride
  frequency becomes a curve with a band rather than a single number.
* Packaging and load path: the pushrod and spring must act close to the rocker
  plane (out-of-plane angle bounded), or the rocker bearings and rod ends carry
  bending; and the transmission angle between each link and its rocker arm stays
  away from toggle.

Units: mm, deg, N/mm, Hz; motion ratio is spring travel over wheel travel.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Sequence

import numpy as np
from scipy.optimize import least_squares

from .kinematics import Hardpoints, SuspensionKinematics

ACTUATION_POINTS = ("pushrod_outer", "rocker_pivot", "rocker_pushrod",
                    "rocker_spring", "spring_inner")


@dataclass
class ActuationTargets:
    """What the linkage must do. Lengths mm, angles deg."""
    mr_static: float = 0.85
    mr_tol: float = 0.05
    max_spread: float = 0.10          # (max - min) / static, over travel
    max_out_of_plane_deg: float = 5.0
    transmission_deg: tuple = (40.0, 140.0)
    stroke_mm: float = 57.0
    usable_stroke: float = 0.80
    travel_mm: tuple = (-25.0, 25.0)

    @classmethod
    def derived(cls, stroke_mm: float = 57.0, travel_mm=(-25.0, 25.0),
                usable_stroke: float = 0.80, margin: float = 0.07,
                max_spread: float = 0.10) -> "ActuationTargets":
        """Static ratio set ``margin`` (fraction) below the stroke ceiling.

        stroke_mm and travel_mm in mm; usable_stroke, margin and max_spread are
        dimensionless fractions; the motion ratio is dimensionless.
        """
        total = float(travel_mm[1] - travel_mm[0])
        mr_max = usable_stroke * stroke_mm / total
        return cls(mr_static=round(mr_max * (1.0 - margin), 3), mr_tol=0.05,
                   max_spread=max_spread, stroke_mm=stroke_mm,
                   usable_stroke=usable_stroke, travel_mm=tuple(travel_mm))

    @property
    def mr_ceiling(self) -> float:
        """Highest usable motion ratio (dimensionless): usable stroke (mm) over travel (mm)."""
        return self.usable_stroke * self.stroke_mm / float(self.travel_mm[1] - self.travel_mm[0])


def _angle_deg(a, b) -> float:
    a = np.asarray(a, float); b = np.asarray(b, float)
    c = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _seg_dist(p0, p1, q0, q1) -> float:
    """Minimum distance between segments p0-p1 and q0-q1 (mm)."""
    p0, p1, q0, q1 = (np.asarray(v, float) for v in (p0, p1, q0, q1))
    u, v, w = p1 - p0, q1 - q0, p0 - q0
    a, b, c, d, e = u @ u, u @ v, v @ v, u @ w, v @ w
    den = a * c - b * b
    sN, sD = (b * e - c * d, den) if den > 1e-12 else (0.0, 1.0)
    tN, tD = (a * e - b * d, den) if den > 1e-12 else (e, c)
    sc = 0.0 if sD == 0 else min(1.0, max(0.0, sN / sD))
    tc = 0.0 if tD == 0 else min(1.0, max(0.0, (b * sc + e) / tD if tD else 0.0))
    return float(np.linalg.norm(w + sc * u - tc * v))


@dataclass
class ActuationMetrics:
    """What a linkage does over travel. Lengths mm, angles deg."""
    ok: bool
    travel: list = field(default_factory=list)
    mr: list = field(default_factory=list)
    mr_static: float = math.nan
    spread: float = math.nan            # (max-min)/static
    slope_per_mm: float = math.nan      # d(MR)/d(travel)
    stroke_used_mm: float = math.nan
    out_of_plane_deg: float = math.nan  # worst of pushrod and spring
    transmission_deg: tuple = (math.nan, math.nan)
    clearance_mm: float = math.inf      # to declared obstacles
    reason: str = ""


def actuation_metrics(hp: Hardpoints, travel_mm=(-25.0, 25.0), n: int = 11,
                      obstacles: Sequence = ()) -> ActuationMetrics:
    """Solve the real pushrod-rocker-spring chain across travel and measure it.

    ``obstacles`` is a list of (p0, p1, radius_mm) capsules, for example frame
    tubes, that the pushrod and spring must clear.
    """
    try:
        kin = SuspensionKinematics(hp)
        if not kin.motion_ratio_is_real():
            return ActuationMetrics(ok=False, reason="actuation points incomplete")
        trv, mr = kin.motion_ratio_curve(travel_mm[0], travel_mm[1], n)
        trv = np.asarray(trv, float); mr = np.asarray(mr, float)
        if not np.all(np.isfinite(mr)):
            return ActuationMetrics(ok=False, reason="linkage does not solve over travel")
        i0 = int(np.argmin(np.abs(trv)))
        m0 = float(mr[i0])
        # walk outward from static in small steps, seeding each rocker solve
        # from its neighbour, so the rocker stays on one branch
        def walk(end):
            L, th, ok = kin.spring_length_at(kin.solve_at_travel(0.0), seed=0.0)
            if not ok:
                return None
            for z in np.linspace(0.0, end, 11)[1:]:
                L, th, ok = kin.spring_length_at(kin.solve_at_travel(float(z)), seed=th)
                if not ok:
                    return None
            return L, th
        lo_w, hi_w = walk(travel_mm[0]), walk(travel_mm[1])
        L0, th0, ok0 = kin.spring_length_at(kin.solve_at_travel(0.0), seed=0.0)
        if lo_w is None or hi_w is None or not ok0:
            return ActuationMetrics(ok=False, reason="spring length unsolved")
        lengths = [lo_w[0], L0, hi_w[0]]
        thetas = [lo_w[1], th0, hi_w[1]]
        k = np.asarray(kin._rocker_axis, float)
        s0 = kin.solve_at_travel(0.0)
        pro = np.asarray(s0.pushrod_outer if s0.pushrod_outer is not None
                         else hp.pushrod_outer, float)
        piv = np.asarray(hp.rocker_pivot, float)
        rp = np.asarray(hp.rocker_pushrod, float); rs = np.asarray(hp.rocker_spring, float)
        si = np.asarray(hp.spring_inner, float)
        oop = max(math.degrees(math.asin(min(1.0, abs(float(k @ (v / np.linalg.norm(v)))))))
                  for v in (rp - pro, si - rs))
        tr = []
        for th, t in zip(thetas, (travel_mm[0], 0.0, travel_mm[1])):
            st = kin.solve_at_travel(float(t))
            p_out = np.asarray(st.pushrod_outer if st.pushrod_outer is not None
                               else kin._pushrod_outer_at(st.lower_outer, st.upper_outer,
                                                          st.tie_rod_outer), float)
            rp_t = kin._rotate_about_axis(rp, th); rs_t = kin._rotate_about_axis(rs, th)
            tr.append(_angle_deg(rp_t - piv, p_out - rp_t))
            tr.append(_angle_deg(rs_t - piv, si - rs_t))
        clear = math.inf
        for (a, b, r) in obstacles:
            for seg in ((pro, rp), (rs, si)):
                clear = min(clear, _seg_dist(seg[0], seg[1], a, b) - float(r))
        return ActuationMetrics(
            ok=True, travel=trv.tolist(), mr=mr.tolist(), mr_static=m0,
            spread=float((mr.max() - mr.min()) / m0),
            slope_per_mm=float(np.polyfit(trv, mr, 1)[0]),
            stroke_used_mm=float(abs(lengths[2] - lengths[0])),
            out_of_plane_deg=float(oop),
            transmission_deg=(float(min(tr)), float(max(tr))), clearance_mm=clear)
    except Exception as e:                                   # noqa: BLE001
        return ActuationMetrics(ok=False, reason=f"{type(e).__name__}: {e}")


def _residual(m: ActuationMetrics, t: ActuationTargets, shrink: float = 1.0) -> np.ndarray:
    """Hinge residuals, zero inside every band; each in units of its tolerance.

    ``shrink`` (dimensionless, <= 1) narrows the bands; the search aims inside
    them so results land strictly within the true band rather than on its edge.
    """
    if not m.ok:
        return np.full(7, 50.0)
    lo, hi = t.transmission_deg
    usable = t.usable_stroke * t.stroke_mm
    return np.array([
        max(0.0, abs(m.mr_static - t.mr_static) - shrink * t.mr_tol) / t.mr_tol,
        max(0.0, m.spread - shrink * t.max_spread) / t.max_spread,
        max(0.0, m.out_of_plane_deg - t.max_out_of_plane_deg) / 5.0,
        max(0.0, lo - m.transmission_deg[0]) / 10.0,
        max(0.0, m.transmission_deg[1] - hi) / 10.0,
        max(0.0, m.stroke_used_mm - usable) / 5.0,
        max(0.0, -m.clearance_mm) / 5.0,
    ])


@dataclass
class ActuationResult:
    ok: bool
    hp: Hardpoints
    metrics: ActuationMetrics
    residual: list
    seed_metrics: ActuationMetrics
    spring_rate_N_per_mm: float = math.nan
    ride_frequency_hz: list = field(default_factory=list)   # over travel
    reason: str = ""


def _set(hp: Hardpoints, x: np.ndarray, pts) -> Hardpoints:
    h = replace(hp) if hasattr(hp, "__dataclass_fields__") else hp
    import copy
    h = copy.deepcopy(hp)
    for i, p in enumerate(pts):
        setattr(h, p, np.asarray(x[3 * i:3 * i + 3], float))
    return h


def synthesize_actuation(hp: Hardpoints, targets: ActuationTargets | None = None,
                         box_mm: float = 40.0, pushrod_box_mm: float = 20.0,
                         points: Sequence[str] = ACTUATION_POINTS,
                         obstacles: Sequence = (), n_starts: int = 4,
                         seed: int = 0, ride_hz: float | None = None,
                         sprung_corner_kg: float | None = None) -> ActuationResult:
    """Place the actuation points so the linkage meets ``targets``.

    Each point moves inside a box around its seed (``pushrod_box_mm`` for the
    pushrod outer, which sits on the lower wishbone). Bounded least squares on
    hinge residuals, from the seed and ``n_starts - 1`` random starts; the best
    is returned. With ``ride_hz`` and ``sprung_corner_kg`` the spring rate that
    gives that frequency at static height is returned, with the frequency
    across travel.
    """
    t = targets or ActuationTargets.derived()
    pts = list(points)
    x0 = np.concatenate([np.asarray(getattr(hp, p), float) for p in pts])
    half = np.concatenate([np.full(3, pushrod_box_mm if p == "pushrod_outer" else box_mm)
                           for p in pts])
    lo, hi = x0 - half, x0 + half
    seed_m = actuation_metrics(hp, t.travel_mm, obstacles=obstacles)

    def fun(x):
        m = actuation_metrics(_set(hp, x, pts), t.travel_mm, obstacles=obstacles)
        reg = 0.002 * (x - x0) / half          # stay near the seed when free
        return np.concatenate([_residual(m, t, shrink=0.98), reg])

    rng = np.random.default_rng(seed)
    starts = [x0] + [lo + rng.random(x0.size) * (hi - lo) for _ in range(max(0, n_starts - 1))]
    best = None
    for s in starts:
        try:
            sol = least_squares(fun, s, bounds=(lo, hi), diff_step=1e-3,
                                max_nfev=120, xtol=1e-6, ftol=1e-8)
        except Exception:                                    # noqa: BLE001
            continue
        r = _residual(actuation_metrics(_set(hp, sol.x, pts), t.travel_mm,
                                        obstacles=obstacles), t)
        score = float(np.sum(r ** 2))
        if best is None or score < best[0]:
            best = (score, sol.x, r)
        if score <= 1e-12:
            break
    if best is None:
        return ActuationResult(ok=False, hp=hp, metrics=seed_m, residual=[],
                               seed_metrics=seed_m, reason="no start solved")
    h = _set(hp, best[1], pts)
    m = actuation_metrics(h, t.travel_mm, obstacles=obstacles)
    k_s, fz = math.nan, []
    if ride_hz and sprung_corner_kg and m.ok:
        k_w = (2 * math.pi * ride_hz) ** 2 * sprung_corner_kg / 1000.0   # N/mm
        k_s = k_w / m.mr_static ** 2
        fz = [ride_hz * v / m.mr_static for v in m.mr]
    return ActuationResult(ok=bool(np.all(best[2] <= 1e-6)), hp=h, metrics=m,
                           residual=best[2].tolist(), seed_metrics=seed_m,
                           spring_rate_N_per_mm=k_s, ride_frequency_hz=fz)
