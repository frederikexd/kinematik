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
from dataclasses import dataclass, field
from functools import partial
from collections.abc import Sequence

import numpy as np
from ._num import vec as _vec
from scipy.optimize import least_squares

from .kinematics import Hardpoints, SuspensionKinematics

_ROWDOT = "ij,ij->i"          # row-wise dot product
_NO_RADIUS = np.zeros(1)
_N_NODE_CANDIDATES = 6
_AXIS_BOX = 0.5
_float_array = partial(np.asarray, dtype=float)

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
    min_clearance_mm: float = 5.0     # swept, to every declared obstacle
    node_reach_mm: float = 30.0       # chassis mounts to their nearest node
    max_arm_offset_mm: float = 5.0    # rocker force points off the bearing mid-plane
    #: the damper's eye-to-eye length at full extension (mm); None leaves the damper's length free (as before). When
    #: given, the damper must not top out before full droop (length at the droop end <= this) and must stay inside its
    #: usable stroke at full bump (length at the bump end >= this - usable_stroke * stroke_mm)
    damper_extended_mm: float | None = None

    @classmethod
    def derived(cls, stroke_mm: float = 57.0, travel_mm=(-25.0, 25.0),
                usable_stroke: float = 0.80, margin: float = 0.07,
                max_spread: float = 0.10, damper_extended_mm: float | None = None) -> ActuationTargets:
        """Static ratio set ``margin`` (fraction) below the stroke ceiling.

        stroke_mm, travel_mm and damper_extended_mm in mm; usable_stroke, margin and
        max_spread are dimensionless fractions; the motion ratio is dimensionless.
        """
        total = float(travel_mm[1] - travel_mm[0])
        mr_max = usable_stroke * stroke_mm / total
        return cls(mr_static=round(mr_max * (1.0 - margin), 3),
                   max_spread=max_spread, stroke_mm=stroke_mm,
                   usable_stroke=usable_stroke, travel_mm=tuple(travel_mm),
                   damper_extended_mm=damper_extended_mm)

    @property
    def mr_ceiling(self) -> float:
        """Highest usable motion ratio (dimensionless): usable stroke (mm) over travel (mm)."""
        return self.usable_stroke * self.stroke_mm / float(self.travel_mm[1] - self.travel_mm[0])


def _angle_deg(a, b) -> float:
    a = _vec(a); b = _vec(b)
    c = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))  # pragma: no mutate (rounding guard)


def _seg_dist(p0, p1, q0, q1) -> float:
    """Minimum distance between segments p0-p1 and q0-q1 (mm)."""
    return _seg_dist_many(p0, p1, _vec(q0)[None], _vec(q1)[None], _NO_RADIUS)

def _seg_dist_many(p0, p1, A, B, R) -> float:
    """Minimum surface clearance (mm) from segment p0-p1 to capsules A-B of radius R."""
    p0 = _vec(p0); p1 = _vec(p1)
    u = p1 - p0; v = B - A; w = p0 - A
    a = u @ u; b = v @ u; c = np.einsum(_ROWDOT, v, v)
    d = w @ u; e = np.einsum(_ROWDOT, v, w)
    den = a * c - b * b
    sc = np.where(den > 1e-12, (b * e - c * d) / np.where(den > 1e-12, den, 1.0), 0.0)  # pragma: no mutate (parallel-segment guard)
    sc = np.clip(sc, 0.0, 1.0)
    tc = np.where(c > 1e-12, (b * sc + e) / np.where(c > 1e-12, c, 1.0), 0.0)  # pragma: no mutate (zero-length capsule guard)
    tc = np.clip(tc, 0.0, 1.0)
    sc = np.where(a > 1e-12, np.clip((tc * b - d) / a, 0.0, 1.0), 0.0)  # pragma: no mutate (zero-length segment guard)
    gap = p0 + sc[:, None] * u - (A + tc[:, None] * v)
    return float(np.min(np.linalg.norm(gap, axis=1) - R))


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
    clearance_mm: float = math.inf      # swept over travel, to declared obstacles
    node_gap_mm: float = 0.0            # worst chassis mount to its nearest node
    arm_offset_mm: float = 0.0          # rocker force points off the bearing mid-plane
    reason: str = ""
    member_clearance_mm: tuple = ()     # pushrod, rocker arm (pushrod), rocker arm (spring), spring
    spring_length_mm: tuple = ()        # damper eye-to-eye at full droop, static and full bump


CHASSIS_MOUNTS = ("rocker_pivot", "spring_inner")


def _upright_motion(s0, st):
    """Rigid motion (R, t) of the upright from state s0 to st, x_st = R x_s0 + t (Kabsch on its three joints)."""
    P = np.array([_vec(getattr(s0, k)) for k in ("lower_outer", "upper_outer", "tie_rod_outer")])
    Q = np.array([_vec(getattr(st, k)) for k in ("lower_outer", "upper_outer", "tie_rod_outer")])
    pc, qc = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - pc).T @ Q)          # = (P - pc)^T (Q - qc): the centred P rows sum to zero
    D = np.diag([1.0, 1.0, float(np.sign(np.linalg.det(Vt.T @ U.T)))])
    R = Vt.T @ D @ U.T
    return R, qc - R @ pc


def _carried(wheel_obstacles, R, t):
    """Capsules of ``wheel_obstacles`` at an upright pose: kind 'wheel' moves both ends with the upright, kind
    'link' (a link or a halfshaft) moves its first end with the upright and keeps the second on the chassis."""
    out = []
    for o in wheel_obstacles:
        p0, p1, r = _vec(o[0]), _vec(o[1]), float(o[2])
        kind = o[3] if len(o) > 3 else "wheel"
        out.append((R @ p0 + t, (R @ p1 + t) if kind == "wheel" else p1, r))
    return out


def actuation_metrics(hp: Hardpoints, travel_mm=(-25.0, 25.0), n: int = 11,
                      obstacles: Sequence = (), nodes=None, wheel_obstacles: Sequence = (),
                      pushrod_skip_mm: float = 0.0, member_radii=(0.0, 0.0, 0.0, 0.0),
                      spring_skip_mm: float = 0.0) -> ActuationMetrics:
    """Solve the real pushrod-rocker-spring chain across travel and measure it.

    ``obstacles`` is a list of (p0, p1, radius_mm) capsules, for example frame
    tubes; the pushrod, both rocker arms and the spring are checked against
    them at every step through travel, not only at static. ``wheel_obstacles``
    are (p0, p1, radius_mm, kind) capsules at static height that move with the
    wheel: kind 'wheel' for the upright, hub and outer joint, kind 'link' for a
    link or halfshaft whose first end rides on the upright and second sits on
    the chassis; they are carried to each wheel position by the upright's rigid
    motion. ``pushrod_skip_mm`` (mm) leaves the pushrod's own rod end and
    bracket at its outer mount out of the check, ``spring_skip_mm`` (mm) the
    spring's end eyes at both ends; ``member_radii`` (mm) are the radii of the
    pushrod, the two rocker arms and the spring (coilover), so clearance is
    surface to surface rather than axis to surface. ``nodes`` (N x 3, mm) are
    frame nodes; the rocker pivot and spring mount are measured to the nearest
    one, because a mount far from a node needs a long, soft bracket.
    """
    try:
        kin = SuspensionKinematics(hp)
        if not kin.motion_ratio_is_real():
            return ActuationMetrics(ok=False, reason="actuation points incomplete")
        trv, mr = kin.motion_ratio_curve(travel_mm[0], travel_mm[1], n)
        trv = _vec(trv); mr = _vec(mr)
        if not np.all(np.isfinite(mr)):
            return ActuationMetrics(ok=False, reason="linkage does not solve over travel")
        i0 = int(np.argmin(np.abs(trv)))
        m0 = float(mr[i0])
        # walk outward from static in small steps, seeding each rocker solve
        # from its neighbour, so the rocker stays on one branch
        swept = []                       # (pushrod_outer, rocker angle) through travel

        def walk(end):
            st = kin.solve_at_travel(0.0)
            L, th, ok = kin.spring_length_at(st, seed=0.0)  # pragma: no mutate (branch seed for the rocker solve; one branch near static)
            if not ok:
                return None
            swept.append((st, th))
            for z in np.linspace(0.0, end, 11)[1:]:
                st = kin.solve_at_travel(float(z))
                L, th, ok = kin.spring_length_at(st, seed=th)  # pragma: no mutate (branch seed from the neighbour)
                if not ok:
                    return None
                swept.append((st, th))
            return L, th
        lo_w, hi_w = walk(travel_mm[0]), walk(travel_mm[1])
        L0, th0, ok0 = kin.spring_length_at(kin.solve_at_travel(0.0), seed=0.0)  # pragma: no mutate (branch seed)
        if lo_w is None or hi_w is None or not ok0:
            return ActuationMetrics(ok=False, reason="spring length unsolved")
        lengths = [lo_w[0], L0, hi_w[0]]
        thetas = [lo_w[1], th0, hi_w[1]]
        k = _vec(kin._rocker_axis)
        s0 = kin.solve_at_travel(0.0)
        pro = _vec(s0.pushrod_outer)          # set whenever the rocker is real, checked above
        piv = _vec(hp.rocker_pivot)
        rp = _vec(hp.rocker_pushrod); rs = _vec(hp.rocker_spring)
        si = _vec(hp.spring_inner)
        oop = max(math.degrees(math.asin(min(1.0, abs(float(k @ (v / np.linalg.norm(v)))))))  # pragma: no mutate (rounding guard)
                  for v in (rp - pro, si - rs))
        tr = []
        for th, t in zip(thetas, (travel_mm[0], 0.0, travel_mm[1])):
            st = kin.solve_at_travel(float(t))
            p_out = _vec(st.pushrod_outer)
            rp_t = kin._rotate_about_axis(rp, th); rs_t = kin._rotate_about_axis(rs, th)
            tr.append(_angle_deg(rp_t - piv, p_out - rp_t))
            tr.append(_angle_deg(rs_t - piv, si - rs_t))
        clear = math.inf
        mclear = [math.inf] * 4
        if len(obstacles) or len(wheel_obstacles):
            for st, th in swept[::2]:
                obs = list(obstacles)
                if len(wheel_obstacles):
                    obs += _carried(wheel_obstacles, *_upright_motion(s0, st))
                A = _float_array([o[0] for o in obs])
                B = _float_array([o[1] for o in obs])
                R = _float_array([o[2] for o in obs])
                p_o = _vec(st.pushrod_outer)
                rp_t = kin._rotate_about_axis(rp, th); rs_t = kin._rotate_about_axis(rs, th)
                if pushrod_skip_mm > 0.0:  # pragma: no mutate (a zero skip moves nothing)
                    u = rp_t - p_o
                    p_o = p_o + min(pushrod_skip_mm, 0.5 * float(np.linalg.norm(u))) * u / np.linalg.norm(u)
                s_a, s_b = rs_t, si
                if spring_skip_mm > 0.0:  # pragma: no mutate (a zero skip moves nothing)
                    u = si - rs_t; Lu = float(np.linalg.norm(u)); e = min(spring_skip_mm, 0.4 * Lu) * u / Lu
                    s_a, s_b = rs_t + e, si - e
                for j, (seg, rm) in enumerate(zip(((p_o, rp_t), (piv, rp_t), (piv, rs_t), (s_a, s_b)), member_radii)):
                    c_ = _seg_dist_many(seg[0], seg[1], A, B, R + float(rm))
                    mclear[j] = min(mclear[j], c_); clear = min(clear, c_)
        gap = 0.0
        if nodes is not None and len(nodes):
            N = _vec(nodes)
            gap = max(float(np.min(np.linalg.norm(N - _vec(getattr(hp, p)), axis=1)))
                      for p in CHASSIS_MOUNTS)
        return ActuationMetrics(
            ok=True, travel=trv.tolist(), mr=mr.tolist(), mr_static=m0,
            spread=float((mr.max() - mr.min()) / m0),
            slope_per_mm=float(np.polyfit(trv, mr, 1)[0]),
            stroke_used_mm=float(abs(lengths[2] - lengths[0])),
            out_of_plane_deg=float(oop),
            transmission_deg=(float(min(tr)), float(max(tr))), clearance_mm=clear,
            node_gap_mm=gap,
            arm_offset_mm=float(max(abs((rp - piv) @ k), abs((rs - piv) @ k))),
            member_clearance_mm=tuple(float(c) for c in mclear),
            spring_length_mm=tuple(float(v) for v in lengths))
    except Exception as e:                                   # noqa: BLE001
        return ActuationMetrics(ok=False, reason=f"{type(e).__name__}: {e}")


def _residual(m: ActuationMetrics, t: ActuationTargets, shrink: float = 1.0) -> np.ndarray:
    """Hinge residuals, zero inside every band; each in units of its tolerance.

    ``shrink`` (dimensionless, <= 1) narrows the bands; the search aims inside
    them so results land strictly within the true band rather than on its edge.
    """
    if not m.ok:
        return np.full(11, 50.0)
    lo, hi = t.transmission_deg
    usable = t.usable_stroke * t.stroke_mm
    return np.array([
        max(0.0, abs(m.mr_static - t.mr_static) - shrink * t.mr_tol) / t.mr_tol,
        max(0.0, m.spread - shrink * t.max_spread) / t.max_spread,
        max(0.0, m.out_of_plane_deg - shrink * t.max_out_of_plane_deg) / 5.0,
        max(0.0, lo + (1 - shrink) * 20.0 - m.transmission_deg[0]) / 10.0,
        max(0.0, m.transmission_deg[1] - hi + (1 - shrink) * 20.0) / 10.0,
        max(0.0, m.stroke_used_mm - shrink * usable) / 5.0,
        max(0.0, t.min_clearance_mm * (2 - shrink) - m.clearance_mm) / 5.0,
        max(0.0, m.node_gap_mm - shrink * t.node_reach_mm) / 5.0,
        max(0.0, m.arm_offset_mm - shrink * t.max_arm_offset_mm) / 5.0,
        # damper length: no top-out before full droop, and full bump inside the usable stroke (a margin of 1 - shrink
        # of the stroke at each end)
        (max(0.0, m.spring_length_mm[0] - (t.damper_extended_mm - (1 - shrink) * t.stroke_mm)) / 5.0
         if t.damper_extended_mm is not None and m.spring_length_mm else 0.0),
        (max(0.0, (t.damper_extended_mm - t.usable_stroke * t.stroke_mm + (1 - shrink) * t.stroke_mm) - m.spring_length_mm[2]) / 5.0
         if t.damper_extended_mm is not None and m.spring_length_mm else 0.0),
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
    import copy
    h = copy.deepcopy(hp)  # pragma: no mutate (the points are reassigned, not edited in place)
    for i, p in enumerate(pts):
        v = _vec(x[3 * i:3 * i + 3])
        setattr(h, p, v / np.linalg.norm(v) if p == "rocker_axis" else v)
    return h


def synthesize_actuation(hp: Hardpoints, targets: ActuationTargets | None = None,
                         box_mm: float = 40.0, pushrod_box_mm: float = 20.0,
                         points: Sequence[str] = ACTUATION_POINTS,
                         obstacles: Sequence = (), n_starts: int = 4,
                         seed: int = 0, ride_hz: float | None = None,
                         sprung_corner_kg: float | None = None,
                         nodes=None, standoff_mm: float = 24.0,
                         n_node_candidates: int = _N_NODE_CANDIDATES, wheel_obstacles: Sequence = (),
                         pushrod_skip_mm: float = 0.0, axis_box: float = _AXIS_BOX,
                         member_radii=(0.0, 0.0, 0.0, 0.0), spring_skip_mm: float = 0.0) -> ActuationResult:
    """Place the actuation points so the linkage meets ``targets``.

    Each point moves inside a box around its seed (``pushrod_box_mm`` for the
    pushrod outer, which sits on the lower wishbone). Bounded least squares on
    hinge residuals, from the seed and ``n_starts - 1`` random starts; the best
    is returned. With ``ride_hz`` (Hz) and ``sprung_corner_kg`` (kg) the spring
    rate (N/mm) that gives that frequency at static height is returned, with the
    frequency across travel. Boxes and standoff in mm; ``nodes`` (N x 3, mm)
    switch on node-seeded synthesis for the chassis mounts. ``wheel_obstacles``
    ``pushrod_skip_mm``, ``member_radii`` and ``spring_skip_mm`` are as in
    :func:`actuation_metrics`; with
    "rocker_axis" among ``points`` the rocker axis is free too (each component
    within ``axis_box`` of the seed's unit axis, renormalised), so the rocker
    can lean to keep a pushrod that passes fore or aft of the hub in its plane.
    """
    t = targets or ActuationTargets.derived()
    pts = list(points)
    x0 = np.concatenate([_vec(getattr(hp, p)) for p in pts])
    half = np.concatenate([np.full(3, pushrod_box_mm if p == "pushrod_outer"
                                   else (axis_box if p == "rocker_axis" else box_mm))
                           for p in pts])
    lo, hi = x0 - half, x0 + half
    kw = dict(obstacles=obstacles, wheel_obstacles=wheel_obstacles, pushrod_skip_mm=pushrod_skip_mm,
              member_radii=member_radii, spring_skip_mm=spring_skip_mm)
    seed_m = actuation_metrics(hp, t.travel_mm, **kw)

    def fun(x):
        m = actuation_metrics(_set(hp, x, pts), t.travel_mm, **kw)
        reg = 0.002 * (x - x0) / half          # stay near the seed when free
        return np.concatenate([_residual(m, t, shrink=0.98), reg])

    if nodes is not None and len(nodes):
        if len(wheel_obstacles) or pushrod_skip_mm or spring_skip_mm or any(member_radii):
            raise ValueError("node-seeded synthesis does not take wheel_obstacles or pushrod_skip_mm")
        # the rocker must sit on structure: seed by carrying the rocker body to
        # candidate nodes (and the spring mount to others), standing off each
        # node far enough to clear its tubes, then optimise locally around the
        # best seeds. Boxes are re-centred on each seed.
        return _synthesize_on_nodes(hp, t, pts, box_mm, pushrod_box_mm, obstacles,
                                    _vec(nodes), standoff_mm,
                                    n_node_candidates, n_starts, ride_hz,
                                    sprung_corner_kg, seed_m)
    rng = np.random.default_rng(seed)
    starts = [x0] + [lo + rng.random(x0.size) * (hi - lo) for _ in range(max(0, n_starts - 1))]
    best = None
    for s in starts:
        try:
            sol = least_squares(fun, s, bounds=(lo, hi), diff_step=1e-3,
                                max_nfev=120, xtol=1e-6, ftol=1e-8)
        except Exception:                                    # noqa: BLE001
            continue
        r = _residual(actuation_metrics(_set(hp, sol.x, pts), t.travel_mm, **kw), t)
        score = float(np.dot(r, r))
        if best is None or score < best[0]:  # pragma: no mutate (a tie keeps the first)
            best = (score, sol.x, r)
        if score <= 1e-12:  # pragma: no mutate (tolerance)
            break
    if best is None:
        return ActuationResult(ok=False, hp=hp, metrics=seed_m, residual=[],
                               seed_metrics=seed_m, reason="no start solved")
    h = _set(hp, best[1], pts)
    m = actuation_metrics(h, t.travel_mm, **kw)
    k_s, fz = math.nan, []
    if ride_hz and sprung_corner_kg and m.ok:
        k_w = (2 * math.pi * ride_hz) ** 2 * sprung_corner_kg / 1000.0   # N/mm
        k_s = k_w / m.mr_static ** 2
        fz = [ride_hz * v / m.mr_static for v in m.mr]
    ok = bool(np.all(best[2] <= 1e-6))  # pragma: no mutate (tolerance)
    return ActuationResult(ok=ok, hp=h, metrics=m,
                           residual=best[2].tolist(), seed_metrics=seed_m,
                           spring_rate_N_per_mm=k_s, ride_frequency_hz=fz)


def _toward(node, target, standoff):
    v = _vec(target) - node
    n = np.linalg.norm(v)
    return node + (v / n * standoff if n > 1e-9 else np.array([0.0, standoff, 0.0]))  # pragma: no mutate (coincident guard)


def _synthesize_on_nodes(hp, t, pts, box_mm, pushrod_box_mm, obstacles, N,
                         standoff, k, n_starts, ride_hz, sprung_corner_kg, seed_m):
    """Node-seeded synthesis: rocker body and spring mount start at frame nodes."""
    import copy
    piv0 = _vec(hp.rocker_pivot)
    near_p = N[np.argsort(np.linalg.norm(N - piv0, axis=1))[:k]]
    near_s = N[np.argsort(np.linalg.norm(N - _vec(hp.spring_inner), axis=1))[:k]]
    seeds = []
    for a in near_p:
        pv = _toward(a, piv0, standoff)
        d = pv - piv0
        for b in near_s:
            if np.allclose(a, b):
                continue
            h = copy.deepcopy(hp)  # pragma: no mutate (as in _set)
            for p in ("rocker_pivot", "rocker_pushrod", "rocker_spring"):
                setattr(h, p, _vec(getattr(hp, p)) + d)
            h.spring_inner = _toward(b, h.rocker_spring, standoff)
            m = actuation_metrics(h, t.travel_mm, obstacles=obstacles, nodes=N)
            r0 = _residual(m, t)
            seeds.append((float(np.dot(r0, r0)), h))
    seeds.sort(key=lambda z: z[0])
    best = None
    for _, h0 in seeds[:max(1, n_starts)]:
        x0 = np.concatenate([_vec(getattr(h0, p)) for p in pts])
        half = np.concatenate([np.full(3, pushrod_box_mm if p == "pushrod_outer" else box_mm)
                               for p in pts])

        def fun(x, h0=h0, x0=x0, half=half):
            m = actuation_metrics(_set(h0, x, pts), t.travel_mm, obstacles=obstacles, nodes=N)
            return np.concatenate([_residual(m, t, shrink=0.98), 0.002 * (x - x0) / half])
        try:
            sol = least_squares(fun, x0, bounds=(x0 - half, x0 + half), diff_step=1e-3,
                                max_nfev=120, xtol=1e-6, ftol=1e-8)
        except Exception:                                    # noqa: BLE001
            continue
        hh = _set(h0, sol.x, pts)
        r = _residual(actuation_metrics(hh, t.travel_mm, obstacles=obstacles, nodes=N), t)
        sc = float(np.dot(r, r))
        if best is None or sc < best[0]:  # pragma: no mutate (a tie keeps the first; best[1] would not compare)
            best = (sc, hh, r)
        if sc <= 1e-12:  # pragma: no mutate (tolerance)
            break
    if best is None:
        return ActuationResult(ok=False, hp=hp, metrics=seed_m, residual=[],
                               seed_metrics=seed_m, reason="no node seed solved")
    m = actuation_metrics(best[1], t.travel_mm, obstacles=obstacles, nodes=N)
    k_s, fz = math.nan, []
    if ride_hz and sprung_corner_kg and m.ok:
        k_w = (2 * math.pi * ride_hz) ** 2 * sprung_corner_kg / 1000.0
        k_s = k_w / m.mr_static ** 2
        fz = [ride_hz * v / m.mr_static for v in m.mr]
    ok = bool(np.all(best[2] <= 1e-6))  # pragma: no mutate (tolerance)
    return ActuationResult(ok=ok, hp=best[1], metrics=m,
                           residual=best[2].tolist(), seed_metrics=seed_m,
                           spring_rate_N_per_mm=k_s, ride_frequency_hz=fz)


# --------------------------------------------------------------------------- #
#  Rocker bearing screening
# --------------------------------------------------------------------------- #
@dataclass
class RockerBearing:
    """The declared rocker bearing pair. C0 in N (static load rating of ONE
    bearing), spacing in mm along the rocker axis; X0, Y0 are the static
    equivalent-load factors (dimensionless; deep-groove ball bearing values by
    default); ``required_s0`` is the static safety factor to meet under shock.
    """
    label: str = "2 x 6001-2RS (declared)"
    C0_N: float = 2360.0
    spacing_mm: float = 30.0
    X0: float = 0.6
    Y0: float = 0.5
    required_s0: float = 2.0


def rocker_bearing_loads(hp: Hardpoints, cases, bearing: RockerBearing = RockerBearing()
                         ) -> list[dict]:
    """Pivot reaction and static safety factor of the rocker bearings, per case.

    Forces in N, moments in N*mm, safety factor dimensionless. For each
    contact-patch load case the member forces are solved at static height; the
    pushrod force acts at the rocker's pushrod point, the spring force along the
    spring at the rocker's spring point with its magnitude from moment balance
    about the rocker axis, and the pivot carries the rest. That reaction splits
    into radial and axial parts, and the moment of the forces about axes
    perpendicular to the rocker axis is carried by the bearing pair as a couple
    over their spacing. Each bearing's static equivalent load is
    P0 = max(Fr, X0 Fr + Y0 Fa), and s0 = C0 / P0 for the worse bearing.
    """
    from . import loadpath as _lp
    kin = SuspensionKinematics(hp)
    if not kin.motion_ratio_is_real():
        raise ValueError("actuation points incomplete")
    st = kin.solve_at_travel(0.0)
    k = _vec(kin._rocker_axis)
    piv = _vec(hp.rocker_pivot)
    rp = _vec(hp.rocker_pushrod); rs = _vec(hp.rocker_spring)
    es = _vec(hp.spring_inner) - rs; es /= np.linalg.norm(es)
    out = []
    for c in cases:
        load = _lp.WheelLoad(Fx=getattr(c, "Fx", 0.0), Fy=getattr(c, "Fy", 0.0),
                             Fz=getattr(c, "Fz", 0.0), Mz=getattr(c, "mz_Nmm", 0.0),
                             Fx_wc=getattr(c, "Fx_wc", 0.0),
                             F_wc_extra=tuple(getattr(c, "F_wc_extra", (0.0, 0.0, 0.0))),
                             M_wc_extra=tuple(getattr(c, "M_wc_extra", (0.0, 0.0, 0.0))))
        mf = _lp.solve_member_forces(kin, st, load)
        T = float(mf.forces["PR"]); u = _vec(mf.axes["PR"])
        Fp = -T * u                                      # on the rocker, at rp
        arm_s = float(k @ np.cross(rs - piv, es))
        f = -float(k @ np.cross(rp - piv, Fp)) / arm_s   # spring force magnitude
        Fs = f * es
        R = Fp + Fs                                      # minus the pivot reaction; only its parts' sizes are used
        Fa = abs(float(R @ k)); Fr = float(np.linalg.norm(R - (R @ k) * k))
        Mv = np.cross(rp - piv, Fp) + np.cross(rs - piv, Fs)
        M = float(np.linalg.norm(Mv - (Mv @ k) * k))
        fr_b = 0.5 * Fr + M / bearing.spacing_mm
        P0 = max(fr_b, bearing.X0 * fr_b + bearing.Y0 * Fa)
        s0 = bearing.C0_N / P0 if P0 > 0 else math.inf  # pragma: no mutate (unloaded guard)
        passes = s0 >= bearing.required_s0  # pragma: no mutate (boundary)
        out.append({"case": getattr(c, "name", str(c)), "pushrod_N": T, "spring_N": f,
                    "pivot_radial_N": Fr, "pivot_axial_N": Fa, "couple_Nmm": M,
                    "bearing_P0_N": P0, "s0": s0, "passes": passes})
    return out


def required_rating_vs_spacing(hp: Hardpoints, cases, spacings_mm,
                               bearing: RockerBearing = RockerBearing()) -> list[dict]:
    """Static rating (N) each rocker bearing needs, at each bearing spacing.

    The couple an offset pushrod puts on the rocker is carried by the bearing
    pair over their spacing, so the requirement falls as they spread. For each
    spacing the worst case's static equivalent load P0 is found and the
    rating needed is ``required_s0`` x P0; compare it with a catalogue C0 to
    choose a bearing type or spacing that the existing node can take.
    """
    out = []
    for sp in spacings_mm:
        b = RockerBearing(spacing_mm=float(sp), X0=bearing.X0, Y0=bearing.Y0)
        p0 = max(r["bearing_P0_N"] for r in rocker_bearing_loads(hp, cases, b))
        out.append({"spacing_mm": float(sp), "P0_N": p0,
                    "required_C0_N": bearing.required_s0 * p0})
    return out


def select_rocker_bearing(hp: Hardpoints, cases, candidates) -> list[dict]:
    """Rank candidate rocker bearings (the team's catalogue data) on the real
    geometry. ``candidates`` is a list of RockerBearing, each with its part's
    catalogue C0 and the spacing the node allows. Returns each with its worst
    static safety factor and whether it meets its ``required_s0``."""
    out = []
    for b in candidates:
        rows = rocker_bearing_loads(hp, cases, b)
        s0 = min(r["s0"] for r in rows)
        meets = s0 >= b.required_s0  # pragma: no mutate (boundary)
        out.append({"label": b.label, "C0_N": b.C0_N, "spacing_mm": b.spacing_mm, "s0": s0, "meets": meets})
    return sorted(out, key=lambda r: -r["s0"])
