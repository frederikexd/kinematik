# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Rear torque distribution, torque vectoring and tire exploitation.

At every instant the car must satisfy force and yaw balance:

    Fy_f + Fy_r = m a_y
    a Fy_f - b Fy_r + Mz_x = Iz r_dot

with a and b the CG-to-axle distances, Mz_x the yaw moment of unequal left
and right longitudinal forces, (Fx_out - Fx_in) t / 2 on the rear axle, and
Iz r_dot the moment that changes the yaw rate (zero in a steady corner). An
equal split (Mz_x = 0) fixes the axle lateral split at the mass ratio; any
Mz_x moves lateral force between the axles, which is how torque vectoring
reaches grip an equal split cannot. More lateral grip therefore needs more
yaw moment, from the split or from the tires.

Distribution options for the driven rear axle, written as the outside and
inside forces F_R/2 + d and F_R/2 - d (N):
    open        equal torque, d = 0
    lsd2, lsd3  clutch limited-slip, torque to the slower inside wheel up to a
                torque-bias ratio of 2 or 3 under drive, d in [F_R/2 (1-k)/(1+k), 0]
    spool       locked axle: the path forces a slip difference t·kappa between
                the wheels, so d = -cx Fz t kappa / 2 (declared slip stiffness),
                capped at each tire's grip; it drags the outside wheel
    tv          independent rear motors: d free, each wheel within a declared
                motor force limit, in drive and in regenerative braking
Mechanical brakes split equally left and right for every option except tv,
which vectors the rear share by regeneration.

Tire forces use the MF5.2 set, tabulated once, with lateral force scaled by
the friction ellipse for each tire's Fx (mu_x = mu declared). Units: SI (N,
m, s, rad) internally; accelerations reported in g; utilisation dimensionless.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .target_derivation import Vehicle, corner_loads
from .tiremodel import default_tire

G = 9.81
OPTIONS = ("open", "lsd2", "lsd3", "spool", "tv")
_ALPHA = np.radians(np.arange(0.0, 20.01, 0.25))
_FZ = np.arange(20.0, 2601.0, 20.0)


class TireTable:
    """Lateral force magnitude (N) tabulated over slip angle (rad) and load (N)."""

    def __init__(self, tire=None, gamma: float = 0.0):
        tire = tire or default_tire()
        self.fy = np.array([[abs(float(tire.fy(a, fz, gamma))) for a in _ALPHA] for fz in _FZ])
        self.mu = self.fy.max(axis=1) / _FZ

    def row(self, fz: float) -> np.ndarray:
        """Fy (N) against slip angle for one load in N, linear in load."""
        i = int(np.clip(np.searchsorted(_FZ, fz) - 1, 0, len(_FZ) - 2))
        w = float(np.clip((fz - _FZ[i]) / (_FZ[i + 1] - _FZ[i]), 0.0, 1.0))
        return self.fy[i] * (1.0 - w) + self.fy[i + 1] * w

    def mu_at(self, fz: float) -> float:
        """Peak friction coefficient (dimensionless) at a load in N."""
        return float(np.interp(fz, _FZ, self.mu))


@dataclass
class Car:
    """Declared car for the torque-distribution studies. Mass kg, lengths m, Iz kg·m²."""
    m: float = 300.0
    wf: float = 0.48
    L: float = 1.63
    t: float = 1.21
    iz: float = 198.95
    cda: float = 1.0
    crr: float = 0.015
    brake_front: float = 0.73
    rho: float = 1.225
    tv_wheel_max_n: float = 1200.0     # declared per-wheel motor force limit for tv
    cx_per_fz: float = 20.0            # declared longitudinal slip stiffness / Fz, for the spool
    front_lltd: float = 0.53           # front share of lateral load transfer

    @property
    def a(self) -> float:
        """CG to front axle, m."""
        return self.L * (1.0 - self.wf)
    @property
    def b(self) -> float:
        """CG to rear axle, m."""
        return self.L * self.wf


def _d_range(option: str, driving: bool, FR: float, car: "Car", fz_out: float, fz_in: float, kappa: float, tt) -> tuple:
    """Allowed outside-minus-mean force d (N) for an option; (lo, hi), equal for a fixed split."""
    if option == "tv":
        D = max(0.0, car.tv_wheel_max_n - abs(FR) / 2.0); return (-D, D)
    if option == "spool":
        fzm = 0.5 * (fz_out + fz_in); d = -car.cx_per_fz * fzm * car.t * kappa / 2.0
        cap = min(tt.mu_at(max(fz_out, 1.0)) * max(fz_out, 1.0), tt.mu_at(max(fz_in, 1.0)) * max(fz_in, 1.0)) - abs(FR) / 2.0
        d = max(d, -max(cap, 0.0)); return (d, d)
    if not driving or option == "open": return (0.0, 0.0)
    k = {"lsd2": 2.0, "lsd3": 3.0}[option]
    return (FR / 2.0 * (1.0 - k) / (1.0 + k), 0.0)


def _axle(tt: TireTable, need: float, fz, fx):
    """Common slip angle solve on one axle; returns per-tire Fy (N) or None if past capacity."""
    rows, scale = [], []
    for z, x in zip(fz, fx):
        z = max(z, 1.0); mu = tt.mu_at(z); r = min(1.0, abs(x) / (mu * z))
        rows.append(tt.row(z)); scale.append(math.sqrt(max(0.0, 1.0 - r * r)))
    tot = scale[0] * rows[0] + scale[1] * rows[1]
    k = int(np.argmax(tot)); need = abs(need)
    if tot[k] < need: return ("over", need / max(tot[k], 1e-6))
    mono = np.maximum.accumulate(tot[:k + 1])
    a = float(np.interp(need, mono, _ALPHA[:k + 1]))
    return [scale[i] * float(np.interp(a, _ALPHA, rows[i])) for i in range(2)], a


def state(V: float, kappa: float, dkds: float, at: float, option: str, car: Car | None = None,
          tt: TireTable | None = None, veh: Vehicle | None = None, n_split: int = 21) -> dict:
    """Best feasible force allocation at one instant; speed m/s, curvature 1/m, dkappa/ds 1/m², at m/s².

    Returns the utilisation of each tire (u), the chosen force difference d (N), the
    split yaw moment Mz_x and required Iz r_dot (N·m), and ``ok`` when every
    tire stays inside its ellipse. Left-hand turns (kappa >= 0): outside = right.
    """
    car = car or Car(); tt = tt or _TT(); veh = veh or Vehicle()
    ay = V * V * kappa; ay_g = ay / G
    mz_req = car.iz * (V * V * dkds + at * kappa)
    fz = corner_loads(veh, a_lat_g=ay_g, a_long_g=-at / G, front_lltd=car.front_lltd)["total"]
    delta = car.L * kappa
    best = None
    for _ in range(2):
        drag = 0.5 * car.rho * car.cda * V * V + car.crr * car.m * G
        f_ind = (best["Fy_f"] * math.sin(delta)) if best else car.m * ay * car.wf * math.sin(delta)
        F = car.m * at + drag + f_ind
        driving = F >= 0.0
        fxf = 0.0 if driving else car.brake_front * F / 2.0
        FR = F if driving else (1.0 - car.brake_front) * F
        lo, hi = _d_range(option, driving, FR, car, fz["RR"], fz["RL"], kappa, tt)
        def evaluate(d):
            """Allocation for an outside-minus-mean rear force d in N; forces in N, moments in N·m."""
            fx_out, fx_in = FR / 2.0 + d, FR / 2.0 - d
            mzx = (fx_out - fx_in) * car.t / 2.0
            fyf = (car.m * ay * car.b + mz_req - mzx) / car.L
            fyr = car.m * ay - fyf
            front = _axle(tt, fyf, (fz["FL"], fz["FR"]), (fxf, fxf))
            rear = _axle(tt, fyr, (fz["RL"], fz["RR"]), (fx_in, fx_out))
            over = [x[1] for x in (front, rear) if x[0] == "over"]
            if over: return {"umax": 1.0 + max(over), "infeasible": True}
            (fy_fl, fy_fr), af = front; (fy_rl, fy_rr), ar = rear
            fx = {"FL": fxf, "FR": fxf, "RL": fx_in, "RR": fx_out}; fy = {"FL": fy_fl, "FR": fy_fr, "RL": fy_rl, "RR": fy_rr}
            u = {k: math.hypot(fx[k], fy[k]) / (tt.mu_at(max(fz[k], 1.0)) * max(fz[k], 1.0)) for k in fx}
            return {"umax": max(u.values()), "u": u, "d": float(d), "mz_x": mzx, "mz_req": mz_req, "Fy_f": fyf, "Fy_r": fyr,
                    "F_long": F, "alpha_f": af, "alpha_r": ar, "fz": fz, "fx": fx, "fy": fy}
        cost = lambda r: r["umax"] if r else 99.0
        if hi <= lo: best = evaluate(lo)
        else:
            grid = list(np.linspace(lo, hi, 25)) + ([0.0] if lo <= 0.0 <= hi else [])
            scan = [(cost(r), x, r) for x in grid for r in [evaluate(x)]]
            c0, xb, rb = min(scan, key=lambda z: z[0])
            step = (hi - lo) / 24.0; a_, b_ = max(lo, xb - step), min(hi, xb + step)
            gr = (math.sqrt(5) - 1) / 2; x1, x2 = b_ - gr * (b_ - a_), a_ + gr * (b_ - a_); r1, r2 = evaluate(x1), evaluate(x2)
            for _ in range(18):
                if cost(r1) <= cost(r2): b_, x2, r2 = x2, x1, r1; x1 = b_ - gr * (b_ - a_); r1 = evaluate(x1)
                else: a_, x1, r1 = x1, x2, r2; x2 = a_ + gr * (b_ - a_); r2 = evaluate(x2)
            cands = [r for r in (rb, r1, r2) if r]
            best = min(cands, key=cost) if cands else None
        if best is None or best.get("infeasible"): return {"ok": False, "umax": math.inf if best is None else best["umax"]}
        delta = car.L * kappa + best["alpha_f"] - best["alpha_r"]
    best["ok"] = best["umax"] <= 1.0; best["ay_g"] = ay_g; best["at_g"] = at / G
    return best


_TT_CACHE = []
def _TT():
    """The tabulated default tire, built once per process (lookup table, units N and rad)."""
    if not _TT_CACHE: _TT_CACHE.append(TireTable())
    return _TT_CACHE[0]


def steady_limit(R: float, option: str, car: Car | None = None) -> dict:
    """Highest steady speed (m/s) on a circle of radius R (m) and the allocation there."""
    lo, hi = 1.0, 30.0
    for _ in range(26):
        mid = 0.5 * (lo + hi)
        if state(mid, 1.0 / R, 0.0, 0.0, option, car)["ok"]: lo = mid
        else: hi = mid
    s = state(lo, 1.0 / R, 0.0, 0.0, option, car)
    return {"R": R, "V": lo, "ay_g": lo * lo / R / G, **{k: s[k] for k in ("d", "mz_x", "umax", "u", "F_long")}}


def corner_path(R: float, arc_deg: float, straight: float = 40.0, transition: float = 6.0, ds: float = 0.5):
    """Curvature profile (1/m) of straight, clothoid, arc, clothoid, straight; returns s, kappa, dkappa/ds."""
    arc = math.radians(arc_deg) * R
    segs = [(straight, 0.0, 0.0), (transition, 0.0, 1.0 / R), (arc, 1.0 / R, 1.0 / R), (transition, 1.0 / R, 0.0), (straight, 0.0, 0.0)]
    s, k, dk = [], [], []; s0 = 0.0
    for L, k0, k1 in segs:
        n = max(1, int(round(L / ds)))
        for i in range(n):
            x = (i + 0.5) / n; s.append(s0 + x * L); k.append(k0 + (k1 - k0) * x); dk.append((k1 - k0) / L)
        s0 += L
    return np.array(s), np.array(k), np.array(dk)


def simulate_corner(R: float, arc_deg: float, option: str, v_top: float = 22.0, car: Car | None = None, ds: float = 0.5) -> dict:
    """Quasi-steady speed profile and tire exploitation through one corner.

    Forward pass at the traction limit, backward pass at the braking limit,
    each tangential acceleration the largest the allocation allows at that
    speed, curvature and curvature rate. Returns time (s), speed (m/s) and the
    per-tire utilisation, split yaw moment and Iz r_dot (N·m) along the path.
    """
    car = car or Car(); s, k, dk = corner_path(R, arc_deg, ds=ds); n = len(s)
    def vcap(i):
        """Highest feasible speed in m/s at path point i with no tangential acceleration."""
        lo, hi = 1.0, v_top
        if state(hi, k[i], dk[i], 0.0, option, car)["ok"]: return hi
        for _ in range(18):
            mid = 0.5 * (lo + hi)
            if state(mid, k[i], dk[i], 0.0, option, car)["ok"]: lo = mid
            else: hi = mid
        return lo
    def amax(V, i, sign):
        """Largest feasible tangential acceleration in m/s² at speed V (m/s), point i; sign +1 drive, -1 brake."""
        lo, hi = 0.0, 16.0
        for _ in range(14):
            mid = 0.5 * (lo + hi)
            if state(V, k[i], dk[i], sign * mid, option, car)["ok"]: lo = mid
            else: hi = mid
        return lo
    cap = np.array([vcap(i) for i in range(n)])
    vf = cap.copy(); vf[0] = min(cap[0], v_top)
    for i in range(1, n): vf[i] = min(cap[i], math.sqrt(vf[i - 1] ** 2 + 2 * amax(vf[i - 1], i - 1, +1) * ds))
    vb = vf.copy()
    for i in range(n - 2, -1, -1): vb[i] = min(vf[i], math.sqrt(vb[i + 1] ** 2 + 2 * amax(vb[i + 1], i + 1, -1) * ds))
    V = vb; at = np.gradient(V * V / 2.0, s)
    rows = [state(V[i], k[i], dk[i], at[i], option, car) for i in range(n)]
    t = np.concatenate([[0.0], np.cumsum(ds / np.maximum(0.5 * (V[1:] + V[:-1]), 0.1))])
    return {"s": s, "t": t, "V": V, "kappa": k, "at": at, "time": float(t[-1]),
            "u": {w: np.array([r["u"][w] if r.get("ok") or "u" in r else np.nan for r in rows]) for w in ("FL", "FR", "RL", "RR")},
            "mz_x": np.array([r.get("mz_x", np.nan) for r in rows]), "mz_req": np.array([r.get("mz_req", np.nan) for r in rows]),
            "ay_g": np.array([V[i] ** 2 * k[i] / G for i in range(n)]), "at_g": at / G}
