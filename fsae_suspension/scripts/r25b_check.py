# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The paper's tire-dependent results re-run on an R25B-shaped tire (supplement S6.34).

fit():    an MF5.2 pure-lateral set whose peak mu and slip at peak match the published Pacejka fit to the Calspan
          flat-belt data of the Hoosier R25B (Lofwenberg et al., Chalmers 2018, table 5.1: mu 2.57, 2.50, 2.41,
          2.36 and peak slip 11, 12, 13, 14.5 deg at 680, 860, 930, 1250 N; paper reference [28]). Fitted: PDY1,
          PDY2, PKY1, PKY2, PCY1, PEY1. Camber and shift terms stay those of the declared set: no source gives them.
tires():  that shape at three friction levels set by LMUY (friction only; cornering stiffness stays the belt fit's):
          the belt fit itself; the car's own 18.0 x 6.0-10 on the belt, 1.82 at 1112 N (Unti et al., Cal Poly 2018,
          paper reference [30]); and the declared 1.55 at 1100 N, which isolates the change of shape.
chain():  for one tire, the results of the paper that depend on it: anti floors against the mu thresholds, steering
          lag, worst combined aligning torque and hand torque on the final front, the trail limit, quasi-steady
          limits, and the dynamic Ackermann target with the axle force the final front gives up at the hairpin.
Loads N, lengths mm, angles deg, torques N*m, times ms."""
from __future__ import annotations

import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.tiremodel import PacejkaLateral, _DEFAULT_FSAE_COEFFS, default_tire   # noqa: E402
from suspension import steering_response as sr, steering_feel as sf                    # noqa: E402
from suspension import dynamic_ackermann as da, tire_utilisation as tu                  # noqa: E402
from suspension.kinematics import SuspensionKinematics                                  # noqa: E402
from suspension.genesis_repro import GenesisManifest, hp_from_dict                      # noqa: E402

SOURCE_A = [(680.0, 2.57, 11.0), (860.0, 2.50, 12.0), (930.0, 2.41, 13.0), (1250.0, 2.36, 14.5)]
CAR_TIRE_BELT_MU_1112 = 1.82          # [30], R25B 18.0 x 6.0-10, 1112 N, 0 deg, 10 psi, read 1.80 to 1.84
FNOMIN = 1100.0
KEYS = ("PDY1", "PDY2", "PKY1", "PKY2", "PCY1", "PEY1")
#: the result of fit(), stored so the chain does not depend on the optimiser
COEFFS = {"PDY1": 2.3975615031750794, "PDY2": -0.40404771211999563, "PKY1": -27.992734107547165,
          "PKY2": 1.2862740477496062, "PCY1": 1.3342057803028988, "PEY1": -0.49043251728143644}
MU_THR_REAR, MU_THR_FRONT = 1.8309945658634041, 1.9485254032910821     # paper Table 4 (rear anti-squat, front anti-dive)
FINAL_FRONT = ROOT / "manifests" / "front_v11_final.recorded.genesis.json"
HAIRPIN_R_AXLE_MM = 3487.3251337106635          # rear-axle-centre radius of the 4.5 m hairpin, tire edge (paper 3.2)
FZ_OUT, FZ_IN = 1248.0, 165.0                    # front outer and inner loads at 1.5 g (paper 3.2)


def make(coeffs: dict, lmuy: float = 1.0) -> PacejkaLateral:
    c = dict(_DEFAULT_FSAE_COEFFS)
    c.update(coeffs)
    return PacejkaLateral(coeffs=c, FNOMIN=FNOMIN, scaling={"LMUY": lmuy})


def peak(tire, fz: float) -> tuple[float, float]:
    """(slip at peak |Fy| in deg, peak mu) refined with a bounded scalar search."""
    from scipy.optimize import minimize_scalar
    a = np.radians(np.linspace(0.0, 30.0, 301))
    i = int(np.argmax(np.abs(tire.fy(a, fz))))
    r = minimize_scalar(lambda x: -abs(float(tire.fy(x, fz))), bounds=(a[max(i - 1, 0)], a[min(i + 1, 300)]),
                        method="bounded", options={"xatol": 1e-7})
    return float(np.degrees(r.x)), -float(r.fun) / fz


def fit() -> dict:
    from scipy.optimize import least_squares

    def res(x):
        t = make(dict(zip(KEYS, x)))
        r = []
        for fz, mu, sl in SOURCE_A:
            s, m = peak(t, fz)
            r += [(m - mu) / 0.01, (s - sl) / 0.25]
        return np.array(r)
    f = least_squares(res, [2.38, -0.12, -40.0, 1.6, 1.4, -0.5],
                      bounds=([1.0, -1.0, -120.0, 0.5, 1.05, -3.0], [4.0, 0.5, -5.0, 5.0, 1.9, 0.99]),
                      x_scale=[0.5, 0.1, 10, 0.5, 0.2, 0.5])
    return dict(zip(KEYS, map(float, f.x)))


def tires(coeffs: dict = COEFFS) -> dict:
    belt = make(coeffs)
    return {"synthetic": default_tire(), "r25b_belt": belt,
            "r25b_car_tire_belt": make(coeffs, CAR_TIRE_BELT_MU_1112 / belt.mu_peak(1112.0, 0.0)),
            "r25b_shape_at_1p55": make(coeffs, 1.55 / belt.mu_peak(1100.0, 0.0))}


def final_front():
    m = GenesisManifest.from_json(FINAL_FRONT.read_text())
    hp = hp_from_dict(m.recorded["winner_hardpoints"])
    se = m.objects()[2].properties.steer_effort
    s = SuspensionKinematics(hp).solve_at_travel(0.0)
    U, L, cp = (np.asarray(v, float) for v in (hp.upper_outer, hp.lower_outer, s.contact_patch))
    k = U - L
    g = L + (-L[2] / k[2]) * k
    return hp, float(cp[0] - g[0]), float(se.kinematic_ratio(hp))


def axle_torque(tire, trail_mm: float, t0_outer_mm: float | None = None, alpha_max: float = 22.0) -> tuple:
    """Worst combined aligning torque of the front wheels at 1.5 g, the slip at it and the torque at the outer's peak.

    The outer wheel's zero-slip trail is varied; the inner keeps its brush-theory trail (as paper section 3.2)."""
    t0o = sf.brush_trail_mm(FZ_OUT) if t0_outer_mm is None else t0_outer_mm
    co = sf.aligning_curve(tire, FZ_OUT, trail_mm, t0o, alpha_max_deg=alpha_max, n=441)
    ci = sf.aligning_curve(tire, FZ_IN, trail_mm, sf.brush_trail_mm(FZ_IN), alpha_max_deg=alpha_max, n=441)
    a, M = np.asarray(co["alpha_deg"]), np.asarray(co["M_Nm"]) + np.asarray(ci["M_Nm"])
    k = int(np.argmax(M))
    return float(M[k]), float(a[k]), float(np.interp(co["alpha_peak_deg"], a, M))


def trail_limit(tire, trail_mm: float, torque_Nm: float) -> float:
    lo, hi = 2.0, 80.0
    for _ in range(60):
        m = 0.5 * (lo + hi)
        lo, hi = (m, hi) if axle_torque(tire, trail_mm, m)[0] < torque_Nm else (lo, m)
    return lo


def g_limit(tire, direction_deg: float, bias: float) -> dict:
    th = math.radians(direction_deg)
    lo, hi = 0.0, 2.5                       # the bracket of supplement S6.21, so the synthetic row reproduces it
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        s = tu.tire_state(mid * math.cos(th), mid * math.sin(th), tire=tire, brake_front=bias)
        lo, hi = (mid, hi) if (s["feasible"] and s["vehicle_u"] <= tu.U_TOL) else (lo, mid)
    assert lo < 2.45, "limit at the top of the bracket"
    s = tu.tire_state(lo * math.cos(th), lo * math.sin(th), tire=tire, brake_front=bias)
    return {"a_g": lo, "limiting": max(tu.TIRES, key=lambda k: s[k]["u"])}


def ackermann(tire, hp, a_lat_g: float) -> dict:
    case = da.AxleCase.from_vehicle(a_lat_g=a_lat_g, radius_mm=HAIRPIN_R_AXLE_MM)
    t = da.dynamic_target(tire, case)
    lo, hi = 0.0, 60.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if da.kinematic_split(hp, mid)["outer_deg"] < t["geometric_outer_deg"] else (lo, mid)
    split = da.kinematic_split(hp, hi)["difference_deg"]
    return {"target_pct": t["dynamic_ackermann_pct"], "slip_outer": t["peak_slip_outer_deg"],
            "slip_inner": t["peak_slip_inner_deg"],
            "final_front_loss_pct": da.axle_force_at_split(tire, case, split)["loss_pct"]}


def chain(tire, hp, trail_mm: float, ratio: float) -> dict:
    o = {"mu": {str(fz): tire.mu_peak(fz, 0.0) for fz in (165.0, 706.0, 1100.0, 1112.0, 1248.0)},
         "peak_slip": {str(fz): da.peak_slip_deg(tire, fz) for fz in (165.0, 706.0, 1100.0, 1248.0)}}
    o["anti_squat_holds"] = o["mu"]["1100.0"] <= MU_THR_REAR
    o["anti_dive_holds"] = o["mu"]["1100.0"] <= MU_THR_FRONT
    car = sr.tire_derived_single_track(tire=tire)
    o["cf_kn"], o["cr_kn"] = car.cf_n_per_rad / 1000.0, car.cr_n_per_rad / 1000.0
    o["lag"] = sr.lag_budget(car, 2.5)
    o["lag_sigma_049"] = sr.lag_budget(replace(car, sigma_m=0.49), 2.5)
    pk, apk, lim = axle_torque(tire, trail_mm)
    o["torque"] = {"peak_Nm": pk, "alpha_deg": apk, "at_limit_Nm": lim, "hand_peak_Nm": pk / ratio,
                   "ratio_for_10Nm": pk / 10.0,
                   "t0_limit_window_mm": trail_limit(tire, trail_mm, 10.0 * 120.0 / 25.6),
                   "t0_limit_final_mm": trail_limit(tire, trail_mm, 10.0 * ratio)}
    s = tu.tire_state(0.0, 1.5, tire=tire)
    o["corner_1p5"] = {"feasible": s["feasible"], "spare_n": s["grip_spare_n"] if s["feasible"] else None}
    o["limits_73"] = {n: g_limit(tire, th, 0.73) for n, th in (("straight_braking", 0.0), ("braking_cornering", 45.0),
                                                               ("steady_cornering", 90.0), ("power_on", 135.0))}
    o["limits_60"] = {n: g_limit(tire, th, 0.60) for n, th in (("straight_braking", 0.0), ("braking_cornering", 45.0))}
    o["ackermann"] = {str(g): ackermann(tire, hp, g) for g in (1.5, 1.0)}
    return o


def run(out_path: str | None = None, refit: bool = False) -> dict:
    coeffs = fit() if refit else COEFFS
    hp, trail, ratio = final_front()
    out = {"coeffs": coeffs, "trail_mm": trail, "ratio_centre": ratio, "mu_thr_rear": MU_THR_REAR,
           "mu_thr_front": MU_THR_FRONT, "fit_rows": [], "levels": {}}
    belt = make(coeffs)
    for fz, mu, sl in SOURCE_A:
        s, m = peak(belt, fz)
        out["fit_rows"].append({"Fz": fz, "mu_src": mu, "mu_fit": m, "slip_src": sl, "slip_fit": s})
    for name, t in tires(coeffs).items():
        out["levels"][name] = chain(t, hp, trail, ratio)
        out["levels"][name]["lmuy"] = float(getattr(t, "scaling", {}).get("LMUY", 1.0)) if name != "synthetic" else 1.0
    if out_path:
        Path(out_path).write_text(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    r = run(sys.argv[1] if len(sys.argv) > 1 else None, refit="--refit" in sys.argv)
    for n, o in r["levels"].items():
        print(n, round(o["mu"]["1100.0"], 3), round(o["lag"]["total_ms"], 1), round(o["torque"]["hand_peak_Nm"], 2),
              round(o["torque"]["t0_limit_final_mm"], 1), round(o["limits_73"]["steady_cornering"]["a_g"], 3),
              round(o["ackermann"]["1.5"]["target_pct"], 1), round(o["ackermann"]["1.5"]["final_front_loss_pct"], 3))
