# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The lap solver (suspension/laptime.py, simulate_lap) against TUM's published quasi-steady-state lap time simulation
and an independent integration of the same equations.

The car is reduced to a constant-mu point mass on both sides (see benchmarks/laptime/limits.json), so this checks the
lap solver: corner limits with downforce, the friction circle, the power limit, drag and rolling resistance, and the
forward/backward passes. It does not check the tyre model or load transfer.

TUM's code is not part of KinematiK. Clone it and point the script at it:

    git clone https://github.com/TUMFTM/laptime-simulation /path/to/laptime-simulation
    pip install trajectory_planning_helpers==0.66 --no-deps && pip install quadprog
    python scripts/laptime_benchmark.py --tum /path/to/laptime-simulation      # exit 1 on any failure

Without --tum the TUM rows keep their stored values and only KinematiK and the independent integration are rerun.
Problem data and limits: benchmarks/laptime/limits.json, registered before the run. Results: results.json.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import laptime as lt  # noqa: E402

BENCH = ROOT / "benchmarks" / "laptime"
REG = json.loads((BENCH / "limits.json").read_text())
V = REG["problem"]["vehicle"]
G = 9.81
W_BOX, H_BOX = 120.0, 60.0
R_BR, R_TR, R_TL, R_BL = 8.0, 15.0, 25.0, 12.0


def segments() -> list[tuple[float, float]]:
    """(length m, radius m or inf), anticlockwise from the middle of the bottom straight."""
    q = math.pi / 2
    bottom = W_BOX - R_BL - R_BR
    return [(bottom / 2, math.inf), (q * R_BR, R_BR), (H_BOX - R_BR - R_TR, math.inf), (q * R_TR, R_TR),
            (W_BOX - R_TR - R_TL, math.inf), (q * R_TL, R_TL), (H_BOX - R_TL - R_BL, math.inf), (q * R_BL, R_BL),
            (bottom / 2, math.inf)]


def corner_names():
    return {R_BR: "R8", R_TR: "R15", R_TL: "R25", R_BL: "R12"}


def _radius_at(s: float) -> float:
    acc = 0.0
    segs = segments()
    for L, r in segs:
        if s < acc + L:
            return r
        acc += L
    return segs[-1][1]


def _min_corner_speeds(s, v) -> dict:
    out, acc = {}, 0.0
    for L, r in segments():
        if math.isfinite(r):
            m = [vv for ss, vv in zip(s, v) if acc <= ss <= acc + L]
            out[corner_names()[r]] = float(min(m))
        acc += L
    return out


# ---------------------------------------------------------------------------------------------- KinematiK
def kinematik() -> dict:
    m, mu, wdf = V["mass_kg"], V["mu"], V["weight_dist_front"]
    pt = lt.Powertrain(power_kw=V["wheel_power_kW"], max_tractive_n=1e9, drivetrain_eff=1.0, cda=V["CdA_m2"],
                       cla=V["ClA_m2"], rho=V["rho_kg_m3"], crr=V["crr"], drive=V["drive"], brake_g_cap=100.0,
                       rot_mass_factor=1.0)
    veh = SimpleNamespace(p=SimpleNamespace(mass=m, weight_dist_front=wdf), max_lateral_g=lambda: mu)
    # the lateral-limit table KinematiK builds from its GGV, for a tyre whose mu is constant
    vs = list(lt._LAT_TABLE_V)
    veh._kx_lat_limit_table = (vs, [mu * (1 + 0.5 * pt.rho * pt.cla * vv * vv / (m * G)) for vv in vs],
                               round(float(mu), 9))
    track = lt.Track("benchmark", [lt.Segment(L, None if not math.isfinite(r) else r) for L, r in segments()], ds=1.0)
    r = lt.simulate_lap(veh, track, pt)
    assert r.ok and not r.warning, r.warning
    return {"lap_time_s": r.lap_time_s, "corner_min_speed_ms": _min_corner_speeds(r.s, r.v),
            "top_speed_ms": r.top_speed_ms, "_s": r.s, "_v": r.v}


# ---------------------------------------------------------------------------------------------- independent
def exact(ds: float = 0.01) -> dict:
    """The same point-mass equations, integrated independently by RK4 in distance."""
    m, mu, wdf = V["mass_kg"], V["mu"], V["weight_dist_front"]
    rho, cla, cda, crr, P = V["rho_kg_m3"], V["ClA_m2"], V["CdA_m2"], V["crr"], V["wheel_power_kW"] * 1000.0
    k = 0.5 * rho * cla / m
    L_tot = sum(L for L, _ in segments())
    n = int(round(L_tot / ds))
    s = np.linspace(0.0, L_tot, n + 1)
    R = np.array([_radius_at(x) for x in s])
    curv = np.where(np.isfinite(R), 1.0 / R, 0.0)
    with np.errstate(divide="ignore"):
        ceil = np.where(curv > 0, np.sqrt(mu * G / np.maximum(curv - mu * k, 1e-12)), 1e9)

    def frac(v, c):
        return min(v * v * c / (mu * (G + k * v * v)), 1.0)

    def acc(v, c):
        fd = 0.5 * rho * cla * v * v
        grip = mu * (m * G + fd) * (1 - wdf) * math.sqrt(max(1 - frac(v, c) ** 2, 0.0))
        return (min(grip, P / max(v, 0.1)) - 0.5 * rho * cda * v * v - crr * (m * G + fd)) / m

    def dec(v, c):
        fd = 0.5 * rho * cla * v * v
        return (mu * (m * G + fd) * math.sqrt(max(1 - frac(v, c) ** 2, 0.0)) + 0.5 * rho * cda * v * v) / m

    def step(v, c, f, h):
        g = lambda vv: f(max(vv, 0.1), c) / max(vv, 0.1)
        k1 = g(v); k2 = g(v + h / 2 * k1); k3 = g(v + h / 2 * k2); k4 = g(v + h * k3)
        return v + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    v = ceil.copy()
    vf = np.empty_like(v)
    v0 = 5.0
    for _ in range(3):                               # laps until the start speed repeats
        vf[0] = min(v0, ceil[0])
        for i in range(n):
            vf[i + 1] = min(step(vf[i], curv[i], acc, ds), ceil[i + 1])
        v0 = vf[-1]
    vb = vf.copy()
    v1 = vb[-1]
    for _ in range(3):
        vb[-1] = min(vf[-1], v1)
        for i in range(n, 0, -1):
            vb[i - 1] = min(vf[i - 1], step(vb[i], curv[i], dec, ds))
        v1 = vb[0]
    t = float(np.sum(2 * ds / (vb[1:] + vb[:-1])))
    return {"lap_time_s": t, "corner_min_speed_ms": _min_corner_speeds(s, vb), "top_speed_ms": float(vb.max()),
            "_s": s, "_v": vb}


# ---------------------------------------------------------------------------------------------- TUM
def tum(path: str) -> dict:
    sys.path.insert(0, path)
    # TUM's code and its helper package predate NumPy 1.24, which removed these aliases; restore them for it
    for name, typ in (("int", int), ("float", float), ("bool", bool)):
        if name not in np.__dict__:
            setattr(np, name, typ)
    import laptimesim  # noqa: F401
    from laptimesim.src.track import Track
    from laptimesim.src.car_electric import CarElectric
    from laptimesim.src.driver import Driver
    from laptimesim.src.lap import Lap

    m, mu, wdf = V["mass_kg"], V["mu"], V["weight_dist_front"]
    wb = 1.53
    tmp = Path(tempfile.mkdtemp())
    pts, x, y, hd = [], 0.0, 0.0, 0.0
    x = R_BL + (W_BOX - R_BL - R_BR) / 2
    for L, r in segments():                              # the centreline every 0.1 m
        nn = int(round(L / 0.1))
        for _ in range(nn):
            pts.append((x, y))
            h = L / nn
            if math.isfinite(r):
                dth = h / r
                x += r * (math.sin(hd + dth) - math.sin(hd))
                y -= r * (math.cos(hd + dth) - math.cos(hd))
                hd += dth
            else:
                x += h * math.cos(hd)
                y += h * math.sin(hd)
    np.savetxt(tmp / "Bench.csv", np.array(pts), delimiter=",")
    (tmp / "track_pars.ini").write_text("[TRACK_PARS]\ntrack_pars=" + json.dumps({"Bench": {
        "s12": 100.0, "s23": 200.0, "mu_mean": 1.0, "drs1_act": 0.0, "drs1_deact": 0.0, "drs2_act": 0.0,
        "drs2_deact": 0.0, "pitspeed": 22.222, "pit_in": 0.0, "pit_out": 0.0}}) + "\n")
    veh = {"powertrain_type": "electric",
           "general": {"lf": wb * (1 - wdf), "lr": wb * wdf, "h_cog": 0.0, "sf": 1.2, "sr": 1.2, "m": m,
                       "f_roll": V["crr"], "c_w_a": V["CdA_m2"], "c_z_a_f": V["ClA_m2"] * wdf,
                       "c_z_a_r": V["ClA_m2"] * (1 - wdf), "g": G, "rho_air": V["rho_kg_m3"], "drs_factor": 0.0},
           "engine": {"topology": "RWD", "pow_e_motor": V["wheel_power_kW"] * 1000.0, "eta_e_motor": 1.0,
                      "eta_e_motor_re": 1.0, "torque_e_motor_max": 1e6},
           "gearbox": {"i_trans": [0.1], "n_shift": [1e9], "e_i": [1.0], "eta_g": 1.0},
           "tires": {"f": {"circ_ref": 1.4, "fz_0": 700.0, "mux": mu, "muy": mu, "dmux_dfz": 0.0, "dmuy_dfz": 0.0},
                     "r": {"circ_ref": 1.4, "fz_0": 700.0, "mux": mu, "muy": mu, "dmux_dfz": 0.0, "dmuy_dfz": 0.0},
                     "tire_model_exp": 2.0}}
    (tmp / "veh.ini").write_text("[VEH_PARS]\nveh_pars=" + json.dumps(veh) + "\n")
    track_opts = {"trackname": "Bench", "flip_track": False, "mu_weather": 1.0, "interp_stepsize_des": 1.0,
                  "curv_filt_width": None, "use_drs1": False, "use_drs2": False, "use_pit": False}
    tr = Track(pars_track=track_opts, parfilepath=str(tmp / "track_pars.ini"), trackfilepath=str(tmp / "Bench.csv"))
    L_tot = sum(L for L, _ in segments())
    scale = L_tot / tr.dists_cl[-1]
    # the exact curvature in place of the spline estimate (sign: anticlockwise positive)
    tr.kappa = np.array([0.0 if not math.isfinite(rr) else 1.0 / rr
                         for rr in (_radius_at(d * scale) for d in tr.dists_cl[:-1])])
    car = CarElectric(parfilepath=str(tmp / "veh.ini"))
    drv = Driver(carobj=car, pars_driver={"vel_subtr_corner": 0.0, "vel_lim_glob": None, "yellow_s1": False,
                                          "yellow_s2": False, "yellow_s3": False, "yellow_throttle": 0.3,
                                          "initial_energy": 4.0e8, "em_strategy": "FCFB", "use_recuperation": False,
                                          "use_lift_coast": False, "lift_coast_dist": 10.0},
                 trackobj=tr, stepsize=tr.stepsize)
    lap = Lap(driverobj=drv, trackobj=tr, pars_solver={"series": "FE", "vehicle": "veh.ini", "limit_braking_weak_side": None, "v_start": 10.0,
                                                       "find_v_start": True, "max_no_em_iters": 5,
                                                       "es_diff_max": 1.0},
              debug_opts={"use_plot": False, "use_debug_plots": False, "use_plot_comparison_tph": False,
                          "use_print": False, "use_print_result": False})
    lap.simulate_lap()
    s = tr.dists_cl * scale
    return {"lap_time_s": float(lap.t_cl[-1]), "corner_min_speed_ms": _min_corner_speeds(s, lap.vel_cl),
            "top_speed_ms": float(np.max(lap.vel_cl)), "track_length_from_splines_m": float(tr.dists_cl[-1])}


def compare(k: dict, ref: dict, lim_t: float, lim_v: float) -> dict:
    rows, ok = {}, True
    rel = k["lap_time_s"] / ref["lap_time_s"] - 1
    rows["lap_time_s"] = {"kinematik": k["lap_time_s"], "reference": ref["lap_time_s"], "rel": rel,
                          "passes": bool(abs(rel) <= lim_t)}
    ok &= rows["lap_time_s"]["passes"]
    for c, vk in k["corner_min_speed_ms"].items():
        vr = ref["corner_min_speed_ms"][c]
        r = vk / vr - 1
        rows[f"min_speed_{c}_ms"] = {"kinematik": vk, "reference": vr, "rel": r, "passes": bool(abs(r) <= lim_v)}
        ok &= rows[f"min_speed_{c}_ms"]["passes"]
    rows["passed"] = bool(ok)
    return rows


def run(tum_path: str | None = None) -> dict:
    lim = REG["limits"]
    strip = lambda d: {q: x for q, x in d.items() if not q.startswith("_")}   # traces stay out of the stored file
    k, e = strip(kinematik()), strip(exact())
    out = {"kinematik": k, "exact": e,
           "vs_exact": compare(k, e, lim["lap_time_vs_exact_relative"], lim["corner_min_speed_vs_exact_relative"])}
    if tum_path:
        t = tum(tum_path)
        out["tum"] = t
    else:
        stored = json.loads((BENCH / "results.json").read_text())
        t = out["tum"] = stored["tum"]
    out["vs_tum"] = compare(k, t, lim["lap_time_vs_tum_relative"], lim["corner_min_speed_vs_tum_relative"])
    out["passed"] = bool(out["vs_exact"]["passed"] and out["vs_tum"]["passed"])
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", default=os.environ.get("TUM_LAPTIMESIM"))
    res = run(ap.parse_args().tum)
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for ref in ("tum", "exact"):
        for q, r in res[f"vs_{ref}"].items():
            if q != "passed":
                print(f"vs {ref:5s} {q:20s} KinematiK {r['kinematik']:8.3f}  ref {r['reference']:8.3f}  "
                      f"{100 * r['rel']:+6.2f} % {'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
