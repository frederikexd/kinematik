# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The through-thickness rotor model (suspension/brake_thermal.py, OneDRotor) against measured disc temperatures on a
full-scale brake dynamometer (Yevtushenko et al., Materials 2022). Limits: benchmarks/brake_rotor_measured/limits.json.

    python scripts/brake_rotor_measured_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import brake_thermal as bt  # noqa: E402

BENCH = ROOT / "benchmarks" / "brake_rotor_measured"
REG = json.loads((BENCH / "limits.json").read_text())
INP = REG["source"]["inputs"]
R_TC = 0.253 + 0.040            # the hottest thermocouple's radius, m
DT = 0.01


def flux_series(tc: dict) -> list[float]:
    """Heat flux into one disc face at R_TC, W/m2, every DT: eqs. (4), (11), (12), (15) of the paper."""
    p0, V0, Rw, eta = INP["p0_MPa"] * 1e6, INP["V0_kmh"] / 3.6, INP["Rw_m"], 0.25
    ti, t0s, ts, f, gam = tc["ti_s"], tc["t0s_s"], tc["ts_s"], tc["f"], tc["gamma"]
    out, t = [], 0.0
    while t < ts:
        pstar = t / ti if t < ti else 1.0
        integ = t * t / (2 * ti) if t < ti else ti / 2 + (t - ti)
        V = max(V0 * (1 - integ / t0s), 0.0)
        out.append(eta * gam * f * p0 * pstar * V * R_TC / Rw)
        t += DT
    return out


def kinematik(tc: dict) -> dict:
    mat = bt.RotorMaterial("cast iron (paper)", 7100.0, 500.0, 51.0, 700.0, 0.0)
    prm = bt.TwoNodeParams(diameter_mm=640.0, thickness_mm=55.0, pad_heat_fraction=0.0, T_ambient_c=tc["T0_C"],
                           enable_radiation=False)
    rotor = bt.OneDRotor(prm, n_nodes=111, rotor_material=mat)           # 0.25 mm nodes
    q = flux_series(tc)
    P = [2.0 * rotor.face_area * x for x in q]                           # W into the rotor, both faces
    r = rotor.simulate(P, DT, h_w_m2K=0.0, record_depth_mm=1.0)
    i = max(range(len(r.T_depth_c)), key=r.T_depth_c.__getitem__)
    return {"max_1mm_C": r.T_depth_c[i], "t_of_max_s": r.t_s[i], "max_surface_C": r.T_surface_peak_c}


def exact_1mm(tc: dict, half_thickness_m: float | None) -> float:
    """Exact maximum at 1 mm for the same flux: Duhamel superposition of the step-flux solution
    (Carslaw & Jaeger 2.9), on a semi-infinite body (None) or on a slab with an insulated mid-plane (method of
    images). A check on the solver, independent of its grid. °C."""
    from math import erfc, exp, pi, sqrt
    k = 51.0
    kap = k / (7100.0 * 500.0)
    q = flux_series(tc)
    dq = [q[0]] + [q[j] - q[j - 1] for j in range(1, len(q))]

    def step(s, z):
        a = sqrt(kap * s)
        return (2 / k) * (a / sqrt(pi) * exp(-z * z / (4 * a * a)) - z / 2 * erfc(z / (2 * a)))

    def resp(s):
        if s <= 0:
            return 0.0
        if half_thickness_m is None:
            return step(s, 1e-3)
        L = half_thickness_m
        return sum(step(s, 2 * m * L + 1e-3) + step(s, 2 * (m + 1) * L - 1e-3) for m in range(6))
    best = 0.0
    for i in range(0, len(q) + 1, 20):
        best = max(best, sum(dq[j] * resp((i - j) * DT) for j in range(i)))
    return tc["T0_C"] + best


def run() -> dict:
    lim = REG["limits"]
    out, ok = {}, True
    for name in ("test_1", "test_2"):
        tc = INP[name]
        k = kinematik(tc)
        dm = k["max_1mm_C"] - tc["measured_max_C"]
        dp = k["max_1mm_C"] - tc["paper_model_max_C"]
        p = bool(abs(dm) <= lim["vs_measured_C"] and abs(dp) <= lim["vs_paper_model_C"])
        out[name] = {**k, "measured_max_C": tc["measured_max_C"], "paper_model_max_C": tc["paper_model_max_C"],
                     "diff_vs_measured_C": dm, "diff_vs_paper_model_C": dp, "passes": p,
                     # added after the first run, not pass/fail: what the difference from the paper's model is
                     "diagnostic_exact_finite_slab_C": exact_1mm(tc, 0.0275),
                     "diagnostic_exact_semi_infinite_C": exact_1mm(tc, None)}
        ok = ok and p
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for n in ("test_1", "test_2"):
        r = res[n]
        print(f"{n}: KinematiK {r['max_1mm_C']:.1f} C at {r['t_of_max_s']:.1f} s; measured {r['measured_max_C']} "
              f"({r['diff_vs_measured_C']:+.1f}); paper's model {r['paper_model_max_C']} ({r['diff_vs_paper_model_C']:+.1f}) "
              f"{'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
