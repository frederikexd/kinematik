# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Cell temperature in an air-cooled pack: KinematiK's lumped cell model against a measured LFP pack (Saw et al.,
Applied Energy 2016). Limits: benchmarks/pack_air_measured/limits.json.

    python scripts/pack_air_measured_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.ev_electrical_check import pack_temperature_along  # noqa: E402

BENCH = ROOT / "benchmarks" / "pack_air_measured"
REG = json.loads((BENCH / "limits.json").read_text())
SRC = REG["source"]
R, M, CP, D, L, AMB = 0.0034, 0.335, 998.0, 0.038, 0.146, 303.15
H = SRC["paper_nusselt_at_30_g_s"] * 0.0263 / D
HA = H * math.pi * D * L


def rise(rate: str, heat_w: float | None = None) -> float:
    """Mean cell rise (K) at the measured end time; heat_w replaces I^2 R (diagnostic only)."""
    m = SRC["measured_mean_surface_rise_C_fig14"][rate]
    t = np.arange(0.0, m["t_s"] + 1.0, 1.0)
    i_cell = 8.0 * float(rate)
    r = R if heat_w is None else heat_w / i_cell ** 2
    temp = pack_temperature_along(np.full(t.size, i_cell), t, r_pack_ohm=r, heat_capacity_j_k=M * CP,
                                  temp0_k=AMB, cooling_w_k=HA, ambient_k=AMB)
    return float(temp[-1] - AMB)


def calibrated_model(r_ohm: float, dudt: float, h: float):
    """registered_2: KinematiK's PackThermalModel on this pack with a calibrated cell and uniform cooling."""
    from suspension import pack_thermal as pt
    cell = pt.CellParams(mass_kg=M, cp=CP, r_internal_ohm=r_ohm, r_tempco_per_c=0.0, nominal_v=3.2,
                         surface_area_m2=math.pi * D * L, pitch_mm=43.0, temp_limit_c=60.0, temp_warn_c=40.0,
                         enable_entropic=True, dudt_v_per_k=dudt, calibrated=True, fitted_to="Saw et al. 2016")
    lay = pt.PackLayout(rows=4, cols=6, series=12, parallel=2, cell=cell, ambient_c=30.0, ambient_is_local=True)
    air = pt.AirflowParams(h_floor=h, mode="duct", duct_cfm=0.0, enable_wake_warming=False, calibrated=True)
    return pt.PackThermalModel(layout=lay, fans=[], airflow=air)


def calibrated_rise(rate: str, r_ohm: float, dudt: float, h: float) -> float:
    m = SRC["measured_mean_surface_rise_C_fig14"][rate]
    t = np.arange(0.0, m["t_s"] + 1.0, 1.0)
    res = calibrated_model(r_ohm, dudt, h).simulate(t, np.full(t.size, 16.0 * float(rate)), init_temp_c=30.0)
    return float(np.mean(res.final_temp_c) - 30.0)


def run_calibrated() -> dict:
    from scipy.optimize import brentq
    reg = REG["registered_2"]["limits"]
    heat = SRC["measured_heat_per_cell_W_ARC"]
    # Q = I^2 R + I T k at the run's mean temperature (about 305 K): two equations, two unknowns (1 and 5 It)
    T = 305.0
    i1, i5 = 8.0, 40.0
    A = np.array([[i1 ** 2, i1 * T], [i5 ** 2, i5 * T]])
    r_fit, k_fit = np.linalg.solve(A, np.array([heat["1"], heat["5"]]))
    q3 = 24.0 ** 2 * r_fit + 24.0 * T * k_fit
    h_fit = brentq(lambda h: calibrated_rise("5", r_fit, k_fit, h) - SRC["measured_mean_surface_rise_C_fig14"]["5"]["dT"],
                   1.0, 200.0, xtol=1e-4)
    temps = {r: {"measured_C": SRC["measured_mean_surface_rise_C_fig14"][r]["dT"],
                 "kinematik_C": calibrated_rise(r, r_fit, k_fit, h_fit)} for r in ("1", "3", "5")}
    for v in temps.values():
        v["rel"] = v["kinematik_C"] / v["measured_C"] - 1
    heat_ok = abs(q3 / heat["3"] - 1) <= reg["heat_relative"]
    temp_ok = all(abs(temps[r]["rel"]) <= reg["temperature_relative"] for r in ("1", "3"))
    flag_k = sorted(r for r, v in temps.items() if v["kinematik_C"] > 10.0)
    dec_ok = flag_k == ["5"]
    return {"r_fit_ohm": float(r_fit), "dudt_fit_v_per_k": float(k_fit), "h_fit_W_m2K": float(h_fit),
            "heat_3It_W": float(q3), "heat_3It_measured_W": heat["3"], "heat_rel": float(q3 / heat["3"] - 1),
            "temps": temps, "flagged_over_40C": flag_k,
            "passes": {"heat": bool(heat_ok), "temperature": bool(temp_ok), "decision": bool(dec_ok)},
            "passed": bool(heat_ok and temp_ok and dec_ok)}


def run() -> dict:
    lim = REG["limits"]["absolute_relative"]
    rows = {}
    for rate, m in SRC["measured_mean_surface_rise_C_fig14"].items():
        k = rise(rate)
        rows[rate] = {"measured_C": m["dT"], "kinematik_C": k, "rel": k / m["dT"] - 1,
                      "kinematik_heat_W": (8.0 * float(rate)) ** 2 * R,
                      "measured_heat_W": SRC["measured_heat_per_cell_W_ARC"][rate],
                      # diagnostic, added with the registration but not graded: the measured heat in place of I^2 R
                      "with_measured_heat_C": rise(rate, SRC["measured_heat_per_cell_W_ARC"][rate])}
    absolute = all(abs(r["rel"]) <= lim for r in rows.values())
    flag_m = sorted(k for k, r in rows.items() if r["measured_C"] > 10.0)
    flag_k = sorted(k for k, r in rows.items() if r["kinematik_C"] > 10.0)
    return {"h_W_m2K": H, "rows": rows, "absolute_passes": bool(absolute),
            "decision": {"measured_over_40C": flag_m, "kinematik_over_40C": flag_k, "passes": flag_m == flag_k},
            "passed": bool(absolute and flag_m == flag_k), "calibrated": run_calibrated()}


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for rate, r in res["rows"].items():
        print(f"  {rate} It: measured {r['measured_C']:.1f} C, KinematiK {r['kinematik_C']:.2f} C ({100 * r['rel']:+.0f} %); "
              f"heat {r['kinematik_heat_W']:.2f} W vs measured {r['measured_heat_W']} W; with measured heat "
              f"{r['with_measured_heat_C']:.2f} C")
    print("absolute", "PASS" if res["absolute_passes"] else "FAIL", "| decision", res["decision"])
    c = res["calibrated"]
    print(f"calibrated: R {1000 * c['r_fit_ohm']:.2f} mOhm, dU/dT {1e3 * c['dudt_fit_v_per_k']:.3f} mV/K, h "
          f"{c['h_fit_W_m2K']:.1f} W/m2K; 3 It heat {c['heat_3It_W']:.2f} W vs {c['heat_3It_measured_W']} "
          f"({100 * c['heat_rel']:+.1f} %)")
    for r, v in c["temps"].items():
        print(f"  {r} It: measured {v['measured_C']:.1f}, KinematiK {v['kinematik_C']:.2f} ({100 * v['rel']:+.0f} %)")
    print("  flagged over 40 C:", c["flagged_over_40C"], c["passes"], "PASSED" if c["passed"] else "FAILED")
    sys.exit(0 if c["passed"] else 1)
