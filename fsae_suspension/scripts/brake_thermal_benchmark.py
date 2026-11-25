# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The 1-D brake rotor conduction model (OneDRotor, suspension/brake_thermal.py) against Ansys VM110.

VM110 is a slab heated by convection from a hot gas on one face and insulated on the other, the same boundary
conditions as one half of a rotor heated on its friction face with a symmetric centre plane. Problem, mapping and
limits: benchmarks/brake_thermal/limits.json, registered before the run. Results: benchmarks/brake_thermal/results.json.

    python scripts/brake_thermal_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import brake_thermal as bt  # noqa: E402

BENCH = ROOT / "benchmarks" / "brake_thermal"
REG = json.loads((BENCH / "limits.json").read_text())
FT, HR = 0.3048, 3600.0
BTU_HR_FT_F = 1.730735          # W/(m K)
BTU_HR_FT2_F = 5.678263         # W/(m2 K)
BTU_LB_F = 4186.8               # J/(kg K)
LB_FT3 = 16.018463              # kg/m3
BTU_FT2 = 11356.53              # J/m2


def vm110() -> dict:
    d = REG["problem"]["data_us"]
    mat = bt.RotorMaterial("VM110 slab", d["rho_lb_ft3"] * LB_FT3, d["c_btu_lb_F"] * BTU_LB_F,
                           d["k_btu_hr_ft_F"] * BTU_HR_FT_F, 1e9, 0.0)
    p = bt.TwoNodeParams(thickness_mm=2 * d["thickness_ft"] * FT * 1000.0, area_factor=2.0,
                         enable_radiation=False, T_ambient_c=0.0, pad_heat_fraction=0.0)
    rotor = bt.OneDRotor(p, n_nodes=11, rotor_material=mat)
    theta0 = (d["T_gas_F"] - d["T_initial_F"]) * 5.0 / 9.0          # K above the gas, cooling form
    t_end = d["t_hr"] * HR
    n = 290
    tr = rotor.simulate([0.0] * n, t_end / n, T_initial_c=theta0, h_w_m2K=d["h_btu_hr_ft2_F"] * BTU_HR_FT2_F)
    T = np.asarray(tr.T_field_c)                                      # cooling form, convective face first
    temps_F = d["T_gas_F"] - T * 9.0 / 5.0
    depth = np.linspace(1.0, 0.0, 11)                                 # ft from the insulated face
    out = {f"{x:.1f}": float(t) for x, t in zip(depth, temps_F)}
    # heat taken in: the rise of the stored energy over the slab (trapezoid on the 0.1 ft grid)
    rise = theta0 - T
    q = mat.rho_kg_m3 * mat.cp_j_kgK * np.trapezoid(rise, dx=rotor.dx)
    return {"temps_F": out, "heat_in_btu_ft2": float(q / BTU_FT2)}


def exact() -> dict:
    """The exact series solution of the same slab (Carslaw & Jaeger; eigenvalues of z tan z = Bi), added after the
    first run showed the manual's targets are graphical estimates. Temperatures in F by depth, heat in BTU/ft2."""
    from scipy.optimize import brentq
    d = REG["problem"]["data_us"]
    L, a = d["thickness_ft"], d["k_btu_hr_ft_F"] / (d["rho_lb_ft3"] * d["c_btu_lb_F"])
    Bi, Fo = d["h_btu_hr_ft2_F"] * L / d["k_btu_hr_ft_F"], a * d["t_hr"] / L ** 2
    x = np.linspace(0.0, 1.0, 11)
    th, frac = np.zeros_like(x), 0.0
    for nn in range(80):
        z = brentq(lambda q: q * np.tan(q) - Bi, nn * np.pi + 1e-9, (nn + 0.5) * np.pi - 1e-9)
        C = 4 * np.sin(z) / (2 * z + np.sin(2 * z))
        th += C * np.exp(-z * z * Fo) * np.cos(z * x)
        frac += C * np.sin(z) / z * np.exp(-z * z * Fo)
    dT0 = d["T_gas_F"] - d["T_initial_F"]
    T = d["T_gas_F"] - dT0 * th
    return {"temps_F": {f"{xx:.1f}": float(t) for xx, t in zip(x, T)},
            "heat_in_btu_ft2": float(d["rho_lb_ft3"] * d["c_btu_lb_F"] * L * dT0 * (1 - frac))}


def run() -> dict:
    r = vm110()
    P = REG["problem"]
    ok = True
    rows = {}
    for x, tgt in P["target_F_by_depth_ft_from_insulated_face"].items():
        k = r["temps_F"][x]
        passes = bool(abs(k - tgt) <= REG["limit"]["temperature_F"])
        rows[x] = {"kinematik_F": k, "target_F": tgt, "diff_F": k - tgt, "passes": passes}
        ok = ok and passes
    q, qt = r["heat_in_btu_ft2"], P["target_heat_in_btu_ft2"]
    heat_ok = bool(abs(q / qt - 1) <= REG["limit"]["heat_relative"])
    ex = exact()
    apdl = {"0.0": 507.0, "0.2": 549.0, "0.4": 675.0, "0.6": 874.0, "0.8": 1134.0, "1.0": 1433.0}
    vs_exact = {x: {"kinematik_F": r["temps_F"][x], "exact_F": ex["temps_F"][x],
                    "kinematik_diff_F": r["temps_F"][x] - ex["temps_F"][x],
                    "mechanical_apdl_diff_F": apdl[x] - ex["temps_F"][x]} for x in apdl}
    return {"temperatures": rows, "heat": {"kinematik": q, "target": qt, "rel": q / qt - 1, "passes": heat_ok},
            "passed": bool(ok and heat_ok),
            "note": "The manual calls its targets graphical estimates from Kreith. The exact series solution below was "
                    "added after the first run, not registered before it.",
            "exact": {"temperatures": vs_exact, "heat_btu_ft2": ex["heat_in_btu_ft2"],
                      "kinematik_heat_rel": q / ex["heat_in_btu_ft2"] - 1,
                      "mechanical_apdl_heat_rel": 20662.0 / ex["heat_in_btu_ft2"] - 1}}


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for x, r in res["temperatures"].items():
        print(f"depth {x} ft: KinematiK {r['kinematik_F']:7.1f} F  Kreith/Ansys {r['target_F']:5d} F  "
              f"{r['diff_F']:+6.1f}  {'PASS' if r['passes'] else 'FAIL'}")
    h = res["heat"]
    print(f"heat in: {h['kinematik']:.0f} BTU/ft2 vs {h['target']} ({100 * h['rel']:+.2f} %) {'PASS' if h['passes'] else 'FAIL'}")
    e = res["exact"]
    print("against the exact series solution (added after the run):")
    for x, r in e["temperatures"].items():
        print(f"  depth {x} ft: exact {r['exact_F']:7.1f} F  KinematiK {r['kinematik_diff_F']:+5.1f}  "
              f"Mechanical APDL {r['mechanical_apdl_diff_F']:+5.1f}")
    print(f"  heat: exact {e['heat_btu_ft2']:.0f}  KinematiK {100 * e['kinematik_heat_rel']:+.2f} %  "
          f"Mechanical APDL {100 * e['mechanical_apdl_heat_rel']:+.2f} %")
    print("PASSED" if res["passed"] else "FAILED against the registered (graphical) targets")
    sys.exit(0 if res["passed"] else 1)
