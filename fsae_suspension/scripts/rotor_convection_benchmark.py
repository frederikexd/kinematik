# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""KinematiK's rotor convection (suspension/brake_thermal.py: h_rotating_disc, convective_area_m2) against STAR-CCM+
CFD of a ventilated disc on a brake dynamometer (Chandrasekar, Chalmers 2026). Limits:
benchmarks/rotor_convection/limits.json.

    python scripts/rotor_convection_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import brake_thermal as bt  # noqa: E402

BENCH = ROOT / "benchmarks" / "rotor_convection"
REG = json.loads((BENCH / "limits.json").read_text())
T_AIR = 20.0
D_MM, R_DYN_MM = 366.0, 372.5
SURF = ["VC", "IFS", "OFS", "ISN", "OSN", "IHI", "OHI", "ROS"]
# Table A.1, HTC W/m2K at 180 km/h, by surface temperature C
HTC = {30: [78.03, 63.25, 42.04, 44.11, 50.24, 66.32, 36.88, 65.49],
       100: [77.93, 60.81, 38.73, 39.89, 47.71, 68.33, 35.27, 63.50],
       200: [78.33, 58.22, 36.32, 37.82, 44.80, 67.51, 33.37, 60.02],
       300: [77.09, 55.19, 37.14, 37.70, 43.70, 70.88, 32.01, 58.31],
       400: [77.12, 53.71, 35.68, 35.63, 40.23, 66.79, 29.45, 55.43],
       500: [75.42, 51.44, 33.48, 34.33, 39.60, 67.84, 31.13, 52.89],
       600: [72.00, 49.14, 31.80, 32.42, 37.73, 66.79, 30.24, 50.42],
       700: [69.53, 48.14, 30.06, 29.69, 35.43, 65.79, 26.81, 48.66],
       800: [65.78, 45.37, 29.47, 28.33, 35.64, 69.98, 28.23, 44.49]}
# Table A.3, heat W at 180 km/h
Q = {30: [98.74, 38.98, 6.33, 15.70, 29.33, 1.94, 12.60, 15.34],
     100: [788.87, 299.83, 46.68, 113.60, 222.81, 16.02, 96.40, 118.98],
     200: [1784.12, 645.93, 98.50, 242.31, 470.75, 35.61, 205.22, 253.06],
     300: [2731.21, 952.37, 156.70, 375.75, 714.32, 58.17, 306.21, 382.42],
     400: [3708.27, 1258.00, 204.31, 481.91, 892.49, 74.40, 382.39, 493.34],
     500: [4580.93, 1521.71, 242.17, 586.64, 1109.67, 95.45, 510.56, 594.65],
     600: [5284.16, 1756.46, 277.93, 669.38, 1277.44, 113.55, 599.18, 684.91],
     700: [5982.75, 2017.73, 308.02, 718.68, 1406.54, 131.13, 622.91, 775.03],
     800: [6492.13, 2181.18, 346.36, 786.58, 1622.91, 159.99, 752.42, 812.88]}
# Table A.6, HTC W/m2K at 500 C, clockwise: VC, IFS, OFS by vehicle speed km/h
SPEED = {30: [19.21, 18.33, 14.28], 80: [42.95, 30.14, 23.12], 130: [59.70, 40.96, 31.45],
         180: [75.42, 51.44, 39.60], 230: [88.48, 60.34, 45.81], 280: [100.65, 68.45, 52.70]}


def ring_width_mm() -> float:
    a_ifs = Q[30][1] / (HTC[30][1] * (30 - T_AIR))
    r_out = D_MM / 2000.0
    return (r_out - math.sqrt(r_out ** 2 - a_ifs / math.pi)) * 1000.0


def k_h(v_kmh: float) -> float:
    return bt.h_rotating_disc(D_MM, v_kmh / 3.6, wheel_radius_mm=R_DYN_MM, crossflow_ms=0.0)


def run() -> dict:
    lim = REG["limits"]
    w = ring_width_mm()
    A = bt.convective_area_m2(D_MM, hat_clearance_mm=w)
    G_k = k_h(180.0) * A
    t1 = []
    for T in range(100, 801, 100):
        G_cfd = sum(Q[T]) / (T - T_AIR)
        t1.append({"T_C": T, "cfd_W_per_K": G_cfd, "kinematik_W_per_K": G_k, "rel": G_k / G_cfd - 1})
    areas = [Q[500][i] / (HTC[500][i] * (500 - T_AIR)) for i in range(3)]
    mean = {v: sum(a * h for a, h in zip(areas, hs)) / sum(areas) for v, hs in SPEED.items()}
    t2 = []
    for v in SPEED:
        c, k = mean[v] / mean[180], k_h(v) / k_h(180.0)
        t2.append({"v_kmh": v, "cfd_ratio": c, "kinematik_ratio": k, "rel": k / c - 1})
    p1 = all(abs(r["rel"]) <= lim["test_1_relative"] for r in t1)
    p2 = all(abs(r["rel"]) <= lim["test_2_relative"] for r in t2)
    A_cfd = sum(Q[30][i] / (HTC[30][i] * 10) for i in range(8))
    # added after the first run, not pass/fail: split the miss into the coefficient and the area
    diag = {"cfd_mean_h_500C_W_m2K": sum(Q[500]) / (500 - T_AIR) / A_cfd, "kinematik_h_W_m2K": k_h(180.0),
            "cfd_area_over_face_annulus": A_cfd / (A / bt.TwoNodeParams().area_factor),
            "kinematik_area_factor_default": bt.TwoNodeParams().area_factor}
    # rerun added after the registered run (not blind): the wetted area from CAD, as a team would give it for a vented
    # rotor (here the thesis's own surfaces), in place of the default area factor; same limits
    Aw = bt.convective_area_m2(D_MM, wetted_area_m2=A_cfd)
    rw = [{"T_C": r["T_C"], "cfd_W_per_K": r["cfd_W_per_K"], "kinematik_W_per_K": k_h(180.0) * Aw,
           "rel": k_h(180.0) * Aw / r["cfd_W_per_K"] - 1} for r in t1]
    rerun = {"test_1": rw, "test_1_passes": all(abs(r["rel"]) <= lim["test_1_relative"] for r in rw)}
    rerun["passed"] = bool(rerun["test_1_passes"] and p2)
    return {"rerun_wetted_area_not_blind": rerun, "diagnostic": diag, "ring_width_mm": w, "A_conv_m2": A, "cfd_total_area_m2": A_cfd,
            "kinematik_h_180_W_m2K": k_h(180.0), "test_1": t1, "test_1_passes": p1,
            "test_2": t2, "test_2_passes": p2, "passed": bool(p1 and p2)}


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    print(f"ring width {res['ring_width_mm']:.1f} mm; KinematiK A_conv {res['A_conv_m2']:.4f} m2 (CFD wetted "
          f"{res['cfd_total_area_m2']:.4f}); h(180) {res['kinematik_h_180_W_m2K']:.1f} W/m2K")
    for r in res["test_1"]:
        print(f"  {r['T_C']:4d} C  CFD {r['cfd_W_per_K']:6.2f} W/K  KinematiK {r['kinematik_W_per_K']:6.2f}  {100*r['rel']:+6.1f} %")
    for r in res["test_2"]:
        print(f"  {r['v_kmh']:4d} km/h  CFD {r['cfd_ratio']:.3f}  KinematiK {r['kinematik_ratio']:.3f}  {100*r['rel']:+6.1f} %")
    print("test 1", "PASS" if res["test_1_passes"] else "FAIL", "| test 2", "PASS" if res["test_2_passes"] else "FAIL")
    for r in res["rerun_wetted_area_not_blind"]["test_1"]:
        print(f"  rerun, wetted area: {r['T_C']:4d} C  {100*r['rel']:+6.1f} %")
    print("rerun", "PASS" if res["rerun_wetted_area_not_blind"]["passed"] else "FAIL")
    sys.exit(0 if res["passed"] else 1)
