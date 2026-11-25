# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The ICE gearbox chain (suspension/ice_powertrain.py: EngineMap, Gearbox) against TUM's published lap time
simulation's combustion car model. Limits: benchmarks/gearbox/limits.json, registered before the run.

    python scripts/gearbox_benchmark.py --tum /path/to/laptime-simulation      # exit 1 on any failure

Without --tum the TUM values are read from the stored results and only KinematiK is rerun.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.ice_powertrain import EngineMap, Gearbox  # noqa: E402

BENCH = ROOT / "benchmarks" / "gearbox"
REG = json.loads((BENCH / "limits.json").read_text())
PRIMARY, RATIOS, FINAL, EFF, SHIFT_RPM = 2.111, (2.750, 2.000, 1.667, 1.444, 1.304, 1.208), 3.0, 0.92, 13500.0
CIRC_REF = 2 * math.pi * 0.228
SPEEDS = [3.0 + 0.5 * k for k in range(75)]


def tum(path: str) -> dict:
    sys.path.insert(0, path)
    for name, typ in (("int", int), ("float", float), ("bool", bool)):
        if name not in np.__dict__:
            setattr(np, name, typ)
    from laptimesim.src.car_hybrid import CarHybrid
    over = [PRIMARY * r * FINAL for r in RATIOS]
    veh = {"powertrain_type": "hybrid",
           "general": {"lf": 0.8, "lr": 0.75, "h_cog": 0.28, "sf": 1.2, "sr": 1.2, "m": 300.0, "f_roll": 0.015,
                       "c_w_a": 1.1, "c_z_a_f": 1.2, "c_z_a_r": 1.3, "g": 9.81, "rho_air": 1.225, "drs_factor": 0.0},
           "engine": {"topology": "RWD", "pow_max": 65e3, "pow_diff": 13e3, "n_begin": 8000.0, "n_max": 12500.0,
                      "n_end": 14000.0, "be_max": 20.0, "pow_e_motor": 0.0, "eta_e_motor": 0.9, "eta_e_motor_re": 0.15,
                      "eta_etc_re": 0.10, "vel_min_e_motor": 100.0, "torque_e_motor_max": 0.0},
           "gearbox": {"i_trans": [1.0 / o for o in over], "n_shift": [SHIFT_RPM] * len(over),
                       "e_i": [1.0] * len(over), "eta_g": EFF},
           "tires": {"f": {"circ_ref": CIRC_REF, "fz_0": 700.0, "mux": 1.5, "muy": 1.5, "dmux_dfz": 0.0, "dmuy_dfz": 0.0},
                     "r": {"circ_ref": CIRC_REF, "fz_0": 700.0, "mux": 1.5, "muy": 1.5, "dmux_dfz": 0.0, "dmuy_dfz": 0.0},
                     "tire_model_exp": 2.0}}
    f = Path(tempfile.mkdtemp()) / "veh.ini"
    f.write_text("[VEH_PARS]\nveh_pars=" + json.dumps(veh) + "\n")
    car = CarHybrid(parfilepath=str(f))

    def torque(n_per_s: float) -> float:
        # CarHybrid.torque is float(power) / (2 pi n); under NumPy 2 float() of a 1-element array raises, so the
        # same expression is evaluated here on TUM's own power curve
        return float(np.ravel(car._CarHybrid__power_engine(n=np.array([n_per_s])))[0]) / (2 * math.pi * n_per_s)
    rpm_grid = np.arange(2000.0, 16000.0 + 1, 50.0)
    curve = [torque(n / 60.0) for n in rpm_grid]
    rows = []
    for v in SPEEDS:
        g, n = car.find_gear(vel=v)
        r = car.r_driven_tire(vel=v)
        F = torque(float(n)) * car.pars_gearbox["eta_g"] / (r * car.pars_gearbox["i_trans"][g]
                                                                     * car.pars_gearbox["e_i"][g])
        rows.append({"v_ms": v, "radius_m": r, "gear": int(g) + 1, "rpm": float(n) * 60.0, "force_N": float(F)})
    return {"rows": rows, "torque_curve": {"rpm": rpm_grid.tolist(), "torque_nm": [float(t) for t in curve]}}


def kinematik(ref: dict) -> list[dict]:
    tc = ref["torque_curve"]
    eng = EngineMap(rpm=tc["rpm"], torque_nm=tc["torque_nm"], redline_rpm=16000.0, idle_rpm=2000.0)
    out = []
    for row in ref["rows"]:
        v, r = row["v_ms"], row["radius_m"]
        base = Gearbox(primary=PRIMARY, ratios=RATIOS, final_drive=FINAL, efficiency=EFF, tire_radius_m=r)
        up = [base.speed_at(SHIFT_RPM, g) * 3.6 for g in range(len(RATIOS) - 1)]
        gb = Gearbox(primary=PRIMARY, ratios=RATIOS, final_drive=FINAL, efficiency=EFF, tire_radius_m=r,
                     upshift_kmh=tuple(up))
        g = sum(1 for s in gb.shift_speeds(eng) if v >= s)          # gears change at the shift speeds
        out.append({"v_ms": v, "gear": g + 1, "rpm": gb.rpm_at(v, g), "force_N": gb.wheel_force(v, g, eng)})
    return out


def run(tum_path: str | None = None) -> dict:
    lim = REG["limits"]
    ref = tum(tum_path) if tum_path else json.loads((BENCH / "results.json").read_text())["tum"]
    k = kinematik(ref)
    rows, mism, ok = [], 0, True
    for a, b in zip(k, ref["rows"]):
        same = a["gear"] == b["gear"]
        mism += 0 if same else 1
        rr = a["rpm"] / b["rpm"] - 1
        fr = a["force_N"] / b["force_N"] - 1
        p = bool(same and abs(rr) <= lim["rpm_relative"] and abs(fr) <= lim["force_relative"])
        ok = ok and p
        rows.append({"v_ms": a["v_ms"], "gear_k": a["gear"], "gear_tum": b["gear"], "rpm_rel": rr, "force_rel": fr,
                     "passes": p})
    return {"tum": ref, "comparison": rows, "gear_mismatches": mism,
            "worst_force_rel": max(abs(r["force_rel"]) for r in rows),
            "worst_rpm_rel": max(abs(r["rpm_rel"]) for r in rows), "passed": bool(ok and mism <= lim["gear_mismatches"])}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", default=os.environ.get("TUM_LAPTIMESIM"))
    res = run(ap.parse_args().tum)
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    print(f"gear mismatches {res['gear_mismatches']}  worst rpm {res['worst_rpm_rel']:.1e}  "
          f"worst force {100 * res['worst_force_rel']:.4f} %")
    for r in res["comparison"]:
        if not r["passes"]:
            print("  FAIL", r)
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
