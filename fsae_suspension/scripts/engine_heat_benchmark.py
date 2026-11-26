# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The ICE heat-to-coolant default (ice_powertrain.fuel_for_lap: coolant_frac, bte) against a measured heat balance
of a water-cooled SI engine (benchmarks/engine_heat/limits.json, which says the data was read before the criterion
was written).

    python scripts/engine_heat_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import inspect
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import ice_powertrain as ice  # noqa: E402

BENCH = ROOT / "benchmarks" / "engine_heat"
REG = json.loads((BENCH / "limits.json").read_text())


def run() -> dict:
    p = inspect.signature(ice.fuel_for_lap).parameters
    cf, bte = float(p["coolant_frac"].default), float(p["bte"].default)
    pts = [r for rows in REG["data"].values() for r in rows]
    cool = [r[2] / 100 for r in pts]
    brake = [r[1] / 100 for r in pts]
    full = [r[2] / 100 for k, rows in REG["data"].items() if k.startswith("full") for r in rows]
    half = [r[2] / 100 for k, rows in REG["data"].items() if k.startswith("half") for r in rows]
    mean = lambda x: sum(x) / len(x)
    out = {"kinematik": {"coolant_frac": cf, "bte": bte},
           "measured": {"coolant_range": [min(cool), max(cool)], "coolant_mean_full_load": mean(full),
                        "coolant_mean_half_load": mean(half), "bte_range": [min(brake), max(brake)]},
           "bias": {"coolant_vs_full_load_mean": cf / mean(full) - 1, "coolant_vs_half_load_mean": cf / mean(half) - 1}}
    out["checks"] = {"coolant_frac_within_measured_range": min(cool) <= cf <= max(cool),
                     "bte_within_measured_range": min(brake) <= bte <= max(brake)}
    out["passed"] = bool(all(out["checks"].values()))
    return out


# Phase 2 (criterion registered blind in limits.json["phase_2"]): Korczak, MSc thesis, University of Illinois 2009,
# Honda F4i on the 2009 Illinois FSAE car. Table 4.1: coolant flow measured at each engine speed (FloCat meter)
# times one radiator temperature drop, 5.67 C, the mean over driving; fuel at wide-open throttle 7.8 kg/h at
# 45.7 MJ/kg = 99.02 kW (sec. 4.3). DISCLOSURE: these numbers were read before the mapping below was chosen; the
# limit (30 %) was not changed. Only the points where the fuel flow applies (the top of the rev range, wide open
# throttle) are graded.
KORCZAK_TABLE_4_1 = [(7000, 28.97, 11.46), (8000, 32.59, 12.89), (9000, 40.11, 15.87), (10000, 47.40, 18.75),
                     (11000, 52.14, 20.63), (12000, 52.14, 20.63)]   # rpm, L/min, kW
KORCZAK_FUEL_KW = 7.8 / 3600 * 45.7e3


def phase_2() -> dict:
    cf = float(inspect.signature(ice.fuel_for_lap).parameters["coolant_frac"].default)
    k = cf * KORCZAK_FUEL_KW
    lim = REG["phase_2"]["limit_relative"]
    graded = [{"rpm": r, "measured_kW": q, "kinematik_kW": k, "rel": k / q - 1} for r, _, q in KORCZAK_TABLE_4_1
              if r >= 11000]
    return {"source": "Korczak, Cost Effective Method for the Evaluation of a Formula SAE Engine Cooling System, MSc "
                      "thesis, University of Illinois at Urbana-Champaign, 2009 (Table 4.1, sec. 4.3)",
            "caveat": "the measured heat is coolant flow at each engine speed times one mean radiator temperature drop "
                      "from driving, not a dyno heat balance at wide-open throttle; the thesis reports the car "
                      "overheating, so the engine at times rejected more than this",
            "fuel_power_kW": KORCZAK_FUEL_KW, "graded": graded,
            "passed": bool(all(abs(g["rel"]) <= lim for g in graded))}


if __name__ == "__main__":
    if "--phase-2" in sys.argv:
        r2 = phase_2()
        (BENCH / "results_phase2.json").write_text(json.dumps(r2, indent=1))
        for g in r2["graded"]:
            print(f"{g['rpm']} rpm: KinematiK {g['kinematik_kW']:.1f} kW, measured {g['measured_kW']:.1f} kW, "
                  f"{100 * g['rel']:+.1f} %")
        print("PASSED" if r2["passed"] else "FAILED")
        sys.exit(0 if r2["passed"] else 1)
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    m, k, b = res["measured"], res["kinematik"], res["bias"]
    print(f"coolant share: KinematiK {k['coolant_frac']:.2f}; measured {m['coolant_range'][0]:.3f}-{m['coolant_range'][1]:.3f}, "
          f"mean {m['coolant_mean_full_load']:.3f} full load ({100 * b['coolant_vs_full_load_mean']:+.0f} %), "
          f"{m['coolant_mean_half_load']:.3f} half load ({100 * b['coolant_vs_half_load_mean']:+.0f} %)")
    print(f"BTE: KinematiK {k['bte']:.2f}; measured {m['bte_range'][0]:.3f}-{m['bte_range'][1]:.3f}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
