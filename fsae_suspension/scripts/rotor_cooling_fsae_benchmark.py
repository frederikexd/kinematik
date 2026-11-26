# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""KinematiK's rotor heat loss (suspension/brake_thermal.py: h_rotating_disc plus radiation) against on-track cooling
of a solid FSAE front rotor (Heffernan, SAE 2006-01-1974). Limits: benchmarks/rotor_cooling_fsae/limits.json.

    python scripts/rotor_cooling_fsae_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import brake_thermal as bt  # noqa: E402

BENCH = ROOT / "benchmarks" / "rotor_cooling_fsae"
REG = json.loads((BENCH / "limits.json").read_text())
TS_K, TA_K = 200.0 + 273.15, 25.0 + 273.15


def h_radiation() -> float:
    """KinematiK's radiation, as a coefficient on the same area, W/m2K."""
    p = bt.TwoNodeParams()
    return p.emissivity * p._SIGMA * (TS_K ** 2 + TA_K ** 2) * (TS_K + TA_K)


def run() -> dict:
    src, sw = REG["source"], REG["mapping"]["unknowns_swept"]
    lim = REG["limits"]["relative_each_speed_every_geometry"]
    meas = [(a + b) / 2 for a, b in zip(src["h_measured_biot_W_m2K"], src["h_measured_heat_loss_W_m2K"])]
    hr = h_radiation()
    rows, worst = [], 0.0
    for D in sw["diameter_mm"]:
        for R in sw["wheel_radius_mm"]:
            for v, hm in zip(src["speeds_m_s"], meas):
                hk = bt.h_rotating_disc(D, v, wheel_radius_mm=R, crossflow_ms=v) + hr   # the correlation, as registered
                rel = hk / hm - 1
                worst = max(worst, abs(rel))
                rows.append({"D_mm": D, "R_mm": R, "v_m_s": v, "measured": hm, "kinematik": hk, "rel": rel})
    out = {"h_radiation_W_m2K": hr, "rows": rows, "worst_abs_rel": worst, "passed": bool(worst <= lim)}
    # registered_2: the calibrated path, fitted on three speeds, checked on the other three
    lim2 = REG["registered_2"]["limit_relative_each_held_out_speed"]
    v = src["speeds_m_s"]
    splits = {"odd_to_even": ([0, 2, 4], [1, 3, 5]), "even_to_odd": ([1, 3, 5], [0, 2, 4])}
    cal, ok2 = {}, True
    for name, (fit, test) in splits.items():
        h0, h1 = bt.fit_convection_line([v[i] for i in fit], [meas[i] for i in fit])
        p = bt.BrakeThermalParams(h_air0=h0, h_air1=h1, calibrated=True)
        pts = [{"v_m_s": v[i], "measured": meas[i], "kinematik": bt.convection_h(p, 230.0, v[i]),
                "rel": bt.convection_h(p, 230.0, v[i]) / meas[i] - 1} for i in test]
        good = all(abs(x["rel"]) <= lim2 for x in pts)
        cal[name] = {"h0": h0, "h1": h1, "held_out": pts, "passes": good}
        ok2 = ok2 and good
    cal["passed"] = bool(ok2)
    out["calibrated_held_out"] = cal
    # the default since 1.7.13 is the line through all six points (less radiation): in-sample, so not evidence; the
    # evidence is the held-out test above
    out["default_in_sample"] = [{"v_m_s": x, "measured": hm, "kinematik": bt.h_rotating_disc(230.0, x) + hr,
                                 "rel": (bt.h_rotating_disc(230.0, x) + hr) / hm - 1} for x, hm in zip(v, meas)]
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    print(f"radiation {res['h_radiation_W_m2K']:.1f} W/m2K")
    for r in res["rows"]:
        if r["D_mm"] == 230 and r["R_mm"] == 250:
            print(f"  D 230, R 250, {r['v_m_s']:5.2f} m/s: measured {r['measured']:5.1f}, KinematiK {r['kinematik']:5.1f} "
                  f"({100 * r['rel']:+5.1f} %)")
    print(f"uncalibrated: worst over all geometries {100 * res['worst_abs_rel']:.1f} %;",
          "PASSED" if res["passed"] else "FAILED")
    for name, c in res["calibrated_held_out"].items():
        if name == "passed":
            continue
        print(f"calibrated {name}: h = {c['h0']:.1f} + {c['h1']:.2f} v; held out " +
              ", ".join(f"{x['v_m_s']} m/s {100 * x['rel']:+.1f} %" for x in c["held_out"]), "PASS" if c["passes"] else "FAIL")
    sys.exit(0 if (res["passed"] and res["calibrated_held_out"]["passed"]) else 1)
