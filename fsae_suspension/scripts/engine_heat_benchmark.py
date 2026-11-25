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


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    m, k, b = res["measured"], res["kinematik"], res["bias"]
    print(f"coolant share: KinematiK {k['coolant_frac']:.2f}; measured {m['coolant_range'][0]:.3f}-{m['coolant_range'][1]:.3f}, "
          f"mean {m['coolant_mean_full_load']:.3f} full load ({100 * b['coolant_vs_full_load_mean']:+.0f} %), "
          f"{m['coolant_mean_half_load']:.3f} half load ({100 * b['coolant_vs_half_load_mean']:+.0f} %)")
    print(f"BTE: KinematiK {k['bte']:.2f}; measured {m['bte_range'][0]:.3f}-{m['bte_range'][1]:.3f}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
