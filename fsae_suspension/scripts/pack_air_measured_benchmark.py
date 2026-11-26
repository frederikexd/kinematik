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
            "passed": bool(absolute and flag_m == flag_k)}


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for rate, r in res["rows"].items():
        print(f"  {rate} It: measured {r['measured_C']:.1f} C, KinematiK {r['kinematik_C']:.2f} C ({100 * r['rel']:+.0f} %); "
              f"heat {r['kinematik_heat_W']:.2f} W vs measured {r['measured_heat_W']} W; with measured heat "
              f"{r['with_measured_heat_C']:.2f} C")
    print("absolute", "PASS" if res["absolute_passes"] else "FAIL", "| decision", res["decision"])
    sys.exit(0 if res["passed"] else 1)
