# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The steering-effort margin of the final front and its alternative upright (paper section 3.3): this car's
declarations on the general suspension/steering_margin.py, which works on any manifest.

Declared here: the tire levels (the declared set; the R25B shape at road friction 1.43 and 1.55; the car tire's belt
friction, supplement S6.34 and S6.38), the assumed trail of 25.4 mm (1 in, the design assumption of paper reference
[30]) and the alternative upright's offset, 2.0 mm forward of the kingpin axis: a round machining value past the
1.79 mm the declared set needs for 10 N*m at 25.4 mm. The 3.1 mm the R25B shape at 1.55 would need is checked too,
and breaks the wheel-clearance wall. The loads, effort limit and ratio are the manifest's own. The steering ratio is
not the lever: the final front needs 118.9 deg of the 120 deg wheel-angle wall at the hairpin.
Loads N, lengths mm, angles deg, torques N*m."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import r25b_check as rc                                                       # noqa: E402
from suspension import steering_feel as sf                                    # noqa: E402
from suspension import steering_margin as smg                                 # noqa: E402
from suspension.genesis_repro import GenesisManifest                          # noqa: E402

T0_ASSUMED = 25.4                    # mm, 1 in, design assumption of [30]
ROAD_MU_HI = 1.43                    # highest road peak mu on MRA's 0.6 factor (supplement S6.38)
OFFSET_MM = -2.0                     # spindle 2.0 mm forward of the kingpin axis (corner x points rearward)
KEYS = ("lock_sw_deg", "ackermann_lock_pct", "compliance_toe_braking_deg", "compliance_toe_deg", "anti_dive_pct",
        "link_fos_min", "rocker_bearing_sf", "band", "caster_deg", "kpi_deg")


def tire_levels() -> dict:
    T = rc.tires()
    belt = T["r25b_belt"]
    return {"declared": T["synthetic"], "r25b_1p55": T["r25b_shape_at_1p55"],
            "r25b_road_1p43": rc.make(rc.COEFFS, ROAD_MU_HI / belt.mu_peak(1100.0, 0.0)),
            "r25b_car_tire_belt": T["r25b_car_tire_belt"]}


def _pick(v: dict) -> dict:
    d = {k: v.get(k) for k in KEYS}
    d["env"] = v.get("clearance_mm")
    return d


def run(out_path: str | None = None) -> dict:
    """The margin and the alternative upright for front_v11_final; torques in N*m, trails and offsets in mm."""
    man = GenesisManifest.from_json(rc.FINAL_FRONT.read_text())
    lv = tire_levels()
    mg = smg.margin(man, lv, T0_ASSUMED)
    levels = {n: {"mu_1100": float(lv[n].mu_peak(1100.0, 0.0)), "hand_brush_Nm": r["hand_brush_Nm"],
                  "hand_25p4_Nm": r["hand_assumed_Nm"], "t0_limit_mm": r["t0_limit_mm"],
                  "trail_for_10Nm_at_25p4": r["trail_for_limit_mm"]} for n, r in mg["tires"].items()}
    chk = smg.offset_check(man, OFFSET_MM)
    t1 = chk["trail_mm"][1]
    alt = smg.margin(man, lv, T0_ASSUMED, hp=smg.spindle_offset(smg.corner(man), OFFSET_MM))
    se = smg.steer_effort(man)
    cont = {"dx_mm": OFFSET_MM, "trail_base_mm": chk["trail_mm"][0], "trail_contingency_mm": t1,
            "base": _pick(chk["before"]), "contingency": _pick(chk["after"]), "walls_hold": chk["walls_hold"],
            "bands_hold": chk["bands_hold"],
            "lightening": {k: sf.limit_feel(lv["declared"], se.fz_outer_n, tr)["lightening"]
                           for k, tr in (("base", chk["trail_mm"][0]), ("contingency", t1))},
            "hand_25p4_Nm": {n: r["hand_assumed_Nm"] for n, r in alt["tires"].items()},
            "hand_brush_Nm": {n: r["hand_brush_Nm"] for n, r in alt["tires"].items()}}
    need = levels["r25b_1p55"]["trail_for_10Nm_at_25p4"]
    dx = smg.offset_for_trail(smg.corner(man), need)
    big = smg.offset_check(man, dx)
    out = {"margin": {"trail_mm": mg["trail_mm"], "ratio": mg["ratio"], "levels": levels}, "offset_mm": OFFSET_MM,
           "contingency": cont, "r25b_1p55_offset": {"dx_mm": dx, "trail_mm": big["trail_mm"][1],
                                                     "env": big["after"]["clearance_mm"],
                                                     "walls_hold": big["walls_hold"]}}
    if out_path:
        Path(out_path).write_text(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    print(json.dumps(run(sys.argv[1] if len(sys.argv) > 1 else None), indent=1))
