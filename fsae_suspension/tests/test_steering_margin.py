# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Paper section 3.3: the steering-effort margin and the pre-verified alternative upright."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import steering_margin as sm                                              # noqa: E402


@pytest.fixture(scope="module")
def run():
    return sm.run()


def test_margin_on_road_level_tires(run):
    lv = run["margin"]["levels"]
    assert round(lv["declared"]["hand_brush_Nm"], 1) == 8.1
    assert round(lv["declared"]["t0_limit_mm"], 1) == 22.4
    for k in ("declared", "r25b_1p55", "r25b_road_1p43"):
        assert lv[k]["hand_brush_Nm"] < 9.0 and lv[k]["t0_limit_mm"] > 20.0
    assert 9.8 < lv["r25b_car_tire_belt"]["hand_brush_Nm"] < 10.0           # belt friction: the thin case
    assert lv["declared"]["hand_25p4_Nm"] > 10.0                            # a 1 in trail breaks the wall


def test_alternative_upright_keeps_every_wall_and_restores_the_effort(run):
    c = run["contingency"]
    assert c["dx_mm"] == -2.0 and round(c["trail_base_mm"] - c["trail_contingency_mm"], 3) == 2.0
    b, n = c["base"], c["contingency"]
    for k in ("caster_deg", "kpi_deg", "lock_sw_deg", "ackermann_lock_pct", "band"):
        assert n[k] == pytest.approx(b[k], abs=1e-9)                        # caster, kingpin, steering, curves unchanged
    assert n["env"] > 0.0 and n["link_fos_min"] >= 2.0 and n["rocker_bearing_sf"] >= 2.0
    assert abs(n["compliance_toe_braking_deg"]) <= 0.25 and abs(n["compliance_toe_deg"]) <= 0.08
    assert c["hand_25p4_Nm"]["declared"] < 10.0 and c["hand_25p4_Nm"]["r25b_road_1p43"] < 10.0


def test_the_larger_offset_breaks_clearance(run):
    r = run["r25b_1p55_offset"]
    assert r["dx_mm"] < -3.0 and r["env"] < 0.0


def test_the_library_works_on_any_front_manifest():
    """suspension.steering_margin reads the loads, limit and ratio from the manifest it is given."""
    from suspension import steering_margin as smg
    from suspension.genesis_repro import GenesisManifest
    from suspension.tiremodel import default_tire
    m = GenesisManifest.from_json((Path(__file__).resolve().parents[1] / "manifests"
                                   / "front_v11_29deg.recorded.genesis.json").read_text())
    out = smg.margin(m, {"declared": default_tire()}, 25.4)
    r = out["tires"]["declared"]
    assert out["ratio"] > 0 and r["t0_limit_mm"] > 0 and r["hand_assumed_Nm"] > r["hand_brush_Nm"] > 0
    hp = smg.corner(m)
    dx = smg.offset_for_trail(hp, out["trail_mm"] - 1.0)
    assert dx == pytest.approx(-1.0, abs=1e-6)               # the trail moves one for one with the offset
    chk = smg.offset_check(m, dx)
    assert chk["trail_mm"][1] == pytest.approx(out["trail_mm"] - 1.0, abs=1e-6)
    assert set(chk) >= {"before", "after", "walls_hold", "bands_hold"}
