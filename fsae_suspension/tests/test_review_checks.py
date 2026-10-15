# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Paper section 3.3 and supplement S13.6: joints and tabs, full lock, full-solve yield."""
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import review_checks as rc                                                  # noqa: E402
import yield_full_check as yfc                                              # noqa: E402
from suspension import compliance_margin as cm                              # noqa: E402
from suspension.genesis_repro import GenesisManifest                        # noqa: E402


@pytest.fixture(scope="module")
def joints():
    return rc.joints()


def test_rigid_ends_reproduce_the_manifest_walls(joints):
    s = joints["sweep"]
    assert round(s["corner"]["toe_deg"]["rigid"], 3) == -0.012 and round(s["braking"]["toe_deg"]["rigid"], 2) == 0.23


def test_compliant_ends_only_add_toe_and_break_the_braking_wall(joints):
    b, c = joints["sweep"]["braking"]["toe_deg"], joints["sweep"]["corner"]["toe_deg"]
    keys = ["rigid", "100000.0", "50000.0", "20000.0"]
    assert all(abs(b[k1]) < abs(b[k2]) for k1, k2 in zip(keys, keys[1:]))
    assert all(abs(c[k1]) < abs(c[k2]) for k1, k2 in zip(keys, keys[1:]))
    assert abs(b["100000.0"]) > 0.25 and abs(c["50000.0"]) < 0.08
    assert 8e3 < joints["required_n_per_mm"]["corner"] < 15e3 and joints["required_n_per_mm"]["braking"] > 4e5
    assert joints["lf_move_clearance_allows_mm"] < joints["lf_move_for_wall_at_ref_mm"]


def test_end_stiffness_is_the_series_sum_of_two_ends():
    m = GenesisManifest.from_json((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    spec, _ = cm.walls(m)["braking"]
    s2 = cm.with_end_stiffness(spec, 50000.0)
    for k, ms in s2.members.items():
        k0 = spec.members[k].k_tab
        want = 25000.0 if k0 is None else 1.0 / (1.0 / k0 + 1.0 / 25000.0)
        assert ms.k_tab == pytest.approx(want)
    assert cm.with_end_stiffness(spec, math.inf) is spec


def test_full_lock_changes_little():
    out = rc.lock()
    f = out["final"]
    assert abs(max(f["arm_inner_mm"], f["arm_outer_mm"]) / f["arm_straight_mm"] - 1.0) < 0.01
    assert abs(f["jacking_static"]["hand_Nm"]) < 0.5 and abs(f["jacking_design"]["hand_Nm"]) < 0.5
    assert f["hand_lock_Nm"] < 10.0 and out["alternative"]["hand_lock_Nm"] < f["hand_lock_Nm"]
    assert out["standstill"]["hand_Nm"] > 10.0                     # a standstill effort, which the limit does not cover


def test_full_solves_agree_with_first_order_at_jig_tolerance():
    o = yfc.check("rear_v11_final", 60)
    j = o["fields"]["jig"]
    assert j["full_yield"] == 1.0 and j["linear_yield"] == 1.0 and j["agreement"] == 1.0
    assert yfc.lower_bound_95(60, 60) == pytest.approx(0.05 ** (1 / 60))


def test_the_front_re_searched_with_joints_holds_every_wall_at_50_kn_per_mm():
    import json
    from suspension.genesis_repro import hp_from_dict
    d = json.loads((ROOT / "manifests" / "front_v11_joints50.recorded.genesis.json").read_text())
    F = json.loads((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    for k in ("targets", "tolerance", "search"):
        assert d[k] == F[k]
    for case in ("elasto", "elasto_braking"):                 # change 1: the link ends at 50 kN/mm
        assert all(m["k_tab"] == 25000.0 for m in d["volume"]["properties"][case]["members"].values())
    ack = {b["prop"]: b for b in d["volume"]["properties"]["bounds"]}["ackermann_lock_pct"]
    assert ack["lo"] == 75.0 and ack["hi"] == 100.0             # change 2: a ceiling on Ackermann at lock
    rest = lambda v: [b for b in v["properties"]["bounds"] if b["prop"] != "ackermann_lock_pct"]   # noqa: E731
    assert rest(d["volume"]) == rest(F["volume"])
    m = GenesisManifest.from_json(json.dumps(d))
    assert m.start_search is not None and m.recorded["winner_verdict"] == "RESILIENT"
    _, tg, vol, _ = m.objects()
    w = hp_from_dict(m.recorded["winner_hardpoints"])
    assert vol.property_violations(w) == [] and not vol.keepout_violations(w) and vol.envelope_margin(w) >= 0.0
    v = vol.properties.evaluate(w)
    assert abs(v["compliance_toe_braking_deg"]) <= 0.25 and abs(v["compliance_toe_deg"]) <= 0.08
    assert m.recorded["start_search"]["evaluations"] == 260
    assert 75.0 <= v["ackermann_lock_pct"] <= 100.0


def test_every_front_check_holds_on_the_front_to_build():
    fb = rc.front_to_build()
    j, f = fb["front_v11_joints50"], fb["front_v11_final"]
    assert j["props"]["compliance_toe_braking_deg"] <= 0.25 and j["props"]["lock_sw_deg"] <= 120.0
    assert j["mu_front"] >= 1.83 and j["clearance_mm"] >= 0.0 and j["margin"]["declared"]["hand_brush_Nm"] < 10.0
    assert j["alt_build"]["walls_hold"] and j["alt_build"]["clearance_mm"] > 0.0
    assert j["alt_build"]["hand_25p4"]["declared"] < 10.0 and j["lock"]["hand_lock_Nm"] < 10.0
    assert j["ackermann_loss_pct"][1.5] < 0.1 and f["ackermann_loss_pct"][1.5] < 0.01   # inside 75 to 100%, a negligible cost
    assert abs(j["mr_static"] - f["mr_static"]) < 0.005 and j["spring_mm"]["0.0"] == pytest.approx(f["spring_mm"]["0.0"], abs=0.1)
    hs = j["half_scale"]["values_deg"]
    assert hs["camber_droop"] > 0 > hs["camber_bump"] and hs["spacer_toe_droop"] > 0 > hs["spacer_toe_bump"]


def test_what_it_changes_on_the_car():
    g = rc.car_gains()
    assert g["decel_g"]["73"] > 1.5 > g["decel_g"]["60"] and g["stop_m"]["73"] < g["stop_m"]["60"]
    d, b = g["delivered"], g["build"]
    assert d["brake_toe_50_deg"] > 0.25 >= b["brake_toe_50_deg"] and b["brake_diff_ok"] > 0.6 > d["brake_diff_ok"]
    assert d["lock_sw_deg"] > 120.0 >= b["lock_sw_deg"]
    assert b["ack_loss_pct"][1.5] < d["ack_loss_pct"][1.5] and b["hand_Nm"] < 10.0
