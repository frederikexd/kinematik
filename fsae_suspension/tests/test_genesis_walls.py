# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Vehicle-level walls of InverseGenesis: brake bias, link strength, rocker bearings."""
from pathlib import Path

import pytest

from suspension import actuation as act, genesis_walls as gw, inverse_genesis as ig
from suspension.genesis_repro import (GenesisManifest, _property_bounds_from_dict,
                                      _property_bounds_to_dict, hp_from_dict)

MAN = Path(__file__).resolve().parents[1] / "manifests"


def _winner(name):
    return hp_from_dict(GenesisManifest.from_json(
        (MAN / f"{name}.recorded.genesis.json").read_text()).recorded["winner_hardpoints"])


def test_brake_decel_reproduces_the_tire_use_model():
    bd = gw.BrakeDecel()
    assert bd.decel_g(0.60) == pytest.approx(1.27, abs=0.005)
    assert bd.decel_g(0.73) == pytest.approx(1.57, abs=0.005)
    bias, dec = bd.balanced()
    assert bias == pytest.approx(0.73, abs=0.005) and dec == pytest.approx(1.57, abs=0.005)


def test_link_structure_reproduces_table_8():
    # the delivered front, pushrod on the lower wishbone, on the corrected (two-body) statics: the leg bending governs
    hp = _winner("front_steering_walls")
    m = gw.LinkStructure.from_shock_cases("front", 0.73).margins(hp)
    assert m["LR"]["fos_buckling"] == pytest.approx(1.22, abs=0.01)
    assert m["LR"]["fos_yield"] == pytest.approx(0.07, abs=0.01)
    up = gw.LinkStructure.from_shock_cases("front", 0.73,
                                           per_member={"LR": [15.875, 1.651], "LF": [15.875, 1.245]})
    assert up.fos_min(hp) == pytest.approx(0.27, abs=0.01)


def test_v11_fronts_as_released_buckle_on_the_specified_tube():
    ls = gw.LinkStructure.from_shock_cases("front", 0.73)
    assert ls.margins(_winner("front_v11_29deg"))["LR"]["fos_buckling"] == pytest.approx(0.81, abs=0.01)
    assert ls.margins(_winner("front_v11_25deg"))["LR"]["fos_buckling"] == pytest.approx(0.88, abs=0.01)


def test_rocker_bearing_wall_reproduces_the_actuation_stage():
    hp = act.synthesize_actuation(_winner("front_steering_walls")).hp
    assert gw.RockerBearingWall.from_vehicle_cases("front", 0.60).s0_min(hp) == pytest.approx(0.60, abs=0.01)
    w = gw.RockerBearingWall.from_vehicle_cases("front", 0.73, label="2 x SKF 6204-2RSH", C0_N=6550.0)
    assert w.s0_min(hp) == pytest.approx(1.67, abs=0.01)


def test_a_vehicle_bound_needs_its_spec():
    with pytest.raises(ValueError):
        ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("link_fos_min", lo=2.0)])
    with pytest.raises(ValueError):
        ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("brake_decel_g", lo=1.5)])


def test_vehicle_walls_round_trip_through_a_manifest_dict():
    b = ig.SolvedPropertyBounds(
        bounds=[ig.PropertyBound("brake_decel_g", lo=1.5), ig.PropertyBound("link_fos_min", lo=2.0),
                ig.PropertyBound("rocker_bearing_sf", lo=2.0)],
        brake_bias_front=0.73, brake=gw.BrakeDecel(),
        structure=gw.LinkStructure.from_shock_cases("front", 0.73),
        bearing=gw.RockerBearingWall.from_vehicle_cases("front", 0.73))
    d = _property_bounds_to_dict(b)
    assert _property_bounds_to_dict(_property_bounds_from_dict(d)) == d


def test_the_declared_bias_is_refused_by_the_deceleration_wall():
    hp = _winner("front_v11_29deg")
    for bias, ok in ((0.60, False), (0.73, True)):
        b = ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("brake_decel_g", lo=1.5)],
                                    brake_bias_front=bias, brake=gw.BrakeDecel())
        assert (not b.violations(hp)) is ok


@pytest.mark.parametrize("name,deg", [("front_v11_29deg", 34.3), ("front_v11_25deg", 27.6)])
def test_joint_swing_at_the_declared_lock(name, deg):
    m = GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())
    _, _, vol, _ = m.objects()
    v = ig.properties_of(_winner(name), vol.properties, only=["joint_swing_lock_deg"])
    assert v["joint_swing_lock_deg"] == pytest.approx(deg, abs=0.05)


@pytest.mark.parametrize("name,fos,s0,swing", [("front_v11_29deg_walls", 0.18, 1.58, 34.3),
                                                ("front_v11_25deg_walls", 0.34, 1.97, 27.4)])
def test_walls_fronts_are_refused_on_the_corrected_statics(name, fos, s0, swing):
    # recorded RESILIENT on the earlier statics; their pushrods ride on the lower wishbone, and on the corrected statics
    # the link and bearing walls refuse the recorded winner (a re-run returns no winner, supplement S13.3)
    m = GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())
    _, _, vol, _ = m.objects()
    hp = _winner(name)
    assert m.recorded["winner_verdict"] == "RESILIENT"
    assert [p for p, _, _ in vol.wall_violations(hp)] == ["link_fos_min", "rocker_bearing_sf"]
    v = vol.properties.evaluate(hp)
    assert v["brake_decel_g"] == pytest.approx(1.57, abs=0.005)
    assert v["link_fos_min"] == pytest.approx(fos, abs=0.01)
    assert v["rocker_bearing_sf"] == pytest.approx(s0, abs=0.01)
    assert v["joint_swing_lock_deg"] == pytest.approx(swing, abs=0.05)


@pytest.mark.parametrize("name,prop", [("front_v11_29deg_b60_brake", "brake_decel_g"),
                                       ("front_v11_29deg_b73_6001", "rocker_bearing_sf"),
                                       ("front_v11_29deg_b73_tube035", "link_fos_min")])
def test_refusal_manifests_name_their_wall(name, prop):
    m = GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())
    _, _, vol, _ = m.objects()
    assert not m.recorded["ok"]
    assert [p for p, _, _ in vol.property_violations(hp_from_dict(m.hardpoints))] == [prop]


def test_node_brief_hash_and_sources():
    import hashlib, json
    b = json.loads((MAN / "chassisv11_node_brief.json").read_text())
    h = b.pop("inputs_sha256")
    assert hashlib.sha256(json.dumps(b, sort_keys=True, separators=(",", ":")).encode()).hexdigest() == h
    for c in b["corners"].values():
        m = GenesisManifest.from_json((MAN / f"{c['source_manifest']}.recorded.genesis.json").read_text())
        assert m.inputs_sha256 == c["source_inputs_sha256"]


def test_final_front_reads_its_lock_on_the_mean_steer():
    m = GenesisManifest.from_json((MAN / "front_v11_25deg_walls.recorded.genesis.json").read_text())
    _, _, vol, _ = m.objects()
    assert vol.properties.steer_effort.lock_basis == "mean"
    v = ig.properties_of(_winner("front_v11_25deg_walls"), vol.properties,
                         only=["ackermann_lock_pct", "compliance_toe_deg", "lock_sw_deg"])
    assert v["ackermann_lock_pct"] == pytest.approx(6.7, abs=0.1)
    assert v["compliance_toe_deg"] == pytest.approx(-0.062, abs=0.0005)          # corrected cornering case
    assert v["lock_sw_deg"] == pytest.approx(119.6, abs=0.05)


def test_hairpin_front_reads_the_two_track_mean_and_holds_its_new_walls():
    # the final front: lock on the mean steer at 25.6 deg, Ackermann >= 75%, misalignment <= 12 deg with the
    # bolts declared, compliance toe with the links in series, on an 81 mm rack and a 27 mm envelope sweep
    m = GenesisManifest.from_json((MAN / "front_v11_hairpin_walls.recorded.genesis.json").read_text())
    _, _, vol, _ = m.objects()
    se = vol.properties.steer_effort
    assert (se.lock_basis, se.lock_road_wheel_deg, se.rack_c_mm_per_rev) == ("mean", 25.6, 81.0)
    b = {x.prop: (x.lo, x.hi) for x in vol.properties.bounds}
    assert b["ackermann_lock_pct"][0] == 75.0 and b["joint_misalignment_lock_deg"][1] == 12.0
    assert b["anti_dive_pct"] == (63.6, 70.0) and b["compliance_toe_deg"] == (-0.08, 0.08)
    assert "joint_swing_lock_deg" not in b                             # withdrawn: it counts rotation about the bolt
    v = ig.properties_of(_winner("front_v11_hairpin_walls"), vol.properties,
                         only=["ackermann_lock_pct", "joint_misalignment_lock_deg", "lock_sw_deg", "compliance_toe_deg"])
    assert v["ackermann_lock_pct"] == pytest.approx(75.3, abs=0.05)
    assert v["joint_misalignment_lock_deg"] == pytest.approx(4.41, abs=0.005)
    assert v["lock_sw_deg"] == pytest.approx(118.9, abs=0.05)
    assert v["compliance_toe_deg"] == pytest.approx(-0.014, abs=0.0005)          # corrected cornering case, 30 kN/mm
    assert vol.envelope_margin(_winner("front_v11_hairpin_walls")) == pytest.approx(0.49, abs=0.005)


def test_lock_report_reproduces_the_delivered_front():
    from suspension import dynamic_ackermann as da
    r = da.lock_report(_winner("front_steering_walls"), 76.2, 20.0)
    assert [round(a, 1) for a in r["ackermann_pct_lo_and_lock"]] == [-3.4, -2.3]
    assert r["camber_at_lock_outside_deg"] == pytest.approx(-3.52, abs=0.01)
    assert r["camber_at_lock_inside_deg"] == pytest.approx(-0.86, abs=0.01)
