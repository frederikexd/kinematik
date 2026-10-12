# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The hairpin lock read on the axle's mean steer, and the links' own axial give in the compliance solve."""
import json
from pathlib import Path

import pytest

from suspension import dynamic_ackermann as da, elastokinematics as ek, inverse_genesis as ig
from suspension.compliance import MemberStiffness
from suspension.genesis_repro import GenesisManifest, hp_from_dict, vehicle_load_cases

MAN = Path(__file__).resolve().parents[1] / "manifests"


def _w(name):
    m = GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())
    return m, hp_from_dict(m.recorded["winner_hardpoints"])


def test_lock_basis_is_validated_and_defaults_to_the_inner_wheel():
    assert ig.SteerEffort().lock_basis == "inner"
    with pytest.raises(ValueError):
        ig.SteerEffort(lock_basis="outer")
    with pytest.raises(ValueError):
        da.lock_report(_w("front_v11_29deg_walls")[1], 76.2, 25.0, lock_basis="outer")


def test_lock_basis_round_trips_and_older_manifests_keep_their_hash():
    se = ig.SteerEffort(lock_basis="mean")
    assert ig.SteerEffort(**se.to_dict()).lock_basis == "mean"
    assert "lock_basis" not in ig.SteerEffort().to_dict()          # unchanged inputs serialise as before
    for name, h in (("front_steering_walls", "bbc4d8297765"), ("front_v11_29deg_walls", "4f088b0e4609")):
        assert _w(name)[0].inputs_sha256.startswith(h)


def test_mean_lock_reaches_the_mean_and_inner_lock_the_inner_wheel():
    m, hp = _w("front_v11_25deg_walls")
    se = m.objects()[2].properties.steer_effort
    assert se.lock_basis == "mean"
    r = da.lock_report(hp, se.rack_c_mm_per_rev, se.lock_road_wheel_deg, lock_basis="mean")
    assert r["mean_deg"] == pytest.approx(25.0, abs=1e-6)
    assert r["inner_deg"] > 25.0 > r["outer_deg"]                    # positive Ackermann puts the inner wheel past the mean
    ri = da.lock_report(hp, se.rack_c_mm_per_rev, se.lock_road_wheel_deg, lock_basis="inner")
    assert ri["inner_deg"] == pytest.approx(25.0, abs=1e-6) and ri["mean_deg"] < 25.0
    assert se.lock_sw_deg(hp) == pytest.approx(r["wheel_lock_deg"], abs=0.05)


def test_elastospec_members_round_trip_and_reject_tabulated_members():
    mem = {"LR": MemberStiffness(material="Steel 4130", od_mm=15.875, wall_mm=1.651),
           "TR": MemberStiffness(material="Steel 4130", od_mm=15.875, wall_mm=0.889)}
    spec = ek.ElastoSpec(stiffness=ek.StiffnessField(30000.0), Fy=1000.0, members=mem)
    d = spec.to_dict()
    back = ek.ElastoSpec.from_dict(json.loads(json.dumps(d)))
    assert set(back.members) == {"LR", "TR"}
    assert back.members["LR"].wall_mm == pytest.approx(1.651)
    assert back.to_dict() == d
    assert "members" not in ek.ElastoSpec(Fy=1000.0).to_dict()     # no members: serialises as before


def test_links_in_series_add_to_pickup_compliance():
    m, hp = _w("front_v11_final")
    el = m.objects()[2].properties.elasto                            # frame-derived flexibility, the declared links
    c = [c for c in vehicle_load_cases(axle="front", brake_bias_front=0.73) if c.name == "1.5g corner"][0]
    kw = dict(Fx=c.Fx, Fy=c.Fy, Fz=c.Fz, Mz=c.mz_Nmm)
    pick = ek.solve_elastokinematic(hp, ek.ElastoSpec(stiffness=el.stiffness, **kw), jacobian=False).change["toe"]
    both = ek.solve_elastokinematic(hp, ek.ElastoSpec(stiffness=el.stiffness, members=dict(el.members), **kw),
                                    jacobian=False).change["toe"]
    assert abs(both) > abs(pick)
    assert abs(both) <= 0.08 + 1e-9                                  # the final front's compliance wall


def test_misalignment_with_declared_bolts_is_far_below_the_link_swing():
    # the link swing counts rotation about the bolt, which a spherical joint takes freely
    for name in ("front_v11_hairpin_walls", "front_v11_29deg_walls", "front_v11_25deg_walls", "front_steering_walls"):
        m, hp = _w(name)
        se = m.objects()[2].properties.steer_effort
        rack = se.lock_sw_deg(hp) * se.rack_c_mm_per_rev / 360.0
        jm = ig.joint_misalignment(hp, rack_travel_mm=rack)
        js = ig.joint_swing(hp, rack_travel_mm=rack)
        assert jm["_worst"][1] < 6.0 < js["_worst"][1]
    m, hp = _w("front_v11_29deg_walls")
    se = m.objects()[2].properties.steer_effort
    rack = se.lock_sw_deg(hp) * se.rack_c_mm_per_rev / 360.0
    jm = ig.joint_misalignment(hp, rack_travel_mm=rack)
    assert ig.joint_swing(hp, rack_travel_mm=rack)["_worst"][1] == pytest.approx(34.3, abs=0.05)
    assert jm["_worst"][1] == pytest.approx(4.74, abs=0.01)


def test_misalignment_is_zero_at_ride_height_and_a_boundable_property():
    _, hp = _w("front_v11_29deg_walls")
    jm = ig.joint_misalignment(hp, travel_mm=(0.0, 0.0), n_travel=1)
    assert jm["_worst"][1] == pytest.approx(0.0, abs=1e-6)
    assert "joint_misalignment_lock_deg" in ig.BOUNDABLE_PROPERTIES
