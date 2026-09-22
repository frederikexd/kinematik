# ============================================================================
#  KinematiK — tests for the wheel envelope inside the InverseGenesis loop
#  Copyright (c) 2026 Frederik Thio. Open source.
# ============================================================================
"""No link may pass through the rim or tire.

The solver has no wheel in it, and an early run returned a corner whose toe
link crossed the tire in plan view while every curve sat in its band. These
tests pin the fix: the envelope reconstructs the wheel from the solved camber
and toe, rejects any link body inside the rim/tire solid and any outboard
pickup outside the upright envelope, over travel and both locks, and the
search refuses every step that would enter it.
"""
from __future__ import annotations

import numpy as np
import pytest

from suspension.kinematics import Hardpoints, SuspensionKinematics
from suspension.kinematik_stochastic import ToleranceField
from suspension.inverse_genesis import (
    GenesisTargets, LegalVolume, TargetCurve, WheelEnvelope, curves_of,
    genesis_solve, inverse_genesis, render_genesis_md, spin_axis, _perturbed,
)

ST = np.array([-25.0, -12.5, 0.0, 12.5, 25.0])
ALL = ["upper_front_inner", "upper_rear_inner", "lower_front_inner",
       "lower_rear_inner", "tie_rod_inner", "upper_outer", "lower_outer",
       "tie_rod_outer"]


def _front() -> Hardpoints:
    """The delivered front corner of the paper (table 13)."""
    d = dict(upper_front_inner=[-120.0, 288.1, 280.5],
             upper_rear_inner=[100.0, 270.2, 267.0],
             lower_front_inner=[-260.0, 170.8, 103.9],
             lower_rear_inner=[42.8, 148.9, 133.7],
             tie_rod_inner=[43.0, 193.8, 147.8],
             upper_outer=[6.5, 560.5, 300.0], lower_outer=[-5.0, 583.6, 120.0],
             tie_rod_outer=[90.0, 572.0, 150.0],
             wheel_center=[0.0, 600.0, 228.0], contact_patch=[0.0, 605.0, 0.0])
    hp = Hardpoints.default()
    for k, v in d.items():
        setattr(hp, k, np.array(v, float))
    hp.static_camber, hp.static_toe = -2.5, 0.0
    return hp


def _targets() -> GenesisTargets:
    return GenesisTargets(curves=[
        TargetCurve("camber_deg", ST, -2.5 - 0.035 * ST, np.full(5, 0.30)),
        TargetCurve("toe_deg", ST, np.zeros(5), np.full(5, 0.08)),
        TargetCurve("rc_height_mm", ST, np.full(5, 55.0), np.full(5, 18.0))])


# --------------------------------------------------------------------------- #
#  Geometry of the check
# --------------------------------------------------------------------------- #
def test_spin_axis_inverts_the_kinematics_conventions():
    """Reconstructed axis must reproduce the solver's own camber and toe."""
    k = SuspensionKinematics(_front())
    for t in (-25.0, 0.0, 25.0):
        s = k.solve_at_travel(t)
        ax = spin_axis(s.camber, s.toe)
        assert np.degrees(np.arctan2(ax[0], abs(ax[1]))) == pytest.approx(
            s.toe, abs=1e-9)
        assert -np.degrees(np.arctan2(ax[2], abs(ax[1]))) == pytest.approx(
            s.camber, abs=1e-9)
        assert np.linalg.norm(ax) == pytest.approx(1.0)


def test_paper_finding_is_reproduced():
    """Fig 3c: the delivered tie-rod outer sits outside the 115 mm envelope."""
    v, m = WheelEnvelope().check(_front())
    assert m < 0.0
    tie = [x for x in v if x[0] == "tie_rod_outer"]
    assert tie and min(x[2] for x in tie) == pytest.approx(-3.25, abs=0.05)


def test_a_link_through_the_tire_is_caught():
    """A tie rod pulled far rearward crosses the tire in plan view."""
    hp = _perturbed(_front(), {"tie_rod_inner": np.array([500.0, 40.0, 0.0])})
    env = WheelEnvelope(pickup_radius_mm=127.0)   # isolate the link rule
    v, _ = env.check(hp)
    assert any(item == "tie_rod link" for item, _, _ in v)


def test_a_clear_geometry_passes():
    """Pull every outboard pickup inboard of the envelope: no violation."""
    env = WheelEnvelope(pickup_radius_mm=127.0)
    hp = _perturbed(_front(), {"tie_rod_outer": np.array([-15.0, 0.0, 0.0])})
    assert all(item != "tie_rod link" for item, _, _ in env.check(hp)[0])


def test_steering_lock_is_checked():
    """A declared rack travel adds both locks to the evaluated states."""
    env = WheelEnvelope(rack_travel_mm=32.0)
    wheres = {w for _, w, _ in env.check(_front())[0]}
    assert any("rack +32" in w for w in wheres)
    assert any("rack -32" in w for w in wheres)


def test_envelope_validation():
    with pytest.raises(ValueError):
        WheelEnvelope(rim_radius_mm=230.0, tire_radius_mm=228.0)
    with pytest.raises(ValueError):
        WheelEnvelope(pickup_radius_mm=140.0, rim_radius_mm=127.0)
    with pytest.raises(ValueError):
        WheelEnvelope(links=("steering_column",))


# --------------------------------------------------------------------------- #
#  Inside the search
# --------------------------------------------------------------------------- #
def test_start_inside_the_wheel_is_refused():
    vol = LegalVolume.around(_front(), 40.0, points=ALL,
                             wheel_envelope=WheelEnvelope())
    c = genesis_solve(_front(), _targets(), vol)
    assert not c.ok and c.envelope_rejections == 1
    assert "wheel envelope" in c.worst_row


@pytest.fixture(scope="module")
def paired_runs():
    """The unbounded and bounded runs on the delivered front corner."""
    hp, tg = _front(), _targets()
    env = WheelEnvelope(rack_travel_mm=32.0)
    fld = ToleranceField.preset("jig_weld")
    free = inverse_genesis(hp, tg, LegalVolume.around(hp, 40.0, points=ALL),
                           fld=fld, n_starts=6, n_yield=1000, seed=0)
    walled = inverse_genesis(
        hp, tg, LegalVolume.around(hp, 40.0, points=ALL, wheel_envelope=env),
        fld=fld, n_starts=6, n_yield=1000, seed=0)
    return env, free, walled


def test_unbounded_search_does_put_links_in_the_wheel(paired_runs):
    """The premise: without the wall, the solver returns such corners."""
    env, free, _ = paired_runs
    hits = [c for c in free.candidates if c.hit]
    inside = [c for c in hits if env.margin(_shift(free, c)) < 0.0]
    assert free.winner is not None
    assert env.margin(free.winner_hp) < 0.0
    assert len(inside) >= 3


def _shift(res, c):
    from suspension.inverse_genesis import _shifted
    hp = _front()
    return _shifted(hp, LegalVolume.around(hp, 40.0, points=ALL), c.shift_vec)


def test_no_candidate_returned_inside_the_wheel(paired_runs):
    """With real lock this run may find nothing; it must never return a
    geometry inside the wheel either way."""
    env, _, walled = paired_runs
    if walled.winner is not None:
        assert env.margin(walled.winner_hp) >= 0.0
    for c in walled.candidates:
        if c.hit:
            assert c.envelope_margin_mm is not None
            assert c.envelope_margin_mm >= -1e-9


def test_report_states_the_wheel_clearance(paired_runs):
    _, _, walled = paired_runs
    md = render_genesis_md(walled)
    if walled.winner is not None:
        assert "wheel envelope: worst clearance" in md
        assert "both steering locks" in md
    else:
        assert "wheel envelope" in md or "wheel" in walled.reason


# --------------------------------------------------------------------------- #
#  Manifest and back-compatibility
# --------------------------------------------------------------------------- #
def test_manifest_carries_the_envelope():
    from suspension import genesis_repro as gr
    env = WheelEnvelope(rack_travel_mm=32.0, clearance_mm=3.0)
    vol = LegalVolume.around(_front(), 40.0, points=ALL, wheel_envelope=env)
    man = gr.GenesisManifest.build("wheel", _front(), _targets(), vol, None)
    _, _, vol2, _ = gr.GenesisManifest.from_json(man.to_json()).objects()
    e2 = vol2.wheel_envelope
    assert e2 is not None
    assert e2.rack_travel_mm == 32.0 and e2.clearance_mm == 3.0
    assert e2.links == env.links


def test_undeclared_envelope_leaves_manifests_unchanged():
    from suspension import genesis_repro as gr
    d = gr.volume_to_dict(LegalVolume.around(_front(), 40.0, points=ALL))
    assert "wheel_envelope" not in d
    assert gr.volume_from_dict(d).wheel_envelope is None


def test_envelope_changes_the_inputs_hash():
    from suspension import genesis_repro as gr
    a = gr.GenesisManifest.build(
        "a", _front(), _targets(),
        LegalVolume.around(_front(), 40.0, points=ALL), None)
    b = gr.GenesisManifest.build(
        "b", _front(), _targets(),
        LegalVolume.around(_front(), 40.0, points=ALL,
                           wheel_envelope=WheelEnvelope()), None)
    assert a.inputs_sha256 != b.inputs_sha256


# --------------------------------------------------------------------------- #
#  The complete wheel model: thickness, rod ends, measured tire, upright parts
# --------------------------------------------------------------------------- #
def test_links_have_thickness():
    """A tube is not a line: its radius eats into the clearance."""
    from suspension.inverse_genesis import WheelEnvelope
    hp = _front()
    thin = WheelEnvelope(link_radius_mm=0.0, rod_end_radius_mm=0.0).margin(hp)
    thick = WheelEnvelope(link_radius_mm=8.0, rod_end_radius_mm=0.0).margin(hp)
    assert thick <= thin


def test_rod_end_body_must_clear_the_barrel():
    """A pickup centre inside the rim is not enough if its housing is not."""
    from suspension.inverse_genesis import WheelEnvelope
    hp = _front()
    ok = WheelEnvelope(pickup_radius_mm=127.0, rod_end_radius_mm=0.0)
    big = WheelEnvelope(pickup_radius_mm=127.0, rod_end_radius_mm=11.0)
    assert not any(i == "tie_rod_outer" for i, _, _ in ok.violations(hp))
    assert any(i == "tie_rod_outer" for i, _, _ in big.violations(hp))


def test_measured_tire_profile_overrides_the_taper():
    from suspension.inverse_genesis import WheelEnvelope
    env = WheelEnvelope(tire_profile=((127.0, 60.0), (228.0, 60.0)))
    assert env._half_width(180.0) == pytest.approx(60.0)
    with pytest.raises(ValueError):
        WheelEnvelope(tire_profile=((100.0, 60.0), (228.0, 60.0)))


def test_upright_fixed_part_is_a_keep_out():
    """A caliper declared over the tie-rod outer makes it a violation."""
    from suspension.inverse_genesis import WheelEnvelope, WheelSector
    hp = _front()
    free = WheelEnvelope(pickup_radius_mm=127.0, rod_end_radius_mm=0.0)
    cal = WheelEnvelope(pickup_radius_mm=127.0, rod_end_radius_mm=0.0,
                        sectors=(WheelSector("caliper", 80.0, 125.0, 0.0,
                                             180.0, -60.0, 20.0),))
    assert not any(i == "tie_rod_outer" for i, _, _ in free.violations(hp))
    assert any(i == "tie_rod_outer" for i, _, _ in cal.violations(hp))


def test_old_thin_line_manifests_replay_as_written():
    """A v21 manifest carried no link radius and must not gain one."""
    from suspension import genesis_repro as gr
    d = {"pickup_radius_mm": 115.0, "rim_radius_mm": 127.0,
         "tire_radius_mm": 228.0, "rim_half_width_mm": 89.0,
         "tire_half_width_mm": 95.0, "wheel_offset_mm": 0.0,
         "clearance_mm": 0.0, "travel_mm": [-25.0, 25.0], "n_travel": 5,
         "rack_travel_mm": 32.0, "samples_per_link": 16,
         "links": ["upper_front", "upper_rear", "lower_front", "lower_rear",
                   "tie_rod"]}
    e = gr._envelope_from_dict(d)
    assert e.link_radius_mm == 0.0 and e.rod_end_radius_mm == 0.0


def test_complete_model_round_trips():
    from suspension import genesis_repro as gr
    from suspension.inverse_genesis import WheelEnvelope, WheelSector
    e = WheelEnvelope(link_radius_mm=7.9, rod_end_radius_mm=10.5,
                      tire_profile=((127.0, 89.0), (180.0, 95.0),
                                    (228.0, 80.0)),
                      sectors=(WheelSector("caliper", 60, 120, 20, 90, -70, 0),))
    e2 = gr._envelope_from_dict(gr._envelope_to_dict(e))
    assert e2.link_radius_mm == 7.9 and e2.rod_end_radius_mm == 10.5
    assert e2.tire_profile == e.tire_profile
    assert e2.sectors[0].label == "caliper"


# --------------------------------------------------------------------------- #
#  Steering lock is a real lock; joints; frame nodes
# --------------------------------------------------------------------------- #
def test_rack_travel_actually_steers_the_wheel():
    """Regression: moving the tie-rod inner in a copy did NOT steer, because
    static toe is a declared alignment and the rod length was re-derived."""
    env = WheelEnvelope(rack_travel_mm=32.0)
    toes = {r: max(abs(s.toe) for s in st) for r, _, st in env._states(_front())}
    assert toes[0.0] < 0.5
    assert toes[32.0] > 15.0 and toes[-32.0] > 15.0


def test_lock_finds_the_wishbone_clash():
    """At real lock the rim swings into the delivered lower-front leg."""
    v, _ = WheelEnvelope(rack_travel_mm=32.0).check(_front())
    assert any(i == "lower_front link" and "rack" in w for i, w, _ in v)
    v0, _ = WheelEnvelope(rack_travel_mm=0.0).check(_front())
    assert not any(i == "lower_front link" for i, _, _ in v0)


def test_joint_swing_is_measured_in_the_housing_frame():
    from suspension.inverse_genesis import joint_swing
    still = joint_swing(_front(), travel_mm=(-0.01, 0.01), n_travel=2)
    assert all(v < 0.1 for k, v in still.items() if k != "_worst")
    locked = joint_swing(_front(), rack_travel_mm=32.0)
    # the tie-rod outer rides the steered upright: large swing at lock
    assert locked["tie_rod.outer"] > 15.0
    # its inner end rides a translating rack: small swing
    assert locked["tie_rod.inner"] < 8.0


def test_joint_swing_limit_is_a_wall_term():
    env = WheelEnvelope(rack_travel_mm=32.0, pickup_radius_mm=127.0,
                        rod_end_radius_mm=0.0, joint_swing_limit_deg=15.0)
    assert any("joint swing" in i for i, _, _ in env.violations(_front()))


def test_node_attachment_wall():
    from suspension.inverse_genesis import NodeAttachment
    hp = _front()
    at = NodeAttachment(nodes=(tuple(hp.lower_front_inner),),
                        points=("lower_front_inner",), max_offset_mm=10.0)
    assert at.violations(hp) == []
    moved = _perturbed(hp, {"lower_front_inner": np.array([40.0, 0.0, 0.0])})
    v = at.violations(moved)
    assert v and v[0][2] == pytest.approx(-30.0, abs=1e-6)


def test_node_wall_refuses_a_stranded_start():
    from suspension.inverse_genesis import NodeAttachment
    hp = _front()
    far = NodeAttachment(nodes=((0.0, 0.0, 0.0),),
                         points=("lower_front_inner",), max_offset_mm=5.0)
    vol = LegalVolume.around(hp, 40.0, points=ALL, node_attachment=far)
    c = genesis_solve(hp, _targets(), vol)
    assert not c.ok and c.node_rejections == 1


def test_node_attachment_round_trips():
    from suspension import genesis_repro as gr
    from suspension.inverse_genesis import NodeAttachment
    hp = _front()
    vol = LegalVolume.around(hp, 40.0, points=ALL, node_attachment=NodeAttachment(
        nodes=((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)), max_offset_mm=30.0))
    vol2 = gr.volume_from_dict(gr.volume_to_dict(vol))
    assert vol2.node_attachment.max_offset_mm == 30.0
    assert len(vol2.node_attachment.nodes) == 2
    assert "node_attachment" not in gr.volume_to_dict(
        LegalVolume.around(hp, 40.0, points=ALL))
