# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The edges of the four core solvers (kinematics, statics, compliance, elastokinematics).

The main suites check the solvers on real corners. These check what happens at the edges: degenerate geometry
(parallel arms, a joint on its own pivot axis, a horizontal kingpin), inputs that must be refused, the fall-back paths
(no rocker, a non-determinate member set, a solve that does not converge) and the parts of the compliance model the
corner studies do not reach (joint linearisation and damping, rear and inboard-drive load cases, required stiffness).
Every test checks an answer, not only that a line ran. Lengths mm, forces N, moments N*mm, angles deg.
"""
import dataclasses
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import compliance as cm, elastokinematics as ek, loadpath as lp     # noqa: E402
from suspension.dynamics import VehicleDynamics, VehicleParams                      # noqa: E402
from suspension.joints import JointCompliance                                       # noqa: E402
from suspension.kinematics import Hardpoints, SuspensionKinematics                  # noqa: E402

LOAD = lp.WheelLoad(Fx=600.0, Fy=-1500.0, Fz=1200.0, Mz=-30000.0)


def _default():
    return Hardpoints.default()


def _no_rocker(hp=None):
    hp = (hp or _default()).copy()
    for n in ("pushrod_outer", "rocker_pivot", "rocker_axis", "rocker_pushrod", "rocker_spring", "spring_inner"):
        setattr(hp, n, None)
    return hp


def _flat(hp=None):
    """Both wishbones with their pivot axes parallel to x: no stagger in side view."""
    hp = (hp or _default()).copy()
    for f, r in (("upper_front_inner", "upper_rear_inner"), ("lower_front_inner", "lower_rear_inner")):
        a, b = np.asarray(getattr(hp, f), float), np.asarray(getattr(hp, r), float)
        m = 0.5 * (a + b)
        setattr(hp, f, np.array([a[0], m[1], m[2]]))
        setattr(hp, r, np.array([b[0], m[1], m[2]]))
    return hp


def _flat_travel(kin):
    """Replace the solver's travel with one where nothing moves vertically (a degenerate stub)."""
    st = kin.static
    kin.solve_at_travel = lambda t, seed=None: st
    return kin


# --------------------------------------------------------------------------- #
#  Kinematics: inputs and compliance hooks
# --------------------------------------------------------------------------- #
def test_a_zero_length_wishbone_is_refused():
    hp = _default()
    hp.lower_rear_inner = np.asarray(hp.lower_outer, float).copy()
    with pytest.raises(ValueError, match="lower rear wishbone has zero length"):
        SuspensionKinematics(hp)


def test_unknown_or_empty_compliance_keys_change_nothing():
    """A length change or a pickup shift on a name the solver does not have, or on a point that is not defined,
    is ignored: the corner solves exactly as the rigid one."""
    hp = _no_rocker()
    rigid = SuspensionKinematics(hp).solve_at_travel(10.0)
    kin = SuspensionKinematics(hp, length_deltas={"not_a_link": 3.0},
                               pickup_deltas={"not_a_point": [5.0, 0, 0], "pushrod_outer": [5.0, 0, 0]})
    s = kin.solve_at_travel(10.0)
    for q in ("camber", "toe", "caster", "kpi"):
        assert getattr(s, q) == pytest.approx(getattr(rigid, q), abs=1e-9)


def test_a_deflected_pickup_moves_the_ball_joint_with_the_link_length_kept():
    """A chassis tab that deflects moves its pickup; the link keeps its unloaded length, so the ball joint is pushed
    by the deflection."""
    hp = _default()
    shift = np.array([0.0, 0.8, 0.0])
    kin = SuspensionKinematics(hp, pickup_deltas={"lower_front_inner": shift})
    s = kin.solve_at_travel(0.0)
    L0 = np.linalg.norm(hp.lower_outer - hp.lower_front_inner)
    assert np.linalg.norm(s.lower_outer - (hp.lower_front_inner + shift)) == pytest.approx(L0, abs=1e-6)
    assert np.allclose(kin.hp.lower_front_inner, hp.lower_front_inner + shift)
    assert np.allclose(hp.lower_front_inner, Hardpoints.default().lower_front_inner)      # caller's copy untouched


# --------------------------------------------------------------------------- #
#  Kinematics: degenerate geometry
# --------------------------------------------------------------------------- #
def test_a_horizontal_kingpin_has_no_scrub_point():
    kin = SuspensionKinematics(_default())
    lo, cp = np.array([0.0, 600.0, 120.0]), np.array([0.0, 605.0, 0.0])
    assert np.isnan(kin._scrub_radius(lo + np.array([0.0, -50.0, 0.0]), lo, cp))


def test_front_view_centre_is_undefined_when_a_joint_sits_on_its_own_pivot_axis():
    kin = SuspensionKinematics(_default())
    hp = kin.hp
    on_axis = 0.5 * (np.asarray(hp.upper_front_inner, float) + np.asarray(hp.upper_rear_inner, float))
    assert np.all(np.isnan(kin._instant_center(on_axis, kin.static.lower_outer)))


def test_parallel_equal_arms_put_the_front_view_centre_at_infinity():
    """Equal, parallel wishbones in front view translate the upright: their instant centre is at infinity."""
    hp = _flat(_no_rocker())
    for n in ("upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner"):
        p = np.asarray(getattr(hp, n), float)
        setattr(hp, n, np.array([p[0], 300.0, 280.0 if n.startswith("upper") else 120.0]))
    hp.upper_outer = np.array([hp.upper_outer[0], 560.0, 280.0])
    hp.lower_outer = np.array([hp.lower_outer[0], 560.0, 120.0])
    kin = SuspensionKinematics(hp)
    assert np.all(np.isnan(kin.static.instant_center))


def test_no_pickup_stagger_means_an_infinite_side_view_swing_arm():
    """With both pivot axes parallel to x, each ball joint moves straight up in side view: the swing arm is
    infinite, the classic zero-anti geometry."""
    kin = SuspensionKinematics(_flat())
    assert np.all(np.isnan(kin._side_view_swing_arm()))
    assert kin.side_view_swing_arm_length() == np.inf


def test_side_view_centre_is_undefined_when_a_joint_sits_on_its_own_pivot_axis():
    kin = SuspensionKinematics(_default())
    hp = kin.hp
    on_axis = 0.5 * (np.asarray(hp.lower_front_inner, float) + np.asarray(hp.lower_rear_inner, float))
    state = dataclasses.replace(kin.static, lower_outer=on_axis)
    assert np.all(np.isnan(kin._side_view_swing_arm(state)))


def test_anti_geometry_reads_zero_not_nan_when_the_patch_has_no_vertical_path():
    kin = _flat_travel(SuspensionKinematics(_default()))
    assert np.isnan(kin._path_slope_xz("contact_patch"))
    assert kin.anti_dive_pct(280.0, 1530.0) == 0.0
    assert kin.anti_squat_pct(280.0, 1530.0) == 0.0
    assert kin.anti_lift_pct(280.0, 1530.0) == 0.0


# --------------------------------------------------------------------------- #
#  Kinematics: rocker and the motion-ratio fall-backs
# --------------------------------------------------------------------------- #
def test_spring_length_needs_a_rocker_and_rebuilds_a_missing_pushrod_point():
    kin = SuspensionKinematics(_default())
    s = kin.solve_at_travel(8.0)
    L, th, ok = kin.spring_length_at(s)
    L2, th2, ok2 = kin.spring_length_at(dataclasses.replace(s, pushrod_outer=None))
    assert ok and ok2 and L2 == pytest.approx(L, abs=1e-9) and th2 == pytest.approx(th, abs=1e-12)
    bare = SuspensionKinematics(_no_rocker())
    assert bare._pushrod_outer_at(s.lower_outer, s.upper_outer, s.tie_rod_outer) is None
    with pytest.raises(ValueError, match="No pushrod/rocker"):
        bare.spring_length_at(bare.static)


def test_motion_ratio_with_explicit_arguments_bypasses_the_cache_and_agrees():
    kin = SuspensionKinematics(_default())
    assert kin.motion_ratio(d=2.0) == pytest.approx(kin.motion_ratio(), rel=2e-3)


def test_direct_acting_proxy_is_the_ball_joints_travel_along_the_spring_axis():
    """Without a rocker, the motion ratio is the lower ball joint's travel along the declared spring axis per mm of
    wheel-centre travel; an offset from the joint changes nothing, since the offset rides with it."""
    hp = _no_rocker()
    kin = SuspensionKinematics(hp)
    assert kin.motion_ratio() == 1.0 and not kin.motion_ratio_is_real()          # nothing declared: 1:1
    si, rp = np.array([0.0, 400.0, 400.0]), np.array([0.0, 520.0, 150.0])
    up, dn = kin.solve_at_travel(5.0), kin.solve_at_travel(-5.0)
    axis = (si - rp) / np.linalg.norm(si - rp)
    expect = abs(np.dot(up.lower_outer - dn.lower_outer, axis)) / (up.wheel_center[2] - dn.wheel_center[2])
    assert kin.motion_ratio(spring_inner=si, rocker_pivot=rp) == pytest.approx(expect, rel=1e-12)
    assert kin.motion_ratio(spring_inner=si, rocker_pivot=rp, push_outer_local=np.array([0.0, -20.0, 10.0])) \
        == pytest.approx(expect, rel=1e-12)
    travels, mr = kin.motion_ratio_curve(n=5)
    assert len(travels) == 5 and mr == [1.0] * 5


def test_a_spring_axis_square_to_the_joint_path_gives_no_motion_ratio_and_no_wheel_rate():
    hp = _no_rocker()
    kin = SuspensionKinematics(hp)
    up, dn = kin.solve_at_travel(5.0), kin.solve_at_travel(-5.0)
    path = up.lower_outer - dn.lower_outer
    perp = np.cross(path, [1.0, 0.0, 0.0])
    hp.rocker_pivot = np.array([0.0, 500.0, 200.0])
    hp.spring_inner = hp.rocker_pivot + 100.0 * perp / np.linalg.norm(perp)
    kin = SuspensionKinematics(hp)
    assert np.isnan(kin.motion_ratio())
    assert np.isnan(kin.wheel_rate(30.0))


def test_motion_ratio_is_undefined_when_the_wheel_does_not_move():
    kin = _flat_travel(SuspensionKinematics(_default()))
    assert np.isnan(kin._compute_motion_ratio())


# --------------------------------------------------------------------------- #
#  Statics: the non-determinate fall-back
# --------------------------------------------------------------------------- #
def test_a_non_determinate_member_set_is_flagged_and_still_solved_by_least_squares(monkeypatch):
    """Seven members over six equations: no unique answer. The solver must say so, flag the conditioning, and still
    return forces that satisfy equilibrium in the least-squares sense rather than fail."""
    kin = SuspensionKinematics(_no_rocker())
    s = kin.static
    real = lp._member_geometry

    def seven(k, st):
        g = real(k, st)
        g["UF2"] = g["UF"]                       # a duplicate leg: a redundant load path
        return g
    monkeypatch.setattr(lp, "_member_geometry", seven)
    mf = lp.solve_member_forces(kin, s, LOAD)
    assert "Expected 6 force members" in mf.note and "poorly conditioned" in mf.note
    assert mf.condition == np.inf
    assert mf.forces["UF"] == pytest.approx(mf.forces["UF2"], rel=1e-6)   # minimum-norm: the duplicate shares
    assert mf.residual < 1e-6 * np.abs(LOAD.force()).sum()


# --------------------------------------------------------------------------- #
#  Compliance: member stiffness, results, linearisation, damping, load cases
# --------------------------------------------------------------------------- #
def test_member_stiffness_refuses_what_it_cannot_compute():
    with pytest.raises(ValueError, match="node_out/node_in"):
        cm.MemberStiffness(flex_body=object()).axial_stiffness(300.0)
    with pytest.raises(ValueError, match="Unknown material"):
        cm.MemberStiffness(material="Unobtainium", od_mm=19.05, wall_mm=0.9).axial_stiffness(300.0)
    with pytest.raises(ValueError, match="no usable stiffness"):
        cm.MemberStiffness().axial_stiffness(300.0)
    with pytest.raises(ValueError, match="neither a link stiffness nor a joint"):
        cm.MemberStiffness().axial_deflection(1000.0, 300.0)


def test_a_rigid_link_puts_all_its_give_in_the_joints():
    """No link stiffness: the member is rigid and every millimetre of give is in its joints, in series."""
    ji, jo = JointCompliance.linear(2000.0), JointCompliance.linear(8000.0)
    ms = cm.MemberStiffness(joint_in=ji, joint_out=jo)
    F = 1600.0
    assert ms.axial_deflection(F, 300.0) == pytest.approx(F / 2000.0 + F / 8000.0)
    assert ms.deflection_breakdown(F, 300.0) == {"link": 0.0, "joint_in": pytest.approx(0.8),
                                                 "joint_out": pytest.approx(0.2)}
    assert ms.tangent_axial_stiffness(F, 300.0) == pytest.approx(1.0 / (1 / 2000.0 + 1 / 8000.0))


def test_an_infinitely_stiff_link_adds_no_give():
    ms = cm.MemberStiffness(k_direct=np.inf, joint_out=JointCompliance.linear(5000.0))
    assert ms.axial_deflection(1000.0, 300.0) == pytest.approx(0.2)
    assert ms.deflection_breakdown(1000.0, 300.0)["link"] == 0.0
    assert ms.tangent_axial_stiffness(1000.0, 300.0) == pytest.approx(5000.0)


def _corner(**kw):
    return cm.CompliantCorner.uniform_tube(_default(), od_mm=12.0, wall_mm=0.7, **kw)


def test_compliance_result_reports_the_differences_it_names():
    res = _corner().solve(LOAD)
    assert res.compliance_caster == res.compliant.caster - res.rigid.caster
    assert np.allclose(res.wheel_center_shift_mm, res.compliant.wheel_center - res.rigid.wheel_center)
    s = res.summary()
    assert s["compliance_toe_deg"] == res.compliance_toe and s["compliance_caster_deg"] == res.compliance_caster
    assert s["wheel_center_shift_mm"] == res.wheel_center_shift_mm.tolist() and s["converged"] == res.converged


def test_a_pushrod_or_unknown_member_in_the_stiffness_map_moves_no_wheel_geometry():
    """The pushrod's stretch moves the rocker, not the wheel, and a name the corner does not have is ignored: the
    loaded geometry is the same, and the pushrod's give is still reported."""
    base = _corner().solve(LOAD)
    cc = _corner()
    cc.stiffness["PR"] = cm.MemberStiffness(k_direct=5000.0)
    cc.stiffness["XX"] = cm.MemberStiffness(k_direct=5000.0)
    res = cc.solve(LOAD)
    assert res.compliance_toe == pytest.approx(base.compliance_toe, abs=1e-12)
    assert res.member_deflection["PR"] == pytest.approx(res.member_forces["PR"] / 5000.0)
    assert "XX" not in res.member_forces


def test_linearised_rates_match_the_joint_laws_at_the_working_force():
    rod = JointCompliance.cubic(4000.0, 50.0, c_viscous=0.2, loss_factor=0.1)
    cc = _corner(rod_end=rod)
    cc.stiffness["XX"] = cm.MemberStiffness(k_direct=5000.0)       # a name the corner has no force for: skipped
    res = cc.solve(LOAD)
    out = cc.linearized_rates(LOAD, freq_hz=15.0)
    assert "XX" not in out and set(out) == set(res.member_forces) & set(cc.stiffness)
    for m, e in out.items():
        T = res.member_forces[m]
        assert e["force_N"] == pytest.approx(T)
        if m != "TR":
            lin = rod.linearize(about_force_N=T, freq_hz=15.0)
            assert e["joints"]["out"] == pytest.approx(lin)
            k = lin["tangent_stiffness_N_per_mm"]
            assert lin["equivalent_viscous_c_N_s_per_mm"] == pytest.approx(0.2 + 0.1 * k / (2 * np.pi * 15.0))
        else:
            assert e["joints"] == {}


def test_damping_summary_adds_the_energy_of_every_joint():
    """Energy per cycle: viscous pi*c*omega*X^2 plus structural pi*eta*k*X^2, summed over the joints that exist."""
    rod = JointCompliance.linear(4000.0, c_viscous=0.2, loss_factor=0.1)
    cc = _corner(rod_end=rod)
    d = cc.damping_summary(LOAD, amplitude_mm=0.4, freq_hz=12.0)
    one = np.pi * 0.2 * (2 * np.pi * 12.0) * 0.4 ** 2 + np.pi * 0.1 * 4000.0 * 0.4 ** 2
    assert set(d["energy_per_cycle_mJ"]) == {f"{m}_out" for m in ("UF", "UR", "LF", "LR")}
    assert all(v == pytest.approx(one) for v in d["energy_per_cycle_mJ"].values())
    assert d["total_energy_per_cycle_mJ"] == pytest.approx(4 * one)


def test_rear_and_inboard_drive_wheel_loads_follow_the_load_transfer():
    veh = VehicleDynamics(VehicleParams(), front_kin=SuspensionKinematics(_default()))
    loads, _ = veh.lateral_load_transfer(1.2)
    outer = cm.corner_wheel_load(veh, "rear", 1.2, outer=True, long_g=-0.5, inboard_drive=True)
    inner = cm.corner_wheel_load(veh, "rear", 1.2, outer=False)
    assert outer.Fz == pytest.approx(loads.rr) and inner.Fz == pytest.approx(loads.rl)
    assert outer.Fx == 0.0 and outer.Fx_wc == pytest.approx(-0.5 * loads.rr)       # traction through the shaft
    assert outer.Fy == pytest.approx(-1.2 * loads.rr)


# --------------------------------------------------------------------------- #
#  Elastokinematics: inputs, serialisation, convergence, required stiffness
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("make, match", [
    (lambda: ek.StiffnessField(default_n_per_mm=0.0), "must be > 0"),
    (lambda: ek.StiffnessField(coupled=np.eye(3)), "15 x 15"),
    (lambda: ek.StiffnessField(coupled=-np.eye(15)), "positive definite"),
    (lambda: ek.StiffnessField(provenance="guessed"), "provenance"),
    (lambda: ek.StiffnessField(per_point={"tie_rod_inner": -5.0}), "must be > 0"),
    (lambda: ek.StiffnessField(per_point={"tie_rod_inner": np.ones(4)}), "scalar or 3x3"),
    (lambda: ek.StiffnessField.from_pull_tests(Hardpoints.default(), {"wheel_center": (1.0, 1.0)}), "unknown pickup"),
    (lambda: ek.StiffnessField.from_pull_tests(Hardpoints.default(), {"tie_rod_inner": (0.0, 1.0)}), "> 0"),
    (lambda: ek.FrameTwist(kt_Nm_per_deg=0.0, torque_Nm=100.0, separation_mm=500.0), "must be > 0"),
    (lambda: ek.FrameTwist(kt_Nm_per_deg=2000.0, torque_Nm=100.0, separation_mm=-1.0), ">= 0"),
    (lambda: ek.ElastoSpec(n_steps=0), "n_steps"),
    (lambda: ek.ElastoSpec(joint_lash_mm={"XX": 0.1}), "unknown member"),
    (lambda: ek.ElastoSpec(joint_lash_mm={"TR": -0.1}), ">= 0"),
])
def test_elastokinematic_inputs_that_must_be_refused(make, match):
    with pytest.raises(ValueError, match=match):
        make()


def test_a_frame_twist_must_be_a_frame_twist():
    with pytest.raises(TypeError, match="FrameTwist"):
        ek.ElastoSpec(twist={"kt_Nm_per_deg": 2000.0})


def test_scaling_a_stiffness_field_scales_every_part_and_its_flexibility_inversely():
    C = np.eye(15) * 1e-4
    f = ek.StiffnessField(default_n_per_mm=20000.0, per_point={"tie_rod_inner": 5000.0}, coupled=C)
    g = f.scaled(2.0)
    assert g.default_n_per_mm == 40000.0 and g.matrix("tie_rod_inner") == pytest.approx(10000.0 * np.eye(3))
    assert np.allclose(g.coupled, C / 2.0) and g.provenance == f.provenance


def test_only_analytic_tubes_are_written_to_a_manifest():
    spec = ek.ElastoSpec(members={"TR": cm.MemberStiffness(k_direct=8000.0)})
    with pytest.raises(ValueError, match="analytic tube members only"):
        spec.to_dict()


def test_lash_on_a_member_with_none_declared_adds_no_deadband():
    hp = _default()
    with_zero = ek.lash_deadband(hp, {"TR": 0.05, "UF": 0.0})
    only_tr = ek.lash_deadband(hp, {"TR": 0.05})
    assert with_zero == only_tr and any(v > 0 for v in only_tr.values())


def test_a_solve_that_runs_out_of_iterations_says_so():
    spec = ek.ElastoSpec(stiffness=ek.StiffnessField(default_n_per_mm=2000.0), Fy=-1800.0, Fz=1200.0, n_steps=1)
    res = ek.solve_elastokinematic(_default(), spec, tol_mm=1e-15, max_iter=1)
    assert res.converged is False


def test_link_give_is_added_to_the_linear_estimate():
    """With link members declared, the linear estimate is the pickup Jacobian plus the links' own stretch, and it
    stays close to the full non-linear solve."""
    tubes = {m: cm.MemberStiffness(material="Steel 4130", od_mm=12.0, wall_mm=0.7) for m in ("UF", "UR", "LF", "LR", "TR")}
    spec = ek.ElastoSpec(Fy=-1800.0, Fz=1200.0, members=tubes)
    res = ek.solve_elastokinematic(_default(), spec)
    assert res.link_deflection_mm and res.converged
    for c in res.change:
        assert res.linear[c] == pytest.approx(res.change[c], abs=0.02 + 0.05 * abs(res.change[c]))


def test_required_stiffness_scales_the_change_onto_the_budget():
    """Change is inversely proportional to uniform stiffness, so holding a channel to a budget needs the current
    stiffness times change over budget; at that stiffness the solve lands on the budget."""
    hp = _default()
    spec = ek.ElastoSpec(stiffness=ek.StiffnessField(default_n_per_mm=20000.0), Fy=-1800.0, Fz=1200.0)
    ch = abs(ek.solve_elastokinematic(hp, spec).change["toe"])
    k = ek.required_uniform_stiffness(hp, spec, "toe", budget_deg=ch / 2.0)
    assert k == pytest.approx(40000.0, rel=1e-9)
    at_k = ek.ElastoSpec(stiffness=ek.StiffnessField(default_n_per_mm=k), Fy=-1800.0, Fz=1200.0)
    assert abs(ek.solve_elastokinematic(hp, at_k).change["toe"]) == pytest.approx(ch / 2.0, rel=0.02)
    unloaded = ek.ElastoSpec(stiffness=ek.StiffnessField(default_n_per_mm=20000.0))
    assert ek.required_uniform_stiffness(hp, unloaded, "toe", budget_deg=0.01) == 0.0


def test_a_compliance_solve_with_no_iterations_returns_the_rigid_corner_and_says_so():
    res = _corner().solve(LOAD, max_iter=0)
    assert not res.converged and res.iterations == 0
    assert res.compliance_toe == 0.0 and "did not fully converge" in res.note


def test_an_elastokinematic_spec_survives_a_round_trip_through_a_manifest():
    """Every field a manifest carries comes back: the coupled flexibility, per-point stiffness, shaft force, joint
    lash, frame twist and analytic tube members (with a tab)."""
    C = np.eye(15) * 2e-5
    spec = ek.ElastoSpec(stiffness=ek.StiffnessField(default_n_per_mm=25000.0, per_point={"tie_rod_inner": 9000.0},
                                                     coupled=C, provenance="fea"),
                         Fx=300.0, Fy=-1700.0, Fz=1250.0, Mz=-20000.0, n_steps=3, Fx_wc=-400.0,
                         joint_lash_mm={"TR": 0.04, "LF": 0.0},
                         twist=ek.FrameTwist(kt_Nm_per_deg=2000.0, torque_Nm=900.0, separation_mm=600.0),
                         members={"TR": cm.MemberStiffness(material="Steel 4130", od_mm=15.875, wall_mm=0.889,
                                                           k_tab=40000.0)})
    d = spec.to_dict()
    back = ek.ElastoSpec.from_dict(d)
    assert back.to_dict() == d
    assert np.allclose(back.stiffness.coupled, C) and back.twist == spec.twist
    m = back.members["TR"]
    assert (m.material, m.od_mm, m.wall_mm, m.k_tab) == ("Steel 4130", 15.875, 0.889, 40000.0)
    assert spec.wheel_load_magnitude() == pytest.approx(np.linalg.norm([300.0 - 400.0, -1700.0, 1250.0]))


def test_frame_twist_toe_is_the_rack_shift_times_the_corners_own_tie_rod_sensitivity():
    """Twist over the rack-to-tie-rod separation moves the rack by angle x lever; toe per mm of tie-rod inner motion
    is measured here by moving the pickup along the tie rod and re-solving, independently of the Jacobian."""
    hp = _default()
    tw = ek.FrameTwist(kt_Nm_per_deg=2000.0, torque_Nm=900.0, separation_mm=600.0, lever_mm=200.0,
                       wheelbase_mm=1630.0)
    u = hp.tie_rod_outer - hp.tie_rod_inner
    u = u / np.linalg.norm(u)
    h = 0.01
    toe = lambda s: SuspensionKinematics(hp, pickup_deltas={"tie_rod_inner": s * u}).static.toe
    per_mm = abs(toe(h) - toe(-h)) / (2 * h)
    expect = np.radians((900.0 / 2000.0) * 600.0 / 1630.0) * 200.0 * per_mm
    assert tw.toe_deg(hp) == pytest.approx(expect, rel=1e-4)


def test_a_coupled_flexibility_equal_to_the_point_stiffnesses_gives_the_same_answer():
    """A 15 x 15 frame flexibility with no cross terms, each block the inverse of a pickup's stiffness, is the same
    structure as the per-point field, so the deflected corner must be the same."""
    hp = _default()
    per = ek.StiffnessField(default_n_per_mm=15000.0, per_point={"tie_rod_inner": 6000.0})
    C = np.zeros((15, 15))
    for i, p in enumerate(ek.PICKUPS):
        C[3 * i:3 * i + 3, 3 * i:3 * i + 3] = per.compliance(p)
    load = dict(Fy=-1800.0, Fz=1200.0, Mz=-30000.0)
    a = ek.solve_elastokinematic(hp, ek.ElastoSpec(stiffness=per, **load), jacobian=False)
    b = ek.solve_elastokinematic(hp, ek.ElastoSpec(stiffness=ek.StiffnessField(default_n_per_mm=15000.0, coupled=C),
                                                   **load), jacobian=False)
    for c in a.change:
        assert b.change[c] == pytest.approx(a.change[c], abs=1e-9)
