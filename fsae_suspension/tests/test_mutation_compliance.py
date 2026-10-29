"""Tests aimed at the mutants the compliance suites let through (elastokinematics.py and compliance.py).

The mutation gate (scripts/mutation_gate.py, group "compliance") breaks the two modules on purpose and counts how many
breaks the tests notice. These checks close what got through: the defaults a saved spec falls back on, every refusal
message, the boundary of each "must be positive" check, the pull-test stiffness in its own link frame, the load-stepped
solve as a fixed point (its pickup deflections are the compliance times the forces on the deflected geometry), and the
series sums of link, tab and joint. Each is an independent calculation, not a re-run of the code under test.
"""
import numpy as np
import pytest

from suspension import compliance as cm
from suspension import elastokinematics as ek
from suspension import loadpath as lp
from suspension.dynamics import VehicleDynamics, VehicleParams
from suspension.joints import JointCompliance
from suspension.kinematics import Hardpoints, SuspensionKinematics

pytestmark = pytest.mark.filterwarnings("ignore")

HP = Hardpoints.default()
LOAD = dict(Fx=400.0, Fy=1918.0, Fz=1235.0, Mz=15000.0)
OUTER = {"upper_front_inner": "upper_outer", "upper_rear_inner": "upper_outer",
         "lower_front_inner": "lower_outer", "lower_rear_inner": "lower_outer",
         "tie_rod_inner": "tie_rod_outer"}


def _axis(pickup):
    u = np.asarray(getattr(HP, OUTER[pickup]), float) - np.asarray(getattr(HP, pickup), float)
    return u / np.linalg.norm(u)


# ------------------------------------------------------------------ stiffness field
def test_a_saved_field_and_spec_fall_back_on_the_documented_defaults():
    f = ek.StiffnessField.from_dict({})
    assert (f.default_n_per_mm, f.per_point, f.provenance, f.coupled) == (30000.0, {}, "declared", None)
    s = ek.ElastoSpec.from_dict({})
    assert (s.Fx, s.Fy, s.Fz, s.Mz, s.Fx_wc, s.n_steps) == (0.0, 0.0, 0.0, 0.0, 0.0, 5)
    assert (s.joint_lash_mm, s.twist, s.members, s.stiffness.default_n_per_mm) == ({}, None, {}, 30000.0)


def test_the_field_round_trips_as_floats():
    C = np.diag(np.full(15, 2e-5))
    f = ek.StiffnessField(20000, per_point={"tie_rod_inner": 9000, "upper_rear_inner": np.diag([1, 2, 3]) * 1000},
                          provenance="fea", coupled=C)
    d = f.to_dict()
    assert isinstance(d["per_point"]["tie_rod_inner"], float)
    assert all(isinstance(x, float) for row in d["per_point"]["upper_rear_inner"] for x in row)
    assert all(isinstance(x, float) for row in d["coupled"] for x in row)
    g = ek.StiffnessField.from_dict(d)
    assert (g.default_n_per_mm, g.provenance) == (20000.0, "fea")
    assert g.per_point["upper_rear_inner"] == pytest.approx(np.diag([1, 2, 3]) * 1000)
    assert g.coupled == pytest.approx(C)
    assert g.per_point["tie_rod_inner"].dtype == np.float64 and g.coupled.dtype == np.float64


def test_scaling_a_field_scales_stiffness_and_divides_flexibility():
    f = ek.StiffnessField(20000, per_point={"tie_rod_inner": 9000}, provenance="measured",
                          coupled=np.diag(np.full(15, 2e-5)))
    g = f.scaled(2.0)
    assert (g.default_n_per_mm, g.provenance) == (40000.0, "measured")
    assert g.per_point["tie_rod_inner"] == pytest.approx(18000.0)
    assert g.coupled == pytest.approx(np.diag(np.full(15, 1e-5)))
    assert ek.StiffnessField(1000.0).scaled(3.0).coupled is None


@pytest.mark.parametrize("kw,message", [
    (dict(default_n_per_mm=0.0), "StiffnessField.default_n_per_mm must be > 0."),
    (dict(coupled=np.eye(3)), "StiffnessField.coupled must be 15 x 15."),
    (dict(coupled=-np.eye(15)), "StiffnessField.coupled must be symmetric positive definite (Maxwell-Betti)."),
    (dict(provenance="guessed"), "provenance must be 'declared', 'measured' or 'fea'."),
    (dict(per_point={"chassis": 1.0}), "StiffnessField: unknown pickup 'chassis'."),
    (dict(per_point={"tie_rod_inner": 0.0}), "stiffness at tie_rod_inner must be > 0."),
    (dict(per_point={"tie_rod_inner": np.ones(2)}), "stiffness at tie_rod_inner must be a scalar or 3x3."),
    (dict(per_point={"tie_rod_inner": np.diag([1.0, 1.0, 0.0])}),
     "stiffness matrix at tie_rod_inner is not positive definite."),
])
def test_the_field_refuses_what_is_not_a_stiffness(kw, message):
    with pytest.raises(ValueError) as e:
        ek.StiffnessField(**kw)
    assert str(e.value) == message


def test_small_but_positive_stiffness_is_accepted():
    f = ek.StiffnessField(0.5, per_point={"tie_rod_inner": 0.5, "upper_rear_inner": np.diag([0.5, 0.5, 0.5])})
    assert f.matrix("tie_rod_inner") == pytest.approx(0.5 * np.eye(3))
    assert f.compliance("upper_rear_inner") == pytest.approx(2.0 * np.eye(3))
    assert f.matrix("lower_front_inner") == pytest.approx(0.5 * np.eye(3))


def test_the_coupled_flexibility_must_be_symmetric_to_its_stated_tolerance():
    def field(a, b, diag=1e-5):
        C = np.diag(np.full(15, diag))
        C[0, 1], C[1, 0] = a, b
        return ek.StiffnessField(coupled=C)
    with pytest.raises(ValueError):
        field(1e-6, 1.000003e-6)            # 3e-12 apart: beyond 1e-6 relative + 1e-12 absolute
    with pytest.raises(ValueError):
        field(0.0, 5e-12)                   # beyond the 1e-12 absolute floor
    f = field(1e-6, 1.0000005e-6)          # 5e-13 apart: inside, and averaged
    assert f.coupled[0, 1] == f.coupled[1, 0] == pytest.approx(1.00000025e-6, rel=1e-12)


# ------------------------------------------------------------------ pull tests
def test_pull_tests_set_the_stiffness_along_each_link_and_across_it():
    tests = {p: (1000.0 + 100 * i, 0.04 + 0.01 * i) for i, p in enumerate(OUTER)}
    f = ek.StiffnessField.from_pull_tests(HP, tests, default_n_per_mm=12345.0)
    assert (f.provenance, f.default_n_per_mm) == ("measured", 12345.0)
    for p, (force, defl) in tests.items():
        K, u = f.matrix(p), _axis(p)
        ka = force / defl
        assert u @ K @ u == pytest.approx(ka)
        v = np.cross(u, [0.0, 0.0, 1.0])
        v /= np.linalg.norm(v)
        assert v @ K @ v == pytest.approx(ka)                       # transverse defaults to the axial value
        assert K @ u == pytest.approx(ka * u)
    g = ek.StiffnessField.from_pull_tests(HP, {"tie_rod_inner": (900.0, 0.03)}, transverse_n_per_mm=5000.0)
    K, u = g.matrix("tie_rod_inner"), _axis("tie_rod_inner")
    v = np.cross(u, [0.0, 0.0, 1.0])
    v /= np.linalg.norm(v)
    assert (u @ K @ u, v @ K @ v) == pytest.approx((30000.0, 5000.0))
    assert g.default_n_per_mm == 30000.0
    assert ek.StiffnessField.from_pull_tests(HP, {"tie_rod_inner": (0.5, 0.001)}).matrix("tie_rod_inner") \
        @ _axis("tie_rod_inner") == pytest.approx(500.0 * _axis("tie_rod_inner"))
    for bad in ((1000.0, 0.0), (0.0, 0.1)):
        with pytest.raises(ValueError, match="from_pull_tests: tie_rod_inner needs force and deflection > 0."):
            ek.StiffnessField.from_pull_tests(HP, {"tie_rod_inner": bad})
    with pytest.raises(ValueError, match="from_pull_tests: unknown pickup 'chassis'."):
        ek.StiffnessField.from_pull_tests(HP, {"chassis": (1.0, 1.0)})


# ------------------------------------------------------------------ spec, twist, lash
def test_spec_and_twist_refuse_bad_input_with_their_messages():
    for kw, err, msg in [
        (dict(n_steps=0), ValueError, "ElastoSpec.n_steps must be >= 1."),
        (dict(joint_lash_mm={"XX": 0.1}), ValueError, "ElastoSpec.joint_lash_mm: unknown member 'XX'."),
        (dict(joint_lash_mm={"TR": -0.1}), ValueError, "ElastoSpec.joint_lash_mm values must be >= 0 (mm)."),
        (dict(twist=5.0), TypeError, "ElastoSpec.twist takes a FrameTwist."),
    ]:
        with pytest.raises(err) as e:
            ek.ElastoSpec(**kw)
        assert str(e.value) == msg
    for kw, msg in [(dict(kt_Nm_per_deg=0.0), "FrameTwist.kt_Nm_per_deg must be > 0."),
                    (dict(wheelbase_mm=0.0), "FrameTwist.wheelbase_mm must be > 0."),
                    (dict(separation_mm=-1.0), "FrameTwist.separation_mm must be >= 0."),
                    (dict(lever_mm=-1.0), "FrameTwist.lever_mm must be >= 0.")]:
        base = dict(kt_Nm_per_deg=1000.0, torque_Nm=500.0, separation_mm=300.0)
        with pytest.raises(ValueError) as e:
            ek.FrameTwist(**{**base, **kw})
        assert str(e.value) == msg
    t = ek.FrameTwist(kt_Nm_per_deg=0.5, torque_Nm=1.0, separation_mm=0.0, lever_mm=0.5, wheelbase_mm=0.5)
    assert t.separation_mm == 0.0
    ek.FrameTwist(kt_Nm_per_deg=1.0, torque_Nm=1.0, separation_mm=0.5)


def test_a_spec_load_scales_every_component():
    s = ek.ElastoSpec(Fx=100.0, Fy=-200.0, Fz=300.0, Mz=4000.0, Fx_wc=-50.0)
    w = s.load(0.5)
    assert (w.Fx, w.Fy, w.Fz, w.Mz, w.Fx_wc) == (50.0, -100.0, 150.0, 2000.0, -25.0)


def test_spec_serialisation_keeps_only_analytic_tubes():
    tube = cm.MemberStiffness(material="Steel 4130", od_mm=15.875, wall_mm=0.889, k_tab=8000.0)
    d = ek.ElastoSpec(members={"TR": tube, "LF": cm.MemberStiffness(material="Steel 4130", od_mm=19.05,
                                                                    wall_mm=0.9)}).to_dict()
    assert d["members"] == {"LF": {"material": "Steel 4130", "od_mm": 19.05, "wall_mm": 0.9},
                            "TR": {"material": "Steel 4130", "od_mm": 15.875, "wall_mm": 0.889, "k_tab": 8000.0}}
    j = JointCompliance.linear(5000.0)
    for bad in (cm.MemberStiffness(od_mm=19.05, wall_mm=0.9),
                cm.MemberStiffness(material="Steel 4130", wall_mm=0.9),
                cm.MemberStiffness(material="Steel 4130", od_mm=19.05),
                cm.MemberStiffness(material="Steel 4130", od_mm=19.05, wall_mm=0.9, k_direct=1.0),
                cm.MemberStiffness(material="Steel 4130", od_mm=19.05, wall_mm=0.9, flex_body=object()),
                cm.MemberStiffness(material="Steel 4130", od_mm=19.05, wall_mm=0.9, joint_in=j),
                cm.MemberStiffness(material="Steel 4130", od_mm=19.05, wall_mm=0.9, joint_out=j)):
        with pytest.raises(ValueError) as e:
            ek.ElastoSpec(members={"TR": bad}).to_dict()
        assert str(e.value) == "ElastoSpec.to_dict serialises analytic tube members only."


def test_lash_deadband_is_the_sum_of_each_members_lash_times_its_sensitivity():
    def sens(key, h=0.002):
        a = SuspensionKinematics(HP, length_deltas={key: h}).static
        b = SuspensionKinematics(HP, length_deltas={key: -h}).static
        return np.abs(np.array([a.camber - b.camber, a.toe - b.toe, a.caster - b.caster, a.kpi - b.kpi]) / (2 * h))
    lash = {"UF": 0.0, "TR": 0.05, "LF": 0.02}             # a zero first: it is skipped, not the end of the list
    want = sens("tie") * 0.05 + sens("lower_f") * 0.02
    got = ek.lash_deadband(HP, lash)
    assert [got[c] for c in ek.CHANNELS] == pytest.approx(want, rel=1e-4, abs=1e-9)
    assert got == ek.lash_deadband(HP, lash, h=0.01)


# ------------------------------------------------------------------ the load-stepped solve
def _soft_spec(**kw):
    members = {m: cm.MemberStiffness(material="Steel 4130", od_mm=12.7, wall_mm=0.7) for m in ("TR", "LF")}
    return ek.ElastoSpec(stiffness=ek.StiffnessField(2000.0, per_point={"tie_rod_inner": 1500.0}),
                         members=members, n_steps=3, **{**LOAD, **kw})


def test_the_solved_corner_is_a_fixed_point_of_force_and_deflection():
    spec = _soft_spec()
    r = ek.solve_elastokinematic(HP, spec, tol_mm=1e-10, jacobian=False)
    assert r.converged and len(r.iterations) == 3 and all(i >= 1 for i in r.iterations)
    pick = {p: np.asarray(r.pickup_deflection_mm[p]) for p in ek.PICKUPS}
    lens = {ek._MEMBER_TO_LENGTHKEY[m]: d for m, d in r.link_deflection_mm.items()}
    kin = SuspensionKinematics(HP, length_deltas=lens, pickup_deltas=pick)
    mf = lp.solve_member_forces(kin, kin.solve_at_travel(0.0), spec.load(1.0))
    for p in ek.PICKUPS:
        assert pick[p] == pytest.approx(spec.stiffness.compliance(p) @ np.asarray(mf.pickup_forces[p]), abs=1e-8)
    assert set(r.link_deflection_mm) == {"TR", "LF"}
    for m, d in r.link_deflection_mm.items():
        p = ek.MEMBER_PICKUP[m]
        L = np.linalg.norm(np.asarray(mf.outboard[m]) - np.asarray(getattr(HP, p)))
        assert d == pytest.approx(spec.members[m].axial_deflection(mf.forces[m], L), abs=1e-9)
    assert r.member_forces == pytest.approx({m: mf.forces[m] for m in ek.MEMBER_PICKUP}, rel=1e-6)
    assert r.all_member_forces == pytest.approx(dict(mf.forces), rel=1e-6) and "PR" in r.all_member_forces
    assert r.provenance == "declared" and (r.linear, r.linearity_error) == ({}, {})
    st = kin.solve_at_travel(0.0)
    rigid = SuspensionKinematics(HP).static
    assert r.change["toe"] == pytest.approx(st.toe - rigid.toe, abs=1e-9)
    full = ek.solve_elastokinematic(HP, spec, tol_mm=1e-10)
    assert full.pickup_deflection_mm == pytest.approx(r.pickup_deflection_mm)
    assert full.member_forces == pytest.approx(r.member_forces) and full.provenance == "declared"
    assert full.iterations == r.iterations and full.link_deflection_mm == pytest.approx(r.link_deflection_mm)
    assert full.all_member_forces == pytest.approx(r.all_member_forces)
    assert set(full.linear) == set(full.linearity_error) == set(ek.CHANNELS)


def test_each_load_step_iterates_at_most_max_iter_times(monkeypatch):
    calls = []
    real = lp.solve_member_forces
    monkeypatch.setattr(ek._lp, "solve_member_forces", lambda *a: calls.append(1) or real(*a))
    r = ek.solve_elastokinematic(HP, _soft_spec(), tol_mm=0.0, max_iter=3, jacobian=False)
    assert len(calls) == 9 and r.iterations == [3, 3, 3] and r.converged is False
    calls.clear()
    ek.solve_elastokinematic(HP, ek.ElastoSpec(n_steps=1, Fz=1000.0), tol_mm=0.0, jacobian=False)
    assert len(calls) == 40                                   # the default max_iter


def test_required_stiffness_scales_the_measured_change_to_the_budget():
    spec = ek.ElastoSpec(stiffness=ek.StiffnessField(20000.0), n_steps=2, **LOAD)
    ch = abs(ek.solve_elastokinematic(HP, ek.ElastoSpec(stiffness=ek.StiffnessField(20000.0), n_steps=2,
                                                        **LOAD)).change["toe"])
    assert ek.required_uniform_stiffness(HP, spec, "toe", 0.01) == pytest.approx(20000.0 * ch / 0.01, rel=1e-12)
    assert ek.required_uniform_stiffness(HP, ek.ElastoSpec(), "toe", 0.0) == 0.0


def test_the_jacobian_default_step_is_half_a_tenth_of_a_millimetre():
    pts = ("tie_rod_inner",)
    assert np.array_equal(ek.compliance_jacobian(HP, points=pts), ek.compliance_jacobian(HP, points=pts, h=0.05))


# ------------------------------------------------------------------ member stiffness
def test_member_stiffness_sources_and_the_tab_in_series():
    assert cm.MemberStiffness(k_direct=1000.0, k_tab=3000.0).axial_stiffness(300.0) == pytest.approx(750.0)
    assert cm.MemberStiffness(k_direct=1000.0, k_tab=0.5).axial_stiffness(300.0) == pytest.approx(1 / (1e-3 + 2.0))
    assert cm.MemberStiffness(k_direct=1000.0, k_tab=0.0).axial_stiffness(300.0) == 1000.0
    for bad in (cm.MemberStiffness(material="Steel 4130", od_mm=19.05),
                cm.MemberStiffness(od_mm=19.05, wall_mm=0.9),
                cm.MemberStiffness(material="Steel 4130", wall_mm=0.9)):
        assert not bad._has_link()
        with pytest.raises(ValueError) as e:
            bad.axial_stiffness(300.0)
        assert str(e.value) == "MemberStiffness has no usable stiffness definition."
    with pytest.raises(ValueError) as e:
        cm.MemberStiffness(flex_body=object(), node_out="a").axial_stiffness(300.0)
    assert str(e.value) == "flex_body given without node_out/node_in names."


def test_link_and_joints_add_their_give_in_series():
    j = JointCompliance.linear(2000.0)
    ms = cm.MemberStiffness(k_direct=1000.0, joint_in=j, joint_out=j)
    assert ms.axial_deflection(600.0, 300.0) == pytest.approx(0.6 + 0.3 + 0.3)
    assert ms.deflection_breakdown(600.0, 300.0) == pytest.approx({"link": 0.6, "joint_in": 0.3, "joint_out": 0.3})
    assert ms.tangent_axial_stiffness(600.0, 300.0) == pytest.approx(500.0)
    soft = cm.MemberStiffness(k_direct=0.5)
    assert soft.axial_deflection(1.0, 300.0) == 2.0 and soft.tangent_axial_stiffness(1.0, 300.0) == 0.5
    assert soft.deflection_breakdown(1.0, 300.0) == {"link": 2.0, "joint_in": 0.0, "joint_out": 0.0}
    for k in (0.0, -5.0):                          # a non-positive link rate is not a stiffness: no give from it
        m = cm.MemberStiffness(k_direct=k, joint_in=j)
        assert m.axial_deflection(600.0, 300.0) == pytest.approx(0.3)
        assert m.deflection_breakdown(600.0, 300.0)["link"] == 0.0
        assert m.tangent_axial_stiffness(600.0, 300.0) == pytest.approx(2000.0)
    loose = cm.MemberStiffness(k_direct=1000.0, joint_in=JointCompliance.freeplay(0.1, 5000.0))
    assert loose.tangent_axial_stiffness(0.0, 300.0) == 0.0         # inside the lash the joint has no rate
    assert cm.MemberStiffness().tangent_axial_stiffness(0.0, 300.0) == np.inf
    with pytest.raises(ValueError) as e:
        cm.MemberStiffness().axial_deflection(1.0, 300.0)
    assert str(e.value) == "MemberStiffness has neither a link stiffness nor a joint — nothing to deflect."


# ------------------------------------------------------------------ the compliant corner
def test_the_tube_builders_put_each_joint_where_they_say():
    b, r = JointCompliance.rubber_bushing(), JointCompliance.spherical_bearing()
    tb, tr = JointCompliance.linear(4000.0), JointCompliance.linear(9000.0)
    cc = cm.CompliantCorner.uniform_tube(HP, tie_od_mm=15.0, tie_wall_mm=1.2, k_tab=7000.0, bushing=b, rod_end=r,
                                         tie_bushing=tb, tie_rod_end=tr)
    for m in ("UF", "UR", "LF", "LR"):
        s = cc.stiffness[m]
        assert (s.material, s.od_mm, s.wall_mm, s.k_tab, s.joint_in, s.joint_out) == \
            ("Steel 4130", 19.05, 0.9, 7000.0, b, r)
    t = cc.stiffness["TR"]
    assert (t.od_mm, t.wall_mm, t.k_tab, t.joint_in, t.joint_out) == (15.0, 1.2, None, tb, tr)
    plain = cm.CompliantCorner.uniform_tube(HP).stiffness["TR"]
    assert (plain.od_mm, plain.wall_mm, plain.joint_in, plain.joint_out) == (19.05, 0.9, None, None)
    wb = cm.CompliantCorner.with_bushings(HP, bushing=b, rod_end=r, material="Al 7075-T6", od_mm=16.0, wall_mm=1.5)
    for m in ("UF", "TR"):
        s = wb.stiffness[m]
        assert (s.material, s.od_mm, s.wall_mm, s.joint_in, s.joint_out) == ("Al 7075-T6", 16.0, 1.5, b, r)
    d = cm.CompliantCorner.with_bushings(HP, bushing=b).stiffness["LF"]
    assert (d.material, d.od_mm, d.wall_mm, d.joint_out) == ("Steel 4130", 19.05, 0.9, None)


def _corner():
    return cm.CompliantCorner.uniform_tube(HP, od_mm=12.7, wall_mm=0.7, rod_end=JointCompliance.linear(20000.0))


def test_the_compliance_loop_reports_what_it_did():
    load = lp.WheelLoad(**LOAD)
    cc = _corner()
    r = cc.solve(load)
    assert r.load is load and r.converged is True and r.iterations > 1
    assert r.iterations == cc.solve(load, tol_deg=1e-4).iterations
    assert r.note == "pushrod on the lower wishbone: two-body solution"
    assert r.member_forces["PR"] == pytest.approx(lp.solve_member_forces(
        SuspensionKinematics(HP), SuspensionKinematics(HP).static, load).tension("PR"), rel=0.02)
    short = cc.solve(load, max_iter=3, tol_deg=0.0)
    assert short.converged is False and short.iterations == 3
    assert short.note == ("pushrod on the lower wishbone: two-body solution  Compliance loop did not fully converge "
                          "in 3 iterations (still useful, but treat the last digit with care).")
    assert set(r.summary()) == {"compliance_toe_deg", "compliance_camber_deg", "compliance_caster_deg",
                                "contact_patch_lateral_shift_mm", "wheel_center_shift_mm", "iterations",
                                "converged", "member_forces_N", "member_deflection_mm",
                                "member_joint_deflection_mm", "note"}
    assert r.summary()["iterations"] == r.iterations and r.summary()["note"] == r.note
    assert r.summary()["compliance_camber_deg"] == r.compliance_camber
    assert r.summary()["contact_patch_lateral_shift_mm"] == r.contact_patch_lateral_shift_mm
    assert r.summary()["member_forces_N"] == r.member_forces
    assert r.summary()["member_deflection_mm"] == r.member_deflection
    assert r.summary()["member_joint_deflection_mm"] == r.member_joint_deflection


def test_members_it_does_not_know_are_skipped_and_the_rest_still_flex():
    ms = cm.MemberStiffness(k_direct=2000.0)
    r = cm.CompliantCorner(HP, {"ZZ": ms, "TR": ms}).solve(lp.WheelLoad(**LOAD))
    assert "ZZ" not in r.member_deflection and r.member_deflection["TR"] != 0.0
    bare = Hardpoints.default()
    bare.pushrod_outer = None
    nb = cm.CompliantCorner(bare, {"TR": ms}).solve(lp.WheelLoad(**LOAD))
    assert "PR" not in nb.member_forces


def test_the_linearisation_is_taken_at_the_solve_it_is_given():
    load = lp.WheelLoad(**LOAD)
    j = JointCompliance.rubber_bushing()
    cc = cm.CompliantCorner(HP, {"ZZ": cm.MemberStiffness(k_direct=1.0),
                                 "TR": cm.MemberStiffness(k_direct=3000.0, joint_in=j, joint_out=j)})
    lin = cc.linearized_rates(load, freq_hz=12.0, max_iter=1)
    one = cc.solve(load, max_iter=1)
    assert set(lin) == {"TR"}
    e = lin["TR"]
    assert e["force_N"] == pytest.approx(one.member_forces["TR"], rel=1e-12)
    assert e["tangent_axial_stiffness_N_per_mm"] == pytest.approx(one.member_stiffness["TR"], rel=1e-12)
    assert set(e["joints"]) == {"in", "out"}
    assert e["joints"]["in"] == j.linearize(about_force_N=one.member_forces["TR"], freq_hz=12.0)


def test_joint_damping_is_summed_per_joint():
    j = JointCompliance.linear(3000.0, c_viscous=2.0, loss_factor=0.1)
    cc = cm.CompliantCorner(HP, {"TR": cm.MemberStiffness(k_direct=3000.0, joint_in=j, joint_out=j)})
    d = cc.damping_summary(lp.WheelLoad(Fz=1000.0))
    e = j.energy_loss_per_cycle(0.5, 15.0)
    assert d == {"amplitude_mm": 0.5, "freq_hz": 15.0, "energy_per_cycle_mJ": {"TR_in": e, "TR_out": e},
                 "total_energy_per_cycle_mJ": 2 * e}


def test_the_corner_load_comes_from_the_load_transfer():
    veh = VehicleDynamics(VehicleParams())
    loads, _ = veh.lateral_load_transfer(1.2)
    w = cm.corner_wheel_load(veh, "front", 1.2)
    assert (w.Fz, w.Fy, w.Fx, w.Fx_wc) == pytest.approx((loads.fr, -1.2 * loads.fr, 0.0, 0.0))
    assert cm.corner_wheel_load(veh, "front", 1.2, outer=False).Fz == pytest.approx(loads.fl)
    assert cm.corner_wheel_load(veh, "rear", 1.2).Fz == pytest.approx(loads.rr)
    assert cm.corner_wheel_load(veh, "rear", 1.2, outer=False).Fz == pytest.approx(loads.rl)
    t = cm.corner_wheel_load(veh, "rear", 1.2, long_g=-0.5, inboard_drive=True)
    assert (t.Fx, t.Fx_wc) == pytest.approx((0.0, -0.5 * loads.rr))
    for kw in (dict(long_g=-0.5), dict(long_g=0.0005, inboard_drive=True), dict(long_g=0.5, inboard_drive=True)):
        w = cm.corner_wheel_load(veh, "rear", 1.2, **kw)
        assert (w.Fx, w.Fx_wc) == pytest.approx((kw["long_g"] * loads.rr, 0.0))


def test_a_member_length_falls_back_on_the_hardpoint_when_the_state_has_no_point():
    import dataclasses
    cc = cm.CompliantCorner(HP, {})
    st = dataclasses.replace(cc.rigid_kin.static, pushrod_outer=None)
    want = np.linalg.norm(np.asarray(HP.pushrod_outer) - np.asarray(HP.rocker_pushrod))
    assert cc._member_length("PR", cc.rigid_kin, st) == pytest.approx(want)
    solved = cc.rigid_kin.static
    assert cc._member_length("PR", cc.rigid_kin, solved) == pytest.approx(
        np.linalg.norm(np.asarray(solved.pushrod_outer) - np.asarray(HP.rocker_pushrod)))


# ------------------------------------------------------------------ added edges
def test_a_member_length_reads_the_moved_state_point():
    import dataclasses
    cc = cm.CompliantCorner(HP, {})
    moved = np.asarray(HP.pushrod_outer, float) + np.array([10.0, -4.0, 6.0])
    st = dataclasses.replace(cc.rigid_kin.static, pushrod_outer=moved)
    assert cc._member_length("PR", cc.rigid_kin, st) == pytest.approx(
        np.linalg.norm(moved - np.asarray(HP.rocker_pushrod)))


def test_the_spec_load_defaults_to_the_full_load():
    sp = ek.ElastoSpec(Fx=100.0, Fy=-50.0, Fz=900.0, Mz=12.0, Fx_wc=30.0)
    full = sp.load()
    assert (full.Fx, full.Fy, full.Fz, full.Mz, full.Fx_wc) == (100.0, -50.0, 900.0, 12.0, 30.0)
    half = sp.load(0.5)
    assert (half.Fx, half.Fz) == (50.0, 450.0)


def test_a_semidefinite_coupled_flexibility_is_refused():
    n = 3 * len(ek.PICKUPS)
    C = np.eye(n)
    C[0, 0] = 0.0
    with pytest.raises(ValueError) as e:
        ek.StiffnessField(coupled=C)
    assert str(e.value) == "StiffnessField.coupled must be symmetric positive definite (Maxwell-Betti)."


def test_the_first_iteration_note_survives_a_one_step_solve():
    cc = cm.CompliantCorner.uniform_tube(HP)
    r = cc.solve(lp.WheelLoad(Fx=300.0, Fy=1500.0, Fz=1200.0, Fx_wc=400.0), max_iter=1)
    assert r.note.startswith("pushrod on the lower wishbone: two-body solution")


class _SoftJoint:
    def displacement(self, F):
        return F / 0.5

    def tangent_stiffness(self, d):
        return 0.5


def test_a_soft_joint_still_counts_in_the_tangent_stiffness():
    ms = cm.MemberStiffness(k_direct=1000.0, joint_in=_SoftJoint())
    assert ms.tangent_axial_stiffness(10.0, 300.0) == pytest.approx(1.0 / (1 / 1000.0 + 1 / 0.5))
    assert ms.axial_deflection(10.0, 300.0) == pytest.approx(10.0 / 1000.0 + 20.0)
