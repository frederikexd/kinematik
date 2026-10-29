"""suspension/bolted_joint.py against VDI 2230 / Shigley worked by hand: preload T / (K d), the bolt and Wileman member
stiffnesses, the load factor, separation at F_i / (1 - Phi), the bolt force before and after the gap opens, the head
bearing pressure, and every finding the board shows, with its exact wording."""
import math

import pytest

from suspension import bolted_joint as bj
from suspension.flex import MATERIALS
from suspension.interfaces import Severity


def test_the_fastener_library():
    assert bj.BOLT_GRADES["8.8"] == bj.BoltGrade("8.8", 580.0, 640.0, 800.0)
    assert bj.BOLT_GRADES["10.9"] == bj.BoltGrade("10.9", 830.0, 940.0, 1040.0)
    assert bj.BOLT_GRADES["12.9"] == bj.BoltGrade("12.9", 970.0, 1100.0, 1220.0)
    assert bj.BOLT_GRADES["A574-alloy"] == bj.BoltGrade("A574-alloy", 970.0, 1100.0, 1220.0)
    assert bj.METRIC_COARSE == {5.0: (0.80, 14.2), 6.0: (1.00, 20.1), 8.0: (1.25, 36.6), 10.0: (1.50, 58.0),
                                12.0: (1.75, 84.3)}
    f = bj.Fastener()
    assert (f.grade, f.nominal_d_mm, f.stress_area_mm2, f.K_factor, f.head_dia_mm, f.hole_dia_mm) == \
        ("10.9", 6.0, None, 0.20, None, None)
    s = bj.ClampedStack()
    assert (s.base_material, s.chassis_material, s.grip_mm, s.base_bearing_allow_MPa, s.k_bolt_N_per_mm,
            s.k_member_N_per_mm) == ("Aluminium 7075", "Aluminium 6061", 8.0, None, None, None)


def test_stress_and_bearing_areas():
    assert bj.Fastener(nominal_d_mm=8.0).stress_area() == 36.6
    assert bj.Fastener(nominal_d_mm=8, stress_area_mm2=40).stress_area() == 40.0
    with pytest.raises(ValueError) as e:
        bj.Fastener(nominal_d_mm=7.0).stress_area()
    assert str(e.value) == ("No built-in stress area for M7.0; supply stress_area_mm2 explicitly. Known: "
                            "[5.0, 6.0, 8.0, 10.0, 12.0]")
    assert bj.Fastener().bearing_area_mm2() is None
    assert bj.Fastener(head_dia_mm=10.0, hole_dia_mm=6.4).bearing_area_mm2() == pytest.approx(
        math.pi / 4 * (100 - 6.4 ** 2))
    assert bj.Fastener(head_dia_mm=10.0).bearing_area_mm2() == pytest.approx(math.pi / 4 * (100 - 6.6 ** 2))
    assert bj.Fastener(head_dia_mm=6.6).bearing_area_mm2() is None


def test_bolt_and_member_stiffness():
    assert bj._bolt_stiffness(bj.Fastener(), 8.0) == pytest.approx(205000 * 20.1 / 8)
    assert bj._bolt_stiffness(bj.Fastener(), 8.0, E_bolt_MPa=200000.0) == pytest.approx(200000 * 20.1 / 8)
    assert bj._member_stiffness(70000.0, 6.0, 8.0) == pytest.approx(70000 * 6 * 0.78715 * math.exp(0.62873 * 6 / 8))


def _hand(T=12.0, ext=3000.0, pry=1.0, d=6.0, At=20.1, K=0.2, grip=8.0, kb=None, km=None, E=None):
    Fi = T * 1000 / (K * d)
    kb = 205000 * At / grip if kb is None else kb
    E = min(MATERIALS["Aluminium 7075"].E, MATERIALS["Aluminium 6061"].E) if E is None else E
    km = E * d * 0.78715 * math.exp(0.62873 * d / grip) if km is None else km
    phi = kb / (kb + km)
    Fe = ext * pry
    Fsep = Fi / (1 - phi)
    Fb = Fi + phi * Fe if Fe <= Fsep else Fe
    return Fi, phi, Fe, Fsep, Fb


def test_a_closed_joint():
    r = bj.analyze_joint(bj.Fastener(head_dia_mm=10.0, hole_dia_mm=6.4), bj.ClampedStack(), 12.0, 3000.0)
    Fi, phi, Fe, Fsep, Fb = _hand()
    assert (r.F_preload, r.load_factor, r.F_bolt_share, r.F_sep, r.F_bolt_max) == pytest.approx((Fi, phi, Fe, Fsep, Fb))
    assert r.sigma_bolt == pytest.approx(Fb / 20.1) and r.sigma_proof == 830.0
    assert r.separation_safety == pytest.approx(Fsep / Fe) and r.separated is False
    assert r.sigma_bearing == pytest.approx(Fb / (math.pi / 4 * (100 - 6.4 ** 2)))
    assert r.bearing_allow == 503.0 and r.bearing_yield == (r.sigma_bearing > 503.0)
    assert (r.prying_factor, r.is_estimate) == (1.0, False) and r.notes == ""
    assert r.as_dict()["F_preload"] == r.F_preload and set(r.as_dict()) >= {"notes", "is_estimate"}


def test_a_joint_that_opens_carries_the_full_load():
    r = bj.analyze_joint(bj.Fastener(), bj.ClampedStack(), 5.0, 6000.0, prying_factor=1.5)
    Fi, phi, Fe, Fsep, Fb = _hand(T=5.0, ext=6000.0, pry=1.5)
    assert r.separated is True and r.F_bolt_max == pytest.approx(Fe) == pytest.approx(9000.0)
    assert r.is_estimate is True and r.prying_factor == 1.5
    assert r.notes == ("JOINT SEPARATED: external load exceeded the gap-opening load; the bolt now carries the full "
                       "raw tensile load and sees a fatigue/peak spike.")
    assert r.sigma_bearing is None and r.bearing_allow is None and r.bearing_yield is None
    edge = bj.analyze_joint(bj.Fastener(), bj.ClampedStack(), 5.0, Fsep / 1.5, prying_factor=1.5)
    assert edge.separated is False                                       # exactly at the gap-opening load: closed


def test_overrides_grades_torque_and_an_over_tensioned_bolt():
    s = bj.ClampedStack(k_bolt_N_per_mm=3e5, k_member_N_per_mm=9e5, base_bearing_allow_MPa=350.0,
                        base_material="Steel 4130")
    f = bj.Fastener(grade="8.8", nominal_d_mm=8.0, K_factor=0.15, head_dia_mm=13.0)
    r = bj.analyze_joint(f, s, 30.0, 2000.0)
    Fi, phi, Fe, Fsep, Fb = _hand(T=30.0, ext=2000.0, d=8.0, At=36.6, K=0.15, kb=3e5, km=9e5)
    assert (r.F_preload, r.load_factor, r.F_bolt_max) == pytest.approx((Fi, 0.25, Fb))
    assert r.bearing_allow == 350.0 and r.sigma_proof == 580.0
    cap = 0.75 * 580.0
    assert Fi / 36.6 > cap and r.notes == (
        f"Preload stress {Fi / 36.6:.0f} MPa exceeds 75% of proof ({cap:.0f} MPa) — torque spec over-tensions "
        "the bolt at install.")
    assert bj.analyze_joint(f, s, 30.0, 2000.0, preload_fraction_of_proof_cap=1.2).notes == ""
    for mat, allow in (("Aluminium 6061", 276.0), ("Steel mild", 250.0), ("Titanium", 200.0)):
        st = bj.ClampedStack(base_material=mat, k_member_N_per_mm=9e5)
        assert bj.analyze_joint(bj.Fastener(head_dia_mm=10.0), st, 10.0, 100.0).bearing_allow == allow
    rigid = bj.analyze_joint(bj.Fastener(), bj.ClampedStack(k_member_N_per_mm=0.0), 10.0, 100.0)
    assert rigid.F_sep == math.inf and rigid.load_factor == 1.0
    assert bj.analyze_joint(bj.Fastener(), bj.ClampedStack(), 10.0, 0.0).separation_safety == math.inf
    gs = bj.analyze_joint(bj.Fastener(), bj.ClampedStack(grip_mm=12.0, chassis_material="Steel 4130"), 10.0, 100.0)
    _, phi12, *_ = _hand(T=10.0, ext=100.0, grip=12.0, E=min(MATERIALS["Aluminium 7075"].E, MATERIALS["Steel 4130"].E))
    assert gs.load_factor == pytest.approx(phi12)


@pytest.mark.parametrize("kw,msg", [
    (dict(assembly_torque_Nm=0.0), "assembly_torque_Nm must be > 0"),
    (dict(prying_factor=0.0), "prying_factor must be > 0"),
])
def test_refusals(kw, msg):
    args = dict(fastener=bj.Fastener(), stack=bj.ClampedStack(), assembly_torque_Nm=10.0, external_tensile_N=100.0)
    args.update(kw)
    with pytest.raises(ValueError) as e:
        bj.analyze_joint(**args)
    assert str(e.value) == msg
    with pytest.raises(ValueError) as e:
        bj.analyze_joint(bj.Fastener(grade="4.6"), bj.ClampedStack(), 10.0, 100.0)
    assert str(e.value) == "Unknown bolt grade '4.6'; known: ['10.9', '12.9', '8.8', 'A574-alloy']"


def _r(**kw):
    base = dict(F_preload=10000.0, load_factor=0.2, F_bolt_share=3000.0, F_bolt_max=10600.0, sigma_bolt=500.0,
                sigma_proof=830.0, F_sep=12500.0, separation_safety=4.1667, separated=False, sigma_bearing=None,
                bearing_allow=None, bearing_yield=None, prying_factor=1.0, is_estimate=False)
    base.update(kw)
    return bj.JointResult(**base)


def test_the_findings_and_their_wording():
    f = bj.joint_findings(_r())
    assert [(x.check, x.severity) for x in f] == [("bolt-separation", Severity.OK)]
    assert f[0].message == "pedal-box mount bolt: joint stays clamped (4.17× margin to separation)."
    assert f[0].subsystems == ["structures", "chassis"] and f[0].detail == _r().as_dict()
    w = bj.joint_findings(_r(separation_safety=1.2, sigma_bolt=720.0), bolt_label="M6", owners=("brakes",))
    assert [(x.check, x.severity) for x in w] == [("bolt-separation", Severity.WARN), ("bolt-stress", Severity.WARN)]
    assert w[0].message == ("M6: thin margin against joint separation (safety 1.20× — under the usual 1.25 target). "
                            "Raise preload/torque or add clamp area before the base starts to peel.")
    assert w[1].message == "M6: bolt at 87% of proof — little headroom." and w[1].subsystems == ["brakes"]
    assert [x.severity for x in bj.joint_findings(_r(separation_safety=1.25))] == [Severity.OK]
    assert [x.check for x in bj.joint_findings(_r(sigma_bolt=830.0 * 0.85))] == ["bolt-separation"]
    s = bj.joint_findings(_r(separated=True, sigma_bolt=900.0, sigma_bearing=600.0, bearing_allow=503.0,
                             bearing_yield=True, prying_factor=1.4, is_estimate=True))
    assert [(x.check, x.severity) for x in s] == [("bolt-separation", Severity.FAIL), ("bolt-stress", Severity.FAIL),
                                                 ("bolt-bearing", Severity.FAIL), ("bolt-prying", Severity.INFO)]
    assert s[0].message == ("pedal-box mount bolt: the clamped joint OPENS under the braking load (external 3000 N > "
                            "gap-opening 12500 N). The base lifts off its tab and the bolt takes the full raw load.")
    assert s[1].message == "pedal-box mount bolt: bolt tensile stress 900 MPa exceeds proof 830 MPa — the bolt yields."
    assert s[2].message == ("pedal-box mount bolt: bearing pressure under the head/washer 600 MPa exceeds the base "
                            "allowable 503 MPa — the base crushes locally, preload relaxes and the joint can back "
                            "out on track. Add a washer or larger head.")
    assert s[3].message == ("pedal-box mount bolt: result uses an ESTIMATED prying factor 1.40× (the bending-moment "
                            "lever amplifying this bolt's tensile share). This is the part a real contact FEA or a hand "
                            "lever-arm calc owns — treat as provisional until that factor is checked.")
    assert s[3].detail == {"prying_factor": 1.4} and all(x.subsystems == ["structures", "chassis"] for x in s)
    assert [x.check for x in bj.joint_findings(_r(sigma_bolt=830.0))] == ["bolt-separation", "bolt-stress"]


def test_every_finding_carries_its_owners_and_the_result():
    for r, own in ((_r(), ("a",)), (_r(separation_safety=1.2, sigma_bolt=720.0), ("b", "c")),
                   (_r(separated=True, sigma_bolt=900.0, sigma_bearing=600.0, bearing_allow=503.0, bearing_yield=True,
                       prying_factor=1.4, is_estimate=True), ("d",))):
        for f in bj.joint_findings(r, owners=own):
            assert f.subsystems == list(own)
            assert f.detail == ({"prying_factor": r.prying_factor} if f.check == "bolt-prying" else r.as_dict())


def test_the_named_base_bearing_allowances_and_small_inputs():
    f = bj.Fastener(head_dia_mm=10.0, hole_dia_mm=6.4)
    for mat, allow in (("Steel 4130", 460.0), ("Steel mild", 250.0), ("Aluminium 6061", 276.0), ("Unobtanium", 200.0)):
        st = bj.ClampedStack(base_material=mat, k_member_N_per_mm=None if mat in MATERIALS else 4e5)
        assert bj.analyze_joint(f, st, 12.0, 3000.0).bearing_allow == allow
    assert bj.analyze_joint(f, bj.ClampedStack(), 0.5, 3000.0).F_preload > 0
    light = bj.analyze_joint(f, bj.ClampedStack(), 12.0, 0.5)
    assert light.separation_safety == pytest.approx(light.F_sep / light.F_bolt_share) and light.separation_safety < math.inf
    assert bj.Fastener(head_dia_mm=6.6, hole_dia_mm=6.6).bearing_area_mm2() is None
    both = bj.analyze_joint(bj.Fastener(), bj.ClampedStack(), 30.0, 60000.0)
    assert both.separated and both.notes.count("  ") == 1 and both.notes.startswith("Preload stress")
