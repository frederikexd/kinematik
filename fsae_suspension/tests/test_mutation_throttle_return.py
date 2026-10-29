"""suspension/throttle_return.py against oracles written here: spring rates from bench points and Shigley's
formula, closing torques from rate x stretch x arm, the healthy and every single-failure case summed by hand, the
verdict bands at their exact edges, the brake-pedal screen against bracket_fos itself, the plate inertia, the cam and
aero terms, and the return-snap integration re-run by an independent RK4 loop. Messages are checked word for word."""
import math

import pytest

from suspension import throttle_return as tr
from suspension.bracket_fos import Bracket, screen_bracket
from suspension.interfaces import Severity

SUB = ["brakes", "powertrain"]


def test_spring_rates():
    assert tr.WIRE_SHEAR_MODULUS_PA == {"music wire (ASTM A228)": 81.7e9, "stainless 302/304": 69.0e9,
                                        "chrome silicon (ASTM A401)": 77.2e9, "oil-tempered (ASTM A229)": 79.3e9,
                                        "phosphor bronze": 41.4e9}
    assert tr.k_from_deflection(50.0, 0.02) == 2500.0
    assert tr.k_from_two_points(10.0, 0.01, 30.0, 0.03) == pytest.approx(1000.0)
    assert tr.k_from_two_points(30.0, 0.03, 10.0, 0.01) == pytest.approx(1000.0)
    assert tr.k_theta_from_torque(2.0, 0.5) == 4.0
    for f, a, msg in ((tr.k_from_deflection, (1.0, 0.0), "deflection must be > 0 to divide by it"),
                      (tr.k_from_deflection, (1.0, -1.0), "deflection must be > 0 to divide by it"),
                      (tr.k_theta_from_torque, (1.0, 0.0), "angle must be > 0 to divide by it"),
                      (tr.k_theta_from_torque, (1.0, -0.1), "angle must be > 0 to divide by it"),
                      (tr.k_from_two_points, (1.0, 0.2, 2.0, 0.2), "the two positions must differ")):
        with pytest.raises(ValueError) as e:
            f(*a)
        assert str(e.value) == msg
    assert tr.k_compression_spring(0.0015, 0.012, 8) == pytest.approx(81.7e9 * 0.0015 ** 4 / (8 * 0.012 ** 3 * 8))
    assert tr.k_compression_spring(0.002, 0.015, 6.5, material="phosphor bronze") == pytest.approx(
        41.4e9 * 0.002 ** 4 / (8 * 0.015 ** 3 * 6.5))
    assert tr.k_compression_spring(0.002, 0.015, 6.5, material="nope", shear_modulus_Pa=1e9) == pytest.approx(
        1e9 * 0.002 ** 4 / (8 * 0.015 ** 3 * 6.5))
    for bad in ((0, 0.01, 5), (0.001, 0, 5), (0.001, 0.01, 0), (-1, 0.01, 5)):
        with pytest.raises(ValueError) as e:
            tr.k_compression_spring(*bad)
        assert str(e.value) == "wire dia, coil dia and active coils must all be > 0"
    with pytest.raises(ValueError) as e:
        tr.k_compression_spring(0.001, 0.01, 5, material="unobtainium")
    assert str(e.value) == ("unknown wire material 'unobtainium' — pass shear_modulus_Pa or pick one of: "
                            + ", ".join(tr.WIRE_SHEAR_MODULUS_PA))


def test_springs_and_resistance():
    s = tr.ReturnSpring.from_linear_spring("p", 2000.0, 0.03, 0.01, 0.02, is_estimate=True)
    assert (s.name, s.torque_closed_Nm, s.torque_open_Nm, s.is_estimate) == ("p", pytest.approx(0.6),
                                                                             pytest.approx(1.8), True)
    assert tr.ReturnSpring.from_linear_spring("q", 1.0, 1.0, 1.0, 1.0).is_estimate is False
    t = tr.ReturnSpring.from_torsion_spring("t", 0.5, 0.4, 1.2)
    assert (t.torque_closed_Nm, t.torque_open_Nm, t.is_estimate) == (pytest.approx(0.2), pytest.approx(0.8), False)
    assert tr.ReturnSpring.from_torsion_spring("t", 1.0, 1.0, 1.0, is_estimate=True).is_estimate is True
    assert s.as_dict() == {"name": "p", "torque_closed_Nm": s.torque_closed_Nm, "torque_open_Nm": s.torque_open_Nm,
                           "is_estimate": True}
    assert tr.ReturnSpring("a", 1, 2).is_estimate is False
    r = tr.ReturnResistance(-0.1, 0.2, -0.3)
    assert r.total_Nm == pytest.approx(0.6) and r.as_dict() == {"friction_Nm": -0.1, "cable_drag_Nm": 0.2,
                                                                 "sensor_detent_Nm": -0.3}
    assert tr.ReturnResistance().total_Nm == 0.0


def test_a_net_case():
    c = tr._net_case("x", [tr.ReturnSpring("a", 1.0, 3.0), tr.ReturnSpring("b", 0.5, 0.2)], 0.5)
    assert (c.label, c.net_closed_Nm, c.net_open_Nm, c.margin, c.closes) == ("x", 1.0, 2.7, 2.0, True)
    assert c.as_dict() == {"label": "x", "net_closed_Nm": 1.0, "net_open_Nm": 2.7, "margin": 2.0, "closes": True}
    z = tr._net_case("z", [tr.ReturnSpring("a", 1.0, 1.0)], 0.0)
    assert z.margin == math.inf and z.closes is True
    edge = tr._net_case("e", [tr.ReturnSpring("a", 0.5, 2.0)], 0.5)
    assert edge.net_closed_Nm == 0.0 and edge.closes is False and edge.margin == 0.0


# ------------------------------------------------------------------ redundancy
def S(name, c, o, est=False):
    return tr.ReturnSpring(name, c, o, is_estimate=est)


def test_too_few_springs():
    r = tr.check_return_redundancy([S("only", 5.0, 9.0, est=True)], tr.ReturnResistance(0.2))
    assert (r.verdict, r.n_springs, r.margin_target, r.worst_case, r.worst_margin, r.is_estimate, r.cases, r.notes) == (
        "FAIL", 1, 1.0, "(insufficient springs)", 0.0, True, [], ["need >= 2 return springs"])
    (f,) = r.findings
    assert (f.check, f.severity, f.subsystems) == ("throttle-return-redundancy", Severity.FAIL, SUB)
    assert f.message == ("Throttle has 1 return spring(s). FSAE requires AT LEAST TWO, arranged so the throttle still "
                         "returns to closed if any one fails. Sensors (TPS/APPS) do not count. Add a second, "
                         "independent return spring.")
    assert f.detail == dict(n_springs=1, rule="two independent return springs")
    assert tr.check_return_redundancy([], tr.ReturnResistance(0.2)).n_springs == 0


def test_a_clean_pass():
    springs = [S("a", 1.0, 2.0), S("b", 1.2, 2.5)]
    r = tr.check_return_redundancy(springs, tr.ReturnResistance(0.2, 0.1, 0.0))
    R = 0.3
    assert [c.label for c in r.cases] == ["all springs healthy", "without 'a'", "without 'b'"]
    assert r.cases[1].margin == pytest.approx((1.2 - R) / R) and r.cases[2].margin == pytest.approx((1.0 - R) / R)
    assert (r.verdict, r.worst_case, r.worst_margin, r.is_estimate, r.notes) == (
        "PASS", "without 'b'", pytest.approx((1.0 - R) / R), False, [])
    (f,) = r.findings
    assert (f.severity, f.subsystems) == (Severity.OK, SUB)
    assert f.message == (f"Throttle is single-fault tolerant: with any one of 2 springs removed it still makes net "
                         f"closing torque across full travel, worst-case margin {(1.0 - R) / R:.2f} over 0.30 N·m "
                         "resistance (without 'b'). Meets the two-independent-return-spring rule.")
    assert f.detail == dict(worst_case="without 'b'", worst_margin=pytest.approx((1.0 - R) / R), margin_target=1.0,
                            resistance_Nm=pytest.approx(R), estimate=False)
    d = r.as_dict()
    assert d["verdict"] == "PASS" and d["cases"][0] == r.cases[0].as_dict() and d["findings"] == [f.as_dict()]
    assert set(d) == {"verdict", "n_springs", "margin_target", "worst_case", "worst_margin", "is_estimate", "cases",
                      "findings", "notes"}
    assert d["notes"] == [] and d["notes"] is not r.notes


def test_the_verdict_edges():
    R = tr.ReturnResistance(0.5)
    # worst single failure: 1.0 - 0.5 = 0.5 net => margin 1.0 exactly: on the floor, below 1.25 => TIGHT
    tight = tr.check_return_redundancy([S("a", 1.0, 3.0), S("b", 1.0, 3.0)], R)
    assert tight.verdict == "TIGHT" and tight.worst_margin == 1.0
    (f,) = tight.findings
    assert f.severity == Severity.WARN and f.message == (
        "Throttle is single-fault tolerant but TIGHT: worst single-failure case (without 'a') closes with margin 1.00 "
        "over resistance, just over the 1.00 floor. It returns, but there's little buffer for in-car stiction the "
        "bench test won't show — worth more preload or less drag.")
    assert f.detail == dict(worst_case="without 'a'", worst_margin=1.0, margin_target=1.0, resistance_Nm=0.5,
                            estimate=False)
    ceiling = tr.check_return_redundancy([S("a", 1.125, 3.0), S("b", 1.125, 3.0)], R)
    assert ceiling.worst_margin == pytest.approx(1.25) and ceiling.verdict == "PASS"
    under = tr.check_return_redundancy([S("a", 0.9, 3.0, est=True), S("b", 0.95, 3.0)], R)
    assert under.verdict == "FAIL" and under.worst_margin == pytest.approx(0.8)
    (f,) = under.findings
    est = " (using an ESTIMATED spring rate — measure or datasheet k to confirm)"
    assert f.message == ("Throttle returns with one spring failed but with too little authority: worst single-failure "
                         "margin 0.80 over resistance is below the 1.00 target (without 'b')" + est + ". A sticky pivot "
                         "or a cold, draggy cable could hang it. Add preload/rate or cut resistance.")
    assert f.detail == dict(worst_case="without 'b'", worst_margin=pytest.approx(0.8), margin_target=1.0,
                            resistance_Nm=0.5, estimate=True)
    assert under.notes == ["At least one spring rate is an estimate. Back out k on the bench with k_from_deflection() "
                           "(hang a known load, measure travel) or from geometry with k_compression_spring(), then "
                           "re-run."]
    custom = tr.check_return_redundancy([S("a", 2.0, 3.0), S("b", 2.0, 3.0)], R, margin_target=2.0, tight_band=0.6)
    assert custom.verdict == "TIGHT" and custom.margin_target == 2.0 and custom.worst_margin == pytest.approx(3.0)
    assert tr.check_return_redundancy([S("a", 2.0, 3.0), S("b", 2.0, 3.0)], R, margin_target=2.0,
                                      tight_band=0.5).verdict == "PASS"


def test_a_spring_that_cannot_close_alone():
    R = tr.ReturnResistance(0.3, 0.2, 0.1)
    r = tr.check_return_redundancy([S("weak", 0.4, 0.9), S("strong", 2.0, 3.0), S("mid", 0.1, 0.5)], R)
    assert r.verdict == "FAIL" and len(r.cases) == 4
    (f,) = r.findings
    dead = r.cases[2]                                     # without 'strong': -0.1 closed, 0.8 open
    assert f.message == (f"Throttle does NOT return to closed with one spring failed: without 'strong' gives net closing "
                         f"torque {min(dead.net_closed_Nm, dead.net_open_Nm):.2f} N·m (<= 0 means it hangs open) against "
                         "0.60 N·m of resistance. This is the rule the pedal-box must pass. Increase the other spring's "
                         "preload/rate, or reduce friction/cable drag, until every single-failure case closes with "
                         "margin.")
    assert f.detail == dict(worst_case="without 'strong'", net_closed=dead.net_closed_Nm, net_open=dead.net_open_Nm,
                            resistance_Nm=pytest.approx(0.6), estimate=False)
    two = tr.check_return_redundancy([S("a", 0.5, 2.0), S("b", 3.0, 0.2)], tr.ReturnResistance(1.0))
    assert two.verdict == "FAIL" and two.findings[0].detail["worst_case"] == "without 'a'"     # -0.8 at full open


def test_zero_resistance_is_demoted():
    r = tr.check_return_redundancy([S("a", 1.0, 2.0), S("b", 1.0, 2.0)])
    assert r.verdict == "TIGHT" and r.worst_margin == math.inf
    assert r.notes == ["Resistance was zero — this is an OPTIMISTIC check. Put real friction/stiction and cable-drag "
                       "numbers in ReturnResistance; a return that only just closes against zero drag will hang in the "
                       "car."]
    ok, cov = r.findings
    assert ok.severity == Severity.OK and (cov.check, cov.severity, cov.subsystems) == (
        "throttle-return-resistance-coverage", Severity.WARN, SUB)
    assert cov.message == ("Return checked against ZERO resistance — friction, cable/rod drag and sensor detent were "
                           "all left at 0. A return that closes only against zero drag is the throttle version of a "
                           "pedal screened in bending alone; it will hang on real in-car stiction. Result demoted from "
                           "PASS to TIGHT because a zero-resistance screen must not read as a validated return. Measure "
                           "the pivot/cable drag (worst-case, cold) and re-run before you trust this.")
    assert cov.detail == dict(resistance_Nm=0.0, demoted_from_pass=True)
    explicit = tr.check_return_redundancy([S("a", 1.0, 2.0), S("b", 1.0, 2.0)], tr.ReturnResistance())
    assert explicit.verdict == "TIGHT"
    hang = tr.check_return_redundancy([S("a", 0.0, 2.0), S("b", 1.0, 2.0)], tr.ReturnResistance())
    assert hang.verdict == "FAIL" and hang.findings[1].detail["demoted_from_pass"] is False
    assert "Result demoted" not in hang.findings[1].message


def test_the_report():
    r = tr.check_return_redundancy([S("a", 1.0, 2.0, est=True), S("b", -0.5, 0.4)], tr.ReturnResistance(0.2))
    txt = tr.return_redundancy_report(r)
    lines = txt.split("\n")
    assert lines[0] == "THROTTLE RETURN-SPRING REDUNDANCY — FAIL"
    assert lines[1] == "  springs: 2   margin target: 1.00   [ESTIMATED k]"
    assert lines[2] == f"  governing single-failure case: {r.worst_case} (margin {r.worst_margin:.2f})"
    assert lines[3] == "  cases (net CLOSING torque, N·m; negative = hangs open):"
    c = r.cases[1]
    assert lines[5] == (f"    - {c.label:<28} closed={c.net_closed_Nm:7.2f}  open={c.net_open_Nm:7.2f}  "
                        "[HANGS OPEN]")
    assert lines[4].endswith("[closes]")
    assert lines[7] == f"  [FAIL] {r.findings[0].message}"
    assert lines[-1] == f"  note: {r.notes[-1]}"
    ok = tr.return_redundancy_report(tr.check_return_redundancy([S("a", 2.0, 3.0), S("b", 2.0, 3.0)],
                                                                 tr.ReturnResistance(0.2)))
    assert "[measured/datasheet k]" in ok and "  [OK] Throttle is single-fault tolerant" in ok


# ------------------------------------------------------------------ the brake pedal
def _bracket(**kw):
    base = dict(name="brake pedal", material="Steel 1018 CR (cold-rolled)", width_mm=30.0, thickness_mm=8.0,
                P_N=2000.0, lever_arm_mm=150.0, hole_dia_mm=0.0, edge_dist_mm=0.0, weld_leg_mm=0.0,
                weld_length_mm=0.0, load_is_shear=False, n_bolts=1, is_estimate=True)
    base.update(kw)
    return screen_bracket(Bracket(**base), fos_target=kw.pop("fos", 1.5))


def test_the_brake_pedal_screen_passes_through():
    assert tr.BRAKE_PEDAL_RULE_LOAD_N == 2000.0
    full = tr.check_brake_pedal_2000N(30.0, 12.0, 120.0, material="Aluminium 7075-T6", pivot_bolt_dia_mm=10.0,
                                      edge_dist_mm=15.0, n_bolts=2, weld_leg_mm=4.0, weld_length_mm=60.0,
                                      load_N=-2500.0, fos_target=2.0, is_estimate=False)
    want = screen_bracket(Bracket(name="brake pedal", material="Aluminium 7075-T6", width_mm=30.0, thickness_mm=12.0,
                                  P_N=2500.0, lever_arm_mm=120.0, hole_dia_mm=10.0, edge_dist_mm=15.0, weld_leg_mm=4.0,
                                  weld_length_mm=60.0, load_is_shear=False, n_bolts=2, is_estimate=False),
                          fos_target=2.0)
    assert full.verdict == want.verdict and [m.mode for m in full.modes] == [m.mode for m in want.modes]
    assert [m.fos for m in full.modes] == pytest.approx([m.fos for m in want.modes])
    assert full.notes == want.notes and len(full.findings) == len(want.findings)


def test_an_incomplete_pedal_screen_is_demoted():
    r = tr.check_brake_pedal_2000N(40.0, 15.0, 100.0)
    base = _bracket(width_mm=40.0, thickness_mm=15.0, lever_arm_mm=100.0)
    assert base.verdict == "PASS" and r.verdict == "TIGHT"
    screened = sorted({m.mode for m in base.modes})
    missing = ["pivot-bolt bearing", "pivot-lug tear-out", "weld-throat shear (if fabricated/welded)"]
    assert r.notes[-1] == ("COVERAGE: screened " + ", ".join(screened) + ". NOT screened: " + "; ".join(missing)
                           + ". This is a bending-governed screen only — supply the pivot-bolt and weld geometry to "
                             "screen those modes before you trust a pass.")
    f = r.findings[-1]
    assert (f.check, f.severity, f.subsystems) == ("brake-pedal-2000N-coverage", Severity.WARN, ["brakes"])
    assert f.message == (f"Brake pedal screened at 2000 N on {len(screened)} mode(s) only; NOT checked: "
                         + ", ".join(missing) + ". Result demoted from PASS to TIGHT because an incomplete screen must "
                         "not read as a clean pedal. Add the pivot lug and weld geometry, or take this straight to "
                         "SolidWorks/ANSYS for those modes — don't let a bending-only pass be the last word.")
    assert f.detail == dict(screened=screened, not_screened=missing, load_N=2000.0, demoted_from_pass=True)
    bolt = tr.check_brake_pedal_2000N(40.0, 15.0, 100.0, pivot_bolt_dia_mm=8.0)
    assert bolt.findings[-1].detail["not_screened"] == ["pivot-lug tear-out", "weld-throat shear (if fabricated/welded)"]
    edge_only = tr.check_brake_pedal_2000N(40.0, 15.0, 100.0, edge_dist_mm=10.0, weld_leg_mm=3.0)
    assert edge_only.findings[-1].detail["not_screened"] == missing
    weld = tr.check_brake_pedal_2000N(40.0, 15.0, 100.0, pivot_bolt_dia_mm=8.0, edge_dist_mm=12.0, weld_leg_mm=3.0,
                                      weld_length_mm=40.0)
    assert not any(f.check == "brake-pedal-2000N-coverage" for f in weld.findings)
    half_weld = tr.check_brake_pedal_2000N(40.0, 15.0, 100.0, pivot_bolt_dia_mm=8.0, edge_dist_mm=12.0,
                                           weld_length_mm=40.0)
    assert half_weld.findings[-1].detail["not_screened"] == ["weld-throat shear (if fabricated/welded)"]


def test_a_tight_or_failed_pedal_screen():
    base = _bracket(width_mm=25.0, thickness_mm=6.0, lever_arm_mm=150.0)
    tight = tr.check_brake_pedal_2000N(25.0, 6.0, 150.0)
    if base.verdict == "TIGHT":
        f = tight.findings[-1]
        assert tight.verdict == "TIGHT" and f.detail["demoted_from_pass"] is False
        assert "Result demoted" not in f.message
    fail = tr.check_brake_pedal_2000N(10.0, 3.0, 200.0)
    assert fail.verdict == "FAIL" and not any(f.check == "brake-pedal-2000N-coverage" for f in fail.findings)
    assert _bracket(width_mm=30.0, thickness_mm=16.0, lever_arm_mm=150.0).verdict == "TIGHT"
    r = tr.check_brake_pedal_2000N(30.0, 16.0, 150.0)
    assert r.verdict == "TIGHT" and r.findings[-1].detail["demoted_from_pass"] is False
    assert "Result demoted" not in r.findings[-1].message


# ------------------------------------------------------------------ inertia, cam, aero
def test_inertia():
    t = tr.ThrottleInertia()
    assert (t.I_kgm2, t.is_estimate, t.source) == (5.0e-4, True, "representative")
    assert t.as_dict() == {"I_kgm2": 5.0e-4, "is_estimate": True, "source": "representative"}
    e = tr.estimate_throttle_inertia()
    assert e.I_kgm2 == pytest.approx(0.02 * 0.02 ** 2 / 3) and e.is_estimate and e.source == "geometry estimate"
    f = tr.estimate_throttle_inertia(0.05, 0.03, 0.2, 0.1, 1e-5)
    assert f.I_kgm2 == pytest.approx(0.05 * 0.03 ** 2 / 3 + 0.2 * 0.01 + 1e-5)
    g = tr.estimate_throttle_inertia(-1.0, 0.03, -0.2, 0.1, -1.0)
    assert g.I_kgm2 == 5.0e-4
    h = tr.estimate_throttle_inertia(0.05, -0.03, 0.0, 0.0, 0.0)
    assert h.I_kgm2 == 5.0e-4
    neg_arm = tr.estimate_throttle_inertia(0.0, 0.0, 0.3, -0.2)
    assert neg_arm.I_kgm2 == pytest.approx(0.3 * 0.04)


def test_the_spring_curve_and_the_cam():
    assert tr._spring_closing_torque_at(0.5, 1.0, 1.0, 3.0) == pytest.approx(2.0)
    assert tr._spring_closing_torque_at(-1.0, 1.0, 1.0, 3.0) == 1.0
    assert tr._spring_closing_torque_at(2.0, 1.0, 1.0, 3.0) == 3.0
    assert tr._spring_closing_torque_at(0.5, 0.0, 1.0, 3.0) == 1.0
    assert tr._spring_closing_torque_at(0.5, -1.0, 1.0, 3.0) == 1.0
    assert tr._cam_multiplier(10.0, None) == 1.0 and tr._cam_multiplier(10.0, []) == 1.0
    prof = [(60.0, 2.0), (0.0, 1.0), (30.0, 0.5)]
    assert tr._cam_multiplier(-5.0, prof) == 1.0 and tr._cam_multiplier(0.0, prof) == 1.0
    assert tr._cam_multiplier(15.0, prof) == pytest.approx(0.75) and tr._cam_multiplier(30.0, prof) == 0.5
    assert tr._cam_multiplier(45.0, prof) == pytest.approx(1.25) and tr._cam_multiplier(70.0, prof) == 2.0
    assert tr._cam_multiplier(60.0, prof) == 2.0
    assert tr._cam_multiplier(20.0, [(0, 1), (20, 3), (20, 5), (40, 7)]) in (3.0, 5.0)
    assert tr._cam_multiplier(20.0, [(0, 1), (20, 3), (20, 5), (40, 7)]) == 3.0


def test_the_snap_model_and_aero():
    m = tr.SnapModel()
    assert (m.backlash_deg, m.backlash_spring_frac, m.cam_profile, m.aero_torque_coeff, m.aero_opens_plate,
            m.intake_speed_ms, m.plate_area_m2, m.plate_radius_m, m.rho) == (0.0, 0.0, None, 0.0, True, 0.0, 0.0, 0.0,
                                                                             1.225)
    assert tr.AIR_DENSITY_KGM3 == 1.225
    assert m.is_active() is False and m.aero_is_unquantified() is False
    assert tr.SnapModel(backlash_deg=1.0).is_active() is True
    assert tr.SnapModel(cam_profile=[]).is_active() is True
    assert tr.SnapModel(aero_torque_coeff=0.1).is_active() is False
    assert tr.SnapModel(intake_speed_ms=10.0).is_active() is False
    assert tr.SnapModel(aero_torque_coeff=-0.1, intake_speed_ms=10.0).is_active() is True
    assert tr.SnapModel(intake_speed_ms=10.0).aero_is_unquantified() is True
    assert tr.SnapModel(intake_speed_ms=10.0, aero_torque_coeff=0.2).aero_is_unquantified() is False
    assert m.as_dict()["rho"] == 1.225
    a = tr.SnapModel(aero_torque_coeff=0.3, intake_speed_ms=40.0, plate_area_m2=1e-3, plate_radius_m=0.02, rho=1.1)
    q = 0.5 * 1.1 * 1600
    for th in (0.0, 0.3, 0.5, 1.0, 1.4):
        frac = min(max(th / 1.2, 0.0), 1.0)
        assert tr._aero_torque(a, th, 1.2) == pytest.approx(0.3 * q * 1e-3 * 0.02 * math.sin(frac * math.pi),
                                                            abs=1e-15)
    closing = tr.SnapModel(aero_torque_coeff=0.3, intake_speed_ms=40.0, plate_area_m2=1e-3, plate_radius_m=0.02,
                           aero_opens_plate=False)
    assert tr._aero_torque(closing, 0.6, 1.2) == pytest.approx(-0.3 * 0.5 * 1.225 * 1600 * 1e-3 * 0.02)
    assert tr._aero_torque(None, 0.6, 1.2) == 0.0
    assert tr._aero_torque(tr.SnapModel(intake_speed_ms=40.0, plate_area_m2=1, plate_radius_m=1), 0.6, 1.2) == 0.0
    assert tr._aero_torque(tr.SnapModel(aero_torque_coeff=1, plate_area_m2=1, plate_radius_m=1), 0.6, 1.2) == 0.0
    assert tr._aero_torque(tr.SnapModel(aero_torque_coeff=1, intake_speed_ms=-5, plate_area_m2=1, plate_radius_m=1),
                           0.6, 1.2) == 0.0
    for A, r in ((0.0, 0.02), (1e-3, 0.0), (-1.0, 0.02), (1e-3, -1.0)):
        assert tr._aero_torque(tr.SnapModel(aero_torque_coeff=1, intake_speed_ms=10, plate_area_m2=A,
                                            plate_radius_m=r), 0.6, 1.2) == 0.0
    assert tr._aero_torque(tr.SnapModel(aero_torque_coeff=1, intake_speed_ms=10, plate_area_m2=1, plate_radius_m=1),
                           0.6, 0.0) == 0.0
    f = tr._aero_unquantified_finding(tr.SnapModel(intake_speed_ms=33.4))
    assert (f.check, f.severity, f.subsystems) == ("throttle-snap-aero", Severity.WARN, SUB)
    assert f.message == ("Aero load on the plate was requested at 33 m/s but no torque coefficient was supplied, so it "
                         "was modelled as ZERO — the return-time here ignores intake aero. The aero torque on a throttle "
                         "plate needs a flow-bench or CFD coefficient; KinematiK won't invent one. Measure it (or get it "
                         "from CFD) and set aero_torque_coeff to include the effect.")
    assert f.detail == dict(intake_speed_ms=33.4, aero_torque_coeff=0.0)


# ------------------------------------------------------------------ the snap, integrated here
def _snap(springs, I=5e-4, fric=0.0, drag=0.0, open_deg=90.0, dt=5e-4, t_max=2.0, w_stall=1e-3, model=None):
    th_o = math.radians(max(open_deg, 1e-3))
    Tc = sum(s.torque_closed_Nm for s in springs)
    To = sum(s.torque_open_Nm for s in springs)
    bl = math.radians(max(model.backlash_deg, 0.0)) if model else 0.0

    def ts(th):
        frac = min(max(th / th_o, 0.0), 1.0)
        v = Tc + (To - Tc) * frac
        if model is not None and model.cam_profile:
            v *= tr._cam_multiplier(math.degrees(th), model.cam_profile)
        if bl > 0 and th > th_o - bl:
            v *= min(max(model.backlash_spring_frac, 0.0), 1.0)
        return v

    def ta(th):
        if model is None or model.aero_torque_coeff == 0 or model.intake_speed_ms <= 0:
            return 0.0
        if model.plate_area_m2 <= 0 or model.plate_radius_m <= 0:
            return 0.0
        mag = model.aero_torque_coeff * 0.5 * model.rho * model.intake_speed_ms ** 2 * model.plate_area_m2 * \
            model.plate_radius_m * math.sin(min(max(th / th_o, 0.0), 1.0) * math.pi)
        return mag if model.aero_opens_plate else -mag

    res = fric + drag
    start = ts(th_o) - max(ta(th_o), 0.0)
    if start <= res:
        return "hung", start

    def acc(th, om):
        drive = -ts(th) + ta(th)
        if abs(om) > w_stall:
            return (drive + res * (1.0 if om < 0 else -1.0)) / I
        if abs(drive) <= res:
            return 0.0
        return (drive - res * (1.0 if drive > 0 else -1.0)) / I

    th, om, t, peak = th_o, 0.0, 0.0, 0.0
    for _ in range(int(t_max / max(dt, 1e-6))):
        k1 = (om, acc(th, om))
        k2 = (om + 0.5 * dt * k1[1], acc(th + 0.5 * dt * k1[0], om + 0.5 * dt * k1[1]))
        k3 = (om + 0.5 * dt * k2[1], acc(th + 0.5 * dt * k2[0], om + 0.5 * dt * k2[1]))
        k4 = (om + dt * k3[1], acc(th + dt * k3[0], om + dt * k3[1]))
        th += dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        om += dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        t += dt
        peak = max(peak, abs(om))
        if th <= 0:
            return "closed", t, peak
        if abs(om) < w_stall and acc(th, 0.0) == 0.0 and th > 1e-4:
            return "stalled", t, math.degrees(th), peak
    return "timeout", t, math.degrees(th), peak


def test_a_clean_snap():
    sp = [S("a", 0.3, 0.8), S("b", 0.25, 0.6)]
    r = tr.simulate_return_snap(sp, tr.ThrottleInertia(4e-4, is_estimate=False), tr.ReturnResistance(0.05, 0.02, 0.01))
    kind, t, peak = _snap(sp, I=4e-4, fric=0.05, drag=0.03)
    assert kind == "closed" and r.returns is True and r.return_time_s == pytest.approx(t)
    assert r.peak_speed_rad_s == pytest.approx(peak) and r.hung_at_deg == 0.0 and r.theta_open_deg == 90.0
    assert r.is_estimate is False and r.trace[0] == (0.0, 90.0, 0.0) and r.trace[-1][1] == 0.0
    assert r.trace[-1][0] == pytest.approx(t)
    (f,) = r.findings
    assert (f.check, f.severity, f.subsystems) == ("throttle-snap", Severity.OK, SUB)
    assert f.message == f"Throttle returns to closed in {t * 1000:.0f} ms. Peak return speed {peak:.1f} rad/s."
    assert f.detail == dict(return_time_s=pytest.approx(t), peak_speed_rad_s=pytest.approx(peak), estimate=False)
    d = r.as_dict()
    assert d == dict(returns=True, return_time_s=r.return_time_s, hung_at_deg=0.0, peak_speed_rad_s=r.peak_speed_rad_s,
                     theta_open_deg=90.0, is_estimate=False, findings=[f.as_dict()], n_trace=len(r.trace))
    est = tr.simulate_return_snap(sp, tr.ReturnResistance(0.05), dt=1e-3)            # arguments swapped, inertia default
    kind, t, peak = _snap(sp, fric=0.05, dt=1e-3)
    assert est.return_time_s == pytest.approx(t) and est.is_estimate is True
    assert est.findings[0].message == (f"Throttle returns to closed in {t * 1000:.0f} ms (ESTIMATED inertia/rate — "
                                       f"confirm with measured values). Peak return speed {peak:.1f} rad/s.")
    swapped = tr.simulate_return_snap(sp, tr.ReturnResistance(0.05), tr.ThrottleInertia(4e-4, is_estimate=False),
                                      dt=1e-3)
    assert swapped.return_time_s == pytest.approx(_snap(sp, I=4e-4, fric=0.05, dt=1e-3)[1])
    plain = tr.simulate_return_snap([S("a", 0.3, 0.8, est=True)], tr.ThrottleInertia(4e-4, is_estimate=False), dt=1e-3)
    assert plain.is_estimate is True and plain.return_time_s == pytest.approx(_snap([S("a", 0.3, 0.8)], I=4e-4,
                                                                                     dt=1e-3)[1])
    tiny = tr.simulate_return_snap(sp, tr.ThrottleInertia(0.0), dt=1e-4, t_max=0.05)
    assert tiny.returns is True


def test_hung_at_wide_open():
    sp = [S("a", 0.1, 0.2)]
    r = tr.simulate_return_snap(sp, None, tr.ReturnResistance(0.15, 0.05), theta_open_deg=80.0)
    assert (r.returns, r.return_time_s, r.hung_at_deg, r.peak_speed_rad_s, r.theta_open_deg, r.trace) == (
        False, math.inf, 80.0, 0.0, 80.0, [(0.0, 80.0, 0.0)])
    (f,) = r.findings
    assert f.severity == Severity.FAIL and f.message == (
        "Throttle HANGS at wide-open: spring closing torque 0.200 N·m can't overcome static friction+drag 0.200 N·m, "
        "so it never starts to return. Time-domain single-fault failure — increase preload/rate, cut "
        "friction/drag/backlash, or reduce aero hold.")
    assert f.detail == dict(net_start=pytest.approx(0.2), T_resist=pytest.approx(0.2), T_aero_open=0.0,
                            estimate=True)
    lash = tr.SnapModel(backlash_deg=5.0, backlash_spring_frac=0.3, intake_speed_ms=20.0)
    r = tr.simulate_return_snap([S("a", 0.3, 0.6)], None, tr.ReturnResistance(0.19), model=lash)
    msg = r.findings[0].message
    assert msg.startswith("Throttle HANGS at wide-open: spring torque through the cable backlash 0.180 N·m can't "
                          "overcome static friction+drag 0.190 N·m, so it never starts")
    assert r.findings[1].check == "throttle-snap-aero" and len(r.findings) == 2
    aero = tr.SnapModel(aero_torque_coeff=50.0, intake_speed_ms=60.0, plate_area_m2=1e-3, plate_radius_m=0.02)
    r = tr.simulate_return_snap([S("a", 0.3, 0.6)], None, tr.ReturnResistance(0.6), theta_open_deg=60.0, model=aero)
    t_open = tr._aero_torque(aero, math.radians(60.0), math.radians(60.0))
    assert t_open > 0                                    # sin(pi) in floating point: a whisker, but positive
    assert r.findings[0].message == (
        f"Throttle HANGS at wide-open: spring closing torque {0.6 - t_open:.3f} N·m can't overcome static friction+drag "
        f"0.600 N·m (aero is holding it open with {t_open:.3f} N·m at 60 m/s), so it never starts to return. "
        "Time-domain single-fault failure — increase preload/rate, cut friction/drag/backlash, or reduce aero hold.")
    assert r.findings[0].detail["T_aero_open"] == pytest.approx(t_open)


def test_stalls_and_timeouts():
    sp = [S("a", 0.05, 0.5)]
    r = tr.simulate_return_snap(sp, tr.ThrottleInertia(5e-4), tr.ReturnResistance(0.3), dt=1e-3)
    kind, t, ang, peak = _snap(sp, fric=0.3, dt=1e-3)
    assert kind == "stalled" and r.returns is False and r.hung_at_deg == pytest.approx(ang)
    assert r.peak_speed_rad_s == pytest.approx(peak) and r.return_time_s == math.inf
    f = r.findings[-1]
    assert f.message == (f"Throttle does NOT fully return within 2000 ms — stalled at {ang:.1f}° (friction/drag too "
                         "high for the spring). Time-domain single-fault failure: add preload/rate or reduce friction.")
    assert f.detail == dict(hung_at_deg=pytest.approx(ang), t_max_s=2.0, estimate=True)
    slow = tr.simulate_return_snap([S("a", 0.3, 0.8)], tr.ThrottleInertia(1.0), dt=1e-3, t_max=0.05)
    kind, t, ang, peak = _snap([S("a", 0.3, 0.8)], I=1.0, dt=1e-3, t_max=0.05)
    assert kind == "timeout" and slow.hung_at_deg == pytest.approx(ang)
    assert slow.findings[-1].message.startswith(f"Throttle does NOT fully return within 50 ms — still at {ang:.1f}° ")
    assert len(slow.trace) == 51


def test_the_rich_model_runs():
    cam = tr.SnapModel(cam_profile=[(0.0, 1.5), (90.0, 0.8)], backlash_deg=4.0, backlash_spring_frac=1.7,
                       aero_torque_coeff=-2.0, intake_speed_ms=30.0, plate_area_m2=2e-3, plate_radius_m=0.02)
    sp = [S("a", 0.3, 0.6), S("b", 0.2, 0.5)]
    r = tr.simulate_return_snap(sp, tr.ThrottleInertia(6e-4), tr.ReturnResistance(0.05), dt=1e-3, model=cam)
    kind, t, peak = _snap(sp, I=6e-4, fric=0.05, dt=1e-3, model=cam)
    assert r.returns and r.return_time_s == pytest.approx(t) and r.peak_speed_rad_s == pytest.approx(peak)
    neg = tr.SnapModel(backlash_deg=-3.0, backlash_spring_frac=-0.5, intake_speed_ms=5.0)
    r = tr.simulate_return_snap(sp, tr.ThrottleInertia(6e-4), None, dt=1e-3, model=neg)
    assert r.returns and r.return_time_s == pytest.approx(_snap(sp, I=6e-4, dt=1e-3, model=neg)[1])
    assert [f.check for f in r.findings] == ["throttle-snap", "throttle-snap-aero"]
    lash = tr.SnapModel(backlash_deg=10.0, backlash_spring_frac=0.5)
    r = tr.simulate_return_snap(sp, tr.ThrottleInertia(6e-4), tr.ReturnResistance(0.05), dt=1e-3, model=lash)
    assert r.return_time_s == pytest.approx(_snap(sp, I=6e-4, fric=0.05, dt=1e-3, model=lash)[1])
    small = tr.simulate_return_snap(sp, tr.ThrottleInertia(6e-4), theta_open_deg=0.0, dt=1e-4)
    assert small.returns and small.theta_open_deg == 0.0
    long = tr.simulate_return_snap([S("a", 0.3, 0.8)], tr.ThrottleInertia(10.0), dt=1e-4, t_max=0.5)
    assert len(long.trace) == 4000


def test_every_single_failure_is_snapped():
    sp = [S("a", 0.3, 0.6), S("b", 0.2, 0.5), S("c", 0.1, 0.3)]
    out = tr.simulate_return_snap_single_failures(sp, tr.ThrottleInertia(6e-4), tr.ReturnResistance(0.05),
                                                  theta_open_deg=70.0, dt=1e-3)
    assert list(out) == ["all springs healthy", "without 'a'", "without 'b'", "without 'c'"]
    assert out["without 'a'"].return_time_s == pytest.approx(_snap(sp[1:], I=6e-4, fric=0.05, open_deg=70.0,
                                                                    dt=1e-3)[1])
    assert out["all springs healthy"].theta_open_deg == 70.0
    one = tr.simulate_return_snap_single_failures([S("a", 0.3, 0.6)], dt=1e-3)
    assert list(one) == ["all springs healthy"]


def test_characterized_outputs():
    """Every result in full — findings (check, severity, message, subsystems, detail), cases, traces — over the
    scenarios above and a few placed on the remaining edges; recorded in tests/golden/throttle_return_*.json."""
    from tests._golden import check
    R = tr.ReturnResistance
    res = {}
    red = {"few": ([S("only", 5.0, 9.0, est=True)], R(0.2)), "clean": ([S("a", 1.0, 2.0), S("b", 1.2, 2.5)],
                                                                     R(0.2, 0.1, 0.0)),
           "tight": ([S("a", 1.0, 3.0), S("b", 1.0, 3.0)], R(0.5)),
           "under": ([S("a", 0.9, 3.0, est=True), S("b", 0.95, 3.0)], R(0.5)),
           "dead": ([S("weak", 0.4, 0.9), S("strong", 2.0, 3.0), S("mid", 0.1, 0.5)], R(0.3, 0.2, 0.1)),
           "zero_r": ([S("a", 1.0, 2.0), S("b", 1.0, 2.0)], R(0.0)),
           "zero_r_est": ([S("a", 1.0, 2.0, est=True), S("b", 1.0, 2.0)], R(0.0)),
           "dead_est": ([S("a", 0.1, 0.2, est=True), S("b", 2.0, 3.0)], R(0.5)),
           "zero_r_fail": ([S("a", 0.0, 0.5), S("b", 1.0, 2.0)], R(0.0))}
    for k, (sp, r) in red.items():
        out = tr.check_return_redundancy(sp, r)
        res["red_" + k] = [out.as_dict(), [f.as_dict() for f in out.findings], tr.return_redundancy_report(out)]
    res["torsion"] = tr.ReturnSpring.from_torsion_spring("tor", 0.02, 30.0, 60.0, is_estimate=False)
    res["k"] = tr.k_compression_spring(1.0, 8.0, 0.5)
    ped = {"default": dict(), "pivot_small": dict(pivot_bolt_dia_mm=0.8, edge_dist_mm=0.9, weld_leg_mm=0.7,
                                                   weld_length_mm=0.6),
           "full": dict(material="Aluminium 7075-T6", pivot_bolt_dia_mm=10.0, edge_dist_mm=15.0, n_bolts=2,
                        weld_leg_mm=4.0, weld_length_mm=60.0, load_N=-2500.0, fos_target=2.0, is_estimate=False),
           "pivot_default_bolts": dict(pivot_bolt_dia_mm=10.0, edge_dist_mm=15.0, weld_leg_mm=4.0, weld_length_mm=60.0),
           "fos": dict(fos_target=3.0), "tight": dict(), "edge_none": dict(pivot_bolt_dia_mm=10.0), "thick": dict()}
    for k, kw in ped.items():
        dims = {"tight": (25.0, 6.0, 150.0), "thick": (30.0, 16.0, 150.0)}.get(k, (40.0, 15.0, 100.0))
        out = tr.check_brake_pedal_2000N(*dims, **kw)
        res["ped_" + k] = [out.verdict, [vars(m) for m in out.modes], out.notes, [f.as_dict() for f in out.findings]]
    res["inertia"] = [tr.estimate_throttle_inertia(), tr.estimate_throttle_inertia(cable_pedal_mass_kg=0.2),
                      tr.estimate_throttle_inertia(pedal_arm_m=0.1)]
    prof = [(0.0, 1.0), (30.0, 0.5), (60.0, 2.0)]
    res["cam"] = [tr._cam_multiplier(x, prof) for x in (0.0, 0.5, 1.0, 29.0, 59.0, 60.0)]
    res["aero"] = [tr._aero_torque(tr.SnapModel(aero_torque_coeff=c, intake_speed_ms=30.0, plate_area_m2=1e-3,
                                                plate_radius_m=0.02), th, math.radians(45.0))
                   for c in (1.0, 0.5) for th in (0.2, 0.5, 0.785, 0.9)]
    res["active"] = [tr.SnapModel(aero_torque_coeff=0.5, intake_speed_ms=0.5).is_active(),
                     tr.SnapModel(intake_speed_ms=0.5).aero_is_unquantified()]
    sp = [S("a", 0.3, 0.8), S("b", 0.25, 0.6)]
    snaps = {"clean": dict(springs=sp, inertia=tr.ThrottleInertia(4e-4, is_estimate=False),
                           resistance=R(0.05, 0.02, 0.01)),
             "slow": dict(springs=[S("a", 0.3, 0.8)], inertia=tr.ThrottleInertia(0.15), dt=1e-3, t_max=3.0),
             "slow_measured": dict(springs=[S("a", 0.3, 0.8)], inertia=tr.ThrottleInertia(0.15, is_estimate=False),
                                   dt=1e-3, t_max=3.0),
             "hung": dict(springs=[S("a", 0.1, 0.2)], inertia=None, resistance=R(0.15, 0.05), theta_open_deg=80.0),
             "lash": dict(springs=[S("a", 0.3, 0.6)], inertia=None, resistance=R(0.19),
                          model=tr.SnapModel(backlash_deg=5.0, backlash_spring_frac=0.3, intake_speed_ms=20.0)),
             "aero_hold": dict(springs=[S("a", 0.3, 0.6)], inertia=None, resistance=R(0.6), theta_open_deg=60.0,
                               model=tr.SnapModel(aero_torque_coeff=50.0, intake_speed_ms=60.0, plate_area_m2=1e-3,
                                                  plate_radius_m=0.02)),
             "aero_mid": dict(springs=[S("a", 0.3, 0.35)], inertia=tr.ThrottleInertia(5e-4), resistance=R(0.02),
                              dt=1e-3, t_max=0.5,
                              model=tr.SnapModel(aero_torque_coeff=30.0, intake_speed_ms=60.0, plate_area_m2=1e-3,
                                                 plate_radius_m=0.02)),
             "stall": dict(springs=[S("a", 0.05, 0.5)], inertia=tr.ThrottleInertia(5e-4), resistance=R(0.3), dt=1e-3),
             "rich": dict(springs=sp, inertia=tr.ThrottleInertia(6e-4), resistance=R(0.05), dt=1e-3,
                          model=tr.SnapModel(cam_profile=[(0.0, 1.5), (90.0, 0.8)], backlash_deg=4.0,
                                             backlash_spring_frac=1.7, aero_torque_coeff=-2.0, intake_speed_ms=30.0,
                                             plate_area_m2=2e-3, plate_radius_m=0.02)),
             "tiny_open": dict(springs=sp, inertia=tr.ThrottleInertia(6e-4), theta_open_deg=0.0, dt=1e-4)}
    for k, kw in snaps.items():
        args = [kw.pop("springs")]
        out = tr.simulate_return_snap(*args, kw.pop("inertia", None), kw.pop("resistance", None), **kw)
        res["snap_" + k] = [out.as_dict(), [f.as_dict() for f in out.findings], out.trace[::25], out.trace[-3:]]
    sf = tr.simulate_return_snap_single_failures([S("a", 0.3, 0.6), S("b", 0.2, 0.5), S("c", 0.1, 0.3)],
                                                 tr.ThrottleInertia(6e-4), R(0.05), theta_open_deg=70.0, dt=1e-3)
    res["single"] = {k: v.as_dict() for k, v in sf.items()}
    sf2 = tr.simulate_return_snap_single_failures([S("a", 0.3, 0.6), S("b", 0.2, 0.5)], dt=2e-3)
    res["single_default"] = {k: [v.as_dict(), v.trace[-1]] for k, v in sf2.items()}
    check("throttle_return_all", res)
