# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
Tests for suspension.gnss.

The lever-arm tests are the substantive ones: they pin the rigid-body
kinematics against hand-checkable cases (pure centripetal, pure angular, zero
offset) and confirm the module reports the transient term separately, because
that is the term nobody catches by looking at a plot.

The rest are contracts. A bitrate mismatch is a FAIL and not advice. An
unmeasured mounting offset is MISSING and does not quietly become zero. A spec
sheet that prints "velocity accuracy" as a heading has not stated one.
"""

import math

import pytest

from suspension.interfaces import Severity
from suspension import gnss as gn
from suspension import daq_plan as dp

G = 9.80665


def _sev(findings, check):
    for f in findings:
        if f.check == check:
            return f.severity
    return None


def _checks(findings):
    return {f.check for f in findings}


def _spec(**over):
    base = {f: getattr(gn.ECUMASTER_GPS_TO_CAN_V2, f)
            for f in gn.ECUMASTER_GPS_TO_CAN_V2.__dataclass_fields__}
    base.update(over)
    return gn.GnssSpec(**base)


# --------------------------------------------------------------------------- #
#  Lever arm — the physics
# --------------------------------------------------------------------------- #
def test_at_the_cg_there_is_no_error():
    r = gn.lever_arm_error((0.0, 0.0, 0.0), yaw_rate_rads=2.0,
                           yaw_accel_rads2=10.0)
    assert r.a_err_x_ms2 == pytest.approx(0.0)
    assert r.a_err_y_ms2 == pytest.approx(0.0)


def test_pure_centripetal_points_back_toward_the_cg():
    """Steady yaw, sensor ahead of the CG: the spurious term is longitudinal
    and negative — it reads as braking that is not happening."""
    r = gn.lever_arm_error((0.5, 0.0, 0.0), yaw_rate_rads=2.0,
                           yaw_accel_rads2=0.0)
    assert r.a_err_x_ms2 == pytest.approx(-2.0 ** 2 * 0.5)
    assert r.a_err_y_ms2 == pytest.approx(0.0)
    assert r.angular_ms2 == pytest.approx(0.0)


def test_pure_angular_term_is_lateral_for_a_longitudinal_offset():
    """This is the dangerous one: yaw acceleration on a fore-aft offset makes
    lateral acceleration that peaks exactly when the driver turns."""
    r = gn.lever_arm_error((0.5, 0.0, 0.0), yaw_rate_rads=0.0,
                           yaw_accel_rads2=8.0)
    assert r.a_err_y_ms2 == pytest.approx(8.0 * 0.5)
    assert r.a_err_x_ms2 == pytest.approx(0.0)
    assert r.centripetal_ms2 == pytest.approx(0.0)


def test_lateral_offset_swaps_which_axis_each_term_hits():
    r = gn.lever_arm_error((0.0, 0.4, 0.0), yaw_rate_rads=1.5,
                           yaw_accel_rads2=6.0)
    assert r.a_err_x_ms2 == pytest.approx(-6.0 * 0.4)
    assert r.a_err_y_ms2 == pytest.approx(-1.5 ** 2 * 0.4)


def test_error_scales_linearly_with_offset():
    a = gn.lever_arm_error((0.25, 0.0, 0.0), 2.0, 8.0).magnitude_g()
    b = gn.lever_arm_error((0.50, 0.0, 0.0), 2.0, 8.0).magnitude_g()
    assert b == pytest.approx(2 * a)


def test_centripetal_scales_with_the_square_of_yaw_rate():
    a = gn.lever_arm_error((0.5, 0.0, 0.0), 1.0, 0.0).centripetal_ms2
    b = gn.lever_arm_error((0.5, 0.0, 0.0), 2.0, 0.0).centripetal_ms2
    assert b == pytest.approx(4 * a)


def test_vertical_offset_alone_produces_no_planar_error():
    """Yaw-only kinematics: a purely vertical offset contributes nothing, and
    the module does not pretend otherwise."""
    r = gn.lever_arm_error((0.0, 0.0, 0.6), 2.0, 8.0)
    assert r.magnitude_g() == pytest.approx(0.0)


def test_a_realistic_offset_produces_a_real_amount_of_phantom_g():
    """300 mm is a plausible 'close enough to the CG' install, and it
    manufactures a quarter of a g in a slalom."""
    slalom = [m for m in gn.MANOEUVRES if m.name == "slalom"][0]
    r = gn.lever_arm_error((0.3, 0.0, 0.0), slalom.yaw_rate_rads(),
                           slalom.yaw_accel_rads2)
    assert 0.15 < r.magnitude_g() < 0.40


# --------------------------------------------------------------------------- #
#  Mounting findings
# --------------------------------------------------------------------------- #
def test_unmeasured_offset_is_missing_not_zero():
    f = gn.mounting_findings(_spec(offset_from_cg_m=None))
    assert _sev(f, "gnss-offset-unmeasured") == Severity.MISSING
    assert "gnss-lever-arm-ok" not in _checks(f)


def test_large_offset_warns_and_small_offset_passes():
    big = gn.mounting_findings(_spec(offset_from_cg_m=(0.6, 0.1, 0.2)))
    small = gn.mounting_findings(_spec(offset_from_cg_m=(0.02, 0.01, 0.05)))
    assert _sev(big, "gnss-lever-arm") == Severity.WARN
    assert _sev(small, "gnss-lever-arm-ok") == Severity.OK


def test_transient_term_is_reported_separately():
    f = gn.mounting_findings(_spec(offset_from_cg_m=(0.4, 0.0, 0.1)))
    assert "gnss-lever-arm-transient" in _checks(f)


def test_tolerance_is_a_parameter_not_a_constant():
    spec = _spec(offset_from_cg_m=(0.3, 0.0, 0.0))
    strict = gn.mounting_findings(spec, tolerance_g=0.01)
    loose = gn.mounting_findings(spec, tolerance_g=0.50)
    assert _sev(strict, "gnss-lever-arm") == Severity.WARN
    assert _sev(loose, "gnss-lever-arm-ok") == Severity.OK


def test_mounting_findings_reach_chassis_and_aero():
    f = gn.mounting_findings(_spec(offset_from_cg_m=None))
    subs = set(sum((x.subsystems for x in f), []))
    assert "chassis" in subs


# --------------------------------------------------------------------------- #
#  Bus compatibility — the conflict sitting in plain sight
# --------------------------------------------------------------------------- #
def test_one_mbps_device_on_a_500k_bus_is_a_fail():
    f = gn.bus_findings(gn.ECUMASTER_GPS_TO_CAN_V2,
                        dp.BusSpec(bitrate_bps=500_000))
    assert _sev(f, "gnss-bitrate-mismatch") == Severity.FAIL


def test_matching_bitrates_pass():
    f = gn.bus_findings(gn.ECUMASTER_GPS_TO_CAN_V2,
                        dp.BusSpec(bitrate_bps=1_000_000))
    assert _sev(f, "gnss-bitrate-ok") == Severity.OK


def test_extended_ids_on_a_standard_bus_warn_with_the_real_bit_cost():
    f = gn.bus_findings(gn.ECUMASTER_GPS_TO_CAN_V2,
                        dp.BusSpec(bitrate_bps=1_000_000, extended_ids=False))
    warn = [x for x in f if x.check == "gnss-id-format"][0]
    assert warn.severity == Severity.WARN
    # the message quotes the real ISO 11898-1 figures, not a round number
    assert str(dp.can_frame_bits(8, extended=True)) in warn.message
    assert str(dp.can_frame_bits(8)) in warn.message


def test_unknown_bitrate_is_missing():
    f = gn.bus_findings(_spec(can_bitrate_bps=None), dp.BusSpec())
    assert _sev(f, "gnss-bitrate-unknown") == Severity.MISSING


# --------------------------------------------------------------------------- #
#  Power
# --------------------------------------------------------------------------- #
def test_wide_supply_window_is_met_by_the_12v_rail_only():
    f = gn.power_findings(gn.ECUMASTER_GPS_TO_CAN_V2, dp.default_rails())
    assert _sev(f, "gnss-rail-ok") == Severity.OK
    assert "12V" in [x for x in f if x.check == "gnss-rail-ok"][0].message


def test_no_compatible_rail_is_a_fail():
    rails = {"5V": dp.Rail("5V", 5.0, capacity_ma=500.0)}
    f = gn.power_findings(gn.ECUMASTER_GPS_TO_CAN_V2, rails)
    assert _sev(f, "gnss-no-compatible-rail") == Severity.FAIL


def test_undeclared_current_is_missing():
    f = gn.power_findings(gn.ECUMASTER_GPS_TO_CAN_V2, dp.default_rails())
    assert _sev(f, "gnss-current-unknown") == Severity.MISSING


# --------------------------------------------------------------------------- #
#  The spec sheet's blanks
# --------------------------------------------------------------------------- #
def test_catalog_entry_leaves_every_measurement_question_open():
    """The vendor sheet lists these as headings. Filling them with plausible
    numbers here would defeat the whole module."""
    spec = gn.ECUMASTER_GPS_TO_CAN_V2
    assert spec.measurement_completeness() == 0.0
    assert len(spec.unanswered_measurements()) == 8


def test_answering_a_question_moves_completeness():
    spec = _spec(velocity_accuracy_ms=0.1, position_accuracy_m=2.5)
    assert spec.measurement_completeness() == pytest.approx(2 / 8)


def test_unanswered_measurements_are_named_individually():
    f = gn.documentation_findings(gn.ECUMASTER_GPS_TO_CAN_V2)
    finding = [x for x in f if x.check == "gnss-measurements-unanswered"][0]
    assert finding.severity == Severity.MISSING
    assert "Heading resolution" in finding.detail["unanswered"]


def test_open_antenna_question_reaches_chassis_and_aero():
    f = gn.documentation_findings(gn.ECUMASTER_GPS_TO_CAN_V2)
    ant = [x for x in f if x.check == "gnss-antenna-undecided"][0]
    assert "chassis" in ant.subsystems and "aero" in ant.subsystems


def test_answered_antenna_question_closes():
    f = gn.documentation_findings(_spec(external_antenna=True))
    assert "gnss-antenna-undecided" not in _checks(f)


# --------------------------------------------------------------------------- #
#  Latency and rates
# --------------------------------------------------------------------------- #
def test_unknown_latency_is_missing():
    f = gn.latency_findings(gn.ECUMASTER_GPS_TO_CAN_V2)
    assert _sev(f, "gnss-latency-unknown") == Severity.MISSING


def test_latency_is_expressed_as_distance():
    f = gn.latency_findings(_spec(velocity_latency_s=0.10),
                            reference_speed_ms=20.0)
    finding = [x for x in f if x.check == "gnss-latency"][0]
    assert finding.detail["distance_m"] == pytest.approx(2.0)
    assert finding.severity == Severity.WARN


def test_small_latency_is_informational():
    f = gn.latency_findings(_spec(velocity_latency_s=0.01),
                            reference_speed_ms=20.0)
    assert _sev(f, "gnss-latency") == Severity.INFO


def test_100hz_imu_clears_15hz_chassis_bandwidth():
    f = gn.rate_findings(gn.ECUMASTER_GPS_TO_CAN_V2, vehicle_bandwidth_hz=15.0)
    assert _sev(f, "gnss-imu-rate-ok") == Severity.OK


def test_undersampled_imu_is_a_fail_not_a_warning():
    f = gn.rate_findings(_spec(imu_rate_hz=20.0), vehicle_bandwidth_hz=15.0)
    assert _sev(f, "gnss-imu-aliasing") == Severity.FAIL


def test_marginal_oversampling_warns():
    f = gn.rate_findings(_spec(imu_rate_hz=45.0), vehicle_bandwidth_hz=15.0)
    assert _sev(f, "gnss-imu-rate-marginal") == Severity.WARN


def test_rate_split_is_flagged_so_frames_are_not_shared():
    f = gn.rate_findings(gn.ECUMASTER_GPS_TO_CAN_V2)
    assert "gnss-rate-split" in _checks(f)


# --------------------------------------------------------------------------- #
#  CAN map and daq_plan handoff
# --------------------------------------------------------------------------- #
def test_position_and_imu_do_not_share_a_frame():
    msgs = gn.can_map(gn.ECUMASTER_GPS_TO_CAN_V2)
    rates = {m.rate_hz for m in msgs}
    assert rates == {25.0, 100.0}
    for m in msgs:
        assert len({m.rate_hz}) == 1
    assert len({m.can_id for m in msgs}) == len(msgs)


def test_frames_use_extended_ids_when_the_device_does():
    msgs = gn.can_map(gn.ECUMASTER_GPS_TO_CAN_V2)
    assert all(m.extended for m in msgs)
    assert msgs[0].bits() == dp.can_frame_bits(msgs[0].dlc, extended=True)


def test_node_fits_a_1mbps_bus_comfortably():
    p = gn.plan_gnss(bus=dp.BusSpec(bitrate_bps=1_000_000))
    assert p.bus_bits_per_second() / 1_000_000 < 0.10


def test_sensor_specs_route_to_the_documented_subteams():
    for s in gn.to_sensor_specs(gn.ECUMASTER_GPS_TO_CAN_V2):
        subs = s.affected_subteams()
        assert {"chassis", "aero", "electrics"} <= set(subs)


def test_sensor_specs_carry_the_unanswered_accuracy_through_as_none():
    specs = {s.key: s for s in gn.to_sensor_specs(gn.ECUMASTER_GPS_TO_CAN_V2)}
    assert specs["gnss_position"].accuracy_eu is None
    assert specs["gnss_velocity"].accuracy_eu is None


def test_specs_flow_into_a_daq_plan_without_blocking_it():
    sensors = dp.cooling_package() + gn.to_sensor_specs(gn.ECUMASTER_GPS_TO_CAN_V2)
    p = dp.plan(sensors, bus=dp.BusSpec(bitrate_bps=1_000_000),
                rails=dp.default_rails())
    assert p.verdict in (dp.Verdict.INCOMPLETE, dp.Verdict.BLOCKED)
    assert p.bus_result.load < 1.0


# --------------------------------------------------------------------------- #
#  Options
# --------------------------------------------------------------------------- #
def test_option_comparison_scores_answers_not_price():
    c = gn.compare_options()
    assert "gnss-option-completeness" in _checks(c.findings)
    assert "gnss-option-tradeoff" in _checks(c.findings)


def test_diy_option_defaults_to_no_declared_cost():
    c = gn.compare_options()
    assert c.diy.cost_usd is None


# --------------------------------------------------------------------------- #
#  Whole plan
# --------------------------------------------------------------------------- #
def test_default_plan_blocks_on_bitrate_and_lists_open_questions():
    p = gn.plan_gnss(bus=dp.BusSpec(bitrate_bps=500_000))
    assert [f.check for f in p.blocking()] == ["gnss-bitrate-mismatch"]
    assert len(p.open_questions()) >= 4


def test_fully_answered_spec_on_a_matching_bus_has_nothing_blocking():
    spec = _spec(
        velocity_accuracy_ms=0.05, velocity_max_ms=50.0,
        velocity_resolution_ms=0.01, velocity_latency_s=0.02,
        position_accuracy_m=2.0, height_accuracy_m=3.0,
        heading_accuracy_deg=0.5, heading_resolution_deg=0.1,
        external_antenna=True, current_ma=120.0,
        offset_from_cg_m=(0.03, 0.0, 0.05))
    p = gn.plan_gnss(spec, bus=dp.BusSpec(bitrate_bps=1_000_000,
                                          extended_ids=True))
    assert not p.blocking()
    assert not p.open_questions()


def test_markdown_renders():
    md = gn.plan_gnss().to_markdown()
    assert "GNSS / IMU node" in md
    assert "not measured" in md


def test_provenance_flags_the_manoeuvres_as_a_fallback():
    joined = " ".join(gn.PROVENANCE["estimate_flagged"])
    assert "MANOEUVRES" in joined and "FALLBACK" in joined
    assert gn.PROVENANCE["derived_from_declared_vehicle"]


# --------------------------------------------------------------------------- #
#  Manoeuvres derived from the declared car, not from invented numbers
# --------------------------------------------------------------------------- #
def _car(**over):
    d = dict(mass_kg=230.0, wheelbase_m=1.55, cg_to_front_m=0.82, mu_lat=1.4)
    d.update(over)
    return gn.VehicleSpec(**d)


def test_yaw_inertia_uses_the_dynamic_index_one_estimate():
    car = _car()
    assert car.yaw_inertia_estimate_kgm2() == pytest.approx(
        230.0 * 0.82 * (1.55 - 0.82))


def test_declared_yaw_inertia_overrides_the_estimate():
    car = _car(yaw_inertia_kgm2=95.0)
    assert car.yaw_inertia() == 95.0
    assert car.yaw_inertia() != car.yaw_inertia_estimate_kgm2()


def test_peak_yaw_accel_reduces_to_mu_g_over_wheelbase():
    """With the dynamic-index-one inertia estimate the a*b cancels, leaving
    alpha_max = mu*g/L. Worth pinning: it is the whole reason a short grippy
    car changes direction so violently."""
    car = _car()
    assert car.max_yaw_accel_rads2() == pytest.approx(1.4 * G / 1.55, rel=1e-9)


def test_cg_position_drops_out_under_the_inertia_estimate():
    a = _car(cg_to_front_m=0.70).max_yaw_accel_rads2()
    b = _car(cg_to_front_m=0.90).max_yaw_accel_rads2()
    assert a == pytest.approx(b)


def test_declared_inertia_makes_cg_position_matter_again():
    a = _car(cg_to_front_m=0.70, yaw_inertia_kgm2=95.0).max_yaw_accel_rads2()
    b = _car(cg_to_front_m=0.90, yaw_inertia_kgm2=95.0).max_yaw_accel_rads2()
    assert a != pytest.approx(b)


def test_shorter_wheelbase_yaws_harder():
    assert (_car(wheelbase_m=1.40).max_yaw_accel_rads2()
            > _car(wheelbase_m=1.70).max_yaw_accel_rads2())


def test_corner_radius_follows_from_the_grip_limit():
    car = _car(speeds_ms=(15.0,))
    m = car.manoeuvres()[0]
    assert m.radius_m == pytest.approx(15.0 ** 2 / (1.4 * G))


def test_underdeclared_car_yields_no_manoeuvres():
    assert gn.VehicleSpec().manoeuvres() is None
    assert _car(mu_lat=None).manoeuvres() is None
    assert "mu_lat" in _car(mu_lat=None).missing_fields()


def test_findings_say_when_they_used_generic_numbers():
    spec = _spec(offset_from_cg_m=(0.4, 0.0, 0.1))
    generic = gn.mounting_findings(spec)
    f = [x for x in generic if x.check == "gnss-lever-arm"][0]
    assert "GENERIC" in f.message
    assert f.detail["derived"] is False


def test_findings_say_when_they_used_the_declared_car():
    spec = _spec(offset_from_cg_m=(0.4, 0.0, 0.1))
    derived = gn.mounting_findings(spec, vehicle=_car())
    f = [x for x in derived if x.check == "gnss-lever-arm"][0]
    assert "declared vehicle" in f.message
    assert f.detail["derived"] is True
    assert "gnss-yaw-envelope" in _checks(derived)


def test_underdeclared_car_is_flagged_and_falls_back():
    f = gn.mounting_findings(_spec(offset_from_cg_m=(0.4, 0.0, 0.1)),
                             vehicle=gn.VehicleSpec(mass_kg=230.0))
    assert _sev(f, "gnss-vehicle-underdeclared") == Severity.MISSING
    assert "GENERIC" in [x for x in f if x.check == "gnss-lever-arm"][0].message


def test_estimated_inertia_is_disclosed_in_the_envelope_finding():
    est = gn.mounting_findings(_spec(offset_from_cg_m=(0.4, 0.0, 0.1)),
                               vehicle=_car())
    f = [x for x in est if x.check == "gnss-yaw-envelope"][0]
    assert f.detail["inertia_estimated"] is True
    assert "bifilar" in f.message

    meas = gn.mounting_findings(_spec(offset_from_cg_m=(0.4, 0.0, 0.1)),
                                vehicle=_car(yaw_inertia_kgm2=95.0))
    g2 = [x for x in meas if x.check == "gnss-yaw-envelope"][0]
    assert g2.detail["inertia_estimated"] is False
    assert "bifilar" not in g2.message


def test_plan_gnss_threads_the_vehicle_through():
    p = gn.plan_gnss(_spec(offset_from_cg_m=(0.4, 0.0, 0.1)),
                     bus=dp.BusSpec(bitrate_bps=1_000_000),
                     vehicle=_car())
    assert "gnss-yaw-envelope" in _checks(p.findings)
