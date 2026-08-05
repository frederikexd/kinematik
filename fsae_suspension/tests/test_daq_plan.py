# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for suspension.daq_plan — the vehicle-side acquisition planner.

These exercise the contract the module promises: the CAN frame arithmetic
matches the published worst-case figures, an undeclared channel makes every
budget a FLOOR rather than an answer, Nyquist violations are FAIL and not
advice, the BMS bridge refuses to invent a frame map, the tractive-system
isolation boundary is enforced by location rather than by memory, and READY is
genuinely unreachable while any review question is open.
"""

import math
import pathlib

import pytest

from suspension.interfaces import Severity
from suspension import daq_plan as dp


# --------------------------------------------------------------------------- #
#  helpers
# --------------------------------------------------------------------------- #
def _sev(findings, check):
    """Severity of the first finding with this check name, or None."""
    for f in findings:
        if f.check == check:
            return f.severity
    return None


def _checks(findings):
    return {f.check for f in findings}


def _complete_sensor(**over):
    """A sensor with every review question answered — the READY baseline."""
    d = dict(
        key="susp_pot_fl", name="Front-left damper position",
        measures="damper displacement", unit="mm",
        why="motion ratio validation and damper velocity histograms",
        location="damper", output=dp.OutputType.ANALOG_V,
        supply_v=5.0, current_ma=8.0, supply_rail="5V",
        connector="AS 0-way", conductors=3,
        signal_bandwidth_hz=30.0, sample_rate_hz=500.0,
        antialias_cutoff_hz=100.0, adc_bits=12,
        range_min_eu=0.0, range_max_eu=75.0,
        accuracy_eu=0.15, resolution_needed_eu=0.1,
        logged_to="logger", payload_bytes=2,
        calibration="dial-indicator sweep at 5 mm steps, fit and record residual",
        owner="daq", source="ELPM-75",
    )
    d.update(over)
    return dp.SensorSpec(**d)


# ===================================================================== #
#  1.  CAN FRAME ARITHMETIC
# ===================================================================== #
class TestCanFrameBits:
    def test_standard_8_byte_worst_case_is_135_bits(self):
        """The published worst-case figure for an 8-byte standard frame."""
        assert dp.can_frame_bits(8) == 135

    def test_extended_8_byte_worst_case_is_160_bits(self):
        assert dp.can_frame_bits(8, extended=True) == 160

    def test_unstuffed_lengths_match_the_field_layout(self):
        # SOF1 + ID11 + RTR1 + IDE1 + r0_1 + DLC4 + data64 + CRC15 + del1
        # + ACK1 + del1 + EOF7 + IFS3 = 111
        assert dp.can_frame_bits(8, worst_case_stuffing=False) == 111
        # extended adds SRR + 18-bit extension + r1 = 20 bits
        assert dp.can_frame_bits(8, extended=True,
                                 worst_case_stuffing=False) == 131

    def test_stuffing_only_ever_adds_bits(self):
        for n in range(9):
            for ext in (False, True):
                assert (dp.can_frame_bits(n, extended=ext)
                        >= dp.can_frame_bits(n, extended=ext,
                                             worst_case_stuffing=False))

    def test_length_is_monotonic_in_payload(self):
        lens = [dp.can_frame_bits(n) for n in range(9)]
        assert lens == sorted(lens)
        # each extra byte is 8 data bits plus at most 2 stuff bits
        for a, b in zip(lens, lens[1:]):
            assert 8 <= b - a <= 10

    def test_dlc_outside_can20_range_is_rejected(self):
        with pytest.raises(ValueError):
            dp.can_frame_bits(9)
        with pytest.raises(ValueError):
            dp.can_frame_bits(-1)


# ===================================================================== #
#  2.  BUS LOAD AND LATENCY
# ===================================================================== #
class TestBusLoad:
    def test_load_is_the_exact_sum_of_frame_bits_times_rate(self):
        bus = dp.BusSpec(bitrate_bps=500_000)
        msgs = [dp.CanMessage("a", 0x100, 8, 100.0),
                dp.CanMessage("b", 0x101, 8, 100.0)]
        r = dp.bus_load(msgs, bus)
        expected = 2 * 135 * 100.0 / 500_000
        assert r.load == pytest.approx(expected)

    def test_unstuffed_load_is_lower_than_worst_case(self):
        bus = dp.BusSpec(bitrate_bps=500_000)
        msgs = [dp.CanMessage("a", 0x100, 8, 500.0)]
        r = dp.bus_load(msgs, bus)
        assert r.load_unstuffed < r.load

    def test_over_budget_bus_is_a_hard_fail(self):
        bus = dp.BusSpec(bitrate_bps=125_000)
        msgs = [dp.CanMessage(f"m{i}", 0x100 + i, 8, 200.0) for i in range(6)]
        r = dp.bus_load(msgs, bus)
        assert r.load > bus.load_fail
        assert _sev(r.findings, "bus-over-budget") == Severity.FAIL

    def test_quiet_bus_reports_ok(self):
        bus = dp.BusSpec(bitrate_bps=1_000_000)
        msgs = [dp.CanMessage("a", 0x100, 2, 10.0)]
        r = dp.bus_load(msgs, bus)
        assert _sev(r.findings, "bus-load-ok") == Severity.OK

    def test_duplicate_identifier_is_a_fail(self):
        """Two producers on one ID corrupts arbitration; it does not merge."""
        msgs = [dp.CanMessage("a", 0x200, 8, 10.0, producer="daq"),
                dp.CanMessage("b", 0x200, 8, 10.0, producer="bms")]
        r = dp.bus_load(msgs, dp.BusSpec())
        assert _sev(r.findings, "can-id-collision") == Severity.FAIL

    def test_identifier_outside_11_bit_range_is_a_fail(self):
        msgs = [dp.CanMessage("a", 0x800, 8, 10.0)]
        r = dp.bus_load(msgs, dp.BusSpec())
        assert _sev(r.findings, "can-id-range") == Severity.FAIL

    def test_extended_ids_allow_the_larger_range(self):
        msgs = [dp.CanMessage("a", 0x800, 8, 10.0, extended=True)]
        r = dp.bus_load(msgs, dp.BusSpec(extended_ids=True))
        assert "can-id-range" not in _checks(r.findings)

    def test_empty_bus_is_zero_load_not_an_error(self):
        r = dp.bus_load([], dp.BusSpec())
        assert r.load == 0.0
        assert r.messages == 0


class TestLatency:
    def test_highest_priority_message_waits_only_for_blocking(self):
        """The lowest ID cannot be preempted, so it waits at most one frame."""
        bus = dp.BusSpec(bitrate_bps=500_000)
        hi = dp.CanMessage("hi", 0x100, 2, 100.0)
        lo = dp.CanMessage("lo", 0x700, 8, 10.0)
        r = dp.bus_load([hi, lo], bus)
        blocking = lo.frame_time_s(bus.bitrate_bps)
        own = hi.frame_time_s(bus.bitrate_bps)
        assert r.latencies["hi"] == pytest.approx(blocking + own)

    def test_lower_priority_waits_longer_than_higher(self):
        bus = dp.BusSpec(bitrate_bps=500_000)
        msgs = [dp.CanMessage(f"m{i}", 0x100 + i, 8, 200.0) for i in range(5)]
        r = dp.bus_load(msgs, bus)
        lats = [r.latencies[f"m{i}"] for i in range(5)]
        assert lats == sorted(lats), "latency must grow as priority falls"

    def test_saturated_bus_makes_low_priority_messages_undeliverable(self):
        bus = dp.BusSpec(bitrate_bps=125_000)
        msgs = [dp.CanMessage(f"m{i}", 0x100 + i, 8, 150.0) for i in range(8)]
        r = dp.bus_load(msgs, bus)
        assert r.unschedulable, "a saturated bus must name its casualties"
        assert _sev(r.findings, "message-deadline-miss") == Severity.FAIL
        # the victim is a low-priority message, never the highest
        assert "m0" not in r.unschedulable

    def test_single_message_meets_its_own_deadline(self):
        bus = dp.BusSpec(bitrate_bps=500_000)
        r = dp.bus_load([dp.CanMessage("solo", 0x100, 8, 100.0)], bus)
        assert math.isfinite(r.latencies["solo"])
        assert not r.unschedulable


# ===================================================================== #
#  3.  THE CHECKLIST AS A SCHEMA
# ===================================================================== #
class TestChecklist:
    def test_a_bare_sensor_answers_nothing(self):
        s = dp.SensorSpec(key="x", name="X")
        assert s.completeness() == 0.0
        assert len(s.unanswered()) == len(dp.CHECKLIST)

    def test_a_fully_specified_sensor_answers_everything(self):
        s = _complete_sensor()
        assert s.completeness() == 1.0
        assert s.unanswered() == []

    def test_none_is_unanswered_and_zero_is_an_answer(self):
        """The distinction the whole module rests on."""
        blank = _complete_sensor(current_ma=None)
        declared = _complete_sensor(current_ma=0.0)
        assert blank.completeness() < 1.0
        assert declared.completeness() == 1.0

    def test_partial_answers_do_not_close_a_question(self):
        """Power needs voltage AND current AND a rail; two of three is open."""
        s = _complete_sensor(supply_rail=None)
        assert "What voltage does it need / how do we power it?" in s.unanswered()

    def test_bus_native_existing_device_waives_only_power_and_wiring(self):
        s = dp.CATALOG["inverter_temp"]
        na = s.not_applicable()
        assert na == {"power", "wiring"}

    def test_waiver_requires_both_bus_output_and_an_existing_device(self):
        """A new CAN sensor of your own still has to answer for its power."""
        s = dp.CATALOG["inverter_temp"]
        own = dp.catalog("inverter_temp", available_on_existing_bus=None)
        assert own.not_applicable() == set()
        assert s.not_applicable()

    def test_completeness_is_measured_over_applicable_questions_only(self):
        s = dp.CATALOG["inverter_temp"]
        # it answers everything that applies to it
        assert s.completeness() == pytest.approx(1.0)


# ===================================================================== #
#  4.  NYQUIST AND THE SIGNAL CHAIN
# ===================================================================== #
class TestSignalChain:
    def test_undersampling_is_a_fail_not_a_warning(self):
        """Aliasing is unrecoverable, so it cannot be advice."""
        s = _complete_sensor(signal_bandwidth_hz=30.0, sample_rate_hz=40.0)
        assert _sev(dp.signal_chain_findings(s), "nyquist-violation") == Severity.FAIL

    def test_just_above_the_theorem_is_still_flagged_as_marginal(self):
        s = _complete_sensor(signal_bandwidth_hz=30.0, sample_rate_hz=90.0)
        assert _sev(dp.signal_chain_findings(s), "nyquist-marginal") == Severity.WARN

    def test_comfortable_oversampling_passes(self):
        s = _complete_sensor(signal_bandwidth_hz=30.0, sample_rate_hz=300.0)
        assert _sev(dp.signal_chain_findings(s), "nyquist-ok") == Severity.OK

    def test_gross_oversampling_is_reported_as_waste_not_as_error(self):
        s = _complete_sensor(signal_bandwidth_hz=0.5, sample_rate_hz=1000.0)
        assert _sev(dp.signal_chain_findings(s), "oversampled") == Severity.INFO

    def test_missing_rate_or_bandwidth_is_missing_not_pass(self):
        for kw in ({"sample_rate_hz": None}, {"signal_bandwidth_hz": None}):
            s = _complete_sensor(**kw)
            assert _sev(dp.signal_chain_findings(s),
                        "rate-undeclared") == Severity.MISSING

    def test_analog_channel_without_an_antialias_filter_is_warned(self):
        s = _complete_sensor(antialias_cutoff_hz=None)
        assert _sev(dp.signal_chain_findings(s),
                    "antialias-undeclared") == Severity.WARN

    def test_filter_above_nyquist_defeats_its_own_purpose(self):
        s = _complete_sensor(sample_rate_hz=100.0, antialias_cutoff_hz=80.0,
                             signal_bandwidth_hz=10.0)
        assert _sev(dp.signal_chain_findings(s),
                    "antialias-too-high") == Severity.FAIL

    def test_bus_native_channel_needs_no_antialias_filter(self):
        s = dp.CATALOG["inverter_temp"]
        assert "antialias-undeclared" not in _checks(dp.signal_chain_findings(s))

    def test_adc_too_coarse_for_the_required_resolution_fails(self):
        s = _complete_sensor(adc_bits=8, range_min_eu=0.0, range_max_eu=75.0,
                             resolution_needed_eu=0.1)
        assert _sev(dp.signal_chain_findings(s),
                    "adc-resolution-short") == Severity.FAIL

    def test_adc_resolution_finer_than_sensor_accuracy_is_noted(self):
        s = _complete_sensor(adc_bits=16, range_min_eu=0.0, range_max_eu=75.0,
                             accuracy_eu=1.0, resolution_needed_eu=0.5)
        assert _sev(dp.signal_chain_findings(s),
                    "adc-over-resolved") == Severity.INFO

    def test_pulse_channel_is_identified_as_counted_not_sampled(self):
        s = dp.CATALOG["coolant_flow"]
        assert _sev(dp.signal_chain_findings(s),
                    "counter-not-sampled") == Severity.INFO


# ===================================================================== #
#  5.  ISOLATION AND CROSS-SUBTEAM ROUTING
# ===================================================================== #
class TestIntegration:
    def test_tractive_system_location_demands_isolation(self):
        for loc in ("motor", "inverter", "accumulator"):
            s = _complete_sensor(location=loc, galvanic_isolation=None)
            assert _sev(dp.integration_findings(s),
                        "isolation-required") == Severity.MISSING

    def test_explicitly_unisolated_tractive_sensor_is_a_fail(self):
        s = _complete_sensor(location="motor", galvanic_isolation=False)
        assert _sev(dp.integration_findings(s),
                    "isolation-required") == Severity.FAIL

    def test_declared_isolation_passes(self):
        s = _complete_sensor(location="motor", galvanic_isolation=True)
        assert _sev(dp.integration_findings(s), "isolation-ok") == Severity.OK

    def test_low_voltage_location_needs_no_isolation(self):
        s = _complete_sensor(location="damper")
        assert "isolation-required" not in _checks(dp.integration_findings(s))

    def test_a_value_already_on_the_bus_is_flagged_as_duplicated_effort(self):
        s = dp.CATALOG["inverter_temp"]
        assert _sev(dp.integration_findings(s),
                    "already-on-bus") == Severity.WARN

    def test_affected_subteams_are_derived_from_the_mounting_location(self):
        s = _complete_sensor(location="upright")
        teams = s.affected_subteams()
        assert "suspension" in teams and "brakes" in teams

    def test_data_acq_and_electrics_are_always_affected(self):
        s = _complete_sensor(location="chassis")
        for t in dp.ALWAYS_AFFECTED:
            assert t in s.affected_subteams()

    def test_unknown_location_is_flagged_because_nobody_owns_the_bracket(self):
        s = _complete_sensor(location="somewhere_on_the_car")
        assert _sev(dp.integration_findings(s),
                    "location-unknown") == Severity.WARN


# ===================================================================== #
#  6.  POWER AND STORAGE
# ===================================================================== #
class TestPower:
    def test_current_sums_onto_the_assigned_rail(self):
        rails = dp.default_rails()
        ss = [_complete_sensor(key=f"s{i}", current_ma=20.0, supply_rail="5V")
              for i in range(4)]
        r = dp.power_budget(ss, rails)
        assert r.per_rail["5V"]["draw_ma"] == pytest.approx(80.0)

    def test_exceeding_the_regulator_is_a_fail(self):
        rails = {"5V": dp.Rail("5V", 5.0, capacity_ma=100.0)}
        ss = [_complete_sensor(key=f"s{i}", current_ma=40.0, supply_rail="5V")
              for i in range(4)]
        r = dp.power_budget(ss, rails)
        assert _sev(r.findings, "rail-over-capacity") == Severity.FAIL

    def test_voltage_mismatch_against_the_rail_is_a_fail(self):
        rails = dp.default_rails()
        s = _complete_sensor(supply_v=12.0, supply_rail="5V")
        r = dp.power_budget([s], rails)
        assert _sev(r.findings, "rail-voltage-mismatch") == Severity.FAIL

    def test_undeclared_current_makes_the_total_a_floor(self):
        rails = dp.default_rails()
        ss = [_complete_sensor(current_ma=None)]
        r = dp.power_budget(ss, rails)
        assert r.is_floor
        assert _sev(r.findings, "current-undeclared") == Severity.MISSING

    def test_current_on_no_rail_is_reported_rather_than_silently_dropped(self):
        r = dp.power_budget([_complete_sensor(supply_rail=None)],
                            dp.default_rails())
        assert _sev(r.findings, "rail-unassigned") == Severity.MISSING


class TestStorage:
    def test_bytes_per_second_scales_with_rate(self):
        lg = dp.LoggerSpec(record_overhead_bytes=0)
        one = dp.storage_budget([_complete_sensor(sample_rate_hz=100.0,
                                                  payload_bytes=2)], lg)
        two = dp.storage_budget([_complete_sensor(sample_rate_hz=200.0,
                                                  payload_bytes=2)], lg)
        assert two.bytes_per_s == pytest.approx(2 * one.bytes_per_s)

    def test_a_session_that_does_not_fit_the_card_is_a_fail(self):
        lg = dp.LoggerSpec(storage_mb=1.0, session_minutes=30.0)
        ss = [_complete_sensor(key=f"s{i}", sample_rate_hz=1000.0,
                               payload_bytes=4) for i in range(20)]
        r = dp.storage_budget(ss, lg)
        assert _sev(r.findings, "storage-over") == Severity.FAIL

    def test_channels_without_a_rate_make_storage_a_floor(self):
        r = dp.storage_budget([_complete_sensor(sample_rate_hz=None)],
                              dp.LoggerSpec())
        assert r.is_floor


# ===================================================================== #
#  7.  DELTA-T ERROR PROPAGATION
# ===================================================================== #
class TestDeltaT:
    def test_difference_inherits_both_errors_in_quadrature(self):
        a = _complete_sensor(key="in", accuracy_eu=2.0, unit="degC")
        b = _complete_sensor(key="out", accuracy_eu=2.0, unit="degC")
        r = dp.delta_t_budget(a, b, expected_delta_t_k=5.0)
        assert r.sigma_delta_t_k == pytest.approx(math.sqrt(8.0))

    def test_a_cheap_pair_cannot_measure_a_small_rise(self):
        a = _complete_sensor(key="in", accuracy_eu=2.0)
        b = _complete_sensor(key="out", accuracy_eu=2.0)
        r = dp.delta_t_budget(a, b, expected_delta_t_k=3.0)
        assert r.relative_error > 0.5
        assert _sev(r.findings, "delta-t-unusable") == Severity.FAIL

    def test_matched_pair_calibration_rescues_the_same_hardware(self):
        a = _complete_sensor(key="in", accuracy_eu=2.0)
        b = _complete_sensor(key="out", accuracy_eu=2.0)
        loose = dp.delta_t_budget(a, b, expected_delta_t_k=6.0)
        tight = dp.delta_t_budget(a, b, expected_delta_t_k=6.0,
                                  matched_pair=True)
        assert tight.sigma_delta_t_k < loose.sigma_delta_t_k
        assert _sev(tight.findings, "delta-t-ok") == Severity.OK

    def test_heat_rejection_carries_flow_and_temperature_error_together(self):
        a = _complete_sensor(key="in", accuracy_eu=0.2)
        b = _complete_sensor(key="out", accuracy_eu=0.2)
        r = dp.delta_t_budget(a, b, expected_delta_t_k=10.0, flow_lpm=15.0,
                              flow_accuracy_frac=0.03)
        assert r.heat_kw is not None and r.heat_kw > 0
        # combined error is at least as large as either contribution
        assert r.heat_relative_error >= r.relative_error
        assert r.heat_relative_error >= 0.03

    def test_heat_rejection_matches_m_dot_cp_delta_t(self):
        a = _complete_sensor(key="in", accuracy_eu=0.1)
        b = _complete_sensor(key="out", accuracy_eu=0.1)
        r = dp.delta_t_budget(a, b, expected_delta_t_k=10.0, flow_lpm=12.0)
        m_dot = dp.COOLANT_RHO * (12.0 / 1000.0 / 60.0)
        assert r.heat_kw == pytest.approx(m_dot * dp.COOLANT_CP * 10.0 / 1000.0)

    def test_undeclared_accuracy_refuses_to_produce_a_number(self):
        a = _complete_sensor(key="in", accuracy_eu=None)
        b = _complete_sensor(key="out", accuracy_eu=1.0)
        r = dp.delta_t_budget(a, b, expected_delta_t_k=5.0)
        assert _sev(r.findings, "delta-t-uncheckable") == Severity.MISSING
        assert r.heat_kw is None


# ===================================================================== #
#  8.  THE BMS BRIDGE
# ===================================================================== #
class TestBmsBridge:
    def test_uart_framing_is_ten_bits_per_byte_for_8N1(self):
        assert dp.UartLink(data_bits=8, parity=None, stop_bits=1).bits_per_byte() == 10

    def test_parity_and_extra_stop_bits_cost_throughput(self):
        assert dp.UartLink(parity="even").bits_per_byte() == 11
        assert dp.UartLink(stop_bits=2).bits_per_byte() == 11

    def test_link_utilisation_is_frame_time_times_rate(self):
        link = dp.UartLink(baud=115200, frame_bytes=64, frame_rate_hz=10.0)
        assert link.utilisation() == pytest.approx(64 * 10 / 115200 * 10.0)

    def test_empty_signal_list_refuses_rather_than_inventing_a_map(self):
        """The core refusal: no datasheet, no frame layout."""
        link = dp.UartLink(frame_bytes=64, frame_rate_hz=10.0)
        b = dp.plan_bms_bridge(link, [])
        assert b.refused
        assert b.messages == []
        assert _sev(b.findings, "bms-signals-unknown") == Severity.MISSING

    def test_a_link_that_cannot_carry_the_frame_rate_fails(self):
        link = dp.UartLink(baud=9600, frame_bytes=200, frame_rate_hz=50.0)
        b = dp.plan_bms_bridge(link, [dp.BmsSignal("v", 16)])
        assert _sev(b.findings, "uart-over-capacity") == Severity.FAIL

    def test_undeclared_isolation_is_raised_because_the_bms_is_on_the_ts_side(self):
        link = dp.UartLink(frame_bytes=32, frame_rate_hz=10.0)
        b = dp.plan_bms_bridge(link, [dp.BmsSignal("v", 16)])
        assert _sev(b.findings, "bms-isolation") == Severity.MISSING

    def test_declared_unisolated_bridge_is_a_hard_fail(self):
        link = dp.UartLink(frame_bytes=32, frame_rate_hz=10.0)
        b = dp.plan_bms_bridge(link, [dp.BmsSignal("v", 16)], isolated=False)
        assert _sev(b.findings, "bms-isolation") == Severity.FAIL

    def test_isolated_bridge_passes(self):
        link = dp.UartLink(frame_bytes=32, frame_rate_hz=10.0)
        b = dp.plan_bms_bridge(link, [dp.BmsSignal("v", 16)], isolated=True)
        assert _sev(b.findings, "bms-isolation-ok") == Severity.OK

    def test_cannot_publish_faster_than_the_bms_produces(self):
        link = dp.UartLink(frame_bytes=32, frame_rate_hz=10.0)
        b = dp.plan_bms_bridge(
            link, [dp.BmsSignal("v", 16, rate_hz=100.0)], isolated=True)
        assert _sev(b.findings, "bridge-rate-impossible") == Severity.FAIL

    def test_shutdown_relevant_latency_is_called_out_as_not_protective(self):
        link = dp.UartLink(baud=9600, frame_bytes=200, frame_rate_hz=1.0)
        b = dp.plan_bms_bridge(
            link, [dp.BmsSignal("cell_v_min", 16, critical=True, rate_hz=1.0)],
            isolated=True, bus=dp.BusSpec())
        assert _sev(b.findings, "bridge-latency-high") == Severity.WARN


class TestSignalPacking:
    def test_signals_of_different_rates_never_share_a_frame(self):
        sigs = [dp.BmsSignal("fast", 16, rate_hz=100.0),
                dp.BmsSignal("slow", 16, rate_hz=1.0)]
        msgs = dp.pack_signals(sigs)
        assert len(msgs) == 2
        for m in msgs:
            assert len(set(m.signals)) == len(m.signals)

    def test_faster_groups_receive_higher_priority_identifiers(self):
        sigs = [dp.BmsSignal("fast", 16, rate_hz=100.0),
                dp.BmsSignal("slow", 16, rate_hz=1.0)]
        msgs = dp.pack_signals(sigs, base_id=0x300)
        fast = next(m for m in msgs if m.rate_hz == 100.0)
        slow = next(m for m in msgs if m.rate_hz == 1.0)
        assert fast.can_id < slow.can_id, "lower ID wins arbitration"

    def test_critical_signals_lead_their_rate_group(self):
        sigs = [dp.BmsSignal("aaa_routine", 16, rate_hz=10.0),
                dp.BmsSignal("zzz_critical", 16, rate_hz=10.0, critical=True)]
        msgs = dp.pack_signals(sigs)
        assert msgs[0].signals[0] == "zzz_critical"

    def test_packing_never_exceeds_eight_bytes(self):
        sigs = [dp.BmsSignal(f"s{i}", 16, rate_hz=10.0) for i in range(9)]
        msgs = dp.pack_signals(sigs)
        assert all(1 <= m.dlc <= 8 for m in msgs)
        assert sum(len(m.signals) for m in msgs) == 9

    def test_every_signal_is_placed_exactly_once(self):
        sigs = [dp.BmsSignal(f"s{i}", 8 * (1 + i % 4), rate_hz=10.0)
                for i in range(20)]
        msgs = dp.pack_signals(sigs)
        placed = [n for m in msgs for n in m.signals]
        assert sorted(placed) == sorted(s.name for s in sigs)

    def test_identifiers_are_unique(self):
        sigs = [dp.BmsSignal(f"s{i}", 32, rate_hz=float(1 + i % 3))
                for i in range(12)]
        msgs = dp.pack_signals(sigs)
        assert len({m.can_id for m in msgs}) == len(msgs)

    def test_a_signal_too_wide_for_a_frame_is_rejected(self):
        with pytest.raises(ValueError):
            dp.pack_signals([dp.BmsSignal("huge", 96)])


# ===================================================================== #
#  9.  THE PLAN AND ITS VERDICT
# ===================================================================== #
class TestPlanVerdict:
    def test_fully_specified_clean_plan_is_ready(self):
        ss = [_complete_sensor(key=f"s{i}", sample_rate_hz=200.0)
              for i in range(3)]
        p = dp.plan(ss, bus=dp.BusSpec(), rails=dp.default_rails(),
                    logger=dp.LoggerSpec())
        assert p.verdict == dp.Verdict.READY
        assert p.completeness == 1.0

    def test_one_unanswered_question_prevents_ready(self):
        ss = [_complete_sensor(key="s0"),
              _complete_sensor(key="s1", calibration=None)]
        p = dp.plan(ss, bus=dp.BusSpec())
        assert p.verdict == dp.Verdict.INCOMPLETE
        assert "s1" in p.open_questions

    def test_a_hard_failure_blocks_regardless_of_documentation(self):
        """Complete paperwork over a Nyquist violation is still blocked."""
        ss = [_complete_sensor(key="s0", signal_bandwidth_hz=100.0,
                               sample_rate_hz=50.0, antialias_cutoff_hz=20.0)]
        p = dp.plan(ss, bus=dp.BusSpec())
        assert p.verdict == dp.Verdict.BLOCKED
        assert p.blocking()

    def test_undeclared_rate_makes_the_bus_figure_a_floor(self):
        ss = [_complete_sensor(key="s0", sample_rate_hz=100.0),
              _complete_sensor(key="s1", sample_rate_hz=None)]
        p = dp.plan(ss, bus=dp.BusSpec())
        assert p.bus_result.is_floor
        assert "bus-budget-floor" in _checks(p.findings)

    def test_a_bus_native_sensor_adds_no_new_message(self):
        ss = [dp.CATALOG["inverter_temp"]]
        p = dp.plan(ss, bus=dp.BusSpec())
        assert p.bus_result is None or p.bus_result.messages == 0

    def test_bridge_messages_are_counted_against_the_same_bus(self):
        link = dp.UartLink(baud=115200, frame_bytes=64, frame_rate_hz=10.0)
        sigs = [dp.BmsSignal(f"s{i}", 16, rate_hz=10.0) for i in range(8)]
        bridge = dp.plan_bms_bridge(link, sigs, isolated=True, bus=dp.BusSpec())
        without = dp.plan([_complete_sensor()], bus=dp.BusSpec())
        with_bridge = dp.plan([_complete_sensor()], bus=dp.BusSpec(),
                              bridge=bridge)
        assert with_bridge.bus_result.load > without.bus_result.load

    def test_a_refused_bridge_contributes_its_finding_but_no_frames(self):
        link = dp.UartLink(frame_bytes=64, frame_rate_hz=10.0)
        bridge = dp.plan_bms_bridge(link, [])
        p = dp.plan([_complete_sensor()], bus=dp.BusSpec(), bridge=bridge)
        assert "bms-signals-unknown" in _checks(p.findings)
        assert p.verdict != dp.Verdict.READY

    def test_findings_route_to_the_subteams_that_must_act(self):
        ss = [_complete_sensor(location="upright", galvanic_isolation=None,
                               owner="")]
        p = dp.plan(ss, bus=dp.BusSpec())
        acts = p.subteam_actions()
        assert "suspension" in acts
        # clean informational findings are not chores
        assert all(f.severity not in (Severity.OK, Severity.INFO)
                   for fs in acts.values() for f in fs)

    def test_empty_plan_does_not_claim_readiness(self):
        p = dp.plan([], bus=dp.BusSpec())
        assert p.verdict != dp.Verdict.READY


# ===================================================================== #
#  10. EXPORTS
# ===================================================================== #
class TestExports:
    def test_markdown_marks_blanks_so_they_cannot_be_skimmed_past(self):
        p = dp.plan([_complete_sensor(connector=None)], bus=dp.BusSpec())
        md = p.to_markdown()
        assert "**—**" in md
        assert "not been answered" in md

    def test_markdown_states_the_verdict_and_lists_open_questions(self):
        p = dp.plan([_complete_sensor(calibration=None)], bus=dp.BusSpec())
        md = p.to_markdown()
        assert "INCOMPLETE" in md
        assert "Open questions" in md

    def test_markdown_routes_work_by_subteam(self):
        p = dp.plan([_complete_sensor(location="upright", owner="")],
                    bus=dp.BusSpec())
        assert "Routed to" in p.to_markdown()

    def test_csv_has_a_row_per_sensor_and_a_completeness_column(self):
        ss = [_complete_sensor(key="a"), _complete_sensor(key="b")]
        rows = dp.plan(ss, bus=dp.BusSpec()).to_csv().strip().splitlines()
        assert len(rows) == 3
        assert "completeness" in rows[0]

    def test_csv_leaves_unanswered_fields_empty_not_the_string_none(self):
        p = dp.plan([_complete_sensor(connector=None)], bus=dp.BusSpec())
        assert "None" not in p.to_csv()


# ===================================================================== #
#  11. CATALOG
# ===================================================================== #
class TestCatalog:
    def test_the_meeting_sensor_list_is_pre_specified(self):
        for k in ("motor_temp", "inverter_temp", "coolant_temp_in",
                  "coolant_temp_out", "coolant_flow"):
            assert k in dp.CATALOG

    def test_catalog_entries_leave_car_specific_questions_open(self):
        """The catalog answers the generic questions, not the local ones."""
        s = dp.CATALOG["motor_temp"]
        assert s.connector is None, "connector depends on your harness"
        assert s.owner == "", "ownership is not something a catalog can know"

    def test_overrides_replace_catalog_defaults(self):
        s = dp.catalog("motor_temp", connector="AS 3-way", owner="ana",
                       sample_rate_hz=25.0)
        assert s.connector == "AS 3-way" and s.owner == "ana"
        assert s.sample_rate_hz == 25.0
        assert dp.CATALOG["motor_temp"].connector is None, "catalog not mutated"

    def test_unknown_catalog_key_raises(self):
        with pytest.raises(KeyError):
            dp.catalog("flux_capacitor_temp")

    def test_motor_and_inverter_entries_sit_on_the_tractive_system(self):
        assert dp.CATALOG["motor_temp"].on_tractive_system()
        assert dp.CATALOG["inverter_temp"].on_tractive_system()
        assert not dp.CATALOG["coolant_flow"].on_tractive_system()

    def test_the_cooling_package_triggers_the_delta_t_check(self):
        p = dp.plan(dp.cooling_package(), bus=dp.BusSpec(),
                    expected_delta_t_k=6.0, flow_lpm=12.0)
        assert p.delta_t is not None
        assert p.delta_t.heat_kw is not None

    def test_coolant_pair_detection_needs_two_loop_temperatures(self):
        assert dp.find_coolant_pair([dp.CATALOG["coolant_flow"]]) is None
        pair = dp.find_coolant_pair(dp.cooling_package())
        assert pair is not None
        assert pair[0].key == "coolant_temp_in"
        assert pair[1].key == "coolant_temp_out"


# ===================================================================== #
#  12. PROVENANCE
# ===================================================================== #
def test_provenance_separates_physics_from_estimate():
    assert dp.PROVENANCE["physics_grounded"]
    assert dp.PROVENANCE["estimate_flagged"]
    assert "FLOOR" in dp.PROVENANCE["hard_rule"]


def test_module_imports_without_streamlit_or_numpy():
    """Pure model layer — must stay headless and cheap to import.

    Checked in a FRESH interpreter, not against this session's sys.modules.
    The old form asserted `"streamlit" not in sys.modules` inside the shared
    pytest process, so it only ever passed on machines where streamlit was not
    installed at all — i.e. it silently inverted into a no-op on every real
    developer machine, and hard-failed as soon as any earlier test in the
    session imported streamlit. What we actually care about is whether
    importing this module *pulls streamlit in*, which only a clean process can
    answer.
    """
    import subprocess
    import sys

    probe = (
        "import sys; import suspension.daq_plan; "
        "assert 'streamlit' not in sys.modules, 'daq_plan pulled in streamlit'; "
        "assert 'numpy' not in sys.modules, 'daq_plan pulled in numpy'"
    )
    proc = subprocess.run([sys.executable, "-c", probe],
                          capture_output=True, text=True,
                          cwd=str(pathlib.Path(__file__).resolve().parents[1]))
    assert proc.returncode == 0, proc.stderr.strip()
