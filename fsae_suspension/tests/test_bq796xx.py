# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
Tests for suspension.bq796xx.

The important ones are the anchors. Every frame builder is checked byte-for-
byte against a frame printed in the device documentation, because the failure
mode this module exists to prevent — a wrong CRC seed — produces silence on the
bus rather than an error, and silence is not something a unit test can notice
after the fact. It has to be caught here.

The rest of the suite treats the refusals as contracts: an undeclared cell
count raises rather than defaulting, and a poll rate above the link's ceiling
is a FAIL rather than a suggestion.
"""


import pytest

from suspension.interfaces import Severity
from suspension import bq796xx as bq
from suspension import daq_plan as dp


def _sev(findings, check):
    for f in findings:
        if f.check == check:
            return f.severity
    return None


def _checks(findings):
    return {f.check for f in findings}


# --------------------------------------------------------------------------- #
#  CRC — the anchors
# --------------------------------------------------------------------------- #
def test_every_documented_frame_passes_its_own_crc():
    results = bq.verify_crc_anchors()
    assert len(results) == 24
    failed = [s for s, ok in results if not ok]
    assert not failed, f"documented frames failing CRC: {failed}"


def test_textbook_crc16_ibm_seed_would_reject_everything():
    """The seed the documentation's prose implies rejects the documentation's
    own examples. This test exists so nobody 'corrects' CRC_INIT back."""
    def crc_arc(data):
        crc = 0x0000
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return crc

    for anchor in bq.CRC_ANCHORS:
        frame = bytes.fromhex(anchor)
        body, rx = frame[:-2], frame[-2:]
        arc = crc_arc(body)
        assert bytes((arc & 0xFF, arc >> 8)) != rx


def test_crc_is_little_endian_on_the_wire():
    body = bytes.fromhex("D0 03 4C 00")
    assert bq.crc16(body) == 0x24FC
    assert bq.crc_bytes(body) == bytes.fromhex("FC 24")


def test_check_crc_rejects_a_corrupted_frame():
    good = bytearray(bytes.fromhex("D0 03 4C 00 FC 24"))
    assert bq.check_crc(bytes(good))
    good[2] ^= 0x01
    assert not bq.check_crc(bytes(good))


# --------------------------------------------------------------------------- #
#  Frame builders reproduce the documented bytes
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("reg,data,expected", [
    (0x034C, 0x00, "D0 03 4C 00 FC 24"),
    (0x0309, 0x01, "D0 03 09 01 0F 74"),
    (0x0306, 0x00, "D0 03 06 00 CB 44"),
    (0x0306, 0x01, "D0 03 06 01 0A 84"),
    (0x0306, 0x02, "D0 03 06 02 4A 85"),
    (0x0308, 0x02, "D0 03 08 02 4E E5"),
    (0x0003, 0x0A, "D0 00 03 0A B8 13"),
    (0x030D, 0x06, "D0 03 0D 06 4C 76"),
])
def test_broadcast_write_matches_documentation(reg, data, expected):
    assert bq.build_write(reg, data).hex(" ").upper() == expected


@pytest.mark.parametrize("dev,reg,data,expected", [
    (0, 0x0308, 0x00, "90 00 03 08 00 13 DD"),
    (2, 0x0308, 0x03, "90 02 03 08 03 52 64"),
])
def test_single_device_write_matches_documentation(dev, reg, data, expected):
    got = bq.build_write(reg, data, cmd=bq.Cmd.SINGLE_WRITE, device=dev)
    assert got.hex(" ").upper() == expected


@pytest.mark.parametrize("reg,n,expected", [
    (0x034C, 1, "C0 03 4C 00 F8 E4"),
    (0x0568, 32, "C0 05 68 1F 42 2D"),
])
def test_broadcast_read_matches_documentation(reg, n, expected):
    assert bq.build_read(reg, n).hex(" ").upper() == expected


def test_read_length_byte_is_n_minus_one():
    """The off-by-one belongs to the device. Getting it wrong returns one byte
    short of the block, which decodes as a plausible wrong voltage."""
    assert bq.build_read(0x0568, 32)[3] == 31
    assert bq.build_read(0x0568, 1)[3] == 0


def test_frame_limits_are_enforced():
    with pytest.raises(ValueError):
        bq.build_read(0x0568, bq.MAX_READ_BYTES + 1)
    with pytest.raises(ValueError):
        bq.build_read(0x0568, 0)
    with pytest.raises(ValueError):
        bq.build_write(0x0568, b"\x00" * (bq.MAX_WRITE_BYTES + 1))
    with pytest.raises(ValueError):
        bq.build_write(0x0308, 0x00, cmd=bq.Cmd.SINGLE_WRITE)   # no device


def test_generated_addressing_sequence_reproduces_the_documented_transcript():
    seq = bq.auto_address_sequence(3)
    frames = [f.hex(" ").upper() for f, _ in seq]
    for expected in ("D0 03 4C 00 FC 24", "D0 03 09 01 0F 74",
                     "D0 03 06 00 CB 44", "D0 03 06 01 0A 84",
                     "D0 03 06 02 4A 85", "D0 03 08 02 4E E5",
                     "90 00 03 08 00 13 DD", "90 02 03 08 03 52 64",
                     "C0 03 4C 00 F8 E4"):
        assert expected in frames


def test_addressing_sequence_scales_with_board_count():
    assert len(bq.auto_address_sequence(6)) == len(bq.auto_address_sequence(3)) + 3


# --------------------------------------------------------------------------- #
#  Scaling
# --------------------------------------------------------------------------- #
def test_cell_lsb_matches_documentation():
    assert bq.CELL_LSB_V == pytest.approx(190.73e-6)


def test_raw_to_volts_is_twos_complement():
    assert bq.raw_to_volts(0x0000) == 0.0
    assert bq.raw_to_volts(0x0001) == pytest.approx(190.73e-6)
    assert bq.raw_to_volts(0xFFFF) == pytest.approx(-190.73e-6)
    assert bq.raw_to_volts(0x8000) < 0        # sign bit really is a sign bit


def test_volts_round_trip_within_one_lsb():
    for v in (2.5, 3.0, 3.7, 4.2):
        assert bq.raw_to_volts(bq.volts_to_raw(v)) == pytest.approx(v, abs=bq.CELL_LSB_V)


def test_decode_reverses_the_block_order():
    """The block reads highest cell first; everything downstream counts from
    cell 1. A silent reversal here presents as a connector wiring error."""
    payload = bytearray()
    for v in (4.0, 3.9, 3.8, 3.7):        # cell 4, 3, 2, 1 as transmitted
        raw = bq.volts_to_raw(v)
        payload += bytes((raw >> 8, raw & 0xFF))
    out = bq.decode_cell_block(bytes(payload), 4)
    assert out[0] == pytest.approx(3.7, abs=1e-4)     # cell 1
    assert out[3] == pytest.approx(4.0, abs=1e-4)     # cell 4


def test_decode_refuses_a_short_payload():
    with pytest.raises(ValueError):
        bq.decode_cell_block(b"\x00" * 10, 16)


# --------------------------------------------------------------------------- #
#  Timing
# --------------------------------------------------------------------------- #
def test_wake_time_scales_with_device_count():
    assert bq.wake_time_s(1) == pytest.approx(2.5e-3 + 10e-3 + 600e-6)
    assert bq.wake_time_s(6) > bq.wake_time_s(1)
    assert (bq.wake_time_s(6) - bq.wake_time_s(1)) == pytest.approx(5 * 600e-6)


def test_adc_settle_scales_with_boards():
    assert bq.adc_settle_s(1) == pytest.approx(192e-6 + 5e-6)
    assert bq.adc_settle_s(35) == pytest.approx(192e-6 + 175e-6)


# --------------------------------------------------------------------------- #
#  Poll budget — the ceiling nobody computes
# --------------------------------------------------------------------------- #
def test_response_is_unknown_until_the_overhead_is_measured():
    """The response length is not a number this module is entitled to state
    before someone has counted bytes on a bench."""
    s = bq.StackSpec(boards=6, cells_per_board=16)
    assert s.response_bytes() is None
    lo, hi = s.response_bounds()
    assert lo < hi
    assert s.payload_bytes() == 192          # this part IS known exactly


def test_measuring_the_overhead_collapses_the_range_to_a_number():
    s = bq.StackSpec(boards=6, cells_per_board=16, response_overhead_bytes=6)
    assert s.response_bytes() == 228
    assert s.response_bounds() == (228, 228)


def test_response_grows_linearly_with_boards_once_measured():
    one = bq.StackSpec(boards=1, cells_per_board=16,
                       response_overhead_bytes=6).response_bytes()
    six = bq.StackSpec(boards=6, cells_per_board=16,
                       response_overhead_bytes=6).response_bytes()
    assert six == 6 * one


def test_overhead_can_be_derived_from_a_byte_count():
    assert bq.measure_response_overhead(228, cells_per_board=16, boards=6) == 6
    assert bq.measure_response_overhead(38, cells_per_board=16, boards=1) == 6


def test_overhead_derivation_refuses_impossible_counts():
    with pytest.raises(ValueError):
        bq.measure_response_overhead(100, cells_per_board=16, boards=6)
    with pytest.raises(ValueError):
        bq.measure_response_overhead(229, cells_per_board=16, boards=6)


def test_one_board_clears_100hz_and_six_boards_do_not_at_115200():
    link = dp.UartLink(baud=115200)
    small = bq.poll_budget(bq.StackSpec(boards=1, cells_per_board=16,
                                        response_overhead_bytes=6), link,
                           requested_rate_hz=100.0)
    big = bq.poll_budget(bq.StackSpec(boards=6, cells_per_board=16,
                                      response_overhead_bytes=6), link,
                         requested_rate_hz=100.0)
    assert small.max_rate_hz > 100.0
    assert big.max_rate_hz < 100.0
    assert _sev(small.findings, "bq-poll-ok") == Severity.OK
    assert _sev(big.findings, "bq-poll-infeasible") == Severity.FAIL


def test_infeasible_poll_names_the_baud_rate_that_would_work():
    stack = bq.StackSpec(boards=6, cells_per_board=16,
                         response_overhead_bytes=6)
    big = bq.poll_budget(stack, dp.UartLink(baud=115200),
                         requested_rate_hz=100.0)
    f = [x for x in big.findings if x.check == "bq-poll-infeasible"][0]
    needed = f.detail["baud_needed"]
    faster = bq.poll_budget(stack, dp.UartLink(baud=int(needed * 1.05)),
                            requested_rate_hz=100.0)
    assert faster.max_rate_hz >= 100.0


def test_poll_budget_refuses_to_guess_the_cell_count():
    with pytest.raises(ValueError):
        bq.poll_budget(bq.StackSpec(boards=6))


def test_tight_poll_rate_warns_about_having_no_room_to_retry():
    stack = bq.StackSpec(boards=6, cells_per_board=16,
                         response_overhead_bytes=6)
    ceiling = bq.poll_budget(stack, dp.UartLink(baud=115200)).max_rate_hz
    tight = bq.poll_budget(stack, dp.UartLink(baud=115200),
                           requested_rate_hz=ceiling * 0.9)
    assert _sev(tight.findings, "bq-poll-tight") == Severity.WARN


# --------------------------------------------------------------------------- #
#  Stack checks
# --------------------------------------------------------------------------- #
def test_standalone_part_cannot_be_stacked():
    f = bq.stack_findings(bq.StackSpec(part="BQ75614-Q1", boards=4,
                                       cells_per_board=16))
    assert _sev(f, "bq-stack-too-deep") == Severity.FAIL


def test_stackable_part_is_limited_to_its_documented_depth():
    ok = bq.stack_findings(bq.StackSpec(part="BQ79616-Q1", boards=35,
                                        cells_per_board=16))
    over = bq.stack_findings(bq.StackSpec(part="BQ79616-Q1", boards=36,
                                          cells_per_board=16))
    assert "bq-stack-too-deep" not in _checks(ok)
    assert _sev(over, "bq-stack-too-deep") == Severity.FAIL


def test_undeclared_isolation_is_missing_and_declared_false_is_fail():
    unknown = bq.stack_findings(bq.StackSpec(cells_per_board=16))
    explicit = bq.stack_findings(bq.StackSpec(cells_per_board=16,
                                              isolated=False))
    assert _sev(unknown, "bq-isolation") == Severity.MISSING
    assert _sev(explicit, "bq-isolation") == Severity.FAIL


def test_undeclared_cell_count_is_missing_not_defaulted():
    f = bq.stack_findings(bq.StackSpec(boards=6))
    assert _sev(f, "bq-cells-undeclared") == Severity.MISSING


def test_nfault_on_an_interrupt_is_ok_and_polled_is_not():
    polled = bq.stack_findings(bq.StackSpec(cells_per_board=16,
                                            nfault_to_interrupt=False))
    wired = bq.stack_findings(bq.StackSpec(cells_per_board=16,
                                           nfault_to_interrupt=True))
    assert _sev(polled, "bq-nfault-polled") == Severity.WARN
    assert _sev(wired, "bq-nfault-interrupt") == Severity.OK


def test_parts_without_current_sense_say_so():
    no_cs = bq.stack_findings(bq.StackSpec(part="BQ79616-Q1", cells_per_board=16))
    with_cs = bq.stack_findings(bq.StackSpec(part="BQ75614-Q1", cells_per_board=16))
    assert "bq-no-current-sense" in _checks(no_cs)
    assert "bq-no-current-sense" not in _checks(with_cs)


def test_resolution_is_not_judged_without_a_requirement():
    assert _sev(bq.resolution_findings(bq.StackSpec()),
                "bq-resolution-unspecified") == Severity.MISSING
    assert _sev(bq.resolution_findings(bq.StackSpec(),
                                       resolution_needed_v=0.001),
                "bq-resolution-ok") == Severity.OK


# --------------------------------------------------------------------------- #
#  Handoff to daq_plan
# --------------------------------------------------------------------------- #
def test_signal_list_turns_the_bridge_refusal_into_a_plan():
    """The refusal was never about the tool guessing — it was about nobody
    having read the datasheet. This module is the datasheet, read."""
    refused = dp.plan_bms_bridge(dp.UartLink(baud=115200), [])
    assert refused.refused

    stack = bq.StackSpec(boards=2, cells_per_board=16,
                         thermistors_per_board=4, isolated=True,
                         response_overhead_bytes=6)
    sigs = bq.to_bms_signals(stack)
    link = dp.UartLink(baud=1_000_000, frame_bytes=stack.response_bytes(),
                       frame_rate_hz=10.0)
    plan = dp.plan_bms_bridge(link, sigs, isolated=True)
    assert not plan.refused
    assert plan.messages


def test_signal_count_matches_the_declared_pack():
    stack = bq.StackSpec(boards=3, cells_per_board=12,
                         thermistors_per_board=2)
    sigs = bq.to_bms_signals(stack)
    names = [s.name for s in sigs]
    assert sum(1 for n in names if n.startswith("cell_v_")) == 36
    assert sum(1 for n in names if n.startswith("pack_temp_")) == 6


def test_signal_list_refuses_without_a_cell_count():
    with pytest.raises(ValueError):
        bq.to_bms_signals(bq.StackSpec(boards=3))


def test_cell_voltages_pack_four_to_a_frame_with_none_straddling():
    stack = bq.StackSpec(boards=1, cells_per_board=16)
    msgs = bq.cell_voltage_can_map(stack)
    assert len(msgs) == 4
    assert all(m.dlc == 8 for m in msgs)
    assert sum(len(m.signals) for m in msgs) == 16
    assert len({m.can_id for m in msgs}) == 4        # no identifier collisions


def test_odd_cell_count_gives_a_short_final_frame_not_a_wrong_one():
    msgs = bq.cell_voltage_can_map(bq.StackSpec(boards=1, cells_per_board=14))
    assert sum(len(m.signals) for m in msgs) == 14
    assert msgs[-1].dlc == 4


def test_sensor_specs_route_to_the_isolation_boundary():
    stack = bq.StackSpec(boards=2, cells_per_board=16, isolated=True)
    specs = bq.to_sensor_specs(stack)
    assert specs
    assert all(s.on_tractive_system() for s in specs)
    assert all("electrics" in s.affected_subteams() for s in specs)


# --------------------------------------------------------------------------- #
#  Whole-plan contract
# --------------------------------------------------------------------------- #
def test_fully_declared_stack_has_no_open_questions():
    stack = bq.StackSpec(part="BQ79616-Q1", boards=4, cells_per_board=16,
                         thermistors_per_board=4, isolated=True,
                         nfault_to_interrupt=True, balancing=True,
                         response_overhead_bytes=6)
    p = bq.plan_stack(stack, link=dp.UartLink(baud=1_000_000),
                      requested_rate_hz=10.0, resolution_needed_v=0.001)
    assert not p.blocking()
    assert not p.open_questions()


def test_empty_stack_declaration_produces_open_questions_not_numbers():
    p = bq.plan_stack(bq.StackSpec())
    assert p.open_questions()
    assert p.budget is None
    assert p.messages == []


def test_markdown_renders():
    p = bq.plan_stack(bq.StackSpec(boards=2, cells_per_board=16, isolated=True),
                      requested_rate_hz=10.0)
    md = p.to_markdown()
    assert "BQ79616-Q1" in md
    assert "Poll ceiling" in md


def test_provenance_records_the_measurement_not_an_assumption():
    joined = " ".join(bq.PROVENANCE["measured_not_assumed"])
    assert "measure_response_overhead" in joined
    assert bq.PROVENANCE["corrected_against_documentation"]
    assert any("SLVAE86B" in s for s in bq.PROVENANCE["sources"])


# --------------------------------------------------------------------------- #
#  Regressions for bugs found against TI SLVAE86B
# --------------------------------------------------------------------------- #
def test_init_byte_encodes_the_payload_length():
    """0xD0 | (n-1). Sending 0xD0 with eight data bytes is a well-formed frame
    with a valid CRC that asks for something else entirely — it fails quietly,
    which is why this needs a test rather than a comment."""
    assert bq.build_write(0x034C, 0x00)[0] == 0xD0
    assert bq.build_write(0x0318, b"\x02" * 8)[0] == 0xD7
    assert bq.build_write(0x0100, b"\x02\xB7\x78\xBC")[0] == 0xD3
    assert bq.build_write(0x0100, b"\x02\xB7\x78\xBC",
                          cmd=bq.Cmd.SINGLE_WRITE, device=0)[0] == 0x93
    assert bq.build_write(0x0100, b"\x02\xB7\x78\xBC",
                          cmd=bq.Cmd.STACK_WRITE)[0] == 0xB3


def test_multibyte_write_matches_the_published_balancing_frame():
    got = bq.build_write(0x0318, b"\x02" * 8).hex(" ").upper()
    assert got == "D7 03 18 02 02 02 02 02 02 02 02 14 BE"


def test_balancing_sequence_reproduces_ti_section_6_2():
    frames = [f.hex(" ").upper() for f, _ in bq.balancing_sequence(16)]
    for expected in ("D0 00 03 0A B8 13",
                     "D7 03 18 02 02 02 02 02 02 02 02 14 BE",
                     "D7 03 20 02 02 02 02 02 02 02 02 27 7F",
                     "D0 03 2E 01 14 84",
                     "D0 03 2A 08 D6 42",
                     "D0 03 2C 05 14 27",
                     "D0 03 2F 03 94 D5"):
        assert expected in frames


def test_single_board_is_told_it_is_base_and_top_in_one_write():
    """One board is both ends of the stack. Running the multi-board sequence
    against it leaves a base with no top of stack, which answers nothing and
    looks exactly like a wiring fault."""
    seq = bq.auto_address_sequence(1)
    comments = " ".join(c for _, c in seq)
    assert "base AND top" in comments
    # exactly one COMM_CTRL single-device write, carrying 0x01
    singles = [f for f, _ in seq if f[0] & 0xF0 == 0x90]
    assert len(singles) == 1
    assert singles[0][4] == bq.COMM_CTRL_BASE_AND_TOP


def test_multi_board_still_uses_two_comm_ctrl_writes():
    seq = bq.auto_address_sequence(3)
    singles = [f for f, _ in seq if f[0] & 0xF0 == 0x90]
    assert [f[4] for f in singles] == [bq.COMM_CTRL_BASE, bq.COMM_CTRL_TOP]


def test_reverse_addressing_matches_ti_section_9_2():
    frames = [f.hex(" ").upper() for f, _ in bq.reverse_direction_sequence(3)]
    for expected in ("90 00 03 09 80 13 ED", "E0 03 09 80 C0 14",
                     "D0 03 09 81 0E D4", "D0 03 07 00 CA D4",
                     "D0 03 07 01 0B 14", "D0 03 07 02 4B 15"):
        assert expected in frames


# --------------------------------------------------------------------------- #
#  The verdict is withheld when an unmeasured constant would decide it
# --------------------------------------------------------------------------- #
def test_rate_that_depends_on_the_unknown_is_indeterminate_not_guessed():
    stack = bq.StackSpec(boards=6, cells_per_board=16)      # unmeasured
    b = bq.poll_budget(stack, dp.UartLink(baud=115200))
    # a rate between the two ceilings: the answer depends entirely on framing
    between = (b.rate_ceiling_min_hz + b.rate_ceiling_max_hz) / 2
    p = bq.poll_budget(stack, dp.UartLink(baud=115200),
                       requested_rate_hz=between)
    assert _sev(p.findings, "bq-poll-indeterminate") == Severity.MISSING
    assert "bq-poll-ok" not in _checks(p.findings)
    assert "bq-poll-infeasible" not in _checks(p.findings)


def test_clearly_infeasible_still_fails_without_a_measurement():
    """If it does not fit even at the most generous overhead, the unknown
    cannot rescue it and the FAIL is honest."""
    p = bq.poll_budget(bq.StackSpec(boards=6, cells_per_board=16),
                       dp.UartLink(baud=115200), requested_rate_hz=100.0)
    assert _sev(p.findings, "bq-poll-infeasible") == Severity.FAIL


def test_clearly_feasible_passes_without_a_measurement():
    p = bq.poll_budget(bq.StackSpec(boards=6, cells_per_board=16),
                       dp.UartLink(baud=115200), requested_rate_hz=5.0)
    assert _sev(p.findings, "bq-poll-ok") == Severity.OK


def test_unmeasured_overhead_is_reported_as_an_open_question():
    p = bq.poll_budget(bq.StackSpec(boards=2, cells_per_board=16),
                       dp.UartLink(baud=115200))
    assert _sev(p.findings, "bq-response-overhead-unmeasured") == Severity.MISSING
    assert p.response_bytes is None


def test_measured_overhead_closes_that_question():
    p = bq.poll_budget(bq.StackSpec(boards=2, cells_per_board=16,
                                    response_overhead_bytes=6),
                       dp.UartLink(baud=115200))
    assert "bq-response-overhead-unmeasured" not in _checks(p.findings)
    assert p.response_bytes == 76
