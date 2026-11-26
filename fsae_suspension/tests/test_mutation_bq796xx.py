"""suspension/bq796xx.py planning layer against tests/golden: five stacks chosen so every check, verdict and
hand-off branch fires at least once (findings with their severity, subsystems and detail, the signal list, the CAN
map, the bring-up frames, the daq_plan channels and the Markdown), plus the boundaries of each verdict. Regenerate
the golden file with KINEMATIK_REGOLD=1 only after reading the diff."""
import dataclasses
import enum
import json
import os
from pathlib import Path

import pytest

from suspension import bq796xx as bq
from suspension.daq_plan import UartLink

GOLD = Path(__file__).parent / "golden" / "bq796xx_plans.json"

STACKS = {
    "full_measured": (bq.StackSpec("BQ79616-Q1", 6, 16, True, True, True, 8, 6),
                      dict(requested_rate_hz=10.0, resolution_needed_v=1e-3)),
    "open_questions": (bq.StackSpec("BQ79616-Q1", 4, 12), dict(requested_rate_hz=None)),
    "indeterminate": (bq.StackSpec("BQ79616-Q1", 30, 16, False, None, False, 3),
                      dict(requested_rate_hz=None, link=UartLink(baud=250_000))),
    "wrong_part": (bq.StackSpec("BQ75614-Q1", 2, 10, None, False, False, 9),
                   dict(requested_rate_hz=500.0, resolution_needed_v=1e-4)),
    "standalone_tight": (bq.StackSpec("BQ79656-Q1", 1, 14, True, True, True, 0, 4),
                         dict(requested_rate_hz=None, continuous_adc=False, base_id=0x400)),
    "no_cells": (bq.StackSpec("BQ79616-Q1", 3), dict()),
}


def _plain(x):
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: _plain(getattr(x, f.name)) for f in dataclasses.fields(x)}
    if isinstance(x, enum.Enum):
        return x.value
    if isinstance(x, bytes):
        return x.hex()
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    if isinstance(x, float):
        return float(f"{x:.12g}")
    return x


def _tight_rate(stack):
    b = bq.poll_budget(stack, continuous_adc=False)
    return round(0.9 * b.rate_ceiling_min_hz, 3)


def _everything():
    out = {}
    for name, (stack, kw) in STACKS.items():
        kw = dict(kw)
        if name == "standalone_tight":
            kw["requested_rate_hz"] = _tight_rate(stack)
        if name == "indeterminate":
            b = bq.poll_budget(stack, kw["link"])
            kw["requested_rate_hz"] = round(0.5 * (b.rate_ceiling_min_hz + b.rate_ceiling_max_hz), 3)
        plan = bq.plan_stack(stack, **kw)
        out[name] = {"plan": _plain(dataclasses.replace(plan, stack=None, device=None)),
                     "markdown": plan.to_markdown().split("\n"),
                     "blocking": [f.check for f in plan.blocking()],
                     "open": [f.check for f in plan.open_questions()],
                     "sensors": _plain(bq.to_sensor_specs(stack)),
                     "headroom": plan.budget.headroom() if plan.budget else None}
    return out


def test_every_plan_word_for_word():
    got = json.loads(json.dumps(_everything()))
    if os.environ.get("KINEMATIK_REGOLD"):
        GOLD.write_text(json.dumps(got, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    want = json.loads(GOLD.read_text(encoding="utf-8"))
    for name in STACKS:
        for part in want[name]:
            assert got[name][part] == want[name][part], (name, part)


def test_the_branches_the_scenarios_are_meant_to_hit():
    g = json.loads(GOLD.read_text(encoding="utf-8"))
    checks = {n: [f["check"] for f in g[n]["plan"]["findings"]] for n in g}
    assert "bq-poll-ok" in checks["full_measured"] and "bq-poll-ceiling" in checks["full_measured"]
    assert "bq-poll-indeterminate" in checks["indeterminate"] and "bq-thermistors-undeclared" not in checks[
        "indeterminate"]
    assert {"bq-stack-too-deep", "bq-cells-out-of-range", "bq-thermistors-over", "bq-poll-infeasible",
            "bq-resolution-insufficient"} <= set(checks["wrong_part"])
    assert {"bq-standalone", "bq-poll-tight"} <= set(checks["standalone_tight"])
    assert {"bq-cells-undeclared", "bq-isolation", "bq-nfault-polled", "bq-thermistors-undeclared",
            "bq-balancing-undeclared", "bq-resolution-unspecified"} <= set(checks["no_cells"])
    assert g["no_cells"]["plan"]["budget"] is None and g["no_cells"]["sensors"] == []


def test_the_poll_verdict_boundaries():
    s = bq.StackSpec("BQ79616-Q1", 4, 16, response_overhead_bytes=6)
    b = bq.poll_budget(s)
    hi = b.rate_ceiling_max_hz

    def last(rate):
        return bq.poll_budget(s, requested_rate_hz=rate).findings[-1].check
    assert last(hi) == "bq-poll-tight" and last(hi * (1 + 1e-9)) == "bq-poll-infeasible"
    assert last(0.8 * hi) == "bq-poll-ok" and last(0.8 * hi * (1 + 1e-9)) == "bq-poll-tight"
    u = bq.StackSpec("BQ79616-Q1", 4, 16)
    ub = bq.poll_budget(u)
    assert bq.poll_budget(u, requested_rate_hz=ub.rate_ceiling_min_hz).findings[-1].check == "bq-poll-tight"
    assert bq.poll_budget(u, requested_rate_hz=ub.rate_ceiling_min_hz * (1 + 1e-9)).findings[-1].check == \
        "bq-poll-indeterminate"
    assert bq.poll_budget(u, requested_rate_hz=ub.rate_ceiling_max_hz).findings[-1].check == "bq-poll-indeterminate"
    ok = bq.poll_budget(u, requested_rate_hz=1.0)
    assert ok.findings[-1].message.endswith(" even at the worst plausible framing overhead, with room for retries.")
    assert ok.headroom() == 1.0 / ok.rate_ceiling_min_hz and bq.poll_budget(u).headroom() is None
    assert (ok.max_rate_hz, ok.response_bytes, ok.cycle_s) == (ok.rate_ceiling_min_hz, None, None)
    m = bq.poll_budget(s, requested_rate_hz=1.0)
    assert (m.response_bytes, m.cycle_s) == (m.response_min, m.cycle_min_s)
    assert m.findings[-1].message.endswith("Hz ceiling, with room for retries.")
    slow = bq.poll_budget(s, UartLink(usable_fraction=0.0))
    assert slow.cycle_min_s == pytest.approx((b.request_bytes + b.response_min) * UartLink().byte_time_s() / 1e-6
                                             + 500e-6, rel=1e-12)
    fast = bq.poll_budget(s, UartLink(usable_fraction=2.0))
    assert fast.cycle_min_s == bq.poll_budget(s, UartLink(usable_fraction=1.0)).cycle_min_s < b.cycle_min_s
    with pytest.raises(ValueError, match=r"^cells_per_board is not declared, so the response length is unknown and "
                                         r"the poll rate cannot be computed\. This is a real unanswered question, "
                                         r"not a default to fill in\.$"):
        bq.poll_budget(bq.StackSpec())


def test_the_stack_check_boundaries():
    def checks(**kw):
        return [f.check for f in bq.stack_findings(bq.StackSpec(**kw))]
    assert "bq-stack-too-deep" not in checks(boards=35, cells_per_board=6)
    assert "bq-stack-too-deep" in checks(boards=36, cells_per_board=6)
    assert "bq-cells-out-of-range" not in checks(cells_per_board=6) + checks(cells_per_board=16)
    assert "bq-cells-out-of-range" in checks(cells_per_board=5) and "bq-cells-out-of-range" in checks(
        cells_per_board=17)
    assert "bq-thermistors-over" not in checks(thermistors_per_board=8)
    assert "bq-thermistors-over" in checks(thermistors_per_board=9)
    assert "bq-standalone" not in checks(part="BQ75614-Q1", boards=2)
    assert "bq-balancing-undeclared" not in checks(balancing=False)
    assert "bq-isolation" not in checks(isolated=True)
    sev = {f.check: f.severity for f in bq.stack_findings(bq.StackSpec(isolated=False, nfault_to_interrupt=False))}
    assert (sev["bq-isolation"], sev["bq-nfault-polled"]) == (bq.Severity.FAIL, bq.Severity.WARN)
    assert bq.resolution_findings(bq.StackSpec(), resolution_needed_v=bq.CELL_LSB_V)[0].check == "bq-resolution-ok"
    assert bq.resolution_findings(bq.StackSpec(), resolution_needed_v=bq.CELL_LSB_V * (1 - 1e-12))[0].check == \
        "bq-resolution-insufficient"


def test_the_hand_off_lists():
    s = bq.StackSpec("BQ79616-Q1", 2, 7, thermistors_per_board=2)
    sig = bq.to_bms_signals(s, cell_rate_hz=20.0, temp_rate_hz=3.0, status_rate_hz=5.0)
    assert [x.name for x in sig] == ([f"cell_v_b{b}_c{c}" for b in range(2) for c in range(1, 8)]
                                     + [f"pack_temp_b{b}_t{t}" for b in range(2) for t in (1, 2)]
                                     + ["nfault_status", "balance_active"])
    assert {x.rate_hz for x in sig[:14]} == {20.0} and {x.rate_hz for x in sig[14:18]} == {3.0}
    assert (sig[-2].rate_hz, sig[-1].rate_hz) == (5.0, 1.0)
    assert [x.name for x in bq.to_bms_signals(s, include_current=True)][-3] == "pack_current"
    assert "pack_current" in [x.name for x in bq.to_bms_signals(bq.StackSpec("BQ75614-Q1", 1, 14))]
    assert "pack_current" not in [x.name for x in bq.to_bms_signals(bq.StackSpec("BQ75614-Q1", 1, 14),
                                                                     include_current=False)]
    d = dataclasses.asdict(bq.to_bms_signals(s)[0])
    assert (d["bits"], d["unit"], d["scale"], d["rate_hz"], d["critical"]) == (16, "V", bq.CELL_LSB_V, 10.0, True)
    with pytest.raises(ValueError, match=r"^cannot derive a signal list without cells_per_board — the number of "
                                         r"cell-voltage signals IS the cell count$"):
        bq.to_bms_signals(bq.StackSpec())
    msgs = bq.cell_voltage_can_map(s, base_id=0x10, rate_hz=7.0, extended=True)
    assert [(m.name, m.can_id, m.dlc, m.signals[0], m.signals[-1]) for m in msgs] == [
        ("BMS_CELL_V_01_04", 0x10, 8, "cell_v_1", "cell_v_4"), ("BMS_CELL_V_05_08", 0x11, 8, "cell_v_5", "cell_v_8"),
        ("BMS_CELL_V_09_12", 0x12, 8, "cell_v_9", "cell_v_12"), ("BMS_CELL_V_13_14", 0x13, 4, "cell_v_13",
                                                                  "cell_v_14")]
    assert all(m.extended and m.rate_hz == 7.0 and m.producer == "bq796xx_bridge" for m in msgs)
    d = bq.cell_voltage_can_map(s)[0]
    assert (d.can_id, d.rate_hz, d.extended) == (0x300, 10.0, False)
    with pytest.raises(ValueError, match=r"^cells_per_board is not declared$"):
        bq.cell_voltage_can_map(bq.StackSpec())
    sp = bq.to_sensor_specs(s, cell_rate_hz=25.0)
    assert sp[0].sample_rate_hz == 25.0 and bq.to_sensor_specs(s)[0].sample_rate_hz == 10.0
    assert [x.key for x in bq.to_sensor_specs(bq.StackSpec("BQ79616-Q1", 2, 7))] == ["bms_cell_voltages"]


def test_a_stackable_part_too_deep_and_a_message_with_table_breaks():
    (f,) = [x for x in bq.stack_findings(bq.StackSpec(boards=36, cells_per_board=6)) if x.check == "bq-stack-too-deep"]
    assert f.message == "36 boards of BQ79616-Q1, which supports 35."
    plan = bq.plan_stack(bq.StackSpec(boards=2, cells_per_board=6, isolated=True, nfault_to_interrupt=True,
                                      thermistors_per_board=1, balancing=True))
    plan.findings = [bq.Finding("x-check", bq.Severity.WARN, "a | b\nc")]
    assert plan.to_markdown().split("\n")[-1] == "| WARNING | x-check | a / b c |"


def _seq(s):
    return [[f.hex(), c] for f, c in s]


def test_the_bring_up_sequences_word_for_word():
    got = {"addr1": _seq(bq.auto_address_sequence(1)), "addr3": _seq(bq.auto_address_sequence(3)),
           "addr3r": _seq(bq.auto_address_sequence(3, reverse=True)), "rev3": _seq(bq.reverse_direction_sequence(3)),
           "bal1": _seq(bq.balancing_sequence(1)), "bal16": _seq(bq.balancing_sequence(16)),
           "bal8": _seq(bq.balancing_sequence(8, timer_code=0x05, duty_code=0x03, done_thresh_code=None)),
           "adc1": _seq(bq.start_adc_sequence(1)), "adc16": _seq(bq.start_adc_sequence(16))}
    f = GOLD.with_name("bq796xx_sequences.json")
    if os.environ.get("KINEMATIK_REGOLD"):
        f.write_text(json.dumps(got, indent=1) + "\n", encoding="utf-8")
    assert got == json.loads(f.read_text(encoding="utf-8"))
    for fn in (bq.balancing_sequence, bq.start_adc_sequence):
        for n in (0, 17):
            with pytest.raises(ValueError, match=r"^cells_per_board must be 1\.\.16$"):
                fn(n)


def test_frames_and_their_refusals():
    assert bq.build_read(0x0568, 4, cmd=bq.Cmd.SINGLE_READ, device=0x105) == \
        bq.build_read(0x0568, 4, cmd=bq.Cmd.SINGLE_READ, device=0x05)
    assert bq.build_read(0x0568, 128) and bq.build_write(0x0309, b"\x01" * 8)
    with pytest.raises(ValueError, match=r"^read of 129 bytes; the device allows 1\.\.128$"):
        bq.build_read(0x0568, 129)
    with pytest.raises(ValueError, match=r"^single_read needs a device address$"):
        bq.build_read(0x0568, 2, cmd=bq.Cmd.SINGLE_READ)
    assert bq.build_read(0x0568, 2, cmd=bq.Cmd.STACK_READ, device=None)
    with pytest.raises(ValueError, match=r"^write payload is 9 bytes; the device accepts 1\.\.8$"):
        bq.build_write(0x0309, b"\x01" * 9)
    with pytest.raises(ValueError, match=r"^single_write needs a device address$"):
        bq.build_write(0x0309, 1, cmd=bq.Cmd.SINGLE_WRITE)
    assert bq.check_crc(bq.append_crc(b"\x01")) and not bq.check_crc(b"\x01\x02")
    with pytest.raises(ValueError, match=r"^payload has 3 bytes, need 4 for 2 cells$"):
        bq.decode_cell_block(b"\x00\x00\x00", 2)
    with pytest.raises(ValueError, match=r"^a stack has at least one device$"):
        bq.wake_time_s(0)
    with pytest.raises(KeyError, match=r"unknown part 'X'\. Known: \['BQ75614-Q1', 'BQ79616-Q1', 'BQ79656-Q1'\]"):
        bq.StackSpec("X").device()
    assert bq.DEVICES["BQ79616-Q1"].max_cells_total() == 560


def test_the_overhead_measurement():
    o = bq.measure_response_overhead(16 * 2 * 3 + 18, cells_per_board=16, boards=3)
    assert o == 6 and isinstance(o, int)
    with pytest.raises(ValueError, match=r"^96 bytes received is not more than the 96-byte payload, so no framing "
                                         r"overhead can be derived\. Either the read was short or the stack is not "
                                         r"fully addressed\.$"):
        bq.measure_response_overhead(96, cells_per_board=16, boards=3)
    with pytest.raises(ValueError, match=r"^7 framing bytes across 3 boards does not divide evenly, so the response "
                                         r"is not a simple repeated frame\. Read the data sheet's Command and "
                                         r"Response Protocol section rather than inferring from this\.$"):
        bq.measure_response_overhead(103, cells_per_board=16, boards=3)


def test_one_and_two_boards_and_the_wake_time_line():
    def plan(n, part="BQ79616-Q1"):
        return bq.plan_stack(bq.StackSpec(part, n, 8))
    assert [c for _, c in plan(1).bring_up] == [c for _, c in bq.start_adc_sequence(8)]
    assert len(plan(2).bring_up) == len(bq.auto_address_sequence(2)) + len(bq.start_adc_sequence(8))
    assert "bq-standalone" not in [f.check for f in plan(1).findings]
    line = [x for x in plan(6).to_markdown().split("\n") if x.startswith("- Wake")][0]
    assert line == f"- Wake-to-addressable: **{bq.wake_time_s(6) * 1000:.1f} ms**"
