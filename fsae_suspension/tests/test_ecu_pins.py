# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""ECU pin check on synthetic PE3 tunes laid out like PE3Monitor V5.7 files (no team file is used)."""
import csv
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from suspension import ecu_pins as ep  # noqa: E402


def _ped(do=None, di=None, an=None, freq=None, size=16438):
    """A tune with the four name blocks at the offsets and strides PE3Monitor V5.7 uses."""
    b = bytearray(size)
    b[:len(ep.PED_MAGIC)] = ep.PED_MAGIC

    def put(off, text, right=False):
        raw = (text.rjust(8) if right else text.ljust(8)).encode()
        b[off:off + 8] = raw

    an = an or {}
    for i in range(8):                                   # analog names: 16-byte records
        put(0x283B + 16 * i, an.get(i + 1, f"An#{i + 1}"), right=True)
    freq = freq or {}
    for i in range(4):                                   # frequency names: 12-byte records
        put(0x28BC + 12 * i, freq.get(i + 1, f"Freq #{i + 1}"), right=True)
    di, do = di or {}, do or {}
    for i in range(7):                                   # digital names: one 8-byte run, inputs then outputs
        put(0x32AF + 8 * i, di.get(i + 1, f"DI #{i + 1}"))
    for i in range(10):
        put(0x32AF + 8 * (7 + i), do.get(i + 1, f"DO #{i + 1}"))
    return bytes(b)


NICK_LIKE = dict(do={1: "Fan", 5: "", 6: "", 7: "", 10: "Fuel Pum"}, di={2: "MPH"}, freq={2: "MPH"})


def test_reads_custom_names_and_skips_defaults_and_blanks():
    names = ep.read_ped_names(_ped(**NICK_LIKE))
    assert names == {"DO1": "Fan", "DO10": "Fuel Pum", "DI2": "MPH", "FREQ2": "MPH"}


def test_analog_names_are_read_too():
    assert ep.read_ped_names(_ped(an={5: "OilPres"}))["AN5"] == "OilPres"


def test_not_a_pe_file_is_refused():
    with pytest.raises(ValueError):
        ep.read_ped_names(b"MoTeC M1 package" + bytes(100))


def test_a_tune_with_every_name_changed_reports_nothing_rather_than_guessing():
    all_renamed = {i: f"Out{i}" for i in range(1, 11)}
    assert "DO1" not in ep.read_ped_names(_ped(do=all_renamed))


def test_pinout_maps_channels_to_pins():
    assert ep.pin_of("DO10") == ("MAIN", 6) and ep.pin_of("DO1") == ("MAIN", 11)
    assert ep.pin_of("DI2") == ep.pin_of("FREQ2") == ("MAIN", 13)
    assert ep.pin_of("AN1") == ("COMM", 5) and ep.pin_of("XX1") is None


def test_harness_pins_reads_both_table_styles():
    wired = ep.harness_pins([
        ["ECU_MAIN.6", "LW4 (From K4)", "DO10 pulls to GND when on"],
        ["K5 - Fan.85", None, "EH.LW6", "Pin M -11"],
        ["ECU_COMM.14", "ETH.1"],
    ])
    assert set(wired) == {("MAIN", 6), ("MAIN", 11), ("COMM", 14)}


def _wiring():
    return ep.harness_pins([
        ["ECU_MAIN.6", "LW4 (From K4)", "DO10 pulls to GND when on; Signal: Fuel Pump Relay Control"],
        ["ECU_MAIN.11", "LW6 (From K5)", "DO1 pulls to GND when on; Signal: Fan Relay Control"],
        ["ECU_MAIN.1", "COIL1.NEG", "Signal: Coil 1"],
        ["ECU_MAIN.3", "INJ1.NEG", "Signal: Injector 1"],
    ])


def test_consistent_names_pass_and_an_unwired_named_input_is_flagged_once():
    rep = ep.check(ep.read_ped_names(_ped(**NICK_LIKE)), _wiring())
    assert [(f.level, f.pin) for f in rep.problems] == [("not wired", ("MAIN", 13))]
    assert "DI2/FREQ2 'MPH'" in rep.problems[0].message
    assert {f.pin for f in rep.findings if f.level == "ok"} == {("MAIN", 6), ("MAIN", 11)}


def test_wiring_table_that_puts_the_wrong_output_on_a_pin_is_a_mismatch():
    wired = ep.harness_pins([["ECU_MAIN.6", "DO9 pulls to GND when on"]])
    (f,) = [p for p in ep.check({}, wired).problems if p.level == "mismatch"]
    assert "MAIN-25" in f.message


def test_wrong_coil_or_injector_number_is_a_mismatch():
    wired = ep.harness_pins([["ECU_MAIN.1", "COIL2.NEG"], ["ECU_MAIN.3", "Injector 2"]])
    assert {f.pin for f in ep.check({}, wired).problems} == {("MAIN", 1), ("MAIN", 3)}


def test_a_row_naming_two_ecu_pins_makes_no_claim():
    wired = ep.harness_pins([["ECU_MAIN.6", "ECU_MAIN.11", "DO10"]])
    assert [p for p in ep.check({}, wired).problems if p.level == "mismatch"] == []


def test_a_pin_the_ecu_does_not_have_is_a_mismatch():
    (f,) = ep.check({}, ep.harness_pins([["ECU_MAIN.40", "spare"]])).problems
    assert f.level == "mismatch" and "no such pin" in f.message


def test_name_that_disagrees_with_the_wired_function_is_flagged_for_a_check():
    wired = ep.harness_pins([["ECU_MAIN.11", "Signal: Fuel Pump Relay Control"]])
    (f,) = ep.check({"DO1": "Fan"}, wired).problems
    assert f.level == "check"


def test_report_states_what_was_not_read():
    text = ep.format_report(ep.check(ep.read_ped_names(_ped(**NICK_LIKE)), _wiring()))
    assert "[not wired]" in text and "behave correctly" in text
    clean = ep.format_report(ep.check({"DO1": "Fan", "DO10": "Fuel Pum"}, _wiring()))
    assert "No disagreement" in clean


def test_command_line_on_csv_and_xlsx(tmp_path, capsys):
    openpyxl = pytest.importorskip("openpyxl")
    ped = tmp_path / "tune.ped"
    ped.write_bytes(_ped(**NICK_LIKE))
    with open(tmp_path / "engine.csv", "w", newline="") as fh:
        csv.writer(fh).writerows([["From", "Note"], ["ECU_MAIN.6", "Fuel Pump Relay Control"]])
    wb = openpyxl.Workbook()
    wb.active.append(["K5 - Fan.85", "Pin M -11"])
    wb.save(tmp_path / "power.xlsx")
    code = ep.main([str(ped), str(tmp_path / "engine.csv"), str(tmp_path / "power.xlsx")])
    out = capsys.readouterr().out
    assert code == 1 and "[not wired]" in out and "DO10 'Fuel Pum'" in out
    assert ep.main([str(ped)]) == 2


def test_run_records_version_and_input_hashes():
    import hashlib
    ped = _ped(**NICK_LIKE)
    table = b"From,Note\nECU_MAIN.6,Fuel Pump Relay Control\n"
    rep = ep.run(("tune.ped", ped), [("engine.csv", table)])
    assert rep.inputs == [("tune.ped", hashlib.sha256(ped).hexdigest()),
                          ("engine.csv", hashlib.sha256(table).hexdigest())]
    text = ep.format_report(rep)
    assert f"KinematiK {rep.version}" in text and hashlib.sha256(ped).hexdigest() in text


def test_panel_asks_for_files_before_checking():
    pytest.importorskip("streamlit")
    from streamlit.testing.v1 import AppTest
    root = str(Path(__file__).resolve().parents[1])
    at = AppTest.from_string(f"import sys\nsys.path.insert(0, {root!r})\nfrom ui import ecu_pins\necu_pins.render()\n")
    at.run()
    assert not at.exception and any("Add a tune" in i.value for i in at.info)


def test_pdf_report_carries_findings_inputs_and_caveat():
    pypdf = pytest.importorskip("pypdf")
    pytest.importorskip("reportlab")
    import io
    rep = ep.run(("tune.ped", _ped(**NICK_LIKE)), [("engine.csv", b"From\nECU_MAIN.6,Fuel Pump\n")])
    pdf = ep.report_pdf(rep)
    text = " ".join(" ".join(p.extract_text().split()) for p in pypdf.PdfReader(io.BytesIO(pdf)).pages)
    assert pdf.startswith(b"%PDF") and "NOT WIRED" in text and "MAIN-13" in text
    assert rep.inputs[0][1] in text.replace(" ", "") and "behave correctly" in text and f"KinematiK {rep.version}" in text


def test_an_output_the_harness_drives_but_the_tune_never_set_up_is_flagged():
    """Nicholas's first case: the pump relay was wired, but no output in the ECU setup was assigned to it."""
    wired = ep.harness_pins([["ECU_MAIN.6", "Fuel Pump Relay Control"], ["ECU_MAIN.11", "Fan Relay Control"]])
    rep = ep.check({"DO1": "Fan"}, wired)
    (f,) = rep.problems
    assert f.level == "check" and f.pin == ("MAIN", 6) and "has not named DO10" in f.message
    assert ep.check({"DO1": "Fan", "DO10": "Fuel Pum"}, wired).problems == []


def test_inputs_and_fixed_function_pins_are_not_prompted():
    wired = ep.harness_pins([["ECU_MAIN.1", "Coil 1"], ["ECU_MAIN.29", "CLT"], ["ECU_MAIN.14", "switch"]])
    assert ep.check({}, wired).problems == []


def test_diff_locates_the_one_setting_that_changed_and_ignores_the_save_date():
    a = bytearray(_ped(**NICK_LIKE)) + b"Date: 11-21-2026"
    b = bytearray(a)
    b[0x1234] = 9
    b[-10:] = b"11-22-2026"[:10]
    assert ep.ped_diff(bytes(a), bytes(b)) == [(0x1234, 0, 9)]
    with pytest.raises(ValueError):
        ep.ped_diff(bytes(a), bytes(a[:-1]))


def test_diff_command_line(tmp_path, capsys):
    a = _ped()
    b = bytearray(a)
    b[100] = 7
    (tmp_path / "a.ped").write_bytes(a)
    (tmp_path / "b.ped").write_bytes(bytes(b))
    assert ep.main(["--diff", str(tmp_path / "a.ped"), str(tmp_path / "b.ped")]) == 0
    assert "0x00064    0 ->   7" in capsys.readouterr().out


def _rows_by_pin(rep):
    return {r.pin: r for r in ep.pin_rows(rep)}


def test_every_wired_pin_is_listed_against_the_pinout():
    rep = ep.check(ep.read_ped_names(_ped(**NICK_LIKE)), _wiring())
    rows = _rows_by_pin(rep)
    assert set(rows) == {"MAIN-1", "MAIN-3", "MAIN-6", "MAIN-11"}
    assert rows["MAIN-1"].function == "Coil #1" and rows["MAIN-1"].result == "matches"
    assert rows["MAIN-1"].wiring == "Coil 1"
    assert rows["MAIN-6"].result == rows["MAIN-11"].result == "matches tune name"


def test_pin_table_marks_mismatches_checks_and_unlabelled_pins():
    wired = ep.harness_pins([["ECU_MAIN.1", "COIL2.NEG; Signal: Coil 2"], ["ECU_MAIN.6", "Signal: Fuel Pump Relay"],
                             ["ECU_MAIN.9", "Signal: Power Ground"], ["ECU_MAIN.12", "W13 green-pink 20 AWG"]])
    rows = _rows_by_pin(ep.check({}, wired))
    assert rows["MAIN-1"].result == "mismatch"
    assert rows["MAIN-6"].result == "check"                   # a driven output the tune never set up
    assert rows["MAIN-9"].result == "matches"                 # GND and Ground are the same thing
    assert rows["MAIN-12"].result == "nothing to compare"


def test_wiring_label_reads_signal_notes_and_relay_names():
    label = ep.wiring_label(["W45 | K4 - Pumps.85 | Pin M-6", "ECU_MAIN.6 | LW4 | Signal: Fuel Pump Relay Control"])
    assert label == "K4 - Pumps / Fuel Pump Relay Control"
    assert ep.wiring_label(["ECU_MAIN.12 | W13"]) == "ECU_MAIN.12 | W13"


def test_every_wired_output_gets_a_pe3monitor_line_to_confirm():
    """Nicholas's first case, pump function never assigned: not readable, so the report says where to look."""
    rep = ep.check({"DO1": "Fan"}, _wiring())
    todo = ep.confirm_list(rep)
    assert len(todo) == 2 and all("Tuning > Digital Outputs" in t for t in todo)
    assert any("DO10 on MAIN-6" in t and "Fuel Pump Relay Control" in t for t in todo)
    assert any("DO1 (named 'Fan' in the tune) on MAIN-11" in t for t in todo)
    text = ep.format_report(rep)
    assert "[ ] PE3Monitor" in text and "Every wired pin:" in text


def test_pdf_has_the_pin_table_and_the_checklist():
    pypdf = pytest.importorskip("pypdf")
    pytest.importorskip("reportlab")
    import io
    pdf = ep.report_pdf(ep.check(ep.read_ped_names(_ped(**NICK_LIKE)), _wiring()))
    text = " ".join(p.extract_text() for p in pypdf.PdfReader(io.BytesIO(pdf)).pages)
    assert "Every wired pin" in text and "Confirm in PE3Monitor" in text and "Coil #1" in text


_POWER = [["ID", "From", "To", "Net"],
          ["W38", "K1 - ECU Power.87", "S12", ""],
          ["W40", "S12", "K2 - Injectors.86", ""],
          ["W39", "K2 - Injectors.87", "S13", "Injectors"],
          ["W47", "S13", "K5 - Fan.86", "Injectors"],
          ["W48", "K5 - Fan.85", "EH.LW6", "Pin M -11"],
          ["W21", "FB.F5", "K5 - Fan.30", ""]]


def test_relay_coil_fed_from_another_relays_output_is_flagged():
    """Calvin's K5 fan relay coil sat on S13, the net K2 (injectors) switches: it only worked while K2 was on."""
    found = ep.relay_chains(_POWER)
    msgs = [f.message for f in found]
    assert len(found) == 1 and found[0].level == "check"
    assert "K5 (Fan) coil 86 is fed from K2 (Injectors) switched output 87, and K2 is itself switched by K1" in msgs[0]


def test_coils_fed_from_the_main_relay_are_ordinary_design():
    """K2, K3 and K4 fed from K1's output is how the other relays were wired, and how Calvin moved K5."""
    fixed = [r for r in _POWER if r[0] != "W47"] + [["W47", "S12", "K5 - Fan.86", ""]]
    assert ep.relay_chains(fixed) == []


def test_relay_coils_on_a_plain_supply_are_not_flagged():
    rows = [["From", "To"], ["FB.F1", "S1"], ["S1", "K1 - Main.86"], ["K1 - Main.87", "ECU.34"]]
    assert ep.relay_chains(rows) == []
    assert ep.relay_chains([["ECU_MAIN.6", "Signal"]]) == []          # no From/To header: nothing to build


def test_run_reports_relay_chains_and_leftover_names(tmp_path):
    openpyxl = pytest.importorskip("openpyxl")
    import io
    wb = openpyxl.Workbook()
    for r in _POWER:
        wb.active.append(r)
    buf = io.BytesIO()
    wb.save(buf)
    rep = ep.run(("tune.ped", _ped(**NICK_LIKE)), [("power.xlsx", buf.getvalue())])
    text = ep.format_report(rep)
    assert "K5 (Fan) coil 86 is fed from K2 (Injectors)" in text
    assert "left over from a base tune" in text


# --- harness-tool exports: connectors named by size, relays named in words (1.7.10) --------------------------------

def test_connector_named_by_size_is_read_as_main_or_comm():
    rows = [["W51", "Fuel Pump Relay #4.85", "ECU 34-Pin.M06", "ECU Pin M-6/Fuel Pump Relay"],
            ["W80", "ECU 26-Pin.C14", "Laptop.1", "Ethernet"],
            ["W81", "ECU 34-Pin.M10", "Injector #2.INJ1.2", "Fuel Injector #2"]]
    pins = ep.harness_pins(rows)
    assert set(pins) == {(ep.MAIN, 6), (ep.MAIN, 10), (ep.COMM, 14)}
    assert len(pins[(ep.MAIN, 6)]) == 1          # 'M06' and 'Pin M-6' in one row: listed once


def test_part_connector_pin_is_not_read_as_a_coil_or_injector_number():
    rows = [["W66", "ECU 34-Pin.M04", "Coil #2.COIL1.2", "Ignition Coil #2/GND"],
            ["W67", "ECU 34-Pin.M10", "Injector #2.INJ1.2", "Fuel Injector #2/GND"]]
    rep = ep.check({}, ep.harness_pins(rows))
    assert not [f for f in rep.findings if f.level == "mismatch"]


def test_relays_named_in_words_are_followed_like_k_numbers():
    rows = [["ID", "From", "To"],
            ["W46", "ECU Power Relay #1.87", "S2"],
            ["W40", "S2", "Injector Power Relay #3.86"],
            ["W39", "Injector Power Relay #3.87", "S4"],
            ["W43", "Radiator Fan Relay #5.86", "S4"],
            ["W50", "S2", "Fuel Pump Relay #4.86"]]
    found = ep.relay_chains(rows)
    assert len(found) == 1 and "K5 (Radiator Fan Relay)" in found[0].message and "K3" in found[0].message


def test_later_sheets_with_other_columns_do_not_join_nets():
    rows = [["ID", "Color", "From", "To", "Net"],
            ["W46", "Red", "ECU Power Relay #1.87", "S2", "BAT +"],
            ["W43", "White", "Radiator Fan Relay #5.86", "S2", "BAT +"],
            # a per-splice sheet: no From/To; its 3rd and 4th columns must not be read as a connection
            ["Destination", "ID", "Color", "Gauge", "Net"],
            ["Injector Power Relay #3.87", "W39", "Radiator Fan Relay #5.86", "18 AWG", "x"],
            ["S2", "W40", "Injector Power Relay #3.86", "16 AWG", "x"]]
    assert ep.relay_chains(rows) == []
