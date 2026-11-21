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
    (f,) = ep.check({}, wired).problems
    assert f.level == "mismatch" and "MAIN-25" in f.message


def test_wrong_coil_or_injector_number_is_a_mismatch():
    wired = ep.harness_pins([["ECU_MAIN.1", "COIL2.NEG"], ["ECU_MAIN.3", "Injector 2"]])
    assert {f.pin for f in ep.check({}, wired).problems} == {("MAIN", 1), ("MAIN", 3)}


def test_a_row_naming_two_ecu_pins_makes_no_claim():
    wired = ep.harness_pins([["ECU_MAIN.6", "ECU_MAIN.11", "DO10"]])
    assert ep.check({}, wired).problems == []


def test_a_pin_the_ecu_does_not_have_is_a_mismatch():
    (f,) = ep.check({}, ep.harness_pins([["ECU_MAIN.40", "spare"]])).problems
    assert f.level == "mismatch" and "no such pin" in f.message


def test_name_that_disagrees_with_the_wired_function_is_flagged_for_a_check():
    wired = ep.harness_pins([["ECU_MAIN.11", "Signal: Fuel Pump Relay Control"]])
    (f,) = ep.check({"DO1": "Fan"}, wired).problems
    assert f.level == "check"


def test_report_states_what_was_not_read():
    text = ep.format_report(ep.check(ep.read_ped_names(_ped(**NICK_LIKE)), _wiring()))
    assert "[not wired]" in text and "not behaviour" in text
    clean = ep.format_report(ep.check({"DO1": "Fan"}, _wiring()))
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
    assert rep.inputs[0][1] in text.replace(" ", "") and "not behaviour" in text and f"KinematiK {rep.version}" in text
