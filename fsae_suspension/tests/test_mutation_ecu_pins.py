"""suspension/ecu_pins.py edge by edge: the name-block search at the file's ends, the claims and names a tune and its
wiring tables can disagree on, the relay-chain nets, the table readers (BOM, bad bytes, formulas, .xlsm), the command
line, and the text and PDF reports word for word against tests/golden (regenerate with KINEMATIK_REGOLD=1 only after
reading the diff)."""
import io
import os
import sys
from pathlib import Path

import pytest

from suspension import ecu_pins as ep

sys.path.insert(0, str(Path(__file__).parent))
from test_ecu_pins import _POWER, _ped  # noqa: E402

GOLD = Path(__file__).parent / "golden"


def _gold(name, text):
    f = GOLD / name
    if os.environ.get("KINEMATIK_REGOLD"):
        f.write_text(text, encoding="utf-8")
    assert text == f.read_text(encoding="utf-8")


TUNE = dict(do={1: "Fan", 9: "Water P", 10: "Fuel Pum"}, di={1: "Clutch", 2: "MPH", 3: "Launch"},
            freq={2: "MPH"}, an={2: "MPH", 3: "Oil Pres", 5: "TPS"})
ROWS = [
    ["ID", "From", "To", "Net", "Note"],
    ["W0", "ECU_COMM.99", "", "", ""],
    ["W1", "ECU_MAIN.6", "K4 - Fuel Pump.86", "", "DO10 pulls to GND; Signal: Fuel Pump Relay Control"],
    ["W2", "ECU_MAIN.11", "K5 - Fan.85", "", "Signal: Fan Relay Control"],
    ["W3a", "ECU_MAIN.1", "ECU_MAIN.4", "", "Coil 3 bridge"],
    ["W3", "ECU_MAIN.1", "COIL2.NEG", "", "Signal: Coil 2"],
    ["W4", "ECU_MAIN.3", "INJ1.NEG", "", "Signal: Injector 1"],
    ["W5", "ECU_MAIN.40", "X", "", "spare"],
    ["W6", "Pin M - 30", "TPS sensor", "", "Signal: throttle position"],
    ["W7", "ECU_COMM.16", "ECU_MAIN.2", "", "DO3 jumper"],
    ["W8", "ECU 34-Pin.M09", "Launch button", "", ""],
    ["W9", "ECU_MAIN.3", "INJ9.NEG", "", "Signal: Injector 9"],
] + [r + [""] for r in _POWER[1:]]


def _rep(version="9.9.9"):
    wired = {}
    for pin, t in ep.harness_pins(ROWS).items():
        wired.setdefault(pin, []).extend(t)
    rep = ep.check(ep.read_ped_names(_ped(**TUNE)), wired)
    rep.findings += ep.relay_chains([["ID", "From", "To", "Net"]] + _POWER[1:])
    rep.inputs = [("tune.ped", "ab" * 32), ("wiring.csv", "cd" * 32)]
    rep.version = version
    return rep


def test_the_text_report_word_for_word():
    rep = _rep()
    _gold("ecu_pins_report.txt", ep.format_report(rep))
    assert [(f.level, f.pin) for f in rep.findings] == [
        ("mismatch", ("COMM", 99)), ("mismatch", ("MAIN", 1)), ("mismatch", ("MAIN", 1)), ("mismatch", ("MAIN", 3)),
        ("mismatch", ("MAIN", 3)), ("mismatch", ("MAIN", 40)),
        ("not wired", ("COMM", 6)), ("not wired", ("COMM", 18)), ("not wired", ("MAIN", 8)),
        ("not wired", ("MAIN", 14)), ("not wired", ("MAIN", 13)), ("check", ("COMM", 16)), ("ok", ("MAIN", 11)),
        ("not wired", ("MAIN", 25)), ("ok", ("MAIN", 6)), ("check", ("MAIN", 2)), ("check", None)]


def _pdf_layout(pdf):
    """Every text run with its page, position (0.1 pt) and font size: the layout as well as the words."""
    runs = []
    for n, page in enumerate(pdf.pages):
        def visit(text, cm, tm, font, size, n=n):
            if text.strip():
                runs.append(f"{n} {tm[4] + cm[4]:.1f} {tm[5] + cm[5]:.1f} {size * cm[0]:.2f} {text.strip()}")
        page.extract_text(visitor_text=visit)
    return "\n".join(runs)


def test_the_pdf_report_word_for_word():
    import pypdf
    pdf = pypdf.PdfReader(io.BytesIO(ep.report_pdf(_rep())))
    _gold("ecu_pins_report_pdf.txt", "\n".join(p.extract_text() for p in pdf.pages))
    _gold("ecu_pins_report_pdf_layout.txt", _pdf_layout(pdf))
    clean = ep.EcuPinReport({"DO1": "Fan"}, {("MAIN", 11): ["ECU_MAIN.11 | Signal: Fan Relay Control"]})
    clean.findings = [ep.Finding("ok", ("MAIN", 11), "DO1 'Fan' matches")]
    _gold("ecu_pins_clean_pdf_layout.txt", _pdf_layout(pypdf.PdfReader(io.BytesIO(ep.report_pdf(clean)))))
    assert (pdf.metadata.title, pdf.metadata.author) == ("ECU pin check", "KinematiK")
    assert [round(float(v)) for v in pdf.pages[0].mediabox[2:]] == [595, 842]


def test_a_bare_report():
    rep = ep.EcuPinReport({}, {})
    assert ep.format_report(rep).split("\n") == [
        "ECU pin check (PE3-84XX)", "", "Tune channel names: none found", "ECU pins referenced by the wiring tables: 0",
        "", "No disagreement between tune names, pinout and wiring tables.", "", ep.CAVEAT]
    import pypdf
    text = pypdf.PdfReader(io.BytesIO(ep.report_pdf(rep))).pages[0].extract_text().split("\n")
    assert text[:2] == ["ECU pin check (PE3-84XX)", "KinematiK"] and "none found" in text
    assert "Inputs (SHA-256)" not in text and "Every wired pin" not in text


def test_the_name_block_at_either_end_of_the_file():
    names = b"".join(f"DO #{i}".ljust(8).encode() for i in range(1, 11))
    assert ep._find_block(names, "do#", 10) == (0, 8)
    assert ep._find_block(b"\x00" * 5 + names, "do#", 10) == (5, 8)
    assert ep._find_block(names[:-1], "do#", 10) is None              # the last name would run past the end
    assert ep._printable(b" ~") and not ep._printable(b"\x7f") and not ep._printable(b"\x1f")


def test_a_tune_with_no_default_digital_inputs_still_gives_its_outputs():
    tune = _ped(di={i: f"In{i}" for i in range(1, 8)}, do={1: "Fan"})
    assert ep.read_ped_names(tune) == {"DO1": "Fan"}         # no default left to anchor the inputs: none guessed
    with pytest.raises(ValueError, match=r"^not a Performance Electronics \.ped file$"):
        ep.read_ped_names(b"PK")


def test_words_relays_and_labels():
    assert ep._shares_word("Fuel", ["fueling"]) and ep._shares_word("Fueling", ["fuel"])
    assert not ep._shares_word("Fan", ["pump"])
    assert ep._relay_pin("K5 - Fan.87A") == ("K5", "Fan", "87a")
    assert ep._relay_pin("Radiator Fan Relay #5.87A") == ("K5", "Radiator Fan Relay", "87a")
    assert ep._relay_pin("S13") is None
    assert ep.wiring_label(["Signal: Fan; x", "Signal: Fan | y"]) == "Fan"
    long = "word " * 30
    assert ep.wiring_label([long]) == " ".join(long.split())[:80] and len(ep.wiring_label([long])) == 80


def _net(*pairs):
    return [["ID", "From", "To", "Net"]] + [[f"W{i}", a, b, ""] for i, (a, b) in enumerate(pairs)]


def test_relay_chains_through_85_and_87a_and_short_rows():
    rows = _net(("K1 - Main.87", "S1"), ("S1", "K2 - Pump.85"), ("K2 - Pump.87a", "S2"), ("S2", "K3 - Horn.85"))
    rows.insert(0, ["junk before any header"])
    # rows with an empty or missing end, and rows too short for the To column, join nothing
    rows += [["W9"], ["W10", None, "K7 - Lamp.86"], ["W11", None, "K8 - Aux.87"], ["W11b", "", "K12 - Y.86"], ["W12", "K1 - Main.87", "S9"],
             ["W13", "S9", "K8 - Aux.86"], ["W14", "K9 - Horn2.85", None], ["W15", "K10 - Aux2.87a", None], ["W15b", "K13 - Z.87", ""],
             ["W16", "S9", "K10 - Aux2.86"], ["W17", "K11 - X.86"]]
    (f,) = ep.relay_chains(rows)
    assert f.message == ("relay K3 (Horn) coil 85 is fed from K2 (Pump) switched output 87a, and K2 is itself "
                         "switched by K1: K3 can only switch while K2 is on. Intended, or feed it from K1's net like "
                         "the others?")
    latch = _net(("K1 - Main.87", "S1"), ("S1", "K6 - Latch.86"), ("K6 - Latch.87", "S1"))
    assert ep.relay_chains(latch) == []


def test_the_tables_are_read_from_csv_and_xlsm(tmp_path):
    csv_bytes = "﻿ID,From,To\n".encode() + b"\xff,K1 - Main.87,S1\n" + "\n".join(
        f"W{i},{a},{b}" for i, (a, b) in enumerate([("S1", "K2 - Pump.86"), ("K2 - Pump.87", "S2"),
                                                     ("S2", "K3 - Fan.86")])).encode()
    rows = list(ep.table_rows("w.csv", csv_bytes))
    assert rows[0] == ["ID", "From", "To"] and rows[1][0] == "�"
    assert len(ep.relay_chains(rows)) == 1
    import openpyxl
    wb = openpyxl.Workbook()
    wb.active.append(["ECU_MAIN.6", "pump"])
    wb.active.append(['="ECU_MAIN.11"'])
    buf = io.BytesIO()
    wb.save(buf)
    got = list(ep.table_rows("W.XLSM", buf.getvalue()))
    assert got[0] == ("ECU_MAIN.6", "pump") and got[1] == (None, None)


def test_a_check_run_over_two_tables_keeps_the_first_tables_relay_chain(monkeypatch):
    import suspension.consistency_check as cc
    monkeypatch.setattr(cc, "tool_version", lambda: "1.2.3")
    power = "\n".join(",".join(r) for r in _POWER).encode()
    rep = ep.run(("t.ped", _ped(do={10: "Fuel Pum"})), [("p.csv", power), ("w.csv", b"ECU_MAIN.6,Fuel pump\n")])
    assert [f.level for f in rep.findings if f.pin is None] == ["check"] and rep.version == "1.2.3"
    assert [n for n, _ in rep.inputs] == ["t.ped", "p.csv", "w.csv"]


def test_the_tune_diff():
    a, b = b"Date:xxDate:yy", b"Date:xzDate:qq"
    assert ep.ped_diff(a, b) == [(6, ord("x"), ord("z"))]
    assert ep.ped_diff(b"Date:ab", b"Date:cd") == []
    assert ep.ped_diff(b"abc", b"abd") == [(2, ord("c"), ord("d"))]
    with pytest.raises(ValueError, match=r"^tunes differ in length \(1 vs 2 bytes\): not two saves of one tune "
                                         r"layout$"):
        ep.ped_diff(b"a", b"ab")


def test_the_command_line(tmp_path, capsys, monkeypatch):
    import suspension.consistency_check as cc
    monkeypatch.setattr(cc, "tool_version", lambda: "1.2.3")
    usage = ("usage: python -m suspension.ecu_pins TUNE.ped WIRING_TABLE.(xlsx|csv) [MORE_TABLES ...]\n"
             "       python -m suspension.ecu_pins --diff TUNE_A.ped TUNE_B.ped\n")
    tune = tmp_path / "t.ped"
    tune.write_bytes(_ped(do={10: "Fuel Pum"}))
    assert ep.main([str(tune)]) == 2 and capsys.readouterr().out == usage
    good, bad = tmp_path / "good.csv", tmp_path / "bad.csv"
    good.write_text("ECU_MAIN.6,Fuel pump relay\n")
    bad.write_text("ECU_MAIN.40,spare\n")
    assert ep.main([str(tune), str(good)]) == 0
    out = capsys.readouterr().out
    assert out.startswith("ECU pin check (PE3-84XX), KinematiK 1.2.3\n") and "good.csv" in out
    assert ep.main([str(tune), str(good), str(bad)]) == 1 and "bad.csv" in capsys.readouterr().out
    monkeypatch.setattr("sys.argv", ["ecu_pins", str(tune), str(good)])
    assert ep.main() == 0 and "good.csv" in capsys.readouterr().out
    a, b = tmp_path / "a.ped", tmp_path / "b.ped"
    a.write_bytes(b"\x01\x02\x03")
    b.write_bytes(b"\x01\x07\x09")
    assert ep.main(["--diff", str(a), str(b)]) == 0
    assert capsys.readouterr().out == "0x00001    2 ->   7\n0x00002    3 ->   9\n2 byte(s) differ\n"
