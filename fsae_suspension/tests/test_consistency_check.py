# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""kinematik-check: declared loads vs the board, offline, reproducible."""
import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from suspension.consistency_check import main, parse_loads_csv, render_markdown, run_check, to_json  # noqa: E402

# A 10 A blade fuse on a rail the powertrain side declares at 25 A, plus a 5 V net nobody declared.
BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (net 0 "") (net 1 "GND") (net 2 "STATOR_PWR") (net 3 "AUX_5V")
  (footprint "Connector:J1" (layer "F.Cu") (at 5 10)
    (property "Reference" "J1") (property "Value" "Regulator out")
    (pad "1" thru_hole circle (at 0 0) (size 4 4) (drill 2.0) (net 2 "STATOR_PWR")))
  (footprint "Fuse:F1" (layer "F.Cu") (at 30 10)
    (property "Reference" "F1") (property "Value" "10A blade")
    (pad "1" smd rect (at -4 0) (size 6 9) (net 2 "STATOR_PWR"))
    (pad "2" smd rect (at 4 0) (size 6 9) (net 2 "STATOR_PWR")))
  (segment (start 5 10) (end 26 10) (width 10.0) (layer "F.Cu") (net 2))
  (segment (start 34 10) (end 60 10) (width 10.0) (layer "F.Cu") (net 2))
  (segment (start 5 40) (end 60 40) (width 0.5) (layer "F.Cu") (net 3))
  (segment (start 5 22) (end 60 22) (width 10.0) (layer "F.Cu") (net 1))
)"""
LOADS = "net,current_a,voltage_v,owner\nSTATOR_PWR,25,12,powertrain\nMOTOR_CTRL_12V,3,12,powertrain\n"


def _run(loads=LOADS):
    return run_check(BOARD, "demo.kicad_pcb", loads)


class TestLoadsFile:
    def test_reads_rows_and_owner(self):
        rows = parse_loads_csv("﻿Net,Current_A,Owner\nA,2.5,powertrain\nB,0,\n")
        assert [(r.net, r.current_a, r.voltage_v, r.owner, r.line) for r in rows] == [
            ("A", 2.5, None, "powertrain", 2), ("B", 0.0, None, "unspecified", 3)]

    @pytest.mark.parametrize("text, msg", [
        ("", "loads file is empty: expected a header 'net,current_a' and one row per powered net"),
        ("net,amps\nA,1\n", "loads file has no 'current_a' column (found: amps, net)"),
        ("net,current_a\n,1\n", "loads file line 2: empty net name"),
        ("net,current_a\nA,x\n", "loads file line 2: current_a 'x' is not a number"),
        ("net,current_a\nA,-1\n", "loads file line 2: current_a must be a finite number ≥ 0, got -1.0"),
        ("net,current_a\nA,nan\n", "loads file line 2: current_a must be a finite number ≥ 0, got nan"),
        ("net,current_a,voltage_v\nA,1,0\n", "loads file line 2: voltage_v must be a finite number > 0, got 0.0"),
        ("net,current_a,voltage_v\nA,1,v\n", "loads file line 2: voltage_v 'v' is not a number"),
        ("net,current_a\nA,1\na,2\n", "loads file line 3: net 'a' is declared twice"),
    ])
    def test_bad_input_names_the_line(self, text, msg):
        with pytest.raises(ValueError) as e:
            parse_loads_csv(text)
        assert str(e.value) == msg


class TestCheck:
    def test_undersized_fuse_against_declared_current_fails(self):
        res = _run()
        assert res.boundary[0]["net"] == "STATOR_PWR" and res.boundary[0]["owner"] == "powertrain"
        assert res.boundary[0]["fuses"] == [("F1", 10.0, "FAIL: blows in normal operation")]
        assert any(f.check == "component — F1" and f.severity.value == "fail" for f in res.findings)

    def test_a_fuse_that_holds_says_by_how_much(self):
        res = _run("net,current_a\nSTATOR_PWR,4\n")
        assert res.boundary[0]["fuses"] == [("F1", 10.0, "holds (2.50× the load)")]
        assert not any(f.check == "component — F1" for f in res.findings)

    def test_declared_net_missing_from_board_is_flagged(self):
        f = next(f for f in _run().findings if f.check == "declared load — MOTOR_CTRL_12V")
        assert f.severity.value == "warning" and "loads file line 3" in f.message

    def test_undeclared_net_is_insufficient_evidence_never_a_pass(self):
        res = _run()
        aux = next(n for n in res.nets if n["net"] == "AUX_5V")
        assert not aux["declared"] and aux["provenance"].startswith("assumed — ")
        assert any(f.check == "current not declared — AUX_5V" and f.severity.value == "missing"
                   for f in res.findings)

    def test_report_is_reproducible_and_hashes_inputs(self):
        a, b = render_markdown(_run()), render_markdown(_run())
        assert a == b
        assert "| demo.kicad_pcb | `" in a and "No result here is correlated with a built board." in a
        j = to_json(_run())
        assert j["counts"]["fail"] >= 1 and j["inputs"]["board"]["sha256"] == _run().board_sha256
        assert all(f["evidence"] for f in j["findings"])

    def test_findings_are_ordered_worst_first(self):
        order = ["fail", "warning", "missing", "info", "ok"]
        sev = [order.index(f.severity.value) for f in _run().findings]
        assert sev == sorted(sev)


class TestCli:
    def _files(self, tmp_path, loads=LOADS):
        b, lf = tmp_path / "demo.kicad_pcb", tmp_path / "loads.csv"
        b.write_text(BOARD, encoding="utf-8")
        lf.write_text(loads, encoding="utf-8")
        return str(b), str(lf)

    def test_fail_exits_1_and_writes_both_reports(self, tmp_path):
        b, lf = self._files(tmp_path)
        md, js = tmp_path / "r.md", tmp_path / "r.json"
        assert main([b, "--loads", lf, "--report", str(md), "--json", str(js)]) == 1
        assert md.read_text(encoding="utf-8").startswith("# KinematiK consistency check")
        assert json.loads(js.read_text(encoding="utf-8"))["tool"] == "kinematik-check"

    def test_fail_on_never_exits_0(self, tmp_path, capsys):
        b, lf = self._files(tmp_path)
        assert main([b, "--loads", lf, "--fail-on", "never"]) == 0
        assert "## Declared loads vs the hardware" in capsys.readouterr().out

    def test_warning_threshold_counts_warnings(self, tmp_path):
        b, lf = self._files(tmp_path, "net,current_a\nSTATOR_PWR,4\nNOT_ON_BOARD,1\n")
        assert main([b, "--loads", lf, "--fail-on", "fail", "--report", str(tmp_path / "r.md")]) == 0
        assert main([b, "--loads", lf, "--fail-on", "warning", "--report", str(tmp_path / "r.md")]) == 1

    def test_bad_input_exits_2_with_the_reason(self, tmp_path, capsys):
        b, lf = self._files(tmp_path, "net,amps\nA,1\n")
        assert main([b, "--loads", lf]) == 2
        assert capsys.readouterr().err == "kinematik-check: loads file has no 'current_a' column (found: amps, net)\n"

    def test_missing_file_exits_2(self, tmp_path):
        b, _ = self._files(tmp_path)
        assert main([b, "--loads", str(tmp_path / "nope.csv")]) == 2
