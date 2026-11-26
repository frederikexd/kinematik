"""Characterization suite for suspension/consistency_check.py (mutation gate); see tests/_golden.py. Loads files in
every shape and every refusal, a board with a fuse that holds, one that blows, one without a readable rating and a
net without a fuse, declared nets the board lacks, a zero load, the Markdown and JSON reports, the evidence labels,
counts at each fail-on level, and the command line's exit codes, outputs and errors."""
import contextlib
import io

from suspension import consistency_check as C
from tests._golden import plain
from tests._golden_lazy import check_lazy

BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (net 0 "") (net 1 "GND") (net 2 "STATOR_PWR") (net 3 "AUX_5V") (net 4 "LV_12V")
  (footprint "Connector:J1" (layer "F.Cu") (at 5 10)
    (property "Reference" "J1") (property "Value" "Regulator out")
    (pad "1" thru_hole circle (at 0 0) (size 4 4) (drill 2.0) (net 2 "STATOR_PWR")))
  (footprint "Fuse:F1" (layer "F.Cu") (at 30 10)
    (property "Reference" "F1") (property "Value" "10A blade")
    (pad "1" smd rect (at -4 0) (size 6 9) (net 2 "STATOR_PWR"))
    (pad "2" smd rect (at 4 0) (size 6 9) (net 2 "STATOR_PWR")))
  (footprint "Fuse:F2" (layer "F.Cu") (at 30 40)
    (property "Reference" "F2") (property "Value" "PTC resettable")
    (pad "1" smd rect (at -2 0) (size 2 2) (net 3 "AUX_5V")))
  (footprint "Fuse:F3" (layer "F.Cu") (at 40 40)
    (property "Reference" "F3") (property "Value" "2.5 A")
    (pad "1" smd rect (at -2 0) (size 2 2) (net 3 "AUX_5V")))
  (footprint "Fuse:F4" (layer "F.Cu") (at 40 60)
    (property "Reference" "F4") (property "Value" "3a")
    (pad "1" smd rect (at -2 0) (size 2 2) (net 4 "LV_12V")))
  (footprint "Res:R1" (layer "F.Cu") (at 50 40)
    (property "Reference" "R1") (property "Value" "1A")
    (pad "1" smd rect (at -2 0) (size 2 2) (net 3 "AUX_5V")))
  (segment (start 5 10) (end 26 10) (width 10.0) (layer "F.Cu") (net 2))
  (segment (start 34 10) (end 60 10) (width 10.0) (layer "F.Cu") (net 2))
  (segment (start 5 40) (end 60 40) (width 0.5) (layer "F.Cu") (net 3))
  (segment (start 5 60) (end 60 60) (width 2.0) (layer "F.Cu") (net 4))
  (segment (start 5 22) (end 60 22) (width 10.0) (layer "F.Cu") (net 1))
)"""
LOADS = ("net,current_a,voltage_v,owner\nSTATOR_PWR,25,12,powertrain\nMOTOR_CTRL_12V,3,12,powertrain\n"
         "aux_5v,0,,\nLV_12V,1.5,12,electrics\n")


def _res(r):
    return {"board": [r.board_file, len(r.board_sha256)], "loads": [r.loads_file, r.loads_sha256], "params": r.params,
            "nets": plain(r.nets), "findings": plain(r.findings), "boundary": plain(r.boundary), "counts": r.counts(),
            "hits": [r.worst_hits(k) for k in ("fail", "warning", "never")], "md": C.render_markdown(r),
            "json": plain(C.to_json(r))}


def _cli(argv):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            code = C.main(argv)
        except SystemExit as e:
            code = f"exit {e.code}"
    return code, out.getvalue(), err.getvalue()


def _cases(tmp):
    yield "evidence", [C.evidence_for(c) for c in ("IR drop on X", "trace ampacity", "fusing margin F1", "component R1",
                                                    "copper short", "copper open", "via bottleneck", "HV clearance",
                                                    "diff pair", "current not declared", "declared load — X", "board",
                                                    "something else", "")]
    yield "version", C.tool_version() == C.tool_version() and isinstance(C.tool_version(), str)
    for text in ("net,current_a\nA,1\n", "﻿Net , Current_A ,Owner\n a , 2.5 , pt \n", "net,current_a,voltage_v\nA,0,5\n",
                 "net,current_a,extra\nA,1,x,y\n", "net,current_a\n", "", "current_a\n1\n", "net\nA\n",
                 "net,current_a\n,1\n", "net,current_a\nA,1\na,2\n", "net,current_a\nA,x\n", "net,current_a\nA,-1\n",
                 "net,current_a\nA,inf\n", "net,current_a\nA,nan\n", "net,current_a,voltage_v\nA,1,x\n",
                 "net,current_a,voltage_v\nA,1,0\n", "net,current_a,voltage_v\nA,1,-5\n",
                 "net,current_a,voltage_v\nA,1,inf\n", "net,current_a,owner\nA,1,\nB,2,me\n",
                 "net,current_a,voltage_v\nA,1,0.5\n", ",\n,\n", "Xnet,current_a\nA,1\n", "net,current_a\nA,1,2\n"):
        try:
            yield f"loads.{text!r}", plain(C.parse_loads_csv(text))
        except ValueError as e:
            yield f"loads.{text!r}", {"raised": str(e)}
    yield "run", _res(C.run_check(BOARD, "dir/demo.kicad_pcb", LOADS, "x/loads.csv"))
    yield "run.bytes", _res(C.run_check(BOARD.encode(), "demo.kicad_pcb", LOADS.encode(), rail_v=3.3, brownout_v=3.0,
                                        ambient_c=25.0, max_temp_c=85.0))
    yield "run.none", _res(C.run_check(BOARD, "demo.kicad_pcb", "net,current_a\nNOPE,1\n"))
    yield "run.light", _res(C.run_check(BOARD, "demo.kicad_pcb", "net,current_a\nSTATOR_PWR,10\nAUX_5V,2.5\n"))
    yield "run.gnd", _res(C.run_check(BOARD, "demo.kicad_pcb", "net,current_a,voltage_v\nGND,0.1,\nAUX_5V,0.5,\n",
                                      rail_v=3.3))
    for bad in ("(not a board)", ""):
        try:
            C.run_check(bad, "broken.kicad_pcb", LOADS)
        except ValueError as e:
            yield f"run.badboard.{bad}", str(e)
    try:
        C.run_check(BOARD, "demo.kicad_pcb", b"\xff\xfe\x00")
    except ValueError as e:
        yield "run.notutf8", str(e)
    b, lf = tmp / "demo.kicad_pcb", tmp / "decl.csv"
    b.write_text(BOARD, encoding="utf-8")
    lf.write_text(LOADS, encoding="utf-8")
    rep, js = tmp / "r.md", tmp / "r.json"
    for argv in ([str(b), "--loads", str(lf)], [str(b), "--loads", str(lf), "--fail-on", "never"],
                 [str(b), "--loads", str(lf), "--fail-on", "warning", "--rail-v", "3.3", "--brownout-v", "3.1",
                  "--ambient-c", "20", "--max-temp-c", "90"]):
        yield f"cli.{' '.join(argv[3:])}", _cli(argv)
    code, out, err = _cli([str(b), "--loads", str(lf), "--report", str(rep), "--json", str(js)])
    yield "cli.files", [code, out, err, rep.read_text(encoding="utf-8"), js.read_text(encoding="utf-8")]
    light = tmp / "light.csv"
    light.write_text("net,current_a\nSTATOR_PWR,1\n")
    yield "cli.clean", [_cli([str(b), "--loads", str(light), "--fail-on", w])[0] for w in ("fail", "warning", "never")]
    yield "cli.missing", [_cli([str(b), "--loads", str(tmp / "none.csv")])[0],
                          _cli([str(b), "--loads", str(tmp / "none.csv")])[2].replace(str(tmp), "<tmp>")]
    bad = tmp / "bad.csv"
    bad.write_text("net,current_a\nA,x\n")
    yield "cli.bad", _cli([str(b), "--loads", str(bad)])
    yield "cli.args", [_cli([])[0], _cli([str(b)])[0], _cli([str(b), "--loads", str(lf), "--fail-on", "x"])[0]]
    code, out, err = _cli(["--help"])
    flat = " " + " ".join(out.split()) + " "         # argparse wraps help to the terminal width
    yield "cli.help", [code, [f" {w} " in flat or f" {w}," in flat or f"[{w}" in flat for w in (
        "kinematik-check", "Check the currents one team declares against the board another team drew. Runs offline.",
        "KiCad .kicad_pcb or Altium .PcbDoc file", "CSV: net,current_a[,voltage_v][,owner]",
        "write the Markdown report here (default: print it)", "also write a machine-readable JSON report here",
        "logic rail voltage for brown-out (default 5)", "brown-out threshold on that rail (default 4.5)",
        "ambient temperature, °C (default 40)", "copper temperature limit, °C (default 105)",
        "exit 1 if any result at or above this level (default fail; 'never' always exits 0)", "--loads", "--json",
        "--rail-v", "--brownout-v", "--ambient-c", "--max-temp-c", "--fail-on", "--report", "board")]]
    yield "tables", [C._STATUS, C._ORDER, {k: list(v) for k, v in C._FAIL_ON.items()}, [list(e) for e in C._EVIDENCE]]


def test_every_load_and_board_check(tmp_path):
    check_lazy("consistency_check", _cases(tmp_path))
