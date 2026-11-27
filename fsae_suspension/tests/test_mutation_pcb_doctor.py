"""suspension/pcb_doctor.py end to end against tests/golden: the KiCad and Altium demo boards, each with a light and a
heavy fan load and two sets of diagnosis limits, through parsing, current assignment, every check, the automatic
fixes and their re-check, the bridge screen, the bring-up sheet, the hand-off report and the board picture (by
SHA-256), plus the sizing formulas over a grid. Regenerate with KINEMATIK_REGOLD=1 only after reading the diff."""
import dataclasses
import enum
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np

from suspension import pcb_doctor as pd
from suspension.pcb_altium import demo_altium_pcb

GOLD = Path(__file__).parent / "golden" / "pcb_doctor.json"


def _plain(x, depth=0):
    if depth > 12:
        return repr(type(x))
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: _plain(getattr(x, f.name), depth + 1) for f in dataclasses.fields(x)
                if f.name not in ("text", "raw")}
    if isinstance(x, enum.Enum):
        return x.value
    if isinstance(x, dict):
        return {str(k): _plain(v, depth + 1) for k, v in x.items()}
    if isinstance(x, (list, tuple, set, frozenset)):
        items = sorted(x, key=repr) if isinstance(x, (set, frozenset)) else x
        return [_plain(v, depth + 1) for v in items]
    if isinstance(x, np.ndarray):
        return _plain(x.tolist(), depth + 1)
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return _plain(x.item(), depth + 1)
    if isinstance(x, float):
        return None if math.isnan(x) else ("inf" if math.isinf(x) else float(f"{x:.9g}"))
    if isinstance(x, bytes):
        return hashlib.sha256(x).hexdigest()
    return x


def _sha(s):
    return hashlib.sha256(s.encode()).hexdigest()


BOARDS = {"kicad": (pd.demo_kicad_pcb, "d.kicad_pcb"), "altium": (demo_altium_pcb, "demo_ecu_board.PcbDoc")}
LIMITS = {"default": {}, "hot": dict(rail_v=12.0, brownout_v=11.0, ambient_c=65.0, max_temp_c=95.0, fuse_sf=1.5,
                                     dT_budget_c=10.0)}


def _case(board_name, fan_a, limits):
    make, fname = BOARDS[board_name]
    board = pd.parse_board(make(), fname)
    assignments = pd.auto_assign_net_currents(board, ledger=None)
    auto = _plain(assignments)
    pd.apply_demo_declarations(board, assignments, overrides={"FAN_PWR": {"current_a": fan_a}})
    rep = pd.diagnose(board, assignments, **LIMITS[limits])
    fixed = pd.fix_board(board, assignments, rep, **LIMITS[limits])
    rows = pd.bridge_risk_pairs(board)
    out = {"format": pd.sniff_format(make(), fname),
           "nets": _plain(board.nets), "layers": _plain(board.copper_layers), "thickness": board.board_thickness_mm,
           "n": [len(board.segments), len(board.vias), len(board.footprints), len(board.zones)],
           "auto_assignments": auto, "assignments": _plain(assignments),
           "summary": rep.summary(), "counts": rep.counts(),
           "findings": _plain(rep.findings), "fixes": _plain(rep.fixes), "vias": _plain(rep.via_recommendations),
           "net_reports": _plain(rep.net_reports),
           "fixed": {k: (_sha(v) if isinstance(v, str) and len(v) > 2000 else _plain(v)) for k, v in fixed.items()},
           "diff_pairs": _plain(pd.find_diff_pairs(board)),
           "bridge_rows": _plain(rows), "hitlist": _plain(pd.bridge_hitlist(rows)),
           "bridge_md": pd.bridge_check_md(board, rows), "symptom_md": pd.bridge_symptom_md(rows),
           "bringup": _plain(pd.bringup_measurements(board)), "bringup_md": pd.bringup_sheet_md(board).split("\n"),
           "svg": _sha(pd.board_svg(board, rep)), "svg_plain": _sha(pd.board_svg(board)),
           "svg_applied": _sha(pd.board_svg(board, rep, show_layers=["F.Cu"], width_px=500, height_px=300,
                                            applied=[a.seg_index for a in fixed.get("applied") or []]))}
    applied = fixed.get("applied", [])
    out["report_md"] = pd.fix_report_md(board, rep, applied, assignments).split("\n")
    return out


def _formulas():
    rows = []
    for i_a in (0.5, 3.0, 12.0):
        for dt in (10.0, 20.0):
            for oz in (0.5, 1.0, 2.0):
                for ext in (True, False):
                    rows.append([i_a, dt, oz, ext, pd.required_area_mil2(i_a, dt, ext),
                                 pd.required_width_mm(i_a, dt, oz, ext), pd.trace_ampacity_a(1.0, dt, oz, ext)])
    vias = [[d, dt, pd.via_ampacity_a(d, dt), pd.vias_needed(5.0, d, dt), pd.via_resistance_ohm(d, 1.6)]
            for d in (0.2, 0.3, 0.6) for dt in (10.0, 20.0)]
    presc = [_plain(pd.prescribe_trace(i_a, dt)) for i_a in (0.2, 2.0, 15.0, 60.0) for dt in (10.0, 20.0)]
    clear = [pd.clearance_required_mm(v) for v in (0.0, 5.0, 15.0, 30.0, 50.0, 100.0, 150.0, 300.0, 500.0, 1000.0)]
    return {"traces": _plain(rows), "vias": _plain(vias), "prescribe": presc, "clearance": _plain(clear)}


def _cases():
    """case name -> a function computing it (computed one at a time, so a broken case fails before the rest)."""
    out = {"formulas": _formulas}
    for b in BOARDS:
        for fan in (2.0, 20.0):
            for lim in LIMITS:
                out[f"{b}_{fan:g}_{lim}"] = (lambda b=b, fan=fan, lim=lim: _case(b, fan, lim))
    return out


def test_every_board_word_for_word():
    cases = _cases()
    if os.environ.get("KINEMATIK_REGOLD"):
        got = json.loads(json.dumps({k: f() for k, f in cases.items()}, ensure_ascii=False))
        GOLD.write_text(json.dumps(got, indent=0, ensure_ascii=False) + "\n", encoding="utf-8")
    want = json.loads(GOLD.read_text(encoding="utf-8"))
    assert set(cases) == set(want)
    for case, f in cases.items():
        got = json.loads(json.dumps(f(), ensure_ascii=False))
        for part in want[case]:
            assert got[part] == want[case][part], (case, part)
