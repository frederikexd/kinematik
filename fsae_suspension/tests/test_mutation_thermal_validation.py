"""Characterization suite for suspension/thermal_validation.py (mutation gate); see tests/_golden.py. Log parsing
(BOM, ragged rows, padding, empty logs), the tolerance at and either side of both limits, pack scoring for library
and generic cells with and without a duct, resistance, inlet column or start temperature, logs the model reproduces,
misses and half-meets, every bad-input message, trace scoring for every spelling of the external flag, both reports,
and the command line's exit codes, files, stdout, stderr and help."""
import contextlib
import io
import json

import numpy as np

from suspension import cell_library as cl
from suspension import pack_thermal as pt
from suspension import thermal_validation as tv
from tests._golden import plain
from tests._golden_lazy import check_lazy

LIB = {"rows": 2, "cols": 3, "parallel": 4, "pitch_mm": 27.0, "ambient_c": 25.0, "cell": "Molicel INR-21700-P42A",
       "duct_cfm": 20.0}
GEN = {"rows": 2, "cols": 2, "ambient_c": 20.0, "r_dc_ohm": 0.03, "init_temp_c": 22.0}
T = np.arange(0.0, 120.0, 4.0)
CUR = np.where((T // 20) % 2 == 0, 120.0, 30.0)


def _nominal(cfg, inlet=None, h=1.0):
    pitch = float(cfg.get("pitch_mm", 22.0))
    if cfg.get("cell"):
        cell, _ = cl.cell_params(cfg["cell"], r_dc_ohm=cfg.get("r_dc_ohm"), pitch_mm=pitch)
    else:
        cell = pt.default_cell_params()
        cell.pitch_mm = pitch
        if cfg.get("r_dc_ohm"):
            cell.r_internal_ohm = float(cfg["r_dc_ohm"])
    amb = float(np.mean(inlet)) if inlet is not None else float(cfg["ambient_c"])
    lay = pt.PackLayout(rows=cfg["rows"], cols=cfg["cols"], parallel=int(cfg.get("parallel", 1)), cell=cell,
                        ambient_c=amb)
    air = pt.AirflowParams(mode="duct", duct_cfm=float(cfg.get("duct_cfm", 0.0)))
    init = float(cfg.get("init_temp_c", amb))
    air = pt.AirflowParams(mode="duct", duct_cfm=air.duct_cfm, h_scale=h)
    return pt.PackThermalModel(layout=lay, airflow=air).simulate(T, CUR, init_temp_c=init).temp_history_c, init


def _log(cfg, tcs, scale=1.0, offset=0.0, inlet=None, extra=(), h=1.0, nudge=()):
    hist, init = _nominal(cfg, inlet, h)
    head = ["time_s", "pack_current_a"] + [n for n, _ in tcs] + (["inlet_c"] if inlet is not None else [])
    lines = [",".join(head + [n for n, _ in extra])]
    for k in range(T.size):
        row = [f"{T[k]}", f"{CUR[k]}"] + [f"{init + scale * (hist[k, j] - init) + offset - 5.0 * (k in nudge)}"
                                          for _, j in tcs]
        if inlet is not None:
            row.append(f"{inlet[k]}")
        lines.append(",".join(row + [v for _, v in extra]))
    return "\n".join(lines) + "\n"


def _try(fn, *a):
    try:
        return plain(fn(*a))
    except (ValueError, KeyError) as e:
        return {"raised": type(e).__name__, "message": str(e)}


def _cli(argv, files=None, tmp=None, summary=False):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            code = tv.main(argv)
        except SystemExit as e:
            code = f"exit {e.code}"
    o, e = out.getvalue(), err.getvalue()
    if summary:                 # argparse's own wording differs between Python versions: record what it names
        return [code, [w in o + e for w in ("usage: kinematik-validate ", "\nScore KinematiK against a measurement.\n",
                                            "--config",
                                            "--log", "--report", "pack", "trace")], bool(o), bool(e)]
    if tmp is not None:
        e = e.replace(str(tmp), "<tmp>")
    return [code, o, e, {k: p.read_text() for k, p in (files or {}).items() if p.exists()}]


def _cases(tmp):
    yield "rows", [_try(tv._rows, s) for s in ("﻿a,b\n1,2\n", "XT,b\n1,2\n", "a,b\n1,2,3\n", "a,b,c\n1\n",
                                                " a , b \n 1 , 2 \n", "", "a,b\n")]
    rows = tv._rows("a,b\n1,x\n2.5,3\n")
    yield "col", [_try(tv._col, rows, "a"), _try(tv._col, rows, "b"), _try(tv._col, rows, "c")]
    yield "within", [tv._within(m, x) for m, x in ((10, 12), (10, 12.001), (12, 10), (100, 115), (115.01, 100),
                                                    (-100, -115), (-100, -115.01), (0, 2), (0, -2.0001), (115, 100))]
    lib_tcs = [("T_1_2", 5), ("T_0_0", 0)]
    gen_tcs = [("T_0_1", 1), ("T_1_0", 2)]
    inlet = np.linspace(18.0, 24.0, T.size)
    logs = {
        "lib.match": (LIB, _log(LIB, lib_tcs)), "lib.hot": (LIB, _log(LIB, lib_tcs, scale=1.6)),
        "lib.cool": (LIB, _log(LIB, lib_tcs, scale=0.5)), "lib.offset": (LIB, _log(LIB, lib_tcs, offset=1.5)),
        "lib.offset3": (LIB, _log(LIB, lib_tcs, offset=3.0)),
        "lib.r": ({**LIB, "r_dc_ohm": 0.02}, _log({**LIB, "r_dc_ohm": 0.02}, lib_tcs)),
        "lib.inlet": (LIB, _log(LIB, lib_tcs, inlet=inlet)),
        "lib.h11": (LIB, _log(LIB, lib_tcs, h=1.1)), "lib.h16": (LIB, _log(LIB, lib_tcs, h=1.6)),
        "lib.share90": (LIB, _log(LIB, lib_tcs, nudge=(1, 2, 3))),
        "lib.share87": (LIB, _log(LIB, lib_tcs, nudge=(1, 2, 3, 4))),
        "gen.duct": ({**GEN, "duct_cfm": 15.0}, _log({**GEN, "duct_cfm": 15.0}, gen_tcs)),
        "gen.match": (GEN, _log(GEN, gen_tcs)), "gen.hot": (GEN, _log(GEN, gen_tcs, scale=1.3)),
        "gen.nor": ({k: v for k, v in GEN.items() if k != "r_dc_ohm"},
                    _log({k: v for k, v in GEN.items() if k != "r_dc_ohm"}, gen_tcs)),
        "gen.zero_r": ({**GEN, "r_dc_ohm": 0}, _log({**GEN, "r_dc_ohm": 0}, gen_tcs)),
        "gen.extra_cols": (GEN, _log(GEN, gen_tcs, extra=(("T_0_0x", "1"), ("note", "x"), ("T_a_b", "2")))),
    }
    for name, (cfg, text) in logs.items():
        r = _try(tv.validate_pack, cfg, text)
        yield f"pack.{name}", r
        if "raised" not in r:
            yield f"pack.{name}.md", tv.render(tv.validate_pack(cfg, text))
    head = "time_s,pack_current_a,{}\n0,1,25\n1,1,25\n"
    for bad in ("T_0_3", "T_2_0", "T_1_2,T_10_0"):
        yield f"pack.outside.{bad}", _try(tv.validate_pack, LIB, head.format(bad).replace(",25\n", ",25" + ",25" * bad
                                                                                         .count(",") + "\n"))
    for text in ("", "time_s,pack_current_a\n0,1\n", "time_s,T_0_0\n0,25\n", "time_s,pack_current_a,T_0_0\n0,x,25\n",
                 "pack_current_a,T_0_0\n1,25\n", "time_s,pack_current_a,T_0_0\n0,1,25\n", "time_s,pack_current_a,T_0_0,inlet_c\n0,1,25,q\n"):
        yield f"pack.bad.{text[:44]}", _try(tv.validate_pack, LIB, text)
    yield "pack.nocfg", [_try(tv.validate_pack, {"rows": 2, "cols": 2}, _log(GEN, gen_tcs)),
                         _try(tv.validate_pack, {"cols": 2, "ambient_c": 20.0}, _log(GEN, gen_tcs))]
    yield "pack.backwards", _try(tv.validate_pack, GEN, "time_s,pack_current_a,T_0_0\n4,10,22\n2,10,22\n0,10,22\n")
    model = __import__("suspension.electronics", fromlist=["Trace"]).Trace(
        name="x", net="n", owner_subsystem="e", width_mm=1.0, copper_oz=1.0).temp_rise_c(4.0)
    tr = ("width_mm,copper_oz,external,current_a,measured_rise_c\n"
          f"1.0,1.0,1,4.0,{model}\n1.0,1.0,0,4.0,{model}\n2.5,2,false,6,{model * 1.5}\n1,1,False,4,{model}\n"
          f"1,0.5,,3,1.0\n0.4,1,true,2,30\n")
    yield "trace", _try(tv.validate_traces, tr)
    yield "trace.md", tv.render(tv.validate_traces(tr))
    yield "trace.noext", _try(tv.validate_traces, f"width_mm,copper_oz,current_a,measured_rise_c\n1,1,4,{model}\n")
    for bad in ("width_mm,copper_oz,current_a\n1,1,4\n", f"width_mm,copper_oz,current_a,measured_rise_c\n1,1,4,{model}"
                "\n1,x,4,3\n", "width_mm,current_a,measured_rise_c\n1,4,3\n", "copper_oz,current_a,measured_rise_c\n1,4,3\n",
                "width_mm,copper_oz,measured_rise_c\n1,1,3\n", ""):
        yield f"trace.bad.{bad[:40]}", _try(tv.validate_traces, bad)
    hand = {"kind": "pack", "passed": True, "defaults_used": [], "ambient_c": 25.0,
            "thermocouples": [{"thermocouple": "T_0_0", "share_in_band": 0.955, "peak_rise_measured_k": 12.34,
                               "peak_rise_model_k": 12.36, "max_error_k": 0.05, "pass": True}]}
    yield "render.nodefaults", tv.render(hand)
    yield "render.miss_defaults", tv.render({**hand, "passed": False, "defaults_used": ["a", "b"]})
    yield "render.trace_empty", tv.render({"kind": "trace", "passed": True, "traces": []})
    cfg, log, rep = tmp / "c.json", tmp / "l.csv", tmp / "r.md"
    cfg.write_text(json.dumps(LIB))
    log.write_text(_log(LIB, lib_tcs), encoding="utf-8")
    yield "cli.pack.report", _cli(["pack", "--config", str(cfg), "--log", str(log), "--report", str(rep)], {"r": rep})
    log.write_text("﻿" + _log(LIB, lib_tcs, scale=1.6), encoding="utf-8")
    yield "cli.pack.stdout", _cli(["pack", "--config", str(cfg), "--log", str(log)])
    tlog, trep = tmp / "t.csv", tmp / "t.md"
    tlog.write_text("﻿" + tr, encoding="utf-8")
    yield "cli.trace.report", _cli(["trace", "--log", str(tlog), "--report", str(trep)], {"t": trep})
    tlog.write_text(f"width_mm,copper_oz,current_a,measured_rise_c\n1,1,4,{model}\n", encoding="utf-8")
    yield "cli.trace.stdout", _cli(["trace", "--log", str(tlog)])
    bad = tmp / "bad.csv"
    bad.write_text("width_mm,current_a\n1,2\n")
    yield "cli.trace.bad", _cli(["trace", "--log", str(bad)])
    yield "cli.missing", _cli(["trace", "--log", str(tmp / "missing.csv")], tmp=tmp)
    jbad = tmp / "j.json"
    jbad.write_text("{not json")
    yield "cli.badjson", _cli(["pack", "--config", str(jbad), "--log", str(log)])
    kcfg = tmp / "k.json"
    kcfg.write_text(json.dumps({"cols": 3}))
    yield "cli.keyerror", _cli(["pack", "--config", str(kcfg), "--log", str(log)])
    for argv in ([], ["pack"], ["pack", "--log", "x"], ["pack", "--config", "x"], ["trace"], ["--help"], ["pack", "--help"], ["trace", "--help"],
                 ["nope"]):
        yield f"cli.args.{' '.join(argv)}", _cli(argv, summary=True)
    yield "consts", [tv.BAND_SHARE, tv.PEAK_REL, tv.ABS_FLOOR_K]


def test_every_log_scored_and_reported(tmp_path):
    check_lazy("thermal_validation", _cases(tmp_path))
