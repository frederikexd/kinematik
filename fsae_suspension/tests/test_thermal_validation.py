# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""kinematik-validate scores a measurement against the registered limits: a model-shaped log passes, a log the
model misses fails, and bad input says why."""
import json

import numpy as np
import pytest

from suspension import pack_thermal as pt
from suspension import thermal_validation as tv

CFG = {"rows": 2, "cols": 3, "parallel": 4, "pitch_mm": 27.0, "ambient_c": 25.0, "cell": "Molicel INR-21700-P42A",
       "duct_cfm": 20.0}


def _log(scale_rise=1.0, offset=0.0):
    from suspension import cell_library as cl
    cell, _ = cl.cell_params(CFG["cell"], pitch_mm=27.0)
    lay = pt.PackLayout(rows=2, cols=3, parallel=4, cell=cell, ambient_c=25.0)
    t = np.arange(0.0, 600.0, 2.0)
    cur = np.where((t // 30) % 2 == 0, 120.0, 30.0)
    res = pt.PackThermalModel(layout=lay, airflow=pt.AirflowParams(mode="duct", duct_cfm=20.0)).simulate(t, cur)
    lines = ["time_s,pack_current_a,T_0_0,T_1_2"]
    for k in range(t.size):
        a = 25.0 + scale_rise * (res.temp_history_c[k, 0] - 25.0) + offset
        b = 25.0 + scale_rise * (res.temp_history_c[k, 5] - 25.0) + offset
        lines.append(f"{t[k]},{cur[k]},{a},{b}")
    return "\n".join(lines) + "\n"


def test_a_log_the_model_reproduces_passes():
    r = tv.validate_pack(CFG, _log())
    assert r["passed"] and all(o["share_in_band"] == 1.0 for o in r["thermocouples"])
    assert r["defaults_used"] == ["specific heat"]


def test_a_hotter_pack_than_the_model_is_a_miss():
    r = tv.validate_pack(CFG, _log(scale_rise=1.6))
    assert not r["passed"] and all(not o["pass"] for o in r["thermocouples"])
    assert "MISS" in tv.render(r)


def test_traces_pass_and_miss():
    from suspension.electronics import Trace
    model = Trace(name="x", net="n", owner_subsystem="e", width_mm=1.0, copper_oz=1.0).temp_rise_c(4.0)
    ok = tv.validate_traces(f"width_mm,copper_oz,external,current_a,measured_rise_c\n1.0,1.0,1,4.0,{model}\n")
    bad = tv.validate_traces(f"width_mm,copper_oz,external,current_a,measured_rise_c\n1.0,1.0,1,4.0,{model * 1.5}\n")
    assert ok["passed"] and not bad["passed"]
    assert "| 2 | 1 mm | 1 oz | 4 A |" in tv.render(ok)


@pytest.mark.parametrize("text, msg", [
    ("", "the log is empty"),
    ("time_s,pack_current_a\n0,1\n", "the log has no thermocouple columns (name them T_<row>_<col>)"),
    ("time_s,T_0_0\n0,25\n", "the log has no 'pack_current_a' column"),
    ("time_s,pack_current_a,T_0_0\n0,x,25\n", "column 'pack_current_a' has a value that is not a number"),
    ("time_s,pack_current_a,T_5_0\n0,1,25\n1,1,25\n", "T_5_0 is outside the 2 x 3 grid"),
])
def test_bad_pack_logs_say_why(text, msg):
    with pytest.raises(ValueError) as e:
        tv.validate_pack(CFG, text)
    assert str(e.value) == msg


def test_cli_exit_codes(tmp_path, capsys):
    cfg, log = tmp_path / "c.json", tmp_path / "l.csv"
    cfg.write_text(json.dumps(CFG))
    log.write_text(_log())
    assert tv.main(["pack", "--config", str(cfg), "--log", str(log), "--report", str(tmp_path / "r.md")]) == 0
    log.write_text(_log(scale_rise=1.6))
    assert tv.main(["pack", "--config", str(cfg), "--log", str(log)]) == 1
    assert tv.main(["trace", "--log", str(tmp_path / "missing.csv")]) == 2
    bad = tmp_path / "t.csv"
    bad.write_text("width_mm,current_a\n1,2\n")
    assert tv.main(["trace", "--log", str(bad)]) == 2
    assert "trace log line 2" in capsys.readouterr().err
