"""suspension/ev_excel_roundtrip.py end to end against tests/golden: stock-layout workbooks (live, flattened, without
dependents, without a pack sheet, an offset profile) through the round trip, the cell thermals, the state of
charge, the minimum pack, the enhanced workbook and its read-back, each workbook digested sheet by sheet (values and
formulas, rounded). Regenerate with KINEMATIK_REGOLD=1 only after reading the diff."""
import dataclasses
import hashlib
import io
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

openpyxl = pytest.importorskip("openpyxl")
from suspension import ev_excel_roundtrip as rt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from test_ev_excel_roundtrip import _stock  # noqa: E402

GOLD = Path(__file__).parent / "golden" / "ev_excel_roundtrip.json"


def _r(x):
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: _r(getattr(x, f.name)) for f in dataclasses.fields(x) if f.name != "excel_bytes"}
    if isinstance(x, dict):
        return {str(k): _r(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_r(v) for v in x]
    if isinstance(x, np.ndarray):
        return _r(x.tolist())
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return _r(x.item())
    if isinstance(x, float):
        return None if math.isnan(x) else ("inf" if math.isinf(x) else float(f"{x:.9g}"))
    if isinstance(x, bytes):
        return len(x)
    if hasattr(x, "isoformat"):
        return x.isoformat()
    return x


def _book(b):
    """Each sheet's cells (values and formulas, numbers to 9 figures) as a SHA-256, plus its size."""
    if not b:
        return None
    out = {}
    for data_only in (False, True):
        wb = openpyxl.load_workbook(io.BytesIO(b), data_only=data_only)
        for ws in wb.worksheets:
            cells = [[c.coordinate, _r(c.value if not hasattr(c.value, "text") else c.value.text)]
                     for row in ws.iter_rows() for c in row if c.value is not None]
            out[f"{ws.title}|{int(data_only)}"] = [ws.max_row, ws.max_column, hashlib.sha256(
                json.dumps(cells, default=str).encode()).hexdigest()[:20]]
    return out


def _lap(n=40, kind="ramp"):
    t = np.arange(n) * 0.5
    if kind == "ramp":
        v = np.concatenate([np.linspace(2.0, 28.0, n // 2), np.linspace(28.0, 6.0, n - n // 2)])
    else:
        v = np.full(n, 18.0)
    return v, t


BOOKS = {"live": dict(), "flat": dict(live=False), "no_dependents": dict(dependents=False),
         "no_pack": dict(pack=False), "offset": dict(svt_first=4, svt_name="Speed Vs Time"),
         "numeric": dict(numeric_ratios=True, pack_name="BatteryPackConfig")}


def _case(name, kw, kind, **rkw):
    src = _stock(**kw)
    v, t = _lap(kind=kind)
    r = rt.lap_to_excel_roundtrip(v, t, src, lap_time_s=float(t[-1]), top_speed_ms=float(v.max()),
                                  avg_speed_ms=float(v.mean()), **rkw)
    out = {"result": _r(r), "book": _book(r.excel_bytes), "params": _r(rt.extract_params_from_excel(src))}
    if r.ok and len(r.current_draw_a):
        th = rt.compute_cell_thermals(r.current_draw_a, r.time_s, r.pack)
        th2 = rt.compute_cell_thermals(r.current_draw_a, r.time_s, r.pack, ambient_c=40.0, thermal_mass_j_k=60.0,
                                       h_conv_w_m2k=25.0, cell_surface_m2=0.004)
        soc = rt.compute_soc_and_stop(r.power_kw, r.time_s, r.pack, r.motor)
        soc2 = rt.compute_soc_and_stop(r.power_kw * 40.0, r.time_s, r.pack, r.motor, usable_frac=0.5)
        mp = rt.compute_minimum_feasible_pack(r.power_kw, r.time_s, r.current_draw_a, r.pack, r.motor)
        mp2 = rt.compute_minimum_feasible_pack(r.power_kw * 40.0, r.time_s, r.current_draw_a * 40.0, r.pack,
                                               r.motor, usable_frac=0.6, safety_margin=0.3)
        enh = rt.build_enhanced_excel(r, src, th, soc, mp, lap_time_s=float(t[-1]), top_speed_ms=float(v.max()),
                                      avg_speed_ms=float(v.mean()))
        out.update(thermals=_r(th), thermals2=_r(th2), soc=_r(soc), soc2=_r(soc2), minpack=_r(mp),
                   minpack2=_r(mp2), enhanced=_book(enh), raw=_r(rt.read_raw_sheets(enh)),
                   feasible=_r(rt.find_feasible_pack_sheet(enh)),
                   ledger=_r(rt.ledger_declarations_from_ev_params(rt.extract_params_from_excel(enh))))
    return out


def _from_db():
    params = rt.extract_params_from_excel(_stock())
    v, t = _lap()
    return _r(rt.lap_to_excel_roundtrip_from_db(v, t, params, lap_time_s=19.5, gear=2))


def _cases():
    """case name -> a function computing it (computed one at a time, so a broken case fails before the rest)."""
    out = {name: (lambda name=name, kw=kw: _case(name, kw, "ramp")) for name, kw in BOOKS.items()}
    out["live_const_gear3"] = lambda: _case("live", {}, "const", gear=3)
    out["flat_uncorrected"] = lambda: _case("flat", dict(live=False), "const", correct_workbook=False)
    out["live_vehicle"] = lambda: _case("live", {}, "ramp", vehicle={"mass_kg": 260.0, "cda": 0.9, "crr": 0.02})
    out["from_db"] = _from_db
    return out


def test_every_workbook_word_for_word():
    cases = _cases()
    if os.environ.get("KINEMATIK_REGOLD"):
        got = json.loads(json.dumps({k: f() for k, f in cases.items()}, default=str))
        GOLD.write_text(json.dumps(got, indent=0) + "\n", encoding="utf-8")
    want = json.loads(GOLD.read_text(encoding="utf-8"))
    assert set(cases) == set(want)
    for case, f in cases.items():
        got = json.loads(json.dumps(f(), default=str))
        if isinstance(want[case], dict) and "result" in want[case]:
            for part in want[case]:
                assert got[part] == want[case][part], (case, part)
        else:
            assert got == want[case], case
