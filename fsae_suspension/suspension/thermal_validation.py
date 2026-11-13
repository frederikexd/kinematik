# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Compare KinematiK's pack and trace temperatures with a measurement: the step from verified to validated.

Every benchmark so far checks KinematiK against other tools and correlations. Only a measured temperature shows
whether the model matches hardware. This turns a logged test into a scored comparison with one command, against
acceptance limits fixed before any data (docs/MEASURED_VALIDATION_PROTOCOL.md):

    kinematik-validate pack  --config pack.json --log pack_log.csv  --report pack_validation.md
    kinematik-validate trace --log traces.csv                       --report trace_validation.md

Pack log (CSV): time_s, pack_current_a, and one column per thermocouple named T_<row>_<col> (°C), with the grid
indices counted from 0 in the layout the config describes; an optional inlet_c column sets the air temperature.
Pack config (JSON): rows, cols, parallel, pitch_mm, ambient_c, and either "cell" (a cell_library name) or the
generic cell, with optional "r_dc_ohm", "duct_cfm" (0 = still air) and "init_temp_c".

Trace log (CSV): width_mm, copper_oz, external (1/0), current_a, ambient_c, measured_rise_c (steady state).

Pack acceptance: at least 90 % of the logged points of every thermocouple inside the model's ±20 % cooling band,
and the peak rise within 15 % (or 2 K, whichever is larger). Trace acceptance: every steady rise within 15 %
(or 2 K). A miss is reported as a miss: the point of this command is to find out.
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import re
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

BAND_SHARE = 0.90          # share of logged points inside the cooling band (unitless)
PEAK_REL = 0.15            # peak-rise tolerance, fraction of the measured rise (unitless)
ABS_FLOOR_K = 2.0          # ...but never tighter than this, K (thermocouple accuracy plus contact)


def _rows(text: str) -> list[dict]:
    rows = list(csv.DictReader(io.StringIO(text.lstrip("﻿"))))
    if not rows:
        raise ValueError("the log is empty")
    return [{(k or "").strip(): (v or "").strip() for k, v in r.items()} for r in rows]


def _col(rows, name) -> np.ndarray:
    try:
        return np.array([float(r[name]) for r in rows])
    except KeyError:
        raise ValueError(f"the log has no '{name}' column") from None
    except ValueError:
        raise ValueError(f"column '{name}' has a value that is not a number") from None


def _within(model: float, measured: float) -> bool:
    return abs(model - measured) <= max(PEAK_REL * abs(measured), ABS_FLOOR_K)


def validate_pack(config: dict, log_text: str) -> dict:
    """Run the logged current through the model and score every thermocouple. Temperatures in degC, rises in
    Kelvin, current in amps."""
    from . import cell_library
    from . import pack_thermal as pt
    rows = _rows(log_text)
    t, cur = _col(rows, "time_s"), _col(rows, "pack_current_a")
    tc_cols = sorted(k for k in rows[0] if re.fullmatch(r"T_\d+_\d+", k))
    if not tc_cols:
        raise ValueError("the log has no thermocouple columns (name them T_<row>_<col>)")
    n_rows, n_cols = int(config["rows"]), int(config["cols"])
    pitch = float(config.get("pitch_mm", 22.0))
    if config.get("cell"):
        cell, defaults = cell_library.cell_params(config["cell"], r_dc_ohm=config.get("r_dc_ohm"), pitch_mm=pitch)
    else:
        cell, defaults = pt.default_cell_params(), ["every cell property (generic 21700)"]
        cell.pitch_mm = pitch
        if config.get("r_dc_ohm"):
            cell.r_internal_ohm = float(config["r_dc_ohm"])
    ambient = float(np.mean(_col(rows, "inlet_c"))) if "inlet_c" in rows[0] else float(config["ambient_c"])
    layout = pt.PackLayout(rows=n_rows, cols=n_cols, parallel=int(config.get("parallel", 1)), cell=cell,
                           ambient_c=ambient)
    air = pt.AirflowParams(mode="duct", duct_cfm=float(config.get("duct_cfm", 0.0)))
    init = float(config.get("init_temp_c", ambient))
    runs = {}
    for key, scale in (("nominal", 1.0), ("cool", 1.2), ("warm", 0.8)):
        model = pt.PackThermalModel(layout=layout, airflow=replace(air, h_scale=scale))
        res = model.simulate(t, cur, init_temp_c=init)
        if not res.ok:
            raise ValueError("the model could not run on this log: " + "; ".join(res.warnings))
        runs[key] = res.temp_history_c
    out = []
    for c in tc_cols:
        r_i, c_i = (int(x) for x in c.split("_")[1:])
        if not (0 <= r_i < n_rows and 0 <= c_i < n_cols):
            raise ValueError(f"{c} is outside the {n_rows} x {n_cols} grid")
        j = r_i * n_cols + c_i
        meas = _col(rows, c)
        lo = np.minimum(runs["cool"][:, j], runs["warm"][:, j])
        hi = np.maximum(runs["cool"][:, j], runs["warm"][:, j])
        share = float(np.mean((meas >= lo - 1e-9) & (meas <= hi + 1e-9)))
        peak_m, peak_k = float(np.max(meas) - init), float(np.max(runs["nominal"][:, j]) - init)
        out.append({"thermocouple": c, "share_in_band": share, "peak_rise_measured_k": peak_m,
                    "peak_rise_model_k": peak_k, "max_error_k": float(np.max(np.abs(runs["nominal"][:, j] - meas))),
                    "pass": share >= BAND_SHARE and _within(peak_k, peak_m)})
    return {"kind": "pack", "thermocouples": out, "defaults_used": defaults, "ambient_c": ambient,
            "passed": all(o["pass"] for o in out)}


def validate_traces(log_text: str) -> dict:
    """Steady rise of each logged trace against the model's, in Kelvin; widths in mm, current in amps."""
    from .electronics import Trace
    out = []
    for i, r in enumerate(_rows(log_text), start=2):
        try:
            tr = Trace(name=f"row{i}", net="test", owner_subsystem="electrics", width_mm=float(r["width_mm"]),
                       copper_oz=float(r["copper_oz"]), is_external=r.get("external", "1") not in ("0", "false"))
            current, measured = float(r["current_a"]), float(r["measured_rise_c"])
        except (KeyError, ValueError):
            raise ValueError(f"trace log line {i}: needs width_mm, copper_oz, current_a and measured_rise_c "
                             "as numbers") from None
        model = tr.temp_rise_c(current)
        out.append({"line": i, "width_mm": tr.width_mm, "copper_oz": tr.copper_oz, "current_a": current,
                    "measured_rise_k": measured, "model_rise_k": model, "pass": _within(model, measured)})
    return {"kind": "trace", "traces": out, "passed": all(o["pass"] for o in out)}


def render(result: dict) -> str:
    verdict = "PASS: within the registered limits" if result["passed"] else "MISS: outside the registered limits"
    L = [f"# KinematiK measured validation: {result['kind']}", "", f"**{verdict}**", ""]
    if result["kind"] == "pack":
        L += ["| Thermocouple | Points in ±20 % band | Peak rise measured | Peak rise model | Max error | Result |",
              "|---|---|---|---|---|---|"]
        for o in result["thermocouples"]:
            L.append(f"| {o['thermocouple']} | {o['share_in_band']:.0%} | {o['peak_rise_measured_k']:.1f} K | "
                     f"{o['peak_rise_model_k']:.1f} K | {o['max_error_k']:.1f} K | {'pass' if o['pass'] else 'MISS'} |")
        if result["defaults_used"]:
            L += ["", "Left at a generic value (not datasheet or measured): " + ", ".join(result["defaults_used"])
                  + ". A miss may come from these inputs rather than the model."]
    else:
        L += ["| Line | Width | Copper | Current | Rise measured | Rise model | Result |",
              "|---|---|---|---|---|---|---|"]
        for o in result["traces"]:
            L.append(f"| {o['line']} | {o['width_mm']:g} mm | {o['copper_oz']:g} oz | {o['current_a']:g} A | "
                     f"{o['measured_rise_k']:.1f} K | {o['model_rise_k']:.1f} K | {'pass' if o['pass'] else 'MISS'} |")
    L += ["", f"Limits (fixed before the data): {BAND_SHARE:.0%} of points in the ±20 % cooling band (pack); peak or "
          f"steady rise within {PEAK_REL:.0%} or {ABS_FLOOR_K:g} K.", ""]
    return "\n".join(L)


def main(argv=None) -> int:
    """Command line. Returns a unitless exit status: 0 within the limits, 1 a miss, 2 bad input."""
    ap = argparse.ArgumentParser(prog="kinematik-validate", description="Score KinematiK against a measurement.")
    sub = ap.add_subparsers(dest="kind", required=True)
    p = sub.add_parser("pack")
    p.add_argument("--config", required=True)
    p.add_argument("--log", required=True)
    p.add_argument("--report")
    t = sub.add_parser("trace")
    t.add_argument("--log", required=True)
    t.add_argument("--report")
    a = ap.parse_args(argv)
    try:
        if a.kind == "pack":
            res = validate_pack(json.loads(Path(a.config).read_text(encoding="utf-8")),
                                Path(a.log).read_text(encoding="utf-8-sig"))
        else:
            res = validate_traces(Path(a.log).read_text(encoding="utf-8-sig"))
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as e:
        print(f"kinematik-validate: {e}", file=sys.stderr)
        return 2
    md = render(res)
    if a.report:
        Path(a.report).write_text(md, encoding="utf-8")
    else:
        print(md)
    return 0 if res["passed"] else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
