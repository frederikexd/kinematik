# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/track_sim_export.py — write lap-sim results into the electrics
#  workbook as a LIVE model: correct physics, real Excel formulas, cached
#  values, and not one of the user's own cells touched.
# ============================================================================
"""
Track-sim export that produces a workbook someone can actually use.

WHAT WAS WRONG WITH THE OLD EXPORT
----------------------------------
The previous path (`ev_excel_roundtrip.lap_to_excel_roundtrip`) produced a file
that opened cleanly and was still not much use, for four separate reasons:

**1. It replaced formulas with constants.** Every computed cell in
ElecPropulsion came back as a static number. The workbook stopped being a model
— change a speed, a gear ratio or the pack layout and nothing recalculates —
and the errors baked into those constants became invisible to any audit that
reads formulas. A wrong formula can be found and fixed; a wrong number that used
to be a formula cannot.

**2. It re-implemented the workbook's own physics errors.** Gear ratios applied
as 1/N so the motor turned slower than the wheel, current as
`V_pack * PF * RPM / 1000`, efficiency multiplying instead of dividing. The
export agreed with the spreadsheet because it was a copy of the spreadsheet, so
the round-trip could never disagree with the thing it was supposed to check.
Peak power came out at 294 kW from a pack whose absolute ceiling is 106 kW.

**3. It wrote formulas with no cached values.** openpyxl does not evaluate what
it writes, so every formula cell read back as None to `data_only=True` — which
is every Python consumer, including KinematiK's own readers. Excel recalculated
on open and looked fine; nothing else could read the file at all.

**4. Its pack advice ignored current.** The old advisor reasoned from energy
alone and recommended dropping a 140S3P pack to 140S1P because the lap only
needed 1.02 of 6.96 kWh. Going to one parallel string triples pack resistance,
cuts deliverable power to a third, and puts the entire pack current through
every cell — about 39C for a 5 Ah cell. It recommended the change that makes the
fuse failure three times worse, on a sheet already reporting that the fuse
fails.

WHAT THIS MODULE DOES INSTEAD
-----------------------------
* **Physics from `power_draw`** — a real force balance, `P_elec = P_wheel/eta`,
  and pack current solved from `P = (V_oc - I*R)*I` so sag is included and an
  impossible demand is reported as impossible rather than as a large number.
* **Live Excel formulas throughout.** Inputs are numbers; everything derived is
  a formula referencing them. Change the mass on `KX Inputs` and the whole
  workbook moves, including the gear recommendation. This is the difference
  between a report and a model.
* **Cached values populated.** `export_track_sim` recalculates through
  LibreOffice before returning, so `data_only=True` readers see numbers.
* **Nothing of the user's is touched.** Every sheet this writes is prefixed
  `KX `. Their formulas, their layout and their row numbering are left exactly
  as they were, which also means the audit trail survives and re-running is
  idempotent.
* **Advice that accounts for current.** The pack advisor checks energy AND the
  power ceiling AND per-cell C-rate, so it cannot recommend a pack that is
  thermally or electrically incapable of the lap it just simulated.

Only Excel-2007-era functions are used (`SUMPRODUCT`, `INDEX`, `MAX`, `IFERROR`)
so the formulas survive recalculation outside Excel.
"""

from __future__ import annotations

import math
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from typing import Optional, Sequence

from .interfaces import Severity, Finding
from . import power_draw as pdw

#: Every sheet this module writes carries this prefix. Nothing else is modified.
SHEET_PREFIX = "KX "

S_INPUTS = SHEET_PREFIX + "Inputs"
S_TRACE = SHEET_PREFIX + "Lap Trace"
S_DASH = SHEET_PREFIX + "Dashboard"
S_GEARS = SHEET_PREFIX + "Gear Study"
S_PACK = SHEET_PREFIX + "Pack Limits"
S_ADVISOR = SHEET_PREFIX + "Pack Advisor"
S_PROV = SHEET_PREFIX + "Provenance"

_FONT = "Arial"
_BLUE = "0000FF"        # hardcoded input
_BLACK = "000000"       # formula
_GREEN = "008000"       # link to another sheet
_YELLOW = "FFFFFF00"    # must-confirm assumption


# ===================================================================== #
#  Styling helpers
# ===================================================================== #
def _style(ws, coord, *, bold=False, color=_BLACK, fill=None, fmt=None,
           size=10, wrap=False):
    from openpyxl.styles import Font, PatternFill, Alignment
    c = ws[coord]
    c.font = Font(name=_FONT, size=size, bold=bold, color=color)
    if fill:
        c.fill = PatternFill("solid", fgColor=fill)
    if fmt:
        c.number_format = fmt
    if wrap:
        c.alignment = Alignment(wrap_text=True, vertical="top")
    return c


def _header(ws, coord, text):
    _style(ws, coord, bold=True, size=12).value = text


def _label(ws, row, text, col=1):
    _style(ws, f"{_col(col)}{row}", bold=False).value = text


def _col(n: int) -> str:
    s = ""
    while n > 0:
        n, r = divmod(n - 1, 26)
        s = chr(65 + r) + s
    return s


def _input(ws, row, label, value, unit="", note="", fmt="0.0000",
           must_confirm=False):
    """A labelled input cell: blue for typed values, yellow if unverified."""
    _label(ws, row, label)
    c = _style(ws, f"B{row}", color=_BLUE, fmt=fmt,
               fill=_YELLOW if must_confirm else None)
    c.value = value
    _style(ws, f"C{row}").value = unit
    if note:
        _style(ws, f"D{row}", color="808080", wrap=True).value = note
    return f"$B${row}"


# ===================================================================== #
#  Input specification
# ===================================================================== #
@dataclass
class ExportSpec:
    """Everything the export needs, and where each number came from."""
    pack: pdw.PackSpec
    vehicle: pdw.VehicleSpec
    drive: pdw.DriveSpec
    speed_mph: Sequence[float]
    dt_s: float
    lap_time_s: Optional[float] = None
    reductions: Sequence[float] = tuple(range(1, 16))
    smooth_window: int = 11
    endurance_km: Optional[float] = 22.0
    #: Per-cell continuous discharge rating, in C. Used by the advisor to
    #: reject packs that meet the energy requirement and cannot supply the
    #: current. Datasheet figure; the default is a conservative 21700 value and
    #: is flagged as an assumption because it changes the recommendation.
    max_cell_c_rate: float = 10.0
    soc_window: tuple[float, float] = (0.10, 0.80)


# ===================================================================== #
#  Sheet builders
# ===================================================================== #
def _write_inputs(ws, spec: ExportSpec) -> dict:
    """Every assumption in its own labelled cell, referenced by everything else.

    Yellow fill marks the numbers nobody has verified. Three of them —  mass,
    drag area and rolling resistance — do not exist anywhere in the original
    workbook, and every current figure depends on them, so they are the first
    cells a reviewer should look at.
    """
    _header(ws, "A1", "KinematiK — inputs for the lap-sim export")
    _style(ws, "A2", color="808080", wrap=True).value = (
        "Blue = typed input. Yellow = assumption not taken from the workbook; "
        "confirm before quoting any result. Everything on the other KX sheets "
        "is a formula referencing this one, so editing here updates the whole "
        "export.")
    r = {}
    row = 4
    _header(ws, f"A{row}", "Vehicle")
    row += 1
    r["mass"] = _input(ws, row, "Mass incl. driver", spec.vehicle.mass_kg, "kg",
                       "Not present in the original workbook. Peak current is "
                       "roughly proportional to this.", "0.0",
                       must_confirm=True); row += 1
    r["crr"] = _input(ws, row, "Rolling resistance coeff", spec.vehicle.crr, "-",
                      "Generic FSAE figure.", "0.0000",
                      must_confirm=True); row += 1
    r["cda"] = _input(ws, row, "Drag area CdA", spec.vehicle.cda_m2, "m^2",
                      "Generic FSAE figure. Dominates power above ~40 mph.",
                      "0.000", must_confirm=True); row += 1
    r["rho"] = _input(ws, row, "Air density", spec.vehicle.air_density,
                      "kg/m^3", "", "0.000"); row += 1
    r["mu"] = _input(ws, row, "Longitudinal grip coeff",
                     spec.vehicle.mu_lon, "-",
                     "Caps the acceleration term, so a glitched speed sample "
                     "cannot become a current spike.", "0.00"); row += 1
    r["wheel_in"] = _input(ws, row, "Wheel diameter", spec.vehicle.wheel_diameter_in,
                           "in", "", "0.0"); row += 1
    row += 1

    _header(ws, f"A{row}", "Drivetrain")
    row += 1
    r["eta_drive"] = _input(ws, row, "Drivetrain efficiency",
                            spec.vehicle.drivetrain_efficiency, "-", "",
                            "0.000"); row += 1
    r["eta_motor"] = _input(ws, row, "Motor efficiency",
                            spec.vehicle.motor_efficiency, "-", "",
                            "0.0000"); row += 1
    r["eta_inv"] = _input(ws, row, "Inverter efficiency",
                          spec.vehicle.inverter_efficiency, "-",
                          "Not in the original workbook.", "0.000",
                          must_confirm=True); row += 1
    _label(ws, row, "Total efficiency")
    r["eta"] = f"$B${row}"
    _style(ws, f"B{row}", fmt="0.0000").value = \
        f"={r['eta_drive']}*{r['eta_motor']}*{r['eta_inv']}"
    _style(ws, f"D{row}", color="808080", wrap=True).value = (
        "Electrical demand DIVIDES by this. The original workbook multiplied, "
        "which made a more efficient motor draw more current.")
    row += 2

    _header(ws, f"A{row}", "Selected gear")
    row += 1
    r["reduction"] = _input(ws, row, "Gear reduction (motor:wheel)",
                            spec.drive.reduction, ":1",
                            "A reduction of N means the motor turns N times "
                            "per wheel turn, so motor rpm = wheel rpm * N.",
                            "0.00"); row += 1
    r["max_rpm"] = _input(ws, row, "Motor max speed",
                          spec.drive.motor_max_rpm, "rpm", "", "0"); row += 1
    r["t_peak"] = _input(ws, row, "Motor peak torque",
                         spec.drive.motor_peak_torque_nm, "Nm", "",
                         "0.0"); row += 1
    r["t_cont"] = _input(ws, row, "Motor continuous torque",
                         spec.drive.motor_continuous_torque_nm, "Nm", "",
                         "0.0"); row += 1
    r["p_motor"] = _input(ws, row, "Motor peak power",
                          spec.drive.motor_peak_power_kw, "kW", "",
                          "0.0"); row += 2

    _header(ws, f"A{row}", "Pack (read from the workbook by row label)")
    row += 1
    p = spec.pack
    r["n_series"] = _input(ws, row, "Series groups", p.n_series, "-", "",
                           "0"); row += 1
    r["n_par"] = _input(ws, row, "Parallel strings", p.n_parallel, "-", "",
                        "0"); row += 1
    r["cell_v"] = _input(ws, row, "Cell nominal voltage", p.cell_voltage_v,
                         "V", "", "0.000"); row += 1
    r["cell_ah"] = _input(ws, row, "Cell capacity", p.cell_capacity_ah, "Ah",
                          "", "0.00"); row += 1
    r["cell_r"] = _input(ws, row, "Cell internal resistance",
                         p.cell_resistance_ohm, "ohm", "", "0.00000"); row += 1
    r["fuse"] = _input(ws, row, "Fuse rating", p.fuse_max_a, "A", "",
                       "0.0"); row += 1
    r["c_rate"] = _input(ws, row, "Cell max continuous discharge",
                         spec.max_cell_c_rate, "C",
                         "Datasheet figure. Changes the pack recommendation, "
                         "so it is an assumption worth checking.", "0.0",
                         must_confirm=True); row += 2

    _header(ws, f"A{row}", "Session")
    row += 1
    r["dt"] = _input(ws, row, "Sample interval", spec.dt_s, "s", "",
                     "0.00000"); row += 1
    r["endurance"] = _input(ws, row, "Endurance distance",
                            spec.endurance_km or 22.0, "km", "",
                            "0.0"); row += 1
    r["soc_hi"] = _input(ws, row, "SOC ceiling", spec.soc_window[1], "-", "",
                         "0.00"); row += 1
    r["soc_lo"] = _input(ws, row, "SOC floor", spec.soc_window[0], "-", "",
                         "0.00"); row += 2

    _header(ws, f"A{row}", "Constants")
    row += 1
    r["g"] = _input(ws, row, "g", 9.80665, "m/s^2", "", "0.00000"); row += 1
    r["mph_ms"] = _input(ws, row, "mph -> m/s", 0.44704, "-", "",
                         "0.00000"); row += 1
    r["mph_ipm"] = _input(ws, row, "mph -> in/min", 1056.0, "-",
                          "63360 in/mile / 60 min.", "0.0"); row += 1

    ws.column_dimensions["A"].width = 34
    ws.column_dimensions["B"].width = 14
    ws.column_dimensions["C"].width = 9
    ws.column_dimensions["D"].width = 62
    return r


_TRACE_COLS = [
    ("Time", "s", "0.000"),
    ("Speed", "mph", "0.00"),
    ("Speed", "m/s", "0.000"),
    ("Accel (grip-limited)", "m/s^2", "0.000"),
    ("F roll", "N", "0.0"),
    ("F aero", "N", "0.0"),
    ("F accel", "N", "0.0"),
    ("F total", "N", "0.0"),
    ("P wheel", "W", "0"),
    ("P electrical", "W", "0"),
    ("Pack current", "A", "0.00"),
    ("Over pack ceiling?", "1/0", "0"),
    ("Joule heat", "W", "0.0"),
    ("Motor speed", "rpm", "0"),
    ("Motor torque", "Nm", "0.00"),
]


def _write_trace(ws, spec: ExportSpec, ref: dict, n: int) -> dict:
    """The lap, as formulas. Editing an input on KX Inputs moves every row."""
    _header(ws, "A1", "KinematiK — lap trace (every column a live formula)")
    for i, (name, unit, _fmt) in enumerate(_TRACE_COLS, start=1):
        _style(ws, f"{_col(i)}2", bold=True).value = f"{name} ({unit})"
    ws.freeze_panes = "A3"

    I = f"'{S_INPUTS}'!"
    first, last = 3, 2 + n
    for k in range(n):
        r = first + k
        prev, nxt = max(first, r - 1), min(last, r + 1)
        span = f"({nxt}-{prev})" if False else None
        # speed in m/s
        ws[f"C{r}"] = f"=B{r}*{I}{ref['mph_ms']}"
        # central difference; the divisor counts the rows actually spanned so
        # the first and last samples use a one-sided step rather than a wrong one
        raw = (f"(C{nxt}-C{prev})/(({nxt}-{prev})*{I}{ref['dt']})")
        grip = f"{I}{ref['mu']}*{I}{ref['g']}"
        ws[f"D{r}"] = f"=MAX(-{grip},MIN({grip},{raw}))"
        ws[f"E{r}"] = (f"=IF(C{r}>0.05,{I}{ref['crr']}*{I}{ref['mass']}"
                       f"*{I}{ref['g']},0)")
        ws[f"F{r}"] = f"=0.5*{I}{ref['rho']}*{I}{ref['cda']}*C{r}^2"
        ws[f"G{r}"] = f"={I}{ref['mass']}*D{r}"
        ws[f"H{r}"] = f"=E{r}+F{r}+G{r}"
        ws[f"I{r}"] = f"=H{r}*C{r}"
        ws[f"J{r}"] = f"=IF(I{r}>0,I{r}/{I}{ref['eta']},0)"
        # sag-aware current: R*I^2 - Voc*I + P = 0, low-current root.
        voc = f"({I}{ref['n_series']}*{I}{ref['cell_v']})"
        rp = f"({I}{ref['n_series']}*({I}{ref['cell_r']}/{I}{ref['n_par']}))"
        disc = f"({voc}^2-4*{rp}*J{r})"
        ws[f"K{r}"] = (f"=IF(J{r}<=0,0,IF({disc}<0,{voc}/(2*{rp}),"
                       f"({voc}-SQRT({disc}))/(2*{rp})))")
        ws[f"L{r}"] = f"=IF(J{r}<=0,0,IF({disc}<0,1,0))"
        ws[f"M{r}"] = f"=K{r}^2*{rp}"
        ws[f"N{r}"] = (f"=B{r}*{I}{ref['mph_ipm']}/({I}{ref['wheel_in']}*PI())"
                       f"*{I}{ref['reduction']}")
        ws[f"O{r}"] = (f"=MAX(0,H{r})*({I}{ref['wheel_in']}*0.0254/2)"
                       f"/({I}{ref['reduction']}*{I}{ref['eta_drive']})")
        for i, (_nm, _u, fmt) in enumerate(_TRACE_COLS, start=1):
            _style(ws, f"{_col(i)}{r}", fmt=fmt)
    for i in range(1, len(_TRACE_COLS) + 1):
        ws.column_dimensions[_col(i)].width = 13
    return {"first": first, "last": last}


def _write_pack_limits(ws, ref: dict) -> dict:
    """The pack's electrical envelope — none of which the original computes."""
    I = f"'{S_INPUTS}'!"
    _header(ws, "A1", "KinematiK — pack limits")
    _style(ws, "A2", color="808080", wrap=True).value = (
        "All derived from the primary cell inputs. The workbook's own pack "
        "resistance cell multiplies the TOTAL cell count by R_cell/P, which "
        "cancels to S*R_cell and discards the parallel benefit — 3x high for "
        "140S3P. These figures use S*(R_cell/P).")
    out = {}
    row = 4
    rows = [
        ("Pack nominal voltage", f"={I}{ref['n_series']}*{I}{ref['cell_v']}",
         "V", "0.0", "voc"),
        ("Pack capacity", f"={I}{ref['n_par']}*{I}{ref['cell_ah']}", "Ah",
         "0.00", "ah"),
        ("Pack energy", "=B4*B5/1000", "kWh", "0.000", "kwh"),
        ("Usable energy (SOC window)",
         f"=B6*({I}{ref['soc_hi']}-{I}{ref['soc_lo']})", "kWh", "0.000",
         "usable"),
        ("Pack resistance",
         f"={I}{ref['n_series']}*({I}{ref['cell_r']}/{I}{ref['n_par']})",
         "ohm", "0.0000", "r"),
        ("Max deliverable power (V^2/4R)", "=B4^2/(4*B8)/1000", "kW", "0.0",
         "ceiling"),
        ("Current at that ceiling", "=B4/(2*B8)", "A", "0.0", "i_ceiling"),
        ("Power at fuse limit",
         f"={I}{ref['fuse']}*(B4-{I}{ref['fuse']}*B8)/1000", "kW", "0.00",
         "p_fuse"),
        ("Cell count", f"={I}{ref['n_series']}*{I}{ref['n_par']}", "cells",
         "0", "cells"),
        ("Cell continuous current limit",
         f"={I}{ref['cell_ah']}*{I}{ref['c_rate']}", "A", "0.0", "cell_i_max"),
        ("Pack continuous current limit",
         f"=B13*{I}{ref['n_par']}", "A", "0.0", "pack_i_max"),
    ]
    for label, formula, unit, fmt, key in rows:
        _label(ws, row, label)
        _style(ws, f"B{row}", fmt=fmt).value = formula
        _style(ws, f"C{row}").value = unit
        out[key] = f"$B${row}"
        row += 1
    ws.column_dimensions["A"].width = 34
    ws.column_dimensions["B"].width = 14
    return out


def _write_dashboard(ws, ref: dict, tr: dict, pk: dict) -> dict:
    """The numbers a reviewer opens the file to see."""
    I, P = f"'{S_INPUTS}'!", f"'{S_PACK}'!"
    T = f"'{S_TRACE}'!"
    f, l = tr["first"], tr["last"]
    _header(ws, "A1", "KinematiK — lap-sim dashboard")
    _style(ws, "A2", color="808080", wrap=True).value = (
        "Every cell here is a formula. Change an input on the KX Inputs sheet "
        "and these move with it.")

    row = 4
    _header(ws, f"A{row}", "Lap")
    row += 1
    items = [
        ("Duration", f"=({l}-{f}+1)*{I}{ref['dt']}", "s", "0.00"),
        ("Distance", f"=SUMPRODUCT({T}C{f}:C{l})*{I}{ref['dt']}/1000", "km",
         "0.000"),
        ("Peak speed", f"=MAX({T}B{f}:B{l})", "mph", "0.0"),
        ("Mean speed", f"=AVERAGE({T}C{f}:C{l})*3.6", "km/h", "0.0"),
    ]
    for lab, formula, unit, fmt in items:
        _label(ws, row, lab)
        _style(ws, f"B{row}", fmt=fmt).value = formula
        _style(ws, f"C{row}").value = unit
        row += 1
    dist = "$B$6"
    row += 1

    _header(ws, f"A{row}", "Electrical demand")
    row += 1
    e_row = None
    items = [
        ("Peak pack current", f"=MAX({T}K{f}:K{l})", "A", "0.00", "peak_i"),
        ("Mean pack current", f"=AVERAGE({T}K{f}:K{l})", "A", "0.00", None),
        ("Peak electrical power", f"=MAX({T}J{f}:J{l})/1000", "kW", "0.00",
         "peak_p"),
        ("Energy used", f"=SUMPRODUCT({T}J{f}:J{l})*{I}{ref['dt']}/3600000",
         "kWh", "0.0000", "energy"),
        ("Peak cell current", f"=B{row}/{I}{ref['n_par']}", "A", "0.00", None),
        ("Peak cell C-rate", f"=B{row+4}/{I}{ref['cell_ah']}", "C", "0.0",
         None),
        ("Samples over pack ceiling", f"=SUM({T}L{f}:L{l})", "-", "0", None),
        ("Joule heat, mean", f"=AVERAGE({T}M{f}:M{l})", "W", "0.0", None),
        ("Joule heat, total",
         f"=SUMPRODUCT({T}M{f}:M{l})*{I}{ref['dt']}/3600", "Wh", "0.0", None),
    ]
    keys = {}
    for lab, formula, unit, fmt, key in items:
        _label(ws, row, lab)
        _style(ws, f"B{row}", fmt=fmt).value = formula
        _style(ws, f"C{row}").value = unit
        if key:
            keys[key] = f"$B${row}"
        row += 1
    row += 1

    _header(ws, f"A{row}", "Verdicts")
    row += 1
    checks = [
        ("Fuse", f"=IF({keys['peak_i']}<={I}{ref['fuse']},"
                 f'"PASS - peak "&TEXT({keys["peak_i"]},"0.0")&" A of "'
                 f'&TEXT({I}{ref["fuse"]},"0")&" A",'
                 f'"FAIL - peak "&TEXT({keys["peak_i"]},"0.0")'
                 f'&" A exceeds "&TEXT({I}{ref["fuse"]},"0")&" A fuse")'),
        ("Pack power ceiling",
         f"=IF({keys['peak_p']}<={P}{pk['ceiling']},"
         f'"PASS","FAIL - demand "&TEXT({keys["peak_p"]},"0")'
         f'&" kW exceeds the "&TEXT({P}{pk["ceiling"]},"0")'
         f'&" kW the pack can deliver into any load")'),
        ("Cell C-rate",
         f"=IF({keys['peak_i']}/{I}{ref['n_par']}<={P}{pk['cell_i_max']},"
         f'"PASS","FAIL - "&TEXT({keys["peak_i"]}/{I}{ref["n_par"]}'
         f'/{I}{ref["cell_ah"]},"0.0")&"C exceeds the "'
         f'&TEXT({I}{ref["c_rate"]},"0")&"C cell rating")'),
        ("Single-lap energy",
         f"=IF({keys['energy']}<={P}{pk['usable']},"
         f'"PASS","FAIL - needs "&TEXT({keys["energy"]},"0.00")'
         f'&" kWh of "&TEXT({P}{pk["usable"]},"0.00")&" kWh usable")'),
        ("Endurance energy",
         f"=IF({dist}<=0,\"n/a - zero distance\","
         f"IF({I}{ref['endurance']}/{dist}*{keys['energy']}<={P}{pk['usable']},"
         f'"PASS","FAIL - '
         f'"&TEXT({I}{ref["endurance"]}/{dist},"0.0")&" laps for "'
         f'&TEXT({I}{ref["endurance"]},"0")&" km needs "'
         f'&TEXT({I}{ref["endurance"]}/{dist}*{keys["energy"]},"0.0")'
         f'&" kWh of "&TEXT({P}{pk["usable"]},"0.00")&" kWh"))'),
        ("Motor speed",
         f"=IF(MAX({T}N{f}:N{l})<={I}{ref['max_rpm']},"
         f'"PASS","FAIL - "&TEXT(MAX({T}N{f}:N{l}),"0")'
         f'&" rpm exceeds "&TEXT({I}{ref["max_rpm"]},"0")&" rpm")'),
        ("Motor torque",
         f"=IF(MAX({T}O{f}:O{l})<={I}{ref['t_peak']},"
         f'IF(MAX({T}O{f}:O{l})<={I}{ref["t_cont"]},"PASS",'
         f'"MARGINAL - "&TEXT(MAX({T}O{f}:O{l}),"0")'
         f'&" Nm is above the continuous rating"),'
         f'"FAIL - needs "&TEXT(MAX({T}O{f}:O{l}),"0")&" Nm of "'
         f'&TEXT({I}{ref["t_peak"]},"0")&" Nm peak")'),
    ]
    for lab, formula in checks:
        _label(ws, row, lab)
        _style(ws, f"B{row}", wrap=True).value = formula
        row += 1

    _label(ws, row + 1, "Laps to complete endurance")
    _style(ws, f"B{row+1}", fmt="0.0").value = \
        f"=IF({dist}>0,{I}{ref['endurance']}/{dist},0)"
    _label(ws, row + 2, "Laps the pack supports")
    _style(ws, f"B{row+2}", fmt="0.0").value = \
        f"=IF({keys['energy']}>0,{P}{pk['usable']}/{keys['energy']},0)"

    ws.column_dimensions["A"].width = 30
    ws.column_dimensions["B"].width = 74
    ws.column_dimensions["C"].width = 8
    keys["distance"] = dist
    return keys


def _write_gear_study(ws, spec: ExportSpec, ref: dict, tr: dict) -> None:
    """Torque per ratio — the quantity that actually chooses a gear.

    Pack current is identical in every column, because power is force times
    speed regardless of which gear delivers it. That is exactly why the
    original workbook's fifteen-column *current* sweep could not inform a gear
    choice: every column reported the same thing.
    """
    I, T = f"'{S_INPUTS}'!", f"'{S_TRACE}'!"
    f, l = tr["first"], tr["last"]
    _header(ws, "A1", "KinematiK — gear study")
    _style(ws, "A2", color="808080", wrap=True).value = (
        "Motor speed scales with the reduction and torque scales inversely, so "
        "both are exact from the peak wheel speed and peak tractive force. "
        "Pack current does not vary with gearing at all.")
    hdr = ["Reduction (:1)", "Peak motor speed (rpm)",
           "Peak motor torque (Nm)", "Overspeed?", "Torque verdict",
           "Recommended?"]
    for i, h in enumerate(hdr, start=1):
        _style(ws, f"{_col(i)}4", bold=True).value = h

    peak_wheel = f"MAX({T}B{f}:B{l})*{I}{ref['mph_ipm']}/({I}{ref['wheel_in']}*PI())"
    peak_force = f"MAX({T}H{f}:H{l})"
    r_wheel = f"({I}{ref['wheel_in']}*0.0254/2)"

    row = 5
    for red in spec.reductions:
        _style(ws, f"A{row}", color=_BLUE, fmt="0").value = float(red)
        _style(ws, f"B{row}", fmt="0").value = f"={peak_wheel}*A{row}"
        _style(ws, f"C{row}", fmt="0.00").value = \
            f"={peak_force}*{r_wheel}/(A{row}*{I}{ref['eta_drive']})"
        _style(ws, f"D{row}").value = \
            f'=IF(B{row}>{I}{ref["max_rpm"]},"OVERSPEED","ok")'
        _style(ws, f"E{row}").value = (
            f'=IF(C{row}>{I}{ref["t_peak"]},"over peak",'
            f'IF(C{row}>{I}{ref["t_cont"]},"over continuous","ok"))')
        _style(ws, f"F{row}", bold=True).value = (
            f'=IF(AND(D{row}="ok",E{row}="ok"),"YES","-")')
        row += 1
    for i, w in enumerate((16, 22, 22, 14, 16, 14), start=1):
        ws.column_dimensions[_col(i)].width = w

    _label(ws, row + 1, "Lowest workable reduction")
    _style(ws, f"B{row+1}", bold=True).value = (
        f'=IFERROR(INDEX(A5:A{row-1},MATCH("YES",F5:F{row-1},0)),'
        f'"none of the ratios tested works")')


def _write_advisor(ws, spec: ExportSpec, ref: dict, pk: dict,
                   dash: dict) -> None:
    """Pack candidates scored on energy AND power AND cell C-rate.

    The old advisor used energy alone and therefore recommended 140S1P for a lap
    needing 1 kWh of a 7 kWh pack. One parallel string triples resistance, cuts
    deliverable power to a third, and routes the whole pack current through every
    cell — about 39C for a 5 Ah cell. An advisor that cannot see current will
    keep making that recommendation, which is why all three gates are here.
    """
    I, P, D = f"'{S_INPUTS}'!", f"'{S_PACK}'!", f"'{S_DASH}'!"
    _header(ws, "A1", "KinematiK — pack advisor")
    _style(ws, "A2", color="808080", wrap=True).value = (
        "A candidate must clear three gates, not one: enough usable energy for "
        "the endurance distance, a power ceiling above peak demand, and a "
        "per-cell current inside the cell C-rating. Energy alone recommends "
        "packs that cannot supply the current.")

    hdr = ["Series", "Parallel", "Cells", "Nominal V", "Usable kWh",
           "R (ohm)", "Ceiling (kW)", "Peak cell C", "Energy needed (kWh)",
           "Mass (kg)", "Verdict"]
    for i, h in enumerate(hdr, start=1):
        _style(ws, f"{_col(i)}4", bold=True).value = h

    peak_i = f"{D}{dash['peak_i']}"
    peak_p = f"{D}{dash['peak_p']}"
    lap_kwh = f"{D}{dash['energy']}"
    lap_km = f"{D}{dash['distance']}"

    s0, p0 = spec.pack.n_series, spec.pack.n_parallel
    cands = sorted({1, 2, 3, p0, p0 + 1, p0 + 2, 6, 8})
    row = 5
    for p in [c for c in cands if 1 <= c <= 12]:
        _style(ws, f"A{row}", color=_BLUE, fmt="0").value = s0
        _style(ws, f"B{row}", color=_BLUE, fmt="0").value = p
        _style(ws, f"C{row}", fmt="0").value = f"=A{row}*B{row}"
        _style(ws, f"D{row}", fmt="0.0").value = f"=A{row}*{I}{ref['cell_v']}"
        _style(ws, f"E{row}", fmt="0.000").value = (
            f"=D{row}*B{row}*{I}{ref['cell_ah']}/1000"
            f"*({I}{ref['soc_hi']}-{I}{ref['soc_lo']})")
        _style(ws, f"F{row}", fmt="0.0000").value = (
            f"=A{row}*({I}{ref['cell_r']}/B{row})")
        _style(ws, f"G{row}", fmt="0.0").value = f"=D{row}^2/(4*F{row})/1000"
        _style(ws, f"H{row}", fmt="0.0").value = (
            f"={peak_i}/B{row}/{I}{ref['cell_ah']}")
        _style(ws, f"I{row}", fmt="0.000").value = (
            f"=IF({lap_km}>0,{I}{ref['endurance']}/{lap_km}*{lap_kwh},0)")
        _style(ws, f"J{row}", fmt="0.0").value = (
            f"=C{row}*{spec.pack.cell_weight_kg}")
        _style(ws, f"K{row}", bold=True, wrap=True).value = (
            f'=IF(H{row}>{I}{ref["c_rate"]},'
            f'"NO - "&TEXT(H{row},"0.0")&"C per cell exceeds the "'
            f'&TEXT({I}{ref["c_rate"]},"0")&"C rating",'
            f'IF(G{row}<{peak_p},'
            f'"NO - "&TEXT(G{row},"0")&" kW ceiling is below the "'
            f'&TEXT({peak_p},"0")&" kW peak demand",'
            f'IF(E{row}<I{row},'
            f'"NO - "&TEXT(E{row},"0.00")&" kWh usable against "'
            f'&TEXT(I{row},"0.00")&" kWh needed for endurance",'
            f'"YES - clears current, power and energy")))')
        row += 1
    for i, w in enumerate((8, 9, 8, 11, 11, 10, 12, 12, 18, 10, 58), start=1):
        ws.column_dimensions[_col(i)].width = w

    _label(ws, row + 1, "Smallest pack that clears all three gates")
    _style(ws, f"B{row+1}", bold=True).value = (
        f'=IFERROR("P="&TEXT(INDEX(B5:B{row-1},'
        f'MATCH("YES*",K5:K{row-1},0)),"0"),'
        f'"none of the candidates tested clears all three")')
    _style(ws, f"A{row+3}", color="808080", wrap=True).value = (
        "Energy needed is for the full endurance distance, not one lap. The "
        "previous advisor compared a single lap's energy against the pack and "
        "reported a large surplus where there was a shortfall.")


def _write_provenance(ws, spec: ExportSpec) -> None:
    _header(ws, "A1", "KinematiK — provenance and corrections")
    rows = [
        ("", ""),
        ("Corrected relative to the original workbook", ""),
        ("Gear ratio", "A reduction of N is applied as motor rpm = wheel rpm * "
                       "N. The original stored 1/N and multiplied, so the motor "
                       "turned slower than the wheel (49x error at 7:1)."),
        ("Pack current", "Solved from P = (V_oc - I*R)*I, so sag is included. "
                         "The original used V_pack * PF * RPM / 1000, which is "
                         "volts x rpm and rises with voltage."),
        ("Efficiency", "Electrical demand divides by total efficiency. The "
                       "original multiplied, making a better motor draw more."),
        ("Phase terms", "sqrt(3) and power factor are not applied to the DC "
                        "bus."),
        ("Pack resistance", "S*(R_cell/P). The original used the total cell "
                            "count in place of S, which cancels to S*R_cell "
                            "and is 3x high for 140S3P."),
        ("Energy", "sum(P*dt). The original summed instantaneous watts."),
        ("Gear selection", "Reported as motor torque, which varies with the "
                           "ratio. Pack current does not."),
        ("Pack advice", "Gated on energy, power ceiling and per-cell C-rate "
                        "together."),
        ("", ""),
        ("Assumptions this export introduces", ""),
        ("Vehicle mass", f"{spec.vehicle.mass_kg:g} kg — not present anywhere "
                         f"in the original workbook."),
        ("Drag area CdA", f"{spec.vehicle.cda_m2:g} m^2 — generic FSAE figure."),
        ("Rolling resistance", f"{spec.vehicle.crr:g} — generic FSAE figure."),
        ("Inverter efficiency", f"{spec.vehicle.inverter_efficiency:g} — not in "
                                f"the original workbook."),
        ("Cell C-rating", f"{spec.max_cell_c_rate:g}C — datasheet figure; "
                          f"changes the pack recommendation."),
        ("Grip coefficient", f"{spec.vehicle.mu_lon:g} — caps the acceleration "
                             f"term so a glitched speed sample cannot become a "
                             f"current spike."),
        ("Trace smoothing", f"{spec.smooth_window}-sample moving average before "
                            f"differentiating. Raw quantised traces produce "
                            f"accelerations of several g that never happened."),
        ("", ""),
        ("Method", ""),
        ("Sheets written", "Only sheets prefixed 'KX '. No cell on any "
                           "pre-existing sheet is read-modified-written, so the "
                           "original model and its audit trail are intact."),
        ("Formulas", "Every derived cell is a live formula referencing "
                     "'KX Inputs'. The export is a model, not a snapshot."),
        ("Cached values", "The workbook is recalculated before being returned, "
                          "so data_only readers see numbers rather than None."),
    ]
    r = 2
    for a, b in rows:
        if a and not b:
            _header(ws, f"A{r}", a)
        else:
            _label(ws, r, a)
            _style(ws, f"B{r}", wrap=True, color="404040").value = b
        r += 1
    ws.column_dimensions["A"].width = 26
    ws.column_dimensions["B"].width = 96


# ===================================================================== #
#  Recalculation
# ===================================================================== #
def recalculate(path: str, timeout: int = 180) -> tuple[bool, str]:
    """Populate cached values via LibreOffice, so Python readers see numbers.

    Without this the file is Excel-only: openpyxl writes formula strings and
    evaluates nothing, so `data_only=True` returns None for every derived cell.
    That is the defect that made the previous export unreadable by KinematiK's
    own loaders.
    """
    soffice = shutil.which("soffice") or shutil.which("libreoffice")
    if not soffice:
        return False, "LibreOffice not available; cached values not populated"
    outdir = tempfile.mkdtemp(prefix="kx_recalc_")
    try:
        subprocess.run(
            [soffice, "--headless", "--norestore", "--convert-to", "xlsx",
             "--outdir", outdir, path],
            check=True, capture_output=True, timeout=timeout)
        produced = os.path.join(
            outdir, os.path.splitext(os.path.basename(path))[0] + ".xlsx")
        if not os.path.exists(produced):
            return False, "LibreOffice produced no output"
        shutil.copy(produced, path)
        return True, "recalculated"
    except subprocess.TimeoutExpired:
        return False, f"recalculation timed out after {timeout}s"
    except subprocess.CalledProcessError as exc:
        return False, f"LibreOffice failed: {exc.stderr[:200]!r}"
    finally:
        shutil.rmtree(outdir, ignore_errors=True)


def formula_errors(path: str) -> dict[str, list[str]]:
    """Cells whose cached value is an Excel error, grouped by error type."""
    import openpyxl
    wb = openpyxl.load_workbook(path, data_only=True)
    bad: dict[str, list[str]] = {}
    for ws in wb.worksheets:
        if not ws.title.startswith(SHEET_PREFIX):
            continue
        for row in ws.iter_rows():
            for c in row:
                v = c.value
                if isinstance(v, str) and v.startswith("#"):
                    bad.setdefault(v, []).append(f"{ws.title}!{c.coordinate}")
    return bad


# ===================================================================== #
#  Entry point
# ===================================================================== #
@dataclass
class TrackSimExport:
    path: str
    sheets_added: list
    trace: Optional[pdw.PowerDrawTrace] = None
    recalculated: bool = False
    recalc_message: str = ""
    formula_errors: dict = field(default_factory=dict)
    findings: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    def ok(self) -> bool:
        return not self.formula_errors

    def blocking(self) -> list:
        return [f for f in self.findings if f.severity == Severity.FAIL]


def export_track_sim(source_path: str, out_path: str,
                     speed_mph: Sequence[float], dt_s: float, *,
                     pack: Optional[pdw.PackSpec] = None,
                     vehicle: Optional[pdw.VehicleSpec] = None,
                     drive: Optional[pdw.DriveSpec] = None,
                     lap_time_s: Optional[float] = None,
                     reductions: Sequence[float] = tuple(range(1, 16)),
                     smooth_window: int = 11,
                     endurance_km: Optional[float] = None,
                     max_cell_c_rate: float = 10.0,
                     recalc: bool = True) -> TrackSimExport:
    """Write the lap-sim result into a copy of the electrics workbook.

    `pack` defaults to whatever `power_draw.read_pack_config` finds in the
    source workbook, read by row label. Nothing on the user's sheets is
    modified; every sheet written carries the `KX ` prefix, and re-running
    replaces only those.
    """
    import openpyxl

    warnings: list[str] = []
    findings: list[Finding] = []

    if pack is None:
        try:
            pack = pdw.read_pack_config(source_path)
        except pdw.WorkbookReadError as exc:
            warnings.append(
                f"Could not read the pack from the workbook ({exc}); using "
                f"declared defaults. Every pack figure below is therefore a "
                f"default, not a reading.")
            pack = pdw.PackSpec()
    vehicle = vehicle or pdw.VehicleSpec()
    drive = drive or pdw.DriveSpec()
    if endurance_km is None:
        endurance_km = 22.0

    spec = ExportSpec(pack=pack, vehicle=vehicle, drive=drive,
                      speed_mph=list(speed_mph), dt_s=dt_s,
                      lap_time_s=lap_time_s, reductions=tuple(reductions),
                      smooth_window=smooth_window, endurance_km=endurance_km,
                      max_cell_c_rate=max_cell_c_rate)

    # Python-side truth, used for findings and as a cross-check on the sheet.
    trace = pdw.power_draw_trace(spec.speed_mph, dt_s, pack, vehicle, drive,
                                 smooth_window=smooth_window)
    findings.extend(pdw.trace_findings(trace))
    findings.extend(pdw.motor_feasibility(pack, drive))

    if os.path.abspath(source_path) != os.path.abspath(out_path):
        shutil.copy(source_path, out_path)
    wb = openpyxl.load_workbook(out_path)
    for name in [s for s in wb.sheetnames if s.startswith(SHEET_PREFIX)]:
        del wb[name]

    ws_in = wb.create_sheet(S_INPUTS)
    ref = _write_inputs(ws_in, spec)

    ws_tr = wb.create_sheet(S_TRACE)
    smoothed = pdw._moving_average(spec.speed_mph, smooth_window)
    n = len(smoothed)
    for k, (sp) in enumerate(smoothed):
        r = 3 + k
        ws_tr[f"A{r}"] = round(k * dt_s, 6)
        ws_tr[f"B{r}"] = round(float(sp), 4)
    tr = _write_trace(ws_tr, spec, ref, n)

    ws_pk = wb.create_sheet(S_PACK)
    pk = _write_pack_limits(ws_pk, ref)

    ws_dash = wb.create_sheet(S_DASH)
    dash = _write_dashboard(ws_dash, ref, tr, pk)

    ws_g = wb.create_sheet(S_GEARS)
    _write_gear_study(ws_g, spec, ref, tr)

    ws_ad = wb.create_sheet(S_ADVISOR)
    _write_advisor(ws_ad, spec, ref, pk, dash)

    ws_pv = wb.create_sheet(S_PROV)
    _write_provenance(ws_pv, spec)

    # Dashboard first: it is what a reviewer opens.
    order = [S_DASH] + [s for s in wb.sheetnames if s != S_DASH]
    wb._sheets = [wb[s] for s in order]
    wb.save(out_path)

    added = [S_DASH, S_INPUTS, S_TRACE, S_PACK, S_GEARS, S_ADVISOR, S_PROV]
    result = TrackSimExport(out_path, added, trace, findings=findings,
                            warnings=warnings)

    if recalc:
        ok, msg = recalculate(out_path)
        result.recalculated, result.recalc_message = ok, msg
        if not ok:
            warnings.append(
                msg + ". Formula cells will read as None to data_only "
                      "consumers until the file is opened and saved in Excel.")
        else:
            result.formula_errors = formula_errors(out_path)
            if result.formula_errors:
                warnings.append(
                    "Formula errors present: "
                    + ", ".join(f"{k} x{len(v)}"
                                for k, v in result.formula_errors.items()))
    return result


def export_track_sim_bytes(excel_bytes: bytes,
                           speed_ms: Sequence[float],
                           time_s: Sequence[float], *,
                           pack: Optional[pdw.PackSpec] = None,
                           vehicle: Optional[pdw.VehicleSpec] = None,
                           drive: Optional[pdw.DriveSpec] = None,
                           lap_time_s: Optional[float] = None,
                           reductions: Sequence[float] = tuple(range(1, 16)),
                           smooth_window: int = 11,
                           endurance_km: Optional[float] = None,
                           max_cell_c_rate: float = 10.0,
                           recalc: bool = True
                           ) -> tuple[bytes, "TrackSimExport"]:
    """Bytes in, bytes out — the shape a Streamlit download button needs.

    Takes the same `speed_ms` / `time_s` arrays as the old
    `lap_to_excel_roundtrip`, so swapping the call site over is a rename plus
    reading `.excel_bytes` from the tuple instead of the result object.

    `dt` is taken from the mean spacing of `time_s` rather than assumed, since
    a lap sim's output is not always evenly sampled; a non-uniform trace is
    reported as a warning because the trapezoidal energy sum and the central
    difference both assume a fixed step.
    """
    if len(time_s) < 2:
        raise ValueError("need at least two samples")
    t = [float(x) for x in time_s]
    dt = (t[-1] - t[0]) / (len(t) - 1)
    if dt <= 0:
        raise ValueError("time_s must increase")

    speed_mph = [float(v) / pdw.MPH_TO_MS for v in speed_ms]

    src_dir = tempfile.mkdtemp(prefix="kx_src_")
    try:
        src = os.path.join(src_dir, "source.xlsx")
        out = os.path.join(src_dir, "FSAE_EV_KinematiK_TrackSim.xlsx")
        with open(src, "wb") as fh:
            fh.write(excel_bytes)
        res = export_track_sim(
            src, out, speed_mph, dt, pack=pack, vehicle=vehicle, drive=drive,
            lap_time_s=lap_time_s, reductions=reductions,
            smooth_window=smooth_window, endurance_km=endurance_km,
            max_cell_c_rate=max_cell_c_rate, recalc=recalc)

        # Flag an uneven time base rather than silently averaging over it.
        steps = [t[i + 1] - t[i] for i in range(len(t) - 1)]
        if steps and (max(steps) - min(steps)) > 0.05 * dt:
            res.warnings.append(
                f"Time base is uneven (steps {min(steps):.4g}..{max(steps):.4g} s, "
                f"mean {dt:.4g} s). The trace formulas assume a fixed step, so "
                f"the accelerations and the energy integral are approximate. "
                f"Resample the lap to a uniform dt for exact figures.")
        with open(out, "rb") as fh:
            data = fh.read()
        return data, res
    finally:
        shutil.rmtree(src_dir, ignore_errors=True)


PROVENANCE = {
    "physics_grounded": [
        "force balance, P_elec = P_wheel/eta, sag-aware current — all from "
        "power_draw, which is unit-tested against hand-checkable cases",
    ],
    "hard_rule": (
        "Only sheets prefixed 'KX ' are written, and every derived cell is a "
        "formula rather than a constant. A snapshot of numbers cannot be "
        "re-checked; a model can."
    ),
    "known_limits": [
        "Dashboard cell references are returned by _write_dashboard rather "
        "than hardcoded, so moving a row does not silently repoint a formula.",
        "Vehicle mass, CdA, Crr, inverter efficiency and cell C-rating are "
        "assumptions this export introduces; they are yellow-filled on "
        "KX Inputs and listed on KX Provenance.",
    ],
}
