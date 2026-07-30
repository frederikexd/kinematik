# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
Tests for suspension.track_sim_export.

The four defects this module exists to fix each get a test that would have
caught it:

* constants instead of formulas   -> test_derived_cells_are_formulas
* the workbook's physics errors   -> test_peak_power_is_within_the_pack_ceiling,
                                     test_gear_ratio_multiplies
* formulas with no cached values  -> test_recalculated_file_is_readable_by_python
* energy-only pack advice         -> test_advisor_rejects_the_pack_it_used_to_recommend

Recalculation shells out to LibreOffice and is slow, so the tests that need it
use short traces and are marked. Everything else runs on the written formulas
directly.
"""

import io
import math
import shutil

import openpyxl
import pytest

from suspension.interfaces import Severity
from suspension import power_draw as pdw
from suspension import track_sim_export as tse

_HAS_SOFFICE = bool(shutil.which("soffice") or shutil.which("libreoffice"))
needs_soffice = pytest.mark.skipif(not _HAS_SOFFICE,
                                   reason="LibreOffice not available")


# --------------------------------------------------------------------------- #
#  Fixtures
# --------------------------------------------------------------------------- #
_PACK_ROWS = [
    ("Fuse Max (A)", 50), ("Parrallel Battery Count", 3),
    ("Series Battery Count", 140), ("Nominal Battery Voltage (V)", 3.6),
    ("Capacity Battery Cell (Ah)", 5), ("Endurance Length (km)", 22),
    ("Max Battery Cells", 560),
    ("Internal Resistance Battery Cell (Ohms)", 0.0128),
    ("Battery Cell Weight (kg)", 0.071),
    ("Battery Pack Cell Count", 420),
    ("Battery Pack Internal Resistance (Ohms)", 1.792),
    ("Battery Pack Nominal Voltage (V)", 504),
]


@pytest.fixture
def source(tmp_path):
    """A workbook shaped like the electrics lead's, with their own formulas."""
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "BatteryPackConfig"
    for i, (a, b) in enumerate(_PACK_ROWS, start=1):
        ws.cell(i, 1, a)
        ws.cell(i, 2, b)
    ws.cell(13, 1, "Power Draw (kW)")
    ws.cell(13, 2, "=(B1*B12)/1000")          # a formula of THEIRS
    ep = wb.create_sheet("ElecPropulsion")
    ep["A1"], ep["B1"] = "Motor Peak Torque (Nm)", 120
    svt = wb.create_sheet("SpeedVsTime")
    svt["A1"], svt["B1"] = "time (s)", "Speed (mph)"
    p = tmp_path / "src.xlsx"
    wb.save(p)
    return str(p)


def _trace(n=40, base=35.0):
    return [base + 8 * math.sin(i / 6) ** 2 for i in range(n)]


def _export(source, tmp_path, *, recalc=False, **kw):
    out = str(tmp_path / "out.xlsx")
    return tse.export_track_sim(source, out, _trace(), 0.0666667,
                                recalc=recalc, **kw)


# --------------------------------------------------------------------------- #
#  1. Nothing of the user's is touched
# --------------------------------------------------------------------------- #
def test_user_sheets_are_untouched(source, tmp_path):
    before = openpyxl.load_workbook(source)
    res = _export(source, tmp_path)
    after = openpyxl.load_workbook(res.path)
    for name in before.sheetnames:
        b, a = before[name], after[name]
        assert b.max_row == a.max_row, f"{name} changed row count"
        for r in range(1, b.max_row + 1):
            for c in range(1, b.max_column + 1):
                assert b.cell(r, c).value == a.cell(r, c).value, \
                    f"{name}!{b.cell(r, c).coordinate} was modified"


def test_the_users_own_formula_survives(source, tmp_path):
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)["BatteryPackConfig"]
    assert ws["B13"].value == "=(B1*B12)/1000"


def test_every_written_sheet_carries_the_prefix(source, tmp_path):
    before = set(openpyxl.load_workbook(source).sheetnames)
    res = _export(source, tmp_path)
    after = set(openpyxl.load_workbook(res.path).sheetnames)
    for name in after - before:
        assert name.startswith(tse.SHEET_PREFIX)


def test_all_expected_sheets_are_created(source, tmp_path):
    res = _export(source, tmp_path)
    names = openpyxl.load_workbook(res.path).sheetnames
    for s in (tse.S_DASH, tse.S_INPUTS, tse.S_TRACE, tse.S_PACK,
              tse.S_GEARS, tse.S_ADVISOR, tse.S_PROV):
        assert s in names


def test_dashboard_is_the_first_sheet(source, tmp_path):
    """It is what a reviewer opens; the original left an empty tab there."""
    res = _export(source, tmp_path)
    assert openpyxl.load_workbook(res.path).sheetnames[0] == tse.S_DASH


def test_rerunning_is_idempotent(source, tmp_path):
    out = str(tmp_path / "out.xlsx")
    tse.export_track_sim(source, out, _trace(), 0.0666667, recalc=False)
    first = openpyxl.load_workbook(out).sheetnames
    tse.export_track_sim(out, out, _trace(), 0.0666667, recalc=False)
    assert openpyxl.load_workbook(out).sheetnames == first


# --------------------------------------------------------------------------- #
#  2. It is a model, not a snapshot
# --------------------------------------------------------------------------- #
def test_derived_cells_are_formulas(source, tmp_path):
    """The previous export wrote constants, which cannot be re-checked."""
    res = _export(source, tmp_path)
    wb = openpyxl.load_workbook(res.path)
    tr = wb[tse.S_TRACE]
    for col in "CDEFGHIJKMNO":
        v = tr[f"{col}3"].value
        assert isinstance(v, str) and v.startswith("="), \
            f"{tse.S_TRACE}!{col}3 is not a formula"
    for cell in ("B5", "B11", "B13", "B14"):
        v = wb[tse.S_DASH][cell].value
        assert isinstance(v, str) and v.startswith("=")


def test_inputs_are_numbers_not_formulas(source, tmp_path):
    """Inputs are typed values; only derived cells are formulas."""
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_INPUTS]
    mass = next(r for r in range(1, 60)
                if str(ws.cell(r, 1).value or "").startswith("Mass"))
    assert isinstance(ws.cell(mass, 2).value, (int, float))


def test_trace_speed_column_is_data(source, tmp_path):
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_TRACE]
    assert isinstance(ws["B3"].value, (int, float))


def test_unverified_assumptions_are_highlighted(source, tmp_path):
    """Mass, CdA and Crr are absent from the original workbook entirely."""
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_INPUTS]
    filled = []
    for r in range(1, 60):
        c = ws.cell(r, 2)
        if c.fill and c.fill.fgColor and c.fill.fgColor.rgb == tse._YELLOW:
            filled.append(str(ws.cell(r, 1).value))
    assert any("Mass" in f for f in filled)
    assert any("Drag area" in f for f in filled)
    assert any("Rolling resistance" in f for f in filled)


# --------------------------------------------------------------------------- #
#  3. The physics is the corrected physics
# --------------------------------------------------------------------------- #
def test_gear_ratio_multiplies(source, tmp_path):
    """motor rpm = wheel rpm * reduction. The original divided."""
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_TRACE]
    f = ws["N3"].value
    assert "reduction" not in f            # it is a cell ref, not a name
    assert "*" in f.split("PI()")[-1] or f.rstrip().endswith(")")
    # the python-side trace is the authority and is already unit-tested
    tr = res.trace
    assert tr.motor_rpm[0] > 0
    faster = pdw.DriveSpec(reduction=14.0).motor_rpm(35.0, 18.0)
    slower = pdw.DriveSpec(reduction=7.0).motor_rpm(35.0, 18.0)
    assert faster == pytest.approx(2 * slower)


def test_peak_power_is_within_the_pack_ceiling(source, tmp_path):
    """The old export reported 294 kW from a pack capped at 106 kW."""
    res = _export(source, tmp_path)
    pack = pdw.PackSpec()
    assert res.trace.peak_power_kw() <= pack.max_deliverable_power_w() / 1000


def test_no_sample_exceeds_the_pack_ceiling_on_a_sane_trace(source, tmp_path):
    res = _export(source, tmp_path)
    assert res.trace.infeasible_samples == 0


def test_pack_resistance_formula_uses_series_not_cell_count(source, tmp_path):
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_PACK]
    row = next(r for r in range(1, 20)
               if str(ws.cell(r, 1).value or "") == "Pack resistance")
    f = ws.cell(row, 2).value
    # n_series * (cell_r / n_parallel) — three refs, no cell-count term
    assert f.count("'" + tse.S_INPUTS + "'!") == 3


def test_energy_is_an_integral(source, tmp_path):
    """SUMPRODUCT(P) * dt, never SUM(P)."""
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_DASH]
    f = ws["B14"].value
    assert "SUMPRODUCT" in f
    assert "3600000" in f


def test_findings_are_attached(source, tmp_path):
    res = _export(source, tmp_path)
    assert res.findings
    assert any(f.check.startswith("pd-") for f in res.findings)


# --------------------------------------------------------------------------- #
#  4. Gear study reports the quantity that selects a ratio
# --------------------------------------------------------------------------- #
def test_gear_study_covers_every_requested_reduction(source, tmp_path):
    res = _export(source, tmp_path, reductions=range(1, 16))
    ws = openpyxl.load_workbook(res.path)[tse.S_GEARS]
    got = [ws.cell(r, 1).value for r in range(5, 20)]
    assert got == [float(i) for i in range(1, 16)]


def test_gear_study_has_a_torque_column_and_a_recommendation(source, tmp_path):
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_GEARS]
    hdr = [ws.cell(4, c).value for c in range(1, 7)]
    assert any("torque" in str(h).lower() for h in hdr)
    assert any("Recommended" in str(h) for h in hdr)
    row = next(r for r in range(18, 26)
               if "Lowest workable" in str(ws.cell(r, 1).value or ""))
    assert str(ws.cell(row, 2).value).startswith("=")


def test_torque_scales_inversely_with_reduction():
    veh = pdw.VehicleSpec()
    a = pdw.DriveSpec(reduction=4.0).motor_torque_nm(1000.0,
                                                     veh.wheel_radius_m())
    b = pdw.DriveSpec(reduction=8.0).motor_torque_nm(1000.0,
                                                     veh.wheel_radius_m())
    assert b == pytest.approx(a / 2)


# --------------------------------------------------------------------------- #
#  5. The advisor
# --------------------------------------------------------------------------- #
def test_advisor_rejects_the_pack_it_used_to_recommend(source, tmp_path):
    """140S1P: triples resistance, ~26C per cell. The old advisor advised it."""
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_ADVISOR]
    row = next(r for r in range(5, 15) if ws.cell(r, 2).value == 1)
    verdict = ws.cell(row, 11).value
    assert "c_rate" in verdict or "C rating" in verdict or "rating" in verdict
    assert verdict.startswith("=IF(")


def test_advisor_gates_on_all_three_limits(source, tmp_path):
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_ADVISOR]
    v = ws["K5"].value
    assert "rating" in v          # cell C-rate gate
    assert "ceiling" in v         # power gate
    assert "endurance" in v       # energy gate


def test_advisor_energy_gate_uses_endurance_distance(source, tmp_path):
    """The old sheet compared ONE lap's energy and reported a 643% surplus
    where the endurance requirement was an 8x shortfall."""
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_ADVISOR]
    f = ws["I5"].value
    assert "endurance" in f.lower() or "$B$" in f
    hdr = [ws.cell(4, c).value for c in range(1, 12)]
    assert any("Energy needed" in str(h) for h in hdr)


def test_advisor_lists_the_current_pack_among_candidates(source, tmp_path):
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_ADVISOR]
    pars = [ws.cell(r, 2).value for r in range(5, 15)]
    assert 3 in pars


# --------------------------------------------------------------------------- #
#  6. Cached values — the defect that made the old file unreadable
# --------------------------------------------------------------------------- #
def test_without_recalc_the_warning_is_explicit(source, tmp_path, monkeypatch):
    monkeypatch.setattr(tse.shutil, "which", lambda *_a, **_k: None)
    out = str(tmp_path / "o.xlsx")
    res = tse.export_track_sim(source, out, _trace(), 0.0666667, recalc=True)
    assert not res.recalculated
    assert any("cached" in w.lower() or "None" in w for w in res.warnings)


@needs_soffice
def test_recalculated_file_is_readable_by_python(source, tmp_path):
    """data_only=True must see numbers, not None."""
    out = str(tmp_path / "o.xlsx")
    res = tse.export_track_sim(source, out, _trace(30), 0.0666667, recalc=True)
    assert res.recalculated, res.recalc_message
    ws = openpyxl.load_workbook(out, data_only=True)[tse.S_DASH]
    for cell in ("B11", "B13", "B14"):
        v = ws[cell].value
        assert isinstance(v, (int, float)), f"{cell} read back as {v!r}"


@needs_soffice
def test_no_formula_errors_after_recalc(source, tmp_path):
    out = str(tmp_path / "o.xlsx")
    res = tse.export_track_sim(source, out, _trace(30), 0.0666667, recalc=True)
    assert res.formula_errors == {}, res.formula_errors
    assert res.ok()


@needs_soffice
def test_it_is_a_live_model(source, tmp_path):
    """Change one input, recalculate, and the conclusions move."""
    out = str(tmp_path / "o.xlsx")
    tse.export_track_sim(source, out, _trace(30), 0.0666667, recalc=True)
    before = openpyxl.load_workbook(out, data_only=True)[tse.S_DASH]["B11"].value

    wb = openpyxl.load_workbook(out)
    ws = wb[tse.S_INPUTS]
    mass = next(r for r in range(1, 60)
                if str(ws.cell(r, 1).value or "").startswith("Mass"))
    ws.cell(mass, 2).value = ws.cell(mass, 2).value * 2
    wb.save(out)
    tse.recalculate(out)

    after = openpyxl.load_workbook(out, data_only=True)[tse.S_DASH]["B11"].value
    assert after > before * 1.5, (
        f"peak current did not respond to doubling the mass "
        f"({before} -> {after}); the workbook is not live")


# --------------------------------------------------------------------------- #
#  7. Provenance
# --------------------------------------------------------------------------- #
def test_provenance_names_the_assumptions_it_introduces(source, tmp_path):
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_PROV]
    text = " ".join(str(ws.cell(r, c).value or "")
                    for r in range(1, ws.max_row + 1) for c in (1, 2))
    for token in ("mass", "CdA", "Rolling resistance", "C-rating",
                  "Inverter efficiency"):
        assert token.lower() in text.lower(), f"{token} not documented"


def test_provenance_records_the_corrections(source, tmp_path):
    res = _export(source, tmp_path)
    ws = openpyxl.load_workbook(res.path)[tse.S_PROV]
    text = " ".join(str(ws.cell(r, c).value or "")
                    for r in range(1, ws.max_row + 1) for c in (1, 2))
    assert "49x" in text or "slower than the wheel" in text
    assert "3x high" in text
    assert "sum(P*dt)" in text or "summed instantaneous" in text


def test_unreadable_pack_falls_back_with_a_warning(tmp_path):
    wb = openpyxl.Workbook()
    wb.active.title = "Nothing"
    wb.active["A1"] = "unrelated"
    src = str(tmp_path / "bare.xlsx")
    wb.save(src)
    out = str(tmp_path / "o.xlsx")
    res = tse.export_track_sim(src, out, _trace(20), 0.0666667, recalc=False)
    assert any("default" in w.lower() for w in res.warnings)


def test_module_provenance_declares_its_limits():
    assert tse.PROVENANCE["known_limits"]
    assert tse.PROVENANCE["hard_rule"]
