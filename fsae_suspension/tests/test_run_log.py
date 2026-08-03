# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for the ANSYS run-log ingest / screening / consolidation feature.

Covers the four things that make this trustworthy rather than a filter that
happens to produce a number:
  1. it parses the sheet the aero team actually keeps — banner row above the
     header, renamed columns, units in the header text, blank filler rows,
  2. every screening gate fires on the physics it claims to, and — the part that
     matters — the y+ gate is judged against the row's OWN turbulence model,
  3. the honesty contract holds: nothing is dropped without a reason, a derived
     coefficient is labelled derived, one run is never presented as a mean, and
     the outlier pass refuses to run on too few samples,
  4. the outputs round-trip: workbook, CSV bundle, and the CoeffResult bridge
     into AeroMap.
"""

import csv
import io
import math
import os
import tempfile

import pytest

from suspension.aero import run_log as rl
from suspension.aero.run_log import (
    ScreenConfig, Severity, WallTreatment, RunRow, CaseKey,
    parse_run_log, parse_rows_from_grid, screen, consolidate, process,
    to_coeff_results, write_workbook, write_csv_bundle, consolidated_csv,
    wall_treatment_for, dynamic_pressure, implied_reference_area,
    modified_z_scores, LOG_LAW_INTERSECTION_YPLUS,
    CANONICAL_FIELDS, SETUP_FIELDS, Discretisation,
    discretisation_of, setup_signature,
)


# --------------------------------------------------------------------------- #
#  Fixtures — the sheet as the team actually keeps it
# --------------------------------------------------------------------------- #
BANNER = ["Wings Team Simulation Results"] + [None] * 8 + ["Volume Mesh Metrics"] \
    + [None] * 17

HEADER = [
    "Contributor", "Front or Rear Wing?", "Ride-Height (mm)", "Velocity (m/s)",
    "Desired Y+", "Min Surface Mesh Length", "Max Surface Mesh Length",
    "First Layer Height (m)", "Number of Layers", "Min Orthogonal Quality",
    "Max Skewness", "Max Aspect Ratio", "Viscous Model", "Scheme", "Order",
    "Pseudo Time Step", "Courant Number", "Initialization", "Lift Force (N)",
    "Lift Coefficient", "Drag Force (N)", "Drag Coefficient", "Max Pressure (Pa)",
    "Min. Pressure (Pa)", "Mass Imbalance (kg/s)", "Average Y+", "Notes",
]

#: A clean run: k-epsilon with y+ in the log layer, healthy mesh, Cp_max ~ 1.
#: Reference area works out to 0.268 m^2 — the same one the real sample implies.
GOOD = [
    "Adriane", "Front Wing", 40, 26.8224, 40, 0.006, 0.012,
    6.2671639751389597e-4, 8, 0.30, 0.70, 1346.37, "k-epsilon", "Simple",
    "Second", "0.5", 20.0, "Standard", -98.22, -0.831694392,
    23.485, 0.19886, 450.2234, -1515.11, 7.327e-6, 45.0, None,
]


def _row(**overrides):
    """A copy of GOOD with named canonical fields overridden by column index."""
    idx = {
        "contributor": 0, "component": 1, "ride_height_mm": 2, "speed_ms": 3,
        "desired_yplus": 4, "min_surface_mesh": 5, "max_surface_mesh": 6,
        "first_layer_height_m": 7, "n_layers": 8, "min_ortho_quality": 9,
        "max_skewness": 10, "max_aspect_ratio": 11, "viscous_model": 12,
        # solver setup — previously unreachable from the fixture
        "scheme": 13, "order": 14, "pseudo_time_step": 15,
        "courant_number": 16, "initialization": 17,
        "lift_force_N": 18, "lift_coeff": 19, "drag_force_N": 20,
        "drag_coeff": 21, "max_pressure_Pa": 22, "min_pressure_Pa": 23,
        "mass_imbalance": 24, "avg_yplus": 25, "notes": 26,
    }
    row = list(GOOD)
    for key, value in overrides.items():
        row[idx[key]] = value
    return row


def grid(*data_rows):
    """A full sheet: banner, header, then the given data rows."""
    return [BANNER, HEADER, *data_rows]


def screen_one(**overrides):
    """Screen a single row built from GOOD and return its Verdict."""
    rows, _, _ = parse_rows_from_grid(grid(_row(**overrides)))
    assert len(rows) == 1
    return screen(rows)[0]


def codes(verdict):
    return {f.code for f in verdict.flags}


# --------------------------------------------------------------------------- #
#  1) Parsing
# --------------------------------------------------------------------------- #
def test_finds_header_beneath_a_banner_row():
    rows, warnings, unmapped = parse_rows_from_grid(grid(_row()))
    assert len(rows) == 1
    r = rows[0]
    assert r.contributor == "Adriane"
    assert r.component == "Front Wing"
    assert r.ride_height_mm == 40
    assert r.viscous_model == "k-epsilon"
    # The banner text itself must not have been read as data.
    assert not any("Wings Team" in (x.contributor or "") for x in rows)


def test_units_in_header_do_not_break_mapping():
    rows, _, _ = parse_rows_from_grid(grid(_row()))
    r = rows[0]
    assert r.speed_ms == pytest.approx(26.8224)          # "Velocity (m/s)"
    assert r.mass_imbalance == pytest.approx(7.327e-6)   # "Mass Imbalance (kg/s)"
    assert r.min_pressure_Pa == pytest.approx(-1515.11)  # "Min. Pressure (Pa)"


def test_desired_and_average_yplus_are_not_confused():
    rows, _, _ = parse_rows_from_grid(grid(_row(desired_yplus=40, avg_yplus=55)))
    assert rows[0].desired_yplus == 40
    assert rows[0].avg_yplus == 55


def test_renamed_columns_still_map():
    header = list(HEADER)
    header[0] = "Engineer"
    header[3] = "Freestream Velocity"
    header[19] = "Cl"
    header[25] = "Wall Y+"
    rows, _, _ = parse_rows_from_grid([BANNER, header, _row()])
    assert rows[0].contributor == "Adriane"
    assert rows[0].speed_ms == pytest.approx(26.8224)
    assert rows[0].lift_coeff == pytest.approx(-0.831694392)
    assert rows[0].avg_yplus == pytest.approx(45.0)


def test_blank_filler_rows_are_dropped():
    empty = [None] * len(HEADER)
    rows, _, _ = parse_rows_from_grid(grid(_row(), empty, empty))
    assert len(rows) == 1


def test_messy_numeric_cells_are_coerced():
    rows, _, _ = parse_rows_from_grid(grid(_row(
        speed_ms="26.8 m/s", lift_force_N="-1,234.5", mass_imbalance="1.33E-5",
        drag_coeff="n/a", min_ortho_quality="0,30")))
    r = rows[0]
    assert r.speed_ms == pytest.approx(26.8)
    assert r.lift_force_N == pytest.approx(-1234.5)
    assert r.mass_imbalance == pytest.approx(1.33e-5)
    assert r.drag_coeff is None                    # placeholder, not a zero
    assert r.min_ortho_quality == pytest.approx(0.30)   # comma decimal


def test_unmapped_columns_are_reported_and_carried_through():
    header = HEADER + ["Cluster Node"]
    rows, _, unmapped = parse_rows_from_grid([BANNER, header, _row() + ["node07"]])
    assert "Cluster Node" in unmapped
    assert rows[0].raw["Cluster Node"] == "node07"     # nothing is lost


def test_parses_csv_and_xlsx_identically(tmp_path):
    csv_path = tmp_path / "runs.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        for r in grid(_row()):
            w.writerow(["" if c is None else c for c in r])
    rows, _, _, _, _ = parse_run_log(str(csv_path))
    assert len(rows) == 1
    assert rows[0].contributor == "Adriane"
    assert rows[0].lift_coeff == pytest.approx(-0.831694392)


def test_parse_accepts_bytes_and_streams(tmp_path):
    text = "\n".join(",".join("" if c is None else str(c) for c in r)
                     for r in grid(_row()))
    rows, _, _, _, _ = parse_run_log(text.encode("utf-8"))
    assert len(rows) == 1
    rows2, _, _, _, _ = parse_run_log(io.BytesIO(text.encode("utf-8")))
    assert len(rows2) == 1


# --------------------------------------------------------------------------- #
#  2) Helpers
# --------------------------------------------------------------------------- #
def test_dynamic_pressure():
    assert dynamic_pressure(26.8224) == pytest.approx(0.5 * 1.225 * 26.8224 ** 2)
    assert dynamic_pressure(None) is None
    assert dynamic_pressure(0) is None


def test_implied_reference_area_recovers_the_normalisation():
    q = dynamic_pressure(26.8224)
    area = implied_reference_area(-98.22, -0.831694392, q)
    assert area == pytest.approx(0.268, rel=1e-3)
    assert implied_reference_area(-98.22, 0.0, q) is None      # no divide by zero
    assert implied_reference_area(None, -0.8, q) is None


@pytest.mark.parametrize("model,expected", [
    ("k-epsilon", WallTreatment.WALL_FUNCTION),
    ("Realizable k-epsilon", WallTreatment.WALL_FUNCTION),
    ("RNG k-e", WallTreatment.WALL_FUNCTION),
    ("Reynolds Stress", WallTreatment.WALL_FUNCTION),
    ("k-omega SST", WallTreatment.RESOLVED),
    ("SST", WallTreatment.RESOLVED),
    ("Spalart-Allmaras", WallTreatment.RESOLVED),
    ("k-omega", WallTreatment.AUTOMATIC),
    ("some in-house closure", WallTreatment.UNKNOWN),
    (None, WallTreatment.UNKNOWN),
])
def test_wall_treatment_classification(model, expected):
    assert wall_treatment_for(model) == expected


def test_modified_z_scores_flags_the_odd_one_out():
    scores = modified_z_scores([1.0, 1.01, 0.99, 1.0, 5.0])
    assert scores[-1] > 3.5
    assert all(s < 3.5 for s in scores[:-1])
    assert modified_z_scores([2.0, 2.0, 2.0]) == [0.0, 0.0, 0.0]
    assert modified_z_scores([]) == []


# --------------------------------------------------------------------------- #
#  3) Screening — each gate fires on the physics it claims
# --------------------------------------------------------------------------- #
def test_clean_row_is_accepted_with_no_flags():
    v = screen_one()
    assert v.accepted
    assert v.flags == []
    assert v.severity == Severity.INFO


def test_yplus_gate_is_judged_against_the_rows_own_model():
    # y+ = 20 is BELOW the log-law intersection for a wall-function closure...
    low = screen_one(viscous_model="k-epsilon", avg_yplus=8.0, desired_yplus=8.0)
    assert not low.accepted
    assert "YPLUS_BAND" in codes(low)

    # ...and the very same y+ is merely marginal for a blended closure.
    auto = screen_one(viscous_model="k-omega", avg_yplus=8.0, desired_yplus=8.0)
    assert auto.accepted

    # y+ = 1 is exactly right for a sublayer-resolving closure.
    sst = screen_one(viscous_model="k-omega SST", avg_yplus=1.0, desired_yplus=1.0)
    assert sst.accepted
    assert "YPLUS_BAND" not in codes(sst)

    # ...and badly wrong for the wall-function one.
    sst_bad = screen_one(viscous_model="k-omega SST", avg_yplus=120.0,
                         desired_yplus=120.0)
    assert not sst_bad.accepted
    assert "YPLUS_BAND" in codes(sst_bad)


def test_yplus_buffer_layer_is_a_warning_not_a_rejection():
    """y+ = 22 with wall functions is poor practice but above the 11.06 floor."""
    v = screen_one(avg_yplus=22.0, desired_yplus=40)
    assert v.accepted
    assert "YPLUS_MARGINAL" in codes(v)
    assert "YPLUS_BAND" not in codes(v)


def test_unknown_turbulence_model_never_rejects_on_yplus():
    v = screen_one(viscous_model="mystery model", avg_yplus=2000.0)
    assert v.accepted                       # warned, not rejected
    assert "YPLUS_EXTREME" in codes(v)


def test_yplus_target_miss_is_flagged():
    v = screen_one(desired_yplus=40, avg_yplus=80)
    assert "YPLUS_TARGET_MISS" in codes(v)
    assert v.derived.yplus_target_miss == pytest.approx(1.0)


def test_inverted_mesh_lengths_are_rejected():
    v = screen_one(min_surface_mesh=0.005, max_surface_mesh=0.004)
    assert not v.accepted
    assert "MESH_LENGTH_INVERTED" in codes(v)


def test_suspicious_mesh_length_ratio_warns():
    v = screen_one(min_surface_mesh=0.006, max_surface_mesh=0.6)
    assert v.accepted
    assert "MESH_LENGTH_RATIO" in codes(v)


def test_mesh_quality_gates():
    assert "ORTHO_QUALITY" in codes(screen_one(min_ortho_quality=0.02))
    assert "ORTHO_QUALITY_LOW" in codes(screen_one(min_ortho_quality=0.15))
    assert "SKEWNESS" in codes(screen_one(max_skewness=0.99))
    assert "SKEWNESS_HIGH" in codes(screen_one(max_skewness=0.92))
    assert not screen_one(min_ortho_quality=0.02).accepted
    assert screen_one(min_ortho_quality=0.15).accepted


def test_aspect_ratio_is_a_warning_by_default():
    """Inflation-layer cells legitimately run high AR; it is a smell, not a fault."""
    v = screen_one(max_aspect_ratio=8000)
    assert v.accepted
    assert "ASPECT_RATIO_HIGH" in codes(v)
    assert not screen_one(max_aspect_ratio=500_000).accepted


def test_mass_imbalance_gate():
    assert "MASS_IMBALANCE" in codes(screen_one(mass_imbalance=5e-3))
    assert not screen_one(mass_imbalance=5e-3).accepted
    assert "MASS_IMBALANCE_HIGH" in codes(screen_one(mass_imbalance=5e-4))
    assert screen_one(mass_imbalance=5e-4).accepted


def test_stagnation_pressure_catches_a_wrong_reference_velocity():
    """Cp_max must sit near 1; 0.25 means the reference conditions are wrong."""
    q = dynamic_pressure(26.8224)
    v = screen_one(max_pressure_Pa=0.25 * q)
    assert not v.accepted
    assert "CP_STAGNATION" in codes(v)
    assert v.derived.cp_max == pytest.approx(0.25, rel=1e-6)

    warn = screen_one(max_pressure_Pa=0.7 * q)
    assert warn.accepted
    assert "CP_STAGNATION_OFF" in codes(warn)


def test_absent_suction_peak_warns_about_the_wall_zone():
    q = dynamic_pressure(26.8224)
    v = screen_one(min_pressure_Pa=-0.1 * q)
    assert "CP_NO_SUCTION" in codes(v)


def test_positive_lift_warns_about_the_sign_convention():
    v = screen_one(lift_force_N=98.22, lift_coeff=0.831694392)
    assert "LIFT_SIGN" in codes(v)
    assert v.accepted                        # a warning: it might be real


def test_row_with_no_result_at_all_is_rejected():
    v = screen_one(lift_force_N=None, lift_coeff=None)
    assert not v.accepted
    assert "NO_RESULT" in codes(v)


def test_scratch_rows_are_excluded_but_reported():
    v = screen_one(contributor="Khalil - Test")
    assert not v.accepted
    assert "TEST_ROW" in codes(v)
    assert "scratch" in v.reason().lower()

    # A word that merely CONTAINS a marker must not trip it.
    assert "TEST_ROW" not in codes(screen_one(contributor="Testarossa Latest"))

    # Configurable down to a warning.
    rows, _, _ = parse_rows_from_grid(grid(_row(contributor="Khalil - Test")))
    v2 = screen(rows, ScreenConfig(reject_test_rows=False))[0]
    assert v2.accepted and "TEST_ROW" in codes(v2)


def test_explicit_non_convergence_is_rejected():
    header = HEADER + ["Converged"]
    rows, _, _ = parse_rows_from_grid([BANNER, header, _row() + ["No"]])
    assert rows[0].converged is False
    assert not screen(rows)[0].accepted


def test_every_rejection_carries_a_reason():
    v = screen_one(min_ortho_quality=0.02, max_skewness=0.99)
    assert not v.accepted
    assert v.reason()                                  # never silent
    assert len(v.reject_codes) == 2


# --------------------------------------------------------------------------- #
#  4) Reference-area consistency — the silent killer
# --------------------------------------------------------------------------- #
def test_a_differently_normalised_row_is_rejected_from_the_group():
    """
    Three runs at one point; one contributor normalised by half the area. Its
    numbers are internally consistent, so only the cross-check can catch it.
    """
    q = dynamic_pressure(26.8224)
    a, b = _row(contributor="A"), _row(contributor="B")
    odd = _row(contributor="C", lift_coeff=-98.22 / (q * 0.134))   # A/2
    verdicts = screen(parse_rows_from_grid(grid(a, b, odd))[0])
    by_who = {v.row.contributor: v for v in verdicts}
    assert by_who["A"].accepted and by_who["B"].accepted
    assert not by_who["C"].accepted
    assert "REF_AREA_MISMATCH" in codes(by_who["C"])


def test_reference_area_is_inferred_and_reported():
    verdicts = screen(parse_rows_from_grid(grid(_row(), _row()))[0])
    cases = consolidate(verdicts)
    assert cases[0].reference_area_m2 == pytest.approx(0.268, rel=1e-3)
    assert "inferred" in cases[0].reference_area_basis


def test_supplied_reference_area_overrides_inference():
    cfg = ScreenConfig(reference_area_m2=0.30, ref_area_tolerance=1.0,
                       ref_area_reject_tolerance=2.0)
    verdicts = screen(parse_rows_from_grid(grid(_row()))[0], cfg)
    cases = consolidate(verdicts, cfg)
    assert cases[0].reference_area_m2 == pytest.approx(0.30)
    assert "supplied" in cases[0].reference_area_basis


def test_missing_drag_coefficient_is_derived_and_labelled():
    """The real sheet leaves Drag Coefficient blank — back it out, say so."""
    verdicts = screen(parse_rows_from_grid(grid(_row(drag_coeff=None)))[0])
    v = verdicts[0]
    assert v.row.drag_coeff is None                     # reported value untouched
    q = dynamic_pressure(26.8224)
    assert v.derived.drag_coeff_derived == pytest.approx(23.485 / (q * 0.268), rel=1e-3)
    case = consolidate(verdicts)[0]
    assert any("derived" in n for n in case.notes)


def test_a_missing_channel_stays_missing_when_it_cannot_be_derived():
    """No velocity => no q => no derivation. It must not be invented."""
    v = screen_one(speed_ms=None, drag_coeff=None)
    assert v.derived.drag_coeff_derived is None
    assert "NO_VELOCITY" in codes(v)


# --------------------------------------------------------------------------- #
#  5) Statistical outlier pass
# --------------------------------------------------------------------------- #
def test_outlier_pass_needs_enough_samples():
    """Rejecting an 'outlier' out of three points is picking a favourite."""
    q = dynamic_pressure(26.8224)

    def run_with(n_normal, odd_cl):
        rows = [_row(contributor=f"N{i}") for i in range(n_normal)]
        # Keep the implied reference area constant so only the VALUE is odd.
        rows.append(_row(contributor="ODD", lift_coeff=odd_cl,
                         lift_force_N=odd_cl * q * 0.268))
        return {v.row.contributor: v for v in screen(parse_rows_from_grid(grid(*rows))[0])}

    few = run_with(2, -1.9)
    assert few["ODD"].accepted                    # 3 samples: pass disabled

    many = run_with(5, -1.9)
    assert not many["ODD"].accepted
    assert "STATISTICAL_OUTLIER" in codes(many["ODD"])
    assert all(many[f"N{i}"].accepted for i in range(5))


def test_outlier_pass_can_be_disabled():
    q = dynamic_pressure(26.8224)
    rows = [_row(contributor=f"N{i}") for i in range(5)]
    rows.append(_row(contributor="ODD", lift_coeff=-1.9,
                     lift_force_N=-1.9 * q * 0.268))
    cfg = ScreenConfig(enable_outlier_pass=False)
    by_who = {v.row.contributor: v for v in screen(parse_rows_from_grid(grid(*rows))[0], cfg)}
    assert by_who["ODD"].accepted


def test_outlier_pass_runs_after_the_physics_gates():
    """An outlier flag should only ever appear on a row that passed everything."""
    q = dynamic_pressure(26.8224)
    rows = [_row(contributor=f"N{i}") for i in range(5)]
    rows.append(_row(contributor="ODD", lift_coeff=-1.9,
                     lift_force_N=-1.9 * q * 0.268))
    verdicts = screen(parse_rows_from_grid(grid(*rows))[0])
    for v in verdicts:
        if "STATISTICAL_OUTLIER" in codes(v):
            assert v.reject_codes == ["STATISTICAL_OUTLIER"]


# --------------------------------------------------------------------------- #
#  6) Consolidation
# --------------------------------------------------------------------------- #
def test_rows_at_one_operating_point_are_averaged():
    a = _row(contributor="A", lift_coeff=-0.80, lift_force_N=-0.80 * 440.7 * 0.268)
    b = _row(contributor="B", lift_coeff=-0.84, lift_force_N=-0.84 * 440.7 * 0.268)
    cases = consolidate(screen(parse_rows_from_grid(grid(a, b))[0]))
    assert len(cases) == 1
    c = cases[0]
    assert c.n_accepted == 2
    assert c.lift_coeff_mean == pytest.approx(-0.82)
    assert c.lift_coeff_min == pytest.approx(-0.84)
    assert c.lift_coeff_max == pytest.approx(-0.80)
    assert c.contributors == ["A", "B"]


def test_different_operating_points_are_kept_apart():
    cases = consolidate(screen(parse_rows_from_grid(grid(
        _row(ride_height_mm=40), _row(ride_height_mm=50),
        _row(component="Rear Wing")))[0]))
    assert len(cases) == 3
    labels = {c.case.label() for c in cases}
    assert any("Rear Wing" in x for x in labels)


def test_near_identical_ride_heights_group_together():
    cases = consolidate(screen(parse_rows_from_grid(grid(
        _row(ride_height_mm=40.0), _row(ride_height_mm=40.1)))[0]))
    assert len(cases) == 1
    assert cases[0].n_accepted == 2


def test_a_single_run_is_never_presented_as_a_mean():
    c = consolidate(screen(parse_rows_from_grid(grid(_row()))[0]))[0]
    assert c.n_accepted == 1
    assert c.lift_coeff_sd is None
    assert "SINGLE RUN" in c.confidence


def test_a_point_where_everything_was_rejected_is_still_reported():
    c = consolidate(screen(parse_rows_from_grid(grid(
        _row(contributor="Test rig", min_ortho_quality=0.01)))[0]))[0]
    assert c.n_accepted == 0
    assert c.n_rejected == 1
    assert c.reject_reasons                    # with the reason attached
    assert "NO DATA" in c.confidence


def test_spread_flags_poor_agreement():
    q = 440.7
    rows = [_row(contributor=f"C{i}", lift_coeff=cl,
                 lift_force_N=cl * q * 0.268)
            for i, cl in enumerate((-0.70, -0.80, -0.90))]
    c = consolidate(screen(parse_rows_from_grid(grid(*rows))[0]))[0]
    assert c.spread_pct == pytest.approx(25.0, rel=1e-2)
    assert "POOR AGREEMENT" in c.confidence


def test_lift_to_drag_is_computed():
    c = consolidate(screen(parse_rows_from_grid(grid(_row()))[0]))[0]
    assert c.lift_to_drag == pytest.approx(abs(-0.831694392 / 0.19886), rel=1e-3)


def test_prefer_latest_per_contributor_is_off_by_default():
    a = _row(contributor="A", lift_coeff=-0.70, lift_force_N=-0.70 * 440.7 * 0.268)
    b = _row(contributor="A", lift_coeff=-0.72, lift_force_N=-0.72 * 440.7 * 0.268)
    assert consolidate(screen(parse_rows_from_grid(grid(a, b))[0]))[0].n_accepted == 2

    cfg = ScreenConfig(prefer_latest_per_contributor=True)
    verdicts = screen(parse_rows_from_grid(grid(a, b))[0], cfg)
    assert consolidate(verdicts, cfg)[0].n_accepted == 1
    assert "SUPERSEDED" in codes(verdicts[0])


# --------------------------------------------------------------------------- #
#  7) End-to-end and reporting
# --------------------------------------------------------------------------- #
def test_process_end_to_end_on_a_messy_sheet():
    report = process(grid(
        _row(contributor="Khalil - Test"),                    # scratch
        _row(contributor="A", min_surface_mesh=0.005, max_surface_mesh=0.004),
        _row(contributor="B"),                                # good
        _row(contributor="C"),                                # good
        [None] * len(HEADER),                                 # filler
    ))
    assert report.n_rows == 4
    assert {v.row.contributor for v in report.accepted} == {"B", "C"}
    assert report.ok
    assert "TEST_ROW" in report.flag_tally()
    assert "2 accepted" in report.summary()


def test_report_summary_lists_every_case():
    report = process(grid(_row(ride_height_mm=40), _row(ride_height_mm=50)))
    text = report.summary()
    assert "40 mm" in text and "50 mm" in text


def test_process_on_a_sheet_with_no_usable_runs_does_not_crash():
    report = process(grid(_row(contributor="Test", lift_force_N=None,
                               lift_coeff=None)))
    assert not report.ok
    assert report.cases[0].n_accepted == 0


def test_empty_input_is_handled():
    report = process([])
    assert report.n_rows == 0
    assert report.cases == []
    assert report.parse_warnings


# --------------------------------------------------------------------------- #
#  8) Outputs
# --------------------------------------------------------------------------- #
def test_workbook_has_the_expected_sheets_and_reopens(tmp_path):
    openpyxl = pytest.importorskip("openpyxl")
    report = process(grid(_row(contributor="Khalil - Test"), _row(contributor="A"),
                          _row(contributor="B")))
    path = str(tmp_path / "out.xlsx")
    write_workbook(report, path)
    assert os.path.exists(path)

    wb = openpyxl.load_workbook(path)
    assert wb.sheetnames == ["Consolidated", "Accepted Runs", "Rejected Runs",
                             "Screening Report", "Contributors", "Config"]
    # The rejected row is present WITH its reason, not merely absent.
    rejected = "\n".join(str(c.value) for row in wb["Rejected Runs"].iter_rows()
                         for c in row if c.value)
    assert "Khalil" in rejected and "scratch" in rejected.lower()
    wb.close()


def test_workbook_aggregates_are_live_formulas(tmp_path):
    """The team must be able to audit the average without trusting this code."""
    openpyxl = pytest.importorskip("openpyxl")
    report = process(grid(_row(contributor="A"), _row(contributor="B")))
    path = str(tmp_path / "out.xlsx")
    write_workbook(report, path)

    wb = openpyxl.load_workbook(path)
    ws = wb["Consolidated"]
    formulas = [c.value for row in ws.iter_rows() for c in row
                if isinstance(c.value, str) and c.value.startswith("=")]
    assert any("AVERAGE" in f and "Accepted Runs" in f for f in formulas)
    # Sheet names containing a space must be quoted in the reference.
    assert all("'Accepted Runs'!" in f for f in formulas)
    wb.close()


def test_workbook_writes_when_every_run_was_rejected(tmp_path):
    pytest.importorskip("openpyxl")
    report = process(grid(_row(contributor="Test", min_ortho_quality=0.01)))
    path = str(tmp_path / "empty.xlsx")
    write_workbook(report, path)
    assert os.path.exists(path)


def test_csv_bundle_round_trips(tmp_path):
    report = process(grid(_row(contributor="Khalil - Test"), _row(contributor="A"),
                          _row(contributor="B")))
    paths = write_csv_bundle(report, str(tmp_path))
    assert len(paths) == 5
    assert all(os.path.exists(p) for p in paths)
    assert paths[0].endswith("_consolidated.csv")

    with open(paths[0], encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 1
    assert rows[0]["Runs Accepted"] == "2"
    assert rows[0]["Runs Rejected"] == "1"


def test_consolidated_csv_is_parseable_text():
    report = process(grid(_row()))
    rows = list(csv.DictReader(io.StringIO(consolidated_csv(report))))
    assert rows[0]["Component"] == "Front Wing"
    assert float(rows[0]["Mean Lift Coefficient"]) == pytest.approx(-0.8317, rel=1e-3)


def test_screening_report_covers_every_row(tmp_path):
    report = process(grid(_row(contributor="A"), _row(contributor="Test rig")))
    paths = write_csv_bundle(report, str(tmp_path))
    log = [p for p in paths if os.path.basename(p).endswith("_screening_report.csv")][0]
    with open(log, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert {r["Contributor"] for r in rows} == {"A", "Test rig"}
    assert any(r["Code"] == "CLEAN" for r in rows)       # clean rows appear too


# --------------------------------------------------------------------------- #
#  9) Bridge into the rest of KinematiK
# --------------------------------------------------------------------------- #
def test_coeff_results_feed_the_aero_map():
    from suspension.aero import AeroMap, Attitude

    report = process(grid(
        _row(contributor="A", ride_height_mm=40),
        _row(contributor="B", ride_height_mm=40),
        _row(contributor="C", ride_height_mm=50),
    ))
    results = to_coeff_results(report)
    assert len(results) == 2
    assert all(r.is_usable() for r in results)
    # Sign convention preserved: this sheet already uses negative = downforce.
    assert all(r.c_lift < 0 for r in results)
    assert all(r.provenance.backend == "ansys-run-log" for r in results)

    amap = AeroMap.from_results(results)
    assert len(amap) == 2
    q = amap.query(Attitude(ride_height_mm=45, speed_ms=26.8224))
    assert q.c_lift is not None


def test_cases_with_no_survivors_do_not_become_results():
    report = process(grid(_row(contributor="Test rig")))
    assert to_coeff_results(report) == []


def test_coeff_result_notes_carry_the_sample_count():
    report = process(grid(_row(contributor="A")))
    r = to_coeff_results(report)[0]
    assert "SINGLE RUN" in r.provenance.notes


# --------------------------------------------------------------------------- #
#  10) Config is honest about itself
# --------------------------------------------------------------------------- #
def test_config_serialises_every_threshold_for_the_report():
    rows = ScreenConfig().as_rows()
    keys = {k for k, _ in rows}
    for expected in ("rho", "yplus_wf_reject", "ortho_quality_reject",
                     "outlier_z_threshold", "min_n_for_outlier",
                     "cp_stagnation_warn", "reject_test_rows"):
        assert expected in keys


def test_log_law_intersection_is_the_wall_function_floor():
    cfg = ScreenConfig()
    assert cfg.yplus_wf_reject[0] == pytest.approx(LOG_LAW_INTERSECTION_YPLUS)


def test_thresholds_are_overridable():
    strict = ScreenConfig(skewness_reject=0.60)
    v = screen(parse_rows_from_grid(grid(_row(max_skewness=0.70)))[0], strict)[0]
    assert not v.accepted
    assert "SKEWNESS" in codes(v)


def test_consolidated_formulas_reference_the_right_block(tmp_path):
    """
    Regression: the Consolidated formulas must span exactly the accepted rows for
    their own case. An off-by-one here still yields a file that opens and
    recalculates with zero errors — it just averages the wrong runs — so the only
    way to catch it is to resolve the ranges and compare against the source rows.
    """
    openpyxl = pytest.importorskip("openpyxl")
    q = dynamic_pressure(26.8224)
    rows = []
    for rh, cls in ((40, (-0.80, -0.84, -0.82)), (50, (-0.70, -0.74))):
        for i, cl in enumerate(cls):
            rows.append(_row(contributor=f"C{rh}_{i}", ride_height_mm=rh,
                             lift_coeff=cl, lift_force_N=cl * q * 0.268))
    report = process(grid(*rows))
    path = str(tmp_path / "blocks.xlsx")
    write_workbook(report, path)

    wb = openpyxl.load_workbook(path)
    ws, acc = wb["Consolidated"], wb["Accepted Runs"]

    header_row = next(r for r in range(1, 12)
                      if ws.cell(row=r, column=1).value == "Case")
    for offset, case in enumerate(report.cases):
        cell = ws.cell(row=header_row + 1 + offset, column=8).value
        assert isinstance(cell, str) and cell.startswith("=")
        # Pull "V5:V7" out of =IFERROR(AVERAGE('Accepted Runs'!V5:V7),"")
        ref = cell.split("!")[1].split(")")[0]
        col = ref.split(":")[0][0]
        lo = int(ref.split(":")[0][1:])
        hi = int(ref.split(":")[1][1:])

        assert hi - lo + 1 == case.n_accepted, "range width != accepted count"
        spanned = [acc.cell(row=r, column=openpyxl.utils.column_index_from_string(col)).value
                   for r in range(lo, hi + 1)]
        assert all(isinstance(x, (int, float)) for x in spanned), \
            f"range {ref} covers non-numeric cells — it is off the data block"
        assert sum(spanned) / len(spanned) == pytest.approx(case.lift_coeff_mean)
        # And the block must belong to THIS case, not a neighbour's.
        labels = {acc.cell(row=r, column=1).value for r in range(lo, hi + 1)}
        assert labels == {case.case.label()}
    wb.close()


# --------------------------------------------------------------------------- #
#  11) The solver-setup columns
# --------------------------------------------------------------------------- #
#  Scheme, Order, Pseudo Time Step, Courant Number and Initialization were parsed
#  and then ignored — carried into the output tables but never used to judge
#  anything. These tests pin what they now do. Two of them have a defensible
#  right answer on their own; the rest only mean something COMPARATIVELY, which
#  is the check that matters: two runs solved differently at one operating point
#  are not two samples of one quantity, however valid each is alone.
def test_canonical_fields_mirror_the_sheet():
    """Output columns read as the same document the team filled in."""
    labels = [label for _key, label in CANONICAL_FIELDS]
    sheet_order = [
        "Contributor", "Front or Rear Wing?", "Ride-Height (mm)",
        "Velocity (m/s)", "Desired Y+", "Min Surface Mesh Length",
        "Max Surface Mesh Length", "First Layer Height (m)", "Number of Layers",
        "Min Orthogonal Quality", "Max Skewness", "Max Aspect Ratio",
        "Viscous Model", "Scheme", "Order", "Pseudo Time Step",
        "Courant Number", "Initialization", "Lift Force (N)",
        "Lift Coefficient", "Drag Force (N)", "Drag Coefficient",
        "Max Pressure (Pa)", "Min. Pressure (Pa)", "Mass Imbalance (kg/s)",
        "Average Y+", "Notes",
    ]
    assert labels[:len(sheet_order)] == sheet_order
    # Optional extras follow, so a sheet carrying them still parses.
    assert labels[len(sheet_order):] == ["Converged", "Iteration"]


def test_every_sheet_column_is_parsed_into_a_field():
    """
    All 27 sheet columns reach a field. Uses a row with every cell filled: the
    point is that no column is silently unmapped, not that a real sheet never
    leaves Notes blank.
    """
    rows, _, unmapped = parse_rows_from_grid(grid(_row(notes="ramped to 2nd")))
    assert not unmapped, f"sheet columns not mapped to a field: {unmapped}"
    r = rows[0]
    for key, label in CANONICAL_FIELDS:
        if key in ("converged", "iteration"):
            continue                      # not on the standard sheet
        assert getattr(r, key) is not None, f"{label} did not reach field {key}"
    # And the five that used to be parsed-then-ignored are all present.
    for key in SETUP_FIELDS:
        assert getattr(r, key) is not None


@pytest.mark.parametrize("text,expected", [
    ("Second", rl.Discretisation.SECOND),
    ("2nd order upwind", rl.Discretisation.SECOND),
    ("QUICK", rl.Discretisation.SECOND),
    ("First", rl.Discretisation.FIRST),
    ("1st order", rl.Discretisation.FIRST),
    ("First to Second Order", rl.Discretisation.MIXED),   # a ramped solve
    ("", rl.Discretisation.UNKNOWN),
    (None, rl.Discretisation.UNKNOWN),
])
def test_discretisation_classification(text, expected):
    assert rl.discretisation_of(text) == expected


def test_first_order_warns_because_it_smears_the_suction_peak():
    v = screen_one(order="First")
    assert v.accepted, "first order is how you START a solve — a warning, not a fault"
    assert "FIRST_ORDER" in codes(v)
    assert "diffusive" in v.reason()


def test_first_order_can_be_made_a_rejection():
    rows, _, _ = parse_rows_from_grid(grid(_row(order="First")))
    v = screen(rows, ScreenConfig(reject_first_order=True))[0]
    assert not v.accepted


def test_second_order_is_clean():
    assert "FIRST_ORDER" not in codes(screen_one(order="Second"))


def test_courant_number_band():
    assert "COURANT_HIGH" in codes(screen_one(courant_number=5000))
    assert "COURANT_LOW" in codes(screen_one(courant_number=0.05))
    assert "COURANT_INVALID" in codes(screen_one(courant_number=-3))
    assert not codes(screen_one(courant_number=50))
    # All warnings — a Courant choice does not invalidate a converged answer.
    assert screen_one(courant_number=5000).accepted


def test_unrecorded_setup_is_flagged_not_silently_ignored():
    v = screen_one(viscous_model=None, scheme=None, order=None,
                   initialization=None)
    assert "SETUP_UNRECORDED" in codes(v)
    assert v.accepted
    assert "cannot be compared" in v.reason()


def test_setup_signature_ignores_convergence_path_settings():
    """
    Courant number and pseudo time step change the PATH to convergence, not the
    converged answer, so runs differing only there are still comparable.
    """
    rows, _, _ = parse_rows_from_grid(grid(
        _row(courant_number=10, pseudo_time_step="Disabled"),
        _row(courant_number=900, pseudo_time_step="0.01")))
    a, b = rows
    assert rl.setup_signature(a) == rl.setup_signature(b)


def test_mixed_turbulence_model_at_one_point_is_flagged():
    """
    The check the setup columns exist for: both runs pass every physics gate and
    are still not two samples of the same quantity.
    """
    q = dynamic_pressure(26.8224)
    rows = [_row(contributor=f"N{i}") for i in range(3)]
    rows.append(_row(contributor="ODD", viscous_model="k-omega SST",
                     avg_yplus=1.0, desired_yplus=1.0))
    verdicts = screen(parse_rows_from_grid(grid(*rows))[0])
    by_who = {v.row.contributor: v for v in verdicts}
    assert "SETUP_MISMATCH" in codes(by_who["ODD"])
    assert "viscous model" in by_who["ODD"].reason()
    assert by_who["ODD"].accepted, "reported, not silently dropped"
    assert by_who["ODD"].derived.setup_matches_group is False
    assert by_who["N0"].derived.setup_matches_group is True


def test_mixed_turbulence_can_be_made_a_rejection():
    rows = [_row(contributor=f"N{i}") for i in range(3)]
    rows.append(_row(contributor="ODD", viscous_model="k-omega SST",
                     avg_yplus=1.0, desired_yplus=1.0))
    cfg = ScreenConfig(reject_mixed_turbulence=True)
    verdicts = screen(parse_rows_from_grid(grid(*rows))[0], cfg)
    by_who = {v.row.contributor: v for v in verdicts}
    assert not by_who["ODD"].accepted


def test_mixed_discretisation_order_is_flagged():
    rows = [_row(contributor=f"N{i}") for i in range(3)]
    rows.append(_row(contributor="ODD", order="First"))
    by_who = {v.row.contributor: v
              for v in screen(parse_rows_from_grid(grid(*rows))[0])}
    assert "SETUP_MISMATCH" in codes(by_who["ODD"])
    assert "discretisation order" in by_who["ODD"].reason()


def test_a_consistent_group_raises_no_mismatch():
    rows = [_row(contributor=f"N{i}") for i in range(4)]
    verdicts = screen(parse_rows_from_grid(grid(*rows))[0])
    assert not any("SETUP_MISMATCH" in codes(v) for v in verdicts)
    assert all(v.derived.setup_matches_group for v in verdicts)


def test_setup_consistency_can_be_disabled():
    rows = [_row(contributor=f"N{i}") for i in range(3)]
    rows.append(_row(contributor="ODD", scheme="Coupled"))
    cfg = ScreenConfig(check_setup_consistency=False)
    verdicts = screen(parse_rows_from_grid(grid(*rows))[0], cfg)
    assert not any("SETUP_MISMATCH" in codes(v) for v in verdicts)


def test_consolidated_case_carries_the_method_behind_the_number():
    case = consolidate(screen(parse_rows_from_grid(grid(_row(), _row()))[0]))[0]
    assert case.viscous_models == ["k-epsilon"]
    assert case.schemes == ["Simple"]
    assert case.discretisations == ["second-order"]
    assert case.initializations == ["Standard"]
    assert case.setup_consistent
    assert "k-epsilon" in case.setup_summary()
    assert "second-order" in case.setup_summary()


def test_mixed_setup_is_called_out_in_the_summary_and_confidence():
    rows = [_row(contributor=f"N{i}") for i in range(3)]
    rows.append(_row(contributor="ODD", scheme="Coupled"))
    case = consolidate(screen(parse_rows_from_grid(grid(*rows))[0]))[0]
    assert not case.setup_consistent
    assert case.setup_summary().startswith("MIXED")
    assert "MIXED SETUP" in case.confidence
    assert any("more than one solver setup" in n for n in case.notes)


def test_courant_range_is_reported():
    rows = [_row(contributor="A", courant_number=10),
            _row(contributor="B", courant_number=200)]
    case = consolidate(screen(parse_rows_from_grid(grid(*rows))[0]))[0]
    assert case.courant_range == (10, 200)


def test_setup_columns_reach_the_consolidated_output():
    report = process(grid(_row(), _row()))
    text = consolidated_csv(report)
    header = text.splitlines()[0]
    for col in ("Viscous Model(s)", "Scheme(s)", "Discretisation",
                "Initialization(s)", "Courant Range", "Setup Consistent?"):
        assert col in header, f"{col} missing from the consolidated output"
    assert "k-epsilon" in text and "second-order" in text


def test_per_run_sheets_carry_the_setup_verdict(tmp_path):
    """The Accepted/Rejected sheets show each run's own setup verdict."""
    report = process(grid(_row(contributor="A"), _row(contributor="B"),
                          _row(contributor="ODD", scheme="Coupled")))
    paths = write_csv_bundle(report, str(tmp_path))
    acc = [p for p in paths
           if os.path.basename(p).endswith("_accepted_runs.csv")][0]
    with open(acc, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    by_who = {r["Contributor"]: r for r in rows}
    assert by_who["A"]["Discretisation"] == "second-order"
    assert by_who["A"]["Setup Matches Group?"] == "yes"
    assert by_who["ODD"]["Setup Matches Group?"] == "NO"
    # And the sheet's own columns are still there, in sheet order.
    assert rows[0]["Scheme"] and rows[0]["Order"] and rows[0]["Initialization"]


# --------------------------------------------------------------------------- #
#  12) The Contributor column earns its keep
# --------------------------------------------------------------------------- #
def test_contributor_stats_summarise_each_person():
    report = process(grid(
        _row(contributor="A"), _row(contributor="A", order="First"),
        _row(contributor="B"), _row(contributor="Khalil - Test")))
    stats = {r["contributor"]: r for r in report.contributor_stats()}
    assert stats["A"]["runs"] == 2 and stats["A"]["accepted"] == 2
    assert stats["Khalil - Test"]["accepted"] == 0
    assert stats["Khalil - Test"]["acceptance_pct"] == 0.0
    assert "TEST_ROW" in stats["Khalil - Test"]["top_flags"]
    assert "FIRST_ORDER" in stats["A"]["top_flags"]


def test_unattributed_rows_are_grouped_not_dropped():
    report = process(grid(_row(contributor=None)))
    stats = report.contributor_stats()
    assert stats[0]["contributor"] == "unattributed"
    assert stats[0]["runs"] == 1


def test_contributors_sheet_and_csv_are_written(tmp_path):
    openpyxl = pytest.importorskip("openpyxl")
    report = process(grid(_row(contributor="A"), _row(contributor="B")))
    path = str(tmp_path / "out.xlsx")
    write_workbook(report, path)
    wb = openpyxl.load_workbook(path)
    assert "Contributors" in wb.sheetnames
    text = "\n".join(str(c.value) for row in wb["Contributors"].iter_rows()
                     for c in row if c.value)
    assert "A" in text and "B" in text
    wb.close()

    paths = write_csv_bundle(report, str(tmp_path))
    who = [p for p in paths
           if os.path.basename(p).endswith("_contributors.csv")][0]
    with open(who, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert {r["Contributor"] for r in rows} == {"A", "B"}



def test_disabled_pseudo_transient_parses_without_complaint():
    """The real sheet often reads 'Disabled' for both — that is not an error."""
    rows, _, _ = parse_rows_from_grid(grid(
        _row(pseudo_time_step="Disabled", courant_number="Disabled")))
    r = rows[0]
    assert r.pseudo_time_step == "Disabled"
    assert r.courant_number is None            # not a number, not invented
    v = screen(rows)[0]
    assert v.accepted
    assert not [c for c in codes(v) if c.startswith("COURANT")]
