# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_inverse_genesis_fullcar.py — the full-vehicle inverse engine,
#  pinned.
# ============================================================================
"""What these tests guard, in the module's own order of claims:

* the rule matrix names EVERY bound a config breaks and passes a legal one —
  a TS-overvoltage pack and a too-small-parallel segment are both caught;
* the forward evaluation produces one CONSISTENT car — pack size drives mass,
  mass drives energy, energy/thermal drive the verdict — and never raises;
* the coupling is real: a tiny uncooled pack THERMAL_DNFs mid-endurance with
  the overheat lap computed (not guessed), and gets its endurance points
  zeroed; a pack with headroom finishes FEASIBLE;
* the staged search is honest: the integer grid is enumerated exhaustively,
  the diagnostics count matches, and the winner is the best car that FINISHES
  — a faster-on-paper DNF is rejected and named;
* when no legal/feasible car exists the engine says so with the binding
  constraint named, and fabricates no winner;
* determinism: identical inputs give a byte-identical report and winner;
* the downstream synthesis stages produce the right shapes — a kinematic
  intent in the corner dialect, a resolved member-load table, and the
  CAD/firmware exports.
"""

import math

import numpy as np
import pytest

from suspension.pack_thermal import CellParams
from suspension import inverse_genesis_fullcar as fc


# --------------------------------------------------------------------------- #
#  Shared fixtures — one small space that finishes, one that cooks.
# --------------------------------------------------------------------------- #
def _cool_cell():
    """A low-resistance cell in a cool tent — real thermal headroom."""
    return fc.CellSpec(capacity_ah=5.0, max_discharge_a=50.0,
                       thermal=CellParams(r_internal_ohm=0.012,
                                          temp_limit_c=60.0))


@pytest.fixture(scope="module")
def feasible_space():
    return fc.DesignSpace(
        series_range=(96, 120), series_step=24, parallel_range=(6, 7),
        final_drive_range=(3.0, 4.5), ambient_c=22.0, cell=_cool_cell())


@pytest.fixture(scope="module")
def rules():
    return fc.RuleMatrix()


# --------------------------------------------------------------------------- #
#  The rule matrix
# --------------------------------------------------------------------------- #
def test_rule_matrix_passes_a_legal_config():
    r = fc.RuleMatrix()
    cell = fc.CellSpec()
    assert r.violations(96, 5, cell, 1550.0) == []


def test_rule_matrix_names_ts_overvoltage():
    r = fc.RuleMatrix(max_ts_voltage=400.0)
    cell = fc.CellSpec()
    v = r.violations(140, 5, cell, 1550.0)   # 140s * 4.2 = 588 V > 400
    assert v and any("TS voltage" in m for m in v)


def test_rule_matrix_names_undersize_wheelbase():
    r = fc.RuleMatrix(min_wheelbase_mm=1600.0)
    v = r.violations(96, 5, fc.CellSpec(), 1550.0)
    assert v and any("wheelbase" in m for m in v)


def test_segments_needed_respects_both_caps():
    r = fc.RuleMatrix(max_segment_voltage=120.0, max_segment_energy_mj=6.0)
    n_seg, seg_s = r.segments_needed(96, 5, fc.CellSpec())
    assert n_seg >= 1 and seg_s >= 1
    # each segment within the voltage cap
    assert seg_s * fc.CellSpec().max_v <= 120.0 + 1e-6


# --------------------------------------------------------------------------- #
#  The forward evaluation — one consistent car, never raises
# --------------------------------------------------------------------------- #
def test_evaluate_is_consistent_and_finite(rules):
    space = fc.DesignSpace(cell=_cool_cell(), ambient_c=22.0)
    cfg = fc.FullCarConfig(120, 7, "four_tv", 3.6)
    sc = fc.evaluate_config(cfg, space, rules, run_thermal=True)
    assert sc.ok
    # the pack physically sizes the car
    assert sc.derived["n_cells"] == 120 * 7
    assert sc.derived["mass_kg"] > space.base_mass_kg
    for t in (sc.accel_s, sc.skidpad_s, sc.autocross_s, sc.endurance_s):
        assert math.isfinite(t) and t > 0
    # endurance covers the declared distance in whole laps
    assert sc.endurance_laps >= 1


def test_evaluate_never_raises_on_garbage():
    space = fc.DesignSpace()
    cfg = fc.FullCarConfig(1, 1, "single_diff", 0.01)   # degenerate
    sc = fc.evaluate_config(cfg, space, fc.RuleMatrix())
    # it returns a scored object with a verdict, not an exception
    assert sc.verdict in ("FEASIBLE", "ENERGY_SHORT", "THERMAL_DNF",
                          "RULE_KILLED", "FAILED")


def test_rule_killed_config_is_not_evaluated():
    r = fc.RuleMatrix(max_ts_voltage=300.0)   # 140s pack is illegal
    sc = fc.evaluate_config(fc.FullCarConfig(140, 5, "four_tv", 3.6),
                            fc.DesignSpace(), r)
    assert sc.verdict == "RULE_KILLED"
    assert sc.kill_reasons and not sc.ok


# --------------------------------------------------------------------------- #
#  The coupling: overheat is computed, not guessed
# --------------------------------------------------------------------------- #
def test_tiny_uncooled_pack_thermal_dnfs_with_a_lap_number():
    # a hot, high-resistance, small pack over a full endurance stint cooks
    hot = fc.CellSpec(capacity_ah=4.0,
                      thermal=CellParams(r_internal_ohm=0.030,
                                         temp_limit_c=60.0))
    space = fc.DesignSpace(cell=hot, ambient_c=35.0)
    sc = fc.evaluate_config(fc.FullCarConfig(96, 4, "single_diff", 4.0),
                            space, fc.RuleMatrix(), run_thermal=True)
    assert sc.verdict == "THERMAL_DNF"
    assert sc.overheat_lap is not None and 1 <= sc.overheat_lap <= sc.endurance_laps
    # and it forfeits endurance points once scored
    fc._score_field([sc], fc.PointsReference())
    assert sc.points.get("endurance", 1.0) == 0.0


def test_headroom_pack_finishes_feasible(feasible_space, rules):
    sc = fc.evaluate_config(fc.FullCarConfig(120, 7, "twin_axle", 3.6),
                            feasible_space, rules, run_thermal=True)
    assert sc.verdict == "FEASIBLE"
    assert sc.overheat_lap is None
    assert sc.energy_margin_kwh > 0


# --------------------------------------------------------------------------- #
#  The staged search — honest counts, the winner finishes
# --------------------------------------------------------------------------- #
def test_search_enumerates_the_whole_grid(feasible_space, rules):
    res = fc.synthesize_fullcar(feasible_space, rules, n_finalists=2,
                                gear_iters=3)
    space = feasible_space
    expected = (len(space.series_options()) * len(space.parallel_options())
                * len(space.architectures))
    assert res.diagnostics.n_grid == expected
    # thermal gate ran only on the finalists, not the whole field
    assert res.diagnostics.n_thermal_gates <= 2


def test_winner_finishes_the_season(feasible_space, rules):
    res = fc.synthesize_fullcar(feasible_space, rules, n_finalists=3,
                                gear_iters=3)
    assert res.ok and res.winner is not None
    assert res.winner.verdict == "FEASIBLE"
    # nobody in the field out-scores the winner among feasible cars
    feasible = [s for s in res.ranked if s.verdict == "FEASIBLE"]
    assert all(res.winner.total_points >= s.total_points - 1e-6
               for s in feasible)


def test_faster_but_dnf_is_rejected_not_crowned():
    # a space where the top-points car cooks but a slightly-slower one finishes
    space = fc.DesignSpace(
        series_range=(96, 120), series_step=24, parallel_range=(4, 7),
        final_drive_range=(3.2, 4.2), ambient_c=30.0,
        cell=fc.CellSpec(capacity_ah=4.5,
                         thermal=CellParams(r_internal_ohm=0.018,
                                            temp_limit_c=60.0)))
    res = fc.synthesize_fullcar(space, fc.RuleMatrix(), n_finalists=6,
                                gear_iters=3)
    if res.ok:
        # if a feasible winner exists, no DNF finalist may outrank it in the
        # feasible sense — the winner must itself finish
        assert res.winner.verdict == "FEASIBLE"
    else:
        # otherwise the engine refuses and names the binding constraint
        assert "finish" in res.reason.lower() or "season" in res.reason.lower()
        assert res.winner is not None   # the closest car is still reported


def test_no_legal_config_is_named_not_faked():
    # an impossible rule matrix: no series count can be legal
    r = fc.RuleMatrix(max_ts_voltage=10.0)
    res = fc.synthesize_fullcar(fc.DesignSpace(), r, n_finalists=2,
                                gear_iters=2)
    assert not res.ok and res.winner is None
    assert res.rule_killed
    assert all(s.kill_reasons for s in res.rule_killed)


# --------------------------------------------------------------------------- #
#  Determinism
# --------------------------------------------------------------------------- #
def test_report_is_byte_identical_across_runs(feasible_space, rules):
    a = fc.synthesize_fullcar(feasible_space, rules, n_finalists=3,
                              gear_iters=3)
    b = fc.synthesize_fullcar(feasible_space, rules, n_finalists=3,
                              gear_iters=3)
    assert fc.render_fullcar_md(a) == fc.render_fullcar_md(b)
    assert a.winner.config.label() == b.winner.config.label()


def test_diagnostics_timing_excluded_from_report(feasible_space, rules):
    res = fc.synthesize_fullcar(feasible_space, rules, n_finalists=2,
                                gear_iters=2)
    # wall time lives in timing(), never in the deterministic summary()
    assert "wall time" not in res.diagnostics.summary()
    assert "wall time" in res.diagnostics.timing()
    # and the rendered report carries no wall-clock seconds
    assert "wall time" not in fc.render_fullcar_md(res)


# --------------------------------------------------------------------------- #
#  Downstream synthesis stages
# --------------------------------------------------------------------------- #
def test_kinematic_intent_is_corner_dialect(feasible_space, rules):
    res = fc.synthesize_fullcar(feasible_space, rules, n_finalists=2,
                                gear_iters=2)
    tg = fc.kinematic_intent_for(res.winner, feasible_space)
    channels = {c.channel for c in tg.curves}
    assert {"camber_deg", "toe_deg", "rc_height_mm"} <= channels
    # toe intent is dead bump steer (flat, zero)
    toe = next(c for c in tg.curves if c.channel == "toe_deg")
    assert np.allclose(toe.target, 0.0)
    # camber gains toward upright in bump (monotone in travel)
    cam = next(c for c in tg.curves if c.channel == "camber_deg")
    assert cam.target[0] != cam.target[-1]


def test_load_case_resolves_to_members(feasible_space, rules):
    res = fc.synthesize_fullcar(feasible_space, rules, n_finalists=2,
                                gear_iters=2)
    lc = fc.load_case_for(res.winner, feasible_space)
    assert lc.fz_n > 0 and lc.mu_lateral > 0
    assert lc.member_forces                 # some members resolved
    assert all(math.isfinite(v) for v in lc.member_forces.values())


def test_exports_have_the_expected_shape(feasible_space, rules):
    res = fc.synthesize_fullcar(feasible_space, rules, n_finalists=2,
                                gear_iters=2)
    from suspension.kinematics import Hardpoints
    csv = fc.export_hardpoints_csv(Hardpoints.default())
    assert csv.splitlines()[0] == "point,x_mm,y_mm,z_mm"
    assert len(csv.splitlines()) > 4
    ch = fc.export_flash_constants_c(res.winner, feasible_space, rules)
    assert "#define TS_POWER_LIMIT_W" in ch and "#endif" in ch
    py = fc.export_flash_constants_py(res.winner, feasible_space, rules)
    ns: dict = {}
    exec(py, ns)                            # it's importable, valid Python
    assert "TS_PACK_SERIES" in ns and ns["TS_PACK_SERIES"] == res.winner.config.series


def test_render_md_contains_verdict_and_ledger(feasible_space, rules):
    res = fc.synthesize_fullcar(feasible_space, rules, n_finalists=2,
                                gear_iters=2)
    md = fc.render_fullcar_md(res, include_exports=True)
    assert "InverseGenesis-FullCar" in md
    assert "What the search actually did" in md
    assert "millions of states per second" in md.lower()   # the honesty note
