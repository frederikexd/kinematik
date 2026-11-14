# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Addendum 4: the parallel split and cell heating against 54 measured modules (Stanford Energy Control Lab,
doi:10.17632/zh58byr53c.1, CC BY 4.0). Limits registered in docs/ACCUMULATOR_HARNESS_BENCHMARK_PLAN.md before any
comparison; the reference was extracted by scripts/parallel_module_reference.py. Misses are pinned, not tuned away."""
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from suspension import pack_thermal as pt  # noqa: E402
from suspension.parallel_split import ladder_currents  # noqa: E402

D = json.loads((ROOT / "benchmarks" / "electrical" / "parallel_modules" / "d_stanford_modules.json")
               .read_text(encoding="utf-8"))
TESTS = D["tests"]
LIMIT = 0.10                                    # registered: 10 % of pack current / 4 (unitless)
TEMPCO = pt.CellParams().r_tempco_per_c


def _r_at_chamber(t):
    """Each cell's HPPC 10 s resistance (ohm, 23 degC) moved to the chamber temperature with the default tempco."""
    return np.array(t["r10_ohm"]) * (1 + TEMPCO * (t["chamber_c"] - 25)) / (1 + TEMPCO * (23 - 25))


def _ladder(t, both=True):
    return ladder_currents(_r_at_chamber(t), t["r_interconnect_mohm"] * 1e-3, t["i_total_a"], both_rails=both)


def _err(model, measured, t):
    return float(np.max(np.abs(np.asarray(model) - np.asarray(measured))) / (t["i_total_a"] / 4))


def _by(key, value, fn):
    return [fn(t) for t in TESTS if t[key] == value]


def test_the_reference_is_the_registered_dataset():
    assert len(TESTS) == 54 and "zh58byr53c" in D["source"]
    assert {t["r_interconnect_mohm"] for t in TESTS} == {0, 1, 3} and {t["chamber_c"] for t in TESTS} == {10, 25, 40}
    assert {t["mix"] for t in TESTS} == {"NCA", "NMC", "Mixed"}
    for t in TESTS:
        assert 14.0 < t["i_total_a"] < 15.0 and len(t["cells"]) == 4                 # 0.75C of four ~4.9 Ah cells


def test_hall_branch_currents_disagree_with_the_cycler_by_up_to_15_percent():
    #  Data quality, found after registration: the four Hall currents do not sum to the cycler's current.
    ratio = np.array([sum(t["i_cell_10s_a"]) / t["i_total_a"] for t in TESTS])
    assert 0.85 < ratio.min() < 0.86 and 1.14 < ratio.max() < 1.16


# ----------------------------------------------------------------------------- D1. the shipped even split -- #
def _even_err(t):
    even = t["i_total_a"] / 4
    return max(_err([even] * 4, t["i_cell_10s_a"], t), _err([even] * 4, t["i_cell_mean_a"], t))


def test_even_split_misses_and_is_kept():
    errs = [_even_err(t) for t in TESTS]
    assert sum(e <= LIMIT for e in errs) == 2                                     # 2 of 54: a miss
    med = {r: np.median(_by("r_interconnect_mohm", r, _even_err)) for r in (0, 1, 3)}
    assert 0.15 < med[0] < 0.18 and 0.35 < med[1] < 0.38 and 0.60 < med[3] < 0.65
    #  The cell nearest the terminals carries more than its share in every resistive-busbar test.
    assert all(t["i_cell_10s_a"][0] > t["i_total_a"] / 4 for t in TESTS if t["r_interconnect_mohm"] > 0)


# --------------------------------------------------------------------------------- D2. the resistive ladder -- #
def _ladder_err(t):
    return _err(_ladder(t), t["i_cell_10s_a"], t)


def test_ladder_misses_the_registered_limit_and_is_kept():
    errs = [_ladder_err(t) for t in TESTS]
    assert sum(e <= LIMIT for e in errs) == 20                                    # 20 of 54; registered 90 %
    assert sum(_err(_ladder(t, False), t["i_cell_10s_a"], t) <= LIMIT for t in TESTS) == 12   # one rail: worse


def test_ladder_beats_the_even_split_wherever_the_busbar_has_resistance():
    for r in (1, 3):
        assert np.median(_by("r_interconnect_mohm", r, _ladder_err)) < 0.5 * np.median(
            _by("r_interconnect_mohm", r, _even_err))
    assert np.median([_ladder_err(t) for t in TESTS]) < 0.16


def test_ladder_against_currents_scaled_to_the_cycler_total_disclosed_not_scored():
    def scaled(t):
        m = np.array(t["i_cell_10s_a"])
        return _err(_ladder(t), m * t["i_total_a"] / m.sum(), t)
    assert sum(scaled(t) <= LIMIT for t in TESTS) == 43                           # 80 %: still short of 90 %
    assert np.median(_by("mix", "Mixed", scaled)) > np.median(_by("mix", "NMC", scaled))   # mixed cells worst


def test_ladder_nearest_cell_current_found_after_registration_disclosed():
    #  Not a registered limit: the nearest cell, which the app reports, over the 36 resistive-link modules.
    e = np.array([_ladder(t)[0] / t["i_cell_10s_a"][0] - 1.0 for t in TESTS if t["r_interconnect_mohm"] > 0])
    assert e.size == 36 and -0.04 < np.median(e) < -0.02 and -0.21 < e.min() < -0.19 and e.max() < 0.22


def test_pack_model_uses_the_ladder_when_links_have_resistance():
    cell = pt.default_cell_params()
    stiff = pt.PackLayout(rows=1, cols=4, series=1, parallel=4, cell=cell)
    assert stiff.cell_current_share() is None
    lay = pt.PackLayout(rows=1, cols=8, series=2, parallel=4, cell=cell, r_interconnect_ohm=0.003)
    share = lay.cell_current_share()
    group = ladder_currents([cell.r_internal_ohm] * 4, 0.003, 1.0)
    assert np.allclose(share, np.tile(group, 2)) and np.isclose(share[:4].sum(), 1.0) and share[0] > 0.25
    t, cur = np.linspace(0, 600, 601), np.full(601, 40.0)
    hot = pt.PackThermalModel(layout=lay, airflow=pt.AirflowParams(mode="duct", duct_cfm=0.0),
                              k_cell_cell=0.0).simulate(t, cur)
    even = pt.PackThermalModel(layout=replace(lay, r_interconnect_ohm=0.0),
                               airflow=pt.AirflowParams(mode="duct", duct_cfm=0.0), k_cell_cell=0.0).simulate(t, cur)
    rise_hot, rise_even = hot.temp_history_c[-1] - lay.ambient_c, even.temp_history_c[-1] - lay.ambient_c
    assert np.allclose(rise_even, rise_even[0])                              # equal split: every cell the same
    assert rise_hot[0] > 1.5 * rise_even[0] and rise_hot[3] < rise_even[3]   # the nearest cell runs hotter
    assert np.allclose(rise_hot[:4], rise_hot[4:])                           # every group alike


# ------------------------------------------------------------------- D3. still air is conservative (check) -- #
def test_still_air_never_under_predicts_the_measured_rise():
    base = pt.default_cell_params()
    ratios = []
    for t in TESTS[::6]:                                             # every chemistry, ageing and resistance
        ts, amb = np.array(t["t_s"]), float(np.mean(t["ambient_c"]))
        for k in range(4):
            cell = replace(base, r_internal_ohm=t["r10_ohm"][k] / (1 + TEMPCO * (23 - 25)))
            lay = pt.PackLayout(rows=1, cols=1, series=1, parallel=1, cell=cell, ambient_c=amb)
            t0 = t["temp_c"][k][0]
            res = pt.PackThermalModel(layout=lay, airflow=pt.AirflowParams(mode="duct", duct_cfm=0.0)).simulate(
                ts, np.array(t["i_cell_a"][k]), init_temp_c=t0)
            meas, model = max(t["temp_c"][k]) - t0, float(res.temp_history_c[:, 0].max() - t0)
            assert model >= meas - 1.0
            ratios.append(model / meas)
    assert np.median(ratios) > 2.0              # a ceiling, not a prediction: the chamber's airflow cools more


# --------------------------------------------------------------------------------------- the solver itself -- #
def test_ladder_known_answers():
    assert np.allclose(ladder_currents([0.02] * 4, 0.0, 10.0), 2.5)
    assert np.allclose(ladder_currents([0.01, 0.03], 0.0, 8.0), [6.0, 2.0])
    i = ladder_currents([0.02, 0.02], 0.003, 10.0)                 # far cell sees R + 2r
    assert np.allclose(i, [10 * 0.026 / 0.046, 10 * 0.020 / 0.046])
    i = ladder_currents([0.02, 0.02], 0.003, 10.0, both_rails=False)
    assert np.allclose(i, [10 * 0.023 / 0.043, 10 * 0.020 / 0.043])
    i = ladder_currents([0.02] * 6, 0.001, 30.0)
    assert np.isclose(i.sum(), 30.0) and np.all(np.diff(i) < 0)
    assert np.allclose(ladder_currents([0.02], 0.005, 4.0), [4.0])


def test_ladder_open_circuit_voltages():
    i = ladder_currents([0.02, 0.02], 0.0, 0.0, ocv_v=[3.7, 3.6])  # no load: the higher cell charges the lower
    assert np.allclose(i, [2.5, -2.5])
    i = ladder_currents([0.02, 0.02], 0.002, 0.0, ocv_v=[3.7, 3.6])
    assert np.allclose(i, [0.1 / 0.044, -0.1 / 0.044])
    assert np.allclose(ladder_currents([0.02] * 3, 0.002, 6.0, ocv_v=[3.6] * 3),
                       ladder_currents([0.02] * 3, 0.002, 6.0))


def test_ladder_takes_numbers_as_text():
    assert np.allclose(ladder_currents(["0.01", "0.03"], 0.0, 8.0), [6.0, 2.0])
    assert np.allclose(ladder_currents([0.02, 0.02], 0.0, 0.0, ocv_v=["3.7", "3.6"]), [2.5, -2.5])


@pytest.mark.parametrize("args, kw, msg", [
    (([], 0.0, 1.0), {}, "needs at least one cell, every cell resistance above 0 ohm, interconnection at least 0"),
    (([0.02, 0.0], 0.0, 1.0), {}, "needs at least one cell, every cell resistance above 0 ohm, interconnection at least 0"),
    (([0.02], -0.001, 1.0), {}, "needs at least one cell, every cell resistance above 0 ohm, interconnection at least 0"),
    (([0.02, 0.02], 0.0, 1.0), {"ocv_v": [3.6]}, "ocv_v needs one value per cell")])
def test_ladder_rejects_bad_inputs(args, kw, msg):
    with pytest.raises(ValueError) as e:
        ladder_currents(*args, **kw)
    assert str(e.value) == msg
