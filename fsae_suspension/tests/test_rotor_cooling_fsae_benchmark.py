# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Rotor cooling on an FSAE car (benchmarks/rotor_cooling_fsae). Registered: the uncalibrated correlation fails at
speed (it over-predicts cooling); the calibrated line, fitted on three speeds, predicts the other three."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import rotor_cooling_fsae_benchmark as rc  # noqa: E402
from suspension import brake_thermal as bt  # noqa: E402


def test_uncalibrated_misses_high_at_speed_and_is_kept():
    r = rc.run()
    assert not r["passed"] and 0.85 < r["worst_abs_rel"] < 1.0
    fast = [x for x in r["rows"] if x["v_m_s"] > 20]
    assert all(x["rel"] > 0.6 for x in fast)               # too much cooling: rotor temperatures read low
    assert json.loads((rc.BENCH / "results_first_run.json").read_text())["passed"] is False


def test_calibrated_line_predicts_held_out_speeds():
    c = rc.run()["calibrated_held_out"]
    assert c["passed"] and c["odd_to_even"]["passes"] and c["even_to_odd"]["passes"]


def test_fit_convection_line_is_least_squares():
    assert bt.fit_convection_line([0, 10], [30, 70]) == pytest.approx((30.0, 4.0))
    with pytest.raises(ValueError):
        bt.fit_convection_line([5, 5], [1, 2])
