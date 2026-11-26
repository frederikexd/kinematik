# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Rotor convection against dyno CFD (benchmarks/rotor_convection). Registered result: speed scaling passes; the
whole-disc conductance of a vented disc fails with the default area factor. Pinned so neither changes silently."""
import json

import pytest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import rotor_convection_benchmark as rc  # noqa: E402
from suspension import brake_thermal as bt  # noqa: E402


def test_registered_outcome():
    r = rc.run()
    assert r["test_2_passes"] and not r["test_1_passes"]
    assert all(-0.67 < x["rel"] < -0.55 for x in r["test_1"])      # the miss: area, not coefficient
    assert abs(r["diagnostic"]["kinematik_h_W_m2K"] / r["diagnostic"]["cfd_mean_h_500C_W_m2K"] - 1) < 0.15


def test_rerun_with_the_wetted_area_passes():
    r = rc.run()["rerun_wetted_area_not_blind"]
    assert r["passed"] and all(abs(x["rel"]) < 0.20 for x in r["test_1"])


def test_crossflow_selects_the_correlation_and_none_the_measured_car_line():
    assert bt.h_rotating_disc(220, 20.0) == pytest.approx(33.0 + 0.74 * 20.0)
    assert bt.h_rotating_disc(220, 20.0, crossflow_ms=0.0) < bt.h_rotating_disc(220, 20.0, crossflow_ms=20.0)


def test_first_run_kept():
    assert json.loads((rc.BENCH / "results_first_run.json").read_text())["passed"] is False
