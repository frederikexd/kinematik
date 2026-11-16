# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Steering benchmark: the command files are current and Chrono agrees with KinematiK's lock and Ackermann."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import adams_steering_benchmark as asb                                              # noqa: E402


@pytest.mark.parametrize("name", asb.CORNERS)
def test_the_command_files_are_the_ones_the_script_writes(name):
    assert (asb.BENCH / f"{name}_steer.cmd").read_text() == asb.command_file(name)


@pytest.mark.parametrize("name", asb.CORNERS)
def test_project_chrono_agrees_within_the_limits_set_before_the_run(name):
    r = asb.compare(name, "chrono")
    assert r is not None and r["passed"], r


@pytest.mark.parametrize("name", asb.CORNERS)
def test_simscape_multibody_agrees_within_the_limits_set_before_the_run(name):
    r = asb.compare(name, "matlab")
    assert r is not None and r["passed"], r


def test_the_lock_read_off_a_curve_is_the_bounds_table_value():
    r = asb.compare("front_v11_final", "chrono")
    assert r["lock_sw_deg"] == pytest.approx(118.9, abs=0.01) and r["ackermann_lock_pct"] == pytest.approx(75.27, abs=0.01)


@pytest.mark.parametrize("name", asb.CORNERS)
def test_adams_view_agrees_once_run(name):
    r = asb.compare(name, "adams")
    if r is None:
        pytest.skip("Adams View tables not exported yet")
    assert r["passed"], r
