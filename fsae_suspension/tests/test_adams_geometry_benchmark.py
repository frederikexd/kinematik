# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Adams View geometry benchmark: the command files are current, and KinematiK's side reproduces hand values."""
import os
import sys

import numpy as np
import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import adams_geometry_benchmark as agb                                              # noqa: E402


@pytest.mark.parametrize("name", agb.CORNERS)
def test_the_command_files_are_the_ones_the_script_writes(name):
    assert (agb.BENCH / f"{name}_geo.cmd").read_text() == agb.command_file(name)


def test_kinematik_side_matches_the_hand_construction_at_ride_height():
    k = agb.kinematik_curves("front_v11_final")
    i = int(np.argmin(np.abs(agb.TRAVEL)))
    # kingpin through LBJ (-5, 583.6, 120) and UBJ (6.5, 560.5, 300) meets z = 0 at x = -12.667, y = 599.0
    assert k["trail"][i] == pytest.approx(12.667, abs=1e-3) and k["scrub"][i] == pytest.approx(6.0, abs=1e-3)
    assert agb.anti_from_path(k["cp_dx"], k["cp_dz"]) == pytest.approx(k["anti_pct"][0], abs=0.05)
    assert k["damper_len"][i] == 0.0 and k["damper_len"][0] > 0 > k["damper_len"][-1]   # droop extends the damper


def test_compare_waits_for_the_adams_run():
    for n in agb.CORNERS:
        if not (agb.BENCH / f"{n}_geo__travel.tab").exists():
            assert agb.compare(n) is None


@pytest.mark.parametrize("name", agb.CORNERS)
def test_project_chrono_agrees_within_the_limits_set_for_adams(name):
    r = agb.compare(name, "chrono")
    assert r is not None and r["passed"], r
