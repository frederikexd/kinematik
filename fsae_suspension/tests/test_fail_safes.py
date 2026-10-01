# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Hand roll centre, member margins and over-travel."""
import pytest

from suspension import robustness as rb
from suspension.ghost_topology import _rc_height_mm
from suspension.kinematics import Hardpoints, SuspensionKinematics
from suspension.loadpath import WheelLoad


def test_hand_roll_centre_matches_the_solver_to_four_decimals():
    hp = Hardpoints.default(); h = rb.hand_roll_centre(hp)
    st = SuspensionKinematics(hp).solve_at_travel(0.0)
    assert h["ic_y_mm"] == pytest.approx(float(st.instant_center[0]), abs=1e-4)
    assert h["rc_height_mm"] == pytest.approx(_rc_height_mm(st, 1210.0), abs=1e-4)


def test_member_margins_are_physical():
    m = rb.member_margins(Hardpoints.default(), {"bump": WheelLoad(Fz=2100.0), "brake": WheelLoad(Fx=1500.0, Fz=1150.0)})
    assert set(m) == {"UF", "UR", "LF", "LR", "TR", "PR"}
    for r in m.values():
        assert r["fos_yield"] > 0 and r["fos_buckling"] > 0
        if r["worst_n"] < 0: assert r["fos_buckling"] >= r["fos_yield"] * 0.3      # buckling and yield both reported


def test_default_linkage_has_no_toggle_to_50_mm():
    o = rb.overtravel_check(Hardpoints.default(), bump_mm=50.0)
    assert o["ok"] and o["monotonic"]
    assert 10.0 < o["transmission_min_deg"] and o["transmission_max_deg"] < 170.0
