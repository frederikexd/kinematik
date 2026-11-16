# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Two published Lotus SHARK studies, scored by the rules set before the run. The passes and the fails are both
pinned: a fail must not quietly turn into a pass by editing the source tables or the limits."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import published_lotus_benchmark as pl                                              # noqa: E402


@pytest.fixture(scope="module")
def a():
    return pl.compare_a()


def test_camber_and_toe_agree_with_lotus_within_the_limits(a):
    assert a["camber"]["passed"] and a["camber"]["worst"] < 0.005 and a["camber"]["scored"] == 49
    assert a["toe"]["passed"] and a["toe"]["worst"] < 0.01 and a["toe"]["scored"] == 26


def test_the_misses_stay_misses(a):
    assert not a["caster"]["passed"] and not a["kpi"]["passed"] and not a["damper_ratio"]["passed"]
    b = pl.compare_b()
    assert not b["camber"]["passed"] and not b["kpi"]["passed"]
    assert abs(b["camber"]["kinematik"][-30] - b["camber"]["lotus"][-30]) < 0.005      # it misses only at +30 mm
    assert abs(b["kpi"]["kinematik"][-30] - b["kpi"]["lotus"][-30]) < 0.005


def test_chrono_solves_the_published_hardpoints_as_kinematik_does():
    """The reason the misses are the papers': an independent solver agrees with KinematiK on both corners."""
    assert max(pl.chrono_agrees().values()) < 1e-4


def test_the_unscored_toe_rows_match_as_ten_thousandths_of_a_degree():
    """A's toe from +1 mm is printed without decimals ('836' at +25 mm): not scored, but KinematiK reads 0.0836."""
    import numpy as np
    k = pl.curves(pl.corner(pl.A_PTS), np.array([12.0, 25.0]))
    assert k["toe"][0] == pytest.approx(0.0220, abs=2e-4) and k["toe"][1] == pytest.approx(0.0836, abs=2e-4)
