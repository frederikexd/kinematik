# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Supplement S6.30: KinematiK against Lotus Suspension Analysis on the published hardpoints of Arevalo et al.
(2018). Lotus values are read from the source's figures; KinematiK's must stay at the published values."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import lotus_benchmark as lb                                              # noqa: E402

LOTUS_BUMP = {160.0: {-30: (1.414, 0.0062), -20: (0.964, 0.0007), -10: (0.492, -0.0015), 10: (-0.524, 0.0054),
                      20: (-1.065, 0.0150), 30: (-1.631, 0.0291)},
              1760.0: {-30: (0.964, 0.1388), -20: (0.660, 0.0734), -10: (0.340, 0.0270), 10: (-0.356, -0.0078),
                       20: (-0.731, 0.0033), 30: (-1.129, 0.0327)}}
LOTUS_ROLL = {160.0: {-3: 1.276, -2: 0.879, -1: 0.449, 1: -0.476, 2: -0.988, 3: -1.512},
              1760.0: {-3: 1.827, -2: 1.240, -1: 0.630, 1: -0.646, 2: -1.315, 3: -2.004}}
LOTUS_RC = {160.0: {-30: 90.9, -20: 75.8, -10: 60.8, 0: 46.0, 10: 31.4, 20: 17.0, 30: 3.0},
            1760.0: {-30: 110.2, -20: 95.3, -10: 80.5, 0: 66.1, 10: 51.8, 20: 37.8, 30: 23.8}}
LOTUS_STEER = {-31.5: 5.507, -30: 5.146, -20: 2.920, -10: 1.302, 10: -1.051, 20: -1.925, 30: -2.647, 31.5: -2.744}


@pytest.mark.parametrize("xa", [160.0, 1760.0])
def test_bump_camber_and_toe_against_lotus(xa):
    k = lb.bump(xa)
    cam = max(abs(k[d][0] - LOTUS_BUMP[xa][d][0]) for d in LOTUS_BUMP[xa])
    toe = max(abs(k[d][1] - LOTUS_BUMP[xa][d][1]) for d in LOTUS_BUMP[xa])
    assert cam <= 0.0095 and toe <= 0.0004                    # table S6.30a: 0.009 deg camber, 0.0003 deg toe


@pytest.mark.parametrize("xa,track", [(160.0, 1200.0), (1760.0, 1180.0)])
def test_roll_camber_against_lotus(xa, track):
    k = lb.roll(xa, track)
    assert max(abs(k[p] - LOTUS_ROLL[xa][p]) for p in LOTUS_ROLL[xa]) <= 0.0095


def test_steer_camber_against_lotus():
    diffs = {a: lb.steer(160.0, a)["on_ground"] - v for a, v in LOTUS_STEER.items()}
    assert max(abs(d) for d in diffs.values()) == pytest.approx(0.050, abs=0.0015)     # table S6.30e, at -20 deg
    assert lb.steer(160.0, -30.0)["held"] == pytest.approx(5.342, abs=0.0015)


@pytest.mark.parametrize("xa,track", [(160.0, 1200.0), (1760.0, 1180.0)])
def test_roll_centre_against_lotus(xa, track):
    k = lb.roll_centre(xa, track)
    assert max(abs(k[d] - LOTUS_RC[xa][d]) for d in LOTUS_RC[xa]) == pytest.approx(3.3 if xa == 1760.0 else 2.1, abs=0.06)
