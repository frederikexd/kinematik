# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Robustness to tire heat, clevis compliance and tie-rod service."""
import math

import pytest

from suspension import robustness as rb
from suspension.kinematics import Hardpoints


def test_neutral_share_and_its_weak_authority():
    nom = rb.balance_under_tire_change(1.0, 1.0)
    assert nom["neutral_share"] == pytest.approx(0.51, abs=0.01)            # the paper's 51% neutral share
    hot = rb.balance_under_tire_change(0.97, 1.0)
    assert hot["neutral_share"] < nom["neutral_share"] - 0.05              # 3% front grip moves neutral far
    assert abs(hot["front_rc_change_mm"]) > 100.0                           # beyond any roll-centre band
    rear = rb.balance_under_tire_change(1.0, 0.95)
    assert rear["neutral_share"] > nom["neutral_share"]                     # hot rears move it the other way


def test_clevis_deflection_moves_statics_not_gains():
    s = rb.clevis_sensitivity(Hardpoints.default(), 1.0)
    for v in s.values():
        assert abs(v["camber_gain"]) < 0.002                                 # gains barely move
    tr = abs(s["tie_rod_inner"]["toe"])
    assert 0.3 < tr < 3.0                                                     # toe follows roughly 1 mm over the steering arm


def test_preset_spare_inside_the_toe_band():
    p = rb.preset_tie_rod_toe(69.4)
    assert p["preset_error_deg"] == pytest.approx(math.degrees(math.atan(0.05 / 69.4)))
    assert p["preset_error_deg"] < p["half_flat_error_deg"] < p["band_deg"]
