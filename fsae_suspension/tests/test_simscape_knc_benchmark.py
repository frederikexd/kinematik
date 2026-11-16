# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""KinematiK on MathWorks' quarter-car K&C rig: the recorded comparison is what the script computes from the rig's
files, the limits are the ones set before the first run, the fails are recorded as fails, and the two diagnostics that
explain them still hold."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import simscape_knc_benchmark as knc                                                # noqa: E402

REC = json.loads((knc.BENCH / "results.json").read_text())
MEASURED = ("Toe", "Camber", "Bump Steer", "Bump Camber", "Roll Steer", "Roll Camber", "Damper Ratio",
            "toe curve", "camber curve")
ESTIMATED = ("Caster", "KPI", "Caster Trail", "Scrub Radius")


def test_the_limits_are_the_ones_set_before_the_first_run():
    assert knc.LIMITS == {"Toe": 0.01, "Camber": 0.01, "Caster": 0.05, "KPI": 0.05, "Caster Trail": 0.5,
                          "Scrub Radius": 0.5, "Bump Steer": 0.05, "Bump Camber": 0.05, "Roll Steer": 0.01,
                          "Roll Camber": 0.01, "Damper Ratio": 0.005, "toe curve": 0.01, "camber curve": 0.01}


@pytest.mark.parametrize("name", knc.CORNERS)
def test_the_record_is_what_the_rig_files_give(name):
    r = knc.compare(name)
    for q, row in r["rows"].items():
        assert row["passed"] == REC[name]["rows"][q]["passed"]
        assert row["diff"] == pytest.approx(REC[name]["rows"][q]["diff"], abs=1e-6)


@pytest.mark.parametrize("name", knc.CORNERS)
def test_every_measured_quantity_passes_and_the_fails_stay_recorded(name):
    rows = REC[name]["rows"]
    assert all(rows[q]["passed"] for q in MEASURED)
    assert not REC[name]["passed"]                                  # caster, KPI and trail miss on both corners
    assert not rows["Caster"]["passed"] and not rows["KPI"]["passed"] and not rows["Caster Trail"]["passed"]


@pytest.mark.parametrize("name", knc.CORNERS)
def test_the_rig_s_steered_wheel_is_kinematik_s_with_iso_camber(name):
    """A sample of the steering test, recomputed: toe, ISO camber and the wheel centre match; the front-view
    projection KinematiK used before does not."""
    m = np.genfromtxt(knc.BENCH / f"{name}_steer.csv", delimiter=",", names=True)
    i = np.linspace(0, len(m) - 1, 40).astype(int)
    k = knc._steered(name, m["rack_mm"][i])
    assert np.max(np.abs(m["toe_in_deg"][i] - k[:, 4])) < 1e-5
    assert np.max(np.abs(m["camber_deg"][i] - k[:, 5])) < 1e-4
    assert np.max(np.abs(m["camber_deg"][i] - k[:, 6])) > 0.03
    sm = REC[name]["steering_motion_worst"]
    assert sm["camber"] < 1e-4 and sm["toe_in"] < 1e-5 and sm["wc_x"] < 1e-3 and sm["wc_y"] < 1e-3


@pytest.mark.parametrize("name", knc.CORNERS)
def test_mathworks_formulas_on_kinematik_s_motion_give_back_the_rig_s_estimates(name):
    f, mw = REC[name]["mathworks_formulas_on_kinematik"], knc.mathworks(name)
    for q in ("Caster", "Hub Long Offset", "Hub Lat Offset", "Caster Trail"):
        assert f[q] == pytest.approx(mw[q], abs=1e-3), q
    rows = REC[name]["rows"]
    assert abs(f["trail_geometric"] - rows["Caster Trail"]["kinematik"]) < 0.5      # with the geometric sign
    assert abs(f["scrub_from_kpi"] - rows["Scrub Radius"]["kinematik"]) < 0.5       # with KinematiK's KPI


def test_pratt_fit_finds_a_circle_s_centre():
    a = np.linspace(-0.4, 0.3, 50)
    xy = np.c_[3.0 + 30.0 * np.cos(a), -2.0 + 30.0 * np.sin(a)]
    assert np.allclose(knc._pratt(xy), [3.0, -2.0], atol=1e-9)


def test_swing_caster_reads_a_known_axis():
    """A wheel steered about an axis with 4 deg of caster and no KPI or camber: the swing formula returns 4 deg."""
    c = math.radians(4.0)
    ax = np.array([math.sin(c), 0.0, math.cos(c)])                  # x forward, top rearward is -x: sign below
    toe, cam = [], []
    for d in np.radians(np.linspace(-3, 3, 601)):
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        R = np.eye(3) + math.sin(d) * K + (1 - math.cos(d)) * K @ K
        s = R @ np.array([0.0, 1.0, 0.0])                           # spin axis, left wheel, outboard +y
        toe.append(math.degrees(math.atan2(s[0], s[1])))
        cam.append(math.degrees(math.asin(s[2])))
    assert abs(abs(knc._swing_caster(np.array(toe), np.array(cam), 1.0)) - 4.0) < 0.01
