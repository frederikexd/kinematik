# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The parts the links load: each check against its hand calculation, and the upright in equilibrium."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from suspension import corner_parts as cp                          # noqa: E402
from suspension import genesis_repro as gr                         # noqa: E402
from suspension.kinematics import Hardpoints                       # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")


def _rows():
    return [{"load_case": "1.5g corner", "member": "LF", "force_N": -400.0},
            {"load_case": "1.5g braking", "member": "LF", "force_N": 7000.0},
            {"load_case": "3g vertical bump", "member": "LF", "force_N": 9000.0},     # shock: not in fatigue
            {"load_case": "1.5g corner", "member": "UF", "force_N": 600.0}]


@pytest.mark.parametrize("pushrod_on_upright", [True, False])
def test_the_upright_joint_loads_balance_the_tyre_load(pushrod_on_upright):
    if pushrod_on_upright:
        man = gr.GenesisManifest.from_json(open(os.path.join(ROOT, "manifests",
                                                             "front_v11_final.recorded.genesis.json")).read())
        hp = man.hardpoints() if callable(getattr(man, "hardpoints", None)) else man.objects()[0]
    else:
        hp = Hardpoints.default()
    cases = gr.vehicle_load_cases(axle="front", brake_bias_front=0.73)
    loads = cp.upright_loads(hp, cases)
    assert loads and max(r["residual_N"] for r in loads) < 1e-6
    assert {r["load_case"] for r in loads} == {c.name for c in cases}


def test_arm_bending_bearing_split_and_bracket_by_hand():
    loads = [{"load_case": "c", "joint": "lower ball joint", "point": np.array([0.0, 0.0, -100.0]),
              "F": np.array([1000.0, 0.0, 0.0])}]
    a = cp.upright_arms(loads, [0, 0, 0], 500.0, {"lower ball joint": 400.0}, fos_target=2.0)[0]
    assert a["moment_Nmm"] == pytest.approx(1e5) and a["Z_required_mm3"] == pytest.approx(400.0)
    assert a["fos"] == pytest.approx(2.0)
    lc = gr.LoadCaseSpec("c", Fz=1000.0, Fy=-500.0)
    b = cp.hub_bearings([lc], tyre_radius_mm=200.0, spacing_mm=50.0, wheel_offset_mm=0.0, c0_kN=10.0)[0]
    fr = max(abs(500.0 + m) for m in (-500.0 * 200.0 / 50.0, 500.0 * 200.0 / 50.0))   # Fz/2 ± Fy R / s
    assert b["Fr_N"] == pytest.approx(fr) and b["Fa_N"] == pytest.approx(500.0)
    assert b["P0_N"] == pytest.approx(max(0.6 * fr + 0.5 * 500.0, fr))
    assert b["s0"] == pytest.approx(10e3 / b["P0_N"])
    br = {r["member"]: r for r in cp.brackets(_rows(), 20.0, 30.0, 5.0, 2, 435.0, 1.28)}
    assert br["LF"]["force_N"] == 9000.0                            # the worst force, shock included
    assert br["LF"]["stress_mpa"] == pytest.approx(1.28 * 4500.0 * 20.0 / (30.0 * 25.0 / 6.0))


def test_fatigue_leaves_out_the_shock_cases_and_follows_the_design_life():
    w = {r["member"]: r for r in cp.link_weld_fatigue(_rows(), {"LF": 50.0, "UF": 50.0}, 71.0, cycles=2e6)}
    assert w["LF"]["range_MPa"] == pytest.approx(7400.0 / 50.0)       # -400 … 7000, bump left out
    assert w["LF"]["limit_MPa"] == pytest.approx(71.0)
    w5 = cp.link_weld_fatigue(_rows(), {"LF": 50.0, "UF": 50.0}, 71.0, cycles=1e5)[0]
    assert w5["limit_MPa"] == pytest.approx(71.0 * 20 ** (1 / 3))
    winf = cp.link_weld_fatigue(_rows(), {"LF": 50.0, "UF": 50.0}, 71.0, cycles=1e8)[0]
    assert winf["limit_MPa"] == pytest.approx(cp.CAFL_FRACTION * 71.0)
    r = {x["member"]: x for x in cp.rod_end_fatigue(_rows(), 9.525, 24, 9.5, mu_ball=0.1, cycles=2e6)}
    d3 = 9.525 - 1.226869 * 25.4 / 24
    assert r["LF"]["force_N"] == 7000.0
    assert r["LF"]["stress_amp_MPa"] == pytest.approx(0.1 * 7000.0 * 9.5 / (math.pi * d3 ** 3 / 32))
    assert r["LF"]["endurance_amp_MPa"] == pytest.approx(0.85 * (150 / 9.525 + 45))
