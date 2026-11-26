"""Characterization suite for suspension/corner_parts.py (mutation gate); see tests/_golden.py. Upright joint loads
for a pushrod-on-upright corner and the default corner over the vehicle load cases, upright arms with and without
declared sections (including a joint with no point and zero moments), hub bearings with and without an offset,
wheel-centre force and a declared rating, brackets at each member's worst force, weld fatigue either side of the
constant-amplitude limit and at and past 1e7 cycles, and rod-end fatigue below and above the knee."""
import os

import numpy as np

from suspension import corner_parts as cp
from suspension import genesis_repro as gr
from suspension.kinematics import Hardpoints
from tests._golden import plain
from tests._golden_lazy import check_lazy

ROOT = os.path.join(os.path.dirname(__file__), "..")


def _rows():
    return [{"load_case": "1.5g corner", "member": "LF", "force_N": -400.0},
            {"load_case": "1.5g braking", "member": "LF", "force_N": 7000.0},
            {"load_case": "3g vertical bump", "member": "LF", "force_N": 9000.0},
            {"load_case": "Kerb strike", "member": "UF", "force_N": -20000.0},
            {"load_case": "POTHOLE", "member": "TR", "force_N": 5000.0},
            {"load_case": "1.5g corner", "member": "UF", "force_N": 600.0},
            {"load_case": "1.5g braking", "member": "UF", "force_N": -650.0},
            {"load_case": "steady", "member": "TR", "force_N": 0.0},
            {"load_case": "curb ride", "member": "PR", "force_N": 100.0},
            {"load_case": "strike", "member": "PR", "force_N": 50.0},
            {"load_case": "tie", "member": "LF", "force_N": -9000.0},
            {"load_case": "late", "member": "LF", "force_N": 100.0},
            {"load_case": "tiny", "member": "Z", "force_N": 0.5}]


def _cases():
    man = gr.GenesisManifest.from_json(open(os.path.join(ROOT, "manifests",
                                                         "front_v11_final.recorded.genesis.json")).read())
    hp_up = man.hardpoints() if callable(getattr(man, "hardpoints", None)) else man.objects()[0]
    cases = gr.vehicle_load_cases(axle="front", brake_bias_front=0.73)
    for name, hp in (("pushrod_upright", hp_up), ("default", Hardpoints.default())):
        loads = cp.upright_loads(hp, cases)
        yield f"loads.{name}", plain(loads)
        wc = np.asarray(hp.wheel_center, float) if hasattr(hp, "wheel_center") else np.zeros(3)
        yield f"arms.{name}", [plain(cp.upright_arms(loads, wc, 500.0)),
                               plain(cp.upright_arms(loads, wc, 500.0, {"upper ball joint": 900.0,
                                                                        "tie-rod end": 0.0}, fos_target=2.0))]
    hand = [{"load_case": "a", "joint": "lower ball joint", "point": np.array([0.0, 0.0, -100.0]),
             "F": np.array([1000.0, 0.0, 0.0])},
            {"load_case": "b", "joint": "lower ball joint", "point": np.array([0.0, 0.0, -100.0]),
             "F": np.array([2000.0, 0.0, 0.0])},
            {"load_case": "c", "joint": "lower ball joint", "point": np.array([0.0, 0.0, -100.0]),
             "F": np.array([1500.0, 0.0, 0.0])},
            {"load_case": "a", "joint": "pushrod", "point": None, "F": np.array([1.0, 0.0, 0.0])},
            {"load_case": "a", "joint": "tie-rod end", "point": np.array([0.0, 0.0, -50.0]),
             "F": np.array([0.0, 0.0, 10.0])},
            {"load_case": "tie", "joint": "lower ball joint", "point": np.array([0.0, 0.0, -100.0]),
             "F": np.array([2000.0, 0.0, 0.0])},
            {"load_case": "a", "joint": "upper", "point": np.array([0.0, 0.0, 100.0]), "F": np.array([0.005, 0.0, 0.0])}]
    yield "arms.hand", [plain(cp.upright_arms(hand, [0, 0, 0], 400.0, {"lower ball joint": 500.0, "tie-rod end": 3.0, "upper": 50.0})),
                        plain(cp.upright_arms(hand, [0, 0, 0], 400.0, None, fos_target=1.0)), cp.upright_arms([], [0, 0, 0], 1.0)]
    lcs = [gr.LoadCaseSpec("v", Fz=1000.0), gr.LoadCaseSpec("c", Fz=1000.0, Fy=-500.0),
           gr.LoadCaseSpec("b", Fz=1200.0, Fx=-900.0, Fx_wc=100.0), gr.LoadCaseSpec("o", Fz=-300.0, Fy=800.0, Fx=400.0)]
    for kw in ({}, {"wheel_offset_mm": 20.0}, {"c0_kN": 12.0}, {"x0": 0.56, "y0": 1.4, "s0_target": 1.5, "c0_kN": 0.0},
               {"wheel_offset_mm": -15.0, "c0_kN": 30.0}):
        yield f"hub.{sorted(kw.items())}", [plain(cp.hub_bearings(lcs, 230.0, 40.0, **kw)),
                                            plain(cp.hub_bearings(lcs, 10.0, 400.0, **kw))]
    yield "brackets", [plain(cp.brackets(_rows(), 25.0, 30.0, 4.0, 2, 300.0, 2.0)),
                       plain(cp.brackets(_rows(), 40.0, 20.0, 3.0, 1.0, 250.0, 1.5)), cp.brackets([], 1, 1, 1, 1, 1, 1)]
    area = {"LF": 60.0, "UF": 55.0, "TR": 40.0, "PR": 50.0, "Z": 1.0}
    for cyc in (1e3, 1e5, 2e6, 5e6, 9.9e6, 1e7, 1e8):
        yield f"weld.{cyc:g}", plain(cp.link_weld_fatigue(_rows(), area, 71.0, cycles=cyc))
    yield "weld.default", plain(cp.link_weld_fatigue(_rows(), area, 90.0))
    for cyc in (1e4, 1e5, 2e6, 1e7):
        yield f"rodend.{cyc:g}", plain(cp.rod_end_fatigue(_rows(), 8.0, 32, 6.0, cycles=cyc))
    yield "rodend.default", plain(cp.rod_end_fatigue(_rows(), 6.35, 28, 5.0, mu_ball=0.2))
    yield "life", [cp.life_factor(c) for c in (1e3, 2e6, 1e7)] + [cp.life_factor(1e6, knee=1e7)]
    yield "consts", [cp.CAFL_FRACTION, cp.SHOCK_WORDS, {k: list(v) for k, v in cp.JOINTS.items()}, cp._JOINT_POINT]


def test_every_corner_part_screen():
    check_lazy("corner_parts", _cases())
