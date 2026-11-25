"""The bolted-joint check against Shigley's published solutions (scripts/bolted_joint_benchmark.py)."""
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "bolted_joint" / "results.json").read_text())["passed"]


def test_live_run_passes():
    import bolted_joint_benchmark as bb
    assert bb.run()["passed"]


def test_shank_in_grip_is_shigley_eq_8_17():
    from suspension.bolted_joint import Fastener, _bolt_stiffness
    E, d, At, grip = 207000.0, 14.0, 115.0, 40.0
    full = _bolt_stiffness(Fastener(nominal_d_mm=d, stress_area_mm2=At), grip, E)
    assert full == pytest.approx(E * At / grip)
    part = _bolt_stiffness(Fastener(nominal_d_mm=d, stress_area_mm2=At, shank_in_grip_mm=21.0), grip, E)
    Ad = math.pi * d * d / 4
    assert part == pytest.approx(Ad * At * E / (Ad * 19.0 + At * 21.0))
    assert part / 1000 == pytest.approx(686.2578, rel=1e-4)       # Shigley 8-22, MN/m
    whole = _bolt_stiffness(Fastener(nominal_d_mm=d, stress_area_mm2=At, shank_in_grip_mm=99.0), grip, E)
    assert whole == pytest.approx(Ad * E / grip)                  # shank longer than the grip: all shank
