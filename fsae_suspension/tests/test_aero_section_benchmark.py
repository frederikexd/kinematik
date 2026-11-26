"""2D section solvers against the exact Joukowski aerofoil and each other (scripts/aero_section_benchmark.py)."""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "aero_section" / "results.json").read_text())["passed"]


def test_live_run_passes():
    import aero_section_benchmark as ab
    assert ab.run()["passed"]


def test_the_answer_does_not_depend_on_where_the_section_sits():
    """A first version set the self-induced term by a tolerance that failed far from the origin."""
    from suspension.aero import panel2d as p2
    x, z, _ = p2.joukowski_section(0.1, 0.08, 200)
    base = p2.thick_cl(x, z, math.radians(4))
    for dz in (10.0, 250.0, 5000.0):
        assert math.isclose(p2.thick_cl(x + 3.0, z + dz, math.radians(4)), base, rel_tol=1e-9)
