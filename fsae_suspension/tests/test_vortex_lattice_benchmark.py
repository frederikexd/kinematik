"""The vortex lattice against Tuck's (1993) exact lift slopes (scripts/vortex_lattice_benchmark.py)."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed():
    res = json.loads((ROOT / "benchmarks" / "vortex_lattice" / "results.json").read_text())
    assert res["passed"] and set(res["cases"]) == {"1", "2", "4", "8"}


@pytest.mark.parametrize("ar,tuck", [(1, 1.46023), (8, 4.58606)])
def test_lift_slope_live(ar, tuck):
    import vortex_lattice_benchmark as vb
    assert vb.slope(float(ar), ns=20, nc=6) == pytest.approx(tuck, rel=0.05)
