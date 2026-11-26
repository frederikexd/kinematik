# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Rotor temperature against a measured dynamometer stop (benchmarks/brake_rotor_measured). The registered result
is one pass and one miss; these tests pin both, so neither can change silently."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import brake_rotor_measured_benchmark as b  # noqa: E402


def test_registered_outcome_is_one_pass_one_miss():
    r = b.run()
    assert r["test_1"]["passes"] and not r["test_2"]["passes"] and not r["passed"]
    assert abs(r["test_2"]["diff_vs_measured_C"] - 5.6) < 0.2      # the miss, as registered


def test_solver_matches_the_exact_finite_slab():
    # the difference from the paper's model is the geometry (finite slab vs half-space), not the solver
    for n in ("test_1", "test_2"):
        tc = b.INP[n]
        assert abs(b.kinematik(tc)["max_1mm_C"] - b.exact_1mm(tc, 0.0275)) < 0.1
        assert abs(b.exact_1mm(tc, None) - tc["paper_model_max_C"]) < 0.5


def test_first_run_kept():
    first = json.loads((b.BENCH / "results_first_run.json").read_text())
    assert first["passed"] is False
