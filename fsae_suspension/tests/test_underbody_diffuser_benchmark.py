# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The underfloor model against Southampton's diffuser bluff body (benchmarks/underbody_diffuser). Registered: a
large miss on the high side; pinned so a change to the model shows up here."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import underbody_diffuser_benchmark as ub  # noqa: E402


def test_registered_miss_is_high_and_kept():
    r = ub.run()
    assert not r["passed"]
    assert all(x["rel"] > 5.0 for x in r["rows"])          # every point at least 6 times the measured downforce
    assert json.loads((ub.BENCH / "results_first_run.json").read_text())["passed"] is False


def test_reruns_with_open_sides_also_fail_and_are_kept():
    r = ub.run()
    assert not r["rerun_2"]["passed"] and all(x["rel"] > 3.0 for x in r["rerun_2"]["rows"])
    r3 = r["rerun_3"]
    assert not r3["passed"] and 0.05 < r3["eta_fitted"] < 0.2
