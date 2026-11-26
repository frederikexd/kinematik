# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Cell temperature against a measured air-cooled LFP pack (benchmarks/pack_air_measured). Registered: a miss on
the low side, from heat generation; pinned so it cannot change silently."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import pack_air_measured_benchmark as pb  # noqa: E402


def test_registered_miss_reads_low_and_misses_the_5_it_breach():
    r = pb.run()
    assert not r["passed"] and not r["decision"]["passes"]
    assert all(-0.9 < x["rel"] < -0.5 for x in r["rows"].values())
    assert all(x["kinematik_heat_W"] < 0.7 * x["measured_heat_W"] for x in r["rows"].values())   # the cause
    assert json.loads((pb.BENCH / "results_first_run.json").read_text())["passed"] is False


def test_calibrated_path_predicts_held_out_heat_and_temperatures():
    c = pb.run()["calibrated"]
    assert c["passed"] and abs(c["heat_rel"]) < 0.05
    assert all(abs(c["temps"][r]["rel"]) < 0.15 for r in ("1", "3"))
    assert c["flagged_over_40C"] == ["5"]
