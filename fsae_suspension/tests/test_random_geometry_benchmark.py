# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""500 random corners against Project Chrono: the recorded run passes the limits the script set, and the limits in
the record are the script's own (nobody loosened them after the run)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import random_geometry_benchmark as rg                                             # noqa: E402


def test_the_recorded_run_passes_on_every_corner():
    r = json.loads((rg.OUT / "summary.json").read_text())
    assert r["passed"] and r["compared"] == r["corners"] == 500 and not r["chrono_unsolved"] and not r["failed"]
    assert all(r["worst"][k] <= lim for k, lim in rg.LIMITS.items())


def test_the_record_holds_the_script_s_limits_and_seed():
    r = json.loads((rg.OUT / "summary.json").read_text())
    assert r["limits"] == rg.LIMITS and r["seed"] == rg.SEED and r["spread_mm"] == rg.SPREAD
