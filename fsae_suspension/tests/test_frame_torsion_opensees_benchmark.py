# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Whole-frame torsion, bending and mount stiffness against OpenSeesPy (benchmarks/frame_torsion_opensees)."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
pytest.importorskip("openseespy")
from scripts import frame_torsion_opensees_benchmark as fb  # noqa: E402


def test_frame_matches_opensees_and_the_stored_result():
    r = fb.run()
    assert r["passed"] and r["n_tubes"] >= 60
    stored = json.loads((fb.BENCH / "results.json").read_text())
    for q, row in r["rows"].items():
        assert row["kinematik"] == pytest.approx(stored["rows"][q]["kinematik"], rel=1e-9)
