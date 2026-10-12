# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Supplement S13: the method benchmarks behind the paper's section 6.3."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import method_benchmarks as mb                                            # noqa: E402


def test_known_answer_recovery_meets_the_curves_not_the_pickups():
    rows = mb.recovery(ks=(2,), draws=3, seed=0, with_lsq=False, log=lambda *a: None)
    for r in rows:
        assert r["start_band"] > 1.0                                       # the start is outside the tight bands
        assert r["ig"]["hit"] and r["ig"]["band"] <= 1.0                   # recovered inside every band
        e = r["ig"]["curve_err11"]
        assert e["camber_deg"] < 0.05 and e["toe_deg"] < 0.02 and e["rc_height_mm"] < 2.0   # also between the target stations
    assert max(r["ig"]["pickup_rms_mm"] for r in rows) > 1.0               # the pickups are not identified by the curves


def test_curves_only_baseline_meets_the_curves_and_fails_the_walls():
    m = mb._manifest("front_v11_curves_only")
    assert m.inputs_sha256 == mb.build_curves_only().inputs_sha256
    assert m.inputs_sha256.startswith("c320eac46d0f")
    j = mb.judge_curves_only(m)
    hits = [c for c in j["candidates"] if c["hit"]]
    assert len(hits) == 4
    win = next(c for c in hits if c["winner"])
    assert win["shift_max_mm"] == pytest.approx(0.0, abs=1e-9)           # the delivered front, unchanged
    # judged against front_v11_final's walls: on the corrected statics the delivered front's wishbone-mounted pushrod
    # also fails the link and bearing walls
    assert set(win["violated"]) == {"lock_sw_deg", "ackermann_lock_pct", "link_fos_min", "rocker_bearing_sf",
                                    "compliance_toe_braking_deg"}
    assert round(win["properties"]["compliance_toe_braking_deg"], 2) == 0.49       # against the 0.25 deg braking wall
    assert round(win["properties"]["link_fos_min"], 2) == 0.27
    assert sorted(round(c["properties"]["link_fos_min"], 2) for c in hits if not c["winner"]) == [0.02, 0.08, 0.43]
    assert win["properties"]["lock_sw_deg"] == pytest.approx(137.1, abs=0.1)
    for c in hits:
        assert c["violated"] or c["envelope_margin_mm"] < 0                # no in-band candidate meets the walls
    others = [c for c in hits if not c["winner"]]
    assert sum(c["envelope_margin_mm"] < 0 for c in others) == 3
    assert max(c["properties"]["anti_dive_pct"] for c in others) == pytest.approx(395.6, abs=0.1)
