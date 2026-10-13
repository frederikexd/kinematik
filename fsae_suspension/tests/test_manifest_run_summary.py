# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""A manifest re-run in the studio reports its grade, yield and record match beside the Run button."""
import json
from pathlib import Path
from types import SimpleNamespace

from suspension.genesis_repro import GenesisManifest
from ui import inverse_genesis as uig

ROOT = Path(__file__).resolve().parents[1]


class _St:
    def __init__(self):
        self.out = []

    def __getattr__(self, name):
        return lambda *a, **k: self.out.append((name, " ".join(str(x) for x in a)))


def _man():
    return GenesisManifest.from_json((ROOT / "manifests" / "rear_v11_final.recorded.genesis.json").read_text())


def test_the_grade_yield_and_record_match_are_shown():
    m = _man()
    w = SimpleNamespace(verdict="RESILIENT", yield_frac=1.0, max_band_frac=0.41, worst_row="camber @ +25 mm")
    run = {"result": SimpleNamespace(winner=w, candidates=[w], start_search=None), "verify": (True, [])}
    st = _St()
    uig._manifest_run_summary(st, run, m)
    text = " ".join(t for _, t in st.out)
    assert "RESILIENT" in text and "build yield" in text and "Matches the recorded outputs" in text
    assert "Recorded in the manifest: RESILIENT" in text


def test_no_fit_and_a_mismatch_are_said_plainly():
    m = _man()
    bf = SimpleNamespace(max_band_frac=1.26, worst_row="camber @ +25 mm")
    run = {"result": SimpleNamespace(winner=None, candidates=[], best_fit=bf, start_search={
        "evaluations": 964, "walls_met": True}), "verify": (False, ["winner moved"])}
    st = _St()
    uig._manifest_run_summary(st, run, m)
    text = " ".join(t for _, t in st.out)
    assert "NO_FIT" in text and "964 evaluations" in text and "Differs from the recorded outputs" in text


def test_the_recorded_verdict_is_shown_as_soon_as_a_manifest_is_loaded():
    line = uig.recorded_verdict_line(_man())
    assert line.startswith("Recorded result: RESILIENT") and "build yield 100.0%" in line
    nofit = GenesisManifest.from_json((ROOT / "manifests" / "front_v11_from_delivered.recorded.genesis.json").read_text())
    assert "NO_FIT" in uig.recorded_verdict_line(nofit)
