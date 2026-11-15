# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""GenesisManifest.replay(): a recorded result rebuilt in seconds, without the search.

The geometry and fits are recomputed with this build and checked against the record; the build yields are copied
from it and never presented as re-sampled. The hosted app cannot finish a full search, so this is how a team sees a
published result there."""
import json
from pathlib import Path

import numpy as np
import pytest

from suspension import genesis_repro as gr
from suspension import inverse_genesis as ig

MAN = Path(__file__).resolve().parents[1] / "manifests"


def _load(name):
    return gr.GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())


@pytest.mark.parametrize("name", ["front_v11_final", "rear_v11_final", "table7_as40",
                                  "front_v11_from_delivered", "front_v11_29deg_b60_brake"])
def test_a_replay_reproduces_the_record(name):
    """RESILIENT front and rear, a TEMPERED result, a NO_FIT reached after a start search, and a NO_FIT with no
    candidate at all."""
    man = _load(name)
    res = man.replay()
    same, diffs = man.verify(res)
    assert same, diffs
    assert res.warnings == [gr.REPLAY_NOTE]
    rec = man.recorded
    assert (res.winner.verdict if res.winner else None) == rec["winner_verdict"]
    assert [c.yield_frac for c in res.candidates] == [c["yield_frac"] for c in rec["candidates"]]


def test_the_fits_are_recomputed_not_copied():
    """The replayed fit comes from the kinematics at the recorded coordinates: a record whose coordinates were moved
    no longer reproduces its fit, and a record whose fit was edited is caught."""
    man = _load("rear_v11_final")
    rec = man.recorded
    moved = json.loads(json.dumps(rec))
    moved["candidates"][0]["shift_vec"][0] += 3.0
    man_moved = gr.GenesisManifest.from_json(man.to_json())
    man_moved.recorded = moved
    res = man_moved.replay()
    assert res.candidates[0].max_band_frac != pytest.approx(rec["candidates"][0]["max_band_frac"], abs=1e-6)

    edited = json.loads(json.dumps(rec))
    edited["candidates"][0]["max_band_frac"] += 0.01
    man_edited = gr.GenesisManifest.from_json(man.to_json())
    man_edited.recorded = edited
    same, diffs = man_edited.verify(man_edited.replay())
    assert not same and any("max_band_frac" in d for d in diffs)


def test_the_replayed_winner_is_the_recorded_geometry_with_its_properties():
    man = _load("front_v11_final")
    res = man.replay()
    w = man.recorded["winner_hardpoints"]
    assert all(np.allclose(getattr(res.winner_hp, p), w[p], atol=1e-9) for p in gr._HP_POINTS)
    assert res.winner is not None and res.winner.properties                      # the solved properties are there
    assert res.best_fit is min(res.candidates, key=lambda c: c.max_band_frac)
    _, tg, _, _ = man.objects()
    r, ok = tg.residual(res.winner_hp)
    assert ok and float(np.max(np.abs(r))) == pytest.approx(res.winner.max_band_frac, abs=1e-12)


def test_a_manifest_without_recorded_outputs_cannot_be_replayed():
    man = _load("rear_v11_final")
    man.recorded = None
    with pytest.raises(ValueError, match="no recorded outputs"):
        man.replay()


def test_a_replay_does_not_search():
    """No yield sampling and no multi-start: the replay of the 21-minute front manifest is seconds."""
    import time
    man = _load("front_v11_final")
    calls = []
    orig = ig.inverse_genesis
    ig.inverse_genesis = lambda *a, **k: calls.append(1) or orig(*a, **k)
    try:
        t = time.perf_counter()
        man.replay()
        assert time.perf_counter() - t < 60 and not calls
    finally:
        ig.inverse_genesis = orig
