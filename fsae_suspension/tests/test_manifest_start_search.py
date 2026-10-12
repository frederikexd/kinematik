# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""A manifest that declares its start search: a start that breaks a wall is searched from, not refused, for any
corner, and a re-run repeats both stages (paper section 2, supplement S13.5)."""
import json

import numpy as np
import pytest

from suspension.kinematics import Hardpoints
from suspension.kinematik_stochastic import _perturbed
from suspension import inverse_genesis as ig
from suspension import genesis_repro as gr
from suspension import genesis_seed as gs

ST = np.array([-25.0, -12.5, 0.0, 12.5, 25.0])
UF, UR = "upper_front_inner", "upper_rear_inner"


def _declared(gap_extra_mm: float = 6.0) -> gr.GenesisManifest:
    """Default corner, curves of a nearby geometry, and a spacing wall the start breaks by ``gap_extra_mm``."""
    hp = Hardpoints.default()
    truth, _ = ig.curves_of(_perturbed(hp, {UF: np.array([-3.0, -2.0, 3.0]), UR: np.array([3.0, -2.0, 3.0])}), ST)
    tg = ig.GenesisTargets(curves=[ig.TargetCurve("camber_deg", ST, truth["camber_deg"], np.full(5, 0.15)),
                                   ig.TargetCurve("rc_height_mm", ST, truth["rc_height_mm"], np.full(5, 6.0))])
    gap = ig.PointSpacing(UF, UR, "x", 0.0).gap(hp)
    vol = ig.LegalVolume(boxes=gr.boxes_about(hp, {UF: 10.0, UR: 10.0}),
                         spacings=[ig.PointSpacing(UF, UR, "x", gap + gap_extra_mm)])
    return gr.GenesisManifest.build("start_breaks_spacing", hp, tg, vol,
                                    gr.field_with_overrides("jig_weld", {}),
                                    gr.SearchSettings(seed=3, n_starts=2, n_yield=300, max_iter=12))


@pytest.fixture(scope="module")
def searched():
    base = _declared()
    hp0 = base.objects()[0]
    m = base.restarted("start_breaks_spacing_searched", hp0,
                       gs.SeedSearch(seed=1, max_evals=300, step0_mm=4.0, step_min_mm=0.25),
                       {"run": "test"})
    res = m.run()
    m.record(res)
    return base, m, res


def test_the_start_breaks_the_wall():
    m = _declared()
    hp, _, vol, _ = m.objects()
    assert vol.keepout_violations(hp)


def test_declared_start_search_reaches_the_walls_and_the_search_runs_from_there(searched):
    base, m, res = searched
    rep = m.recorded["start_search"]
    assert rep["walls_hold"] and rep["shift_mm"] and rep["evaluations"] > 1
    hp, _, vol, _ = m.objects()
    assert vol.keepout_violations(hp)                         # the declared start still breaks it
    if res.winner_hp is not None:                             # whatever the search returns meets the wall
        assert not vol.keepout_violations(res.winner_hp)


def test_rerun_repeats_both_stages(searched):
    _, m, _ = searched
    m2 = gr.GenesisManifest.from_json(m.to_json())
    assert m2.start_search == m.start_search and m2.inputs_sha256 == m.inputs_sha256
    same, diffs = m2.verify(m2.run())
    assert same, diffs


def test_start_search_is_an_input_and_absent_means_the_old_hash(searched):
    base, m, _ = searched
    d = json.loads(m.to_json())
    d["start_search"]["seed"] = 2
    with pytest.raises(ValueError, match="sha256"):
        gr.GenesisManifest.from_json(json.dumps(d))
    plain = base.restarted("x", base.objects()[0])           # no start search declared
    assert "start_search" not in json.loads(plain.to_json())
    assert plain.inputs_sha256 == base.inputs_sha256          # the same inputs, the same hash as before


def test_restarted_keeps_every_other_input(searched):
    base, m, _ = searched
    for k in ("targets", "volume", "tolerance"):
        assert getattr(m, k) == getattr(base, k)
    assert m.search.__dict__ == base.search.__dict__
    assert m.context["declared_from"].startswith("start_breaks_spacing (inputs ")


def test_a_start_inside_the_walls_is_left_where_it_is():
    m = _declared(gap_extra_mm=-5.0)                          # the wall already holds
    hp, tg, vol, _ = m.objects()
    out, rep = gs.seed_search(hp, tg, vol, gs.SeedSearch(seed=1, max_evals=50, walls_only=True))
    assert rep["walls_hold"] and rep["evaluations"] == 1 and not rep["shift_mm"]


def test_candidate_shifts_are_measured_from_the_search_start(searched):
    from suspension import inverse_genesis as ig
    _, m, res = searched
    hp0 = m.search_start()
    hp_decl = m.objects()[0]
    moved = [p for p in ("upper_front_inner", "upper_rear_inner")
             if not np.allclose(getattr(hp0, p), getattr(hp_decl, p))]
    assert moved                                              # the search started where the start search ended
    if res.winner is not None:
        h = ig._shifted(hp0, m.objects()[2], np.asarray(m.recorded["candidates"][0]["shift_vec"], float))
        for p in ("upper_front_inner", "upper_rear_inner"):
            assert np.allclose(getattr(h, p), m.recorded["winner_hardpoints"][p], atol=1e-6)
