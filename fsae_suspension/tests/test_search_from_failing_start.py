# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The search designing from a failing start (paper section 3.2, supplement S13.5).

front_v11_from_85mm declares every input of front_v11_final with the 85 mm-base candidate (front_v11_29deg) as its
start, which fails the link and braking-toe walls; genesis_seed moves the pickups until every wall holds and
InverseGenesis returns that geometry RESILIENT. front_v11_from_delivered does the same from the delivered front and
ends NO_FIT: the search is local. Both are declared by scripts/design_from_delivered.py.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from suspension import genesis_seed as gs
from suspension.genesis_repro import GenesisManifest, hp_from_dict

ROOT = Path(__file__).resolve().parents[1]
MAN = ROOT / "manifests"
sys.path.insert(0, str(ROOT / "scripts"))
import design_from_delivered as dfd                                       # noqa: E402


def _m(name):
    return GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())


def _seedrep(m):
    return (m.recorded or {}).get("start_search") or m.context["seed_search"]


def _base(hp):
    return float(np.linalg.norm(np.subtract(hp.lower_rear_inner, hp.lower_front_inner)))


def test_the_manifests_carry_every_input_of_the_final_front():
    F = json.loads(_m("front_v11_final").to_json())
    for name in ("front_v11_from_85mm", "front_v11_from_delivered"):
        d = json.loads(_m(name).to_json())
        for k in ("targets", "volume", "tolerance", "search"):
            assert d[k] == F[k], (name, k)
        for k in dfd.ACTUATION:
            assert d["hardpoints"][k] == F["hardpoints"][k]


def test_the_85mm_start_fails_two_walls_and_the_search_meets_them():
    m = _m("front_v11_from_85mm")
    _, tg, vol, _ = m.objects()
    start = dfd.declare("seeded", "29deg").objects()[0]
    assert [w[0] for w in vol.wall_violations(start)] == ["link_fos_min", "compliance_toe_braking_deg"]
    assert round(_base(start)) == 85
    win = hp_from_dict(m.recorded["winner_hardpoints"])
    assert vol.wall_violations(win) == []
    r, ok = tg.residual(win)
    assert ok and np.max(np.abs(r)) == pytest.approx(0.541, abs=0.0005)
    assert m.recorded["winner_verdict"] == "RESILIENT" and m.recorded["winner_yield"] == 1.0
    assert round(_base(win)) == 113
    sr = _seedrep(m)
    assert sr["evaluations"] == 432 and sr["walls_hold"] and sr["start_violated"] == ["link_fos_min", "compliance_toe_braking_deg"]
    v = vol.properties.evaluate(win)
    assert (round(v["link_fos_min"], 2), round(v["compliance_toe_braking_deg"], 3)) == (2.12, 0.247)


def test_from_the_delivered_front_the_search_is_local():
    m = _m("front_v11_from_delivered")
    assert m.recorded["winner_verdict"] is None
    assert m.recorded["candidates"][0]["max_band_frac"] == pytest.approx(1.256, abs=0.0005)
    sr = _seedrep(m)
    assert sr["final_violated"] == [] and sr["final_band"] == pytest.approx(1.298, abs=0.0005)   # walls hold, bands do not


def test_walls_only_seed_stops_as_soon_as_the_walls_hold():
    m = _m("front_v11_final")
    hp, tg, vol, _ = m.objects()
    _, rep = gs.seed_search(hp, tg, vol, gs.SeedSearch(walls_only=True, clearance_target_mm=0.45, max_evals=5))
    assert rep["walls_hold"] and rep["shift_mm"] == {} and rep["final_cost"] == 0.0   # nothing to improve: no move


@pytest.mark.parametrize("name,old", [("front_v11_from_85mm", "e60bfbf64f1c"),
                                      ("front_v11_from_delivered", "a83d1f1a5d7e")])
def test_the_declared_start_search_repeats_the_earlier_two_stage_record(name, old):
    """Re-recorded with the start search inside the manifest: the pattern search lands on the start the earlier
    record stored, and the search from it returns the earlier outputs (coordinates to 1e-6 mm, fits and yields 1e-9)."""
    m = _m(name)
    assert m.start_search is not None and m.start_search["graded_keepouts"] is False
    prev = json.loads((MAN / "superseded" / f"{name}.{old}.recorded.genesis.json").read_text())
    assert m.recorded["start_search"]["evaluations"] == prev["context"]["seed_search"]["evaluations"]
    sh_new, sh_old = m.recorded["start_search"]["shift_mm"], prev["context"]["seed_search"]["shift_mm"]
    assert set(sh_new) == set(sh_old) and all(abs(sh_new[k] - sh_old[k]) < 1e-6 for k in sh_old)
    a, b = prev["recorded"], m.recorded
    assert a["winner_verdict"] == b["winner_verdict"] and len(a["candidates"]) == len(b["candidates"])
    for ca, cb in zip(a["candidates"], b["candidates"]):
        assert abs(ca["max_band_frac"] - cb["max_band_frac"]) <= 1e-9
        assert (ca["yield_frac"] is None) == (cb["yield_frac"] is None)
        assert ca["yield_frac"] is None or abs(ca["yield_frac"] - cb["yield_frac"]) <= 1e-9
        assert np.max(np.abs(np.subtract(ca["shift_vec"], cb["shift_vec"]))) <= 1e-6
