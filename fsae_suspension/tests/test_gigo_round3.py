"""Garbage-in benchmark, round 3 (held-out decks): its sealed catalog, its arithmetic and, once run, its result."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import gigo_round3 as r3  # noqa: E402


def test_the_seal_holds_the_catalog_the_runner_uses():
    rec = json.loads(r3.SEAL.read_text())
    assert rec["corruptions"] == json.loads(json.dumps(r3.systematic() + r3.curated(), ensure_ascii=False))
    assert rec["predictions"] == r3.PREDICTIONS and rec["random"]["n"] == r3.N_RANDOM
    assert set(rec["code_sha256"]) == set(r3.SEALED_CODE) and rec["decks"] == ["fastsim:bolt", "fsae_ev", "fsae_ice"]


def test_none_of_the_held_out_decks_was_used_before():
    import gigo_benchmark as gb
    assert "bolt" not in r3.fb.VEHICLES and r3.BOLT[0] not in {v[0] for v in r3.fb.VEHICLES.values()}
    assert r3.DECK / "inputs.json" not in {gb.FS_DECK / gb.FS_LAP / "inputs.json", gb.FS_DECK / gb.FS_LAUNCH / "inputs.json"}


@pytest.mark.parametrize("k, n, lo, hi", [(0, 10, 0.0, 0.2775), (5, 10, 0.2366, 0.7634), (10, 10, 0.7225, 1.0),
                                          (65, 119, 0.4567, 0.6328)])
def test_wilson_intervals(k, n, lo, hi):
    a, b = r3.wilson(k, n)
    assert a == pytest.approx(lo, abs=1e-4) and b == pytest.approx(hi, abs=1e-4)


def test_wilson_with_no_trials_is_undefined():
    import math
    assert all(math.isnan(x) for x in r3.wilson(0, 0))


def test_the_catalog_reaches_every_held_out_deck_and_field():
    rows = r3.systematic() + r3.curated()
    assert {r["deck"] for r in rows} == {"fastsim:bolt", "fsae_ev", "fsae_ice"}
    assert {r["field"] for r in rows if r["deck"] == "fsae_ice"} == set(r3.ICE_FIELDS)
    assert len(r3.systematic()) == 98 and len(r3.curated()) == 30


def test_the_random_set_is_fixed_by_the_seed():
    assert r3.random_set("a") == r3.random_set("a") != r3.random_set("b")
    assert len(r3.random_set("a")) == r3.N_RANDOM


def test_corruptions_on_the_formula_student_decks():
    ev = r3.fs_deck("fsae_ev")
    c = {"field": "cell_v", "how": "times_series"}
    assert r3.corrupt_fs(ev, c, "fsae_ev")["ev"]["pack"]["cell_v"] == pytest.approx(3.6 * 140)
    ice = r3.fs_deck("fsae_ice")
    t = r3.corrupt_fs(ice, {"field": "engine_torque", "how": 1 / 1.35582}, "fsae_ice")
    assert t["_torque_scale"] == pytest.approx(1 / 1.35582) and ice == r3.fs_deck("fsae_ice")
    t = r3.corrupt_fs(ice, {"field": "engine_torque", "how": {"typo": "drop", "draw": 1}}, "fsae_ice")
    assert t["_torque_typo"] == {"typo": "drop", "draw": 1}
    assert r3.corrupt_fs(ice, {"field": "final_drive", "how": "inverse"}, "fsae_ice")["ice"]["final_drive"] == \
        pytest.approx(1 / 3.0)


def test_the_stored_run_grades_as_reported():
    """Round 3 (seed: seal commit 8a02c0f): entry 72 of 106 harmful overall, 64 of 83 on the two held-out Formula
    Student decks; second opinion 23 of 23 on the Bolt; no false alarm. Re-graded only while the code is sealed."""
    import hashlib
    digest, name = (r3.BENCH / "MANIFEST.sha256").read_text().split()
    assert hashlib.sha256((r3.BENCH / name).read_bytes()).hexdigest() == digest
    runs = json.loads(r3.RUNS.read_text())
    assert runs["seed_commit"] == "8a02c0f01e18fd9c983822477a3eb4f8fcaead13"
    if json.loads(r3.SEAL.read_text())["code_sha256"] != r3.code_fingerprint():
        pytest.skip("the code has moved on from the sealed commit; the stored GRADED.txt is the record")
    gr = r3.grade(runs)
    s = r3.summarize(gr)
    assert (s["all"]["errors"], s["all"]["harmful"], s["all"]["gate1"], s["all"]["gate2"], s["all"]["gate2_n"]) == \
        (171, 106, 72, 23, 23)
    assert (s["formula student"]["harmful"], s["formula student"]["gate1"]) == (83, 64)
    assert (s["fsae_ev"]["gate1"], s["fsae_ev"]["harmful"], s["fsae_ice"]["gate1"], s["fsae_ice"]["harmful"]) == \
        (29, 35, 35, 48)
    assert gr["false_alarms_gate1"] == [] and gr["false_alarms_gate2"] == [] and s["no_change"] == 17
    assert s["all"]["crashed"] == 1
