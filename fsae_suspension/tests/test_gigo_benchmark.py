"""The garbage-in benchmark (docs/GIGO_BENCHMARK_PLAN.md): its sealed catalog, its corruption arithmetic and, once
run, its stored result."""
import json
import random
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import gigo_benchmark as gb  # noqa: E402


@pytest.mark.parametrize("seal, predictions, extra", [(gb.SEAL1, gb.ROUND1_PREDICTIONS, {"suspension/plausibility.py"}),
                                                     (gb.SEAL, gb.PREDICTIONS, set())])
def test_each_seal_holds_the_catalog_the_runner_uses(seal, predictions, extra):
    """Both rounds share the catalog; round 2 adds the plausibility module to the sealed code."""
    rec = json.loads(seal.read_text())
    assert rec["corruptions"] == json.loads(json.dumps(gb.systematic() + gb.curated(), ensure_ascii=False))
    assert rec["predictions"] == predictions and rec["random"]["n"] == gb.N_RANDOM
    assert rec["gates"]["gate2_rel"] == gb.GATE2_REL and rec["gates"]["harm_rel_fastsim"] == gb.HARM_REL
    assert set(rec["code_sha256"]) | extra == set(gb.SEALED_CODE) and rec["created_utc"].endswith("Z")


def test_every_field_meets_every_confusion_of_its_dimension():
    rows = gb.systematic()
    per = {(r["deck"], r["field"]) for r in rows}
    for deck, field in per:
        dim = (gb.FSAE_FIELDS[field][0] if deck == "fsae" else gb.FS_FIELDS[field][0])
        assert {r["key"] for r in rows if (r["deck"], r["field"]) == (deck, field)} == \
            {k for k, _, _ in gb.CONFUSIONS[dim]}
    assert len(rows) == 165 and len(gb.curated()) == 56


@pytest.mark.parametrize("how, x, want", [(2.20462, 100.0, 220.462), ("square", 0.9, 0.81), ("inverse", 4.0, 0.25),
                                          ("minus_75", 300.0, 225.0)])
def test_corruptions_change_the_value_as_stated(how, x, want):
    assert gb.apply_value(x, how) == pytest.approx(want)


def test_typing_errors_are_the_ones_named():
    rng = random.Random(1)
    assert gb._typo(123.4, "decimal_right", rng) == pytest.approx(1234.0)
    assert gb._typo(123.4, "decimal_left", rng) == pytest.approx(12.34)
    swapped = gb._typo(1234.0, "swap", random.Random(0))
    assert sorted(str(int(swapped))) == sorted("1234") and swapped != 1234.0
    dropped = gb._typo(1234.0, "drop", random.Random(0))
    assert dropped != 1234.0 and len(str(int(dropped))) == 4


def test_the_random_set_is_fixed_by_the_seed_and_varies_with_it():
    a, b = gb.random_set("aaaa"), gb.random_set("bbbb")
    assert a == gb.random_set("aaaa") and a != b and len(a) == gb.N_RANDOM


def test_a_curated_error_hits_the_right_input_of_the_formula_student_deck():
    inp = gb._fsae_deck()
    c = next(x for x in gb.curated() if x["deck"] == "fsae" and x["key"] == "cell_for_pack_v")
    out = gb.corrupt_fsae(inp, c)
    assert inp["ev"]["pack"]["series"] == 1 and out["ev"]["pack"]["cell_v"] == 3.6   # the pack is one lumped cell
    assert json.dumps(inp) != json.dumps(out) and json.dumps(gb._fsae_deck()) == json.dumps(inp)   # NaN-safe


def test_the_stored_round_one_run_grades_as_reported():
    """The run of 6 Nov 2026 (seed: seal commit 4c18a8a): Gate 1 0 of 120 harmful, Gate 2 99 of 99, no false alarm.
    Re-graded only while the code is still the sealed code."""
    import hashlib
    digest, name = (gb.BENCH1 / "MANIFEST.sha256").read_text().split()
    assert hashlib.sha256((gb.BENCH1 / name).read_bytes()).hexdigest() == digest
    runs = json.loads(gb.RUNS1.read_text())
    assert runs["seed_commit"] == "4c18a8ae7579eee7a5eceaf85f99685983c5c46c"
    rec = json.loads(gb.SEAL1.read_text())
    if rec["code_sha256"] != gb.code_fingerprint():
        pytest.skip("the code has moved on from the sealed commit; the stored GRADED.txt is the record")
    gr = gb.grade(runs)
    a = gb.summarize(gr)["all"]
    assert (a["corruptions"], a["harmful"], a["no_change"], a["tool_refused"]) == (265, 120, 16, 31)
    assert (a["gate1_harmful"], a["gate2_harmful"], a["gate2_applicable"], a["either_harmful"]) == (0, 99, 99, 99)
    assert gr["false_alarms_gate1"] == [] and gr["false_alarms_gate2"] == []
    assert len(a["silent_harmful"]) == 21 and all(s.startswith("fsae ") for s in a["silent_harmful"])


def test_the_stored_round_two_run_grades_as_reported():
    """Round 2 (seed: seal commit 383bda9, with suspension/plausibility.py at entry): Gate 1 65 of 119 harmful, 18 of
    20 on the Formula Student deck; Gate 2 99 of 99; no false alarm. Re-graded only while the code is the sealed code."""
    import hashlib
    digest, name = (gb.BENCH / "MANIFEST.sha256").read_text().split()
    assert hashlib.sha256((gb.BENCH / name).read_bytes()).hexdigest() == digest
    runs = json.loads(gb.RUNS.read_text())
    assert runs["seed_commit"] == "383bda9126b0885acfe3e6ad0b8e62b6202740a9"
    if json.loads(gb.SEAL.read_text())["code_sha256"] != gb.code_fingerprint():
        pytest.skip("the code has moved on from the sealed commit; the stored GRADED.txt is the record")
    gr = gb.grade(runs)
    a = gb.summarize(gr)["all"]
    assert (a["corruptions"], a["harmful"], a["no_change"], a["tool_refused"]) == (269, 119, 12, 35)
    assert (a["gate1_harmful"], a["gate2_harmful"], a["gate2_applicable"], a["either_harmful"]) == (65, 99, 99, 117)
    fs = [r for r in gr["rows"] if not r.get("status") and r["harmful"] and r["deck"] == "fsae"]
    assert (sum(r["gate1"] for r in fs), len(fs)) == (18, 20)
    assert gr["false_alarms_gate1"] == [] and gr["false_alarms_gate2"] == []
    assert sorted(a["silent_harmful"]) == ["fsae inverter_eff typo_15_drop (23%)", "fsae mass_kg driver_dropped (16%)"]
