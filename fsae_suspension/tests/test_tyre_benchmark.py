"""Magic Formula lateral force against MFeval 4.3.1 (scripts/tyre_benchmark.py)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "tyre" / "results.json").read_text())["passed"]


def test_live_run_matches_mfeval():
    import tyre_benchmark as tb
    r = tb.run()
    assert r["passed"]
    mf52 = r["MagicFormula52_Parameters"]
    assert mf52["scope"] == "all" and all(g["worst_dFy_N"] < 1e-3 for g in mf52["groups"])


def test_every_mfeval_file_is_graded_now_that_mf61_is_implemented():
    import tyre_benchmark as tb
    r = tb.run()
    assert all(v["scope"] == "all" for k, v in r.items() if k != "passed")


def test_load_tir_picks_the_version_and_the_default_is_still_mf52():
    from suspension.tiremodel import load_tir, default_tire
    ref = ROOT / "benchmarks" / "tyre" / "mfeval_4.3.1"
    assert load_tir(str(ref / "mfeval_ref_MagicFormula52_Parameters.tir")).mf_version == "5.2"
    assert load_tir(str(ref / "mfeval_ref_FSAE_Defaults.tir")).mf_version == "6.1"
    assert default_tire().mf_version == "5.2"
