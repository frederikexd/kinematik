# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Direct-acting dampers against the benchmark tools (benchmarks/direct_acting): the independent checks pass inside the
registered limits, every deck and sheet is the one the generator writes, and a tool without results is NOT RUN."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import direct_acting_benchmark as dab                                     # noqa: E402


def test_limits_are_the_registered_ones():
    got = {t: (v["damper_mm"], v["abs_N"], v["rel"]) for t, v in dab.LIMITS.items()}
    assert got == {"independent": (0.001, 0.5, 0.001), "adams_view": (0.01, 0.5, 0.001),
                   "motionsolve": (0.01, 0.5, 0.001), "ard": (0.05, 2.0, 0.005)}


@pytest.mark.parametrize("name", dab.CORNERS)
def test_corner_files_are_generated(name):
    assert dab.corner(name) == json.loads(json.dumps(dab.build_corner(name)))
    assert dab.corner(name)["actuation"] == "direct"


@pytest.mark.parametrize("name", dab.CORNERS)
def test_decks_are_generated(name):
    B = dab.BENCH
    assert (B / "adams" / f"{name}_mr.cmd").read_text() == dab.mr_command_file(name)
    assert (B / "adams" / f"{name}_loads.cmd").read_text() == dab.loads_command_file(name)
    assert (B / "motionsolve" / f"{name}_mr.adm").read_text() == dab.mr_adm(name)
    assert (B / "motionsolve" / f"{name}_loads.adm").read_text() == dab.loads_adm(name)
    assert (B / "inputs.csv").read_text() == dab.inputs_sheet()


@pytest.mark.parametrize("name", dab.CORNERS)
def test_independent_checks_pass(name):
    r = dab.compare("independent", name)
    assert r["damper"] == "PASS"
    assert r["loads"] == ("PASS" if name == "direct_upright" else "NOT RUN")


def test_the_damper_compresses_in_bump():
    for n in dab.CORNERS:
        d = dab.kinematik_damper(n)
        assert d[-1] < 0 < d[0]


@pytest.mark.parametrize("tool", ["adams_view", "motionsolve", "ard"])
def test_a_tool_without_results_is_not_run(tool, tmp_path, monkeypatch):
    for sub in ("corners",):
        (tmp_path / sub).mkdir()
        for n in dab.CORNERS:
            (tmp_path / sub / f"{n}.json").write_text((dab.BENCH / sub / f"{n}.json").read_text())
    (tmp_path / "ard_damper.csv").write_text(dab.damper_template("ard"))
    monkeypatch.setattr(dab, "BENCH", tmp_path)
    r = dab.compare(tool, "direct_lower")
    assert r["damper"] == "NOT RUN" and r["loads"] == "NOT RUN"


def test_a_wrong_hand_entered_damper_length_fails(tmp_path, monkeypatch):
    (tmp_path / "corners").mkdir()
    for n in dab.CORNERS:
        (tmp_path / "corners" / f"{n}.json").write_text((dab.BENCH / "corners" / f"{n}.json").read_text())
    import numpy as np
    k = dab.kinematik_damper("direct_lower")
    rows = ["corner,travel_mm,damper_mm"]
    for t in dab.HAND_TRAVEL:
        v = 300.0 + float(np.interp(t, dab.TRAVEL, k)) + (0.2 if t == 10.0 else 0.0)
        rows.append(f"direct_lower,{t:g},{v:.4f}")
    (tmp_path / "ard_damper.csv").write_text("\n".join(rows) + "\n")
    monkeypatch.setattr(dab, "BENCH", tmp_path)
    assert dab.compare("ard", "direct_lower")["damper"] == "FAIL"
