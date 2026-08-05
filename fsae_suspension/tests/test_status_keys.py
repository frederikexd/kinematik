# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""Tests for rule/spec key resolution on the status dashboard.

The regression: a rule's ``param`` and a component's spec keys are typed into
two different free-text boxes. Matched with a bare case-sensitive
``specs.get()``, "Mass" and "mass" never met — the check reported "not declared
yet" forever while the number sat in the same row, and the board stayed amber
in a way that reads at a design review as an unfinished car.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from suspension import status_dashboard as sd            # noqa: E402


SPECS = {"Weight": "2.4 kg", "Offset": "42.0 mm", "Wall thickness": "TBD"}


def _rule(param, op="<=", value=2.5):
    return {"param": param, "op": op, "value": value, "unit": "kg",
            "label": param}


# --- resolution ------------------------------------------------------------
def test_exact_key_resolves():
    val, key, sug = sd.resolve_spec("Weight", SPECS)
    assert val == "2.4 kg" and key == "Weight" and sug is None


def test_case_insensitive():
    assert sd.resolve_spec("weight", SPECS)[1] == "Weight"
    assert sd.resolve_spec("WEIGHT", SPECS)[1] == "Weight"


def test_whitespace_and_separators_normalized():
    assert sd.resolve_spec("  Weight  ", SPECS)[1] == "Weight"
    assert sd.resolve_spec("wall_thickness", SPECS)[1] == "Wall thickness"
    assert sd.resolve_spec("wall-thickness", SPECS)[1] == "Wall thickness"


def test_typo_yields_a_suggestion_not_a_match():
    val, key, sug = sd.resolve_spec("Wieght", SPECS)
    assert val is None and key is None
    assert sug == "Weight"


def test_absent_key_suggests_nothing():
    assert sd.resolve_spec("Torque", SPECS) == (None, None, None)


def test_substring_matches_are_refused():
    """Guessing would check the WRONG number — worse than the amber it replaces.

    A close key may still be offered as a *suggestion*; what must never happen
    is it resolving to a value, which would silently validate the wrong number.
    """
    val, key, _sug = sd.resolve_spec("Wall thickness",
                                     {"Min wall thickness": "3 mm"})
    assert val is None and key is None
    val, key, _sug = sd.resolve_spec("Offset", {"Offset tolerance": "0.1 mm"})
    assert val is None and key is None


def test_empty_and_missing_inputs_are_safe():
    assert sd.resolve_spec("", SPECS) == (None, None, None)
    assert sd.resolve_spec("Weight", {}) == (None, None, None)
    assert sd.resolve_spec("Weight", None) == (None, None, None)


# --- rule evaluation -------------------------------------------------------
def test_case_mismatched_rule_now_passes():
    assert sd.evaluate_rule(_rule("weight"), SPECS).status == sd.GREEN


def test_failing_rule_still_fails():
    assert sd.evaluate_rule(_rule("weight", "<=", 1.0), SPECS).status == sd.RED


def test_three_amber_reasons_are_distinguishable():
    """A dead-end amber and a one-click-fixable amber must not read alike."""
    typo = sd.evaluate_rule(_rule("Wieght"), SPECS)
    absent = sd.evaluate_rule(_rule("Torque"), SPECS)
    novalue = sd.evaluate_rule(_rule("Wall thickness"), SPECS)
    assert all(r.status == sd.AMBER for r in (typo, absent, novalue))
    assert "did you mean" in typo.message
    assert "Weight" in typo.message
    assert "not declared yet" in absent.message
    assert "no number to check" in novalue.message


# --- component rollup ------------------------------------------------------
def _row(specs, **kw):
    base = {"name": "Diff mount", "subteam": "suspension", "status": "verified",
            "specs": specs, "has_file": True}
    base.update(kw)
    return base


def test_component_turns_green_despite_key_case_mismatch():
    st_ = sd.status_for_component(_row({"Mass": "2.1 kg"}),
                                  [_rule("mass")])
    assert st_.status == sd.GREEN


def test_unverified_still_amber():
    st_ = sd.status_for_component(_row({"Mass": "2.1 kg"}, status="unverified"),
                                  [_rule("mass")])
    assert st_.status == sd.AMBER


def test_missing_file_still_red():
    st_ = sd.status_for_component(_row({"Mass": "2.1 kg"}, has_file=False),
                                  [_rule("mass")])
    assert st_.status == sd.RED


def test_weight_and_mass_are_not_silently_conflated():
    """The quick-add templates hard-code "Weight". A team that declared "mass"
    still gets amber — correctly. Weight and mass are different quantities, and
    a validator that quietly treats them as one is worse than one that asks.

    The fix for this case is the UI's: the rule Parameter is now a picker of
    the keys actually declared, so the mismatch can't be created by hand. What
    this test pins is that the MODEL never guesses its way out of it.
    """
    tpl = sd.template_for("Weight")
    st_ = sd.status_for_component(_row({"mass": "2.2 kg"}), [tpl])
    assert st_.status == sd.AMBER
    assert "not declared" in st_.rule_results[0].message
