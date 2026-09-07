# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""Tests for suspension/rationale.py — the "why did you run this" capture.

The properties that matter are behavioural, not functional: this feature only
works if it is cheap enough to actually use, and only helps if it never invents
a reason nobody gave.
"""
import ast
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from suspension import rationale as R                     # noqa: E402


# --- recording what changed, so we never have to ask ----------------------
def test_change_is_recorded():
    ss = {}
    R.record_change(ss, "kinematics", "Ride height (mm)", 10.0, 12.0)
    assert R.changes_for(ss, "kinematics")[0] == {
        "label": "Ride height (mm)", "from": "10", "to": "12"}


def test_dragging_one_slider_reads_as_a_single_move():
    """Ten intermediate positions are not ten changes; the member moved one
    input from where it started to where it ended."""
    ss = {}
    for a, b in ((10.0, 11.0), (11.0, 12.0), (12.0, 14.0)):
        R.record_change(ss, "kinematics", "Ride height (mm)", a, b)
    rows = R.changes_for(ss, "kinematics")
    assert len(rows) == 1
    assert rows[0]["from"] == "10" and rows[0]["to"] == "14"


def test_a_non_change_is_not_recorded():
    ss = {}
    R.record_change(ss, "kinematics", "Ride height", 10.0, 10.0)
    assert R.changes_for(ss, "kinematics") == []


def test_changes_are_bounded():
    ss = {}
    for i in range(40):
        R.record_change(ss, "k", f"input {i}", i, i + 1)
    assert len(R.changes_for(ss, "k")) == R.MAX_CHANGES


def test_prefill_names_the_inputs_and_their_values():
    """This is the whole trick: the tool says what changed so the member only
    has to supply the why."""
    ss = {}
    R.record_change(ss, "k", "Ride height (mm)", 10.0, 12.0)
    R.record_change(ss, "k", "Static camber", -1.5, -2.0)
    out = R.describe_changes(ss, "k")
    assert "Ride height (mm) (10 → 12)" in out
    assert "Static camber (-1.5 → -2)" in out


def test_prefill_summarises_when_there_are_many():
    ss = {}
    for i in range(9):
        R.record_change(ss, "k", f"in{i}", 0, 1)
    assert "and 5 more" in R.describe_changes(ss, "k")


def test_prefill_is_empty_when_nothing_moved():
    assert R.describe_changes({}, "k") == ""


# --- the entry itself ------------------------------------------------------
def test_one_click_entry_is_valid():
    """Minimum viable note is a dropdown selection. If that isn't enough, the
    feature is a form, and forms don't get filled."""
    ss = {}
    e = R.add_entry(ss, "k", intent="check it against a rules limit")
    assert e is not None
    assert R.sentence(e, "Kinematics") == \
        "Ran Kinematics to check it against a rules limit."


def test_empty_entry_is_refused():
    """A row that says nothing still counts as an answer wherever coverage is
    shown, which quietly turns the completeness figure into a lie."""
    ss = {}
    assert R.add_entry(ss, "k", intent="", detail="") is None
    assert R.add_entry(ss, "k", intent="   ", detail="  ") is None
    assert R.entries_for(ss, "k") == []


def test_whitespace_only_changed_field_does_not_qualify_alone():
    ss = {}
    assert R.add_entry(ss, "k", intent="", detail="",
                       changed="Ride height") is None


def test_full_sentence_reads_as_prose():
    e = R.add_entry({}, "k", intent="chase something the driver reported",
                    detail="vague turn-in", changed="Ride height (10 → 12)",
                    why_changed="we wanted camber recovery back",
                    outcome="we're changing the design")
    s = R.sentence(e, "Kinematics")
    assert s.startswith("Ran Kinematics to chase something the driver reported")
    assert "because we wanted camber recovery back" in s
    assert s.endswith("Outcome: we're changing the design.")


def test_sentence_omits_clauses_that_were_left_blank():
    """A short true sentence beats a long one with holes in it."""
    e = R.add_entry({}, "k", intent="compare two design options")
    s = R.sentence(e, "Brakes")
    assert "Changed" not in s and "Outcome" not in s


def test_entries_are_bounded_and_newest_wins():
    ss = {}
    for i in range(30):
        R.add_entry(ss, "k", intent="just exploring — no decision yet",
                    detail=f"run {i}")
    rows = R.entries_for(ss, "k")
    assert len(rows) == R.MAX_ENTRIES
    assert rows[-1]["detail"] == "run 29"


# --- honesty about absence -------------------------------------------------
def test_no_entries_yields_no_lines_rather_than_a_placeholder():
    """The caller writes the 'not recorded' line, in its own voice. If this
    module invented one it would read like a real entry."""
    assert R.report_lines({}, "k", "Kinematics") == []


def test_report_lines_attribute_unsigned_notes_honestly():
    ss = {}
    R.add_entry(ss, "k", intent="compare two design options")
    assert "unattributed" in R.report_lines(ss, "k", "Kinematics")[0]


def test_coverage_counts_only_features_with_a_real_note():
    ss = {}
    R.add_entry(ss, "kinematics", intent="compare two design options")
    R.add_entry(ss, "brakes", intent="", detail="")        # refused
    assert R.coverage(ss, ["kinematics", "brakes", "aero"]) == (1, 3)


def test_intent_and_outcome_menus_include_an_honest_escape():
    """If the true answer isn't on the menu people pick a wrong one, and a
    wrong reason is worse than an honest shrug."""
    assert any("exploring" in i for i in R.INTENTS)
    assert any("no decision" in o for o in R.OUTCOMES)


# --- robustness ------------------------------------------------------------
def test_nothing_raises_on_junk():
    for bad in (None, "", 0):
        R.record_change({}, bad, "x", 1, 2)
        assert R.entries_for({}, bad) == []
        assert R.describe_changes({}, bad) == ""
        assert R.sentence(None) == ""


# --- wiring ----------------------------------------------------------------
def _src(name):
    return open(os.path.join(ROOT, name), encoding="utf-8").read()


def test_prompt_is_attached_to_the_shared_feature_panel():
    """One hook, all 36 documentable features — not 36 per-tab edits."""
    src = _src("streamlit_app.py")
    assert "_render_rationale_prompt(_feat, _lbl, _kp)" in src
    i = src.index("def render_feature_documentation(")
    assert "_render_rationale_prompt" in src[i:i + 3000]


def test_report_states_when_no_rationale_was_recorded():
    src = _src("streamlit_app.py")
    assert "No rationale recorded" in src
    assert "why this was run" in src


def test_changed_clause_is_prefilled_from_real_edits():
    src = _src("streamlit_app.py")
    i = src.index("def _render_rationale_prompt(")
    block = src[i:i + 4000]
    assert "_rat.describe_changes(" in block
    assert "value=_auto" in block, "the 'I changed' field is not prefilled"
