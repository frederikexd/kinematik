# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_brief_coverage.py — the briefing covers EVERY feature, pinned.
# ============================================================================
"""
What these tests guard.

The mission briefing tailors on three axes — subsystem (role), goal, proficiency
— and tailoring is exactly where coverage silently leaks. Three leaks existed and
these tests make each of them a build failure rather than a thing a member
notices six months later:

  1. UNREACHABLE TOOLS. A tab with complete briefing copy that no goal, purpose
     or note keyword recommends is written, paid for and never shown. `daq`,
     `frames` and `phantom_env` were all in this state — the Data Acquisition
     subteam could not reach the Data Acquisition tool.
  2. TAILORING THAT SHRANK COVERAGE. The old resolver used goal-specific copy
     XOR the canonical feature list. So writing tailored copy for a (goal, tool)
     pair HID every capability that copy did not mention, for that goal.
  3. ADVANCED PROFICIENCY GETTING NOTHING. No tailored copy + advanced returned
     an empty list: the tool appeared in the plan with no features at all.

The invariant that replaces all three: for every (role, goal, tool, proficiency)
the questionnaire can produce, the member sees a non-empty feature list whose
union spans the tool's complete canonical capability set. Proficiency changes the
number of WORDS, never the number of CAPABILITIES.

Streamlit is never imported — the tables are lifted out of the app's AST, the
same trick tests/test_mission_briefing.py uses.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension.brief_coverage import (  # noqa: E402
    BriefingTables, CoverageGap, GAP_KINDS, PROFICIENCIES,
    audit, distinctive_tokens, effective_role_goals, extract_tables,
    load_tables_from_app, reachable_tools, resolve_feature_lines,
)

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_APPS = [os.path.join(_ROOT, "streamlit_app.py"),
         os.path.join(_ROOT, "suspension", "streamlit_app.py")]


@pytest.fixture(scope="module")
def tables():
    return load_tables_from_app(_APPS[0])


# --------------------------------------------------------------------------- #
#  The headline contract
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("app_path", _APPS, ids=["root", "package"])
def test_briefing_coverage_has_no_gaps(app_path):
    """
    The whole point. Every tool reachable, briefed, glossed, tailored for every
    goal that recommends it, and summonable from the note box.

    Both copies of the app are checked: they drift, and a fix applied to one is
    a fix the other's users never get.
    """
    report = audit(load_tables_from_app(app_path))
    assert report.ok, "\n" + report.summary()


def test_every_goal_tool_proficiency_combination_yields_features(tables):
    """
    The combinatorial sweep — every (role, goal, tool, proficiency) the
    questionnaire can produce must yield at least one feature line.

    This is the test that would have caught advanced-with-no-tailored-copy
    returning nothing.
    """
    empty = []
    for role, goals in sorted(effective_role_goals(tables).items()):
        for goal_key, _label, tool_ids in goals:
            for tool_id in tool_ids:
                for prof in PROFICIENCIES:
                    plan = resolve_feature_lines(tool_id, [goal_key], tables, prof)
                    if not plan.all_lines:
                        empty.append(f"{role}/{goal_key}/{tool_id}/{prof}")
    assert not empty, "combinations with no feature lines:\n  " + "\n  ".join(empty)


def test_every_combination_spans_the_full_capability_set(tables):
    """
    Coverage, not just non-emptiness: every canonical feature of the tool must be
    shown, or demonstrably echoed by the tailored copy.
    """
    incomplete = []
    for role, goals in sorted(effective_role_goals(tables).items()):
        for goal_key, _label, tool_ids in goals:
            for tool_id in tool_ids:
                for prof in PROFICIENCIES:
                    plan = resolve_feature_lines(tool_id, [goal_key], tables, prof)
                    if not plan.complete:
                        missing = plan.n_canonical - (plan.n_canonical_shown
                                                      + plan.n_canonical_echoed)
                        incomplete.append(
                            f"{role}/{goal_key}/{tool_id}/{prof}: "
                            f"{missing} canonical feature(s) unaccounted for")
    assert not incomplete, "\n  " + "\n  ".join(incomplete)


def test_proficiency_changes_words_not_capabilities(tables):
    """
    The distinction the whole redesign rests on: an advanced member gets a
    terser briefing, not a smaller one.
    """
    checked = 0
    for role, goals in effective_role_goals(tables).items():
        for goal_key, _label, tool_ids in goals:
            for tool_id in tool_ids:
                plans = {p: resolve_feature_lines(tool_id, [goal_key], tables, p)
                         for p in PROFICIENCIES}
                counts = {p: len(pl.all_lines) for p, pl in plans.items()}
                assert len(set(counts.values())) == 1, (
                    f"{role}/{goal_key}/{tool_id}: proficiency changed the "
                    f"NUMBER of capabilities: {counts}")
                adv, beg = plans["advanced"], plans["beginner"]
                if len(adv.additional) > 1 and adv.primary:
                    # Same capabilities, more compact presentation: advanced
                    # condenses the block to a single line, beginner keeps it
                    # as one bullet per capability. (Raw character count is not
                    # the test — "Also: " plus semicolons can exceed the bullet
                    # markers it replaces; the shape is what differs.)
                    assert "\n" not in adv.additional_as_text()
                    assert "\n" in beg.additional_as_text()
                checked += 1
    assert checked > 50, "sweep did not cover a meaningful number of combinations"


# --------------------------------------------------------------------------- #
#  Reachability
# --------------------------------------------------------------------------- #
def test_every_tool_is_reachable_from_the_questionnaire(tables):
    reach = reachable_tools(tables)
    orphans = sorted(tables.tool_ids - set(reach))
    assert not orphans, f"tools no goal or purpose recommends: {orphans}"


@pytest.mark.parametrize("tool_id,role", [
    ("daq", "dataacq"),          # the subteam that owns it must reach it
    ("phantom_env", "chassis"),
    ("frames", "suspension"),    # cross-cutting: appended to every subteam
])
def test_previously_orphaned_tools_reach_their_owning_subteam(tables, tool_id, role):
    goals = effective_role_goals(tables)[role]
    reached = {t for _gk, _lab, ids in goals for t in ids}
    assert tool_id in reached, (
        f"{tool_id} is still not reachable from the {role} subteam")


def test_every_tool_has_a_note_box_route(tables):
    """A member who types the exact problem a tool solves must be handed it."""
    routed = {tid for kws, tid, _adv in tables.freetext if tid != "any"}
    assert not tables.tool_ids - routed, (
        f"tools with no freetext keyword: {sorted(tables.tool_ids - routed)}")


def test_note_keywords_are_lowercase_and_nonempty(tables):
    """The matcher lower-cases the note, so an upper-case keyword never fires."""
    bad = []
    for keywords, tid, advice in tables.freetext:
        for kw in keywords:
            if not kw.strip() or kw != kw.lower():
                bad.append(f"{tid}: {kw!r}")
        if not advice.strip():
            bad.append(f"{tid}: empty advice")
    assert not bad, bad


def test_note_keywords_are_ascii(tables):
    """
    Regression: a Cyrillic homoglyph slipped into a keyword during the coverage
    fix ("rубs" for "rubs"). It parses, it lints, and it can never match.
    """
    bad = [f"{tid}: {kw!r}" for kws, tid, _a in tables.freetext for kw in kws
           if not kw.isascii()]
    assert not bad, f"non-ASCII characters in note keywords: {bad}"


# --------------------------------------------------------------------------- #
#  Copy quality
# --------------------------------------------------------------------------- #
def test_every_tool_has_copy_gloss_and_features(tables):
    missing = []
    for tool_id in sorted(tables.tool_ids):
        if tool_id not in tables.tools:
            missing.append(f"{tool_id}: no _BRIEF_TOOLS entry")
        if tool_id not in tables.simple:
            missing.append(f"{tool_id}: no _BRIEF_SIMPLE gloss")
        if not tables.tool_features.get(tool_id):
            missing.append(f"{tool_id}: no _BRIEF_TOOL_FEATURES list")
    assert not missing, missing


def test_goal_feature_entries_reference_real_goals_and_tools(tables):
    """Copy written for a goal or tool that no longer exists is dead weight."""
    goal_keys = {gk for goals in effective_role_goals(tables).values()
                 for gk, _l, _i in goals}
    stale = [f"({gk}, {tid})" for (gk, tid) in tables.goal_features
             if gk not in goal_keys or tid not in tables.tool_ids]
    assert not stale, f"goal-feature entries for unknown goal/tool: {stale}"


def test_feature_lines_are_complete_sentences(tables):
    """Bullets are read aloud by the audio briefing; fragments sound wrong."""
    bad = []
    for (gk, tid), lines in tables.goal_features.items():
        for line in lines:
            if not line.strip():
                bad.append(f"({gk}, {tid}): empty line")
            elif not line.rstrip().endswith((".", "!", "?")):
                bad.append(f"({gk}, {tid}): no terminal punctuation: {line[:60]}")
            elif not line[0].isupper() and not line[0].isdigit():
                bad.append(f"({gk}, {tid}): not capitalised: {line[:60]}")
    assert not bad, bad[:20]


def test_no_duplicate_feature_lines_within_a_pair(tables):
    dupes = []
    for key, lines in tables.goal_features.items():
        if len(set(lines)) != len(lines):
            dupes.append(key)
    assert not dupes, f"(goal, tool) pairs with duplicate bullets: {dupes}"


# --------------------------------------------------------------------------- #
#  The resolver's own behaviour
# --------------------------------------------------------------------------- #
def _fixture_tables():
    """A tiny hand-built table set — the resolver's rules, in isolation."""
    return BriefingTables(
        tab_meta={"toolA": ("🅰️", "Tool A"), "toolB": ("🅱️", "Tool B")},
        tools={"toolA": ("A does things.", "why", "vs"),
               "toolB": ("B does things.", "why", "vs")},
        simple={"toolA": "plain A", "toolB": "plain B"},
        tool_features={
            "toolA": ["Alpha capability with beta gamma delta wording.",
                      "Epsilon capability about zeta eta theta.",
                      "Iota capability concerning kappa lambda mu."],
            "toolB": ["Only capability of tool B, nu xi omicron."],
        },
        goal_features={
            ("g1", "toolA"): ["Alpha capability with beta gamma delta wording."],
        },
        role_goals={"r1": [("g1", "Goal one", ["toolA", "toolB"])]},
        verify_goals=[], purposes=[], freetext=[],
    )


def test_resolver_keeps_capabilities_the_tailored_copy_did_not_mention():
    """The core fix: tailoring adds relevance, it does not subtract coverage."""
    t = _fixture_tables()
    plan = resolve_feature_lines("toolA", ["g1"], t, "intermediate")
    assert plan.tailored
    assert len(plan.primary) == 1
    # The two capabilities the tailored line never mentioned are still shown.
    assert len(plan.additional) == 2
    assert "Epsilon" in plan.additional[0]
    assert plan.complete


def test_resolver_does_not_repeat_a_capability_the_tailored_copy_already_states():
    t = _fixture_tables()
    plan = resolve_feature_lines("toolA", ["g1"], t, "intermediate")
    # The tailored line IS the first canonical line, so it must not appear twice.
    assert plan.all_lines.count(
        "Alpha capability with beta gamma delta wording.") == 1
    assert plan.n_canonical_echoed == 1


def test_resolver_never_returns_empty_at_any_proficiency():
    t = _fixture_tables()
    for prof in PROFICIENCIES:
        # toolB has no tailored copy for g1 at all — the old rule gave advanced
        # members nothing here.
        plan = resolve_feature_lines("toolB", ["g1"], t, prof)
        assert plan.all_lines, f"{prof} got no feature lines"
        assert not plan.tailored
        assert plan.complete


def test_resolver_falls_back_to_the_blurb_for_a_tool_with_no_features():
    t = _fixture_tables()
    t.tool_features["toolB"] = []
    plan = resolve_feature_lines("toolB", ["g1"], t, "advanced")
    assert plan.all_lines == ["B does things."]


def test_resolver_dedupes_across_multiple_active_goals():
    t = _fixture_tables()
    t.goal_features[("g2", "toolA")] = [
        "Alpha capability with beta gamma delta wording.",
        "A second tailored line, unique to g2.",
    ]
    plan = resolve_feature_lines("toolA", ["g1", "g2"], t, "intermediate")
    assert len(plan.primary) == 2
    assert len(set(plan.primary)) == 2


def test_advanced_condenses_additional_into_one_line():
    t = _fixture_tables()
    adv = resolve_feature_lines("toolA", ["g1"], t, "advanced")
    inter = resolve_feature_lines("toolA", ["g1"], t, "intermediate")
    assert "\n" not in adv.additional_as_text()
    assert inter.additional_as_text().count("\n") == 1     # two bullets
    assert len(adv.all_lines) == len(inter.all_lines)      # same capabilities


def test_unknown_proficiency_degrades_to_intermediate():
    t = _fixture_tables()
    plan = resolve_feature_lines("toolA", ["g1"], t, "wizard")
    assert plan.proficiency == "intermediate"


def test_resolver_handles_no_goals_at_all():
    """Integration/Validation close every briefing with no goal naming them."""
    t = _fixture_tables()
    plan = resolve_feature_lines("toolA", [], t, "advanced")
    assert len(plan.all_lines) == 3
    assert not plan.tailored


def test_echo_detection_is_conservative():
    """
    When in doubt the line is SHOWN. A short or only-partly-overlapping canonical
    line must survive, because hiding a capability is the expensive mistake.
    """
    t = _fixture_tables()
    t.goal_features[("g1", "toolA")] = ["Alpha beta."]        # few tokens
    plan = resolve_feature_lines("toolA", ["g1"], t, "intermediate")
    assert len(plan.additional) == 3, "partial overlap must not hide a capability"


def test_distinctive_tokens_drops_glue_words():
    tokens = distinctive_tokens("The camber gain curve updates with the geometry")
    assert "camber" in tokens and "geometry" in tokens
    assert "the" not in tokens and "with" not in tokens


# --------------------------------------------------------------------------- #
#  The audit itself
# --------------------------------------------------------------------------- #
def test_audit_detects_an_orphaned_tool():
    t = _fixture_tables()
    t.tab_meta["toolC"] = ("©️", "Tool C")
    t.tools["toolC"] = ("C.", "why", "vs")
    t.simple["toolC"] = "plain C"
    t.tool_features["toolC"] = ["Something."]
    report = audit(t, require_freetext=False)
    assert not report.ok
    assert [g.tool for g in report.of_kind("UNREACHABLE")] == ["toolC"]


def test_audit_detects_an_untailored_goal_pair():
    t = _fixture_tables()
    report = audit(t, require_freetext=False, require_plain_english=False)
    pairs = report.of_kind("GOAL_PAIR_UNTAILORED")
    assert [g.tool for g in pairs] == ["toolB"]      # g1 -> toolB has no copy
    assert pairs[0].goal == "g1" and pairs[0].role == "r1"


def test_audit_detects_missing_copy():
    t = _fixture_tables()
    del t.tools["toolB"]
    t.tool_features["toolB"] = []
    del t.simple["toolB"]
    report = audit(t, require_freetext=False)
    kinds = {g.kind for g in report.gaps if g.tool == "toolB"}
    assert {"NO_BRIEFING_COPY", "NO_FEATURE_LIST", "NO_PLAIN_ENGLISH"} <= kinds


def test_audit_gap_kinds_are_all_declared():
    t = _fixture_tables()
    t.tab_meta["toolC"] = ("©️", "Tool C")
    report = audit(t)
    assert all(g.kind in GAP_KINDS for g in report.gaps)
    assert report.tally()


def test_clean_tables_audit_clean():
    t = _fixture_tables()
    t.goal_features[("g1", "toolB")] = ["Tailored line for B."]
    t.freetext = [(["a"], "toolA", "advice"), (["b"], "toolB", "advice")]
    report = audit(t)
    assert report.ok, report.summary()
    assert "COVERAGE COMPLETE" in report.summary()


def test_effective_role_goals_appends_the_verification_goals():
    t = _fixture_tables()
    t.verify_goals = [("vg_x", "Cross-cutting", ["toolB"])]
    goals = effective_role_goals(t)["r1"]
    assert [g[0] for g in goals] == ["g1", "vg_x"]
    # ...and does not duplicate one the subteam already defines.
    t.verify_goals = [("g1", "dupe", ["toolB"])]
    assert [g[0] for g in effective_role_goals(t)["r1"]] == ["g1"]


# --------------------------------------------------------------------------- #
#  Table extraction
# --------------------------------------------------------------------------- #
def test_extract_tables_reads_the_real_app(tables):
    assert len(tables.tab_meta) >= 40
    assert len(tables.goal_features) > 60
    assert tables.role_goals and tables.verify_goals and tables.purposes


def test_extract_tables_survives_a_module_without_them():
    t = extract_tables("x = 1\n")
    assert t.tab_meta == {} and t.tool_ids == set()


@pytest.mark.parametrize("app_path", _APPS, ids=["root", "package"])
def test_both_app_copies_carry_identical_briefing_tables(app_path, tables):
    """
    The two streamlit_app.py copies drift. A coverage fix applied to one and not
    the other is a fix half the users never get.
    """
    other = extract_tables(open(app_path, encoding="utf-8").read())
    assert other.tab_meta == tables.tab_meta
    assert other.tool_features == tables.tool_features
    assert other.goal_features == tables.goal_features
    assert other.role_goals == tables.role_goals
    assert other.verify_goals == tables.verify_goals
    assert other.freetext == tables.freetext
