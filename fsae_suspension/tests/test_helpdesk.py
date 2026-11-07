"""The KinematiK Guide (suspension/helpdesk.py): a chat-style helper that answers from a fixed knowledge base.

It is not an AI, so what matters is that it (a) finds the right prewritten answer for the ways people actually ask,
(b) says "I'm not sure" instead of guessing when nothing fits, (c) is honest about what it is, and (d) never points at
a tab or an entry that does not exist.
"""
import re
from pathlib import Path

import pytest

from suspension import helpdesk as h

APP = (Path(__file__).resolve().parent.parent / "streamlit_app.py").read_text(encoding="utf-8")
CTX = {"car_basics": "280 kg · CG 300 mm · 47 % front", "springs": "springs 35.0 / 35.0 N/mm",
       "geometry": "Double wishbone · Example front corner", "start_plan": "PLAN-FOR-YOU"}


@pytest.fixture(scope="module")
def guide():
    return h.build_guide()


# --- matching the ways people ask --------------------------------------------------------------------------------
@pytest.mark.parametrize("question, expected", [
    ("where do i change the weight of the car", "change_mass"),
    ("I want to change the CG", "change_mass"),
    ("weight distribution", "change_mass"),
    ("where are springs", "change_springs"),
    ("sway bar stiffness", "change_springs"),
    ("how do I edit pickup points", "change_geometry"),
    ("hardponts", "change_geometry"),                      # typo
    ("can i import optimum k files", "import"),
    ("reset everything", "reset_geometry"),
    ("metric to inches", "units"),
    ("which way is z", "frames"),
    ("save my project", "save"),
    ("how does the free trial work", "preview"),
    ("how accurate is this", "accuracy"),
    ("does this replace adams", "replace_adams"),
    ("what does a plausibility warning mean", "garbage"),
    ("what are my current numbers", "my_car"),
    ("is it open source", "license"),
    ("found a bug", "feedback"),
    ("whats bumpsteer", "bump_steer"),
    ("what is bump steer", "bump_steer"),
    ("camber", "camber_gain"),
    ("roll centre?", "roll_center"),                       # British spelling
    ("what's the anti squat", "anti"),
    ("what is the steering axis", "kingpin"),
    ("motion ratio", "motion_ratio"),
    ("understeer", "understeer"),
    ("what should i do first", "start"),
    ("im lost", "start"),
])
def test_common_questions_find_their_answer(guide, question, expected):
    r = guide.reply(question, CTX)
    assert r.confident and r.entry_id == expected, (question, r.entry_id, r.text[:80])


def test_answers_carry_the_live_values_from_the_app(guide):
    assert "280 kg · CG 300 mm · 47 % front" in guide.reply("where do I change the mass", CTX).text
    assert guide.reply("what should I do first", CTX).text == "PLAN-FOR-YOU"
    # a placeholder the app did not fill stays visible instead of raising
    assert "{car_basics}" in guide.reply("where do I change the mass", {}).text


# --- honesty and small talk ---------------------------------------------------------------------------------------
@pytest.mark.parametrize("q", ["are you an AI?", "are you chatgpt", "who are you", "is this an llm"])
def test_it_says_what_it_is(guide, q):
    r = guide.reply(q)
    assert r.entry_id == "_who" and "not an AI" in r.text and "leaves this page" in r.text


@pytest.mark.parametrize("q, eid", [("hi", "_greet"), ("Hello!", "_greet"), ("help", "_help"),
                                    ("what can I ask you?", "_help"), ("thanks!", "_thanks"),
                                    ("what is kinematik", "what_is"), ("whats this app", "what_is")])
def test_small_talk(guide, q, eid):
    assert guide.reply(q).entry_id == eid


def test_it_does_not_guess(guide):
    for q in ["asdfgh", "qwertyuiop zxcv", "", "   "]:
        r = guide.reply(q)
        assert r.entry_id is None and r.suggestions, q
    r = guide.reply("asdfgh")
    assert not r.confident and "don't have an answer" in r.text


def test_a_vague_question_gets_choices_not_an_answer():
    g = h.Guide([h.Entry("a", "Where are the brake discs?", "A", ("brake", "disc")),
                 h.Entry("b", "Where are the brake pads?", "B", ("brake", "pad")),
                 h.Entry("c", "What is the steering rack?", "C", ("steering", "rack"))])
    r = g.reply("brake pad disc rack")             # covers several entries, none well enough
    assert not r.confident and r.entry_id is None
    assert len(r.suggestions) == len(set(r.suggestions)) >= 2


# --- clicking a suggestion ----------------------------------------------------------------------------------------
def test_a_clicked_suggestion_answers_that_entry_exactly(guide):
    title = guide.by_id["roll_center"].title
    r = guide.reply_to_title(title, CTX)
    assert r.entry_id == "roll_center" and r.suggestions
    assert guide.reply_to_title("not a title at all", CTX).entry_id is None


def test_welcome_offers_the_starters(guide):
    w = guide.welcome()
    assert w.suggestions == [guide.by_id[i].title for i in h.STARTERS]


# --- the knowledge base is internally consistent ------------------------------------------------------------------
def test_every_related_id_and_starter_exists(guide):
    for e in h.FAQ:
        for r in e.related:
            assert r in guide.by_id, (e.id, r)
    for s in h.STARTERS:
        assert s in guide.by_id


def test_every_tab_the_faq_points_to_exists_in_the_app():
    meta = APP[APP.index("_TAB_META = {"):APP.index("_FULL_ORDER = list(_TAB_META.keys())")]
    ids = set(re.findall(r'^\s+"([a-z_0-9]+)":\s+\(', meta, re.M))
    assert {"kinematics", "roll", "setup", "laptime"} <= ids
    for e in h.FAQ:
        for t in e.tabs:
            assert t in ids, (e.id, t)


def test_ids_and_titles_are_unique_and_app_entries_cannot_override_the_faq():
    dup_title = h.Entry("goal:x", h.FAQ[0].title, "other")
    dup_id = h.Entry(h.FAQ[1].id, "Another title", "other")
    fresh = h.Entry("tab:new", "What is New for?", "new")
    g = h.build_guide([dup_title, dup_id, fresh, h.Entry("tab:new2", "What is New for?", "again")])
    assert g.by_id[h.FAQ[1].id].title == h.FAQ[1].title
    assert [e.id for e in g.entries].count("tab:new") == 1 and "tab:new2" not in g.by_id
    assert len({e.title for e in g.entries}) == len(g.entries)


def test_faq_spelling_is_american_in_answers():
    for e in h.FAQ:
        # (the tab names "Setup Optimiser" and "Roll & Load Transfer" are proper names and stay as they are)
        for word in ("centre", "tyre", "colour", "behaviour"):
            assert word not in e.answer.lower(), (e.id, word)


# --- text handling ------------------------------------------------------------------------------------------------
def test_tokens_fold_phrases_spelling_and_shorthand():
    assert h.tokens("Roll centre") == h.tokens("roll center") == h.tokens("RC") == ["rollcenter"]
    assert h.tokens("tyres") == h.tokens("tires") == ["tire"]
    assert h.tokens("ARBs") == h.tokens("anti-roll bars") == h.tokens("sway bar") == ["antirollbar"]
    assert h.tokens("springs") == h.tokens("spring") == ["spring"]          # no over-stemming to "spr"
    assert h.tokens("the of and") == []


def test_stream_words_rebuilds_the_text_exactly():
    text = "Open **🚗 Car basics**\n\n- one\n- two  three"
    assert "".join(h.stream_words(text)) == text
    assert list(h.stream_words("")) == []


def test_rank_is_empty_for_unknown_words(guide):
    assert guide.rank("zzzz qqqq") == []


# --- with the app's own entries added (as the app builds it) ------------------------------------------------------
def _app_tables():
    import ast
    want = {"_TAB_META", "_BRIEF_SIMPLE", "_BRIEF_TOOLS", "_ROLE_GOALS", "_ROLE_LABELS"}
    found = {}
    for node in ast.parse(APP).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and getattr(node.targets[0], "id", None) in want:
            found[node.targets[0].id] = ast.literal_eval(node.value)
    assert set(found) == want
    return found


@pytest.fixture(scope="module")
def app_guide():
    t = _app_tables()
    return h.build_guide(h.app_entries(t["_TAB_META"], t["_BRIEF_SIMPLE"], t["_BRIEF_TOOLS"], t["_ROLE_GOALS"],
                                       t["_ROLE_LABELS"]))


def test_the_app_entries_do_not_steal_the_faq_answers(app_guide):
    for q, expected in [("where do i change the weight of the car", "change_mass"), ("what is bump steer",
                        "bump_steer"), ("what should i do first", "start"), ("how accurate is this", "accuracy"),
                        ("save my project", "save"), ("roll centre?", "roll_center")]:
        r = app_guide.reply(q, CTX)
        assert r.confident and r.entry_id == expected, (q, r.entry_id)


def test_tab_and_goal_questions_are_answered_from_the_app_copy(app_guide):
    r = app_guide.reply("tyre grip")
    assert r.entry_id == "tab:tire" and r.tabs == ["tire"]
    r = app_guide.reply("what is the lap time tab for")
    assert r.entry_id == "tab:laptime"
    goal = next(e for e in app_guide.entries if e.id.startswith("goal:"))
    r = app_guide.reply_to_title(goal.title)
    assert r.entry_id == goal.id and r.text.startswith("Open these, in order:\n1. **")
    assert len({e.title for e in app_guide.entries}) == len(app_guide.entries)


def test_app_entries_skip_tabs_without_copy_and_use_whatever_copy_exists():
    meta = {"a": ("🅰️", "Alpha"), "b": ("🅱️", "Beta"), "c": ("©️", "Gamma")}
    es = h.app_entries(meta, {"a": "Plain words."}, {"b": ("What.", "Why.", "Why here.")},
                       {"r": [("g1", "Do the thing", ["b", "zz", "a"])]}, {"r": "Role R"})
    by = {e.id: e for e in es}
    assert set(by) == {"tab:a", "tab:b", "goal:g1"}                 # "c" has no copy, "zz" is not a tab
    assert by["tab:a"].answer == "Plain words."
    assert by["tab:b"].answer == "What.\n\n**Why here first:** Why here."
    assert by["goal:g1"].answer == "Open these, in order:\n1. **🅱️ Beta**: What.\n2. **🅰️ Alpha**"
    assert by["goal:g1"].tabs == ("b",) and "Role R" in by["goal:g1"].keywords
