# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_mission_briefing.py — the mission-briefing engine, pinned.
# ============================================================================
"""What these tests guard, without importing Streamlit:

The mission briefing lives in streamlit_app.py, which imports Streamlit and so
can't be imported in a headless CI run. But the parts that matter here are pure
data (the per-tool copy dicts) and one pure compiler (_build_briefing). We reach
them by parsing the module's AST and either inspecting the dict literals or
exec-ing just the compiler plus its real dependencies — so these tests exercise
the ACTUAL code, not a reimplementation.

The invariants, in the order they'd silently rot:

* COVERAGE — every tab in _TAB_META has briefing copy in _BRIEF_TOOLS, a
  plain-English gloss in _BRIEF_SIMPLE, and a feature list in
  _BRIEF_TOOL_FEATURES. A new tab added without briefing copy is the classic
  regression (it happened to genesis_fc/frames/phantom_env/cost); this fails
  the moment it recurs.
* REACHABILITY — the full-car synthesis goals actually surface genesis_fc, so
  the FullCar tab is recommendable from the questionnaire rather than orphaned.
* PROFICIENCY — _build_briefing accepts and stores the proficiency axis, and a
  briefing compiles at each level (beginner/intermediate/advanced) with the
  chosen tools intact. This is the subsystem/goal/proficiency trio the upgrade
  added.
"""

import ast
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_APP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "streamlit_app.py")


def _load_app_tree():
    with open(_APP, encoding="utf-8") as f:
        src = f.read()
    return src, ast.parse(src)


def _dict_keys(tree, name):
    """Keys of a top-level `name = { ... }` dict literal, or None."""
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if (isinstance(t, ast.Name) and t.id == name
                        and isinstance(node.value, ast.Dict)):
                    return [k.value for k in node.value.keys
                            if isinstance(k, ast.Constant)]
    return None


def _tab_ids(tree):
    keys = _dict_keys(tree, "_TAB_META")
    assert keys, "_TAB_META not found as a dict literal"
    return set(keys)


# --------------------------------------------------------------------------- #
#  Coverage — every tab is briefed at every depth
# --------------------------------------------------------------------------- #
def test_every_tab_has_full_briefing_copy():
    _src, tree = _load_app_tree()
    tabs = _tab_ids(tree)
    for dict_name in ("_BRIEF_TOOLS", "_BRIEF_SIMPLE", "_BRIEF_TOOL_FEATURES"):
        keys = set(_dict_keys(tree, dict_name) or [])
        missing = sorted(tabs - keys)
        assert not missing, f"{dict_name} is missing copy for: {missing}"


def test_new_tools_specifically_covered():
    # the four that were missing before this upgrade — a targeted guard
    _src, tree = _load_app_tree()
    for dict_name in ("_BRIEF_TOOLS", "_BRIEF_SIMPLE", "_BRIEF_TOOL_FEATURES"):
        keys = set(_dict_keys(tree, dict_name) or [])
        for tid in ("genesis_fc", "frames", "phantom_env", "cost"):
            assert tid in keys, f"{tid} absent from {dict_name}"


# --------------------------------------------------------------------------- #
#  The pure compiler — exec _build_briefing with its real dependencies
# --------------------------------------------------------------------------- #
def _load_build_briefing():
    src, tree = _load_app_tree()
    need = {"_BRIEF_PURPOSES", "_ROLE_GOALS", "_VERIFY_GOALS",
            "_FREETEXT_KEYWORDS", "_FULL_ORDER", "_TAB_META", "_BRIEF_TOOLS",
            "_build_briefing", "_brief_goal_options",
            "_freetext_matched_tools", "_BRIEF_PURPOSE_MAP"}
    segs = []
    for node in tree.body:
        names = []
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target,
                                                            ast.Name):
            names = [node.target.id]
        elif isinstance(node, ast.FunctionDef):
            names = [node.name]
        if names and any(n in need for n in names):
            segs.append((node.lineno, ast.get_source_segment(src, node)))
    segs.sort()
    ns: dict = {}
    exec("from __future__ import annotations\n"
         + "\n".join(s for _, s in segs), ns)  # noqa: S102 — trusted repo source
    return ns


def test_build_briefing_accepts_and_stores_proficiency():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    for prof in ("beginner", "intermediate", "advanced"):
        bf = build(["suspension"], "design", [], "", style="visual",
                   proficiency=prof)
        assert bf["proficiency"] == prof
        assert bf["style"] == "visual"


def test_fullcar_goal_recommends_genesis_fc_at_every_level():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    opts = {k: (lab, ids)
            for k, lab, ids in ns["_brief_goal_options"](["powertrain"])}
    synth = next((k for k, (lab, ids) in opts.items()
                  if "genesis_fc" in ids), None)
    assert synth, "no powertrain goal surfaces genesis_fc"
    for prof in ("beginner", "intermediate", "advanced"):
        bf = build(["powertrain"], "design", [synth], "",
                   style="visual", proficiency=prof)
        assert "genesis_fc" in bf["core_tabs"], \
            f"genesis_fc not recommended at {prof}"


def test_default_proficiency_is_intermediate():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    bf = build(["suspension"], "design", [], "")   # no proficiency passed
    assert bf["proficiency"] == "intermediate"


def test_freetext_still_expands_the_toolbox():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    bf = build(["powertrain"], "design", [], "pack overheats in endurance",
               style="numbers", proficiency="advanced")
    # the note should pull in at least one energy/thermal-related tab
    assert bf["note_tabs"], "freetext note added no tools"
