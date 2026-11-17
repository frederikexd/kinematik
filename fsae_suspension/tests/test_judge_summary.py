# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The judge summary: structured to the FS design score sheet, and never writes a reason the team did not give."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from suspension import judge_summary as js                       # noqa: E402


def _kw(**over):
    kw = dict(title="Suspension design summary", team="", corner_name="c", axle="front",
              specs=js.specification_rows([{"channel": "camber_deg", "target_at": -2.5, "band": 0.3,
                                            "delivered": -2.5}],
                                          {}, [{"label": "caster (deg)", "lo": 3.65, "hi": 8.0, "value": 3.0}]),
              decisions={"camber": {"target": "-2.5 deg", "result": "gain -0.035 deg/mm"}},
              rationale={}, evidence=["e"], tools=["t"], loads=[("LR", 3.47, "1.5g braking")], safety=["s"],
              questions=[("Caster: 3 deg", "outside the range 3.65 … 8.")], answers={})
    kw.update(over)
    return kw


def test_every_score_sheet_heading_is_there_and_a_missing_reason_is_said_not_filled():
    md = js.build(**_kw())
    for h in ("## 1. What we set out to achieve", "## 2. Why this design, and from what alternatives",
              "## 3. Use of design tools and analysis", "## 4. Load cases and load path",
              "## 5. Safety considerations", "## 6. Questions a judge will ask",
              "## 7. What this pre-validation does not cover"):
        assert h in md
    assert "Score-sheet area B" in md
    for _key, label, question in js.DECISIONS:              # every decision asked, its question printed with the gap
        assert f"### {label}" in md and question in md
    # why + alternatives each, the one answer, and the target chain, weighting and iterations not given
    assert md.count(js.MISSING) == 2 * len(js.DECISIONS) + 1 + 4


def test_the_teams_words_replace_the_gap_and_nothing_else_does():
    rat = {"camber": {"why": "Tyre optimum at 1.5 g.", "alternatives": "More gain.", "trade_off": "0.07 deg short."}}
    md = js.build(**_kw(rationale=rat, answers={"Caster: 3 deg": "Shimmed on the car."}))
    assert "Tyre optimum at 1.5 g." in md and "More gain." in md and "0.07 deg short." in md
    assert "Team answer: Shimmed on the car." in md
    assert md.count(js.MISSING) == 2 * (len(js.DECISIONS) - 1) + 4     # + requirements, chain, weighting, iterations


def test_specification_rows_mark_met_by_the_bounds():
    rows = js.specification_rows([{"channel": "toe_deg", "target_at": 0, "band": 0.08, "delivered": 0.1}], {},
                                 [{"label": "a", "lo": 0, "hi": None, "value": 1},
                                  {"label": "b", "lo": None, "hi": 120, "value": 118.9},
                                  {"label": "c", "lo": 63.6, "hi": 70, "value": 70.1}])
    assert [r[3] for r in rows] == [False, True, True, False]
    assert rows[1][1] == "≥ 0" and rows[2][1] == "≤ 120" and rows[3][1] == "63.6 to 70"


def test_open_questions_raise_every_red_amber_self_graded_thin_and_default():
    q = js.open_questions([{"check": "Caster", "value": 3.656, "unit": "deg", "status": "watch", "range": "3.65 … 8"},
                           {"check": "Camber gain", "value": -0.035, "unit": "deg/mm", "status": "pass",
                            "range": "x"},
                           {"check": "Wheel rate does not fall in bump", "value": "no", "unit": "", "status": "fail",
                            "range": "yes"}],
                          [("caster", "0.006")], [("Anti-dive", "68.9 %", "0 … 50")], 25, 38)
    pts = [p for p, _ in q]
    assert pts[0] == "Caster: 3.656 deg" and pts[1] == "Wheel rate does not fall in bump: no"
    assert any(p.startswith("Anti-dive") for p in pts) and any("little margin" in p for p in pts)
    assert pts[-1] == "25 of 38 analysis inputs are tool defaults" and len(pts) == 5


def test_an_answer_stays_with_its_question_when_the_numbers_change():
    assert js.answer_key("Caster: 3.656 deg") == js.answer_key("Caster: 3.71 deg") == "ig_ans_caster"
    assert js.answer_key("25 of 38 analysis inputs are tool defaults") == \
        js.answer_key("12 of 38 analysis inputs are tool defaults")
    ss = {"ig_rat_camber_why": "w", js.answer_key("Caster: 3"): "a", "ig_js_team": "T", "ig_other": "x", "k": 1}
    assert js.reasons_payload(ss) == {"ig_rat_camber_why": "w", "ig_ans_caster": "a", "ig_js_team": "T"}


# ---- one level up the V, the iterations and the weighting (Dan Jones, FS design judge, Nov 2026) ----------------
def _min_build(**extra):
    from suspension import judge_summary as js
    return js.build(title="t", team="", corner_name="c", axle="front", specs=[], decisions={}, rationale={},
                    evidence=[], tools=[], loads=[], safety=[], questions=[], answers={}, **extra)


def test_without_them_the_summary_asks_the_team_for_targets_iterations_and_weighting():
    md = _min_build()
    for head in ("### Where the car-level requirements came from", "### Where the targets came from",
                 "### How the trade-offs were weighted", "### How the design got here"):
        i = md.index(head)
        assert "Team to state" in md[i:i + 300]


def test_the_target_chain_derives_camber_gain_from_tyre_optimum_and_roll():
    import math
    from suspension import judge_summary as js
    rows = js.target_chain(track_mm=1100, body_roll_deg=1.18, tyre_optimum_camber_deg=-1.83, static_camber_deg=-2.0,
                           camber_gain_deg_per_mm=-0.0764, lateral_mu=1.55, mass_kg=300, cg_height_mm=280,
                           wheelbase_mm=1550, sprung_corner_kg=60.1, ride_hz=3.1, anti_pct=1.1,
                           sources={"tyre_optimum_camber": "default", "wheelbase_mm": "declared"})
    z = 550 * math.tan(math.radians(1.18))
    gain = next(r for r in rows if r[0] == "Camber gain")
    assert gain[3] == f"{js._num((-1.83 - 1.18 + 2.0) / z)} deg/mm"
    assert "tool default: tyre_optimum_camber" in gain[1]
    assert [r[0] for r in rows] == ["Design lateral acceleration", "Camber gain", "Camber gain delivered",
                                    "Minimum anti-dive", "Anti delivered"]
    md = _min_build(target_chain=rows, iterations=[("run 1", "worst station 1.15× band", "NO_FIT")],
                    weighting="Lap time first.")
    assert "| Camber gain |" in md and "| run 1 | worst station 1.15× band | NO_FIT |" in md
    assert "Lap time first." in md


def test_two_levels_up_the_team_states_where_the_requirements_came_from():
    from suspension import judge_summary as js
    md = _min_build(requirements="Hoosier R20 for its camber window; 3.1 Hz from driver feedback on bumps.")
    i, j = md.index("### Where the car-level requirements came from"), md.index("### Where the targets came from")
    assert i < j and "Hoosier R20 for its camber window" in md[i:j] and js.MISSING not in md[i:j]
    assert "ig_js_requirements" in js.reasons_payload({"ig_js_requirements": "x", "other": "y"})
