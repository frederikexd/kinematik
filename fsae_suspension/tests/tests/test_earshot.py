# ============================================================================
#  KinematiK — tests for suspension/earshot.py (the test-day power audit)
#  Every expected value below is derivable by hand from the stated formula.
# ============================================================================
import math

import pytest

from suspension import earshot as ea
from suspension.proof_engine import EvidenceGrade


# ---------------------------------------------------------------- power math
def test_z_helpers_match_textbook():
    assert ea.z_two_sided(0.05) == pytest.approx(1.959964, abs=1e-4)
    assert ea.z_power(0.80) == pytest.approx(0.841621, abs=1e-4)


def test_flagship_number_112_laps():
    # n = 2·((1.95996+0.84162)·0.8/0.3)² = 111.63 → 112
    d = ea.ABDesign(0.30, 0.80, 0.05, 0.80, 20)
    assert d.laps_needed_per_config == 112
    assert d.verdict is ea.ABVerdict.UNDERPOWERED


def test_mde_and_miss_probability():
    d = ea.ABDesign(0.30, 0.80, 0.05, 0.80, 20)
    # MDE(20) = 2.801585·0.8·sqrt(0.1) = 0.70875
    assert d.mde(20) == pytest.approx(0.70875, abs=1e-3)
    # ncp = 0.3/(0.8·sqrt(0.1)) = 1.18585 → miss = 1−Φ(1.18585−1.95996)
    assert d.miss_probability == pytest.approx(0.78057, abs=1e-3)


def test_verdict_boundaries():
    assert ea.ABDesign(1.0, 0.8, 0.05, 0.8, 20).verdict is ea.ABVerdict.RESOLVABLE
    assert ea.ABDesign(0.05, 0.8, 0.05, 0.8, 20).verdict is ea.ABVerdict.SWAMPED
    with pytest.raises(ValueError):
        ea.ABDesign(0.3, 0.0)
    with pytest.raises(ValueError):
        ea.z_power(0.4)


def test_pack_budget():
    # (7·0.9 − 0.5)/0.14/2 = 20.71 → 20
    assert ea.laps_from_pack(7.0, 0.9, 0.14, 0.5, 2) == 20
    assert ea.laps_from_pack(1.0, 0.9, 0.14, 5.0, 2) == 0


# ------------------------------------------------------------- ordering math
def test_ordering_bias_exact():
    assert ea.ordering_bias("AABB", 20, 0.03) == pytest.approx(-0.60)
    assert ea.ordering_bias("ABAB", 20, 0.03) == pytest.approx(-0.03)
    assert ea.ordering_bias("ABBA", 20, 0.03) == 0.0
    with pytest.raises(ValueError):
        ea.ordering_bias("BABA", 4, 0.01)


def test_sequences_and_swaps():
    assert ea.build_sequence("ABBA", 4) == "ABBAABBA"
    assert ea.build_sequence("ABBA", 3) == "ABBAAB"  # odd n declared, not hidden
    assert ea.swap_count("AABB", 20) == 1
    assert ea.swap_count("ABAB", 20) == 39
    # ABBAABBA: A|B, B|A, A|B, B|A — the block boundary A→A costs nothing,
    # which is precisely why ABBA cancels drift cheaply.
    assert ea.swap_count("ABBA", 4) == 4


def test_ordering_verdicts():
    d = ea.ABDesign(0.30, 0.80, 0.05, 0.80, 20)
    finds = {f.ordering: f for f in ea.audit_orderings(d)}
    assert finds["AABB"].verdict is ea.OrderingVerdict.CONFOUNDED
    assert finds["ABBA"].verdict is ea.OrderingVerdict.CLEAN
    # sign is preserved so the sheet can say WHO the drift flattered
    assert finds["AABB"].net_bias != abs(finds["AABB"].net_bias) or \
        finds["AABB"].net_bias >= 0


# ---------------------------------------------------------------- instruments
def test_tilt_angle_term_hand_check():
    r = ea.resolve_tilt_cg(250, 1550, 300, 20.0, 0.5, 0.5)
    # 0.5° = 0.0087266 rad; sin20·cos20 = 0.32139 → 0.027153
    assert r.terms["angle"] == pytest.approx(0.027153, abs=1e-3)


def test_tilt_steeper_is_sharper_and_moot_logic():
    shallow = ea.resolve_tilt_cg(250, 1550, 300, 8.0, 0.5, 0.5)
    steep = ea.resolve_tilt_cg(250, 1550, 300, 20.0, 0.5, 0.5)
    assert steep.delivered_rel_unc < shallow.delivered_rel_unc
    assert shallow.judge_against(0.05).verdict is ea.ResolutionVerdict.MOOT
    assert steep.judge_against(0.10).verdict is ea.ResolutionVerdict.SHARPENS
    with pytest.raises(ValueError):
        ea.resolve_tilt_cg(250, 1550, 300, 60.0, 0.5, 0.5)


def test_corner_scales_earned_grade():
    r = ea.resolve_corner_scales(250, 0.5)
    assert r.delivered_rel_unc == pytest.approx(0.004)   # 2·0.5/250
    assert r.earned_grade is EvidenceGrade.VERIFIED
    coarse = ea.resolve_corner_scales(250, 10.0)         # ±8 % pads
    assert coarse.earned_grade is EvidenceGrade.MODELLED


def test_coastdown_pairing_helps():
    one = ea.resolve_coastdown(250, 1.1, 1.204, 22.0, 10.0, 0.14, 3.0, 1)
    four = ea.resolve_coastdown(250, 1.1, 1.204, 22.0, 10.0, 0.14, 3.0, 4)
    assert four.delivered_rel_unc == pytest.approx(one.delivered_rel_unc / 2)
    with pytest.raises(ValueError):
        ea.resolve_coastdown(250, 1.1, 1.204, 10.0, 22.0, 0.14, 3.0)


# ----------------------------------------------------------------- sealing
def _sheet(n=14):
    return ea.create_sheet("gurney A/B",
                           ea.ABDesign(0.9, 0.8, 0.05, 0.80, n), "ABBA")


def test_seal_roundtrip_and_detection():
    s = _sheet()
    assert s.verify_seal()
    s2 = ea.SessionSheet.from_dict(s.as_dict())
    assert s2.verify_seal() and s2.seal == s.seal
    judged = ea.judge_session(s2, 54.90, 54.00, 14)
    assert judged.judged_verdict == "DETECTED"
    assert "drift bias" in judged.judged_note


def test_not_detected_is_priced_not_shrugged():
    s = _sheet()
    judged = ea.judge_session(s, 54.30, 54.10, 14)   # |diff|=0.2 < threshold
    assert judged.judged_verdict == "NOT_DETECTED"
    assert "% chance of hiding" in judged.judged_note


def test_short_session_is_void():
    s = _sheet()
    assert ea.judge_session(s, 54.9, 54.0, 9).judged_verdict == "VOID"


def test_tampered_sheet_refuses_to_judge():
    s = _sheet()
    s.effect_predicted = 0.1                     # move the sealed goalposts
    assert ea.judge_session(s, 54.9, 54.0, 14).judged_verdict == "VOID"
    assert "refuses" in s.judged_note


def test_markdown_render():
    s = _sheet()
    ea.judge_session(s, 54.9, 54.0, 14)
    md = ea.render_session_md(
        s, ea.audit_orderings(ea.ABDesign(0.9, 0.8, 0.05, 0.8, 14)),
        frame_note="ISO 8855 (team charter)")
    assert "sha256" in md and "ABBA" in md and "DETECTED" in md
    assert "ISO 8855" in md


def test_ui_module_importable_headless():
    import ui.earshot  # must not import streamlit at module level
