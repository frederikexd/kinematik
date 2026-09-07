# ============================================================================
#  KinematiK — tests for suspension/fusebox.py (the failure-order audit)
#  Every expected value below is derivable by hand from the stated formula.
# ============================================================================
import math

import pytest

from suspension import fusebox as fb
from suspension.fusebox import (PathElement, OverloadPath, Severity,
                                PathVerdict, IncidentVerdict)
from suspension.proof_engine import EvidenceGrade


def _el(key, fos, grade=EvidenceGrade.MODELLED, sev=Severity.S2_STRUCTURAL,
        cost=100.0, days=1.0, **kw):
    return PathElement(key, key.replace("_", " "), fos, grade, sev,
                       cost, days, **kw)


# ------------------------------------------------------------- first-failure
def test_pairwise_matches_napkin_formula():
    # Φ((1.8−1.35)/√(0.135²+0.72²)) with σ = FoS·rel_unc
    a = _el("a", 1.35, EvidenceGrade.MODELLED)   # ±10% → σ 0.135
    b = _el("b", 1.80, EvidenceGrade.GUESS)      # ±40% → σ 0.72
    z = (1.80 - 1.35) / math.sqrt(0.135**2 + 0.72**2)
    expect = 0.5 * (1 + math.erf(z / math.sqrt(2)))
    assert fb.pairwise_first(a.mu, a.sigma, b.mu, b.sigma) == \
        pytest.approx(expect, abs=1e-9)


def test_quadrature_collapses_to_pairwise_for_two():
    a, b = _el("a", 1.35), _el("b", 1.80, EvidenceGrade.GUESS)
    p = fb.first_failure_probs([a, b])
    assert p["a"] == pytest.approx(
        fb.pairwise_first(a.mu, a.sigma, b.mu, b.sigma), abs=1e-4)
    assert sum(p.values()) == pytest.approx(1.0, abs=1e-9)


def test_flagship_coin_flip_27_percent():
    # MODELLED FoS 1.35 vs GUESS FoS 1.8: the guess wins the race ~27%.
    p = fb.first_failure_probs([_el("tie", 1.35),
                                _el("upr", 1.80, EvidenceGrade.GUESS)])
    assert 0.25 < p["upr"] < 0.29


def test_probs_deterministic_and_ordering_sane():
    els = [_el("a", 1.2), _el("b", 1.5), _el("c", 2.0)]
    p1 = fb.first_failure_probs(els)
    p2 = fb.first_failure_probs(els)
    assert p1 == p2                                     # byte-identical
    assert p1["a"] > p1["b"] > p1["c"]                  # lower FoS, first
    assert fb.first_failure_probs([_el("solo", 1.5)]) == {"solo": 1.0}


def test_sigma_uses_proof_engine_band_law_with_staleness():
    fresh = _el("x", 2.0, EvidenceGrade.MEASURED)
    stale = _el("y", 2.0, EvidenceGrade.MEASURED, age_days=10_000.0)
    assert fresh.sigma == pytest.approx(2.0 * 0.03)
    assert stale.sigma > fresh.sigma        # staleness inflates, same law


def test_fos_must_be_positive():
    with pytest.raises(ValueError):
        _el("bad", 0.0)


# ------------------------------------------------------------------ verdicts
def _path(els, fuse=""):
    return OverloadPath("p", "P", "story", els, designated_fuse_key=fuse)


def test_fused_when_fuse_dominates():
    a = fb.audit_path(_path(
        [_el("fuse", 1.10, EvidenceGrade.MEASURED, Severity.S1_FUSE_GRADE,
             40, 0.2),
         _el("big", 2.50, EvidenceGrade.MODELLED)], "fuse"))
    assert a.verdict is PathVerdict.FUSED
    assert a.fuse_p >= fb.DEFAULT_CONFIDENCE


def test_coin_flip_names_contenders():
    a = fb.audit_path(_path(
        [_el("fuse", 1.35, sev=Severity.S1_FUSE_GRADE),
         _el("upr", 1.80, EvidenceGrade.GUESS)], "fuse"))
    assert a.verdict is PathVerdict.COIN_FLIP
    assert set(a.contenders) == {"fuse", "upr"}


def test_inverted_when_structural_leads():
    a = fb.audit_path(_path(
        [_el("fuse", 2.0, EvidenceGrade.MEASURED, Severity.S1_FUSE_GRADE,
             45, 0.2),
         _el("upr", 1.30, EvidenceGrade.MODELLED, cost=900, days=21)],
        "fuse"))
    assert a.verdict is PathVerdict.INVERTED
    assert a.leader_key == "upr"
    assert "$900" in a.headline


def test_unfused_when_no_s1_exists():
    a = fb.audit_path(_path([_el("a", 1.5), _el("b", 2.0)], "a"))
    assert a.verdict is PathVerdict.UNFUSED


def test_breach_risk_outranks_everything():
    a = fb.audit_path(_path(
        [_el("fuse", 1.10, EvidenceGrade.MEASURED, Severity.S1_FUSE_GRADE),
         _el("accu", 2.0, EvidenceGrade.GUESS, Severity.S3_FORBIDDEN)],
        "fuse"))
    assert a.verdict is PathVerdict.BREACH_RISK
    assert a.forbidden_hits and a.forbidden_hits[0][0] == "accu"
    # sharpen the grade — the breach retires with zero new metal
    a2 = fb.audit_path(_path(
        [_el("fuse", 1.10, EvidenceGrade.MEASURED, Severity.S1_FUSE_GRADE),
         _el("accu", 2.0, EvidenceGrade.MEASURED, Severity.S3_FORBIDDEN)],
        "fuse"))
    assert a2.verdict is not PathVerdict.BREACH_RISK


def test_empty_path_is_a_blind_spot_not_a_pass():
    a = fb.audit_path(OverloadPath("e", "Empty", "s", [], ""))
    assert a.verdict is PathVerdict.UNFUSED
    assert a.blind_spots


def test_missing_severity_reported_never_silent():
    a = fb.audit_path(_path(
        [_el("fuse", 1.1, EvidenceGrade.MEASURED, Severity.S1_FUSE_GRADE),
         PathElement("mys", "Mystery", 2.5)], "fuse"))
    assert any("severity" in b for b in a.blind_spots)


def test_expected_bill_is_probability_weighted():
    els = [_el("a", 1.3, cost=100, days=1), _el("b", 1.6, cost=1000, days=10)]
    a = fb.audit_path(_path(els + [], "a"))
    p = fb.first_failure_probs(els)
    assert a.expected_cost_usd == pytest.approx(
        p["a"] * 100 + p["b"] * 1000, rel=1e-9)
    assert a.expected_downtime_days == pytest.approx(
        p["a"] * 1 + p["b"] * 10, rel=1e-9)


# ------------------------------------------------------------ fix arithmetic
def test_prescriptions_solve_pairwise_exactly():
    path = _path([_el("fuse", 1.35, sev=Severity.S1_FUSE_GRADE),
                  _el("upr", 1.80, EvidenceGrade.GUESS)], "fuse")
    pr = fb.prescribe(path, 0.95)[0]
    fuse, upr = path.element("fuse"), path.element("upr")
    if pr.raise_rival_fos_to is not None:
        m = pr.raise_rival_fos_to
        assert fb.pairwise_first(fuse.mu, fuse.sigma, m, m * upr.rel_unc) \
            == pytest.approx(0.95, abs=5e-4)          # 3-dp display rounding
    if pr.lower_fuse_fos_to is not None:
        m = pr.lower_fuse_fos_to
        assert fb.pairwise_first(m, m * fuse.rel_unc, upr.mu, upr.sigma) \
            == pytest.approx(0.95, abs=5e-4)


def test_measure_dont_machine_lever():
    path = _path([_el("fuse", 1.35, sev=Severity.S1_FUSE_GRADE),
                  _el("upr", 1.80, EvidenceGrade.GUESS)], "fuse")
    pr = fb.prescribe(path, 0.95)[0]
    assert pr.sharpen_rival_to is EvidenceGrade.MODELLED
    # and it works: swap the grade in, ordering restored
    fixed = _path([_el("fuse", 1.35, sev=Severity.S1_FUSE_GRADE),
                   _el("upr", 1.80, EvidenceGrade.MODELLED)], "fuse")
    assert fb.pairwise_first(fixed.element("fuse").mu,
                             fixed.element("fuse").sigma,
                             fixed.element("upr").mu,
                             fixed.element("upr").sigma) >= 0.95


def test_fuse_floor_respected():
    # dominating a rival at FoS 1.2 would need the fuse below the floor
    path = _path([_el("fuse", 1.35, sev=Severity.S1_FUSE_GRADE),
                  _el("rival", 1.22, EvidenceGrade.ESTIMATE)], "fuse")
    pr = fb.prescribe(path, 0.95)[0]
    assert pr.lower_fuse_fos_to is None
    assert "floor" in pr.lower_fuse_note or "infeasible" in pr.lower_fuse_note


def test_infeasible_raise_when_band_grows_with_fos():
    # z·r ≥ 1: at 95% (z≈1.645) a GUESS rival (r=0.40)·z = 0.658 is feasible,
    # but a stale guess capped band can't exceed 0.40 — construct r via a
    # tighter confidence instead: 0.995 → z≈2.576, z·0.40 = 1.03 > 1.
    path = _path([_el("fuse", 1.35, EvidenceGrade.VERIFIED,
                      Severity.S1_FUSE_GRADE),
                  _el("upr", 1.80, EvidenceGrade.GUESS)], "fuse")
    pr = fb.prescribe(path, 0.995)[0]
    assert pr.raise_rival_fos_to is None
    assert "no amount of metal" in pr.raise_rival_note.lower() or \
        "infeasible" in pr.raise_rival_note.lower()


def test_satisfied_rivals_not_prescribed():
    path = _path([_el("fuse", 1.10, EvidenceGrade.MEASURED,
                      Severity.S1_FUSE_GRADE),
                  _el("big", 3.0, EvidenceGrade.MEASURED)], "fuse")
    assert fb.prescribe(path, 0.90) == []


# ------------------------------------------------------------------- charter
def test_charter_seal_roundtrip_and_tamper():
    ch = fb.create_charter({"p": "fuse"}, 0.9, 0.01, "meeting 7")
    assert fb.charter_intact(ch)
    ch["forbidden_p"] = 0.4
    assert not fb.charter_intact(ch)


def test_charter_validation():
    with pytest.raises(ValueError):
        fb.create_charter({}, confidence=0.4)
    with pytest.raises(ValueError):
        fb.create_charter({}, forbidden_p=0.6)


def test_incident_verdicts_and_free_datum():
    path = _path([_el("fuse", 1.1, EvidenceGrade.MEASURED,
                      Severity.S1_FUSE_GRADE),
                  _el("upr", 2.0),
                  _el("accu", 2.5, EvidenceGrade.GUESS,
                      Severity.S3_FORBIDDEN)], "fuse")
    ch = fb.create_charter({"p": "fuse"})
    v, msg = fb.judge_incident(ch, path, "fuse")
    assert v is IncidentVerdict.AS_DESIGNED and "Free datum" in msg
    v, msg = fb.judge_incident(ch, path, "upr")
    assert v is IncidentVerdict.SURPRISE and "Free datum" in msg
    v, _ = fb.judge_incident(ch, path, "accu")
    assert v is IncidentVerdict.BREACH


def test_tampered_charter_refuses_to_judge():
    path = _path([_el("fuse", 1.1, sev=Severity.S1_FUSE_GRADE)], "fuse")
    ch = fb.create_charter({"p": "fuse"})
    ch["designations"] = {"p": "someone_else"}
    with pytest.raises(ValueError, match="seal"):
        fb.judge_incident(ch, path, "fuse")


def test_unknown_element_rejected():
    path = _path([_el("fuse", 1.1, sev=Severity.S1_FUSE_GRADE)], "fuse")
    ch = fb.create_charter({"p": "fuse"})
    with pytest.raises(ValueError):
        fb.judge_incident(ch, path, "ghost")


# ---------------------------------------------------------------- seeds + md
def test_seeds_tell_the_documented_story():
    paths = fb.seed_paths()
    audits = {p.key: fb.audit_path(p) for p in paths}
    assert audits["front_curb"].verdict is PathVerdict.COIN_FLIP
    assert audits["side_accu"].verdict is PathVerdict.BREACH_RISK
    assert audits["side_accu"].forbidden_hits[0][0] == "accu_mount"


def test_markdown_export_deterministic_and_complete():
    paths = fb.seed_paths()
    audits = [fb.audit_path(p) for p in paths]
    ch = fb.create_charter({p.key: p.designated_fuse_key for p in paths})
    md1 = fb.render_fusebox_md(paths, audits, ch)
    md2 = fb.render_fusebox_md(paths, audits, ch)
    assert md1 == md2
    for token in ("BREACH-RISK", "sha256", "⛓️", "Expected overload bill"):
        assert token in md1
    ch["confidence"] = 0.6
    assert "SEAL BROKEN" in fb.render_fusebox_md(paths, audits, ch)


def test_roundtrip_serialisation():
    p = fb.seed_paths()[0]
    p2 = OverloadPath.from_dict(p.as_dict())
    assert fb.first_failure_probs(p.elements) == \
        fb.first_failure_probs(p2.elements)
