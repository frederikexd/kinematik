# ============================================================================
#  Tests for suspension/provenance.py — the shared confidence-badge helpers.
# ============================================================================
"""Guards two things: (1) the badge ± bands never drift from the single source
of truth in proof_engine.EvidenceGrade, and (2) the render helpers can never
raise, because a provenance signal must not be able to break a tool body."""

from suspension.provenance import (
    provenance_tag, confidence_note, grade_key, _GRADE_BADGE,
)
from suspension.proof_engine import _GRADE_UNC, EvidenceGrade


def test_badge_bands_match_proof_engine():
    # Every grade's displayed ± band equals proof_engine's uncertainty number.
    for k, unc in _GRADE_UNC.items():
        assert _GRADE_BADGE[k][2] == f"±{int(round(unc * 100))}%", k


def test_all_grades_have_a_badge():
    for g in EvidenceGrade:
        assert g.value in _GRADE_BADGE


def test_grade_key_normalises_and_is_safe():
    assert grade_key(EvidenceGrade.MODELLED) == "modelled"
    assert grade_key("MEASURED") == "measured"
    assert grade_key(" Verified ") == "verified"
    # Anything unrecognised falls back to the honest middle, never raises.
    assert grade_key("banana") == "estimate"
    assert grade_key(None) == "estimate"
    assert grade_key(object()) == "estimate"


def test_provenance_tag_calibrated_vs_not():
    cal = provenance_tag("modelled", calibrated=True)
    assert "±10%" in cal and "uncalibrated" not in cal
    unc = provenance_tag("modelled", calibrated=False)
    assert "uncalibrated" in unc and "shape" in unc


def test_provenance_tag_extra_clause():
    assert provenance_tag("estimate", extra="1D ladder").endswith("1D ladder")


def test_confidence_note_never_raises():
    calls = []

    class _Stub:
        def caption(self, s):
            calls.append(s)

    stub = _Stub()
    confidence_note(stub, "modelled", calibrated=False,
                    calibrate_with="one TTC sweep")
    assert calls  # rendered the tag
    assert any("Calibrate with" in c for c in calls)  # and the upgrade path

    # A broken container must be swallowed, not propagated.
    class _Boom:
        def caption(self, s):
            raise RuntimeError("render blew up")

    confidence_note(_Boom(), "guess", calibrate_with="x")  # must not raise
    confidence_note(None, "guess")  # None container is a silent no-op


def test_calibrate_path_only_shown_when_indicative():
    seen = []

    class _S:
        def caption(self, s):
            seen.append(s)

    s = _S()
    # measured + calibrated → no calibrate-with line
    confidence_note(s, "measured", calibrated=True, calibrate_with="a scale")
    assert not any("Calibrate with" in c for c in seen)
