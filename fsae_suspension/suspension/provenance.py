# ============================================================================
#  KinematiK — provenance / confidence badges
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""One honest provenance signal per derived output.

The failure mode a senior simulation engineer probes for is not that a
parameter is approximate — every concept-stage parameter is — but that the UI
renders an *indicative* number with the same visual confidence as a *measured*
one, so the reader cannot tell which is load-bearing. A number with a clear
provenance tag next to it is unattackable; the same number rendered bare invites
exactly that hole-poking.

This module reuses the ``EvidenceGrade`` vocabulary already defined in
``suspension/proof_engine.py`` (guess / estimate / modelled / measured /
verified, each with a conservative uncertainty band) so every tool speaks ONE
provenance language instead of each inventing its own.

Rule of use: exactly one clear provenance signal next to any output derived from
ballpark parameters — a tag, not a paragraph of hedging. The tag carries the
epistemic status; prose must not repeat it. Naming the *calibration path* (the
one measurement that upgrades the grade) turns "these params are ballpark" from
a hole a reviewer pokes into a roadmap you volunteered.

No streamlit / pandas / plotly imports at module load — the render helpers take
any st-like object so this stays unit-testable headless.
"""

from __future__ import annotations

# Per-grade UI metadata: (emoji, short tag, ± band text). The grades and their
# uncertainty numbers are the single source of truth in proof_engine; this only
# chooses how to render them. Kept in sync with proof_engine._GRADE_UNC.
_GRADE_BADGE = {
    "guess":    ("⚪", "guess",    "±40%"),
    "estimate": ("🟡", "estimate", "±20%"),
    "modelled": ("🔵", "modelled", "±10%"),
    "measured": ("🟢", "measured", "±3%"),
    "verified": ("✅", "verified", "±1%"),
}


def grade_key(grade) -> str:
    """Normalise an EvidenceGrade / enum / string to its lowercase key, safely.

    Defaults to 'estimate' — the honest middle — for anything unrecognised, so a
    bad grade can never crash a render, only under-claim confidence.
    """
    try:
        k = getattr(grade, "value", grade)
        k = str(k).strip().lower()
        return k if k in _GRADE_BADGE else "estimate"
    except Exception:
        return "estimate"


def provenance_tag(grade, *, calibrated: bool = True, extra: str = "") -> str:
    """A one-line inline provenance tag for a derived output.

    e.g. ``🔵 modelled · ±10% — closed-form surrogate``. When ``calibrated`` is
    False the grade is shown but explicitly demoted to 'uncalibrated', because an
    uncalibrated modelled number is really a guess with a shape. ``extra`` adds a
    short mechanism/source clause. Safe everywhere; never raises.
    """
    emoji, tag, band = _GRADE_BADGE[grade_key(grade)]
    if not calibrated:
        body = (f"{emoji} {tag} · **uncalibrated** — trust the shape & the "
                "delta, not the absolute")
    else:
        body = f"{emoji} {tag} · {band}"
    if extra:
        body += f" — {extra}"
    return body


def confidence_note(container, grade, *, calibrated: bool = True, extra: str = "",
                    calibrate_with: str = "") -> None:
    """Render the standard provenance caption under a block of derived metrics,
    plus — when the number is indicative — the ONE measurement that upgrades it.

    ``calibrate_with`` is that measurement (e.g. 'one TTC temperature sweep' /
    'one corner-scale pass'). ``container`` is any st-like object (st, a column,
    an expander). Never raises — a provenance signal must not be able to break a
    tool body.
    """
    if container is None:
        return
    try:
        container.caption(provenance_tag(grade, calibrated=calibrated,
                                         extra=extra))
        if (not calibrated or grade_key(grade) in ("guess", "estimate")) \
                and calibrate_with:
            container.caption(
                f"↑ Calibrate with {calibrate_with} — then this output reads as "
                "a measured number, not an indicative one.")
    except Exception:
        pass
