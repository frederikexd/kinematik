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


# --------------------------------------------------------------------------- #
#  The gate: a number cannot reach a reader without its grade
# --------------------------------------------------------------------------- #
#  The helpers above are good and nothing forces anyone to call them. That is
#  the whole remaining gap: a placeholder tyre coefficient and a hand-verified
#  bolt stress area still format identically if someone writes f"{x:.2f}".
#
#  `graded()` is the single formatter the report and metric paths should use. It
#  takes the grade as a REQUIRED argument, so omitting provenance becomes a
#  TypeError at the call site rather than a judgement call nobody makes at 2am.
#  Pair it with test_repo_accuracy_audit's report-path check, which fails CI on
#  a bare float in a graded context.

_GRADE_ORDER = ["guess", "estimate", "modelled", "measured", "verified"]


def graded(value, grade, unit: str = "", *, digits: int = 4,
           calibrated: bool = True, limited_by: str = "") -> str:
    """Format a number together with its evidence grade. Never bare.

    `grade` is positional and required — that is the point. A number rendered
    through this function carries its pedigree; a number rendered through an
    f-string does not, and the reader cannot tell the two apart afterwards.

    `limited_by` names the input that set the grade, which is the actionable
    half: "MODELLED, limited by cg_height" is a work order, "MODELLED" alone is
    only a worry.
    """
    key = grade_key(grade)
    emoji, tag, band = _GRADE_BADGE[key]
    try:
        v = f"{float(value):.{digits}g}"
    except (TypeError, ValueError):
        v = str(value)
    u = f" {unit}" if unit else ""
    suffix = tag if calibrated else f"{tag}, uncalibrated"
    out = f"{v}{u} {emoji} {suffix} · {band if calibrated else 'trust the delta'}"
    if limited_by:
        out += f" (limited by {limited_by})"
    return out


def worst_grade(*grades) -> str:
    """Weakest-link grade key over any mix of grades, enums or strings.

    Mirrors proof_engine.aggregate_grades for code that has only strings to
    hand. Averaging would let good evidence launder bad — three measured inputs
    do not rescue a guessed fourth — so this takes the minimum, always.
    """
    keys = [grade_key(g) for g in grades] or ["estimate"]
    return min(keys, key=lambda k: _GRADE_ORDER.index(k))


def render_report_value(container, label: str, value, grade, unit: str = "",
                        *, calibrated: bool = True, limited_by: str = "",
                        calibrate_with: str = "") -> None:
    """Emit one labelled, graded number plus its upgrade path, if any.

    The upgrade path is the product promise in one line: this tool's job is to
    say what to go measure or simulate next, so a weak number should always
    arrive with the single action that strengthens it.
    """
    if container is None:
        return
    try:
        container.markdown(
            f"**{label}:** {graded(value, grade, unit, calibrated=calibrated, limited_by=limited_by)}")
        if calibrate_with and (not calibrated or grade_key(grade) in ("guess", "estimate")):
            container.caption(f"↑ Upgrade with {calibrate_with}.")
    except Exception:
        pass
