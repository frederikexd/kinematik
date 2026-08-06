# ============================================================================
#  KinematiK — suspension/rationale.py
#  "Why did you run this, and what did you change?"
#
#  The one thing a calculation report cannot reconstruct from the numbers.
#  Six months later a handover reader can see that camber gain moved from
#  -0.35 to -0.20 deg/10mm, and has no way at all to learn that it moved
#  because the driver reported vague turn-in at Michigan and the team traded
#  camber recovery for a lower roll centre migration. That sentence is the
#  whole value of the document, and it is the only part nobody writes.
#
#  Design constraints, learned from every engineering-notes feature that has
#  ever been ignored:
#
#    1. A blank textarea gets nothing. A sentence with blanks in it gets
#       filled, because completing someone else's sentence is a much smaller
#       act than composing your own.
#    2. Never ask what the tool already knows. The app records which widgets
#       the member changed; asking "what did you change?" wastes the one
#       question they were willing to answer. Ask WHY. See changes_for().
#    3. A dropdown of common intents means the minimum viable entry is one
#       click. Typing is an upgrade, not a toll.
#    4. Missing is missing. An unanswered rationale must read as absent in the
#       report, never be quietly filled with a plausible default — a fabricated
#       reason is worse than none, because it cannot be told from a real one.
# ============================================================================
"""Structured "why" notes, one per feature, that flow into the reports."""

from __future__ import annotations

import datetime as _dt

#: Session keys.
STORE_KEY = "_kk_rationale"
CHANGES_KEY = "_kk_changes"

#: How many recorded widget edits to keep per feature. Enough to prefill the
#: "I changed …" clause; bounded so a slider being dragged cannot grow without
#: limit.
MAX_CHANGES = 12

#: Rationale entries kept per feature. The report shows the latest; the rest
#: are history a lead can scroll.
MAX_ENTRIES = 20

#: The one-click intents. Deliberately short, and phrased as the member would
#: say it out loud rather than as a form field. "Just exploring" is on the list
#: on purpose: if the honest answer is absent from the menu, people pick a
#: wrong one, and a wrong reason is worse than an honest shrug.
INTENTS = [
    "check it against a rules limit",
    "compare two design options",
    "chase something the driver reported",
    "chase something we saw in the data",
    "produce a number for the design report",
    "check a change someone else made",
    "re-run after a part or supplier changed",
    "just exploring — no decision yet",
]

#: What the run led to. Same reasoning: an "undecided" option must exist or
#: the field becomes a lie.
OUTCOMES = [
    "we're keeping this",
    "we're changing the design",
    "needs more work before deciding",
    "no decision — just recording the number",
]


# --------------------------------------------------------------------------- #
#  Recording what changed (so we never have to ask)
# --------------------------------------------------------------------------- #
def _fmt(v):
    try:
        if isinstance(v, float):
            return f"{v:g}"
        s = str(v)
        return s if len(s) <= 40 else s[:37] + "…"
    except Exception:
        return "?"


def record_change(store, feature, label, before, after):
    """Note that the member edited one input. Never raises.

    Called from the widget on_change path, which Streamlit fires only on a real
    edit — so this is a log of what a person did, not of what re-rendered.
    """
    try:
        if not feature or before == after:
            return
        rows = store.setdefault(CHANGES_KEY, {}).setdefault(str(feature), [])
        entry = {"label": str(label or "an input")[:60],
                 "from": _fmt(before), "to": _fmt(after)}
        # Same widget touched twice: keep the ORIGINAL starting value and the
        # latest landing value, so a slider dragged through ten positions reads
        # as one move from where it started to where it ended.
        for r in rows:
            if r["label"] == entry["label"]:
                r["to"] = entry["to"]
                return
        rows.append(entry)
        del rows[:-MAX_CHANGES]
    except Exception:
        pass


def changes_for(store, feature) -> list:
    try:
        return list((store.get(CHANGES_KEY) or {}).get(str(feature)) or [])
    except Exception:
        return []


def clear_changes(store, feature) -> None:
    try:
        (store.get(CHANGES_KEY) or {}).pop(str(feature), None)
    except Exception:
        pass


def describe_changes(store, feature, limit=4) -> str:
    """The 'I changed …' clause, written for the member from what they did."""
    rows = changes_for(store, feature)
    if not rows:
        return ""
    parts = [f"{r['label']} ({r['from']} → {r['to']})" for r in rows[:limit]]
    extra = len(rows) - len(parts)
    out = ", ".join(parts)
    return out + (f", and {extra} more" if extra > 0 else "")


# --------------------------------------------------------------------------- #
#  The entries themselves
# --------------------------------------------------------------------------- #
def add_entry(store, feature, *, intent, detail="", changed="",
              why_changed="", outcome="", author="", when=None) -> dict | None:
    """Record one rationale. Returns the stored entry, or None if it was empty.

    An entry with no intent and no prose is NOT stored. A row that says nothing
    still counts as an answer everywhere it is displayed, which quietly turns
    the completeness figure into a lie.
    """
    try:
        entry = {
            "intent": str(intent or "").strip(),
            "detail": str(detail or "").strip(),
            "changed": str(changed or "").strip(),
            "why_changed": str(why_changed or "").strip(),
            "outcome": str(outcome or "").strip(),
            "author": str(author or "").strip(),
            "when": when or _dt.datetime.now().strftime("%Y-%m-%d %H:%M"),
        }
        if not entry["intent"] and not entry["detail"]:
            return None
        rows = store.setdefault(STORE_KEY, {}).setdefault(str(feature), [])
        rows.append(entry)
        del rows[:-MAX_ENTRIES]
        return entry
    except Exception:
        return None


def entries_for(store, feature) -> list:
    try:
        return list((store.get(STORE_KEY) or {}).get(str(feature)) or [])
    except Exception:
        return []


def latest(store, feature):
    rows = entries_for(store, feature)
    return rows[-1] if rows else None


def has_rationale(store, feature) -> bool:
    return bool(entries_for(store, feature))


def all_features(store) -> list:
    try:
        return sorted(k for k, v in (store.get(STORE_KEY) or {}).items() if v)
    except Exception:
        return []


# --------------------------------------------------------------------------- #
#  Rendering
# --------------------------------------------------------------------------- #
def sentence(entry, feature_label="this feature") -> str:
    """One readable sentence from a filled-in entry.

    Built by joining only the clauses that were actually answered, so a
    one-click entry produces a short true sentence instead of a long one with
    holes in it.
    """
    if not entry:
        return ""
    bits = []
    intent = entry.get("intent") or ""
    detail = entry.get("detail") or ""
    if intent and detail:
        bits.append(f"Ran {feature_label} to {intent} — {detail}")
    elif intent:
        bits.append(f"Ran {feature_label} to {intent}")
    elif detail:
        bits.append(f"Ran {feature_label}: {detail}")
    changed = entry.get("changed") or ""
    if changed:
        why = entry.get("why_changed") or ""
        bits.append(f"Changed {changed}" + (f", because {why}" if why else ""))
    outcome = entry.get("outcome") or ""
    if outcome:
        bits.append(f"Outcome: {outcome}")
    s = ". ".join(b.rstrip(".") for b in bits if b)
    return (s + ".") if s else ""


def report_lines(store, feature, feature_label="this feature") -> list:
    """Markdown lines for the feature's report section, or [] if none.

    Deliberately returns [] rather than a placeholder when nothing was written.
    The report's own "no rationale recorded" line is written by the caller, so
    the absence is stated once, in the caller's voice, instead of this module
    inventing an entry that reads like a real one.
    """
    rows = entries_for(store, feature)
    if not rows:
        return []
    out = []
    for e in reversed(rows[-5:]):
        who = e.get("author") or "unattributed"
        out.append(f"- **{e.get('when', '')}** · {who} — "
                   f"{sentence(e, feature_label)}")
    if len(rows) > 5:
        out.append(f"_{len(rows) - 5} earlier note(s) not shown._")
    return out


def coverage(store, features) -> tuple:
    """(features_with_a_note, features_considered). Drives the nudge."""
    try:
        feats = [str(f) for f in features]
        return sum(1 for f in feats if has_rationale(store, f)), len(feats)
    except Exception:
        return 0, 0
