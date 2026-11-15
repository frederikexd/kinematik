# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""What a tab body tells the documentation capture layer (streamlit_app.py) about what it renders.

The capture layer records every table, chart and banner a tab shows, and the feature report prints them. Three things
it cannot know by itself, and a tab says here:

* ``unreported``: this block is on screen but is not a result of the design: an example the user has not replaced
  with their own numbers, a description of a loaded file, or the whole body while a loaded file has not been run.
* ``heading``: the name of the table or chart rendered next. Without one a table is known by its shape, two tables
  of one shape are one row, and the later replaces the earlier in the report.
* ``reset_on_change``: the thing being reported changed (another corner, another run), so what was recorded for the
  old one is dropped and not printed beside the new one.

Pure session-state bookkeeping: ``ss`` is any mutable mapping, so nothing here needs Streamlit. The key strings are
duplicated in streamlit_app.py, which must not import this at module scope; tests/test_report_capture.py pins that
the two agree.
"""
from __future__ import annotations

from contextlib import contextmanager

PAUSED_KEY = "_ax_capture_paused"
HEADING_KEY = "_ax_next_heading"
RESULTS_KEY = "_kinematik_feature_results"
RENDERING_KEY = "_ax_rendering_tab"


@contextmanager
def unreported(ss, active: bool = True):
    """Nothing rendered inside is captured for the report (when ``active``). Nests; restores the outer state on any
    exit, including Streamlit's rerun and stop exceptions."""
    if not active:
        yield
        return
    outer = bool(ss.get(PAUSED_KEY))
    ss[PAUSED_KEY] = True
    try:
        yield
    finally:
        if outer:
            ss[PAUSED_KEY] = True
        else:
            ss.pop(PAUSED_KEY, None)


def engage(ss, feature=None) -> None:
    """Mark the rendering tab as used this session, so the capture layer records it. The layer captures a tab only
    once it knows the user was there; the tab-open signal it relies on can be missing (a deployment where the tab
    container never reports open, or the open event failing to log), and then a tab showing a result its user
    produced captured nothing and its report was empty. A tab holding a result the user made is proof enough."""
    f = feature or ss.get(RENDERING_KEY)
    if f:
        ss[f"_ax_engaged_{f}"] = True


def clear_stale_pause(ss) -> None:
    """Drop a pause left over from a script run that ended inside ``unreported`` without unwinding it. Called where
    no pause can legitimately be open: before any tab body, and at the top of a tab body that pauses."""
    ss.pop(PAUSED_KEY, None)


def heading(ss, text) -> None:
    """Name the table or chart rendered next."""
    if text:
        ss[HEADING_KEY] = str(text)


def reset(ss, feature=None) -> None:
    """Drop everything captured for ``feature`` (default: the tab rendering now)."""
    f = feature or ss.get(RENDERING_KEY)
    if f:
        (ss.get(RESULTS_KEY) or {}).pop(str(f), None)


def forget(ss, *headings, feature=None) -> None:
    """Drop the rendering tab's captured tables and charts with these headings: a block that was reported and is
    an example again (its values set back) must leave the report, not stay in it as it last was."""
    f = feature or ss.get(RENDERING_KEY)
    got = (ss.get(RESULTS_KEY) or {}).get(str(f)) if f else None
    if got and got.get("artifacts"):
        got["artifacts"] = [a for a in got["artifacts"] if a.get("heading") not in headings]


def reset_on_change(ss, key: str, identity) -> bool:
    """Drop the rendering tab's captures when ``identity`` differs from the one stored under ``key``; True when it
    did. The first call stores the identity and resets too, so a report never starts from another session state."""
    if ss.get(key) == identity:
        return False
    ss[key] = identity
    reset(ss)
    return True
