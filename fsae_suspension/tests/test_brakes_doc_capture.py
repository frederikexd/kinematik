# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Guard the Brakes ▸ Documentation capture pipeline.

The failure this exists to prevent is SILENT: the Documentation tab renders
happily and simply reports that nothing was captured, with no error anywhere to
trace. It happens whenever a Brakes view computes results but does not stash them
into session_state under the keys the Documentation branch reads back -- which is
exactly what the Packaging & travel view did when it was first added.

Three properties are checked, all by reading streamlit_app.py as text so no
Streamlit runtime is needed (Streamlit is an app dependency, not a test one):

  1. every `pbx_doc_*` slot the report READS is actually WRITTEN by a _capture()
     call, and every key it indexes out of that payload exists -- a writer/reader
     key mismatch is either a blank section or a KeyError mid-report;
  2. the packaging view records per-feature activity, which is what drives the
     "Nothing captured yet" panel independently of the report sections;
  3. the report never publishes a silent empty list -- an empty state must
     explain which views to open, because "nothing is captured" with no further
     guidance is the exact dead end this pipeline kept producing.
"""

import os
import re

import pytest


_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_APP = os.path.join(_ROOT, "streamlit_app.py")


@pytest.fixture(scope="module")
def app_src():
    if not os.path.exists(_APP):
        pytest.skip("streamlit_app.py not present in this checkout")
    with open(_APP, encoding="utf-8") as fh:
        return fh.read()


@pytest.fixture(scope="module")
def packaging_view(app_src):
    a = app_src.index("def _render_pedal_packaging(")
    b = app_src.index("def _render_rotor_thermal(")
    return app_src[a:b]


@pytest.fixture(scope="module")
def doc_branch(app_src):
    a = app_src.index('_pk = st.session_state.get("pbx_doc_stack")')
    b = app_src.index('publish_doc_sections("brakes", _bx)', a)
    return app_src[a:b]


def _written_keys(view_src):
    """{slot: {payload keys}} from each _capture("slot", dict(...), ...) call."""
    out = {}
    for m in re.finditer(
            r'_capture\(\s*\n?\s*"(\w+)",\s*\n\s*dict\((.*?)\),\s*\n\s*"',
            view_src, re.S):
        out[m.group(1)] = set(re.findall(r"(\w+)\s*=", m.group(2)))
    return out


def _read_keys(doc_src):
    """{slot: {keys indexed/get out of the stashed payload}}."""
    out = {}
    for var, slot in (("_pk", "stack"), ("_pl", "plan"),
                      ("_bb", "bias"), ("_tv", "travel")):
        ks = set(re.findall(re.escape(var) + r"\['(\w+)'\]", doc_src))
        ks |= set(re.findall(re.escape(var) + r'\.get\(["\'](\w+)["\']', doc_src))
        out[slot] = ks
    return out


SLOTS = ("stack", "plan", "bias", "travel")


def test_view_captures_every_documented_slot(packaging_view):
    written = _written_keys(packaging_view)
    missing = [s for s in SLOTS if s not in written]
    assert not missing, (
        f"The packaging view computes results but never stashes {missing} — "
        "the Documentation report will show nothing for them.")


def test_report_reads_only_keys_the_view_writes(packaging_view, doc_branch):
    written, read = _written_keys(packaging_view), _read_keys(doc_branch)
    broken = {}
    for slot in SLOTS:
        gap = read.get(slot, set()) - written.get(slot, set())
        if gap:
            broken[slot] = sorted(gap)
    assert not broken, (
        f"The report indexes keys the view never writes: {broken}. That is a "
        "blank section at best and a KeyError mid-report at worst.")


def test_every_captured_slot_is_actually_reported(packaging_view, doc_branch):
    """Stashing something no report reads is work that silently goes nowhere."""
    written = _written_keys(packaging_view)
    for slot in written:
        assert f"pbx_doc_{slot}" in doc_branch, (
            f"'{slot}' is captured but the Documentation branch never reads "
            f"pbx_doc_{slot} — it will never reach a report.")


def test_capture_logs_feature_activity(packaging_view):
    """The 'Nothing captured yet' panel is driven by activity, not sections."""
    assert "record_feature_activity(" in packaging_view, (
        "Without a record_feature_activity call the per-feature documentation "
        "panel stays empty even when the report has content.")


def test_capture_uses_real_activity_kinds(packaging_view):
    """A typo'd kind silently lands in the catch-all 'Other actions' bucket."""
    kinds = set(re.findall(r'_capture\(\s*\n?\s*"\w+",\s*\n\s*dict\(.*?\),\s*\n'
                           r'\s*"(\w+)"', packaging_view, re.S))
    valid = {"myth", "material", "condition", "calculation", "optimisation",
             "note"}
    assert kinds, "no _capture kinds found — the regex or the call shape moved"
    assert kinds <= valid, f"unknown activity kind(s): {sorted(kinds - valid)}"


def test_activity_is_not_relogged_on_every_rerun(packaging_view):
    """record_feature_activity dedupes by bumping a counter; firing it on every
    Streamlit rerun would show a meaningless '(x47)' against each row."""
    assert "_pbx_logged_" in packaging_view, (
        "capture must skip re-logging when the detail is unchanged")


def test_empty_report_explains_itself(app_src):
    a = app_src.index('_pk = st.session_state.get("pbx_doc_stack")')
    b = app_src.index('publish_doc_sections("brakes", _bx)', a)
    tail = app_src[a:b + 200]
    assert "if not _bx:" in tail, (
        "An empty brakes report must say which views to open rather than "
        "publishing a silent empty list.")
    assert "Packaging & travel" in tail, (
        "the empty-state guidance should name the views that populate it")


def test_capture_output_is_unit_converted(packaging_view):
    """Details land in a report a US-units member may read."""
    caps = re.findall(r"_capture\((.*?)\n\n", packaging_view, re.S)
    assert caps, "no _capture calls found"
    assert sum("_uS(" in c for c in caps) >= 3, (
        "capture details carrying mm/cc figures must go through usentence, or "
        "an imperial user gets millimetres in their report")
