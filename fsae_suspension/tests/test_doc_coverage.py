# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""Every feature must be documentable to PDF, the same way Kinematics is.

Parses streamlit_app.py rather than importing it — the module is a Streamlit
entrypoint that runs the whole app on import, so a static read is the only way
to assert this in CI.

The guarantee: for each of the ~40 registered features, a user can open the tab,
hit "Document this feature", and get a PDF with that feature's results, verdicts
and charts. The failure this pins is silent — a feature added to _TAB_META but
missing from the pipeline gets no panel and no error, and nobody notices until
someone goes looking for its report at a design review.
"""
import ast
import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
APP = os.path.join(ROOT, "streamlit_app.py")


@pytest.fixture(scope="module")
def src():
    with open(APP, encoding="utf-8") as fh:
        return fh.read()


@pytest.fixture(scope="module")
def consts(src):
    """Literal module-level constants, read without executing the app."""
    out = {}
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name):
                    try:
                        out[tgt.id] = ast.literal_eval(node.value)
                    except Exception:
                        pass
    return out


def _features(consts):
    return set(consts["_TAB_META"])


# --- the core guarantee ----------------------------------------------------
def test_every_feature_is_documentable_or_explicitly_skipped(consts):
    skip = set(consts["_DOC_PANEL_SKIP"])
    missing = _features(consts) - skip
    assert missing, "sanity: some features must be documentable"
    # Nothing may be skipped that isn't a real registered feature — a stale
    # entry here silently suppresses a panel nobody knows is missing.
    assert skip <= _features(consts), \
        f"skip list names unknown features: {sorted(skip - _features(consts))}"


def test_every_skip_has_a_written_reason(consts):
    """A bare set drifts. This one had grown to hide three analysis tabs."""
    for key, reason in consts["_DOC_PANEL_SKIP"].items():
        assert isinstance(reason, str) and len(reason) > 15, \
            f"{key} is skipped without a real justification"


def test_analysis_tabs_are_not_skipped(consts):
    """Tabs that produce results about the CAR must be documentable.

    registry, model3d and weight were all silently excluded before; each
    produces engineering output a design review would ask for.
    """
    for key in ("registry", "model3d", "weight", "kinematics"):
        if key in _features(consts):
            assert key not in consts["_DOC_PANEL_SKIP"], \
                f"{key} produces analysis and must be documentable"


def test_every_feature_has_a_label_for_its_report_heading(consts):
    """_feature_label() drives the PDF's title; a missing one yields a report
    headed by a raw internal id."""
    for key, meta in consts["_TAB_META"].items():
        assert isinstance(meta, (list, tuple)) and len(meta) >= 2, key
        assert str(meta[1]).strip(), f"{key} has no display label"


def test_every_feature_maps_to_a_subsystem(consts):
    """The Integration Document groups committed features by subsystem; one
    with no mapping lands in a fallback bucket instead of its own team."""
    subsys = consts.get("_FEATURE_SUBSYS", {})
    unmapped = sorted(_features(consts) - set(subsys)
                      - set(consts["_DOC_PANEL_SKIP"]))
    assert not unmapped, f"features with no subsystem: {unmapped}"


def test_every_feature_has_a_tab_container(src, consts):
    """Without a container the tab never passes through _TabOpenProxy, so it
    gets no capture attribution and no documentation panel."""
    ids = set(re.findall(r'_id_to_container\[\s*[\'"](\w+)[\'"]\s*\]', src))
    m = re.search(r'_id_to_container\s*=\s*\{(.*?)\n\}', src, re.S)
    if m:
        ids |= set(re.findall(r'[\'"](\w+)[\'"]\s*:', m.group(1)))
    missing = sorted(_features(consts) - ids)
    assert not missing, f"features with no tab container: {missing}"


# --- the pipeline behind the panel ----------------------------------------
def test_documentation_panel_is_wired_to_the_proxy(src):
    """The panel is appended centrally in _TabOpenProxy.__exit__. If that call
    is ever removed, no feature gets a panel and no test would otherwise fail —
    render_feature_documentation() has no other call site in the app."""
    assert "_fn = globals().get(\"render_feature_documentation\")" in src
    assert "_key not in _DOC_PANEL_SKIP" in src


def test_feature_pdf_export_passes_captured_figures(src):
    """A report without its charts is the bug this whole pipeline exists to fix."""
    assert "figures=collect_report_figures([_feat])" in src


def test_all_four_pdf_exports_pass_figures(src):
    """Feature, subsystem, Integration Document and Handover. The handover was
    missed on the first pass; this pins all four."""
    assert src.count("figures=collect_report_figures") >= 4
