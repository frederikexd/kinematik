# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""Tests for the handover document's coverage of what the app produces.

Producers that had no consumer here: stamped reports and the Integration
Document. And a consumer that was fed fabricated data: the caller coerced every
unreadable geometry value to 0.0, so a handover could state
"scrub_radius_mm: 0.00" — a plausible number nobody measured.
"""
import dataclasses
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from suspension import project as pj                      # noqa: E402


def _store():
    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    os.unlink(path)
    s = pj.ProjectStore(path)
    s.team_name, s.season = "Elbee Racing", "2026"
    return s


def _a_report(title="Bump steer sweep", team="suspension"):
    from suspension.report_store import ReportRecord
    kw = {f.name: (f.default if f.default is not dataclasses.MISSING else "")
          for f in dataclasses.fields(ReportRecord)}
    kw.update(title=title, team=team, part="front upright",
              date="2026-07-14", content_hash="a1b2c3d4e5f6a7b8",
              signed_off=True)
    return ReportRecord(**kw)


IDOC = {"kinematics": {"label": "Kinematics", "subsystem": "suspension",
                       "committed_on": "2026-08-05 09:00", "md": "# k"}}


# --- unavailable geometry must not print as zero --------------------------
def test_unavailable_geometry_is_named_not_zeroed():
    md = pj.build_handover_markdown(
        _store(), geometry={"scrub_radius_mm": None, "caster_deg": 5.1})
    assert "not available" in md
    assert "scrub_radius_mm: 0.00" not in md
    assert "caster_deg: 5.10" in md


def test_a_real_zero_still_prints_as_zero():
    """The fix must not make a genuine 0.00 unsayable — static toe is often 0."""
    md = pj.build_handover_markdown(_store(),
                                    geometry={"static_toe_deg": 0.0})
    assert "static_toe_deg: 0.00" in md
    assert "not available" not in md


# --- stamped reports were produced and never read -------------------------
def test_stamped_reports_appear_in_the_handover():
    s = _store()
    s.reports = [_a_report()]
    md = pj.build_handover_markdown(s)
    assert "Signed-off calculation reports" in md
    assert "Bump steer sweep" in md
    assert "a1b2c3d4e5f6" in md          # content hash identifies the inputs


def test_no_reports_means_no_empty_section():
    assert "Signed-off calculation reports" not in \
        pj.build_handover_markdown(_store())


# --- Integration Document coverage ----------------------------------------
def test_integration_document_coverage_is_listed():
    s = _store()
    s.integration_document = IDOC
    md = pj.build_handover_markdown(s)
    assert "Integration Document coverage" in md
    assert "Kinematics" in md and "suspension" in md


def test_no_integration_document_means_no_empty_section():
    assert "Integration Document coverage" not in \
        pj.build_handover_markdown(_store())


# --- nothing else regressed -----------------------------------------------
def test_core_sections_survive():
    s = _store()
    s.add_decision(pj.Decision(team="suspension", title="Rocker ratio 0.52",
                               rationale="Packaging vs wheel rate.",
                               author="ft", tags="kinematics"))
    md = pj.build_handover_markdown(s, frame_tag="x-rear y-right z-up")
    assert "Elbee Racing — Handover Report" in md
    assert "Coordinate convention" in md
    assert "Weight budget" in md
    assert "Rocker ratio 0.52" in md


def test_handover_renders_to_pdf_with_the_new_sections():
    s = _store()
    s.reports = [_a_report()]
    s.integration_document = IDOC
    md = pj.build_handover_markdown(s, geometry={"scrub_radius_mm": None})
    fd, path = tempfile.mkstemp(suffix=".pdf")
    os.close(fd)
    pj.render_pdf(md, path)
    assert os.path.getsize(path) > 0
    with open(path, "rb") as fh:
        assert fh.read(4) == b"%PDF"
    os.unlink(path)
