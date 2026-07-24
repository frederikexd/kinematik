# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/report.py — stamped calculation / sign-off PDF reports. Turns a
#  finished calculation (its inputs, solved geometry, and provenance-graded
#  outputs) plus a member sign-off into a deterministic, timestamped PDF whose
#  every number carries its EvidenceGrade — the same provenance language the
#  rest of KinematiK already speaks.
# ============================================================================
"""
Stamped calculation reports — the paper trail a design review actually wants.

WHAT THIS DOES
--------------
When a member or lead finishes a calculation and signs off, this produces a
single-page-or-more PDF that records, in order:

  * WHO signed off, WHEN, on WHAT (author, ISO timestamp, calculation title,
    the part/system it concerns) — the same sign-off fields the project's
    ``Decision`` model already carries;
  * the INPUTS they used (the exact parameter values, so the calc is
    reproducible, not just asserted);
  * the SOLVED GEOMETRY / RESULTS;
  * and — the point of the whole thing — a PROVENANCE TAG on every derived
    output, drawn from the ``EvidenceGrade`` vocabulary in proof_engine.py
    (guess / estimate / modelled / measured / verified, each with its ± band),
    so a reader can tell at a glance which numbers are load-bearing measurements
    and which are concept-stage estimates.

WHY IT'S HONEST BY CONSTRUCTION
-------------------------------
The report never upgrades a number's confidence. Each output row prints the
grade the calculation author assigned; an ungraded output is stamped the honest
middle ('estimate'), never silently promoted. A content hash of the inputs +
outputs is stamped in the footer so a report can be checked against the data it
claims to describe — a stamped PDF whose hash doesn't match its numbers is a
tampered or stale report, and that's detectable. Nothing here fabricates a
result: it renders exactly the record it's given.

NO HARD STREAMLIT/DRIVE DEPENDENCY
----------------------------------
This module only needs reportlab (already vendored) + the stdlib. It builds a
PDF to a path and returns that path. Delivery (download button, Google Drive
export) is a separate concern handled by the UI / drive_export module, so this
stays unit-testable headless and can't be broken by an auth problem.
"""

from __future__ import annotations

import hashlib
import json
import datetime as _dt
from dataclasses import dataclass, field, asdict
from typing import Optional

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, HRFlowable)

from .provenance import grade_key, _GRADE_BADGE


# Map each grade to a table cell colour so provenance is visible at a glance,
# mirroring the emoji badges the UI uses (which don't render in base PDF fonts).
_GRADE_COLOR = {
    "guess":    colors.HexColor("#9e9e9e"),
    "estimate": colors.HexColor("#e6a700"),
    "modelled": colors.HexColor("#2f6fdb"),
    "measured": colors.HexColor("#2e9e4b"),
    "verified": colors.HexColor("#1f7a34"),
}


def _hx(color) -> str:
    """reportlab HexColor.hexval() returns '0xrrggbb'; inline <font color> markup
    needs '#rrggbb'. Normalise here so every call site is correct."""
    return "#" + color.hexval()[2:]


# ===================================================================== #
#  1.  THE RECORD  (what a finished, signed-off calculation contains)
# ===================================================================== #
@dataclass
class OutputRow:
    """One derived output with its provenance grade.

    grade : an EvidenceGrade / enum / string; normalised via grade_key so a bad
            value under-claims (→ 'estimate') rather than crashing.
    calibrated : False demotes the shown grade to 'uncalibrated' in the tag —
            an uncalibrated modelled number is a guess with a shape.
    """
    name: str
    value: str                      # pre-formatted display value (with units)
    grade: str = "estimate"
    calibrated: bool = True
    source: str = ""                # short mechanism/source clause

    def grade_k(self) -> str:
        return grade_key(self.grade)

    def tag_text(self) -> str:
        _, tag, band = _GRADE_BADGE[self.grade_k()]
        if not self.calibrated:
            return f"{tag} · uncalibrated"
        return f"{tag} · {band}"


@dataclass
class CalculationRecord:
    """A finished, signed-off calculation ready to be stamped into a PDF."""
    title: str
    author: str
    team: str = ""
    part: str = ""                  # the part/system this concerns
    signed_off: bool = False        # did the author explicitly sign off?
    date: str = ""                  # ISO; auto-stamped if empty
    tool: str = ""                  # which KinematiK tool produced this
    inputs: dict = field(default_factory=dict)     # name -> display value
    outputs: list = field(default_factory=list)    # list[OutputRow]
    notes: str = ""
    app_version: str = ""

    def __post_init__(self):
        if not self.date:
            self.date = _dt.datetime.now().isoformat(timespec="seconds")
        # tolerate outputs passed as dicts (e.g. from JSON)
        self.outputs = [o if isinstance(o, OutputRow) else OutputRow(**o)
                        for o in self.outputs]

    # ---- integrity hash: binds the PDF to the data it claims -------------
    def content_hash(self) -> str:
        """Stable SHA-256 over the inputs + outputs (order-independent for dicts).
        Stamped in the footer so a report can be verified against its numbers."""
        payload = {
            "title": self.title, "author": self.author, "part": self.part,
            "inputs": self.inputs,
            "outputs": [(o.name, o.value, o.grade_k(), o.calibrated)
                        for o in self.outputs],
        }
        blob = json.dumps(payload, sort_keys=True, default=str).encode("utf-8")
        return hashlib.sha256(blob).hexdigest()

    def to_dict(self) -> dict:
        d = asdict(self)
        d["content_hash"] = self.content_hash()
        return d

    @staticmethod
    def from_decision(decision, inputs=None, outputs=None, tool="") -> "CalculationRecord":
        """Build a record from the project's existing ``Decision`` sign-off, so a
        signed decision and its report share one source of truth."""
        return CalculationRecord(
            title=getattr(decision, "title", "Calculation"),
            author=getattr(decision, "author", ""),
            team=getattr(decision, "team", ""),
            part=getattr(decision, "part", ""),
            signed_off=True,
            date=getattr(decision, "date", "") or "",
            tool=tool,
            inputs=inputs or {},
            outputs=outputs or [],
            notes=getattr(decision, "rationale", ""),
        )


# ===================================================================== #
#  2.  THE PDF BUILDER
# ===================================================================== #
def _styles():
    ss = getSampleStyleSheet()
    ss.add(ParagraphStyle("KTitle", parent=ss["Title"], fontSize=18,
                          spaceAfter=2, textColor=colors.HexColor("#1a1a1a")))
    ss.add(ParagraphStyle("KSub", parent=ss["Normal"], fontSize=9,
                          textColor=colors.HexColor("#666666")))
    ss.add(ParagraphStyle("KH2", parent=ss["Heading2"], fontSize=12,
                          spaceBefore=10, spaceAfter=4,
                          textColor=colors.HexColor("#222222")))
    ss.add(ParagraphStyle("KCell", parent=ss["Normal"], fontSize=9, leading=12))
    ss.add(ParagraphStyle("KFoot", parent=ss["Normal"], fontSize=7,
                          textColor=colors.HexColor("#888888")))
    return ss


def _signoff_banner(rec: CalculationRecord, ss) -> Table:
    status = ("SIGNED OFF" if rec.signed_off else "DRAFT — NOT SIGNED OFF")
    status_col = (colors.HexColor("#2e9e4b") if rec.signed_off
                  else colors.HexColor("#c0392b"))
    rows = [
        [Paragraph("<b>Author</b>", ss["KCell"]), Paragraph(rec.author or "—", ss["KCell"]),
         Paragraph("<b>Status</b>", ss["KCell"]),
         Paragraph(f'<font color="{_hx(status_col)}"><b>{status}</b></font>', ss["KCell"])],
        [Paragraph("<b>Team</b>", ss["KCell"]), Paragraph(rec.team or "—", ss["KCell"]),
         Paragraph("<b>Date</b>", ss["KCell"]), Paragraph(rec.date, ss["KCell"])],
        [Paragraph("<b>Part / system</b>", ss["KCell"]), Paragraph(rec.part or "—", ss["KCell"]),
         Paragraph("<b>Tool</b>", ss["KCell"]), Paragraph(rec.tool or "—", ss["KCell"])],
    ]
    t = Table(rows, colWidths=[28*mm, 62*mm, 24*mm, 56*mm])
    t.setStyle(TableStyle([
        ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#cccccc")),
        ("INNERGRID", (0, 0), (-1, -1), 0.25, colors.HexColor("#eeeeee")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#fafafa")),
        ("BACKGROUND", (2, 0), (2, -1), colors.HexColor("#fafafa")),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    return t


def _inputs_table(rec: CalculationRecord, ss) -> Table:
    rows = [[Paragraph("<b>Input</b>", ss["KCell"]),
             Paragraph("<b>Value</b>", ss["KCell"])]]
    for k, v in rec.inputs.items():
        rows.append([Paragraph(str(k), ss["KCell"]), Paragraph(str(v), ss["KCell"])])
    if len(rows) == 1:
        rows.append([Paragraph("<i>no inputs recorded</i>", ss["KCell"]), Paragraph("", ss["KCell"])])
    t = Table(rows, colWidths=[90*mm, 80*mm])
    t.setStyle(TableStyle([
        ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#cccccc")),
        ("INNERGRID", (0, 0), (-1, -1), 0.25, colors.HexColor("#eeeeee")),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#f2f2f2")),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]))
    return t


def _outputs_table(rec: CalculationRecord, ss) -> Table:
    rows = [[Paragraph("<b>Output</b>", ss["KCell"]),
             Paragraph("<b>Value</b>", ss["KCell"]),
             Paragraph("<b>Provenance</b>", ss["KCell"]),
             Paragraph("<b>Source</b>", ss["KCell"])]]
    grade_rows = []
    for o in rec.outputs:
        rows.append([
            Paragraph(o.name, ss["KCell"]),
            Paragraph(str(o.value), ss["KCell"]),
            Paragraph(o.tag_text(), ss["KCell"]),
            Paragraph(o.source or "—", ss["KCell"]),
        ])
        grade_rows.append(o.grade_k())
    if not rec.outputs:
        rows.append([Paragraph("<i>no outputs recorded</i>", ss["KCell"]),
                     Paragraph("", ss["KCell"]), Paragraph("", ss["KCell"]),
                     Paragraph("", ss["KCell"])])
    t = Table(rows, colWidths=[52*mm, 34*mm, 46*mm, 38*mm])
    style = [
        ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#cccccc")),
        ("INNERGRID", (0, 0), (-1, -1), 0.25, colors.HexColor("#eeeeee")),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#f2f2f2")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]
    # colour the provenance cell per grade so it reads at a glance
    for i, gk in enumerate(grade_rows, start=1):
        style.append(("TEXTCOLOR", (2, i), (2, i), _GRADE_COLOR.get(gk, colors.black)))
        style.append(("FONTNAME", (2, i), (2, i), "Helvetica-Bold"))
    t.setStyle(TableStyle(style))
    return t


def _provenance_legend(ss) -> Paragraph:
    parts = []
    for key in ("guess", "estimate", "modelled", "measured", "verified"):
        _, tag, band = _GRADE_BADGE[key]
        col = _hx(_GRADE_COLOR[key])
        parts.append(f'<font color="{col}"><b>{tag}</b></font> {band}')
    return Paragraph("Provenance scale: " + " &nbsp;·&nbsp; ".join(parts),
                     ss["KSub"])


def build_report(rec: CalculationRecord, out_path: str) -> str:
    """Render the record to a stamped PDF at out_path. Returns out_path.

    Deterministic given the same record (the content hash and body are fixed;
    only reportlab's internal producer timestamp varies, which the footer hash
    does not cover). Never raises on empty inputs/outputs — it stamps 'none
    recorded' rather than failing, so a thin calc still gets an honest report.
    """
    ss = _styles()
    story = []
    chash = rec.content_hash()

    # header
    story.append(Paragraph("KinematiK — Calculation Report", ss["KSub"]))
    story.append(Paragraph(rec.title, ss["KTitle"]))
    story.append(HRFlowable(width="100%", thickness=1,
                            color=colors.HexColor("#dddddd"),
                            spaceBefore=2, spaceAfter=8))

    # sign-off banner
    story.append(_signoff_banner(rec, ss))
    story.append(Spacer(1, 8))

    # inputs
    story.append(Paragraph("Inputs", ss["KH2"]))
    story.append(_inputs_table(rec, ss))

    # outputs + provenance
    story.append(Paragraph("Solved geometry &amp; results", ss["KH2"]))
    story.append(_outputs_table(rec, ss))
    story.append(Spacer(1, 4))
    story.append(_provenance_legend(ss))

    # notes / rationale
    if rec.notes:
        story.append(Paragraph("Rationale", ss["KH2"]))
        story.append(Paragraph(rec.notes.replace("\n", "<br/>"), ss["KCell"]))

    # footer stamp — the integrity line
    story.append(Spacer(1, 12))
    story.append(HRFlowable(width="100%", thickness=0.5,
                            color=colors.HexColor("#dddddd"),
                            spaceBefore=2, spaceAfter=4))
    stamp = (f"Stamped {_dt.datetime.now().isoformat(timespec='seconds')} · "
             f"content SHA-256 {chash[:16]}… · "
             f"KinematiK {rec.app_version or 'dev'} · "
             f"provenance grades per proof_engine EvidenceGrade. "
             f"This report renders exactly the recorded calculation; a mismatch "
             f"between the numbers above and this hash indicates a stale or "
             f"altered report.")
    story.append(Paragraph(stamp, ss["KFoot"]))

    def _footer(canvas, doc):
        canvas.saveState()
        canvas.setFont("Helvetica", 7)
        canvas.setFillColor(colors.HexColor("#999999"))
        canvas.drawRightString(200*mm, 10*mm, f"page {doc.page}")
        canvas.drawString(15*mm, 10*mm,
                          f"{rec.title} · {rec.author} · {rec.date}")
        canvas.restoreState()

    doc = SimpleDocTemplate(
        out_path, pagesize=A4,
        leftMargin=15*mm, rightMargin=15*mm,
        topMargin=15*mm, bottomMargin=18*mm,
        title=rec.title, author=rec.author,
        subject=f"KinematiK calculation report · {rec.part}",
        creator="KinematiK")
    doc.build(story, onFirstPage=_footer, onLaterPages=_footer)
    return out_path


def suggested_filename(rec: CalculationRecord) -> str:
    """A tidy, collision-resistant filename for the report."""
    def slug(s):
        return "".join(c if c.isalnum() else "-" for c in s.lower()).strip("-")[:40]
    date = (rec.date or "").split("T")[0] or _dt.date.today().isoformat()
    base = "_".join(x for x in [date, slug(rec.part or rec.title),
                                slug(rec.author)] if x)
    return f"{base or 'kinematik-report'}.pdf"
