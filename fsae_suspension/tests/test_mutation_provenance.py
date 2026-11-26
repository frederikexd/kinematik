"""Characterization suite for suspension/provenance.py (mutation gate); see tests/_golden.py. Every grade spelling
(strings in any case and padding, enums, objects with a value, garbage and an object whose str() raises) through
the tag, the note, the graded number and the report line, calibrated or not, with and without extras, limits and
upgrade paths, into a recording container, no container and one that raises."""
import enum

from suspension import provenance as P
from tests._golden_lazy import check_lazy


class G(enum.Enum):
    MEASURED = "measured"
    VERIFIED = "Verified "


class Boom:
    def __str__(self):
        raise RuntimeError("no str")


class Box:
    def __init__(self):
        self.out = []

    def caption(self, s):
        self.out.append(["caption", s])

    def markdown(self, s):
        self.out.append(["markdown", s])


class Broken:
    def __init__(self):
        self.out = []

    def caption(self, s):
        self.out.append(["caption", s])
        raise RuntimeError("render failed")

    markdown = caption


GRADES = ["guess", "estimate", "modelled", "measured", "verified", " GUESS ", "Modelled", "bogus", "", None, 3,
          G.MEASURED, G.VERIFIED, Boom()]


def _note(fn, container, *a, **kw):
    fn(container, *a, **kw)
    return container.out


def _cases():
    yield "keys", [P.grade_key(g) for g in GRADES]
    for i, g in enumerate(GRADES):
        yield f"tag.{i}", [P.provenance_tag(g), P.provenance_tag(g, calibrated=False),
                           P.provenance_tag(g, extra="closed form"), P.provenance_tag(g, calibrated=False, extra="x")]
        yield f"note.{i}", [_note(P.confidence_note, Box(), g), _note(P.confidence_note, Box(), g, calibrate_with="a sweep"),
                            _note(P.confidence_note, Box(), g, calibrated=False, calibrate_with="a pass", extra="e"),
                            _note(P.confidence_note, Box(), g, calibrated=False),
                            _note(P.confidence_note, Broken(), g, calibrate_with="b")]
        yield f"graded.{i}", [P.graded(12.3456789, g), P.graded(12.3456789, g, "mm", digits=2),
                              P.graded("n/a", g, "N", calibrated=False, limited_by="cg"), P.graded(None, g),
                              P.graded(1e-9, g, limited_by="x", calibrated=True)]
        yield f"report.{i}", [_note(P.render_report_value, Box(), "Lap", 61.2, g, "s"),
                              _note(P.render_report_value, Box(), "Lap", 61.2, g, "s", calibrate_with="a log"),
                              _note(P.render_report_value, Box(), "F", "x", g, calibrated=False, limited_by="m",
                                    calibrate_with="a test"),
                              _note(P.render_report_value, Broken(), "F", 1, g, calibrate_with="t")]
    yield "none_container", [P.confidence_note(None, "guess", calibrate_with="x"),
                             P.render_report_value(None, "a", 1, "guess", calibrate_with="x")]
    yield "worst", [P.worst_grade(), P.worst_grade("verified"), P.worst_grade("measured", "guess", "verified"),
                    P.worst_grade(G.VERIFIED, "modelled"), P.worst_grade("bogus", "verified"),
                    P.worst_grade("measured", "verified"), P.worst_grade("estimate", "modelled"),
                    P.worst_grade("estimate", "guess"), P.worst_grade("verified", "modelled")]
    yield "tables", [P._GRADE_ORDER, {k: list(v) for k, v in P._GRADE_BADGE.items()}]


def test_every_grade_rendered():
    check_lazy("provenance", _cases())
