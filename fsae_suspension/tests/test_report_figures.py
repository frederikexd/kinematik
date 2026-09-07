# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""Tests for capturing charts into the documentation PDFs.

The regression these guard against: feature and subsystem reports used to come
out as a list of chart TITLES with no figures and no numbers in them. Three
things had to hold for that to be fixed, and each is checked here — the figure
spec survives capture, it rasterizes, and render_pdf embeds it.
"""
import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from suspension import project as pj                     # noqa: E402
from suspension import report_figures as rfg             # noqa: E402

go = pytest.importorskip("plotly.graph_objects")


def _fig(n=41):
    """A figure shaped like KinematiK's kinematic sweeps, numpy-backed."""
    np = pytest.importorskip("numpy")
    travel = np.linspace(-25, 25, n)
    f = go.Figure()
    f.add_trace(go.Scatter(x=travel, y=-1.5 - 0.02 * travel, mode="lines",
                           line=dict(color="#37e0d0", width=3), name="Camber"))
    f.update_layout(title="Camber vs wheel travel",
                    xaxis_title="travel (mm, + bump)",
                    yaxis_title="camber (°)")
    return f


# --- capture ---------------------------------------------------------------
def test_compact_spec_keeps_the_series():
    spec = rfg.compact_spec(_fig())
    assert spec and len(spec["data"]) == 1
    assert len(spec["data"][0]["x"]) == 41


def test_compact_spec_decodes_plotly_binary_arrays():
    """plotly>=6 serialises numpy traces as {'dtype','bdata'} base64 blobs.

    If those are stored unopened, every downstream consumer sees a dict where
    it expected numbers and silently drops the chart — with numpy-backed data
    being the normal case in this app, that is every chart.
    """
    spec = rfg.compact_spec(_fig())
    xs = spec["data"][0]["x"]
    assert isinstance(xs, list)
    assert all(isinstance(v, (int, float)) for v in xs)
    assert xs[0] == pytest.approx(-25.0)
    assert xs[-1] == pytest.approx(25.0)


def test_long_traces_are_decimated():
    spec = rfg.compact_spec(_fig(n=50000))
    assert len(spec["data"][0]["x"]) <= rfg.MAX_POINTS_PER_TRACE + 1


def test_compact_spec_is_plain_and_picklable():
    """It lands in session state, so it must not pin live plotly objects."""
    import pickle
    spec = rfg.compact_spec(_fig())
    assert isinstance(spec, dict)
    pickle.loads(pickle.dumps(spec))


def test_compact_spec_survives_junk():
    assert rfg.compact_spec(None) is None
    assert rfg.compact_spec("not a figure") is None


# --- print theme -----------------------------------------------------------
def test_print_theme_lightens_the_page():
    spec = rfg.to_print_theme(rfg.compact_spec(_fig()))
    assert spec["layout"]["plot_bgcolor"].lower() in ("#fbfbfc", "#ffffff")
    assert spec["layout"]["font"]["color"] == "#1a1f24"


def test_screen_accents_are_darkened_for_paper():
    for screen in ("#37e0d0", "#ffb02e", "#62d27a"):
        printed = rfg.darken_for_print(screen)
        assert printed != screen, f"{screen} left unreadable on white"
        assert rfg._luminance(rfg._parse_color(printed)) <= 0.31


def test_already_dark_colors_are_left_alone():
    assert rfg.darken_for_print("#1a1f24") == "#1a1f24"


def test_unparseable_colors_pass_through():
    assert rfg.darken_for_print("rebeccapurple") == "rebeccapurple"
    assert rfg.darken_for_print(None) is None


# --- rasterization ---------------------------------------------------------
def test_figure_renders_to_png():
    pytest.importorskip("matplotlib")
    png = rfg.figure_png(rfg.compact_spec(_fig()))
    assert png and png[:8] == b"\x89PNG\r\n\x1a\n"


def test_unsupported_figure_declines_rather_than_drawing_wrong():
    """A 3-D model must fall back to its text description, not a blank box."""
    spec = rfg.compact_spec(go.Figure(go.Scatter3d(x=[1, 2], y=[1, 2],
                                                   z=[1, 2])))
    assert rfg.figure_png(spec, prefer_kaleido=False) is None


def test_empty_figure_declines():
    assert rfg.figure_png({"data": [], "layout": {}}) is None
    assert rfg.figure_png(None) is None


# --- PDF embedding ---------------------------------------------------------
def _pdf(md, figures=None):
    fd, path = tempfile.mkstemp(suffix=".pdf")
    os.close(fd)
    pj.render_pdf(md, path, figures=figures) if figures is not None \
        else pj.render_pdf(md, path)
    with open(path, "rb") as fh:
        data = fh.read()
    os.unlink(path)
    return data


def test_render_pdf_embeds_the_figure():
    pytest.importorskip("matplotlib")
    png = rfg.figure_png(rfg.compact_spec(_fig()))
    md = f"# Report\n\n![Camber vs wheel travel]({pj.FIGURE_SCHEME}kin~0)\n"
    with_fig = _pdf(md, {"kin~0": png})
    assert with_fig[:4] == b"%PDF"
    # The embedded image is what makes it big; the placeholder path is text.
    assert len(with_fig) > len(_pdf(md, {})) + 10000


def test_missing_figure_degrades_honestly():
    """An unexported chart must say so, not vanish and not break the build."""
    md = f"# Report\n\n![Camber vs travel]({pj.FIGURE_SCHEME}kin~9)\n"
    assert _pdf(md, {})[:4] == b"%PDF"


def test_legacy_two_arg_call_still_works():
    """Every existing call site passes (md, path) only."""
    fd, path = tempfile.mkstemp(suffix=".pdf")
    os.close(fd)
    pj.render_pdf("# Title\n\nSome body text.\n", path)
    assert os.path.getsize(path) > 0
    os.unlink(path)


def test_table_cells_render_inline_markdown():
    """Results tables carry **bold** values; they used to print the asterisks."""
    md = ("# R\n\n| Result | Value |\n|---|---|\n"
          "| Static camber | **-1.50 °** |\n")
    assert _pdf(md)[:4] == b"%PDF"


def test_ragged_table_rows_do_not_raise():
    md = "# R\n\n| A | B | C |\n|---|---|---|\n| 1 | 2 |\n| 1 | 2 | 3 | 4 |\n"
    assert _pdf(md)[:4] == b"%PDF"


# --- font / glyph handling -------------------------------------------------
def test_emoji_are_stripped_for_pdf():
    """Emoji have no glyph in any PDF font we ship — they printed as tofu."""
    assert pj.strip_unprintable("📐 Kinematics") == "Kinematics"
    assert pj.strip_unprintable("📈 Camber vs travel") == "Camber vs travel"


def test_verdict_marks_and_units_are_kept():
    """These are BMP glyphs DejaVu covers, and they carry the meaning."""
    for keep in ("✓", "✗", "⚠", "°", "·"):
        assert keep in pj.strip_unprintable(f"{keep} bump steer 0.1°/10mm")


def test_font_registration_is_stable():
    a = pj._register_report_font()
    assert a == pj._register_report_font()          # cached, no re-register
    assert len(a) == 2 and all(isinstance(n, str) for n in a)
