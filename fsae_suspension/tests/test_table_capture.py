# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""Reports must carry a table's VALUES, not a description of its shape.

The regression: `_ax_table_summary` produced "40 rows x 7 cols · Message, ID,
DLC, …" and that string was the whole of what reached the PDF. For a
table-heavy feature like Data Acquisition — whose entire output is tables and
which draws no charts at all — the report told a design-review reader that a
CAN message breakdown existed and nothing about what it said.

Parses the functions out of streamlit_app.py rather than importing it, since
that module is a Streamlit entrypoint that runs the whole app on import.
"""
import ast
import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

_WANT_FN = ("_ax_cell", "_ax_table_rows", "_ax_plural", "_ax_table_summary")
_WANT_C = ("_MAX_TABLE_ROWS", "_MAX_TABLE_COLS")


@pytest.fixture(scope="module")
def mod():
    src = open(os.path.join(ROOT, "streamlit_app.py"), encoding="utf-8").read()
    chunks = []
    for n in ast.parse(src).body:
        if isinstance(n, ast.FunctionDef) and n.name in _WANT_FN:
            chunks.append(ast.get_source_segment(src, n))
        elif isinstance(n, ast.Assign):
            for t in n.targets:
                if isinstance(t, ast.Name) and t.id in _WANT_C:
                    chunks.append(ast.get_source_segment(src, n))
    ns = {"re": re}
    exec("\n\n".join(chunks), ns)          # noqa: S102 - deliberate, see docstring
    return ns


# --- the shapes the app actually passes to st.dataframe / st.table ---------
def test_dataframe_values_are_captured(mod):
    pd = pytest.importorskip("pandas")
    df = pd.DataFrame({"Message": ["motor_temp", "ts_current"],
                       "Rate (Hz)": [10.0, 500.0]})
    t = mod["_ax_table_rows"](df)
    assert t["header"] == ["Message", "Rate (Hz)"]
    assert t["rows"][0][0] == "motor_temp"
    assert t["rows"][1][1] == "500"


def test_list_of_dicts(mod):
    t = mod["_ax_table_rows"]([{"a": 1, "b": 2}, {"a": 3, "c": 4}])
    assert t["header"] == ["a", "b", "c"]        # union, in first-seen order
    assert t["rows"][1] == ["3", "", "4"]


def test_dict_of_lists(mod):
    t = mod["_ax_table_rows"]({"x": [1, 2], "y": [3, 4]})
    assert t["header"] == ["x", "y"]
    assert t["rows"] == [["1", "3"], ["2", "4"]]


def test_list_of_lists_and_flat_list(mod):
    assert mod["_ax_table_rows"]([[1, 2], [3, 4]])["header"] == ["col 1", "col 2"]
    assert mod["_ax_table_rows"]([1, 2, 3])["rows"] == [["1"], ["2"], ["3"]]


def test_unusable_input_returns_none_rather_than_raising(mod):
    for junk in (None, [], "not a table", 42):
        assert mod["_ax_table_rows"](junk) is None


# --- bounded, and honest about it -----------------------------------------
def test_long_table_is_truncated_and_says_so(mod):
    pd = pytest.importorskip("pandas")
    t = mod["_ax_table_rows"](pd.DataFrame({"a": range(300)}))
    assert len(t["rows"]) == mod["_MAX_TABLE_ROWS"]
    assert t["truncated"] is True
    assert t["total_rows"] == 300


def test_short_table_is_not_marked_truncated(mod):
    t = mod["_ax_table_rows"]([{"a": 1}, {"a": 2}])
    assert t["truncated"] is False


def test_wide_table_is_column_capped(mod):
    pd = pytest.importorskip("pandas")
    t = mod["_ax_table_rows"](pd.DataFrame({f"c{i}": [1] for i in range(30)}))
    assert len(t["header"]) == mod["_MAX_TABLE_COLS"]
    assert all(len(r) == mod["_MAX_TABLE_COLS"] for r in t["rows"])


# --- cells must survive a Markdown table ----------------------------------
def test_pipes_and_newlines_cannot_break_the_row(mod):
    """A stray pipe would split the cell and shift every column after it."""
    assert "|" not in mod["_ax_cell"]("needs barrier|now")
    assert "\n" not in mod["_ax_cell"]("line one\nline two")


def test_missing_values_render_blank_not_nan(mod):
    np = pytest.importorskip("numpy")
    assert mod["_ax_cell"](None) == ""
    assert mod["_ax_cell"](float("nan")) == ""
    assert mod["_ax_cell"](np.nan) == ""


def test_booleans_are_words_not_python_repr(mod):
    assert mod["_ax_cell"](True) == "yes"
    assert mod["_ax_cell"](False) == "no"


def test_floats_are_readable(mod):
    """Round engineering numbers must not become scientific notation.

    The first version used ",.4g", which rendered a 115200 baud rate as
    "1.152e+05" and a 500 kbit/s bus as "5e+05" in the middle of a CAN table.
    """
    c = mod["_ax_cell"]
    assert c(0.0015) == "0.0015"
    assert c(0.075) == "0.075"
    assert c(1234.5678) == "1,234.57"
    assert c(115200.0) == "115,200"
    assert c(500000.0) == "500,000"
    assert c(96.0) == "96"


def test_scientific_only_where_it_helps(mod):
    c = mod["_ax_cell"]
    assert "e" in c(0.0000001).lower()
    assert "e" in c(9.9e15).lower()


def test_non_finite_floats_do_not_leak_python_repr(mod):
    c = mod["_ax_cell"]
    assert c(float("inf")) == "inf"
    assert c(float("-inf")) == "-inf"


def test_very_long_cell_is_clipped(mod):
    out = mod["_ax_cell"]("x" * 500)
    assert len(out) <= 60 and out.endswith("…")


# --- build stamp ----------------------------------------------------------
def test_report_header_carries_a_build_stamp():
    """A report that LOOKS stale and a deployment that IS stale are
    indistinguishable from the PDF alone. A DAQ export arrived showing the old
    table description after the fix had shipped, and nothing in the document
    said which code produced it."""
    src = open(os.path.join(ROOT, "streamlit_app.py"), encoding="utf-8").read()
    assert "_build_stamp()" in src
    i = src.index('_report_title(f"{_lbl} Feature Report")')
    assert "_build_stamp()" in src[i:i + 400], \
        "the feature report header is not stamped"


def test_build_stamp_is_content_derived_not_a_constant():
    """A version constant is something you have to remember to bump; the whole
    point is to survive someone forgetting."""
    src = open(os.path.join(ROOT, "streamlit_app.py"), encoding="utf-8").read()
    fn = src[src.index("def _build_stamp("):]
    fn = fn[:fn.index("\ndef ", 1)]
    assert "hashlib" in fn and "sha256" in fn


# --- PDF table layout ------------------------------------------------------
def _render(md):
    import tempfile
    from suspension import project as pj
    fd, path = tempfile.mkstemp(suffix=".pdf")
    os.close(fd)
    pj.render_pdf(md, path)
    size = os.path.getsize(path)
    with open(path, "rb") as fh:
        magic = fh.read(4)
    os.unlink(path)
    return magic, size


def test_wide_table_renders():
    """7 columns is the DAQ CAN breakdown; 12 is the column cap."""
    for n in (2, 7, 12):
        hdr = "| " + " | ".join(f"c{i}" for i in range(n)) + " |"
        row = "| " + " | ".join(str(i) for i in range(n)) + " |"
        magic, size = _render(f"# T\n\n{hdr}\n|{'---|' * n}\n{row}\n")
        assert magic == b"%PDF" and size > 0, n


def test_long_prose_cell_does_not_starve_other_columns():
    """One paragraph-length finding must not squeeze the ID column to nothing;
    the per-column width is capped so a single long cell cannot take the frame."""
    long = "Sampling at 50 Hz a signal with content to 30 Hz folds down. " * 4
    md = ("# T\n\n| Channel | Finding | Owner |\n|---|---|---|\n"
          f"| damper_pot_fl | {long} | M. Haddad |\n")
    magic, _ = _render(md)
    assert magic == b"%PDF"


def test_forty_row_table_renders():
    rows = "\n".join(f"| BMS_{i} | 0x3{i:02x} | 8 | 10 |" for i in range(40))
    md = "# T\n\n| Message | ID | DLC | Rate |\n|---|---|---|---|\n" + rows
    magic, _ = _render(md)
    assert magic == b"%PDF"


def test_column_widths_account_for_cell_padding():
    """Reportlab's horizontal padding is a fixed cost per column, so it eats a
    trivial slice of a wide column and most of a narrow one. Distributing the
    whole frame proportionally is why a 39 pt 'ID' column still wrapped
    '0x400' onto two lines."""
    src = open(os.path.join(ROOT, "suspension", "project.py"),
               encoding="utf-8").read()
    fn = src[src.index("def render_pdf("):]
    assert "overhead = 2 * _PAD * ncols" in fn
    assert '("LEFTPADDING"' in fn and '("RIGHTPADDING"' in fn


def test_complete_table_has_no_redundant_shape_caption():
    """The captured title is the shape line. Printing it under the actual table
    restates the headers the reader is looking at."""
    src = open(os.path.join(ROOT, "streamlit_app.py"), encoding="utf-8").read()
    i = src.index('_lines.append("| " + " | ".join(_tbl["header"]) + " |")')
    block = src[i:i + 1400]
    assert 'Showing the first' in block          # truncated case still speaks
    assert '_lines.append(f"_{_title}_")' not in block
