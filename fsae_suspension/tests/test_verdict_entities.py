"""A warning chip written as HTML must reach the feature report as text, not as "&amp;" (1.7.12)."""
import ast
import html
import re
from pathlib import Path

SRC = (Path(__file__).resolve().parent.parent / "streamlit_app.py").read_text(encoding="utf-8")


def _seg(name):
    tree = ast.parse(SRC)
    node = next(n for n in tree.body if (isinstance(n, ast.FunctionDef) and n.name == name)
                or (isinstance(n, ast.Assign) and any(getattr(t, "id", None) == name for t in n.targets)))
    return ast.get_source_segment(SRC, node)


def test_chip_entities_are_unescaped_in_the_report():
    got = []
    ns = {"re": re, "_html": html, "capture_verdict": lambda t, sev: got.append((t, sev))}
    for name in ("_AX_TAG_SEV", "_AX_TAG_RE", "_AX_HTML_TAG_RE", "_ax_positional_args", "_ax_wrap_markdown"):
        exec(_seg(name), ns)
    md = ns["_ax_wrap_markdown"](lambda *a, **k: None)
    md('<span class="tag warn">enable it in Pushrod &amp; rocker and enter <b>your</b> geometry</span>',
       unsafe_allow_html=True)
    assert got == [("enable it in Pushrod & rocker and enter your geometry", "warning")]
