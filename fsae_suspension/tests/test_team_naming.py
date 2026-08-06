# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""No team's name may be baked into the tool.

Report headers used to read `# Elbee Racing — …` as a literal string in four
builders, and every export was named `elbee_*.pdf`. That is fine for the team
that wrote it and a blocker for every other: another team's design-review PDF
arrives headed with someone else's name, so the tool cannot be handed on
without its author in the loop.

Functions are parsed out of streamlit_app.py rather than imported, since that
module is a Streamlit entrypoint that runs the whole app on import.
"""
import ast
import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

_FN = ("_report_org_name", "_report_title", "_report_slug")
_C = ("_ORG_NAME_FALLBACK", "_LEGACY_TEAM_NAME")

#: The value the app itself treats as "unset"; it must still appear once, in
#: the constant that defines it, or old project files stop being recognised.
_LEGACY = "Elbee Racing"


@pytest.fixture(scope="module")
def src():
    return open(os.path.join(ROOT, "streamlit_app.py"), encoding="utf-8").read()


@pytest.fixture(scope="module")
def chunks(src):
    out = []
    for n in ast.parse(src).body:
        if isinstance(n, ast.FunctionDef) and n.name in _FN:
            out.append(ast.get_source_segment(src, n))
        elif isinstance(n, ast.Assign):
            for t in n.targets:
                if isinstance(t, ast.Name) and t.id in _C:
                    out.append(ast.get_source_segment(src, n))
    return out


def build(chunks, workspace=None, team=""):
    class _WS:
        def __init__(self, n):
            self.name = n

    class _Ctx:
        def __init__(self, n):
            self.workspace = _WS(n)

    class _Store:
        def __init__(self, t):
            self.team_name = t

    ns = {"re": re,
          "_active_workspace_ctx": lambda: (_Ctx(workspace)
                                            if workspace is not None else None),
          "get_store": lambda: _Store(team)}
    exec("\n\n".join(chunks), ns)          # noqa: S102 - see module docstring
    return ns


# --- the ask: workspace name heads the document ---------------------------
def test_workspace_name_is_used(chunks):
    ns = build(chunks, workspace="Long Beach Racing")
    assert ns["_report_org_name"]() == "Long Beach Racing"
    assert ns["_report_title"]("Kinematics Feature Report") == \
        "# Long Beach Racing — Kinematics Feature Report"


def test_a_new_team_needs_no_configuration(chunks):
    """Sign up, open a tab, export — the header is already right."""
    ns = build(chunks, workspace="Rose-Hulman Motorsports", team="")
    assert "Rose-Hulman Motorsports" in ns["_report_title"]("X")


# --- precedence -----------------------------------------------------------
def test_an_explicit_team_name_beats_the_workspace(chunks):
    """The Team box is SEEDED from the workspace, so if the workspace won,
    editing that box would silently do nothing and look broken."""
    ns = build(chunks, workspace="CSULB", team="Long Beach Racing")
    assert ns["_report_org_name"]() == "Long Beach Racing"


def test_the_legacy_baked_in_name_counts_as_unset(chunks):
    """An existing project.json from before this change must not keep stamping
    one team's name onto another team's documents."""
    ns = build(chunks, workspace="Bulldog Motorsports", team="Elbee Racing")
    assert ns["_report_org_name"]() == "Bulldog Motorsports"


def test_legacy_name_with_no_workspace_yields_nothing(chunks):
    ns = build(chunks, workspace=None, team="Elbee Racing")
    assert ns["_report_org_name"]() == ""


# --- fallbacks ------------------------------------------------------------
def test_unnamed_reports_omit_the_prefix_rather_than_inventing_one(chunks):
    ns = build(chunks, workspace=None, team="")
    assert ns["_report_org_name"]() == ""
    assert ns["_report_title"]("Kinematics Feature Report") == \
        "# Kinematics Feature Report"


def test_plumbing_workspaces_are_not_organisations(chunks):
    for name in ("Personal", "Sandbox", "default", "My Workspace", "Untitled"):
        ns = build(chunks, workspace=name)
        assert ns["_report_org_name"]() == "", name


def test_whitespace_is_normalised(chunks):
    ns = build(chunks, workspace="  Cal   Poly  ")
    assert ns["_report_org_name"]() == "Cal Poly"


def test_resolution_never_raises(chunks):
    """A report must not fail because a name could not be resolved."""
    ns = {"re": re,
          "_active_workspace_ctx": lambda: (_ for _ in ()).throw(RuntimeError),
          "get_store": lambda: (_ for _ in ()).throw(RuntimeError)}
    exec("\n\n".join(chunks), ns)          # noqa: S102
    assert ns["_report_org_name"]() == ""
    assert ns["_report_slug"]() == "kinematik"


# --- filenames ------------------------------------------------------------
def test_export_filenames_follow_the_team(chunks):
    """Downloads were named elbee_*.pdf whoever exported them, so a judge's
    folder collected a dozen files under one team's name."""
    ns = build(chunks, workspace="Long Beach Racing")
    assert ns["_report_slug"]() == "long_beach_racing"


def test_slug_is_filesystem_safe(chunks):
    for name in ("Rose-Hulman Motorsports", "École Polytechnique",
                 "Team //bad\\name??", "A" * 100):
        slug = build(chunks, workspace=name)["_report_slug"]()
        assert re.fullmatch(r"[a-z0-9_]+", slug), (name, slug)
        assert len(slug) <= 40


def test_slug_falls_back_to_the_product_not_a_team(chunks):
    assert build(chunks, workspace=None)["_report_slug"]() == "kinematik"


# --- nothing left hardcoded ------------------------------------------------
def _output_strings(src):
    """Every string literal that could reach a user, excluding docstrings.

    A raw text scan flags the comments and docstrings that EXPLAIN the old
    hardcoding, which is the opposite of useful — it would force the history to
    be deleted to make the test pass. Walking the AST checks only literals that
    can actually be rendered.
    """
    tree = ast.parse(src)
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.ClassDef)):
            body = getattr(node, "body", None)
            if body and isinstance(body[0], ast.Expr) and \
                    isinstance(body[0].value, ast.Constant) and \
                    isinstance(body[0].value.value, str):
                docstrings.add(id(body[0].value))
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) \
                and id(node) not in docstrings:
            out.append(node.value)
    return out


def test_no_team_name_is_baked_into_the_live_app(src):
    hits = [s for s in _output_strings(src)
            if "elbee" in s.lower() and s != _LEGACY]
    assert not hits, f"a team name is still hardcoded: {hits[:5]}"


def test_no_team_name_in_exported_filenames(src):
    hits = [s for s in _output_strings(src)
            if s.lower().endswith((".pdf", ".md", ".json"))
            and "elbee" in s.lower()]
    assert not hits, f"export filenames still carry a team name: {hits[:5]}"


def test_no_team_name_is_baked_into_the_live_modules():
    """The database ROW KEY is exempt.

    SupabaseBackend.LEGACY_ROW_KEY is the primary key of rows already written in
    deployed databases. It is never displayed and is not a team name. Changing
    it would point an existing deployment at a different, empty row — which
    looks exactly like total data loss to the team it happens to. Renaming it
    is a migration, not an edit.
    """
    import glob
    from suspension.project import SupabaseBackend
    exempt = {SupabaseBackend.LEGACY_ROW_KEY}
    bad = []
    for path in (glob.glob(os.path.join(ROOT, "suspension", "*.py"))
                 + glob.glob(os.path.join(ROOT, "ui", "*.py"))):
        base = os.path.basename(path)
        if base.startswith("test_") or base == "streamlit_app.py":
            continue
        src_m = open(path, encoding="utf-8").read()
        hits = [s for s in _output_strings(src_m)
                if "elbee" in s.lower() and s not in exempt]
        if hits:
            bad.append((base, hits[:3]))
    assert not bad, f"hardcoded team name in: {bad}"


def test_project_store_has_no_default_team_name():
    from suspension import project as pj
    import tempfile
    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    os.unlink(path)
    assert pj.ProjectStore(path).team_name == ""


def test_handover_header_tolerates_an_unset_team():
    from suspension import project as pj
    import tempfile
    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    os.unlink(path)
    md = pj.build_handover_markdown(pj.ProjectStore(path))
    assert md.splitlines()[0] == "# Handover Report"
