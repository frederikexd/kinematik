# ============================================================================
#  KinematiK — streamlit_app.py has exactly one home
# ============================================================================
"""There is ONE app file: ``streamlit_app.py`` at the repo root.

It used to have a twin at ``suspension/streamlit_app.py``, kept in sync by
assertions in test_run_log_ui.py and test_brief_coverage.py. Consolidated
Aug 2026: nothing imported the package copy, no console script exposed it, no
doc mentioned it. It existed only to be policed.

This file replaces the old parity guard (test_app_copy_parity.py). The risk has
inverted: it is no longer "the two copies drift apart", it is "someone
reintroduces a second copy". That is not hypothetical here — the tree had
accumulated `streamlit_app (1).py`, three copies of `backends_PATCHED.py`, four
duplicated `ui/` modules and a 96-file clone of the test suite, and two copies
of `ui/omnicore` had already drifted to opposite sides of a real NameError with
nothing indicating which was authoritative.

Cheap to run, and it fails the moment a GUI copy or a "let's keep a package
copy too" lands.
"""
import os

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Names a stray copy tends to arrive under.
_FORBIDDEN = [
    os.path.join("suspension", "streamlit_app.py"),
    "streamlit_app (1).py",
    "streamlit_app copy.py",
    "streamlit_app kopie.py",
    "streamlit_app_old.py",
    "streamlit_app_backup.py",
]


def test_the_live_entry_point_exists():
    """Streamlit Cloud serves the repo-root file. Nothing else is the app."""
    app = os.path.join(_ROOT, "streamlit_app.py")
    assert os.path.exists(app), (
        "streamlit_app.py is missing from the repo root. This is the file "
        "Streamlit Cloud serves — without it there is no deployed app.")
    assert os.path.getsize(app) > 100_000, (
        "streamlit_app.py is suspiciously small. If it was replaced by a shim, "
        "confirm the deployment entry point still resolves to the real app.")


@pytest.mark.parametrize("relpath", _FORBIDDEN)
def test_no_second_copy_of_the_app(relpath):
    assert not os.path.exists(os.path.join(_ROOT, relpath)), (
        f"{relpath} exists — a second copy of the app has reappeared.\n"
        "The app was deliberately consolidated to a single file at the repo "
        "root. Two 1.8 MB files WILL drift, and when they do nothing in the "
        "tree says which one is authoritative. If you genuinely need a second "
        "entry point, make it a thin shim that imports the root file rather "
        "than a copy of it, and update this test to say so.")


def test_no_stray_app_copies_anywhere_in_the_tree():
    """Catches copies under names this file did not think to enumerate."""
    hits = []
    for dirpath, dirnames, filenames in os.walk(_ROOT):
        dirnames[:] = [d for d in dirnames
                       if d not in {"_attic", ".git", "__pycache__",
                                    ".venv", "venv", "node_modules"}]
        for name in filenames:
            if not name.endswith(".py") or "streamlit_app" not in name:
                continue
            full = os.path.join(dirpath, name)
            if os.path.relpath(full, _ROOT) == "streamlit_app.py":
                continue                      # the one true app
            if os.path.getsize(full) > 200_000:   # a copy, not a small helper
                hits.append(os.path.relpath(full, _ROOT))
    assert not hits, (
        "large streamlit_app-like files found outside the root entry point: "
        f"{hits}. See test_no_second_copy_of_the_app for why this matters.")
