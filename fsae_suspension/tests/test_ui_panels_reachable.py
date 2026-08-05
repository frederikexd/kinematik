# ============================================================================
#  KinematiK — every ui/ panel is either reachable or declared unreachable
# ============================================================================
"""A rendered panel nobody can open is not a feature.

`ui/__init__.py` says `render()` is the only public surface and the shell
imports each module lazily. Nothing checked that the shell actually *does*.
Four finished panels — ~36 KB of working UI, each with a complete `render()` —
were sitting in `ui/` unreachable from the app. Their duplicates
(`ui_arch_synth.py`, `ui_degradation.py`, `ui_worthwhile.py`, `ui_report.py`)
had already been quarantined; the originals were never wired in.

This file does for the UI surface what `KNOWN_MISSING_SUBMODULES` does for the
public API: it does not permit the gap, it makes the gap *declared*. A new
orphan fails the suite. A resolved orphan also fails it, forcing the registry
to shrink. Neither can rot quietly.

Wiring these in is a product decision — where each belongs in which tab menu —
so it is deliberately not guessed at here.
"""
import pathlib
import re

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[1]
_UI = _ROOT / "ui"
_APP = _ROOT / "streamlit_app.py"


#: Panels that exist, work, and are not reachable from the app. Each needs a
#: home in a tab's feature_menu. Remove the entry in the same commit that
#: wires it in — test_no_stale_unwired_entries below will insist.
#  Emptied Aug 2026: all four were wired into the Integration tab's
#  feature_menu in the same pass that discovered them. Each answers a whole-car
#  question rather than a single-subsystem one, which is what made Integration
#  the right home. Keep this empty — an entry here is a declared gap, not a
#  parking space.
UNWIRED: dict[str, str] = {}


def _modules():
    return {p.stem for p in _UI.glob("*.py") if p.stem != "__init__"}


def _wired():
    src = _APP.read_text(encoding="utf-8")
    return (set(re.findall(r"from ui import (\w+)", src))
            | set(re.findall(r"from ui\.(\w+) import", src)))


def test_no_new_unreachable_panels():
    """A panel added to ui/ but never wired in is a feature nobody can open."""
    orphans = _modules() - _wired() - set(UNWIRED)
    assert not orphans, (
        f"ui/ modules the shell cannot reach: {sorted(orphans)}\n"
        "Either wire the panel into a tab's feature_menu in streamlit_app.py, "
        "or add it to UNWIRED above with a note saying what it is. Silently "
        "shipping an unreachable panel is how four of them accumulated.")


def test_no_stale_unwired_entries():
    """When a panel gets wired in, this fails until the registry shrinks."""
    resolved = sorted(set(UNWIRED) & _wired())
    assert not resolved, (
        f"these are wired in now — remove them from UNWIRED: {resolved}")

    gone = sorted(set(UNWIRED) - _modules())
    assert not gone, (
        f"these no longer exist in ui/ — remove them from UNWIRED: {gone}")


@pytest.mark.parametrize("name", sorted(_modules()))
def test_every_panel_honours_the_ui_contract(name):
    """`ui/__init__.py`'s contract, enforced on EVERY panel.

    Was parametrised over UNWIRED only, which meant it evaporated the moment
    that registry emptied — the four panels would have lost their only check
    at exactly the point they became reachable. Covering all of ui/ instead
    found a fifth violation in `daq_plan`, which had been wired in all along.

    NOTE on the headless check: an earlier version asserted
    `"streamlit" not in mod.__dict__`. That passes trivially for the
    `try: import streamlit as st / except: st = None` pattern these modules
    used — the binding is `st`, not `streamlit`. It was checking the wrong
    name and could never fail. It now asks the question that matters: does
    importing this module drag streamlit in, in a FRESH interpreter.
    """
    import importlib
    import subprocess
    import sys

    mod = importlib.import_module(f"ui.{name}")
    assert callable(getattr(mod, "render", None)), (
        f"ui/{name}.py must expose a callable render() per ui/__init__.py")

    probe = (f"import sys, ui.{name}; "
             f"assert 'streamlit' not in sys.modules, "
             f"'ui/{name}.py pulls streamlit in at import'")
    proc = subprocess.run([sys.executable, "-c", probe], capture_output=True,
                          text=True, cwd=str(_ROOT))
    assert proc.returncode == 0, proc.stderr.strip()


def test_every_wired_panel_actually_exists():
    """The mirror image: the shell importing a module that is not there."""
    missing = sorted(_wired() - _modules())
    assert not missing, (
        f"streamlit_app.py imports ui modules that do not exist: {missing}")
