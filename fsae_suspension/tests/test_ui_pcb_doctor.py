# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
UI coverage for the PCB Doctor panel — the layer every other test skips.

`test_pcb_doctor.py` proves the physics and the parsers. Nothing proved that
the Streamlit panel wired to them actually runs, and that gap is not academic:
the panel is where the session-state juggling, the caches, the format branches
and the error paths live, and a mistake there is invisible to an API-level test
while being the only thing a member ever sees.

This runs the *real* app through Streamlit's `AppTest` harness and inspects
what it rendered. It is not a substitute for a human looking at the screen —
`AppTest` sees the element tree, not pixels, so layout and legibility are still
eyeballs-only. What it does catch is the whole class of "that code path was
never executed once": an exception on load, a caption that never renders, an
error branch that takes half the panel down with it.

Two mechanics worth knowing before editing this file:

  * **One scenario per process.** Several `AppTest` instances in the same
    interpreter interfere — the second one renders an empty tree. That looked
    exactly like a real regression when it first appeared, so every case is
    spawned as a subprocess rather than looped in-process.
  * **The landing gate is bypassed** by seeding `kk_entered` / `kk_show_all`,
    and boards are seeded straight into session state rather than clicked in.
    `AppTest` cannot drive a second rerun on this app, so click *handlers* are
    still verified by construction; everything downstream of them is verified
    here.

Skipped wholesale when Streamlit (or the app's optional heavy dependencies)
are not installed, so a headless analytics box does not fail on it.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import textwrap
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _deps_available() -> bool:
    for mod in ("streamlit", "plotly", "trimesh"):
        try:
            __import__(mod)
        except Exception:                                    # noqa: BLE001
            return False
    return os.path.exists(os.path.join(_ROOT, "streamlit_app.py"))


_HARNESS = '''
import sys, json
sys.path.insert(0, {root!r})
import importlib
from streamlit.testing.v1 import AppTest

_APP = {root!r} + "/_ui_probe_app.py"
with open(_APP, "w") as fh:
    fh.write("import sys\\nsys.path.insert(0, {root!r})\\n"
             "import importlib\\nimportlib.import_module('streamlit_app')\\n")

from suspension import pcb_doctor as pdr
from suspension import pcb_altium as alt

case = {case!r}
seed = {{}}
if case == "kicad":
    seed = {{"pdr_text": pdr.demo_kicad_pcb(), "pdr_name": "demo.kicad_pcb"}}
elif case == "ascii":
    seed = {{"pdr_text": alt.demo_altium_pcb(), "pdr_name": "demo.PcbDoc"}}
elif case == "unreadable":
    seed = {{"pdr_text": "not a board at all", "pdr_name": "notes.txt"}}
elif case == "empty":
    seed = {{}}

at = AppTest.from_file(_APP, default_timeout=1800)
at.session_state["kk_entered"] = True
at.session_state["kk_show_all"] = True
for k, v in seed.items():
    at.session_state[k] = v
at.run()

caps = [c.value for c in at.caption]
exps = [e.label for e in at.expander]
errs = [e.value for e in at.error]


def has(xs, *needles):
    return any(all(n in x for n in needles) for x in xs)


print("@@" + json.dumps({{
    "exception": at.exception[0].message[:300] if at.exception else None,
    "caption": next((c for c in caps if "read as" in c), None),
    "diagnosis": has(exps, "Diagnosis"),
    "viewer": has(exps, "viewer"),
    "retrace": has(exps, "Re-trace"),
    "notes": has(exps, "import had to assume"),
    "prescriber": has(exps, "Trace Prescriber"),
    "parse_error": next((e for e in errs
                         if "Couldn't read that board file" in e), None),
    "panel_reached": has([b.label for b in at.button], "Demo"),
}}))
'''


def _render(case: str) -> dict:
    """Render one scenario in a fresh interpreter and return what it drew."""
    out = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(_HARNESS).format(
            root=_ROOT, case=case)],
        capture_output=True, text=True, timeout=1800, cwd=_ROOT)
    probe = os.path.join(_ROOT, "_ui_probe_app.py")
    if os.path.exists(probe):
        os.remove(probe)          # never leave scratch behind in the repo
    for line in out.stdout.splitlines():
        if line.startswith("@@"):
            return json.loads(line[2:])
    raise AssertionError(
        f"harness produced no result for {case!r}\\n"
        f"stdout tail: {out.stdout[-800:]}\\nstderr tail: {out.stderr[-800:]}")


@unittest.skipUnless(_deps_available(),
                     "streamlit/plotly/trimesh not installed")
class TestPcbDoctorPanel(unittest.TestCase):
    """Every board format must survive the panel, not just the parser."""

    def _assert_full_panel(self, r, fmt_label):
        self.assertIsNone(r["exception"], f"panel raised: {r['exception']}")
        self.assertTrue(r["panel_reached"], "panel did not render at all")
        self.assertIsNotNone(r["caption"], "board loaded but no caption drawn")
        self.assertIn(fmt_label, r["caption"])
        self.assertTrue(r["diagnosis"], "no diagnosis expander")
        self.assertTrue(r["retrace"], "no re-trace expander")
        self.assertIsNone(r["parse_error"])

    def test_kicad_board_renders(self):
        self._assert_full_panel(_render("kicad"), "KiCad")

    def test_altium_ascii_board_renders(self):
        r = _render("ascii")
        self._assert_full_panel(r, "Altium / Protel ASCII")
        self.assertTrue(r["notes"],
                        "Altium imports must surface their assumptions")

    def test_unreadable_file_keeps_the_prescriber(self):
        """A bare `return` on the error path used to take the Trace Prescriber
        down with it — removing the one tool that needs no file at all, at the
        exact moment the member is holding a file that would not open."""
        r = _render("unreadable")
        self.assertIsNone(r["exception"])
        self.assertIsNotNone(r["parse_error"], "bad file drew no error")
        self.assertTrue(r["prescriber"],
                        "the Prescriber must survive an unreadable board")

    def test_panel_is_usable_with_no_board_at_all(self):
        r = _render("empty")
        self.assertIsNone(r["exception"])
        self.assertTrue(r["prescriber"])
        self.assertIsNone(r["caption"])


if __name__ == "__main__":
    unittest.main()
