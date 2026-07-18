# ============================================================================
#  KinematiK — ui/ package
#
#  The strangulation boundary from the CTO roadmap: streamlit_app.py is frozen
#  for additions. Every NEW tab is a module in this package exposing a single
#  `render()` entry point, imported lazily by the shell. When an old tab is
#  touched for any other reason, it moves here. Target: no file over 3,000
#  lines within two seasons.
#
#  Rules for modules in this package:
#    * `render()` is the only public surface the shell calls.
#    * No physics — equations live in `suspension/`; a ui module orchestrates
#      solvers and draws, nothing else.
#    * Import streamlit inside render(), never at module top level, so the
#      package stays importable headless (and testable for its pure helpers).
# ============================================================================
