KinematiK — everything the audit changed
========================================
177 files. Folder structure matches the repo, so this unzips over a checkout
in place. Runtime artefacts (analytics_buffer.jsonl, process_library.xlsx)
were deliberately excluded — they change on every local run and are machine
state, not fixes.

APPLY IN THIS ORDER
-------------------
  1. unzip over your checkout
  2. ./APPLY_REMOVALS.sh        <- REQUIRED, see below
  3. ruff check . && python -m pytest

STEP 2 IS NOT OPTIONAL
----------------------
A zip cannot express a deletion. Unzipping alone leaves all 140 removed files
in place, and one of them — the stale 96-file tests/tests clone — inserts the
wrong root at sys.path[0] and can shadow the real `suspension` package for an
entire test session. That was the cause of all 16 original collection errors.

APPLY_REMOVALS.sh moves them to _attic/removed_by_audit/ rather than deleting.
Nothing is lost. REMOVED_FILES.txt lists all 140.

NON-PYTHON FILES (6)
--------------------
  pyproject.toml             packaging (ui/, coordinate_frames as py-modules),
                             drive + voice extras, lint select/ignore with
                             reasons, pytest norecursedirs, deprecations as
                             errors
  .streamlit/config.toml     XSRF + CORS re-enabled. Shipped disabled with a
                             comment saying "review before a real public
                             deploy"; that review never happened.
  .github/workflows/ci.yml   NEW. There was no CI of any kind. Three jobs:
                             ruff, an installed-package import check, and the
                             full suite with [all] extras.
  .gitignore                 blocks "(1)"/"kopie"/"copy" artefacts, _attic/,
                             and the committed analytics buffer
  docs/EXTRACTING_A_VIEW.md  NEW. The monolith-split procedure, proven on the
                             first slice.
  STRUCTURAL_AUDIT_2026-08.md  the full findings write-up

SUBSTANTIVE PYTHON CHANGES (~24 of 171)
---------------------------------------
  suspension/sim_handoff.py        NEW — rebuilt from its 478-line test spec
  suspension/bracket_fos.py        tear-out + net-section corrections (FoS math)
  suspension/hardpoint_import.py   duplicate token in the link vocabulary
  suspension/express.py            warnings.warn stacklevel
  coordinate_frames.py             repointed a shadowed `import project`
  ui/phantom_envelope.py           NameError fix (_units out of scope)
  ui/run_log.py                    NEW — first slice out of the monolith
  streamlit_app.py                 run-log view replaced by a delegation call

  tests/test_sim_handoff.py          restored direct import
  tests/test_public_api_exports.py   known-gaps registry (now empty)
  tests/test_track_sim_export.py     renamed a silently-shadowed test
  tests/test_report.py               credential vs library branch; importorskip
  tests/test_daq_plan.py             headless guard now runs in a subprocess
  tests/test_fuse_test.py            same, plus removed an `or True` no-op
  tests/test_bracket_fos.py          4 new tests pinning the FoS corrections
  tests/test_integration.py          assert False -> raise AssertionError
  tests/test_kinematics.py           same
  tests/test_run_log_ui.py           harness now CALLS the view, not exec()s it
  tests/test_brief_coverage.py       retired the two-app-copy contract
  tests/test_app_single_source.py    NEW — guards against a copy reappearing
  tests/test_drive_oauth.py          NEW to tests/ (was dead inside the package)
  tests/test_report_store.py         NEW to tests/ (same)
  tests/test_ui_panels_reachable.py  NEW — four finished ui/ panels are not
                                     reachable from the app; this declares and
                                     tracks them instead of hiding them

THE OTHER ~147
--------------
Changed only by the pyupgrade pass: Optional[X] -> X | None, List[x] ->
list[x], unquoted annotations, datetime.UTC, and the typing imports that
became unused as a result. No behaviour change. Worth knowing before you
review 147 diffs looking for meaning that isn't there.

EXPECTED RESULT
---------------
  2,731 passed, 17 skipped, 0 failed
  ruff: all checks passed

FOUR PANELS, NOW WIRED IN
-------------------------
ui/arch_synth.py, ui/degradation.py, ui/report.py and ui/worthwhile.py were
complete, working panels that nothing imported. All four are now in the
Integration tab's feature_menu as "Architecture synthesis", "Transient
degradation", "Calculation report" and "Worthwhile once assembled?" — each
answers a whole-car question rather than a single-subsystem one, which is what
made Integration the right home. Look at them in the app; if one belongs in a
different tab, moving it is a one-line change to the menu list and its dispatch
branch.

Five ui modules (those four plus daq_plan) had streamlit imported at module
scope, against the ui/__init__.py contract. Moved inside render().
tests/test_ui_panels_reachable.py now enforces that on EVERY panel.

HEADS-UP FOR THE TEAM
---------------------
The bracket_fos corrections make some brackets FAIL that previously passed —
tear-out was up to 2.67x optimistic. Any design screened against the old
numbers needs re-checking, and some of it sat closer to the edge than the tool
reported. Tell people before they find out themselves.
