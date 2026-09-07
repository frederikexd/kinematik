# ============================================================================
#  KinematiK — ui/run_log.py
#
#  ANSYS run-log consolidation, extracted from streamlit_app.py (Aug 2026).
#
#  THE FIRST EXTRACTION under the strangulation plan in ui/__init__.py. Chosen
#  because it was the cleanest seam in the monolith, not the biggest win: the
#  block was already written to be lifted (its inner exception class carries a
#  comment saying so), and a free-variable analysis found exactly two
#  dependencies on the shell — `st` and the aero reference area. Everything
#  else it needs it imports itself.
#
#  That is the whole point of the pattern: a view that reaches into thirty
#  shell locals cannot be extracted safely, and a view that reaches into two
#  can. Run the same analysis before moving the next one.
#
#  Per ui/__init__.py: `render()` is the only public surface, no physics lives
#  here (that is suspension/aero/run_log.py), and streamlit is passed in rather
#  than imported at module top level so this file stays importable headless.
# ============================================================================
from __future__ import annotations


def render(st, aero_area: float = 1.0) -> None:
    """Draw the run-log consolidation view.

    Args:
        st: the streamlit module (or a recording mock, in tests).
        aero_area: reference area the coefficients are referenced to, m^2.
    """
    _aero_area = float(aero_area or 1.0)

    # Everything in this view is wrapped so that a failure here reports ITSELF
    # and leaves the rest of the Aerodynamics tab standing. Without this, one
    # missing attribute on the engine raised all the way to the tab-level
    # handler and replaced the whole workspace with "Could not build the aero
    # workspace" — the view's problem, presented as the tab's.
    try:
      class _RunLogReported(Exception):
          """Already shown the user a specific, actionable message; the
          wrapper below swallows this so it is not reported twice. Defined
          here rather than at module level so the view block stays
          self-contained and can be lifted out and tested on its own."""

      import suspension.aero.run_log as _rl

      # The view and the engine ship together, so a partial update (or a stale
      # .pyc left behind by one) puts an older run_log.py on the path and the
      # view reaches for fields it does not have. That used to raise straight
      # past this view into the tab-level handler and take the ENTIRE
      # Aerodynamics tab down with it. Check up front and say what to do.
      _rl_missing = [_n for _n in ("ScreenConfig", "process", "write_workbook",
                                   "consolidated_csv", "to_coeff_results",
                                   "Flag", "Severity")
                     if not hasattr(_rl, _n)]
      _rl_missing += ["ConsolidatedCase." + _n
                      for _n in ("setup_summary", "setup_consistent")
                      if not hasattr(getattr(_rl, "ConsolidatedCase", object), _n)]
      _rl_missing += ["ConsolidationReport." + _n
                      for _n in ("contributor_stats",)
                      if not hasattr(getattr(_rl, "ConsolidationReport", object), _n)]
      if _rl_missing:
          st.error(
              "**This view is newer than the engine behind it.** "
              f"`suspension/aero/run_log.py` is missing: "
              f"{', '.join(_rl_missing)}.\n\n"
              "Replace `suspension/aero/run_log.py` with the version that "
              "shipped alongside this `streamlit_app.py`, then delete any "
              "stale `__pycache__` folders (`find . -name __pycache__ -exec "
              "rm -rf {} +`) and restart Streamlit \u2014 an unzipped file can "
              "carry an older timestamp than the .pyc cached beside it, and "
              "Python will keep using the cache.")
          st.caption(f"Loaded engine: {getattr(_rl, '__file__', 'unknown')}")
          raise _RunLogReported()

      st.markdown(
          '<p class="hint">Drop in the <b>run log the wings team fills in after '
          'each Fluent run</b> — the sheet with mesh settings on the left and '
          'coefficients on the right. Every row is screened against explicit '
          'physics and mesh-quality criteria, the runs that survive are averaged '
          'per operating point, and <b>every exclusion carries a reason</b>. '
          'Nothing is dropped silently. Three checks do most of the work: y+ is '
          'judged against the row\u2019s own turbulence model (y+ 25 is excellent '
          'for k-omega SST and unusable for k-epsilon), peak gauge pressure \u00f7 q '
          'must sit near Cp = 1 or the reference conditions are wrong, and each '
          'row\u2019s implied reference area is cross-checked against its '
          'neighbours\u2019 \u2014 the failure that makes two contributors\u2019 '
          'coefficients silently incomparable.</p>',
          unsafe_allow_html=True)

      _rl_up = st.file_uploader(
          "Run log (.xlsx / .csv) exported from ANSYS or kept by hand",
          type=["xlsx", "xlsm", "csv", "tsv"], key="rl_upload",
          help="The parser tolerates a banner row above the header, renamed "
               "columns, units baked into the header text and blank filler "
               "rows \u2014 that is what the real sheet looks like.")

      with st.expander("Screening criteria \u2014 the thresholds these verdicts use",
                       expanded=False):
          st.caption("Defaults are documented engineering judgement, not laws. "
                     "They live here so you can argue with them once you see "
                     "which ones fire. Every value used is written into the "
                     "Config sheet of the workbook you download.")
          _rc1 = st.columns([1, 1, 1])
          _rl_rho = _rc1[0].number_input(
              "Air density \u03c1 (kg/m\u00b3)", 0.9, 1.4, value=1.225, step=0.005,
              key="rl_rho",
              help="Sets dynamic pressure q = \u00bd\u03c1V\u00b2, which every Cp "
                   "and reference-area check is measured against.")
          _rl_area_mode = _rc1[1].selectbox(
              "Reference area", ["Infer from the rows", "Use the value below"],
              key="rl_area_mode",
              help="Inferred per row as |L|/(q\u00b7|Cl|), then the median across "
                   "the operating point. Override if the team has a declared area.")
          _rl_area = _rc1[2].number_input(
              "Reference area A (m\u00b2)", 0.05, 3.0,
              value=float(_aero_area if _aero_area > 0 else 0.268), step=0.001,
              format="%.4f", key="rl_area",
              help="Only used when 'Use the value below' is selected above.")

          _rc2 = st.columns([1, 1, 1])
          _rl_test = _rc2[0].checkbox(
              "Exclude scratch rows", value=True, key="rl_test",
              help="Rows whose contributor, component or notes match 'test', "
                   "'scratch', 'ignore', 'wip'\u2026 as whole words. Uncheck to "
                   "flag them without dropping them.")
          _rl_outlier = _rc2[1].checkbox(
              "Statistical outlier pass", value=True, key="rl_outlier",
              help="After every physics gate, runs that disagree with their "
                   "peers by more than 3.5 modified z-scores are rejected. "
                   "Automatically disabled below 4 runs at a point \u2014 with "
                   "three samples, 'the odd one out' is picking a favourite.")
          _rl_downforce = _rc2[2].checkbox(
              "Expect downforce (negative lift)", value=True, key="rl_downforce",
              help="This sheet's convention is negative lift = downforce. A "
                   "positive value then warns about a missing sign flip.")

          _rc3 = st.columns([1, 1, 1])
          _rl_first = _rc3[0].checkbox(
              "Reject first-order runs", value=False, key="rl_first_order",
              help="First-order spatial discretisation is diffusive \u2014 it "
                   "smears the pressure gradients a suction peak is made of, "
                   "so downforce reads low. Warned by default because it is a "
                   "legitimate way to START a solve; tick this once your team "
                   "has agreed every reported run finishes second-order.")
          _rl_setupchk = _rc3[1].checkbox(
              "Check setup consistency", value=True, key="rl_setupchk",
              help="Compares each run's turbulence model, scheme, "
                   "discretisation order and initialization against the other "
                   "runs at the same operating point. Two runs solved "
                   "differently are not two samples of the same quantity.")
          _rl_mixedturb = _rc3[2].checkbox(
              "Reject mixed turbulence models", value=False, key="rl_mixedturb",
              help="A mixed turbulence model at one operating point is the "
                   "sharpest version of that problem. Off by default: the tool "
                   "reports the split rather than picking which half of the "
                   "team was right. Tick it once you have decided.")

      _rl_report = st.session_state.get("rl_report")

      if _rl_up is not None and st.button(
              "\u26a1 Screen & consolidate", type="primary", key="rl_go"):
          try:
              _rl_cfg = _rl.ScreenConfig(
                  rho=float(_rl_rho),
                  reference_area_m2=(float(_rl_area)
                                     if _rl_area_mode.startswith("Use") else None),
                  reject_test_rows=bool(_rl_test),
                  enable_outlier_pass=bool(_rl_outlier),
                  expect_downforce=bool(_rl_downforce),
                  reject_first_order=bool(_rl_first),
                  check_setup_consistency=bool(_rl_setupchk),
                  reject_mixed_turbulence=bool(_rl_mixedturb))
              _rl_report = _rl.process(_rl_up.getvalue(), _rl_cfg)
              st.session_state["rl_report"] = _rl_report
              st.session_state["rl_source_name"] = _rl_up.name
          except Exception as _rl_e:
              st.session_state.pop("rl_report", None)
              st.error(f"Could not read that run log: {_rl_e}")
              _rl_report = None

      if _rl_report is not None:
          _rl_name = st.session_state.get("rl_source_name", "run log")
          st.caption(f"Source: **{_rl_name}**"
                     + (f" \u00b7 sheet: {_rl_report.sheet}"
                        if _rl_report.sheet else ""))

          for _w in _rl_report.parse_warnings:
              st.warning(_w)
          if _rl_report.unmapped_headers:
              st.caption("Columns carried through but not screened: "
                         + ", ".join(_rl_report.unmapped_headers))

          _rm = st.columns([1, 1, 1, 1])
          _rm[0].metric("Runs parsed", _rl_report.n_rows)
          _rm[1].metric("Accepted", len(_rl_report.accepted))
          _rm[2].metric("Rejected", len(_rl_report.rejected))
          _rm[3].metric("Operating points", len(_rl_report.cases))

          if not _rl_report.ok:
              st.error("Every run in this sheet was rejected \u2014 see the "
                       "reasons below. Nothing is averaged from it.")

          # --- the answer ------------------------------------------------- #
          st.markdown("**Consolidated results** \u2014 one row per operating point")
          _rl_rows = []
          for _c in _rl_report.cases:
              _rl_rows.append({
                  "Component": _c.case.component,
                  "Ride height (mm)": _c.case.ride_height_mm,
                  "Velocity (m/s)": _c.case.speed_ms,
                  "Runs kept": f"{_c.n_accepted}/{_c.n_total}",
                  "Mean Cl": (None if _c.lift_coeff_mean is None
                              else round(_c.lift_coeff_mean, 4)),
                  "Cl SD": (None if _c.lift_coeff_sd is None
                            else round(_c.lift_coeff_sd, 4)),
                  "Mean Cd": (None if _c.drag_coeff_mean is None
                              else round(_c.drag_coeff_mean, 4)),
                  "L/D": (None if _c.lift_to_drag is None
                          else round(_c.lift_to_drag, 2)),
                  "Spread %": (None if _c.spread_pct is None
                               else round(_c.spread_pct, 1)),
                  "Ref area (m\u00b2)": (None if _c.reference_area_m2 is None
                                     else round(_c.reference_area_m2, 4)),
                  "Confidence": _c.confidence,
                  # A coefficient without its method is not reproducible, so
                  # the setup travels with the number rather than sitting in a
                  # separate sheet nobody opens.
                  "Solver setup": _c.setup_summary(),
                  "Setup consistent?": ("yes" if _c.setup_consistent
                                        else "NO \u2014 mixed methods"),
              })
          if _rl_rows:
              st.dataframe(_rl_rows, width="stretch", hide_index=True)
          for _c in _rl_report.cases:
              if _c.notes:
                  st.caption(f"{_c.case.label()} \u2014 {'; '.join(_c.notes)}")

          # --- why runs were excluded ------------------------------------- #
          if _rl_report.rejected:
              st.markdown("**Excluded runs** \u2014 every one with its reason")
              st.dataframe(
                  [{"Row": _v.row.source_row,
                    "Contributor": _v.row.contributor,
                    "Operating point": _v.case.label() if _v.case else "",
                    "Flags": ", ".join(_v.reject_codes),
                    "Why": _v.reason()}
                   for _v in _rl_report.rejected],
                  width="stretch", hide_index=True)

          _rl_tally = _rl_report.flag_tally()
          if _rl_tally:
              st.caption("What is going wrong, most common first: "
                         + ", ".join(f"{_k} \u00d7{_n}"
                                     for _k, _n in _rl_tally.items()))

          with st.expander("Contributors \u2014 who submitted what",
                           expanded=False):
              st.caption("Not a leaderboard \u2014 a map of where a recurring "
                         "setup mistake lives, so it gets fixed once at the "
                         "source instead of being screened out of every batch.")
              st.dataframe(
                  [{"Contributor": _r["contributor"], "Runs": _r["runs"],
                    "Accepted": _r["accepted"], "Rejected": _r["rejected"],
                    "Acceptance (%)": round(_r["acceptance_pct"], 1),
                    "Most common findings": _r["top_flags"]}
                   for _r in _rl_report.contributor_stats()],
                  width="stretch", hide_index=True)

          with st.expander("Full screening report \u2014 every row, every finding",
                           expanded=False):
              st.caption("Clean rows appear too, so a row's absence from this "
                         "table is never how you learn it was dropped.")
              _rl_log = []
              for _v in _rl_report.verdicts:
                  _entries = _v.flags or [_rl.Flag(
                      "CLEAN", _rl.Severity.INFO,
                      "passed every screening criterion")]
                  for _f in _entries:
                      _rl_log.append({
                          "Row": _v.row.source_row,
                          "Contributor": _v.row.contributor,
                          "Verdict": "ACCEPTED" if _v.accepted else "REJECTED",
                          "Severity": _f.severity.upper(),
                          "Code": _f.code,
                          "Value": _f.value,
                          "Limit": _f.limit,
                          "Explanation": _f.message})
              st.dataframe(_rl_log, width="stretch", hide_index=True)

          # --- take it away ----------------------------------------------- #
          st.markdown("**Download**")
          _rl_dl = st.columns([1, 1])
          try:
              import os as _rl_os
              import tempfile as _rl_tmp
              _rl_dir = _rl_tmp.mkdtemp(prefix="kk_runlog_")
              _rl_xlsx = _rl_os.path.join(_rl_dir, "aero_consolidated.xlsx")
              _rl.write_workbook(_rl_report, _rl_xlsx)
              with open(_rl_xlsx, "rb") as _fh:
                  _rl_dl[0].download_button(
                      "\U0001f4d7 Consolidated workbook (.xlsx)", _fh.read(),
                      file_name="aero_consolidated.xlsx",
                      mime="application/vnd.openxmlformats-officedocument."
                           "spreadsheetml.sheet",
                      key="rl_dl_xlsx", width="stretch",
                      help="Five sheets: Consolidated, Accepted Runs, Rejected "
                           "Runs, Screening Report, Config. The mean/SD cells "
                           "are live formulas over the Accepted Runs sheet, so "
                           "the team can audit the average \u2014 or reinstate a "
                           "run \u2014 without re-running this tool.")
          except ImportError:
              _rl_dl[0].caption("Workbook export needs openpyxl "
                                "(`pip install openpyxl`).")
          except Exception as _rl_we:
              _rl_dl[0].caption(f"Workbook export failed: {_rl_we}")
          _rl_dl[1].download_button(
              "\U0001f4c4 Consolidated results (.csv)",
              _rl.consolidated_csv(_rl_report),
              file_name="aero_consolidated.csv", mime="text/csv",
              key="rl_dl_csv", width="stretch")

          # --- hand it to the rest of KinematiK ---------------------------- #
          _rl_results = _rl.to_coeff_results(_rl_report)
          if _rl_results:
              st.markdown("**Use these in the lap sim**")
              st.caption(
                  f"{len(_rl_results)} consolidated point(s) convert to the same "
                  "CoeffResult objects a solver backend produces, so they feed "
                  "AeroMap and the lap sim directly. Each carries its n / spread "
                  "in provenance \u2014 a single-run point stays labelled as one "
                  "all the way through.")
              if st.button("\u2b07\ufe0f Load into the aero map",
                           key="rl_to_map"):
                  st.session_state["aero_runlog_results"] = _rl_results
                  st.success(
                      f"{len(_rl_results)} point(s) staged for the aero map. "
                      "Sign convention preserved: this sheet already uses "
                      "negative = downforce, so nothing was flipped.")
      elif _rl_up is None:
          st.caption("No file yet. The sheet can be the raw .xlsx the team "
                     "keeps \u2014 banner row, renamed columns and all.")
    except _RunLogReported:
      pass          # the guard above already told the user exactly what to do
    except Exception as _rl_fatal:
      st.error(f"The run-log view failed: {_rl_fatal}")
      st.caption("The rest of the Aerodynamics tab is unaffected. If this "
                 "mentions a missing attribute, `suspension/aero/run_log.py` "
                 "is out of step with this file.")
