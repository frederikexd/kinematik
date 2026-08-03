# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_run_log_ui.py — the Aerodynamics-tab run-log view, pinned.
# ============================================================================
"""
The run-log engine is tested in tests/test_run_log.py. This file tests the thing
that engine is useless without: the UI wiring in streamlit_app.py that lets a
member actually reach it.

Streamlit cannot be imported in a headless CI run, and the app is 1.7 MB, so we
do what tests/test_mission_briefing.py does — lift the view's source out of the
AST and exec it against a recording mock of `st`. That exercises the ACTUAL view
code, not a reimplementation of it, and catches the failures a syntax check
cannot: a widget called with arguments it does not take, a report attribute that
was renamed, a download built from a method that no longer exists.

What's guarded:
  * the view is registered in the aero feature menu (otherwise it is unreachable
    no matter how correct the body is),
  * uploading a real run log runs the pipeline and renders results,
  * the rejected-runs table is populated with reasons — the honesty contract
    reaching the screen,
  * both downloads are offered and carry real bytes,
  * a corrupt upload surfaces an error instead of raising,
  * the view survives being rendered with no upload at all.
"""

import ast
import io
import os
import sys
import textwrap

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_APPS = [os.path.join(_ROOT, "streamlit_app.py"),
         os.path.join(_ROOT, "suspension", "streamlit_app.py")]

_VIEW_HEAD = 'elif _view == "ANSYS run-log consolidation":'


# --------------------------------------------------------------------------- #
#  Lift the view body out of the app source
# --------------------------------------------------------------------------- #
def _view_source(app_path):
    """The view's body, dedented and turned into a standalone `if True:` block."""
    with io.open(app_path, encoding="utf-8") as fh:
        lines = fh.readlines()
    start = next(i for i, ln in enumerate(lines) if _VIEW_HEAD in ln)
    indent = len(lines[start]) - len(lines[start].lstrip())
    end = start + 1
    for i in range(start + 1, len(lines)):
        ln = lines[i]
        if ln.strip() and (len(ln) - len(ln.lstrip())) <= indent:
            break
        end = i + 1
    body = "".join(lines[start:end])
    body = body.replace(_VIEW_HEAD, "if True:", 1)
    return textwrap.dedent(body)


# --------------------------------------------------------------------------- #
#  A recording mock of Streamlit
# --------------------------------------------------------------------------- #
class _Rec:
    """Records every widget call; returns whatever the test configured."""

    def __init__(self, log, answers):
        self._log = log
        self._answers = answers

    def _record(self, name, args, kwargs):
        self._log.append((name, args, kwargs))

    # -- passive surfaces ------------------------------------------------- #
    def markdown(self, *a, **k): self._record("markdown", a, k)
    def caption(self, *a, **k): self._record("caption", a, k)
    def error(self, *a, **k): self._record("error", a, k)
    def warning(self, *a, **k): self._record("warning", a, k)
    def success(self, *a, **k): self._record("success", a, k)
    def info(self, *a, **k): self._record("info", a, k)
    def metric(self, *a, **k): self._record("metric", a, k)
    def dataframe(self, *a, **k): self._record("dataframe", a, k)
    def write(self, *a, **k): self._record("write", a, k)

    # -- inputs ----------------------------------------------------------- #
    def _answer(self, name, key, default):
        self._record(name, (), {"key": key})
        return self._answers.get(key, default)

    def file_uploader(self, label, **k):
        self._record("file_uploader", (label,), k)
        return self._answers.get("upload")

    def button(self, label, **k):
        self._record("button", (label,), k)
        return bool(self._answers.get(k.get("key"), False))

    def download_button(self, label, data, **k):
        self._record("download_button", (label, data), k)
        return False

    def number_input(self, label, *a, **k):
        return self._answer("number_input", k.get("key"),
                            k.get("value", a[-1] if a else 0.0))

    def selectbox(self, label, options, **k):
        return self._answer("selectbox", k.get("key"), options[0])

    def checkbox(self, label, **k):
        return self._answer("checkbox", k.get("key"), k.get("value", False))

    def radio(self, label, options, **k):
        return self._answer("radio", k.get("key"), options[0])

    def text_input(self, label, **k):
        return self._answer("text_input", k.get("key"), k.get("value", ""))

    # -- layout ----------------------------------------------------------- #
    def columns(self, spec, **k):
        n = spec if isinstance(spec, int) else len(spec)
        return [_Rec(self._log, self._answers) for _ in range(n)]

    def expander(self, label, **k):
        self._record("expander", (label,), k)
        return self

    def __enter__(self): return self
    def __exit__(self, *exc): return False


class _MockSt(_Rec):
    def __init__(self, answers=None):
        self.log = []
        super().__init__(self.log, answers or {})
        self.session_state = {}

    def calls(self, name):
        return [(a, k) for n, a, k in self.log if n == name]

    def texts(self, name):
        out = []
        for a, _k in self.calls(name):
            out.extend(str(x) for x in a)
        return out


class _Upload:
    """Stands in for a Streamlit UploadedFile."""

    def __init__(self, data: bytes, name: str = "wings_runs.xlsx"):
        self._data = data
        self.name = name

    def getvalue(self):
        return self._data


# --------------------------------------------------------------------------- #
#  A real run log to feed it
# --------------------------------------------------------------------------- #
def _sample_csv() -> bytes:
    """A sheet with a banner row, a scratch row, a broken row and good rows."""
    q = 0.5 * 1.225 * 26.8224 ** 2
    area = 0.268
    header = [
        "Contributor", "Front or Rear Wing?", "Ride-Height (mm)", "Velocity (m/s)",
        "Desired Y+", "Min Surface Mesh Length", "Max Surface Mesh Length",
        "First Layer Height (m)", "Number of Layers", "Min Orthogonal Quality",
        "Max Skewness", "Max Aspect Ratio", "Viscous Model",
        # The solver-setup columns are part of the real sheet; without them
        # every row carries a SETUP_UNRECORDED warning and nothing reads CLEAN.
        "Scheme", "Order", "Pseudo Time Step", "Courant Number",
        "Initialization",
        "Lift Force (N)",
        "Lift Coefficient", "Drag Force (N)", "Drag Coefficient",
        "Max Pressure (Pa)", "Min. Pressure (Pa)", "Mass Imbalance (kg/s)",
        "Average Y+", "Notes",
    ]

    def row(who, cl, mesh=(0.006, 0.012), ortho=0.30, note=""):
        return [who, "Front Wing", 40, 26.8224, 40, mesh[0], mesh[1],
                6.267e-4, 8, ortho, 0.70, 1346.37, "k-epsilon",
                "Simple", "Second", "0.5", 20.0, "Standard",
                cl * q * area, cl, 0.199 * q * area, "",
                1.02 * q, -3.4 * q, 7.3e-6, 45.0, note]

    rows = [
        ["Wings Team Simulation Results"] + [""] * (len(header) - 1),
        header,
        row("Khalil - Test", -0.815, note="first go, ignore"),
        row("Khalil", -0.809, mesh=(0.005, 0.004)),      # min > max
        row("Adriane", -0.826),
        row("Rohan", -0.824),
    ]
    buf = io.StringIO()
    import csv
    csv.writer(buf, lineterminator="\n").writerows(rows)
    return buf.getvalue().encode("utf-8")


def _run_view(answers, app_path=_APPS[0]):
    """Exec the real view body against a mock st; return the mock."""
    st = _MockSt(answers)
    ns = {"st": st, "_view": "ANSYS run-log consolidation", "_aero_area": 1.0}
    exec(compile(_view_source(app_path), "<view>", "exec"), ns)  # noqa: S102
    return st


def _table_with(mock, *columns):
    """Find a rendered dataframe by its columns, not by its render order."""
    for a, _k in mock.calls("dataframe"):
        rows = a[0]
        if rows and all(c in rows[0] for c in columns):
            return rows
    raise AssertionError(f"no rendered table with columns {columns}")


# --------------------------------------------------------------------------- #
#  Registration — an unreachable view is not a feature
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("app_path", _APPS, ids=["root", "package"])
def test_view_is_registered_in_the_aero_feature_menu(app_path):
    with io.open(app_path, encoding="utf-8") as fh:
        src = fh.read()
    menu = src.split('feature_menu("aerodynamics"', 1)[1][:1200]
    assert '"ANSYS run-log consolidation"' in menu, (
        "the view exists but is not offered in the Aerodynamics tool menu")
    assert "Screen the Fluent run sheet" in menu, "no menu description"


@pytest.mark.parametrize("app_path", _APPS, ids=["root", "package"])
def test_view_body_parses_standalone(app_path):
    ast.parse(_view_source(app_path))


@pytest.mark.parametrize("app_path", _APPS, ids=["root", "package"])
def test_both_app_copies_carry_the_same_view(app_path):
    assert _view_source(app_path) == _view_source(_APPS[0])


# --------------------------------------------------------------------------- #
#  The happy path
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def ran():
    return _run_view({
        "upload": _Upload(_sample_csv(), "wings_runs.csv"),
        "rl_go": True,
    })


def test_upload_runs_the_pipeline_and_stores_the_report(ran):
    report = ran.session_state.get("rl_report")
    assert report is not None, "the screen button did not produce a report"
    assert report.n_rows == 4
    assert len(report.accepted) == 2
    assert len(report.rejected) == 2


def test_headline_metrics_are_rendered(ran):
    labels = [a[0] for a, _k in ran.calls("metric")]
    assert labels == ["Runs parsed", "Accepted", "Rejected", "Operating points"]
    values = [a[1] for a, _k in ran.calls("metric")]
    assert values == [4, 2, 2, 1]


def test_consolidated_table_reaches_the_screen(ran):
    consolidated = _table_with(ran, "Runs kept", "Mean Cl")
    assert len(consolidated) == 1
    row = consolidated[0]
    assert row["Runs kept"] == "2/4"
    assert row["Mean Cl"] == pytest.approx(-0.825, abs=1e-3)
    # Drag Coefficient is blank in the sheet — it must have been derived.
    assert row["Mean Cd"] is not None


def test_rejected_runs_are_shown_with_their_reasons(ran):
    """The honesty contract, reaching the screen: no silent filtering."""
    rejected = _table_with(ran, "Why", "Flags")
    assert len(rejected) == 2
    reasons = " ".join(r["Why"] for r in rejected)
    assert "scratch" in reasons.lower()
    assert "larger than the max" in reasons
    assert all(r["Flags"] for r in rejected)


def test_screening_report_includes_clean_rows(ran):
    """A row's absence must never be how you learn it was dropped."""
    log = _table_with(ran, "Code", "Severity", "Explanation")
    assert any(r["Code"] == "CLEAN" for r in log)
    assert {r["Verdict"] for r in log} == {"ACCEPTED", "REJECTED"}


def test_both_downloads_are_offered_with_real_bytes(ran):
    dl = ran.calls("download_button")
    labels = [a[0] for a, _k in dl]
    assert any(".xlsx" in x for x in labels), "no workbook download"
    assert any(".csv" in x for x in labels), "no CSV download"
    for a, _k in dl:
        payload = a[1]
        assert payload, "download offered with empty payload"
    xlsx = next(a[1] for a, _k in dl if ".xlsx" in a[0])
    assert xlsx[:2] == b"PK", "workbook payload is not a real xlsx"


def test_csv_download_is_the_consolidated_table(ran):
    dl = ran.calls("download_button")
    csv_payload = next(a[1] for a, _k in dl if ".csv" in a[0])
    assert "Mean Lift Coefficient" in csv_payload
    assert csv_payload.count("\n") >= 2


def test_lap_sim_handoff_is_offered(ran):
    buttons = [a[0] for a, _k in ran.calls("button")]
    assert any("aero map" in b for b in buttons)


def test_handoff_stages_coeff_results():
    st = _run_view({
        "upload": _Upload(_sample_csv(), "wings_runs.csv"),
        "rl_go": True, "rl_to_map": True,
    })
    staged = st.session_state.get("aero_runlog_results")
    assert staged and len(staged) == 1
    assert staged[0].c_lift < 0, "sign convention was flipped"
    assert staged[0].provenance.backend == "ansys-run-log"
    assert any("staged" in t for t in st.texts("success"))


# --------------------------------------------------------------------------- #
#  Settings are actually wired to the engine
# --------------------------------------------------------------------------- #
def test_keeping_scratch_rows_changes_the_verdict():
    st = _run_view({
        "upload": _Upload(_sample_csv(), "wings_runs.csv"),
        "rl_go": True, "rl_test": False,
    })
    report = st.session_state["rl_report"]
    assert len(report.accepted) == 3, "the scratch-row toggle is not wired through"
    codes = {f.code for v in report.verdicts for f in v.flags}
    assert "TEST_ROW" in codes, "the row should still be flagged, just not dropped"


def test_supplied_reference_area_is_wired_through():
    st = _run_view({
        "upload": _Upload(_sample_csv(), "wings_runs.csv"),
        "rl_go": True,
        "rl_area_mode": "Use the value below",
        "rl_area": 0.5,
    })
    case = st.session_state["rl_report"].cases[0]
    assert case.reference_area_m2 == pytest.approx(0.5)
    assert "supplied" in case.reference_area_basis


# --------------------------------------------------------------------------- #
#  Failure modes
# --------------------------------------------------------------------------- #
def test_corrupt_upload_surfaces_an_error_instead_of_raising():
    st = _run_view({
        "upload": _Upload(b"PK\x03\x04 not really a workbook", "broken.xlsx"),
        "rl_go": True,
    })
    assert st.texts("error"), "a broken file must produce a visible error"
    assert "rl_report" not in st.session_state


def test_no_upload_renders_the_empty_state():
    st = _run_view({"upload": None})
    assert not st.calls("metric")
    assert any("No file yet" in t for t in st.texts("caption"))


def test_upload_without_pressing_the_button_does_nothing():
    st = _run_view({"upload": _Upload(_sample_csv(), "x.csv"), "rl_go": False})
    assert "rl_report" not in st.session_state
    assert not st.calls("metric")


def test_sheet_where_everything_is_rejected_says_so():
    q = 0.5 * 1.225 * 26.8224 ** 2
    header = ("Contributor,Front or Rear Wing?,Ride-Height (mm),Velocity (m/s),"
              "Min Orthogonal Quality,Viscous Model,Scheme,Order,"
              "Initialization,Lift Force (N),Lift Coefficient,Average Y+\n")
    bad = (f"Test rig,Front Wing,40,26.8224,0.01,k-epsilon,Simple,Second,"
           f"Standard,{-0.8 * q * 0.268},-0.8,45\n")
    st = _run_view({
        "upload": _Upload(("banner\n" + header + bad).encode("utf-8"), "x.csv"),
        "rl_go": True,
    })
    assert st.texts("error"), "an all-rejected sheet must say so loudly"
    assert any("rejected" in t.lower() for t in st.texts("error"))


# --------------------------------------------------------------------------- #
#  The solver-setup parameters and contributor breakdown reach the screen
# --------------------------------------------------------------------------- #
def test_consolidated_table_shows_the_method_behind_the_number(ran):
    """A coefficient without its solver setup is not reproducible."""
    row = _table_with(ran, "Solver setup", "Setup consistent?")[0]
    assert "k-epsilon" in row["Solver setup"]
    assert "second-order" in row["Solver setup"]
    assert row["Setup consistent?"] == "yes"


def test_contributors_table_is_rendered(ran):
    who = _table_with(ran, "Contributor", "Acceptance (%)")
    assert {r["Contributor"] for r in who} >= {"Adriane", "Rohan"}
    # The fixture has both "Khalil" and "Khalil - Test"; match exactly.
    scratch = [r for r in who if r["Contributor"] == "Khalil - Test"][0]
    assert scratch["Accepted"] == 0
    assert "TEST_ROW" in scratch["Most common findings"]
    broken = [r for r in who if r["Contributor"] == "Khalil"][0]
    assert "MESH_LENGTH_INVERTED" in broken["Most common findings"]


def test_first_order_toggle_is_wired_through():
    csvdata = _sample_csv().decode().replace("Second", "First")
    st = _run_view({"upload": _Upload(csvdata.encode(), "x.csv"),
                    "rl_go": True, "rl_first_order": True})
    report = st.session_state["rl_report"]
    assert len(report.accepted) == 0, "first-order rejection is not wired through"
    assert any("FIRST_ORDER" in v.reject_codes for v in report.rejected)


def test_setup_consistency_toggle_is_wired_through():
    """Same sheet, one contributor on a different scheme."""
    base = _sample_csv().decode().splitlines()
    odd = base[-1].replace("Rohan", "Priya").replace(",Simple,", ",Coupled,")
    data = "\n".join(base + [odd]) + "\n"
    on = _run_view({"upload": _Upload(data.encode(), "x.csv"), "rl_go": True})
    codes_on = {c for v in on.session_state["rl_report"].verdicts
                for c in v.warn_codes}
    assert "SETUP_MISMATCH" in codes_on

    off = _run_view({"upload": _Upload(data.encode(), "x.csv"),
                     "rl_go": True, "rl_setupchk": False})
    codes_off = {c for v in off.session_state["rl_report"].verdicts
                 for c in v.warn_codes}
    assert "SETUP_MISMATCH" not in codes_off


# --------------------------------------------------------------------------- #
#  A stale engine must not take the tab down
# --------------------------------------------------------------------------- #
#  Reported from the field: "Could not build the aero workspace: 'ConsolidatedCase'
#  object has no attribute 'setup_summary'". The view and the engine ship
#  together, so a partial update (or a stale .pyc — an unzipped file can carry an
#  older timestamp than the cache beside it) leaves an old run_log.py on the path.
#  The real defect was not the mismatch: it was that ONE missing attribute raised
#  past this view into the tab-level handler and replaced the entire Aerodynamics
#  workspace with a message about the tab. A view's problem must stay the view's.
import types                                                       # noqa: E402


def _with_engine(monkeypatch, module):
    """Swap the engine the view imports. `import a.b.c as x` resolves through
    the PARENT PACKAGE attribute, so patching sys.modules alone is not enough."""
    import suspension.aero
    monkeypatch.setitem(sys.modules, "suspension.aero.run_log", module)
    monkeypatch.setattr(suspension.aero, "run_log", module, raising=False)


def _outdated_engine():
    import suspension.aero.run_log as rl

    class _OldCase:            # pre-solver-setup ConsolidatedCase
        pass

    class _OldReport:          # pre-contributor-stats ConsolidationReport
        pass

    return types.SimpleNamespace(
        ScreenConfig=rl.ScreenConfig, process=rl.process,
        write_workbook=rl.write_workbook, consolidated_csv=rl.consolidated_csv,
        to_coeff_results=rl.to_coeff_results, Flag=rl.Flag, Severity=rl.Severity,
        ConsolidatedCase=_OldCase, ConsolidationReport=_OldReport,
        __file__="/somewhere/old/run_log.py")


def test_outdated_engine_reports_itself_and_names_what_is_missing(monkeypatch):
    _with_engine(monkeypatch, _outdated_engine())
    st = _run_view({"upload": _Upload(_sample_csv(), "x.csv"), "rl_go": True})

    errors = st.texts("error")
    assert len(errors) == 1, "the same problem must not be reported twice"
    msg = errors[0]
    assert "newer than the engine" in msg
    for missing in ("setup_summary", "setup_consistent", "contributor_stats"):
        assert missing in msg, f"{missing} not named in the message"
    # Actionable: says what to replace AND warns about the stale .pyc trap.
    assert "run_log.py" in msg and "__pycache__" in msg
    # And names which file actually got loaded, so a shadowed copy is findable.
    assert any("/somewhere/old/run_log.py" in c for c in st.texts("caption"))


def test_outdated_engine_stops_before_rendering_half_a_result(monkeypatch):
    _with_engine(monkeypatch, _outdated_engine())
    st = _run_view({"upload": _Upload(_sample_csv(), "x.csv"), "rl_go": True})
    assert not st.calls("metric")
    assert not st.calls("dataframe")
    assert not st.calls("download_button")


def test_an_unexpected_engine_failure_is_contained_to_the_view(monkeypatch):
    """
    Not just the known mismatch: ANY exception in the view must be caught here.
    This is the regression for the tab-level "Could not build the aero
    workspace" that the field report actually showed.
    """
    import suspension.aero.run_log as rl

    def _boom(*a, **k):
        raise RuntimeError("engine exploded")

    # NOT process(): that call already has its own, more specific handler
    # ("Could not read that run log"). consolidated_csv runs later, outside it —
    # exactly the kind of call that used to escape and take the tab with it.
    broken = types.SimpleNamespace(
        ScreenConfig=rl.ScreenConfig, process=rl.process,
        write_workbook=rl.write_workbook, consolidated_csv=_boom,
        to_coeff_results=rl.to_coeff_results, Flag=rl.Flag, Severity=rl.Severity,
        ConsolidatedCase=rl.ConsolidatedCase,
        ConsolidationReport=rl.ConsolidationReport, __file__="x")
    _with_engine(monkeypatch, broken)

    # Must not raise — if it does, the tab-level handler eats the whole tab.
    st = _run_view({"upload": _Upload(_sample_csv(), "x.csv"), "rl_go": True})
    assert any("engine exploded" in e for e in st.texts("error"))
    assert any("rest of the Aerodynamics tab is unaffected" in c
               for c in st.texts("caption"))
    # The results rendered before the failure are still on screen.
    assert st.calls("metric")


def test_a_failure_inside_process_gets_its_own_specific_message(monkeypatch):
    """The inner handler is more specific and should win over the wrapper."""
    import suspension.aero.run_log as rl

    def _boom(*a, **k):
        raise RuntimeError("engine exploded")

    broken = types.SimpleNamespace(
        ScreenConfig=rl.ScreenConfig, process=_boom,
        write_workbook=rl.write_workbook, consolidated_csv=rl.consolidated_csv,
        to_coeff_results=rl.to_coeff_results, Flag=rl.Flag, Severity=rl.Severity,
        ConsolidatedCase=rl.ConsolidatedCase,
        ConsolidationReport=rl.ConsolidationReport, __file__="x")
    _with_engine(monkeypatch, broken)
    st = _run_view({"upload": _Upload(_sample_csv(), "x.csv"), "rl_go": True})
    assert any("Could not read that run log" in e for e in st.texts("error"))
    assert "rl_report" not in st.session_state


def test_current_engine_triggers_no_version_warning(ran):
    """The guard must be silent when the shipped engine is in place."""
    assert not any("newer than the engine" in e for e in ran.texts("error"))
