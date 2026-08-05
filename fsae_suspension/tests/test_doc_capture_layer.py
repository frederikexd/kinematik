# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Behavioural tests for the app-wide documentation capture layer.

Every feature tab used to produce a document that said, at best, "Ran a
calculation" — the numbers a member actually looked at were never recorded, so
the Documentation tab reported nothing captured for 38 of 39 features. The
capture layer fixes that generically: it lifts `st.metric` values and the
success/warning/error verdict banners off whatever a feature renders, with no
per-feature wiring.

Because that layer is pure session-state bookkeeping around Streamlit, it can be
exercised without Streamlit installed: this module EXECS THE REAL SOURCE BLOCKS
out of streamlit_app.py against a stub `st`, so the behaviour under test is the
shipped code rather than a copy that can drift.

What is pinned here:
  * a metric rendered inside an opened feature is captured, with its value as the
    user saw it (already unit-converted upstream);
  * metrics dedupe by label with last-value-wins, so scrubbing a slider leaves
    one current row rather than a history;
  * verdict banners are captured with severity, and re-renders don't duplicate;
  * ATTRIBUTION: a feature only captures its own renders. Every tab body runs on
    every Streamlit script-run, so this is the property that stops all 38 tabs'
    numbers landing on whichever tab happens to be visible;
  * GATING: a feature whose body executed but which the user never opened
    captures nothing, so reports never contain work nobody looked at;
  * caps hold, so a long session cannot grow session-state without bound;
  * the section builder emits nothing for an untouched feature and real
    markdown for a used one.
"""

import os
import types

import pytest


_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_APP = os.path.join(_ROOT, "streamlit_app.py")


class _FakeDG:
    """Stands in for a Streamlit container (a column / DeltaGenerator).

    Deliberately NOT a str: the wrapper distinguishes `col.metric(label, ...)`
    from the already-bound `st.metric(label, ...)` by whether the first
    positional argument is a string, so any non-string object stands in.
    """


def _extract(src, start_marker, end_marker):
    a = src.index(start_marker)
    b = src.index(end_marker, a)
    return src[a:b]


@pytest.fixture
def cap():
    """The real capture layer, exec'd against a stub `st`.

    Returns the module namespace so tests can call capture_metric /
    capture_verdict / _captured_result_sections directly and inspect the
    session-state the shipped code writes.
    """
    if not os.path.exists(_APP):
        pytest.skip("streamlit_app.py not present in this checkout")
    with open(_APP, encoding="utf-8") as fh:
        src = fh.read()

    # --- stub streamlit -------------------------------------------------- #
    st = types.SimpleNamespace()
    st.session_state = {}

    ns = {"st": st, "DeltaGenerator": _FakeDG}

    # The capture layer itself: constants through get_feature_results.
    ns_src = _extract(src, "_FEATURE_RESULTS_KEY = ",
                      "def _ax_positional_args(")
    exec(compile(ns_src, "<capture-core>", "exec"), ns)

    # The wrappers.
    wrap_src = _extract(src, "def _ax_positional_args(args):",
                        "if not getattr(st, \"_ax_capture_patched\", False):")
    exec(compile(wrap_src, "<capture-wrappers>", "exec"), ns)

    # Feature registry + label helper, needed by the section builder.
    reg_src = _extract(src, "_FEATURE_SUBSYS = {", "def _feature_subsys(")
    exec(compile(reg_src, "<registry>", "exec"), ns)
    meta_src = _extract(src, "_TAB_META = {", "\n_TAB_ORDER") \
        if "_TAB_ORDER" in src else None
    if meta_src:
        try:
            exec(compile(meta_src, "<tabmeta>", "exec"), ns)
        except Exception:
            ns.setdefault("_TAB_META", {})
    ns.setdefault("_TAB_META", {})
    ns.setdefault("_feature_label", lambda f: str(f).replace("_", " ").title())

    # The section builders.
    sec_src = _extract(src, "_SEVERITY_MARK = ",
                       "def _captured_sections_for_subsystem(")
    exec(compile(sec_src, "<sections>", "exec"), ns)
    sub_src = _extract(src, "def _captured_sections_for_subsystem(",
                       "\ndef _render_doc_and_verdict(")
    exec(compile(sub_src, "<sections2>", "exec"), ns)

    ns["_st"] = st
    return ns


EMPTY = {"metrics": [], "verdicts": [], "artifacts": []}


def _open_feature(cap, feature):
    """Mark a feature as genuinely opened + currently rendering."""
    cap["_st"].session_state[f"_ax_open_{feature}"] = True
    cap["_st"].session_state["_ax_rendering_tab"] = feature


# --------------------------------------------------------------------------- #
#  Capture basics
# --------------------------------------------------------------------------- #
def test_metric_is_captured_for_the_rendering_feature(cap):
    _open_feature(cap, "brakes")
    cap["capture_metric"]("Installed length", "374.9 mm")
    got = cap["get_feature_results"]("brakes")["metrics"]
    assert got == [{"label": "Installed length", "value": "374.9 mm"}]


def test_metric_dedupes_by_label_last_value_wins(cap):
    _open_feature(cap, "brakes")
    for v in ("300 mm", "350 mm", "374.9 mm"):
        cap["capture_metric"]("Installed length", v)
    got = cap["get_feature_results"]("brakes")["metrics"]
    assert len(got) == 1, "scrubbing a slider must not append a row per rerun"
    assert got[0]["value"] == "374.9 mm"


def test_metric_delta_is_kept(cap):
    _open_feature(cap, "brakes")
    cap["capture_metric"]("Verdict", "DOES NOT FIT", delta="+84.9 mm")
    assert cap["get_feature_results"]("brakes")["metrics"][0]["delta"] \
        == "+84.9 mm"


def test_verdict_is_captured_with_severity(cap):
    _open_feature(cap, "brakes")
    cap["capture_verdict"]("The assembly is 84.9 mm longer than available.",
                           "fail")
    got = cap["get_feature_results"]("brakes")["verdicts"]
    assert got[0]["severity"] == "fail"
    assert "84.9 mm" in got[0]["text"]


def test_verdict_rerender_does_not_duplicate(cap):
    _open_feature(cap, "brakes")
    for _ in range(5):
        cap["capture_verdict"]("Fits with 12 mm to spare.", "ok")
    assert len(cap["get_feature_results"]("brakes")["verdicts"]) == 1


def test_short_chrome_strings_are_ignored(cap):
    _open_feature(cap, "brakes")
    cap["capture_verdict"]("Saved", "ok")
    cap["capture_verdict"]("OK", "ok")
    assert cap["get_feature_results"]("brakes")["verdicts"] == []


def test_raw_html_is_not_captured_as_a_finding(cap):
    _open_feature(cap, "brakes")
    cap["capture_verdict"]('<p class="hint">some layout chrome here</p>', "info")
    assert cap["get_feature_results"]("brakes")["verdicts"] == []


def test_whitespace_is_normalised(cap):
    _open_feature(cap, "brakes")
    cap["capture_verdict"]("The   assembly\n  is  too   long by 84.9 mm.", "fail")
    assert cap["get_feature_results"]("brakes")["verdicts"][0]["text"] \
        == "The assembly is too long by 84.9 mm."


# --------------------------------------------------------------------------- #
#  Attribution and gating — the two properties that make this safe app-wide
# --------------------------------------------------------------------------- #
def test_each_feature_captures_only_its_own_renders(cap):
    """Every tab body runs on every script-run; numbers must not cross over."""
    for feat, val in (("brakes", "374.9 mm"), ("aero", "1.85 CdA"),
                      ("kinematics", "-1.2 deg")):
        _open_feature(cap, feat)
        cap["capture_metric"]("Headline", val)
    assert cap["get_feature_results"]("brakes")["metrics"][0]["value"] \
        == "374.9 mm"
    assert cap["get_feature_results"]("aero")["metrics"][0]["value"] == "1.85 CdA"
    assert cap["get_feature_results"]("kinematics")["metrics"][0]["value"] \
        == "-1.2 deg"


def test_unopened_feature_captures_nothing(cap):
    """A tab body executes even when never visited — it must stay silent."""
    cap["_st"].session_state["_ax_rendering_tab"] = "aero"   # rendering...
    # ...but no _ax_open_aero flag, i.e. the user never went there.
    cap["capture_metric"]("Downforce", "540 N")
    cap["capture_verdict"]("Something was computed in the background.", "ok")
    assert cap["get_feature_results"]("aero") == EMPTY


def test_capture_outside_any_tab_is_a_noop(cap):
    cap["_st"].session_state["_ax_rendering_tab"] = None
    cap["capture_metric"]("Stray", "1")
    assert cap["_st"].session_state.get(cap["_FEATURE_RESULTS_KEY"], {}) == {}


def test_explicit_feature_argument_overrides_attribution(cap):
    cap["_st"].session_state["_ax_rendering_tab"] = None
    cap["capture_metric"]("Rotor ⌀", "220 mm", feature="brakes")
    assert cap["get_feature_results"]("brakes")["metrics"][0]["value"] == "220 mm"


# --------------------------------------------------------------------------- #
#  Bounds
# --------------------------------------------------------------------------- #
def test_metric_cap_holds(cap):
    _open_feature(cap, "brakes")
    for i in range(200):
        cap["capture_metric"](f"metric {i}", str(i))
    assert len(cap["get_feature_results"]("brakes")["metrics"]) \
        == cap["_MAX_CAPTURED_METRICS"]


def test_verdict_cap_holds(cap):
    _open_feature(cap, "brakes")
    for i in range(200):
        cap["capture_verdict"](f"A finding number {i} worth recording.", "warning")
    assert len(cap["get_feature_results"]("brakes")["verdicts"]) \
        == cap["_MAX_CAPTURED_VERDICTS"]


def test_overlong_verdict_is_skipped(cap):
    _open_feature(cap, "brakes")
    cap["capture_verdict"]("x" * 5000, "info")
    assert cap["get_feature_results"]("brakes")["verdicts"] == []


# --------------------------------------------------------------------------- #
#  Never raise
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("bad", [None, object(), 12.5, b"bytes"])
def test_capture_survives_junk_input(cap, bad):
    _open_feature(cap, "brakes")
    cap["capture_metric"](bad, bad)
    cap["capture_verdict"](bad, "info")     # must not raise


def test_capture_survives_broken_session_state(cap):
    class Exploding(dict):
        def setdefault(self, *a, **k):
            raise RuntimeError("boom")

        def get(self, *a, **k):
            raise RuntimeError("boom")

    cap["_st"].session_state = Exploding()
    cap["capture_metric"]("x", "1")          # must not raise
    cap["capture_verdict"]("a finding that is long enough", "ok")
    assert cap["get_feature_results"]("brakes") == EMPTY


# --------------------------------------------------------------------------- #
#  Section building
# --------------------------------------------------------------------------- #
def test_untouched_feature_yields_no_sections(cap):
    assert cap["_captured_result_sections"]("aero") == []


def test_sections_carry_the_numbers_and_the_verdicts(cap):
    _open_feature(cap, "brakes")
    cap["capture_metric"]("Installed length", "374.9 mm")
    cap["capture_metric"]("Available", "290.0 mm")
    cap["capture_verdict"]("The assembly is 84.9 mm too long.", "fail")
    secs = cap["_captured_result_sections"]("brakes")
    assert len(secs) == 2
    body = "\n".join(l for _, lines in secs for l in lines)
    assert "374.9 mm" in body and "290.0 mm" in body
    assert "84.9 mm too long" in body
    assert "❌" in body, "severity should be visible in the report"


def test_metrics_section_precedes_verdicts(cap):
    _open_feature(cap, "brakes")
    cap["capture_metric"]("A", "1")
    cap["capture_verdict"]("A finding worth reporting here.", "ok")
    heads = [h for h, _ in cap["_captured_result_sections"]("brakes")]
    assert "results" in heads[0]
    assert "verdicts" in heads[1]


def test_subsystem_rollup_gathers_every_owned_feature(cap):
    """The whole point: a subsystem report picks up all its tools, unwired."""
    for feat in ("kinematics", "tire", "laptime"):
        _open_feature(cap, feat)
        cap["capture_metric"]("Headline", f"{feat} value")
    secs = cap["_captured_sections_for_subsystem"]("suspension")
    body = "\n".join(l for _, lines in secs for l in lines)
    for feat in ("kinematics", "tire", "laptime"):
        assert f"{feat} value" in body


def test_subsystem_rollup_is_stably_ordered(cap):
    for feat in ("tire", "kinematics", "laptime"):
        _open_feature(cap, feat)
        cap["capture_metric"]("Headline", feat)
    a = cap["_captured_sections_for_subsystem"]("suspension")
    b = cap["_captured_sections_for_subsystem"]("suspension")
    assert a == b, "a report must not reshuffle between runs"


def test_rollup_ignores_features_owned_by_other_subsystems(cap):
    _open_feature(cap, "brakes")
    cap["capture_metric"]("Brake number", "42")
    secs = cap["_captured_sections_for_subsystem"]("suspension")
    body = "\n".join(l for _, lines in secs for l in lines)
    assert "Brake number" not in body


# --------------------------------------------------------------------------- #
#  Wrapper plumbing
# --------------------------------------------------------------------------- #
def test_metric_wrapper_captures_and_still_renders(cap):
    _open_feature(cap, "brakes")
    calls = []
    wrapped = cap["_ax_wrap_metric"](lambda *a, **k: calls.append((a, k)))
    wrapped("Travel at the pad", "54 mm")
    assert calls, "the original render must still happen"
    assert cap["get_feature_results"]("brakes")["metrics"][0]["label"] \
        == "Travel at the pad"


def test_metric_wrapper_handles_the_bound_class_signature(cap):
    """On the DeltaGenerator class the first positional is `self`."""
    _open_feature(cap, "brakes")
    wrapped = cap["_ax_wrap_metric"](lambda *a, **k: None)
    wrapped(_FakeDG(), "MC stroke", "9.3 mm")
    got = cap["get_feature_results"]("brakes")["metrics"]
    assert got[0]["label"] == "MC stroke", got


def test_alert_wrapper_captures_severity_and_renders(cap):
    _open_feature(cap, "brakes")
    seen = []
    wrapped = cap["_ax_wrap_alert"](lambda *a, **k: seen.append(a), "fail")
    wrapped("The pedal reaches the floor before lock-up.")
    assert seen
    got = cap["get_feature_results"]("brakes")["verdicts"]
    assert got[0]["severity"] == "fail"


def test_wrapper_never_breaks_the_render_on_capture_failure(cap):
    """If capture explodes, the widget must still draw."""
    _open_feature(cap, "brakes")

    def _boom(*a, **k):
        raise RuntimeError("capture blew up")

    cap["capture_metric"] = _boom
    rendered = []
    wrapped = cap["_ax_wrap_metric"](lambda *a, **k: rendered.append(1))
    wrapped("label", "value")
    assert rendered == [1]


# --------------------------------------------------------------------------- #
#  App-wide coverage
# --------------------------------------------------------------------------- #
def test_every_registered_feature_can_capture(cap):
    """No feature is structurally excluded from documentation."""
    missed = []
    for feat in cap["_FEATURE_SUBSYS"]:
        _open_feature(cap, feat)
        cap["capture_metric"]("probe", "1")
        if not cap["get_feature_results"](feat)["metrics"]:
            missed.append(feat)
    assert not missed, f"these features cannot capture results: {missed}"


def test_every_subsystem_rolls_up(cap):
    """Every owning subsystem must surface its features' captured work."""
    for feat in cap["_FEATURE_SUBSYS"]:
        _open_feature(cap, feat)
        cap["capture_metric"]("probe", f"value-{feat}")
    for subsys in set(cap["_FEATURE_SUBSYS"].values()):
        secs = cap["_captured_sections_for_subsystem"](subsys)
        assert secs, f"subsystem '{subsys}' rolls up nothing"


# --------------------------------------------------------------------------- #
#  Charts and tables
#
#  For a large share of features the real output is a plot or a table, not a
#  metric. Those documented as "Ran a calculation" and nothing else, which reads
#  as though no work happened. These pin that a chart's identity (title, axes,
#  series count) and a table's shape reach the report.
# --------------------------------------------------------------------------- #
class _FakeAxisTitle:
    def __init__(self, text):
        self.text = text


class _FakeAxis:
    def __init__(self, text):
        self.title = _FakeAxisTitle(text)


class _FakeLayout:
    def __init__(self, title, x, y):
        self.title = _FakeAxisTitle(title)
        self.xaxis = _FakeAxis(x)
        self.yaxis = _FakeAxis(y)


class _FakeFig:
    """Quacks like a plotly Figure for the summariser."""
    def __init__(self, title="", x="", y="", n_series=1):
        self.layout = _FakeLayout(title, x, y)
        self.data = tuple(range(n_series))


class _FakeFrame:
    """Quacks like a pandas DataFrame for the summariser."""
    def __init__(self, rows, cols):
        self.shape = (rows, len(cols))
        self.columns = list(cols)


def test_chart_title_and_shape_are_captured(cap):
    _open_feature(cap, "laptime")
    cap["capture_artifact"]("chart", "Lap time vs corner radius", "3 series")
    got = cap["get_feature_results"]("laptime")["artifacts"]
    assert got[0]["title"] == "Lap time vs corner radius"
    assert got[0]["detail"] == "3 series"


def test_plot_summary_reads_title_axes_and_series(cap):
    fig = _FakeFig("GG-V envelope", x="lateral g", y="longitudinal g",
                   n_series=4)
    title, shape = cap["_ax_plot_summary"](fig)
    assert title == "GG-V envelope"
    assert "longitudinal g vs lateral g" in shape
    assert "4 series" in shape


def test_plot_summary_survives_a_junk_figure(cap):
    for junk in (None, object(), {}, {"layout": {}}, 42):
        title, shape = cap["_ax_plot_summary"](junk)
        assert isinstance(title, str) and isinstance(shape, str)


def test_plot_summary_handles_a_dict_spec(cap):
    spec = {"layout": {"title": {"text": "Tyre curve"},
                       "xaxis": {"title": {"text": "slip angle"}},
                       "yaxis": {"title": {"text": "Fy"}}},
            "data": [1, 2]}
    title, shape = cap["_ax_plot_summary"](spec)
    assert title == "Tyre curve"
    assert "Fy vs slip angle" in shape and "2 series" in shape


def test_chart_wrapper_captures_and_still_renders(cap):
    _open_feature(cap, "aero")
    drawn = []
    wrapped = cap["_ax_wrap_chart"](lambda *a, **k: drawn.append(1))
    wrapped(_FakeFig("Downforce vs speed", "speed", "downforce", 2))
    assert drawn == [1]
    got = cap["get_feature_results"]("aero")["artifacts"]
    assert got[0]["title"] == "Downforce vs speed"
    assert "2 series" in got[0]["detail"]


def test_chart_wrapper_handles_the_bound_class_signature(cap):
    _open_feature(cap, "aero")
    wrapped = cap["_ax_wrap_chart"](lambda *a, **k: None)
    wrapped(_FakeDG(), _FakeFig("Cl vs ride height", "ride height", "Cl", 1))
    got = cap["get_feature_results"]("aero")["artifacts"]
    assert got and got[0]["title"] == "Cl vs ride height"


def test_untitled_chart_is_named_by_its_axes(cap):
    """"(untitled chart)" tells a reviewer nothing; "y vs x" describes it."""
    _open_feature(cap, "aero")
    wrapped = cap["_ax_wrap_chart"](lambda *a, **k: None)
    wrapped(_FakeFig("", "travel (mm)", "camber (deg)", 1))
    got = cap["get_feature_results"]("aero")["artifacts"]
    assert got, "an untitled plot is still evidence work was done"
    assert got[0]["title"] == "camber (deg) vs travel (mm)"
    assert "untitled" not in got[0]["title"].lower()


def test_wholly_unlabelled_chart_still_records(cap):
    _open_feature(cap, "aero")
    wrapped = cap["_ax_wrap_chart"](lambda *a, **k: None)
    wrapped(_FakeFig("", "", "", 3))
    got = cap["get_feature_results"]("aero")["artifacts"]
    assert got and "untitled" in got[0]["title"].lower()


def test_no_question_marks_leak_into_a_report(cap):
    """"? vs cost ($)" reads like a defect in a design-review document."""
    _, shape = cap["_ax_plot_summary"](_FakeFig("Cost by commodity", "", "cost ($)", 1))
    assert "?" not in shape, shape
    assert "cost ($)" in shape


def test_series_count_is_pluralised(cap):
    _, one = cap["_ax_plot_summary"](_FakeFig("t", "x", "y", 1))
    _, many = cap["_ax_plot_summary"](_FakeFig("t", "x", "y", 4))
    assert "1 series" in one and "4 series" in many


def test_table_rows_and_cols_are_pluralised(cap):
    """A report that says "1 rows" looks unproofed."""
    one = cap["_ax_table_summary"](_FakeFrame(1, ["a"]))
    many = cap["_ax_table_summary"](_FakeFrame(11, ["a", "b"]))
    assert "1 row x 1 col" in one, one
    assert "11 rows x 2 cols" in many, many


def test_table_shape_and_columns_are_captured(cap):
    _open_feature(cap, "cost")
    wrapped = cap["_ax_wrap_table"](lambda *a, **k: None, "table")
    wrapped(_FakeFrame(120, ["part", "qty", "unit cost", "total"]))
    got = cap["get_feature_results"]("cost")["artifacts"]
    assert "120 rows x 4 cols" in got[0]["title"]
    assert "part" in got[0]["title"]


def test_table_summary_truncates_a_wide_frame(cap):
    s = cap["_ax_table_summary"](_FakeFrame(5, [f"c{i}" for i in range(20)]))
    assert "5 rows x 20 cols" in s
    assert s.count(",") <= 5, "column list must not run away"
    assert "…" in s


def test_table_summary_handles_plain_python_data(cap):
    assert "3 rows" in cap["_ax_table_summary"]([{"a": 1}, {"a": 2}, {"a": 3}])
    assert "2 columns" in cap["_ax_table_summary"]({"a": [1], "b": [2]})
    assert cap["_ax_table_summary"](object()) == ""


def test_artifacts_dedupe_on_rerender(cap):
    _open_feature(cap, "laptime")
    for _ in range(6):
        cap["capture_artifact"]("chart", "Lap time trace", "1 series")
    assert len(cap["get_feature_results"]("laptime")["artifacts"]) == 1


def test_artifact_cap_holds(cap):
    _open_feature(cap, "laptime")
    for i in range(200):
        cap["capture_artifact"]("chart", f"plot {i}")
    assert len(cap["get_feature_results"]("laptime")["artifacts"]) \
        == cap["_MAX_CAPTURED_ARTIFACTS"]


def test_artifacts_are_gated_like_everything_else(cap):
    cap["_st"].session_state["_ax_rendering_tab"] = "aero"   # never opened
    cap["capture_artifact"]("chart", "Something nobody looked at")
    assert cap["get_feature_results"]("aero")["artifacts"] == []


def test_a_chart_only_feature_still_documents(cap):
    """The exact gap this closes: no metrics, no banners, just a plot."""
    _open_feature(cap, "laptime")
    wrapped = cap["_ax_wrap_chart"](lambda *a, **k: None)
    wrapped(_FakeFig("Lap time vs gear ratio", "gear ratio", "lap time", 3))
    secs = cap["_captured_result_sections"]("laptime")
    assert secs, "a chart-only feature must not document as empty"
    body = "\n".join(l for _, lines in secs for l in lines)
    assert "Lap time vs gear ratio" in body
    assert "3 series" in body


def test_artifact_section_comes_last(cap):
    _open_feature(cap, "brakes")
    cap["capture_metric"]("A", "1")
    cap["capture_verdict"]("A finding worth reporting here.", "ok")
    cap["capture_artifact"]("chart", "Some plot")
    heads = [h for h, _ in cap["_captured_result_sections"]("brakes")]
    assert "charts & tables" in heads[-1]


# --------------------------------------------------------------------------- #
#  Gate resilience
#
#  The gate accepts EITHER of two independent presence signals. Requiring only
#  the tab-open one would mean that if `.open` is ever unavailable (it is a
#  real-browser signal — AppTest reports None for every tab), nothing captures
#  anywhere and every document silently goes empty again.
# --------------------------------------------------------------------------- #
def test_engagement_alone_is_enough_to_capture(cap):
    ss = cap["_st"].session_state
    ss["_ax_rendering_tab"] = "tire"
    ss["_ax_engaged_tire"] = True          # touched a widget; never "opened"
    cap["capture_metric"]("Peak Fy", "1420 N")
    assert cap["get_feature_results"]("tire")["metrics"], (
        "engagement must gate capture on its own, or a deployment where the "
        "tab-open signal is unavailable documents nothing at all")


def test_open_alone_is_enough_to_capture(cap):
    ss = cap["_st"].session_state
    ss["_ax_rendering_tab"] = "tire"
    ss["_ax_open_tire"] = True             # opened; no widget touched yet
    cap["capture_metric"]("Peak Fy", "1420 N")
    assert cap["get_feature_results"]("tire")["metrics"]


def test_neither_signal_means_no_capture(cap):
    cap["_st"].session_state["_ax_rendering_tab"] = "tire"
    cap["capture_metric"]("Peak Fy", "1420 N")
    assert cap["get_feature_results"]("tire")["metrics"] == []


def test_no_double_status_glyph_in_a_report(cap):
    """App banners often already start with ✓/✗; a second mark looks unproofed."""
    _open_feature(cap, "accum")
    cap["capture_verdict"]("✓ System voltage < 600 V — pack tops out at 588 V.",
                           "ok")
    cap["capture_verdict"]("Under-cooled by 1485 W at the design delta-T.",
                           "warning")
    secs = cap["_captured_result_sections"]("accum")
    body = "\n".join(l for _, lines in secs for l in lines)
    assert "✅ ✓" not in body, body
    assert "- ✓ System voltage" in body, "the app's own glyph should survive"
    assert "⚠️ Under-cooled" in body, "an unmarked banner still gets a mark"


def test_marker_helper_respects_existing_glyphs(cap):
    assert cap["_mark_for"]("✓ already marked", "ok") == ""
    assert cap["_mark_for"]("✗ failed", "fail") == ""
    assert cap["_mark_for"]("plain text", "ok") == "✅"
    assert cap["_mark_for"]("", "warning") == "⚠️"
