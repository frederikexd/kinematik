# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The InverseGenesis tab's report, end to end: the real ``render()`` driven over several script runs through a
stand-in Streamlit, with the shipped capture layer exec'd out of streamlit_app.py around its outputs and the report
built by the shipped section builder. Only the widgets are stood in for; no search is run, so this is fast.

Each test is a sequence a user can perform, and each failed before the fix it pins:
a manifest loaded and not run exported its recorded verdict above the seed geometry's tables; a warning that had
been cleared stayed in the report; an example block that had been reported stayed after its values were set back."""
import datetime
import json
import math
import os
import re
import sys
import time
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "streamlit_app.py"
FEATURE = "genesis"
NOTICE = "loaded but has not been run"


class Rerun(Exception):
    pass


class Box:
    """A container / column / expander / spinner: a context manager whose widget calls go to the stand-in."""
    def __init__(self, st):
        self._st = st

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def __getattr__(self, name):
        return getattr(self._st, name)


class St(types.ModuleType):
    def __init__(self):
        super().__init__("streamlit")
        self.session_state = {}
        self.clicks = set()
        self.unknown = set()
        self.editors = {}                      # key -> function(df) -> df, for st.data_editor

    # -- widget state, as Streamlit keeps it ------------------------------
    def _w(self, key, default):
        if key is None:
            return default
        if key not in self.session_state:
            self.session_state[key] = default
        return self.session_state[key]

    def number_input(self, label, *a, key=None, **k):
        value = k.get("value", a[2] if len(a) > 2 else (a[0] if a else 0.0))
        return self._w(key, value)

    def text_input(self, label, value="", key=None, **k):
        return self._w(key, value)
    text_area = text_input

    def checkbox(self, label, value=False, key=None, **k):
        return self._w(key, value)
    toggle = checkbox

    def radio(self, label, options, index=0, key=None, **k):
        return self._w(key, list(options)[index or 0])
    selectbox = radio

    def multiselect(self, label, options, default=None, key=None, **k):
        return self._w(key, list(default) if default is not None else [])

    def button(self, label, key=None, **k):
        return (key in self.clicks) and not k.get("disabled")

    def download_button(self, *a, **k):
        return False

    def file_uploader(self, label, type=None, key=None, **k):
        return self.session_state.get(key)

    def data_editor(self, df, key=None, **k):
        f = self.editors.get(key)
        return f(df) if f else df

    # -- layout -----------------------------------------------------------
    def columns(self, spec, **k):
        return [Box(self) for _ in range(spec if isinstance(spec, int) else len(spec))]

    def tabs(self, names):
        return [Box(self) for _ in names]

    def expander(self, *a, **k):
        return Box(self)
    container = spinner = popover = form = expander

    def rerun(self):
        raise Rerun()

    # -- output: no-ops (the capture wrappers go around these) --------------
    def _noop(self, *a, **k):
        return None
    caption = markdown = subheader = info = success = warning = error = divider = json = _noop
    dataframe = table = plotly_chart = line_chart = metric = progress = write = _noop

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        if name == "column_config":
            return _Any()
        self.unknown.add(name)
        return lambda *a, **k: Box(self)


class _Any:
    def __getattr__(self, name):
        return lambda *a, **k: None


class _Upload:
    def __init__(self, text):
        self.text = text

    def getvalue(self):
        return self.text.encode()


@pytest.fixture()
def tab(monkeypatch):
    """(st, ns, uig): the stand-in with the shipped capture wrappers on its outputs, the capture namespace, the tab."""
    if not APP.exists():
        pytest.skip("streamlit_app.py not present in this checkout")
    st = St()
    monkeypatch.setitem(sys.modules, "streamlit", st)
    src = APP.read_text(encoding="utf-8")

    def cut(a, b):
        i = src.index(a)
        return src[i:src.index(b, i)]

    ns = {"st": st, "DeltaGenerator": Box, "re": re, "os": os, "json": json, "math": math, "time": time,
          "types": types, "datetime": datetime}
    exec(compile(cut("_FEATURE_RESULTS_KEY = ", "def _ax_positional_args("), "<capture-core>", "exec"), ns)
    exec(compile(cut("def _ax_positional_args(args):", 'if not getattr(st, "_ax_capture_patched", False):'),
                 "<capture-wrappers>", "exec"), ns)
    exec(compile(cut("_FEATURE_SUBSYS = {", "def _feature_subsys("), "<registry>", "exec"), ns)
    ns.setdefault("_TAB_META", {})
    ns.setdefault("_feature_label", lambda f: str(f).title())
    exec(compile(cut("_SEVERITY_MARK = ", "def _captured_sections_for_subsystem("), "<sections>", "exec"), ns)

    for name, sev in (("success", "ok"), ("warning", "warning"), ("error", "fail")):     # as streamlit_app patches st
        setattr(st, name, ns["_ax_wrap_alert"](getattr(st, name), sev))
    st.plotly_chart = ns["_ax_wrap_chart"](st.plotly_chart)
    captured = ns["_ax_wrap_table"](st.dataframe, "table")
    st.drawn = []

    def counted(*a, **k):                                     # every table drawn while capture is live
        if not st.session_state.get("_ax_capture_paused"):
            st.drawn.append(st.session_state.get("_ax_next_heading"))
        return captured(*a, **k)
    st.dataframe = counted
    st.session_state["_ax_rendering_tab"] = FEATURE
    st.session_state[f"_ax_open_{FEATURE}"] = True
    from ui import inverse_genesis as uig
    return st, ns, uig


def _run(st, uig, clicks=()):
    """One script run of the tab; st.rerun() starts it again, as Streamlit does."""
    st.clicks = set(clicks)
    for _ in range(5):
        st.drawn = []
        try:
            return uig.render()
        except Rerun:
            st.clicks = set()
    raise RuntimeError("rerun loop")


def _report(ns):
    return "\n".join(line for head, lines in ns["_captured_result_sections"](FEATURE) for line in ["## " + head, *lines])


def _tables(ns):
    return {a.get("heading") or a["title"]: a.get("table") for a in ns["get_feature_results"](FEATURE)["artifacts"]}


def _verdicts(ns):
    return [line for line in _report(ns).splitlines() if line.startswith("- ")]


def _cell(tbl, row_label, col=1):
    return next(r[col] for r in tbl["rows"] if r[0] == row_label)


def test_a_manifest_that_is_loaded_but_not_run_reports_nothing_but_that(tab):
    st, ns, uig = tab
    _run(st, uig)
    assert any("No generated geometry yet" in v for v in _verdicts(ns))                 # the seed is called the seed
    assert _cell(_tables(ns)["Kinematics and anti-geometry, front corner"], "camber gain, LSQ") == "-0.0182"

    text = (ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text()
    st.session_state["ig_manifest_up"] = _Upload(text)
    for _ in range(2):                                        # the load, then any rerun (opening another tab)
        _run(st, uig)
        rep = _report(ns)
        assert _tables(ns) == {} and len(_verdicts(ns)) == 1 and NOTICE in _verdicts(ns)[0]
        assert "RESILIENT" not in rep and "Recorded result" not in rep and "-0.0182" not in rep
    assert "_ax_capture_paused" not in st.session_state

    del st.session_state["ig_manifest_up"]                    # file removed: back to the seed, said to be the seed
    _run(st, uig)
    assert NOTICE not in _report(ns) and any("No generated geometry yet" in v for v in _verdicts(ns))
    assert len(_tables(ns)) == len(st.drawn) == 13


def test_every_table_drawn_is_reported_once_and_reruns_change_nothing(tab):
    st, ns, uig = tab
    _run(st, uig)
    first = {k: json.dumps(v) for k, v in _tables(ns).items()}
    assert len(first) == len(st.drawn) and all(st.drawn) and len(set(st.drawn)) == len(st.drawn)
    kin = _tables(ns)["Kinematics and anti-geometry, front corner"]
    act = _tables(ns)["Actuation and ride, front corner (solved from the linkage)"]
    assert len(kin["rows"]) == len(act["rows"]) == 15         # the pair that used to be one row in the store
    assert kin["rows"][0][0] == "camber gain, LSQ" and act["rows"][0][0] == "motion ratio, static"
    for _ in range(4):
        _run(st, uig)
        assert {k: json.dumps(v) for k, v in _tables(ns).items()} == first
    assert list(first)[-1].startswith("Design review")


def test_a_warning_that_has_been_cleared_leaves_the_report(tab):
    st, ns, uig = tab
    _run(st, uig)
    v = uig._veh(st.session_state)
    wheelbase, od = v["wheelbase_mm"], v["tube_od_mm"]
    v["wheelbase_mm"] = wheelbase + 70.0
    _run(st, uig)
    assert any("declared wheelbase differs" in x for x in _verdicts(ns))
    v["wheelbase_mm"] = wheelbase
    _run(st, uig)
    assert not any("declared wheelbase differs" in x for x in _verdicts(ns))

    v["tube_od_mm"] = 6.0
    _run(st, uig)
    gov = [x for x in _verdicts(ns) if "Governing member" in x]
    assert len(gov) == 1 and gov[0].startswith("- ❌")
    v["tube_od_mm"] = od
    _run(st, uig)
    gov = [x for x in _verdicts(ns) if "Governing member" in x]
    assert len(gov) == 1 and gov[0].startswith("- ✅")        # not a ✅ and a ❌ for one member in one report


def test_an_example_block_is_reported_only_while_it_holds_the_users_own_values(tab):
    st, ns, uig = tab
    ss = st.session_state
    link, bands, brk = uig._STATED_LINK_TITLE, uig._STATED_BANDS_TITLE, uig._BRACKET_TITLE
    review = lambda: {r[1]: r for r in next(t for k, t in _tables(ns).items()             # noqa: E731
                                            if k.startswith("Design review"))["rows"]}
    _run(st, uig)
    assert not {link, bands, brk} & set(_tables(ns)) and "4799" not in _report(ns)
    assert review()["Bracket factor of safety, as drawn"][0] == "⚪"

    ss["ig_cmp_F"] = 3200.0
    _run(st, uig)
    assert _cell(_tables(ns)[link], "stated link force") == "3200" and bands in _tables(ns) and brk not in _tables(ns)
    ss["ig_cmp_F"] = uig._STATED_EXAMPLE[0]                   # set back: it is the example again
    _run(st, uig)
    assert link not in _tables(ns) and bands not in _tables(ns)

    def shorter(df):
        df = df.copy()
        df.loc[0, "reach (mm)"] = 12.0
        return df
    st.editors["ig_brk"] = shorter
    _run(st, uig)
    row = review()["Bracket factor of safety, as drawn"]
    assert brk in _tables(ns) and row[0] != "⚪" and float(row[2]) == pytest.approx(float(_tables(ns)[brk]["rows"][0][5]))
    del st.editors["ig_brk"]                                  # the edit undone, with no other input changed
    _run(st, uig)
    assert brk not in _tables(ns) and review()["Bracket factor of safety, as drawn"][0] == "⚪"


def test_loading_the_recorded_result_reports_the_replay_and_never_a_re_sample(tab):
    st, ns, uig = tab
    text = (ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text()
    _run(st, uig)
    st.session_state["ig_manifest_up"] = _Upload(text)
    _run(st, uig)
    _run(st, uig, clicks={"ig_manifest_load"})
    rep, v = _report(ns), _verdicts(ns)
    assert st.session_state["ig_run"]["replayed"] is True and st.session_state["ig_run"]["verify"][0] is True
    assert sum("Replay of `front_v11_final` (RESILIENT)" in x for x in v) == 1
    assert "not re-sampled" in rep and "yields to 1e-9" not in rep and NOTICE not in rep
    assert "Quick search" not in rep                          # the recorded yield is the full N = 2000 one
    assert _cell(_tables(ns)["Kinematics and anti-geometry, front corner"], "camber gain, LSQ") == "-0.035"
    st.session_state["ig_run"] = dict(st.session_state["ig_run"], replayed=False)   # as if then fully re-run
    _run(st, uig)
    assert not any("Replay of" in x for x in _verdicts(ns))   # the replay line does not outlive the replay


def test_the_hosted_form_opens_as_a_quick_search(tab, monkeypatch):
    st, ns, uig = tab
    monkeypatch.setenv("KINEMATIK_HOSTED", "1")
    _run(st, uig)
    assert (st.session_state["ig_nstarts"], st.session_state["ig_nyield"]) == (uig.HOSTED_N_STARTS, uig.HOSTED_N_YIELD)
    monkeypatch.setenv("KINEMATIK_HOSTED", "0")
    st.session_state.pop("ig_nyield")
    st.session_state.pop("ig_nstarts")
    _run(st, uig)
    assert (st.session_state["ig_nstarts"], st.session_state["ig_nyield"]) == (6, 4000)


def test_the_hosted_quick_search_finishes_in_seconds_and_says_it_is_quick(tab, monkeypatch):
    """The hosted defaults, run through the tab's own Generate button on its default corner: a result, in well
    under the time a hosted app allows, labelled quick in the report."""
    import time
    st, ns, uig = tab
    monkeypatch.setenv("KINEMATIK_HOSTED", "1")
    _run(st, uig)
    t = time.perf_counter()
    _run(st, uig, clicks={"ig_run_btn"})
    took = time.perf_counter() - t
    run = st.session_state.get("ig_run")
    assert run is not None and took < 60, took
    assert run["result"].winner is not None                   # the default corner gets a graded winner
    assert any("Quick search: 1 start (the seed geometry only)" in x for x in _verdicts(ns))


def test_the_form_run_completes(tab):
    """🧬 Generate from the form: the RC-height field once shadowed the report module and crashed every form run."""
    st, ns, uig = tab
    st.session_state["ig_nstarts"] = 1
    _run(st, uig)
    _run(st, uig, clicks={"ig_run_btn"})
    assert st.session_state.get("ig_run") is not None
    assert not any("No generated geometry yet" in x for x in _verdicts(ns))
