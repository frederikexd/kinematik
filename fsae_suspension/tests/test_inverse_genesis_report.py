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
BOUNDS = "Solved properties the search bounded: the bound, the value delivered, the margin left"


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
    assert len(_tables(ns)) == len(st.drawn) == 21                     # 14, and the 7 of the parts screen


def test_every_table_drawn_is_reported_once_and_reruns_change_nothing(tab):
    st, ns, uig = tab
    _run(st, uig)
    first = {k: json.dumps(v) for k, v in _tables(ns).items()}
    assert len(first) == len(st.drawn) and all(st.drawn) and len(set(st.drawn)) == len(st.drawn)
    kin = _tables(ns)["Kinematics and anti-geometry, front corner"]
    act = _tables(ns)["Actuation and ride, front corner (solved from the linkage)"]
    assert (len(kin["rows"]), len(act["rows"])) == (16, 15)  # the pair that collided at 15 rows each
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
    assert review()["Bracket factor of safety, as drawn"][0] == "⚪ N/A"

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
    assert brk in _tables(ns) and row[0] != "⚪ N/A" and float(row[2]) == pytest.approx(float(_tables(ns)[brk]["rows"][0][5]))
    del st.editors["ig_brk"]                                  # the edit undone, with no other input changed
    _run(st, uig)
    assert brk not in _tables(ns) and review()["Bracket factor of safety, as drawn"][0] == "⚪ N/A"


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


def test_a_loaded_result_is_reported_without_the_tab_open_signal_or_after_a_stale_pause(tab):
    """In the hosted app a replayed result was on screen and the report said 'Nothing captured yet': capture waits for
    a signal that the user was in the tab, and that signal can be missing. A result the user made is that signal; and
    a capture pause left set by an interrupted run must not silence the tab."""
    st, ns, uig = tab
    ss = st.session_state
    del ss[f"_ax_open_{FEATURE}"]                             # no open signal at all
    _run(st, uig)
    assert _tables(ns) == {}                                  # without it, nothing is captured: the layer's own rule
    ss["ig_manifest_up"] = _Upload((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    _run(st, uig)
    _run(st, uig, clicks={"ig_manifest_load"})
    assert ss.get(f"_ax_engaged_{FEATURE}") is True
    assert "Kinematics and anti-geometry, front corner" in _tables(ns)
    ss["_ax_capture_paused"] = True                           # left behind by an interrupted run
    ns["get_feature_results"](FEATURE)["artifacts"].clear()
    _run(st, uig)
    assert "Kinematics and anti-geometry, front corner" in _tables(ns)


def test_a_loaded_manifest_is_reviewed_with_the_car_it_declared(tab):
    """front_v11_final's search bounded anti-dive at 63.6–70 % with its own car (brake bias 0.73); the review read the
    default car (0.60) and a generic 0–50 % range, so the same geometry showed 68.9 % passing in one table and 57.8 %
    failing in the next. Section 5 now takes the manifest's car, the review takes its bounds, and both tables print
    the one number."""
    st, ns, uig = tab
    _run(st, uig)
    st.session_state["ig_manifest_up"] = _Upload((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    _run(st, uig)
    _run(st, uig, clicks={"ig_manifest_load"})
    v = uig._veh(st.session_state)
    assert v["brake_bias_front"] == 0.73 and v["tube_wall_mm"] == 0.889 and v["yield_mpa"] == 435.0
    assert st.session_state["ig_v_brake_bias_front"] == 0.73          # the editor shows what the review reads
    t = _tables(ns)
    bound = next(r for r in t[BOUNDS]["rows"] if r[0].startswith("anti-dive"))
    review = next(r for k, x in t.items() if k.startswith("Design review") for r in x["rows"] if r[1].startswith("Anti"))
    kin = t["Kinematics and anti-geometry, front corner"]
    assert bound[2] == review[2] == _cell(kin, "anti-dive, from the contact patch's path (virtual work)") == "68.935"
    # The tangent row is labelled as the other form and is exactly tan × L/h × bias of the row above it, so a reader
    # who multiplies it out gets the number printed beside it, not a contradiction.
    tan = float(_cell(kin, "tan swing arm, referenced to the contact patch"))
    svic = float(_cell(kin, "anti-dive, side-view IC tangent form, for comparison"))
    assert svic == pytest.approx(100 * tan * 1630 / 280 * 0.73, rel=1e-3) and svic != 68.935
    prompt = [x for x in _verdicts(ns) if "Red is a prompt" in x]
    assert prompt and "these ranges are generic starting points (" in prompt[0]
    assert "Anti-dive" not in prompt[0] and "Steering ratio needed" in prompt[0]   # only the generic ranges are named
    assert review[0] == "🟢 PASS" and review[4] == "63.6 to 70"
    v_lines = _verdicts(ns)
    assert any("Section 5 set from the car `front_v11_final`" in x for x in v_lines)
    assert any("steering rack (81 mm per revolution" in x and "aligning moment" in x for x in v_lines)
    # the manifest's per-link tubes are what the strength, stretch and compliance tables use
    loads = t["Link forces and factor of safety, every load case (N, + tension)"]
    assert {r[1]: r[-1] for r in loads["rows"]} == {"UF": "15.875 × 0.889", "UR": "15.875 × 0.889",
                                                   "LF": "15.875 × 1.245", "LR": "15.875 × 1.651",
                                                   "TR": "19.05 × 2.108"}
    assert next(r for k, x in t.items() if k.startswith("Design review") for r in x["rows"]
                if r[1] == "Worst link factor of safety")[2] == "3.47"
    assert sum("Replayed from the recorded outputs" in x for x in v_lines) == 0   # said once, in the replay line


def test_manifest_vehicle_reads_only_what_was_declared():
    from suspension import genesis_repro as gr
    from ui import inverse_genesis as uig
    man = gr.GenesisManifest.from_json((ROOT / "manifests" / "table7_control.recorded.genesis.json").read_text())
    fields, missing, _tubes = uig.manifest_vehicle(man)
    props = (man.volume or {}).get("properties") or {}
    assert ("brake_bias_front" in fields) == (props.get("brake_bias_front") is not None)
    man.volume = {}
    assert uig.manifest_vehicle(man) == ({}, [], {})


def test_a_line_no_longer_drawn_leaves_the_report_even_when_nothing_changed(tab):
    """A session that outlived an app update kept the old build's 'ranges are generic' line beside the new one: the
    report was cleared only when the run or its inputs changed. Every render now starts the tab's report afresh."""
    st, ns, uig = tab
    _run(st, uig)
    ns["get_feature_results"](FEATURE)["verdicts"].append({"text": "a line an older build drew", "severity": "warning"})
    _run(st, uig)
    assert not any("older build" in x for x in _verdicts(ns))
    assert _tables(ns)                                           # the redraw itself is all there


def test_a_judge_can_see_where_every_input_came_from_and_what_the_result_does_not_show(tab):
    """The questions a design judge asks first, answered in the report itself: which inputs are this car's and which
    are the tool's defaults; whether the yield is measured; whether the search compared anything; whether it
    generated the geometry; how close the links run to the wheel; and whether a bound the team set itself agrees
    with the generic range."""
    st, ns, uig = tab
    _run(st, uig)
    st.session_state["ig_manifest_up"] = _Upload((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    _run(st, uig)
    _run(st, uig, clicks={"ig_manifest_load"})
    t, v = _tables(ns), _verdicts(ns)
    src = {r[0]: r[2] for r in t["Inputs to this review, and where each came from"]["rows"]}
    assert src["Front brake bias"] == "manifest" and src["Coil rate front (lb/in)"] == "default"
    assert src["Link tube LR, od × wall (mm)"] == "manifest" and "Coil rate rear (lb/in)" not in src
    n_default = sum(x == "default" for x in src.values())
    assert any(f"{n_default} of the {len(src)} inputs to this review are the tool's defaults" in x for x in v)
    assert any("priced against shop preset 'jig_weld'" in x and "Measure your own parts" in x for x in v)
    assert any("One candidate only: 1 of the 6 starts" in x for x in v)
    assert any("No pickup moved" in x and "did not generate it" in x for x in v)
    assert any("3.49 mm from the nearest link" in x and "3 mm required, 0.49 mm to spare" in x for x in v)
    assert any("Anti-dive / anti-squat: 68.935 % is inside the range the manifest declared (63.6 to 70) but outside "
               "the generic starting range (0 to 50)" in x for x in v)
    assert any("motion ratio falls in bump (0.8043 at ride height, 0.7509 at full bump)" in x for x in v)
    assert any("only in the kerb strike case" in x for x in v)
    review = {r[1]: r for k, x in t.items() if k.startswith("Design review") for r in x["rows"]}
    assert review["Wheel rate does not fall in bump"][:5] == ["🔴 FAIL", "Wheel rate does not fall in bump", "no", "", "must be yes"]
    steer = t["Steering effort at the design lateral acceleration"]
    with_mz, trail_only = float(_cell(steer, "ratio needed for the target")), float(
        _cell(steer, "ratio needed from trail alone, no aligning moment"))
    trail = float(_cell(steer, "mechanical trail, kingpin axis at the road"))
    assert trail == pytest.approx(12.667, abs=1e-3)             # kingpin meets the road ahead of the patch, not R·tan(caster)
    fy = float(_cell(steer, "outer Fy")) + 1.55 * float(_cell(steer, "inner front Fz"))
    assert trail_only == pytest.approx(fy * trail / 1000.0 / 10.0, abs=2e-3)
    assert with_mz == pytest.approx(trail_only + 56.0 / 10.0, abs=2e-3)
    points = [r[0] for r in next(x for k, x in t.items() if k.startswith("Hardpoints (mm): x rearward"))["rows"]]
    assert "rocker_axis (unit vector, not mm)" in points and "rocker_axis" not in points
    assert all(len(x) <= 402 for x in v)                       # nothing was cut by the report's banner limit

    st.session_state["ig_v_spring_front_lbin"] = 350.0        # the team enters its spring
    uig._veh(st.session_state)["spring_front_lbin"] = 350.0
    _run(st, uig)
    src = {r[0]: r[2] for r in _tables(ns)["Inputs to this review, and where each came from"]["rows"]}
    assert src["Coil rate front (lb/in)"] == "entered"
    assert any(f"{n_default - 1} of the {len(src)} inputs" in x for x in _verdicts(ns))


def test_result_caveats_and_self_graded_say_nothing_when_there_is_nothing_to_say():
    from types import SimpleNamespace as NS
    import numpy as np
    from ui import inverse_genesis as uig
    moved = NS(yield_frac=0.97, shift_vec=np.array([0.0, 2.5]))
    res = NS(winner=moved, candidates=[moved, moved], n_starts=6)
    man = NS(search=NS(n_starts=6))
    assert uig.result_caveats(res, man, NS(calibrated=True, provenance="inspection data")) == []
    assert uig.result_caveats(NS(winner=None, candidates=[], n_starts=0), man, None) == []
    still = NS(yield_frac=None, shift_vec=np.zeros(2))
    got = uig.result_caveats(NS(winner=still, candidates=[still], n_starts=1), man, None)
    assert len(got) == 2 and "1 of the 6 starts" in got[0] and "No pickup moved" in got[1]
    row = {"check": "Caster", "key": "caster_deg", "value": 3.656}
    generic = [{"check": "Caster", "lo": 2.0, "hi": 8.0}]
    assert uig.self_graded([row], {"Caster": (3.65, 8.0)}, generic) == []     # inside both ranges
    assert uig.self_graded([dict(row, value=9.0)], {"Caster": (3.65, 12.0)}, generic) == [(dict(row, value=9.0), 2.0, 8.0)]
    assert uig.self_graded([dict(row, value=9.0)], {}, generic) == []         # not a manifest range
    assert uig.self_graded([dict(row, value=None)], {"Caster": (3.65, 8.0)}, generic) == []


def test_the_wheel_envelope_line_says_what_the_clearance_is_measured_beyond(tab):
    """0.49 mm 'worst clearance' read as half a millimetre from the rim; it was the margin beyond a 3 mm gap."""
    from types import SimpleNamespace as NS
    st, ns, uig = tab
    lines = []
    fake = NS(success=lambda t: lines.append(("ok", t)), warning=lambda t: lines.append(("warn", t)),
              error=lambda t: lines.append(("fail", t)))

    def line(margin, gap):
        lines.clear()
        src = (ROOT / "ui" / "inverse_genesis.py").read_text(encoding="utf-8")
        a = src.index("        m = res.winner.envelope_margin_mm")
        b = src.index("    if res.winner_hp is None:", a)
        import textwrap
        exec(textwrap.dedent(src[a:b]), {"res": NS(winner=NS(envelope_margin_mm=margin),
                                                    wheel_envelope=NS(clearance_mm=gap, rack_travel_mm=27.0)),
                                         "st": fake, "_cell": uig._cell,
                                         "_MIN_RUNNING_CLEARANCE_MM": uig._MIN_RUNNING_CLEARANCE_MM})
        return lines[0]
    assert line(0.49, 3.0)[0] == "ok" and "3 mm required, 0.49 mm to spare" in line(0.49, 3.0)[1]
    assert line(0.49, 0.0)[0] == "warn" and "no running clearance was declared" in line(0.49, 0.0)[1]
    assert line(6.0, 0.0)[0] == "ok"
    assert line(-0.2, 3.0)[0] == "fail" and "closer to it than the 3 mm required" in line(-0.2, 3.0)[1]


def test_the_report_closes_the_loops_a_judge_would_try_to_close(tab):
    """Second reading as a judge: every table is named (the hardpoints with their frame and units); a bound prints
    as a number, not a label cut off before it; the car's own steering ratio is there to compare the 'ratio needed'
    with; the two link factors and two compliance numbers in one report are explained; the scope is stated; and one
    candidate is not called 'the most buildable'."""
    st, ns, uig = tab
    _run(st, uig)
    st.session_state["ig_manifest_up"] = _Upload((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    _run(st, uig)
    _run(st, uig, clicks={"ig_manifest_load"})
    t, v = _tables(ns), _verdicts(ns)
    assert not [k for k in t if " rows x " in k or " row x " in k]             # no table known only by its shape
    assert any(k.startswith("Hardpoints (mm): x rearward of the wheel centre, y outboard of the centreline, z above ground")
               for k in t)
    bounds = {r[0]: r[1] for r in t[BOUNDS]["rows"]}
    assert bounds["anti-dive, front (%)"] == "63.6 to 70" and bounds["caster (deg)"] == "3.65 to 8"
    assert bounds["lowest link buckling/yield factor under the shock cases (-)"] == "≥ 2"
    assert bounds["steering-wheel angle to reach lock (deg)"] == "≤ 120"
    steer = t["Steering effort at the design lateral acceleration"]
    ratio = float(_cell(steer, "steering ratio of the declared rack, at lock"))
    assert ratio == pytest.approx(118.9 / 25.6, abs=2e-3)
    trail_total = (float(_cell(steer, "outer Fy")) + 1.55 * float(_cell(steer, "inner front Fz"))) \
        * float(_cell(steer, "mechanical trail, kingpin axis at the road")) / 1000.0    # N·m, trail at the road
    assert float(_cell(steer, "steering-wheel torque at the rack's ratio, trail alone")) == pytest.approx(trail_total / ratio, abs=5e-3)
    assert float(_cell(steer, "steering-wheel torque at the rack's ratio, with the declared aligning moments")) == \
        pytest.approx((trail_total + 56.0) / ratio, abs=5e-3)
    assert any("its own structural load cases (4, such as braking 1.8 g) and pickup flexibility" in x for x in v)
    assert any(x.startswith("- ⚠️ Scope: one corner, solved as rigid links") for x in v)
    assert any("The curves are reached inside every declared wall, and the geometry is RESILIENT" in x for x in v)
    assert not any("the best fit is also the most buildable" in x for x in v)
    assert all(len(x) <= 402 for x in v)


def test_bound_text():
    from ui import inverse_genesis as uig
    inf = float("inf")
    assert [uig.bound_text(lo, hi) for lo, hi in ((63.6, 70.0), (2.0, inf), (None, 120.0), (-inf, 120.0), (None, None))] \
        == ["63.6 to 70", "≥ 2", "≤ 120", "≤ 120", "unbounded"]


def test_the_build_stamp_changes_when_any_source_file_does(tmp_path):
    """Six updates to ui/ and suspension/ changed what a report said while its header kept one build stamp: it hashed
    streamlit_app.py alone."""
    src = APP.read_text(encoding="utf-8")
    a = src.index("def _source_fingerprint(root):")
    ns = {"os": os}
    exec(src[a:src.index("def _build_stamp():", a)], ns)
    (tmp_path / "ui").mkdir()
    (tmp_path / "suspension").mkdir()
    (tmp_path / "streamlit_app.py").write_text("app")
    (tmp_path / "ui" / "tab.py").write_text("x = 1")
    (tmp_path / "suspension" / "core.py").write_text("y = 1")
    (tmp_path / "ui" / "notes.txt").write_text("not source")
    one = ns["_source_fingerprint"](str(tmp_path))
    assert len(one) == 8 and ns["_source_fingerprint"](str(tmp_path)) == one
    (tmp_path / "ui" / "notes.txt").write_text("still not source")
    assert ns["_source_fingerprint"](str(tmp_path)) == one
    (tmp_path / "ui" / "tab.py").write_text("x = 2")
    two = ns["_source_fingerprint"](str(tmp_path))
    (tmp_path / "suspension" / "core.py").write_text("y = 2")
    assert len({one, two, ns["_source_fingerprint"](str(tmp_path))}) == 3


def test_a_green_check_says_which_defaults_it_rests_on_and_thin_bounds_are_named(tab):
    """Third reading as a judge: 'static clearance 31.5 mm' and 'wheelbase margin 105 mm' were green from the tool's
    default axle stations and ground plane; the 100 % yield said nothing about caster at 0.006° from its bound; ride
    frequencies left out the tyre."""
    st, ns, uig = tab
    _run(st, uig)
    st.session_state["ig_manifest_up"] = _Upload((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    _run(st, uig)
    _run(st, uig, clicks={"ig_manifest_load"})
    t, v = _tables(ns), _verdicts(ns)
    review = next(x for k, x in t.items() if k.startswith("Design review"))
    col = review["header"].index("rests on defaults")
    rests = {r[1]: r[col] for r in review["rows"]}
    assert rests["Static clearance under frame"] == "ground plane, lowest frame member"
    assert rests["Wheelbase margin to rules"] == "axle stations"
    assert rests["Damper stroke used"] == "damper stroke"
    assert rests["Camber gain"] == "" and rests["Anti-dive / anti-squat"] == ""          # geometry and manifest only
    assert any("rest on the tool's defaults" in x and "Static clearance under frame" in x for x in v)
    thin = next(x for x in v if x.startswith("- ⚠️ Build yield prices the camber, toe and roll-centre curves only"))
    assert "caster 0.006 deg" in thin and "Ackermann at the declared lock 0.273 %" in thin and len(thin) <= 402
    ride = [r[0] for r in t["Actuation and ride, front corner (solved from the linkage)"]["rows"] if r[0].startswith("ride")]
    assert ride and all("no tyre in series" in r for r in ride)

    for f in ("ground_y_mm", "lowest_member_y_mm"):                       # the team declares its frame
        uig._veh(st.session_state)[f] = uig._veh(st.session_state)[f] + 1.0
    _run(st, uig)
    review = next(x for k, x in _tables(ns).items() if k.startswith("Design review"))
    assert {r[1]: r[col] for r in review["rows"]}["Static clearance under frame"] == ""


def test_as_built_unpriced_and_defaults_behind():
    from types import SimpleNamespace as NS
    from ui import inverse_genesis as uig
    inf = float("inf")
    bounds = [NS(prop="caster_deg", lo=3.65, hi=8.0, margin=lambda x: min(x - 3.65, 8.0 - x)),
              NS(prop="kpi_deg", lo=0.0, hi=10.0, margin=lambda x: min(x, 10.0 - x)),
              NS(prop="link_fos_min", lo=2.0, hi=inf, margin=lambda x: x - 2.0)]
    res = NS(winner=NS(properties={"caster_deg": 3.656, "kpi_deg": 7.3, "link_fos_min": 2.05}),
             property_bounds=NS(bounds=bounds))
    got = dict(uig.as_built_unpriced(res))
    assert set(got) == {"caster (deg)", "lowest link buckling/yield factor under the shock cases (-)"}
    assert uig.as_built_unpriced(NS(winner=None, property_bounds=None)) == []
    rows = [{"key": "stroke_used"}, {"key": "camber_gain_deg_per_mm"}]
    srcs = [{"input": "Damper stroke (mm)", "source": "default"}]
    assert uig.defaults_behind(rows, srcs) == {"stroke_used": "damper stroke"}
    assert uig.defaults_behind(rows, [{"input": "Damper stroke (mm)", "source": "manifest"}]) == {}


def test_the_pdf_keeps_every_number_whole_and_every_status_readable(tab, tmp_path):
    """The PDF the judge reads, not the markdown: its font has no colour emoji, so the review's status column printed
    empty; and narrow columns broke numbers across lines ('3.656' as '3.65' over '6')."""
    pytest.importorskip("reportlab")
    pypdf = pytest.importorskip("pypdf")
    from suspension import project
    st, ns, uig = tab
    _run(st, uig)
    st.session_state["ig_manifest_up"] = _Upload((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    _run(st, uig)
    _run(st, uig, clicks={"ig_manifest_load"})
    out = tmp_path / "r.pdf"
    project.render_pdf("# Sample\n\n" + _report(ns), str(out))
    text = "\n".join(p.extract_text() for p in pypdf.PdfReader(str(out)).pages)
    lines = [ln.strip() for ln in text.splitlines()]
    for status in ("FAIL", "WATCH", "PASS", "N/A"):
        assert any(ln.startswith(status) or f" {status} " in f" {ln} " for ln in lines), status
    for number in ("3.656", "68.935", "0.7024", "6.21e-05", "52.813", "1.282", "17.159"):
        assert number in text, number                         # never split across two lines


def test_the_judge_summary_is_built_from_the_review_with_the_teams_reasons_and_its_figure(tab, monkeypatch, tmp_path):
    """The Design Event summary from a loaded result: the yes/no checks read as yes/no, a written reason replaces the
    gap, an answer follows its question, and the PDF carries the curves figure. Nothing of it enters the report."""
    from suspension import judge_summary as js
    st, ns, uig = tab
    got = {}
    real = uig.judge_summary_inputs

    def spy(*a, **k):
        got["kw"], got["fig"] = r = real(*a, **k)
        return r
    monkeypatch.setattr(uig, "judge_summary_inputs", spy)
    _run(st, uig)
    st.session_state["ig_manifest_up"] = _Upload((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    _run(st, uig)
    _run(st, uig, clicks={"ig_manifest_load"})
    before = _report(ns)
    st.session_state["ig_rat_anti_dive_why"] = "Front holds the rear's peak mu at 73 % bias."
    st.session_state[js.answer_key("Caster: x")] = "Shimmed on the built car."
    _run(st, uig)
    assert _report(ns) == before                                          # the panel adds nothing to the report
    kw, fig = got["kw"], got["fig"]
    pts = [q for q, _ in kw["questions"]]
    assert "Wheel rate does not fall in bump: no" in pts
    md = js.build(**kw)
    assert "Front holds the rear's peak mu at 73 % bias." in md and "Team answer: Shimmed on the built car." in md
    # one level up the V: the targets derived from the car's requirements, not left to the team to reconstruct
    assert kw["target_chain"] and [r[0] for r in kw["target_chain"]][:3] == [
        "Design lateral acceleration", "Camber gain", "Camber gain delivered"]
    assert "| Camber gain |" in md
    assert "| Ackermann at the declared lock (%) | ≥ 75 | 75.27 | yes |" in md
    assert "**Where the link loads go**" in md and "| Upright arm to the lower ball joint | bending at the root |" in md
    assert "| Link weld, LF | fatigue, IIW FAT 71 |" in md and "Rod-end shank" in md and "Hub bearing" in md
    assert "Fatigue beyond the infinite-life screen" in md
    pytest.importorskip("matplotlib")
    assert fig[:8] == b"\x89PNG\r\n\x1a\n"
    pytest.importorskip("reportlab")
    pypdf = pytest.importorskip("pypdf")
    from suspension import project
    out = tmp_path / "j.pdf"
    project.render_pdf(md, str(out), figures={"judge_curves": fig})
    reader = pypdf.PdfReader(str(out))
    assert any(p.images for p in reader.pages)
    text = "\n".join(p.extract_text() for p in reader.pages)
    assert "Why this design, and from what alternatives" in text and "75.27" in text
