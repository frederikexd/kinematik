# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""What InverseGenesis tells the report about what it shows (suspension/report_capture.py, ui/inverse_genesis.py).

A feature report exported with a manifest loaded but not run printed the manifest's recorded RESILIENT verdict above
tables computed from the form's seed geometry; printed the example link (4799 N, 485 mm) and the example bracket
(FoS 0.92 "as drawn") as results; and printed a motion-ratio table where the kinematics table belonged, because both
had 15 rows. Each of those is pinned here."""
from pathlib import Path

import pandas as pd
import pytest

from suspension import genesis_analysis as ga
from suspension import report_capture as rc
from suspension.kinematics import Hardpoints
from ui import inverse_genesis as uig

APP = Path(__file__).resolve().parents[1] / "streamlit_app.py"


class _St:
    """Just enough of streamlit: widgets return their defaults (or a forced value by key), and every table records
    its heading and whether capture was paused when it was drawn."""
    def __init__(self, force=None, edit=None):
        self.session_state, self.force, self.edit, self.tables, self.notes = {}, force or {}, edit, [], []

    def number_input(self, label, lo, hi, value, step, key=None, **k):
        return self.force.get(key, value)

    def columns(self, n):
        return [self] * n

    def data_editor(self, df, **k):
        return self.edit(df) if self.edit else df

    def dataframe(self, df, **k):
        self.tables.append({"heading": self.session_state.pop(rc.HEADING_KEY, None),
                            "paused": bool(self.session_state.get(rc.PAUSED_KEY)), "df": df})

    def caption(self, t, **k):
        self.notes.append(t)
    markdown = warning = success = error = info = caption

    def line_chart(self, *a, **k):
        pass


# ---- the bookkeeping --------------------------------------------------------
def test_the_app_and_the_tabs_use_the_same_session_keys():
    if not APP.exists():
        pytest.skip("streamlit_app.py not present in this checkout")
    src = APP.read_text(encoding="utf-8")
    assert f'_CAPTURE_PAUSED_KEY = "{rc.PAUSED_KEY}"' in src
    assert f'_CAPTURE_HEADING_KEY = "{rc.HEADING_KEY}"' in src
    assert f'_FEATURE_RESULTS_KEY = "{rc.RESULTS_KEY}"' in src
    assert f'st.session_state.get("{rc.RENDERING_KEY}")' in src


def test_unreported_pauses_nests_and_restores_even_on_an_exception():
    ss = {}
    with rc.unreported(ss):
        assert ss[rc.PAUSED_KEY] is True
        with rc.unreported(ss):
            assert ss[rc.PAUSED_KEY] is True
        assert ss[rc.PAUSED_KEY] is True                       # the inner block does not un-pause the outer
    assert rc.PAUSED_KEY not in ss
    with pytest.raises(RuntimeError):
        with rc.unreported(ss):
            raise RuntimeError("streamlit rerun")
    assert rc.PAUSED_KEY not in ss
    with rc.unreported(ss, False):
        assert rc.PAUSED_KEY not in ss                         # inactive: reported as usual


def test_reset_on_change_drops_only_the_rendering_tab_and_only_when_the_identity_changes():
    ss = {rc.RENDERING_KEY: "genesis",
          rc.RESULTS_KEY: {"genesis": {"artifacts": [1]}, "brakes": {"artifacts": [2]}}}
    assert rc.reset_on_change(ss, "id", "seed") is True
    assert ss[rc.RESULTS_KEY] == {"brakes": {"artifacts": [2]}}
    ss[rc.RESULTS_KEY]["genesis"] = {"artifacts": [3]}
    assert rc.reset_on_change(ss, "id", "seed") is False and "genesis" in ss[rc.RESULTS_KEY]
    assert rc.reset_on_change(ss, "id", "run abc") is True and "genesis" not in ss[rc.RESULTS_KEY]
    rc.reset({})                                               # nothing rendering, nothing stored: no error
    rc.heading(ss, "")
    assert rc.HEADING_KEY not in ss


def test_forget_drops_only_the_named_tables_of_the_rendering_tab():
    arts = [{"kind": "table", "heading": "A"}, {"kind": "table", "heading": "B"}, {"kind": "table", "title": "3 rows"}]
    ss = {rc.RENDERING_KEY: "genesis", rc.RESULTS_KEY: {"genesis": {"artifacts": list(arts)},
                                                        "brakes": {"artifacts": [{"kind": "table", "heading": "A"}]}}}
    rc.forget(ss, "A", "not there")
    assert ss[rc.RESULTS_KEY]["genesis"]["artifacts"] == arts[1:]
    assert ss[rc.RESULTS_KEY]["brakes"]["artifacts"] == [{"kind": "table", "heading": "A"}]
    rc.forget(ss, "A", feature="brakes")
    assert ss[rc.RESULTS_KEY]["brakes"]["artifacts"] == []
    rc.forget({}, "A")                                         # nothing rendering, nothing stored: no error
    rc.forget({rc.RENDERING_KEY: "genesis"}, "A")


# ---- a manifest that is loaded but not run ----------------------------------
class _Upload:
    def __init__(self, text):
        self.text = text

    def getvalue(self):
        return self.text.encode()


def _manifest_json():
    return (Path(__file__).resolve().parents[1] / "manifests" / "front_v11_final.recorded.genesis.json").read_text()


def test_a_loaded_manifest_is_pending_until_that_manifest_has_been_run():
    from suspension import genesis_repro as gr
    text = _manifest_json()
    sha = gr.GenesisManifest.from_json(text).inputs_sha256
    assert uig.manifest_pending({}) is False
    ss = {"ig_manifest_up": _Upload(text)}
    assert uig.manifest_pending(ss) is True                                    # loaded, never run
    ss["ig_run"] = {"source": "manifest", "inputs_sha256": "0" * 64}
    assert uig.manifest_pending(ss) is True                                    # another manifest's run
    ss["ig_run"] = {"inputs_sha256": sha}
    assert uig.manifest_pending(ss) is True                                    # a form run, not the manifest's
    ss["ig_run"] = {"source": "manifest", "inputs_sha256": sha}
    assert uig.manifest_pending(ss) is False
    assert uig.manifest_pending({"ig_manifest_up": _Upload("not json"), "ig_run": ss["ig_run"]}) is True


def test_the_report_identity_follows_the_run():
    what = lambda ss, pending=False: uig.report_identity(ss, pending).split(" | ")[0]    # noqa: E731
    assert what({}) == "seed"
    assert what({"ig_run": {"inputs_sha256": "abc"}}) == "run abc"
    assert uig.report_identity({"ig_run": {"inputs_sha256": "abc"}}, True) == "manifest loaded, not run"
    old = {"ig_run": {"manifest_json": _manifest_json()}}                      # a run stored before the hash was
    assert what(old).startswith("run ") and len(what(old)) == 68
    assert what({"ig_run": {"manifest_json": "{"}}) == "run unreadable"


def test_the_report_identity_follows_every_declaration_the_analysis_reads():
    """A warning that no longer applies is not drawn again, so nothing replaces it: the identity has to change when
    an input does, or the report keeps 'the declared wheelbase differs' beside a wheelbase that no longer does."""
    ss = {}
    a = uig.report_identity(ss, False)
    assert uig.report_identity(ss, False) == a                                 # the same inputs: the same identity
    uig._veh(ss)["wheelbase_mm"] += 70.0
    b = uig.report_identity(ss, False)
    assert b != a and b.split(" | ")[0] == "seed"
    seen = {a, b}
    for key, value in (("ig_seed_hp", {"lower_outer": [0.0, 520.0, 85.0]}), ("ig_axle", "rear"), ("ig_travel", 30.0),
                       ("ig_review_limits", [{"key": "kpi_deg", "lo": 0.0, "hi": 12.0}]),
                       ("genesis_corners", {"rear": {}}), ("ig_brk", {"edited_rows": {"0": {"reach (mm)": 12.0}}}),
                       ("ig_cmp_F", 3200.0), ("ig_cmp_L", 400.0), ("ig_cmp_rep", 0.4), ("ig_cmp_dcam", 0.2),
                       ("ig_cmp_dtoe", 0.05)):
        ss[key] = value
        seen.add(uig.report_identity(ss, False))
    assert len(seen) == 13                                                     # each one changes it
    ss["ig_sw_go"] = True                                                      # a button press is not an input
    assert uig.report_identity(ss, False) in seen


# ---- examples are not results ------------------------------------------------
def _screened():
    v = uig._veh({})
    st = _St()
    return v, uig._sec_structural(st, pd, v, Hardpoints.default(), "front")


def test_the_example_bracket_is_left_out_of_the_report_until_it_is_edited():
    v, sc = _screened()
    st = _St()
    out = uig._sec_brackets(st, pd, v, sc, "k")
    assert out["declared"] is False and st.tables[-1]["paused"] is True
    assert any("example bracket" in n for n in st.notes)
    assert st.session_state.get(rc.PAUSED_KEY) is None                         # the pause ends with the block

    def shorter(df):
        df = df.copy()
        df.loc[0, "reach (mm)"] = 12.0
        return df
    st = _St(edit=shorter)
    out = uig._sec_brackets(st, pd, v, sc, "k")
    t = st.tables[-1]
    assert out["declared"] is True and t["paused"] is False and t["heading"].startswith("Bracket screening")
    assert out["fos_as_drawn"] == pytest.approx(float(t["df"]["FoS"][0]), rel=1e-3)
    assert list(t["df"]["reach (mm)"])[0] == "12"                               # the report shows what was screened
    assert not any("example bracket" in n for n in st.notes)


def test_bracket_is_declared_by_any_size_or_row_change_but_not_by_the_load():
    ex = pd.DataFrame({"case": list("abc"), "load (N)": [1.0, 2.0, 3.0],
                       **{k: list(x) for k, x in uig._BRACKET_EXAMPLE.items()}})
    assert uig.bracket_is_declared(ex) is False                                # the load follows the worst link force
    for col in uig._BRACKET_EXAMPLE:
        d = ex.copy()
        d.loc[2, col] += 0.5
        assert uig.bracket_is_declared(d) is True
    assert uig.bracket_is_declared(ex.iloc[:2]) is True
    assert uig.bracket_is_declared(pd.DataFrame({"case": list("abc")})) is True   # a missing column is not the example


def test_the_stated_link_example_is_left_out_of_the_report_until_values_are_entered():
    v, sc = _screened()
    bands = {"camber": 0.30, "toe": 0.08}
    st = _St()
    uig._sec_compliance(st, pd, v, Hardpoints.default(), "front", sc, bands)
    by = {t["heading"]: t for t in st.tables}
    assert by["Stated link: how much of the reported extension stretch and lash explain"]["paused"] is True
    assert by["Stated deflections against the tolerance bands"]["paused"] is True
    assert by["Compliance camber and steer against the tolerance bands"]["paused"] is False
    assert any("example values" in n for n in st.notes)

    st = _St(force={"ig_cmp_F": 3200.0})
    uig._sec_compliance(st, pd, v, Hardpoints.default(), "front", sc, bands)
    by = {t["heading"]: t for t in st.tables}
    stated = by["Stated link: how much of the reported extension stretch and lash explain"]
    assert stated["paused"] is False and by["Stated deflections against the tolerance bands"]["paused"] is False
    assert list(stated["df"]["value"])[:3] == ["3200", "485", "0.6"]             # the inputs are printed with the result
    assert not any("example values" in n for n in st.notes)


def test_is_declared_compares_every_value():
    ex = uig._STATED_EXAMPLE
    assert uig.is_declared(ex, ex) is False
    for i in range(len(ex)):
        assert uig.is_declared([x + (0.001 if j == i else 0.0) for j, x in enumerate(ex)], ex) is True


# ---- every table is named, and no two share a name -------------------------
def test_every_analysis_table_has_its_own_heading():
    v, sc = _screened()
    st = _St()
    hp = Hardpoints.default()
    uig._sec_kinematics(st, pd, v, hp, "front", 25.0)
    uig._sec_actuation(st, pd, v, hp, "front")
    uig._sec_steering(st, pd, v, hp, hp, "k")
    uig._sec_structural(st, pd, v, hp, "front")
    uig._sec_compliance(st, pd, v, hp, "front", sc, {"camber": 0.3, "toe": 0.08})
    uig._sec_brackets(st, pd, v, sc, "k")
    uig._sec_packaging(st, pd, v, hp)
    heads = [t["heading"] for t in st.tables]
    assert len(heads) == 15 and all(heads) and len(set(heads)) == len(heads), heads
    assert rc.HEADING_KEY not in st.session_state
    steer = next(t["df"] for t in st.tables if t["heading"].startswith("Steering effort"))
    assert any("aligning moment, declared" in q for q in steer.iloc[:, 0])     # the 50 N·m is named as an input


def test_a_table_without_a_title_sets_no_heading():
    st = _St()
    uig._table(st, pd, [("a (mm)", 1.0)], ["quantity", "value"])
    assert st.tables[0]["heading"] is None
    uig._table(object.__new__(_NoState), pd, [("a (mm)", 1.0)], ["quantity", "value"], title="x")


class _NoState:
    """A streamlit stand-in with no session_state: a title is then simply not recorded."""
    def dataframe(self, df, **k):
        pass


# ---- the review ---------------------------------------------------------------
def test_the_review_checks_the_wheel_rate_spread_and_only_a_declared_bracket():
    out = {"act": {"ok": True, "wheel_rate_spread_pct": 191.5, "stroke_fraction_used": 0.48},
           "brk": {"declared": False, "fos_as_drawn": 0.92}}
    vals = uig._review_values({}, out, "front")
    assert vals["wheel_rate_spread"] == 191.5 and "bracket_fos" not in vals
    rows = {r["key"]: r for r in ga.design_review(vals)}
    assert rows["wheel_rate_spread"]["status"] == "fail" and rows["bracket_fos"]["status"] == "n/a"
    out["brk"]["declared"] = True
    vals = uig._review_values({}, out, "front")
    assert vals["bracket_fos"] == 0.92
    assert {r["key"]: r for r in ga.design_review(vals)}["bracket_fos"]["status"] == "fail"
    assert uig._range_text(rows["wheel_rate_spread"]) == "≤ 25"
    assert uig._range_text(rows["bracket_fos"]) == "≥ 1.5"


def test_a_declared_motion_ratio_still_reports_its_wheel_rate_spread():
    hp = Hardpoints.default()
    hp.rocker_pivot = None                                                     # no linkage: the declared curve is used
    v = uig._veh({})
    st = _St()
    a = uig._sec_actuation(st, pd, v, hp, "front")
    assert not a.get("ok") and a["wheel_rate_spread_pct"] is not None
    assert any("DECLARED" in (t["heading"] or "") for t in st.tables)
