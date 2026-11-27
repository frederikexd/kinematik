"""Characterization suite for suspension/aero/plug_builder.py (mutation gate); see tests/_golden.py. Several lofts
(blunt, conical, squarish, CAD-section prisms) through every geometry query, slice plans on several sheets and
bondlines (with tolerances, slivers and trims), 1:1 templates, recipes and BOMs, build-day schedules (fitting, not
fitting, odd clocks, cycles, unknown needs), the preflight gate in every state, the assembled plan and report, and
the Streamlit panel driven through a recording stand-in for streamlit (with and without plotly, scale links and
similitude, and both error exits)."""
import io
import sys
import types
import zipfile


from suspension.aero import plug_builder as P
from suspension.aero.scale_model import ScaleSpec, SimilitudePlan, ToleranceBudget
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _try(fn, *a, **k):
    try:
        return plain(fn(*a, **k))
    except ValueError as e:
        return {"raised": "ValueError", "message": str(e)}


def _r(v, n=6):
    return plain(round(v, n)) if isinstance(v, float) else plain(v)


BODIES = {
    "meeting": dict(length_mm=520, base_width_mm=250, base_height_mm=260),
    "cone": dict(length_mm=300, base_width_mm=200, base_height_mm=90, nose_exponent=1.0, height_exponent=1.0,
                 section_exponent=2.0, note="cone"),
    "square": dict(length_mm=700, base_width_mm=400, base_height_mm=180, nose_exponent=0.4, height_exponent=0.5,
                   section_exponent=3.0),
    "low": dict(length_mm=250, base_width_mm=150, base_height_mm=30, nose_exponent=2.0, height_exponent=0.2,
                section_exponent=1.5),
}


def _prism(z):
    return [] if z >= 100 else [(0, 50), (100, 50), (100, -50), (0, -50)]


def _body_cases(name, b):
    xs = (0.0, 1.0, b.length_mm * 0.37, b.length_mm, b.length_mm * 1.2, -5.0)
    out = {"label": b.label(), "area": _r(b.shell_area_m2()), "area_top": _r(b.shell_area_m2(False)),
           "area_n": _r(b.shell_area_m2(n=40)), "vol": _r(b.volume_l()), "vol_n": _r(b.volume_l(n=30))}
    out["pw"] = [_r(b.plan_half_width(x)) for x in xs]
    out["crest"] = [_r(b.crest_height(x)) for x in xs]
    zs = (0.0, b.base_height_mm * 0.3, b.base_height_mm * 0.99, b.base_height_mm, -1.0)
    out["hw"] = [[_r(b.half_width_at(x, z)) for z in zs] for x in xs]
    out["arc"] = [_r(b._section_arc_mm(x)) for x in xs[:4]] + [_r(b._section_arc_mm(xs[2], k=8))]
    out["sec"] = [_r(b._section_area_mm2(x)) for x in xs[:4]] + [_r(b._section_area_mm2(xs[2], k=6))]
    out["outline"] = [plain([(round(x, 6), round(y, 6)) for x, y in b.slice_outline(z, n_pts=12)]) for z in zs]
    out["outline_n"] = len(b.slice_outline(1.0))
    out["outline_low"] = plain([(round(x, 6), round(y, 6)) for x, y in b.slice_outline(0.5, n_pts=4)])
    return out


def _plan_cases(plan):
    t = plan.tolerance
    out = {"n": plan.n_layers(), "trim": _r(plan.top_trim_mm), "bond": _r(plan.bonded_area_m2()),
           "nest": _r(plan.nesting_area_m2()), "sliver": _r(plan.top_layer_sliver()), "summary": plan.summary(),
           "sheets": [_try(plan.sheets_needed, e) for e in (0.75, 1.0, 0.3, 0.29, 1.01, 0.5)],
           "sheets_def": _try(plan.sheets_needed), "tols": [plan.bondline_tol_mm, plan.template_alignment_tol_mm]}
    out["tol"] = None if t is None else [t.n_layers, t.n_bondlines, _r(t.height_rss_mm), _r(t.height_worst_mm),
                                         _r(t.alignment_step_mm), t.summary()]
    out["layers"] = [[ly.index, _r(ly.z_bottom_mm), _r(ly.z_top_mm), _r(ly.thickness_mm), _r(ly.length_mm),
                      _r(ly.width_mm), _r(ly.bbox_area_m2()), len(ly.cut_outline), len(ly.sand_to_outline),
                      plain([(round(x, 4), round(y, 4)) for x, y in ly.cut_outline[:5]]),
                      plain([(round(x, 4), round(y, 4)) for x, y in ly.sand_to_outline[:5]])]
                     for ly in plan.layers]
    return out


def _sched(s):
    return {"fits": s.fits_in_day, "finish": s.finish_min, "day": s.day_minutes, "start": s.day_start_label,
            "crit": list(s.critical_path), "verdict": s.verdict, "sugg": list(s.suggestions), "timeline": s.timeline(),
            "clock": [s.clock(m) for m in (0, 59, 60, 61, 725, 1500)],
            "steps": [[x.step.key, x.start_min, x.active_end_min, x.cure_end_min] for x in s.scheduled]}


def _gate(g):
    return {"go": g.go, "summary": g.summary(), "items": [[i.ok, i.severity, i.text] for i in g.items]}


def _bom(b):
    return {"lines": [[ln.item, _r(ln.quantity), ln.unit, ln.basis, ln.display_qty()] for ln in b.lines],
            "warn": list(b.warnings), "area": _r(b.shell_area_m2), "csv": b.to_csv(), "summary": b.summary()}


class _Rec:
    """A recording stand-in for the streamlit module: every call is logged with its arguments, input widgets
    return the value the scenario sets for their key (else their default)."""

    def __init__(self, log, values, tag="st"):
        self.log, self.values, self.tag = log, values, tag
        self.session_state = {}

    def _put(self, name, a, k):
        self.log.append([self.tag, name, [self._clean(x) for x in a], {q: self._clean(v) for q, v in k.items()}])

    @staticmethod
    def _clean(x):
        if isinstance(x, bytes):
            z = zipfile.ZipFile(io.BytesIO(x))
            return [[i.filename, i.compress_type, z.read(i).decode()] for i in z.infolist()]
        if hasattr(x, "to_plotly_json"):
            j = x.to_plotly_json()
            j["layout"].pop("template", None)
            return plain(j)
        if callable(x):
            return getattr(x, "__name__", "fn")
        return plain(x)

    def columns(self, n):
        self._put("columns", [n], {})
        return [_Rec(self.log, self.values, f"{self.tag}.c{i}") for i in range(n)]

    def expander(self, label, expanded=False):
        self._put("expander", [label], {"expanded": expanded})
        return self

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def _input(self, name, a, k, default):
        self._put(name, a, k)
        return self.values.get(k.get("key"), default)

    def number_input(self, *a, **k):
        return self._input("number_input", a, k, a[3] if len(a) > 3 else a[1])

    def slider(self, *a, **k):
        return self._input("slider", a, k, a[3])

    def text_input(self, *a, **k):
        return self._input("text_input", a, k, a[1])

    def selectbox(self, *a, **k):
        return self._input("selectbox", a, k, a[1][0])

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)

        def call(*a, **k):
            self._put(name, a, k)
        call.__name__ = name
        return call


def _panel(values=None, no_plotly=False, **kw):
    log = []
    st = _Rec(log, values or {})
    saved = {m: sys.modules.get(m) for m in ("streamlit", "plotly.graph_objects")}
    sys.modules["streamlit"] = types.ModuleType("streamlit")
    sys.modules["streamlit"].__dict__.update({n: getattr(st, n) for n in (
        "markdown", "expander", "columns", "error", "success", "dataframe", "selectbox", "download_button",
        "warning", "plotly_chart", "text", "info", "number_input", "text_input")})
    sys.modules["streamlit"].session_state = st.session_state
    if no_plotly:
        sys.modules["plotly.graph_objects"] = None
    try:
        P.render_streamlit_panel(**kw)
    finally:
        for m, v in saved.items():
            if v is None:
                sys.modules.pop(m, None)
            else:
                sys.modules[m] = v
    keys = sorted(st.session_state)
    plan = st.session_state.get("pb_build_plan")
    return {"log": log, "state": keys, "plan": None if plan is None else plan.report()}


def _cases():
    yield "errors.body", [_try(P.NoseconeBody, **{**BODIES["meeting"], **d}) for d in (
        {"length_mm": 0}, {"base_width_mm": -1}, {"base_height_mm": 0}, {"nose_exponent": 0.19},
        {"nose_exponent": 2.01}, {"height_exponent": 0.19}, {"height_exponent": 2.01},
        {"section_exponent": 1.49}, {"nose_exponent": 0.2}, {"height_exponent": 2.0}, {"base_width_mm": 0.5})]
    yield "errors.sheet", [_try(P.FoamSheet, **d) for d in (
        {"thickness_mm": 0}, {"length_mm": -1}, {"width_mm": 0}, {"thickness_tol_mm": -0.01},
        {"thickness_tol_mm": 0.0}, {"length_mm": 0.5})]
    yield "errors.recipe", [_try(P.LayupRecipe, **d) for d in (
        {"plies": 0}, {"plies": 1}, {"fabric_areal_gsm": 0}, {"coating_thickness_mm": -1},
        {"coating_density_g_cm3": 0}, {"adhesive_coverage_ml_m2": 0}, {"fabric_waste_frac": 1.0},
        {"fabric_waste_frac": -0.01}, {"resin_mix_loss_frac": 1.0}, {"resin_mix_loss_frac": -0.1},
        {"fabric_waste_frac": 0.0, "resin_mix_loss_frac": 0.0})]
    sheet = P.FoamSheet()
    yield "sheet", [_r(sheet.area_m2()), plain(sheet.__dict__)]
    bodies = {k: P.NoseconeBody(**v) for k, v in BODIES.items()}
    bodies["prism"] = P.NoseconeBody.from_sections(100, 100, 100, _prism)
    bodies["prism_note"] = P.NoseconeBody.from_sections(100, 120, 90, _prism, note="CAD v2")
    for name, b in bodies.items():
        yield f"body.{name}", _body_cases(name, b)
    sheets = {"std": P.FoamSheet(), "thin": P.FoamSheet(thickness_mm=10.0, length_mm=600, width_mm=300,
                                                        thickness_tol_mm=0.0, name="tooling"),
              "thick": P.FoamSheet(thickness_mm=50.0, thickness_tol_mm=1.5), "tiny": P.FoamSheet(100, 200, 150)}
    plans = {}
    for bn in ("meeting", "cone", "square", "low", "prism"):
        for sn, sh in sheets.items():
            for kw in ({}, {"bondline_mm": 0.0, "bondline_tol_mm": 0.0, "template_alignment_tol_mm": 0.0},
                       {"bondline_mm": 1.0, "bondline_tol_mm": 0.5, "template_alignment_tol_mm": 2.5,
                        "outline_pts": 20}):
                key = f"{bn}.{sn}.{len(kw)}"
                p = plans[key] = P.SlicePlan.plan(bodies[bn], sh, **kw)
                yield f"plan.{key}", _plan_cases(p)
    yield "plan.errors", [_try(P.SlicePlan.plan, bodies["meeting"], sheet, **d) for d in (
        {"bondline_mm": -0.1}, {"bondline_tol_mm": -0.1}, {"template_alignment_tol_mm": -1})]
    tol = plans["meeting.std.0"].tolerance
    spec = ScaleSpec(ratio=0.4, scaled_chord_mm=520, scaled_height_mm=260, scaled_width_mm=250)
    for h in (260.0, 100.0):
        rep = tol.feed(ToleranceBudget(spec), h).report()
        yield f"tol.feed.{h}", [rep.summary, _r(rep.cl_uncertainty_frac)]
    lay = plans["meeting.std.0"].layers
    yield "svg", [P.layer_template_svg(lay[0]), P.layer_template_svg(lay[-1], margin_mm=0.0)]
    empty = P.FoamLayer(1, 0.0, 1.0, (), (), 0.0, 0.0)
    yield "svg.empty", _try(P.layer_template_svg, empty)
    nosand = P.FoamLayer(2, 0.0, 1.0, lay[0].cut_outline, (), lay[0].length_mm, lay[0].width_mm)
    yield "svg.nosand", P.layer_template_svg(nosand, margin_mm=5.0)
    recipes = {"def": P.LayupRecipe(), "thin": P.LayupRecipe(coating_thickness_mm=0.45, flatting_removal_mm=0.3),
               "edge": P.LayupRecipe(coating_thickness_mm=0.5, flatting_removal_mm=0.3),
               "heavy": P.LayupRecipe(plies=5, fabric_areal_gsm=400, resin_to_fiber_ratio=0.6, fabric_waste_frac=0.0,
                                      coating_layers=1, coating_thickness_mm=1.5, coating_density_g_cm3=1.4,
                                      flatting_removal_mm=0.0, resin_mix_loss_frac=0.0, adhesive_coverage_ml_m2=50)}
    yield "recipes", {k: [_r(r.coating_margin_mm()), plain(r.__dict__)] for k, r in recipes.items()}
    for key in ("meeting.std.0", "cone.thin.3", "square.thick.0", "low.tiny.3", "prism.std.0"):
        bn = key.split(".")[0]
        sh = sheets[key.split(".")[1]]
        for rn, rc in recipes.items():
            for crew in (1, 6, 12):
                yield f"bom.{key}.{rn}.{crew}", _try(lambda: _bom(P.MaterialsEstimate.compute(
                    bodies[bn], plans[key], rc, sh, crew_size=crew)))
    yield "bom.pack", [_bom(P.MaterialsEstimate.compute(bodies["square"], plans["square.std.0"], recipes["def"],
                                                         sheets["std"], packing_efficiency=e)) for e in (1.0, 0.4)]
    yield "bom.err", _try(P.MaterialsEstimate.compute, bodies["meeting"], plans["meeting.std.0"], recipes["def"],
                          sheet, crew_size=0)
    yield "bomline", [P.BOMLine(i, q, u, "b").display_qty() for i, q, u in (
        ("a", 1.2, "sheets"), ("b", 2.0, "cans"), ("c", 0.1, "pcs"), ("d", 3.0, "pairs"), ("e", 1.01, "sets"),
        ("f", 1.50, "kg"), ("g", 2.0, "m"), ("h", 0.125, "L"), ("i", 10.0, "ml"), ("j", 0.004, "kg"))]
    for key in ("meeting.std.0", "square.thin.0", "low.tiny.3"):
        for rn in ("def", "heavy"):
            for kw in ({}, {"adhesive_cure_min": 15, "laminating_cure_min": 30},
                       {"adhesive_cure_min": 600, "laminating_cure_min": 900}):
                steps = P.default_build_day(plans[key], recipes[rn], **kw)
                yield f"steps.{key}.{rn}.{len(kw)}.{kw.get('adhesive_cure_min')}", [
                    [s.key, s.label, s.crew, s.active_min, s.cure_min, list(s.needs), s.note] for s in steps]
                for win in (("08:00", "20:00"), ("07:30", "23:59"), ("09:15", "13:05")):
                    yield f"sched.{key}.{rn}.{kw.get('adhesive_cure_min')}.{win[0]}", _sched(
                        P.BuildDaySchedule.plan(steps, *win))
    B = P.BuildStep
    yield "sched.default_args", _sched(P.BuildDaySchedule.plan([B("a", "A", "cut", 30)]))
    yield "sched.par", _sched(P.BuildDaySchedule.plan([
        B("a", "A", "cut", 60, 30), B("b", "B", "shape", 60), B("c", "C", "cut", 20, 0, ("a",)),
        B("d", "D", "layup", 10, 100, ("b", "c")), B("e", "E", "cut", 5, 0, ("d",), "n")], "06:00", "10:00"))
    yield "sched.exact", _sched(P.BuildDaySchedule.plan([B("a", "A", "cut", 60, 60)], "08:00", "10:00"))
    yield "sched.over1", _sched(P.BuildDaySchedule.plan([B("a", "A", "cut", 60, 61)], "08:00", "10:00"))
    yield "sched.errors", [
        _try(P.BuildDaySchedule.plan, [B("a", "A", "cut", 1)], "10:00", "10:00"),
        _try(P.BuildDaySchedule.plan, [B("a", "A", "cut", 1)], "10:00", "09:59"),
        _try(P.BuildDaySchedule.plan, [B("a", "A", "cut", 1, 0, ("zz",))]),
        _try(P.BuildDaySchedule.plan, [B("a", "A", "cut", 1, 0, ("b",)), B("b", "B", "cut", 1, 0, ("a",))]),
        _try(P.BuildDaySchedule.plan, [B("a", "A", "cut", 1, 0, ("a",))])]
    pm = plans["meeting.std.0"]
    good = P.BuildDaySchedule.plan(P.default_build_day(pm, recipes["def"]))
    bad_order = P.BuildDaySchedule.plan([B("layup1", "L", "layup", 10), B("release", "R", "cut", 10, 30)])
    missing = P.BuildDaySchedule.plan([B("layup1", "L", "layup", 10)])
    missing2 = P.BuildDaySchedule.plan([B("release", "R", "cut", 10)])
    touch = P.BuildDaySchedule.plan([B("release", "R", "cut", 10, 5), B("layup1", "L", "layup", 10, 0, ("release",))])
    sim_ok = SimilitudePlan.match_reynolds(spec, 15.0, 60.0)
    sim_bad = SimilitudePlan.match_reynolds(spec, 30.0, 20.0)
    for gname, g in (("all", dict(schedule=good, similitude=sim_ok, crew_size=6)),
                     ("bad_sim", dict(schedule=good, similitude=sim_bad, crew_size=1)),
                     ("noschedule", dict()), ("order", dict(schedule=bad_order, crew_size=3)),
                     ("missing", dict(schedule=missing)), ("missing2", dict(schedule=missing2)),
                     ("touch", dict(schedule=touch, crew_size=2))):
        for rn in ("def", "thin", "edge", "heavy"):
            for key in ("meeting.std.0", "low.tiny.3", "meeting.thick.0", "cone.thin.3", "prism.std.0"):
                yield f"gate.{gname}.{rn}.{key}", _gate(P.PreflightGate.check(plans[key], recipes[rn], **g))
    notol = P.SlicePlan(bodies["meeting"], sheet, 0.3, 0.2, 1.0, layers=pm.layers)
    yield "gate.notol", _gate(P.PreflightGate.check(notol, recipes["def"], crew_size=2))
    yield "notol", [notol.top_layer_sliver(), notol.n_layers(), notol.summary()]
    bom = P.MaterialsEstimate.compute(bodies["meeting"], pm, recipes["def"], sheet)
    gate = P.PreflightGate.check(pm, recipes["def"], schedule=good, crew_size=6)
    for name, kw in (("full", dict(gate=gate, scale_spec=spec, similitude=sim_ok)), ("bare", {}),
                     ("spec", dict(scale_spec=spec))):
        for pn, pl in (("pm", pm), ("notol", notol)):
            bp = P.PlugBuildPlan(bodies["meeting"], pl, recipes["def"], bom, good, **kw)
            tb = bp.tolerance_budget()
            yield f"build.{name}.{pn}", [bp.provenance(), bp.report(),
                                         None if tb is None else tb.report().summary]
    late = P.BuildDaySchedule.plan(P.default_build_day(plans["square.thin.0"], recipes["heavy"],
                                                       adhesive_cure_min=600, laminating_cure_min=900),
                                   "09:15", "13:05")
    yield "build.late", P.PlugBuildPlan(bodies["square"], plans["square.thin.0"], recipes["heavy"], bom, late).report()
    tall = P.NoseconeBody.from_sections(100, 100, 150, _prism)
    yield "plan.tall", P.SlicePlan.plan(tall, P.FoamSheet(thickness_mm=40.0)).n_layers()
    over = P.NoseconeBody(**{**BODIES["meeting"], "base_height_mm": 99.0})
    yield "plan.over", P.SlicePlan.plan(over, P.FoamSheet(thickness_mm=46.0), bondline_mm=4.0).n_layers()
    yield "plan.fit", [_try(P.SlicePlan.plan(bodies["prism"], P.FoamSheet(length_mm=lg, width_mm=wd)).sheets_needed)
                       for lg, wd in ((100, 100), (100, 200), (200, 100))]
    exact = P.NoseconeBody(**{**BODIES["meeting"], "base_height_mm": 100.0})
    xp = P.SlicePlan.plan(exact, P.FoamSheet(thickness_mm=50.0, thickness_tol_mm=0.5), bondline_mm=0.0,
                          bondline_tol_mm=0.0)
    yield "gate.frac", _gate(P.PreflightGate.check(xp, recipes["def"], crew_size=2))
    feeds = []
    for th, rss, align in ((0.0, 0.5, 0.0), (0.5, 0.5, 0.0), (100.0, 0.0, 0.0), (100.0, 0.5, 0.0), (100.0, 1.0, 1.0)):
        st = P.StackTolerance(n_layers=1, n_bondlines=0, height_rss_mm=rss, height_worst_mm=rss, alignment_step_mm=align)
        rep = _try(lambda: st.feed(ToleranceBudget(spec), th).report())
        feeds.append(rep if isinstance(rep, dict) and "raised" in rep else None)
        if feeds[-1] is None:
            rp = st.feed(ToleranceBudget(spec), th).report()
            feeds[-1] = [rp.summary, _r(rp.cl_uncertainty_frac)]
    yield "tol.feed.edges", feeds
    yield "sched.edges", [
        _sched(P.BuildDaySchedule.plan([B("a", "A", "cut", 30), B("b", "B", "cut", 20)], "08:00", "08:01")),
        _sched(P.BuildDaySchedule.plan([B("layup1", "L", "layup", 300)], "08:00", "10:00")),
        _sched(P.BuildDaySchedule.plan([B("layup2", "L", "layup", 300)], "08:00", "10:00")),
        _sched(P.BuildDaySchedule.plan([B("a", "A", "cut", 30, 10), B("b", "B", "shape", 5),
                                        B("c", "C", "cut", 10, 0, ("a", "b"))], "08:00", "10:00"))]
    yield "bomline.more", [P.BOMLine(i, q, u, "b").display_qty() for i, q, u in (
        ("a", 2.3, "cans"), ("b", 1.2, "pairs"), ("c", 2.5, "m2"), ("d", 2.0, "kg"))]
    yield "panel.custom", _panel({"pb_t": 30.0, "pb_sl": 1500.0, "pb_sw": 700.0, "pb_ttol": 0.8, "pb_coats": 2,
                                  "pb_cthk": 0.6, "pb_start": "07:15", "pb_gcure": 45, "pb_lcure": 200})
    yield "panel.default", _panel()
    yield "panel.linked", _panel({"pb_layer": 2, "pb_plies": 3, "pb_crew": 2, "pb_cthk": 0.35, "pb_blunt": 0.9},
                                 scale_spec=spec, similitude=sim_bad)
    yield "panel.noplotly", _panel({"pb_end": "11:00", "pb_ratio": 0.5, "pb_lcure": 1440}, no_plotly=True,
                                   similitude=sim_ok)
    yield "panel.badplan", _panel({"pb_bond": -1.0})
    yield "panel.badsched", _panel({"pb_end": "07:00"})


def test_every_plug_builder_step():
    check_lazy("aero_plug_builder", _cases(), rel=1e-7, abs_=1e-9)
