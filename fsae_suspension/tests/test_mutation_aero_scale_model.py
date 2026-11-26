"""Characterization suite for suspension/aero/scale_model.py (mutation gate); see tests/_golden.py. Air properties
across temperatures, Reynolds numbers, scale specs with and without heights and widths, Reynolds matching with no
tunnel limit, a reachable one and a capped one (including the low-Re bubble warning), every tolerance channel alone
and together, mount alignment nominal and off, and the run plan's uncertainty, provenance and report."""
from suspension.aero import scale_model as M
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _props(obj, names):
    out = {}
    for n in names:
        try:
            v = getattr(obj, n)
            out[n] = plain(v() if callable(v) else v)
        except (ValueError, ZeroDivisionError, TypeError) as e:
            out[n] = {"raised": type(e).__name__, "message": str(e)}
    return out


SPEC_PROPS = ("full_chord_mm", "full_height_mm", "full_width_mm", "inverse_ratio", "scaled_chord_m", "full_chord_m",
              "area_ratio", "scaled_frontal_area_m2", "label", "full_size_label")


def _specs():
    return {"half": M.ScaleSpec(0.5, 400.0, 120.0, 900.0, "half-scale wing"), "third": M.ScaleSpec(1 / 3, 250.0),
            "full": M.ScaleSpec(1.0, 1250.0, None, 1400.0), "tiny": M.ScaleSpec(0.1, 60.0, 15.0),
            "tall_only": M.ScaleSpec(0.4, 300.0, 200.0)}


def _cases():
    yield "nu", [M.air_kinematic_viscosity(t) for t in (-20.0, 0.0, 15.0, 25.0, 40.0)] + [M.air_kinematic_viscosity()]
    yield "re", [M.reynolds(v, L) for v in (5.0, 20.0, 40.0) for L in (0.1, 0.5, 1.25)] + [
        M.reynolds(20.0, 0.5, nu=1.6e-5), M.reynolds(0.0, 1.0)]
    for k, s in _specs().items():
        yield f"spec.{k}", [plain(s), _props(s, SPEC_PROPS)]
        for full, tunnel, temp in ((20.0, None, 15.0), (20.0, 60.0, 15.0), (25.0, 30.0, 25.0), (10.0, 8.0, 0.0),
                                   (5.0, 100.0, 40.0)):
            p = M.SimilitudePlan.match_reynolds(s, full, tunnel_max_speed_ms=tunnel, temp_c=temp)
            yield f"sim.{k}.{full}.{tunnel}.{temp}", [plain(p), p.provenance_note()]
        b = M.ToleranceBudget(s)
        yield f"tol.{k}.empty", plain(b.report())
        for name, fn, args in (("chord", "add_chord_deviation_mm", (0.8,)), ("camber", "add_camber_deviation_mm", (0.5,)),
                               ("span", "add_span_deviation_mm", (2.0, 450.0)),
                               ("wavy", "add_surface_waviness_mm", (0.3,)),
                               ("neg", "add_chord_deviation_mm", (-1.2,)),
                               ("custom", "add_custom", ("paint", 0.01, 0.02, "spray test"))):
            one = M.ToleranceBudget(s)
            ret = getattr(one, fn)(*args)
            yield f"tol.{k}.{name}", [plain(one.report()), ret is one]
            getattr(b, fn)(*args)
        yield f"tol.{k}.all", plain(b.report())
    s = _specs()["half"]
    for inc, pos in ((0.0, 0.0), (0.5, 0.0), (0.0, 2.0), (-1.5, -3.0), (1e-7, 1e-7)):
        m = M.MountAlignment(inc, pos, note="as built")
        yield f"mount.{inc}.{pos}", [plain(m), m.incidence_uncertainty_frac(), m.status()]
    m2 = M.MountAlignment(1.0, 0.0, cl_per_deg_frac=0.1)
    yield "mount.slope", [m2.incidence_uncertainty_frac(), m2.status()]
    for tunnel in (None, 30.0):
        sim = M.SimilitudePlan.match_reynolds(s, 20.0, tunnel_max_speed_ms=tunnel)
        tol = M.ToleranceBudget(s).add_chord_deviation_mm(0.5).add_camber_deviation_mm(0.4)
        for mount in (None, M.MountAlignment(0.7, 1.0)):
            plan = M.ScaledRunPlan(sim, tol) if mount is None else M.ScaledRunPlan(sim, tol, mount)
            yield f"plan.{tunnel}.{mount is None}", _props(plan, ("tunnel_reynolds", "model_scale",
                                                                 "combined_cl_uncertainty_frac", "provenance", "report"))
    def err(fn, *a, **k):
        try:
            return plain(fn(*a, **k))
        except ValueError as e:
            return str(e)
    yield "errors", [err(M.reynolds, 10.0, 0.0), err(M.reynolds, 10.0, -1.0), err(M.reynolds, 10.0, 1.0, nu=0.0),
                     err(M.reynolds, 10.0, 0.5, nu=-1e-5), err(M.reynolds, 10.0, 0.5, nu=1e-5),
                     err(M.ScaleSpec, 0.0, 100.0), err(M.ScaleSpec, 1.5, 100.0), err(M.ScaleSpec, -0.5, 100.0),
                     err(M.ScaleSpec, 0.5, 0.0), err(M.ScaleSpec, 0.5, -3.0), plain(M.ScaleSpec(0.5, 0.5)),
                     plain(M.ScaleSpec(1.0, 10.0)),
                     err(lambda: M.ToleranceBudget(s).add_span_deviation_mm(1.0, 0.0).report()),
                     err(lambda: M.ToleranceBudget(s).add_span_deviation_mm(1.0, -2.0).report()),
                     err(lambda: M.ToleranceBudget(s).add_span_deviation_mm(0.2, 0.5).report())]
    tiny = M.ScaleSpec(0.1, 60.0, 15.0)
    big = M.ToleranceBudget(tiny).add_chord_deviation_mm(9.0).add_camber_deviation_mm(4.0).add_span_deviation_mm(
        6.0, 40.0).add_surface_waviness_mm(2.0)
    yield "tol.big", plain(big.report())
    sim = M.SimilitudePlan.match_reynolds(tiny, 30.0, tunnel_max_speed_ms=40.0)
    yield "plan.big", _props(M.ScaledRunPlan(sim, big, M.MountAlignment(2.0, 5.0)), ("provenance", "report"))
    full_v = 20.0
    need = M.SimilitudePlan.match_reynolds(s, full_v).matched_speed_ms if hasattr(
        M.SimilitudePlan.match_reynolds(s, full_v), "matched_speed_ms") else None
    yield "sim.need", need
    if need is not None:
        for tunnel in (need, need - 0.5, need + 0.5):
            yield f"sim.tunnel.{tunnel - need:+.1f}", plain(M.SimilitudePlan.match_reynolds(s, full_v,
                                                                                            tunnel_max_speed_ms=tunnel))
    yield "sim.zero_speed", plain(M.SimilitudePlan.match_reynolds(s, 0.0))
    yield "sim.two_warnings", plain(M.SimilitudePlan.match_reynolds(tiny, 30.0, tunnel_max_speed_ms=5.0))
    yield "consts", [M.DEFAULT_AIR_DENSITY, M.DEFAULT_AIR_KINEMATIC_VISCOSITY, M.LOW_RE_BUBBLE_THRESHOLD]


def test_every_scale_plan_and_budget():
    check_lazy("aero_scale_model", _cases())
