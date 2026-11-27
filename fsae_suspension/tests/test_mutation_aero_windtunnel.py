"""Characterization suite for suspension/aero/windtunnel.py (mutation gate); see tests/_golden.py. Ride-height
conversions both ways, grids and their run matrices, every tunnel provenance (floor, scale, blockage, Reynolds),
a measured map and its recovered grid, the force-to-coefficient helpers and their refusals, point errors with
missing and near-zero references, and the correlation of CFD that matches, sits at each tolerance, over- and
under-predicts, misses points, adds unmatched ones, did not converge, or brings no provenance."""
import math

from suspension.aero import cfd
from suspension.aero import windtunnel as W
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _map(tunnel=None, wheelbase=None, area=1.1, length=1.55):
    m = W.PhysicalAeroMap(tunnel or W.TunnelProvenance("A2"), reference_area_m2=area, reference_length_m=length,
                          wheelbase_mm=wheelbase)
    for i, (f, r) in enumerate(((25.0, 35.0), (25.0, 45.0), (35.0, 35.0), (35.0, 45.0))):
        rh = W.RideHeights(f, r, 20.0, m.wheelbase_mm)
        m.add_measurement(rh, -3.0 - 0.1 * i, 1.2 + 0.02 * i, aero_balance_front=0.45 + 0.01 * i,
                          c_side=0.01 * i if i else None)
    return m


def _cfd(phys, cl=1.0, cd=1.0, bal=0.0, prov=True, converged=True, drop=0, extra=False, bal_none=False):
    out = []
    p = cfd.CFDProvenance(backend="star", fidelity=cfd.SolverFidelity.RANS, is_correlated=False,
                          turbulence_model="kOmegaSST") if prov else None
    for r in phys.measured_points()[drop:]:
        out.append(cfd.CoeffResult(attitude=r.attitude, c_lift=r.c_lift * cl, c_drag=r.c_drag * cd,
                                   aero_balance_front=None if bal_none else r.aero_balance_front + bal,
                                   converged=converged, provenance=p))
    if extra:
        out.append(cfd.CoeffResult(attitude=cfd.Attitude(ride_height_mm=99.0), c_lift=-1.0, c_drag=1.0, converged=True,
                                   provenance=p))
    return out


def _rep(r):
    d = plain(r.as_dict())
    d["worst"] = None if r.worst_cl is None else r.worst_cl.as_dict()
    return d


def _cases():
    rh = W.RideHeights(30.0, 42.0)
    yield "rh", [rh.rake_mm, rh.pitch_deg, rh.label(), W.RideHeights(40.0, 30.0, 15.0, 1600.0).label(),
                 W.RideHeights(40.0, 30.0, 15.0, 1600.0).pitch_deg]
    a = W.ride_heights_to_attitude(rh)
    yield "to_att", [plain(a), plain(W.ride_heights_to_attitude(rh, roll_deg=1.5, yaw_deg=-2.0))]
    yield "from_att", [plain(W.attitude_to_ride_heights(a)), plain(W.attitude_to_ride_heights(a, 1600.0)),
                       plain(W.attitude_to_ride_heights(cfd.Attitude(pitch_deg=1.0, ride_height_mm=20.0, speed_ms=12.0)))]
    g = W.AeroMapGrid([20.0, 30.0], [30.0, 40.0, 50.0], (15.0, 25.0), 1600.0)
    yield "grid", [len(g), plain(g.ride_height_points()), plain(g.attitudes()), plain(g.to_run_matrix())]
    yield "grid.default", [len(W.AeroMapGrid([25.0], [35.0])), plain(W.AeroMapGrid([25.0], [35.0]).attitudes())]
    for name, t in (("default", W.TunnelProvenance("A2")),
                    ("fixed", W.TunnelProvenance("Uni", W.GroundState.FIXED_FLOOR, 0.5, False, 0.08, 1.2e6, 1.0, 1.5,
                                                 "n")),
                    ("suction", W.TunnelProvenance("S", W.GroundState.SUCTION_FIXED, 0.4, True, None, 0.0)),
                    ("uncorrected", W.TunnelProvenance("U", blockage_corrected=False, reynolds=3.3e6)),
                    ("near_full", W.TunnelProvenance("N", model_scale=1.0000001))):
        yield f"tunnel.{name}", [t.status(), plain(t.to_cfd_provenance().__dict__)]
    m = _map()
    yield "map", [m.wheelbase_mm, plain(m.measured_points()), plain(m.ride_height_grid()), m.status()]
    yield "map.wheelbase", [_map(wheelbase=1600.0).wheelbase_mm, plain(_map(wheelbase=1600.0).ride_height_grid()),
                            _map(length=1.7).wheelbase_mm]
    yield "forces", [W.downforce_to_clift(900.0, 1.225, 1.1, 20.0), W.downforce_to_clift(900.0, 1.225, 0.0, 20.0),
                     W.downforce_to_clift(900.0, 0.0, 1.1, 20.0), W.downforce_to_clift(900.0, 1.225, -1.0, 20.0),
                     W.drag_to_cdrag(300.0, 1.225, 1.1, 20.0), W.drag_to_cdrag(300.0, 1.225, 1.1, 0.0),
                     W.drag_to_cdrag(300.0, 1.225, -2.0, 20.0)]
    pc = [W.PointCorrelation(rh, -3.0, -3.3, 1.2, 1.1, 0.45, 0.47, True), W.PointCorrelation(rh, None, -3.0, 1e-10, 1.0),
          W.PointCorrelation(rh, -3.0, None, 1.2, 1.2, None, 0.4), W.PointCorrelation(rh, 0.0, 1.0, 1.0, 1.0, 0.4, None)]
    yield "points", [p.as_dict() for p in pc]
    vt = W.VirtualWindTunnel(m, "car.stl", rho=1.2)
    yield "vt", [plain(vt.grid()), plain(vt.run_matrix()), plain(vt.attitudes()), plain(vt.case_specs()),
                 plain(vt.case_specs(cfd.SolverFidelity.DES, 30.0)), vt.plan(), vt.plan(60.0, 4), vt.plan(90.0, 0)]
    for name, kw, tol in (("match", {}, None), ("cl5", {"cl": 1.05}, None), ("cl5x", {"cl": 1.0501}, None),
                          ("under", {"cl": 0.9}, None), ("over", {"cl": 1.1}, None), ("cd", {"cd": 1.06}, None),
                          ("bal2", {"bal": 0.02}, None), ("bal_off", {"bal": 0.0201}, None),
                          ("bal_none", {"bal_none": True}, None), ("rms", {"cl": 1.045}, {"map_rms_pct": 4.5}),
                          ("rms_tight", {"cl": 1.045}, {"map_rms_pct": 4.4}), ("holes", {"drop": 2, "extra": True}, None),
                          ("unconverged", {"converged": False}, None), ("noprov", {"prov": False}, None),
                          ("loose", {"cl": 1.2}, {"cl_pct": 25.0, "map_rms_pct": 25.0})):
        yield f"corr.{name}", _rep(vt.correlate(_cfd(m, **kw), tol=tol))
    off = _cfd(m, cl=1.05, cd=1.03, bal=0.01)
    r0 = vt.correlate(off)
    pts = [p for p in r0.points if p.paired]
    exact = {"cl_pct": max(abs(p.cl_err_pct) for p in pts), "cd_pct": max(abs(p.cd_err_pct) for p in pts),
             "balance_pts": max(abs(p.bal_err_pts) for p in pts), "map_rms_pct": r0.cl_rms_pct}
    yield "corr.exact", _rep(vt.correlate(off, tol=exact))
    yield "corr.bias_edge", _rep(vt.correlate(off, tol={"cl_pct": r0.cl_bias_pct}))
    yield "corr.empty", _rep(vt.correlate([]))
    yield "corr.only_extra", _rep(vt.correlate(_cfd(m, drop=4, extra=True)))
    yield "tol", W.DEFAULT_TUNNEL_TOL
    dm = W.PhysicalAeroMap(W.TunnelProvenance("D"))
    yield "map.defaults", [dm.reference_area_m2, dm.reference_length_m, plain(dm.provenance.__dict__),
                           W.VirtualWindTunnel(m, "car.stl").rho]
    fine = W.PhysicalAeroMap(W.TunnelProvenance("F"))
    for f in (25.00006, 25.00012):
        fine.add_measurement(W.RideHeights(f, 35.00006, 20.00006), -3.0, 1.2)
    yield "grid.fine", plain(fine.ride_height_grid())
    yield "forces.slow", [W.downforce_to_clift(0.5, 1.225, 1.1, 1.0), W.drag_to_cdrag(0.5, 1.225, 1.1, 1.0),
                          W.drag_to_cdrag(300.0, 1.225, 0.0, 20.0)]
    yield "specs.length", [s.reference_length_m for s in W.VirtualWindTunnel(_map(length=1.7), "c.stl").case_specs()]
    wm = _map(wheelbase=1600.0)
    wvt = W.VirtualWindTunnel(wm, "car.stl")
    order = _cfd(wm, drop=2, extra=True)
    yield "corr.wheelbase", _rep(wvt.correlate(order[::-1]))
    yield "corr.small_bias", _rep(vt.correlate(_cfd(m, cl=1.008), tol={"cl_pct": 0.5}))
    yield "nan", math.isnan(W.PointCorrelation(rh, None, None, None, None).cl_err_pct)


def test_every_tunnel_step():
    check_lazy("aero_windtunnel", _cases())
