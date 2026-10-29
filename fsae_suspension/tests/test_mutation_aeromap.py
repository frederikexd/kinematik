"""suspension/aero/aeromap.py: only usable results are kept, the envelope and swept axes come from what varied,
multilinear interpolation reproduces a bilinear field exactly, queries clamp and say so, a missing corner falls back
to the normalised inverse-distance blend written out here, an absent channel stays None, and the CSV round trip and
status read back what went in."""

import pytest

from suspension.aero.aeromap import AeroMap, AeroQuery, _AXES
from suspension.aero.cfd import Attitude, CFDProvenance, CoeffResult, SolverFidelity

PROV = CFDProvenance("openfoam", SolverFidelity.RANS)


def F(p, h):
    return -1.0 + 0.2 * p + 0.01 * h + 0.05 * p * h


def _r(p, h, conv=True, side=None, bal=None, drag=True, prov=None, roll=0.0, yaw=0.0, v=20.0):
    return CoeffResult(attitude=Attitude(roll, p, yaw, h, v), c_lift=F(p, h), c_drag=(0.5 + 0.01 * p) if drag else None,
                       c_side=side, aero_balance_front=bal, converged=conv, provenance=prov)


def _grid(**kw):
    return AeroMap.from_results([_r(p, h, **kw) for p in (-1.0, 0.0, 2.0) for h in (20.0, 40.0)], reference_area_m2=1.1,
                                reference_length_m=1.6)


def test_storage_keeps_only_usable_results_and_the_first_provenance():
    m = AeroMap()
    assert (m.reference_area_m2, m.reference_length_m, m.provenance, len(m)) == (1.0, 1.55, None, 0)
    assert m.add(_r(0, 20, conv=False)) is False and m.add(_r(0, 20, drag=False)) is False and len(m) == 0
    assert m.add(_r(0, 20)) is True and m.provenance is None
    other = CFDProvenance("fluent", SolverFidelity.DES)
    assert m.add(_r(1, 20, prov=PROV)) is True and m.provenance is PROV
    m.add(_r(2, 20, prov=other))
    assert m.provenance is PROV and len(m) == 3
    m.add(_r(2, 20))                                                    # same key replaces
    assert len(m) == 3
    g = _grid()
    assert (g.reference_area_m2, g.reference_length_m, len(g)) == (1.1, 1.6, 6)
    assert AeroMap(provenance=other).add(_r(0, 20, prov=PROV)) and AeroMap(provenance=other).provenance is other
    assert _AXES == ("roll_deg", "pitch_deg", "yaw_deg", "ride_height_mm", "speed_ms")


def test_the_envelope_and_the_swept_axes():
    g = _grid()
    assert g._axis_values("pitch_deg") == [-1.0, 0.0, 2.0] and g.swept_axes() == ["pitch_deg", "ride_height_mm"]
    assert g.envelope() == {"roll_deg": (0.0, 0.0), "pitch_deg": (-1.0, 2.0), "yaw_deg": (0.0, 0.0),
                            "ride_height_mm": (20.0, 40.0), "speed_ms": (20.0, 20.0)}
    assert AeroMap().envelope() == {} and AeroMap().swept_axes() == []


def test_interpolation_is_exact_on_a_bilinear_field():
    g = _grid(side=0.3, bal=0.45)
    for p, h in ((-0.5, 25.0), (1.3, 37.0), (0.0, 30.0), (2.0, 40.0), (-1.0, 20.0)):
        q = g.query(Attitude(0.0, p, 0.0, h, 20.0))
        assert q.c_lift == pytest.approx(F(p, h), rel=1e-12) and q.c_drag == pytest.approx(0.5 + 0.01 * p, rel=1e-12)
        assert q.c_side == pytest.approx(0.3) and q.aero_balance_front == pytest.approx(0.45)
        assert (q.extrapolated, q.note) == (False, "")
    q = g.query(Attitude(3.0, 5.0, 2.0, 10.0, 35.0))
    assert q.c_lift == pytest.approx(F(2.0, 20.0)) and q.extrapolated is True and q.note == "clamped to solved envelope"
    low = g.query(Attitude(0.0, -4.0, 0.0, 30.0, 20.0))
    assert low.c_lift == pytest.approx(F(-1.0, 30.0)) and low.extrapolated is True
    hi = g.query(Attitude(0.0, 0.5, 0.0, 99.0, 20.0))
    assert hi.c_lift == pytest.approx(F(0.5, 40.0)) and hi.extrapolated is True
    edge = g.query(Attitude(0.0, 2.0, 0.0, 40.0, 20.0))
    assert edge.extrapolated is False
    assert _grid().query(Attitude(0.0, 0.5, 0.0, 30.0, 20.0)).c_side is None          # never invented


def test_an_empty_map_and_a_single_point():
    q = AeroMap().query(Attitude())
    assert q == AeroQuery(None, None, None, None, extrapolated=False, note="empty map")
    one = AeroMap.from_results([_r(1.0, 25.0, bal=0.4)])
    q = one.query(Attitude(0.0, 9.0, 0.0, 99.0, 20.0))
    assert (q.c_lift, q.aero_balance_front, q.c_side) == (F(1.0, 25.0), 0.4, None) and q.extrapolated is True


def _idw(points, q, axes, every=None):
    num = den = 0.0
    for a, v in points:
        d2 = 0.0
        for ax in axes:
            vals = sorted({getattr(b, ax) for b in (every or [b for b, _ in points])})
            rng = (vals[-1] - vals[0]) if len(vals) > 1 else 1.0
            d2 += ((getattr(a, ax) - q[ax]) / (rng or 1.0)) ** 2
        if d2 < 1e-12:
            return v
        num += v / d2
        den += 1 / d2
    return num / den


def test_a_missing_corner_falls_back_to_inverse_distance():
    rs = [_r(p, h) for p, h in ((-1.0, 20.0), (2.0, 20.0), (-1.0, 40.0), (0.0, 30.0))]   # (2, 40) never solved
    m = AeroMap.from_results(rs)
    att = Attitude(0.0, 1.0, 0.0, 35.0, 20.0)
    q = m.query(att)
    pts = [(r.attitude, r.c_lift) for r in rs]
    want = _idw(pts, {"pitch_deg": 1.0, "ride_height_mm": 35.0}, ["pitch_deg", "ride_height_mm"])
    assert q.c_lift == pytest.approx(want, rel=1e-12) and not q.extrapolated
    on = m.query(Attitude(0.0, 0.0, 0.0, 30.0, 20.0))
    assert on.c_lift == pytest.approx(F(0.0, 30.0))                      # sits on a solved point
    part = AeroMap.from_results([_r(-1.0, 20.0, bal=0.4), _r(2.0, 20.0), _r(-1.0, 40.0, bal=0.5), _r(0.0, 30.0)])
    bal = part.query(att).aero_balance_front
    assert bal == pytest.approx(_idw([(Attitude(0, -1.0, 0, 20.0, 20.0), 0.4), (Attitude(0, -1.0, 0, 40.0, 20.0), 0.5)],
                                     {"pitch_deg": 1.0, "ride_height_mm": 35.0}, ["pitch_deg", "ride_height_mm"],
                                     every=[r.attitude for r in part._points.values()]))
    assert part.query(att).c_side is None


def test_a_three_axis_grid_and_a_corner_channel_gap():
    rs = [_r(p, h, v=v, bal=(None if (p, h, v) == (0.0, 20.0, 30.0) else 0.4))
          for p in (0.0, 2.0) for h in (20.0, 40.0) for v in (20.0, 30.0)]
    m = AeroMap.from_results(rs)
    q = m.query(Attitude(0.0, 1.0, 0.0, 30.0, 25.0))
    assert q.c_lift == pytest.approx(F(1.0, 30.0)) and q.aero_balance_front is None


def test_the_csv_round_trip():
    g = _grid(side=0.25, bal=0.42)
    g.add(_r(5.0, 30.0))
    text = g.to_csv()
    lines = text.strip().splitlines()
    assert lines[0] == "roll_deg,pitch_deg,yaw_deg,ride_height_mm,speed_ms,c_lift,c_drag,c_side,aero_balance_front,converged"
    assert lines[-1] == f"0.0,5.0,0.0,30.0,20.0,{F(5.0, 30.0)},{0.5 + 0.05},,,1"
    back = AeroMap.from_csv(text, reference_area_m2=1.2, reference_length_m=1.5)
    assert len(back) == 7 and (back.reference_area_m2, back.reference_length_m) == (1.2, 1.5)
    for k, r in g._points.items():
        b = back._points[k]
        assert (b.c_lift, b.c_drag, b.c_side, b.aero_balance_front, b.converged) == (
            pytest.approx(r.c_lift), pytest.approx(r.c_drag), r.c_side, r.aero_balance_front, True)
    p = back.provenance
    assert (p.backend, p.fidelity, p.is_correlated) == ("imported-csv", SolverFidelity.RANS, False)
    assert p.notes == ("Imported aero map; provenance is whatever the source was. KinematiK did not run or validate "
                       "this CFD.")
    kept = AeroMap.from_csv(text, provenance=PROV)
    assert kept.provenance is PROV and all(r.provenance is PROV for r in kept._points.values())
    sparse = "c_lift,c_drag,converged\n-1.0,0.5,\n-2.0,0.6,0\n-3.0,0.7,TRUE\n-4.0, ,yes\n"
    s = AeroMap.from_csv(sparse)
    assert len(s) == 1 and list(s._points) == [Attitude(0, 0, 0, 30, 20).key()]   # one attitude, last usable wins
    assert s._points[Attitude(0, 0, 0, 30, 20).key()].c_lift == -3.0
    odd = "roll_deg,pitch_deg,yaw_deg,ride_height_mm,speed_ms,c_lift,c_drag,converged\n1,2,3,,,-1,0.5,yes\n"
    assert list(AeroMap.from_csv(odd)._points) == [Attitude(1, 2, 3, 30, 20).key()]
    blank = "roll_deg,c_lift,c_drag,converged\n,-1,0.5,true\n"
    assert list(AeroMap.from_csv(blank)._points) == [Attitude(0, 0, 0, 30, 20).key()]
    no = "c_lift,c_drag,converged\n-1,0.5,no\n"
    assert len(AeroMap.from_csv(no)) == 0


def test_the_status():
    assert AeroMap().status() == "empty aero map"
    g = _grid(prov=PROV)
    assert g.status() == f"AeroMap: 6 points over ['pitch_deg', 'ride_height_mm']; {PROV.status()}"
    one = AeroMap.from_results([_r(0, 20)])
    assert one.status() == "AeroMap: 1 points over ['single point']; provenance unknown"


def test_characterized_maps():
    from tests._golden import check
    res = {}
    cube = AeroMap.from_results([CoeffResult(attitude=Attitude(0.0, p, 0.0, h, v), c_lift=F(p, h) + 0.003 * v * p,
                                             c_drag=0.5 + 0.01 * p + 0.002 * v, c_side=0.01 * h,
                                             aero_balance_front=0.4 + 0.01 * p, converged=True)
                                 for p in (-1.0, 0.0, 2.0) for h in (20.0, 40.0) for v in (15.0, 30.0)])
    qs = [Attitude(0.0, p, 0.0, h, v) for p, h, v in ((-1.0, 20.0, 15.0), (2.0, 40.0, 30.0), (0.5, 27.0, 22.0),
                                                       (-3.0, 50.0, 10.0), (0.0, 30.0, 40.0), (1.0, 20.0, 15.0))]
    res["cube"] = [cube.query(q) for q in qs]
    pts = [_r(p, h) for p in (-1.0, 0.0, 2.0) for h in (20.0, 40.0) if (p, h) != (2.0, 40.0)]
    pts[0] = _r(-1.0, 20.0, side=0.3)
    sparse = AeroMap.from_results(pts)
    res["sparse"] = [sparse.query(Attitude(0.0, p, 0.0, h, 20.0)) for p, h in ((1.0, 30.0), (2.0, 40.0), (0.0, 30.0),
                                                                              (-1.0, 20.0))]
    one = AeroMap.from_results([_r(0.5, 25.0, v=18.0)])
    res["one"] = [one.query(Attitude(3.0, 1.0, 2.0, 10.0, 40.0)), one.reference_length_m, one.reference_area_m2]
    text = ("roll_deg,pitch_deg,ride_height_mm,speed_ms,c_lift,c_drag,c_side,converged\n"
            "0,-1,22,17,-1.1,0.5,,true\n0,1,22,17,-0.9,0.52,0.02,\n0,-1,44,25,-1.3,0.55,,no\n"
            "0,1,44,25,-1.2,0.56,0.03,YES\n,,,,-1.0,0.5,,1\n")
    m = AeroMap.from_csv(text)
    res["csv"] = [sorted(m._points), m.reference_length_m, m.provenance.backend, m.provenance.is_correlated,
                  m.to_csv(), m.query(Attitude(0.0, 0.0, 0.0, 30.0, 20.0))]
    empty = AeroMap.from_csv("c_lift,c_drag,converged\n1,1,0\n", reference_length_m=2.0, provenance=PROV)
    res["csv_empty"] = [len(empty), empty.provenance is PROV, empty.reference_length_m]
    res["csv_prov"] = AeroMap.from_csv(text, provenance=PROV).provenance is PROV
    check("aeromap_maps", res)
