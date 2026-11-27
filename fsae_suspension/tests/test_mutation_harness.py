"""suspension/harness.py against tests/golden: three looms chosen so every check fires both ways (bends, strain
relief, anchoring, 3-D clearance, the flex gate's span, sag, resonance, slack and draped-clearance verdicts,
ampacity from each current source), with the cut list, BOM, mass distribution, formboard, serialisation and summary
compared word for word and number for number. Regenerate with KINEMATIK_REGOLD=1 only after reading the diff."""
import dataclasses
import enum
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from tests._golden import _same

from suspension import harness as H
from suspension.harness import Connector, HarnessLedger, WireRun, check_harness
from suspension.mountpoints import KeepOut

GOLD = Path(__file__).parent / "golden" / "harness.json"


def _plain(x):
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: _plain(getattr(x, f.name)) for f in dataclasses.fields(x)}
    if isinstance(x, enum.Enum):
        return x.value
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, set)):
        return [_plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return _plain(x.tolist())
    if isinstance(x, (np.floating, np.integer)):
        return _plain(x.item())
    if isinstance(x, float):
        return None if math.isnan(x) else ("inf" if math.isinf(x) else float(f"{x:.9g}"))
    return x


def _loom_a():
    """A front loom: a kinked power feed, a gentle sweep, a long unclamped span through a box when it droops."""
    h = HarnessLedger()
    h.set_connector(Connector("ECU", "electrics", xyz_mm=(0, 0, 300), cavities=34, part_number="AS-34",
                              strain_relief_mm=30.0, mass_g=55.0))
    h.set_connector(Connector("PDM", "electrics", xyz_mm=(900, 0, 300), cavities=12, part_number="DTM-12",
                              strain_relief_mm=0.0, mass_g=20.0))
    h.set_connector(Connector("MOT", "powertrain", xyz_mm=(1800, 300, 150), cavities=3, mass_g=None))
    h.set_connector(Connector("SPARE", "electrics", xyz_mm=(50, 50, 50)))
    h.set_connector(Connector("PDM2", "electrics", xyz_mm=(950, 0, 300), cavities=12, part_number="DTM-12",
                              mass_g=21.0))
    h.set_connector(Connector("PDM3", "electrics", xyz_mm=(990, 0, 300), cavities=12, part_number="DTM-12"))
    h.set_wire(WireRun("feed", "electrics", gauge_awg=10, path_mm=[(0, 0, 300), (400, 0, 300), (402, 2, 300),
                                                                   (900, 0, 300)],
                       from_conn="ECU", to_conn="PDM", net="BAT+", carries_current_a=40.0, strip_mm=10.0,
                       service_loop_mm=50.0))
    h.set_wire(WireRun("sweep", "electrics", gauge_awg=18, path_mm=[(0, 0, 300), (300, 0, 300), (600, 150, 300),
                                                                    (900, 0, 300)],
                       from_conn="ECU", to_conn="PDM", net="CAN_H", clamp_idx=[1, 2]))
    h.set_wire(WireRun("droop", "powertrain", gauge_awg=14,
                       path_mm=[(900, 0, 300), (1350, 150, 300), (1800, 300, 150)],
                       from_conn="PDM", to_conn="MOT", net="MOTOR_EN", od_mm=3.2, bundle_min_radius_mult=8.0))
    h.set_wire(WireRun("loose", "electrics", gauge_awg=22, path_mm=[(10, 10, 10), (20, 10, 10)],
                       from_conn="ECU", to_conn="NOWHERE", net="SENSE"))
    return h


def _loom_b():
    """A rear loom on a hot car: bundled conductors, an AWG with no NEC ampacity, a short tight one."""
    h = HarnessLedger(ambient_c=75.0, ambient_is_local=True, insulation="pvc", termination_c=75.0,
                      clearance_warn_mm=25.0, clearance_fail_mm=2.0, vib_g=8.0, excitation_lo_hz=5.0,
                      excitation_hi_hz=60.0)
    h.set_connector(Connector("ACC", "accumulator", xyz_mm=(0, 0, 0), cavities=2, mass_g=120.0))
    h.set_connector(Connector("INV", "powertrain", xyz_mm=(600, 0, 0), cavities=2, mass_g=80.0,
                              strain_relief_mm=5.0))
    for i in range(4):
        h.set_wire(WireRun(f"hv{i}", "accumulator", gauge_awg=8 if i < 2 else 30,
                           path_mm=[(0, 0, 0), (200, 0, 40 + 5 * i), (400, 0, 40), (600, 0, 0)],
                           from_conn="ACC", to_conn="INV", net="HV+" if i % 2 == 0 else "HV-",
                           clamp_idx=[2], is_estimate=False, set_by="FT", notes=f"run {i}"))
    h.set_wire(WireRun("hot", "accumulator", gauge_awg=20, path_mm=[(0, 0, 0), (5, 0, 0), (5, 5, 0), (600, 0, 0)],
                       from_conn="ACC", to_conn="INV", net="TEMP"))
    h.set_wire(WireRun("one", "accumulator", gauge_awg=20, path_mm=[(0, 0, 0)], from_conn="ACC"))
    return h


def _loom_c():
    """An empty loom and one wire with no route: the blind spots must be reported, not passed."""
    h = HarnessLedger()
    h.set_wire(WireRun("ghost", "electrics"))
    return h


def _loom_d():
    """Twelve wires on seeded random routes, gauges, clamps and connectors: breadth the hand-built looms miss."""
    rng = np.random.default_rng(7)
    h = HarnessLedger(vib_g=5.0)
    for i in range(12):
        n = int(rng.integers(2, 7))
        pts = np.cumsum(rng.uniform(-250.0, 250.0, size=(n, 3)), axis=0) + [600.0, 0.0, 300.0]
        a, b = f"C{i}a", f"C{i}b"
        h.set_connector(Connector(a, "electrics", xyz_mm=tuple(pts[0]), cavities=int(rng.integers(1, 30)),
                                  strain_relief_mm=float(rng.uniform(0, 60)), mass_g=float(rng.uniform(0, 60))))
        h.set_connector(Connector(b, "powertrain" if i % 3 else "electrics", xyz_mm=tuple(pts[-1] + (i % 2) * 3.0),
                                  strain_relief_mm=float(rng.uniform(0, 60))))
        clamps = sorted({int(c) for c in rng.integers(1, max(n - 1, 2), size=int(rng.integers(0, 3)))})
        h.set_wire(WireRun(f"r{i}", "electrics" if i % 2 else "powertrain",
                           gauge_awg=int(rng.choice([8, 10, 12, 14, 16, 18, 20, 22, 24, 26])),
                           path_mm=[tuple(p) for p in pts], from_conn=a, to_conn=b, net=f"N{i % 4}",
                           od_mm=None if i % 4 else float(rng.uniform(1.0, 6.0)),
                           bundle_min_radius_mult=float(rng.choice([4.0, 6.0, 10.0])),
                           service_loop_mm=float(rng.uniform(0, 80)), strip_mm=float(rng.uniform(4, 12)),
                           carries_current_a=None if i % 3 == 0 else float(rng.uniform(0.5, 60.0)),
                           clamp_idx=clamps or None, is_estimate=bool(i % 2)))
    return h


def _loom_e():
    """One case per remaining verdict: a bend inside 1.5x of its minimum on a measured route, a short exit into a
    strain-relieved connector at the far end, a span with 5 % slack, and a long span that only fouls a box below it
    when it droops."""
    h = HarnessLedger(vib_g=10.0)
    h.set_connector(Connector("A", "electrics", xyz_mm=(0, 0, 500)))
    h.set_connector(Connector("B", "electrics", xyz_mm=(1000, 0, 500), strain_relief_mm=40.0))
    od = H.awg_nominal_od_mm(18)
    r_min = 6.0 * od
    d = 8.0          # two 45 degree turns 11 mm apart: about 1.3 x the minimum radius, inside the 1.5 x margin
    assert r_min < 14.0 < 1.5 * r_min
    h.set_wire(WireRun("warnbend", "electrics", gauge_awg=18, path_mm=[(0, 0, 500), (300, 0, 500),
                                                                      (300 + d, d, 500), (300 + d, 300, 500)],
                       from_conn="A", is_estimate=False))
    h.set_wire(WireRun("shortexit", "electrics", gauge_awg=18, path_mm=[(0, 0, 500), (980, 0, 500),
                                                                       (995, 10, 500), (1000, 0, 500)],
                       from_conn="A", to_conn="B"))
    h.set_wire(WireRun("slack", "electrics", gauge_awg=20, path_mm=[(0, 0, 500), (250, 0, 540), (500, 0, 500),
                                                                   (750, 0, 460), (1000, 0, 500)],
                       from_conn="A", to_conn="B", clamp_idx=[4]))
    h.set_wire(WireRun("drape", "electrics", gauge_awg=10, path_mm=[(0, 200, 500), (1000, 200, 500)],
                       from_conn="A", to_conn="B"))
    return h


def _loom_f():
    """A short, clamped, gentle loom far from every box: the all-clear verdicts."""
    h = HarnessLedger()
    h.set_connector(Connector("P", "electrics", xyz_mm=(0, 0, 0), strain_relief_mm=10.0))
    h.set_connector(Connector("Q", "electrics", xyz_mm=(160, 0, 0), strain_relief_mm=10.0))
    h.set_wire(WireRun("ok", "electrics", gauge_awg=10, path_mm=[(40.0 * i, 0, 0) for i in range(5)],
                       from_conn="P", to_conn="Q", clamp_idx=[1, 2, 3], carries_current_a=1.0))
    return h


def _loom_g():
    """A nine-branch star, a second chain that does not touch it, and runs listed hub-last: the formboard layout."""
    h = HarnessLedger()
    h.set_connector(Connector("HUB", "electrics", xyz_mm=(0, 0, 0), cavities=20))
    for i in range(9):
        a = math.radians(40 * i)
        end = (300 * math.cos(a), 300 * math.sin(a), 20.0 * i)
        h.set_connector(Connector(f"L{i}", "electrics", xyz_mm=end))
        mid = (150 * math.cos(a) + 30, 150 * math.sin(a) - 20, 10.0 * i)
        path = [(0, 0, 0), mid, end]
        if i % 3 == 0:
            h.set_wire(WireRun(f"s{i}", "electrics", gauge_awg=18 + i % 4, path_mm=path[::-1], from_conn=f"L{i}",
                               to_conn="HUB"))
        else:
            h.set_wire(WireRun(f"s{i}", "electrics", gauge_awg=18 + i % 4, path_mm=path, from_conn="HUB",
                               to_conn=f"L{i}"))
    for n in ("X", "Y", "Z"):
        h.set_connector(Connector(n, "powertrain", xyz_mm=(1000 + 100 * "XYZ".index(n), 0, 0)))
    h.set_wire(WireRun("c1", "powertrain", gauge_awg=16, path_mm=[(1000, 0, 0), (1100, 0, 0)], from_conn="X",
                       to_conn="Y"))
    h.set_wire(WireRun("c2", "powertrain", gauge_awg=16, path_mm=[(1100, 0, 0), (1150, 40, 0), (1200, 0, 0)],
                       from_conn="Y", to_conn="Z"))
    h.set_wire(WireRun("solo", "powertrain", gauge_awg=24, path_mm=[(0, 900, 0), (50, 900, 0)]))
    return h


def _loom_h():
    """Order and bookkeeping: a route-less wire listed first, a measured wire whose from-plug is undeclared and whose
    to-end lands 1.5 mm off its plug, two plugs of one part number with known masses, a wire on a subsystem the ledger
    does not know, a zero-length wire, a clamped loop that returns to its own clamp, three out-and-back runs on their
    own plugs, a three-wire chain, a long span that only fouls the box below it when it droops, and non-default flex
    limits."""
    h = HarnessLedger(max_span_mm=150.0, max_sag_mm=0.5, vib_g=4.0)
    h.set_wire(WireRun("aaa_noroute", "electrics", from_conn="J1", to_conn="J2"))
    h.set_connector(Connector("J1", "electrics", xyz_mm=(0, 0, 0), part_number="M12", mass_g=4.0))
    h.set_connector(Connector("J2", "aero", xyz_mm=(300, 0, 0), part_number="M12", mass_g=6.5, strain_relief_mm=200.0))
    h.set_wire(WireRun("offplug", "aero", gauge_awg=16, path_mm=[(-40, 0, 0), (150, 0, 0), (160, 10, 0), (301.5, 0, 0)],
                       from_conn="NOPE", to_conn="J2", is_estimate=False, clamp_idx=["x", 2, 99]))
    h.set_wire(WireRun("dot", "electrics", path_mm=[(5, 5, 5), (5, 5, 5)]))
    h.set_wire(WireRun("loop", "electrics", gauge_awg=18, path_mm=[(100, 50, 0), (150, 50, 0), (150, 80, 0),
                                                                   (100, 50, 0), (200, 50, 0)], clamp_idx=[3]))
    for k, back in ((1, 100.0), (2, 99.0)):
        z = f"Z{k}"
        h.set_wire(WireRun(f"oab{k}", "electrics", gauge_awg=22, path_mm=[(2000, 100 * k, 0), (2100, 100 * k, 0),
                                                                           (2100 - back, 100 * k, 0)],
                           from_conn=z, to_conn=z))
    h.set_wire(WireRun("fb1", "electrics", gauge_awg=20, path_mm=[(0, 900, 0), (60, 930, 0), (100, 900, 0)],
                       from_conn="K1", to_conn="K2"))
    h.set_wire(WireRun("fb2", "electrics", gauge_awg=20, path_mm=[(100, 900, 0), (170, 880, 0), (200, 950, 0)],
                       from_conn="K2", to_conn="K3"))
    h.set_wire(WireRun("fb3", "electrics", gauge_awg=20, path_mm=[(200, 950, 0), (260, 1000, 0)], from_conn="K3",
                       to_conn="K4"))
    h.set_wire(WireRun("over", "electrics", gauge_awg=10, path_mm=[(0, 500, 600), (600, 500, 600)]))
    return h


BOX_H = [KeepOut("belly", "chassis", lo_mm=(250, 450, 400), hi_mm=(350, 550, 577.9)),
         KeepOut("near", "chassis", lo_mm=(140, -20, -20), hi_mm=(155, -5, 20))]


def _loom_i():
    """One short thin wire and nothing else: a loom lighter than a gram still has a centre of mass."""
    h = HarnessLedger()
    h.set_wire(WireRun("hair", "electrics", gauge_awg=30, path_mm=[(1, 2, 3), (4, 2, 3)], strip_mm=0.0))
    return h


BOX_E = [KeepOut("tank", "accumulator", lo_mm=(400, 150, 300), hi_mm=(600, 250, 480))]


BOXES = [KeepOut("accumulator", "accumulator", lo_mm=(1200, 50, 100), hi_mm=(1500, 250, 280)),
         KeepOut("damper", "suspension", lo_mm=(380, -30, 250), hi_mm=(420, 30, 296)),
         KeepOut("far", "chassis", lo_mm=(5000, 5000, 5000), hi_mm=(5100, 5100, 5100))]


def _ledger():
    from suspension.interfaces import IntegrationLedger, SubsystemInterface
    led = IntegrationLedger(ambient_c=55.0)
    led.set(SubsystemInterface("powertrain", peak_current_a=12.5))
    led.set(SubsystemInterface("electrics"))
    return led


def _run(h, keepouts, ledger=None, nets=None):
    r = check_harness(h, keepouts=keepouts, ledger=ledger, net_currents=nets)
    return {"result": _plain(r), "summary": r.summary(), "hard_fail": r.has_hard_fail(),
            "currents": _plain(h.resolve_currents(ledger=ledger, net_currents=nets)),
            "spans": _plain(h.flex_spans()), "as_dict": _plain(h.as_dict()),
            "draped": _plain({n: H.sagged_polyline_mm(w, vib_g=h.vib_g) for n, w in h.wires.items()}),
            "roundtrip": _plain(HarnessLedger.from_dict(json.loads(json.dumps(_plain(h.as_dict())))).as_dict()),
            "bundles": {n: h.bundle_size(w) for n, w in h.wires.items()},
            "formboard_dict": _plain(r.formboard.as_dict()) if r.formboard else None,
            "parts_dict": _plain([c.as_dict() for c in h.connectors.values()] + [w.as_dict() for w in h.wires.values()]),
            "wires": _plain({n: {"od": w.outside_diameter_mm, "minr": w.min_bend_radius_mm,
                          "routed": w.routed_length_mm(), "cut": w.cut_length_mm(),
                          "r20": w.resistance_ohm(), "r90": w.resistance_ohm(90.0), "vd": w.voltage_drop_v(10.0),
                          "cu": w.copper_mass_g(), "ins": w.insulation_mass_g(), "m": w.mass_g(),
                          "sup": w.support_indices(), "mpm": w.mass_per_m_kg(), "ei": w.ei_bounds_nm2(),
                          "c": w.centroid_mm()} for n, w in h.wires.items()})}


def _everything():
    return {"a": _run(_loom_a(), BOXES, _ledger(), {"CAN_H": 0.1, "SENSE": 0.5}),
            "a_nokeep": _run(_loom_a(), None),
            "b": _run(_loom_b(), BOXES[:1], None, {"HV-": 150.0, "TEMP": 1.0}),
            "c": _run(_loom_c(), []),
            "d": _run(_loom_d(), BOXES + [KeepOut("mid", "chassis", lo_mm=(400, -200, 100), hi_mm=(800, 200, 400))],
                      _ledger(), {"N0": 3.0, "N2": 45.0}),
            "e": _run(_loom_e(), BOX_E),
            "f": _run(_loom_f(), BOXES),
            "g": _run(_loom_g(), []),
            "h": _run(_loom_h(), BOX_H, _ledger(), {}),
            "h_nokeep": _run(_loom_h(), None),
            "i": _run(_loom_i(), None)}


def test_every_loom_word_for_word():
    got = json.loads(json.dumps(_everything(), ensure_ascii=False))
    if os.environ.get("KINEMATIK_REGOLD"):
        GOLD.write_text(json.dumps(got, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    want = json.loads(GOLD.read_text(encoding="utf-8"))
    # The checks round to 0.01; a value sitting on a rounding edge can land one step apart on another CPU's float
    # path, so numbers compare to one rounding step and everything else (text, flags, structure) exactly.
    for loom in want:
        for part in want[loom]:
            _same(got[loom][part], want[loom][part], f"{loom}.{part}", rel=1e-6, abs_=0.0101)


def test_editing_the_loom():
    h = _loom_a()
    h.remove_wire("loose")
    h.remove_connector("SPARE")
    assert "loose" not in h.wires and "SPARE" not in h.connectors
    h.remove_wire("absent")
    h.remove_connector("absent")
    assert len(h.wires) == 3 and len(h.connectors) == 5


LO, HI = np.array([0.0, 0.0, 0.0]), np.array([10.0, 20.0, 30.0])


def test_signed_distance_to_a_box():
    pts = {(15, 25, 35): math.sqrt(75.0), (5, 5, 5): -5.0, (-3, 10, 10): 3.0, (10.5, 10, 10): 0.5,
           (9, 10, 10): -1.0, (5, 19, 15): -1.0, (5, 10, 29.5): -0.5, (5, 10, 31): 1.0}
    for p, want in pts.items():
        assert H._point_aabb_signed_dist_mm(np.array(p, float), LO, HI) == pytest.approx(want, abs=1e-12)
    assert H._point_aabb_signed_dist_mm(np.array([10.0, 10.0, 10.0]), LO, HI) == 0.0
    rng = np.random.default_rng(1)
    cloud = rng.uniform(-15.0, 45.0, size=(200, 3))
    batch = H._signed_dist_batch(cloud, LO, HI)
    assert batch == pytest.approx([H._point_aabb_signed_dist_mm(p, LO, HI) for p in cloud], abs=1e-12)


def test_the_closest_approach_of_a_route_to_a_box():
    through = np.array([[-10.0, 10.0, 15.0], [20.0, 10.0, 15.0]])
    gap, at = H._polyline_aabb_clearance_detail(through, LO, HI)
    assert gap == pytest.approx(-5.0, abs=1e-9) and at[1:] == pytest.approx([10.0, 15.0], abs=1e-9)
    assert 0.0 <= at[0] <= 10.0 and H._polyline_aabb_clearance_mm(through, LO, HI) == gap
    beside = np.array([[-5.0, -5.0, 15.0], [3.0, -5.0, 15.0], [15.0, -5.0, 15.0]])
    assert H._polyline_aabb_clearance_detail(beside, LO, HI)[0] == pytest.approx(5.0, abs=1e-9)
    corner = np.array([[-10.0, -10.0, 40.0], [20.0, -10.0, 40.0]])
    g, p = H._polyline_aabb_clearance_detail(corner, LO, HI)
    assert g == pytest.approx(math.hypot(10.0, 10.0), abs=1e-9) and -1e-6 <= p[0] <= 10.0 + 1e-6
    end = np.array([[12.0, 25.0, 35.0], [40.0, 60.0, 90.0]])
    g, p = H._polyline_aabb_clearance_detail(end, LO, HI)
    assert g == pytest.approx(math.sqrt(4 + 25 + 25), abs=1e-9) and p == pytest.approx([12.0, 25.0, 35.0])
    assert H._polyline_aabb_clearance_detail(np.array([[1.0, 1.0, 1.0]]), LO, HI) == (math.inf, None)


def test_turn_angles_and_span_midpoints_on_degenerate_routes():
    assert H.turn_angles_deg(np.array([[0.0, 0, 0], [1.0, 0, 0]])).shape == (0,)
    dup = np.array([[0.0, 0, 0], [1.0, 0, 0], [1.0, 0, 0], [1.0, 1.0, 0], [0.0, 1.0, 0], [5.0, 1.0, 0]])
    assert H.turn_angles_deg(dup) == pytest.approx([0.0, 0.0, 90.0, 180.0])
    back = np.array([[0.0, 0, 0], [1e-3, 0, 0], [0.0, 0, 0]])
    assert H.turn_angles_deg(back) == pytest.approx([180.0])
    assert H._span_mid_point(np.array([[2.0, 2, 2], [2.0, 2, 2]])) == pytest.approx([2.0, 2.0, 2.0])
    assert H._span_mid_point(np.array([[0.0, 0, 0], [0.0, 0, 0], [4.0, 0, 0]])) == pytest.approx([2.0, 0.0, 0.0])
    assert H._span_mid_point(np.array([[0.0, 0, 0], [1.0, 0, 0], [3.0, 0, 0]])) == pytest.approx([1.5, 0.0, 0.0])
    assert H._span_mid_point(np.array([[0.0, 0, 0], [2.0, 0, 0], [4.0, 0, 0]])) == pytest.approx([2.0, 0.0, 0.0])


def test_the_summary_with_part_of_its_totals():
    r = H.HarnessCheckResult(findings=[], bom={"total_wire_m": 1.5}, mass={})
    assert r.summary() == H.HarnessCheckResult(findings=[]).summary()
    assert "m wire" not in r.summary()
    r2 = H.HarnessCheckResult(findings=[], bom={}, mass={"total_copper_g": 3.0})
    assert "copper" not in r2.summary()


def test_the_new_looms_reach_their_verdicts():
    r = check_harness(_loom_h(), keepouts=BOX_H)
    codes = {(f.check, f.detail.get("wire")) for f in r.findings}
    assert ("harness-sag-clearance", "over") in codes and ("harness-anchor", "offplug") in codes
    assert ("harness-strain-relief", "offplug") in codes and ("harness-route", None) in codes
    assert _loom_h().wires["offplug"].support_indices() == [0, 2, 3]
    assert check_harness(_loom_i()).mass["harness_cg_mm"] == [2.5, 2.0, 3.0]
    ties = {t["connector"] for t in r.formboard.ties}
    assert ties == {"Z2"}


def test_flex_defaults_and_degenerate_geometry():
    w = _loom_h().wires["over"]
    assert H.wire_span_flex(w) == H.wire_span_flex(w, vib_g=3.0, band_hz=(20.0, 200.0))
    assert H.wire_span_flex(w) != H.wire_span_flex(w, vib_g=3.0, band_hz=(30.0, 200.0))
    assert np.array_equal(H.sagged_polyline_mm(w), H.sagged_polyline_mm(w, vib_g=3.0))
    assert H.wire_span_flex(w) != H.wire_span_flex(w, vib_g=4.0)
    h = HarnessLedger()
    h.set_connector(Connector("C", "electrics", xyz_mm=(0, 0, 0)))
    h.set_wire(WireRun("w", "electrics", path_mm=[(1.5, 0, 0), (50, 0, 0)], from_conn="C"))
    assert [f.check for f in h.check_anchoring()] == ["harness-anchor"] and h.check_anchoring(tol_mm=2.0) == []
    assert WireRun("e", "electrics").as_polyline().shape == (0, 3)
    assert H.segment_lengths_mm(np.zeros((1, 3))).shape == (0,)
    flat = H._unfold_branch_2d(np.zeros((1, 3)), np.array([5.0, 6.0]), 30.0)
    assert flat.shape == (1, 2) and flat.tolist() == [[5.0, 6.0]]
    zig = np.array([[0.0, 0, 0], [10.0, 0, 0], [10.0, 10.0, 0]])
    assert H._unfold_branch_2d(zig, np.zeros(2), 0.0, target_heading_deg=-90.0)[-1] == pytest.approx([10.0, -10.0])
    assert H._unfold_branch_2d(zig, np.zeros(2), 0.0)[-1] == pytest.approx([10.0, 10.0])
    a = np.array([2.739233746429086, -4.604265724722594, -9.180529521276107])
    fold = np.array([np.zeros(3), a, a - a * 0.26362359173243805])
    assert H.turn_angles_deg(fold) == pytest.approx([180.0])
    shallow = np.array([[0.0, 0, 0], [100.0, 0, 0], [200.0, 200.0 * math.tan(math.radians(0.5)), 0]])
    assert math.isfinite(H.vertex_bend_radius_mm(shallow)[0])
    assert WireRun("z", "electrics", path_mm=[(1, 1, 1), (1, 1, 1)]).centroid_mm() is None
    assert WireRun("t", "electrics", path_mm=[(0, 0, 0), (0.5, 0, 0)]).centroid_mm() == pytest.approx([0.25, 0, 0])


def test_parts_from_plain_dicts():
    c = Connector.from_dict({"name": "C", "owner_subsystem": "electrics", "xyz_mm": [1, 2, 3], "junk": 1})
    assert c.xyz_mm == (1, 2, 3) and isinstance(c.xyz_mm, tuple)
    w = WireRun.from_dict({"name": "w", "owner_subsystem": "electrics", "path_mm": [[0, 0, 0], [1, 0, 0]]})
    assert w.path_mm == [(0, 0, 0), (1, 0, 0)] and all(isinstance(p, tuple) for p in w.path_mm)


def test_signed_distance_near_an_offset_box():
    lo, hi = np.array([1.0, 2.0, 3.0]), np.array([11.0, 22.0, 33.0])
    assert H._point_aabb_signed_dist_mm(np.array([10.0, 20.0, 30.0]), lo, hi) == pytest.approx(-1.0)
    assert H._point_aabb_signed_dist_mm(np.array([2.5, 12.0, 18.0]), lo, hi) == pytest.approx(-1.5)
    assert H._point_aabb_signed_dist_mm(np.array([11.5, 22.5, 18.0]), lo, hi) == pytest.approx(math.sqrt(0.5))
    near = np.array([[11.5, 22.5, 18.0], [0.4, 12.0, 18.0], [6.0, 12.0, 33.9]])
    assert H._signed_dist_batch(near, lo, hi) == pytest.approx([math.sqrt(0.5), 0.6, 0.9])
