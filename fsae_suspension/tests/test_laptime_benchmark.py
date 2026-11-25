"""The lap solver against TUM's lap time simulation and an independent integration (scripts/laptime_benchmark.py)."""
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))
STORED = json.loads((ROOT / "benchmarks" / "laptime" / "results.json").read_text())


def test_the_stored_run_passed():
    assert STORED["passed"] and STORED["vs_tum"]["passed"] and STORED["vs_exact"]["passed"]


def test_live_kinematik_matches_the_stored_references():
    import laptime_benchmark as lb
    k = lb.kinematik()
    assert k["lap_time_s"] == pytest.approx(STORED["exact"]["lap_time_s"], rel=0.005)
    assert k["lap_time_s"] == pytest.approx(STORED["tum"]["lap_time_s"], rel=0.02)
    for c, v in STORED["exact"]["corner_min_speed_ms"].items():
        assert k["corner_min_speed_ms"][c] == pytest.approx(v, rel=0.005)


def test_braking_reaches_corner_speed_at_the_corner_not_a_station_early():
    """The last step of a straight is straight: it brakes with the full grip, so the station before the corner is
    faster than the corner. Before the fix it braked as if already cornering and arrived at corner speed early."""
    import laptime_benchmark as lb
    from types import SimpleNamespace
    from suspension import laptime as lt
    m, mu = 280.0, 1.5
    pt = lt.Powertrain(power_kw=60.0, max_tractive_n=1e9, drivetrain_eff=1.0, cla=0.0, cda=0.0, crr=0.0,
                       brake_g_cap=100.0)
    veh = SimpleNamespace(p=SimpleNamespace(mass=m, weight_dist_front=0.46), max_lateral_g=lambda: mu)
    vs = list(lt._LAT_TABLE_V)
    veh._kx_lat_limit_table = (vs, [mu] * len(vs), round(mu, 9))
    r = lt.simulate_lap(veh, lt.Track("t", [lt.Segment(60.0), lt.Segment(math.pi * 10.0, 10.0), lt.Segment(60.0)],
                                      ds=1.0), pt)
    v_corner = math.sqrt(mu * 9.81 * 10.0)
    i = r.s.index(60.0)
    assert r.v[i] == pytest.approx(v_corner, rel=1e-6)
    assert r.v[i - 1] == pytest.approx(math.sqrt(v_corner ** 2 + 2 * mu * 9.81 * 1.0), rel=1e-6)
