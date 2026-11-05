"""The FASTSim benchmark (scripts/fastsim_benchmark.py, docs/FASTSIM_BENCHMARK.md) keeps its findings and its
safeguards: FASTSim's stored runs are pinned by SHA-256, KinematiK reproduces FASTSim's battery and fuel energy on
every vehicle and cycle, the road-load layer agrees to a fraction of a percent, and a step-interpolated FASTSim map
is carried into KinematiK's linear table without changing it."""
import importlib.util
import json
import shutil
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location("fastsim_benchmark", ROOT / "scripts" / "fastsim_benchmark.py")
fb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fb)
LIMIT = json.loads((fb.CYCLE_DIR / "references.json").read_text())["tolerance"]["limit"]


@pytest.fixture(scope="module")
def rows():
    return fb.rows()


def test_the_manifest_pins_every_stored_run_and_cycle(tmp_path, monkeypatch):
    assert fb.verify_manifest() == []
    names = [ln.split(maxsplit=1)[1] for ln in fb.MANIFEST.read_text().splitlines() if ln.strip()]
    runs = fb._runs()
    assert runs["fastsim_version"] == fb.FASTSIM_VERSION
    assert set(runs["vehicles"]) == set(fb.VEHICLES)
    assert {f"fastsim/{k}_{c}.csv" for k in fb.VEHICLES for c in fb.CYCLES} <= set(names)
    assert {"fastsim/fastsim_runs.json"} | {f"gt_suite/{c}" for c in fb.CYCLES.values()} <= set(names)
    root = tmp_path / "benchmarks"
    shutil.copytree(fb.BENCH, root / "fastsim")
    shutil.copytree(fb.CYCLE_DIR, root / "gt_suite")
    monkeypatch.setattr(fb, "ROOT", tmp_path)
    monkeypatch.setattr(fb, "MANIFEST", root / "fastsim" / "MANIFEST.sha256")
    assert fb.verify_manifest() == []
    p = root / "fastsim" / "fastsim_runs.json"
    p.write_text(p.read_text().replace('"energy_wh_km": 1', '"energy_wh_km": 2', 1))
    assert fb.verify_manifest() == ["fastsim/fastsim_runs.json"]


def test_every_vehicle_and_cycle_is_graded_and_passes(rows):
    assert len(rows) == len(fb.VEHICLES) * len(fb.CYCLES)
    assert {r["powertrain"] for r in rows} == {"ev", "ice"}
    for r in rows:
        assert abs(r["err"]) <= LIMIT, (r["vehicle"], r["cycle"], r["err"])
    e = np.array([r["err"] for r in rows])
    assert np.mean(np.abs(e)) < 0.002                    # the findings: same physics, 0.02 % mean, 0.04 % worst
    assert np.max(np.abs(e)) < 0.002


def test_the_road_load_layer_matches_on_its_own(rows):
    for r in rows:
        assert r["tractive_k"] == pytest.approx(r["tractive_fs"], rel=1e-3), (r["vehicle"], r["cycle"])


def test_the_willans_rows_are_reported_for_the_petrol_cars(rows):
    ice_rows = [r for r in rows if r["powertrain"] == "ice"]
    assert ice_rows and all("willans_k" in r for r in ice_rows)
    assert all("willans_k" not in r for r in rows if r["powertrain"] == "ev")
    udds = next(r for r in ice_rows if r["vehicle"] == "fusion" and r["cycle"] == "udds")
    assert udds["willans_k"] / udds["energy_fs"] - 1 < -0.1   # the straight line misses FASTSim's low-load curve


def test_a_step_map_becomes_a_staircase():
    frac, eff = fb._linear_table([0.0, 0.2, 0.5, 1.0], [0.8, 0.9, 0.95, 0.93], "Step-Lower")
    x = np.array([0.0, 0.1, 0.1999, 0.2, 0.35, 0.5, 0.75, 1.0, 1.2])
    step = np.array([0.8, 0.8, 0.8, 0.9, 0.9, 0.95, 0.95, 0.93, 0.93])
    assert np.interp(x, frac, eff) == pytest.approx(step, abs=1e-6)
    lin = fb._linear_table([0.0, 1.0], [0.8, 0.9], "Linear")
    assert np.interp(0.5, *lin) == pytest.approx(0.85)
    with pytest.raises(ValueError):
        fb._linear_table([0.0, 1.0], [0.8, 0.9], "Cubic")


def test_the_report_passes(capsys):
    assert fb.main([]) == 0
    out = capsys.readouterr().out
    assert f"{len(fb.VEHICLES) * len(fb.CYCLES)}/{len(fb.VEHICLES) * len(fb.CYCLES)} within" in out
