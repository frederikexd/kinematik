"""Characterization suite for suspension/throttle_flutter_cosim.py (mutation gate); see tests/_golden.py."""
import json
import os
import tempfile

import pytest

from suspension import throttle_flutter_cosim as tf
from tests._golden import check, plain


def _norm(x, tmp_path):
    return json.loads(json.dumps(plain(x)).replace(str(tmp_path), "<T>"))

F = tf.FlutterFidelity


def _files(d):
    return {n: open(os.path.join(d, n)).read() for n in sorted(os.listdir(d))}


def test_provenance_case_and_derivative():
    res = {}
    provs = [tf.FlutterProvenance("b", F.URANS), tf.FlutterProvenance("b", F.DES, True, True, "sst", 2_500_000, "rig", "n"),
             tf.FlutterProvenance("b", F.EXPERIMENT, True, False), tf.FlutterProvenance("c", F.QUASI_STEADY, cell_count=0)]
    res["prov"] = [[p.status(), p.as_dict()] for p in provs]
    res["case"] = [tf.OscillationCase().as_dict(), tf.OscillationCase(30.0, 1.0, 80.0, 25.0, 0.015, 7e-4, 1.1).as_dict()]
    d = tf.FlutterDerivative(-0.002, 40.0, 1.0, 2.0, provs[1], [])
    res["deriv"] = [d.as_dict(), d.is_usable(), d.to_flutter_params(5.0), d.to_flutter_params(5.0, 2e-3),
                    tf.FlutterDerivative(None, 1.0, provenance=provs[1]).is_usable(),
                    tf.FlutterDerivative(1.0, 1.0).is_usable(), tf.FlutterDerivative(1.0, 1.0, provenance=provs[2]).is_usable(),
                    tf.FlutterDerivative(1.0, 1.0).as_dict()]
    with pytest.raises(ValueError) as e:
        tf.FlutterDerivative(None, 1.0).to_flutter_params(1.0)
    res["err"] = str(e.value)
    res["enum"] = [f.value for f in F]
    check("flutter_basic", res)


def test_backends(tmp_path):
    res = {}
    cases = [tf.OscillationCase(), tf.OscillationCase(30.0, 1.0, 80.0, 25.0, 0.015, 7e-4, 1.1),
             tf.OscillationCase(intake_speed_ms=0.0), tf.OscillationCase(plate_area_m2=-1.0, plate_radius_m=-1.0)]
    models = [tf.QuasiSteadyFlutterModel(), tf.QuasiSteadyFlutterModel(1.3, True), tf.QuasiSteadyFlutterModel("2", 1)]
    for i, m in enumerate(models):
        res[f"q{i}"] = [m.name, m.moment_slope_per_rad, m.reduces_stability, m.provenance().as_dict(),
                        [m._derivative(c) for c in cases], [m.run_case(c, "unused").as_dict() for c in cases],
                        m.read_result(cases[1], "unused").as_dict()]
    d = tmp_path / "q"
    p1 = models[0].write_case(cases[1], str(d))
    p2 = models[0].write_case(cases[0], str(d))
    res["qfiles"] = [os.path.basename(p1), p1 == p2, _files(d)]
    ext = [tf.ExternalCFDFlutterBackend(), tf.ExternalCFDFlutterBackend("fluent", F.DES)]
    for i, b in enumerate(ext):
        w = tmp_path / f"e{i}"
        p = b.write_case(cases[1], str(w))
        b.write_case(cases[1], str(w))
        res[f"e{i}"] = [b.name, b.provenance().as_dict(), os.path.basename(p), _files(w),
                        b.case_artifacts(cases[0]), b.case_artifacts(cases[1])]
        with pytest.raises(tf.SolverUnavailable) as e:
            b.run_case(cases[0], str(w))
        res[f"e{i}_run"] = str(e.value)
        with pytest.raises(tf.SolverUnavailable) as e:
            b.read_result(cases[0], str(w))
        res[f"e{i}_noresult"] = str(e.value).replace(str(tmp_path), "<T>")
        for j, data in enumerate([{"c_aero_Nms": -0.003, "converged": True, "cell_count": 1200000,
                                   "turbulence_model": "sst", "is_correlated": True, "correlated_against": "rig"},
                                  {"c_aero_Nms": 0.001}, {}]):
            (w / f"{b.name}_result.json").write_text(json.dumps(data))
            res[f"e{i}_r{j}"] = b.read_result(cases[1], str(w)).as_dict()
    check("flutter_backends", _norm(res, tmp_path))


def test_extract(tmp_path, monkeypatch):
    res = {}
    c = tf.OscillationCase(frequency_hz=60.0)
    res["default"] = tf.extract_flutter_derivative(c, workdir=str(tmp_path / "a")).as_dict()
    res["ext"] = tf.extract_flutter_derivative(c, tf.ExternalCFDFlutterBackend(), str(tmp_path / "b")).as_dict()
    seen = []
    real = tempfile.mkdtemp
    monkeypatch.setattr(tempfile, "mkdtemp", lambda **kw: seen.append(kw) or real(**kw))
    res["tmp"] = tf.extract_flutter_derivative(c).as_dict()
    res["seen"] = seen
    res["files"] = sorted(os.listdir(tmp_path / "a")) + sorted(os.listdir(tmp_path / "b"))
    check("flutter_extract", _norm(res, tmp_path))


def test_the_cell_count_in_the_status_line():
    p = tf.FlutterProvenance("rans", list(tf.FlutterFidelity)[0], cell_count=1_050_000)
    assert ", 1.1M cells" in p.status()
