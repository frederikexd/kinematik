"""Files a team uploads are untrusted. A value that is not a finite number must never become part of a model, and
no upload may exhaust the server's memory. Each case here was found by fuzzing the parsers with damaged files."""
import io
import math
import zipfile

import numpy as np
import pytest

from suspension import cad_ingest
from suspension.aero import run_log
from suspension.aero.aeromap import AeroMap

HEAD = "roll_deg,pitch_deg,yaw_deg,ride_height_mm,speed_ms,c_lift,c_drag,c_side,aero_balance_front,converged\n"


def test_aero_map_csv_reads_a_clean_file():
    m = AeroMap.from_csv(HEAD + "0,0,0,30,20,2.5,1.1,0,0.45,1\n0,1,0,30,20,2.4,1.1,,0.44,1\n")
    assert len(m) == 2


@pytest.mark.parametrize("cell", ["nan", "inf", "-inf", "1e999", "abc"])
@pytest.mark.parametrize("col", range(9))
def test_aero_map_csv_refuses_a_value_that_is_not_a_finite_number(cell, col):
    row = ["0", "0", "0", "30", "20", "2.5", "1.1", "0", "0.45", "1"]
    row[col] = cell
    name = HEAD.split(",")[col]
    with pytest.raises(ValueError, match=rf"row 2, column {name}: '{cell}' is not a (finite )?number"):
        AeroMap.from_csv(HEAD + ",".join(row) + "\n")


@pytest.mark.parametrize("speed", ["0", "-20"])
def test_aero_map_csv_refuses_a_speed_that_is_not_positive(speed):
    with pytest.raises(ValueError, match="row 3, column speed_ms"):
        AeroMap.from_csv(HEAD + "0,0,0,30,20,2.5,1.1,0,0.45,1\n" + f"0,0,0,30,{speed},2.5,1.1,0,0.45,1\n")


def test_aero_map_csv_that_is_not_csv_is_a_value_error():
    with pytest.raises(ValueError, match="not a readable CSV"):
        AeroMap.from_csv(HEAD + "0,0,0,30,20," + "9" * 200_000 + "\n")      # one field beyond csv's size limit


def test_aero_map_csv_keeps_its_documented_defaults_for_empty_cells():
    m = AeroMap.from_csv("c_lift,c_drag\n2.5,1.1\n")
    (att,) = [r.attitude for r in m._points.values()]
    assert (att.roll_deg, att.pitch_deg, att.yaw_deg, att.ride_height_mm, att.speed_ms) == (0.0, 0.0, 0.0, 30.0, 20.0)


@pytest.mark.parametrize("value", [float("inf"), float("-inf"), float("nan"), "1e999", "-1e999", "nan"])
def test_run_log_number_that_is_not_finite_is_no_value(value):
    assert run_log._coerce_number(value) is None


def test_run_log_still_reads_ordinary_numbers():
    assert run_log._coerce_number("26,8 m/s") == pytest.approx(26.8)
    assert run_log._coerce_number("45 %") == pytest.approx(0.45)
    assert run_log._coerce_number(3) == 3.0 and math.isfinite(run_log._coerce_number("1e30"))


def _zip(members):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for name, data in members:
            zf.writestr(name, data)
    return buf.getvalue()


def test_zip_member_larger_than_the_limit_is_skipped_not_unpacked(monkeypatch):
    monkeypatch.setattr(cad_ingest, "MAX_MEMBER_BYTES", 1000)
    man = cad_ingest.ingest_bundle("b.zip", _zip([("small.txt", b"x" * 500), ("bomb.txt", b"\0" * 200_000)]))
    assert man["file_count"] == 1
    assert man["skipped"] == [{"name": "bomb.txt", "reason": "too large to unpack safely"}]


def test_zip_total_larger_than_the_limit_stops_unpacking(monkeypatch):
    monkeypatch.setattr(cad_ingest, "MAX_BUNDLE_BYTES", 1200)
    man = cad_ingest.ingest_bundle("b.zip", _zip([(f"f{i}.txt", b"x" * 500) for i in range(4)]))
    assert man["file_count"] == 2 and len(man["skipped"]) == 2
    assert man["total_bytes"] <= 1200


def test_zip_member_whose_header_lies_about_its_size_is_still_bounded(monkeypatch):
    raw = bytearray(_zip([("liar.txt", b"\0" * 100_000)]))
    # the uncompressed-size field sits 22 bytes into the local header and 24 into the central directory entry
    for sig, off in ((b"PK\x03\x04", 22), (b"PK\x01\x02", 24)):
        i = raw.find(sig)
        raw[i + off:i + off + 4] = (10).to_bytes(4, "little")
    monkeypatch.setattr(cad_ingest, "MAX_MEMBER_BYTES", 1000)
    man = cad_ingest.ingest_bundle("b.zip", bytes(raw))
    assert man["total_bytes"] <= 1000                        # whatever zipfile makes of it, no more than the limit is kept


def test_a_zip_python_cannot_open_is_kept_as_one_opaque_file():
    raw = bytearray(_zip([("a.txt", b"hello")]))
    i = raw.find(b"PK\x01\x02")
    raw[i + 10:i + 12] = (99).to_bytes(2, "little")           # an unknown compression method
    j = raw.find(b"PK\x03\x04")
    raw[j + 8:j + 10] = (99).to_bytes(2, "little")
    man = cad_ingest.ingest_bundle("b.zip", bytes(raw))
    assert man["file_count"] in (0, 1)                        # skipped member or one opaque file, never a crash


# --- dyno CSV (engine map) ------------------------------------------------------------------------------------ #
from suspension import ice_powertrain as ice  # noqa: E402


@pytest.mark.parametrize("text,why", [
    ("", "header row and at least one row"),
    ("rpm,torque_nm\n", "header row and at least one row"),
    ("rpm,torque_nm\n3000\n6000,50", "row 2 has fewer values than the header has columns \\(2\\)"),
    ("rpm,torque_nm\n3000,40\n6000", "row 3 has fewer values"),
    ("speed,torque_nm\n3000,40\n6000,50", "no rpm column"),
    ("rpm,torque_nm\n3000,nan\n6000,50", "not a finite number"),
    ("rpm,torque_nm\n3000,40\n6000,inf", "not a finite number"),
    ("rpm,torque_nm\n1e999,40\n6000,50", "not a finite number"),
    ("rpm,torque_nm\n-3000,40\n6000,50", "cannot be negative"),
])
def test_dyno_csv_refuses_what_it_cannot_trust(text, why):
    with pytest.raises(ValueError, match=why):
        ice.EngineMap.from_csv(text)


def test_dyno_csv_still_reads_a_clean_file_and_extra_columns():
    e = ice.EngineMap.from_csv("rpm,torque_nm,note\n6000,60,1\n2000,40,2\n")
    assert e.rpm == [2000.0, 6000.0] and e.torque_nm == [40.0, 60.0]


# --- hardpoint tables (CSV / Excel) --------------------------------------------------------------------------- #
from suspension import hardpoint_import  # noqa: E402


@pytest.mark.parametrize("data,name", [(b"PK\x03\x04" + b"\x00" * 200, "book.xlsx"), (b"PK\x05\x06" + b"\x00" * 18, "x.xlsx"),
                                       (b"not a workbook at all", "pts.xlsx")])
def test_a_damaged_workbook_is_a_value_error_not_a_zip_traceback(data, name):
    with pytest.raises(ValueError, match="not a readable Excel workbook"):
        hardpoint_import.parse_tabular(data, name)


def test_a_csv_the_reader_cannot_walk_is_a_value_error():
    with pytest.raises(ValueError, match="not a readable CSV"):
        hardpoint_import.parse_tabular(b"Point,X,Y,Z\nA,1,2," + b"9" * 200_000 + b"\n", "pts.csv")


# --- STEP tube import and reproducibility manifests ----------------------------------------------------------- #
from suspension import genesis_analysis as ga  # noqa: E402
from suspension import genesis_repro  # noqa: E402

STEP = """ISO-10303-21;
HEADER;
ENDSEC;
DATA;
#9000=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#1=CARTESIAN_POINT('',(0.0,0.0,0.0));
#2=CARTESIAN_POINT('',(0.0,0.0,0.0));
#3=DIRECTION('',(1.0,0.0,0.0));
#4=AXIS2_PLACEMENT_3D('',#2,#3,$);
#5=CYLINDRICAL_SURFACE('',#4,12.7);
#6=CARTESIAN_POINT('',(0.0,-12.7,0.0));
#7=VERTEX_POINT('',#6);
#8=CARTESIAN_POINT('',(500.0,-12.7,0.0));
#9=VERTEX_POINT('',#8);
#10=LINE('',#1,#1);
#11=EDGE_CURVE('',#7,#9,#10,.T.);
#12=ORIENTED_EDGE('',*,*,#11,.T.);
#13=LINE('',#1,#1);
#14=EDGE_CURVE('',#9,#7,#13,.T.);
#15=ORIENTED_EDGE('',*,*,#14,.T.);
#16=EDGE_LOOP('',(#12,#15));
#17=FACE_OUTER_BOUND('',#16,.T.);
#18=ADVANCED_FACE('',(#17),#5,.T.);
ENDSEC;
END-ISO-10303-21;
"""


def test_step_tube_import_reads_the_clean_file():
    out = ga.parse_step_tubes(STEP, outer_radius_mm=12.7)
    assert len(out.axes) == 1
    p0, p1, _wall = out.axes[0]
    assert abs(float(np.linalg.norm(np.asarray(p1) - np.asarray(p0))) - 500.0) < 1e-6


@pytest.mark.parametrize("damage", [
    lambda s: s.replace("#8=CARTESIAN_POINT('',(500.0,-12.7,0.0));\n", ""),      # a vertex points at nothing
    lambda s: s.replace("#3=DIRECTION('',(1.0,0.0,0.0));\n", ""),                # the axis has no direction
    lambda s: s.replace("#5=CYLINDRICAL_SURFACE('',#4,12.7);", "#5=CYLINDRICAL_SURFACE('',#4);"),
])
def test_step_with_a_missing_or_truncated_entity_is_a_value_error(damage):
    with pytest.raises(ValueError, match="damaged or incomplete"):
        ga.parse_step_tubes(damage(STEP), outer_radius_mm=12.7)


@pytest.mark.parametrize("bad", ["1e999", "nan", "inf"])
def test_step_with_a_coordinate_that_is_not_finite_is_refused(bad):
    with pytest.raises(ValueError):
        out = ga.parse_step_tubes(STEP.replace("(500.0,-12.7,0.0)", f"({bad},-12.7,0.0)"), outer_radius_mm=12.7)
        assert np.isfinite(np.asarray(out.vertices, float)).all()      # or, if it was read, nothing non-finite got in
        raise ValueError("read without the bad value")


def test_manifest_with_nan_or_infinity_is_refused():
    schema = genesis_repro.MANIFEST_SCHEMA
    for token in ("NaN", "Infinity", "-Infinity"):
        with pytest.raises(ValueError, match="not a finite number"):
            genesis_repro.GenesisManifest.from_json(f'{{"schema": "{schema}", "name": "x", "volume": {token}}}')


@pytest.mark.parametrize("text,why", [
    ("[]", "Not a genesis manifest"),
    ("3", "Not a genesis manifest"),
    (f'{{"schema": "{genesis_repro.MANIFEST_SCHEMA}"}}', "has no 'name' entry"),
    (f'{{"schema": "{genesis_repro.MANIFEST_SCHEMA}", "name": "x", "hardpoints": {{}}, "targets": {{}}, "volume": {{}},'
     ' "search": {"no_such": 1}}', "search settings are not readable"),
])
def test_manifest_that_is_not_one_is_a_value_error(text, why):
    with pytest.raises(ValueError, match=why):
        genesis_repro.GenesisManifest.from_json(text)


# --- chassis frame CSVs (the frame that goes on to FEA) ------------------------------------------------------- #
from suspension.tubeframe import FrameGraph  # noqa: E402

NODES = "id,x,y,z\na,0,0,0\nb,500,0,0\nc,0,400,300\n"
TUBES = "name,a,b,class,size\nt1,a,b,side_impact,A\nt2,b,c,non_structural,C\n"


def test_frame_csv_reads_a_clean_pair():
    g = FrameGraph.from_csv(NODES, TUBES)
    assert sorted(g.nodes) == ["a", "b", "c"] and len(g.tubes) == 2


@pytest.mark.parametrize("bad", ["nan", "inf", "-inf", "1e999"])
@pytest.mark.parametrize("axis", ["x", "y", "z"])
def test_frame_node_that_is_not_at_a_finite_position_is_refused(bad, axis):
    row = {"x": "500", "y": "0", "z": "0"}
    row[axis] = bad
    with pytest.raises(ValueError, match="nodes CSV row 3: node 'b': its position must be three finite numbers"):
        FrameGraph.from_csv(f"id,x,y,z\na,0,0,0\nb,{row['x']},{row['y']},{row['z']}\n", "name,a,b\n")


@pytest.mark.parametrize("nodes,tubes,why", [
    ("id,x,y\na,0,0\n", TUBES, "nodes CSV row 2: no value for z"),
    ("id,x,y,z\na,0,0,0\nb,5,,0\n", TUBES, "nodes CSV row 3: no value for y"),
    ("id,x,y,z\na,0,zero,0\n", TUBES, "nodes CSV row 2: could not convert"),
    (NODES, "name,a\nt1,a\n", "tubes CSV row 2: no value for b"),
    (NODES, "name,a,b\nt1,a,zz\n", "tubes CSV row 2: tube 't1': unknown node"),
    (NODES, "name,a,b\nt1,a,a\n", "tubes CSV row 2: tube 't1': both ends on node 'a'"),
    (NODES, "name,a,b\nt1,a," + "9" * 200_000 + "\n", "tubes file is not a readable CSV"),
])
def test_frame_csv_refuses_with_the_row_and_the_reason(nodes, tubes, why):
    with pytest.raises(ValueError, match=why):
        FrameGraph.from_csv(nodes, tubes)


def test_add_node_refuses_a_non_finite_position_on_every_path():
    with pytest.raises(ValueError, match="three finite numbers"):
        FrameGraph().add_node("n", (0.0, float("nan"), 0.0))


# --- the shared guards (suspension/_num.py) ------------------------------------------------------------------- #
from suspension import _num  # noqa: E402


def test_finite_returns_a_flat_float_array_and_leaves_units_alone():
    a = _num.finite("x", [[1, 2], [3, 4]])
    assert a.dtype == float and a.tolist() == [1.0, 2.0, 3.0, 4.0]
    assert _num.finite("x", [5.0], 1).tolist() == [5.0]
    assert _num.finite("x", [-3.0, 3.0], 2, 3.0).tolist() == [-3.0, 3.0]          # the bound itself is allowed


@pytest.mark.parametrize("values,min_len,max_abs,why", [
    ([1.0, float("nan"), 3.0], 1, None, r"x: 1 of 3 value\(s\) are not finite numbers"),
    ([float("inf"), float("-inf")], 1, None, r"x: 2 of 2 value\(s\) are not finite"),
    ([], 1, None, r"x: needs at least 1 value\(s\), got 0"),
    ([1.0, 2.0], 3, None, r"x: needs at least 3 value\(s\), got 2"),
    (["a", "b"], 1, None, "x: not a list of numbers"),
    ([[1, 2], [3]], 1, None, "x: not a list of numbers"),
    ([1.0, -7.0, 2.0], 1, 5.0, r"x: a value of -7 is beyond ±5"),
])
def test_finite_refuses_and_names_the_input(values, min_len, max_abs, why):
    with pytest.raises(ValueError, match=why):
        _num.finite("x", values, min_len, max_abs)


def test_finite_scalar():
    assert _num.finite_scalar("m", "2.5") == 2.5 and _num.finite_scalar("m", -1) == -1.0
    assert _num.finite_scalar("m", 1e-9, positive=True) == 1e-9
    for bad, why in ((float("nan"), "m: nan is not a finite number"), (float("inf"), "m: inf is not a finite"),
                     ("abc", "m: not a number"), (None, "m: not a number")):
        with pytest.raises(ValueError, match=why):
            _num.finite_scalar("m", bad)
    for bad in (0, -2.0):
        with pytest.raises(ValueError, match="m: must be above zero, got"):
            _num.finite_scalar("m", bad, positive=True)


def test_all_finite_walks_every_container():
    from dataclasses import dataclass

    @dataclass
    class Box:
        v: object

    assert _num.all_finite([1.0, (2, {"a": np.array([1.0, 2.0])}), Box(3.0), "nan", None, True, np.array([1, 2])])
    for bad in (float("nan"), np.float64("inf"), [1.0, float("inf")], (float("nan"),), {"a": float("-inf")},
                np.array([1.0, np.nan]), Box([np.array([np.inf])]), {float("nan")}):
        assert not _num.all_finite(bad)


# --- track geometry (GPS breadcrumbs, cone maps) -------------------------------------------------------------- #
from suspension import laptime  # noqa: E402

XS, YS = [0.0, 10.0, 20.0, 20.0, 10.0, 0.0], [0.0, 0.0, 5.0, 15.0, 20.0, 12.0]


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), 2.0e6])
def test_track_from_path_refuses_a_bad_coordinate(bad):
    for x, y, name in ((XS[:2] + [bad] + XS[3:], YS, "track x"), (XS, YS[:4] + [bad] + YS[5:], "track y")):
        with pytest.raises(ValueError, match=name):
            laptime.track_from_path(x, y)


def test_track_from_path_refuses_short_mismatched_or_bad_step():
    assert laptime.track_from_path([0, 1], [0, 1]).segments == []     # too short is an empty track, as before
    assert laptime.track_from_path(XS, YS[:5]).segments == []          # ragged: empty track, as before
    for ds in (0.0, -1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="track step ds"):
            laptime.track_from_path(XS, YS, ds=ds)
    assert len(laptime.track_from_path(XS, YS).segments) > 3     # and a clean path still builds


def test_cones_and_gps_refuse_bad_values():
    lx, ly, rx, ry = [0, 5, 10], [2, 2, 2], [0, 5, 10], [-2, -2, -2]
    cx, cy = laptime.cones_to_centerline(lx, ly, rx, ry)
    assert np.allclose(cy, 0.0) and np.allclose(cx, [0, 5, 10])
    for i, name in enumerate(("left cone x", "left cone y", "right cone x", "right cone y")):
        rows = [list(lx), list(ly), list(rx), list(ry)]
        rows[i][1] = float("nan")
        with pytest.raises(ValueError, match=name):
            laptime.cones_to_centerline(*rows)
    with pytest.raises(ValueError, match="as many x as y"):
        laptime.cones_to_centerline(lx, ly[:2], rx, ry)
    with pytest.raises(ValueError, match="as many x as y"):
        laptime.cones_to_centerline(lx, ly, rx[:2], ry)
    x, y = laptime.latlon_to_xy([52.0, 52.001], [4.0, 4.002])
    assert np.isfinite(x).all() and np.isfinite(y).all()
    for lat, lon, why in (([52.0, float("nan")], [4.0, 4.0], "latitude"), ([52.0, 52.0], [4.0, float("inf")], "longitude"),
                          ([52.0], [4.0, 4.1], "same length, got 1 and 2"), ([91.0, 52.0], [4.0, 4.0], "within ±90"),
                          ([52.0, 52.0], [4.0, -181.0], "within ±90"), ([-90.0, 90.0], [180.0, -180.0], None)):
        if why is None:
            laptime.latlon_to_xy(lat, lon)                       # the limits themselves are valid
        else:
            with pytest.raises(ValueError, match=why):
                laptime.latlon_to_xy(lat, lon)


# --- speed traces, tire files, bench logs, meshes ------------------------------------------------------------- #
from suspension import chassis, correlation, ev_electrical_check, throttle_return_ingest, tirefit, tiremodel  # noqa: E402


def test_speed_trace_correlation_marks_a_trace_with_a_bad_sample_not_ok():
    d = np.linspace(0, 100, 21)
    good = correlation.correlate_speed_trace(d, 10 + 0.1 * d, d, 10 + 0.1 * d)
    assert good.trace.ok and good.trace.rmse == pytest.approx(0.0, abs=1e-9)
    for which in range(4):
        arrs = [d.copy(), 10 + 0.1 * d, d.copy(), 10 + 0.1 * d]
        arrs[which][5] = float("nan")
        rep = correlation.correlate_speed_trace(*arrs)
        assert not rep.trace.ok and "not a finite number" in rep.trace.note


def test_lap_check_from_a_speed_trace_refuses_bad_samples():
    p = ev_electrical_check.ElecParams()
    t, v = list(range(10)), [30.0] * 10
    assert ev_electrical_check.check_lap_from_speed_csv(v, t, p) is not None
    cases = [(v[:5] + [float("nan")] + v[6:], t, "speed trace speed"), (v, t[:5] + [float("inf")] + t[6:], "speed trace time"),
             ([], [], "needs at least 1"), (v, t[:9], "one speed per time, got 10 and 9"),
             (v, [0, 1, 2, 2, 4, 5, 6, 7, 8, 9], "time must rise"), (v, list(range(9, -1, -1)), "time must rise"),
             (v[:5] + [1001.0] + v[6:], t, "above 1000 mph")]
    for vv, tt, why in cases:
        with pytest.raises(ValueError, match=why):
            ev_electrical_check.check_lap_from_speed_csv(vv, tt, p)
    for kw, why in (({"vehicle_mass_kg": 0.0}, "vehicle mass"), ({"vehicle_mass_kg": float("nan")}, "vehicle mass"),
                    ({"drag_cda": float("inf")}, "drag CdA"), ({"crr": float("nan")}, "rolling resistance"),
                    ({"drivetrain_eff": float("nan")}, "drivetrain efficiency")):
        with pytest.raises(ValueError, match=why):
            ev_electrical_check.check_lap_from_speed_csv(v, t, p, **kw)


def test_tir_file_with_an_overflowing_value_is_refused():
    tir = "[MODEL]\nLONGVL = 16.9\nFNOMIN = 6000\nPKY1 = -20\nPKY4 = 2\n"
    assert ev_electrical_check.TireLosses.from_tir(tir).fz_nom_n == 6000.0
    with pytest.raises(ValueError, match=r"\.tir PKY4"):
        ev_electrical_check.TireLosses.from_tir(tir.replace("PKY4 = 2", "PKY4 = 1e999"))
    for n in (0, -1, 2.5, float("nan"), True):
        with pytest.raises(ValueError, match="whole number of tires"):
            ev_electrical_check.TireLosses.from_tir(tir, n_tires=n)


def test_tire_model_refuses_coefficients_that_are_not_finite():
    c = dict(tiremodel.default_tire().coeffs)
    assert tiremodel.PacejkaLateral(coeffs=c, FNOMIN=1100.0).mu_peak(1100.0) > 0
    for bad in (float("nan"), float("inf")):
        with pytest.raises(ValueError, match=r"not finite: \['PDY1'\]"):
            tiremodel.PacejkaLateral(coeffs={**c, "PDY1": bad}, FNOMIN=1100.0)
    for f in (0.0, -5.0, float("nan"), float("inf"), "1100"):
        with pytest.raises(ValueError, match="FNOMIN"):
            tiremodel.PacejkaLateral(coeffs=c, FNOMIN=f)


def test_bench_log_that_cannot_give_a_rate_says_so():
    ok = throttle_return_ingest.spring_rate_from_bench_log(b"deflection_mm,force_N\n10,5\n20,10\n30,15\n40,20\n")
    assert ok.is_trustworthy and ok.k_N_per_m == pytest.approx(500.0)
    for log in (b"deflection_mm,force_N\n", b"deflection_mm,force_N\n1e-320,3\n", b"deflection_mm,force_N\n10,1e999\n",
                b"deflection_mm,force_N\nnan,nan\n", b"deflection_mm,force_N\n10," + b"9" * 200_000 + b"\n"):
        fit = throttle_return_ingest.spring_rate_from_bench_log(log)
        assert not fit.is_trustworthy and fit.k_N_per_m == 0.0 and fit.findings


def test_tire_data_channels_must_be_finite_and_matching():
    sa = np.linspace(-12, 12, 21)
    ch = {"SA": sa, "FZ": np.full(21, 1000.0), "FY": -1500 * np.tanh(sa / 4)}
    with pytest.raises(ValueError, match="tire data channel FY"):
        tirefit.fit_from_ttc_channels({**ch, "FY": np.where(sa == 0, np.nan, ch["FY"])})
    with pytest.raises(ValueError, match="same length"):
        tirefit.fit_from_ttc_channels({**ch, "FZ": ch["FZ"][:20]})
    with pytest.raises(ValueError, match="needs at least 8"):
        tirefit.fit_from_ttc_channels({k: v[:4] for k, v in ch.items()})


def test_damaged_mesh_file_is_a_value_error(tmp_path):
    p = tmp_path / "part.glb"
    p.write_bytes(b"glTF" + b"\x02\x00\x00\x00" + b"\xff" * 40)
    with pytest.raises(ValueError):
        chassis.load_part_mesh(str(p))
    stl = tmp_path / "tri.stl"
    stl.write_text("solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 10 0 0\nvertex 0 10 0\nendloop\n"
                   "endfacet\nendsolid t\n")
    assert chassis.load_part_mesh(str(stl))["triangles"] == 1
    for scale in (0.0, -1.0, float("nan"), float("inf"), 1e308):
        with pytest.raises(ValueError):
            chassis.load_part_mesh(str(stl), unit_scale=scale)


def test_json_finite_reads_text_bytes_and_files_and_refuses_nan_tokens():
    assert _num.json_finite('{"a": [1, 2.5, "NaN"]}') == {"a": [1, 2.5, "NaN"]}      # the word in a string is fine
    assert _num.json_finite(b'\xef\xbb\xbf{"a": 1}') == {"a": 1}                     # bytes, with a byte-order mark
    assert _num.json_finite(io.StringIO("[1, 2]")) == [1, 2]
    assert _num.json_finite(io.BytesIO(b"[3]")) == [3]
    for token in ("NaN", "Infinity", "-Infinity"):
        with pytest.raises(ValueError, match=f"the file holds {token}, which is not a finite number"):
            _num.json_finite(f'{{"hardpoints": {{"x": {token}}}}}')
    with pytest.raises(ValueError):
        _num.json_finite("{not json")


def test_frame_from_dict_refuses_a_non_finite_node():
    good = {"nodes": [{"nid": "a", "xyz_mm": [0, 0, 0]}, {"nid": "b", "xyz_mm": [1, 2, 3]}], "tubes": []}
    assert sorted(FrameGraph.from_dict(good).nodes) == ["a", "b"]
    bad = {"nodes": [{"nid": "a", "xyz_mm": [0, float("nan"), 0]}], "tubes": []}
    with pytest.raises(ValueError, match="node 'a': its position must be three finite numbers"):
        FrameGraph.from_dict(bad)
