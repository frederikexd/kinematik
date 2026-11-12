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
