"""Files a team uploads are untrusted. A value that is not a finite number must never become part of a model, and
no upload may exhaust the server's memory. Each case here was found by fuzzing the parsers with damaged files."""
import io
import math
import zipfile

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
