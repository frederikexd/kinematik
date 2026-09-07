# ============================================================================
#  KinematiK — tests for suspension/report_store.py
#  Verifies: PDF bytes stored OUTSIDE the project blob (metadata only inside),
#  tenant-namespaced paths (isolation), local fallback when no Supabase, byte
#  round-trip, provenance summary, and on-demand Drive export against a mock.
# ============================================================================
import os
import tempfile
import pytest

from suspension.report_store import (
    ReportStore, ReportRecord, serialize_reports, deserialize_reports,
)
from suspension.report import CalculationRecord, OutputRow, build_report, suggested_filename


# ---- a minimal fake ProjectStore matching the real one's surface --------
class _FakeBackend:
    def __init__(self, project_id):
        self.project_id = project_id


class _FakeProject:
    def __init__(self, project_id="elbee", path="/tmp/fake_project.json"):
        self.backend = _FakeBackend(project_id)
        self.path = path
        self.reports = []


def _calc(signed=True):
    return CalculationRecord(
        title="Front upright sign-off", author="Frederik Thio",
        team="Suspension", part="Front upright", signed_off=signed,
        inputs={"Wheel": "13 in"},
        outputs=[
            OutputRow("Camber gain", "0.98", grade="modelled"),
            OutputRow("Scrub", "12mm", grade="measured"),
            OutputRow("RC height", "42mm", grade="estimate", calibrated=False),
        ])


def _build_pdf(calc, d):
    return build_report(calc, os.path.join(d, suggested_filename(calc)))


# ---- metadata vs bytes split -------------------------------------------
def test_bytes_stored_outside_project_blob():
    with tempfile.TemporaryDirectory() as d:
        proj = _FakeProject()
        store = ReportStore(proj, supabase_client=None, local_dir=d)
        calc = _calc()
        pdf = _build_pdf(calc, d)
        rec = store.save_report(pdf, calc, "r.pdf")
        # the project only holds the small metadata record...
        assert len(proj.reports) == 1
        meta = proj.reports[0]
        assert isinstance(meta, ReportRecord)
        # ...and that record contains NO pdf bytes, only a pointer + summary
        as_dict = meta.__dict__
        for v in as_dict.values():
            assert not (isinstance(v, (bytes, bytearray))), "no bytes in metadata"
        assert meta.storage_path and meta.size_bytes > 0
        assert meta.content_hash                       # hash carried for integrity


def test_provenance_summary_in_metadata():
    with tempfile.TemporaryDirectory() as d:
        proj = _FakeProject()
        store = ReportStore(proj, local_dir=d)
        calc = _calc()
        rec = store.save_report(_build_pdf(calc, d), calc, "r.pdf")
        # 1 modelled, 1 measured, 1 estimate — order normalised
        assert "measured" in rec.provenance_summary
        assert "modelled" in rec.provenance_summary
        assert "estimate" in rec.provenance_summary


# ---- tenant isolation ---------------------------------------------------
def test_storage_path_is_tenant_namespaced():
    with tempfile.TemporaryDirectory() as d:
        a = ReportStore(_FakeProject(project_id="team-alpha"), local_dir=d)
        b = ReportStore(_FakeProject(project_id="team-beta"), local_dir=d)
        calc = _calc()
        ra = a.save_report(_build_pdf(calc, d), calc, "r.pdf")
        rb = b.save_report(_build_pdf(calc, d), calc, "r.pdf")
        # same filename, DIFFERENT tenant -> different stored paths, no collision
        assert ra.storage_path != rb.storage_path
        assert "team-alpha" in ra.storage_path
        assert "team-beta" in rb.storage_path


# ---- byte round-trip ----------------------------------------------------
def test_fetch_bytes_round_trip_local():
    with tempfile.TemporaryDirectory() as d:
        proj = _FakeProject()
        store = ReportStore(proj, local_dir=d)
        calc = _calc()
        pdf = _build_pdf(calc, d)
        with open(pdf, "rb") as f:
            original = f.read()
        rec = store.save_report(pdf, calc, "r.pdf")
        got = store.fetch_bytes(rec)
        assert got == original                          # exact bytes preserved
        assert got[:4] == b"%PDF"


# ---- list ordering ------------------------------------------------------
def test_list_reports_newest_first():
    with tempfile.TemporaryDirectory() as d:
        proj = _FakeProject()
        store = ReportStore(proj, local_dir=d)
        c1 = _calc(); c1.date = "2026-01-01T00:00:00"
        c2 = _calc(); c2.date = "2026-06-01T00:00:00"
        store.save_report(_build_pdf(c1, d), c1, "a.pdf")
        store.save_report(_build_pdf(c2, d), c2, "b.pdf")
        # manually stamp record dates to test ordering
        store.project.reports[0].date = "2026-01-01T00:00:00"
        store.project.reports[1].date = "2026-06-01T00:00:00"
        lst = store.list_reports()
        assert lst[0].date >= lst[1].date


# ---- supabase bucket path (mock client) --------------------------------
class _MockStorageBucket:
    def __init__(self, store): self.store = store
    def upload(self, path, file, file_options=None):
        self.store[path] = file
        return {"path": path}
    def download(self, path):
        return self.store[path]

class _MockStorage:
    def __init__(self): self.data = {}
    def from_(self, bucket): return _MockStorageBucket(self.data)

class _MockSupabase:
    def __init__(self): self.storage = _MockStorage()


def test_bucket_storage_used_when_client_present():
    with tempfile.TemporaryDirectory() as d:
        proj = _FakeProject(project_id="team-x")
        client = _MockSupabase()
        store = ReportStore(proj, supabase_client=client, local_dir=d)
        calc = _calc()
        rec = store.save_report(_build_pdf(calc, d), calc, "r.pdf")
        assert rec.storage_kind == "bucket"
        assert "team-x" in rec.storage_path
        # round trip through the mock bucket
        assert store.fetch_bytes(rec)[:4] == b"%PDF"


def test_bucket_failure_falls_back_to_local_with_reason():
    class _BadStorage:
        def from_(self, b):
            raise RuntimeError("bucket missing")
    class _BadClient:
        storage = _BadStorage()
    with tempfile.TemporaryDirectory() as d:
        store = ReportStore(_FakeProject(), supabase_client=_BadClient(), local_dir=d)
        calc = _calc()
        rec = store.save_report(_build_pdf(calc, d), calc, "r.pdf")
        assert rec.storage_kind == "local"              # fell back
        assert "unavailable" in store._store_reason.lower()  # said why


# ---- on-demand Drive export (mock drive_export) ------------------------
def test_export_to_drive_records_link(monkeypatch):
    from suspension import drive_export as dx
    with tempfile.TemporaryDirectory() as d:
        store = ReportStore(_FakeProject(), local_dir=d)
        calc = _calc()
        rec = store.save_report(_build_pdf(calc, d), calc, "r.pdf")

        def fake_export(path, name, **kw):
            return dx.DriveResult(True, file_id="F1",
                                  web_link="https://drive.google.com/F1",
                                  folder_path="KinematiK Reports/Suspension/2026",
                                  reason="ok")
        monkeypatch.setattr(dx, "export_report", fake_export)

        outcome = store.export_to_drive(rec)
        assert outcome.ok
        assert rec.drive_file_id == "F1"                # recorded back on the record
        assert rec.exported_to_drive
        assert rec.drive_web_link.endswith("F1")


def test_export_all_skips_already_exported(monkeypatch):
    from suspension import drive_export as dx
    with tempfile.TemporaryDirectory() as d:
        store = ReportStore(_FakeProject(), local_dir=d)
        calc = _calc()
        r1 = store.save_report(_build_pdf(calc, d), calc, "r1.pdf")
        r2 = store.save_report(_build_pdf(calc, d), calc, "r2.pdf")
        r1.drive_file_id = "already"                    # pretend r1 done

        calls = []
        def fake_export(path, name, **kw):
            calls.append(name)
            return dx.DriveResult(True, file_id="NEW",
                                  web_link="x", folder_path="p", reason="ok")
        monkeypatch.setattr(dx, "export_report", fake_export)

        store.export_all_to_drive(only_unexported=True)
        assert len(calls) == 1                          # only the un-exported one


def test_export_missing_bytes_fails_cleanly():
    with tempfile.TemporaryDirectory() as d:
        store = ReportStore(_FakeProject(), local_dir=d)
        rec = ReportRecord(id="x", title="t", author="a",
                           storage_kind="local", storage_path="/no/such.pdf")
        outcome = store.export_to_drive(rec)
        assert outcome.ok is False and "read" in outcome.reason.lower()


# ---- serialisation hooks (project blob persistence) --------------------
def test_serialize_deserialize_round_trip():
    rec = ReportRecord(id="1", title="t", author="a", storage_path="p")
    raw = serialize_reports([rec])
    assert isinstance(raw[0], dict) and "storage_path" in raw[0]
    back = deserialize_reports(raw)
    assert back[0].id == "1" and back[0].storage_path == "p"


def test_deserialize_tolerates_bad_rows():
    # an older/garbled row must not crash the whole load
    back = deserialize_reports([{"id": "ok", "title": "t", "author": "a"},
                                {"unexpected": "shape"}])
    assert len(back) == 1 and back[0].id == "ok"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
