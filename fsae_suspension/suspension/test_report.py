# ============================================================================
#  KinematiK — tests for suspension/report.py and suspension/drive_export.py
#  Verifies the stamped PDF (sign-off fields, provenance grades, integrity hash)
#  and the Drive export logic (capability probe, idempotent folders, honest
#  failure modes) — the latter against a mock Drive service, since a live Google
#  upload can't run in CI.
# ============================================================================
import os
import io
import json
import tempfile
import pytest

from pypdf import PdfReader

from suspension.report import (
    CalculationRecord, OutputRow, build_report, suggested_filename,
)
from suspension import drive_export as dx


# ============================ REPORT ============================ #
def _sample_record(signed=True):
    return CalculationRecord(
        title="Front upright hardpoint sign-off",
        author="Frederik Thio", team="Suspension", part="Front upright",
        signed_off=signed, tool="Inverse Genesis",
        inputs={"Wheel size": "13 in", "Static camber": "-1.5 deg"},
        outputs=[
            OutputRow("Camber gain", "0.98 deg/25mm", grade="modelled",
                      source="closed-form sweep"),
            OutputRow("Roll centre height", "42 mm", grade="estimate",
                      calibrated=False, source="single-corner proxy"),
            OutputRow("Scrub radius", "12 mm", grade="measured",
                      source="CMM of built part"),
        ],
        notes="Signed off for manufacture.", app_version="2026.7")


def test_report_builds_valid_pdf():
    rec = _sample_record()
    with tempfile.TemporaryDirectory() as d:
        path = build_report(rec, os.path.join(d, "r.pdf"))
        assert os.path.exists(path) and os.path.getsize(path) > 1000
        reader = PdfReader(path)
        assert len(reader.pages) >= 1


def test_report_contains_signoff_and_provenance():
    rec = _sample_record()
    with tempfile.TemporaryDirectory() as d:
        path = build_report(rec, os.path.join(d, "r.pdf"))
        txt = PdfReader(path).pages[0].extract_text()
    assert "SIGNED OFF" in txt
    assert "Frederik Thio" in txt
    assert "Camber gain" in txt
    # provenance grades are printed as words
    for grade in ("modelled", "measured", "estimate"):
        assert grade in txt


def test_unsigned_report_shows_draft():
    rec = _sample_record(signed=False)
    with tempfile.TemporaryDirectory() as d:
        path = build_report(rec, os.path.join(d, "r.pdf"))
        txt = PdfReader(path).pages[0].extract_text()
    assert "DRAFT" in txt
    assert "SIGNED OFF" not in txt.replace("NOT SIGNED OFF", "")


def test_integrity_hash_is_stable_and_content_bound():
    rec = _sample_record()
    h1 = rec.content_hash()
    # same content -> same hash
    assert rec.content_hash() == h1
    # changing an output value changes the hash
    rec2 = _sample_record()
    rec2.outputs[0].value = "1.20 deg/25mm"
    assert rec2.content_hash() != h1


def test_hash_appears_in_pdf():
    rec = _sample_record()
    with tempfile.TemporaryDirectory() as d:
        path = build_report(rec, os.path.join(d, "r.pdf"))
        txt = PdfReader(path).pages[0].extract_text()
    assert rec.content_hash()[:16] in txt


def test_ungraded_output_defaults_to_estimate_not_promoted():
    o = OutputRow("x", "1", grade="nonsense-grade")
    assert o.grade_k() == "estimate"     # bad grade under-claims, never promotes


def test_uncalibrated_tag_is_demoted():
    o = OutputRow("x", "1", grade="modelled", calibrated=False)
    assert "uncalibrated" in o.tag_text()


def test_empty_calc_still_builds():
    rec = CalculationRecord(title="Thin", author="A")
    with tempfile.TemporaryDirectory() as d:
        path = build_report(rec, os.path.join(d, "r.pdf"))
        assert os.path.exists(path)


def test_suggested_filename_is_clean():
    rec = _sample_record()
    fn = suggested_filename(rec)
    assert fn.endswith(".pdf")
    assert " " not in fn and "/" not in fn


def test_from_decision_bridges_project_signoff():
    class FakeDecision:
        title = "Rear ARB rate"
        author = "Lead A"
        team = "suspension"
        part = "rear ARB"
        date = "2026-07-01"
        rationale = "chosen for balance"
    rec = CalculationRecord.from_decision(FakeDecision(), tool="Balance")
    assert rec.signed_off is True
    assert rec.author == "Lead A"
    assert rec.notes == "chosen for balance"


# ============================ DRIVE EXPORT ============================ #
def test_available_reports_missing_creds_cleanly(monkeypatch):
    # no creds in env
    monkeypatch.delenv("GOOGLE_SERVICE_ACCOUNT_JSON", raising=False)
    ok, reason = dx.available(read_credential=lambda n: None)
    assert ok is False
    assert reason                                  # actionable, non-empty


def test_service_account_info_parses_env(monkeypatch):
    fake = {"type": "service_account", "client_email": "x@y.iam"}
    monkeypatch.setenv("GOOGLE_SERVICE_ACCOUNT_JSON", json.dumps(fake))
    got = dx.service_account_info()
    assert got["client_email"] == "x@y.iam"


def test_export_missing_local_file_fails_cleanly():
    r = dx.export_report("/no/such/file.pdf", "x.pdf")
    assert r.ok is False and "not found" in r.reason.lower()


def test_export_without_creds_returns_actionable_reason(monkeypatch, tmp_path):
    monkeypatch.delenv("GOOGLE_SERVICE_ACCOUNT_JSON", raising=False)
    pdf = tmp_path / "r.pdf"
    pdf.write_bytes(b"%PDF-1.4\n%%EOF\n")
    r = dx.export_report(str(pdf), "r.pdf", team="Suspension",
                         read_credential=lambda n: None)
    assert r.ok is False
    # no fake success; reason names what's missing
    assert "credential" in r.reason.lower() or "oauth" in r.reason.lower()
    assert r.file_id == ""


# ---- folder logic + upload against a MOCK Drive service ----------------
class _MockFiles:
    """Mimics service.files() for list/create, tracking created folders/files."""
    def __init__(self, store):
        self.store = store
        self._pending = None

    def list(self, **kw):
        # return folders matching the name in the query
        q = kw.get("q", "")
        name = None
        if "name='" in q:
            name = q.split("name='")[1].split("'")[0]
        matches = [f for f in self.store["folders"]
                   if f["name"] == name]
        self._pending = {"files": [{"id": m["id"], "name": m["name"]}
                                   for m in matches]}
        return self

    def create(self, **kw):
        body = kw.get("body", {})
        if body.get("mimeType") == dx._FOLDER_MIME:
            fid = f"folder-{len(self.store['folders'])+1}"
            self.store["folders"].append({"id": fid, "name": body["name"],
                                          "parents": body.get("parents")})
            self._pending = {"id": fid}
        else:
            fid = f"file-{len(self.store['files'])+1}"
            self.store["files"].append({"id": fid, "name": body["name"],
                                        "parents": body.get("parents")})
            self._pending = {"id": fid,
                             "webViewLink": f"https://drive.google.com/{fid}"}
        return self

    def execute(self):
        return self._pending


class _MockService:
    def __init__(self):
        self.store = {"folders": [], "files": []}
    def files(self):
        return _MockFiles(self.store)


def test_ensure_folder_path_is_idempotent():
    svc = _MockService()
    id1 = dx.ensure_folder_path(svc, ["KinematiK Reports", "Suspension", "2026"])
    # second call must REUSE, not duplicate
    id2 = dx.ensure_folder_path(svc, ["KinematiK Reports", "Suspension", "2026"])
    assert id1 == id2
    # exactly 3 folders created, not 6
    assert len(svc.store["folders"]) == 3


def test_upload_via_mock_service_returns_id_and_link(tmp_path, monkeypatch):
    pdf = tmp_path / "r.pdf"
    pdf.write_bytes(b"%PDF-1.4\n%%EOF\n")
    svc = _MockService()

    # patch the service builder + MediaFileUpload so no real Google call happens
    monkeypatch.setattr(dx, "_service_from_service_account", lambda info: svc)
    monkeypatch.setattr(dx, "libraries_present", lambda: True)
    monkeypatch.setattr(dx, "service_account_info",
                        lambda read_credential=None: {"type": "service_account"})
    import googleapiclient.http as gh
    monkeypatch.setattr(gh, "MediaFileUpload",
                        lambda *a, **k: object())

    r = dx.export_report(str(pdf), "front-upright.pdf", team="Suspension",
                         read_credential=lambda n: "{}")
    assert r.ok is True
    assert r.file_id.startswith("file-")
    assert r.web_link.startswith("https://drive.google.com/")
    assert r.folder_path == "KinematiK Reports/Suspension/2026"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
