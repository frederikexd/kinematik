# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/report_store.py — persist stamped calculation reports as the
#  team's system of record, then optionally EXPORT them to Google Drive on
#  demand. Supabase is the always-on home (zero setup for members); Drive is a
#  destination you push to when a team wants an externally-browsable archive.
# ============================================================================
"""
Report storage + on-demand Drive export.

THE ARCHITECTURE THIS SETTLES
-----------------------------
After a long design conversation, the decision: SUPABASE IS THE SYSTEM OF
RECORD, DRIVE IS AN OPTIONAL EXPORT. Reports save into the team's existing
Supabase-backed project with no Google setup, no OAuth, no console account —
they just persist where the team already is. A team that wants an
externally-browsable archive can later push any/all reports to their Google
Drive with the drive_export flow, but nothing depends on that.

THE STORAGE SPLIT (this matters for cost, not just tidiness)
------------------------------------------------------------
The project persists as ONE JSON blob per workspace that the backend caches and
only re-transfers when it changes — that egress discipline is deliberate and
load-bearing (see SupabaseBackend.read). So a PDF must NOT go in that blob:
base64-ing a multi-page PDF into the cached ~MB document would re-inflate the
exact transfer the backend works to avoid. Therefore:

  * METADATA (small: title, author, provenance summary, content hash, storage
    pointer) lives in the project blob, as a list of ReportRecord dataclasses —
    the same pattern decisions/notes already use, so it serialises and isolates
    per workspace exactly like the rest of the project.
  * PDF BYTES live outside the blob: in a Supabase STORAGE BUCKET when available,
    or a local file when running on the JSON-file backend (laptop/tests). The
    metadata carries a pointer (bucket path or local path), never the bytes.

TENANT ISOLATION (the SaaS-relevant part)
-----------------------------------------
Report metadata rides inside the project blob, which is already keyed by
project_id / workspace and already isolated per tenant by the backend. A
report's storage path is namespaced by the same project_id, so tenant A's PDF
path can never collide with or resolve into tenant B's. No new isolation model
is introduced — reports inherit the project's, which is the safe choice.

HONESTY
-------
Every function degrades to a clear reason rather than a fake success: if the
Storage bucket isn't reachable it falls back to local and says so; if Drive
isn't connected the report still saved to Supabase. Nothing claims a durable
save it didn't perform. No supabase/google imports at module load — all lazy.
"""

from __future__ import annotations

import os
import base64
import hashlib
import datetime as _dt
from dataclasses import dataclass, field, asdict
from typing import Optional


DEFAULT_BUCKET = "kinematik-reports"


# ===================================================================== #
#  1.  METADATA RECORD  (small; lives in the project blob)
# ===================================================================== #
@dataclass
class ReportRecord:
    """Pointer + summary for one stored report. Small enough to live in the
    project JSON blob without bloating it — the PDF bytes are elsewhere."""
    id: str
    title: str
    author: str
    team: str = ""
    part: str = ""
    date: str = ""
    content_hash: str = ""          # from CalculationRecord.content_hash()
    provenance_summary: str = ""    # e.g. "3 modelled · 1 measured · 1 estimate"
    signed_off: bool = False
    # where the bytes live: ("bucket", "<project>/<file>.pdf") or ("local", path)
    storage_kind: str = "local"
    storage_path: str = ""
    size_bytes: int = 0
    # drive export state (empty until pushed)
    drive_file_id: str = ""
    drive_web_link: str = ""
    drive_exported_at: str = ""

    def __post_init__(self):
        if not self.date:
            self.date = _dt.datetime.now().isoformat(timespec="seconds")
        if not self.id:
            self.id = _dt.datetime.now().strftime("%Y%m%d%H%M%S%f")

    @property
    def exported_to_drive(self) -> bool:
        return bool(self.drive_file_id)


def provenance_summary_from_outputs(outputs) -> str:
    """Roll a report's output grades into a one-line summary for the list view."""
    from .provenance import grade_key
    counts: dict[str, int] = {}
    for o in outputs:
        g = grade_key(getattr(o, "grade", "estimate"))
        counts[g] = counts.get(g, 0) + 1
    order = ["verified", "measured", "modelled", "estimate", "guess"]
    parts = [f"{counts[g]} {g}" for g in order if g in counts]
    return " · ".join(parts)


# ===================================================================== #
#  2.  THE STORE  (metadata in project, bytes in bucket/local)
# ===================================================================== #
class ReportStore:
    """Save/list/fetch reports, and export them to Drive on demand.

    Construct with the app's ProjectStore (for metadata persistence + tenant
    key) and optionally the Supabase client (for the Storage bucket). With no
    Supabase client it uses local files — fine for laptops/tests.
    """

    def __init__(self, project_store, supabase_client=None,
                 bucket: str = DEFAULT_BUCKET, local_dir: Optional[str] = None):
        self.project = project_store
        self.client = supabase_client
        self.bucket = bucket
        # tenant namespace: the project's own id, so paths can't cross tenants
        self.tenant = getattr(getattr(project_store, "backend", None),
                              "project_id", None) or getattr(
                              project_store, "path", "local")
        self.local_dir = local_dir or os.path.join(
            os.path.dirname(getattr(project_store, "path", ".") or "."),
            "kinematik_reports")
        # the metadata list lives on the project so it persists in the blob
        if not hasattr(self.project, "reports") or self.project.reports is None:
            self.project.reports = []

    # ---- persistence of the bytes --------------------------------------
    def _tenant_slug(self) -> str:
        s = str(self.tenant)
        return "".join(c if c.isalnum() else "-" for c in s).strip("-")[:60] or "tenant"

    def _store_bytes(self, pdf_bytes: bytes, filename: str) -> tuple[str, str, str]:
        """Persist the PDF bytes. Returns (storage_kind, storage_path, reason).

        Prefers the Supabase Storage bucket (namespaced by tenant); falls back to
        a local file if the bucket isn't reachable, and says which happened.
        """
        object_path = f"{self._tenant_slug()}/{filename}"
        if self.client is not None:
            try:
                storage = self.client.storage.from_(self.bucket)
                # upsert so re-saving the same report replaces cleanly
                storage.upload(
                    path=object_path, file=pdf_bytes,
                    file_options={"content-type": "application/pdf",
                                  "upsert": "true"})
                return "bucket", object_path, ""
            except Exception as e:
                # fall through to local, but report why the bucket failed
                reason = (f"Supabase Storage unavailable ({e}); saved locally "
                          f"instead. Create a '{self.bucket}' bucket to persist "
                          f"reports durably.")
                kind, path = self._store_local(pdf_bytes, filename)
                return kind, path, reason
        kind, path = self._store_local(pdf_bytes, filename)
        return kind, path, ""

    def _store_local(self, pdf_bytes: bytes, filename: str) -> tuple[str, str]:
        d = os.path.join(self.local_dir, self._tenant_slug())
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, filename)
        with open(path, "wb") as f:
            f.write(pdf_bytes)
        return "local", path

    def fetch_bytes(self, rec: ReportRecord) -> Optional[bytes]:
        """Read a stored report's PDF bytes back, from wherever they live."""
        if rec.storage_kind == "bucket" and self.client is not None:
            try:
                return self.client.storage.from_(self.bucket).download(
                    rec.storage_path)
            except Exception:
                return None
        if rec.storage_kind == "local" and os.path.exists(rec.storage_path):
            with open(rec.storage_path, "rb") as f:
                return f.read()
        return None

    # ---- the main save -------------------------------------------------
    def save_report(self, pdf_path: str, calc_record, filename: str) -> ReportRecord:
        """Store a built PDF as the team's record. Returns the ReportRecord.

        calc_record is a report.CalculationRecord (for hash + provenance summary).
        Persists metadata into the project (call project.save() after) and the
        bytes into the bucket/local store.
        """
        with open(pdf_path, "rb") as f:
            pdf_bytes = f.read()

        kind, path, reason = self._store_bytes(pdf_bytes, filename)
        rec = ReportRecord(
            id="",
            title=getattr(calc_record, "title", "Report"),
            author=getattr(calc_record, "author", ""),
            team=getattr(calc_record, "team", ""),
            part=getattr(calc_record, "part", ""),
            content_hash=(calc_record.content_hash()
                          if hasattr(calc_record, "content_hash") else ""),
            provenance_summary=provenance_summary_from_outputs(
                getattr(calc_record, "outputs", [])),
            signed_off=getattr(calc_record, "signed_off", False),
            storage_kind=kind, storage_path=path, size_bytes=len(pdf_bytes))
        self.project.reports.append(rec)
        self._store_reason = reason      # UI can surface a fallback notice
        return rec

    def list_reports(self) -> list:
        """All stored reports for this tenant, newest first."""
        recs = [r if isinstance(r, ReportRecord) else ReportRecord(**r)
                for r in getattr(self.project, "reports", [])]
        return sorted(recs, key=lambda r: r.date, reverse=True)

    # ---- on-demand Drive export ----------------------------------------
    def export_to_drive(self, rec: ReportRecord, *, read_credential=None,
                        oauth_token=None) -> "DriveExportOutcome":
        """Push ONE stored report to Google Drive. Reads the bytes from the store,
        writes a temp file, and hands it to drive_export. Records the resulting
        Drive id/link back onto the ReportRecord. Never raises."""
        from . import drive_export as dx
        import tempfile

        pdf_bytes = self.fetch_bytes(rec)
        if pdf_bytes is None:
            return DriveExportOutcome(False, "Could not read the stored PDF back "
                                      "to export it.")
        tmp = tempfile.NamedTemporaryFile(suffix=".pdf", delete=False)
        try:
            tmp.write(pdf_bytes)
            tmp.close()
            res = dx.export_report(
                tmp.name, f"{rec.id}.pdf", team=rec.team or "General",
                read_credential=read_credential, oauth_token=oauth_token)
        finally:
            try:
                os.unlink(tmp.name)
            except OSError:
                pass

        if res.ok:
            rec.drive_file_id = res.file_id
            rec.drive_web_link = res.web_link
            rec.drive_exported_at = _dt.datetime.now().isoformat(timespec="seconds")
            return DriveExportOutcome(True, res.reason, res.web_link,
                                      res.folder_path)
        return DriveExportOutcome(False, res.reason)

    def export_all_to_drive(self, *, read_credential=None, oauth_token=None,
                            only_unexported: bool = True) -> dict:
        """Push many reports at once. Returns {report_id: DriveExportOutcome}."""
        out = {}
        for rec in self.list_reports():
            if only_unexported and rec.exported_to_drive:
                continue
            out[rec.id] = self.export_to_drive(
                rec, read_credential=read_credential, oauth_token=oauth_token)
        return out


@dataclass
class DriveExportOutcome:
    ok: bool
    reason: str = ""
    web_link: str = ""
    folder_path: str = ""


# ===================================================================== #
#  3.  PROJECT SERIALISATION HOOKS
# ===================================================================== #
# The ProjectStore serialises known collections in _payload/_apply. Reports use
# the same dataclass pattern, so these helpers let the app wire them in without
# editing project.py's core: call add_reports_to_payload(store, payload) inside
# _payload and load_reports_from_dict(store, d) inside _apply — or just persist
# store.reports alongside decisions/notes the same way.
def serialize_reports(reports) -> list:
    return [asdict(r) if isinstance(r, ReportRecord) else dict(r)
            for r in (reports or [])]


def deserialize_reports(raw) -> list:
    out = []
    for r in raw or []:
        try:
            out.append(ReportRecord(**r) if isinstance(r, dict) else r)
        except Exception:
            # tolerate rows persisted by an older schema — skip rather than crash
            continue
    return out
