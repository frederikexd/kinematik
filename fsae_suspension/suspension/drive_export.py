# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/drive_export.py — export stamped calculation PDFs into an
#  organised Google Drive folder. Honest about auth: a Drive upload REQUIRES
#  credentials, so this module degrades cleanly (and says exactly what's missing)
#  rather than pretending an unauthenticated upload succeeded.
# ============================================================================
"""
Google Drive export for KinematiK reports — the version that actually works.

THE CONSTRAINT, STATED PLAINLY
------------------------------
There is no way to push a file into someone's Google Drive without Google
credentials. Any "export straight to Drive" feature is really "authenticate to
Drive, then upload". This module makes that explicit and supports the two paths
that genuinely work in a Streamlit deployment, and it NEVER reports success for
an upload it didn't actually perform:

  1. SERVICE ACCOUNT (recommended for a team).
     A Google Cloud service account's JSON key is stored in st.secrets. Reports
     land in a Shared Drive (or a folder the service account can write to),
     organised into per-team / per-date subfolders. Every team member's reports
     go to ONE canonical team Drive — which is usually what "organised Drive
     folder" means for an FSAE team. The service account is not a person, so to
     write into a normal My-Drive folder it must be granted access to that
     folder (or use a Shared Drive, which is the clean setup).

  2. USER OAUTH (per-member, into their own Drive).
     Each member authorises KinematiK once via OAuth; reports go to THEIR Drive.
     This needs an OAuth client + a redirect flow, which Streamlit can host but
     which the deploying team must configure. Provided as a token-based uploader
     so the UI can drive the consent flow and hand this module a credential.

WHAT THIS MODULE GUARANTEES
---------------------------
  * It builds the folder path (Team / Year / …) idempotently — reusing an
    existing folder instead of duplicating it.
  * It returns a structured result with the created file's Drive ID and link on
    success, or a clear, actionable reason on failure (missing creds, missing
    library, permission denied) — no silent failure, no fake success.
  * If the Google libraries or credentials are absent, ``available()`` returns
    False and the UI falls back to a plain download, so the PDF feature never
    depends on Drive being set up.

No import of google libraries at module load — they're imported lazily inside
the functions that need them, so importing this module never fails on a
deployment that hasn't installed them.
"""

from __future__ import annotations

import os
import io
import json
import datetime as _dt
from dataclasses import dataclass
from typing import Optional


DRIVE_SCOPES = ["https://www.googleapis.com/auth/drive.file"]
_FOLDER_MIME = "application/vnd.google-apps.folder"


@dataclass
class DriveResult:
    ok: bool
    reason: str = ""                 # why it failed (actionable), empty on success
    file_id: str = ""
    web_link: str = ""
    folder_id: str = ""
    folder_path: str = ""

    def as_dict(self) -> dict:
        return {
            "ok": self.ok, "reason": self.reason, "file_id": self.file_id,
            "web_link": self.web_link, "folder_id": self.folder_id,
            "folder_path": self.folder_path,
        }


# ===================================================================== #
#  Capability probe — never assume Drive is set up
# ===================================================================== #
def libraries_present() -> bool:
    """True if the google-api client libraries are importable."""
    try:
        import googleapiclient  # noqa: F401
        import google.oauth2    # noqa: F401
        return True
    except Exception:
        return False


def service_account_info(read_credential=None) -> Optional[dict]:
    """Return the service-account JSON dict from secrets/env, or None.

    read_credential: optional callable(name)->str (e.g. the app's own
    _read_credential that checks st.secrets first). Falls back to env.
    """
    raw = None
    if read_credential is not None:
        try:
            raw = read_credential("GOOGLE_SERVICE_ACCOUNT_JSON")
        except Exception:
            raw = None
    if not raw:
        raw = os.environ.get("GOOGLE_SERVICE_ACCOUNT_JSON")
    if not raw:
        return None
    try:
        return json.loads(raw) if isinstance(raw, str) else dict(raw)
    except Exception:
        return None


def available(read_credential=None) -> tuple[bool, str]:
    """(can_export, reason_if_not). The UI calls this to decide whether to show
    'Export to Drive' or fall back to a plain download."""
    if not libraries_present():
        return False, ("Google API libraries not installed. Add "
                       "google-api-python-client, google-auth and "
                       "google-auth-oauthlib to requirements.txt.")
    if service_account_info(read_credential) is None:
        return False, ("No Drive credentials configured. Add a service-account "
                       "key as GOOGLE_SERVICE_ACCOUNT_JSON in secrets, or use the "
                       "per-user OAuth flow.")
    return True, ""


# ===================================================================== #
#  Drive service builders (lazy imports)
# ===================================================================== #
def _service_from_service_account(info: dict):
    from google.oauth2 import service_account
    from googleapiclient.discovery import build
    creds = service_account.Credentials.from_service_account_info(
        info, scopes=DRIVE_SCOPES)
    return build("drive", "v3", credentials=creds, cache_discovery=False)


def _service_from_oauth_token(token: dict):
    """Build a Drive service from a user's OAuth token dict (the UI runs the
    consent flow and passes the resulting token here)."""
    from google.oauth2.credentials import Credentials
    from googleapiclient.discovery import build
    creds = Credentials(
        token=token.get("token"),
        refresh_token=token.get("refresh_token"),
        token_uri=token.get("token_uri", "https://oauth2.googleapis.com/token"),
        client_id=token.get("client_id"),
        client_secret=token.get("client_secret"),
        scopes=token.get("scopes", DRIVE_SCOPES))
    return build("drive", "v3", credentials=creds, cache_discovery=False)


# ===================================================================== #
#  Folder handling — idempotent path creation
# ===================================================================== #
def _find_or_create_folder(service, name: str, parent_id: Optional[str],
                           shared_drive_id: Optional[str] = None) -> str:
    """Return the id of a folder `name` under `parent_id`, creating it if absent.
    Idempotent: reuses an existing folder rather than duplicating."""
    safe = name.replace("'", "\\'")
    q = (f"name='{safe}' and mimeType='{_FOLDER_MIME}' and trashed=false")
    if parent_id:
        q += f" and '{parent_id}' in parents"
    list_kw = dict(q=q, spaces="drive", fields="files(id,name)",
                   pageSize=10)
    if shared_drive_id:
        list_kw.update(corpora="drive", driveId=shared_drive_id,
                       includeItemsFromAllDrives=True, supportsAllDrives=True)
    resp = service.files().list(**list_kw).execute()
    files = resp.get("files", [])
    if files:
        return files[0]["id"]
    meta = {"name": name, "mimeType": _FOLDER_MIME}
    if parent_id:
        meta["parents"] = [parent_id]
    elif shared_drive_id:
        meta["parents"] = [shared_drive_id]
    created = service.files().create(
        body=meta, fields="id", supportsAllDrives=True).execute()
    return created["id"]


def ensure_folder_path(service, path_parts: list[str],
                       root_id: Optional[str] = None,
                       shared_drive_id: Optional[str] = None) -> str:
    """Ensure a nested folder path exists (e.g. ['KinematiK Reports',
    'Suspension', '2026']) and return the deepest folder's id."""
    parent = root_id or shared_drive_id
    for part in path_parts:
        parent = _find_or_create_folder(service, part, parent, shared_drive_id)
    return parent


# ===================================================================== #
#  The upload
# ===================================================================== #
def _upload_pdf(service, local_path: str, drive_name: str,
                folder_id: str) -> tuple[str, str]:
    """Upload a PDF and return (file_id, web_link)."""
    from googleapiclient.http import MediaFileUpload
    media = MediaFileUpload(local_path, mimetype="application/pdf",
                            resumable=False)
    meta = {"name": drive_name, "parents": [folder_id]}
    created = service.files().create(
        body=meta, media_body=media,
        fields="id,webViewLink", supportsAllDrives=True).execute()
    return created["id"], created.get("webViewLink", "")


def export_report(local_pdf_path: str,
                  drive_filename: str,
                  team: str = "",
                  *,
                  read_credential=None,
                  oauth_token: Optional[dict] = None,
                  root_folder_name: str = "KinematiK Reports",
                  shared_drive_id: Optional[str] = None,
                  root_folder_id: Optional[str] = None) -> DriveResult:
    """Export one stamped PDF into an organised Drive folder.

    Folder layout: <root_folder_name>/<Team>/<Year>/<file>. The folder path is
    created idempotently. Auth precedence: an explicit oauth_token (per-user
    Drive) if given, else a service account from secrets/env.

    Returns a DriveResult. On any failure it returns ok=False with an actionable
    reason and performs NO partial claim of success.
    """
    if not os.path.exists(local_pdf_path):
        return DriveResult(False, reason=f"Local PDF not found: {local_pdf_path}")

    if not libraries_present():
        return DriveResult(False, reason=(
            "Google API libraries not installed — cannot upload. The PDF is "
            "still available for direct download."))

    # build the service by whichever auth is available
    try:
        if oauth_token:
            service = _service_from_oauth_token(oauth_token)
            drive_kind = "user OAuth"
        else:
            info = service_account_info(read_credential)
            if info is None:
                return DriveResult(False, reason=(
                    "No Drive credentials configured. Add "
                    "GOOGLE_SERVICE_ACCOUNT_JSON to secrets (and share the target "
                    "folder / Shared Drive with the service-account email), or "
                    "pass a user OAuth token. The PDF is available for download."))
            service = _service_from_service_account(info)
            drive_kind = "service account"
    except Exception as e:
        return DriveResult(False, reason=f"Could not authenticate to Drive: {e}")

    # organised path: root / team / year
    year = _dt.date.today().year
    path_parts = [root_folder_name]
    if team:
        path_parts.append(team)
    path_parts.append(str(year))
    folder_path = "/".join(path_parts)

    try:
        folder_id = ensure_folder_path(
            service, path_parts, root_id=root_folder_id,
            shared_drive_id=shared_drive_id)
    except Exception as e:
        return DriveResult(False, folder_path=folder_path, reason=(
            f"Authenticated, but could not create/find the folder path "
            f"'{folder_path}': {e}. For a service account, ensure it can write "
            f"to a Shared Drive or a folder shared with its email."))

    try:
        file_id, link = _upload_pdf(service, local_pdf_path, drive_filename,
                                    folder_id)
    except Exception as e:
        return DriveResult(False, folder_id=folder_id, folder_path=folder_path,
                           reason=f"Folder ready but upload failed: {e}")

    return DriveResult(True, file_id=file_id, web_link=link,
                       folder_id=folder_id, folder_path=folder_path,
                       reason=f"Uploaded via {drive_kind}.")
