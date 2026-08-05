# ============================================================================
#  KinematiK — ui/report.py
#  Streamlit panel: generate a stamped calculation report PDF from the current
#  sign-off, offer a direct download, and — when Drive credentials are actually
#  configured — export it into an organised Google Drive folder. Honest by
#  design: if Drive isn't set up, the button explains what's missing instead of
#  failing silently, and the download always works regardless.
# ============================================================================
"""Render the Calculation Report tab.

Design intent (consistent with the rest of KinematiK): never claim a Drive
upload that didn't happen. The panel probes drive_export.available() and only
shows the export button when it can actually succeed; otherwise it shows the
plain download plus the exact setup step needed. Every generated PDF stamps the
provenance grade of each output, so the report can't imply more confidence than
the calculation had.
"""

from __future__ import annotations

import os
import tempfile

#: Bound by render(). Per ui/__init__.py, streamlit is NOT imported at
#: module scope — that keeps this file importable headless and cheap,
#: and it is what lets the reachability test verify the contract in a
#: fresh interpreter.
st = None

from suspension.report import (
    CalculationRecord, build_report, suggested_filename,
)
from suspension import drive_export as dx
from suspension import drive_oauth as ox


def _read_credential(name):
    """Mirror the app's secret lookup (st.secrets first, then env)."""
    try:
        if st is not None:
            v = st.secrets.get(name)
            if v:
                return v
    except Exception:
        pass
    return os.environ.get(name)


def render(record: CalculationRecord | None = None,
           default_team: str = "",
           default_author: str = ""):
    global st
    import streamlit as st          # noqa: PLW0603 - see note above
    ss = st.session_state

    st.subheader("📄 Calculation Report — stamped sign-off PDF")
    st.caption(
        "Turn a finished calculation into a timestamped PDF: who signed off, "
        "their inputs, the solved results, and a provenance grade on every "
        "output. Download it, or export straight into your team's Google Drive "
        "folder when Drive is configured."
    )

    # If no record was passed, let the user compose a minimal one so the tab is
    # usable standalone. In the app this is normally handed a real record from
    # the tool the member just finished.
    if record is None:
        record = ss.get("report_record")
    if record is None:
        st.info("No calculation handed to the report yet. Finish a calculation "
                "in any tool and choose ‘Create report’, or fill the fields "
                "below to stamp one manually.", icon="ℹ️")
        with st.expander("Compose a report manually", expanded=True):
            title = st.text_input("Title", "Calculation sign-off", key="rep_title")
            c1, c2 = st.columns(2)
            with c1:
                author = st.text_input("Author", default_author, key="rep_auth")
                team = st.text_input("Team", default_team, key="rep_team")
            with c2:
                part = st.text_input("Part / system", "", key="rep_part")
                tool = st.text_input("Tool", "", key="rep_tool")
            signed = st.checkbox("Signed off", value=True, key="rep_signed")
            notes = st.text_area("Rationale / notes", "", key="rep_notes")
            if st.button("Stamp this report", key="rep_make"):
                record = CalculationRecord(
                    title=title, author=author, team=team, part=part,
                    tool=tool, signed_off=signed, notes=notes,
                    inputs=ss.get("report_inputs", {}),
                    outputs=ss.get("report_outputs", []))
                ss["report_record"] = record
        if record is None:
            return

    # ---- preview the record --------------------------------------------
    st.markdown(f"**{record.title}** — {record.author or '—'} "
                f"· {record.team or '—'} · {record.part or '—'}")
    st.caption(("✅ signed off" if record.signed_off else "⚠️ draft — not signed off")
               + f" · {record.date}")
    if record.outputs:
        import pandas as pd
        rows = [{"Output": o.name, "Value": o.value, "Provenance": o.tag_text(),
                 "Source": o.source or "—"} for o in record.outputs]
        st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)

    # ---- build the PDF --------------------------------------------------
    if st.button("Generate PDF", type="primary", key="rep_gen"):
        out_dir = tempfile.mkdtemp(prefix="kinematik_rep_")
        fn = suggested_filename(record)
        path = build_report(record, os.path.join(out_dir, fn))
        ss["report_pdf_path"] = path
        ss["report_pdf_name"] = fn

    path = ss.get("report_pdf_path")
    if not path or not os.path.exists(path):
        return

    with open(path, "rb") as f:
        _pdf_bytes = f.read()

    # Primary action: save into the team's project (the system of record). Zero
    # setup — persists in Supabase alongside decisions/notes, isolated per team.
    proj = ss.get("project_store")
    if proj is not None:
        if st.button("💾 Save to team project", type="primary", key="rep_save"):
            try:
                from suspension.report_store import ReportStore
                client = getattr(getattr(proj, "backend", None), "client", None)
                store = ReportStore(proj, supabase_client=client)
                rec = store.save_report(path, record,
                                        ss.get("report_pdf_name", "report.pdf"))
                proj.save()
                fallback = getattr(store, "_store_reason", "")
                if fallback:
                    st.warning(f"Saved — but {fallback}", icon="⚠️")
                else:
                    st.success(f"Saved to the team project as **{rec.title}** "
                               f"({rec.provenance_summary}). Everyone in the "
                               "workspace can see it.", icon="✅")
            except Exception as e:
                st.error(f"Couldn't save to the project: {e}. "
                         "Use the download below instead.")

    st.download_button("⬇️ Download report PDF", _pdf_bytes,
                       file_name=ss.get("report_pdf_name", "report.pdf"),
                       mime="application/pdf", key="rep_dl")

    # ---- Drive export: OPTIONAL. Reports already persist in the project; this
    # pushes a browsable copy to Google Drive for teams that want one. -------
    st.markdown("#### Also export to Google Drive (optional)")
    st.caption("Reports already save to your team project above. Export to Drive "
               "only if you want an externally-browsable copy — it's not required "
               "for storage.")

    # Capture an OAuth redirect the moment we land back from Google, before
    # anything else — the ?code=&state= is in the URL exactly once.
    _capture_oauth_return()

    sa_ok, sa_reason = dx.available(read_credential=_read_credential)
    user_token = ss.get("drive_user_token")
    oauth_ok, oauth_reason = ox.oauth_available(read_credential=_read_credential)

    # Path A: a per-user token is already connected -> frictionless from here on
    if ox.token_is_usable(user_token):
        email = ss.get("drive_user_email") or "your Google account"
        st.success(f"Connected to Drive as **{email}**. Reports file into "
                   "`KinematiK Reports / <Team> / <Year> /` in your Drive.",
                   icon="✅")
        _export_button(ss, record, path, oauth_token=user_token)
        if st.button("Disconnect Drive", key="rep_disc"):
            ss.pop("drive_user_token", None)
            ss.pop("drive_user_email", None)
            st.rerun()
        return

    # Path B: service account configured -> zero per-user friction
    if sa_ok:
        st.success("Drive is configured for the team — reports file into "
                   "`KinematiK Reports / <Team> / <Year> /`.", icon="✅")
        _export_button(ss, record, path, oauth_token=None)
        # still offer personal connect as an option
        if oauth_ok:
            st.caption("Prefer reports in your *own* Drive instead of the team "
                       "Shared Drive?")
            _connect_button(ss, _read_credential)
        return

    # Path C: nothing configured yet -> offer one-click connect if OAuth is set up
    if oauth_ok:
        st.info("Connect your Google account once to export reports straight to "
                "your Drive. It's a single Google consent screen — then it's "
                "one click forever after.", icon="🔗")
        _connect_button(ss, _read_credential)
        return

    # Path D: neither configured -> honest, actionable, download still works
    st.info("Direct Drive export isn't set up yet, so use the download above. "
            f"{sa_reason}", icon="🔌")
    with st.expander("How to enable Drive export (deployer, one-time)"):
        st.markdown(
            "- **Team Shared Drive (zero per-member friction, recommended):** "
            "create a Google Cloud service account, share your Shared Drive with "
            "its email, and paste its JSON key into secrets as "
            "`GOOGLE_SERVICE_ACCOUNT_JSON`. Members then export with one click and "
            "never see an OAuth screen.\n"
            "- **Per-member Drive (each member's own account):** create an OAuth "
            "*Web* client and add `GOOGLE_OAUTH_CLIENT_ID`, "
            "`GOOGLE_OAUTH_CLIENT_SECRET`, `GOOGLE_OAUTH_REDIRECT_URI` (the app's "
            "URL) to secrets. Members then get a one-click ‘Connect Google "
            "Drive’ button.\n\n"
            "Reports are filed as `KinematiK Reports / <Team> / <Year> /` either "
            "way.")


def _connect_button(ss, read_credential):
    """Render the single 'Connect Google Drive' button that starts OAuth."""
    if st.button("🔗 Connect Google Drive", type="primary", key="rep_connect"):
        url, state = ox.build_auth_url(read_credential=read_credential)
        if url is None:
            st.error(f"Couldn't start the Google connection: {state}")
            return
        ss["drive_oauth_state"] = state
        # Google's consent page must open top-level; a link_button lets the user
        # click through and Google redirects back to the app with ?code=&state=.
        st.link_button("Continue to Google →", url, type="primary")
        st.caption("You'll pick your Google account, approve Drive access once, "
                   "and land right back here — connected.")


def _capture_oauth_return():
    """If we've just been redirected back from Google, exchange the code for a
    token automatically and cache it. Runs at the top of the Drive section so the
    user sees 'Connected' without any extra click."""
    ss = st.session_state
    try:
        qp = st.query_params
        code = qp.get("code")
        returned_state = qp.get("state")
    except Exception:
        return
    if not code or ss.get("drive_user_token"):
        return
    expected = ss.get("drive_oauth_state", "")
    token, reason = ox.exchange_code(code, expected, returned_state or "",
                                     read_credential=_read_credential)
    if token:
        ss["drive_user_token"] = token
        ss["drive_user_email"] = ox.connected_account_email(token)
        # clear the code/state from the URL so a refresh doesn't re-exchange
        try:
            for k in ("code", "state", "scope", "authuser", "prompt"):
                if k in st.query_params:
                    del st.query_params[k]
        except Exception:
            pass
        st.rerun()
    else:
        st.warning(f"Google connection didn't complete: {reason}")


def _export_button(ss, record, path, oauth_token):
    """The actual 'Export to Drive' action, shared by both auth paths."""
    if st.button("📤 Export to Drive", key="rep_drive"):
        with st.spinner("Uploading to Drive…"):
            res = dx.export_report(
                path, ss.get("report_pdf_name", "report.pdf"),
                team=record.team or "General",
                read_credential=_read_credential,
                oauth_token=oauth_token,
                shared_drive_id=_read_credential("GOOGLE_SHARED_DRIVE_ID"),
                root_folder_id=_read_credential("GOOGLE_DRIVE_ROOT_FOLDER_ID"))
        if res.ok:
            link = f" [Open in Drive]({res.web_link})" if res.web_link else ""
            st.success(f"Exported to **{res.folder_path}**.{link}")
        else:
            st.error(f"Drive export did not complete: {res.reason} "
                     "Your PDF is still available via the download button above.")
