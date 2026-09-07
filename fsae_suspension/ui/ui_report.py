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

try:
    import streamlit as st
except Exception:
    st = None

from suspension.report import (
    CalculationRecord, OutputRow, build_report, suggested_filename,
)
from suspension import drive_export as dx


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
    if st is None:
        raise RuntimeError("streamlit not available")
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
        st.download_button("⬇️ Download report PDF", f.read(),
                           file_name=ss.get("report_pdf_name", "report.pdf"),
                           mime="application/pdf", key="rep_dl")

    # ---- Drive export (only when it can actually work) -----------------
    st.markdown("#### Export to Google Drive")
    can_export, reason = dx.available(read_credential=_read_credential)
    if not can_export:
        st.info(
            "Direct Drive export isn't set up yet, so use the download above. "
            f"To enable it: {reason}", icon="🔌")
        with st.expander("How to enable Drive export"):
            st.markdown(
                "- **Team Shared Drive (recommended):** create a Google Cloud "
                "service account, share your Shared Drive (or a folder) with its "
                "email, and paste its JSON key into secrets as "
                "`GOOGLE_SERVICE_ACCOUNT_JSON`. Every member's reports then land "
                "in one organised team Drive.\n"
                "- **Per-member Drive:** configure an OAuth client and let each "
                "member authorise once; their reports go to their own Drive.\n\n"
                "Reports are filed as `KinematiK Reports / <Team> / <Year> /`.")
        return

    st.success("Drive is configured — reports file into "
               "`KinematiK Reports / <Team> / <Year> /`.", icon="✅")
    if st.button("📤 Export to Drive", key="rep_drive"):
        with st.spinner("Uploading to Drive…"):
            res = dx.export_report(
                path, ss.get("report_pdf_name", "report.pdf"),
                team=record.team or "General",
                read_credential=_read_credential,
                shared_drive_id=_read_credential("GOOGLE_SHARED_DRIVE_ID"),
                root_folder_id=_read_credential("GOOGLE_DRIVE_ROOT_FOLDER_ID"))
        if res.ok:
            st.success(f"Exported to **{res.folder_path}**. "
                       f"{f'[Open in Drive]({res.web_link})' if res.web_link else ''}")
        else:
            # honest failure — no fake success, the download still works
            st.error(f"Drive export did not complete: {res.reason} "
                     "Your PDF is still available via the download button above.")
