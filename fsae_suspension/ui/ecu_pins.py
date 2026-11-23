# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/ecu_pins.py — the ECU pin check panel in the Electronics tab
# ============================================================================
"""
Upload a PE3 `.ped` tune and the harness wiring tables; see where the tune's
channel names, Performance Electronics' pinout and the tables disagree, and
download the report. All checking lives in suspension/ecu_pins.py; this module
only reads the uploads and draws (see ui/__init__.py rules).

Uploads are read in memory for this run and are not saved to the project.
"""

from __future__ import annotations

_LEVEL_STYLE = {"mismatch": "error", "not wired": "warning", "check": "warning", "ok": "success"}


def render():
    import streamlit as st

    from suspension import ecu_pins

    with st.expander("🔌 ECU pin check — PE3 tune against the harness wiring tables", expanded=False):
        st.caption(
            "Reads the channel names a PE3 tune stores (for example 'Fan' on Digital Output #1), maps them to pins "
            "with Performance Electronics' published PE3-84XX pinout, and compares them with every ECU pin your "
            "wiring tables reference ('ECU_MAIN.6' or 'Pin M-6'). Only names are read, not the output settings "
            "behind them. Files are read in memory for this check and are not saved to the project.")
        tune = st.file_uploader("PE3 tune (.ped)", type=["ped"], key="ecu_pins_ped")
        tables = st.file_uploader("Wiring tables (.xlsx or .csv, one or more)", type=["xlsx", "xlsm", "csv"],
                                  accept_multiple_files=True, key="ecu_pins_tables")
        if not tune or not tables:
            st.info("Add a tune and at least one wiring table to run the check.")
            return
        try:
            rep = ecu_pins.run((tune.name, tune.getvalue()), [(t.name, t.getvalue()) for t in tables])
        except ValueError as err:
            st.error(f"Could not read the tune: {err}")
            return
        if rep.names:
            st.markdown("**Tune channel names:** " + ", ".join(f"{k} '{v}'" for k, v in sorted(rep.names.items())))
        else:
            st.warning("No renamed channels found in the tune, so only the wiring tables against the pinout are checked.")
        st.markdown(f"**ECU pins referenced by the wiring tables:** {len(rep.wired)}")
        for f in rep.findings:
            getattr(st, _LEVEL_STYLE[f.level])(f"{f.level}: {f.message}")
        if not rep.problems:
            st.success("No disagreement between tune names, pinout and wiring tables.")
        import pandas as pd
        rows = ecu_pins.pin_rows(rep)
        if rows:
            # The heading carries the legend into the feature report, which does not capture captions.
            st.session_state["_ax_next_heading"] = ("Every wired pin against the PE3-84XX pinout ('matches' = the "
                                                    "wiring table uses the pinout's name for the pin)")
            st.markdown("**Every wired pin against the PE3-84XX pinout**")
            st.dataframe(pd.DataFrame([{"Pin": r.pin, "PE3-84XX function": r.function, "Wiring tables say": r.wiring,
                                        "Result": r.result} for r in rows]), hide_index=True, width="stretch")
            st.caption(ecu_pins.PIN_LEGEND)
        todo = ecu_pins.confirm_list(rep)
        if todo:
            st.session_state["_ax_next_heading"] = ("Confirm in PE3Monitor before first power-up (which function "
                                                    "an output switches is not readable from the tune)")
            st.markdown("**Confirm in PE3Monitor before first power-up.** Which function an output switches is not "
                        "readable from the tune, so check these by eye; a relay wired to an output nobody assigned is "
                        "caught here.")
            st.dataframe(pd.DataFrame([{"Confirm": t} for t in todo + [ecu_pins.OUTPUT_DIAGNOSTICS]]),
                         hide_index=True, width="stretch")
        # A warning, not a caption or st.info: warnings are what the feature report captures, and a report
        # carrying the green findings without what was not read overstates the check.
        st.warning(f"**Scope of this check.** {ecu_pins.CAVEAT}")
        # The files as a table, not a run-on sentence of hashes: readable on screen, and a table is what the
        # feature report captures under charts & tables. The heading key is the capture layer's
        # (_CAPTURE_HEADING_KEY in streamlit_app.py), so the table is named in the report.
        title = "Files checked"
        st.session_state["_ax_next_heading"] = title
        st.markdown(f"**{title}**")
        st.dataframe(pd.DataFrame([{"File": n, "SHA-256": d} for n, d in rep.inputs]), hide_index=True,
                     width="stretch")
        c1, c2 = st.columns(2)
        c1.download_button("Download report (.txt)", ecu_pins.format_report(rep), file_name="ecu_pin_check.txt",
                           mime="text/plain", key="ecu_pins_dl")
        try:
            c2.download_button("Download report (.pdf)", ecu_pins.report_pdf(rep), file_name="ecu_pin_check.pdf",
                               mime="application/pdf", key="ecu_pins_pdf")
        except Exception as err:                 # noqa: BLE001 — the .txt is still there
            c2.caption(f"PDF not available: {err}")
