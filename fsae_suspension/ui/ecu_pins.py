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
        st.download_button("Download report (.txt)", ecu_pins.format_report(rep), file_name="ecu_pin_check.txt",
                           mime="text/plain", key="ecu_pins_dl")
