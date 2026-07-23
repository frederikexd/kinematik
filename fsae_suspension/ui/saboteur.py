# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/saboteur.py — the 🧨 Saboteur tab (adversarial pre-flight)
# ============================================================================
"""
The tab that attacks the deck before ANSYS can. It injects every catalogued
corruption class (unit slips, frame flips, dropped and doubled roll-up terms)
into a shadow copy of the live uncertainty ledger, shows which corruptions
would come back from a sim LOOKING PLAUSIBLE, picks the smallest set of
tripwire checksums that exposes them, seals that pre-flight sheet, and — when
the run's readings come back — fingerprints any tripped pattern against the
predicted signatures so the audit starts with a named suspect.

All physics and all state transitions live in suspension/saboteur.py.
This module only orchestrates and draws (see ui/__init__.py rules).
Session keys used:
    proof_pedigree    dict quantity_key -> pedigree override (SHARED with the
                      Proof Planner — one pedigree, two consumers, on purpose)
    sabotage_sheets   list of PreflightSheet dicts
    ledger            the shared IntegrationLedger dict (read-only here)
"""

from __future__ import annotations


def _frame_note(ss) -> str:
    ch = ss.get("frames_charter") or {}
    name = ch.get("frame_name") or ch.get("name") or ""
    return f"{name} (team charter)" if name else ""


def render():
    import streamlit as st
    from suspension import units as _units
    from suspension import proof_engine as pe
    from suspension import saboteur as sab
    from suspension.interfaces import IntegrationLedger

    ss = st.session_state
    ss.setdefault("proof_pedigree", {})
    ss.setdefault("sabotage_sheets", [])

    st.subheader("🧨 Saboteur — which input errors would you fail to notice?")
    st.caption(
        "Mutation testing for the input deck. KinematiK corrupts a shadow "
        "copy of the ledger with the classic garbage classes — lb into kg, "
        "the kilo prefix slipping, a Z-down sheet in a Z-up deck, a subsystem "
        "dropped from the roll-up — and asks, for each one: would anyone "
        "notice? The ones nobody would notice get tripwires: cheap checksums "
        "recorded with the run, chosen by detectability arithmetic, sealed "
        "before the run like a validation contract. Deterministic: same "
        "deck, same kill board.")

    led = IntegrationLedger.from_dict(ss.get("ledger") or {})
    quantities = pe.build_uncertainty_ledger(led, overrides=ss.proof_pedigree)
    if not quantities:
        st.info("The Integration ledger has no declared numbers yet. Declare "
                "your subsystem interfaces in 🔗 Integration first — the "
                "Saboteur corrupts numbers that exist.")
        return

    # ---------------- 1 · objective & optional contract band ----------------
    objs = {o.label: o for o in pe.DEFAULT_OBJECTIVES}
    olabel = st.selectbox("Result the run will produce", list(objs.keys()),
                          index=1, key="sab_obj")
    obj = objs[olabel]

    pass_band = None
    open_contracts = [c for c in ss.get("proof_contracts", [])
                      if c.get("status") == "open"]
    if open_contracts:
        titles = ["(none)"] + [c["title"] for c in open_contracts]
        pick = st.selectbox(
            "Judge against a sealed contract's acceptance band (asks the "
            "scariest question: could garbage hand you a PASS?)",
            titles, key="sab_contract")
        if pick != "(none)":
            c = next(c for c in open_contracts if c["title"] == pick)
            pass_band = (c["pass_lo"], c["pass_hi"])

    report = sab.run_sweep(obj, quantities, pass_band=pass_band)
    n = len(report.findings)
    silent = report.silent_killers

    a, b, c3 = st.columns(3)
    a.metric("Corruptions injected", f"{n}")
    b.metric("Caught by the result alone",
             f"{(n - len(silent))}/{n}",
             help="Corruptions that push the result outside its own 3σ "
                  "plausibility envelope — the Proof Engine's DISCREPANT "
                  "verdict already owns these.")
    c3.metric("Would come back looking plausible", f"{len(silent)}",
              delta=None if not silent else "the dangerous ones",
              delta_color="inverse")
    if pass_band is not None:
        fakers = [f for f in report.findings if f.fakes_pass]
        if fakers:
            st.error(f"**{len(fakers)}** catalogued corruption(s) would land "
                     "the result *inside the sealed acceptance band* — the "
                     "contract itself would smile at the garbage. The sheet "
                     "below is what stands between those and a false PASS.")

    # ---------------- 2 · the kill board ------------------------------------
    with st.expander("2 · Kill board — every corruption, and who catches it",
                     expanded=False):
        st.caption(
            "σ is the shift each corruption causes, in units of the "
            "channel's own band. ≥3σ = caught. *Envelope* means the primary "
            "result exposes it; *wire* names the checksum that does; "
            "**nothing** means it sails through — that row is why this tab "
            "exists.")
        rows = []
        for f in report.findings:
            if f.envelope_catches:
                catch = "envelope"
            elif f.caught_by:
                catch = "wire: " + ", ".join(f.caught_by)
            else:
                catch = "⚠️ NOTHING"
            rows.append({
                "corruption": f.mutation_label,
                "strikes": f.target_label,
                "result shift": f"{f.delta_objective:+.3g} {report.unit} "
                                f"({f.objective_sigmas:.1f}σ)",
                "caught by": catch,
                **({"fakes a PASS": "✔" if f.fakes_pass else ""}
                   if pass_band is not None else {}),
            })
        st.dataframe(rows, use_container_width=True, hide_index=True)

    # ---------------- 3 · the sealed pre-flight sheet -----------------------
    st.markdown("### 🪤 Pre-flight sheet — the checksums to record with "
                "every run")
    sheet_now = sab.build_sheet(report, author=ss.get("member_name", "") or "")
    st.progress(sheet_now.coverage_before,
                text=f"Detection without the sheet: "
                     f"{sheet_now.coverage_before * 100:.0f}% of the catalog")
    st.progress(sheet_now.coverage_after,
                text=f"Detection with the sheet: "
                     f"{sheet_now.coverage_after * 100:.0f}%")
    for w in sheet_now.wires:
        lo = w["clean"] - 3.0 * w["band"]
        hi = w["clean"] + 3.0 * w["band"]
        st.markdown(f"- **{w['label']}** — expect **{w['clean']:.4g} "
                    f"{w['unit']}**, must land in [{lo:.4g}, {hi:.4g}] "
                    f"(±{w.get('tol', 0) * 100:.0f}%). _{w['how']}_")
    if sheet_now.blind_spots:
        st.warning(
            "**Honest blind spots** — invisible to the result *and* to every "
            "available tripwire; the only defence is measuring the input "
            "directly:\n"
            + "\n".join(f"- {b['mutation_label']} striking "
                        f"*{b['target_label']}*"
                        for b in sheet_now.blind_spots))
        st.caption("Blind spots usually mean a channel with no declared "
                   "cross-check partner — declaring the missing channels in "
                   "🔗 Integration brings more wires online.")

    if st.button("🔏 Seal this sheet for the next run", key="sab_seal"):
        ss.sabotage_sheets.append(sheet_now.as_dict())
        st.success(f"Sealed. Seal `{sheet_now.seal[:16]}…` — the wire set "
                   "and bands are now fixed; tape the sheet next to the "
                   "solver seat.")
    st.download_button(
        "⬇️ Pre-flight sheet (markdown, tape it to the ANSYS seat)",
        sab.render_preflight_md(sheet_now, report,
                                frame_note=_frame_note(ss)),
        file_name=f"preflight_{obj.key}.md", mime="text/markdown",
        key="sab_dl")

    # ---------------- 4 · judge a run's readings ----------------------------
    if ss.sabotage_sheets:
        st.divider()
        st.markdown("### 🕵️ Judge a run — enter the readings, the garbage "
                    "names itself")
        st.caption(
            "Type the checksums the run actually reported. All inside their "
            "sealed bands: the catalogued corruption classes are excluded. "
            "Any wire tripped: the deviation pattern is matched against every "
            "predicted corruption signature (cosine on band-normalised "
            "deviations — reproducible by hand) and the audit starts with a "
            "named suspect instead of an evening of guessing.")
        for si, sd in enumerate(reversed(ss.sabotage_sheets)):
            sh = sab.PreflightSheet.from_dict(sd)
            ok = sh.verify_seal()
            head = (f"Sheet sealed {sh.created_on} · "
                    f"{len(sh.wires)} wires · "
                    f"{sh.coverage_after * 100:.0f}% coverage")
            if not ok:
                head = "⚠️ SEAL BROKEN — " + head
            with st.expander(head, expanded=(si == 0)):
                if not ok:
                    st.error("A sealed field changed after creation. This "
                             "sheet judges nothing; re-seal above.")
                    continue
                readings = {}
                cols = st.columns(min(len(sh.wires), 3))
                for wi, w in enumerate(sh.wires):
                    readings[w["key"]] = cols[wi % len(cols)].number_input(
                        f"{w['label']} ({w['unit']})",
                        value=float(w["clean"]),
                        key=f"sab_r_{sh.id}_{w['key']}",
                        format="%.6g")
                if st.button("Judge readings", key=f"sab_j_{sh.id}"):
                    try:
                        v = sab.judge_readings(sh, readings)
                    except ValueError as e:
                        st.error(str(e))
                        continue
                    if v.status == "clean":
                        st.success("🟢 CLEAN — " + v.note)
                    elif v.status == "incomplete":
                        st.warning(v.note)
                    else:
                        st.error("🔴 TRIPPED — " + v.note)
                        for s_ in v.suspects[:3]:
                            st.markdown(
                                f"- suspect `{s_['finding']}` — direction "
                                f"match {s_['cosine']:.2f}, magnitude "
                                f"{s_['magnitude_ratio']:.2f}× predicted")
                        st.caption(
                            "A tripped sheet means the run consumed a deck "
                            "that disagrees with this one — do not judge the "
                            "primary result's validation contract until the "
                            "suspect is cleared. Frame flips: 🧭 Frames & "
                            "Datums. Roll-up terms: 🔗 Integration.")
