# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/earshot.py — the 🎙️ Earshot tab (the test-day power audit)
# ============================================================================
"""
The tab that asks, before the trailer loads: CAN this session hear the
answer? Power analysis for the A-B test day, drift-confound arithmetic for
the run order, instrument propagation for what a parameter test will really
deliver (and the evidence grade it EARNS), and a sha256-sealed session sheet
so what counts as a detection was decided before the result existed.

All statistics live in suspension/earshot.py. This module only orchestrates
and draws (see ui/__init__.py rules).
Session keys used:
    proof_pedigree    SHARED with Proof Planner / Saboteur / Phantom Car —
                      one pedigree, fourth consumer, on purpose
    earshot_sheet     sealed SessionSheet dict (None until sealed)
    ledger            the shared IntegrationLedger dict (read-only here)
"""

from __future__ import annotations


_AB_STYLE = {
    "RESOLVABLE":   ("🟢", "success"),
    "UNDERPOWERED": ("🟠", "warning"),
    "SWAMPED":      ("🔴", "error"),
}
_ORD_STYLE = {
    "CLEAN":      "🟢",
    "BIASED":     "🟠",
    "CONFOUNDED": "🔴",
}


def _frame_note(ss) -> str:
    ch = ss.get("frames_charter") or {}
    name = ch.get("frame_name") or ch.get("name") or ""
    return f"{name} (team charter)" if name else ""


def render():
    import streamlit as st
    from suspension import earshot as ea

    ss = st.session_state
    ss.setdefault("earshot_sheet", None)

    st.subheader("🎙️ Earshot — can the test day hear the answer?")
    st.caption(
        "The Proof Planner says WHICH test is worth doing. This tab asks the "
        "question everyone skips: as planned, is the answer even within "
        "earshot? Laps needed vs laps you have, drift bias per run order, "
        "and the band an instrument will actually deliver — all decided, and "
        "sealed, before the trailer loads.")

    tab_ab, tab_order, tab_inst, tab_seal = st.tabs(
        ["📢 A/B detectability", "🔀 Run-order audit",
         "📏 Instrument resolution", "🔏 Sealed session sheet"])

    # ------------------------------------------------------------------ A/B
    with tab_ab:
        c1, c2, c3 = st.columns(3)
        with c1:
            delta = st.number_input(
                "Predicted effect δ (s)", 0.01, 10.0, 0.30, 0.05,
                help="The lap-time delta the change is predicted to make — "
                     "from the lap sim, the aero map, or a declared claim. "
                     "If the Proof Planner has an attribution loaded, use "
                     "its objective delta here.")
            alpha = st.selectbox("Significance α", [0.10, 0.05, 0.01], index=1)
        with c2:
            sigma = st.number_input(
                "Driver lap σ (s)", 0.05, 5.0, 0.80, 0.05,
                help="Lap-to-lap standard deviation of the SAME driver on "
                     "the SAME course. Starts life as a GUESS; measure it "
                     "from ~10 baseline laps and it becomes the single "
                     "highest-leverage number on this page.")
            power = st.selectbox("Power", [0.80, 0.90], index=0,
                                 format_func=lambda p: f"{p:.0%}")
        with c3:
            from_pack = st.checkbox(
                "Laps from pack energy", value=True,
                help="The EV session budget: usable kWh over kWh per lap, "
                     "split across both configurations.")
            if from_pack:
                pk = st.number_input("Pack (kWh)", 1.0, 20.0, 7.0, 0.5)
                use = st.number_input("Usable fraction", 0.5, 1.0, 0.90, 0.05)
                epl = st.number_input("kWh per lap", 0.01, 2.0, 0.14, 0.01)
                res = st.number_input("Reserve (kWh)", 0.0, 5.0, 0.5, 0.1)
                laps = ea.laps_from_pack(pk, use, epl, res, configs=2)
                st.metric("Laps per config the pack can feed", laps)
            else:
                laps = int(st.number_input("Laps per config", 1, 500, 20))

        design = ea.ABDesign(delta, sigma, alpha, power, laps)
        v = design.verdict.value
        icon, kind = _AB_STYLE[v]
        m1, m2, m3 = st.columns(3)
        m1.metric("Laps needed / config",
                  design.laps_needed_per_config
                  if design.laps_needed_per_config <= ea.SWAMPED_LAP_LIMIT
                  else f"> {ea.SWAMPED_LAP_LIMIT}")
        m2.metric("Session can hear (MDE)", f"{design.mde():.2f} s")
        m3.metric("Chance a real effect hides", f"{design.miss_probability:.0%}")

        msg = {
            "RESOLVABLE": f"{icon} **RESOLVABLE** — {laps} laps per config "
                          f"covers the {design.laps_needed_per_config} this "
                          "effect needs. Book it — and seal the sheet first.",
            "UNDERPOWERED": f"{icon} **UNDERPOWERED** — the effect needs "
                            f"{design.laps_needed_per_config} laps per "
                            f"config; the session has {laps}. Run it anyway "
                            f"and a real gain has a "
                            f"{design.miss_probability:.0%} chance of being "
                            "read as 'doesn't work'. Grow the session, "
                            "shrink σ with a consistency stint, or test a "
                            "bigger change.",
            "SWAMPED": f"{icon} **SWAMPED** — this effect sits below the "
                       "noise floor; no bookable session can hear it. The "
                       "honest options: bundle changes into a bigger effect, "
                       "or measure the mechanism directly (pressures, loads) "
                       "instead of lap time.",
        }[v]
        getattr(st, kind)(msg)
        ss["_earshot_design"] = design.as_dict()

    # ------------------------------------------------------------ ordering
    with tab_order:
        st.caption(
            "Every session drifts — tires wear, the track rubbers in, the "
            "pack sags. The bias each run order inherits from a LINEAR drift "
            "is exact arithmetic, shown here next to the swaps it costs.")
        drifts = []
        for d0 in ea.DEFAULT_DRIFTS:
            c1, c2 = st.columns([3, 2])
            with c1:
                on = st.checkbox(f"{d0.label}", value=True, key=f"eo_{d0.key}",
                                 help=d0.note)
            with c2:
                rate = st.number_input(
                    "s/lap", -0.2, 0.2, d0.rate_per_lap, 0.005,
                    key=f"eor_{d0.key}", label_visibility="collapsed")
            if on:
                drifts.append(ea.DriftSource(d0.key, d0.label, rate, d0.note))

        dd = ss.get("_earshot_design") or {}
        design = ea.ABDesign(dd.get("effect_predicted", 0.3),
                             dd.get("noise_sigma", 0.8),
                             dd.get("alpha", 0.05), dd.get("power", 0.8),
                             dd.get("laps_available_per_config", 20))
        finds = ea.audit_orderings(design, drifts)
        rows = [{"Order": f.ordering,
                 "Net drift bias (s)": f"{f.net_bias:+.3f}",
                 "Swaps": f.swaps,
                 "Verdict": f"{_ORD_STYLE[f.verdict.value]} {f.verdict.value}",
                 "Note": f.note} for f in finds]
        st.table(rows)
        st.info(
            "AABB is what tired teams run because it costs one swap — and it "
            "hands the whole session's drift to one side of the comparison. "
            "ABBA blocks cancel linear drift exactly for a handful of swaps. "
            "The sheet also reserves burn-in laps that count for nobody, "
            "because driver learning is the steepest drift of all.")

    # ---------------------------------------------------------- instruments
    with tab_inst:
        st.caption(
            "A parameter test earns its evidence grade from instrument "
            "arithmetic, not from having been performed. MOOT means the band "
            "this plan delivers is no tighter than the ledger already is — "
            "the test, as planned, cannot teach the team anything.")
        test = st.selectbox("Planned test", [
            "Tilt test (CG height)", "Coast-down (CdA)", "Corner scales (mass)"])
        cur = st.number_input(
            "Current ledger band on this channel (± %)", 0.5, 60.0, 20.0, 0.5,
            help="From the Proof Planner's uncertainty ledger — the "
                 "evidence-graded, staleness-inflated band as of today.") / 100.0

        if test.startswith("Tilt"):
            c1, c2, c3 = st.columns(3)
            m = c1.number_input("Car+driver mass (kg)", 100.0, 500.0, 250.0)
            wb = c1.number_input("Wheelbase (mm)", 1000.0, 2500.0, 1550.0)
            h = c2.number_input("Expected CG height (mm)", 100.0, 600.0, 300.0)
            ang = c2.number_input("Tilt angle (°)", 3.0, 40.0, 8.0, 1.0)
            sres = c3.number_input("Scale resolution (kg)", 0.05, 5.0, 0.5)
            asig = c3.number_input("Angle σ (°)", 0.05, 3.0, 0.5)
            rep = int(c3.number_input("Independent repeats", 1, 10, 1))
            r = ea.resolve_tilt_cg(m, wb, h, ang, sres, asig, rep)
        elif test.startswith("Coast"):
            c1, c2, c3 = st.columns(3)
            m = c1.number_input("Mass (kg)", 100.0, 500.0, 250.0)
            cda = c1.number_input("Declared CdA (m²)", 0.3, 3.0, 1.1)
            vh = c2.number_input("High band speed (m/s)", 5.0, 40.0, 22.0)
            vl = c2.number_input("Low band speed (m/s)", 2.0, 30.0, 10.0)
            sv = c3.number_input("Speed sensor σ (m/s)", 0.01, 1.0, 0.14)
            tb = c3.number_input("Band duration (s)", 0.5, 20.0, 3.0)
            rp = int(c3.number_input("Paired runs (2 directions)", 1, 20, 4))
            r = ea.resolve_coastdown(m, cda, 1.204, vh, vl, sv, tb, rp)
        else:
            c1, c2 = st.columns(2)
            m = c1.number_input("Mass (kg)", 100.0, 500.0, 250.0)
            pres = c2.number_input("Pad resolution (kg)", 0.05, 5.0, 0.5)
            rep = int(c2.number_input("Re-zeroed repeats", 1, 10, 1))
            r = ea.resolve_corner_scales(m, pres, rep)

        r.judge_against(cur)
        k1, k2, k3 = st.columns(3)
        k1.metric("Band this plan delivers", f"±{r.delivered_rel_unc * 100:.1f} %")
        k2.metric("Grade it earns", r.earned_grade.value.upper())
        k3.metric("Verdict", r.verdict.value)
        (st.success if r.verdict.value == "SHARPENS" else st.error)(r.note)
        with st.expander("Where the band comes from (RSS terms)"):
            st.table([{"Term": k, "± %": f"{v * 100:.2f}"}
                      for k, v in sorted(r.terms.items(),
                                         key=lambda kv: -kv[1])])

    # ---------------------------------------------------------------- seal
    with tab_seal:
        dd = ss.get("_earshot_design") or {}
        design = ea.ABDesign(dd.get("effect_predicted", 0.3),
                             dd.get("noise_sigma", 0.8),
                             dd.get("alpha", 0.05), dd.get("power", 0.8),
                             dd.get("laps_available_per_config", 20))
        if ss["earshot_sheet"] is None:
            title = st.text_input("Session title", "A/B session")
            order = st.selectbox("Sealed run order", ["ABBA", "ABAB", "AABB"])
            burn = int(st.number_input("Burn-in laps (count for nobody)",
                                       0, 20, 3))
            abort = st.text_input(
                "Abort criterion (sealed)",
                "Abort if rain, red flag > 20 min, or σ of first 5 baseline "
                "laps exceeds 1.5× the declared σ.")
            if st.button("🔏 Seal the session sheet", type="primary"):
                sigma_grade = "estimate"
                sheet = ea.create_sheet(title, design, order,
                                        burn_in_laps=burn,
                                        sigma_grade=sigma_grade,
                                        abort_note=abort)
                ss["earshot_sheet"] = sheet.as_dict()
                st.rerun()
            if design.verdict.value != "RESOLVABLE":
                st.warning(
                    "You are about to seal an "
                    f"{design.verdict.value} design. Allowed — but the sheet "
                    "will carry the miss probability forever, so a "
                    "non-detection can never be quietly read as 'the change "
                    "does nothing'.")
        else:
            sheet = ea.SessionSheet.from_dict(ss["earshot_sheet"])
            sealed_ok = sheet.verify_seal()
            (st.success if sealed_ok else st.error)(
                f"{'🔏 Seal intact' if sealed_ok else '💥 SEAL BROKEN'} — "
                f"sha256 `{sheet.seal[:16]}…` · sealed {sheet.created_utc}")
            st.markdown(f"**Run order:** "
                        f"`{ea.build_sequence(sheet.ordering, sheet.laps_per_config)}`")
            if sheet.judged_verdict is None:
                c1, c2, c3 = st.columns(3)
                ma = c1.number_input("Mean lap, config A (s)", 0.0, 300.0, 55.0)
                mb = c2.number_input("Mean lap, config B (s)", 0.0, 300.0, 54.5)
                lr = int(c3.number_input("Laps actually run / config", 0, 500,
                                         sheet.laps_per_config))
                if st.button("⚖️ Judge the session"):
                    ss["earshot_sheet"] = ea.judge_session(sheet, ma, mb, lr).as_dict()
                    st.rerun()
            else:
                icon = {"DETECTED": "🟢", "NOT_DETECTED": "🟠",
                        "VOID": "🔴"}[sheet.judged_verdict]
                st.markdown(f"### {icon} {sheet.judged_verdict}")
                st.write(sheet.judged_note)
            st.download_button(
                "⬇️ Session sheet (markdown)",
                ea.render_session_md(sheet, frame_note=_frame_note(ss)),
                file_name="earshot_session_sheet.md")
            if st.button("Start a new sheet"):
                ss["earshot_sheet"] = None
                st.rerun()
