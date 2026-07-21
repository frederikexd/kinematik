# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/inverse_genesis_fullcar.py — the 🧬🏁 InverseGenesis-FullCar tab
#  (declare the track + rulebook + objective · synthesize the whole car)
# ============================================================================
"""
The tab that runs the SEASON'S loop backwards.

The corner-level InverseGenesis tab reverses one loop (curves → hardpoints).
This tab reverses the biggest one: state the objective (points on this track
under this rulebook) and the engine synthesizes the battery configuration,
the drive architecture, the gear ratio, then the kinematic intent, the
hardpoints (through the corner engine), the structural load cases, and the
firmware calibration — one consistent car, evaluated through the repo's own
QSS lap chain and transient pack-thermal network.

Four declarations, one synthesized car:

  1. THE RULE MATRIX — the FSAE-EV constraint bounds (power/voltage/segment
     caps, wheelbase minimum, cell temperature limit, endurance distance).
     Seeded near the common EV rules; NONE of it is scrutineering.
  2. THE DESIGN SPACE — the choices the engine may make (series/parallel
     range, architectures, gear range) and the fixed car around them.
  3. THE OBJECTIVE — maximum points; optionally anchored to declared event
     bests, else scored relative to the field.
  4. SYNTHESIZE — the staged inverse search runs; the winner is the highest-
     points car THAT FINISHES THE SEASON, with every rejected faster-but-DNF
     candidate named.

All physics lives in suspension/inverse_genesis_fullcar.py; this module only
orchestrates the engine and draws (see ui/__init__.py rules).

Session keys used:
    fullcar_last   summary dict of the last synthesis (for cross-tab reads)
    genesis_targets (write): the winner's derived kinematic intent, so the
                   corner InverseGenesis tab can pick up where this leaves off.
"""

from __future__ import annotations

_ARCH_UI = {
    "single_diff": "1 motor + diff",
    "twin_axle":   "2 motors (axle split)",
    "four_tv":     "4 motors (torque vectoring)",
}

_VERDICT_UI = {
    "FEASIBLE": ("🟢", "Hits the objective AND finishes the season — energy "
                       "and cell temperature both inside the rules. This is "
                       "the car to develop."),
    "ENERGY_SHORT": ("🟠", "Fast enough, but the pack does not cover the "
                           "endurance energy; it must derate to finish, and "
                           "the lap-time cost of that is priced in."),
    "THERMAL_DNF": ("🔴", "Wins on paper and cooks its cells mid-endurance — "
                          "the overheat lap is computed from the cell's own "
                          "time-to-limit, not guessed. Rejected on purpose."),
    "RULE_KILLED": ("⚪", "Breaks a bound in the rule matrix; never "
                         "evaluated. The binding rule is named."),
    "FAILED": ("⚫", "The QSS chain could not follow this car."),
}


def render():
    import numpy as np
    import pandas as pd
    import streamlit as st
    from suspension import inverse_genesis_fullcar as fc
    from suspension.pack_thermal import CellParams

    ss = st.session_state

    st.subheader("🧬🏁 InverseGenesis-FullCar — declare the objective; "
                 "synthesize the car")
    st.caption(
        "The corner engine reverses one loop; this reverses the season's. "
        "State the track, the rulebook bounds and the points objective, and "
        "the engine walks the design chain BACKWARDS — points → battery "
        "configuration, architecture and gear → kinematic intent → hardpoints "
        "→ structural load cases → firmware constants. One consistent car per "
        "candidate: pack size sets mass, mass sets lap time AND energy AND "
        "cell current, current sets temperature, temperature decides whether "
        "Endurance finishes. The winner is the highest-points car that "
        "FINISHES THE SEASON — the fastest-on-paper car that overheats on "
        "lap 9 loses on purpose. Event times come from the QSS lap chain "
        "(relative comparison is its strength); validate the winner in the "
        "high-fidelity tabs before cutting metal.")

    st.info(
        "**The rule matrix is seeded near the common FSAE-EV rules — it is "
        "NOT scrutineering.** Every bound is editable and must be verified "
        "against your competition year before a review trusts a verdict. And "
        "there is no \"millions of states per second\": the integer grid is "
        "enumerated exhaustively, the gear ratio refined by golden-section "
        "search, and the exact evaluation count is printed with the result.")

    # ================= 1 · the rule matrix ================================
    st.markdown("###### 1 · The rule matrix — the constraint bounds")
    r1, r2, r3 = st.columns(3)
    max_power = r1.number_input("Max TS power (kW)", 20.0, 200.0, 80.0, 5.0,
                                key="fc_pwr")
    max_ts_v = r2.number_input("Max TS voltage (VDC)", 100.0, 800.0, 600.0,
                               10.0, key="fc_tsv")
    endurance_km = r3.number_input("Endurance distance (km)", 5.0, 30.0, 22.0,
                                   0.5, key="fc_endkm")
    r4, r5, r6 = st.columns(3)
    max_seg_v = r4.number_input("Max segment voltage (VDC)", 40.0, 200.0,
                                120.0, 5.0, key="fc_segv")
    max_seg_e = r5.number_input("Max segment energy (MJ)", 1.0, 12.0, 6.0,
                                0.5, key="fc_sege")
    cell_limit = r6.number_input("Cell temp limit (°C)", 40.0, 80.0, 60.0,
                                 1.0, key="fc_tlim")
    min_wb = st.number_input("Minimum wheelbase (mm)", 1400.0, 1800.0, 1525.0,
                             5.0, key="fc_wb")
    rules = fc.RuleMatrix(
        max_power_kw=max_power, max_ts_voltage=max_ts_v,
        max_segment_voltage=max_seg_v, max_segment_energy_mj=max_seg_e,
        min_wheelbase_mm=min_wb, cell_temp_limit_c=cell_limit,
        endurance_km=endurance_km)

    # ================= 2 · the design space ===============================
    st.markdown("###### 2 · The design space — what the engine may choose")
    s1, s2 = st.columns(2)
    series_lo, series_hi = s1.slider("Series count range (sets voltage)",
                                     60, 160, (84, 132), 4, key="fc_ser")
    series_step = s2.select_slider("Series step (enumerate every N)",
                                   [4, 8, 12, 16], value=12, key="fc_serstep")
    p1, p2 = st.columns(2)
    par_lo, par_hi = p1.slider("Parallel count range (splits current)",
                               2, 10, (4, 7), 1, key="fc_par")
    gear_lo, gear_hi = p2.slider("Final-drive ratio range", 2.0, 6.0,
                                 (2.8, 5.0), 0.1, key="fc_gear")
    archs = st.multiselect(
        "Drive architectures to consider", list(_ARCH_UI),
        default=list(_ARCH_UI), format_func=lambda a: _ARCH_UI[a],
        key="fc_arch")
    if not archs:
        st.info("Pick at least one architecture for the engine to choose "
                "between.")
        return

    with st.expander("The fixed car & cell (the search does not move these)"):
        f1, f2, f3 = st.columns(3)
        base_mass = f1.number_input("Base mass excl. cells & motors (kg)",
                                    120.0, 320.0, 215.0, 5.0, key="fc_bm")
        wheelbase = f2.number_input("Wheelbase (mm)", 1400.0, 1800.0, 1550.0,
                                    5.0, key="fc_wbcar")
        ambient = f3.number_input("Cooling-air inlet (°C)", 10.0, 45.0, 30.0,
                                  1.0, key="fc_amb")
        c1, c2, c3 = st.columns(3)
        cap_ah = c1.number_input("Cell capacity (Ah)", 1.0, 10.0, 4.5, 0.1,
                                 key="fc_cap")
        r_int = c2.number_input("Cell resistance (mΩ)", 5.0, 60.0, 22.0, 1.0,
                                key="fc_rint") / 1000.0
        max_dis = c3.number_input("Cell max discharge (A)", 10.0, 100.0, 45.0,
                                  1.0, key="fc_maxdis")
        calibrated = st.checkbox(
            "Cell thermal model is calibrated to datasheet/rig data",
            value=False, key="fc_cal",
            help="Leave off and every temperature (and every THERMAL_DNF "
                 "verdict) is physically-shaped but NOT measured — the report "
                 "says so.")

    cell = fc.CellSpec(
        capacity_ah=cap_ah, max_discharge_a=max_dis,
        thermal=CellParams(r_internal_ohm=r_int, temp_limit_c=cell_limit,
                           calibrated=calibrated))
    space = fc.DesignSpace(
        series_range=(int(series_lo), int(series_hi)),
        series_step=int(series_step),
        parallel_range=(int(par_lo), int(par_hi)),
        final_drive_range=(float(gear_lo), float(gear_hi)),
        architectures=tuple(archs), cell=cell,
        base_mass_kg=base_mass, wheelbase_mm=wheelbase, ambient_c=ambient)

    # ================= 3 · the objective ==================================
    st.markdown("###### 3 · The objective — points, anchored or relative")
    st.caption("Leave a best time at 0 to score that event RELATIVE to the "
               "best candidate in the search (correct for choosing a "
               "configuration; not an absolute points prediction). Enter your "
               "competition's best times to score in absolute points.")
    o1, o2, o3, o4 = st.columns(4)
    accel_best = o1.number_input("Accel best (s)", 0.0, 10.0, 0.0, 0.05,
                                 key="fc_ba")
    skid_best = o2.number_input("Skidpad best (s)", 0.0, 10.0, 0.0, 0.05,
                                key="fc_bs")
    autox_best = o3.number_input("Autocross best (s)", 0.0, 120.0, 0.0, 0.5,
                                 key="fc_bx")
    end_best = o4.number_input("Endurance best (s)", 0.0, 3000.0, 0.0, 5.0,
                               key="fc_be")
    ref = fc.PointsReference(
        accel_s=accel_best or None, skidpad_s=skid_best or None,
        autocross_s=autox_best or None, endurance_s=end_best or None)

    n_final = st.select_slider(
        "Thermal-gate the top N finalists (the expensive stage)",
        [2, 3, 4, 6, 8], value=4, key="fc_nfin")

    est = (len(range(int(series_lo), int(series_hi) + 1, int(series_step)))
           * len(range(int(par_lo), int(par_hi) + 1)) * len(archs))
    st.caption(f"Grid to enumerate: **{est} configurations** "
               f"(× a golden-section gear search each). A few thousand lap "
               "sims — seconds to a minute on a laptop.")

    if not st.button("🧬🏁 Synthesize the full car", type="primary",
                     key="fc_go"):
        return

    with st.spinner("Walking the design chain backwards…"):
        try:
            res = fc.synthesize_fullcar(space, rules, ref,
                                        n_finalists=int(n_final))
        except Exception as exc:                       # never kill the tab
            st.error(f"Synthesis failed: {exc}")
            return

    # ================= results ============================================
    st.divider()
    if res.ok and res.winner is not None:
        st.success(res.reason)
    else:
        st.error(res.reason)

    if res.winner is not None:
        w = res.winner
        d = w.derived
        icon, blurb = _VERDICT_UI.get(w.verdict, ("", ""))
        st.markdown(f"### {icon} The synthesized car — **{w.config.label()}**")
        st.caption(blurb)

        m1, m2, m3, m4 = st.columns(4)
        m1.metric("Total points",
                  f"{w.total_points:.0f}",
                  help="relative" if res.relative_scoring else "absolute")
        m2.metric("Mass", f"{d['mass_kg']:.0f} kg")
        m3.metric("Pack", f"{d['pack_energy_kwh']:.1f} kWh",
                  f"{d['pack_nominal_v']:.0f} V")
        if w.thermal is not None and w.thermal.ok:
            m4.metric("Peak cell",
                      f"{w.thermal.hottest_peak_c:.0f} °C",
                      f"{w.energy_margin_kwh:+.1f} kWh margin",
                      delta_color="off")

        # event breakdown
        rows = []
        for k in fc._EVENT_KEYS:
            t = getattr(w, f"{k}_s")
            rows.append({"event": fc._EVENT_LABEL[k],
                         "time (s)": round(t, 2 if k != "endurance" else 1),
                         "points": round(w.points.get(k, 0.0), 0)})
        rows.append({"event": "TOTAL", "time (s)": None,
                     "points": round(w.total_points, 0)})
        st.dataframe(pd.DataFrame(rows), hide_index=True,
                     use_container_width=True)

        if w.tv_yaw_note:
            st.caption("ℹ️ " + w.tv_yaw_note)

    # candidate field
    if res.ranked:
        st.markdown("###### Candidate field (best first)")
        frows = []
        for i, s in enumerate(res.ranked[:15], 1):
            note = (f"overheat lap {s.overheat_lap}" if s.overheat_lap
                    else (s.kill_reasons[0][:44] if s.kill_reasons
                          else "finishes"))
            icon = _VERDICT_UI.get(s.verdict, ("", ""))[0]
            frows.append({
                "#": i, "configuration": s.config.label(),
                "verdict": f"{icon} {s.verdict}",
                "points": round(s.total_points, 0),
                "E margin (kWh)": (round(s.energy_margin_kwh, 2)
                                   if np.isfinite(s.energy_margin_kwh)
                                   else None),
                "note": note})
        st.dataframe(pd.DataFrame(frows), hide_index=True,
                     use_container_width=True)

    if res.rule_killed:
        with st.expander(f"⚪ Rule-killed configs ({len(res.rule_killed)}) — "
                         "each names its broken bound"):
            for s in res.rule_killed[:20]:
                st.markdown(f"- **{s.config.label()}**: {s.kill_reasons[0]}")

    # the honesty ledger
    st.markdown("###### What the search actually did")
    st.code(res.diagnostics.summary() + "  " + res.diagnostics.timing(),
            language=None)
    for wmsg in res.warnings:
        st.warning(wmsg)

    # ---- downstream synthesis & exports ---------------------------------- #
    if res.ok and res.winner is not None:
        st.divider()
        st.markdown("###### Hand-offs — intent, load cases, and exports")
        tabs = st.tabs(["Kinematic intent", "Load cases",
                        "CAD coordinates", "Firmware constants"])

        with tabs[0]:
            st.caption("The winner's DERIVED kinematic intent, in the corner "
                       "engine's dialect — roll-cancelling camber gain from "
                       "its own peak lateral g, dead bump steer, held roll "
                       "centre. Hand this to the InverseGenesis tab to "
                       "generate the hardpoints with build-yield pricing.")
            try:
                tg = fc.kinematic_intent_for(res.winner, space)
                irows = []
                for c in tg.curves:
                    for t, v, b in zip(c.travel_mm, c.target, c.band):
                        irows.append({"channel": c.channel,
                                      "travel (mm)": round(float(t), 1),
                                      "target": round(float(v), 3),
                                      "± band": round(float(b), 3)})
                st.dataframe(pd.DataFrame(irows), hide_index=True,
                             use_container_width=True)
                if st.button("Send this intent to the InverseGenesis tab",
                             key="fc_send_intent"):
                    ss["genesis_targets"] = tg
                    st.success("Intent staged. Open the 🧬 InverseGenesis "
                               "tab, declare a legal volume, and generate.")
            except Exception as exc:
                st.info(f"Intent synthesis unavailable: {exc}")

        with tabs[1]:
            st.caption("The peak-cornering outer-wheel load resolved through "
                       "the linkage into per-member axial forces — the load "
                       "table to hand the frame/FEA seat.")
            try:
                lc = fc.load_case_for(res.winner, space)
                st.markdown(f"Outer-tyre vertical load **{lc.fz_n:.0f} N** at "
                            f"**{lc.mu_lateral:.2f} g** lateral.")
                lrows = [{"member": k, "axial force (N, + tension)":
                          round(v, 0)} for k, v in lc.member_forces.items()]
                st.dataframe(pd.DataFrame(lrows), hide_index=True,
                             use_container_width=True)
                if lc.note:
                    st.caption("Note: " + lc.note)
            except Exception as exc:
                st.info(f"Load-case synthesis unavailable: {exc}")

        with tabs[2]:
            st.caption("The nominal corner geometry as a coordinate table — "
                       "the CAD/DXF tools' input, not STEP (KinematiK carries "
                       "no CAD kernel).")
            try:
                from suspension.kinematics import Hardpoints
                csv = fc.export_hardpoints_csv(Hardpoints.default())
                st.code(csv, language=None)
                st.download_button("Download hardpoints.csv", csv,
                                   file_name="hardpoints.csv",
                                   mime="text/csv", key="fc_dl_csv")
            except Exception as exc:
                st.info(f"CAD export unavailable: {exc}")

        with tabs[3]:
            st.caption("The derived control CALIBRATION — power/current "
                       "limits, regen bounds, drive-grip ceilings, BMS "
                       "thresholds. Constants a control stack consumes, NOT a "
                       "control stack. Verify against your hardware before "
                       "flashing.")
            try:
                ch = fc.export_flash_constants_c(res.winner, space, rules)
                st.code(ch, language="c")
                cpy = fc.export_flash_constants_py(res.winner, space, rules)
                dl1, dl2 = st.columns(2)
                dl1.download_button("Download calib.h", ch,
                                    file_name="fullcar_calib.h",
                                    key="fc_dl_h")
                dl2.download_button("Download calib.py", cpy,
                                    file_name="fullcar_calib.py",
                                    key="fc_dl_py")
            except Exception as exc:
                st.info(f"Firmware export unavailable: {exc}")

    # stash a small summary for cross-tab reads
    try:
        ss["fullcar_last"] = {
            "ok": res.ok,
            "winner": (res.winner.config.label()
                       if res.winner else None),
            "verdict": res.winner.verdict if res.winner else None,
            "points": (res.winner.total_points if res.winner else None),
            "relative": res.relative_scoring,
        }
    except Exception:
        pass
