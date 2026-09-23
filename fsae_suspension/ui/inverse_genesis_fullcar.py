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

A DECLARED-CAR mode sits above the search: state one vehicle in full (mass,
distribution, CG, tracks, tyre model, roll stiffness or springs + bars, aero,
and the real front/rear corners) and evaluate it directly — vehicle summary,
lap time and one-parameter lap sensitivities — with the inputs and outputs
downloadable as JSON so the numbers can be regenerated.

Session keys used:
    fullcar_last    summary dict of the last synthesis (for cross-tab reads)
    fc_res          the last synthesis, kept across reruns
    fc_declared     the last declared-car evaluation
    genesis_corners read: {"front"/"rear": hardpoints dict} from the corner tab
    genesis_targets (write): {"targets", "axle"} — the derived or declared
                    kinematic intent, read by the corner InverseGenesis tab.
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


def _corner_hp(ss, axle):
    """The corner for ``axle``: the InverseGenesis result for that axle if one
    was produced, else the live/default geometry — and say which."""
    from suspension import genesis_repro as gr
    from ui.inverse_genesis import _hardpoints_from_session
    d = (ss.get("genesis_corners") or {}).get(axle)
    if d:
        return gr.hp_from_dict(d), f"{axle} corner from the InverseGenesis tab"
    hp, note = _hardpoints_from_session(ss)
    return hp, note


#: The vehicle-level lap-sensitivity setup: linear grip model, default aero,
#: no corner geometry, 55 % front roll split at baseline, 1200/1180 mm tracks.
_LAP_REF = {
    "declaration": {"mass_kg": 300.0, "weight_dist_front": 0.48,
                    "cg_height_mm": 280.0, "wheelbase_mm": 1630.0,
                    "track_front_mm": 1200.0, "track_rear_mm": 1180.0,
                    "tire_model": "linear", "power_kw": 80.0, "cla": 2.6,
                    "cda": 1.1, "drive": "rwd",
                    "roll_mode": "declared directly",
                    "roll_stiffness_front": 357.5,
                    "roll_stiffness_rear": 292.5},
    "widgets": {"dc_src_front": "None — vehicle-level model only",
                "dc_src_rear": "None — vehicle-level model only",
                "dc_chan": "front_roll_share",
                "dc_vals": "0.40, 0.42, 0.44, 0.46, 0.48, 0.50, 0.52, 0.54, "
                           "0.56, 0.58, 0.60, 0.62, 0.64, 0.66, 0.68, 0.70"},
}


def _load_lap_reference():
    import streamlit as st
    from ui.inverse_genesis import _veh, _VEH_PREFIXES
    ss = st.session_state
    v = _veh(ss)
    for f, x in _LAP_REF["declaration"].items():
        v[f] = x
        for p in _VEH_PREFIXES:
            ss[f"{p}_{f}"] = x
    for k, x in _LAP_REF["widgets"].items():
        ss[k] = x
    ss.pop("fc_declared", None)


def _declared_car_panel(st, pd, np, ss, fc):
    """Declared-car mode on the shared vehicle declaration: vehicle summary,
    lap sensitivity, tyre, steering effort and actuation of both axles."""
    import json
    from suspension import genesis_repro as gr
    from suspension.kinematics import Hardpoints
    from ui.inverse_genesis import (_vehicle_editor, _declared_car, _table,
                                    _floats, _sec_tyre, _sec_steering,
                                    _sec_actuation, _parse_hardpoints_text)
    from ui.inverse_genesis import _hint
    _hint(st, ss, "The vehicle & build declaration below is the same one the "
                  "InverseGenesis tab uses — enter a number once and both "
                  "tabs follow. Declared-car mode evaluates exactly that car; "
                  "the synthesis further down searches battery and gearing "
                  "around it.")
    v = _vehicle_editor(st, ss, "fc_v")
    with st.expander("🚗 Declared-car mode — evaluate the declared vehicle "
                     "(no search)", expanded=False):
        st.caption("Uses the vehicle & build declaration above and the "
                   "corners from InverseGenesis. Download the JSON to keep "
                   "the inputs with the results.")
        _hint(st, ss, "Pick the corners (a generated InverseGenesis corner "
                      "is used automatically once you have one), choose a "
                      "parameter to sweep, and press Evaluate. The lap "
                      "sensitivity shows how much lap time that parameter "
                      "is worth on this layout.")
        st.button("Load the vehicle-level lap-sensitivity setup",
                  key="dc_lapref", on_click=_load_lap_reference,
                  help="Linear grip model, ClA 2.6 / CdA 1.1, no corner "
                       "geometry, 55 % front roll split, 1200 / 1180 mm "
                       "tracks, and a 40–70 % roll-split sweep. Changes the "
                       "shared declaration — note your values first.")
        corners = {}
        waiting = []
        g = st.columns(2)
        for col, axle in zip(g, ("front", "rear")):
            src = col.selectbox(f"{axle.title()} corner",
                                ["InverseGenesis / live", "KinematiK default",
                                 "Paste hardpoints",
                                 "None — vehicle-level model only"],
                                key=f"dc_src_{axle}",
                                help="Paste hardpoints: CSV (point,x,y,z) or "
                                     "JSON, corner frame, mm — e.g. a table of "
                                     "archived hardpoints or a genesis "
                                     "manifest. None: no corner geometry is "
                                     "attached, so roll centres and camber "
                                     "come from the vehicle-level model's own "
                                     "defaults.")
            if src.startswith("None"):
                corners[axle] = None
                col.caption("No corner geometry attached.")
            elif src == "KinematiK default":
                corners[axle] = Hardpoints.default()
            elif src == "Paste hardpoints":
                txt = col.text_area(
                    f"{axle.title()} hardpoints (CSV point,x,y,z or JSON)",
                    height=150, key=f"dc_txt_{axle}",
                    placeholder="point,x,y,z\nupper_front_inner,-120.0,288.1,"
                                "280.5\n…")
                cam = col.number_input(
                    f"{axle.title()} static camber (deg)", -6.0, 3.0,
                    -1.5 if axle == "front" else -1.0, 0.05,
                    key=f"dc_cam_{axle}")
                if not txt.strip():
                    col.info("Paste the " + axle + " corner to continue.")
                    waiting.append(axle)
                    continue
                try:
                    body = txt.strip()
                    if body.startswith("{"):
                        d = json.loads(body)
                        w = (d.get("recorded") or {}).get("winner_hardpoints")
                        if w:
                            body = json.dumps(w)
                    hp = _parse_hardpoints_text(body, Hardpoints.default())
                    hp.static_camber = float(cam)
                    corners[axle] = hp
                    col.caption("Pasted " + axle + " corner, static camber "
                                + str(cam) + "°.")
                except Exception as e:      # noqa: BLE001
                    col.error(f"Could not read the {axle} corner: {e}")
                    waiting.append(axle)
            else:
                corners[axle], note = _corner_hp(ss, axle)
                col.caption(note)
        if waiting:
            return
        car = _declared_car(v, corners["front"], corners["rear"])
        st.markdown("**Lap sensitivity**")
        c = st.columns(2)
        chan = c[0].selectbox("Sweep", ["front_roll_share", "cg_height_mm",
                                        "mass_kg", "weight_dist_front",
                                        "cla", "power_kw"], key="dc_chan")
        vals_txt = c[1].text_input("Values (comma-separated)",
                                   "0.40, 0.45, 0.50, 0.54, 0.55, 0.60, "
                                   "0.65, 0.70", key="dc_vals")
        if st.button("Evaluate the declared car", key="dc_go",
                     type="primary"):
            try:
                vals = _floats(vals_txt, ())
                with st.spinner("Solving the declared car…"):
                    summ = car.summary(v["lateral_g"])
                    sens = fc.lap_sensitivity(car, chan, vals)
                ss["fc_declared"] = {
                    "schema": "kinematik.declared_car/2",
                    "declaration": dict(v),
                    "front_hp": (gr.hp_to_dict(car.front_hp)
                                 if car.front_hp is not None else None),
                    "rear_hp": (gr.hp_to_dict(car.rear_hp)
                                if car.rear_hp is not None else None),
                    "sweep": {"channel": chan, "values": vals},
                    "summary": summ, "sensitivity": sens}
            except Exception as exc:          # noqa: BLE001
                st.error(f"Evaluation failed: {exc}")
        out = ss.get("fc_declared")
        if out:
            summ, sens = out["summary"], out["sensitivity"]
            st.markdown("**Vehicle summary**")
            _table(st, pd, [(k, json.dumps(x) if isinstance(x, dict) else x)
                            for k, x in summ.items()], ["quantity", "value"])
            if ("PROXY" in json.dumps(summ["motion_ratio"])
                    and summ["roll_stiffness_source"].startswith("spring")):
                st.warning("Spring-rate roll stiffness is running on a "
                           "PROXY motion ratio (no rocker defined): these "
                           "roll stiffnesses are provisional.")
            st.markdown("**Lap sensitivity** — baseline "
                        + str(round(sens["baseline_s"], 3)) + " s on a "
                        + str(round(sens["track_length_m"])) + " m layout; "
                        + sens["channel"] + " spread "
                        + str(round(sens["spread_s"], 3)) + " s, best at "
                        + str(sens["best_value"]) + ".")
            st.line_chart(pd.DataFrame(sens["rows"], columns=[
                sens["channel"], "lap time (s)"]).set_index(sens["channel"]))
            st.download_button("Declared car + results (.json)",
                               json.dumps(out, indent=2, default=float),
                               file_name="declared_car.json",
                               mime="application/json", key="dc_dl")
        st.markdown("**Tyre**")
        _sec_tyre(st, pd, v, "dc_tyre_loads")
        st.markdown("**Steering effort**")
        if corners["front"] is None:
            st.caption("Needs a front corner.")
        else:
            _sec_steering(st, pd, v, corners["front"],
                          corners["rear"] or Hardpoints.default(), "dc_steer")
        for axle in ("front", "rear"):
            st.markdown(f"**Actuation, ride and roll stiffness — {axle}**")
            if corners[axle] is None:
                st.caption("Needs a " + axle + " corner.")
            else:
                _sec_actuation(st, pd, v, corners[axle], axle)


def render():
    import numpy as np
    import pandas as pd
    import streamlit as st
    from suspension import units as _units
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

    _declared_car_panel(st, pd, np, ss, fc)

    # ================= 1 · the rule matrix ================================
    st.markdown("###### 1 · The rule matrix — the constraint bounds")
    r1, r2, r3 = st.columns(3)
    max_power = _units.unum(r1, "Max TS power (kW)", 20.0, 200.0, 80.0, 'kW', step=5.0, key="fc_pwr")
    max_ts_v = r2.number_input("Max TS voltage (VDC)", 100.0, 800.0, 600.0,
                               10.0, key="fc_tsv")
    endurance_km = _units.unum(r3, "Endurance distance (km)", 5.0, 30.0, 22.0, 'km', step=0.5, key="fc_endkm")
    r4, r5, r6 = st.columns(3)
    max_seg_v = r4.number_input("Max segment voltage (VDC)", 40.0, 200.0,
                                120.0, 5.0, key="fc_segv")
    max_seg_e = r5.number_input("Max segment energy (MJ)", 1.0, 12.0, 6.0,
                                0.5, key="fc_sege")
    cell_limit = _units.unum(r6, "Cell temp limit (°C)", 40.0, 80.0, 60.0, '°C', step=1.0, key="fc_tlim")
    min_wb = _units.unum(st, "Minimum wheelbase (mm)", 1400.0, 1800.0, 1525.0, 'mm', step=5.0, key="fc_wb")
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
        from ui.inverse_genesis import _veh
        vd = _veh(ss)
        f1, f3 = st.columns(2)
        base_mass = _units.unum(f1, "Base mass excl. cells & motors (kg)", 120.0, 320.0, 215.0, 'kg', step=5.0, key="fc_bm")
        ambient = _units.unum(f3, "Cooling-air inlet (°C)", 10.0, 45.0, 30.0, '°C', step=1.0, key="fc_amb")
        st.caption("Wheelbase, CG height, weight distribution, tracks, tyre "
                   "model and aero come from the vehicle & build declaration "
                   "at the top of this tab.")
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
        base_mass_kg=base_mass, wheelbase_mm=vd["wheelbase_mm"],
        ambient_c=ambient, cg_height_mm=vd["cg_height_mm"],
        weight_dist_front=vd["weight_dist_front"],
        track_mm=vd["track_front_mm"], track_rear_mm=vd["track_rear_mm"],
        cla=vd["cla"], cda=vd["cda"], tire_model=vd["tire_model"])

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

    if st.button("🧬🏁 Synthesize the full car", type="primary",
                 key="fc_go"):
        with st.spinner("Walking the design chain backwards…"):
            try:
                res = fc.synthesize_fullcar(space, rules, ref,
                                            n_finalists=int(n_final))
            except Exception as exc:                   # never kill the tab
                st.error(f"Synthesis failed: {exc}")
                return
        ss["fc_res"] = (res, space, rules)
    if ss.get("fc_res") is None:
        return
    res, space, rules = ss["fc_res"]

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
                     width="stretch")

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
                     width="stretch")

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
            k1, k2, k3 = st.columns(3)
            i_axle = k1.selectbox("Axle", ["front", "rear"], key="fc_i_axle")
            from ui.inverse_genesis import _veh
            _vd = _veh(ss)
            rgrad = float(_vd["body_roll_deg"]) / max(
                float(_vd["lateral_g"]), 1e-6)
            k2.caption("Roll gradient " + str(round(rgrad, 3))
                       + " deg/g — declared body roll ÷ design lateral g.")
            i_mode = k3.selectbox("Intent", ["derived", "declared"],
                                  key="fc_i_mode")
            decl = None
            bands = None
            if i_mode == "declared":
                d1, d2, d3, d4 = st.columns(4)
                decl = {"static_camber": d1.number_input(
                            "Static camber", -6.0, 3.0,
                            -1.5 if i_axle == "front" else -1.0, 0.05,
                            key=f"fc_i_g0_{i_axle}"),
                        "camber_gain": d2.number_input(
                            "Camber gain (deg/mm)", -0.5, 0.5,
                            -0.035 if i_axle == "front" else -0.028, 0.001,
                            format="%.4f", key=f"fc_i_gain_{i_axle}"),
                        "toe": 0.0,
                        "rc_height": d3.number_input(
                            "RC height (mm)", -200.0, 300.0,
                            55.0 if i_axle == "front" else 38.0, 0.5,
                            key=f"fc_i_rc_{i_axle}")}
                bands = {"camber_deg": 0.30, "toe_deg": 0.08,
                         "rc_height_mm": d4.number_input(
                             "RC band (mm)", 0.1, 200.0, 18.0, 0.5,
                             key="fc_i_rb")}
            try:
                hp_i, note_i = _corner_hp(ss, i_axle)
                st.caption(f"Seed geometry: {note_i}.")
                tg = fc.kinematic_intent_for(
                    res.winner, space, hp=hp_i,
                    roll_gradient_deg_per_g=rgrad, bands=bands,
                    declared=decl)
                irows = []
                for c in tg.curves:
                    for t, v, b in zip(c.travel_mm, c.target, c.band):
                        irows.append({"channel": c.channel,
                                      "travel (mm)": round(float(t), 1),
                                      "target": round(float(v), 3),
                                      "± band": round(float(b), 3)})
                st.dataframe(pd.DataFrame(irows), hide_index=True,
                             width="stretch")
                if st.button("Send this intent to the InverseGenesis tab",
                             key="fc_send_intent"):
                    ss["genesis_targets"] = {"targets": tg,
                                             "axle": i_axle}
                    st.success("Intent staged. Open the 🧬 InverseGenesis "
                               "tab, declare a legal volume, and generate.")
            except Exception as exc:
                st.info(f"Intent synthesis unavailable: {exc}")

        with tabs[1]:
            st.caption("The peak-cornering outer-wheel load resolved through "
                       "the linkage into per-member axial forces — the load "
                       "table to hand the frame/FEA seat.")
            try:
                hp_l, note_l = _corner_hp(ss, "front")
                st.caption(f"Geometry: {note_l}.")
                lc = fc.load_case_for(res.winner, space, hp=hp_l)
                st.markdown(f"Outer-tyre vertical load **{lc.fz_n:.0f} N** at "
                            f"**{lc.mu_lateral:.2f} g** lateral.")
                lrows = [{"member": k, "axial force (N, + tension)":
                          round(v, 0)} for k, v in lc.member_forces.items()]
                st.dataframe(pd.DataFrame(lrows), hide_index=True,
                             width="stretch")
                if lc.note:
                    st.caption("Note: " + lc.note)
            except Exception as exc:
                st.info(f"Load-case synthesis unavailable: {exc}")

            st.markdown("**Under load: what the pickups and the frame do to the wheel**")
            st.caption("The same peak-corner load, solved with the chassis "
                       "pickups on springs (node, bracket and bearing in series) "
                       "and applied in steps, re-resolving member forces on the "
                       "deflected geometry each time. Then the frame's own twist "
                       "between the rack and the tie-rod plane. Use measured "
                       "stiffness where you have it.")
            e1, e2, e3 = st.columns(3)
            fk = float(e1.number_input("Pickup stiffness (kN/mm)", 1.0, 500.0,
                                       30.0, 1.0, key="fc_ek_k"))
            fkt = float(e2.number_input("Frame torsional stiffness K_T (N·m/deg)",
                                        100.0, 50000.0, 2000.0, 100.0,
                                        key="fc_ek_kt"))
            fsep = float(e3.number_input("Rack to tie-rod plane (mm)", 0.0,
                                         600.0, 150.0, 5.0, key="fc_ek_sep"))
            try:
                from suspension import elastokinematics as _ek
                hp_e, _ = _corner_hp(ss, "front")
                r = fc.elastokinematic_check(
                    res.winner, space, hp=hp_e,
                    stiffness=_ek.StiffnessField(fk * 1000.0))
                ft = fc.frame_twist_toe_budget(res.winner, space, fkt, fsep, hp=hp_e)
                erows = [{"source": "pickup compliance", "camber (deg)":
                          round(r.change["camber"], 4), "toe (deg)":
                          round(r.change["toe"], 4), "caster (deg)":
                          round(r.change["caster"], 4), "KPI (deg)":
                          round(r.change["kpi"], 4)},
                         {"source": "frame twist, rack vs tie-rod plane",
                          "camber (deg)": None, "toe (deg)": round(ft["toe_deg"], 4),
                          "caster (deg)": None, "KPI (deg)": None}]
                st.dataframe(pd.DataFrame(erows), hide_index=True, width="stretch")
                from suspension.provenance import graded as _g
                _lim = f"{r.provenance} pickup stiffness"
                st.caption("Converged: " + str(r.converged) + " (iterations per "
                           "step " + str(r.iterations) + "); linear-correction "
                           "error " + _g(max(r.linearity_error.values()),
                                         "modelled", "deg", digits=2,
                                         limited_by=_lim)
                           + ". Frame twist " + _g(ft["twist_deg"], "modelled",
                                                   "deg", digits=3,
                                                   limited_by="declared K_T")
                           + " across the wheelbase under "
                           + _g(ft["torque_Nm"], "modelled", "N·m", digits=0,
                                limited_by="declared mass and CG") + ".")
            except Exception as exc:
                st.info(f"Elastokinematic check unavailable: {exc}")

        with tabs[2]:
            st.caption("The nominal corner geometry as a coordinate table — "
                       "the CAD/DXF tools' input, not STEP (KinematiK carries "
                       "no CAD kernel).")
            try:
                from suspension.kinematics import Hardpoints
                hp_c, note_c = _corner_hp(ss, "front")
                st.caption(f"Geometry: {note_c}.")
                csv = fc.export_hardpoints_csv(hp_c)
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
