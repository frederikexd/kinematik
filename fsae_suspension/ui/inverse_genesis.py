# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/inverse_genesis.py — the 🧬 InverseGenesis tab
#  (draw the curves · declare the legal volume · generate resilient geometry)
# ============================================================================
"""
The tab that runs the design loop backwards.

Three panels, three declarations, one generated geometry:

  1. DRAW THE CURVES — the kinematic intent, stated as curves over wheel
     travel with acceptance bands, seeded from the live nominal so the sheet
     solves before the first edit. Bend the numbers to the car you want.
  2. THE LEGAL VOLUME — which hardpoints the engine may move, how far, and
     the keep-out volumes (headers, mounts, bodywork) no candidate may
     touch. The boundary filter is a wall, not a penalty.
  3. THE CO-OPTIMIZER — declare the shop's error field (the Stochastic
     Inversion presets) and generate. The winner is the highest BUILD-YIELD
     geometry that hits the curves — the knife-edge best-fit loses on
     purpose, and the yield premium it forfeited is printed.

All physics lives in suspension/inverse_genesis.py; this module only
orchestrates the engine and draws (see ui/__init__.py rules).

Reproducibility: every run is captured as a GenesisManifest
(suspension/genesis_repro.py) — geometry at full precision, targets, boxes,
spacing constraints, keep-outs, tolerance field, seed, sample counts and
solver constants — downloadable with its recorded outputs, and re-runnable
from the top of the tab with a byte-for-byte verification.

Session keys used:
    hp               read: the Kinematics editor's geometry (dict)
    hardpoints       read/write: a Hardpoints object; "apply" writes the
                     generated corner back so every other tab consumes it
    genesis_targets  read: intent staged by the FullCar tab
    genesis_corners  write: {"front"/"rear": hardpoints dict} for FullCar
    ig_run           the last run (manifest + result) so the diagnostics
                     panels survive Streamlit reruns
    genesis_last     summary dict of the last run (for cross-tab reads)
"""

from __future__ import annotations

_AXES = ("x", "y", "z")

_SHOPS = {
    "Hand-welded tabs (±1.5 mm)": "hand_weld",
    "Jig-welded tabs (±0.5 mm)": "jig_weld",
    "CNC everything (±0.05 mm)": "cnc",
}

_CHANNEL_UI = {
    "camber_deg":   ("Camber vs travel (°)", "the gain curve"),
    "toe_deg":      ("Toe vs travel (°)", "bump steer, drawn whole"),
    "rc_height_mm": ("Roll-centre height vs travel (mm)", "RC migration"),
    "scrub_mm":     ("Scrub radius vs travel (mm)", "steering feel & wear"),
}

_VERDICT_BLURB = {
    "RESILIENT": ("🟢", "This geometry hits your drawn curves AND survives "
                        "your shop. The population of cars inside the "
                        "declared error field lands inside the bands at this "
                        "yield — generate the drawings."),
    "TEMPERED":  ("🟡", "Hits the curves, but roughly one build in five "
                        "drifts outside a band. The candidate table shows "
                        "what a point of yield costs in fit — or jig the "
                        "dominant tab and rerun."),
    "KNIFE_EDGE": ("🔴", "Every curve-hitting geometry is a knife edge under "
                         "your declared field. The bands, the legal volume "
                         "and the shop are jointly unsatisfiable — the "
                         "engine prices the gamble instead of recommending "
                         "it. Jig a tab, widen a band, or free a "
                         "coordinate."),
    "NO_FIT":    ("⚪", "No legal geometry reaches the drawn curves; the "
                        "closest legal approach and its binding constraint "
                        "are named below."),
}


def _hardpoints_from_session(ss):
    """The live hardpoint set, else the KinematiK default corner.

    The Kinematics editor stores its geometry under ``hp`` (a dict of lists);
    an applied InverseGenesis result lives under ``hardpoints`` (an object).
    Both are read — reading only ``hardpoints`` silently ran every tab
    session on the default corner.
    """
    import numpy as np
    from suspension.kinematics import Hardpoints
    raw = ss.get("hardpoints")
    if isinstance(raw, Hardpoints):
        return raw, "applied InverseGenesis geometry"
    for key, note in (("hardpoints", "live hardpoints"),
                      ("hp", "live hardpoints from the Kinematics tab")):
        raw = ss.get(key)
        if isinstance(raw, dict) and raw:
            try:
                kw = {k: (np.asarray(v, float) if isinstance(v, (list, tuple))
                          else v) for k, v in raw.items()
                      if k in Hardpoints.__dataclass_fields__}
                return Hardpoints(**kw), note
            except Exception:           # noqa: BLE001
                pass
    return Hardpoints.default(), \
        "default FSAE front corner (no live hardpoints set)"


_POINT_ROWS = ("upper_front_inner", "upper_rear_inner", "lower_front_inner",
               "lower_rear_inner", "tie_rod_inner", "upper_outer",
               "lower_outer", "tie_rod_outer", "wheel_center",
               "contact_patch")


def _parse_hardpoints_text(text: str, base):
    """JSON (manifest-style dict) or CSV ``point,x,y,z`` → Hardpoints."""
    import io
    import json
    import numpy as np
    import pandas as pd
    from suspension import genesis_repro as gr
    text = text.strip()
    if text.startswith("{"):
        d = json.loads(text)
        d = d.get("hardpoints", d)
        merged = gr.hp_to_dict(base)
        merged.update(d)
        return gr.hp_from_dict(merged)
    df = pd.read_csv(io.StringIO(text))
    cols = {c.lower().strip(): c for c in df.columns}
    name_col = cols.get("point") or cols.get("hardpoint") or df.columns[0]
    merged = gr.hp_to_dict(base)
    for _, row in df.iterrows():
        n = str(row[name_col]).strip()
        if n in merged:
            merged[n] = [float(row[cols[a]]) for a in ("x", "y", "z")]
    return gr.hp_from_dict(merged)


def _results_panel(st, pd, np, ss, run):
    """Everything shown after a run; reads only ``ss['ig_run']``."""
    from suspension import inverse_genesis as ig
    from suspension import genesis_repro as gr

    man = gr.GenesisManifest.from_json(run["manifest_json"])
    res = run["result"]
    hp, targets, volume, fld = man.objects()
    travel = float(max(abs(t) for t in targets.stations()))

    if run.get("verify") is not None:
        same, diffs = run["verify"]
        if same:
            st.success("✅ Re-run reproduces the recorded outputs exactly "
                       "(every candidate's coordinates, fit and yield).")
        else:
            st.error("❌ Re-run differs from the recorded outputs.")
        for d in diffs:
            st.caption(d)

    if res.winner is None:
        st.markdown("## ⚪ NO LEGAL GEOMETRY REACHES THE CURVES")
        st.error(res.reason)
        if res.best_fit is not None:
            st.caption(f"Closest legal approach: "
                       f"{res.best_fit.max_band_frac:.2f}× band, governed "
                       f"by {res.best_fit.worst_row}. This is an upper bound "
                       "on the true minimax distance, not the distance.")
    else:
        w = res.winner
        badge, blurb = _VERDICT_BLURB.get(w.verdict, ("", ""))
        ytxt = f" — build yield {w.yield_frac*100:.1f}%" \
            if w.yield_frac is not None else ""
        st.markdown(f"## {badge} {w.verdict}{ytxt}")
        st.caption(blurb)
        (st.success if res.ok else st.warning)(res.reason)
        m1, m2, m3, m4 = st.columns(4)
        if w.yield_frac is not None:
            m1.metric("Build yield", f"{w.yield_frac*100:.1f} %")
        m2.metric("Worst station", f"{w.max_band_frac:.2f}× band",
                  delta=w.worst_row, delta_color="off")
        m3.metric("Iterations", f"{w.iterations}")
        if res.resilience_premium is not None:
            m4.metric("Resilience premium",
                      f"{res.resilience_premium*100:+.1f} pts")

    st.caption(f"Manifest `{man.name}` · inputs sha256 "
               f"`{man.inputs_sha256[:16]}…` · seed {man.search.seed} · "
               f"{man.search.n_starts} starts · N = {man.search.n_yield} · "
               f"KinematiK {man.kinematik_version}")

    # ---- candidate field -------------------------------------------------- #
    st.markdown("###### Candidate field")
    st.dataframe(pd.DataFrame(
        [{"#": i + 1, "verdict": c.verdict, "hit": c.hit,
          "worst station (×band)": round(c.max_band_frac, 3),
          "build yield": (f"{c.yield_frac*100:.1f}%"
                          if c.yield_frac is not None else "—"),
          "governed by": c.worst_row, "iterations": c.iterations,
          "clamped to box face": ", ".join(c.clamped) or "—",
          "refused steps": c.keepout_rejections}
         for i, c in enumerate(res.candidates)]),
        hide_index=True, width="stretch")

    if res.winner_hp is None:
        return
    whp = res.winner_hp

    # ---- the deliverable, at full precision ------------------------------- #
    st.markdown("###### The generated hardpoints — full precision (the "
                "deliverable)")
    whd = gr.hp_to_dict(whp)
    st.dataframe(pd.DataFrame(
        [{"point": p, "x": whd[p][0], "y": whd[p][1], "z": whd[p][2],
          "designed": p in volume.boxes} for p in _POINT_ROWS]),
        hide_index=True, width="stretch",
        column_config={a: st.column_config.NumberColumn(format="%.6f")
                       for a in "xyz"})
    csv = "point,x,y,z\n" + "\n".join(
        f"{p},{whd[p][0]!r},{whd[p][1]!r},{whd[p][2]!r}" for p in _POINT_ROWS)
    d1, d2, d3 = st.columns(3)
    d1.download_button("Manifest + outputs (.json)", run["manifest_json"],
                       file_name=f"{man.name}.genesis.json",
                       mime="application/json", key="ig_dl_manifest")
    d2.download_button("Hardpoints (.csv, full precision)", csv,
                       file_name=f"{man.name}_hardpoints.csv",
                       mime="text/csv", key="ig_dl_csv")
    d3.download_button("Report (.md)", ig.render_genesis_md(res, targets),
                       file_name=f"{man.name}.md", mime="text/markdown",
                       key="ig_dl_md")

    # curve overlay
    dense = np.linspace(-travel, travel, 41)
    gen_vals, gen_ok = ig.curves_of(whp, dense, track_mm=targets.track_mm)
    if gen_ok:
        with st.expander("Curves against their bands"):
            for c in targets.curves:
                st.markdown(f"**{_CHANNEL_UI[c.channel][0]}**")
                st.line_chart(pd.DataFrame({
                    "band lo": np.interp(dense, c.travel_mm,
                                         c.target - c.band),
                    "band hi": np.interp(dense, c.travel_mm,
                                         c.target + c.band),
                    "generated": gen_vals[c.channel],
                }, index=pd.Index(dense.round(1), name="travel (mm)")),
                    height=200)

    # ---- properties the channels do not see ------------------------------ #
    with st.expander("🔎 Properties outside the objective (caster, KPI, "
                     "scrub, RC migration, anti-dive/-squat)",
                     expanded=True):
        a1, a2, a3, a4 = st.columns(4)
        axle = a1.selectbox("Axle", ["front", "rear"],
                            index=0 if run.get("axle", "front") == "front"
                            else 1, key="ig_diag_axle")
        wb = a2.number_input("Wheelbase (mm)", 1000.0, 3000.0, 1630.0, 5.0,
                             key="ig_diag_wb")
        cgh = a3.number_input("CG height (mm)", 100.0, 600.0, 280.0, 5.0,
                              key="ig_diag_cg")
        bias = a4.number_input("Front brake bias", 0.0, 1.0, 0.60, 0.01,
                               key="ig_diag_bias")
        dg = gr.corner_diagnostics(whp, stations=targets.stations(),
                                   track_mm=targets.track_mm, axle=axle,
                                   wheelbase_mm=wb, cg_height_mm=cgh,
                                   brake_bias_front=bias)
        if dg.get("ok"):
            rows = [
                ("Camber gain (LSQ slope)", dg["camber_gain_deg_per_mm"],
                 "deg/mm"),
                ("Bump steer (LSQ slope)", dg["bump_steer_deg_per_mm"],
                 "deg/mm"),
                ("Toe change (max − min)", dg["toe_change_deg"], "deg"),
                ("RC height, static", dg["rc_height_static_mm"], "mm"),
                ("RC migration, chassis frame",
                 dg["rc_migration_chassis_mm_per_mm"], "mm/mm"),
                ("RC migration, above ground",
                 dg["rc_migration_ground_mm_per_mm"], "mm/mm"),
                ("RC above ground, minimum", dg["rc_above_ground_min_mm"],
                 "mm"),
                ("Caster", dg["caster_deg"], "deg"),
                ("Kingpin inclination", dg["kpi_deg"], "deg"),
                ("Scrub radius", dg["scrub_static_mm"], "mm"),
                ("Contact-patch rise per mm of travel",
                 dg["contact_patch_rise_per_mm"], "mm/mm"),
                ("Side-view IC, x rearward", dg["side_view_ic_x_mm"], "mm"),
                ("Side-view IC, height", dg["side_view_ic_z_mm"], "mm"),
                (f"tan swing-arm ({dg['side_view_reference']})",
                 dg["side_view_tan"], "—"),
            ]
            if "anti_dive_pct" in dg:
                rows.append(("Anti-dive", dg["anti_dive_pct"], "%"))
            if "anti_squat_pct" in dg:
                rows.append(("Anti-squat", dg["anti_squat_pct"], "%"))
            st.dataframe(pd.DataFrame(rows, columns=["quantity", "value",
                                                     "unit"]),
                         hide_index=True, width="stretch",
                         column_config={"value": st.column_config.NumberColumn(
                             format="%.4f")})
            st.caption("None of these is a channel. The solver is "
                       "indifferent to all of them — check them on every "
                       "run.")
            ss.setdefault("genesis_corners", {})[axle] = gr.hp_to_dict(whp)

        st.markdown("**Camber to the road under roll**")
        c1, c2 = st.columns(2)
        roll = c1.number_input("Body roll (deg)", 0.0, 5.0, 1.18, 0.01,
                               key="ig_roll")
        opt = c2.number_input("Tyre optimum camber (deg)", -5.0, 0.0,
                              -1.83, 0.01, key="ig_opt")
        if dg.get("ok"):
            ctr = gr.camber_to_road(whp.static_camber,
                                    dg["camber_gain_deg_per_mm"], roll,
                                    targets.track_mm, opt)
            st.dataframe(pd.DataFrame(
                [(k, float(v)) for k, v in ctr.items()],
                columns=["quantity", "value"]), hide_index=True,
                width="stretch")

    # ---- yield, taken apart ---------------------------------------------- #
    if fld is not None and fld.specs:
        with st.expander("📊 Yield taken apart — per channel, worst case, "
                         "paired comparison"):
            n = man.search.n_yield
            yb = gr.yield_breakdown(whp, targets, fld, n=n,
                                    seed=man.search.seed,
                                    step_mm=man.search.step_mm)
            if yb.get("ok"):
                st.dataframe(pd.DataFrame(
                    [{"channel": k, "builds failing (%)": 100 * v}
                     for k, v in yb["per_channel_fail_frac"].items()]
                    + [{"channel": "ALL channels pass (%)",
                        "builds failing (%)": 100 * yb["yield"]}]),
                    hide_index=True, width="stretch")
                st.markdown(
                    f"First-order worst case **W = {yb['worst_case_W']:.3f}** "
                    f"({yb['worst_case_row']}) — "
                    + ("every build in the box passes to first order."
                       if yb["guaranteed_first_order"] else
                       "W > 1: corner builds of the tolerance box can fail "
                       "even if no sample did.")
                    + f" Governing row {yb['governing_row']} has "
                    f"{yb['governing_headroom_sigma']:.2f} σ of headroom.")
                if "yield_lower_bound_95" in yb:
                    st.caption(f"Zero failures in {n}: yield ≥ "
                               f"{100*yb['yield_lower_bound_95']:.2f}% at "
                               "95% (rule of three).")
                else:
                    st.caption(f"Standard error ≈ {100*yb['yield_se']:.2f} "
                               "points.")
            if res.best_fit is not None and res.best_fit is not res.winner:
                pa = gr.pass_vector(whp, targets, fld, n, man.search.seed,
                                    man.search.step_mm)
                pb = gr.pass_vector(
                    ig._shifted(hp, volume, res.best_fit.shift_vec), targets,
                    fld, n, man.search.seed, man.search.step_mm)
                if pa is not None and pb is not None:
                    dcb = gr.discordant_builds(pa, pb)
                    st.markdown(
                        f"Winner vs closest fit: **{dcb['discordant']}** "
                        f"discordant builds ({dcb['a_only']} pass only the "
                        f"winner, {dcb['b_only']} only the fit); exact "
                        f"two-sided p = {dcb['p_two_sided']:.3g}.")
            if st.button("Rounding sensitivity (±0.05 mm cell)",
                         key="ig_round"):
                rs = gr.rounding_sensitivity(
                    whp, targets, fld, list(volume.boxes), n=min(n, 2000),
                    seed=man.search.seed)
                if rs.get("ok"):
                    st.info(f"Yield within the 0.1 mm rounding cell: "
                            f"{100*rs['min']:.1f}% – {100*rs['max']:.1f}% "
                            f"over {rs['n_trials']} trials. Publish "
                            "coordinates at full precision.")

    # ---- swept volume ------------------------------------------------------ #
    obstacles = [o for o in volume.keep_out
                 if isinstance(o, gr.CapsuleObstacle)]
    frame = ss.get("tf_frame")
    with st.expander("🧱 Swept-volume clearance against the frame"):
        if not obstacles and frame:
            s1, s2 = st.columns(2)
            za = s1.number_input("Axle station, CAD Z (mm)", -5000.0, 5000.0,
                                 float(run.get("axle_station", 950.0)), 1.0,
                                 key="ig_sw_za")
            yg = s2.number_input("Ground plane, CAD Y (mm)", -1000.0, 1000.0,
                                 float(run.get("ground_y", -50.0)), 0.5,
                                 key="ig_sw_yg")
            obstacles = [gr.capsules_from_framegraph(frame, za, yg)]
        if not obstacles:
            st.caption("Load a frame in the Frame Planner (or declare frame "
                       "capsules as keep-outs) to run this check.")
        else:
            e1, e2, e3 = st.columns(3)
            nst = e1.number_input("Travel stations", 3, 101, 21, 2,
                                  key="ig_sw_n")
            excl = e2.number_input("Inboard exclusion (mm)", 0.0, 300.0,
                                   70.0, 5.0, key="ig_sw_ex")
            lrad = e3.number_input("Link radius (mm)", 0.0, 30.0, 7.94,
                                   0.01, key="ig_sw_lr")
            if st.button("Run swept-volume check", key="ig_sw_go"):
                sw = gr.swept_clearance(whp, obstacles[0],
                                        travel_mm=travel,
                                        n_stations=int(nst),
                                        link_radius_mm=lrad,
                                        exclusion_mm=excl)
                if not sw.get("ok"):
                    st.error(sw.get("reason", "check failed"))
                else:
                    (st.success if sw["min_clearance_mm"] >= 0
                     else st.error)(
                        f"Minimum clearance {sw['min_clearance_mm']:+.1f} mm "
                        f"— {sw['link']} vs {sw['tube']} at "
                        f"{sw['travel_mm']:+.1f} mm travel.")
                    st.dataframe(pd.DataFrame(
                        [{"link": k, **v} for k, v in sw["per_link"].items()]),
                        hide_index=True, width="stretch")

    for wmsg in res.warnings:
        st.warning(wmsg)
    if res.ok and st.button("Apply the generated geometry to the live "
                            "hardpoints", key="ig_apply"):
        ss["hardpoints"] = whp
        st.success("Applied — every tab now consumes the generated corner.")


def render():
    import json
    import numpy as np
    import pandas as pd
    import streamlit as st
    from suspension import units as _units
    from suspension import inverse_genesis as ig
    from suspension import kinematik_stochastic as ks
    from suspension import genesis_repro as gr

    ss = st.session_state

    st.subheader("🧬 InverseGenesis — draw the curves; the engine generates "
                 "the geometry")
    st.caption(
        "Draw the kinematic curves you want inside acceptance bands, box the "
        "legal volume each hardpoint may occupy, declare the shop, and the "
        "engine pulls the coordinates into the curves — then ranks the "
        "candidates by BUILD YIELD. Every run is saved as a manifest you can "
        "download and re-run byte for byte. Rigid kinematics only.")

    # ================= 0 · re-run a manifest ==============================
    with st.expander("📂 Re-run a saved manifest (reproduce a result)"):
        up = st.file_uploader("Genesis manifest (.json)", type=["json"],
                              key="ig_manifest_up")
        if up is not None:
            try:
                man = gr.GenesisManifest.from_json(up.getvalue().decode())
                st.caption(f"`{man.name}` · sha256 "
                           f"`{man.inputs_sha256[:16]}…` · recorded on "
                           f"KinematiK {man.kinematik_version} · "
                           + ("has recorded outputs" if man.recorded
                              else "no recorded outputs"))
                if man.context:
                    st.json(man.context, expanded=False)
                if st.button("▶ Run this manifest", key="ig_manifest_run",
                             type="primary"):
                    with st.spinner("Re-running the manifest…"):
                        res = man.run()
                    ver = man.verify(res) if man.recorded else None
                    if not man.recorded:
                        man.record(res)
                    ss["ig_run"] = {"manifest_json": man.to_json(),
                                    "result": res, "verify": ver,
                                    **{k: man.context.get(k) for k in
                                       ("axle", "axle_station", "ground_y")
                                       if man.context.get(k) is not None}}
            except ValueError as e:
                st.error(f"Manifest refused: {e}")

    # ================= geometry ===========================================
    st.markdown("###### Geometry — the seed the engine moves from")
    live_hp, live_note = _hardpoints_from_session(ss)
    src = st.radio("Source", ["Live (Kinematics tab)",
                              "KinematiK default corner",
                              "Paste hardpoints (corner frame)",
                              "CAD coordinates (SolidWorks frame)"],
                   horizontal=True, key="ig_src")
    from suspension.kinematics import Hardpoints
    axle_station = ground_y = None
    axle = st.radio("Axle", ["front", "rear"], horizontal=True,
                    key="ig_axle")
    if src.startswith("Live"):
        hp = live_hp
        st.caption(f"Geometry: {live_note}.")
    elif src.startswith("KinematiK default"):
        hp = Hardpoints.default()
    elif src.startswith("Paste"):
        txt = st.text_area(
            "JSON (``{\"upper_front_inner\": [x, y, z], …}``) or CSV "
            "(``point,x,y,z``) — corner frame, mm, full precision. Points "
            "not listed keep the default.", height=160, key="ig_hp_txt")
        hp = Hardpoints.default()
        if txt.strip():
            try:
                hp = _parse_hardpoints_text(txt, hp)
            except Exception as e:          # noqa: BLE001
                st.error(f"Could not read the hardpoints: {e}")
                return
    else:
        g1, g2 = st.columns(2)
        axle_station = g1.number_input("Axle station, CAD Z (mm)", -5000.0,
                                       5000.0, 950.0, 1.0, key="ig_za")
        ground_y = g2.number_input("Ground plane, CAD Y (mm)", -1000.0,
                                   1000.0, -50.0, 0.5, key="ig_yg")
        base = gr.hp_to_dict(Hardpoints.default())
        seed_cad = pd.DataFrame(
            [{"point": p, **dict(zip(("X", "Y", "Z"),
                                     gr.corner_to_cad(base[p], axle_station,
                                                      ground_y).tolist()))}
             for p in _POINT_ROWS])
        cad = st.data_editor(seed_cad, key="ig_cad_pts", hide_index=True,
                             column_config={"point": st.column_config
                                            .TextColumn(disabled=True)})
        st.caption("x = −(Z − Zₐ), y = X, z = Y − Y_g. The two frames have "
                   "opposite handedness; X must be the vehicle's RIGHT.")
        d = dict(base)
        for _, r in cad.iterrows():
            d[r["point"]] = gr.cad_to_corner(
                [r["X"], r["Y"], r["Z"]], axle_station, ground_y).tolist()
        hp = gr.hp_from_dict(d)

    hp = hp.copy()          # never mutate the live session object
    a1, a2 = st.columns(2)
    hp.static_camber = float(a1.number_input(
        "Static camber γ₀ (deg) — declared input", -6.0, 3.0,
        float(hp.static_camber), 0.05, key="ig_gamma0"))
    hp.static_toe = float(a2.number_input(
        "Static toe δ₀ (deg) — declared input", -3.0, 3.0,
        float(hp.static_toe), 0.01, key="ig_delta0"))

    # ================= 1 · targets ========================================
    st.markdown("###### 1 · Target curves")
    c1, c2, c3 = st.columns(3)
    travel = float(c1.number_input("Travel range (± mm)", 2.0, 80.0, 25.0,
                                   0.5, key="ig_travel"))
    n_st = int(c2.number_input("Stations", 2, 41, 5, 1, key="ig_nst"))
    track = float(c3.number_input("Track for RC construction (mm)", 500.0,
                                  2500.0, 1210.0, 5.0, key="ig_track"))
    stations = np.linspace(-travel, travel, n_st)

    nom_vals, nom_ok = ig.curves_of(hp, stations, track_mm=track)
    if not nom_ok:
        st.error("The seed geometry does not solve over this travel range.")
        return

    staged = ss.get("genesis_targets")
    modes = ["Formula (paper style)", "Per-station table"]
    if staged is not None:
        modes.append("Staged from FullCar")
    tmode = st.radio("Targets as", modes, horizontal=True, key="ig_tmode")

    if tmode.startswith("Staged"):
        targets = staged["targets"] if isinstance(staged, dict) else staged
        if isinstance(staged, dict) and staged.get("axle"):
            st.caption(f"Intent staged for the {staged['axle']} axle.")
        stations = targets.stations()
        st.dataframe(pd.DataFrame(
            [{"channel": c.channel, "travel": t, "target": v, "band": b}
             for c in targets.curves
             for t, v, b in zip(c.travel_mm, c.target, c.band)]),
            hide_index=True, width="stretch")
    elif tmode.startswith("Formula"):
        rows = []
        f1, f2, f3, f4 = st.columns(4)
        use_c = f1.checkbox("Camber", True, key="ig_f_c")
        gain = f2.number_input("gain (deg/mm)", -0.5, 0.5, -0.035, 0.001,
                               format="%.4f", key="ig_f_gain")
        cband = f3.number_input("± band (deg)", 0.001, 5.0, 0.30, 0.01,
                                key="ig_f_cb")
        f4.caption(f"target = γ₀ + gain·t, γ₀ = {hp.static_camber:+.2f}°")
        g1, g2, g3, _ = st.columns(4)
        use_t = g1.checkbox("Toe", True, key="ig_f_t")
        toe = g2.number_input("toe (deg)", -2.0, 2.0, 0.0, 0.01,
                              key="ig_f_toe")
        tband = g3.number_input("± band (deg) ", 0.001, 5.0, 0.08, 0.01,
                                key="ig_f_tb")
        h1, h2, h3, _ = st.columns(4)
        use_r = h1.checkbox("RC height", True, key="ig_f_r")
        rc = h2.number_input("RC height (mm)", -200.0, 300.0, 55.0, 0.5,
                             key="ig_f_rc")
        rband = h3.number_input("± band (mm)", 0.01, 200.0, 18.0, 0.5,
                                key="ig_f_rb")
        k1, k2, k3, _ = st.columns(4)
        use_s = k1.checkbox("Scrub", False, key="ig_f_s")
        scrub = k2.number_input("scrub (mm)", -100.0, 100.0, 5.0, 0.5,
                                key="ig_f_scrub")
        sband = k3.number_input("± band (mm) ", 0.01, 100.0, 3.0, 0.5,
                                key="ig_f_sb")
        try:
            targets = gr.linear_targets(
                stations, static_camber=hp.static_camber,
                camber_gain=gain if use_c else None, camber_band=cband,
                toe=toe if use_t else None, toe_band=tband,
                rc_height=rc if use_r else None, rc_band=rband,
                scrub=scrub if use_s else None, scrub_band=sband,
                track_mm=track)
        except ValueError as e:
            st.error(str(e))
            return
    else:
        default_band = {"camber_deg": 0.20, "toe_deg": 0.08,
                        "rc_height_mm": 6.0, "scrub_mm": 3.0}
        enabled = st.multiselect(
            "Channels to anchor", list(ig.CHANNELS),
            default=["camber_deg", "toe_deg", "rc_height_mm"],
            format_func=lambda ch: _CHANNEL_UI[ch][0], key="ig_channels")
        if not enabled:
            st.info("Anchor at least one channel.")
            return
        curves = []
        for ch in enabled:
            label, hint = _CHANNEL_UI[ch]
            st.markdown(f"**{label}** — {hint}")
            edited = st.data_editor(pd.DataFrame({
                "travel_mm": stations,
                "target": np.asarray(nom_vals[ch], float).round(4),
                "band": np.full(n_st, default_band[ch])}),
                key=f"ig_curve_{ch}_{n_st}_{travel}", hide_index=True,
                column_config={"travel_mm": st.column_config.NumberColumn(
                    "travel (mm)", disabled=True)})
            band = np.maximum(np.abs(edited["band"].to_numpy(float)), 1e-3)
            try:
                curves.append(ig.TargetCurve(
                    ch, stations, edited["target"].to_numpy(float), band))
            except ValueError as e:
                st.error(f"{label}: {e}")
                return
        targets = ig.GenesisTargets(curves=curves, track_mm=track)

    # ================= 2 · legal volume ===================================
    st.markdown("###### 2 · The legal volume")
    movable = st.multiselect(
        "Hardpoints the engine may move", list(ig.DESIGNABLE_POINTS),
        default=["upper_front_inner", "upper_rear_inner",
                 "lower_front_inner", "lower_rear_inner", "tie_rod_inner"],
        key="ig_movable")
    if not movable:
        st.info("Free at least one hardpoint.")
        return
    bmode = st.radio("Boxes as", ["± half-width per axis about the seed",
                                  "Absolute bounds (corner frame)"],
                     horizontal=True, key="ig_bmode")
    whd = gr.hp_to_dict(hp)
    if bmode.startswith("±"):
        hdf = st.data_editor(pd.DataFrame(
            [{"point": p, "hx": 40.0, "hy": 40.0, "hz": 40.0,
              "free x": False, "free y": False, "free z": False}
             for p in movable]), key=f"ig_half_{'_'.join(movable)}",
            hide_index=True,
            column_config={"point": st.column_config.TextColumn(
                disabled=True)})
        half = {}
        for _, r in hdf.iterrows():
            h = [np.inf if r[f"free {a}"] else abs(float(r[f"h{a}"]))
                 for a in "xyz"]
            half[r["point"]] = h
        boxes = gr.boxes_about(hp, half)
    else:
        adf = st.data_editor(pd.DataFrame(
            [{"point": p,
              "x_lo": whd[p][0] - 40, "x_hi": whd[p][0] + 40,
              "y_lo": whd[p][1] - 40, "y_hi": whd[p][1] + 40,
              "z_lo": whd[p][2] - 40, "z_hi": whd[p][2] + 40}
             for p in movable]), key=f"ig_abs_{'_'.join(movable)}",
            hide_index=True,
            column_config={"point": st.column_config.TextColumn(
                disabled=True)})
        boxes = {r["point"]: (np.array([r["x_lo"], r["y_lo"], r["z_lo"]],
                                       float),
                              np.array([r["x_hi"], r["y_hi"], r["z_hi"]],
                                       float))
                 for _, r in adf.iterrows()}
        outside = [p for p, (lo, hi) in boxes.items()
                   if np.any(np.asarray(whd[p]) < lo)
                   or np.any(np.asarray(whd[p]) > hi)]
        if outside:
            st.warning("Seed lies outside its box for: "
                       + ", ".join(outside)
                       + " — the first step clamps it onto the box.")

    with st.expander("Spacing constraints (wishbone base, fore/aft "
                     "ordering)"):
        st.caption("Each row requires b.axis − a.axis ≥ gap (x is "
                   "rearward), or |b − a| ≥ gap with axis = dist. "
                   "Example: lower_front_inner → lower_rear_inner, x, 150.")
        sdf = st.data_editor(
            pd.DataFrame(columns=["a", "b", "axis", "min_gap_mm"]),
            num_rows="dynamic", key="ig_spacing", hide_index=True,
            column_config={
                "a": st.column_config.SelectboxColumn(
                    options=list(ig.PERTURBABLE_OR_FIXED)),
                "b": st.column_config.SelectboxColumn(
                    options=list(ig.PERTURBABLE_OR_FIXED)),
                "axis": st.column_config.SelectboxColumn(
                    options=["x", "y", "z", "dist"]),
                "min_gap_mm": st.column_config.NumberColumn()})
    spacings = []
    for _, r in sdf.iterrows():
        if not r.get("a") or not r.get("b"):
            continue
        try:
            spacings.append(ig.PointSpacing(
                str(r["a"]), str(r["b"]), str(r.get("axis") or "x"),
                float(r.get("min_gap_mm") or 0.0)))
        except (ValueError, TypeError) as e:
            st.error(f"Spacing row skipped: {e}")

    with st.expander("Keep-out volumes"):
        ko_df = st.data_editor(
            pd.DataFrame(columns=["label", "x_lo", "y_lo", "z_lo",
                                  "x_hi", "y_hi", "z_hi"]),
            key="ig_keepout", num_rows="dynamic", hide_index=True)
        use_frame = False
        if ss.get("tf_frame"):
            use_frame = st.checkbox(
                "Add the Frame Planner frame as capsule keep-outs",
                key="ig_use_frame")
            if use_frame and axle_station is None:
                f1, f2 = st.columns(2)
                axle_station = f1.number_input(
                    "Axle station, CAD Z (mm) ", -5000.0, 5000.0, 950.0, 1.0,
                    key="ig_ko_za")
                ground_y = f2.number_input(
                    "Ground plane, CAD Y (mm) ", -1000.0, 1000.0, -50.0, 0.5,
                    key="ig_ko_yg")
        k1, k2 = st.columns(2)
        probe = _units.unum(k1, "Probe radius (mm)", 0.0, 30.0, 6.0, 'mm',
                            step=1.0, key="ig_probe")
        min_cl = _units.unum(k2, "Required clearance (mm)", 0.0, 20.0, 2.0,
                             'mm', step=0.5, key="ig_mincl")
    keep_out = []
    for _, row in ko_df.iterrows():
        try:
            keep_out.append(ig.KeepOutBox(
                np.array([row["x_lo"], row["y_lo"], row["z_lo"]], float),
                np.array([row["x_hi"], row["y_hi"], row["z_hi"]], float),
                label=str(row.get("label") or "keep-out box")))
        except (ValueError, TypeError) as e:
            st.error(f"Keep-out row skipped: {e}")
    if use_frame:
        keep_out.append(gr.capsules_from_framegraph(
            ss["tf_frame"], axle_station, ground_y))

    try:
        volume = ig.LegalVolume(boxes=boxes, keep_out=keep_out,
                                probe_radius_mm=float(probe),
                                min_clearance_mm=float(min_cl),
                                spacings=spacings)
    except ValueError as e:
        st.error(f"Legal volume refused: {e}")
        return

    # ================= 3 · shop & search ==================================
    st.markdown("###### 3 · The shop and the search")
    s1, s2, s3 = st.columns([2, 1, 1])
    shop_label = s1.selectbox("Shop class", list(_SHOPS.keys()),
                              index=1, key="ig_shop")
    pull = _units.unum(s2, "Weld pull (mm)", 0.0, 3.0, 0.0, 'mm', step=0.1,
                       key="ig_pull")
    pull_axis = s3.selectbox("Pull axis", _AXES, index=2, key="ig_pull_axis")
    with st.expander("Per-point tolerance overrides"):
        st.caption("E.g. a welded rear toe-link tab: tie_rod_inner at the "
                   "tab tolerance instead of the machined class.")
        odf = st.data_editor(pd.DataFrame(columns=["point", "± mm"]),
                             num_rows="dynamic", key="ig_tol_over",
                             hide_index=True,
                             column_config={"point": st.column_config
                                            .SelectboxColumn(options=list(
                                                ks.PERTURBABLE_POINTS))})
    overrides = {str(r["point"]): float(r["± mm"])
                 for _, r in odf.iterrows()
                 if r.get("point") and r.get("± mm") is not None
                 and not pd.isna(r.get("± mm"))}
    fld = gr.field_with_overrides(_SHOPS[shop_label], overrides,
                                  weld_pull_mm=float(pull),
                                  pull_axis=pull_axis)

    r1, r2, r3, r4 = st.columns(4)
    seed = int(r1.number_input("Seed s", 0, 2**31 - 1, 0, 1, key="ig_seed"))
    n_starts = int(r2.number_input("Starts", 1, 64, 6, 1, key="ig_nstarts"))
    n_yield = int(r3.number_input("Sampled builds N", 100, 50000, 4000, 100,
                                  key="ig_nyield"))
    n_verify = int(r4.number_input("Full-solve verification", 0, 2000, 0,
                                   10, key="ig_nverify"))
    with st.expander("Solver constants"):
        q1, q2, q3, q4, q5 = st.columns(5)
        max_iter = int(q1.number_input("Iteration cap", 1, 500, 30, 1,
                                       key="ig_maxit"))
        step = float(q2.number_input("Jacobian step h (mm)", 0.01, 5.0,
                                     0.25, 0.01, key="ig_step"))
        th_r = float(q3.number_input("RESILIENT ≥", 0.0, 1.0, 0.95, 0.01,
                                     key="ig_thr"))
        th_t = float(q4.number_input("TEMPERED ≥", 0.0, 1.0, 0.80, 0.01,
                                     key="ig_tht"))
        th_v = float(q5.number_input("Linearisation floor", 0.0, 1.0, 0.98,
                                     0.01, key="ig_thv"))
    name = st.text_input("Run name", f"{axle}_corner", key="ig_name")

    if st.button("🧬 Generate the geometry", key="ig_run_btn",
                 type="primary"):
        search = gr.SearchSettings(seed=seed, n_starts=n_starts,
                                   n_yield=n_yield, n_verify_full=n_verify,
                                   max_iter=max_iter, step_mm=step,
                                   resilient_yield=th_r, tempered_yield=th_t,
                                   verify_agreement=th_v)
        ctx = {"axle": axle, "geometry_source": src}
        if axle_station is not None:
            ctx.update(axle_station=axle_station, ground_y=ground_y)
        if overrides:
            ctx["tolerance_overrides"] = overrides
        try:
            man = gr.GenesisManifest.build(name, hp, targets, volume, fld,
                                           search, ctx)
        except TypeError as e:
            st.error(f"Cannot record this run: {e}")
            return
        with st.spinner("Reverse gradients pulling the points into the "
                        "curves, pricing build yield…"):
            try:
                res = man.run()
            except ValueError as e:
                st.error(f"Engine refused: {e}")
                return
        man.record(res)
        ss["ig_run"] = {"manifest_json": man.to_json(), "result": res,
                        "verify": None, **ctx}
        w = res.winner
        ss["genesis_last"] = {
            "ok": res.ok, "verdict": w.verdict if w else "NO_FIT",
            "yield": w.yield_frac if w else None,
            "fit_band_frac": w.max_band_frac if w else None,
            "n_candidates": len(res.candidates),
            "premium": res.resilience_premium,
            "inputs_sha256": man.inputs_sha256,
        }

    run = ss.get("ig_run")
    if run is None:
        st.info("Declare the geometry, curves, volume and shop, then "
                "generate. The same declarations always produce the same "
                "geometry, and the manifest proves it.")
        return
    st.divider()
    _results_panel(st, pd, np, ss, run)
