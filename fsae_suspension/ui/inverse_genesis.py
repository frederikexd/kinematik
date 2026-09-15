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


_ROCKER_PTS = ("pushrod_outer", "rocker_pivot", "rocker_axis",
               "rocker_pushrod", "rocker_spring", "spring_inner")


def _parse_hardpoints_text(text: str, base):
    """JSON (manifest-style dict) or CSV ``point,x,y,z`` → Hardpoints.

    Linkage points not listed keep the base corner's values, but the base
    corner's pushrod/rocker is NOT carried over unless the text defines one:
    a pasted corner without a rocker must not inherit a motion ratio."""
    import io
    import json
    import pandas as pd
    from suspension import genesis_repro as gr
    text = text.strip()
    merged = gr.hp_to_dict(base)
    for k in _ROCKER_PTS:
        merged.pop(k, None)
    if text.startswith("{"):
        d = json.loads(text)
        d = d.get("hardpoints", d)
        merged.update(d)
        return gr.hp_from_dict(merged)
    df = pd.read_csv(io.StringIO(text))
    cols = {c.lower().strip(): c for c in df.columns}
    name_col = cols.get("point") or cols.get("hardpoint") or df.columns[0]
    known = set(merged) | set(_ROCKER_PTS)
    for _, row in df.iterrows():
        n = str(row[name_col]).strip()
        if n in known:
            merged[n] = [float(row[cols[a]]) for a in ("x", "y", "z")]
    return gr.hp_from_dict(merged)


def _log_run(ss, man, res, clearance=None):
    """Append one declaration → outcome row for the declaration log."""
    from suspension import genesis_repro as gr
    w = res.winner
    best = w if w is not None else res.best_fit
    row = {"declaration": man.name, "verdict": w.verdict if w else "NO_FIT",
           "best yield (%)": (round(100 * w.yield_frac, 1)
                              if w is not None and w.yield_frac is not None
                              else None),
           "worst station (× band)": (round(best.max_band_frac, 3)
                                      if best is not None else None),
           "binding row": best.worst_row if best is not None else "",
           "shop": (man.tolerance or {}).get("provenance", "")[:40],
           "RC band (± mm)": next((c["band"][0] for c in
                                   man.targets["curves"]
                                   if c["channel"] == "rc_height_mm"), None),
           "RC migration (mm/mm)": None, "clearance (mm)": clearance,
           "sha256": man.inputs_sha256[:12]}
    if res.winner_hp is not None:
        _, tg, _, _ = man.objects()
        d = gr.corner_diagnostics(res.winner_hp, stations=tg.stations(),
                                  track_mm=tg.track_mm)
        if d.get("ok"):
            row["RC migration (mm/mm)"] = round(
                d["rc_migration_chassis_mm_per_mm"], 3)
    ss.setdefault("ig_log", []).append(row)


def _g(value, unit="", limited_by="", digits=4):
    """Every number this tab prints is analytical: render it MODELLED."""
    from suspension.provenance import graded
    return graded(value, "modelled", unit, digits=digits,
                  limited_by=limited_by)


_TOL = "declared tolerance field"


def _results_panel(st, pd, np, ss, run):
    """Everything shown after a run; reads only ``ss['ig_run']``."""
    from suspension import inverse_genesis as ig
    from suspension import genesis_repro as gr
    from suspension import kinematik_stochastic as ks

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
                       f"{_g(res.best_fit.max_band_frac, '× band', digits=3)}, governed "
                       f"by {res.best_fit.worst_row}. This is an upper bound "
                       "on the true minimax distance, not the distance.")
    else:
        w = res.winner
        badge, blurb = _VERDICT_BLURB.get(w.verdict, ("", ""))
        ytxt = f" — build yield {_g(w.yield_frac * 100, '%', _TOL)}" \
            if w.yield_frac is not None else ""
        st.markdown(f"## {badge} {w.verdict}{ytxt}")
        st.caption(blurb)
        (st.success if res.ok else st.warning)(res.reason)
        m1, m2, m3, m4 = st.columns(4)
        if w.yield_frac is not None:
            m1.metric("Build yield", _g(w.yield_frac * 100, "%", _TOL))
        m2.metric("Worst station", _g(w.max_band_frac, "× band", digits=3),
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
          "build yield": (_g(c.yield_frac * 100, "%", _TOL)
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
    _ALL_ROWS = _POINT_ROWS + tuple(q for q in _ROCKER_PTS if q in whd)
    st.dataframe(pd.DataFrame(
        [{"point": p, "x": whd[p][0], "y": whd[p][1], "z": whd[p][2],
          "designed": p in volume.boxes} for p in _ALL_ROWS]),
        hide_index=True, width="stretch",
        column_config={a: st.column_config.NumberColumn(format="%.6f")
                       for a in "xyz"})
    csv = "point,x,y,z\n" + "\n".join(
        f"{p},{whd[p][0]!r},{whd[p][1]!r},{whd[p][2]!r}" for p in _ALL_ROWS)
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
                    f"First-order worst case **W = {_g(yb['worst_case_W'], digits=3, limited_by=_TOL)}** "
                    f"({yb['worst_case_row']}) — "
                    + ("every build in the box passes to first order."
                       if yb["guaranteed_first_order"] else
                       "W > 1: corner builds of the tolerance box can fail "
                       "even if no sample did.")
                    + f" Governing row {yb['governing_row']} has "
                    f"{_g(yb['governing_headroom_sigma'], 'σ', digits=3)} of headroom.")
                if "yield_lower_bound_95" in yb:
                    st.caption(f"Zero failures in {n}: yield ≥ "
                               f"{_g(100 * yb['yield_lower_bound_95'], '%')} at "
                               "95% (rule of three).")
                else:
                    st.caption(f"Standard error ≈ {_g(100 * yb['yield_se'], '', digits=3)} "
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
                        f"two-sided p = {_g(dcb['p_two_sided'], digits=3)}.")
            if st.button("Rounding sensitivity (±0.05 mm cell)",
                         key="ig_round"):
                rs = gr.rounding_sensitivity(
                    whp, targets, fld, list(volume.boxes), n=min(n, 2000),
                    seed=man.search.seed)
                if rs.get("ok"):
                    st.info(f"Yield within the 0.1 mm rounding cell: "
                            f"{_g(100 * rs['min'], '%', _TOL)} – {_g(100 * rs['max'], '%', _TOL)} "
                            f"over {rs['n_trials']} trials. Publish "
                            "coordinates at full precision.")

    # ---- swept volume ------------------------------------------------------ #
    obstacles = [o for o in volume.keep_out
                 if isinstance(o, gr.CapsuleObstacle)]
    frame = ss.get("tf_frame")
    with st.expander("🧱 Swept-volume clearance against the frame"):
        if not obstacles and frame:
            v = _veh(ss)
            ax = run.get("axle", "front")
            za = float(run.get("axle_station", v[f"{ax}_station_mm"]))
            yg = float(run.get("ground_y", v["ground_y_mm"]))
            st.caption(f"Frame placed with the declared {ax} axle station "
                       f"({za} mm) and ground plane ({yg} mm).")
            obstacles = [gr.capsules_from_framegraph(frame, za, yg)]
        if not obstacles:
            st.caption("Recover the frame from STEP under 2 · Keep-out "
                       "volumes, or load one in the Frame Planner, to run "
                       "this check.")
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
                    if ss.get("ig_log"):
                        ss["ig_log"][-1]["clearance (mm)"] = round(
                            sw["min_clearance_mm"], 1)
                    (st.success if sw["min_clearance_mm"] >= 0
                     else st.error)(
                        f"Minimum clearance {sw['min_clearance_mm']:+.1f} mm "
                        f"— {sw['link']} vs {sw['tube']} at "
                        f"{sw['travel_mm']:+.1f} mm travel.")
                    st.dataframe(pd.DataFrame(
                        [{"link": k, **v} for k, v in sw["per_link"].items()]),
                        hide_index=True, width="stretch")

    # ---- the same field under every shop class --------------------------- #
    with st.expander("🏭 Candidate yield by shop class"):
        st.caption("Re-runs these exact declarations once per shop class; "
                   "each row is ranked independently.")
        n6 = int(st.number_input("Sampled builds N per class", 100, 50000,
                                 int(man.search.n_yield), 100,
                                 key="ig_t6_n"))
        if st.button("Run all shop classes", key="ig_t6_go"):
            import copy
            rows6 = []
            for label, key in _SHOPS.items():
                m6 = copy.deepcopy(man)
                m6.tolerance = gr.field_to_dict(
                    ks.ToleranceField.preset(key))
                m6.search.n_yield = n6
                m6.recorded = None
                with st.spinner(label):
                    r6 = m6.run()
                ys = sorted((round(100 * c.yield_frac, 1)
                             for c in r6.candidates
                             if c.hit and c.yield_frac is not None),
                            reverse=True)
                rows6.append({"tolerance class": label,
                              **{str(i + 1): y for i, y in enumerate(ys)}})
            ss["ig_t6"] = rows6
        if ss.get("ig_t6"):
            st.dataframe(pd.DataFrame(ss["ig_t6"]), hide_index=True,
                         width="stretch")

    # ---- the declaration log ---------------------------------------------- #
    with st.expander("🗂 Declaration log — every run this session"):
        log = ss.get("ig_log", [])
        if log:
            df = pd.DataFrame(log)
            st.dataframe(df.astype(str), hide_index=True, width="stretch")
            st.download_button("Log (.csv)", df.to_csv(index=False),
                               file_name="declaration_log.csv",
                               mime="text/csv", key="ig_log_dl")
            if st.button("Clear log", key="ig_log_clear"):
                ss["ig_log"] = []
        st.caption("Name each run (Run name) after its declaration; run the "
                   "swept-volume check to fill the clearance column.")

    for wmsg in res.warnings:
        st.warning(wmsg)
    if res.ok and st.button("Apply the generated geometry to the live "
                            "hardpoints", key="ig_apply"):
        ss["hardpoints"] = whp
        st.success("Applied — every tab now consumes the generated corner.")


def render():
    import numpy as np
    import pandas as pd
    import streamlit as st

    ss = st.session_state
    _render_generate()
    _render_analysis(st, pd, np, ss)


def _render_generate():
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
        _v = _veh(ss)
        axle_station = float(_v[f"{axle}_station_mm"])
        ground_y = float(_v["ground_y_mm"])
        st.caption(f"Transform uses the declared {axle} axle station "
                   f"({axle_station} mm) and ground plane ({ground_y} mm) — "
                   "edit them under 4 · Vehicle & build declaration.")
        base = gr.hp_to_dict(Hardpoints.default())
        for _k in _ROCKER_PTS:
            base.pop(_k, None)
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
    ss["ig_seed_hp"] = gr.hp_to_dict(hp)

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
    modes = ["Formula (static + gain·t)", "Per-station table"]
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
        st.markdown("**Chassis frame from STEP**")
        _sec_frame_from_step(st, pd, np, ss, _veh(ss), "ig_step")
        use_frame = False
        if ss.get("tf_frame"):
            use_frame = st.checkbox(
                "Use the frame as capsule keep-outs", key="ig_use_frame")
            if use_frame and axle_station is None:
                _v = _veh(ss)
                axle_station = float(_v[f"{axle}_station_mm"])
                ground_y = float(_v["ground_y_mm"])
                st.caption(f"Placed with the declared {axle} axle station "
                           f"({axle_station} mm) and ground plane "
                           f"({ground_y} mm).")
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
        _log_run(ss, man, res)
        if res.winner_hp is not None:
            ss.setdefault("genesis_corners", {})[axle] = gr.hp_to_dict(
                res.winner_hp)
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
                "geometry, and the manifest proves it. The analysis below "
                "already runs on the seed geometry.")
        return
    st.divider()
    _results_panel(st, pd, np, ss, run)


# =========================================================================== #
#  Vehicle & build declaration — ONE set of declared numbers, shared by this
#  tab and the InverseGenesis-FullCar tab (kept in sync through session
#  state), feeding the corner & vehicle analysis below.
# =========================================================================== #
_VEH_KEY = "kk_vehicle"
_VEH_PREFIXES = ("ig_v", "fc_v")

#: (group, [(field, label, lo, hi, step, default)])
_VEH_FIELDS = [
    ("Vehicle", [
        ("mass_kg", "Mass incl. driver (kg)", 50.0, 1000.0, 1.0, 300.0),
        ("weight_dist_front", "Front weight fraction", 0.2, 0.8, 0.005, 0.48),
        ("cg_height_mm", "CG height (mm)", 50.0, 800.0, 1.0, 280.0),
        ("wheelbase_mm", "Wheelbase (mm)", 500.0, 4000.0, 1.0, 1630.0),
        ("track_front_mm", "Track front (mm)", 500.0, 2500.0, 1.0, 1210.0),
        ("track_rear_mm", "Track rear (mm)", 500.0, 2500.0, 1.0, 1210.0),
        ("tire_radius_mm", "Tyre radius (mm)", 100.0, 500.0, 1.0, 228.0),
        ("brake_bias_front", "Front brake bias", 0.0, 1.0, 0.01, 0.60),
    ]),
    ("Powertrain & aero", [
        ("power_kw", "Power (kW)", 1.0, 500.0, 1.0, 80.0),
        ("cla", "ClA (m²)", 0.0, 10.0, 0.1, 0.0),
        ("cda", "CdA (m²)", 0.0, 5.0, 0.05, 1.1),
    ]),
    ("Springs, bars & roll", [
        ("spring_front_lbin", "Coil rate front (lb/in)", 10.0, 3000.0, 1.0, 295.0),
        ("spring_rear_lbin", "Coil rate rear (lb/in)", 10.0, 3000.0, 1.0, 349.0),
        ("sprung_front_kg", "Sprung corner mass front (kg)", 5.0, 500.0, 0.1, 60.1),
        ("sprung_rear_kg", "Sprung corner mass rear (kg)", 5.0, 500.0, 0.1, 66.1),
        ("damper_stroke_mm", "Damper stroke (mm)", 1.0, 300.0, 1.0, 57.0),
        ("arb_front", "ARB front (N·m/deg)", 0.0, 5000.0, 1.0, 220.0),
        ("arb_rear", "ARB rear (N·m/deg)", 0.0, 5000.0, 1.0, 120.0),
        ("roll_stiffness_front", "Direct roll stiffness front (N·m/deg)", 0.0, 10000.0, 1.0, 458.0),
        ("roll_stiffness_rear", "Direct roll stiffness rear (N·m/deg)", 0.0, 10000.0, 1.0, 420.0),
    ]),
    ("Loads & tyre", [
        ("lateral_g", "Design lateral acceleration (g)", 0.1, 4.0, 0.05, 1.5),
        ("lateral_mu", "Lateral µ at the steering load", 0.1, 4.0, 0.01, 1.55),
        ("aligning_outer_Nm", "Aligning torque, outer (N·m)", 0.0, 500.0, 0.5, 50.0),
        ("aligning_inner_Nm", "Aligning torque, inner (N·m)", 0.0, 500.0, 0.5, 6.0),
        ("roll_share_front", "Front roll share for load cases", 0.0, 1.0, 0.01, 0.55),
        ("body_roll_deg", "Body roll at design g (deg)", 0.0, 10.0, 0.01, 1.18),
        ("tyre_optimum_camber", "Tyre optimum camber (deg)", -6.0, 1.0, 0.01, -1.83),
    ]),
    ("Steering", [
        ("steering_target_Nm", "Steering-wheel torque target (N·m)", 1.0, 100.0, 0.5, 10.0),
    ]),
    ("Chassis & packaging", [
        ("front_station_mm", "Front axle station, CAD Z (mm)", -5000.0, 5000.0, 1.0, 950.0),
        ("rear_station_mm", "Rear axle station, CAD Z (mm)", -5000.0, 5000.0, 1.0, -680.0),
        ("rules_min_wheelbase_mm", "Rules minimum wheelbase (mm)", 0.0, 5000.0, 1.0, 1525.0),
        ("ground_y_mm", "Ground plane, CAD Y (mm)", -1000.0, 1000.0, 0.5, -50.0),
        ("lowest_member_y_mm", "Lowest frame member, CAD Y (mm)", -1000.0, 1000.0, 0.1, -18.5),
        ("rim_offset_mm", "Max ball-joint offset from wheel centre (mm)", 10.0, 300.0, 1.0, 115.0),
    ]),
    ("Links & brackets", [
        ("tube_od_mm", "Link tube OD (mm)", 1.0, 100.0, 0.01, 15.88),
        ("tube_wall_mm", "Link tube wall (mm)", 0.1, 20.0, 0.001, 0.889),
        ("yield_mpa", "Allowable stress (MPa)", 50.0, 3000.0, 1.0, 460.0),
        ("E_gpa", "Young's modulus (GPa)", 10.0, 500.0, 1.0, 205.0),
        ("fos_min", "Required factor of safety", 0.5, 10.0, 0.1, 1.5),
        ("rod_end_lash_mm", "Rod-end lash per joint (mm)", 0.0, 2.0, 0.001, 0.025),
        ("bracket_kt", "Bracket stress-concentration Kt", 1.0, 5.0, 0.01, 1.28),
        ("bracket_plates", "Bracket plates (double shear = 2)", 1.0, 4.0, 1.0, 2.0),
    ]),
]
_VEH_CHOICES = {
    "tire_model": ("Tyre model", ["mf52_generic", "linear"], "mf52_generic"),
    "drive": ("Drive", ["rwd", "awd"], "rwd"),
    "roll_mode": ("Roll stiffness from",
                  ["declared directly", "springs × MR² + ARB"],
                  "declared directly"),
}
_VEH_TEXT = {"steering_ratios": ("Steering ratios", "4, 5, 6, 8")}


def _veh(ss):
    """The shared declaration dict, with every field present."""
    v = ss.setdefault(_VEH_KEY, {})
    for _, fields in _VEH_FIELDS:
        for f, *_rest, d in fields:
            v.setdefault(f, d)
    for f, (_, _, d) in _VEH_CHOICES.items():
        v.setdefault(f, d)
    for f, (_, d) in _VEH_TEXT.items():
        v.setdefault(f, d)
    return v


def _veh_sync(field, key):
    import streamlit as st
    ss = st.session_state
    val = ss[key]
    _veh(ss)[field] = val
    for p in _VEH_PREFIXES:
        other = f"{p}_{field}"
        if other != key:
            ss[other] = val


def _vehicle_editor(st, ss, prefix, expanded=False):
    """Draw the shared declaration. Two tabs draw it with different
    prefixes; an edit in either is copied to the other."""
    v = _veh(ss)
    with st.expander("🚗 Vehicle & build declaration — shared by "
                     "InverseGenesis and InverseGenesis-FullCar",
                     expanded=expanded):
        st.caption("Declared numbers, entered once. Every analysis in both "
                   "tabs reads them; an edit here shows up in the other tab.")
        c = st.columns(3)
        for i, (f, (label, opts, _)) in enumerate(_VEH_CHOICES.items()):
            key = f"{prefix}_{f}"
            if key not in ss:
                ss[key] = v[f]
            c[i].selectbox(label, opts, key=key, on_change=_veh_sync,
                           args=(f, key))
        for group, fields in _VEH_FIELDS:
            st.markdown(f"**{group}**")
            cols = st.columns(4)
            for i, (f, label, lo, hi, step, _) in enumerate(fields):
                key = f"{prefix}_{f}"
                if key not in ss:
                    ss[key] = float(v[f])
                fmt = "%.3f" if step < 0.01 else None
                cols[i % 4].number_input(label, lo, hi, step=step, key=key,
                                         format=fmt, on_change=_veh_sync,
                                         args=(f, key))
            if group == "Steering":
                for f, (label, _) in _VEH_TEXT.items():
                    key = f"{prefix}_{f}"
                    if key not in ss:
                        ss[key] = v[f]
                    cols[1].text_input(label, key=key, on_change=_veh_sync,
                                       args=(f, key))
    return v


# =========================================================================== #
#  Shared rendering helpers
# =========================================================================== #
def _floats(text, fallback):
    try:
        v = [float(x) for x in str(text).replace(";", ",").split(",")
             if x.strip()]
        return v or list(fallback)
    except ValueError:
        return list(fallback)


def _cell(v):
    if isinstance(v, bool) or v is None:
        return "—" if v is None else str(v)
    try:
        return str(round(float(v), 4))
    except (TypeError, ValueError):
        return str(v)


def _table(st, pd, rows, cols=None):
    """Render rows with every column stringified (4-decimal rounding) so a
    column that mixes numbers and labels never breaks the Arrow renderer."""
    df = pd.DataFrame(rows, columns=cols) if cols else pd.DataFrame(rows)
    for c in df.columns:
        df[c] = [_cell(x) for x in df[c]]
    st.dataframe(df, hide_index=True, width="stretch")


_LBIN = 0.1751268


def _declared_car(v, front_hp, rear_hp):
    """A DeclaredCar built from the shared declaration and two corners."""
    from suspension import inverse_genesis_fullcar as fc
    springs = (dict(use_spring_rates=True,
                    spring_rate_front=v["spring_front_lbin"] * _LBIN,
                    spring_rate_rear=v["spring_rear_lbin"] * _LBIN,
                    arb_rate_front=v["arb_front"],
                    arb_rate_rear=v["arb_rear"])
               if v["roll_mode"].startswith("springs") else
               dict(use_spring_rates=False,
                    roll_stiffness_front=v["roll_stiffness_front"],
                    roll_stiffness_rear=v["roll_stiffness_rear"]))
    return fc.DeclaredCar(
        mass_kg=v["mass_kg"], weight_dist_front=v["weight_dist_front"],
        cg_height_mm=v["cg_height_mm"], wheelbase_mm=v["wheelbase_mm"],
        track_front_mm=v["track_front_mm"], track_rear_mm=v["track_rear_mm"],
        tire_model=v["tire_model"], front_hp=front_hp, rear_hp=rear_hp,
        power_kw=v["power_kw"], cla=v["cla"], cda=v["cda"],
        drive=v["drive"], **springs)


def _corner_pair(ss, corner, axle):
    """(front, rear) corners: ``corner`` on its own axle, the other axle from
    an earlier InverseGenesis run if there was one, else the default."""
    from suspension import genesis_repro as gr
    from suspension.kinematics import Hardpoints
    store = ss.get("genesis_corners") or {}
    other = "rear" if axle == "front" else "front"
    oth = (gr.hp_from_dict(store[other]) if store.get(other)
           else Hardpoints.default())
    return (corner, oth) if axle == "front" else (oth, corner)


# =========================================================================== #
#  Analysis sections (each takes a corner + the declaration)
# =========================================================================== #
def _sec_steering(st, pd, v, front_hp, rear_hp, key):
    from suspension import genesis_analysis as ga
    from suspension.kinematics import SuspensionKinematics
    st.caption("Kingpin torque from mechanical trail and aligning torque, "
               "with the corner loads taken from the declared vehicle at the "
               "design lateral acceleration.")
    try:
        car = _declared_car(v, front_hp, rear_hp)
        loads, _ = car.vehicle().lateral_load_transfer(v["lateral_g"])
        fz_in, fz_out = float(loads.fl), float(loads.fr)
        caster = float(SuspensionKinematics(front_hp)
                       .solve_at_travel(0.0).caster)
    except Exception as e:                    # noqa: BLE001
        st.error(f"Vehicle model failed: {e}")
        return
    ratios = _floats(v["steering_ratios"], (4, 5, 6, 8))
    s = ga.steering_torque(caster, v["tire_radius_mm"], fz_out, fz_in,
                           v["lateral_mu"], v["aligning_outer_Nm"],
                           v["aligning_inner_Nm"], ratios,
                           v["steering_target_Nm"])
    _table(st, pd, [("caster, from the front corner (deg)", caster),
                    ("outer front Fz (N)", fz_out),
                    ("inner front Fz (N)", fz_in),
                    ("mechanical trail (mm)", s["trail_mm"]),
                    ("outer Fy (N)", s["fy_outer_N"]),
                    ("outer trail torque (N·m)", s["trail_torque_outer_Nm"]),
                    ("outer kingpin torque (N·m)", s["outer_Nm"]),
                    ("inner kingpin torque (N·m)", s["inner_Nm"]),
                    ("road-wheel total (N·m)", s["road_wheel_total_Nm"]),
                    ("ratio needed for the target", s["ratio_needed_for_target"])],
           ["quantity", "value"])
    _table(st, pd, s["rows"])


def _sec_actuation(st, pd, v, corner, axle):
    from suspension import genesis_analysis as ga
    k = v[f"spring_{axle}_lbin"] * _LBIN
    m = v[f"sprung_{axle}_kg"]
    trk = v[f"track_{axle}_mm"]
    a = ga.actuation_summary(corner, spring_rate_N_mm=k,
                             sprung_corner_mass_kg=m,
                             damper_stroke_mm=v["damper_stroke_mm"])
    mr = None
    if a.get("ok"):
        mr = a["mr_static"]
        _table(st, pd, [
            ("motion ratio, static", a["mr_static"]),
            ("motion ratio, droop", a["mr_droop"]),
            ("motion ratio, bump", a["mr_bump"]),
            ("rate character", a["rate_character"]),
            ("pushrod length (mm)", a["pushrod_length_mm"]),
            ("attachment, fraction out along the " + a["pushrod_attach"]
             + " arm", a["attachment_fraction"]),
            ("damper static length (mm)", a["damper_static_mm"]),
            ("damper travel used (mm)", a["damper_travel_used_mm"]),
            ("fraction of stroke used", a.get("stroke_fraction_used")),
            ("motion-ratio spread (%)", a["mr_spread_pct"]),
            ("wheel-rate spread (%)", a["wheel_rate_spread_pct"]),
            ("ride frequency, droop (Hz)", a["ride_hz_droop"]),
            ("ride frequency, static (Hz)", a["ride_hz_static"]),
            ("ride frequency, bump (Hz)", a["ride_hz_bump"])],
            ["quantity", "value"])
        st.line_chart(pd.DataFrame({"motion ratio": a["motion_ratio"]},
                                   index=pd.Index(a["travel_mm"],
                                                  name="travel (mm)")),
                      height=180)
    else:
        st.warning("No complete pushrod/rocker on this corner ("
                   + a["reason"] + "): the motion ratio below is ASSUMED, "
                   "not solved. Add the rocker points to the hardpoints to "
                   "solve it.")
    mr_used = mr if mr is not None else 1.0
    rs = ga.roll_stiffness_from_spring(k, mr_used, trk)
    _table(st, pd, [
        ("motion ratio used", f"{round(mr_used, 4)} "
         + ("(solved)" if mr is not None else "(ASSUMED placeholder)")),
        ("wheel rate (N/mm)", rs["wheel_rate_N_mm"]),
        ("spring roll stiffness (N·m/deg)", rs["roll_stiffness_Nm_deg"]),
        ("same with MR = 1.0 placeholder (N·m/deg)", rs["placeholder_Nm_deg"]),
        ("placeholder error (%)", rs["placeholder_error_pct"])],
        ["quantity", "value"])


def _sec_structural(st, pd, v, corner, axle):
    from suspension import genesis_repro as gr
    tube = gr.TubeSpec(od_mm=v["tube_od_mm"], wall_mm=v["tube_wall_mm"],
                       yield_mpa=v["yield_mpa"], E_gpa=v["E_gpa"])
    sc = gr.structural_screening(
        corner, tube=tube, fos_min=v["fos_min"], axle=axle,
        mass_kg=v["mass_kg"], weight_dist_front=v["weight_dist_front"],
        cg_height_mm=v["cg_height_mm"], track_mm=v[f"track_{axle}_mm"],
        wheelbase_mm=v["wheelbase_mm"],
        brake_bias_front=v["brake_bias_front"],
        roll_share_front=v["roll_share_front"],
        aligning_torque_Nm=v["aligning_outer_Nm"])
    st.caption("Five load cases (design-g cornering, 1.5 g braking, combined "
               "1.06 g, 3 g bump, kerb 2 g + 1 g); tension yield and "
               "pinned-pinned Euler buckling on the declared tube.")
    (st.success if sc["all_pass"] else st.error)(
        f"Governing member {sc['governing_member']}: worst FoS "
        f"{_g(sc['worst_fos_overall'], digits=3, limited_by='declared material')}")
    _table(st, pd, [{"member": m, "worst FoS": f,
                     "governing case": sc["governing_case"][m]}
                    for m, f in sc["worst_fos_per_member"].items()])
    _table(st, pd, sc["rows"])
    return sc


def _sec_compliance(st, pd, v, corner, axle, sc, bands):
    from suspension import genesis_repro as gr
    from suspension import genesis_analysis as ga
    cb = gr.compliance_budget(corner, axle=axle, od_mm=v["tube_od_mm"],
                              wall_mm=v["tube_wall_mm"],
                              lateral_g=v["lateral_g"], mass_kg=v["mass_kg"],
                              weight_dist_front=v["weight_dist_front"],
                              cg_height_mm=v["cg_height_mm"],
                              track_mm=v[f"track_{axle}_mm"],
                              wheelbase_mm=v["wheelbase_mm"],
                              roll_share_front=v["roll_share_front"],
                              aligning_torque_Nm=v["aligning_outer_Nm"])
    rows = []
    for case in ("cornering", "kerb strike"):
        r = cb.get(case, {})
        if "error" in r:
            st.error(f"{case}: {r['error']}")
            continue
        fr = ga.band_fractions(
            {"camber": r["compliance_camber_deg"],
             "toe": r["compliance_toe_deg"]}, bands)
        rows.append({"case": case,
                     "compliance camber (deg)": r["compliance_camber_deg"],
                     "share of camber band (%)": 100 * fr.get("camber", 0),
                     "compliance steer (deg)": r["compliance_toe_deg"],
                     "share of toe band (%)": 100 * fr.get("toe", 0),
                     "max link extension (mm)": r["max_link_extension_mm"]})
    _table(st, pd, rows)
    st.caption(f"Bands: camber ±{bands['camber']}°, toe ±{bands['toe']}° "
               "(from the run's targets when there is one). "
               + cb.get("note", ""))
    # per-link axial strain at the worst screened force, plus rod-end lash
    lrows = []
    worst = {}
    for r in sc["rows"]:
        if abs(r["force_N"]) > abs(worst.get(r["member"], {"force_N": 0})
                                   ["force_N"]):
            worst[r["member"]] = r
    for m, r in worst.items():
        b = ga.axial_budget(abs(r["force_N"]), r["length_mm"],
                            v["tube_od_mm"], v["tube_wall_mm"], v["E_gpa"],
                            v["rod_end_lash_mm"], 2)
        lrows.append({"member": m, "worst force (N)": r["force_N"],
                      "case": r["load_case"], "length (mm)": r["length_mm"],
                      "axial strain (mm)": b["axial_strain_mm"],
                      "rod-end lash (mm)": b["lash_mm"],
                      "total (mm)": b["sum_mm"]})
    _table(st, pd, lrows)


def _sec_brackets(st, pd, v, sc, key):
    from suspension import genesis_analysis as ga
    worst = max(sc["rows"], key=lambda r: abs(r["force_N"]))
    st.caption("Double-shear clevis root bending. The load defaults to the "
               f"worst screened link force ({worst['member']}, "
               f"{worst['load_case']}); reach and plate size are the design "
               "variables — shortening the reach is usually worth more than "
               "thickening the plate.")
    F0 = float(abs(worst["force_N"]))
    cases = st.data_editor(pd.DataFrame(
        {"case": ["as drawn", "thicker plates", "shorter reach"],
         "load (N)": [F0] * 3, "reach (mm)": [27.9, 27.9, 10.0],
         "plate width (mm)": [30.0, 40.0, 30.0],
         "plate thickness (mm)": [5.0, 6.0, 5.0]}),
        num_rows="dynamic", key=key, hide_index=True)
    rows = []
    for _, r in cases.iterrows():
        try:
            b = ga.bracket_fos(float(r["load (N)"]), float(r["reach (mm)"]),
                               float(r["plate width (mm)"]),
                               float(r["plate thickness (mm)"]),
                               int(v["bracket_plates"]), v["yield_mpa"],
                               v["bracket_kt"])
            rows.append({"case": r["case"], "stress (MPa)": b["stress_mpa"],
                         "FoS": b["fos"],
                         "meets required FoS": b["fos"] >= v["fos_min"]})
        except (TypeError, ValueError, ZeroDivisionError):
            continue
    _table(st, pd, rows)


def _sec_packaging(st, pd, v, corner):
    from suspension import genesis_analysis as ga
    w = ga.wheelbase_check(v["front_station_mm"], v["rear_station_mm"],
                           v["rules_min_wheelbase_mm"])
    cl = ga.static_clearance(v["lowest_member_y_mm"], v["ground_y_mm"])
    e = ga.ball_joint_envelope(corner, v["rim_offset_mm"])
    _table(st, pd, [
        ("wheelbase from axle stations (mm)", w["wheelbase_mm"]),
        ("margin to rules minimum (mm)", w["margin_mm"]),
        ("meets rules minimum", w["passes"]),
        ("declared wheelbase (mm)", v["wheelbase_mm"]),
        ("static clearance under frame (mm)", cl["clearance_mm"]),
        ("upper ball joint above wheel centre (mm)", e["upper_above_wc_mm"]),
        ("lower ball joint below wheel centre (mm)", e["lower_below_wc_mm"]),
        ("both joints inside the rim envelope", e["inside"])],
        ["quantity", "value"])
    if abs(w["wheelbase_mm"] - v["wheelbase_mm"]) > 0.5:
        st.warning("The declared wheelbase differs from the axle stations.")


def _sec_kinematics(st, pd, v, corner, axle, travel):
    from suspension import genesis_repro as gr
    trk = v[f"track_{axle}_mm"]
    d = gr.corner_diagnostics(corner, travel_mm=travel, track_mm=trk,
                              axle=axle, wheelbase_mm=v["wheelbase_mm"],
                              cg_height_mm=v["cg_height_mm"],
                              brake_bias_front=v["brake_bias_front"])
    if not d.get("ok"):
        st.error("The corner does not solve over this travel range.")
        return None
    rows = [("camber gain, LSQ (deg/mm)", d["camber_gain_deg_per_mm"]),
            ("bump steer, LSQ (deg/mm)", d["bump_steer_deg_per_mm"]),
            ("toe change, max − min (deg)", d["toe_change_deg"]),
            ("RC height, static (mm)", d["rc_height_static_mm"]),
            ("RC migration, chassis frame (mm/mm)",
             d["rc_migration_chassis_mm_per_mm"]),
            ("RC migration, above ground (mm/mm)",
             d["rc_migration_ground_mm_per_mm"]),
            ("RC above ground, minimum (mm)", d["rc_above_ground_min_mm"]),
            ("caster (deg)", d["caster_deg"]),
            ("kingpin inclination (deg)", d["kpi_deg"]),
            ("scrub radius (mm)", d["scrub_static_mm"]),
            ("contact-patch rise per mm of travel",
             d["contact_patch_rise_per_mm"]),
            ("side-view IC, x rearward (mm)", d["side_view_ic_x_mm"]),
            ("side-view IC, height (mm)", d["side_view_ic_z_mm"]),
            ("tan swing arm (" + d["side_view_reference"] + ")",
             d["side_view_tan"])]
    if "anti_dive_pct" in d:
        rows.append(("anti-dive (%)", d["anti_dive_pct"]))
    if "anti_squat_pct" in d:
        rows.append(("anti-squat (%)", d["anti_squat_pct"]))
    _table(st, pd, rows, ["quantity", "value"])
    _table(st, pd, [{"travel (mm)": t, "camber (deg)": a, "toe (deg)": b,
                     "RC height (mm)": r, "scrub (mm)": sc}
                    for t, a, b, r, sc in zip(
                        d["stations_mm"], d["camber_deg"], d["toe_deg"],
                        d["rc_height_mm"], d["scrub_mm"])])
    ctr = gr.camber_to_road(corner.static_camber,
                            d["camber_gain_deg_per_mm"], v["body_roll_deg"],
                            trk, v["tyre_optimum_camber"])
    st.markdown("**Loaded tyre camber relative to the road** "
                "(outside wheel at the declared body roll)")
    _table(st, pd, [(k, x) for k, x in ctr.items()], ["quantity", "value"])
    st.caption("None of these is a channel — the solver is indifferent to "
               "all of them, so check them on every corner.")
    return d


def _sec_tyre(st, pd, v, key):
    from suspension import genesis_analysis as ga
    from suspension.tiremodel import default_tire
    if v["tire_model"] != "mf52_generic":
        st.caption("The linear placeholder tyre has no load or camber "
                   "sensitivity to tabulate.")
        return
    fz_s = v["mass_kg"] * 9.81 * v["weight_dist_front"] / 2
    loads = _floats(st.text_input("Vertical loads (N)", "550, 1100, 1650",
                                  key=key, help="static front corner ≈ "
                                  + str(round(fz_s)) + " N"), (550, 1100, 1650))
    _table(st, pd, ga.tyre_table(loads, default_tire()))
    st.caption("Synthetic MF5.2 pure-lateral set — not fitted to measured "
               "data.")


def _sec_frame_from_step(st, pd, np, ss, v, key):
    """Chassis STEP → tube axes → frame summary, stored as the Frame Planner
    frame so keep-outs and the swept-volume check can use it."""
    from suspension import genesis_analysis as ga
    from suspension.tubeframe import FrameGraph
    up = st.file_uploader("Chassis STEP (.step / .stp)", type=["step", "stp"],
                          key=f"{key}_up")
    c = st.columns(4)
    R = c[0].number_input("Tube outer radius (mm)", 1.0, 50.0, 12.70, 0.01,
                          key=f"{key}_R")
    rtol = c[1].number_input("Radius tolerance (mm)", 0.001, 2.0, 0.02,
                             0.001, format="%.3f", key=f"{key}_rtol")
    atol = c[2].number_input("Coaxial tolerance (mm)", 0.01, 10.0, 0.5, 0.01,
                             key=f"{key}_atol")
    tols = _floats(c[3].text_input("Node clustering tolerances (mm)",
                                   "20, 30, 37, 40, 50", key=f"{key}_tols"),
                   (20, 30, 37, 40, 50))
    extra = st.data_editor(
        pd.DataFrame(columns=["bend major radius (mm)", "bend angle (deg)"]),
        num_rows="dynamic", key=f"{key}_bends", hide_index=True)
    manual = [(float(r.iloc[0]), float(r.iloc[1]))
              for _, r in extra.iterrows()
              if not (pd.isna(r.iloc[0]) or pd.isna(r.iloc[1]))]
    if up is not None and st.button("Recover the frame", key=f"{key}_go"):
        with st.spinner("Reading the B-rep…"):
            res = ga.parse_step_tubes(
                up.getvalue().decode("utf-8", errors="replace"), R, rtol, atol)
            stats = ga.frame_stats(res.axes, list(res.bends) + manual, tols,
                                   res.vertices, R)
        g = FrameGraph()
        ends = np.array([q for a, b, _ in res.axes for q in (a, b)])
        if len(ends):
            _, lab = ga._cluster_count(ends, float(tols[0]))
            for l in sorted(set(lab.tolist())):
                g.add_node(f"N{l}", tuple(ends[lab == l].mean(axis=0)))
            for i in range(len(res.axes)):
                a, b = f"N{lab[2 * i]}", f"N{lab[2 * i + 1]}"
                if a != b:
                    g.add_tube(f"T{i + 1:02d}", a, b)
            ss["tf_frame"] = g.as_dict()
        ss["ig_step_frame"] = (res, stats)
    if ss.get("ig_step_frame"):
        res, s = ss["ig_step_frame"]
        st.caption(res.units_note + " · axis lines rejected: "
                   + str(res.rejected) + " · stored as the Frame Planner "
                   "frame, so it is available as keep-outs below and to the "
                   "swept-volume check.")
        _table(st, pd, [("tubes", s["tubes"]), ("straight run (m)", s["straight_m"]),
                        ("bends", s["bends"]), ("bend arc (m)", s["bend_arc_m"]),
                        ("total tube length (m)", s["total_m"])],
               ["quantity", "value"])
        _table(st, pd, [{"clustering tolerance (mm)": k, "nodes": x["nodes"],
                         "node-to-node length (m)": x["node_to_node_m"]}
                        for k, x in s["nodes"].items()])
        _table(st, pd, [{"wall (mm)": str(k), "tubes": n}
                        for k, n in s["walls_mm"].items()])
        if res.bends:
            _table(st, pd, [{"bend": i + 1, "major radius (mm)": r,
                             "swept angle (deg)": a}
                            for i, (r, a) in enumerate(res.bends)])
        if "vertices" in s:
            _table(st, pd, [{**s["vertices"],
                             "cylinder radii (mm: faces)": str(res.radii_mm)}])


def _render_analysis(st, pd, np, ss):
    """Section 4 + 5 of the tab: the shared declaration and every analysis of
    the current corner (generated, else the seed)."""
    from suspension import genesis_repro as gr
    from suspension.kinematics import Hardpoints
    st.divider()
    st.markdown("###### 4 · Vehicle & build declaration")
    v = _vehicle_editor(st, ss, "ig_v")

    run = ss.get("ig_run")
    if run is not None and run["result"].winner_hp is not None:
        corner, note = run["result"].winner_hp, "the generated corner"
        axle = run.get("axle", "front")
        _, tg, _, _ = gr.GenesisManifest.from_json(
            run["manifest_json"]).objects()
        travel = float(max(abs(t) for t in tg.stations()))
        bands = {"camber": 0.30, "toe": 0.08}
        for c in tg.curves:
            if c.channel == "camber_deg":
                bands["camber"] = float(np.min(c.band))
            if c.channel == "toe_deg":
                bands["toe"] = float(np.min(c.band))
    else:
        seed = ss.get("ig_seed_hp")
        corner = gr.hp_from_dict(seed) if seed else Hardpoints.default()
        note = "the seed geometry (no run yet)"
        axle = ss.get("ig_axle", "front")
        travel = float(ss.get("ig_travel", 25.0))
        bands = {"camber": 0.30, "toe": 0.08}

    st.markdown("###### 5 · Corner & vehicle analysis")
    st.caption(f"For {note}, {axle} axle, with the declaration above.")
    front_hp, rear_hp = _corner_pair(ss, corner, axle)

    with st.expander("🔎 Kinematics, anti-geometry and loaded camber",
                     expanded=True):
        _sec_kinematics(st, pd, v, corner, axle, travel)
    with st.expander("📐 Actuation, ride frequency and roll stiffness"):
        _sec_actuation(st, pd, v, corner, axle)
    with st.expander("🛞 Steering effort"):
        if axle != "front":
            st.caption("Uses the front corner — shown with the front corner "
                       "from an earlier run, or the default.")
        _sec_steering(st, pd, v, front_hp, rear_hp, "ig_steer")
    sc = None
    with st.expander("🔩 Structural screening of the links"):
        sc = _sec_structural(st, pd, v, corner, axle)
    with st.expander("⚙️ Compliance budget — link strain, lash and band use"):
        _sec_compliance(st, pd, v, corner, axle, sc, bands)
    with st.expander("🪝 Bracket screening"):
        _sec_brackets(st, pd, v, sc, "ig_brk")
    with st.expander("📦 Packaging — wheelbase, ground clearance, rim envelope"):
        _sec_packaging(st, pd, v, corner)
