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

Session keys used:
    hardpoints     read/write: the live geometry (Kinematics tab dialect);
                   "apply" writes the generated Hardpoints back so every
                   other tab consumes the generated car.
    genesis_last   summary dict of the last run (for cross-tab reads)
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
    """The live hardpoint set if the kinematics tab has one, else default."""
    import numpy as np
    from suspension.kinematics import Hardpoints
    raw = ss.get("hardpoints")
    if isinstance(raw, Hardpoints):
        return raw, "live hardpoints from the Kinematics tab"
    if isinstance(raw, dict):
        try:
            kw = {k: (np.asarray(v, float) if isinstance(v, (list, tuple))
                      else v) for k, v in raw.items()}
            return Hardpoints(**kw), "live hardpoints from the Kinematics tab"
        except Exception:
            pass
    return Hardpoints.default(), \
        "default FSAE front corner (no live hardpoints set)"


def render():
    import numpy as np
    import pandas as pd
    import streamlit as st
    from suspension import inverse_genesis as ig
    from suspension import kinematik_stochastic as ks

    ss = st.session_state

    st.subheader("🧬 InverseGenesis — draw the curves; the engine generates "
                 "the geometry")
    st.caption(
        "Every other tab runs the loop forward: guess coordinates, solve, "
        "read the curves, guess again — days of translating intent into "
        "millimetres by hand. This tab runs it backwards. Draw the kinematic "
        "curves you want inside acceptance bands, box the legal volume each "
        "hardpoint may occupy (minus the space the headers and mounts "
        "already claim), and the engine pulls the coordinates into "
        "alignment with deterministic reverse gradients — then rejects "
        "every knife-edge optimum your shop can't hold, keeping the "
        "geometry with the highest BUILD YIELD under the Stochastic "
        "Inversion error field. Rigid kinematics only; validate the "
        "generated corner in Ghost Topology and full simulation before "
        "cutting metal.")

    hp, hp_note = _hardpoints_from_session(ss)
    st.caption(f"Geometry: {hp_note}.")

    # ================= 1 · draw the curves ================================
    st.markdown("###### 1 · Draw the target curves — the intent, in bands")
    st.caption("Seeded from the live nominal's own curves so the sheet "
               "solves before your first edit — bend targets and bands to "
               "the car you want. A band is the acceptance half-width: it's "
               "also the currency build-yield spends, so a zero band would "
               "honestly ask for a probability-zero car (the editor floors "
               "it).")
    c1, c2 = st.columns(2)
    travel = c1.slider("Travel range (± mm)", 10.0, 35.0, 25.0, 2.5,
                       key="ig_travel")
    n_st = int(c2.select_slider("Stations", [3, 5, 7, 9], value=5,
                                key="ig_nst"))
    stations = np.linspace(-travel, travel, n_st)

    nom_vals, nom_ok = ig.curves_of(hp, stations)
    if not nom_ok:
        st.error("The nominal geometry does not solve over this travel "
                 "range — shrink the range or fix the hardpoints in the "
                 "Kinematics tab first.")
        return

    default_band = {"camber_deg": 0.20, "toe_deg": 0.08,
                    "rc_height_mm": 6.0, "scrub_mm": 3.0}
    enabled = st.multiselect(
        "Channels to anchor", list(ig.CHANNELS),
        default=["camber_deg", "toe_deg", "rc_height_mm"],
        format_func=lambda ch: _CHANNEL_UI[ch][0], key="ig_channels")
    if not enabled:
        st.info("Anchor at least one channel — with no curves drawn there "
                "is nothing to invert.")
        return

    curves = []
    for ch in enabled:
        label, hint = _CHANNEL_UI[ch]
        st.markdown(f"**{label}** — {hint}")
        seed_df = pd.DataFrame({
            "travel_mm": stations.round(1),
            "target": np.asarray(nom_vals[ch], float).round(3),
            "band": np.full(n_st, default_band[ch]),
        })
        edited = st.data_editor(
            seed_df, key=f"ig_curve_{ch}", hide_index=True,
            column_config={
                "travel_mm": st.column_config.NumberColumn(
                    "travel (mm)", disabled=True),
                "target": st.column_config.NumberColumn("target",
                                                        format="%.3f"),
                "band": st.column_config.NumberColumn("± band",
                                                      format="%.3f"),
            })
        band = np.maximum(np.abs(edited["band"].to_numpy(float)), 1e-3)
        try:
            curves.append(ig.TargetCurve(
                ch, stations, edited["target"].to_numpy(float), band))
        except ValueError as e:
            st.error(f"{label}: {e}")
            return
    targets = ig.GenesisTargets(curves=curves)

    # ================= 2 · the legal volume ===============================
    st.markdown("###### 2 · The legal volume — where points may exist")
    st.caption("The physics-informed boundary filter: every search step is "
               "clamped to these per-point boxes, and geometries touching a "
               "keep-out volume are rejected outright — a wall, not a "
               "penalty.")
    movable = st.multiselect(
        "Hardpoints the engine may move", list(ig.DESIGNABLE_POINTS),
        default=["upper_front_inner", "upper_rear_inner",
                 "lower_front_inner", "lower_rear_inner"],
        key="ig_movable")
    if not movable:
        st.info("Free at least one hardpoint — with zero freedoms there is "
                "nothing to generate.")
        return
    half = {}
    cols = st.columns(min(4, len(movable)))
    for i, p in enumerate(movable):
        half[p] = cols[i % len(cols)].number_input(
            f"{p} ± (mm)", 0.5, 40.0, 8.0, 0.5, key=f"ig_half_{p}")

    with st.expander("Keep-out volumes (headers, mounts, bodywork)"):
        st.caption("Axis-aligned boxes in corner axes (mm), same frame as "
                   "the hardpoints. The headless API also accepts carved "
                   "Phantom Envelope point clouds of NEIGHBOURING "
                   "assemblies — never this corner's own envelope, whose "
                   "capsule endpoints would always violate.")
        ko_df = st.data_editor(
            pd.DataFrame(columns=["label", "x_lo", "y_lo", "z_lo",
                                  "x_hi", "y_hi", "z_hi"]),
            key="ig_keepout", num_rows="dynamic", hide_index=True)
        k1, k2 = st.columns(2)
        probe = k1.number_input("Probe radius (mm) — covers the tab, not "
                                "just the pickup", 0.0, 30.0, 6.0, 1.0,
                                key="ig_probe")
        min_cl = k2.number_input("Required clearance (mm)", 0.0, 20.0, 2.0,
                                 0.5, key="ig_mincl")
    keep_out = []
    for _, row in ko_df.iterrows():
        try:
            lo = np.array([row["x_lo"], row["y_lo"], row["z_lo"]], float)
            hi = np.array([row["x_hi"], row["y_hi"], row["z_hi"]], float)
            keep_out.append(ig.KeepOutBox(lo, hi,
                                          label=str(row.get("label") or
                                                    "keep-out box")))
        except (ValueError, TypeError) as e:
            st.error(f"Keep-out row skipped: {e}")

    try:
        volume = ig.LegalVolume.around(hp, half, points=movable,
                                       keep_out=keep_out,
                                       probe_radius_mm=float(probe),
                                       min_clearance_mm=float(min_cl))
    except ValueError as e:
        st.error(f"Legal volume refused: {e}")
        return

    # ================= 3 · the co-optimizer ===============================
    st.markdown("###### 3 · The build-yield co-optimizer — the shop's veto")
    s1, s2, s3 = st.columns([2, 1, 1])
    shop_label = s1.selectbox("Shop class (Stochastic Inversion preset)",
                              list(_SHOPS.keys()), key="ig_shop")
    pull = s2.number_input("Weld pull (mm)", 0.0, 3.0, 0.0, 0.1,
                           key="ig_pull")
    pull_axis = s3.selectbox("Pull axis", _AXES, index=2, key="ig_pull_axis")
    fld = ks.ToleranceField.preset(_SHOPS[shop_label],
                                   weld_pull_mm=float(pull),
                                   pull_axis=pull_axis)
    r1, r2, r3 = st.columns(3)
    n_starts = int(r1.select_slider("Deterministic starts", [3, 5, 8, 12],
                                    value=5, key="ig_nstarts"))
    n_yield = int(r2.select_slider("Sampled builds / candidate",
                                   [1000, 2000, 4000, 8000], value=4000,
                                   key="ig_nyield"))
    n_verify = int(r3.select_slider("Full-solve verification",
                                    [0, 30, 60, 120], value=60,
                                    key="ig_nverify"))

    if not st.button("🧬 Generate the geometry", key="ig_run",
                     type="primary"):
        st.info("Draw the curves, box the volume, declare the shop, then "
                "generate. The engine is deterministic — the same "
                "declarations always produce the same geometry.")
        return

    with st.spinner("Reverse gradients pulling the points into the curves, "
                    "co-optimizing against the error field, pricing the "
                    "linearisation…"):
        try:
            res = ig.inverse_genesis(hp, targets, volume, fld=fld,
                                     n_starts=n_starts, n_yield=n_yield,
                                     n_verify_full=n_verify, seed=0)
        except ValueError as e:
            st.error(f"Engine refused: {e}")
            return

    # ================= the verdict ========================================
    if res.winner is None:
        st.markdown("## ⚪ NO LEGAL GEOMETRY REACHES THE CURVES")
        st.error(res.reason)
        if res.best_fit is not None:
            st.caption(f"Closest legal approach: "
                       f"{res.best_fit.max_band_frac:.2f}× band, governed "
                       f"by {res.best_fit.worst_row}.")
        ss["genesis_last"] = {"ok": False, "verdict": "NO_FIT"}
        return

    w = res.winner
    badge, blurb = _VERDICT_BLURB.get(w.verdict, ("", ""))
    ytxt = f" — build yield {w.yield_frac*100:.1f}%" \
        if w.yield_frac is not None else ""
    st.markdown(f"## {badge} {w.verdict}{ytxt}")
    st.caption(blurb)
    (st.success if res.ok else st.warning)(res.reason)

    m1, m2, m3, m4 = st.columns(4)
    if w.yield_frac is not None:
        m1.metric("Build yield", f"{w.yield_frac*100:.1f} %",
                  help="Fraction of the cars your declared shop would weld "
                       "whose curves STAY inside your drawn bands — fit "
                       "residual spends band width first; scatter gets the "
                       "headroom.")
    m2.metric("Worst station", f"{w.max_band_frac:.2f}× band",
              delta=w.worst_row, delta_color="off")
    m3.metric("Iterations", f"{w.iterations}",
              help="Damped Gauss–Newton steps: linearise, step, clamp to "
                   "the legal boxes, reject on keep-out contact, repeat.")
    if res.resilience_premium is not None and res.winner is not res.best_fit:
        m4.metric("Resilience premium", f"{res.resilience_premium*100:+.1f} %",
                  help="Yield bought by rejecting the pure best-fit "
                       "knife-edge in favour of this candidate.")

    # generated shifts
    st.markdown("###### The generated geometry — shifts from nominal (mm)")
    if w.shifts:
        st.dataframe(pd.DataFrame(
            [{"hardpoint": p, "Δx": round(float(v[0]), 2),
              "Δy": round(float(v[1]), 2), "Δz": round(float(v[2]), 2)}
             for p, v in sorted(w.shifts.items())]),
            hide_index=True, use_container_width=True)
    else:
        st.caption("The nominal already satisfies the drawn curves.")
    if w.clamped:
        st.caption(f"Pinned to the legal box: {', '.join(w.clamped)} — the "
                   "volume is binding there; more freedom would buy more "
                   "fit or yield.")
    if w.keepout_rejections:
        st.caption(f"The boundary filter refused {w.keepout_rejections} "
                   "step(s) toward a keep-out volume.")

    # curve overlay: nominal vs generated vs band
    dense = np.linspace(-travel, travel, 41)
    gen_vals, gen_ok = ig.curves_of(res.winner_hp, dense)
    nomd, _ = ig.curves_of(hp, dense)
    if gen_ok:
        for c in targets.curves:
            lo_b = np.interp(dense, c.travel_mm, c.target - c.band)
            hi_b = np.interp(dense, c.travel_mm, c.target + c.band)
            st.markdown(f"**{_CHANNEL_UI[c.channel][0]}**")
            st.line_chart(pd.DataFrame({
                "band lo": lo_b, "band hi": hi_b,
                "nominal": nomd[c.channel],
                "generated": gen_vals[c.channel],
            }, index=pd.Index(dense.round(1), name="travel (mm)")),
                height=220)

    # candidate family
    if len(res.candidates) > 1:
        st.markdown("###### The candidate family — what a point of yield "
                    "costs in fit")
        st.dataframe(pd.DataFrame(
            [{"verdict": c.verdict, "fit (×band)":
              round(c.max_band_frac, 2),
              "build yield": (f"{c.yield_frac*100:.1f}%"
                              if c.yield_frac is not None else "—"),
              "governed by": c.worst_row}
             for c in res.candidates]),
            hide_index=True, use_container_width=True)

    if res.verify_yield is not None:
        st.caption(f"Linearisation, priced: full-solve verification yield "
                   f"{res.verify_yield*100:.1f}%; linear/full pass-fail "
                   f"agreement {res.verify_agreement*100:.1f}% (floor "
                   f"{res.thresholds.verify_agreement:.0%}).")
    for wmsg in res.warnings:
        st.warning(wmsg)

    # ================= exports & apply ====================================
    st.download_button("Report (.md)", ig.render_genesis_md(res, targets),
                       file_name="inverse_genesis.md",
                       mime="text/markdown", key="ig_dl_md")
    if res.ok and st.button("Apply the generated geometry to the live "
                            "hardpoints", key="ig_apply"):
        ss["hardpoints"] = res.winner_hp
        st.success("Applied — every tab now consumes the generated corner. "
                   "Take it to Ghost Topology next: this engine solved the "
                   "rigid car; the loaded one still needs its audit.")

    ss["genesis_last"] = {
        "ok": res.ok, "verdict": w.verdict,
        "yield": w.yield_frac, "fit_band_frac": w.max_band_frac,
        "n_candidates": len(res.candidates),
        "premium": res.resilience_premium,
    }
