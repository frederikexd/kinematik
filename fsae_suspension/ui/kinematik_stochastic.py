# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/kinematik_stochastic.py — the 🎲🛡️ Stochastic Inversion tab
#  (manufacturing tolerance sweep · robust nudge · alignment prescription)
# ============================================================================
"""
The tab that stops pretending the car will be built perfectly.

Three panels, three questions:

  1. TOLERANCE SWEEP — declare the error field your shop actually produces
     (per point, per axis, asymmetric — a weld pulls TOWARD the bead) and the
     kinematic bands you're willing to accept, and get the manufacturing
     YIELD: the fraction of cars you could build that are still the car you
     designed, with the yield-killing metric and hardpoint coordinate named.
  2. ROBUST NUDGE — when the field is asymmetric, the expected as-built car
     is biased off the intent before the first cut. The nudge re-aims the
     nominal up-wind of the pull; a centred field gets the honest sentence
     instead of a fabricated optimum.
  3. ALIGNMENT PRESCRIPTION — after welding, paste the measured as-built
     coordinates and the adjusters the car really has (shim packs, rod ends:
     axis, range, step), and get the shim arithmetic that restores the
     intent — verified by a full re-solve, unreachable residuals named.

All physics lives in suspension/kinematik_stochastic.py; this module only
orchestrates the solver and draws (see ui/__init__.py rules).

Session keys used:
    hardpoints        read: the live geometry when the Kinematics tab set it
    stochastic_last   summary dict of the last sweep (for cross-tab reads)
"""

from __future__ import annotations


_AXES = ("x", "y", "z")

_SHOPS = {
    "Hand-welded tabs (±1.5 mm)": "hand_weld",
    "Jig-welded tabs (±0.5 mm)": "jig_weld",
    "CNC everything (±0.05 mm)": "cnc",
}

_POINT_HELP = {
    "upper_front_inner": "upper wishbone front chassis tab",
    "upper_rear_inner":  "upper wishbone rear chassis tab",
    "lower_front_inner": "lower wishbone front chassis tab",
    "lower_rear_inner":  "lower wishbone rear chassis tab",
    "upper_outer":       "upper ball joint (upright, machined)",
    "lower_outer":       "lower ball joint (upright, machined)",
    "tie_rod_inner":     "tie rod at the rack (machined)",
    "tie_rod_outer":     "tie rod at the upright (machined)",
}

_VERDICT_BLURB = {
    "ROBUST":   ("🟢", "Your shop can build this geometry. The population of "
                       "cars inside your declared error field meets the "
                       "kinematic intent at this yield — the design is "
                       "computationally immune to the declared error."),
    "MARGINAL": ("🟡", "Roughly one build in five misses the intent. Look at "
                       "the dominant error source below: jigging THAT tab (or "
                       "renegotiating that band) buys the most yield per hour "
                       "of shop discipline."),
    "FRAGILE":  ("🔴", "This geometry only exists on screen. Most of the cars "
                       "your shop would actually produce are NOT the car you "
                       "designed — the tuning you do in the other tabs is "
                       "tuning a car that won't be built."),
    "SOLVER_LIMITED": ("🔴", "Perturbed geometries near your tolerance bounds "
                             "fail to solve at all — the nominal sits near a "
                             "kinematic singularity. No shim fixes that; move "
                             "the design away from the cliff first."),
}


def _hardpoints_from_session(ss):
    """The live hardpoint set if the kinematics tab has one, else the default."""
    import numpy as np
    from suspension.kinematics import Hardpoints
    raw = ss.get("hardpoints")
    if isinstance(raw, Hardpoints):
        return raw, "live hardpoints from the Kinematics tab"
    if isinstance(raw, dict):
        try:
            kw = {k: (np.asarray(v, float) if isinstance(v, (list, tuple)) else v)
                  for k, v in raw.items()}
            return Hardpoints(**kw), "live hardpoints from the Kinematics tab"
        except Exception:
            pass
    return Hardpoints.default(), "default FSAE front corner (no live hardpoints set)"


def _parse_asbuilt_csv(text: str, hp):
    """name,x,y,z lines → {point: measured coords}; returns (dict, errors)."""
    import numpy as np
    out, errs = {}, []
    for ln in text.strip().splitlines():
        parts = [p.strip() for p in ln.replace("\t", ",").split(",") if p.strip()]
        if not parts:
            continue
        if len(parts) != 4:
            errs.append(f"'{ln}' — expected name,x,y,z")
            continue
        name = parts[0]
        if not hasattr(hp, name) or getattr(hp, name) is None:
            errs.append(f"'{name}' is not a hardpoint on this corner")
            continue
        try:
            out[name] = np.array([float(v) for v in parts[1:]])
        except ValueError:
            errs.append(f"'{ln}' — coordinates must be numbers")
    return out, errs


def render():
    import numpy as np
    import streamlit as st
    from suspension import units as _units
    from suspension import kinematik_stochastic as ks

    ss = st.session_state

    st.subheader("🎲🛡️ Stochastic Inversion — design the car your shop can "
                 "actually build")
    st.caption(
        "Every other tab takes the hardpoints as exact. Out on the floor the "
        "welds pull, the jigs stack, the rod ends carry play — the car that "
        "gets built is a random draw from a cloud around the design, and no "
        "tool in the chain has ever asked which of those cars the solver was "
        "solving. This tab declares the error field your shop actually "
        "produces, sweeps thousands of buildable cars through the kinematics, "
        "and reports the YIELD: the fraction that are still the car you "
        "designed. Then it aims the nominal up-wind of any systematic weld "
        "pull, and — once the chassis is welded and measured — turns the "
        "as-built error into a shim-pack prescription. Kinematic intent only; "
        "loads, clearances and temperatures stay with Ghost Topology, "
        "Phantom Envelope and ThermicPatch, which this tab hands a "
        "population instead of a point.")

    hp, hp_note = _hardpoints_from_session(ss)
    st.caption(f"Geometry: {hp_note}.")

    # ================= 1 · the error field ================================
    st.markdown("###### 1 · The error field — what your shop actually holds")
    c1, c2, c3 = st.columns([2, 1, 1])
    with c1:
        shop_label = st.selectbox("Shop class (seed — every cell editable "
                                  "below)", list(_SHOPS.keys()),
                                  key="stoch_shop")
    with c2:
        pull = _units.unum(st, "Systematic weld pull (mm)", 0.0, 3.0, 0.0, 'mm', step=0.1, key="stoch_pull", help="A weld bead laid on one side draws the "
                                    "tab TOWARD it every time — a bias, not a "
                                    "scatter. Applied to the four wishbone "
                                    "tabs along the axis chosen next.")
    with c3:
        pull_axis = st.selectbox("Pull axis", _AXES, index=2,
                                 key="stoch_pull_axis",
                                 help="SAE corner axes: x rearward, y "
                                      "outboard, z up.")
    dist = st.radio("Error distribution", ["uniform", "normal"], index=0,
                    horizontal=True, key="stoch_dist",
                    help="Uniform = worst-case-honest box (anywhere in the "
                         "bounds equally likely). Normal = a shop that mostly "
                         "lands mid-box (truncated at the bounds, σ = span/4).")

    fld = ks.ToleranceField.preset(_SHOPS[shop_label],
                                   weld_pull_mm=float(pull),
                                   pull_axis=pull_axis)

    with st.expander("Per-point bounds (mm) — edit to your shop"):
        st.caption("Asymmetric on purpose: lo and hi are separate per axis, "
                   "because manufacturing error has a direction. lo ≤ hi.")
        edited = {}
        for pname in sorted(fld.specs):
            spec = fld.specs[pname]
            st.markdown(f"**{pname}** — {_POINT_HELP.get(pname, '')}")
            cols = st.columns(6)
            lo = np.zeros(3)
            hi = np.zeros(3)
            for a in range(3):
                lo[a] = cols[2 * a].number_input(
                    f"{_AXES[a]} lo", value=float(spec.lo[a]), step=0.1,
                    key=f"stoch_lo_{pname}_{a}", format="%.2f")
                hi[a] = cols[2 * a + 1].number_input(
                    f"{_AXES[a]} hi", value=float(spec.hi[a]), step=0.1,
                    key=f"stoch_hi_{pname}_{a}", format="%.2f")
            try:
                edited[pname] = ks.ToleranceSpec(lo=lo, hi=hi, dist=dist)
            except ValueError as e:
                st.error(f"{pname}: {e}")
                edited[pname] = spec
        fld = ks.ToleranceField(edited)
    if dist == "normal":
        fld = ks.ToleranceField({p: ks.ToleranceSpec(lo=s.lo, hi=s.hi,
                                                     dist="normal")
                                 for p, s in fld.specs.items()})

    # ================= 2 · the acceptance bands ===========================
    st.markdown("###### 2 · The acceptance bands — what still counts as "
                "your car")
    st.caption("Max acceptable |as-built − design| per metric. These are "
               "design decisions, not physics — every band you widen is "
               "yield you buy with tuning envelope.")
    b1, b2, b3, b4, b5 = st.columns(5)
    yspec = ks.YieldSpec(
        camber_bump_deg=b1.number_input("Δcamber @ bump (°)", 0.01, 2.0, 0.25,
                                        0.05, key="stoch_b_camber"),
        bump_steer_deg=b2.number_input("Δbump steer (°)", 0.01, 1.0, 0.10,
                                       0.01, key="stoch_b_steer"),
        rc_height_mm=_units.unum(b3, "ΔRC height (mm)", 0.5, 30.0, 8.0, 'mm', step=0.5, key="stoch_b_rc"),
        scrub_mm=_units.unum(b4, "Δscrub (mm)", 0.5, 15.0, 4.0, 'mm', step=0.5, key="stoch_b_scrub"),
        caster_deg=b5.number_input("Δcaster (°)", 0.05, 2.0, 0.30, 0.05,
                                   key="stoch_b_caster"))

    r1, r2 = st.columns([1, 2])
    n_samples = r1.select_slider("Sampled builds", [1000, 2000, 5000, 10000],
                                 value=5000, key="stoch_n")
    mode = r2.radio("Engine", ["linear (instant, self-pricing)",
                               "full (every sample a nonlinear solve — slow)"],
                    index=0, horizontal=True, key="stoch_mode")
    mode_key = "linear" if mode.startswith("linear") else "full"
    if mode_key == "full" and n_samples > 2000:
        st.warning("Full mode at this sample count is ~15 ms per build — "
                   "expect a coffee. The linear engine prices its own error "
                   "against full solves; start there.")

    if not st.button("🎲 Run the tolerance sweep", key="stoch_run",
                     type="primary"):
        st.info("Declare the field and bands above, then run. The sweep is "
                "deterministic — the same inputs always give the same yield.")
        return

    with st.spinner("Building the sensitivity matrix, sampling the error "
                    "cloud, pricing the linearisation…"):
        try:
            res = ks.stochastic_sweep(hp, fld, yspec, n=int(n_samples),
                                      seed=0, mode=mode_key)
        except ValueError as e:
            st.error(f"Sweep refused: {e}")
            return

    # ================= 3 · the verdict ====================================
    badge, blurb = _VERDICT_BLURB[res.verdict]
    st.markdown(f"## {badge} {res.verdict} — manufacturing yield "
                f"{res.yield_frac*100:.1f}%")
    st.caption(blurb)

    worst_m, worst_f = res.worst_metric()
    dom_lab, dom_share = res.dominant_coord(worst_m)
    m1, m2, m3, m4 = st.columns(4)
    m1.metric("Yield", f"{res.yield_frac*100:.1f} %",
              help="Fraction of the sampled buildable cars whose kinematics "
                   "stay inside every acceptance band.")
    m2.metric("Yield killer", ks._METRIC_LABELS[worst_m].split(" (")[0],
              delta=f"{worst_f*100:.1f}% of builds miss it",
              delta_color="inverse" if worst_f > 0 else "off",
              help="The metric that fails most often across the population.")
    m3.metric("Dominant error source", dom_lab,
              delta=f"{dom_share*100:.0f}% of its variance",
              delta_color="off",
              help="The hardpoint coordinate whose tolerance drives the "
                   "yield-killing metric — the tab to jig FIRST (first-order "
                   "attribution).")
    if res.mode == "linear" and res.verify_agreement is not None:
        m4.metric("Linearisation priced", f"{res.verify_agreement*100:.0f} %",
                  help="Pass/fail agreement between the instant linear engine "
                       "and full nonlinear solves on the verification "
                       "subsample. Below 98% the result demotes itself and "
                       "tells you to run full mode.")
    else:
        m4.metric("Engine", "full nonlinear",
                  help="Every sampled build was a complete solver run.")

    # Provenance: the yield is a modelled number whose trustworthiness is
    # bounded entirely by the declared shop error field above — it is a
    # proactive buildability risk indicator, not a measured pass rate. Say so
    # once, in the shared grade vocabulary, and name the measurement that
    # upgrades it from indicative to measured.
    try:
        from suspension.provenance import confidence_note as _conf
        _cal = ss.get("stochastic_field_measured", False)
        _conf(st, "modelled", calibrated=_cal,
              extra=("first-order sweep over the error field YOU declared — "
                     "the yield inherits that field's accuracy"),
              calibrate_with=("one CMM / FARO-arm pass over ~10 built chassis "
                              "to replace the assumed ± bands with your shop's "
                              "measured spread"))
    except Exception:
        pass

    for w in res.warnings:
        st.warning(w)

    # publish a compact summary for cross-tab consumers / handover
    ss["stochastic_last"] = {
        "yield": res.yield_frac, "verdict": res.verdict, "mode": res.mode,
        "n": res.n, "worst_metric": worst_m, "dominant_coord": dom_lab}

    # ================= 4 · the anatomy ====================================
    tab_metrics, tab_attr, tab_nudge, tab_rx = st.tabs(
        ["Per-metric spread", "Error attribution", "Robust nudge",
         "Alignment prescription (as-built)"])

    with tab_metrics:
        st.caption("The population's deviation from the design intent, per "
                   "metric: the bias E[Δ] an asymmetric field injects before "
                   "the first cut, the 5th–95th percentile spread, and the "
                   "fraction of builds outside each band.")
        rows = []
        for i, m in enumerate(ks.METRICS):
            rows.append({
                "metric": ks._METRIC_LABELS[m],
                "band ±": float(res.bands[i]),
                "bias E[Δ]": float(res.bias[i]),
                "Δ p5": float(res.p05[i]),
                "Δ p95": float(res.p95[i]),
                "fail %": float(res.fail_frac_per_metric[i] * 100.0)})
        st.dataframe(rows, use_container_width=True)

    with tab_attr:
        st.caption("First-order variance share of the yield-killing metric "
                   "per hardpoint coordinate — which tab to jig, which rod "
                   "end to measure-and-sort. Linear attribution, stated as "
                   "such.")
        i = ks.METRICS.index(worst_m)
        labels = res.sens.coord_labels()
        shares = res.attribution[i]
        order = np.argsort(shares)[::-1][:10]
        st.bar_chart({"variance share (%)":
                      {labels[j]: float(shares[j] * 100.0) for j in order}})

    with tab_nudge:
        st.caption("An asymmetric field biases the EXPECTED as-built car off "
                   "the intent. The nudge solves the nominal shift that "
                   "re-centres the whole cloud in the bands — aiming up-wind "
                   "of the weld pull — bounded by how far you allow the "
                   "design to move.")
        freedom = _units.unum(st, "Design freedom per coordinate (± mm)", 0.5, 10.0, 3.0, 'mm', step=0.5, key="stoch_freedom")
        verify_n = st.select_slider("Full-solve verification samples",
                                    [0, 100, 200, 400], value=100,
                                    key="stoch_nudge_verify")
        with st.spinner("Solving and verifying the nudge…"):
            nud = ks.robust_nudge(hp, fld, res, freedom_mm=float(freedom),
                                  n_verify_full=int(verify_n))
        if not nud.ok:
            st.info(nud.reason)
        else:
            st.markdown(f"**{nud.reason}**")
            for pnt, v in sorted(nud.shifts.items()):
                st.markdown(f"- shift **{pnt}** by "
                            f"[{v[0]:+.2f}, {v[1]:+.2f}, {v[2]:+.2f}] mm")
            line = (f"Predicted yield {nud.baseline_yield*100:.1f}% → "
                    f"**{nud.predicted_yield*100:.1f}%** (linear)")
            if nud.verified_yield is not None:
                line += (f"; full-solve verified "
                         f"**{nud.verified_yield*100:.1f}%** against the "
                         "ORIGINAL design intent")
            st.markdown(line + ".")
            if nud.clamped:
                st.warning("Freedom-box clamps on: " + ", ".join(nud.clamped)
                           + " — the ideal re-centring wants more design "
                             "freedom than you allowed.")

        md = ks.render_stochastic_md(res, nud if nud else None,
                                     title=hp_note)
        st.download_button("📥 Download the sweep report (markdown)", md,
                           file_name="stochastic_inversion.md",
                           key="stoch_dl")

    with tab_rx:
        st.caption("The metrology feedback loop: measure the welded chassis "
                   "(calipers or a CMM arm), paste the as-built coordinates, "
                   "declare the adjusters the car actually has, and get the "
                   "shim arithmetic that restores the intent — verified by a "
                   "full re-solve of the shimmed as-built geometry.")
        default_csv = "\n".join(
            f"{p}, {getattr(hp, p)[0]:.2f}, {getattr(hp, p)[1]:.2f}, "
            f"{getattr(hp, p)[2]:.2f}"
            for p in ("upper_front_inner", "upper_rear_inner",
                      "lower_front_inner", "lower_rear_inner"))
        text = st.text_area("As-built coordinates (name, x, y, z — mm, same "
                            "frame and datum as the design; seeded with the "
                            "NOMINAL values, overwrite with measurements)",
                            value=default_csv, height=120, key="stoch_csv")
        as_built, errs = _parse_asbuilt_csv(text, hp)
        for e in errs:
            st.error(e)

        st.markdown("**Adjusters the car actually has**")
        n_adj = st.number_input("How many", 1, 6, 3, 1, key="stoch_nadj")
        adjusters = []
        defaults = [("tie_rod_inner", "z"), ("upper_rear_inner", "x"),
                    ("lower_rear_inner", "x"), ("tie_rod_inner", "y"),
                    ("upper_front_inner", "x"), ("lower_front_inner", "x")]
        for k in range(int(n_adj)):
            a1, a2, a3, a4 = st.columns([2, 1, 1, 1])
            dp, da = defaults[k % len(defaults)]
            pnt = a1.selectbox(f"Adjuster {k+1} point",
                               list(ks.PERTURBABLE_POINTS),
                               index=list(ks.PERTURBABLE_POINTS).index(dp),
                               key=f"stoch_adj_p{k}")
            ax = a2.selectbox("axis", _AXES, index=_AXES.index(da),
                              key=f"stoch_adj_a{k}")
            rng = _units.unum(a3, "± range (mm)", 0.5, 10.0, 3.0, 'mm', step=0.5, key=f"stoch_adj_r{k}")
            stp = _units.unum(a4, "shim step (mm)", 0.0, 2.0, 0.5, 'mm', step=0.05, key=f"stoch_adj_s{k}")
            adjusters.append(ks.Adjuster(pnt, ax, -float(rng), float(rng),
                                         float(stp)))

        if st.button("🔧 Solve the alignment prescription", key="stoch_rx"):
            if not as_built:
                st.error("Paste at least one measured point first.")
            else:
                with st.spinner("Solving the as-built car, then the shims, "
                                "then verifying…"):
                    try:
                        rx = ks.alignment_prescription(hp, as_built,
                                                       adjusters, yspec)
                    except ValueError as e:
                        st.error(f"Prescription refused: {e}")
                        rx = None
                if rx is not None:
                    rb = {"RESTORED": "🟢", "PARTIAL": "🟡",
                          "UNSHIMMABLE": "🔴"}[rx.verdict]
                    st.markdown(f"### {rb} {rx.verdict}")
                    for line in rx.lines():
                        st.markdown(f"- **{line}**")
                    rows = []
                    for i, m in enumerate(ks.METRICS):
                        rows.append({
                            "metric": ks._METRIC_LABELS[m],
                            "band ±": float(rx.bands[i]),
                            "as-built Δ": float(rx.delta_before[i]),
                            "after shims Δ": float(rx.delta_after[i]),
                            "inside band": bool(abs(rx.delta_after[i])
                                                <= rx.bands[i])})
                    st.dataframe(rows, use_container_width=True)
                    if rx.unreachable:
                        st.error("Unreachable with these adjusters: "
                                 + ", ".join(ks._METRIC_LABELS[m]
                                             for m in rx.unreachable)
                                 + " — no declared shim direction moves this "
                                   "metric. Add a different adjuster or "
                                   "accept the residual; more turns of the "
                                   "current ones cannot fix it.")
                    for w in rx.warnings:
                        st.warning(w)
                    st.download_button(
                        "📥 Download the prescription (markdown)",
                        ks.render_prescription_md(rx, title=hp_note),
                        file_name="alignment_prescription.md",
                        key="stoch_rx_dl")

    st.caption("_Independent per-point errors, links built-to-fit, "
               "first-order attribution priced against full solves — the "
               "scope notes live in the module docstring and the report "
               "footer._")
