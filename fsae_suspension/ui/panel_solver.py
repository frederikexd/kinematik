# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Panel-method solver view — run the in-house 3D BEM on the team's own STL.

WHY THIS FILE EXISTS
--------------------
The solver (`suspension/aero/panel_method.py`) has been complete, tested and
documented for some time, and the README, the mission briefing and this app's
own hint text all describe it as something a member can run. It was never wired
to a control. An aero lead at a real team read the briefing, went looking for
"the tool to run the model-based simulation for the rear wing and the full
vehicle", could not find it, and spent a while troubleshooting before giving up
— he assumed he was missing something. He was not. There was no button.

That is the worst kind of gap: the capability existed, the documentation
promised it, and the user concluded the fault was theirs.

WHAT THIS VIEW IS
-----------------
The shell only. Upload geometry, choose an attitude or a sweep, run, read the
numbers, download them. Every physical decision stays in the engine; nothing
here computes anything. It follows `ui/run_log.py`: the app hands over what the
view needs and the view imports the rest itself.

The honesty is load-bearing, not decoration. Potential flow is inviscid and
attached by assumption, so it does not capture separation, wake, stall or
vortex shedding. A number from here is a directionally-correct starting point
to correlate against the tunnel or Fluent, and the view says so where the
result is displayed rather than in a footnote nobody reads.
"""

from __future__ import annotations

import io
import os
import tempfile
import time


#  The solve is dense and O(N^2) in memory, so panel count is the cost lever.
#  These bands are chosen so a single case stays interactive and a sweep stays
#  under a coffee: measured ~0.4-0.6 s per case at 768 panels.
_PANEL_BANDS = {
    "Fast (800 panels)": 800,
    "Balanced (2000 panels)": 2000,
    "Fine (4000 panels — slow)": 4000,
}

_MAX_STL_MB = 60


def _fmt(v, nd=4):
    return "—" if v is None else f"{v:+.{nd}f}"


def render(st, default_area_m2: float = 1.0,
           default_length_m: float = 1.55) -> None:
    """Render the panel-solver view.

    `default_area_m2` / `default_length_m` come from the aero tab so the
    reference values match what the rest of the tab is already normalising by
    — a coefficient divided by a different area than the map uses is silently
    wrong, which is the exact failure `CaseSpec` documents.
    """
    st.markdown(
        '<p class="hint">Solve the real flow on <b>your STL</b> — a 3D '
        'source-panel (boundary-element) potential-flow solve with a ground '
        'image, so ground effect comes out of the physics rather than a tuned '
        'constant. Seconds per attitude, so a ride-height sweep is '
        'interactive. This is the fidelity step between the analytic surrogate '
        'and a RANS solve: it captures attached-flow pressure, ground effect '
        'and the downforce trend with rake, and it does <b>not</b> capture '
        'separation, wake, stall or vortex shedding \u2014 which is what your '
        'Fluent deck is for.</p>'
        '<p class="hint"><b>Bluff bodies, not wings.</b> This is a '
        'source-panel method with no circulation (no doublets, no Kutta '
        'condition). It models displacement \u2014 thickness, ground effect, '
        'attached-flow pressure \u2014 but cannot produce trustworthy lift on '
        'an isolated lifting surface, and the linear system goes '
        'ill-conditioned as the upper and lower surfaces approach. Floors, '
        'undertrays, sidepods, noses and full cars are what it is for. A wing '
        'is refused up front rather than returning a number you would have to '
        'distrust \u2014 size wings with a method that models '
        'circulation.</p>',
        unsafe_allow_html=True)

    # ---------------------------------------------------------------- deps --
    try:
        import trimesh                              # noqa: F401
    except Exception:                               # noqa: BLE001
        st.error(
            "This solver needs the `trimesh` package to read your STL. "
            "Add `trimesh` to requirements.txt and redeploy — everything else "
            "in the Aerodynamics tab works without it.")
        return

    from suspension.aero.panel_method import (PanelMethodModel, PanelParams,
                                              PanelMethodUnavailable)
    from suspension.aero.cfd import CaseSpec, Attitude

    # ------------------------------------------------------------ geometry --
    up = st.file_uploader(
        "Surface geometry (.stl) — the floor, undertray, sidepods or the whole car",
        type=["stl"], key="pm_stl",
        help="Export from SolidWorks: File ▸ Save As ▸ STL. Fine resolution "
             "is not needed; the mesh is decimated to the panel budget below. "
             "The road is at z = 0 in the STL's own coordinates.")

    if up is None:
        st.info(
            "Drop an STL to run a solve. If you want to see the shape of the "
            "output first, the **Aero map (attitude sweep)** view runs on the "
            "analytic surrogate and needs no geometry.")
        return

    raw = up.getvalue()
    if len(raw) > _MAX_STL_MB * 1024 * 1024:
        st.error(f"{up.name} is {len(raw)/1e6:.0f} MB — over the "
                 f"{_MAX_STL_MB} MB limit. Decimate it in SolidWorks or "
                 f"export a coarser STL; the solve decimates anyway.")
        return

    # ------------------------------------------------------------- controls --
    c = st.columns([1, 1, 1, 1])
    speed = c[0].number_input("Speed (m/s)", 5.0, 60.0, 20.0, 1.0, key="pm_v")
    pitch = c[1].number_input("Pitch / rake (°)", -8.0, 8.0, 0.0, 0.25,
                              key="pm_pitch",
                              help="Positive is nose-up. Rake is captured here.")
    yaw = c[2].number_input("Yaw (°)", -20.0, 20.0, 0.0, 1.0, key="pm_yaw")
    roll = c[3].number_input("Roll (°)", -8.0, 8.0, 0.0, 0.5, key="pm_roll")

    c2 = st.columns([1, 1, 1])
    area = c2[0].number_input("Reference area A (m²)", 0.05, 5.0,
                              float(default_area_m2), 0.01, key="pm_area",
                              help="Must match what the rest of the aero tab "
                                   "normalises by, or the coefficients are "
                                   "silently inconsistent with the map.")
    length = c2[1].number_input("Reference length (m)", 0.5, 4.0,
                                float(default_length_m), 0.05, key="pm_len",
                                help="Wheelbase, for the pitching-moment "
                                     "coefficient.")
    band = c2[2].selectbox("Panel budget", list(_PANEL_BANDS),
                           index=1, key="pm_band",
                           help="The linear system is dense, so this is the "
                                "cost lever. More panels resolve more of the "
                                "surface and take longer.")

    mode = st.radio("Run", ["Single attitude", "Ride-height sweep"],
                    horizontal=True, key="pm_mode")
    if mode == "Single attitude":
        heights = [st.number_input("Ride height (mm)", 5.0, 200.0, 30.0, 1.0,
                                   key="pm_h")]
    else:
        sc = st.columns([1, 1, 1])
        h0 = sc[0].number_input("From (mm)", 5.0, 200.0, 15.0, 1.0, key="pm_h0")
        h1 = sc[1].number_input("To (mm)", 5.0, 200.0, 60.0, 1.0, key="pm_h1")
        n = int(sc[2].number_input("Points", 2, 12, 5, 1, key="pm_hn"))
        lo, hi = min(h0, h1), max(h0, h1)
        heights = [lo + (hi - lo) * i / (n - 1) for i in range(n)]
        st.caption(f"{n} solves at roughly half a second each.")

    if not st.button("Run panel solve", type="primary", key="pm_run"):
        return

    # ----------------------------------------------------------------- run --
    #  The engine takes a path, so the upload is written to a temp file and
    #  removed afterwards. Nothing is persisted: the team's geometry is theirs.
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as fh:
            fh.write(raw)
            tmp = fh.name

        model = PanelMethodModel(PanelParams(max_panels=_PANEL_BANDS[band]))
        rows, first_note, prov = [], "", None
        prog = st.progress(0.0, text="Solving…")
        t0 = time.time()

        for i, h in enumerate(heights):
            spec = CaseSpec(
                attitude=Attitude(roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw,
                                  ride_height_mm=h, speed_ms=speed),
                geometry_path=tmp, reference_area_m2=area,
                reference_length_m=length)
            try:
                r = model.solve(spec)
            except PanelMethodUnavailable as exc:
                #  A specific, reportable reason — coarse surface, unreadable
                #  geometry. Say which, rather than "solve failed".
                st.error(f"Could not solve at {h:.0f} mm: {exc}")
                return
            rows.append({
                "ride height (mm)": round(h, 1),
                "C_L": r.c_lift, "C_D": r.c_drag,
                "C_side": r.c_side, "C_pitch": r.c_pitch,
                "aero balance (front)": r.aero_balance_front,
                "downforce (N)": r.downforce_N(1.225, area, speed),
                "converged": r.converged,
            })
            first_note = first_note or (r.notes or "")
            prov = prov or r.provenance
            prog.progress((i + 1) / len(heights),
                          text=f"Solved {i+1} of {len(heights)}")
        prog.empty()
        elapsed = time.time() - t0
    finally:
        if tmp and os.path.exists(tmp):
            os.remove(tmp)

    # -------------------------------------------------------------- output --
    ok = [r for r in rows if r["converged"]]
    if not ok:
        st.error("No case converged. The geometry is probably too coarse or "
                 "self-intersecting — try a finer STL export.")
        return

    best = ok[0]
    m = st.columns(4)
    m[0].metric("C_L", _fmt(best["C_L"]),
                help="Negative is downforce in this convention.")
    m[1].metric("C_D", _fmt(best["C_D"]))
    m[2].metric("Downforce at %.0f m/s" % speed,
                "—" if best["downforce (N)"] is None
                else f"{best['downforce (N)']:.0f} N")
    m[3].metric("L/D",
                "—" if not best["C_D"] else f"{abs(best['C_L'])/best['C_D']:.2f}")

    import pandas as pd
    df = pd.DataFrame(rows)
    st.dataframe(df, width="stretch", hide_index=True)

    if len(ok) > 1:
        import plotly.graph_objects as go
        fig = go.Figure(go.Scatter(
            x=[r["ride height (mm)"] for r in ok],
            y=[r["C_L"] for r in ok], mode="lines+markers",
            marker=dict(size=9)))
        fig.update_layout(
            title="C_L vs ride height (more negative = more downforce)",
            xaxis_title="ride height (mm)", yaxis_title="C_L",
            height=380, paper_bgcolor="rgba(0,0,0,0)",
            plot_bgcolor="rgba(0,0,0,0)", font=dict(color="#cdd6df", size=11),
            margin=dict(l=0, r=0, t=40, b=0))
        st.plotly_chart(fig, width="stretch", key="pm_sweep")
        st.caption(
            "If C_L does not become more negative as the car gets closer to "
            "the road, either the road plane is not at z = 0 in your STL or "
            "the geometry is upside down — both are worth checking before "
            "trusting anything else on this page.")

    st.download_button(
        "Download results (.csv)", df.to_csv(index=False).encode(),
        file_name=f"panel_solve_{up.name.rsplit('.',1)[0]}.csv",
        mime="text/csv", key="pm_dl")

    st.caption(f"{len(heights)} case(s) in {elapsed:.1f} s at "
               f"{_PANEL_BANDS[band]} panels.")

    #  Provenance and the honest caveat sit WITH the number, not in a footnote.
    with st.expander("What these numbers are, and what they are not",
                     expanded=False):
        if prov is not None:
            st.write(f"**Fidelity:** {getattr(prov, 'fidelity', 'POTENTIAL')} "
                     f"· **correlated:** {getattr(prov, 'is_correlated', False)}")
        st.markdown(
            "- Potential flow is **inviscid and attached by assumption**. No "
            "separation, no real wake, no stall, no vortex shedding.\n"
            "- Trust **deltas between geometries** far more than absolute "
            "levels. Comparing two floors is what this is for; quoting an "
            "absolute C_L to a design judge is not.\n"
            "- Correlate against the tunnel or a Fluent run before reporting "
            "an absolute number. The **ANSYS run-log consolidation** view "
            "takes those runs and turns them into one defensible coefficient "
            "per operating point.\n"
            "- Ground effect is an image system reflected through z = 0, so "
            "the road position in your STL matters.")
        if first_note:
            st.info(first_note)
