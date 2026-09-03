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


#  The solve is dense and O(N^2) in MEMORY as well as time, and the app runs in
#  roughly 1 GB. Measured peak RSS on a real solve:
#
#        800 panels   167 MB    0.6 s
#       1200 panels   252 MB    1.9 s
#       1600 panels   363 MB    3.3 s
#       2400 panels   638 MB   11.9 s
#       4000 panels   OOM-killed
#
#  So 1600 is the ceiling offered here: it leaves headroom for Streamlit, the
#  session and a second user, and a ride-height sweep multiplies the time but
#  not the peak memory. 4000 took the app down in production and is gone.
#
#  This mattered more than it looks: the budget used to have no effect at all,
#  because decimation was calling trimesh with the wrong argument inside a bare
#  `except: pass`. Every solve ran the full mesh whatever was selected here.
#  A CAP, NOT A TARGET. The mesh is solved exactly as supplied — see
#  panel_method.py for why decimation was removed. This is the largest mesh the
#  solver will accept, and the memory is what sets it. Measured peak RSS after
#  chunked assembly:
#
#        768 faces    166 MB     0.6 s
#       3072 faces    313 MB    10.1 s
#       4220 faces    426 MB    24.5 s
#
#  Before chunking, 3072 faces cost 973 MB and the app was OOM-killed. 5000 is
#  the ceiling offered here: it leaves room for Streamlit, the session and a
#  second user.
_PANEL_BANDS = {
    "Up to 1,500 triangles (fast)": 1500,
    "Up to 3,000 triangles": 3000,
    "Up to 5,000 triangles (slow)": 5000,
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
                 f"{_MAX_STL_MB} MB limit. Export a coarser STL; the solve "
                 f"decimates to the panel budget anyway, so the extra "
                 f"triangles are discarded.")
        return

    #  Reading a very large STL is itself the memory risk, before any solve.
    #  trimesh holds the full vertex and face arrays, and the app has about a
    #  gigabyte for everything. Say so with the actual number rather than
    #  letting the process get OOM-killed, which is what took the app down
    #  when a real wing arrived at tens of thousands of triangles.
    _tris = None
    try:
        import trimesh as _tm, io as _io
        _probe = _tm.load(_io.BytesIO(raw), file_type="stl", force="mesh")
        _tris = 0 if _probe is None else len(_probe.faces)
    except Exception:                                       # noqa: BLE001
        pass
    if _tris is not None and _tris > 400_000:
        st.error(
            f"{up.name} has {_tris:,} triangles, which is too large to load "
            f"here. Export a coarser STL from SolidWorks — under about "
            f"100,000 is comfortable, and it makes no difference to the "
            f"result because the mesh is decimated to at most "
            f"{max(_PANEL_BANDS.values()):,} panels before solving.")
        return
    if _tris:
        st.caption(f"{_tris:,} triangles — solved as supplied, nothing decimated.")

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
    band = c2[2].selectbox("Mesh size limit", list(_PANEL_BANDS),
                           index=0, key="pm_band",
                           help="Your mesh is solved exactly as supplied — "
                                "nothing is decimated, because reducing a "
                                "closed thin-walled surface corrupts the "
                                "answer. This is the largest mesh accepted; a "
                                "bigger STL is refused rather than altered. "
                                "The solve is dense in time and memory, so a "
                                "finer mesh costs both.")

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

    #  TELL THEM BEFORE THEY SPEND THE TIME.
    #
    #  Panels are point sources at their centroids, so the ground image only
    #  holds while the mean panel is small against the gap to the road. Mean
    #  panel size is ~sqrt(area / N), which means the usable ride height scales
    #  with the SIZE of the part and the budget — and for a full-size undertray
    #  at a realistic 40 mm, the arithmetic asks for roughly 9000 panels, which
    #  does not fit in the memory this app has.
    #
    #  That was previously discovered only by running it: the solve completed,
    #  reported non-convergence, and the reason sat inside a notes string. A
    #  user reasonably concluded their STL was broken. Estimating it up front
    #  from the mesh area costs nothing and turns a dead end into a choice.
    _lo_h = min(heights)
    try:
        import math as _math
        _area = float(_probe.area) if _tris else None
    except Exception:                                       # noqa: BLE001
        _area = None
    if _area:
        _panel_mm = _math.sqrt(_area / _PANEL_BANDS[band]) * 1000.0
        _min_h = _panel_mm / 0.4
        if _lo_h < _min_h:
            _need = int(_area / ((0.4 * _lo_h / 1000.0) ** 2))
            _fits = _need <= max(_PANEL_BANDS.values())
            st.warning(
                f"**This will not resolve at {_lo_h:.0f} mm ride height.** At "
                f"{_PANEL_BANDS[band]:,} panels the mean panel is about "
                f"{_panel_mm:.0f} mm, and the ground image needs panels well "
                f"under the gap to the road — so this budget is trustworthy "
                f"only above roughly **{_min_h:.0f} mm**.\n\n"
                + (f"Raise the budget to about {_need:,} panels, or run at a "
                   f"higher ride height."
                   if _fits else
                   f"Resolving {_lo_h:.0f} mm on a part this size would take "
                   f"about **{_need:,} panels**, which is beyond what this "
                   f"solver can hold in memory. Either run this part at a "
                   f"higher ride height to see the trend, or solve a smaller "
                   f"piece of it \u2014 a floor section or a sidepod resolves "
                   f"down to realistic ride heights at this budget.")
                + "\n\nYou can still run it; the result will be flagged.")

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
                #  GRID CONVERGENCE: SOLVE TWICE, AND BELIEVE NEITHER ALONE.
                #
                #  A single solve reports a number with no indication of
                #  whether it is resolved. On a real undertray this is not a
                #  subtlety: at 1600 / 2400 / 3200 panels the same geometry at
                #  150 mm gave +1.07, -0.07 and +0.03 — three different signs
                #  and three different ride-height trends. The conditioning
                #  guard flagged only the worst two, so the others were
                #  presented as trustworthy.
                #
                #  Re-solving at half the budget and comparing is the standard
                #  check, and it costs about a third more time because the
                #  coarse solve is much cheaper than the fine one. If the two
                #  disagree, the mesh is not resolving the physics and the
                #  number must not be quoted, whatever the residual says.
                _coarse = None
                try:
                    _coarse = PanelMethodModel(
                        PanelParams(max_panels=max(200, _PANEL_BANDS[band] // 2))
                    ).solve(spec)
                except Exception:                          # noqa: BLE001
                    pass
                _gci = None
                if _coarse is not None and _coarse.converged:
                    _den = max(abs(r.c_lift), abs(_coarse.c_lift), 1e-6)
                    _gci = abs(r.c_lift - _coarse.c_lift) / _den
            except PanelMethodUnavailable as exc:
                #  A specific, reportable reason — coarse surface, unreadable
                #  geometry. Say which, rather than "solve failed".
                st.error(f"Could not solve at {h:.0f} mm: {exc}")
                return
            rows.append({
                "ride height (mm)": round(h, 1),
                "grid Δ": ("—" if _gci is None else f"{100*_gci:.0f}%"),
                "resolved": (_gci is not None and _gci < 0.15),
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
    #  "converged" from the solver means the linear system was well-posed.
    #  "resolved" means the answer stops changing when the mesh is refined.
    #  A number needs both, and they are not the same thing.
    ok = [r for r in rows if r["converged"]]
    _unresolved = [r for r in rows if r["converged"] and not r["resolved"]]
    if not ok:
        st.error(
            "No case converged. Most often this is one of three things:\n\n"
            "- **The panel budget is too coarse for the ride height.** Panels "
            "act as point sources at their centroids, so once the mean panel "
            "is bigger than the gap to the road the ground image breaks down. "
            "Raise the budget or raise the ride height.\n"
            "- **The geometry intersects the road plane** — the road is z = 0 "
            "in the STL's own coordinates, so check where your model sits.\n"
            "- **The surface is self-intersecting or not closed.** Re-export "
            "from CAD rather than repairing by hand.")
        if first_note:
            st.info(first_note)
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

    if _unresolved:
        st.error(
            f"**Not grid-converged — do not quote these numbers.** "
            f"{len(_unresolved)} of {len(rows)} case(s) changed by more than "
            f"15% when re-solved at half the panel budget, which means the "
            f"mesh is not resolving the flow. The `grid Δ` column shows how "
            f"much each moved.\n\n"
            f"Raise the panel budget until the column settles, run at a higher "
            f"ride height, or solve a smaller part. A result that shifts with "
            f"resolution is telling you about the mesh, not about the car.")

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

    #  SURFACE THE GRID WARNING INSTEAD OF BURYING IT.
    #
    #  The solver already detects when the mean panel is large relative to the
    #  gap to the road, and says so — but only inside the notes string, which
    #  sat in a collapsed expander. So a user could sweep ride height, watch
    #  C_L move, and never learn the number was not grid-converged. On the
    #  sample undertray at 30 mm ride height, C_L reads -0.020 at 800 panels
    #  and -0.074 at 1600: a factor of three, purely from resolution.
    #
    #  Deltas between geometries at a FIXED budget are still usable. A single
    #  absolute value at a coarse budget is not, and the user has to be told
    #  which of the two they are looking at.
    _note = (first_note or "")
    if "WARNING" in _note:
        st.warning(
            "**This result is not grid-converged.** " + _note.split("[", 1)[-1]
            .rstrip("]") + "\n\nRaise the panel budget, or compare geometries "
            "at a fixed budget rather than quoting the absolute value.")
    elif "large fraction" in _note:
        st.info(
            "**Treat the absolute level as indicative.** " +
            _note.split("[", 1)[-1].rstrip("]") + "\n\nDeltas between "
            "geometries at the same budget are still meaningful; a single "
            "absolute C_L at this resolution is not. If you need the level, "
            "raise the budget until it stops moving.")

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
