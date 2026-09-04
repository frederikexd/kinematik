# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Aero solver view — panel method (bluff bodies) and vortex lattice (wings).

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

TWO SOLVERS, ONE VIEW
---------------------
The panel method is a source-panel BEM: it models displacement (thickness,
ground effect, attached-flow pressure) but carries no circulation, so it cannot
resolve lift on an isolated wing. The vortex-lattice method solves a horseshoe
lattice over the mean camber surface, enforcing the Kutta condition
geometrically. Each refuses the other's case rather than returning a number
nobody should trust. The user selects the solver explicitly; the view renders
the appropriate controls for each.
"""

from __future__ import annotations

import io
import os
import tempfile
from dataclasses import replace
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
#       1000 faces    157 MB     0.6 s
#       2500 faces   ~250 MB     ~3 s
#       4000 faces   ~430 MB     ~4 s
#
#  The app has roughly 1 GB for Streamlit, the session and EVERY concurrent
#  viewer, and the baseline before any solve is already ~200 MB. So the
#  default is the light band: a team of five clicking at once must still fit.
#
#  Before chunking, 3072 faces cost 973 MB and the app was OOM-killed. 8000 is
#  the ceiling offered here: it leaves room for Streamlit, the session and a
#  second user.
_PANEL_BANDS = {
    "Up to 1,000 triangles (light)": 1000,
    "Up to 2,500 triangles": 2500,
    "Up to 4,000 triangles (heavy)": 4000,
}

_MAX_STL_MB = 60


def _fmt(v, nd=4):
    return "—" if v is None else f"{v:+.{nd}f}"


def render(st, default_area_m2: float = 1.0,
           default_length_m: float = 1.55) -> None:
    """Render the aero solver view — panel method and vortex lattice.

    `default_area_m2` / `default_length_m` come from the aero tab so the
    reference values match what the rest of the tab is already normalising by
    — a coefficient divided by a different area than the map uses is silently
    wrong, which is the exact failure `CaseSpec` documents.
    """
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
    from suspension.aero.vortex_lattice import (VortexLatticeModel,
                                                VortexLatticeUnavailable,
                                                DEFAULT_SPANWISE,
                                                DEFAULT_CHORDWISE)
    from suspension.aero.cfd import CaseSpec, Attitude

    # -------------------------------------------------------- solver picker --
    solver = st.radio(
        "Solver",
        ["Panel method (bluff bodies — floors, undertrays, full car)",
         "Vortex lattice (lifting surfaces — wings, dive planes)"],
        horizontal=True, key="aero_solver",
        help="The panel method models displacement with source panels: "
             "ground effect, attached-flow pressure, bluff bodies. It carries "
             "no circulation and cannot resolve an isolated wing. The vortex "
             "lattice solves a horseshoe lattice over the mean camber surface "
             "with the Kutta condition enforced geometrically — use it for "
             "wings and other lifting surfaces.")
    use_vlm = solver.startswith("Vortex")

    if use_vlm:
        st.markdown(
            '<p class="hint">Horseshoe vortex-lattice solve over the mean '
            'camber surface extracted from your STL, with image vortices for '
            'ground effect. Validated on a rectangular AR = 8 wing: C_L '
            'within 6% of lifting-line theory, span efficiency e = 1.004, '
            'grid-converged under 0.5%. <b>Inviscid and attached</b> — no '
            'stall, no separation, no profile drag. The drag reported is '
            '<b>induced drag only</b>; total drag is a lower bound. Trust '
            'lift, span loading and the ground-effect trend; correlate against '
            'the tunnel or Fluent before quoting absolutes.</p>',
            unsafe_allow_html=True)
    else:
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
            'an isolated lifting surface. Floors, undertrays, sidepods, noses and '
            'full cars are what it is for. A wing is refused up front rather than '
            'returning a number you would have to distrust \u2014 use the vortex '
            'lattice solver for wings.</p>',
            unsafe_allow_html=True)

    # ------------------------------------------------------------ geometry --
    up_label = (
        "Wing or dive-plane geometry (.stl) — a closed solid; the camber surface is extracted automatically"
        if use_vlm else
        "Surface geometry (.stl) — the floor, undertray, sidepods or the whole car"
    )
    up = st.file_uploader(
        up_label, type=["stl"],
        key="vlm_stl" if use_vlm else "pm_stl",
        help=("Export a closed solid from SolidWorks: File ▸ Save As ▸ STL. "
              "The solver slices it spanwise and extracts the mean camber line "
              "at each station — it does not need a clean camber surface from "
              "you. The road is at z = 0 in the STL's own coordinates."
              if use_vlm else
              "SolidWorks: File ▸ Save As ▸ STL. Solved exactly as supplied — "
              "never decimated, so keep it under the panel budget below. The "
              "road is z = 0 in the STL's own coordinates."))

    #  SECOND EXPORT FOR THE GRID CHECK.
    #
    #  The grid check needs the same part at two mesh densities. It used to try
    #  to make the coarse one itself by halving the panel budget, but since
    #  decimation was removed a budget is a REFUSAL threshold, not a target —
    #  so for any mesh bigger than half the budget the coarse solve was refused
    #  and the check silently never ran. The banner then told the member to
    #  "export a coarser STL and solve that too", which the UI gave them no way
    #  to do: one uploader, one file.
    #
    #  So take the second file. Optional — leave it empty and the old
    #  half-budget attempt still runs, which works when the mesh is small
    #  enough for it.
    up_coarse = None
    if not use_vlm and up is not None:
        up_coarse = st.file_uploader(
            "Coarser export of the SAME part (.stl) — optional, enables the "
            "grid-convergence check",
            type=["stl"], key="pm_stl_coarse",
            help=("Same part, roughly half the triangles (Save As ▸ STL ▸ "
                  "Options, raise the tolerances). Both are solved and their "
                  "C_L compared — agreement within a few percent means the "
                  "fine mesh is resolving the flow."))

    if up is None:
        st.info(
            "Drop an STL to run a solve. If you want to see the shape of the "
            "output first, the **Aero map (attitude sweep)** view runs on the "
            "analytic surrogate and needs no geometry.")
        return

    raw = up.getvalue()
    if len(raw) > _MAX_STL_MB * 1024 * 1024:
        st.error(f"{up.name} is {len(raw)/1e6:.0f} MB — over the "
                 f"{_MAX_STL_MB} MB limit. Export a coarser STL.")
        return

    _tris = None
    _probe = None
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
            f"100,000 is comfortable.")
        return
    if _tris:
        st.caption(f"{_tris:,} triangles in the uploaded STL.")

    # ------------------------------------------------------------- controls --
    c = st.columns([1, 1, 1, 1])
    speed = c[0].number_input("Speed (m/s)", 5.0, 60.0, 20.0, 1.0,
                              key="vlm_v" if use_vlm else "pm_v")
    pitch = c[1].number_input("Pitch / rake (°)", -8.0, 8.0, 0.0, 0.25,
                              key="vlm_pitch" if use_vlm else "pm_pitch",
                              help="Positive is nose-up. Rake is captured here.")
    yaw   = c[2].number_input("Yaw (°)", -20.0, 20.0, 0.0, 1.0,
                              key="vlm_yaw" if use_vlm else "pm_yaw")
    roll  = c[3].number_input("Roll (°)", -8.0, 8.0, 0.0, 0.5,
                              key="vlm_roll" if use_vlm else "pm_roll")

    #  MEASURE THE PLANFORM BEFORE ASKING FOR IT.
    #
    #  The lattice extracts span and mean chord from the STL to build itself,
    #  so it already knows the planform area. It then normalised C_L by
    #  `spec.reference_area_m2` — a box that defaulted to the panel method's
    #  whole-car 1.00 m². On a part whose measured planform is 0.84 m² that is
    #  a silent 16% error in every coefficient, with nothing on screen hinting
    #  the two numbers were different.
    #
    #  Measure it, show it, and default to it. Overriding is still allowed —
    #  matching the rest of the aero map matters more than being locally
    #  correct — but now it is a choice rather than an accident.
    _vlm_planform = None
    if use_vlm:
        _pp = None
        try:
            from suspension.aero.vortex_lattice import camber_surface_from_stl
            with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as _fh:
                _fh.write(raw)
                _pp = _fh.name
            _vs, _vc, _va = camber_surface_from_stl(_pp)[1:4]
            _vlm_planform = (_vs, _vc, _va)
        except Exception:                                   # noqa: BLE001
            #  Extraction failing here is not an error worth surfacing: the
            #  solve below will raise VortexLatticeUnavailable with the real
            #  reason. Fall back to the manual defaults and say nothing.
            _vlm_planform = None
        finally:
            if _pp and os.path.exists(_pp):
                os.remove(_pp)

    c2 = st.columns([1, 1, 1])
    _area_default = (float(_vlm_planform[2]) if _vlm_planform
                     else float(default_area_m2))
    area = c2[0].number_input("Reference area A (m²)", 0.05, 5.0,
                              _area_default, 0.01,
                              key="vlm_area" if use_vlm else "pm_area",
                              help=("Measured off your STL. Change it only to "
                                    "match what the rest of the aero tab "
                                    "normalises by."
                                    if _vlm_planform else
                                    "Must match what the rest of the aero tab "
                                    "normalises by, or the coefficients are "
                                    "silently inconsistent with the map."))
    if use_vlm:
        #  The lattice never reads reference_length_m — it reports no pitching
        #  moment. Showing a box that does nothing invited exactly the
        #  confusion above, so show the measured mean chord instead, which is
        #  the number that actually characterises the section.
        length = float(_vlm_planform[1]) if _vlm_planform else float(default_length_m)
        c2[1].metric("Mean chord (measured)",
                     f"{length:.3f} m" if _vlm_planform else "—")
    else:
        length = c2[1].number_input("Reference length (m)", 0.5, 4.0,
                                    float(default_length_m), 0.05,
                                    key="pm_len",
                                    help="Wheelbase, for the pitching-moment "
                                         "coefficient.")

    if _vlm_planform:
        st.caption(
            f"Measured from the STL: span {_vlm_planform[0]:.3f} m x mean "
            f"chord {_vlm_planform[1]:.3f} m = {_vlm_planform[2]:.3f} m² "
            f"planform, aspect ratio "
            f"{_vlm_planform[0] / max(_vlm_planform[1], 1e-9):.2f}.")

    if use_vlm:
        n_span = int(c2[2].number_input(
            "Spanwise panels", 8, 48, int(DEFAULT_SPANWISE), 4,
            key="vlm_span",
            help="Panels across the span. The default 24×6 grid is in the "
                 "converged region — going finer costs O(N²) time and changes "
                 "C_L by under 0.5%."))
        n_chord = int(st.columns([1, 2])[0].number_input(
            "Chordwise panels", 4, 16, int(DEFAULT_CHORDWISE), 2,
            key="vlm_chord",
            help="Panels along the chord. 6 is validated; raising it beyond "
                 "8 rarely changes the answer on a thin FSAE section."))
    else:
        band = c2[2].selectbox("Mesh size limit", list(_PANEL_BANDS),
                               index=0, key="pm_band",
                               help="Largest mesh accepted. Bigger STLs are "
                                    "refused, not decimated — reducing a closed "
                                    "thin-walled surface corrupts the answer. "
                                    "Time and memory scale steeply with this.")

    mode = st.radio("Run", ["Single attitude", "Ride-height sweep"],
                    horizontal=True,
                    key="vlm_mode" if use_vlm else "pm_mode")
    if mode == "Single attitude":
        heights = [st.number_input("Ride height (mm)", 5.0, 200.0, 30.0, 1.0,
                                   key="vlm_h" if use_vlm else "pm_h")]
    else:
        sc = st.columns([1, 1, 1])
        h0 = sc[0].number_input("From (mm)", 5.0, 200.0, 15.0, 1.0,
                                 key="vlm_h0" if use_vlm else "pm_h0")
        h1 = sc[1].number_input("To (mm)", 5.0, 200.0, 60.0, 1.0,
                                 key="vlm_h1" if use_vlm else "pm_h1")
        n = int(sc[2].number_input("Points", 2, 12, 5, 1,
                                   key="vlm_hn" if use_vlm else "pm_hn"))
        lo, hi = min(h0, h1), max(h0, h1)
        heights = [lo + (hi - lo) * i / (n - 1) for i in range(n)]
        #  The caption used to read "roughly half a second each". That was
        #  true of the analytic surrogate and has never been true of the panel
        #  method: a few thousand panels is a dense O(N^3) solve that runs for
        #  tens of seconds on the deploy box. A member reading "half a second"
        #  reasonably set 12 points, which is how a sweep became a twenty
        #  minute run that took the app over its resource limit.
        if use_vlm:
            st.caption(f"{n} lattice solves — fast, well under a second each.")
        else:
            st.caption(
                f"{n} solves at {_PANEL_BANDS[band]:,} panels — tens of "
                f"seconds each, so minutes total. Cost scales with the cube of "
                f"the panel count: halving the budget is ~8x faster.")

    if not use_vlm:
        #  TELL THEM BEFORE THEY SPEND THE TIME.
        _lo_h = min(heights)
        try:
            import math as _math
            _area = float(_probe.area) if _tris else None
        except Exception:                                   # noqa: BLE001
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

    btn_label = "Run vortex-lattice solve" if use_vlm else "Run panel solve"
    if not st.button(btn_label, type="primary",
                     key="vlm_run" if use_vlm else "pm_run"):
        return

    # ----------------------------------------------------------------- run --
    tmp = None
    tmp_coarse = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as fh:
            fh.write(raw)
            tmp = fh.name
        if up_coarse is not None:
            with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as fh:
                fh.write(up_coarse.getvalue())
                tmp_coarse = fh.name

        if use_vlm:
            model = VortexLatticeModel(n_span=n_span, n_chord=n_chord)
        else:
            model = PanelMethodModel(PanelParams(max_panels=_PANEL_BANDS[band]))

        rows, first_note, prov = [], "", None
        #  Grid-check outcome per row, kept alongside `rows` rather than in it
        #  so it never reaches the displayed table. None means THE CHECK COULD
        #  NOT RUN — which is not the same as the check failing, and used to be
        #  reported as though it were. See the banner logic below.
        _gcis = []
        #  Why each unrun check did not run, so the banner can quote the
        #  solver's own words instead of assuming a cause.
        _gci_why = []
        #  The single mesh verdict, computed at the tightest ride height and
        #  reused by every other row in the sweep.
        _sweep_gci, _sweep_why = None, None
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

                _gci, _why = None, None
                #  ONCE PER SWEEP, NOT ONCE PER HEIGHT.
                #
                #  Grid convergence is a property of the MESH — whether the
                #  triangles resolve the flow — not of the attitude. Running it
                #  inside this loop doubled every sweep: a 12-point sweep did
                #  24 dense solves to answer one question 12 times with the
                #  same mesh. At tens of seconds per solve that is what put the
                #  app over its resource limit.
                #
                #  Check at the LOWEST height, where the gap is smallest and
                #  the mesh is most stretched, then carry the verdict across
                #  the sweep. If it resolves at the tightest ride height it
                #  resolves at the looser ones.
                #  First case only. It has to be the first rather than the
                #  lowest, because every later row inherits the verdict and
                #  cannot inherit one that has not been computed yet. The
                #  sweep is built from the low end up, so the first case is
                #  also the tightest gap — the strictest place to check.
                _check_here = (i == 0)
                if not use_vlm and _check_here:
                    #  GRID CONVERGENCE: SOLVE TWICE, AND BELIEVE NEITHER ALONE.
                    #
                    #  Record WHY the check did not run, rather than inferring
                    #  it later. There is more than one way to get here — the
                    #  coarse budget refusing an oversized mesh is the common
                    #  one, but the coarse solve can also come back
                    #  ill-conditioned, or fail for a reason particular to the
                    #  geometry. Guessing a single cause in the banner is how
                    #  the old "changed by more than 15%" message came to be
                    #  wrong; stating a different single cause would repeat the
                    #  mistake with better manners.
                    _coarse = None
                    try:
                        if tmp_coarse is not None:
                            #  Same case, same attitude, the OTHER export. This
                            #  is a real second discretisation of the geometry,
                            #  which is the only honest way to do this once
                            #  decimation is off the table.
                            _cspec = replace(spec, geometry_path=tmp_coarse)
                            _coarse = PanelMethodModel(
                                PanelParams(max_panels=_PANEL_BANDS[band])
                            ).solve(_cspec)
                        else:
                            _coarse = PanelMethodModel(
                                PanelParams(max_panels=max(200, _PANEL_BANDS[band] // 2))
                            ).solve(spec)
                    except Exception as _ce:               # noqa: BLE001
                        _why = str(_ce).strip() or type(_ce).__name__
                    if _coarse is not None and not _coarse.converged:
                        _why = ("the coarse solve ran but came back "
                                "ill-conditioned, so its C_L is not worth "
                                "comparing against")
                    if _coarse is not None and _coarse.converged:
                        _den = max(abs(r.c_lift), abs(_coarse.c_lift), 1e-6)
                        _gci = abs(r.c_lift - _coarse.c_lift) / _den
                    _sweep_gci, _sweep_why = _gci, _why
                elif not use_vlm:
                    #  Inherit the one verdict, so every row is labelled and
                    #  none of them pays for it again.
                    _gci, _why = _sweep_gci, _sweep_why

            except (PanelMethodUnavailable, VortexLatticeUnavailable) as exc:
                st.error(f"Could not solve at {h:.0f} mm: {exc}")
                return

            row = {
                "ride height (mm)": round(h, 1),
                "C_L": r.c_lift, "C_D": r.c_drag,
                "C_side": r.c_side, "C_pitch": r.c_pitch,
                "aero balance (front)": r.aero_balance_front,
                "downforce (N)": r.downforce_N(1.225, area, speed),
                "converged": r.converged,
            }
            if not use_vlm:
                row["grid Δ"] = "—" if _gci is None else f"{100*_gci:.0f}%"
                #  None, not False: an unrun check is unknown, not failed.
                row["resolved"] = None if _gci is None else (_gci < 0.15)
                _gcis.append(_gci)
                _gci_why.append(_why)
            rows.append(row)
            first_note = first_note or (r.notes or "")
            prov = prov or r.provenance
            prog.progress((i + 1) / len(heights),
                          text=f"Solved {i+1} of {len(heights)}")
        prog.empty()
        elapsed = time.time() - t0
    finally:
        if tmp and os.path.exists(tmp):
            os.remove(tmp)
        if tmp_coarse and os.path.exists(tmp_coarse):
            os.remove(tmp_coarse)

    # -------------------------------------------------------------- output --
    ok = [r for r in rows if r["converged"]]
    #  TWO DIFFERENT OUTCOMES, TWO DIFFERENT MESSAGES.
    #
    #  The coarse re-solve runs at half the selected cap. Since decimation was
    #  removed, a cap is a REFUSAL threshold, not a target: any mesh larger
    #  than half the cap makes the coarse solve raise, and _gci comes back
    #  None. That is "we could not check", but it used to fall into the same
    #  bucket as "we checked and it moved 40%", so every mesh between half the
    #  cap and the cap was told its numbers had shifted by more than 15% when
    #  nothing had been compared at all — with an empty `grid Δ` cell as the
    #  only hint. Separate them.
    if not use_vlm:
        _unresolved = [r for r, g in zip(rows, _gcis)
                       if r["converged"] and g is not None and g >= 0.15]
        _unchecked = [r for r, g in zip(rows, _gcis)
                      if r["converged"] and g is None]
        _unchecked_why = sorted({w for r, g, w in zip(rows, _gcis, _gci_why)
                                 if r["converged"] and g is None and w})
    else:
        _unresolved, _unchecked, _unchecked_why = [], [], []

    if not ok:
        if use_vlm:
            st.error(
                "No case converged. Most likely causes:\n\n"
                "- **The STL is not a wing.** The vortex lattice needs a "
                "lifting surface: long in span, short in chord, thin in "
                "section. Bluff bodies (floors, undertrays, full cars) should "
                "use the panel method solver.\n"
                "- **The geometry is an open shell.** Export a closed solid "
                "from SolidWorks so the solver can find upper and lower "
                "surfaces at each spanwise station.\n"
                "- **The section is too thick.** If thickness exceeds half the "
                "chord the geometry is refused — use the panel method instead.")
        else:
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

    #  WHAT THIS IS FOR, IN ONE LINE, WHERE THE NUMBERS ARE.
    #
    #  Both solvers are potential-flow: no separation, no wake, no profile
    #  drag. Absolute levels carry real error; differences between geometries
    #  at the same settings largely do not, because the error is systematic.
    #  That is the screening use, and saying so once beats every caveat further
    #  down being read as boilerplate.
    st.caption(
        "**Screening tool.** Compare geometries at identical settings and "
        "trust the ranking; treat any single absolute number as indicative. "
        "Potential flow — no separation, wake or profile drag.")

    best = ok[0]
    m = st.columns(4)
    m[0].metric("C_L", _fmt(best["C_L"]),
                help="Negative is downforce in this convention.")
    m[1].metric("C_D", _fmt(best["C_D"]),
                help="Induced drag only for the vortex lattice — total drag "
                     "is higher." if use_vlm else None)
    m[2].metric("Downforce at %.0f m/s" % speed,
                "—" if best["downforce (N)"] is None
                else f"{best['downforce (N)']:.0f} N")
    m[3].metric("L/D",
                "—" if not best["C_D"] else f"{abs(best['C_L'])/best['C_D']:.2f}")

    #  A blank balance column is a result, not a missing value, and the
    #  headline row above this view always shows one — so name the difference
    #  here too rather than leaving the reader to reconcile them.
    if not use_vlm and any(r.get("aero balance (front)") is None for r in rows):
        st.caption(
            "`aero balance (front)` is blank where one end lifts and the other "
            "pushes down — the ratio is meaningless there. The headline figure "
            "above is the analytic model, not this solve.")

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

    if _unchecked:
        #  SAY IT ONCE.
        #
        #  The solver's own refusal message already carries the remedy —
        #  export a coarser STL, here is where the setting lives, decimating
        #  is unsafe. Restating that underneath it produced a banner that gave
        #  the same instruction twice and buried the one thing the reader did
        #  not already know: that nothing was compared. So the closing
        #  paragraph is only added when there is no solver message to defer
        #  to; when there is one, it speaks for itself.
        _why_txt = "\n".join(f"- {w}" for w in _unchecked_why)
        st.warning(
            f"**Grid convergence not checked** — unverified, not wrong. "
            f"`grid Δ` is empty for {len(_unchecked)} of {len(rows)} case(s): "
            f"the check needs a second, coarser mesh and none was available.\n\n"
            + ("Drop a coarser export of the same part into the second "
               "uploader above." if tmp_coarse is None else
               "The coarse export you supplied did not solve:\n\n" + _why_txt))

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
        st.plotly_chart(fig, width="stretch",
                        key="vlm_sweep" if use_vlm else "pm_sweep")
        st.caption(
            "If C_L does not become more negative as the car gets closer to "
            "the road, either the road plane is not at z = 0 in your STL or "
            "the geometry is upside down — both are worth checking before "
            "trusting anything else on this page.")

    solver_tag = (f"vortex lattice {n_span}×{n_chord}"
                  if use_vlm else f"panel method {_PANEL_BANDS[band]} panels")
    st.download_button(
        "Download results (.csv)", df.to_csv(index=False).encode(),
        file_name=f"aero_solve_{up.name.rsplit('.',1)[0]}.csv",
        mime="text/csv", key="vlm_dl" if use_vlm else "pm_dl")

    st.caption(f"{len(heights)} case(s) in {elapsed:.1f} s — {solver_tag}.")

    if not use_vlm:
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
        if use_vlm:
            st.markdown(
                "- Horseshoe vortex lattice — **inviscid and attached by "
                "construction**. No stall, no separation, no profile drag.\n"
                "- The drag reported is **induced drag only**. Total drag on a "
                "real wing is always higher — add profile drag from a 2D polar "
                "or a RANS run.\n"
                "- Trust **lift, span loading and the ground-effect trend**. "
                "Correlate against the tunnel or Fluent before quoting an "
                "absolute number.\n"
                "- Ground effect is modelled by image vortices reflected "
                "through z = 0 — the road position in your STL matters.\n"
                "- Validated on a rectangular AR = 8 wing: C_L within 6% of "
                "lifting-line theory, span efficiency e = 1.004.")
        else:
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
