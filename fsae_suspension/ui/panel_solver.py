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
#  PANEL BANDS, WITH HONEST TIMING.
#
#  The solve is O(N^3) in the panel count, so the relationship between mesh
#  size and time is steep. Measured on this box (an O(N^3) dense solve):
#
#      triangles    solve time      memory
#        768          0.4 s         ~100 MB
#      3,072          3.3 s         ~300 MB
#      5,120          7.0 s         630 MB
#      5,912         10.7 s         801 MB
#     12,288         57   s        ~3,000 MB
#
#  Interpolating the measured points, 5,000 triangles is about 14 s and 800 MB
#  HERE, so roughly four minutes on the deploy box — and 800 MB of peak on top
#  of the app's ~250 MB baseline is over the 690 MB Streamlit Community Cloud
#  guarantees. The 5,000 band exists because it was asked for, and it is
#  labelled with what it costs. The memory guard in panel_method refuses before
#  the allocation rather than letting the container OOM mid-solve.
#
#  RELABELLED against the deploy box itself. The old figures came from an
#  early run — 60–70 s for a 2,316-face mesh — and were measured BEFORE the
#  near-field pair search stopped being O(N^2). Real numbers now, from a
#  5,996-triangle floor at the 6,000 band: a single attitude with its grid
#  check took 33.6 s (two solves) and a 5-point sweep took 94.7 s (six solves).
#  That is about 16–17 s per solve, not minutes. Any mesh above
#  ~800 faces at a 1,000-panel limit will be refused, so members who upload a
#  real part will almost always land on 2,500 or higher and wait minutes.
#
#  The right answer for the panel method is not a faster box — it is a coarser
#  mesh. A noise-cone or rollhoop has smooth surfaces that are well-resolved by
#  a few hundred faces. Only push higher if the warning says the panels are too
#  coarse to trust.
_PANEL_BANDS = {
    "Up to 800 triangles (<1 s)": 800,
    "Up to 1,500 triangles (~1 s)": 1500,
    "Up to 2,500 triangles (~3 s)": 2500,
    "Up to 5,000 triangles (~10 s, ~600 MB)": 5000,
    "Up to 6,000 triangles (~17 s, ~870 MB)": 6000,
}

_MAX_STL_MB = 60


def _grid_remedy(tris, band, two_file):
    """What to actually DO about a failed grid check, for this exact case.

    The old text said "raise the panel budget until the column settles", which
    is useless advice to someone already on the largest band — and that is
    where a 6,000-triangle part necessarily is. Work out which lever is still
    available and name it, with the numbers.

    Verified that refining is the right lever when it IS available: on a clean
    floor at 70 mm, C_L across 1,088 / 2,104 / 3,768 / 5,912 faces moves 1%,
    1%, 0%. A mesh that keeps moving instead is telling you something about
    the tessellation, not about resolution — which is why the two-file case
    gets different advice.
    """
    cap = _PANEL_BANDS[band]
    biggest = max(_PANEL_BANDS.values())
    lines = []

    if two_file:
        lines.append(
            "**This delta came from your two uploads**, so it measures the "
            "difference between two CAD exports — not resolution alone. Check "
            "first that both are the same part in the same position; a "
            "re-export that moved or changed shape shows up here as a huge "
            "delta and no amount of refining will fix it.")
        lines.append(
            "If they are the same part, bring the coarse export CLOSER to the "
            "fine one — halving the triangles is plenty. A very coarse "
            "comparison mesh fails the check on its own roughness rather than "
            "on the fine mesh being wrong.")
    elif tris and tris <= cap // 2:
        lines.append(
            f"Your mesh is {tris:,} triangles and the check re-solves at "
            f"{cap // 2:,}, so it had nothing coarser to compare against. Use "
            f"the second uploader: export the part again at a looser STL "
            f"tolerance, around {max(200, tris // 2):,} triangles.")
    elif cap < biggest:
        lines.append(
            f"You are on the {cap:,}-triangle band. Export a finer STL and "
            f"move up to the {biggest:,} band — refining is the lever that "
            f"works when the mesh really is too coarse, and it costs seconds, "
            f"not minutes.")
    else:
        lines.append(
            f"You are already on the largest band ({cap:,}), so refining "
            f"further is not available — the memory guard stops the solve "
            f"above roughly 6,000 panels. Three things still help: run at a "
            f"higher ride height (the mesh has to resolve a bigger gap, which "
            f"is easier), cut the part down to just the surface you care "
            f"about so the same triangles cover less area, or accept the "
            f"result as a TREND only and compare it against another geometry "
            f"solved identically.")
    return "\n\n".join(lines)


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
        ["Underfloor channel (floors, undertrays — ground effect)",
         "Vortex lattice (lifting surfaces — wings, dive planes)",
         "Panel method (thickness + circulation — wings, bluff bodies)"],
        horizontal=True, key="aero_solver",
        help="Pick by MECHANISM, not by part. The underfloor model treats the "
             "gap between the floor and the road as a duct and solves "
             "continuity plus Bernoulli — that is how a floor makes "
             "downforce. The vortex lattice carries circulation, which is how "
             "a wing makes lift. The panel method has neither: source panels "
             "model displacement only, so use it where displacement IS the "
             "mechanism. The old label pointed floors at the panel method, "
             "which is why it reported 14 N on a floor making a few hundred.")
    use_vlm = solver.startswith("Vortex")
    use_duct = solver.startswith("Underfloor")


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

    #  Drop the latched duct result when the part or the solver changes —
    #  otherwise a member swaps STLs and reads the previous floor's numbers
    #  under the new filename.
    _ctx = (solver, getattr(up, "name", None), getattr(up, "size", None))
    if st.session_state.get("_duct_ctx") != _ctx:
        st.session_state["_duct_ctx"] = _ctx
        st.session_state.pop("_duct_ran", None)

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
                f"{n} solves plus one for the grid check at "
                f"{_PANEL_BANDS[band]:,} panels. On Streamlit "
                f"Cloud expect roughly {_PANEL_BANDS[band]//80:.0f}–"
                f"{_PANEL_BANDS[band]//60:.0f} s per solve, so "
                f"~{n*_PANEL_BANDS[band]//80//60:.0f}–"
                f"{n*_PANEL_BANDS[band]//60//60:.0f} minutes for the sweep. "
                f"Drop to the lightest band if you just need the trend.")

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

    btn_label = ("Run underfloor solve" if use_duct else
                 "Run vortex-lattice solve" if use_vlm else "Run panel solve")
    _pressed = st.button(btn_label, type="primary",
                         key="duct_run" if use_duct else
                         ("vlm_run" if use_vlm else "pm_run"))

    #  A BUTTON IS TRUE FOR ONE RUN ONLY, WHICH BROKE THE WHAT-IF SLIDERS.
    #
    #  Everything below used to be gated on the button directly. Streamlit
    #  reruns the whole script on any widget change, and on that rerun the
    #  button reads False — so moving a slider inside the what-if expander
    #  returned early and wiped the results that contained the slider. From the
    #  member's side the page just reloaded and nothing they touched had any
    #  effect.
    #
    #  The duct solve is milliseconds, so it re-runs freely once started: latch
    #  a flag on the press and render whenever it is set. The two panel solvers
    #  stay gated on the press itself — they take tens of seconds, and having
    #  them re-fire on every stray widget change is exactly the behaviour that
    #  got the app throttled.
    if use_duct:
        if _pressed:
            st.session_state["_duct_ran"] = True
        if not st.session_state.get("_duct_ran"):
            return
    elif not _pressed:
        return

    # ------------------------------------------------------ underfloor duct --
    #  Self-contained: the duct model answers a different question from the two
    #  panel solvers and returns a different shape of result, so it renders its
    #  own compact view rather than being forced through the coefficient table.
    if use_duct:
        _tmp_d = None
        try:
            from suspension.aero import underfloor as _uf
            with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as _f:
                _f.write(raw)
                _tmp_d = _f.name
            _ride0 = float(heights[0]) if heights else 40.0
            _res = _uf.solve(_tmp_d, _ride0, speed, area)
            _c = st.columns(4)
            _c[0].metric("C_L", f"{_res.c_lift:+.3f}")
            _c[1].metric(f"Downforce at {speed:.0f} m/s",
                         f"{_res.downforce_N:.0f} N")
            _c[2].metric("Inlet / throat area", f"{_res.area_ratio:.2f}")
            _c[3].metric("Diffuser half-angle",
                         f"{_res.diffuser_angle_deg:.1f}°")
            if not _res.attached:
                st.warning(_res.notes)
            else:
                st.caption(_res.notes)

            #  THE ROW YOU ASKED FOR MUST BE IN THE TABLE.
            #
            #  This was a fixed ladder of 80/60/50/40/30/25/20 mm, so a member
            #  who typed 130 saw a headline for 130 and a table starting at 80
            #  — their own setting absent from a table that looked like it
            #  contained it. Put the requested height in as the first row,
            #  mark it, and show the standard ladder below it as what lowering
            #  the car buys.
            _ride_i = int(round(_ride0))
            _hs = [_ride_i] + [h for h in (80, 60, 50, 40, 30, 25, 20)
                               if h < _ride_i]
            if len(_hs) > 2:
                _rows = []
                for _h in _hs:
                    _r = _uf.solve(_tmp_d, float(_h), speed, area)
                    _rows.append({
                        "ride height (mm)": (f"{_h}  ← set"
                                             if _h == _ride_i else str(_h)),
                        "C_L": round(_r.c_lift, 4),
                        "downforce (N)": round(_r.downforce_N, 1),
                        "inlet/throat": round(_r.area_ratio, 2)})
                st.dataframe(_rows, width="stretch", hide_index=True)
                st.caption(
                    f"First row is the {_ride_i} mm you set above and matches "
                    f"the headline; the rest is what lowering the car buys at "
                    f"this geometry.")
            # ---- parametric what-if -------------------------------------
            #  The duct solve is ~0.1 ms, so a 2-D grid costs nothing and
            #  answers the question a team actually has: not "what does this
            #  floor do" but "which way should I move". Re-exporting an STL
            #  per idea is slow enough that people stop asking.
            with st.expander("What-if sweep — throat position vs ride height",
                             expanded=False):
                _sa, _sb = st.columns(2)
                _inl = _sa.slider("Inlet height above throat (mm)",
                                  0, 120, 55, 5, key="uf_inl")
                _exi = _sb.slider("Diffuser exit above throat (mm)",
                                  0, 250, 115, 5, key="uf_exi")
                _tfs = [0.25, 0.35, 0.45, 0.55, 0.65]
                _hs2 = [h for h in (60, 50, 40, 30, 25, 20)]
                _grid = _uf.sweep("throat_frac", _tfs, "ride_height_mm",
                                  [float(h) for h in _hs2],
                                  speed_ms=speed, ref_area_m2=area,
                                  inlet_rise_mm=float(_inl),
                                  exit_rise_mm=float(_exi),
                                  baseline={"throat_frac": 0.45,
                                            "ride_height_mm": 40.0})
                _tbl = []
                for _h in _hs2:
                    _r = {"ride height (mm)": _h}
                    for _t in _tfs:
                        _m = next((g for g in _grid
                                   if g["throat_frac"] == _t
                                   and g["ride_height_mm"] == float(_h)), None)
                        _r[f"throat {int(_t*100)}%"] = (
                            f"{_m['vs_baseline_pct']:+.0f}%"
                            + ("" if _m["attached"] else " ⚠")) if _m else "—"
                    _tbl.append(_r)
                st.dataframe(_tbl, width="stretch", hide_index=True)
                st.caption(
                    "**Change against the 45% / 40 mm cell**, not absolute "
                    "force — the parametric duct is an idealisation of your "
                    "meshed one and runs about 1.5x its magnitude, so the two "
                    "are comparable in direction and ranking but not in "
                    "newtons. For absolute numbers, solve the STL above. ⚠ "
                    "marks a diffuser past the 7° attachment limit, where the "
                    "figure is an upper bound.")

            st.caption(
                "**Screening model.** One-dimensional: no spanwise variation, "
                "no edge vortices, no yaw. It assumes the diffuser stays "
                "attached, which is the assumption most likely to be wrong on "
                "a real car — the angle above is the check. Compare floors "
                "against each other; correlate before quoting absolutes.")
        except Exception as _de:                        # noqa: BLE001
            st.error(f"Underfloor solve failed: {_de}")
        finally:
            if _tmp_d and os.path.exists(_tmp_d):
                os.remove(_tmp_d)
        return

    # ----------------------------------------------------------------- run --
    tmp = None
    tmp_coarse = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as fh:
            fh.write(raw)
            tmp = fh.name
        _coarse_same = False
        if up_coarse is not None:
            _cb = up_coarse.getvalue()
            _coarse_same = (_cb == raw)
            with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as fh:
                fh.write(_cb)
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
                        if (tmp_coarse is not None and _coarse_same):
                            #  SAME FILE TWICE IS NOT A GRID CHECK.
                            #
                            #  Dropping the fine STL into the coarse slot makes
                            #  both solves identical and the column reports 0%
                            #  — the most reassuring output the check has, from
                            #  the one input that proves nothing. Byte-identical
                            #  uploads are refused rather than flattered.
                            _why = ("the coarse slot holds the same file as "
                                    "the fine one, so both solves would use "
                                    "identical triangles. Export the part "
                                    "again at a looser STL tolerance")
                        elif tmp_coarse is not None:
                            #  Same case, same attitude, the OTHER export. This
                            #  is a real second discretisation of the geometry,
                            #  which is the only honest way to do this once
                            #  decimation is off the table.
                            _cspec = replace(spec, geometry_path=tmp_coarse)
                            _coarse = PanelMethodModel(
                                PanelParams(max_panels=_PANEL_BANDS[band])
                            ).solve(_cspec)
                        elif _tris and _tris <= max(200, _PANEL_BANDS[band] // 2):
                            #  THE TAUTOLOGY CASE.
                            #
                            #  The coarse solve runs at half the mesh size
                            #  limit, but the limit is a REFUSAL threshold —
                            #  nothing is decimated. So when the mesh already
                            #  fits under half, both solves use the IDENTICAL
                            #  triangles and return the identical C_L, and the
                            #  column proudly reports 0%. That is a mesh
                            #  compared with itself, not a convergence check,
                            #  and 0% is the most reassuring thing the column
                            #  can say — so it was the worst possible lie.
                            #
                            #  Refuse to report a number rather than report a
                            #  meaningless one. The second uploader is the only
                            #  way to check a mesh this size.
                            _why = (f"this mesh has {_tris:,} triangles and the "
                                    f"check re-solves at "
                                    f"{max(200, _PANEL_BANDS[band] // 2):,}, so "
                                    f"the coarse pass would use the very same "
                                    f"triangles and return the very same "
                                    f"answer. Nothing would be compared")
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

            #  A NUMBER YOU MUST NOT USE SHOULD NOT BE PRINTED.
            #
            #  A diverged case used to sit in the table looking like data with
            #  only an unticked box against it: a floor at 15 mm showed
            #  C_L +5.0414 and downforce -333 N — the sign inverted, three
            #  hundred newtons of lift on a part that makes downforce — and
            #  nothing but an empty checkbox said so. Someone reads the row,
            #  not the checkbox. Blank the values instead and say why below.
            _ok = bool(r.converged)
            row = {
                "ride height (mm)": round(h, 1),
                "C_L": r.c_lift if _ok else None,
                "C_D": r.c_drag if _ok else None,
                "C_side": r.c_side if _ok else None,
                "C_pitch": r.c_pitch if _ok else None,
                "aero balance (front)": (r.aero_balance_front if _ok else None),
                "downforce (N)": (r.downforce_N(1.225, area, speed)
                                  if _ok else None),
                "solver ok": _ok,
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
    ok = [r for r in rows if r["solver ok"]]
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
                       if r["solver ok"] and g is not None and g >= 0.15]
        _unchecked = [r for r, g in zip(rows, _gcis)
                      if r["solver ok"] and g is None]
        _unchecked_why = sorted({w for r, g, w in zip(rows, _gcis, _gci_why)
                                 if r["solver ok"] and g is None and w})
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
    if use_vlm:
        st.caption(
            "**Screening tool.** Compare geometries at identical settings and "
            "matched mesh density, and read the RATIO between them — that "
            "holds to a few percent under refinement, absolute levels do not. "
            "Inviscid: no separation, no wake, no profile drag, so a diffuser "
            "is assumed to stay attached.")
    else:
        st.caption(
            "**Screening tool.** Source panels for thickness, blockage and "
            "skin friction, plus a vortex lattice on the mean camber surface "
            "for circulation — so lift from camber is now represented. On a "
            "bluff body with no camber surface the lattice declines and you "
            "get the displacement answer, which is the right one there. "
            "Compare geometries at matched settings; treat absolute levels as "
            "indicative.")

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
    #  L/D IS ONLY L/D WHEN THE DRAG IS ALL OF THE DRAG.
    #
    #  The lattice reports INDUCED drag and nothing else — no profile drag, no
    #  separation, no base drag. Near the road the image cancels most of the
    #  downwash, so induced drag collapses toward zero and the ratio explodes:
    #  a floor showed C_D = 0.0009 against C_L = 0.64 and the headline read
    #  "L/D 708.65". That is not an efficient floor, it is a missing
    #  denominator, and a member reading 708 has no way to know the number is
    #  three quarters absent. On a real FSAE package L/D is single digits.
    #
    #  So the lattice gets a differently-named metric. The panel method
    #  includes a skin-friction estimate, so its ratio is at least a lower
    #  bound on the real one and keeps the familiar label.
    if use_vlm:
        _ld = ("—" if not best["C_D"]
               else f"{abs(best['C_L'])/best['C_D']:.0f}")
        m[3].metric("L/D induced only", _ld,
                    help="Lift over INDUCED drag. Profile, separation and base "
                         "drag are not modelled, and near the road induced "
                         "drag collapses — so this runs far above the real "
                         "lift-to-drag ratio and is not comparable to a "
                         "tunnel or CFD number. Use it to compare two wings "
                         "against each other, never as an efficiency.")
    else:
        m[3].metric("L/D",
                    "—" if not best["C_D"]
                    else f"{abs(best['C_L'])/best['C_D']:.2f}",
                    help="Includes the flat-plate skin-friction estimate, so "
                         "it is a lower bound on the real ratio.")

    #  A blank balance column is a result, not a missing value, and the
    #  headline row above this view always shows one — so name the difference
    #  here too rather than leaving the reader to reconcile them.
    if not use_vlm and any(r.get("aero balance (front)") is None for r in rows):
        st.caption(
            "`aero balance (front)` is blank where one end lifts and the other "
            "pushes down — the ratio is meaningless there. The headline figure "
            "above is the analytic model, not this solve.")

    #  Every solver reaches this, unlike the grid banners below which are
    #  panel-method only. A lattice that diverges near the road produced no
    #  message at all before this.
    _bad = [r for r in rows if not r["solver ok"]]
    if _bad:
        _hs_bad = ", ".join(f"{r['ride height (mm)']:g} mm" for r in _bad)
        st.error(
            f"**{len(_bad)} of {len(rows)} case(s) did not converge and have "
            f"been blanked** ({_hs_bad}). The solver ran but the answer is not "
            f"physical — near the road an inviscid method with a ground image "
            f"diverges, and the first sign is usually C_L changing sign or "
            f"jumping by an order of magnitude.\n\n"
            f"Raise the ride height until the values return, and read the "
            f"remaining rows. The blanked cases are not a smaller version of "
            f"the right answer; they are not an answer.")

    if _unresolved:
        _worst = max((100.0 * g for r, g in zip(rows, _gcis)
                      if g is not None), default=0.0)
        st.error(
            #  TWO DIFFERENT WORDS FOR TWO DIFFERENT THINGS.
            #
            #  The table's tick meant the LINEAR SOLVE succeeded; this banner
            #  means the MESH does not resolve the flow. Both said
            #  "converged", so the page appeared to contradict itself — ticks
            #  all the way down a column next to a red box saying not
            #  converged. The column is now "solver ok" and this banner only
            #  ever talks about the grid.
            f"**Mesh not resolving the flow — do not quote these numbers.** "
            f"The linear solve succeeded (that is the `solver ok` column); "
            f"what failed is the grid check. {len(_unresolved)} of "
            f"{len(rows)} case(s) moved more than 15% when re-solved on a "
            f"coarser mesh, at {_worst:.0f}% for the worst.\n\n"
            + ("Marginal — a few points over the line, so the trend across "
               "ride heights is probably still telling you something even "
               "though the absolute level is not settled. "
               if _worst < 25.0 else
               "Well over the line, so neither the level nor the trend is "
               "safe to read. ")
            + _grid_remedy(_tris, band, tmp_coarse is not None))

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
