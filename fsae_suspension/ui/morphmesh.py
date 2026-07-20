# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#
#  ui/morphmesh.py — the 🕸️🔩 MorphMesh tab (structural auto-synthesizer)
# ============================================================================
"""
The tab that GROWS a bracket instead of asking anyone to sketch one — and lets
the shop floor veto the growth. It hooks straight into the transient results
Ghost Topology already solved, condenses the shifting member force history into
a LOAD FAN (direction × amplitude × exposure), grows a minimum-compliance
topology against the whole fan at once, then measures the finished shape
against the declared fabrication limits of the team's own shop class (the same
hand_weld / jig_weld / cnc names Stochastic Inversion presets carry). Ribs the
mill or the welder cannot hold are REJECTED — the growth is forced coarser and
the mass/stiffness premium of buildability is PRINTED, not hidden.

What it does, in order:
  1. runs the chosen transient event and audits the corner through Ghost
     Topology (deflected geometry, member force at every instant);
  2. builds the load fan for the picked member's chassis-side tab — every
     distinct direction the force sweeps through, weighted by exposure;
  3. grows a SIMP topology on the declared plate domain against all fan cases
     simultaneously (compliance-weighted, filtered, projected);
  4. audits the binarised shape against the shop's minimum-rib and
     HAZ rules by morphological opening — REJECTS and re-grows coarser until
     the shape passes or the volume budget can't satisfy the shop;
  5. screens the survivor at peak fan loads (von Mises FoS vs the standing
     1.5-on-yield rule) and ships the shape as cells CSV, outline segments,
     JSON summary, and a markdown report.

All physics lives in suspension/morphmesh.py (which joins ghost_topology,
kinematics, and the stochastic shop classes). This module only orchestrates
and draws (see ui/__init__.py rules).

Session keys used:
    hardpoints        the live hardpoint dict, if the kinematics tab set one
    morphmesh_last    summary of the last growth (for handover)
    morphmesh_result  the last MorphResult (so download reruns don't re-solve)
"""

from __future__ import annotations


_MANEUVERS = {
    "Step steer (J-turn)": "step_steer",
    "Snap oversteer + countersteer": "snap_oversteer",
    "Brake → throttle transition": "brake_to_throttle",
    "Curb strike": "curb_strike",
}

_MEMBERS = {
    "LF": "LCA front (lower wishbone, front leg)",
    "LR": "LCA rear (lower wishbone, rear leg)",
    "UF": "UCA front (upper wishbone, front leg)",
    "UR": "UCA rear (upper wishbone, rear leg)",
    "TR": "tie rod",
    "PR": "pushrod (rocker-side tab)",
}

_SHOPS = {
    "Hand-welded (the club-garage default)": "hand_weld",
    "Jig-welded (fixtured, TIG)": "jig_weld",
    "CNC-machined (billet / router)": "cnc",
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


def _draw_morph(res, dom):
    """The grown shape: density field, pixel-exact outline, stress-path
    arrows, anchors, bores, and the load fan drawn at the loaded bore."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    h = dom.h_mm
    W, H = dom.width_mm, dom.height_mm
    fig, ax = plt.subplots(figsize=(7.2, 7.2 * min(1.4, H / max(W, 1e-9))))

    # 1 · the density field (what the optimiser grew)
    ax.imshow(res.density, origin="lower", extent=(0, W, 0, H),
              cmap="Greys", vmin=0.0, vmax=1.0, interpolation="nearest")

    # 2 · the pixel-exact outline the CAD seat traces
    for x1, y1, x2, y2 in res.outline_segments(h):
        ax.plot([x1, x2], [y1, y2], color="#e08a3c", lw=1.0, solid_capstyle="butt")

    # 3 · the stress-path map — exposure-weighted principal directions
    mag = res.stress_mag
    if np.any(mag > 0):
        ny, nx = mag.shape
        step = max(1, int(round(4.0 / h)))          # one arrow ≈ every 4 mm
        ys, xs = np.mgrid[0:ny:step, 0:nx:step]
        m = mag[ys, xs]
        keep = m > 0.05 * float(mag.max())
        if np.any(keep):
            cx = (xs[keep] + 0.5) * h
            cy = (ys[keep] + 0.5) * h
            d = res.stress_dir[ys, xs][keep]
            s = m[keep] / float(mag.max())
            ax.quiver(cx, cy, d[:, 0] * s, d[:, 1] * s,
                      color="#4c78a8", alpha=0.75, pivot="mid",
                      scale=nx / (2.2 * step), width=0.004,
                      headwidth=0, headlength=0, headaxislength=0)

    # 4 · anchors + bores
    if dom.anchor == "bottom_edge":
        ax.plot([0, W], [0, 0], color="#d1495b", lw=3.5,
                solid_capstyle="butt", label="weld line (fixed)")
        if res.limits.haz_mm > 0:
            ax.axhspan(0, res.limits.haz_mm, color="#d1495b", alpha=0.08)
            ax.text(1.5, res.limits.haz_mm - 1.0, "HAZ", fontsize=7,
                    color="#d1495b", va="top")
    else:
        for (bx, by, br) in dom.anchor_bores:
            ax.add_patch(plt.Circle((bx, by), br, fill=False,
                                    color="#d1495b", lw=2.0))
            ax.plot([bx], [by], "x", color="#d1495b", ms=6)
    cx0, cy0, r0 = dom.load_bore
    ax.add_patch(plt.Circle((cx0, cy0), r0, fill=False, color="#2a9d8f", lw=2.0))

    # 5 · the load fan, drawn at the loaded bore
    if res.fan is not None and res.fan.cases:
        Fmax = max(c.F_N for c in res.fan.cases) or 1.0
        L = 0.22 * min(W, H)
        for c in res.fan.cases:
            v = c.dir2 / (np.linalg.norm(c.dir2) or 1.0)
            ln = L * (0.35 + 0.65 * c.F_N / Fmax)
            ax.annotate("", xy=(cx0 + v[0] * ln, cy0 + v[1] * ln),
                        xytext=(cx0, cy0),
                        arrowprops=dict(arrowstyle="-|>", lw=1.0 + 2.5 * c.weight,
                                        color="#2a9d8f", alpha=0.9))

    ax.set_xlim(-0.02 * W, 1.02 * W)
    ax.set_ylim(-0.04 * H, 1.04 * H)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title(f"{dom.name} — grown against {len(res.fan.cases) if res.fan else 0} "
                 "fan case(s)", fontsize=10)
    fig.tight_layout()
    return fig


def render():
    import numpy as np
    import streamlit as st
    from suspension import ghost_topology as gt
    from suspension import morphmesh as mmx
    from suspension import transient as tr
    from suspension.compliance import CompliantCorner

    ss = st.session_state

    st.subheader("🕸️🔩 MorphMesh — grow the bracket, let the shop veto it")
    st.caption(
        "Big-software generative design grows beautiful organic parts against "
        "a static load arrow — then the shop rejects them, because a transient "
        "event never loads a tab from one direction, and a hand welder cannot "
        "hold a 1.5 mm rib next to a bead. This tab condenses the SHIFTING "
        "force history Ghost Topology already solved into a load fan, grows a "
        "topology against the whole fan at once, and lets the declared "
        "fabrication limits of your own shop class REJECT ribs the floor "
        "can't build — re-growing coarser and printing the mass/stiffness "
        "premium of buildability. A screening shape for the CAD seat and the "
        "FEA queue, not a certified part — the scope notes are in the module "
        "docstring and the report footer.")

    hp, hp_note = _hardpoints_from_session(ss)
    st.caption(f"Geometry: {hp_note}.")

    # ---------------- 1 · the event + the member ---------------------------
    c1, c2, c3, c4 = st.columns([3, 2, 4, 3])
    with c1:
        man_label = st.selectbox("Transient event (the load history)",
                                 list(_MANEUVERS.keys()), key="mmx_maneuver")
    with c2:
        corner = st.selectbox("Corner", ["FR", "FL", "RR", "RL"],
                              key="mmx_corner",
                              help="A left turn (+steer) loads the RIGHT side. "
                                   "The engine applies the body→corner sign "
                                   "mapping for whichever corner you pick.")
    with c3:
        mem_label = st.selectbox("Member whose chassis tab we grow",
                                 list(_MEMBERS.values()), key="mmx_member")
        member = [k for k, v in _MEMBERS.items() if v == mem_label][0]
    with c4:
        n_samples = st.slider("Audited instants", 8, 32, 14, key="mmx_n",
                              help="How many instants of the event feed the "
                                   "load fan. Load extremes always included.")

    # ---------------- 2 · the plate domain ---------------------------------
    with st.expander("The plate the shape grows inside", expanded=False):
        preset = st.radio("Domain preset",
                          ["Chassis tab (welded, bore up top)",
                           "Pivot bracket (bolted bellcrank web)"],
                          key="mmx_preset", horizontal=True)
        p1, p2, p3, p4 = st.columns(4)
        wdt = p1.number_input("Width (mm)", 30.0, 200.0,
                              60.0 if preset.startswith("Chassis") else 90.0,
                              5.0, key="mmx_w")
        hgt = p2.number_input("Height (mm)", 30.0, 200.0,
                              80.0 if preset.startswith("Chassis") else 70.0,
                              5.0, key="mmx_h")
        thk = p3.number_input("Thickness (mm)", 1.5, 12.0,
                              4.0 if preset.startswith("Chassis") else 5.0,
                              0.5, key="mmx_t")
        bore = p4.number_input("Load bore r (mm)", 3.0, 12.0, 5.0, 0.5,
                               key="mmx_bore")
        m1, m2, m3 = st.columns(3)
        material = m1.selectbox("Material", ["Steel 4130", "Steel mild",
                                             "Aluminium 6061", "Aluminium 7075",
                                             "Titanium Ti-6Al-4V"],
                                index=(0 if preset.startswith("Chassis") else 3),
                                key="mmx_mat")
        yield_default = {"Steel 4130": 460.0, "Steel mild": 250.0,
                         "Aluminium 6061": 276.0, "Aluminium 7075": 503.0,
                         "Titanium Ti-6Al-4V": 880.0}[material]
        yld = m2.number_input("Yield (MPa)", 100.0, 1200.0, yield_default, 5.0,
                              key="mmx_yield")
        h_mm = m3.select_slider("Mesh cell (mm)", options=[1.0, 1.5, 2.0, 2.5],
                                value=1.5, key="mmx_hmm",
                                help="Element size of the growth grid. 1.5 mm "
                                     "is the interactive sweet spot; 1.0 mm "
                                     "roughly quadruples the solve time.")

    # ---------------- 3 · the shop's veto ----------------------------------
    with st.expander("Fabrication limits — the shop's veto", expanded=True):
        st.caption(
            "The same shop-class names Stochastic Inversion presets carry. "
            "min rib = web floor + 2× positional accuracy (each cut edge lands "
            "within ±u); the HAZ rule is a stated screening heuristic, not a "
            "welding simulation. Every number is a knob — declare YOUR floor.")
        shop_label = st.selectbox("Shop class", list(_SHOPS.keys()),
                                  key="mmx_shop")
        seeded = None
        try:
            seeded = __import__("suspension.morphmesh", fromlist=["x"]) \
                .FabricationLimits.from_shop(_SHOPS[shop_label])
        except Exception:
            pass
        f1, f2, f3 = st.columns(3)
        rib = f1.number_input("Min rib (mm)", 0.5, 20.0,
                              float(seeded.min_rib_mm) if seeded else 5.0, 0.5,
                              key=f"mmx_rib_{_SHOPS[shop_label]}")
        haz = f2.number_input("HAZ band (mm)", 0.0, 25.0,
                              float(seeded.haz_mm) if seeded else 8.0, 1.0,
                              key=f"mmx_haz_{_SHOPS[shop_label]}",
                              help="Measured up from the weld line. 0 disables "
                                   "the zone (bolted / machined brackets).")
        ribh = f3.number_input("Min rib in HAZ (mm)", 0.5, 30.0,
                               float(seeded.min_rib_haz_mm) if seeded else 8.0,
                               0.5, key=f"mmx_ribh_{_SHOPS[shop_label]}")

    # ---------------- 4 · the growth budget --------------------------------
    with st.expander("Growth budget & link tubes", expanded=False):
        g1, g2, g3 = st.columns(3)
        volfrac = g1.slider("Volume budget", 0.2, 0.7, 0.4, 0.05,
                            key="mmx_vf",
                            help="Fraction of the active plate the shape may "
                                 "keep. The optimiser strips the rest.")
        max_iter = g2.slider("Iterations per β step", 10, 40, 22,
                             key="mmx_iter")
        n_cases = g3.slider("Fan cases (max)", 2, 6, 4, key="mmx_ncases",
                            help="Direction-static events collapse to fewer "
                                 "cases on their own.")
        t1, t2, t3, t4 = st.columns(4)
        od = t1.number_input("Link OD (mm)", 8.0, 30.0, 19.05, 0.05,
                             key="mmx_od")
        wall = t2.number_input("Wall (mm)", 0.5, 3.0, 0.9, 0.05,
                               key="mmx_wall")
        tmat = t3.selectbox("Tube material", ["Steel 4130", "Steel mild",
                                              "Aluminium 6061",
                                              "Aluminium 7075",
                                              "Titanium Ti-6Al-4V"],
                            key="mmx_tmat")
        tyld = t4.number_input("Tube yield (MPa)", 100.0, 1200.0,
                               {"Steel 4130": 460.0, "Steel mild": 250.0,
                                "Aluminium 6061": 276.0,
                                "Aluminium 7075": 503.0,
                                "Titanium Ti-6Al-4V": 880.0}[tmat], 5.0,
                               key="mmx_tyld")

    # ---------------- 5 · run ----------------------------------------------
    run = st.button("Grow the shape", type="primary", key="mmx_run")
    if not run and "morphmesh_result" not in ss:
        st.info("Pick the event, the member, and your shop class, then press "
                "grow. A transient audit plus a few hundred small plane-stress "
                "solves — under a minute at the default mesh, not an "
                "optimisation-cluster queue.")
        return

    if run:
        with st.spinner("Auditing the transient, condensing the load fan, "
                        "growing the topology, and letting the shop measure "
                        "it…"):
            try:
                res_tr = tr.run_maneuver(None, kind=_MANEUVERS[man_label])
                if not res_tr.ok:
                    st.error("Transient run flagged itself failed: "
                             + "; ".join(res_tr.warnings[:3]))
                    return
                # build the GhostCorner exactly like the Ghost Topology tab does
                cc = CompliantCorner.uniform_tube(hp, material=tmat,
                                                  od_mm=od, wall_mm=wall)
                params = tr.TransientParams.from_vehicle(None)
                k_wheel = (params.k_wheel_front if corner in ("FL", "FR")
                           else params.k_wheel_rear) / 1000.0
                Fz_static = float(tr.TransientSolver().static_corner_loads()[
                    {"FL": 0, "FR": 1, "RL": 2, "RR": 3}[corner]])
                track = (params.track_front if corner in ("FL", "FR")
                         else params.track_rear) * 1000.0
                gc = gt.GhostCorner(
                    cc, gt.uniform_sections(od_mm=od, wall_mm=wall,
                                            material=tmat, yield_MPa=tyld),
                    wheel_rate_N_per_mm=k_wheel, Fz_static_N=Fz_static,
                    track_mm=track)
                audit = gt.ghost_audit_transient(gc, res_tr, corner=corner,
                                                 n_samples=int(n_samples))

                if preset.startswith("Chassis"):
                    dom = mmx.PlateDomain.chassis_tab(
                        width_mm=wdt, height_mm=hgt, bore_r_mm=bore,
                        thickness_mm=thk, material=material, yield_MPa=yld,
                        h_mm=float(h_mm))
                else:
                    dom = mmx.PlateDomain.pivot_bracket(
                        width_mm=wdt, height_mm=hgt, bore_r_mm=bore,
                        thickness_mm=thk, material=material, yield_MPa=yld,
                        h_mm=float(h_mm))
                limits = mmx.FabricationLimits(
                    process=_SHOPS[shop_label], min_rib_mm=rib, haz_mm=haz,
                    min_rib_haz_mm=(ribh if haz > 0 else rib),
                    accuracy_mm=(seeded.accuracy_mm if seeded else 0.5),
                    web_floor_mm=(seeded.web_floor_mm if seeded else 2.0))

                res = mmx.morph_from_audit(
                    audit, hp, member=member, dom=dom, limits=limits,
                    n_cases=int(n_cases), volfrac=float(volfrac),
                    max_iter=int(max_iter))
            except Exception as err:      # noqa: BLE001 — surface, don't crash
                st.error(f"MorphMesh failed: {err}")
                return
        ss["morphmesh_result"] = (res, dom, member, corner,
                                  _MANEUVERS[man_label])

    res, dom, member, corner, kind = ss["morphmesh_result"]
    _show_results(st, np, res, dom, member, corner, kind)


def _show_results(st, np, res, dom, member, corner, kind):
    from suspension import morphmesh as mmx

    # ---------------- 6 · the verdict --------------------------------------
    icon = {"FORGEABLE": "🟢", "COARSENED": "🟡", "UNBUILDABLE": "🔴",
            "LOAD_STARVED": "⚪", "SOLVER_LIMITED": "🔴"}.get(res.verdict, "⚪")
    st.markdown(f"## {icon} {res.verdict} — {dom.name}, member {member}, "
                f"corner {corner}")
    st.caption(mmx._VERDICT_LINES.get(res.verdict, ""))

    if res.verdict in ("LOAD_STARVED", "SOLVER_LIMITED"):
        for f in res.findings:
            st.error(f.get("message", ""))
        for w in res.warnings:
            st.warning(w)
        return

    n_rej = sum(1 for r in res.rounds if not r.accepted)
    a, b, c, d = st.columns(4)
    a.metric("Mass", f"{res.mass_g:.0f} g",
             help="Binarised shape × thickness × density of the plate "
                  "material.")
    b.metric("Worst FoS (peak fan)", f"{res.fos:.2f}"
             if np.isfinite(res.fos) else "—",
             help="von Mises vs yield on the finished binary shape, worst "
                  f"across cases — governed by the {res.fos_case_deg:.0f}° "
                  f"case. The standing rule is {mmx._FOS_RULE}.")
    c.metric("Shop rejections", f"{n_rej}",
             help="Growth attempts REJECTED by the declared fabrication "
                  "limits before one passed.")
    d.metric("FE solves", f"{res.n_solves}",
             help="Small plane-stress factorisations behind this shape.")

    if res.coarsen_premium:
        prem = res.coarsen_premium
        st.warning(
            f"**The premium of buildability**: the shop's veto cost "
            f"**{prem['d_compliance_frac'] * 100:.1f}% compliance** (stiffness "
            f"given up vs the first, unbuildable growth) for "
            f"**+{prem['d_filter_mm']:.1f} mm** of enforced feature size. "
            "That number is the argument for a jig — or the receipt for "
            "keeping the hand welder.")

    if np.isfinite(res.fos) and res.fos < mmx._FOS_RULE and \
            res.suggested_thickness_mm:
        st.error(
            f"The finished shape screens at FoS {res.fos:.2f} < "
            f"{mmx._FOS_RULE} at peak fan loads. Linear plane-stress scaling "
            f"suggests **{res.suggested_thickness_mm:.1f} mm** plate to make "
            "the rule — or hand the shape to the FEA queue.")

    # ---------------- 7 · the picture --------------------------------------
    st.markdown("#### The grown shape")
    st.caption("Grey = grown density. Orange = the pixel-exact outline the CAD "
               "seat traces. Blue whiskers = the exposure-weighted stress-path "
               "map (principal direction × magnitude). Green arrows = the load "
               "fan at the bore; red = the fixed anchor and its HAZ band.")
    fig = _draw_morph(res, dom)
    st.pyplot(fig, clear_figure=True)

    # Provenance: the shape is a modelled screening output; its buildability
    # verdict rides on the declared limits, its FoS on a plane-stress screen.
    try:
        from suspension.provenance import confidence_note as _conf
        _conf(st, "modelled", calibrated=False,
              extra=("multi-case SIMP growth + morphological buildability "
                     "audit against declared shop limits — a screening shape"),
              calibrate_with=("a test coupon per shop class (measure the "
                              "thinnest rib your welder actually holds) and "
                              "a full-fidelity FEA pass on the exported "
                              "outline"))
    except Exception:
        pass

    # ---------------- 8 · the load fan -------------------------------------
    st.markdown("#### The load fan")
    st.caption("The shifting member force history condensed into distinct "
               "directions. Weight = share of the event's load exposure; "
               "F = the peak amplitude seen inside that direction group.")
    if res.fan is not None and res.fan.cases:
        st.dataframe([{
            "direction (°)": f"{cse.angle_deg:.1f}",
            "peak F (N)": f"{cse.F_N:.0f}",
            "exposure": f"{cse.weight * 100:.0f}%",
            "instants": cse.n_instants,
            "window (ms)": f"{cse.t_lo_s * 1000:.0f}–{cse.t_hi_s * 1000:.0f}",
        } for cse in res.fan.cases], use_container_width=True, hide_index=True)
        if res.fan.span_deg > 1.0:
            st.caption(f"Fan span {res.fan.span_deg:.0f}° — a single static "
                       "load arrow would have missed "
                       f"{max(0.0, 1.0 - max(cse.weight for cse in res.fan.cases)) * 100:.0f}% "
                       "of the event's load exposure.")

    # ---------------- 9 · the rejection ledger -----------------------------
    st.markdown("#### The shop's ledger")
    st.caption("Every growth attempt, what the morphological audit measured, "
               "and whether the shop signed it. viol = share of the shape "
               "thinner than the declared minimum rib (bulk / HAZ).")
    st.dataframe([{
        "round": i + 1,
        "enforced feature (mm)": f"{r.filter_mm:.1f}",
        "iterations": r.iterations,
        "compliance (N·mm)": f"{r.compliance:.2f}",
        "vol frac": f"{r.volfrac:.3f}",
        "viol bulk": f"{r.audit.get('viol_frac', 0.0) * 100:.1f}%",
        "viol HAZ": f"{r.audit.get('viol_haz_frac', 0.0) * 100:.1f}%",
        "verdict": "✅ signed" if r.accepted else "❌ REJECTED",
    } for i, r in enumerate(res.rounds)], use_container_width=True,
        hide_index=True)

    for f in res.findings:
        sev = f.get("severity", "info")
        line = f.get("message", "")
        (st.error if sev == "critical" else
         st.warning if sev == "warning" else st.info)(line)
    for w in res.warnings:
        st.warning(w)

    # ---------------- 10 · the deliverables --------------------------------
    st.markdown("#### Ship the shape")
    st.caption("Cells as data, the outline a CAD seat traces, the audit "
               "summary, and the report for the design-review binder.")
    segs = res.outline_segments(dom.h_mm)
    seg_csv = "x1_mm,y1_mm,x2_mm,y2_mm\n" + "\n".join(
        f"{a1:.2f},{b1:.2f},{a2:.2f},{b2:.2f}" for a1, b1, a2, b2 in segs)
    md = mmx.render_morph_md(res)
    d1, d2, d3, d4 = st.columns(4)
    d1.download_button("Cells (.csv)", res.cells_csv(dom.h_mm),
                       file_name=f"morphmesh_{member}_{corner}.csv",
                       mime="text/csv", key="mmx_dl_cells")
    d2.download_button("Outline (.csv)", seg_csv,
                       file_name=f"morphmesh_outline_{member}_{corner}.csv",
                       mime="text/csv", key="mmx_dl_out")
    d3.download_button("Summary (.json)", res.to_json(),
                       file_name=f"morphmesh_{member}_{corner}.json",
                       mime="application/json", key="mmx_dl_json")
    d4.download_button("Report (.md)", md,
                       file_name=f"morphmesh_{member}_{corner}.md",
                       mime="text/markdown", key="mmx_dl_md")

    st.session_state["morphmesh_last"] = {
        "verdict": res.verdict, "member": member, "corner": corner,
        "event": kind, "mass_g": round(res.mass_g, 1),
        "fos": round(res.fos, 2) if np.isfinite(res.fos) else None,
        "rejections": sum(1 for r in res.rounds if not r.accepted),
        "process": res.limits.process,
    }
