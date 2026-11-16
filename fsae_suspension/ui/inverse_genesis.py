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
from the top of the tab, verified against the recorded outputs (coordinates to
1e-6 mm, fits and yields to 1e-9).

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

from dataclasses import asdict

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


# =========================================================================== #
#  Plain-language help — shown as tooltips and in the glossary
# =========================================================================== #
_HELP = {
    "ig_src": "Where the starting corner comes from. The engine moves only "
              "the points you free in step 2; everything else stays put.",
    "ig_axle": "Which axle this corner is. It picks the axle station, track "
               "and spring used by the analysis.",
    "ig_travel": "Wheel travel either side of ride height over which the "
                 "curves are checked. ±25 mm is a common FSAE starting point.",
    "ig_nst": "How many travel positions are checked. 5 (−25, −12.5, 0, "
              "+12.5, +25 mm) is enough for smooth curves.",
    "ig_track": "Track width used to place the roll centre. Use the axle's "
                "real track.",
    "ig_tmode": "Formula: type a static value and a slope, the usual way a "
                "design brief states targets. Table: type every point.",
    "ig_f_gain": "Camber gain: how much the wheel leans in (negative) per mm "
                 "of bump. Negative keeps the outside tyre upright in roll. "
                 "Typical −0.02 to −0.05 deg/mm.",
    "ig_f_cb": "Band: how far the camber curve may stray from the target at "
               "each position. Wider bands are easier to hit and to build.",
    "ig_f_toe": "Toe target through travel. 0 means no bump steer.",
    "ig_f_tb": "How much toe change is acceptable. ±0.08 deg is tight; bump "
               "steer is felt by the driver.",
    "ig_f_rc": "Roll-centre height at each position (chassis frame, above "
               "static ground). Sets how lateral load splits between links "
               "and springs.",
    "ig_f_rb": "How far the roll centre may move. A wide band lets it "
               "migrate — check the migration in the analysis.",
    "ig_f_scrub": "Scrub radius: distance on the ground from the steering "
                  "axis to the tyre centre. Small and positive is usual.",
    "ig_f_sb": "How far scrub may stray from its target.",
    "ig_movable": "The points the engine is allowed to move. Start with the "
                  "five inboard pickups; keep ball joints fixed unless the "
                  "upright is also being designed.",
    "ig_bmode": "± half-width: a box around each starting point. Absolute: "
                "exact min/max coordinates, e.g. the space a rail allows.",
    "ig_shop": "How accurately your team can build. The engine picks the "
               "geometry that survives THIS tolerance best.",
    "ig_seed": "Random seed. Same seed + same inputs = identical result.",
    "ig_nstarts": "How many starting points the search tries. More starts "
                  "find more candidates but take longer. 6 is a good default.",
    "ig_nyield": "How many simulated builds are used to estimate yield. "
                 "4000 gives about ±0.5 % resolution.",
    "ig_nverify": "Re-solve some builds exactly to check the fast estimate. "
                  "0 is fine for exploration.",
    "ig_name": "Name the run after what you changed — it becomes the row "
               "name in the declaration log.",
    "ig_probe": "Treat each pickup as a ball of this radius when checking "
                "keep-outs (think bracket size).",
    "ig_mincl": "Minimum gap required between that ball and any keep-out.",
    "ig_pbounds": "Ranges on properties the four curve channels do not "
                  "carry. Each row is a wall: the search evaluates the "
                  "property on every trial geometry and refuses a step "
                  "that leaves the range, exactly as it refuses a "
                  "keep-out. Leave lo or hi empty for a one-sided "
                  "bound, but note that a floor alone lets the solver "
                  "overshoot: bound the band you actually want.",
    "ig_pb_drive": "Share of tractive force reaching this axle. 1.0 for "
                   "the driven axle of a RWD car, 0.0 for the front. "
                   "Only anti-squat uses it.",
    "ig_pb_nodes": "Sweep nodes used to evaluate the bounded properties "
                   "on each trial geometry. Roll-centre migration is the "
                   "least-squares slope over these nodes. More nodes "
                   "cost solver time on every step.",
    "ig_pull": "Systematic weld distortion toward the bead. 0 if unknown.",
}

_GLOSSARY = [
    ("Hardpoint", "A suspension pivot or ball-joint location (x rearward, "
     "y outboard, z up from the ground, mm, origin at the wheel centre)."),
    ("Camber gain", "Change of wheel lean per mm of bump. Negative = the top "
     "of the wheel moves inboard in bump."),
    ("Bump steer", "Toe change per mm of bump. Ideally zero."),
    ("Roll centre", "The point the body rolls about in front view; its "
     "height splits lateral load between links and springs."),
    ("RC migration", "How fast the roll centre moves with travel. Not a "
     "solver channel, so check it."),
    ("Scrub radius", "Ground distance from the steering axis to the tyre "
     "centre line."),
    ("Caster / trail", "Rearward tilt of the steering axis; trail is the "
     "ground distance it creates. Both set steering weight."),
    ("Band", "The ± window each target curve may sit in. It is also the "
     "margin that build scatter consumes."),
    ("Legal volume", "The box each free point may occupy — where a bracket "
     "can physically go."),
    ("Keep-out", "Space a point may not enter (frame tubes, headers, "
     "mounts)."),
    ("Build yield", "Share of simulated builds, at your shop's tolerance, "
     "whose curves still stay inside every band."),
    ("RESILIENT / TEMPERED / KNIFE_EDGE", "Yield ≥ 95 % / ≥ 80 % / below "
     "80 %."),
    ("W (worst case)", "First-order worst corner of the tolerance box. "
     "W ≤ 1 guarantees every build passes (to first order)."),
    ("Anti-dive / anti-squat", "Share of pitch load carried by the links "
     "instead of the springs under braking / drive."),
    ("Motion ratio", "Damper travel per unit wheel travel. Roll stiffness "
     "scales with its square."),
    ("Manifest", "A JSON file holding every input of a run; re-running it "
     "reproduces the result exactly."),
]

#: Worked examples. Each sets widget values before the tab draws.
_PRESETS = {
    "Front corner — jig-welded (recommended first run)": {
        "ig_src": "KinematiK default corner", "ig_axle": "front",
        "ig_travel": 25.0, "ig_nst": 5, "ig_track": 1210.0,
        "ig_tmode": "Formula (static + gain·t)",
        "ig_f_c": True, "ig_f_gain": -0.035, "ig_f_cb": 0.30,
        "ig_f_t": True, "ig_f_toe": 0.0, "ig_f_tb": 0.08,
        "ig_f_r": True, "ig_f_rc": 55.0, "ig_f_rb": 18.0, "ig_f_s": False,
        "ig_movable": ["upper_front_inner", "upper_rear_inner",
                       "lower_front_inner", "lower_rear_inner",
                       "tie_rod_inner"],
        "ig_bmode": "± half-width per axis about the seed",
        "ig_shop": "Jig-welded tabs (±0.5 mm)", "ig_seed": 0,
        "ig_nstarts": 6, "ig_nyield": 4000, "ig_nverify": 0,
        "ig_name": "front, jig weld, ±40 mm boxes",
    },
    "Same corner — hand-welded (see how the shop changes the answer)": {
        "ig_src": "KinematiK default corner", "ig_axle": "front",
        "ig_tmode": "Formula (static + gain·t)",
        "ig_f_c": True, "ig_f_gain": -0.035, "ig_f_cb": 0.30,
        "ig_f_t": True, "ig_f_toe": 0.0, "ig_f_tb": 0.08,
        "ig_f_r": True, "ig_f_rc": 55.0, "ig_f_rb": 18.0, "ig_f_s": False,
        "ig_shop": "Hand-welded tabs (±1.5 mm)", "ig_seed": 0,
        "ig_nstarts": 6, "ig_nyield": 2000,
        "ig_name": "front, hand weld, ±40 mm boxes",
    },
    "Quick look — 2 starts, 500 builds (seconds, not a final answer)": {
        "ig_nstarts": 2, "ig_nyield": 500, "ig_name": "quick look",
    },
}


#: The hosted app's quick search. Measured on front_v11_final (one core): pricing the yield is seconds at any N, but
#: every random start after the first is drawn until it clears every declared wall, up to 200 draws of about a
#: second each, so 2 starts took 220 s and 6 took 21 minutes. One start (the seed itself) took 5 s and returned the
#: same RESILIENT winner. The shared hosted CPU throttles a run of minutes before it finishes.
#: Below this gap to the rim or tire, with no running clearance declared, the envelope result is a warning.
_MIN_RUNNING_CLEARANCE_MM = 3.0
HOSTED_N_STARTS = 1
HOSTED_N_YIELD = 500
#: Below these a result is labelled a quick search: the published manifests all use 6 starts and N = 2000.
FULL_N_STARTS = 6
FULL_N_YIELD = 2000


def hosted(env=None, mount="/mount/src") -> bool:
    """True on the hosted app. KINEMATIK_HOSTED=1/0 decides when set; otherwise Streamlit Community Cloud is
    recognised by the folder it mounts the repository in."""
    import os
    env = os.environ if env is None else env
    flag = str(env.get("KINEMATIK_HOSTED", "")).strip().lower()
    if flag in ("1", "true", "yes"):
        return True
    if flag in ("0", "false", "no"):
        return False
    return os.path.isdir(mount)


def yield_precision_note(yield_frac, n, n_starts=FULL_N_STARTS) -> str | None:
    """What a quick search left out, for a search below FULL_N_STARTS starts or FULL_N_YIELD sampled builds; None
    for a full one. The yield's precision is in percentage points; a yield of 0 or 1 gets the one-sided 95 % bound
    (rule of three)."""
    few_starts = n_starts is not None and int(n_starts) < FULL_N_STARTS
    few_builds = yield_frac is not None and n is not None and int(n) < FULL_N_YIELD
    if not (few_starts or few_builds):
        return None
    parts = []
    if few_starts:
        parts.append(f"{int(n_starts)} start{'s' if int(n_starts) != 1 else ''} "
                     + ("(the seed geometry only), so other basins of the legal volume were not explored and a "
                        "more buildable geometry may exist" if int(n_starts) == 1
                        else "instead of " + str(FULL_N_STARTS) + ", so fewer basins of the legal volume were explored"))
    if few_builds:
        k = max(int(n), 1)
        p = float(yield_frac)
        if p >= 1.0:
            what = f"at least {round(100 * (1 - 3 / k), 1)} % (95 % confidence)"
        elif p <= 0.0:
            what = f"at most {round(100 * 3 / k, 1)} % (95 % confidence)"
        else:
            what = f"±{round(200 * (p * (1 - p) / k) ** 0.5, 1)} points (95 % confidence)"
        parts.append(f"the build yield is from {k} sampled builds, so it is {what}")
    return ("Quick search: " + "; ".join(parts) + f". Before quoting it, run {FULL_N_STARTS} starts and "
            f"N = {FULL_N_YIELD} or more on your own machine.")


def _apply_preset(name):
    import streamlit as st
    ss = st.session_state
    for k, v in _PRESETS[name].items():
        ss[k] = v
    if "ig_axle" in _PRESETS[name] and _PRESETS[name].get("ig_src", "").startswith("KinematiK"):
        ss["ig_gamma0"] = -1.5
        ss["ig_delta0"] = 0.0
    ss["ig_preset_msg"] = name


def _guided(ss):
    return ss.get("ig_mode", "Guided") == "Guided"


def _hint(st, ss, text):
    """A plain-language 'what this step does' box, Guided mode only."""
    if _guided(ss):
        st.info(text, icon="💡")


def _intro(st, ss):
    """Mode switch, worked examples and glossary at the top of the tab."""
    c1, c2 = st.columns([2, 3])
    c1.radio("Experience", ["Guided", "Expert"], horizontal=True,
             key="ig_mode",
             help="Guided adds a short explanation to every step and "
                  "plain-language result summaries. Expert keeps the page "
                  "compact. Nothing is hidden in either mode.")
    with c2.expander("📖 Glossary"):
        for term, text in _GLOSSARY:
            st.markdown(f"**{term}** — {text}")
    with st.expander("🚀 Start from a worked example",
                     expanded=_guided(ss) and "ig_run" not in ss):
        st.caption("Loads sensible inputs into every step below. Change "
                   "anything afterwards; press 🧬 Generate when ready.")
        cols = st.columns(len(_PRESETS))
        for col, name in zip(cols, _PRESETS):
            col.button(name, key=f"ig_preset_{abs(hash(name)) % 10**8}",
                       on_click=_apply_preset, args=(name,),
                       width="stretch")
        if ss.get("ig_preset_msg"):
            st.success("Loaded: " + ss["ig_preset_msg"])


def _progress(st, ss, box):
    """Five-step status bar, drawn into ``box`` after the page has run."""
    steps = [("Geometry", ss.get("ig_ok_geom", False)),
             ("Targets", ss.get("ig_ok_targets", False)),
             ("Legal volume", ss.get("ig_ok_volume", False)),
             ("Generated", ss.get("ig_run") is not None),
             ("Reviewed", ss.get("ig_ok_review", False))]
    done = sum(ok for _, ok in steps)
    with box:
        st.progress(done / len(steps))
        cols = st.columns(len(steps))
        for i, (col, (label, ok)) in enumerate(zip(cols, steps)):
            col.markdown(("✅ " if ok else "⬜ ") + f"**{i + 1}. {label}**")


_VERDICT_NEXT = {
    "RESILIENT": "Buildable as declared. Next: read the design review below, "
                 "then download the hardpoints and the manifest.",
    "TEMPERED": "Mostly buildable. To raise yield: widen the band that "
                "governs (named above), give the engine bigger boxes, free "
                "one more point, or build to a tighter shop class.",
    "KNIFE_EDGE": "It fits the curves, but many real builds will not. Do "
                  "not cut metal yet: widen the governing band, enlarge the "
                  "boxes, or pick a tighter shop class, then run again.",
}


def _explain_result(st, ss, res, man):
    """Plain-language 'what this means / what next' for a run."""
    lines = []
    w = res.winner
    if w is None:
        gov = res.best_fit.worst_row if res.best_fit else "a target"
        lines.append("**No geometry inside the boxes reached every band.** "
                     f"The closest attempt missed on *{gov}*.")
        if res.n_starts and not res.candidates:
            lines.append("Every start failed before the search began, which "
                         "usually means the declaration, not the linkage, is "
                         "the problem: targets left at defaults that this "
                         "corner cannot reach, boxes far from the seed "
                         "geometry, or a solved-property bound no start "
                         "satisfies. If you meant to reproduce a published "
                         "result, upload its manifest at the top of the tab "
                         "instead of filling the form in.")
        lines.append("Try, in order: widen that channel's band; enlarge the "
                     "boxes of the points that control it; free one more "
                     "point; check the target is physically plausible for "
                     "this corner (e.g. camber gain sign).")
    else:
        lines.append(_VERDICT_NEXT.get(w.verdict, ""))
        if w.clamped:
            lines.append("**" + str(len(w.clamped)) + " coordinate(s) "
                         "finished on a box face** (" + ", ".join(w.clamped)
                         + "). The box, not the targets, is limiting — "
                         "widen it if the chassis allows.")
        if (res.resilience_premium or 0) > 0.005:
            lines.append("Ranking by yield chose a different candidate than "
                         "the closest fit — the closest fit is less "
                         "buildable at this tolerance.")
        hits = [c for c in res.candidates if c.hit and c.yield_frac is not None]
        if len(hits) >= 2:
            ys = [c.yield_frac for c in hits]
            if max(ys) - min(ys) > 0.2:
                lines.append("Candidates that all fit the curves differ "
                             "widely in yield — fit alone would not have "
                             "told them apart.")
    with st.container(border=True):
        st.markdown("**What this means**")
        for ln in lines:
            if ln:
                st.markdown("- " + ln)



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


_ACT_LABELS = ("Rocker assembly (pushrod + bell-crank)", "Direct-acting damper (no rocker)")
_ATTACH_LABELS = {"lower wishbone": "lower", "upper wishbone": "upper", "upright": "upright"}


def _actuation_selector(st, hp, key: str, title: str = "Spring/damper actuation"):
    """Choose rocker assembly or direct-acting damper for a corner; returns a copy of hp set up for the choice.

    Direct-acting: the damper runs from a point on the lower or upper wishbone or the upright straight to a chassis
    mount, and its length change gives the real motion ratio and its axial force the real damper load. Rocker: the
    pushrod/rocker points the corner already carries are used (or designed in the actuation stage)."""
    import numpy as np
    from suspension import genesis_analysis as ga
    hp = hp.copy()
    cur = 1 if getattr(hp, "actuation", "rocker") == "direct" else 0
    choice = st.radio(title, _ACT_LABELS, index=cur, horizontal=True, key=f"{key}_act_kind",
                      help="Rocker: pushrod to a bell-crank, spring/damper across the rocker. Direct-acting: the "
                           "damper mounts on the wishbone or upright and runs straight to the chassis, no rocker.")
    if choice == _ACT_LABELS[0]:
        hp.actuation = "rocker"
        if not hp.has_rocker():
            st.caption("No complete rocker on this corner yet: the motion ratio falls back to the declared curve "
                       "until the pushrod/rocker points are pasted or designed in the actuation stage.")
        return hp
    hp.actuation = "direct"
    if cur == 1 and hp.pushrod_outer is not None and hp.spring_inner is not None:   # already direct: keep its ends
        out0, in0 = np.asarray(hp.pushrod_outer, float), np.asarray(hp.spring_inner, float)
    else:
        out0, in0 = ga.default_direct_damper(hp)
        hp.pushrod_attach = "lower"
    labels = list(_ATTACH_LABELS)
    body = st.selectbox("Damper outer end is on the", labels,
                        index=list(_ATTACH_LABELS.values()).index(getattr(hp, "pushrod_attach", "lower")),
                        key=f"{key}_act_body")
    hp.pushrod_attach = _ATTACH_LABELS[body]
    pts = {}
    for name, label, v0 in (("pushrod_outer", "Damper outer end (on the " + body + ")", out0),
                            ("spring_inner", "Damper chassis mount", in0)):
        c = st.columns(3)
        pts[name] = np.array([float(c[i].number_input(f"{label} {a} (mm)", -5000.0, 5000.0, float(v0[i]), 0.1,
                                                       key=f"{key}_act_{name}_{a}"))
                              for i, a in enumerate("xyz")])
    hp.pushrod_outer, hp.spring_inner = pts["pushrod_outer"], pts["spring_inner"]
    try:
        from suspension.kinematics import SuspensionKinematics
        mr = SuspensionKinematics(hp).motion_ratio()
        L0 = float(np.linalg.norm(pts["spring_inner"] - pts["pushrod_outer"]))
        if np.isfinite(mr):
            st.caption(f"Direct-acting damper: motion ratio {_g(mr, digits=3)} (damper / wheel travel), "
                       f"static length {_g(L0, 'mm', digits=4)}. Frame: x rearward, y outboard, z up from the ground.")
        else:
            st.warning("The damper does not change length with wheel travel here; move its ends.")
    except Exception as e:                  # noqa: BLE001
        st.warning(f"Could not solve the corner with this damper: {e}")
    return hp


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


def _name_next(ss, title) -> bool:
    """Name the table drawn next for the report (see report_capture.heading). True, so it can sit in an expression."""
    from suspension import report_capture as rc
    rc.heading(ss, title)
    return True


def bound_text(lo, hi) -> str:
    """A bound as a reader checks it: '63.6 to 70', '≥ 2' or '≤ 120'. The table used to repeat the property's whole
    label here, which the report cut off before the number on the longer ones."""
    import math
    has_lo = lo is not None and math.isfinite(float(lo))
    has_hi = hi is not None and math.isfinite(float(hi))
    if has_lo and has_hi:
        return _cell(lo) + " to " + _cell(hi)
    if has_lo:
        return "≥ " + _cell(lo)
    return ("≤ " + _cell(hi)) if has_hi else "unbounded"


def result_caveats(res, man, fld) -> list:
    """What a reader must know before quoting a result, each as one sentence: a build yield priced against a shop
    preset is not a measurement; a search that returned one candidate compared it with nothing; a winner that did
    not move was verified, not generated."""
    import numpy as _np
    out = []
    w = res.winner
    if w is not None and w.yield_frac is not None and fld is not None and not getattr(fld, "calibrated", True):
        out.append("The build yield is priced against " + str(fld.provenance).split(". ")[0].rstrip(".")
                   + ": it is the yield if your shop holds those tolerances. Measure your own parts before quoting it.")
    asked = int(getattr(man.search, "n_starts", 0) or 0)
    if w is not None and asked > 1 and len(res.candidates) <= 1:
        got = int(getattr(res, "n_starts", 0) or 0)
        out.append(f"One candidate only: {got} of the {asked} starts asked for produced a geometry (the others were "
                   "never drawn clear of the declared walls). 'Most buildable' compares it with nothing, and no "
                   "other region of the legal volume was explored.")
    if w is not None and _np.max(_np.abs(_np.asarray(w.shift_vec, float)), initial=0.0) < 1e-6:
        out.append("No pickup moved: the starting geometry already met every curve and bound. This run verified "
                   "that geometry and priced its build yield; it did not generate it.")
    return out


def _manifest_run_summary(st, run, man):
    """The outcome of a manifest re-run, shown beside its Run button: the grade, build yield and fit the run returned,
    whether they match the recorded ones, and what the start search did. Yield in %, fit as a fraction of the band."""
    res, ver = run["result"], run.get("verify")
    w = getattr(res, "winner", None)
    rec = man.recorded or {}
    replayed = bool(run.get("replayed"))
    st.markdown("###### Recorded result, replayed" if replayed else "###### Result of this run")
    if w is not None:
        badge, _ = _VERDICT_BLURB.get(w.verdict, ("", ""))
        y = f" — build yield {_g(w.yield_frac * 100, '%', _TOL)}" if w.yield_frac is not None else ""
        st.markdown(f"### {badge} {w.verdict}{y}")
        st.caption(f"Curves at {_g(w.max_band_frac, '× band', digits=3)} of their bands (worst: {w.worst_row}); "
                   f"{len(res.candidates)} candidates searched.")
    else:
        st.markdown("### ⚪ NO_FIT — no geometry meets every band")
        bf = getattr(res, "best_fit", None)
        if bf is not None:
            st.caption(f"Closest approach: {_g(bf.max_band_frac, '× band', digits=3)}, governed by {bf.worst_row}.")
    if rec.get("winner_verdict") is not None or rec:
        ry = rec.get("winner_yield")
        st.caption("Recorded in the manifest: " + str(rec.get("winner_verdict") or "NO_FIT")
                   + (f", build yield {round(100 * ry, 2)}%" if ry is not None else ""))
    rep = getattr(res, "start_search", None)
    if rep:
        ok = rep.get("walls_met", rep.get("walls_hold"))
        st.caption(f"Start search: {rep['evaluations']} evaluations; "
                   + ("every wall holds." if ok else "walls still broken: " + ", ".join(rep.get("final_violated") or []) + "."))
    if ver is not None:
        same, diffs = ver
        real = [d for d in diffs if not d.startswith("(note)")]
        verdict = f" ({w.verdict})" if w is not None else " (NO_FIT)"
        if replayed:
            (st.success if same else st.error)(
                f"✅ Replay of `{man.name}`{verdict}: the recorded geometry reproduces its recorded fits with this "
                "build (coordinates to 1e-6 mm, fits to 1e-9). Build yields are the recorded ones, not re-sampled: "
                "▶ Run this manifest re-samples them." if same
                else f"❌ Replay of `{man.name}`{verdict}: with this build the recorded geometry no longer gives its "
                     f"recorded fits, in {len(real)} place(s); details in the results below.")
        else:
            what = f"This run of `{man.name}`" + verdict
            (st.success if same else st.error)(
                f"✅ {what} matches the recorded outputs (coordinates to 1e-6 mm, fits and yields to 1e-9)." if same
                else f"❌ {what} differs from the recorded outputs in {len(diffs)} place(s); details in the results below.")
    st.caption("The candidate table, solved properties and diagnostics are in the results panel at the bottom of "
               "this tab.")


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
        if run.get("replayed"):
            from suspension import report_capture as rc
            with rc.unreported(ss):             # said once, in the run summary at the top of the tab
                (st.success if same else st.error)(
                    "✅ Replayed from the recorded outputs: geometry and fits reproduce with this build; build "
                    "yields as recorded, not re-sampled." if same
                    else "❌ The recorded geometry no longer reproduces its recorded fits with this build.")
        elif same:
            st.success("✅ Re-run reproduces the recorded outputs: every candidate's "
                       "coordinates to 1e-6 mm, fits and yields to 1e-9"
                       + (", and the start search's moves" if run.get("result") is not None
                          and getattr(run["result"], "start_search", None) else "") + ".")
        else:
            st.error("❌ Re-run differs from the recorded outputs.")
        for d in diffs:
            st.caption(d)

    rep = getattr(res, "start_search", None)
    if rep:
        moved = ", ".join(f"{k} {round(v, 1)}" for k, v in sorted(rep.get("shift_mm", {}).items(),
                                                          key=lambda kv: -abs(kv[1]))[:6])
        msg = (f"Start search: {rep['evaluations']} evaluations; "
               + ("every wall holds" if rep.get("walls_met", rep.get("walls_hold")) else
                  "walls still broken: " + ", ".join(rep.get("final_violated") or []))
               + (f", curves at {round(rep['final_band'], 2)} of their bands" if rep.get("final_band") is not None else "")
               + (f". Largest moves (mm): {moved}." if moved else "."))
        (st.info if rep.get("walls_met", rep.get("walls_hold")) else st.warning)(msg)

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
        quick = None if run.get("replayed") else yield_precision_note(w.yield_frac, man.search.n_yield,
                                                                      man.search.n_starts)
        if quick:
            st.warning("⚡ " + quick)
        if res.ok and len(res.candidates) <= 1:
            # the engine's sentence compares the winner with the other candidates; with one there is no comparison
            st.success("The curves are reached inside every declared wall"
                       + (f", and the geometry is {w.verdict} at the declared tolerances "
                          f"({_cell(round(100 * w.yield_frac, 1))} % build yield)." if w.yield_frac is not None else "."))
        else:
            (st.success if res.ok else st.warning)(res.reason)
        for caveat in result_caveats(res, man, fld):
            st.warning(caveat)
        thin = as_built_unpriced(res)
        if thin:
            # one line under the report's 400-character banner limit, however many properties are thin
            items = [f"{lab.split(' (')[0]} {_cell(round(m, 3))} {lab.split('(')[-1].rstrip(')')}"
                     if "(" in lab else f"{lab} {_cell(round(m, 3))}" for lab, m in thin]
            head = "Build yield prices the camber, toe and roll-centre curves only. Met at nominal, unchecked across builds, little margin: "
            tail = ". Tolerances can take these out of bounds."
            while items and len(head + "; ".join(items) + tail) > 395:
                items = items[:-1]
            more = len(thin) - len(items)
            st.warning(head + "; ".join(items) + (f"; {more} more" if more else "") + tail)
        m1, m2, m3, m4 = st.columns(4)
        if w.yield_frac is not None:
            m1.metric("Build yield", _g(w.yield_frac * 100, "%", _TOL))
        m2.metric("Worst station", _g(w.max_band_frac, "× band", digits=3),
                  delta=w.worst_row, delta_color="off")
        m3.metric("Iterations", f"{w.iterations}")
        if res.resilience_premium is not None:
            m4.metric("Resilience premium",
                      f"{res.resilience_premium*100:+.1f} pts")

    _explain_result(st, ss, res, man)
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
          "steps refused by keep-out / property / wheel / node":
              f"{c.keepout_rejections} / {c.property_rejections} / {c.envelope_rejections} / {c.node_rejections}",
          "wheel clearance beyond the required gap (mm)": (round(c.envelope_margin_mm, 2)
                                                           if c.envelope_margin_mm is not None
                                                           else "—")}
         for i, c in enumerate(res.candidates)])
        if _name_next(ss, "Candidates the search returned, best first") else None,
        hide_index=True, width="stretch")

    # ---- solved properties the search bounded ----------------------------- #
    if res.property_bounds is not None and res.winner is not None \
            and res.winner.properties:
        st.markdown("###### Solved properties, bounded inside the search")
        st.caption("Evaluated at the static state on every trial geometry "
                   "(migration as the least-squares slope over the travel "
                   "range), and a step that left the range was refused. "
                   "These bound the nominal corner, not every as-built one: "
                   "the build-yield stage still prices only the four curve "
                   "channels.")
        _name_next(ss, "Solved properties the search bounded: the bound, the value delivered, the margin left")
        st.dataframe(pd.DataFrame(
            [{"property": ig._PROPERTY_LABELS[b.prop], "bound": bound_text(b.lo, b.hi),
              "delivered": round(res.winner.properties.get(b.prop,
                                                           float("nan")), 3),
              "margin": round(b.margin(res.winner.properties.get(
                  b.prop, float("nan"))), 3)}
             for b in res.property_bounds.bounds]),
            hide_index=True, width="stretch")

    if res.wheel_envelope is None and res.winner_hp is not None:
        # Never silent: with the wall off, still check the result against a
        # standard wheel and say so if a link passes through it.
        post = ig.WheelEnvelope(travel_mm=(-25.0, 25.0))
        pv = post.violations(res.winner_hp)
        if pv:
            w = min(pv, key=lambda v: v[2])
            st.warning(f"This geometry puts **{w[0]}** {abs(w[2]):.1f} mm "
                       f"inside a standard 10-inch wheel ({w[1]}). The search "
                       "did not forbid it because the wheel envelope was off. "
                       "Turn on 🛞 Wheel envelope in section 3, with your own "
                       "wheel dimensions, and regenerate.")
    if res.wheel_envelope is not None and res.winner is not None \
            and res.winner.envelope_margin_mm is not None:
        m = res.winner.envelope_margin_mm
        gap = float(getattr(res.wheel_envelope, "clearance_mm", 0.0) or 0.0)
        where = "over the declared travel" + (" and both steering locks" if res.wheel_envelope.rack_travel_mm else "")
        if m < 0:
            st.error(f"Wheel envelope: worst clearance {m:+.2f} mm {where}: a link or outboard pickup is inside "
                     "the rim or tire" + (f", or closer to it than the {_cell(gap)} mm required" if gap else "") + ".")
        elif gap > 0:
            st.success(f"Wheel envelope: {_cell(round(m + gap, 2))} mm from the nearest link or outboard pickup to "
                       f"the rim and tire {where}: {_cell(gap)} mm required, {_cell(round(m, 2))} mm to spare.")
        elif m < _MIN_RUNNING_CLEARANCE_MM:
            st.warning(f"Wheel envelope: nothing enters the rim or tire {where}, but the nearest link is "
                       f"{_cell(round(m, 2))} mm from it and no running clearance was declared: rim run-out and "
                       f"link deflection can use that up. Declare a gap ({_MIN_RUNNING_CLEARANCE_MM:g} mm or more) "
                       "in the wheel envelope.")
        else:
            st.success(f"Wheel envelope: {_cell(round(m, 2))} mm from the nearest link or outboard pickup to the "
                       f"rim and tire {where}.")

    if res.winner_hp is None:
        return
    whp = res.winner_hp

    # ---- the deliverable, at full precision ------------------------------- #
    st.markdown("###### The generated hardpoints — full precision (the "
                "deliverable)")
    whd = gr.hp_to_dict(whp)
    _ALL_ROWS = _POINT_ROWS + tuple(q for q in _ROCKER_PTS if q in whd)
    _name_next(ss, "Hardpoints (mm): x rearward of the wheel centre, y outboard of the centreline, z above ground; "
                   "designed = free to move")
    st.dataframe(pd.DataFrame(
        [{"point": p + (" (unit vector, not mm)" if p == "rocker_axis" else ""),
          "x": whd[p][0], "y": whd[p][1], "z": whd[p][2],
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
                _name_next(ss, f"Sampled builds failing each channel (N = {n}, sampled now with this build)")
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
                    ig._shifted(man.search_start(), volume, res.best_fit.shift_vec), targets,
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
            st.caption("Recover the frame from STEP under 3 · Keep-out "
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
        if wmsg == gr.REPLAY_NOTE:
            from suspension import report_capture as rc
            with rc.unreported(ss):          # the run summary already says it, once, in the report
                st.warning(wmsg)
        else:
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
    st.subheader("🧬 InverseGenesis — describe the suspension you want; "
                 "the engine finds geometry your shop can build")
    st.caption("You state the curves (camber, toe, roll centre) and where "
               "each pickup may go. The engine moves the pickups until the "
               "curves fit, then keeps the candidate that survives your "
               "shop's build tolerance best. Every run can be downloaded and "
               "re-run exactly.")
    _intro(st, ss)
    bar = st.container()
    for k in ("ig_ok_geom", "ig_ok_targets", "ig_ok_volume", "ig_ok_review"):
        ss[k] = False
    from suspension import report_capture as rc
    rc.clear_stale_pause(ss)
    pending = manifest_pending(ss)
    if ss.get("ig_run") is not None:
        rc.engage(ss)                         # a result the user made here: report it, whatever the open signal said
    if (ss.get("ig_run") or {}).get("source") != "manifest":
        _veh(ss).pop("link_tubes", None)      # per-link tubes belong to the manifest that declared them
    # Every run redraws everything this tab reports, so it starts from nothing: a line an earlier run drew and this
    # one does not (an input since changed, or an older build of this code in the same session) must leave the
    # report, and clearing on a change of inputs alone left the second kind in.
    rc.reset(ss)
    ss["ig_report_identity"] = report_identity(ss, pending)
    if pending:
        st.warning("A manifest is loaded but has not been run, so nothing in "
                   "this tab is reported: the tables below describe the form "
                   "and the seed geometry, not the manifest. Press ▶ Run "
                   "this manifest, or remove the file.")
    with rc.unreported(ss, pending):
        _render_generate()
        _render_analysis(st, pd, np, ss)
    _progress(st, ss, bar)


def manifest_pending(ss) -> bool:
    """True while an uploaded manifest has not been run in this session: the
    tab then shows the manifest's recorded verdict above tables computed from
    something else, and a report of that reads as one design."""
    up = ss.get("ig_manifest_up")
    if up is None:
        return False
    run = ss.get("ig_run")
    if not run or run.get("source") != "manifest":
        return True
    try:
        from suspension import genesis_repro as gr
        sha = gr.GenesisManifest.from_json(up.getvalue().decode()).inputs_sha256
    except Exception:                         # noqa: BLE001  (an unreadable file has not been run)
        return True
    return run.get("inputs_sha256") != sha


def report_identity(ss, pending: bool) -> str:
    """What this tab's report is about: the run's inputs hash (or the seed
    geometry), then a digest of every declaration the analysis reads. When
    either changes, what was recorded for the old one is dropped: a table is
    replaced by its next drawing anyway, but a warning that no longer applies
    is simply not drawn again, and stayed in the report beside the numbers
    that had cleared it."""
    if pending:
        return "manifest loaded, not run"
    run = ss.get("ig_run")
    if not run:
        return "seed | " + analysis_inputs_digest(ss)
    sha = run.get("inputs_sha256")
    if sha is None:
        try:
            from suspension import genesis_repro as gr
            sha = gr.GenesisManifest.from_json(run["manifest_json"]).inputs_sha256
        except Exception:                     # noqa: BLE001
            sha = "unreadable"
    return ("replay " if run.get("replayed") else "run ") + str(sha) + " | " + analysis_inputs_digest(ss)


#: Session keys the analysis sections read besides the run and the vehicle declaration.
_ANALYSIS_INPUT_KEYS = ("ig_seed_hp", "ig_axle", "ig_travel", "ig_review_limits", "genesis_corners", "ig_brk",
                        "ig_cmp_F", "ig_cmp_L", "ig_cmp_rep", "ig_cmp_dcam", "ig_cmp_dtoe")


def analysis_inputs_digest(ss) -> str:
    """Twelve hex digits of the SHA-256 of the vehicle & build declaration and the other inputs of the analysis
    sections, as they stand at the start of a script run."""
    import hashlib
    import json
    state = {"vehicle": _veh(ss), **{k: ss.get(k) for k in _ANALYSIS_INPUT_KEYS}}
    return hashlib.sha256(json.dumps(state, sort_keys=True, default=str).encode()).hexdigest()[:12]


def recorded_verdict_line(man) -> str | None:
    """The result a manifest records, in one line: grade, build yield (%), fit (fraction of the band); None when the
    manifest has no recorded outputs."""
    rec = man.recorded or {}
    if not rec:
        return None
    v = rec.get("winner_verdict")
    if not v:
        return "Recorded result: NO_FIT, no geometry met every band"
    y, f = rec.get("winner_yield"), None
    cands = rec.get("candidates") or []
    if cands:
        f = min(c.get("max_band_frac", float("inf")) for c in cands if c.get("max_band_frac") is not None)
    return ("Recorded result: " + str(v) + (f" — build yield {round(100 * y, 2)}%" if y is not None else "")
            + (f", curves at {round(f, 3)} of their bands" if f is not None and f != float("inf") else ""))


def _manifest_summary(st, ig, man):
    """One-glance summary of what a manifest will actually run, led by the result it records."""
    line = recorded_verdict_line(man)
    if line:
        badge, _ = _VERDICT_BLURB.get((man.recorded or {}).get("winner_verdict") or "", ("", ""))
        st.success(f"{badge} **{line}**. ⏩ Load the recorded result shows it in seconds; ▶ Run this manifest "
                   "repeats the whole search and checks it reproduces, which can take 20 minutes or more.")
    try:
        hp, targets, volume, fld = man.objects()
    except (ValueError, KeyError, TypeError) as e:
        st.warning(f"Manifest loaded but could not be summarised: {e}")
        return
    chans = ", ".join(sorted({c.channel for c in targets.curves})) or "none"
    pts = ", ".join(volume.points()) or "none"
    lines = [
        f"- **Free hardpoints:** {pts}",
        f"- **Target channels:** {chans} over "
        f"{len(targets.stations())} stations",
        f"- **Keep-out volumes:** {len(volume.keep_out)} · "
        f"spacing rules: {len(volume.spacings)}",
        f"- **Search:** seed {man.search.seed} · {man.search.n_starts} "
        f"starts · N = {man.search.n_yield}",
        f"- **Tolerance field:** "
        + (fld.provenance if fld is not None else "none declared"),
    ]
    lines.append("- **Wheel envelope:** "
                 + (volume.wheel_envelope.label if volume.has_wheel_envelope()
                    else "not declared — links may pass through the wheel"))
    if volume.has_property_bounds():
        b = "; ".join(p.label for p in volume.properties.bounds)
        lines.append(f"- **Solved-property bounds (enforced in the search):** "
                     f"{b}")
    else:
        lines.append("- **Solved-property bounds:** none — anti-squat, "
                     "migration and caster run free")
    ssd = getattr(man, "start_search", None)
    if ssd:
        lines.append(f"- **Start search (run first):** pattern search, seed {ssd.get('seed')}, "
                     f"up to {ssd.get('max_evals')} evaluations, steps {ssd.get('step0_mm')} to "
                     f"{ssd.get('step_min_mm')} mm"
                     + (", walls only" if ssd.get("walls_only") else ""))
    st.markdown("\n".join(lines))


def _render_generate():
    import numpy as np
    import pandas as pd
    import streamlit as st
    from suspension import units as _units
    from suspension import inverse_genesis as ig
    from suspension import kinematik_stochastic as ks
    from suspension import genesis_repro as gr
    from suspension import report_capture as rc

    ss = st.session_state

    # ================= 0 · re-run a manifest ==============================
    #  Two ways in, and they must not be confusable: a manifest REPLACES every
    #  form field below, so a user who uploads one and then presses the form's
    #  own button gets a different run than the one they meant. The expander
    #  therefore opens itself the moment a file is present and says plainly
    #  which button does what.
    _man_loaded = ss.get("ig_manifest_up") is not None
    with st.expander("📂 Re-run a saved manifest — reproduce a published "
                     "result exactly (skips every field below)",
                     expanded=_man_loaded):
        st.caption("A manifest carries the whole declaration: hardpoints, "
                   "target curves, boxes, keep-outs, solved-property bounds, "
                   "shop class and search settings. Running it ignores the "
                   "form below entirely, so you do not have to fill anything "
                   "in. Use this to reproduce a number from a paper or to "
                   "re-run your own archived result on a newer build.")
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
                with rc.unreported(ss):       # what the file says, not what a run returned
                    _manifest_summary(st, ig, man)
                if man.context:
                    st.json(man.context, expanded=False)
                st.info("Press the button below — **not** 🧬 Generate the "
                        "geometry further down, which runs the form instead "
                        "and will ignore this file.")
                b_load, b_run = st.columns(2)
                load = b_load.button(
                    "⏩ Load the recorded result", key="ig_manifest_load", type="primary",
                    disabled=not man.recorded,
                    help="Seconds. Places every recorded candidate at its recorded coordinates and recomputes its "
                         "curves and fits with this build; the build yields are the recorded ones, not re-sampled.")
                run = b_run.button(
                    "▶ Run this manifest", key="ig_manifest_run",
                    help="Repeats the whole search, re-sampling every build yield, and checks it reproduces the "
                         "recorded outputs. 20 minutes or more for the published manifests.")
                if hosted():
                    b_run.caption("On the hosted app a full run can be throttled before it finishes. Load the "
                                  "recorded result here, or run KinematiK on your own machine to re-sample.")
                if load or run:
                    if run:
                        with st.spinner("Re-running the manifest (20 minutes or more)…"):
                            res = man.run()
                    else:
                        res = man.replay()
                    ver = man.verify(res) if man.recorded else None
                    if not man.recorded:
                        man.record(res)
                    apply_manifest_vehicle(ss, man, man.context.get("axle") or "front")
                    ss["ig_run"] = {"manifest_json": man.to_json(),
                                    "result": res, "verify": ver, "replayed": bool(load and not run),
                                    "source": "manifest", "inputs_sha256": man.inputs_sha256,
                                    **{k: man.context.get(k) for k in
                                       ("axle", "axle_station", "ground_y")
                                       if man.context.get(k) is not None}}
                    st.rerun()                # draw (and report) the whole tab for the run, not half of it
                _run = ss.get("ig_run")
                if _run and _run.get("source") == "manifest" and _run.get("inputs_sha256") == man.inputs_sha256:
                    _manifest_run_summary(st, _run, man)
            except ValueError as e:
                st.error(f"Manifest refused: {e}")

    if _man_loaded:
        with rc.unreported(ss):               # a note about the tab, not a finding about the car
            st.warning("A manifest is loaded above. The fields below are **not** "
                       "what it will run — use ▶ Run this manifest inside that "
                       "section. Remove the file to go back to filling the form "
                       "in yourself.")

    # ================= geometry ===========================================
    st.markdown("###### 1 · Starting geometry")
    _hint(st, ss, "Pick the corner the engine starts from. New to this? Use "
                  "the KinematiK default corner or a worked example above. "
                  "Have CAD numbers? Paste them — corner frame is x rearward, "
                  "y outboard, z up from the ground, in mm.")
    live_hp, live_note = _hardpoints_from_session(ss)
    src = st.radio("Source", ["Live (Kinematics tab)",
                              "KinematiK default corner",
                              "Paste hardpoints (corner frame)",
                              "CAD coordinates (SolidWorks frame)"],
                   horizontal=True, help=_HELP["ig_src"], key="ig_src")
    from suspension.kinematics import Hardpoints
    axle_station = ground_y = None
    axle = st.radio("Axle", ["front", "rear"], horizontal=True,
                    help=_HELP["ig_axle"], key="ig_axle")
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
                   "edit them under 5 · Vehicle & build declaration.")
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
    hp = _actuation_selector(st, hp, "ig")
    ss["ig_seed_hp"] = gr.hp_to_dict(hp)

    # ================= 1 · targets ========================================
    ss["ig_ok_geom"] = True
    st.markdown("###### 2 · What the suspension should do")
    _hint(st, ss, "State each curve as a target plus a ± band. The engine "
                  "only has to land inside the band, so a sensible band is "
                  "as important as the target: too tight and nothing is "
                  "buildable.")
    c1, c2, c3 = st.columns(3)
    travel = float(c1.number_input("Travel range (± mm)", 2.0, 80.0, 25.0,
                                   0.5, help=_HELP["ig_travel"], key="ig_travel"))
    n_st = int(c2.number_input("Stations", 2, 41, 5, 1, help=_HELP["ig_nst"], key="ig_nst"))
    track = float(c3.number_input("Track for RC construction (mm)", 500.0,
                                  2500.0, 1210.0, 5.0, help=_HELP["ig_track"], key="ig_track"))
    stations = np.linspace(-travel, travel, n_st)

    nom_vals, nom_ok = ig.curves_of(hp, stations, track_mm=track)
    if not nom_ok:
        st.error("The seed geometry does not solve over this travel range.")
        return

    staged = ss.get("genesis_targets")
    modes = ["Formula (static + gain·t)", "Per-station table"]
    if staged is not None:
        modes.append("Staged from FullCar")
    tmode = st.radio("Targets as", modes, horizontal=True, help=_HELP["ig_tmode"], key="ig_tmode")

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
                               format="%.4f", help=_HELP["ig_f_gain"], key="ig_f_gain")
        cband = f3.number_input("± band (deg)", 0.001, 5.0, 0.30, 0.01,
                                help=_HELP["ig_f_cb"], key="ig_f_cb")
        f4.caption(f"target = γ₀ + gain·t, γ₀ = {hp.static_camber:+.2f}°")
        g1, g2, g3, _ = st.columns(4)
        use_t = g1.checkbox("Toe", True, key="ig_f_t")
        toe = g2.number_input("toe (deg)", -2.0, 2.0, 0.0, 0.01,
                              help=_HELP["ig_f_toe"], key="ig_f_toe")
        tband = g3.number_input("± band (deg) ", 0.001, 5.0, 0.08, 0.01,
                                help=_HELP["ig_f_tb"], key="ig_f_tb")
        h1, h2, h3, _ = st.columns(4)
        use_r = h1.checkbox("RC height", True, key="ig_f_r")
        rc_mm = h2.number_input("RC height (mm)", -200.0, 300.0, 55.0, 0.5,
                             help=_HELP["ig_f_rc"], key="ig_f_rc")
        rband = h3.number_input("± band (mm)", 0.01, 200.0, 18.0, 0.5,
                                help=_HELP["ig_f_rb"], key="ig_f_rb")
        k1, k2, k3, _ = st.columns(4)
        use_s = k1.checkbox("Scrub", False, key="ig_f_s")
        scrub = k2.number_input("scrub (mm)", -100.0, 100.0, 5.0, 0.5,
                                help=_HELP["ig_f_scrub"], key="ig_f_scrub")
        sband = k3.number_input("± band (mm) ", 0.01, 100.0, 3.0, 0.5,
                                help=_HELP["ig_f_sb"], key="ig_f_sb")
        try:
            targets = gr.linear_targets(
                stations, static_camber=hp.static_camber,
                camber_gain=gain if use_c else None, camber_band=cband,
                toe=toe if use_t else None, toe_band=tband,
                rc_height=rc_mm if use_r else None, rc_band=rband,
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
    ss["ig_ok_targets"] = True
    st.markdown("###### 3 · Where the pickups may go")
    _hint(st, ss, "Free the pickups the engine may move and give each a box "
                  "it must stay in — the space a bracket can really reach on "
                  "your frame. Add keep-outs for tubes, headers and mounts. "
                  "Anything you do not constrain, the engine may exploit.")
    movable = st.multiselect(
        "Hardpoints the engine may move", list(ig.DESIGNABLE_POINTS),
        default=["upper_front_inner", "upper_rear_inner",
                 "lower_front_inner", "lower_rear_inner", "tie_rod_inner"],
        help=_HELP["ig_movable"], key="ig_movable")
    if not movable:
        st.info("Free at least one hardpoint.")
        return
    bmode = st.radio("Boxes as", ["± half-width per axis about the seed",
                                  "Absolute bounds (corner frame)"],
                     horizontal=True, help=_HELP["ig_bmode"], key="ig_bmode")
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

    with st.expander("🧱 Your chassis — upload the STEP file",
                     expanded=not ss.get("tf_frame")):
        _sec_frame_from_step(st, pd, np, ss, _veh(ss), "ig_step")
    use_frame = False
    if ss.get("tf_frame"):
        use_frame = st.checkbox(
            "Keep the pickups and links clear of the chassis tubes "
            "(use the frame as keep-outs)", value=True, key="ig_use_frame",
            help="Every free pickup must stay the required clearance away "
                 "from every tube. After a run, the swept-volume check "
                 "uses the same frame.")
        if use_frame and axle_station is None:
            _v = _veh(ss)
            axle_station = float(_v[f"{axle}_station_mm"])
            ground_y = float(_v["ground_y_mm"])
            st.caption(f"Frame placed with the declared {axle} axle station "
                       f"({axle_station} mm) and ground plane "
                       f"({ground_y} mm) — edit them in section 5.")
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
        k1, k2 = st.columns(2)
        probe = _units.unum(k1, "Probe radius (mm)", 0.0, 30.0, 6.0, 'mm',
                            step=1.0, help=_HELP["ig_probe"], key="ig_probe")
        min_cl = _units.unum(k2, "Required clearance (mm)", 0.0, 20.0, 2.0,
                             'mm', step=0.5, help=_HELP["ig_mincl"], key="ig_mincl")
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

    # ---- bounds on properties the curve channels do not carry ------------ #
    with st.expander("⭐ Bounds on solved properties — anti-squat, "
                     "migration, caster (the channels do NOT carry these)"):
        st.caption("The four curve channels do not carry anti-squat, "
                   "anti-dive, roll-centre migration rate, caster or kingpin "
                   "inclination, and the engine is indifferent to any "
                   "property it is not told about. A row here is a wall: the "
                   "property is evaluated on every trial geometry and a step "
                   "that leaves the range is refused, exactly as a keep-out "
                   "violation is. Leave a side empty for a one-sided bound.")
        pb_df = st.data_editor(
            pd.DataFrame(columns=["property", "lo", "hi"]),
            num_rows="dynamic", key="ig_pbounds", hide_index=True,
            column_config={
                "property": st.column_config.SelectboxColumn(
                    options=list(ig.BOUNDABLE_PROPERTIES), width="medium"),
                "lo": st.column_config.NumberColumn(help="empty = no floor"),
                "hi": st.column_config.NumberColumn(help="empty = no ceiling")})
        pv = _veh(ss)
        p1, p2 = st.columns(2)
        drive_rear = float(p1.number_input(
            "Drive bias to this axle", 0.0, 1.0,
            1.0 if axle == "rear" else 0.0, 0.05,
            help=_HELP["ig_pb_drive"], key="ig_pb_drive"))
        pb_nodes = int(p2.number_input(
            "Property sweep nodes", 3, 21, 5, 1,
            help=_HELP["ig_pb_nodes"], key="ig_pb_nodes"))
        # ---- elastokinematic load case for the compliance_* rows ------- #
        st.markdown("**Compliance under load** — for any `compliance_*` row")
        st.caption("The corner is solved with the chassis pickups on springs: "
                   "the node, bracket and bearing in series at each one. Member "
                   "forces are re-resolved on the deflected geometry at every "
                   "load step, so compliance steer and compliance camber become "
                   "walls like any other property. Use the stiffness your pull "
                   "tests measured; a declared value is reported as declared.")
        c1, c2, c3 = st.columns(3)
        ek_k = float(c1.number_input("Pickup stiffness (kN/mm)", 1.0, 500.0,
                                     30.0, 1.0, key="ig_ek_k",
                                     help="Series stiffness of node, bracket "
                                          "and bearing at every chassis pickup."))
        ek_prov = c2.selectbox("Stiffness source", ["declared", "measured", "fea"],
                               key="ig_ek_prov")
        ek_steps = int(c3.number_input("Load steps", 1, 20, 5, 1, key="ig_ek_steps"))
        c4, c5 = st.columns(2)
        ek_lat = float(c4.number_input("Lateral g at this corner", 0.0, 3.0,
                                       1.5, 0.1, key="ig_ek_lat"))
        ek_fz = float(c5.number_input("Outer-wheel vertical load (N)", 0.0,
                                      10000.0, 1235.0, 5.0, key="ig_ek_fz"))
        if len(pb_df.index):
            st.caption(f"Anti-dive and anti-squat also read the declared "
                       f"CG height ({pv['cg_height_mm']:.0f} mm), wheelbase "
                       f"({pv['wheelbase_mm']:.0f} mm) and front brake bias "
                       f"({pv['brake_bias_front']:.2f}) from the vehicle "
                       f"declaration below. A bound cannot be enforced "
                       f"against an undeclared vehicle.")

    pbounds = []
    for _, r in pb_df.iterrows():
        prop = r.get("property")
        if not prop or (pd.isna(r.get("lo")) and pd.isna(r.get("hi"))):
            continue
        try:
            pbounds.append(ig.PropertyBound(
                str(prop),
                lo=float("-inf") if pd.isna(r.get("lo")) else float(r["lo"]),
                hi=float("inf") if pd.isna(r.get("hi")) else float(r["hi"])))
        except (ValueError, TypeError) as e:
            st.error(f"Property bound skipped: {e}")

    from suspension import elastokinematics as _ek
    properties = None
    if pbounds:
        try:
            properties = ig.SolvedPropertyBounds(
                bounds=pbounds,
                cg_height_mm=float(pv["cg_height_mm"]),
                wheelbase_mm=float(pv["wheelbase_mm"]),
                track_mm=float(track),
                brake_bias_front=float(pv["brake_bias_front"]),
                drive_bias_rear=drive_rear,
                travel_mm=(-float(travel), float(travel)),
                n_nodes=pb_nodes,
                elasto=(_ek.ElastoSpec(
                    stiffness=_ek.StiffnessField(ek_k * 1000.0,
                                                 provenance=ek_prov),
                    Fy=ek_lat * ek_fz, Fz=ek_fz, n_steps=ek_steps)
                    if any(b.prop.startswith("compliance_") for b in pbounds)
                    else None))
        except (ValueError, TypeError) as e:
            st.error(f"Solved-property bounds refused: {e}")
            return

    # ---- the wheel as a solid no link may enter ------------------------- #
    with st.expander("🛞 Wheel envelope — no link may pass through the rim "
                     "or tire", expanded=False):
        st.caption("The solver has no wheel in it: without this, it can "
                   "return a corner whose toe link crosses the tire while "
                   "every curve sits in its band. With it on, the rim and tire "
                   "are a solid the search refuses to enter, checked along "
                   "every link body at every travel station and, if you give "
                   "a rack travel, at both steering locks. Replace the "
                   "defaults with your own wheel and upright.")
        use_env = st.checkbox(
            "Enforce the wheel envelope inside the search", value=False,
            key="ig_env_on",
            help="Off by default only so that existing declarations and "
                 "manifests replay unchanged. Even when off, every result is "
                 "checked against this wheel afterwards and you are told if a "
                 "link passes through it.")
        e1, e2, e3 = st.columns(3)
        env_pick = float(e1.number_input(
            "Upright envelope radius (mm)", 40.0, 250.0, 115.0, 1.0,
            key="ig_env_pick",
            help="Outboard pickups must sit within this radius of the spin "
                 "axis: the space the hub, upright and brake leave."))
        env_rim = float(e2.number_input(
            "Rim inner radius (mm)", 60.0, 300.0, 127.0, 1.0,
            key="ig_env_rim",
            help="Inner radius of the rim/tire solid. 127 mm is a 10-inch "
                 "bead seat."))
        env_tire = float(e3.number_input(
            "Tire outer radius (mm)", 100.0, 400.0, 228.0, 1.0,
            key="ig_env_tire"))
        e4, e5, e6 = st.columns(3)
        env_rimw = float(e4.number_input(
            "Rim half-width (mm)", 20.0, 200.0, 89.0, 1.0, key="ig_env_rimw",
            help="Flange to flange, halved. 89 mm is a 7-inch barrel."))
        env_tirew = float(e5.number_input(
            "Tire section half-width (mm)", 20.0, 250.0, 95.0, 1.0,
            key="ig_env_tirew"))
        env_rack = float(e6.number_input(
            "Rack travel to lock (± mm)", 0.0, 120.0,
            32.0 if axle == "front" else 0.0, 1.0, key="ig_env_rack",
            help="Checked at centre and at both locks. 0 for a rear corner. "
                 "Rack travel ≈ steering arm × sin(road-wheel lock)."))
        e7, e8 = st.columns(2)
        env_off = float(e7.number_input(
            "Rim offset from wheel centre (mm, + outboard)", -100.0, 100.0,
            0.0, 1.0, key="ig_env_off"))
        env_cl = float(e8.number_input(
            "Extra clearance required (mm)", 0.0, 30.0, 0.0, 0.5,
            key="ig_env_cl"))
        e9, e10, e11 = st.columns(3)
        env_lr = float(e9.number_input(
            "Link tube radius (mm)", 0.0, 20.0, 8.0, 0.1, key="ig_env_lr",
            help="Links are checked as tubes, not lines. 8 mm covers a "
                 "5/8-inch wishbone tube. This number decides tight packaging "
                 "cases: use your real tube."))
        env_ro = float(e10.number_input(
            "Rod-end / ball-joint housing radius (mm)", 0.0, 25.0, 11.0, 0.5,
            key="ig_env_ro",
            help="The housing around each outboard pickup must clear the rim "
                 "barrel, not just its centre."))
        env_swing = float(e11.number_input(
            "Rod-end misalignment rating (± deg, 0 = off)", 0.0, 45.0, 0.0,
            0.5, key="ig_env_swing",
            help="Every joint's swing over the travel and both locks must "
                 "stay inside this, or the rod end is loaded in bending. "
                 "Standard rod ends are typically rated around ±13-16 deg; "
                 "high-misalignment parts or spacers go further. Use the "
                 "rating printed for your part."))

    wheel_env = None
    if use_env:
        try:
            wheel_env = ig.WheelEnvelope(
                pickup_radius_mm=env_pick, rim_radius_mm=env_rim,
                tire_radius_mm=env_tire, rim_half_width_mm=env_rimw,
                tire_half_width_mm=env_tirew, wheel_offset_mm=env_off,
                clearance_mm=env_cl, link_radius_mm=env_lr,
                rod_end_radius_mm=env_ro,
                joint_swing_limit_deg=(env_swing if env_swing > 0 else None),
                travel_mm=(-float(travel), float(travel)),
                rack_travel_mm=env_rack)
        except (ValueError, TypeError) as e:
            st.error(f"Wheel envelope refused: {e}")
            return
        seed_v = wheel_env.violations(hp)
        if seed_v:
            w = min(seed_v, key=lambda v: v[2])
            st.warning(f"The seed geometry is already inside the wheel "
                       f"envelope ({w[0]}, {w[2]:+.1f} mm at {w[1]}). The "
                       "search cannot start from it; free the outboard "
                       "pickups or give their boxes room to move out.")

    # ---- chassis pickups at frame nodes ---------------------------------- #
    node_att = None
    if use_frame:
        with st.expander("🔩 Chassis pickups at frame nodes — no brackets "
                         "at tube midspans"):
            st.caption("A pickup at a tube midspan loads the tube in bending "
                       "and needs a long, weak bracket. With this on, each "
                       "chosen pickup must sit within the bracket reach of a "
                       "node of the loaded chassis, and the search refuses "
                       "any step that would strand it.")
            use_nodes = st.checkbox("Enforce node attachment", value=False,
                                    key="ig_nodes_on")
            n_pts = st.multiselect(
                "Pickups that must sit at a node",
                [p for p in ig.DESIGNABLE_POINTS if p.endswith("_inner")],
                default=[p for p in ("upper_front_inner", "upper_rear_inner",
                                     "lower_front_inner", "lower_rear_inner")
                         if p in boxes], key="ig_nodes_pts")
            n_reach = float(st.number_input(
                "Longest bracket you will build (mm)", 5.0, 150.0, 25.0, 1.0,
                key="ig_nodes_reach"))
        if use_nodes and n_pts:
            try:
                node_att = ig.NodeAttachment(
                    nodes=gr.nodes_from_framegraph(ss["tf_frame"],
                                                   axle_station, ground_y),
                    points=tuple(n_pts), max_offset_mm=n_reach)
            except (ValueError, TypeError, KeyError) as e:
                st.error(f"Node attachment refused: {e}")
                return
            nv = node_att.violations(hp)
            if nv:
                st.warning("The seed already has pickups off a node: "
                           + "; ".join(f"{p} {w}" for p, w, _ in nv)
                           + ". Move their boxes over a node.")

    try:
        volume = ig.LegalVolume(boxes=boxes, keep_out=keep_out,
                                probe_radius_mm=float(probe),
                                min_clearance_mm=float(min_cl),
                                spacings=spacings,
                                properties=properties,
                                wheel_envelope=wheel_env,
                                node_attachment=node_att)
    except ValueError as e:
        st.error(f"Legal volume refused: {e}")
        return

    # ================= 3 · shop & search ==================================
    ss["ig_ok_volume"] = True
    st.markdown("###### 4 · How it will be built, and how hard to search")
    _hint(st, ss, "Pick the tolerance your team can really hold. The engine "
                  "simulates thousands of builds at that tolerance and keeps "
                  "the geometry that survives best. The defaults are fine "
                  "for a first run.")
    s1, s2, s3 = st.columns([2, 1, 1])
    shop_label = s1.selectbox("Shop class", list(_SHOPS.keys()),
                              index=1, help=_HELP["ig_shop"], key="ig_shop")
    pull = _units.unum(s2, "Weld pull (mm)", 0.0, 3.0, 0.0, 'mm', step=0.1,
                       help=_HELP["ig_pull"], key="ig_pull")
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
    seed = int(r1.number_input("Seed s", 0, 2**31 - 1, 0, 1, help=_HELP["ig_seed"], key="ig_seed"))
    _hosted = hosted()
    n_starts = int(r2.number_input("Starts", 1, 64, HOSTED_N_STARTS if _hosted else 6, 1,
                                   help=_HELP["ig_nstarts"], key="ig_nstarts"))
    n_yield = int(r3.number_input("Sampled builds N", 100, 50000, HOSTED_N_YIELD if _hosted else 4000, 100,
                                  help=_HELP["ig_nyield"], key="ig_nyield"))
    if _hosted:
        st.caption(f"⚡ Hosted app: a quick search by default ({HOSTED_N_STARTS} start, the seed geometry, and "
                   f"N = {HOSTED_N_YIELD} sampled builds): seconds, where the full {FULL_N_STARTS}-start search "
                   f"takes 20 minutes or more and the shared CPU throttles it before it finishes. The result is "
                   f"labelled quick and says what it left out; for a result you will quote, run KinematiK on your "
                   f"own machine with {FULL_N_STARTS} starts and N = {FULL_N_YIELD}.")
        if n_starts > HOSTED_N_STARTS:
            st.warning("Each start after the first is drawn until it clears every declared wall, which can take "
                       "minutes per start: on the hosted app this search may be throttled before it finishes.")
    n_verify = int(r4.number_input("Full-solve verification", 0, 2000, 0,
                                   10, help=_HELP["ig_nverify"], key="ig_nverify"))
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
    from suspension import genesis_seed as gs
    b1, b2 = st.columns([3, 1])
    seed_first = b1.checkbox(
        "If the start breaks a wall, search for a start inside the walls first "
        "(pattern search), then run the search from there", value=True, key="ig_seed_first",
        help="The search refuses a start inside a wall and would return NO_FIT. With this on, "
             "a start that breaks one is first moved, inside the same boxes and one coordinate "
             "at a time, until every wall holds with the curves near their bands. The step is "
             "declared in the manifest, so a re-run repeats it; if it cannot reach the walls, "
             "the result says which ones it could not meet. A start inside every wall is "
             "searched from directly, as before.")
    seed_budget = int(b2.number_input("Evaluations", 50, 10000, 1500, 50,
                                      key="ig_seed_budget", disabled=not seed_first))
    name = st.text_input("Run name", f"{axle}_corner", help=_HELP["ig_name"], key="ig_name")

    # A run started here uses the form, never an uploaded manifest. Say so on
    # the control itself rather than only in the section above, because this
    # is the button a user reaches by scrolling.
    st.caption("🧬 Generate runs **the fields on this page**. To reproduce a "
               "published result instead, use *Re-run a saved manifest* at "
               "the top of the tab.")
    if pbounds:
        st.caption("Enforcing inside the search: "
                   + "; ".join(p.label for p in pbounds))
    if st.button("🧬 Generate the geometry (from the fields above)",
                 key="ig_run_btn", type="primary",
                 disabled=_man_loaded,
                 help="Disabled while a manifest is loaded, so an uploaded "
                      "result cannot be silently replaced by a form run."
                      if _man_loaded else None):
        search = gr.SearchSettings(seed=seed, n_starts=n_starts,
                                   n_yield=n_yield, n_verify_full=n_verify,
                                   max_iter=max_iter, step_mm=step,
                                   resilient_yield=th_r, tempered_yield=th_t,
                                   verify_agreement=th_v)
        ctx = {"axle": axle, "geometry_source": src, "entered": "form"}
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
        broken = gs.walls_broken(hp, targets, volume) if seed_first else []
        if broken:
            st.info("The start breaks " + ", ".join(broken) + ": searching for a start "
                    "inside the walls first.")
            man.start_search = asdict(gs.SeedSearch(max_evals=seed_budget, step_min_mm=0.125))
        with st.spinner(("Searching for a start inside the walls, then " if broken else "")
                        + "reverse gradients pulling the points into the "
                        "curves, pricing build yield…"):
            try:
                res = man.run()
            except ValueError as e:
                st.error(f"Engine refused: {e}")
                return
        man.record(res)
        ss["ig_run"] = {"manifest_json": man.to_json(), "result": res,
                        "verify": None, "inputs_sha256": man.inputs_sha256, **ctx}
        rc.reset(ss)                          # the report is about this run from here on
        ss["ig_report_identity"] = report_identity(ss, False)
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
    _actuation_panel(st, pd, np, ss, run)


def _actuation_panel(st, pd, np, ss, run):
    """Second stage: design the pushrod, rocker and spring on the returned corner.

    Actuation points do not enter camber, toe or roll centre, so this cannot
    spoil the kinematic result. Targets are derived from the damper stroke.
    """
    res = run.get("result")
    if res is None or res.winner_hp is None:
        return
    if getattr(res.winner_hp, "actuation", "rocker") == "direct":
        _direct_damper_panel(st, pd, np, ss, res)
        return
    from suspension import actuation as _ac
    with st.expander("\u2699\ufe0f Actuation synthesis \u2014 design the pushrod, rocker "
                     "and spring for this corner"):
        st.caption("The motion ratio is set as high as the damper stroke allows, "
                   "because more damper travel per millimetre of wheel travel "
                   "gives more usable damping resolution, and its spread across "
                   "travel is bounded so the wheel rate, and the ride frequency, "
                   "stay near constant. The pushrod and spring must act close to "
                   "the rocker plane and away from toggle.")
        a1, a2, a3 = st.columns(3)
        stroke = float(a1.number_input("Damper stroke (mm)", 10.0, 200.0, 57.0,
                                       1.0, key="ig_act_stroke"))
        spread = float(a2.number_input("Max motion-ratio spread (%)", 1.0, 50.0,
                                       10.0, 1.0, key="ig_act_spread")) / 100.0
        box = float(a3.number_input("Point movement box (\u00b1 mm)", 5.0, 100.0,
                                    40.0, 1.0, key="ig_act_box"))
        b1, b2 = st.columns(2)
        hz = float(b1.number_input("Ride frequency at static (Hz)", 0.5, 8.0,
                                   2.5, 0.05, key="ig_act_hz"))
        ms = float(b2.number_input("Sprung corner mass (kg)", 5.0, 300.0, 60.1,
                                   0.1, key="ig_act_ms"))
        if not st.button("Synthesise actuation", key="ig_act_go"):
            return
        t = _ac.ActuationTargets.derived(stroke_mm=stroke, max_spread=spread)
        with st.spinner("Designing the linkage\u2026"):
            r = _ac.synthesize_actuation(res.winner_hp, t, box_mm=box,
                                         ride_hz=hz, sprung_corner_kg=ms)
        d, m = r.seed_metrics, r.metrics
        rows = [{"": "default linkage", "motion ratio": d.mr_static,
                 "spread (%)": 100 * d.spread, "stroke used (mm)": d.stroke_used_mm,
                 "out of plane (deg)": d.out_of_plane_deg,
                 "transmission min (deg)": d.transmission_deg[0]},
                {"": "synthesised", "motion ratio": m.mr_static,
                 "spread (%)": 100 * m.spread, "stroke used (mm)": m.stroke_used_mm,
                 "out of plane (deg)": m.out_of_plane_deg,
                 "transmission min (deg)": m.transmission_deg[0]}]
        st.dataframe(pd.DataFrame(rows), hide_index=True, width="stretch")
        if r.ok:
            st.success("Every actuation band met.")
        else:
            st.warning("Closest linkage shown; not every band is met. Widen the "
                       "movement box or relax the spread.")
        pts = [{"point": p, "x": getattr(r.hp, p)[0], "y": getattr(r.hp, p)[1],
                "z": getattr(r.hp, p)[2]} for p in _ac.ACTUATION_POINTS]
        st.dataframe(pd.DataFrame(pts), hide_index=True, width="stretch")
        if r.ride_frequency_hz:
            st.dataframe(pd.DataFrame({"travel (mm)": m.travel,
                                       "ride frequency (Hz)": r.ride_frequency_hz,
                                       "spring rate (N/mm)": [r.spring_rate_N_per_mm] * len(m.travel)}),
                         hide_index=True, width="stretch")


def _direct_damper_panel(st, pd, np, ss, res):
    """Second stage for a direct-acting corner: the damper rides with the body it is bolted to, so its outer end is
    carried onto the returned corner's wishbone (or upright); the user can then move either end."""
    from suspension import genesis_analysis as ga
    from suspension import genesis_repro as gr
    with st.expander("\u2699\ufe0f Direct-acting damper \u2014 place the damper on this corner"):
        whp = res.winner_hp.copy()
        seed = ss.get("ig_seed_hp")
        if seed is not None:
            carried = ga.carry_actuation_point(gr.hp_from_dict(seed), whp)
            if carried is not None:
                whp.pushrod_outer = carried
        st.caption("The damper's outer end keeps its position on the " + whp.pushrod_attach + (
            " wishbone" if whp.pushrod_attach != "upright" else "") + " as the search moved it; the chassis mount "
            "stays where it was declared. Edit either end below.")
        whp = _actuation_selector(st, whp, "ig_dd", title="Actuation of the returned corner")
        if getattr(whp, "actuation", "rocker") != "direct":
            st.info("Switched to a rocker: design it with the rocker synthesis after rerunning with a rocker "
                    "selected in step 1.")
            return
        a = ga.actuation_summary(whp)
        if not a.get("ok"):
            st.warning(a["reason"])
            return
        _table(st, pd, [("motion ratio, static", a["mr_static"]), ("motion ratio, droop", a["mr_droop"]),
                        ("motion ratio, bump", a["mr_bump"]), ("rate character", a["rate_character"]),
                        ("damper static length (mm)", a["damper_static_mm"]),
                        ("damper travel used (mm)", a["damper_travel_used_mm"]),
                        ("motion-ratio spread (%)", a["mr_spread_pct"])], ["quantity", "value"])
        rows = [{"point": p, "x": getattr(whp, p)[0], "y": getattr(whp, p)[1], "z": getattr(whp, p)[2]}
                for p in ("pushrod_outer", "spring_inner")]
        st.dataframe(pd.DataFrame(rows).replace({"pushrod_outer": "damper_outer", "spring_inner": "damper_inner"}),
                     hide_index=True, width="stretch")
        ss["ig_direct_hp"] = gr.hp_to_dict(whp)


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
    ("Motion ratio when the corner has no rocker", [
        ("mr_front_droop", "Front MR at full droop", 0.05, 3.0, 0.001, 0.585),
        ("mr_front_static", "Front MR at ride height", 0.05, 3.0, 0.001, 0.600),
        ("mr_front_bump", "Front MR at full bump", 0.05, 3.0, 0.001, 0.596),
        ("mr_rear_droop", "Rear MR at full droop", 0.05, 3.0, 0.001, 0.603),
        ("mr_rear_static", "Rear MR at ride height", 0.05, 3.0, 0.001, 0.620),
        ("mr_rear_bump", "Rear MR at full bump", 0.05, 3.0, 0.001, 0.630),
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


def manifest_vehicle(man, axle="front"):
    """What a manifest declared about the car, as section-5 fields, and what it declared that section 5 cannot hold.

    Returns (fields, not_held, link_tubes): fields maps section-5 field names to the manifest's values (mm, kg,
    fraction, MPa, GPa, N·m); not_held lists sentences for declarations the analysis sections have no field for;
    link_tubes maps a link to its own (od, wall) in mm where the manifest sizes links individually. A manifest's
    solved-property bounds are computed from these values; the review sections below a run read section 5, so
    without them the same car passed a bound in the search and failed the matching review row."""
    props = ((man.volume or {}).get("properties") or {}) if isinstance(man.volume, dict) else {}
    out, missing = {}, []
    brake = props.get("brake") or {}
    for f, val in (("cg_height_mm", props.get("cg_height_mm", brake.get("cg_height_mm"))),
                   ("wheelbase_mm", props.get("wheelbase_mm", brake.get("wheelbase_mm"))),
                   (f"track_{axle}_mm", props.get("track_mm", brake.get("track_mm"))),
                   ("brake_bias_front", props.get("brake_bias_front")),
                   ("mass_kg", brake.get("mass_kg"))):
        if val is not None:
            out[f] = float(val)
    st_ = props.get("structure") or {}
    for f, k in (("tube_od_mm", "od_mm"), ("tube_wall_mm", "wall_mm"), ("yield_mpa", "sy_mpa")):
        if st_.get(k) is not None:
            out[f] = float(st_[k])
    if st_.get("e_mpa") is not None:
        out["E_gpa"] = float(st_["e_mpa"]) / 1000.0
    per = {m: (float(od), float(w)) for m, (od, w) in (st_.get("per_member") or {}).items()}
    se = props.get("steer_effort") or {}
    if se.get("effort_limit_nm") is not None:
        out["steering_target_Nm"] = float(se["effort_limit_nm"])
    if se.get("rack_c_mm_per_rev") is not None:
        missing.append(f"the steering rack ({se['rack_c_mm_per_rev']:g} mm per revolution, lock "
                       f"{se.get('lock_road_wheel_deg', '?')}°). The search bounded that rack's effort margin from "
                       "trail alone; the review's 'steering ratio needed' also adds the tyre aligning moment of "
                       "section 5, a default unless you entered it, which is most of the difference between them")
    cases = list(st_.get("cases") or {})
    if cases or props.get("elasto"):
        missing.append("its own structural load cases" + (f" ({len(cases)}, such as {cases[0]})" if cases else "")
                       + " and pickup flexibility. The bounds table's link factor and compliance steer come from "
                         "those; the review's link and compliance tables use five generic load cases and link "
                         "stiffness only, so the two differ")
    return out, missing, per


def apply_manifest_vehicle(ss, man, axle="front"):
    """Set section 5 to the car a manifest declared (see manifest_vehicle) and remember what was applied and what
    could not be, for the review to say. Returns (fields, not_held)."""
    from suspension import genesis_analysis as ga
    fields, missing, per = manifest_vehicle(man, axle)
    v = _veh(ss)
    if per:
        v["link_tubes"] = {m: list(t) for m, t in per.items()}
    else:
        v.pop("link_tubes", None)
    for f, val in fields.items():
        v[f] = val
        for p in _VEH_PREFIXES:
            ss[f"{p}_{f}"] = val
    # A bound the manifest declared is the team's range for that check: the review judges the delivered value
    # against it, not against the generic starting range, or the same number passes the search and fails the review.
    review_key = {"caster_deg": "caster_deg", "kpi_deg": "kpi_deg", "scrub_static_mm": "scrub_mm",
                  ("anti_dive_pct" if axle == "front" else "anti_squat_pct"): "anti_pct"}
    limits = ss.get("ig_review_limits") or [dict(r) for r in ga.DEFAULT_REVIEW_LIMITS]
    by_key = {r["key"]: r for r in limits}
    ranges = {}
    props = ((man.volume or {}).get("properties") or {}) if isinstance(man.volume, dict) else {}
    for b in props.get("bounds") or []:
        k = review_key.get(b.get("prop"))
        if k in by_key:
            lo = -1e9 if b.get("lo") is None else float(b["lo"])
            hi = 1e9 if b.get("hi") is None else float(b["hi"])
            by_key[k]["lo"], by_key[k]["hi"] = lo, hi
            ranges[by_key[k]["check"]] = (b.get("lo"), b.get("hi"))
    ss["ig_review_limits"] = limits
    se = props.get("steer_effort") or {}
    ss["ig_veh_from_manifest"] = {"name": man.name, "fields": fields, "not_held": missing, "ranges": ranges,
                                  "link_tubes": {m: list(t) for m, t in per.items()},
                                  "lock_road_wheel_deg": se.get("lock_road_wheel_deg")}
    return fields, missing


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
    """Readable number: precision by magnitude, trailing zeros trimmed."""
    if isinstance(v, bool) or v is None:
        return "—" if v is None else ("yes" if v else "no")
    if isinstance(v, int):
        return str(v)
    try:
        x = float(v)
    except (TypeError, ValueError):
        return str(v)
    if x != x:
        return "—"
    a = abs(x)
    if a >= 1e8:
        return "∞" if x > 0 else "−∞"
    if a >= 100:
        t = str(round(x, 1))
    elif a >= 1:
        t = str(round(x, 3))
    elif a >= 0.001 or a == 0:
        t = str(round(x, 4))
    else:
        t = "%.3g" % x  # noqa: UP031 -- an f-string here would count as an ungated render in test_repo_accuracy_audit
    if "." in t and "e" not in t:
        t = t.rstrip("0").rstrip(".")
    return "0" if t in ("-0", "") else t


def _split_unit(label):
    """'camber gain (deg/mm)' → ('camber gain', 'deg/mm')."""
    label = str(label)
    if label.endswith(")") and " (" in label:
        head, _, unit = label.rpartition(" (")
        return head, unit[:-1]
    return label, ""


def _table(st, pd, rows, cols=None, title=None):
    """Render rows with readable numbers. A two-column quantity/value table
    gets its units split into their own column. ``title`` names the table in
    the feature report, where it is printed above it: without one a table is
    known there only by its shape, and two of one shape are one."""
    ss = getattr(st, "session_state", None)
    if title and ss is not None:
        from suspension import report_capture as rc
        rc.heading(ss, title)
    df = pd.DataFrame(rows, columns=cols) if cols else pd.DataFrame(rows)
    if len(df.columns) == 2 and str(df.columns[1]).startswith("value"):
        q = [_split_unit(x) for x in df.iloc[:, 0]]
        df = pd.DataFrame({df.columns[0]: [a for a, _ in q],
                           "value": list(df.iloc[:, 1]),
                           "unit": [u for _, u in q]})
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
def _sec_steering(st, pd, v, front_hp, rear_hp, key, rack_ratio=None):
    from suspension import genesis_analysis as ga
    from suspension.kinematics import SuspensionKinematics
    st.caption("Kingpin torque from mechanical trail (where the kingpin axis meets the road) and aligning torque, "
               "with the corner loads taken from the declared vehicle at the "
               "design lateral acceleration.")
    try:
        car = _declared_car(v, front_hp, rear_hp)
        loads, _ = car.vehicle().lateral_load_transfer(v["lateral_g"])
        fz_in, fz_out = float(loads.fl), float(loads.fr)
        from suspension import inverse_genesis as ig
        st0 = SuspensionKinematics(front_hp).solve_at_travel(0.0)
        caster = float(st0.caster)
        trail = float(ig.mechanical_trail_mm(st0))   # where the kingpin axis meets the road, not R·tan(caster)
    except Exception as e:                    # noqa: BLE001
        st.error(f"Vehicle model failed: {e}")
        return
    ratios = _floats(v["steering_ratios"], (4, 5, 6, 8))
    s = ga.steering_torque(caster, v["tire_radius_mm"], fz_out, fz_in,
                           v["lateral_mu"], v["aligning_outer_Nm"],
                           v["aligning_inner_Nm"], ratios,
                           v["steering_target_Nm"], trail_mm=trail)
    _table(st, pd, [("caster, from the front corner (deg)", caster),
                    ("outer front Fz (N)", fz_out),
                    ("inner front Fz (N)", fz_in),
                    ("mechanical trail, kingpin axis at the road (mm)", s["trail_mm"]),
                    ("outer Fy (N)", s["fy_outer_N"]),
                    ("outer trail torque (N·m)", s["trail_torque_outer_Nm"]),
                    ("outer tyre aligning moment, declared (N·m)", v["aligning_outer_Nm"]),
                    ("inner tyre aligning moment, declared (N·m)", v["aligning_inner_Nm"]),
                    ("outer kingpin torque, trail + aligning (N·m)", s["outer_Nm"]),
                    ("inner kingpin torque, trail + aligning (N·m)", s["inner_Nm"]),
                    ("road-wheel total (N·m)", s["road_wheel_total_Nm"]),
                    ("steering-wheel torque target, declared (N·m)", v["steering_target_Nm"]),
                    ("ratio needed for the target", s["ratio_needed_for_target"]),
                    ("road-wheel total from trail alone, no aligning moment (N·m)",
                     s["road_wheel_total_Nm"] - v["aligning_outer_Nm"] - v["aligning_inner_Nm"]),
                    ("ratio needed from trail alone, no aligning moment",
                     (s["road_wheel_total_Nm"] - v["aligning_outer_Nm"] - v["aligning_inner_Nm"])
                     / v["steering_target_Nm"])]
           + ([] if not rack_ratio else [
               ("steering ratio of the declared rack, at lock", rack_ratio),
               ("steering-wheel torque at the rack's ratio, trail alone (N·m)",
                (s["road_wheel_total_Nm"] - v["aligning_outer_Nm"] - v["aligning_inner_Nm"]) / rack_ratio),
               ("steering-wheel torque at the rack's ratio, with the declared aligning moments (N·m)",
                s["road_wheel_total_Nm"] / rack_ratio)]),
           ["quantity", "value"], title="Steering effort at the design lateral acceleration")
    _table(st, pd, s["rows"], title="Steering-wheel torque by steering ratio")
    return s


def _sec_actuation(st, pd, v, corner, axle):
    from suspension import genesis_analysis as ga
    k = v[f"spring_{axle}_lbin"] * _LBIN
    m = v[f"sprung_{axle}_kg"]
    trk = v[f"track_{axle}_mm"]
    a = ga.actuation_summary(corner, spring_rate_N_mm=k,
                             sprung_corner_mass_kg=m,
                             damper_stroke_mm=v["damper_stroke_mm"])
    mr = None
    if a.get("ok") and a.get("rate_character") == "falling":
        st.warning(f"The motion ratio falls in bump ({_cell(a['mr_static'])} at ride height, {_cell(a['mr_bump'])} at "
                   "full bump): the wheel rate drops as the suspension compresses, the opposite of what resists "
                   "bottoming. Give the reason, or move the rocker or damper mounts until it is flat or rising.")
    if a.get("ok"):
        mr = a["mr_static"]
        _table(st, pd, [
            ("motion ratio, static", a["mr_static"]),
            ("motion ratio, droop", a["mr_droop"]),
            ("motion ratio, bump", a["mr_bump"]),
            ("rate character", a["rate_character"]),
            ("actuation", "direct-acting damper" if a.get("actuation") == "direct"
             else "pushrod + rocker"),
            ("pushrod length (mm)" if a.get("actuation") != "direct"
             else "damper length at static (mm)", a["pushrod_length_mm"]),
            ("attachment, fraction out along the " + a["pushrod_attach"]
             + " arm", a["attachment_fraction"]),
            ("damper static length (mm)", a["damper_static_mm"]),
            ("damper travel used (mm)", a["damper_travel_used_mm"]),
            ("fraction of stroke used", a.get("stroke_fraction_used")),
            ("motion-ratio spread (%)", a["mr_spread_pct"]),
            ("wheel-rate spread (%)", a["wheel_rate_spread_pct"]),
            ("ride frequency, droop, wheel rate only, no tyre in series (Hz)", a["ride_hz_droop"]),
            ("ride frequency, static, wheel rate only, no tyre in series (Hz)", a["ride_hz_static"]),
            ("ride frequency, bump, wheel rate only, no tyre in series (Hz)", a["ride_hz_bump"])],
            ["quantity", "value"], title="Actuation and ride, " + axle + " corner (solved from the linkage)")
        st.line_chart(pd.DataFrame({"motion ratio": a["motion_ratio"]},
                                   index=pd.Index(a["travel_mm"],
                                                  name="travel (mm)")),
                      height=180)
    else:
        st.warning("No complete actuation on this corner ("
                   + a["reason"] + "). Using the DECLARED motion-ratio curve "
                   "from the vehicle & build declaration — pushrod and damper "
                   "lengths need the rocker points.")
        dm = ga.declared_mr_summary(v[f"mr_{axle}_droop"],
                                    v[f"mr_{axle}_static"],
                                    v[f"mr_{axle}_bump"], k, m)
        _table(st, pd, [
            ("motion ratio, static — declared", dm["mr_static"]),
            ("motion ratio, droop — declared", dm["mr_droop"]),
            ("motion ratio, bump — declared", dm["mr_bump"]),
            ("rate character", dm["rate_character"]),
            ("motion-ratio spread (%)", dm["mr_spread_pct"]),
            ("wheel-rate spread (%)", dm["wheel_rate_spread_pct"]),
            ("ride frequency, droop, wheel rate only, no tyre in series (Hz)", dm["ride_hz_droop"]),
            ("ride frequency, static, wheel rate only, no tyre in series (Hz)", dm["ride_hz_static"]),
            ("ride frequency, bump, wheel rate only, no tyre in series (Hz)", dm["ride_hz_bump"])],
            ["quantity", "value"], title="Actuation and ride, " + axle + " corner (DECLARED motion ratio)")
        a = dict(a, wheel_rate_spread_pct=dm["wheel_rate_spread_pct"])
    solved = mr is not None
    mr_used = mr if solved else float(v[f"mr_{axle}_static"])
    rs = ga.roll_stiffness_from_spring(k, mr_used, trk)
    _table(st, pd, [
        ("motion ratio used", str(round(mr_used, 4))
         + (" (solved)" if solved else " (declared)")),
        ("wheel rate (N/mm)", rs["wheel_rate_N_mm"]),
        ("spring roll stiffness (N·m/deg)", rs["roll_stiffness_Nm_deg"]),
        ("same with MR = 1.0 placeholder (N·m/deg)", rs["placeholder_Nm_deg"]),
        ("placeholder error (%)", rs["placeholder_error_pct"])],
        ["quantity", "value"], title="Wheel rate and spring roll stiffness, " + axle + " axle")
    return a


def _sec_structural(st, pd, v, corner, axle):
    from suspension import genesis_repro as gr
    tube = gr.TubeSpec(od_mm=v["tube_od_mm"], wall_mm=v["tube_wall_mm"],
                       yield_mpa=v["yield_mpa"], E_gpa=v["E_gpa"])
    tubes = {m: gr.TubeSpec(od_mm=od, wall_mm=w, yield_mpa=v["yield_mpa"], E_gpa=v["E_gpa"])
             for m, (od, w) in (v.get("link_tubes") or {}).items()}
    sc = gr.structural_screening(
        corner, tube=tube, tubes=tubes or None, fos_min=v["fos_min"], axle=axle,
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
                    for m, f in sc["worst_fos_per_member"].items()],
           title="Link factor of safety, worst case per member")
    _table(st, pd, sc["rows"], title="Link forces and factor of safety, every load case (N, + tension)")
    return sc


#: The parts inputs, with the value each opens with (a tool default until the team changes it).
PARTS_INPUTS = (
    ("ig_parts_upright_yield", "Upright material yield (MPa)", 50.0, 2000.0, 503.0,
     "7075-T6 plate, a common machined upright"),
    ("ig_parts_arm_z", "Upright arm section modulus at its root (mm³), 0 = not declared", 0.0, 1e6, 0.0, ""),
    ("ig_parts_brg_spacing", "Hub bearing spacing along the spin axis (mm)", 5.0, 200.0, 40.0, ""),
    ("ig_parts_brg_c0", "Hub bearing static load rating C0 (kN), 0 = not declared", 0.0, 500.0, 0.0, ""),
    ("ig_parts_fat", "Link tube-to-insert weld, IIW detail class FAT (MPa)", 20.0, 200.0, 71.0,
     "check your weld detail in the IIW table"),
    ("ig_parts_cycles", "Fatigue design life, full-load service cycles (count)", 1e3, 1e9, 1e5,
     "a season of hard braking and cornering with margin"),
    ("ig_parts_re_mu", "Rod-end ball friction coefficient (-)", 0.01, 0.5, 0.1, "PTFE-lined"),
    ("ig_parts_re_dia", "Rod-end shank nominal diameter (mm)", 3.0, 30.0, 9.525, "3/8 in"),
    ("ig_parts_re_tpi", "Rod-end shank threads per inch (1/in)", 8.0, 80.0, 24.0, "3/8-24 UNF"),
    ("ig_parts_re_ball", "Rod-end ball radius (mm)", 2.0, 30.0, 9.5, ""),
)


def _sec_parts(st, pd, ss, v, corner, axle, sc, brk):
    """Brackets, upright, hub bearings and fatigue under the link screen's load cases (suspension/corner_parts.py).
    Forces in N, lengths in mm, stresses in MPa, factors dimensionless."""
    import numpy as np
    from suspension import corner_parts as cp
    from suspension import genesis_repro as gr
    st.caption("Closes the load path from the tyre to the frame with hand calculations on the link screen's load "
               "cases: the upright's joint loads (checked for equilibrium), its arms as cantilevers, the hub "
               "bearings, the pickup brackets, and fatigue of the link welds and rod-end shanks. A screen ahead of "
               "FEA, not a replacement: where a part is not declared it shows what the part must have.")
    c = st.columns(4)
    val = {}
    for i, (key, label, lo, hi, default, _note) in enumerate(PARTS_INPUTS):
        val[key] = float(c[i % 4].number_input(label, lo, hi, default, key=key))
    _table(st, pd, [{"input": label, "value": val[key],
                     "source": "entered" if abs(val[key] - default) > 1e-9 else
                     ("default" + (f": {note}" if note else ""))}
                    for key, label, lo, hi, default, note in PARTS_INPUTS],
           title="Inputs to the parts screen, and where each came from")
    cases = gr.vehicle_load_cases(
        axle=axle, mass_kg=v["mass_kg"], weight_dist_front=v["weight_dist_front"], cg_height_mm=v["cg_height_mm"],
        track_mm=v[f"track_{axle}_mm"], wheelbase_mm=v["wheelbase_mm"], brake_bias_front=v["brake_bias_front"],
        roll_share_front=v["roll_share_front"], aligning_torque_Nm=v["aligning_outer_Nm"])
    loads = cp.upright_loads(corner, cases)
    _table(st, pd, [{"load case": r["load_case"], "joint": r["joint"], "Fx (N)": r["F"][0], "Fy (N)": r["F"][1],
                     "Fz (N)": r["F"][2], "magnitude (N)": float(np.linalg.norm(r["F"]))} for r in loads],
           title="Upright joint loads, every load case (N, corner axes; the FEA load set)")
    resid = max((r["residual_N"] for r in loads), default=0.0)
    arms = cp.upright_arms(loads, corner.wheel_center, val["ig_parts_upright_yield"],
                           {j: val["ig_parts_arm_z"] for j in cp.JOINTS} if val["ig_parts_arm_z"] > 0 else None,
                           fos_target=v["fos_min"])
    _table(st, pd, [{"arm": a["joint"], "length (mm)": a["arm_mm"], "worst moment (N·mm)": a["moment_Nmm"],
                     "case": a["load_case"], "section modulus needed (mm³)": a["Z_required_mm3"],
                     "FoS, declared section": a["fos"] if a["fos"] is not None else "not declared"} for a in arms],
           title=f"Upright arms as cantilevers from the wheel centre (FoS {_cell(v['fos_min'])} on yield)")
    off = float(corner.contact_patch[1] - corner.wheel_center[1])
    brg = cp.hub_bearings(cases, v["tire_radius_mm"], val["ig_parts_brg_spacing"], off,
                          c0_kN=val["ig_parts_brg_c0"] or None)
    _table(st, pd, [{"load case": b["load_case"], "radial (N)": b["Fr_N"], "axial (N)": b["Fa_N"],
                     "P0 (N)": b["P0_N"], "C0 needed for s0 = 2 (kN)": b["C0_required_kN"],
                     "s0, declared bearing": b["s0"] if b["s0"] is not None else "not declared"} for b in brg],
           title="Hub bearings: static load on the more loaded bearing (ISO 76)")
    g = (brk or {}).get("as_drawn") or {"reach_mm": _BRACKET_EXAMPLE["reach (mm)"][0],
                                        "plate_width_mm": _BRACKET_EXAMPLE["plate width (mm)"][0],
                                        "plate_thickness_mm": _BRACKET_EXAMPLE["plate thickness (mm)"][0]}
    bdecl = bool((brk or {}).get("declared"))
    br = cp.brackets(sc["rows"], g["reach_mm"], g["plate_width_mm"], g["plate_thickness_mm"], v["bracket_plates"],
                     v["yield_mpa"], v["bracket_kt"])
    _table(st, pd, [{"member": b["member"], "force (N)": b["force_N"], "case": b["load_case"],
                     "stress (MPa)": b["stress_mpa"], "FoS": b["fos"]} for b in br],
           title=("Pickup brackets at each link's worst force, the bracket as drawn" if bdecl else
                  "Pickup brackets at each link's worst force, the tool's example bracket (enter yours above)"))
    tube = gr.TubeSpec(od_mm=v["tube_od_mm"], wall_mm=v["tube_wall_mm"], yield_mpa=v["yield_mpa"], E_gpa=v["E_gpa"])
    areas = {m: gr.TubeSpec(od_mm=od, wall_mm=w).area for m, (od, w) in (v.get("link_tubes") or {}).items()}
    areas = {m: areas.get(m, tube.area) for m in {r["member"] for r in sc["rows"]}}
    weld = cp.link_weld_fatigue(sc["rows"], areas, val["ig_parts_fat"], val["ig_parts_cycles"])
    _table(st, pd, [{"member": w["member"], "stress range (MPa)": w["range_MPa"],
                     "allowable range at the design life (MPa)": w["limit_MPa"], "factor": w["fos"]}
                    for w in weld],
           title="Fatigue: link welds over the repeated service cases (IIW slope 3, at the design life)")
    rod = cp.rod_end_fatigue(sc["rows"], val["ig_parts_re_dia"], val["ig_parts_re_tpi"], val["ig_parts_re_ball"],
                             val["ig_parts_re_mu"], val["ig_parts_cycles"])
    _table(st, pd, [{"member": r["member"], "service force (N)": r["force_N"],
                     "stress amplitude (MPa)": r["stress_amp_MPa"],
                     "allowable at the design life (MPa)": r["endurance_amp_MPa"], "factor": r["fos"]} for r in rod],
           title="Fatigue: rod-end shank bending from ball friction (VDI 2230, at the design life)")
    return {"residual_N": resid, "arms": arms, "bearings": brg, "brackets": br, "bracket_declared": bdecl,
            "weld": weld, "rod": rod, "inputs": val, "cases": [lc.name for lc in cases]}


def _sec_compliance(st, pd, v, corner, axle, sc, bands):
    from suspension import genesis_repro as gr
    from suspension import genesis_analysis as ga
    link_tubes = {m: tuple(t) for m, t in (v.get("link_tubes") or {}).items()}
    cb = gr.compliance_budget(corner, axle=axle, od_mm=v["tube_od_mm"],
                              wall_mm=v["tube_wall_mm"], tubes=link_tubes or None,
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
    _table(st, pd, rows, title="Compliance camber and steer against the tolerance bands")
    over = [r for r in rows if r["share of toe band (%)"] > 50.0]
    if over and len(over) < len(rows):
        calm = next(r for r in rows if r not in over)
        st.warning(f"Compliance steer is over half the toe band only in the {over[0]['case']} case "
                   f"({_cell(over[0]['compliance steer (deg)'])}°, {_cell(over[0]['share of toe band (%)'])} % of "
                   f"the band); in {calm['case']} it is {_cell(calm['compliance steer (deg)'])}° "
                   f"({_cell(calm['share of toe band (%)'])} %). The review row takes the larger. Link stiffness "
                   "only: brackets, chassis and upright add to it.")
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
        od, wall = link_tubes.get(m, (v["tube_od_mm"], v["tube_wall_mm"]))
        b = ga.axial_budget(abs(r["force_N"]), r["length_mm"], od, wall, v["E_gpa"],
                            v["rod_end_lash_mm"], 2)
        lrows.append({"member": m, "tube od × wall (mm)": _cell(od) + " × " + _cell(wall),
                      "worst force (N)": r["force_N"],
                      "case": r["load_case"], "length (mm)": r["length_mm"],
                      "axial stretch (mm)": b["axial_strain_mm"],
                      "rod-end lash (mm)": b["lash_mm"],
                      "total (mm)": b["sum_mm"]})
    _table(st, pd, lrows, title="Link stretch at each member's worst force (axial stretch + rod-end lash)")

    st.markdown("**Check a stated link and stated deflections**")
    c = st.columns(3)
    F = c[0].number_input("Link force (N)", 0.0, 100000.0, _STATED_EXAMPLE[0], 1.0,
                          key="ig_cmp_F")
    L = c[1].number_input("Link length (mm)", 1.0, 5000.0, _STATED_EXAMPLE[1], 1.0,
                          key="ig_cmp_L")
    rep = c[2].number_input("Reported max extension (mm)", 0.0, 50.0, _STATED_EXAMPLE[2],
                            0.01, key="ig_cmp_rep")
    c = st.columns(2)
    dcam = c[0].number_input("Stated camber loss (deg)", 0.0, 10.0, _STATED_EXAMPLE[3],
                             0.001, format="%.3f", key="ig_cmp_dcam")
    dtoe = c[1].number_input("Stated compliance steer (deg)", 0.0, 10.0,
                             _STATED_EXAMPLE[4], 0.001, format="%.3f", key="ig_cmp_dtoe")
    stated = is_declared((F, L, rep, dcam, dtoe), _STATED_EXAMPLE)
    if not stated:
        st.caption("These are example values, not this corner's: the two tables "
                   "under them are left out of the report until you enter a "
                   "link and deflections of your own.")
    from suspension import report_capture as rc
    _ss = getattr(st, "session_state", {})
    if not stated:
        rc.forget(_ss, _STATED_LINK_TITLE, _STATED_BANDS_TITLE)
    with rc.unreported(_ss, not stated):
        b = ga.axial_budget(F, L, v["tube_od_mm"], v["tube_wall_mm"], v["E_gpa"],
                            v["rod_end_lash_mm"], 2, rep)
        _table(st, pd, [("stated link force (N)", F),
                        ("stated link length (mm)", L),
                        ("reported extension (mm)", rep),
                        ("tube area (mm²)", b["area_mm2"]),
                        ("axial stretch (mm)", b["axial_strain_mm"]),
                        ("rod-end lash, two joints (mm)", b["lash_mm"]),
                        ("stretch + lash (mm)", b["sum_mm"]),
                        ("unexplained part of the reported extension (mm)",
                         b["unexplained_mm"])], ["quantity", "value"],
               title=_STATED_LINK_TITLE)
        fr = ga.band_fractions({"camber": dcam, "toe": dtoe}, bands)
        _table(st, pd, [{"channel": ch, "stated deflection (deg)": {"camber": dcam, "toe": dtoe}[ch],
                         "band (± deg)": bands[ch],
                         "share of band (%)": 100 * x} for ch, x in fr.items()],
               title=_STATED_BANDS_TITLE)
    return rows


def _sec_brackets(st, pd, v, sc, key):
    from suspension import genesis_analysis as ga
    worst = max(sc["rows"], key=lambda r: abs(r["force_N"]))
    st.caption("Double-shear clevis root bending. The load defaults to the "
               f"worst screened link force ({worst['member']}, "
               f"{worst['load_case']}); reach and plate size are the design "
               "variables — shortening the reach is usually worth more than "
               "thickening the plate.")
    F0 = float(abs(worst["force_N"]))
    example = pd.DataFrame(
        {"case": ["as drawn", "thicker plates", "shorter reach"],
         "load (N)": [F0] * 3, "reach (mm)": list(_BRACKET_EXAMPLE["reach (mm)"]),
         "plate width (mm)": list(_BRACKET_EXAMPLE["plate width (mm)"]),
         "plate thickness (mm)": list(_BRACKET_EXAMPLE["plate thickness (mm)"])})
    cases = st.data_editor(example, num_rows="dynamic", key=key, hide_index=True)
    declared = bracket_is_declared(cases)
    if not declared:
        st.caption("The three rows are an example bracket, not yours: row 1 "
                   "is the bracket as drawn, the others are alternatives. "
                   "Until you change the reach or plate size the table is "
                   "left out of the report and the review shows the bracket "
                   "check as not available.")
    rows = []
    for _, r in cases.iterrows():
        try:
            b = ga.bracket_fos(float(r["load (N)"]), float(r["reach (mm)"]),
                               float(r["plate width (mm)"]),
                               float(r["plate thickness (mm)"]),
                               int(v["bracket_plates"]), v["yield_mpa"],
                               v["bracket_kt"])
            rows.append({"case": r["case"], "load (N)": float(r["load (N)"]),
                         "reach (mm)": float(r["reach (mm)"]),
                         "plate (mm)": _cell(float(r["plate width (mm)"])) + " × "
                         + _cell(float(r["plate thickness (mm)"])),
                         "stress (MPa)": b["stress_mpa"],
                         "FoS": b["fos"],
                         "meets required FoS": b["fos"] >= v["fos_min"]})
        except (TypeError, ValueError, ZeroDivisionError):
            continue
    from suspension import report_capture as rc
    _ss = getattr(st, "session_state", {})
    if not declared:
        rc.forget(_ss, _BRACKET_TITLE)
    with rc.unreported(_ss, not declared):
        _table(st, pd, rows, title=_BRACKET_TITLE)
    first = cases.iloc[0] if len(cases) else None
    geom = None
    if first is not None:
        try:
            geom = {"reach_mm": float(first["reach (mm)"]), "plate_width_mm": float(first["plate width (mm)"]),
                    "plate_thickness_mm": float(first["plate thickness (mm)"])}
        except (TypeError, ValueError):
            geom = None
    return {"declared": declared, "fos_as_drawn": rows[0]["FoS"] if rows else None, "as_drawn": geom}


#: The values the two example blocks open with. Numbers a user has not replaced are not results of their design, so
#: the report leaves those blocks out until they differ from these.
_STATED_EXAMPLE = (4799.0, 485.0, 0.60, 0.116, 0.074)
_STATED_LINK_TITLE = "Stated link: how much of the reported extension stretch and lash explain"
_STATED_BANDS_TITLE = "Stated deflections against the tolerance bands"
_BRACKET_TITLE = "Bracket screening, double-shear clevis root bending"
_BRACKET_EXAMPLE = {"reach (mm)": (27.9, 27.9, 10.0), "plate width (mm)": (30.0, 40.0, 30.0),
                    "plate thickness (mm)": (5.0, 6.0, 5.0)}


def is_declared(values, example) -> bool:
    """True when any entered value differs from the example the field opens with."""
    return any(abs(float(a) - float(b)) > 1e-9 for a, b in zip(values, example))


def bracket_is_declared(cases) -> bool:
    """True when the bracket table is no longer the example: a row added or removed, or any reach or plate size
    changed. The load column follows the worst link force, so it does not count."""
    try:
        if len(cases) != 3:
            return True
        return any(is_declared(list(cases[col]), ex) for col, ex in _BRACKET_EXAMPLE.items())
    except (KeyError, TypeError, ValueError):
        return True


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
        ["quantity", "value"], title="Packaging: wheelbase, ground clearance, rim envelope")
    if abs(w["wheelbase_mm"] - v["wheelbase_mm"]) > 0.5:
        st.warning("The declared wheelbase differs from the axle stations.")
    return {"wheelbase": w, "clearance": cl, "envelope": e}


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
            ("tan swing arm, referenced to the " + d["side_view_reference"],
             d["side_view_tan"])]
    if "anti_dive_pct" in d:
        # Not tan × L/h × bias of the row above: that is the side-view-IC form, and this is the contact patch's own
        # path slope (virtual work), the form the search's anti-dive bound uses. Say so, or the arithmetic a reader
        # does on the table disagrees with the table.
        rows.append(("anti-dive, from the contact patch's path (virtual work) (%)", d["anti_dive_pct"]))
        if d.get("anti_dive_svic_pct") is not None:
            rows.append(("anti-dive, side-view IC tangent form, for comparison (%)", d["anti_dive_svic_pct"]))
    if "anti_squat_pct" in d:
        rows.append(("anti-squat (%)", d["anti_squat_pct"]))
    if "anti_lift_pct" in d:
        rows.append(("anti-lift, outboard rear brakes (%)",
                     d["anti_lift_pct"]))
    _table(st, pd, rows, ["quantity", "value"], title="Kinematics and anti-geometry, " + axle + " corner")
    _table(st, pd, [{"travel (mm)": t, "camber (deg)": a, "toe (deg)": b,
                     "RC height (mm)": r, "scrub (mm)": sc}
                    for t, a, b, r, sc in zip(
                        d["stations_mm"], d["camber_deg"], d["toe_deg"],
                        d["rc_height_mm"], d["scrub_mm"])],
           title="Camber, toe, roll-centre height and scrub over wheel travel")
    ctr = gr.camber_to_road(corner.static_camber,
                            d["camber_gain_deg_per_mm"], v["body_roll_deg"],
                            trk, v["tyre_optimum_camber"])
    st.markdown("**Loaded tyre camber relative to the road** "
                "(outside wheel at the declared body roll)")
    names = {"bump_mm": "outside-wheel bump from roll (mm)",
             "camber_change_deg": "camber change from gain (deg)",
             "camber_to_chassis_deg": "camber to chassis (deg)",
             "camber_to_road_deg": "camber to road (deg)",
             "gain_required_deg_per_mm": "gain needed to hold the optimum (deg/mm)",
             "ratio_to_delivered": "needed ÷ delivered gain (×)",
             "error_from_optimum_deg": "distance from the tyre optimum (deg)"}
    _table(st, pd, [(names.get(k, k), x) for k, x in ctr.items()],
           ["quantity", "value"], title="Loaded tyre camber relative to the road (outside wheel at the declared roll)")
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
    """Chassis STEP → tube axes → frame, read as soon as a file is dropped
    and stored as the Frame Planner frame for keep-outs and the swept-volume
    check."""
    from suspension import genesis_analysis as ga
    _hint(st, ss, "Export the chassis from your CAD as STEP (AP203/AP214, "
                  "solid bodies) and drop it here. Tubes, their sizes, wall "
                  "thicknesses and bends are read straight from the file; "
                  "nothing is meshed. Coordinates are taken as CAD axes: "
                  "X to the vehicle's right, Y up, Z forward.")
    up = st.file_uploader("Chassis STEP file",
                          type=["step", "stp", "STEP", "STP"],
                          key=f"{key}_up",
                          help="Solid-body STEP export. Multi-body weldments "
                               "and single fused bodies both work.")
    adv = st.toggle("Reading settings", key=f"{key}_adv",
                    help="Only needed if the automatic tube-size detection "
                         "picks the wrong cylinders.")
    radii_txt, rtol, atol, classic = "", 0.02, 0.5, False
    tols = [20.0, 30.0, 37.0, 40.0, 50.0]
    manual = []
    if adv:
        c = st.columns(4)
        radii_txt = c[0].text_input(
            "Tube outer radii (mm), blank = detect", "", key=f"{key}_radii",
            help="e.g. 12.7, 9.525 for 1 in and 3/4 in tube.")
        rtol = c[1].number_input("Radius tolerance (mm)", 0.001, 2.0, 0.02,
                                 0.001, format="%.3f", key=f"{key}_rtol")
        atol = c[2].number_input("Coaxial tolerance (mm)", 0.01, 10.0, 0.5,
                                 0.01, key=f"{key}_atol")
        tols = _floats(c[3].text_input("Node clustering tolerances (mm)",
                                       "20, 30, 37, 40, 50",
                                       key=f"{key}_tols"), tols)
        classic = st.checkbox(
            "Merge collinear segments into one tube (classic method)",
            value=False, key=f"{key}_classic",
            help="Off: two separate tubes on the same line (e.g. left and "
                 "right stubs) stay two tubes. On: one tube per unique axis "
                 "line, spanning both.")
        st.caption("Bends the file does not carry as toroidal faces can be "
                   "added by hand:")
        extra = st.data_editor(
            pd.DataFrame(columns=["bend major radius (mm)",
                                  "bend angle (deg)"]),
            num_rows="dynamic", key=f"{key}_bends", hide_index=True)
        manual = [(float(r.iloc[0]), float(r.iloc[1]))
                  for _, r in extra.iterrows()
                  if not (pd.isna(r.iloc[0]) or pd.isna(r.iloc[1]))]
    radii = _floats(radii_txt, ()) or None

    if up is not None:
        sig = (up.name, up.size, radii_txt, rtol, atol, tuple(tols),
               tuple(manual), classic)
        if ss.get("ig_step_sig") != sig:
            with st.spinner("Reading " + up.name + "…"):
                try:
                    raw = up.getvalue()
                    text = raw.decode("utf-8", errors="replace")
                    if "ISO-10303-21" not in text[:2000]:
                        raise ValueError("this does not look like a STEP "
                                         "file (no ISO-10303-21 header)")
                    res = ga.parse_step_tubes(
                        text, radii, rtol, atol,
                        split_gap_mm=None if classic else 1.0)
                    stats = ga.frame_stats(res.axes, list(res.bends) + manual,
                                           tols, res.vertices,
                                           od_mm=res.od_mm)
                    ss["ig_step_frame"] = (res, stats, up.name)
                    ss["ig_step_err"] = None
                    if res.axes:
                        g = ga.frame_graph_from_step(res, float(tols[0]))
                        ss["tf_frame"] = g.as_dict()
                except Exception as e:           # noqa: BLE001
                    ss["ig_step_frame"] = None
                    ss["ig_step_err"] = str(e)
            ss["ig_step_sig"] = sig

    if ss.get("ig_step_err"):
        st.error("Could not read the file: " + ss["ig_step_err"])
        return
    if not ss.get("ig_step_frame"):
        if ss.get("tf_frame"):
            st.caption("A frame is already loaded (from the Frame Planner).")
        return
    res, s, fname = ss["ig_step_frame"]
    if not res.axes:
        common = sorted(res.radii_mm.items(), key=lambda kv: -kv[1])[:6]
        st.error("No tubes found in " + fname + ". Cylinder radii in the "
                 "file (mm: faces): "
                 + (", ".join(_cell(r) + ": " + str(n) for r, n in common)
                    or "none")
                 + ". Open Reading settings and type the tube outer "
                 "radius, or check that the export contains solid bodies.")
        return
    sizes = ", ".join(str(n) + " × " + _cell(od) + " × "
                      + ("?" if w is None else _cell(w)) + " mm"
                      for (od, w), n in sorted(s["sizes_mm"].items(),
                                               key=lambda kv: -kv[1]))
    st.success(fname + ": " + str(s["tubes"]) + " tubes, "
               + _cell(s["total_m"]) + " m of tube ("
               + _cell(s["straight_m"]) + " m straight + "
               + _cell(s["bend_arc_m"]) + " m of bends). Sizes (OD × wall): "
               + sizes + ".")
    st.caption(res.units_note + " · tube radii used: "
               + ", ".join(_cell(r) for r in res.tube_radii_mm) + " mm")
    try:
        import plotly.graph_objects as go
        fig = go.Figure()
        for (a, b, w), od in zip(res.axes, res.od_mm):
            fig.add_trace(go.Scatter3d(
                x=[a[0], b[0]], y=[a[2], b[2]], z=[a[1], b[1]],
                mode="lines", line=dict(width=max(2.0, od / 4)),
                hovertext=_cell(od) + " × "
                + ("?" if w is None else _cell(w)) + " mm",
                hoverinfo="text", showlegend=False))
        fig.update_layout(height=380, margin=dict(l=0, r=0, t=0, b=0),
                          scene=dict(aspectmode="data",
                                     xaxis_title="X right (mm)",
                                     yaxis_title="Z forward (mm)",
                                     zaxis_title="Y up (mm)"))
        from suspension import report_capture as rc
        rc.heading(ss, "Frame tube centrelines read from " + str(fname))
        st.plotly_chart(fig, key=f"{key}_fig")
    except Exception:                            # noqa: BLE001
        pass
    _table(st, pd, [{"node clustering (mm)": k, "nodes": x["nodes"],
                     "node-to-node length (m)": x["node_to_node_m"]}
                    for k, x in s["nodes"].items()])
    if res.bends:
        _table(st, pd, [{"bend": i + 1, "major radius (mm)": r,
                         "swept angle (deg)": a}
                        for i, (r, a) in enumerate(res.bends)])
    if "vertices" in s:
        vx = s["vertices"]
        st.caption("File check: " + str(vx["total"]) + " vertices, "
                   + str(vx["in_tube_envelope"]) + " on tubes, "
                   + str(vx["outside_envelopes"]) + " elsewhere (plates, "
                   "holes, bend tangents).")

def _render_analysis(st, pd, np, ss):
    """Section 4 + 5 of the tab: the shared declaration and every analysis of
    the current corner (generated, else the seed)."""
    from suspension import genesis_repro as gr
    from suspension.kinematics import Hardpoints
    st.divider()
    st.markdown("###### 5 · Vehicle & build declaration")
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

    st.markdown("###### 6 · Design review and analysis")
    _hint(st, ss, "Everything the solver was NOT asked about: steering "
                  "geometry, strength, packaging, ride. The review "
                  "summarises it against editable starting ranges; the "
                  "sections below show the working.")
    st.caption(f"For {note}, {axle} axle, with the declaration above.")
    vm = ss.get("ig_veh_from_manifest")
    if run is not None and vm and run.get("source") == "manifest":
        applied = ", ".join(f"{k} = {_cell(x)}" for k, x in vm["fields"].items())
        rng = "; ".join(f"{c} {'' if lo is None else _cell(lo)} to {'' if hi is None else _cell(hi)}".replace("  ", " ")
                        for c, (lo, hi) in (vm.get("ranges") or {}).items())
        # One line each: the report keeps a banner only up to 400 characters, so one long one was dropped whole.
        st.success(f"ℹ️ Section 5 set from the car `{vm['name']}` declares, so this review reads the car the search "
                   f"was bounded with: {applied}.")
        if rng:
            st.success(f"ℹ️ Review ranges taken from the manifest's own bounds: {rng}.")
        for item in vm["not_held"]:
            st.warning(f"Declared in `{vm['name']}` but not held by section 5: {item}.")
    if run is None or run["result"].winner_hp is None:
        st.warning("No generated geometry yet: every number in this review "
                   "and the sections under it describes the seed geometry "
                   "the search would start from, not a result of it.")
    front_hp, rear_hp = _corner_pair(ss, corner, axle)
    st.warning(_SCOPE)
    _inputs_table(st, pd, ss, v, run, corner, axle)
    review_box = st.container()

    out = {}
    with st.expander("🔎 Kinematics, anti-geometry and loaded camber"):
        out["kin"] = _sec_kinematics(st, pd, v, corner, axle, travel)
    with st.expander("📐 Actuation, ride frequency and roll stiffness"):
        out["act"] = _sec_actuation(st, pd, v, corner, axle)
    with st.expander("🛞 Steering effort"):
        if axle != "front":
            st.caption("Uses the front corner — shown with the front corner "
                       "from an earlier run, or the default.")
        out["steer"] = _sec_steering(st, pd, v, front_hp, rear_hp,
                                     "ig_steer", rack_ratio=declared_rack_ratio(ss, run, axle))
    with st.expander("🔩 Structural screening of the links"):
        out["sc"] = _sec_structural(st, pd, v, corner, axle)
    with st.expander("⚙️ Compliance budget — link strain, lash and band use"):
        out["comp"] = _sec_compliance(st, pd, v, corner, axle, out["sc"],
                                      bands)
    with st.expander("🪝 Bracket screening"):
        out["brk"] = _sec_brackets(st, pd, v, out["sc"], "ig_brk")
    with st.expander("📦 Packaging — wheelbase, ground clearance, rim "
                     "envelope"):
        out["pack"] = _sec_packaging(st, pd, v, corner)
    with st.expander("🧱 Where the link loads go: brackets, upright, hub bearings, fatigue"):
        out["parts"] = _sec_parts(st, pd, ss, v, corner, axle, out["sc"], out["brk"])

    with review_box:
        _render_review(st, pd, ss, v, out, axle, note, run)


def _review_values(v, out, axle):
    """Collect the numbers the design review checks from the sections."""
    vals = {}
    d = out.get("kin") or {}
    if d.get("ok"):
        vals.update(caster_deg=d["caster_deg"], kpi_deg=d["kpi_deg"],
                    scrub_mm=d["scrub_static_mm"],
                    camber_gain_deg_per_mm=d["camber_gain_deg_per_mm"],
                    bump_steer_abs=abs(d["bump_steer_deg_per_mm"]),
                    toe_change_deg=d["toe_change_deg"],
                    rc_height_mm=d["rc_height_static_mm"],
                    rc_migration_abs=abs(d["rc_migration_chassis_mm_per_mm"]))
        anti = d.get("anti_dive_pct", d.get("anti_squat_pct"))
        if anti is not None:
            vals["anti_pct"] = anti
    pk = out.get("pack")
    if pk:
        vals["joints_in_rim"] = 1.0 if pk["envelope"]["inside"] else 0.0
        vals["wheelbase_margin_mm"] = pk["wheelbase"]["margin_mm"]
        vals["ground_clearance_mm"] = pk["clearance"]["clearance_mm"]
    sc = out.get("sc")
    if sc:
        vals["worst_fos"] = sc["worst_fos_overall"]
    stq = out.get("steer")
    if stq:
        vals["steering_ratio_needed"] = stq["ratio_needed_for_target"]
    a = out.get("act") or {}
    if a.get("wheel_rate_spread_pct") is not None:
        vals["wheel_rate_spread"] = a["wheel_rate_spread_pct"]
    if a.get("rate_character"):
        vals["rate_not_falling"] = 0.0 if a["rate_character"] == "falling" else 1.0
    brk = out.get("brk") or {}
    if brk.get("declared") and brk.get("fos_as_drawn") is not None:
        vals["bracket_fos"] = brk["fos_as_drawn"]
    vals["mr_solved"] = 1.0 if a.get("ok") else 0.0
    if a.get("stroke_fraction_used") is not None:
        vals["stroke_used"] = a["stroke_fraction_used"]
    shares = [r["share of toe band (%)"] / 100 for r in (out.get("comp") or [])]
    if shares:
        vals["toe_band_share"] = max(shares)
    return vals


_STATUS_ICON = {"pass": "🟢", "watch": "🟡", "fail": "🔴", "n/a": "⚪"}
#: The status in words beside the icon: the PDF's font has no colour emoji, and a review whose status column printed
#: empty told a reader nothing.
_STATUS_WORD = {"pass": "PASS", "watch": "WATCH", "fail": "FAIL", "n/a": "N/A"}


#: Review checks whose value is a yes or a no.
_YES_NO_CHECKS = ("joints_in_rim", "mr_solved", "rate_not_falling")


def _range_text(r):
    lo, hi = r["lo"], r["hi"]
    if r["key"] in _YES_NO_CHECKS:
        return "must be yes"
    if hi >= 1e8:
        return "≥ " + _cell(lo)
    if lo == 0 and r["key"].endswith(("_abs", "used", "share", "change_deg",
                                     "needed", "spread")):
        return "≤ " + _cell(hi)
    return _cell(lo) + " to " + _cell(hi)


def declared_rack_ratio(ss, run, axle):
    """The steering ratio of the rack a manifest declared: the steering-wheel angle the search solved for the
    declared lock, over that road-wheel lock (both deg, so dimensionless). None without a manifest front run that
    bounded the lock."""
    vm = ss.get("ig_veh_from_manifest") or {}
    if run is None or run.get("source") != "manifest" or axle != "front" or not vm.get("lock_road_wheel_deg"):
        return None
    w = getattr(run["result"], "winner", None)
    sw = (getattr(w, "properties", None) or {}).get("lock_sw_deg") if w is not None else None
    return None if not sw else float(sw) / float(vm["lock_road_wheel_deg"])


#: What every number in the review is, and is not. A judge asks; the report says.
_SCOPE = ("Scope: one corner, solved as rigid links on spherical joints, at ride height and over the declared wheel "
          "travel. Not in it: the whole car in roll or pitch (no roll-centre lateral migration), a tyre model, "
          "dynamics, and bracket, upright or chassis flexibility unless a table says so. Check the result in "
          "full-vehicle simulation and on the car.")


def input_sources(ss, v, run, corner, axle):
    """Every section-5 input the analysis of this corner reads, with where its value came from: 'manifest' (the
    loaded manifest declared it), 'entered' (it differs from the tool's default) or 'default' (nobody declared it
    for this car). Rows are {input, value, source}; values in the units of their labels."""
    vm = ss.get("ig_veh_from_manifest") if run is not None and run.get("source") == "manifest" else None
    from_man = set((vm or {}).get("fields") or {})
    other = "rear" if axle == "front" else "front"
    linkage = corner.has_rocker() or corner.has_direct_damper()
    rows = []
    for group, fields in _VEH_FIELDS:
        if group == "Powertrain & aero" or (linkage and group.startswith("Motion ratio when")):
            continue
        for f, label, _lo, _hi, _step, default in fields:
            if f.endswith(f"_{other}_lbin") or f.endswith(f"_{other}_kg") or f.endswith(f"_{other}") \
                    or f.startswith(f"mr_{other}_"):
                continue
            src = ("manifest" if f in from_man else
                   "default" if abs(float(v[f]) - float(default)) < 1e-12 else "entered")
            rows.append({"input": label, "value": v[f], "source": src})
    for m, (od, w) in sorted((v.get("link_tubes") or {}).items()):
        rows.append({"input": f"Link tube {m}, od × wall (mm)", "value": _cell(od) + " × " + _cell(w),
                     "source": "manifest"})
    return rows


def _inputs_table(st, pd, ss, v, run, corner, axle):
    """The first table of the review: its inputs and their sources, and how many are still defaults."""
    rows = input_sources(ss, v, run, corner, axle)
    with st.expander("📥 Inputs to this review and where each came from"):
        _table(st, pd, rows, title="Inputs to this review, and where each came from")
    n = sum(r["source"] == "default" for r in rows)
    if n:
        st.warning(f"{n} of the {len(rows)} inputs to this review are the tool's defaults, not values declared for "
                   "this car (marked 'default' in the inputs table). The ride, roll-stiffness, loaded-camber and "
                   "steering-effort numbers below are only as good as those: enter the car's own in section 5.")


#: Section-5 inputs each review check reads, by the short name the review prints. Checks not listed read only the
#: geometry, or inputs a manifest always declares.
_CHECK_INPUTS = {
    "steering_ratio_needed": {"aligning_outer_Nm": "aligning torque", "aligning_inner_Nm": "aligning torque",
                              "lateral_mu": "lateral µ", "lateral_g": "design g", "weight_dist_front": "weight split",
                              "tire_radius_mm": "tyre radius"},
    "worst_fos": {"lateral_g": "design g", "roll_share_front": "roll share", "weight_dist_front": "weight split",
                  "aligning_outer_Nm": "aligning torque"},
    "toe_band_share": {"lateral_g": "design g", "roll_share_front": "roll share", "weight_dist_front": "weight split",
                       "aligning_outer_Nm": "aligning torque"},
    "stroke_used": {"damper_stroke_mm": "damper stroke"},
    "joints_in_rim": {"rim_offset_mm": "ball-joint offset limit"},
    "bracket_fos": {"bracket_kt": "bracket Kt", "bracket_plates": "bracket plates"},
    "wheelbase_margin_mm": {"front_station_mm": "axle stations", "rear_station_mm": "axle stations"},
    "ground_clearance_mm": {"ground_y_mm": "ground plane", "lowest_member_y_mm": "lowest frame member"},
}


def _review_corner(ss, run):
    """The corner the review is about (the run's winner, else the seed), for input_sources."""
    from suspension import genesis_repro as gr
    from suspension.kinematics import Hardpoints
    if run is not None and run["result"].winner_hp is not None:
        return run["result"].winner_hp
    seed = ss.get("ig_seed_hp")
    return gr.hp_from_dict(seed) if seed else Hardpoints.default()


def defaults_behind(rows, sources):
    """{check key: 'design g, roll share, …'}: the tool's default inputs each review check rests on, by short name,
    from the inputs table's sources. Empty for a check that reads none."""
    by_label = {r["input"]: r["source"] for r in sources}
    label_of = {f: label for _, fields in _VEH_FIELDS for f, label, *_ in fields}
    out = {}
    for r in rows:
        names = []
        for f, short in _CHECK_INPUTS.get(r["key"], {}).items():
            if by_label.get(label_of.get(f)) == "default" and short not in names:
                names.append(short)
        if names:
            out[r["key"]] = ", ".join(names)
    return out


def as_built_unpriced(res, frac=0.05):
    """Bounded properties the winner meets with less than ``frac`` of the bound's own scale to spare: [(label,
    margin)]. The build yield prices the curve channels only; these are met by the nominal geometry and not checked
    across builds, so the tolerances can take them out of their bounds."""
    import math
    w = getattr(res, "winner", None)
    pb = getattr(res, "property_bounds", None)
    if w is None or pb is None or not w.properties:
        return []
    from suspension import inverse_genesis as ig
    out = []
    for b in pb.bounds:
        val = w.properties.get(b.prop)
        if val is None or not math.isfinite(float(val)):
            continue
        m = float(b.margin(val))
        ends = [abs(float(x)) for x in (b.lo, b.hi) if x is not None and math.isfinite(float(x))]
        scale = max(ends + [abs(float(val)), 1e-9])
        if 0.0 <= m < frac * scale:
            out.append((ig._PROPERTY_LABELS[b.prop], m))
    return out


def self_graded(rows, ranges, generic):
    """Review rows judged against a range the manifest declared, whose value is outside the tool's generic starting
    range for the same check: [(row, generic lo, generic hi)]. Passing a bound you set yourself is not evidence the
    number is a good one, so the report says the generic range disagrees and asks for the reason."""
    out = []
    by_check = {g["check"]: g for g in generic}
    for r in rows:
        g = by_check.get(r["check"])
        if r["check"] in ranges and g is not None and r.get("value") is not None \
                and not g["lo"] <= float(r["value"]) <= g["hi"]:
            out.append((r, g["lo"], g["hi"]))
    return out


def _render_review(st, pd, ss, v, out, axle, note, run):
    """Traffic-light design review with editable ranges and a report."""
    from suspension import genesis_analysis as ga
    limits = ss.get("ig_review_limits")
    if limits is None:
        limits = [dict(r) for r in ga.DEFAULT_REVIEW_LIMITS]
        for r in limits:
            if r["key"] in ("worst_fos", "bracket_fos"):
                r["lo"] = float(v["fos_min"])
        ss["ig_review_limits"] = limits
    rows = ga.design_review(_review_values(v, out, axle), limits)
    counts = {k: sum(r["status"] == k for r in rows)
              for k in ("pass", "watch", "fail", "n/a")}
    ss["ig_ok_review"] = counts["fail"] == 0
    with st.container(border=True):
        head = ("**📋 Design review** — 🟢 " + str(counts["pass"])
                + " pass · 🟡 " + str(counts["watch"]) + " watch · 🔴 "
                + str(counts["fail"]) + " fail")
        if counts["n/a"]:
            head += " · ⚪ " + str(counts["n/a"]) + " not available"
        st.markdown(head)
        order = {"fail": 0, "watch": 1, "pass": 2, "n/a": 3}
        show = sorted(rows, key=lambda r: order[r["status"]])
        defaults = defaults_behind(rows, input_sources(ss, v, run, _review_corner(ss, run), axle))
        _table(st, pd, [{"status": _STATUS_ICON[r["status"]] + " " + _STATUS_WORD[r["status"]], "check": r["check"],
                         "value": ("yes" if r["value"] == 1.0 else "no")
                         if r["key"] in _YES_NO_CHECKS
                         and r["value"] is not None else r["value"],
                         "unit": r["unit"], "range": _range_text(r),
                         "rests on defaults": defaults.get(r["key"], ""),
                         "why it matters": r["why"],
                         "what to change": r["fix"]
                         if r["status"] in ("fail", "watch") else ""}
                        for r in show],
               title="Design review: FAIL outside its range, WATCH inside but within 10 % of a limit, "
                     "PASS inside, N/A not available")
        vm_ranges = ((ss.get("ig_veh_from_manifest") or {}).get("ranges") or {}) \
            if run is not None and run.get("source") == "manifest" else {}
        on_defaults = [r["check"] for r in rows if defaults.get(r["key"]) and r["status"] in ("pass", "watch", "fail")]
        if on_defaults:
            st.warning("These review results rest on the tool's defaults, not this car's values (the 'rests on "
                       "defaults' column names which): " + "; ".join(on_defaults)
                       + ". Their colour is only as good as those inputs.")
        for r, glo, ghi in self_graded(rows, vm_ranges, ga.DEFAULT_REVIEW_LIMITS):
            st.warning(f"{r['check']}: {_cell(r['value'])} {r['unit']} is inside the range the manifest declared "
                       f"({_range_text(r)}) but outside the generic starting range "
                       f"({_range_text({'key': r['key'], 'lo': glo, 'hi': ghi})}). The manifest does not say why "
                       "this car wants it: give the reason in 'Why did you run this?'.")
        if _guided(ss) and counts["fail"]:
            from_manifest = set(vm_ranges)
            red_generic = [r["check"] for r in rows if r["status"] == "fail" and r["check"] not in from_manifest]
            if red_generic:
                st.warning("Red is a prompt, not a verdict: "
                           + ("these ranges are" if from_manifest else "the ranges are")
                           + " generic starting points" + (f" ({', '.join(red_generic)})" if from_manifest else "")
                           + ". Either change the design (see 'what to change') or set the range to your team's "
                             "target and write down why.")
        with st.expander("Edit review ranges (your team's targets)"):
            ed = st.data_editor(
                pd.DataFrame([{"check": r["check"], "unit": r["unit"],
                               "min": r["lo"], "max": r["hi"]}
                              for r in limits]),
                key="ig_review_editor", hide_index=True,
                disabled=["check", "unit"])
            c1, c2 = st.columns(2)
            if c1.button("Apply ranges", key="ig_review_apply"):
                for r, (_, e) in zip(limits, ed.iterrows()):
                    r["lo"], r["hi"] = float(e["min"]), float(e["max"])
                ss["ig_review_limits"] = limits
                st.rerun()
            if c2.button("Reset to starting ranges", key="ig_review_reset"):
                ss.pop("ig_review_limits", None)
                st.rerun()
        st.download_button("⬇ Design report (.md)",
                           _report_md(v, out, rows, axle, note, run),
                           file_name="design_report_" + axle + ".md",
                           mime="text/markdown", key="ig_report_dl")
    _judge_summary_panel(st, ss, v, out, rows, axle, run, defaults)


def judge_summary_inputs(ss, v, out, rows, axle, run, defaults):
    """Everything judge_summary.build needs, from what the tab already computed. Returns (kwargs, figure png or
    None). Pure apart from reading ``ss``; no Streamlit calls."""
    import numpy as np
    from suspension import genesis_repro as gr
    from suspension import judge_summary as js

    d = out.get("kin") or {}
    res = run["result"] if run is not None else None
    man = gr.GenesisManifest.from_json(run["manifest_json"]) if run is not None else None
    tg = man.objects()[1] if man is not None else None
    w = getattr(res, "winner", None)
    stations = list(d.get("stations_mm") or [])
    i0 = int(np.argmin(np.abs(np.asarray(stations)))) if stations else None

    targets, bands = [], {}
    for c in (tg.curves if tg is not None else []):
        tr, tv, bd = (np.asarray(x, float) for x in (c.travel_mm, c.target, c.band))
        j = int(np.argmin(np.abs(tr)))
        series = d.get(c.channel) if c.channel in ("camber_deg", "toe_deg", "rc_height_mm", "scrub_mm") else None
        targets.append({"channel": c.channel, "target_at": float(tv[j]), "band": float(bd[j]),
                        "delivered": None if series is None or i0 is None else float(series[i0])})
        bands[c.channel] = (tr, tv, bd)
    props = []
    if res is not None and getattr(res, "property_bounds", None) is not None and w is not None and w.properties:
        from suspension import inverse_genesis as ig
        for b in res.property_bounds.bounds:
            props.append({"label": ig._PROPERTY_LABELS[b.prop], "lo": b.lo, "hi": b.hi,
                          "value": w.properties.get(b.prop)})
    specs = js.specification_rows(targets, d, props)

    def g(key, digits=4):
        return js._num(d.get(key), digits)
    act = out.get("act") or {}
    steer = out.get("steer") or {}
    sc = out.get("sc") or {}
    decisions = {
        "camber": {"target": next((f"{js._num(t['target_at'])}° ± {js._num(t['band'])}° at ride height, curve over "
                                   f"±{js._num(max(abs(x) for x in stations) if stations else 0)} mm"
                                   for t in targets if t["channel"] == "camber_deg"), ""),
                   "result": f"gain {g('camber_gain_deg_per_mm')} deg/mm; static {js._num(d['camber_deg'][i0]) if i0 is not None else '—'}°",
                   "evidence": "camber curve in the figure; loaded camber to road in the appendix"},
        "roll_centre": {"target": next((f"{js._num(t['target_at'])} ± {js._num(t['band'])} mm"
                                        for t in targets if t["channel"] == "rc_height_mm"), ""),
                        "result": f"{g('rc_height_static_mm')} mm at ride height; migration "
                                  f"{g('rc_migration_chassis_mm_per_mm')} mm/mm in the chassis frame",
                        "evidence": "roll-centre curve in the figure"},
        "toe": {"target": next((f"{js._num(t['target_at'])}° ± {js._num(t['band'])}°"
                                for t in targets if t["channel"] == "toe_deg"), ""),
                "result": f"bump steer {g('bump_steer_deg_per_mm')} deg/mm; {g('toe_change_deg')}° over the travel",
                "evidence": "toe curve in the figure"},
        "anti_dive": {"result": f"{g('anti_dive_pct')} % at brake bias {js._num(v['brake_bias_front'])} "
                                f"(contact-patch path, virtual work)"},
        "steering": {"result": f"caster {g('caster_deg')}°, kingpin inclination {g('kpi_deg')}°, scrub "
                               f"{g('scrub_static_mm')} mm, mechanical trail {js._num(steer.get('trail_mm'))} mm"},
        "ride": {"result": (f"motion ratio {js._num(act.get('mr_static'))} at ride height "
                            f"({js._num(act.get('mr_droop'))} droop, {js._num(act.get('mr_bump'))} bump, "
                            f"{act.get('rate_character', '—')}); ride frequency {js._num(act.get('ride_hz_static'), 3)} Hz "
                            "from wheel rate only") if act.get("ok") else ""},
        "loads": {"result": (f"weakest link {sc.get('governing_member', '—')}, factor of safety "
                             f"{js._num(sc.get('worst_fos_overall'), 3)}") if sc else "",
                  },
        "safety": {},
    }
    for k, r, name in (("anti_dive", "anti_pct", "anti-dive"), ("steering", "caster_deg", "caster")):
        row = next((x for x in rows if x["key"] == r), None)
        if row is not None:
            decisions[k]["target"] = f"{name} {_range_text(row)} {row['unit']}".strip()

    # The manifest's own structural cases and pickup flexibility are what its bounds (section 1) were computed
    # with; the review tables (section 4) use the generic cases and link stiffness only. Say so where both appear.
    mprops = ((man.volume or {}).get("properties") or {}) if man is not None and isinstance(man.volume, dict) else {}
    mcases = list((mprops.get("structure") or {}).get("cases") or {})
    generic = list(dict.fromkeys(r["load_case"] for r in (sc.get("rows") or [])))
    if sc:
        decisions["loads"]["evidence"] = (
            (f"section 1's link factor comes from the design's own {len(mcases)} load cases ({', '.join(mcases)}); "
             if mcases else "")
            + f"section 4 screens every link under {len(generic)} generic load cases ({', '.join(generic)}); every "
              "link force and factor of safety is in the appendix")
    if mcases or mprops.get("elasto"):
        decisions["safety"]["evidence"] = (
            "the compliance steer in section 1 comes from the design's own load cases and pickup flexibility; the "
            "appendix's generic compliance table uses link stiffness only, so its kerb-strike value is larger")

    vm = ss.get("ig_veh_from_manifest") or {}
    srcs = input_sources(ss, v, run, _review_corner(ss, run), axle)
    n_def = sum(r["source"] == "default" for r in srcs)
    tools = ["Kinematics and link loads from KinematiK's rigid-link corner solver. Its published benchmarks "
             "(BENCHMARK.md): kinematics within 0.00003 deg and 0.00003 mm of Adams View 2026.1; link forces within "
             "0.001 N of Adams View, MATLAB Simscape Multibody and Project Chrono 9.",
             "Geometry found by InverseGenesis from the stated targets and pickup zones, and priced for build yield "
             "against the declared shop tolerances; the run is reproducible from its manifest"
             + (f" `{man.name}` (inputs sha256 {man.inputs_sha256[:12]})" if man is not None else "") + "."]
    evidence = [f"{len(srcs) - n_def} of {len(srcs)} analysis inputs are declared for this car; {n_def} are tool "
                "defaults, each named in the appendix's inputs table."]
    loads = [(m, f, sc.get("governing_case", {}).get(m, "")) for m, f in (sc.get("worst_fos_per_member") or {}).items()]
    safety = []
    if sc:
        safety.append(f"Every link at or above factor of safety {js._num(v['fos_min'])} in every load case: weakest "
                      f"{sc.get('governing_member')} at {js._num(sc.get('worst_fos_overall'), 3)} (tension yield and "
                      "pinned-pinned Euler buckling).")
    env = getattr(res, "wheel_envelope", None) if res is not None else None
    if w is not None and env is not None and w.envelope_margin_mm is not None:
        gap = float(getattr(env, "clearance_mm", 0.0) or 0.0)
        safety.append(f"Nearest link or outboard pickup {js._num(w.envelope_margin_mm + gap, 3)} mm from the rim "
                      f"and tyre over the travel and both steering locks ({js._num(gap)} mm required).")
    def _with_unit(lab, m):
        unit = lab.rsplit(" (", 1)[1].rstrip(")") if lab.endswith(")") and " (" in lab else ""
        return lab.rsplit(" (", 1)[0], (js._num(m, 3) + ("" if unit in ("", "-") else " " + unit))
    thin = [_with_unit(lab, m) for lab, m in (as_built_unpriced(res) if res is not None else [])]
    if res is not None and w is not None and w.yield_frac is not None:
        safety.append(f"Build yield {js._num(100 * w.yield_frac, 3)} % at the declared shop tolerances (camber, toe "
                      "and roll-centre curves); measure the built parts before quoting it.")
    vm_ranges = vm.get("ranges") or {} if run is not None and run.get("source") == "manifest" else {}
    from suspension import genesis_analysis as ga
    sg = [(r["check"], f"{js._num(r['value'])} {r['unit']}".strip(),
           _range_text({"key": r["key"], "lo": lo, "hi": hi}))
          for r, lo, hi in self_graded(rows, vm_ranges, ga.DEFAULT_REVIEW_LIMITS)]
    review = [dict(r, range=_range_text(r),
                   value=("yes" if r["value"] == 1.0 else "no") if r["key"] in _YES_NO_CHECKS and r["value"] is not None
                   else r["value"]) for r in rows]
    questions = js.open_questions(review, thin, sg, n_def, len(srcs))

    parts = out.get("parts")
    parts_rows = []
    if parts:
        pin = parts["inputs"]
        fmin = float(v["fos_min"])
        for a in parts["arms"]:
            parts_rows.append((f"Upright arm to the {a['joint']}", "bending at the root",
                               a["fos"], f"needs Z ≥ {js._num(a['Z_required_mm3'], 3)} mm³ for factor of safety "
                                         f"{js._num(fmin)}", a["load_case"]))
        b = max(parts["bearings"], key=lambda x: x["P0_N"])
        parts_rows.append(("Hub bearing, more loaded", "static, ISO 76", b["s0"],
                           f"needs C0 ≥ {js._num(b['C0_required_kN'], 3)} kN for s0 2 at "
                           f"{js._num(pin['ig_parts_brg_spacing'])} mm spacing", b["load_case"]))
        bw = min(parts["brackets"], key=lambda x: x["fos"])
        parts_rows.append((f"Pickup bracket, {bw['member']}" + ("" if parts["bracket_declared"] else
                                                                " (tool's example bracket)"),
                           "clevis root bending", bw["fos"], "", bw["load_case"]))
        ww = min(parts["weld"], key=lambda x: x["fos"])
        life = f"{int(round(pin['ig_parts_cycles'])):,} cycles"
        parts_rows.append((f"Link weld, {ww['member']}", f"fatigue, IIW FAT {js._num(pin['ig_parts_fat'])}",
                           ww["fos"], life, "service cases"))
        rw = min(parts["rod"], key=lambda x: x["fos"])
        parts_rows.append((f"Rod-end shank, {rw['member']}", "fatigue, VDI 2230", rw["fos"],
                           f"{life}; {js._num(pin['ig_parts_re_dia'])} mm, {js._num(pin['ig_parts_re_tpi'])} tpi",
                           "service cases"))
        for name, check, f, need, case in parts_rows:
            fatigue = check.startswith("fatigue")
            need_f = 1.0 if fatigue else fmin
            if f is None:
                questions.append((f"{name}: not declared",
                                  f"{need}, in {case}; a judge will ask what the part has."))
            elif f < need_f:
                questions.append((f"{name}: factor {js._num(f, 3)}",
                                  (f"below 1: at these loads it does not reach the {need.split(';')[0]} design life "
                                   f"({check})." if fatigue else
                                   f"below the required {js._num(fmin)} ({check}, {case}).")))
        if parts["residual_N"] > 1.0:
            questions.append(("Upright joint loads do not balance the tyre load",
                              f"residual {js._num(parts['residual_N'], 3)} N."))

    fig = None
    if stations and d:
        series = {k: d[k] for k in ("camber_deg", "toe_deg", "rc_height_mm") if k in d}
        try:
            fig = js.curves_png(stations, series, bands)
        except Exception:                     # noqa: BLE001  (a figure must never cost the summary)
            fig = None
    rationale = {k: {f: ss.get(f"ig_rat_{k}_{f}", "") or "" for f in ("why", "alternatives", "trade_off")}
                 for k, *_ in js.DECISIONS}
    answers = {q: ss.get(js.answer_key(q), "") or "" for q, _ in questions}
    chain = None
    try:
        static = float(d["camber_deg"][i0]) if i0 is not None else float(ss.get("ig_gamma0", -2.0))
        chain = js.target_chain(
            track_mm=float(v[f"track_{axle}_mm"]), body_roll_deg=float(v["body_roll_deg"]),
            tyre_optimum_camber_deg=float(v["tyre_optimum_camber"]), static_camber_deg=static,
            camber_gain_deg_per_mm=float(d.get("camber_gain_deg_per_mm")), lateral_mu=float(v["lateral_mu"]),
            mass_kg=float(v["mass_kg"]), cg_height_mm=float(v["cg_height_mm"]), wheelbase_mm=float(v["wheelbase_mm"]),
            sprung_corner_kg=float(v[f"sprung_{axle}_kg"]), ride_hz=(act.get("ride_hz_static") if act.get("ok") else None),
            anti_pct=d.get("anti_dive_pct" if axle == "front" else "anti_squat_pct"), axle=axle,
            sources={f: ("default" if abs(float(v[f]) - float(dflt)) < 1e-12 else "declared")
                     for _g, fields in _VEH_FIELDS for f, _l, _lo, _hi, _st, dflt in fields
                     if f in ("lateral_mu", "tyre_optimum_camber", "body_roll_deg", "mass_kg", "cg_height_mm",
                              "wheelbase_mm")})
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        chain = None
    from suspension import rationale as _rat
    iterations = [(r.get("declaration", "run"), f"worst station {r.get('worst station (× band)')}× band, binding "
                   f"{r.get('binding row') or '—'}", str(r.get("verdict", "")))
                  for r in (ss.get("ig_log") or [])]
    iterations += [("input edit", f"{c['label']}: {c['from']} → {c['to']}", "")
                   for c in _rat.changes_for(ss, "genesis")]
    return dict(title="Suspension design summary", team=str(ss.get("ig_js_team", "") or ""),
                corner_name=man.name if man is not None else "seed geometry", axle=axle, specs=specs,
                decisions=decisions, rationale=rationale, evidence=evidence, tools=tools, loads=loads,
                safety=safety, questions=questions, answers=answers, not_checked=not_covered(rows, parts),
                parts=parts_rows,
                figure_ref="judge_curves" if fig else None,
                appendix_name="the InverseGenesis feature report",
                target_chain=chain, iterations=iterations, weighting=str(ss.get("ig_js_weighting", "") or "")), fig


def not_covered(rows, parts=None):
    """What the pre-validation of one corner does not cover, for the judge summary: the solver's scope, every review
    check that could not be evaluated, and, without the parts screen, the parts whose loads are produced but not
    checked. Text only, unitless."""
    out = ["The whole car in roll or pitch (so no roll-centre lateral migration), a tyre model and vehicle dynamics: "
           "check in full-vehicle simulation and on the car.",
           "Bracket, upright and chassis flexibility: the appendix's compliance table uses link stretch and rod-end "
           "lash only; a compliance bound in section 1 adds pickup flexibility only where the design declared it.",
           ]
    if parts:
        out += ["Upright, bracket and frame-node stresses beyond the hand calculations in section 4: the upright "
                "joint loads per case (appendix) are the FEA load set for the upright and the frame nodes.",
                "Fatigue beyond the infinite-life screen: the service cases stand in for a duty cycle; logged loads "
                "from the car set the real one."]
    else:
        out += ["Upright, hub, bracket and chassis-node strength: the link forces in section 4 are the loads to "
                "check them with.",
                "Fatigue, including rod-end shank bending: the factors of safety are static, at the peak of each "
                "load case."]
    out += [f"{r['check']}: not evaluated. {r['why']}" for r in rows if r["status"] == "n/a"
            and not (parts and r["key"] == "bracket_fos")]
    return out


def _judge_summary_panel(st, ss, v, out, rows, axle, run, defaults):
    """Where the team writes its reasons, and the summary a judge reads: a separate document, not part of the
    feature report, so nothing here is captured into it."""
    from suspension import judge_summary as js
    from suspension import report_capture as rc
    if run is None:
        return
    with rc.unreported(ss), st.expander("🧑‍⚖️ Judge summary: your design, as the Design Event scores it"):
        st.caption("Two pages for the Design Event, structured to the FS score sheet (area B: specifications, use of "
                   "design tools & analysis, load cases and load path, safety). The tool fills the facts and data; "
                   "the reasons are yours. Anything you leave blank prints as 'Team to state', so a judge never "
                   "reads a reason you did not give.")
        up = st.file_uploader("Load saved reasons (.json)", type=["json"], key="ig_js_load")
        if up is not None:
            import hashlib
            import json
            raw = up.getvalue()
            tag = hashlib.sha256(raw).hexdigest()
            if ss.get("_ig_js_loaded") != tag:        # apply once, so later edits are not overwritten
                try:
                    data = json.loads(raw.decode("utf-8"))
                    ss.update(js.reasons_payload(data if isinstance(data, dict) else {}))
                    ss["_ig_js_loaded"] = tag
                except (ValueError, UnicodeDecodeError) as e:
                    st.warning(f"Could not read the saved reasons: {e}")
        st.text_input("Team name (printed in the header)", key="ig_js_team")
        for key, label, question in js.DECISIONS:
            st.markdown(f"**{label}**")
            st.caption(question)
            c = st.columns(3)
            c[0].text_area("Why", key=f"ig_rat_{key}_why", height=90)
            c[1].text_area("Alternatives considered", key=f"ig_rat_{key}_alternatives", height=90)
            c[2].text_area("Trade-off accepted", key=f"ig_rat_{key}_trade_off", height=90)
        st.markdown("**How the trade-offs were weighted**")
        st.caption("Which requirement won where two pulled against each other, and why: lap time, points per event, "
                   "driver feedback. Judges ask this after the targets.")
        st.text_area("Weighting", key="ig_js_weighting", height=90)
        kw, fig = judge_summary_inputs(ss, v, out, rows, axle, run, defaults)
        if kw["questions"]:
            st.markdown("**Questions a judge will ask**")
            for q, why in kw["questions"]:
                st.text_area(q, key=js.answer_key(q), height=70, help=why)
            kw, fig = judge_summary_inputs(ss, v, out, rows, axle, run, defaults)
        md = js.build(**kw)
        import json as _json
        c1, c2, c3 = st.columns(3)
        c3.download_button("⬇ Save your reasons (.json)", _json.dumps(js.reasons_payload(ss), indent=1),
                           file_name=f"judge_reasons_{axle}.json", mime="application/json", key="ig_js_save")
        c1.download_button("⬇ Judge summary (.md)", md, file_name=f"judge_summary_{axle}.md",
                           mime="text/markdown", key="ig_js_md")
        if c2.button("🖨 Build judge summary PDF", key="ig_js_pdf_build"):
            try:
                import os
                import tempfile
                from suspension import project
                path = os.path.join(tempfile.gettempdir(), f"judge_summary_{axle}.pdf")
                project.render_pdf(md, path, figures={"judge_curves": fig} if fig else None)
                with open(path, "rb") as fh:
                    c2.download_button("⬇ Judge summary (.pdf)", fh.read(), file_name=f"judge_summary_{axle}.pdf",
                                       mime="application/pdf", key="ig_js_pdf")
            except Exception as e:                # noqa: BLE001
                c2.warning(f"PDF unavailable: {e}")


def _report_md(v, out, rows, axle, note, run):
    """A self-contained markdown design report of the current state."""
    from suspension import genesis_repro as gr
    L = ["# Corner design report", "", "- Corner: " + note + ", " + axle
         + " axle"]
    if run is not None:
        man = gr.GenesisManifest.from_json(run["manifest_json"])
        w = run["result"].winner
        L.append("- Run: `" + man.name + "` · inputs sha256 `"
                 + man.inputs_sha256 + "`")
        verdict = w.verdict if w else "NO_FIT"
        if w is not None and w.yield_frac is not None:
            verdict += ", build yield " + _cell(100 * w.yield_frac) + " %"
        L.append("- Verdict: " + verdict)
    L += ["", "## Design review", "",
          "| status | check | value | unit | range |",
          "|---|---|---|---|---|"]
    for r in rows:
        L.append("| " + " | ".join([r["status"], r["check"],
                                     _cell(r["value"]), r["unit"],
                                     _range_text(r)]) + " |")
    d = out.get("kin") or {}
    if d.get("ok"):
        L += ["", "## Kinematics", "",
              "| travel (mm) | camber (deg) | toe (deg) | RC height (mm) | "
              "scrub (mm) |", "|---|---|---|---|---|"]
        for vals in zip(d["stations_mm"], d["camber_deg"], d["toe_deg"],
                        d["rc_height_mm"], d["scrub_mm"]):
            L.append("| " + " | ".join(_cell(x) for x in vals) + " |")
    sc = out.get("sc")
    if sc:
        L += ["", "## Link factors of safety", "",
              "| member | worst FoS | governing case |", "|---|---|---|"]
        for m, f in sc["worst_fos_per_member"].items():
            L.append("| " + m + " | " + _cell(f) + " | "
                     + sc["governing_case"][m] + " |")
    L += ["", "## Vehicle & build declaration", "", "| field | value |",
          "|---|---|"]
    for k, x in v.items():
        L.append("| " + k + " | " + _cell(x) + " |")
    L += ["", "_Analytical results from declared inputs; review ranges are "
          "starting points, not requirements._", ""]
    return "\n".join(L)
