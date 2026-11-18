# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The judge summary: one corner's design, written the way the Formula Student Design Event scores it.

The FS 2025 Design Judging Score Sheet scores suspension under area B, "Structural Design inc Suspension & Brakes:
specifications, use of design tools & analysis, load cases and load path appreciation, safety considerations", and
asks every team to "explain why they chose this design and from what other possible solutions … and justify fully
with facts and data". Judges score the team's understanding and rationale; a tool that stands in for them is a red
flag. So this summary is built around the team's own words: for every design decision it prints the target, the
team's reason, the alternatives they considered, the evidence and the trade-off accepted, and where the team has not
written a reason it says so in bold rather than filling the gap. The tool supplies the facts and data; the reasons
are the team's.

Pure functions: the inputs are plain dicts and lists the InverseGenesis tab already holds, so the summary can be
built and tested without Streamlit. Angles in deg, lengths in mm, forces in N, fractions dimensionless.
"""
from __future__ import annotations

import math

#: Score-sheet area every section answers (FS 2025 universal design judging score sheet, area B).
SCORE_SHEET_AREA = ("B: Structural Design inc Suspension & Brakes (specifications, use of design tools & analysis, "
                    "load cases and load path appreciation, safety considerations)")

#: The decisions a suspension judge asks about, in the order they are usually discussed: key, title, the question
#: the team's reason must answer.
DECISIONS = (
    ("camber", "Camber gain and static camber",
     "Why this static camber and this gain: what tyre camber did you want at the design lateral acceleration, and "
     "how did you choose it?"),
    ("roll_centre", "Roll-centre height and migration",
     "Why this roll-centre height, and why this much (or little) migration?"),
    ("toe", "Bump steer",
     "Why this toe curve over travel, and how did you place the tie rod to get it?"),
    ("anti_dive", "Anti-dive",
     "Why this much anti-dive, and what did you give up for it (brake feel, pitch, jacking)?"),
    ("steering", "Steering geometry: caster, kingpin inclination, scrub, Ackermann",
     "Why these values, and what steering effort and feel do they give the driver?"),
    ("ride", "Motion ratio and ride",
     "Why this motion ratio and its shape over travel, and what ride frequency did you aim for?"),
    ("loads", "Load cases and load path",
     "Why these load cases, and how does the load reach the chassis?"),
    ("safety", "Safety margins",
     "Which margins did you set, why, and how will you check them on the built car?"),
)

#: A reason the team has not written is printed as this, so the gap is visible and never filled by the tool.
MISSING = "**Team to state.**"


def _num(x, digits=4):
    """A number as a judge reads it: no float noise, no scientific notation for ordinary values."""
    if x is None:
        return "—"
    try:
        v = float(x)
    except (TypeError, ValueError):
        return str(x)
    if not math.isfinite(v):
        return "—"
    if v != 0 and (abs(v) < 1e-3 or abs(v) >= 1e6):
        return f"{v:.3g}"
    return f"{v:.{digits}g}"


def specification_rows(targets, kin, props):
    """[(parameter, target, delivered, met)] for the summary's first table.

    ``targets`` is [{"channel", "target_at", "band"}] (the target curve's static value and its tightest band),
    ``kin`` the kinematics table of the corner (camber gain, bump steer, RC height, caster, KPI, scrub, anti-dive),
    ``props`` [{"label", "lo", "hi", "value"}] for every bound the search enforced. A row is met when the delivered
    value is inside its target."""
    rows = []
    for t in targets:
        name = {"camber_deg": "Camber at ride height (deg)", "toe_deg": "Toe at ride height (deg)",
                "rc_height_mm": "Roll-centre height (mm)", "scrub_mm": "Scrub radius (mm)"}.get(t["channel"],
                                                                                              t["channel"])
        got = t.get("delivered")
        ok = None if got is None else abs(float(got) - float(t["target_at"])) <= float(t["band"])
        rows.append((name, f"{_num(t['target_at'])} ± {_num(t['band'])}", _num(got), ok))
    for p in props:
        lo, hi, v = p.get("lo"), p.get("hi"), p.get("value")
        has_lo = lo is not None and math.isfinite(float(lo))
        has_hi = hi is not None and math.isfinite(float(hi))
        rng = (f"{_num(lo)} to {_num(hi)}" if has_lo and has_hi else
               f"≥ {_num(lo)}" if has_lo else f"≤ {_num(hi)}" if has_hi else "—")
        ok = None if v is None else ((not has_lo or float(v) >= float(lo)) and (not has_hi or float(v) <= float(hi)))
        label = p["label"][:1].upper() + p["label"][1:]
        rows.append((label, rng, _num(v), ok))
    return rows


def target_chain(*, track_mm, body_roll_deg, tyre_optimum_camber_deg, static_camber_deg, camber_gain_deg_per_mm,
                 lateral_mu, mass_kg, cg_height_mm, wheelbase_mm, sprung_corner_kg, ride_hz=None, anti_pct=None,
                 axle="front", sources=None):
    """One level up the V: each kinematic target derived from the car's requirements (suspension.target_derivation),
    as [(target, what it follows from, how, value)]. Angles in deg, lengths in mm, masses in kg, frequency in Hz,
    anti in %. ``sources`` maps an input name to 'declared' or 'default', printed next to it so a judge sees which
    requirements are the team's own."""
    from . import target_derivation as td
    src = sources or {}

    def tag(*names):
        d = [n for n in names if src.get(n) == "default"]
        return f" (tool default: {', '.join(d)})" if d else ""
    veh = td.Vehicle(mass_kg=mass_kg, cg_height_mm=cg_height_mm, wheelbase_mm=wheelbase_mm, track_mm=track_mm,
                     sprung_corner_front_kg=sprung_corner_kg, sprung_corner_rear_kg=sprung_corner_kg)
    tyre = td.Tire(camber_opt_deg=tyre_optimum_camber_deg, peak_mu=lateral_mu)
    acc = td.design_accelerations(tyre)
    z = 0.5 * track_mm * math.tan(math.radians(body_roll_deg))
    gain = (tyre_optimum_camber_deg - body_roll_deg - static_camber_deg) / z if z else float("nan")
    rows = [("Design lateral acceleration", "the tyre's peak friction" + tag("lateral_mu"),
             f"{_num(td.LAT_REF_G)} g scaled by µ {_num(lateral_mu)} / {_num(td.MU_REF)}", f"{_num(acc['lateral'], 3)} g"),
            ("Camber gain", "keep the outside tyre at its optimum camber at that acceleration"
             + tag("tyre_optimum_camber", "body_roll_deg"),
             f"(optimum {_num(tyre_optimum_camber_deg)}° − roll {_num(body_roll_deg)}° − static "
             f"{_num(static_camber_deg)}°) / outside-wheel bump {_num(z, 3)} mm", f"{_num(gain)} deg/mm"),
            ("Camber gain delivered", "the geometry", "from the kinematics", f"{_num(camber_gain_deg_per_mm)} deg/mm")]
    if ride_hz:
        fl = td.anti_floor_pct(veh, ride_hz, acc["combined"], acc["combined"], axle)
        rows.append(("Minimum anti-" + ("dive" if axle == "front" else "squat"),
                     "keep combined braking and cornering inside the jounce budget"
                     + tag("mass_kg", "cg_height_mm", "wheelbase_mm"),
                     f"ride frequency {_num(ride_hz, 3)} Hz, {_num(acc['combined'], 3)} g combined, "
                     f"{_num(veh.jounce_budget_mm)} mm jounce budget", f"{_num(max(fl, 0.0), 3)} %"))
        if anti_pct is not None:
            rows.append(("Anti delivered", "the geometry", "contact-patch path", f"{_num(anti_pct, 3)} %"))
    return rows


#: What usually sets each kind of target, printed beside a target whose origin the team has not stated, so the gap
#: says what a judge will ask. Matched as a substring of the target's name in lower case; the first match wins.
ORIGIN_HINTS = (
    ("camber at ride height", "the tyre's camber window, chosen together with the camber gain"),
    ("toe at ride height", "straight-line stability against turn-in response, and the build tolerance on toe"),
    ("roll-centre", "the front share of lateral load transfer the balance needs, and the jacking the car accepts"),
    ("anti-dive", "pitch under braking against brake feel, from the brake bias and the jounce budget"),
    ("anti-squat", "pitch under acceleration against traction feel, from the drive layout and the jounce budget"),
    ("kingpin", "packaging the ball joints inside the rim, and the self-centring it adds with steer"),
    ("scrub", "steering feedback and kickback from braking forces and kerbs"),
    ("caster", "self-centring and steering weight at the design lateral acceleration"),
    ("ratio", "the steering-wheel torque target and the driver's hand travel"),
    ("steering-wheel angle", "the tightest corner's radius and the driver's hand travel"),
    ("ackermann", "the tightest corner's radius and speed"),
    ("deceleration", "the tyre's friction and the brake bias"),
    ("buckling", "the team's factor-of-safety policy for the shock cases"),
    ("safety factor", "the team's factor-of-safety policy"),
    ("compliance steer", "the toe band left after build scatter"),
    ("misalignment", "the rod ends' rated misalignment"),
)


def origin_key(target_name) -> str:
    """A stable widget key for the team's statement of where a target came from: its name, not its position."""
    import re
    return "ig_js_from_" + (re.sub(r"[^a-z]+", "_", str(target_name).lower()).strip("_") or "t")


def target_origin_rows(specs, origins):
    """Every declared target as a row of the 'where the targets came from' table, [(target, follows from, how,
    target value)]: the team's own statement from ``origins`` (target name -> text), or MISSING with what usually
    sets that kind of target, so a judge sees which targets the team has traced one level up and which it has not.
    The rows are text (unitless); each target's own units are printed in its name."""
    rows = []
    for name, tgt, _got, _ok in specs:
        txt = str((origins or {}).get(name) or "").strip()
        if txt:
            rows.append((name, txt, "stated by the team", str(tgt)))
        else:
            hint = next((h for k, h in ORIGIN_HINTS if k in str(name).lower()), "")
            rows.append((name, MISSING + (f" Usually set by {hint}." if hint else ""), "—", str(tgt)))
    return rows


def answer_key(question):
    """A stable widget key for a question's answer: its check name, not its position or its value, so an answer
    stays with its question when the design (and so the list or the numbers in it) changes. Text only, unitless."""
    import re
    return "ig_ans_" + (re.sub(r"[^a-z]+", "_", question.split(":")[0].lower()).strip("_") or "q")


def reasons_payload(ss):
    """The team's written reasons and answers in session state, as a dict that can be saved and loaded back. Text only, unitless."""
    return {k: v for k, v in ss.items()
            if isinstance(k, str) and isinstance(v, str)
            and k.startswith(("ig_rat_", "ig_ans_", "ig_js_team", "ig_js_weighting", "ig_js_requirements",
                                       "ig_js_from_"))}


def open_questions(review_rows, thin, self_graded, defaults_count, inputs_count):
    """Every point a judge will raise that the analysis cannot settle, as [(point, why it will be asked)].

    Each value is printed in its review unit (deg, mm, %, or dimensionless for ratios and fractions)."""
    out = []
    for r in review_rows:
        if r["status"] in ("fail", "watch"):
            out.append((f"{r['check']}: {_num(r['value'])} {r['unit']}".rstrip(),
                        ("outside" if r["status"] == "fail" else "close to") + f" the range {r['range']}. "
                        + (r.get("why") or "")))
    for check, value, generic in self_graded:
        out.append((f"{check}: {value}", f"inside the team's own range but outside the generic starting range "
                                         f"{generic}; a judge will ask why this car wants it."))
    if thin:
        out.append(("Properties met with little margin: " + "; ".join(f"{lab} {m}" for lab, m in thin),
                    "met by the nominal geometry; build tolerances can take them outside their bounds."))
    if defaults_count:
        out.append((f"{defaults_count} of {inputs_count} analysis inputs are tool defaults",
                    "ride, roll stiffness, steering effort and some checks rest on tool defaults, not on values "
                    "from this car."))
    return out


def build(*, title, team, corner_name, axle, specs, decisions, rationale, evidence, tools, loads, safety,
          questions, answers, figure_ref=None, appendix_name=None, not_checked=None, parts=None,
          target_chain=None, iterations=None, weighting="", requirements="", origins=None):
    """The judge summary as Markdown (angles in deg, lengths in mm, factors of safety dimensionless).

    ``decisions`` maps a DECISIONS key to {"target", "result", "evidence"} (strings the tool fills from the
    analysis); ``rationale`` maps the same keys to {"why", "alternatives", "trade_off"} in the team's words (any
    missing or blank is printed as MISSING); ``evidence`` is a list of sentences on how the numbers were produced;
    ``tools`` is a list of sentences on the tools and their validation; ``loads`` is [(member, worst FoS,
    governing case)]; ``safety`` a list of sentences; ``questions`` from open_questions; ``answers`` maps a question
    to the team's answer. ``figure_ref`` names the curves figure for render_pdf. ``not_checked`` lists what the
    pre-validation does not cover, printed as its own section so a judge reads the limits from the team first.
    ``parts`` is [(part, check, factor of safety or None, what it needs, governing case)] for the parts the links
    load: brackets, upright, hub bearings, and fatigue of welds and rod ends. ``target_chain`` is [(target, what it
    follows from, how, value)] from target_chain(): where the targets came from, one level up the V. ``iterations``
    is [(step, what changed, outcome)]: how the design got here. ``weighting`` is the team's own account of how the
    trade-offs were weighted against each other. ``requirements`` is the team's account of where the car-level
    requirements behind the chain came from (the tyre, the ride frequency, the design accelerations): two levels up
    the V. ``origins`` maps a target's name to the team's statement of where it came from; when given, every target in
    ``specs`` gets a row in the same table (target_origin_rows), so none is left without a stated origin unseen."""
    from .project import FIGURE_SCHEME
    head = f"{title}" + (f" · {team}" if team else "")
    L = [f"# {head}", "",
         f"Design summary of the {axle} suspension corner {corner_name}, for the Formula Student Design Event. "
         f"Score-sheet area {SCORE_SHEET_AREA}.", "",
         "The reasons, alternatives and trade-offs below are the team's own words. The tool supplies the facts "
         "and data behind them; where a reason has not been written, the summary says so.", "",
         "This is a pre-validation: it checks the design before it is built and tested. Section 7 lists what it "
         "does not cover.", ""]

    L += ["## 1. What we set out to achieve", "",
          "| parameter | target | delivered | met |", "|---|---|---|---|"]
    for name, tgt, got, ok in specs:
        L.append(f"| {name} | {tgt} | {got} | {'yes' if ok else 'no' if ok is not None else '—'} |")
    L.append("")
    L += ["### Where the car-level requirements came from", "",
          (requirements or "").strip() or f"{MISSING} Why this tyre, this ride frequency and these design "
          "accelerations: the top-level objectives (lap time, event points, driver feedback, rules) they follow "
          "from (two levels up the V).", ""]
    L += ["### Where the targets came from", ""]
    chain_rows = list(target_chain or []) + (target_origin_rows(specs, origins) if origins is not None else [])
    if origins is not None:
        L += ["The first rows are derived by the tool from the car's requirements; the rest are the team's own "
              "statement of where each target came from.", ""]
    if chain_rows:
        L += ["| target | follows from | how | value |", "|---|---|---|---|"]
        L += [f"| {t} | {f} | {h} | {v} |" for t, f, h, v in chain_rows] + [""]
    else:
        L += [f"{MISSING} How each target follows from the car's requirements (one level up the V).", ""]
    if figure_ref:
        L += [f"![Camber, toe and roll-centre height over wheel travel, against the target bands]"
              f"({FIGURE_SCHEME}{figure_ref})", ""]

    L += ["## 2. Why this design, and from what alternatives", ""]
    for key, label, question in DECISIONS:
        d = decisions.get(key) or {}
        r = rationale.get(key) or {}
        L += [f"### {label}", ""]
        if d.get("target"):
            L.append(f"- **Target:** {d['target']}")
        L.append(f"- **Why:** {r.get('why', '').strip() or MISSING + ' ' + question}")
        L.append(f"- **Alternatives considered:** {r.get('alternatives', '').strip() or MISSING}")
        if d.get("result"):
            L.append(f"- **Result:** {d['result']}")
        if d.get("evidence"):
            L.append(f"- **Evidence:** {d['evidence']}")
        if r.get("trade_off", "").strip() or d.get("trade_off"):
            L.append(f"- **Trade-off accepted:** {r.get('trade_off', '').strip() or d.get('trade_off')}")
        L.append("")

    L += ["### How the trade-offs were weighted", "",
          (weighting or "").strip() or f"{MISSING} Which requirement won where two pulled against each other, and "
          "why (for example lap time, points per event, or driver feedback).", ""]
    L += ["### How the design got here", ""]
    if iterations:
        L += ["| step | what changed | outcome |", "|---|---|---|"]
        L += [f"| {s} | {c} | {o} |" for s, c, o in iterations] + [""]
    else:
        L += [f"{MISSING} The iterations: what was tried, what it showed, and what changed as a result.", ""]

    L += ["## 3. Use of design tools and analysis", ""] + [f"- {s}" for s in tools + evidence] + [""]

    L += ["## 4. Load cases and load path", "",
          "| link | worst factor of safety | governing load case |", "|---|---|---|"]
    L += [f"| {m} | {_num(f)} | {c} |" for m, f, c in loads] + [""]
    if loads:
        L += ["Load path: tyre contact patch, hub bearings, upright, ball joints and rod ends, links (axial members "
              "on spherical joints), pickup brackets, frame. Every link force and upright joint load, per load "
              "case, is in the appendix; what is not checked here is in section 7.", ""]
    if parts:
        L += ["**Where the link loads go**", "",
              "| part | check | factor of safety | needs | governing case |", "|---|---|---|---|---|"]
        L += [f"| {p} | {c} | {_num(f) if f is not None else 'not declared'} | {n or '—'} | {g} |"
              for p, c, f, n, g in parts] + [""]

    L += ["## 5. Safety considerations", ""] + [f"- {s}" for s in safety] + [""]

    L += ["## 6. Questions a judge will ask, and the team's answers", ""]
    if not questions:
        L += ["None outstanding.", ""]
    for q, why in questions:
        a = (answers or {}).get(q, "").strip()
        L += [f"- **{q}** ({why.strip()})", f"  - Team answer: {a or MISSING}"]
    L.append("")
    L += ["## 7. What this pre-validation does not cover, and how the team will check it", ""]
    L += [f"- {s}" for s in (not_checked or ["Nothing listed."])] + [""]
    if appendix_name:
        L += [f"Every number above is traced, with its inputs and their sources, in the appendix: {appendix_name}.",
              ""]
    return "\n".join(L)


def curves_png(stations, series, bands):
    """PNG bytes of camber, toe and roll-centre height over wheel travel, each with its target band shaded.

    ``stations`` mm; ``series`` {channel: values at the stations}; ``bands`` {channel: (travel, target, band)} from
    the target curves. Channels without data are left out."""
    import io

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    panels = [("camber_deg", "Camber (deg)"), ("toe_deg", "Toe (deg)"), ("rc_height_mm", "Roll-centre height (mm)")]
    panels = [p for p in panels if p[0] in series]
    fig, axes = plt.subplots(1, len(panels), figsize=(3.2 * len(panels), 2.6), dpi=150)
    axes = np.atleast_1d(axes)
    for ax, (ch, label) in zip(axes, panels):
        if ch in bands:
            tr, tg, bd = (np.asarray(x, float) for x in bands[ch])
            ax.fill_between(tr, tg - bd, tg + bd, color="#cfe8df", label="target band")
            ax.plot(tr, tg, color="#6aa891", lw=1, ls="--", label="target")
        ax.plot(stations, series[ch], color="#1f4e79", lw=1.8, marker="o", ms=3, label="delivered")
        ax.set_xlabel("wheel travel (mm, + bump)", fontsize=7)
        ax.set_title(label, fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=6, loc="best")
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    plt.close(fig)
    return buf.getvalue()
