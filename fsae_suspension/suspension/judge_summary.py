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
        rng = (f"{_num(lo)} … {_num(hi)}" if has_lo and has_hi else
               f"≥ {_num(lo)}" if has_lo else f"≤ {_num(hi)}" if has_hi else "—")
        ok = None if v is None else ((not has_lo or float(v) >= float(lo)) and (not has_hi or float(v) <= float(hi)))
        label = p["label"][:1].upper() + p["label"][1:]
        rows.append((label, rng, _num(v), ok))
    return rows


def answer_key(question):
    """A stable widget key for a question's answer: its check name, not its position or its value, so an answer
    stays with its question when the design (and so the list or the numbers in it) changes. Text only, unitless."""
    import re
    return "ig_ans_" + (re.sub(r"[^a-z]+", "_", question.split(":")[0].lower()).strip("_") or "q")


def reasons_payload(ss):
    """The team's written reasons and answers in session state, as a dict that can be saved and loaded back. Text only, unitless."""
    return {k: v for k, v in ss.items()
            if isinstance(k, str) and isinstance(v, str) and k.startswith(("ig_rat_", "ig_ans_", "ig_js_team"))}


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
          questions, answers, figure_ref=None, appendix_name=None, not_checked=None):
    """The judge summary as Markdown (angles in deg, lengths in mm, factors of safety dimensionless).

    ``decisions`` maps a DECISIONS key to {"target", "result", "evidence"} (strings the tool fills from the
    analysis); ``rationale`` maps the same keys to {"why", "alternatives", "trade_off"} in the team's words (any
    missing or blank is printed as MISSING); ``evidence`` is a list of sentences on how the numbers were produced;
    ``tools`` is a list of sentences on the tools and their validation; ``loads`` is [(member, worst FoS,
    governing case)]; ``safety`` a list of sentences; ``questions`` from open_questions; ``answers`` maps a question
    to the team's answer. ``figure_ref`` names the curves figure for render_pdf. ``not_checked`` lists what the
    pre-validation does not cover, printed as its own section so a judge reads the limits from the team first."""
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

    L += ["## 3. Use of design tools and analysis", ""] + [f"- {s}" for s in tools + evidence] + [""]

    L += ["## 4. Load cases and load path", "",
          "| link | worst factor of safety | governing load case |", "|---|---|---|"]
    L += [f"| {m} | {_num(f)} | {c} |" for m, f, c in loads] + [""]
    if loads:
        L += ["Load path: tyre contact patch, upright, ball joints and rod ends, links (axial members on spherical "
              "joints), chassis pickups. Every link force, per load case, is in the appendix; what is not checked "
              "here is in section 7.", ""]

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
