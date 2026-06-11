"""
KinematiK — open-source Formula SAE suspension design studio.

Edit double-wishbone hardpoints live and watch the kinematics (camber gain, bump
steer, caster, KPI, scrub) and the vehicle-level consequences (roll-centre
migration, lateral load transfer, grip balance) update together. Built for the
FSAE garage where OptimumK / ADAMS budgets don't reach.

Run:  streamlit run app.py
"""

import json
import os
import tempfile
import datetime as _datetime
import numpy as np
import streamlit as st
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from suspension import (
    SuspensionKinematics, Hardpoints,
    VehicleDynamics, VehicleParams,
)
from suspension import chassis as chassis_mod
from suspension import integration as integ_mod
from suspension import project as project_mod

st.set_page_config(page_title="KinematiK · FSAE Suspension Studio",
                   page_icon="◢", layout="wide",
                   initial_sidebar_state="expanded")

# --------------------------------------------------------------------------- #
#  Aesthetic: technical instrument panel. Dark carbon, amber/cyan telemetry,
#  monospace data, a single high-contrast accent. No generic dashboard look.
# --------------------------------------------------------------------------- #
st.markdown("""
<style>
@import url('https://fonts.googleapis.com/css2?family=Archivo:wght@400;600;800&family=JetBrains+Mono:wght@400;600&display=swap');

:root{
  --bg:#0b0d10; --panel:#13171c; --panel2:#171c22;
  --line:#262d36; --ink:#e7ecf1; --dim:#8d99a6;
  --amber:#ffb02e; --cyan:#37e0d0; --red:#ff5a52; --grid:#1d242c;
}
.stApp{ background:
  radial-gradient(1200px 600px at 80% -10%, #14202655 0%, transparent 60%),
  var(--bg); color:var(--ink); }
section[data-testid="stSidebar"]{ background:var(--panel); border-right:1px solid var(--line); }
h1,h2,h3,h4{ font-family:'Archivo',sans-serif!important; letter-spacing:-.02em; }
body, p, span, div, label{ font-family:'Archivo',sans-serif; }
.mono, .stMetric, code{ font-family:'JetBrains Mono',monospace!important; }

.brand{ display:flex; align-items:baseline; gap:.6rem; border-bottom:1px solid var(--line);
        padding-bottom:.5rem; margin-bottom:.2rem;}
.brand .mark{ font-family:'Archivo'; font-weight:800; font-size:2.1rem;
        background:linear-gradient(90deg,var(--amber),var(--cyan)); -webkit-background-clip:text;
        -webkit-text-fill-color:transparent; }
.brand .sub{ color:var(--dim); font-family:'JetBrains Mono'; font-size:.78rem; letter-spacing:.18em; text-transform:uppercase;}

.card{ background:linear-gradient(180deg,var(--panel2),var(--panel));
       border:1px solid var(--line); border-radius:14px; padding:1.0rem 1.1rem; }
.metric{ display:flex; flex-direction:column; gap:.15rem; padding:.7rem .9rem;
         border:1px solid var(--line); border-radius:12px; background:var(--panel2);}
.metric .v{ font-family:'JetBrains Mono'; font-weight:600; font-size:1.45rem; line-height:1; }
.metric .k{ color:var(--dim); font-size:.7rem; letter-spacing:.12em; text-transform:uppercase;}
.metric .u{ color:var(--dim); font-size:.85rem; font-weight:400;}
.tag{ display:inline-block; font-family:'JetBrains Mono'; font-size:.7rem; padding:.18rem .5rem;
      border-radius:6px; border:1px solid var(--line); color:var(--dim);}
.good{ color:var(--cyan); border-color:#1f4d49;}
.warn{ color:var(--amber); border-color:#5a4317;}
.bad{ color:var(--red); border-color:#5a2422;}
.stTabs [data-baseweb="tab-list"]{ gap:2px; }
.stTabs [data-baseweb="tab"]{ background:var(--panel); border:1px solid var(--line);
      border-bottom:none; border-radius:10px 10px 0 0; color:var(--dim); font-family:'JetBrains Mono'; font-size:.8rem;}
.stTabs [aria-selected="true"]{ color:var(--ink); background:var(--panel2); border-color:#34507c;}
.hint{ color:var(--dim); font-size:.82rem; }
hr{ border-color:var(--line);}
[data-testid="stMetricValue"]{ font-family:'JetBrains Mono'!important;}

/* Buttons and download buttons — dark theme (Streamlit defaults render white) */
.stButton > button, .stDownloadButton > button{
  background:var(--panel2)!important;
  color:var(--ink)!important;
  border:1px solid var(--line)!important;
  border-radius:10px!important;
  font-family:'JetBrains Mono',monospace!important;
  font-size:.82rem!important;
  font-weight:600!important;
  transition:border-color .15s ease, background .15s ease;
}
.stButton > button:hover, .stDownloadButton > button:hover{
  border-color:var(--amber)!important;
  background:#1b222a!important;
  color:var(--amber)!important;
}
.stButton > button:active, .stDownloadButton > button:active{ background:#11161b!important; }
.stButton > button:focus, .stDownloadButton > button:focus{
  box-shadow:none!important; border-color:var(--amber)!important;
}
.stTextInput input, .stTextArea textarea, .stNumberInput input,
.stSelectbox div[data-baseweb="select"] > div{
  background:var(--panel2)!important; color:var(--ink)!important; border-color:var(--line)!important;
}
.stFileUploader > div{ background:var(--panel2)!important; border-color:var(--line)!important; }
</style>
""", unsafe_allow_html=True)

PLOT_LAYOUT = dict(
    paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="#0e1216",
    font=dict(family="JetBrains Mono, monospace", color="#cdd6df", size=11),
    margin=dict(l=55, r=20, t=40, b=45),
    xaxis=dict(gridcolor="#1d242c", zerolinecolor="#33414e"),
    yaxis=dict(gridcolor="#1d242c", zerolinecolor="#33414e"),
    legend=dict(bgcolor="rgba(0,0,0,0)", font=dict(size=10)),
)
AMBER, CYAN, RED, DIM = "#ffb02e", "#37e0d0", "#ff5a52", "#8d99a6"


# --------------------------------------------------------------------------- #
#  State
# --------------------------------------------------------------------------- #
def init_state():
    if "hp" not in st.session_state:
        st.session_state.hp = Hardpoints.default().as_dict()
    if "vp" not in st.session_state:
        st.session_state.vp = VehicleParams().__dict__.copy()

init_state()

POINTS = [
    ("upper_front_inner", "Upper wishbone · front inner (chassis)"),
    ("upper_rear_inner",  "Upper wishbone · rear inner (chassis)"),
    ("lower_front_inner", "Lower wishbone · front inner (chassis)"),
    ("lower_rear_inner",  "Lower wishbone · rear inner (chassis)"),
    ("upper_outer",       "Upper ball joint (upright)"),
    ("lower_outer",       "Lower ball joint (upright)"),
    ("tie_rod_inner",     "Tie rod · inner (rack)"),
    ("tie_rod_outer",     "Tie rod · outer (upright)"),
    ("wheel_center",      "Wheel centre"),
    ("contact_patch",     "Contact patch"),
]


def metric(label, value, unit="", cls=""):
    return f"""<div class="metric"><span class="k">{label}</span>
    <span class="v {cls}">{value}<span class="u"> {unit}</span></span></div>"""


PROJECT_PATH = os.path.join(os.getcwd(), "project.json")


def log_decision_now(team, title, rationale, author="auto"):
    """Append a decision straight to the persistent store from any tab."""
    st_ = project_mod.ProjectStore(PROJECT_PATH)
    st_.add_decision(project_mod.Decision(
        team=team, title=title, rationale=rationale, author=author,
        tags="auto-captured"))
    st_.save()


# --------------------------------------------------------------------------- #
#  Sidebar — geometry editor
# --------------------------------------------------------------------------- #
with st.sidebar:
    st.markdown('<div class="brand"><span class="mark">◢ KinematiK</span></div>',
                unsafe_allow_html=True)
    st.markdown('<div class="sub" style="color:#8d99a6;font-family:JetBrains Mono;font-size:.7rem;letter-spacing:.18em;margin-bottom:.6rem;">HARDPOINT EDITOR · mm · SAE x-rear y-right z-up</div>', unsafe_allow_html=True)

    colA, colB = st.columns(2)
    if colA.button("↺ Reset", use_container_width=True):
        st.session_state.hp = Hardpoints.default().as_dict()
        st.rerun()
    preset = colB.selectbox("Preset", ["Front (default)", "Low roll-centre",
                                       "High anti-dive"], label_visibility="collapsed")

    st.markdown("###### Design intent")
    c1, c2 = st.columns(2)
    st.session_state.hp["static_camber"] = c1.number_input(
        "Static camber °", value=float(st.session_state.hp.get("static_camber", -1.5)),
        step=0.1, format="%.2f")
    st.session_state.hp["static_toe"] = c2.number_input(
        "Static toe °", value=float(st.session_state.hp.get("static_toe", 0.0)),
        step=0.05, format="%.2f")

    st.markdown("###### Pickup coordinates")
    for key, label in POINTS:
        with st.expander(label, expanded=False):
            v = st.session_state.hp[key]
            cols = st.columns(3)
            nv = []
            for i, ax in enumerate("xyz"):
                nv.append(cols[i].number_input(
                    f"{ax}", value=float(v[i]), step=2.0, key=f"{key}_{ax}",
                    format="%.1f", label_visibility="visible"))
            st.session_state.hp[key] = nv

    st.markdown("---")
    st.markdown("###### Vehicle")
    vp = st.session_state.vp
    vp["mass"] = st.slider("Mass + driver (kg)", 180, 360, int(vp["mass"]))
    vp["cg_height"] = st.slider("CG height (mm)", 200, 400, int(vp["cg_height"]))
    vp["weight_dist_front"] = st.slider("Front weight (%)", 40, 60,
                                        int(vp["weight_dist_front"] * 100)) / 100
    cc1, cc2 = st.columns(2)
    vp["roll_stiffness_front"] = cc1.number_input("Roll stiff F (N·m/°)",
                                                  value=float(vp["roll_stiffness_front"]), step=10.0)
    vp["roll_stiffness_rear"] = cc2.number_input("Roll stiff R (N·m/°)",
                                                 value=float(vp["roll_stiffness_rear"]), step=10.0)


# Apply presets (simple variations on the default)
def apply_preset(name, hp):
    hp = dict(hp)
    if name == "Low roll-centre":
        hp["lower_front_inner"][2] = 95
        hp["lower_rear_inner"][2] = 95
    elif name == "High anti-dive":
        hp["lower_rear_inner"][2] = 150
        hp["upper_rear_inner"][2] = 320
    return hp

hp_dict = apply_preset(preset, st.session_state.hp)


# --------------------------------------------------------------------------- #
#  Solve
# --------------------------------------------------------------------------- #
try:
    hp = Hardpoints.from_dict(hp_dict)
    kin = SuspensionKinematics(hp)
    veh = VehicleDynamics(VehicleParams(**st.session_state.vp),
                          front_kin=kin, rear_kin=kin)
    sweep = kin.sweep(-30, 30, 41)
    solve_ok = all(s.converged for s in sweep)
except Exception as e:
    st.error(f"Solver failed for this geometry: {e}")
    st.stop()

st.markdown('<div class="brand"><span class="mark">◢ KinematiK</span>'
            '<span class="sub">FSAE double-wishbone studio · open source</span></div>',
            unsafe_allow_html=True)

s = kin.static
mid = veh.lateral_load_transfer(1.2)[1]

# headline metrics
def gain(metric_fn):
    a = metric_fn(kin.solve_at_travel(-10))
    b = metric_fn(kin.solve_at_travel(10))
    return (b - a) / 20.0  # per mm

camber_gain = gain(lambda st_: st_.camber)
bump_steer = gain(lambda st_: st_.toe)

cols = st.columns(6)
items = [
    ("Static camber", f"{s.camber:+.2f}", "°", ""),
    ("Camber gain", f"{camber_gain*10:+.2f}", "°/10mm",
     "good" if camber_gain < 0 else "warn"),
    ("Bump steer", f"{bump_steer*10:+.3f}", "°/10mm",
     "good" if abs(bump_steer*10) < 0.1 else "warn"),
    ("Caster", f"{s.caster:+.1f}", "°", ""),
    ("KPI", f"{s.kpi:+.1f}", "°", ""),
    ("Scrub radius", f"{s.scrub_radius:+.0f}", "mm",
     "good" if abs(s.scrub_radius) < 25 else "warn"),
]
for c, (k, v, u, cls) in zip(cols, items):
    c.markdown(metric(k, v, u, cls), unsafe_allow_html=True)

if not solve_ok:
    st.markdown('<span class="tag bad">⚠ linkage does not close over full travel — '
                'check wishbone lengths</span>', unsafe_allow_html=True)

st.write("")
with st.expander("👋 New here? Start here (30-second tour)", expanded=False):
    st.markdown("""
**What KinematiK is:** a shared tool for the whole FSAE team. It does two jobs —
checks parts against the chassis before you manufacture, and keeps a searchable
record of *why* the team made its design decisions so that knowledge doesn't vanish
at graduation.

**Where to go, by what you want to do:**
- **Designing suspension geometry?** → *Kinematics*, *Roll & Load Transfer*, *Grip
  Balance*, *Geometry 3D* tabs. Edit hardpoints in the sidebar; everything updates live.
- **Checking if your part fits the chassis?** → *Team Fit* tab. Load the chassis once,
  load your part, get a collision/clearance verdict before you cut anything.
- **Suspension vs chassis clearance through travel?** → *Suspension vs Chassis* tab.
- **Logging a decision / tracking weight / handover?** → *Weight & Handover* tab.
  Tap a quick-template, fill the brackets, done. This is the part next year's team
  will thank you for.
- **Leaving a note for another subteam?** → *Lead Notes* tab.

**The one habit that makes this worth it:** log your decisions as you make them —
especially the things that *didn't* work. It takes ten seconds with the templates,
and it's the difference between next year starting ahead or relearning everything.
    """)

tab1, tab2, tab3, tab4, tab5, tab6, tab7, tab8 = st.tabs(
    ["  KINEMATICS  ", "  ROLL & LOAD TRANSFER  ", "  GRIP BALANCE  ",
     "  GEOMETRY 3D  ", "  SUSPENSION vs CHASSIS  ", "  TEAM FIT  ",
     "  WEIGHT & HANDOVER  ", "  LEAD NOTES  "])

travels = [st_.travel for st_ in sweep]

# ----------------------------- TAB 1 --------------------------------------- #
with tab1:
    c1, c2 = st.columns(2)
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=travels, y=[st_.camber for st_ in sweep],
                  mode="lines", line=dict(color=CYAN, width=3), name="Camber"))
    fig.update_layout(**PLOT_LAYOUT, title="Camber vs wheel travel",
                      xaxis_title="travel (mm, + bump)", yaxis_title="camber (°)",
                      height=340)
    c1.plotly_chart(fig, use_container_width=True)

    fig2 = go.Figure()
    fig2.add_trace(go.Scatter(x=travels, y=[st_.toe for st_ in sweep],
                   mode="lines", line=dict(color=AMBER, width=3), name="Toe"))
    fig2.update_layout(**PLOT_LAYOUT, title="Bump steer (toe vs travel)",
                       xaxis_title="travel (mm, + bump)", yaxis_title="toe (°, + out)",
                       height=340)
    c2.plotly_chart(fig2, use_container_width=True)

    c3, c4 = st.columns(2)
    fig3 = go.Figure()
    fig3.add_trace(go.Scatter(x=travels, y=[st_.scrub_radius for st_ in sweep],
                   mode="lines", line=dict(color="#9b8cff", width=3)))
    fig3.update_layout(**PLOT_LAYOUT, title="Scrub radius vs travel",
                       xaxis_title="travel (mm)", yaxis_title="scrub (mm)", height=320)
    c3.plotly_chart(fig3, use_container_width=True)

    fig4 = go.Figure()
    fig4.add_trace(go.Scatter(x=travels, y=[st_.caster for st_ in sweep],
                   mode="lines", line=dict(color="#62d27a", width=3)))
    fig4.update_layout(**PLOT_LAYOUT, title="Caster vs travel",
                       xaxis_title="travel (mm)", yaxis_title="caster (°)", height=320)
    c4.plotly_chart(fig4, use_container_width=True)

    st.markdown('<p class="hint">Camber gain should be negative in bump so the '
                'outside wheel keeps its contact patch flat as the car rolls. Aim to '
                'keep bump steer under ~0.1°/10 mm — non-zero toe change with travel '
                'steers the car over bumps and under load.</p>', unsafe_allow_html=True)

# ----------------------------- TAB 2 --------------------------------------- #
with tab2:
    rc_heights = []
    for st_ in sweep:
        kin._tmp = st_
        rc_heights.append(veh.roll_center_height(kin, veh.p.track_front))
    # roll-centre vs travel needs a per-state RC; approximate via IC migration
    rc_static = veh.roll_center_height(kin, veh.p.track_front)

    c1, c2 = st.columns([1.3, 1])
    # load transfer vs lateral g
    gs = np.linspace(0, 1.8, 30)
    fl, fr, rl, rr = [], [], [], []
    for g in gs:
        ld, _ = veh.lateral_load_transfer(g)
        fl.append(ld.fl); fr.append(ld.fr); rl.append(ld.rl); rr.append(ld.rr)
    figL = go.Figure()
    figL.add_trace(go.Scatter(x=gs, y=fr, name="Front outer", line=dict(color=CYAN, width=3)))
    figL.add_trace(go.Scatter(x=gs, y=fl, name="Front inner", line=dict(color=CYAN, width=1.5, dash="dot")))
    figL.add_trace(go.Scatter(x=gs, y=rr, name="Rear outer", line=dict(color=AMBER, width=3)))
    figL.add_trace(go.Scatter(x=gs, y=rl, name="Rear inner", line=dict(color=AMBER, width=1.5, dash="dot")))
    figL.update_layout(**PLOT_LAYOUT, title="Tire vertical load vs lateral g",
                       xaxis_title="lateral acceleration (g)", yaxis_title="vertical load (N)",
                       height=380)
    c1.plotly_chart(figL, use_container_width=True)

    info = veh.lateral_load_transfer(1.2)[1]
    c2.markdown(metric("Roll-centre F", f"{info['rc_front']:.0f}", "mm"), unsafe_allow_html=True)
    c2.markdown(metric("Roll-centre R", f"{info['rc_rear']:.0f}", "mm"), unsafe_allow_html=True)
    c2.markdown(metric("Body roll @1.2g", f"{info['roll_angle']:.2f}", "°",
                       "good" if info['roll_angle'] < 2.5 else "warn"), unsafe_allow_html=True)
    c2.markdown(metric("Front LLT @1.2g", f"{info['ltd_front']:.0f}", "N"), unsafe_allow_html=True)
    c2.markdown(metric("Rear LLT @1.2g", f"{info['ltd_rear']:.0f}", "N"), unsafe_allow_html=True)

    # Roll-centre migration through travel — the honest picture vs a static number.
    mt, mrc = veh.roll_center_migration(kin, veh.p.track_front, -30, 30, 21)
    figM = go.Figure()
    figM.add_trace(go.Scatter(x=mt, y=mrc, mode="lines",
                              line=dict(color="#9b8cff", width=3)))
    figM.update_layout(**PLOT_LAYOUT, title="Roll-centre height migration vs travel",
                       xaxis_title="travel (mm, + bump)", yaxis_title="RC height (mm)",
                       height=300)
    st.plotly_chart(figM, use_container_width=True)
    _rc_swing = max(mrc) - min(mrc) if all(np.isfinite(mrc)) else float("nan")
    st.markdown(f'<p class="hint">Across ±30 mm of travel the front roll centre moves '
                f'{_rc_swing:.0f} mm. Large RC migration means the load-transfer balance '
                f'shifts as the car heaves and rolls — a flatter curve is generally more '
                f'predictable. The load-transfer numbers above use the static RC; this '
                f'plot shows how much that assumption drifts under travel.</p>',
                unsafe_allow_html=True)

    st.markdown(f'<p class="hint">Roll centre sits {rc_static:.0f} mm above ground at '
                'the front. A higher RC reduces body roll but adds jacking and lateral '
                'scrub; most FSAE cars keep it 20–60 mm. The geometric/elastic split of '
                'load transfer is what you tune with bar stiffness and RC height to set '
                'the balance.</p>', unsafe_allow_html=True)
    st.markdown('<p class="hint" style="border-left:2px solid #5a4317;padding-left:10px;">'
                '<b>Steady-state model.</b> These numbers assume sustained cornering at '
                'the given lateral g — they capture the car loaded and balanced mid-corner, '
                'but not transient load: turn-in, trail-braking, kerb strikes, or damper '
                'behaviour. Use it for balance and geometry tuning, not for transient '
                'response.</p>', unsafe_allow_html=True)

# ----------------------------- TAB 3 --------------------------------------- #
with tab3:
    max_g = veh.max_lateral_g()
    bal, uf, ur = veh.balance_index(min(1.2, max_g))
    verdict = ("NEUTRAL", "good") if abs(bal) < 0.03 else \
              (("UNDERSTEER", "warn") if bal > 0 else ("OVERSTEER", "bad"))

    c1, c2, c3 = st.columns(3)
    c1.markdown(metric("Max lateral grip", f"{max_g:.2f}", "g"), unsafe_allow_html=True)
    c2.markdown(metric("Balance", verdict[0], "", verdict[1]), unsafe_allow_html=True)
    c3.markdown(metric("Front/rear util", f"{uf:.2f}/{ur:.2f}", ""), unsafe_allow_html=True)

    gs = np.linspace(0.3, max(max_g + 0.2, 1.0), 30)
    bidx = []
    for g in gs:
        b, _, _ = veh.balance_index(g)
        bidx.append(b)
    figB = go.Figure()
    figB.add_trace(go.Scatter(x=gs, y=bidx, line=dict(color=AMBER, width=3),
                              fill="tozeroy", fillcolor="rgba(255,176,46,.08)"))
    figB.add_hline(y=0, line_color=DIM, line_dash="dash")
    figB.update_layout(**PLOT_LAYOUT,
                       title="Handling balance vs lateral g  (+ understeer / − oversteer)",
                       xaxis_title="lateral acceleration (g)", yaxis_title="balance index",
                       height=380)
    st.plotly_chart(figB, use_container_width=True)
    st.markdown('<p class="hint">Balance index compares how hard each axle is working. '
                'Positive means the front saturates first (push/understeer), negative '
                'means the rear lets go first (oversteer). Shift it with roll-stiffness '
                'distribution, RC heights, and weight distribution in the sidebar.</p>',
                unsafe_allow_html=True)
    st.markdown('<p class="hint" style="border-left:2px solid #5a4317;padding-left:10px;">'
                '<b>Steady-state, simple tire model.</b> Balance is computed at sustained '
                'cornering with a linear load-sensitivity grip model — good for comparing '
                'setups and seeing which way the balance moves, not an absolute prediction '
                'of grip or transient behaviour. A Pacejka tire model is on the roadmap.</p>',
                unsafe_allow_html=True)

# ----------------------------- TAB 4 --------------------------------------- #
with tab4:
    fig3d = go.Figure()

    def seg(p, q, color, w=6, name=None):
        fig3d.add_trace(go.Scatter3d(
            x=[p[0], q[0]], y=[p[1], q[1]], z=[p[2], q[2]],
            mode="lines", line=dict(color=color, width=w),
            name=name, showlegend=name is not None))

    H = hp
    st0 = kin.static
    # wishbones
    seg(H.upper_front_inner, st0.upper_outer, CYAN, name="Upper wishbone")
    seg(H.upper_rear_inner, st0.upper_outer, CYAN)
    seg(H.lower_front_inner, st0.lower_outer, AMBER, name="Lower wishbone")
    seg(H.lower_rear_inner, st0.lower_outer, AMBER)
    seg(st0.lower_outer, st0.upper_outer, "#ffffff", 7, name="Upright / kingpin")
    seg(H.tie_rod_inner, st0.tie_rod_outer, RED, 4, name="Tie rod")
    seg(st0.contact_patch, st0.wheel_center, "#6f7d8c", 3, name="Wheel")

    pts = {k: getattr(H, k) for k, _ in POINTS}
    fig3d.add_trace(go.Scatter3d(
        x=[p[0] for p in pts.values()], y=[p[1] for p in pts.values()],
        z=[p[2] for p in pts.values()], mode="markers",
        marker=dict(size=4, color="#e7ecf1"), name="Hardpoints",
        text=list(pts.keys()), hoverinfo="text+x+y+z"))

    fig3d.update_layout(
        paper_bgcolor="rgba(0,0,0,0)",
        scene=dict(
            xaxis=dict(title="x (rear)", backgroundcolor="#0e1216", gridcolor="#1d242c", color="#8d99a6"),
            yaxis=dict(title="y (right)", backgroundcolor="#0e1216", gridcolor="#1d242c", color="#8d99a6"),
            zaxis=dict(title="z (up)", backgroundcolor="#0e1216", gridcolor="#1d242c", color="#8d99a6"),
            aspectmode="data",
            camera=dict(eye=dict(x=1.6, y=-1.5, z=0.9))),
        font=dict(family="JetBrains Mono", color="#cdd6df", size=10),
        height=560, margin=dict(l=0, r=0, t=10, b=0),
        legend=dict(bgcolor="rgba(0,0,0,0)"))
    st.plotly_chart(fig3d, use_container_width=True)

# ----------------------------- TAB 5 --------------------------------------- #
with tab5:
    st.markdown('<p class="hint">Load the team\'s chassis CAD (STEP or STL) to check '
                'two things before you cut tube: do the inboard pickups land on the '
                'frame (fit), and does the moving linkage clear the chassis through '
                'full travel (clearance). Coordinates must share the suspension origin '
                '— use the offset boxes to align the CAD if needed.</p>',
                unsafe_allow_html=True)

    up = st.file_uploader("Chassis CAD", type=["step", "stp", "stl", "obj", "glb"],
                          label_visibility="collapsed")
    oc1, oc2, oc3, oc4 = st.columns(4)
    off_x = oc1.number_input("offset x (mm)", value=0.0, step=10.0)
    off_y = oc2.number_input("offset y (mm)", value=0.0, step=10.0)
    off_z = oc3.number_input("offset z (mm)", value=0.0, step=10.0)
    cad_scale = oc4.number_input("scale (m→mm = 1000)", value=1.0, step=1.0)

    if up is None:
        st.markdown('<p class="hint" style="padding-top:.5rem;">Waiting for a chassis '
                    'file. Don\'t have the CAD handy? Export it from your assembly as '
                    'STEP — that\'s the most reliable format here.</p>',
                    unsafe_allow_html=True)
    else:
        import tempfile as _tf
        suffix = "." + up.name.split(".")[-1]
        with _tf.NamedTemporaryFile(suffix=suffix, delete=False) as f:
            f.write(up.getbuffer())
            cad_path = f.name
        try:
            with st.spinner("Loading chassis and sweeping the linkage…"):
                mesh = chassis_mod.load_chassis(
                    cad_path, offset=(off_x, off_y, off_z), scale=cad_scale)
                summ = chassis_mod.mesh_summary(mesh)
                fit = chassis_mod.fit_check(hp, mesh, tol_mm=12.0)
                clr = chassis_mod.clearance_check(kin, mesh, warn_mm=8.0)

            verdict = clr["verdict"]
            vcolor = {"CLEAR": ("good", "Linkage clears the chassis"),
                      "TIGHT": ("warn", "Clearance below 8 mm — review before fab"),
                      "COLLISION": ("bad", "Linkage hits the chassis — fix geometry")}[verdict]
            st.markdown(f'<div class="metric" style="margin:.4rem 0;">'
                        f'<span class="k">CLEARANCE VERDICT</span>'
                        f'<span class="v {vcolor[0]}">{verdict}'
                        f'<span class="u"> · {vcolor[1]}</span></span></div>',
                        unsafe_allow_html=True)

            cL, cR = st.columns(2)
            with cL:
                st.markdown("###### Inboard pickup fit")
                for r in fit:
                    tag = "good" if r["mountable"] else "bad"
                    note = "on frame" if r["mountable"] else "off frame"
                    st.markdown(metric(r["label"], f"{r['distance_mm']:.1f}",
                                       f"mm · {note}", tag), unsafe_allow_html=True)
            with cR:
                st.markdown("###### Link clearance (min over travel)")
                order = sorted(clr["per_link"].items(),
                               key=lambda kv: kv[1]["min_clearance_mm"])
                for link, v in order:
                    tag = ("bad" if v["collision"] else
                           "warn" if v["warning"] else "good")
                    label = link.replace("_", " ")
                    st.markdown(metric(label, f"{v['min_clearance_mm']:.1f}", "mm", tag),
                                unsafe_allow_html=True)

            # 3D overlay: chassis mesh + swept linkage
            st.markdown("###### Linkage swept through travel, overlaid on chassis")
            pts, names = chassis_mod.sweep_link_points(kin, -30, 30, 11)
            fig = go.Figure()
            vx, vy, vz = mesh.vertices[:, 0], mesh.vertices[:, 1], mesh.vertices[:, 2]
            i, j, k = mesh.faces[:, 0], mesh.faces[:, 1], mesh.faces[:, 2]
            fig.add_trace(go.Mesh3d(x=vx, y=vy, z=vz, i=i, j=j, k=k,
                          color="#5a6b7a", opacity=0.35, name="Chassis",
                          flatshading=True))
            names_arr = np.array(names)
            palette = {"upper_wishbone_front": CYAN, "upper_wishbone_rear": CYAN,
                       "lower_wishbone_front": AMBER, "lower_wishbone_rear": AMBER,
                       "upright": "#ffffff", "tie_rod": RED, "wheel_spindle": "#9b8cff"}
            for link in np.unique(names_arr):
                m = names_arr == link
                fig.add_trace(go.Scatter3d(
                    x=pts[m, 0], y=pts[m, 1], z=pts[m, 2], mode="markers",
                    marker=dict(size=2, color=palette.get(link, "#888")),
                    name=link.replace("_", " ")))
            fig.update_layout(
                paper_bgcolor="rgba(0,0,0,0)",
                scene=dict(
                    xaxis=dict(title="x", backgroundcolor="#0e1216", gridcolor="#1d242c", color="#8d99a6"),
                    yaxis=dict(title="y", backgroundcolor="#0e1216", gridcolor="#1d242c", color="#8d99a6"),
                    zaxis=dict(title="z", backgroundcolor="#0e1216", gridcolor="#1d242c", color="#8d99a6"),
                    aspectmode="data", camera=dict(eye=dict(x=1.6, y=-1.5, z=0.9))),
                font=dict(family="JetBrains Mono", color="#cdd6df", size=10),
                height=520, margin=dict(l=0, r=0, t=10, b=0),
                legend=dict(bgcolor="rgba(0,0,0,0)", font=dict(size=9)))
            st.plotly_chart(fig, use_container_width=True)

            st.markdown(f'<p class="hint">Chassis mesh: {summ["triangles"]:,} triangles, '
                        f'bounding box {summ["size_mm"][0]:.0f}×{summ["size_mm"][1]:.0f}×'
                        f'{summ["size_mm"][2]:.0f} mm. If the linkage and chassis look '
                        f'misaligned above, adjust the offset boxes so the origins match.</p>',
                        unsafe_allow_html=True)

            if verdict in ("COLLISION", "TIGHT"):
                worst = clr["worst_link"].replace("_", " ")
                if verdict == "COLLISION":
                    sug = (f"Suspension: {worst} hits the chassis through travel "
                           f"(worst {clr['worst_clearance_mm']:.0f} mm). Geometry "
                           f"adjusted / flagged before cutting tube.")
                else:
                    sug = (f"Suspension: {worst} clears the chassis by only "
                           f"{clr['worst_clearance_mm']:.1f} mm at full travel — tight. "
                           f"Reviewed before fabrication.")
                st.markdown('<p class="hint" style="margin-top:.4rem;">⚑ Worth recording '
                            'for handover:</p>', unsafe_allow_html=True)
                edited = st.text_area("Decision note (edit before logging)",
                                      value=sug, height=80, key="autocap_susp")
                if st.button("＋ Log this to handover", key="autocap_susp_btn"):
                    log_decision_now("suspension",
                                     f"Suspension {verdict.lower()} vs chassis",
                                     edited, author="SUSPENSION vs CHASSIS")
                    st.success("Logged to project.json — visible in WEIGHT & HANDOVER.")

            sheet = chassis_mod.manufacturing_sheet(hp, kin)
            st.download_button("⬇ Manufacturing pickup schedule (.csv)", sheet,
                               file_name="kinematik_pickups.csv", mime="text/csv")
        except Exception as e:
            st.error(f"Could not process the chassis file: {e}")
        finally:
            try:
                os.unlink(cad_path)
            except Exception:
                pass

# ----------------------------- TAB 6 --------------------------------------- #
with tab6:
    st.markdown('<p class="hint">Any Elbee subteam: load the shared chassis once as '
                'the reference, then load your part (caliper, radiator, battery box, '
                'wing mount, ECU tray — anything). You get the same collision / tight / '
                'clear verdict suspension gets. <b>We can\'t out-spend USC, so we '
                'out-integrate them</b> — catch the interference here before the first '
                'cut, because rework is the tax for not integrating in CAD.</p>',
                unsafe_allow_html=True)

    tcol1, tcol2 = st.columns(2)
    team_keys = list(integ_mod.TEAMS.keys())
    team = tcol1.selectbox("Your subteam", team_keys,
                           format_func=lambda k: integ_mod.TEAMS[k]["label"])
    part_name = tcol2.text_input("Part name", value="my_part")

    rc1, rc2 = st.columns(2)
    with rc1:
        st.markdown("###### Shared chassis (reference)")
        chassis_up = st.file_uploader("Chassis CAD", type=["step", "stp", "stl", "obj", "glb"],
                                      key="team_chassis", label_visibility="collapsed")
    with rc2:
        st.markdown(f"###### {integ_mod.TEAMS[team]['label']} part")
        part_up = st.file_uploader("Part CAD", type=["step", "stp", "stl", "obj", "glb"],
                                   key="team_part", label_visibility="collapsed")

    st.markdown("###### Position your part in the chassis frame")
    pc = st.columns(7)
    p_ox = pc[0].number_input("x mm", value=0.0, step=10.0, key="p_ox")
    p_oy = pc[1].number_input("y mm", value=0.0, step=10.0, key="p_oy")
    p_oz = pc[2].number_input("z mm", value=0.0, step=10.0, key="p_oz")
    p_rx = pc[3].number_input("rot x°", value=0.0, step=15.0, key="p_rx")
    p_ry = pc[4].number_input("rot y°", value=0.0, step=15.0, key="p_ry")
    p_rz = pc[5].number_input("rot z°", value=0.0, step=15.0, key="p_rz")
    p_scale = pc[6].number_input("scale", value=1.0, step=1.0, key="p_scale")

    if chassis_up is None or part_up is None:
        st.markdown('<p class="hint" style="padding-top:.5rem;">Load both the chassis '
                    'and your part to run the check. Only chassis and suspension have '
                    'CAD right now — as your team produces geometry, this works the same '
                    'way for you. Export STEP from your assembly for best results.</p>',
                    unsafe_allow_html=True)
    else:
        import tempfile as _tf
        def _save(uploaded):
            sfx = "." + uploaded.name.split(".")[-1]
            with _tf.NamedTemporaryFile(suffix=sfx, delete=False) as f:
                f.write(uploaded.getbuffer())
                return f.name
        ch_path = _save(chassis_up)
        pt_path = _save(part_up)
        try:
            with st.spinner("Loading geometry and checking interference…"):
                ref = integ_mod.load_part(ch_path)
                part = integ_mod.load_part(
                    pt_path, offset=(p_ox, p_oy, p_oz), scale=p_scale,
                    rotate_deg=(p_rx, p_ry, p_rz))
                res = integ_mod.interference_check(part, ref, warn_mm=5.0)
                psum = integ_mod.part_summary(part)

            vmap = {"CLEAR": ("good", "Part clears the chassis"),
                    "TIGHT": ("warn", "Under 5 mm — review before fab"),
                    "COLLISION": ("bad", "Part intersects the chassis — reposition")}
            tag, msg = vmap[res["verdict"]]
            st.markdown(f'<div class="metric" style="margin:.4rem 0;">'
                        f'<span class="k">INTERFERENCE VERDICT · {integ_mod.TEAMS[team]["label"].upper()}</span>'
                        f'<span class="v {tag}">{res["verdict"]}'
                        f'<span class="u"> · {msg}</span></span></div>',
                        unsafe_allow_html=True)

            mc1, mc2, mc3 = st.columns(3)
            mc1.markdown(metric("Min clearance", f"{res['min_clearance_mm']:.1f}", "mm", tag),
                         unsafe_allow_html=True)
            mc2.markdown(metric("Part overlap", f"{res['collision_fraction']*100:.0f}", "%",
                                "bad" if res['collision_fraction'] > 0 else "good"),
                         unsafe_allow_html=True)
            mc3.markdown(metric("Part size",
                                f"{psum['size_mm'][0]:.0f}×{psum['size_mm'][1]:.0f}×{psum['size_mm'][2]:.0f}",
                                "mm"), unsafe_allow_html=True)

            if res["verdict"] in ("COLLISION", "TIGHT"):
                tlabel = integ_mod.TEAMS[team]["label"]
                if res["verdict"] == "COLLISION":
                    suggested = (f"{tlabel}: {part_name} intersects the chassis "
                                 f"(overlap {res['collision_fraction']*100:.0f}%, "
                                 f"worst point {res['min_clearance_mm']:.0f} mm inside). "
                                 f"Repositioned / flagged for redesign before fabrication.")
                else:
                    suggested = (f"{tlabel}: {part_name} clears the chassis by only "
                                 f"{res['min_clearance_mm']:.1f} mm — below the 5 mm "
                                 f"margin. Reviewed for clearance before fabrication.")
                st.markdown('<p class="hint" style="margin-top:.4rem;">⚑ This is worth '
                            'recording for handover — log it so next year knows the '
                            'constraint existed, and ping the team that owns what it '
                            'hits:</p>', unsafe_allow_html=True)
                edited = st.text_area("Decision note (edit before logging)",
                                      value=suggested, height=80, key="autocap_team")
                ncol = st.columns([1.6, 1.4, 1.4])
                notify_opts = ["(don't notify)"] + list(integ_mod.TEAMS.keys())
                default_idx = notify_opts.index("chassis") if "chassis" in notify_opts else 0
                notify_team = ncol[0].selectbox(
                    "Notify team", notify_opts, index=default_idx,
                    format_func=lambda k: k if k == "(don't notify)"
                    else integ_mod.TEAMS[k]["label"], key="notify_team")
                notify_urgent = ncol[1].checkbox("Mark urgent", key="notify_urgent",
                                                 value=(res["verdict"] == "COLLISION"))
                note_author = ncol[2].text_input("Your name", key="notify_author")
                if st.button("＋ Log to handover" +
                             (" & notify" if notify_team != "(don't notify)" else ""),
                             key="autocap_team_btn"):
                    _s = project_mod.ProjectStore(PROJECT_PATH)
                    _s.add_decision(project_mod.Decision(
                        team=team, title=f"{part_name} chassis {res['verdict'].lower()}",
                        rationale=edited, author="TEAM FIT", tags="auto-captured"))
                    posted = ""
                    if notify_team != "(don't notify)":
                        _s.add_note(project_mod.Note(
                            from_team=team, to_team=notify_team,
                            message=(f"{part_name} {res['verdict'].lower()} vs chassis "
                                     f"(min {res['min_clearance_mm']:.1f} mm). {edited}"),
                            author=note_author or "TEAM FIT",
                            is_request=True, urgent=notify_urgent))
                        posted = f" · note sent to {integ_mod.TEAMS[notify_team]['label']}"
                    _s.save()
                    st.success(f"Logged to handover{posted}.")

            # Auto-populate the weight budget from this part's CAD volume
            if psum.get("volume_mm3"):
                st.markdown('<p class="hint" style="margin-top:.4rem;">This part is '
                            'watertight, so its mass can be estimated from CAD volume — '
                            'log it straight into the weight budget:</p>',
                            unsafe_allow_html=True)
                awc = st.columns([1.6, 1, 1])
                aw_mat = awc[0].selectbox("Material", list(project_mod.MATERIALS.keys()),
                                          key="awmat")
                aw_qty = awc[1].number_input("Qty", value=1, min_value=1, step=1, key="awqty")
                est = project_mod.estimate_mass_g(psum["volume_mm3"], aw_mat)
                awc[2].markdown(metric("Est. mass each",
                                       f"{est:.0f}" if est else "—", "g"),
                                unsafe_allow_html=True)
                if est and st.button("＋ Add to weight budget", key="aw_btn"):
                    s_ = project_mod.ProjectStore(PROJECT_PATH)
                    s_.add_weight(project_mod.WeightItem(
                        team=team, name=part_name, mass_g=float(est), qty=int(aw_qty),
                        material=aw_mat, source="cad_estimate"))
                    s_.save()
                    st.success(f"Added {part_name} ({est:.0f} g × {aw_qty}) to the budget.")
                elif not est:
                    st.markdown('<p class="hint">Pick a material with a known density to '
                                'estimate mass (or use manual entry in WEIGHT & HANDOVER '
                                'for hollow/lattice parts).</p>', unsafe_allow_html=True)

            fig = go.Figure()
            for mesh, color, name, opac in [(ref, "#5a6b7a", "Chassis", 0.30),
                                            (part, integ_mod.TEAMS[team]["color"], part_name, 0.65)]:
                v = mesh.vertices
                f = mesh.faces
                fig.add_trace(go.Mesh3d(x=v[:, 0], y=v[:, 1], z=v[:, 2],
                              i=f[:, 0], j=f[:, 1], k=f[:, 2],
                              color=color, opacity=opac, name=name, flatshading=True))
            if res["worst_point"] and res["verdict"] != "CLEAR":
                wp = res["worst_point"]
                fig.add_trace(go.Scatter3d(x=[wp[0]], y=[wp[1]], z=[wp[2]],
                              mode="markers", marker=dict(size=6, color=RED),
                              name="Worst point"))
            fig.update_layout(
                paper_bgcolor="rgba(0,0,0,0)",
                scene=dict(
                    xaxis=dict(backgroundcolor="#0e1216", gridcolor="#1d242c", color="#8d99a6"),
                    yaxis=dict(backgroundcolor="#0e1216", gridcolor="#1d242c", color="#8d99a6"),
                    zaxis=dict(backgroundcolor="#0e1216", gridcolor="#1d242c", color="#8d99a6"),
                    aspectmode="data", camera=dict(eye=dict(x=1.6, y=-1.5, z=0.9))),
                font=dict(family="JetBrains Mono", color="#cdd6df", size=10),
                height=520, margin=dict(l=0, r=0, t=10, b=0),
                legend=dict(bgcolor="rgba(0,0,0,0)", font=dict(size=10)))
            st.plotly_chart(fig, use_container_width=True)
            st.markdown('<p class="hint">If the part is in the wrong place relative to '
                        'the chassis, adjust the offset and rotation above until it sits '
                        'where it mounts. The red dot marks the tightest/worst point so '
                        'you know which corner to move.</p>', unsafe_allow_html=True)
        except Exception as e:
            st.error(f"Could not process the files: {e}")
        finally:
            for p in (ch_path, pt_path):
                try:
                    os.unlink(p)
                except Exception:
                    pass

# ----------------------------- TAB 7 --------------------------------------- #
with tab7:
    store = project_mod.ProjectStore(PROJECT_PATH)

    # Surface storage problems instead of silently losing data.
    _degraded = getattr(store.backend, "degraded_reason", None)
    if _degraded:
        st.error(f"⚠ {_degraded}")
    if getattr(store, "load_error", None):
        st.error(f"⚠ {store.load_error}")

    # Tell the user whether their data is persisting or session-only.
    _is_persistent = type(store.backend).__name__ == "SupabaseBackend"
    if _is_persistent:
        st.markdown('<span class="tag good">● persistent storage — data survives '
                    'restarts</span>', unsafe_allow_html=True)
    else:
        st.markdown('<span class="tag warn">● local/session storage — set up Supabase '
                    'for permanent team data (see README)</span>', unsafe_allow_html=True)

    st.markdown('<p class="hint">The lightest reliable car is the advantage money '
                'can\'t buy — and the reasoning behind your design is the thing a team '
                'loses every graduation. This page is the persistent record: it saves '
                'to <code>project.json</code> in the project folder, so commit that file '
                'to the repo and the knowledge survives the handover.</p>',
                unsafe_allow_html=True)

    hcol1, hcol2, hcol3 = st.columns(3)
    store.team_name = hcol1.text_input("Team", value=store.team_name)
    store.season = hcol2.text_input("Season", value=store.season)
    store.target_mass_kg = hcol3.number_input("Target mass (kg)",
                                              value=float(store.target_mass_kg), step=5.0)

    b = store.budget_status()
    bcol = st.columns(3)
    bcol[0].markdown(metric("Current mass", f"{b['total_kg']:.1f}", "kg"), unsafe_allow_html=True)
    bcol[1].markdown(metric("Target", f"{b['target_kg']:.0f}", "kg"), unsafe_allow_html=True)
    bcol[2].markdown(metric("Delta", f"{b['delta_kg']:+.1f}", "kg",
                            "bad" if b["over_budget"] else "good"), unsafe_allow_html=True)

    if store.mass_by_team():
        figW = go.Figure()
        teams = list(store.mass_by_team().keys())
        masses = list(store.mass_by_team().values())
        colors = [integ_mod.TEAMS.get(t, {}).get("color", "#888") for t in teams]
        figW.add_trace(go.Bar(x=masses, y=teams, orientation="h",
                              marker_color=colors))
        figW.update_layout(**PLOT_LAYOUT, title="Mass by subteam (kg)",
                           height=max(220, 40 * len(teams)), xaxis_title="kg",
                           yaxis_title="")
        st.plotly_chart(figW, use_container_width=True)

    st.markdown("###### Log a part's mass")
    wc = st.columns([1.2, 1.4, 0.7, 1, 1.4, 1])
    w_team = wc[0].selectbox("Team", list(integ_mod.TEAMS.keys()),
                             format_func=lambda k: integ_mod.TEAMS[k]["label"], key="w_team")
    w_name = wc[1].text_input("Part name", key="w_name")
    w_qty = wc[2].number_input("Qty", value=1, min_value=1, step=1, key="w_qty")
    w_mass = wc[3].number_input("Mass each (g)", value=0.0, step=10.0, key="w_mass")
    w_mat = wc[4].selectbox("Material", list(project_mod.MATERIALS.keys()), key="w_mat")
    w_src = wc[5].selectbox("Source", ["manual", "cad_estimate"], key="w_src")
    if st.button("+ Add part", use_container_width=False):
        if w_name and w_mass > 0:
            store.add_weight(project_mod.WeightItem(
                team=w_team, name=w_name, mass_g=float(w_mass), qty=int(w_qty),
                material=w_mat, source=w_src))
            store.save()
            st.rerun()
        else:
            st.warning("Enter a part name and a mass above zero.")

    if store.weights:
        st.markdown("###### Logged parts")
        for i, w in enumerate(store.weights):
            cc = st.columns([2, 3, 1, 1.5, 1.5, 0.8])
            cc[0].markdown(f"<span class='tag'>{integ_mod.TEAMS.get(w.team,{}).get('label',w.team)}</span>",
                           unsafe_allow_html=True)
            cc[1].write(w.name)
            cc[2].write(f"×{w.qty}")
            cc[3].write(f"{w.mass_g:.0f} g")
            cc[4].write(f"= {w.total_g/1000:.2f} kg")
            if cc[5].button("✕", key=f"del_{i}"):
                store.remove_weight(i)
                store.save()
                st.rerun()

    st.markdown("---")
    st.markdown("###### Log a design decision")
    st.markdown('<p class="hint">This is the section next year\'s team thanks you for. '
                'Write down <i>why</i>, not just what — the reasoning is what gets lost.</p>',
                unsafe_allow_html=True)

    # ---- Quick-add: one-tap templates to kill logging friction ----------
    QUICK_TEMPLATES = {
        "⚙ Geometry change": ("Geometry change", "changed-geometry",
                              "Changed [what] from [old] to [new] because [reason]. "
                              "Trade-off: [what it costs]."),
        "🔧 Material / part choice": ("Material choice", "material",
                              "Chose [material/part] for [component] because [reason]. "
                              "Considered [alternative] but [why not]."),
        "⚠ Interference found": ("Interference found", "interference",
                              "[Part] interferes with [what] at [condition]. "
                              "Resolved by [action] / flagged for [who]."),
        "🧪 Test result": ("Test result", "test",
                              "Tested [what]. Result: [outcome]. "
                              "Means we should [implication]."),
        "❌ Didn't work": ("Didn't work", "rejected",
                              "Tried [approach] for [goal]. Didn't work because [reason]. "
                              "Avoid repeating — instead [what to do]."),
    }
    st.markdown('<p class="hint" style="margin-bottom:.2rem;">Quick start — tap a '
                'template, then just fill in the brackets:</p>', unsafe_allow_html=True)
    qcols = st.columns(len(QUICK_TEMPLATES))
    for i, (label, (title, tag, body)) in enumerate(QUICK_TEMPLATES.items()):
        if qcols[i].button(label, key=f"qt_{i}", use_container_width=True):
            # Seed the widget keys directly, before the widgets are created below.
            st.session_state["d_title"] = title
            st.session_state["d_tags"] = tag
            st.session_state["d_rationale"] = body
            st.rerun()

    dc = st.columns([1.2, 2, 1.2])
    d_team = dc[0].selectbox("Team", list(integ_mod.TEAMS.keys()),
                             format_func=lambda k: integ_mod.TEAMS[k]["label"], key="d_team")
    d_title = dc[1].text_input("Decision", key="d_title")
    d_author = dc[2].text_input("Author", key="d_author")
    d_rationale = st.text_area("Rationale — why this choice, what were the trade-offs",
                               key="d_rationale", height=90)
    tc = st.columns([1.4, 1.4])
    d_part = tc[0].text_input("Part / system (e.g. front upright, radiator)", key="d_part",
                              placeholder="what this decision is about")
    d_tags = tc[1].text_input("Tags (comma-separated)", key="d_tags",
                              placeholder="roll-centre, front, packaging…")
    if st.button("+ Log decision"):
        if d_title and d_rationale:
            store.add_decision(project_mod.Decision(
                team=d_team, title=d_title, rationale=d_rationale, author=d_author,
                tags=d_tags, part=d_part))
            store.save()
            for k in ("d_title", "d_tags", "d_rationale", "d_part"):
                st.session_state.pop(k, None)
            st.rerun()
        else:
            st.warning("Enter a decision title and rationale.")

    if store.decisions:
        st.markdown("###### Search the decision log")
        sc = st.columns([2.2, 1.2, 1.2, 1.2])
        d_query = sc[0].text_input("Search", key="dec_search",
                                   placeholder="search title, rationale, author, tags, part…",
                                   label_visibility="collapsed")
        team_opts = ["all teams"] + list(integ_mod.TEAMS.keys())
        d_fteam = sc[1].selectbox("Team", team_opts, key="dec_fteam",
                                  format_func=lambda k: "All teams" if k == "all teams"
                                  else integ_mod.TEAMS[k]["label"], label_visibility="collapsed")
        tag_opts = ["all tags"] + store.all_decision_tags()
        d_ftag = sc[2].selectbox("Tag", tag_opts, key="dec_ftag",
                                 format_func=lambda k: "All tags" if k == "all tags" else k,
                                 label_visibility="collapsed")
        part_opts = ["all parts"] + store.all_decision_parts()
        d_fpart = sc[3].selectbox("Part", part_opts, key="dec_fpart",
                                  format_func=lambda k: "All parts" if k == "all parts" else k,
                                  label_visibility="collapsed")

        results = store.search_decisions(
            query=d_query,
            team=None if d_fteam == "all teams" else d_fteam,
            tag=None if d_ftag == "all tags" else d_ftag,
            part=None if d_fpart == "all parts" else d_fpart)

        st.markdown(f"<p class='hint'>{len(results)} of {len(store.decisions)} "
                    f"decisions</p>", unsafe_allow_html=True)

        for d in results:
            meta = f"{integ_mod.TEAMS.get(d.team,{}).get('label',d.team)} · {d.date}"
            if d.author:
                meta += f" · {d.author}"
            dpart = getattr(d, "part", "") or ""
            if dpart:
                meta += f" · ⛭ {dpart}"
            auto = "<span class='tag good' style='margin-left:6px;'>auto-captured</span>" \
                if "auto" in (d.tags or "") else ""
            # render user tags as chips (excluding the internal auto-captured marker)
            chips = ""
            for t in (d.tags or "").split(","):
                t = t.strip()
                if t and t != "auto-captured":
                    chips += f"<span class='tag' style='margin-right:4px;'>{t}</span>"
            chip_row = f"<div style='margin-top:.3rem;'>{chips}</div>" if chips else ""
            st.markdown(f"<div class='card' style='margin:.3rem 0;'>"
                        f"<b>{d.title}</b>{auto}<br><span class='hint'>{meta}</span><br>"
                        f"<span style='font-size:.9rem;'>{d.rationale}</span>{chip_row}</div>",
                        unsafe_allow_html=True)
        if not results:
            st.markdown("<p class='hint'>No decisions match — try a broader search or "
                        "clear the filters.</p>", unsafe_allow_html=True)

    st.markdown("---")
    st.markdown("###### Export handover report")
    geo = {
        "static_camber_deg": s.camber, "static_toe_deg": s.toe,
        "caster_deg": s.caster, "kpi_deg": s.kpi,
        "scrub_radius_mm": s.scrub_radius,
        "roll_centre_front_mm": mid["rc_front"], "roll_centre_rear_mm": mid["rc_rear"],
        "max_lateral_g": veh.max_lateral_g(),
    }
    md = project_mod.build_handover_markdown(store, geometry=geo)
    ec = st.columns(3)
    ec[0].download_button("⬇ Handover (.md)", md, file_name="elbee_handover.md",
                          mime="text/markdown", use_container_width=True)
    ec[1].download_button("⬇ Project data (.json)", store.as_json(),
                          file_name="project.json", mime="application/json",
                          use_container_width=True)
    try:
        pdf_path = os.path.join(tempfile.gettempdir(), "elbee_handover.pdf")
        project_mod.render_pdf(md, pdf_path)
        with open(pdf_path, "rb") as f:
            ec[2].download_button("⬇ Handover (.pdf)", f.read(),
                                  file_name="elbee_handover.pdf",
                                  mime="application/pdf", use_container_width=True)
    except Exception as e:
        ec[2].markdown(f"<p class='hint'>PDF unavailable: {e}</p>", unsafe_allow_html=True)

# ----------------------------- TAB 8 --------------------------------------- #
with tab8:
    nstore = project_mod.ProjectStore(PROJECT_PATH)

    st.markdown('<p class="hint">Cross-team notes between leads — for keeping '
                'interfaces from going stale. Unlike Discord, a note here is addressed '
                'to a team, has an open/resolved status, and lives next to the work in '
                '<code>project.json</code>. <b>The way you out-integrate a richer team '
                'is by never letting two finished parts surprise each other.</b></p>',
                unsafe_allow_html=True)

    team_keys = list(integ_mod.TEAMS.keys())
    # Open-item summary across all teams
    open_counts = {t: nstore.open_note_count(t) for t in team_keys}
    open_counts = {t: c for t, c in open_counts.items() if c > 0}
    if open_counts:
        chips = " ".join(
            f"<span class='tag warn'>{integ_mod.TEAMS[t]['label']}: {c} open</span>"
            for t, c in open_counts.items())
        st.markdown(f"<div style='margin:.3rem 0 .6rem;'>{chips}</div>",
                    unsafe_allow_html=True)

    st.markdown("###### Post a note")
    pc = st.columns([1.2, 1.2, 1.2])
    n_from = pc[0].selectbox("From", team_keys,
                             format_func=lambda k: integ_mod.TEAMS[k]["label"], key="n_from")
    n_to = pc[1].selectbox("To", ["all"] + team_keys,
                           format_func=lambda k: "All teams" if k == "all"
                           else integ_mod.TEAMS[k]["label"], key="n_to")
    n_author = pc[2].text_input("Your name", key="n_author")
    n_msg = st.text_area("Note", key="n_msg", height=80,
                         placeholder="e.g. Upright moved 8 mm inboard — recheck caliper clearance")
    fc = st.columns([1, 1, 3])
    n_req = fc[0].checkbox("Requests action", key="n_req")
    n_urg = fc[1].checkbox("Urgent", key="n_urg")
    if st.button("Post note", key="n_post"):
        if n_msg.strip():
            nstore.add_note(project_mod.Note(
                from_team=n_from, to_team=n_to, message=n_msg.strip(),
                author=n_author, is_request=n_req, urgent=n_urg))
            nstore.save()
            st.rerun()
        else:
            st.warning("Write a note before posting.")

    st.markdown("---")
    fcol1, fcol2 = st.columns([1.5, 3])
    view_team = fcol1.selectbox("Show notes for", ["all teams"] + team_keys,
                                format_func=lambda k: "All notes" if k == "all teams"
                                else integ_mod.TEAMS[k]["label"], key="n_view")
    show_resolved = fcol2.checkbox("Show resolved", value=False, key="n_showres")

    if view_team == "all teams":
        notes = sorted(nstore.notes, key=lambda n: n.ts, reverse=True)
    else:
        notes = nstore.notes_for(view_team)
    if not show_resolved:
        notes = [n for n in notes if n.status == "open"]

    if not notes:
        st.markdown('<p class="hint">No notes yet. When a check in TEAM FIT or '
                    'SUSPENSION vs CHASSIS affects another team, post a note here so '
                    'their lead sees it the next time they open the tool.</p>',
                    unsafe_allow_html=True)
    else:
        for n in notes:
            fclr = integ_mod.TEAMS.get(n.from_team, {}).get("color", "#888")
            tclr = integ_mod.TEAMS.get(n.to_team, {}).get("color", "#888") \
                if n.to_team != "all" else "#8d99a6"
            to_label = "All teams" if n.to_team == "all" \
                else integ_mod.TEAMS.get(n.to_team, {}).get("label", n.to_team)
            from_label = integ_mod.TEAMS.get(n.from_team, {}).get("label", n.from_team)
            badges = ""
            if n.urgent:
                badges += "<span class='tag bad'>urgent</span> "
            if n.is_request:
                badges += "<span class='tag warn'>action requested</span> "
            if n.status == "resolved":
                badges += "<span class='tag good'>resolved</span> "
            meta = f"{from_label} → {to_label} · {n.ts.replace('T',' ')[:16]}"
            if n.author:
                meta += f" · {n.author}"
            st.markdown(
                f"<div class='card' style='margin:.3rem 0; border-left:3px solid {fclr};'>"
                f"<div style='margin-bottom:.2rem;'>{badges}</div>"
                f"<span style='font-size:.95rem;'>{n.message}</span><br>"
                f"<span class='hint'>{meta}</span></div>", unsafe_allow_html=True)
            bc = st.columns([1, 6])
            if n.status == "open":
                if bc[0].button("Mark resolved", key=f"res_{n.id}"):
                    nstore.resolve_note(n.id)
                    nstore.save()
                    st.rerun()
            else:
                if bc[0].button("Reopen", key=f"reo_{n.id}"):
                    nstore.reopen_note(n.id)
                    nstore.save()
                    st.rerun()

# --------------------------------------------------------------------------- #
#  Save / Load project — one file captures the whole session
# --------------------------------------------------------------------------- #
st.markdown("---")
st.markdown("#### Save / load your work")
st.markdown('<p class="hint">One file holds your whole session — geometry, vehicle '
            'setup, and the handover log (decisions, notes, weights). Save it to keep '
            'your progress or hand it to a teammate; load it to pick up exactly where '
            'you left off.</p>', unsafe_allow_html=True)

# Build the unified project bundle.
_store_for_save = project_mod.ProjectStore(PROJECT_PATH)
project_bundle = {
    "kinematik_version": "1.0",
    "saved": _datetime.datetime.now().isoformat(timespec="seconds"),
    "hardpoints": hp_dict,
    "vehicle": st.session_state.vp,
    "handover": json.loads(_store_for_save.as_json()),
}

sc1, sc2, sc3 = st.columns([1, 1, 1])
sc1.download_button("💾 Save project (.json)", json.dumps(project_bundle, indent=2),
                    file_name="kinematik_project.json", mime="application/json",
                    use_container_width=True)

# CSV of the sweep (tabular data — handy for report plots / Excel)
import io
buf = io.StringIO()
buf.write("travel_mm,camber_deg,toe_deg,caster_deg,kpi_deg,scrub_mm\n")
for st_ in sweep:
    buf.write(f"{st_.travel:.2f},{st_.camber:.4f},{st_.toe:.4f},"
              f"{st_.caster:.4f},{st_.kpi:.4f},{st_.scrub_radius:.3f}\n")
sc2.download_button("⬇ Sweep data (.csv)", buf.getvalue(),
                    file_name="kinematik_sweep.csv", mime="text/csv",
                    use_container_width=True)

with sc3:
    loaded = st.file_uploader("📂 Load project (.json)", type=["json"],
                              key="load_project", label_visibility="visible")
    if loaded is not None:
        try:
            data = json.load(loaded)
            if "hardpoints" in data:
                st.session_state.hp = data["hardpoints"]
            if "vehicle" in data:
                st.session_state.vp = data["vehicle"]
            # restore handover data into the store
            if "handover" in data:
                _s = project_mod.ProjectStore(PROJECT_PATH)
                _s._apply(data["handover"])
                _s.save()
            st.success("Project loaded — geometry, vehicle, and handover restored.")
            if st.button("Apply loaded project"):
                st.rerun()
        except Exception as e:
            st.error(f"Couldn't read that project file: {e}")

st.markdown('<p class="hint" style="padding-top:.4rem;">Open source · MIT. Fork it, '
            'validate against your OptimumK model, send a PR. '
            '<i>Tip: on the hosted app, save your project before closing the tab — '
            'geometry tweaks aren\'t auto-saved the way the handover log is.</i></p>',
            unsafe_allow_html=True)
