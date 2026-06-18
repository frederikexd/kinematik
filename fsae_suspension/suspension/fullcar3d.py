# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Full-vehicle 3D assembly renderer (pure Python + Plotly).
# ============================================================================

"""
Build a single Plotly 3D figure of the WHOLE car from the data the team has
already filled out in KinematiK: one corner's hardpoint geometry plus the vehicle
params (wheelbase, front/rear track, CG height, mass). No CAD, no extra deps —
just numpy + plotly, so it drops straight into the existing Streamlit app and runs
anywhere the app already runs.

How the four corners are generated
----------------------------------
KinematiK stores ONE corner of suspension geometry (the right-hand corner, +y) and
reuses it for both axles (`front_kin = rear_kin = kin`), exactly as the rest of the
app does. To draw a full car we place that corner at all four wheel stations:

    * RIGHT corners keep the geometry as authored (+y).
    * LEFT corners mirror it across the car centreline (y -> -y).
    * The whole corner is scaled laterally so its contact patch lands on the
      half-track of its axle (front track vs rear track), then shifted fore/aft so
      the front axle sits at x = +wheelbase/2 and the rear at x = -wheelbase/2
      (SAE x points rearward, so "front" is +x in vehicle layout terms; we keep
      the kinematics' own x sign and just translate).

This is a faithful *layout* of the geometry the team typed in — same assumption the
solver already makes — not a claim that the real front and rear uprights are
identical. When a team later carries distinct front/rear hardpoints, this function
takes them as two arguments and the assumption goes away.

Everything stays in the kinematics frame: millimetres, SAE axes (x rear, y right,
z up). The figure is self-contained and themeable via the COLORS dict so it matches
the app's dark instrument styling.
"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from .kinematics import Hardpoints, SuspensionKinematics

# Palette matched to the app's existing 3D tab.
COLORS = dict(
    upper="#37e0d0",      # cyan  — upper wishbone
    lower="#ffb02e",      # amber — lower wishbone
    upright="#ffffff",    # white — upright / kingpin
    tie="#ff5a52",        # red   — tie rod
    push="#9b8cff",       # violet— pushrod
    rocker="#5ad17a",     # green — rocker
    spring="#ff9f43",     # orange— spring/damper
    wheel="#6f7d8c",      # steel — wheel spokes
    tire="#23292f",       # dark  — tire surface
    tire_edge="#3a434c",
    chassis="#7d8893",    # steel-grey representative frame
    point="#e7ecf1",
    floor="#0c1014",
)


# --------------------------------------------------------------------------- #
#  Geometry transforms
# --------------------------------------------------------------------------- #
def _corner_transform(p, *, mirror_y: bool, lateral_scale: float, x_shift: float,
                      y_center_ref: float):
    """Map a reference-corner point into a specific wheel-station frame.

    p             : (3,) point in the authored corner frame (mm).
    mirror_y      : True for left-hand corners (reflect across centreline).
    lateral_scale : multiply lateral offset so the patch lands on this axle's
                    half-track (front vs rear differ).
    x_shift       : translate fore/aft to the axle's longitudinal station.
    y_center_ref  : the car-centreline y of the authored corner (≈0); lateral
                    distances are measured from here before scaling.
    """
    if p is None:
        return None
    q = np.array(p, float).copy()
    # lateral: scale distance from centreline, then optionally mirror
    dy = (q[1] - y_center_ref) * lateral_scale
    q[1] = y_center_ref + (-dy if mirror_y else dy)
    # longitudinal station
    q[0] = q[0] + x_shift
    return q


def _solved_corner_points(hp: Hardpoints):
    """Solve the linkage once and return the static moving points + fixed pickups
    as a flat dict of name -> (3,) array, in the authored corner frame."""
    kin = SuspensionKinematics(hp)
    s = kin.static
    pts = dict(
        upper_front_inner=np.array(hp.upper_front_inner, float),
        upper_rear_inner=np.array(hp.upper_rear_inner, float),
        lower_front_inner=np.array(hp.lower_front_inner, float),
        lower_rear_inner=np.array(hp.lower_rear_inner, float),
        tie_rod_inner=np.array(hp.tie_rod_inner, float),
        upper_outer=np.array(s.upper_outer, float),
        lower_outer=np.array(s.lower_outer, float),
        tie_rod_outer=np.array(s.tie_rod_outer, float),
        wheel_center=np.array(s.wheel_center, float),
        contact_patch=np.array(s.contact_patch, float),
    )
    if hp.has_rocker():
        for k in ("rocker_pivot", "rocker_pushrod", "rocker_spring", "spring_inner"):
            v = getattr(hp, k)
            if v is not None:
                pts[k] = np.array(v, float)
        po = s.pushrod_outer if s.pushrod_outer is not None else hp.pushrod_outer
        if po is not None:
            pts["pushrod_outer"] = np.array(po, float)
    return pts, s


# --------------------------------------------------------------------------- #
#  Mesh builders
# --------------------------------------------------------------------------- #
def _tire_mesh(center, axis, radius, width, n=28):
    """Triangulated cylinder (open) approximating a tire, centred at `center`
    with its spin axis along `axis` (a 3-vector, need not be unit)."""
    axis = np.asarray(axis, float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    # two orthonormal vectors spanning the wheel plane
    ref = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, ref); u /= (np.linalg.norm(u) + 1e-12)
    v = np.cross(axis, u)
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    rim = np.array([radius * (np.cos(t) * u + np.sin(t) * v) for t in th])
    c = np.asarray(center, float)
    inner = c - axis * (width / 2.0) + rim
    outer = c + axis * (width / 2.0) + rim
    verts = np.vstack([inner, outer])
    I, J, K = [], [], []
    for a in range(n):
        b = (a + 1) % n
        ai, bi, ao, bo = a, b, a + n, b + n
        I += [ai, ai]; J += [bi, ao]; K += [ao, bo]  # two tris per quad
    return verts, np.array(I), np.array(J), np.array(K)


def _chassis_wireframe(half_track_f, half_track_r, x_f, x_r,
                       z_lo, z_hi):
    """A simple representative space-frame as line segments: lower & upper
    rectangles between the four pickup zones plus connecting verticals/diagonals.
    Purely illustrative scaffolding so the suspension reads as a car, not a claim
    about the actual frame (load it from CAD in the INTEGRATION tab for that)."""
    # footprint slightly inboard of the wheels
    yf, yr = half_track_f * 0.62, half_track_r * 0.62
    # eight corner nodes of a tapered box
    nodes = {
        "flb": (x_f,  yf, z_lo), "frb": (x_f, -yf, z_lo),
        "rlb": (x_r,  yr, z_lo), "rrb": (x_r, -yr, z_lo),
        "flt": (x_f,  yf, z_hi), "frt": (x_f, -yf, z_hi),
        "rlt": (x_r,  yr, z_hi), "rrt": (x_r, -yr, z_hi),
    }
    edges = [
        # lower perimeter
        ("flb", "frb"), ("frb", "rrb"), ("rrb", "rlb"), ("rlb", "flb"),
        # upper perimeter
        ("flt", "frt"), ("frt", "rrt"), ("rrt", "rlt"), ("rlt", "flt"),
        # verticals
        ("flb", "flt"), ("frb", "frt"), ("rlb", "rlt"), ("rrb", "rrt"),
        # main hoop diagonals (a hint of the roll structure)
        ("rlb", "rlt"), ("rrb", "rrt"), ("rlt", "rrt"),
        # side diagonals
        ("flb", "rlt"), ("frb", "rrt"),
    ]
    segs = [(np.array(nodes[a]), np.array(nodes[b])) for a, b in edges]
    return segs


# --------------------------------------------------------------------------- #
#  Main entry point
# --------------------------------------------------------------------------- #
def build_full_car_figure(
    hp_front: Hardpoints,
    vp,
    hp_rear: Hardpoints | None = None,
    *,
    show_chassis: bool = True,
    show_tires: bool = True,
    show_floor: bool = True,
    tire_width_mm: float = 180.0,
    height: int = 640,
) -> go.Figure:
    """Assemble a full-vehicle 3D Plotly figure from corner geometry + vehicle params.

    hp_front : Hardpoints for the front corner (the authored corner if you only have one).
    vp       : a VehicleParams (needs .wheelbase, .track_front, .track_rear; .cg_height
               and .mass used for the CG marker if present).
    hp_rear  : optional distinct rear-corner Hardpoints; defaults to hp_front (matching
               the app's single-corner assumption).
    """
    if hp_rear is None:
        hp_rear = hp_front

    wb = float(getattr(vp, "wheelbase", 1550.0))
    tf = float(getattr(vp, "track_front", 1200.0))
    tr = float(getattr(vp, "track_rear", 1180.0))

    fpts, fstate = _solved_corner_points(hp_front)
    rpts, rstate = _solved_corner_points(hp_rear)

    # The authored corner's own track = 2 * |contact_patch.y - centreline|.
    y_center = 0.0
    cp_f = fpts["contact_patch"]; cp_r = rpts["contact_patch"]
    authored_half_f = abs(cp_f[1] - y_center) or 1.0
    authored_half_r = abs(cp_r[1] - y_center) or 1.0
    scale_f = (tf / 2.0) / authored_half_f
    scale_r = (tr / 2.0) / authored_half_r

    # Longitudinal stations. Keep the authored corner's x as the local datum and
    # shift each axle to ±wheelbase/2 about the car origin.
    x_front = +wb / 2.0
    x_rear = -wb / 2.0

    fig = go.Figure()

    def seg(p, q, color, w=5, name=None, group=None):
        fig.add_trace(go.Scatter3d(
            x=[p[0], q[0]], y=[p[1], q[1]], z=[p[2], q[2]],
            mode="lines", line=dict(color=color, width=w),
            name=name, legendgroup=group, showlegend=name is not None,
            hoverinfo="skip"))

    legend_done = set()

    def corner_name(base):
        # show each link type in the legend exactly once
        if base in legend_done:
            return None
        legend_done.add(base)
        return base

    # ---- draw the four corners ------------------------------------------- #
    stations = [
        ("front", fpts, fstate, scale_f, x_front, False),  # front right
        ("front", fpts, fstate, scale_f, x_front, True),   # front left
        ("rear",  rpts, rstate, scale_r, x_rear,  False),  # rear right
        ("rear",  rpts, rstate, scale_r, x_rear,  True),   # rear left
    ]

    for axle, pts, state, lat_scale, x_shift, mirror in stations:
        T = lambda name: _corner_transform(
            pts.get(name), mirror_y=mirror, lateral_scale=lat_scale,
            x_shift=x_shift, y_center_ref=y_center)

        ufi, uri = T("upper_front_inner"), T("upper_rear_inner")
        lfi, lri = T("lower_front_inner"), T("lower_rear_inner")
        uo, lo = T("upper_outer"), T("lower_outer")
        tri, tro = T("tie_rod_inner"), T("tie_rod_outer")
        wc, cp = T("wheel_center"), T("contact_patch")

        seg(ufi, uo, COLORS["upper"], 5, corner_name("Upper wishbone"), "upper")
        seg(uri, uo, COLORS["upper"], 5, None, "upper")
        seg(lfi, lo, COLORS["lower"], 5, corner_name("Lower wishbone"), "lower")
        seg(lri, lo, COLORS["lower"], 5, None, "lower")
        seg(lo, uo, COLORS["upright"], 6, corner_name("Upright / kingpin"), "upright")
        seg(tri, tro, COLORS["tie"], 4, corner_name("Tie rod"), "tie")
        seg(cp, wc, COLORS["wheel"], 3, corner_name("Wheel hub"), "wheel")

        # pushrod / rocker / spring if defined
        po = T("pushrod_outer"); rpv = T("rocker_pivot")
        rpu = T("rocker_pushrod"); rsp = T("rocker_spring"); spi = T("spring_inner")
        if po is not None and rpu is not None:
            seg(po, rpu, COLORS["push"], 4, corner_name("Pushrod"), "push")
        if rpv is not None and rpu is not None and rsp is not None:
            seg(rpv, rpu, COLORS["rocker"], 4, corner_name("Rocker"), "rocker")
            seg(rpv, rsp, COLORS["rocker"], 4, None, "rocker")
        if rsp is not None and spi is not None:
            seg(rsp, spi, COLORS["spring"], 6, corner_name("Spring/damper"), "spring")

        # tire mesh: spin axis is the line through the two ball joints' wheel-plane
        # normal; a good proxy is the wheel-centre-to-contact-patch perpendicular.
        # Use the kingpin-independent estimate: axis ≈ direction from contact patch
        # projected, but simplest robust proxy = lateral (y) tilted by camber.
        if show_tires and wc is not None:
            # build axis from the upright: perpendicular to (uo-lo) within reason,
            # but the cleanest spin axis is (wheel_center - contact_patch) rotated.
            # Use lateral axis tilted by static camber for a clean upright tire.
            cam = np.deg2rad(getattr(hp_front if axle == "front" else hp_rear,
                                     "static_camber", -1.5))
            sign = -1.0 if mirror else 1.0
            # spin axis mostly along y, leaning by camber about x
            axis = np.array([0.0, sign * np.cos(cam), np.sin(cam)])
            radius = abs(wc[2] - cp[2]) or 228.0
            verts, I, J, Kk = _tire_mesh(wc, axis, radius, tire_width_mm)
            fig.add_trace(go.Mesh3d(
                x=verts[:, 0], y=verts[:, 1], z=verts[:, 2],
                i=I, j=J, k=Kk, color=COLORS["tire"], opacity=0.55,
                flatshading=True, name="Tire",
                showlegend=("Tire" not in legend_done),
                hoverinfo="skip"))
            legend_done.add("Tire")

        # hardpoint markers for this corner
        mk = [v for v in (ufi, uri, lfi, lri, uo, lo, tri, tro) if v is not None]
        fig.add_trace(go.Scatter3d(
            x=[p[0] for p in mk], y=[p[1] for p in mk], z=[p[2] for p in mk],
            mode="markers", marker=dict(size=3, color=COLORS["point"]),
            showlegend=False, hoverinfo="skip"))

    # ---- chassis wireframe ----------------------------------------------- #
    if show_chassis:
        z_all = []
        for pts in (fpts, rpts):
            for v in pts.values():
                z_all.append(v[2])
        z_lo = min(z_all) * 0.9
        z_hi = max(z_all) * 1.02
        segs = _chassis_wireframe(tf / 2.0, tr / 2.0, x_front, x_rear, z_lo, z_hi)
        for a, b in segs:
            seg(a, b, COLORS["chassis"], 3,
                corner_name("Chassis (representative)"), "chassis")

    # ---- CG marker ------------------------------------------------------- #
    cg_h = float(getattr(vp, "cg_height", 0.0) or 0.0)
    wdist = float(getattr(vp, "weight_dist_front", 0.5) or 0.5)
    if cg_h > 0:
        # weight_dist_front fraction on front axle -> x position between axles
        cg_x = x_rear + wdist * (x_front - x_rear)
        fig.add_trace(go.Scatter3d(
            x=[cg_x], y=[0.0], z=[cg_h], mode="markers+text",
            marker=dict(size=7, color="#ffd166", symbol="diamond"),
            text=["CG"], textposition="top center",
            textfont=dict(color="#ffd166", size=11),
            name="Centre of gravity", hoverinfo="text"))

    # ---- ground plane ---------------------------------------------------- #
    if show_floor:
        pad = max(tf, tr) * 0.7
        xs = [x_rear - pad, x_front + pad]
        ys = [-max(tf, tr) / 2 - pad, max(tf, tr) / 2 + pad]
        gx, gy = np.meshgrid(xs, ys)
        gz = np.zeros_like(gx)
        fig.add_trace(go.Surface(
            x=gx, y=gy, z=gz, showscale=False, opacity=0.25,
            colorscale=[[0, COLORS["floor"]], [1, COLORS["floor"]]],
            hoverinfo="skip", name="Ground", showlegend=False))

    fig.update_layout(
        paper_bgcolor="rgba(0,0,0,0)",
        scene=dict(
            xaxis=dict(title="x (rear ←→ front)", backgroundcolor="#0e1216",
                       gridcolor="#1d242c", color="#8d99a6"),
            yaxis=dict(title="y (right)", backgroundcolor="#0e1216",
                       gridcolor="#1d242c", color="#8d99a6"),
            zaxis=dict(title="z (up)", backgroundcolor="#0e1216",
                       gridcolor="#1d242c", color="#8d99a6"),
            aspectmode="data",
            camera=dict(eye=dict(x=1.7, y=-1.6, z=1.0))),
        font=dict(family="JetBrains Mono", color="#cdd6df", size=10),
        height=height, margin=dict(l=0, r=0, t=10, b=0),
        legend=dict(bgcolor="rgba(0,0,0,0)", font=dict(size=10)))
    return fig
