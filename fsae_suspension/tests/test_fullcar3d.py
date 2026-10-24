"""The full-car 3D model: geometry primitives, sizing laws, the ledger read-outs and the figure.

The figure is a drawing, not an analysis, so these tests check what a drawing can get
wrong: the mesh primitives have the size and shape they claim (a box's volume, a
cylinder's radius, a NACA section's thickness), every body sits where its documented
sizing law says it should, overrides and suppression reach all four corners, the
heat map and CG marker use the same numbers as the ledger, and the helpers that read
a figure back (parts, corners, footprints) agree with what was drawn.
"""
from types import SimpleNamespace as NS

import numpy as np
import plotly.graph_objects as go
import pytest

from suspension import fullcar3d as fc
from suspension.interfaces import IntegrationLedger, SubsystemInterface
from suspension.kinematics import Hardpoints

pytestmark = pytest.mark.filterwarnings("ignore")

R_TIRE = 228.0                       # wheel-centre height of Hardpoints.default()


def _vp(**kw):
    base = dict(wheelbase=1550.0, track_front=1200.0, track_rear=1180.0,
                spring_rate_front=35.0, cg_height=280.0, weight_dist_front=0.46, mass=0.0)
    base.update(kw)
    return NS(**base)


def _ledger(**ifaces):
    led = IntegrationLedger()
    for name, fields in ifaces.items():
        led.set(SubsystemInterface(name=name.replace("_", "-"), **fields))
    return led


def _meshes(fig, name):
    return [t for t in fig.data if t.type == "mesh3d" and t.name == name]


def _xyz(t):
    return np.column_stack([np.asarray(t.x, float), np.asarray(t.y, float),
                            np.asarray(t.z, float)])


def _closed_volume(v, i, j, k):
    """Volume of a closed triangle mesh by the divergence theorem."""
    a, b, c = v[i], v[j], v[k]
    return abs(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0


def _edges_shared_twice(i, j, k):
    from collections import Counter
    cnt = Counter()
    for a, b, c in zip(i, j, k):
        for e in ((a, b), (b, c), (c, a)):
            cnt[tuple(sorted(e))] += 1
    return set(cnt.values()) == {2}


# --------------------------------------------------------------------------- primitives
def test_box_is_a_closed_box_of_the_given_size():
    v, i, j, k = fc._box(10, -20, 30, 400, 200, 100)
    assert v.min(axis=0) == pytest.approx([-190, -120, -20])
    assert v.max(axis=0) == pytest.approx([210, 80, 80])
    assert len(i) == 12 and _edges_shared_twice(i, j, k)
    assert _closed_volume(v, i, j, k) == pytest.approx(400 * 200 * 100)


@pytest.mark.parametrize("axis", [(0, 0, 1), (1, 0, 0), (0.3, 1.0, 0.2)])
def test_cylinder_radius_length_and_volume(axis):
    n, r, L = 24, 50.0, 120.0
    c = np.array([5.0, -7.0, 11.0])
    v, i, j, k = fc._cylinder(c, axis, r, L, n=n)
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    rim = v[:2 * n] - c
    along = rim @ a
    radial = np.linalg.norm(rim - np.outer(along, a), axis=1)
    assert radial == pytest.approx(np.full(2 * n, r))
    assert sorted(set(np.round(along, 9))) == pytest.approx([-L / 2, L / 2])
    assert _edges_shared_twice(i, j, k)
    polygon = 0.5 * n * r * r * np.sin(2 * np.pi / n)
    assert _closed_volume(v, i, j, k) == pytest.approx(polygon * L)
    open_v, oi, _, _ = fc._cylinder(c, axis, r, L, n=n, cap=False)
    assert len(open_v) == 2 * n and len(oi) == 2 * n


def test_tube_runs_between_its_end_points_and_swept_tube_chains_them():
    p0, p1 = np.array([0.0, 0, 0]), np.array([30.0, 40, 0])
    v, *_ = fc._tube(p0, p1, 5.0, n=12)
    a = (p1 - p0) / 50.0
    proj = (v - p0) @ a
    assert proj.min() == pytest.approx(0.0) and proj.max() == pytest.approx(50.0)
    pts = [[0, 0, 0], [100, 0, 0], [100, 100, 0]]
    sv, si, sj, sk = fc._swept_tube(pts, 4.0, n=10)
    assert len(sv) == 2 * (2 * 10 + 2)
    assert si.max() < len(sv) and sk.max() < len(sv)


def test_sphere_points_all_at_the_radius():
    v, i, *_ = fc._sphere([1, 2, 3], 40.0, n=16)
    assert np.linalg.norm(v - [1, 2, 3], axis=1) == pytest.approx(np.full(len(v), 40.0))
    assert len(i) == 2 * 15 * 15


def test_bbox_wire_has_twelve_edges_of_the_box():
    xs, ys, zs = fc._bbox_wire([0, 0, 0], [3, 4, 5])
    assert xs.count(None) == 12
    total = 0.0
    for s in range(12):
        p = np.array([xs[3 * s], ys[3 * s], zs[3 * s]], float)
        q = np.array([xs[3 * s + 1], ys[3 * s + 1], zs[3 * s + 1]], float)
        total += np.linalg.norm(q - p)
    assert total == pytest.approx(4 * (3 + 4 + 5))


def test_prism_and_ellipse_ring():
    ring = fc._ellipse_ring(8)
    assert np.hypot(*np.array(ring).T) == pytest.approx(np.ones(8))
    v, i, j, k = fc._prism_xsection(ring, [0.0, 10.0, 20.0],
                                    [(1, 1, 0, 0), (2, 3, 5, 0), (1, 1, 0, 7)])
    assert len(v) == 24 and len(i) == 2 * 8 * 2
    st1 = v[8:16]
    assert st1[:, 0] == pytest.approx(np.full(8, 10.0))
    assert st1[:, 1].max() == pytest.approx(5 + 2) and st1[:, 2].max() == pytest.approx(3)


def test_airfoil_section_has_the_naca_thickness():
    t = 0.12
    sec = fc._airfoil_section(n=401, thickness=t)
    upper, lower = sec[:401], sec[401:][::-1]
    assert upper[:, 0] == pytest.approx(lower[:, 0])
    thick = upper[:, 1] - lower[:, 1]
    assert thick.max() == pytest.approx(t, rel=2e-3)          # NACA 4-digit: max = t
    assert upper[np.argmax(thick), 0] == pytest.approx(0.30, abs=0.01)
    assert sec[0] == pytest.approx([0, 0]) and sec[-1] == pytest.approx([0, 0])


def test_wing_element_span_chord_and_incidence():
    v, *_ = fc._wing_element(100, 20, 50, chord=200, span=800, aoa_deg=0.0, n_sec=40)
    assert v[:, 1].min() == pytest.approx(20 - 400) and v[:, 1].max() == pytest.approx(20 + 400)
    assert v[:, 0].max() - v[:, 0].min() == pytest.approx(200, rel=1e-3)
    tilted, *_ = fc._wing_element(0, 0, 0, chord=200, span=800, aoa_deg=-10.0, n_sec=40)
    le = tilted[np.argmin(tilted[:, 0])]
    te = tilted[np.argmax(tilted[:, 0])]
    assert np.degrees(np.arctan2(te[2] - le[2], te[0] - le[0])) == pytest.approx(-10, abs=1.5)


def test_orient_part_mesh_axis_maps_rotations_and_recentring():
    V = np.array([[0, 0, 0], [0, 2, 0], [10, 1, 4]], float)     # bbox centre (5, 1, 2)
    out = fc._orient_part_mesh(V, centre=(100, 0, 0))
    assert (out.min(axis=0) + out.max(axis=0)) / 2 == pytest.approx([100, 0, 0])
    up = np.array([[0, -1, 0], [0, 1, 0]], float)
    assert fc._orient_part_mesh(up, axis_map="y_up")[1] == pytest.approx([0, 0, 1])
    xu = np.array([[-1, 0, 0], [1, 0, 0]], float)
    assert fc._orient_part_mesh(xu, axis_map="x_up")[1] == pytest.approx([0, 0, 1])
    y = np.array([[0, -1, 0], [0, 1, 0]], float)
    assert fc._orient_part_mesh(y, roll_deg=90)[1] == pytest.approx([0, 0, 1])
    z = np.array([[0, 0, -1], [0, 0, 1]], float)
    assert fc._orient_part_mesh(z, pitch_deg=90)[1] == pytest.approx([1, 0, 0])
    x = np.array([[-1, 0, 0], [1, 0, 0]], float)
    assert fc._orient_part_mesh(x, yaw_deg=90, scale=3)[1] == pytest.approx([0, 3, 0])
    assert fc._orient_part_mesh(np.zeros((0, 3))).shape == (0, 3)


def test_corner_transform_mirrors_scales_and_shifts():
    assert fc._corner_transform(None, mirror_y=False, lateral_scale=1, x_shift=0,
                                y_center_ref=0) is None
    p = [10.0, 300.0, 100.0]
    q = fc._corner_transform(p, mirror_y=True, lateral_scale=2.0, x_shift=500,
                             y_center_ref=0.0)
    assert q == pytest.approx([510, -600, 100])
    g = fc._corner_transform(p, mirror_y=False, lateral_scale=1.0, x_shift=0,
                             y_center_ref=0.0, size_scale=2.0, z_ground=20.0)
    assert g == pytest.approx([20, 300, 180])


# ------------------------------------------------------------------------ sizing laws
def test_wing_size_law_is_monotonic_and_clamped():
    assert fc._wing_span_chord(None, 1000, 200) == (1000, 200)
    assert fc._wing_span_chord(600, 1000, 200) == pytest.approx((1000, 200))
    spans = [fc._wing_span_chord(f, 1000, 200) for f in (100, 300, 600, 900, 1300, 5000)]
    assert all(b[0] >= a[0] and b[1] >= a[1] for a, b in zip(spans, spans[1:]))
    assert spans[0] == pytest.approx((1000 * 0.45 ** 0.3, 200 * 0.45 ** 0.5))
    assert spans[-1] == pytest.approx((1000 * 2.2 ** 0.3, 200 * 2.2 ** 0.5))


def test_heat_colour_ramp():
    assert fc.heat_color(0.0) == "#2563ad"
    assert fc.heat_color(1.0) == "#d62121"
    assert fc.heat_color(0.5) == "#5ad17a"
    assert fc.heat_color(-3) == fc.heat_color(0) and fc.heat_color(7) == fc.heat_color(1)
    mid = fc.heat_color(0.125)                      # halfway between the first two stops
    assert mid == "#2ea2be"                         # (46, 161.5, 190.5) rounded half-to-even


def test_colour_keys_and_labels():
    reg = {}
    a = fc._agnostic_color("L2 front", reg)
    b = fc._agnostic_color("L3", reg)
    assert fc._agnostic_color("L2 rear", reg) == a and a != b
    assert fc._agnostic_color(None, reg)[1] == "link"
    assert fc.sub_color_key("brakes") == "brake" and fc.sub_color_key("??") == "custom"
    assert fc.corner_label("FL") == "Front-left" and fc.corner_label("X") == "X"
    assert fc.part_label("Tire#FR") == "Tire · Front-right"
    assert fc.part_label("Brake disc") == "Brake disc (×4)"
    assert fc.part_label("upper#ZZ") == "Upper wishbone · ZZ"
    assert fc.part_label("panhardish") == "Panhardish"
    assert fc.part_label("Front wing") == "Front wing"
    assert fc.is_pickup("Upper ball joint#FL") and not fc.is_pickup("Tire")


def test_units_axis_follows_the_unit_system(monkeypatch):
    from suspension import units
    monkeypatch.setattr(units, "label", lambda u: "mm")
    m = fc._units_axis_cfg("x", 0.0, 3000.0)
    assert m["title"] == "x [mm]" and m["tickvals"] == [0, 500, 1000, 1500, 2000, 2500, 3000]
    monkeypatch.setattr(units, "label", lambda u: "in")
    i = fc._units_axis_cfg("x", 0.0, 3000.0)
    assert i["title"] == "x [in]"
    assert [float(t) * 25.4 for t in i["ticktext"]] == pytest.approx(i["tickvals"])
    steps = np.diff([float(t) for t in i["ticktext"]])
    assert set(np.round(steps, 9)) == {20.0}              # 118 in / 6 -> a 20 in step

    def boom(u):
        raise RuntimeError("no units")
    monkeypatch.setattr(units, "label", boom)
    assert fc._units_axis_cfg("z", 0, 10)["title"] == "z [mm]"
    assert "tickvals" not in fc._units_axis_cfg("z", "not a number", 10)
    assert "tickvals" not in fc._units_axis_cfg("z", 5, 5)


# ----------------------------------------------------------------- ledger read-outs
def test_ledger_helpers():
    class Broken:
        def get(self, name):
            raise KeyError(name)
    assert fc._iface(None, "brakes") is None and fc._iface(Broken(), "brakes") is None
    assert fc._g(None, "x", 3) == 3 and fc._g(NS(x=None), "x", 4) == 4
    led = _ledger(brakes=dict(brake_torque_nm=900.0))
    as_dict = led.as_dict()
    assert fc._iface(fc._as_ledger(as_dict), "brakes").brake_torque_nm == 900.0


def test_dict_ledger_now_drives_the_drawing():
    """A ledger in its as_dict() form (what session state holds) sizes the car like the object."""
    led = _ledger(electrics=dict(env_x_mm=300.0, env_y_mm=250.0, env_z_mm=200.0))
    a = fc.build_full_car_figure(vp=_vp(), ledger=led)._part_boxes["Accumulator"]
    b = fc.build_full_car_figure(vp=_vp(), ledger=led.as_dict())._part_boxes["Accumulator"]
    assert a["size"] == pytest.approx([300, 250, 200]) and b == a


def test_heat_metric_flags_the_sources_cooling_cannot_carry():
    led = _ledger(powertrain=dict(heat_reject_w=4000.0, cooling_airflow_cms=0.3),
                  electrics=dict(heat_reject_w=1500.0, cooling_airflow_cms=0.2),
                  cooling=dict(cooling_airflow_cms=0.4, heat_reject_w=100.0))
    m = fc.subsystem_metrics(_vp(), led, "heat")
    assert m["values"] == {"cooling": 100.0, "electrics": 1500.0, "powertrain": 4000.0}
    assert m["norm"]["electrics"] == pytest.approx(1500 / 4000) and m["vmax"] == 4000
    assert m["capacity"] == pytest.approx(0.4)
    assert m["over"] == {"electrics": True, "powertrain": True}       # cooling is not blamed
    assert [w["severity"] for w in m["warnings"]] == ["fail"]
    ok = _ledger(powertrain=dict(heat_reject_w=4000.0, cooling_airflow_cms=0.3),
                 cooling=dict(cooling_airflow_cms=0.4))
    assert fc.subsystem_metrics(_vp(), ok, "heat")["over"] == {}
    unknown = fc.subsystem_metrics(_vp(), led, "colour")
    assert unknown["metric"] == "heat" and unknown["over"] == m["over"]


def test_power_metric_blames_only_low_voltage_loads():
    led = _ledger(data_acquisition=dict(power_draw_w=50.0),
                  electrics=dict(power_draw_w=700.0, voltage_v=24.0),
                  powertrain=dict(power_draw_w=5000.0, voltage_v=350.0))
    m = fc.subsystem_metrics(_vp(), led, "power")
    assert m["capacity"] == 600.0
    assert m["over"] == {"data-acquisition": True, "electrics": True}
    assert {w["severity"] for w in m["warnings"]} == {"fail", "warning"}    # LV + HV mismatch
    led.lv_supply_capacity_w = 1000.0
    assert fc.subsystem_metrics(_vp(), led, "power")["over"] == {}


def test_mass_metric_marks_the_heaviest_contributors():
    led = _ledger(chassis=dict(mass_kg=120.0), powertrain=dict(mass_kg=90.0),
                  brakes=dict(mass_kg=10.0), electrics=dict(mass_kg=40.0))
    m = fc.subsystem_metrics(_vp(), led, "mass")
    assert sum(m["values"].values()) == 260.0 and m["capacity"] == 230.0
    assert m["over"] == {"chassis": True, "powertrain": True}            # mean = 65 kg
    assert any(w["severity"] == "fail" for w in m["warnings"])
    led.target_mass_kg = 300.0
    assert fc.subsystem_metrics(_vp(), led, "mass")["over"] == {}


def test_mass_metric_without_ledger_checks_uses_the_budget_directly():
    class Bare:                           # declarations but no check_all()
        target_mass_kg = "not a number"

        def get(self, name):
            return {"chassis": NS(mass_kg=150.0)}.get(name)
    m = fc.subsystem_metrics(_vp(target_mass_kg=100.0), Bare(), "mass")
    assert m["capacity"] == 100.0 and m["over"] == {"chassis": True} and m["warnings"] == []
    empty = fc.subsystem_metrics(_vp(), None, "mass")
    assert empty["values"] == {} and empty["vmax"] == 0.0 and empty["capacity"] == 0.0


# ------------------------------------------------------------------ the default car
@pytest.fixture(scope="module")
def car():
    return fc.build_full_car_figure(vp=_vp())


def test_contact_patches_sit_on_the_track_at_each_axle(car):
    hubs = {t.customdata[0][2]: t for t in car.data
            if t.type == "scatter3d" and t.legendgroup == "wheel"}
    expect = {"FL": (775, -600), "FR": (775, 600), "RL": (-775, -590), "RR": (-775, 590)}
    for cid, (x, y) in expect.items():
        assert (hubs[cid].x[0], hubs[cid].y[0]) == pytest.approx((x, y))
        assert hubs[cid].z[0] == pytest.approx(0.0, abs=1e-9)
        assert hubs[cid].z[1] == pytest.approx(R_TIRE)


def test_every_corner_part_has_four_instances_and_corner_keys(car):
    boxes = car._part_instance_boxes
    for name in ("Tire", "Brake disc"):
        assert len(boxes[name]) == 4
        xs = sorted(round(b["centre"][0]) for b in boxes[name])
        assert xs == [-775, -775, 775, 775]
    assert len(boxes["Sidepod (cooling)"]) == 2 and len(boxes["Spaceframe"]) == 1
    assert fc.part_corners(car, "Tire") == [(c, "Tire#" + c) for c in ("FL", "FR", "RL", "RR")]
    assert fc.part_corners(car, "Spaceframe") == []
    assert set(fc.subsys_centroids(car)) >= {"suspension", "chassis", "aerodynamics"}
    assert fc.subsys_centroids(go.Figure()) == {}


def test_default_car_nominal_bodies(car):
    pb = car._part_boxes
    assert pb["Accumulator"]["size"] == pytest.approx([1550 * 0.16, 160 * 1.2, R_TIRE * 0.55])
    assert len(_meshes(car, "Front wing")) == 3 + 2                    # 3 elements, 2 endplates
    span = 1200 * 0.98
    assert pb["Front wing"]["size"][1] == pytest.approx(span + 6, rel=1e-6)
    hover = [t.text for t in _meshes(car, "Brake disc")][0]
    assert hover == f"Brake disc · r≈{R_TIRE * 0.62:.0f} mm"
    assert "placeholder" in [t.text for t in _meshes(car, "Data logger")][0]
    cg = [t for t in car.data if t.name == "Centre of gravity"][0]
    assert cg.x[0] == pytest.approx(-775 + 0.46 * 1550) and cg.text[0] == "CG (params)"
    assert car.layout.uirevision == "wide"


def test_default_arguments_and_no_rocker_corner():
    plain = fc.build_full_car_figure()
    assert any(t.legendgroup == "push" for t in plain.data)
    hp = Hardpoints.default()
    hp.pushrod_outer = None
    bare = fc.build_full_car_figure(hp_front=hp, vp=_vp())
    groups = {t.legendgroup for t in bare.data if t.type == "scatter3d"}
    assert {"upper", "lower", "tie", "upright"} <= groups
    assert not groups & {"push", "rocker", "spring"}


def test_softer_springs_lower_the_body_not_the_wheels():
    lo = {}
    for kf in (35.0, 15.0, 60.0):
        f = fc.build_full_car_figure(vp=_vp(spring_rate_front=kf))
        pb = f._part_boxes
        ground = [t for t in f.data if t.name == "Ground"][0]
        assert np.asarray(ground.z) == pytest.approx(np.zeros((2, 2)))
        lo[kf] = {n: pb[n]["centre"][2] - pb[n]["size"][2] / 2 for n in pb}
        inner = [t for t in f.data if t.type == "scatter3d" and t.customdata is not None
                 and t.customdata[0][1:] == ["Upper front pickup", "FL"]][0]
        outer = [t for t in f.data if t.type == "scatter3d" and t.customdata is not None
                 and t.customdata[0][1:] == ["Upper ball joint", "FL"]][0]
        lo[kf]["inner"], lo[kf]["outer"] = inner.z[0], outer.z[0]
    drop = (35 - 15) * 0.6                                         # 12 mm of static sag
    for body in ("Spaceframe", "Front wing", "Accumulator", "Data logger", "inner"):
        assert lo[15][body] == pytest.approx(lo[35][body] - drop)
        assert lo[60][body] == pytest.approx(lo[35][body] + 12.0)  # clamp at -12 mm
    for unsprung in ("Tire", "Brake disc", "outer"):
        assert lo[15][unsprung] == pytest.approx(lo[35][unsprung])


def test_declared_numbers_size_the_bodies():
    led = _ledger(
        aerodynamics=dict(downforce_n_at_v=(1000.0, 20.0)),
        brakes=dict(brake_torque_nm=800.0),
        cooling=dict(cooling_airflow_cms=0.8, heat_reject_w=9000.0),
        powertrain=dict(peak_power_kw=80.0, peak_torque_nm=230.0),
        electrics=dict(mass_kg=30.0, power_draw_w=120.0),
        chassis=dict(mass_kg=31.5),
        data_acquisition=dict(mass_kg=1.2))
    f = fc.build_full_car_figure(vp=_vp(), ledger=led)
    pb = f._part_boxes
    assert len(_meshes(f, "Front wing")) == 4 + 2                     # 2 + 1000/500 elements
    disc_r = R_TIRE * min(0.62 + 800 / 4000, 0.85)
    assert _meshes(f, "Brake disc")[0].text == \
        f"Brake disc · r≈{disc_r:.0f} mm (sized from 800 N·m)"
    fac = 0.8 / 0.4
    pod = f._part_instance_boxes["Sidepod (cooling)"][0]["size"]
    assert pod == pytest.approx([1550 * 0.34 * fac ** 0.4, 110 * fac ** 0.5,
                                 R_TIRE * 0.7 * fac ** 0.4])
    assert "(sized from 0.80 m³/s) · rejects 9000 W" in _meshes(f, "Sidepod (cooling)")[0].text
    motor = _meshes(f, "Motor + inverter")[0].text
    assert motor == "Traction motor (sized from 80 kW) · 230 N·m"
    side = (30 * 1.6e6) ** (1 / 3)
    assert pb["Accumulator"]["size"] == pytest.approx([side * 1.4, side * 1.1, side * 0.7])
    assert "(sized from 30.0 kg) · 120 W" in _meshes(f, "Accumulator")[0].text
    assert "31.5 kg" in _meshes(f, "Spaceframe")[0].text
    assert _meshes(f, "Data logger")[0].text == "Data-acquisition logger · 1.2 kg"


def test_declared_envelopes_are_drawn_literally():
    led = _ledger(powertrain=dict(env_x_mm=300.0, env_y_mm=260.0, env_z_mm=200.0),
                  electrics=dict(env_x_mm=420.0, env_y_mm=310.0, env_z_mm=180.0))
    f = fc.build_full_car_figure(vp=_vp(), ledger=led)
    assert f._part_boxes["Accumulator"]["size"] == pytest.approx([420, 310, 180])
    assert "(declared envelope)" in _meshes(f, "Motor + inverter")[0].text
    motor_cyl = _xyz(_meshes(f, "Motor + inverter")[0])
    assert np.ptp(motor_cyl[:, 1]) == pytest.approx(260.0)            # lies across the car


def test_cg_from_the_ledger_and_from_dropped_parts():
    full = {s.replace("-", "_"): dict(mass_kg=25.0, cg_x_mm=700.0, cg_y_mm=0.0, cg_z_mm=300.0)
            for s in fc.SUBSYSTEMS}
    led = _ledger(**full)
    f = fc.build_full_car_figure(vp=_vp(), ledger=led)
    cg = [t for t in f.data if t.name == "Centre of gravity"][0]
    assert (cg.x[0], cg.z[0]) == pytest.approx((775 - 700, 300))      # +rearward from front axle
    assert cg.text[0] == "CG (declared 200 kg)"

    part = dict(name="Ballast", l_mm=100, w_mm=100, h_mm=100, x_mm=500, y_mm=100, z_mm=100,
                mass_kg=50.0)
    f2 = fc.build_full_car_figure(vp=_vp(), ledger=led, custom_parts=[part])
    cg2 = [t for t in f2.data if t.name == "Centre of gravity"][0]
    want = (np.array([75, 0, 300]) * 200 + np.array([500, 100, 100]) * 50) / 250
    assert [cg2.x[0], cg2.y[0], cg2.z[0]] == pytest.approx(want)
    assert cg2.text[0] == "CG (+50.0 kg parts)"

    no_led = fc.build_full_car_figure(vp=_vp(mass=300.0), custom_parts=[part])
    cg3 = [t for t in no_led.data if t.name == "Centre of gravity"][0]
    base_x = -775 + 0.46 * 1550
    assert cg3.x[0] == pytest.approx((base_x * 300 + 500 * 50) / 350)
    nominal = fc.build_full_car_figure(vp=_vp(), custom_parts=[part, dict(part, mass_kg="x")])
    cg4 = [t for t in nominal.data if t.name == "Centre of gravity"][0]
    assert cg4.x[0] == pytest.approx((base_x * 220 + 500 * 50) / 270)  # nominal 220 kg car


def test_cg_tolerates_a_ledger_without_a_rollup():
    class GetOnly:
        def get(self, name):
            return None
    part = dict(name="P", l_mm=10, w_mm=10, h_mm=10, mass_kg=10.0)
    f = fc.build_full_car_figure(vp=_vp(), ledger=GetOnly(), custom_parts=[part])
    cg = [t for t in f.data if t.name == "Centre of gravity"][0]
    assert cg.text[0] == "CG (+10.0 kg parts)"
    assert not [t for t in fc.build_full_car_figure(vp=_vp(cg_height=0.0)).data
                if t.name == "Centre of gravity"]


def test_layers_can_be_switched_off():
    f = fc.build_full_car_figure(vp=_vp(), show_tires=False, show_brakes=False,
                                 show_bodywork=False, show_aero=True, show_cooling=False,
                                 show_powertrain=False, show_electrics=False,
                                 show_floor=False, show_cg=False)
    names = {t.name for t in f.data}
    assert {"Front wing", "Data logger"} <= names
    assert not names & {"Tire", "Brake disc", "Spaceframe", "Sidepod (cooling)",
                        "Motor + inverter", "Accumulator", "Ground", "Centre of gravity"}
    mounts = [t for t in f.data if t.legendgroup == "fw_mount"]
    assert abs(mounts[0].y[1]) == pytest.approx(1200 * 0.98 * 0.12)    # no tub to mount on
    bare = fc.build_full_car_figure(vp=_vp(), show_aero=False, hp_rear=Hardpoints.default())
    assert not _meshes(bare, "Front wing") and not _meshes(bare, "Rear wing")


# ------------------------------------------------------------- overrides & suppression
def test_overrides_move_and_scale_a_part_about_its_own_centre(car):
    base = car._part_boxes["Front wing"]
    f = fc.build_full_car_figure(vp=_vp(), part_overrides={
        "Front wing": dict(dx=100, scale=None), "Accumulator": dict(scale=2.0),
        "Driver": dict(hide=True), "Radiator core": dict(sz=1.5),
        "Spaceframe": dict(dx=0, dy=0, dz=0, scale=1)})
    moved = f._part_boxes["Front wing"]
    assert moved["centre"][0] == pytest.approx(base["centre"][0] + 100)
    assert moved["size"] == pytest.approx(base["size"])
    acc0, acc1 = car._part_boxes["Accumulator"], f._part_boxes["Accumulator"]
    assert acc1["size"] == pytest.approx(2 * np.array(acc0["size"]))
    assert acc1["centre"] == pytest.approx(acc0["centre"])
    assert not _meshes(f, "Driver") and "Driver" in f._part_boxes
    assert f._part_boxes["Spaceframe"] == car._part_boxes["Spaceframe"]
    rad0 = car._part_instance_boxes["Radiator core"][0]["size"][2]
    assert f._part_instance_boxes["Radiator core"][0]["size"][2] == pytest.approx(1.5 * rad0)


def test_corner_parts_scale_in_place_and_members_move_at_all_four_corners(car):
    f = fc.build_full_car_figure(vp=_vp(), part_overrides={
        "Tire": dict(scale=1.1), "Tie rod": dict(dz=50), "fw_mount": dict(dx=-30)})
    c0 = sorted(b["centre"] for b in car._part_instance_boxes["Tire"])
    c1 = sorted(b["centre"] for b in f._part_instance_boxes["Tire"])
    assert np.array(c1) == pytest.approx(np.array(c0))
    tie = lambda fig: [t.z[0] for t in fig.data if t.legendgroup == "tie"]
    assert np.array(tie(f)) - np.array(tie(car)) == pytest.approx([50, 50, 50, 50])
    mx = lambda fig: [t.x[0] for t in fig.data if t.legendgroup == "fw_mount"]
    assert np.array(mx(f)) - np.array(mx(car)) == pytest.approx([-30, -30])


def test_suppression_hides_the_placeholder_but_keeps_its_box():
    f = fc.build_full_car_figure(vp=_vp(), suppress_subsystems={"aerodynamics"},
                                 suppress_parts={"Upright", "Wheel hub"})
    assert not _meshes(f, "Front wing") and not _meshes(f, "Rear wing")
    assert "Front wing" in f._part_boxes
    groups = [t.legendgroup for t in f.data if t.type == "scatter3d"]
    assert "upright" not in groups and "wheel" not in groups


# ------------------------------------------------------------- highlight & heat map
def test_highlight_lights_one_subteam_and_ghosts_the_rest():
    for hl in ("brakes", {"brakes"}, ["brakes", None]):
        f = fc.build_full_car_figure(vp=_vp(), highlight_subsystem=hl)
        assert _meshes(f, "Brake disc")[0].opacity == pytest.approx(0.92)
        assert _meshes(f, "Tire")[0].opacity == pytest.approx(0.95 * 0.16)
        susp = [t for t in f.data if t.legendgroup == "upper"][0]
        assert susp.opacity == pytest.approx(0.14)
        assert [t for t in f.data if t.name == "Centre of gravity"][0].opacity == 0.18
    none = fc.build_full_car_figure(vp=_vp(), highlight_subsystem=[])
    assert _meshes(none, "Tire")[0].opacity == pytest.approx(0.95)


def test_heat_map_colours_bodies_by_load_and_outlines_overloads():
    led = _ledger(powertrain=dict(heat_reject_w=4000.0, cooling_airflow_cms=0.5),
                  brakes=dict(heat_reject_w=1000.0),
                  suspension=dict(heat_reject_w=10.0),
                  cooling=dict(cooling_airflow_cms=0.2))
    hm = fc.subsystem_metrics(_vp(), led, "heat")
    stand_in = dict(name="Guess", provisional=True, l_mm=50, w_mm=50, h_mm=50)
    f = fc.build_full_car_figure(vp=_vp(), ledger=led, heatmap=hm, custom_parts=[stand_in])
    assert _meshes(f, "Motor + inverter")[0].color == fc.heat_color(1.0)
    assert _meshes(f, "Brake disc")[0].color == fc.heat_color(0.25)
    assert _meshes(f, "Spaceframe")[0].color == fc._HEAT_NEUTRAL
    outlines = [t for t in f.data if t.type == "scatter3d" and t.line.color == fc._OVER_BUDGET_COLOR]
    assert outlines                                    # powertrain and brakes are under-cooled
    arm = [t for t in f.data if t.legendgroup == "upper"][0]
    assert arm.line.color == fc.heat_color(10 / 4000) and arm.opacity == 0.95
    mount = [t for t in f.data if t.legendgroup == "fw_mount"][0]
    assert mount.line.color == fc._HEAT_NEUTRAL and mount.opacity == 0.5
    edges = [t for t in f.data if t.type == "scatter3d" and t.line.width == 3
             and t.customdata is None]
    assert len(edges) == 12 and edges[0].line.color == fc.COLORS["cg"]   # no subsystem: as drawn


# ------------------------------------------------------------------- custom parts
def _cube_mesh(lx, ly, lz):
    v, i, j, k = fc._box(0, 0, 0, lx, ly, lz)
    return dict(verts=v.tolist(), faces=np.column_stack([i, j, k]).tolist())


def test_custom_parts_land_in_real_millimetres():
    parts = [
        dict(name="  ", subsys="cooling", l_mm=289, w_mm=124, h_mm=34, x_mm=-80, y_mm=400, z_mm=150),
        dict(name="Fan", subsys="(custom / unassigned)", shape="cylinder", l_mm=40, w_mm=200,
             h_mm=0, x_mm=0, y_mm=-400, z_mm=200, color="#123456"),
        dict(name="Pump", subsys="cooling", provisional=True, l_mm=80, w_mm=60, h_mm=60),
        dict(name="Box (awaiting CAD)", provisional=True, l_mm=10, w_mm=10, h_mm=10),
        dict(name="Zero", l_mm=0, w_mm=10, h_mm=10),
        dict(name="Bad", l_mm="wide", w_mm=10, h_mm=10),
        dict(name="Bracket", subsys="chassis", mesh=_cube_mesh(50, 40, 30), axis_map="y_up",
             x_mm=100, y_mm=0, z_mm=300, dx_mm=5),
    ]
    f = fc.build_full_car_figure(vp=_vp(), custom_parts=parts)
    pb = f._part_boxes
    assert pb["Custom part"]["centre"] == pytest.approx([-80, 400, 150])
    assert pb["Custom part"]["size"] == pytest.approx([289, 124, 34])
    assert _meshes(f, "Custom part")[0].color == fc.COLORS["radiator"]
    fan = _meshes(f, "Fan")[0]
    assert fan.color == "#123456" and fan.customdata is None              # unassigned
    assert pb["Fan"]["size"][1] == pytest.approx(200, rel=1e-3)
    pump = _meshes(f, "Pump (awaiting CAD)")[0]
    assert pump.opacity == pytest.approx(0.30) and "PROVISIONAL" in pump.text
    wire = [t for t in f.data if t.type == "scatter3d" and t.line.color == fc.COLORS["cg"]
            and t.line.width == 3]
    assert len(wire) == 2 * 12                                            # both stand-ins
    assert _meshes(f, "Box (awaiting CAD)")
    assert not _meshes(f, "Zero") and not _meshes(f, "Bad")
    br = pb["Bracket"]
    assert br["centre"] == pytest.approx([105, 0, 300])
    assert br["size"] == pytest.approx([50, 30, 40])                     # y-up export: y<->z
    assert _meshes(f, "Bracket")[0].color == fc.COLORS["cad"]


def test_define_car_chassis_fills_the_placeholder_frame():
    car = dict(name="Frame CAD", subsys="chassis", define_car=True,
               mesh=_cube_mesh(2000, 500, 400), l_mm=2000, w_mm=500, h_mm=400)
    f = fc.build_full_car_figure(vp=_vp(), custom_parts=[car],
                                 suppress_subsystems={"chassis"})
    frame, cad = f._part_boxes["Spaceframe"], f._part_boxes["Frame CAD"]
    lo_f = np.array(frame["centre"]) - np.array(frame["size"]) / 2
    hi_f = np.array(frame["centre"]) + np.array(frame["size"]) / 2
    lo_c = np.array(cad["centre"]) - np.array(cad["size"]) / 2
    hi_c = np.array(cad["centre"]) + np.array(cad["size"]) / 2
    assert np.all(lo_c >= lo_f - 1e-6) and np.all(hi_c <= hi_f + 1e-6)
    # Height is the tightest ratio here (400 mm of CAD into a 0.95 r deck), so it matches.
    assert cad["size"][2] == pytest.approx(R_TIRE * 0.95)
    ratios = np.array(cad["size"]) / np.array([2000, 500, 400])
    assert ratios == pytest.approx(np.full(3, ratios[0]))         # true shape kept
    no_dims = fc.build_full_car_figure(vp=_vp(), custom_parts=[dict(car, l_mm=0, w_mm=0, h_mm=0)])
    assert no_dims._part_boxes["Frame CAD"]["size"] == pytest.approx([2000, 500, 400])
    broken = fc.build_full_car_figure(vp=_vp(), custom_parts=[dict(car, l_mm="?")])
    assert "Frame CAD" not in broken._part_boxes and "Spaceframe" in broken._part_boxes


# ------------------------------------------------------------------------ camera
def test_focus_zooms_on_a_part_a_point_or_a_subsystem(car):
    f = fc.build_full_car_figure(vp=_vp(), focus_part="Front wing")
    assert f.layout.uirevision == "part:Front wing"
    allp = np.vstack([np.asarray(_xyz(t)) for t in f.data
                      if t.customdata is not None and t.type in ("mesh3d", "scatter3d")])
    smin, smax = allp.min(axis=0), allp.max(axis=0)
    pb = f._part_boxes["Front wing"]
    c = (np.array(pb["centre"]) - (smin + smax) / 2) / (smax - smin)
    cam = f.layout.scene.camera.center
    assert [cam.x, cam.y, cam.z] == pytest.approx(c, abs=1e-6)

    p = fc.build_full_car_figure(vp=_vp(), focus_part="Upper ball joint#FL")
    eye, ctr = p.layout.scene.camera.eye, p.layout.scene.camera.center
    d = np.linalg.norm([eye.x - ctr.x, eye.y - ctr.y, eye.z - ctr.z])
    assert d == pytest.approx(0.85 + 0.08 * 1.4)                  # a point gets a standoff

    s = fc.build_full_car_figure(vp=_vp(), focus_part="Nope", focus_subsystem="cooling")
    assert s.layout.uirevision == "focus:cooling"
    w = fc.build_full_car_figure(vp=_vp(), focus_subsystem="nothing-here")
    assert w.layout.uirevision == "wide"


# --------------------------------------------------------------- other topologies
def test_macpherson_corner_draws_its_own_members():
    from suspension import adapter, topologies
    gk = adapter.GenericKinematics(topologies.example("macpherson_strut"))
    f = fc.build_full_car_figure(vp=_vp(spring_rate_front=20.0), corner_front=gk)
    labels = {t.name for t in f.data if t.type == "scatter3d" and t.name}
    assert {"strut_body", "LF", "TR"} <= labels and "Wheel" not in labels
    picks = dict(fc.pickup_parts(f))
    assert "Strut top" in picks and "Upper front pickup" not in picks    # rear follows front
    hubs = [t for t in f.data if t.legendgroup == "wheel"]
    front = [h for h in hubs if h.customdata[0][2].startswith("F")]
    assert [abs(h.y[0]) for h in front] == pytest.approx([600, 600])
    assert all(h.z[0] == pytest.approx(0.0, abs=1e-9) for h in hubs)   # still on the ground


# ----------------------------------------------------------- reading a figure back
def test_available_parts_and_pickups(car):
    parts = fc.available_parts(car)
    susp = [k for _l, k in parts["suspension"]]
    assert susp[:2] == ["upper", "lower"] and "Upper ball joint" not in susp
    assert ("Brake disc (×4)", "Brake disc") in parts["brakes"]
    assert not {"fw_mount", "rw_mount", "drive", "helmet"} & {k for v in parts.values() for _l, k in v}
    with_p = fc.available_parts(car, include_pickups=True)
    assert "Upper ball joint" in [k for _l, k in with_p["suspension"]]
    pk = [k for _l, k in fc.pickup_parts(car)]
    assert len(pk) == len(set(pk)) == 8
    odd = NS(data=[NS(customdata=None), NS(customdata=[5]), NS(customdata=[["s"]]),
                   NS(customdata=[[None, "x"]]), NS(customdata=[["s", "k", ""]]),
                   NS(customdata=[["s", "k", "FL"]]), NS(customdata=[["s", "k", "FR"]])])
    assert fc.available_parts(odd) == {"s": [("K", "k")]}
    assert fc.pickup_parts(odd) == []
    assert fc.part_corners(odd, "k") == [("FL", "k#FL"), ("FR", "k#FR")]


def test_dummy_footprint_matches_the_drawn_box(car):
    fp = fc.dummy_body_footprint(car, "Accumulator")
    box = car._part_boxes["Accumulator"]
    assert [fp["x_mm"], fp["y_mm"], fp["z_mm"]] == pytest.approx(box["centre"])
    assert [fp["l_mm"], fp["w_mm"], fp["h_mm"]] == pytest.approx(box["size"])
    assert fc.dummy_body_footprint(car, "No such part") is None
    assert fc.dummy_body_footprint(car, "Ground")["l_mm"] > 1550
    nan = float("nan")
    odd = NS(data=[NS(name="p", x=None, y=[0], z=[0]), NS(name="p", x=[nan], y=[nan], z=[nan]),
                   NS(name="p", x=["a"], y=[0], z=[0]), NS(name="p", x=[1, 3], y=[2, 2], z=[0, 4])])
    assert fc.dummy_body_footprint(odd, "p") == dict(l_mm=2.0, w_mm=0.0, h_mm=4.0,
                                                     x_mm=2.0, y_mm=2.0, z_mm=2.0)


# ------------------------------------------------------------- plain-language helpers
def test_influence_summary_says_what_the_drawing_does():
    rows = {r["subsystem"]: r for r in fc.influence_summary(_vp(), None)}
    assert rows["electrics"] == dict(subsystem="electrics", status="default",
                                     detail="no mass/envelope → placeholder battery box")
    assert rows["chassis"]["detail"] == "spaceframe (no mass declared)"
    assert rows["aerodynamics"]["status"] == "default" and "ALL" not in rows
    led = _ledger(aerodynamics=dict(downforce_n_at_v=(900.0, 15.0)),
                  powertrain=dict(env_x_mm=300.0, env_y_mm=200.0, env_z_mm=200.0),
                  cooling=dict(cooling_airflow_cms=0.5), brakes=dict(brake_torque_nm=700.0),
                  electrics=dict(env_x_mm=400.0, env_y_mm=300.0, env_z_mm=200.0),
                  chassis=dict(mass_kg=30.0))
    r = {x["subsystem"]: x for x in fc.influence_summary(_vp(), led.as_dict(), "MacPherson")}
    assert r["aerodynamics"]["detail"] == "900 N @ 15 m/s → wing span/chord"
    assert r["powertrain"] == dict(subsystem="powertrain", status="sized",
                                   detail="declared envelope → motor + inverter size")
    assert r["cooling"]["detail"] == "0.50 m³/s → sidepod size"
    assert r["brakes"]["detail"] == "700 N·m → brake-disc diameter"
    assert r["electrics"]["detail"] == "declared envelope → battery box"
    assert r["chassis"]["detail"] == "30.0 kg spaceframe"
    assert r["suspension"]["detail"].startswith("MacPherson · track F/R 1200/1180 mm")
    assert r["ALL"]["detail"] == "declared 30.0 kg vs target 230 kg (Δ -200.0 kg); CG needs all masses+positions"
    r2 = {x["subsystem"]: x for x in fc.influence_summary(
        _vp(), _ledger(powertrain=dict(peak_power_kw=80.0), electrics=dict(mass_kg=25.0)))}
    assert r2["powertrain"]["detail"] == "80 kW → motor + inverter size"
    assert r2["electrics"]["detail"] == "25.0 kg → battery box size"
    full = {s.replace("-", "_"): dict(mass_kg=25.0, cg_x_mm=1.0, cg_y_mm=0.0, cg_z_mm=1.0)
            for s in fc.SUBSYSTEMS}
    r3 = {x["subsystem"]: x for x in fc.influence_summary(_vp(), _ledger(**full))}
    assert r3["ALL"]["detail"].endswith("; CG live")

    class GetOnly:
        def get(self, name):
            return None
    assert "ALL" not in {x["subsystem"] for x in fc.influence_summary(_vp(), GetOnly())}
    assert fc.override_influence_summary is fc.influence_summary


def test_custom_part_fit_reports_clearance_in_millimetres():
    ok = fc.custom_part_fit(_vp(), dict(l_mm=200, w_mm=200, h_mm=200, z_mm=300))
    assert ok["status"] == "ok" and ok["clearances"]["front of front axle"] == pytest.approx(675)
    tight = fc.custom_part_fit(_vp(), dict(l_mm=100, w_mm=100, h_mm=100, x_mm=715, z_mm=300))
    assert tight["status"] == "tight" and tight["messages"] == ["only 10 mm clear front of front axle"]
    late = fc.custom_part_fit(_vp(), dict(l_mm=100, w_mm=100, h_mm=100, x_mm=-750, y_mm=530,
                                          z_mm=300))
    assert late["status"] == "over" and late["messages"] == [
        "pokes out behind rear axle by 25 mm", "only 10 mm clear right of track"]
    over = fc.custom_part_fit(_vp(), dict(l_mm=100, w_mm=1200, h_mm=100, x_mm=715, z_mm=30))
    assert over["status"] == "over"
    assert "pokes out right of track by 10 mm" in over["messages"]
    assert "pokes out below ground by 20 mm" in over["messages"]
    assert "only 10 mm clear front of front axle" in over["messages"]


def test_part_dims_from_mesh_summary():
    d = fc.part_dims_from_mesh(dict(size_mm=[-10, 20, 30], bbox_min=[0, 0, 0], bbox_max=[10, 20, 30]))
    assert d == dict(l_mm=10, w_mm=20, h_mm=30, centre_mm=[5, 10, 15], size_mm=[10, 20, 30])
    assert fc.part_dims_from_mesh({})["l_mm"] == 0.0


def test_reconcile_part_verdicts():
    g = dict(l_mm=300, w_mm=200, h_mm=100)
    assert fc.reconcile_part({}, g)["status"] == "new"
    assert fc.reconcile_part(g, dict(l_mm=305, w_mm=196, h_mm=100))["status"] == "match"
    assert fc.reconcile_part(g, dict(l_mm=100, w_mm=300, h_mm=200))["status"] == "mirrored"
    assert fc.reconcile_part(g, dict(l_mm=100.4, w_mm=304, h_mm=203))["status"] == "mirrored"
    r = fc.reconcile_part(g, dict(l_mm=360, w_mm=200, h_mm=80))
    assert r["status"] == "resize" and r["deltas"] == [60, 0, -20]
    assert r["messages"] == ["Real part differs from your stand-in: L +60 mm, H -20 mm. "
                             "Repackage around the real size."]


def test_resize_lists_only_the_axes_that_failed():
    """Tolerance is 5% of the larger extent; a 52 mm change on a 1052 mm part is inside it."""
    g = dict(l_mm=1000, w_mm=200, h_mm=100)
    r = fc.reconcile_part(g, dict(l_mm=1052, w_mm=260, h_mm=100))
    assert r["status"] == "resize"
    assert r["messages"][0] == ("Real part differs from your stand-in: W +60 mm. "
                                "Repackage around the real size.")


@pytest.mark.parametrize("sub,shape", [
    ("powertrain", "cylinder"), ("electrics", "box"), ("cooling", "box"),
    ("aerodynamics", "box"), ("brakes", "cylinder"), ("data-acquisition", "box"),
    ("chassis", "box"), ("suspension", "box"), ("mystery", "box")])
def test_suggested_geometry_for_every_subsystem(sub, shape):
    s = fc.suggest_part_geometry(_vp(), sub)
    assert s["shape"] == shape and len(s["basis"]) == 3 and s["from_declared"] == []
    assert min(s["l_mm"], s["w_mm"], s["h_mm"]) > 0
    assert abs(s["x_mm"]) <= 1550 / 2 + 2 * R_TIRE


def test_suggested_geometry_completes_a_partial_declaration():
    led = _ledger(electrics=dict(env_x_mm=500.0, env_z_mm=150.0))
    s = fc.suggest_part_geometry(_vp(), "electrics", ledger=led.as_dict())
    assert (s["l_mm"], s["h_mm"]) == (500, 150) and s["from_declared"] == ["L", "H"]
    assert s["w_mm"] == round(min((1180 / 2 - 220) * 2 * 0.95, 340))
    w = fc.suggest_part_geometry(_vp(), "cooling", ledger=_ledger(cooling=dict(env_y_mm=90.0)))
    assert w["w_mm"] == 90 and w["from_declared"] == ["W"]


@pytest.mark.parametrize("key", ["front_wing", "rear_wing", "roll_hoop", "driver", "data_logger",
                                 "tire", "brake_disc", "upright", "motor", "accumulator"])
def test_part_anchors(key):
    a = fc.part_anchor(_vp(), key)
    assert min(a["l_mm"], a["w_mm"], a["h_mm"]) > 0 and len(a["basis"]) == 3
    if key in ("motor", "accumulator"):
        sub = fc.PART_SUBSYS_BY_KEY[key]
        assert a == fc.suggest_part_geometry(_vp(), sub)


def test_catalogs_are_consistent():
    keys = [k for k, _d, _n in fc.subsystem_catalog()]
    assert set(keys) <= set(fc.SUBSYSTEMS)
    for k, dn, sub, _corner in fc.part_catalog():
        assert dn in fc.SUBSYS_DRAWNAMES[sub] and fc.PART_KEY_BY_DRAWNAME[dn] == k
    assert set(fc.SUBSYS_ANCHOR_PART.values()) <= set(fc.PART_DRAWNAME_BY_KEY)
