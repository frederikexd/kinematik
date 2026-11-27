"""suspension/fullcar3d.py against tests/golden: the full-car figure for a set of cars (default, a declared ledger,
every layer off, highlight, focus, heat map, part edits, custom parts, suppression), each trace digested by type,
name, size and a SHA-256 of its rounded data, plus every helper's output for every part and subsystem. Regenerate
with KINEMATIK_REGOLD=1 only after reading the diff."""
import hashlib
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import fullcar3d as fc
from suspension.interfaces import IntegrationLedger, SubsystemInterface
from suspension.kinematics import Hardpoints

pytestmark = pytest.mark.filterwarnings("ignore")
GOLD = Path(__file__).parent / "golden" / "fullcar3d.json"


def _r(x):
    if isinstance(x, dict):
        return {str(k): _r(v) for k, v in sorted(x.items(), key=lambda kv: str(kv[0]))}
    if isinstance(x, (list, tuple)):
        return [_r(v) for v in x]
    if isinstance(x, np.ndarray):
        return _r(x.tolist())
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return _r(x.item())
    if isinstance(x, float):
        return None if math.isnan(x) else ("inf" if math.isinf(x) else round(x, 6) + 0.0)
    if isinstance(x, set):
        return sorted(_r(v) for v in x)
    return x


def _sha(x):
    return hashlib.sha256(json.dumps(_r(x), sort_keys=True, default=str).encode()).hexdigest()[:20]


def _digest(fig):
    js = fig.to_plotly_json()
    js["layout"].pop("template", None)      # plotly's ambient default (Streamlit swaps it on import), not the figure's
    traces = [[t.get("type"), t.get("name"), len(t.get("x") or []), _sha(t)] for t in js["data"]]
    return {"traces": traces, "layout": _sha(js["layout"]), "n": len(traces),
            "boxes": _r(getattr(fig, "_part_boxes", {})), "instances": _r(getattr(fig, "_part_instance_boxes", {})),
            "centroids": _r(fc.subsys_centroids(fig)),
            "parts": _r(fc.available_parts(fig)), "parts_all": _r(fc.available_parts(fig, include_pickups=True)),
            "pickups": _r(fc.pickup_parts(fig)),
            "corners": {k: _r(fc.part_corners(fig, k)) for k in fc.available_parts(fig, include_pickups=True)},
            "dummies": {k: _r(fc.dummy_body_footprint(fig, k)) for k in ("Driver", "Accumulator", "Nope")}}


def _vp(**kw):
    base = dict(wheelbase=1550.0, track_front=1200.0, track_rear=1180.0, spring_rate_front=35.0, cg_height=280.0,
                weight_dist_front=0.46, mass=0.0)
    base.update(kw)
    return NS(**base)


def _ledger():
    led = IntegrationLedger()
    for name, f in (("chassis", dict(mass_kg=32.0, cg_x_mm=800.0, cg_z_mm=300.0)),
                    ("aerodynamics", dict(mass_kg=9.0, downforce_n=900.0)),
                    ("cooling", dict(mass_kg=6.0, heat_reject_w=2500.0, cooling_airflow_cms=0.4)),
                    ("powertrain", dict(mass_kg=28.0, heat_reject_w=6000.0, peak_current_a=200.0)),
                    ("electrics", dict(mass_kg=45.0, heat_reject_w=900.0, power_draw_w=300.0, voltage_v=400.0,
                                       env_x_mm=600.0, env_y_mm=400.0, env_z_mm=300.0,
                                       env_origin_mm=(700.0, -200.0, 100.0))),
                    ("suspension", dict(mass_kg=20.0, mount_load_n=8000.0)),
                    ("brakes", dict(mass_kg=8.0, heat_reject_w=4000.0))):
        try:
            led.set(SubsystemInterface(name=name, **f))
        except TypeError:
            led.set(SubsystemInterface(name=name, **{k: v for k, v in f.items() if k != "downforce_n"}))
    return led


def _figures():
    """name -> a function that builds the figure (built one at a time, so a broken one fails before the rest)."""
    hp = Hardpoints.default()
    led = _ledger()
    F = fc.build_full_car_figure
    return {
        "default": lambda: F(),
        "car": lambda: F(hp_front=hp, vp=_vp(), hp_rear=hp, ledger=led, topology_label="Pushrod"),
        "car_dict_ledger": lambda: F(hp_front=hp, vp=_vp(wheelbase=1600.0, track_front=1250.0),
                                                    ledger=led.as_dict(), tire_width_mm=200.0, height=600),
        "all_off": lambda: F(vp=_vp(), show_chassis=False, show_tires=False, show_floor=False,
                                            show_aero=False, show_powertrain=False, show_cooling=False,
                                            show_electrics=False, show_brakes=False, show_bodywork=False,
                                            show_cg=False),
        "highlight": lambda: F(vp=_vp(), ledger=led, highlight_subsystem="cooling"),
        "focus": lambda: F(vp=_vp(), ledger=led, focus_subsystem="electrics",
                                          focus_part="Accumulator"),
        "heat": lambda: F(vp=_vp(), ledger=led, heatmap=fc.subsystem_metrics(_vp(), led, "heat")),
        "edits": lambda: F(vp=_vp(), ledger=led, part_overrides={
            "accumulator": {"l_mm": 500.0, "w_mm": 350.0, "h_mm": 250.0, "dx_mm": 40.0, "dz_mm": -20.0},
            "radiator": {"scale": 1.3}, "front_wing": {"hidden": True}},
            custom_parts=[{"name": "Logger", "subsystem": "electrics", "l_mm": 120.0, "w_mm": 80.0, "h_mm": 40.0,
                           "x_mm": 900.0, "y_mm": 150.0, "z_mm": 200.0, "color": "#ff0000"},
                          {"name": "Huge", "subsystem": "cooling", "l_mm": 3000.0, "w_mm": 2000.0, "h_mm": 900.0,
                           "x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0}]),
        "suppressed": lambda: F(vp=_vp(), ledger=led, suppress_subsystems={"aerodynamics"},
                                               suppress_parts={"Driver", "Radiator core"}),
        "odd_vp": lambda: F(vp=ODD, ledger=led, hp_front=_hp(static_camber=-3.0), hp_rear=_hp(pushrod_outer=None)),
        "empty_vp": lambda: F(vp=NS(), hp_front=_hp(rocker_pivot=None), hp_rear=_hp(spring_inner=None)),
        "no_rocker": lambda: F(vp=ODD, hp_front=_hp(rocker_pushrod=None), hp_rear=_hp(rocker_spring=None)),
        "overrides": lambda: F(vp=ODD, ledger=led, part_overrides={
            "Accumulator": {"dx": 40.0, "dy": -15.0, "dz": -20.0, "sx": 1.2, "sy": 0.9, "sz": 1.1},
            "Radiator core": {"scale": 1.3}, "Front wing": {"hide": True}, "Rear wing": {"scale": None, "dz": 25.0},
            "Driver": {"dx": 0.0, "dy": 0.0, "dz": 0.0, "scale": 1.0}, "Motor + inverter": {"sx": 0, "scale": 0.8},
            "Sidepod (cooling)": {"dy": 30.0}, "Spaceframe": {"sz": 1.15}, "Roll hoop": {}, "Tire": {"scale": 1.1},
            "Brake disc": {"dx": 5.0}, "Upright": {"hide": True}, None: {"dx": 9.0}}),
        "partial_vp": lambda: F(vp=NS(wheelbase=1580.0, cg_height=290.0), ledger=led,
                                custom_parts=[dict(name="Ballast", l_mm=50.0, w_mm=50.0, h_mm=50.0, mass_kg=5.0,
                                                   x_mm=800.0, y_mm=100.0, z_mm=150.0),
                                              dict(name="Light", l_mm=50.0, w_mm=50.0, h_mm=50.0, mass_kg=0.5),
                                              dict(name="Massless", l_mm=50.0, w_mm=50.0, h_mm=50.0)]),
        "partial_vp_noledger": lambda: F(vp=NS(cg_height=300.0, mass=240.0, weight_dist_front=0.47),
                                         custom_parts=[dict(name="Ballast", l_mm=50.0, w_mm=50.0, h_mm=50.0,
                                                            mass_kg=3.0, x_mm=600.0, y_mm=-50.0, z_mm=120.0)]),
        "heat_over": lambda: F(vp=ODD, ledger=_budget_ledger("cool_fail"),
                               heatmap=fc.subsystem_metrics(ODD, _budget_ledger("cool_fail"), "heat")),
        "power_over": lambda: F(vp=ODD, ledger=_budget_ledger("lv_fail"), highlight_subsystem="electrics",
                                heatmap=fc.subsystem_metrics(ODD, _budget_ledger("lv_fail"), "power")),
        "heat_empty": lambda: F(vp=ODD, heatmap={}),
        "focus_tire": lambda: F(vp=ODD, ledger=led, focus_subsystem="suspension", focus_part="Tire"),
        "focus_corner": lambda: F(vp=ODD, ledger=led, focus_subsystem="suspension", focus_part="Tire#FR"),
        "focus_pickup": lambda: F(vp=ODD, focus_subsystem="suspension", focus_part="Lower ball joint"),
        "focus_sub": lambda: F(vp=ODD, ledger=led, focus_subsystem="brakes"),
        "focus_missing": lambda: F(vp=ODD, focus_subsystem="nope", focus_part="nope"),
        "custom_all": lambda: F(vp=ODD, ledger=led, custom_parts=CUSTOM),
        "custom_define": lambda: F(vp=ODD, custom_parts=[dict(name="Frame CAD", define_car=True, mesh=_mesh(1800, 600, 500),
                                                              l_mm=1800.0, w_mm=600.0, h_mm=500.0, dx_mm=15.0,
                                                              dy_mm=-8.0, dz_mm=4.0)] + CUSTOM[:3]),
        "custom_define_flat": lambda: F(custom_parts=[dict(name="Frame", define_car=True, mesh=_mesh(1000, 900, 300))]),
        "brake_tq": lambda: F(vp=_vp(), ledger=_with_brakes(800.0)),
        "brake_tq_big": lambda: F(vp=_vp(), ledger=_with_brakes(5000.0), tire_width_mm=100.0),
        "heat_partial": lambda: F(vp=_vp(), ledger=led, heatmap={"metric": "heat"}),
        **{f"cg_{n}": (lambda v=v, lg=lg: F(vp=v, ledger=lg, custom_parts=[dict(name="Ballast", l_mm=50.0, w_mm=50.0,
                                                                            h_mm=50.0, mass_kg=5.0, x_mm=800.0,
                                                                            y_mm=100.0, z_mm=150.0)]))
           for n, v, lg in (("blank_ledger", NS(cg_height=300.0, mass=240.0), IntegrationLedger()),
                            ("half_kg", NS(cg_height=300.0, mass=240.0), _tiny_ledger()),
                            ("vp_half", NS(cg_height=300.0, mass=0.5), None),
                            ("low", NS(cg_height=0.5, mass=240.0), None), ("zero", NS(cg_height=0.0, mass=240.0), None))},
        "hl_susp": lambda: F(vp=_vp(), ledger=led, highlight_subsystem="suspension"),
        "narrow": lambda: F(vp=NS(wheelbase=1500.0, track_front=800.0, track_rear=820.0, spring_rate_front=50.0),
                            custom_parts=[dict(name="Frame CAD", define_car=True, mesh=_mesh(1800, 600, 500),
                                               l_mm=1800.0, w_mm=600.0, h_mm=500.0)]),
        **{f"define_{i}": (lambda dims=dims: F(vp=_vp(), custom_parts=[dict(
            name="Frame CAD", define_car=True, mesh=_mesh(900, 500, 400), l_mm=dims[0], w_mm=dims[1], h_mm=dims[2])]))
           for i, dims in enumerate(((100.0, 2000.0, 50.0), (100.0, 100.0, 3000.0), (1.0, 1.0, 1.0),
                                     (1.5, 1.5, 1.5), (0.0, 0.0, 0.0)))},
        **{f"wheel_{z}": (lambda z=z: F(vp=_vp(), hp_front=_hp(wheel_center=(0.0, 600.0, z)),
                                        custom_parts=[dict(name="Frame CAD", define_car=True, mesh=_mesh(1800, 600, 500),
                                                           l_mm=1800.0, w_mm=600.0, h_mm=500.0)]))
           for z in (150.0, 420.0)},
        **{f"gk_{n}": (lambda n=n: F(vp=ODD, corner_front=_gk(n), corner_rear=_gk("trailing_arm" if n != "trailing_arm"
                                                                                  else "solid_axle")))
           for n in GK},
    }


def _tiny_ledger():
    led = IntegrationLedger()
    led.set(SubsystemInterface(name="brakes", mass_kg=0.5, cg_x_mm=800.0, cg_y_mm=0.0, cg_z_mm=200.0))
    return led


def _with_brakes(tq):
    led = _ledger()
    led.set(SubsystemInterface(name="brakes", mass_kg=8.0, heat_reject_w=4000.0, brake_torque_nm=tq))
    return led


def _mesh(l, w, h):
    v = [[0, 0, 0], [l, 0, 0], [0, w, 0], [0, 0, h], [l, w, h]]
    return {"verts": [[a - l / 2, b - w / 2, c - h / 2] for a, b, c in v],
            "faces": [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 4], [1, 3, 4], [2, 3, 4]]}


CUSTOM = [
    dict(name="  ", subsys="(custom / unassigned)", l_mm=120.0, w_mm=80.0, h_mm=40.0, x_mm=900.0, y_mm=150.0, z_mm=200.0),
    dict(name=None, subsys="electrics", l_mm=100.0, w_mm=60.0, x_mm=700.0, y_mm=-120.0, z_mm=250.0, shape="cylinder"),
    dict(name="Flat box", l_mm=100.0, w_mm=60.0, h_mm=0.0, x_mm=10.0, y_mm=10.0, z_mm=10.0),
    dict(name="No width", l_mm=100.0, w_mm=0.0, h_mm=10.0),
    dict(name="No l", w_mm=50.0, h_mm=50.0, x_mm=40.0), dict(name="No w", l_mm=50.0, h_mm=50.0, x_mm=50.0),
    dict(name="No h", l_mm=50.0, w_mm=50.0, x_mm=60.0),
    dict(name="At origin", l_mm=50.0, w_mm=40.0, h_mm=30.0),
    dict(name="Half mm", l_mm=0.5, w_mm=0.5, h_mm=0.5, x_mm=1.0, y_mm=2.0, z_mm=3.0),
    dict(name="Prov", provisional=True, subsys="cooling", l_mm=200.0, w_mm=100.0, h_mm=50.0, x_mm=300.0, y_mm=20.0,
         z_mm=300.0, color="#00ff00"),
    dict(name="Prov plain", provisional=True, l_mm=210.0, w_mm=110.0, h_mm=55.0, x_mm=310.0, y_mm=-20.0, z_mm=310.0),
    dict(name="Prov2 (awaiting CAD)", provisional=True, l_mm=200.0, w_mm=100.0, h_mm=50.0, shape="cylinder",
         x_mm=320.0, y_mm=40.0, z_mm=320.0),
    dict(name="Mesh part", subsys="powertrain", mesh=_mesh(100, 80, 60), x_mm=500.0, y_mm=30.0, z_mm=200.0,
         dx_mm=12.0, dy_mm=-7.0, dz_mm=5.0, mesh_scale=1.5, axis_map="y_up", yaw_deg=30.0, roll_deg=10.0,
         pitch_deg=-5.0, l_mm=100.0, w_mm=80.0, h_mm=60.0),
    dict(name="Mesh x-up", subsys="aerodynamics", mesh=_mesh(40, 30, 20), x_mm=550.0, y_mm=-30.0, z_mm=220.0,
         axis_map="x_up", mesh_scale=0.0),
    dict(name="Mesh prov", provisional=True, subsys="brakes", mesh=_mesh(50, 50, 50), x_mm=600.0, color="#123456"),
    dict(name="Mesh prov plain", provisional=True, subsys="brakes", mesh=_mesh(55, 50, 45), x_mm=620.0),
    dict(name="Mesh no faces", mesh={"verts": [[0, 0, 0], [1, 1, 1]], "faces": []}, l_mm=50.0, w_mm=50.0, h_mm=50.0,
         x_mm=650.0, y_mm=5.0, z_mm=6.0),
    dict(name="Mesh only verts", mesh={"verts": [[0, 0, 0], [1, 1, 1]]}, x_mm=660.0),
    dict(name="Define no mesh", define_car=True, l_mm=60.0, w_mm=60.0, h_mm=60.0, x_mm=670.0, y_mm=7.0, z_mm=8.0),
]


GK = ("macpherson_strut", "multilink", "semi_trailing_arm", "solid_axle", "trailing_arm", "twist_beam", "double_wishbone")
ODD = NS(wheelbase=1610.0, track_front=1234.0, track_rear=1199.0, spring_rate_front=28.0, cg_height=301.0,
         weight_dist_front=0.44, mass=251.0)


def _hp(**kw):
    import dataclasses
    return dataclasses.replace(Hardpoints.default(), **kw)


def _gk(name):
    from suspension import adapter, topologies
    return adapter.GenericKinematics(topologies.example(name))


def _helpers():
    """name -> a function computing that helper's output."""
    vp, led = _vp(), _ledger()
    return {"metrics": lambda: {m: _r(fc.subsystem_metrics(vp, led, m)) for m in ("heat", "mass", "power", "load", "x")},
           "metrics_empty": lambda: _r(fc.subsystem_metrics(vp, IntegrationLedger(), "heat")),
           "influence": lambda: _r(fc.influence_summary(vp, led, "Pushrod")),
           "influence_empty": lambda: _r(fc.influence_summary(vp, None)),
           "heat_color": lambda: [fc.heat_color(t) for t in (-1.0, 0.0, 0.2, 0.5, 0.75, 1.0, 2.0)],
           "labels": lambda: {k: [fc.part_label(k), fc.is_pickup(k)] for k in
                      ("Tire", "Upright", "front_wing", "pickup:UCA_front", "Pickup UCA front", "Accumulator", "")},
           "corner_labels": lambda: [fc.corner_label(c) for c in ("FL", "FR", "RL", "RR", "X", "")],
           "sub_colors": lambda: [fc.sub_color_key(s) for s in ("chassis", "cooling", "unknown", "")],
           "suggest": lambda: {k: _r(fc.suggest_part_geometry_for(vp, k, led)) for k, *_ in fc.part_catalog()},
           "suggest_noledger": lambda: {k: _r(fc.suggest_part_geometry_for(vp, k)) for k, *_ in fc.part_catalog()},
           "suggest_sub": lambda: {s: _r(fc.suggest_part_geometry(vp, s, led)) for s, *_ in fc.subsystem_catalog()},
           "anchors": lambda: {k: _r(fc.part_anchor(vp, k, led)) for k, *_ in fc.part_catalog()},
           "fit": lambda: [_r(fc.custom_part_fit(vp, p)) for p in (
               {"l_mm": 200, "w_mm": 150, "h_mm": 100, "x_mm": 800, "y_mm": 0, "z_mm": 300},
               {"l_mm": 2000, "w_mm": 1500, "h_mm": 1200, "x_mm": 0, "y_mm": 0, "z_mm": 0},
               {"l_mm": 300, "w_mm": 1150, "h_mm": 100, "x_mm": 800, "y_mm": 0, "z_mm": 1100},
               {"l_mm": 0, "w_mm": None, "h_mm": "", "x_mm": -500, "y_mm": 700, "z_mm": -10}, {})],
           "dims": lambda: [_r(fc.part_dims_from_mesh(s)) for s in (
               {"size_mm": [300.0, 200.0, 100.0]}, {"bbox_min": [0, 0, 0], "bbox_max": [10.0, 20.0, 30.0]}, {})],
           "reconcile": lambda: [_r(fc.reconcile_part(g, r, **kw)) for g, r, kw in (
               ({"l_mm": 300, "w_mm": 200, "h_mm": 100}, {"l_mm": 305, "w_mm": 200, "h_mm": 100}, {}),
               ({"l_mm": 300, "w_mm": 200, "h_mm": 100}, {"l_mm": 400, "w_mm": 150, "h_mm": 100}, {}),
               ({"l_mm": 300, "w_mm": 200, "h_mm": 100}, {"l_mm": 320, "w_mm": 200, "h_mm": 100},
                dict(tol_mm=1.0, tol_frac=0.01)), ({}, {}, {}))],
           **_vp_helpers("odd", ODD, led), **_vp_helpers("empty", NS(), led), **_vp_helpers("none", None, None),
           **_vp_helpers("narrow", NS(wheelbase=1500.0, track_front=600.0, track_rear=560.0, cg_height=250.0), led),
           **_vp_helpers("wide", NS(wheelbase=1700.0, track_front=1500.0, track_rear=1460.0), None),
           "geometry": lambda: _r(_geometry()),
           "introspect": lambda: _r(_introspect()),
           **{f"budget.{n}.{m}": (lambda n=n, m=m: _r(fc.subsystem_metrics(vp, _budget_ledger(n), m)))
              for n in ("cool_fail", "cool_missing", "lv_fail", "lv_default", "zeros", "cap_ledger", "lv_edges",
                        "mass_driver", "mass_exact", "mass_zero", "duct_lateral") for m in ("heat", "power", "mass")},
           **{f"budget.{n}.influence": (lambda n=n: _r(fc.influence_summary(vp, _budget_ledger(n), "Pushrod")))
              for n in ("cool_fail", "lv_fail", "zeros")},
           "extra_keys": lambda: [_r(fc.suggest_part_geometry(ODD, k, led)) for k in (
               "data-acquisition", "unknown", "", "electrics", "Cooling")] + [
               _r(fc.suggest_part_geometry_for(ODD, k, led)) for k in ("data_logger", "nope", "")],
           "reconcile2": lambda: [_r(fc.reconcile_part(g, r, **kw)) for g, r, kw in (
               ({"l_mm": 0.5, "w_mm": 0.4, "h_mm": 0.2}, {"l_mm": 0.5, "w_mm": 0.4, "h_mm": 0.2}, {}),
               ({"l_mm": 300, "w_mm": 200, "h_mm": 100}, {"l_mm": 100, "w_mm": 300, "h_mm": 200}, {}),
               ({"l_mm": 300, "w_mm": 200, "h_mm": 100}, {"l_mm": 302, "w_mm": 200, "h_mm": 100},
                dict(tol_mm=2.0, tol_frac=0.0)),
               ({"l_mm": 300, "w_mm": 200, "h_mm": 100}, {"l_mm": 303, "w_mm": 200, "h_mm": 100},
                dict(tol_mm=0.0, tol_frac=0.01)),
               ({"l_mm": 0.2, "w_mm": 0.2, "h_mm": 0.2}, {"l_mm": 1.5, "w_mm": 0.2, "h_mm": 0.2},
                dict(tol_mm=0.0, tol_frac=1.0)),
               ({"l_mm": 300.04, "w_mm": 200, "h_mm": 100}, {"l_mm": 100, "w_mm": 300.01, "h_mm": 200}, {}),
               ({"l_mm": None, "w_mm": None}, {"l_mm": 5}, {}),
               ({}, {"l_mm": 5.0, "w_mm": 7.0, "h_mm": 9.0}, {}),
               ({"l_mm": 300, "w_mm": 200, "h_mm": 100}, {"l_mm": 103, "w_mm": 298, "h_mm": 201}, {}),
               ({"l_mm": 300.2, "w_mm": 200, "h_mm": 100}, {"l_mm": 100, "w_mm": 300.4, "h_mm": 200},
                dict(tol_mm=0.0, tol_frac=0.0)),
               ({"l_mm": 10, "w_mm": 10, "h_mm": 10}, {"l_mm": 6, "w_mm": 10, "h_mm": 10}, dict(tol_mm=0.0, tol_frac=0.5)),
               ({"l_mm": 0.9, "w_mm": 0.9, "h_mm": 0.9}, {"l_mm": 0.42, "w_mm": 0.9, "h_mm": 0.9},
                dict(tol_mm=0.0, tol_frac=0.5)),
               ({"l_mm": 300, "w_mm": 200, "h_mm": 100}, {"l_mm": 380, "w_mm": 150, "h_mm": 101}, {}))]}


def _introspect():
    """The figure readers on hand-made figures: traces with no customdata, short or malformed customdata, per-corner
    and hidden keys, a repeated pickup, and footprints with missing or empty coordinates."""
    pk = sorted(fc._PICKUP_NAMES)[:2]
    tr = [NS(customdata=None, name="a"), NS(customdata=[], name="b"), NS(customdata=["flat"], name="c"),
          NS(customdata=[["suspension"]], name="d"), NS(customdata=[["suspension", ""]], name="e"),
          NS(customdata=[["suspension", f"{pk[0]}#FL"]], name="f"), NS(customdata=[["suspension", pk[0], "FL"]]),
          NS(customdata=[["suspension", pk[0], "FR"]]), NS(customdata=[["suspension", pk[1]]]),
          NS(customdata=[("chassis", "Driver", "")]), NS(customdata=[["chassis", "helmet"]]),
          NS(customdata=[["", "Tire"]]), NS(customdata=[["suspension", "Tire", "FL"]]),
          NS(customdata=[["suspension", "Tire", "RR"]]), NS(customdata=[["suspension", "Tire#RL", "RL"]]),
          NS(customdata=[["aero", "front_wing"]])]
    fig = NS(data=tr)
    out = {"pickups": fc.pickup_parts(fig), "parts": fc.available_parts(fig),
           "parts_all": fc.available_parts(fig, include_pickups=True),
           "corners": {k: fc.part_corners(fig, k) for k in ("Tire", pk[0], "Driver", "nope")},
           "labels": [fc.part_label(k) for k in ("Tire#FR", "Tire#ZZ", "front_wing", "lowercase", "two words", "Upper",
                                                 pk[0], f"{pk[1]}#RL")],
           "is_pickup": [fc.is_pickup(k) for k in (pk[0], f"{pk[0]}#FL", "Tire", "", 5)]}
    bodies = NS(data=[NS(name="Driver", x=None, y=[1.0], z=[2.0]), NS(name="Driver", x=[1.0], y=None, z=[2.0]),
                      NS(name="Driver", x=[1.0], y=[2.0], z=None), NS(name="Driver", x=[], y=[1.0], z=[1.0]),
                      NS(name="Driver", x=[1.0], y=[], z=[1.0]), NS(name="Driver", x=[1.0], y=[1.0], z=[]),
                      NS(name="Accumulator", x=[0.0, 10.0], y=[-5.0, 5.0], z=[1.0, 4.0]),
                      NS(name="Accumulator", x=[20.0], y=[0.0], z=[2.0]), NS(name=None, x=[1.0], y=[1.0], z=[1.0])])
    out["foot"] = {k: fc.dummy_body_footprint(bodies, k) for k in ("Driver", "Accumulator", "Nope")}
    out["foot_nodata"] = fc.dummy_body_footprint(NS(), "Driver")
    return out


def _budget_ledger(kind):
    """Ledgers whose budgets fail, are undeclared, or fall back, so the heat/power maps flag subsystems."""
    led = IntegrationLedger()
    if kind == "cool_fail":
        led.set(SubsystemInterface(name="cooling", cooling_airflow_cms=0.2, heat_reject_w=50.0))
        led.set(SubsystemInterface(name="powertrain", cooling_airflow_cms=0.3, heat_reject_w=6000.0))
        led.set(SubsystemInterface(name="electrics", cooling_airflow_cms=0.1, heat_reject_w=900.0))
        led.total_cooling_airflow_cms = 0.15
    elif kind == "cool_missing":
        led.set(SubsystemInterface(name="powertrain", cooling_airflow_cms=0.3, heat_reject_w=6000.0))
        led.set(SubsystemInterface(name="brakes", heat_reject_w=0.5))
    elif kind in ("lv_fail", "lv_default"):
        led.set(SubsystemInterface(name="electrics", power_draw_w=300.0, voltage_v=12.0, mass_kg=40.0))
        led.set(SubsystemInterface(name="powertrain", power_draw_w=50.0, voltage_v=400.0, mass_kg=30.0))
        led.set(SubsystemInterface(name="aerodynamics", power_draw_w=30.0, mass_kg=9.0))
        led.set(SubsystemInterface(name="cooling", power_draw_w=80.0, voltage_v=24.0, mass_kg=0.6))
        led.lv_supply_capacity_w = 100.0
        led.target_mass_kg = 60.0
        if kind == "lv_fail":
            led.lv_voltage_v = 12.0
    elif kind == "cap_ledger":              # the airflow budget declared on the ledger only
        led.set(SubsystemInterface(name="powertrain", cooling_airflow_cms=0.3, heat_reject_w=6000.0))
        led.total_cooling_airflow_cms = 0.5
    elif kind == "lv_edges":                # LV users either side of 5 V from a 12 V bus
        for name, v in (("electrics", 12.0), ("cooling", 17.0), ("brakes", 16.9), ("aerodynamics", 17.5),
                        ("data-acquisition", 6.5)):
            led.set(SubsystemInterface(name=name, power_draw_w=100.0, voltage_v=v))
        led.lv_supply_capacity_w = 50.0
        led.lv_voltage_v = 12.0
    elif kind == "mass_driver":             # over the target once the driver allowance is taken off
        led.set(SubsystemInterface(name="chassis", mass_kg=40.0))
        led.set(SubsystemInterface(name="powertrain", mass_kg=20.0))
        led.target_mass_kg = 100.0
        led.includes_driver_kg = 70.0
    elif kind in ("mass_exact", "mass_zero"):
        led.set(SubsystemInterface(name="chassis", mass_kg=40.0))
        led.set(SubsystemInterface(name="powertrain", mass_kg=20.0, power_draw_w=10.0, voltage_v=19.5))
        led.target_mass_kg = 60.0 if kind == "mass_exact" else 0.0
        led.lv_supply_capacity_w = 5.0
    elif kind == "duct_lateral":            # an estimated aero package beside a cooler, and a car off-centre
        led.set(SubsystemInterface(name="cooling", env_x_mm=200.0, heat_reject_w=10.0, mass_kg=6.0, cg_x_mm=900.0,
                                   cg_y_mm=80.0, cg_z_mm=250.0))
        led.set(SubsystemInterface(name="aerodynamics", is_estimate=True, mass_kg=9.0, cg_x_mm=1000.0, cg_y_mm=60.0,
                                   cg_z_mm=400.0))
        led.target_mass_kg = 10.0
    else:
        for name in ("chassis", "brakes", "electrics"):
            led.set(SubsystemInterface(name=name, heat_reject_w=0.0, power_draw_w=0.0, mass_kg=0.0))
    return led


def _geometry():
    """The private mesh and placement helpers on their own, where the figure digests can hide a face index."""
    prof = [(0.0, 0.0), (1.0, 0.1), (0.8, -0.2), (0.2, -0.1)]
    out = {"prism": fc._prism_xsection(prof, [0.0, 100.0, 250.0], [(1.0, 2.0, 0.0, 5.0), (0.5, 1.0, 10.0, 0.0),
                                                                    (2.0, 0.5, -3.0, 7.0)]),
           "ring": fc._ellipse_ring(7), "air": fc._airfoil_section(9, 0.2), "air0": fc._airfoil_section(),
           "box": fc._box(1.0, 2.0, 3.0, 40.0, 20.0, 10.0), "wire": fc._bbox_wire((0, -5, 1), (10, 5, 4)),
           "cyl": fc._cylinder((0, 0, 0), (0, 1, 0), 50.0, 20.0, n=6),
           "cyl_z": fc._cylinder((1, 2, 3), (0, 0, 2.0), 5.0, 8.0, n=5, cap=False),
           "tube": fc._tube((0, 0, 0), (10, 5, 2), 1.5, n=5), "swept": fc._swept_tube([(0, 0, 0), (5, 1, 0), (9, 4, 2)],
                                                                                    1.0, n=4),
           "axis": [fc._units_axis_cfg(t, lo, hi) for t, lo, hi in (("x", 0.0, 1000.0), ("z", -50.0, 5.0))]}
    v = [[0, 0, 0], [100, 0, 0], [0, 50, 0], [0, 0, 20], [100, 50, 20]]
    for am in ("z_up", "y_up", "x_up", "weird"):
        out[f"orient.{am}"] = fc._orient_part_mesh(v, axis_map=am, yaw_deg=30.0, scale=1.5, centre=(10.0, -5.0, 3.0),
                                                   roll_deg=12.0, pitch_deg=-7.0)
    out["orient.default"] = fc._orient_part_mesh(v)
    for mirror in (False, True):
        out[f"xf.{mirror}"] = [fc._corner_transform(p, mirror_y=mirror, lateral_scale=1.1, x_shift=20.0,
                                                    y_center_ref=300.0, size_scale=0.9, z_ground=5.0)
                               for p in ((10.0, 400.0, 50.0), None)]
    hp = Hardpoints.default()
    out["solved"] = fc._solved_corner_points(hp)
    out["solved_drop"] = fc._solved_corner_points(_hp(pushrod_outer=None, rocker_pivot=None), ride_drop_mm=12.5)
    out["solved_int"] = fc._solved_corner_points(_hp(upper_front_inner=(100, 240, 280), tie_rod_inner=(-100, 230, 160)))
    return out


def _vp_helpers(tag, vp, led):
    """The vehicle-driven helpers again on a car whose numbers differ from every default (or that has none)."""
    return {f"{tag}.metrics": lambda: {m: _r(fc.subsystem_metrics(vp, led, m)) for m in ("heat", "mass", "power",
                                                                                         "load")},
            f"{tag}.influence": lambda: _r(fc.influence_summary(vp, led, "MacPherson")),
            f"{tag}.suggest": lambda: {k: _r(fc.suggest_part_geometry_for(vp, k, led)) for k, *_ in fc.part_catalog()},
            f"{tag}.suggest_sub": lambda: {s: _r(fc.suggest_part_geometry(vp, s, led))
                                           for s, *_ in fc.subsystem_catalog()},
            f"{tag}.anchors": lambda: {k: _r(fc.part_anchor(vp, k, led)) for k, *_ in fc.part_catalog()},
            f"{tag}.fit": lambda: [_r(fc.custom_part_fit(vp, p)) for p in (
                {"l_mm": 200, "w_mm": 150, "h_mm": 100, "x_mm": 800, "y_mm": 0, "z_mm": 300},
                {"l_mm": 300, "w_mm": 1150, "h_mm": 100, "x_mm": 800, "y_mm": 600, "z_mm": 1100},
                {"l_mm": 100, "w_mm": 100, "h_mm": 100, "x_mm": 1400, "y_mm": -590, "z_mm": 20})]}


def _everything():
    return {"figures": {k: _digest(f()) for k, f in _figures().items()},
            "helpers": {k: f() for k, f in _helpers().items()}}

def test_every_figure_and_helper():
    if os.environ.get("KINEMATIK_REGOLD"):
        GOLD.write_text(json.dumps(json.loads(json.dumps(_everything(), default=str)), indent=0) + "\n",
                        encoding="utf-8")
    want = json.loads(GOLD.read_text(encoding="utf-8"))
    for sec, cases in (("figures", {k: (lambda f=f: _digest(f())) for k, f in _figures().items()}),
                       ("helpers", _helpers())):
        assert set(cases) == set(want[sec]), sec
        for k, f in cases.items():
            assert json.loads(json.dumps(f(), default=str)) == want[sec][k], (sec, k)
