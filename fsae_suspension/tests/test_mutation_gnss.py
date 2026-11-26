"""Characterization suite for suspension/gnss.py (mutation gate); see tests/_golden.py. Five GPS/IMU nodes (the
bought one as shipped, one with every question answered, a tight one, a bad one and a bare one) through every check
with several cars, buses, rails and tolerances, plus the lever-arm kinematics and the plan's markdown."""
import dataclasses

from suspension import gnss as G
from suspension.daq_plan import BusSpec, Rail
from tests._golden_lazy import check_lazy
from tests._golden import plain as _plain


def plain(x):
    """tests._golden.plain, with sets sorted (their repr follows the hash seed)."""
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: plain(getattr(x, f.name)) for f in dataclasses.fields(x)}
    if isinstance(x, (set, frozenset)):
        return sorted((plain(v) for v in x), key=repr)
    if isinstance(x, dict):
        return {str(k): plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    return _plain(x)


def _specs():
    full = dict(key="full", name="Full node", position_rate_hz=20.0, imu_rate_hz=200.0, can_bitrate_bps=500_000.0,
                can_extended_ids=False, supply_v_min=7.0, supply_v_max=18.0, current_ma=150.0, connector="DT04-4P",
                mating_connector="DT06-4S", external_antenna=True, cost_usd=300.0, velocity_accuracy_ms=0.05,
                velocity_max_ms=80.0, velocity_resolution_ms=0.01, velocity_latency_s=0.05, position_accuracy_m=1.5,
                height_accuracy_m=3.0, heading_accuracy_deg=0.5, heading_resolution_deg=0.1,
                offset_from_cg_m=(0.3, 0.1, -0.2), calibration="bench", owner="DAQ", source="manual")
    tight = dict(full, key="tight", name="Tight node", position_rate_hz=50.0, imu_rate_hz=500.0,
                 velocity_latency_s=0.005, offset_from_cg_m=(0.02, 0.0, 0.01), external_antenna=False,
                 current_ma=40.0, supply_v_min=4.5, supply_v_max=30.0)
    bad = dict(full, key="bad", name="Bad node", position_rate_hz=5.0, imu_rate_hz=20.0, can_bitrate_bps=250_000.0,
               can_extended_ids=True, supply_v_min=13.0, supply_v_max=16.0, current_ma=900.0,
               velocity_latency_s=0.3, offset_from_cg_m=(-0.9, -0.6, 0.4), heading_accuracy_deg=None,
               connector=None, mating_connector=None, external_antenna=None, calibration=None)
    zero = dict(full, key="zero", name="At the CG", offset_from_cg_m=(0.0, 0.0, 0.0), velocity_latency_s=0.0)
    extra = {"hog12": dict(full, key="hog12", supply_v_min=9.0, supply_v_max=16.0, current_ma=1999.5),
             "hog5": dict(full, key="hog5", supply_v_min=4.5, supply_v_max=5.5, current_ma=499.5),
             "edges": dict(full, key="edges", supply_v_min=5.0, supply_v_max=12.0, position_rate_hz=30.0,
                           imu_rate_hz=30.0, can_bitrate_bps=500_001.0),
             "off15": dict(full, key="off15", can_bitrate_bps=500_001.5, supply_v_max=None),
             "slow": dict(full, key="slow", velocity_latency_s=1.0, offset_from_cg_m=(1.0, 0.0, 0.0),
                          name="Node | A\nB", imu_rate_hz=None),
             "half": dict(key="half", name="Half", position_rate_hz=10.0, imu_rate_hz=100.0, can_bitrate_bps=500e3,
                          supply_v_min=7.0, supply_v_max=18.0, current_ma=100.0, connector="DT04", cost_usd=150.0)}
    return {"cots": G.ECUMASTER_GPS_TO_CAN_V2, "full": G.GnssSpec(**full), "tight": G.GnssSpec(**tight),
            "bad": G.GnssSpec(**bad), "zero": G.GnssSpec(**zero), "bare": G.GnssSpec(key="bare", name="Bare"),
            **{k: G.GnssSpec(**v) for k, v in extra.items()}}


def _cars():
    return {"none": None,
            "full": G.VehicleSpec(mass_kg=280.0, wheelbase_m=1.55, cg_to_front_m=0.82, mu_lat=1.6,
                                  speeds_ms=(10.0, 18.0), source="team"),
            "inertia": G.VehicleSpec(mass_kg=250.0, wheelbase_m=1.6, cg_to_front_m=0.7, mu_lat=1.4,
                                     yaw_inertia_kgm2=95.0),
            "partial": G.VehicleSpec(mass_kg=260.0, mu_lat=1.5),
            "no_mu": G.VehicleSpec(mass_kg=260.0, wheelbase_m=1.5, cg_to_front_m=0.75),
            "wb_only": G.VehicleSpec(wheelbase_m=1.5), "cgf_only": G.VehicleSpec(cg_to_front_m=0.8),
            "no_cgf": G.VehicleSpec(mass_kg=260.0, wheelbase_m=1.5, mu_lat=1.5),
            "no_mass": G.VehicleSpec(wheelbase_m=1.5, cg_to_front_m=0.75, mu_lat=1.5),
            "no_wb": G.VehicleSpec(mass_kg=260.0, cg_to_front_m=0.75, mu_lat=1.5, yaw_inertia_kgm2=90.0),
            "i_no_mass": G.VehicleSpec(wheelbase_m=1.5, cg_to_front_m=0.75, mu_lat=1.5, yaw_inertia_kgm2=90.0)}


def _buses():
    return {"default": BusSpec(), "fast_ext": BusSpec(name="DAQ CAN", bitrate_bps=1_000_000.0, extended_ids=True),
            "slow_tight": BusSpec(name="slow", bitrate_bps=125_000.0, load_ok=0.05, load_warn=0.1, load_fail=0.2)}


def _rails():
    return {"default": None, "small5": {"5V": Rail("5V", 5.0, capacity_ma=100.0, fuse_a=0.2)},
            "nofuse12": {"12V": Rail("12V", 12.0, capacity_ma=1000.0)},
            "both": {"5V": Rail("5V", 5.0, capacity_ma=500.0, fuse_a=1.0),
                     "12V": Rail("12V", 12.0, capacity_ma=160.0, fuse_a=0.15)}}


def _car(v):
    if v is None:
        return None
    return {"rear": v.cg_to_rear_m(), "Iest": v.yaw_inertia_estimate_kgm2(), "I": v.yaw_inertia(),
            "alpha": v.max_yaw_accel_rads2(), "ay": v.max_lat_accel_ms2(), "missing": v.missing_fields(),
            "man": plain(v.manoeuvres()), "rates": [m.yaw_rate_rads() for m in (v.manoeuvres() or ())]}


def _everything():
    specs, cars, buses, rails = _specs(), _cars(), _buses(), _rails()
    for k, s in specs.items():
        yield f"{k}.spec", {"un": s.unanswered_measurements(), "done": s.measurement_completeness(),
                            "arm": s.lever_arm_m(), "doc": plain(G.documentation_findings(s)),
                            "lat": plain(G.latency_findings(s)), "lat5": plain(G.latency_findings(s, reference_speed_ms=5.0)),
                            "rate": plain(G.rate_findings(s)), "rate60": plain(G.rate_findings(s, vehicle_bandwidth_hz=60.0)),
                            "can": plain(G.can_map(s)), "can100": plain(G.can_map(s, base_id=0x100)),
                            "sensors": plain(G.to_sensor_specs(s))}
        for b, bus in buses.items():
            yield f"{k}.bus.{b}", plain(G.bus_findings(s, bus))
        for r, rl in rails.items():
            if rl is not None:
                yield f"{k}.power.{r}", plain(G.power_findings(s, rl))
        for c, car in cars.items():
            for tol in (0.05, 0.2, 0.001):
                yield f"{k}.mount.{c}.{tol}", plain(G.mounting_findings(s, tolerance_g=tol, vehicle=car))
        yield f"{k}.mount.manoeuvres", plain(G.mounting_findings(s, manoeuvres=(
            G.Manoeuvre("hairpin", 8.0, 7.5, 3.0, "tight"), G.Manoeuvre("straight", 30.0, None, 0.0, "launch"),
            G.Manoeuvre("slalom", 14.0, 0.0, 6.0, "zero radius"))))
        for name, kw in (("default", {}), ("car", dict(vehicle=cars["full"], bus=buses["fast_ext"])),
                         ("tight", dict(rails=rails["both"], tolerance_g=0.01, vehicle_bandwidth_hz=40.0,
                                        reference_speed_ms=30.0, base_id=0x300, bus=buses["slow_tight"]))):
            p = G.plan_gnss(s, **kw)
            yield f"{k}.plan.{name}", {"plan": plain(p), "blocking": plain(p.blocking()),
                                       "open": plain(p.open_questions()), "bps": p.bus_bits_per_second(),
                                       "md": p.to_markdown()}
    for c, car in cars.items():
        yield f"car.{c}", _car(car)
    for off in ((0.3, 0.1, 0.0), (-0.2, 0.4, 1.0), (0.0, 0.0, 0.0)):
        for w, a in ((1.5, 0.0), (0.8, 4.0), (0.0, -2.0)):
            r = G.lever_arm_error(off, w, a)
            yield f"arm.{off}.{w}.{a}", {"r": plain(r), "g": r.a_err_g(), "mag": r.magnitude_g()}
    yield "arm.default_alpha", plain(G.lever_arm_error((0.5, 0.2, 0.0), 1.0))
    yield "compare.default", plain(G.compare_options())
    yield "compare.diy", plain(G.compare_options(specs["full"], specs["bad"]))
    yield "compare.cots_tight", plain(G.compare_options(cots=specs["tight"], diy=specs["bare"]))
    yield "compare.low", plain(G.compare_options(cots=specs["bare"], diy=specs["half"]))
    yield "compare.nomating", plain(G.compare_options(cots=specs["bad"], diy=specs["full"]))
    yield "provenance", plain(G.PROVENANCE)


def test_every_node_through_every_check():
    check_lazy("gnss", _everything())
