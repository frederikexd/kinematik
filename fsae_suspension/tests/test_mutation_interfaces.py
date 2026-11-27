"""Characterization suite for suspension/interfaces.py (mutation gate); see tests/_golden.py. Every check at and
either side of its threshold: the mass budget with and without a driver allowance or missing masses, CG sanity and
lateral offset, envelopes from the ledger or the chassis, cooling airflow missing, short, exact and ample (with
the cooling package's own figure), LV draw by voltage, HV mismatch, driveline torque from the ledger or the
suspension, mount loads into either carrier, the estimate pass; the environment push onto specs that inherit,
opt out or have no temperature; the round trips, summaries, change diffs and the Markdown contract."""
from types import SimpleNamespace

from suspension import interfaces as I
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _full(**kw):
    led = I.blank_ledger()
    for k, v in kw.items():
        setattr(led, k, v)
    for name, m, z in (("suspension", 35, 180), ("chassis", 32, 300), ("powertrain", 40, 270),
                       ("aerodynamics", 12, 400), ("brakes", 8, 200), ("cooling", 6, 250), ("electrics", 10, 220),
                       ("data-acquisition", 3, 240)):
        led.set(I.SubsystemInterface(name=name, mass_kg=m, cg_x_mm=900 + m, cg_y_mm=0.0, cg_z_mm=z,
                                     is_estimate=False))
    return led


def _with(led, name, **kw):
    it = led.get(name) or I.SubsystemInterface(name=name)
    for k, v in kw.items():
        setattr(it, k, v)
    led.set(it)
    return led


def _checks(led):
    f = led.check_all()
    return {"findings": [x.as_dict() for x in f], "summary": I.summarize(f), "roll": plain(led.mass_rollup()),
            "susp": [x.check for x in I.findings_for(f, "suspension")]}


class Spec:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Inlet(Spec):
    ENV_CHANNEL = "cooling_inlet"


def _cases():
    yield "blank", _checks(I.blank_ledger())
    yield "empty", _checks(I.IntegrationLedger())
    for target in (146.0, 145.9, 139.05, 139.04, 160.0):
        yield f"mass.{target}", _checks(_full(target_mass_kg=target))
    yield "mass.driver", _checks(_full(target_mass_kg=216.0, includes_driver_kg=70.0))
    yield "mass.negdriver", _checks(_full(target_mass_kg=146.0, includes_driver_kg=-5.0))
    part = _full()
    part.interfaces.pop("brakes")
    _with(part, "cooling", mass_kg=None)
    yield "mass.partial", _checks(part)
    yield "cg.missing", _checks(_with(_full(), "chassis", cg_y_mm=None))
    for y in (25.0, 25.1, -30.0):
        led = _full()
        for it in led.interfaces.values():
            it.cg_y_mm = y
        yield f"cg.lateral.{y}", _checks(led)
    yield "env.ledger.fit", _checks(_with(_full(chassis_envelope_mm=(500, 400, 300)), "powertrain", env_x_mm=500,
                                          env_y_mm=400, env_z_mm=300))
    yield "env.ledger.over", _checks(_with(_full(chassis_envelope_mm=(500, 400, 300)), "powertrain", env_x_mm=500,
                                           env_y_mm=400.1, env_z_mm=300))
    yield "env.chassis", _checks(_with(_with(_full(), "chassis", env_x_mm=600, env_y_mm=500, env_z_mm=400), "cooling",
                                       env_x_mm=700, env_y_mm=10, env_z_mm=10))
    yield "env.chassis.partial", _checks(_with(_full(), "chassis", env_x_mm=600, env_y_mm=500))
    yield "env.partial_part", _checks(_with(_full(chassis_envelope_mm=(500, 400, 300)), "brakes", env_x_mm=900))
    for have, own in ((0.0, None), (0.05, None), (0.08, None), (0.0, 0.08), (0.05, 0.2), (0.1, 0.0)):
        led = _with(_with(_full(total_cooling_airflow_cms=have), "powertrain", cooling_airflow_cms=0.05), "electrics",
                    cooling_airflow_cms=0.03)
        _with(led, "cooling", cooling_airflow_cms=own)
        yield f"cool.{have}.{own}", _checks(led)
    yield "cool.only_cooling", _checks(_with(_full(), "cooling", cooling_airflow_cms=0.5))
    duct = _with(_with(_full(), "cooling", env_x_mm=200), "aerodynamics", is_estimate=True)
    yield "cool.duct", _checks(duct)
    yield "cool.duct.confirmed", _checks(_with(_with(_full(), "cooling", env_x_mm=200), "aerodynamics",
                                              is_estimate=False))
    for v_e, draw in ((None, 300.0), (24.0, 300.0), (28.99, 300.0), (29.0, 300.0), (19.01, 600.0), (12.0, 700.0)):
        led = _with(_with(_full(), "electrics", power_draw_w=draw, voltage_v=v_e), "data-acquisition",
                    power_draw_w=0.5)
        yield f"lv.{v_e}.{draw}", _checks(led)
    yield "lv.over", _checks(_with(_full(lv_supply_capacity_w=100.0), "electrics", power_draw_w=100.001))
    yield "lv.exact", _checks(_with(_full(lv_supply_capacity_w=100.0), "electrics", power_draw_w=100.0))
    for v in (60.0, 60.1, 380.0, 379.9, 420.0, 420.1, 400.0):
        yield f"hv.{v}", _checks(_with(_full(), "powertrain", voltage_v=v))
    yield "torque.none", _checks(_with(_full(), "powertrain", peak_torque_nm=None))
    yield "torque.nolimit", _checks(_with(_full(), "powertrain", peak_torque_nm=200.0))
    for lim in (200.0, 199.0):
        yield f"torque.ledger.{lim}", _checks(_with(_full(driveline_torque_limit_nm=lim), "powertrain",
                                                    peak_torque_nm=200.0))
    yield "torque.susp", _checks(_with(_with(_full(), "powertrain", peak_torque_nm=250.0), "suspension",
                                       peak_torque_nm=240.0))
    yield "torque.susp0", _checks(_with(_with(_full(), "powertrain", peak_torque_nm=250.0), "suspension",
                                        peak_torque_nm=0.0))
    for design, carrier, load in ((None, "suspension", 900.0), (1000.0, "suspension", 1000.0),
                                  (1000.0, "suspension", 1000.1), (1000.0, "chassis", 5000.0), (1000.0, None, 5000.0)):
        led = _with(_full(upright_design_load_n=design), "brakes", mount_load_n=load, mounts_on=carrier,
                    mount_points=4 if carrier else None)
        yield f"mount.{design}.{carrier}.{load}", _checks(led)
    yield "estimates", _checks(_with(_with(_full(), "brakes", is_estimate=True), "cooling", is_estimate=True))
    est_blank = I.blank_ledger()
    est_blank.get("brakes").is_estimate = True
    yield "estimates.undeclared", _checks(est_blank)
    led = _full(ambient_c=35.0, cooling_inlet_c=55.0)
    specs = [Spec(ambient_c=20.0), Spec(T_ambient_c=21.0), Inlet(ambient_c=22.0), Spec(ambient_c=10.0,
                                                                                         ambient_is_local=True),
             Spec(ambient_c=35.0, ambient_is_local=True), Inlet(T_ambient_c=1.0, ambient_is_local=True),
             Spec(other=1.0), None, SimpleNamespace(ambient_c=5.0, T_ambient_c=6.0)]
    out = led.apply_environment(*specs)
    yield "env.push", [[x.as_dict() for x in out], [vars(s) for s in specs if s is not None]]
    yield "env.channels", [led.environment_c(), led.environment_c("cooling_inlet"), led.environment_c("x"),
                           _full(ambient_c=30.0).environment_c("cooling_inlet")]
    rt = I.IntegrationLedger.from_dict(_with(_full(chassis_envelope_mm=(1, 2, 3)), "aerodynamics",
                                             downforce_n_at_v=(600.0, 15.0), drag_n_at_v=[200.0, 15.0],
                                             env_origin_mm=[1, 2, 3]).as_dict())
    yield "roundtrip", plain(rt.as_dict())
    yield "fromdict.list", plain(I.IntegrationLedger.from_dict({"chassis_envelope_mm": [1, 2, 3], "junk": 1,
                                                                "interfaces": None}).as_dict())
    yield "iface", [I.SubsystemInterface.from_dict({"name": "x", "drag_n_at_v": [1, 2], "junk": 5,
                                                    "env_origin_mm": (1, 2, 3)}).as_dict(),
                    I.SubsystemInterface(name="y", mass_kg=0.0, notes="n").declared_fields(),
                    I.SubsystemInterface(name="y", mass_kg=0.0, voltage_v=12.0).numeric_values()]
    yield "get", [I.blank_ledger().get("nope"), plain(I.blank_ledger().get("brakes"))]
    yield "fmt", [I._fmt_val(f, v) for f, v in (("mass_kg", None), ("mass_kg", 12.5), ("mass_kg", 12),
                                                 ("downforce_n_at_v", (600.0, 15.0)), ("drag_n_at_v", [1.5, 2]),
                                                 ("mounts_on", "chassis"), ("mount_points", 4), ("other", 1.0))]
    new = I.SubsystemInterface(name="aero", mass_kg=14.5, downforce_n_at_v=(600.0, 15.0), is_estimate=False,
                               rationale=" CFD run 4 ")
    yield "diff", [I.diff_interfaces(None, new), I.diff_interfaces({}, new),
                   I.diff_interfaces(new.as_dict(), new),
                   I.diff_interfaces(dict(new.as_dict(), mass_kg=12.0, is_estimate=True, rationale="CFD run 4"), new),
                   I.diff_interfaces(dict(new.as_dict(), rationale="old"), new),
                   I.diff_interfaces(new.as_dict(), I.SubsystemInterface(name="aero", rationale="  ")),
                   I.diff_interfaces(dict(new.as_dict(), is_estimate=False),
                                     I.SubsystemInterface(name="aero", is_estimate=True))]
    yield "summarize", [I.summarize([]), I.summarize(_full().check_all())]
    yield "labels", [I.SUBSYSTEMS, I.FIELD_LABELS, [s.value for s in I.Severity]]
    tup = I.SubsystemInterface.from_dict({"name": "x", "env_origin_mm": [1, 2, 3], "downforce_n_at_v": [600, 15],
                                          "drag_n_at_v": [200, 15]})
    yield "iface.tuples", [type(tup.env_origin_mm).__name__, type(tup.downforce_n_at_v).__name__,
                           type(tup.drag_n_at_v).__name__]
    led = _full(ambient_c=31.0)
    yield "env.default", [led.environment_c(), led.environment_c("cooling_inlet")]

    class Sub:
        ENV_CHANNEL = "ambient"

        def __init__(self, t, local=None):
            self.ambient_c = t
            if local is not None:
                self.ambient_is_local = local
    subs = [Sub(31.5), Sub(31.0), Sub(20.0, True), Sub(25.0, False), None, Sub(31.5, True)]
    yield "env.apply", [[str(f) for f in led.apply_environment(*subs)], [x.ambient_c for x in subs if x is not None]]
    tiny = I.blank_ledger()
    tiny.set(I.SubsystemInterface(name="chassis", mass_kg=0.5, cg_x_mm=1.0, cg_y_mm=2.0, cg_z_mm=3.0))
    yield "rollup.tiny", plain(tiny.mass_rollup())
    fd = I.IntegrationLedger.from_dict({"interfaces": None, "target_mass_kg": 200.0})
    yield "fromdict.none", [fd.target_mass_kg, sorted(fd.interfaces)]
    exact = _full(target_mass_kg=146 * 1.05 / 1.05, includes_driver_kg=0.5)
    yield "mass.driver_small", [str(f) for f in exact._check_mass_budget()]
    for tgt in (138.0, 140.0, 146.0 / 1.05):
        yield f"mass.edge.{tgt:.4f}", [str(f) for f in _full(target_mass_kg=tgt)._check_mass_budget()]
    gap = _full(total_cooling_airflow_cms=0.01)
    gap.interfaces.pop("aerodynamics")
    _with(gap, "brakes", cooling_airflow_cms=0.2)
    yield "thermal.gap", [str(f) for f in gap._check_thermal()]


def _docs():
    yield "md.blank", I.build_interface_markdown(I.blank_ledger())
    rich = _with(_with(_full(chassis_envelope_mm=(1500, 600, 500), driveline_torque_limit_nm=400.0,
                             total_cooling_airflow_cms=0.25), "aerodynamics", rationale=" CFD ", owner="Ana",
                       updated_on="2026-05-01", updated_by="Bo", notes=" check ", downforce_n_at_v=(600.0, 15.0),
                       is_estimate=True), "brakes", owner="Cy", updated_on="2026-05-02")
    yield "md.rich", I.build_interface_markdown(rich, team_name="Team X", season="2026")
    part = _full()
    part.interfaces.pop("brakes")
    _with(part, "chassis", cg_z_mm=None)
    yield "md.partial", I.build_interface_markdown(part, season="s")
    yield "md.cg_estimates", I.build_interface_markdown(_with(_full(), "brakes", is_estimate=True), team_name="T")
    env = _with(_full(total_cooling_airflow_cms=0.0), "chassis", env_origin_mm=(1.0, 2.0, 3.0))
    env.interfaces.pop("brakes")
    _with(env, "cooling", cooling_airflow_cms=None)
    _with(env, "powertrain", cooling_airflow_cms=0.3, peak_torque_nm=900.0)
    env.driveline_torque_limit_nm = 100.0
    yield "md.env_origin", I.build_interface_markdown(env)

    class F:
        def __init__(self, sev, msg, subs):
            self.severity, self.message, self.subsystems = sev, msg, subs
    fake = [F(I.Severity.OK, "ok", ["a"]), F(I.Severity.FAIL, "bad", []), F(I.Severity.WARN, "hmm", ["b", "c"]),
            F(I.Severity.MISSING, "where", ["d"]), F(I.Severity.INFO, "fyi", [])]
    led2 = _full()
    led2.check_all = lambda: fake
    yield "md.order", I.build_interface_markdown(led2).split("---")[-2]


def test_every_interface_check():
    check_lazy("interfaces", _cases())


def test_the_interface_contract(monkeypatch):
    class _D:
        @staticmethod
        def today():
            return SimpleNamespace(isoformat=lambda: "2026-01-02")
    monkeypatch.setattr(I, "_dt", SimpleNamespace(date=_D))
    check_lazy("interfaces_md", _docs())
