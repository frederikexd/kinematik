"""suspension/tire_cosim_driver.py: the staggered co-simulation contract. Each corner's tyre advances exactly once per
macro-step, from the wheel motion the vehicle solver computed (slip, load, camber with its side, wheel-centre
velocity at the corner, wheel travel, the step length), its forces are held as the next step's input, a backend that
raises holds its previous force and says so, and a failed or unknown manoeuvre returns a message instead of raising.
Stand-in backends and a stand-in solver make every hand-off visible."""
import numpy as np
import pytest

from suspension import tire_cosim_driver as cd
import importlib
from suspension.tire_cosim import TireOutput, WheelState


def _transient():
    """The module run_cosim_maneuver's relative import resolves to now (other suites swap sys.modules entries)."""
    return importlib.import_module("suspension.transient")


class Tire:
    """Fy = 1000 alpha + Fz / 10, Fx = 7 per step taken; records every state it is handed."""

    def __init__(self, fail_step=False, fail_reset=False, warn=(), status="ref backend"):
        self.seen, self.resets, self.fail_step, self.fail_reset, self.warn, self.st = [], [], fail_step, fail_reset, \
            list(warn), status
        self.synth = ["gas_temp"]

    def reset(self, init=None):
        if self.fail_reset:
            raise OSError("no licence")
        self.resets.append(init)

    def step(self, ws):
        if self.fail_step:
            raise ValueError("diverged")
        self.seen.append(ws)
        return TireOutput(Fx=7.0 * len(self.seen), Fy=1000.0 * ws.alpha + ws.Fz / 10, Fz=ws.Fz, synthesized=list(self.synth))

    def warnings(self):
        return self.warn

    def provenance(self):
        st = self.st

        class P:
            def status(self):
                if st is None:
                    raise RuntimeError
                return st
        return P()


def _factory(**kw):
    made = []

    def f():
        t = Tire(**kw)
        made.append(t)
        return t
    return f, made


def test_the_corner_set_holds_and_advances_the_forces():
    f, made = _factory()
    cs = cd.CosimCornerSet(backend_factory=f)
    assert len(made) == 4 and cs.corners == made and cs.warnings == []
    assert [x.tolist() for x in cs.forces()] == [[0.0] * 4] * 3
    ws = [WheelState(alpha=0.01 * (i + 1), Fz=1000.0 + 100 * i) for i in range(4)]
    made[1].synth = ["only FR"]
    outs = cs.advance(ws)
    assert [m.seen for m in made] == [[w] for w in ws]
    Fx, Fy, Fz = cs.forces()
    assert Fx.tolist() == [7.0] * 4 and Fy == pytest.approx([1000 * 0.01 * (i + 1) + 100 + 10 * i for i in range(4)])
    assert Fz.tolist() == [1000.0, 1100.0, 1200.0, 1300.0] and [o.Fz for o in outs] == Fz.tolist()
    assert cs.absent_channels() == ["gas_temp"] and cs.status() == "ref backend"
    init = [WheelState(Fz=i) for i in range(4)]
    cs.reset(init)
    assert [m.resets for m in made] == [[init[i]] for i in range(4)] and cs.forces()[0].tolist() == [0.0] * 4
    cs.reset()
    assert [m.resets[-1] for m in made] == [None] * 4
    from suspension.tire_cosim import default_structural_tire
    assert all(type(c).__name__ == type(default_structural_tire()).__name__ for c in cd.CosimCornerSet().corners)   # the default
    assert (cd.FL, cd.FR, cd.RL, cd.RR, cd.CORNER_NAMES) == (0, 1, 2, 3, ("FL", "FR", "RL", "RR"))


def test_faults_hold_the_previous_force_and_are_reported_once():
    f, made = _factory()
    cs = cd.CosimCornerSet(backend_factory=f)
    ws = [WheelState(alpha=0.02, Fz=900.0)] * 4
    cs.advance(ws)
    made[2].fail_step = True
    made[0].warn = ["carcass soft", "carcass soft"]
    cs.advance(ws)
    cs.advance(ws)
    assert cs.forces()[0].tolist() == [21.0, 21.0, 7.0, 21.0]                     # RL held its first-step force
    assert cs.warnings == ["RL step raised (ValueError); held previous force.", "FL: carcass soft"]
    made[1].fail_reset = True
    cs.reset()
    assert cs.warnings[-1] == "FR reset failed (OSError)."
    made[0].st = None
    assert cs.status() == "unknown backend"
    cs._last = []
    assert cs.absent_channels() == []


class FakeSolver:
    """A solver whose algebraic block reports slips and loads, and whose run calls it at known times."""
    times = (0.002, 0.01, 0.01, 0.03)

    def __init__(self, veh, params=None):
        self.veh, self.params = veh, params
        self.p = type("P", (), {"dt": 0.005, "corner_xy": lambda s: (np.array([0.8, 0.8, -0.7, -0.7]),
                                                                      np.array([0.6, -0.6, 0.58, -0.58]))})()
        self._cam_f, self._cam_r = 0.02, 0.01
        self.calls = []

    def algebraic(self, t, y, driver, roadinp):
        self.calls.append((t, np.asarray(y).tolist(), driver, roadinp))
        return {"alpha_lag": np.array([0.01, 0.02, 0.03, 0.04]), "Fz": np.array([800.0, 900.0, 1000.0, 1100.0]),
                "Fx": np.zeros(4), "Fy": np.zeros(4)}

    def run(self, t_end, driver, road, u0):
        y = np.arange(20, dtype=float) * 0.1 + 1.0
        self.out = [self.algebraic(t, y, driver, road) for t in self.times]
        res = type("R", (), {})()
        res.t, res.Fx, res.Fy, res.Fz, res.meta, res.warnings = np.array(self.times), "fx", "fy", "fz", {}, ["solver"]
        res.args = (t_end, driver, road, u0)
        return res


def test_the_manoeuvre_feeds_each_tyre_the_solvers_wheel_motion(monkeypatch):
    built = {}
    monkeypatch.setattr(_transient(), "TransientSolver", lambda veh, params=None: built.setdefault(
        "sim", FakeSolver(veh, params)))
    monkeypatch.setattr(_transient(), "snap_oversteer_maneuver", lambda **kw: ("drv", "road", 1.5, 14.0,
                                                                            f"snap {kw}"))
    f, made = _factory(warn=["hot"])
    out = cd.run_cosim_maneuver("VEH", "snap_oversteer", backend_factory=f, params="PRM", amp=3)
    sim = built["sim"]
    assert (sim.veh, sim.params) == ("VEH", "PRM") and out["result"].args == (1.5, "drv", "road", 14.0)
    assert out["result"].meta == {"maneuver": "snap {'amp': 3}", "cosim_backend": "ref backend"}
    y = np.arange(20, dtype=float) * 0.1 + 1.0
    u, v, r = y[0], y[1], y[2]
    x_i, y_i = sim.p.corner_xy()
    dts = [0.002, 0.008, 0.005, 0.02]                    # from t = 0; a repeated time falls back to dt
    assert sim.calls == [(t, y.tolist(), "drv", "road") for t in FakeSolver.times]
    for i, m in enumerate(made):
        assert len(m.seen) == 4 and m.resets == [None]
        for k, ws in enumerate(m.seen):
            assert ws.alpha == pytest.approx(0.01 * (i + 1)) and ws.kappa == 0.0
            assert ws.Fz == 800.0 + 100 * i and ws.dt == pytest.approx(dts[k])
            assert ws.gamma == pytest.approx((0.02 if i < 2 else 0.01) * np.sign(y_i[i]))
            assert (ws.v_x, ws.v_y) == pytest.approx((u - r * y_i[i], v + r * x_i[i]))
            assert (ws.z_wheel, ws.zdot_wheel) == pytest.approx((y[12 + i], y[16 + i]))
    for k, A in enumerate(sim.out):
        assert A["Fx"].tolist() == [7.0 * (k + 1)] * 4
        assert A["Fy"] == pytest.approx([1000 * 0.01 * (i + 1) + 80 + 10 * i for i in range(4)])
    th = out["tire_history"]
    assert (th.t.tolist(), th.Fx, th.Fy, th.Fz) == (list(FakeSolver.times), "fx", "fy", "fz")
    assert th.absent_channels == ["gas_temp"] and th.backend_status == "ref backend" == out["backend_status"]
    assert set(th.warnings) == {"solver", "FL: hot", "FR: hot", "RL: hot", "RR: hot"} and out["warnings"] is th.warnings
    assert th.carcass_deflection_m is None and th.gas_temp_c is None and th.tread_temp_c is None


def test_every_named_manoeuvre_and_the_failures(monkeypatch):
    seen = []
    for name in ("step_steer", "snap_oversteer", "brake_to_throttle", "curb_strike"):
        monkeypatch.setattr(_transient(), f"{name}_maneuver", lambda _n=name, **kw: seen.append(_n) or (
            "d", "r", 0.1, 10.0, _n))
    monkeypatch.setattr(_transient(), "TransientSolver", FakeSolver)
    f, _ = _factory()
    for name in ("step_steer", "snap_oversteer", "brake_to_throttle", "curb_strike"):
        assert cd.run_cosim_maneuver("V", name, backend_factory=f)["result"].meta["maneuver"] == name
    assert seen == ["step_steer", "snap_oversteer", "brake_to_throttle", "curb_strike"]
    assert cd.run_cosim_maneuver("V", backend_factory=f)["result"].meta["maneuver"] == "step_steer"
    assert cd.run_cosim_maneuver("V", "donut") == dict(result=None, tire_history=None, backend_status="",
                                                       warnings=["unknown manoeuvre 'donut'"])

    def boom(**kw):
        raise ValueError("bad speed")
    monkeypatch.setattr(_transient(), "curb_strike_maneuver", boom)
    assert cd.run_cosim_maneuver("V", "curb_strike") == dict(
        result=None, tire_history=None, backend_status="",
        warnings=["cosim manoeuvre 'curb_strike' failed (ValueError: bad speed)"])


def test_the_history_defaults():
    h = cd.CosimTireHistory(t=np.zeros(1), Fx=np.zeros(1), Fy=np.zeros(1), Fz=np.zeros(1))
    assert (h.carcass_deflection_m, h.gas_temp_c, h.tread_temp_c, h.absent_channels, h.backend_status,
            h.warnings) == (None, None, None, [], "", [])
