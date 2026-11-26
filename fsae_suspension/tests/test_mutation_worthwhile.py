"""Characterization suite for suspension/worthwhile.py (mutation gate); see tests/_golden.py. Assumptions on every
reconciled field, in and out of tolerance (and exactly on it), uncheckable ones, the largest-contributor rule for
mass and CG height (with an assumer that is itself the largest), the reconciled vehicle with and without a driver
allowance or declared data, the event scoring against a reference (missing and zero times), and full assessments:
blocked by a contradiction, by missing physics input and by a non-fit, and buildable lighter, heavier and estimated."""
from suspension import worthwhile as W
from suspension.dynamics import VehicleParams
from suspension.interfaces import SubsystemInterface, blank_ledger
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _ledger(pt_mass=40.0, driver_kg=70.0, target=300.0, estimate=None, z_pt=270):
    led = blank_ledger()
    led.target_mass_kg = target
    led.includes_driver_kg = driver_kg
    for name, m, z in (("suspension", 35, 180), ("chassis", 32, 300), ("powertrain", pt_mass, z_pt),
                       ("aerodynamics", 12, 400), ("brakes", 8, 200), ("cooling", 6, 250), ("electrics", 10, 220),
                       ("data-acquisition", 3, 240)):
        led.set(SubsystemInterface(name=name, mass_kg=m, cg_x_mm=900 + m, cg_y_mm=m - 20, cg_z_mm=z,
                                   is_estimate=(name == estimate)))
    return led


def _verdict(v):
    return {"buildable": v.buildable, "blocking": plain(v.blocking), "findings": plain(v.findings),
            "paper": plain(v.paper_points), "real": plain(v.real_points), "delta": plain(v.points_delta),
            "any_estimate": v.any_estimate, "text": v.verdict_text, "total": v.total_delta()}


def _cases():
    led = _ledger()
    roll = led.mass_rollup()
    real = {"total_mass_kg": roll["total_kg"], "cg_x_mm": roll["cg_mm"][0], "cg_y_mm": roll["cg_mm"][1],
            "cg_z_mm": roll["cg_mm"][2]}
    yield "roll", plain(roll)
    for field, x in real.items():
        for by in ("suspension", "powertrain", "aerodynamics"):
            asm = [W.Assumption(by, field, x), W.Assumption(by, field, x + 5.0, tol=5.0),
                   W.Assumption(by, field, x + 5.1, tol=5.0), W.Assumption(by, field, x - 12.0, about="aero", rationale="r"),
                   W.Assumption(by, field, x + 0.5)]
            yield f"assume.{field}.{by}", plain(W.check_assumptions(led, asm))
    yield "assume.other", plain(W.check_assumptions(led, [W.Assumption("brakes", "wheelbase_mm", 1530.0),
                                                          W.Assumption("brakes", "cg_x_mm", 0.0)]))
    yield "assume.blank", plain(W.check_assumptions(blank_ledger(), [W.Assumption("s", f, 1.0) for f in real]))
    yield "culprit", [W._largest_contributor(led, f) for f in ("total_mass_kg", "cg_z_mm", "cg_x_mm", "wheelbase", "")]
    tall = _ledger(pt_mass=20, z_pt=900)
    yield "culprit.tall", [W._largest_contributor(tall, "cg_z_mm"), W._largest_contributor(tall, "total_mass_kg")]
    partial = blank_ledger()
    partial.set(SubsystemInterface(name="chassis", mass_kg=None))
    partial.set(SubsystemInterface(name="powertrain", mass_kg=10.0, cg_z_mm=None))
    yield "culprit.partial", [W._largest_contributor(partial, "cg_z_mm"), W._largest_contributor(blank_ledger(),
                                                                                                "total_mass_kg")]
    base = VehicleParams(mass=999.0, cg_height=777.0)
    for name, lg in (("full", _ledger()), ("nodriver", _ledger(driver_kg=0.0)), ("negdriver", _ledger(driver_kg=-5.0)),
                     ("blank", blank_ledger())):
        v = W.vehicle_from_ledger(lg, base=base)
        yield f"vehicle.{name}", [v.p.mass, v.p.cg_height]
    yield "vehicle.default", W.vehicle_from_ledger(blank_ledger()).p.mass
    yield "score", [W._score_against_reference({"a": 10.0, "b": None, "c": 5.0, "d": 4.0},
                                               {"a": 9.0, "b": 3.0, "c": None, "d": 0.0}),
                    W._score_against_reference({}, {})]
    paper = VehicleParams(mass=186.0, cg_height=270.0)
    for name, lg, asm, pb in (
            ("contradiction", _ledger(pt_mass=95), [W.Assumption("suspension", "total_mass_kg", 100.0, tol=5.0)], None),
            ("missing", blank_ledger(), None, None), ("heavy", _ledger(pt_mass=100), None, paper),
            ("light", _ledger(pt_mass=10), [W.Assumption("chassis", "cg_z_mm", 260.0, tol=100.0)], paper),
            ("estimate", _ledger(estimate="aerodynamics"), None, VehicleParams(mass=250.0, cg_height=270.0)),
            ("default_paper", _ledger(), None, None)):
        yield f"assess.{name}", _verdict(W.assess(lg, assumptions=asm, paper_baseline=pb))
    nofit = _ledger()
    nofit.chassis_envelope_mm = (500, 400, 300)
    nofit.set(SubsystemInterface(name="powertrain", mass_kg=40, cg_x_mm=900, cg_y_mm=0, cg_z_mm=270, env_x_mm=900,
                                 env_y_mm=400, env_z_mm=300, is_estimate=False))
    yield "assess.nofit", _verdict(W.assess(nofit))
    nocg = blank_ledger()
    nocg.set(SubsystemInterface(name="chassis", mass_kg=30.0))
    yield "assess.nocg", _verdict(W.assess(nocg))
    yield "assess.laps", _verdict(W.assess(_ledger(pt_mass=60), paper_baseline=paper, endurance_laps=2))
    one = blank_ledger()
    one.set(SubsystemInterface(name="brakes", mass_kg=0.5, cg_z_mm=1.0))
    neg = blank_ledger()
    neg.set(SubsystemInterface(name="brakes", mass_kg=-1.5))
    tie = blank_ledger()
    tie.set(SubsystemInterface(name="brakes", mass_kg=5.0))
    tie.set(SubsystemInterface(name="chassis", mass_kg=5.0))
    yield "culprit.small", [W._largest_contributor(one, "total_mass_kg"), W._largest_contributor(one, "cg_z_mm"),
                            W._largest_contributor(neg, "total_mass_kg"), W._largest_contributor(tie, "total_mass_kg")]
    yield "score.small_ref", W._score_against_reference({"a": 0.5}, {"a": 0.5})
    from types import SimpleNamespace as NS
    fake = {"a": NS(ok=True, event_time=4.0), "b": NS(ok=False, event_time=4.0), "c": NS(ok=True, event_time=None),
            "d": NS(ok=True, event_time=float("inf")), "e": NS(ok=True, event_time=0.0), "f": NS(event_time=3.0),
            "g": NS(ok=True, event_time=0.5), "h": NS(ok=True), "i": NS(ok=True, event_time=-1.0)}
    real_sim = W.lapsim.simulate_events
    try:
        W.lapsim.simulate_events = lambda veh, params=None, endurance_laps=1: fake
        yield "times.fake", W._event_times(None)
    finally:
        W.lapsim.simulate_events = real_sim
    from suspension.kinematics import Hardpoints, SuspensionKinematics
    from suspension.tiremodel import default_tire
    veh = W.vehicle_from_ledger(_ledger())
    sim = W.lapsim.LapSimParams(mass=260.0, cl_a=3.0)
    yield "times", [W._event_times(veh), W._event_times(veh, sim), W._event_times(veh, None, 2)]
    kin = SuspensionKinematics(Hardpoints.default())
    tv = W.vehicle_from_ledger(_ledger(), base=paper, front_kin=kin, rear_kin=kin, tire=default_tire())
    yield "vehicle.kin", [tv.p.mass, tv.front_kin is kin, tv.rear_kin is kin, tv.tire is not None]
    yield "assess.kin", _verdict(W.assess(_ledger(pt_mass=70), paper_baseline=paper, front_kin=kin, rear_kin=kin,
                                          tire=default_tire(), sim_params=sim))
    yield "assess.front_only", _verdict(W.assess(_ledger(pt_mass=70), paper_baseline=paper, front_kin=kin))
    yield "assess.rear_only", _verdict(W.assess(_ledger(pt_mass=70), paper_baseline=paper, rear_kin=kin))
    yield "assess.sim", _verdict(W.assess(_ledger(pt_mass=70), paper_baseline=paper, sim_params=sim))
    allcg = _ledger()
    allcg.get("brakes").cg_z_mm = None
    yield "assess.allmass_nocg", _verdict(W.assess(allcg))
    yield "verdict.none", W.WorthwhileVerdict(False, [], [], None, None, None, False, "").total_delta()
    yield "provenance", W.PROVENANCE


def test_every_assumption_and_assessment():
    check_lazy("worthwhile", _cases())
