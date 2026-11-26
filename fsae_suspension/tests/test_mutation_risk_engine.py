"""Characterization suite for suspension/risk_engine.py (mutation gate); see tests/_golden.py. Severity and
occurrence elevation at, under and past their clamps; rule matching on exact and per-segment channels, deficits on
both limit kinds and a zero limit; the default matrix and a custom one (and a duplicate id); full evaluations with
compliant, failing, repeated, unmatched and propagating readings, the report's views and DFMEA records; manifold
and network readings; slotted-hole joints round, offset, narrow and bearing-capped, with every refusal, and their
pedal-tab readings."""
import math
from types import SimpleNamespace

from suspension import risk_engine as R
from suspension.bolted_joint import Fastener
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _rep(rep):
    return {"risks": [r.as_dict() for r in rep.risks], "active": [r.rule_id + "|" + r.channel for r in rep.active()],
            "worst": None if rep.worst() is None else rep.worst().rule_id, "summary": rep.summary(),
            "records": rep.as_records(), "all_records": rep.as_records(triggered_only=False),
            "unmatched": plain(rep.unmatched)}


class Seg:
    def __init__(self, name, dp):
        self.name, self.dp = name, dp

    def audit(self, q, fluid):
        return {"dp_pa": self.dp * q * 1e3 * (2.0 if fluid == "glycol" else 1.0)}


class Net:
    fluid = "glycol"

    def __init__(self):
        self.segments = [Seg("a", 10.0), Seg("b", 40.0), SimpleNamespace(audit=lambda q, f: {"dp_pa": 5000.0})]

    def operating_point(self):
        return {"q_m3s": 0.5}


def _cases():
    yield "readings", plain(R.readings(("brakes", "x", 1), ("cooling", "y", "2.5", "kPa"), ("a", "b", 3, "", "solver")))
    yield "elevate", [R.elevate_severity(b, d, gain=g) for b, d, g in ((5, 0.0, 6.0), (5, -0.2, 6.0), (5, 0.1, 6.0),
                                                                       (5, 0.1001, 6.0), (9, 0.5, 6.0), (0, -1.0, 6.0),
                                                                       (12, 0.0, 6.0), (5, 0.1, 8.0), (-5, 0.01, 6.0))] + [
        R.elevate_severity(6, 0.2), R.elevate_severity(6, 0.3), R.elevate_severity(3, -0.5, gain=-6.0)]
    yield "occurrence", [R.occurrence_from_deficit(d, floor=f) for d, f in ((0.0, 3), (-1.0, 3), (0.01, 3), (0.2, 3),
                                                                            (0.6, 3), (0.0, 1), (0.1, 0))] + [
        R.occurrence_from_deficit(0.05), R.occurrence_from_deficit(0.01, floor=-5)]
    rules = R.default_rules()
    yield "rules", plain(rules)
    yield "rules.custom", plain(R.default_rules(chassis_fos_min=2.0, brake_fos_min=1.8, shaft_fos_min=1.5,
                                                pedal_joint_fos_min=1.3, manifold_dp_max_kpa=30.0,
                                                vapor_margin_min_kpa=25.0))
    seg = next(r for r in rules if r.channel.endswith(":"))
    node = rules[0]
    yield "match", [seg.matches(R.Reading("cooling", "manifold_dp_kpa:s1", 1.0)),
                    seg.matches(R.Reading("cooling", "manifold_dp_kpa", 1.0)),
                    seg.matches(R.Reading("brakes", "manifold_dp_kpa:s1", 1.0)),
                    node.matches(R.Reading("chassis", "node_fos", 1.0)),
                    node.matches(R.Reading("chassis", "node_fos:x", 1.0))]
    yield "deficit", [node.deficit(v) for v in (1.5, 1.2, 2.0)] + [seg.deficit(v) for v in (35.0, 40.0, 10.0)] + [
        R.RiskRule("z", "s", "c", R.LimitKind.MIN, 0.0, "f", "e", 5, 5).deficit(3.0),
        R.RiskRule("n", "s", "c", R.LimitKind.MAX, -10.0, "f", "e", 5, 5).deficit(-5.0)]
    eng = R.RiskEngine()
    rds = R.readings(("chassis", "bracket_fos", 1.2), ("chassis", "node_fos", 1.4), ("chassis", "pedalbox_base_fos", 2.0),
                     ("brakes", "caliper_bracket_fos", 1.5), ("brakes", "caliper_bracket_fos", 1.3),
                     ("brakes", "caliper_bracket_fos", 1.4), ("brakes", "bias_margin_fos", 1.2),
                     ("brakes", "pedal_tab_joint_fos", 0.9), ("powertrain", "shaft_fos", 1.0),
                     ("powertrain", "motor_mount_fos", 1.6), ("powertrain", "motor_temp_margin_fos", 0.95),
                     ("powertrain", "inverter_temp_margin_fos", 1.1),
                     ("cooling", "manifold_dp_kpa:seg1", 50.0, "kPa", "solver"), ("cooling", "manifold_dp_kpa:seg2", 20.0),
                     ("cooling", "pump_inlet_margin_kpa", 10.0), ("aero", "wing_fos", 3.0), ("chassis", "mystery", 1.0))
    yield "eval.all", _rep(eng.evaluate(rds))
    yield "eval.clean", _rep(eng.evaluate(R.readings(("chassis", "node_fos", 3.0), ("brakes", "caliper_bracket_fos", 2.0),
                                                     ("powertrain", "motor_mount_fos", 2.0))))
    yield "eval.empty", _rep(eng.evaluate([]))
    yield "eval.dp_only", _rep(eng.evaluate(R.readings(("cooling", "manifold_dp_kpa:a", 36.0),
                                                       ("cooling", "manifold_dp_kpa:b", 70.0),
                                                       ("powertrain", "motor_temp_margin_fos", 1.2),
                                                       ("powertrain", "inverter_temp_margin_fos", 1.2))))
    yield "eval.twice", _rep(R.RiskEngine(rules=rules[:2], propagation=()).evaluate(R.readings(
        ("chassis", "node_fos", 1.0), ("chassis", "node_fos", 1.0), ("chassis", "bracket_fos", 1.6))))
    try:
        R.RiskEngine(rules=[rules[0], rules[0]])
    except ValueError as e:
        yield "dup", str(e)
    yield "edges", plain(R.PROPAGATION)
    yield "manifold", [plain(R.manifold_readings({"b": 12.0, "a": 40.0})),
                       plain(R.manifold_readings({"x": 1}, pump_head_kpa=50.0, inlet_abs_kpa=101.3)),
                       plain(R.manifold_readings({}, inlet_abs_kpa=40.0, vapor_pressure_kpa=20.0, source="s"))]
    yield "network", [plain(R.network_readings(Net())), plain(R.network_readings(Net(), q_m3s=0.25, source="t"))]
    joints = {"default": R.SlottedHoleJoint(),
              "offset": R.SlottedHoleJoint(slot_offset_mm=4.0),
              "narrow": R.SlottedHoleJoint(washer_od_mm=10.0, slot_length_mm=30.0),
              "soft": R.SlottedHoleJoint(bearing_allow_MPa=60.0),
              "m8": R.SlottedHoleJoint(fastener=Fastener(grade="8.8", nominal_d_mm=8.0), slot_width_mm=8.6,
                                       washer_od_mm=20.0, thread_mu=0.12, head_mu=0.1, thread_half_angle_deg=29.0),
              "round": R.SlottedHoleJoint(slot_length_mm=6.6),
              "custom_area": R.SlottedHoleJoint(fastener=Fastener(nominal_d_mm=7.0, stress_area_mm2=28.0),
                                                slot_width_mm=7.5)}
    for name, j in joints.items():
        a = R.analyze_slotted_joint(j)
        b = R.analyze_slotted_joint(j, assembly_torque_Nm=9.0, target_preload_fraction_of_proof=0.5,
                                    bearing_safety=2.0)
        yield f"joint.{name}", [a.as_dict(), b.as_dict(), plain(a.pedal_tab_fos_reading(1500.0)),
                                plain(b.pedal_tab_fos_reading(800.0, n_interfaces=2, mu_joint=0.2)),
                                plain(a.pedal_tab_fos_reading(0.0)), plain(b.pedal_tab_fos_reading(500.0, n_interfaces=0)),
                                plain(a.pedal_tab_fos_reading(0.5)), plain(a.pedal_tab_fos_reading(-1.0))]
    for name, j in (("thread", R.SlottedHoleJoint(fastener=Fastener(nominal_d_mm=7.0))),
                    ("grade", R.SlottedHoleJoint(fastener=Fastener(grade="4.6"))),
                    ("slot", R.SlottedHoleJoint(slot_width_mm=6.0)),
                    ("washer", R.SlottedHoleJoint(washer_od_mm=6.6))):
        try:
            R.analyze_slotted_joint(j)
        except ValueError as e:
            yield f"joint.bad.{name}", str(e)
    zero = R.analyze_slotted_joint(R.SlottedHoleJoint())
    yield "joint.none", plain(R.SlottedJointResult(**{**zero.as_dict(), "F_clamp_at_torque_N": None, "F_target_N": None})
                              .pedal_tab_fos_reading(100.0))
    yield "patch", [R._contact_patch(R.SlottedHoleJoint(), n_theta=36, n_r=12),
                    R._contact_patch(R.SlottedHoleJoint(slot_offset_mm=2.0), n_theta=40, n_r=10),
                    R._contact_patch(R.SlottedHoleJoint(washer_od_mm=1.0, slot_width_mm=30.0), n_theta=8, n_r=4),
                    R._contact_patch(R.SlottedHoleJoint(washer_od_mm=1.0, slot_width_mm=0.5, slot_length_mm=0.5),
                                     n_theta=24, n_r=8),
                    R._annulus(9.0, 3.3)]
    MIN, MAX = R.LimitKind.MIN, R.LimitKind.MAX
    custom = R.RiskEngine(rules=[
        R.RiskRule("lo", "s", "c", MIN, 2.0, "f", "e", 4, 8, cause=""),
        R.RiskRule("hi", "s", "c", MAX, 4.0, "f2", "e2", 5, 2, item="it", cause="heat", severity_gain=3.0),
        R.RiskRule("tgt", "t", "d:", MIN, 1.0, "f3", "e3", 6, 6, cause="m")],
        propagation=(R.PropagationEdge("s", "c", "tgt", 9, "heat-soak"),
                     R.PropagationEdge("s", "c", "tgt", 2, "m")))
    yield "eval.custom", _rep(custom.evaluate([
        R.Reading("s", "c", 1.5, "", "first"), R.Reading("s", "c", 5.0, "", "second"),
        R.Reading("s", "c", 1.5, "", "third"), R.Reading("s", "c", 5.0, "", "fourth"),
        R.Reading("t", "d:a", 0.9), R.Reading("t", "d:b", 1.0), R.Reading("t", "d:c", 2.0),
        R.Reading("u", "c", 1.0)]))
    yield "eval.at_limit", _rep(custom.evaluate([R.Reading("s", "c", 2.0), R.Reading("t", "d:x", 1.0)]))
    tie = custom.evaluate([R.Reading("s", "c", 1.8), R.Reading("t", "d:a", 0.9)])
    a1, a2 = [r for r in tie.risks if r.triggered][:2]
    a1.rpn = a2.rpn = 100
    a1.severity, a2.severity = 3, 7
    yield "active.tie", [r.rule_id + "|" + r.channel for r in tie.active()]
    yield "consts", [R.SUBSYSTEMS, [k.value for k in R.LimitKind], math.isinf(math.inf)]


def test_every_risk_and_joint():
    check_lazy("risk_engine", _cases())
