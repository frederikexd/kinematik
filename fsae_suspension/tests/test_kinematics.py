"""
Physics sanity tests for the kinematics engine.

These aren't exhaustive validation against a commercial solver — that's a great
PR to add — but they pin the conventions and catch regressions in the signs and
gains that matter most when tuning a real car.

Run:  python -m pytest tests/  (or just: python tests/test_kinematics.py)
"""
import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension import SuspensionKinematics, Hardpoints, VehicleDynamics, VehicleParams


def test_static_matches_design_intent():
    hp = Hardpoints.default()
    kin = SuspensionKinematics(hp)
    assert abs(kin.static.camber - hp.static_camber) < 0.05
    assert abs(kin.static.toe - hp.static_toe) < 0.05


def test_linkage_closes_over_travel():
    kin = SuspensionKinematics(Hardpoints.default())
    for s in kin.sweep(-30, 30, 21):
        assert s.converged, f"linkage failed to close at travel {s.travel}"


def test_negative_camber_gain_in_bump():
    # Good FSAE geometry gains negative camber as the wheel moves into bump.
    kin = SuspensionKinematics(Hardpoints.default())
    c_bump = kin.solve_at_travel(20).camber
    c_droop = kin.solve_at_travel(-20).camber
    assert c_bump < c_droop, "expected more negative camber in bump"


def test_caster_positive_for_rearward_kingpin():
    kin = SuspensionKinematics(Hardpoints.default())
    assert kin.static.caster > 0


def test_roll_angle_is_physical():
    kin = SuspensionKinematics(Hardpoints.default())
    veh = VehicleDynamics(VehicleParams(), front_kin=kin, rear_kin=kin)
    _, info = veh.lateral_load_transfer(1.2)
    assert 0 < info["roll_angle"] < 6, "roll angle out of physical range"


def test_outer_wheels_gain_load():
    kin = SuspensionKinematics(Hardpoints.default())
    veh = VehicleDynamics(VehicleParams(), front_kin=kin, rear_kin=kin)
    loads, _ = veh.lateral_load_transfer(1.0)
    assert loads.fr > loads.fl and loads.rr > loads.rl


def test_load_conservation():
    kin = SuspensionKinematics(Hardpoints.default())
    p = VehicleParams()
    veh = VehicleDynamics(p, front_kin=kin, rear_kin=kin)
    loads, _ = veh.lateral_load_transfer(0.0)
    total = sum(loads.as_tuple())
    assert abs(total - p.mass * p.g) < 1.0


def test_max_g_in_reasonable_range():
    kin = SuspensionKinematics(Hardpoints.default())
    veh = VehicleDynamics(VehicleParams(), front_kin=kin, rear_kin=kin)
    assert 0.9 < veh.max_lateral_g() < 2.2


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    for fn in fns:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {fn.__name__}: {e}")
    print(f"\n{passed}/{len(fns)} tests passed")
