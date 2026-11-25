"""The rear axle can have its own corner: whole-car results use it, and the app no longer reuses the front."""
from pathlib import Path

import numpy as np

from suspension.dynamics import VehicleDynamics, VehicleParams
from suspension.kinematics import Hardpoints, SuspensionKinematics

APP = (Path(__file__).resolve().parents[1] / "streamlit_app.py").read_text()


def _rear():
    d = Hardpoints.default().as_dict()
    for k in ("upper_front_inner", "upper_rear_inner"):
        d[k] = [d[k][0], d[k][1], d[k][2] + 30.0]                # a different rear geometry
    return SuspensionKinematics(Hardpoints.from_dict(d))


def test_vehicle_rear_quantities_come_from_the_rear_corner():
    front = SuspensionKinematics(Hardpoints.default())
    rear = _rear()
    same = VehicleDynamics(VehicleParams(), front_kin=front, rear_kin=front)
    own = VehicleDynamics(VehicleParams(), front_kin=front, rear_kin=rear)
    assert own.anti_squat_pct() != same.anti_squat_pct()
    assert np.isclose(own.anti_squat_pct(), rear.anti_squat_pct(VehicleParams().cg_height, VehicleParams().wheelbase))
    assert own.anti_dive_pct() == same.anti_dive_pct()           # the front is untouched


def test_the_app_never_reuses_the_front_corner_for_the_rear_silently():
    assert "rear_kin=kin," not in APP and "rear_kin=kin)" not in APP
    assert "front_kin=kin, rear_kin=rear_kin" in APP
    assert '"hardpoints_rear": st.session_state.get("hp_rear")' in APP
    assert 'data.get("hardpoints_rear")' in APP
