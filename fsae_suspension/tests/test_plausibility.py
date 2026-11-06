"""Plausibility checks at entry (suspension/plausibility.py) and where they are wired in: the road-load function,
the ICE vehicle, gearbox, engine map and fuel function, and the Formula Student deck loader."""
import json
import math
import sys
import warnings
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from suspension import ev_electrical_check as ec
from suspension import ice_powertrain as ice
from suspension import plausibility as pl

ROOT = Path(__file__).resolve().parent.parent
W = pl.PlausibilityWarning


def _quiet(fn, *a, **k):
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        return fn(*a, **k)


@pytest.mark.parametrize("table, cls", [(pl.GENERIC, None), (pl.FSAE, "fsae")])
def test_every_range_passes_its_middle_and_flags_both_sides(table, cls):
    for name, (lo, hi, unit, why, slip) in table.items():
        mid = math.sqrt(lo * hi) if lo > 0 else 0.5 * (lo + hi)
        if name == "specific_power_w_kg":
            continue
        assert pl.check({name: mid}, cls, warn=False) == [], name
        for bad in (hi * 1.01 + 1e-9, (lo * 0.99 if lo > 0 else lo - 1.0)):
            if name in pl.OPTIONAL_TERMS and bad == 0.0:
                continue
            msgs = pl.check({name: bad}, cls, warn=False)
            assert len(msgs) == 1 and name in msgs[0] and slip in msgs[0] and why in msgs[0], (name, bad)
            assert f"{lo:g}–{hi:g}" in msgs[0]


def test_the_bounds_themselves_pass():
    for name, (lo, hi, *_rest) in pl.GENERIC.items():
        if name != "specific_power_w_kg":
            assert pl.check({name: lo}, warn=False) == [] and pl.check({name: hi}, warn=False) == []


def test_formula_student_ranges_are_narrower_and_fall_back_to_generic():
    assert pl.check({"mass_kg": 608.0}, warn=False) == []
    assert "pounds" in pl.check({"mass_kg": 608.0}, "fsae", warn=False)[0]
    assert pl.check({"crr": 1.5}, "fsae", warn=False) == pl.check({"crr": 1.5}, warn=False)
    assert pl.check({"mass_kg": 608.0}, "generic", warn=False) == []
    with pytest.raises(ValueError, match="unknown vehicle class 'kart'; one of"):
        pl.check({"mass_kg": 1.0}, "kart")


def test_specific_power_needs_both_power_and_mass():
    assert "specific_power_w_kg" in pl.check({"peak_power_w": 5e3, "mass_kg": 1000.0}, warn=False)[0]
    assert pl.check({"peak_power_w": 5e3}, warn=False) == []
    assert pl.check({"peak_power_w": 5e3, "mass_kg": None}, warn=False) == []
    assert pl.check({"peak_power_w": None, "mass_kg": 1000.0}, warn=False) == []
    assert pl.check({"peak_power_w": 5e4, "mass_kg": 1000.0}, warn=False) == []


def test_off_terms_unknown_names_and_missing_values_are_left_alone():
    assert pl.check({"aux_power_w": 0.0, "crr": 0.0, "cda_m2": 0.0, "cla_m2": 0.0}, warn=False) == []
    assert pl.check({"mass_kg": 0.0}, warn=False) != []
    assert pl.check({"colour": 7.0, "mass_kg": None, "mu": math.nan}, warn=False) == []


def test_a_list_is_judged_by_its_worst_value_once():
    msgs = pl.check({"drivetrain_eff": [0.95, 0.3, 0.1]}, warn=False)
    assert len(msgs) == 1 and "0.1 " in msgs[0]
    msgs = pl.check({"drivetrain_eff": [0.95, 1.2, 0.39]}, warn=False)
    assert "1.2 " in msgs[0]
    assert "-3 " in pl.check({"cla_m2": [-3.0, 9.0]}, warn=False)[0]


def test_messages_carry_where_and_are_warned_unless_told_not_to():
    with pytest.warns(W, match=r"^here: mass_kg = 1e\+05 kg is outside 60–60000 kg"):
        out = pl.check({"mass_kg": 1e5}, where="here")
    assert out[0].startswith("here: ")
    assert pl.check({"crr": 1.5}, warn=False)[0].startswith("crr = 1.5 is outside 0.002–0.05 (")
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        pl.check({"mass_kg": 1e5}, warn=False)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        pl.check({"mass_kg": 1e5, "crr": 1.5})
    assert [x.category for x in w] == [W, W]


# ---------------------------------------------------------------------------------------------------------------------
#  The road-load function
# ---------------------------------------------------------------------------------------------------------------------
V, A = np.array([10.0, 12.0]), np.array([0.5, 0.0])


@pytest.mark.parametrize("kw", [dict(drivetrain_eff=0.0), dict(drivetrain_eff=1.01), dict(drivetrain_eff=92.0),
                                dict(regen_eff=1.01), dict(regen_eff=-0.1)])
def test_road_load_refuses_efficiencies_that_are_not_fractions(kw):
    with pytest.raises(ValueError, match=r"^drive and regeneration efficiencies are fractions, at most 1: was one "
                                         r"typed in percent\?$"):
        ec.traction_power_w(V, A, **kw)


def test_road_load_accepts_the_edges_of_meaningful_efficiencies():
    _quiet(ec.traction_power_w, V, A, drivetrain_eff=1.0, regen_eff=1.0)
    _quiet(ec.traction_power_w, V, A, regen_eff=0.0)


@pytest.mark.parametrize("kw, name", [(dict(vehicle_mass_kg=1e5), "mass_kg"), (dict(drag_cda=20.0), "cda_m2"),
                                      (dict(cla_m2=10.0), "cla_m2"), (dict(crr=1.5), "crr"),
                                      (dict(drivetrain_eff=0.3), "drivetrain_eff"),
                                      (dict(aux_power_w=3e5), "aux_power_w"),
                                      (dict(motor_eff_map=((0.0, 2e6), (0.9, 0.9)), vehicle_mass_kg=2000.0),
                                       "peak_power_w")])
def test_road_load_warns_on_each_input(kw, name):
    with pytest.warns(W, match=rf"^traction_power_w: {name} = "):
        ec.traction_power_w(V, A, **kw)


def test_road_load_leaves_crr_to_the_tire_model_and_checks_no_motor_map_power():
    _quiet(ec.traction_power_w, V, A, crr=1.5, tire=ec.TireLosses())
    mm = ec.MotorEfficiencyMap((0.0, 20000.0), (0.0, 100.0), ((0.9, 0.9), (0.9, 0.9)))
    _quiet(ec.traction_power_w, V, A, motor_eff_map=mm)
    _quiet(ec.traction_power_w, V, A, motor_eff_map=((0.0, 5e4), (0.9, 0.9)), vehicle_mass_kg=300.0)


def test_road_load_specific_power_uses_the_vehicle_mass():
    with pytest.warns(W, match="specific_power_w_kg"):
        ec.traction_power_w(V, A, motor_eff_map=((0.0, 5e4), (0.9, 0.9)), vehicle_mass_kg=30000.0)


# ---------------------------------------------------------------------------------------------------------------------
#  The ICE entry points
# ---------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("eff", [0.0, 1.01, 92.0])
def test_gearbox_refuses_an_efficiency_that_is_not_a_fraction(eff):
    with pytest.raises(ValueError, match=r"^the gearbox efficiency is a fraction, at most 1: was it typed in percent\?$"):
        ice.Gearbox(efficiency=eff)


@pytest.mark.parametrize("kw, name", [(dict(final_drive=0.5), "final_drive"), (dict(tire_radius_m=233.0), "wheel_radius_m"),
                                      (dict(efficiency=0.3), "drivetrain_eff")])
def test_gearbox_warns_on_each_input(kw, name):
    with pytest.warns(W, match=rf"^Gearbox: {name} = "):
        ice.Gearbox(**kw)


def test_the_default_gearbox_and_vehicle_are_plausible():
    _quiet(ice.Gearbox, efficiency=1.0)
    _quiet(ice.ICEVehicle)


@pytest.mark.parametrize("kw, name", [(dict(mass_kg=0.3), "mass_kg"), (dict(cg_height_m=280.0), "cg_height_m"),
                                      (dict(wheelbase_m=1550.0), "wheelbase_m"), (dict(mu=140.0), "mu"),
                                      (dict(cda_m2=20.0), "cda_m2"), (dict(cla_m2=30.0), "cla_m2"),
                                      (dict(crr=1.5), "crr")])
def test_ice_vehicle_warns_on_each_input(kw, name):
    with pytest.warns(W, match=rf"^ICEVehicle: {name} = "):
        ice.ICEVehicle(**kw)


def test_engine_map_warns_on_its_peak_power():
    with pytest.warns(W, match=r"^PowerEfficiencyMap: peak_power_w = 6e\+07 W"):
        ice.PowerEfficiencyMap((0.0, 1e5, 6e7), (0.0, 0.3, 0.3))   # the top point, not the second
    _quiet(ice.PowerEfficiencyMap, (0.0, 6e4), (0.0, 0.3))


def test_fuel_for_lap_warns_on_its_auxiliary_power():
    lap = SimpleNamespace(s=np.array([0.0, 10.0, 20.0]), v=np.array([10.0, 10.0, 10.0]))
    eng = ice.EngineMap(rpm=[500.0, 8000.0], torque_nm=[50.0, 50.0], redline_rpm=8000.0, idle_rpm=500.0)
    with pytest.warns(W, match=r"^fuel_for_lap: aux_power_w = 3e\+05 W"):
        ice.fuel_for_lap(lap, eng, ice.Gearbox(), aux_power_w=3e5)
    _quiet(ice.fuel_for_lap, lap, eng, ice.Gearbox(), aux_power_w=300.0)


# ---------------------------------------------------------------------------------------------------------------------
#  The Formula Student deck loader
# ---------------------------------------------------------------------------------------------------------------------
sys.path.insert(0, str(ROOT / "scripts"))
import powertrain_benchmark as pb  # noqa: E402

DECK = ROOT / "benchmarks" / "powertrain" / "simulink_blind2" / "runs" / "hock_fs2" / "inputs.json"


def _deck():
    return json.loads(DECK.read_text())


def test_every_stored_simscape_deck_is_plausible_as_a_formula_student_car():
    decks = sorted((ROOT / "benchmarks" / "powertrain").glob("simulink_*/**/inputs.json"))
    assert len(decks) >= 19
    for f in decks:
        assert _quiet(pb.check_deck, json.loads(f.read_text())) == [], f


@pytest.mark.parametrize("path, value, name", [
    (("vehicle", "mass_kg"), 608.0, "mass_kg"), (("vehicle", "cda_m2"), 12.9, "cda_m2"),
    (("vehicle", "cla_m2"), 32.0, "cla_m2"), (("vehicle", "crr"), 0.7, "crr"),
    (("vehicle", "tire_radius_m"), 0.466, "wheel_radius_m"), (("vehicle", "mu"), 111.0, "mu"),
    (("vehicle", "cg_height_m"), 289.0, "cg_height_m"), (("vehicle", "wheelbase_m"), 153.0, "wheelbase_m"),
    (("ev", "final_drive"), 0.5, "final_drive"), (("ev", "motor_max_rpm"), 2e5, "motor_max_rpm"),
    (("ev", "motor_peak_torque_nm"), 1e4, "motor_peak_torque_nm"), (("ev", "pack_power_kw"), 250.0, "peak_power_w"),
    (("ev", "inverter_eff"), 0.3, "drivetrain_eff"), (("ev", "pack", "cell_v"), 3.6, "pack_voltage_v"),
    (("ev", "pack", "cell_ah"), 25000.0, "capacity_ah"), (("ev", "pack", "cell_ohm"), 79.0, "pack_resistance_ohm")])
def test_the_deck_loader_warns_on_each_input(path, value, name):
    d = _deck()
    obj = d
    for k in path[:-1]:
        obj = obj[k]
    obj[path[-1]] = value
    with pytest.warns(W, match=rf"^deck: {name} = "):
        pb.check_deck(d, where="deck")


@pytest.mark.parametrize("key", ["inverter_eff", "motor_eff", "driveline_eff"])
def test_the_deck_loader_refuses_an_efficiency_in_percent(key):
    d = _deck()
    d["ev"][key] = 98.0
    with pytest.raises(ValueError, match="deck: drive efficiencies are fractions"):
        pb.check_deck(d, where="deck")
    d["ev"][key] = 0.0
    with pytest.raises(ValueError):
        pb.check_deck(d, where="deck")


def test_the_deck_loader_tolerates_missing_entries_and_names_the_class():
    d = _deck()
    del d["ev"]["pack_power_kw"], d["ev"]["pack"]["cell_ohm"], d["ev"]["inverter_eff"]
    assert _quiet(pb.check_deck, d) == []
    d["vehicle"]["mass_kg"] = 608.0
    assert pb.check_deck(d, vehicle_class="generic", where="x") == []
    with pytest.warns(W, match="^inputs.json: mass_kg"):
        pb.check_deck(d)


def test_lap_and_launch_grading_check_the_deck_first(tmp_path):
    d = _deck()
    d["ev"]["driveline_eff"] = 100.0
    (tmp_path / "inputs.json").write_text(json.dumps(d))
    run = DECK.parent
    with pytest.raises(ValueError, match="drive efficiencies are fractions"):
        pb.simulink_lap_rows(tmp_path / "inputs.json", run / "simulink_lap.csv")
    with pytest.raises(ValueError, match="drive efficiencies are fractions"):
        pb.blind_launch_rows(tmp_path / "inputs.json", run.parent / "launch_fs2" / "simulink_lap.csv")
