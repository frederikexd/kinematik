"""suspension/mountpoints.py: the box distance written out per axis, the clash rules (own volumes and the mounted-on
structure skipped, inside = FAIL, inside the clearance band = WARN, the band edge itself clear), every message and
detail word for word, the dict round trips, and one move carried through to the ledger's CG."""
import math

import numpy as np
import pytest

from suspension.interfaces import IntegrationLedger, Severity, SubsystemInterface
from suspension.mountpoints import GeometryLedger, KeepOut, MountPoint, PropagationResult, propagate_mount_move


def test_the_point_and_its_dict():
    mp = MountPoint("wing", (1.0, 2.0, 3.0), "aerodynamics")
    assert (mp.mounts_on, mp.min_clearance_mm, mp.is_estimate, mp.set_by, mp.notes) == ("chassis", 5.0, True, "", "")
    assert mp.as_array().tolist() == [1.0, 2.0, 3.0]
    d = mp.as_dict()
    assert d == dict(name="wing", xyz_mm=(1.0, 2.0, 3.0), owner_subsystem="aerodynamics", mounts_on="chassis",
                     min_clearance_mm=5.0, is_estimate=True, set_by="", notes="")
    back = MountPoint.from_dict({**d, "xyz_mm": [1.0, 2.0, 3.0], "junk": 1})
    assert back == mp and isinstance(back.xyz_mm, tuple)
    assert MountPoint.from_dict({**d, "xyz_mm": (4, 5, 6)}).xyz_mm == (4, 5, 6)


def test_the_box_orders_its_corners_and_measures_signed_distance():
    ko = KeepOut("legs", "chassis", (10.0, -5.0, 8.0), (0.0, 5.0, 2.0))
    assert ko.lo_mm == (0.0, -5.0, 2.0) and ko.hi_mm == (10.0, 5.0, 8.0) and ko.is_estimate is True and ko.notes == ""
    assert ko.signed_distance_mm(np.array([13.0, 9.0, 5.0])) == pytest.approx(5.0)             # (3, 4, 0)
    assert ko.signed_distance_mm(np.array([-3.0, 0.0, 0.0])) == pytest.approx(math.hypot(3.0, 2.0))
    assert ko.signed_distance_mm(np.array([5.0, 0.0, 3.0])) == pytest.approx(-1.0)              # nearest face z = 2
    assert ko.signed_distance_mm(np.array([9.0, 0.0, 5.0])) == pytest.approx(-1.0)              # nearest face x = 10
    assert ko.signed_distance_mm(np.array([5.0, -4.5, 5.0])) == pytest.approx(-0.5)
    assert ko.signed_distance_mm(np.array([10.0, 0.0, 5.0])) == 0.0
    d = ko.as_dict()
    assert d == dict(name="legs", owner_subsystem="chassis", lo_mm=(0.0, -5.0, 2.0), hi_mm=(10.0, 5.0, 8.0),
                     is_estimate=True, notes="")
    back = KeepOut.from_dict({**d, "lo_mm": [0.0, -5.0, 2.0], "hi_mm": [10.0, 5.0, 8.0], "junk": 2})
    assert back == ko and isinstance(back.lo_mm, tuple) and isinstance(back.hi_mm, tuple)
    only_hi = KeepOut.from_dict({**d, "lo_mm": (0.0, -5.0, 2.0), "hi_mm": [10.0, 5.0, 8.0]})
    assert only_hi == ko


def _geom():
    g = GeometryLedger()
    g.set_point(MountPoint("wing_l", (0.0, 0.0, 0.0), "aerodynamics", is_estimate=False, min_clearance_mm=4.0))
    g.set_point(MountPoint("cooler", (50.0, 0.0, 0.0), "cooling", mounts_on="powertrain", is_estimate=False))
    return g


def test_no_points_and_no_volumes():
    assert GeometryLedger().check_clashes() == []
    (f,) = _geom().check_clashes()
    assert (f.check, f.severity, f.subsystems) == ("clash", Severity.MISSING, ["aerodynamics", "cooling"])
    assert f.message == ("Mount points are declared but no keep-out volumes exist to check them against — no "
                         "clearance check is possible yet.")


def test_the_clash_rules_and_messages():
    g = _geom()
    g.set_keepout(KeepOut("tube", "chassis", (-1.0, -1.0, -2.0), (1.0, 1.0, 2.0), is_estimate=False))   # mounted on
    g.set_keepout(KeepOut("duct", "cooling", (-2.0, -2.0, -0.5), (2.0, 2.0, 1.5), is_estimate=False))
    g.set_keepout(KeepOut("box", "electrics", (52.0, -1.0, -1.0), (60.0, 1.0, 1.0)))
    g.set_keepout(KeepOut("pt", "powertrain", (49.0, -1.0, -1.0), (51.0, 1.0, 1.0), is_estimate=False))
    g.set_keepout(KeepOut("aero", "aerodynamics", (-1.0, -1.0, -1.0), (1.0, 1.0, 1.0), is_estimate=False))
    out = g.check_clashes()
    assert [(f.check, f.severity) for f in out] == [("clash-interference", Severity.FAIL),
                                                     ("clash-clearance", Severity.WARN)]
    fail, warn = out
    assert fail.subsystems == ["aerodynamics", "cooling"] and fail.message == (
        "aerodynamics's 'wing_l' is 0.5 mm INSIDE cooling's keep-out 'duct' — hard interference. cooling's master "
        "file clashes here.")
    assert fail.detail == dict(point="wing_l", keepout="duct", penetration_mm=0.5, clearance_req_mm=4.0,
                               estimate=False)
    assert warn.subsystems == ["cooling", "electrics"] and warn.message == (
        "cooling's 'cooler' clears electrics's 'box' by only 2.0 mm (needs 5 mm) (estimated geometry).")
    assert warn.detail == dict(point="cooler", keepout="box", gap_mm=2.0, clearance_req_mm=5.0, estimate=True)


def test_estimates_from_either_side_and_the_band_edge():
    g = GeometryLedger()
    g.set_point(MountPoint("p", (0.0, 0.0, 4.0), "brakes", is_estimate=True))
    g.set_keepout(KeepOut("k", "suspension", (-1.0, -1.0, -3.0), (1.0, 1.0, 1.0), is_estimate=False))
    (f,) = g.check_clashes()
    assert f.severity == Severity.WARN and f.detail["estimate"] is True and "(estimated geometry)" in f.message
    g.points["p"].xyz_mm = (0.0, 0.0, 6.0)                                   # gap exactly the clearance: clear
    (ok,) = g.check_clashes()
    assert (ok.check, ok.severity, ok.subsystems) == ("clash", Severity.OK, ["brakes"])
    assert ok.message == "All 1 mount point(s) clear every keep-out with required margin."
    g.points["p"].xyz_mm = (0.0, 0.0, 1.0)                                   # on the surface: a warning, not a clash
    assert g.check_clashes()[0].severity == Severity.WARN


def test_moves_and_the_ledger_round_trip():
    g = _geom()
    g.set_keepout(KeepOut("k", "chassis", (0, 0, 0), (1, 1, 1)))
    mp = g.move_point("wing_l", [1, 2.5, 3], set_by="ana")
    assert mp is g.points["wing_l"] and mp.xyz_mm == (1.0, 2.5, 3.0) and mp.set_by == "ana"
    g.move_point("wing_l", (4, 5, 6))
    assert g.points["wing_l"].set_by == "ana"
    d = g.as_dict()
    assert set(d) == {"points", "keepouts"} and d["points"]["cooler"]["mounts_on"] == "powertrain"
    back = GeometryLedger.from_dict(d)
    assert back.points == g.points and back.keepouts == g.keepouts
    assert GeometryLedger.from_dict(None).points == {} and GeometryLedger.from_dict({"points": None}).keepouts == {}


def _ledger():
    L = IntegrationLedger()
    L.set(SubsystemInterface("aerodynamics", mass_kg=10.0, cg_x_mm=1500.0, cg_y_mm=0.0, cg_z_mm=900.0))
    L.set(SubsystemInterface("chassis", mass_kg=30.0, cg_x_mm=800.0, cg_y_mm=0.0, cg_z_mm=300.0))
    return L


def test_a_move_carries_the_cg_with_it():
    g = _geom()
    g.points["wing_l"].xyz_mm = (1500.0, 0.0, 900.0)
    g.set_keepout(KeepOut("box", "electrics", (1590.0, -5.0, 1000.0), (1610.0, 5.0, 1020.0)))
    L = _ledger()
    r = propagate_mount_move(g, L, "wing_l", (1600.0, 0.0, 1010.0), set_by="ana", update_interface_cg=True)
    assert g.points["wing_l"].set_by == "ana" and r.moved_point == "wing_l" and r.new_xyz_mm == (1600.0, 0.0, 1010.0)
    assert (L.get("aerodynamics").cg_x_mm, L.get("aerodynamics").cg_z_mm) == (1600.0, 1010.0)
    before = ((10 * 1500 + 30 * 800) / 40, 0.0, (10 * 900 + 30 * 300) / 40)
    after = ((10 * 1600 + 30 * 800) / 40, 0.0, (10 * 1010 + 30 * 300) / 40)
    assert r.cg_before_mm == pytest.approx(before) and r.cg_after_mm == pytest.approx(after)
    assert r.cg_delta_mm == pytest.approx((25.0, 0.0, 27.5)) and r.mass_total_kg == 40.0
    assert r.notes == ["aerodynamics CG shifted by (100.0, 0.0, 110.0) mm with the point."]
    assert r.has_hard_clash is True and r.clash_findings[0].severity == Severity.FAIL
    assert r.summary() == "Moved wing_l → (1600.0, 0.0, 1010.0) mm. HARD CLASH flagged. CG moved +27.50 mm in z (now 477.5 mm)."
    y = propagate_mount_move(g, L, "wing_l", (1600.04, 3.06, 1010.0), update_interface_cg=True)
    assert y.notes == ["aerodynamics CG shifted by (0.0, 3.1, 0.0) mm with the point."]
    assert L.get("aerodynamics").cg_y_mm == pytest.approx(3.06)


def test_a_move_without_cg_data():
    g = _geom()
    L = IntegrationLedger()
    L.set(SubsystemInterface("aerodynamics", mass_kg=10.0, cg_x_mm=1500.0, cg_y_mm=None, cg_z_mm=900.0))
    r = propagate_mount_move(g, L, "wing_l", (1.04, 2.26, 3.0), update_interface_cg=True)
    assert r.notes == ["update_interface_cg requested but owner has no declared CG to shift — left unchanged."]
    assert L.get("aerodynamics").cg_x_mm == 1500.0 and r.cg_before_mm is None and r.cg_delta_mm is None
    assert r.has_hard_clash is False and r.mass_total_kg == 10.0
    assert r.summary() == "Moved wing_l → (1.0, 2.3, 3.0) mm. no interference. CG not recomputable (mass/CG data incomplete)."
    assert propagate_mount_move(g, IntegrationLedger(), "cooler", (0, 0, 0), update_interface_cg=True).notes == [
        "update_interface_cg requested but owner has no declared CG to shift — left unchanged."]
    plain = propagate_mount_move(g, _ledger(), "wing_l", (0, 0, 0))
    assert plain.notes == [] and plain.cg_delta_mm == (0.0, 0.0, 0.0) and g.points["wing_l"].set_by == ""
    empty = propagate_mount_move(g, IntegrationLedger(), "wing_l", (0, 0, 0))
    assert empty.mass_total_kg == 0.0 and empty.cg_after_mm is None
    for i, axis in enumerate(("x", "y", "z")):
        L2 = _ledger()
        setattr(L2.get("aerodynamics"), f"cg_{axis}_mm", None)
        n = propagate_mount_move(_geom(), L2, "wing_l", (1, 1, 1), update_interface_cg=True).notes
        assert n[0].startswith("update_interface_cg requested but owner has no declared CG")


def test_the_result_flags_only_failures():
    from suspension.interfaces import Finding
    r = PropagationResult("p", (0.0, 0.0, 0.0), [Finding("c", Severity.WARN, "m")], None, None, None, 0.0)
    assert r.has_hard_clash is False and r.notes == []
    r2 = PropagationResult("p", (0.0, 0.0, 0.0), [Finding("c", Severity.WARN, "m"), Finding("c", Severity.FAIL, "m")],
                           (0.0, 0.0, 0.0), (0.0, 0.0, -1.234), (0.0, 0.0, -1.234), 0.0)
    assert r2.has_hard_clash is True and r2.summary().endswith("CG moved -1.23 mm in z (now -1.2 mm).")


def test_the_cg_shift_adds_to_an_offset_cg():
    g = _geom()
    L = _ledger()
    L.get("aerodynamics").cg_y_mm = 2.0
    propagate_mount_move(g, L, "wing_l", (0.0, 5.0, 0.0), update_interface_cg=True)
    assert L.get("aerodynamics").cg_y_mm == 7.0


def test_a_corner_outside_the_box_is_the_euclidean_gap():
    k = KeepOut("k", "s", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    assert k.signed_distance_mm(np.array([1.5, 1.5, 0.5])) == pytest.approx(math.sqrt(0.5))
    assert k.signed_distance_mm(np.array([1.5, 0.5, 0.5])) == pytest.approx(0.5)
    back = KeepOut.from_dict(dict(name="k", owner_subsystem="s", lo_mm=[1, 1, 1], hi_mm=[0, 0, 0], bogus=1))
    assert back.lo_mm == (0.0, 0.0, 0.0) and back.hi_mm == (1.0, 1.0, 1.0)


def test_the_clearance_band_edge_is_exact():
    g = GeometryLedger()
    g.set_point(MountPoint("p", (0.0, 0.0, 3.0), "brakes", min_clearance_mm=2.5))
    g.set_keepout(KeepOut("k", "suspension", (-1.0, -1.0, -1.0), (1.0, 1.0, 1.0)))
    assert g.check_clashes()[0].severity == Severity.WARN                   # 2.0 mm against 2.5 mm
    c = 2.0 + 1e-9
    while c - 1e-9 > 2.0:
        c = np.nextafter(c, -np.inf)
    while c - 1e-9 < 2.0:
        c = np.nextafter(c, np.inf)
    assert c - 1e-9 == 2.0
    g.points["p"].min_clearance_mm = float(c)
    assert g.check_clashes()[0].severity == Severity.OK
