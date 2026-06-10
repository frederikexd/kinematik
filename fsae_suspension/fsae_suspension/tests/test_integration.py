"""Tests for the generic multi-team part-vs-chassis interference checker."""
import numpy as np, trimesh, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from suspension import integration as ig


def _box(extents, at):
    b = trimesh.creation.box(extents=extents)
    b.apply_translation(at)
    return b


def test_part_inside_reference_collides():
    chassis = _box([600, 500, 400], [0, 0, 200])
    part = _box([200, 120, 300], [0, 0, 200])
    assert ig.interference_check(part, chassis)["verdict"] == "COLLISION"


def test_part_outside_is_clear():
    chassis = _box([600, 500, 400], [0, 0, 200])
    part = _box([200, 120, 300], [0, 500, 200])
    assert ig.interference_check(part, chassis)["verdict"] == "CLEAR"


def test_part_just_outside_is_tight():
    chassis = _box([600, 500, 400], [0, 0, 200])
    part = _box([100, 100, 100], [0, 303, 200])
    assert ig.interference_check(part, chassis, warn_mm=5)["verdict"] == "TIGHT"


def test_collision_fraction_bounds():
    chassis = _box([600, 500, 400], [0, 0, 200])
    part = _box([200, 120, 300], [0, 0, 200])
    f = ig.interference_check(part, chassis)["collision_fraction"]
    assert 0.0 <= f <= 1.0


def test_all_discord_teams_registered():
    expected = {"aerodynamics", "brakes", "chassis", "cooling",
                "data-acquisition", "electrics", "powertrain", "suspension"}
    assert expected.issubset(set(ig.TEAMS.keys()))
    for v in ig.TEAMS.values():
        assert v["color"].startswith("#") and len(v["color"]) == 7


def test_mass_estimate_from_density():
    part = _box([100, 100, 100], [0, 0, 0])
    rec = ig.part_record("cooling", "block", part, density_kg_m3=2700)
    assert rec.mass_g is not None and rec.mass_g > 0


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    p = 0
    for fn in fns:
        try:
            fn(); print(f"  PASS  {fn.__name__}"); p += 1
        except AssertionError as e:
            print(f"  FAIL  {fn.__name__}: {e}")
    print(f"\n{p}/{len(fns)} integration tests passed")
