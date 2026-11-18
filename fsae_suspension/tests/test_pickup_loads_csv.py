"""The pickup-loads CSV a chassis optimiser reads: every load case and link, the force on the chassis along the link."""
import csv
import io
import json
import pathlib

import numpy as np

from suspension import genesis_repro as gr

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _screen():
    d = json.loads((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    hp = gr.hp_from_dict(d["recorded"]["winner_hardpoints"])
    return gr.structural_screening(hp, axle="front")


def test_every_row_has_its_pickups_and_the_chassis_force_along_the_link():
    sc = _screen()
    text = gr.pickup_loads_csv(sc, "front corner")
    assert text.startswith("# KinematiK pickup loads for front corner. Frame: x rearward")
    rows = list(csv.DictReader(io.StringIO(text.split("\n", 1)[1])))
    assert len(rows) == len(sc["rows"]) and {r["member"] for r in rows} == set(sc["member_points"])
    for r in rows:
        a = np.array([float(r[f"inboard_{k}_mm"]) for k in "xyz"])
        b = np.array([float(r[f"outboard_{k}_mm"]) for k in "xyz"])
        f = np.array([float(r[f"chassis_F{k}_N"]) for k in "xyz"])
        F = float(r["force_N"])
        assert abs(np.linalg.norm(f) - abs(F)) < 0.2                       # the link's axial force, nothing more
        if abs(F) > 1:
            assert np.sign(np.dot(f, b - a)) == np.sign(F)                 # tension pulls the chassis outboard


def test_a_teams_own_load_cases_parse_and_drive_the_screen():
    import pytest
    cases = gr.parse_load_cases("# from Adams Car\nmax cornering, -800, 2600, 2400, 12\n\nmax braking, 3000, 0, 2900")
    assert [c.name for c in cases] == ["max cornering", "max braking"]
    assert (cases[0].Fx, cases[0].Fy, cases[0].Fz, cases[0].mz_Nmm) == (-800.0, 2600.0, 2400.0, 12000.0)
    for bad in ("max cornering, -800, 2600", "x, a, 1, 2", ", 1, 2, 3", "x, 1, 2, nan"):
        with pytest.raises(ValueError, match="line 1"):
            gr.parse_load_cases(bad)
    d = json.loads((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
    sc = gr.structural_screening(gr.hp_from_dict(d["recorded"]["winner_hardpoints"]), axle="front", load_cases=cases)
    assert {r["load_case"] for r in sc["rows"]} == {"max cornering", "max braking"}
    assert "max braking" in gr.pickup_loads_csv(sc)
