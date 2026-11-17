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
