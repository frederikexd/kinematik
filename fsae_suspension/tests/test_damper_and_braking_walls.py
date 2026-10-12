# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The damper-length wall of the actuation synthesis and the braking compliance wall of InverseGenesis.

Damper: ActuationTargets.damper_extended_mm declares a real damper's eye-to-eye length at full extension; the spring
may not top out before full droop and must stay inside the usable stroke at full bump. Left as None the targets and the
residual are as before. Braking: compliance_toe_braking_deg is the elastokinematic toe change under a second declared
load case (SolvedPropertyBounds.elasto_braking), bounded like any solved property and written to a manifest only when
declared, so earlier manifests keep their hash.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from suspension import actuation as act, elastokinematics as ek, inverse_genesis as ig, robustness as rb
from suspension.genesis_repro import GenesisManifest, hp_from_dict, vehicle_load_cases

MAN = Path(__file__).resolve().parents[1] / "manifests"


def _w(name):
    return hp_from_dict(GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())
                        .recorded["winner_hardpoints"])


def test_damper_wall_is_inert_until_declared():
    hp = _w("front_v11_final")
    m = act.actuation_metrics(hp)
    t0 = act.ActuationTargets.derived()
    assert t0.damper_extended_mm is None
    r = act._residual(m, t0)
    assert r.shape == (11,) and r[-2:].tolist() == [0.0, 0.0]
    assert act._residual(act.ActuationMetrics(ok=False), t0).shape == (11,)


@pytest.mark.parametrize("extended,top_out,bottoms", [(190.0, True, False), (260.0, False, True), (200.0, False, False)])
def test_damper_wall_refuses_top_out_and_bottoming(extended, top_out, bottoms):
    # the final front's spring runs 197.2 mm at full droop and 157.2 mm at full bump. A 190 mm damper would top out
    # before full droop; a 260 mm one, closed to 80% of its 57 mm stroke, is still 214.4 mm long at full bump; the
    # declared 200 mm damper fits both ends
    m = act.actuation_metrics(_w("front_v11_final"))
    r = act._residual(m, act.ActuationTargets.derived(damper_extended_mm=extended, stroke_mm=57.0))
    assert (r[-2] > 0.0, r[-1] > 0.0) == (top_out, bottoms)


def test_damper_window_of_the_final_front():
    m = act.actuation_metrics(_w("front_v11_final"))
    ld, l0, lb = m.spring_length_mm
    assert ld > l0 > lb
    lo = 200.0 - 0.8 * 57.0 + (l0 - lb)          # shortest static length that keeps full bump inside 80% of the stroke
    hi = 200.0 - (ld - l0)                       # longest that keeps full droop short of top-out
    assert lo < l0 < hi
    assert l0 == pytest.approx(0.5 * (lo + hi), abs=0.1)   # centred in the window (scripts/final_corners_v3.py)


def test_overtravel_counts_the_damper_top_out_margin():
    hp = _w("rear_v11_final")
    free = rb.overtravel_check(hp)
    real = rb.overtravel_check(hp, damper_extended_mm=200.0)
    assert free["stroke_available_mm"] == 57.0
    assert real["stroke_available_mm"] == pytest.approx(57.0 - (200.0 - 197.59), abs=0.01)
    assert real["damper_bottoms_at_mm"] < free["damper_bottoms_at_mm"]


def _braking_spec(hp_manifest):
    pr = GenesisManifest.from_json((MAN / f"{hp_manifest}.recorded.genesis.json").read_text()).objects()[2].properties
    return pr


def test_braking_toe_property_matches_the_direct_solve():
    pr = _braking_spec("front_v11_final")
    hp = _w("front_v11_final")
    assert pr.elasto_braking is not None
    v = ig.properties_of(hp, pr, only=["compliance_toe_braking_deg"])["compliance_toe_braking_deg"]
    d = ek.solve_elastokinematic(hp, pr.elasto_braking, jacobian=False).change["toe"]
    assert v == pytest.approx(d, abs=1e-12)
    c = [c for c in vehicle_load_cases(axle="front", brake_bias_front=0.73) if c.name == "1.5g braking"][0]
    assert (pr.elasto_braking.Fx, pr.elasto_braking.Fz) == (pytest.approx(c.Fx), pytest.approx(c.Fz))
    assert v == pytest.approx(0.230, abs=0.0005)
    bound = [b for b in pr.bounds if b.prop == "compliance_toe_braking_deg"][0]
    assert (bound.lo, bound.hi) == (-0.25, 0.25)
    # a 60% left-right brake force difference steers the axle by 0.3 times the per-wheel toe: inside +/-0.08 deg
    assert 0.3 * bound.hi <= 0.08 and 0.3 * v < 0.08


def test_braking_bound_needs_its_load_case():
    with pytest.raises(ValueError):
        ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("compliance_toe_braking_deg", lo=-0.25, hi=0.25)],
                                elasto=ek.ElastoSpec(Fy=1000.0))


def test_elasto_braking_round_trips_and_is_absent_from_earlier_manifests():
    for name in ("front_v11_hairpin_walls", "front_v11_29deg_walls", "rear_v11_final"):
        d = json.loads((MAN / f"{name}.recorded.genesis.json").read_text())
        assert "elasto_braking" not in json.dumps(d["volume"])
    m = GenesisManifest.from_json((MAN / "front_v11_final.recorded.genesis.json").read_text())
    again = GenesisManifest.from_json(m.to_json())
    assert again.inputs_sha256 == m.inputs_sha256
    assert again.objects()[2].properties.elasto_braking.to_dict() == m.objects()[2].properties.elasto_braking.to_dict()


def test_node_brief_carries_the_final_corners_and_their_damper():
    b = json.loads((MAN / "chassisv11_node_brief.json").read_text())
    for axle, name in (("front", "front_v11_final"), ("rear", "rear_v11_final")):
        co = b["corners"][axle]
        hp = _w(name)
        for p, v in co["nodes_corner_mm"].items():
            assert np.allclose(v, getattr(hp, p), atol=5e-4)
        assert co["damper"]["unit"].startswith("Ohlins TTX25 MkII FSAE")
        assert co["damper"]["top_out_margin_at_full_droop_mm"] > 0 and co["damper"]["stroke_left_at_full_bump_mm"] > 0.2 * 57.0
    assert b["corners"]["front"]["properties"]["lower_wishbone_base_mm"] == pytest.approx(166.9, abs=0.05)
    assert not any(x.startswith("front braking compliance toe") for x in b["open_items"])


def test_springs_against_the_ohlins_catalogue():
    # Ohlins lists TTX25 MkII FSAE springs from 150 to 650 lb/in. The rear's spring is its 200 lb/in; the front's is
    # below the range (made to rate). With the 150 lb/in catalogue spring the front would run at 2.68 Hz and about
    # 12 Nm/deg less front bar keeps the roll-stiffness share; no hardpoint moves.
    lbin = 0.17512683
    k = {}
    for name, hz in (("front_v11_final", 2.6), ("rear_v11_final", 3.0)):
        m = act.actuation_metrics(_w(name))
        mass = 60.1 if name.startswith("front") else 66.1
        k[name] = (2 * np.pi * hz) ** 2 * mass / 1000.0 / m.mr_static ** 2  # spring rate, N/mm
    kf, kr = k["front_v11_final"], k["rear_v11_final"]
    assert round(kf / lbin) == 142 and kf < 150 * lbin
    assert round(kr / lbin) == 199 and 3.0 * (200 * lbin / kr) ** 0.5 == pytest.approx(3.01, abs=0.005)
    assert 2.6 * (150 * lbin / kf) ** 0.5 == pytest.approx(2.68, abs=0.005)
    mr = act.actuation_metrics(_w("front_v11_final")).mr_static
    dbar = (150 * lbin - kf) * mr ** 2 * 1000 * 1.21 ** 2 / 2 * np.pi / 180
    assert round(dbar) == 12
