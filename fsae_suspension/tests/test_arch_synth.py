# ============================================================================
#  KinematiK — tests for suspension/arch_synth.py
#  Verifies the mixed discrete-continuous co-optimizer: correctness of the
#  NSGA-II primitives, real-solver integration, determinism, and Pareto sanity.
# ============================================================================
import numpy as np
import pytest

from suspension.arch_synth import (
    ArchitectureProblem, PointsModel, MassModel,
    fast_non_dominated_sort, crowding_distance, _dominates,
    synthesize, compare_architectures, tradeoff_table, evaluate_kinematics,
    default_discrete_space, default_continuous_space,
)
from suspension.kinematics import Hardpoints


# ---- NSGA-II primitives are mathematically correct ---------------------
def test_dominance_relation():
    assert _dominates(np.array([1.0, 1.0]), np.array([2.0, 2.0]))
    assert _dominates(np.array([1.0, 2.0]), np.array([1.0, 3.0]))
    assert not _dominates(np.array([1.0, 3.0]), np.array([2.0, 1.0]))  # trade-off
    assert not _dominates(np.array([1.0, 1.0]), np.array([1.0, 1.0]))  # equal


def test_non_dominated_sort_known_case():
    # Four points: three on a trade-off front, one dominated by all.
    objs = np.array([
        [1.0, 3.0],   # 0  front 0
        [2.0, 2.0],   # 1  front 0
        [3.0, 1.0],   # 2  front 0
        [4.0, 4.0],   # 3  front 1 (dominated by 0,1,2)
    ])
    fronts = fast_non_dominated_sort(objs)
    assert set(fronts[0]) == {0, 1, 2}
    assert fronts[1] == [3]


def test_crowding_distance_endpoints_infinite():
    objs = np.array([[1.0, 3.0], [2.0, 2.0], [3.0, 1.0]])
    d = crowding_distance(objs, [0, 1, 2])
    assert np.isinf(d[0]) and np.isinf(d[2])   # extremes preserved
    assert np.isfinite(d[1])                    # interior finite


# ---- economic models are transparent and monotone ----------------------
def test_mass_model_monotone_in_motors():
    m = MassModel()
    base = {"wheel_in": 13, "motors": 1, "pack_v": 400, "damper": "outboard"}
    m1 = m.mass(base)
    m4 = m.mass({**base, "motors": 4})
    assert m4 > m1                              # 4 motors heavier than 1


def test_points_model_penalises_bumpsteer():
    pm = PointsModel()
    arch = {"wheel_in": 13, "motors": 1, "pack_v": 400, "damper": "outboard"}
    good = pm.points(arch, 210.0, {"camber_gain_deg": 1.0, "bumpsteer_deg": 0.05})
    bad = pm.points(arch, 210.0, {"camber_gain_deg": 1.0, "bumpsteer_deg": 2.0})
    assert good > bad                           # more bump steer => fewer points


# ---- real solver integration -------------------------------------------
def test_evaluate_kinematics_on_default_geometry():
    kin = evaluate_kinematics(Hardpoints.default())
    assert kin["ok"] is True
    assert 0.0 <= kin["camber_gain_deg"] < 10.0     # sane magnitude
    assert kin["bumpsteer_deg"] >= 0.0


def test_degenerate_geometry_penalised_not_crashed():
    hp = Hardpoints.default()
    hp.upper_outer = hp.lower_outer.copy()      # collapse the upright -> degenerate
    kin = evaluate_kinematics(hp)
    # must not raise; returns a large-penalty sentinel
    assert "camber_gain_deg" in kin


# ---- problem glue -------------------------------------------------------
def test_decode_roundtrips_discrete_choices():
    prob = ArchitectureProblem()
    rng = np.random.default_rng(3)
    g = prob.random_genome(rng)
    arch, cont, hp = prob.decode(g)
    assert arch["wheel_in"] in (10, 13)
    assert arch["motors"] in (1, 2, 4)
    assert arch["pack_v"] in (400, 600)
    assert arch["damper"] in ("inboard", "outboard")
    assert isinstance(hp, Hardpoints)


def test_evaluate_returns_three_objectives():
    prob = ArchitectureProblem()
    c = prob.evaluate(prob.random_genome(np.random.default_rng(1)))
    assert c.objectives.shape == (3,)


# ---- end-to-end optimisation -------------------------------------------
def test_synthesize_runs_and_returns_pareto():
    res = synthesize(pop_size=16, generations=6, seed=0)
    assert len(res.pareto) >= 1
    # the Pareto set must be internally non-dominated
    objs = np.array([c.objectives for c in res.pareto])
    for i in range(len(objs)):
        for j in range(len(objs)):
            if i != j:
                assert not _dominates(objs[j], objs[i]), \
                    "Pareto front contains a dominated point"


def test_synthesize_is_deterministic():
    a = synthesize(pop_size=16, generations=5, seed=42)
    b = synthesize(pop_size=16, generations=5, seed=42)
    pa = sorted(c.points for c in a.pareto)
    pb = sorted(c.points for c in b.pareto)
    assert np.allclose(pa, pb), "same seed must give same front"


def test_compare_architectures_distinct_keys():
    res = synthesize(pop_size=24, generations=8, seed=7)
    rows = compare_architectures(res)
    keys = [(r["wheel_in"], r["motors"], r["pack_v"], r["damper"]) for r in rows]
    assert len(keys) == len(set(keys)), "each architecture row must be distinct"
    assert all(r["feasible"] for r in rows)


def test_tradeoff_table_shape():
    res = synthesize(pop_size=16, generations=5, seed=2)
    tbl = tradeoff_table(res)
    assert tbl and "points" in tbl[0] and "mass_kg" in tbl[0]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
