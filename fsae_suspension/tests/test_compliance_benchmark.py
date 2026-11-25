"""Compliance steer and camber against an independent solve and OpenSees, both checked against Ansys VM4
(scripts/compliance_benchmark.py)."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
BENCH = ROOT / "benchmarks" / "compliance"


def test_the_stored_run_passed_every_case_and_vm4():
    res = json.loads((BENCH / "results.json").read_text())
    assert res["passed"]
    assert res["vm4"]["independent"]["passes"] and res["vm4"]["opensees"]["passes"]
    assert set(res["cases"]) == {"bump", "corner", "brake", "corner_drive"}
    assert all(all(r["passes"].values()) for r in res["cases"].values())


@pytest.mark.filterwarnings("ignore")
def test_both_references_reproduce_ansys_vm4():
    import compliance_benchmark as cb
    v = cb.vm4()
    for tool in ("independent", "opensees"):
        assert v[tool]["stress_psi"] == pytest.approx(10000.0, rel=1e-6)
        assert v[tool]["deflection_in"] == pytest.approx(-0.120, rel=1e-6)


@pytest.mark.filterwarnings("ignore")
@pytest.mark.parametrize("case", ["bump", "brake"])
def test_kinematik_matches_the_independent_solve_live(case):
    import compliance_benchmark as cb
    d = cb.CASES[case]
    k, i = cb.kinematik(d), cb.independent(d)
    assert k["converged"]
    for q in ("compliance_toe_deg", "compliance_camber_deg"):
        assert k[q] == pytest.approx(i[q], abs=1e-5)
    assert k["contact_patch_lateral_mm"] == pytest.approx(i["contact_patch_lateral_mm"], abs=1e-4)


def test_the_loaded_pose_holds_the_damper_length():
    """Before 1.7.13 the loaded pose pinned the lower ball joint height; the damper length must now be unchanged."""
    import compliance_benchmark as cb
    from suspension import loadpath as lp
    from suspension.compliance import CompliantCorner, MemberStiffness
    from suspension.genesis_repro import hp_from_dict
    cc = CompliantCorner(hp_from_dict(cb.CORNER), {m: MemberStiffness(k_direct=v) for m, v in cb.K.items()})
    r = cc.solve(lp.WheelLoad(Fx=1933.0, Fz=1161.0))
    kin = cc.rigid_kin
    L0 = kin.spring_length_at(kin.static)[0]
    assert abs(r.compliant.travel) > 1e-3                        # the wheel found its own height
    assert cc._spring_L0 == pytest.approx(L0)
