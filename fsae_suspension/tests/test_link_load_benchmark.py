# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Link loads against the kinematics benchmark tools (benchmarks/link_loads). The independent static solve runs here and
must sit inside the limits registered before it; the Adams View command files must be the ones the manifests generate;
a tool without results is NOT RUN, never a pass."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import independent_link_loads as ind                                        # noqa: E402
import link_load_benchmark as llb                                           # noqa: E402


def test_limits_are_the_registered_ones():
    reg = json.loads((llb.BENCH / "limits.json").read_text())
    got = {t: (v["abs_N"], v["rel"]) for t, v in reg["limits"].items()}
    assert got == {"independent": (0.5, 0.001), "adams_view": (0.5, 0.001), "lotus": (2.0, 0.005), "ard": (2.0, 0.005),
                   "motionsolve": (0.5, 0.001), "chrono": (0.5, 0.001), "matlab": (0.5, 0.001)}
    assert list(reg["load_cases"]) == ["bump", "corner", "brake", "corner_drive"]


@pytest.mark.parametrize("name", llb.CORNERS)
def test_kinematik_agrees_with_the_independent_solve(name):
    r = llb.compare("independent", name)
    assert r["status"] == "PASS" and r["ratio"] <= 1.0, r


def test_independent_result_does_not_depend_on_the_stiffnesses(monkeypatch):
    hp = llb._winner("front_v11_final")
    case = llb.CASES["corner_drive"]
    a = ind.member_forces(hp, case)
    monkeypatch.setattr(ind, "STIFFNESS", dict(zip(ind.MEMBERS, (6e7, 1e7, 5e7, 2e7, 4e7, 3e7))))
    b = ind.member_forces(hp, case)
    assert all(abs(a[m] - b[m]) < 0.05 for m in ind.MEMBERS)


def test_independent_solve_closes_vertical_equilibrium():
    """The six member forces on the upright plus the wheel load sum to zero."""
    import numpy as np
    hp = llb._winner("rear_v11_final")
    f = ind.member_forces(hp, llb.CASES["bump"])
    P = {k: np.asarray(v, float) for k, v in hp.items() if isinstance(v, list) and len(v) == 3}
    tot = sum(f[m] * (P[i] - P[o]) / np.linalg.norm(P[i] - P[o]) for m, (o, i) in ind.ENDS.items())
    assert np.allclose(tot + [0.0, 0.0, 1200.0], 0.0, atol=0.05)


@pytest.mark.parametrize("name", llb.CORNERS)
def test_adams_command_file_is_generated_from_the_manifest(name):
    assert (llb.BENCH / f"{name}.cmd").read_text() == llb.command_file(name)


@pytest.mark.parametrize("tool", ["adams_view", "motionsolve", "ard"])
def test_a_tool_without_results_is_not_run_not_passed(tool, tmp_path, monkeypatch):
    monkeypatch.setattr(llb, "BENCH", tmp_path)
    (tmp_path / "ard_results.csv").write_text(llb.csv_template("ard"))          # empty template
    assert llb.compare(tool, "front_v11_final")["status"] == "NOT RUN"


def test_a_wrong_hand_entered_value_fails(tmp_path, monkeypatch):
    monkeypatch.setattr(llb, "BENCH", tmp_path)
    kk = llb.kinematik("front_v11_final")
    rows = ["corner,case," + ",".join(llb.MEMBERS)]
    for c in llb.CASES:
        vals = [round(kk[c][m]) for m in llb.MEMBERS]
        if c == "brake":
            vals[4] += 25                                                       # tie rod off by 25 N
        rows.append(f"front_v11_final,{c}," + ",".join(str(v) for v in vals))
    (tmp_path / "ard_results.csv").write_text("\n".join(rows) + "\n")
    r = llb.compare("ard", "front_v11_final")
    assert r["status"] == "FAIL" and r["where"] == ("brake", "TR")


def test_an_adams_table_from_the_wrong_model_is_refused(tmp_path, monkeypatch):
    monkeypatch.setattr(llb, "BENCH", tmp_path)
    (tmp_path / "front_v11_final.tab").write_text('"rear_v11_final_loads_v1"\n\n"Time"\t"T_UF"\n0\t1\n')
    with pytest.raises(ValueError, match="rear_v11_final"):
        llb.compare("adams_view", "front_v11_final")


def test_the_arevalo_tables_do_not_balance_their_own_geometry():
    """The paper's obstacle row is a vertical wheel-centre load; no reading of its conventions makes it one."""
    import arevalo_link_load_check as ar
    assert ar.recovered_loads("front")[0][0] > 0.1 and ar.recovered_loads("rear")[0][0] > 0.3


@pytest.mark.parametrize("name", llb.CORNERS)
def test_motionsolve_dataset_and_inputs_sheet_are_generated_from_the_manifest(name):
    assert (llb.BENCH / "motionsolve" / f"{name}.adm").read_text() == llb.adm_file(name)
    assert (llb.BENCH / "motionsolve" / f"{name}.acf").read_text() == llb.acf_file(name)
    assert (llb.BENCH / "inputs.csv").read_text() == llb.inputs_sheet()


@pytest.mark.parametrize("name", llb.CORNERS)
def test_kinematik_agrees_with_project_chrono(name):
    """benchmarks/link_loads/chrono_results.csv is the output of scripts/chrono_benchmark.py (Project Chrono 9.0.1)."""
    assert llb.compare("chrono", name)["status"] == "PASS"


@pytest.mark.parametrize("name", llb.CORNERS)
def test_kinematik_agrees_with_simscape_multibody(name):
    """benchmarks/link_loads/matlab_results.csv is the output of scripts/matlab/kk_bench_simscape.m (MATLAB Simscape
    Multibody), run by the author."""
    assert llb.compare("matlab", name)["status"] == "PASS"


def _spreadsheet(model, rs, times, values):
    """An Adams View Spread Sheet export, as Adams 2026.1 writes it."""
    head = f'"(No Title)"\n\t"2026-11-"\t"5 10:25:14"\n\n" Result ID"\t"Result Type"\n"      {rs}"\t"  Variable"\n\n'
    head += f'".{model}.Last_Run.{rs}.TIME"\t".{model}.Last_Run.{rs}.Q"\n'
    return head + "".join(f"{t:.6e}\t{v:.6e}\n" for t, v in zip(times, values)) + "\n\n"


def test_adams_spreadsheet_exports_are_read_and_compared(tmp_path, monkeypatch):
    """The per-measure files the command file writes (adams/<corner>__T_xx.tab) are read; KinematiK's own numbers at
    Adams' 7 significant digits pass, one member off by 5 N fails."""
    kk = llb.kinematik("front_v11_final")
    (tmp_path / "adams").mkdir()
    for m in llb.MEMBERS:
        vals = [kk[c][m] + (5.0 if (m == "TR" and c == "brake") else 0.0) for c in llb.CASES]
        (tmp_path / "adams" / f"front_v11_final__T_{m}.tab").write_text(
            _spreadsheet("front_v11_final_loads_v1", f"T_{m}", range(4), vals))
    monkeypatch.setattr(llb, "BENCH", tmp_path)
    r = llb.compare("adams_view", "front_v11_final")
    assert r["status"] == "FAIL" and r["where"] == ("brake", "TR")
    kk_ok = {c: dict(kk[c]) for c in kk}
    (tmp_path / "adams" / "front_v11_final__T_TR.tab").write_text(
        _spreadsheet("front_v11_final_loads_v1", "T_TR", range(4), [kk_ok[c]["TR"] for c in llb.CASES]))
    assert llb.compare("adams_view", "front_v11_final")["status"] == "PASS"
