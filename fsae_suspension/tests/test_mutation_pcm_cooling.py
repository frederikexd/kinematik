"""suspension/pcm_cooling.py: the apparent-heat-capacity law (cp + L / window inside the melt window), the melt
fraction, the latent buffer and wax volume, the energy accounting that holds the hottest cell at the plateau until
the buffer is spent (heat above onset = C x the rising steps whose midpoint is above onset), the inverse sizing, and
the findings text. Stand-in pack results give hand-countable temperature histories."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import pcm_cooling as pc
from suspension.interfaces import Severity

M = pc.PCMMaterial(t_melt_c=45.0, melt_window_c=4.0, latent_heat_j_per_g=200.0, cp_solid_j_per_gk=2.0,
                   cp_liquid_j_per_gk=2.4, density_g_per_cc=0.8)


def _pack(T, t=None, hot=1, cells=3, ok=True, cal=False, mass=0.05, cp=900.0):
    T = np.asarray(T, float)
    hist = np.column_stack([T - 5.0, T, T - 3.0][:cells])
    res = NS(ok=ok, hottest_cell_index=hot, temp_history_c=hist, time_s=np.arange(len(T)) * 10.0 if t is None else t)
    return res, NS(cell=NS(mass_kg=mass, cp=cp, calibrated=cal), n_cells=420)


def test_the_material_law():
    d = pc.PCMMaterial()
    assert (d.t_melt_c, d.melt_window_c, d.latent_heat_j_per_g, d.cp_solid_j_per_gk, d.cp_liquid_j_per_gk,
            d.density_g_per_cc, d.calibrated, d.fitted_to) == (45.0, 4.0, 200.0, 2.0, 2.2, 0.90, False, "")
    assert pc.default_pcm() == d and M.latent_heat_j_per_kg() == 200000.0
    assert M.effective_cp_j_per_gk(42.0) == 2.0 and M.effective_cp_j_per_gk(47.5) == 2.4
    assert M.effective_cp_j_per_gk(43.0) == pytest.approx(2.2 + 50.0) == M.effective_cp_j_per_gk(47.0)
    assert M.effective_cp_j_per_gk(45.0) == pytest.approx(52.2)
    sharp = pc.PCMMaterial(melt_window_c=0.0)
    assert sharp.effective_cp_j_per_gk(45.0) == pytest.approx(2.1 + 200.0 / 1e-6)
    assert [M.melt_fraction_at(x) for x in (40.0, 43.0, 44.0, 45.0, 47.0, 50.0)] == pytest.approx(
        [0.0, 0.0, 0.25, 0.5, 1.0, 1.0])
    assert sharp.melt_fraction_at(45.0) == 0.0 and sharp.melt_fraction_at(45.1) == 1.0
    assert M.as_dict()["latent_heat_j_per_g"] == 200.0


def test_the_allocation():
    a = pc.PCMAllocation(M, mass_per_cell_g=12.0, set_by="cooling")
    assert a.latent_buffer_j_per_cell() == 2400.0 and a.volume_per_cell_cc() == pytest.approx(15.0)
    d = a.as_dict()
    assert d["material"] == M.as_dict() and d["latent_buffer_j_per_cell"] == 2400.0 and d["mass_per_cell_g"] == 12.0
    assert pc.PCMAllocation(M).mass_per_cell_g == 15.0 and pc.PCMAllocation(M).is_estimate is True
    assert pc.PCMAllocation(pc.PCMMaterial(density_g_per_cc=0.0), 1.0).volume_per_cell_cc() == pytest.approx(1e9)


def test_the_provenance_wording():
    assert pc._provenance(True, True) == "calibrated: cell + PCM parameters from rig/datasheet data"
    assert pc._provenance(False, False) == ("SYNTHESIZED — cell, PCM uncalibrated; use for ranking layouts and finding "
                                            "where the wax runs out, not absolute °C")
    assert pc._provenance(True, False).startswith("SYNTHESIZED — PCM uncalibrated")
    assert pc._provenance(False, True).startswith("SYNTHESIZED — cell uncalibrated")


T_RUN = [30.0, 38.0, 42.0, 44.0, 47.0, 50.0, 49.0, 53.0, 56.0]


def _above(T, onset=43.0, C=45.0):
    T = np.asarray(T)
    dT, mid = np.diff(T), 0.5 * (T[1:] + T[:-1])
    return np.cumsum(C * np.clip(dT, 0, None) * (mid >= onset))


def test_the_wax_holds_the_whole_run():
    res, lay = _pack(T_RUN)
    total = _above(T_RUN)[-1]                                         # 45 J/K x (2+3+3+4+3) = 675 J
    r = pc.evaluate_pcm_buffer(res, lay, pc.PCMAllocation(M, mass_per_cell_g=total / 200.0 + 0.01))
    assert r.ok and r.hold_time_s is None and r.fully_melted is False and r.warnings == []
    assert (r.peak_cell_c_with_pcm, r.peak_cell_c_without_pcm, r.plateau_temp_c) == (47.0, 56.0, 45.0)
    assert r.total_pcm_mass_kg == pytest.approx((total / 200 + 0.01) * 420 / 1000)
    assert r.total_pcm_volume_cc == pytest.approx((total / 200 + 0.01) / 0.8 * 420)
    assert r.synthesized is True and r.provenance == pc._provenance(False, False)
    exact = pc.evaluate_pcm_buffer(res, lay, pc.PCMAllocation(M, mass_per_cell_g=total / 200.0))
    assert exact.fully_melted is False                                # the buffer exactly covers it
    cool, lay2 = _pack([30.0, 35.0, 40.0, 41.0], cal=True)
    c = pc.evaluate_pcm_buffer(cool, lay2, pc.PCMAllocation(M, 5.0), material=pc.PCMMaterial(calibrated=True))
    assert c.peak_cell_c_with_pcm == 41.0 and c.synthesized is False and c.provenance.startswith("calibrated")
    low, lay3 = _pack([30.0, 44.0, 46.0, 40.0])
    lo = pc.evaluate_pcm_buffer(low, lay3, pc.PCMAllocation(M, 50.0))
    assert lo.peak_cell_c_with_pcm == 46.0                            # under the window top: its own peak


def test_the_wax_melts_out():
    res, lay = _pack(T_RUN)
    cq = _above(T_RUN)
    r = pc.evaluate_pcm_buffer(res, lay, pc.PCMAllocation(M, mass_per_cell_g=300.0 / 200.0))
    idx = min(max(int(np.searchsorted(cq, 300.0)), 0), len(T_RUN) - 2)
    first = int(np.argmax(np.asarray(T_RUN) >= 43.0))
    assert r.hold_time_s == pytest.approx(10.0 * (idx + 1) - 10.0 * first) and r.fully_melted is True
    assert r.peak_cell_c_with_pcm == 56.0 and r.warnings == [
        f"Wax fully melts {r.hold_time_s:.1f}s into the high-load phase — after that the plateau is gone and cells "
        f"resume climbing. Either add wax or bring airflow in before then."]
    late = pc.evaluate_pcm_buffer(res, lay, pc.PCMAllocation(M, mass_per_cell_g=(cq[-1] - 1.0) / 200.0))
    assert late.hold_time_s == pytest.approx(10.0 * (len(T_RUN) - 1) - 10.0 * first)
    none = pc.evaluate_pcm_buffer(res, lay, pc.PCMAllocation(M, mass_per_cell_g=0.0))
    assert (none.hold_time_s, none.fully_melted, none.peak_cell_c_with_pcm) == (0.0, True, 56.0)
    assert none.warnings == ["PCM mass is zero — no latent buffer; equals bare cell."]
    other = pc.evaluate_pcm_buffer(res, lay, pc.PCMAllocation(M, 1.0), material=pc.PCMMaterial(t_melt_c=60.0))
    assert other.plateau_temp_c == 60.0 and other.fully_melted is False


def test_failures_are_reported_not_raised():
    res, lay = _pack(T_RUN, ok=False)
    r0 = pc.evaluate_pcm_buffer(res, lay, pc.PCMAllocation(M))
    assert r0.ok is False and r0.warnings == ["bare pack-thermal result was not ok"] and r0.provenance == "run failed"
    short, lay = _pack([40.0])
    assert pc.evaluate_pcm_buffer(short, lay, pc.PCMAllocation(M)).warnings == ["temperature history too short"]
    bad, lay = _pack(T_RUN, hot=7)
    r = pc.evaluate_pcm_buffer(bad, lay, pc.PCMAllocation(M))
    assert r.ok is False and r.warnings[0].startswith("PCM evaluation crashed: IndexError(")
    f = pc.PCMResult.failed(["x"])
    assert (f.ok, f.hold_time_s, f.peak_cell_c_with_pcm, f.peak_cell_c_without_pcm, f.fully_melted,
            f.latent_buffer_j_per_cell, f.total_pcm_mass_kg, f.total_pcm_volume_cc, f.synthesized, f.provenance,
            f.warnings) == (False, None, None, None, False, 0.0, 0.0, 0.0, True, "run failed", ["x"])
    assert math.isnan(f.plateau_temp_c)
    zero_c, lay = _pack(T_RUN, mass=0.0)
    assert pc.evaluate_pcm_buffer(zero_c, lay, pc.PCMAllocation(M, 1e-6)).fully_melted is False      # 1e-9 J/K floor


def test_sizing_the_wax_for_a_hold_time():
    res, lay = _pack(T_RUN)
    cq = _above(T_RUN)
    first = int(np.argmax(np.asarray(T_RUN) >= 43.0))                 # index 3 (44 °C)
    r = pc.size_pcm_for_hold(res, lay, M, 25.0)
    end = min(max(int(np.searchsorted(np.arange(9) * 10.0, 30.0 + 25.0)), first), len(cq) - 1)
    q = cq[end] - cq[first - 1]
    assert r == {"ok": True, "warnings": [], "required_buffer_j_per_cell": pytest.approx(q),
                 "pcm_mass_per_cell_g": pytest.approx(q / 200), "pcm_volume_per_cell_cc": pytest.approx(q / 200 / 0.8),
                 "pack_pcm_mass_kg": pytest.approx(q / 200 * 420 / 1000),
                 "pack_pcm_volume_cc": pytest.approx(q / 200 / 0.8 * 420), "n_cells": 420, "synthesized": True,
                 "provenance": pc._provenance(False, False)}
    long = pc.size_pcm_for_hold(res, lay, M, 1e4)
    assert long["required_buffer_j_per_cell"] == pytest.approx(cq[-1] - cq[first - 1])
    hot0, lay0 = _pack([44.0, 46.0, 50.0, 52.0], cal=True)
    z = pc.size_pcm_for_hold(hot0, lay0, pc.PCMMaterial(calibrated=True, latent_heat_j_per_g=0.0,
                                                         density_g_per_cc=0.0), 15.0)
    q0 = _above([44.0, 46.0, 50.0, 52.0])[2]
    assert z["required_buffer_j_per_cell"] == pytest.approx(q0) and z["pcm_mass_per_cell_g"] == pytest.approx(q0 / 1e-9)
    assert z["pcm_volume_per_cell_cc"] == pytest.approx(q0 / 1e-9 / 1e-9) and z["synthesized"] is False
    cold, lay1 = _pack([20.0, 25.0, 30.0, 35.0])
    assert pc.size_pcm_for_hold(cold, lay1, M, 10.0)["required_buffer_j_per_cell"] == 0.0
    neg, lay1 = _pack([44.0, 50.0, 46.0, 45.0])
    assert pc.size_pcm_for_hold(neg, lay1, M, -50.0)["required_buffer_j_per_cell"] >= 0.0
    nok, lay1 = _pack(T_RUN, ok=False)
    assert pc.size_pcm_for_hold(nok, lay1, M, 10.0) == {"ok": False, "warnings": ["bare pack-thermal result was not ok"]}
    br, lay1 = _pack(T_RUN, hot=9)
    b = pc.size_pcm_for_hold(br, lay1, M, 10.0)
    assert b["ok"] is False and b["warnings"][0].startswith("PCM sizing crashed: IndexError(")


def test_the_findings():
    subs = ["cooling", "battery-pack"]
    f = pc.check_pcm(pc.PCMResult.failed(["a", "b"]))
    assert [(x.check, x.severity, x.message, x.subsystems) for x in f] == [
        ("pcm", Severity.MISSING, "PCM evaluation did not complete; a; b", subs)]
    base = dict(ok=True, peak_cell_c_with_pcm=47.0, peak_cell_c_without_pcm=58.4, plateau_temp_c=45.0,
                latent_buffer_j_per_cell=3000.0, total_pcm_mass_kg=6.3, total_pcm_volume_cc=7000.0)
    hold = pc.check_pcm(pc.PCMResult(hold_time_s=None, fully_melted=False, synthesized=False, **base))
    assert [(x.check, x.severity, x.subsystems) for x in hold] == [("pcm-hold", Severity.OK, subs),
                                                                   ("pcm-packaging", Severity.INFO, subs)]
    assert hold[0].message == ("The wax never fully melts over the run — it holds the hottest cell near 45°C for the "
                               "whole stint. (6.30kg / 7000cc of PCM in the pack.)")
    assert hold[0].detail == {"plateau_c": 45.0, "pack_pcm_kg": 6.3}
    assert hold[1].message == ("PCM volume to package: 7000cc (6.30kg) across the pack — check it fits the nylon holder "
                               "and counts against the mass budget.")
    long = pc.check_pcm(pc.PCMResult(hold_time_s=1600.0, fully_melted=True, **base))
    assert long[0].severity == Severity.OK and long[0].message == (
        "Wax holds for 1600s ≥ the 1500s stint before melting out (SYNTHESIZED — uncalibrated).")
    assert long[0].detail == {"hold_time_s": 1600.0}
    edge = pc.check_pcm(pc.PCMResult(hold_time_s=900.0, fully_melted=True, **base), endurance_time_s=900.0)
    assert edge[0].severity == Severity.OK
    short = pc.check_pcm(pc.PCMResult(hold_time_s=600.0, fully_melted=True, **base))
    assert short[0].severity == Severity.WARN and short[0].message == (
        "Wax fully melts 600s into the high-load phase — short of the 1500s stint. After melt-out the hottest cell "
        "resumes climbing toward 58°C (SYNTHESIZED — uncalibrated). Add wax or bring the fan in before melt-out.")
    assert short[0].detail == {"hold_time_s": 600.0, "peak_after_meltout_c": 58.4}
    nohold = pc.check_pcm(pc.PCMResult(hold_time_s=None, fully_melted=True, **{**base, "total_pcm_volume_cc": 0.0}))
    assert len(nohold) == 1 and nohold[0].detail["hold_time_s"] == 0.0 and "melts 0s" in nohold[0].message


def test_the_wax_runs_out_in_the_first_hot_step_and_onset_ties():
    T = [30.0, 43.0, 47.0, 50.0]                                      # 43 is exactly the onset
    res, lay = _pack(T)
    cq = _above(T)
    r = pc.evaluate_pcm_buffer(res, lay, pc.PCMAllocation(M, mass_per_cell_g=cq[1] * 0.5 / 200.0))
    assert r.hold_time_s == pytest.approx(10.0 * 2 - 10.0 * 1) and r.latent_buffer_j_per_cell == pytest.approx(
        cq[1] * 0.5)
    two, lay2 = _pack([44.0, 48.0])
    assert pc.evaluate_pcm_buffer(two, lay2, pc.PCMAllocation(M, 1.0)).ok is True
    cal, lay3 = _pack(T, cal=True)
    assert pc.evaluate_pcm_buffer(cal, lay3, pc.PCMAllocation(M, 1.0)).synthesized is True     # the wax is not
    z = pc.size_pcm_for_hold(cal, lay3, M, 15.0)
    assert z["synthesized"] is True and z["provenance"] == pc._provenance(True, False)
    z2 = pc.size_pcm_for_hold(res, lay, pc.PCMMaterial(calibrated=True), 15.0)
    assert z2["provenance"] == pc._provenance(False, True)
    s = pc.size_pcm_for_hold(res, lay, M, 15.0)
    first = 1
    end = min(max(int(np.searchsorted(np.arange(4) * 10.0, 10.0 + 15.0)), first), 2)
    assert s["required_buffer_j_per_cell"] == pytest.approx(cq[end] - cq[first - 1])
    later, lay4 = _pack([42.0, 50.0, 53.0, 56.0])                  # onset first reached at index 1, step 0 counts
    cl = _above([42.0, 50.0, 53.0, 56.0])
    assert cl[0] > 0
    s2 = pc.size_pcm_for_hold(later, lay4, M, 1e4)
    assert s2["required_buffer_j_per_cell"] == pytest.approx(cl[-1] - cl[0])
    zc, lay5 = _pack([44.0, 46.0, 50.0], mass=0.0)
    q = pc.size_pcm_for_hold(zc, lay5, M, 1e4)["required_buffer_j_per_cell"]
    assert q == pytest.approx(1e-9 * 6.0)


def test_small_windows_volumes_and_the_dictionaries():
    assert pc.PCMMaterial(melt_window_c=0.5).melt_fraction_at(45.125) == pytest.approx(0.75)
    d = pc.PCMAllocation(M, 2.0).as_dict()
    assert set(d) == {"material", "mass_per_cell_g", "set_by", "is_estimate", "latent_buffer_j_per_cell"}
    base = dict(ok=True, hold_time_s=None, peak_cell_c_with_pcm=47.0, peak_cell_c_without_pcm=58.4, plateau_temp_c=45.0,
                fully_melted=False, latent_buffer_j_per_cell=3000.0, total_pcm_mass_kg=0.0005, total_pcm_volume_cc=0.5)
    f = pc.check_pcm(pc.PCMResult(**base))
    assert [x.check for x in f] == ["pcm-hold", "pcm-packaging"]
    subs = ["cooling", "battery-pack"]
    for hold, sev in ((1600.0, "OK"), (600.0, "WARN")):
        g = pc.check_pcm(pc.PCMResult(**{**base, "hold_time_s": hold, "fully_melted": True}))[0]
        assert (g.check, g.severity.name, g.subsystems) == ("pcm-hold", sev, subs)
