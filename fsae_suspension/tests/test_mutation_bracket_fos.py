"""suspension/bracket_fos.py: every failure mode against its hand formula (gross and net section, root bending
6 P a / (w t^2), bearing P / (n d t), tear-out over two planes of e - d/2, fillet throat 0.707 h with the bending couple
6 M / (throat L^2)), the verdict bands, the geometry and material refusals with their wording, the 4130 weld notes, the
material comparison and the report."""
import math

import pytest

from suspension import bracket_fos as bf
from suspension.interfaces import Severity

M1018 = "Steel 1018 CR (cold-rolled)"


def test_the_material_library():
    m = bf.MATERIALS[M1018]
    assert (m.yield_MPa, m.uts_MPa, m.rho_kg_m3, m.shear_yield_factor, m.weldable_no_pwht) == (370, 440, 7870, 0.577,
                                                                                               True)
    assert m.shear_yield_MPa == pytest.approx(370 * 0.577)
    want = {"Steel 1018 HR (hot-rolled)": (235, 400, 7870, True), "Steel 4130 (normalized tube)": (460, 670, 7850, False),
            "Steel 4130 (as-welded, no PWHT)": (360, 560, 7850, False), "Aluminium 6061-T6": (276, 310, 2700, False),
            "Aluminium 6061-T4 (welded HAZ)": (110, 240, 2700, True), "Aluminium 7075-T6": (503, 572, 2810, False)}
    for k, (y, u, rho, w) in want.items():
        mm = bf.MATERIALS[k]
        assert (mm.name, mm.yield_MPa, mm.uts_MPa, mm.rho_kg_m3, mm.weldable_no_pwht) == (k, y, u, rho, w)
    assert len(bf.MATERIALS) == 7 and all(bf.MATERIALS[k].weld_note for k in bf.MATERIALS)


def _br(**kw):
    base = dict(name="seat", material=M1018, width_mm=30.0, thickness_mm=3.0, P_N=2000.0)
    base.update(kw)
    return bf.Bracket(**base)


def _mode(res, name):
    return next(m for m in res.modes if m.mode == name)


def test_every_mode_by_hand():
    br = _br(lever_arm_mm=25.0, hole_dia_mm=8.4, edge_dist_mm=12.0, weld_leg_mm=3.0, weld_length_mm=60.0, n_bolts=2)
    r = bf.screen_bracket(br)
    Sy, Ss = 370.0, 370.0 * 0.577
    P, w, t = 2000.0, 30.0, 3.0
    expect = {"direct shear (section)": (P / (w * t), Ss),
              "direct shear (net section at hole)": (P / ((w - 8.4) * t), Ss),
              "root bending": (P * 25 * 1.5 / (w * t ** 3 / 12), Sy),
              "bolt-hole bearing": (P / 2 / (8.4 * t), Sy),
              "hole tear-out (shear)": (P / 2 / (2 * (12 - 4.2) * t), Ss)}
    throat = 0.707 * 3.0
    expect["weld throat shear"] = (math.hypot(P / (throat * 60), 6 * P * 25 / (throat * 3600)), Ss)
    assert [m.mode for m in r.modes] == list(expect)
    for name, (s, allow) in expect.items():
        m = _mode(r, name)
        assert (m.stress_MPa, m.allow_MPa, m.fos) == pytest.approx((s, allow, allow / s))
    gov = min(r.modes, key=lambda m: m.fos)
    assert gov.governs and sum(m.governs for m in r.modes) == 1 and r.governing_mode == gov.mode
    assert r.min_fos == gov.fos and r.screening_only is True and r.material == M1018
    t2 = bf.screen_bracket(_br(load_is_shear=False, hole_dia_mm=6.0))
    assert [m.mode for m in t2.modes] == ["direct tension (section)", "direct tension (net section at hole)",
                                          "bolt-hole bearing"]
    assert _mode(t2, "direct tension (net section at hole)").stress_MPa == pytest.approx(2000 / (24 * 3))
    assert _mode(t2, "direct tension (section)").allow_MPa == 370.0
    assert _mode(t2, "bolt-hole bearing").stress_MPa == pytest.approx(2000 / (6 * 3))      # one bolt by default
    nolev = bf.screen_bracket(_br(weld_leg_mm=3.0, weld_length_mm=60.0, P_N=-2000.0))
    assert _mode(nolev, "weld throat shear").stress_MPa == pytest.approx(2000 / (0.707 * 3 * 60))
    zero = bf.screen_bracket(_br(P_N=0.0, lever_arm_mm=10.0, hole_dia_mm=5.0, edge_dist_mm=10.0, weld_leg_mm=2.0,
                                 weld_length_mm=20.0))
    assert all(m.fos == math.inf for m in zero.modes) and zero.verdict == "PASS"
    assert bf.screen_bracket(_br(n_bolts=0, hole_dia_mm=6.0)).modes[-1].stress_MPa == pytest.approx(2000 / 18)


def test_the_verdict_bands_and_their_findings():
    def at(fos, **kw):
        P = 370 * 0.577 * 90.0 / fos                 # direct shear governs with no other mode
        return bf.screen_bracket(_br(P_N=P, **kw), **{k: v for k, v in kw.items() if k in ()})
    fail, tight, ok = at(1.4), at(1.6), at(1.8)
    assert (fail.verdict, tight.verdict, ok.verdict) == ("FAIL", "TIGHT", "PASS")
    assert at(1.5).verdict == "TIGHT" and bf.screen_bracket(_br(P_N=370 * 0.577 * 90 / 1.725)).verdict == "PASS"
    assert bf.screen_bracket(_br(P_N=370 * 0.577 * 90 / 1.7), tight_band=0.2).verdict == "TIGHT"
    assert bf.screen_bracket(_br(P_N=370 * 0.577 * 90 / 1.9), fos_target=2.0).verdict == "FAIL"
    f = fail.findings[-1]
    assert (f.check, f.severity, f.subsystems) == ("bracket-fos", Severity.FAIL, ["chassis"])
    assert f.message == (f"chassis bracket 'seat' FAILS the FoS rule: governing mode is direct shear (section) at FoS "
                         f"1.40 vs required 1.50 ({M1018}) (estimated geometry/load). Thicken/widen, shorten the lever, "
                         f"add weld/bolt area, or move to a stronger material before fab.")
    assert f.detail == pytest.approx(dict(bracket="seat", material=M1018, governing_mode="direct shear (section)",
                                          fos=1.4, fos_target=1.5, stress_MPa=fail.modes[0].stress_MPa,
                                          allow_MPa=370 * 0.577, estimate=True))
    w = tight.findings[-1]
    assert w.severity == Severity.WARN and w.message == (
        f"chassis bracket 'seat' passes but is TIGHT: direct shear (section) at FoS 1.60, just over the 1.50 floor "
        f"({M1018}) (estimated geometry/load). No margin for the rushed-manufacturing variation the FoS buffer is meant "
        f"to absorb — worth one more pass or an FEA confirm.")
    assert w.detail == pytest.approx(dict(bracket="seat", material=M1018, governing_mode="direct shear (section)",
                                          fos=1.6, fos_target=1.5, estimate=True))
    o = bf.screen_bracket(_br(P_N=370 * 0.577 * 90 / 1.8, is_estimate=False)).findings[-1]
    assert o.severity == Severity.OK and o.message == (
        f"chassis bracket 'seat' meets FoS ≥ 1.50 on yield (governing direct shear (section), FoS 1.80, {M1018}). "
        f"Screening only — confirm with SolidWorks/FEA before sign-off.")
    assert o.detail == pytest.approx(dict(bracket="seat", material=M1018, governing_mode="direct shear (section)",
                                          fos=1.8, fos_target=1.5))
    assert "(estimated" not in bf.screen_bracket(_br(P_N=50000.0, is_estimate=False)).findings[-1].message


def test_refusals_and_geometry_findings():
    r = bf.screen_bracket(_br(material="Unobtainium"))
    assert (r.verdict, r.governing_mode, r.min_fos, r.notes) == ("INVALID", "material", 0.0, ["material not in library"])
    assert (r.name, r.material) == (_br().name, "Unobtainium")
    assert r.findings[0].severity == Severity.MISSING and r.findings[0].message == (
        "Unknown material 'Unobtainium' — pick one from the bracket material library.")
    for kw in (dict(width_mm=0.0), dict(thickness_mm=-1.0)):
        g = bf.screen_bracket(_br(**kw))
        assert (g.verdict, g.governing_mode, g.notes) == ("INVALID", "geometry", ["non-physical cross-section"])
        assert (g.name, g.material) == (_br().name, _br().material)
        assert g.findings[0].message == "Bracket width and thickness must be > 0 to screen."
        assert g.findings[0].subsystems == ["chassis"] and g.fos_target == 1.5
    wide = bf.screen_bracket(_br(hole_dia_mm=30.0))
    assert wide.findings[0].message == ("Hole ⌀30 mm is as wide as or wider than the 30 mm tab — there is no net "
                                        "section left to carry load.") and wide.findings[0].severity == Severity.FAIL
    assert "direct shear (net section at hole)" not in [m.mode for m in wide.modes]
    brk = bf.screen_bracket(_br(hole_dia_mm=8.0, edge_dist_mm=4.0))
    assert brk.findings[0].message == ("Edge distance 4 mm is less than the hole radius (4 mm) — the hole breaks out "
                                       "through the free edge. There is no tear-out path to check; fix the geometry "
                                       "before reading any factor of safety here.")
    assert len(brk.findings) == 2 and "hole tear-out (shear)" not in [m.mode for m in brk.modes]


def test_weld_provenance_notes():
    assert bf.screen_bracket(_br()).notes == []
    n = bf.screen_bracket(_br(weld_leg_mm=3.0, weld_length_mm=40.0)).notes
    assert n == [bf.MATERIALS[M1018].weld_note]
    t = bf.screen_bracket(_br(material="Steel 4130 (normalized tube)", weld_leg_mm=3.0, weld_length_mm=40.0)).notes
    assert t == [bf.MATERIALS["Steel 4130 (normalized tube)"].weld_note,
                 "You're screening a WELDED 4130 bracket on parent-tube yield. If you won't post-weld heat-treat, "
                 "re-run on '4130 (as-welded, no PWHT)' — that's the apples-to-apples vs 1018 CR."]
    assert bf.screen_bracket(_br(material="Steel 4130 (normalized tube)")).notes == [
        bf.MATERIALS["Steel 4130 (normalized tube)"].weld_note]
    assert bf.screen_bracket(_br(material="Aluminium 6061-T6")).notes == [bf.MATERIALS["Aluminium 6061-T6"].weld_note]
    assert bf.screen_bracket(_br(material="Steel 4130 (as-welded, no PWHT)", weld_leg_mm=2.0,
                                 weld_length_mm=10.0)).notes == [bf.MATERIALS["Steel 4130 (as-welded, no PWHT)"].weld_note]


def test_round_trip_and_the_result_dict():
    br = _br(lever_arm_mm=10.0, set_by="me")
    assert bf.Bracket.from_dict({**br.as_dict(), "extra": 1}) == br
    res = bf.screen_bracket(br)
    d = res.as_dict()
    assert d["modes"][0]["mode"] == "direct shear (section)" and d["findings"][0]["severity"] == res.findings[0].severity.value
    raw = bf.BracketResult("x", "m", 1.0, "g", "PASS", 1.5, modes=[{"mode": "a"}], findings=[{"f": 1}])
    assert raw.as_dict()["modes"] == [{"mode": "a"}] and raw.as_dict()["findings"] == [{"f": 1}]
    assert bf.ModeResult("a", 1.0, 2.0, 2.0).as_dict() == {"mode": "a", "stress_MPa": 1.0, "allow_MPa": 2.0, "fos": 2.0,
                                                           "governs": False}


def test_the_material_comparison():
    br = _br(lever_arm_mm=20.0)
    rows = bf.compare_materials(br)
    assert len(rows) == 7 and [r["min_fos"] for r in rows] == sorted((r["min_fos"] for r in rows), reverse=True)
    for r in rows:
        s = bf.screen_bracket(bf.Bracket.from_dict({**br.as_dict(), "material": r["material"]}))
        m = bf.MATERIALS[r["material"]]
        assert r == {"material": m.name, "yield_MPa": m.yield_MPa, "min_fos": s.min_fos,
                     "governing_mode": s.governing_mode, "verdict": s.verdict, "density_kg_m3": m.rho_kg_m3,
                     "weldable_no_pwht": m.weldable_no_pwht}
    two = bf.compare_materials(br, [M1018, "nope", "Aluminium 7075-T6"], fos_target=3.0)
    assert [r["material"] for r in two] == ["Aluminium 7075-T6", M1018]
    assert two[1]["verdict"] == bf.screen_bracket(br, fos_target=3.0).verdict


def test_the_report():
    r = bf.screen_bracket(_br(lever_arm_mm=20.0, material="Steel 4130 (normalized tube)"))
    txt = bf.bracket_report(r).split("\n")
    assert txt[0] == "KinematiK bracket FoS screening — seat"
    assert txt[1] == "material: Steel 4130 (normalized tube)   target FoS (yield): 1.50"
    assert txt[2] == f"VERDICT: {r.verdict}   min FoS: {r.min_fos:.2f} (governing: {r.governing_mode})"
    assert txt[3] == "" and txt[4] == "mode,stress_MPa,allowable_MPa,FoS,governs"
    m = r.modes[1]
    assert txt[6] == f"{m.mode},{m.stress_MPa:.1f},{m.allow_MPa:.1f},{m.fos:.2f},<<"
    assert txt[5].endswith(",")
    assert txt[7:10] == ["", "notes:", f"  - {r.notes[0]}"]
    assert txt[-1].startswith("Screening only: standard closed-form hand checks (Shigley / AWS throat shear). NOT a "
                              "substitute") and txt[-2] == ""
    plain = bf.bracket_report(bf.screen_bracket(_br())).split("\n")
    assert "notes:" not in plain and len(plain) == 8
    d = bf.BracketResult("n", "m", 2.0, "a", "PASS", 1.5, modes=[bf.ModeResult("a", 1.0, 2.0, 2.0, True).as_dict()])
    assert bf.bracket_report(d).split("\n")[5] == "a,1.0,2.0,2.00,<<"


def _tag(f):
    return (f.check, f.severity, f.subsystems)


def test_every_finding_is_tagged_for_the_chassis_board():
    cases = [(_br(material="Unobtainium"), [Severity.MISSING]), (_br(width_mm=0.0), [Severity.MISSING]),
             (_br(hole_dia_mm=30.0), [Severity.FAIL, None]), (_br(hole_dia_mm=8.0, edge_dist_mm=4.0),
                                                                [Severity.FAIL, None]),
             (_br(P_N=50000.0), [Severity.FAIL]), (_br(P_N=1.0), [Severity.OK])]
    for br, sev in cases:
        fs = bf.screen_bracket(br).findings
        assert len(fs) == len(sev)
        for f, s in zip(fs, sev):
            assert f.check == "bracket-fos" and f.subsystems == ["chassis"] and (s is None or f.severity == s)
    Ssy = 370 * 0.577
    tight = bf.screen_bracket(_br(P_N=Ssy * 90.0 / 1.6))                  # direct shear FoS 1.6: inside the band
    assert tight.verdict == "TIGHT" and _tag(tight.findings[0]) == ("bracket-fos", Severity.WARN, ["chassis"])
    for res in (bf.screen_bracket(_br(material="Unobtainium", name="a")), bf.screen_bracket(_br(thickness_mm=0.0,
                                                                                               name="a"))):
        assert (res.name, res.material, res.min_fos, res.fos_target, res.modes) == ("a", res.material, 0.0, 1.5, [])
    assert bf.screen_bracket(_br(material="Unobtainium"), fos_target=2.0).fos_target == 2.0
    assert bf.screen_bracket(_br(thickness_mm=0.0), fos_target=2.0).fos_target == 2.0
    assert bf.screen_bracket(_br(thickness_mm=0.0)).material == M1018
    plain = bf.screen_bracket(_br(P_N=50000.0, is_estimate=False))
    g = plain.modes[0]
    assert plain.findings[-1].message == (
        f"chassis bracket 'seat' FAILS the FoS rule: governing mode is direct shear (section) at FoS {g.fos:.2f} vs "
        f"required 1.50 ({M1018}). Thicken/widen, shorten the lever, add weld/bolt area, or move to a stronger material "
        f"before fab.")
    assert plain.screening_only is True


def test_small_dimensions_count_and_non_positive_ones_skip_the_mode():
    tiny = _br(width_mm=0.8, thickness_mm=0.5, P_N=3.0, lever_arm_mm=0.5, hole_dia_mm=0.3, edge_dist_mm=0.9,
               weld_leg_mm=0.6, weld_length_mm=0.7, n_bolts=0)
    r = bf.screen_bracket(tiny)
    want = {"direct shear (section)": 3.0 / 0.4, "direct shear (net section at hole)": 3.0 / (0.5 * 0.5),
            "root bending": 3.0 * 0.5 * 0.25 / (0.8 * 0.125 / 12), "bolt-hole bearing": 3.0 / (0.3 * 0.5),
            "hole tear-out (shear)": 3.0 / (2 * 0.75 * 0.5)}
    throat = 0.707 * 0.6
    want["weld throat shear"] = math.hypot(3.0 / (throat * 0.7), 6 * 3.0 * 0.5 / (throat * 0.49))
    assert {m.mode: m.stress_MPa for m in r.modes} == pytest.approx(want)
    for m in r.modes:
        assert m.fos == pytest.approx(m.allow_MPa / m.stress_MPa)
    assert bf.screen_bracket(_br(width_mm=0.8, thickness_mm=0.5)).verdict != "INVALID"
    for kw in (dict(hole_dia_mm=-2.0, edge_dist_mm=10.0), dict(lever_arm_mm=-5.0), dict(hole_dia_mm=0.0,
               edge_dist_mm=10.0), dict(weld_leg_mm=-3.0, weld_length_mm=40.0),
               dict(weld_leg_mm=3.0, weld_length_mm=-40.0), dict(weld_leg_mm=3.0, weld_length_mm=0.0)):
        assert [m.mode for m in bf.screen_bracket(_br(**kw)).modes] == ["direct shear (section)"]
    neg_edge = bf.screen_bracket(_br(hole_dia_mm=6.0, edge_dist_mm=-10.0))
    assert "hole tear-out (shear)" not in [m.mode for m in neg_edge.modes] and len(neg_edge.findings) == 1
    near = bf.screen_bracket(_br(hole_dia_mm=8.0, edge_dist_mm=4.5))           # 0.5 mm of tear-out plane is a path
    assert _mode(near, "hole tear-out (shear)").stress_MPa == pytest.approx(2000.0 / (2 * 0.5 * 3.0))
    slim = bf.screen_bracket(_br(hole_dia_mm=29.5))                            # 0.5 mm of net section still carries
    assert _mode(slim, "direct shear (net section at hole)").stress_MPa == pytest.approx(2000.0 / 1.5)
    lever_only_weld = bf.screen_bracket(_br(weld_leg_mm=3.0, weld_length_mm=40.0, lever_arm_mm=-5.0))
    assert _mode(lever_only_weld, "weld throat shear").stress_MPa == pytest.approx(2000.0 / (0.707 * 3.0 * 40.0))


def test_a_light_load_and_no_load():
    full = dict(lever_arm_mm=20.0, hole_dia_mm=6.0, edge_dist_mm=12.0, weld_leg_mm=3.0, weld_length_mm=40.0)
    for shear in (True, False):
        light = bf.screen_bracket(_br(P_N=1.0, load_is_shear=shear, **full))
        assert len(light.modes) == 6
        for m in light.modes:
            assert 0.0 < m.stress_MPa < 1.0 and m.fos == pytest.approx(m.allow_MPa / m.stress_MPa)
        none = bf.screen_bracket(_br(P_N=0.0, load_is_shear=shear, **full))
        assert [m.fos for m in none.modes] == [math.inf] * 6 and none.verdict == "PASS"
    t = bf.screen_bracket(_br(P_N=900.0, load_is_shear=False, hole_dia_mm=6.0))
    assert (t.modes[0].mode, t.modes[0].stress_MPa, t.modes[0].allow_MPa, t.modes[0].fos) == (
        "direct tension (section)", pytest.approx(10.0), 370, pytest.approx(37.0))
    assert (t.modes[1].mode, t.modes[1].stress_MPa, t.modes[1].allow_MPa) == (
        "direct tension (net section at hole)", pytest.approx(900.0 / 72.0), 370)
    assert bf.screen_bracket(_br(P_N=-900.0, load_is_shear=False)).modes[0].stress_MPa == pytest.approx(10.0)


def test_bolts_share_the_bearing_and_the_tear_out():
    one = bf.screen_bracket(_br(hole_dia_mm=6.0, edge_dist_mm=12.0, n_bolts=1))
    zero = bf.screen_bracket(_br(hole_dia_mm=6.0, edge_dist_mm=12.0, n_bolts=0))
    assert _mode(one, "bolt-hole bearing").stress_MPa == pytest.approx(2000.0 / 18.0) == _mode(
        zero, "bolt-hole bearing").stress_MPa
    assert _mode(zero, "hole tear-out (shear)").stress_MPa == pytest.approx(2000.0 / (2 * 9.0 * 3.0))


def test_the_result_dict_keys_and_the_comparison_target():
    d = bf.screen_bracket(_br()).as_dict()
    assert set(d) == {"name", "material", "min_fos", "governing_mode", "verdict", "fos_target", "modes", "findings",
                      "screening_only", "notes"}
    br = _br(P_N=370 * 0.577 * 90.0 / 1.7)                                   # FoS 1.7 in 1018 CR
    row = next(r for r in bf.compare_materials(br) if r["material"] == M1018)
    assert row["verdict"] == "TIGHT"
    row = next(r for r in bf.compare_materials(br, fos_target=1.8) if r["material"] == M1018)
    assert row["verdict"] == "FAIL"


def test_the_report_caveat():
    txt = bf.bracket_report(bf.screen_bracket(_br())).split("\n")
    assert txt[-1] == ("Screening only: standard closed-form hand checks (Shigley / AWS throat shear). NOT a substitute "
                       "for the SolidWorks/Ansys FEA the design rule also requires — confirm the governing mode there "
                       "before sign-off.")
