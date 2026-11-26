"""suspension/fusebox.py against re-derivations written here: the first-failure probabilities by a numpy trapezoid on
the documented grid, the pairwise closed form, the two fix levers by solving their own probability equation, every
verdict and incident message word for word, the charter seal, the serialisation defaults and the Markdown report."""
import hashlib
import json
import math
from statistics import NormalDist

import numpy as np
import pytest

from suspension import fusebox as fb
from suspension.fusebox import OverloadPath, PathElement, Severity, PathVerdict, IncidentVerdict
from suspension.proof_engine import EvidenceGrade as G, effective_rel_unc

N = NormalDist()
S1, S2, S3 = Severity.S1_FUSE_GRADE, Severity.S2_STRUCTURAL, Severity.S3_FORBIDDEN


def _el(key, fos, grade=G.MODELLED, sev=S2, cost=100.0, days=1.0, **kw):
    return PathElement(key, key.upper(), fos, grade, sev, cost, days, **kw)


def _probs(els, n=4001):
    """The documented quadrature: trapezoid on n points over [min(mu - 6 sd), max(mu + 6 sd)], renormalised."""
    mu = np.array([e.fos for e in els])
    sd = np.array([max(e.fos * effective_rel_unc(e.grade, e.age_days), 1e-12) for e in els])
    x = np.linspace((mu - 6 * sd).min(), (mu + 6 * sd).max(), max(n, 101))
    pdf = np.array([[NormalDist(m, s).pdf(t) for t in x] for m, s in zip(mu, sd)])
    sv = np.array([[1 - NormalDist(m, s).cdf(t) for t in x] for m, s in zip(mu, sd)])
    w = np.full(x.size, x[1] - x[0])
    w[0] *= 0.5
    w[-1] *= 0.5
    raw = [float(np.sum(w * pdf[i] * np.prod(np.delete(sv, i, axis=0), axis=0))) for i in range(len(els))]
    return {e.key: r / sum(raw) for e, r in zip(els, raw)}


def test_the_element_and_its_band():
    e = PathElement("k", "K", 1.5)
    assert (e.grade, e.severity, e.replace_cost_usd, e.downtime_days, e.age_days, e.note) == (
        G.ESTIMATE, None, 0.0, 0.0, 0.0, "")
    old = _el("o", 2.0, G.MODELLED, age_days=200.0)
    assert old.rel_unc == effective_rel_unc(G.MODELLED, 200.0) > 0.10
    assert old.mu == 2.0 and old.sigma == pytest.approx(2.0 * old.rel_unc, rel=1e-15)
    with pytest.raises(ValueError, match=r"^z: FoS must be > 0$"):
        PathElement("z", "Z", 0.0)
    assert PathElement("t", "T", 1e-9).fos == 1e-9


def test_serialisation_with_and_without_the_optional_fields():
    e = _el("a", 1.4, G.MEASURED, S1, 12.5, 0.25, age_days=3.0, note="n")
    d = e.as_dict()
    assert d == {"key": "a", "label": "A", "fos": 1.4, "grade": "measured", "severity": "S1",
                 "replace_cost_usd": 12.5, "downtime_days": 0.25, "age_days": 3.0, "note": "n"}
    assert PathElement.from_dict(d) == e
    assert PathElement("b", "B", 1.0).as_dict()["severity"] is None
    bare = PathElement.from_dict({"key": "b", "label": "B", "fos": 1.2, "severity": "", "extra": 1})
    assert (bare.grade, bare.severity) == (G.ESTIMATE, None)
    assert PathElement.from_dict({"key": "c", "label": "C", "fos": 1.2})
    p = OverloadPath("p", "P", "story", [e], "a")
    assert p.as_dict() == {"key": "p", "label": "P", "story": "story", "designated_fuse_key": "a",
                           "elements": [d]}
    assert OverloadPath.from_dict(p.as_dict()) == p
    q = OverloadPath.from_dict({"key": "q"})
    assert (q.label, q.story, q.designated_fuse_key, q.elements) == ("q", "", "", [])
    assert p.element("a") is e and p.element("zz") is None


def test_pairwise_and_its_degenerate_bands():
    assert fb.pairwise_first(1.0, 0.1, 1.3, 0.2) == N.cdf(0.3 / math.hypot(0.1, 0.2))
    assert fb.pairwise_first(1.0, 0.0, 1.3, 0.0) == 1.0
    assert fb.pairwise_first(1.3, 0.0, 1.0, 0.0) == 0.0
    assert fb.pairwise_first(1.3, 0.0, 1.3, 0.0) == 0.5


@pytest.mark.parametrize("n", [101, 50, 4001])
def test_first_failure_probabilities_on_the_documented_grid(n):
    els = [_el("a", 1.3, G.MODELLED), _el("b", 1.5, G.ESTIMATE), _el("c", 1.7, G.GUESS)]
    got = fb.first_failure_probs(els, grid_points=n)
    want = _probs(els, n)
    assert list(got) == ["a", "b", "c"]
    for k in want:
        assert got[k] == pytest.approx(want[k], rel=1e-9, abs=1e-12)


def test_the_default_grid_and_the_small_cases():
    els = [_el("a", 1.3, G.MODELLED), _el("b", 1.5, G.ESTIMATE)]
    assert fb.GRID_POINTS == 4001
    assert fb.first_failure_probs(els) == fb.first_failure_probs(els, 4001) != fb.first_failure_probs(els, 101)
    assert fb.first_failure_probs([]) == {}
    assert fb.first_failure_probs([els[0]]) == {"a": 1.0}


def test_a_survival_that_underflows_takes_the_product_of_the_others():
    els = [_el("a", 1.0, G.VERIFIED), _el("b", 1.6, G.VERIFIED), _el("c", 1.7, G.VERIFIED)]
    got = fb.first_failure_probs(els, grid_points=301)
    want = _probs(els, 301)
    for k in want:
        assert got[k] == pytest.approx(want[k], rel=1e-9, abs=1e-15)
    assert fb._prod_except([("a", 0.5), ("b", 0.25), ("c", 0.0)], 2) == 0.125
    assert fb._prod_except([("a", 0.5), ("b", 0.25)], 0) == 0.25


def _path(els, fuse):
    return OverloadPath("p", "Path P", "A story.", els, fuse)


def test_an_empty_path():
    a = fb.audit_path(OverloadPath("e", "Empty", "s", [], "x"))
    assert a == fb.PathAudit("e", PathVerdict.UNFUSED, {}, "", 0.0, "x", 0.0, [], [], 0.0, 0.0, 0.0, 0.0,
                             ["path 'Empty' has no declared elements — nothing here is audited"],
                             "No elements declared: this chain's first failure is completely unexamined.")


def test_fused():
    f, r = _el("f", 1.2, G.MEASURED, S1, 40.0, 0.5), _el("r", 1.8, G.MEASURED, S2, 900.0, 9.0)
    a = fb.audit_path(_path([f, r], "f"))
    p = _probs([f, r])
    assert a.verdict is PathVerdict.FUSED and a.leader_key == "f" and a.fuse_key == "f"
    assert a.fuse_p == pytest.approx(p["f"], rel=1e-9) and a.leader_p == a.fuse_p
    assert a.headline == (f"'F' fails first with {a.fuse_p:.0%} probability (charter asks 90%). The car has "
                          "chosen its victim.")
    assert a.expected_cost_usd == pytest.approx(p["f"] * 40 + p["r"] * 900, rel=1e-9)
    assert a.expected_downtime_days == pytest.approx(p["f"] * 0.5 + p["r"] * 9, rel=1e-9)
    assert (a.fuse_cost_usd, a.fuse_downtime_days, a.blind_spots, a.forbidden_hits) == (40.0, 0.5, [], [])
    assert a.contenders == ["f"]


def test_a_fused_lead_below_the_asked_confidence_is_a_coin_flip():
    f, r = _el("f", 1.2, G.MODELLED, S1, 40.0, 0.5), _el("r", 1.35, G.MODELLED, S2, 900.0, 9.0)
    a = fb.audit_path(_path([f, r], "f"))
    p = _probs([f, r])
    assert 0.5 < p["f"] < 0.9 and a.verdict is PathVerdict.COIN_FLIP
    assert a.contenders == ["f", "r"]
    assert a.headline == (f"The ordering is undetermined at the deck's own σ: 'F' {p['f']:.0%}, 'R' "
                          f"{p['r']:.0%}. The designated fuse holds {p['f']:.0%} against a 90% charter — the "
                          "first failure is a weighted coin flip, not a decision.")
    assert fb.audit_path(_path([f, r], "f"), confidence=a.fuse_p).verdict is PathVerdict.FUSED
    assert fb.audit_path(_path([f, r], "f"), confidence=a.fuse_p * (1 + 1e-12)).verdict is PathVerdict.COIN_FLIP


def test_inverted():
    f = _el("f", 1.6, G.MODELLED, S1, 40.0, 0.5)
    r = _el("r", 1.2, G.MODELLED, S2, 1234.0, 12.25)
    o = _el("o", 2.4, G.GUESS, None, 10.0, 1.0)
    a = fb.audit_path(_path([f, r, o], "f"))
    p = _probs([f, r, o])
    assert a.verdict is PathVerdict.INVERTED and a.leader_key == "r"
    assert a.headline == (f"Most likely first failure is 'R' (S2, {p['r']:.0%}) — not the designated fuse "
                          f"({p['f']:.0%}). Conditional on the hit, that swaps a $40/0.5-day repair for "
                          "$1,234/12.2 days.")
    assert a.blind_spots == ["'O' has no severity class — treated as structural for verdicts, listed here so "
                             "the gap is never silent"]
    # a leader with no severity class is structural for the verdict
    r2 = _el("r", 1.2, G.MODELLED, None, 1234.0, 12.25)
    a2 = fb.audit_path(_path([f, r2], "f"))
    assert a2.verdict is PathVerdict.INVERTED and "'R' (S2, " in a2.headline


def test_another_fuse_grade_leader_is_a_coin_flip_and_a_missing_fuse_too():
    f, g = _el("f", 1.6, G.MODELLED, S1, 40.0, 0.5), _el("g", 1.2, G.MODELLED, S1, 30.0, 0.2)
    a = fb.audit_path(_path([f, g], "f"))
    assert a.verdict is PathVerdict.COIN_FLIP and a.leader_key == "g"
    lone = fb.audit_path(_path([g, _el("s", 1.9, G.MODELLED, S2)], "nope"))
    assert lone.verdict is PathVerdict.INVERTED or lone.verdict is PathVerdict.COIN_FLIP
    assert (lone.fuse_p, lone.fuse_cost_usd, lone.fuse_downtime_days) == (0.0, 0.0, 0.0)
    assert fb.audit_path(_path([g, _el("s", 1.9, G.MEASURED, S2)], "g")).verdict is PathVerdict.FUSED


def test_unfused_and_breach():
    a = fb.audit_path(_path([_el("a", 1.3), _el("b", 1.6)], ""))
    assert a.verdict is PathVerdict.UNFUSED and a.headline == (
        "No fuse-grade (S1) element exists on this chain — whatever breaks first is expensive by construction. "
        "Designate or add a sacrificial element.")
    f = _el("f", 1.3, G.MODELLED, S1)
    x = _el("x", 1.5, G.GUESS, S3)
    y = _el("y", 1.6, G.ESTIMATE, S3)
    els = [f, x, y]
    p = _probs(els)
    b = fb.audit_path(_path(els, "f"))
    assert b.verdict is PathVerdict.BREACH_RISK
    assert [k for k, _ in b.forbidden_hits] == ["x", "y"] and p["x"] > p["y"] > 0.01
    assert b.headline == (f"'X' is forbidden-first (S3) yet carries {p['x']:.1%} first-failure probability — above "
                          "the 1% the charter tolerates. Its capacity is GUESS grade (±40%); sharpening that band "
                          "may retire the breach without metal.")
    assert fb.audit_path(_path(els, "f"), forbidden_p=b.probs["y"]).forbidden_hits == [("x", b.probs["x"])]
    assert fb.audit_path(_path(els, "f"), forbidden_p=b.probs["x"]).forbidden_hits == []
    assert b.contenders == sorted([k for k in p if p[k] >= 0.10], key=lambda k: -p[k])


def test_contenders_start_at_ten_percent():
    els = [_el("a", 1.3, G.MODELLED, S1), _el("b", 1.45, G.MODELLED), _el("c", 1.9, G.ESTIMATE)]
    p = fb.first_failure_probs(els)
    a = fb.audit_path(_path(els, "a"))
    assert a.contenders == [k for k in sorted(p, key=lambda k: -p[k]) if p[k] >= 0.10]
    t = sorted(p.values())[1]
    els2 = els
    assert t > 0.0
    exact = fb.audit_path(_path(els2, "a"))
    assert len(exact.contenders) == sum(v >= 0.10 for v in p.values())


def _solve(mu, sd, r, z, lower):
    """Root of Phi(d / sqrt(...)) = c by bisection on the probability itself."""
    lo, hi = (1e-6, mu) if lower else (mu, 50.0 * mu)
    for _ in range(200):
        m = 0.5 * (lo + hi)
        p = (N.cdf((mu - m) / math.hypot(r * m, sd)) if lower else N.cdf((m - mu) / math.hypot(sd, r * m)))
        if (p > N.cdf(z)) == lower:
            lo = m
        else:
            hi = m
    return 0.5 * (lo + hi)


def test_the_two_fix_levers_solve_their_own_probability():
    z = N.inv_cdf(0.9)
    m = fb._solve_lower_fuse(1.5, 0.15, 0.1, z)
    assert m == pytest.approx(_solve(1.5, 0.15, 0.1, z, True), rel=1e-9)
    assert N.cdf((1.5 - m) / math.hypot(0.1 * m, 0.15)) == pytest.approx(0.9, abs=1e-12)
    h = fb._solve_raise_rival(1.2, 0.12, 0.2, z)
    assert h == pytest.approx(_solve(1.2, 0.12, 0.2, z, False), rel=1e-9)
    assert N.cdf((h - 1.2) / math.hypot(0.12, 0.2 * h)) == pytest.approx(0.9, abs=1e-12)
    assert fb._solve_lower_fuse(1.5, 0.15, 1.0 / z, z) is None
    assert fb._solve_raise_rival(1.2, 0.12, 1.0 / z, z) is None
    assert fb._solve_lower_fuse(0.1, 5.0, 0.01, z) is None              # the root would be below zero
    assert fb._solve_lower_fuse(1.0, 0.0, 0.0, 0.0) == 1.0


def test_prescriptions_word_for_word():
    fuse = _el("f", 1.5, G.MODELLED, S1)
    near = _el("n", 1.75, G.ESTIMATE, S2)                                # (a) feasible, (c) MEASURED
    wide = _el("w", 1.9, G.GUESS, S2)                                    # (c) MODELLED is enough
    safe = _el("s", 3.5, G.MEASURED, S2)                                 # already dominated: no prescription
    out = fb.prescribe(_path([fuse, near, wide, safe], "f"))
    z = N.inv_cdf(0.9)
    assert [p.rival_key for p in out] == sorted(["n", "w"], key=lambda k: fb.pairwise_first(
        1.5, 0.15, *(lambda e: (e.mu, e.sigma))(near if k == "n" else wide)))
    pn = next(p for p in out if p.rival_key == "n")
    assert pn.pair_p_now == pytest.approx(fb.pairwise_first(1.5, 0.15, 1.75, 0.35), rel=1e-14)
    m = fb._solve_lower_fuse(1.75, 0.35, 0.1, z)
    assert pn.lower_fuse_fos_to == round(m, 3) and pn.lower_fuse_note == (
        f"soften 'F' to FoS {m:.2f} at this path's reference load")
    h = fb._solve_raise_rival(1.5, 0.15, 0.2, z)
    assert pn.raise_rival_fos_to == round(h, 3) and pn.raise_rival_note == f"stiffen 'N' to FoS {h:.2f}"
    assert pn.sharpen_rival_to is G.MEASURED and pn.sharpen_note == (
        "measure, don't machine: re-grade 'N' capacity to MEASURED (±3%) and the ordering is restored with zero "
        "new metal")
    pw = next(p for p in out if p.rival_key == "w")
    assert pw.sharpen_rival_to is G.MODELLED
    pw = fb.prescribe(_path([fuse, wide], "f"), confidence=0.999)[0]     # z x 40 % > 1: no FoS is enough
    assert pw.raise_rival_fos_to is None and pw.raise_rival_note == (
        "infeasible: 'W' is GUESS grade (±40%) — its band grows as fast as any FoS you add; no amount of metal "
        "fixes an unknown. Sharpen the grade instead")
    assert fb.prescribe(_path([fuse, near], "nope")) == []


def test_the_fuse_floor_the_wide_fuse_and_the_closed_means():
    soft = fb.prescribe(_path([_el("f", 1.25, G.MODELLED, S1), _el("r", 1.2, G.MODELLED, S2)], "f"))[0]
    m = fb._solve_lower_fuse(1.2, 0.12, 0.1, N.inv_cdf(0.9))
    assert m < fb.MIN_FUSE_FOS == 1.10
    assert soft.lower_fuse_fos_to is None and soft.lower_fuse_note == (
        f"needs FoS {m:.2f} < the 1.10 floor — a fuse that soft pops in normal driving; use another lever")
    wide = fb.prescribe(_path([_el("f", 1.5, G.GUESS, S1), _el("r", 1.6, G.MODELLED, S2)], "f"), 0.999)[0]
    assert wide.lower_fuse_fos_to is None and wide.lower_fuse_note == (
        "infeasible: the fuse's own band is too wide for any FoS to dominate — sharpen the fuse's grade first")
    close = fb.prescribe(_path([_el("f", 1.5, G.MODELLED, S1), _el("r", 1.52, G.ESTIMATE, S2)], "f"))[0]
    assert close.sharpen_rival_to is None and close.sharpen_note == (
        "even a VERIFIED band can't restore the ordering alone — the means are genuinely too close; combine with "
        "lever (a) or (b)")
    # a rival already sharper than MODELLED is only offered a sharper grade still
    meas = fb.prescribe(_path([_el("f", 1.5, G.MODELLED, S1), _el("r", 1.70, G.MEASURED, S2)], "f"))[0]
    assert meas.sharpen_rival_to is G.VERIFIED
    exact = _el("r", 1.5, G.MODELLED, S2)
    pr = fb.prescribe(_path([_el("f", 1.2, G.MODELLED, S1), exact], "f"), confidence=0.95)
    assert pr[0].sharpen_rival_to is G.MEASURED
    p_now = fb.pairwise_first(1.2, 0.12, 1.5, 0.15)
    assert fb.prescribe(_path([_el("f", 1.2, G.MODELLED, S1), exact], "f"), confidence=p_now) == []


def test_the_charter_seal():
    ch = fb.create_charter({"b": "x", "a": "y"}, 0.8, 0.05, "n")
    assert list(ch["designations"]) == ["a", "b"] and (ch["confidence"], ch["forbidden_p"], ch["note"]) == (
        0.8, 0.05, "n")
    assert ch["created_utc"].endswith(" UTC") and len(ch["created_utc"]) == 20
    payload = json.dumps({k: ch[k] for k in ("confidence", "forbidden_p", "designations", "created_utc", "note")},
                         sort_keys=True, separators=(",", ":"))
    assert fb._charter_payload(ch) == payload and ch["seal"] == hashlib.sha256(payload.encode()).hexdigest()
    assert fb.charter_intact(ch)
    assert not fb.charter_intact({**ch, "note": "m"}) and not fb.charter_intact({**ch, "seal": ""})
    d = fb.create_charter({})
    assert (d["confidence"], d["forbidden_p"], d["note"]) == (0.9, 0.01, "")
    for bad in (0.5, 1.0):
        with pytest.raises(ValueError, match=r"^confidence must be in \(0\.5, 1\.0\)$"):
            fb.create_charter({}, confidence=bad)
    for bad in (0.0, 0.5):
        with pytest.raises(ValueError, match=r"^forbidden_p must be in \(0, 0\.5\)$"):
            fb.create_charter({}, forbidden_p=bad)
    assert fb.create_charter({}, 0.5000001, 0.4999999)["seal"]


DATUM = ("Free datum: reality just measured '{}' — its true capacity is bounded by this event. Re-grade that "
         "capacity to MEASURED with the event magnitude and the whole pecking order sharpens at zero cost.")


def test_incidents_word_for_word():
    path = _path([_el("f", 1.2, G.MODELLED, S1), _el("s", 1.6), _el("x", 2.0, G.MODELLED, S3)], "f")
    ch = fb.create_charter({"p": "f"})
    assert fb.judge_incident(ch, path, "f") == (IncidentVerdict.AS_DESIGNED, (
        "AS-DESIGNED: the designated fuse 'F' took the hit, exactly as sealed. Swap the spare, log the event "
        "load if known, race on. " + DATUM.format("F")))
    assert fb.judge_incident(ch, path, "s", "lap 3") == (IncidentVerdict.SURPRISE, (
        "SURPRISE: 'S' failed first, not the designated fuse. The sealed ordering was wrong about reality — audit "
        "whether the miss was capacity (this element weaker than declared) or load share (it saw more of the path "
        "load than its FoS assumed). " + DATUM.format("S") + " [lap 3]"))
    assert fb.judge_incident(ch, path, "s")[1].endswith("zero cost.")
    assert fb.judge_incident(ch, path, "x", "e") == (IncidentVerdict.BREACH, (
        "BREACH: forbidden-first element 'X' failed first. This is a safety event, not a repair bill — the load "
        "path must be redesigned so an S1/S2 element yields before it. " + DATUM.format("X") + " [e]"))
    assert fb.judge_incident(ch, path, "x")[1].endswith("zero cost.")
    assert fb.judge_incident(ch, path, "f", "e")[1].endswith(" [e]")
    assert fb.judge_incident({**fb.create_charter({}), "designations": {}}, path, "f")[0] is not IncidentVerdict.BREACH
    nodes = {k: v for k, v in fb.create_charter({}).items()}
    assert fb.judge_incident(nodes, path, "f")[0] is IncidentVerdict.SURPRISE
    with pytest.raises(ValueError, match=r"^Fuse Charter seal is broken — a sealed designation was edited after "
                                         r"sealing\. Re-seal a new charter; this one refuses to judge\.$"):
        fb.judge_incident({**ch, "note": "edited"}, path, "f")
    with pytest.raises(ValueError, match=r"^'q' is not an element of path 'p'$"):
        fb.judge_incident(ch, path, "q")


def test_the_markdown_report_line_by_line():
    f = _el("f", 1.2, G.MEASURED, S1, 40.0, 0.5)
    r = _el("r", 1.8, G.MEASURED, None, 1900.0, 9.0)
    p = _path([f, r], "f")
    a = fb.audit_path(p)
    skip = OverloadPath("q", "Skipped", "s", [f], "f")
    body = ["", "## Path P — **FUSED**", "", "> A story.", "", a.headline, "",
            "| element | severity | FoS | grade (±) | P(first) | cost | days |", "|---|---|---|---|---|---|---|",
            f"| F ⛓️ | S1 | 1.20 | measured ±3% | {a.probs['f']:.1%} | $40 | 0.5 |",
            f"| R | — | 1.80 | measured ±3% | {a.probs['r']:.1%} | $1,900 | 9.0 |", "",
            f"Expected overload bill (conditional on the hit): **${a.expected_cost_usd:,.0f} · "
            f"{a.expected_downtime_days:.1f} days** — the intended fuse alone would cost $40 · 0.5 days.", "",
            "- ⚠️ " + a.blind_spots[0], "",
            "*Probabilities are first-order statistics of the minimum of independent normal capacities, σ from the "
            "Proof Engine's evidence-grade band law. Deterministic: same deck in, same order out.*"]
    head = ["# ⛓️ Fusebox — the failure-order audit", ""]
    assert fb.render_fusebox_md([p, skip], [a]).split("\n") == head + [
        "*No sealed charter — audits use module defaults (confidence 90%, forbidden ≤ 1%).*"] + body
    ch = fb.create_charter({"p": "f"}, 0.85, 0.02)
    assert fb.render_fusebox_md([p], [a], ch, frame_tag="ISO 8855").split("\n") == head + [
        f"*Charter sealed {ch['created_utc']} · confidence 85% · forbidden ≤ 2% · sha256 `{ch['seal'][:16]}…`*",
        "*Coordinate convention: ISO 8855*"] + body
    assert fb.render_fusebox_md([p], [a], {**ch, "note": "x"}).split("\n")[2] == (
        "*⚠️ CHARTER SEAL BROKEN — verdicts below are advisory only; re-seal before judging any incident.*")
    clean = fb.audit_path(_path([f, _el("r", 1.8, G.MEASURED, S2, 1900.0, 9.0)], "f"))
    md = fb.render_fusebox_md([_path([f, _el("r", 1.8, G.MEASURED, S2, 1900.0, 9.0)], "f")], [clean]).split("\n")
    assert md[-2] == "" and md[-3].startswith("Expected overload bill") and "- ⚠️" not in "\n".join(md)


def test_a_coarse_grid_against_a_narrow_and_a_wide_capacity():
    els = [_el("n", 1.0, G.VERIFIED), _el("w", 1.6, G.GUESS), _el("m", 1.2, G.MODELLED)]
    for n in (101, 150, 4001):
        got, want = fb.first_failure_probs(els, n), _probs(els, n)
        for k in want:
            assert got[k] == pytest.approx(want[k], rel=1e-12, abs=1e-15)


def test_orderings_follow_probability_not_name():
    els = [_el("a", 1.5, G.MODELLED, S1), _el("z", 1.3, G.MODELLED, S2), _el("m", 2.4, G.GUESS, S2)]
    a = fb.audit_path(_path(els, "a"))
    assert a.contenders[0] == "z" and a.contenders == sorted(a.contenders, key=lambda k: -a.probs[k])
    x = [_el("f", 1.3, G.MODELLED, S1), _el("b", 1.9, G.GUESS, S3), _el("a", 1.6, G.ESTIMATE, S3)]
    b = fb.audit_path(_path(x, "f"))
    assert [k for k, _ in b.forbidden_hits] == sorted(["a", "b"], key=lambda k: -b.probs[k])
    assert b.probs["b"] != b.probs["a"]


def test_a_dominated_rival_listed_first_does_not_stop_the_others():
    fuse = _el("f", 1.5, G.MODELLED, S1)
    out = fb.prescribe(_path([_el("s", 3.5, G.MEASURED), fuse, _el("n", 1.75, G.ESTIMATE)], "f"))
    assert [p.rival_key for p in out] == ["n"]
    assert fb._solve_lower_fuse(0.0, 0.0, 0.1, 1.0) is None


def test_a_charter_without_designations_and_a_skipped_path_first():
    ch = fb.create_charter({"p": "f"})
    del ch["designations"]
    ch["seal"] = hashlib.sha256(fb._charter_payload(ch).encode()).hexdigest()
    path = _path([_el("f", 1.2, G.MODELLED, S1), _el("s", 1.6)], "f")
    assert fb.judge_incident(ch, path, "f")[0] is IncidentVerdict.SURPRISE
    f = _el("f", 1.2, G.MEASURED, S1, 40.0, 0.5)
    p = _path([f], "f")
    md = fb.render_fusebox_md([OverloadPath("q", "Skipped", "s", [f], "f"), p], [fb.audit_path(p)])
    assert "## Path P — **FUSED**" in md and "Skipped" not in md
