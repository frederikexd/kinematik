#!/usr/bin/env python3
# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Prediction ledger protocol v2: test it across years, then freeze it for the next competition.

  crossyear  fit on FSG 2025 -> predict FSG 2026 (forward, as the blind test will run), and fit on 2026 ->
             predict 2025 (reverse check). Writes benchmarks/ledger/protocol-v2/CROSS_YEAR.md and records.
  freeze     fit on every listed year, take the team record from the latest, and write CALIBRATION.json:
             the frozen v2 for the next competition. Push it before that competition's data is used.
             --challenger writes CALIBRATION_challenger.json instead: the pre-registered challenger
             (acceleration at the fastest quarter of the field), sealed alongside v2 and decided blind.
  registered run the tests fixed in protocol-v2/REGISTRATION.md (FSG 2023/2024 and autocross pace)
  verify-all  check every sealed record and frozen calibration in benchmarks/ledger (exit 1 on any failure)
  other-events  run the test fixed in other-events/REGISTRATION.md (FS Czech and FS Italy, same cars)
  seal       predictions for a specs CSV with a frozen CALIBRATION.json -> SEALED.json
  score      as scripts/ledger.py score, plus the best-lap skidpad channel

Run from the repo root.
"""
import argparse
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from suspension import ledger as L  # noqa: E402
from suspension import ledger_v2 as V  # noqa: E402

ROOT = Path("benchmarks/ledger")
YEARS = {"2025": ROOT / "fsg2025", "2026": ROOT / "fsg2026"}


def YEAR_DIR(year):  # noqa: N802
    return ROOT / f"fsg{year}"


def _load(year):
    d = YEARS[year]
    specs = L.load_specs_csv(d / "specs.csv")
    return specs, [V.physics(s) for s in specs], L.load_results_csv(d / "results.csv")


def _clean(o):
    if isinstance(o, float) and math.isnan(o):
        return None
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, list):
        return [_clean(v) for v in o]
    return o


def _row(label, sc, bl=None):
    out = []
    for ev in ("skidpad", "accel"):
        s = sc["summary"].get(ev)
        if not s:
            continue
        rho = "n/a" if math.isnan(s["spearman"]) else format(s["spearman"], ".2f")
        out.append(f"| {label} | {'Skidpad' if ev == 'skidpad' else 'Acceleration'} | {s['n']} | "
                   f"{s['mae_pct']:.1f} % | {s['median_abs_pct']:.1f} % | {s['bias_pct']:+.1f} % | "
                   f"{s['within_5pct']} of {s['n']} | {rho} |")
    if bl:
        out.append(f"| {label} | Skidpad, faster lap | {bl['n']} | {bl['mae_pct']:.1f} % | | "
                   f"{bl['bias_pct']:+.1f} % | {bl['within_5pct']} of {bl['n']} | |")
    return out


def crossyear(out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    data = {y: _load(y) for y in YEARS}
    hdr = ["| Protocol | Event | Cars | Mean abs. error | Median | Bias | Within 5 % | Rank corr. |",
           "|---|---|---|---|---|---|---|---|"]
    lines = ["# Protocol v2 across years", "",
             "Each test year is predicted with factors fitted only on the other year. The forward test (fit on "
             "2025, predict 2026) is how the blind FSG 2027 test will run. Both years' results were public when "
             "this was made, so these are hindcasts.", ""]
    summary = {}
    for train, test, label in (("2025", "2026", "Forward: fit on FSG 2025, predict FSG 2026"),
                               ("2026", "2025", "Reverse: fit on FSG 2026, predict FSG 2025")):
        specs_tr, phys_tr, res_tr = data[train]
        specs_te, phys_te, res_te = data[test]
        calib = V.fit(phys_tr, res_tr, f"FSG {train}")
        record = V.team_record(phys_tr, res_tr, calib)
        calib_c = V.fit(phys_tr, res_tr, f"FSG {train}", accel_quantile=V.CHALLENGER_ACCEL_QUANTILE)
        record_c = V.team_record(phys_tr, res_tr, calib_c)
        v1 = L.score({"event": f"FSG {test}", "kind": "hindcast", "record_sha256": "",
                      "predictions": [L.predict(s) for s in specs_te]}, res_te)
        recs = {}
        for name, cal_used, rec_used, evs in (
                ("v2 shared factors (1, 2, 3, 5)", calib, None, ()),
                ("tried: team record on both events", calib, record, ("skidpad", "accel")),
                ("v2 final (1-5, record on skidpad)", calib, record, V.TEAM_RECORD_EVENTS),
                ("challenger: accel at fastest quarter", calib_c, record_c, V.TEAM_RECORD_EVENTS)):
            r = V.seal(specs_te, f"FSG {test}", "hindcast", cal_used, rec_used, record_events=evs,
                       note=f"{label}. Results for both years were public when this was made.")
            recs[name] = r
        lines += [f"## {label}", "",
                  f"Factors: skidpad R20 {calib['skidpad']['R20']:.3f}, LC0 {calib['skidpad']['LC0']:.3f}, "
                  f"other {calib['skidpad']['other']:.3f} (pooled {calib['skidpad_pooled']:.3f}); acceleration "
                  f"{calib['accel']:.3f} (challenger, fastest quarter: {calib_c['accel']:.3f}). Training runs: "
                  f"{calib['n']}. Teams with a record: {len(record)}.", ""]
        lines += hdr + _row("v1 (defaults only)", v1)
        summary[f"{train}->{test}"] = {"v1": v1["summary"]}
        for name, r in recs.items():
            sc = L.score(r, res_te)
            bl = V.best_lap_summary(r, res_te)
            lines += _row(name, sc, bl)
            summary[f"{train}->{test}"][name] = {**sc["summary"], "best_lap": bl}
            if name.startswith("v2 final"):
                (out_dir / f"fsg{test}_from{train}.SEALED.json").write_text(
                    json.dumps(r, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
        lines += [""]
        rej = [(p["car"], p["team"], p["rejected"]) for p in recs["v2 final (1-5, record on skidpad)"]["predictions"]
               if p["rejected"]]
        if rej:
            lines += ["Published values set aside by rule 2: " + "; ".join(
                f"{t} (car {c}): {', '.join(f'{k} {v}' for k, v in d.items())}" for c, t, d in rej), ""]
    (out_dir / "CROSS_YEAR.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (out_dir / "cross_year.json").write_text(json.dumps(_clean(summary), indent=1) + "\n", encoding="utf-8")
    print("\n".join(lines))


REGISTERED_YEARS = ("2023", "2024")
PACE_YEARS = ("2023", "2024", "2025", "2026")


def _pct(x):
    return "n/a" if x is None else format(x, ".1f") + " %"


def registered(out_dir: Path):
    """Run the tests fixed in protocol-v2/REGISTRATION.md and write REGISTERED_RESULTS.md / .json."""
    from suspension import ledger_pace as P
    d = Path(out_dir)
    reg = (d / "REGISTRATION.md").read_bytes()
    cals = {"v2 (median)": json.loads((d / "CALIBRATION.json").read_text(encoding="utf-8")),
            "challenger (fastest quarter)": json.loads((d / "CALIBRATION_challenger.json").read_text(encoding="utf-8"))}
    out = {"registration_sha256": hashlib.sha256(reg).hexdigest(), "test_a": {}, "test_b": {}, "factors": {}}
    lines = ["# Results of the registered tests", "",
             f"Protocol: [`REGISTRATION.md`](REGISTRATION.md), sha256 `{out['registration_sha256']}`. Every number "
             "below comes from `python fsae_suspension/scripts/ledger_v2.py registered`.", "",
             "## Test A: frozen v2 and the challenger on FSG 2023 and 2024", "",
             "| Calibration | Year | Event | Cars | Mean abs. error | Median | Bias | Within 5 % | Rank corr. |",
             "|---|---|---|---|---|---|---|---|---|"]
    pooled = {}
    for name, cal in cals.items():
        for y in REGISTERED_YEARS:
            specs = L.load_specs_csv(YEAR_DIR(y) / "specs.csv")
            res = L.load_results_csv(YEAR_DIR(y) / "results.csv")
            rec = {"event": f"FSG {y}", "kind": "hindcast", "record_sha256": "",
                   "predictions": [V.predict(V.physics(s), cal["calibration"], None) for s in specs]}
            sc = L.score(rec, res)
            out["test_a"].setdefault(name, {})[y] = _clean(sc["summary"])
            for ev in ("skidpad", "accel"):
                m = sc["summary"][ev]
                rho = "n/a" if math.isnan(m["spearman"]) else format(m["spearman"], ".2f")
                lines.append(f"| {name} | {y} | {'Skidpad' if ev == 'skidpad' else 'Acceleration'} | {m['n']} | "
                             f"{m['mae_pct']:.1f} % | {m['median_abs_pct']:.1f} % | {m['bias_pct']:+.1f} % | "
                             f"{m['within_5pct']} of {m['n']} | {rho} |")
                pooled.setdefault(name, {}).setdefault(ev, []).extend(
                    r["error_pct"] for r in sc["rows"] if r["event"] == ev and not r["excluded"])
    lines += ["", "Pooled over both years:", "", "| Calibration | Event | Cars | Mean abs. error | Within 5 % |",
              "|---|---|---|---|---|"]
    for name, evs in pooled.items():
        for ev, e in evs.items():
            mae = sum(abs(x) for x in e) / len(e)
            n5 = sum(1 for x in e if abs(x) <= 5.0)
            out["test_a"][name][f"pooled_{ev}"] = {"n": len(e), "mae_pct": mae, "within_5pct": n5}
            lines.append(f"| {name} | {'Skidpad' if ev == 'skidpad' else 'Acceleration'} | {len(e)} | "
                         f"{mae:.1f} % | {n5} of {len(e)} |")
    a, c = out["test_a"]["v2 (median)"]["pooled_accel"], out["test_a"]["challenger (fastest quarter)"]["pooled_accel"]
    wins = c["mae_pct"] < a["mae_pct"] and c["within_5pct"] >= a["within_5pct"]
    out["challenger_adopted"] = wins
    lines += ["", f"**Decision rule (registered): {'the challenger wins' if wins else 'v2 stands'}.** Pooled "
              f"acceleration: challenger {c['mae_pct']:.1f} %, {c['within_5pct']} within 5 %; v2 {a['mae_pct']:.1f} "
              f"%, {a['within_5pct']} within 5 %.", "",
              "## Test B: autocross relative pace, FSG 2023 to 2026", "",
              "| Year | Cars | Model mean abs. error | Model within 5 % | Rank corr. | Spec-blind baseline | "
              "Baseline within 5 % |", "|---|---|---|---|---|---|---|"]
    for y in PACE_YEARS:
        specs = L.load_specs_csv(YEAR_DIR(y) / "specs.csv")
        sc = P.score_pace({s.car: P.autocross_lap_s(s) for s in specs},
                          L.load_results_csv(YEAR_DIR(y) / "autocross.csv"))
        out["test_b"][y] = _clean(sc)
        m, b = sc["model"], sc["baseline"]
        rho = "n/a" if m.get("spearman") is None else format(m["spearman"], ".2f")
        lines.append(f"| {y} | {m['n']} | {_pct(m['mae_pct'])} | {m['within_5pct']} of {m['n']} | {rho} | "
                     f"{_pct(b['mae_pct'])} | {b['within_5pct']} of {b['n']} |")
    lines += ["", "## The calibration factors, fitted on each year alone", "",
              "| Year | Skidpad factor (pooled) | Skidpad cars | Implied lateral g, model | Implied lateral g, "
              "official (median) | Acceleration factor | Acceleration cars |", "|---|---|---|---|---|---|---|"]
    for y in PACE_YEARS:
        specs = L.load_specs_csv(YEAR_DIR(y) / "specs.csv")
        res = L.load_results_csv(YEAR_DIR(y) / "results.csv")
        phys = [V.physics(s) for s in specs]
        cal = V.fit(phys, res, f"FSG {y}")
        gm, go = [], []
        for p in phys:
            rad = p["inputs"]["skidpad_radius"]["value"]
            gm.append((2 * math.pi * rad / p["skidpad_s"]) ** 2 / rad / 9.81)
            r = res.get(p["car"], {}).get("skidpad")
            if r and not L._excluded("skidpad", r):
                go.append((2 * math.pi * rad / r["best_s"]) ** 2 / rad / 9.81)
        g_m, g_o = statistics.median(gm), statistics.median(go)
        out["factors"][y] = {"skidpad_pooled": cal["skidpad_pooled"], "accel": cal["accel"], "n": cal["n"],
                             "lat_g_model": g_m, "lat_g_official": g_o}
        lines.append(f"| {y} | {cal['skidpad_pooled']:.3f} | {cal['n']['skidpad']} | {g_m:.2f} g | {g_o:.2f} g | "
                     f"{cal['accel']:.3f} | {cal['n']['accel']} |")
    (d / "REGISTERED_RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (d / "registered_results.json").write_text(json.dumps(_clean(out), indent=1) + "\n", encoding="utf-8")
    print("\n".join(lines))


OTHER_EVENTS = (("fscz2026", "FS Czech 2026", "2026", "2025"), ("fsita2026", "FS Italy 2026", "2026", "2025"),
                ("fscz2025", "FS Czech 2025", "2025", "2026"), ("fsita2025", "FS Italy 2025", "2025", "2026"))


def other_events(out_dir: Path):
    """Run the test fixed in other-events/REGISTRATION.md and write RESULTS.md / results.json there."""
    d = Path(out_dir)
    reg = (d / "REGISTRATION.md").read_bytes()
    frozen = json.loads((ROOT / "protocol-v2" / "CALIBRATION.json").read_text(encoding="utf-8"))["calibration"]
    fits = {}
    for y in ("2025", "2026"):
        _specs, phys_y, res = _load(y)
        fits[y] = V.fit(phys_y, res, f"FSG {y}")
    out = {"registration_sha256": hashlib.sha256(reg).hexdigest(), "events": {}, "pooled": {}}
    lines = ["# Results: other organizers, same cars", "",
             f"Protocol: [`REGISTRATION.md`](REGISTRATION.md), sha256 `{out['registration_sha256']}`. Every number "
             "below comes from `python fsae_suspension/scripts/ledger_v2.py other-events`.", "",
             "Headline: v2 with factors fitted on the other FSG year (never on the season being predicted), no team "
             "record.", "",
             "| Event | Model | Event type | Cars | Mean abs. error | Median | Bias | Within 5 % | Rank corr. |",
             "|---|---|---|---|---|---|---|---|---|"]
    pooled: dict = {}
    for folder, name, season, train in OTHER_EVENTS:
        specs = L.load_specs_csv(YEAR_DIR(season) / "specs.csv")
        res = L.load_results_csv(d / folder / "results.csv")
        phys = [V.physics(s) for s in specs]
        models = {f"v2, factors from FSG {train}": [V.predict(p, fits[train], None) for p in phys],
                  "v1, no calibration": [L.predict(s) | {"car": s.car} for s in specs],
                  "v2, frozen (FSG 2025 + 2026)": [V.predict(p, frozen, None) for p in phys]}
        for model, preds in models.items():
            rec = {"event": name, "kind": "hindcast", "record_sha256": "", "predictions": preds}
            sc = L.score(rec, res)
            out["events"].setdefault(name, {})[model] = _clean(sc["summary"])
            key = "v2 (other FSG year)" if model.startswith("v2, factors") else model
            for ev in ("skidpad", "accel"):
                m = sc["summary"][ev]
                if not m["n"]:
                    continue
                rho = "n/a" if math.isnan(m["spearman"]) else format(m["spearman"], ".2f")
                lines.append(f"| {name} | {model} | {'Skidpad' if ev == 'skidpad' else 'Acceleration'} | "
                             f"{m['n']} | {m['mae_pct']:.1f} % | {m['median_abs_pct']:.1f} % | "
                             f"{m['bias_pct']:+.1f} % | {m['within_5pct']} of {m['n']} | {rho} |")
                pooled.setdefault(key, {}).setdefault(ev, []).extend(
                    r["error_pct"] for r in sc["rows"] if r["event"] == ev and not r["excluded"])
    lines += ["", "## Pooled over all four events", "",
              "| Model | Event type | Cars | Mean abs. error | Bias | Within 5 % |", "|---|---|---|---|---|---|"]
    for model, evs in pooled.items():
        for ev, e in evs.items():
            mae = sum(abs(x) for x in e) / len(e)
            bias = sum(e) / len(e)
            n5 = sum(1 for x in e if abs(x) <= 5.0)
            out["pooled"].setdefault(model, {})[ev] = {"n": len(e), "mae_pct": mae, "bias_pct": bias,
                                                      "within_5pct": n5}
            lines.append(f"| {model} | {'Skidpad' if ev == 'skidpad' else 'Acceleration'} | {len(e)} | "
                         f"{mae:.1f} % | {bias:+.1f} % | {n5} of {len(e)} |")
    (d / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (d / "results.json").write_text(json.dumps(_clean(out), indent=1) + "\n", encoding="utf-8")
    print("\n".join(lines))


def verify_all(root: Path = ROOT) -> int:
    """Check every sealed record and frozen calibration under benchmarks/ledger: unchanged since sealing, and
    reproduced by today's code when the code is unchanged. Prints one line each; returns 1 if any fails."""
    bad = 0
    for path in sorted(root.rglob("*.json")):
        try:
            rec = json.loads(path.read_text(encoding="utf-8"))
        except ValueError:
            continue
        rel = path.relative_to(root.parent.parent)
        if path.name.startswith("CALIBRATION"):
            body = {k: v for k, v in rec.items() if k != "sha256"}
            ok = hashlib.sha256(L._canon(body)).hexdigest() == rec.get("sha256")
            print(f"{'OK  ' if ok else 'FAIL'} {rel}: frozen calibration {'unchanged' if ok else 'CHANGED'}")
            bad += not ok
            continue
        if "record_sha256" not in rec or "predictions" not in rec:
            continue
        v = V.verify(rec) if str(rec.get("protocol_version")) == "2" else L.verify(rec)
        ok = v["intact"] and (v["reproduced"] or not v["same_code"])
        note = ("reproduced by today's code" if v["reproduced"] else
                "code changed since sealing, so not re-run" if not v["same_code"] else "NOT reproduced")
        print(f"{'OK  ' if ok else 'FAIL'} {rel}: {'intact' if v['intact'] else 'ALTERED'}, {note}")
        bad += not ok
    return 1 if bad else 0


def freeze(years, out: Path, accel_quantile: float = V.ACCEL_QUANTILE):
    phys, res, latest = [], {}, max(years)
    for y in years:
        _s, p, r = _load(y)
        phys += [{**x, "car": f"{y}:{x['car']}"} for x in p]
        res.update({f"{y}:{k}": v for k, v in r.items()})
    calib = V.fit(phys, res, " + ".join(f"FSG {y}" for y in years), accel_quantile=accel_quantile)
    _s, p_last, r_last = _load(latest)
    record = V.team_record(p_last, r_last, calib)
    body = {"protocol_version": V.PROTOCOL_VERSION, "protocol": V.PROTOCOL_TEXT, "calibration": calib,
            "team_record": record, "team_record_from": f"FSG {latest}", "code_sha256": V.code_fingerprint()}
    body["sha256"] = hashlib.sha256(L._canon(body)).hexdigest()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(body, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"frozen v2 -> {out}\nsha256 {body['sha256']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("crossyear")
    c.add_argument("--out", default=str(ROOT / "protocol-v2"))
    f = sub.add_parser("freeze")
    f.add_argument("--years", nargs="+", default=list(YEARS))
    f.add_argument("--out", default=str(ROOT / "protocol-v2" / "CALIBRATION.json"))
    f.add_argument("--challenger", action="store_true",
                   help="freeze the pre-registered challenger (acceleration at the fastest quarter) instead")
    sub.add_parser("verify-all")
    o = sub.add_parser("other-events")
    o.add_argument("--out", default=str(ROOT / "other-events"))
    g = sub.add_parser("registered")
    g.add_argument("--out", default=str(ROOT / "protocol-v2"))
    s = sub.add_parser("seal")
    s.add_argument("--calibration", required=True)
    s.add_argument("--specs", required=True)
    s.add_argument("--event", required=True)
    s.add_argument("--kind", required=True, choices=["blind", "hindcast"])
    s.add_argument("--note", default="")
    s.add_argument("--out", required=True)
    k = sub.add_parser("score")
    k.add_argument("--record", required=True)
    k.add_argument("--results", required=True)
    k.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    if a.cmd == "crossyear":
        crossyear(Path(a.out))
    elif a.cmd == "verify-all":
        return verify_all()
    elif a.cmd == "other-events":
        other_events(Path(a.out))
    elif a.cmd == "registered":
        registered(Path(a.out))
    elif a.cmd == "freeze":
        if a.challenger and a.out == str(ROOT / "protocol-v2" / "CALIBRATION.json"):
            a.out = str(ROOT / "protocol-v2" / "CALIBRATION_challenger.json")
        freeze(a.years, Path(a.out), V.CHALLENGER_ACCEL_QUANTILE if a.challenger else V.ACCEL_QUANTILE)
    elif a.cmd == "seal":
        cal = json.loads(Path(a.calibration).read_text(encoding="utf-8"))
        body = {k: v for k, v in cal.items() if k != "sha256"}
        if hashlib.sha256(L._canon(body)).hexdigest() != cal.get("sha256"):
            print("REFUSED: CALIBRATION.json was altered after it was frozen", file=sys.stderr)
            return 1
        rec = V.seal(L.load_specs_csv(a.specs), a.event, a.kind, cal["calibration"], cal["team_record"],
                     note=a.note + f" Frozen calibration sha256 {cal['sha256']}.")
        Path(a.out).write_text(json.dumps(rec, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"sealed {len(rec['predictions'])} cars -> {a.out}\nrecord sha256 {rec['record_sha256']}")
    else:
        rec = json.loads(Path(a.record).read_text(encoding="utf-8"))
        if not V.verify(rec)["intact"]:
            print("REFUSED: the sealed record has been altered since it was sealed", file=sys.stderr)
            return 1
        res = L.load_results_csv(a.results)
        sc = L.score(rec, res)
        md = L.scorecard_markdown(sc)
        bl = V.best_lap_summary(rec, res)
        if bl:
            md += (f"\nSkidpad against the faster of the left and right laps: {bl['n']} cars, mean abs. error "
                   f"{bl['mae_pct']:.1f} %, bias {bl['bias_pct']:+.1f} %, {bl['within_5pct']} within 5 %.\n")
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "SCORECARD.md").write_text(md, encoding="utf-8")
        (out / "scorecard.json").write_text(json.dumps(_clean({**sc, "best_lap": bl}), indent=1) + "\n",
                                            encoding="utf-8")
        print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
