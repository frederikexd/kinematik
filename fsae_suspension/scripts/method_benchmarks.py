# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Method benchmarks of the paper (supplement S13).

recovery():  known-answer test. A hidden geometry moves k of the final front's six inboard pickups by up to +/-10 mm
             per axis; its camber, toe and roll-centre curves at five stations become targets with tight bands
             (0.05 deg, 0.02 deg, 2 mm); genesis_solve starts from the unmoved front with +/-15 mm boxes. Optionally a
             generic bounded least-squares solver (scipy trf) on the same residual is the comparison.
build_curves_only(): the final front's targets, boxes, tolerance and search settings with every wall removed,
             started from the delivered front (manifest front_v11_curves_only).
judge_curves_only(): every in-band candidate of that run judged against the final front's walls.
Lengths mm, angles deg, times s."""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import inverse_genesis as ig                                          # noqa: E402
from suspension.kinematik_stochastic import _perturbed                                 # noqa: E402
from suspension.genesis_repro import GenesisManifest, SearchSettings, hp_from_dict     # noqa: E402

MAN = ROOT / "manifests"
PTS = ["upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner", "tie_rod_inner", "tie_rod_outer"]
ST5 = np.array([-25.0, -12.5, 0.0, 12.5, 25.0])
ST11 = np.linspace(-25.0, 25.0, 11)
BANDS = {"camber_deg": 0.05, "toe_deg": 0.02, "rc_height_mm": 2.0}


def _manifest(name: str) -> GenesisManifest:
    return GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())


def recovery(ks=(2, 4, 6), draws: int = 10, seed: int = 0, with_lsq: bool = True, log=print) -> list[dict]:
    """Known-answer recovery draws on the final front; one dict per draw."""
    base = hp_from_dict(_manifest("front_v11_hairpin_walls").recorded["winner_hardpoints"])
    vol = ig.LegalVolume.around(base, 15.0, points=PTS)
    rng = np.random.default_rng(seed)
    rows = []
    for k in ks:
        for trial in range(draws):
            pts = list(rng.choice(PTS, size=k, replace=False))
            shift = {p: rng.uniform(-10, 10, 3) for p in pts}
            truth = _perturbed(base, shift)
            tv, ok = ig.curves_of(truth, ST5)
            if not ok:
                rows.append({"k": k, "trial": trial, "skipped": "truth does not solve"}); continue
            tg = ig.GenesisTargets(curves=[ig.TargetCurve(ch, ST5, tv[ch], np.full(5, b)) for ch, b in BANDS.items()])
            r0, _ = tg.residual(base)
            t0 = time.time(); c = ig.genesis_solve(base, tg, vol, max_iter=60); tig = time.time() - t0
            hp_c = ig._shifted(base, vol, c.shift_vec)

            def errs(h):
                v, ok = ig.curves_of(h, ST11); tt, _ = ig.curves_of(truth, ST11)
                return {ch: float(np.max(np.abs(v[ch] - tt[ch]))) for ch in BANDS} if ok else None

            def pick(h):
                return float(np.sqrt(np.mean([np.sum((np.asarray(getattr(h, p), float) - np.asarray(getattr(truth, p), float)) ** 2) for p in PTS])))

            row = {"k": k, "trial": trial, "moved": pts, "start_band": float(np.max(np.abs(r0))),
                   "start_pickup_rms_mm": pick(base),
                   "ig": {"hit": bool(c.hit), "band": float(c.max_band_frac), "iters": int(c.iterations), "secs": tig,
                          "curve_err11": errs(hp_c), "pickup_rms_mm": pick(hp_c)}}
            if with_lsq:
                from scipy.optimize import least_squares
                lo, hi = vol.bounds_vec(base)

                def f(xv):
                    r, ok = tg.residual(ig._shifted(base, vol, xv)); return r if ok else np.full(len(r0), 1e3)
                t0 = time.time(); ls = least_squares(f, np.zeros(len(lo)), bounds=(lo, hi), method="trf", max_nfev=2000); tls = time.time() - t0
                hp_l = ig._shifted(base, vol, ls.x); rl, okl = tg.residual(hp_l)
                row["lsq"] = {"hit": bool(okl and np.max(np.abs(rl)) <= 1.0), "band": float(np.max(np.abs(rl))) if okl else None,
                              "nfev": int(ls.nfev), "secs": tls, "curve_err11": errs(hp_l), "pickup_rms_mm": pick(hp_l)}
            rows.append(row)
            log(k, trial, round(row["start_band"], 2), row["ig"]["hit"], round(row["ig"]["band"], 3))
    return rows


def build_curves_only() -> GenesisManifest:
    """The curves-only baseline manifest (not yet run)."""
    H = json.loads((MAN / "front_v11_hairpin_walls.recorded.genesis.json").read_text())
    D = json.loads((MAN / "front_steering_walls.recorded.genesis.json").read_text())
    vol = {k: v for k, v in H["volume"].items() if k not in ("properties", "wheel_envelope")}
    ctx = {"paper": "v11", "axle": "front",
           "run": ("baseline, curves only: the curve targets, +/-100 mm pickup boxes, jig-weld tolerance field and search settings of "
                   "front_v11_hairpin_walls (inputs 7f1436d93a63) with every wall removed (no solved-property bounds, no wheel envelope), "
                   "started from the delivered ChassisV10 front (front_steering_walls winner, inputs bbc4d8297765); the winner is then "
                   "judged against the hairpin front's walls (scripts/baseline_curves_only.py)")}
    return GenesisManifest(name="front_v11_curves_only", hardpoints=D["recorded"]["winner_hardpoints"], targets=H["targets"], volume=vol,
                           tolerance=H["tolerance"], search=SearchSettings(**H["search"]), context=ctx, kinematik_version=H["kinematik_version"])


def judge_curves_only(manifest: GenesisManifest | None = None) -> dict:
    """Every candidate of the recorded curves-only run, the in-band ones judged against the final front's walls
    (front_v11_final: the hairpin front's walls on the corrected load cases and statics, frame-derived compliance)."""
    B = manifest or _manifest("front_v11_curves_only")
    _, _, volH, _ = _manifest("front_v11_final").objects()
    hpB, _, volB, _ = B.objects()
    win = hp_from_dict(B.recorded["winner_hardpoints"])
    out = {"inputs_sha256": B.inputs_sha256, "winner_verdict": B.recorded["winner_verdict"], "winner_yield": B.recorded["winner_yield"], "candidates": []}
    for i, c in enumerate(B.recorded["candidates"]):
        sv = np.asarray(c["shift_vec"], float)
        h = ig._shifted(hpB, volB, sv)
        row = {"i": i, "hit": c["hit"], "band": c["max_band_frac"], "yield": c.get("yield_frac"), "verdict": c.get("verdict"),
               "winner": all(np.allclose(np.asarray(getattr(h, p), float), np.asarray(getattr(win, p), float)) for p in volB.boxes),
               "shift_max_mm": float(np.max(np.abs(sv)))}
        if c["hit"]:
            props = {}
            for b in volH.properties.bounds:
                try:
                    x = ig.properties_of(h, volH.properties, only=[b.prop])[b.prop]
                except Exception:
                    x = None
                props[b.prop] = float(x) if x is not None and np.isfinite(x) else None
            row.update({"properties": props, "violated": [p for p, _, _ in volH.property_violations(h)],
                        "envelope_margin_mm": float(volH.envelope_margin(h)), "wall_violations": [str(v) for v in volH.wall_violations(h)]})
        out["candidates"].append(row)
    return out


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "judge"
    if what == "recovery":
        json.dump(recovery(), open(sys.argv[2] if len(sys.argv) > 2 else "bench_recovery.json", "w"), indent=1)
    elif what == "baseline":
        m = build_curves_only(); r = m.run(); m.record(r)
        Path(sys.argv[2] if len(sys.argv) > 2 else "front_v11_curves_only.recorded.genesis.json").write_text(m.to_json(indent=1))
    else:
        print(json.dumps(judge_curves_only(), indent=1))
