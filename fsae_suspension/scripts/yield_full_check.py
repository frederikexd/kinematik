# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Full-solve check of the final corners' build yield (paper section 3.3, supplement S13.6).

The manifests price yield to first order (inverse_genesis.build_yield: the weld scatter propagated through the curve
Jacobian, n_verify_full = 0). This re-solves every sampled build exactly: each perturbed corner is solved over the
travel stations and judged against the bands, a build that does not solve counting as a fail
(inverse_genesis._verify_yield_full). It reports, at the manifest's jig tolerance and at the hand-weld preset, the
first-order yield on the manifest's own samples, the full-solve yield on N further builds, their pass/fail agreement
and the one-sided 95% lower bound on the full-solve yield (Clopper-Pearson; 1 - 0.05^(1/N) when every build passes).
Nothing in the manifests changes. usage: yield_full_check.py [N] [out.json]"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import genesis_repro as gr                                 # noqa: E402
from suspension import inverse_genesis as ig                               # noqa: E402


def lower_bound_95(k: int, n: int) -> float:
    """One-sided 95% Clopper-Pearson lower bound on a pass fraction (dimensionless)."""
    if k == 0:
        return 0.0
    if k == n:
        return 0.05 ** (1.0 / n)
    from scipy.stats import beta
    return float(beta.ppf(0.05, k, n - k + 1))


def check(name: str, n_full: int = 1000) -> dict:
    """Yield of a manifest's winner at jig and hand-weld tolerance, first order and full solve (dimensionless)."""
    m = gr.GenesisManifest.from_json((ROOT / "manifests" / f"{name}.recorded.genesis.json").read_text())
    _, tg, _, fld_jig = m.objects()
    hp = gr.hp_from_dict(m.recorded["winner_hardpoints"])
    r_fit, ok = tg.residual(hp)
    assert ok
    out = {"name": name, "recorded_yield": m.recorded["winner_yield"], "n_full": n_full, "fields": {}}
    for label, fld in (("jig", fld_jig), ("hand_weld", gr.field_with_overrides("hand_weld"))):
        lin, _ = ig.build_yield(hp, tg, fld, r_fit, n=m.search.n_yield, seed=m.search.seed, step_mm=m.search.step_mm)
        J = ig._jacobian(hp, tg, fld.coords(), step_mm=m.search.step_mm)
        full, agree = ig._verify_yield_full(hp, tg, fld, r_fit, J, n_full, m.search.seed)
        k = int(round(full * n_full))
        out["fields"][label] = {"provenance": fld.provenance, "linear_yield": lin, "full_yield": full,
                                "agreement": agree, "full_passes": k, "full_lower_95": lower_bound_95(k, n_full)}
    return out


def scatter(name: str, n: int = 300, seed: int = 0) -> dict:
    """Hand-weld scatter of the static roll-centre height (mm) and the bump-steer slope (deg per 10 mm of travel)
    over ``n`` builds of a manifest's winner: 95th percentile and worst absolute change from nominal."""
    import numpy as np
    from suspension.kinematik_stochastic import _perturbed
    m = gr.GenesisManifest.from_json((ROOT / "manifests" / f"{name}.recorded.genesis.json").read_text())
    _, tg, _, _ = m.objects()
    hp = gr.hp_from_dict(m.recorded["winner_hardpoints"])
    st = np.array([-12.5, 0.0, 12.5])
    tr = getattr(tg, "track_mm", None) or 1200.0

    def _ch(h):
        c, ok = ig.curves_of(h, st, track_mm=tr)
        return (float(c["rc_height_mm"][1]), float(c["toe_deg"][2] - c["toe_deg"][0]) / 25.0 * 10.0) if ok else None
    rc0, bs0 = _ch(hp)
    fld = gr.field_with_overrides("hand_weld")
    S, coords = fld.sample(n, seed=seed), fld.coords()
    d_rc, d_bs, fails = [], [], 0
    for i in range(n):
        r = _ch(_perturbed(hp, ig._unflatten(S[i], coords)))
        if r is None:
            fails += 1
            continue
        d_rc.append(abs(r[0] - rc0)); d_bs.append(abs(r[1] - bs0))
    return {"n": n, "fails": fails, "rc_p95_mm": float(np.percentile(d_rc, 95)), "rc_max_mm": float(max(d_rc)),
            "bump_steer_p95_deg_per_10mm": float(np.percentile(d_bs, 95)),
            "bump_steer_max_deg_per_10mm": float(max(d_bs))}


def run(n_full: int = 1000, out_path: str | None = None) -> dict:
    """Both final corners; yields dimensionless, scatter in mm and deg."""
    res = {n: check(n, n_full) for n in ("front_v11_final", "rear_v11_final")}
    for n in res:
        res[n]["scatter_hand_weld"] = scatter(n)
    if out_path:
        Path(out_path).write_text(json.dumps(res, indent=1))
    return res


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    r = run(n, sys.argv[2] if len(sys.argv) > 2 else None)
    for name, o in r.items():
        for lab, f in o["fields"].items():
            print(name, lab, round(f["linear_yield"], 4), round(f["full_yield"], 4), round(f["agreement"], 4),
                  round(f["full_lower_95"], 4))
