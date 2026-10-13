# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The final front re-searched with its joints and tabs declared (paper section 3.3, supplement S13.6).

front_v11_final meets its compliance walls with rigid link ends. This declares every input of it unchanged except
one: each link's two ends (joint, tab, upright arm) in series along the link at K_END per end, in both compliance
load cases (MemberStiffness.k_tab = K_END / 2). Its winner then breaks the braking-toe wall, so the start search is
declared (GenesisManifest.start_search): the pattern search moves the pickups inside the same boxes until every wall
holds, and InverseGenesis searches from there. usage: front_with_joints.py [K_END N/mm] [out_dir] [max_evals]"""
from __future__ import annotations

import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import genesis_seed as gs                                     # noqa: E402
from suspension.genesis_repro import GenesisManifest, hp_from_dict            # noqa: E402

MAN = ROOT / "manifests"


def declare(k_end: float = 50000.0, max_evals: int = 3000, ack_hi: float | None = 100.0) -> GenesisManifest:
    """front_v11_final with link ends of ``k_end`` N/mm each, started from its own winner; ``ack_hi`` (%) adds a
    ceiling to the Ackermann-at-lock wall (None keeps the floor alone)."""
    F = GenesisManifest.from_json((MAN / "front_v11_final.recorded.genesis.json").read_text())
    d = json.loads(F.to_json())
    for case in ("elasto", "elasto_braking"):
        for m in d["volume"]["properties"][case]["members"].values():
            m["k_tab"] = k_end / 2.0
    if ack_hi is not None:
        for b in d["volume"]["properties"]["bounds"]:
            if b["prop"] == "ackermann_lock_pct":
                b["hi"] = float(ack_hi)
                b["label"] = f"{b['lo']:g} <= Ackermann at the declared lock (%) <= {ack_hi:g}"
    vol = json.loads(json.dumps(d["volume"]))
    base = GenesisManifest.from_json(json.dumps({k: v for k, v in d.items() if k not in ("inputs_sha256", "recorded")}))
    m = base.restarted(f"front_v11_joints{k_end / 1000:.0f}", hp_from_dict(F.recorded["winner_hardpoints"]),
                       gs.SeedSearch(seed=1, max_evals=max_evals, step0_mm=16.0, step_min_mm=0.125,
                                     clearance_target_mm=0.45, stop_band_frac=0.6),
                       {"paper": "v11", "axle": "front", "run": (
                           "every input of front_v11_final (inputs " + F.inputs_sha256[:12] + ") with each link's two "
                           f"ends (joint, tab, upright arm) declared at {k_end / 1000:.0f} kN/mm along the link in both "
                           "compliance load cases" + ("" if ack_hi is None else f", Ackermann at lock at most {ack_hi:g}%") + ", started from its winner; start search declared, then InverseGenesis "
                           "(scripts/front_with_joints.py)")})
    assert m.volume == vol
    m.context.pop("declared_from", None)
    return m


if __name__ == "__main__":
    a = sys.argv[1:]
    k = float(a[0]) if a else 50000.0
    out = Path(a[1]) if len(a) > 1 else MAN
    m = declare(k, int(a[2]) if len(a) > 2 else 3000)
    t = time.time()
    res = m.run()
    m.record(res)
    (out / f"{m.name}.recorded.genesis.json").write_text(m.to_json(indent=1))
    ss = m.recorded["start_search"]
    print("joints", round(time.time() - t), m.inputs_sha256[:12], ss["evaluations"], ss["walls_met"], ss["final_band"],
          ss["final_violated"], m.recorded["winner_verdict"], m.recorded["winner_yield"],
          [round(c["max_band_frac"], 3) for c in m.recorded["candidates"]], flush=True)
