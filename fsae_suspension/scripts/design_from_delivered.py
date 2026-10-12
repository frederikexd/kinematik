# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The search designing from a bad start: the delivered ChassisV10 front under every wall of front_v11_final.

The delivered front (front_steering_walls) meets the curve bands but fails three of the final front's walls: it needs
137 deg of steering wheel to reach the 4.5 m hairpin (<= 120), runs -1.9% Ackermann there (>= 75) and toes each wheel
out 0.48 deg in a 1.5 g stop (+/- 0.25). InverseGenesis refuses a start inside a wall, so the search runs in two
recorded stages, both deterministic:

1. genesis_seed.seed_search moves the free pickups, inside the same +/-100 mm boxes, until every wall holds. With
   ``walls_only`` the cost is the wall penalty alone and it stops as soon as the walls hold, wherever the curves are.
2. InverseGenesis, every wall absolute, then brings the camber, toe and roll-centre curves into their bands and prices
   the build yield.

The manifest is the final front's inputs (targets, boxes, tolerance, search settings, every wall, the frame-derived
pickup flexibility and the actuation of front_v11_final, pushrod on the upright) with the failing start's pickups as
its hardpoints and stage 1 declared as its start_search (GenesisManifest.restarted), so running or re-running it
repeats both stages and its recorded outputs carry the pattern search's report. --stage1 and --via keep their seed
stages in this script and record them in the context.

usage: python scripts/design_from_delivered.py [walls_only|seeded] [out_dir] [max_evals] [--from 29deg]
                                              [--stage1 previous.genesis.json] [--via M]

--from 29deg starts from the 85 mm-base ChassisV11 candidate (front_v11_29deg winner) instead of the delivered front;
--stage1 continues the seed search from a previous stage's result, recording both stages in the context.
--via M runs a first seed stage under manifest M's own targets and walls, then the final front's (recorded as
       <name>_via_<M>); from the delivered front via front_v11_29deg it stalls in the second stage (supplement S13.5).
"""
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import genesis_seed as gs                                                       # noqa: E402
from suspension.genesis_repro import GenesisManifest, hp_from_dict, hp_to_dict                               # noqa: E402

MAN = ROOT / "manifests"
ACTUATION = ("pushrod_outer", "rocker_pivot", "rocker_axis", "rocker_pushrod", "rocker_spring", "spring_inner",
             "pushrod_attach")


START = {"delivered": ("front_steering_walls", "the delivered ChassisV10 front's pickups", "front_v11_from_delivered",
                       "which fails the lock (137 deg of wheel), Ackermann (-1.9%) and braking compliance (0.48 deg) walls"),
         "29deg": ("front_v11_29deg", "the 85 mm-base ChassisV11 candidate's pickups", "front_v11_from_85mm",
                   "which fails the link wall (1.37 on the declared legs) and the braking compliance wall (0.77 deg)")}


def declare(variant: str = "walls_only", start: str = "delivered") -> GenesisManifest:
    """Every input of front_v11_final with another start (its pickups, the final front's actuation)."""
    src, what, name, fails = START[start]
    F = GenesisManifest.from_json((MAN / "front_v11_final.recorded.genesis.json").read_text())
    D = json.loads((MAN / f"{src}.recorded.genesis.json").read_text())
    hp = dict(D["recorded"]["winner_hardpoints"])
    fh = json.loads(F.to_json())["hardpoints"]
    for k in ACTUATION:
        hp[k] = fh[k]
    m = F.restarted(name + ("_walls_only" if variant == "walls_only" else ""), hp_from_dict(hp), context={
        "paper": "v11", "axle": "front", "run": (
            "the search from a bad start: every input of front_v11_final (inputs " + F.inputs_sha256[:12] + "; "
            "targets, +/-100 mm boxes, tolerance field, search settings, every wall, the frame-derived pickup "
            "flexibility, the braking case and the actuation with the pushrod on the upright) with " + what + " ("
            + src + " winner, inputs " + D["inputs_sha256"][:12] + ") as the start, " + fails + "; the pattern search "
            "(genesis_seed, declared as this manifest's start_search, so a re-run repeats it) "
            + ("on the wall penalty alone, stopping as soon as every wall holds" if variant == "walls_only" else
               "on band fraction plus wall penalties") + ", then InverseGenesis with every wall absolute "
            "(scripts/design_from_delivered.py)")})
    del m.context["declared_from"]
    return m


def declared_run(variant="seeded", out=MAN, max_evals=4000, start="delivered") -> GenesisManifest:
    """The search from a failing start as one manifest: the start search is declared, so run() and every re-run
    repeat both stages. graded_keepouts=False is the setting the published searches were made with."""
    m = declare(variant, start)
    m.start_search = asdict(gs.SeedSearch(seed=1, max_evals=max_evals, step0_mm=16.0, step_min_mm=0.125,
                                          clearance_target_mm=0.45, stop_band_frac=0.6,
                                          walls_only=(variant == "walls_only"), graded_keepouts=False))
    t = time.time()
    res = m.run()
    m.record(res)
    (Path(out) / f"{m.name}.recorded.genesis.json").write_text(m.to_json(indent=1))
    ss = m.recorded["start_search"]
    print("declared", round(time.time() - t), m.inputs_sha256[:12], ss["evaluations"], ss["walls_met"],
          ss["final_band"], m.recorded["winner_verdict"], m.recorded["winner_yield"],
          [round(c["max_band_frac"], 3) for c in m.recorded["candidates"]], flush=True)
    return m


def main(variant="walls_only", out=MAN, max_evals=4000, start="delivered", stage1=None, via=None):
    m = declare(variant, start)
    hp, tg, vol, _ = m.objects()
    prev = None
    if via:                                      # stage 1 under an intermediate manifest's own targets and walls
        V = json.loads((MAN / f"{via}.recorded.genesis.json").read_text())
        vm = GenesisManifest.from_json(json.dumps({k: v for k, v in V.items() if k != "recorded"}))
        _, vtg, vvol, _ = vm.objects()
        st1 = gs.SeedSearch(seed=1, max_evals=max_evals, step0_mm=16.0, step_min_mm=0.125, clearance_target_mm=0.45,
                            stop_band_frac=0.6)
        t = time.time()
        hp, prev = gs.seed_search(hp, vtg, vvol, st1, log=lambda s_: print("stage1", s_, flush=True))
        prev = dict(prev); prev["declared_inputs"] = f"{via} (inputs {V['inputs_sha256'][:12]}): its targets and walls"
        print("stage1", round(time.time() - t), prev["evaluations"], prev["walls_hold"], prev["final_band"], flush=True)
    if stage1:                                   # continue from a previous stage's start, from the same box origin
        pd = json.loads(Path(stage1).read_text())
        prev = pd["context"]["seed_search"]
        from suspension.genesis_repro import hp_from_dict
        hp_prev = hp_from_dict(pd["hardpoints"])
        x0 = np.array([getattr(hp_prev, p)[a] - getattr(hp, p)[a] for (p, a) in vol.coords()], float)
        from suspension import inverse_genesis as ig
        hp = ig._shifted(hp, vol, x0)
    st = gs.SeedSearch(seed=2 if (stage1 or via) else 1, max_evals=max_evals, step0_mm=16.0, step_min_mm=0.125,
                       clearance_target_mm=0.45, stop_band_frac=0.6, walls_only=(variant == "walls_only"))
    t = time.time()
    hp1, rep = gs.seed_search(hp, tg, vol, st, log=lambda s: print(s, flush=True))
    print("seed", round(time.time() - t), rep["evaluations"], rep["walls_hold"], rep["final_band"], rep["final_violated"],
          flush=True)
    d = json.loads(m.to_json())
    d.pop("inputs_sha256", None); d.pop("recorded", None)
    if prev is not None:
        d["context"]["seed_search_stage1"] = prev
    d["context"]["seed_search"] = rep
    d["context"]["seed_start_hardpoints"] = (START[start][0] + " winner with the actuation of front_v11_final"
                                             + ("; stage 2 from the stage 1 result" if prev is not None else ""))
    if via:
        d["name"] = d["name"] + "_via_" + via.replace("front_v11_", "")
        d["context"]["run"] += ("; stage 1 of genesis_seed runs under " + prev["declared_inputs"] + ", stage 2 under the "
                                "final front's, then InverseGenesis")
    d["hardpoints"] = hp_to_dict(hp1)
    m = GenesisManifest.from_json(json.dumps(d))
    (Path(out) / f"{m.name}.genesis.json").write_text(m.to_json(indent=1))
    if not rep["walls_hold"]:
        print("seed: " + ("every wall holds" if not rep["final_violated"] else f"walls broken {rep['final_violated']}")
              + f", curves at {rep['final_band']:.3f} of their bands", flush=True)
    t = time.time(); r = m.run(); m.record(r)
    (Path(out) / f"{m.name}.recorded.genesis.json").write_text(m.to_json(indent=1))
    print("IG", round(time.time() - t), m.inputs_sha256[:12], m.recorded["winner_verdict"], m.recorded["winner_yield"],
          [round(c["max_band_frac"], 3) for c in m.recorded["candidates"]], flush=True)


if __name__ == "__main__":
    a = sys.argv[1:]
    kw = {}
    if "--from" in a:
        i = a.index("--from"); kw["start"] = a[i + 1]; del a[i:i + 2]
    if "--via" in a:
        i = a.index("--via"); kw["via"] = a[i + 1]; del a[i:i + 2]
    if "--stage1" in a:
        i = a.index("--stage1"); kw["stage1"] = a[i + 1]; del a[i:i + 2]
    args = (a[0] if a else "walls_only", Path(a[1]) if len(a) > 1 else MAN, int(a[2]) if len(a) > 2 else 4000)
    if kw.get("stage1") or kw.get("via"):
        main(*args, **kw)                                 # staged runs: the seed stages in the script, recorded in context
    else:
        declared_run(*args, start=kw.get("start", "delivered"))
