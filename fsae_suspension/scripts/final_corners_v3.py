# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The final corners with a damper that exists and, at the front, a braking compliance wall (manifests front_v11_final
and rear_v11_final, superseding the declarations of scripts/final_corners_v2.py).

Two requirements are added and nothing else changes:

* Damper. The actuation is checked against a real damper, the Ohlins TTX25 MkII FSAE: 200 mm eye to eye at full
  extension, 57 mm stroke (manufacturer data). It must not top out before full droop and must stay inside its usable
  stroke (80%) at full bump. Each corner's spring mount is moved along the damper's own axis to the middle of that
  window; the line of the spring, and with it the static motion ratio and the rocker, is unchanged (the longer spring
  turns less through travel, so the motion ratio varies less).
* Braking toe. Both front wheels toe out under braking by the same amount, which steers nothing on its own; what steers
  is the difference when one wheel brakes harder than the other. With a left-right brake force difference of up to 60%
  (one front wheel at the point of lock-up) the axle steers e * d / 2, which the +/-0.08 deg toe band bounds:
  d <= 2 x 0.08 / 0.6 = 0.27 deg, declared as |compliance toe in the 1.5 g stop| <= 0.25 deg on the frame-derived
  flexibility with the links in series. The lower wishbone's short base is what puts the braking reaction into its
  legs; moving its inner pickups along their own pivot axis leaves every kinematic curve unchanged (the axis is the
  same line), so the front pickup is moved forward along that axis until the lower-front leg reaches the wheel
  envelope's existing margin, and the search verifies the result with the wall absolute.

usage: python scripts/final_corners_v3.py front|rear [out_dir] [--declare-only]
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "scripts"))
from suspension import actuation as act, elastokinematics as ek, inverse_genesis as ig       # noqa: E402
from suspension.genesis_repro import GenesisManifest, hp_from_dict, hp_to_dict, vehicle_load_cases  # noqa: E402

DAMPER = {"label": "Ohlins TTX25 MkII FSAE", "extended_mm": 200.0, "stroke_mm": 57.0}
BRAKE_TOE_DEG = 0.25                      # see module doc: 60% brake imbalance inside the +/-0.08 deg toe band
SRC = {"front": "front_v11_final.cea1a7c48229", "rear": "rear_v11_final.b0e6ef7369d9"}   # the declarations this builds on


def _src(axle):
    """The earlier declaration of the final corner (scripts/final_corners_v2.py), kept in manifests/superseded."""
    return json.loads((ROOT / "manifests" / "superseded" / f"{SRC[axle]}.recorded.genesis.json").read_text())


def damper_window(hp, targets=None) -> dict:
    """Spring lengths (mm) and the static length that centres the used stroke in the damper's window."""
    t = targets or act.ActuationTargets.derived(damper_extended_mm=DAMPER["extended_mm"], stroke_mm=DAMPER["stroke_mm"])
    m = act.actuation_metrics(hp)
    Ld, L0, Lb = m.spring_length_mm
    lo = t.damper_extended_mm - t.usable_stroke * t.stroke_mm + (L0 - Lb)       # shortest static length that fits bump
    hi = t.damper_extended_mm - (Ld - L0)                                         # longest that keeps droop extended
    return {"lengths": (Ld, L0, Lb), "window": (lo, hi), "target": 0.5 * (lo + hi)}


def fit_damper(hp):
    """Move spring_inner along the damper axis so the static eye-to-eye length sits mid-window (mm)."""
    import copy
    w = damper_window(hp)
    h = copy.deepcopy(hp)
    rs, si = np.asarray(hp.rocker_spring, float), np.asarray(hp.spring_inner, float)
    u = (si - rs) / np.linalg.norm(si - rs)
    for _ in range(3):                                   # the window shifts slightly with the angle; iterate
        L0 = act.actuation_metrics(h).spring_length_mm[1]
        h.spring_inner = np.asarray(h.spring_inner, float) + (w["target"] - L0) * u
        w = damper_window(h)
    return h, w


def widen_lower_base(hp, vol, tol_mm: float = 0.005):
    """Move lower_front_inner forward along the lower pivot axis as far as the wheel envelope keeps the start's
    margin (mm); returns the geometry and the move (mm)."""
    import copy
    A, B = np.asarray(hp.lower_front_inner, float), np.asarray(hp.lower_rear_inner, float)
    u = (B - A) / np.linalg.norm(B - A)
    m0 = vol.envelope_margin(hp)

    def at(d):
        h = copy.deepcopy(hp); h.lower_front_inner = A - d * u
        return h
    lo, hi = 0.0, 120.0
    while hi - lo > 0.25:
        mid = 0.5 * (lo + hi)
        if vol.envelope_margin(at(mid)) >= m0 - tol_mm:
            lo = mid
        else:
            hi = mid
    return at(lo), lo


def braking_case(axle="front"):
    return [c for c in vehicle_load_cases(axle=axle, brake_bias_front=0.73) if c.name == "1.5g braking"][0]


def declare(axle: str, hpd: dict, flex=None) -> GenesisManifest:
    d = _src(axle)
    d["hardpoints"] = hpd
    pr = d["volume"]["properties"]
    if axle == "front":
        pr["elasto"]["stiffness"] = {"default_n_per_mm": 30000.0, "per_point": {}, "provenance": "fea",
                                     "coupled": np.asarray(flex, float).tolist()}
        c = braking_case()
        eb = dict(pr["elasto"]); eb.update({"Fx": float(c.Fx), "Fy": float(c.Fy), "Fz": float(c.Fz), "Mz": float(c.mz_Nmm)})
        pr["elasto_braking"] = eb
        pr["bounds"] = [b for b in pr["bounds"] if b["prop"] != "compliance_toe_braking_deg"] + [{
            "prop": "compliance_toe_braking_deg", "lo": -BRAKE_TOE_DEG, "hi": BRAKE_TOE_DEG,
            "label": (f"-{BRAKE_TOE_DEG} <= compliance steer in the 1.5 g stop, frame-derived pickup flexibility and tube "
                      f"links in series (deg) <= {BRAKE_TOE_DEG}")}]
    ctx = dict(d["context"])
    ctx["run"] = (f"the inputs of {d['name']} (inputs {d['inputs_sha256'][:12]}) with the damper declared "
                  f"({DAMPER['label']}, {DAMPER['extended_mm']:.0f} mm eye to eye extended, {DAMPER['stroke_mm']:.0f} mm stroke: "
                  "the spring mount moved along the damper axis to centre the used stroke, the static motion ratio and the rocker unchanged)"
                  + ("; the lower-front inner pickup moved forward along the lower pivot axis (every kinematic curve unchanged) "
                     "until the lower-front leg reaches the wheel envelope's margin, and a braking compliance wall: "
                     f"|toe in the 1.5 g stop| <= {BRAKE_TOE_DEG} deg (a 60% left-right brake difference then steers the "
                     "axle inside the +/-0.08 deg toe band); pickup flexibility re-derived for the moved pickups "
                     "(scripts/frame_derivation.py)" if axle == "front" else "")
                  + ". Earlier run: " + ctx.get("run", ""))
    d["context"] = ctx
    m = GenesisManifest.from_json(json.dumps({k: v for k, v in d.items() if k not in ("recorded", "inputs_sha256")}))
    m.recorded = None
    return m


def build(axle: str):
    d = _src(axle)
    hp = hp_from_dict(d["recorded"]["winner_hardpoints"])
    vol = GenesisManifest.from_json(json.dumps(d)).objects()[2]
    moved = 0.0
    if axle == "front":
        hp, moved = widen_lower_base(hp, vol)
    hp, w = fit_damper(hp)
    return hp, w, moved


if __name__ == "__main__":
    only = "--declare-only" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--declare-only"]
    axle = args[0]
    out = Path(args[1]) if len(args) > 1 else ROOT / "manifests"
    hp, w, moved = build(axle)
    print(axle, "lower-front pickup moved", round(moved, 2), "mm; damper", [round(v, 1) for v in w["lengths"]],
          "window", [round(v, 1) for v in w["window"]], flush=True)
    flex = None
    if axle == "front":
        import frame_derivation as fd
        fd.HP_OVERRIDE["front"] = hp_to_dict(hp)
        other = build("rear")[0]
        fd.HP_OVERRIDE["rear"] = hp_to_dict(other)
        flex = fd.run()["corners"]["front"]["flexibility_mm_per_N"]
    m = declare(axle, hp_to_dict(hp), flex)
    h, tg, vol, fld = m.objects()
    print(m.name, m.inputs_sha256[:12], "start walls", vol.wall_violations(h),
          {k: round(v, 4) for k, v in vol.properties.evaluate(h).items()}, flush=True)
    (out / f"{m.name}.genesis.json").write_text(m.to_json(indent=1))
    if only:
        sys.exit(0)
    t = time.time(); r = m.run(); m.record(r)
    (out / f"{m.name}.recorded.genesis.json").write_text(m.to_json(indent=1))
    print("IG", round(time.time() - t), m.recorded["winner_verdict"], m.recorded["winner_yield"],
          [round(c["max_band_frac"], 3) for c in m.recorded["candidates"]], flush=True)
