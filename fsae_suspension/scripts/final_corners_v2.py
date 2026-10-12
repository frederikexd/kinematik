# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The final corners re-declared on corrected load cases and derived pickup stiffness (manifests front_v11_final and
rear_v11_final).

The inputs of front_v11_hairpin_walls and rear_v11_walls with these changes and nothing else:
* the cornering cases of the rocker-bearing wall and of the compliance wall put the outer wheel's ground force toward
  the turn centre (inboard, Fy < 0) with the aligning torque that goes with it (Mz < 0); the earlier manifests carried
  both signs reversed (genesis_repro.vehicle_load_cases, corrected);
* the link wall carries the 3 g curb strike with its lateral kick inboard as well as outboard (robustness.shock_cases);
* the front compliance wall uses the pickup flexibility derived from the frame (scripts/frame_derivation.py:
  ChassisV10 with a four-tube node bay at every pickup, frame free and balanced by the car's inertia) in place of the
  declared 30 kN/mm;
* the pushrod acts on the upright beside the lower ball joint (every link stays a two-force member; first tried at the
  ball joint itself, where it ran through the upright, hub and rear halfshaft), and the rocker and spring are re-placed
  for it (scripts/actuation_package.py: same targets, against the frame tubes and the declared envelopes of the upright,
  hub, links and, at the rear, halfshaft and differential, at every wheel position, the rocker axis free); on the
  lower wishbone 177 mm inboard of the ball joint, where the earlier corners carried it, it bent the lower legs
  (loadpath's two-body solution), which bare tubes cannot carry;
* the rear lower legs are 5/8 x 0.065 in (front leg) and 5/8 x 0.049 in (rear leg) and the rear rocker bearings an SKF
  6205-2RSH pair: with the pushrod on the upright aft of the halfshaft the 5/8 x 0.035 in front leg buckles at 1.40 in the
  traction exit with a curb strike and the 6204 pair reaches 1.74 against 2.0;
* the front tie rod is 3/4 x 0.083 in: with the pushrod at the ball joint the 5/8 x 0.035 in tie rod's own give took
  the corner past the compliance wall on the derived frame (0.107 deg); with the pushrod on the upright it would hold
  (0.049 deg) and 3/4 x 0.083 in gives 0.023 deg; it is kept, because the derived frame bounds the built one from above.

The front's actuation points are those printed in supplement Table S3.3 (TABLE_S3_3_FRONT): the earlier declaration they
were synthesised for (inputs adb57b80b750, scripts/actuation_package.py) is not among the published manifests, so
front_v11_final is declared from the printed points on the pickups of front_v11_hairpin_walls ("table_s3.3" below), with
the frame-derived flexibility recomputed for them ("derive": scripts/frame_derivation.py with those hardpoints).
usage: python scripts/final_corners_v2.py front|rear [out_dir] [frame_derived.json | derive] [hardpoints.json | table_s3.3]
       [--declare-only]"""
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "scripts"))
from suspension.genesis_repro import GenesisManifest, vehicle_load_cases                 # noqa: E402
from suspension.robustness import shock_cases                                            # noqa: E402
from suspension import genesis_walls as gw                                               # noqa: E402

SRC = {"front": "front_v11_hairpin_walls", "rear": "rear_v11_walls"}
TIE_ROD = (19.05, 2.108)                     # 3/4 x 0.083 in 4130
REAR_LF, REAR_LR = (15.875, 1.651), (15.875, 1.245)     # 5/8 x 0.065 and 5/8 x 0.049 in, as the front's lower legs
REAR_BEARING = (7800.0, "2 x SKF 6205-2RSH")             # C0 per bearing (N), SKF data
NEW = {"front": "front_v11_final", "rear": "rear_v11_final"}
#: the final front's actuation as printed in supplement Table S3.3 (corner frame, mm; rocker axis to 0.001, normalised)
TABLE_S3_3_FRONT = {"pushrod_outer": [8.22, 522.81, 132.84], "rocker_pivot": [27.04, 365.46, 319.92],
                    "rocker_pushrod": [13.50, 435.40, 360.87], "rocker_spring": [31.29, 332.37, 387.98],
                    "spring_inner": [65.26, 234.78, 361.25], "rocker_axis": [0.972, 0.233, 0.043], "pushrod_attach": "upright"}


def table_s3_3_front() -> dict:
    """front_v11_hairpin_walls's winner with the actuation points of Table S3.3."""
    d = json.loads((ROOT / "manifests" / f"{SRC['front']}.recorded.genesis.json").read_text())["recorded"]["winner_hardpoints"]
    d = dict(d, **{k: v for k, v in TABLE_S3_3_FRONT.items()})
    ax = np.asarray(d["rocker_axis"], float)
    d["rocker_axis"] = (ax / np.linalg.norm(ax)).tolist()
    return d


def declare(axle: str, flex=None, hardpoints: dict | None = None) -> GenesisManifest:
    d = json.loads((ROOT / "manifests" / f"{SRC[axle]}.recorded.genesis.json").read_text())
    if hardpoints is not None:
        d["hardpoints"] = hardpoints
    vol = d["volume"]; pr = vol["properties"]
    pr["structure"]["cases"] = {k: v for k, v in sorted(gw.LinkStructure.from_shock_cases(axle, 0.73).to_dict()["cases"].items())}
    kw = {"traction_g": 1.06, "inboard_drive": True, "inboard_brakes": True} if axle == "rear" else {}
    pr["bearing"]["cases"] = gw.RockerBearingWall(cases=list(vehicle_load_cases(axle=axle, brake_bias_front=0.73, **kw)),
                                                  label=pr["bearing"]["label"], C0_N=pr["bearing"]["C0_N"]).to_dict()["cases"] \
        if hasattr(gw.RockerBearingWall, "to_dict") else None
    if axle == "rear":                        # pushrod on the upright, aft of the halfshaft: the lower legs carry more
        pr["structure"]["per_member"] = {"LF": list(REAR_LF), "LR": list(REAR_LR)}
        pr["bearing"]["C0_N"], pr["bearing"]["label"] = REAR_BEARING
    if axle == "front":                       # tie rod 3/4 x 0.083 in: it carries the cornering toe (see module doc)
        pr["structure"]["per_member"]["TR"] = list(TIE_ROD)
        if "elasto" in pr:
            pr["elasto"]["members"]["TR"] = {"material": "Steel 4130", "od_mm": TIE_ROD[0], "wall_mm": TIE_ROD[1]}
    if "elasto" in pr:
        c = [c for c in vehicle_load_cases(axle=axle, brake_bias_front=0.73) if c.name == "1.5g corner"][0]
        pr["elasto"].update({"Fx": float(c.Fx), "Fy": float(c.Fy), "Fz": float(c.Fz), "Mz": float(c.mz_Nmm)})
        if flex is not None:
            pr["elasto"]["stiffness"] = {"default_n_per_mm": 30000.0, "per_point": {}, "provenance": "fea",
                                         "coupled": np.asarray(flex, float).tolist()}
        for b in pr["bounds"]:
            if b["prop"] == "compliance_toe_deg":
                b["label"] = ("-0.08 <= compliance steer in the 1.5 g corner (outer wheel, force toward the turn centre), "
                              "pickup flexibility derived from the frame and tube links in series (deg) <= 0.08")
    ctx = dict(d["context"])
    ctx["run"] = (f"the inputs of {SRC[axle]} (inputs {d['inputs_sha256'][:12]}) with the cornering cases of the bearing"
                  + (" and compliance walls" if "elasto" in pr else " wall")
                  + " corrected to put the outer wheel's ground force toward the turn centre (Fy < 0, Mz < 0), the curb-strike "
                  "kick declared inboard as well as outboard in the link wall"
                  + ("; the compliance wall on the pickup flexibility derived from the frame (scripts/frame_derivation.py), "
                     "not the declared 30 kN/mm" if "elasto" in pr else "")
                  + "; the pushrod moved onto the upright beside the lower ball joint (every link a two-force member) and the rocker and "
                  "spring re-placed for it against the frame tubes and the declared envelopes of the upright, hub, links"
                  + (", halfshaft and differential" if axle == "rear" else " and rack")
                  + " at every wheel position, with the rocker axis free (scripts/actuation_package.py"
                  + ("; declared at the points printed in supplement Table S3.3)" if axle == "front" else ")")
                  + ("; tie rod 3/4 x 0.083 in" if axle == "front" else
                     "; lower legs 5/8 x 0.065 in (front) and 5/8 x 0.049 in (rear) and SKF 6205-2RSH rocker bearings")
                  + ". Earlier run: " + ctx.get("run", ""))
    m = GenesisManifest.from_json(json.dumps({k: v for k, v in d.items() if k not in ("recorded", "inputs_sha256")}))
    m.name = NEW[axle]; m.context = ctx; m.volume = vol; m.recorded = None
    return m


if __name__ == "__main__":
    only = "--declare-only" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--declare-only"]
    axle = args[0]
    out = Path(args[1]) if len(args) > 1 else ROOT / "manifests"
    src = args[3] if len(args) > 3 else f"/home/claude/runs/final_v2/{axle}_hp.json"
    hpd = table_s3_3_front() if (axle == "front" and src == "table_s3.3") else json.loads(Path(src).read_text())
    flex = None
    if axle == "front":
        fsrc = args[2] if len(args) > 2 else "/home/claude/runs/frame_derived.json"
        if fsrc == "derive":
            import frame_derivation as fd
            fd.HP_OVERRIDE["front"] = hpd
            flex = fd.run()["corners"]["front"]["flexibility_mm_per_N"]
        else:
            flex = json.loads(Path(fsrc).read_text())["corners"]["front"]["flexibility_mm_per_N"]
    m = declare(axle, flex, hpd)
    hp, tg, vol, fld = m.objects()
    print(m.name, m.inputs_sha256[:12], "start walls", vol.wall_violations(hp),
          {k: round(v, 4) for k, v in vol.properties.evaluate(hp).items()}, flush=True)
    (out / f"{m.name}.genesis.json").write_text(m.to_json(indent=1))
    if only:
        sys.exit(0)
    t = time.time(); r = m.run(); m.record(r)
    (out / f"{m.name}.recorded.genesis.json").write_text(m.to_json(indent=1))
    print("IG", round(time.time() - t), m.recorded["winner_verdict"], m.recorded["winner_yield"],
          [round(c["max_band_frac"], 3) for c in m.recorded["candidates"]], flush=True)
