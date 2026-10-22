# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The claims scripts/paper_check.py checks: each quote is printed in the paper (P) or supplement (S) and must carry
the value its code recomputes from the published manifests (front_v11_final, rear_v11_final unless named)."""
import re

from paper_check import Claim, manifest, winner
from suspension.genesis_repro import GenesisManifest, hp_from_dict

C = Claim
F = lambda k: (lambda c: c.corners["F"][k])        # noqa: E731
R = lambda k: (lambda c: c.corners["R"][k])        # noqa: E731
ST = lambda k: (lambda c: c.steering[k])           # noqa: E731
WF = lambda k: (lambda c: c.walls["F"][k])         # noqa: E731
WR = lambda k: (lambda c: c.walls["R"][k])         # noqa: E731
AF = lambda k: (lambda c: c.actuation["F"][k])     # noqa: E731
AR = lambda k: (lambda c: c.actuation["R"][k])     # noqa: E731


def _fit(name):
    return lambda c: manifest(name).recorded["candidates"][0]["max_band_frac"]


def _hash(name):
    return lambda c: int(manifest(name).inputs_sha256[:12], 16)


def _refused(name, prop):
    """A walled manifest's recorded winner (its start), judged by its own walls on the current code."""
    def f(c):
        m = manifest(name); vol = m.objects()[2]
        return vol.properties.evaluate(winner(name))[prop]
    return f


def _brief_hash(c):
    import hashlib, json
    from paper_check import MAN
    b = json.loads((MAN / "chassisv11_node_brief.json").read_text())
    h = b.pop("inputs_sha256")
    assert hashlib.sha256(json.dumps(b, sort_keys=True, separators=(",", ":")).encode()).hexdigest() == h
    for co in b["corners"].values():
        assert manifest(co["source_manifest"]).inputs_sha256 == co["source_inputs_sha256"]
    return int(h[:12], 16)


LBIN = 0.17512683                     # N/mm per lb/in
TRACK_M = 1.21                        # m, the declared track
DAMPER_MM = 200.0                     # Ohlins TTX25 MkII FSAE, eye to eye at full extension (paper reference 35)


def _base(hp):
    import numpy as np
    return float(np.linalg.norm(np.subtract(hp.lower_rear_inner, hp.lower_front_inner)))


def _lf_move(c):
    """How far the final front's lower-front inner pickup sits from the declaration it was built on (mm)."""
    import json
    import numpy as np
    from paper_check import ROOT
    d = json.loads((ROOT / "manifests" / "superseded" / "front_v11_final.cea1a7c48229.recorded.genesis.json").read_text())
    return float(np.linalg.norm(np.subtract(c.hF.lower_front_inner, d["recorded"]["winner_hardpoints"]["lower_front_inner"])))


def _seedrep(name):
    """The pattern search's report: recorded with the outputs when the manifest declares its start search, else the
    context of the earlier two-stage records."""
    m = manifest(name)
    return (m.recorded or {}).get("start_search") or m.context["seed_search"]


def _env(name):
    def f(c):
        m = manifest(name)
        return m.objects()[2].envelope_margin(winner(name))
    return f


CLAIMS = [
    # ---- identity of the final corners -------------------------------------------------------------------- #
    C("final-front-hash", "P §6.1", "front_v11_final (2d35663ecd27) and rear_v11_final", _hash("front_v11_final"), "{:012x}"),
    C("final-rear-hash", "P §6.1", "rear_v11_final (1d2b4a0a7fca) are each returned by the optimizer", _hash("rear_v11_final"), "{:012x}"),
    C("final-front-verdict", "P Table 6", "RESILIENT, 100%", lambda c: 100 * manifest("front_v11_final").recorded["winner_yield"], "{:.0f}"),
    C("final-front-fit", "P Table 6", "0.347", _fit("front_v11_final"), "{:.3f}"),
    C("final-rear-fit", "P Table 6", "RESILIENT, 100%   0.41", _fit("rear_v11_final"), "{:.2f}"),
    C("final-front-walls", "P §6.1", "each returned unchanged by InverseGenesis as its winner, RESILIENT at 100%",
      lambda c: float(len(c.walls["F_viol"]) + len(c.walls["R_viol"])), "{:.0f}"),
    # ---- kinematics (same pickups as front_v11_hairpin_walls and rear_v11_walls) ----------------------------- #
    C("rc-front-rear", "P §3.1", "the geometric ones stay at 53 and 41 mm", lambda c: c.corners["F"]["rc"], "{:.0f}"),
    C("share", "P Table 3", "0.78 deg/g; 52.5%", lambda c: 100 * c.corners["F"]["share"]),
    C("gain-front", "P §3.1", "The final gains (−0.035 and −0.029 deg/mm)", F("gain"), "{:.3f}"),
    C("gain-rear", "P §3.1", "The final gains (−0.035 and −0.029 deg/mm)", R("gain"), "{:.3f}"),
    C("bump-steer-front", "P §5.4", "steer of the final corners is 0.028 degrees front and 0.033 degrees rear over", F("bump_steer"), "{:.3f}"),
    C("bump-steer-rear", "P §5.4", "Bump steer of the final corners is 0.028 degrees front and 0.033 degrees rear", R("bump_steer"), "{:.3f}"),
    C("anti-dive-73", "P §3.1", "at the final front’s +68.9% anti-dive (73% bias), and", F("ad73")),
    C("anti-dive-60b", "P §5.5", "68.9% anti-dive there (56.7% at 60%)", F("ad60")),
    C("anti-squat", "P §3.1", "the final rear’s +46.7% anti-squat", R("as")),
    C("anti-lift-in", "P §3.1", "+12.6% with inboard brakes at 73% bias", R("al_in")),
    C("anti-lift-out", "P §3.1", "the final rear sits -86.8% with outboard brakes and", R("al_out")),
    C("floor-brake", "P §3.1", "The floors are 1.42 Hz front straight-line braking", lambda c: c.targets["f_brake"], "{:.2f}"),
    C("floor-comb-F", "P §3.1", "2.00 Hz combined braking and cornering", lambda c: c.targets["f_comb_F"], "{:.2f}"),
    C("floor-rear", "P §3.1", "and 1.77 and 2.50 Hz at the final rear’s +46.7%", lambda c: c.targets["f_trac"], "{:.2f}"),
    C("floor-rear-c", "P §3.1", "and 1.77 and 2.50 Hz at the final rear’s +46.7%", lambda c: c.targets["f_comb_R"], "{:.2f}"),
    C("ride-derived-F", "P §3.1", "the floor falls to 2.21 and 2.76 Hz", lambda c: c.targets["ride_F"], "{:.2f}"),
    C("ride-derived-R", "P §3.1", "the floor falls to 2.21 and 2.76 Hz", lambda c: c.targets["ride_R"], "{:.2f}"),
    C("mu-rear", "P §3.3", "holds while peak stays at or below 1.83 (front-dive 1.95). Lower grip", lambda c: c.targets["mu_R"], "{:.2f}"),
    C("mu-front", "P §3.3", "data and holds while peak stays at or below 1.83 (front-dive 1.95).", lambda c: c.targets["mu_F"], "{:.2f}"),
    C("dive", "P §4", "rate. 1.5g braking dives the front about 7.3 mm.", lambda c: c.pitch["dive"]),
    C("rise", "P §5.3", "the rear rises 14.1 mm", lambda c: c.pitch["rise"]),
    C("unload", "P §5.3", "The rear springs then unload 331 N", lambda c: c.pitch["unload"], "{:.0f}"),
    C("pitch", "P §5.3", "is 0.75 degrees of pitch at 1.5g braking", lambda c: c.pitch["deg"], "{:.2f}"),
    C("pitch18", "P §5.3", "and about 0.90 degrees at 1.8g", lambda c: c.pitch["deg18"], "{:.2f}"),
    C("migration-share", "P Table 3", "0.09 points; the pitch itself adds 1.9 points above", lambda c: abs(c.pitch["shift_body_pts"]), "{:.2f}"),
    # ---- steering ---------------------------------------------------------------------------------------------- #
    C("ratio-centre", "P Table 3", "4.99:1 at center, 25.6 degrees mean steer at 118.9 degrees", ST("ratio_centre"), "{:.2f}"),
    C("lock-sw", "P Table 3", "4.99:1 at center, 25.6 degrees mean steer at 118.9 degrees;", ST("lock_sw")),
    C("ratio-avg", "P Table 6", "4.64:1 averaged, 4.99:1 at center; margin +0.95", ST("ratio_avg"), "{:.2f}"),
    C("effort-margin", "P Table 6", "4.64:1 averaged, 4.99:1 at center; margin +0.95", ST("effort_margin"), "{:.2f}"),
    C("inner", "P §5.2", "28.7 degrees inner, 22.5 degrees outer, 75.3% Ackermann) at", ST("inner")),
    C("outer", "P §5.2", "28.7 degrees inner, 22.5 degrees outer, 75.3% Ackermann) at", ST("outer")),
    C("ackermann", "P §5.2", "28.7 degrees inner, 22.5 degrees outer, 75.3% Ackermann) at", ST("ack")),
    C("camber-lock-out", "P Table 10", "outside -3.61 degrees, inside 0.18 degrees", ST("cam_out"), "{:.2f}"),
    C("camber-lock-in", "P Table 10", "outside -3.61 degrees, inside 0.18 degrees", ST("cam_in"), "{:.2f}"),
    C("misalign", "P Table 10", "4.4 degrees worst joint", ST("misalign")),
    C("swing", "P Table 10", "[link swing 32.9 degrees, an upper bound]", ST("swing")),
    C("clearance-F", "P Table 10", "3.49 mm (0.49 beyond the 3 mm wall)", lambda c: c.walls["F_env"], "{:.2f}"),
    C("clearance-R", "P Table 10", "4.19 mm (1.19 beyond the wall)", lambda c: c.walls["R_env"], "{:.2f}"),
    C("decel", "P §5.5", "the balanced bias is 73% to the nearest point, 1.57g (24%", WF("brake_decel_g"), "{:.2f}"),
    C("twist-toe-per-mm", "P §5.4", "0.91 degrees per mm of rod, solved on the linkage),", lambda c: c.twist["toe_per_mm"], "{:.2f}"),
    C("twist-one-flat", "P §5.4", "moves the final front’s toe 0.160 degrees (0.91", lambda c: c.twist["one_flat"], "{:.3f}"),
    C("twist-618", "P §9.1", "gives 0.15 degrees per wheel under 618 Nm", lambda c: c.twist["t618"], "{:.2f}"),
    # ---- the rear's walls and actuation (rear_v11_final) ------------------------------------------------------ #
    C("rear-link", "P Table 6", "link factor 4.63 (lower legs", WR("link_fos_min"), "{:.2f}"),
    C("rear-bearing", "P Table 6", "bearings 2.08 (6205-2RSH)", WR("rocker_bearing_sf"), "{:.2f}"),
    C("rear-mr", "P Table 7", "0.821; 3.4%", AR("mr"), "{:.3f}"),
    C("rear-spread", "P Table 7", "0.821; 3.4%", lambda c: 100 * c.actuation["R"]["spread"]),
    C("rear-stroke", "P Table 7", "40.8 mm", AR("stroke")),
    C("rear-tilt", "S S6.37", "rear           15                       0.821", AR("tilt"), "{:.0f}", doc=""),
    C("rear-bottoms", "P §5.5", "closes at 45.6 mm of bump front and 43.1 mm rear", AR("ot_bottom")),
    C("front-bottoms", "P §5.5", "closes at 45.6 mm of bump front and 43.1 mm rear", AF("ot_bottom")),
    C("front-topout", "P §5.5", "the damper, 2.8 mm front and 2.4 mm rear short of top-out", lambda c: DAMPER_MM - c.actuation["F"]["spring_mm"][0]),
    C("rear-topout", "P §5.5", "the damper, 2.8 mm front and 2.4 mm rear short of top-out", lambda c: DAMPER_MM - c.actuation["R"]["spring_mm"][0]),
    C("front-eye-t7", "P Table 7", "- 197.2/176.8/157.2 mm -", lambda c: c.actuation["F"]["spring_mm"][1]),
    C("front-eye-t7-d", "P Table 7", "- 197.2/176.8/157.2 mm -", lambda c: c.actuation["F"]["spring_mm"][0]),
    C("front-eye-t7-b", "P Table 7", "- 197.2/176.8/157.2 mm -", lambda c: c.actuation["F"]["spring_mm"][2]),
    C("rear-eye-t7", "P Table 7", "mm - 197.6/177.0/156.8 mm", lambda c: c.actuation["R"]["spring_mm"][1]),
    C("rear-eye-t7-d", "P Table 7", "mm - 197.6/177.0/156.8 mm", lambda c: c.actuation["R"]["spring_mm"][0]),
    C("rear-eye-t7-b", "P Table 7", "mm - 197.6/177.0/156.8 mm", lambda c: c.actuation["R"]["spring_mm"][2]),
    C("spring-F-lbin", "P §5.4", "the front’s 24.8 N/mm (142 lb/inch)", lambda c: c.packaging["F"]["spring"] / LBIN, "{:.0f}"),
    C("spring-cat-F-hz", "P §5.4", "with the 150 lb/inch spring instead the front runs at 2.68 Hz",
      lambda c: 2.6 * (150 * LBIN / c.packaging["F"]["spring"]) ** 0.5, "{:.2f}"),
    C("spring-cat-bar", "P §5.4", "about 12 Nm/degree less front bar keeps the roll-stiffness share",
      lambda c: (150 * LBIN - c.packaging["F"]["spring"]) * c.actuation["F"]["mr"] ** 2 * 1000 * (TRACK_M ** 2) / 2 * 3.141592653589793 / 180,
      "{:.0f}"),
    C("spring-cat-R-hz", "P §5.4", "200 lb/inch spring (35.0 N/mm, 3.01 Hz)l", lambda c: 3.0 * (200 * LBIN / c.packaging["R"]["spring"]) ** 0.5, "{:.2f}"),
    C("spring-cat-R-N", "P §5.4", "200 lb/inch spring (35.0 N/mm, 3.01 Hz)l", lambda c: 200 * LBIN + 0 * c.packaging["R"]["spring"]),
    C("eye-text", "P §5.4", "176.8 and 177.0 mm eye to eye at ride height", lambda c: c.actuation["R"]["spring_mm"][1]),
    # ---- the final front's walls, actuation, structure and compliance (front_v11_final) ------------------------- #
    C("front-link-s1", "P §1", "link factor 2.85 on ⅝ x 0.065 inch and ⅝ x 0.049 inch legs, bearings 2.44", WF("link_fos_min"), "{:.2f}"),
    C("front-bearing-s1", "P §1", "link factor 2.85 on ⅝ x 0.065 inch and ⅝ x 0.049 inch legs, bearings 2.44", WF("rocker_bearing_sf"), "{:.2f}"),
    C("front-link-t6", "P Table 6", "lower base 167 mm, link factor 2.85, bearings 2.44", WF("link_fos_min"), "{:.2f}"),
    C("front-ctoe-t6", "P Table 6", "compliance toe -0.012 degrees cornering, 0.23 degrees braking", WF("compliance_toe_deg"), "{:.3f}"),
    C("front-btoe-t6", "P Table 6", "compliance toe -0.012 degrees cornering, 0.23 degrees braking", WF("compliance_toe_braking_deg"), "{:.2f}"),
    C("front-base", "P §5.2", "so it moves 41 mm, to a 167 mm base", lambda c: _base(c.hF), "{:.0f}"),
    C("front-move", "P §5.2", "so it moves 41 mm, to a 167 mm base", _lf_move, "{:.0f}"),
    C("front-btoe-52", "P §5.2", "braking toe falls to 0.23 degrees,", WF("compliance_toe_braking_deg"), "{:.2f}"),
    C("front-btoe-axle", "S D3", "per-wheel toe, 0.07 degrees, inside the +/- 0.08 degree band", lambda c: 0.3 * c.walls["F"]["compliance_toe_braking_deg"], "{:.2f}", doc=""),
    C("front-btoe-row9", "P Table 10", "toe 0.23 degrees per front wheel in a 1.5g stop,", WF("compliance_toe_braking_deg"), "{:.2f}"),
    C("front-mr", "P §5.2", "(motion ratio 0.804, swept clearance 9.5 mm", AF("mr"), "{:.3f}"),
    C("front-clear", "P §5.2", "(motion ratio 0.804, swept clearance 9.5 mm", lambda c: c.packaging["F"]["clear_all"]),
    C("front-mr-t7", "P Table 7", "0.804; 7.7%", AF("mr"), "{:.3f}"),
    C("front-spread-t7", "P Table 7", "0.804; 7.7%", lambda c: 100 * c.actuation["F"]["spread"]),
    C("front-stroke-t7", "P Table 7", "40.0 mm", AF("stroke")),
    C("front-tr-lo", "P Table 7", "61-112 degrees; 4.9 degrees", lambda c: c.actuation["F"]["tr"][0], "{:.0f}"),
    C("front-tr-hi", "P Table 7", "61-112 degrees; 4.9 degrees 28", lambda c: c.actuation["F"]["tr"][1], "{:.0f}"),
    C("front-oop", "P Table 7", "61-112 degrees; 4.9 degrees", AF("oop")),
    C("front-node-t7", "P Table 7", "196 mm; 9.5 mm", lambda c: c.packaging["F"]["node_reach"], "{:.0f}"),
    C("rear-node-t7", "P Table 7", "143 mm; 5.1 mm", lambda c: c.packaging["R"]["node_reach"], "{:.0f}"),
    C("rear-clear-t7", "P Table 7", "143 mm; 5.1 mm", lambda c: c.packaging["R"]["clear_all"]),
    C("front-spring-t7", "P Table 7", "24.8 N/mm; 2.43 to 2.63", lambda c: c.packaging["F"]["spring"]),
    C("front-ride-lo", "P Table 7", "24.8 N/mm; 2.43 to 2.63 Hz", lambda c: c.packaging["F"]["ride"][0], "{:.2f}"),
    C("front-ride-hi", "P Table 7", "24.8 N/mm; 2.43 to 2.63 Hz", lambda c: c.packaging["F"]["ride"][1], "{:.2f}"),
    C("rear-spring-t7", "P Table 7", "34.8 N/mm; 2.90 to 3.00", lambda c: c.packaging["R"]["spring"]),
    C("bracket-reach", "P §5.4", "on a bracket 64 mm front and 47 mm rear from the lower ball joint", AF("bracket_mm"), "{:.0f}"),
    C("bracket-reach-R", "P §5.4", "on a bracket 64 mm front and 47 mm rear from the lower ball joint", AR("bracket_mm"), "{:.0f}"),
    C("rocker-tilt", "P §5.4", "the rocker tilt 14 degrees front and 15 degrees rear,", AF("tilt"), "{:.0f}"),
    C("rocker-tilt-R", "P §5.4", "the rocker tilt 14 degrees front and 15 degrees rear, and", AR("tilt"), "{:.0f}"),
    C("mount-reach", "P §5.4", "the mounts sit 196 and 143 mm from the nearest V10 node", lambda c: c.packaging["F"]["node_reach"], "{:.0f}"),
    C("mount-reach-R", "P §5.4", "the mounts sit 196 and 143 mm from the nearest V10 node", lambda c: c.packaging["R"]["node_reach"], "{:.0f}"),
    C("roll-change-F", "P §5.4", "spring roll stiffness changes −4.4 and −1.8% at 12.4 mm of bump", lambda c: c.packaging["F"]["roll_change_pct"]),
    C("roll-change-R", "P §5.4", "spring roll stiffness changes −4.4 and −1.8% at 12.4 mm of bump", lambda c: c.packaging["R"]["roll_change_pct"]),
    C("setup-spring", "P §3.1", "rate, 24.8 N/mm spring at the front)", lambda c: c.packaging["F"]["spring"]),
    C("warp-spring-out", "P §3.1", "lengthens the outer spring by 4.7 mm of wheel travel (1.2 mm inner)", lambda c: c.warp["spring_dz_right_mm"]),
    C("warp-spring-in", "P §3.1", "lengthens the outer spring by 4.7 mm of wheel travel (1.2 mm inner)", lambda c: c.warp["spring_dz_left_mm"]),
    C("warp-pair", "P §3.1", "together they shift load diagonally by 44 N", lambda c: abs(c.warp["front_pair_N"]), "{:.0f}"),
    C("warp-patch", "P §3.1", "Steering to the hairpin lock lifts the outer patch 0.76 mm and drops the inner 0.96 mm",
      lambda c: c.warp["patch_dz_right_mm"], "{:.2f}"),
    C("warp-row13", "P Table 10", "RF -44 N, LF +44 N, RR +44 N, LR -44", lambda c: c.warp["left_steer_N"]["FR"], "{:.0f}"),
    C("corner-load", "P §3.1", "and up to 1434 N (+14.9%) with 442 N of downforce", lambda c: c.corner_load, "{:.0f}"),
    C("steer-spring-S", "S S6.37", "the front spring changes length by +3.8 and -1.0 mm at fixed ride height",
      lambda c: c.packaging["F"]["steer_spring"][0], "{:.1f}", doc="S"),
    C("t8-LF-load", "P Table 8", "+8736", lambda c: c.structure["F"]["LF"]["worst_n"], "{:.0f}"),
    C("t8-LF-yield", "P Table 8", "lower front (437), ⅝ × 0.049 in: yield 2.85",
      lambda c: c.structure["F"]["LF"]["fos_yield"], "{:.2f}", doc=""),
    C("t8-LR-load", "P Table 8", "−7744", lambda c: c.structure["F"]["LR"]["worst_n"], "{:.0f}"),
    C("t8-LR-buck", "P §5.5", "carry 7.7 kN of compression at buckling 2.85 and yield 3.46 and 8.7 kN of tension at yield 2.85",
      lambda c: c.structure["F"]["LR"]["fos_buckling"], "{:.2f}"),
    C("t8-LR-yield", "P §5.5", "carry 7.7 kN of compression at buckling 2.85 and yield 3.46 and 8.7 kN of tension at yield 2.85",
      lambda c: c.structure["F"]["LR"]["fos_yield"], "{:.2f}"),
    C("t8-LF-yield-s", "P §5.5", "carry 7.7 kN of compression at buckling 2.85 and yield 3.46 and 8.7 kN of tension at yield 2.85",
      lambda c: c.structure["F"]["LF"]["fos_yield"], "{:.2f}"),
    C("t8-bare-LR-b", "P §5.5", "(1.78/1.85 and 2.08 on that tube)", lambda c: c.structure["F_bare"]["LR"]["fos_buckling"], "{:.2f}"),
    C("t8-bare-LR-y", "P §5.5", "(1.78/1.85 and 2.08 on that tube)", lambda c: c.structure["F_bare"]["LR"]["fos_yield"], "{:.2f}"),
    C("t8-bare-LF", "P §5.5", "(1.78/1.85 and 2.08 on that tube)", lambda c: c.structure["F_bare"]["LF"]["fos_yield"], "{:.2f}"),
    C("t8-others", "P §5.5", "every other front link clears 6.7 and the pushrod 7.2",
      lambda c: min(min(v["fos_buckling"], v["fos_yield"]) for k, v in c.structure["F"].items() if k not in ("LF", "LR"))),
    C("t8-pushrod", "P §5.5", "every other front link clears 6.7 and the pushrod 7.2", lambda c: c.pushrod["F"]["yield"]),
    C("t8-rear-UR", "P Table 8", "rear upper rear (359) -1687 curb strike 3g + kick", lambda c: c.structure["R"]["UR"]["worst_n"], "{:.0f}", doc="S"),
    C("first-yield-F", "P Table 8", "load factor to first yield 2.37 (1.45 on ⅝ x 0.035 inch)", lambda c: c.load_factors["F_LR"], "{:.2f}"),
    C("first-yield-F-bare", "P Table 8", "load factor to first yield 2.37 (1.45 on ⅝ x 0.035 inch)", lambda c: c.load_factors["F_LR_bare"], "{:.2f}"),
    C("first-yield-R", "P Table 8", "load factors to first yield 4.03 on the lower-front leg and 7.92 on the upper-rear", lambda c: c.load_factors["R_LF"], "{:.2f}"),
    C("first-yield-R-UR", "P Table 8", "load factors to first yield 4.03 on the lower-front leg and 7.92 on the upper-rear", lambda c: c.load_factors["R_UR"], "{:.2f}"),
    C("b6001-F", "P §5.5", "a static safety factor of 0.88 front and 0.63 rear", lambda c: c.bearings["F"]["s0_6001"], "{:.2f}"),
    C("b6001-R", "P §5.5", "a static safety factor of 0.88 front and 0.63 rear", lambda c: c.bearings["R"]["s0_6001"], "{:.2f}"),
    C("C0-30-F", "P §5.5", "static rating of 5.4 kN front and 7.5 kN rear at 30 mm spacing (4.4 and 6.1 kN at 60 mm).", lambda c: c.bearings["F"]["C0_30"] / 1000),
    C("C0-30-R", "P §5.5", "static rating of 5.4 kN front and 7.5 kN rear at 30 mm spacing (4.4 and 6.1 kN at 60 mm).", lambda c: c.bearings["R"]["C0_30"] / 1000),
    C("C0-60-F", "P §5.5", "static rating of 5.4 kN front and 7.5 kN rear at 30 mm spacing (4.4 and 6.1 kN at 60 mm).", lambda c: c.bearings["F"]["C0_60"] / 1000),
    C("C0-60-R", "P §5.5", "static rating of 5.4 kN front and 7.5 kN rear at 30 mm spacing (4.4 and 6.1 kN at 60 mm).", lambda c: c.bearings["R"]["C0_60"] / 1000),
    C("bear-F", "P §5.5", "30 mm apart, giving 2.44 front and 2.08 rear", lambda c: c.bearings["F"]["s0"], "{:.2f}"),
    C("hub-F", "P §5.5", "hubs clear the ChassisV10 tubes by 23.1 and 53.9 mm", lambda c: c.packaging["F"]["hub"]),
    C("hub-R", "P §5.5", "hubs clear the ChassisV10 tubes by 23.1 and 53.9 mm", lambda c: c.packaging["R"]["hub"]),
    C("bracket-load", "P §5.5", "8.7 kN in a curb strike while braking, gives 2.13 at the node", lambda c: c.bracket["F"] / 1000),
    C("bracket-bound", "P §5.5", "8.7 kN in a curb strike while braking, gives 2.13 at the node", lambda c: c.bracket["bound"], "{:.2f}"),
    C("bracket-req", "P §5.5", "the brief requires the bracket to carry 17.5 kN", lambda c: c.bracket["required_kN"]),
    C("bracket-angle", "P §5.5", "bound: the final front’s lower-front leg runs 72 degrees from its pivot axis, so 2.7 kN acts along the bolt", lambda c: c.bracket["angle"], "{:.0f}"),
    C("bracket-along", "P §5.5", "the final front’s lower-front leg runs 72 degrees from its pivot axis, so 2.7 kN acts along the bolt, and", lambda c: c.bracket["along_kN"]),
    C("bracket-inplane", "P §5.5", "with the in-plane 8.3 kN taken as direct stress", lambda c: c.bracket["inplane_kN"]),
    C("bracket-resolved", "P §5.5", "the node bracket reaches 4.16", lambda c: c.bracket["resolved"], "{:.2f}"),
    C("tube-mass", "P §5.5", "Per corner tube mass is 1053 grams front (896", lambda c: c.packaging["F"]["mass_g"], "{:.0f}"),
    C("tube-mass-bare", "P §5.5", "Per corner tube mass is 1053 grams front (896", lambda c: c.packaging["F"]["mass_bare_g"], "{:.0f}"),
    C("tube-mass-R", "P §5.5", "and 894 grams rear", lambda c: c.packaging["R"]["mass_g"], "{:.0f}"),
    C("tube-mass-delta", "P §5.5", "for 157 grams more tube than", lambda c: c.packaging["F"]["mass_g"] - c.packaging["F"]["mass_bare_g"], "{:.0f}"),
    C("ot-tr-F", "P §5.5", "transmission angle stays between 37.9 and 124.0 degrees front 54.4 and 114.2 degrees rear to 50 mm", lambda c: c.actuation["F"]["ot_tr"][0]),
    C("ot-tr-F2", "P §5.5", "Over-travel: the transmission angle stays between 37.9 and 124.0 degrees front 54.4 and 114.2 degrees rear to", lambda c: c.actuation["F"]["ot_tr"][1]),
    C("ot-tr-R", "P §5.5", "the transmission angle stays between 37.9 and 124.0 degrees front 54.4 and 114.2 degrees rear to 50", lambda c: c.actuation["R"]["ot_tr"][0]),
    C("ot-mr", "P §5.5", "the front motion ratio falls from 0.81 to 0.58", lambda c: c.actuation["F"]["ot_mr"][1], "{:.2f}"),
    # ---- compliance on the derived frame (Table S6.36; paper section 5.4) ----------------------------------------- #
    C("K_T", "P §2", "requires. With the rear pickups held and the front twisted the frame gives 2.88 kNm/deg", lambda c: c.frame["torsion"]["v11_bays_Nm_per_deg"] / 1000, "{:.2f}"),
    C("ctoe-corner", "P §5.4", "in the 1.5g corner to -0.012 degrees, a slight toe-in, on the corner’s own flexibility (14% of the band; -0.020 degrees from the links",
      lambda c: c.compliance_table[("F", "1.5g corner")]["derived"]["toe"], "{:.3f}"),
    C("ctoe-band", "P §5.4", "(14% of the band;", lambda c: 100 * c.compliance_table[("F", "1.5g corner")]["derived"]["toe"] / 0.08, "{:.0f}"),
    C("ctoe-links", "P §5.4", "-0.020 degrees from the links alone", lambda c: c.compliance_table[("F", "1.5g corner")]["links"]["toe"], "{:.3f}"),
    C("ctoe-frame", "P §9.1", "is +0.015 degrees, inside a fifth of the 0.08 degree band. A",
      lambda c: c.compliance_table[("F", "1.5g corner")]["frame_part"]["toe"], "{:.3f}"),
    C("ctoe-whole", "P §5.4", "-0.005 degrees with all four corners", lambda c: c.compliance_table[("F", "1.5g corner")]["whole"]["toe"], "{:.3f}"),
    C("rear-camber", "P §5.4", "At the rear, camber in the 1.5g corner is 0.123 degrees (41% of its band)",
      lambda c: c.compliance_table[("R", "1.5g corner")]["derived"]["camber"], "{:.3f}"),
    C("brake-toe", "P §5.4", "stop to 0.23 degrees per wheel (0.19 degrees on the whole car), with 1.0 degrees of caster lost,",
      lambda c: c.compliance_table[("F", "1.5g braking")]["derived"]["toe"], "{:.2f}"),
    C("brake-caster", "P §5.4", "1.5g stop to 0.23 degrees per wheel (0.19 degrees on the whole car), with 1.0 degrees of caster lost",
      lambda c: -c.compliance_table[("F", "1.5g braking")]["derived"]["caster"], "{:.1f}"),
    C("brake-links", "S D3", "+0.185 degrees from the links alone),",
      lambda c: c.compliance_table[("F", "1.5g braking")]["links"]["toe"], "{:.3f}", doc="S"),
    C("brake-whole", "P §5.4", "1.5g stop to 0.23 degrees per wheel (0.19 degrees on the whole car), with 1.0 degrees of caster lost",
      lambda c: c.compliance_table[("F", "1.5g braking")]["whole"]["toe"], "{:.2f}"),
    C("comb-toe", "P §5.4", "cornering puts 0.17 degrees of toe-out on the loaded outer wheel",
      lambda c: c.compliance_table[("F", "combined 1.06g")]["derived"]["toe"], "{:.2f}"),
    C("launch-toe", "P §5.4", "1.06g launch toes each rear wheel in 0.164 degrees and",
      lambda c: c.compliance_table[("R", "1.06g traction")]["derived"]["toe"], "{:.3f}"),
    C("exit-toe", "P §5.4", "a traction and cornering exit 0.146 degrees (0.061 degrees from the links)",
      lambda c: c.compliance_table[("R", "combined 1.06g traction")]["derived"]["toe"], "{:.3f}"),
    C("abstract-brake", "P Table 2", "axis, every curve unchanged: 0.23 degrees", lambda c: c.compliance_table[("F", "1.5g braking")]["derived"]["toe"], "{:.2f}"),
    C("row12", "P Table 10", "2.88 kNm/deg (upper bound); +0.015 degrees per front wheel at 1.5g Frame",
      lambda c: c.compliance_table[("F", "1.5g corner")]["frame_part"]["toe"], "{:.3f}"),
    C("concl-ctoe", "P §10", "toe -0.012 degrees in the 1.5g corner (14% of the band)",
      lambda c: c.compliance_table[("F", "1.5g corner")]["derived"]["toe"], "{:.3f}"),
    C("concl-link", "P §10", "link factor 2.85 on ⅝ x 0.065 inch and ⅝ x 0.049 inch legs, bearings 2.44",
      WF("link_fos_min"), "{:.2f}"),
    # ---- the delivered and released fronts on the corrected statics (pushrod on the lower wishbone) ------------------- #
    C("deliv-LR-kN", "P §5.5", "they put 10.06 kN of compression into the ⅝ x 0.035 inch lower-rear leg (buckling 1.22; yield 0.07 with",
      lambda c: -c.earlier["d73"]["LR"]["worst_n"] / 1000, "{:.2f}"),
    C("deliv-LR-b", "P §5.5", "(buckling 1.22; yield 0.07 with the bending", lambda c: c.earlier["d73"]["LR"]["fos_buckling"], "{:.2f}"),
    C("deliv-LR-y", "P §5.5", "(buckling 1.22; yield 0.07 with the bending", lambda c: c.earlier["d73"]["LR"]["fos_yield"], "{:.2f}"),
    C("deliv-LF-kN", "P §5.5", "and 10.51 kN of tension into the lower-front leg (yield 0.32)", lambda c: c.earlier["d73"]["LF"]["worst_n"] / 1000, "{:.2f}"),
    C("deliv-LF-y", "P §5.5", "and 10.51 kN of tension into the lower-front leg (yield 0.32)", lambda c: c.earlier["d73"]["LF"]["fos_yield"], "{:.2f}"),
    C("deliv-LR-b60", "P §5.5", "against buckling 1.48 at the declared 60% bias", lambda c: c.earlier["d60"]["LR"]["fos_buckling"], "{:.2f}"),
    C("released-29", "P §1", "a buckling factor of 0.81 and 0.88", lambda c: c.earlier["front_v11_29deg"]["LR"]["fos_buckling"], "{:.2f}"),
    C("released-25", "P §1", "a buckling factor of 0.81 and 0.88", lambda c: c.earlier["front_v11_25deg"]["LR"]["fos_buckling"], "{:.2f}"),
    C("released-29-kN", "P §1", "which puts 16.5 and 14.2 kN into the lower-rear leg", lambda c: -c.earlier["front_v11_29deg"]["LR"]["worst_n"] / 1000),
    C("released-25-kN", "P §1", "which puts 16.5 and 14.2 kN into the lower-rear leg", lambda c: -c.earlier["front_v11_25deg"]["LR"]["worst_n"] / 1000),
    C("tube035-now", "S D4", "(-17.3 kN and 0.77 on the corrected statics)", lambda c: c.earlier["tube035_LR"]["fos_buckling"], "{:.2f}", doc="S"),
    C("tube035-rec", "S D4", "lower rear leg-16.0 kN, buckling 0.83 on ⅝ x 0.035 inch", lambda c: float(re.search(
        r"buckling factor ([0-9.]+)", manifest("front_v11_29deg_b73_tube035").context["binding_wall"]).group(1)), "{:.2f}", doc="S"),
    C("b6001-rec", "S D3", "2x b001 rocker bearings at 0.89 against 2.0", lambda c: float(re.search(
        r"rocker_bearing_sf = ([0-9.]+)", manifest("front_v11_29deg_b73_6001").context["binding_wall"]).group(1)), "{:.2f}", doc="S"),
    C("b6001-now", "P §1", "0.58 on the 29.4 degrees candidate, front_v11_29deg_b73_6001)", lambda c: c.earlier["front_v11_29deg_b73_6001"]["rocker_bearing_sf"], "{:.2f}"),
    C("deliv-bracket", "P §5.5", "plates) reach 0.31 at the 10.5 kN stop and, moved to its node at 13 mm reach with 50 x 6.35 mm plates, 1.77;",
      lambda c: c.earlier["bracket_built"], "{:.2f}"),
    C("deliv-bracket-node", "P §5.5", "plates) reach 0.31 at the 10.5 kN stop and, moved to its node at 13 mm reach with 50 x 6.35 mm plates, 1.77; the",
      lambda c: c.earlier["bracket_node"], "{:.2f}"),
    C("concl-deliv", "P §10", "lower-rear leg (buckling 1.22; yield 0.07 with the bending the pushrod adds",
      lambda c: c.earlier["d73"]["LR"]["fos_yield"], "{:.2f}"),
    # ---- the re-run on the corrected code (paper section 6.1, supplement S13.3) ------------------------------------- #
    C("refused-front_v11_29deg_walls-link", "S S13.3", "link factor 0.18, 0.34, 0.24 and 0.13", _refused("front_v11_29deg_walls", "link_fos_min"), "{:.2f}", doc="S"),
    C("refused-front_v11_29deg_walls-bearing", "S S13.3", "rocker bearings 1.58, 1.97, 1.61 and 1.94", _refused("front_v11_29deg_walls", "rocker_bearing_sf"), "{:.2f}", doc="S"),
    C("refused-front_v11_25deg_walls-link", "S S13.3", "link factor 0.18, 0.34, 0.24 and 0.13", _refused("front_v11_25deg_walls", "link_fos_min"), "{:.2f}", doc="S"),
    C("refused-front_v11_25deg_walls-bearing", "S S13.3", "rocker bearings 1.58, 1.97, 1.61 and 1.94", _refused("front_v11_25deg_walls", "rocker_bearing_sf"), "{:.2f}", doc="S"),
    C("refused-front_v11_hairpin_walls-link", "S S13.3", "link factor 0.18, 0.34, 0.24 and 0.13", _refused("front_v11_hairpin_walls", "link_fos_min"), "{:.2f}", doc="S"),
    C("refused-front_v11_hairpin_walls-bearing", "S S13.3", "rocker bearings 1.58, 1.97, 1.61 and 1.94", _refused("front_v11_hairpin_walls", "rocker_bearing_sf"), "{:.2f}", doc="S"),
    C("refused-rear_v11_walls-link", "S S13.3", "link factor 0.18, 0.34, 0.24 and 0.13", _refused("rear_v11_walls", "link_fos_min"), "{:.2f}", doc="S"),
    C("refused-rear_v11_walls-bearing", "S S13.3", "rocker bearings 1.58, 1.97, 1.61 and 1.94", _refused("rear_v11_walls", "rocker_bearing_sf"), "{:.2f}", doc="S"),
    C("brief-hash", "P §9.4", "chassisv11_node_brief.json (inputs SHA-256 4f1c95fff3ab)", _brief_hash, "{:012x}"),
    C("S6.36-F-1.5g_corner-k30-toe", "S Table S6.36", "front 1.5g corner -0.032/+0.029 -0.020/+0.014 -0.012/+0.100 -0.005/-0.043", lambda c: c.compliance_table[("F", "1.5g corner")]["k30"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-1.5g_corner-k30-camber", "S Table S6.36", "front 1.5g corner -0.032/+0.029 -0.020/+0.014 -0.012/+0.100 -0.005/-0.043", lambda c: c.compliance_table[("F", "1.5g corner")]["k30"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-1.5g_corner-derived-toe", "S Table S6.36", "front 1.5g corner -0.032/+0.029 -0.020/+0.014 -0.012/+0.100 -0.005/-0.043", lambda c: c.compliance_table[("F", "1.5g corner")]["derived"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-1.5g_corner-derived-camber", "S Table S6.36", "front 1.5g corner -0.032/+0.029 -0.020/+0.014 -0.012/+0.100 -0.005/-0.043", lambda c: c.compliance_table[("F", "1.5g corner")]["derived"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-1.5g_corner-whole-toe", "S Table S6.36", "front 1.5g corner -0.032/+0.029 -0.020/+0.014 -0.012/+0.100 -0.005/-0.043", lambda c: c.compliance_table[("F", "1.5g corner")]["whole"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-1.5g_corner-whole-camber", "S Table S6.36", "front 1.5g corner -0.032/+0.029 -0.020/+0.014 -0.012/+0.100 -0.005/-0.043", lambda c: c.compliance_table[("F", "1.5g corner")]["whole"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-1.5g_braking-derived-toe", "S Table S6.36", "front 1.5g braking +0.353/-0.013 +0.185/-0.012 +0.230/-0.068 +0.194/-0.118", lambda c: c.compliance_table[("F", "1.5g braking")]["derived"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-1.5g_braking-derived-camber", "S Table S6.36", "front 1.5g braking +0.353/-0.013 +0.185/-0.012 +0.230/-0.068 +0.194/-0.118", lambda c: c.compliance_table[("F", "1.5g braking")]["derived"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-1.5g_braking-whole-toe", "S Table S6.36", "front 1.5g braking +0.353/-0.013 +0.185/-0.012 +0.230/-0.068 +0.194/-0.118", lambda c: c.compliance_table[("F", "1.5g braking")]["whole"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-1.5g_braking-whole-camber", "S Table S6.36", "front 1.5g braking +0.353/-0.013 +0.185/-0.012 +0.230/-0.068 +0.194/-0.118", lambda c: c.compliance_table[("F", "1.5g braking")]["whole"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-combined_1.06g-derived-toe", "S Table S6.36", "front combined 1.06g +0.245/+0.018 +0.124/+0.004 +0.172/+0.045 +0.160/-0.067", lambda c: c.compliance_table[("F", "combined 1.06g")]["derived"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-F-combined_1.06g-derived-camber", "S Table S6.36", "front combined 1.06g +0.245/+0.018 +0.124/+0.004 +0.172/+0.045 +0.160/-0.067", lambda c: c.compliance_table[("F", "combined 1.06g")]["derived"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-R-1.5g_corner-derived-toe", "S Table S6.36", "rear 1.5g corner -0.031/+0.085 -0.015/+0.047 -0.026/+0.123 +0.009/+0.075", lambda c: c.compliance_table[("R", "1.5g corner")]["derived"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-R-1.5g_corner-derived-camber", "S Table S6.36", "rear 1.5g corner -0.031/+0.085 -0.015/+0.047 -0.026/+0.123 +0.009/+0.075", lambda c: c.compliance_table[("R", "1.5g corner")]["derived"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-R-1.5g_corner-whole-toe", "S Table S6.36", "rear 1.5g corner -0.031/+0.085 -0.015/+0.047 -0.026/+0.123 +0.009/+0.075", lambda c: c.compliance_table[("R", "1.5g corner")]["whole"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-R-1.5g_corner-whole-camber", "S Table S6.36", "rear 1.5g corner -0.031/+0.085 -0.015/+0.047 -0.026/+0.123 +0.009/+0.075", lambda c: c.compliance_table[("R", "1.5g corner")]["whole"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-R-1.06g_traction-derived-toe", "S Table S6.36", "rear 1.06g traction -0.144/-0.055 -0.067/-0.030 -0.164/-0.070", lambda c: c.compliance_table[("R", "1.06g traction")]["derived"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-R-1.06g_traction-derived-camber", "S Table S6.36", "rear 1.06g traction -0.144/-0.055 -0.067/-0.030 -0.164/-0.070", lambda c: c.compliance_table[("R", "1.06g traction")]["derived"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-R-combined_1.06g_traction-whole-toe", "S Table S6.36", "rear combined 1.06g traction -0.131/+0.059 -0.061/+0.034 -0.146/+0.072 -0.079/+0.039", lambda c: c.compliance_table[("R", "combined 1.06g traction")]["whole"]["toe"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.36-R-combined_1.06g_traction-whole-camber", "S Table S6.36", "rear combined 1.06g traction -0.131/+0.059 -0.061/+0.034 -0.146/+0.072 -0.079/+0.039", lambda c: c.compliance_table[("R", "combined 1.06g traction")]["whole"]["camber"] + 0.0, "{:+.3f}", doc="S"),
    C("S6.37-front-tilt", "S Table S6.37", "front          14                       0.804; 7.7%", AF("tilt"), "{:.0f}", doc=""),
    C("S6.37-front-clear-tubes", "S Table S6.37", "9.5 / 11.7", lambda c: c.packaging["F"]["clear_tubes"], "{:.1f}", doc="S"),
    C("S6.37-rear-clear-tubes", "S Table S6.37", "5.1 / 70.2", lambda c: c.packaging["R"]["clear_tubes"], "{:.1f}", doc="S"),
    C("damper-eye", "S S7.3", "static eye to eye 176.8 mm front and 177.0 mm rear", lambda c: c.damper_front["eye"], "{:.1f}", doc="S"),
    # ---- the curves-only baseline judged against front_v11_final's walls (paper section 6.3, Table S13.2) -------- #
    C("baseline-winner-link", "P §6.3", "winner RESILIENT, 100% 0.349 0 Ackermann, lock, link strength, rocker bearings, braking compliance toe +0.9 69.7 0.27", lambda c: c.baseline["winner"]["link_fos_min"], "{:.2f}"),
    C("baseline-winner-btoe", "P §6.3", "a 0.07 link factor and 0.49 degrees of braking toe", lambda c: c.baseline["winner"]["compliance_toe_braking_deg"], "{:.2f}"),
    C("baseline-link-lo", "P §6.3", "start 1 RESILIENT, 97% 0.928 109 lock sweep does not solve, wheel envelope -81.8 84.1 0.02", lambda c: min(c.baseline["others_link"]), "{:.2f}"),
    C("baseline-link-hi", "P §6.3", "swept through tire 395.6 0.43", lambda c: max(c.baseline["others_link"]), "{:.2f}"),
    C("baseline-lock", "P §6.3", "front at 137 degrees of wheel for the hairpin and -1.9% Ackermann", lambda c: c.baseline["winner"]["lock_sw_deg"], "{:.0f}"),
    C("baseline-ad", "P §6.3", "carry 8 or 396% anti-dive", lambda c: max(c.baseline["others_ad"]), "{:.0f}"),
    C("baseline-bearing", "S S13.2", "the bearing wall (1.998 against 2.0)", lambda c: c.baseline["winner"]["rocker_bearing_sf"], "{:.3f}", doc="S"),
]

# ---- the s7 paper keeps the argument (problem, method, three results, limits); the detail behind it, with these claims,
# moved verbatim to supplement Part M, whose sections and tables carry an M (section 5.5 -> M5.5, Table 8 -> Table M8)
_PART_M = [(re.compile(r"\bTables (\d+) and (\d+)"), r"Tables M\1 and M\2"),
           (re.compile(r"\bTable (\d+)\b(?!\.\d)"), r"Table M\1"),
           (re.compile(r"§(\d+(?:\.\d+)?)"), r"M\1"),
           (re.compile(r"\b[Ss]upplement Table (?=[SD]\d)"), "Table "),
           (re.compile(r"\b[Ss]upplement (?=[SD]\d)"), ""),
           (re.compile(r"\bFigure (\d)\b"), r"paper Figure \1")]


def to_part_m(s: str) -> str:
    for rx, rep in _PART_M:
        s = rx.sub(rep, s)
    return s


MOVED_TO_PART_M = {
    "C0-30-F", "C0-30-R", "C0-60-F", "C0-60-R", "K_T", "ackermann", "anti-dive-60b", "anti-dive-73", "anti-lift-in",
    "anti-lift-out", "anti-squat", "b6001-F", "b6001-R", "b6001-now", "baseline-ad", "baseline-link-hi", "baseline-link-lo",
    "baseline-lock", "baseline-winner-btoe", "baseline-winner-link", "bear-F", "bracket-along", "bracket-angle",
    "bracket-bound", "bracket-inplane", "bracket-load", "bracket-reach", "bracket-reach-R", "bracket-req",
    "bracket-resolved", "brake-caster", "brake-toe", "brake-whole", "bump-steer-front", "bump-steer-rear",
    "camber-lock-in", "camber-lock-out", "clearance-F", "clearance-R", "comb-toe", "concl-ctoe", "concl-deliv",
    "concl-link", "corner-load", "ctoe-band", "ctoe-corner", "ctoe-frame", "ctoe-links", "ctoe-whole", "decel",
    "deliv-LF-kN", "deliv-LF-y", "deliv-LR-b", "deliv-LR-b60", "deliv-LR-kN", "deliv-LR-y", "deliv-bracket",
    "deliv-bracket-node", "dive", "effort-margin", "exit-toe", "eye-text", "final-front-verdict", "final-front-walls",
    "final-rear-fit", "first-yield-F", "first-yield-F-bare", "first-yield-R", "first-yield-R-UR", "floor-brake",
    "floor-comb-F", "floor-rear", "floor-rear-c", "front-base", "front-bearing-s1", "front-bottoms", "front-btoe-52",
    "front-btoe-row9", "front-btoe-t6", "front-clear", "front-ctoe-t6", "front-eye-t7", "front-eye-t7-b", "front-eye-t7-d",
    "front-link-s1", "front-link-t6", "front-move", "front-mr", "front-mr-t7", "front-node-t7", "front-oop",
    "front-ride-hi", "front-ride-lo", "front-spread-t7", "front-spring-t7", "front-stroke-t7", "front-topout",
    "front-tr-hi", "front-tr-lo", "gain-front", "gain-rear", "hub-F", "hub-R", "inner", "launch-toe", "lock-sw",
    "migration-share", "misalign", "mount-reach", "mount-reach-R", "mu-front", "mu-rear", "ot-mr", "ot-tr-F",
    "ot-tr-F2", "ot-tr-R", "outer", "pitch", "pitch18", "ratio-avg", "ratio-centre", "rc-front-rear", "rear-bearing",
    "rear-bottoms", "rear-camber", "rear-clear-t7", "rear-eye-t7", "rear-eye-t7-b", "rear-eye-t7-d", "rear-link",
    "rear-mr", "rear-node-t7", "rear-spread", "rear-spring-t7", "rear-stroke", "rear-topout", "released-25",
    "released-25-kN", "released-29", "released-29-kN", "ride-derived-F", "ride-derived-R", "rise", "rocker-tilt",
    "rocker-tilt-R", "roll-change-F", "roll-change-R", "row12", "setup-spring", "share", "spring-F-lbin", "spring-cat-F-hz",
    "spring-cat-R-N", "spring-cat-R-hz", "spring-cat-bar", "swing", "t8-LF-load", "t8-LF-yield-s", "t8-LR-buck",
    "t8-LR-load", "t8-LR-yield", "t8-bare-LF", "t8-bare-LR-b", "t8-bare-LR-y", "t8-others", "t8-pushrod", "tube-mass",
    "tube-mass-R", "tube-mass-bare", "tube-mass-delta", "twist-618", "twist-one-flat", "twist-toe-per-mm",
    "unload", "warp-pair", "warp-patch", "warp-row13", "warp-spring-in", "warp-spring-out"}
for _c in CLAIMS:
    if _c.id in MOVED_TO_PART_M:
        _c.quote, _c.doc, _c.where = to_part_m(_c.quote), "S", "S " + to_part_m(_c.where[2:])

# ---- the s7 paper's own text and tables -------------------------------------------------------------------------- #
def _superseded_front():
    import json
    from paper_check import ROOT
    d = json.loads((ROOT / "manifests" / "superseded" / "front_v11_final.cea1a7c48229.recorded.genesis.json").read_text())
    return GenesisManifest.from_json(json.dumps(d))


def _superseded_spring(c):
    from suspension import actuation as act
    m = _superseded_front()
    return act.actuation_metrics(hp_from_dict(m.recorded["winner_hardpoints"])).spring_length_mm[1]


def _superseded_braking_toe(c):
    """The earlier declaration's braking toe: its own frame-derived flexibility and links, the 1.5 g stop at 73%."""
    from suspension import elastokinematics as ek
    from suspension.genesis_repro import vehicle_load_cases
    m = _superseded_front()
    el = m.objects()[2].properties.elasto
    cs = [x for x in vehicle_load_cases(axle="front", brake_bias_front=0.73) if x.name == "1.5g braking"][0]
    spec = ek.ElastoSpec(stiffness=el.stiffness, members=dict(el.members), Fx=cs.Fx, Fy=cs.Fy, Fz=cs.Fz, Mz=cs.mz_Nmm,
                         Fx_wc=getattr(cs, "Fx_wc", 0.0))
    return ek.solve_elastokinematic(hp_from_dict(m.recorded["winner_hardpoints"]), spec, jacobian=False).change["toe"]


def _decel60(c):
    return c.vF.properties.brake.decel_g(0.60)


def _rear_final_antilift60(c):
    from suspension.kinematics import SuspensionKinematics
    return SuspensionKinematics(winner("rear_final")).anti_lift_pct(280.0, 1630.0, brake_bias_rear=0.40)


S7 = [
    C("s7-decel60", "P Table 2", "stops at 1.27g, limited by the rear tires", _decel60, "{:.2f}"),
    C("s7-decel73", "P Table 2", "(wall 1.5g) 73%; 1.57g", WF("brake_decel_g"), "{:.2f}"),
    C("s7-antilift", "P Table 2", "anti-lift −121% at 60% bias: the rear extends 30 mm", _rear_final_antilift60, "{:.0f}"),
    C("s7-antilift-in", "P Table 2", "inboard rear brakes: +12.6% at 73%", R("al_in")),
    C("s7-released-29", "P Table 2", "buckling factor 0.81 and 0.88 in the 1.8g stop ⅝",
      lambda c: c.earlier["front_v11_29deg"]["LR"]["fos_buckling"], "{:.2f}"),
    C("s7-released-25", "P Table 2", "buckling factor 0.81 and 0.88 in the 1.8g stop ⅝",
      lambda c: c.earlier["front_v11_25deg"]["LR"]["fos_buckling"], "{:.2f}"),
    C("s7-link", "P Table 2", "⅝ x 0.065 and ⅝ x 0.049 inch legs on a 167 mm base: 2.85", WF("link_fos_min"), "{:.2f}"),
    C("s7-base", "P Table 2", "stop ⅝ x 0.065 and ⅝ x 0.049 inch legs on a 167 mm base: 2.85", lambda c: _base(c.hF), "{:.0f}"),
    C("s7-6001-F", "P Table 2", "static safety factor 0.88 front, 0.63 rear", lambda c: c.bearings["F"]["s0_6001"], "{:.2f}"),
    C("s7-6001-R", "P Table 2", "static safety factor 0.88 front, 0.63 rear", lambda c: c.bearings["R"]["s0_6001"], "{:.2f}"),
    C("s7-6204", "P Table 2", "2.44 and 2.08", WF("rocker_bearing_sf"), "{:.2f}"),
    C("s7-6205", "P Table 2", "2.44 and 2.08", WR("rocker_bearing_sf"), "{:.2f}"),
    C("s7-wishbone-yield", "P Table 2", "(yield factor 0.07)", lambda c: c.earlier["d73"]["LR"]["fos_yield"], "{:.2f}"),
    C("s7-spring-old", "P Table 2", "106.7 mm eye to eye at ride height", _superseded_spring, "{:.1f}"),
    C("s7-spring", "P Table 2", "mount on the damper axis: 176.8 mm, 14.2 mm of stroke left", lambda c: c.actuation["F"]["spring_mm"][1]),
    C("s7-stroke-left", "P Table 2", "mount on the damper axis: 176.8 mm, 14.2 mm of stroke left",
      lambda c: c.actuation["F"]["spring_mm"][2] - (DAMPER_MM - 57.0)),
    C("s7-btoe-old", "P Table 2", "base 0.67 degree of toe-out per wheel in a 1.5g stop", _superseded_braking_toe, "{:.2f}"),
    C("s7-rear-v11", "P Table 2", "0.25 mm from the tire (wall 3 mm)", _env("rear_v11"), "{:.2f}"),
    C("s7-rear-clear", "P Table 2", "+1.19 mm beyond the wall", lambda c: c.walls["R_env"], "{:.2f}"),
    C("s7-deliv-lock", "P Table 2", "137 degrees of steering wheel to the hairpin, -1.9% Ackermann",
      lambda c: c.baseline["winner"]["lock_sw_deg"], "{:.0f}"),
    C("s7-deliv-ack", "P Table 2", "137 degrees of steering wheel to the hairpin, -1.9% Ackermann (81",
      lambda c: c.baseline["winner"]["ackermann_lock_pct"]),
    C("s7-final-lock", "P Table 2", "118.9 degrees, 75.3%", ST("lock_sw")),
    C("s7-final-ack", "P Table 2", "118.9 degrees, 75.3%", ST("ack")),
    C("s7-caster", "P §2", "fix it at +3.66 degrees", WF("caster_deg"), "{:.2f}"),
    C("s7-t4-fit", "P Table 4", "0.347; 100% (78.0% at hand weld)", _fit("front_v11_final"), "{:.3f}"),
    C("s7-t4-fit-R", "P Table 4", "0.41; 100% (93.2% at hand weld)", _fit("rear_v11_final"), "{:.2f}"),
    C("s7-t4-clear", "P Table 4", "+0.49 mm", lambda c: c.walls["F_env"], "{:.2f}"),
    C("s7-t4-hairpin", "P Table 4", "118.9 degrees; 75.3%", ST("lock_sw")),
    C("s7-t4-ctoe", "P Table 4", "-0.012 degrees; 0.23 degrees", WF("compliance_toe_deg"), "{:.3f}"),
    C("s7-t4-btoe", "P Table 4", "-0.012 degrees; 0.23 degrees", WF("compliance_toe_braking_deg"), "{:.2f}"),
    C("s7-t4-camber", "P Table 4", "degrees camber 0.123 degrees (0.30 degrees)", lambda c: c.compliance_table[("R", "1.5g corner")]["derived"]["camber"], "{:.3f}"),
    C("s7-t4-link", "P Table 4", "2.85; 2.44", WF("link_fos_min"), "{:.2f}"),
    C("s7-t4-link-R", "P Table 4", "2.85; 2.44 4.63; 2.08", WR("link_fos_min"), "{:.2f}"),
    C("s7-t4-mr", "P Table 4", "0.804; 176.8 mm", AF("mr"), "{:.3f}"),
    C("s7-t4-mr-R", "P Table 4", "0.821; 177.0 mm", AR("mr"), "{:.3f}"),
    C("s7-t4-eye-R", "P Table 4", "0.821; 177.0 mm", lambda c: c.actuation["R"]["spring_mm"][1]),
    C("s7-mu", "P §4", "inside the 1.83 to which the corners hold", lambda c: c.targets["mu_R"], "{:.2f}"),
]
CLAIMS += S7

# ---- the search from a failing start (paper section 3.2, Table 3; supplement S13.5) -------------------------------- #
def _from85(prop):
    """A property of front_v11_from_85mm's winner, or of its start (the 85 mm candidate under the same walls)."""
    def f(c):
        m = manifest("front_v11_from_85mm")
        vol = m.objects()[2]
        v = vol.properties.evaluate(winner("front_v11_from_85mm"))
        r, ok = m.objects()[1].residual(winner("front_v11_from_85mm"))
        v["band"] = float(abs(r).max()); v["env"] = vol.envelope_margin(winner("front_v11_from_85mm"))
        v["base"] = _base(winner("front_v11_from_85mm"))
        return v[prop]
    return f


def _start85(prop, src="front_v11_29deg", under="front_v11_from_85mm"):
    def f(c):
        import json
        from paper_check import ROOT
        F = json.loads((ROOT / "manifests" / "front_v11_final.recorded.genesis.json").read_text())
        hp = dict(manifest(src).recorded["winner_hardpoints"])
        for k in ("pushrod_outer", "rocker_pivot", "rocker_axis", "rocker_pushrod", "rocker_spring", "spring_inner",
                  "pushrod_attach"):
            hp[k] = F["hardpoints"][k]
        vol = manifest(under).objects()[2]
        return vol.properties.evaluate(hp_from_dict(hp))[prop]
    return f


S7 += [
    C("r2-start-link", "P §3.2", "its lower legs reach a factor of 1.37 where 2.0 is required", _start85("link_fos_min"), "{:.2f}"),
    C("r2-start-btoe", "P §3.2", "toes each wheel out 0.77 degrees against 0.25 degrees.", _start85("compliance_toe_braking_deg"), "{:.2f}"),
    C("r2-evals", "P §3.2", "moves the pickups in 432 evaluations",
      lambda c: _seedrep("front_v11_from_85mm")["evaluations"], "{:d}"),
    C("r2-band", "P §3.2", "with the curves inside 0.54 of their bands", _from85("band"), "{:.2f}"),
    C("r2-base", "P §3.2", "widening the lower base from 85 to 113 mm", _from85("base"), "{:.0f}"),
    C("r2-base0", "P §3.2", "widening the lower base from 85 to 113 mm", lambda c: _base(winner("front_v11_29deg")), "{:.0f}"),
    C("r2-hash", "P §3.2", "front_v11_from_85mm, 41c4fcdd9766", _hash("front_v11_from_85mm"), "{:012x}"),
    C("r2-yield", "P Table 3", "RESILIENT, 100%", lambda c: 100 * manifest("front_v11_from_85mm").recorded["winner_yield"], "{:.0f}"),
    C("r2-t-lock", "P Table 3", "116.7 degrees", _from85("lock_sw_deg")),
    C("r2-t-ack", "P Table 3", "86.1%", _from85("ackermann_lock_pct")),
    C("r2-t-btoe", "P Table 3", "0.247 degrees", _from85("compliance_toe_braking_deg"), "{:.3f}"),
    C("r2-t-ctoe", "P Table 3", "Toe in the 1.5g corner 0.08 degrees -0.017 degrees -0.005 degrees -0.010 degrees 0.006 degrees", _from85("compliance_toe_deg"), "{:.3f}"),
    C("r2-t-ad", "P Table 3", "68.4%", _from85("anti_dive_pct")),
    C("r2-t-link", "P Table 3", "2.12; 2.43", _from85("link_fos_min"), "{:.2f}"),
    C("r2-t-bear", "P Table 3", "2.12; 2.43", _from85("rocker_bearing_sf"), "{:.2f}"),
    C("r2-t-env", "P Table 3", "+2.21 mm", _from85("env"), "{:.2f}"),
    C("r2-t-s-lock", "P Table 3", "110.3 degrees", _start85("lock_sw_deg")),
    C("r2-t-s-ack", "P Table 3", "98.1%", _start85("ackermann_lock_pct")),
    C("r2-del-nofit", "P §3.2", "brings them only to 1.26 and ends with no winner (NO_FIT",
      lambda c: manifest("front_v11_from_delivered").recorded["candidates"][0]["max_band_frac"], "{:.2f}"),
    C("r2-del-stall", "P §3.2", "but the curves end at 1.30 of their bands",
      lambda c: _seedrep("front_v11_from_delivered")["final_band"], "{:.2f}"),
    C("r2-del-btoe", "P §3.2", "toes 0.48 degrees per wheel in a 1.5g stop with rigid joints",
      _start85("compliance_toe_braking_deg", "front_steering_walls", "front_v11_from_delivered"), "{:.2f}"),
]
CLAIMS += S7[len(S7) - 21:]


def _delbest(prop):
    """front_v11_from_delivered's best candidate (NO_FIT), rebuilt from its recorded shift."""
    def f(c):
        import numpy as np
        from suspension import inverse_genesis as ig
        m = manifest("front_v11_from_delivered")
        _, tg, vol, _ = m.objects()
        h = ig._shifted(m.search_start(), vol, np.asarray(m.recorded["candidates"][0]["shift_vec"], float))
        if prop == "camber_bump":
            return abs(float(tg.residual(h)[0][4]))
        if prop == "rc_droop":
            return abs(float(tg.residual(h)[0][10]))
        if prop == "base":
            return _base(h)
        if prop == "env":
            return vol.envelope_margin(h) + 0.0
        if prop == "walls":
            return float(len(vol.wall_violations(h)))
        return vol.properties.evaluate(h)[prop]
    return f


S8 = [
    C("r3-del-lock", "P Table 3", "117.0 degrees", _delbest("lock_sw_deg")),
    C("r3-del-ack", "P Table 3", "102.7%", _delbest("ackermann_lock_pct")),
    C("r3-del-btoe", "P Table 3", "0.240 degrees", _delbest("compliance_toe_braking_deg"), "{:.3f}"),
    C("r3-del-ctoe", "P Table 3", "Toe in the 1.5g corner 0.08 degrees -0.017 degrees -0.005 degrees -0.010 degrees 0.006 degrees", _delbest("compliance_toe_deg"), "{:.3f}"),
    C("r3-del-ad", "P Table 3", "66.3%", _delbest("anti_dive_pct")),
    C("r3-del-link", "P Table 3", "2.74; 2.40", _delbest("link_fos_min"), "{:.2f}"),
    C("r3-del-bear", "P Table 3", "2.74; 2.40", _delbest("rocker_bearing_sf"), "{:.2f}"),
    C("r3-del-base", "P Table 3", "171 mm", _delbest("base"), "{:.0f}"),
    C("r3-del-env", "P Table 3", "+0.00 mm", _delbest("env"), "{:+.2f}"),
    C("r3-del-band", "P Table 3", "1.26 (camber at full bump)", _delbest("camber_bump"), "{:.2f}"),
    C("r3-del-camber", "P §3.2", "camber at full bump misses its band by 26%", lambda c: 100 * (_delbest("camber_bump")(c) - 1), "{:.0f}"),
    C("r3-del-rc", "P §3.2", "roll-center height at full droop by 14%", lambda c: 100 * (_delbest("rc_droop")(c) - 1), "{:.0f}"),
    C("r3-del-evals", "P §3.2", "The pattern search reaches every wall in 964", lambda c: _seedrep("front_v11_from_delivered")["evaluations"], "{:d}"),
    C("r3-del-start-lock", "P Table 3", "137.1 degrees", _start85("lock_sw_deg", "front_steering_walls", "front_v11_from_delivered")),
    C("r3-del-start-ad", "P Table 3", "69.7%", _start85("anti_dive_pct", "front_steering_walls", "front_v11_from_delivered")),
    C("r3-del-start-link", "P Table 3", "2.07; 2.44", _start85("link_fos_min", "front_steering_walls", "front_v11_from_delivered"), "{:.2f}"),
]
CLAIMS += S8


# ---- the tire values that stay declared, on their published basis (paper section 4, supplement S6.38) ------------- #
def _camber0(axle, what):
    def f(c):
        from dataclasses import replace as _r
        from suspension import target_derivation as td
        V, T = td.Vehicle(), _r(td.Tire(), camber_opt_deg=0.0)
        g = c.corners[axle]["gain"]
        static = -2.5 if axle == "F" else -2.6
        if what == "static":
            return td.static_camber_needed(V, T, g)
        return abs(td.loaded_camber_error(V, T, static, g))
    return f


MRA_ROAD_FACTOR = 0.6          # [38]
S9 = [
    C("t-road-lo", "P §4", "at road 1.09 to 1.43", lambda c: MRA_ROAD_FACTOR * 1.82, "{:.2f}"),
    C("t-road-hi", "P §4", "at road 1.09 to 1.43", lambda c: MRA_ROAD_FACTOR * 2.38, "{:.2f}"),
    C("t-camber-F", "P §4", "about -0.7 degrees front and -0.8 degrees rear", _camber0("F", "static")),
    C("t-camber-R", "P §4", "about -0.7 degrees front and -0.8 degrees rear", _camber0("R", "static")),
    C("t-camber-cost-lo", "P §4", "give up 0.5 to 2.1% of peak grip", lambda c: 0.3 * _camber0("F", "error")(c)),
    C("t-camber-cost-hi", "P §4", "give up 0.5 to 2.1% of peak grip", lambda c: 1.2 * _camber0("F", "error")(c)),
    C("t-camber-S-F", "S S6.38", "-0.74 degrees front and -0.82 degrees", _camber0("F", "static"), "{:.2f}", doc="S"),
    C("t-camber-S-R", "S S6.38", "-0.74 degrees front and -0.82 degrees rear", _camber0("R", "static"), "{:.2f}", doc="S"),
    C("t-camber-err-F", "S S6.38", "1.76 and 1.78 deg", _camber0("F", "error"), "{:.2f}", doc="S"),
    C("t-camber-err-R", "S S6.38", "1.76 and 1.78 deg", _camber0("R", "error"), "{:.2f}", doc="S"),
]
CLAIMS += S9


def _deliv_6001(c):
    from suspension import actuation as act
    from suspension import genesis_repro as gr
    # the delivered front's own recorded actuation (pushrod on the lower wishbone); re-synthesising it moves the rocker
    return min(r["s0"] for r in act.rocker_bearing_loads(winner("front_steering_walls"), gr.vehicle_load_cases()))


S10 = [
    C("d-buck", "P §3.1", "reach a buckling factor of 1.22 in a 1.8g stop, and", lambda c: c.earlier["d73"]["LR"]["fos_buckling"], "{:.2f}"),
    C("d-yield", "P §3.1", "bent by the pushrod mounted on that wishbone, a yield factor of 0.07 in a curb strike while braking", lambda c: c.earlier["d73"]["LR"]["fos_yield"], "{:.2f}"),
    C("d-bracket", "P §3.1", "off its node, reaches a factor of 0.31", lambda c: c.earlier["bracket_built"], "{:.2f}"),
    C("d-6001", "P §3.1", "its 6001 rocker bearings reach a static factor of 0.72", _deliv_6001, "{:.2f}"),
    C("d-decel", "P §3.1", "the car stops at 1.27g,", _decel60, "{:.2f}"),
    C("d-lock", "P §3.1", "it needs 137 degrees of steering wheel to reach the hairpin on", lambda c: c.baseline["winner"]["lock_sw_deg"], "{:.0f}"),
    C("c-within-F", "P §2", "(within 0.07 degrees)", lambda c: abs(__import__("suspension.target_derivation", fromlist=["x"]).loaded_camber_error(
        __import__("suspension.target_derivation", fromlist=["x"]).Vehicle(), __import__("suspension.target_derivation", fromlist=["x"]).Tire(),
        -2.5, c.corners["F"]["gain"])), "{:.2f}"),
    C("c-pub-F", "P §2", "at -0.7 degrees and -0.8 degrees", _camber0("F", "static")),
    C("c-pub-R", "P §2", "at -0.7 degrees and -0.8 degrees", _camber0("R", "static")),
]
CLAIMS += S10


_SM = {}


def _sm(*path):
    """A value of scripts/steering_margin.run() (computed once): the effort margin and the alternative upright."""
    def f(c):
        if not _SM:
            import steering_margin
            _SM.update(steering_margin.run())
        v = _SM
        for k in path:
            v = v[k]
        return v
    return f


S11 = [
    C("sm-decl", "P §3.3", "carries margin on every road-level tire: 8.1 Nm at", _sm("margin", "levels", "declared", "hand_brush_Nm")),
    C("sm-road-lo", "P §3.3", "and 7.8 to 8.4 on the R25B shape", _sm("margin", "levels", "r25b_road_1p43", "hand_brush_Nm")),
    C("sm-road-hi", "P §3.3", "and 7.8 to 8.4 on the R25B shape", _sm("margin", "levels", "r25b_1p55", "hand_brush_Nm")),
    C("sm-lim", "P §3.3", "pneumatic trail of 22.4 mm (20.5 to 24.0 mm)", _sm("margin", "levels", "declared", "t0_limit_mm")),
    C("sm-lim-lo", "P §3.3", "pneumatic trail of 22.4 mm (20.5 to 24.0 mm)", _sm("margin", "levels", "r25b_1p55", "t0_limit_mm")),
    C("sm-lim-hi", "P §3.3", "pneumatic trail of 22.4 mm (20.5 to 24.0 mm)", _sm("margin", "levels", "r25b_road_1p43", "t0_limit_mm")),
    C("sm-belt", "P §3.3", "does the margin shrink to 9.9 against 10", _sm("margin", "levels", "r25b_car_tire_belt", "hand_brush_Nm")),
    C("sm-25", "P §3.3", "the hand torque would be 10.7 Nm.", _sm("margin", "levels", "declared", "hand_25p4_Nm")),
    C("sm-dx", "P §3.3", "the spindle moved 2.0 mm forward relative to the kingpin axis", lambda c: -_sm("contingency", "dx_mm")(c)),
    C("sm-t0", "P §3.3", "trail from 12.7 to 10.7 mm", _sm("contingency", "trail_base_mm")),
    C("sm-t1", "P §3.3", "trail from 12.7 to 10.7 mm", _sm("contingency", "trail_contingency_mm")),
    C("sm-env", "P §3.3", "(wheel clearance +0.14 mm beyond the wall", _sm("contingency", "contingency", "env"), "{:+.2f}"),
    C("sm-link", "P §3.3", "link factor 2.87, braking toe", _sm("contingency", "contingency", "link_fos_min"), "{:.2f}"),
    C("sm-btoe", "P §3.3", "braking toe 0.23 degrees, curves", _sm("contingency", "contingency", "compliance_toe_braking_deg"), "{:.2f}"),
    C("sm-band", "P §3.3", "curves at 0.35 of their bands", _sm("contingency", "contingency", "band"), "{:.2f}"),
    C("sm-c-decl", "P §3.3", "gives 9.9 Nm on the declared set and 9.6 at road 1.43. The",
      _sm("contingency", "hand_25p4_Nm", "declared")),
    C("sm-c-road", "P §3.3", "gives 9.9 Nm on the declared set and 9.6 at road 1.43. The",
      _sm("contingency", "hand_25p4_Nm", "r25b_road_1p43")),
    C("sm-lock", "P §3.3", "takes 118.9 degrees of the 120 degree wheel-angle wall, so", _sm("contingency", "contingency", "lock_sw_deg")),
    C("sm-r-dx", "P §3.3", "the 3.1 mm offset that the R25B shape", lambda c: -_sm("r25b_1p55_offset", "dx_mm")(c)),
    C("sm-r-env", "P §3.3", "breaks the clearance wall by 0.30 mm", _sm("r25b_1p55_offset", "env"), "{:.2f}"),
]
CLAIMS += S11


# ---- joints and tabs, full lock, full-solve yield (paper section 3.3, supplement S13.6) ---------------------------- #
_RV = {}


def _rv(part, *path):
    """A value of scripts/review_checks.py (joints, lock) or of the 2000-build full-solve yield check, computed once."""
    def f(c):
        if part not in _RV:
            import review_checks as rcx
            import yield_full_check as yfc
            _RV[part] = {"joints": rcx.joints, "lock": rcx.lock,
                         "yield": lambda: {n: yfc.check(n, 2000) for n in ("front_v11_final", "rear_v11_final")}}[part]()
        v = _RV[part]
        for k in path:
            v = v[k]
        return v
    return f


def _pct(f):
    return lambda c: 100.0 * f(c)


_JB = lambda k: _rv("joints", "sweep", "braking", "toe_deg", str(k))       # noqa: E731
_JC = lambda k: _rv("joints", "sweep", "corner", "toe_deg", str(k))        # noqa: E731
_YH = lambda n, k: _rv("yield", n, "fields", "hand_weld", k)               # noqa: E731
S12 = [
    C("j-t4-c", "P Table 4", "-0.027 degrees; 0.43 degrees (broken)", _JC(50000.0), "{:.3f}"),
    C("j-t4-b", "P Table 4", "-0.027 degrees; 0.43 degrees (broken)", _JB(50000.0), "{:.2f}"),
    C("j-req-c", "P §3.3", "the cornering wall holds while each end is stiffer than 11 kN/mm (-0.027",
      lambda c: _rv("joints", "required_n_per_mm", "corner")(c) / 1000.0, "{:.0f}"),
    C("j-c50", "P §3.3", "(-0.027 degrees at 50 kN/mm).", _JC(50000.0), "{:.3f}"),
    C("j-b100", "P §3.3", "rises to 0.33 degrees at 100 kN/mm per end", _JB(100000.0), "{:.2f}"),
    C("j-b50", "P §3.3", "0.43 degrees at 50 and 0.74 degrees at 20", _JB(50000.0), "{:.2f}"),
    C("j-b20", "P §3.3", "0.43 degrees at 50 and 0.74 degrees at 20, and", _JB(20000.0), "{:.2f}"),
    C("j-req-b", "P §3.3", "the wall holds only above 494 kN/mm",
      lambda c: _rv("joints", "required_n_per_mm", "braking")(c) / 1000.0, "{:.0f}"),
    C("j-lf", "P §3.3", "which carries 7.3 kN in the stop", lambda c: _rv("joints", "lf_force_braking_n")(c) / 1000.0),
    C("j-move", "P §3.3", "would need 23 mm at 50 kN/mm", _rv("joints", "lf_move_for_wall_at_ref_mm"), "{:.0f}"),
    C("j-clear", "P §3.3", "where the wheel-clearance wall allows 6 mm", _rv("joints", "lf_move_clearance_allows_mm"),
      "{:.0f}"),
    C("j-diff", "P §3.3", "for a left-right brake difference up to 37%", _pct(_rv("joints", "brake_diff_allowed_at_ref")),
      "{:.0f}"),
    C("j-lash", "P §3.3", "Joint clearance adds a further 0.18 degrees of toe deadband per", _rv("joints", "lash_toe_per_0p1mm_deg"),
      "{:.2f}"),
    C("j-t5", "P §3.3", "where front_v11_final would need 494 kN/mm",
      lambda c: _rv("joints", "required_n_per_mm", "braking")(c) / 1000.0, "{:.0f}"),
    C("l-arm", "P §3.3", "change by under 0.6%", lambda c: 100.0 * (max(_rv("lock", "final", "arm_inner_mm")(c),
                                                                      _rv("lock", "final", "arm_outer_mm")(c))
                                                                  / _rv("lock", "final", "arm_straight_mm")(c) - 1.0)),
    C("l-jack", "P §3.3", "adds at most 0.16 Nm", _rv("lock", "final", "jacking_static", "hand_Nm"), "{:.2f}"),
    C("l-hand", "P §3.3", "so the hand torque at the hairpin is 8.3 Nm", _rv("lock", "final", "hand_lock_Nm")),
    C("l-alt", "P §3.3", "(7.5 with the alternative upright)", _rv("lock", "alternative", "hand_lock_Nm")),
    C("l-stand", "P §3.3", "would take about 16 Nm at the hand (an", _rv("lock", "standstill", "hand_Nm"), "{:.0f}"),
    C("y-t4-F", "P Table 4", "0.347; 100% (78.0% at hand weld)", _pct(_YH("front_v11_final", "full_yield"))),
    C("y-t4-R", "P Table 4", "0.41; 100% (93.2% at hand weld)", _pct(_YH("rear_v11_final", "full_yield"))),
    C("y-jig-lb", "P §3.3", "(95% lower bound 99.9%)", _pct(_rv("yield", "front_v11_final", "fields", "jig", "full_lower_95"))),
    C("y-lin-F", "P §3.3", "against 80.5% and 94.0% to first order", _pct(_YH("front_v11_final", "linear_yield"))),
    C("y-lin-R", "P §3.3", "against 80.5% and 94.0% to first order", _pct(_YH("rear_v11_final", "linear_yield"))),
]
CLAIMS += S12


# ---- the final front re-searched with its joints declared (paper section 3.3, supplement S13.6) -------------------- #
def _jf(*path):
    def f(c):
        if "joints_front" not in _RV:
            import review_checks as rcx
            _RV["joints_front"] = rcx.joints_front()
        v = _RV["joints_front"]
        for k in path:
            v = v[k]
        return v
    return f


S13 = [
    C("jf-hash", "P §3.3", "returns front_v11_joints50 (3678a72d786b)", _hash("front_v11_joints50"), "{:012x}"),
    C("jf-evals", "P §3.3", "in 260 evaluations moves", _jf("evaluations"), "{:d}"),
    C("jf-lf", "P §3.3", "moves the lower-front 32.0 mm", _jf("moves_mm", "lower_front_inner"), "{:.0f}"),
    C("jf-lr", "P §3.3", "the lower-rear 16.0 mm", _jf("moves_mm", "lower_rear_inner"), "{:.0f}"),
    C("jf-uf", "P §3.3", "the upper-front 23.3 mm", _jf("moves_mm", "upper_front_inner"), "{:.0f}"),
    C("jf-ur", "P §3.3", "the upper-rear 6.0 mm", _jf("moves_mm", "upper_rear_inner"), "{:.0f}"),
    C("jf-tro", "P §3.3", "the tie-rod inner 22.6 mm", _jf("moves_mm", "tie_rod_inner"), "{:.1f}"),
    C("jf-yield", "P §3.3", "returns front_v11_joints50 (3678a72d786b), RESILIENT at 99.35%", lambda c: 100.0 * _jf("yield")(c), "{:.2f}"),
    C("jf-abs", "P §3.3", "moves the lower-front 32.0 mm", lambda c: max(_jf("moves_mm")(c).values()), "{:.0f}"),
]
CLAIMS += S13


# ---- the front to build, every front check repeated (paper Table 4, section 3.3, Table 5) ------------------------- #
def _fb(name, *path):
    def f(c):
        if "front_to_build" not in _RV:
            import review_checks as rcx
            _RV["front_to_build"] = rcx.front_to_build()
        v = _RV["front_to_build"][name]
        for k in path:
            v = v[k]
        return v
    return f


def _yj(*path):
    def f(c):
        if "yield_j50" not in _RV:
            import yield_full_check as yfc
            _RV["yield_j50"] = yfc.check("front_v11_joints50", 2000)
        v = _RV["yield_j50"]
        for k in path:
            v = v[k]
        return v
    return f


_J5 = lambda *p: _fb("front_v11_joints50", *p)                              # noqa: E731
_F0 = lambda *p: _fb("front_v11_final", *p)                                 # noqa: E731
S14 = [
    C("b-band", "P Table 4", "0.594; 99.1% (57.1% at hand weld)", _J5("band"), "{:.3f}"),
    C("b-yield", "P Table 4", "RESILIENT at 99.35% (99.1% in full solves)", lambda c: 100.0 * _J5("yield")(c), "{:.2f}"),
    C("b-hand", "P Table 4", "0.594; 99.1% (57.1% at hand weld)", lambda c: 100.0 * _yj("fields", "hand_weld", "full_yield")(c)),
    C("b-clr", "P Table 4", "+0.71 mm", _J5("clearance_mm"), "{:+.2f}"),
    C("b-lock", "P Table 4", "115.6 degrees; 85.3%", _J5("props", "lock_sw_deg")),
    C("b-ack", "P Table 4", "115.6 degrees; 85.3%", _J5("props", "ackermann_lock_pct")),
    C("b-rig-c", "P Table 4", "rigid joints: −0.006° in the 1.5 g corner (no longer printed)", _J5("toe_rigid", "corner"), "{:.3f}", doc=""),
    C("b-rig-b", "P Table 4", "0.035°; 0.09°", _J5("toe_rigid", "braking"), "{:.2f}", doc=""),
    C("b-50-c", "P Table 4", "-0.018 degrees; 0.249 degrees", _J5("props", "compliance_toe_deg"), "{:.3f}"),
    C("b-50-b", "P Table 4", "-0.018 degrees; 0.249 degrees", _J5("props", "compliance_toe_braking_deg"), "{:.3f}"),
    C("b-ad-F", "P Table 4", "68.9%; 1.95", _F0("anti_dive_73")),
    C("b-mu-F", "P Table 4", "68.9%; 1.95", _F0("mu_front"), "{:.2f}"),
    C("b-ad", "P Table 4", "64.1%; 1.84", _J5("anti_dive_73")),
    C("b-mu", "P Table 4", "64.1%; 1.84", _J5("mu_front"), "{:.2f}"),
    C("b-hand-F", "P Table 4", "8.1 Nm; 22.4 mm", _F0("margin", "declared", "hand_brush_Nm")),
    C("b-t0-F", "P Table 4", "8.1 Nm; 22.4 mm", _F0("margin", "declared", "t0_limit_mm")),
    C("b-hand-J", "P Table 4", "8.3 Nm; 21.5 mm", _J5("margin", "declared", "hand_brush_Nm")),
    C("b-t0-J", "P Table 4", "8.3 Nm; 21.5 mm", _J5("margin", "declared", "t0_limit_mm")),
    C("b-link", "P Table 4", "2.72; 2.43", _J5("props", "link_fos_min"), "{:.2f}"),
    C("b-brg", "P Table 4", "2.72; 2.43", _J5("props", "rocker_bearing_sf"), "{:.2f}"),
    C("b-mr", "P Table 4", "0.802; 176.8 mm", _J5("mr_static"), "{:.3f}"),
    C("b-spring", "P Table 4", "0.802; 176.8 mm", _J5("spring_mm", "0.0")),
    C("b-ack-txt", "P §3.3", "the front to build runs 85.3% and gives up 0.02%", _J5("props", "ackermann_lock_pct"), "{:.0f}"),
    C("b-ackloss15", "P §3.3", "the front to build, at 85.3%, gives up 0.02%", _J5("ackermann_loss_pct", 1.5), "{:.2f}"),
    C("b-ackloss10", "P Table 5", "0.02%; 0.00%", _J5("ackermann_loss_pct", 1.0), "{:.2f}", doc=""),
    C("b-ratio", "P §3.3", "8.3 Nm at the hand on the front to build’s 4.88:1", _J5("ratio"), "{:.2f}", doc="S"),
    C("b-hand-txt", "P §3.3", "the same checks give 8.3 Nm at the hand", _J5("margin", "declared", "hand_brush_Nm")),
    C("b-t0", "P §3.3", "a trail limit of 21.5 mm", _J5("margin", "declared", "t0_limit_mm")),
    C("b-25", "P §3.3", "10.9 Nm at 25.4 mm of trail", _J5("margin", "declared", "hand_assumed_Nm")),
    C("b-lockT", "P §3.3", "and 8.5 Nm at the hairpin.", _J5("lock", "hand_lock_Nm")),
    C("b-alt05", "P §3.3", "clearance +0.15 mm, every wall held), giving 9.9 Nm at 25.4 mm", _J5("alt_build", "clearance_mm"), "{:+.2f}"),
    C("b-alt05-d", "P §3.3", "clearance +0.15 mm, every wall held), giving 9.9 Nm at 25.4 mm", _J5("alt_build", "hand_25p4", "declared")),
    C("b-alt05-r", "P §3.3", "on the declared set and 9.6 at road 1.43", _J5("alt_build", "hand_25p4", "r25b_road_1p43")),
    C("b-belt", "P §3.1", "(9.9 Nm at the hand against 10; 10.1 Nm, just over the wall, on the front to build", _J5("margin", "r25b_car_tire_belt", "hand_brush_Nm")),
    C("b-limits", "P §4", "and the 21.5 mm of the front to build", _J5("margin", "declared", "t0_limit_mm")),
    C("b-hs-c", "P Table 5", "+0.82 to -1.03 degrees", _J5("half_scale", "values_deg", "camber_bump"), "{:+.2f}"),
    C("b-hs-c0", "P Table 5", "+0.82 to -1.03 degrees", _J5("half_scale", "values_deg", "camber_droop"), "{:+.2f}"),
    C("b-hs-t", "P Table 5", "-0.048 to -0.044 degrees", _J5("half_scale", "values_deg", "toe_droop"), "{:+.3f}"),
    C("b-hs-s", "P Table 5", "+0.43 to -0.51 degrees", _J5("half_scale", "values_deg", "spacer_toe_bump"), "{:+.2f}"),
    C("b-hs-band", "P Table 5", "within 0.016 degrees, 0.043 degrees, and 0.042 degrees (95%",
      lambda c: max(_J5("half_scale", "band_deg", "camber_droop")(c), _J5("half_scale", "band_deg", "camber_bump")(c)),
      "{:.3f}"),
    C("b-t5-clr", "P Table 5", "the front to build clears by 3.71 mm", lambda c: 3.0 + _J5("clearance_mm")(c), "{:.2f}"),
    C("b-t5-mis", "P Table 5", "3.71 mm; 4.5°", _J5("props", "joint_misalignment_lock_deg"), doc=""),
    C("b-t5-gain", "P Table 5", "−0.037 deg/mm; 0.048°", _J5("corner", "gain"), "{:.3f}", doc=""),
    C("b-t5-bs", "P Table 5", "−0.037 deg/mm; 0.048°", _J5("corner", "bump_steer"), "{:.3f}", doc=""),
    C("b-t5-t0", "P Table 5", "zero-slip trail 21.5 mm; near 0.47 m", _J5("margin", "declared", "t0_limit_mm")),
]
CLAIMS += S14


# ---- what it changes on the car (paper section 3.4, Table 5) ------------------------------------------------------- #
def _cg(*path):
    def f(c):
        if "car_gains" not in _RV:
            import review_checks as rcx
            _RV["car_gains"] = rcx.car_gains()
        v = _RV["car_gains"]
        for k in path:
            v = v[k]
        return v
    return f


_gain = lambda c: 100.0 * (_cg("decel_g", "73")(c) / _cg("decel_g", "60")(c) - 1.0)          # noqa: E731
S15 = [
    C("g-d60", "P §3.4", "At the declared 60% front bias the rear tires saturate first and the car stops at 1.27g", _cg("decel_g", "60"), "{:.2f}"),
    C("g-d73", "P §3.4", "lifts the limit to 1.57g, 24% more", _cg("decel_g", "73"), "{:.2f}"),
    C("g-pct", "P §3.4", "the limit to 1.57g, 24% more", _gain, "{:.0f}"),
    C("g-s73", "P §3.4", "25.0 m from 100 km/h instead of 31.0 m", _cg("stop_m", "73")),
    C("g-s60", "P §3.4", "25.0 m from 100 km/h instead of 31.0 m", _cg("stop_m", "60")),
    C("g-dtoe", "P §3.4", "The delivered front toes out 0.80 degrees per wheel,", _cg("delivered", "brake_toe_50_deg"), "{:.2f}"),
    C("g-ddiff", "P §3.4", "so a left-right brake difference above 20%", lambda c: 100.0 * _cg("delivered", "brake_diff_ok")(c), "{:.0f}"),
    C("g-btoe", "P §3.4", "The front to build toes out 0.249 degrees,", _cg("build", "brake_toe_50_deg"), "{:.3f}"),
    C("g-bdiff", "P §3.4", "the band then holds up to a 64% brake difference", lambda c: 100.0 * _cg("build", "brake_diff_ok")(c), "{:.0f}"),
    C("g-dlock", "P §3.4", "The delivered front needs 137 degrees of steering wheel at", _cg("delivered", "lock_sw_deg"), "{:.0f}"),
    C("g-block", "P §3.4", "the front to build needs 115.6 degrees with 8.3", _cg("build", "lock_sw_deg")),
    C("g-bhand", "P §3.4", "with 8.3 Nm at the hand in a 1.5g corner. The", _cg("build", "hand_Nm")),
    C("g-dack", "P §3.4", "runs −1.9% Ackermann", _cg("delivered", "ackermann_pct")),
    C("g-dloss", "P §3.4", "which gives up 2.8% of the front axle’s", _cg("delivered", "ack_loss_pct", 1.5)),
    C("g-bloss", "P §3.4", "the front to build runs 85.3% and gives up 0.02%", _cg("build", "ack_loss_pct", 1.5), "{:.2f}"),
    C("g-t-d60", "P Table 5", "1.27g at 60% bias", _cg("decel_g", "60"), "{:.2f}"),
    C("g-t-dtoe", "P Table 5", "so a left-right brake difference above 20% steers the axle past it", lambda c: 100.0 * _cg("delivered", "brake_diff_ok")(c), "{:.0f}"),
    C("g-t-dlock", "P Table 5", "137.1 degrees", _cg("delivered", "lock_sw_deg")),
    C("g-t-dl10", "P Table 5", "2.8%; 3.5%", _cg("delivered", "ack_loss_pct", 1.0)),
    C("g-t-bl10", "P Table 5", "0.02%; 0.00%", _cg("build", "ack_loss_pct", 1.0), "{:.2f}", doc=""),
    C("g-t-dh", "P Table 5", "7.3 Nm; 5.56:1", _cg("delivered", "hand_Nm")),
    C("g-t-dr", "P Table 5", "7.3 Nm; 5.56:1", _cg("delivered", "ratio"), "{:.2f}"),
    C("g-t-br", "P Table 5", "8.3 Nm at the hand on the front to build’s 4.88:1", _cg("build", "ratio"), "{:.2f}", doc="S"),
    C("g-abs", "P abstract", "of the declared 60%, brakes 24% harder on the declared tire", _gain, "{:.0f}"),
    C("g-abs-b", "P abstract", "the band then holds up to a 64% brake difference",
      lambda c: 100.0 * _cg("build", "brake_diff_ok")(c), "{:.0f}"),
    C("g-con", "P §6", "so a left-right brake difference above 20% steers the axle past it",
      lambda c: 100.0 * _cg("delivered", "brake_diff_ok")(c), "{:.0f}"),
]
CLAIMS += S15


# ---- review items (paper s14): brake result against grip, independent solver ------------------------------------- #
def _bg(*path):
    def f(c):
        if "brake_grip" not in _RV:
            import review_checks as rcx
            _RV["brake_grip"] = rcx.brake_grip()
        v = _RV["brake_grip"]
        for k in path:
            v = v[k]
        return v
    return f


def _ik(name, key):
    def f(c):
        if "ik" not in _RV:
            import independent_kinematics as ikx
            _RV["ik"] = {n: ikx.compare(n) for n in ("front_v11_final", "rear_v11_final", "front_v11_joints50")}
        return _RV["ik"][name][key]
    return f


_ikmax = lambda c: max(abs(_ik(n, "rc_independent_mm")(c) - _ik(n, "rc_kinematik_mm")(c))           # noqa: E731
                       for n in ("front_v11_final", "rear_v11_final", "front_v11_joints50"))
S16 = [
    C("bg-mu60", "P §3.4", "The declared 60% balances only at μ = 0.70", _bg("mu_balanced_at_60"), "{:.2f}", doc=""),
    C("bg-155g", "P §3.4", "brakes 24% harder on the declared tire", _bg("const155", "gain73_pct"), "{:.0f}"),
    C("bg-155a", "P §3.4", "at a constant μ of 1.55 it is the same 24% (1.21 to 1.50 g)", _bg("const155", "d60"), "{:.2f}", doc=""),
    C("bg-155b", "P §3.4", "at a constant μ of 1.55 it is the same 24% (1.21 to 1.50 g)", _bg("const155", "d73"), "{:.2f}", doc=""),
    C("bg-blo", "P §3.4", "the balanced bias is 67 to 73% and", lambda c: 100 * _bg("road", "1.09", "b_bal")(c), "{:.0f}", doc=""),
    C("bg-bhi", "P §3.4", "the balanced bias is 67 to 73% and", lambda c: 100 * _bg("road", "1.43", "b_bal")(c), "{:.0f}", doc=""),
    C("bg-glo", "P §3.4", "brakes 24% harder on the declared tire (13 to 24% across published road grip", _bg("road", "1.09", "gain_bal_pct"), "{:.0f}"),
    C("bg-ghi", "P §3.4", "brakes 24% harder on the declared tire (13 to 24% across published road grip", _bg("road", "1.43", "gain_bal_pct"), "{:.0f}"),
    C("bg-s60", "P §3.4", "(34.2 against 27.5 m at μ 1.43)", _bg("road", "1.43", "stop60_m"), doc=""),
    C("bg-sbal", "P §3.4", "(34.2 against 27.5 m at μ 1.43)", _bg("road", "1.43", "stopbal_m"), doc=""),
    C("bg-abs-lo", "P abstract", "tire (13 to 24% across published road grip", _bg("road", "1.09", "gain_bal_pct"), "{:.0f}"),
    C("bg-abs-hi", "P abstract", "tire (13 to 24% across published road grip", _bg("road", "1.43", "gain_bal_pct"), "{:.0f}"),
    C("bg-t5", "P Table 5", "lifts the limit to 1.57g, 24% more deceleration", _bg("road", "1.43", "gain_bal_pct"), "{:.0f}"),
    C("bg-con", "P §6", "13 to 24% across published road grip", _bg("road", "1.09", "gain_bal_pct"), "{:.0f}"),
    C("ik-rcF", "P §3.3", "the roll centers (52.9 against 52.8, 42.2 against 41.6 and 56.2 against 56.1 mm)", _ik("front_v11_final", "rc_kinematik_mm"), doc="S"),
    C("ik-rcFi", "P §3.3", "the roll centers (52.9 against 52.8, 42.2 against 41.6 and 56.2 against 56.1 mm)", _ik("front_v11_final", "rc_independent_mm"), doc="S"),
    C("ik-rcR", "P §3.3", "the roll centers (52.9 against 52.8, 42.2 against 41.6 and 56.2 against 56.1 mm)", _ik("rear_v11_final", "rc_kinematik_mm"), doc="S"),
    C("ik-rcRi", "P §3.3", "the roll centers (52.9 against 52.8, 42.2 against 41.6 and 56.2 against 56.1 mm)", _ik("rear_v11_final", "rc_independent_mm"), doc="S"),
    C("ik-rcmax", "P §3.3", "roll-centre heights to 0.6 mm", _ikmax, doc=""),
]


def _superseded_ack(c):
    from paper_check import MAN as _MAN
    from suspension.genesis_repro import GenesisManifest as _GM, hp_from_dict as _hp
    m = _GM.from_json((_MAN / "superseded" / "front_v11_joints50.7d226fcc1090.recorded.genesis.json").read_text())
    return m.objects()[2].properties.evaluate(_hp(m.recorded["winner_hardpoints"]))["ackermann_lock_pct"]


S16 += [
    C("ak-ratioF", "P §3.3", "walls in the search 4.99:1 at center", _F0("ratio"), "{:.2f}"),
    C("ak-lock", "P §3.3", "and reaches the hairpin with 115.6 degrees of wheel", _J5("props", "lock_sw_deg")),
    C("ak-super", "P §3.3", "Without the Ackermann ceiling the same search left 144% at lock", _superseded_ack, "{:.0f}"),
]
CLAIMS += S16


# ---- Adams View benchmark (paper 3.3, table 6; supplement M6.4, S6.39) ---------------------------------------- #
_AV_CORNERS = ("front_v11_final", "rear_v11_final", "front_v11_joints50")
_AV_ANG, _AV_MM = ("camber", "toe_in", "caster", "kpi"), ("wc_dx", "wc_dy")


def _av(name, q):
    def f(c):
        if "av" not in _RV:
            import adams_view_benchmark as avx
            _RV["av"] = {n: avx.compare(n) for n in _AV_CORNERS}
        return _RV["av"][name]["worst"][q]
    return f


_av_ang = lambda c: max(_av(n, q)(c) for n in _AV_CORNERS for q in _AV_ANG)     # noqa: E731
_av_mm = lambda c: max(_av(n, q)(c) for n in _AV_CORNERS for q in _AV_MM)       # noqa: E731
_AV_P = "to within 0.00003 degrees and 0.00003 mm against limits of 0.005 to 0.01 degrees and 0.05 mm"
_AV_T6 = "Passed: at most 0.00003 degrees and 0.00003 mm"
_AV_M64 = "Over all three corners the largest differences are 0.00003 degrees and 0.00003 mm (S6.39"
S16b = [
    C("av-ang", "P §3.3", _AV_P, _av_ang, "{:.5f}"),
    C("av-mm", "P §3.3", _AV_P, _av_mm, "{:.5f}"),
    C("av-t6-ang", "P Table 6", _AV_T6, _av_ang, "{:.5f}"),
    C("av-t6-mm", "P Table 6", _AV_T6, _av_mm, "{:.5f}"),
    C("av-m64-ang", "S M6.4", _AV_M64, _av_ang, "{:.5f}", doc="S"),
    C("av-m64-mm", "S M6.4", _AV_M64, _av_mm, "{:.5f}", doc="S"),
]
_AV_ROWS = {"front_v11_final": "front_v11_final 0.000026 0.000004 0.000009 0.000004 0.000025 0.000027",
            "rear_v11_final": "rear_v11_final 0.000002 0.000005 0.000005 0.000003 0.000013 0.000025",
            "front_v11_joints50": "front_v11_joints50 0.000027 0.000004 0.000010 0.000006 0.000017 0.000027"}
for _n, _row in _AV_ROWS.items():
    for _q in _AV_ANG + _AV_MM:
        S16b.append(C(f"av-{_n}-{_q}", "S Table S6.39", _row, _av(_n, _q), "{:.6f}", doc="S"))
CLAIMS += S16b


# ---- the target chain (paper s16, Section 2): roll gradient derived, 3.0 Hz rear ------------------------------ #
def _rg(axle):
    def f(c):
        from suspension import target_derivation as tdx
        F, R = c.corners["F"], c.corners["R"]
        return (tdx.roll_gradient_for_friction(tdx.Vehicle(), tdx.Tire(), R["as"], "rear", 3.0, 1.82) if axle == "R"
                else tdx.roll_gradient_for_friction(tdx.Vehicle(), tdx.Tire(), F["ad73"], "front", 2.6, 1.82))
    return f


def _springs_grad(c):
    from suspension import target_derivation as tdx
    return tdx.springs_only_roll_gradient(tdx.Vehicle(), c.corners["F"]["rc"], c.corners["R"]["rc"])["gradient_deg_per_g"]


S17 = [
    C("rg-R", "P §2", "budget at 1.82 is 0.79 deg/g (the front binds at 0.87)", _rg("R"), "{:.2f}"),
    C("rg-F", "P §2", "at 1.82 is 0.79 deg/g (the front binds at 0.87); 0.78", _rg("F"), "{:.2f}"),
    C("rg-spr", "P §2", "springs alone give 1.36 deg/g", _springs_grad, "{:.2f}"),
    C("rg-t1", "P Table 1", "0.79 deg/g, the softest that holds at 1.82 (1.02", _rg("R"), "{:.2f}"),
    C("rr-floor", "P §2", "above the final rear’s 2.76 Hz floor and 15% above the front, so", lambda c: c.targets["ride_R"], "{:.2f}"),
    C("rr-pct", "P §2", "above the final rear’s 2.76 Hz floor and 15% above the front,", lambda c: 100 * (3.0 / 2.6 - 1), "{:.0f}"),
    C("rg-S", "S M3.1", "up to 0.788 deg/g at 1.82, the", _rg("R"), "{:.3f}", doc="S"),
]
S17 += [
    C("rf-final", "P §2", "lowers its floor to 2.21 Hz, and the 0.39 Hz between them", lambda c: c.targets["ride_F"], "{:.2f}"),
    C("rf-margin", "P §2", "lowers its floor to 2.21 Hz, and the 0.39 Hz between them", lambda c: 2.6 - c.targets["ride_F"], "{:.2f}"),
    C("comb-g", "P §2", "and 1.06g each way for combined braking and", lambda c: 1.5 / 2 ** 0.5, "{:.2f}"),
]
CLAIMS += S17

