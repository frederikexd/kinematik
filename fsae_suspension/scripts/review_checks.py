# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Three checks on the final front that its walls do not make (paper section 3.3, supplement S13.6).

joints():  the compliance walls with the link ends (joints, tabs, upright arms) in series along every link, one
           stiffness per end (suspension/compliance_margin.py): toe at 100, 50 and 20 kN/mm per end, the end stiffness
           each wall needs, the lower-front leg's force in the stop, the toe deadband per 0.1 mm of joint clearance, the
           brake difference the braking wall's own rule then allows, and the lower-front pickup moved forward along its
           pivot axis (every curve unchanged): how far the braking wall needs it at 50 kN/mm and how far the wheel
           clearance allows.
lock():    the steering at full lock (suspension/steering_margin.lock_check): moment arms about the steering axis,
           weight jacking by virtual work, the hand torque at the hairpin, on the final front and the alternative
           upright; and the standstill scrub torque as an estimate, which the effort limit does not cover.
yield_():  full solves of sampled builds against the first-order yield (scripts/yield_full_check.py).
Lengths mm, forces N, stiffness N/mm, angles deg, torques N*m. usage: review_checks.py [N_full] [out.json]"""
from __future__ import annotations

import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from suspension import compliance_margin as cm                              # noqa: E402
from suspension import elastokinematics as ek                               # noqa: E402
from suspension import steering_margin as smg                               # noqa: E402
from suspension.genesis_repro import GenesisManifest                        # noqa: E402

FINAL_FRONT = ROOT / "manifests" / "front_v11_final.recorded.genesis.json"
K_REF = 50000.0                      # N/mm per link end, the reference case the paper quotes
BRAKE_DIFF_DECLARED = 0.60           # the left-right brake force difference behind the +/-0.25 deg wall
ALT_BUILD_MM = -2.5                  # mm, the front to build's alternative upright: spindle 2.5 mm forward
TOE_BAND = 0.08                      # deg, the toe band the net steer must stay inside


def _man():
    return GenesisManifest.from_json(FINAL_FRONT.read_text())


def _lf_moved(hp, d_mm: float):
    lf, lr = np.asarray(hp.lower_front_inner, float), np.asarray(hp.lower_rear_inner, float)
    u = (lf - lr) / np.linalg.norm(lf - lr)
    return replace(hp, lower_front_inner=tuple(lf + d_mm * u))


def _bisect(f, lo: float, hi: float, n: int = 40) -> float:
    """Root of a monotone f on [lo, hi] (f(lo) and f(hi) of opposite sign)."""
    flo = f(lo)
    for _ in range(n):
        m = 0.5 * (lo + hi)
        if (f(m) > 0) == (flo > 0):
            lo, flo = m, f(m)
        else:
            hi = m
    return 0.5 * (lo + hi)


def joints() -> dict:
    """Toe with joint and tab compliance at the link ends; stiffness N/mm, toe deg, force N, offsets mm."""
    m = _man()
    hp = smg.corner(m)
    W = cm.walls(m)
    sweep = cm.end_stiffness_sweep(m, (math.inf, 100000.0, K_REF, 20000.0))
    req = cm.required_end_stiffness(m)
    sb, lb = W["braking"]
    rb = ek.solve_elastokinematic(hp, sb, jacobian=False)
    toe_ref = sweep["braking"]["toe_deg"][K_REF]
    _, _, vol, _ = m.objects()
    spec_ref = cm.with_end_stiffness(sb, K_REF)
    d_wall = _bisect(lambda d: abs(cm.toe(_lf_moved(hp, d), spec_ref)) - lb, 0.0, 80.0)
    d_clear = _bisect(lambda d: vol.envelope_margin(_lf_moved(hp, d)), 0.0, 40.0)
    return {"sweep": {c: {"limit_deg": v["limit_deg"], "toe_deg": {str(k): t for k, t in v["toe_deg"].items()}}
                      for c, v in sweep.items()},
            "required_n_per_mm": {c: v[0] for c, v in req.items()},
            "lf_force_braking_n": float(rb.member_forces["LF"]),
            "lash_toe_per_0p1mm_deg": cm.lash_toe_per_0p1mm(m),
            "brake_diff_allowed_at_ref": 2.0 * TOE_BAND / abs(toe_ref),
            "lf_move_for_wall_at_ref_mm": d_wall,
            "lf_move_clearance_allows_mm": d_clear,
            "clearance_at_wall_move_mm": float(vol.envelope_margin(_lf_moved(hp, d_wall)))}


def joints_front() -> dict:
    """front_v11_joints50 (the final front re-searched with 50 kN/mm link ends) against front_v11_final: pickup moves
    in mm, the walls' values (deg, %, mm) and the recorded grade and yield (dimensionless)."""
    J = GenesisManifest.from_json((ROOT / "manifests" / "front_v11_joints50.recorded.genesis.json").read_text())
    F = _man()
    from suspension.genesis_repro import hp_from_dict
    w, f = hp_from_dict(J.recorded["winner_hardpoints"]), smg.corner(F)
    _, tg, vol, _ = J.objects()
    pts = ["upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner", "tie_rod_inner",
           "tie_rod_outer"]
    v = vol.properties.evaluate(w)
    r, _ = tg.residual(w)
    base = lambda h: float(np.linalg.norm(np.subtract(h.lower_rear_inner, h.lower_front_inner)))    # noqa: E731
    keys = ("lock_sw_deg", "ackermann_lock_pct", "compliance_toe_braking_deg", "compliance_toe_deg", "anti_dive_pct",
            "link_fos_min", "rocker_bearing_sf")
    return {"hash": J.inputs_sha256[:12], "evaluations": J.recorded["start_search"]["evaluations"],
            "moves_mm": {p: float(np.linalg.norm(np.subtract(getattr(w, p), getattr(f, p)))) for p in pts},
            "base_mm": base(w), "base_final_mm": base(f), "props": {k: float(v[k]) for k in keys},
            "clearance_mm": float(vol.envelope_margin(w)), "band": float(np.max(np.abs(r))),
            "verdict": J.recorded["winner_verdict"], "yield": J.recorded["winner_yield"]}


def half_scale(hp, n: int = 400, tol_mm: float = 0.4, seed: int = 0) -> dict:
    """Predictions for the half-scale model of a corner (Table M10, rows 15 to 17), computed at full scale: angles do
    not scale, so +/-12.5 mm of model travel is +/-25 mm here and a 4.0 mm model spacer is 8.0 mm. Camber and toe
    change from static at droop and bump (deg); the same toe change with the tie-rod inner raised by the spacer; and
    each one's build band, the 95th-percentile change over ``n`` models with every point within +/-``tol_mm``
    (full scale, i.e. +/-0.2 mm on the model, uniform per axis)."""
    from suspension.kinematics import SuspensionKinematics
    from suspension.kinematik_stochastic import _perturbed

    def _q(h):
        k = SuspensionKinematics(h)
        s = {t: k.solve_at_travel(t) for t in (-25.0, 0.0, 25.0)}
        ks = SuspensionKinematics(h, pickup_deltas={"tie_rod_inner": np.array([0.0, 0.0, 8.0])})
        p = {t: ks.solve_at_travel(t) for t in (-25.0, 0.0, 25.0)}
        return np.array([s[-25.0].camber - s[0.0].camber, s[25.0].camber - s[0.0].camber,
                         s[-25.0].toe - s[0.0].toe, s[25.0].toe - s[0.0].toe,
                         p[-25.0].toe - p[0.0].toe, p[25.0].toe - p[0.0].toe], float)
    q0 = _q(hp)
    rng = np.random.default_rng(seed)
    pts = ["upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner", "tie_rod_inner",
           "upper_outer", "lower_outer", "tie_rod_outer"]
    dev = []
    for _ in range(n):
        try:
            dev.append(np.abs(_q(_perturbed(hp, {p: rng.uniform(-tol_mm, tol_mm, 3) for p in pts})) - q0))
        except Exception:
            continue
    band = np.percentile(np.array(dev), 95, axis=0)
    names = ("camber_droop", "camber_bump", "toe_droop", "toe_bump", "spacer_toe_droop", "spacer_toe_bump")
    return {"values_deg": dict(zip(names, map(float, q0))), "band_deg": dict(zip(names, map(float, band))),
            "n": len(dev), "tol_mm": tol_mm}


def _alt(m, hp, lv: dict, dx: float) -> dict:
    """A spindle offset of ``dx`` mm on a manifest's winner: walls, clearance (mm) and hand torque (N*m) at 25.4 mm."""
    import steering_margin as sms
    c = smg.offset_check(m, dx, hp=hp)
    se = smg.steer_effort(m)
    h = smg.spindle_offset(hp, dx)
    tr, ratio = smg.mechanical_trail(h), float(se.kinematic_ratio(hp))
    return {"dx_mm": dx, "trail_mm": tr, "walls_hold": c["walls_hold"], "clearance_mm": c["after"]["clearance_mm"],
            "hand_25p4": {n: smg.hand_torque(t, tr, ratio, se.fz_outer_n, se.fz_inner_n, sms.T0_ASSUMED)
                          for n, t in lv.items()}}


def front_to_build() -> dict:
    """Every front check of the paper repeated on front_v11_joints50 (the front the brief carries), beside
    front_v11_final: Table 4's rows, the anti-dive friction threshold, the steering margin, full lock, the alternative
    upright, the hairpin Ackermann cost and the half-scale model. Lengths mm, angles deg, torques N*m, yields
    dimensionless."""
    import steering_margin as sms
    import paper_check as pc
    from dataclasses import replace as _r
    from suspension import target_derivation as td, tire_field as tf
    from suspension.genesis_repro import hp_from_dict
    from suspension.kinematics import SuspensionKinematics
    from suspension import actuation as act
    import r25b_check as r25
    out = {}
    for name in ("front_v11_final", "front_v11_joints50"):
        m = GenesisManifest.from_json((ROOT / "manifests" / f"{name}.recorded.genesis.json").read_text())
        hp = hp_from_dict(m.recorded["winner_hardpoints"])
        _, tg, vol, _ = m.objects()
        v = vol.properties.evaluate(hp)
        r, _ = tg.residual(hp)
        k = SuspensionKinematics(hp)
        L = {t: k.spring_length_at(k.solve_at_travel(t), seed=0.0)[0] for t in (-25.0, 0.0, 25.0)}
        ad73 = pc.Ctx._anti(hp, "anti_dive_pct", brake_bias_front=0.73)
        lv = sms.tire_levels()
        mg = smg.margin(m, lv, sms.T0_ASSUMED)
        alt = smg.offset_check(m, sms.OFFSET_MM)
        alt_m = smg.margin(m, lv, sms.T0_ASSUMED, hp=smg.spindle_offset(hp, sms.OFFSET_MM))
        T = r25.tires()
        out[name] = {
            "band": float(np.max(np.abs(r))), "yield": m.recorded["winner_yield"], "verdict": m.recorded["winner_verdict"],
            "clearance_mm": float(vol.envelope_margin(hp)), "props": {kk: float(x) for kk, x in v.items()},
            "spring_mm": {str(t): float(x) for t, x in L.items()}, "mr": float((L[-25.0] - L[25.0]) / 50.0),
            "anti_dive_73": float(ad73),
            "mu_front": float(tf.mu_threshold_for("anti_dive_floor_pct", ad73, veh=_r(td.Vehicle(), ride_front_hz=2.6))),
            "margin": {n: {kk: float(x) for kk, x in t.items()} for n, t in mg["tires"].items()},
            "ratio": mg["ratio"], "trail_mm": mg["trail_mm"],
            "lock": smg.lock_check(m),
            "alt": {"walls_hold": alt["walls_hold"], "bands_hold": alt["bands_hold"], "after": alt["after"],
                    "hand_25p4": {n: t["hand_assumed_Nm"] for n, t in alt_m["tires"].items()}},
            "ackermann_loss_pct": {g: r25.ackermann(T["synthetic"], hp, g)["final_front_loss_pct"] for g in (1.5, 1.0)},
            "alt_build": _alt(m, hp, lv, ALT_BUILD_MM),
            "half_scale": half_scale(hp),
            "mr_static": float(act.actuation_metrics(hp).mr_static),
            "corner": {kk: float(x) for kk, x in pc.Ctx._corner(hp).items() if kk in ("gain", "bump_steer")},
            "toe_rigid": {c: cm.toe(hp, _r(sp, members={kk: _r(ms, k_tab=None) for kk, ms in sp.members.items()}))
                          for c, (sp, _) in cm.walls(m).items()}}
    return out


def car_gains() -> dict:
    """The delivered ChassisV10 front against the front to build, in what the car does (paper section 3.4): the delivered
    pickups (front_v11_from_delivered's declared start) and front_v11_joints50, both on the final front's rack, tire,
    load cases and actuation, with 50 kN/mm at every link end. Decelerations in g, distances in m, toe in deg, torques
    in N*m, force losses in % (dimensionless), from 100 km/h."""
    import r25b_check as r25
    from suspension.genesis_repro import hp_from_dict
    F = _man()
    D = GenesisManifest.from_json((ROOT / "manifests" / "front_v11_from_delivered.recorded.genesis.json").read_text())
    J = GenesisManifest.from_json((ROOT / "manifests" / "front_v11_joints50.recorded.genesis.json").read_text())
    _, _, vol, _ = F.objects()
    sp, _ = cm.walls(F)["braking"]
    spec50 = cm.with_end_stiffness(sp, K_REF)
    se = smg.steer_effort(F)
    tire = r25.tires()["synthetic"]
    v = 100.0 / 3.6
    out = {"decel_g": {"60": float(vol.properties.brake.decel_g(0.60)), "73": float(vol.properties.brake.decel_g(0.73))}}
    out["stop_m"] = {k: v * v / (2.0 * 9.80665 * g) for k, g in out["decel_g"].items()}
    for name, hp in (("delivered", D.objects()[0]), ("build", hp_from_dict(J.recorded["winner_hardpoints"]))):
        p = vol.properties.evaluate(hp)
        t50 = cm.toe(hp, spec50) if name == "delivered" else float(J.objects()[2].properties.evaluate(hp)[
            "compliance_toe_braking_deg"])
        ratio = float(se.kinematic_ratio(hp))
        out[name] = {"lock_sw_deg": float(p["lock_sw_deg"]), "ackermann_pct": float(p["ackermann_lock_pct"]),
                     "brake_toe_50_deg": float(t50), "brake_diff_ok": 2.0 * TOE_BAND / abs(t50),
                     "ack_loss_pct": {g: float(r25.ackermann(tire, hp, g)["final_front_loss_pct"]) for g in (1.5, 1.0)},
                     "ratio": ratio,
                     "hand_Nm": smg.hand_torque(tire, smg.mechanical_trail(hp), ratio, se.fz_outer_n, se.fz_inner_n)}
    return out


def brake_grip(mass_kg: float = 300.0, cg_height_mm: float = 280.0, wheelbase_mm: float = 1630.0,
               weight_front: float = 0.48, mu_road=(1.09, 1.43), bias=(0.60, 0.73)) -> dict:
    """The brake result's dependence on grip (paper section 3.4), closed form at a constant friction coefficient.

    With every tire at the same mu, the front axle saturates at a/g = mu w_f / (b - mu h/L) and the rear at
    a/g = mu (1 - w_f) / ((1 - b) + mu h/L); the limit is the lower. Both meet at b = w_f + mu h/L, where a/g = mu.
    Returns, at mu 1.55 and at each road mu, the limits (g) at the declared and the built bias, the balanced bias
    (fraction), the gains (%) and the stops from 100 km/h (m); and the mu at which the declared bias is balanced.
    The mass does not enter: every term scales with it."""
    hL, wf = cg_height_mm / wheelbase_mm, weight_front

    def dec(mu, b):
        return min(mu * (1.0 - wf) / ((1.0 - b) + mu * hL), mu * wf / (b - mu * hL))

    v = 100.0 / 3.6
    stop = lambda a: v * v / (2.0 * 9.80665 * a)                        # noqa: E731
    b0, b1 = bias
    out = {"const155": {"d60": dec(1.55, b0), "d73": dec(1.55, b1),
                        "gain73_pct": 100.0 * (dec(1.55, b1) / dec(1.55, b0) - 1.0)},
           "mu_balanced_at_60": (b0 - wf) / hL, "road": {}}
    for mu in mu_road:
        bb = wf + mu * hL
        out["road"][f"{mu:.2f}"] = {"b_bal": bb, "d60": dec(mu, b0), "d73": dec(mu, b1), "dbal": dec(mu, bb),
                                    "gain73_pct": 100.0 * (dec(mu, b1) / dec(mu, b0) - 1.0),
                                    "gain_bal_pct": 100.0 * (dec(mu, bb) / dec(mu, b0) - 1.0),
                                    "stop60_m": stop(dec(mu, b0)), "stopbal_m": stop(dec(mu, bb))}
    return out


def lock() -> dict:
    """Full-lock steering on the final front and the alternative upright; torques N*m, arms mm."""
    m = _man()
    hp = smg.corner(m)
    out = {"final": smg.lock_check(m), "alternative": smg.lock_check(m, hp=smg.spindle_offset(hp, -2.0))}
    se = smg.steer_effort(m)
    st = 0.5 * (se.fz_outer_n + se.fz_inner_n)
    from suspension.tiremodel import default_tire
    mu = float(default_tire().mu_peak(st, 0.0))
    out["standstill"] = {"fz_n": st, "mu": mu, "per_wheel_Nm": smg.standstill_scrub_torque(st, mu),
                         "hand_Nm": 2.0 * smg.standstill_scrub_torque(st, mu) / float(se.kinematic_ratio(hp))}
    return out


def yield_(n_full: int = 2000) -> dict:
    """First-order against full-solve yield of the final corners (dimensionless fractions)."""
    import yield_full_check as y
    return y.run(n_full)


def run(n_full: int = 2000, out_path: str | None = None) -> dict:
    """All three checks; lengths in mm, forces in N, angles in deg, torques in N·m, yields dimensionless."""
    out = {"joints": joints(), "joints_front": joints_front(), "front_to_build": front_to_build(), "lock": lock(),
           "car_gains": car_gains(), "brake_grip": brake_grip(),
           "yield": yield_(n_full) if n_full else None}
    if out_path:
        Path(out_path).write_text(json.dumps(out, indent=1, default=float))
    return out


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
    print(json.dumps(run(n, sys.argv[2] if len(sys.argv) > 2 else None), indent=1, default=float))
