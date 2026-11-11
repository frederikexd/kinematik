# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""A team's own car, blind: predict two or more setups, seal, test, then score absolute error and A vs B.

The ledger can only use what teams publish. Prevalidation is about a team's own car with its own inputs, and about
the decision between setups: is B faster than A, and by how much? This module makes that a blind test a team can
run on its next test day:

  1. Plan.   The car (mass, CG, geometry, roll stiffness, tire file, aero, powertrain) and the setups to compare,
             each a set of overrides on the base car. Optional: the test course as an x, y centreline.
  2. Seal.   KinematiK predicts every channel for every setup, plus every A vs B difference, and seals the result
             with the code fingerprint and the time. Push the sealed record (or post its SHA-256 in a public issue)
             BEFORE the test day; that timestamp is what makes it blind.
  3. Test.   The team runs the setups and logs the channels.
  4. Score.  Measured values are compared with the sealed predictions: error per channel, and for every pair of
             setups whether the predicted difference has the right sign and size. Nothing is refitted.

Private data stays private: a tire file enters the record only as its SHA-256, never its coefficients.

Channels (s or g): skidpad_s (one timed circle), accel_s (from rest over accel_distance_m), lap_s (the test
course, if given), max_lat_g (steady-state lateral limit).

Suspension channels (deg), when the plan has a "kinematics" block: camber_<corner>_<travel>mm and
toe_<corner>_<travel>mm from the corner's hardpoints, at each listed wheel travel (mm, + = bump, wheel centre
relative to the chassis). Conventions are KinematiK's: camber negative = top of the wheel leans inboard, toe
positive = toe-out. These are scored as an error in degrees against plan["tolerance"] (for example camber 0.3,
toe 0.1), not in percent, because a percent of a near-zero angle means nothing.
"""
from __future__ import annotations

import csv
import hashlib
import itertools
import json
import math
from dataclasses import fields
from datetime import UTC, datetime
from pathlib import Path

from . import ledger as L
from .dynamics import VehicleDynamics, VehicleParams
from .kinematics import Hardpoints, SuspensionKinematics
from .laptime import MotorMap, Powertrain, acceleration_time, simulate_lap, skidpad_time, track_from_path
from .tiremodel import default_tire, load_from_json

KIT = "KinematiK team blind test"
KIT_VERSION = "1"
CHANNELS = ("skidpad_s", "accel_s", "lap_s", "max_lat_g")
_VP_FIELDS = {f.name for f in fields(VehicleParams)}
_PT_KEYS = ("power_kw", "drive", "cla", "cda", "drivetrain_eff", "crr", "brake_g_cap")
_CAR_KEYS = ("wheel_radius_m", "overall_ratio", "redline_rpm", "motor_torque_nm", "tire_json", "tire_mu_scale",
             "skidpad_radius_m", "accel_distance_m")
_CODE_FILES = ("team_test.py", "laptime.py", "dynamics.py", "tiremodel.py")


def _sha_file(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve(base: dict, overrides: dict) -> dict:
    """One setup's full car (units as in VehicleParams / Powertrain: kg, mm, N·m/deg, kW, m^2): base updated by
    the setup's overrides. Unknown keys raise ValueError, so a typo can't silently do nothing."""
    car = {**base, **overrides}
    known = _VP_FIELDS | set(_PT_KEYS) | set(_CAR_KEYS)
    bad = sorted(k for k in car if k not in known)
    if bad:
        raise ValueError(f"unknown input(s): {', '.join(bad)}")
    return car


def build(car: dict, root: Path | None = None):
    """(VehicleDynamics, Powertrain) for a resolved car (mm, kg, kW, m, rpm, N·m). A relative tire_json path is
    read from `root`."""
    vp = VehicleParams(**{k: v for k, v in car.items() if k in _VP_FIELDS})
    if car.get("tire_json"):
        tire = load_from_json(str((root or Path(".")) / car["tire_json"]))
    else:
        tire = default_tire()
    if car.get("tire_mu_scale"):
        tire.scaling = {**(tire.scaling or {}), "LMUY": float(car["tire_mu_scale"])}
    veh = VehicleDynamics(vp, tire=tire)
    pt_kw = {k: car[k] for k in _PT_KEYS if k in car}
    motor_map = None
    if car.get("overall_ratio") and car.get("redline_rpm"):
        r = car.get("wheel_radius_m", 0.20)
        torque = car.get("motor_torque_nm") or (L.TRACTION_TORQUE_MARGIN_G * vp.mass * vp.g * r
                                                / car["overall_ratio"] / max(pt_kw.get("drivetrain_eff", 0.9), 0.05))
        motor_map = MotorMap.from_peak(torque, pt_kw.get("power_kw", 80.0), car["redline_rpm"],
                                       final_drive=car["overall_ratio"], wheel_radius_m=r)
    pt = Powertrain(**pt_kw, max_tractive_n=math.inf, motor_map=motor_map)
    return veh, pt


def _course(plan: dict, root: Path | None):
    path = plan.get("course_csv")
    if not path:
        return None
    xs, ys = [], []
    with open((root or Path(".")) / path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
    return track_from_path(xs, ys, name=plan.get("course_name", "Test course"),
                           closed=bool(plan.get("course_closed", True)))


def _kin_inputs(plan: dict, overrides: dict, root: Path):
    """{corner: (hardpoints dict, travels)} for one setup: the plan's kinematics block, each corner's points from
    an inline "hardpoints" dict or a "hardpoints_json" file, then the setup's per-corner point overrides."""
    out = {}
    for corner, spec in (plan.get("kinematics") or {}).items():
        if spec.get("hardpoints_json"):
            hp = json.loads((root / spec["hardpoints_json"]).read_text(encoding="utf-8"))
        else:
            hp = dict(spec.get("hardpoints") or {})
        hp.update((overrides.get("kinematics") or {}).get(corner, {}))
        out[corner] = (hp, [float(t) for t in spec.get("travel_mm", (-25, -10, 0, 10, 25))])
    return out


def predict_kinematics(corners: dict) -> dict:
    """{camber_<corner>_<t>mm, toe_<corner>_<t>mm: deg} from {corner: (hardpoints dict in mm, travels in mm)}."""
    out = {}
    for corner, (hp, travels) in corners.items():
        kin = SuspensionKinematics(Hardpoints.from_dict(hp))
        for t in travels:
            st = kin.solve_at_travel(t)
            if not st.converged:
                raise ValueError(f"{corner}: the linkage does not solve at {t:+g} mm of travel")
            tag = f"{corner}_{t:+g}mm"
            out[f"camber_{tag}"] = round(float(st.camber), 3)
            out[f"toe_{tag}"] = round(float(st.toe), 3)
    return out


_POINT_KEYS = ("upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner", "upper_outer",
               "lower_outer", "tie_rod_inner", "tie_rod_outer", "wheel_center", "contact_patch")


def kinematics_uncertainty(corners: dict, sigma_mm: float, n: int = 200, seed: int = 0) -> dict:
    """How far camber and toe (deg) can move just from measuring the hardpoints with an error of sigma_mm (mm,
    one standard deviation, each coordinate independently): {channel: 2-sigma half-width in deg} over n random
    draws. Use it to set plan["tolerance"] before the test, from the measuring method, not from the results."""
    import numpy as np
    rng = np.random.default_rng(seed)
    nominal = predict_kinematics(corners)
    draws: dict = {ch: [] for ch in nominal}
    for _ in range(n):
        jittered = {}
        for corner, (hp, travels) in corners.items():
            j = dict(hp)
            for k in _POINT_KEYS:
                if k in j and j[k] is not None:
                    j[k] = (np.asarray(j[k], float) + rng.normal(0.0, sigma_mm, 3)).tolist()
            jittered[corner] = (j, travels)
        try:
            got = predict_kinematics(jittered)
        except ValueError:
            continue
        for ch, v in got.items():
            draws[ch].append(v - nominal[ch])
    return {ch: round(2.0 * float(np.std(d)), 3) if d else None for ch, d in draws.items()}


def suggested_tolerance(plan: dict, sigma_mm: float, gauge_deg: float, root: Path | None = None,
                        n: int = 200) -> dict:
    """{"camber": deg, "toe": deg}: the worst 2-sigma spread from hardpoint measuring error (sigma_mm, mm) over
    the plan's corners and travels, combined with the angle gauge's own accuracy (gauge_deg, deg) as a root sum
    of squares."""
    corners = _kin_inputs(plan, {}, Path(root) if root else Path("."))
    unc = kinematics_uncertainty(corners, sigma_mm, n=n)
    out = {}
    for kind in ("camber", "toe"):
        vals = [v for ch, v in unc.items() if ch.startswith(kind + "_") and v is not None]
        worst = max(vals, default=0.0)
        out[kind] = round(math.hypot(worst, gauge_deg), 2)
    return out


def _is_angle(channel: str) -> bool:
    return channel.startswith(("camber_", "toe_"))


def predict_setup(car: dict, course=None, root: Path | None = None) -> dict:
    """{channel: value} for one resolved car: times in s, max_lat_g in g. lap_s only when a course is given."""
    veh, pt = build(car, root)
    out = {"skidpad_s": float(skidpad_time(veh, pt, radius_m=car.get("skidpad_radius_m", 9.125)).lap_time_s),
           "accel_s": float(acceleration_time(veh, pt, distance_m=car.get("accel_distance_m", 75.0)).lap_time_s),
           "max_lat_g": float(veh.max_lateral_g())}
    if course is not None:
        out["lap_s"] = float(simulate_lap(veh, course, pt).lap_time_s)
    return {k: round(v, 4) for k, v in out.items()}


def _pairs(setups):
    return list(itertools.combinations(setups, 2))


def seal(plan: dict, root: Path | None = None, now: datetime | None = None) -> dict:
    """A sealed blind-test record (times in s, lateral limit in g; hashes unitless): every setup's predictions and
    every pairwise difference (B minus A), with the plan, the code fingerprint, input-file hashes and the time."""
    if not plan.get("setups") or len(plan["setups"]) < 2:
        raise ValueError("a blind test needs at least two setups to compare")
    root = Path(root) if root else Path(".")
    course = _course(plan, root)
    preds = {}
    for name, ov in plan["setups"].items():
        car_ov = {k: v for k, v in ov.items() if k != "kinematics"}
        preds[name] = {**predict_setup(resolve(plan.get("base", {}), car_ov), course, root),
                       **predict_kinematics(_kin_inputs(plan, ov, root))}
    deltas = {f"{b} - {a}": {ch: round(preds[b][ch] - preds[a][ch], 4) for ch in preds[a]}
              for a, b in _pairs(list(preds))}
    files = {}
    paths = [plan.get("course_csv")] + [spec.get("hardpoints_json") for spec in (plan.get("kinematics") or {}).values()]
    paths += [resolve(plan.get("base", {}), {k: v for k, v in ov.items() if k != "kinematics"}).get("tire_json")
              for ov in plan["setups"].values()]
    for p in paths:
        if p:
            files[p] = _sha_file(root / p)
    base = Path(__file__).resolve().parent
    rec = {"kit": KIT, "kit_version": KIT_VERSION, "team": plan.get("team", ""), "car": plan.get("car", ""),
           "test": plan.get("test", ""), "sealed_utc": (now or datetime.now(UTC)).strftime("%Y-%m-%dT%H:%M:%SZ"),
           "code_sha256": {f: _sha_file(base / f) for f in _CODE_FILES}, "input_files_sha256": files,
           "plan": plan, "predictions": preds, "predicted_deltas": deltas}
    rec["record_sha256"] = hashlib.sha256(L._canon(rec)).hexdigest()
    return rec


def verify(rec: dict, root: Path | None = None) -> dict:
    """{"intact", "same_code", "same_inputs", "reproduced"} (unitless booleans). Input files are checked by hash;
    reproduction needs them present."""
    body = {k: v for k, v in rec.items() if k != "record_sha256"}
    intact = hashlib.sha256(L._canon(body)).hexdigest() == rec.get("record_sha256")
    base = Path(__file__).resolve().parent
    same_code = {f: _sha_file(base / f) for f in _CODE_FILES} == rec.get("code_sha256")
    root = Path(root) if root else Path(".")
    present = all((root / p).exists() for p in rec.get("input_files_sha256", {}))
    same_inputs = present and all(_sha_file(root / p) == h for p, h in rec.get("input_files_sha256", {}).items())
    reproduced = False
    if same_inputs:
        again = seal(rec["plan"], root, now=datetime.now(UTC))
        reproduced = again["predictions"] == rec["predictions"]
    return {"intact": intact, "same_code": same_code, "same_inputs": same_inputs, "reproduced": reproduced}


def load_measured_csv(path) -> dict:
    """{setup: {channel: value}} from a CSV with columns setup, channel, value (s or g). Several rows for the
    same setup and channel are averaged (repeat runs)."""
    acc: dict = {}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            acc.setdefault(row["setup"].strip(), {}).setdefault(row["channel"].strip(), []).append(float(row["value"]))
    return {s: {c: sum(v) / len(v) for c, v in chans.items()} for s, chans in acc.items()}


def score(rec: dict, measured: dict) -> dict:
    """Score a sealed record against measured values (s or g; errors in %). Returns
    {"absolute": [...], "ab": [...], "summary": {...}}. A vs B rows say whether the predicted difference has the
    measured sign and how far its size is off; a measured difference within `noise` of zero is reported as a tie,
    not a hit or a miss. `noise` (per channel, same units) comes from plan["noise"], default 0."""
    body = {k: v for k, v in rec.items() if k != "record_sha256"}
    if hashlib.sha256(L._canon(body)).hexdigest() != rec.get("record_sha256"):
        raise ValueError("record altered since it was sealed")
    noise = rec["plan"].get("noise", {})
    tol = rec["plan"].get("tolerance", {})
    absolute, ab, angles = [], [], []
    for s, chans in rec["predictions"].items():
        for ch, p in chans.items():
            m = measured.get(s, {}).get(ch)
            if m is None:
                continue
            if _is_angle(ch):
                kind = ch.split("_")[0]
                band = tol.get(kind)
                err = p - m
                angles.append({"setup": s, "channel": ch, "predicted": p, "measured": round(m, 4),
                               "error_deg": round(err, 3), "tolerance_deg": band,
                               "within": None if band is None else abs(err) <= float(band)})
            else:
                absolute.append({"setup": s, "channel": ch, "predicted": p, "measured": round(m, 4),
                                 "error_pct": round(100.0 * (p - m) / m, 2)})
    names = list(rec["predictions"])
    for a, b in _pairs(names):
        for ch in rec["predictions"][a]:
            ma, mb = measured.get(a, {}).get(ch), measured.get(b, {}).get(ch)
            if ma is None or mb is None:
                continue
            pd, md = rec["predictions"][b][ch] - rec["predictions"][a][ch], mb - ma
            tie = abs(md) <= float(noise.get(ch, noise.get(ch.split("_")[0], 0.0)))
            ab.append({"pair": f"{b} - {a}", "channel": ch, "predicted_delta": round(pd, 4),
                       "measured_delta": round(md, 4), "tie": tie,
                       "sign_right": None if tie else (pd * md > 0),
                       "size_error_pct": None if tie or md == 0 else round(100.0 * (pd - md) / abs(md), 1)})
    decided = [r for r in ab if not r["tie"]]
    summary = {"absolute_n": len(absolute),
               "absolute_mae_pct": (sum(abs(r["error_pct"]) for r in absolute) / len(absolute)) if absolute else None,
               "absolute_within_5pct": sum(1 for r in absolute if abs(r["error_pct"]) <= 5.0),
               "ab_decided": len(decided), "ab_sign_right": sum(1 for r in decided if r["sign_right"]),
               "ab_ties": len(ab) - len(decided), "angles_n": len(angles),
               "angles_max_abs_error_deg": max((abs(r["error_deg"]) for r in angles), default=None),
               "angles_within_tolerance": sum(1 for r in angles if r["within"]),
               "angles_with_tolerance": sum(1 for r in angles if r["within"] is not None)}
    return {"team": rec.get("team", ""), "test": rec.get("test", ""), "sealed_utc": rec["sealed_utc"],
            "record_sha256": rec["record_sha256"], "absolute": absolute, "angles": angles, "ab": ab,
            "summary": summary}


def scorecard_markdown(sc: dict) -> str:
    """Human-readable scorecard (times in s, lateral limit in g, errors in %)."""
    s = sc["summary"]
    lines = [f"# Blind test: {sc['team']} — {sc['test']}", "",
             f"Sealed {sc['sealed_utc']}, record sha256 `{sc['record_sha256']}`. Predictions were sealed before the "
             "test and are not refitted.", "",
             f"- Absolute: {s['absolute_n']} values, mean abs. error "
             + ("n/a" if s["absolute_mae_pct"] is None else format(s["absolute_mae_pct"], ".1f") + " %")
             + f", {s['absolute_within_5pct']} within 5 %.",
             f"- A vs B: right direction in {s['ab_sign_right']} of {s['ab_decided']} decided comparisons "
             f"({s['ab_ties']} measured ties left out).",
             f"- Suspension: {s['angles_n']} camber/toe values"
             + ("" if s["angles_max_abs_error_deg"] is None else
                f", largest error {s['angles_max_abs_error_deg']:.2f} deg, {s['angles_within_tolerance']} of "
                f"{s['angles_with_tolerance']} within the stated tolerance") + ".", "",
             "| Setup | Channel | Predicted | Measured | Error |", "|---|---|---|---|---|"]
    lines += [f"| {r['setup']} | {r['channel']} | {r['predicted']} | {r['measured']} | {r['error_pct']:+.1f} % |"
              for r in sc["absolute"]]
    if sc["angles"]:
        lines += ["", "| Setup | Channel | Predicted (deg) | Measured (deg) | Error (deg) | Within tolerance |",
                  "|---|---|---|---|---|---|"]
        for r in sc["angles"]:
            w = "no tolerance set" if r["within"] is None else ("yes" if r["within"] else "NO")
            lines.append(f"| {r['setup']} | {r['channel']} | {r['predicted']:+.3f} | {r['measured']:+.3f} | "
                         f"{r['error_deg']:+.3f} | {w} |")
    lines += ["", "| Pair | Channel | Predicted change | Measured change | Direction | Size error |",
              "|---|---|---|---|---|---|"]
    for r in sc["ab"]:
        d = "tie (within noise)" if r["tie"] else ("right" if r["sign_right"] else "WRONG")
        se = "" if r["size_error_pct"] is None else format(r["size_error_pct"], "+.0f") + " %"
        lines.append(f"| {r['pair']} | {r['channel']} | {r['predicted_delta']:+.4f} | {r['measured_delta']:+.4f} | "
                     f"{d} | {se} |")
    return "\n".join(lines) + "\n"


def write_json(obj, path) -> None:
    """Write a record or scorecard as JSON (UTF-8)."""
    Path(path).write_text(json.dumps(obj, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
