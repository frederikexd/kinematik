# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/express_jobs.py — the Express Lane's second tranche of jobs
# ============================================================================
"""
Everything here is a `register_job` call and a function. Nothing in this module
knows about parsing, sniffing, planning, bundling or Streamlit — that is the
whole point of the registry: adding a tool to the express lane is a local edit
that cannot break the pipeline around it.

Two families:

  MODEL JOBS run on the parsed sentence alone. They need no upload, so they
  fire whenever the grammar recognises their tool word. Lap times, the g-g-v
  envelope, corner compliance, EV energy, anti-geometry, the rules-driven
  chassis hardware checks.

  DATA JOBS run on what was dropped. They declare the channels they need and
  are activated by the presence of those channels, asked for or not — a damper
  pot in the log gets you a velocity histogram whether or not anyone typed the
  word "damper". These are the ones that turn a log into an answer instead of
  a plot: measured brake bias against the declared one, measured roll gradient
  against the model's, measured pack energy against the endurance requirement.

Every job obeys the same three rules as the first tranche:
  * it declares its screening fidelity IN the report, not in a docstring;
  * it names what it assumed when the member did not say;
  * it fails into the bundle, never out of it (the runner handles that).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from .express import (JOBS, Artifact, Ctx, Job, register_job, _csv_bytes,
                      _fnum, _md)


# =========================================================================== #
#  MODEL JOBS
# =========================================================================== #
def _veh(ctx: Ctx):
    """The vehicle the sentence described, with every unstated value taken
    from the declared defaults — built once, the same way, for every job."""
    from .kinematics import SuspensionKinematics
    from .dynamics import VehicleDynamics, VehicleParams
    kin = SuspensionKinematics(ctx.hardpoints)
    p = VehicleParams(
        mass=float(ctx.ask.param("mass_kg", 280.0)),
        cg_height=float(ctx.ask.param("cg_height_mm", 300.0)),
        wheelbase=float(ctx.ask.param("wheelbase_mm", 1550.0)),
        track_front=float(ctx.ask.param("track_front_mm", 1200.0)),
        track_rear=float(ctx.ask.param("track_rear_mm", 1180.0)),
        weight_dist_front=float(ctx.ask.param("weight_dist_front", 0.47)),
        mu_peak=float(ctx.ask.param("mu_peak", 1.55)))
    return kin, p, VehicleDynamics(p, front_kin=kin, rear_kin=kin)


# --- anti-dive / anti-squat -------------------------------------------------- #
def _job_anti(ctx: Ctx) -> List[Artifact]:
    kin, p, _vd = _veh(ctx)
    bias = float(ctx.ask.param("brake_bias_front", 0.62))
    travels = np.linspace(-25.0, 25.0, 11)
    rows = []
    for t in travels:
        st = kin.solve_at_travel(float(t))
        rows.append((float(t),
                     float(kin.anti_dive_pct(p.cg_height, p.wheelbase,
                                             bias, state=st)),
                     float(kin.anti_squat_pct(p.cg_height, p.wheelbase,
                                              1.0, state=st)),
                     float(kin.side_view_swing_arm_length(state=st))))
    ad0 = float(kin.anti_dive_pct(p.cg_height, p.wheelbase, bias))
    as0 = float(kin.anti_squat_pct(p.cg_height, p.wheelbase, 1.0))
    svsa = float(kin.side_view_swing_arm_length())
    swing = max(r[1] for r in rows) - min(r[1] for r in rows)

    md = _md("Anti-dive, anti-squat and the side-view swing arm", [
        f"Brake bias **{bias:.0%} front** · cg **{p.cg_height:g} mm** · "
        f"wheelbase **{p.wheelbase:g} mm**.",
        "",
        "| quantity | at ride |",
        "|---|---|",
        f"| anti-dive | **{_fnum(ad0)} %** |",
        f"| anti-squat | **{_fnum(as0)} %** |",
        f"| side-view swing-arm length | {_fnum(svsa)} mm |",
        "",
        f"Anti-dive moves **{_fnum(swing)} percentage points** across ±25 mm "
        "of travel. A large swing means the car's pitch behaviour under "
        "braking changes with ride height — which is why it feels different "
        "on a fresh set of springs.",
        "",
        "Anti-geometry is a **percentage of the load transfer taken through "
        "the links instead of the springs**. It is not free: every point of "
        "anti- is a point of harshness fed into the chassis, and 100 % "
        "anti-dive is a car that will not use its front tyres over a kerb.",
        "",
        "*Both figures depend on the brake bias you gave. Change the bias in "
        "your sentence and anti-dive moves with it — anti-squat does not.*",
    ])
    csvb = _csv_bytes(["travel_mm", "anti_dive_pct", "anti_squat_pct",
                       "svsa_length_mm"], rows)
    return [Artifact("kinematics/anti_geometry.md", md, "md"),
            Artifact("kinematics/anti_geometry.csv", csvb, "csv")]


register_job(Job("anti_geometry", "Anti-dive / anti-squat", "kinematics",
                 _job_anti))


# --- corner compliance ------------------------------------------------------- #
def _job_compliance(ctx: Ctx) -> List[Artifact]:
    from . import compliance as cp
    _kin, _p, vd = _veh(ctx)
    g_lat = float(ctx.ask.param("lateral_g", 1.4))
    corner = cp.CompliantCorner.uniform_tube(ctx.hardpoints)
    load = cp.corner_wheel_load(vd, "front", g_lat, outer=True)
    res = corner.solve(load)

    d_camber = float(res.compliant.camber) - float(res.rigid.camber)
    d_toe = float(res.compliant.toe) - float(res.rigid.toe)
    rows = sorted(
        ((m, float(res.member_forces.get(m, float("nan"))),
          float(res.member_deflection.get(m, float("nan"))),
          float(res.member_stiffness.get(m, float("nan"))))
         for m in res.member_forces),
        key=lambda r: -abs(r[2] if np.isfinite(r[2]) else 0.0))
    worst = rows[0] if rows else None

    md = _md("Corner compliance — which member is actually the constraint", [
        f"Outer front corner at **{g_lat:g} g** lateral. Wheel load "
        f"Fy **{_fnum(load.Fy)} N**, Fz **{_fnum(load.Fz)} N**.",
        "",
        "| | rigid | compliant | Δ |",
        "|---|---|---|---|",
        f"| camber (deg) | {_fnum(res.rigid.camber)} | "
        f"{_fnum(res.compliant.camber)} | **{d_camber:+.4f}** |",
        f"| toe (deg) | {_fnum(res.rigid.toe)} | "
        f"{_fnum(res.compliant.toe)} | **{d_toe:+.4f}** |",
        "",
        "### Member by member, worst deflection first",
        "",
        "| member | force (N) | deflection (mm) | axial stiffness (N/mm) |",
        "|---|---|---|---|",
    ] + [f"| {m} | {_fnum(f)} | {_fnum(d)} | {_fnum(k)} |"
         for m, f, d, k in rows] + [
        "",
        (f"**{worst[0]} is the constraint** — it deflects "
         f"{abs(worst[2]):.3f} mm, more than any other member in the corner. "
         "That is where a stiffness change actually buys you something; "
         "stiffening anything else first is money spent on the wrong tube."
         if worst else ""),
        "",
        f"Compliance is eating **{abs(d_camber):.3f} deg of camber** and "
        f"**{abs(d_toe):.3f} deg of toe** at this load. Compare that against "
        "your camber-curve target before you go chasing the target with "
        "geometry — a rigid-body kinematics fix cannot recover an angle the "
        "structure is giving away.",
        "",
        f"*Converged in {res.iterations} iterations "
        f"({'converged' if res.converged else '**did not converge**'}). "
        "Members are the default uniform 4130 tube set — load your real tube "
        "sizes in the 🧬 Compliance tab before quoting these deflections.*",
    ])
    csvb = _csv_bytes(["member", "force_N", "deflection_mm",
                       "axial_stiffness_N_per_mm"], rows)
    return [Artifact("compliance/corner_compliance.md", md, "md"),
            Artifact("compliance/member_forces.csv", csvb, "csv")]


register_job(Job("compliance_corner", "Corner compliance and member loads",
                 "compliance", _job_compliance))


# --- lap / event times ------------------------------------------------------- #
def _job_lap_events(ctx: Ctx) -> List[Artifact]:
    from . import lapsim
    _kin, p, vd = _veh(ctx)
    params = lapsim.LapSimParams(
        power_w=float(ctx.ask.param("power_kw", 60.0)) * 1000.0,
        mass=p.mass)
    laps = int(round(float(ctx.ask.param("endurance_laps", 22.0))))
    res = lapsim.simulate_events(vd, params, endurance_laps=1)

    rows, lines = [], []
    for name in sorted(res):
        r = res[name]
        rows.append((name, float(getattr(r, "lap_time", float("nan"))),
                     float(getattr(r, "event_time", float("nan"))),
                     float(getattr(r, "avg_speed", float("nan"))),
                     float(getattr(r, "top_speed", float("nan"))),
                     bool(getattr(r, "ok", True))))
    auto = res.get("autocross")
    endurance_s = (float(auto.lap_time) * laps
                   if auto is not None else float("nan"))

    md = _md("Event times — the whole FSAE dynamic weekend", [
        f"Vehicle: **{p.mass:g} kg**, **{ctx.ask.param('power_kw', 60.0):g} "
        f"kW**, μ_peak {p.mu_peak:g}, cg {p.cg_height:g} mm.",
        "",
        "| event | lap (s) | event (s) | avg speed (m/s) | top (m/s) |",
        "|---|---|---|---|---|",
    ] + [f"| {n} | {_fnum(lt)} | {_fnum(et)} | {_fnum(av)} | {_fnum(tp)} |"
         for n, lt, et, av, tp, _ok in rows] + [
        "",
        f"At the autocross lap time, **{laps} endurance laps is "
        f"{_fnum(endurance_s)} s** ({_fnum(endurance_s / 60.0)} min) of "
        "driving before any driver-change or traffic allowance.",
        "",
        "These are **point-mass quasi-steady** times: a g-g-v envelope walked "
        "along a synthetic track. They are excellent for ranking two setups "
        "against each other and poor for predicting an absolute time on a "
        "track you have not modelled. Use the delta, not the number.",
        "",
        "*Take a real track into the 🏁 Track Testing tab — it accepts cone "
        "coordinates and GPS paths, and will optimise a racing line against "
        "the centreline.*",
    ])
    csvb = _csv_bytes(["event", "lap_time_s", "event_time_s",
                       "avg_speed_ms", "top_speed_ms", "ok"], rows)
    return [Artifact("laptime/event_times.md", md, "md"),
            Artifact("laptime/event_times.csv", csvb, "csv")]


register_job(Job("lap_events", "Skidpad / acceleration / autocross times",
                 "laptime", _job_lap_events))


# --- g-g-v envelope ---------------------------------------------------------- #
def _job_ggv(ctx: Ctx) -> List[Artifact]:
    from . import ggv
    _kin, p, _vd = _veh(ctx)
    res = ggv.quick_ggv(mass=p.mass, cg_height=p.cg_height,
                        weight_dist_front=p.weight_dist_front,
                        track_front=p.track_front, track_rear=p.track_rear,
                        wheelbase=p.wheelbase)
    speeds = np.asarray(res.speeds, float)
    lat = np.asarray(res.long_g, float)          # (n_speed, n_theta)
    lng = np.asarray(res.lat_g, float)
    #  the grid orientation varies with the generator's conventions, so the
    #  per-speed envelope is taken as the max magnitude on each axis rather
    #  than assumed — a wrong transpose would otherwise be invisible.
    grid = np.asarray(res.theta, float)
    rows = []
    for i, v in enumerate(speeds):
        row_lat = float(np.nanmax(np.abs(grid[i]))) if grid.ndim == 2 else float("nan")
        rows.append((float(v),
                     float(np.asarray(res.max_lat_g, float).ravel()[i]
                           if np.asarray(res.max_lat_g).size > i
                           else float("nan")),
                     float(np.asarray(res.max_accel_g, float).ravel()[i]
                           if np.asarray(res.max_accel_g).size > i
                           else float("nan")),
                     float(np.asarray(res.max_brake_g, float).ravel()[i]
                           if np.asarray(res.max_brake_g).size > i
                           else float("nan")),
                     row_lat))
    mlat = np.asarray(res.max_lat_g, float).ravel()

    md = _md("The g-g-v envelope", [
        f"Grip model: **{res.grip_model}** · {speeds.size} speed stations "
        f"from {speeds.min():.0f} to {speeds.max():.0f} m/s.",
        "",
        f"- Peak lateral, low speed: **{_fnum(mlat[0])} g**",
        f"- Peak lateral, top speed: **{_fnum(mlat[-1])} g** "
        f"({'aero is doing real work' if mlat[-1] > mlat[0] * 1.05 else 'flat — no aero in this model'})",
        "",
        "| speed (m/s) | max lat (g) | max accel (g) | max brake (g) |",
        "|---|---|---|---|",
    ] + [f"| {v:.0f} | {_fnum(a)} | {_fnum(b)} | {_fnum(c)} |"
         for v, a, b, c, _d in rows] + [
        "",
        "The envelope is what the lap sim walks. If a measured log sits well "
        "inside it (see `validation/measured_vs_model.md` when you drop a "
        "log), the gap is driver, tyre temperature, or an envelope built on "
        "optimistic μ — in roughly that order of likelihood.",
        "",
    ] + [f"- ⚠️ {w}" for w in (res.warnings or [])] + [
        "",
        "*Quasi-steady: no transient weight transfer, no tyre relaxation "
        "length, no thermal model.*",
    ])
    csvb = _csv_bytes(["speed_ms", "max_lat_g", "max_accel_g", "max_brake_g"],
                      [(a, b, c, d) for a, b, c, d, _e in rows])
    return [Artifact("tire/ggv_envelope.md", md, "md"),
            Artifact("tire/ggv_envelope.csv", csvb, "csv")]


register_job(Job("ggv_envelope", "g-g-v envelope", "tire", _job_ggv))


# --- EV energy and architecture --------------------------------------------- #
def _job_ev(ctx: Ctx) -> List[Artifact]:
    from . import ev_powertrain as evp, lapsim
    _kin, p, vd = _veh(ctx)
    ev_params = evp.EVParams(pack_energy_kwh=float(
        ctx.ask.param("pack_kwh", 6.5)))
    base = lapsim.LapSimParams(
        power_w=float(ctx.ask.param("power_kw", 60.0)) * 1000.0, mass=p.mass)
    laps_needed = int(round(float(ctx.ask.param("endurance_laps", 22.0))))
    track = lapsim.autocross_track(laps=1)
    sim = evp.EVLapSimulator(vd, base, ev_params)

    rows, warn = [], []
    for arch in evp.Powertrain:
        r = sim.run_architecture(arch, track)
        rows.append((arch.value,
                     float(r.lap_time),
                     float(r.energy_per_lap_kwh),
                     float(r.regen_recovered_kwh),
                     float(r.laps_until_empty),
                     float(r.effective_mass_kg),
                     bool(r.finishes_event)))
        warn.extend(r.warnings or [])
    best = min(rows, key=lambda r: r[1])
    longest = max(rows, key=lambda r: r[4])

    md = _md("EV energy — three architectures on one pack", [
        f"Pack **{ev_params.pack_energy_kwh:g} kWh** · "
        f"**{ctx.ask.param('power_kw', 60.0):g} kW** · car {p.mass:g} kg · "
        f"endurance target **{laps_needed} laps**.",
        "",
        "| architecture | lap (s) | kWh/lap | regen (kWh) | laps until "
        "empty | effective mass (kg) |",
        "|---|---|---|---|---|---|",
    ] + [f"| {a} | {_fnum(lt)} | {_fnum(e)} | {_fnum(rg)} | {_fnum(lp)} | "
         f"{_fnum(m)} |" for a, lt, e, rg, lp, m, _f in rows] + [
        "",
        f"- Fastest lap: **{best[0]}** at {best[1]:.2f} s",
        f"- Longest range: **{longest[0]}** at {longest[4]:.1f} laps",
        "",
        (f"⚠️ **The pack does not cover the {laps_needed}-lap target** on "
         f"any architecture — the best is {longest[4]:.1f} laps. Either the "
         "pack grows, the target shrinks, or the driver lifts."
         if longest[4] < laps_needed else
         f"✅ Every architecture covers the {laps_needed}-lap target; the "
         f"margin on the best is {longest[4] - laps_needed:.1f} laps."),
        "",
        "Torque vectoring's lap-time benefit is **flagged, not modelled** by "
        "this sim — it appears as a declared yaw benefit rather than an "
        "emergent one, so treat any four-motor advantage here as an "
        "assumption you still owe evidence for.",
        "",
    ] + [f"- ⚠️ {w}" for w in dict.fromkeys(warn)],)
    csvb = _csv_bytes(["architecture", "lap_time_s", "kwh_per_lap",
                       "regen_kwh", "laps_until_empty", "effective_mass_kg",
                       "finishes_event"], rows)
    return [Artifact("ev/energy_architectures.md", md, "md"),
            Artifact("ev/energy_architectures.csv", csvb, "csv")]


register_job(Job("ev_energy", "EV energy and architecture comparison", "ev",
                 _job_ev))


# --- rules-driven chassis hardware ------------------------------------------- #
def _rulebook_footer() -> str:
    """One line, generated from the loaded ruleset rather than written by
    hand, so it cannot drift out of date the way a hardcoded year does."""
    from . import rules_fsae as rf
    status = ("a **DRAFT for public comment**, not valid for any competition"
              if not rf.RULESET.binding else "the published ruleset")
    return (f"**Nothing here substitutes for the rulebook.** This toolkit "
            f"currently carries {rf.RULESET.label()}, which is {status}. "
            f"Rule numbers and thresholds move year to year — check these "
            f"loads against your season's actual text before you cut tube, "
            f"and see `rules/declared_check.md` for what the encoded subset "
            f"could and could not verify.")


def _job_chassis_rules(ctx: Ctx) -> List[Artifact]:
    from . import tubeframe as tf
    driver = 77.0
    harness = tf.harness_attachment_loads(driver_mass_kg=driver)
    seat = tf.seat_mount_check(seat_mass_kg=5.0, driver_mass_kg=driver)

    pts = harness.get("points", [])
    md = _md("Harness and seat mounting — the loads the rules imply", [
        f"Driver **{driver:g} kg** at **{harness.get('decel_g', 20):g} g** "
        f"deceleration → total restraint load "
        f"**{_fnum(harness.get('F_total_N'))} N**.",
        "",
        "| point | count | belt tension (N) | mounts to |",
        "|---|---|---|---|",
    ] + [f"| {p.get('point','')} | {p.get('n','')} | "
         f"{_fnum(p.get('belt_tension_N'))} | {p.get('mounts_to','')} |"
         for p in pts] + [
        "",
        "### Seat mounting",
        "",
        f"- Combined seat + driver: **{_fnum(seat.get('combined_mass_kg'))} "
        f"kg**",
        f"- Resultant: **{_fnum(seat.get('resultant_g'))} g** → "
        f"**{_fnum(seat.get('F_resultant_N'))} N** across "
        f"{seat.get('n_mounts', 4)} mounts",
        f"- Per mount: **{_fnum(seat.get('load_per_mount_N'))} N**",
        f"- Selected fastener: **{seat.get('chosen', {}).get('name', '—')}**",
        "",
        "These are the loads the restraint rules imply, not a stress "
        "analysis of your bracket. They tell you what the bracket has to "
        "survive; the 🧬 Compliance and bracket tabs tell you whether yours "
        "does.",
        "",
        _rulebook_footer(),
    ])
    rows = [(p.get("point", ""), p.get("n", ""),
             p.get("belt_tension_N", float("nan")), p.get("mounts_to", ""))
            for p in pts]
    csvb = _csv_bytes(["point", "count", "belt_tension_N", "mounts_to"], rows)
    return [Artifact("frames/harness_and_seat.md", md, "md"),
            Artifact("frames/harness_loads.csv", csvb, "csv")]


register_job(Job("chassis_rules", "Harness and seat mounting loads",
                 "frames", _job_chassis_rules))


# =========================================================================== #
#  DATA JOBS
# =========================================================================== #
_DAMPERS = ("damper_fl", "damper_fr", "damper_rl", "damper_rr")


def _job_damper_histogram(ctx: Ctx) -> List[Artifact]:
    """The damper-velocity histogram: the single most-read plot in paddock
    data analysis, and the one most often produced with the wrong sign
    convention. Bump is defined positive here and said so."""
    db = ctx.data
    t = db.series["time"]
    ok = np.isfinite(t)
    t = t[ok]
    present = [c for c in _DAMPERS if c in db.series]
    edges = np.array([-250, -150, -100, -50, -25, -10, 10, 25, 50, 100,
                      150, 250], float)
    rows, lines = [], []
    for c in present:
        y = db.series[c][ok]
        good = np.isfinite(y) & np.isfinite(np.concatenate(([np.nan],
                                                            np.diff(y))))
        v = np.gradient(np.nan_to_num(y, nan=float(np.nanmean(y))), t)
        v = v[np.isfinite(v)]
        if v.size == 0:
            continue
        hist, _ = np.histogram(v, bins=edges)
        frac = hist / max(hist.sum(), 1)
        lowspeed = float(np.sum(np.abs(v) <= 25.0) / v.size)
        rows.append((c, float(np.percentile(v, 1)), float(np.percentile(v, 99)),
                     float(np.max(np.abs(v))), lowspeed))
        lines.append(f"| {db.channels[c].label} | "
                     f"{_fnum(np.percentile(v, 1))} | "
                     f"{_fnum(np.percentile(v, 99))} | "
                     f"{_fnum(np.max(np.abs(v)))} | {lowspeed:.1%} |")

    hist_rows = []
    for c in present:
        y = db.series[c][ok]
        v = np.gradient(np.nan_to_num(y, nan=float(np.nanmean(y))), t)
        v = v[np.isfinite(v)]
        hist, _ = np.histogram(v, bins=edges)
        for i in range(len(edges) - 1):
            hist_rows.append((c, float(edges[i]), float(edges[i + 1]),
                              int(hist[i]),
                              float(hist[i] / max(hist.sum(), 1))))

    md = _md("Damper velocity histogram", [
        f"Channels: {', '.join(db.channels[c].label for c in present)} · "
        f"differentiated against the {_fnum(db.sample_rate_hz)} Hz timebase.",
        "",
        "**Sign convention: positive is bump** (damper closing). If your pot "
        "is wired the other way the histogram is mirrored and every reading "
        "below flips — check one known kerb strike before trusting it.",
        "",
        "| channel | 1st pct (mm/s) | 99th pct (mm/s) | peak |mm/s| | "
        "% time low-speed (≤25 mm/s) |",
        "|---|---|---|---|---|",
    ] + lines + [
        "",
        "The number to read first is the **low-speed fraction**. A car "
        "spending well over 80 % of its time under 25 mm/s is a car whose "
        "handling lives entirely in the low-speed knee — high-speed clicks "
        "will not fix a balance problem, no matter how many you turn.",
        "",
        "A histogram badly skewed to one side means the damper is not "
        "working symmetrically about ride height: check the platform and the "
        "spring preload before you touch a valve.",
        "",
        "*Velocity is a numerical gradient of the pot signal — at low sample "
        "rates or with an unfiltered pot, the tails of this histogram are "
        "differentiation noise, not the road. Full binning and filtering "
        "live in the 🎛️ Setup Optimiser.*",
    ])
    csvb = _csv_bytes(["channel", "bin_low_mm_s", "bin_high_mm_s", "count",
                       "fraction"], hist_rows)
    stats = _csv_bytes(["channel", "p01_mm_s", "p99_mm_s", "peak_abs_mm_s",
                        "lowspeed_fraction"], rows)
    return [Artifact("setup/damper_histogram.md", md, "md"),
            Artifact("setup/damper_histogram.csv", csvb, "csv"),
            Artifact("setup/damper_velocity_stats.csv", stats, "csv")]


register_job(Job("damper_histogram", "Damper velocity histogram", "setup",
                 _job_damper_histogram, needs_channels=("time",),
                 needs_any=_DAMPERS, data_activated=True))


# --- measured roll gradient -------------------------------------------------- #
def _job_roll_correlation(ctx: Ctx) -> List[Artifact]:
    """Measured roll gradient from the front damper pair against the model's.

    This is the cheapest correlation a team can run and almost nobody does,
    because it needs two channels and one line of geometry — both of which
    are already here.
    """
    db = ctx.data
    _kin, p, vd = _veh(ctx)
    ok = np.isfinite(db.series["time"])
    fl = db.series["damper_fl"][ok]
    fr = db.series["damper_fr"][ok]
    ay = db.series["ay"][ok]
    mr = float(ctx.ask.param("motion_ratio", 0.0)) or None
    if mr is None:
        try:
            mr = float(_kin.motion_ratio())
        except Exception:                                    # noqa: BLE001
            mr = 0.5
    #  wheel-travel difference across the axle → roll angle
    dw = (fl - fr) / max(mr, 1e-6)
    roll_deg = np.degrees(np.arctan2(dw, p.track_front))
    good = np.isfinite(roll_deg) & np.isfinite(ay)
    roll_deg, ay_g = roll_deg[good], ay[good]
    if roll_deg.size < 20:
        grad_meas = float("nan")
        r2 = float("nan")
    else:
        A = np.vstack([ay_g, np.ones_like(ay_g)]).T
        coef, *_ = np.linalg.lstsq(A, roll_deg, rcond=None)
        grad_meas = float(coef[0])
        pred = A @ coef
        ss_res = float(np.sum((roll_deg - pred) ** 2))
        ss_tot = float(np.sum((roll_deg - roll_deg.mean()) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    g_ref = 1.0
    _loads, detail = vd.lateral_load_transfer(g_ref)
    grad_model = float(detail.get("roll_angle", float("nan"))) / g_ref
    ratio = (grad_meas / grad_model
             if grad_model and np.isfinite(grad_model) else float("nan"))

    if not np.isfinite(ratio):
        verdict = "Not enough usable samples to fit a gradient."
    elif ratio > 1.25:
        verdict = ("The car rolls **more** than the model says it should by "
                   "over 25 %. The usual culprits, in order: the roll "
                   "stiffnesses in the model are the design values rather "
                   "than the built ones, the ARB is not actually engaging, "
                   "or chassis torsion is soft enough to be part of the roll "
                   "path.")
    elif ratio < 0.80:
        verdict = ("The car rolls **less** than modelled. Check the motion "
                   "ratio assumption first — it scales this measurement "
                   "linearly and is the most common source of the error.")
    else:
        verdict = ("Measured and modelled roll gradients agree within 20 %. "
                   "That is a correlated roll model, which means the load "
                   "transfer numbers in `roll/load_transfer.md` are worth "
                   "believing.")

    md = _md("Roll gradient — measured against modelled", [
        "| source | roll gradient (deg/g) |", "|---|---|",
        f"| measured, least-squares fit on {roll_deg.size} samples | "
        f"**{_fnum(grad_meas)}** |",
        f"| model, at {g_ref:g} g | **{_fnum(grad_model)}** |",
        f"| ratio | **{_fnum(ratio, '{:.2f}')}** |",
        f"| fit quality R² | {_fnum(r2, '{:.3f}')} |",
        "",
        verdict,
        "",
        f"Measured roll is reconstructed from the front damper pair with a "
        f"motion ratio of **{mr:.3f}** and a front track of "
        f"**{p.track_front:g} mm**. Both of those are assumptions this job "
        "inherits — if the motion ratio is wrong by 10 %, so is the measured "
        "gradient.",
        "",
        (f"⚠️ R² is only {r2:.2f} — the fit is weak, so treat the gradient "
         "as indicative. A weak fit usually means the log has little "
         "sustained cornering, or the pots are picking up single-wheel bumps "
         "as roll." if np.isfinite(r2) and r2 < 0.6 else ""),
    ])
    csvb = _csv_bytes(["lateral_g", "roll_deg"],
                      [(float(a), float(b))
                       for a, b in zip(ay_g[:5000], roll_deg[:5000])])
    return [Artifact("validation/roll_correlation.md", md, "md"),
            Artifact("validation/roll_vs_ay.csv", csvb, "csv")]


register_job(Job("roll_correlation", "Measured vs modelled roll gradient",
                 "roll", _job_roll_correlation,
                 needs_channels=("time", "ay", "damper_fl", "damper_fr"),
                 data_activated=True))


# --- measured brake bias ----------------------------------------------------- #
def _bias_verdict(n_samples: int, delta: float) -> str:
    """Three outcomes, and "undefined" is one of them.

    The first version of this job printed "measured and declared agree" when
    the pressure channels were flat and the comparison had produced a NaN —
    a green verdict on no evidence at all. A comparison that could not be made
    must say so; silence or false reassurance is worse than a red flag,
    because nobody goes back to check a green one.
    """
    if n_samples < 20 or not np.isfinite(delta):
        return ("**No verdict.** Fewer than 20 samples cleared the pressure "
                "threshold, so there is nothing to fit. Either the car never "
                "braked in this log, or the pressure channels are dead — "
                "check the flags in `daq/telemetry_summary.md` before "
                "assuming the former.")
    if abs(delta) > 0.02:
        return (f"The car is running **{abs(delta):.1%} "
                f"{'more' if delta > 0 else 'less'} front bias** than the "
                "number your analysis assumes. Every lock-up order, every "
                "rotor thermal run and every anti-dive figure built on the "
                "declared value is off by that much.")
    return ("Measured and declared bias agree within 2 % — the pressure "
            "hardware is doing what the spreadsheet says it does.")


def _job_brake_bias(ctx: Ctx) -> List[Artifact]:
    db = ctx.data
    pf = db.series["brake_front"]
    pr = db.series["brake_rear"]
    good = np.isfinite(pf) & np.isfinite(pr)
    pf, pr = pf[good], pr[good]
    #  only pressures above a real threshold — the noise floor at zero pedal
    #  would otherwise dominate a ratio
    m = (pf > 0.05 * np.nanmax(pf)) if pf.size else np.array([], bool)
    pf_m, pr_m = pf[m], pr[m]
    if pf_m.size >= 20:
        slope = float(np.linalg.lstsq(pr_m[:, None], pf_m, rcond=None)[0][0])
        bias_meas = slope / (1.0 + slope)
    else:
        slope, bias_meas = float("nan"), float("nan")
    declared = float(ctx.ask.param("brake_bias_front", 0.62))
    delta = bias_meas - declared

    md = _md("Brake bias — measured from the pressure traces", [
        f"Samples above 5 % of peak front pressure: **{pf_m.size}** of "
        f"{pf.size}.",
        "",
        "| quantity | value |", "|---|---|",
        f"| front/rear pressure slope | {_fnum(slope)} |",
        f"| measured front bias | **{_fnum(bias_meas, '{:.1%}')}** |",
        f"| bias you declared | {declared:.1%} |",
        f"| difference | **{delta:+.1%}** |",
        "",
        _bias_verdict(pf_m.size, delta),
        "",
        "**This is a pressure bias, not a torque bias.** Converting it needs "
        "piston areas, rotor radii and pad μ, which the 🛑 Brakes tab has "
        "and this job does not. Two cars with identical pressure bias can "
        "have very different torque bias.",
        "",
        "*Fitted through the origin: a pedal with a residual offset will "
        "bend this slope. Check that both traces read zero at rest.*",
    ])
    csvb = _csv_bytes(["front_pressure", "rear_pressure"],
                      [(float(a), float(b))
                       for a, b in zip(pf_m[:5000], pr_m[:5000])])
    return [Artifact("brakes/measured_bias.md", md, "md"),
            Artifact("brakes/pressure_pairs.csv", csvb, "csv")]


register_job(Job("brake_bias_measured", "Measured brake bias", "brakes",
                 _job_brake_bias,
                 needs_channels=("brake_front", "brake_rear"),
                 data_activated=True))


# --- measured pack energy ---------------------------------------------------- #
def _job_energy_from_log(ctx: Ctx) -> List[Artifact]:
    from . import earshot
    db = ctx.data
    ok = np.isfinite(db.series["time"])
    t = db.series["time"][ok]
    v = db.series["pack_v"][ok]
    i = db.series["pack_i"][ok]
    good = np.isfinite(v) & np.isfinite(i)
    t, v, i = t[good], v[good], i[good]
    p_w = v * i
    #  trapezoid over the real timebase, not a nominal dt
    e_j = float(np.trapezoid(p_w, t)) if t.size > 1 else float("nan")
    e_kwh = e_j / 3.6e6
    duration = float(t[-1] - t[0]) if t.size > 1 else float("nan")

    laps_logged = float(ctx.ask.param("logged_laps", 0.0)) or None
    if laps_logged is None:
        #  no lap count given: report per-minute and say so rather than
        #  inventing a lap length
        per_lap = float("nan")
        lap_note = ("No lap count given, so energy per lap cannot be formed. "
                    "Add 'this log is 4 laps' to your sentence and this job "
                    "will divide it for you.")
    else:
        per_lap = e_kwh / laps_logged
        lap_note = (f"Divided by the {laps_logged:g} laps you declared → "
                    f"**{per_lap:+.4f} kWh/lap**.")

    pack_kwh = float(ctx.ask.param("pack_kwh", 6.5))
    target = int(round(float(ctx.ask.param("endurance_laps", 22.0))))
    if not np.isfinite(per_lap):
        range_line = ("Range needs energy per lap, which needs a lap count. "
                      "This job has neither invented one nor guessed a lap "
                      "length from the speed trace.")
    elif per_lap <= 0:
        range_line = (
            f"**Net energy over this log is {e_kwh:+.4f} kWh — negative.** "
            "More was recovered than drawn, which no traction segment does. "
            "Either the current sign convention is inverted, or this log is "
            "a coast-down or a tow. Fix the sign before reading anything "
            "else in this file; every number above inherits it.")
    else:
        laps = earshot.laps_from_pack(pack_kwh, 0.92, per_lap, 0.0)
        margin = laps - target
        range_line = (
            f"At **{per_lap:.4f} kWh/lap** measured, a **{pack_kwh:g} kWh** "
            f"pack at 92 % usable supports **{laps} laps** — against a "
            f"{target}-lap target, a margin of **{margin:+d} laps**."
            + ("  ⚠️ That does not cover the target."
               if margin < 0 else ""))

    md = _md("Pack energy — integrated from your own log", [
        f"**{t.size}** samples over **{_fnum(duration)} s**.",
        "",
        "| quantity | value |", "|---|---|",
        f"| mean bus power | {_fnum(float(np.nanmean(p_w)) / 1000.0)} kW |",
        f"| peak bus power | {_fnum(float(np.nanmax(p_w)) / 1000.0)} kW |",
        f"| peak regen | {_fnum(float(np.nanmin(p_w)) / 1000.0)} kW |",
        f"| net energy over the log | **{_fnum(e_kwh)} kWh** |",
        f"| mean pack voltage | {_fnum(float(np.nanmean(v)))} V |",
        f"| voltage sag (max − min) | {_fnum(float(np.nanmax(v) - np.nanmin(v)))} V |",
        "",
        lap_note,
        "",
        range_line,
        "",
        "This is **measured** energy, which beats every modelled figure in "
        "this bundle for the same quantity. Where `ev/energy_architectures."
        "md` disagrees with this file, believe this file and go find out why "
        "the model is optimistic.",
        "",
        "*Net of regen: the integral is signed, so recovered energy is "
        "already subtracted. Trapezoid integration on the raw timebase — no "
        "resampling, no filtering.*",
    ])
    n = min(t.size, 5000)
    csvb = _csv_bytes(["time_s", "pack_v", "pack_i", "power_w"],
                      [(float(t[k]), float(v[k]), float(i[k]), float(p_w[k]))
                       for k in range(n)])
    return [Artifact("ev/measured_energy.md", md, "md"),
            Artifact("ev/power_trace.csv", csvb, "csv")]


register_job(Job("energy_from_log", "Measured pack energy and range", "ev",
                 _job_energy_from_log,
                 needs_channels=("time", "pack_v", "pack_i"),
                 data_activated=True))


# =========================================================================== #
#  THE REST OF THE TOOLKIT
#  Costs below are DECLARED, measured once on a reference machine and written
#  into the source. They drive admission (see `plan`), so a wrong one is a
#  bug: `ExpressRun.timings` carries the measured value so it can be
#  corrected here rather than guessed at forever.
# =========================================================================== #

# --- aero -------------------------------------------------------------------- #
def _job_aero(ctx: Ctx) -> List[Artifact]:
    from .aero import AeroProvider
    _kin, p, _vd = _veh(ctx)
    prov = AeroProvider(reference_area_m2=1.0)
    speeds = np.array([10, 15, 20, 25, 30, 35, 40], float)
    rho = 1.225
    rows = []
    for v in speeds:
        cla, cda = prov.cla_cda(float(v)) if hasattr(prov, "cla_cda") \
            else (prov.fallback_cl_a, prov.fallback_cd_a)
        q = 0.5 * rho * v * v
        lift, drag = q * float(cla), q * float(cda)
        rows.append((float(v), float(cla), float(cda), lift, drag,
                     lift / drag if drag else float("nan"),
                     lift / (p.mass * 9.80665)))
    v_eq = float(np.interp(1.0, [r[6] for r in rows], speeds)) \
        if rows[-1][6] >= 1.0 else float("nan")

    mapped = bool(getattr(prov, "is_mapped", False))
    source = ("a correlated aero map" if mapped else
              "declared fallback coefficients — no map is loaded")
    md = _md("Aero — downforce, drag and what it is worth", [
        f"Coefficients come from **{source}** "
        f"(ClA {prov.fallback_cl_a:g}, CdA {prov.fallback_cd_a:g}).",
        "",
        "| speed (m/s) | ClA | CdA | downforce (N) | drag (N) | L/D | "
        "DF / car weight |",
        "|---|---|---|---|---|---|---|",
    ] + [f"| {v:.0f} | {_fnum(cl)} | {_fnum(cd)} | {_fnum(l)} | {_fnum(d)} | "
         f"{_fnum(ld)} | {_fnum(f, '{:.2f}')} |"
         for v, cl, cd, l, d, ld, f in rows] + [
        "",
        (f"The car generates its own weight in downforce at about "
         f"**{v_eq:.0f} m/s**." if np.isfinite(v_eq) else
         "The car never reaches its own weight in downforce inside this "
         "speed range — which is normal for FSAE and worth saying out loud "
         "when someone proposes a bigger wing."),
        "",
        "**These are fallback coefficients unless a correlated map is "
        "loaded.** They are a placeholder shaped like an answer: fine for "
        "ranking a decision, useless as evidence. An aero number without a "
        "correlation behind it is the single easiest thing for a design "
        "judge to take apart.",
        "",
        "*Reference area 1.0 m² so ClA and CdA read directly. No ride-height "
        "or attitude sensitivity — the 🪂 Aero tab carries the map.*",
    ])
    csvb = _csv_bytes(["speed_ms", "ClA", "CdA", "downforce_N", "drag_N",
                       "L_over_D", "df_over_weight"], rows)
    return [Artifact("aero/aero_baseline.md", md, "md"),
            Artifact("aero/aero_vs_speed.csv", csvb, "csv")]


register_job(Job("aero_baseline", "Aero baseline vs speed", "aero",
                 _job_aero, cost_s=0.3))


# --- DFMEA -------------------------------------------------------------------- #
def _job_dfmea(ctx: Ctx) -> List[Artifact]:
    from . import dfmea
    records = dfmea.seed_rows()
    stats = dfmea.dashboard_stats(records)
    actions = dfmea.action_items(records)
    rows = []
    for r in records:
        row = dfmea.row_from_mapping(r)
        rpn = dfmea.compute_rpn(row.severity, row.occurrence, row.detection)
        rows.append((row.subsystem, row.item, row.failure_mode,
                     row.severity, row.occurrence, row.detection, rpn,
                     str(dfmea.classify_risk(row.severity, rpn))))
    rows.sort(key=lambda r: -r[6])

    md = _md("DFMEA — the risk register, sorted by RPN", [
        f"**{stats.total} rows** · by band: "
        + ", ".join(f"{k} {v}" for k, v in sorted(stats.by_band.items()))
        + ".",
        "",
        "| subsystem | item | failure mode | S | O | D | RPN | band |",
        "|---|---|---|---|---|---|---|---|",
    ] + [f"| {a} | {b} | {c} | {d} | {e} | {f} | **{g}** | {h} |"
         for a, b, c, d, e, f, g, h in rows[:20]] + [
        "",
        f"### {len(actions)} open action items",
        "",
    ] + [f"- {a.get('item', a)}" for a in actions[:12]] + [
        "",
        "**This is the seeded register, not yours.** It is here so the shape "
        "of the document exists before the review, and so nobody spends the "
        "night building a table instead of thinking about failure modes. "
        "Every row needs replacing with your car's.",
        "",
        "RPN is a **ranking**, not a measurement: severity 9 × occurrence 2 "
        "× detection 5 and severity 2 × occurrence 9 × detection 5 both give "
        "90, and only one of them can hurt somebody. Sort by RPN, then "
        "re-sort by severity and look again.",
    ])
    csvb = _csv_bytes(["subsystem", "item", "failure_mode", "severity",
                       "occurrence", "detection", "rpn", "band"], rows)
    return [Artifact("dfmea/risk_register.md", md, "md"),
            Artifact("dfmea/risk_register.csv", csvb, "csv")]


register_job(Job("dfmea_register", "DFMEA risk register", "dfmea",
                 _job_dfmea, cost_s=0.2))


# --- fusebox ------------------------------------------------------------------ #
def _job_fusebox(ctx: Ctx) -> List[Artifact]:
    from . import fusebox
    paths = fusebox.seed_paths()
    audits = [fusebox.audit_path(p) for p in paths]
    body = fusebox.render_fusebox_md(paths, audits)
    rows = []
    for path, audit in zip(paths, audits):
        leader = getattr(audit, "leader_key", "")
        for key, prob in sorted(getattr(audit, "probs", {}).items(),
                                key=lambda kv: -kv[1]):
            rows.append((path.key, path.label, key, float(prob),
                         str(getattr(audit, "verdict", "")),
                         key == leader))
    verdicts = {}
    for a in audits:
        verdicts[str(getattr(a, "verdict", "?"))] = \
            verdicts.get(str(getattr(a, "verdict", "?")), 0) + 1

    md = _md("Fusebox — what breaks first, and whether you chose it", [
        f"**{len(paths)} overload paths** audited · verdicts: "
        + ", ".join(f"{k} × {v}" for k, v in sorted(verdicts.items())) + ".",
        "",
        "A path where the first-failure probabilities are near-even is a "
        "**coin flip**: the car has no designated fuse on that path, and "
        "which part breaks in a curb strike is down to the tolerance stack "
        "on the day. That is a decision you have not made yet, being made "
        "for you at speed.",
        "",
        "---",
        "",
        body if isinstance(body, str) else str(body),
    ])
    csvb = _csv_bytes(["path_key", "path_label", "element", "p_first_fail",
                       "verdict", "is_leader"], rows)
    return [Artifact("fusebox/overload_paths.md", md, "md"),
            Artifact("fusebox/first_failure_probs.csv", csvb, "csv")]


register_job(Job("fusebox_audit", "Fusebox overload-path audit", "fusebox",
                 _job_fusebox, cost_s=0.3))


# --- PCB (data job: needs a board) -------------------------------------------- #
def _job_pcb(ctx: Ctx) -> List[Artifact]:
    from . import pcb_doctor as pd
    board = pd.parse_kicad_pcb(ctx.data.extras["kicad_pcb"])
    nets = getattr(board, "nets", {}) or {}
    rows, findings = [], []
    for nid in sorted(nets)[:200]:
        try:
            res = pd.analyze_net(board, nid)
        except Exception:                                    # noqa: BLE001
            continue
        rows.append((nid, str(nets.get(nid, "")),
                     float(res.get("current_a", float("nan"))),
                     float(res.get("min_width_mm", float("nan"))),
                     float(res.get("required_width_mm", float("nan"))),
                     str(res.get("verdict", ""))))
        for f in res.get("findings", []) or []:
            findings.append(f)
    tight = [r for r in rows
             if np.isfinite(r[3]) and np.isfinite(r[4]) and r[3] < r[4]]

    md = _md("PCB Doctor — trace widths against their current", [
        f"Board parsed: **{len(nets)} nets**, "
        f"{len(getattr(board, 'segments', []))} segments, "
        f"{len(getattr(board, 'vias', []))} vias.",
        "",
        f"**{len(tight)} nets carry a trace narrower than the current "
        f"needs.**" if tight else "No net is narrower than its current "
        "requires, on the currents this analysis could assign.",
        "",
        "| net | name | current (A) | min width (mm) | required (mm) | "
        "verdict |",
        "|---|---|---|---|---|---|",
    ] + [f"| {a} | {b} | {_fnum(c)} | {_fnum(d)} | {_fnum(e)} | {f} |"
         for a, b, c, d, e, f in (tight or rows)[:25]] + [
        "",
        "Currents are **assigned, not measured** — from the integration "
        "ledger where one exists and from defaults where it does not. A net "
        "whose real current you have never metered is a net whose width "
        "verdict is only as good as the guess behind it.",
        "",
        "*IPC-2221 external/internal area rules at the stated temperature "
        "rise. No thermal coupling between adjacent traces, no copper pour "
        "credit — both make this conservative, which is the right direction "
        "for a screening pass.*",
    ])
    csvb = _csv_bytes(["net_id", "net_name", "current_a", "min_width_mm",
                       "required_width_mm", "verdict"], rows)
    return [Artifact("pcb/trace_widths.md", md, "md"),
            Artifact("pcb/net_analysis.csv", csvb, "csv")]


register_job(Job("pcb_check", "PCB trace-width check", "pcb", _job_pcb,
                 needs_extra=("kicad_pcb",), data_activated=True,
                 cost_s=1.5))


# --- pack thermal (slow: chains a lap sim) ------------------------------------ #
def _spread_note(spread_c: float) -> str:
    """Commentary that answers the number, not the topic.

    A paragraph about why gradients matter, printed under a gradient of
    exactly zero, teaches the reader that the prose is decoration. If the
    model produced no spread, the honest thing is to say why.
    """
    if not np.isfinite(spread_c):
        return "The model returned no usable cell temperatures."
    if spread_c < 0.5:
        return ("**Every cell finished within half a degree of every other.** "
                "That is the model, not the pack: no fans are placed and the "
                "airflow field is uniform, so there is nothing here to make "
                "cells differ. The gradient is precisely what you came for, "
                "and it only appears once fans and duct geometry go in — "
                "which is the 🌡️ Pack Thermal tab's job, not this lane's.")
    if spread_c > 10.0:
        return (f"**A {spread_c:.1f} °C spread across the pack.** The coolest "
                "cells are being asked to do work the hottest ones can no "
                "longer do, and the BMS will derate on the hottest one — so "
                "this gradient, not the peak, is what limits the car.")
    return (f"Spread is {spread_c:.1f} °C, which is tight. Watch it across a "
            "full endurance run rather than one lap before calling it solved.")


def _job_pack_thermal(ctx: Ctx) -> List[Artifact]:
    from . import pack_thermal as pt, lapsim
    _kin, p, vd = _veh(ctx)
    lp = lapsim.LapSimParams(
        power_w=float(ctx.ask.param("power_kw", 60.0)) * 1000.0, mass=p.mass)
    lap = lapsim.LapSimulator(vd, lp).simulate(lapsim.autocross_track(laps=1))
    res = pt.simulate_pack_thermal(lap, lp)
    if not getattr(res, "ok", False):
        return [Artifact("thermal/pack_thermal.md", _md(
            "Pack thermal — no result", [
                "The thermal model did not converge on this lap. That is "
                "usually a lap result the solver could not read a current "
                "trace from, not a thermal problem.",
                "",
                "Warnings: " + "; ".join(getattr(res, "warnings", []) or
                                         ["(none reported)"]),
            ]), "md")]
    temps = np.asarray(res.final_temp_c, float)
    peak = float(np.nanmax(temps))
    md = _md("Pack thermal — one autocross lap", [
        f"Layout **{res.rows} × {res.cols}** cells.",
        "",
        f"- Hottest cell at the end of the lap: **{_fnum(peak)} °C**",
        f"- Coolest: {_fnum(float(np.nanmin(temps)))} °C · spread "
        f"**{_fnum(peak - float(np.nanmin(temps)))} °C**",
        f"- Cells over 60 °C: **{int(np.nansum(temps > 60.0))}**",
        "",
        _spread_note(peak - float(np.nanmin(temps))),
        "",
        "*One lap from cold. Endurance is twenty-two of these back to back, "
        "so treat this as the gradient's shape rather than the temperature "
        "you will actually see. The 🌡️ Pack Thermal tab runs the full "
        "event and will place fans against it.*",
    ])
    rows = [(r, c, float(temps[r][c]))
            for r in range(temps.shape[0]) for c in range(temps.shape[1])] \
        if temps.ndim == 2 else [(0, i, float(v)) for i, v in enumerate(temps)]
    return [Artifact("thermal/pack_thermal.md", md, "md"),
            Artifact("thermal/cell_temps.csv",
                     _csv_bytes(["row", "col", "final_temp_c"], rows), "csv")]


register_job(Job("pack_thermal", "Pack thermal over a lap", "thermal",
                 _job_pack_thermal, cost_s=3.0))


# --- SimulForge (slow) -------------------------------------------------------- #
def _job_simulforge(ctx: Ctx) -> List[Artifact]:
    from . import simulforge as sf
    _kin, _p, vd = _veh(ctx)
    res = sf.run_simulforge(vd, "step_steer")
    tr = getattr(res, "transient", res)
    fields = {}
    for name in ("peak_yaw_rate", "settling_time_s", "overshoot_pct",
                 "rise_time_s", "steady_yaw_rate", "peak_lateral_g"):
        v = getattr(tr, name, getattr(res, name, None))
        if v is not None:
            fields[name] = float(v) if np.isscalar(v) else v
    md = _md("SimulForge — transient step steer", [
        "A step-steer manoeuvre through the full mechatronic path: driver "
        "input, bus latency, actuator dynamics, then the vehicle.",
        "",
        "| quantity | value |", "|---|---|",
    ] + [f"| {k.replace('_', ' ')} | {_fnum(v)} |"
         for k, v in sorted(fields.items())] + [
        "",
        "The quasi-steady numbers elsewhere in this bundle cannot see any of "
        "this. A car with a good steady-state balance and a slow yaw "
        "response is a car drivers describe as 'lazy' and lap sims describe "
        "as 'fine' — this is the file that catches that disagreement.",
        "",
    ] + [f"- ⚠️ {w}" for w in (getattr(res, "warnings", []) or [])] + [
        "",
        "*Declared cost ~7 s, which is why it is a budgeted job rather than "
        "a free one. Degradation presets and the full manoeuvre set live in "
        "the ⚡🔩 SimulForge tab.*",
    ])
    return [Artifact("transient/step_steer.md", md, "md")]


register_job(Job("simulforge_step", "Transient step steer", "transient",
                 _job_simulforge, cost_s=7.0))


# --- OmniCore (deep) ---------------------------------------------------------- #
def _job_omnicore(ctx: Ctx) -> List[Artifact]:
    """The member's own sentence is already a mission brief — hand it over.

    OmniCore has its own deterministic mission grammar. Rather than
    translating between two receipts, the express lane passes the raw text
    straight through and prints BOTH receipts, so the member can see exactly
    what each grammar made of the same words.
    """
    from . import omnicore as oc
    mission = oc.parse_mission(ctx.ask.text or "minimise bump steer")
    res = oc.run_omnicore(ctx.hardpoints, mission, oc.OmniKnobs())
    pick = getattr(res, "pick", None) or getattr(res, "knee", None)
    configs = getattr(res, "configs", []) or []

    lines = [
        "OmniCore read your sentence with **its own** mission grammar — a "
        "second, independent parse of the same words. Where the two receipts "
        "disagree, the words were ambiguous, and that is worth knowing.",
        "",
        "### OmniCore's reading",
        "",
        f"- Manoeuvre: **{getattr(mission, 'maneuver', '—')}**",
        f"- Shop class: **{getattr(mission, 'shop', '—')}**",
    ]
    for c in getattr(mission, "consumed", [])[:10]:
        lines.append(f"- ✅ {c}")
    for a in getattr(mission, "assumptions", [])[:10]:
        lines.append(f"- ➖ {a}")
    if getattr(mission, "ignored", None):
        lines.append(f"- 🕳️ not understood: "
                     f"{', '.join(mission.ignored[:15])}")
    lines += ["", f"### Result — {len(configs)} configurations evaluated", ""]
    if pick is not None:
        for f in ("label", "score", "shift_mm", "note"):
            v = getattr(pick, f, None)
            if v is not None:
                lines.append(f"- {f}: **{v}**")
    for w in (getattr(res, "warnings", []) or [])[:10]:
        lines.append(f"- ⚠️ {w}")
    lines += [
        "",
        "This is a **knee pick on a Pareto front**, not an optimum. It is "
        "the configuration where giving up a little more on one axis starts "
        "costing a lot on another — a defensible place to stand, not the "
        "only one. The 🧠 OmniCore tab lets you walk the front yourself and "
        "pick a different knee, which is often the right answer once a human "
        "looks at what each axis actually costs to build.",
        "",
        "*Declared cost ~30 s. It ran because you asked for it by name and "
        "your budget covered it.*",
    ]
    return [Artifact("omnicore/mission_result.md",
                     _md("OmniCore — your sentence, optimised", lines), "md")]


register_job(Job("omnicore_mission", "OmniCore mission optimisation",
                 "omnicore", _job_omnicore, cost_s=30.0))


# --- ghost topology: every corner the data can reach --------------------------- #
_CORNERS = ("FL", "FR", "RL", "RR")
_CORNER_POT = {"FL": "damper_fl", "FR": "damper_fr",
               "RL": "damper_rl", "RR": "damper_rr"}


@dataclass
class GhostSweep:
    """The result of auditing every corner the data could reach.

    `worst_audit` is what MorphMesh consumes — the corner the car is actually
    struggling at, which on an asymmetric car is frequently not the one a
    symmetric model would have picked.
    """
    by_corner: Dict[str, object]
    summaries: Dict[str, dict]
    worst_corner: str
    worst_audit: object
    load_source: str
    notes: List[str]


def _corner_histories(ctx: Ctx, n_max: int = 240):
    """Per-corner (t, Fx, Fy, Fz) from the log — measured where the data
    allows it, modelled where it does not, and never quietly either.

    TWO PATHS, and which one ran is the most important line in the report:

      MEASURED — with all four damper pots, vertical load comes from the pots
      themselves: wheel travel × wheel rate, about each pot's own median as
      ride height. This can see a corner sitting low, a bent pushrod, a
      platform nobody squared. It is the only path that can find asymmetry,
      because it is the only one not built on the assumption of symmetry.

      MODELLED — without the pots, load comes from closed-form transfers with
      the lateral share split by a TLLTD taken once from the model. Left and
      right then differ only by the sign of the transfer, so the two sides
      are symmetric BY CONSTRUCTION. Any asymmetry a report finds on this
      path is arithmetic, not evidence, and the report says so.
    """
    from .kinematics import SuspensionKinematics
    db = ctx.data
    kin, p, vd = _veh(ctx)
    ok = np.isfinite(db.series["time"])
    t = db.series["time"][ok]
    ay = db.series["ay"][ok]
    ax = db.series["ax"][ok] if "ax" in db.series else np.zeros_like(ay)
    keep = np.isfinite(ay) & np.isfinite(ax)
    idx_all = np.flatnonzero(keep)
    if idx_all.size > n_max:                 # deterministic decimation
        idx_all = idx_all[np.linspace(0, idx_all.size - 1,
                                      n_max).astype(int)]
    t, ay, ax = t[idx_all], ay[idx_all], ax[idx_all]

    g = 9.80665
    w = p.mass * g
    static = {"FL": w * p.weight_dist_front / 2.0,
              "FR": w * p.weight_dist_front / 2.0,
              "RL": w * (1.0 - p.weight_dist_front) / 2.0,
              "RR": w * (1.0 - p.weight_dist_front) / 2.0}
    notes: List[str] = []

    pots = [c for c in _CORNERS if _CORNER_POT[c] in db.series]
    if len(pots) == 4:
        source = "measured — all four damper pots"
        try:
            mr = float(kin.motion_ratio())
        except Exception:                                    # noqa: BLE001
            mr = 0.5
        k_wheel = float(kin.wheel_rate(
            float(ctx.ask.param("spring_rate_N_per_mm", 35.0))))
        raws = {}
        for c in _CORNERS:
            r = db.series[_CORNER_POT[c]][ok][idx_all]
            raws[c] = np.nan_to_num(r, nan=float(np.nanmedian(r)))
        #  ONE reference for all four pots, not one each. Zeroing every pot on
        #  its own median would remove precisely the signal this job exists to
        #  find: a corner sitting low is a STATIC offset, and per-pot zeroing
        #  subtracts it out and then reports a beautifully square car.
        ref = float(np.mean([np.median(raws[c]) for c in _CORNERS]))
        notes.append(
            f"Wheel load from pot travel: motion ratio {mr:.3f}, wheel rate "
            f"{k_wheel:.1f} N/mm, all four pots zeroed on ONE common "
            f"reference ({ref:.2f} mm, the mean of the four medians) so that "
            "static corner-to-corner offsets survive into the loads. "
            "Positive pot travel is bump.")
        notes.append(
            "That reference assumes the four pots share a calibration. If one "
            "reads long, this job will report it as a corner carrying extra "
            "load — which is why pot calibration is the first thing to rule "
            "out when the asymmetry section lights up.")
        Fz = {c: np.clip(static[c] + (raws[c] - ref) / max(mr, 1e-6) * k_wheel,
                         0.0, None) for c in _CORNERS}
    else:
        source = ("modelled — symmetric by construction"
                  + (f" ({len(pots)} of 4 pots present, which is not enough "
                     "to measure load)" if pots else ""))
        try:
            _loads, detail = vd.lateral_load_transfer(1.0)
            ltd_f = float(detail.get("ltd_front", float("nan")))
            ltd_r = float(detail.get("ltd_rear", float("nan")))
            tlltd = ltd_f / (ltd_f + ltd_r) if (ltd_f + ltd_r) else 0.5
        except Exception:                                    # noqa: BLE001
            tlltd = 0.5
        notes.append(
            f"Lateral transfer split front/rear by a TLLTD of {tlltd:.1%}, "
            "taken once from the model at 1 g and held constant — it is "
            "mildly load-dependent and this ignores that.")
        dW_lat_tot = (p.mass * ay * g * (p.cg_height / 1000.0)
                      / (p.track_front / 1000.0))
        dW_f = dW_lat_tot * tlltd
        dW_r = dW_lat_tot * (1.0 - tlltd)
        dW_lon = (p.mass * ax * g * (p.cg_height / 1000.0)
                  / (p.wheelbase / 1000.0)) / 2.0
        #  Convention, declared: positive ay loads the RIGHT-hand pair.
        Fz = {"FL": static["FL"] - dW_f - dW_lon,
              "FR": static["FR"] + dW_f - dW_lon,
              "RL": static["RL"] - dW_r + dW_lon,
              "RR": static["RR"] + dW_r + dW_lon}
        Fz = {k: np.clip(v, 0.0, None) for k, v in Fz.items()}

    hist = {}
    for c in _CORNERS:
        share = np.divide(Fz[c], np.maximum(w, 1e-6))
        #  Lateral force in EACH CORNER'S OWN frame, outboard-positive. The
        #  tyre always pushes the car toward the turn centre, so in a mirrored
        #  frame both sides see the same sign. Carrying the global sign of ay
        #  instead would push a left and a right corner in opposite directions
        #  through the same left-corner geometry, and the audit would report
        #  the mirror image as an asymmetry — which it is not.
        Fy = -np.abs(ay) * w * share
        Fx = ax * w * share
        hist[c] = (t, Fx, Fy, Fz[c])
    notes.append(
        "Lateral and longitudinal force at each corner is that corner's "
        "share of axle force, in proportion to its vertical load. That share "
        "rule is first-order and degrades near the limit.")
    notes.append(
        "**One geometry, four corners.** The toolkit holds a single corner's "
        "hardpoints, so all four audits run that geometry with the corner's "
        "own load history. Differences between corners here are therefore "
        "differences in LOAD, never in geometry — which is the right "
        "comparison for finding an unsquare car, and the wrong one for "
        "comparing a front upright against a rear one.")
    return hist, source, notes


def _job_ghost(ctx: Ctx) -> List[Artifact]:
    from . import compliance as cp, ghost_topology as gt
    hist, source, notes = _corner_histories(ctx)
    corner = cp.CompliantCorner.uniform_tube(ctx.hardpoints)
    _kin, p, _vd = _veh(ctx)

    audits, summaries = {}, {}
    for c in _CORNERS:
        t, Fx, Fy, Fz = hist[c]
        gc = gt.GhostCorner(corner, gt.uniform_sections(),
                            Fz_static_N=float(np.median(Fz)),
                            track_mm=p.track_front)
        a = gt.ghost_audit(gc, t, Fx, Fy, Fz, corner_label=c)
        audits[c] = a
        summaries[c] = (a.summary()
                        if callable(getattr(a, "summary", None)) else {})

    def _fos(c):
        v = summaries[c].get("worst_fos", float("inf"))
        return float(v) if v is not None and np.isfinite(v) else float("inf")
    worst_corner = min(_CORNERS, key=_fos)
    ctx.results["ghost_topology"] = GhostSweep(
        by_corner=audits, summaries=summaries, worst_corner=worst_corner,
        worst_audit=audits[worst_corner], load_source=source, notes=notes)

    measured = source.startswith("measured")
    rows = [(c, str(summaries[c].get("verdict", "?")),
             float(summaries[c].get("worst_fos", float("nan"))),
             str(summaries[c].get("worst_fos_member", "—")),
             float(summaries[c].get("max_d_camber_deg", float("nan"))),
             float(summaries[c].get("max_d_toe_deg", float("nan"))),
             float(summaries[c].get("max_loop_gain", float("nan"))))
            for c in _CORNERS]

    #  the asymmetry the symmetric model could never have shown you
    def _spread(i):
        vals = [r[i] for r in rows if np.isfinite(r[i])]
        return (max(vals) - min(vals)) if vals else float("nan")

    def _pair(a, b):
        """Side-to-side gap, absolute AND relative.

        An absolute factor-of-safety gap is close to meaningless on its own:
        0.3 between two corners sitting at 7 is noise, 0.3 between two sitting
        at 1.2 is the whole story. Everything downstream reads the relative
        one; the absolute is printed because people ask for it.
        """
        try:
            va = float(summaries[a].get("worst_fos", float("nan")))
            vb = float(summaries[b].get("worst_fos", float("nan")))
            gap = abs(va - vb)
            mean = 0.5 * (va + vb)
            return gap, (gap / mean if mean else float("nan"))
        except Exception:                                    # noqa: BLE001
            return float("nan"), float("nan")

    front_gap, front_rel = _pair("FL", "FR")
    rear_gap, rear_rel = _pair("RL", "RR")
    verdicts = {r[1] for r in rows}
    eroded = [r[0] for r in rows if "FAITHFUL" not in r[1].upper()]

    lines = [
        f"**All four corners audited.** Vertical load is **{source}**.",
        "",
        "| corner | verdict | worst FoS | on | Δcamber (deg) | Δtoe (deg) | "
        "loop gain |",
        "|---|---|---|---|---|---|---|",
    ] + [f"| **{c}** | `{v}` | {_fnum(f)} | {m} | {_fnum(dc)} | {_fnum(dt)} "
         f"| {_fnum(lg)} |" for c, v, f, m, dc, dt, lg in rows] + [
        "",
        (f"⚠️ **Eroded at {', '.join(eroded)}.** The rigid model stopped "
         f"describing {'those corners' if len(eroded) > 1 else 'that corner'} "
         "during this event, so every camber and toe number elsewhere in this "
         "bundle inherits that — and no amount of geometry work recovers an "
         "angle the structure is giving away."
         if eroded else
         "**The rigid model held at every corner.** For this event, the "
         "geometry numbers elsewhere in this bundle are standing on solid "
         "ground."),
        "",
        "### Asymmetry",
        "",
    ]
    if measured:
        lines += [
            f"- Front pair FL↔FR: **{_fnum(front_rel, '{:.1%}')}** apart in "
            f"worst factor of safety ({_fnum(front_gap)} absolute)",
            f"- Rear pair RL↔RR: **{_fnum(rear_rel, '{:.1%}')}** apart "
            f"({_fnum(rear_gap)} absolute)",
            f"- Spread across all four: FoS **{_fnum(_spread(2))}**, camber "
            f"loss **{_fnum(_spread(4))} deg**",
            "",
            _asymmetry_note(front_rel, rear_rel),
        ]
    else:
        lines += [
            "**Not assessed.** Without four damper pots the load history is "
            "modelled, and a modelled history makes left and right symmetric "
            "by construction — so any difference between the two sides of "
            "this table is arithmetic, not evidence. Log all four pots and "
            "this section becomes the most useful one in the file: a corner "
            "sitting low, a bent pushrod or an unsquared platform shows up "
            "here and nowhere else.",
        ]
    lines += [
        "",
        "**Loop gain** is the one to watch. It is how much the deflection "
        "feeds back into the load that caused it; as it approaches 1 the "
        "corner stops converging and the structure is no longer a spring in "
        "the loop, it *is* the loop.",
        "",
        "### How the loads were built",
        "",
    ] + [f"- {n}" for n in notes] + [
        "",
        f"*Screening fidelity: {len(_CORNERS)} corners, "
        f"{summaries[worst_corner].get('n_instants', '—')} instants each, on "
        "the default uniform tube set. Load your real sections in the 👻 "
        "Ghost Topology tab before quoting a factor of safety.*",
    ]

    inst_rows = []
    for c in _CORNERS:
        for inst in (getattr(audits[c], "instants", []) or []):
            ld = getattr(inst, "load", None)
            inst_rows.append((c, float(getattr(inst, "t", float("nan"))),
                              float(getattr(ld, "Fx", float("nan"))) if ld else None,
                              float(getattr(ld, "Fy", float("nan"))) if ld else None,
                              float(getattr(ld, "Fz", float("nan"))) if ld else None))
    return [
        Artifact("ghost/ghost_audit.md",
                 _md("Ghost topology — all four corners", lines), "md"),
        Artifact("ghost/corner_summary.csv",
                 _csv_bytes(["corner", "verdict", "worst_fos",
                             "worst_fos_member", "max_d_camber_deg",
                             "max_d_toe_deg", "max_loop_gain"], rows), "csv"),
        Artifact("ghost/instants.csv",
                 _csv_bytes(["corner", "t_s", "Fx_N", "Fy_N", "Fz_N"],
                            inst_rows), "csv"),
        Artifact("ghost/summary.json",
                 __import__("json").dumps(
                     {"load_source": source, "worst_corner": worst_corner,
                      "corners": summaries}, indent=2, sort_keys=True,
                     default=str).encode(), "json"),
    ]


#  Relative side-to-side thresholds. Declared, not folklore: below 5 % is
#  inside what phasing between lateral and longitudinal events produces on a
#  perfectly square car, and above 15 % is bigger than any plausible phasing
#  artefact. Both were set by running a synthetic symmetric log through this
#  job and reading what a square car actually scores.
_ASYM_CLOSE, _ASYM_LARGE = 0.05, 0.15


def _asymmetry_note(front_rel: float, rear_rel: float) -> str:
    """Commentary that answers the numbers. A side-to-side gap is either
    worth chasing or it is not, and saying which is the entire value."""
    worst = max([g for g in (front_rel, rear_rel) if np.isfinite(g)],
                default=float("nan"))
    if not np.isfinite(worst):
        return ("The pairs could not be compared — at least one corner "
                "produced no factor of safety.")
    if worst < _ASYM_CLOSE:
        return (f"The two sides agree to within {worst:.1%}. Whatever this "
                "car's problem is, it is not a lopsided corner — which is "
                "worth knowing, because side-to-side is the first thing "
                "people suspect and the most expensive thing to chase on a "
                "hunch.")
    if worst < _ASYM_LARGE:
        return (f"A visible but modest side-to-side difference "
                f"({worst:.1%}). Check corner "
                "weights and pot calibration before reading anything into "
                "it: a miscalibrated pot produces exactly this signature and "
                "costs nothing to rule out.")
    return (f"**A large side-to-side difference — {worst:.1%}.** One corner "
            "is working "
            "materially harder than its opposite. Corner weights, a bent "
            "pushrod, a damper not on its platform, or a pot reading long — "
            "in that order of likelihood, and all four are cheap to check "
            "before you spend a session tuning a car that is not square.")


register_job(Job("ghost_topology", "Ghost topology — all four corners",
                 "ghost", _job_ghost, needs_channels=("time", "ay"),
                 data_activated=True, cost_s=2.5))


# --- MorphMesh, reachable only through the ghost audit ------------------------ #
def _job_morph(ctx: Ctx) -> List[Artifact]:
    """The legitimate route into topology optimisation.

    MorphMesh was left out of the express lane at first because it needs a
    component, and the lane has the car's numbers rather than the bracket's
    geometry. The ghost audit supplies exactly what was missing: a load fan
    on a named member, derived from the team's own log. So the dependency is
    not a convenience — it is the reason this job is allowed to exist here.
    """
    from . import morphmesh as mm
    sweep = ctx.results.get("ghost_topology")
    audit = getattr(sweep, "worst_audit", sweep)
    corner = getattr(sweep, "worst_corner", "FL")
    sm = audit.summary() if callable(getattr(audit, "summary", None)) else {}
    member = str(sm.get("worst_fos_member") or "LF")

    #  Reduced settings, declared. A full run is ~3× this and belongs in the
    #  tab, where the wait comes with controls and a picture.
    res = mm.morph_from_audit(audit, ctx.hardpoints, member=member,
                              n_cases=3, max_iter=12, betas=(1.0, 2.0, 4.0))
    ok = bool(getattr(res, "ok", False))
    lines = [
        f"Corner **{corner}**, member **{member}** — chosen because the "
        f"ghost audit found it carried the worst factor of safety "
        f"({_fnum(sm.get('worst_fos'))}) of all four corners across your "
        f"event. The load fan is your own log's, not a synthetic one, and on "
        f"an asymmetric car this is frequently not the corner a symmetric "
        f"model would have sent you to.",
        "",
        "| quantity | value |", "|---|---|",
        f"| result | {'converged' if ok else '**did not converge**'} |",
        f"| mass | **{_fnum(getattr(res, 'mass_g', None))} g** |",
        f"| factor of safety | **{_fnum(getattr(res, 'fos', None))}** |",
        f"| suggested thickness | "
        f"{_fnum(getattr(res, 'suggested_thickness_mm', None))} mm |",
        f"| coarsen premium | "
        f"{_fnum(getattr(res, 'coarsen_premium', None))} |",
        f"| solves | {getattr(res, 'n_solves', '—')} |",
        "",
    ]
    for f in (getattr(res, "findings", []) or [])[:10]:
        if isinstance(f, dict):
            lines.append(f"- **{str(f.get('severity', '')).upper()}** — "
                         f"{f.get('message', '')}")
    lines += [
        "",
        "**Coarsen premium** is the mass you pay for making the part "
        "buildable in your shop rather than ideal in the solver. If it is "
        "large, the constraint is your process, not your design — and the "
        "cheapest gram on the car is the one you buy by improving the "
        "process, not the geometry.",
        "",
        "*Reduced settings: 3 load cases, 12 iterations, a three-step beta "
        "continuation — roughly a third of the full run. The result is a "
        "shape you can argue about, not one you should cut. The 🕸️ "
        "MorphMesh tab runs it properly and draws it.*",
    ]
    arts = [Artifact("morph/bracket_morph.md",
                     _md("MorphMesh — the member your own log flagged",
                         lines), "md")]
    cells = getattr(res, "cells_csv", None)
    if isinstance(cells, str) and cells.strip():
        arts.append(Artifact("morph/density_field.csv", cells.encode(),
                             "csv"))
    if callable(getattr(res, "to_json", None)):
        try:
            arts.append(Artifact(
                "morph/morph_result.json",
                __import__("json").dumps(res.to_json(), indent=2,
                                         sort_keys=True,
                                         default=str).encode(), "json"))
        except Exception:                                    # noqa: BLE001
            pass
    return arts


register_job(Job("morph_bracket", "MorphMesh from the ghost audit", "morph",
                 _job_morph, needs_jobs=("ghost_topology",), cost_s=10.0))


# =========================================================================== #
#  RULES — the car against the ruleset, with the ruleset's provenance attached
# =========================================================================== #
def _job_rules_declared(ctx: Ctx) -> List[Artifact]:
    from . import rules_fsae as rf
    p = dict(ctx.ask.params)
    findings = rf.check_declared(p)
    by_sev = {}
    for f in findings:
        by_sev.setdefault(f.severity, []).append(f)
    n_check, n_total = rf.coverage()

    for f in findings:
        if f.severity in ("ok",):
            continue
        ctx.flag(
            subsystem="Rules compliance", item=f"{f.rid} — {f.rule.title if f.rule else f.rid}",
            mode=("Limit exceeded" if f.severity == "violation" else
                  "Operating with no margin to the limit"
                  if f.severity == "watch" else
                  "Not verified against the rule"),
            effect=("Technical inspection failure or event penalty"),
            cause=f.message,
            severity=9 if f.severity == "violation" else 6,
            status=f.severity,
            evidence="unchecked" if f.severity == "unknown" else "modelled",
            action=("Correct the design to the limit." if f.severity == "violation"
                    else "State the value and re-run — an unchecked rule is "
                         "not a passed rule."))
    icon = {"violation": "🔴", "watch": "🟠", "ok": "🟢", "unknown": "⚪"}
    lines = rf.banner() + [
        "",
        f"Checked **{len(findings)} numbers** against "
        f"**{rf.RULESET.label()}**.",
        "",
        "| | rule | finding |",
        "|---|---|---|",
    ]
    for sev in ("violation", "watch", "ok", "unknown"):
        for f in by_sev.get(sev, []):
            lines.append(f"| {icon[sev]} | `{f.rid}` | {f.message} |")
    lines += [
        "",
        "### What this did not check",
        "",
        f"This toolkit encodes **{n_total} rules**, of which **{n_check}** "
        "carry a number that can be checked at all. The published rulebook "
        "has several hundred, and most of them are not numbers: *operable by "
        "an untrained person in ten seconds*, *visible from all angles*, "
        "*must visually show disconnected*. None of those can be checked from "
        "a spreadsheet, and no amount of green above says anything about "
        "them.",
        "",
        "**An unchecked rule is not a passed rule.** Every value you did not "
        "state came back `unknown` rather than `ok` for exactly that reason.",
        "",
        "### The ones that will actually cost you",
        "",
        "The numeric limits are the easy half. The half that fails cars at "
        "inspection is the half about isolation, labelling, and whether the "
        "shutdown path does what the diagram says — so book the time with "
        "your ESF, not with this file.",
    ]
    rows = [(f.rid, f.severity, f.value, f.limit, f.unit, f.message)
            for f in findings]
    return [Artifact("rules/declared_check.md",
                     _md("Rules check — the numbers you declared", lines),
                     "md"),
            Artifact("rules/declared_check.csv",
                     _csv_bytes(["rule", "severity", "value", "limit", "unit",
                                 "finding"], rows), "csv")]


register_job(Job("rules_declared", "Rules check — declared numbers", "rules",
                 _job_rules_declared, cost_s=0.2))


def _job_rules_measured(ctx: Ctx) -> List[Artifact]:
    """Score the team's own log the way the event's Energy Meter scores it.

    This is the one that earns its place. The power limit is not "stay under
    80 kW", it is EV.3.4.1's dwell-and-moving-average algorithm, and a team
    can run it on last week's endurance data instead of finding out on the
    day what it costs.
    """
    from . import rules_fsae as rf
    db = ctx.data
    ok = np.isfinite(db.series["time"])
    t = db.series["time"][ok]
    v = db.series["pack_v"][ok]
    i = db.series["pack_i"][ok]
    spd = db.series["speed"][ok] if "speed" in db.series else None
    res = rf.check_measured(t, v, i, speed_ms=spd)

    if res.n_violations:
        ctx.flag(
            subsystem="Rules compliance", item="EV.3.3.1 / EV.3.4.1 power limit",
            mode="Power limit exceeded in a logged run",
            effect=f"{res.endurance_penalty_s:.0f} s of Endurance penalty "
                   f"under EV.3.5.2; best-run disqualification in the dynamic "
                   f"events under EV.3.5.1",
            cause=f"{res.n_violations} excursion(s) past "
                  f"{rf.limit('EV.3.3.1'):g} kW",
            severity=7, status="violation", evidence="measured",
            action="Clamp the torque request in software below the limit with "
                   "margin for the event's meter, not yours.")
    if not res.rate_adequate:
        ctx.flag(
            subsystem="Data acquisition", item="Pack V/I logging rate",
            mode="Log too slow to resolve the rule it is checked against",
            effect="A clean result that means nothing; a violation found at "
                   "the event instead of in the workshop",
            cause=f"{res.sample_rate_hz:.0f} Hz against a "
                  f"{(rf.limit('EV.3.4.1') or 0.1)*1000:.0f} ms dwell",
            severity=5, status="violation", evidence="measured",
            action="Log pack voltage and current at 100 Hz or better.")
    icon = {"violation": "🔴", "watch": "🟠", "ok": "🟢", "unknown": "⚪"}
    ev_rows = [("power", a, b, k) for a, b, k in res.power_events]
    ev_rows += [("voltage", a, b, k) for a, b, k in res.voltage_events]
    ev_rows += [("regen", a, b, k) for a, b, k in res.regen_events]
    ev_rows.sort(key=lambda r: r[1])

    lines = rf.banner() + [
        "",
        f"Log scored at **{_fnum(res.sample_rate_hz)} Hz** using the "
        f"violation definition in `EV.3.4.1`: an excursion counts once it "
        f"persists continuously for "
        f"{(rf.limit('EV.3.4.1') or 0.1)*1000:.0f} ms, **or** once a "
        f"{(rf.limit('EV.3.4.1b') or 0.5)*1000:.0f} ms moving average crosses "
        "the limit. Overlapping detections are merged so one excursion is "
        "counted once.",
        "",
        "| quantity | measured | limit |", "|---|---|---|",
        f"| peak power | **{_fnum(res.peak_kw)} kW** | "
        f"{rf.limit('EV.3.3.1'):g} kW |",
        f"| peak pack voltage | **{_fnum(res.peak_v)} V** | "
        f"{rf.limit('EV.3.3.2'):g} V |",
        "",
    ]
    if not res.rate_adequate:
        lines += [
            f"🔴 **This log cannot resolve the rule.** At "
            f"{_fnum(res.sample_rate_hz)} Hz there are too few samples inside "
            f"a {(rf.limit('EV.3.4.1') or 0.1)*1000:.0f} ms dwell to say "
            "whether an excursion lasted long enough to count. A clean result "
            "below is the logger being blind, not the car being legal — log "
            "pack voltage and current at 100 Hz or better before trusting "
            "any of it.",
            "",
        ]

    if res.n_violations:
        lines += [
            f"## 🔴 {res.n_violations} "
            f"violation{'s' if res.n_violations != 1 else ''} — "
            f"**{res.endurance_penalty_s:.0f} s** of Endurance penalty",
            "",
            f"At `EV.3.5.2`'s {rf.limit('EV.3.5.2'):g} s per violation, this "
            f"run would have added **{res.endurance_penalty_s:.0f} seconds** "
            "to your Endurance time. In the Acceleration, Skidpad and "
            "Autocross events the accounting is harsher: `EV.3.5.1` "
            "disqualifies your best run for each run containing a violation.",
            "",
            "| kind | from (s) | to (s) | caught by |",
            "|---|---|---|---|",
        ] + [f"| {k} | {a:.3f} | {b:.3f} | {w} |" for k, a, b, w in ev_rows
             if k != "regen"]
    else:
        lines += ["## 🟢 No power or voltage violations in this log", ""]
    if res.regen_events:
        lines += [
            "",
            f"### ⚠️ {len(res.regen_events)} low-speed regeneration episodes",
            "",
            f"`EV.3.3.3` prohibits regeneration below "
            f"{rf.limit('EV.3.3.3'):g} km/h. This is usually a control-"
            "software cutoff nobody implemented rather than a hardware "
            "problem, and it is cheap to fix before it is expensive to "
            "explain.",
        ]
    lines += ["", "### Findings", ""]
    for f in res.findings:
        lines.append(f"- {icon.get(f.severity, '·')} `{f.rid}` — {f.message}")
    lines += [
        "",
        "*The event's Energy Meter is the instrument of record, not your "
        "logger. Sensor placement, calibration and filtering all differ, so "
        "treat a narrow pass here as a fail: `EV.3.4.3` makes tampering with "
        "that meter a disqualification, and arguing with it afterwards has "
        "never worked for anybody.*",
    ]
    csvb = _csv_bytes(["kind", "t_start_s", "t_end_s", "caught_by"], ev_rows)
    return [Artifact("rules/measured_check.md",
                     _md("Rules check — scored from your own log", lines),
                     "md"),
            Artifact("rules/violations.csv", csvb, "csv")]


register_job(Job("rules_measured", "Rules check — measured from the log",
                 "rules", _job_rules_measured,
                 needs_channels=("time", "pack_v", "pack_i"),
                 data_activated=True, cost_s=0.5))


# =========================================================================== #
#  COOLING · PRINTED PARTS · POWERTRAIN · TRACTIVE SAFETY
# =========================================================================== #
def _job_printed_part(ctx: Ctx) -> List[Artifact]:
    """A forced filament substitution, priced instead of eyeballed."""
    from . import printed_parts as pp
    low = (ctx.ask.text or "").lower()
    wanted = "paht_cf"
    got = "onyx" if "onyx" in low or "markforged" in low else "onyx"
    for k in pp.MATERIALS:
        if k.replace("_", "") in low.replace("-", "").replace(" ", ""):
            got = k
            break
    t_c = float(ctx.ask.param("coolant_c", 80.0))
    duty = pp.Duty(service_temp_c=t_c, load_is_sustained=True,
                   across_layers=True, dried_and_sealed=False,
                   design_fos=3.0)
    sub = pp.substitute(wanted, got, duty)
    man_up = pp.manifold_check(got, inner_dia_mm=float(ctx.ask.param("bore_mm", 25.0)),
                               wall_mm=float(ctx.ask.param("wall_mm", 3.0)),
                               pressure_bar=float(ctx.ask.param("cap_bar", 1.1)),
                               duty=duty, printed_upright=True)
    man_flat = pp.manifold_check(got, inner_dia_mm=float(ctx.ask.param("bore_mm", 25.0)),
                                 wall_mm=float(ctx.ask.param("wall_mm", 3.0)),
                                 pressure_bar=float(ctx.ask.param("cap_bar", 1.1)),
                                 duty=duty, printed_upright=False)

    ctx.flag(
        subsystem="Cooling", item="Printed coolant manifold",
        mode=f"Material substituted {sub.wanted.material.name} → "
             f"{sub.got.material.name}",
        effect="Manifold burst or creep leak; coolant loss and a hot shutdown",
        cause=f"Bureau does not stock the design filament; allowable stress "
              f"falls to {sub.ratio:.0%} of design at {t_c:g} °C",
        severity=8,
        status="violation" if sub.ratio < 0.5 else "watch",
        evidence="modelled",
        action="Re-run the FEA against "
               f"{sub.got.allowable_mpa:.1f} MPa allowable, and print the "
               "part lying down — orientation alone is worth "
               f"{man_flat.fos/max(man_up.fos,1e-9):.1f}x.")
    ctx.flag(
        subsystem="Cooling", item="Printed manifold",
        mode="Print orientation not specified on the drawing",
        effect="Hoop stress lands across the layer bond; burst at a fraction "
               "of the expected pressure",
        cause="Orientation decided by whoever loads the plate",
        severity=8, status="watch", evidence="unchecked",
        action="Put the build orientation on the drawing as a controlled "
               "characteristic.")
    lines = [
        f"Duty: **{t_c:g} °C**, sustained load, principal stress across the "
        f"layers, filament not stated as dried and sealed. Design FoS 3.",
        "",
        f"### {sub.wanted.material.name} → {sub.got.material.name}",
        "",
        sub.verdict,
        "",
        "| | wanted | got |",
        "|---|---|---|",
        f"| datasheet in-plane | {sub.wanted.base_mpa:g} MPa | "
        f"{sub.got.base_mpa:g} MPa |",
        f"| HDT | {sub.wanted.material.hdt_c:g} °C | "
        f"{sub.got.material.hdt_c:g} °C |",
        f"| **allowable at this duty** | "
        f"**{sub.wanted.allowable_mpa:.1f} MPa** | "
        f"**{sub.got.allowable_mpa:.1f} MPa** |",
        f"| working stress (÷FoS) | "
        f"{sub.wanted.working_stress_mpa():.1f} MPa | "
        f"{sub.got.working_stress_mpa():.1f} MPa |",
        "",
        "### Where the strength went",
        "",
        "| knockdown | factor | why |",
        "|---|---|---|",
    ] + [f"| {n} | ×{v:.2f} | {w} |"
         for n, v, w in sub.got.factors] + [
        f"| **total** | **×{sub.got.total_knockdown:.2f}** | |",
        "",
        "That column is the answer to *why does the datasheet say 40 MPa and "
        "you say 6*. None of those four is optional and none of them appears "
        "in a default isotropic FEA material card.",
        "",
        "### What to do about it",
        "",
    ] + [f"- {a}" for a in sub.actions] + [
        "",
        "### The manifold as a pressure vessel",
        "",
        f"{ctx.ask.param('bore_mm', 25.0):g} mm bore × "
        f"{ctx.ask.param('wall_mm', 3.0):g} mm wall at "
        f"{ctx.ask.param('cap_bar', 1.1):g} bar.",
        "",
        "| print orientation | hoop (MPa) | FoS | min wall for FoS 3 |",
        "|---|---|---|---|",
        f"| upright (layers around the hoop) | {man_up.hoop_mpa:.2f} | "
        f"**{man_up.fos:.2f}** | {man_up.min_wall_mm:.1f} mm |",
        f"| lying down (hoop in-plane) | {man_flat.hoop_mpa:.2f} | "
        f"**{man_flat.fos:.2f}** | {man_flat.min_wall_mm:.1f} mm |",
        "",
        f"**Print orientation is worth {man_flat.fos/max(man_up.fos,1e-9):.1f}× "
        f"on this part** and costs nothing. It is a structural decision, and "
        "it is currently being made by whoever loads the plate.",
        "",
    ] + [f"- {n}" for n in man_up.notes] + [
        "",
        f"*Material data are published datasheet values "
        f"({sub.got.material.source}) from someone else's printer, not "
        f"coupons from yours. {sub.got.material.note} Print and pull your own "
        f"coupons in the same orientation as the part — it is a day of work "
        f"and it replaces every assumption on this page.*",
    ]
    rows = [(n, f"{v:.3f}", w) for n, v, w in sub.got.factors]
    return [Artifact("printing/material_substitution.md",
                     _md("Printed part — what the substitution costs", lines),
                     "md"),
            Artifact("printing/knockdowns.csv",
                     _csv_bytes(["knockdown", "factor", "reason"], rows),
                     "csv")]


register_job(Job("printed_part_check", "Printed-part material substitution",
                 "printing", _job_printed_part, cost_s=0.2))


def _job_cooling_loop(ctx: Ctx) -> List[Artifact]:
    from . import cooling as cl
    p_kw = float(ctx.ask.param("power_kw", 60.0))
    q_w, q_notes = cl.heat_load_w(p_kw * 1000.0)
    low = (ctx.ask.text or "").lower()
    ck = "water" if "water" in low and "glycol" not in low else "eg50"
    spec = cl.LoopSpec(heat_w=q_w, coolant_key=ck,
                       flow_lpm=float(ctx.ask.param("flow_lpm", 12.0)),
                       cap_pressure_bar=float(ctx.ask.param("cap_bar", 1.1)),
                       radiator_ua_w_per_k=float(ctx.ask.param("rad_ua",
                                                               120.0)))
    r = cl.size_loop(spec)
    water = cl.size_loop(cl.LoopSpec(heat_w=q_w, coolant_key="water",
                                     flow_lpm=spec.flow_lpm,
                                     radiator_ua_w_per_k=
                                     spec.radiator_ua_w_per_k))
    ctx.flag(
        subsystem="Cooling", item="Radiator / coolant loop",
        mode="Radiator undersized for the heat load",
        effect=f"Coolant settles at {r.steady_coolant_c:.0f} °C against a "
               f"{spec.max_coolant_c:g} °C limit; inverter derate mid-event",
        cause=f"{r.required_ua_w_per_k:.0f} W/K required against "
              f"{spec.radiator_ua_w_per_k:g} W/K installed",
        severity=7, status="ok" if r.ok else "violation", evidence="modelled",
        action="Size the radiator to the required UA before the bodywork is "
               "drawn, and consider water over glycol.")
    lines = [
        f"**{'✅' if r.ok else '⚠️'}** Loop at {p_kw:g} kW shaft power.",
        "",
    ] + [f"- {n}" for n in q_notes] + [
        "",
        "| quantity | value |", "|---|---|",
        f"| heat into coolant | **{r.spec.heat_w/1000:.2f} kW** |",
        f"| ΔT across the load | **{r.delta_t_k:.1f} K** |",
        f"| mass flow | {r.mass_flow_kg_s:.3f} kg/s |",
        f"| radiator UA required | **{r.required_ua_w_per_k:.0f} W/K** |",
        f"| UA margin | {r.ua_margin:.2f}× |",
        f"| steady coolant temperature | **{r.steady_coolant_c:.1f} °C** |",
        f"| time to limit with no radiator | {r.time_to_limit_s:.0f} s |",
        f"| margin to boiling | {r.boil_margin_c:.0f} °C |",
        "",
    ] + [f"- {n}" for n in r.notes] + [
        "",
        f"### Coolant choice is worth {r.delta_t_k/max(water.delta_t_k,1e-9):.2f}× "
        f"on ΔT",
        "",
        f"Same load and flow on distilled water: **{water.delta_t_k:.1f} K** "
        f"instead of {r.delta_t_k:.1f} K, and "
        f"{water.required_ua_w_per_k:.0f} W/K of radiator instead of "
        f"{r.required_ua_w_per_k:.0f}. Glycol buys freeze protection you do "
        "not need in July and costs radiator area you do.",
        "",
        "*Lumped steady-state: one heat source, one radiator, a constant UA "
        "and no air-side model. It sizes the radiator and it does not "
        "predict a transient — the pack thermal and rig files are where "
        "transients live.*",
    ]
    rows = [("heat_w", r.spec.heat_w), ("delta_t_k", r.delta_t_k),
            ("mass_flow_kg_s", r.mass_flow_kg_s),
            ("ua_required_w_per_k", r.required_ua_w_per_k),
            ("ua_margin", r.ua_margin),
            ("steady_coolant_c", r.steady_coolant_c),
            ("time_to_limit_s", r.time_to_limit_s),
            ("boil_margin_c", r.boil_margin_c)]
    return [Artifact("cooling/loop_sizing.md",
                     _md("Cooling loop sizing", lines), "md"),
            Artifact("cooling/loop_sizing.csv",
                     _csv_bytes(["quantity", "value"], rows), "csv")]


register_job(Job("cooling_loop", "Cooling loop sizing", "cooling",
                 _job_cooling_loop, cost_s=0.2))


def _job_cooling_rig(ctx: Ctx) -> List[Artifact]:
    """The question a cooling rig lives or dies on, asked before it is built.

    A rig measures heat rejection by computing it from a flow rate and two
    temperatures. If the sensors cannot resolve the difference the rig is
    built to detect, it will produce a summer of data that settles nothing —
    and that is decided by the sensor order, not by the plumbing.
    """
    from . import cooling as cl
    p_kw = float(ctx.ask.param("power_kw", 60.0))
    q_w, _n = cl.heat_load_w(p_kw * 1000.0)
    flow = float(ctx.ask.param("flow_lpm", 12.0))
    low = (ctx.ask.text or "").lower()
    ck = "water" if "water" in low and "glycol" not in low else "eg50"
    target = 0.10

    combos = [(ts, fm) for ts in ("type_k", "pt100_b", "pt100_a",
                                  "pt100_110din", "matched_pair")
              for fm in ("paddle", "turbine", "coriolis")]
    rows, best = [], None
    for ts, fm in combos:
        r = cl.rig_uncertainty(heat_w=q_w, flow_lpm=flow, coolant_key=ck,
                               temp_sensor=ts, flow_meter=fm,
                               target_resolution_rel=target)
        rows.append((cl.TEMP_SENSORS[ts].name, cl.FLOW_METERS[fm].name,
                     r.u_dt_rel, r.u_flow_rel, r.u_total_rel,
                     r.u_total_rel <= target))
        if r.u_total_rel <= target and best is None:
            best = (ts, fm, r)

    baseline = cl.rig_uncertainty(heat_w=q_w, flow_lpm=flow, coolant_key=ck,
                                  temp_sensor="pt100_b",
                                  flow_meter="paddle",
                                  target_resolution_rel=target)
    need_dt = cl.required_delta_t(temp_sensor="pt100_b",
                                  flow_meter="turbine",
                                  target_resolution_rel=target)
    half = cl.rig_uncertainty(heat_w=q_w, flow_lpm=flow / 2.0,
                              coolant_key=ck, temp_sensor="pt100_b",
                              flow_meter="turbine",
                              target_resolution_rel=target)

    ctx.flag(
        subsystem="Cooling", item="Cooling test rig",
        mode="Instrumentation cannot resolve the quantity the rig measures",
        effect="A summer of data that cannot distinguish two radiators; "
               "radiator sized on an unvalidated model",
        cause=f"Heat rejection computed from flow and two temperatures; "
              f"baseline sensor list gives ±{baseline.u_total_rel:.0%}",
        severity=6,
        status="violation" if baseline.u_total_rel > target else "ok",
        evidence="modelled",
        action="Buy a matched PT100 pair and plumb a throttling valve so the "
               "rig can run below the car's flow rate. Both are decided "
               "before the hoses are cut.")
    lines = [
        f"The rig has to resolve heat rejection to **±{target:.0%}** to be "
        f"worth building — below that it cannot tell two radiator designs "
        f"apart, and it cannot validate a CFD model to any tolerance anyone "
        f"will accept in a design review.",
        "",
        f"Load {q_w/1000:.2f} kW, flow {flow:g} L/min, "
        f"{cl.coolant(ck).name} → **ΔT = {baseline.delta_t_k:.2f} K**. "
        "Everything below follows from how small that number is.",
        "",
        "### A typical first sensor list",
        "",
        baseline.verdict,
        "",
    ] + [f"- {n}" for n in baseline.notes] + [
        "",
        "### The whole matrix",
        "",
        "| temperature | flow | ΔT term | flow term | total | meets target |",
        "|---|---|---|---|---|---|",
    ] + [f"| {a} | {b} | ±{c:.1%} | ±{d:.1%} | **±{e:.1%}** | "
         f"{'✅' if f else '—'} |" for a, b, c, d, e, f in rows] + [
        "",
        (f"**Cheapest combination that works: {cl.TEMP_SENSORS[best[0]].name} "
         f"+ {cl.FLOW_METERS[best[1]].name}**, at ±{best[2].u_total_rel:.1%}."
         if best else
         "**No combination in this table reaches the target at this flow "
         "rate.** That is not a reason to buy a better sensor; it is a reason "
         "to change the rig, which the next section explains."),
        "",
        "### Two structural fixes, both decided before the hoses are cut",
        "",
        f"**1 · Design the rig to run below the car's flow rate.** The error "
        f"is a fraction of ΔT, so halving the flow doubles ΔT and halves the "
        f"temperature term outright: at {flow/2:g} L/min the same PT100-B + "
        f"turbine list gives ±{half.u_total_rel:.1%} instead of "
        f"±{cl.rig_uncertainty(heat_w=q_w, flow_lpm=flow, coolant_key=ck, temp_sensor='pt100_b', flow_meter='turbine').u_total_rel:.1%}. "
        f"That needs a throttling valve and a bypass — two fittings, decided "
        f"now, impossible to retrofit cleanly later.",
        "",
        (f"To hit ±{target:.0%} with class-B PT100s and a turbine meter at "
         f"all, the rig must be run at a ΔT of at least "
         f"**{need_dt:.1f} K**. Size the throttling range around that number, "
         f"not around the car's operating point."
         if need_dt else
         "With that combination the flow meter alone already exceeds the "
         "target, so no ΔT saves it."),
        "",
        "**2 · Measure the difference, not two temperatures.** A matched "
        "PT100 pair is calibrated together so common-mode error cancels; it "
        "costs about what two good absolute sensors cost and resolves several "
        "times better. If you buy one thing off this page, buy that.",
        "",
        "### What else the rig should be instrumented for",
        "",
        "- **Pressure, both sides of the pump and across the radiator.** "
        "Without it you cannot separate a flow problem from a heat-transfer "
        "problem, and every confusing result will be one of those two.",
        "- **Air-side inlet temperature and face velocity.** Heat rejection "
        "quoted without the air-side condition is not a number anyone can "
        "reuse, including you in September.",
        "- **A repeatability run.** Run the same point three times on "
        "different days before trusting any comparison; the spread you get is "
        "the real error bar, and it is usually worse than the one computed "
        "here.",
        "",
        "*Root-sum-square propagation through Q = ρ·V̇·cp·ΔT, assuming "
        "independent errors and a 2 % uncertainty on cp. Systematic errors — "
        "a sensor in a fitting reading wall temperature instead of fluid, an "
        "un-developed flow profile at the meter — are not in here and are "
        "usually larger than everything that is.*",
    ]
    return [Artifact("cooling/rig_instrumentation.md",
                     _md("Cooling rig — can it measure what it is for?",
                         lines), "md"),
            Artifact("cooling/sensor_matrix.csv",
                     _csv_bytes(["temp_sensor", "flow_meter", "dt_term",
                                 "flow_term", "total", "meets_target"],
                                rows), "csv")]


register_job(Job("cooling_rig", "Cooling rig instrumentation budget",
                 "cooling", _job_cooling_rig, cost_s=0.3))


def _job_gear_ratio(ctx: Ctx) -> List[Artifact]:
    """Gear ratio does not need the output shaft CAD.

    A ratio sweep needs the motor curve, the rolling radius and the mass. The
    shaft geometry is downstream of the answer, not upstream of it — so this
    is not blocked, and treating it as blocked costs weeks.
    """
    from .laptime import MotorMap
    from . import pt_integration as pt
    _kin, p, _vd = _veh(ctx)
    torque = float(ctx.ask.param("motor_torque_nm", 140.0))
    power = float(ctx.ask.param("power_kw", 80.0))
    r_wheel = float(ctx.ask.param("wheel_radius_mm", 228.0)) / 1000.0
    mm = MotorMap.from_peak(peak_torque_nm=torque, peak_power_kw=power,
                            redline_rpm=6000.0, wheel_radius_m=r_wheel)
    solver = pt.GearRatioSolver(mm, mass_kg=p.mass, wheel_r_m=r_wheel,
                                mu=p.mu_peak)
    ratios = [round(x, 2) for x in np.arange(2.5, 6.01, 0.25)]
    out = {}
    for obj in pt.GearObjective:
        out[obj.value] = solver.sweep(ratios, objective=obj)

    bal = out["balanced"]
    rows = [(c.final_drive, c.top_speed_kmh, c.redline_speed_kmh,
             c.accel_0_75_s, c.launch_force_n, c.grip_limited_launch,
             c.score) for c in bal.candidates]
    picks = [(k, v.best.final_drive, v.best.accel_0_75_s, v.best.top_speed_kmh)
             for k, v in sorted(out.items()) if v.best]
    sp = pt.sprocket_design(bal.best.final_drive, torque) if bal.best else None

    lines = [
        f"Motor {torque:g} Nm / {power:g} kW, wheel radius "
        f"{r_wheel*1000:g} mm, car {p.mass:g} kg, μ {p.mu_peak:g}.",
        "",
        "> **This did not need the output shaft CAD.** A ratio sweep needs "
        "the motor curve, the rolling radius and the mass — the shaft is "
        "sized *from* the ratio, not before it. If gear ratio is being held "
        "behind shaft geometry on a schedule, that dependency runs the wrong "
        "way and it is costing weeks.",
        "",
        "| objective | best final drive | 0–75 km/h (s) | top speed (km/h) |",
        "|---|---|---|---|",
    ] + [f"| {k} | **{fd:g}** | {_fnum(a)} | {_fnum(t)} |"
         for k, fd, a, t in picks] + [
        "",
        "The three objectives disagreeing is the actual result. FSAE scoring "
        "weights Acceleration and Autocross far above top speed, so the "
        "acceleration-biased ratio is usually right — but check it against "
        "the longest straight on your event's track before committing, "
        "because a redline-limited car on that straight loses more than the "
        "launch gains.",
        "",
        "| final drive | top (km/h) | at redline (km/h) | 0–75 (s) | "
        "launch force (N) | grip limited |",
        "|---|---|---|---|---|---|",
    ] + [f"| {a:g} | {_fnum(b)} | {_fnum(c)} | {_fnum(d)} | {_fnum(e)} | "
         f"{'yes' if f else 'no'} |" for a, b, c, d, e, f, _s in rows] + [
        "",
        "**Grip-limited launch is the line to read.** Where it says yes, more "
        "ratio buys nothing — the tyre is already the constraint and you are "
        "just adding driveline torque for the mounts to react.",
        "",
    ]
    if sp is not None:
        lines += [
            "### Sprockets for the chosen ratio",
            "",
            f"- {sp.motor_sprocket_teeth}T → {sp.driven_sprocket_teeth}T "
            f"gives **{sp.actual_ratio:.3f}** against a target of "
            f"{bal.best.final_drive:g}",
            f"- Chain: {getattr(sp, 'chain_label', '—')}",
        ] + [f"- {n}" for n in (getattr(sp, "notes", []) or [])]
    lines += [
        "",
        "*Point-mass longitudinal model with a synthesised motor curve from "
        "peak torque and power. Replace it with your dyno pull — the shape "
        "between base speed and redline is exactly what decides this, and a "
        "synthesised curve is smoother than any real motor.*",
    ]
    return [Artifact("powertrain/gear_ratio.md",
                     _md("Gear ratio — swept, not blocked", lines), "md"),
            Artifact("powertrain/gear_ratio_sweep.csv",
                     _csv_bytes(["final_drive", "top_speed_kmh",
                                 "redline_speed_kmh", "accel_0_75_s",
                                 "launch_force_n", "grip_limited", "score"],
                                rows), "csv")]


register_job(Job("gear_ratio", "Gear ratio sweep", "powertrain",
                 _job_gear_ratio, cost_s=1.0))


def _job_tractive_safety(ctx: Ctx) -> List[Artifact]:
    from . import tractive_system as ts
    v = float(ctx.ask.param("pack_v_max", 400.0))
    pc = ts.PrechargeCircuit(pack_voltage_v=v, link_capacitance_f=250e-6,
                             precharge_r_ohm=1000.0, discharge_r_ohm=10000.0)
    trace = ts.simulate_precharge(pc)
    findings = list(ts.check_precharge(pc))
    chain = ts.ShutdownChain()
    findings += list(ts.check_shutdown_chain(chain))
    findings += list(ts.check_tsal(ts.TSAL(flash_hz=3.0, safe_threshold_v=60.0)))
    findings += list(ts.check_bspd(ts.BSPD(brake_threshold=0.6,
                                           power_threshold_w=5000.0,
                                           reaction_time_s=0.4)))
    sev_icon = {"ERROR": "🔴", "WARN": "🟠", "INFO": "⚪", "OK": "🟢"}
    rows = [(str(getattr(f, "check", "")),
             str(getattr(getattr(f, "severity", ""), "name",
                         getattr(f, "severity", ""))),
             str(getattr(f, "message", ""))) for f in findings]
    n_err = sum(1 for _c, s, _m in rows if "ERROR" in s.upper())

    lines = [
        f"Precharge, shutdown chain, TSAL and BSPD at a **{v:g} V** pack.",
        "",
        f"- Precharge to 90 %: **{_fnum(getattr(trace, 't_90pct_s', None))} s**",
        f"- Discharge below the safe threshold: "
        f"**{_fnum(getattr(trace, 't_discharge_safe_s', None))} s**",
        f"- Peak resistor power: "
        f"{_fnum(getattr(trace, 'peak_resistor_w', None))} W",
        "",
        f"### {len(rows)} findings, {n_err} of them errors",
        "",
        "| | check | finding |", "|---|---|---|",
    ] + [f"| {sev_icon.get(s.upper(), '·')} | `{c}` | {m} |"
         for c, s, m in rows] + [
        "",
        "The precharge resistor is the part teams get wrong twice: sized for "
        "the time constant and never checked for the energy it actually "
        "absorbs on every start-up. Half of ½CV² lands in that resistor each "
        "time, and a 1 W part in a 3 W duty fails quietly, weeks later, "
        "leaving a car that welds its contactors.",
        "",
        "*Values here are defaults, not your car's. Put your real link "
        "capacitance and resistor values in and re-run — and cross-check "
        "against `rules/declared_check.md`, which carries the rule numbers "
        "these checks implement.*",
    ]
    return [Artifact("ev/tractive_safety.md",
                     _md("Tractive system — precharge, shutdown, TSAL, BSPD",
                         lines), "md"),
            Artifact("ev/tractive_findings.csv",
                     _csv_bytes(["check", "severity", "message"], rows),
                     "csv")]


register_job(Job("tractive_safety", "Tractive system safety checks", "ev",
                 _job_tractive_safety, cost_s=0.5))


# =========================================================================== #
#  WIRING — the chart, and then everything the chart does not know
# =========================================================================== #
def _detect_insulation(text: str) -> str:
    """Which wire the sentence is actually about. Defaults to Tefzel, which
    is what most FSAE teams run, and says so in the report rather than
    silently assuming building wire."""
    low = (text or "").lower()
    for key, words in (("silicone", ("silicone", "silicon wire")),
                       ("tefzel", ("tefzel", "m22759", "etfe")),
                       ("thhn", ("thhn", "building wire", "thwn")),
                       ("xlpe125", ("xlpe", "cross-linked", "125"))):
        if any(w in low for w in words):
            return key
    return "tefzel"


def _fuse_line(pick, fuse_a, ambient_c: float, n_bundled: int):
    """Coordination needs both a fuse and a conductor. Missing either is its
    own message — reporting 'no fuse stated' when a fuse WAS stated but no
    gauge could be chosen sends the reader to fix the wrong thing."""
    from . import wiring as wr
    if not fuse_a:
        return ("unknown",
                "No fuse rating given, so coordination was not checked. "
                "State it — a fuse that does not protect its conductor is "
                "the single most common wiring error on an FSAE car.")
    if not pick:
        return ("unknown",
                f"A {float(fuse_a):g} A fuse was stated, but no single "
                f"conductor cleared the derated table, so there is nothing "
                f"to coordinate it against yet. Choose the conductor "
                f"arrangement first — with parallel runs the fuse must "
                f"protect the SMALLEST path, not the sum.")
    return wr.fuse_coordination(pick, float(fuse_a), insulation="tefzel",
                                ambient_c=ambient_c, n_bundled=n_bundled)





def _gauge_line(pick, current_a: float, ambient_c: float,
                n_bundled: int) -> str:
    """Render a recommendation, including the case where there isn't one.

    Printing "None AWG" is worse than useless. When no single conductor in
    the table can carry the current, the real answer is parallel runs or a
    higher-temperature wire, and that is what belongs on the page.
    """
    from . import wiring as wr
    if pick:
        return f"**{pick} AWG** (25 % margin on current)"
    n_par, per = wr.parallel_needed(current_a, insulation="tefzel",
                                    ambient_c=ambient_c,
                                    n_bundled=n_bundled)
    if not n_par:
        return ("**no recommendation** — the derated table has nothing that "
                "carries this current")
    return (f"**no single conductor in this table carries "
            f"{current_a:.0f} A** at {ambient_c:g} °C in a bundle of "
            f"{n_bundled}. The largest listed gauge (4/0) is good for "
            f"{per:.0f} A derated, so you need **{n_par} conductors in "
            f"parallel**, a 150–200 °C wire whose real rating is far above "
            f"this table's 90 °C floor, a busbar, or fewer conductors in the "
            f"bundle. This is the point at which the NEC chart stops being "
            f"the right document")


def _job_wiring(ctx: Ctx) -> List[Artifact]:
    from . import wiring as wr
    a = ctx.ask
    p_kw = float(a.param("power_kw", 60.0))
    v_ts = float(a.param("pack_v_max", 400.0))
    amb = float(a.param("ambient_c", 30.0))
    n_bund = int(round(float(a.param("n_bundled", 1.0))))
    run_m = float(a.param("run_length_mm", 1500.0)) / 1000.0
    i_ts = p_kw * 1000.0 / max(v_ts, 1e-6)

    ins = _detect_insulation(a.text)
    term = a.params.get("termination_c")
    hv_pick, hv_ladder = wr.recommend_gauge(
        current_a=i_ts, length_m=run_m, system_v=v_ts,
        insulation=ins, ambient_c=amb, n_bundled=n_bund,
        termination_c=term)
    #  The LV side: pump and fans are the cooling system's electrical bill,
    #  and they run on a long thin loom where volts, not heat, decide.
    lv_i = float(a.param("lv_current_a", 0.0)) or 15.0
    lv_pick, lv_ladder = wr.recommend_gauge(
        current_a=lv_i, length_m=max(run_m * 2.5, 3.0), system_v=12.0,
        insulation="silicone", ambient_c=amb, n_bundled=max(n_bund, 8),
        termination_c=term)

    hot = wr.derate("6", insulation=ins, ambient_c=amb, n_bundled=n_bund,
                    termination_c=term)
    rows = []
    for awg in ("14", "12", "10", "8", "6", "4", "2", "1/0"):
        c = wr.conductor(awg)
        d1 = wr.derate(awg, insulation=ins, ambient_c=amb,
                       n_bundled=n_bund, termination_c=term)
        rows.append((awg, c.area_mm2, c.a_90c, d1.temp_factor,
                     d1.bundle_factor, d1.allowed_a))
    #  What the insulation choice alone is worth at this ambient.
    ins_rows = []
    for k in ("thhn", "xlpe125", "tefzel", "silicone"):
        wire_only = wr.derate("6", insulation=k, ambient_c=amb,
                              n_bundled=n_bund).allowed_a
        as_built = wr.derate("6", insulation=k, ambient_c=amb,
                             n_bundled=n_bund,
                             termination_c=term).allowed_a
        pick_k, _l = wr.recommend_gauge(current_a=i_ts, length_m=run_m,
                                        system_v=v_ts, insulation=k,
                                        ambient_c=amb, n_bundled=n_bund,
                                        termination_c=term)
        ins_rows.append((wr.INSULATIONS[k][1].split(" —")[0].split(".")[0],
                         wr.rating_of(k), wire_only, as_built,
                         pick_k or "none single"))
    #  If a termination caps everything, the insulation upgrade buys nothing,
    #  and that is the single most useful line this job can print.
    #  How much of the best wire's capability the termination throws away.
    #  Comparing best-as-terminated against best-wire-alone is always a valid
    #  statement, where "do all the upgrades collapse to one value" is only
    #  sometimes one.
    _wire = [r[2] for r in ins_rows if r[2] is not None]
    _built = [r[3] for r in ins_rows if r[3] is not None]
    best_wire = max(_wire) if _wire else 0.0
    best_built = max(_built) if _built else 0.0
    wasted = (1.0 - best_built / best_wire) if best_wire else 0.0
    #  Which insulations gain nothing once the termination is applied.
    capped_out = [r[0].split("(")[0].strip() for r in ins_rows
                  if term and r[1] > term and r[3] is not None
                  and abs(r[3] - best_built) < 0.01]

    hv_run = next((x for x in hv_ladder if x.awg == hv_pick), None)
    fuse = a.params.get("fuse_a")
    fuse_sev, fuse_msg = _fuse_line(hv_pick, fuse, amb, n_bund)

    ctx.flag(
        subsystem="Electrical", item="Tractive-system conductor",
        mode="Conductor sized from an uncorrected building-wire chart",
        effect="Insulation degradation and an in-loom failure at speed",
        cause=f"Chart assumes 30 °C and no bundling; this loom is "
              f"{amb:g} °C with {n_bund} conductors → ×{hot.temp_factor*hot.bundle_factor:.2f}",
        severity=9,
        status="watch" if hv_pick else "violation",
        evidence="modelled",
        action=("Confirm the chosen gauge against the loom's real ambient."
                if hv_pick else
                "No single conductor clears this current — go to parallel "
                "runs, a busbar, or fewer conductors per bundle."))
    if term:
        ctx.flag(
            subsystem="Electrical", item="HV termination",
            mode="Termination rated below the conductor",
            effect=f"Overheating at the lug, not in the run; "
                   f"{wasted:.0%} of the conductor's capability unusable",
            cause=f"{term:g} °C termination on a "
                  f"{wr.rating_of(ins)} °C conductor",
            severity=8, status="watch" if wasted > 0.02 else "ok",
            evidence="modelled",
            action="Upgrade the termination before paying for insulation you "
                   "cannot use.")
    else:
        ctx.flag(
            subsystem="Electrical", item="HV termination",
            mode="Termination temperature rating unknown",
            effect="Circuit limited by an unquantified component; melted "
                   "cable ends",
            cause="No termination rating stated anywhere in the design",
            severity=8, status="unknown", evidence="unchecked",
            action="Record the lug, boot and connector ratings and re-run. "
                   "This is usually the real limit on the car.")
    lines = [
        f"Tractive system: **{p_kw:g} kW at {v_ts:g} V → {i_ts:.0f} A** "
        f"continuous-equivalent. Loom ambient **{amb:g} °C**, "
        f"**{n_bund}** current-carrying conductors bundled, "
        f"**{run_m:g} m** one-way run.",
        "",
        "> The base ampacities below are the **NEC building-wire** figures. "
        "They assume 30 °C ambient and a conductor that is not bundled with "
        "eleven friends inside a sleeve routed past a motor. A car is not a "
        "building, so the chart value is a starting point and never an "
        "answer.",
        "",
        "### What the chart says, and what you actually get",
        "",
        f"| AWG | area (mm²) | chart @30 °C | ×ambient | ×bundling | "
        f"**allowed @{amb:g} °C, {n_bund} bundled** |",
        "|---|---|---|---|---|---|",
    ] + [f"| {g} | {ar:g} | {_fnum(b)} A | ×{tf:.2f} | ×{bf:.2f} | "
         f"**{_fnum(al)} A** |" for g, ar, b, tf, bf, al in rows] + [
        "",
        f"Ambient and bundling **multiply**. That is the step teams miss: "
        f"each alone looks survivable, and together they take 6 AWG from "
        f"{_fnum(hot.base_a)} A on the chart to **{_fnum(hot.allowed_a)} A** "
        f"where you are putting it — a factor of "
        f"{(hot.base_a or 1)/max(hot.allowed_a or 1, 1e-9):.1f}.",
        "",
        "### Tractive-system cable",
        "",
        _gauge_line(hv_pick, i_ts, amb, n_bund),
        "",
    ] + ([f"- {n}" for n in hv_run.notes] if hv_run else []) + [
        "",
        f"- Governing constraint: **{hv_run.governing if hv_run else '—'}**",
        "",
        "### Fuse coordination",
        "",
        f"{'🔴' if fuse_sev == 'violation' else '🟠' if fuse_sev == 'watch' else '🟢' if fuse_sev == 'ok' else '⚪'} "
        f"{fuse_msg}",
        "",
        "A fuse is sized to protect the **conductor**, not the load. If the "
        "fuse rating exceeds the derated ampacity of the wire it feeds, the "
        "wire is the weakest element in the path — and it will fail inside a "
        "loom, on track, at the least convenient moment.",
        "",
        "### The cooling system's electrical bill",
        "",
        f"Pump and fans at **{lv_i:g} A** on 12 V, over a "
        f"{max(run_m*2.5, 3.0):g} m run in a loom of "
        f"{max(n_bund, 8)}: recommended **{lv_pick} AWG**.",
        "",
        "Low-voltage runs are almost never limited by heat. They are limited "
        "by **voltage drop** — a conductor that is thermally comfortable can "
        "still drop enough volts over a six-metre round trip to brown out an "
        "ECU on a crank. Every run above is checked against a 3 % drop limit "
        "as well as its ampacity, and the file says which one governs.",
        "",
        f"### What the insulation choice is worth at {amb:g} °C",
        "",
        "The NEC table stops at 90 °C, but its correction factors are just "
        "`√((T_rating − T_ambient) / (T_column − 30))` — the same physics "
        "for any rating. Extrapolated from the published column and corrected "
        "for the copper resistance shift, a high-temperature wire gets a real "
        "number instead of a 90 °C floor:",
        "",
        "| insulation | rating | 6 AWG, wire alone | 6 AWG, as terminated | "
        "smallest gauge for this run |",
        "|---|---|---|---|---|",
    ] + [f"| {name} | {r} °C | {_fnum(wo)} A | {_fnum(ab)} A | **{pk}** |"
         for name, r, wo, ab, pk in ins_rows] + [
        "",
        (f"⚠️ **The {term:g} °C termination throws away {wasted:.0%} of the "
         f"best wire's capability** — {_fnum(best_wire)} A of conductor "
         f"becomes {_fnum(best_built)} A as installed."
         + (f" Everything at or above {term:g} °C "
            f"({', '.join(capped_out)}) lands on the same number, so "
            f"upgrading between them buys flexibility and abrasion "
            f"resistance, not amps. The lug is the thing to change first."
            if len(capped_out) > 1 else
            " Upgrade the termination before paying for insulation you "
            "cannot use.")
         if term and wasted > 0.02 else
         f"Running **{ins}** rather than building-wire THHN is worth "
         f"{(ins_rows[2][3] or 0)/max(ins_rows[0][3] or 1, 1e-9):.2f}× on "
         f"this conductor as terminated. On an HV pair that is usually a "
         f"gauge or two of copper you stop carrying for the whole event."),
        "",
        "### The limit that actually bites",
        "",
        (f"Termination stated at **{term:g} °C**, and every gauge above is "
         f"capped by it. A conductor is only as good as what it lands in."
         if term else
         "**No termination rating was stated, so the conductor's own rating "
         "was used — which on a real car is optimistic.** Crimp lugs, "
         "heat-shrink boots and connector bodies are typically 105–125 °C. A "
         "200 °C cable into a 105 °C lug is a 105 °C circuit, and the melted "
         "end will be at the termination, not in the middle of the run. Say "
         "`termination 105` and re-run to see the real number."),
        "",
        f"*Source: {wr.SOURCE} Correction factors are NEC 310.15(B)(1) and "
        "310.15(C)(1). None of this substitutes for the rules or a qualified "
        "engineer, and none of it has been validated against your loom — "
        "put a thermocouple on the hottest cable under real load before you "
        "trust any of it.*",
    ]
    csv_rows = [(g, ar, b, tf, bf, al) for g, ar, b, tf, bf, al in rows]
    return [Artifact("wiring/conductor_sizing.md",
                     _md("Wiring — ampacity, derated for a car", lines), "md"),
            Artifact("wiring/derated_ampacity.csv",
                     _csv_bytes(["awg", "area_mm2", "chart_a_30c",
                                 "temp_factor", "bundle_factor",
                                 "allowed_a"], csv_rows), "csv")]


register_job(Job("wiring_sizing", "Conductor sizing and fuse coordination",
                 "wiring", _job_wiring, cost_s=0.3))


def _job_wiring_from_log(ctx: Ctx) -> List[Artifact]:
    """Size the cable on the RMS the car actually draws.

    Ampacity is a continuous rating and a race car's current is anything but.
    The nameplate peak is the wrong input and so is the average; RMS is the
    value that produces the same heating, and it is sitting in the log.
    """
    from . import wiring as wr
    db = ctx.data
    ok = np.isfinite(db.series["time"])
    i = db.series["pack_i"][ok]
    v = float(np.nanmedian(db.series["pack_v"][ok])) \
        if "pack_v" in db.series else float(ctx.ask.param("pack_v_max", 400.0))
    amb = float(ctx.ask.param("ambient_c", 30.0))
    n_bund = int(round(float(ctx.ask.param("n_bundled", 1.0))))
    run_m = float(ctx.ask.param("run_length_mm", 1500.0)) / 1000.0

    ls = wr.size_from_log(i, length_m=run_m, system_v=v,
                          insulation="tefzel", ambient_c=amb,
                          n_bundled=n_bund)
    chosen = ls.recommended_awg
    run = wr.check_run(chosen, current_a=ls.rms_a, length_m=run_m,
                       system_v=v, insulation="tefzel", ambient_c=amb,
                       n_bundled=n_bund) if chosen else None
    peak_run = wr.check_run(chosen, current_a=ls.peak_a, length_m=run_m,
                            system_v=v, insulation="tefzel", ambient_c=amb,
                            n_bundled=n_bund) if chosen else None
    fuse = ctx.ask.params.get("fuse_a")
    fsev, fmsg = _fuse_line(chosen, fuse, amb, n_bund)

    amb_l, n_bund_l = amb, n_bund
    lines = [
        f"Sized from **{i.size} samples** of your own pack current at "
        f"{_fnum(v)} V, not from a nameplate.",
        "",
        "| quantity | value |", "|---|---|",
        f"| RMS current | **{ls.rms_a:.1f} A** |",
        f"| peak current | {ls.peak_a:.1f} A |",
        f"| mean current | {ls.mean_a:.1f} A |",
        f"| crest factor | {ls.crest_factor if hasattr(ls, 'crest_factor') else ls.crest:.2f} |",
        f"| time above own RMS | {ls.duty_over_rms:.0%} |",
        "",
    ] + [f"- {n}" for n in ls.notes] + [
        "",
        "### The recommendation",
        "",
        f"- Size on RMS: {_gauge_line(ls.recommended_awg, ls.rms_a, amb, n_bund)}",
        f"- Size on peak: {_gauge_line(ls.peak_awg, ls.peak_a, amb, n_bund)}",
        "",
        "RMS is the number that produces the same heating, so RMS is what "
        "ampacity should be checked against. The peak still matters — but "
        "for **voltage drop**, not for heat, because a sag at full power is a "
        "real problem even when it only lasts four seconds.",
        "",
    ]
    if run and peak_run:
        lines += [
            "| case | current | drop | % of bus | cable loss |",
            "|---|---|---|---|---|",
            f"| at RMS | {ls.rms_a:.0f} A | {run.drop_v:.2f} V | "
            f"{run.drop_pct:.2f} % | {run.loss_w:.0f} W |",
            f"| at peak | {ls.peak_a:.0f} A | {peak_run.drop_v:.2f} V | "
            f"{peak_run.drop_pct:.2f} % | {peak_run.loss_w:.0f} W |",
            "",
            f"At peak the cable burns **{peak_run.loss_w:.0f} W** — heat the "
            "cooling system never budgeted for, deposited along something "
            "that is probably cable-tied to a part you care about.",
        ]
    lines += [
        "",
        "### Fuse",
        "",
        f"{'🔴' if fsev == 'violation' else '🟠' if fsev == 'watch' else '🟢' if fsev == 'ok' else '⚪'} {fmsg}",
        "",
        f"*Derated for {amb:g} °C ambient and {n_bund} bundled conductors "
        "off the NEC base table. Give it the real loom ambient — the number "
        "moves fast with temperature and the pit is not where the wire is.*",
    ]
    return [Artifact("wiring/sized_from_log.md",
                     _md("Wiring — sized from your own current trace", lines),
                     "md")]


register_job(Job("wiring_from_log", "Conductor sizing from the log",
                 "wiring", _job_wiring_from_log,
                 needs_channels=("time", "pack_i"), data_activated=True,
                 cost_s=0.3))


# =========================================================================== #
#  DFMEA — assembled from the engineering, not typed up afterwards
# =========================================================================== #
#  Severity is stated by the job that raises the finding, because only it
#  knows what the failure does. Occurrence and Detection are derived from the
#  finding's status and its evidence, on declared ladders:
#
#    OCCURRENCE — how likely the mode is to be present, given what the check
#    found. A violation is present now; an unknown could be anything, which is
#    why it does not score better than a watch.
#
#    DETECTION — the DFMEA sense: how likely you are to CATCH it before it
#    bites. This is where "unknown is not a pass" becomes arithmetic. A mode
#    confirmed against measured data is highly detectable; one nobody has
#    checked is barely detectable at all, and its RPN says so.
_OCCURRENCE = {"violation": 8, "watch": 5, "unknown": 5, "ok": 2}
_DETECTION = {"measured": 2, "modelled": 5, "unchecked": 9}


def _job_dfmea_autofill(ctx: Ctx) -> List[Artifact]:
    """The answer to 'are DFMEAs worth it for us'.

    They are worth it. They are just not worth *typing*. Every analysis in
    this bundle already identified a failure mode, its cause, its effect and
    what to do about it — a DFMEA is that same information in a table that
    can be sorted. So it is assembled here from what the other jobs raised,
    at no marginal cost in anyone's evening.

    The register below is not seeded, not generic, and not an example. Every
    row came from a number computed in this run, and every row carries the
    file it came from so a reviewer can go and argue with the analysis rather
    than with the table.
    """
    from . import dfmea as df
    raised = list(ctx.findings)
    rows = []
    for f in raised:
        sev = int(f["severity"])
        occ = _OCCURRENCE.get(f["status"], 5)
        det = _DETECTION.get(f["evidence"], 5)
        rpn = df.compute_rpn(sev, occ, det)
        band = str(df.classify_risk(sev, rpn))
        src = f.get("source", "")
        title = JOBS[src].title if src in JOBS else src
        rows.append((f["subsystem"], f["item"], f["mode"], f["effect"],
                     f["cause"], sev, occ, det, rpn, band, f["status"],
                     f["evidence"], f["action"], title))
    rows.sort(key=lambda r: (-r[8], -r[5]))

    if not rows:
        return [Artifact("dfmea/generated_register.md", _md(
            "DFMEA — generated from this run", [
                "**No failure modes were raised**, because no job in this "
                "run raises them. That is an empty register, not a clean "
                "one — ask for the cooling, wiring, printed-part or rules "
                "checks and this table fills itself.",
            ]), "md")]
    n_v = sum(1 for f in raised if f["status"] == "violation")
    n_u = sum(1 for f in raised if f["evidence"] == "unchecked")
    top = rows[:1][0] if rows else None

    lines = [
        f"**{len(rows)} failure modes**, every one of them derived from a "
        f"number computed in this run. Nothing here was typed by hand and "
        f"nothing here is a seeded example.",
        "",
        "> Your meeting asked whether DFMEAs are necessary, helpful, or just "
        "too time-consuming. The honest answer is that the *document* is not "
        "the valuable part — the **thinking** is, and you have already done "
        "the thinking. Every analysis you ran identified a failure mode, its "
        "cause, its effect and a corrective action. A DFMEA is that same "
        "information in a table you can sort. So it should cost you nothing, "
        "and here it costs 0.3 seconds.",
        "",
        f"- **{n_v}** modes are live violations right now",
        f"- **{n_u}** are things nobody has checked — which is why they score "
        f"badly on detection, not because they are unlikely",
        "",
    ]
    if top:
        lines += [
            f"### Highest RPN: {top[8]} — {top[1]}",
            "",
            f"**{top[2]}** → {top[3]}",
            "",
            f"*Cause:* {top[4]}",
            "",
            f"➜ **{top[12]}**",
            "",
        ]
    lines += [
        "### The register",
        "",
        "| RPN | subsystem | item | failure mode | S | O | D | evidence | "
        "raised by |",
        "|---|---|---|---|---|---|---|---|---|",
    ] + [f"| **{r[8]}** | {r[0]} | {r[1]} | {r[2]} | {r[5]} | {r[6]} | "
         f"{r[7]} | {r[11]} | {r[13]} |" for r in rows] + [
        "",
        "### How S, O and D were assigned",
        "",
        "**Severity** is stated by the analysis that raised the mode, because "
        "only it knows what the failure does to the car.",
        "",
        "**Occurrence** comes from what the check found: a violation is "
        "present now (8), a near-miss is likely (5), an unknown scores the "
        "same as a near-miss (5) because it could be either, and a clean "
        "check is unlikely (2).",
        "",
        "**Detection** is the DFMEA sense — how likely you are to catch it "
        "before it bites. A mode confirmed against **measured** data scores "
        "2; one supported only by a **model** scores 5; one **nobody has "
        "checked** scores 9. That last row is the important one: it turns "
        "*'we never looked at that'* from an absence into a number that sorts "
        "to the top of the table.",
        "",
        "### Read it twice",
        "",
        "RPN is a **ranking**, not a measurement. Severity 9 × occurrence 2 × "
        "detection 5 and severity 2 × occurrence 9 × detection 5 both give "
        "90, and only one of them can hurt somebody. Sort by RPN, then sort "
        "by severity and look again — anything with severity 8 or above "
        "belongs on the list regardless of where its RPN landed.",
        "",
        "### What this register does not contain",
        "",
        "Only what the analyses in this bundle looked at. Manufacturing "
        "modes, assembly errors, driver mistakes, and every subsystem you did "
        "not run today are absent — and absence here is not evidence of "
        "safety. Use this as the spine of your DFMEA and add the human and "
        "process modes by hand, which is the part a tool genuinely cannot do "
        "for you.",
        "",
        "*Regenerate it every time the design moves. A DFMEA that is "
        "rebuilt in under a second alongside the analysis is a living "
        "document; one that is typed up before a design review is an "
        "artifact of the review, and everybody can tell the difference.*",
    ]
    csvb = _csv_bytes(
        ["rpn", "band", "subsystem", "item", "failure_mode", "effect",
         "cause", "severity", "occurrence", "detection", "status",
         "evidence", "raised_by", "recommended_action"],
        [(r[8], r[9], r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7],
          r[10], r[11], r[13], r[12]) for r in rows])
    return [Artifact("dfmea/generated_register.md",
                     _md("DFMEA — generated from this run", lines), "md"),
            Artifact("dfmea/generated_register.csv", csvb, "csv")]


register_job(Job("dfmea_autofill", "DFMEA generated from the run's findings",
                 "dfmea", _job_dfmea_autofill, cost_s=0.2, runs_last=True,
                 note="Harvests structured findings raised by every other "
                      "job. Runs last by construction."))
