# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Kinematic target derivation, and its sensitivity to the tire inputs.

Targets are derived from vehicle requirements rather than declared, in the
order a designer fixes them:

* Camber. The outside tire should sit at its optimum inclination when the
  body rolls at the design lateral acceleration. Camber to the road is
  gamma_static + g*z + phi, so the gain that holds the optimum is
  g = (gamma_opt - phi - gamma_static) / z, with phi the roll angle (deg) and
  z the outside-wheel bump from that roll (mm).
* Ride frequency and anti-geometry. Under a longitudinal acceleration the
  spring must absorb the part of the load transfer the anti-geometry does not,
  inside the jounce budget left after roll:

      f_min = (1/2 pi) * sqrt( (1 - A) * dW / ((b - z_roll) * m_s) )

  with dW = m * g0 * a_x * h / (2 L) the per-wheel transfer (N), b the jounce
  budget (mm), z_roll the bump from roll (mm), A the anti fraction and m_s the
  sprung corner mass (kg). Inverting it gives the minimum anti fraction that
  lets a declared ride frequency keep the manoeuvre inside the budget.

Three inputs come from the tire model: the camber optimum, the peak friction
coefficient (which sets the design accelerations) and the camber sensitivity
(the grip lost per degree away from the optimum). ``tire_sensitivity`` varies
them and reports which targets move, so a synthetic tire model can be bounded
rather than merely admitted.

Units: mm, kg, deg, g (accelerations), Hz, N; anti fractions in percent at the
interface.
"""
from __future__ import annotations

import math

import numpy as np
from dataclasses import dataclass, replace

G0 = 9.81  # m/s^2


@dataclass(frozen=True)
class Vehicle:
    """Table 2 of the paper. Lengths in mm, masses in kg."""
    mass_kg: float = 300.0
    cg_height_mm: float = 280.0
    wheelbase_mm: float = 1630.0
    track_mm: float = 1210.0
    jounce_budget_mm: float = 17.5
    sprung_corner_front_kg: float = 60.1
    sprung_corner_rear_kg: float = 66.1
    roll_gradient_deg_per_g: float = 0.78
    ride_front_hz: float = 2.8
    ride_rear_hz: float = 3.0
    #: tire vertical rate (N/mm) in series with the suspension; None keeps the
    #: tires rigid, which is how every table of the paper was computed. With a
    #: rate, the whole lateral transfer deflects the tires and rolls the car
    #: further relative to the road, and the camber targets count it.
    tire_rate_n_per_mm: float | None = None
    #: suspension compression (mm) from aerodynamic downforce at the speed of
    #: the design case; it comes out of the jounce budget before roll and
    #: pitch are charged against it. 0 = no aero package.
    aero_heave_mm: float = 0.0


@dataclass(frozen=True)
class Tire:
    """The three tire-model inputs to the target chain."""
    camber_opt_deg: float = -1.83
    peak_mu: float = 1.55
    camber_sensitivity_pct_per_deg: float = 0.36


#: The design accelerations are tied to peak mu: 1.5 g lateral at mu = 1.55,
#: and the combined cases at 1.06 g each way (1.5 / sqrt 2).
MU_REF = 1.55
LAT_REF_G = 1.5
COMB_REF_G = 1.06
STRAIGHT_REF_G = 1.06


def design_accelerations(tire: Tire) -> dict[str, float]:
    """Design accelerations in g, scaled with peak mu from the reference."""
    s = tire.peak_mu / MU_REF
    return {"lateral": LAT_REF_G * s, "combined": COMB_REF_G * s,
            "straight": STRAIGHT_REF_G * s}


def roll(veh: Vehicle, a_lat_g: float) -> tuple[float, float]:
    """(roll angle deg, outside-wheel bump mm) at a lateral acceleration in g."""
    phi = veh.roll_gradient_deg_per_g * a_lat_g
    return phi, 0.5 * veh.track_mm * math.tan(math.radians(phi))


def tire_roll_deg(veh: Vehicle, a_lat_g: float) -> float:
    """Extra roll of the whole car relative to the road (deg) from tire deflection.

    Every newton of lateral transfer passes through the tires, geometric and
    elastic paths alike, so the moment m g a h deflects them against a roll
    stiffness of Kt * t^2 (both axles, each Kt * t^2 / 2). Zero when the
    tires are declared rigid (``tire_rate_n_per_mm`` None).
    """
    if not veh.tire_rate_n_per_mm:
        return 0.0
    M = veh.mass_kg * G0 * a_lat_g * veh.cg_height_mm / 1000.0          # N*m
    k = veh.tire_rate_n_per_mm * 1000.0 * (veh.track_mm / 1000.0) ** 2  # N*m/rad
    return math.degrees(M / k)


def roll_to_road(veh: Vehicle, a_lat_g: float) -> tuple[float, float]:
    """(roll of the body relative to the road deg, outside-wheel bump mm).

    The bump is the suspension's own, from the suspension roll gradient; the
    roll to the road adds the tire deflection, which moves no suspension
    travel but tilts the wheel against the road.
    """
    phi, z = roll(veh, a_lat_g)
    return phi + tire_roll_deg(veh, a_lat_g), z


def series_roll_stiffness(k_front: float, k_rear: float,
                          tire_rate_n_per_mm: float, track_mm: float
                          ) -> dict[str, float]:
    """Axle roll stiffness (N*m/deg) with each axle's tires in series.

    ``k_front``/``k_rear`` are the springs-plus-bars values. Returns the
    effective values, the tire roll stiffness per axle, the front share on
    rigid tires and with the tires in series.
    """
    kt = (tire_rate_n_per_mm * 1000.0 * (track_mm / 1000.0) ** 2 / 2.0
          * math.pi / 180.0)                                           # N*m/deg
    f = k_front * kt / (k_front + kt)
    r = k_rear * kt / (k_rear + kt)
    return {"tire_axle_Nm_per_deg": kt, "front": f, "rear": r,
            "total": f + r, "share_rigid": k_front / (k_front + k_rear),
            "share_series": f / (f + r)}


def aero_heave_mm(cla_m2: float, speed_kmh: float, wheel_rate_n_per_mm: float,
                  axle_share: float = 0.5, rho: float = 1.225) -> float:
    """Suspension compression per corner (mm) from downforce at a speed.

    Downforce 0.5 rho ClA v^2 (N), ``axle_share`` of it on this axle, split
    over two corners, against the corner's wheel rate (N/mm).
    """
    v = speed_kmh / 3.6
    F = 0.5 * rho * cla_m2 * v * v
    return F * axle_share / 2.0 / wheel_rate_n_per_mm


def camber_gain_needed(veh: Vehicle, tire: Tire, static_camber_deg: float
                       ) -> float:
    """Gain (deg/mm) that puts the loaded outside tire at its optimum."""
    phi, z = roll_to_road(veh, design_accelerations(tire)["lateral"])
    return (tire.camber_opt_deg - phi - static_camber_deg) / z


def static_camber_needed(veh: Vehicle, tire: Tire, gain_deg_per_mm: float
                         ) -> float:
    """Static camber (deg) that restores the optimum for a fixed gain.

    This is an alignment change: it moves no hardpoint.
    """
    phi, z = roll_to_road(veh, design_accelerations(tire)["lateral"])
    return tire.camber_opt_deg - phi - gain_deg_per_mm * z


def loaded_camber_error(veh: Vehicle, tire: Tire, static_camber_deg: float,
                        gain_deg_per_mm: float) -> float:
    """Loaded outside-tire camber minus the optimum, deg, at the design case."""
    phi, z = roll_to_road(veh, design_accelerations(tire)["lateral"])
    return static_camber_deg + gain_deg_per_mm * z + phi - tire.camber_opt_deg


def f_min(veh: Vehicle, anti_pct: float, a_x_g: float, a_y_g: float,
          axle: str) -> float:
    """Minimum ride frequency (Hz) that keeps a manoeuvre inside the budget."""
    m_s = veh.sprung_corner_front_kg if axle == "front" else veh.sprung_corner_rear_kg
    dW = veh.mass_kg * G0 * a_x_g * veh.cg_height_mm / (2.0 * veh.wheelbase_mm)
    room = veh.jounce_budget_mm - veh.aero_heave_mm - roll(veh, a_y_g)[1]
    if room <= 0.0:
        return float("inf")
    k = (1.0 - anti_pct / 100.0) * dW / (room / 1000.0)      # N/m
    return math.sqrt(k / m_s) / (2.0 * math.pi)


def anti_floor_pct(veh: Vehicle, ride_hz: float, a_x_g: float, a_y_g: float,
                   axle: str) -> float:
    """Minimum anti (%) for which ``ride_hz`` keeps the manoeuvre in budget."""
    m_s = veh.sprung_corner_front_kg if axle == "front" else veh.sprung_corner_rear_kg
    dW = veh.mass_kg * G0 * a_x_g * veh.cg_height_mm / (2.0 * veh.wheelbase_mm)
    room = veh.jounce_budget_mm - veh.aero_heave_mm - roll(veh, a_y_g)[1]
    if room <= 0.0:
        return float("inf")
    k = (2.0 * math.pi * ride_hz) ** 2 * m_s
    return 100.0 * (1.0 - k * (room / 1000.0) / dW)


def targets(veh: Vehicle = Vehicle(), tire: Tire = Tire(),
            static_front_deg: float = -2.5, static_rear_deg: float = -2.6
            ) -> dict[str, float]:
    """The tire-dependent targets of section 4 for one tire."""
    a = design_accelerations(tire)
    return {
        "camber_gain_front_deg_per_mm": camber_gain_needed(veh, tire, static_front_deg),
        "camber_gain_rear_deg_per_mm": camber_gain_needed(veh, tire, static_rear_deg),
        "anti_dive_floor_pct": anti_floor_pct(veh, veh.ride_front_hz,
                                              a["combined"], a["combined"], "front"),
        "anti_squat_floor_pct": anti_floor_pct(veh, veh.ride_rear_hz,
                                               a["combined"], a["combined"], "rear"),
    }


@dataclass(frozen=True)
class Corner:
    """What a reported corner delivers, for checking against moved targets."""
    name: str
    static_camber_deg: float
    camber_gain_deg_per_mm: float
    anti_pct: float
    axle: str


def tire_sensitivity(corners: list[Corner], veh: Vehicle = Vehicle(),
                     base: Tire = Tire(), span: float = 0.25) -> list[dict]:
    """Vary each tire input by +/- span (fractional) and report the effects.

    For each case and corner: the loaded camber error at the design case with
    the corner as built, the static camber that would restore the optimum
    (an alignment change), the grip that error costs at that case's camber
    sensitivity, and whether the corner's anti-geometry still clears the
    ride-frequency floor.
    """
    cases = [("reference", base)]
    for f in (1.0 - span, 1.0 + span):
        cases.append((f"camber optimum x{f:.2f}",
                      replace(base, camber_opt_deg=base.camber_opt_deg * f)))
        cases.append((f"peak mu x{f:.2f}", replace(base, peak_mu=base.peak_mu * f)))
    for f in (0.5, 5.0):
        cases.append((f"camber sensitivity x{f:g}",
                      replace(base, camber_sensitivity_pct_per_deg=
                              base.camber_sensitivity_pct_per_deg * f)))
    out = []
    for label, t in cases:
        a = design_accelerations(t)
        row = {"case": label, "tire": t, **targets(veh, t)}
        for c in corners:
            err = loaded_camber_error(veh, t, c.static_camber_deg,
                                      c.camber_gain_deg_per_mm)
            floor = (row["anti_dive_floor_pct"] if c.axle == "front"
                     else row["anti_squat_floor_pct"])
            row[c.name] = {
                "camber_error_deg": err,
                "static_to_restore_deg": static_camber_needed(
                    veh, t, c.camber_gain_deg_per_mm),
                "grip_cost_pct": abs(err) * t.camber_sensitivity_pct_per_deg,
                "anti_floor_pct": floor,
                "anti_margin_pct": c.anti_pct - floor,
            }
        out.append(row)
    return out


# --------------------------------------------------------------------------- #
#  Deriving the set-up values the targets were first declared with
# --------------------------------------------------------------------------- #
def ride_frequency_derived(veh: Vehicle, tire: Tire, anti_pct: float, axle: str,
                           mu_allowance: float = 0.10) -> float:
    """Ride frequency (Hz): the travel floor with an allowance on peak mu.

    A frequency above its floor costs mechanical grip over bumps and buys no
    travel, so the derived value is the combined-case floor evaluated with peak
    mu raised by ``mu_allowance`` (fraction) while the tire is not measured.
    """
    t = replace(tire, peak_mu=tire.peak_mu * (1.0 + mu_allowance))
    a = design_accelerations(t)
    return f_min(veh, anti_pct, a["combined"], a["combined"], axle)


def roll_gradient_ceiling(veh: Vehicle, tire: Tire, anti_pct: float, axle: str,
                          ride_hz: float, mu_allowance: float = 0.0) -> float:
    """Largest roll gradient (deg/g) at which ``ride_hz`` still holds the budget."""
    t = replace(tire, peak_mu=tire.peak_mu * (1.0 + mu_allowance))
    a = design_accelerations(t)
    lo, hi = 0.05, 3.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        ok = f_min(replace(veh, roll_gradient_deg_per_g=mid), anti_pct,
                   a["combined"], a["combined"], axle) < ride_hz
        lo, hi = (mid, hi) if ok else (lo, mid)
    return lo


def share_per_mm_front_rc(veh: Vehicle, front_stiffness_share: float = 0.522,
                          rear_weight_fraction: float = 0.52) -> float:
    """Front lateral load-transfer share (fraction) per mm of front roll-centre height.

    Linearised over the geometric and elastic paths: raising the front roll
    centre adds geometric transfer at the front and lowers the roll axis, which
    removes elastic transfer in proportion to the front stiffness share.
    """
    msf = 2.0 * veh.sprung_corner_front_kg
    ms = 2.0 * (veh.sprung_corner_front_kg + veh.sprung_corner_rear_kg)
    return (msf - front_stiffness_share * rear_weight_fraction * ms) / (
        veh.mass_kg * veh.cg_height_mm)


#: Where the tire numbers in this module come from. The load sensitivity below
#: is the synthetic MF5.2 set of the paper, not a fit to measured data; every
#: result that uses it (the neutral share, the understeer margin) must be
#: re-derived when a measured tire replaces it.
TIRE_PROVENANCE = "synthesised MF5.2 pure-lateral set; uncalibrated against measured data"


def mu_of_load(fz_N: float) -> float:
    """Synthetic tire peak mu (dimensionless) vs load: 1.66 at 550 N, 1.44 at 1650 N.

    Provenance: see TIRE_PROVENANCE; not measured.
    """
    return 1.66 - 0.0002 * (fz_N - 550.0)


def axle_capacity_ratio(veh: Vehicle, front_share: float, a_lat_g: float = 1.5,
                        front_weight_fraction: float = 0.48) -> float:
    """Front over rear normalised lateral capacity (dimensionless); < 1 is understeer."""
    lt = veh.mass_kg * G0 * a_lat_g * veh.cg_height_mm / veh.track_mm
    wf = veh.mass_kg * G0 * front_weight_fraction / 2.0
    wr = veh.mass_kg * G0 * (1.0 - front_weight_fraction) / 2.0
    cf = sum(mu_of_load(f) * f for f in (wf + front_share * lt, wf - front_share * lt)) / (2 * wf)
    cr = sum(mu_of_load(f) * f for f in (wr + (1 - front_share) * lt,
                                         wr - (1 - front_share) * lt)) / (2 * wr)
    return cf / cr


def neutral_front_share(veh: Vehicle, a_lat_g: float = 1.5) -> float:
    """Front load-transfer share (fraction) at which both axles saturate together."""
    lo, hi = 0.3, 0.8
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if axle_capacity_ratio(veh, mid, a_lat_g) > 1.0 else (lo, mid)
    return lo


# --------------------------------------------------------------------------- #
#  Hot inflation: pressure growth over a stint and the tire rate it moves
# --------------------------------------------------------------------------- #
def hot_pressure_kpa(p_cold_kpa: float, t_cold_c: float, t_hot_c: float,
                     p_atm_kpa: float = 101.325) -> float:
    """Gauge pressure (kPa) after the inflation gas heats at constant volume."""
    p_abs = (p_cold_kpa + p_atm_kpa) * (t_hot_c + 273.15) / (t_cold_c + 273.15)
    return p_abs - p_atm_kpa


def tire_rate_at_pressure(rate_cold_n_per_mm: float, p_cold_kpa: float,
                          p_hot_kpa: float, carcass_share: float = 0.3) -> float:
    """Tire vertical rate (N/mm) at a new gauge pressure.

    The rate is a carcass part, independent of pressure, plus an inflation
    part proportional to it: k(p) = k0 (s + (1 - s) p / p0), with s the
    carcass share at the cold pressure. ``carcass_share`` is DECLARED (0.2 to
    0.4 is typical of racing tires); a two-pressure load test fixes it.
    """
    s = float(carcass_share)
    return rate_cold_n_per_mm * (s + (1.0 - s) * p_hot_kpa / p_cold_kpa)


# --------------------------------------------------------------------------- #
#  Tire vertical load at each corner: every term, broken out
# --------------------------------------------------------------------------- #
def corner_loads(veh: "Vehicle", a_lat_g: float = 0.0, a_long_g: float = 0.0,
                 bank_deg: float = 0.0, grade_deg: float = 0.0,
                 downforce_n: float = 0.0, aero_front_share: float = 0.5,
                 front_lltd: float = 0.53, steer_warp_n: float = 0.0,
                 weight_front: float = 0.48) -> dict:
    """Vertical load (N) on each tire, FL FR RL RR, with each term separated.

    Fz = static + longitudinal transfer + lateral transfer + bank/grade +
    downforce + steer warp. Conventions: a_lat_g > 0 is a left turn (load to
    the right wheels), a_long_g > 0 is braking (load to the front). A bank
    raises the outside of the turn by ``bank_deg``: in the car's frame gravity
    then adds m g sin(beta) against the centripetal load, the normal load
    grows to m (g cos beta + a_y sin beta), and the lateral transfer uses the
    lateral load the tires actually carry, m (a_y cos beta - g sin beta). A
    grade of ``grade_deg`` (uphill > 0) tilts gravity rearward. Downforce is
    split front/rear by ``aero_front_share``. ``steer_warp_n`` is the
    diagonal load change for steering LEFT (``steer_warp``'s ``left_steer_N``):
    with positive caster, positive trail and positive scrub the outside (right)
    front patch rises relative to the chassis and the inside one drops, so
    steering left loads FL and RR and unloads FR and RL. Steering into a left
    turn therefore unloads the outer front; only counter-steer loads it, so
    pass a negative value for that case. Units: N, g, deg.
    """
    m, g = veh.mass_kg, G0
    L = veh.wheelbase_mm / 1000.0; h = veh.cg_height_mm / 1000.0
    t = veh.track_mm / 1000.0
    b, gr = math.radians(bank_deg), math.radians(grade_deg)
    ay, ax = a_lat_g * g, a_long_g * g
    normal = m * (g * math.cos(b) * math.cos(gr) + ay * math.sin(b))
    lat = m * (ay * math.cos(b) - g * math.sin(b))
    lon = m * ax - m * g * math.sin(gr)               # braking > 0, uphill < 0
    wf = float(weight_front)
    static = {"FL": normal * wf / 2, "FR": normal * wf / 2,
              "RL": normal * (1 - wf) / 2, "RR": normal * (1 - wf) / 2}
    dlon = lon * h / L / 2.0
    lon_t = {"FL": dlon, "FR": dlon, "RL": -dlon, "RR": -dlon}
    dlat = lat * h / t
    lat_t = {"FL": -front_lltd * dlat, "FR": front_lltd * dlat,
             "RL": -(1 - front_lltd) * dlat, "RR": (1 - front_lltd) * dlat}
    aero = {"FL": downforce_n * aero_front_share / 2,
            "FR": downforce_n * aero_front_share / 2,
            "RL": downforce_n * (1 - aero_front_share) / 2,
            "RR": downforce_n * (1 - aero_front_share) / 2}
    w = float(steer_warp_n)
    warp = {"FL": w, "FR": -w, "RL": -w, "RR": w}           # left steer, w > 0
    total = {c: static[c] + lon_t[c] + lat_t[c] + aero[c] + warp[c] for c in static}
    grav_static = m * g * np.array([wf / 2, wf / 2, (1 - wf) / 2, (1 - wf) / 2])
    bank_grade = {c: static[c] - grav_static[i] for i, c in enumerate(static)}
    return {"total": total, "static_flat": dict(zip(static, grav_static)),
            "bank_grade": bank_grade, "longitudinal": lon_t, "lateral": lat_t,
            "downforce": aero, "steer_warp": warp,
            "check_sum_N": sum(total.values()), "normal_N": normal + downforce_n}


def steer_warp(hp, rack_mm: float, k_roll_front_nm_per_deg: float,
               k_roll_rear_nm_per_deg: float, track_front_mm: float,
               track_rear_mm: float) -> dict:
    """Diagonal load change (N) from steering, through caster and KPI jacking.

    At fixed wheel travel (lower ball joint height) a steered wheel's contact
    patch moves vertically: caster lifts one front corner and drops the
    other, kingpin inclination lifts both. The difference is a front-axle
    roll input phi = (dz_right - dz_left) / t_f that the body resists through
    the front and rear roll stiffnesses in series, so the diagonal pair
    carries dF = phi K_f K_r / (K_f + K_r) / t (front at t_f, rear at t_r).
    The common part lifts the front evenly and changes no corner weight on a
    rigid body. The left wheel at +rack is the mirrored right wheel at -rack.
    """
    from .kinematics import SuspensionKinematics

    def patch_z(r):
        """Contact-patch height in mm at a rack travel in mm."""
        k = SuspensionKinematics(hp, pickup_deltas={
            "tie_rod_inner": np.array([0.0, float(r), 0.0])})
        return float(k.solve_at_travel(0.0).contact_patch[2])

    z0 = patch_z(0.0)
    dzR, dzL = patch_z(rack_mm) - z0, patch_z(-rack_mm) - z0
    phi = (dzR - dzL) / track_front_mm                       # rad
    kf = k_roll_front_nm_per_deg * 180.0 / math.pi           # N*m/rad
    kr = k_roll_rear_nm_per_deg * 180.0 / math.pi
    M = phi * kf * kr / (kf + kr)                            # N*m
    # ``hp`` is a right-hand corner (y outboard); +rack toes it in, a LEFT
    # steer, so dzR is the outside wheel and dzL (the mirror at -rack) the
    # inside one. The patch height change is sin(delta) (s sin(caster) + t
    # sin(KPI)) to first order: positive scrub s and trail t lift the
    # outside patch, dropping that corner of the body. A positive moment M
    # therefore loads the FL-RR diagonal when steering left.
    pair = M / (track_front_mm / 1000.0)
    return {"dz_right_mm": dzR, "dz_left_mm": dzL, "phi_deg": math.degrees(phi),
            "left_steer_N": {"FL": pair, "FR": -pair, "RL": -M / (track_rear_mm / 1000.0), "RR": M / (track_rear_mm / 1000.0)},
            "warp_moment_Nm": M, "front_pair_N": M / (track_front_mm / 1000.0),
            "rear_pair_N": M / (track_rear_mm / 1000.0),
            "lift_common_mm": 0.5 * (dzR + dzL)}


def front_lltd_share(rc_front_mm: float, rc_rear_mm: float, front_stiffness_share: float,
                     cg_height_mm: float = 280.0, weight_front: float = 0.48) -> float:
    """Front share (dimensionless) of lateral load transfer from roll centres and stiffness.

    Geometric transfer goes through each axle's roll centre with that axle's
    weight; the elastic part, the CG height over the roll axis at the CG
    station, splits by the front roll-stiffness share. Heights in mm.
    """
    h_ra = rc_front_mm + (rc_rear_mm - rc_front_mm) * (1.0 - weight_front)
    return (weight_front * rc_front_mm + front_stiffness_share * (cg_height_mm - h_ra)) / cg_height_mm


def stiffness_share_for_lltd(target: float, rc_front_mm: float, rc_rear_mm: float,
                             cg_height_mm: float = 280.0, weight_front: float = 0.48) -> float:
    """Front roll-stiffness share (dimensionless) giving a front LLTD target; heights in mm."""
    h_ra = rc_front_mm + (rc_rear_mm - rc_front_mm) * (1.0 - weight_front)
    return (target * cg_height_mm - weight_front * rc_front_mm) / (cg_height_mm - h_ra)
