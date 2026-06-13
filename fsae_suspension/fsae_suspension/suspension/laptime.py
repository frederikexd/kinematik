"""
Lap-time simulation — turn the grip envelope into the only number that wins:
seconds.

Everything else in KinematiK reports steady-state grip at a single operating
point. But competition is decided by *lap time*, which is a transient, track-
dependent integral of that grip envelope. A better-funded team buys that integral
empirically by testing fresh rubber all season. An underfunded team that can only
run ONE tire set has to predict it instead — and predicting it well, before the
build is frozen, is the single highest-leverage thing the software can do.

This module is a quasi-steady-state (QSS) point-mass lap simulator. It takes the
*same* `VehicleDynamics` object the rest of the tool already builds — so every
geometry / setup / tire change you make upstream flows straight through to a
predicted time — and runs it around:

    * the FSAE skidpad (a fixed-radius circle: the cleanest possible validation
      case, with a closed-form steady-state answer), and
    * a parameterisable autocross / track defined as a sequence of segments
      (straights + constant-radius corners), solved with a standard three-pass
      QSS method: per-corner limit speeds, a forward acceleration pass, a backward
      braking pass, then integrate dt = ds / v.

QSS is deliberate. A full transient model needs tyre relaxation, yaw inertia, and
combined-slip data we don't have on one tyre set; QSS needs only the lateral grip
envelope we already trust and a defensible longitudinal model, and on FSAE-scale
tracks it lands within a few percent — accurate enough to RANK setups, which is
what actually moves you up the results sheet. It pairs with the setup optimiser:
optimise for max grip, then confirm the change is worth seconds here.

DESIGN RULE FOR THIS MODULE: never let one bad data point kill a session. Every
public function is wrapped so that if a calculation can't complete it returns a
safe, clearly-flagged default (with a `warning` string) instead of raising. A lap
sim is run interactively on geometry the user is actively dragging around — a
non-convergent linkage or a degenerate track must surface a warning in the UI, not
crash the app.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import List, Optional

import numpy as np

from .dynamics import VehicleDynamics, VehicleParams


# --------------------------------------------------------------------------- #
#  Powertrain / longitudinal envelope
# --------------------------------------------------------------------------- #
@dataclass
class Powertrain:
    """
    A deliberately simple longitudinal model — enough to make the straights and
    the corner-exit acceleration realistic without pretending we have a motor map
    we don't. All fields have FSAE-EV-representative defaults; override with your
    own numbers on the Lap Sim tab.

    Tractive force is the lesser of (a) what the tyres can put down — mu * rear (or
    all-wheel) vertical load — and (b) what the motor delivers at the current speed
    (power-limited: F = P / v, capped by a low-speed torque limit). Drag and rolling
    resistance oppose. Braking is grip-limited on all four tyres.
    """
    power_kw: float = 80.0           # peak electric power at the wheels, kW
    max_tractive_n: float = 2600.0   # low-speed torque/traction cap, N
    drivetrain_eff: float = 0.90     # wheel power / battery power
    cda: float = 1.10                # drag area Cd*A, m^2
    cla: float = 2.60                # downforce area Cl*A, m^2 (aero pkg; 0 if none)
    rho: float = 1.20                # air density, kg/m^3
    crr: float = 0.018               # rolling resistance coefficient
    drive: str = "rwd"               # "rwd" or "awd" — which axle loads cap traction
    brake_g_cap: float = 1.8         # mechanical brake ceiling (g), grip-limited below

    def power_w(self) -> float:
        return max(self.power_kw, 0.0) * 1000.0 * max(self.drivetrain_eff, 0.05)


# --------------------------------------------------------------------------- #
#  Track description
# --------------------------------------------------------------------------- #
@dataclass
class Segment:
    """One piece of track. A straight has radius=None; a corner has radius_m>0."""
    length_m: float
    radius_m: Optional[float] = None   # None / <=0 => straight

    @property
    def is_corner(self) -> bool:
        return self.radius_m is not None and self.radius_m > 0.0


@dataclass
class Track:
    name: str
    segments: List[Segment] = field(default_factory=list)
    ds: float = 1.0                    # integration step, m

    def total_length(self) -> float:
        return float(sum(max(s.length_m, 0.0) for s in self.segments))


# --------------------------------------------------------------------------- #
#  Result container
# --------------------------------------------------------------------------- #
@dataclass
class LapResult:
    lap_time_s: float
    avg_speed_ms: float
    top_speed_ms: float
    min_speed_ms: float
    distance_m: float
    # per-station traces (for plotting); always finite, may be empty on failure
    s: list = field(default_factory=list)
    v: list = field(default_factory=list)
    ok: bool = True
    warning: str = ""

    def as_summary(self) -> dict:
        return dict(lap_time_s=round(self.lap_time_s, 3),
                    avg_speed_ms=round(self.avg_speed_ms, 2),
                    top_speed_ms=round(self.top_speed_ms, 2),
                    min_speed_ms=round(self.min_speed_ms, 2),
                    distance_m=round(self.distance_m, 1),
                    ok=self.ok, warning=self.warning)


def _safe_lap(distance=0.0, warning="calculation unavailable") -> LapResult:
    """A finite, non-crashing placeholder result with a surfaced warning."""
    return LapResult(lap_time_s=float("nan"), avg_speed_ms=0.0, top_speed_ms=0.0,
                     min_speed_ms=0.0, distance_m=float(distance), s=[], v=[],
                     ok=False, warning=warning)


# --------------------------------------------------------------------------- #
#  Core grip lookups (wrapped so a bad geometry never throws)
# --------------------------------------------------------------------------- #
def _max_lat_g(veh: VehicleDynamics) -> float:
    """Steady-state lateral g from the live dynamics model, guarded."""
    try:
        g = float(veh.max_lateral_g())
        if not math.isfinite(g) or g <= 0.0:
            return 1.4  # safe representative fallback, flagged by caller
        return g
    except Exception:
        return 1.4


def _corner_limit_speed(veh: VehicleDynamics, radius_m: float, pt: Powertrain,
                        max_lat_g: float) -> float:
    """
    Max speed through a constant-radius corner. With aero downforce, grip grows
    with speed, so the limit is the fixed point of:
        v^2 / R = a_lat(v) = max_lat_g * g * (1 + downforce/weight)
    Solve directly (downforce ∝ v^2 makes this closed-form).
    """
    if radius_m <= 0.0 or not math.isfinite(radius_m):
        return float("inf")  # treat as straight
    g = 9.81
    m = max(veh.p.mass, 1.0)
    W = m * g
    a0 = max_lat_g * g                      # grip accel at zero aero, m/s^2
    # downforce accel coefficient: F_down = 0.5*rho*ClA*v^2 ; extra a_lat = mu*F/m
    # approximate mu ~ max_lat_g (grip already includes load sensitivity envelope)
    k = 0.5 * pt.rho * max(pt.cla, 0.0) / m  # downforce / v^2 per unit mass
    # v^2/R = a0 + max_lat_g * k * v^2   ->  v^2 (1/R - mu*k) = a0
    denom = (1.0 / radius_m) - max_lat_g * k
    if denom <= 1e-9:
        # aero would (unphysically) let speed run away; cap at no-aero solution
        v2 = a0 * radius_m
    else:
        v2 = a0 / denom
    v2 = max(v2, 0.0)
    return math.sqrt(v2)


def _accel_long(veh: VehicleDynamics, v: float, pt: Powertrain,
                max_lat_g: float, lat_used_g: float) -> float:
    """
    Available longitudinal acceleration (m/s^2) at speed v, accounting for the
    grip already spent cornering (friction-circle coupling) plus power, drag and
    rolling resistance. Used in the forward pass.
    """
    g = 9.81
    m = max(veh.p.mass, 1.0)
    v = max(v, 0.1)
    # vertical load with aero
    F_down = 0.5 * pt.rho * max(pt.cla, 0.0) * v * v
    # longitudinal grip ceiling (driven axle share)
    axle_frac = 1.0 if pt.drive == "awd" else (1.0 - veh.p.weight_dist_front)
    N_drive = (m * g + F_down) * (axle_frac if pt.drive != "awd" else 1.0)
    mu = max(max_lat_g, 0.3)
    F_grip = mu * N_drive
    # friction circle: subtract lateral usage
    frac_lat = min(lat_used_g / max(max_lat_g, 1e-6), 1.0)
    long_grip_frac = math.sqrt(max(1.0 - frac_lat * frac_lat, 0.0))
    F_grip *= long_grip_frac
    # power limit
    F_power = pt.power_w() / v
    F_drive = min(F_grip, F_power, pt.max_tractive_n)
    # resistances
    F_drag = 0.5 * pt.rho * pt.cda * v * v
    F_roll = pt.crr * (m * g + F_down)
    a = (F_drive - F_drag - F_roll) / m
    return a


def _decel_long(veh: VehicleDynamics, v: float, pt: Powertrain,
                max_lat_g: float, lat_used_g: float) -> float:
    """
    Available braking deceleration (m/s^2, positive number) at speed v under the
    same friction-circle coupling. Drag and downforce *help* braking.
    """
    g = 9.81
    m = max(veh.p.mass, 1.0)
    v = max(v, 0.1)
    F_down = 0.5 * pt.rho * max(pt.cla, 0.0) * v * v
    mu = max(max_lat_g, 0.3)
    F_grip = mu * (m * g + F_down)          # all four tyres brake
    frac_lat = min(lat_used_g / max(max_lat_g, 1e-6), 1.0)
    long_grip_frac = math.sqrt(max(1.0 - frac_lat * frac_lat, 0.0))
    F_grip *= long_grip_frac
    F_brake = min(F_grip, pt.brake_g_cap * m * g)
    F_drag = 0.5 * pt.rho * pt.cda * v * v
    a = (F_brake + F_drag) / m
    return a


# --------------------------------------------------------------------------- #
#  Skidpad — the clean closed-form case
# --------------------------------------------------------------------------- #
# FSAE skidpad: two 15.25 m centreline-diameter circles in a figure-8; the timed
# run is one full lap of one circle. Standard path radius ~ 9.125 m (8.5 m inner
# circle radius + ~0.625 m to the tyre centreline track). We use the commonly
# cited timed-circle radius and circumference.
SKIDPAD_RADIUS_M = 9.125
SKIDPAD_CIRCUMFERENCE_M = 2.0 * math.pi * SKIDPAD_RADIUS_M


def skidpad_time(veh: VehicleDynamics, pt: Optional[Powertrain] = None,
                 radius_m: float = SKIDPAD_RADIUS_M) -> LapResult:
    """
    Predicted FSAE skidpad time for one timed circle, from the live grip model.
    Steady-state and closed-form: v = sqrt(a_lat * R), t = circumference / v.
    This is the cleanest possible check that the whole grip stack is sane — you
    can sanity-check it by hand and against your own skidpad runs.
    """
    try:
        pt = pt or Powertrain()
        if radius_m <= 0 or not math.isfinite(radius_m):
            return _safe_lap(warning="skidpad radius invalid; check inputs")
        max_lat_g = _max_lat_g(veh)
        v = _corner_limit_speed(veh, radius_m, pt, max_lat_g)
        if not math.isfinite(v) or v <= 0.0:
            return _safe_lap(warning="grip model returned no usable speed; "
                                     "using safe default — check geometry/tire")
        circ = 2.0 * math.pi * radius_m
        t = circ / v
        return LapResult(lap_time_s=t, avg_speed_ms=v, top_speed_ms=v,
                         min_speed_ms=v, distance_m=circ,
                         s=[0.0, circ], v=[v, v], ok=True,
                         warning="" if max_lat_g != 1.4 else
                                 "grip fell back to default 1.4 g — verify tire/geometry")
    except Exception as e:                       # never crash the session
        return _safe_lap(warning=f"skidpad sim failed safely: {e}")


# --------------------------------------------------------------------------- #
#  General QSS lap over an arbitrary track
# --------------------------------------------------------------------------- #
def simulate_lap(veh: VehicleDynamics, track: Track,
                 pt: Optional[Powertrain] = None) -> LapResult:
    """
    Quasi-steady-state lap time over `track`. Three passes:
      1. corner limit speed at every station (vertical asymptote of grip),
      2. forward pass: cap acceleration out of corners by available traction,
      3. backward pass: cap entry speed by available braking,
    then integrate dt = ds / v_mean. Returns a LapResult with traces; on any
    failure returns a flagged safe default rather than raising.
    """
    try:
        pt = pt or Powertrain()
        if not track.segments or track.total_length() <= 0.0:
            return _safe_lap(warning="track is empty; add at least one segment")

        ds = track.ds if track.ds and track.ds > 0 else 1.0
        max_lat_g = _max_lat_g(veh)

        # Build station list: position s, local radius (inf for straight)
        s_pts, radii = [], []
        s_cur = 0.0
        for seg in track.segments:
            L = max(seg.length_m, 0.0)
            r = seg.radius_m if seg.is_corner else float("inf")
            n = max(int(round(L / ds)), 1)
            for _ in range(n):
                s_pts.append(s_cur)
                radii.append(r)
                s_cur += L / n
        s_pts.append(s_cur)
        radii.append(radii[-1] if radii else float("inf"))
        N = len(s_pts)
        if N < 2:
            return _safe_lap(warning="track too short to integrate")

        # Pass 1: cornering speed ceiling at each station
        v_ceiling = np.empty(N)
        for i in range(N):
            r = radii[i]
            if math.isinf(r):
                v_ceiling[i] = 1e6   # straight: no cornering limit (capped later)
            else:
                v_ceiling[i] = _corner_limit_speed(veh, r, pt, max_lat_g)
        # clamp non-finite
        v_ceiling = np.where(np.isfinite(v_ceiling), v_ceiling, 1e6)
        v_ceiling = np.clip(v_ceiling, 0.0, 1e6)

        # Pass 2: forward (acceleration-limited), closed loop -> seed from ceiling
        v_fwd = v_ceiling.copy()
        # iterate twice for the closed lap so the start speed is consistent
        for _ in range(2):
            for i in range(1, N):
                ds_i = max(s_pts[i] - s_pts[i - 1], 1e-3)
                v0 = v_fwd[i - 1]
                lat_used = (v0 * v0 / radii[i - 1] / 9.81) if math.isfinite(radii[i - 1]) else 0.0
                a = _accel_long(veh, v0, pt, max_lat_g, lat_used)
                v_next = math.sqrt(max(v0 * v0 + 2.0 * a * ds_i, 0.0))
                v_fwd[i] = min(v_next, v_ceiling[i])
            v_fwd[0] = min(v_fwd[0], v_fwd[-1])  # close the loop

        # Pass 3: backward (braking-limited)
        v = v_fwd.copy()
        for _ in range(2):
            for i in range(N - 2, -1, -1):
                ds_i = max(s_pts[i + 1] - s_pts[i], 1e-3)
                v1 = v[i + 1]
                lat_used = (v1 * v1 / radii[i + 1] / 9.81) if math.isfinite(radii[i + 1]) else 0.0
                d = _decel_long(veh, v1, pt, max_lat_g, lat_used)
                v_prev = math.sqrt(max(v1 * v1 + 2.0 * d * ds_i, 0.0))
                v[i] = min(v[i], v_prev, v_ceiling[i])
            v[-1] = min(v[-1], v[0])

        v = np.clip(v, 0.05, 1e6)   # avoid div-by-zero in dt

        # Integrate time: dt = ds / v_mean over each interval
        t = 0.0
        for i in range(1, N):
            ds_i = max(s_pts[i] - s_pts[i - 1], 0.0)
            v_mean = 0.5 * (v[i] + v[i - 1])
            t += ds_i / max(v_mean, 0.05)

        dist = s_pts[-1]
        warning = ""
        if max_lat_g == 1.4:
            warning = "grip fell back to default 1.4 g — verify tire/geometry"
        return LapResult(
            lap_time_s=float(t),
            avg_speed_ms=float(dist / t) if t > 0 else 0.0,
            top_speed_ms=float(np.max(v)),
            min_speed_ms=float(np.min(v)),
            distance_m=float(dist),
            s=[float(x) for x in s_pts],
            v=[float(x) for x in v],
            ok=True, warning=warning)
    except Exception as e:
        return _safe_lap(warning=f"lap sim failed safely: {e}")


# --------------------------------------------------------------------------- #
#  A representative autocross track (parameterisable)
# --------------------------------------------------------------------------- #
def default_autocross(scale: float = 1.0) -> Track:
    """
    A representative FSAE-style autocross lap: ~800 m mixing slow hairpins, a
    slalom (modelled as a chain of short alternating-radius corners), sweepers and
    short straights. It is NOT a specific competition map — it's a fixed, sensible
    yardstick so that comparing two setups on it is an apples-to-apples ranking.
    `scale` stretches every length if you want a longer/shorter lap.
    """
    R = lambda r: max(r, 0.1)
    segs = [
        Segment(40 * scale),                 # start straight
        Segment(18 * scale, R(9.0)),         # right sweeper
        Segment(25 * scale),
        Segment(12 * scale, R(5.0)),         # hairpin
        Segment(20 * scale),
        # slalom: alternating tight corners
        *[Segment(6 * scale, R(7.0)) for _ in range(6)],
        Segment(30 * scale),                 # back straight
        Segment(15 * scale, R(12.0)),        # fast sweeper
        Segment(22 * scale),
        Segment(10 * scale, R(6.0)),         # medium corner
        Segment(35 * scale),                 # straight
        Segment(11 * scale, R(4.5)),         # tight hairpin
        Segment(18 * scale),
        Segment(14 * scale, R(8.0)),         # sweeper to finish
        Segment(28 * scale),                 # finish straight
    ]
    return Track(name=f"Representative autocross (×{scale:g})", segments=segs, ds=1.0)


def event_points_estimate(your_time: float, best_time: float,
                          event: str = "autocross") -> float:
    """
    FSAE-style points estimate so a time delta reads as what it's worth on the
    scoresheet. Uses the standard dynamic-event shape: points scale between a
    floor at the max allowed time (~145% of best) and the max at the best time.
    Returns a points estimate (0..~max) — indicative, for prioritisation only.
    """
    try:
        maxpts = {"skidpad": 75.0, "autocross": 125.0,
                  "endurance": 275.0}.get(event, 100.0)
        minpts = 0.05 * maxpts
        if your_time <= 0 or best_time <= 0 or not math.isfinite(your_time):
            return 0.0
        t_max = 1.45 * best_time
        if your_time >= t_max:
            return minpts
        frac = (t_max / your_time - 1.0) / (t_max / best_time - 1.0)
        return float(minpts + (maxpts - minpts) * max(0.0, min(frac, 1.0)))
    except Exception:
        return 0.0
