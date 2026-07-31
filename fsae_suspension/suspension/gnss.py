# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/gnss.py — the GPS/IMU logging node as a checkable channel group.
#  Does the two pieces of arithmetic a GPS-to-CAN spec sheet implies and never
#  performs: what the mounting offset from the centre of gravity does to the
#  measured accelerations, and what the velocity latency does to position.
# ============================================================================
"""
GNSS + IMU — "mount it near the CG" and "1 Mbps", made checkable.

WHY THIS MODULE EXISTS
----------------------
A GPS-to-CAN node arrives as a specification sheet: update rates, accuracies,
an operating voltage, a connector part number, a bitrate. Every line reads like
a settled fact, and two of them are not facts at all — they are requirements
placed on the team, stated in a form that makes them easy to nod at.

**"Mounted: as close to the car's centre of gravity as possible."**
This is a tolerance with no number, and the cost of missing it is not a small
offset. An IMU displaced from the CG by a lever arm *r* measures the CG's
acceleration plus two rotational terms it cannot distinguish from real vehicle
response:

    a_measured = a_cg + alpha x r + omega x (omega x r)

The centripetal term omega x (omega x r) scales with yaw rate squared, so on a
skidpad it is a steady bias that looks like a calibration offset. The angular
term alpha x r scales with yaw *acceleration*, so in a slalom it is transient,
it peaks exactly when the driver turns, and it therefore correlates with
steering input — which is precisely what makes it undetectable by eye. A 0.5 m
longitudinal offset in a hard slalom manufactures a few tenths of a g of
lateral acceleration that was never there, and it arrives with the right phase
to be mistaken for real understeer.

The mitigation is cheap and almost never done: measure *r* to a few
millimetres, write it down, and subtract the terms in post using the yaw rate
the same device already reports. What makes it not happen is that nobody ever
computes how big the error is, so it stays a vague preference for mounting
things "near the middle". `lever_arm_error` computes it.

**"CAN bus bitrate: 1 Mbps."**
A vehicle bus running at 500 kbps does not slow this device down or drop some
of its frames. It does not communicate with it at all, and the failure is not
quiet — a node transmitting at the wrong bitrate produces error frames that
degrade the bus for every *other* node on it. This is a hard incompatibility
between two numbers on two different spec sheets, discoverable in five seconds
and, in practice, discovered during the first test session.

**And the spec sheet's blanks are the point.**
The measurement list — velocity accuracy, resolution, latency, position and
height and heading accuracy — is a list of *questions*. Left as bullet points
with no values, they cannot support any claim about what the data is good for.
This module keeps them Optional and reports each blank as its own MISSING
finding, in the same spirit as `daq_plan.SensorSpec`: a blank cell and a
considered answer must never look alike.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from .interfaces import Severity, Finding
from .daq_plan import (
    CanMessage, BusSpec, SensorSpec, OutputType, Rail,
    NYQUIST_PRACTICAL, can_frame_bits,
)


# ===================================================================== #
#  1.  THE DEVICE SPEC — every measurement question, Optional
# ===================================================================== #
@dataclass
class GnssSpec:
    """A GPS/IMU node, and every question the review asks about it.

    Split deliberately into what the vendor states (rates, voltage, connector,
    bitrate — facts) and what the review must extract from the manual or a
    bench test (accuracies, resolutions, latency — questions). The second group
    all default to None, and None means unanswered.
    """
    key: str
    name: str

    # --- vendor-stated interface ------------------------------------------ #
    position_rate_hz: Optional[float] = None
    imu_rate_hz: Optional[float] = None
    can_bitrate_bps: Optional[float] = None
    can_extended_ids: Optional[bool] = None      # 2.0B == 29-bit identifiers
    supply_v_min: Optional[float] = None
    supply_v_max: Optional[float] = None
    current_ma: Optional[float] = None
    connector: Optional[str] = None
    mating_connector: Optional[str] = None
    external_antenna: Optional[bool] = None      # the doc's open question
    cost_usd: Optional[float] = None

    # --- the measurement questions ---------------------------------------- #
    velocity_accuracy_ms: Optional[float] = None
    velocity_max_ms: Optional[float] = None
    velocity_resolution_ms: Optional[float] = None
    velocity_latency_s: Optional[float] = None
    position_accuracy_m: Optional[float] = None
    height_accuracy_m: Optional[float] = None
    heading_accuracy_deg: Optional[float] = None
    heading_resolution_deg: Optional[float] = None

    # --- installation ------------------------------------------------------ #
    #: Offset from the CG in vehicle axes, metres: x forward, y right, z up.
    #: None means not measured — which is the usual state, and the reason the
    #: lever-arm correction never gets applied.
    offset_from_cg_m: Optional[tuple[float, float, float]] = None
    calibration: Optional[str] = None
    owner: str = ""
    source: str = ""
    notes: str = ""

    # ------------------------------------------------------------------ #
    #: The measurement questions, as (attribute, human label) pairs. This is
    #: the list the spec sheet prints as bullet points; keeping it here means
    #: adding a question makes every existing device incomplete until answered.
    MEASUREMENT_QUESTIONS = (
        ("velocity_accuracy_ms", "Velocity accuracy"),
        ("velocity_max_ms", "Velocity maximum"),
        ("velocity_resolution_ms", "Velocity resolution"),
        ("velocity_latency_s", "Velocity latency"),
        ("position_accuracy_m", "Position accuracy"),
        ("height_accuracy_m", "Height accuracy"),
        ("heading_accuracy_deg", "Heading accuracy"),
        ("heading_resolution_deg", "Heading resolution"),
    )

    def unanswered_measurements(self) -> list[str]:
        return [label for attr, label in self.MEASUREMENT_QUESTIONS
                if getattr(self, attr, None) is None]

    def measurement_completeness(self) -> float:
        n = len(self.MEASUREMENT_QUESTIONS)
        return (n - len(self.unanswered_measurements())) / n

    def lever_arm_m(self) -> Optional[float]:
        if self.offset_from_cg_m is None:
            return None
        x, y, z = self.offset_from_cg_m
        return math.sqrt(x * x + y * y + z * z)


#: The COTS option, filled in exactly as far as the vendor sheet goes and no
#: further. Every measurement field is None on purpose: the sheet lists those
#: as headings, and inventing plausible numbers for them here would defeat the
#: whole module.
ECUMASTER_GPS_TO_CAN_V2 = GnssSpec(
    key="gps_can_v2", name="ECUMaster GPS to CAN V2",
    position_rate_hz=25.0, imu_rate_hz=100.0,
    can_bitrate_bps=1_000_000.0, can_extended_ids=True,
    supply_v_min=6.0, supply_v_max=22.0,
    connector="Deutsch DT06-4S (plug) / DT15-4P (socket)",
    mating_connector="F02U.B00.656-01",
    cost_usd=480.0,
    calibration="USB to a laptop; zero every channel stationary on a flat "
                "surface, then move the unit through each axis and confirm "
                "the signs and magnitudes change as expected",
    source="ECUMaster GPS to CAN V2 manual",
    notes="Antenna requirement not yet confirmed. External antenna is not a "
          "detail — it decides whether this needs a roll-hoop mount, a "
          "coaxial run and a ground plane, or nothing at all.")


# ===================================================================== #
#  2.  LEVER ARM — the number behind "mount it near the CG"
# ===================================================================== #
@dataclass
class LeverArmResult:
    """Spurious acceleration an offset IMU reports, in vehicle axes."""
    offset_m: tuple[float, float, float]
    yaw_rate_rads: float
    yaw_accel_rads2: float
    a_err_x_ms2: float           # longitudinal
    a_err_y_ms2: float           # lateral
    centripetal_ms2: float       # the steady, yaw-rate-squared part
    angular_ms2: float           # the transient, yaw-acceleration part

    def a_err_g(self) -> tuple[float, float]:
        return (self.a_err_x_ms2 / 9.80665, self.a_err_y_ms2 / 9.80665)

    def magnitude_g(self) -> float:
        gx, gy = self.a_err_g()
        return math.sqrt(gx * gx + gy * gy)


def lever_arm_error(offset_m: tuple[float, float, float],
                    yaw_rate_rads: float,
                    yaw_accel_rads2: float = 0.0) -> LeverArmResult:
    """Acceleration an IMU at `offset_m` reads that the CG does not experience.

    Planar yaw-only rigid-body kinematics, which is the dominant case for a
    car on a flat surface:

        a_meas = a_cg + alpha x r + omega x (omega x r)

    with omega = (0, 0, w) and r = (x, y, z) in vehicle axes (x forward,
    y right, z up), giving

        err_x = -alpha*y - w^2 * x
        err_y = +alpha*x - w^2 * y

    Roll and pitch rates add further terms; they are smaller on a car than on
    the yaw axis and are deliberately not folded in silently. If you are
    chasing a vertical-axis discrepancy, this function is not the whole story.
    """
    x, y, _z = offset_m
    w2 = yaw_rate_rads * yaw_rate_rads
    err_x = -yaw_accel_rads2 * y - w2 * x
    err_y = yaw_accel_rads2 * x - w2 * y
    centripetal = w2 * math.sqrt(x * x + y * y)
    angular = abs(yaw_accel_rads2) * math.sqrt(x * x + y * y)
    return LeverArmResult(offset_m, yaw_rate_rads, yaw_accel_rads2,
                          err_x, err_y, centripetal, angular)


@dataclass(frozen=True)
class Manoeuvre:
    """A representative event, for turning a mounting offset into a number."""
    name: str
    speed_ms: float
    radius_m: Optional[float]        # None for straight-line
    yaw_accel_rads2: float
    why: str
    derived: bool = False            # True when computed from a VehicleSpec

    def yaw_rate_rads(self) -> float:
        if not self.radius_m:
            return 0.0
        return self.speed_ms / self.radius_m


@dataclass
class VehicleSpec:
    """The car, to the extent needed to bound its yaw behaviour.

    Every field Optional, same rule as everywhere else: None is unanswered. The
    point of this class is that the lever-arm result stops depending on numbers
    this module invented and starts depending on numbers the team declared.
    """
    mass_kg: Optional[float] = None
    wheelbase_m: Optional[float] = None
    #: Front axle to CG, metres. With wheelbase this gives the rear distance.
    cg_to_front_m: Optional[float] = None
    #: Peak lateral tyre coefficient actually achieved, not the datasheet peak.
    mu_lat: Optional[float] = None
    #: Yaw inertia about the CG. If None, estimated — see `yaw_inertia_kgm2`.
    yaw_inertia_kgm2: Optional[float] = None
    #: Speeds at which to evaluate. Corner radii come from the grip limit.
    speeds_ms: tuple[float, ...] = (11.0, 15.0, 20.0)
    source: str = ""

    def cg_to_rear_m(self) -> Optional[float]:
        if self.wheelbase_m is None or self.cg_to_front_m is None:
            return None
        return self.wheelbase_m - self.cg_to_front_m

    def yaw_inertia_estimate_kgm2(self) -> Optional[float]:
        """I_zz ~= m * a * b, the dynamic-index-one approximation.

        The dynamic index is k^2/(a*b), where k is the yaw radius of gyration.
        It sits near unity for most passenger and formula cars, and assuming
        exactly one is the standard first estimate. It is an estimate, and
        `yaw_inertia_kgm2` overrides it the moment anyone does a bifilar swing
        test.
        """
        b = self.cg_to_rear_m()
        if self.mass_kg is None or self.cg_to_front_m is None or b is None:
            return None
        return self.mass_kg * self.cg_to_front_m * b

    def yaw_inertia(self) -> Optional[float]:
        return self.yaw_inertia_kgm2 or self.yaw_inertia_estimate_kgm2()

    def max_yaw_accel_rads2(self) -> Optional[float]:
        """Peak yaw acceleration the tyres can produce, from the parameters.

        Bound the yaw moment by one axle saturated laterally while the other
        contributes nothing — the transient state entering a slalom reversal:

            M_z = mu * N_front * a = mu * m * g * (b/L) * a

        Divide by the yaw inertia. If I_zz is the dynamic-index-one estimate
        m*a*b, the a*b cancels completely and the result reduces to

            alpha_max = mu * g / L

        which is worth noticing: to first order the peak yaw acceleration of a
        car depends on grip and wheelbase alone, not on where the CG sits or
        what it weighs. A short, grippy car changes direction violently, and
        that is exactly the car an FSAE team builds.
        """
        a, b, I = self.cg_to_front_m, self.cg_to_rear_m(), self.yaw_inertia()
        if (self.mu_lat is None or self.mass_kg is None
                or a is None or b is None or I is None
                or self.wheelbase_m is None):
            return None
        m_z = self.mu_lat * self.mass_kg * 9.80665 * (b / self.wheelbase_m) * a
        return m_z / I

    def max_lat_accel_ms2(self) -> Optional[float]:
        if self.mu_lat is None:
            return None
        return self.mu_lat * 9.80665

    def missing_fields(self) -> list[str]:
        need = {"mass_kg": self.mass_kg, "wheelbase_m": self.wheelbase_m,
                "cg_to_front_m": self.cg_to_front_m, "mu_lat": self.mu_lat}
        return [k for k, v in need.items() if v is None]

    def manoeuvres(self) -> Optional[tuple[Manoeuvre, ...]]:
        """Derived manoeuvre set, or None if the car is not declared enough.

        Corner radius is not assumed either — it follows from the speed and the
        grip limit, R = v^2 / a_lat_max, which is the tightest corner the car
        can actually hold at that speed.
        """
        a_lat = self.max_lat_accel_ms2()
        alpha = self.max_yaw_accel_rads2()
        if a_lat is None or alpha is None:
            return None
        out: list[Manoeuvre] = []
        for i, v in enumerate(self.speeds_ms):
            r = v * v / a_lat
            # Steady case: at the grip limit, no yaw acceleration.
            out.append(Manoeuvre(
                f"steady {v:.0f} m/s", v, r, 0.0,
                "cornering at the grip limit — the error is a constant offset "
                "that reads as a miscalibrated sensor",
                derived=True))
            # Transient case: same corner, peak yaw acceleration.
            out.append(Manoeuvre(
                f"transient {v:.0f} m/s", v, r, alpha,
                "yaw reversal at the tyre limit — the error peaks with "
                "steering input and therefore imitates real vehicle response",
                derived=True))
        return tuple(out)


#: Fallback only. Representative FSAE numbers, used when no VehicleSpec is
#: supplied. Kept because a lever-arm figure from plausible numbers is more
#: useful than no figure at all — but any finding computed from these is
#: labelled as such, and `VehicleSpec.manoeuvres()` should replace them as soon
#: as the team's own mass, wheelbase, CG and grip are declared.
MANOEUVRES: tuple[Manoeuvre, ...] = (
    Manoeuvre("skidpad", speed_ms=11.2, radius_m=9.125, yaw_accel_rads2=0.0,
              why="steady-state cornering — the error is a constant offset "
                  "that reads as a miscalibrated sensor"),
    Manoeuvre("slalom", speed_ms=15.0, radius_m=12.0, yaw_accel_rads2=8.0,
              why="rapid yaw reversal — the error peaks with steering input "
                  "and therefore imitates real vehicle response"),
    Manoeuvre("autocross corner", speed_ms=18.0, radius_m=20.0,
              yaw_accel_rads2=4.0,
              why="a typical timed-event corner entry"),
)


def mounting_findings(spec: GnssSpec, *,
                      tolerance_g: float = 0.05,
                      vehicle: Optional[VehicleSpec] = None,
                      manoeuvres: Optional[tuple[Manoeuvre, ...]] = None
                      ) -> list[Finding]:
    """Turn "as close to the CG as possible" into a pass or a fail.

    `tolerance_g` is the spurious acceleration you are willing to accept
    uncorrected. 0.05 g is a defensible default for a car that pulls well over
    1 g: it is roughly the point where the error stops being lost in tyre noise
    and starts moving a fitted cornering stiffness.

    Pass a `VehicleSpec` and the yaw rates and yaw accelerations are derived
    from the team's own mass, wheelbase, CG position and measured grip. Without
    one, generic FSAE numbers are used and every resulting finding says so —
    because the lever-arm result is only ever as good as the yaw acceleration
    behind it, and an unlabelled generic number is the kind of thing that ends
    up quoted in a design review as if it were measured.
    """
    out: list[Finding] = []

    derived = None
    if manoeuvres is None and vehicle is not None:
        derived = vehicle.manoeuvres()
        if derived is None:
            out.append(Finding(
                "gnss-vehicle-underdeclared", Severity.MISSING,
                f"Vehicle parameters are incomplete "
                f"({', '.join(vehicle.missing_fields())}), so the yaw "
                f"behaviour cannot be derived and generic FSAE numbers are "
                f"used instead. Mass, wheelbase, CG position and measured "
                f"lateral grip are four numbers the team already owns; "
                f"supplying them replaces the only remaining guess in this "
                f"calculation.",
                subsystems=["suspension", "chassis", "dataacq"]))
    use = manoeuvres or derived or MANOEUVRES
    is_derived = bool(use and use[0].derived)

    if is_derived and vehicle is not None:
        alpha = vehicle.max_yaw_accel_rads2()
        est = vehicle.yaw_inertia_kgm2 is None
        out.append(Finding(
            "gnss-yaw-envelope", Severity.INFO,
            f"Peak yaw acceleration derived from the declared car: "
            f"{alpha:.1f} rad/s^2, from one axle saturated at mu = "
            f"{vehicle.mu_lat:g} over a {vehicle.wheelbase_m:g} m wheelbase"
            + (f", with yaw inertia estimated as m*a*b = "
               f"{vehicle.yaw_inertia():.0f} kg m^2 (dynamic index 1). A "
               f"bifilar swing test replaces that estimate."
               if est else
               f", with the declared yaw inertia "
               f"{vehicle.yaw_inertia_kgm2:g} kg m^2."),
            subsystems=["suspension", "dataacq"],
            detail={"yaw_accel_rads2": alpha,
                    "yaw_inertia_kgm2": vehicle.yaw_inertia(),
                    "inertia_estimated": est}))

    if spec.offset_from_cg_m is None:
        out.append(Finding(
            "gnss-offset-unmeasured", Severity.MISSING,
            "The mounting offset from the centre of gravity has not been "
            "measured. 'As close to the CG as possible' is a preference, not a "
            "specification, and without the actual vector the rotational terms "
            "cannot be subtracted in post — which means whatever offset exists "
            "stays in the data permanently. Measuring it is a tape measure and "
            "five minutes; recovering from not having measured it is not "
            "possible after the session.",
            subsystems=["chassis", "dataacq", "suspension"]))
        return out

    r = spec.lever_arm_m()
    worst: Optional[tuple[Manoeuvre, LeverArmResult]] = None
    for m in use:
        res = lever_arm_error(spec.offset_from_cg_m, m.yaw_rate_rads(),
                              m.yaw_accel_rads2)
        if worst is None or res.magnitude_g() > worst[1].magnitude_g():
            worst = (m, res)

    m, res = worst
    mag = res.magnitude_g()
    gx, gy = res.a_err_g()
    basis = ("derived from the declared vehicle"
             if is_derived else
             "from GENERIC FSAE manoeuvre numbers, not this car")

    detail = {"lever_arm_m": r, "worst_manoeuvre": m.name,
              "err_g": mag, "err_x_g": gx, "err_y_g": gy,
              "derived": is_derived}

    if mag > tolerance_g:
        out.append(Finding(
            "gnss-lever-arm", Severity.WARN,
            f"A {r*1000:.0f} mm offset from the CG produces up to "
            f"{mag:.3f} g of acceleration the car does not experience "
            f"({gx:+.3f} g longitudinal, {gy:+.3f} g lateral) in the "
            f"{m.name} case ({basis}) — {m.why}. That is above the "
            f"{tolerance_g:.2f} g you said you would accept. Two ways out, and "
            f"the second is usually the real one: move the unit closer to the "
            f"CG, or record the offset vector and subtract "
            f"alpha x r + omega x (omega x r) in post using the yaw rate this "
            f"same device already reports. The correction is arithmetic; what "
            f"makes it fail is not knowing the offset.",
            subsystems=["chassis", "dataacq", "suspension", "aero"],
            detail=detail))
    else:
        out.append(Finding(
            "gnss-lever-arm-ok", Severity.OK,
            f"{r*1000:.0f} mm from the CG — worst-case spurious acceleration "
            f"{mag:.3f} g ({basis}), inside the {tolerance_g:.2f} g tolerance.",
            subsystems=["chassis", "dataacq"], detail=detail))

    # The transient term deserves its own finding even when the total passes:
    # it is the one that does not look like an error.
    if res.angular_ms2 > 0.02 * 9.80665:
        out.append(Finding(
            "gnss-lever-arm-transient", Severity.INFO,
            f"Of that, {res.angular_ms2/9.80665:.3f} g comes from yaw "
            f"acceleration rather than steady cornering. This part is phase-"
            f"locked to steering input, so on a plot it is indistinguishable "
            f"from genuine vehicle response and will not be spotted by "
            f"inspection. The steady centripetal part "
            f"({res.centripetal_ms2/9.80665:.3f} g) is the easy one — it looks "
            f"like a zero offset and someone will eventually 'fix' it by "
            f"re-zeroing, which removes the evidence without removing the "
            f"error.",
            subsystems=["dataacq", "suspension"]))
    return out


# ===================================================================== #
#  3.  LATENCY — what a stale velocity is worth at speed
# ===================================================================== #
def latency_findings(spec: GnssSpec, *,
                     reference_speed_ms: float = 20.0) -> list[Finding]:
    """Velocity latency, expressed as the distance the car covered meanwhile."""
    out: list[Finding] = []
    if spec.velocity_latency_s is None:
        out.append(Finding(
            "gnss-latency-unknown", Severity.MISSING,
            "Velocity latency is not declared. GNSS velocity is a filtered "
            "quantity and lags reality by substantially more than the IMU "
            "does, so fusing the two without knowing the offset time-shifts "
            "one against the other. At competition speeds a tenth of a second "
            "is metres of track, and a lap-position trace built from a lagged "
            "velocity puts every event in the wrong corner.",
            subsystems=["dataacq"]))
        return out

    d = spec.velocity_latency_s * reference_speed_ms
    sev = Severity.WARN if d > 1.0 else Severity.INFO
    out.append(Finding(
        "gnss-latency", sev,
        f"{spec.velocity_latency_s*1000:.0f} ms of velocity latency is "
        f"{d:.2f} m of track at {reference_speed_ms:.0f} m/s. Correct for it "
        f"by time-shifting on a known event — a hard braking onset is visible "
        f"in both the IMU and the GNSS velocity, and the offset between them "
        f"is the number you need.",
        subsystems=["dataacq"],
        detail={"latency_s": spec.velocity_latency_s, "distance_m": d}))
    return out


def rate_findings(spec: GnssSpec, *,
                  vehicle_bandwidth_hz: float = 15.0) -> list[Finding]:
    """Update rates against the bandwidth of what is being measured."""
    out: list[Finding] = []

    if spec.imu_rate_hz is None:
        out.append(Finding(
            "gnss-imu-rate-unknown", Severity.MISSING,
            "IMU update rate not declared.", subsystems=["dataacq"]))
    else:
        ratio = spec.imu_rate_hz / vehicle_bandwidth_hz
        if ratio < 2.0:
            out.append(Finding(
                "gnss-imu-aliasing", Severity.FAIL,
                f"{spec.imu_rate_hz:g} Hz IMU against {vehicle_bandwidth_hz:g} "
                f"Hz of chassis motion is below Nyquist. The aliased content "
                f"is not noise — it is real motion folded to a frequency it "
                f"never had, and nothing downstream can undo it.",
                subsystems=["dataacq", "suspension"]))
        elif ratio < NYQUIST_PRACTICAL:
            out.append(Finding(
                "gnss-imu-rate-marginal", Severity.WARN,
                f"{spec.imu_rate_hz:g} Hz IMU is only {ratio:.1f}x the "
                f"{vehicle_bandwidth_hz:g} Hz of interest. Above Nyquist but "
                f"below the ~5x where peaks stop being clipped by sampling "
                f"phase.",
                subsystems=["dataacq"]))
        else:
            out.append(Finding(
                "gnss-imu-rate-ok", Severity.OK,
                f"{spec.imu_rate_hz:g} Hz IMU is {ratio:.1f}x the "
                f"{vehicle_bandwidth_hz:g} Hz chassis bandwidth.",
                subsystems=["dataacq"], detail={"ratio": ratio}))

    if spec.position_rate_hz is not None and spec.imu_rate_hz is not None:
        out.append(Finding(
            "gnss-rate-split", Severity.INFO,
            f"Position updates at {spec.position_rate_hz:g} Hz and inertial "
            f"data at {spec.imu_rate_hz:g} Hz — a {spec.imu_rate_hz/spec.position_rate_hz:.0f}:1 "
            f"ratio. These must stay in separate CAN frames on separate "
            f"identifiers. Sharing a frame forces the position data to the IMU "
            f"rate, which does not make it fresher; it repeats stale values at "
            f"{spec.imu_rate_hz:g} Hz, which looks like fast data and is not.",
            subsystems=["dataacq"]))
    return out


# ===================================================================== #
#  4.  BUS AND POWER COMPATIBILITY
# ===================================================================== #
def bus_findings(spec: GnssSpec, bus: BusSpec) -> list[Finding]:
    """The device's bitrate against the bus it is being asked to join."""
    out: list[Finding] = []

    if spec.can_bitrate_bps is None:
        out.append(Finding(
            "gnss-bitrate-unknown", Severity.MISSING,
            "Device CAN bitrate not declared.", subsystems=["dataacq"]))
    elif abs(spec.can_bitrate_bps - bus.bitrate_bps) > 1.0:
        out.append(Finding(
            "gnss-bitrate-mismatch", Severity.FAIL,
            f"The device transmits at {spec.can_bitrate_bps/1000:.0f} kbps; "
            f"{bus.name} runs at {bus.bitrate_bps/1000:.0f} kbps. These do not "
            f"partially work. A node at the wrong bitrate cannot win "
            f"arbitration or acknowledge a frame, so it error-frames "
            f"continuously and degrades the bus for every other node — the "
            f"symptom is the rest of the car misbehaving, which sends people "
            f"looking anywhere but here. Three ways out: reconfigure the "
            f"device if it supports it, run the vehicle bus at "
            f"{spec.can_bitrate_bps/1000:.0f} kbps, or give it its own bus and "
            f"gateway the frames across on the logger.",
            subsystems=["dataacq", "electrics", "chassis"],
            detail={"device_bps": spec.can_bitrate_bps,
                    "bus_bps": bus.bitrate_bps}))
    else:
        out.append(Finding(
            "gnss-bitrate-ok", Severity.OK,
            f"Device and {bus.name} both at "
            f"{bus.bitrate_bps/1000:.0f} kbps.",
            subsystems=["dataacq"]))

    if spec.can_extended_ids and not bus.extended_ids:
        out.append(Finding(
            "gnss-id-format", Severity.WARN,
            f"The device uses CAN 2.0B 29-bit identifiers; the bus is "
            f"declared 11-bit. Mixed formats coexist electrically, but an "
            f"extended frame costs "
            f"{can_frame_bits(8, extended=True) - can_frame_bits(8)} more bits "
            f"than a standard one — {can_frame_bits(8, extended=True)} against "
            f"{can_frame_bits(8)} for eight data bytes — so the bus budget has "
            f"to be recomputed with the right frame length, and every 29-bit "
            f"frame loses arbitration to every 11-bit frame regardless of what "
            f"the numbers mean.",
            subsystems=["dataacq", "electrics"]))
    return out


def power_findings(spec: GnssSpec, rails: dict) -> list[Finding]:
    """Operating voltage window against the rails that actually exist."""
    out: list[Finding] = []
    if spec.supply_v_min is None or spec.supply_v_max is None:
        out.append(Finding(
            "gnss-supply-unknown", Severity.MISSING,
            "Operating voltage window not declared.",
            subsystems=["electrics"]))
        return out

    usable = [r for r in rails.values()
              if spec.supply_v_min <= r.voltage_v <= spec.supply_v_max]
    if not usable:
        out.append(Finding(
            "gnss-no-compatible-rail", Severity.FAIL,
            f"The device needs {spec.supply_v_min:g}-{spec.supply_v_max:g} V "
            f"and no declared rail sits in that window "
            f"({', '.join(f'{r.name} {r.voltage_v:g}V' for r in rails.values())}). "
            f"A 5 V sensor rail cannot power it; this needs the 12 V side or a "
            f"dedicated regulator.",
            subsystems=["electrics"]))
    else:
        out.append(Finding(
            "gnss-rail-ok", Severity.OK,
            f"{spec.supply_v_min:g}-{spec.supply_v_max:g} V window is met by "
            f"{', '.join(r.name for r in usable)}. The wide window means "
            f"unregulated GLV is acceptable, so this does not need its own "
            f"regulator and does not brown out on a crank.",
            subsystems=["electrics"]))
    if spec.current_ma is None:
        out.append(Finding(
            "gnss-current-unknown", Severity.MISSING,
            "Supply current not declared — the rail budget cannot include it.",
            subsystems=["electrics", "dataacq"]))
    return out


def documentation_findings(spec: GnssSpec) -> list[Finding]:
    """Every unanswered measurement question, named individually."""
    out: list[Finding] = []
    open_q = spec.unanswered_measurements()
    if open_q:
        out.append(Finding(
            "gnss-measurements-unanswered", Severity.MISSING,
            f"{len(open_q)} of {len(spec.MEASUREMENT_QUESTIONS)} measurement "
            f"questions are unanswered: {', '.join(open_q)}. These are the "
            f"lines that decide what the data can be used for. Position "
            f"accuracy decides whether a racing line is meaningful or whether "
            f"consecutive laps just look different; heading resolution decides "
            f"whether a slip-angle estimate is possible at all. Until they are "
            f"filled in from the manual, this node is a purchase and not yet a "
            f"channel.",
            subsystems=["dataacq"],
            detail={"unanswered": open_q}))
    if spec.external_antenna is None:
        out.append(Finding(
            "gnss-antenna-undecided", Severity.MISSING,
            "Whether an external antenna is required is still an open "
            "question. It is not a small one: an external antenna adds a "
            "coaxial run, a ground-plane requirement and a mounting point that "
            "has to see the sky, which lands on chassis and on aero rather "
            "than on electrics.",
            subsystems=["chassis", "aero", "electrics", "dataacq"]))
    if spec.calibration is None:
        out.append(Finding(
            "gnss-calibration-undeclared", Severity.MISSING,
            "No calibration procedure declared.", subsystems=["dataacq"]))
    return out


# ===================================================================== #
#  5.  CAN FRAME MAP
# ===================================================================== #
def can_map(spec: GnssSpec, *, base_id: int = 0x600) -> list[CanMessage]:
    """Frames the node puts on the bus, grouped by rate.

    Position and inertial data are split because their rates differ by 4x, for
    exactly the reason `daq_plan.pack_signals` groups by rate: a shared frame
    forces one of them to the other's period.
    """
    ext = bool(spec.can_extended_ids)
    msgs: list[CanMessage] = []
    prate = spec.position_rate_hz or 0.0
    irate = spec.imu_rate_hz or 0.0

    if irate:
        msgs.append(CanMessage("GNSS_ACCEL", base_id, 6, irate, ext,
                               "gnss_node", ["accel_x", "accel_y", "accel_z"]))
        msgs.append(CanMessage("GNSS_GYRO", base_id + 1, 6, irate, ext,
                               "gnss_node", ["gyro_x", "gyro_y", "gyro_z"]))
    if prate:
        msgs.append(CanMessage("GNSS_POSITION", base_id + 2, 8, prate, ext,
                               "gnss_node", ["latitude", "longitude"]))
        msgs.append(CanMessage("GNSS_VELOCITY", base_id + 3, 8, prate, ext,
                               "gnss_node",
                               ["speed", "heading", "height", "fix_quality"]))
    return msgs


def to_sensor_specs(spec: GnssSpec) -> list[SensorSpec]:
    """The node as daq_plan channels, so it enters the bus and storage budget.

    Location is "chassis" because that is where it bolts, which routes it to
    the subteams the spec sheet names — chassis, aero and electrics — through
    `LOCATION_SUBTEAMS` rather than through somebody remembering to tell them.
    """
    out: list[SensorSpec] = []
    common = dict(
        location="gnss_node", output=OutputType.CAN,
        supply_v=(spec.supply_v_min if spec.supply_v_min is None
                  else 12.0),
        current_ma=spec.current_ma, supply_rail="12V",
        connector=spec.connector, conductors=4,
        logged_to="logger", calibration=spec.calibration,
        available_on_existing_bus="gnss_node",
        source=spec.source, owner=spec.owner)

    if spec.imu_rate_hz:
        out.append(SensorSpec(
            key="imu_accel", name="Chassis acceleration (IMU)",
            measures="3-axis acceleration at the sensor", unit="g",
            why="the measured side of every load case the suspension and aero "
                "models currently assert",
            signal_bandwidth_hz=15.0, sample_rate_hz=spec.imu_rate_hz,
            adc_bits=16, range_min_eu=-4.0, range_max_eu=4.0,
            accuracy_eu=None, resolution_needed_eu=0.01,
            payload_bytes=6, **common))
        out.append(SensorSpec(
            key="imu_gyro", name="Chassis angular rate (IMU)",
            measures="3-axis angular rate", unit="deg/s",
            why="yaw rate closes the loop on the handling model, and it is "
                "also what makes the lever-arm correction computable",
            signal_bandwidth_hz=15.0, sample_rate_hz=spec.imu_rate_hz,
            adc_bits=16, range_min_eu=-300.0, range_max_eu=300.0,
            accuracy_eu=None, resolution_needed_eu=0.1,
            payload_bytes=6, **common))
    if spec.position_rate_hz:
        out.append(SensorSpec(
            key="gnss_position", name="GNSS position",
            measures="latitude/longitude", unit="deg",
            why="lap segmentation and corner-by-corner comparison; without it "
                "every lap time is a single number with no structure",
            signal_bandwidth_hz=2.0, sample_rate_hz=spec.position_rate_hz,
            adc_bits=32,
            range_min_eu=-180.0, range_max_eu=180.0,
            accuracy_eu=spec.position_accuracy_m,
            payload_bytes=8, **common))
        out.append(SensorSpec(
            key="gnss_velocity", name="GNSS speed and heading",
            measures="ground speed and track heading", unit="m/s",
            why="an absolute speed reference that does not depend on wheel "
                "speed, so it stays true under lock-up and wheelspin — which "
                "is exactly when wheel speed lies",
            signal_bandwidth_hz=2.0, sample_rate_hz=spec.position_rate_hz,
            adc_bits=16, range_min_eu=0.0,
            range_max_eu=spec.velocity_max_ms or 40.0,
            accuracy_eu=spec.velocity_accuracy_ms,
            resolution_needed_eu=spec.velocity_resolution_ms,
            payload_bytes=8, **common))
    return out


# ===================================================================== #
#  6.  COTS vs DIY
# ===================================================================== #
@dataclass
class OptionComparison:
    """Two ways to get the same channels, compared on answers not on price."""
    cots: GnssSpec
    diy: GnssSpec
    findings: list = field(default_factory=list)


def compare_options(cots: GnssSpec = ECUMASTER_GPS_TO_CAN_V2,
                    diy: Optional[GnssSpec] = None) -> OptionComparison:
    """Compare a bought node against a built one on the checklist, not on cost.

    The DIY option is not scored badly here for being DIY. It is scored on how
    many of the review questions it currently has answers for, which for a
    board that has not been built yet is close to none — and that is the real
    difference between the two, not the price. A bought unit's central value is
    that somebody else already answered the questions and wrote the numbers
    down.
    """
    diy = diy or GnssSpec(
        key="diy_gnss", name="DIY: Teensy 4.1 + GNSS module",
        cost_usd=None,
        notes="Cost is not the deciding variable and is left blank on "
              "purpose.")
    findings: list[Finding] = []

    c_ans = cots.measurement_completeness()
    d_ans = diy.measurement_completeness()
    findings.append(Finding(
        "gnss-option-completeness", Severity.INFO,
        f"Measurement questions answered: {cots.name} "
        f"{c_ans*100:.0f}%, {diy.name} {d_ans*100:.0f}%. Both start low, and "
        f"that is the honest state — but the bought unit's answers exist in a "
        f"manual and can be transcribed this afternoon, whereas the DIY "
        f"figures do not exist anywhere until the board is built and bench "
        f"tested. Those two kinds of blank are not the same size.",
        subsystems=["dataacq"],
        detail={"cots": c_ans, "diy": d_ans}))

    findings.append(Finding(
        "gnss-option-tradeoff", Severity.INFO,
        "The trade is calibration effort against integration effort. The COTS "
        "unit arrives with a fixed bitrate and a fixed frame layout you must "
        "accommodate, and it is the reason the bitrate conflict above exists "
        "at all. The DIY node lets you choose the bitrate, the identifiers and "
        "the update rates to fit the bus you already have, and in exchange you "
        "own the GNSS driver, the IMU fusion, the enclosure, the connector "
        "termination and every accuracy figure — none of which are hard "
        "individually, all of which are a term's work together. If the team "
        "already runs Teensy 4.1 with FlexCAN_T4 elsewhere, the firmware side "
        "is genuinely cheaper than it looks; the sensor characterisation is "
        "not.",
        subsystems=["dataacq", "electrics", "chassis"]))

    if cots.cost_usd:
        findings.append(Finding(
            "gnss-option-cost", Severity.INFO,
            f"{cots.name} is ${cots.cost_usd:.0f} before the antenna and the "
            f"mating connector ({cots.mating_connector or 'part unlisted'}). "
            f"Deutsch DT terminations also need the right crimp tool, which is "
            f"a real line item the first time.",
            subsystems=["dataacq"]))
    return OptionComparison(cots, diy, findings)


# ===================================================================== #
#  7.  ONE CALL
# ===================================================================== #
@dataclass
class GnssPlan:
    spec: GnssSpec
    messages: list
    sensors: list
    findings: list = field(default_factory=list)

    def blocking(self) -> list:
        return [f for f in self.findings if f.severity == Severity.FAIL]

    def open_questions(self) -> list:
        return [f for f in self.findings if f.severity == Severity.MISSING]

    def bus_bits_per_second(self) -> float:
        return sum(m.bits() * m.rate_hz for m in self.messages)

    def to_markdown(self) -> str:
        L = [f"# GNSS / IMU node — {self.spec.name}", ""]
        L.append(f"- Position rate: **{self.spec.position_rate_hz} Hz**")
        L.append(f"- IMU rate: **{self.spec.imu_rate_hz} Hz**")
        L.append(f"- Device bitrate: **{(self.spec.can_bitrate_bps or 0)/1000:.0f} kbps**")
        L.append(f"- Frames: **{len(self.messages)}**, "
                 f"**{self.bus_bits_per_second()/1000:.1f} kbit/s** of bus")
        r = self.spec.lever_arm_m()
        L.append(f"- Offset from CG: "
                 + (f"**{r*1000:.0f} mm**" if r is not None else "**not measured**"))
        L.append("")
        L.append("| Severity | Check | Message |")
        L.append("|---|---|---|")
        order = {Severity.FAIL: 0, Severity.MISSING: 1, Severity.WARN: 2,
                 Severity.INFO: 3, Severity.OK: 4}
        for f in sorted(self.findings, key=lambda f: order[f.severity]):
            msg = f.message.replace("|", "/").replace("\n", " ")
            L.append(f"| {f.severity.value.upper()} | {f.check} | {msg} |")
        return "\n".join(L)


def plan_gnss(spec: GnssSpec = ECUMASTER_GPS_TO_CAN_V2, *,
              bus: Optional[BusSpec] = None,
              rails: Optional[dict] = None,
              vehicle: Optional[VehicleSpec] = None,
              base_id: int = 0x600,
              tolerance_g: float = 0.05,
              vehicle_bandwidth_hz: float = 15.0,
              reference_speed_ms: float = 20.0) -> GnssPlan:
    """Every check, one call."""
    bus = bus or BusSpec()
    rails = rails or {"5V": Rail("5V", 5.0, capacity_ma=500.0, fuse_a=1.0),
                      "12V": Rail("12V", 12.0, capacity_ma=2000.0, fuse_a=5.0)}
    findings: list[Finding] = []
    findings.extend(documentation_findings(spec))
    findings.extend(mounting_findings(spec, tolerance_g=tolerance_g,
                                      vehicle=vehicle))
    findings.extend(latency_findings(spec,
                                     reference_speed_ms=reference_speed_ms))
    findings.extend(rate_findings(spec,
                                  vehicle_bandwidth_hz=vehicle_bandwidth_hz))
    findings.extend(bus_findings(spec, bus))
    findings.extend(power_findings(spec, rails))
    return GnssPlan(spec, can_map(spec, base_id=base_id),
                    to_sensor_specs(spec), findings)


# ===================================================================== #
#  8.  PROVENANCE
# ===================================================================== #
PROVENANCE = {
    "physics_grounded": [
        "lever-arm error from planar rigid-body kinematics, "
        "a_meas = a_cg + alpha x r + omega x (omega x r), yaw axis only",
        "latency expressed as distance travelled, v * t",
        "Nyquist applied to the IMU rate against declared chassis bandwidth",
        "CAN frame length via daq_plan.can_frame_bits (ISO 11898-1 layout), so "
        "the 29-bit identifier cost is computed rather than asserted",
    ],
    "from_documentation": [
        "position 25 Hz, IMU 100 Hz, CAN 1 Mbps, CAN 2.0B",
        "operating voltage 6-22 V DC",
        "Deutsch DT06-4S / DT15-4P, mating part F02U.B00.656-01",
        "calibration by USB, zeroed stationary on a flat surface",
        "mounting requirement: as close to the centre of gravity as possible",
    ],
    "derived_from_declared_vehicle": [
        "yaw rate from the grip limit, omega = v / R with R = v^2 / (mu g)",
        "peak yaw acceleration from one axle saturated laterally, "
        "M_z = mu m g (b/L) a, divided by yaw inertia — which reduces to "
        "alpha_max = mu g / L under the dynamic-index-one inertia estimate",
        "VehicleSpec.manoeuvres() replaces the generic MANOEUVRES entirely "
        "once mass, wheelbase, CG position and measured grip are declared, "
        "and every finding states which basis it used",
    ],
    "estimate_flagged": [
        "yaw inertia defaults to the dynamic-index-one approximation "
        "I_zz ~= m a b; a bifilar swing test replaces it and the finding says "
        "so explicitly whenever the estimate was used",
        "MANOEUVRES is a FALLBACK only, used when no VehicleSpec is supplied. "
        "Findings computed from it are labelled 'from GENERIC FSAE manoeuvre "
        "numbers, not this car' in their own message text, so the label "
        "travels with the number into any document it is pasted into",
        "default tolerance_g = 0.05 is a judgement about what is worth "
        "correcting, not a physical threshold",
        "vehicle_bandwidth_hz = 15 Hz is a generic chassis figure",
    ],
    "hard_rule": (
        "Every measurement question stays Optional and every blank is reported "
        "individually. A spec sheet that lists 'velocity accuracy' as a "
        "heading has not stated a velocity accuracy, and the difference "
        "between those two is the difference between a channel and a purchase."
    ),
}
