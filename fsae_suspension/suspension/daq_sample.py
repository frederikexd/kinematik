# ============================================================================
#  KinematiK — suspension/daq_sample.py
#  A worked example channel plan for the Data Acquisition lead.
#
#  This is a DEMO dataset, not a recommendation. It is built to make every
#  check in daq_plan.py fire at least once, so a new lead can see what the tool
#  actually does in one pass instead of discovering the aliasing check three
#  months later when the damper trace looks wrong.
#
#  Several channels below are DELIBERATELY BROKEN. Each one is marked with a
#  `# DEMO:` comment naming the fault it is there to demonstrate. Do not copy
#  this file into a real plan — start from CATALOG entries and put your own
#  numbers in.
# ============================================================================
"""A 14-channel FSAE EV plan with a fault planted for every check.

Run it headless::

    python3 -m suspension.daq_sample          # prints the full analysis

Or load it in the app: Data Acquisition -> Channels -> "Load the sample plan".

What each planted fault demonstrates
------------------------------------
=====================  ====================================================
channel                what it is there to trip
=====================  ====================================================
damper_pot_fl          ALIASING — 30 Hz of real motion sampled at 50 Hz
                       with the anti-alias filter left blank. The one
                       acquisition error you cannot fix in post.
coolant_temp_in        OVERSAMPLING — 0.5 Hz signal at 200 Hz, burning bus
                       and card for nothing.
coolant_temp_out       RESOLUTION — paired with the inlet, but +/-1.0 K
                       accuracy against an expected 4 K rise. The delta-T
                       page shows why this pair cannot measure the quantity
                       it was bought to measure.
brake_pressure_f       UNANSWERED — connector, conductors and calibration
                       left None, so the plan cannot report READY.
ts_current             ISOLATION — sits on the tractive-system side with
                       galvanic_isolation=False.
inverter_temp          NOT APPLICABLE — already broadcast by the inverter,
                       so its power and connector questions are waived
                       rather than counted as debt.
wheel_speed_*          BUS LOAD — four 200 Hz channels, which is where a
                       500 kbit/s bus actually starts to hurt.
gps_position           STORAGE + a wide payload on a slow channel.
=====================  ====================================================
"""

from __future__ import annotations

from .daq_plan import (
    BmsSignal, BusSpec, LoggerSpec, OutputType, Rail, SensorSpec, UartLink,
    catalog, default_rails, plan, plan_bms_bridge,
)


# --------------------------------------------------------------------------- #
#  Bus, rails, logger
# --------------------------------------------------------------------------- #
def sample_bus() -> BusSpec:
    """A 500 kbit/s GLV bus — the common FSAE choice, and slower than teams
    assume once four wheel-speed channels are on it."""
    return BusSpec(name="GLV CAN", bitrate_bps=500_000.0, extended_ids=False)


def sample_rails() -> dict:
    """Stock 5 V / 12 V rails. The 5 V rail is deliberately close to its
    500 mA budget once the sample channels are added — see the Power section
    of the report."""
    return default_rails()


def sample_logger() -> LoggerSpec:
    return LoggerSpec(name="SD logger", storage_mb=8192.0,
                      session_minutes=25.0)


# --------------------------------------------------------------------------- #
#  Channels
# --------------------------------------------------------------------------- #
def sample_sensors() -> list[SensorSpec]:
    """14 channels: 5 answered properly, 9 with a planted fault each."""
    s: list[SensorSpec] = []

    # --- cooling loop: starts from the catalog, then breaks two things ----- #
    s.append(catalog(
        "motor_temp", owner="R. Okafor", source="TE NTC 44006 datasheet",
        connector="AMP Superseal 2-way", conductors=2,
        calibration="ice bath + boiling water, 2-point, before each event",
        adc_bits=12, sample_rate_hz=10.0, antialias_cutoff_hz=2.0,
        payload_bytes=2, logged_to="logger"))

    # DEMO: already on the bus. Its power/connector questions get WAIVED —
    # the inverter is wired whether or not anyone logs this.
    s.append(catalog(
        "inverter_temp", owner="R. Okafor",
        available_on_existing_bus="Cascadia PM100DX (0x0A2, 100 Hz)",
        sample_rate_hz=10.0, payload_bytes=2, logged_to="logger",
        source="PM100DX CAN protocol rev 5"))

    # DEMO: OVERSAMPLING. Coolant temperature moves at ~0.5 Hz. 200 Hz is
    # 400x Nyquist — bus and card spent on nothing.
    s.append(catalog(
        "coolant_temp_in", owner="P. Lindqvist", source="PT1000 class A",
        connector="Deutsch DTM 2-way", conductors=3,
        calibration="2-point against a reference RTD at 20 C and 60 C",
        signal_bandwidth_hz=0.5, sample_rate_hz=200.0,
        antialias_cutoff_hz=5.0, adc_bits=12,
        accuracy_eu=1.0,                    # DEMO: see delta-T below
        payload_bytes=2, logged_to="logger"))

    # DEMO: RESOLUTION. Same +/-1.0 K on the outlet. Two of these cannot
    # resolve a 4 K rise — the delta-T page does that error propagation.
    s.append(catalog(
        "coolant_temp_out", owner="P. Lindqvist", source="PT1000 class A",
        connector="Deutsch DTM 2-way", conductors=3,
        calibration="2-point against a reference RTD at 20 C and 60 C",
        signal_bandwidth_hz=0.5, sample_rate_hz=5.0,
        antialias_cutoff_hz=2.0, adc_bits=12, accuracy_eu=1.0,
        payload_bytes=2, logged_to="logger"))

    s.append(catalog(
        "coolant_flow", owner="P. Lindqvist", source="Gems FT-110",
        connector="Deutsch DTM 3-way", conductors=3,
        calibration="gravimetric, 3 flow points, bucket and scale",
        sample_rate_hz=20.0, antialias_cutoff_hz=5.0, adc_bits=12,
        payload_bytes=2, logged_to="logger"))

    # --- suspension ------------------------------------------------------- #
    # DEMO: ALIASING. Damper velocity carries content to ~30 Hz. Sampling at
    # 50 Hz is under the practical floor, and antialias_cutoff_hz is None, so
    # there is no filter in front of it either. This is the finding to look at
    # first: it is the only error in the list that cannot be undone later.
    s.append(SensorSpec(
        key="damper_pot_fl", name="Damper position, front left",
        measures="damper shaft displacement", unit="mm",
        why="damper velocity histogram; sets the low-speed knee we actually "
            "run at Michigan",
        location="damper", output=OutputType.ANALOG_V,
        supply_v=5.0, current_ma=12.0, supply_rail="5V",
        connector="Deutsch DTM 3-way", conductors=3,
        signal_bandwidth_hz=30.0,
        sample_rate_hz=50.0,                # DEMO: below the practical floor
        antialias_cutoff_hz=None,           # DEMO: no filter declared
        adc_bits=12, range_min_eu=0.0, range_max_eu=75.0,
        accuracy_eu=0.2, resolution_needed_eu=0.1,
        logged_to="logger", payload_bytes=2,
        calibration="dial gauge against the pot at 10 mm steps, both "
                    "directions, check hysteresis",
        galvanic_isolation=True, owner="J. Doe",
        source="Penny+Giles SLS095/75"))

    s.append(SensorSpec(
        key="steer_angle", name="Steering angle",
        measures="steering wheel angle", unit="deg",
        why="separates driver input from yaw response when a corner goes "
            "wrong on the data",
        location="steering", output=OutputType.ANALOG_V,
        supply_v=5.0, current_ma=15.0, supply_rail="5V",
        connector="Deutsch DTM 3-way", conductors=3,
        signal_bandwidth_hz=8.0, sample_rate_hz=100.0,
        antialias_cutoff_hz=20.0, adc_bits=12,
        range_min_eu=-120.0, range_max_eu=120.0,
        accuracy_eu=0.5, resolution_needed_eu=0.25,
        logged_to="logger", payload_bytes=2,
        calibration="centre with the rack at mid-travel, then lock-to-lock "
                    "against a protractor on the hub",
        galvanic_isolation=True, owner="J. Doe",
        source="Bourns 6639S-1-103"))

    # DEMO: four 200 Hz channels. This is what actually loads the bus.
    for corner, own in (("fl", "J. Doe"), ("fr", "J. Doe"),
                        ("rl", "JJ. Doe"), ("rr", "JJ. Doe")):
        s.append(SensorSpec(
            key=f"wheel_speed_{corner}", name=f"Wheel speed {corner.upper()}",
            measures="wheel rotational speed", unit="rad/s",
            why="slip ratio for the traction controller, and the only honest "
                "check that a corner locked under braking",
            location="wheel", output=OutputType.PULSE,
            supply_v=12.0, current_ma=20.0, supply_rail="12V",
            connector="Deutsch DTM 3-way", conductors=3,
            signal_bandwidth_hz=80.0, sample_rate_hz=200.0,
            antialias_cutoff_hz=90.0, adc_bits=12,
            range_min_eu=0.0, range_max_eu=250.0,
            accuracy_eu=0.5, resolution_needed_eu=0.25,
            logged_to="logger", payload_bytes=2,
            calibration="rolling road at three known speeds; confirm the "
                        "tooth count in firmware matches the ring",
            galvanic_isolation=True, owner=own,
            source="Honeywell VG481V1"))

    # --- brakes: DEMO: UNANSWERED QUESTIONS ------------------------------- #
    # Connector, conductors and calibration left None on purpose. The plan
    # will refuse READY while these are blank, and will report bus/power as
    # FLOORS rather than answers.
    s.append(SensorSpec(
        key="brake_pressure_f", name="Brake pressure, front circuit",
        measures="hydraulic line pressure", unit="bar",
        why="proves brake balance and gives the rules a trace showing all "
            "four wheels locked",
        location="brake_line", output=OutputType.ANALOG_V,
        supply_v=5.0, current_ma=10.0, supply_rail="5V",
        connector=None,                     # DEMO: unanswered
        conductors=None,                    # DEMO: unanswered
        signal_bandwidth_hz=25.0, sample_rate_hz=200.0,
        antialias_cutoff_hz=50.0, adc_bits=12,
        range_min_eu=0.0, range_max_eu=100.0,
        accuracy_eu=1.0, resolution_needed_eu=0.5,
        logged_to="logger", payload_bytes=2,
        calibration=None,                   # DEMO: unanswered
        galvanic_isolation=True, owner="", is_estimate=True,
        source=""))

    # --- tractive system: DEMO: ISOLATION --------------------------------- #
    s.append(SensorSpec(
        key="ts_current", name="Tractive system current",
        measures="pack current", unit="A",
        why="energy used per lap, and the number the endurance strategy is "
            "actually built on",
        location="accumulator", output=OutputType.ANALOG_V,
        supply_v=5.0, current_ma=25.0, supply_rail="5V",
        connector="Deutsch DTM 4-way", conductors=4,
        signal_bandwidth_hz=100.0, sample_rate_hz=500.0,
        antialias_cutoff_hz=200.0, adc_bits=12,
        range_min_eu=-100.0, range_max_eu=300.0,
        accuracy_eu=1.0, resolution_needed_eu=0.5,
        logged_to="logger", payload_bytes=2,
        calibration="against a clamp meter at 10 A, 50 A and 150 A with the "
                    "car on jacks",
        galvanic_isolation=False,           # DEMO: on the TS side, unisolated
        owner="A. Ferreira", source="LEM HASS 200-S"))

    # --- GNSS ------------------------------------------------------------- #
    s.append(SensorSpec(
        key="gps_position", name="GNSS position",
        measures="latitude / longitude / speed over ground", unit="deg",
        why="lap segmentation, so every other channel can be compared corner "
            "by corner instead of by eye",
        location="gnss_node", output=OutputType.UART,
        supply_v=5.0, current_ma=45.0, supply_rail="5V",
        connector="JST GH 6-way", conductors=4,
        signal_bandwidth_hz=5.0, sample_rate_hz=10.0,
        antialias_cutoff_hz=5.0, adc_bits=32,
        range_min_eu=-180.0, range_max_eu=180.0,
        accuracy_eu=1.5, resolution_needed_eu=0.5,
        logged_to="logger", payload_bytes=16,
        calibration="static soak on a surveyed point for 10 min; check the "
                    "reported fix type is RTK before trusting the trace",
        galvanic_isolation=True, owner="A. Ferreira",
        source="u-blox ZED-F9P"))

    return s


# --------------------------------------------------------------------------- #
#  BMS bridge
# --------------------------------------------------------------------------- #
def sample_bridge():
    """A UART link out of the BMS, sized so the link budget has something
    to say.

    9600 baud is not a strawman: it is the shipping default on several BMS
    boards teams actually buy, and it goes unquestioned because the first two
    signals fit fine. The load arrives when someone asks for per-cell
    voltages — 96 cells cannot travel as one signal (a CAN 2.0 frame holds
    64 bits, and daq_plan refuses to pretend otherwise), so they multiplex as
    4-cell blocks. 24 blocks at 10 Hz is where the link runs out.
    """
    link = UartLink(baud=9600, data_bits=8, stop_bits=1, parity=None,
                    # The frame the BMS actually emits: 96 cell voltages at
                    # 2 bytes each, plus a 12-byte header/checksum wrapper,
                    # broadcast at 10 Hz. Declaring these is what makes the
                    # link budget computable instead of "uncheckable".
                    frame_bytes=204, frame_rate_hz=10.0)
    signals = [
        BmsSignal(name="cell_v_min", bits=16, unit="V", scale=0.001,
                  rate_hz=10.0, critical=True),
        BmsSignal(name="cell_v_max", bits=16, unit="V", scale=0.001,
                  rate_hz=10.0, critical=True),
        BmsSignal(name="cell_t_max", bits=16, unit="degC", scale=0.1,
                  rate_hz=10.0, critical=True),
        BmsSignal(name="pack_voltage", bits=16, unit="V", scale=0.1,
                  rate_hz=10.0),
        BmsSignal(name="pack_current", bits=16, unit="A", scale=0.1,
                  offset=-3200.0, rate_hz=50.0, critical=True),
        BmsSignal(name="soc", bits=8, unit="%", scale=0.5, rate_hz=1.0),
    ]
    # DEMO: 96 cells, 4 per 64-bit block, all refreshed at 10 Hz.
    signals += [
        BmsSignal(name=f"cell_v_block_{i:02d}", bits=64, unit="V",
                  scale=0.001, rate_hz=10.0)
        for i in range(1, 25)
    ]
    return plan_bms_bridge(link, signals, bus=sample_bus(), isolated=True)
    return plan_bms_bridge(link, signals, bus=sample_bus(), isolated=True)


# --------------------------------------------------------------------------- #
#  The whole thing
# --------------------------------------------------------------------------- #
def sample_plan():
    """The full analysed plan, ready to print or render."""
    return plan(
        sample_sensors(),
        bus=sample_bus(),
        rails=sample_rails(),
        logger=sample_logger(),
        bridge=sample_bridge(),
        # The cooling loop this plan is really about: a 4 K rise at 12 L/min,
        # with an unmatched sensor pair. The delta-T page turns that into an
        # uncertainty on heat rejection.
        expected_delta_t_k=4.0,
        flow_lpm=12.0,
        matched_pair=False,
    )


# --------------------------------------------------------------------------- #
#  Headless runner:  python3 -m suspension.daq_sample
# --------------------------------------------------------------------------- #
def _report() -> str:
    p = sample_plan()
    L = ["=" * 74,
         "  KinematiK — sample DAQ channel plan (DEMO DATA, faults planted)",
         "=" * 74, ""]

    L.append(f"Verdict     : {p.verdict.value.upper()}")
    L.append(f"Completeness: {p.completeness * 100:.0f}% of review questions "
             f"answered")
    L.append(f"Channels    : {len(p.sensors)}")
    L.append("")

    b = p.bus_result
    L.append(f"CAN bus     : {b.load:.0%} worst-case load, {b.messages} "
             f"messages on {b.bus.name}")
    if b.unschedulable:
        # The lesson this dataset exists to teach: load and schedulability are
        # different questions. 36% "looks fine" on any dashboard.
        L.append(f"              !! {len(b.unschedulable)} message(s) cannot "
                 f"meet their own period despite that load:")
        L.append(f"                 {', '.join(b.unschedulable)}")
    L.append("")

    for rail, d in (p.power.per_rail or {}).items():
        L.append(f"Rail {rail:<6} : {d['draw_ma']:.0f} / {d['capacity_ma']:.0f} "
                 f"mA ({d['frac']:.0%})")
    s = p.storage
    L.append(f"Storage     : {s.bytes_per_s / 1000:.1f} kB/s, "
             f"{s.session_mb:.0f} MB per session ({s.frac:.1%} of the card)")
    if p.delta_t:
        d = p.delta_t
        L.append(f"Coolant dT  : {d.delta_t_k:g} K +/- {d.sigma_delta_t_k:.2f} K "
                 f"({d.relative_error:.0%}) -> heat {d.heat_kw:.1f} +/- "
                 f"{d.sigma_heat_kw:.1f} kW")
    L.append("")

    order = {"fail": 0, "missing": 1, "warning": 2, "info": 3, "ok": 4}
    rows = sorted(p.findings, key=lambda f: order.get(f.severity.value, 9))
    counts: dict[str, int] = {}
    for f in p.findings:
        counts[f.severity.value] = counts.get(f.severity.value, 0) + 1
    L.append("Findings    : " + ", ".join(
        f"{v} {k}" for k, v in sorted(counts.items(),
                                      key=lambda kv: order.get(kv[0], 9))))
    L.append("-" * 74)
    for f in rows:
        if f.severity.value in ("ok", "info"):
            continue
        L.append(f"[{f.severity.value:7s}] {f.message}")
    L.append("-" * 74)
    L.append("Everything above is DEMO data with faults planted on purpose.")
    L.append("See the module docstring for which channel demonstrates what.")
    return "\n".join(L)


if __name__ == "__main__":
    print(_report())
