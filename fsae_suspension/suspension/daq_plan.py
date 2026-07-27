# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/daq_plan.py — the VEHICLE-side data-acquisition planner. Turns the
#  "what to document" checklist a data-acq meeting reads off a slide into an
#  enforced schema, and does the four pieces of arithmetic that the checklist
#  ASKS FOR but never actually performs: Nyquist, CAN bus load, rail current,
#  and log storage. Refuses to declare a channel plan ready while any answer is
#  still blank.
# ============================================================================
"""
DAQ Plan — the sensor list that checks itself.

WHY THIS MODULE EXISTS
----------------------
Every data-acquisition meeting produces the same two artefacts: a list of
sensors somebody wants, and a list of questions somebody should answer about
each one. Both live in a slide deck. Neither is arithmetic. The result is a
season-long failure mode that is completely predictable and completely
invisible until the car is on track:

  * A sensor list grows one meeting at a time. Nobody ever multiplies
    (channels x sample rate x frame size) and compares it to the bus. The bus
    is fine, fine, fine, and then at competition the frames that get dropped
    are the ones nobody was watching.
  * The documentation checklist is answered in a shared doc where a blank cell
    and a wrong answer look identical, and both look like progress.
  * "What sampling rate is needed?" gets answered by taste. Half the channels
    are oversampled by 100x and burn bus for nothing; one channel is
    undersampled and aliases, which is not a small error — it is unrecoverable
    corruption that looks like real data forever.
  * "What other subteam does this affect?" is answered by whoever is in the
    room. The upright bracket, the rail current and the isolation boundary all
    have owners who were not in the room.

So this module takes the checklist literally and makes it executable.

WHAT IS ENFORCED SCHEMA vs COMPUTED PHYSICS
-------------------------------------------
`SensorSpec` is the checklist, one field per question, every field Optional so
that UNANSWERED is a distinct state from a value — never a silent default. That
part is bookkeeping, and it is the half that stops a plan from lying about how
complete it is.

The computed half is real and testable:

  * CAN frame length uses the actual ISO 11898-1 field layout including the
    worst-case bit-stuffing bound, not a round number. Bus load is the exact
    sum, and worst-case message latency is the standard fixed-point
    response-time analysis (Tindell/Davis) for non-preemptive priority
    arbitration.
  * Nyquist is the theorem, applied per channel against the declared signal
    bandwidth, with the anti-alias filter treated as mandatory rather than
    optional -- because aliasing is the one acquisition error you cannot undo
    in post.
  * ADC resolution is span / 2^bits in engineering units, compared to the
    resolution the channel was specified to need.
  * The coolant delta-T uncertainty is standard uncorrelated error propagation,
    and it is the reason a pair of inlet/outlet temperature sensors bought on
    price is often incapable of measuring the quantity they were bought for.
  * The UART link budget is byte-framing arithmetic against the baud rate.

THE HARD RULE
-------------
A plan with unanswered checklist questions never returns READY, and its bus,
power and storage numbers are reported as FLOORS rather than answers -- because
an undeclared channel can only ever ADD load. The failure mode this exists to
kill is a green "38% bus load, all good" printed over six channels whose sample
rate nobody has picked yet. Undeclared is not zero.

Likewise, the BMS bridge REFUSES to emit a frame map when the signal list is
empty. "Convert the BMS onto CAN" is not a task you can plan before someone
reads the datasheet and writes down what the signals are; a tool that produces
a confident-looking frame layout from no signals is producing fiction.
"""

from __future__ import annotations

import csv
import io
import math
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Optional

from .interfaces import Severity, Finding


# ===================================================================== #
#  0.  VOCABULARY
# ===================================================================== #
class OutputType(str, Enum):
    """How the sensor hands its value over. Drives which checks apply.

    The checklist question is "Analog, digital, CAN, PWM, etc." -- and the
    "etc." is where the interesting failures live. A pulse-output flow meter is
    not sampled at all, it is COUNTED over a gate window, so asking for its
    "sampling rate" is the wrong question and gets a wrong answer. A sensor
    that is already a CAN broadcast needs no ADC channel, no rail and no
    connector -- it needs a DBC entry, and possibly nothing else at all.
    """
    ANALOG_V = "analog_voltage"       # ratiometric or absolute voltage into an ADC
    ANALOG_MA = "analog_current"      # 4-20 mA loop, needs a sense resistor
    RTD = "rtd"                       # PT100/PT1000, needs excitation + 3/4-wire
    THERMOCOUPLE = "thermocouple"     # needs cold-junction compensation
    DIGITAL = "digital"               # on/off line
    PULSE = "pulse"                   # frequency/counter input (flow, wheel speed)
    PWM = "pwm"                       # duty-cycle encoded
    CAN = "can"                       # already a bus message
    SENT = "sent"                     # SAE J2716
    I2C = "i2c"
    SPI = "spi"
    UART = "uart"


#: Output types whose value is measured by counting edges over a gate window
#: rather than by sampling an amplitude. Nyquist does not apply to these the
#: way it applies to an analog channel; resolution comes from the gate time.
COUNTER_TYPES = frozenset({OutputType.PULSE, OutputType.PWM})

#: Output types that go through an ADC and therefore MUST have an anti-alias
#: filter in front of them. This is not a style preference: an alias is a real
#: frequency folded to a fake one, and no amount of post-processing can tell
#: you which is which afterwards.
ANALOG_TYPES = frozenset({
    OutputType.ANALOG_V, OutputType.ANALOG_MA,
    OutputType.RTD, OutputType.THERMOCOUPLE,
})

#: Output types that arrive as a digital message and consume no ADC channel.
BUS_TYPES = frozenset({
    OutputType.CAN, OutputType.SENT, OutputType.I2C,
    OutputType.SPI, OutputType.UART,
})


#: Where a sensor physically lives -> which subteams own a piece of it. This is
#: the answer to "what other subteam does this affect?", computed instead of
#: remembered. The mounting location determines who owns the bracket; the rail
#: and the bus always drag in electrics and data-acq.
LOCATION_SUBTEAMS: dict[str, tuple[str, ...]] = {
    "motor":          ("powertrain", "electrics", "cooling"),
    "inverter":       ("powertrain", "electrics"),
    "accumulator":    ("electrics", "cooling"),
    "coolant_loop":   ("cooling", "powertrain"),
    "radiator":       ("cooling", "aero", "chassis"),
    "upright":        ("suspension", "brakes"),
    "wheel":          ("suspension", "brakes"),
    "pushrod":        ("suspension",),
    "damper":         ("suspension",),
    "brake_line":     ("brakes",),
    "brake_rotor":    ("brakes", "suspension"),
    "pedal_box":      ("brakes", "chassis"),
    "steering":       ("chassis", "suspension"),
    "chassis":        ("chassis",),
    "cockpit":        ("chassis", "electrics"),
    "aero_element":   ("aero", "chassis"),
    "ecu":            ("electrics",),
    "gearbox":        ("powertrain",),
}

#: Locations that sit on the TRACTIVE-SYSTEM side of the isolation boundary.
#: Any measurement crossing from here to the grounded low-voltage system needs
#: a galvanic barrier. This is a rules and a safety matter, not an EMC nicety:
#: a thermistor potted against a stator winding is referenced to the motor, and
#: an un-isolated wire from it to the logger ground is a fault path.
TRACTIVE_SYSTEM_LOCATIONS = frozenset({
    "motor", "inverter", "accumulator",
})

#: Every subteam is downstream of the two that always are.
ALWAYS_AFFECTED = ("dataacq", "electrics")


# ===================================================================== #
#  1.  THE CHECKLIST, AS A SCHEMA
# ===================================================================== #
#: The documentation questions, in the order a review reads them. Each maps to
#: the SensorSpec field(s) that answer it. This IS the checklist -- the module
#: derives completeness from it rather than from a hand-maintained count, so
#: adding a question to the review automatically makes every existing sensor
#: incomplete until it is answered, which is the correct behaviour.
CHECKLIST: tuple[tuple[str, str, tuple[str, ...]], ...] = (
    ("purpose",     "What does it measure and why do we need it?",
     ("measures", "why")),
    ("mounting",    "Where does it need to be mounted?",
     ("location",)),
    ("output",      "What is the output type?",
     ("output",)),
    ("power",       "What voltage does it need / how do we power it?",
     ("supply_v", "current_ma", "supply_rail")),
    ("wiring",      "What connector / wiring does it need?",
     ("connector", "conductors")),
    ("rate",        "What sampling rate is needed?",
     ("sample_rate_hz", "signal_bandwidth_hz")),
    ("logging",     "Where is the signal logged?",
     ("logged_to",)),
    ("calibration", "How do we test / calibrate it?",
     ("calibration",)),
    ("range",       "What range and accuracy?",
     ("range_min_eu", "range_max_eu", "accuracy_eu")),
)


@dataclass
class SensorSpec:
    """One channel, and every question the review asks about it.

    EVERY engineering field is Optional and defaults to None. None means NOT
    ANSWERED YET, which this module reports as MISSING -- honestly distinct
    from a declared zero. That distinction is the entire point: a shared doc
    with an empty cell and a shared doc with a considered "0 mA, it is
    bus-powered" look the same to a human skimming it before a design review,
    and they are not the same at all.
    """
    key: str
    name: str

    # --- what and why -------------------------------------------------- #
    measures: Optional[str] = None          # the physical quantity
    unit: Optional[str] = None              # engineering unit ("degC", "L/min")
    why: Optional[str] = None               # what decision this data feeds

    # --- where ---------------------------------------------------------- #
    location: Optional[str] = None          # key into LOCATION_SUBTEAMS

    # --- how it talks ---------------------------------------------------- #
    output: Optional[OutputType] = None

    # --- power ----------------------------------------------------------- #
    supply_v: Optional[float] = None
    current_ma: Optional[float] = None
    supply_rail: Optional[str] = None       # key into the Rail table

    # --- wiring ------------------------------------------------------------ #
    connector: Optional[str] = None
    conductors: Optional[int] = None        # core count incl. shield drain

    # --- signal chain ------------------------------------------------------ #
    signal_bandwidth_hz: Optional[float] = None   # highest frequency of INTEREST
    sample_rate_hz: Optional[float] = None
    antialias_cutoff_hz: Optional[float] = None   # -3 dB of the filter in front
    adc_bits: Optional[int] = None
    range_min_eu: Optional[float] = None
    range_max_eu: Optional[float] = None
    accuracy_eu: Optional[float] = None           # +/- absolute, engineering units
    resolution_needed_eu: Optional[float] = None  # the smallest change that matters

    # --- logging ------------------------------------------------------------ #
    logged_to: Optional[str] = None         # "logger", "dash", "telemetry", ...
    payload_bytes: Optional[int] = None     # bytes on the wire per sample

    # --- test ---------------------------------------------------------------- #
    calibration: Optional[str] = None       # the actual procedure, not "TBD"

    # --- safety / integration ------------------------------------------------ #
    galvanic_isolation: Optional[bool] = None   # isolated from the TS side?
    available_on_existing_bus: Optional[str] = None  # device already broadcasting it

    # --- bookkeeping --------------------------------------------------------- #
    owner: str = ""
    source: str = ""                        # datasheet URL / part number
    is_estimate: bool = False               # numbers are placeholders
    notes: str = ""

    # ------------------------------------------------------------------ #
    def not_applicable(self) -> set[str]:
        """Checklist questions this channel structurally cannot have.

        Only one case qualifies, and it is narrow on purpose: a value that is
        already broadcast by a device on an existing bus consumes no rail and
        no connector of ITS OWN. The inverter is powered and wired regardless
        of whether anyone logs its temperature, so charging that channel with
        an unanswered power question would be counting someone else's work as
        this plan's debt.

        Nothing else is ever waived. "Not applicable" is not a way to close a
        question you have not thought about, which is why this is derived from
        the declaration rather than settable as a field.
        """
        na: set[str] = set()
        if self.output in BUS_TYPES and self.available_on_existing_bus:
            na.update({"power", "wiring"})
        return na

    def answered(self) -> dict[str, bool]:
        """Which checklist questions have every field they need.

        Inapplicable questions report True — they are closed, not open.
        """
        na = self.not_applicable()
        out = {}
        for qkey, _label, fields_ in CHECKLIST:
            if qkey in na:
                out[qkey] = True
                continue
            out[qkey] = all(getattr(self, f, None) is not None for f in fields_)
        return out

    def completeness(self) -> float:
        """Fraction of APPLICABLE checklist questions answered, 0..1."""
        na = self.not_applicable()
        applicable = [q for q, _, _ in CHECKLIST if q not in na]
        if not applicable:
            return 1.0
        a = self.answered()
        return sum(1 for q in applicable if a[q]) / len(applicable)

    def unanswered(self) -> list[str]:
        """Human-readable list of the questions still open."""
        a = self.answered()
        na = self.not_applicable()
        return [label for qkey, label, _ in CHECKLIST
                if qkey not in na and not a[qkey]]

    def affected_subteams(self) -> list[str]:
        """Everyone who owns a piece of this channel.

        Location gives the bracket owner. The rail and the bus always drag in
        electrics and data-acq. A tractive-system location additionally lands
        on whoever owns the isolation boundary.
        """
        out: list[str] = []
        for t in LOCATION_SUBTEAMS.get(self.location or "", ()):
            if t not in out:
                out.append(t)
        for t in ALWAYS_AFFECTED:
            if t not in out:
                out.append(t)
        return out

    def on_tractive_system(self) -> bool:
        return (self.location or "") in TRACTIVE_SYSTEM_LOCATIONS

    def span_eu(self) -> Optional[float]:
        if self.range_min_eu is None or self.range_max_eu is None:
            return None
        return abs(self.range_max_eu - self.range_min_eu)

    def as_dict(self) -> dict:
        d = asdict(self)
        if self.output is not None:
            d["output"] = self.output.value
        return d


# ===================================================================== #
#  2.  CAN BUS ARITHMETIC  (ISO 11898-1 field layout)
# ===================================================================== #
#: Fixed (non-data, non-stuffed) overhead bits of a CAN 2.0 data frame:
#: SOF(1) + RTR(1) + IDE(1) + r0(1) + DLC(4) + CRC(15) + CRCdel(1) + ACK(1)
#: + ACKdel(1) + EOF(7) + IFS(3), plus the identifier field.
_BASE_ID_BITS = 11
_EXT_ID_EXTRA_BITS = 20         # SRR + IDE + 18-bit ID extension + r1


def can_frame_bits(dlc: int, *, extended: bool = False,
                   worst_case_stuffing: bool = True) -> int:
    """Bits on the wire for one CAN 2.0 data frame carrying `dlc` data bytes.

    The stuffing rule inserts one complementary bit after five identical
    consecutive bits, and it applies from SOF through the CRC field. The
    worst-case bound is therefore floor((stuffable - 1) / 4) added bits.

    Sanity anchors, both standard published figures:
      * 8-byte standard-ID frame -> 135 bits worst case
      * 8-byte extended-ID frame -> 160 bits worst case

    Passing worst_case_stuffing=False gives the unstuffed floor, which is what
    an oscilloscope shows on friendly data and what an optimistic spreadsheet
    assumes. Plan on the worst case: it is the one that drops your frames.
    """
    if not (0 <= dlc <= 8):
        raise ValueError(f"CAN 2.0 DLC must be 0..8, got {dlc}")
    id_bits = _BASE_ID_BITS + (_EXT_ID_EXTRA_BITS if extended else 0)
    # Fixed overhead that is neither identifier nor data:
    # SOF(1) RTR(1) IDE(1) r0(1) DLC(4) CRC(15) CRCdel(1) ACK(1) ACKdel(1)
    # EOF(7) IFS(3). The extended frame's SRR, r1 and 18-bit ID extension are
    # carried in _EXT_ID_EXTRA_BITS, so IDE is counted here exactly once for
    # both formats.
    fixed = 1 + 1 + 1 + 1 + 4 + 15 + 1 + 1 + 1 + 7 + 3
    base = fixed + id_bits + 8 * dlc
    if not worst_case_stuffing:
        return base
    # Stuffing runs from SOF through the end of the CRC sequence, and stops at
    # the CRC delimiter — the delimiter, ACK, EOF and IFS are fixed-form fields.
    stuffable = 1 + id_bits + 1 + 1 + 1 + 4 + 8 * dlc + 15
    return base + (stuffable - 1) // 4


@dataclass
class CanMessage:
    """One periodic message on the bus."""
    name: str
    can_id: int
    dlc: int
    rate_hz: float
    extended: bool = False
    producer: str = ""
    signals: list = field(default_factory=list)   # names carried in this frame

    def bits(self, worst_case_stuffing: bool = True) -> int:
        return can_frame_bits(self.dlc, extended=self.extended,
                              worst_case_stuffing=worst_case_stuffing)

    def frame_time_s(self, bitrate_bps: float,
                     worst_case_stuffing: bool = True) -> float:
        return self.bits(worst_case_stuffing) / float(bitrate_bps)


@dataclass
class BusSpec:
    """The physical bus the messages have to fit inside."""
    name: str = "GLV CAN"
    bitrate_bps: float = 500_000.0
    extended_ids: bool = False
    #: Load thresholds. Below `load_ok` a bus is comfortable; between ok and
    #: warn it works but latency starts to matter; above `load_fail` arbitration
    #: delays for low-priority frames grow without bound and buffers overrun.
    load_ok: float = 0.30
    load_warn: float = 0.50
    load_fail: float = 0.80


@dataclass
class BusLoadResult:
    bus: BusSpec
    load: float                       # 0..1, worst-case-stuffed
    load_unstuffed: float
    bits_per_second: float
    messages: int
    per_message: dict                 # name -> {bits, rate_hz, load}
    latencies: dict                   # name -> worst-case response time (s)
    unschedulable: list               # names whose response time diverged
    is_floor: bool = False            # True when undeclared channels exist
    findings: list = field(default_factory=list)


def bus_load(messages: list[CanMessage], bus: BusSpec) -> BusLoadResult:
    """Exact bus utilisation and worst-case per-message latency.

    Load is the straightforward sum of (frame bits x rate) / bitrate. The
    latency is the standard fixed-point response-time analysis for
    non-preemptive fixed-priority arbitration: a message waits for at most one
    lower-priority frame already in transmission (blocking, because CAN
    arbitration cannot preempt a frame in flight), plus every higher-priority
    frame that can arrive during its own busy period.

    Assumptions stated plainly: strictly periodic transmission, no queueing
    jitter, one message per identifier, and a controller that always has the
    highest-priority pending frame ready to arbitrate. Real ECUs with FIFO
    transmit buffers do worse than this. Treat the numbers as a lower bound on
    latency, not an upper one.
    """
    findings: list[Finding] = []
    if not messages:
        return BusLoadResult(bus, 0.0, 0.0, 0.0, 0, {}, {}, [], findings=findings)

    br = float(bus.bitrate_bps)
    tau_bit = 1.0 / br

    # --- duplicate identifiers: two producers, one arbitration slot ------- #
    seen: dict[int, str] = {}
    for m in messages:
        if m.can_id in seen:
            findings.append(Finding(
                "can-id-collision", Severity.FAIL,
                f"CAN ID 0x{m.can_id:03X} is claimed by both '{seen[m.can_id]}' "
                f"and '{m.name}'. Two nodes transmitting the same identifier "
                f"with different data corrupts arbitration and produces error "
                f"frames, not a merged message.",
                subsystems=["dataacq", "electrics"],
                detail={"can_id": m.can_id, "a": seen[m.can_id], "b": m.name}))
        seen[m.can_id] = m.name

    # --- identifier range -------------------------------------------------- #
    for m in messages:
        limit = 0x1FFFFFFF if m.extended else 0x7FF
        if not (0 <= m.can_id <= limit):
            findings.append(Finding(
                "can-id-range", Severity.FAIL,
                f"'{m.name}' uses ID 0x{m.can_id:X}, outside the "
                f"{'29' if m.extended else '11'}-bit range.",
                subsystems=["dataacq"], detail={"can_id": m.can_id}))

    per: dict[str, dict] = {}
    bits_ps = 0.0
    bits_ps_un = 0.0
    for m in messages:
        b = m.bits(True)
        bu = m.bits(False)
        contrib = b * max(m.rate_hz, 0.0)
        bits_ps += contrib
        bits_ps_un += bu * max(m.rate_hz, 0.0)
        per[m.name] = {
            "bits": b, "bits_unstuffed": bu, "rate_hz": m.rate_hz,
            "load": contrib / br, "can_id": m.can_id, "dlc": m.dlc,
            "producer": m.producer,
        }

    load = bits_ps / br
    load_un = bits_ps_un / br

    # --- worst-case response times ---------------------------------------- #
    ordered = sorted(messages, key=lambda m: m.can_id)   # lower ID = higher prio
    latencies: dict[str, float] = {}
    unschedulable: list[str] = []
    for i, m in enumerate(ordered):
        c_m = m.frame_time_s(br)
        period = 1.0 / m.rate_hz if m.rate_hz > 0 else math.inf
        lower = ordered[i + 1:]
        blocking = max((x.frame_time_s(br) for x in lower), default=0.0)
        hp = ordered[:i]
        w = blocking + c_m
        ok = False
        for _ in range(500):
            nxt = blocking + c_m
            for x in hp:
                if x.rate_hz <= 0:
                    continue
                nxt += math.ceil((w + tau_bit) * x.rate_hz) * x.frame_time_s(br)
            if abs(nxt - w) < 1e-12:
                ok = True
                break
            w = nxt
            if w > period and math.isfinite(period):
                break
        if ok and w <= period:
            latencies[m.name] = w
        else:
            latencies[m.name] = float("inf")
            unschedulable.append(m.name)

    # --- verdict on the load ----------------------------------------------- #
    if load >= bus.load_fail:
        findings.append(Finding(
            "bus-over-budget", Severity.FAIL,
            f"{bus.name} is at {load*100:.0f}% worst-case load "
            f"({bits_ps/1000:.1f} kbit/s of {br/1000:.0f} kbit/s). Above "
            f"{bus.load_fail*100:.0f}% low-priority frames queue behind bursts "
            f"and transmit buffers overrun. Either raise the bitrate, cut "
            f"sample rates, or move channels to a second bus.",
            subsystems=["dataacq", "electrics"],
            detail={"load": load, "bitrate_bps": br}))
    elif load >= bus.load_warn:
        findings.append(Finding(
            "bus-load-high", Severity.WARN,
            f"{bus.name} is at {load*100:.0f}% worst-case load. It works, but "
            f"latency for the lowest-priority messages is now sensitive to any "
            f"channel you add. Budget the remaining headroom deliberately.",
            subsystems=["dataacq"], detail={"load": load}))
    elif load >= bus.load_ok:
        findings.append(Finding(
            "bus-load-moderate", Severity.INFO,
            f"{bus.name} at {load*100:.0f}% worst-case load — comfortable, with "
            f"{(bus.load_warn-load)*100:.0f} points of headroom before latency "
            f"becomes a design constraint.",
            subsystems=["dataacq"], detail={"load": load}))
    else:
        findings.append(Finding(
            "bus-load-ok", Severity.OK,
            f"{bus.name} at {load*100:.0f}% worst-case load "
            f"({load_un*100:.0f}% without bit stuffing).",
            subsystems=["dataacq"], detail={"load": load}))

    for nm in unschedulable:
        findings.append(Finding(
            "message-deadline-miss", Severity.FAIL,
            f"'{nm}' cannot reliably complete transmission within its own "
            f"period at this bus load. Its worst-case arbitration delay exceeds "
            f"the interval before the next sample is due, so samples will be "
            f"overwritten in the transmit buffer before they reach the bus.",
            subsystems=["dataacq"], detail={"message": nm}))

    return BusLoadResult(bus=bus, load=load, load_unstuffed=load_un,
                         bits_per_second=bits_ps, messages=len(messages),
                         per_message=per, latencies=latencies,
                         unschedulable=unschedulable, findings=findings)


# ===================================================================== #
#  3.  SIGNAL-CHAIN CHECKS  (the arithmetic behind "what rate is needed?")
# ===================================================================== #
#: Sample-rate multiples of the signal bandwidth. 2x is the theorem's floor and
#: reconstructs a band-limited signal exactly in principle; in practice, for
#: reading peaks and shapes off a plot, you want an order of magnitude.
NYQUIST_FLOOR = 2.0
NYQUIST_PRACTICAL = 5.0
NYQUIST_COMFORTABLE = 10.0
#: Above this multiple the channel is spending bus and storage for information
#: that is not there.
OVERSAMPLE_WASTE = 50.0


def signal_chain_findings(s: SensorSpec) -> list[Finding]:
    """Nyquist, anti-aliasing and ADC resolution for one channel."""
    out: list[Finding] = []
    who = s.affected_subteams()

    fs = s.sample_rate_hz
    bw = s.signal_bandwidth_hz

    # ---- Nyquist ------------------------------------------------------- #
    if fs is not None and bw is not None and bw > 0:
        ratio = fs / bw
        if ratio < NYQUIST_FLOOR:
            out.append(Finding(
                "nyquist-violation", Severity.FAIL,
                f"{s.name}: sampling at {fs:g} Hz a signal with content to "
                f"{bw:g} Hz ({ratio:.1f}x). Everything above {fs/2:g} Hz folds "
                f"down and appears as a lower frequency that is "
                f"indistinguishable from real data. This is not noise you can "
                f"filter out afterwards — the information is destroyed at the "
                f"ADC. Sample at {NYQUIST_PRACTICAL*bw:g} Hz or above.",
                subsystems=who,
                detail={"fs": fs, "bandwidth": bw, "ratio": ratio}))
        elif ratio < NYQUIST_PRACTICAL:
            out.append(Finding(
                "nyquist-marginal", Severity.WARN,
                f"{s.name}: {ratio:.1f}x oversampling clears the theorem but "
                f"not practice. Peaks land between samples and the trace shape "
                f"is unreliable. {NYQUIST_COMFORTABLE*bw:g} Hz would let you "
                f"read this channel off a plot with confidence.",
                subsystems=who, detail={"ratio": ratio}))
        elif ratio > OVERSAMPLE_WASTE:
            out.append(Finding(
                "oversampled", Severity.INFO,
                f"{s.name}: {ratio:.0f}x the signal bandwidth. Nothing is wrong "
                f"with the data, but every one of those samples costs bus and "
                f"card. Dropping to {NYQUIST_COMFORTABLE*bw:g} Hz would free "
                f"{(1 - NYQUIST_COMFORTABLE/ratio)*100:.0f}% of this channel's "
                f"load with no loss of information.",
                subsystems=who, detail={"ratio": ratio}))
        else:
            out.append(Finding(
                "nyquist-ok", Severity.OK,
                f"{s.name}: {ratio:.0f}x oversampled — appropriate.",
                subsystems=who, detail={"ratio": ratio}))
    elif fs is None or bw is None:
        missing = []
        if fs is None:
            missing.append("sample rate")
        if bw is None:
            missing.append("signal bandwidth")
        out.append(Finding(
            "rate-undeclared", Severity.MISSING,
            f"{s.name}: {' and '.join(missing)} not declared, so the Nyquist "
            f"check cannot run and this channel contributes nothing to the bus "
            f"budget — which means the budget is a floor, not an answer.",
            subsystems=who, detail={"missing": missing}))

    # ---- anti-alias filter ------------------------------------------------ #
    if s.output in ANALOG_TYPES:
        if s.antialias_cutoff_hz is None:
            out.append(Finding(
                "antialias-undeclared", Severity.WARN,
                f"{s.name} is an analog channel with no declared anti-alias "
                f"filter. The filter is what makes the sample rate meaningful; "
                f"without one, any interference above half the sample rate "
                f"(switching noise, inverter carrier, RF) folds into the band "
                f"you care about and cannot be separated from it afterwards.",
                subsystems=who))
        elif fs is not None and s.antialias_cutoff_hz > fs / 2.0:
            out.append(Finding(
                "antialias-too-high", Severity.FAIL,
                f"{s.name}: anti-alias cutoff {s.antialias_cutoff_hz:g} Hz sits "
                f"above the {fs/2:g} Hz Nyquist frequency, so the filter passes "
                f"exactly the content it exists to remove. Set the cutoff at or "
                f"below {fs/2:g} Hz — typically {fs/4:g} Hz to leave room for "
                f"the filter's own roll-off.",
                subsystems=who,
                detail={"cutoff": s.antialias_cutoff_hz, "nyquist": fs / 2.0}))

    # ---- ADC resolution ---------------------------------------------------- #
    span = s.span_eu()
    if span is not None and s.adc_bits:
        step = span / (2 ** s.adc_bits - 1)
        need = s.resolution_needed_eu
        unit = s.unit or "EU"
        if need is not None and step > need:
            out.append(Finding(
                "adc-resolution-short", Severity.FAIL,
                f"{s.name}: {s.adc_bits}-bit over a {span:g} {unit} span gives "
                f"{step:.4g} {unit} per count, coarser than the {need:g} {unit} "
                f"this channel was specified to resolve. Either narrow the "
                f"range with gain in front of the ADC, or use more bits.",
                subsystems=who, detail={"step": step, "needed": need}))
        elif need is not None:
            out.append(Finding(
                "adc-resolution-ok", Severity.OK,
                f"{s.name}: {step:.4g} {unit}/count vs {need:g} {unit} needed "
                f"({need/step:.0f}x margin).",
                subsystems=who, detail={"step": step}))
        # accuracy dominated by the sensor, not the ADC?
        if s.accuracy_eu is not None and s.accuracy_eu > 0 and step < s.accuracy_eu / 10:
            out.append(Finding(
                "adc-over-resolved", Severity.INFO,
                f"{s.name}: the ADC resolves {step:.4g} {unit} but the sensor is "
                f"only accurate to +/-{s.accuracy_eu:g} {unit}. The extra bits "
                f"are digitising the sensor's own error — useful for averaging, "
                f"misleading if anyone reads the last digits as truth.",
                subsystems=who, detail={"step": step, "accuracy": s.accuracy_eu}))

    # ---- counter channels are not sampled --------------------------------- #
    if s.output in COUNTER_TYPES and fs is not None:
        out.append(Finding(
            "counter-not-sampled", Severity.INFO,
            f"{s.name} is a {s.output.value} output. It is not sampled — it is "
            f"counted over a gate window, and its resolution comes from the "
            f"gate time and the pulses-per-unit constant, not from a sample "
            f"rate or ADC bits. The {fs:g} Hz figure is the update rate of the "
            f"computed value; make sure the gate window is documented too.",
            subsystems=who))

    return out


# ===================================================================== #
#  4.  DELTA-T UNCERTAINTY  (why a pair of cheap temp sensors fails)
# ===================================================================== #
#: 50/50 water-ethylene-glycol at ~60 degC, the usual FSAE coolant.
COOLANT_RHO = 1030.0        # kg/m^3
COOLANT_CP = 3400.0         # J/(kg*K)


@dataclass
class DeltaTResult:
    delta_t_k: float
    sigma_delta_t_k: float
    relative_error: float           # sigma_dT / dT
    heat_kw: Optional[float]
    sigma_heat_kw: Optional[float]
    heat_relative_error: Optional[float]
    findings: list = field(default_factory=list)


def delta_t_budget(inlet: SensorSpec, outlet: SensorSpec,
                   *, expected_delta_t_k: float,
                   flow_lpm: Optional[float] = None,
                   flow_accuracy_frac: float = 0.03,
                   rho: float = COOLANT_RHO, cp: float = COOLANT_CP,
                   matched_pair: bool = False) -> DeltaTResult:
    """Error propagation for a coolant inlet/outlet temperature pair.

    This is the check that decides whether a coolant instrumentation plan is
    capable of measuring the thing it was designed to measure. Two temperature
    sensors are bought to produce a DIFFERENCE, and a difference inherits the
    error of both ends while keeping only a fraction of the magnitude:

        sigma_dT = sqrt(sigma_in^2 + sigma_out^2)     (uncorrelated)

    A pair of +/-2 K sensors reading a 5 K rise gives +/-2.8 K on a 5 K number:
    a 57% measurement. The heat-rejection figure computed from it is not a
    measurement, it is a rumour.

    `matched_pair=True` models sensors calibrated together against a common
    reference, which removes the shared systematic offset and leaves only the
    residual. That is usually the difference between a useless channel pair and
    a good one, and it costs an afternoon with a stirred water bath rather than
    a different purchase order.
    """
    findings: list[Finding] = []
    sa = inlet.accuracy_eu
    sb = outlet.accuracy_eu
    if sa is None or sb is None:
        findings.append(Finding(
            "delta-t-uncheckable", Severity.MISSING,
            "Coolant delta-T uncertainty cannot be computed: sensor accuracy "
            "is not declared for both ends. Until it is, any heat-rejection "
            "number from this pair has unknown error.",
            subsystems=["cooling", "dataacq"]))
        return DeltaTResult(expected_delta_t_k, float("nan"), float("nan"),
                            None, None, None, findings)

    if matched_pair:
        # A common-mode offset cancels in the difference; assume calibration
        # knocks the correlated part down to a quarter of the spec accuracy.
        sa, sb = sa * 0.25, sb * 0.25

    sigma = math.sqrt(sa ** 2 + sb ** 2)
    rel = sigma / expected_delta_t_k if expected_delta_t_k else float("inf")

    heat = sigma_heat = heat_rel = None
    if flow_lpm is not None:
        m_dot = rho * (flow_lpm / 1000.0 / 60.0)      # kg/s
        heat = m_dot * cp * expected_delta_t_k / 1000.0    # kW
        heat_rel = math.sqrt(flow_accuracy_frac ** 2 + rel ** 2)
        sigma_heat = heat * heat_rel

    if rel > 0.5:
        findings.append(Finding(
            "delta-t-unusable", Severity.FAIL,
            f"Coolant delta-T is +/-{sigma:.2f} K on an expected {expected_delta_t_k:g} K "
            f"rise — {rel*100:.0f}% error. These two sensors cannot measure the "
            f"quantity they are being bought for. Calibrate them as a MATCHED "
            f"PAIR against one reference (a stirred bath and an afternoon), "
            f"which cancels the shared offset, or buy a single dual-probe "
            f"delta-T sensor.",
            subsystems=["cooling", "dataacq"],
            detail={"sigma_k": sigma, "relative": rel}))
    elif rel > 0.2:
        findings.append(Finding(
            "delta-t-marginal", Severity.WARN,
            f"Coolant delta-T is +/-{sigma:.2f} K on {expected_delta_t_k:g} K "
            f"({rel*100:.0f}%). Usable for trends, too coarse to size a "
            f"radiator from. A matched-pair calibration would cut it roughly "
            f"fourfold.",
            subsystems=["cooling", "dataacq"],
            detail={"sigma_k": sigma, "relative": rel}))
    else:
        findings.append(Finding(
            "delta-t-ok", Severity.OK,
            f"Coolant delta-T +/-{sigma:.2f} K on {expected_delta_t_k:g} K "
            f"({rel*100:.0f}%) — good enough to size from.",
            subsystems=["cooling"], detail={"relative": rel}))

    if heat is not None:
        findings.append(Finding(
            "heat-rejection", Severity.INFO,
            f"Implied heat rejection {heat:.1f} +/-{sigma_heat:.1f} kW "
            f"({heat_rel*100:.0f}%) at {flow_lpm:g} L/min. That uncertainty is "
            f"the honest width of the radiator-sizing input.",
            subsystems=["cooling"],
            detail={"heat_kw": heat, "sigma_kw": sigma_heat}))

    return DeltaTResult(expected_delta_t_k, sigma, rel,
                        heat, sigma_heat, heat_rel, findings)


# ===================================================================== #
#  5.  POWER AND STORAGE BUDGETS
# ===================================================================== #
@dataclass
class Rail:
    name: str
    voltage_v: float
    capacity_ma: float              # what the regulator can actually deliver
    fuse_a: Optional[float] = None


@dataclass
class PowerResult:
    per_rail: dict                  # rail -> {draw_ma, capacity_ma, frac, sensors}
    undeclared: list                # sensors with no declared current
    is_floor: bool
    findings: list = field(default_factory=list)


def power_budget(sensors: list[SensorSpec], rails: dict[str, Rail]) -> PowerResult:
    """Sum sensor current per rail against what the rail can supply.

    Steady-state only, and deliberately so — this does not model inrush,
    thermistor self-heating or the excitation current an RTD bridge draws
    beyond its quiescent figure. It is the arithmetic the checklist implies
    when it asks every sensor what voltage it needs and then never adds the
    column up.
    """
    findings: list[Finding] = []
    per: dict[str, dict] = {
        k: {"draw_ma": 0.0, "capacity_ma": r.capacity_ma, "frac": 0.0,
            "voltage_v": r.voltage_v, "sensors": []}
        for k, r in rails.items()
    }
    undeclared: list[str] = []

    for s in sensors:
        if s.output in BUS_TYPES and s.current_ma is None and s.supply_rail is None:
            continue        # bus-powered device, nothing to budget
        rail = s.supply_rail
        if s.current_ma is None:
            undeclared.append(s.name)
            continue
        if rail is None or rail not in per:
            findings.append(Finding(
                "rail-unassigned", Severity.MISSING,
                f"{s.name} draws {s.current_ma:g} mA but is not assigned to a "
                f"declared rail, so its current lands in nobody's budget.",
                subsystems=s.affected_subteams(), detail={"sensor": s.key}))
            continue
        per[rail]["draw_ma"] += s.current_ma
        per[rail]["sensors"].append(s.name)
        # voltage compatibility
        if s.supply_v is not None:
            rv = rails[rail].voltage_v
            if abs(s.supply_v - rv) > 0.5:
                findings.append(Finding(
                    "rail-voltage-mismatch", Severity.FAIL,
                    f"{s.name} needs {s.supply_v:g} V but is assigned to the "
                    f"{rail} rail at {rv:g} V.",
                    subsystems=s.affected_subteams(),
                    detail={"needs_v": s.supply_v, "rail_v": rv}))

    for k, d in per.items():
        cap = d["capacity_ma"] or 0.0
        d["frac"] = (d["draw_ma"] / cap) if cap else float("inf")
        if cap and d["draw_ma"] > cap:
            findings.append(Finding(
                "rail-over-capacity", Severity.FAIL,
                f"{k} rail: {d['draw_ma']:.0f} mA of sensor draw against a "
                f"{cap:.0f} mA regulator. The regulator will current-limit or "
                f"go thermal, and the first symptom is sensors reading wrong "
                f"rather than a clean failure.",
                subsystems=["electrics", "dataacq"], detail={"rail": k}))
        elif cap and d["draw_ma"] > 0.8 * cap:
            findings.append(Finding(
                "rail-tight", Severity.WARN,
                f"{k} rail at {d['frac']*100:.0f}% of capacity "
                f"({d['draw_ma']:.0f}/{cap:.0f} mA). Little room for the next "
                f"sensor, and none for inrush.",
                subsystems=["electrics", "dataacq"], detail={"rail": k}))
        elif cap:
            findings.append(Finding(
                "rail-ok", Severity.OK,
                f"{k} rail at {d['frac']*100:.0f}% "
                f"({d['draw_ma']:.0f}/{cap:.0f} mA).",
                subsystems=["electrics"], detail={"rail": k}))

    if undeclared:
        findings.append(Finding(
            "current-undeclared", Severity.MISSING,
            f"{len(undeclared)} sensor(s) have no declared current draw "
            f"({', '.join(undeclared[:4])}{'…' if len(undeclared) > 4 else ''}). "
            f"Every rail total below is a FLOOR — undeclared draw can only add.",
            subsystems=["electrics", "dataacq"],
            detail={"sensors": undeclared}))

    return PowerResult(per, undeclared, bool(undeclared), findings)


@dataclass
class LoggerSpec:
    name: str = "logger"
    storage_mb: float = 8192.0
    session_minutes: float = 25.0       # an endurance run plus margin
    record_overhead_bytes: int = 4      # timestamp / channel id per sample


@dataclass
class StorageResult:
    bytes_per_s: float
    session_mb: float
    storage_mb: float
    frac: float
    is_floor: bool
    findings: list = field(default_factory=list)


def storage_budget(sensors: list[SensorSpec], logger: LoggerSpec) -> StorageResult:
    """Bytes per second on the card, and whether a session fits."""
    findings: list[Finding] = []
    bps = 0.0
    undeclared = 0
    for s in sensors:
        if s.sample_rate_hz is None:
            undeclared += 1
            continue
        payload = s.payload_bytes if s.payload_bytes is not None else 2
        bps += (payload + logger.record_overhead_bytes) * s.sample_rate_hz

    session_mb = bps * logger.session_minutes * 60.0 / 1e6
    frac = session_mb / logger.storage_mb if logger.storage_mb else float("inf")

    if frac > 1.0:
        findings.append(Finding(
            "storage-over", Severity.FAIL,
            f"A {logger.session_minutes:g}-minute session writes "
            f"{session_mb:.0f} MB to a {logger.storage_mb:.0f} MB card. Logging "
            f"stops partway through the run, and it stops silently.",
            subsystems=["dataacq"], detail={"session_mb": session_mb}))
    elif frac > 0.7:
        findings.append(Finding(
            "storage-tight", Severity.WARN,
            f"{session_mb:.0f} MB per session on a {logger.storage_mb:.0f} MB "
            f"card ({frac*100:.0f}%) — one forgotten download and the next "
            f"session has nowhere to go.",
            subsystems=["dataacq"], detail={"session_mb": session_mb}))
    else:
        findings.append(Finding(
            "storage-ok", Severity.OK,
            f"{bps/1000:.1f} kB/s, {session_mb:.0f} MB per "
            f"{logger.session_minutes:g}-minute session "
            f"({frac*100:.0f}% of the card).",
            subsystems=["dataacq"], detail={"session_mb": session_mb}))

    if undeclared:
        findings.append(Finding(
            "storage-floor", Severity.MISSING,
            f"{undeclared} channel(s) have no sample rate, so this figure is a "
            f"floor.", subsystems=["dataacq"]))

    return StorageResult(bps, session_mb, logger.storage_mb, frac,
                         bool(undeclared), findings)


# ===================================================================== #
#  6.  THE BMS BRIDGE  (UART in, CAN out)
# ===================================================================== #
@dataclass
class UartLink:
    """The serial side of the bridge.

    `bits_per_byte` is the framing overhead: 8N1 is 10 bits per byte (start +
    8 data + stop), 8E1 or 8N2 is 11. Getting this wrong by one bit is a 10%
    error in the link budget, which is exactly the size of margin people
    assume they have.
    """
    baud: int = 115_200
    data_bits: int = 8
    parity: Optional[str] = None        # None, "even", "odd"
    stop_bits: int = 1
    frame_bytes: Optional[int] = None   # bytes in one BMS message
    frame_rate_hz: Optional[float] = None
    #: Fraction of the raw baud you can actually plan on. Real links lose time
    #: to inter-byte gaps, resynchronisation and the host's turnaround.
    usable_fraction: float = 0.80

    def bits_per_byte(self) -> int:
        return 1 + self.data_bits + (1 if self.parity else 0) + self.stop_bits

    def byte_time_s(self) -> float:
        return self.bits_per_byte() / float(self.baud)

    def frame_time_s(self) -> Optional[float]:
        if self.frame_bytes is None:
            return None
        return self.frame_bytes * self.byte_time_s()

    def utilisation(self) -> Optional[float]:
        ft = self.frame_time_s()
        if ft is None or self.frame_rate_hz is None:
            return None
        return ft * self.frame_rate_hz


@dataclass
class BmsSignal:
    """One value the BMS reports, as it exists in the serial frame."""
    name: str
    bits: int
    unit: str = ""
    scale: float = 1.0
    offset: float = 0.0
    rate_hz: Optional[float] = None      # how often it needs to reach the bus
    critical: bool = False               # shutdown-relevant (cell V, temp, current)


@dataclass
class BridgePlan:
    link: UartLink
    signals: list
    messages: list                       # CanMessage list produced
    uart_utilisation: Optional[float]
    latency_s: Optional[float]
    refused: bool
    refusal_reason: str = ""
    findings: list = field(default_factory=list)


def pack_signals(signals: list[BmsSignal], *, base_id: int = 0x300,
                 extended: bool = False,
                 default_rate_hz: float = 10.0,
                 producer: str = "bms_bridge") -> list[CanMessage]:
    """Pack signals into 8-byte CAN frames, grouped by transmit rate.

    Signals that update at different rates must not share a frame — putting a
    10 Hz cell voltage in the same frame as a 100 Hz pack current forces one of
    them to the other's rate, which either wastes nine tenths of the bandwidth
    or delivers the fast one late. Grouping by rate first is the whole trick.

    Critical (shutdown-relevant) signals are placed in the lowest identifiers
    within their rate group, because on CAN a lower identifier wins arbitration
    — priority is not a field you set, it is the number you choose.
    """
    by_rate: dict[float, list[BmsSignal]] = {}
    for s in signals:
        r = s.rate_hz if s.rate_hz is not None else default_rate_hz
        by_rate.setdefault(r, []).append(s)

    msgs: list[CanMessage] = []
    next_id = base_id
    # fastest rate gets the lowest ids -> highest priority
    for rate in sorted(by_rate.keys(), reverse=True):
        group = sorted(by_rate[rate], key=lambda s: (not s.critical, s.name))
        bit_cursor = 0
        cur: list[BmsSignal] = []
        for s in group:
            if s.bits > 64:
                raise ValueError(f"signal {s.name} is {s.bits} bits — "
                                 f"cannot fit one CAN 2.0 frame")
            if bit_cursor + s.bits > 64:
                msgs.append(CanMessage(
                    name=f"BMS_{int(rate)}Hz_{len(msgs)}", can_id=next_id,
                    dlc=math.ceil(bit_cursor / 8), rate_hz=rate,
                    extended=extended, producer=producer,
                    signals=[x.name for x in cur]))
                next_id += 1
                bit_cursor = 0
                cur = []
            bit_cursor += s.bits
            cur.append(s)
        if cur:
            msgs.append(CanMessage(
                name=f"BMS_{int(rate)}Hz_{len(msgs)}", can_id=next_id,
                dlc=math.ceil(bit_cursor / 8), rate_hz=rate,
                extended=extended, producer=producer,
                signals=[x.name for x in cur]))
            next_id += 1
    return msgs


def plan_bms_bridge(link: UartLink, signals: list[BmsSignal], *,
                    base_id: int = 0x300, extended: bool = False,
                    bus: Optional[BusSpec] = None,
                    isolated: Optional[bool] = None,
                    parse_overhead_s: float = 0.0005) -> BridgePlan:
    """Design the UART-to-CAN bridge, or refuse to.

    THE REFUSAL: with no declared signal list, this returns a refused plan and
    no frame map. "How do we get the BMS onto CAN" reads like a wiring question
    and is actually a datasheet question — the answer depends entirely on what
    the serial frame contains, and any frame layout produced before someone has
    read that is invented. A tool that emits a confident-looking CAN map from
    an empty signal list has manufactured the appearance of progress, which is
    worse than an empty page because it stops anyone opening the datasheet.
    """
    findings: list[Finding] = []

    if not signals:
        findings.append(Finding(
            "bms-signals-unknown", Severity.MISSING,
            "No BMS signals declared, so no bridge can be designed. This is the "
            "'what signals exist / how do we read them' question, and it has to "
            "be answered from the datasheet before anything downstream is real: "
            "the serial frame layout sets the CAN frame layout, the update rate "
            "sets the bus load, and the value widths set the scaling. Read the "
            "datasheet, list the signals with their bit widths and rates, and "
            "this will design the rest.",
            subsystems=["dataacq", "electrics"]))
        return BridgePlan(link, [], [], link.utilisation(), None,
                          refused=True,
                          refusal_reason="BMS signal list is empty",
                          findings=findings)

    # ---- isolation: the BMS sits on the tractive-system side --------------- #
    if isolated is not True:
        findings.append(Finding(
            "bms-isolation", Severity.FAIL if isolated is False else Severity.MISSING,
            "The BMS is referenced to the accumulator, on the tractive-system "
            "side of the isolation boundary. Its UART must cross into the "
            "grounded low-voltage system through a galvanic barrier — a digital "
            "isolator on the serial lines with an isolated supply on the BMS "
            "side. Without it the serial ground wire is a conductive path "
            "between the two systems, which is both a rules failure and the "
            "reason a single fault takes out the logger and the BMS together."
            + ("" if isolated is False else " Isolation has not been declared "
               "either way; declare it explicitly."),
            subsystems=["electrics", "dataacq"]))
    else:
        findings.append(Finding(
            "bms-isolation-ok", Severity.OK,
            "Galvanic isolation declared on the BMS serial link.",
            subsystems=["electrics"]))

    # ---- UART link budget --------------------------------------------------- #
    util = link.utilisation()
    if util is None:
        findings.append(Finding(
            "uart-budget-uncheckable", Severity.MISSING,
            "UART frame size and/or frame rate not declared — the link budget "
            "cannot be checked, so nothing confirms the BMS can actually stream "
            "at the rate the plan assumes.",
            subsystems=["dataacq"]))
    elif util > 1.0:
        findings.append(Finding(
            "uart-over-capacity", Severity.FAIL,
            f"The BMS frame ({link.frame_bytes} bytes at "
            f"{link.bits_per_byte()} bits/byte) needs "
            f"{util*100:.0f}% of {link.baud} baud to sustain "
            f"{link.frame_rate_hz:g} Hz. The link physically cannot carry it. "
            f"Either raise the baud rate or accept a lower update rate — and "
            f"note that a BMS often will not let you do the first.",
            subsystems=["dataacq", "electrics"], detail={"utilisation": util}))
    elif util > link.usable_fraction:
        findings.append(Finding(
            "uart-tight", Severity.WARN,
            f"UART at {util*100:.0f}% of raw baud, above the "
            f"{link.usable_fraction*100:.0f}% you can realistically plan on "
            f"once inter-byte gaps and resynchronisation are accounted for. "
            f"Expect dropped or partial frames under load.",
            subsystems=["dataacq"], detail={"utilisation": util}))
    else:
        findings.append(Finding(
            "uart-ok", Severity.OK,
            f"UART link at {util*100:.0f}% utilisation "
            f"({link.frame_bytes} B x {link.frame_rate_hz:g} Hz at {link.baud} baud).",
            subsystems=["dataacq"], detail={"utilisation": util}))

    # ---- rate sanity: nothing can leave faster than it arrives -------------- #
    if link.frame_rate_hz is not None:
        too_fast = [s for s in signals
                    if s.rate_hz is not None and s.rate_hz > link.frame_rate_hz]
        if too_fast:
            findings.append(Finding(
                "bridge-rate-impossible", Severity.FAIL,
                f"{len(too_fast)} signal(s) are scheduled onto CAN faster than "
                f"the BMS produces them ({link.frame_rate_hz:g} Hz): "
                f"{', '.join(s.name for s in too_fast[:4])}. The extra frames "
                f"would repeat stale values, which looks like fresh data on a "
                f"plot and is not.",
                subsystems=["dataacq"],
                detail={"signals": [s.name for s in too_fast]}))

    msgs = pack_signals(signals, base_id=base_id, extended=extended,
                        default_rate_hz=(link.frame_rate_hz or 10.0))

    # ---- end-to-end latency -------------------------------------------------- #
    latency = None
    ft = link.frame_time_s()
    if ft is not None:
        can_time = 0.0
        if bus is not None and msgs:
            can_time = max(m.frame_time_s(bus.bitrate_bps) for m in msgs)
        latency = ft + parse_overhead_s + can_time
        crit = [s for s in signals if s.critical]
        if crit and latency > 0.100:
            findings.append(Finding(
                "bridge-latency-high", Severity.WARN,
                f"Shutdown-relevant signals ({', '.join(s.name for s in crit[:3])}) "
                f"reach the bus {latency*1000:.0f} ms after the BMS measures "
                f"them. That is fine for logging and too slow to be part of any "
                f"protective action — the shutdown circuit must remain hardware, "
                f"not a CAN message.",
                subsystems=["electrics", "dataacq"],
                detail={"latency_s": latency}))
        else:
            findings.append(Finding(
                "bridge-latency", Severity.INFO,
                f"End-to-end bridge latency approximately {latency*1000:.1f} ms "
                f"(UART frame {ft*1000:.1f} ms + parse {parse_overhead_s*1000:.1f} ms "
                f"+ CAN {can_time*1000:.2f} ms).",
                subsystems=["dataacq"], detail={"latency_s": latency}))

    findings.append(Finding(
        "bridge-mapped", Severity.OK,
        f"{len(signals)} BMS signals map into {len(msgs)} CAN frames "
        f"(IDs 0x{base_id:03X}–0x{base_id+len(msgs)-1:03X}).",
        subsystems=["dataacq"], detail={"messages": len(msgs)}))

    return BridgePlan(link, list(signals), msgs, util, latency,
                      refused=False, findings=findings)


# ===================================================================== #
#  7.  DOCUMENTATION + INTEGRATION CHECKS PER SENSOR
# ===================================================================== #
def documentation_findings(s: SensorSpec) -> list[Finding]:
    """The checklist, enforced."""
    out: list[Finding] = []
    open_q = s.unanswered()
    who = s.affected_subteams()
    n_applicable = len(CHECKLIST) - len(s.not_applicable())
    if open_q:
        out.append(Finding(
            "doc-incomplete", Severity.MISSING,
            f"{s.name}: {len(open_q)} of {n_applicable} applicable review "
            f"questions unanswered — {'; '.join(open_q)}.",
            subsystems=who,
            detail={"open": open_q, "completeness": s.completeness()}))
    else:
        out.append(Finding(
            "doc-complete", Severity.OK,
            f"{s.name}: every review question answered.",
            subsystems=who, detail={"completeness": 1.0}))

    if s.is_estimate:
        out.append(Finding(
            "doc-estimated", Severity.WARN,
            f"{s.name} is marked as estimated — the numbers are placeholders, "
            f"not datasheet values, and every budget below inherits that.",
            subsystems=who))
    if not s.source:
        out.append(Finding(
            "doc-no-source", Severity.WARN,
            f"{s.name} has no part number or datasheet link. A specification "
            f"with no source cannot be checked by the next person, and the "
            f"next person is you in October.",
            subsystems=who))
    if not s.owner:
        out.append(Finding(
            "doc-no-owner", Severity.WARN,
            f"{s.name} has no owner. Unowned channels are the ones that arrive "
            f"unmounted.", subsystems=who))
    return out


def integration_findings(s: SensorSpec) -> list[Finding]:
    """Isolation, redundancy against the existing bus, and mounting ownership."""
    out: list[Finding] = []
    who = s.affected_subteams()

    # ---- tractive-system isolation ----------------------------------------- #
    if s.on_tractive_system():
        if s.galvanic_isolation is True:
            out.append(Finding(
                "isolation-ok", Severity.OK,
                f"{s.name} is on the tractive-system side and declares galvanic "
                f"isolation.", subsystems=who))
        else:
            sev = Severity.FAIL if s.galvanic_isolation is False else Severity.MISSING
            out.append(Finding(
                "isolation-required", Severity(sev),
                f"{s.name} mounts on the {s.location}, which is on the "
                f"tractive-system side of the isolation boundary. Its signal "
                f"path into the grounded low-voltage system needs a galvanic "
                f"barrier. This is not an EMC preference — an un-isolated "
                f"sensor wire is a conductive path across the boundary, and it "
                f"will be found at scrutineering if it is not found earlier.",
                subsystems=who + ["electrics"],
                detail={"location": s.location}))

    # ---- already on the bus -------------------------------------------------- #
    if s.available_on_existing_bus:
        out.append(Finding(
            "already-on-bus", Severity.WARN,
            f"{s.name} duplicates a value the {s.available_on_existing_bus} "
            f"already broadcasts. Adding a physical sensor buys an independent "
            f"reading, and costs a channel, a rail, a connector, a mount and a "
            f"calibration. If the reason is redundancy or distrust of the "
            f"device's own sensor, write that down as the justification — "
            f"otherwise read the existing message and spend the effort "
            f"elsewhere.",
            subsystems=who,
            detail={"device": s.available_on_existing_bus}))

    # ---- mounting ownership -------------------------------------------------- #
    if s.location and s.location not in LOCATION_SUBTEAMS:
        out.append(Finding(
            "location-unknown", Severity.WARN,
            f"{s.name} declares location '{s.location}', which maps to no known "
            f"subteam, so nobody has been told they own the bracket.",
            subsystems=["dataacq"]))
    elif s.location:
        others = [t for t in who if t not in ALWAYS_AFFECTED]
        if others:
            out.append(Finding(
                "cross-subteam", Severity.INFO,
                f"{s.name} on the {s.location} lands on: {', '.join(others)}. "
                f"They own the mount, the clearance and the harness route "
                f"through their volume.",
                subsystems=who, detail={"subteams": others}))
    return out


# ===================================================================== #
#  8.  THE PLAN
# ===================================================================== #
def sensor_to_can(s: SensorSpec, can_id: int, *,
                  extended: bool = False) -> Optional[CanMessage]:
    """The CAN message a sampled channel would produce, if it needs one.

    Bus-native sensors already have a message; they are not re-transmitted.
    """
    if s.sample_rate_hz is None:
        return None
    if s.output in BUS_TYPES:
        return None
    payload = s.payload_bytes if s.payload_bytes is not None else 2
    return CanMessage(name=s.key, can_id=can_id, dlc=min(8, max(1, payload)),
                      rate_hz=s.sample_rate_hz, extended=extended,
                      producer=s.owner or "daq", signals=[s.key])


class Verdict(str, Enum):
    READY = "ready"                 # every question answered, every budget clears
    INCOMPLETE = "incomplete"       # arithmetic clears, documentation does not
    BLOCKED = "blocked"             # a hard failure in the plan


@dataclass
class DaqPlan:
    sensors: list
    bus: BusSpec
    bus_result: Optional[BusLoadResult]
    power: Optional[PowerResult]
    storage: Optional[StorageResult]
    bridge: Optional[BridgePlan]
    findings: list
    verdict: Verdict
    completeness: float
    open_questions: dict            # sensor key -> [unanswered question labels]
    delta_t: Optional[DeltaTResult] = None

    # ------------------------------------------------------------------ #
    def by_severity(self, sev: Severity) -> list:
        return [f for f in self.findings if f.severity == sev]

    def blocking(self) -> list:
        return self.by_severity(Severity.FAIL)

    def subteam_actions(self) -> dict:
        """Findings routed to whoever has to do something about them.

        This is the "what other subteam does this affect?" answer, inverted:
        instead of a per-sensor note nobody reads, a per-subteam list of the
        things that are theirs.
        """
        out: dict[str, list] = {}
        for f in self.findings:
            if f.severity in (Severity.OK, Severity.INFO):
                continue
            for t in (f.subsystems or ["dataacq"]):
                out.setdefault(t, []).append(f)
        return out

    # ------------------------------------------------------------------ #
    def to_markdown(self) -> str:
        """The documentation table the review asks for, generated.

        The point is not that a table is hard to write. The point is that a
        hand-written one is a snapshot that starts drifting from the plan the
        moment either changes, and this one cannot drift because it IS the
        plan. Blanks render as an explicit marker rather than an empty cell,
        so an unanswered question looks unanswered.
        """
        def cell(v, suffix=""):
            if v is None or v == "":
                return "**—**"
            if isinstance(v, OutputType):
                return v.value
            if isinstance(v, float):
                return f"{v:g}{suffix}"
            if isinstance(v, bool):
                return "yes" if v else "no"
            return f"{v}{suffix}"

        lines = ["# DAQ channel plan", ""]
        lines.append(f"**Verdict:** {self.verdict.value.upper()} — "
                     f"documentation {self.completeness*100:.0f}% complete, "
                     f"{len(self.blocking())} blocking finding(s).")
        lines.append("")
        lines.append("A dash means the question has not been answered yet. "
                     "It does not mean zero, and it does not mean not "
                     "applicable.")
        lines.append("")
        hdr = ("| Sensor | Measures | Why | Mounted | Output | Supply | "
               "Current | Connector | Rate | Bandwidth | Logged | "
               "Calibration | Affects |")
        lines.append(hdr)
        lines.append("|" + "---|" * 13)
        for s in self.sensors:
            lines.append(
                f"| {s.name} | {cell(s.measures)} | {cell(s.why)} | "
                f"{cell(s.location)} | {cell(s.output)} | {cell(s.supply_v, ' V')} | "
                f"{cell(s.current_ma, ' mA')} | {cell(s.connector)} | "
                f"{cell(s.sample_rate_hz, ' Hz')} | "
                f"{cell(s.signal_bandwidth_hz, ' Hz')} | {cell(s.logged_to)} | "
                f"{cell(s.calibration)} | {', '.join(s.affected_subteams())} |")
        lines.append("")

        if self.bus_result is not None:
            br = self.bus_result
            lines.append("## Bus budget")
            lines.append("")
            floor = " (a FLOOR — undeclared channels can only add)" if br.is_floor else ""
            lines.append(f"{br.bus.name} at {br.bus.bitrate_bps/1000:.0f} kbit/s: "
                         f"**{br.load*100:.1f}% worst-case load**{floor} across "
                         f"{br.messages} messages.")
            lines.append("")

        if self.open_questions:
            lines.append("## Open questions")
            lines.append("")
            for k, qs in self.open_questions.items():
                lines.append(f"* **{k}** — " + "; ".join(qs))
            lines.append("")

        acts = self.subteam_actions()
        if acts:
            lines.append("## Routed to")
            lines.append("")
            for team in sorted(acts):
                lines.append(f"### {team}")
                for f in acts[team]:
                    lines.append(f"* `{f.severity.value}` {f.message}")
                lines.append("")
        return "\n".join(lines)

    def to_csv(self) -> str:
        buf = io.StringIO()
        w = csv.writer(buf)
        cols = ["key", "name", "measures", "unit", "why", "location", "output",
                "supply_v", "current_ma", "supply_rail", "connector",
                "conductors", "signal_bandwidth_hz", "sample_rate_hz",
                "antialias_cutoff_hz", "adc_bits", "range_min_eu",
                "range_max_eu", "accuracy_eu", "resolution_needed_eu",
                "logged_to", "payload_bytes", "calibration",
                "galvanic_isolation", "owner", "source", "is_estimate",
                "completeness", "affects"]
        w.writerow(cols)
        for s in self.sensors:
            d = s.as_dict()
            d["completeness"] = round(s.completeness(), 3)
            d["affects"] = " ".join(s.affected_subteams())
            w.writerow([d.get(c, "") if d.get(c) is not None else "" for c in cols])
        return buf.getvalue()


def find_coolant_pair(sensors: list[SensorSpec]
                      ) -> Optional[tuple[SensorSpec, SensorSpec]]:
    """Locate an inlet/outlet coolant temperature pair, if one is specified.

    A pair of temperature sensors in the coolant loop is almost never two
    independent measurements — it is one delta-T measurement wearing a
    disguise, and it has to be checked as such. Detecting it automatically
    means nobody has to remember to ask.
    """
    loop = [s for s in sensors
            if s.location == "coolant_loop"
            and (s.unit or "").lower().startswith("degc")]
    if len(loop) < 2:
        return None
    inlet = next((s for s in loop if "in" in s.key.lower()), None)
    outlet = next((s for s in loop if "out" in s.key.lower()), None)
    if inlet is not None and outlet is not None and inlet is not outlet:
        return inlet, outlet
    return loop[0], loop[1]


def plan(sensors: list[SensorSpec], *,
         bus: Optional[BusSpec] = None,
         rails: Optional[dict] = None,
         logger: Optional[LoggerSpec] = None,
         bridge: Optional[BridgePlan] = None,
         base_can_id: int = 0x400,
         extra_messages: Optional[list] = None,
         expected_delta_t_k: Optional[float] = None,
         flow_lpm: Optional[float] = None,
         matched_pair: bool = False) -> DaqPlan:
    """Run every check and return the plan with an honest verdict.

    The verdict rules, in order:
      * any FAIL anywhere  -> BLOCKED. Something in here does not work.
      * any unanswered question or MISSING finding -> INCOMPLETE, and the
        budgets are labelled as floors.
      * otherwise -> READY.

    READY is deliberately hard to reach. A plan that reaches it has a declared
    answer to every review question for every channel, clears Nyquist, fits the
    bus, fits the rails, fits the card, and has isolation declared wherever the
    boundary is crossed. That is the bar for "we can start wiring", and it is
    the bar the checklist was always implying without enforcing.
    """
    bus = bus or BusSpec()
    findings: list[Finding] = []
    open_q: dict[str, list[str]] = {}

    for s in sensors:
        findings.extend(documentation_findings(s))
        findings.extend(signal_chain_findings(s))
        findings.extend(integration_findings(s))
        oq = s.unanswered()
        if oq:
            open_q[s.key] = oq

    # ---- assemble the bus ---------------------------------------------- #
    msgs: list[CanMessage] = []
    nid = base_can_id
    for s in sensors:
        m = sensor_to_can(s, nid, extended=bus.extended_ids)
        if m is not None:
            msgs.append(m)
            nid += 1
    if bridge is not None and not bridge.refused:
        msgs.extend(bridge.messages)
        findings.extend(bridge.findings)
    elif bridge is not None:
        findings.extend(bridge.findings)
    if extra_messages:
        msgs.extend(extra_messages)

    bus_result = bus_load(msgs, bus) if msgs else None
    if bus_result is not None:
        undeclared_rate = sum(1 for s in sensors
                              if s.sample_rate_hz is None
                              and s.output not in BUS_TYPES)
        bus_result.is_floor = undeclared_rate > 0
        findings.extend(bus_result.findings)
        if bus_result.is_floor:
            findings.append(Finding(
                "bus-budget-floor", Severity.MISSING,
                f"{undeclared_rate} channel(s) have no declared sample rate and "
                f"contribute nothing to the {bus_result.load*100:.0f}% figure "
                f"above. Treat it as a floor: the real load is higher by "
                f"however much those channels turn out to need.",
                subsystems=["dataacq"]))

    power = power_budget(sensors, rails) if rails else None
    if power is not None:
        findings.extend(power.findings)

    storage = storage_budget(sensors, logger) if logger else None
    if storage is not None:
        findings.extend(storage.findings)

    # ---- coolant delta-T, if a pair is specified ------------------------ #
    dt = None
    pair = find_coolant_pair(sensors)
    if pair is not None:
        dt = delta_t_budget(pair[0], pair[1],
                            expected_delta_t_k=(expected_delta_t_k or 8.0),
                            flow_lpm=flow_lpm, matched_pair=matched_pair)
        findings.extend(dt.findings)

    # ---- completeness --------------------------------------------------- #
    completeness = (sum(s.completeness() for s in sensors) / len(sensors)
                    if sensors else 0.0)

    has_fail = any(f.severity == Severity.FAIL for f in findings)
    has_missing = any(f.severity == Severity.MISSING for f in findings)
    if has_fail:
        verdict = Verdict.BLOCKED
    elif has_missing or completeness < 1.0:
        verdict = Verdict.INCOMPLETE
    else:
        verdict = Verdict.READY

    return DaqPlan(sensors=list(sensors), bus=bus, bus_result=bus_result,
                   power=power, storage=storage, bridge=bridge,
                   findings=findings, verdict=verdict,
                   completeness=completeness, open_questions=open_q,
                   delta_t=dt)


# ===================================================================== #
#  9.  CATALOG — the sensors a meeting names, pre-specified
# ===================================================================== #
#  Every entry here carries datasheet-grade defaults for the questions that
#  have a defensible generic answer (bandwidth, output type, typical accuracy),
#  and leaves None on the ones that genuinely depend on the part you buy and
#  the car you bolt it to. The Nones are the point: they are the questions the
#  catalog cannot answer for you, marked as such rather than filled with a
#  plausible number.
# ===================================================================== #
def _c(**kw) -> SensorSpec:
    return SensorSpec(**kw)


CATALOG: dict[str, SensorSpec] = {
    "motor_temp": _c(
        key="motor_temp", name="Motor stator temperature",
        measures="stator winding temperature", unit="degC",
        why="derate before the magnets are damaged; the thermal limit is what "
            "actually caps a long run, not the peak torque number",
        location="motor", output=OutputType.RTD,
        supply_v=5.0, current_ma=5.0, supply_rail="5V",
        connector=None, conductors=3,
        signal_bandwidth_hz=0.5, sample_rate_hz=10.0, antialias_cutoff_hz=2.0,
        adc_bits=12, range_min_eu=-20.0, range_max_eu=200.0,
        accuracy_eu=1.0, resolution_needed_eu=1.0,
        logged_to="logger", payload_bytes=2,
        calibration="two-point in a stirred oil bath against a reference probe "
                    "at 25 and 100 degC; record the residual",
        galvanic_isolation=None,
        source="", owner="",
        notes="Often already embedded in the motor and reported by the "
              "inverter — check before adding a second one."),

    "inverter_temp": _c(
        key="inverter_temp", name="Inverter / IGBT temperature",
        measures="power stage temperature", unit="degC",
        why="the inverter derates on this; logging it explains torque loss "
            "that otherwise looks like a driver or a grip problem",
        location="inverter", output=OutputType.CAN,
        supply_v=None, current_ma=None, supply_rail=None,
        connector="existing CAN", conductors=0,
        signal_bandwidth_hz=1.0, sample_rate_hz=10.0,
        range_min_eu=-40.0, range_max_eu=150.0, accuracy_eu=2.0,
        resolution_needed_eu=1.0,
        logged_to="logger", payload_bytes=2,
        calibration="cross-check the broadcast value against a surface probe "
                    "on the heatsink during a cool-down",
        available_on_existing_bus="inverter",
        source="", owner="",
        notes="Nearly every FSAE-class inverter already broadcasts device "
              "temperature. Read the message before specifying a sensor."),

    "coolant_temp_in": _c(
        key="coolant_temp_in", name="Coolant temperature — radiator inlet",
        measures="coolant temperature entering the radiator", unit="degC",
        why="with the outlet probe and the flow meter, gives measured heat "
            "rejection instead of a spreadsheet estimate",
        location="coolant_loop", output=OutputType.ANALOG_V,
        supply_v=5.0, current_ma=2.0, supply_rail="5V",
        connector=None, conductors=2,
        signal_bandwidth_hz=0.2, sample_rate_hz=5.0, antialias_cutoff_hz=1.0,
        adc_bits=12, range_min_eu=-10.0, range_max_eu=120.0,
        accuracy_eu=2.0, resolution_needed_eu=0.5,
        logged_to="logger", payload_bytes=2,
        calibration="MATCHED PAIR with the outlet probe against one reference "
                    "in a stirred bath — the pair matters far more than either "
                    "sensor's absolute accuracy",
        source="", owner=""),

    "coolant_temp_out": _c(
        key="coolant_temp_out", name="Coolant temperature — radiator outlet",
        measures="coolant temperature leaving the radiator", unit="degC",
        why="the other half of the delta-T that sizes the radiator",
        location="coolant_loop", output=OutputType.ANALOG_V,
        supply_v=5.0, current_ma=2.0, supply_rail="5V",
        connector=None, conductors=2,
        signal_bandwidth_hz=0.2, sample_rate_hz=5.0, antialias_cutoff_hz=1.0,
        adc_bits=12, range_min_eu=-10.0, range_max_eu=120.0,
        accuracy_eu=2.0, resolution_needed_eu=0.5,
        logged_to="logger", payload_bytes=2,
        calibration="MATCHED PAIR with the inlet probe — calibrate both against "
                    "the same reference at the same time",
        source="", owner=""),

    "coolant_flow": _c(
        key="coolant_flow", name="Coolant flow rate",
        measures="volumetric coolant flow", unit="L/min",
        why="turns the delta-T into kilowatts; without it the temperature pair "
            "measures a difference of unknown significance",
        location="coolant_loop", output=OutputType.PULSE,
        supply_v=5.0, current_ma=10.0, supply_rail="5V",
        connector=None, conductors=3,
        signal_bandwidth_hz=2.0, sample_rate_hz=20.0,
        range_min_eu=0.0, range_max_eu=30.0, accuracy_eu=0.9,
        resolution_needed_eu=0.5,
        logged_to="logger", payload_bytes=2,
        calibration="catch-and-weigh: run into a measured vessel for a timed "
                    "interval at three pump speeds and fit the K-factor",
        source="", owner="",
        notes="Pulse output — counted over a gate window, not sampled. "
              "Resolution comes from the gate time and the K-factor. Check the "
              "pressure drop it adds to the loop before committing."),
}


def catalog(key: str, **overrides) -> SensorSpec:
    """A catalog entry with your own numbers substituted in.

    Overriding is expected: the catalog is a starting point that answers the
    generic questions, not a substitute for the part you actually bought.
    """
    if key not in CATALOG:
        raise KeyError(f"unknown sensor '{key}'. Known: {sorted(CATALOG)}")
    base = CATALOG[key]
    d = {f: getattr(base, f) for f in base.__dataclass_fields__}
    d.update(overrides)
    return SensorSpec(**d)


#: A ready-made starting set for the four sensors a cooling/powertrain
#: instrumentation discussion typically lands on.
def cooling_package() -> list[SensorSpec]:
    return [catalog(k) for k in
            ("motor_temp", "inverter_temp", "coolant_temp_in",
             "coolant_temp_out", "coolant_flow")]


#: Default rails for a typical GLV sensor supply.
def default_rails() -> dict:
    return {
        "5V": Rail("5V", 5.0, capacity_ma=500.0, fuse_a=1.0),
        "12V": Rail("12V", 12.0, capacity_ma=2000.0, fuse_a=5.0),
    }


# ===================================================================== #
#  10. PROVENANCE
# ===================================================================== #
PROVENANCE = {
    "physics_grounded": [
        "CAN frame length from the ISO 11898-1 field layout including the "
        "worst-case bit-stuffing bound (135 bits for an 8-byte standard frame, "
        "160 for extended)",
        "worst-case message latency by fixed-point response-time analysis for "
        "non-preemptive fixed-priority arbitration",
        "Nyquist criterion applied per channel against declared bandwidth",
        "ADC quantisation as span / (2^bits - 1) in engineering units",
        "delta-T uncertainty by uncorrelated error propagation; heat rejection "
        "as rho * V_dot * cp * dT",
        "UART link budget from the byte framing (start + data + parity + stop)",
    ],
    "arithmetic": [
        "bus load, rail current, storage per session — exact sums over "
        "declared values",
    ],
    "estimate_flagged": [
        "any sensor marked is_estimate",
        "catalog defaults, which are generic values and not the part you bought",
        "latency analysis assumes strictly periodic transmission and no "
        "queueing jitter; real controllers with FIFO buffers do worse",
    ],
    "hard_rule": (
        "A plan with unanswered review questions never returns READY, and its "
        "bus, power and storage figures are reported as FLOORS. An undeclared "
        "channel contributes zero to the arithmetic and a positive amount to "
        "the real bus — so a green load figure over a half-specified channel "
        "list is not an optimistic estimate, it is a wrong one. Likewise the "
        "BMS bridge refuses to emit a frame map from an empty signal list "
        "rather than inventing a plausible layout."
    ),
}
