# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/bq796xx.py — the TI BQ796xx / BQ756xx battery-monitor stack, as
#  an executable protocol and timing model rather than a page of notes. Builds
#  real command frames with a CRC pinned to the documented examples, computes
#  what the daisy chain costs in time, and hands the result to daq_plan's
#  UART->CAN bridge so the bridge no longer has to refuse.
# ============================================================================
"""
BQ796xx — the BMS link, with the arithmetic done.

WHY THIS MODULE EXISTS
----------------------
`daq_plan.plan_bms_bridge` refuses to design a bridge from an empty signal
list, on the grounds that the frame layout is a datasheet question and not a
wiring question. That refusal is correct, and it is also a standing invoice.
This module pays it: it is the datasheet, read, for the specific part family
the accumulator is being built around.

Three things here are worth more than the notes they came from.

**1. The CRC seed is not what the prose says it is.**
The EVM documentation describes the frame check as "CRC-16-IBM polynomial, 2
bytes appended to every frame". Implement that literally — CRC-16/IBM is
CRC-16/ARC, seeded 0x0000 — and every one of the twelve worked example frames
printed in the same document fails its own checksum. Seeded 0xFFFF, all twelve
pass. The polynomial is right (0x8005, reflected to 0xA001); the seed is not
stated, and the value people assume from the name is wrong.

This is the single most expensive bit in the module. A wrong CRC seed does not
produce a diagnosable error. It produces silence: every frame is transmitted
correctly, arrives correctly, and is discarded by the device as corrupt. The
symptom is a stack that never answers, which reads exactly like a wiring fault,
a wake-sequence fault, or a dead board, and sends people to the oscilloscope
for two days. `CRC_ANCHORS` pins the implementation to the published frames so
this cannot regress quietly.

**2. Polling a daisy chain has a speed limit, and it is lower than people
assume.** "Poll at 10-100 Hz depending on what the DAQ needs" is a sentence
that sounds like a configuration choice and is actually a claim about UART
byte time multiplied by board count. Sixteen cells on one board at 115200 baud
polls comfortably at 100 Hz. The same code on a six-board stack cannot reach
40 Hz, because the response payload grew sixfold and the baud rate did not.
`poll_budget` computes the ceiling and names which of the two numbers has to
move.

**3. NFAULT is a wire, and the poll loop is not.** The device asserts a
hardware fault pin. Reading fault status inside the poll loop finds out about
an over-voltage one poll period late, which at 10 Hz is 100 ms of a cell being
over-voltage before the firmware has an opinion. The pin is already routed to
the host header; treating it as an interrupt costs one GPIO and one ISR.

WHAT IS DOCUMENTED FACT vs WHAT IS ASSUMED
------------------------------------------
Everything in `CRC_ANCHORS`, `INIT_BYTES`, the timing constants, the register
addresses and `CELL_LSB_V` comes from the device documentation and is testable
against it. `RESPONSE_OVERHEAD_BYTES` is the one number this module assumes:
the response frame layout lives in the BQ79616-Q1 *Software Design Reference*,
which the EVM guide references and does not include. It is declared as a named
constant with a stated default rather than buried in an expression, so that
when someone finally pulls that document the correction is one line and every
derived figure moves with it. `PROVENANCE` says so out loud.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from .interfaces import Severity, Finding
from .daq_plan import (
    BmsSignal, CanMessage, UartLink, BusSpec,
    SensorSpec, OutputType,
)


# ===================================================================== #
#  0.  FRAME CHECK — the CRC, pinned to the published examples
# ===================================================================== #
#: Reflected form of the 0x8005 polynomial the documentation names.
_CRC_POLY_REFLECTED = 0xA001

#: The seed the documentation does NOT name. See module docstring: the prose
#: says "CRC-16-IBM", whose textbook seed is 0x0000, and that seed rejects
#: every worked example in the same document. 0xFFFF accepts all of them.
CRC_INIT = 0xFFFF

#: Every complete example frame published by TI, used as regression anchors
#: for `crc16`. If a future edit breaks the CRC, these fail before any hardware
#: does — which is the entire point of writing them down, because on hardware
#: this failure mode is indistinguishable from a dead bus.
#:
#: Sources: the EVM documentation, and TI SLVAE86B (BQ79616-Q1 Software Design
#: Reference, Rev. B, August 2023) sections 4.2, 5.2, 6.2 and 9.2.
#:
#: Note on TI's own tables: the CRC column in the Table 1-2..1-7 command-frame
#: templates is rendered inconsistently — some entries print the value in
#: transmitted byte order and some in integer order. The complete worked
#: byte strings below are unambiguous, so those are what is pinned here.
CRC_ANCHORS: tuple[str, ...] = (
    # --- auto-addressing, SLVAE86B section 4.2 ------------------------------
    "D0 03 4C 00 FC 24",        # dummy broadcast write OTP_ECC_TEST, sync DLL
    "D0 03 09 01 0F 74",        # CONTROL1 = 0x01, enable auto-address mode
    "D0 03 06 00 CB 44",        # DIR0_ADDR of device 0
    "D0 03 06 01 0A 84",        # device 1
    "D0 03 06 02 4A 85",        # device 2
    "D0 03 08 02 4E E5",        # everyone a stack device, provisionally
    "90 00 03 08 00 13 DD",     # device 0 -> base      (COMM_CTRL = 0x00)
    "90 02 03 08 03 52 64",     # device 2 -> top of stack (COMM_CTRL = 0x03)
    "C0 03 4C 00 F8 E4",        # dummy broadcast read, re-sync the DLL
    # --- cell voltages, SLVAE86B section 5.2 --------------------------------
    "D0 00 03 0A B8 13",        # ACTIVE_CELL = 0x0A, all 16 cells active
    "D0 03 0D 06 4C 76",        # ADC_CTRL1 = 0x06, continuous run, start ADC
    "C0 05 68 1F 42 2D",        # broadcast read, 32 bytes of cell voltages
    # --- cell balancing, SLVAE86B section 6.2 -------------------------------
    # These two are the only published multi-byte writes, and they are what
    # proves the init byte encodes the payload length: 0xD7 == 0xD0 | (8-1).
    "D7 03 18 02 02 02 02 02 02 02 02 14 BE",   # CB_CELL8..1_CTRL timers
    "D7 03 20 02 02 02 02 02 02 02 02 27 7F",   # CB_CELL16..9_CTRL timers
    "D0 03 2E 01 14 84",        # BAL_CTRL1 duty cycle
    "D0 03 2A 08 D6 42",        # VCB_DONE_THRESH
    "D0 03 2C 05 14 27",        # OVUV_CTRL, round robin
    "D0 03 2F 03 94 D5",        # BAL_CTRL2 = 0x03, start auto balancing
    # --- reverse addressing, SLVAE86B section 9.2 ---------------------------
    "90 00 03 09 80 13 ED",     # CONTROL1 DIR_SEL=1 on the base device
    "E0 03 09 80 C0 14",        # broadcast write REVERSE, the only legal use
    "D0 03 09 81 0E D4",        # auto-address mode, keeping reverse direction
    "D0 03 07 00 CA D4",        # DIR1_ADDR device 0
    "D0 03 07 01 0B 14",        # DIR1_ADDR device 1
    "D0 03 07 02 4B 15",        # DIR1_ADDR device 2
)


def crc16(data: bytes) -> int:
    """Frame CRC over `data`, as the device actually computes it.

    Reflected 0x8005 seeded 0xFFFF, no final XOR. Returned as an integer; use
    `crc_bytes` for the on-wire order, which is LOW BYTE FIRST.
    """
    crc = CRC_INIT
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ _CRC_POLY_REFLECTED if crc & 1 else crc >> 1
    return crc & 0xFFFF


def crc_bytes(data: bytes) -> bytes:
    """The two CRC bytes in transmission order (little-endian)."""
    c = crc16(data)
    return bytes((c & 0xFF, (c >> 8) & 0xFF))


def append_crc(body: bytes) -> bytes:
    """A frame body with its CRC appended — i.e. the bytes you transmit."""
    return bytes(body) + crc_bytes(bytes(body))


def check_crc(frame: bytes) -> bool:
    """True if a received frame's trailing CRC matches its body."""
    if len(frame) < 3:
        return False
    return crc_bytes(bytes(frame[:-2])) == bytes(frame[-2:])


def verify_crc_anchors() -> list[tuple[str, bool]]:
    """Re-check every documented example frame. All must pass."""
    out = []
    for s in CRC_ANCHORS:
        out.append((s, check_crc(bytes.fromhex(s))))
    return out


# ===================================================================== #
#  1.  COMMAND FRAMES
# ===================================================================== #
class Cmd(str, Enum):
    """Frame types, keyed to the documented init bytes."""
    SINGLE_READ = "single_read"
    SINGLE_WRITE = "single_write"
    STACK_READ = "stack_read"
    STACK_WRITE = "stack_write"
    BROADCAST_READ = "broadcast_read"
    BROADCAST_WRITE = "broadcast_write"
    BROADCAST_WRITE_REVERSE = "broadcast_write_reverse"


#: Init byte per frame type, from the documentation's table.
INIT_BYTES: dict[Cmd, int] = {
    Cmd.SINGLE_READ:             0x80,
    Cmd.SINGLE_WRITE:            0x90,
    Cmd.STACK_READ:              0xA0,
    Cmd.STACK_WRITE:             0xB0,
    Cmd.BROADCAST_READ:          0xC0,
    Cmd.BROADCAST_WRITE:         0xD0,
    Cmd.BROADCAST_WRITE_REVERSE: 0xE0,
}

#: Frame types that carry a device address byte after the init byte.
_ADDRESSED = frozenset({Cmd.SINGLE_READ, Cmd.SINGLE_WRITE})

#: Write payload ceiling, from the documentation.
MAX_WRITE_BYTES = 8

#: Read ceiling. The length byte is (bytes requested - 1), so 128 bytes is the
#: largest legal request and 0x7F the largest legal length byte.
MAX_READ_BYTES = 128


def build_write(reg: int, data: bytes | int, *,
                cmd: Cmd = Cmd.BROADCAST_WRITE,
                device: Optional[int] = None) -> bytes:
    """A complete write frame: [init][addr?][reg hi][reg lo][data...][crc].

    THE INIT BYTE ENCODES THE PAYLOAD LENGTH. The low nibble is
    (number of data bytes - 1), so a broadcast write of one byte is 0xD0 and of
    eight bytes is 0xD7. The base values in `INIT_BYTES` are the one-byte forms,
    which is why every single-byte example in the documentation shows a round
    number and hides the encoding completely.

    This matters because the balancing sequence is the first thing anyone
    writes that needs a multi-byte payload. Sending 0xD0 with eight data bytes
    is a well-formed frame with a valid CRC that asks the device to do
    something different from what was intended, so it fails quietly rather than
    erroring — the balancing timers simply do not take.

    >>> build_write(0x034C, 0x00).hex(' ').upper()
    'D0 03 4C 00 FC 24'
    >>> build_write(0x0318, b'\\x02' * 8).hex(' ').upper()
    'D7 03 18 02 02 02 02 02 02 02 02 14 BE'
    """
    if isinstance(data, int):
        data = bytes((data & 0xFF,))
    data = bytes(data)
    if not 1 <= len(data) <= MAX_WRITE_BYTES:
        raise ValueError(f"write payload is {len(data)} bytes; the device "
                         f"accepts 1..{MAX_WRITE_BYTES}")
    if cmd in _ADDRESSED and device is None:
        raise ValueError(f"{cmd.value} needs a device address")
    body = bytearray((INIT_BYTES[cmd] | (len(data) - 1),))
    if cmd in _ADDRESSED:
        body.append(device & 0xFF)
    body += bytes(((reg >> 8) & 0xFF, reg & 0xFF))
    body += data
    return append_crc(bytes(body))


def build_read(reg: int, n_bytes: int, *,
               cmd: Cmd = Cmd.BROADCAST_READ,
               device: Optional[int] = None) -> bytes:
    """A complete read frame. `n_bytes` is what you want; the wire carries n-1.

    The off-by-one is the device's, not ours, and it is the kind of detail that
    silently returns one byte short of a register block.

    >>> build_read(0x0568, 32).hex(' ').upper()
    'C0 05 68 1F 42 2D'
    """
    if not 1 <= n_bytes <= MAX_READ_BYTES:
        raise ValueError(f"read of {n_bytes} bytes; the device allows "
                         f"1..{MAX_READ_BYTES}")
    if cmd in _ADDRESSED and device is None:
        raise ValueError(f"{cmd.value} needs a device address")
    body = bytearray((INIT_BYTES[cmd],))
    if cmd in _ADDRESSED:
        body.append(device & 0xFF)
    body += bytes(((reg >> 8) & 0xFF, reg & 0xFF, n_bytes - 1))
    return append_crc(bytes(body))


# ===================================================================== #
#  2.  REGISTERS  (the subset the bring-up sequence touches)
# ===================================================================== #
REG_ACTIVE_CELL = 0x0003
REG_DIR0_ADDR = 0x0306
REG_DIR1_ADDR = 0x0307         # reverse-direction address
REG_COMM_CTRL = 0x0308
REG_CONTROL1 = 0x0309
REG_ADC_CTRL1 = 0x030D
REG_CB_CELL8_CTRL = 0x0318     # CB_CELL8..1_CTRL, eight timers, descending
REG_CB_CELL16_CTRL = 0x0320    # CB_CELL16..9_CTRL
REG_VCB_DONE_THRESH = 0x032A
REG_OVUV_CTRL = 0x032C
REG_BAL_CTRL1 = 0x032E
REG_BAL_CTRL2 = 0x032F
REG_OTP_ECC_TEST = 0x034C      # the dummy target used to sync the DLL
REG_VCELL_BLOCK = 0x0568       # VCELL16_HI .. VCELL1_LO, descending

#: Kept as an alias: the EVM notes call this "the dummy write to sync the DLL"
#: without naming the register. It is OTP_ECC_TEST, and it is chosen precisely
#: because writing zero to it does nothing.
REG_DLL_SYNC = REG_OTP_ECC_TEST

#: COMM_CTRL values, from SLVAE86B section 4.1.
COMM_CTRL_BASE = 0x00          # base device in a multi-board stack
COMM_CTRL_BASE_AND_TOP = 0x01  # single board: base AND top of stack
COMM_CTRL_STACK = 0x02         # a middle device
COMM_CTRL_TOP = 0x03           # top of a multi-board stack

#: Cell-voltage registers are 16-bit, two per cell, highest cell first.
BYTES_PER_CELL = 2

#: Cell voltage scale factor. Two's-complement 16-bit count * this = volts.
CELL_LSB_V = 190.73e-6


def raw_to_volts(raw: int) -> float:
    """One 16-bit two's-complement cell-voltage register to volts."""
    if raw & 0x8000:
        raw -= 0x10000
    return raw * CELL_LSB_V


def volts_to_raw(v: float) -> int:
    """Inverse of `raw_to_volts`, for building test vectors."""
    n = int(round(v / CELL_LSB_V))
    return n & 0xFFFF


def decode_cell_block(payload: bytes, cells: int) -> list[float]:
    """Decode a cell-voltage payload into volts, lowest cell index first.

    The block reads highest cell first, so this reverses it — because every
    plot, every balancing decision and every human conversation numbers cells
    from one upward, and a silent index reversal is a bug that looks like a
    wiring error at the connector.
    """
    need = cells * BYTES_PER_CELL
    if len(payload) < need:
        raise ValueError(f"payload has {len(payload)} bytes, need {need} "
                         f"for {cells} cells")
    vals = [raw_to_volts((payload[i] << 8) | payload[i + 1])
            for i in range(0, need, BYTES_PER_CELL)]
    return list(reversed(vals))


# ===================================================================== #
#  3.  TIMING  — the numbers that set the poll ceiling
# ===================================================================== #
#: Active-low wake pulse held on the RX line.
WAKE_PULSE_S = 2.5e-3

#: After the pulse, the chain needs this long before it will answer anything:
#: a fixed part plus a per-device part as the wake tone propagates up the
#: daisy chain. Polling before this elapses gets silence, which is routinely
#: misread as a dead board.
WAKE_SETTLE_BASE_S = 10.0e-3
WAKE_SETTLE_PER_DEVICE_S = 600e-6

#: Round-robin ADC settle before cell results are valid.
ADC_SETTLE_BASE_S = 192e-6
ADC_SETTLE_PER_BOARD_S = 5e-6

#: NOT PUBLISHED IN ANY DOCUMENT AVAILABLE TO THIS MODULE.
#:
#: The response frame layout is not in the EVM guide and not in SLVAE86B
#: either — the Software Design Reference documents command frames only and
#: defers to the "Command and Response Protocol" section of the BQ79616-Q1 /
#: BQ79614-Q1 / BQ79612-Q1 data sheet for bit-level detail. So rather than
#: assume a number and let it propagate silently into every timing figure,
#: this module treats the per-device response overhead as an UNKNOWN with
#: plausible bounds, and provides a five-minute bench procedure to replace the
#: bounds with a measurement.
#:
#: Lower bound 3: the minimum any framing can be — one init/length byte plus
#: the two CRC bytes that every frame in this protocol carries.
#: Upper bound 8: init byte, device address, two register address bytes, two
#: CRC bytes, plus two bytes of slack for anything undocumented.
RESPONSE_OVERHEAD_BOUNDS: tuple[int, int] = (3, 8)


def measure_response_overhead(received_bytes: int, *, cells_per_board: int,
                              boards: int) -> int:
    """Derive the real per-device overhead from one broadcast read.

    The bench procedure, in full: wake and address the stack, start the ADC,
    send one broadcast read of the cell-voltage block, and count the bytes that
    come back. The payload is known exactly — two bytes per cell per board —
    so everything else is framing.

        overhead = (received - 2 * cells * boards) / boards

    This takes five minutes and permanently removes the largest assumption in
    the module. `BqCanBridge.ino` has a menu entry that performs it and prints
    the answer, so it does not even require writing code.
    """
    payload = cells_per_board * BYTES_PER_CELL * boards
    if received_bytes <= payload:
        raise ValueError(
            f"{received_bytes} bytes received is not more than the "
            f"{payload}-byte payload, so no framing overhead can be derived. "
            f"Either the read was short or the stack is not fully addressed.")
    rem = received_bytes - payload
    if rem % boards:
        raise ValueError(
            f"{rem} framing bytes across {boards} boards does not divide "
            f"evenly, so the response is not a simple repeated frame. Read the "
            f"data sheet's Command and Response Protocol section rather than "
            f"inferring from this.")
    return rem // boards


def wake_time_s(n_devices: int) -> float:
    """Total time from starting the wake pulse to the chain being addressable."""
    if n_devices < 1:
        raise ValueError("a stack has at least one device")
    return (WAKE_PULSE_S + WAKE_SETTLE_BASE_S
            + WAKE_SETTLE_PER_DEVICE_S * n_devices)


def adc_settle_s(boards: int) -> float:
    """Round-robin settle time before a cell measurement is valid."""
    return ADC_SETTLE_BASE_S + ADC_SETTLE_PER_BOARD_S * boards


# ===================================================================== #
#  4.  THE PART FAMILY
# ===================================================================== #
@dataclass(frozen=True)
class BqDevice:
    """One monitor variant, with the properties that change a design."""
    part: str
    min_cells: int
    max_cells: int
    stackable: bool
    max_boards: int
    current_sense: bool          # integrated shunt ADC -> SOC/SOH without a
                                 # separate current sensor and its own channel
    asil: Optional[str] = None
    gpio: int = 8
    notes: str = ""

    def max_cells_total(self) -> int:
        return self.max_cells * self.max_boards


DEVICES: dict[str, BqDevice] = {
    "BQ79616-Q1": BqDevice(
        part="BQ79616-Q1", min_cells=6, max_cells=16,
        stackable=True, max_boards=35, current_sense=False,
        notes="The only stackable variant. Daisy-chains over an isolated "
              "differential bus, so a large pack is one host UART and N boards "
              "rather than N hosts. No integrated current sense — pack current "
              "stays a separate channel with its own connector and rail."),
    "BQ75614-Q1": BqDevice(
        part="BQ75614-Q1", min_cells=14, max_cells=16,
        stackable=False, max_boards=1, current_sense=True,
        notes="Standalone. Integrated current-sense ADC, so pack current is "
              "measured coherently with cell voltage — the two land in the "
              "same sample window, which is what makes a real SOC estimate "
              "possible rather than two loosely-related logs."),
    "BQ79656-Q1": BqDevice(
        part="BQ79656-Q1", min_cells=14, max_cells=16,
        stackable=False, max_boards=1, current_sense=True, asil="D",
        notes="Standalone, with balancer, current sense and ASIL-D diagnostic "
              "coverage. The safety features matter for the argument you make "
              "to scrutineering, not just for the electronics."),
}


@dataclass
class StackSpec:
    """The pack as it is actually wired: which part, how many, how many cells.

    `cells_per_board` defaults to None because it is a decision, not a
    property. A 16-cell-capable board carrying 12 cells is a normal and often
    correct choice, and assuming the maximum would understate nothing and
    overstate the poll time — so it is asked for rather than guessed.
    """
    part: str = "BQ79616-Q1"
    boards: int = 1
    cells_per_board: Optional[int] = None
    #: Cell-balancing intent, only used for findings.
    balancing: Optional[bool] = None
    #: Is the serial link galvanically isolated from the GLV system?
    isolated: Optional[bool] = None
    #: Is NFAULT wired to a host interrupt-capable GPIO?
    nfault_to_interrupt: Optional[bool] = None
    #: Thermistors actually populated on the GPIO channels.
    thermistors_per_board: Optional[int] = None
    #: MEASURED per-device response framing overhead, from
    #: `measure_response_overhead`. None means not measured, and the timing
    #: results are reported as a range rather than a number.
    response_overhead_bytes: Optional[int] = None

    def device(self) -> BqDevice:
        if self.part not in DEVICES:
            raise KeyError(f"unknown part '{self.part}'. "
                           f"Known: {sorted(DEVICES)}")
        return DEVICES[self.part]

    def total_cells(self) -> Optional[int]:
        if self.cells_per_board is None:
            return None
        return self.cells_per_board * self.boards

    def payload_bytes(self) -> Optional[int]:
        """Cell-voltage payload for the whole chain — known exactly."""
        if self.cells_per_board is None:
            return None
        return self.cells_per_board * BYTES_PER_CELL * self.boards

    def response_bounds(self) -> Optional[tuple[int, int]]:
        """(min, max) bytes returned for one broadcast cell-voltage read.

        Collapses to a single value once the overhead has been measured.
        """
        payload = self.payload_bytes()
        if payload is None:
            return None
        if self.response_overhead_bytes is not None:
            n = payload + self.response_overhead_bytes * self.boards
            return (n, n)
        lo, hi = RESPONSE_OVERHEAD_BOUNDS
        return (payload + lo * self.boards, payload + hi * self.boards)

    def response_bytes(self) -> Optional[int]:
        """Response length, but only when it is actually known.

        Returns None while the overhead is unmeasured, rather than a plausible
        number. Callers that need a figure must decide what to do about not
        having one — which is the whole point.
        """
        b = self.response_bounds()
        if b is None or b[0] != b[1]:
            return None
        return b[0]


# ===================================================================== #
#  5.  BRING-UP SEQUENCE, AS FRAMES
# ===================================================================== #
def auto_address_sequence(boards: int, *,
                          reverse: bool = False) -> list[tuple[bytes, str]]:
    """The documented auto-addressing sequence, generated for `boards` boards.

    Returned as (frame, comment) pairs so the same list drives a test, a
    printout and the firmware's command table. Generating it means a six-board
    stack does not require hand-editing a transcript written for three.

    THE SINGLE-BOARD CASE IS DIFFERENT. With one board there is no "base and
    then top" — the one device is both, and it is told so with a single
    COMM_CTRL = 0x01. Running the two-board sequence against a single board
    leaves it configured as a base device with no top of stack, which responds
    to nothing and looks like a wiring fault. This is the shape of bug that
    survives testing on the multi-board bench rig and appears only when someone
    tries a single module.
    """
    reg_addr = REG_DIR1_ADDR if reverse else REG_DIR0_ADDR
    control1 = 0x81 if reverse else 0x01
    seq: list[tuple[bytes, str]] = [
        (build_write(REG_OTP_ECC_TEST, 0x00),
         "dummy broadcast write OTP_ECC_TEST, sync the DLL"),
        (build_write(REG_CONTROL1, control1),
         f"CONTROL1 = 0x{control1:02X}, enable auto-address mode"
         + (" (keeping reverse direction)" if reverse else "")),
    ]
    for i in range(boards):
        seq.append((build_write(reg_addr, i),
                    f"{'DIR1' if reverse else 'DIR0'}_ADDR of device {i}"))
    seq.append((build_write(REG_COMM_CTRL, COMM_CTRL_STACK),
                "provisionally mark every device a stack device"))
    if boards == 1:
        seq.append((build_write(REG_COMM_CTRL, COMM_CTRL_BASE_AND_TOP,
                                cmd=Cmd.SINGLE_WRITE, device=0),
                    "single board: device 0 is base AND top of stack "
                    "(COMM_CTRL = 0x01)"))
    else:
        seq.append((build_write(REG_COMM_CTRL, COMM_CTRL_BASE,
                                cmd=Cmd.SINGLE_WRITE, device=0),
                    "device 0 is the base (COMM_CTRL = 0x00)"))
        seq.append((build_write(REG_COMM_CTRL, COMM_CTRL_TOP,
                                cmd=Cmd.SINGLE_WRITE, device=boards - 1),
                    f"device {boards - 1} is top of stack (COMM_CTRL = 0x03)"))
    seq.append((build_read(REG_OTP_ECC_TEST, 1),
                "dummy broadcast read, re-sync the DLL"))
    return seq


def reverse_direction_sequence(boards: int) -> list[tuple[bytes, str]]:
    """Flip the daisy chain to communicate in the reverse direction.

    Useful for a ring architecture: if one board's upward link fails, the host
    can reach the rest of the stack from the other end instead of losing every
    board above the fault. Worth knowing exists before the pack is wired,
    because it is a topology decision and not a firmware one.

    The broadcast-write-reverse init byte (0xE0) is documented as legal for
    exactly this one purpose. Using it for anything else is not a shortcut.
    """
    seq: list[tuple[bytes, str]] = [
        (build_write(REG_CONTROL1, 0x80, cmd=Cmd.SINGLE_WRITE, device=0),
         "base device: CONTROL1 DIR_SEL = 1"),
        (build_write(REG_CONTROL1, 0x80, cmd=Cmd.BROADCAST_WRITE_REVERSE),
         "broadcast write REVERSE: flip the rest of the stack"),
    ]
    return seq + auto_address_sequence(boards, reverse=True)


def balancing_sequence(cells_per_board: int, *,
                       timer_code: int = 0x02,
                       duty_code: int = 0x01,
                       done_thresh_code: Optional[int] = 0x08
                       ) -> list[tuple[bytes, str]]:
    """Start automatic passive balancing.

    The two CB_CELL*_CTRL writes are eight-byte payloads, which is what makes
    this the first sequence where the init-byte length encoding matters. See
    `build_write`.
    """
    if not 1 <= cells_per_board <= 16:
        raise ValueError("cells_per_board must be 1..16")
    active = 0x0A - (16 - cells_per_board)
    seq = [
        (build_write(REG_ACTIVE_CELL, active),
         f"ACTIVE_CELL = 0x{active:02X}"),
        (build_write(REG_CB_CELL8_CTRL, bytes([timer_code] * 8)),
         "CB_CELL8..1_CTRL balance timers (nonzero = balance that channel)"),
        (build_write(REG_CB_CELL16_CTRL, bytes([timer_code] * 8)),
         "CB_CELL16..9_CTRL balance timers"),
        (build_write(REG_BAL_CTRL1, duty_code),
         "BAL_CTRL1 duty cycle, even/odd cell alternation"),
    ]
    if done_thresh_code is not None:
        seq.append((build_write(REG_VCB_DONE_THRESH, done_thresh_code),
                    "VCB_DONE_THRESH auto-stop voltage"))
        seq.append((build_write(REG_OVUV_CTRL, 0x05),
                    "OVUV_CTRL = 0x05, run OV/UV comparators in round robin"))
    seq.append((build_write(REG_BAL_CTRL2, 0x03),
                "BAL_CTRL2 = 0x03, start auto balancing"))
    return seq


def start_adc_sequence(cells_per_board: int) -> list[tuple[bytes, str]]:
    """Activate the cell channels and start the ADC in continuous run mode."""
    if not 1 <= cells_per_board <= 16:
        raise ValueError("cells_per_board must be 1..16")
    # ACTIVE_CELL encodes the highest active channel; 16 cells -> 0x0A.
    active = 0x0A - (16 - cells_per_board)
    return [
        (build_write(REG_ACTIVE_CELL, active),
         f"ACTIVE_CELL = 0x{active:02X}, activate {cells_per_board} cells"),
        (build_write(REG_ADC_CTRL1, 0x06),
         "ADC_CTRL1 = 0x06, continuous run mode, start ADC"),
    ]


def read_cells_frame(cells_per_board: int) -> bytes:
    """The broadcast read that returns the cell-voltage block."""
    return build_read(REG_VCELL_BLOCK, cells_per_board * BYTES_PER_CELL)


# ===================================================================== #
#  6.  POLL BUDGET — the ceiling nobody computes
# ===================================================================== #
@dataclass
class PollBudget:
    """What one cell-voltage poll costs, and how fast it can therefore repeat.

    Every figure is an interval. It collapses to a point once the response
    overhead has been measured — see `measure_response_overhead`. Reporting a
    range rather than a midpoint is the honest form: the midpoint of an
    unmeasured quantity is not an estimate of anything, it is a number that
    looks like one.
    """
    stack: StackSpec
    link: UartLink
    request_bytes: int
    response_min: int
    response_max: int
    cycle_min_s: float
    cycle_max_s: float
    rate_ceiling_min_hz: float      # from the SLOWEST cycle — the safe figure
    rate_ceiling_max_hz: float
    settle_s: float
    parse_s: float
    measured: bool
    requested_rate_hz: Optional[float] = None
    findings: list = field(default_factory=list)

    @property
    def max_rate_hz(self) -> float:
        """The defensible ceiling: the one that holds at the worst overhead."""
        return self.rate_ceiling_min_hz

    @property
    def response_bytes(self) -> Optional[int]:
        return self.response_min if self.measured else None

    @property
    def cycle_s(self) -> Optional[float]:
        return self.cycle_min_s if self.measured else None

    def headroom(self) -> Optional[float]:
        if self.requested_rate_hz is None:
            return None
        return self.requested_rate_hz / self.rate_ceiling_min_hz


def poll_budget(stack: StackSpec, link: Optional[UartLink] = None, *,
                requested_rate_hz: Optional[float] = None,
                parse_s: float = 500e-6,
                continuous_adc: bool = True) -> PollBudget:
    """How fast this stack can actually be polled over this UART.

    The cycle is: transmit the read request, wait out the chain's response,
    parse it. In continuous run mode the ADC free-runs, so the round-robin
    settle is a start-up cost rather than a per-poll one; `continuous_adc=False`
    charges it every cycle, which is what a triggered-conversion design pays.

    While the per-device response overhead is unmeasured, every result is an
    interval across `RESPONSE_OVERHEAD_BOUNDS`. A feasibility verdict is issued
    only when it is the same at both ends of that interval; where the two ends
    disagree the finding is MISSING and names the measurement that settles it,
    because "probably fits" is not an answer you can wire a car to.
    """
    link = link or UartLink()
    if stack.cells_per_board is None:
        raise ValueError(
            "cells_per_board is not declared, so the response length is "
            "unknown and the poll rate cannot be computed. This is a real "
            "unanswered question, not a default to fill in.")

    req = len(read_cells_frame(stack.cells_per_board))
    resp_lo, resp_hi = stack.response_bounds()
    measured = stack.response_overhead_bytes is not None
    usable = max(1e-6, min(1.0, link.usable_fraction))
    settle = 0.0 if continuous_adc else adc_settle_s(stack.boards)

    def cycle(resp: int) -> float:
        return (req + resp) * link.byte_time_s() / usable + settle + parse_s

    cyc_lo, cyc_hi = cycle(resp_lo), cycle(resp_hi)
    rate_hi, rate_lo = 1.0 / cyc_lo, 1.0 / cyc_hi     # slow cycle -> low rate

    findings: list[Finding] = []
    if measured:
        findings.append(Finding(
            "bq-poll-ceiling", Severity.INFO,
            f"{stack.boards} x {stack.part} ({stack.total_cells()} cells) "
            f"returns {resp_lo} bytes per cell-voltage poll (overhead "
            f"measured at {stack.response_overhead_bytes} B/device). At "
            f"{link.baud} baud one cycle costs {cyc_lo*1000:.2f} ms, so the "
            f"ceiling is {rate_hi:.0f} Hz.",
            subsystems=["dataacq", "electrics"],
            detail={"cycle_s": cyc_lo, "max_rate_hz": rate_hi,
                    "response_bytes": resp_lo}))
    else:
        findings.append(Finding(
            "bq-response-overhead-unmeasured", Severity.MISSING,
            f"Per-device response framing overhead has not been measured, and "
            f"it is not published in the EVM guide or in SLVAE86B — both "
            f"document command frames and defer the response layout to the "
            f"data sheet's Command and Response Protocol section. Assuming "
            f"{RESPONSE_OVERHEAD_BOUNDS[0]}-{RESPONSE_OVERHEAD_BOUNDS[1]} "
            f"B/device, the response is {resp_lo}-{resp_hi} bytes and the poll "
            f"ceiling is {rate_lo:.0f}-{rate_hi:.0f} Hz. Measure it: one "
            f"broadcast read of the cell block, count the bytes back, feed the "
            f"count to measure_response_overhead(). Five minutes, and it "
            f"turns every figure here from a range into a number.",
            subsystems=["dataacq"],
            detail={"response_min": resp_lo, "response_max": resp_hi,
                    "rate_min_hz": rate_lo, "rate_max_hz": rate_hi}))

    if requested_rate_hz is not None:
        fits_worst = requested_rate_hz <= rate_lo
        fits_best = requested_rate_hz <= rate_hi

        if not fits_best:
            # Infeasible even at the most generous overhead — the assumption
            # cannot rescue it, so this is a real FAIL either way.
            baud_needed = link.baud * (requested_rate_hz / rate_hi)
            findings.append(Finding(
                "bq-poll-infeasible", Severity.FAIL,
                f"{requested_rate_hz:g} Hz polling is not achievable at any "
                f"plausible framing overhead: the link tops out at "
                f"{rate_hi:.0f} Hz even in the best case. The response is "
                f"{resp_lo}-{resp_hi} bytes and it grows linearly with board "
                f"count, so this does not improve with tuning. Either raise "
                f"the baud rate to at least about {baud_needed/1000:.0f} "
                f"kbaud, poll a shorter register block, or accept the lower "
                f"rate. Note that the rate people quote for this bridge is "
                f"usually chosen before anyone multiplies board count by "
                f"payload.",
                subsystems=["dataacq", "electrics"],
                detail={"rate_max_hz": rate_hi,
                        "requested_hz": requested_rate_hz,
                        "baud_needed": baud_needed}))
        elif not fits_worst:
            # The verdict depends entirely on the unmeasured constant. Say so
            # rather than picking whichever side of it looks better.
            findings.append(Finding(
                "bq-poll-indeterminate", Severity.MISSING,
                f"{requested_rate_hz:g} Hz fits if the framing overhead is at "
                f"the low end and does not if it is at the high end — the "
                f"ceiling is somewhere in {rate_lo:.0f}-{rate_hi:.0f} Hz and "
                f"the answer depends entirely on a constant nobody has "
                f"measured. This is deliberately not resolved by picking a "
                f"midpoint: the design either fits or it does not, and a "
                f"five-minute bench measurement decides it. Until then, plan "
                f"against {rate_lo:.0f} Hz.",
                subsystems=["dataacq", "electrics"],
                detail={"rate_min_hz": rate_lo, "rate_max_hz": rate_hi,
                        "requested_hz": requested_rate_hz}))
        elif requested_rate_hz > 0.8 * rate_lo:
            findings.append(Finding(
                "bq-poll-tight", Severity.WARN,
                f"{requested_rate_hz:g} Hz uses "
                f"{requested_rate_hz/rate_lo*100:.0f}% of the "
                f"{rate_lo:.0f} Hz worst-case ceiling. There is no room here "
                f"for a retry after a CRC failure, and a stack that cannot "
                f"retry drops a whole sample rather than a byte.",
                subsystems=["dataacq"],
                detail={"headroom": requested_rate_hz / rate_lo}))
        else:
            findings.append(Finding(
                "bq-poll-ok", Severity.OK,
                f"{requested_rate_hz:g} Hz polling fits inside the "
                f"{rate_lo:.0f} Hz ceiling"
                + ("" if measured else " even at the worst plausible framing "
                   "overhead")
                + ", with room for retries.",
                subsystems=["dataacq"],
                detail={"headroom": requested_rate_hz / rate_lo}))

    return PollBudget(stack, link, req, resp_lo, resp_hi, cyc_lo, cyc_hi,
                      rate_lo, rate_hi, settle, parse_s, measured,
                      requested_rate_hz, findings)


# ===================================================================== #
#  7.  STACK-LEVEL CHECKS
# ===================================================================== #
def stack_findings(stack: StackSpec) -> list[Finding]:
    """Everything checkable about the stack itself, before any timing."""
    out: list[Finding] = []
    dev = stack.device()

    # ---- board count against what the part supports ---------------------- #
    if stack.boards > dev.max_boards:
        out.append(Finding(
            "bq-stack-too-deep", Severity.FAIL,
            f"{stack.boards} boards of {dev.part}, which supports "
            f"{dev.max_boards}."
            + ("" if dev.stackable else
               f" {dev.part} is a standalone monitor and does not daisy-chain "
               f"at all — a multi-board pack needs the stackable variant."),
            subsystems=["electrics", "dataacq"]))
    elif not dev.stackable and stack.boards == 1:
        out.append(Finding(
            "bq-standalone", Severity.INFO,
            f"{dev.part} is standalone: one board, one host link, no "
            f"daisy-chain addressing. That removes the auto-addressing "
            f"sequence and its failure modes entirely.",
            subsystems=["dataacq"]))

    # ---- cells per board ------------------------------------------------- #
    if stack.cells_per_board is None:
        out.append(Finding(
            "bq-cells-undeclared", Severity.MISSING,
            f"Cells per board is not declared. It sets the response length, "
            f"which sets the poll ceiling, which sets whether the requested "
            f"logging rate is possible — so this one blank makes three "
            f"downstream numbers unavailable rather than approximate.",
            subsystems=["electrics", "dataacq"]))
    elif not dev.min_cells <= stack.cells_per_board <= dev.max_cells:
        out.append(Finding(
            "bq-cells-out-of-range", Severity.FAIL,
            f"{stack.cells_per_board} cells per board; {dev.part} supports "
            f"{dev.min_cells}-{dev.max_cells}. Below the minimum the device "
            f"does not simply measure fewer channels — unused inputs have to "
            f"be tied off correctly or the stack reports faults that look "
            f"like real cell problems.",
            subsystems=["electrics"]))

    # ---- current sense --------------------------------------------------- #
    if not dev.current_sense:
        out.append(Finding(
            "bq-no-current-sense", Severity.INFO,
            f"{dev.part} has no integrated current-sense ADC, so pack current "
            f"is a separate channel with its own shunt, amplifier, rail and "
            f"connector — and, more importantly, its own sample clock. Current "
            f"and voltage sampled on unrelated clocks cannot be combined into "
            f"a defensible SOC estimate; if SOC matters, budget the alignment "
            f"work now or specify BQ75614-Q1 / BQ79656-Q1 instead.",
            subsystems=["electrics", "dataacq"]))

    # ---- isolation: same boundary rule daq_plan enforces elsewhere -------- #
    if stack.isolated is not True:
        out.append(Finding(
            "bq-isolation",
            Severity.FAIL if stack.isolated is False else Severity.MISSING,
            "The monitor stack is referenced to the accumulator, on the "
            "tractive-system side of the isolation boundary. The host UART "
            "crosses into the grounded low-voltage system and needs a digital "
            "isolator with an isolated supply on the pack side. The "
            "daisy-chain between boards is already differential and isolated; "
            "the host link is the one that usually is not, because it is the "
            "one that gets added last during bring-up."
            + ("" if stack.isolated is False else
               " Isolation has not been declared either way."),
            subsystems=["electrics", "dataacq"]))

    # ---- NFAULT ---------------------------------------------------------- #
    if stack.nfault_to_interrupt is not True:
        out.append(Finding(
            "bq-nfault-polled",
            Severity.WARN if stack.nfault_to_interrupt is False
            else Severity.MISSING,
            "NFAULT is a hardware fault line already routed to the host "
            "header. If fault status is only read inside the poll loop, the "
            "firmware learns about an over-voltage up to one poll period late "
            "— 100 ms at 10 Hz, during which nothing has reacted. Wiring it to "
            "an interrupt-capable GPIO costs one pin and one handler and "
            "decouples fault latency from logging rate. It does not replace "
            "the hardware shutdown path, which stays hardware.",
            subsystems=["electrics", "dataacq"]))
    else:
        out.append(Finding(
            "bq-nfault-interrupt", Severity.OK,
            "NFAULT wired to a host interrupt — fault latency is independent "
            "of the poll rate.",
            subsystems=["electrics"]))

    # ---- thermistors ----------------------------------------------------- #
    if stack.thermistors_per_board is None:
        out.append(Finding(
            "bq-thermistors-undeclared", Severity.MISSING,
            f"Thermistor count per board not declared. Each device has "
            f"{dev.gpio} GPIO channels usable for pack temperature, and pack "
            f"temperature is the measurement the thermal model needs and the "
            f"rules care about. Unpopulated is a legitimate answer; blank is "
            f"not.",
            subsystems=["electrics", "cooling", "dataacq"]))
    elif stack.thermistors_per_board > dev.gpio:
        out.append(Finding(
            "bq-thermistors-over", Severity.FAIL,
            f"{stack.thermistors_per_board} thermistors per board against "
            f"{dev.gpio} GPIO channels.",
            subsystems=["electrics"]))

    # ---- balancing ------------------------------------------------------- #
    if stack.balancing is None:
        out.append(Finding(
            "bq-balancing-undeclared", Severity.MISSING,
            "Passive balancing not declared. It is not free: the internal FETs "
            "dissipate into the module, which is a thermal load on the pack "
            "the cooling model does not currently include, and the duty cycle "
            "is a parameter someone has to choose rather than inherit.",
            subsystems=["electrics", "cooling"]))

    return out


def resolution_findings(stack: StackSpec, *,
                        resolution_needed_v: Optional[float] = None
                        ) -> list[Finding]:
    """Does the converter resolve what the balancing decision needs?"""
    out: list[Finding] = []
    if resolution_needed_v is None:
        out.append(Finding(
            "bq-resolution-unspecified", Severity.MISSING,
            f"The cell-voltage resolution the balancing decision needs is not "
            f"declared, so {CELL_LSB_V*1e6:.2f} uV/LSB cannot be judged "
            f"adequate or excessive. 'The datasheet number is small' is not "
            f"the same as 'small enough for the decision we make with it'.",
            subsystems=["dataacq", "electrics"]))
        return out

    ratio = resolution_needed_v / CELL_LSB_V
    if ratio < 1.0:
        out.append(Finding(
            "bq-resolution-insufficient", Severity.FAIL,
            f"Balancing needs {resolution_needed_v*1e6:.0f} uV resolution; the "
            f"converter delivers {CELL_LSB_V*1e6:.2f} uV/LSB. The requirement "
            f"is finer than one count.",
            subsystems=["dataacq", "electrics"]))
    else:
        out.append(Finding(
            "bq-resolution-ok", Severity.OK,
            f"{CELL_LSB_V*1e6:.2f} uV/LSB against a "
            f"{resolution_needed_v*1e6:.0f} uV requirement — {ratio:.0f} "
            f"counts per meaningful step. Note this is quantisation only; "
            f"absolute accuracy is a separate datasheet figure and is the one "
            f"that decides whether two cells 5 mV apart are really 5 mV apart.",
            subsystems=["dataacq"],
            detail={"counts_per_step": ratio}))
    return out


# ===================================================================== #
#  8.  HANDOFF TO daq_plan — the signal list the bridge refuses without
# ===================================================================== #
def to_bms_signals(stack: StackSpec, *,
                   cell_rate_hz: float = 10.0,
                   temp_rate_hz: float = 2.0,
                   status_rate_hz: float = 10.0,
                   include_current: Optional[bool] = None
                   ) -> list[BmsSignal]:
    """The declared signal list, derived from the stack as actually specified.

    This is what turns `plan_bms_bridge`'s refusal into a plan. The refusal was
    never about the tool being unable to guess — it was about nobody having
    read the datasheet. This function is the datasheet, read, so the signals it
    emits are counted and typed rather than assumed.
    """
    if stack.cells_per_board is None:
        raise ValueError(
            "cannot derive a signal list without cells_per_board — the number "
            "of cell-voltage signals IS the cell count")

    dev = stack.device()
    sigs: list[BmsSignal] = []

    for b in range(stack.boards):
        for c in range(stack.cells_per_board):
            sigs.append(BmsSignal(
                name=f"cell_v_b{b}_c{c+1}", bits=16, unit="V",
                scale=CELL_LSB_V, rate_hz=cell_rate_hz, critical=True))

    n_therm = stack.thermistors_per_board or 0
    for b in range(stack.boards):
        for t in range(n_therm):
            sigs.append(BmsSignal(
                name=f"pack_temp_b{b}_t{t+1}", bits=16, unit="degC",
                scale=0.1, rate_hz=temp_rate_hz, critical=True))

    want_current = dev.current_sense if include_current is None else include_current
    if want_current:
        sigs.append(BmsSignal(
            name="pack_current", bits=16, unit="A", scale=0.1,
            rate_hz=status_rate_hz, critical=True))

    sigs.append(BmsSignal(
        name="nfault_status", bits=8, unit="", rate_hz=status_rate_hz,
        critical=True))
    sigs.append(BmsSignal(
        name="balance_active", bits=16, unit="", rate_hz=1.0))

    return sigs


def cell_voltage_can_map(stack: StackSpec, *, base_id: int = 0x300,
                         rate_hz: float = 10.0,
                         extended: bool = False) -> list[CanMessage]:
    """Cell voltages packed four to a frame, in cell order.

    Four 16-bit cells fill an 8-byte frame exactly, which is the one packing
    where no bits are wasted and no cell straddles a frame boundary. Straddling
    is worth avoiding for a reason beyond tidiness: a decoder that has to
    reassemble one value from two frames has to decide what to do when only one
    of them arrives, and the honest answer is 'discard', which nobody
    implements.
    """
    if stack.cells_per_board is None:
        raise ValueError("cells_per_board is not declared")
    total = stack.total_cells()
    msgs: list[CanMessage] = []
    per_frame = 4
    for i in range(math.ceil(total / per_frame)):
        first = i * per_frame + 1
        last = min(total, first + per_frame - 1)
        n = last - first + 1
        msgs.append(CanMessage(
            name=f"BMS_CELL_V_{first:02d}_{last:02d}",
            can_id=base_id + i, dlc=n * BYTES_PER_CELL, rate_hz=rate_hz,
            extended=extended, producer="bq796xx_bridge",
            signals=[f"cell_v_{c}" for c in range(first, last + 1)]))
    return msgs


def to_sensor_specs(stack: StackSpec, *,
                    cell_rate_hz: float = 10.0) -> list[SensorSpec]:
    """The stack as daq_plan channels, so it lands in the bus/storage budget.

    Deliberately declared as one aggregate channel per quantity rather than one
    per cell: on the plan side these are already CAN traffic produced by the
    bridge, and the thing the budget needs from them is bytes per second, not a
    hundred near-identical checklist rows nobody will read.
    """
    if stack.cells_per_board is None:
        return []
    total = stack.total_cells()
    specs = [
        SensorSpec(
            key="bms_cell_voltages",
            name=f"Cell voltages ({total} cells via {stack.part})",
            measures="individual cell terminal voltage", unit="V",
            why="the only measurement that finds a weak cell before it becomes "
                "an incident; also the input to every balancing decision",
            location="accumulator", output=OutputType.CAN,
            connector="existing CAN", conductors=0,
            signal_bandwidth_hz=1.0, sample_rate_hz=cell_rate_hz,
            adc_bits=16, range_min_eu=0.0, range_max_eu=5.0,
            resolution_needed_eu=0.001,
            logged_to="logger",
            payload_bytes=total * BYTES_PER_CELL,
            calibration="inject a known cell voltage from a calibrated source "
                        "into one channel and compare the reported count "
                        "against 190.73 uV/LSB; repeat at pack top and bottom",
            galvanic_isolation=stack.isolated,
            available_on_existing_bus="bq796xx_bridge",
            source="BQ796xx EVM documentation",
            notes="Produced by the UART->CAN bridge, not sampled directly."),
    ]
    if stack.thermistors_per_board:
        n = stack.thermistors_per_board * stack.boards
        specs.append(SensorSpec(
            key="bms_pack_temps", name=f"Pack temperatures ({n} thermistors)",
            measures="cell/module surface temperature", unit="degC",
            why="the thermal model is currently an estimate; this is the "
                "measurement that confirms or refutes it",
            location="accumulator", output=OutputType.CAN,
            connector="existing CAN", conductors=0,
            signal_bandwidth_hz=0.2, sample_rate_hz=2.0,
            adc_bits=16, range_min_eu=-20.0, range_max_eu=80.0,
            accuracy_eu=2.0, resolution_needed_eu=0.5,
            logged_to="logger", payload_bytes=n * 2,
            calibration="stirred water bath against one reference probe, all "
                        "thermistors at once — the spread between them matters "
                        "more than any single absolute reading",
            galvanic_isolation=stack.isolated,
            available_on_existing_bus="bq796xx_bridge",
            source="BQ796xx EVM documentation"))
    return specs


# ===================================================================== #
#  9.  ONE CALL
# ===================================================================== #
@dataclass
class BqPlan:
    stack: StackSpec
    device: BqDevice
    budget: Optional[PollBudget]
    signals: list
    messages: list
    bring_up: list
    findings: list = field(default_factory=list)

    def blocking(self) -> list:
        return [f for f in self.findings if f.severity == Severity.FAIL]

    def open_questions(self) -> list:
        return [f for f in self.findings if f.severity == Severity.MISSING]

    def to_markdown(self) -> str:
        L = [f"# BMS monitor stack — {self.device.part}", ""]
        L.append(f"- Boards: **{self.stack.boards}** "
                 f"(part supports {self.device.max_boards})")
        L.append(f"- Cells per board: **{self.stack.cells_per_board}**")
        L.append(f"- Total cells: **{self.stack.total_cells()}**")
        if self.budget:
            b = self.budget
            if b.measured:
                L.append(f"- Response per poll: **{b.response_min} B**")
                L.append(f"- Poll ceiling: **{b.rate_ceiling_min_hz:.0f} Hz** "
                         f"at {b.link.baud} baud")
            else:
                L.append(f"- Response per poll: **{b.response_min}-"
                         f"{b.response_max} B** (framing overhead unmeasured)")
                L.append(f"- Poll ceiling: **{b.rate_ceiling_min_hz:.0f}-"
                         f"{b.rate_ceiling_max_hz:.0f} Hz** "
                         f"at {b.link.baud} baud")
        L.append(f"- Wake-to-addressable: "
                 f"**{wake_time_s(self.stack.boards)*1000:.1f} ms**")
        L.append("")
        L.append("## Findings")
        L.append("")
        L.append("| Severity | Check | Message |")
        L.append("|---|---|---|")
        order = {Severity.FAIL: 0, Severity.MISSING: 1, Severity.WARN: 2,
                 Severity.INFO: 3, Severity.OK: 4}
        for f in sorted(self.findings, key=lambda f: order[f.severity]):
            msg = f.message.replace("|", "/").replace("\n", " ")
            L.append(f"| {f.severity.value.upper()} | {f.check} | {msg} |")
        return "\n".join(L)


def plan_stack(stack: StackSpec, *,
               link: Optional[UartLink] = None,
               requested_rate_hz: Optional[float] = None,
               resolution_needed_v: Optional[float] = None,
               base_id: int = 0x300,
               continuous_adc: bool = True) -> BqPlan:
    """Check the stack, budget the link, and emit the frames and the CAN map."""
    dev = stack.device()
    findings = stack_findings(stack)
    findings.extend(resolution_findings(
        stack, resolution_needed_v=resolution_needed_v))

    budget = None
    signals: list = []
    messages: list = []
    bring_up: list = []

    if stack.cells_per_board is not None:
        budget = poll_budget(stack, link, requested_rate_hz=requested_rate_hz,
                             continuous_adc=continuous_adc)
        findings.extend(budget.findings)
        rate = requested_rate_hz or 10.0
        signals = to_bms_signals(stack, cell_rate_hz=rate)
        messages = cell_voltage_can_map(stack, base_id=base_id, rate_hz=rate)
        if dev.stackable and stack.boards > 1:
            bring_up = auto_address_sequence(stack.boards)
        bring_up = bring_up + start_adc_sequence(stack.cells_per_board)

    return BqPlan(stack, dev, budget, signals, messages, bring_up, findings)


# ===================================================================== #
#  10. PROVENANCE
# ===================================================================== #
PROVENANCE = {
    "from_documentation": [
        "init byte table, CRC polynomial, read-length encoding (n-1, max 128) "
        "and the 8-byte write ceiling",
        "the init byte's low nibble encodes (data bytes - 1) on writes, "
        "confirmed by SLVAE86B Tables 1-3/1-5/1-7 (0x93, 0xB3, 0xD3 for four "
        "data bytes) and by the section 6.2 balancing frames (0xD7 for eight)",
        "wake pulse 2.5 ms; wake settle 10 ms + 600 us per device",
        "round-robin ADC settle 192 us + 5 us per board",
        "cell voltage scale 190.73 uV/LSB, 16-bit two's complement",
        "auto-addressing, balancing and reverse-addressing sequences, "
        "including the single-board COMM_CTRL = 0x01 case",
        "part capabilities: BQ79616-Q1 stackable to 35 boards; BQ75614-Q1 and "
        "BQ79656-Q1 standalone with integrated current sense; BQ79656-Q1 ASIL-D",
    ],
    "sources": [
        "BQ796xx EVM documentation (team notes)",
        "TI SLVAE86B — BQ79616-Q1 Software Design Reference, Rev. B, "
        "August 2023. Sections 4.2, 5.2, 6.2 and 9.2 supply 24 complete "
        "worked frames, all of which CRC_ANCHORS pins.",
    ],
    "corrected_against_documentation": [
        "The prose names the frame check 'CRC-16-IBM', whose textbook seed is "
        "0x0000. That seed rejects all 24 worked example frames. Seed 0xFFFF "
        "accepts all 24, so 0xFFFF is what is implemented. A wrong seed "
        "produces silence rather than an error, which is why this is worth "
        "stating.",
        "TI's own Table 1-2..1-7 CRC column is rendered inconsistently — some "
        "entries in transmitted byte order, some in integer order. The "
        "complete worked byte strings are unambiguous and are what is pinned.",
    ],
    "measured_not_assumed": [
        "Per-device response framing overhead is NOT published in the EVM "
        "guide or in SLVAE86B; both document command frames only and defer "
        "the response layout to the data sheet's Command and Response "
        "Protocol section. Rather than assume a value, this module reports "
        "response length and poll ceiling as intervals across "
        "RESPONSE_OVERHEAD_BOUNDS and refuses to issue a feasibility verdict "
        "where the two ends of the interval disagree. "
        "measure_response_overhead() converts it to a measurement in five "
        "minutes, and the bridge firmware has a menu entry that performs it.",
    ],
    "estimate_flagged": [
        "parse_s = 500 us of host processing per poll, an order-of-magnitude "
        "placeholder until it is measured on the actual host.",
    ],
    "hard_rule": (
        "cells_per_board has no default. It sets the response length, which "
        "sets the poll ceiling, which decides whether the requested logging "
        "rate is possible — so guessing it would produce three confident "
        "downstream numbers from one invented one. Undeclared raises rather "
        "than assumes, and where an unmeasured constant decides a verdict the "
        "verdict is withheld rather than taken from the midpoint."
    ),
}
