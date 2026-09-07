# Data acquisition channel plan — usage

Two new files that let KinematiK answer a question it previously could not:
**"can we actually log all this?"**

* `suspension/daq_plan.py` — the vehicle-side channel model: the review
  checklist as an enforced schema, plus CAN bus load, arbitration latency,
  Nyquist, ADC quantisation, rail current, card space, coolant ΔT error
  propagation, and the BMS UART→CAN bridge.
* `ui/daq_plan.py` — the 📡 **Data Acquisition** tab.

---

## Why these exist

The `dataacq` role in `streamlit_app.py` used to map to `["pcb", "dfmea"]`.
The data-acquisition subteam had no tool of its own — it borrowed the
electronics board checker and the failure-mode matrix, neither of which knows
what a channel is.

`suspension/aero/daq.py` is not this. That module is *bench* acquisition for
the wind tunnel: force-balance calibration matrices, pressure scanners,
vibration filtering, a virtual instrument. It has nothing to say about the car.
Nothing in the repo did CAN budgeting, sensor documentation, or a serial
bridge:

```
grep -ril "bus load\|bit stuff\|CAN utilisation" suspension/*.py   # nothing
```

So a sensor list grew one meeting at a time, in a slide deck, with a
ten-question review checklist that nothing enforced and four pieces of
arithmetic that nobody performed.

---

## The hard rule

A plan with unanswered review questions **never returns READY**, and its bus,
power and storage figures are reported as **floors**.

This is the point of the module. An undeclared channel contributes *zero* to
the arithmetic and a *positive* amount to the real bus, so a green "38% load"
printed over six half-specified channels is not an optimistic estimate — it is
a wrong one, and it is the number that gets screenshotted into a design review.

The same rule governs the bridge: `plan_bms_bridge` **refuses** to emit a frame
map from an empty signal list rather than inventing a plausible layout, because
an invented map is worse than a blank page — it stops the next person opening
the datasheet.

---

## The checklist as a schema

`SensorSpec` has one field per review question. Every engineering field is
`Optional` and defaults to `None`, and `None` means **not answered yet** — a
state honestly distinct from a declared zero.

```python
from suspension import daq_plan as dp

s = dp.SensorSpec(key="motor_temp", name="Motor stator temperature")
s.completeness()      # 0.0
s.unanswered()        # every question, verbatim
```

Completeness is derived from the cells, so it cannot drift from the table the
way a hand-maintained status column does. Add a question to `CHECKLIST` and
every existing sensor becomes incomplete until it is answered — which is the
correct behaviour.

**The only waiver** is narrow and derived, never settable: a value already
broadcast by an existing device (`output` in `BUS_TYPES` *and*
`available_on_existing_bus` set) consumes no rail and no connector of its own.
The inverter is powered and wired whether or not anyone logs its temperature.

---

## What is computed, and how

| Quantity | Method |
|---|---|
| CAN frame length | ISO 11898-1 field layout + worst-case bit-stuffing bound. Anchors: **135 bits** for an 8-byte standard frame, **160** extended |
| Bus load | exact Σ (frame bits × rate) / bitrate, worst-case stuffed |
| Message latency | fixed-point response-time analysis for non-preemptive fixed-priority arbitration (one blocking frame + higher-priority interference) |
| Nyquist | per channel against declared bandwidth; **2×** floor, **5×** practical, **10×** comfortable, **>50×** flagged as waste |
| ADC resolution | span / (2^bits − 1) in engineering units vs the resolution specified |
| Coolant ΔT | uncorrelated error propagation, σ = √(σ₁² + σ₂²) |
| Heat rejection | ρ·V̇·cp·ΔT, with flow and temperature error combined |
| UART link | byte framing (start + data + parity + stop) against baud |

`PROVENANCE` separates the physics from the estimates. The latency analysis
assumes strictly periodic transmission and no queueing jitter, so it is a
**lower** bound — a controller with a FIFO transmit buffer does worse.

---

## Quick start

```python
from suspension import daq_plan as dp

sensors = dp.cooling_package()          # motor/inverter temp, coolant in/out, flow

p = dp.plan(sensors,
            bus=dp.BusSpec(bitrate_bps=500_000),
            rails=dp.default_rails(),
            logger=dp.LoggerSpec(storage_mb=8192, session_minutes=25),
            expected_delta_t_k=6.0, flow_lpm=12.0)

p.verdict            # Verdict.INCOMPLETE
p.completeness       # 0.91
p.bus_result.load    # 0.006
p.blocking()         # the FAIL findings
p.subteam_actions()  # findings routed to whoever must act
print(p.to_markdown())
```

Override catalog defaults with your own parts — the catalog answers the generic
questions, not the local ones:

```python
s = dp.catalog("motor_temp", connector="AS 3-way", owner="ana",
               sample_rate_hz=25.0, galvanic_isolation=True)
```

---

## The BMS bridge

```python
link = dp.UartLink(baud=115200, frame_bytes=64, frame_rate_hz=10.0)

dp.plan_bms_bridge(link, []).refused          # True — read the datasheet first

sigs = [
    dp.BmsSignal("pack_voltage", 16, "V", 0.1, rate_hz=10, critical=True),
    dp.BmsSignal("cell_v_min",   16, "V", 0.0001, rate_hz=10, critical=True),
    dp.BmsSignal("soc",           8, "%", 0.5, rate_hz=1),
]
b = dp.plan_bms_bridge(link, sigs, base_id=0x300,
                       bus=dp.BusSpec(), isolated=True)
```

Signals are grouped **by rate** before packing — a 10 Hz cell voltage sharing a
frame with a 100 Hz pack current forces one of them to the other's rate, which
either wastes nine tenths of the bandwidth or delivers the fast one late.
Within a rate group, shutdown-relevant signals get the lowest identifiers,
because on CAN priority is not a field you set, it is the number you choose.

The bridge always raises the isolation boundary: the BMS is referenced to the
accumulator, so its serial link crosses into the grounded low-voltage system
and needs a galvanic barrier. Un-isolated, the serial ground wire is a
conductive path between the two systems.

---

## Coolant ΔT — the check worth running first

An inlet and an outlet probe are not two measurements. They are one
*difference*, and a difference inherits the error of both ends while keeping a
fraction of the magnitude:

```
two ±2 K probes, 6 K rise   →  ±2.83 K  (47%)  →  4.2 ± 2.0 kW
same hardware, matched pair →  ±0.71 K  (12%)  →  4.2 ± 0.5 kW
```

That is the same two sensors. The difference is an afternoon with a stirred
water bath calibrating both against one reference, which cancels the shared
systematic offset — not a different purchase order. `find_coolant_pair()`
detects the pair automatically, so nobody has to remember to ask.

---

## Cross-subteam routing

`LOCATION_SUBTEAMS` maps where a sensor mounts to who owns a piece of it, so
"what other subteam does this affect?" is computed rather than answered by
whoever happened to be in the room. `TRACTIVE_SYSTEM_LOCATIONS` marks the
locations that sit across the isolation boundary; a sensor there without
declared isolation is `MISSING`, and one explicitly un-isolated is `FAIL`.

`DaqPlan.subteam_actions()` inverts the findings into a per-team chore list,
dropping the OK and INFO entries — a team's list should contain only things
that are theirs *and* actionable.

---

## Tab wiring

Registered in `streamlit_app.py` as tab id `daq` under **Design & Sizing**,
ahead of `pcb` so the `dataacq` role lands on it first. Shared with
`electrics`, `cooling` and `powertrain`, which all own sensors. Follows the
`ui/` strangulation pattern from `CONTRIBUTING.md`: physics in `suspension/`,
drawing in `ui/`, a single `render()` entry point, streamlit imported inside it
so the package stays importable headless.

---

## Tests

`tests/test_daq_plan.py` — 98 tests. The frame arithmetic is pinned to the two
published anchors, the response-time analysis is checked for priority ordering
and for naming its casualties on a saturated bus, and the refusal paths are
tested as contracts rather than as edge cases.

```bash
python -m pytest tests/test_daq_plan.py -q
```
