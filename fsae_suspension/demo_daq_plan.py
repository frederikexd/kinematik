#!/usr/bin/env python3
# ============================================================================
#  KinematiK — demo_daq_plan.py
#  Walks the vehicle-side DAQ planner through the exact state a data-acq
#  meeting leaves behind: a wish list of sensors, a datasheet nobody has read
#  yet, and a review checklist that nothing enforces.
# ============================================================================
"""Run me:  python demo_daq_plan.py

Five acts, in the order the problems actually surface.
"""

from suspension import daq_plan as dp
from suspension.interfaces import Severity


BAR = "=" * 78
SEV = {Severity.OK: "OK  ", Severity.INFO: "INFO", Severity.WARN: "WARN",
       Severity.FAIL: "FAIL", Severity.MISSING: "OPEN"}


def show(findings, only=None, limit=None):
    n = 0
    for f in findings:
        if only and f.severity not in only:
            continue
        print(f"  [{SEV[f.severity]}] {f.message}")
        n += 1
        if limit and n >= limit:
            break
    if not n:
        print("  (none)")


# ---------------------------------------------------------------- act 1 ---- #
print(BAR)
print("ACT 1 — the wish list, straight off the slide")
print(BAR)

sensors = dp.cooling_package()
for s in sensors:
    print(f"  {s.name:<42s} {s.completeness()*100:5.0f}% documented")

p = dp.plan(sensors, bus=dp.BusSpec(bitrate_bps=500_000),
            rails=dp.default_rails(), logger=dp.LoggerSpec())
print(f"\n  verdict: {p.verdict.value.upper()}   "
      f"documentation {p.completeness*100:.0f}% complete")
print("\n  Still open:")
for k, qs in p.open_questions.items():
    print(f"    {k}: {'; '.join(qs)}")

print("\n  Worth noticing before anyone orders parts:")
show([f for f in p.findings if f.check in ("already-on-bus",
                                           "isolation-required")])

# ---------------------------------------------------------------- act 2 ---- #
print()
print(BAR)
print("ACT 2 — 'what sampling rate is needed?', answered by taste")
print(BAR)

guessed = [
    dp.catalog("coolant_temp_in", sample_rate_hz=1000.0),   # thermal at 1 kHz
    dp.catalog("coolant_flow", sample_rate_hz=2.0,
               signal_bandwidth_hz=2.0),                    # aliases
]
for s in guessed:
    print(f"\n  {s.name} @ {s.sample_rate_hz:g} Hz "
          f"(bandwidth {s.signal_bandwidth_hz:g} Hz)")
    show(dp.signal_chain_findings(s),
         only={Severity.FAIL, Severity.WARN, Severity.INFO})

# ---------------------------------------------------------------- act 3 ---- #
print()
print(BAR)
print("ACT 3 — the BMS bridge, before and after the datasheet")
print(BAR)

link = dp.UartLink(baud=115_200, frame_bytes=64, frame_rate_hz=10.0)

print("\n  Before — 'how do we get the BMS onto CAN?'")
b0 = dp.plan_bms_bridge(link, [], bus=dp.BusSpec())
print(f"    refused: {b0.refused} ({b0.refusal_reason})")
show(b0.findings)

print("\n  After — someone read it and wrote the signals down:")
sigs = [
    dp.BmsSignal("pack_voltage", 16, "V", 0.1, rate_hz=10, critical=True),
    dp.BmsSignal("pack_current", 16, "A", 0.1, offset=-3200.0,
                 rate_hz=10, critical=True),
    dp.BmsSignal("cell_v_min", 16, "V", 0.0001, rate_hz=10, critical=True),
    dp.BmsSignal("cell_v_max", 16, "V", 0.0001, rate_hz=10, critical=True),
    dp.BmsSignal("temp_max", 8, "degC", 1.0, offset=-40.0,
                 rate_hz=10, critical=True),
    dp.BmsSignal("fault_flags", 16, "", rate_hz=10, critical=True),
    dp.BmsSignal("soc", 8, "%", 0.5, rate_hz=1),
    dp.BmsSignal("balancing_mask", 32, "", rate_hz=1),
]
bridge = dp.plan_bms_bridge(link, sigs, base_id=0x300, bus=dp.BusSpec(),
                            isolated=None)
print(f"    {len(sigs)} signals -> {len(bridge.messages)} CAN frames")
for m in bridge.messages:
    print(f"      0x{m.can_id:03X}  dlc={m.dlc}  @{m.rate_hz:>4g} Hz   "
          f"{', '.join(m.signals)}")
print()
show(bridge.findings, only={Severity.FAIL, Severity.MISSING, Severity.WARN})

# ---------------------------------------------------------------- act 4 ---- #
print()
print(BAR)
print("ACT 4 — the coolant pair cannot measure what it was bought for")
print(BAR)

pair = dp.find_coolant_pair(sensors)
for matched, label in ((False, "as bought"), (True, "matched-pair calibrated")):
    r = dp.delta_t_budget(pair[0], pair[1], expected_delta_t_k=6.0,
                          flow_lpm=12.0, matched_pair=matched)
    print(f"\n  {label}:")
    print(f"    delta-T   {6.0:.1f} +/- {r.sigma_delta_t_k:.2f} K "
          f"({r.relative_error*100:.0f}%)")
    print(f"    heat      {r.heat_kw:.1f} +/- {r.sigma_heat_kw:.1f} kW "
          f"({r.heat_relative_error*100:.0f}%)")
print("\n  Same two sensors. The difference is an afternoon with a stirred")
print("  water bath, not a different purchase order.")

# ---------------------------------------------------------------- act 5 ---- #
print()
print(BAR)
print("ACT 5 — what happens when the channel list keeps growing")
print(BAR)

full = list(sensors)
for i in range(4):
    full.append(dp.SensorSpec(
        key=f"susp_pot_{i}", name=f"Damper position {i}",
        measures="damper displacement", unit="mm", why="motion ratio",
        location="damper", output=dp.OutputType.ANALOG_V,
        supply_v=5.0, current_ma=8.0, supply_rail="5V",
        connector="AS 3-way", conductors=3,
        signal_bandwidth_hz=30.0, sample_rate_hz=500.0,
        antialias_cutoff_hz=100.0, adc_bits=12,
        range_min_eu=0.0, range_max_eu=75.0, accuracy_eu=0.15,
        resolution_needed_eu=0.1, logged_to="logger", payload_bytes=2,
        calibration="dial-indicator sweep", owner="daq", source="ELPM-75"))

for rate in (125, 250, 500):
    q = dp.plan(full, bus=dp.BusSpec(bitrate_bps=rate * 1000.0),
                rails=dp.default_rails(), logger=dp.LoggerSpec(),
                bridge=bridge)
    br = q.bus_result
    flag = "  <-- over budget" if br.load >= q.bus.load_fail else ""
    late = f"  ({len(br.unschedulable)} late)" if br.unschedulable else ""
    print(f"  {rate:>4d} kbit/s : {br.load*100:5.1f}% worst-case load"
          f"{late}{flag}")

q = dp.plan(full, bus=dp.BusSpec(bitrate_bps=125_000),
            rails=dp.default_rails(), logger=dp.LoggerSpec(), bridge=bridge)
print(f"\n  At 125 kbit/s the verdict is {q.verdict.value.upper()}:")
show(q.blocking(), limit=4)

print()
print(BAR)
print("The review table is generated, not maintained:")
print(BAR)
print(q.to_markdown()[:900] + "\n  ...")
