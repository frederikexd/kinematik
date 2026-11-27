"""Characterization suite for suspension/daq_plan.py (mutation gate); see tests/_golden.py. The 14-channel sample
plan with a fault planted for every check, every catalog sensor alone and with its questions knocked out one at a
time, the CAN frame arithmetic for every DLC, bus loads at three bitrates, the coolant delta-T budget, power and
storage budgets, the BMS bridge (refused, fitting, overrun) and the plan's verdict, markdown and CSV."""
import dataclasses

from suspension import daq_plan as D
from suspension import daq_sample as S
from tests._golden_lazy import check_lazy
from tests._golden import plain as _plain


def plain(x):
    """tests._golden.plain, with sets sorted (their repr follows the hash seed)."""
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: plain(getattr(x, f.name)) for f in dataclasses.fields(x)}
    if isinstance(x, (set, frozenset)):
        return sorted((plain(v) for v in x), key=repr)
    if isinstance(x, dict):
        return {str(k): plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    return _plain(x)


def _try(fn, *a, **kw):
    try:
        return plain(fn(*a, **kw))
    except (ValueError, KeyError, TypeError) as e:
        return {"raised": type(e).__name__, "message": str(e)}


def _sensor(s):
    return {"na": plain(s.not_applicable()), "ans": s.answered(), "done": s.completeness(), "un": s.unanswered(),
            "teams": s.affected_subteams(), "ts": s.on_tractive_system(), "span": s.span_eu(), "dict": plain(s.as_dict()),
            "chain": plain(D.signal_chain_findings(s)), "doc": plain(D.documentation_findings(s)),
            "integ": plain(D.integration_findings(s)), "can": plain(D.sensor_to_can(s, 0x123)),
            "can_ext": plain(D.sensor_to_can(s, 0x1ABCDE, extended=True))}


def _variants(s):
    """The sensor with each answered question removed, one at a time, plus a few deliberately wrong numbers."""
    out = {}
    for f in dataclasses.fields(s):
        if f.name in ("key", "name", "owner", "source", "notes", "is_estimate"):
            continue
        if getattr(s, f.name) is not None:
            out[f"no_{f.name}"] = dataclasses.replace(s, **{f.name: None})
    out["aliased"] = dataclasses.replace(s, sample_rate_hz=3.0, signal_bandwidth_hz=10.0, antialias_cutoff_hz=None)
    out["oversampled"] = dataclasses.replace(s, sample_rate_hz=5000.0, signal_bandwidth_hz=2.0, antialias_cutoff_hz=1.0)
    out["coarse"] = dataclasses.replace(s, adc_bits=6, range_min_eu=0.0, range_max_eu=1000.0, resolution_needed_eu=0.1)
    out["on_ts"] = dataclasses.replace(s, location="accumulator", galvanic_isolation=False)
    out["estimate"] = dataclasses.replace(s, is_estimate=True, calibration="TBD")
    return out


def _everything():
    for k in sorted(D.CATALOG):
        s = D.catalog(k)
        yield f"cat.{k}", _sensor(s)
        for v, sv in _variants(s).items():
            yield f"cat.{k}.{v}", {"chain": plain(D.signal_chain_findings(sv)), "doc": plain(D.documentation_findings(sv)),
                                   "integ": plain(D.integration_findings(sv)), "done": sv.completeness(),
                                   "can": plain(D.sensor_to_can(sv, 0x200))}
    yield "cat.override", plain(D.catalog("motor_temp", sample_rate_hz=25.0, owner="PT"))
    try:
        D.catalog("nope")
    except KeyError as e:
        yield "cat.unknown", str(e)
    for dlc in range(0, 9):
        for ext in (False, True):
            yield f"bits.{dlc}.{ext}", [D.can_frame_bits(dlc, extended=ext),
                                        D.can_frame_bits(dlc, extended=ext, worst_case_stuffing=False)]
    sensors = S.sample_sensors()
    msgs = [m for i, s in enumerate(sensors) if (m := D.sensor_to_can(s, 0x400 + i)) is not None]
    for m in msgs[:3]:
        yield f"msg.{m.name}", [m.bits(), m.bits(False), m.frame_time_s(500_000.0), m.frame_time_s(1e6, False)]
    for name, bus in (("sample", S.sample_bus()), ("slow", D.BusSpec(bitrate_bps=125_000.0)),
                      ("fast", D.BusSpec(bitrate_bps=1_000_000.0, extended_ids=True)),
                      ("tight", D.BusSpec(bitrate_bps=250_000.0, load_ok=0.01, load_warn=0.02, load_fail=0.04))):
        yield f"bus.{name}", plain(D.bus_load(msgs, bus))
        yield f"bus.{name}.x10", plain(D.bus_load([dataclasses.replace(m, rate_hz=m.rate_hz * 10) for m in msgs], bus))
    yield "bus.empty", plain(D.bus_load([], D.BusSpec()))
    pair = D.find_coolant_pair(D.cooling_package())
    yield "pair", plain(pair)
    yield "pair.none", plain(D.find_coolant_pair([D.catalog("motor_temp")]))
    if pair:
        a, b = pair
        for dt in (2.0, 8.0, 0.5):
            for flow in (None, 20.0):
                for mp in (False, True):
                    yield f"dt.{dt}.{flow}.{mp}", plain(D.delta_t_budget(a, b, expected_delta_t_k=dt, flow_lpm=flow,
                                                                          matched_pair=mp))
        yield "dt.unanswered", plain(D.delta_t_budget(dataclasses.replace(a, accuracy_eu=None), b,
                                                      expected_delta_t_k=5.0))
    for name, rails in (("sample", S.sample_rails()), ("default", D.default_rails()),
                        ("tiny", {"5V": D.Rail("5V", 5.0, capacity_ma=20.0, fuse_a=0.05)})):
        yield f"power.{name}", plain(D.power_budget(sensors, rails))
    for name, lg in (("sample", S.sample_logger()), ("small", D.LoggerSpec(storage_mb=1.0, session_minutes=60.0)),
                     ("default", D.LoggerSpec())):
        yield f"storage.{name}", plain(D.storage_budget(sensors, lg))
    for link in (D.UartLink(), D.UartLink(baud=9600, parity="even", stop_bits=2, frame_bytes=40),
                 D.UartLink(baud=19200, data_bits=7, parity="odd", frame_bytes=12, frame_rate_hz=50.0)):
        yield f"uart.{link.baud}.{link.parity}", [link.bits_per_byte(), link.byte_time_s(), link.frame_time_s(),
                                                  link.utilisation()]
    sig = [D.BmsSignal("v", 16, "V", 0.1, rate_hz=10.0, critical=True), D.BmsSignal("i", 16, "A", 0.1, -3200, 50.0, True),
           D.BmsSignal("soc", 8, "%", 0.5, rate_hz=1.0), D.BmsSignal("t", 12, "degC"), D.BmsSignal("blk", 64, rate_hz=10.0),
           D.BmsSignal("huge", 80)]
    yield "pack", _try(D.pack_signals, sig[:5])
    yield "pack.too_wide", _try(D.pack_signals, sig)
    yield "pack.ext", plain(D.pack_signals(sig[:4], base_id=0x18FF0000, extended=True, default_rate_hz=5.0, producer="x"))
    yield "bridge.sample", plain(S.sample_bridge())
    yield "bridge.refused", plain(D.plan_bms_bridge(D.UartLink(frame_bytes=20), []))
    yield "bridge.fits", _try(D.plan_bms_bridge, D.UartLink(baud=115_200, frame_bytes=20, frame_rate_hz=10.0), sig[:3],
                              isolated=None, bus=D.BusSpec())
    yield "bridge.not_isolated", plain(D.plan_bms_bridge(D.UartLink(frame_bytes=None), sig[:2], isolated=False))
    for name, kw in (("sample", None), ("bare", dict(sensors=[D.catalog("motor_temp")])),
                     ("cooling", dict(sensors=D.cooling_package(), expected_delta_t_k=6.0, flow_lpm=18.0,
                                      matched_pair=True, rails=D.default_rails(), logger=D.LoggerSpec())),
                     ("empty", dict(sensors=[]))):
        p = S.sample_plan() if kw is None else D.plan(**kw)
        yield f"plan.{name}", {"plan": plain(p), "blocking": plain(p.blocking()), "actions": plain(p.subteam_actions()),
                               "warn": plain(p.by_severity(D.Severity.WARN)), "md": p.to_markdown(), "csv": p.to_csv()}
    yield "provenance", plain(D.PROVENANCE)


def test_every_sensor_bus_and_budget():
    check_lazy("daq_plan", _everything())
