"""Characterization suite for suspension/aero/daq.py (mutation gate); see tests/_golden.py. Balance calibrations
(identity, cross-coupled, singular, uncalibrated, railed, wrong shapes), scanner, balance and chassis status
lines, both vibration filters on clean, toned, structural, short, holed and all-NaN signals (resolved and not), the
biquad designs, acquisition specs and provenance, balance readings and their status, the offline and synthetic
backends, and the virtual instrument end to end with and without filters, balance or scanners."""
import math

import numpy as np

from suspension.aero import cfd
from suspension.aero import daq as D
from suspension.aero.pressure_tap import TapCalibration, TapLocation, WingSurface
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _cal(**kw):
    m = np.eye(6) * np.array([200., 200., 400., 50., 50., 50.])
    m[2, 0], m[4, 0], m[2, 1] = 30.0, 8.0, 12.0
    return D.BalanceCalibration(matrix=m, is_calibrated=True, serial="B6", **kw)


def _scanner(cal_ok=True, p="t"):
    taps = [TapLocation(p + "0", x_over_c=0.1), TapLocation(p + "1", x_over_c=0.5, surface=WingSurface.PRESSURE),
            TapLocation(p + "2", x_over_c=0.9)]
    cals = {p + "0": TapCalibration(sensitivity_pa_per_v=1000.0, is_calibrated=cal_ok, saturation_v=5.0, serial="s0"),
            p + "1": TapCalibration(sensitivity_pa_per_v=0.0, is_calibrated=True)}
    return D.PressureScannerSpec(vendor=D.ScannerVendor.CHELL, port_taps=taps, calibrations=cals, serial="Z")


def _try(fn, *a, **k):
    try:
        return plain(fn(*a, **k))
    except (ValueError, D.DAQUnavailable) as e:
        return {"raised": type(e).__name__, "message": str(e)}


def _sig(n=1000, fs=2000.0, tone=137.0, amp=0.05, struct=None, seed=3):
    t = np.arange(n) / fs
    x = 1.0 + 0.2 * np.sin(2 * np.pi * 3.0 * t) + amp * np.sin(2 * np.pi * tone * t)
    if struct:
        x += 0.03 * np.sin(2 * np.pi * struct * t)
    return x + 0.002 * np.random.default_rng(seed).standard_normal(n)


def _filt(f, x):
    clean, rep = f.apply(x)
    return [plain(np.round(clean[::50], 9)), plain(rep.as_dict())]


def _cases():
    yield "axes", [[a.value for a in D.BalanceAxis.forces()], [a.value for a in D.BalanceAxis.all_six()]]
    ident = D.BalanceCalibration.identity([100, 100, 200, 10, 10, 10], serial="I", notes="lab")
    for name, c in (("cross", _cal(saturation_v=10.0)), ("ident", ident),
                    ("ident_plain", D.BalanceCalibration.identity([1, 2, 3, 4, 5, 6])),
                    ("singular", D.BalanceCalibration(matrix=np.zeros((6, 6)), is_calibrated=True)),
                    ("uncal", D.BalanceCalibration(is_calibrated=False)),
                    ("nomatrix", D.BalanceCalibration(is_calibrated=True)),
                    ("zeros", D.BalanceCalibration(matrix=np.eye(6), zero_volts=[0.1] * 6, is_calibrated=True,
                                                   saturation_v=-2.0))):
        v = np.array([[0.1, 0.2, 0.3, 0.0, 0.0, 0.0], [9.99, 0.0, 0.0, 0.0, 0.0, 0.0], [10.0, 0, 0, 0, 0, 0],
                      [-2.0, 0, 0, 0, 0, 0], [1.99, 0, 0, 0, 0, 0]])
        yield f"cal.{name}", [c.status(), c.usable(), c._well_conditioned(), plain(c.decouple(v)),
                              plain(c.decouple(v[0])), plain(c.zero_volts), c.notes]
    yield "cal.errors", [_try(D.BalanceCalibration, matrix=np.eye(5)), _try(D.BalanceCalibration, zero_volts=[0.0] * 5),
                         _try(D.BalanceCalibration.identity, [1, 2, 3]), _try(_cal().decouple, np.zeros((3, 5)))]
    yield "scanner", [_scanner().n_channels, _scanner().status(),
                      D.PressureScannerSpec(D.ScannerVendor.SCANIVALVE, [], {}).status()]
    yield "balance_spec", [D.ForceBalanceSpec(_cal()).status(), D.ForceBalanceSpec(ident, "strut", "S9").status()]
    yield "chassis", [D.DAQChassis().status(), D.DAQChassis(500.0, "cDAQ-9178", 8).status(), D.DAQChassis(10.0).nyquist_hz,
                      _try(D.DAQChassis, 0.0), _try(D.DAQChassis, -5.0)]
    x = _sig()
    holed = x.copy()
    holed[100:110] = np.nan
    for name, f in (("vf", D.VibrationFilter(2000.0, fan_blade_pass_hz=137.0)),
                    ("vf_struct", D.VibrationFilter(2000.0, fan_blade_pass_hz=137.0, structural_hz=(55.0, 0.0, 1500.0),
                                                    n_harmonics=0, aero_cutoff_hz=20.0, notch_q=10.0)),
                    ("vf_none", D.VibrationFilter(2000.0)),
                    ("vf_hi", D.VibrationFilter(2000.0, fan_blade_pass_hz=600.0, n_harmonics=3, notch_q=0.0)),
                    ("svf", D.StreamingVibrationFilter(2000.0, fan_blade_pass_hz=137.0)),
                    ("svf_struct", D.StreamingVibrationFilter(2000.0, structural_hz=(55.0, None), aero_cutoff_hz=1200.0,
                                                              n_harmonics=2, warmup_s=0.1)),
                    ("svf_none", D.StreamingVibrationFilter(2000.0, fan_blade_pass_hz=-1.0))):
        yield f"filt.{name}", [_filt(f, x), _filt(f, holed), _filt(f, x[:3]), _filt(f, np.full(20, np.nan)),
                               _filt(f, _sig(n=40, fs=2000.0)), plain(f._notch_freqs())]
    s = D.StreamingVibrationFilter(1000.0, fan_blade_pass_hz=137.0)
    s.reset(1.0)
    yield "svf.stream", [s.warmup_samples, [s.process_sample(v) for v in (1.0, 1.2, 0.8, 1.1)],
                         plain(np.round(s.process_block(np.array([1.0, 1.05, 0.95, 1.0])), 12))]
    yield "svf.bad", _try(D.StreamingVibrationFilter, 0.0)
    yield "biquads", [plain(D._rbj_notch(137.0, 2000.0, 30.0)), plain(D._rbj_lowpass(30.0, 2000.0, 0.7071))]
    bq = D._Biquad(*D._rbj_lowpass(30.0, 2000.0, 0.7071))
    bq.reset(2.0)
    yield "biquad.step", [bq.step(2.0), bq.step(3.0), bq.step(1.0)]
    spec = D.AcquisitionSpec(seconds=0.5, attitude=cfd.Attitude(pitch_deg=0.3), speed_ms=20.0)
    ch = D.DAQChassis(2000.0)
    yield "spec", [spec.n_samples(ch), D.AcquisitionSpec(seconds=0.0001).n_samples(ch), D.AcquisitionSpec().n_samples(ch)]
    prov = D.DAQProvenance("A2", "synthetic", 2000.0, 3.0, 6000, 20.0, 1.2, synthetic=True, filtered=True,
                           dropped_samples=4, notes="n")
    yield "prov", [prov.averaging_ok(), prov.averaging_ok(3.0), prov.averaging_ok(3.01), prov.status(),
                   D.DAQProvenance("B", "nidaq", 500.0, 10.0, 5000, 25.0, 1.225, D.GroundState.FIXED_FLOOR).status()]
    rd = D.BalanceReading({"Fx": 120.0, "Fy": -3.0, "Fz": -900.0, "Mx": 1.0}, {"Fx": 1.5, "Fz": 4.0}, 1000,
                          provenance=prov)
    hole = D.BalanceReading({"Fx": math.nan, "Fz": math.nan}, {}, 0)
    yield "reading", [rd.Fx, rd.Fy, rd.Fz, rd.downforce_N(), rd.is_usable(), rd.status(), rd.value(D.BalanceAxis.MZ),
                      hole.downforce_N(), hole.is_usable(), hole.status()]
    off = D.OfflineDAQ()
    yield "offline", [_try(off.read_balance, spec, ch), _try(off.read_scanner, _scanner(), spec, ch), off.name,
                      off.synthetic, isinstance(off, D.DAQBackend)]
    loads = {"Fx": 150.0, "Fy": 5.0, "Fz": -1200.0, "My": 30.0}
    pres = {"t0": -800.0, "t1": 300.0, "t2": -100.0}
    for name, cal in (("cal", _cal()), ("nocal", None), ("uncal", D.BalanceCalibration(is_calibrated=False))):
        syn = D.SyntheticDAQ(loads, pres, balance_cal=cal, seed=4)
        yield f"syn.{name}", [plain(np.round(syn.read_balance(spec, ch)[:3], 9)),
                              plain(np.round(syn.read_scanner(_scanner(), spec, ch)[:3], 9)),
                              plain(syn.read_scanner(D.PressureScannerSpec(D.ScannerVendor.GENERIC, [], {}), spec,
                                                     ch).shape)]
    for name, vib in (("raw", None), ("vf", D.VibrationFilter(2000.0, fan_blade_pass_hz=137.0)),
                      ("svf", D.StreamingVibrationFilter(2000.0, fan_blade_pass_hz=137.0))):
        sp = D.AcquisitionSpec(seconds=0.5, attitude=cfd.Attitude(), speed_ms=22.0, rho=1.2, vibration=vib,
                               p_static_inf_pa=10.0, p_total_inf_pa=300.0)
        vi = D.VirtualInstrument(facility="A2", chassis=ch, balance=D.ForceBalanceSpec(_cal(saturation_v=10.0)),
                                 scanners=[_scanner(), _scanner(False, "u")],
                                 backend=D.SyntheticDAQ(loads, pres, balance_cal=_cal(), seed=5))
        f = vi.acquire_forces(sp)
        p = vi.acquire_pressures(sp)
        yield f"vi.{name}", [plain(f.forces_N), plain(f.stderr_N), f.n_used, f.status(), plain(p.volts.shape),
                             plain(np.round(p.volts[:2], 9)), [t.tap_id for t in p.taps], sorted(p.calibrations),
                             plain(vi.scan_provenance(sp).__dict__), vi.status(), plain(f.provenance.__dict__),
                             plain(f.interaction_matrix), plain(p.attitude)]
    bare = D.VirtualInstrument(facility="X", chassis=ch)
    yield "vi.bare", [_try(bare.acquire_forces, spec), _try(bare.acquire_pressures, spec), bare.status(),
                      bare.backend.name]
    one = D.VirtualInstrument(facility="Y", chassis=ch, balance=D.ForceBalanceSpec(D.BalanceCalibration()),
                              backend=D.SyntheticDAQ(loads, pres, seed=1), ground_state=D.GroundState.SUCTION_FIXED)
    r1 = one.acquire_forces(spec)
    yield "vi.uncal", [plain(r1.forces_N), r1.n_used, r1.status(), one.status(), plain(r1.stderr_N),
                       plain(r1.provenance.__dict__), plain(r1.interaction_matrix)]
    both = D.VirtualInstrument(facility="Z", chassis=ch, balance=D.ForceBalanceSpec(_cal()), scanners=[_scanner()],
                               backend=D.SyntheticDAQ(loads, pres, balance_cal=_cal(), fan_amp_v=0.0, turb_v=0.0))
    fr, sc = both.acquire(spec)
    yield "vi.acquire", [plain(fr.forces_N), plain(fr.stderr_N), plain(np.round(sc.volts[:2], 12))]

    class Bad:
        name, synthetic = "bad", False

        def read_balance(self, s, c):
            return np.zeros((10, 5))

        def read_scanner(self, sc, s, c):
            return np.zeros((10, 1))
    badvi = D.VirtualInstrument(facility="B", chassis=ch, balance=D.ForceBalanceSpec(_cal()), scanners=[_scanner()],
                                backend=Bad())
    yield "vi.badshape", [_try(badvi.acquire_forces, spec), _try(badvi.acquire_pressures, spec)]

    class IntB(Bad):
        name = "int"

        def read_scanner(self, sc, s, c):
            return np.arange(s.n_samples(c) * 3).reshape(-1, 3)
    iv = D.VirtualInstrument(facility="I", chassis=ch, scanners=[_scanner()], backend=IntB())
    sp_v = D.AcquisitionSpec(seconds=0.05, vibration=D.VibrationFilter(2000.0, fan_blade_pass_hz=137.0))
    yield "vi.int", plain(np.round(iv.acquire_pressures(sp_v).volts[:3], 9))
    noisy = D.VirtualInstrument(facility="N", chassis=ch, balance=D.ForceBalanceSpec(_cal()),
                                backend=D.SyntheticDAQ(loads, pres, balance_cal=_cal(), seed=2))
    yield "vi.tiny", [[plain(noisy.acquire_forces(D.AcquisitionSpec(seconds=s)).stderr_N),
                       plain(noisy.acquire_forces(D.AcquisitionSpec(seconds=s)).forces_N)] for s in (0.0005, 0.001)]
    nanm = D.BalanceCalibration(matrix=np.full((6, 6), np.nan), is_calibrated=True)
    yield "cal.extra", [_try(nanm._well_conditioned), plain(D.BalanceCalibration(zero_volts=[[0.1] * 6]).zero_volts),
                        plain(D.BalanceCalibration.identity([[1, 2, 3, 4, 5, 6]]).matrix)]
    yield "chassis.one", [_try(lambda: D.DAQChassis(1.0).nyquist_hz),
                          _try(lambda: D.StreamingVibrationFilter(1.0)._notch_freqs())]
    tail = x[:300].copy()
    tail[200:] = np.nan
    for name, f in (("vf_slow", D.VibrationFilter(2000.0, fan_blade_pass_hz=0.5, structural_hz=(0.5, -5.0, 1000.0))),
                    ("vf_band", D.VibrationFilter(2000.0, fan_blade_pass_hz=100.0, n_harmonics=1, notch_q=50.0)),
                    ("vf_low", D.VibrationFilter(2000.0, structural_hz=(20.0,))),
                    ("vf_edge", D.VibrationFilter(2000.0, structural_hz=(50.0,))),
                    ("svf_slow", D.StreamingVibrationFilter(2000.0, fan_blade_pass_hz=0.5, n_harmonics=0,
                                                            structural_hz=(0.5, -5.0, 1000.0, 1500.0), warmup_s=0.0)),
                    ("svf_one", D.StreamingVibrationFilter(2000.0, fan_blade_pass_hz=137.0, n_harmonics=1,
                                                           warmup_s=0.0005, aero_cutoff_hz=1000.0)),
                    ("svf_low", D.StreamingVibrationFilter(2000.0, structural_hz=(20.0,), warmup_s=0.0)),
                    ("svf_wlow", D.StreamingVibrationFilter(2000.0, structural_hz=(20.0,), warmup_s=0.01)),
                    ("svf_tail", D.StreamingVibrationFilter(2000.0, structural_hz=(55.0,), warmup_s=0.1))):
        yield f"filt2.{name}", [_filt(f, x[:4]), _filt(f, x[:999]), _filt(f, np.ones(100)), _filt(f, x[:40]),
                                _filt(f, x[:100]), _filt(f, x[:150]), _filt(f, tail), plain(f._notch_freqs())]
    s2 = D.StreamingVibrationFilter(1000.0, fan_blade_pass_hz=137.0)
    s2.reset()
    cold = D._Biquad(*D._rbj_lowpass(30.0, 2000.0, 0.7071))
    first = [cold.step(1.0), cold.step(1.0)]
    cold.reset()
    sa = []
    for a in ([1.0, -1.0, 0.0], [1.0, 0.5, -0.5]):
        q = D._Biquad(np.array([1.0, 2.0, 3.0]), np.array(a))
        try:
            q.reset(2.0)
            sa.append([q.z1, q.z2])
        except ZeroDivisionError:
            sa.append("zero division")
    yield "biquad.extra", [s2.process_sample(1.0), first, cold.step(1.0), sa]
    p55 = D.DAQProvenance("C", "nidaq", 500.0, 5.5, 2750, 25.0, 1.225)
    yield "prov.extra", [p55.averaging_ok(), p55.status(), D.BalanceReading({"Fz": math.inf}, {}, 1).downforce_N()]
    zc = D.BalanceCalibration(matrix=np.eye(6), zero_volts=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0], is_calibrated=True)
    sc2 = _scanner()
    sc2.calibrations["t0"] = TapCalibration(sensitivity_pa_per_v=1000.0, is_calibrated=True, zero_offset_v=0.2)
    syn2 = D.SyntheticDAQ(loads, {"t1": 300.0}, balance_cal=zc)
    yield "syn.extra", [plain(np.round(syn2.read_balance(spec, ch)[:2], 9)),
                        plain(np.round(syn2.read_scanner(sc2, spec, ch)[:2], 9))]


def test_every_daq_step():
    check_lazy("aero_daq", _cases(), rel=1e-7, abs_=1e-9)
