"""Three physics corrections, each pinned against a closed form it must reproduce.

* EV energy and pack current at constant speed: the motor still overcomes drag and rolling resistance when the
  car is not accelerating, so a steady stretch costs F v / eta, not nothing.
* Brake rotor spin: the rotor turns with the wheel, omega = v / R_wheel, not v over the rotor's own radius.
* Tyre inflation gas: the cavity is a fixed volume, so the gas heats with c_v (718 J/kg K), not c_p.
Units SI unless named.
"""
import math

import numpy as np
import pytest

from suspension.brake_thermal import h_rotating_disc
from suspension.ev_powertrain import EVLapSimulator, EVParams, LapSimParams
from suspension.pack_thermal import pack_current_trace
from suspension.tire_cosim import WheelState
from suspension.tire_thermal import ThermalTireModel


class _Lap:
    pass


def _steady_lap(v=20.0, length=1000.0, n=1001):
    lap = _Lap()
    lap.distance = np.linspace(0.0, length, n)
    lap.speed = np.full(n, v)
    lap.long_g = np.zeros(n)
    return lap


def test_constant_speed_draws_drag_and_rolling_energy():
    p, ev = LapSimParams(), EVParams()
    v, length = 20.0, 1000.0
    net_kwh, regen_kwh = EVLapSimulator(ev)._energy_from_trace(_steady_lap(v, length), p)
    force = 0.5 * p.rho * p.cd_a * v * v + p.rolling_g * p.mass * p.g
    expected = force * length / ev.inverter_motor_eff / 3.6e6
    assert net_kwh == pytest.approx(expected, rel=1e-9)
    assert regen_kwh == 0.0


def test_gentle_deceleration_below_drag_still_draws():
    """A deceleration smaller than drag plus rolling resistance alone would give still needs the motor to push."""
    p, ev = LapSimParams(), EVParams()
    lap = _steady_lap()
    resist = 0.5 * p.rho * p.cd_a * 20.0 ** 2 + p.rolling_g * p.mass * p.g
    lap.long_g = np.full(lap.speed.size, -0.5 * resist / (p.mass * p.g))      # half of what drag would give
    net_kwh, regen_kwh = EVLapSimulator(ev)._energy_from_trace(lap, p)
    assert net_kwh > 0.0 and regen_kwh == 0.0


def test_pack_current_at_constant_speed_matches_the_force_balance():
    p, ev = LapSimParams(), EVParams()
    v, v_pack = 20.0, 400.0
    t, cur = pack_current_trace(_steady_lap(v), p, pack_nominal_v=v_pack,
                                inverter_motor_eff=ev.inverter_motor_eff,
                                regen_eff=ev.regen_eff, regen_max_g=ev.regen_max_g)
    force = 0.5 * p.rho * p.cd_a * v * v + p.rolling_g * p.mass * p.g
    assert cur[1:] == pytest.approx(force * v / ev.inverter_motor_eff / v_pack, rel=1e-9)
    net_kwh, _ = EVLapSimulator(ev)._energy_from_trace(_steady_lap(v), p)
    assert float(np.trapezoid(cur * v_pack, t)) / 3.6e6 == pytest.approx(net_kwh, rel=0.01)


def test_rotor_spins_with_the_wheel():
    v, d_mm, r_wheel_mm = 20.0, 220.0, 228.0
    nu, k, pr = 1.6e-5, 0.026, 0.71
    r, D = d_mm / 2000.0, d_mm / 1000.0
    omega = v / (r_wheel_mm / 1000.0)
    h_rot = 0.015 * (omega * r * r / nu) ** 0.8 * k / r
    h_x = 0.037 * (v * D / nu) ** 0.8 * pr ** (1.0 / 3.0) * k / D
    assert h_rotating_disc(d_mm, v, wheel_radius_mm=r_wheel_mm) == pytest.approx(math.hypot(h_rot, h_x), rel=1e-9)
    # a bigger wheel turns the same rotor more slowly at the same road speed: less convection
    assert h_rotating_disc(d_mm, v, wheel_radius_mm=300.0) < h_rotating_disc(d_mm, v, wheel_radius_mm=200.0)


def test_inflation_gas_heats_at_constant_volume():
    m = ThermalTireModel()
    m.reset(WheelState(ambient_temp_c=25.0, track_temp_c=25.0))
    m.T_carcass, m.T_gas = 60.0, 25.0
    p, dt = m.params, 0.01
    m.step(WheelState(Fz=0.0, v_x=0.0, dt=dt, ambient_temp_c=25.0, track_temp_c=25.0))
    expected = p.k_carcass_gas * (60.0 - 25.0) * dt / (p.m_gas_kg * 718.0)
    assert m.T_gas - 25.0 == pytest.approx(expected, rel=1e-9)
