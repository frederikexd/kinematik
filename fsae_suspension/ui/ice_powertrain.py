# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/ice_powertrain.py — the ⛽ combustion side of the ⚡ Powertrain tab's EV / ICE switch
# ============================================================================
"""
The combustion counterpart of the EV panels: engine curve against the restrictor, gears and shift points, the
clutch launch and the 75 m, the lap and its fuel, endurance fuel and CO2, the engine's heat into the cooling loop,
engine braking against the brake balance, the feasibility gate and the ledger.

All physics lives in suspension/ice_powertrain.py. This module only orchestrates and draws (see ui/__init__.py).
Session keys used:
    ice_curve_csv     the uploaded dyno or engine-code curve (text), None for the representative shape
    ledger            the shared IntegrationLedger (written only on "Publish")
"""
from __future__ import annotations

_STATUS = {"PASS": "🟢", "WARN": "🟠", "FAIL": "🔴", "MISSING": "⚪"}


def _ratios(text: str) -> tuple:
    return tuple(float(x) for x in text.replace(";", ",").split(",") if x.strip())


def render():
    import numpy as np
    import pandas as pd
    import streamlit as st

    from suspension import Hardpoints, SuspensionKinematics, VehicleDynamics, VehicleParams
    from suspension.provenance import graded
    from suspension import ice_powertrain as ice
    from suspension import laptime as lt
    from suspension import tiremodel as tm

    ss = st.session_state
    ss.setdefault("ice_curve_csv", None)
    st.info("Beta: the combustion powertrain is new. Its physics is checked against closed forms, not yet against "
            "GT-SUITE, OptimumLap or a dyno. Feedback on results that look wrong is very welcome.")
    st.caption(
        "For a combustion car: the same questions the EV panels answer, for an engine and a gearbox. The torque "
        "curve is declared (dyno, GT-POWER or Ricardo WAVE export); until you load one, a representative restricted "
        "600 cc inline-four shape is used and every result says so.")

    c1, c2, c3, c4 = st.columns(4)
    with c1:
        fuel_key = st.selectbox("Fuel", list(ice.FUELS), format_func=lambda k: ice.FUELS[k].name, key="ice_fuel")
        f = ice.FUELS[fuel_key]
        restr = st.number_input("Restrictor diameter (mm)", 10.0, 30.0, f.restrictor_mm, 0.1,
                                key=f"ice_restr_{fuel_key}",
                                help=f"Rule maximum for {f.name}: {f.restrictor_mm} mm (check the current rules).")
        bte = st.number_input("Brake thermal efficiency", 0.15, 0.45, 0.30, 0.01, key="ice_bte",
                              help="One number for the whole map: fuel is a planning estimate until a BSFC map "
                                   "from the dyno replaces it.")
    with c2:
        up = st.file_uploader("Torque curve (CSV: rpm, torque_nm or power_kw)", type=["csv", "txt"],
                              key="ice_curve_up")
        if up is not None:
            ss.ice_curve_csv = up.getvalue().decode("utf-8", "replace")
        redline = st.number_input("Redline (rpm)", 5000.0, 20000.0, 13500.0, 100.0, key="ice_redline")
        launch_rpm = st.number_input("Launch rpm", 2000.0, 15000.0, 9000.0, 100.0, key="ice_launch")
    with c3:
        primary = st.number_input("Primary reduction", 1.0, 4.0, 2.111, 0.001, format="%.3f", key="ice_primary")
        ratios_txt = st.text_input("Gear ratios, first to top", "2.750, 2.000, 1.667, 1.444, 1.304, 1.208",
                                   key="ice_ratios")
        final = st.number_input("Final drive", 1.5, 6.0, 3.0, 0.05, key="ice_final")
        shift_t = st.number_input("Shift time (s)", 0.0, 0.5, 0.08, 0.01, key="ice_shift")
    with c4:
        mass = st.number_input("Mass with driver (kg)", 150.0, 450.0, 300.0, 1.0, key="ice_mass")
        rear = st.number_input("Static rear share", 0.3, 0.8, 0.55, 0.01, key="ice_rear")
        tire_r = st.number_input("Loaded tire radius (m)", 0.15, 0.35, 0.225, 0.001, format="%.3f", key="ice_r")
        tank = st.number_input("Usable tank volume (L, 0 = undeclared)", 0.0, 20.0, 0.0, 0.1, key="ice_tank")

    try:
        eng = (ice.EngineMap.from_csv(ss.ice_curve_csv, redline_rpm=redline) if ss.ice_curve_csv
               else ice.EngineMap.representative_600_i4())
        if not ss.ice_curve_csv:
            eng.redline_rpm = redline
        gb = ice.Gearbox(primary=primary, ratios=_ratios(ratios_txt), final_drive=final, shift_time_s=shift_t,
                         tire_radius_m=tire_r)
    except Exception as e:                                  # noqa: BLE001 - show the input error
        st.error(f"Check the engine curve and the gear ratios: {e}")
        return
    veh = ice.ICEVehicle(mass_kg=mass, driven_weight_frac=rear)
    rep_curve = eng.source.startswith("representative")
    grade = "estimate" if rep_curve else "modelled"
    why = "the representative torque curve" if rep_curve else "the declared torque curve"

    def gv(value, unit, digits=4, fuel_based=False):
        """A result with its evidence grade; fuel results also rest on the declared BTE (uncalibrated)."""
        return graded(value, grade, unit, digits=digits, calibrated=not fuel_based,
                      limited_by="the declared BTE" if fuel_based else why)
    if eng.source.startswith("representative"):
        st.warning("Representative engine curve in use: load your dyno or engine-code curve for real numbers.")

    tabs = st.tabs(["📈 Curve & restrictor", "⚙️ Gears & shift points", "🚦 Launch & 75 m", "🏁 Lap & fuel",
                    "🌡️ Cooling", "🛑 Engine braking", "✅ Feasibility gate", "📤 Publish"])

    with tabs[0]:
        rc = eng.restrictor_check(fuel_key, bte, d_mm=restr)
        rpm = np.linspace(eng.rpm[0], eng.redline_rpm, 120)
        st.line_chart(pd.DataFrame({"torque (Nm)": [eng.torque_at(x) for x in rpm],
                                    "power (kW)": [eng.power_kw_at(x) for x in rpm],
                                    "restrictor ceiling (kW)": rc["ceiling_kw"]}, index=rpm))
        st.metric("Choked air flow", graded(ice.choked_mass_flow_kg_s(restr, cd=0.95) * 1e3, "modelled", "g/s",
                                            digits=3),
                  help="Cd 0.95; 101.325 kPa and 25 °C upstream")
        (st.warning if rc["exceeds"] else st.success)(
            f"Peak {gv(rc['peak_kw'], 'kW', 3)} at {round(rc['peak_rpm'])} rpm against a ceiling of "
            f"{gv(rc['ceiling_kw'], 'kW', 3, fuel_based=True)} at BTE {bte}"
            + (": the curve is unrestricted, or the BTE is set low." if rc["exceeds"] else "."))

    with tabs[1]:
        sv = gb.shift_speeds(eng)
        st.dataframe(pd.DataFrame({"gear": [f"{g + 1} → {g + 2}" for g in range(gb.n - 1)],
                                   "upshift at (km/h)": [round(v * 3.6, 1) for v in sv],
                                   "redline speed (km/h)": [round(gb.speed_at(eng.redline_rpm, g) * 3.6, 1)
                                                            for g in range(gb.n - 1)]}), hide_index=True)
        v = np.linspace(1.0, gb.top_speed_at_redline(eng), 120)
        st.line_chart(pd.DataFrame({f"gear {g + 1}": [gb.wheel_force(x, g, eng) if gb.rpm_at(x, g) <= eng.redline_rpm
                                                      else None for x in v] for g in range(gb.n)}, index=v * 3.6))
        st.caption(f"Top speed at redline in top gear: {gv(gb.top_speed_at_redline(eng) * 3.6, 'km/h', 3)}. "
                   f"Ratios: {gb.source}.")

    with tabs[2]:
        res = ice.simulate_launch(eng, gb, veh, fuel_key=fuel_key, bte=bte, launch_rpm=launch_rpm)
        m1, m2, m3, m4 = st.columns(4)
        m1.metric("75 m", gv(res.time_s, "s"))
        m2.metric("Trap speed", gv(res.trap_speed_kmh, "km/h"))
        m3.metric("Shifts", len(res.shifts))
        m4.metric("Clutch heat", gv(res.clutch_heat_kj, "kJ", 3))
        st.line_chart(pd.DataFrame({"speed (km/h)": res.v_ms * 3.6, "rpm / 100": res.rpm / 100},
                                   index=res.t_s), x_label="time (s)")
        for w in res.warnings:
            st.caption("⚠️ " + w)
        if st.button("Sweep the final drive", key="ice_sweep"):
            sw = ice.sweep_final_drive(eng, gb, veh, fuel_key=fuel_key, bte=bte, launch_rpm=launch_rpm)
            st.dataframe(pd.DataFrame(sw.as_table()).round(3), hide_index=True)
            st.success(f"Fastest 75 m at a final drive of {round(sw.best_ratio, 2)}: {gv(sw.times_s.min(), 's')}.")

    kin = SuspensionKinematics(Hardpoints.default())
    vdyn = VehicleDynamics(VehicleParams(mass=mass), front_kin=kin, rear_kin=kin, tire=tm.default_tire())
    lap = ice.simulate_lap(vdyn, lt.default_autocross(), eng, gb, veh)
    fr = ice.fuel_for_lap(lap.lap, eng, gb, veh, fuel_key=fuel_key, bte=bte) if lap.lap.ok else None

    with tabs[3]:
        if fr is None:
            st.error("The lap did not solve: " + "; ".join(lap.warnings))
        else:
            end = ice.endurance_fuel(fr, tank_l=tank or None, fuel_key=fuel_key)
            m1, m2, m3, m4 = st.columns(4)
            m1.metric("Lap (default autocross)", gv(lap.lap_time_s, "s"),
                      help=f"QSS lap {gv(lap.lap.lap_time_s, 's')} plus {gv(lap.shift_penalty_s, 's', 2)} for "
                           f"{lap.shifts} upshifts")
            m2.metric("Fuel per lap", gv(fr.fuel_l * 1000, "mL", 3, fuel_based=True))
            m3.metric("Endurance fuel (22 km)", gv(end.fuel_l, "L", 3, fuel_based=True))
            m4.metric("Endurance CO2", gv(end.co2_kg, "kg", 3, fuel_based=True))
            st.caption(f"{gv(end.l_per_100km, 'L/100 km', 3, fuel_based=True)} against the "
                       f"{ice.FUEL_LIMIT_L_PER_100KM} L/100 km "
                       "combustion limit of the 2024 efficiency proposal (check the current rules). The "
                       "efficiency score needs the field's best values, so it is not computed.")
            st.line_chart(pd.DataFrame({"speed (km/h)": np.asarray(lap.lap.v) * 3.6, "gear": lap.gear_trace},
                                       index=lap.lap.s), x_label="distance (m)")
            for w in lap.warnings + fr.warnings:
                st.caption("⚠️ " + w)

    cool = None
    with tabs[4]:
        if fr is not None:
            k1, k2, k3 = st.columns(3)
            share = k1.number_input("Fuel energy to coolant", 0.15, 0.45, 0.28, 0.01, key="ice_cfrac")
            ua = k2.number_input("Radiator UA (W/K)", 50.0, 3000.0, 700.0, 10.0, key="ice_ua")
            flow = k3.number_input("Coolant flow (L/min)", 2.0, 80.0, 25.0, 1.0, key="ice_flow")
            fr_c = ice.fuel_for_lap(lap.lap, eng, gb, veh, fuel_key=fuel_key, bte=bte, coolant_frac=share)
            cool = ice.size_cooling(fr_c, radiator_ua_w_per_k=ua, flow_lpm=flow)
            (st.success if cool.ok else st.error)(
                f"{gv(fr_c.mean_coolant_heat_w / 1e3, 'kW', 3, fuel_based=True)} lap-mean to coolant (peak "
                f"{gv(fr_c.peak_coolant_heat_w / 1e3, 'kW', 3, fuel_based=True)}); radiator UA margin "
                f"{gv(cool.ua_margin, '', 3, fuel_based=True)}; steady coolant "
                f"{gv(cool.steady_coolant_c, '°C', 3, fuel_based=True)}.")
            for n in cool.notes:
                st.caption(n)

    with tabs[5]:
        b1, b2, b3 = st.columns(3)
        dec = b1.number_input("Deceleration (g)", 0.2, 2.0, 1.5, 0.05, key="ice_dec")
        bias = b2.number_input("Declared front brake bias", 0.4, 0.9, 0.60, 0.01, key="ice_bias")
        spd = b3.number_input("Speed (km/h)", 10.0, 150.0, 70.0, 1.0, key="ice_eb_v")
        eng.overrun_torque_nm = st.number_input("Overrun torque at the crank (Nm)", 0.0, 40.0,
                                                eng.overrun_torque_nm, 0.5, key="ice_overrun")
        bb = ice.brake_balance_with_engine_braking(dec, bias, spd / 3.6, eng, gb, veh)
        st.info(f"Engine braking adds {graded(bb['engine_braking_n'], 'guess', 'N', digits=3, limited_by='the declared overrun torque')} "
                f"at the rear, so the effective front share is "
                f"{graded(100 * bb['effective_front_share'], 'guess', '%', digits=3, limited_by='the declared overrun torque')} "
                f"against the declared {round(100 * bias)}%.")

    with tabs[6]:
        if fr is not None:
            p1, p2 = st.columns(2)
            pump = p1.number_input("Fuel pump flow (L/h, 0 = undeclared)", 0.0, 400.0, 0.0, 1.0, key="ice_pump")
            inj = p2.number_input("Injector flow (cc/min each, 0 = undeclared)", 0.0, 2000.0, 0.0, 5.0,
                                  key="ice_inj")
            gate = ice.feasibility_gate(eng, gb, fr, fuel_key=fuel_key, bte=bte, restrictor_mm=restr,
                                        tank_l=tank or None, fuel_pump_lph=pump or None,
                                        injector_cc_min=inj or None, lap_top_speed_ms=max(lap.lap.v), cooling=cool)
            st.dataframe(pd.DataFrame([{"": _STATUS.get(g.status, ""), "check": g.check, "status": g.status,
                                        "detail": g.detail} for g in gate]), hide_index=True)

    with tabs[7]:
        st.caption("Writes the engine's peak torque and power to the powertrain interface and the lap-mean coolant "
                   "heat to cooling, marked as estimates.")
        if st.button("Publish to the Integration ledger", key="ice_publish"):
            ice.publish_to_ledger(ss.get("ledger"), eng, fr)
            st.success("Published.")
