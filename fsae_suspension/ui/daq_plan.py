# ============================================================================
#  KinematiK — ui/daq_plan.py
#  Streamlit panel for the vehicle-side DAQ channel planner. Holds the sensor
#  list, runs the checklist + Nyquist + bus/power/storage budgets and the BMS
#  bridge, and refuses to show a READY verdict over an unanswered question.
# ============================================================================
"""Render the Data Acquisition tab.

Design intent, consistent with the rest of KinematiK: this panel is built to
FAIL LOUDLY and to say when it does not know. A budget computed from a
half-specified channel list is shown as a FLOOR with the word floor on it, not
as a reassuring percentage — because the number that gets screenshotted into a
design review is the one people remember.

No physics here. Every equation lives in suspension/daq_plan.py; this file
orchestrates and draws.
"""

from __future__ import annotations

try:
    import streamlit as st
except Exception:                       # keeps the package importable headless
    st = None

from suspension.interfaces import Severity
from suspension import daq_plan as dp


_SEV_ICON = {
    Severity.OK: "✅", Severity.INFO: "ℹ️", Severity.WARN: "⚠️",
    Severity.FAIL: "❌", Severity.MISSING: "⭕",
}
_SEV_ORDER = [Severity.FAIL, Severity.MISSING, Severity.WARN,
              Severity.INFO, Severity.OK]

_OUTPUT_LABELS = {o.value: o for o in dp.OutputType}


# --------------------------------------------------------------------------- #
#  session helpers
# --------------------------------------------------------------------------- #
def _sensors() -> list:
    ss = st.session_state
    if "daq_sensors" not in ss:
        # Start from the sensors a cooling/powertrain instrumentation
        # discussion lands on, so the tab opens on something real rather than
        # an empty table nobody fills in.
        ss["daq_sensors"] = dp.cooling_package()
    return ss["daq_sensors"]


def _bridge():
    return st.session_state.get("daq_bridge")


def _spec_from_row(row: dict, base: dp.SensorSpec) -> dp.SensorSpec:
    """Rebuild a SensorSpec from an edited table row.

    Blank cells come back as None and stay None. That is the whole contract:
    the editor must never silently turn 'not answered' into a default.
    """
    def g(col, cast=None):
        v = row.get(col, None)
        if v is None or v == "":
            return None
        if cast is None:
            return v
        try:
            return cast(v)
        except (TypeError, ValueError):
            return None

    out = g("output")
    return dp.SensorSpec(
        key=base.key, name=g("name") or base.name,
        measures=g("measures"), unit=g("unit") or base.unit, why=g("why"),
        location=g("location"),
        output=_OUTPUT_LABELS.get(out) if out else None,
        supply_v=g("supply_v", float), current_ma=g("current_ma", float),
        supply_rail=g("supply_rail"),
        connector=g("connector"), conductors=g("conductors", int),
        signal_bandwidth_hz=g("signal_bandwidth_hz", float),
        sample_rate_hz=g("sample_rate_hz", float),
        antialias_cutoff_hz=g("antialias_cutoff_hz", float),
        adc_bits=g("adc_bits", int),
        range_min_eu=g("range_min_eu", float),
        range_max_eu=g("range_max_eu", float),
        accuracy_eu=g("accuracy_eu", float),
        resolution_needed_eu=g("resolution_needed_eu", float),
        logged_to=g("logged_to"), payload_bytes=g("payload_bytes", int),
        calibration=g("calibration"),
        galvanic_isolation=(None if row.get("galvanic_isolation") in (None, "")
                            else bool(row.get("galvanic_isolation"))),
        available_on_existing_bus=base.available_on_existing_bus,
        owner=g("owner") or "", source=g("source") or "",
        is_estimate=bool(row.get("is_estimate", False)),
        notes=base.notes,
    )


# --------------------------------------------------------------------------- #
#  main entry point
# --------------------------------------------------------------------------- #
def render():
    if st is None:
        raise RuntimeError("streamlit not available")
    ss = st.session_state

    st.subheader("📡 Data Acquisition — the channel plan that checks itself")
    st.caption(
        "Every sensor the team wants, with the review questions attached to it "
        "and the arithmetic those questions imply actually carried out: "
        "Nyquist, CAN bus load, rail current, card space. A blank stays blank "
        "and counts against the plan."
    )

    st.info("**How to read this.** " + dp.PROVENANCE["hard_rule"], icon="⚖️")

    tabs = st.tabs(["Channels", "Bus & budgets", "BMS bridge",
                    "Cooling ΔT", "Export"])

    with tabs[0]:
        _render_channels()
    with tabs[1]:
        _render_budgets()
    with tabs[2]:
        _render_bridge()
    with tabs[3]:
        _render_delta_t()
    with tabs[4]:
        _render_export()


# --------------------------------------------------------------------------- #
#  1. channels
# --------------------------------------------------------------------------- #
def _render_channels():
    ss = st.session_state
    sensors = _sensors()

    st.markdown("#### The sensor list")
    st.caption(
        "Nine review questions per channel. The completeness bar is computed "
        "from the cells, so it cannot drift from the table the way a "
        "hand-maintained status column does."
    )

    # ---- add from catalog / blank -------------------------------------- #
    c1, c2, c3 = st.columns([3, 2, 2])
    with c1:
        pick = st.selectbox("Add from catalog",
                            ["—"] + sorted(dp.CATALOG.keys()),
                            key="daq_add_pick")
    with c2:
        if st.button("Add catalog sensor", key="daq_add_cat",
                     use_container_width=True) and pick != "—":
            new = dp.catalog(pick)
            n = sum(1 for s in sensors if s.key.startswith(pick))
            if n:
                new.key = f"{pick}_{n+1}"
                new.name = f"{new.name} ({n+1})"
            sensors.append(new)
            st.rerun()
    with c3:
        if st.button("Add blank channel", key="daq_add_blank",
                     use_container_width=True):
            sensors.append(dp.SensorSpec(key=f"channel_{len(sensors)+1}",
                                         name=f"New channel {len(sensors)+1}"))
            st.rerun()

    if not sensors:
        st.warning("No channels yet. Add one above — an empty plan is not a "
                   "passing plan, it is an absent one.")
        return

    # ---- completeness at a glance --------------------------------------- #
    st.markdown("#### Documentation completeness")
    for s in sensors:
        c = s.completeness()
        cols = st.columns([3, 4, 3])
        with cols[0]:
            st.markdown(f"**{s.name}**")
        with cols[1]:
            st.progress(c, text=f"{c*100:.0f}%")
        with cols[2]:
            open_q = s.unanswered()
            if open_q:
                st.caption(f"⭕ {len(open_q)} open")
            else:
                st.caption("✅ complete")
        if s.unanswered():
            with st.expander(f"Open questions — {s.name}", expanded=False):
                for q in s.unanswered():
                    st.markdown(f"* {q}")
                na = s.not_applicable()
                if na:
                    st.caption(
                        "Waived as structurally not applicable: "
                        + ", ".join(sorted(na))
                        + " — this value is already broadcast by an existing "
                          "device, which is powered and wired regardless.")

    # ---- the editable table ---------------------------------------------- #
    st.markdown("#### Edit the specifications")
    st.caption("Leave a cell empty when you do not know yet. Empty is an "
               "honest state and the plan tracks it; a guessed number is not.")

    try:
        import pandas as pd
        rows = []
        for s in sensors:
            d = s.as_dict()
            d["output"] = s.output.value if s.output else None
            rows.append(d)
        df = pd.DataFrame(rows)
        cols = ["name", "measures", "unit", "why", "location", "output",
                "supply_v", "current_ma", "supply_rail", "connector",
                "conductors", "signal_bandwidth_hz", "sample_rate_hz",
                "antialias_cutoff_hz", "adc_bits", "range_min_eu",
                "range_max_eu", "accuracy_eu", "resolution_needed_eu",
                "logged_to", "payload_bytes", "calibration",
                "galvanic_isolation", "owner", "source", "is_estimate"]
        df = df[[c for c in cols if c in df.columns]]

        edited = st.data_editor(
            df, use_container_width=True, hide_index=True, num_rows="fixed",
            key="daq_editor",
            column_config={
                "output": st.column_config.SelectboxColumn(
                    "output", options=sorted(_OUTPUT_LABELS.keys())),
                "location": st.column_config.SelectboxColumn(
                    "location", options=sorted(dp.LOCATION_SUBTEAMS.keys())),
                "galvanic_isolation": st.column_config.CheckboxColumn(
                    "isolated"),
                "is_estimate": st.column_config.CheckboxColumn("estimate"),
            })

        if st.button("Apply edits", key="daq_apply", type="primary"):
            new = []
            for i, s in enumerate(sensors):
                try:
                    new.append(_spec_from_row(edited.iloc[i].to_dict(), s))
                except Exception:
                    new.append(s)          # a bad row must never lose the rest
            st.session_state["daq_sensors"] = new
            st.rerun()
    except Exception as e:
        st.caption(f"(editor unavailable: {e})")

    # ---- remove ------------------------------------------------------------ #
    with st.expander("Remove a channel", expanded=False):
        rm = st.selectbox("Channel", [s.name for s in sensors], key="daq_rm_pick")
        if st.button("Remove", key="daq_rm"):
            st.session_state["daq_sensors"] = [s for s in sensors if s.name != rm]
            st.rerun()

    # ---- who this lands on -------------------------------------------------- #
    st.markdown("#### Who else this lands on")
    st.caption("Derived from where each sensor mounts — the answer to "
               "\"what other subteam does this affect?\", computed rather than "
               "remembered.")
    routed: dict[str, list[str]] = {}
    for s in sensors:
        for t in s.affected_subteams():
            routed.setdefault(t, []).append(s.name)
    for t in sorted(routed):
        st.markdown(f"* **{t}** — {', '.join(routed[t])}")


# --------------------------------------------------------------------------- #
#  2. budgets
# --------------------------------------------------------------------------- #
def _render_budgets():
    ss = st.session_state
    sensors = _sensors()

    st.markdown("#### The bus, the rails and the card")

    c1, c2, c3 = st.columns(3)
    with c1:
        bitrate = st.selectbox("CAN bitrate (kbit/s)", [125, 250, 500, 1000],
                               index=2, key="daq_bitrate")
        ext = st.checkbox("29-bit (extended) identifiers", value=False,
                          key="daq_ext")
    with c2:
        cap5 = st.number_input("5V rail capacity (mA)", value=500.0, step=50.0,
                               key="daq_cap5")
        cap12 = st.number_input("12V rail capacity (mA)", value=2000.0,
                                step=100.0, key="daq_cap12")
    with c3:
        card = st.number_input("Logger storage (MB)", value=8192.0, step=512.0,
                               key="daq_card")
        session = st.number_input("Session length (min)", value=25.0, step=5.0,
                                  key="daq_session")

    bus = dp.BusSpec(bitrate_bps=bitrate * 1000.0, extended_ids=ext)
    rails = {"5V": dp.Rail("5V", 5.0, cap5), "12V": dp.Rail("12V", 12.0, cap12)}
    logger = dp.LoggerSpec(storage_mb=card, session_minutes=session)

    p = dp.plan(sensors, bus=bus, rails=rails, logger=logger,
                bridge=_bridge())
    ss["daq_plan"] = p

    # ---- verdict banner --------------------------------------------------- #
    if p.verdict == dp.Verdict.BLOCKED:
        st.error(f"### ❌ BLOCKED — {len(p.blocking())} hard failure(s)\n\n"
                 f"Something in this plan does not work. The budgets below are "
                 f"real, but they describe a channel list that cannot be built "
                 f"as specified.")
    elif p.verdict == dp.Verdict.INCOMPLETE:
        st.warning(
            f"### ⭕ INCOMPLETE — documentation {p.completeness*100:.0f}% "
            f"complete\n\nNo hard failures, but questions are still open. "
            f"Every budget below is a **floor**: an unspecified channel "
            f"contributes nothing to these numbers and a positive amount to "
            f"the real car.")
    else:
        st.success("### ✅ READY — every question answered, every budget clears")

    # ---- bus --------------------------------------------------------------- #
    if p.bus_result is not None:
        br = p.bus_result
        st.markdown("#### CAN bus")
        m1, m2, m3 = st.columns(3)
        m1.metric("Worst-case load", f"{br.load*100:.1f}%",
                  help="Includes worst-case bit stuffing — the case that drops "
                       "your frames, not the case an oscilloscope shows.")
        m2.metric("Without stuffing", f"{br.load_unstuffed*100:.1f}%")
        m3.metric("Messages", f"{br.messages}")

        st.progress(min(br.load, 1.0))
        if br.is_floor:
            st.caption("⭕ This is a FLOOR — channels with no declared sample "
                       "rate contribute nothing to it.")

        with st.expander("Per-message load and worst-case latency",
                         expanded=False):
            try:
                import pandas as pd
                rows = []
                for name, d in sorted(br.per_message.items(),
                                      key=lambda kv: kv[1]["can_id"]):
                    lat = br.latencies.get(name, float("nan"))
                    rows.append({
                        "Message": name,
                        "ID": f"0x{d['can_id']:03X}",
                        "DLC": d["dlc"],
                        "Rate (Hz)": d["rate_hz"],
                        "Bits": d["bits"],
                        "Load %": round(d["load"] * 100, 3),
                        "Worst-case latency (ms)":
                            ("∞" if lat != lat or lat == float("inf")
                             else round(lat * 1000, 3)),
                    })
                st.dataframe(pd.DataFrame(rows), use_container_width=True,
                             hide_index=True)
            except Exception as e:
                st.caption(f"(table unavailable: {e})")
            st.caption(
                "Latency is the fixed-point response-time bound for "
                "non-preemptive priority arbitration: one blocking frame plus "
                "every higher-priority frame that can arrive while this one "
                "waits. It assumes strictly periodic transmission and no "
                "queueing jitter, so treat it as a lower bound — a controller "
                "with a FIFO transmit buffer does worse.")

    # ---- power -------------------------------------------------------------- #
    if p.power is not None:
        st.markdown("#### Rails")
        cols = st.columns(len(p.power.per_rail) or 1)
        for col, (name, d) in zip(cols, p.power.per_rail.items()):
            with col:
                frac = d["frac"]
                st.metric(f"{name} rail", f"{d['draw_ma']:.0f} mA",
                          f"{frac*100:.0f}% of {d['capacity_ma']:.0f} mA")
                st.progress(min(frac, 1.0) if frac == frac else 0.0)
        if p.power.is_floor:
            st.caption("⭕ Floor — sensors with no declared current draw are "
                       "not in these totals.")

    # ---- storage -------------------------------------------------------------- #
    if p.storage is not None:
        st.markdown("#### Logger")
        s1, s2 = st.columns(2)
        s1.metric("Write rate", f"{p.storage.bytes_per_s/1000:.1f} kB/s")
        s2.metric("Per session", f"{p.storage.session_mb:.0f} MB",
                  f"{p.storage.frac*100:.0f}% of the card")

    _render_findings(p)


def _render_findings(p):
    st.markdown("#### Findings")
    by_sev = {s: [] for s in _SEV_ORDER}
    for f in p.findings:
        by_sev.setdefault(f.severity, []).append(f)

    counts = " · ".join(f"{_SEV_ICON[s]} {len(by_sev.get(s, []))}"
                        for s in _SEV_ORDER)
    st.caption(counts)

    for sev in _SEV_ORDER:
        items = by_sev.get(sev, [])
        if not items:
            continue
        opened = sev in (Severity.FAIL, Severity.MISSING)
        with st.expander(f"{_SEV_ICON[sev]} {sev.value} ({len(items)})",
                         expanded=opened):
            for f in items:
                st.markdown(f"**{f.check}** "
                            f"({', '.join(f.subsystems) or '—'}) — {f.message}")

    acts = p.subteam_actions()
    if acts:
        with st.expander("Routed by subteam — what each team has to do",
                         expanded=False):
            for team in sorted(acts):
                st.markdown(f"**{team}** ({len(acts[team])})")
                for f in acts[team]:
                    st.markdown(f"* {_SEV_ICON.get(f.severity, '•')} {f.message}")


# --------------------------------------------------------------------------- #
#  3. BMS bridge
# --------------------------------------------------------------------------- #
def _render_bridge():
    ss = st.session_state
    st.markdown("#### BMS → CAN bridge")
    st.caption(
        "The BMS speaks a serial protocol and the logger listens on CAN. This "
        "sizes the link, packs the signals into frames, and checks the "
        "isolation boundary the accumulator puts in the middle."
    )

    st.warning(
        "**This tool will not design a bridge from an empty signal list.** "
        "The serial frame layout sets the CAN frame layout, the update rate "
        "sets the bus load, and the value widths set the scaling — all of it "
        "comes off the datasheet. A frame map produced before anyone reads it "
        "would be invented, and an invented map is worse than a blank page "
        "because it stops the next person opening the datasheet.",
        icon="📄")

    c1, c2, c3 = st.columns(3)
    with c1:
        baud = st.selectbox("Baud", [9600, 19200, 38400, 57600, 115200, 230400],
                            index=4, key="daq_baud")
        parity = st.selectbox("Parity", ["none", "even", "odd"], key="daq_par")
    with c2:
        fbytes = st.number_input("BMS frame size (bytes)", value=64, step=1,
                                 min_value=1, key="daq_fbytes")
        frate = st.number_input("BMS frame rate (Hz)", value=10.0, step=1.0,
                                min_value=0.1, key="daq_frate")
    with c3:
        base_id = st.text_input("Base CAN ID (hex)", value="300",
                                key="daq_baseid")
        iso = st.selectbox("Galvanic isolation on the serial link",
                           ["not declared", "yes", "no"], key="daq_iso")

    link = dp.UartLink(baud=baud, parity=(None if parity == "none" else parity),
                       frame_bytes=int(fbytes), frame_rate_hz=float(frate))

    st.caption(f"Framing: {link.bits_per_byte()} bits per byte "
               f"({'8' }{'N' if parity == 'none' else parity[0].upper()}1) → "
               f"one {int(fbytes)}-byte frame takes "
               f"{link.frame_time_s()*1000:.2f} ms on the wire.")

    st.markdown("##### Signals from the datasheet")
    st.caption("Name, width in bits, update rate, and whether it is "
               "shutdown-relevant. Critical signals are given the identifiers "
               "that win arbitration.")

    if "daq_bms_signals" not in ss:
        ss["daq_bms_signals"] = []

    try:
        import pandas as pd
        if ss["daq_bms_signals"]:
            base = pd.DataFrame([{
                "name": s.name, "bits": s.bits, "unit": s.unit,
                "scale": s.scale, "offset": s.offset,
                "rate_hz": s.rate_hz, "critical": s.critical}
                for s in ss["daq_bms_signals"]])
        else:
            base = pd.DataFrame(columns=["name", "bits", "unit", "scale",
                                         "offset", "rate_hz", "critical"])
        edited = st.data_editor(base, num_rows="dynamic",
                                use_container_width=True, hide_index=True,
                                key="daq_sig_editor")
        if st.button("Save signal list", key="daq_sig_save", type="primary"):
            sigs = []
            for _, r in edited.iterrows():
                nm = str(r.get("name") or "").strip()
                if not nm:
                    continue
                try:
                    bits = int(r.get("bits") or 0)
                except (TypeError, ValueError):
                    continue
                if bits <= 0:
                    continue
                rate = r.get("rate_hz")
                sigs.append(dp.BmsSignal(
                    name=nm, bits=bits, unit=str(r.get("unit") or ""),
                    scale=float(r.get("scale") or 1.0),
                    offset=float(r.get("offset") or 0.0),
                    rate_hz=(None if rate in (None, "") or rate != rate
                             else float(rate)),
                    critical=bool(r.get("critical", False))))
            ss["daq_bms_signals"] = sigs
            st.rerun()
    except Exception as e:
        st.caption(f"(signal editor unavailable: {e})")

    try:
        bid = int(base_id, 16)
    except ValueError:
        bid = 0x300
        st.caption("(unparseable base ID — using 0x300)")

    bitrate = st.session_state.get("daq_bitrate", 500)
    bridge = dp.plan_bms_bridge(
        link, ss.get("daq_bms_signals", []), base_id=bid,
        bus=dp.BusSpec(bitrate_bps=bitrate * 1000.0),
        isolated=(None if iso == "not declared" else iso == "yes"))
    ss["daq_bridge"] = bridge

    if bridge.refused:
        st.error("### ⭕ No bridge designed\n\n" + bridge.refusal_reason
                 + " — read the datasheet and list the signals above.")
    else:
        st.success(f"### ✅ {len(bridge.signals)} signals → "
                   f"{len(bridge.messages)} CAN frames")
        try:
            import pandas as pd
            st.dataframe(pd.DataFrame([{
                "CAN ID": f"0x{m.can_id:03X}", "DLC": m.dlc,
                "Rate (Hz)": m.rate_hz,
                "Signals": ", ".join(m.signals)} for m in bridge.messages]),
                use_container_width=True, hide_index=True)
        except Exception as e:
            st.caption(f"(frame table unavailable: {e})")

    for sev in _SEV_ORDER:
        for f in bridge.findings:
            if f.severity == sev:
                st.markdown(f"{_SEV_ICON[sev]} **{f.check}** — {f.message}")


# --------------------------------------------------------------------------- #
#  4. cooling delta-T
# --------------------------------------------------------------------------- #
def _render_delta_t():
    sensors = _sensors()
    st.markdown("#### Coolant ΔT — can this pair measure what it is for?")
    st.caption(
        "An inlet and an outlet probe are not two measurements, they are one "
        "difference. A difference inherits the error of both ends while "
        "keeping a fraction of the magnitude, which is why a pair chosen on "
        "price often cannot resolve the rise it was bought to measure."
    )

    pair = dp.find_coolant_pair(sensors)
    if pair is None:
        st.info("No inlet/outlet coolant temperature pair in the channel list. "
                "Add both from the catalog on the Channels tab to run this "
                "check.")
        return

    st.caption(f"Pair detected: **{pair[0].name}** and **{pair[1].name}**")

    c1, c2, c3 = st.columns(3)
    with c1:
        dt = st.number_input("Expected ΔT across the radiator (K)", value=6.0,
                             step=0.5, min_value=0.1, key="daq_dt")
    with c2:
        flow = st.number_input("Coolant flow (L/min)", value=12.0, step=0.5,
                               min_value=0.0, key="daq_flow")
    with c3:
        matched = st.checkbox("Calibrated as a matched pair", value=False,
                              key="daq_matched",
                              help="Both probes calibrated together against "
                                   "one reference, which cancels the shared "
                                   "offset that dominates a small difference.")

    r = dp.delta_t_budget(pair[0], pair[1], expected_delta_t_k=dt,
                          flow_lpm=(flow if flow > 0 else None),
                          matched_pair=matched)

    m1, m2 = st.columns(2)
    m1.metric("ΔT uncertainty", f"±{r.sigma_delta_t_k:.2f} K",
              f"{r.relative_error*100:.0f}% of a {dt:g} K rise",
              delta_color="inverse")
    if r.heat_kw is not None:
        m2.metric("Implied heat rejection",
                  f"{r.heat_kw:.1f} ± {r.sigma_heat_kw:.1f} kW",
                  f"±{r.heat_relative_error*100:.0f}%", delta_color="off")

    for f in r.findings:
        st.markdown(f"{_SEV_ICON.get(f.severity, '•')} **{f.check}** — "
                    f"{f.message}")

    if not matched:
        st.caption(
            "Try the matched-pair checkbox. On the same hardware it is usually "
            "the difference between a channel pair you can size a radiator "
            "from and one you cannot — and it costs an afternoon with a "
            "stirred water bath, not a new purchase order.")


# --------------------------------------------------------------------------- #
#  5. export
# --------------------------------------------------------------------------- #
def _render_export():
    p = st.session_state.get("daq_plan")
    if p is None:
        st.info("Open the **Bus & budgets** tab once to build the plan, then "
                "come back here.")
        return

    st.markdown("#### The documentation table, generated")
    st.caption(
        "This is the review table, produced from the plan rather than "
        "maintained beside it. A hand-written one starts drifting the moment "
        "either changes; this one cannot, because it is the same object the "
        "budgets were computed from. Unanswered questions render as a dash, "
        "so a blank looks blank."
    )

    md = p.to_markdown()
    st.download_button("Download the review table (Markdown)", md,
                       file_name="daq_channel_plan.md", mime="text/markdown",
                       key="daq_dl_md")
    st.download_button("Download the channel list (CSV)", p.to_csv(),
                       file_name="daq_channels.csv", mime="text/csv",
                       key="daq_dl_csv")

    with st.expander("Preview", expanded=True):
        st.markdown(md)
