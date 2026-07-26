# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Tests for the EV accumulator / tractive-system section of the manufacturing-
release gate (release_gate._ev_checks).

Contract under test:
  * section is inert on a non-EV car (ev_present=False) — adds zero checks;
  * a green EV car (electrical + thermal envelopes clear, with margin) releases;
  * every EV check vetoes on its own single defect (fuse blown, near-fuse peak,
    cell overcurrent, energy brown-out, over-temp, cell-over-limit);
  * absent evidence FAILS — a pack is never released on a check nobody ran;
  * a non-converged thermal run (NaN peak) FAILS rather than sneaking through;
  * the gate consumes BOTH real duck-typed solver result objects AND the
    precomputed scalar overrides, and agrees between them;
  * a green EV gate's printed clipboard carries the accumulator inspection
    section; a red one still refuses to print at all.

Run: python tests/test_release_gate_ev.py   (or under pytest)
"""

import importlib
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def _load(name):
    return importlib.import_module(f"suspension.{name}")


BJ = _load("bolted_joint")
RE = _load("risk_engine")
RG = _load("release_gate")

_PASS, _FAIL = [], []


def check(name, cond):
    (_PASS if cond else _FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")


# --- lightweight duck-typed stand-ins for the real solver results ------------ #
# The gate only reads named attributes; we mirror exactly the fields
# ElecCheckResult / PackThermalResult expose, so a passing test here means the
# gate wires to the real objects without importing the heavy solver modules.
class FakeElec:
    def __init__(self, fuse_blown=False, cell_overcurrent=False, energy_empty=False,
                 peak_current_a=180.0, fuse_max_a=250.0):
        self.fuse_blown = fuse_blown
        self.cell_overcurrent = cell_overcurrent
        self.energy_empty = energy_empty
        self.peak_current_a = peak_current_a
        self.fuse_max_a = fuse_max_a


class FakeThermal:
    def __init__(self, hottest_peak_c=48.0, breach_count=0):
        self.hottest_peak_c = hottest_peak_c
        self.breach_count = breach_count
        self.any_cell_breached_limit = breach_count > 0


slotted = RE.analyze_slotted_joint(
    RE.SlottedHoleJoint(fastener=BJ.Fastener(grade="10.9", nominal_d_mm=6.0),
                        slot_width_mm=6.6, slot_length_mm=20.0, washer_od_mm=18.0),
    assembly_torque_Nm=16.0)


def green_ev_inputs(**over):
    """A fully green EV car: every non-EV section clear (reusing the shape the
    core gate test proves), plus a clear electrical + thermal envelope."""
    base = dict(
        chassis_quads=[], chassis_loadpath_findings=[],
        manifold_dp_kpa={"radiator_in": 14.0, "motor_branch": 22.0},
        pump_head_kpa=55.0, inlet_margin_kpa=38.0,
        brake_fos={"caliper bracket": 2.1, "rotor thermal": 1.9},
        pedal_joint=slotted,
        pedal_slip_demand_N=0.35 * slotted.F_clamp_at_torque_N / 1.5,
        torque_specs=[
            RG.TorqueSpec("caliper bracket bolts", "brakes/FL", 8.0, "10.9",
                          spec_torque_Nm=26.0, K=0.20, qty=2),
            RG.TorqueSpec("pedal tab (slotted)", "brakes/pedalbox", 6.0, "10.9",
                          spec_torque_Nm=16.0, slotted=slotted, qty=2),
            RG.TorqueSpec("motor mount", "powertrain/mount", 10.0, "12.9",
                          spec_torque_Nm=52.0, K=0.18, qty=4),
        ],
        required_fastener_locations=["brakes/FL", "brakes/pedalbox",
                                     "powertrain/mount"],
        ev_present=True,
        elec_result=FakeElec(),
        pack_thermal_result=FakeThermal(),
        team="Elbee Racing", car="EB-26E", event="FSAE Electric")
    base.update(over)
    return RG.GateInputs(**base)


# --- section is inert on a non-EV car ---------------------------------------- #
non_ev = green_ev_inputs(ev_present=False, elec_result=None,
                         pack_thermal_result=None)
rep_non_ev = RG.run_gate(non_ev)
check("non-EV car adds zero EV checks",
      not any(c.check_id.startswith("EV-") for c in rep_non_ev.checks))
check("non-EV green car still releases", rep_non_ev.released)

# --- green EV releases, deterministically ------------------------------------ #
rep = RG.run_gate(green_ev_inputs())
check("fully green EV releases", rep.released)
check("EV section contributes all four checks",
      len([c for c in rep.checks if c.check_id.startswith("EV-")]) == 4)
check("EV verdict deterministic",
      RG.run_gate(green_ev_inputs()).as_dict() == rep.as_dict())
check("all check ids unique on an EV car",
      len({c.check_id for c in rep.checks}) == len(rep.checks))


def mutate(**over):
    return RG.run_gate(green_ev_inputs(**over))


# --- IFF: any single EV defect vetoes ---------------------------------------- #
check("blown fuse vetoes",
      not mutate(elec_result=FakeElec(fuse_blown=True)).released)
check("peak current inside the margin vetoes",
      not mutate(elec_result=FakeElec(peak_current_a=245.0, fuse_max_a=250.0)).released)
check("cell overcurrent vetoes",
      not mutate(elec_result=FakeElec(cell_overcurrent=True)).released)
check("energy brown-out vetoes",
      not mutate(elec_result=FakeElec(energy_empty=True)).released)
check("over-temperature cell vetoes",
      not mutate(pack_thermal_result=FakeThermal(hottest_peak_c=58.0)).released)
check("thin thermal margin vetoes (peak between limit-margin and limit)",
      not mutate(pack_thermal_result=FakeThermal(hottest_peak_c=57.0)).released)
check("any cell over limit vetoes even if hottest scalar looks ok",
      not mutate(pack_thermal_result=FakeThermal(hottest_peak_c=48.0,
                                                 breach_count=2)).released)

# --- missing evidence fails (never silently passes) -------------------------- #
no_elec = RG.run_gate(green_ev_inputs(elec_result=None))
check("EV car with no electrical result does NOT release", not no_elec.released)
check("missing electrical check is an EV-01 failure",
      any(c.check_id == "EV-01" and not c.passed for c in no_elec.checks))

no_therm = RG.run_gate(green_ev_inputs(pack_thermal_result=None))
check("EV car with no thermal result does NOT release", not no_therm.released)
check("missing thermal run is an EV-04 failure",
      any(c.check_id == "EV-04" and not c.passed for c in no_therm.checks))

# --- a non-converged thermal run (NaN peak) fails, does not sneak through ----- #
check("NaN thermal peak vetoes",
      not mutate(pack_thermal_result=FakeThermal(
          hottest_peak_c=float("nan"))).released)

# --- real duck-typed objects and scalar overrides agree ---------------------- #
via_objects = RG.run_gate(green_ev_inputs())
via_scalars = RG.run_gate(green_ev_inputs(
    elec_result=None, pack_thermal_result=None,
    fuse_blown=False, cell_overcurrent=False, energy_empty=False,
    peak_current_a=180.0, fuse_max_a=250.0,
    pack_peak_cell_c=48.0, pack_cells_over_limit=0))
check("scalar-override path also releases the green EV car", via_scalars.released)
check("object path and scalar path agree on the EV verdicts",
      [(c.check_id, c.passed) for c in via_objects.checks if c.check_id.startswith("EV-")]
      == [(c.check_id, c.passed) for c in via_scalars.checks if c.check_id.startswith("EV-")])

# --- clipboard carries the accumulator section on green, refuses on red ------- #
clip = RG.build_clipboard(rep, green_ev_inputs())
titles = [t for t, _ in clip.sections]
check("green EV clipboard includes the accumulator inspection section",
      any("Accumulator" in t for t in titles))

red = RG.run_gate(green_ev_inputs(elec_result=FakeElec(fuse_blown=True)))
try:
    RG.build_clipboard(red, green_ev_inputs(elec_result=FakeElec(fuse_blown=True)))
    check("red EV gate refuses to print a checklist", False)
except RG.GateNotPassed:
    check("red EV gate refuses to print a checklist", True)

# --- digest reflects the EV section ------------------------------------------ #
check("green EV digest marks the accumulator audited",
      rep.inputs_digest.get("ev_accumulator") == "audited")
check("non-EV digest marks the accumulator n/a",
      rep_non_ev.inputs_digest.get("ev_accumulator") == "n/a")
check("EV car missing evidence is digest 'incomplete'",
      RG.run_gate(green_ev_inputs(
          elec_result=None, pack_thermal_result=None
      )).inputs_digest.get("ev_accumulator") == "incomplete")


# ---------------------------------------------------------------------------- #
print(f"\n{len(_PASS)} passed, {len(_FAIL)} failed")
if _FAIL:
    print("FAILURES:")
    for f in _FAIL:
        print("  -", f)
    sys.exit(1)


def test_release_gate_ev():
    """pytest entry point — re-runs the module body's assertions as one test."""
    assert not _FAIL, f"failures: {_FAIL}"
